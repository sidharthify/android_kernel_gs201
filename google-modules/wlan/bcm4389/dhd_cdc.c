/*
 * DHD Protocol Module for CDC and BDC.
 *
 * Copyright (C) 2025, Broadcom.
 *
 *      Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2 (the "GPL"),
 * available at http://www.broadcom.com/licenses/GPLv2.php, with the
 * following added to such license:
 *
 *      As a special exception, the copyright holders of this software give you
 * permission to link this software with independent modules, and to copy and
 * distribute the resulting executable under terms of your choice, provided that
 * you also meet, for each linked independent module, the terms and conditions of
 * the license of that module.  An independent module is a module which is not
 * derived from this software.  The special exception does not apply to any
 * modifications of the software.
 *
 *
 * <<Broadcom-WL-IPTag/Open:>>
 *
 * $Id$
 *
 * BDC is like CDC, except it includes a header for data packets to convey
 * packet priority over the bus, and flags (e.g. to indicate checksum status
 * for dongle offload.)
 */

#include <typedefs.h>
#include <osl.h>

#include <bcmutils.h>
#include <bcmcdc.h>
#include <bcmendian.h>

#include <dngl_stats.h>
#include <dhd.h>
#include <dhd_proto.h>
#ifdef BCMDBUS
#include <dbus.h>
#else
#include <dhd_bus.h>
#endif /* BCMDBUS */
#include <dhd_dbg.h>

#ifdef EXT_STA
#include <siutils.h>
#include <wlc_cfg.h>
#include <wlc_pub.h>
#endif /* EXT_STA */

#ifdef PROP_TXSTATUS
#include <wlfc_proto.h>
#include <dhd_wlfc.h>
#endif

#define RETRIES 2		/* # of retries to retrieve matching ioctl response */
#define BUS_HEADER_LEN	(24+DHD_SDALIGN)	/* Must be at least SDPCM_RESERVE
				 * defined in dhd_sdio.c (amount of header tha might be added)
				 * plus any space that might be needed for alignment padding.
				 */
#define ROUND_UP_MARGIN	2048	/* Biggest SDIO block size possible for
				 * round off at the end of buffer
				 */

/* This value is from Legacy chipsets */
#define DEFAULT_WLC_API_VERSION_MAJOR	3
#define DEFAULT_WLC_API_VERSION_MINOR	0

typedef struct dhd_prot {
	uint16 reqid;
	uint8 pending;
	uint32 lastcmd;
#ifdef BCMDBUS
	uint ctl_completed;
#endif
	uint8 bus_header[BUS_HEADER_LEN];
	cdc_ioctl_t msg;
	unsigned char buf[WLC_IOCTL_MAXLEN + ROUND_UP_MARGIN];
} dhd_prot_t;

uint16
dhd_prot_get_ioctl_trans_id(dhd_pub_t *dhdp)
{
	/* SDIO does not have ioctl_trans_id yet, so return -1 */
	return -1;
}

#if defined(BCMDBUS)
extern int dhd_dbus_txdata(dhd_pub_t *dhdp, void *pktbuf);
#endif

static int
dhdcdc_msg(dhd_pub_t *dhd)
{
#ifdef BCMDBUS
	int timeout = 0;
#endif /* BCMDBUS */
	int err = 0;
	dhd_prot_t *prot = dhd->prot;
	int len = ltoh32(prot->msg.len) + sizeof(cdc_ioctl_t);

	DHD_TRACE(("%s: Enter\n", __FUNCTION__));

	DHD_OS_WAKE_LOCK(dhd);

	/* NOTE : cdc->msg.len holds the desired length of the buffer to be
	 *        returned. Only up to CDC_MAX_MSG_SIZE of this buffer area
	 *	  is actually sent to the dongle
	 */
	if (len > CDC_MAX_MSG_SIZE)
		len = CDC_MAX_MSG_SIZE;

	/* Send request */
#ifdef BCMDBUS
	prot->ctl_completed = FALSE;
	err = dbus_send_ctl(dhd->dbus, (void *)&prot->msg, len);
	if (err) {
		DHD_ERROR(("dbus_send_ctl error=0x%x\n", err));
		DHD_OS_WAKE_UNLOCK(dhd);
		return err;
	}
#else
	err = dhd_bus_txctl(dhd->bus, (uchar*)&prot->msg, len);
#endif

#ifdef BCMDBUS
	timeout = dhd_os_ioctl_resp_wait(dhd, &prot->ctl_completed);
	if ((!timeout) || (!prot->ctl_completed)) {
		DHD_ERROR(("Txctl timeout %d ctl_completed %d\n",
			timeout, prot->ctl_completed));
		DHD_ERROR(("Txctl wait timed out\n"));
		err = -1;
	}
#endif
#if defined(BCMDBUS) && defined(INTR_EP_ENABLE)
	/* If the ctl write is successfully completed, wait for an acknowledgement
	* that indicates that it is now ok to do ctl read from the dongle
	*/
	if (err != -1) {
		prot->ctl_completed = FALSE;
		if (dbus_poll_intr(dhd->dbus)) {
			DHD_ERROR(("dbus_poll_intr not submitted\n"));
		} else {
			/* interrupt polling is sucessfully submitted. Wait for dongle to send
			* interrupt
			*/
			timeout = dhd_os_ioctl_resp_wait(dhd, &prot->ctl_completed);
			if (!timeout) {
				DHD_ERROR(("intr poll wait timed out\n"));
			}
		}
	}
#endif /* defined(BCMDBUS) && defined(INTR_EP_ENABLE) */
	DHD_OS_WAKE_UNLOCK(dhd);
	return err;
}

static int
dhdcdc_cmplt(dhd_pub_t *dhd, uint32 id, uint32 len)
{
#ifdef BCMDBUS
	int timeout = 0;
#endif /* BCMDBUS */
	int ret;
	int cdc_len = len + sizeof(cdc_ioctl_t);
	dhd_prot_t *prot = dhd->prot;

	DHD_TRACE(("%s: Enter\n", __FUNCTION__));
	/*
	 * prot->msg is the buffer used to send the ioctl msg in dhdcdc_msg and the same is
	 * used for receiving the ioctl response.
	 * As this buffer is not cleared after sending message and before re-using for receive
	 * operation, problems are seen when bus errors occur.
	 * Example:- An error is seen on the bus where a 0 byte pkt is received.
	 * the bus-controller "layer below cdc" does not update the buffer in this 0 byte pkt case.
	 * bus-controller layer calls the completion callback without any error and with the
	 * same buffer.(no change)
	 * This issue is seen on DBUS/USB + FPGA platforms
	 * In this function if there is no update to the buffer, the stale id field of sent msg
	 * matches to expected ID and further processing is done thinking that
	 * proper response is received.
	 * This is a generic problem and its a good idea to clear the buffer or atleast
	 * the buffer's key value (ioctl ID within flags field in this case) before re-using it.
	 *
	 * To ensure that a new content is indeed received from the bus making flags = 0.
	 * Note that ID=0 is invalid value.
	 */
	prot->msg.flags = 0;

	do {
#ifdef BCMDBUS
		prot->ctl_completed = FALSE;
		ret = dbus_recv_ctl(dhd->dbus, (uchar*)&prot->msg, cdc_len);
		if (ret) {
			DHD_ERROR(("dbus_recv_ctl error=0x%x(%d)\n", ret, ret));
			goto done;
		}
		timeout = dhd_os_ioctl_resp_wait(dhd, &prot->ctl_completed);
		if ((!timeout) || (!prot->ctl_completed)) {
			DHD_ERROR(("Rxctl timeout %d ctl_completed %d\n",
				timeout, prot->ctl_completed));
			ret = -1;

			goto done;
		}

		/* XXX FIX: Must return cdc_len, not len, because after query_ioctl()
		 * it subtracts sizeof(cdc_ioctl_t);  The other approach is
		 * to have dbus_recv_ctl() return actual len.
		 */
		ret = cdc_len;
#else
		ret = dhd_bus_rxctl(dhd->bus, (uchar*)&prot->msg, cdc_len);
#endif /* BCMDBUS */
		if (ret < 0)
			break;
	} while (CDC_IOC_ID(ltoh32(prot->msg.flags)) != id);

	/* update ret to len on success */
	if (ret == cdc_len) {
		ret = len;
	}

#ifdef BCMDBUS
done:
#endif /* BCMDBUS */
	return ret;
}

/* XXX: due to overlays this should not be called directly; call dhd_wl_ioctl_cmd() instead */
static int
dhdcdc_query_ioctl(dhd_pub_t *dhd, int ifidx, uint cmd, void *buf, uint len, uint8 action)
{
	dhd_prot_t *prot = dhd->prot;
	cdc_ioctl_t *msg = &prot->msg;
	int ret = 0, retries = 0;
	uint32 id, flags = 0;

	DHD_TRACE(("%s: Enter\n", __FUNCTION__));
	DHD_CTL(("%s: cmd %d len %d\n", __FUNCTION__, cmd, len));

	/* Respond "bcmerror" and "bcmerrorstr" with local cache */
	if (cmd == WLC_GET_VAR && buf)
	{
		if (!strcmp((char *)buf, "bcmerrorstr"))
		{
			strlcpy((char *)buf, bcmerrorstr(dhd->dongle_error), len);
			goto done;
		}
		else if (!strcmp((char *)buf, "bcmerror"))
		{
			*(int *)buf = dhd->dongle_error;
			goto done;
		}
	}

	memset(msg, 0, sizeof(cdc_ioctl_t));

#ifdef BCMSPI
	/* 11bit gSPI bus allows 2048bytes of max-data.  We restrict 'len'
	 * value which is 8Kbytes for various 'get' commands to 2000.  48 bytes are
	 * left for sw headers and misc.
	 */
	if (len > 2000) {
		DHD_ERROR(("dhdcdc_query_ioctl: len is truncated to 2000 bytes\n"));
		len = 2000;
	}
#endif /* BCMSPI */
	msg->cmd = htol32(cmd);
	msg->len = htol32(len);
	msg->flags = (++prot->reqid << CDCF_IOC_ID_SHIFT);
	CDC_SET_IF_IDX(msg, ifidx);
	/* add additional action bits */
	action &= WL_IOCTL_ACTION_MASK;
	msg->flags |= (action << CDCF_IOC_ACTION_SHIFT);
	msg->flags = htol32(msg->flags);

	if (buf)
		memcpy(prot->buf, buf, len);

	if ((ret = dhdcdc_msg(dhd)) < 0) {
		if (!dhd->hang_was_sent)
		DHD_ERROR(("dhdcdc_query_ioctl: dhdcdc_msg failed w/status %d\n", ret));
		goto done;
	}

retry:
	/* wait for interrupt and get first fragment */
	if ((ret = dhdcdc_cmplt(dhd, prot->reqid, len)) < 0)
		goto done;

	flags = ltoh32(msg->flags);
	id = (flags & CDCF_IOC_ID_MASK) >> CDCF_IOC_ID_SHIFT;

	if ((id < prot->reqid) && (++retries < RETRIES))
		goto retry;
	if (id != prot->reqid) {
		DHD_ERROR(("%s: %s: unexpected request id %d (expected %d)\n",
		           dhd_ifname(dhd, ifidx), __FUNCTION__, id, prot->reqid));
		ret = -EINVAL;
		goto done;
	}

	/* Copy info buffer */
	if (buf)
	{
		if (ret < (int)len)
			len = ret;
		memcpy(buf, (void*) prot->buf, len);
	}

	/* Check the ERROR flag */
	if (flags & CDCF_IOC_ERROR)
	{
		ret = ltoh32(msg->status);
		/* Cache error from dongle */
		dhd->dongle_error = ret;
	}

done:
	return ret;
}

#ifdef DHD_PM_CONTROL_FROM_FILE
extern bool g_pm_control;
#endif /* DHD_PM_CONTROL_FROM_FILE */

/* XXX: due to overlays this should not be called directly; call dhd_wl_ioctl_cmd() instead */
static int
dhdcdc_set_ioctl(dhd_pub_t *dhd, int ifidx, uint cmd, void *buf, uint len, uint8 action)
{
	dhd_prot_t *prot = dhd->prot;
	cdc_ioctl_t *msg = &prot->msg;
	int ret = 0;
	uint32 flags, id;

	DHD_TRACE(("%s: Enter\n", __FUNCTION__));
	DHD_CTL(("%s: cmd %d len %d\n", __FUNCTION__, cmd, len));

	if (dhd->busstate == DHD_BUS_DOWN) {
		DHD_ERROR(("%s : bus is down. we have nothing to do\n", __FUNCTION__));
		return -EIO;
	}

	/* don't talk to the dongle if fw is about to be reloaded */
	if (dhd->hang_was_sent) {
		DHD_ERROR(("%s: HANG was sent up earlier. Not talking to the chip\n",
			__FUNCTION__));
		return -EIO;
	}

	if (cmd == WLC_SET_PM) {
#ifdef DHD_PM_CONTROL_FROM_FILE
		if (g_pm_control == TRUE) {
			DHD_ERROR(("%s: SET PM ignored!(Requested:%d)\n",
				__FUNCTION__, buf ? *(char *)buf : 0));
			goto done;
		}
#endif /* DHD_PM_CONTROL_FROM_FILE */
#ifdef DHD_PM_OVERRIDE
		{
			extern bool g_pm_override;
			if (g_pm_override == TRUE) {
				DHD_ERROR(("%s: PM override SET PM ignored!(Requested:%d)\n",
					__FUNCTION__, buf ? *(char *)buf : 0));
				goto done;
			}
		}
#endif /* DHD_PM_OVERRIDE */
		DHD_TRACE_HW4(("%s: SET PM to %d\n", __FUNCTION__, buf ? *(char *)buf : 0));
	}

	memset(msg, 0, sizeof(cdc_ioctl_t));

	msg->cmd = htol32(cmd);
	msg->len = htol32(len);
	msg->flags = (++prot->reqid << CDCF_IOC_ID_SHIFT);
	CDC_SET_IF_IDX(msg, ifidx);
	/* add additional action bits */
	action &= WL_IOCTL_ACTION_MASK;
	msg->flags |= (action << CDCF_IOC_ACTION_SHIFT) | CDCF_IOC_SET;
	msg->flags = htol32(msg->flags);

	if (buf)
		memcpy(prot->buf, buf, len);

	if ((ret = dhdcdc_msg(dhd)) < 0) {
		DHD_ERROR(("%s: dhdcdc_msg failed w/status %d\n", __FUNCTION__, ret));
		goto done;
	}

	if ((ret = dhdcdc_cmplt(dhd, prot->reqid, len)) < 0)
		goto done;

	flags = ltoh32(msg->flags);
	id = (flags & CDCF_IOC_ID_MASK) >> CDCF_IOC_ID_SHIFT;

	if (id != prot->reqid) {
		DHD_ERROR(("%s: %s: unexpected request id %d (expected %d)\n",
		           dhd_ifname(dhd, ifidx), __FUNCTION__, id, prot->reqid));
		ret = -EINVAL;
		goto done;
	}

	/* Copy fw response to buf */
	if (buf) {
		ASSERT(ret == len);
		memcpy(buf, (void*) prot->buf, len);
	}

	/* Check the ERROR flag */
	if (flags & CDCF_IOC_ERROR)
	{
		ret = ltoh32(msg->status);
		/* Cache error from dongle */
		dhd->dongle_error = ret;
	}

done:
	return ret;
}

#ifdef BCMDBUS
int
dhd_prot_ctl_complete(dhd_pub_t *dhd)
{
	dhd_prot_t *prot;

	if (dhd == NULL)
		return BCME_ERROR;

	prot = dhd->prot;

	ASSERT(prot);
	prot->ctl_completed = TRUE;
	dhd_os_ioctl_resp_wake(dhd);
	return 0;
}
#endif /* BCMDBUS */

/* XXX: due to overlays this should not be called directly; call dhd_wl_ioctl() instead */
int
dhd_prot_ioctl(dhd_pub_t *dhd, int ifidx, wl_ioctl_t * ioc, void * buf, int len)
{
	dhd_prot_t *prot = dhd->prot;
	int ret = -1;
	uint8 action;

	if ((dhd->busstate == DHD_BUS_DOWN) || dhd->hang_was_sent) {
		DHD_ERROR(("%s : bus is down. we have nothing to do - bs: %d, has: %d\n",
				__FUNCTION__, dhd->busstate, dhd->hang_was_sent));
		goto done;
	}

	DHD_TRACE(("%s: Enter\n", __FUNCTION__));

	ASSERT(len <= WLC_IOCTL_MAXLEN);

	if (len > WLC_IOCTL_MAXLEN)
		goto done;

	if (prot->pending == TRUE) {
		DHD_ERROR(("CDC packet is pending!!!! cmd=0x%x (%lu) lastcmd=0x%x (%lu)\n",
			ioc->cmd, (unsigned long)ioc->cmd, prot->lastcmd,
			(unsigned long)prot->lastcmd));
		if ((ioc->cmd == WLC_SET_VAR) || (ioc->cmd == WLC_GET_VAR)) {
			DHD_TRACE(("iovar cmd=%s\n", buf ? (char*)buf : "\0"));
		}
		goto done;
	}

	prot->pending = TRUE;
	prot->lastcmd = ioc->cmd;
	action = ioc->set;
	if (action & WL_IOCTL_ACTION_SET)
		ret = dhdcdc_set_ioctl(dhd, ifidx, ioc->cmd, buf, len, action);
	else {
		ret = dhdcdc_query_ioctl(dhd, ifidx, ioc->cmd, buf, len, action);
		if (ret > 0)
			ioc->used = ret - sizeof(cdc_ioctl_t);
	}

	/* Too many programs assume ioctl() returns 0 on success */
	if (ret >= 0)
		ret = 0;
	else {
		cdc_ioctl_t *msg = &prot->msg;
		ioc->needed = ltoh32(msg->len); /* len == needed when set/query fails from dongle */
	}

	/* Intercept the wme_dp ioctl here */
	if ((!ret) && (ioc->cmd == WLC_SET_VAR) && (!strcmp(buf, "wme_dp"))) {
		int slen, val = 0;

		slen = strlen("wme_dp") + 1;
		if (len >= (int)(slen + sizeof(int)))
			bcopy(((char *)buf + slen), &val, sizeof(int));
		dhd->wme_dp = (uint8) ltoh32(val);
	}

	prot->pending = FALSE;

done:

	return ret;
}

int
dhd_prot_iovar_op(dhd_pub_t *dhdp, const char *name,
                  void *params, int plen, void *arg, int len, bool set)
{
	return BCME_UNSUPPORTED;
}

void
dhd_prot_dump(dhd_pub_t *dhdp, struct bcmstrbuf *strbuf)
{
	if (!dhdp || !dhdp->prot) {
		return;
	}

	bcm_bprintf(strbuf, "Protocol CDC: reqid %d\n", dhdp->prot->reqid);
#ifdef PROP_TXSTATUS
	dhd_wlfc_dump(dhdp, strbuf);
#endif
}

void
dhd_prot_counters(dhd_pub_t *dhdp, struct bcmstrbuf *strbuf,
	bool print_ringinfo, bool print_pktidinfo)
{
	return;
}

void
dhd_bus_counters(dhd_pub_t *dhdp, struct bcmstrbuf *strbuf)
{
	return;
}

/*	The FreeBSD PKTPUSH could change the packet buf pinter
	so we need to make it changable
*/
#define PKTBUF pktbuf
void
dhd_prot_hdrpush(dhd_pub_t *dhd, int ifidx, void *PKTBUF)
{
#ifdef BDC
	struct bdc_header *h;
#endif /* BDC */

	DHD_TRACE(("%s: Enter\n", __FUNCTION__));

#ifdef BDC
	/* Push BDC header used to convey priority for buses that don't */

	PKTPUSH(dhd->osh, PKTBUF, BDC_HEADER_LEN);

	h = (struct bdc_header *)PKTDATA(dhd->osh, PKTBUF);

	h->flags = (BDC_PROTO_VER << BDC_FLAG_VER_SHIFT);
	if (PKTSUMNEEDED(PKTBUF))
		h->flags |= BDC_FLAG_SUM_NEEDED;

#ifdef EXT_STA
	/* save pkt encryption exemption info for dongle */
	h->flags &= ~BDC_FLAG_EXEMPT;
	h->flags |= (WLPKTFLAG_EXEMPT_GET(WLPKTTAG(pktbuf)) & BDC_FLAG_EXEMPT);
#endif /* EXT_STA */

	h->priority = (PKTPRIO(PKTBUF) & BDC_PRIORITY_MASK);
	h->flags2 = 0;
	h->dataOffset = 0;
#endif /* BDC */
	BDC_SET_IF_IDX(h, ifidx);
}
#undef PKTBUF	/* Only defined in the above routine */

uint
dhd_prot_hdrlen(dhd_pub_t *dhd, void *PKTBUF)
{
	uint hdrlen = 0;
#ifdef BDC
	/* Length of BDC(+WLFC) headers pushed */
	hdrlen = BDC_HEADER_LEN + (((struct bdc_header *)PKTBUF)->dataOffset * 4);
#endif
	return hdrlen;
}

int
dhd_prot_hdrpull(dhd_pub_t *dhd, int *ifidx, void *pktbuf, uchar *reorder_buf_info,
	uint *reorder_info_len)
{
#ifdef BDC
	struct bdc_header *h;
#endif
	uint8 data_offset = 0;

	DHD_TRACE(("%s: Enter\n", __FUNCTION__));

#ifdef BDC
	if (reorder_info_len)
		*reorder_info_len = 0;
	/* Pop BDC header used to convey priority for buses that don't */

	if (PKTLEN(dhd->osh, pktbuf) < BDC_HEADER_LEN) {
		DHD_ERROR(("%s: rx data too short (%d < %d)\n", __FUNCTION__,
		           PKTLEN(dhd->osh, pktbuf), BDC_HEADER_LEN));
		return BCME_ERROR;
	}

	h = (struct bdc_header *)PKTDATA(dhd->osh, pktbuf);

	if (!ifidx) {
		/* for tx packet, skip the analysis */
		data_offset = h->dataOffset;
		PKTPULL(dhd->osh, pktbuf, BDC_HEADER_LEN);
		goto exit;
	}

	*ifidx = BDC_GET_IF_IDX(h);

	if (((h->flags & BDC_FLAG_VER_MASK) >> BDC_FLAG_VER_SHIFT) != BDC_PROTO_VER) {
		DHD_ERROR(("%s: non-BDC packet received, flags = 0x%x\n",
		           dhd_ifname(dhd, *ifidx), h->flags));
		if (((h->flags & BDC_FLAG_VER_MASK) >> BDC_FLAG_VER_SHIFT) == BDC_PROTO_VER_1)
			h->dataOffset = 0;
		else
		return BCME_ERROR;
	}

	if (h->flags & BDC_FLAG_SUM_GOOD) {
		DHD_INFO(("%s: BDC packet received with good rx-csum, flags 0x%x\n",
		          dhd_ifname(dhd, *ifidx), h->flags));
		PKTSETSUMGOOD(pktbuf, TRUE);
	}

	PKTSETPRIO(pktbuf, (h->priority & BDC_PRIORITY_MASK));
	data_offset = h->dataOffset;
	PKTPULL(dhd->osh, pktbuf, BDC_HEADER_LEN);
#endif /* BDC */

#ifdef PROP_TXSTATUS
	if (!DHD_PKTTAG_PKTDIR(PKTTAG(pktbuf))) {
		/*
		- parse txstatus only for packets that came from the firmware
		*/
		dhd_wlfc_parse_header_info(dhd, pktbuf, (data_offset << 2),
			reorder_buf_info, reorder_info_len);

#ifdef BCMDBUS
#ifndef DHD_WLFC_THREAD
		dhd_wlfc_commit_packets(dhd,
			(f_commitpkt_t)dhd_dbus_txdata, (void *)dhd, NULL, FALSE);
#endif /* DHD_WLFC_THREAD */
#endif /* BCMDBUS */
	}
#endif /* PROP_TXSTATUS */

exit:
	PKTPULL(dhd->osh, pktbuf, (data_offset << 2));
	return 0;
}

int
dhd_prot_attach(dhd_pub_t *dhd)
{
	dhd_prot_t *cdc;

	if (!(cdc = (dhd_prot_t *)DHD_OS_PREALLOC(dhd, DHD_PREALLOC_PROT, sizeof(dhd_prot_t)))) {
		DHD_ERROR(("%s: kmalloc failed\n", __FUNCTION__));
		goto fail;
	}
	memset(cdc, 0, sizeof(dhd_prot_t));

	/* ensure that the msg buf directly follows the cdc msg struct */
	if ((uintptr)(&cdc->msg + 1) != (uintptr)cdc->buf) {
		DHD_ERROR(("dhd_prot_t is not correctly defined\n"));
		goto fail;
	}

	dhd->prot = cdc;
#ifdef BDC
	dhd->hdrlen += BDC_HEADER_LEN;
#endif
	dhd->maxctl = WLC_IOCTL_MAXLEN + sizeof(cdc_ioctl_t) + ROUND_UP_MARGIN;
	return 0;

fail:
	if (cdc != NULL)
		DHD_OS_PREFREE(dhd, cdc, sizeof(dhd_prot_t));
	return BCME_NOMEM;
}

/* ~NOTE~ What if another thread is waiting on the semaphore?  Holding it? */
void
dhd_prot_detach(dhd_pub_t *dhd)
{
#ifdef PROP_TXSTATUS
	dhd_wlfc_deinit(dhd);
#endif
	DHD_OS_PREFREE(dhd, dhd->prot, sizeof(dhd_prot_t));
	dhd->prot = NULL;
}

void
dhd_prot_dstats(dhd_pub_t *dhd)
{
	/*  copy bus stats */

	dhd->dstats.tx_packets = dhd->tx_packets;
	dhd->dstats.tx_errors = dhd->tx_errors;
	dhd->dstats.rx_packets = dhd->rx_packets;
	dhd->dstats.rx_errors = dhd->rx_errors;
	dhd->dstats.rx_dropped = dhd->rx_dropped;
	dhd->dstats.multicast = dhd->rx_multicast;
	return;
}

int
dhd_sync_with_dongle(dhd_pub_t *dhd)
{
	int ret = 0;
	wlc_rev_info_t revinfo;
	char buf[128];

	DHD_TRACE(("%s: Enter\n", __FUNCTION__));

#ifdef DHD_FW_COREDUMP
	/* Check the memdump capability */
	dhd_get_memdump_info(dhd);
#endif /* DHD_FW_COREDUMP */

#ifdef BCMASSERT_LOG
	dhd_get_assert_info(dhd);
#endif /* BCMASSERT_LOG */

	/* Get the device rev info */
	memset(&revinfo, 0, sizeof(revinfo));
	ret = dhd_wl_ioctl_cmd(dhd, WLC_GET_REVINFO, &revinfo, sizeof(revinfo), FALSE, 0);
	if (ret < 0)
		goto done;

	/* query for 'wlc_ver' to get version info from firmware */
	/* memsetting to zero */
	bzero(buf, sizeof(buf));
	ret = bcm_mkiovar("wlc_ver", NULL, 0, buf, sizeof(buf));
	if (ret == 0) {
		ret = BCME_BUFTOOSHORT;
		goto done;
	}
	ret = dhd_wl_ioctl_cmd(dhd, WLC_GET_VAR, buf, sizeof(buf), FALSE, 0);
	if (ret == BCME_UNSUPPORTED) {
		dhd->wlc_ver_major = DEFAULT_WLC_API_VERSION_MAJOR;
		dhd->wlc_ver_minor = DEFAULT_WLC_API_VERSION_MINOR;
	} else if (ret < 0) {
		DHD_ERROR(("%s failed %d\n", __FUNCTION__, ret));
		goto done;
	} else {
		dhd->wlc_ver_major = ((wl_wlc_version_t*)buf)->wlc_ver_major;
		dhd->wlc_ver_minor = ((wl_wlc_version_t*)buf)->wlc_ver_minor;
	}
	DHD_ERROR(("\nwlc_ver_major %d, wlc_ver_minor %d", dhd->wlc_ver_major, dhd->wlc_ver_minor));

#if defined(BCMDBUS) && defined(BCMDHDUSB)
	/* dbus_set_revinfo(dhd->dbus, revinfo.chipnum, revinfo.chiprev); */
#endif /* BCMDBUS && BCMDHDUSB */

	DHD_SSSR_DUMP_INIT(dhd);

	dhd_process_cid_mac(dhd, TRUE);
	ret = dhd_preinit_ioctls(dhd);
	dhd_process_cid_mac(dhd, FALSE);

	/* Always assumes wl for now */
	dhd->iswl = TRUE;

	/* XXX Could use WLC_GET_REVINFO to get driver version? */
done:
	return ret;
}

int dhd_prot_init(dhd_pub_t *dhd)
{
	return BCME_OK;
}

void
dhd_prot_stop(dhd_pub_t *dhd)
{
/* Nothing to do for CDC */
}

static void
dhd_get_hostreorder_pkts(void *osh, struct reorder_info *ptr, void **pkt,
	uint32 *pkt_count, void **pplast, uint8 start, uint8 end)
{
	void *plast = NULL, *p;
	uint32 pkt_cnt = 0;

	if (ptr->pend_pkts == 0) {
		DHD_REORDER(("%s: no packets in reorder queue \n", __FUNCTION__));
		*pplast = NULL;
		*pkt_count = 0;
		*pkt = NULL;
		return;
	}
	do {
		p = (void *)(ptr->p[start]);
		ptr->p[start] = NULL;

		if (p != NULL) {
			if (plast == NULL)
				*pkt = p;
			else
				PKTSETNEXT(osh, plast, p);

			plast = p;
			pkt_cnt++;
		}
		start++;
		if (start > ptr->max_idx)
			start = 0;
	} while (start != end);
	*pplast = plast;
	*pkt_count = pkt_cnt;
	ptr->pend_pkts -= (uint8)pkt_cnt;
}

int
dhd_process_pkt_reorder_info(dhd_pub_t *dhd, uchar *reorder_info_buf, uint reorder_info_len,
	void **pkt, uint32 *pkt_count)
{
	uint8 flow_id, max_idx, cur_idx, exp_idx;
	struct reorder_info *ptr;
	uint8 flags;
	void *cur_pkt, *plast = NULL;
	uint32 cnt = 0;

	if (pkt == NULL) {
		if (pkt_count != NULL)
			*pkt_count = 0;
		return 0;
	}

	flow_id = reorder_info_buf[WLHOST_REORDERDATA_FLOWID_OFFSET];
	flags = reorder_info_buf[WLHOST_REORDERDATA_FLAGS_OFFSET];

	DHD_REORDER(("flow_id %d, flags 0x%02x, idx(%d, %d, %d)\n", flow_id, flags,
		reorder_info_buf[WLHOST_REORDERDATA_CURIDX_OFFSET],
		reorder_info_buf[WLHOST_REORDERDATA_EXPIDX_OFFSET],
		reorder_info_buf[WLHOST_REORDERDATA_MAXIDX_OFFSET]));

	/* validate flags and flow id */
	if (flags == 0xFF) {
		DHD_ERROR(("%s: invalid flags...so ignore this packet\n", __FUNCTION__));
		*pkt_count = 1;
		return 0;
	}

	cur_pkt = *pkt;
	*pkt = NULL;

	ptr = dhd->reorder_bufs[flow_id];
	if (flags & WLHOST_REORDERDATA_DEL_FLOW) {
		uint32 buf_size = sizeof(struct reorder_info);

		DHD_REORDER(("%s: Flags indicating to delete a flow id %d\n",
			__FUNCTION__, flow_id));

		if (ptr == NULL) {
			DHD_REORDER(("%s: received flags to cleanup, but no flow (%d) yet\n",
				__FUNCTION__, flow_id));
			*pkt_count = 1;
			*pkt = cur_pkt;
			return 0;
		}

		dhd_get_hostreorder_pkts(dhd->osh, ptr, pkt, &cnt, &plast,
			ptr->exp_idx, ptr->exp_idx);
		/* set it to the last packet */
		if (plast) {
			PKTSETNEXT(dhd->osh, plast, cur_pkt);
			cnt++;
		}
		else {
			if (cnt != 0) {
				DHD_ERROR(("%s: del flow: something fishy, pending packets %d\n",
					__FUNCTION__, cnt));
			}
			*pkt = cur_pkt;
			cnt = 1;
		}
		buf_size += ((ptr->max_idx + 1) * sizeof(void *));
		MFREE(dhd->osh, ptr, buf_size);
		dhd->reorder_bufs[flow_id] = NULL;
		*pkt_count = cnt;
		return 0;
	}
	/* all the other cases depend on the existance of the reorder struct for that flow id */
	if (ptr == NULL) {
		uint32 buf_size_alloc = sizeof(reorder_info_t);
		max_idx = reorder_info_buf[WLHOST_REORDERDATA_MAXIDX_OFFSET];

		buf_size_alloc += ((max_idx + 1) * sizeof(void*));
		/* allocate space to hold the buffers, index etc */

		DHD_REORDER(("%s: alloc buffer of size %d size, reorder info id %d, maxidx %d\n",
			__FUNCTION__, buf_size_alloc, flow_id, max_idx));
		ptr = (struct reorder_info *)MALLOC(dhd->osh, buf_size_alloc);
		if (ptr == NULL) {
			DHD_ERROR(("%s: Malloc failed to alloc buffer\n", __FUNCTION__));
			*pkt_count = 1;
			return 0;
		}
		bzero(ptr, buf_size_alloc);
		dhd->reorder_bufs[flow_id] = ptr;
		ptr->p = (void *)(ptr+1);
		ptr->max_idx = max_idx;
	}
	/* XXX: validate cur, exp indices */
	if (flags & WLHOST_REORDERDATA_NEW_HOLE)  {
		DHD_REORDER(("%s: new hole, so cleanup pending buffers\n", __FUNCTION__));
		if (ptr->pend_pkts) {
			dhd_get_hostreorder_pkts(dhd->osh, ptr, pkt, &cnt, &plast,
				ptr->exp_idx, ptr->exp_idx);
			ptr->pend_pkts = 0;
		}
		ptr->cur_idx = reorder_info_buf[WLHOST_REORDERDATA_CURIDX_OFFSET];
		ptr->exp_idx = reorder_info_buf[WLHOST_REORDERDATA_EXPIDX_OFFSET];
		ptr->max_idx = reorder_info_buf[WLHOST_REORDERDATA_MAXIDX_OFFSET];
		ptr->p[ptr->cur_idx] = cur_pkt;
		ptr->pend_pkts++;
		*pkt_count = cnt;
	}
	else if (flags & WLHOST_REORDERDATA_CURIDX_VALID) {
		cur_idx = reorder_info_buf[WLHOST_REORDERDATA_CURIDX_OFFSET];
		exp_idx = reorder_info_buf[WLHOST_REORDERDATA_EXPIDX_OFFSET];

		if ((exp_idx == ptr->exp_idx) && (cur_idx != ptr->exp_idx)) {
			/* still in the current hole */
			/* enqueue the current on the buffer chain */
			if (ptr->p[cur_idx] != NULL) {
				DHD_REORDER(("%s: HOLE: ERROR buffer pending..free it\n",
					__FUNCTION__));
				PKTFREE(dhd->osh, ptr->p[cur_idx], TRUE);
				ptr->p[cur_idx] = NULL;
			}
			ptr->p[cur_idx] = cur_pkt;
			ptr->pend_pkts++;
			ptr->cur_idx = cur_idx;
			DHD_REORDER(("%s: fill up a hole..pending packets is %d\n",
				__FUNCTION__, ptr->pend_pkts));
			*pkt_count = 0;
			*pkt = NULL;
		}
		else if (ptr->exp_idx == cur_idx) {
			/* got the right one ..flush from cur to exp and update exp */
			DHD_REORDER(("%s: got the right one now, cur_idx is %d\n",
				__FUNCTION__, cur_idx));
			if (ptr->p[cur_idx] != NULL) {
				DHD_REORDER(("%s: Error buffer pending..free it\n",
					__FUNCTION__));
				PKTFREE(dhd->osh, ptr->p[cur_idx], TRUE);
				ptr->p[cur_idx] = NULL;
			}
			ptr->p[cur_idx] = cur_pkt;
			ptr->pend_pkts++;

			ptr->cur_idx = cur_idx;
			ptr->exp_idx = exp_idx;

			dhd_get_hostreorder_pkts(dhd->osh, ptr, pkt, &cnt, &plast,
				cur_idx, exp_idx);
			*pkt_count = cnt;
			DHD_REORDER(("%s: freeing up buffers %d, still pending %d\n",
				__FUNCTION__, cnt, ptr->pend_pkts));
		}
		else {
			uint8 end_idx;
			bool flush_current = FALSE;
			/* both cur and exp are moved now .. */
			DHD_REORDER(("%s:, flow %d, both moved, cur %d(%d), exp %d(%d)\n",
				__FUNCTION__, flow_id, ptr->cur_idx, cur_idx,
				ptr->exp_idx, exp_idx));
			if (flags & WLHOST_REORDERDATA_FLUSH_ALL)
				end_idx = ptr->exp_idx;
			else
				end_idx = exp_idx;

			/* flush pkts first */
			dhd_get_hostreorder_pkts(dhd->osh, ptr, pkt, &cnt, &plast,
				ptr->exp_idx, end_idx);

			if (cur_idx == ptr->max_idx) {
				if (exp_idx == 0)
					flush_current = TRUE;
			} else {
				if (exp_idx == cur_idx + 1)
					flush_current = TRUE;
			}
			if (flush_current) {
				if (plast)
					PKTSETNEXT(dhd->osh, plast, cur_pkt);
				else
					*pkt = cur_pkt;
				cnt++;
			}
			else {
				ptr->p[cur_idx] = cur_pkt;
				ptr->pend_pkts++;
			}
			ptr->exp_idx = exp_idx;
			ptr->cur_idx = cur_idx;
			*pkt_count = cnt;
		}
	}
	else {
		uint8 end_idx;
		/* no real packet but update to exp_seq...that means explicit window move */
		exp_idx = reorder_info_buf[WLHOST_REORDERDATA_EXPIDX_OFFSET];

		DHD_REORDER(("%s: move the window, cur_idx is %d, exp is %d, new exp is %d\n",
			__FUNCTION__, ptr->cur_idx, ptr->exp_idx, exp_idx));
		if (flags & WLHOST_REORDERDATA_FLUSH_ALL)
			end_idx =  ptr->exp_idx;
		else
			end_idx =  exp_idx;

		dhd_get_hostreorder_pkts(dhd->osh, ptr, pkt, &cnt, &plast, ptr->exp_idx, end_idx);
		if (plast)
			PKTSETNEXT(dhd->osh, plast, cur_pkt);
		else
			*pkt = cur_pkt;
		cnt++;
		*pkt_count = cnt;
		/* set the new expected idx */
		ptr->exp_idx = exp_idx;
	}
	return 0;
}
