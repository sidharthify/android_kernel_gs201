/*
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
 * <<Broadcom-WL-IPTag/Dual:>>
 *
 */

/** WL flow control for PROP_TXSTATUS. Related to host AMPDU reordering. */

#ifndef __wlfc_proto_definitions_h__
#define __wlfc_proto_definitions_h__

	/* Use TLV to convey WLFC information.
	 ---------------------------------------------------------------------------
	| Type |  Len | value                    | Description
	 ---------------------------------------------------------------------------
	|  1   |   1  | (handle)                 | MAC OPEN
	 ---------------------------------------------------------------------------
	|  2   |   1  | (handle)                 | MAC CLOSE
	 ---------------------------------------------------------------------------
	|  3   |   2  | (count, handle, prec_bmp)| Set the credit depth for a MAC dstn
	 ---------------------------------------------------------------------------
	|  4   |   4+ | see pkttag comments      | TXSTATUS
	|      |   12 | TX status & timestamps   | Present only when pkt timestamp is enabled
	 ---------------------------------------------------------------------------
	|  5   |   4  | see pkttag comments      | PKKTTAG [host->firmware]
	 ---------------------------------------------------------------------------
	|  6   |   8  | (handle, ifid, MAC)      | MAC ADD
	 ---------------------------------------------------------------------------
	|  7   |   8  | (handle, ifid, MAC)      | MAC DEL
	 ---------------------------------------------------------------------------
	|  8   |   1  | (rssi)                   | RSSI - RSSI value for the packet.
	 ---------------------------------------------------------------------------
	|  9   |   1  | (interface ID)           | Interface OPEN
	 ---------------------------------------------------------------------------
	|  10  |   1  | (interface ID)           | Interface CLOSE
	 ---------------------------------------------------------------------------
	|  11  |   8  | fifo credit returns map  | FIFO credits back to the host
	|      |      |                          |
	|      |      |                          | --------------------------------------
	|      |      |                          | | ac0 | ac1 | ac2 | ac3 | bcmc | atim |
	|      |      |                          | --------------------------------------
	|      |      |                          |
	 ---------------------------------------------------------------------------
	|  12  |   2  | MAC handle,              | Host provides a bitmap of pending
	|      |      | AC[0-3] traffic bitmap   | unicast traffic for MAC-handle dstn.
	|      |      |                          | [host->firmware]
	 ---------------------------------------------------------------------------
	|  13  |   3  | (count, handle, prec_bmp)| One time request for packet to a specific
	|      |      |                          | MAC destination.
	 ---------------------------------------------------------------------------
	|  15  |  12  | (pkttag, timestamps)     | Send TX timestamp at reception from host
	 ---------------------------------------------------------------------------
	|  16  |  12  | (pkttag, timestamps)     | Send WLAN RX timestamp along with RX frame
	 ---------------------------------------------------------------------------
	| 255  |  N/A |  N/A                     | FILLER - This is a special type
	|      |      |                          | that has no length or value.
	|      |      |                          | Typically used for padding.
	 ---------------------------------------------------------------------------
	*/

typedef enum {
	WLFC_CTL_TYPE_MAC_OPEN			= 1,
	WLFC_CTL_TYPE_MAC_CLOSE			= 2,
	WLFC_CTL_TYPE_MAC_REQUEST_CREDIT	= 3,
	WLFC_CTL_TYPE_TXSTATUS			= 4,
	WLFC_CTL_TYPE_PKTTAG			= 5, /** host<->dongle */

	WLFC_CTL_TYPE_MACDESC_ADD		= 6,
	WLFC_CTL_TYPE_MACDESC_DEL		= 7,
	WLFC_CTL_TYPE_RSSI			= 8,

	WLFC_CTL_TYPE_INTERFACE_OPEN		= 9,
	WLFC_CTL_TYPE_INTERFACE_CLOSE		= 10,

	WLFC_CTL_TYPE_FIFO_CREDITBACK		= 11,

	WLFC_CTL_TYPE_PENDING_TRAFFIC_BMP	= 12, /** host->dongle */
	WLFC_CTL_TYPE_MAC_REQUEST_PACKET	= 13,
	WLFC_CTL_TYPE_HOST_REORDER_RXPKTS	= 14,

	WLFC_CTL_TYPE_TX_ENTRY_STAMP		= 15,
	WLFC_CTL_TYPE_RX_STAMP			= 16,

	WLFC_CTL_TYPE_UPD_FLR_FETCH		= 17, /* PCIE_FLOWCTL: Update Flowring Fetch */

	WLFC_CTL_TYPE_TRANS_ID			= 18,
	WLFC_CTL_TYPE_COMP_TXSTATUS		= 19,

	WLFC_CTL_TYPE_TID_OPEN			= 20, /* open flowring/s with tid */
	WLFC_CTL_TYPE_TID_CLOSE			= 21, /* close flowring/s with tid */
	WLFC_CTL_TYPE_UPD_FLR_WEIGHT		= 22, /* WLATF_DONGLE */
	WLFC_CTL_TYPE_ENAB_FFSCH		= 23, /* WLATF_DONGLE */

	WLFC_CTL_TYPE_UPDATE_FLAGS		= 24, /* clear the flags set in flowring */
	WLFC_CTL_TYPE_CLEAR_SUPPR		= 25, /* free the supression info in the flowring */

	WLFC_CTL_TYPE_FLOWID_OPEN		= 26, /* open flowring with flowid */
	WLFC_CTL_TYPE_FLOWID_CLOSE		= 27, /* close flowring with flowid */

	WLFC_CTL_TYPE_PENDING_TX_PKTS		= 28, /* Get the outstandinding packets in host
							* flowring for the given interface.
							*/
	WLFC_CTL_TYPE_UPD_SCB_RATESEL_CHANGE	= 29, /* Upd flow's max rate dynamically */
	WLFC_CTL_TYPE_AMSDU_STATE		= 30, /* Upd flow's AMSDU state(Enabled/Disabled) */
	WLFC_CTL_TYPE_APP_STATE			= 31, /* Upd flow's APP state, enable/disable APP */
	WLFC_CTL_TYPE_HP2P_EXT_TXSTATUS		= 32, /* Hp2p extended tx status */
	WLFC_CTL_TYPE_HP2P_ACTIVE_STATE		= 33, /* Get status of HP2P ring active or not */
	WLFC_CTL_TYPE_HP2P_QUERY_LIFETIME	= 34, /* Query lifetime for last unacked */

	WLFC_CTL_TYPE_FLOWID_MAC		= 35, /* Get flow's MAC (used by WLMESH) */

	WLFC_CTL_TYPE_LLW_OPEN              = 36,
	WLFC_CTL_TYPE_LLW_CLOSE             = 37,

	WLFC_CTL_TYPE_FILLER			= 255
} wlfc_ctl_type_t;

#define WLFC_CTL_VALUE_LEN_FLOWID		2u	/* flowid legth in TLV */

#define WLFC_CTL_VALUE_LEN_MACDESC		8u	/** handle, interface, MAC */

#define WLFC_CTL_VALUE_LEN_MAC			1u	/** MAC-handle */
#define WLFC_CTL_VALUE_LEN_RSSI			1u

#define WLFC_CTL_VALUE_LEN_INTERFACE		1u
#define WLFC_CTL_VALUE_LEN_PENDING_TRAFFIC_BMP	2u

#define WLFC_CTL_VALUE_LEN_TXSTATUS		4u
#define WLFC_CTL_VALUE_LEN_PKTTAG		4u
#define WLFC_CTL_VALUE_LEN_TIMESTAMP		12u	/** 4-byte rate info + 2 TSF */

#define WLFC_CTL_VALUE_LEN_SEQ			2u
#define WLFC_CTL_VALUE_LEN_TID			3u	/* interface index, TID */

#define WLFC_CTL_EXT_TXSTATUS_PAYLOAD_LEN	8u	/* Payload legnth of extention tx status */

#define WLFC_CTL_VALUE_LEN_LLW			8u	/** tid, ifindex, MAC */

/* Reset the flags set for the corresponding flowring of the SCB which is de-inited */
/* FLOW_RING_FLAG_LAST_TIM | FLOW_RING_FLAG_INFORM_PKTPEND | FLOW_RING_FLAG_PKT_REQ */
#define WLFC_RESET_ALL_FLAGS			0
#define WLFC_CTL_VALUE_LEN_FLAGS		7	/** flags, MAC */

/* free the data stored to be used for suppressed packets in future */
#define WLFC_CTL_VALUE_LEN_SUPR			8	/** ifindex, tid, MAC */

#define WLFC_CTL_VALUE_LEN_SCB_RATESEL_CHANGE		7	/* ifindex, MAC */
/* enough space to host all 4 ACs, bc/mc and atim fifo credit */
#define WLFC_CTL_VALUE_LEN_FIFO_CREDITBACK	6

#define WLFC_CTL_VALUE_LEN_REQUEST_CREDIT	3	/* credit, MAC-handle, prec_bitmap */
#define WLFC_CTL_VALUE_LEN_REQUEST_PACKET	3	/* credit, MAC-handle, prec_bitmap */

/*
	WLFC packet identifier: b[31:0] (WLFC_CTL_TYPE_PKTTAG)

	Generation	: b[31]		=> generation number for this packet [host->fw]
	                           OR, current generation number [fw->host]
	Flags		: b[30:27]	=> command, status flags
	FIFO-AC		: b[26:24]	=> AC-FIFO id

	h-slot		: b[23:8]	=> hanger-slot
	freerun		: b[7:0]	=> A free running counter?

	As far as the firmware is concerned, host generated b[23:0] should be just
	reflected back on txstatus.
*/

#ifndef WLFC_PKTFLAG_COMPAT
#define WLFC_PKTFLAG_PKTFROMHOST	0x01
#define WLFC_PKTFLAG_PKT_REQUESTED	0x02
#define WLFC_PKTFLAG_PKT_SENDTOHOST	0x04
#define WLFC_PKTFLAG_PKT_FLUSHED	0x08
#else
#define WLFC_PKTFLAG_PKTFROMHOST_MASK		0x01
#define WLFC_PKTFLAG_PKT_REQUESTED_MASK		0x02
#define WLFC_PKTFLAG_PKT_SENDTOHOST_MASK	0x04
#define WLFC_PKTFLAG_PKT_FLUSHED_MASK		0x08
#endif /* WLFC_PKTFLAG_COMPAT */

#define WL_TXSTATUS_STATUS_MASK			0xffu /* allow 8 bits */
#define WL_TXSTATUS_STATUS_SHIFT		24u

#define WL_TXSTATUS_SET_STATUS(x, status)	((x)  = \
	((x) & ~(WL_TXSTATUS_STATUS_MASK << WL_TXSTATUS_STATUS_SHIFT)) | \
	(((status) & WL_TXSTATUS_STATUS_MASK) << WL_TXSTATUS_STATUS_SHIFT))
#define WL_TXSTATUS_GET_STATUS(x)	(((x) >> WL_TXSTATUS_STATUS_SHIFT) & \
	WL_TXSTATUS_STATUS_MASK)

/**
 * Bit 31 of the 32-bit packet tag is defined as 'generation ID'. It is set by the host to the
 * "current" generation, and by the firmware to the "expected" generation, toggling on suppress. The
 * firmware accepts a packet when the generation matches; on reset (startup) both "current" and
 * "expected" are set to 0.
 */
#define WL_TXSTATUS_GENERATION_MASK		1u /* allow 1 bit */
#define WL_TXSTATUS_GENERATION_SHIFT		31u

#define WL_TXSTATUS_SET_GENERATION(x, gen)	((x) = \
	((x) & ~(WL_TXSTATUS_GENERATION_MASK << WL_TXSTATUS_GENERATION_SHIFT)) | \
	(((gen) & WL_TXSTATUS_GENERATION_MASK) << WL_TXSTATUS_GENERATION_SHIFT))

#define WL_TXSTATUS_GET_GENERATION(x)	(((x) >> WL_TXSTATUS_GENERATION_SHIFT) & \
	WL_TXSTATUS_GENERATION_MASK)

#define WL_TXSTATUS_FLAGS_MASK			0xfu /* allow 4 bits only */
#define WL_TXSTATUS_FLAGS_SHIFT			27u

#define WL_TXSTATUS_SET_FLAGS(x, flags)	((x)  = \
	((x) & ~(WL_TXSTATUS_FLAGS_MASK << WL_TXSTATUS_FLAGS_SHIFT)) | \
	(((flags) & WL_TXSTATUS_FLAGS_MASK) << WL_TXSTATUS_FLAGS_SHIFT))
#define WL_TXSTATUS_GET_FLAGS(x)		(((x) >> WL_TXSTATUS_FLAGS_SHIFT) & \
	WL_TXSTATUS_FLAGS_MASK)
#define WL_TXSTATUS_CLEAR_FLAGS(x, flags)	((x)  = \
	((x) & ~(((flags) & WL_TXSTATUS_FLAGS_MASK) << WL_TXSTATUS_FLAGS_SHIFT)))

#define WL_TXSTATUS_FIFO_MASK			0x7u /* allow 3 bits for FIFO ID */
#define WL_TXSTATUS_FIFO_SHIFT			24u

#define WL_TXSTATUS_SET_FIFO(x, flags)	((x)  = \
	((x) & ~(WL_TXSTATUS_FIFO_MASK << WL_TXSTATUS_FIFO_SHIFT)) | \
	(((flags) & WL_TXSTATUS_FIFO_MASK) << WL_TXSTATUS_FIFO_SHIFT))
#define WL_TXSTATUS_GET_FIFO(x)		(((x) >> WL_TXSTATUS_FIFO_SHIFT) & WL_TXSTATUS_FIFO_MASK)

#define WL_TXSTATUS_PKTID_MASK			0xffffffu /* allow 24 bits */
#define WL_TXSTATUS_SET_PKTID(x, num)	((x) = \
	((x) & ~WL_TXSTATUS_PKTID_MASK) | (num))
#define WL_TXSTATUS_GET_PKTID(x)		((x) & WL_TXSTATUS_PKTID_MASK)

#define WL_TXSTATUS_HSLOT_MASK			0xffffu /* allow 16 bits */
#define WL_TXSTATUS_HSLOT_SHIFT			8u

#define WL_TXSTATUS_SET_HSLOT(x, hslot)	((x)  = \
	((x) & ~(WL_TXSTATUS_HSLOT_MASK << WL_TXSTATUS_HSLOT_SHIFT)) | \
	(((hslot) & WL_TXSTATUS_HSLOT_MASK) << WL_TXSTATUS_HSLOT_SHIFT))
#define WL_TXSTATUS_GET_HSLOT(x)	(((x) >> WL_TXSTATUS_HSLOT_SHIFT)& \
	WL_TXSTATUS_HSLOT_MASK)

#define WL_TXSTATUS_FREERUNCTR_MASK		0xffu /* allow 8 bits */

#define WL_TXSTATUS_SET_FREERUNCTR(x, ctr)	((x)  = \
	((x) & ~(WL_TXSTATUS_FREERUNCTR_MASK)) | \
	((ctr) & WL_TXSTATUS_FREERUNCTR_MASK))
#define WL_TXSTATUS_GET_FREERUNCTR(x)		((x)& WL_TXSTATUS_FREERUNCTR_MASK)

/* packet prio phase bit updated */
#define WL_SEQ_PKTPRIO_PHASE_MASK	0x1
#define WL_SEQ_PKTPRIO_PHASE_SHIFT	15
#define WL_SEQ_SET_PKTPRIO_PHASE(x, val)		((x) = \
	((x) & ~(WL_SEQ_PKTPRIO_PHASE_MASK << WL_SEQ_PKTPRIO_PHASE_SHIFT)) | \
	(((val) & WL_SEQ_PKTPRIO_PHASE_MASK) << WL_SEQ_PKTPRIO_PHASE_SHIFT))
#define WL_SEQ_PKTPRIO_PHASE(x)	(((x) >> WL_SEQ_PKTPRIO_PHASE_SHIFT) & \
	WL_SEQ_PKTPRIO_PHASE_MASK)

/* AMSDU part of d11 seq number */
#define WL_SEQ_AMSDU_MASK             0x1 /* allow 1 bit */
#define WL_SEQ_AMSDU_SHIFT            14
#define WL_SEQ_SET_AMSDU(x, val)      ((x) = \
	((x) & ~(WL_SEQ_AMSDU_MASK << WL_SEQ_AMSDU_SHIFT)) | \
	(((val) & WL_SEQ_AMSDU_MASK) << WL_SEQ_AMSDU_SHIFT)) /**< sets a single AMSDU bit */
/** returns TRUE if ring item is AMSDU (seq = d11 seq nr) */
#define WL_SEQ_IS_AMSDU(x)   (((x) >> WL_SEQ_AMSDU_SHIFT) & \
	WL_SEQ_AMSDU_MASK)

/* indicates last_suppr_seq is valid */
#define WL_SEQ_VALIDSUPPR_MASK		0x1 /* allow 1 bit */
#define WL_SEQ_VALIDSUPPR_SHIFT		12
#define WL_SEQ_SET_VALIDSUPPR(x, val)	((x) = \
	((x) & ~(WL_SEQ_VALIDSUPPR_MASK << WL_SEQ_VALIDSUPPR_SHIFT)) | \
	(((val) & WL_SEQ_VALIDSUPPR_MASK) << WL_SEQ_VALIDSUPPR_SHIFT))
#define WL_SEQ_GET_VALIDSUPPR(x)	(((x) >> WL_SEQ_VALIDSUPPR_SHIFT) & \
	WL_SEQ_VALIDSUPPR_MASK)

#define WL_SEQ_FROMFW_MASK		0x1 /* allow 1 bit */
#define WL_SEQ_FROMFW_SHIFT		13
#define WL_SEQ_SET_FROMFW(x, val)	((x) = \
	((x) & ~(WL_SEQ_FROMFW_MASK << WL_SEQ_FROMFW_SHIFT)) | \
	(((val) & WL_SEQ_FROMFW_MASK) << WL_SEQ_FROMFW_SHIFT))
/** Set when firmware assigns D11 sequence number to packet */
#define SET_WL_HAS_ASSIGNED_SEQ(x)	WL_SEQ_SET_FROMFW((x), 1)

/** returns TRUE if packet has been assigned a d11 seq number by the WL firmware layer */
#define GET_WL_HAS_ASSIGNED_SEQ(x)	(((x) >> WL_SEQ_FROMFW_SHIFT) & WL_SEQ_FROMFW_MASK)

#ifdef WLFC_PKTFLAG_COMPAT
/* Helper macros for WLFC pktflags */
#define WLFC_PKTFLAG_PKTFROMHOST(p) \
	(WL_TXSTATUS_GET_FLAGS(WLPKTTAG(p)->wl_hdr_information) & WLFC_PKTFLAG_PKTFROMHOST_MASK)
#define WLFC_PKTFLAG_PKT_REQUESTED(p) \
	(WL_TXSTATUS_GET_FLAGS(WLPKTTAG(p)->wl_hdr_information) & WLFC_PKTFLAG_PKT_REQUESTED_MASK)
#define WLFC_PKTFLAG_PKT_SENDTOHOST(p) \
	(WL_TXSTATUS_GET_FLAGS(WLPKTTAG(p)->wl_hdr_information) & WLFC_PKTFLAG_PKT_SENDTOHOST_MASK)
#define WLFC_PKTFLAG_PKT_FLUSHED(p) \
	(WL_TXSTATUS_GET_FLAGS(WLPKTTAG(p)->wl_hdr_information) & WLFC_PKTFLAG_PKT_FLUSHED_MASK)
#endif /* WLFC_PKTFLAG_COMPAT */

/**
 * Proptxstatus related.
 *
 * When a packet is suppressed by WL or the D11 core, the packet has to be retried. Assigning
 * a new d11 sequence number for the packet when retrying would cause the peer to be unable to
 * reorder the packets within an AMPDU. So, suppressed packet from bus layer (DHD for SDIO and
 * pciedev for PCIE) is re-using d11 seq number, so FW should not assign a new one.
 */
#define WL_SEQ_FROMDRV_MASK		0x1 /* allow 1 bit */
#define WL_SEQ_FROMDRV_SHIFT		12

/**
 * Proptxstatus, host or fw PCIe layer requests WL layer to reuse d11 seq no. Bit is reset by WL
 * subsystem when it reuses the seq number.
 */
#define WL_SEQ_SET_REUSE(x, val)	((x) = \
	((x) & ~(WL_SEQ_FROMDRV_MASK << WL_SEQ_FROMDRV_SHIFT)) | \
	(((val) & WL_SEQ_FROMDRV_MASK) << WL_SEQ_FROMDRV_SHIFT))
#define SET_WL_TO_REUSE_SEQ(x)   WL_SEQ_SET_REUSE((x), 1)
#define RESET_WL_TO_REUSE_SEQ(x) WL_SEQ_SET_REUSE((x), 0)

/** Proptxstatus, related to reuse of d11 seq numbers when retransmitting */
#define IS_WL_TO_REUSE_SEQ(x)	(((x) >> WL_SEQ_FROMDRV_SHIFT) & \
	WL_SEQ_FROMDRV_MASK)

#define WL_SEQ_NUM_MASK			0xfff /* allow 12 bit */
#define WL_SEQ_NUM_SHIFT		0
/** Proptxstatus, sets d11seq no in pkt tag, related to reuse of d11seq no when retransmitting */
#define WL_SEQ_SET_NUM(x, val)	((x) = \
	((x) & ~(WL_SEQ_NUM_MASK << WL_SEQ_NUM_SHIFT)) | \
	(((val) & WL_SEQ_NUM_MASK) << WL_SEQ_NUM_SHIFT))
/** Proptxstatus, gets d11seq no from pkt tag, related to reuse of d11seq no when retransmitting */
#define WL_SEQ_GET_NUM(x)	(((x) >> WL_SEQ_NUM_SHIFT) & \
	WL_SEQ_NUM_MASK)

#define WL_SEQ_AMSDU_SUPPR_MASK	((WL_SEQ_FROMDRV_MASK << WL_SEQ_FROMDRV_SHIFT) | \
				(WL_SEQ_AMSDU_MASK << WL_SEQ_AMSDU_SHIFT) | \
				(WL_SEQ_NUM_MASK << WL_SEQ_NUM_SHIFT))

/* 32 STA should be enough??, 6 bits; Must be power of 2 */
#define WLFC_MAC_DESC_TABLE_SIZE	32
#define WLFC_MAX_IFNUM				16
#define WLFC_MAC_DESC_ID_INVALID	0xff

/* b[7:5] -reuse guard, b[4:0] -value */
#define WLFC_MAC_DESC_GET_LOOKUP_INDEX(x) ((x) & 0x1f)

#define WLFC_PKTFLAG_SET_PKTREQUESTED(x)	(x) |= \
	(WLFC_PKTFLAG_PKT_REQUESTED << WL_TXSTATUS_FLAGS_SHIFT)

#define WLFC_PKTFLAG_CLR_PKTREQUESTED(x)	(x) &= \
	~(WLFC_PKTFLAG_PKT_REQUESTED << WL_TXSTATUS_FLAGS_SHIFT)

#define WLFC_MAX_PENDING_DATALEN	120

/* host is free to discard the packet */
#define WLFC_CTL_PKTFLAG_DISCARD	0
/* D11 suppressed a packet */
#define WLFC_CTL_PKTFLAG_D11SUPPRESS	1
/* WL firmware suppressed a packet because MAC is
	already in PSMode (short time window)
*/
#define WLFC_CTL_PKTFLAG_WLSUPPRESS	2
/* Firmware tossed this packet */
#define WLFC_CTL_PKTFLAG_TOSSED_BYWLC	3
/* Firmware tossed after retries */
#define WLFC_CTL_PKTFLAG_DISCARD_NOACK	4
/* Firmware wrongly reported suppressed previously,now fixing to acked */
#define WLFC_CTL_PKTFLAG_SUPPRESS_ACKED	5
/* Firmware send this packet expired, lifetime expiration */
#define WLFC_CTL_PKTFLAG_EXPIRED	6
/* Firmware drop this packet for any other reason  */
#define WLFC_CTL_PKTFLAG_DROPPED	7
/* Firmware free this packet  */
#define WLFC_CTL_PKTFLAG_MKTFREE	8
/* Firmware drop this packet due to AMPDU cleanup  */
#define WLFC_CTL_PKTFLAG_AMPDU_CLEANUP	8
/* Firmware dropped the frame after suppress retries reached max */
#define WLFC_CTL_PKTFLAG_MAX_SUP_RETR   9

/* Firmware forced packet lifetime expiry */
#define WLFC_CTL_PKTFLAG_FORCED_EXPIRED	10

#define WLFC_CTL_PKTFLAG_MASK		(0x0f)	/* For 4-bit mask with one extra bit */

#if defined(PROP_TXSTATUS_DEBUG) && !defined(BINCMP)
#define WLFC_DBGMESG(x) printf x
/* wlfc-breadcrumb */
#define WLFC_BREADCRUMB(x) do {if ((x) == NULL) \
	{printf("WLFC :%d:caller:%p\n", \
	__LINE__, CALL_SITE);}} while (0)
#define WLFC_WHEREIS(s) printf("WLFC: at %d, %s\n", __LINE__, (s))
#else
#define WLFC_DBGMESG(x)
#define WLFC_BREADCRUMB(x)
#define WLFC_WHEREIS(s)
#endif /* PROP_TXSTATUS_DEBUG && !BINCMP */

/* AMPDU host reorder packet flags */
#define WLHOST_REORDERDATA_MAXFLOWS		256
#define WLHOST_REORDERDATA_LEN		 10
#define WLHOST_REORDERDATA_TOTLEN	(WLHOST_REORDERDATA_LEN + 1 + 1) /* +tag +len */

#define WLHOST_REORDERDATA_FLOWID_OFFSET		0
#define WLHOST_REORDERDATA_MAXIDX_OFFSET		2
#define WLHOST_REORDERDATA_FLAGS_OFFSET			4
#define WLHOST_REORDERDATA_CURIDX_OFFSET		6
#define WLHOST_REORDERDATA_EXPIDX_OFFSET		8

#define WLHOST_REORDERDATA_DEL_FLOW		0x01
#define WLHOST_REORDERDATA_FLUSH_ALL		0x02
#define WLHOST_REORDERDATA_CURIDX_VALID		0x04
#define WLHOST_REORDERDATA_EXPIDX_VALID		0x08
#define WLHOST_REORDERDATA_NEW_HOLE		0x10

/* transaction id data len byte 0: rsvd, byte 1: seqnumber, byte 2-5 will be used for timestampe */
#define WLFC_CTL_TRANS_ID_LEN			6
#define WLFC_TYPE_TRANS_ID_LEN			6

#define WLFC_MODE_HANGER	1 /* use hanger */
#define WLFC_MODE_AFQ		2 /* use afq (At Firmware Queue) */
#define WLFC_IS_OLD_DEF(x) ((x & 1) || (x & 2))

#define WLFC_MODE_AFQ_SHIFT		2u	/* afq bit */
#define WLFC_SET_AFQ(x, val)	((x) = \
	((x) & ~(1u << WLFC_MODE_AFQ_SHIFT)) | \
	(((val) & 1u) << WLFC_MODE_AFQ_SHIFT))
/** returns TRUE if firmware supports 'at firmware queue' feature */
#define WLFC_GET_AFQ(x)	(((x) >> WLFC_MODE_AFQ_SHIFT) & 1)

#define WLFC_MODE_REUSESEQ_SHIFT	3u	/* seq reuse bit */
#define WLFC_SET_REUSESEQ(x, val)	((x) = \
	((x) & ~(1u << WLFC_MODE_REUSESEQ_SHIFT)) | \
	(((val) & 1u) << WLFC_MODE_REUSESEQ_SHIFT))

/** returns TRUE if 'd11 sequence reuse' has been agreed upon between host and dongle */
#if defined(BCMPCIEDEV_ENABLED)
/* GET_REUSESEQ is always TRUE in pciedev */
#define WLFC_GET_REUSESEQ(x)	(TRUE)
#else
#define WLFC_GET_REUSESEQ(x)	(((x) >> WLFC_MODE_REUSESEQ_SHIFT) & 1)
#endif /* defined(BCMPCIEDEV_ENABLED) */

#define WLFC_MODE_REORDERSUPP_SHIFT	4u	/* host reorder suppress pkt bit */
#define WLFC_SET_REORDERSUPP(x, val)	((x) = \
	((x) & ~(1u << WLFC_MODE_REORDERSUPP_SHIFT)) | \
	(((val) & 1u) << WLFC_MODE_REORDERSUPP_SHIFT))
/** returns TRUE if 'reorder suppress' has been agreed upon between host and dongle */
#define WLFC_GET_REORDERSUPP(x)	(((x) >> WLFC_MODE_REORDERSUPP_SHIFT) & 1)

#define FLOW_RING_CREATE             1u
#define FLOW_RING_DELETE             2u
#define FLOW_RING_FLUSH              3u
#define FLOW_RING_OPEN               4u
#define FLOW_RING_CLOSED             5u
#define FLOW_RING_FLUSHED            6u
#define FLOW_RING_TIM_SET            7u
#define FLOW_RING_TIM_RESET          8u
#define FLOW_RING_FLUSH_TXFIFO       9u
#define FLOW_RING_GET_PKT_MAX        10u
#define FLOW_RING_RESET_WEIGHT       11u
#define FLOW_RING_UPD_PRIOMAP        12u
#define FLOW_RING_HP2P_CREATE        13u
#define FLOW_RING_HP2P_DELETE        14u
#define FLOW_RING_GET_BUFFERED_TIME  15u
#define FLOW_RING_HP2P_TXQ_STRT      16u
#define FLOW_RING_HP2P_TXQ_STOP      17u
#define FLOW_RING_GET_TXPARAMS       18u

/* bit 7, indicating if is TID(1) or AC(0) mapped info in tid field) */
#define PCIEDEV_IS_AC_TID_MAP_MASK	0x80

#define WLFC_PCIEDEV_AC_PRIO_MAP	 0
#define WLFC_PCIEDEV_TID_PRIO_MAP     1
#define WLFC_PCIEDEV_LLR_PRIO_MAP	2

void wlc_wlfc_set_pkttime(void* pkt, uint16 time);

/* reason for disabling APP, when none are set, APP will be enabled */
typedef enum {
	APP_STS_FLOWRING_NO_APP		= 0u,	/* Reason code used by pciedev */
	APP_STS_FLOWRING_CLOSED		= 1u,	/* Disable APP as flowring is closed */
	APP_STS_CRYPTO_UNSUPPORTED	= 2u,	/* Secuirity type doesn't support APP */
	APP_STS_80211_FRAGMENTATION	= 3u,   /* 802.11 fragmentation enabled */
	APP_STS_DISABLE_FOR_BTCX	= 4u,	/* BTCX requested APP disable */
	APP_STS_MAX			= 5u	/* MAX */
} app_disable_reason_s;

/* shared structure between wlc and pciedev layer to set/reset a reason code */
typedef struct app_upd_sts {
	bool	set;			/* if set, app is disabled for reason rsn */
	bool	sta;			/* set if scb/flowring  belong to sta */
	app_disable_reason_s rsn;	/* APP disable reason codes. */
} app_upd_sts_t;

#endif /* __wlfc_proto_definitions_h__ */
