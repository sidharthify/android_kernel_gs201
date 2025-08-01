/*
 * Linux cfgp2p driver
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
 * <<Broadcom-WL-IPTag/Dual:>>
 */
#ifndef _wl_cfgp2p_h_
#define _wl_cfgp2p_h_
#include <802.11.h>
#include <p2p.h>

struct bcm_cfg80211;
extern u32 wl_dbg_level;
extern u32 wl_log_level;

typedef struct wifi_p2p_ie wifi_wfd_ie_t;
/* Enumeration of the usages of the BSSCFGs used by the P2P Library.  Do not
 * confuse this with a bsscfg index.  This value is an index into the
 * saved_ie[] array of structures which in turn contains a bsscfg index field.
 */
typedef enum {
	P2PAPI_BSSCFG_PRIMARY, /**< maps to driver's primary bsscfg */
	P2PAPI_BSSCFG_DEVICE, /**< maps to driver's P2P device discovery bsscfg */
	P2PAPI_BSSCFG_CONNECTION1, /**< maps to driver's P2P connection bsscfg */
	P2PAPI_BSSCFG_CONNECTION2,
	P2PAPI_BSSCFG_MAX
} p2p_bsscfg_type_t;

typedef enum {
	P2P_SCAN_PURPOSE_MIN,
	P2P_SCAN_SOCIAL_CHANNEL, /**< scan for social channel */
	P2P_SCAN_AFX_PEER_NORMAL, /**< scan for action frame search */
	P2P_SCAN_AFX_PEER_REDUCED, /**< scan for action frame search with short time */
	P2P_SCAN_DURING_CONNECTED, /**< scan during connected status */
	P2P_SCAN_CONNECT_TRY, /**< scan for connecting */
	P2P_SCAN_NORMAL, /**< scan during not-connected status */
	P2P_SCAN_PURPOSE_MAX
} p2p_scan_purpose_t;

/** vendor ies max buffer length for probe response or beacon */
#define VNDR_IES_MAX_BUF_LEN	1400
/** normal vendor ies buffer length */
#define VNDR_IES_BUF_LEN 		512

struct p2p_bss {
	s32 bssidx;
	struct net_device *dev;
	void *private_data;
	struct ether_addr mac_addr;
};

struct p2p_info {
	bool on;    /**< p2p on/off switch */
	bool scan;
	int16 search_state;
	s8 vir_ifname[IFNAMSIZ];
	unsigned long status;
	struct p2p_bss bss[P2PAPI_BSSCFG_MAX];
	timer_list_compat_t listen_timer;
	wl_p2p_sched_t noa;
	wl_p2p_ops_t ops;
	wlc_ssid_t ssid;
	s8 p2p_go_count;
};

#define MAX_VNDR_IE_NUMBER	10

struct parsed_vndr_ie_info {
	const char *ie_ptr;
	u32 ie_len;	/**< total length including id & length field */
	vndr_ie_t vndrie;
};

struct parsed_vndr_ies {
	u32 count;
	struct parsed_vndr_ie_info ie_info[MAX_VNDR_IE_NUMBER];
};

/* dongle status */
enum wl_cfgp2p_status {
	WLP2P_STATUS_DISCOVERY_ON = 0,
	WLP2P_STATUS_SEARCH_ENABLED,
	WLP2P_STATUS_IF_ADDING,
	WLP2P_STATUS_IF_DELETING,
	WLP2P_STATUS_IF_CHANGING,
	WLP2P_STATUS_IF_CHANGED,
	WLP2P_STATUS_LISTEN_EXPIRED,
	WLP2P_STATUS_ACTION_TX_COMPLETED,
	WLP2P_STATUS_ACTION_TX_NOACK,
	WLP2P_STATUS_SCANNING,
	WLP2P_STATUS_GO_NEG_PHASE,
	WLP2P_STATUS_DISC_IN_PROGRESS
};

#define wl_to_p2p_bss_ndev(cfg, type)		((cfg)->p2p->bss[type].dev)
#define wl_to_p2p_bss_bssidx(cfg, type)		((cfg)->p2p->bss[type].bssidx)
#define wl_to_p2p_bss_macaddr(cfg, type)     &((cfg)->p2p->bss[type].mac_addr)
#define wl_to_p2p_bss_saved_ie(cfg, type)	((cfg)->p2p->bss[type].saved_ie)
#define wl_to_p2p_bss_private(cfg, type)		((cfg)->p2p->bss[type].private_data)
#define wl_to_p2p_bss(cfg, type)			((cfg)->p2p->bss[type])
#define wl_get_p2p_status(cfg, stat) ((!(cfg)->p2p_supported) ? 0 : \
		test_bit(WLP2P_STATUS_ ## stat, &(cfg)->p2p->status))
#define wl_set_p2p_status(cfg, stat) ((!(cfg)->p2p_supported) ? 0 : \
		set_bit(WLP2P_STATUS_ ## stat, &(cfg)->p2p->status))
#define wl_clr_p2p_status(cfg, stat) ((!(cfg)->p2p_supported) ? 0 : \
		clear_bit(WLP2P_STATUS_ ## stat, &(cfg)->p2p->status))
#define wl_chg_p2p_status(cfg, stat) ((!(cfg)->p2p_supported) ? 0 : \
	change_bit(WLP2P_STATUS_ ## stat, &(cfg)->p2p->status))
#define p2p_on(cfg) ((cfg)->p2p->on)
#define p2p_scan(cfg) ((cfg)->p2p->scan)
#define p2p_is_on(cfg) ((cfg)->p2p && (cfg)->p2p->on)

#if defined(CUSTOMER_DBG_PREFIX_ENABLE)
#define USER_PREFIX_CFGP2P		"[cfgp2p][wlan] "
#define CFGP2P_ERROR_TEXT		USER_PREFIX_CFGP2P
#define CFGP2P_INFO_TEXT		USER_PREFIX_CFGP2P
#define CFGP2P_ACTION_TEXT		USER_PREFIX_CFGP2P
#define CFGP2P_DEBUG_TEXT		USER_PREFIX_CFGP2P
#else
#ifdef CUSTOMER_HW4_DEBUG
#define CFGP2P_ERROR_TEXT		"CFGP2P-INFO2) "
#else
#define CFGP2P_ERROR_TEXT		"CFGP2P-ERROR) "
#endif /* CUSTOMER_HW4_DEBUG */
#define CFGP2P_INFO_TEXT		"CFGP2P-INFO) "
#define CFGP2P_ACTION_TEXT		"CFGP2P-ACTION) "
#define CFGP2P_DEBUG_TEXT		"CFGP2P-DEBUG) "
#endif /* defined(CUSTOMER_DBG_PREFIX_ENABLE) */

#ifdef DHD_LOG_DUMP
#define CFGP2P_ERR(args)						\
	do {								\
		if (wl_dbg_level & WL_DBG_ERR) {			\
			WL_DBG_PRINT_SYSTEM_TIME;			\
			pr_cont(CFGP2P_ERROR_TEXT "%s : ", __func__);	\
			pr_cont args;					\
		}							\
		if (wl_log_level & WL_DBG_ERR) {			\
			DHD_LOG_DUMP_WRITE_TS_FN;			\
			DHD_LOG_DUMP_WRITE args;			\
		}							\
	} while (0)
#define CFGP2P_INFO(args)						\
	do {								\
		if (wl_dbg_level & WL_DBG_INFO) {			\
			WL_DBG_PRINT_SYSTEM_TIME;			\
			pr_cont(CFGP2P_INFO_TEXT "%s : ", __func__);	\
			pr_cont args;					\
		}							\
		if (wl_log_level & WL_DBG_INFO) {			\
			DHD_LOG_DUMP_WRITE_TS_FN;			\
			DHD_LOG_DUMP_WRITE args;			\
		}							\
	} while (0)
#define CFGP2P_ACTION(args)						\
	do {								\
		if (wl_dbg_level & WL_DBG_P2P_ACTION) {			\
			WL_DBG_PRINT_SYSTEM_TIME;			\
			pr_cont(CFGP2P_ACTION_TEXT "%s :", __func__);	\
			pr_cont args;					\
		}							\
		if (wl_log_level & WL_DBG_P2P_ACTION) {			\
			DHD_LOG_DUMP_WRITE_TS_FN;			\
			DHD_LOG_DUMP_WRITE args;			\
		}							\
	} while (0)
#else
#define CFGP2P_ERR(args)									\
	do {										\
		if (wl_dbg_level & WL_DBG_ERR) {				\
			WL_DBG_PRINT_SYSTEM_TIME;				\
			pr_cont(CFGP2P_ERROR_TEXT "%s : ", __func__);	\
			pr_cont args;					\
		}									\
	} while (0)
#define	CFGP2P_INFO(args)									\
	do {										\
		if (wl_dbg_level & WL_DBG_INFO) {				\
			WL_DBG_PRINT_SYSTEM_TIME;				\
			pr_cont(CFGP2P_INFO_TEXT "%s : ", __func__);	\
			pr_cont args;					\
		}									\
	} while (0)
#define	CFGP2P_ACTION(args)								\
	do {									\
		if (wl_dbg_level & WL_DBG_P2P_ACTION) {			\
			WL_DBG_PRINT_SYSTEM_TIME;				\
			pr_cont(CFGP2P_ACTION_TEXT "%s :", __func__);	\
			pr_cont args;					\
		}									\
	} while (0)
#endif /* DHD_LOG_DUMP */

#define	CFGP2P_DBG(args)								\
	do {									\
		if (wl_dbg_level & WL_DBG_DBG) {			\
			WL_DBG_PRINT_SYSTEM_TIME;				\
			pr_cont(CFGP2P_DEBUG_TEXT "%s :", __func__);	\
			pr_cont args;					\
		}									\
	} while (0)

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 8, 0)) && !defined(WL_CFG80211_P2P_DEV_IF)
#define WL_CFG80211_P2P_DEV_IF

#ifdef WL_SUPPORT_BACKPORTED_KPATCHES
#undef WL_SUPPORT_BACKPORTED_KPATCHES
#endif
#else
#ifdef WLP2P
/* Enable P2P network Interface if P2P support is enabled */
#define WL_ENABLE_P2P_IF
#endif /* WLP2P */
#endif /* (LINUX_VERSION >= VERSION(3, 8, 0)) */

#ifndef WL_CFG80211_P2P_DEV_IF
#ifdef WL_NEWCFG_PRIVCMD_SUPPORT
#undef WL_NEWCFG_PRIVCMD_SUPPORT
#endif
#endif /* WL_CFG80211_P2P_DEV_IF */

#if !defined(WLP2P) && defined(WL_CFG80211_P2P_DEV_IF)
#error WLP2P not defined
#endif

#if defined(WL_CFG80211_P2P_DEV_IF)
#define bcm_struct_cfgdev	struct wireless_dev
#else
#define bcm_struct_cfgdev	struct net_device
#endif /* WL_CFG80211_P2P_DEV_IF */

/* If we take 10 or 30 as count value, operation
 * may failed due to full scan and noisy environments.
 * So, we choose 50 as the optimum value for P2P ECSA.
 */
#define P2P_ECSA_CNT 50

extern void
wl_cfgp2p_listen_expired(unsigned long data);
extern bool
wl_cfgp2p_is_pub_action(void *frame, u32 frame_len);
extern bool
wl_cfgp2p_is_p2p_action(void *frame, u32 frame_len);
extern bool
wl_cfgp2p_is_gas_action(void *frame, u32 frame_len);
extern bool
wl_cfgp2p_is_p2p_gas_action(void *frame, u32 frame_len);
extern void
wl_cfgp2p_print_actframe(bool tx, void *frame, u32 frame_len, u32 channel);
extern s32
wl_cfgp2p_init_priv(struct bcm_cfg80211 *cfg);
extern void
wl_cfgp2p_deinit_priv(struct bcm_cfg80211 *cfg);
extern s32
wl_cfgp2p_set_firm_p2p(struct bcm_cfg80211 *cfg);
extern s32
wl_cfgp2p_set_p2p_mode(struct bcm_cfg80211 *cfg, u8 mode,
            u32 channel, u16 listen_ms, int bssidx);
extern s32
wl_cfgp2p_ifadd(struct bcm_cfg80211 *cfg, struct ether_addr *mac, u8 if_type,
            chanspec_t chspec);
extern s32
wl_cfgp2p_ifdisable(struct bcm_cfg80211 *cfg, struct ether_addr *mac);
extern s32
wl_cfgp2p_ifdel(struct bcm_cfg80211 *cfg, struct ether_addr *mac);
extern s32
wl_cfgp2p_ifchange(struct bcm_cfg80211 *cfg, struct ether_addr *mac, u8 if_type,
	chanspec_t chspec, s32 conn_idx);

extern s32
wl_cfgp2p_ifidx(struct bcm_cfg80211 *cfg, struct ether_addr *mac, s32 *index);

extern s32
wl_cfgp2p_init_discovery(struct bcm_cfg80211 *cfg);
extern s32
wl_cfgp2p_enable_discovery(struct bcm_cfg80211 *cfg, struct net_device *dev, const u8 *ie,
	u32 ie_len);
extern s32
wl_cfgp2p_disable_discovery(struct bcm_cfg80211 *cfg);
extern s32
wl_cfgp2p_escan(struct bcm_cfg80211 *cfg, struct net_device *dev, u16 active, u32 num_chans,
	u16 *channels,
	s32 search_state, u16 action, u32 bssidx, struct ether_addr *tx_dst_addr,
	p2p_scan_purpose_t p2p_scan_purpose);

extern s32
wl_cfgp2p_act_frm_search(struct bcm_cfg80211 *cfg, struct net_device *ndev,
	s32 bssidx, s32 channel, struct ether_addr *tx_dst_addr);

extern const wpa_ie_fixed_t *
wl_cfgp2p_find_wpaie(const u8 *parse, u32 len);

extern const wpa_ie_fixed_t *
wl_cfgp2p_find_wpsie(const u8 *parse, u32 len);

extern wifi_p2p_ie_t *
wl_cfgp2p_find_p2pie(const u8 *parse, u32 len);

extern const wifi_wfd_ie_t *
wl_cfgp2p_find_wfdie(const u8 *parse, u32 len);
extern s32
wl_cfgp2p_set_management_ie(struct bcm_cfg80211 *cfg, struct net_device *ndev, s32 bssidx,
            s32 pktflag, const u8 *vndr_ie, u32 vndr_ie_len);
extern s32
wl_cfgp2p_clear_management_ie(struct bcm_cfg80211 *cfg, s32 bssidx);

extern struct net_device *
wl_cfgp2p_find_ndev(struct bcm_cfg80211 *cfg, s32 bssidx);
extern s32
wl_cfgp2p_find_type(struct bcm_cfg80211 *cfg, s32 bssidx, s32 *type);

extern s32
wl_cfgp2p_listen_complete(struct bcm_cfg80211 *cfg, bcm_struct_cfgdev *cfgdev,
	const wl_event_msg_t *e, void *data);
extern s32
wl_cfgp2p_discover_listen(struct bcm_cfg80211 *cfg, s32 channel, u32 duration_ms);

extern s32
wl_cfgp2p_discover_enable_search(struct bcm_cfg80211 *cfg, u8 enable);

extern s32
wl_cfgp2p_action_tx_complete(struct bcm_cfg80211 *cfg, bcm_struct_cfgdev *cfgdev,
	const wl_event_msg_t *e, void *data);

extern s32
wl_cfgp2p_tx_action_frame(struct bcm_cfg80211 *cfg, bcm_struct_cfgdev *cfgdev,
	struct net_device *dev, wl_af_params_v1_t *af_params, s32 bssidx, const u8 *sa);

extern void
wl_cfgp2p_generate_bss_mac(struct bcm_cfg80211 *cfg);

extern void
wl_cfg80211_change_ifaddr(u8* buf, struct ether_addr *p2p_int_addr, u8 element_id);

extern s32
wl_cfgp2p_supported(struct bcm_cfg80211 *cfg, struct net_device *ndev);

extern s32
wl_cfgp2p_down(struct bcm_cfg80211 *cfg);

extern s32
wl_cfgp2p_set_p2p_noa(struct bcm_cfg80211 *cfg, struct net_device *ndev, char* buf, int len);

extern s32
wl_cfgp2p_get_p2p_noa(struct bcm_cfg80211 *cfg, struct net_device *ndev, char* buf, int len);

extern s32
wl_cfgp2p_set_p2p_ps(struct bcm_cfg80211 *cfg, struct net_device *ndev, char* buf, int len);

extern s32
wl_cfgp2p_set_p2p_ecsa(struct bcm_cfg80211 *cfg, struct net_device *ndev, char* buf, int len);

extern s32
wl_cfgp2p_increase_p2p_bw(struct bcm_cfg80211 *cfg, struct net_device *ndev, char* buf, int len);

extern const u8 *
wl_cfgp2p_retreive_p2pattrib(const void *buf, u8 element_id);

extern const u8*
wl_cfgp2p_find_attrib_in_all_p2p_Ies(const u8 *parse, u32 len, u32 attrib);

extern const u8 *
wl_cfgp2p_retreive_p2p_dev_addr(wl_bss_info_v109_t *bi, u32 bi_length);

extern s32
wl_cfgp2p_register_ndev(struct bcm_cfg80211 *cfg);

extern s32
wl_cfgp2p_unregister_ndev(struct bcm_cfg80211 *cfg);

extern bool
wl_cfgp2p_is_ifops(const struct net_device_ops *if_ops);

extern u32
wl_cfgp2p_vndr_ie(struct bcm_cfg80211 *cfg, u8 *iebuf, s32 pktflag,
                  s8 *oui, s32 ie_id, const s8 *data, s32 datalen, const s8* add_del_cmd);

extern int wl_cfgp2p_get_conn_idx(struct bcm_cfg80211 *cfg);

extern
int wl_cfg_multip2p_operational(struct bcm_cfg80211 *cfg);

extern
int wl_cfgp2p_vif_created(struct bcm_cfg80211 *cfg);

#if defined(WL_CFG80211_P2P_DEV_IF)
extern struct wireless_dev *
wl_cfgp2p_add_p2p_disc_if(struct bcm_cfg80211 *cfg);

extern int
wl_cfgp2p_start_p2p_device(struct wiphy *wiphy, struct wireless_dev *wdev);

extern void
wl_cfgp2p_stop_p2p_device(struct wiphy *wiphy, struct wireless_dev *wdev);

extern int
wl_cfgp2p_del_p2p_disc_if(struct wireless_dev *wdev, struct bcm_cfg80211 *cfg);

#endif /* WL_CFG80211_P2P_DEV_IF */

extern void
wl_cfgp2p_need_wait_actfrmae(struct bcm_cfg80211 *cfg, void *frame, u32 frame_len, bool tx);

extern int
wl_cfgp2p_is_p2p_specific_scan(struct cfg80211_scan_request *request);

extern s32
wl_cfg80211_abort_action_frame(struct bcm_cfg80211 *cfg, struct net_device *dev, s32 bssidx);

/* WiFi Direct */
#define SOCIAL_CHAN_1 1
#define SOCIAL_CHAN_2 6
#define SOCIAL_CHAN_3 11
#define IS_P2P_SOCIAL_CHANNEL(channel) ((channel == SOCIAL_CHAN_1) || \
					(channel == SOCIAL_CHAN_2) || \
					(channel == SOCIAL_CHAN_3))
#define SOCIAL_CHAN_CNT 3
#define AF_PEER_SEARCH_CNT 2
#define WL_P2P_WILDCARD_SSID "DIRECT-"
#define WL_P2P_WILDCARD_SSID_LEN 7
#define WL_P2P_INTERFACE_PREFIX "p2p"
#define WL_P2P_TEMP_CHAN 11
#define WL_P2P_TEMP_CHAN_5G 36
#define WL_P2P_AF_STATUS_OFFSET 9

/* If the provision discovery is for JOIN operations,
 * or the device discoverablity frame is destined to GO
 * then we need not do an internal scan to find GO.
 */
#define IS_ACTPUB_WITHOUT_GROUP_ID(p2p_ie, len) \
	(wl_cfgp2p_retreive_p2pattrib(p2p_ie, P2P_SEID_GROUP_ID) == NULL)

#define IS_GAS_REQ(frame, len) (wl_cfgp2p_is_gas_action(frame, len) && \
					((frame->action == P2PSD_ACTION_ID_GAS_IREQ) || \
					(frame->action == P2PSD_ACTION_ID_GAS_CREQ)))

#define IS_P2P_PUB_ACT_RSP_SUBTYPE(subtype) ((subtype == P2P_PAF_GON_RSP) || \
							((subtype == P2P_PAF_GON_CONF) || \
							(subtype == P2P_PAF_INVITE_RSP) || \
							(subtype == P2P_PAF_PROVDIS_RSP)))
#define IS_P2P_SOCIAL(ch) ((ch == SOCIAL_CHAN_1) || (ch == SOCIAL_CHAN_2) || (ch == SOCIAL_CHAN_3))
#define IS_P2P_SSID(ssid, len) (!memcmp(ssid, WL_P2P_WILDCARD_SSID, WL_P2P_WILDCARD_SSID_LEN) && \
					(len >= WL_P2P_WILDCARD_SSID_LEN))

/* Min FW ver required to support chanspec
 * instead of channel in actframe iovar.
 */
#define FW_MAJOR_VER_ACTFRAME_CHSPEC    14
#endif				/* _wl_cfgp2p_h_ */
