/*
 * Linux cfg80211 driver - Android related functions
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

#include <linux/module.h>
#include <linux/netdevice.h>
#include <wldev_common.h>

/* If any feature uses the Generic Netlink Interface, put it here to enable WL_GENL
 * automatically
 */
#if defined(WL_SDO)
#define WL_GENL
#endif

#ifdef WL_GENL
#include <net/genetlink.h>
#endif

#if !defined(WL_MBO_IOV_VERSION)
/* MBO IOV API version */
#define WL_MBO_IOV_VERSION WL_MBO_IOV_VERSION_1_1
#endif

typedef struct _android_wifi_priv_cmd {
    char *buf;
    int used_len;
    int total_len;
} android_wifi_priv_cmd;

#ifdef CONFIG_COMPAT
typedef struct _compat_android_wifi_priv_cmd {
    compat_caddr_t buf;
    int used_len;
    int total_len;
} compat_android_wifi_priv_cmd;
#endif /* CONFIG_COMPAT */

/**
 * Android platform dependent functions, feel free to add Android specific functions here
 * (save the macros in dhd). Please do NOT declare functions that are NOT exposed to dhd
 * or cfg, define them as static in wl_android.c
 */

/**
 * wl_android_init will be called from module init function (dhd_module_init now), similarly
 * wl_android_exit will be called from module exit function (dhd_module_cleanup now)
 */
int wl_android_init(void);
int wl_android_exit(void);
void wl_android_post_init(void);
void wl_android_set_wifi_on_flag(bool enable);
#if defined(WLAN_ACCEL_BOOT)
int wl_android_wifi_accel_on(struct net_device *dev, bool force_reg_on);
int wl_android_wifi_accel_off(struct net_device *dev, bool force_reg_on);
#endif /* WLAN_ACCEL_BOOT */
int wl_android_wifi_on(struct net_device *dev);
int wl_android_wifi_off(struct net_device *dev, bool on_failure);
int wl_android_priv_cmd(struct net_device *net, struct ifreq *ifr);
int wl_handle_private_cmd(struct net_device *net, char *command, u32 cmd_len);
int wl_android_set_spect(struct net_device *dev, int spect);

#ifdef WL_GENL
typedef struct bcm_event_hdr {
	u16 event_type;
	u16 len;
} bcm_event_hdr_t;

/* attributes (variables): the index in this enum is used as a reference for the type,
 *             userspace application has to indicate the corresponding type
 *             the policy is used for security considerations
 */
enum {
	BCM_GENL_ATTR_UNSPEC,
	BCM_GENL_ATTR_STRING,
	BCM_GENL_ATTR_MSG,
	__BCM_GENL_ATTR_MAX
};
#define BCM_GENL_ATTR_MAX (__BCM_GENL_ATTR_MAX - 1)

/* commands: enumeration of all commands (functions),
 * used by userspace application to identify command to be ececuted
 */
enum {
	BCM_GENL_CMD_UNSPEC,
	BCM_GENL_CMD_MSG,
	__BCM_GENL_CMD_MAX
};
#define BCM_GENL_CMD_MAX (__BCM_GENL_CMD_MAX - 1)

/* Enum values used by the BCM supplicant to identify the events */
enum {
	BCM_E_UNSPEC,
	BCM_E_SVC_FOUND,
	BCM_E_DEV_FOUND,
	BCM_E_DEV_LOST,
	BCM_E_MAX
};

s32 wl_genl_send_msg(struct net_device *ndev, u32 event_type,
	const u8 *string, u16 len, u8 *hdr, u16 hdrlen);
#endif /* WL_GENL */
s32 wl_netlink_send_msg(int pid, int type, int seq, const void *data, size_t size);

/* hostap mac mode */
#define MACLIST_MODE_DISABLED   0
#define MACLIST_MODE_DENY       1
#define MACLIST_MODE_ALLOW      2

/* max number of assoc list */
#define MAX_NUM_OF_ASSOCLIST    64

/* Bandwidth */
#define WL_CH_BANDWIDTH_20MHZ 20
#define WL_CH_BANDWIDTH_40MHZ 40
#define WL_CH_BANDWIDTH_80MHZ 80
#define WL_CH_BANDWIDTH_160MHZ 160

/* max number of mac filter list
 * restrict max number to 10 as maximum cmd string size is 255
 */
#define MAX_NUM_MAC_FILT        10
#define	WL_GET_BAND(ch)	(((uint)(ch) <= CH_MAX_2G_CHANNEL) ?	\
	WLC_BAND_2G : WLC_BAND_5G)

/* SoftAP auto channel feature */
#define APCS_BAND_2G_LEGACY1	20
#define APCS_BAND_2G_LEGACY2	0
#define APCS_BAND_AUTO		"band=auto"
#define APCS_BAND_2G		"band=2g"
#define APCS_BAND_5G		"band=5g"
#define APCS_BAND_6G		"band=6g"
#define FREQ_STR		"freq="
#define APCS_MAX_2G_CHANNELS	11
#define APCS_MAX_RETRY		10
#define APCS_DEFAULT_2G_CH	1
#define APCS_DEFAULT_5G_CH	149
#define APCS_DEFAULT_6G_CH	5

extern int wl_android_set_whitelist_ssid(struct net_device *dev,
	wl_ssid_whitelist_t *ssid_whitelist, uint32 len, uint32 flush);
extern int wl_android_set_blacklist_bssid(struct net_device *dev, maclist_t *blacklist,
    uint32 len, uint32 flush);
int wl_android_set_ap_mac_list(struct net_device *dev, int macmode, struct maclist *maclist);
#ifdef WL_BCNRECV
extern int wl_android_bcnrecv_config(struct net_device *ndev, char *data,
		int total_len);
extern int wl_android_bcnrecv_stop(struct net_device *ndev, uint reason);
extern int wl_android_bcnrecv_resume(struct net_device *ndev);
extern int wl_android_bcnrecv_suspend(struct net_device *ndev);
extern int wl_android_bcnrecv_event(struct net_device *ndev,
		uint attr_type, uint status, uint reason, uint8 *data, uint data_len);
#endif /* WL_BCNRECV */
#ifdef WL_CAC_TS
#define TSPEC_UPLINK_DIRECTION (0 << 5)	/* uplink direction traffic stream */
#define TSPEC_DOWNLINK_DIRECTION (1 << 5)	/* downlink direction traffic stream */
#define TSPEC_BI_DIRECTION (3 << 5)	/* bi direction traffic stream */
#define TSPEC_EDCA_ACCESS (1 << 7)	/* EDCA access policy */
#define TSPEC_UAPSD_PSB (1 << 2)		/* U-APSD power saving behavior */
#define TSPEC_TSINFO_TID_SHIFT 1		/* TID Shift */
#define TSPEC_TSINFO_PRIO_SHIFT 3		/* PRIO Shift */
#define TSPEC_MAX_ACCESS_CATEGORY 3
#define TSPEC_MAX_USER_PRIO	7
#define TSPEC_MAX_DIALOG_TOKEN	255
#define TSPEC_MAX_SURPLUS_BW 12410
#define TSPEC_MIN_SURPLUS_BW 11210
#define TSPEC_MAX_MSDU_SIZE 1520
#define TSPEC_DEF_MEAN_DATA_RATE 120000
#define TSPEC_DEF_MIN_PHY_RATE 6000000
#define TSPEC_DEF_DIALOG_TOKEN 7
#endif /* WL_CAC_TS */

#ifdef WL_SUPPORT_AUTO_CHANNEL
#define WLC_ACS_BAND_INVALID	0xffffu
#endif /* WL_SUPPORT_AUTO_CHANNEL */
#define WL_PRIV_CMD_LEN 64
#define CHECK_SCNPRINTF_RET_VAL(ret) \
	{ \
		if (ret < 0) { \
				WL_ERR(("scnprintf failed %d\n", ret)); \
				return BCME_ERROR; \
		} \
	}

#if defined(CUSTOM_CONTROL_HE_6G_FEATURES)
extern int wl_android_set_he_6g_band(struct net_device *dev, bool enable);
#endif /* CUSTOM_CONTROL_HE_6G_FEATURES */
extern int wl_android_rcroam_turn_on(struct net_device *dev, int rcroam_enab);
