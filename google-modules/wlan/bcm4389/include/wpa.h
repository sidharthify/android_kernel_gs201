/*
 * Fundamental types and constants relating to WPA
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

#ifndef _proto_wpa_h_
#define _proto_wpa_h_

#include <typedefs.h>
#include <ethernet.h>

/* This marks the start of a packed structure section. */
#include <packed_section_start.h>

/* Reason Codes */

/* 13 through 23 taken from IEEE Std 802.11i-2004 */
#define DOT11_RC_INVALID_WPA_IE		13	/* Invalid info. element */
#define DOT11_RC_MIC_FAILURE		14	/* Michael failure */
#define DOT11_RC_4WH_TIMEOUT		15	/* 4-way handshake timeout */
#define DOT11_RC_GTK_UPDATE_TIMEOUT	16	/* Group key update timeout */
#define DOT11_RC_WPA_IE_MISMATCH	17	/* WPA IE in 4-way handshake differs from
						 * (re-)assoc. request/probe response
						 */
#define DOT11_RC_INVALID_MC_CIPHER	18	/* Invalid multicast cipher */
#define DOT11_RC_INVALID_UC_CIPHER	19	/* Invalid unicast cipher */
#define DOT11_RC_INVALID_AKMP		20	/* Invalid authenticated key management protocol */
#define DOT11_RC_BAD_WPA_VERSION	21	/* Unsupported WPA version */
#define DOT11_RC_INVALID_WPA_CAP	22	/* Invalid WPA IE capabilities */
#define DOT11_RC_8021X_AUTH_FAIL	23	/* 802.1X authentication failure */

#define WPA2_PMKID_LEN	16

/* WPA IE fixed portion */
typedef BWL_PRE_PACKED_STRUCT struct
{
	uint8 tag;	/* TAG */
	uint8 length;	/* TAG length */
	uint8 oui[3];	/* IE OUI */
	uint8 oui_type;	/* OUI type */
	BWL_PRE_PACKED_STRUCT struct {
		uint8 low;
		uint8 high;
	} BWL_POST_PACKED_STRUCT version;	/* IE version */
} BWL_POST_PACKED_STRUCT wpa_ie_fixed_t;
#define WPA_IE_OUITYPE_LEN	4
#define WPA_IE_FIXED_LEN	8
#define WPA_IE_TAG_FIXED_LEN	6

#define BIP_OUI_TYPE WPA2_OUI "\x06"

typedef BWL_PRE_PACKED_STRUCT struct {
	uint8 tag;	/* TAG */
	uint8 length;	/* TAG length */
	BWL_PRE_PACKED_STRUCT struct {
		uint8 low;
		uint8 high;
	} BWL_POST_PACKED_STRUCT version;	/* IE version */
} BWL_POST_PACKED_STRUCT wpa_rsn_ie_fixed_t;
#define WPA_RSN_IE_FIXED_LEN	4
#define WPA_RSN_IE_TAG_FIXED_LEN	2
typedef uint8 wpa_pmkid_t[WPA2_PMKID_LEN];

#define WFA_OSEN_IE_FIXED_LEN	6

/* WPA suite/multicast suite */
typedef BWL_PRE_PACKED_STRUCT struct
{
	uint8 oui[3];
	uint8 type;
} BWL_POST_PACKED_STRUCT wpa_suite_t, wpa_suite_mcast_t;
#define WPA_SUITE_LEN	4

/* WPA unicast suite list/key management suite list */
typedef BWL_PRE_PACKED_STRUCT struct
{
	BWL_PRE_PACKED_STRUCT struct {
		uint8 low;
		uint8 high;
	} BWL_POST_PACKED_STRUCT count;
	wpa_suite_t list[BCM_FLEX_ARRAY];
} BWL_POST_PACKED_STRUCT wpa_suite_ucast_t, wpa_suite_auth_key_mgmt_t;
#define WPA_IE_SUITE_COUNT_LEN	2
typedef BWL_PRE_PACKED_STRUCT struct
{
	BWL_PRE_PACKED_STRUCT struct {
		uint8 low;
		uint8 high;
	} BWL_POST_PACKED_STRUCT count;
	wpa_pmkid_t list[BCM_FLEX_ARRAY];
} BWL_POST_PACKED_STRUCT wpa_pmkid_list_t;

/* WPA cipher suites */
#define WPA_CIPHER_NONE		0	/* None */
#define WPA_CIPHER_WEP_40	1	/* WEP (40-bit) */
#define WPA_CIPHER_TKIP		2	/* TKIP: default for WPA */
#define WPA_CIPHER_AES_OCB	3	/* AES (OCB) */
#define WPA_CIPHER_AES_CCM	4	/* AES (CCM) */
#define WPA_CIPHER_WEP_104	5	/* WEP (104-bit) */
#define WPA_CIPHER_BIP		6	/* WEP (104-bit) */
#define WPA_CIPHER_TPK		7	/* Group addressed traffic not allowed */
#ifdef BCMCCX
#define WPA_CIPHER_CKIP		8	/* KP with no MIC */
#define WPA_CIPHER_CKIP_MMH	9	/* KP with MIC ("CKIP/MMH", "CKIP+CMIC") */
#define WPA_CIPHER_WEP_MMH	10	/* MIC with no KP ("WEP/MMH", "CMIC") */

#define IS_CCX_CIPHER(cipher)	((cipher) == WPA_CIPHER_CKIP || \
				 (cipher) == WPA_CIPHER_CKIP_MMH || \
				 (cipher) == WPA_CIPHER_WEP_MMH)
#endif /* BCMCCX */

#define WPA_CIPHER_AES_GCM	8	/* AES (GCM) */
#define WPA_CIPHER_AES_GCM256	9	/* AES (GCM256) */
#define WPA_CIPHER_CCMP_256	10	/* CCMP-256 */
#define WPA_CIPHER_BIP_GMAC_128	11	/* BIP_GMAC_128 */
#define WPA_CIPHER_BIP_GMAC_256 12	/* BIP_GMAC_256 */
#define WPA_CIPHER_BIP_CMAC_256 13	/* BIP_CMAC_256 */

#define IS_WPA_CIPHER(cipher)	((cipher) == WPA_CIPHER_NONE || \
				 (cipher) == WPA_CIPHER_WEP_40 || \
				 (cipher) == WPA_CIPHER_WEP_104 || \
				 (cipher) == WPA_CIPHER_TKIP || \
				 (cipher) == WPA_CIPHER_AES_OCB || \
				 (cipher) == WPA_CIPHER_AES_CCM || \
				 (cipher) == WPA_CIPHER_AES_GCM || \
				 (cipher) == WPA_CIPHER_AES_GCM256 || \
				 (cipher) == WPA_CIPHER_CCMP_256 || \
				 (cipher) == WPA_CIPHER_TPK)

#define IS_WPA_BIP_CIPHER(cipher)  ((cipher) == WPA_CIPHER_BIP || \
				    (cipher) == WPA_CIPHER_BIP_GMAC_128 || \
				    (cipher) == WPA_CIPHER_BIP_GMAC_256 || \
				    (cipher) == WPA_CIPHER_BIP_CMAC_256)

#define IS_VALID_AKM(akm) ((akm) == RSN_AKM_NONE || \
			(akm) == RSN_AKM_UNSPECIFIED || \
			(akm) == RSN_AKM_PSK || \
			(akm) == RSN_AKM_FBT_1X || \
			(akm) == RSN_AKM_FBT_PSK || \
			(akm) == RSN_AKM_MFP_1X || \
			(akm) == RSN_AKM_MFP_PSK || \
			(akm) == RSN_AKM_SHA256_1X || \
			(akm) == RSN_AKM_SHA256_PSK || \
			(akm) == RSN_AKM_TPK || \
			(akm) == RSN_AKM_SAE_PSK || \
			(akm) == RSN_AKM_SAE_FBT || \
			(akm) == RSN_AKM_FILS_SHA256 || \
			(akm) == RSN_AKM_FILS_SHA384 || \
			(akm) == RSN_AKM_OWE || \
			(akm) == RSN_AKM_SUITEB_SHA256_1X || \
			(akm) == RSN_AKM_SUITEB_SHA384_1X || \
			(akm) == RSN_AKM_PASN)

#define IS_VALID_BIP_CIPHER(cipher) ((cipher) == WPA_CIPHER_BIP || \
					(cipher) == WPA_CIPHER_BIP_GMAC_128 || \
					(cipher) == WPA_CIPHER_BIP_GMAC_256 || \
					(cipher) == WPA_CIPHER_BIP_CMAC_256 || \
					(cipher) == WPA_CIPHER_TPK)

#define WPA_IS_FT_AKM(akm)	((akm) == RSN_AKM_FBT_SHA256 || \
			(akm) == RSN_AKM_FBT_SHA384)

#define WPA_IS_FILS_AKM(akm)	((akm) == RSN_AKM_FILS_SHA256 || \
			(akm) == RSN_AKM_FILS_SHA384)

#define WPA_IS_FILS_FT_AKM(akm)	((akm) == RSN_AKM_FBT_SHA256_FILS || \
			(akm) == RSN_AKM_FBT_SHA384_FILS)

/* WPA TKIP countermeasures parameters */
#define WPA_TKIP_CM_DETECT	60	/* multiple MIC failure window (seconds) */
#define WPA_TKIP_CM_BLOCK	60	/* countermeasures active window (seconds) */

/* RSN IE defines */
#define RSN_CAP_LEN		2	/* Length of RSN capabilities field (2 octets) */

/* RSN Capabilities defined in 802.11i */
#define RSN_CAP_PREAUTH			0x0001
#define RSN_CAP_NOPAIRWISE		0x0002
#define RSN_CAP_PTK_REPLAY_CNTR_MASK	0x000C
#define RSN_CAP_PTK_REPLAY_CNTR_SHIFT	2
#define RSN_CAP_GTK_REPLAY_CNTR_MASK	0x0030
#define RSN_CAP_GTK_REPLAY_CNTR_SHIFT	4
#define RSN_CAP_1_REPLAY_CNTR		0
#define RSN_CAP_2_REPLAY_CNTRS		1
#define RSN_CAP_4_REPLAY_CNTRS		2
#define RSN_CAP_16_REPLAY_CNTRS		3
#define RSN_CAP_MFPR			0x0040
#define RSN_CAP_MFPC			0x0080
#define RSN_CAP_SPPC			0x0400
#define RSN_CAP_SPPR			0x0800
#define RSN_CAP_OCVC			0x4000

/* WPA capabilities defined in 802.11i */
#define WPA_CAP_4_REPLAY_CNTRS		RSN_CAP_4_REPLAY_CNTRS
#define WPA_CAP_16_REPLAY_CNTRS		RSN_CAP_16_REPLAY_CNTRS
#define WPA_CAP_REPLAY_CNTR_SHIFT	RSN_CAP_PTK_REPLAY_CNTR_SHIFT
#define WPA_CAP_REPLAY_CNTR_MASK	RSN_CAP_PTK_REPLAY_CNTR_MASK

/* WPA capabilities defined in 802.11zD9.0 */
#define WPA_CAP_PEER_KEY_ENABLE		(0x1 << 1)	/* bit 9 */

/* WPA Specific defines */
#define WPA_CAP_LEN	RSN_CAP_LEN	/* Length of RSN capabilities in RSN IE (2 octets) */
#define WPA_PMKID_CNT_LEN	2	/* Length of RSN PMKID count (2 octests) */

#define	WPA_CAP_WPA2_PREAUTH		RSN_CAP_PREAUTH

#define WPA2_PMKID_COUNT_LEN	2

/* RSN dev type in rsn_info struct */
typedef enum {
	DEV_NONE = 0,
	DEV_STA = 1,
	DEV_AP = 2
} device_type_t;

typedef uint32 rsn_akm_mask_t;			/* RSN_AKM_... see 802.11.h */
typedef uint8  rsn_cipher_t;			/* WPA_CIPHER_xxx */
typedef uint32 rsn_ciphers_t;			/* mask of rsn_cipher_t */
typedef uint8 rsn_akm_t;
typedef uint8 auth_ie_type_mask_t;

/* Old location for this structure. Moved to bcmwpa.h */
#ifndef RSN_IE_INFO_STRUCT_RELOCATED
typedef struct rsn_ie_info {
	uint8 version;
	rsn_cipher_t g_cipher;
	uint8 p_count;
	uint8 akm_count;
	uint8 pmkid_count;
	rsn_akm_t sta_akm;			/* single STA akm */
	uint16 caps;
	rsn_ciphers_t p_ciphers;
	rsn_akm_mask_t akms;
	uint8 pmkids_offset;			/* offset into the IE */
	rsn_cipher_t g_mgmt_cipher;
	device_type_t dev_type;			/* AP or STA */
	rsn_cipher_t sta_cipher;		/* single STA cipher */
	uint16 key_desc;			/* key descriptor version as STA */
	int parse_status;
	uint16 mic_len;				/* unused. keep for ROM compatibility. */
	auth_ie_type_mask_t auth_ie_type;	/* bit field of WPA, WPA2 and (not yet) CCX WAPI */
	uint8 pmk_len;				/* EAPOL PMK */
	uint8 kck_mic_len;			/* EAPOL MIC (by KCK) */
	uint8 kck_len;				/* EAPOL KCK */
	uint8 kek_len;				/* EAPOL KEK */
	uint8 tk_len;				/* EAPOL TK */
	uint8 ptk_len;				/* EAPOL PTK */
	uint8 kck2_len;				/* EAPOL KCK2 */
	uint8 kek2_len;				/* EAPOL KEK2 */
	uint8 rsnxe_len;			/* RSNXE IE from assoc request */
	uint8 *rsnxe;				/* RSNXE IE length */
	uint8 kdk_len;				/* EAPOL KDK */
	uint8 pad[3];
} rsn_ie_info_t;
#endif /* RSN_IE_INFO_STRUCT_RELOCATED */

/* This marks the end of a packed structure section. */
#include <packed_section_end.h>

#endif /* _proto_wpa_h_ */
