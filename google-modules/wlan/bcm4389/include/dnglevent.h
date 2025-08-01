/*
 * Broadcom Event protocol definitions
 *
 * Dependencies: bcmeth.h
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
 *
 * -----------------------------------------------------------------------------
 *
 */

/*
 * Broadcom dngl Ethernet Events protocol defines
 *
 */

#ifndef _DNGLEVENT_H_
#define _DNGLEVENT_H_

#ifndef _TYPEDEFS_H_
#include <typedefs.h>
#endif
#include <bcmeth.h>
#include <ethernet.h>
#ifdef HEALTH_CHECK
#include <dngl_defs.h>
#endif /* HEALTH_CHECK */

/* This marks the start of a packed structure section. */
#include <packed_section_start.h>
#define BCM_DNGL_EVENT_MSG_VERSION		1
#define DNGL_E_RSRVD_1				0x0
#define DNGL_E_RSRVD_2				0x1
#define DNGL_E_SOCRAM_IND			0x2
#define DNGL_E_PROFILE_DATA_IND			0x3
#define DNGL_E_SPMI_RESET_IND			0x4
typedef BWL_PRE_PACKED_STRUCT struct
{
	uint16  version; /* Current version is 1 */
	uint16  reserved; /* reserved for any future extension */
	uint16  event_type; /* DNGL_E_SOCRAM_IND */
	uint16  datalen; /* Length of the event payload */
} BWL_POST_PACKED_STRUCT bcm_dngl_event_msg_t;

typedef BWL_PRE_PACKED_STRUCT struct bcm_dngl_event {
	struct ether_header eth;
	bcmeth_hdr_t        bcm_hdr;
	bcm_dngl_event_msg_t      dngl_event;
	/* data portion follows */
} BWL_POST_PACKED_STRUCT bcm_dngl_event_t;

typedef BWL_PRE_PACKED_STRUCT struct bcm_dngl_socramind {
	uint16			tag;	/* data tag */
	uint16			length; /* data length */
	uint8			value[BCM_FLEX_ARRAY]; /* variable length specified by length */
} BWL_POST_PACKED_STRUCT bcm_dngl_socramind_t;

typedef BWL_PRE_PACKED_STRUCT struct bcm_dngl_profile_data_ind_t {
	uint16 tag;
	uint16 length;
	uint8 value[];
} BWL_POST_PACKED_STRUCT bcm_dngl_profile_data_ind_t;

#define DNGL_E_SPMI_RESET_IND_VERSION_1 1u
#define DNGL_E_SPMI_RESET_IND_VERSION DNGL_E_SPMI_RESET_IND_VERSION_1

typedef BWL_PRE_PACKED_STRUCT struct bcm_dngl_spmi_reset_ind_v1_t {
	uint16		version;	/* Current version is 1 */
	uint16		num_resets;	/* number of resets seen since last message */
	uint8		slave_idx;	/* Slave idx of the SPMI core that was reset */
	uint8		PAD[3];
} BWL_POST_PACKED_STRUCT bcm_dngl_spmi_reset_ind_v1_t;

typedef BWL_PRE_PACKED_STRUCT struct bcm_dngl_arm_event {
	uint32 type;
	uint32 value;
} BWL_POST_PACKED_STRUCT bcm_dngl_arm_event_t;

#define PROFILE_DATA_IND_INFO 0x1

#define PROFILE_SUB_TYPE_ARM_STATS_INFO 0x1

typedef BWL_PRE_PACKED_STRUCT struct bcm_dngl_arm_stats_ind {
	uint16	tag;
	uint16	length;
	uint8	value[];
} BWL_POST_PACKED_STRUCT bcm_dngl_arm_stats_ind_t;

typedef BWL_PRE_PACKED_STRUCT struct bcm_dngl_arm_stats {
	uint32	cycles;
	uint32	timestamp;
	uint16	freq;
	uint16	roh;
	uint16	num_events;
	uint16  seq_no;
	uint8	value[];
} BWL_POST_PACKED_STRUCT bcm_dngl_arm_stats_t;

/* SOCRAM_IND type tags */
typedef enum socram_ind_tag {
	SOCRAM_IND_ASSERT_TAG = 1,
	SOCRAM_IND_TAG_HEALTH_CHECK = 2
} socram_ind_tag_t;

/* Health check top level module tags */
typedef BWL_PRE_PACKED_STRUCT struct bcm_dngl_healthcheck {
	uint16			top_module_tag;	/* top level module tag */
	uint16			top_module_len; /* Type of PCIE issue indication */
	uint8			value[BCM_FLEX_ARRAY]; /* variable length specified by length */
} BWL_POST_PACKED_STRUCT bcm_dngl_healthcheck_t;

/* Health check top level module tags */
#define HEALTH_CHECK_TOP_LEVEL_MODULE_PCIEDEV_RTE 1
#define HEALTH_CHECK_PCIEDEV_VERSION_1	1
#define HEALTH_CHECK_PCIEDEV_FLAG_IN_D3_SHIFT	0
#define HEALTH_CHECK_PCIEDEV_FLAG_AER_SHIFT		1
#define HEALTH_CHECK_PCIEDEV_FLAG_LINKDOWN_SHIFT		2
#define HEALTH_CHECK_PCIEDEV_FLAG_MSI_INT_SHIFT			3
#define HEALTH_CHECK_PCIEDEV_FLAG_NODS_SHIFT			4
#define HEALTH_CHECK_PCIEDEV_FLAG_NO_HOST_WAKE_SHIFT		5
#define HEALTH_CHECK_PCIEDEV_FLAG_IN_D3	1 << HEALTH_CHECK_PCIEDEV_FLAG_IN_D3_SHIFT
#define HEALTH_CHECK_PCIEDEV_FLAG_AER	1 << HEALTH_CHECK_PCIEDEV_FLAG_AER_SHIFT
#define HEALTH_CHECK_PCIEDEV_FLAG_LINKDOWN	1 << HEALTH_CHECK_PCIEDEV_FLAG_LINKDOWN_SHIFT
#define HEALTH_CHECK_PCIEDEV_FLAG_MSI_INT	1 << HEALTH_CHECK_PCIEDEV_FLAG_MSI_INT_SHIFT
#define HEALTH_CHECK_PCIEDEV_FLAG_NODS	1 << HEALTH_CHECK_PCIEDEV_FLAG_NODS_SHIFT
#define HEALTH_CHECK_PCIEDEV_FLAG_NO_HOST_WAKE	1 << HEALTH_CHECK_PCIEDEV_FLAG_NO_HOST_WAKE_SHIFT
/* PCIE Module TAGs */
#define HEALTH_CHECK_PCIEDEV_INDUCED_IND	0x1
#define HEALTH_CHECK_PCIEDEV_H2D_DMA_IND	0x2
#define HEALTH_CHECK_PCIEDEV_D2H_DMA_IND	0x3
#define HEALTH_CHECK_PCIEDEV_IOCTL_STALL_IND	0x4
#define HEALTH_CHECK_PCIEDEV_D3ACK_STALL_IND	0x5
#define HEALTH_CHECK_PCIEDEV_NODS_IND	0x6
#define HEALTH_CHECK_PCIEDEV_LINKSPEED_FALLBACK_IND	0x7
#define HEALTH_CHECK_PCIEDEV_DSACK_STALL_IND	0x8
#define HEALTH_CHECK_PCIEDEV_FLOWRING_IND	0x9
#define HEALTH_CHECK_PCIEDEV_HW_ASSERT_LONG_IND 0xA
#define HEALTH_CHECK_PCIEDEV_RXPOST_LONG_IND	0xB
#define HEALTH_CHECK_PCIEDEV_PTM_DRIFT_IND	0xC
#define HEALTH_CHECK_PCIEDEV_PTM_FAIL_IND	0xD
#define HEALTH_CHECK_PCIEDEV_PTM_TIMEOUT_IND	0xE

#define HC_PCIEDEV_CONFIG_REGLIST_MAX	25
typedef BWL_PRE_PACKED_STRUCT struct bcm_dngl_pcie_hc {
	uint16			version; /* HEALTH_CHECK_PCIEDEV_VERSION_1 */
	uint16			reserved;
	uint16			pcie_err_ind_type; /* PCIE Module TAGs */
	uint16			pcie_flag;
	uint32			pcie_control_reg;
	uint32			pcie_config_regs[HC_PCIEDEV_CONFIG_REGLIST_MAX];
} BWL_POST_PACKED_STRUCT bcm_dngl_pcie_hc_t;

/* define to avoid compile issues in older branches which define hchk_sw_entity_t */
#ifdef HCHK_COMMON_SW_EVENT
/* Enumerating top level SW entities for use by health check */
typedef enum {
	HCHK_SW_ENTITY_UNDEFINED = 0,
	HCHK_SW_ENTITY_PCIE = 1,
	HCHK_SW_ENTITY_SDIO = 2,
	HCHK_SW_ENTITY_USB = 3,
	HCHK_SW_ENTITY_RTE = 4,
	HCHK_SW_ENTITY_WL_PRIMARY = 5, /* WL instance 0 */
	HCHK_SW_ENTITY_WL_SECONDARY = 6, /* WL instance 1 */
	HCHK_SW_ENTITY_MAX
} hchk_sw_entity_t;
#endif /* HCHK_COMMON_SW_EVENT */

/* This marks the end of a packed structure section. */
#include <packed_section_end.h>

#endif /* _DNGLEVENT_H_ */
