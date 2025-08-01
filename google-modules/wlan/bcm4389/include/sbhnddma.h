/*
 * Generic Broadcom Home Networking Division (HND) DMA engine HW interface
 * This supports the following chips: BCM42xx, 44xx, 47xx .
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

#ifndef	_sbhnddma_h_
#define	_sbhnddma_h_

/* DMA structure:
 *  support two DMA engines: 32 bits address or 64 bit addressing
 *  basic DMA register set is per channel(transmit or receive)
 *  a pair of channels is defined for convenience
 */

/* 32 bits addressing */

/** dma registers per channel(xmt or rcv) */
typedef volatile struct {
	uint32	control;		/**< enable, et al */
	uint32	addr;			/**< descriptor ring base address (4K aligned) */
	uint32	ptr;			/**< last descriptor posted to chip */
	uint32	status;			/**< current active descriptor, et al */
} dma32regs_t;

typedef volatile struct {
	dma32regs_t	xmt;		/**< dma tx channel */
	dma32regs_t	rcv;		/**< dma rx channel */
} dma32regp_t;

typedef volatile struct {	/* diag access */
	uint32	fifoaddr;		/**< diag address */
	uint32	fifodatalow;		/**< low 32bits of data */
	uint32	fifodatahigh;		/**< high 32bits of data */
	uint32	pad;			/**< reserved */
} dma32diag_t;

/**
 * DMA Descriptor
 * Descriptors are only read by the hardware, never written back.
 */
typedef volatile struct {
	uint32	ctrl;		/**< misc control bits & bufcount */
	uint32	addr;		/**< data buffer address */
} dma32dd_t;

/** Each descriptor ring must be 4096byte aligned, and fit within a single 4096byte page. */
#define	D32RINGALIGN_BITS	12
#define	D32MAXRINGSZ		(1 << D32RINGALIGN_BITS)
#define	D32RINGALIGN		(1 << D32RINGALIGN_BITS)

#define	D32MAXDD	(D32MAXRINGSZ / sizeof (dma32dd_t))

/* transmit channel control */
#define	XC_XE		((uint32)1 << 0)	/**< transmit enable */
#define	XC_SE		((uint32)1 << 1)	/**< transmit suspend request */
#define	XC_LE		((uint32)1 << 2)	/**< loopback enable */
#define	XC_FL		((uint32)1 << 4)	/**< flush request */
#define XC_MR_MASK	0x000001C0		/**< Multiple outstanding reads */
#define XC_MR_SHIFT	6
#define	XC_PD		((uint32)1 << 11)	/**< parity check disable */
#define	XC_AE		((uint32)3 << 16)	/**< address extension bits */
#define	XC_AE_SHIFT	16
#define XC_BL_MASK	0x001C0000		/**< BurstLen bits */
#define XC_BL_SHIFT	18
#define XC_PC_MASK	0x00E00000		/**< Prefetch control */
#define XC_PC_SHIFT	21
#define XC_PT_MASK	0x03000000		/**< Prefetch threshold */
#define XC_PT_SHIFT	24

/** Multiple outstanding reads */
#define DMA_MR_1	0
#define DMA_MR_2	1
#define DMA_MR_4	2
#define DMA_MR_8	3
#define DMA_MR_12	4
#define DMA_MR_16	5
#define DMA_MR_20	6
#define DMA_MR_32	7

/** DMA Burst Length in bytes */
#define DMA_BL_16	0
#define DMA_BL_32	1
#define DMA_BL_64	2
#define DMA_BL_128	3
#define DMA_BL_256	4
#define DMA_BL_512	5
#define DMA_BL_1024	6
#define DMA_BL_INVALID	0xFF

/** Prefetch control */
#define DMA_PC_0	0
#define DMA_PC_4	1
#define DMA_PC_8	2
#define DMA_PC_16	3
#define DMA_PC_32	4
/* others: reserved */

/** Prefetch threshold */
#define DMA_PT_1	0
#define DMA_PT_2	1
#define DMA_PT_4	2
#define DMA_PT_8	3

/** Channel Switch */
#define DMA_CS_OFF	0
#define DMA_CS_ON	1

/* transmit descriptor table pointer */
#define	XP_LD_MASK	0xfff			/**< last valid descriptor */

/* transmit channel status */
#define	XS_CD_MASK	0x0fff			/**< current descriptor pointer */
#define	XS_XS_MASK	0xf000			/**< transmit state */
#define	XS_XS_SHIFT	12
#define	XS_XS_DISABLED	0x0000			/**< disabled */
#define	XS_XS_ACTIVE	0x1000			/**< active */
#define	XS_XS_IDLE	0x2000			/**< idle wait */
#define	XS_XS_STOPPED	0x3000			/**< stopped */
#define	XS_XS_SUSP	0x4000			/**< suspend pending */
#define	XS_XE_MASK	0xf0000			/**< transmit errors */
#define	XS_XE_SHIFT	16
#define	XS_XE_NOERR	0x00000			/**< no error */
#define	XS_XE_DPE	0x10000			/**< descriptor protocol error */
#define	XS_XE_DFU	0x20000			/**< data fifo underrun */
#define	XS_XE_BEBR	0x30000			/**< bus error on buffer read */
#define	XS_XE_BEDA	0x40000			/**< bus error on descriptor access */
#define	XS_AD_MASK	0xfff00000		/**< active descriptor */
#define	XS_AD_SHIFT	20

/* receive channel control */
#define	RC_RE		((uint32)1 << 0)	/**< receive enable */
#define	RC_RO_MASK	0xfe			/**< receive frame offset */
#define	RC_RO_SHIFT	1
#define	RC_FM		((uint32)1 << 8)	/**< direct fifo receive (pio) mode */
#define	RC_SH		((uint32)1 << 9)	/**< separate rx header descriptor enable */
#define	RC_OC		((uint32)1 << 10)	/**< overflow continue */
#define	RC_PD		((uint32)1 << 11)	/**< parity check disable */
#define	RC_AE		((uint32)3 << 16)	/**< address extension bits */
#define	RC_AE_SHIFT	16
#define RC_BL_MASK	0x001C0000		/**< BurstLen bits */
#define RC_BL_SHIFT	18
#define RC_PC_MASK	0x00E00000		/**< Prefetch control */
#define RC_PC_SHIFT	21
#define RC_PT_MASK	0x03000000		/**< Prefetch threshold */
#define RC_PT_SHIFT	24
#define RC_WAITCMP_MASK 0x00001000
#define RC_WAITCMP_SHIFT 12
/* receive descriptor table pointer */
#define	RP_LD_MASK	0xfff			/**< last valid descriptor */

/* receive channel status */
#define	RS_CD_MASK	0x0fff			/**< current descriptor pointer */
#define	RS_RS_MASK	0xf000			/**< receive state */
#define	RS_RS_SHIFT	12
#define	RS_RS_DISABLED	0x0000			/**< disabled */
#define	RS_RS_ACTIVE	0x1000			/**< active */
#define	RS_RS_IDLE	0x2000			/**< idle wait */
#define	RS_RS_STOPPED	0x3000			/**< reserved */
#define	RS_RE_MASK	0xf0000			/**< receive errors */
#define	RS_RE_SHIFT	16
#define	RS_RE_NOERR	0x00000			/**< no error */
#define	RS_RE_DPE	0x10000			/**< descriptor protocol error */
#define	RS_RE_DFO	0x20000			/**< data fifo overflow */
#define	RS_RE_BEBW	0x30000			/**< bus error on buffer write */
#define	RS_RE_BEDA	0x40000			/**< bus error on descriptor access */
#define	RS_AD_MASK	0xfff00000		/**< active descriptor */
#define	RS_AD_SHIFT	20

/* fifoaddr */
#define	FA_OFF_MASK	0xffff			/**< offset */
#define	FA_SEL_MASK	0xf0000			/**< select */
#define	FA_SEL_SHIFT	16
#define	FA_SEL_XDD	0x00000			/**< transmit dma data */
#define	FA_SEL_XDP	0x10000			/**< transmit dma pointers */
#define	FA_SEL_RDD	0x40000			/**< receive dma data */
#define	FA_SEL_RDP	0x50000			/**< receive dma pointers */
#define	FA_SEL_XFD	0x80000			/**< transmit fifo data */
#define	FA_SEL_XFP	0x90000			/**< transmit fifo pointers */
#define	FA_SEL_RFD	0xc0000			/**< receive fifo data */
#define	FA_SEL_RFP	0xd0000			/**< receive fifo pointers */
#define	FA_SEL_RSD	0xe0000			/**< receive frame status data */
#define	FA_SEL_RSP	0xf0000			/**< receive frame status pointers */

/* descriptor control flags */
#define	CTRL_BC_MASK	0x00001fff		/**< buffer byte count, real data len must <= 4KB */
#define	CTRL_AE		((uint32)3 << 16)	/**< address extension bits */
#define	CTRL_AE_SHIFT	16
#define	CTRL_PARITY	((uint32)3 << 18)	/**< parity bit */
#define	CTRL_EOT	((uint32)1 << 28)	/**< end of descriptor table */
#define	CTRL_IOC	((uint32)1 << 29)	/**< interrupt on completion */
#define	CTRL_EOF	((uint32)1 << 30)	/**< end of frame */
#define	CTRL_SOF	((uint32)1 << 31)	/**< start of frame */

/** control flags in the range [27:20] are core-specific and not defined here */
#define	CTRL_CORE_MASK	0x0ff00000

/* 64 bits addressing */

/** dma registers per channel(xmt or rcv) */
typedef volatile struct {
	uint32	control;	/**< enable, et al */
	uint32	ptr;		/**< last descriptor posted to chip */
	uint32	addrlow;	/**< descriptor ring base address low 32-bits (8K aligned) */
	uint32	addrhigh;	/**< descriptor ring base address bits 63:32 (8K aligned) */
	uint32	status0;	/**< current descriptor, xmt state */
	uint32	status1;	/**< active descriptor, xmt error */
} dma64regs_t;

typedef volatile struct {
	dma64regs_t	tx;		/**< dma64 tx channel */
	dma64regs_t	rx;		/**< dma64 rx channel */
} dma64regp_t;

typedef volatile struct {		/**< diag access */
	uint32	fifoaddr;		/**< diag address */
	uint32	fifodatalow;		/**< low 32bits of data */
	uint32	fifodatahigh;		/**< high 32bits of data */
	uint32	pad;			/**< reserved */
} dma64diag_t;

/**
 * DMA Descriptor
 * Descriptors are only read by the hardware, never written back.
 */
typedef volatile struct {
	uint32	ctrl1;		/**< misc control bits */
	uint32	ctrl2;		/**< buffer count and address extension */
	uint32	addrlow;	/**< memory address of the date buffer, bits 31:0 */
	uint32	addrhigh;	/**< memory address of the date buffer, bits 63:32 */
} dma64dd_t;

/**
 * Pool implementation: each pool is 64KB max.  Align it to maximize ability to grow
 */
#define D64POOLALIGN_BITS	15u
#define D64POOLALIGN_BITS_MAX	16u
/**
 * Each descriptor ring must be 8kB aligned, and fit within a contiguous 8kB physical addresss.
 */
#define D64RINGALIGN_BITS	13
#define	D64MAXRINGSZ		(1 << D64RINGALIGN_BITS)
#define	D64RINGBOUNDARY		(1 << D64RINGALIGN_BITS)

#define	D64MAXDD	(D64MAXRINGSZ / sizeof (dma64dd_t))

/** for cores with large descriptor ring support, descriptor ring size can be up to 4096 */
#define	D64MAXDD_LARGE		((1 << 16) / sizeof (dma64dd_t))

/**
 * for cores with large descriptor ring support (4k descriptors), descriptor ring cannot cross
 * 64K boundary
 */
#define	D64RINGBOUNDARY_LARGE	(1 << 16)

/*
 * Default DMA Burstlen values for USBRev >= 12 and SDIORev >= 11.
 * When this field contains the value N, the burst length is 2**(N + 4) bytes.
 */
#define D64_DEF_USBBURSTLEN     2
#define D64_DEF_SDIOBURSTLEN    1

#ifndef D64_USBBURSTLEN
#define D64_USBBURSTLEN	DMA_BL_64
#endif
#ifndef D64_SDIOBURSTLEN
#define D64_SDIOBURSTLEN	DMA_BL_32
#endif

/* transmit channel control */
#define	D64_XC_XE		0x00000001	/**< transmit enable */
#define	D64_XC_SE		0x00000002	/**< transmit suspend request */
#define	D64_XC_LE		0x00000004	/**< loopback enable */
#define	D64_XC_FL		0x00000010	/**< flush request */
#define D64_XC_MR_MASK		0x000001C0	/**< Multiple outstanding reads */
#define D64_XC_MR_SHIFT		6
#define D64_XC_CS_SHIFT		9		/**< channel switch enable */
#define D64_XC_CS_MASK		0x00000200      /**< channel switch enable */
#define	D64_XC_PD		0x00000800	/**< parity check disable */
#define	D64_XC_AE		0x00030000	/**< address extension bits */
#define	D64_XC_AE_SHIFT		16
#define D64_XC_BL_MASK		0x001C0000	/**< BurstLen bits */
#define D64_XC_BL_SHIFT		18
#define D64_XC_PC_MASK		0x00E00000		/**< Prefetch control */
#define D64_XC_PC_SHIFT		21
#define D64_XC_PT_MASK		0x03000000		/**< Prefetch threshold */
#define D64_XC_PT_SHIFT		24
#define D64_XC_CO_MASK		0x04000000	/**< coherent transactions for descriptors */
#define D64_XC_CO_SHIFT		26

/* transmit descriptor table pointer */
#define	D64_XP_LD_MASK		0x00001fff	/**< last valid descriptor */

/* transmit channel status */
#define	D64_XS0_CD_MASK		(di->d64_xs0_cd_mask)	/**< current descriptor pointer */
#define	D64_XS0_XS_MASK		0xf0000000     	/**< transmit state */
#define	D64_XS0_XS_SHIFT		28
#define	D64_XS0_XS_DISABLED	0x00000000	/**< disabled */
#define	D64_XS0_XS_ACTIVE	0x10000000	/**< active */
#define	D64_XS0_XS_IDLE		0x20000000	/**< idle wait */
#define	D64_XS0_XS_STOPPED	0x30000000	/**< stopped */
#define	D64_XS0_XS_SUSP		0x40000000	/**< suspend pending */

#define	D64_XS1_AD_MASK		(di->d64_xs1_ad_mask)	/**< active descriptor */
#define	D64_XS1_XE_MASK		0xf0000000     	/**< transmit errors */
#define	D64_XS1_XE_SHIFT		28
#define	D64_XS1_XE_NOERR	0x00000000	/**< no error */
#define	D64_XS1_XE_DPE		0x10000000	/**< descriptor protocol error */
#define	D64_XS1_XE_DFU		0x20000000	/**< data fifo underrun */
#define	D64_XS1_XE_DTE		0x30000000	/**< data transfer error */
#define	D64_XS1_XE_DESRE	0x40000000	/**< descriptor read error */
#define	D64_XS1_XE_COREE	0x50000000	/**< core error */

/* receive channel control */
#define	D64_RC_RE		0x00000001	/**< receive enable */
#define	D64_RC_RO_MASK		0x000000fe	/**< receive frame offset */
#define	D64_RC_RO_SHIFT		1
#define	D64_RC_FM		0x00000100	/**< direct fifo receive (pio) mode */
#define	D64_RC_SH		0x00000200	/**< separate rx header descriptor enable */
#define	D64_RC_SHIFT		9	/**< separate rx header descriptor enable */
#define	D64_RC_OC		0x00000400	/**< overflow continue */
#define	D64_RC_PD		0x00000800	/**< parity check disable */
#define D64_RC_WAITCMP_MASK	0x00001000
#define D64_RC_WAITCMP_SHIFT	12
#define D64_RC_SA		0x00002000	/**< select active */
#define D64_RC_GE		0x00004000	/**< Glom enable */
#define	D64_RC_AE		0x00030000	/**< address extension bits */
#define	D64_RC_AE_SHIFT		16
#define D64_RC_BL_MASK		0x001C0000	/**< BurstLen bits */
#define D64_RC_BL_SHIFT		18
#define D64_RC_PC_MASK		0x00E00000	/**< Prefetch control */
#define D64_RC_PC_SHIFT		21
#define D64_RC_PT_MASK		0x03000000	/**< Prefetch threshold */
#define D64_RC_PT_SHIFT		24
#define D64_RC_CO_MASK		0x04000000	/**< coherent transactions for descriptors */
#define D64_RC_CO_SHIFT		26
#define	D64_RC_ROEXT_MASK	0x08000000	/**< receive frame offset extension bit */
#define	D64_RC_ROEXT_SHIFT	27
#define D64_RC_MOW_SHIFT	(28u)		/**< multiple outstanding write */
#define D64_RC_MOW_MASK		((0x3u) << D64_RC_MOW_SHIFT)

/* receive control values */
/* RcvCtrl.MultipleOutstandingWrites(MOW) valid values(N) listed below.
 * (N + 1) out standing write(s) supported
 */
#define D64_RC_MOW_1		(0u)		/**< 1 outstanding write */
#define D64_RC_MOW_2		(1u)		/**< 2 outstanding writes */
#define D64_RC_MOW_3		(2u)		/**< 3 outstanding writes */
#define D64_RC_MOW_4		(3u)		/**< 4 outstanding writes */

/* flags for dma controller */
#define DMA_CTRL_PEN		(1u << 0u)	/**< partity enable */
#define DMA_CTRL_ROC		(1u << 1u)	/**< rx overflow continue */
#define DMA_CTRL_RXMULTI	(1u << 2u)	/**< allow rx scatter to multiple descriptors */
#define DMA_CTRL_UNFRAMED	(1u << 3u)	/**< Unframed Rx/Tx data */
#define DMA_CTRL_USB_BOUNDRY4KB_WAR (1u << 4u)	/**< USB core REV9's SETUP dma channel's
						*  buffer can not crossed 4K boundary PR80468
						*/
#define DMA_CTRL_DMA_AVOIDANCE_WAR (1u << 5u)	/**< DMA avoidance WAR for 4331 */
#define DMA_CTRL_RXSINGLE	(1u << 6u)	/**< always single buffer */
#define DMA_CTRL_SDIO_RXGLOM	(1u << 7u)	/**< DMA Rx glome is enabled */
#define DMA_CTRL_DESC_ONLY_FLAG (1u << 8u)	/**< For DMA which posts only descriptors,
						 * no packets
						 */
#define DMA_CTRL_DESC_CD_WAR	(1u << 9u)	/**< WAR for descriptor only DMA's CD not being
						 * updated correctly by HW in CT mode.
						 */
#define DMA_CTRL_CS		(1u << 10u)	/* channel switch enable */
#define DMA_CTRL_ROEXT		(1u << 11u)	/* receive frame offset extension support */
#define DMA_CTRL_RX_ALIGN_8BYTE	(1u << 12u)	/* RXDMA address 8-byte aligned */
#define DMA_CTRL_SHARED_POOL	(1u << 15u)	/** shared descriptor pool */
#define DMA_CTRL_COREUNIT_SHIFT	(17u)		/* Core unit shift */
#define DMA_CTRL_COREUNIT_MASK	(0x3u << 17u)	/* Core unit mask */

#define DMA_CTRL_SET_COREUNIT(di, coreunit) \
	((di)->hnddma.dmactrlflags |= \
	(((coreunit) << DMA_CTRL_COREUNIT_SHIFT) & DMA_CTRL_COREUNIT_MASK))

#define DMA_CTRL_GET_COREUNIT(di) \
	(((di)->hnddma.dmactrlflags & DMA_CTRL_COREUNIT_MASK) >> DMA_CTRL_COREUNIT_SHIFT)

/* receive descriptor table pointer */
#define	D64_RP_LD_MASK		0x00001fff	/**< last valid descriptor */

/* receive channel status */
#define	D64_RS0_CD_MASK		(di->d64_rs0_cd_mask)	/**< current descriptor pointer */
#define	D64_RS0_RS_MASK		0xf0000000     	/**< receive state */
#define	D64_RS0_RS_SHIFT		28
#define	D64_RS0_RS_DISABLED	0x00000000	/**< disabled */
#define	D64_RS0_RS_ACTIVE	0x10000000	/**< active */
#define	D64_RS0_RS_IDLE		0x20000000	/**< idle wait */
#define	D64_RS0_RS_STOPPED	0x30000000	/**< stopped */
#define	D64_RS0_RS_SUSP		0x40000000	/**< suspend pending */

#define	D64_RS1_AD_MASK		(di->d64_rs1_ad_mask)	/* active descriptor pointer */
#define	D64_RS1_RE_MASK		0xf0000000	/* receive errors */
#define	D64_RS1_RE_SHIFT		28
#define	D64_RS1_RE_NOERR	0x00000000	/**< no error */
#define	D64_RS1_RE_DPO		0x10000000	/**< descriptor protocol error */
#define	D64_RS1_RE_DFU		0x20000000	/**< data fifo overflow */
#define	D64_RS1_RE_DTE		0x30000000	/**< data transfer error */
#define	D64_RS1_RE_DESRE	0x40000000	/**< descriptor read error */
#define	D64_RS1_RE_COREE	0x50000000	/**< core error */

/* fifoaddr */
#define	D64_FA_OFF_MASK		0xffff		/**< offset */
#define	D64_FA_SEL_MASK		0xf0000		/**< select */
#define	D64_FA_SEL_SHIFT	16
#define	D64_FA_SEL_XDD		0x00000		/**< transmit dma data */
#define	D64_FA_SEL_XDP		0x10000		/**< transmit dma pointers */
#define	D64_FA_SEL_RDD		0x40000		/**< receive dma data */
#define	D64_FA_SEL_RDP		0x50000		/**< receive dma pointers */
#define	D64_FA_SEL_XFD		0x80000		/**< transmit fifo data */
#define	D64_FA_SEL_XFP		0x90000		/**< transmit fifo pointers */
#define	D64_FA_SEL_RFD		0xc0000		/**< receive fifo data */
#define	D64_FA_SEL_RFP		0xd0000		/**< receive fifo pointers */
#define	D64_FA_SEL_RSD		0xe0000		/**< receive frame status data */
#define	D64_FA_SEL_RSP		0xf0000		/**< receive frame status pointers */

/* descriptor control flags 1 */
#define D64_CTRL_COREFLAGS	0x0ff00000		/**< core specific flags */

/**< bzero operation for receive channels or a compare-to-zero operation for transmit engines */
#define D64_CTRL1_BIT_BZEROBCMP		(15u)
/* WAR for JIRA CRWLDMA-245 */
#define D64_DMA_COREFLAGS_WAR_BIT	(25u)

#define D64_CTRL1_COHERENT      ((uint32)1 << 17)       /**< cache coherent per transaction */
#define	D64_CTRL1_NOTPCIE	((uint32)1 << 18)	/**< buirst size control */
#define	D64_CTRL1_EOT		((uint32)1 << 28)	/**< end of descriptor table */
#define	D64_CTRL1_IOC		((uint32)1 << 29)	/**< interrupt on completion */
#define	D64_CTRL1_EOF		((uint32)1 << 30)	/**< end of frame */
#define	D64_CTRL1_SOF		((uint32)1 << 31)	/**< start of frame */
#define D64_CTRL1_SOFPTR	0x0000FFFFu
#define D64_CTRL1_NUMD_MASK	0x00F00000u
#define D64_CTRL1_NUMD_SHIFT	20u

/* descriptor control flags 2 */
#define	D64_CTRL2_MAX_LEN	0x0000fff7 /* Max transfer length (buffer byte count) <= 65527 */
#define	D64_CTRL2_BC_MASK	0x0000ffff /**< mask for buffer byte count */
#define	D64_CTRL2_AE		0x00030000 /**< address extension bits */
#define	D64_CTRL2_AE_SHIFT	16
#define D64_CTRL2_PARITY	0x00040000      /* parity bit */

/** control flags in the range [27:20] are core-specific and not defined here */
#define	D64_CTRL_CORE_MASK	0x0ff00000

#define D64_RX_FRM_STS_LEN	0x0000ffff	/**< frame length mask */
#define D64_RX_FRM_STS_OVFL	0x00800000	/**< RxOverFlow */
#define D64_RX_FRM_STS_DSCRCNT	0x0f000000 /**< no. of descriptors used - 1, d11corerev >= 22 */
#define D64_RX_FRM_STS_DSCRCNT_SHIFT   24      /* Shift for no .of dma descriptor field */
#define D64_RX_FRM_STS_DATATYPE	0xf0000000	/**< core-dependent data type */

#define BCM_D64_CTRL2_BOUND_DMA_LENGTH(len) \
(((len) > D64_CTRL2_MAX_LEN) ? D64_CTRL2_MAX_LEN : (len))

/** receive frame status */
typedef volatile struct {
	uint16 len;
	uint16 flags;
} dma_rxh_t;

#endif	/* _sbhnddma_h_ */
