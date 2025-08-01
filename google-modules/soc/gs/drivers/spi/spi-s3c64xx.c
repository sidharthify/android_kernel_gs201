// SPDX-License-Identifier: GPL-2.0+
//
// Copyright (c) 2009 Samsung Electronics Co., Ltd.
//      Jaswinder Singh <jassi.brar@samsung.com>

#include <linux/init.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/spi/spi.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>

#include <linux/platform_data/spi-s3c64xx-gs.h>

#include <linux/dma/dma-pl330.h>

#ifdef CONFIG_CPU_IDLE
#include <soc/google/exynos-cpupm.h>
#endif

static LIST_HEAD(drvdata_list);

#define MAX_SPI_PORTS		23
#define SPI_AUTOSUSPEND_TIMEOUT		(100)

/* Registers and bit-fields */

#define S3C64XX_SPI_CH_CFG		0x00
#define S3C64XX_SPI_CLK_CFG		0x04
#define S3C64XX_SPI_MODE_CFG		0x08
#define S3C64XX_SPI_SLAVE_SEL		0x0C
#define S3C64XX_SPI_INT_EN		0x10
#define S3C64XX_SPI_STATUS		0x14
#define S3C64XX_SPI_TX_DATA		0x18
#define S3C64XX_SPI_RX_DATA		0x1C
#define S3C64XX_SPI_PACKET_CNT		0x20
#define S3C64XX_SPI_PENDING_CLR		0x24
#define S3C64XX_SPI_SWAP_CFG		0x28
#define S3C64XX_SPI_FB_CLK		0x2C

#define S3C64XX_SPI_CH_HS_EN		BIT(6)	/* High Speed Enable */
#define S3C64XX_SPI_CH_SW_RST		BIT(5)
#define S3C64XX_SPI_CH_SLAVE		BIT(4)
#define S3C64XX_SPI_CPOL_L		BIT(3)
#define S3C64XX_SPI_CPHA_B		BIT(2)
#define S3C64XX_SPI_CH_RXCH_ON		BIT(1)
#define S3C64XX_SPI_CH_TXCH_ON		BIT(0)

#define S3C64XX_SPI_CLKSEL_SRCMSK	(3 << 9)
#define S3C64XX_SPI_CLKSEL_SRCSHFT	9
#define S3C64XX_SPI_ENCLK_ENABLE	BIT(8)
#define S3C64XX_SPI_PSR_MASK		0xff

#define S3C64XX_SPI_MODE_CH_TSZ_BYTE		(0 << 29)
#define S3C64XX_SPI_MODE_CH_TSZ_HALFWORD	BIT(29)
#define S3C64XX_SPI_MODE_CH_TSZ_WORD		(2 << 29)
#define S3C64XX_SPI_MODE_CH_TSZ_MASK		(3 << 29)
#define S3C64XX_SPI_MODE_BUS_TSZ_BYTE		(0 << 17)
#define S3C64XX_SPI_MODE_BUS_TSZ_HALFWORD	BIT(17)
#define S3C64XX_SPI_MODE_BUS_TSZ_WORD		(2 << 17)
#define S3C64XX_SPI_MODE_BUS_TSZ_MASK		(3 << 17)
#define S3C64XX_SPI_MODE_SELF_LOOPBACK		BIT(3)
#define S3C64XX_SPI_MODE_RXDMA_ON		BIT(2)
#define S3C64XX_SPI_MODE_TXDMA_ON		BIT(1)
#define S3C64XX_SPI_MODE_4BURST			BIT(0)

#define S3C64XX_SPI_SLAVE_NSC_CNT_2		(2 << 4)
#define S3C64XX_SPI_SLAVE_NSC_CNT_1		BIT(4)
#define S3C64XX_SPI_SLAVE_AUTO			BIT(1)
#define S3C64XX_SPI_SLAVE_SIG_INACT		BIT(0)

#define S3C64XX_SPI_INT_TRAILING_EN		BIT(6)
#define S3C64XX_SPI_INT_RX_OVERRUN_EN		BIT(5)
#define S3C64XX_SPI_INT_RX_UNDERRUN_EN		BIT(4)
#define S3C64XX_SPI_INT_TX_OVERRUN_EN		BIT(3)
#define S3C64XX_SPI_INT_TX_UNDERRUN_EN		BIT(2)
#define S3C64XX_SPI_INT_RX_FIFORDY_EN		BIT(1)
#define S3C64XX_SPI_INT_TX_FIFORDY_EN		BIT(0)

#define S3C64XX_SPI_ST_RX_OVERRUN_ERR		BIT(5)
#define S3C64XX_SPI_ST_RX_UNDERRUN_ERR		BIT(4)
#define S3C64XX_SPI_ST_TX_OVERRUN_ERR		BIT(3)
#define S3C64XX_SPI_ST_TX_UNDERRUN_ERR		BIT(2)
#define S3C64XX_SPI_ST_RX_FIFORDY		BIT(1)
#define S3C64XX_SPI_ST_TX_FIFORDY		BIT(0)

#define S3C64XX_SPI_PACKET_CNT_EN		BIT(16)

#define S3C64XX_SPI_PND_TX_UNDERRUN_CLR		BIT(4)
#define S3C64XX_SPI_PND_TX_OVERRUN_CLR		BIT(3)
#define S3C64XX_SPI_PND_RX_UNDERRUN_CLR		BIT(2)
#define S3C64XX_SPI_PND_RX_OVERRUN_CLR		BIT(1)
#define S3C64XX_SPI_PND_TRAILING_CLR		BIT(0)

#define S3C64XX_SPI_SWAP_RX_HALF_WORD		BIT(7)
#define S3C64XX_SPI_SWAP_RX_BYTE		BIT(6)
#define S3C64XX_SPI_SWAP_RX_BIT			BIT(5)
#define S3C64XX_SPI_SWAP_RX_EN			BIT(4)
#define S3C64XX_SPI_SWAP_TX_HALF_WORD		BIT(3)
#define S3C64XX_SPI_SWAP_TX_BYTE		BIT(2)
#define S3C64XX_SPI_SWAP_TX_BIT			BIT(1)
#define S3C64XX_SPI_SWAP_TX_EN			BIT(0)

#define S3C64XX_SPI_FBCLK_MSK			(3 << 0)

#define FIFO_LVL_MASK(i) ((i)->port_conf->fifo_lvl_mask[i->port_id])
#define S3C64XX_SPI_ST_TX_DONE(v, i) (((v) & \
				(1 << (i)->port_conf->tx_st_done)) ? 1 : 0)
#define TX_FIFO_LVL(v, i) (((v) >> 6) & FIFO_LVL_MASK(i))
#define RX_FIFO_LVL(v, i) (((v) >> (i)->port_conf->rx_lvl_offset) & \
					FIFO_LVL_MASK(i))

#define S3C64XX_SPI_MAX_TRAILCNT	0x3ff
#define S3C64XX_SPI_TRAILCNT_OFF	19

#define S3C64XX_SPI_TRAILCNT		S3C64XX_SPI_MAX_TRAILCNT

#define S3C64XX_SPI_DMA_4BURST_LEN	0x4
#define S3C64XX_SPI_DMA_1BURST_LEN	0x1

#define msecs_to_loops(t) (loops_per_jiffy / 1000 * HZ * (t))

#define RXBUSY    BIT(2)
#define TXBUSY    BIT(3)

#define SPI_DBG_MODE		(0x1 << 0)
#define SPI_LOOPBACK_MODE	(0x1 << 1)

#define USI_CON				(0xC4)
#define USI_OPTION			(0xC8)

#define USI_RESET			(0 << 0)
#define USI_HWACG_CLKREQ_ON		BIT(1)
#define USI_HWACG_CLKSTOP_ON		BIT(2)

/* MAX SIZE of COUNT_VALUE in PACKET_CNT_REG */
#define S3C64XX_SPI_PACKET_CNT_MAX 0xffff

#define USI_SW_CONF_MASK	(0x7 << 0)
#define USI_SPI_SW_CONF		BIT(1)
#define USI_I2C_SW_CONF		BIT(2)

/* Make sure the busy wait won't take too long time. */
#define MAX_CS_CLOCK_DELAY_US 20

/**
 * struct s3c64xx_spi_port_config - SPI Controller hardware info
 * @fifo_lvl_mask: Bit-mask for {TX|RX}_FIFO_LVL bits in SPI_STATUS register.
 * @rx_lvl_offset: Bit offset of RX_FIFO_LVL bits in SPI_STATUS regiter.
 * @tx_st_done: Bit offset of TX_DONE bit in SPI_STATUS regiter.
 * @high_speed: True, if the controller supports HIGH_SPEED_EN bit.
 * @clk_from_cmu: True, if the controller does not include a clock mux and
 *	prescaler unit.
 *
 * The Samsung s3c64xx SPI controller are used on various Samsung SoC's but
 * differ in some aspects such as the size of the fifo and spi bus clock
 * setup. Such differences are specified to the driver using this structure
 * which is provided as driver data to the driver.
 */
struct s3c64xx_spi_port_config {
	int	fifo_lvl_mask[MAX_SPI_PORTS];
	int	rx_lvl_offset;
	int	tx_st_done;
	bool	high_speed;
	bool	clk_from_cmu;
};

static ssize_t
spi_dbg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	ret += scnprintf(buf + ret, PAGE_SIZE - ret,
			"SPI Debug Mode Configuration.\n"
			"0 : Change loopback & DBG mode.\n"
			"1 : Change DBG mode.\n"
			"2 : Change Normal mode.\n");

	return ret;
}

static ssize_t
spi_dbg_store(struct device *dev, struct device_attribute *attr,
	      const char *buf, size_t count)
{
	struct spi_master *master = dev_get_drvdata(dev);
	struct s3c64xx_spi_driver_data *sdd = spi_master_get_devdata(master);
	struct s3c64xx_spi_info *sci = sdd->cntrlr_info;
	struct s3c64xx_spi_info *check_sci;
	int ret, input_cmd;

	ret = kstrtoint(buf, 10, &input_cmd);
	if (ret)
		return ret;

	list_for_each_entry(check_sci, &drvdata_list, node) {
		if (check_sci != sci)
			continue;

		switch (input_cmd) {
		case 0:
			dev_info(&sdd->pdev->dev, "Change SPI%d to Loopback(DBG) mode\n",
				 sdd->port_id);
			sci->dbg_mode = SPI_DBG_MODE | SPI_LOOPBACK_MODE;
			break;
		case 1:
			dev_info(&sdd->pdev->dev, "Change SPI%d to DBG mode\n",
				 sdd->port_id);
			sci->dbg_mode = SPI_DBG_MODE;
			break;
		case 2:
			dev_info(&sdd->pdev->dev, "Change SPI%d to normal mode\n",
				 sdd->port_id);
			sci->dbg_mode = 0;
			break;
		default:
			dev_info(&sdd->pdev->dev, "Wrong Command!(0/1/2)\n");
		}

		break;
	}

	return count;
}
DEVICE_ATTR_RW(spi_dbg);

static void s3c64xx_spi_dump_reg(struct s3c64xx_spi_driver_data *sdd)
{
	void __iomem *regs = sdd->regs;
	struct device *dev = &sdd->pdev->dev;

	dev_err(dev, "Register dump for SPI\n"
		"	CH_CFG       0x%08x\n"
		"	MODE_CFG     0x%08x\n"
		"	CS_REG       0x%08x\n"
		"	STATUS       0x%08x\n"
		"	PACKET_CNT   0x%08x\n",
		readl(regs + S3C64XX_SPI_CH_CFG),
		readl(regs + S3C64XX_SPI_MODE_CFG),
		readl(regs + S3C64XX_SPI_SLAVE_SEL),
		readl(regs + S3C64XX_SPI_STATUS),
		readl(regs + S3C64XX_SPI_PACKET_CNT)
	);
}

static void flush_fifo(struct s3c64xx_spi_driver_data *sdd)
{
	void __iomem *regs = sdd->regs;
	unsigned long loops;
	u32 val;

	val = readl(regs + S3C64XX_SPI_CH_CFG);
	val &= ~(S3C64XX_SPI_CH_RXCH_ON | S3C64XX_SPI_CH_TXCH_ON);
	writel(val, regs + S3C64XX_SPI_CH_CFG);

	writel(0, regs + S3C64XX_SPI_PACKET_CNT);

	val = readl(regs + S3C64XX_SPI_CH_CFG);
	val |= S3C64XX_SPI_CH_SW_RST;
	val &= ~S3C64XX_SPI_CH_HS_EN;
	writel(val, regs + S3C64XX_SPI_CH_CFG);

	/* Flush TxFIFO*/
	loops = msecs_to_loops(1);
	do {
		val = readl(regs + S3C64XX_SPI_STATUS);
	} while (TX_FIFO_LVL(val, sdd) && loops--);

	if (loops == 0)
		dev_warn(&sdd->pdev->dev, "Timed out flushing TX FIFO\n");

	/* Flush RxFIFO*/
	loops = msecs_to_loops(1);
	do {
		val = readl(regs + S3C64XX_SPI_STATUS);
		if (RX_FIFO_LVL(val, sdd))
			readl(regs + S3C64XX_SPI_RX_DATA);
		else
			break;
	} while (loops--);

	if (loops == 0)
		dev_warn(&sdd->pdev->dev, "Timed out flushing RX FIFO\n");

	val = readl(regs + S3C64XX_SPI_CH_CFG);
	val &= ~S3C64XX_SPI_CH_SW_RST;
	writel(val, regs + S3C64XX_SPI_CH_CFG);

	val = readl(regs + S3C64XX_SPI_MODE_CFG);
	val &= ~(S3C64XX_SPI_MODE_TXDMA_ON | S3C64XX_SPI_MODE_RXDMA_ON);
	writel(val, regs + S3C64XX_SPI_MODE_CFG);
}

static void s3c64xx_spi_dmacb(void *data)
{
	struct s3c64xx_spi_driver_data *sdd;
	struct s3c64xx_spi_dma_data *dma = data;
	unsigned long flags;

	if (dma->direction == DMA_DEV_TO_MEM)
		sdd = container_of(data,
				   struct s3c64xx_spi_driver_data, rx_dma);
	else
		sdd = container_of(data,
				   struct s3c64xx_spi_driver_data, tx_dma);

	spin_lock_irqsave(&sdd->lock, flags);

	if (dma->direction == DMA_DEV_TO_MEM) {
		sdd->state &= ~RXBUSY;
		if (!(sdd->state & TXBUSY))
			complete(&sdd->xfer_completion);
	} else {
		sdd->state &= ~TXBUSY;
		if (!(sdd->state & RXBUSY))
			complete(&sdd->xfer_completion);
	}

	spin_unlock_irqrestore(&sdd->lock, flags);
}

/* FIXME: remove this section once arch/arm/mach-s3c64xx uses dmaengine */

static struct s3c2410_dma_client s3c64xx_spi_dma_client = {
	.name = "samsung-spi-dma",
};

static void prepare_dma(struct s3c64xx_spi_dma_data *dma,
			unsigned int len, dma_addr_t buf)
{
	struct s3c64xx_spi_driver_data *sdd;
	struct samsung_dma_prep info;
	struct samsung_dma_config config;
	u32 modecfg;

	if (dma->direction == DMA_DEV_TO_MEM) {
		sdd = container_of((void *)dma,
				   struct s3c64xx_spi_driver_data, rx_dma);
		config.direction = sdd->rx_dma.direction;
		config.fifo = sdd->sfr_start + S3C64XX_SPI_RX_DATA;
		config.width = sdd->cur_bpw / 8;
		modecfg = readl(sdd->regs + S3C64XX_SPI_MODE_CFG);
		config.maxburst = modecfg & S3C64XX_SPI_MODE_4BURST ?
				S3C64XX_SPI_DMA_4BURST_LEN :
				S3C64XX_SPI_DMA_1BURST_LEN;

#ifdef CONFIG_ARM64
		sdd->ops->config((unsigned long)sdd->rx_dma.ch, &config);
#else
		sdd->ops->config((enum dma_ch)sdd->rx_dma.ch, &config);
#endif
	} else {
		sdd = container_of((void *)dma,
				   struct s3c64xx_spi_driver_data, tx_dma);
		config.direction =  sdd->tx_dma.direction;
		config.fifo = sdd->sfr_start + S3C64XX_SPI_TX_DATA;
		config.width = sdd->cur_bpw / 8;
		modecfg = readl(sdd->regs + S3C64XX_SPI_MODE_CFG);
		config.maxburst = modecfg & S3C64XX_SPI_MODE_4BURST ?
				S3C64XX_SPI_DMA_4BURST_LEN :
				S3C64XX_SPI_DMA_1BURST_LEN;

#ifdef CONFIG_ARM64
		sdd->ops->config((unsigned long)sdd->tx_dma.ch, &config);
#else
		sdd->ops->config((enum dma_ch)sdd->tx_dma.ch, &config);
#endif
	}

	info.cap = DMA_SLAVE;
	info.len = len;
	info.fp = s3c64xx_spi_dmacb;
	info.fp_param = dma;
	info.direction = dma->direction;
	info.buf = buf;

#ifdef CONFIG_ARM64
	sdd->ops->prepare((unsigned long)dma->ch, &info);
	sdd->ops->trigger((unsigned long)dma->ch);
#else
	sdd->ops->prepare((enum dma_ch)dma->ch, &info);
	sdd->ops->trigger((enum dma_ch)dma->ch);
#endif
}

static int acquire_dma(struct s3c64xx_spi_driver_data *sdd)
{
	struct samsung_dma_req req;
	struct device *dev = &sdd->pdev->dev;

	sdd->ops = samsung_dma_get_ops();

	req.cap = DMA_SLAVE;
	req.client = &s3c64xx_spi_dma_client;

	if (!sdd->rx_dma.ch)
		sdd->rx_dma.ch = (void *)sdd->ops->request(sdd->rx_dma.dmach,
							&req, dev, "rx");
	if (!sdd->tx_dma.ch)
		sdd->tx_dma.ch = (void *)sdd->ops->request(sdd->tx_dma.dmach,
							&req, dev, "tx");

	return 1;
}

static void exynos_usi_init(struct s3c64xx_spi_driver_data *sdd);
static void s3c64xx_spi_hwinit(struct s3c64xx_spi_driver_data *sdd,
			       int channel);

static int s3c64xx_spi_prepare_transfer(struct spi_master *spi)
{
	struct s3c64xx_spi_driver_data *sdd = spi_master_get_devdata(spi);
	struct s3c64xx_spi_info *sci = sdd->cntrlr_info;
#ifdef CONFIG_PM
	int ret;
#endif

#ifndef CONFIG_PM
	if (sci->dma_mode == DMA_MODE) {
		/* Acquire DMA channels */
		while (!acquire_dma(sdd))
			usleep_range(10000, 11000);
	}
#endif

#ifdef CONFIG_PM
	ret = pm_runtime_get_sync(&sdd->pdev->dev);
	if (ret < 0)
		return ret;
#endif

	if (sci->need_hw_init) {
		exynos_usi_init(sdd);
		s3c64xx_spi_hwinit(sdd, sdd->port_id);
	}

	return 0;
}

static int s3c64xx_spi_unprepare_transfer(struct spi_master *spi)
{
	struct s3c64xx_spi_driver_data *sdd = spi_master_get_devdata(spi);
#ifdef CONFIG_PM
	int ret;
#endif

#ifndef CONFIG_PM
	struct s3c64xx_spi_info *sci = sdd->cntrlr_info;

	/* Free DMA channels */
	if (sci->dma_mode == DMA_MODE && sdd->is_probed && sdd->ops) {
#ifdef CONFIG_ARM64
		if (sdd->rx_dma.ch)
			sdd->ops->release((unsigned long)sdd->rx_dma.ch,
				&s3c64xx_spi_dma_client);

		if (sdd->tx_dma.ch)
			sdd->ops->release((unsigned long)sdd->tx_dma.ch,
				&s3c64xx_spi_dma_client);
#else
		if (sdd->rx_dma.ch)
			sdd->ops->release((enum dma_ch)sdd->rx_dma.ch,
				&s3c64xx_spi_dma_client);

		if (sdd->tx_dma.ch)
			sdd->ops->release((enum dma_ch)sdd->tx_dma.ch,
				&s3c64xx_spi_dma_client);
#endif
		sdd->rx_dma.ch = NULL;
		sdd->tx_dma.ch = NULL;
	}
#endif

#ifdef CONFIG_PM
	pm_runtime_mark_last_busy(&sdd->pdev->dev);
	ret = pm_runtime_put_autosuspend(&sdd->pdev->dev);
	if (ret < 0) {
		dev_err(&sdd->pdev->dev,
			"pm_runtime_put_autosuspend fails. ret: %d", ret);
		return ret;
	}
#endif

	return 0;
}

static void s3c64xx_spi_dma_stop(struct s3c64xx_spi_driver_data *sdd,
				 struct s3c64xx_spi_dma_data *dma)
{
#ifdef CONFIG_ARM64
	sdd->ops->stop((unsigned long)dma->ch);
#else
	sdd->ops->stop((enum dma_ch)dma->ch);
#endif
}

static void s3c64xx_dma_debug(struct s3c64xx_spi_driver_data *sdd,
			      struct s3c64xx_spi_dma_data *dma)
{
#ifdef CONFIG_ARM64
	sdd->ops->debug((unsigned long)dma->ch);
#else
	sdd->ops->debug((enum dma_ch)dma->ch);
#endif
}

static inline void writesb_32(void __iomem *addr, const void *buffer,
			      unsigned int count)
{
	if (count) {
		const u8 *buf = buffer;

		do {
			__raw_writel(*buf++, addr);
		} while (--count);
	}
}

static inline void writesw_32(void __iomem *addr, const void *buffer,
			      unsigned int count)
{
	if (count) {
		const u16 *buf = buffer;

		do {
			__raw_writel(*buf++, addr);
		} while (--count);
	}
}

static void enable_datapath(struct s3c64xx_spi_driver_data *sdd,
			    struct spi_device *spi,
			    struct spi_transfer *xfer, int dma_mode)
{
	void __iomem *regs = sdd->regs;
	u32 modecfg, chcfg, dma_burst_len, packet_cnt_en;
	struct s3c64xx_spi_csinfo *cs = spi->controller_data;

	chcfg = readl(regs + S3C64XX_SPI_CH_CFG);
	chcfg &= ~S3C64XX_SPI_CH_TXCH_ON;

	modecfg = readl(regs + S3C64XX_SPI_MODE_CFG);
	modecfg &= ~S3C64XX_SPI_MODE_4BURST;

	if (dma_mode) {
		chcfg &= ~S3C64XX_SPI_CH_RXCH_ON;

		dma_burst_len = (sdd->cur_bpw / 8) * S3C64XX_SPI_DMA_4BURST_LEN;
		if (!(xfer->len % dma_burst_len))
			modecfg |= S3C64XX_SPI_MODE_4BURST;
	} else {
		/* Always shift in data in FIFO, even if xfer is Tx only,
		 * this helps setting PCKT_CNT value for generating clocks
		 * as exactly needed.
		 */
		chcfg |= S3C64XX_SPI_CH_RXCH_ON;

		packet_cnt_en = readl(regs + S3C64XX_SPI_PACKET_CNT);
		packet_cnt_en &= ~S3C64XX_SPI_PACKET_CNT_EN;
		writel(packet_cnt_en, regs + S3C64XX_SPI_PACKET_CNT);

		writel(((xfer->len * 8 / sdd->cur_bpw) & 0xffff),
		       regs + S3C64XX_SPI_PACKET_CNT);

		packet_cnt_en = readl(regs + S3C64XX_SPI_PACKET_CNT);
		packet_cnt_en |= S3C64XX_SPI_PACKET_CNT_EN;
		writel(packet_cnt_en, regs + S3C64XX_SPI_PACKET_CNT);
	}

	writel(modecfg, regs + S3C64XX_SPI_MODE_CFG);
	modecfg = readl(regs + S3C64XX_SPI_MODE_CFG);
	modecfg &= ~(S3C64XX_SPI_MODE_TXDMA_ON | S3C64XX_SPI_MODE_RXDMA_ON);

	if (xfer->tx_buf) {
		sdd->state |= TXBUSY;
		chcfg |= S3C64XX_SPI_CH_TXCH_ON;

		if (cs->cs_mode == MANUAL_CS_MODE && cs->cs_delay && !spi->cs_gpiod) {
			writel(chcfg, regs + S3C64XX_SPI_CH_CFG);
			udelay(cs->cs_delay);
		}

		if (dma_mode) {
			modecfg |= S3C64XX_SPI_MODE_TXDMA_ON;
			prepare_dma(&sdd->tx_dma, xfer->len, xfer->tx_dma);
		} else {
			switch (sdd->cur_bpw) {
			case 32:
				iowrite32_rep(regs + S3C64XX_SPI_TX_DATA,
					      xfer->tx_buf, xfer->len / 4);
				break;
			case 16:
				writesw_32(regs + S3C64XX_SPI_TX_DATA,
					   xfer->tx_buf, xfer->len / 2);
				break;
			default:
				writesb_32(regs + S3C64XX_SPI_TX_DATA,
					   xfer->tx_buf, xfer->len);
				break;
			}
		}
	}

	if (xfer->rx_buf) {
		sdd->state |= RXBUSY;

		if (sdd->port_conf->high_speed &&
		    sdd->cur_speed >= 30000000UL &&
		    !(sdd->cur_mode & SPI_CPHA))
			chcfg |= S3C64XX_SPI_CH_HS_EN;

		if (dma_mode) {
			modecfg |= S3C64XX_SPI_MODE_RXDMA_ON;
			chcfg |= S3C64XX_SPI_CH_RXCH_ON;
			writel(((xfer->len * 8 / sdd->cur_bpw) & 0xffff)
					| S3C64XX_SPI_PACKET_CNT_EN,
					regs + S3C64XX_SPI_PACKET_CNT);
			prepare_dma(&sdd->rx_dma, xfer->len, xfer->rx_dma);
		}
	}

	writel(modecfg, regs + S3C64XX_SPI_MODE_CFG);
	writel(chcfg, regs + S3C64XX_SPI_CH_CFG);
}

static inline void enable_cs(struct s3c64xx_spi_driver_data *sdd,
			     struct spi_device *spi)
{
	struct s3c64xx_spi_csinfo *cs;

	if (sdd->tgl_spi) { /* If last device toggled after mssg */
		if (sdd->tgl_spi != spi) { /* if last mssg on diff device */
			/* Deselect the last toggled device */
			cs = sdd->tgl_spi->controller_data;
			if (spi->cs_gpiod)
				gpiod_set_value(spi->cs_gpiod, 0);
			/* Quiesce the signals */
			writel(spi->mode & SPI_CS_HIGH ?
				0 : S3C64XX_SPI_SLAVE_SIG_INACT,
				sdd->regs + S3C64XX_SPI_SLAVE_SEL);
		}
		sdd->tgl_spi = NULL;
	}

	cs = spi->controller_data;
	if (spi->cs_gpiod) {
		gpiod_set_value(spi->cs_gpiod, 1);
		if (cs->cs_delay)
			udelay(cs->cs_delay);
	}

	if (cs->cs_mode == AUTO_CS_MODE ||
	    cs->cs_mode == AUTO_CS_MODE_FORCE_QUIESCE) {
		/* Set auto chip selection */
		writel(readl(sdd->regs + S3C64XX_SPI_SLAVE_SEL) |
				S3C64XX_SPI_SLAVE_AUTO |
				S3C64XX_SPI_SLAVE_NSC_CNT_2,
		sdd->regs + S3C64XX_SPI_SLAVE_SEL);
	} else {
		/* Start the signals */
		writel(spi->mode & SPI_CS_HIGH ?
				S3C64XX_SPI_SLAVE_SIG_INACT : 0,
				sdd->regs + S3C64XX_SPI_SLAVE_SEL);
	}
}

static int wait_for_xfer(struct s3c64xx_spi_driver_data *sdd,
			 struct spi_transfer *xfer, int dma_mode)
{
	void __iomem *regs = sdd->regs;
	unsigned long val;
	int ms;

	/* millisecs to xfer 'len' bytes @ 'cur_speed' */
	ms = xfer->len * 8 * 1000 / sdd->cur_speed;
	ms = (ms * 10) + 30; /* some tolerance */
	ms = max(ms, 100); /* minimum timeout */

	if (dma_mode) {
		val = msecs_to_jiffies(ms) + 10;
		val = wait_for_completion_timeout(&sdd->xfer_completion, val);
	} else {
		u32 status;

		val = msecs_to_loops(ms);
		do {
			status = readl(regs + S3C64XX_SPI_STATUS);
		} while (RX_FIFO_LVL(status, sdd) < xfer->len && --val);
	}

	if (!val)
		return -EIO;

	if (dma_mode) {
		u32 status;

		/*
		 * DmaTx returns after simply writing data in the FIFO,
		 * w/o waiting for real transmission on the bus to finish.
		 * DmaRx returns only after Dma read data from FIFO which
		 * needs bus transmission to finish, so we don't worry if
		 * Xfer involved Rx(with or without Tx).
		 */
		if (!xfer->rx_buf) {
			val = msecs_to_loops(10);
			status = readl(regs + S3C64XX_SPI_STATUS);
			while ((TX_FIFO_LVL(status, sdd) ||
				!S3C64XX_SPI_ST_TX_DONE(status, sdd)) &&
				--val) {
				cpu_relax();
				status = readl(regs + S3C64XX_SPI_STATUS);
			}

			if (!val)
				return -EIO;
		}
	} else {
		/* If it was only Tx */
		if (!xfer->rx_buf) {
			sdd->state &= ~TXBUSY;
			return 0;
		}

		switch (sdd->cur_bpw) {
		case 32:
			ioread32_rep(regs + S3C64XX_SPI_RX_DATA,
				     xfer->rx_buf, xfer->len / 4);
			break;
		case 16:
			ioread16_rep(regs + S3C64XX_SPI_RX_DATA,
				     xfer->rx_buf, xfer->len / 2);
			break;
		default:
			ioread8_rep(regs + S3C64XX_SPI_RX_DATA,
				    xfer->rx_buf, xfer->len);
			break;
		}
		sdd->state &= ~RXBUSY;
	}

	return 0;
}

static inline void disable_cs(struct s3c64xx_spi_driver_data *sdd,
			      struct spi_device *spi)
{
	struct s3c64xx_spi_csinfo *cs = spi->controller_data;

	if (sdd->tgl_spi == spi)
		sdd->tgl_spi = NULL;

	if (spi->cs_gpiod)
		gpiod_set_value(spi->cs_gpiod, 0);

	if (cs->cs_mode != AUTO_CS_MODE) {
		/* Quiesce the signals */
		writel(spi->mode & SPI_CS_HIGH ?
				0 : S3C64XX_SPI_SLAVE_SIG_INACT,
				sdd->regs + S3C64XX_SPI_SLAVE_SEL);
	}
}

static void s3c64xx_spi_config(struct s3c64xx_spi_driver_data *sdd)
{
	struct s3c64xx_spi_info *sci = sdd->cntrlr_info;
	void __iomem *regs = sdd->regs;
	u32 val;
	int ret;

	/* Disable Clock */
	if (!sdd->port_conf->clk_from_cmu) {
		val = readl(regs + S3C64XX_SPI_CLK_CFG);
		val &= ~S3C64XX_SPI_ENCLK_ENABLE;
		writel(val, regs + S3C64XX_SPI_CLK_CFG);
	}

	/* Set Polarity and Phase */
	val = readl(regs + S3C64XX_SPI_CH_CFG);
	val &= ~(S3C64XX_SPI_CH_SLAVE |
			S3C64XX_SPI_CPOL_L |
			S3C64XX_SPI_CPHA_B);

	if (sdd->cur_mode & SPI_CPOL)
		val |= S3C64XX_SPI_CPOL_L;

	if (sdd->cur_mode & SPI_CPHA)
		val |= S3C64XX_SPI_CPHA_B;

	writel(val, regs + S3C64XX_SPI_CH_CFG);

	/* Set Channel & DMA Mode */
	val = readl(regs + S3C64XX_SPI_MODE_CFG);
	val &= ~(S3C64XX_SPI_MODE_BUS_TSZ_MASK |
			S3C64XX_SPI_MODE_CH_TSZ_MASK);

	switch (sdd->cur_bpw) {
	case 32:
		val |= S3C64XX_SPI_MODE_BUS_TSZ_WORD;
		val |= S3C64XX_SPI_MODE_CH_TSZ_WORD;
		if (sci->swap_mode == SWAP_MODE)
			writel(S3C64XX_SPI_SWAP_TX_EN |
				S3C64XX_SPI_SWAP_TX_BYTE |
				S3C64XX_SPI_SWAP_TX_HALF_WORD |
				S3C64XX_SPI_SWAP_RX_EN |
				S3C64XX_SPI_SWAP_RX_BYTE |
				S3C64XX_SPI_SWAP_RX_HALF_WORD,
				regs + S3C64XX_SPI_SWAP_CFG);
		break;
	case 16:
		val |= S3C64XX_SPI_MODE_BUS_TSZ_HALFWORD;
		val |= S3C64XX_SPI_MODE_CH_TSZ_HALFWORD;
		if (sci->swap_mode == SWAP_MODE)
			writel(S3C64XX_SPI_SWAP_TX_EN |
				S3C64XX_SPI_SWAP_TX_BYTE |
				S3C64XX_SPI_SWAP_RX_EN |
				S3C64XX_SPI_SWAP_RX_BYTE,
				regs + S3C64XX_SPI_SWAP_CFG);
		break;
	default:
		val |= S3C64XX_SPI_MODE_BUS_TSZ_BYTE;
		val |= S3C64XX_SPI_MODE_CH_TSZ_BYTE;
		if (sci->swap_mode == SWAP_MODE)
			writel(0, regs + S3C64XX_SPI_SWAP_CFG);
		break;
	}

	if (sci->dbg_mode & SPI_LOOPBACK_MODE) {
		dev_info(&sdd->pdev->dev, "Change Loopback mode!\n");
		val |= S3C64XX_SPI_MODE_SELF_LOOPBACK;
	}

	writel(val, regs + S3C64XX_SPI_MODE_CFG);

	if (sdd->port_conf->clk_from_cmu) {
		if (clk_get_rate(sdd->src_clk) != (sdd->cur_speed * 4)) {
		/* There is a quarter-multiplier before the SPI */
			ret = clk_set_rate(sdd->src_clk, sdd->cur_speed * 4);
			if (ret < 0)
				dev_err(&sdd->pdev->dev, "SPI clk set failed\n");
			else
				dev_dbg(&sdd->pdev->dev, "Set SPI clock rate: %u(%lu)\n",
					 sdd->cur_speed,
					clk_get_rate(sdd->src_clk));
		} else {
			dev_dbg(&sdd->pdev->dev, "Set SPI clock rate: %u(%lu)\n",
				 sdd->cur_speed, clk_get_rate(sdd->src_clk));
		}

	} else {
		/* Configure Clock */
		val = readl(regs + S3C64XX_SPI_CLK_CFG);
		val &= ~S3C64XX_SPI_PSR_MASK;
		val |= ((clk_get_rate(sdd->src_clk) / sdd->cur_speed / 2 - 1) &
				S3C64XX_SPI_PSR_MASK);
		writel(val, regs + S3C64XX_SPI_CLK_CFG);

		/* Enable Clock */
		val = readl(regs + S3C64XX_SPI_CLK_CFG);
		val |= S3C64XX_SPI_ENCLK_ENABLE;
		writel(val, regs + S3C64XX_SPI_CLK_CFG);
	}

	if (sci->dbg_mode & SPI_DBG_MODE) {
		dev_info(&sdd->pdev->dev, "SPI_MODE_%d", sdd->cur_mode & 0x3);
		dev_info(&sdd->pdev->dev, "BTS : %d", sdd->cur_bpw);
		dev_info(&sdd->pdev->dev, "Set SPI clock rate: %u(%lu)\n",
					 sdd->cur_speed,
					clk_get_rate(sdd->src_clk));
	}
}

static int s3c64xx_spi_dma_initialize(struct s3c64xx_spi_driver_data *sdd,
				      struct spi_message *msg)
{
	struct spi_transfer *xfer;

	/* First mark all xfer unmapped */
	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		xfer->rx_dma = DMA_MAPPING_ERROR;
		xfer->tx_dma = DMA_MAPPING_ERROR;
	}

	return 0;
}

static int s3c64xx_spi_map_one_msg(struct s3c64xx_spi_driver_data *sdd,
				   struct spi_message *msg,
				   struct spi_transfer *xfer)
{
	struct s3c64xx_spi_info *sci = sdd->cntrlr_info;
	struct device *dev = &sdd->pdev->dev;

	if (msg->is_dma_mapped || sci->dma_mode != DMA_MODE)
		return 0;

	if (xfer->len <= ((FIFO_LVL_MASK(sdd) >> 1) + 1))
		return 0;

	if (xfer->tx_buf) {
		xfer->tx_dma = dma_map_single(dev, (void *)xfer->tx_buf,
					      xfer->len, DMA_TO_DEVICE);
		if (dma_mapping_error(dev, xfer->tx_dma)) {
			dev_err(dev, "dma_map_single Tx failed\n");
			xfer->tx_dma = DMA_MAPPING_ERROR;
			return -ENOMEM;
		}
	}

	if (xfer->rx_buf) {
		xfer->rx_dma = dma_map_single(dev, xfer->rx_buf,
					      xfer->len, DMA_FROM_DEVICE);
		if (dma_mapping_error(dev, xfer->rx_dma)) {
			dev_err(dev, "dma_map_single Rx failed\n");
			if (xfer->tx_dma != DMA_MAPPING_ERROR)
				dma_unmap_single(dev, xfer->tx_dma,
						 xfer->len, DMA_TO_DEVICE);
			xfer->tx_dma = DMA_MAPPING_ERROR;
			xfer->rx_dma = DMA_MAPPING_ERROR;
			return -ENOMEM;
		}
	}

	return 0;
}

static void s3c64xx_spi_unmap_one_msg(struct s3c64xx_spi_driver_data *sdd,
				      struct spi_message *msg,
				      struct spi_transfer *xfer)
{
	struct s3c64xx_spi_info *sci = sdd->cntrlr_info;
	struct device *dev = &sdd->pdev->dev;

	if (msg->is_dma_mapped || sci->dma_mode != DMA_MODE)
		return;

	if (xfer->len <= ((FIFO_LVL_MASK(sdd) >> 1) + 1))
		return;

	if (xfer->rx_buf && xfer->rx_dma != DMA_MAPPING_ERROR)
		dma_unmap_single(dev, xfer->rx_dma, xfer->len, DMA_FROM_DEVICE);

	if (xfer->tx_buf && xfer->tx_dma != DMA_MAPPING_ERROR)
		dma_unmap_single(dev, xfer->tx_dma, xfer->len, DMA_TO_DEVICE);
}

static int s3c64xx_spi_transfer_one_message(struct spi_master *master,
					    struct spi_message *msg)
{
	struct s3c64xx_spi_driver_data *sdd = spi_master_get_devdata(master);
	struct s3c64xx_spi_info *sci = sdd->cntrlr_info;
	struct spi_device *spi = msg->spi;
	struct s3c64xx_spi_csinfo *cs = spi->controller_data;
	struct spi_transfer *xfer;
	int status = 0, cs_toggle = 0;
	const void *origin_tx_buf = NULL;
	void *origin_rx_buf = NULL;
	unsigned int target_len = 0, origin_len = 0;
	unsigned int fifo_lvl = (FIFO_LVL_MASK(sdd) >> 1) + 1;
	u32 speed;
	u8 bpw;

	if (!master->running) {
		dev_err(&spi->dev, "Controller is in suspend state.\n");
		return -EACCES;
	}

	/* If Master's(controller) state differs from that needed by Slave */
	if (sdd->cur_speed != spi->max_speed_hz || sdd->cur_mode != spi->mode ||
	    sdd->cur_bpw != spi->bits_per_word) {
		sdd->cur_bpw = spi->bits_per_word;
		sdd->cur_speed = spi->max_speed_hz;
		sdd->cur_mode = spi->mode;
		s3c64xx_spi_config(sdd);
	}

	if (!msg->is_dma_mapped && sci->dma_mode == DMA_MODE)
		s3c64xx_spi_dma_initialize(sdd, msg);

	/* Configure feedback delay */
	writel(cs->fb_delay & 0x3, sdd->regs + S3C64XX_SPI_FB_CLK);

	list_for_each_entry(xfer, &msg->transfers, transfer_list) {
		unsigned long flags;
		int use_dma;

		reinit_completion(&sdd->xfer_completion);

		/* Only BPW and Speed may change across transfers */
		bpw = xfer->bits_per_word;
		speed = xfer->speed_hz ? xfer->speed_hz : spi->max_speed_hz;

		if (xfer->len % (bpw / 8)) {
			dev_err(&spi->dev, "Xfer length(%u) not a multiple of word size(%u)\n",
				xfer->len, bpw / 8);
			status = -EIO;
			goto out;
		}

		if (bpw != sdd->cur_bpw || speed != sdd->cur_speed) {
			sdd->cur_bpw = bpw;
			sdd->cur_speed = speed;
			s3c64xx_spi_config(sdd);
		}

		/* verify cpu mode */
		if (sci->dma_mode != DMA_MODE) {
			use_dma = 0;

			/* backup original tx, rx buf ptr & xfer length */
			origin_tx_buf = xfer->tx_buf;
			origin_rx_buf = xfer->rx_buf;
			origin_len = xfer->len;

			target_len = xfer->len;
			if (xfer->len > fifo_lvl)
				xfer->len = fifo_lvl;
		} else {
			/* backup original tx, rx buf ptr & xfer length */
			origin_tx_buf = xfer->tx_buf;
			origin_rx_buf = xfer->rx_buf;
			origin_len = xfer->len;

			target_len = xfer->len;
			if (xfer->len > S3C64XX_SPI_PACKET_CNT_MAX *
					sdd->cur_bpw / 8)
				xfer->len = S3C64XX_SPI_PACKET_CNT_MAX *
					sdd->cur_bpw / 8;
		}
try_transfer:
		if (sci->dma_mode == DMA_MODE) {
			/* Map the transfer if needed */
			if (s3c64xx_spi_map_one_msg(sdd, msg, xfer)) {
				dev_err(&spi->dev, "Xfer: Unable to map message buffers!\n");
				status = -ENOMEM;
				goto out;
			}

			/*
			 * Polling method for xfers not bigger than FIFO
			 * capacity
			 */
			if (xfer->len <= fifo_lvl)
				use_dma = 0;
			else
				use_dma = 1;
		}

		spin_lock_irqsave(&sdd->lock, flags);

		/* Pending only which is to be done */
		sdd->state &= ~RXBUSY;
		sdd->state &= ~TXBUSY;

		if (cs->cs_mode == AUTO_CS_MODE ||
		    cs->cs_mode == AUTO_CS_MODE_FORCE_QUIESCE ||
		    (cs->cs_mode == MANUAL_CS_MODE && cs->cs_delay && !spi->cs_gpiod)) {
			/* Slave Select */
			enable_cs(sdd, spi);

			enable_datapath(sdd, spi, xfer, use_dma);
		} else {
			enable_datapath(sdd, spi, xfer, use_dma);

			/* Slave Select */
			enable_cs(sdd, spi);
		}

		spin_unlock_irqrestore(&sdd->lock, flags);

		status = wait_for_xfer(sdd, xfer, use_dma);

		if (status) {
			dev_err(&spi->dev, "I/O Error: rx-%d tx-%d res:rx-%c tx-%c len-%d\n",
				xfer->rx_buf ? 1 : 0, xfer->tx_buf ? 1 : 0,
				(sdd->state & RXBUSY) ? 'f' : 'p',
				(sdd->state & TXBUSY) ? 'f' : 'p',
				xfer->len);

			if (use_dma) {
				if (xfer->tx_buf && (sdd->state & TXBUSY)) {
					s3c64xx_dma_debug(sdd, &sdd->tx_dma);
					s3c64xx_spi_dma_stop(sdd, &sdd->tx_dma);
				}
				if (xfer->rx_buf && (sdd->state & RXBUSY)) {
					s3c64xx_dma_debug(sdd, &sdd->rx_dma);
					s3c64xx_spi_dma_stop(sdd, &sdd->rx_dma);
				}
			}

			s3c64xx_spi_dump_reg(sdd);
			flush_fifo(sdd);

			goto out;
		}

		spi_transfer_delay_exec(xfer);

		if (xfer->cs_change) {
			/*
			 * Hint that the next mssg is gonna be
			 * for the same device
			 */
			if (list_is_last(&xfer->transfer_list,
					 &msg->transfers))
				cs_toggle = 1;
		}

		msg->actual_length += xfer->len;

		flush_fifo(sdd);

		if (sci->dma_mode != DMA_MODE) {
			target_len -= xfer->len;

			if (xfer->tx_buf)
				xfer->tx_buf += xfer->len;

			if (xfer->rx_buf)
				xfer->rx_buf += xfer->len;

			if (target_len > 0) {
				if (target_len > fifo_lvl)
					xfer->len = fifo_lvl;
				else
					xfer->len = target_len;
				goto try_transfer;
			}

			/* restore original tx, rx buf_ptr & xfer length */
			xfer->tx_buf = origin_tx_buf;
			xfer->rx_buf = origin_rx_buf;
			xfer->len = origin_len;
		} else {
			s3c64xx_spi_unmap_one_msg(sdd, msg, xfer);

			target_len -= xfer->len;

			if (xfer->tx_buf)
				xfer->tx_buf += xfer->len;

			if (xfer->rx_buf)
				xfer->rx_buf += xfer->len;

			if (target_len > 0) {
				if (target_len > S3C64XX_SPI_PACKET_CNT_MAX *
						sdd->cur_bpw / 8)
					xfer->len = S3C64XX_SPI_PACKET_CNT_MAX *
						sdd->cur_bpw / 8;
				else
					xfer->len = target_len;
				goto try_transfer;
			}

			/* restore original tx, rx buf_ptr & xfer length */
			xfer->tx_buf = origin_tx_buf;
			xfer->rx_buf = origin_rx_buf;
			xfer->len = origin_len;
		}
	}

out:
	if (!cs_toggle || status)
		disable_cs(sdd, spi);
	else
		sdd->tgl_spi = spi;

	msg->status = status;

	if(master->cur_msg)
		spi_finalize_current_message(master);
	else {
		dev_err(&master->dev, "Controller lost cur_msg!\n");
		return -EINVAL;
	}

	return 0;
}

static struct s3c64xx_spi_csinfo *s3c64xx_get_slave_ctrldata
(struct spi_device *spi)
{
	struct s3c64xx_spi_csinfo *cs;
	struct device_node *slave_np, *data_np = NULL;
	u32 fb_delay = 0;
	u32 cs_mode = 0;
	u32 cs_delay = 0;

	slave_np = spi->dev.of_node;
	if (!slave_np) {
		dev_err(&spi->dev, "device node not found\n");
		return ERR_PTR(-EINVAL);
	}

	cs = kzalloc(sizeof(*cs), GFP_KERNEL);
	if (!cs) {
		return ERR_PTR(-ENOMEM);
	}

	// Setting defaults in case there's no controller-data
	cs->cs_init_state = 1;
	cs->cs_mode = AUTO_CS_MODE;

	data_np = of_get_child_by_name(slave_np, "controller-data");
	if (!data_np) {
		dev_warn(&spi->dev, "child node 'controller-data' not found\n");
		return cs;
	}

	if (!of_property_read_u32(data_np, "cs-clock-delay", &cs_delay))
		cs->cs_delay = min(cs_delay, (u32) MAX_CS_CLOCK_DELAY_US);

	of_property_read_u32(data_np, "cs-init-state", &cs->cs_init_state);

	of_property_read_u32(data_np, "samsung,spi-feedback-delay", &fb_delay);
	cs->fb_delay = fb_delay;

	if (!of_property_read_u32(data_np,
				  "samsung,spi-chip-select-mode", &cs_mode)) {
		if (cs_mode == 1)
			cs->cs_mode = AUTO_CS_MODE;
		else if (cs_mode == 2)
			cs->cs_mode = AUTO_CS_MODE_FORCE_QUIESCE;
		else
			cs->cs_mode = MANUAL_CS_MODE;
	}

	of_node_put(data_np);
	return cs;
}

/*
 * Here we only check the validity of requested configuration
 * and save the configuration in a local data-structure.
 * The controller is actually configured only just before we
 * get a message to transfer.
 */
static int s3c64xx_spi_setup(struct spi_device *spi)
{
	struct s3c64xx_spi_csinfo *cs = spi->controller_data;
	struct s3c64xx_spi_driver_data *sdd;
	struct s3c64xx_spi_info *sci;
	int err;

	sdd = spi_master_get_devdata(spi->master);
	if (!cs && spi->dev.of_node) {
		cs = s3c64xx_get_slave_ctrldata(spi);
		spi->controller_data = cs;
	}

	if (IS_ERR(cs)) {
		dev_err(&spi->dev, "No CS for SPI(%d)\n", spi->chip_select);
		return -ENODEV;
	}

	sci = sdd->cntrlr_info;

	if (sci->secure_mode == SECURE_MODE) {
		dev_info(&spi->dev, "spi configuration for secure channel is skipped\n");
		return 0;
	}

	if (!spi_get_ctldata(spi))
		spi_set_ctldata(spi, cs);

	if (spi->bits_per_word != 8 && spi->bits_per_word != 16 &&
	    spi->bits_per_word != 32) {
		dev_err(&spi->dev, "setup: %dbits/wrd not supported!\n",
			spi->bits_per_word);
		err = -EINVAL;
		goto setup_exit;
	}

#ifdef CONFIG_PM
	pm_runtime_get_sync(&sdd->pdev->dev);
#endif

	/* Check if we can provide the requested rate */
	if (!sdd->port_conf->clk_from_cmu) {
		u32 psr, speed;

		/* Max possible */
		speed = (unsigned int)clk_get_rate(sdd->src_clk) / 2 / (0 + 1);
		if (!speed) {
			dev_err(&spi->dev, "clock rate of speed is 0\n");
			err = -EINVAL;
			goto setup_exit;
		}

		if (spi->max_speed_hz > speed)
			spi->max_speed_hz = speed;

		psr = clk_get_rate(sdd->src_clk) / 2 /
			spi->max_speed_hz - 1;
		psr &= S3C64XX_SPI_PSR_MASK;
		if (psr == S3C64XX_SPI_PSR_MASK)
			psr--;

		speed = clk_get_rate(sdd->src_clk) / 2 /
			(psr + 1);
		if (spi->max_speed_hz < speed) {
			if (psr + 1 < S3C64XX_SPI_PSR_MASK) {
				psr++;
			} else {
				err = -EINVAL;
				goto setup_exit;
			}
		}

		speed = clk_get_rate(sdd->src_clk) / 2 /
			(psr + 1);
		if (spi->max_speed_hz >= speed) {
			spi->max_speed_hz = speed;
		} else {
			dev_err(&spi->dev, "Can't set %dHz transfer speed\n",
				spi->max_speed_hz);
			err = -EINVAL;
			goto setup_exit;
		}
	}

	if (cs->cs_init_state) {
		disable_cs(sdd, spi);
	} else {
		/* When cs_init_state is set to 0, we want to configure the CS GPIO to be
		 * low on init. The current spi core driver doesn't let us do that when the
		 * spi driver is configured as active low. So set it here. Refer to
		 * b/181043058 for more details.
		 */
		if (spi->cs_gpiod)
			gpiod_set_raw_value(spi->cs_gpiod, 0);
	}

#ifdef CONFIG_PM
	pm_runtime_mark_last_busy(&sdd->pdev->dev);
	pm_runtime_put_autosuspend(&sdd->pdev->dev);
#endif

	if (sci->dbg_mode & SPI_DBG_MODE) {
		dev_info(&spi->dev, "SPI feedback-delay : %d\n", cs->fb_delay);
		dev_info(&spi->dev, "SPI clock : %u(%lu)\n",
			 sdd->cur_speed, clk_get_rate(sdd->src_clk));
		dev_info(&spi->dev, "SPI %s CS mode", cs->cs_mode ?
				"AUTO" : "MANUAL");
	}

	return 0;

setup_exit:
	/* setup() returns with device de-selected */
	if (cs->cs_init_state) {
		disable_cs(sdd, spi);
	} else {
		/* When cs_init_state is set to 0, we want to configure the CS GPIO to be
		 * low on init. The current spi core driver doesn't let us do that when the
		 * spi driver is configured as active low. So set it here. Refer to
		 * b/181043058 for more details.
		 */
		if (spi->cs_gpiod)
			gpiod_set_raw_value(spi->cs_gpiod, 0);
	}

	spi_set_ctldata(spi, NULL);

	if (spi->dev.of_node)
		kfree(cs);

	return err;
}

static void s3c64xx_spi_cleanup(struct spi_device *spi)
{
	struct s3c64xx_spi_csinfo *cs = spi_get_ctldata(spi);

	if (cs) {
		if (spi->dev.of_node)
			kfree(cs);
	}
	spi_set_ctldata(spi, NULL);
}

static irqreturn_t s3c64xx_spi_irq(int irq, void *data)
{
	struct s3c64xx_spi_driver_data *sdd = data;
	struct spi_master *spi = sdd->master;
	unsigned int val, clr = 0;

	val = readl(sdd->regs + S3C64XX_SPI_STATUS);

	if (val & S3C64XX_SPI_ST_RX_OVERRUN_ERR) {
		clr = S3C64XX_SPI_PND_RX_OVERRUN_CLR;
		dev_err(&spi->dev, "RX overrun\n");
	}
	if (val & S3C64XX_SPI_ST_RX_UNDERRUN_ERR) {
		clr |= S3C64XX_SPI_PND_RX_UNDERRUN_CLR;
		dev_err(&spi->dev, "RX underrun\n");
	}
	if (val & S3C64XX_SPI_ST_TX_OVERRUN_ERR) {
		clr |= S3C64XX_SPI_PND_TX_OVERRUN_CLR;
		dev_err(&spi->dev, "TX overrun\n");
	}
	if (val & S3C64XX_SPI_ST_TX_UNDERRUN_ERR) {
		clr |= S3C64XX_SPI_PND_TX_UNDERRUN_CLR;
		dev_err(&spi->dev, "TX underrun\n");
	}

	/* Clear the pending irq by setting and then clearing it */
	writel(clr, sdd->regs + S3C64XX_SPI_PENDING_CLR);
	writel(0, sdd->regs + S3C64XX_SPI_PENDING_CLR);

	return IRQ_HANDLED;
}

static void exynos_usi_init(struct s3c64xx_spi_driver_data *sdd)
{
	struct s3c64xx_spi_info *sci = sdd->cntrlr_info;
	void __iomem *regs = sdd->regs;

	/* USI_RESET is active High signal.
	 * Reset value of USI_RESET is 'h1 to drive stable value to PAD.
	 * Due to this feature, the USI_RESET must be cleared (set as '0')
	 * before transaction starts.
	 */

	if (sci->secure_mode == SECURE_MODE)
		return;

	writel(USI_RESET, regs + USI_CON);
}

static void s3c64xx_spi_hwinit(struct s3c64xx_spi_driver_data *sdd, int channel)
{
	struct s3c64xx_spi_info *sci = sdd->cntrlr_info;
	void __iomem *regs = sdd->regs;
	unsigned int val;

	if (sci->secure_mode == SECURE_MODE)
		return;

	sdd->cur_speed = 0;

	writel(S3C64XX_SPI_SLAVE_SIG_INACT, sdd->regs + S3C64XX_SPI_SLAVE_SEL);

	/* Disable Interrupts - we use Polling if not DMA mode */
	writel(0, regs + S3C64XX_SPI_INT_EN);

	if (!sdd->port_conf->clk_from_cmu)
		writel(sci->src_clk_nr << S3C64XX_SPI_CLKSEL_SRCSHFT,
		       regs + S3C64XX_SPI_CLK_CFG);
	writel(0, regs + S3C64XX_SPI_MODE_CFG);
	writel(0, regs + S3C64XX_SPI_PACKET_CNT);

	/* Clear any irq pending bits, should set and clear the bits */
	val = S3C64XX_SPI_PND_RX_OVERRUN_CLR |
		S3C64XX_SPI_PND_RX_UNDERRUN_CLR |
		S3C64XX_SPI_PND_TX_OVERRUN_CLR |
		S3C64XX_SPI_PND_TX_UNDERRUN_CLR;
	writel(val, regs + S3C64XX_SPI_PENDING_CLR);
	writel(0, regs + S3C64XX_SPI_PENDING_CLR);

	writel(0, regs + S3C64XX_SPI_SWAP_CFG);

	val = readl(regs + S3C64XX_SPI_MODE_CFG);
	val &= ~S3C64XX_SPI_MODE_4BURST;
	val &= ~(S3C64XX_SPI_MAX_TRAILCNT << S3C64XX_SPI_TRAILCNT_OFF);
	val |= (S3C64XX_SPI_TRAILCNT << S3C64XX_SPI_TRAILCNT_OFF);
	writel(val, regs + S3C64XX_SPI_MODE_CFG);

	flush_fifo(sdd);

	sci->need_hw_init = 0;
}

#ifdef CONFIG_OF
static struct s3c64xx_spi_info *s3c64xx_spi_parse_dt(struct device *dev)
{
	struct s3c64xx_spi_info *sci;
	u32 temp;

	sci = devm_kzalloc(dev, sizeof(*sci), GFP_KERNEL);
	if (!sci)
		return ERR_PTR(-ENOMEM);

	if (of_get_property(dev->of_node, "dma-mode", NULL))
		sci->dma_mode = DMA_MODE;
	else
		sci->dma_mode = CPU_MODE;

	if (!of_property_read_u32(dev->of_node, "swap-mode", &temp) &&
	    !(temp == 0))
		sci->swap_mode = SWAP_MODE;
	else
		sci->swap_mode = NO_SWAP_MODE;

	if (of_get_property(dev->of_node, "secure-mode", NULL)) {
		dev_info(dev, "secure-mode. Accessing spi register isn't allowed\n");
		sci->secure_mode = SECURE_MODE;
	} else {
		sci->secure_mode = NONSECURE_MODE;
	}

	if (of_property_read_u32(dev->of_node, "samsung,spi-src-clk", &temp)) {
		dev_warn(dev, "spi bus clock parent not specified, using clock at index 0 as parent\n");
		sci->src_clk_nr = 0;
	} else {
		sci->src_clk_nr = temp;
	}

	if (of_property_read_u32(dev->of_node, "num-cs", &temp)) {
		dev_warn(dev, "number of chip select lines not specified, assuming 1 chip select line\n");
		sci->num_cs = 1;
	} else {
		sci->num_cs = temp;
	}

	sci->domain = !!of_get_property(dev->of_node, "power-domains", NULL);

	return sci;
}
#else
static struct s3c64xx_spi_info *s3c64xx_spi_parse_dt(struct device *dev)
{
	return dev_get_platdata(dev);
}
#endif

static const struct of_device_id s3c64xx_spi_dt_match[];

static inline struct s3c64xx_spi_port_config *s3c64xx_spi_get_port_config(
						struct platform_device *pdev)
{
#ifdef CONFIG_OF
	if (pdev->dev.of_node) {
		const struct of_device_id *match;

		match = of_match_node(s3c64xx_spi_dt_match, pdev->dev.of_node);
		return (struct s3c64xx_spi_port_config *)match->data;
	}
#endif
	return (struct s3c64xx_spi_port_config *)platform_get_device_id(pdev)->driver_data;
}

static int s3c64xx_spi_probe(struct platform_device *pdev)
{
	struct resource	*mem_res;
	struct resource	*res;
	struct device *dev = &pdev->dev;
	struct s3c64xx_spi_driver_data *sdd;
	struct s3c64xx_spi_info *sci = dev_get_platdata(&pdev->dev);
	struct spi_master *master;
	int ret, irq;
	char clk_name[16];
	int fifosize;

	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(36));
	if (ret)
		return ret;

	if (!sci && pdev->dev.of_node) {
		sci = s3c64xx_spi_parse_dt(&pdev->dev);
		if (IS_ERR(sci))
			return PTR_ERR(sci);
	}

	if (!sci) {
		dev_err(&pdev->dev, "platform_data missing!\n");
		return -ENODEV;
	}

	if (of_property_read_u32(dev->of_node, "samsung,usi-offset",
				 &sci->usi_offset))
		dev_warn(dev, "usi offset is not specified\n");

	sci->usi_reg = syscon_regmap_lookup_by_phandle(dev->of_node,
						       "samsung,usi-phandle");
	if (sci->secure_mode != SECURE_MODE) {
		if (IS_ERR(sci->usi_reg)) {
			dev_info(dev, "no lookup for usi-phandle.\n");
		} else {
			u32 val;

			regmap_update_bits(sci->usi_reg, sci->usi_offset,
					   USI_SW_CONF_MASK, USI_SPI_SW_CONF);
			regmap_read(sci->usi_reg, sci->usi_offset, &val);
			dev_info(dev, "set usi mode - 0x%x\n", val &
					USI_SW_CONF_MASK);
		}
	}

	mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem_res) {
		dev_err(&pdev->dev, "Unable to get SPI MEM resource\n");
		return -ENXIO;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_warn(&pdev->dev, "Failed to get IRQ: %d\n", irq);
		return irq;
	}

	master = spi_alloc_master(&pdev->dev,
				  sizeof(struct s3c64xx_spi_driver_data));
	if (!master) {
		dev_err(&pdev->dev, "Unable to allocate SPI Master\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, master);

	sdd = spi_master_get_devdata(master);
	sdd->port_conf = s3c64xx_spi_get_port_config(pdev);
	sdd->master = master;
	sdd->cntrlr_info = sci;
	sdd->pdev = pdev;
	sdd->sfr_start = mem_res->start;
	sdd->is_probed = 0;
	sdd->ops = NULL;

#ifdef CONFIG_CPU_IDLE
	sdd->idle_ip_index = exynos_get_idle_ip_index(dev_name(&pdev->dev));
#endif

	if (pdev->dev.of_node) {
		ret = of_alias_get_id(pdev->dev.of_node, "spi");
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to get alias id, errno %d\n",
				ret);
			goto err0;
		}
		pdev->id = ret;
		sdd->port_id = ret;
	} else {
		sdd->port_id = pdev->id;
	}

	if (sdd->port_id >= MAX_SPI_PORTS) {
		dev_err(&pdev->dev, "the port %d exceeded MAX_SPI_PORTS(%d)\n",
			sdd->port_id, MAX_SPI_PORTS);
		goto err0;
	}

	sdd->cur_bpw = 8;

	if (sci->dma_mode == DMA_MODE) {
		if (!sdd->pdev->dev.of_node) {
			res = platform_get_resource(pdev, IORESOURCE_DMA,  0);
			if (!res) {
				dev_err(&pdev->dev,
					"Unable to get SPI tx dma resource\n");
				return -ENXIO;
			}
			sdd->tx_dma.dmach = res->start;

			res = platform_get_resource(pdev, IORESOURCE_DMA,  1);
			if (!res) {
				dev_err(&pdev->dev,
					"Unable to get SPI rx dma resource\n");
				return -ENXIO;
			}
			sdd->rx_dma.dmach = res->start;
		}

		sdd->tx_dma.direction = DMA_MEM_TO_DEV;
		sdd->rx_dma.direction = DMA_DEV_TO_MEM;
	}

	master->dev.of_node = pdev->dev.of_node;
	master->bus_num = sdd->port_id;
	master->setup = s3c64xx_spi_setup;
	master->cleanup = s3c64xx_spi_cleanup;
	master->prepare_transfer_hardware = s3c64xx_spi_prepare_transfer;
	master->transfer_one_message = s3c64xx_spi_transfer_one_message;
	master->unprepare_transfer_hardware = s3c64xx_spi_unprepare_transfer;
	master->use_gpio_descriptors = true;
	master->dma_alignment = 8;
	master->bits_per_word_mask = BIT(32 - 1) | BIT(16 - 1) | BIT(8 - 1);
	/* the spi->mode bits understood by this driver: */
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH;

	sdd->regs = devm_ioremap_resource(&pdev->dev, mem_res);
	if (IS_ERR(sdd->regs)) {
		ret = PTR_ERR(sdd->regs);
		goto err0;
	}

	if (sci->cfg_gpio && sci->cfg_gpio()) {
		dev_err(&pdev->dev, "Unable to config gpio\n");
		ret = -EBUSY;
		goto err0;
	}

	/* Setup clocks */
	sdd->clk = devm_clk_get(&pdev->dev, "gate_spi_clk");
	if (IS_ERR(sdd->clk)) {
		dev_err(&pdev->dev, "Unable to acquire clock 'spi'\n");
		ret = PTR_ERR(sdd->clk);
		goto err0;
	}

	sdd->src_clk = devm_clk_get(&pdev->dev, "ipclk_spi");
	if (IS_ERR(sdd->src_clk)) {
		dev_err(&pdev->dev,
			"Unable to acquire clock '%s'\n", clk_name);
		ret = PTR_ERR(sdd->src_clk);
		goto err2;
	}
#ifdef CONFIG_PM
	pm_runtime_use_autosuspend(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	pm_runtime_get_sync(&pdev->dev);

	sdd->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(sdd->pinctrl)) {
		dev_warn(&pdev->dev, "Couldn't get pinctrl.\n");
		sdd->pinctrl = NULL;
	}

	if (sdd->pinctrl) {
		sdd->pin_def = pinctrl_lookup_state(sdd->pinctrl,
						    PINCTRL_STATE_DEFAULT);
		if (IS_ERR(sdd->pin_def)) {
			dev_warn(&pdev->dev, "Default state not defined\n");
			sdd->pin_def = NULL;
		}

		sdd->pin_idle = pinctrl_lookup_state(sdd->pinctrl,
						     PINCTRL_STATE_IDLE);
		if (IS_ERR(sdd->pin_idle)) {
			dev_info(&pdev->dev, "Not using idle state\n");
			sdd->pin_idle = NULL;
		}
	}
#else
#ifdef CONFIG_CPU_IDLE
	exynos_update_ip_idle_status(sdd->idle_ip_index, 0);
#endif

	if (clk_prepare_enable(sdd->clk)) {
		dev_err(&pdev->dev, "Couldn't enable clock 'spi'\n");
		ret = -EBUSY;
		goto err0;
	}

	if (clk_prepare_enable(sdd->src_clk)) {
		dev_err(&pdev->dev, "Couldn't enable clock '%s'\n", clk_name);
		ret = -EBUSY;
		goto err2;
	}
#endif

	if (of_property_read_u32(pdev->dev.of_node, "spi-clkoff-time",
				 (int *)&sdd->spi_clkoff_time)) {
		dev_info(&pdev->dev, "spi clkoff-time is empty(Default: 0ms)\n");
		sdd->spi_clkoff_time = 0;
	} else {
		dev_info(&pdev->dev, "spi clkoff-time %d\n",
			 sdd->spi_clkoff_time);
	}

	if (of_property_read_u32(pdev->dev.of_node,
				 "samsung,spi-fifosize", &fifosize)) {
		dev_err(&pdev->dev, "PORT %d fifosize is not specified\n",
			sdd->port_id);
		ret = -EINVAL;
		goto err3;
	} else {
		sdd->port_conf->fifo_lvl_mask[sdd->port_id] =
			(fifosize << 1) - 1;
		dev_info(&pdev->dev, "PORT %d fifo_lvl_mask = 0x%x\n",
			 sdd->port_id,
				sdd->port_conf->fifo_lvl_mask[sdd->port_id]);
	}

	exynos_usi_init(sdd);

	/* Setup Deufult Mode */
	s3c64xx_spi_hwinit(sdd, sdd->port_id);

	spin_lock_init(&sdd->lock);
	init_completion(&sdd->xfer_completion);

	ret = devm_request_irq(&pdev->dev, irq, s3c64xx_spi_irq, 0,
			       "spi-s3c64xx", sdd);
	if (ret != 0) {
		dev_err(&pdev->dev, "Failed to request IRQ %d: %d\n",
			irq, ret);
		goto err3;
	}

	if (sci->secure_mode != SECURE_MODE)
		writel(S3C64XX_SPI_INT_RX_OVERRUN_EN |
				S3C64XX_SPI_INT_RX_UNDERRUN_EN |
				S3C64XX_SPI_INT_TX_OVERRUN_EN |
				S3C64XX_SPI_INT_TX_UNDERRUN_EN,
				sdd->regs + S3C64XX_SPI_INT_EN);

#ifdef CONFIG_PM
	pm_runtime_mark_last_busy(&pdev->dev);
	pm_runtime_put_sync(&pdev->dev);
#endif

	if (spi_register_master(master)) {
		dev_err(&pdev->dev, "cannot register SPI master\n");
		ret = -EBUSY;
		goto err3;
	}

	list_add_tail(&sci->node, &drvdata_list);

	sdd->is_probed = 1;
#ifdef CONFIG_PM
	if (sci->domain == NO_POWER_DOMAIN)
		pm_runtime_set_autosuspend_delay(&pdev->dev,
						 sdd->spi_clkoff_time);
	else
		pm_runtime_set_autosuspend_delay(&pdev->dev,
						 SPI_AUTOSUSPEND_TIMEOUT);
#endif

	dev_dbg(&pdev->dev, "Samsung SoC SPI Driver loaded for Bus SPI-%d with %d Slaves attached\n",
		sdd->port_id, master->num_chipselect);
	dev_dbg(&pdev->dev, "\tIOmem=[%pR]\tFIFO %dbytes\tDMA=[Rx-%ld, Tx-%ld]\n",
		mem_res, (FIFO_LVL_MASK(sdd) >> 1) + 1, sdd->rx_dma.dmach,
		sdd->tx_dma.dmach);

	ret = device_create_file(&pdev->dev, &dev_attr_spi_dbg);
	if (ret < 0)
		dev_err(&pdev->dev, "failed to create sysfs file.\n");
	sci->dbg_mode = 0;

	return 0;

err3:
#ifdef CONFIG_PM
	pm_runtime_disable(&pdev->dev);
#else
	clk_disable_unprepare(sdd->src_clk);
#endif
err2:
#ifndef CONFIG_PM
	clk_disable_unprepare(sdd->clk);
#endif
err0:
	platform_set_drvdata(pdev, NULL);
	spi_master_put(master);

	return ret;
}

static int s3c64xx_spi_remove(struct platform_device *pdev)
{
	struct spi_master *master = spi_master_get(platform_get_drvdata(pdev));
	struct s3c64xx_spi_driver_data *sdd = spi_master_get_devdata(master);

#ifdef CONFIG_PM
	pm_runtime_disable(&pdev->dev);
#endif

	spi_unregister_master(master);

	writel(0, sdd->regs + S3C64XX_SPI_INT_EN);

	clk_disable_unprepare(sdd->src_clk);

	clk_disable_unprepare(sdd->clk);

#ifdef CONFIG_CPU_IDLE
	exynos_update_ip_idle_status(sdd->idle_ip_index, 1);
#endif

	platform_set_drvdata(pdev, NULL);
	spi_master_put(master);

	return 0;
}

#ifdef CONFIG_PM
static void s3c64xx_spi_pin_ctrl(struct device *dev, int en)
{
	struct spi_master *master = dev_get_drvdata(dev);
	struct s3c64xx_spi_driver_data *sdd = spi_master_get_devdata(master);
	struct pinctrl_state *pin_stat;

	if (!sdd->pin_idle)
		return;

	pin_stat = en ? sdd->pin_def : sdd->pin_idle;
	if (!IS_ERR(pin_stat)) {
		if (pinctrl_select_state(sdd->pinctrl, pin_stat))
			dev_err(dev, "could not set pinctrl.\n");
	} else {
		dev_warn(dev, "pinctrl stat is null pointer.\n");
	}
}

static int s3c64xx_spi_runtime_suspend(struct device *dev)
{
	struct spi_master *master = dev_get_drvdata(dev);
	struct s3c64xx_spi_driver_data *sdd = spi_master_get_devdata(master);
	struct s3c64xx_spi_info *sci = sdd->cntrlr_info;

	clk_disable_unprepare(sdd->clk);
	clk_disable_unprepare(sdd->src_clk);

#ifdef CONFIG_CPU_IDLE
	exynos_update_ip_idle_status(sdd->idle_ip_index, 1);
#endif

	/* Free DMA channels */
	if (sci->dma_mode == DMA_MODE && sdd->is_probed && sdd->ops) {
#ifdef CONFIG_ARM64
		if (sdd->rx_dma.ch)
			sdd->ops->release((unsigned long)sdd->rx_dma.ch,
				&s3c64xx_spi_dma_client);

		if (sdd->tx_dma.ch)
			sdd->ops->release((unsigned long)sdd->tx_dma.ch,
				&s3c64xx_spi_dma_client);
#else
		if (sdd->rx_dma.ch)
			sdd->ops->release((enum dma_ch)sdd->rx_dma.ch,
				&s3c64xx_spi_dma_client);

		if (sdd->tx_dma.ch)
			sdd->ops->release((enum dma_ch)sdd->tx_dma.ch,
				&s3c64xx_spi_dma_client);
#endif
		sdd->rx_dma.ch = NULL;
		sdd->tx_dma.ch = NULL;
	}

	s3c64xx_spi_pin_ctrl(dev, 0);

	return 0;
}

static int s3c64xx_spi_runtime_resume(struct device *dev)
{
	struct spi_master *master = dev_get_drvdata(dev);
	struct s3c64xx_spi_driver_data *sdd = spi_master_get_devdata(master);
	struct s3c64xx_spi_info *sci = sdd->cntrlr_info;

	s3c64xx_spi_pin_ctrl(dev, 1);

	if (sci->dma_mode == DMA_MODE && sdd->is_probed) {
		/* Acquire DMA channels */
		while (!acquire_dma(sdd))
			usleep_range(10000, 11000);
	}

#ifdef CONFIG_CPU_IDLE
	exynos_update_ip_idle_status(sdd->idle_ip_index, 0);
#endif
	clk_prepare_enable(sdd->src_clk);
	clk_prepare_enable(sdd->clk);

	if (sci->domain != NO_POWER_DOMAIN) {
		/* To avoid SW RESET make CS low, change to I2C */
		regmap_update_bits(sci->usi_reg, sci->usi_offset,
				USI_SW_CONF_MASK, USI_I2C_SW_CONF);
	}
	exynos_usi_init(sdd);
	regmap_update_bits(sci->usi_reg, sci->usi_offset,
			USI_SW_CONF_MASK, USI_SPI_SW_CONF);
	s3c64xx_spi_hwinit(sdd, sdd->port_id);

	return 0;
}
#endif /* CONFIG_PM */

#ifdef CONFIG_PM_SLEEP
static int s3c64xx_spi_suspend_operation(struct device *dev)
{
	struct spi_master *master = dev_get_drvdata(dev);
	struct s3c64xx_spi_driver_data *sdd = spi_master_get_devdata(master);
#ifndef CONFIG_PM
	struct s3c64xx_spi_info *sci = sdd->cntrlr_info;
#endif

	int ret = spi_master_suspend(master);

	if (ret) {
		dev_warn(dev, "cannot suspend master\n");
		return ret;
	}

#ifndef CONFIG_PM
	if (sci->domain == NO_POWER_DOMAIN) {
		/* Disable the clock */
		clk_disable_unprepare(sdd->src_clk);
		clk_disable_unprepare(sdd->clk);
#ifdef CONFIG_CPU_IDLE
		exynos_update_ip_idle_status(sdd->idle_ip_index, 1);
#endif
	}
#endif
	ret = pm_runtime_force_suspend(dev);
	if (ret < 0)
		return ret;

	sdd->cur_speed = 0; /* Output Clock is stopped */

	return 0;
}

static int s3c64xx_spi_resume_operation(struct device *dev)
{
	struct spi_master *master = dev_get_drvdata(dev);
	struct s3c64xx_spi_driver_data *sdd = spi_master_get_devdata(master);
	struct s3c64xx_spi_info *sci = sdd->cntrlr_info;
	int ret;

	ret = pm_runtime_force_resume(dev);
	if (ret < 0)
		return ret;

	if (sci->domain == NO_POWER_DOMAIN) {
		/* Enable the clock */
#ifdef CONFIG_CPU_IDLE
		exynos_update_ip_idle_status(sdd->idle_ip_index, 0);
#endif
		clk_prepare_enable(sdd->src_clk);
		clk_prepare_enable(sdd->clk);

		if (sci->cfg_gpio)
			sci->cfg_gpio();

		if (sci->secure_mode) {
			sci->need_hw_init = 1;
		} else {
			exynos_usi_init(sdd);
			s3c64xx_spi_hwinit(sdd, sdd->port_id);
		}

#ifdef CONFIG_PM
		/* Disable the clock */
		clk_disable_unprepare(sdd->src_clk);
		clk_disable_unprepare(sdd->clk);
#ifdef CONFIG_CPU_IDLE
		exynos_update_ip_idle_status(sdd->idle_ip_index, 1);
#endif
#endif
	}

	/* Start the queue running */
	ret = spi_master_resume(master);
	if (ret)
		dev_err(dev, "problem starting queue (%d)\n", ret);
	else
		dev_dbg(dev, "resumed\n");

	return ret;
}

static int s3c64xx_spi_suspend(struct device *dev)
{
	struct spi_master *master = dev_get_drvdata(dev);
	struct s3c64xx_spi_driver_data *sdd = spi_master_get_devdata(master);
	struct s3c64xx_spi_info *sci = sdd->cntrlr_info;

	dev_dbg(dev, "spi suspend is handled in device suspend, dma mode = %d\n",
		sci->dma_mode);
	return s3c64xx_spi_suspend_operation(dev);
}

static int s3c64xx_spi_resume(struct device *dev)
{
	struct spi_master *master = dev_get_drvdata(dev);
	struct s3c64xx_spi_driver_data *sdd = spi_master_get_devdata(master);
	struct s3c64xx_spi_info *sci = sdd->cntrlr_info;

	dev_dbg(dev, "spi resume is handled in device resume, dma mode = %d\n",
		sci->dma_mode);
	return s3c64xx_spi_resume_operation(dev);
}

#else
static int s3c64xx_spi_suspend(struct device *dev)
{
	return 0;
}

static int s3c64xx_spi_resume(struct device *dev)
{
	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static const struct dev_pm_ops s3c64xx_spi_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(s3c64xx_spi_suspend, s3c64xx_spi_resume)
	SET_RUNTIME_PM_OPS(s3c64xx_spi_runtime_suspend,
			   s3c64xx_spi_runtime_resume, NULL)
};

static const struct s3c64xx_spi_port_config s3c2443_spi_port_config = {
	.fifo_lvl_mask	= { 0x7f },
	.rx_lvl_offset	= 13,
	.tx_st_done	= 21,
	.high_speed	= true,
};

static const struct s3c64xx_spi_port_config s3c6410_spi_port_config = {
	.fifo_lvl_mask	= { 0x7f, 0x7F },
	.rx_lvl_offset	= 13,
	.tx_st_done	= 21,
};

static const struct s3c64xx_spi_port_config s5pv210_spi_port_config = {
	.fifo_lvl_mask	= { 0x1ff, 0x7F },
	.rx_lvl_offset	= 15,
	.tx_st_done	= 25,
	.high_speed	= true,
};

static const struct s3c64xx_spi_port_config exynos4_spi_port_config = {
	.fifo_lvl_mask	= { 0x1ff, 0x7F, 0x7F },
	.rx_lvl_offset	= 15,
	.tx_st_done	= 25,
	.high_speed	= true,
	.clk_from_cmu	= true,
};

static const struct s3c64xx_spi_port_config exynos7_spi_port_config = {
	.fifo_lvl_mask	= { 0x1ff, 0x7F, 0x7F, 0x7F, 0x7F, 0x1ff},
	.rx_lvl_offset	= 15,
	.tx_st_done	= 25,
	.high_speed	= true,
	.clk_from_cmu	= true,
};

static struct s3c64xx_spi_port_config exynos5_spi_port_config = {
	.fifo_lvl_mask  = { 0x1ff, 0x7F, 0x7F, 0x1ff, 0x1ff },
	.rx_lvl_offset  = 15,
	.tx_st_done     = 25,
	.high_speed     = true,
	.clk_from_cmu   = true,
};

static struct s3c64xx_spi_port_config exynos543x_spi_port_config = {
	.fifo_lvl_mask	= { 0x1ff, 0x7F, 0x7F, 0x7F, 0x7F, 0x1ff, 0x1ff },
	.rx_lvl_offset	= 15,
	.tx_st_done	= 25,
	.high_speed	= true,
	.clk_from_cmu	= true,
};

static struct s3c64xx_spi_port_config exynos742x_spi_port_config = {
	.fifo_lvl_mask	= { 0x1ff, 0x7F, 0x7F, 0x7F, 0x7F, 0x1ff, 0x1ff,
		0x1ff },
	.rx_lvl_offset	= 15,
	.tx_st_done	= 25,
	.high_speed	= true,
	.clk_from_cmu	= true,
};

static struct s3c64xx_spi_port_config exynos758x_spi_port_config = {
	.fifo_lvl_mask	= { 0x1ff, 0x7F, 0x7F, 0x1ff, 0x1ff },
	.rx_lvl_offset	= 15,
	.tx_st_done	= 25,
	.high_speed	= true,
	.clk_from_cmu	= true,
};

static struct s3c64xx_spi_port_config exynos_spi_port_config = {
	.fifo_lvl_mask	= { 0, },
	.rx_lvl_offset	= 15,
	.tx_st_done	= 25,
	.high_speed	= true,
	.clk_from_cmu	= true,
};

static const struct platform_device_id s3c64xx_spi_driver_ids[] = {
	{
		.name		= "s3c2443-spi",
		.driver_data	= (kernel_ulong_t)&s3c2443_spi_port_config,
	}, {
		.name		= "s3c6410-spi",
		.driver_data	= (kernel_ulong_t)&s3c6410_spi_port_config,
	}, {
		.name		= "s5pv210-spi",
		.driver_data	= (kernel_ulong_t)&s5pv210_spi_port_config,
	}, {
		.name		= "exynos4210-spi",
		.driver_data	= (kernel_ulong_t)&exynos4_spi_port_config,
	}, {
		.name		= "exynos5410-spi",
		.driver_data	= (kernel_ulong_t)&exynos5_spi_port_config,
	}, {
		.name		= "exynos543x-spi",
		.driver_data	= (kernel_ulong_t)&exynos543x_spi_port_config,
	}, {
		.name		= "exynos742x-spi",
		.driver_data	= (kernel_ulong_t)&exynos742x_spi_port_config,
	}, {
		.name		= "exynos758x-spi",
		.driver_data	= (kernel_ulong_t)&exynos758x_spi_port_config,
	}, {
		.name		= "exynos-spi",
		.driver_data	= (kernel_ulong_t)&exynos_spi_port_config,
	},
	{ },
};

#ifdef CONFIG_OF
static const struct of_device_id s3c64xx_spi_dt_match[] = {
	{ .compatible = "samsung,exynos4210-spi",
			.data = (void *)&exynos4_spi_port_config,
	},
	{ .compatible = "samsung,exynos5410-spi",
			.data = (void *)&exynos5_spi_port_config,
	},
	{ .compatible = "samsung,exynos543x-spi",
			.data = (void *)&exynos543x_spi_port_config,
	},
	{ .compatible = "samsung,exynos742x-spi",
			.data = (void *)&exynos742x_spi_port_config,
	},
	{ .compatible = "samsung,exynos758x-spi",
			.data = (void *)&exynos758x_spi_port_config,
	},
	{ .compatible = "samsung,exynos-spi",
			.data = (void *)&exynos_spi_port_config,
	},
	{ },
};
MODULE_DEVICE_TABLE(of, s3c64xx_spi_dt_match);
#endif /* CONFIG_OF */

static struct platform_driver s3c64xx_spi_driver = {
	.driver = {
		.name	= "s3c64xx-spi",
		.pm = &s3c64xx_spi_pm,
		.of_match_table = of_match_ptr(s3c64xx_spi_dt_match),
	},
	.probe = s3c64xx_spi_probe,
	.remove = s3c64xx_spi_remove,
	.id_table = s3c64xx_spi_driver_ids,
};

module_platform_driver(s3c64xx_spi_driver);

MODULE_ALIAS("platform:s3c64xx-spi");
MODULE_AUTHOR("Jaswinder Singh <jassi.brar@samsung.com>");
MODULE_DESCRIPTION("S3C64XX SPI Controller Driver");
MODULE_LICENSE("GPL");
