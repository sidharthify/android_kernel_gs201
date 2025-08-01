// SPDX-License-Identifier: GPL-2.0
/*
 * dwc3-exynos.c - Samsung Exynos DWC3 Specific Glue layer
 *
 * Copyright (C) 2022 Samsung Electronics Co., Ltd.
 */
#include <dwc3/core.h> /* $(srctree)/drivers/usb/dwc3/core.h */
#include <dwc3/io.h> /* $(srctree)/drivers/usb/dwc3/io.h */
#include <dwc3/gadget.h> /* $(srctree)/drivers/usb/dwc3/gadget.h */

#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/extcon.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/usb/dwc3-exynos.h>
#include <linux/usb/gadget.h>
#include <linux/usb/of.h>
#include <linux/usb/usb_phy_generic.h>
#include <linux/workqueue.h>

#include <soc/google/exynos-cpupm.h>

#include "core-exynos.h"
#include "dwc3-exynos-ldo.h"
#include "exynos-otg.h"
#if IS_ENABLED(CONFIG_USB_XHCI_GOOG_DMA)
#include "xhci-goog-dma.h"
#endif

static const struct of_device_id exynos_dwc3_match[] = {
	{
		.compatible = "samsung,exynos9-dwusb",
	},
	{},
};
MODULE_DEVICE_TABLE(of, exynos_dwc3_match);

static int dwc3_exynos_clk_get(struct dwc3_exynos *exynos)
{
	struct device *dev = exynos->dev;
	const char **clk_ids;
	struct clk *clk;
	int clk_count;
	int ret, i;

	clk_count = of_property_count_strings(dev->of_node, "clock-names");
	if (IS_ERR_VALUE((unsigned long)clk_count)) {
		dev_err(dev, "invalid clk list in %s node", dev->of_node->name);
		return -EINVAL;
	}

	clk_ids = devm_kmalloc(dev,
			       (clk_count + 1) * sizeof(const char *),
				GFP_KERNEL);
	if (!clk_ids) {
		dev_err(dev, "failed to alloc for clock ids");
		return -ENOMEM;
	}

	for (i = 0; i < clk_count; i++) {
		ret = of_property_read_string_index(dev->of_node, "clock-names",
						    i, &clk_ids[i]);
		if (ret) {
			dev_err(dev, "failed to read clocks name %d from %s node\n",
				i, dev->of_node->name);
			return ret;
		}
		/*
		 * Check Bus clock to get clk node from DT.
		 * CAUTION : Bus clock SHOULD be defiend at the last.
		 */
		if (!strncmp(clk_ids[i], "bus", 3)) {
			exynos->bus_clock = devm_clk_get(exynos->dev, clk_ids[i]);
			if (IS_ERR_OR_NULL(exynos->bus_clock))
				dev_err(dev, "Can't get Bus clock.\n");
			else
				clk_count--;
		}
	}
	clk_ids[clk_count] = NULL;

	exynos->clocks = devm_kmalloc(exynos->dev,
				      (clk_count + 1) * sizeof(struct clk *),
				      GFP_KERNEL);
	if (!exynos->clocks)
		return -ENOMEM;

	for (i = 0; clk_ids[i]; i++) {
		clk = devm_clk_get(exynos->dev, clk_ids[i]);
		if (IS_ERR_OR_NULL(clk))
			goto err;

		exynos->clocks[i] = clk;
	}
	exynos->clocks[i] = NULL;

	return 0;

err:
	dev_err(exynos->dev, "couldn't get %s clock\n", clk_ids[i]);
	return -EINVAL;
}

static int dwc3_exynos_clk_prepare_enable(struct dwc3_exynos *exynos)
{
	int i;
	int ret;

	for (i = 0; exynos->clocks[i]; i++) {
		ret = clk_prepare_enable(exynos->clocks[i]);
		if (ret)
			goto err;
	}

	return 0;

err:
	dev_err(exynos->dev, "couldn't enable clock[%d]\n", i);

	/* roll back */
	for (i = i - 1; i >= 0; i--)
		clk_disable_unprepare(exynos->clocks[i]);

	return ret;
}

static void dwc3_exynos_clk_disable_unprepare(struct dwc3_exynos *exynos)
{
	int i;

	for (i = 0; exynos->clocks[i]; i++){
		clk_disable_unprepare(exynos->clocks[i]);
	}
}

static void dwc3_core_config(struct dwc3 *dwc, struct dwc3_exynos *exynos)
{
	u32 reg;

	/* AHB bus configuration */
	reg = dwc3_exynos_readl(dwc->regs, DWC3_GSBUSCFG0);
	reg |= (DWC3_GSBUSCFG0_INCRBRSTEN | DWC3_GSBUSCFG0_INCR16BRSTEN);
	/**
	 * AXI Bus' cache type configuration for DMA transfer.
	 * By below setting, cache type was set to Cacheable/Modifiable.
	 * From DWC USB3.0 Link version 2.20A, this cache type could be set.
	 */
	if (!DWC3_VER_IS_PRIOR(DWC3, 220A))
		reg |= (DWC3_GSBUSCFG0_DESWRREQINFO |
			DWC3_GSBUSCFG0_DATWRREQINFO |
			DWC3_GSBUSCFG0_DESRDREQINFO |
			DWC3_GSBUSCFG0_DATRDREQINFO);
	dwc3_exynos_writel(dwc->regs, DWC3_GSBUSCFG0, reg);

	reg = dwc3_exynos_readl(dwc->regs, DWC3_GSBUSCFG1);

	if (DWC3_VER_IS(DWC31, 170A)) {
		/*
		 * Setting MO request limit to 8 resolved ITMON issue
		 * on MTP and DM functions. Further investigation
		 * should be done by design team.
		 */
		reg |= (DWC3_GSBUSCFG1_BREQLIMIT(0x8));
		dwc3_exynos_writel(dwc->regs, DWC3_GSBUSCFG1, reg);
	}

	/*
	 * WORKAROUND:
	 * For ss bulk-in data packet, when the host detects
	 * a DPP error or the internal buffer becomes full,
	 * it retries with an ACK TP Retry=1. Under the following
	 * conditions, the Retry=1 is falsely carried over to the next
	 * DWC3_GUCTL_USBHSTINAUTORETRYEN should be set to a one
	 * regardless of revision
	 * - There is only single active asynchronous SS EP at the time.
	 * - The active asynchronous EP is a Bulk IN EP.
	 * - The burst with the correctly Retry=1 ACK TP and
	 *   the next burst belong to the same transfer.
	 */
	reg = dwc3_exynos_readl(dwc->regs, DWC3_GUCTL);
	reg |= (DWC3_GUCTL_USBHSTINAUTORETRYEN);

	/* fix ITP interval time to 125us */
	if (DWC3_VER_IS(DWC31, 170A)) {
		reg &= ~DWC3_GUCTL_REFCLKPER_MASK;
		reg |= DWC3_GUCTL_REFCLKPER(0xF);
	} else if (DWC3_VER_IS_WITHIN(DWC31, 180A, 190A)) {
		reg &= ~DWC3_GUCTL_REFCLKPER_MASK;
		reg |= DWC3_GUCTL_REFCLKPER(0x34);
	}

	if (exynos->config.sparse_transfer_control)
		reg |= DWC3_GUCTL_SPRSCTRLTRANSEN;

	if (exynos->config.no_extra_delay)
		reg |= DWC3_GUCTL_NOEXTRDL;

	if (exynos->config.usb_host_device_timeout) {
		reg &= ~DWC3_GUCTL_DTOUT_MASK;
		reg |= DWC3_GUCTL_DTOUT(exynos->config.usb_host_device_timeout);
	}

	dwc3_exynos_writel(dwc->regs, DWC3_GUCTL, reg);
	if (DWC3_VER_IS_WITHIN(DWC3, 190A, 210A)) {
		reg = dwc3_exynos_readl(dwc->regs, DWC3_GRXTHRCFG);
		reg &= ~(DWC3_GRXTHRCFG_USBRXPKTCNT_MASK |
			DWC3_GRXTHRCFG_USBMAXRXBURSTSIZE_MASK);
		reg |= (DWC3_GRXTHRCFG_USBRXPKTCNTSEL |
			DWC3_GRXTHRCFG_USBRXPKTCNT(3) |
			DWC3_GRXTHRCFG_USBMAXRXBURSTSIZE(3));
		dwc3_exynos_writel(dwc->regs, DWC3_GRXTHRCFG, reg);
	}

	/*
	 * WORKAROUND: DWC3 revisions 2.10a and earlier have a bug
	 * The delay of the entry to a low power state such that
	 * for applications where the link stays in a non-U0 state
	 * for a short duration(< 1 microsecond),
	 * the local PHY does not enter the low power state prior
	 * to receiving a potential LFPS wakeup.
	 * This causes the PHY CDR (Clock and Data Recovery) operation
	 * to be unstable for some Synopsys PHYs.
	 * The proposal now is to change the default and the recommended value
	 * for GUSB3PIPECTL[21:19] in the RTL from 3'b100 to a minimum of 3'b001
	 */
	if (DWC3_VER_IS_PRIOR(DWC3, 220A)) {
		reg = dwc3_exynos_readl(dwc->regs, DWC3_GUSB3PIPECTL(0));
		reg &= ~(DWC3_GUSB3PIPECTL_DEP1P2P3_MASK);
		reg |= (DWC3_GUSB3PIPECTL_DEP1P2P3_EN);
		dwc3_exynos_writel(dwc->regs, DWC3_GUSB3PIPECTL(0), reg);
	}

	if (!DWC3_VER_IS_PRIOR(DWC3, 250A)) {
		reg = dwc3_exynos_readl(dwc->regs, DWC3_GUSB3PIPECTL(0));
		reg |= DWC3_GUSB3PIPECTL_DISRXDETINP3;
		dwc3_exynos_writel(dwc->regs, DWC3_GUSB3PIPECTL(0), reg);
	}

	reg = dwc3_exynos_readl(dwc->regs, DWC3_GUSB3PIPECTL(0));
	if (exynos->config.ux_exit_in_px_quirk)
		reg |= DWC3_GUSB3PIPECTL_UX_EXIT_PX;
	if (exynos->config.elastic_buf_mode_quirk)
		reg |= DWC3_ELASTIC_BUFFER_MODE;
	dwc3_exynos_writel(dwc->regs, DWC3_GUSB3PIPECTL(0), reg);

	if (DWC3_VER_IS(DWC31, 170A)) {
		reg = dwc3_exynos_readl(dwc->regs, DWC3_LLUCTL);
		reg |= (DWC3_PENDING_HP_TIMER_US(0xb) | DWC3_EN_US_HP_TIMER);
		reg |= DWC3_FORCE_GEN1;
		dwc3_exynos_writel(dwc->regs, DWC3_LLUCTL, reg);

		reg = dwc3_exynos_readl(dwc->regs, DWC3_LSKIPFREQ);
		reg |= (DWC3_PM_ENTRY_TIMER_US(0x9) |
			DWC3_PM_LC_TIMER_US(0x5) | DWC3_EN_PM_TIMER_US);
		dwc3_exynos_writel(dwc->regs, DWC3_LSKIPFREQ, reg);
	} else if (DWC3_VER_IS_WITHIN(DWC31, 180A, 190A)) {
		reg = dwc3_exynos_readl(dwc->regs, DWC3_LLUCTL);
		reg &= ~(DWC3_LLUCTL_TX_TS1_CNT_MASK);
		reg |= (DWC3_PENDING_HP_TIMER_US(0xb) | DWC3_EN_US_HP_TIMER) |
		    (DWC3_LLUCTL_PIPE_RESET) | (DWC3_LLUCTL_LTSSM_TIMER_OVRRD) |
		    (DWC3_LLUCTL_TX_TS1_CNT(0x0));

		if (exynos->config.force_gen1 && exynos->force_speed != USB_SPEED_SUPER_PLUS)
			reg |= DWC3_FORCE_GEN1;

		dwc3_exynos_writel(dwc->regs, DWC3_LLUCTL, reg);

		reg = dwc3_exynos_readl(dwc->regs, DWC3_LSKIPFREQ);
		reg &= ~(DWC3_PM_LC_TIMER_US_MASK |
				DWC3_PM_ENTRY_TIMER_US_MASK);
		reg |= (DWC3_PM_ENTRY_TIMER_US(0x9) |
			DWC3_PM_LC_TIMER_US(0x5) | DWC3_EN_PM_TIMER_US);
		dwc3_exynos_writel(dwc->regs, DWC3_LSKIPFREQ, reg);

		reg = dwc3_exynos_readl(dwc->regs, DWC3_GUSB3PIPECTL(0));
		reg &= ~DWC3_GUSB3PIPECTL_DISRXDETINP3;
		reg &= ~DWC3_GUSB3PIPECTL_RX_DETOPOLL;
		dwc3_exynos_writel(dwc->regs, DWC3_GUSB3PIPECTL(0), reg);

		/*
		 * Use Default Value
		 * reg = dwc3_exynos_readl(dwc->regs, DWC3_LU3LFPSRXTIM);
		 * dwc3_exynos_writel(dwc->regs, DWC3_LU3LFPSRXTIM, reg);
		 */
		reg = dwc3_exynos_readl(dwc->regs, DWC3_GSBUSCFG0);
		reg |= (DWC3_GSBUSCFG0_INCR8BRSTEN |
				DWC3_GSBUSCFG0_INCR4BRSTEN);
		dwc3_exynos_writel(dwc->regs, DWC3_GSBUSCFG0, reg);

		reg = dwc3_exynos_readl(dwc->regs, DWC3_BU31RHBDBG);
		reg |= DWC3_BU31RHBDBG_TOUTCTL;
		dwc3_exynos_writel(dwc->regs, DWC3_BU31RHBDBG, reg);

		reg = dwc3_exynos_readl(dwc->regs, DWC3_GUCTL1);
		reg &= ~DWC3_GUCTL1_IP_GAP_ADD_ON_MASK;
		reg |= DWC3_GUCTL1_IP_GAP_ADD_ON(0x1);
		dwc3_exynos_writel(dwc->regs, DWC3_GUCTL1, reg);
	}

	/*
	 * WORKAROUND: DWC3 revisions 2.10a and earlier have a bug
	 * Race Condition in PORTSC Write Followed by Read
	 * If the software quickly does a read to the PORTSC,
	 * some fields (port status change related fields
	 * like OCC, etc.) may not have correct value
	 * due to the current way of handling these bits.
	 * After clearing the status register (for example, OCC) bit
	 * by writing PORTSC tregister, software can insert some delay
	 * (for example, 5 mac2_clk -> UTMI clock = 60 MHz ->
	 * (16.66 ns x 5 = 84ns)) before reading the PORTSC to check status.
	 */
	if (DWC3_VER_IS_PRIOR(DWC3, 220A)) {
		reg = dwc3_exynos_readl(dwc->regs, DWC3_GUSB2PHYCFG(0));
		reg |= (DWC3_GUSB2PHYCFG_PHYIF(UTMI_PHYIF_16_BIT));
		dwc3_exynos_writel(dwc->regs, DWC3_GUSB2PHYCFG(0), reg);
	}
}

static void dwc3_exynos_phy_setup(struct dwc3 *dwc, struct dwc3_exynos *exynos)
{
	u32 reg;

	reg = dwc3_exynos_readl(dwc->regs, DWC3_GUSB3PIPECTL(0));

	if (exynos->config.ux_exit_in_px_quirk)
		reg |= DWC3_GUSB3PIPECTL_UX_EXIT_PX;

	if (exynos->config.u1u2_exitfail_to_recov_quirk)
		reg |= DWC3_GUSB3PIPECTL_U1U2EXITFAIL_TO_RECOV;

	dwc3_exynos_writel(dwc->regs, DWC3_GUSB3PIPECTL(0), reg);

	reg = dwc3_exynos_readl(dwc->regs, DWC3_GUSB2PHYCFG(0));

	if (!dwc->dis_enblslpm_quirk)
		reg |= DWC3_GUSB2PHYCFG_ENBLSLPM;

	if (exynos->config.adj_sof_accuracy)
		reg &= ~DWC3_GUSB2PHYCFG_U2_FREECLK_EXISTS;

	dwc3_exynos_writel(dwc->regs, DWC3_GUSB2PHYCFG(0), reg);
}

/* Exynos Specific Configurations */
int dwc3_exynos_core_init(struct dwc3 *dwc, struct dwc3_exynos *exynos)
{
	u32 reg;
	u32 dft;

	dwc3_exynos_phy_setup(dwc, exynos);

	dwc3_core_config(dwc, exynos);

	if (DWC3_VER_IS_WITHIN(DWC31, 180A, 190A)) {
		/* FOR ref_clk 19.2MHz */
		reg = dwc3_exynos_readl(dwc->regs, DWC3_GFLADJ);
		dft = reg & DWC3_GFLADJ_30MHZ_MASK;
		if (dft != dwc->fladj) {
			reg &= ~DWC3_GFLADJ_30MHZ_MASK;
			reg |= dwc->fladj;
		}

		reg &= ~DWC3_GFLADJ_REFCLK_240MHZ_DECR_MASK;
		reg |= DWC3_GFLADJ_REFCLK_240MHZ_DECR(0xc);
		reg |= DWC3_GFLADJ_REFCLK_240MHZDECR_PLS1;

		reg |= DWC3_GFLADJ_REFCLK_LPM_SEL;
		reg &= ~DWC3_GFLADJ_REFCLK_FLADJ_MASK;
		reg |= DWC3_GFLADJ_30MHZ_SDBND_SEL;

		dwc3_exynos_writel(dwc->regs, DWC3_GFLADJ, reg);
	}

	reg = dwc3_exynos_readl(dwc->regs, DWC3_GCTL);

	if (dwc->dis_u2_freeclk_exists_quirk)
		reg |= DWC3_GCTL_SOFITPSYNC;
	else
		reg &= ~DWC3_GCTL_SOFITPSYNC;

	if (exynos->config.adj_sof_accuracy)
		reg &= ~DWC3_GCTL_SOFITPSYNC;

	dwc3_exynos_writel(dwc->regs, DWC3_GCTL, reg);

	return 0;
}

void dwc3_exynos_gadget_disconnect_proc(struct dwc3 *dwc)
{
	int			reg;

	reg = dwc3_exynos_readl(dwc->regs, DWC3_DCTL);
	reg &= ~DWC3_DCTL_INITU1ENA;
	dwc3_exynos_writel(dwc->regs, DWC3_DCTL, reg);

	reg &= ~DWC3_DCTL_INITU2ENA;
	dwc3_exynos_writel(dwc->regs, DWC3_DCTL, reg);

	dwc->gadget->speed = USB_SPEED_UNKNOWN;
	dwc->setup_packet_pending = false;
	usb_gadget_set_state(dwc->gadget, USB_STATE_NOTATTACHED);

	dwc->connected = false;
}

int dwc3_core_susphy_set(struct dwc3 *dwc, int on)
{
	u32		reg;

	reg = dwc3_exynos_readl(dwc->regs, DWC3_GUSB3PIPECTL(0));
	if (on)
		reg |= DWC3_GUSB3PIPECTL_SUSPHY;
	else
		reg &= ~DWC3_GUSB3PIPECTL_SUSPHY;
	dwc3_exynos_writel(dwc->regs, DWC3_GUSB3PIPECTL(0), reg);

	reg = dwc3_exynos_readl(dwc->regs, DWC3_GUSB2PHYCFG(0));
	if (on)
		reg |= DWC3_GUSB2PHYCFG_SUSPHY;
	else
		reg &= ~DWC3_GUSB2PHYCFG_SUSPHY;
	dwc3_exynos_writel(dwc->regs, DWC3_GUSB2PHYCFG(0), reg);

	return 0;
}


/* -------------------------------------------------------------------------- */

int dwc3_exynos_set_bus_clock(struct device *dev, int clk_level)
{
	struct dwc3_exynos *exynos = dev_get_drvdata(dev);

	if (!IS_ERR_OR_NULL(exynos->bus_clock)) {
		if (clk_level < 0) {
			dev_dbg(dev, "Set USB Bus clock to 66Mhz\n");
			clk_set_rate(exynos->bus_clock, 66666666);
		} else if (clk_level == 1) {
			dev_dbg(dev, "Set USB Bus clock to 177Mhz\n");
			clk_set_rate(exynos->bus_clock, 177750000);
		} else if (clk_level == 0) {
			dev_dbg(dev, "Set USB Bus clock to 266Mhz\n");
			clk_set_rate(exynos->bus_clock, 266625000);
		} else {
			dev_dbg(dev, "Unsupported clock level");
		}

		dev_dbg(dev, "Changed USB Bus clock %lu\n",
			 clk_get_rate(exynos->bus_clock));
	}

	return 0;
}

int dwc3_exynos_host_event(struct device *dev, int action)
{
	struct dwc3_exynos	*exynos;
	struct dwc3_otg *dotg;

	exynos = dev_get_drvdata(dev);
	if (!exynos)
		return -ENOENT;

	dotg = exynos->dotg;
	if (!dotg)
		return -ENOENT;

	if (dotg->host_on != action) {
		dotg->host_on = action;
		dwc3_exynos_set_role(dotg);
	}

	return 0;
}

int dwc3_exynos_device_event(struct device *dev, bool action)
{
	struct dwc3_exynos	*exynos;
	struct dwc3_otg *dotg;

	exynos = dev_get_drvdata(dev);
	if (!exynos)
		return -ENOENT;

	dotg = exynos->dotg;
	if (!dotg)
		return -ENOENT;

	if (dotg->device_on != action) {
		dotg->device_on = action;
		dwc3_exynos_set_role(dotg);
	}

	return 0;
}

bool dwc3_exynos_check_usb_suspend(struct dwc3_otg *dotg)
{
	int wait_counter = 0;

	do {
		if (!dotg->dwc3_suspended)
			break;

		wait_counter++;
		msleep(20);
	} while (wait_counter < DWC3_EXYNOS_MAX_WAIT_COUNT);

	return wait_counter < DWC3_EXYNOS_MAX_WAIT_COUNT;
}

/*
 * dwc3_exynos_phy_enable - received combo phy control.
 */
int dwc3_exynos_phy_enable(int owner, bool on)
{
	struct dwc3_exynos	*exynos;
	struct device_node *np = NULL;
	struct platform_device *pdev = NULL;
	struct dwc3		*dwc;
	struct device		*dev;
	struct dwc3_otg		*dotg;
	int ret = 0;

	pr_info("%s owner=%d on=%d +\n", __func__, owner, on);

	np = of_find_compatible_node(NULL, NULL, "samsung,exynos9-dwusb");
	if (np) {
		pdev = of_find_device_by_node(np);
		if (!pdev) {
			pr_err("%s we can't get platform device\n", __func__);
			ret = -ENODEV;
			goto err;
		}
		of_node_put(np);
	} else {
		pr_err("%s we can't get np\n", __func__);
		ret = -ENODEV;
		goto err;
	}

	exynos = platform_get_drvdata(pdev);
	if (!exynos) {
		pr_err("%s we can't get drvdata\n", __func__);
		ret = -ENOENT;
		goto err;
	}

	dwc = exynos->dwc;
	dev = dwc->dev;
	dotg = exynos->dotg;
	if (on) {
		if (!exynos->phy_owner_bits && !dwc3_exynos_check_usb_suspend(dotg))
			dev_err(dev, "too long to wait for dwc3 suspended\n");

		mutex_lock(&dotg->lock);
		exynos->need_dr_role = 1;
		ret = pm_runtime_resume_and_get(dev);
		if (ret < 0) {
			dev_err(dwc->dev, "%s: failed to initialize core: %d\n",
					__func__, ret);
		}
		exynos->need_dr_role = 0;
		exynos->phy_owner_bits |= BIT(owner);
		mutex_unlock(&dotg->lock);
	} else {
		mutex_lock(&dotg->lock);
		if (!dotg->otg_connection)
			exynos->dwc->current_dr_role = DWC3_GCTL_PRTCAP_DEVICE;
		pm_runtime_put_sync_suspend(dev);
		exynos->phy_owner_bits &= ~BIT(owner);
		mutex_unlock(&dotg->lock);
	}

err:
	return ret;
}
EXPORT_SYMBOL_GPL(dwc3_exynos_phy_enable);

static int dwc3_exynos_register_phys(struct dwc3_exynos *exynos)
{
	struct platform_device	*pdev;
	int			ret;

	pdev = platform_device_alloc("usb_phy_generic", PLATFORM_DEVID_AUTO);
	if (!pdev)
		return -ENOMEM;

	exynos->usb2_phy = pdev;

	pdev = platform_device_alloc("usb_phy_generic", PLATFORM_DEVID_AUTO);
	if (!pdev) {
		ret = -ENOMEM;
		goto err1;
	}

	exynos->usb3_phy = pdev;

	ret = platform_device_add(exynos->usb2_phy);
	if (ret)
		goto err2;

	ret = platform_device_add(exynos->usb3_phy);
	if (ret)
		goto err3;

	return 0;

err3:
	platform_device_del(exynos->usb2_phy);

err2:
	platform_device_put(exynos->usb3_phy);

err1:
	platform_device_put(exynos->usb2_phy);

	return ret;
}

static int dwc3_exynos_remove_child(struct device *dev, void *unused)
{
	struct platform_device *pdev = to_platform_device(dev);

	platform_device_unregister(pdev);

	return 0;
}

#if IS_ENABLED(CONFIG_USB_XHCI_GOOG_DMA)
static struct xhci_goog_dma_coherent_mem **dwc3_exynos_get_dma_coherent_mem(struct device *dev)
{
	struct xhci_goog_dma_coherent_mem **dma_mem = NULL;
	struct device *dwc3_exynos = dev->parent;
	struct dwc3_exynos *exynos = dev_get_drvdata(dwc3_exynos);

	if (exynos) {
		if (!exynos->mem) {
			exynos->mem = devm_kzalloc(dev,
				XHCI_GOOG_DMA_RMEM_MAX*sizeof(struct xhci_goog_dma_coherent_mem *),
				GFP_KERNEL);
		}

		dma_mem = exynos->mem;
	}

	return dma_mem;
}

static void dwc3_exynos_put_dma_coherent_mem(struct device *dev)
{
	struct device *dwc3_exynos = dev->parent;
	struct dwc3_exynos *exynos = dev_get_drvdata(dwc3_exynos);

	if (exynos) {
		if (exynos->mem) {
			dev_dbg(dev, "Free the DMA memory.\n");
			exynos->mem[XHCI_GOOG_DMA_RMEM_SRAM] = NULL;
			exynos->mem[XHCI_GOOG_DMA_RMEM_DRAM] = NULL;
			devm_kfree(dev, exynos->mem);
			exynos->mem = NULL;
		}
	}
}
#endif

int dwc3_exynos_host_init(struct dwc3_exynos *exynos)
{
	struct dwc3		*dwc = exynos->dwc;
	struct device		*dev = exynos->dev;
	struct property_entry	props[4];
	struct platform_device	*xhci;
	struct resource		*res;
	struct platform_device	*dwc3_pdev = to_platform_device(dwc->dev);
	int			prop_idx = 0;
	int			ret = 0;
	int			irq;

	/* Configuration xhci resources */
	res = platform_get_resource(dwc3_pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "missing memory resource\n");
		return -ENODEV;
	}
	dwc->xhci_resources[0].start = res->start;
	dwc->xhci_resources[0].end = dwc->xhci_resources[0].start +
					DWC3_XHCI_REGS_END;
	dwc->xhci_resources[0].flags = res->flags;
	dwc->xhci_resources[0].name = res->name;

	irq = platform_get_irq(dwc3_pdev, 0);
	if (irq < 0) {
		dev_err(dev, "missing irq resource\n");
		return irq;
	}

	dwc->xhci_resources[1].start = irq;
	dwc->xhci_resources[1].end = irq;
	dwc->xhci_resources[1].flags = IORESOURCE_IRQ | irq_get_trigger_type(irq);
	dwc->xhci_resources[1].name = of_node_full_name(dwc3_pdev->dev.of_node);

#if IS_ENABLED(CONFIG_USB_XHCI_GOOG_DMA)
	xhci_goog_register_get_cb(dwc3_exynos_get_dma_coherent_mem);
	xhci_goog_register_put_cb(dwc3_exynos_put_dma_coherent_mem);
#endif

	xhci = platform_device_alloc("xhci-hcd-exynos", PLATFORM_DEVID_AUTO);
	if (!xhci) {
		dev_err(dwc->dev, "couldn't allocate xHCI device\n");
		return -ENOMEM;
	}

	xhci->dev.parent	= dwc->dev;
	ret = dma_set_mask_and_coherent(&xhci->dev, DMA_BIT_MASK(36));
	if (ret) {
		pr_err("xhci dma set mask ret = %d\n", ret);
		goto err;
	}

	ret = platform_device_add_resources(xhci, dwc->xhci_resources,
						DWC3_XHCI_RESOURCES_NUM);
	if (ret) {
		dev_err(dwc->dev, "couldn't add resources to xHCI device\n");
		goto err;
	}

	memset(props, 0, sizeof(struct property_entry) * ARRAY_SIZE(props));

	if (dwc->usb3_lpm_capable)
		props[prop_idx++] = PROPERTY_ENTRY_BOOL("usb3-lpm-capable");

	if (dwc->usb2_lpm_disable)
		props[prop_idx++] = PROPERTY_ENTRY_BOOL("usb2-lpm-disable");

	/**
	 * WORKAROUND: dwc3 revisions <=3.00a have a limitation
	 * where Port Disable command doesn't work.
	 *
	 * The suggested workaround is that we avoid Port Disable
	 * completely.
	 *
	 * This following flag tells XHCI to do just that.
	 */
	if (dwc->revision <= DWC3_REVISION_300A)
		props[prop_idx++] = PROPERTY_ENTRY_BOOL("quirk-broken-port-ped");

	if (prop_idx) {
		ret = device_create_managed_software_node(&xhci->dev, props, NULL);
		if (ret) {
			dev_err(dwc->dev, "failed to add properties to xHCI\n");
			goto err;
		}
	}

	dwc->xhci = xhci;

	return 0;
err:
	platform_device_put(xhci);
	return ret;
}
EXPORT_SYMBOL_GPL(dwc3_exynos_host_init);

void dwc3_exynos_host_exit(struct dwc3_exynos *exynos)
{
	struct dwc3		*dwc = exynos->dwc;

	platform_device_unregister(dwc->xhci);

#if IS_ENABLED(CONFIG_USB_XHCI_GOOG_DMA)
	xhci_goog_unregister_get_cb();
	xhci_goog_unregister_put_cb();
#endif
}
EXPORT_SYMBOL_GPL(dwc3_exynos_host_exit);

static int dwc3_exynos_device_notifier(struct notifier_block *nb,
				     unsigned long action, void *dev)
{
	struct dwc3_exynos *exynos = container_of(nb, struct dwc3_exynos, device_nb);

	dev_info(exynos->dev, "turn %s USB gadget\n", action ? "on" : "off");

	if (!exynos->usb_data_enabled) {
		dev_info(exynos->dev, "skip the notification due to USB enumeration disabled\n");
		return NOTIFY_OK;
	}

	dwc3_exynos_device_event(exynos->dev, action);

	return NOTIFY_OK;
}

static int dwc3_exynos_host_notifier(struct notifier_block *nb,
				   unsigned long action, void *dev)
{
	struct dwc3_exynos *exynos = container_of(nb, struct dwc3_exynos, host_nb);

	dev_info(exynos->dev, "turn %s USB host\n", action ? "on" : "off");

	if (!exynos->usb_data_enabled) {
		dev_info(exynos->dev, "skip the notification due to USB enumeration disabled\n");
		return NOTIFY_OK;
	}

	dwc3_exynos_host_event(exynos->dev, action);

	return NOTIFY_OK;
}

static int dwc3_exynos_extcon_register(struct dwc3_exynos *exynos)
{
	struct device *dev = exynos->dev;
	int ret = 0;

	if (!of_property_read_bool(dev->of_node, "extcon"))
		return -EINVAL;

	exynos->edev = extcon_get_edev_by_phandle(dev, 0);
	if (IS_ERR_OR_NULL(exynos->edev)) {
		dev_err(exynos->dev, "couldn't get extcon\n");
		return exynos->edev ? PTR_ERR(exynos->edev) : -ENODEV;
	}

	exynos->device_nb.notifier_call = dwc3_exynos_device_notifier;
	ret = extcon_register_notifier(exynos->edev, EXTCON_USB, &exynos->device_nb);

	if (ret < 0) {
		dev_err(exynos->dev, "couldn't register notifier for EXTCON_USB\n");
		return ret;
	}

	exynos->host_nb.notifier_call = dwc3_exynos_host_notifier;
	ret = extcon_register_notifier(exynos->edev, EXTCON_USB_HOST, &exynos->host_nb);

	if (ret < 0)
		dev_err(exynos->dev, "couldn't register notifier for EXTCON_USB_HOST\n");

	return ret;
}

static int dwc3_exynos_get_properties(struct dwc3_exynos *exynos)
{
	struct device *dev = exynos->dev;
	struct device_node *node = dev->of_node;
	u32 value;
	int ret = 0;

	if (!of_property_read_u32(node, "adj-sof-accuracy", &value)) {
		exynos->config.adj_sof_accuracy = value ? true : false;
		dev_info(dev, "adj-sof-accuracy set from %s node", node->name);
	} else {
		dev_err(dev, "can't get adj-sof-accuracy from %s node", node->name);
		return -EINVAL;
	}
	if (!of_property_read_u32(node, "is_not_vbus_pad", &value)) {
		exynos->config.is_not_vbus_pad = value ? true : false;
		dev_info(dev, "is_not_vbus_pad set from %s node", node->name);
	} else {
		dev_err(dev, "can't get adj-sof-accuracy from %s node", node->name);
		return -EINVAL;
	}
	if (!of_property_read_u32(node, "enable_sprs_transfer", &value)) {
		exynos->config.sparse_transfer_control = value ? true : false;
	} else {
		dev_err(dev, "can't get sprs-xfer-ctrl from %s node", node->name);
		return -EINVAL;
	}
	if (!of_property_read_u32(node, "usb_host_device_timeout", &value)) {
		exynos->config.usb_host_device_timeout = value;
	} else {
		dev_info(dev, "usb_host_device_timeout is not defined...\n");
		exynos->config.usb_host_device_timeout = 0x0;
	}
	if (!of_property_read_u32(node, "suspend_clk_freq", &value)) {
		exynos->config.suspend_clk_freq = value;
	} else {
		dev_info(dev, "Set suspend clock freq to 26Mhz(Default)\n");
		exynos->config.suspend_clk_freq = 26000000;
	}

	exynos->config.no_extra_delay = device_property_read_bool(dev,
					"no_extra_delay");
	exynos->config.ux_exit_in_px_quirk = device_property_read_bool(dev,
				"ux_exit_in_px_quirk");
	exynos->config.elastic_buf_mode_quirk = device_property_read_bool(dev,
				"elastic_buf_mode_quirk");
	exynos->config.force_gen1 = device_property_read_bool(dev,
				"force_gen1");
	exynos->config.u1u2_exitfail_to_recov_quirk = device_property_read_bool(
				dev, "u1u2_exitfail_quirk");

	return ret;
}

/* -------------------------------------------------------------------------- */

/* Current data role is updated after the data role change has been completed */
static ssize_t
dwc3_exynos_otg_state_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct dwc3_exynos	*exynos = dev_get_drvdata(dev);
	struct dwc3_otg		*dotg;
	int			ret = 0;

	mutex_lock(&exynos->dotg_lock);

	dotg = exynos->dotg;
	if (!dotg) {
		mutex_unlock(&exynos->dotg_lock);
		return -ENOENT;
	}

	ret = sysfs_emit(buf, "%s\n", usb_role_string(dotg->current_role));

	mutex_unlock(&exynos->dotg_lock);

	return ret;
}

static DEVICE_ATTR_RO(dwc3_exynos_otg_state);

static ssize_t
dwc3_exynos_otg_b_sess_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct dwc3_exynos	*exynos = dev_get_drvdata(dev);
	struct dwc3_otg		*dotg;
	int			ret = 0;

	mutex_lock(&exynos->dotg_lock);

	dotg = exynos->dotg;
	if (!dotg) {
		mutex_unlock(&exynos->dotg_lock);
		return -ENOENT;
	}

	ret = sysfs_emit(buf, "%d\n", dotg->device_on);

	mutex_unlock(&exynos->dotg_lock);
	return ret;
}

static ssize_t
dwc3_exynos_otg_b_sess_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	struct dwc3_exynos	*exynos = dev_get_drvdata(dev);
	struct dwc3_otg		*dotg;
	int		b_sess_vld;

	if (kstrtoint(buf, 10, &b_sess_vld) != 0)
		return -EINVAL;

	mutex_lock(&exynos->dotg_lock);

	dotg = exynos->dotg;
	if (!dotg) {
		mutex_unlock(&exynos->dotg_lock);
		return -ENOENT;
	}

	dwc3_exynos_device_event(exynos->dev, !!b_sess_vld);
	dwc3_exynos_wait_role(dotg);

	mutex_unlock(&exynos->dotg_lock);
	return n;
}

static DEVICE_ATTR_RW(dwc3_exynos_otg_b_sess);

static ssize_t
dwc3_exynos_otg_id_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct dwc3_exynos	*exynos = dev_get_drvdata(dev);
	struct dwc3_otg		*dotg;
	int			ret = 0;

	mutex_lock(&exynos->dotg_lock);

	dotg = exynos->dotg;
	if (!dotg) {
		mutex_unlock(&exynos->dotg_lock);
		return -ENOENT;
	}

	// id state is true when host mode is off, vice versa.
	ret = sysfs_emit(buf, "%d\n", !dotg->host_on);

	mutex_unlock(&exynos->dotg_lock);
	return ret;
}

static ssize_t
dwc3_exynos_otg_id_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t n)
{
	struct dwc3_exynos	*exynos = dev_get_drvdata(dev);
	struct dwc3_otg		*dotg;
	int id;

	if (kstrtoint(buf, 10, &id) != 0)
		return -EINVAL;

	mutex_lock(&exynos->dotg_lock);

	dotg = exynos->dotg;
	if (!dotg) {
		mutex_unlock(&exynos->dotg_lock);
		return -ENOENT;
	}

	dwc3_exynos_host_event(exynos->dev, !id);
	dwc3_exynos_wait_role(dotg);

	mutex_unlock(&exynos->dotg_lock);
	return n;
}

static DEVICE_ATTR_RW(dwc3_exynos_otg_id);

static ssize_t dwc3_exynos_extra_delay_show(struct device *dev,struct device_attribute *attr,
					    char *buf)
{
	struct dwc3_exynos      *exynos = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", exynos->extra_delay);
}

static ssize_t dwc3_exynos_extra_delay_store(struct device *dev, struct device_attribute *attr,
					     const char *buf, size_t n)
{
	struct dwc3_exynos	*exynos = dev_get_drvdata(dev);
	bool			extra_delay;

	if (kstrtobool(buf, &extra_delay))
		return -EINVAL;

	exynos->extra_delay = extra_delay;

	return n;
}
static DEVICE_ATTR_RW(dwc3_exynos_extra_delay);

static ssize_t usb_data_enabled_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dwc3_exynos *exynos = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%s\n", exynos->usb_data_enabled ? "enabled" : "disabled");
}

static ssize_t usb_data_enabled_store(struct device *dev, struct device_attribute *attr,
				      const char *buf, size_t n)
{
	struct dwc3_exynos *exynos = dev_get_drvdata(dev);
	bool enabled = true;

	if (kstrtobool(buf, &enabled))
		return -EINVAL;

	exynos->usb_data_enabled = enabled;

	return n;
}
static DEVICE_ATTR_RW(usb_data_enabled);

static ssize_t force_speed_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dwc3_exynos *exynos = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%s\n", usb_speed_string(exynos->force_speed));
}

static ssize_t force_speed_store(struct device *dev, struct device_attribute *attr, const char *buf,
				 size_t n)
{
	struct dwc3_exynos *exynos = dev_get_drvdata(dev);
	struct dwc3_otg *dotg;
	bool toggle_gadget;

	if (sysfs_streq(buf, "super-speed-plus")) {
		exynos->force_speed = USB_SPEED_SUPER_PLUS;
	} else if (sysfs_streq(buf, "super-speed")) {
		exynos->force_speed = USB_SPEED_SUPER;
	} else if (sysfs_streq(buf, "high-speed")) {
		exynos->force_speed = USB_SPEED_HIGH;
	} else if (sysfs_streq(buf, "full-speed")) {
		exynos->force_speed = USB_SPEED_FULL;
	} else {
		return -EINVAL;
	}

	mutex_lock(&exynos->dotg_lock);

	dotg = exynos->dotg;
	if (!dotg) {
		mutex_unlock(&exynos->dotg_lock);
		return -ENOENT;
	}

	toggle_gadget = dotg->device_on;
	if (toggle_gadget) {
		dwc3_exynos_device_event(exynos->dev, 0);
		dwc3_exynos_wait_role(dotg);
	}

	exynos->dwc->maximum_speed = exynos->force_speed;
	exynos->dwc->gadget->max_speed = exynos->dwc->maximum_speed;

	if (toggle_gadget) {
		dwc3_exynos_device_event(exynos->dev, 1);
		dwc3_exynos_wait_role(dotg);
	}

	mutex_unlock(&exynos->dotg_lock);
	return n;
}
static DEVICE_ATTR_RW(force_speed);

static ssize_t dwc3_exynos_gadget_state_show(struct device *dev, struct device_attribute *attr,
					     char *buf)
{
	struct dwc3_exynos	*exynos = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", exynos->gadget_state);
}
static DEVICE_ATTR_RO(dwc3_exynos_gadget_state);

/* New data role is updated before the data role change is executed */
static ssize_t
new_data_role_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct dwc3_exynos	*exynos = dev_get_drvdata(dev);
	struct dwc3_otg		*dotg;
	int			ret = 0;

	mutex_lock(&exynos->dotg_lock);

	dotg = exynos->dotg;
	if (!dotg) {
		mutex_unlock(&exynos->dotg_lock);
		return -ENOENT;
	}

	ret = sysfs_emit(buf, "%s", usb_role_string(dotg->desired_role));

	mutex_unlock(&exynos->dotg_lock);
	return ret;
}

static DEVICE_ATTR_RO(new_data_role);

static struct attribute *dwc3_exynos_otg_attrs[] = {
	&dev_attr_dwc3_exynos_otg_id.attr,
	&dev_attr_dwc3_exynos_otg_b_sess.attr,
	&dev_attr_dwc3_exynos_otg_state.attr,
	&dev_attr_dwc3_exynos_extra_delay.attr,
	&dev_attr_usb_data_enabled.attr,
	&dev_attr_force_speed.attr,
	&dev_attr_dwc3_exynos_gadget_state.attr,
	&dev_attr_new_data_role.attr,
	NULL
};
ATTRIBUTE_GROUPS(dwc3_exynos_otg);

static int dwc3_exynos_probe(struct platform_device *pdev)
{
	struct dwc3_exynos	*exynos;
	struct device		*dev = &pdev->dev;
	struct platform_device *dwc3_pdev;
	struct device_node	*node = dev->of_node, *dwc3_np;
	int			ret;
	struct phy		*temp_usb_phy;
	struct device_node	*s2mpu_np;
	struct platform_device	*s2mpu_pdev;

	temp_usb_phy = devm_phy_get(dev, "usb2-phy");
	if (IS_ERR(temp_usb_phy)) {
		dev_dbg(dev, "USB phy is not probed - deferred return!\n");
		return  -EPROBE_DEFER;
	}

	s2mpu_np = of_parse_phandle(dev->of_node, "s2mpus", 0);
	if (s2mpu_np) {
		s2mpu_pdev = of_find_device_by_node(s2mpu_np);
		of_node_put(s2mpu_np);
		if (s2mpu_pdev) {
			device_link_add(dev, &s2mpu_pdev->dev,
					DL_FLAG_AUTOREMOVE_CONSUMER | DL_FLAG_PM_RUNTIME);
		}
	}

#if IS_ENABLED(CONFIG_EXYNOS_PD_HSI0)
	if (!exynos_pd_hsi0_get_ldo_status()) {
		dev_err(dev, "pd-hsi0 is not powered, deferred probe!");
		return -EPROBE_DEFER;
	}
#endif

	exynos = devm_kzalloc(dev, sizeof(*exynos), GFP_KERNEL);
	if (!exynos)
		return -ENOMEM;

	ret = dma_set_mask(dev, DMA_BIT_MASK(36));
	if (ret) {
		dev_err(dev, "dma set mask ret = %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, exynos);

	exynos->dev = dev;

	exynos->idle_ip_index = exynos_get_idle_ip_index(dev_name(dev));
	exynos_update_ip_idle_status(exynos->idle_ip_index, 0);

	ret = dwc3_exynos_clk_get(exynos);
	if (ret)
		return ret;

	/*
	 * Directly check for the property as EIVAL and other err values can be returned in extcon
	 * APIs in which case we want to defer probe.
	 */
	if (of_property_read_bool(dev->of_node, "extcon")) {
		ret = dwc3_exynos_extcon_register(exynos);
		if (ret < 0) {
			dev_err(dev, "failed to register extcon (%d)\n", ret);
			ret = -EPROBE_DEFER;
			goto vdd33_err;
		}
	} else {
		dev_warn(dev, "no extcon found\n");
	}

	ret = dwc3_exynos_register_phys(exynos);
	if (ret) {
		dev_err(dev, "couldn't register PHYs\n");
		goto extcon_unregister;
	}

	ret = dwc3_exynos_get_properties(exynos);
	if (ret) {
		dev_err(dev, "couldn't get properties.\n");
		goto extcon_unregister;
	}

	pm_runtime_enable(dev);
	ret = pm_runtime_get_sync(dev);
	if (ret < 0)
		goto extcon_unregister;

	pm_runtime_forbid(dev);

	dwc3_np = of_get_child_by_name(node, "dwc3");
	if (!dwc3_np) {
		dev_err(dev, "failed to find dwc3 core child!\n");
		ret = -EEXIST;
		goto extcon_unregister;
	}

	exynos_usbdrd_s2mpu_manual_control(1);

	if (node) {
		ret = of_platform_populate(node, NULL, NULL, dev);
		if (ret) {
			dev_err(dev, "failed to add dwc3 core\n");
			goto populate_err;
		}
	} else {
		dev_err(dev, "no device node, failed to add dwc3 core\n");
		ret = -ENODEV;
		goto populate_err;
	}

	dwc3_pdev = of_find_device_by_node(dwc3_np);
	exynos->dwc = platform_get_drvdata(dwc3_pdev);
	if (exynos->dwc == NULL)
		goto populate_err;

	/* dwc3 core configurations */
	pm_runtime_allow(exynos->dwc->dev);
	ret = dma_set_mask_and_coherent(exynos->dwc->dev, DMA_BIT_MASK(36));
	if (ret) {
		dev_err(dev, "dwc3 core dma_set_mask returned FAIL!(%d)\n", ret);
		goto populate_err;
	}
	exynos->dwc->gadget->sg_supported = false;
	exynos->dwc->imod_interval = 100;
	pm_runtime_dont_use_autosuspend(exynos->dwc->dev);

	/* set the initial value */
	mutex_init(&exynos->dotg_lock);
	exynos->usb_data_enabled = true;

	exynos_usbdrd_phy_tune(exynos->dwc->usb2_generic_phy, 0);
	exynos_usbdrd_phy_tune(exynos->dwc->usb3_generic_phy, 0);

	ret = pm_runtime_put(dev);
	pm_runtime_allow(dev);

	ret = dwc3_exynos_otg_init(exynos->dwc, exynos);
	if (ret < 0) {
		dev_err(dev, "failed to initialize dwc3_exynos_otg\n");
		goto populate_err;
	}

	/* disconnect gadget in probe */
	usb_udc_vbus_handler(exynos->dwc->gadget, false);

	if (of_property_read_bool(dev->of_node, "extcon")) {
		/*
		 * To avoid missing notification in kernel booting check extcon state to run state
		 * machine.
		 */
		if (extcon_get_state(exynos->edev, EXTCON_USB) > 0)
			dwc3_exynos_device_event(exynos->dev, 1);
		else if (extcon_get_state(exynos->edev, EXTCON_USB_HOST) > 0)
			dwc3_exynos_host_event(exynos->dev, 1);
	} else {
		dev_warn(exynos->dev, "Couldn't find extcon. Enable vbus event forcibly.");
		dwc3_exynos_device_event(exynos->dev, 1);
	}

	return 0;

populate_err:
	platform_device_unregister(exynos->usb2_phy);
	platform_device_unregister(exynos->usb3_phy);
extcon_unregister:
	if (exynos->edev) {
		extcon_unregister_notifier(exynos->edev, EXTCON_USB, &exynos->device_nb);
		extcon_unregister_notifier(exynos->edev, EXTCON_USB_HOST, &exynos->host_nb);
	}
vdd33_err:
	dwc3_exynos_clk_disable_unprepare(exynos);
	exynos_update_ip_idle_status(exynos->idle_ip_index, 1);
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	dev_err(dev, "%s err = %d\n", __func__, ret);

	return ret;
}

static int dwc3_exynos_remove(struct platform_device *pdev)
{
	struct dwc3_exynos	*exynos = platform_get_drvdata(pdev);
	struct dwc3	*dwc = exynos->dwc;

	mutex_lock(&exynos->dotg_lock);
	dwc3_exynos_otg_exit(dwc, exynos);
	mutex_unlock(&exynos->dotg_lock);

	pm_runtime_get_sync(&pdev->dev);

	dwc3_ulpi_exit(dwc);

	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_allow(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	device_for_each_child(&pdev->dev, NULL, dwc3_exynos_remove_child);
	platform_device_unregister(exynos->usb2_phy);
	platform_device_unregister(exynos->usb3_phy);

	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev)) {
		dwc3_exynos_clk_disable_unprepare(exynos);
		pm_runtime_set_suspended(&pdev->dev);
	}

	return 0;
}

static void dwc3_exynos_shutdown(struct platform_device *pdev)
{
	struct dwc3_exynos *exynos = platform_get_drvdata(pdev);

	/*
	 * According to extcon state, turn off USB gadget or USB host
	 * during the shutdown process.
	 */
	if (extcon_get_state(exynos->edev, EXTCON_USB) > 0)
		dwc3_exynos_device_event(exynos->dev, 0);
	else if (extcon_get_state(exynos->edev, EXTCON_USB_HOST) > 0)
		dwc3_exynos_host_event(exynos->dev, 0);

	/* unregister the notifiers for USB and USB_HOST*/
	extcon_unregister_notifier(exynos->edev, EXTCON_USB, &exynos->device_nb);
	extcon_unregister_notifier(exynos->edev, EXTCON_USB_HOST, &exynos->host_nb);

	dwc3_exynos_remove(pdev);

	return;
}

#ifdef CONFIG_PM
static int dwc3_exynos_runtime_suspend(struct device *dev)
{
	struct dwc3_exynos *exynos = dev_get_drvdata(dev);
	struct dwc3 *dwc;
	unsigned long flags;

	if (!exynos)
		return 0;

	if (pm_runtime_suspended(dev))
		return 0;

	dwc3_exynos_clk_disable_unprepare(exynos);

	/* inform what USB state is idle to IDLE_IP */
	exynos_update_ip_idle_status(exynos->idle_ip_index, 1);

	dwc = exynos->dwc;
	spin_lock_irqsave(&dwc->lock, flags);
	/* After disconnecting cable, it will ignore core operations like
	 * dwc3_suspend/resume in core.c
	 */
	dwc->current_dr_role = DWC3_EXYNOS_IGNORE_CORE_OPS;
	spin_unlock_irqrestore(&dwc->lock, flags);

	return 0;
}

static int dwc3_exynos_runtime_resume(struct device *dev)
{
	struct dwc3_exynos *exynos = dev_get_drvdata(dev);
	int ret = 0;

	if (!exynos)
		return 0;

	if (exynos->need_dr_role)
		exynos->dwc->current_dr_role = DWC3_GCTL_PRTCAP_DEVICE;

	/* inform what USB state is not idle to IDLE_IP */
	exynos_update_ip_idle_status(exynos->idle_ip_index, 0);

	ret = dwc3_exynos_clk_prepare_enable(exynos);
	if (ret) {
		dev_err(dev, "%s: clk_prepare_enable failed\n", __func__);
		return ret;
	}

	pm_runtime_mark_last_busy(dev);
	return 0;
}
#endif

#ifdef CONFIG_PM_SLEEP
static int dwc3_exynos_suspend(struct device *dev)
{
	struct dwc3_exynos *exynos = dev_get_drvdata(dev);

	if (pm_runtime_suspended(dev))
		return 0;

	dwc3_exynos_clk_disable_unprepare(exynos);

	return 0;
}

static int dwc3_exynos_resume(struct device *dev)
{
	struct dwc3_exynos *exynos = dev_get_drvdata(dev);
	int		ret;

	ret = dwc3_exynos_clk_prepare_enable(exynos);
	if (ret) {
		dev_err(dev, "%s: clk_prepare_enable failed\n", __func__);
		return ret;
	}

	pm_runtime_disable(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	return 0;
}

static const struct dev_pm_ops dwc3_exynos_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(dwc3_exynos_suspend, dwc3_exynos_resume)
	SET_RUNTIME_PM_OPS(dwc3_exynos_runtime_suspend,
			   dwc3_exynos_runtime_resume, NULL)
};

#define DEV_PM_OPS	(&dwc3_exynos_dev_pm_ops)
#else
#define DEV_PM_OPS	NULL
#endif /* CONFIG_PM_SLEEP */

static struct platform_driver dwc3_exynos_driver = {
	.probe		= dwc3_exynos_probe,
	.remove		= dwc3_exynos_remove,
	.shutdown	= dwc3_exynos_shutdown,
	.driver		= {
		.name	= "exynos-dwc3",
		.of_match_table = exynos_dwc3_match,
		.dev_groups = dwc3_exynos_otg_groups,
		.pm	= DEV_PM_OPS,
	},
};

module_platform_driver(dwc3_exynos_driver);

MODULE_SOFTDEP("pre: phy-exynos-usbdrd-super");
MODULE_AUTHOR("Anton Tikhomirov <av.tikhomirov@samsung.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("DesignWare USB3 EXYNOS Glue Layer");
