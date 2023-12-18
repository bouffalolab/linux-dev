// SPDX-License-Identifier: GPL-2.0+
/*
 * Central probing code for the FOTG210 dual role driver
 * We register one driver for the hardware and then we decide
 * whether to proceed with probing the host or the peripheral
 * driver.
 */
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/usb.h>
#include <linux/usb/otg.h>
#include <linux/bitfield.h>

#include "fotg210.h"

/* Role Register 0x80 */
#define FOTG210_RR			0x80
#define FOTG210_RR_ID			BIT(21) /* 1 = B-device, 0 = A-device */
#define FOTG210_RR_CROLE		BIT(20) /* 1 = device, 0 = host */

/*
 * Gemini-specific initialization function, only executed on the
 * Gemini SoC using the global misc control register.
 *
 * The gemini USB blocks are connected to either Mini-A (host mode) or
 * Mini-B (peripheral mode) plugs. There is no role switch support on the
 * Gemini SoC, just either-or.
 */
#define GEMINI_GLOBAL_MISC_CTRL		0x30
#define GEMINI_MISC_USB0_WAKEUP		BIT(14)
#define GEMINI_MISC_USB1_WAKEUP		BIT(15)
#define GEMINI_MISC_USB0_VBUS_ON	BIT(22)
#define GEMINI_MISC_USB1_VBUS_ON	BIT(23)
#define GEMINI_MISC_USB0_MINI_B		BIT(29)
#define GEMINI_MISC_USB1_MINI_B		BIT(30)

static int fotg210_gemini_init(struct fotg210 *fotg, struct resource *res,
			       enum usb_dr_mode mode)
{
	struct device *dev = fotg->dev;
	struct device_node *np = dev->of_node;
	struct regmap *map;
	bool wakeup;
	u32 mask, val;
	int ret;

	map = syscon_regmap_lookup_by_phandle(np, "syscon");
	if (IS_ERR(map))
		return dev_err_probe(dev, PTR_ERR(map), "no syscon\n");
	fotg->map = map;
	wakeup = of_property_read_bool(np, "wakeup-source");

	/*
	 * Figure out if this is USB0 or USB1 by simply checking the
	 * physical base address.
	 */
	mask = 0;
	if (res->start == 0x69000000) {
		fotg->port = GEMINI_PORT_1;
		mask = GEMINI_MISC_USB1_VBUS_ON | GEMINI_MISC_USB1_MINI_B |
			GEMINI_MISC_USB1_WAKEUP;
		if (mode == USB_DR_MODE_HOST)
			val = GEMINI_MISC_USB1_VBUS_ON;
		else
			val = GEMINI_MISC_USB1_MINI_B;
		if (wakeup)
			val |= GEMINI_MISC_USB1_WAKEUP;
	} else {
		fotg->port = GEMINI_PORT_0;
		mask = GEMINI_MISC_USB0_VBUS_ON | GEMINI_MISC_USB0_MINI_B |
			GEMINI_MISC_USB0_WAKEUP;
		if (mode == USB_DR_MODE_HOST)
			val = GEMINI_MISC_USB0_VBUS_ON;
		else
			val = GEMINI_MISC_USB0_MINI_B;
		if (wakeup)
			val |= GEMINI_MISC_USB0_WAKEUP;
	}

	ret = regmap_update_bits(map, GEMINI_GLOBAL_MISC_CTRL, mask, val);
	if (ret) {
		dev_err(dev, "failed to initialize Gemini PHY\n");
		return ret;
	}

	dev_info(dev, "initialized Gemini PHY in %s mode\n",
		 (mode == USB_DR_MODE_HOST) ? "host" : "gadget");
	return 0;
}

#define BFLB_CLKGATE_CFG1_REG		0x584
#define BFLB_AHB_CLKGATE_USB_MASK	BIT(13)

#define BFLB_WIFI_PLL_CFG10_REG		0x838
#define BFLB_WIFI_PLL_MMDIV_MASK	BIT(29)
#define BFLB_WIFI_PLL_RST_MASK		BIT(28)

#define BFLB_PDS_USB_PHY_CTRL_REG	0x504
#define BFLB_PDS_USB_PHY_PONRST_MASK	BIT(0)
#define BFLB_PDS_USB_PHY_XTLSEL_MASK	BIT(2)
#define BFLB_PDS_PU_USB20_PSW_MASK	BIT(6)

#define BFLB_PDS_USB_CTL_REG		0x500
#define BFLB_PDS_USB_SW_RST_MASK	BIT(0)
#define BFLB_PDS_USB_EXT_SUSP_MASK	BIT(1)
#define BFLB_PDS_USB_IDDIG_MASK		BIT(5)

#define BFLB_USB_GLB_INT_REG		0xc4
#define BFLB_USB_GLB_DEV_INT_MASK	BIT(0)
#define BFLB_USB_GLB_OTG_INT_MASK	BIT(1)
#define BFLB_USB_GLB_HOST_INT_MASK	BIT(2)

static int bflb_usb_init(struct fotg210 *fotg)
{
#ifdef CONFIG_BFLB_FOTG210_PATCH
	u32 tmp;
	struct regmap *map;
	struct device *dev = fotg->dev;
	struct device_node *np = dev_of_node(dev);

	map = syscon_regmap_lookup_by_phandle(np, "syscon");
	if (IS_ERR(map))
		return dev_err_probe(dev, PTR_ERR(map), "no syscon\n");

	/* enable AHB clock gate for USB */
	regmap_update_bits(map, BFLB_CLKGATE_CFG1_REG, BFLB_AHB_CLKGATE_USB_MASK,
				FIELD_PREP(BFLB_AHB_CLKGATE_USB_MASK, 1));
	/* set usb clock from wifi pll */
	regmap_update_bits(map, BFLB_WIFI_PLL_CFG10_REG, BFLB_WIFI_PLL_MMDIV_MASK,
				FIELD_PREP(BFLB_WIFI_PLL_MMDIV_MASK, 1));
	regmap_update_bits(map, BFLB_WIFI_PLL_CFG10_REG, BFLB_WIFI_PLL_RST_MASK,
				FIELD_PREP(BFLB_WIFI_PLL_RST_MASK, 1));
	udelay(2);
	regmap_update_bits(map, BFLB_WIFI_PLL_CFG10_REG, BFLB_WIFI_PLL_RST_MASK,
				FIELD_PREP(BFLB_WIFI_PLL_RST_MASK, 0));
	udelay(2);
	regmap_update_bits(map, BFLB_WIFI_PLL_CFG10_REG, BFLB_WIFI_PLL_RST_MASK,
				FIELD_PREP(BFLB_WIFI_PLL_RST_MASK, 1));
	/* enable USB in PDS */
	map = syscon_regmap_lookup_by_phandle(np, "pds");
	if (IS_ERR(map))
		return dev_err_probe(dev, PTR_ERR(map), "no pds\n");

	regmap_update_bits(map, BFLB_PDS_USB_PHY_CTRL_REG, BFLB_PDS_USB_PHY_XTLSEL_MASK,
				FIELD_PREP(BFLB_PDS_USB_PHY_XTLSEL_MASK, 0));
	regmap_update_bits(map, BFLB_PDS_USB_PHY_CTRL_REG, BFLB_PDS_PU_USB20_PSW_MASK,
				FIELD_PREP(BFLB_PDS_PU_USB20_PSW_MASK, 1));
	regmap_update_bits(map, BFLB_PDS_USB_PHY_CTRL_REG, BFLB_PDS_USB_PHY_PONRST_MASK,
				FIELD_PREP(BFLB_PDS_USB_PHY_PONRST_MASK, 1));
	udelay(1);

	regmap_update_bits(map, BFLB_PDS_USB_CTL_REG, BFLB_PDS_USB_SW_RST_MASK,
			FIELD_PREP(BFLB_PDS_USB_SW_RST_MASK, 0));
	udelay(1);
	regmap_update_bits(map, BFLB_PDS_USB_CTL_REG, BFLB_PDS_USB_EXT_SUSP_MASK,
			FIELD_PREP(BFLB_PDS_USB_EXT_SUSP_MASK, 1));
	mdelay(3);
	regmap_update_bits(map, BFLB_PDS_USB_CTL_REG, BFLB_PDS_USB_SW_RST_MASK,
			FIELD_PREP(BFLB_PDS_USB_SW_RST_MASK, 1));
	mdelay(2);

	regmap_update_bits(map, BFLB_PDS_USB_CTL_REG, BFLB_PDS_USB_IDDIG_MASK,
			FIELD_PREP(BFLB_PDS_USB_IDDIG_MASK, 0));

	/* only enable usb host interrupt */
	tmp = readl(fotg->base + BFLB_USB_GLB_INT_REG);
	tmp &= ~BFLB_USB_GLB_HOST_INT_MASK;
	tmp |= BFLB_USB_GLB_DEV_INT_MASK;
	tmp |= BFLB_USB_GLB_OTG_INT_MASK;
	writel(tmp, fotg->base + BFLB_USB_GLB_INT_REG);
/* CONFIG_BFLB_FOTG210_PATCH */
#endif
	return 0;
}

/**
 * fotg210_vbus() - Called by gadget driver to enable/disable VBUS
 * @enable: true to enable VBUS, false to disable VBUS
 */
void fotg210_vbus(struct fotg210 *fotg, bool enable)
{
	u32 mask;
	u32 val;
	int ret;

	switch (fotg->port) {
	case GEMINI_PORT_0:
		mask = GEMINI_MISC_USB0_VBUS_ON;
		val = enable ? GEMINI_MISC_USB0_VBUS_ON : 0;
		break;
	case GEMINI_PORT_1:
		mask = GEMINI_MISC_USB1_VBUS_ON;
		val = enable ? GEMINI_MISC_USB1_VBUS_ON : 0;
		break;
	default:
		return;
	}
	ret = regmap_update_bits(fotg->map, GEMINI_GLOBAL_MISC_CTRL, mask, val);
	if (ret)
		dev_err(fotg->dev, "failed to %s VBUS\n",
			enable ? "enable" : "disable");
	dev_info(fotg->dev, "%s: %s VBUS\n", __func__, enable ? "enable" : "disable");
}

static int fotg210_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	enum usb_dr_mode mode;
	struct fotg210 *fotg;
	u32 val;
	int ret;

	fotg = devm_kzalloc(dev, sizeof(*fotg), GFP_KERNEL);
	if (!fotg)
		return -ENOMEM;
	fotg->dev = dev;

	fotg->base = devm_platform_get_and_ioremap_resource(pdev, 0, &fotg->res);
	if (IS_ERR(fotg->base))
		return PTR_ERR(fotg->base);

	fotg->pclk = devm_clk_get_optional_enabled(dev, "PCLK");
	if (IS_ERR(fotg->pclk))
		return PTR_ERR(fotg->pclk);

	mode = usb_get_dr_mode(dev);

	if (of_device_is_compatible(dev->of_node, "cortina,gemini-usb")) {
		ret = fotg210_gemini_init(fotg, fotg->res, mode);
		if (ret)
			return ret;
	}

	if (of_device_is_compatible(dev->of_node, "bflb,bl808-usb")) {
		ret = bflb_usb_init(fotg);
		if (ret)
			return ret;
	}

	val = readl(fotg->base + FOTG210_RR);
	if (mode == USB_DR_MODE_PERIPHERAL) {
		if (!(val & FOTG210_RR_CROLE))
			dev_err(dev, "block not in device role\n");
		ret = fotg210_udc_probe(pdev, fotg);
	} else {
		if (val & FOTG210_RR_CROLE)
			dev_err(dev, "block not in host role\n");
		ret = fotg210_hcd_probe(pdev, fotg);
	}

	return ret;
}

static void fotg210_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	enum usb_dr_mode mode;

	mode = usb_get_dr_mode(dev);

	if (mode == USB_DR_MODE_PERIPHERAL)
		fotg210_udc_remove(pdev);
	else
		fotg210_hcd_remove(pdev);
}

#ifdef CONFIG_OF
static const struct of_device_id fotg210_of_match[] = {
	{ .compatible = "faraday,fotg200" },
	{ .compatible = "faraday,fotg210" },
	/* TODO: can we also handle FUSB220? */
	{},
};
MODULE_DEVICE_TABLE(of, fotg210_of_match);
#endif

static struct platform_driver fotg210_driver = {
	.driver = {
		.name   = "fotg210",
		.of_match_table = of_match_ptr(fotg210_of_match),
	},
	.probe  = fotg210_probe,
	.remove_new = fotg210_remove,
};

static int __init fotg210_init(void)
{
	if (IS_ENABLED(CONFIG_USB_FOTG210_HCD) && !usb_disabled())
		fotg210_hcd_init();
	return platform_driver_register(&fotg210_driver);
}
module_init(fotg210_init);

static void __exit fotg210_cleanup(void)
{
	platform_driver_unregister(&fotg210_driver);
	if (IS_ENABLED(CONFIG_USB_FOTG210_HCD))
		fotg210_hcd_cleanup();
}
module_exit(fotg210_cleanup);

MODULE_AUTHOR("Yuan-Hsin Chen, Feng-Hsin Chiang");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("FOTG210 Dual Role Controller Driver");
