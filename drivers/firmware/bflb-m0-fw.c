// SPDX-License-Identifier: GPL-2.0-only
/*
 * Helpers for loading firmware/resetting M0 on BL808.
 *
 * Copyright (C) Bouffalo Lab 2016-2023
 */

#include <linux/firmware/bflb-m0-fw.h>
#include <linux/delay.h>
#include <linux/io.h>

#define GLB_BASE 0x20000000
#define PDS_BASE 0x2000e000

#define PDS_CPU_CORE_CFG1_OFFSET	(0x114)
#define PDS_REG_MCU1_CLK_EN_POS		(8U)
#define PDS_CPU_CORE_CFG14_OFFSET	(0x148)

#define GLB_SWRST_CFG2_OFFSET		(0x548)
#define GLB_REG_CTRL_CPU_RESET_POS	(1U)

int bflb_halt_m0(void)
{
	int ret = 0;
	u8 __iomem *glb_base = NULL;
	u8 __iomem *pds_base = NULL;
	u32 tmp;

	if ((glb_base = ioremap(GLB_BASE, 0x1000)) == NULL ||
			(pds_base = ioremap(PDS_BASE, 0x1000)) == NULL) {
		ret = -ENOMEM;
		goto cleanup;
	}

	/* disable M0 clock */
	tmp = readl(pds_base + PDS_CPU_CORE_CFG1_OFFSET);
	tmp &= ~BIT(PDS_REG_MCU1_CLK_EN_POS);
	writel(tmp, pds_base + PDS_CPU_CORE_CFG1_OFFSET);
	udelay(1);
	/* reset CPU */
	tmp = readl(glb_base + GLB_SWRST_CFG2_OFFSET);
	tmp |= BIT(GLB_REG_CTRL_CPU_RESET_POS);
	writel(tmp, glb_base + GLB_SWRST_CFG2_OFFSET);

cleanup:
	if (glb_base)
		iounmap(glb_base);
	if (pds_base)
		iounmap(pds_base);

	return ret;
}
EXPORT_SYMBOL_GPL(bflb_halt_m0);

int bflb_reset_m0(u32 reset_addr)
{
	int ret = 0;
	u8 __iomem *glb_base = NULL;
	u8 __iomem *pds_base = NULL;
	u32 tmp;

	if ((glb_base = ioremap(GLB_BASE, 0x1000)) == NULL ||
			(pds_base = ioremap(PDS_BASE, 0x1000)) == NULL) {
		ret = -ENOMEM;
		goto cleanup;
	}

	/* set reset address */
	writel(reset_addr, pds_base + PDS_CPU_CORE_CFG14_OFFSET);
	/* enable M0 clock */
	tmp = readl(pds_base + PDS_CPU_CORE_CFG1_OFFSET);
	tmp |= BIT(PDS_REG_MCU1_CLK_EN_POS);
	writel(tmp, pds_base + PDS_CPU_CORE_CFG1_OFFSET);
	udelay(1);
	/* reset CPU */
	tmp = readl(glb_base + GLB_SWRST_CFG2_OFFSET);
	tmp |= BIT(GLB_REG_CTRL_CPU_RESET_POS);
	writel(tmp, glb_base + GLB_SWRST_CFG2_OFFSET);

cleanup:
	if (glb_base)
		iounmap(glb_base);
	if (pds_base)
		iounmap(pds_base);

	return ret;
}
EXPORT_SYMBOL_GPL(bflb_reset_m0);

int bflb_load_m0_fw(struct device *dev, const char *fwname, u32 load_address)
{
	const struct firmware *fw;
	int ret = 0;
	u8 __iomem *tgt = NULL;

	ret = request_firmware(&fw, fwname, dev);
	if (ret) {
		dev_err(dev, "Request firmware failed\n");
		return ret;
	}

	if ((tgt = ioremap(load_address, fw->size)) == NULL) {
		ret = -ENOMEM;
		goto cleanup;
	}

	/* copy fw */
	memcpy_toio(tgt, fw->data, fw->size);

cleanup:
	if (tgt)
		iounmap(tgt);
	release_firmware(fw);

	return ret;
}
EXPORT_SYMBOL_GPL(bflb_load_m0_fw);

int bflb_run_m0_fw(struct device *dev, const char *fwname, u32 load_address)
{
	int ret;

	if ((ret = bflb_halt_m0()))
		return ret;
	if ((ret = bflb_load_m0_fw(dev, fwname, load_address)))
		return ret;
	return bflb_reset_m0(load_address);
}
EXPORT_SYMBOL_GPL(bflb_run_m0_fw);
