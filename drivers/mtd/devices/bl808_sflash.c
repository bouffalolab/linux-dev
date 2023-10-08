// SPDX-License-Identifier: GPL-2.0-only
/*
 * bl808_sflash.c - Bouffalo Lab Serial Flash Controller
 *
 * Author: Chien Wong <qwang@bouffalolab.com>
 *
 * Copyright (C) Bouffalo Lab 2016-2023
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/spi-nor.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of.h>

#include "bl808_sflash.h"

struct bl_sflash {
	struct device		*dev;
	struct mtd_info		mtd;
	struct mutex		lock;
	spi_flash_cfg_type	flash_cfg;
};

static uint8_t __iomem *reg_base;
static spi_flash_cfg_type *p_flash_cfg;

#define getreg32 readl
#define putreg32(v, a) writel(v, a)
#define BFLB_SF_CTRL_BUF_BASE (reg_base + 0x600)

static uint8_t bflb_sf_ctrl_get_busy_state(void)
{
	uint32_t regval = 0;

	regval = getreg32(reg_base + SF_CTRL_IF1_SAHB_OFFSET + SF_CTRL_IF_SAHB_0_OFFSET);
	if (regval & SF_CTRL_IF_BUSY)
		return 1;

	return 0;
}

static void bflb_sf_ctrl_set_owner(uint8_t owner)
{
	uint32_t regval = 0;
	uint32_t time_out = 0;

	time_out = SF_CTRL_BUSY_STATE_TIMEOUT;

	while (bflb_sf_ctrl_get_busy_state()) {
		time_out--;

		if (time_out == 0)
			return;
	}

	regval = getreg32(reg_base + SF_CTRL_1_OFFSET);
	/* Set owner */
	if (owner)
		regval |= SF_CTRL_SF_IF_FN_SEL;
	else
		regval &= ~SF_CTRL_SF_IF_FN_SEL;

	/* Set iahb to flash interface */
	if (owner == SF_CTRL_OWNER_IAHB)
		regval |= SF_CTRL_SF_AHB2SIF_EN;
	else
		regval &= ~SF_CTRL_SF_AHB2SIF_EN;
	putreg32(regval, reg_base + SF_CTRL_1_OFFSET);
}

static void bflb_sf_ctrl_select_clock(uint8_t sahb_sram_sel)
{
}

static void bflb_sf_ctrl_sendcmd(struct sf_ctrl_cmd_cfg_type *cfg)
{
	uint32_t regval = 0;
	uint32_t time_out = 0;
	u8 __iomem *cmd_offset = 0;

	time_out = SF_CTRL_BUSY_STATE_TIMEOUT;

	while (bflb_sf_ctrl_get_busy_state()) {
		time_out--;

		if (time_out == 0)
			return;
	}

	regval = getreg32(reg_base + SF_CTRL_1_OFFSET);
	if (regval & SF_CTRL_SF_IF_FN_SEL)
		return;
	cmd_offset = reg_base + SF_CTRL_IF1_SAHB_OFFSET;

	/* Clear trigger */
	regval = getreg32(cmd_offset + SF_CTRL_IF_SAHB_0_OFFSET);
	regval &= ~SF_CTRL_IF_0_TRIG;
	putreg32(regval, cmd_offset + SF_CTRL_IF_SAHB_0_OFFSET);

	/* Copy command buffer */
	putreg32(cfg->cmd_buf[0], cmd_offset + SF_CTRL_IF_SAHB_1_OFFSET);
	putreg32(cfg->cmd_buf[1], cmd_offset + SF_CTRL_IF_SAHB_2_OFFSET);

	regval = getreg32(cmd_offset + SF_CTRL_IF_SAHB_0_OFFSET);
	/* Configure SPI and IO mode*/
	if (cfg->cmd_mode == SF_CTRL_CMD_1_LINE)
		regval &= ~SF_CTRL_IF_0_QPI_MODE_EN;
	else
		regval |= SF_CTRL_IF_0_QPI_MODE_EN;

	regval &= ~SF_CTRL_IF_0_SPI_MODE_MASK;
	if (cfg->addr_mode == SF_CTRL_ADDR_1_LINE) {
		if (cfg->data_mode == SF_CTRL_DATA_1_LINE)
			regval |= (SF_CTRL_NIO_MODE << SF_CTRL_IF_0_SPI_MODE_SHIFT);
		else if (cfg->data_mode == SF_CTRL_DATA_2_LINES)
			regval |= (SF_CTRL_DO_MODE << SF_CTRL_IF_0_SPI_MODE_SHIFT);
		else if (cfg->data_mode == SF_CTRL_DATA_4_LINES)
			regval |= (SF_CTRL_QO_MODE << SF_CTRL_IF_0_SPI_MODE_SHIFT);
	} else if (cfg->addr_mode == SF_CTRL_ADDR_2_LINES) {
		regval |= (SF_CTRL_DIO_MODE << SF_CTRL_IF_0_SPI_MODE_SHIFT);
	} else if (cfg->addr_mode == SF_CTRL_ADDR_4_LINES) {
		regval |= (SF_CTRL_QIO_MODE << SF_CTRL_IF_0_SPI_MODE_SHIFT);
	}

	/* Configure cmd */
	regval |= SF_CTRL_IF_0_CMD_EN;
	regval &= ~SF_CTRL_IF_0_CMD_BYTE_MASK;

	/* Configure address */
	regval &= ~SF_CTRL_IF_0_ADR_BYTE_MASK;
	if (cfg->addr_size != 0) {
		regval |= SF_CTRL_IF_0_ADR_EN;
		regval |= ((cfg->addr_size - 1) << SF_CTRL_IF_0_ADR_BYTE_SHIFT);
	} else {
		regval &= ~SF_CTRL_IF_0_ADR_EN;
	}

	/* Configure dummy */
	regval &= ~SF_CTRL_IF_0_DMY_BYTE_MASK;
	if (cfg->dummy_clks != 0) {
		regval |= SF_CTRL_IF_0_DMY_EN;
		regval |= ((cfg->dummy_clks - 1) << SF_CTRL_IF_0_DMY_BYTE_SHIFT);
	} else {
		regval &= ~SF_CTRL_IF_0_DMY_EN;
	}

	/* Configure data */
	regval &= ~SF_CTRL_IF_0_DAT_BYTE_MASK;
	if (cfg->nb_data != 0) {
		regval |= SF_CTRL_IF_0_DAT_EN;
		regval |= ((cfg->nb_data - 1) << SF_CTRL_IF_0_DAT_BYTE_SHIFT);
	} else {
		regval &= ~SF_CTRL_IF_0_DAT_EN;
	}

	/* Set read write flag */
	if (cfg->rw_flag)
		regval |= SF_CTRL_IF_0_DAT_RW;
	else
		regval &= ~SF_CTRL_IF_0_DAT_RW;
	putreg32(regval, cmd_offset + SF_CTRL_IF_SAHB_0_OFFSET);

	bflb_sf_ctrl_select_clock(1);

	/* Trigger */
	regval |= SF_CTRL_IF_0_TRIG;
	putreg32(regval, cmd_offset + SF_CTRL_IF_SAHB_0_OFFSET);

	time_out = SF_CTRL_BUSY_STATE_TIMEOUT;
	while (bflb_sf_ctrl_get_busy_state()) {
		time_out--;

		if (time_out == 0) {
			bflb_sf_ctrl_select_clock(0);
			return;
		}
	}

	bflb_sf_ctrl_select_clock(0);
}

static int bflb_sflash_read_reg(spi_flash_cfg_type *flash_cfg, uint8_t reg_index, uint8_t *reg_value, uint8_t reg_len)
{
	uint8_t __iomem *const flash_ctrl_buf = BFLB_SF_CTRL_BUF_BASE;
	struct sf_ctrl_cmd_cfg_type flash_cmd;
	uint32_t cnt = 0;

	memset(&flash_cmd, 0, sizeof(flash_cmd));

	flash_cmd.cmd_buf[0] = (flash_cfg->read_reg_cmd[reg_index]) << 24;
	flash_cmd.rw_flag = SF_CTRL_READ;
	flash_cmd.nb_data = reg_len;

	bflb_sf_ctrl_sendcmd(&flash_cmd);

	while (bflb_sf_ctrl_get_busy_state()) {
		udelay(1);
		cnt++;

		if (cnt > 1000)
			return -1;
	}

	memcpy_fromio(reg_value, flash_ctrl_buf, reg_len);
	return 0;
}

static int bflb_sflash_write_enable(spi_flash_cfg_type *flash_cfg)
{
	uint32_t stat = 0;
	struct sf_ctrl_cmd_cfg_type flash_cmd;

	memset(&flash_cmd, 0, sizeof(flash_cmd));

	/* Write enable*/
	flash_cmd.cmd_buf[0] = (flash_cfg->write_enable_cmd) << 24;
	/* rw_flag don't care */
	flash_cmd.rw_flag = SF_CTRL_READ;
	bflb_sf_ctrl_sendcmd(&flash_cmd);

	bflb_sflash_read_reg(flash_cfg, flash_cfg->wr_enable_index, (uint8_t *)&stat, flash_cfg->wr_enable_read_reg_len);

	if ((stat & (1 << flash_cfg->wr_enable_bit)) != 0)
		return 0;

	return -1;
}

static int bflb_sflash_read(spi_flash_cfg_type *flash_cfg, uint8_t io_mode, uint8_t cont_read, uint32_t addr, uint8_t *data, uint32_t len)
{
	uint8_t __iomem *const flash_ctrl_buf = BFLB_SF_CTRL_BUF_BASE;
	uint32_t cur_len = 0, i = 0;
	uint8_t cmd = 0, dummy_clks = 0;
	uint32_t timeout = 0;
	struct sf_ctrl_cmd_cfg_type flash_cmd;
	uint8_t no_read_mode_cfg = 0;
	uint8_t c_read_support = 0;
	uint8_t is_32bits_addr = 0;

	memset(&flash_cmd, 0, sizeof(flash_cmd));

	if (io_mode == SF_CTRL_NIO_MODE) {
		cmd = flash_cfg->fast_read_cmd;
		dummy_clks = flash_cfg->fr_dmy_clk;
	} else if (io_mode == SF_CTRL_DO_MODE) {
		flash_cmd.data_mode = SF_CTRL_DATA_2_LINES;
		cmd = flash_cfg->fast_read_do_cmd;
		dummy_clks = flash_cfg->fr_do_dmy_clk;
	} else if (io_mode == SF_CTRL_DIO_MODE) {
		flash_cmd.addr_mode = SF_CTRL_ADDR_2_LINES;
		flash_cmd.data_mode = SF_CTRL_DATA_2_LINES;
		cmd = flash_cfg->fast_read_dio_cmd;
		dummy_clks = flash_cfg->fr_dio_dmy_clk;
	} else if (io_mode == SF_CTRL_QO_MODE) {
		flash_cmd.data_mode = SF_CTRL_DATA_4_LINES;
		cmd = flash_cfg->fast_read_qo_cmd;
		dummy_clks = flash_cfg->fr_qo_dmy_clk;
	} else if (io_mode == SF_CTRL_QIO_MODE) {
		flash_cmd.addr_mode = SF_CTRL_ADDR_4_LINES;
		flash_cmd.data_mode = SF_CTRL_DATA_4_LINES;
		cmd = flash_cfg->fast_read_qio_cmd;
		dummy_clks = flash_cfg->fr_qio_dmy_clk;
	} else {
		return -1;
	}

	is_32bits_addr = (flash_cfg->io_mode & 0x20);
	/* Prepare command */
	flash_cmd.rw_flag = SF_CTRL_READ;
	flash_cmd.addr_size = 3;

	if (is_32bits_addr > 0)
		flash_cmd.addr_size++;

	if (io_mode == SF_CTRL_QIO_MODE || io_mode == SF_CTRL_DIO_MODE) {
		no_read_mode_cfg = flash_cfg->c_read_support & 0x02;
		c_read_support = flash_cfg->c_read_support & 0x01;

		if (no_read_mode_cfg == 0) {
			/* Read mode must be set */
			if (c_read_support == 0) {
				/* Not support cont read, but we still need set read mode(winbond 80dv) */
				if (is_32bits_addr > 0)
					flash_cmd.cmd_buf[1] |= (flash_cfg->c_read_mode << 16);
				else
					flash_cmd.cmd_buf[1] = (flash_cfg->c_read_mode << 24);
			} else {
				/* Flash support cont read, setting depend on user parameter */
				if (cont_read) {
					if (is_32bits_addr > 0)
						flash_cmd.cmd_buf[1] |= (flash_cfg->c_read_mode << 16);
					else
						flash_cmd.cmd_buf[1] = (flash_cfg->c_read_mode << 24);
				} else {
					if (is_32bits_addr > 0)
						flash_cmd.cmd_buf[1] |= ((!flash_cfg->c_read_mode) << 16);
					else
						flash_cmd.cmd_buf[1] = ((!flash_cfg->c_read_mode) << 24);
				}
			}

			flash_cmd.addr_size++;
		}
	}

	flash_cmd.dummy_clks = dummy_clks;

	/* Read data */
	for (i = 0; i < len;) {
		/* Prepare command */
		if (is_32bits_addr > 0) {
			flash_cmd.cmd_buf[0] = (cmd << 24) | (addr >> 8);
			flash_cmd.cmd_buf[1] |= (addr << 24);
		} else {
			flash_cmd.cmd_buf[0] = (cmd << 24) | (addr);
		}

		cur_len = len - i;

		if (cur_len >= NOR_FLASH_CTRL_BUF_SIZE) {
			cur_len = NOR_FLASH_CTRL_BUF_SIZE;
			flash_cmd.nb_data = cur_len;
		} else {
			/* Make sf_ctrl word read */
			flash_cmd.nb_data = ((cur_len + 3) >> 2) << 2;
		}

		bflb_sf_ctrl_sendcmd(&flash_cmd);

		timeout = SF_CTRL_BUSY_STATE_TIMEOUT;

		while (bflb_sf_ctrl_get_busy_state()) {
			timeout--;

			if (timeout == 0)
				return -2;
		}

		memcpy_fromio(data, flash_ctrl_buf, cur_len);

		addr += cur_len;
		i += cur_len;
		data += cur_len;
	}

	return 0;
}

static int bflb_sflash_busy(spi_flash_cfg_type *flash_cfg)
{
	uint32_t stat = 0;

	bflb_sflash_read_reg(flash_cfg, flash_cfg->busy_index, (uint8_t *)&stat, flash_cfg->busy_read_reg_len);

	if ((stat & (1 << flash_cfg->busy_bit)) == 0)
		return 0;

	return 1;
}

static int bflb_sflash_program(spi_flash_cfg_type *flash_cfg, uint8_t io_mode, uint32_t addr, const uint8_t *data, uint32_t len)
{
	uint8_t __iomem *const flash_ctrl_buf = BFLB_SF_CTRL_BUF_BASE;
	uint32_t i = 0, cur_len = 0;
	uint32_t cnt = 0;
	int stat = 0;
	uint8_t is_32bits_addr = 0;
	uint8_t cmd = 0;
	struct sf_ctrl_cmd_cfg_type flash_cmd;

	memset(&flash_cmd, 0, sizeof(flash_cmd));

	if (io_mode == SF_CTRL_NIO_MODE || io_mode == SF_CTRL_DO_MODE || io_mode == SF_CTRL_DIO_MODE) {
		cmd = flash_cfg->page_program_cmd;
	} else if (io_mode == SF_CTRL_QIO_MODE || io_mode == SF_CTRL_QO_MODE) {
		flash_cmd.addr_mode = flash_cfg->qpp_addr_mode;
		flash_cmd.data_mode = SF_CTRL_DATA_4_LINES;
		cmd = flash_cfg->qpage_program_cmd;
	} else {
		return -1;
	}

	is_32bits_addr = (flash_cfg->io_mode & 0x20);
	/* Prepare command */
	flash_cmd.rw_flag = SF_CTRL_WRITE;
	flash_cmd.addr_size = 3;

	if (is_32bits_addr > 0)
		flash_cmd.addr_size++;

	for (i = 0; i < len;) {
		/* Write enable is needed for every program */
		stat = bflb_sflash_write_enable(flash_cfg);

		if (stat != 0)
			return stat;

		/* Get current programmed length within page size */
		cur_len = flash_cfg->page_size - addr % flash_cfg->page_size;

		if (cur_len > len - i)
			cur_len = len - i;

		/* Prepare command */
		memcpy_toio(flash_ctrl_buf, data, cur_len);

		if (is_32bits_addr > 0) {
			flash_cmd.cmd_buf[0] = (cmd << 24) | (addr >> 8);
			flash_cmd.cmd_buf[1] = (addr << 24);
		} else {
			flash_cmd.cmd_buf[0] = (cmd << 24) | (addr);
		}

		flash_cmd.nb_data = cur_len;

		bflb_sf_ctrl_sendcmd(&flash_cmd);

		/* Adjust address and programmed length */
		addr += cur_len;
		i += cur_len;
		data += cur_len;

		/* Wait for write done */
		cnt = 0;

		while (bflb_sflash_busy(flash_cfg)) {
			usleep_range(80, 100);
			cnt++;

			if (cnt > flash_cfg->time_page_pgm * 20)
				return -1;
		}
	}

	return 0;
}

static int bflb_sflash_sz_erase(spi_flash_cfg_type *flash_cfg, uint32_t blk_num, uint32_t blk_size, uint8_t erase_cmd, uint32_t time_limit)
{
	struct sf_ctrl_cmd_cfg_type flash_cmd;
	uint32_t cnt = 0;
	uint8_t is_32bits_addr = 0;
	int stat = bflb_sflash_write_enable(flash_cfg);

	if (stat != 0)
		return stat;

	memset(&flash_cmd, 0, sizeof(flash_cmd));

	is_32bits_addr = (flash_cfg->io_mode & 0x20);
	/* rw_flag don't care */
	flash_cmd.rw_flag = SF_CTRL_READ;
	flash_cmd.addr_size = 3;

	if (is_32bits_addr > 0) {
		flash_cmd.addr_size++;
		flash_cmd.cmd_buf[0] = (erase_cmd << 24) | ((blk_size * blk_num) >> 8);
		flash_cmd.cmd_buf[1] = ((blk_size * blk_num) << 24);
	} else {
		flash_cmd.cmd_buf[0] = (erase_cmd << 24) | (blk_size * blk_num);
	}

	bflb_sf_ctrl_sendcmd(&flash_cmd);

	while (bflb_sflash_busy(flash_cfg)) {
		usleep_range(450, 500);
		cnt++;

		if (cnt > time_limit * 3)
			return -1;
	}

	return 0;
}

static int bflb_sflash_erase(spi_flash_cfg_type *flash_cfg, uint32_t start_addr, uint32_t end_addr)
{
	uint32_t len = 0;
	uint32_t erase_len = 0;
	uint8_t ret = 0;

	if (start_addr > end_addr)
		return -1;

	while (start_addr <= end_addr) {
		len = end_addr - start_addr + 1;

		if (flash_cfg->blk64_erase_cmd != BFLB_SPIFLASH_CMD_INVALID &&
				(start_addr & (BFLB_SPIFLASH_BLK64K_SIZE - 1)) == 0 &&
				len > (BFLB_SPIFLASH_BLK64K_SIZE - flash_cfg->sector_size * 1024)) {
			/* 64K margin address,and length > 64K-sector size, erase one first */
			ret = bflb_sflash_sz_erase(flash_cfg, start_addr / BFLB_SPIFLASH_BLK64K_SIZE, BFLB_SPIFLASH_BLK64K_SIZE, flash_cfg->blk64_erase_cmd, flash_cfg->time_e_64k);
			erase_len = BFLB_SPIFLASH_BLK64K_SIZE;
		} else if (flash_cfg->blk32_erase_cmd != BFLB_SPIFLASH_CMD_INVALID &&
				(start_addr & (BFLB_SPIFLASH_BLK32K_SIZE - 1)) == 0 &&
				len > (BFLB_SPIFLASH_BLK32K_SIZE - flash_cfg->sector_size * 1024)) {
			/* 32K margin address,and length > 32K-sector size, erase one first */
			ret = bflb_sflash_sz_erase(flash_cfg, start_addr / BFLB_SPIFLASH_BLK32K_SIZE, BFLB_SPIFLASH_BLK32K_SIZE, flash_cfg->blk32_erase_cmd, flash_cfg->time_e_32k);
			erase_len = BFLB_SPIFLASH_BLK32K_SIZE;
		} else {
			/* Sector erase */
			start_addr = ((start_addr) & (~(flash_cfg->sector_size * 1024 - 1)));
			ret = bflb_sflash_sz_erase(flash_cfg, start_addr / flash_cfg->sector_size / 1024, flash_cfg->sector_size * 1024, flash_cfg->sector_erase_cmd, flash_cfg->time_e_sector);
			erase_len = flash_cfg->sector_size * 1024;
		}

		start_addr += erase_len;

		if (ret != 0)
			return -1;
	}

	return 0;
}

static void bflb_sflash_reset_continue_read(spi_flash_cfg_type *flash_cfg)
{
	struct sf_ctrl_cmd_cfg_type flash_cmd;

	memset(&flash_cmd, 0, sizeof(flash_cmd));

	/* Reset continous read */
	memset(&flash_cmd.cmd_buf[0], flash_cfg->reset_c_read_cmd, 4);
	/* rw_flag don't care */
	flash_cmd.rw_flag = SF_CTRL_READ;
	flash_cmd.addr_size = flash_cfg->reset_c_read_cmd_size;
	bflb_sf_ctrl_sendcmd(&flash_cmd);
}

static void bflb_sflash_disable_burst_wrap(spi_flash_cfg_type *flash_cfg)
{
	uint8_t __iomem *const flash_ctrl_buf = BFLB_SF_CTRL_BUF_BASE;
	uint8_t cmd = 0, dummy_clks = 0;
	uint32_t wrap_data = 0;
	struct sf_ctrl_cmd_cfg_type flash_cmd;

	memset(&flash_cmd, 0, sizeof(flash_cmd));

	flash_cmd.addr_mode = flash_cfg->de_burst_wrap_data_mode;
	flash_cmd.data_mode = flash_cfg->de_burst_wrap_data_mode;
	dummy_clks = flash_cfg->de_burst_wrap_cmd_dmy_clk;
	cmd = flash_cfg->de_burst_wrap_cmd;
	wrap_data = flash_cfg->de_burst_wrap_data;
	memcpy_toio(flash_ctrl_buf, &wrap_data, 4);
	flash_cmd.cmd_buf[0] = (cmd << 24);
	flash_cmd.rw_flag = SF_CTRL_WRITE;
	flash_cmd.dummy_clks = dummy_clks;
	flash_cmd.nb_data = 1;

	bflb_sf_ctrl_sendcmd(&flash_cmd);
}

static void bflb_sf_ctrl_32bits_addr_en(uint8_t en32_bits_addr)
{
	uint32_t regval = 0;

	regval = getreg32(reg_base + SF_CTRL_0_OFFSET);
	if (en32_bits_addr)
		regval |= SF_CTRL_SF_IF_32B_ADR_EN;
	else
		regval &= ~SF_CTRL_SF_IF_32B_ADR_EN;
	putreg32(regval, reg_base + SF_CTRL_0_OFFSET);
}

static void bflb_sf_ctrl_set_flash_image_offset(uint32_t addr_offset, uint8_t group, uint8_t bank)
{
	if (bank == SF_CTRL_FLASH_BANK0) {
		if (group)
			putreg32(addr_offset, reg_base + SF_CTRL_SF_ID1_OFFSET_OFFSET);
		else
			putreg32(addr_offset, reg_base + SF_CTRL_SF_ID0_OFFSET_OFFSET);
	}
}

static int bflb_sflash_set_32bits_addr_mode(spi_flash_cfg_type *flash_cfg, uint8_t en_32bits_addr)
{
	struct sf_ctrl_cmd_cfg_type flash_cmd;
	uint8_t cmd = 0;

	if ((flash_cfg->io_mode & 0x20) == 0)
		return -1;

	memset(&flash_cmd, 0, sizeof(flash_cmd));

	bflb_sf_ctrl_32bits_addr_en(en_32bits_addr);

	if (en_32bits_addr)
		cmd = flash_cfg->enter_32bits_addr_cmd;
	else
		cmd = flash_cfg->exit_32bits_addr_cmd;

	flash_cmd.cmd_buf[0] = (cmd << 24);
	/* rw_flag don't care */
	flash_cmd.rw_flag = SF_CTRL_READ;

	bflb_sf_ctrl_sendcmd(&flash_cmd);

	return 0;
}

static int bflb_sflash_write_reg(spi_flash_cfg_type *flash_cfg, uint8_t reg_index, uint8_t *reg_value, uint8_t reg_len)
{
	uint8_t __iomem *const flash_ctrl_buf = BFLB_SF_CTRL_BUF_BASE;
	uint32_t cnt = 0;
	struct sf_ctrl_cmd_cfg_type flash_cmd;

	memset(&flash_cmd, 0, sizeof(flash_cmd));

	memcpy_toio(flash_ctrl_buf, reg_value, reg_len);

	flash_cmd.cmd_buf[0] = (flash_cfg->write_reg_cmd[reg_index]) << 24;
	flash_cmd.rw_flag = SF_CTRL_WRITE;
	flash_cmd.nb_data = reg_len;

	bflb_sf_ctrl_sendcmd(&flash_cmd);

	/* take 40ms for tw(write status register) as default */
	while (bflb_sflash_busy(flash_cfg)) {
		usleep_range(80, 100);
		cnt++;

		if (cnt > 400)
			return -1;
	}

	return 0;
}

static int bflb_sflash_qspi_enable(spi_flash_cfg_type *flash_cfg)
{
	uint32_t stat = 0, ret = 0;

	if (flash_cfg->qe_read_reg_len == 0) {
		ret = bflb_sflash_write_enable(flash_cfg);

		if (ret == 0)
			return -1;

		bflb_sflash_write_reg(flash_cfg, flash_cfg->qe_index, (uint8_t *)&stat, flash_cfg->qe_write_reg_len);
		return 0;
	}

	bflb_sflash_read_reg(flash_cfg, flash_cfg->qe_index, (uint8_t *)&stat, flash_cfg->qe_read_reg_len);

	if (flash_cfg->qe_data == 0) {
		if ((stat & (1 << flash_cfg->qe_bit)) != 0)
			return 0;
	} else {
		if (((stat >> (flash_cfg->qe_bit & 0x08)) & 0xff) == flash_cfg->qe_data)
			return 0;
	}

	if (flash_cfg->qe_write_reg_len != 1) {
		/* This is read r0,read r1 write r0,r1 case */
		bflb_sflash_read_reg(flash_cfg, 0, (uint8_t *)&stat, 1);
		bflb_sflash_read_reg(flash_cfg, 1, ((uint8_t *)&stat) + 1, 1);

		if (flash_cfg->qe_data == 0) {
			stat |= (1 << (flash_cfg->qe_bit + 8 * flash_cfg->qe_index));
		} else {
			stat = stat & (~(0xff << (8 * flash_cfg->qe_index)));
			stat |= (flash_cfg->qe_data << (8 * flash_cfg->qe_index));
		}
	} else {
		if (flash_cfg->qe_data == 0)
			stat |= (1 << (flash_cfg->qe_bit % 8));
		else
			stat = flash_cfg->qe_data;
	}

	ret = bflb_sflash_write_enable(flash_cfg);

	if (ret != 0)
		return -1;

	bflb_sflash_write_reg(flash_cfg, flash_cfg->qe_index, (uint8_t *)&stat, flash_cfg->qe_write_reg_len);
	bflb_sflash_read_reg(flash_cfg, flash_cfg->qe_index, (uint8_t *)&stat, flash_cfg->qe_read_reg_len);

	if (flash_cfg->qe_data == 0) {
		if ((stat & (1 << flash_cfg->qe_bit)) != 0)
			return 0;
	} else {
		if (((stat >> (flash_cfg->qe_bit & 0x08)) & 0xff) == flash_cfg->qe_data)
			return 0;
	}

	return -1;
}

static int bl_sflash_mtd_read(struct mtd_info *mtd, loff_t from, size_t len,
			      size_t *retlen, u_char *buf)
{
	struct bl_sflash *sf = dev_get_drvdata(mtd->dev.parent);
	uint8_t io_mode = p_flash_cfg->io_mode & 0xf;
	int ret;

	dev_dbg(sf->dev, "%s from 0x%08x, len %zd\n",
		__func__, (u32)from, len);

	mutex_lock(&sf->lock);
	ret = bflb_sflash_read(p_flash_cfg, io_mode, 0, from, buf, len);
	*retlen = len;
	mutex_unlock(&sf->lock);

	return ret;
}

static int bl_sflash_mtd_write(struct mtd_info *mtd, loff_t to, size_t len,
			   size_t *retlen, const u_char *buf)
{
	struct bl_sflash *sf = dev_get_drvdata(mtd->dev.parent);
	uint8_t io_mode = p_flash_cfg->io_mode & 0xf;

	int ret = 0;

	dev_dbg(sf->dev, "%s to 0x%08x, len %zd\n", __func__, (u32)to, len);

	mutex_lock(&sf->lock);
	ret = bflb_sflash_program(p_flash_cfg, io_mode, to, buf, len);
	*retlen = len;
	mutex_unlock(&sf->lock);

	return ret;
}

static int bl_sflash_mtd_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	struct bl_sflash *sf = dev_get_drvdata(mtd->dev.parent);
	u32 addr, len;
	int ret;

	dev_dbg(sf->dev, "%s at 0x%llx, len %lld\n", __func__,
		(long long)instr->addr, (long long)instr->len);

	addr = instr->addr;
	len = instr->len;

	mutex_lock(&sf->lock);
	ret = bflb_sflash_erase(p_flash_cfg, addr, addr + len - 1);
	mutex_unlock(&sf->lock);

	return ret;
}

static int bl_sflash_init(struct bl_sflash *sf, struct xram_flash_cfg __iomem *cfg)
{
	memcpy_fromio(&sf->flash_cfg, &cfg->flash_cfg, sizeof(sf->flash_cfg));
	p_flash_cfg = &sf->flash_cfg;

	print_hex_dump(KERN_DEBUG, "flash_cfg: ", DUMP_PREFIX_OFFSET, 16, 1, p_flash_cfg, sizeof(*p_flash_cfg), false);
	printk(KERN_DEBUG "flash_cfg: size %u", cfg->flash_size);

	bflb_sf_ctrl_set_owner(SF_CTRL_OWNER_SAHB);
	/* Exit form continous read for accepting command */
	bflb_sflash_reset_continue_read(p_flash_cfg);
	/* For disable command that is setting register instead of send command, we need write enable */
	bflb_sflash_disable_burst_wrap(p_flash_cfg);
	/* Enable 32Bits addr mode again in case reset command make it reset */
	bflb_sflash_set_32bits_addr_mode(p_flash_cfg, 1);
	if ((p_flash_cfg->io_mode & 0x0f) == SF_CTRL_QO_MODE || (p_flash_cfg->io_mode & 0x0f) == SF_CTRL_QIO_MODE)
		/* Enable QE again in case reset command make it reset */
		bflb_sflash_qspi_enable(p_flash_cfg);
	/* Deburst again to make sure */
	bflb_sflash_disable_burst_wrap(p_flash_cfg);

	/* Clear offset setting */
	bflb_sf_ctrl_set_flash_image_offset(0, 0, 0);
	return 0;
}

static int bl_sflash_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct resource *res;
	struct resource *res_cfg;
	struct xram_flash_cfg __iomem *cfg;
	uint32_t flash_size;
	struct bl_sflash *sf;
	int ret;

	if (!np) {
		dev_err(&pdev->dev, "No DT found\n");
		return -EINVAL;
	}

	sf = devm_kzalloc(&pdev->dev, sizeof(*sf), GFP_KERNEL);
	if (!sf)
		return -ENOMEM;

	sf->dev = &pdev->dev;

	platform_set_drvdata(pdev, sf);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Resource not found\n");
		devm_kfree(&pdev->dev, sf);
		return -ENODEV;
	}

	res_cfg = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		dev_err(&pdev->dev, "flash cfg addr not found\n");
		devm_kfree(&pdev->dev, sf);
		return -ENODEV;
	}

	reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(reg_base)) {
		devm_kfree(&pdev->dev, sf);
		return PTR_ERR(reg_base);
	}

	cfg = devm_ioremap_resource(&pdev->dev, res_cfg);
	if (IS_ERR(cfg)) {
		devm_kfree(&pdev->dev, sf);
		devm_ioremap_release(&pdev->dev, reg_base);
		return PTR_ERR(cfg);
	}

	mutex_init(&sf->lock);

	ret = bl_sflash_init(sf, cfg);
	memcpy_fromio(&flash_size, &cfg->flash_size, sizeof(flash_size));
	devm_ioremap_release(&pdev->dev, cfg);
	if (ret) {
		dev_err(&pdev->dev, "Failed to initialise sflash Controller\n");
		devm_kfree(&pdev->dev, sf);
		devm_ioremap_release(&pdev->dev, reg_base);
		return ret;
	}

	sf->mtd.name		= "bflb_sflash";
	sf->mtd.dev.parent	= &pdev->dev;
	mtd_set_of_node(&sf->mtd, np);
	sf->mtd.type		= MTD_NORFLASH;
	sf->mtd.writesize	= 1;
	sf->mtd.writebufsize	= NOR_FLASH_CTRL_BUF_SIZE;
	sf->mtd.flags		= MTD_CAP_NORFLASH;
	sf->mtd.size		= flash_size;
	sf->mtd.erasesize	= p_flash_cfg->sector_size * 1024;

	sf->mtd._read  = bl_sflash_mtd_read;
	sf->mtd._write = bl_sflash_mtd_write;
	sf->mtd._erase = bl_sflash_mtd_erase;

	return mtd_device_register(&sf->mtd, NULL, 0);
}

static int bl_sflash_remove(struct platform_device *pdev)
{
	struct bl_sflash *sf = platform_get_drvdata(pdev);

	WARN_ON(mtd_device_unregister(&sf->mtd));

	return 0;
}

static const struct of_device_id bl_sflash_match[] = {
	{ .compatible = "bflb,bl808-sflash", },
	{},
};
MODULE_DEVICE_TABLE(of, bl_match);

static struct platform_driver bl_sflash_driver = {
	.probe		= bl_sflash_probe,
	.remove		= bl_sflash_remove,
	.driver		= {
		.name	= "bflb-sflash",
		.of_match_table = bl_sflash_match,
	},
};
module_platform_driver(bl_sflash_driver);

MODULE_AUTHOR("Chien Wong <qwang@bouffalolab.com>");
MODULE_DESCRIPTION("Bouffalo Lab sflash driver");
MODULE_LICENSE("GPL");
