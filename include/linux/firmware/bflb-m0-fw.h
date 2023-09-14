// SPDX-License-Identifier: GPL-2.0-only
/*
 * Helpers for loading firmware/resetting M0 on BL808.
 *
 * Copyright (C) Bouffalo Lab 2016-2023
 */

#include <linux/device.h>
#include <linux/firmware.h>

int bflb_halt_m0(void);
int bflb_reset_m0(u32 reset_addr);
int bflb_load_m0_fw(struct device *dev, const char *fwname, u32 load_address);
int bflb_run_m0_fw(struct device *dev, const char *fwname, u32 load_address);
