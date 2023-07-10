// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2020-2021 SiFive, Inc
 *
 * Authors:
 *   Pragnesh Patel <pragnesh.patel@sifive.com>
 */

#include <init.h>
#include <spl.h>
#include <misc.h>
#include <log.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <asm/gpio.h>
#include <asm/arch/spl.h>

int spl_board_init_f(void)
{
	/*nothing init*/
	return 0;
}

u32 spl_boot_device(void)
{
	debug("trying boot from MMC1\n");
	return BOOT_DEVICE_MMC1;
}

#ifdef CONFIG_SPL_LOAD_FIT
int board_fit_config_name_match(const char *name)
{
	/* boot using first FIT config */
	return 0;
}
#endif
