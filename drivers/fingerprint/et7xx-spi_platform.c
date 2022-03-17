/*
 * Copyright (C) 2016 Samsung Electronics. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include "fingerprint.h"
#include "et7xx.h"
#include <linux/platform_data/spi-mt65xx.h>
#ifdef ENABLE_SENSORS_FPRINT_SECURE
#if defined(CONFIG_SECURE_OS_BOOSTER_API)
#include <mach/secos_booster.h>
#elif defined(CONFIG_TZDEV_BOOST)
#if defined(CONFIG_TEEGRIS_VERSION) && (CONFIG_TEEGRIS_VERSION >= 4)
#include <../drivers/misc/tzdev/extensions/boost.h>
#else
#include <../drivers/misc/tzdev/tz_boost.h>
#endif
#endif
#endif

#ifndef ENABLE_SENSORS_FPRINT_SECURE
static struct mtk_chip_config mtk_chip_info = {
	.rx_mlsb = 1,
	.tx_mlsb = 1,

	.sample_sel = 0,
	.cs_setuptime = 0,
	.cs_holdtime = 0,
	.cs_idletime = 0,
	.deassert_mode = 0,
	.tick_delay = 0,
};
#endif

int fps_resume_set(void) {
	return 0;
}

int fps_suspend_set(struct et7xx_data* etspi) {
	return 0;
}

int et7xx_register_platform_variable(struct et7xx_data* etspi)
{
	int retval = 0;

	pr_info("Entry\n");
#ifdef ENABLE_SENSORS_FPRINT_SECURE

#endif
	return retval;
}

int et7xx_unregister_platform_variable(struct et7xx_data* etspi)
{
	int retval = 0;

	pr_info("Entry\n");
#ifdef ENABLE_SENSORS_FPRINT_SECURE

#endif

	return retval;
}

#ifndef ENABLE_SENSORS_FPRINT_SECURE
void et7xx_get_ctrldata(struct spi_device* spi)
{
	struct device_node* np, * data_np = NULL;
	u32 tckdly = 0;

	np = spi->dev.of_node;
	if (!np) {
		pr_err("%s : device node not founded\n", __func__);
		return;
	}

	data_np = of_get_child_by_name(np, "controller-data");
	if (!data_np) {
		pr_err("%s : controll-data not founded\n", __func__);
		return;
	}

	of_property_read_u32(data_np, "mediatek,tckdly", &tckdly);
	mtk_chip_info.tick_delay = tckdly;

	of_node_put(data_np);
	spi->controller_data = (void*)&mtk_chip_info;
	pr_info("%s done\n", __func__);
}
#endif

#ifdef ENABLE_SENSORS_FPRINT_SECURE
static int et7xx_sec_spi_prepare(struct et7xx_data* etspi)
{

	int retval = 0;



	return retval;
}

static int et7xx_sec_spi_unprepare(struct et7xx_data* etspi)
{


	return 0;
}
#endif

int et7xx_spi_clk_enable(struct et7xx_data* etspi)
{
	int retval = 0;
#ifdef ENABLE_SENSORS_FPRINT_SECURE
	pr_info("FP_ENABLE_SPI_CLOCK0\n");
	if (etspi->enabled_clk) {
		if (etspi->spi_speed == SLOW_BAUD_RATE) {
			pr_info("Already same clock.\n");
			return 0;
		}
		pr_info("Already enabled, DISABLE_SPI_CLOCK\n");
		mt_spi_disable_master_clk(etspi->spi);
		wake_unlock(&etspi->fp_spi_lock);
		etspi->enabled_clk = false;
}
	pr_info("FP_ENABLE_SPI_CLOCK1\n");
	etspi->spi_speed = SLOW_BAUD_RATE;
	wake_lock(&etspi->fp_spi_lock);
	mt_spi_enable_master_clk(etspi->spi);
	etspi->enabled_clk = true;
#else
	etspi->spi_speed = SLOW_BAUD_RATE;

#endif
	return retval;
}

int et7xx_spi_clk_disable(struct et7xx_data* etspi)
{
	int retval = 0;
#ifdef ENABLE_SENSORS_FPRINT_SECURE
	pr_info("FP_DISABLE_SPI_CLOCK\n");
	if (etspi->enabled_clk) {
		pr_info("%s DISABLE_SPI_CLOCK\n", __func__);
		mt_spi_disable_master_clk(etspi->spi);
		wake_unlock(&etspi->fp_spi_lock);
		etspi->enabled_clk = false;
		pr_info("%s, clk disalbed\n", __func__);
	}

#endif
	return retval;
}

int et7xx_set_cpu_speedup(struct et7xx_data* etspi, int onoff)
{
	int retval = 0;
#ifdef ENABLE_SENSORS_FPRINT_SECURE

#endif /* ENABLE_SENSORS_FPRINT_SECURE */
	return retval;
}
