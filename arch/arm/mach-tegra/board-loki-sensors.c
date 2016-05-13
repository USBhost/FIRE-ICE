/*
 * arch/arm/mach-tegra/board-loki-sensors.c
 *
 * Copyright (c) 2013-2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/mpu.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/nct1008.h>
#include <linux/tegra-fuse.h>
#include <linux/of_platform.h>
#include <media/camera.h>
#include <media/mt9m114.h>
#include <media/ov7695.h>
#include <mach/gpio-tegra.h>
#include <mach/edp.h>
#include <mach/io_dpd.h>
#include <linux/gpio.h>
#include <linux/therm_est.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/generic_adc_thermal.h>
#include <linux/pid_thermal_gov.h>

#include "board.h"
#include "board-common.h"
#include "board-loki.h"
#include "tegra-board-id.h"
#include <linux/platform/tegra/dvfs.h>
#include <linux/platform/tegra/cpu-tegra.h>

static struct board_info board_info;

#ifndef CONFIG_USE_OF

/* MPU board file definition    */
static struct mpu_platform_data mpu6050_gyro_data = {
	.int_config     = 0x10,
	.level_shifter  = 0,
	/* Located in board_[platformname].h */
	.orientation    = MPU_GYRO_ORIENTATION,
	.sec_slave_type = SECONDARY_SLAVE_TYPE_NONE,
	.key            = {0x4E, 0xCC, 0x7E, 0xEB, 0xF6, 0x1E, 0x35, 0x22,
			0x00, 0x34, 0x0D, 0x65, 0x32, 0xE9, 0x94, 0x89},
};

/* MPU board file definition    */
static struct mpu_platform_data mpu6050_gyro_data_fab_0 = {
	.int_config     = 0x10,
	.level_shifter  = 0,
	/* Located in board_[platformname].h */
	.orientation    = MPU_GYRO_ORIENTATION_FAB0,
	.sec_slave_type = SECONDARY_SLAVE_TYPE_NONE,
	.key            = {0x4E, 0xCC, 0x7E, 0xEB, 0xF6, 0x1E, 0x35, 0x22,
			0x00, 0x34, 0x0D, 0x65, 0x32, 0xE9, 0x94, 0x89},
};

/* MPU board file definition    */
static struct mpu_platform_data mpu6050_gyro_data_t_1_95 = {
	.int_config     = 0x10,
	.level_shifter  = 0,
	/* Located in board_[platformname].h */
	.orientation    = MPU_GYRO_ORIENTATION_T_1_95,
	.sec_slave_type = SECONDARY_SLAVE_TYPE_NONE,
	.key            = {0x4E, 0xCC, 0x7E, 0xEB, 0xF6, 0x1E, 0x35, 0x22,
			0x00, 0x34, 0x0D, 0x65, 0x32, 0xE9, 0x94, 0x89},
};

static struct mpu_platform_data mpu_compass_data = {
	.orientation	= MPU_COMPASS_ORIENTATION,
	.config		= NVI_CONFIG_BOOT_HOST,
};

static struct i2c_board_info __initdata inv_mpu6050_i2c0_board_info[] = {
	{
		I2C_BOARD_INFO(MPU_GYRO_NAME, MPU_GYRO_ADDR),
		.platform_data = &mpu6050_gyro_data,
	},
	{
		I2C_BOARD_INFO(MPU_COMPASS_NAME, MPU_COMPASS_ADDR),
		.platform_data = &mpu_compass_data,
	},
};

static void mpuirq_init(void)
{
	int ret = 0;
	unsigned gyro_irq_gpio = MPU_GYRO_IRQ_GPIO;
	unsigned gyro_bus_num = MPU_GYRO_BUS_NUM;
	char *gyro_name = MPU_GYRO_NAME;

	pr_info("*** MPU START *** mpuirq_init...\n");

	ret = gpio_request(gyro_irq_gpio, gyro_name);

	if (ret < 0) {
		pr_err("%s: gpio_request failed %d\n", __func__, ret);
		return;
	}

	ret = gpio_direction_input(gyro_irq_gpio);
	if (ret < 0) {
		pr_err("%s: gpio_direction_input failed %d\n", __func__, ret);
		gpio_free(gyro_irq_gpio);
		return;
	}
	pr_info("*** MPU END *** mpuirq_init...\n");

	if (board_info.board_id == BOARD_E2549)
		inv_mpu6050_i2c0_board_info[0].platform_data =
						&mpu6050_gyro_data_t_1_95;
	else {
		struct board_info displayboard_info;
		tegra_get_display_board_info(&displayboard_info);
		if (displayboard_info.fab == 0x0)
			inv_mpu6050_i2c0_board_info[0].platform_data =
				&mpu6050_gyro_data_fab_0;
	}
	inv_mpu6050_i2c0_board_info[0].irq = gpio_to_irq(MPU_GYRO_IRQ_GPIO);
	i2c_register_board_info(gyro_bus_num, inv_mpu6050_i2c0_board_info,
		ARRAY_SIZE(inv_mpu6050_i2c0_board_info));
}

#else

static void mpu_dt_update(void)
{
	struct device_node *np;
	struct board_info displayboard_info;
	static signed char mpu_gyro_orientation_1_95[] = {
		0,  1,  0,  0,  0,  1,  1,  0,  0
	};
	static signed char mpu_gyro_orientation_fab0[] = {
		0, -1,  0, -1,  0,  0,  0,  0, -1
	};
	static struct property orientation = {
		.name = "invensense,orientation",
		.value = mpu_gyro_orientation_1_95,
		.length = sizeof(mpu_gyro_orientation_1_95),
	};

	np = of_find_compatible_node(NULL, NULL, "invensense,mpu6050");
	if (np == NULL) {
		pr_err("%s: Cannot find mpu6050 node\n", __func__);
		return;
	}

	if (board_info.board_id == BOARD_E2549) {
		of_update_property(np, &orientation);
	} else {
		tegra_get_display_board_info(&displayboard_info);
		if (displayboard_info.fab == 0x0) {
			orientation.value = mpu_gyro_orientation_fab0;
			of_update_property(np, &orientation);
		}
	}

	of_node_put(np);
}

#endif

static struct tegra_io_dpd csia_io = {
	.name			= "CSIA",
	.io_dpd_reg_index	= 0,
	.io_dpd_bit		= 0,
};

static struct tegra_io_dpd csib_io = {
	.name			= "CSIB",
	.io_dpd_reg_index	= 0,
	.io_dpd_bit		= 1,
};

static struct tegra_io_dpd csie_io = {
	.name			= "CSIE",
	.io_dpd_reg_index	= 1,
	.io_dpd_bit		= 12,
};

static int loki_mt9m114_power_on(struct mt9m114_power_rail *pw)
{
	int err;
	if (unlikely(!pw || !pw->avdd || !pw->iovdd))
		return -EFAULT;

	/* disable CSIA IOs DPD mode to turn on front camera for ardbeg */
	tegra_io_dpd_disable(&csia_io);

	gpio_set_value(CAM_RSTN, 0);
	gpio_set_value(CAM2_PWDN, 1);
	usleep_range(1000, 1020);

	err = regulator_enable(pw->iovdd);
	if (unlikely(err))
		goto mt9m114_iovdd_fail;

	err = regulator_enable(pw->avdd);
	if (unlikely(err))
		goto mt9m114_avdd_fail;

	usleep_range(1000, 1020);
	gpio_set_value(CAM_RSTN, 1);
	gpio_set_value(CAM2_PWDN, 0);
	usleep_range(1000, 1020);

	/* return 1 to skip the in-driver power_on swquence */
	return 1;

mt9m114_avdd_fail:
	regulator_disable(pw->iovdd);

mt9m114_iovdd_fail:
	gpio_set_value(CAM_RSTN, 0);
	/* put CSIA IOs into DPD mode to save additional power for ardbeg */
	tegra_io_dpd_enable(&csia_io);
	return -ENODEV;
}

static int loki_mt9m114_power_off(struct mt9m114_power_rail *pw)
{
	if (unlikely(!pw || !pw->avdd || !pw->iovdd)) {
		/* put CSIA IOs into DPD mode to
		 * save additional power for ardbeg
		 */
		tegra_io_dpd_enable(&csia_io);
		return -EFAULT;
	}

	usleep_range(100, 120);
	gpio_set_value(CAM_RSTN, 0);
	usleep_range(100, 120);
	regulator_disable(pw->avdd);
	usleep_range(100, 120);
	regulator_disable(pw->iovdd);

	/* put CSIA IOs into DPD mode to save additional power for ardbeg */
	tegra_io_dpd_enable(&csia_io);
	return 1;
}

struct mt9m114_platform_data loki_mt9m114_pdata = {
	.power_on = loki_mt9m114_power_on,
	.power_off = loki_mt9m114_power_off,
};

static int loki_ov7695_power_on(struct ov7695_power_rail *pw)
{
	int err;

	if (unlikely(WARN_ON(!pw || !pw->avdd || !pw->iovdd)))
		return -EFAULT;

	/* disable CSIA IOs DPD mode to turn on front camera for ardbeg */
	tegra_io_dpd_disable(&csia_io);

	gpio_set_value(CAM2_PWDN, 0);
	usleep_range(1000, 1020);

	err = regulator_enable(pw->avdd);
	if (unlikely(err))
		goto ov7695_avdd_fail;
	usleep_range(300, 320);

	err = regulator_enable(pw->iovdd);
	if (unlikely(err))
		goto ov7695_iovdd_fail;
	usleep_range(1000, 1020);

	gpio_set_value(CAM2_PWDN, 1);
	usleep_range(1000, 1020);

	return 0;

ov7695_iovdd_fail:
	regulator_disable(pw->avdd);

ov7695_avdd_fail:
	gpio_set_value(CAM_RSTN, 0);
	/* put CSIA IOs into DPD mode to save additional power for ardbeg */
	tegra_io_dpd_enable(&csia_io);
	return -ENODEV;
}

static int loki_ov7695_power_off(struct ov7695_power_rail *pw)
{
	if (unlikely(WARN_ON(!pw || !pw->avdd || !pw->iovdd))) {
		/* put CSIA IOs into DPD mode to
		 * save additional power for ardbeg
		 */
		tegra_io_dpd_enable(&csia_io);
		return -EFAULT;
	}

	usleep_range(100, 120);

	regulator_disable(pw->iovdd);
	usleep_range(100, 120);

	regulator_disable(pw->avdd);
	usleep_range(100, 120);

	gpio_set_value(CAM2_PWDN, 0);

	/* put CSIA IOs into DPD mode to save additional power for ardbeg */
	tegra_io_dpd_enable(&csia_io);
	return 0;
}

struct ov7695_platform_data loki_ov7695_pdata = {
	.power_on = loki_ov7695_power_on,
	.power_off = loki_ov7695_power_off,
};

static struct camera_data_blob loki_camera_lut[] = {
	{"loki_ov7695_pdata", &loki_ov7695_pdata},
	{},
};

void __init loki_camera_auxdata(void *data)
{
	struct of_dev_auxdata *aux_lut = data;
	while (aux_lut && aux_lut->compatible) {
		if (!strcmp(aux_lut->compatible, "nvidia,tegra124-camera")) {
			pr_info("%s: update camera lookup table.\n", __func__);
			aux_lut->platform_data = loki_camera_lut;
		}
		aux_lut++;
	}
}

static int loki_camera_init(void)
{
	pr_debug("%s: ++\n", __func__);

	/* put CSIA/B/E IOs into DPD mode to
	 * save additional power
	 */
	tegra_io_dpd_enable(&csia_io);
	tegra_io_dpd_enable(&csib_io);
	tegra_io_dpd_enable(&csie_io);

	/* Don't turn on CAM_MCLK for FFD fab a3 or higher */
	if (board_info.board_id == BOARD_P2530 && board_info.fab >= 0xa3)
		loki_ov7695_pdata.mclk_name = "ext_mclk";

	return 0;
}

static struct throttle_table tj_throttle_table[] = {
	/* CPU_THROT_LOW cannot be used by other than CPU */
	/*      CPU,    GPU,  C2BUS,  C3BUS,   SCLK,    EMC   */
	{ { 2295000, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 2269500, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 2244000, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 2218500, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 2193000, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 2167500, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 2142000, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 2116500, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 2091000, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 2065500, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 2040000, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 2014500, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1989000, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1963500, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1938000, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1912500, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1887000, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1861500, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1836000, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1810500, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1785000, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1759500, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1734000, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1708500, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1683000, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1657500, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1632000, NO_CAP, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1606500, 790000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1581000, 776000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1555500, 762000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1530000, 749000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1504500, 735000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1479000, 721000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1453500, 707000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1428000, 693000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1402500, 679000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1377000, 666000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1351500, 652000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1326000, 638000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1300500, 624000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1275000, 610000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1249500, 596000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1224000, 582000, NO_CAP, NO_CAP, NO_CAP, 792000 } },
	{ { 1198500, 569000, NO_CAP, NO_CAP, NO_CAP, 792000 } },
	{ { 1173000, 555000, NO_CAP, NO_CAP, 360000, 792000 } },
	{ { 1147500, 541000, NO_CAP, NO_CAP, 360000, 792000 } },
	{ { 1122000, 527000, NO_CAP, 684000, 360000, 792000 } },
	{ { 1096500, 513000, 444000, 684000, 360000, 792000 } },
	{ { 1071000, 499000, 444000, 684000, 360000, 792000 } },
	{ { 1045500, 486000, 444000, 684000, 360000, 792000 } },
	{ { 1020000, 472000, 444000, 684000, 324000, 792000 } },
	{ {  994500, 458000, 444000, 684000, 324000, 792000 } },
	{ {  969000, 444000, 444000, 600000, 324000, 792000 } },
	{ {  943500, 430000, 444000, 600000, 324000, 792000 } },
	{ {  918000, 416000, 396000, 600000, 324000, 792000 } },
	{ {  892500, 402000, 396000, 600000, 324000, 792000 } },
	{ {  867000, 389000, 396000, 600000, 324000, 792000 } },
	{ {  841500, 375000, 396000, 600000, 288000, 792000 } },
	{ {  816000, 361000, 396000, 600000, 288000, 792000 } },
	{ {  790500, 347000, 396000, 600000, 288000, 792000 } },
	{ {  765000, 333000, 396000, 504000, 288000, 792000 } },
	{ {  739500, 319000, 348000, 504000, 288000, 792000 } },
	{ {  714000, 306000, 348000, 504000, 288000, 624000 } },
	{ {  688500, 292000, 348000, 504000, 288000, 624000 } },
	{ {  663000, 278000, 348000, 504000, 288000, 624000 } },
	{ {  637500, 264000, 348000, 504000, 288000, 624000 } },
	{ {  612000, 250000, 348000, 504000, 252000, 624000 } },
	{ {  586500, 236000, 348000, 504000, 252000, 624000 } },
	{ {  561000, 222000, 348000, 420000, 252000, 624000 } },
	{ {  535500, 209000, 288000, 420000, 252000, 624000 } },
	{ {  510000, 195000, 288000, 420000, 252000, 624000 } },
	{ {  484500, 181000, 288000, 420000, 252000, 624000 } },
	{ {  459000, 167000, 288000, 420000, 252000, 624000 } },
	{ {  433500, 153000, 288000, 420000, 252000, 396000 } },
	{ {  408000, 139000, 288000, 420000, 252000, 396000 } },
	{ {  382500, 126000, 288000, 420000, 252000, 396000 } },
	{ {  357000, 112000, 288000, 420000, 252000, 396000 } },
	{ {  331500,  98000, 288000, 420000, 252000, 396000 } },
	{ {  306000,  84000, 288000, 420000, 252000, 396000 } },
	{ {  280500,  84000, 288000, 420000, 252000, 396000 } },
	{ {  255000,  84000, 288000, 420000, 252000, 396000 } },
	{ {  229500,  84000, 288000, 420000, 252000, 396000 } },
	{ {  204000,  84000, 288000, 420000, 252000, 396000 } },
};

static struct balanced_throttle tj_throttle = {
	.throt_tab_size = ARRAY_SIZE(tj_throttle_table),
	.throt_tab = tj_throttle_table,
};

static int __init loki_throttle_init(void)
{
	if (of_machine_is_compatible("nvidia,loki") ||
		of_machine_is_compatible("nvidia,t132loki"))
		balanced_throttle_register(&tj_throttle, "tegra-balanced");
	return 0;
}
module_init(loki_throttle_init);

static int loki_fan_est_match(struct thermal_zone_device *thz, void *data)
{
	return (strcmp((char *)data, thz->type) == 0);
}

static int loki_fan_est_get_temp(void *data, long *temp)
{
	struct thermal_zone_device *thz;

	thz = thermal_zone_device_find(data, loki_fan_est_match);

	if (!thz || thz->ops->get_temp(thz, temp))
		*temp = 25000;

	return 0;
}

static struct thermal_zone_params fan_tzp = {
	.governor_name = "pid_thermal_gov",
};

static int active_trip_temps_loki[] = {0, 60000, 68000, 79000, 90000,
				140000, 150000, 160000, 170000, 180000};
static int active_hysteresis_loki[] = {0, 20000, 7000, 10000, 10000,
							0, 0, 0, 0, 0};

static int active_trip_temps_foster[] = {0, 63000, 74000, 85000, 120000,
				140000, 150000, 160000, 170000, 180000};
static int active_hysteresis_foster[] = {0, 15000, 11000, 6000, 4000,
							0, 0, 0, 0, 0};
/*Fan thermal estimator data for P2548*/
static struct therm_fan_est_data fan_est_data = {
	.toffset = 0,
	.polling_period = 1100,
	.ndevs = 2,
	.devs = {
			{
				.dev_data = "CPU-therm",
				.get_temp = loki_fan_est_get_temp,
				.coeffs = {
					50, 0, 0, 0,
					0, 0, 0, 0,
					0, 0, 0, 0,
					0, 0, 0, 0,
					0, 0, 0, 0
				},
			},
			{
				.dev_data = "GPU-therm",
				.get_temp = loki_fan_est_get_temp,
				.coeffs = {
					50, 0, 0, 0,
					0, 0, 0, 0,
					0, 0, 0, 0,
					0, 0, 0, 0,
					0, 0, 0, 0
				},
			},
	},
	.cdev_type = "pwm-fan",
	.tzp = &fan_tzp,
};

static struct platform_device loki_fan_therm_est_device = {
	.name   = "therm-fan-est",
	.id     = -1,
	.num_resources  = 0,
	.dev = {
		.platform_data = &fan_est_data,
	},
};

static int __init loki_fan_est_init(void)
{
	if ((board_info.sku == 900) && (board_info.board_id == BOARD_P2530)) {
		memcpy((&fan_est_data)->active_trip_temps,
				&active_trip_temps_foster,
				sizeof(active_trip_temps_foster));
		memcpy((&fan_est_data)->active_hysteresis,
				&active_hysteresis_foster,
				sizeof(active_hysteresis_foster));
	} else {
		memcpy((&fan_est_data)->active_trip_temps,
				&active_trip_temps_loki,
				sizeof(active_trip_temps_loki));
		memcpy((&fan_est_data)->active_hysteresis,
				&active_hysteresis_loki,
				sizeof(active_hysteresis_loki));
	}

	platform_device_register(&loki_fan_therm_est_device);

	return 0;
}

struct ntc_thermistor_adc_table {
	int temp; /* degree C */
	int adc;
};

static struct ntc_thermistor_adc_table loki_ffd_thermistor_table[] = {
	{ -40, 4082 }, { -39, 4082 }, { -38, 4081 }, { -37, 4079 },
	{ -36, 4078 }, { -35, 4077 }, { -34, 4075 }, { -33, 4074 },
	{ -32, 4072 }, { -31, 4071 }, { -30, 4068 }, { -29, 4067 },
	{ -28, 4065 }, { -27, 4063 }, { -26, 4060 }, { -25, 4058 },
	{ -24, 4055 }, { -23, 4053 }, { -22, 4050 }, { -21, 4047 },
	{ -20, 4043 }, { -19, 4040 }, { -18, 4036 }, { -17, 4033 },
	{ -16, 4028 }, { -15, 4023 }, { -14, 4019 }, { -13, 4015 },
	{ -12, 4010 }, { -11, 4004 }, { -10, 3998 }, {  -9, 3992 },
	{  -8, 3987 }, {  -7, 3980 }, {  -6, 3973 }, {  -5, 3964 },
	{  -4, 3958 }, {  -3, 3950 }, {  -2, 3942 }, {  -1, 3932 },
	{   0, 3922 }, {   1, 3913 }, {   2, 3904 }, {   3, 3893 },
	{   4, 3881 }, {   5, 3868 }, {   6, 3857 }, {   7, 3845 },
	{   8, 3832 }, {   9, 3817 }, {  10, 3801 }, {  11, 3788 },
	{  12, 3773 }, {  13, 3757 }, {  14, 3739 }, {  15, 3719 },
	{  16, 3703 }, {  17, 3685 }, {  18, 3666 }, {  19, 3644 },
	{  20, 3621 }, {  21, 3601 }, {  22, 3580 }, {  23, 3557 },
	{  24, 3532 }, {  25, 3504 }, {  26, 3481 }, {  27, 3456 },
	{  28, 3429 }, {  29, 3400 }, {  30, 3368 }, {  31, 3342 },
	{  32, 3314 }, {  33, 3283 }, {  34, 3250 }, {  35, 3214 },
	{  36, 3184 }, {  37, 3153 }, {  38, 3119 }, {  39, 3082 },
	{  40, 3043 }, {  41, 3010 }, {  42, 2975 }, {  43, 2938 },
	{  44, 2899 }, {  45, 2856 }, {  46, 2821 }, {  47, 2784 },
	{  48, 2744 }, {  49, 2702 }, {  50, 2658 }, {  51, 2621 },
	{  52, 2582 }, {  53, 2541 }, {  54, 2498 }, {  55, 2452 },
	{  56, 2414 }, {  57, 2374 }, {  58, 2332 }, {  59, 2288 },
	{  60, 2242 }, {  61, 2204 }, {  62, 2164 }, {  63, 2123 },
	{  64, 2079 }, {  65, 2034 }, {  66, 1996 }, {  67, 1958 },
	{  68, 1917 }, {  69, 1875 }, {  70, 1832 }, {  71, 1795 },
	{  72, 1758 }, {  73, 1719 }, {  74, 1679 }, {  75, 1638 },
	{  76, 1604 }, {  77, 1568 }, {  78, 1532 }, {  79, 1495 },
	{  80, 1456 }, {  81, 1424 }, {  82, 1392 }, {  83, 1358 },
	{  84, 1324 }, {  85, 1289 }, {  86, 1259 }, {  87, 1229 },
	{  88, 1199 }, {  89, 1167 }, {  90, 1135 }, {  91, 1109 },
	{  92, 1082 }, {  93, 1054 }, {  94, 1026 }, {  95,  997 },
	{  96,  973 }, {  97,  949 }, {  98,  924 }, {  99,  899 },
	{ 100,  874 }, { 101,  853 }, { 102,  831 }, { 103,  809 },
	{ 104,  787 }, { 105,  765 }, { 106,  746 }, { 107,  727 },
	{ 108,  707 }, { 109,  688 }, { 110,  668 }, { 111,  652 },
	{ 112,  635 }, { 113,  618 }, { 114,  601 }, { 115,  584 },
	{ 116,  569 }, { 117,  555 }, { 118,  540 }, { 119,  525 },
	{ 120,  510 }, { 121,  498 }, { 122,  485 }, { 123,  472 },
	{ 124,  459 }, { 125,  446 },
};

static struct ntc_thermistor_adc_table loki_tbat_thermistor_table[] = {
	{ -40, 3906 }, { -39, 3896 }, { -38, 3886 }, { -37, 3875 },
	{ -36, 3862 }, { -35, 3847 }, { -34, 3836 }, { -33, 3823 },
	{ -32, 3809 }, { -31, 3793 }, { -30, 3776 }, { -29, 3761 },
	{ -28, 3746 }, { -27, 3729 }, { -26, 3710 }, { -25, 3689 },
	{ -24, 3672 }, { -23, 3653 }, { -22, 3633 }, { -21, 3611 },
	{ -20, 3586 }, { -19, 3566 }, { -18, 3544 }, { -17, 3521 },
	{ -16, 3495 }, { -15, 3466 }, { -14, 3443 }, { -13, 3418 },
	{ -12, 3391 }, { -11, 3361 }, { -10, 3330 }, {  -9, 3303 },
	{  -8, 3275 }, {  -7, 3244 }, {  -6, 3212 }, {  -5, 3176 },
	{  -4, 3147 }, {  -3, 3115 }, {  -2, 3082 }, {  -1, 3046 },
	{   0, 3008 }, {   1, 2976 }, {   2, 2942 }, {   3, 2906 },
	{   4, 2868 }, {   5, 2827 }, {   6, 2793 }, {   7, 2757 },
	{   8, 2719 }, {   9, 2679 }, {  10, 2636 }, {  11, 2601 },
	{  12, 2564 }, {  13, 2525 }, {  14, 2484 }, {  15, 2440 },
	{  16, 2404 }, {  17, 2367 }, {  18, 2328 }, {  19, 2286 },
	{  20, 2243 }, {  21, 2207 }, {  22, 2170 }, {  23, 2131 },
	{  24, 2090 }, {  25, 2048 }, {  26, 2013 }, {  27, 1976 },
	{  28, 1939 }, {  29, 1899 }, {  30, 1859 }, {  31, 1825 },
	{  32, 1790 }, {  33, 1754 }, {  34, 1717 }, {  35, 1678 },
	{  36, 1646 }, {  37, 1613 }, {  38, 1579 }, {  39, 1544 },
	{  40, 1508 }, {  41, 1478 }, {  42, 1447 }, {  43, 1416 },
	{  44, 1383 }, {  45, 1350 }, {  46, 1322 }, {  47, 1294 },
	{  48, 1264 }, {  49, 1235 }, {  50, 1204 }, {  51, 1179 },
	{  52, 1153 }, {  53, 1126 }, {  54, 1099 }, {  55, 1072 },
	{  56, 1048 }, {  57, 1025 }, {  58, 1001 }, {  59,  976 },
	{  60,  952 }, {  61,  931 }, {  62,  909 }, {  63,  888 },
	{  64,  866 }, {  65,  844 }, {  66,  825 }, {  67,  806 },
	{  68,  787 }, {  69,  767 }, {  70,  748 }, {  71,  731 },
	{  72,  714 }, {  73,  697 }, {  74,  680 }, {  75,  662 },
	{  76,  647 }, {  77,  633 }, {  78,  617 }, {  79,  602 },
	{  80,  587 }, {  81,  573 }, {  82,  560 }, {  83,  546 },
	{  84,  533 }, {  85,  519 }, {  86,  507 }, {  87,  495 },
	{  88,  483 }, {  89,  471 }, {  90,  459 }, {  91,  448 },
	{  92,  437 }, {  93,  427 }, {  94,  416 }, {  95,  405 },
	{  96,  396 }, {  97,  386 }, {  98,  377 }, {  99,  367 },
	{ 100,  358 }, { 101,  349 }, { 102,  341 }, { 103,  333 },
	{ 104,  324 }, { 105,  316 }, { 106,  309 }, { 107,  301 },
	{ 108,  294 }, { 109,  287 }, { 110,  279 }, { 111,  273 },
	{ 112,  266 }, { 113,  260 }, { 114,  254 }, { 115,  247 },
	{ 116,  242 }, { 117,  236 }, { 118,  230 }, { 119,  225 },
	{ 120,  219 }, { 121,  214 }, { 122,  209 }, { 123,  204 },
	{ 124,  199 }, { 125,  195 },
};


static struct ntc_thermistor_adc_table *thermistor_table;
static int thermistor_table_size;

static int gadc_thermal_thermistor_adc_to_temp(
		struct gadc_thermal_platform_data *pdata, int val, int val2)
{
	int temp = 0, adc_hi, adc_lo;
	int i;

	for (i = 0; i < thermistor_table_size; i++)
		if (val >= thermistor_table[i].adc)
			break;

	if (i == 0) {
		temp = thermistor_table[i].temp * 1000;
	} else if (i >= (thermistor_table_size - 1)) {
		temp = thermistor_table[thermistor_table_size - 1].temp * 1000;
	} else {
		adc_hi = thermistor_table[i - 1].adc;
		adc_lo = thermistor_table[i].adc;
		temp = thermistor_table[i].temp * 1000;
		temp -= ((val - adc_lo) * 1000 / (adc_hi - adc_lo));
	}

	return temp;
};

static struct ntc_thermistor_adc_table *tbat_thermistor_table;
static int tbat_thermistor_table_size;

static int gadc_tbat_thermistor_adc_to_temp(
		struct gadc_thermal_platform_data *pdata, int val, int val2)
{
	int temp = 0, adc_hi, adc_lo;
	int i;

	for (i = 0; i < tbat_thermistor_table_size; i++)
		if (val >= tbat_thermistor_table[i].adc)
			break;

	if (i == 0) {
		temp = tbat_thermistor_table[i].temp * 1000;
	} else if (i >= (tbat_thermistor_table_size - 1)) {
		temp =
		tbat_thermistor_table[tbat_thermistor_table_size - 1].temp *
									1000;
	} else {
		adc_hi = tbat_thermistor_table[i - 1].adc;
		adc_lo = tbat_thermistor_table[i].adc;
		temp = tbat_thermistor_table[i].temp * 1000;
		temp -= ((val - adc_lo) * 1000 / (adc_hi - adc_lo));
	}

	return temp;
};

#define TDIODE_PRECISION_MULTIPLIER	1000000000LL
#define TDIODE_MIN_TEMP			-25000LL
#define TDIODE_MAX_TEMP			125000LL

static int gadc_thermal_tdiode_adc_to_temp(
		struct gadc_thermal_platform_data *pdata, int val, int val2)
{
	/*
	 * Series resistance cancellation using multi-current ADC measurement.
	 * diode temp = ((adc2 - k * adc1) - (b2 - k * b1)) / (m2 - k * m1)
	 * - adc1 : ADC raw with current source 400uA
	 * - m1, b1 : calculated with current source 400uA
	 * - adc2 : ADC raw with current source 800uA
	 * - m2, b2 : calculated with current source 800uA
	 * - k : 2 (= 800uA / 400uA)
	 */
	const s64 m1 = -0.00571005 * TDIODE_PRECISION_MULTIPLIER;
	const s64 b1 = 2524.29891 * TDIODE_PRECISION_MULTIPLIER;
	const s64 m2 = -0.005519811 * TDIODE_PRECISION_MULTIPLIER;
	const s64 b2 = 2579.354349 * TDIODE_PRECISION_MULTIPLIER;
	s64 temp = TDIODE_PRECISION_MULTIPLIER;

	temp *= (s64)((val2) - 2 * (val));
	temp -= (b2 - 2 * b1);
	temp = div64_s64(temp, (m2 - 2 * m1));
	temp = min_t(s64, max_t(s64, temp, TDIODE_MIN_TEMP), TDIODE_MAX_TEMP);
	return temp;
};

static struct gadc_thermal_platform_data gadc_thermal_thermistor_pdata = {
	.iio_channel_name = "thermistor",
	.tz_name = "Tboard",
	.temp_offset = 0,
	.adc_to_temp = gadc_thermal_thermistor_adc_to_temp,
};

static struct gadc_thermal_platform_data gadc_thermal_tdiode_pdata = {
	.iio_channel_name = "tdiode",
	.tz_name = "Tdiode",
	.temp_offset = 0,
	.dual_mode = true,
	.adc_to_temp = gadc_thermal_tdiode_adc_to_temp,
};

static struct gadc_thermal_platform_data gadc_thermal_tbat_pdata = {
	.iio_channel_name = "tbat",
	.tz_name = "battery-temp",
	.temp_offset = 0,
	.adc_to_temp = gadc_tbat_thermistor_adc_to_temp,
};

static struct platform_device gadc_thermal_thermistor = {
	.name   = "generic-adc-thermal",
	.id     = 0,
	.dev	= {
		.platform_data = &gadc_thermal_thermistor_pdata,
	},
};

static struct platform_device gadc_thermal_tdiode = {
	.name   = "generic-adc-thermal",
	.id     = 1,
	.dev	= {
		.platform_data = &gadc_thermal_tdiode_pdata,
	},
};

static struct platform_device gadc_tbat_thermistor = {
	.name   = "generic-adc-thermal",
	.id     = 2,
	.dev	= {
		.platform_data = &gadc_thermal_tbat_pdata,
	},
};

static struct platform_device *gadc_thermal_devices_p2530[] = {
	&gadc_thermal_thermistor,
	&gadc_thermal_tdiode,
	&gadc_tbat_thermistor,
};

static struct platform_device *gadc_thermal_devices_p2530_foster[] = {
	&gadc_thermal_thermistor,
	&gadc_thermal_tdiode,
};

int __init loki_sensors_init(void)
{
	tegra_get_board_info(&board_info);

	if (board_info.board_id == BOARD_P2530 && board_info.fab >= 0xa1) {
		thermistor_table = &loki_ffd_thermistor_table[0];
		thermistor_table_size = ARRAY_SIZE(loki_ffd_thermistor_table);
		if (board_info.sku == BOARD_SKU_FOSTER) {
			platform_add_devices(gadc_thermal_devices_p2530_foster,
				ARRAY_SIZE(gadc_thermal_devices_p2530_foster));
		} else if (board_info.sku == BOARD_SKU_100 ||
					board_info.sku == BOARD_SKU_0) {
			tbat_thermistor_table =
					&loki_tbat_thermistor_table[0];
			tbat_thermistor_table_size =
					ARRAY_SIZE(loki_tbat_thermistor_table);
			platform_add_devices(gadc_thermal_devices_p2530,
				ARRAY_SIZE(gadc_thermal_devices_p2530));

		} else {
			pr_err("thermal: Not a known sku\n");
		}
	}
	loki_fan_est_init();
	if (!(board_info.board_id == BOARD_P2530 &&
		board_info.sku == BOARD_SKU_FOSTER)) {
#ifndef CONFIG_USE_OF
		mpuirq_init();
#else
		mpu_dt_update();
#endif

		if (board_info.board_id != BOARD_E2549)
			loki_camera_init();
	}
	return 0;
}
