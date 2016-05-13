/*
 * arch/arm/mach-tegra/tegra11_soctherm.c
 *
 * Copyright (c) 2011-2014, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/debugfs.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <linux/seq_file.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/irqdomain.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/uaccess.h>
#include <linux/thermal.h>
#include <linux/platform_data/thermal_sensors.h>
#include <linux/pid_thermal_gov.h>
#include <linux/bug.h>
#include <linux/tegra-fuse.h>
#include <linux/tegra-pmc.h>
#include <linux/regulator/consumer.h>

#include "iomap.h"
#include "tegra11_soctherm.h"
#include "gpio-names.h"
#include <linux/platform/tegra/common.h>
#include "board-ardbeg.h"
#include <linux/platform/tegra/dvfs.h>

static const int MAX_HIGH_TEMP = 127000;
static const int MIN_LOW_TEMP = -127000;

/* Min temp granularity specified as X in 2^X.
 * -1: Hi precision option: 2^-1 = 0.5C (T12x onwards)
 *  0: Lo precision option: 2^0  = 1.0C
 */
#ifdef CONFIG_ARCH_TEGRA_12x_SOC
static const int precision = -1; /* Use high precision on T12x */
#else
static const int precision; /* default 0 -> low precision */
#endif

#define LOWER_PRECISION_FOR_CONV(val)	((!precision) ? ((val)*2) : (val))
#define LOWER_PRECISION_FOR_TEMP(val)	((!precision) ? ((val)/2) : (val))
#define PRECISION_IS_LOWER()		((!precision))
#define PRECISION_TO_STR()		((!precision) ? "Lo" : "Hi")

#define TS_TSENSE_REGS_SIZE		0x20
#define TS_TSENSE_REG_OFFSET(reg, ts)	((reg) + ((ts) * TS_TSENSE_REGS_SIZE))

#define TS_THERM_LVL_REGS_SIZE		0x20
#define TS_THERM_GRP_REGS_SIZE		0x04
#define TS_THERM_REG_OFFSET(rg, lv, gr)	((rg) + ((lv) * TS_THERM_LVL_REGS_SIZE)\
					+ ((gr) * TS_THERM_GRP_REGS_SIZE))

#define CTL_LVL0_CPU0			0x0
#define CTL_LVL0_CPU0_UP_THRESH_SHIFT	17
#define CTL_LVL0_CPU0_UP_THRESH_MASK	0xff
#define CTL_LVL0_CPU0_DN_THRESH_SHIFT	9
#define CTL_LVL0_CPU0_DN_THRESH_MASK	0xff
#define CTL_LVL0_CPU0_EN_SHIFT		8
#define CTL_LVL0_CPU0_EN_MASK		0x1
#define CTL_LVL0_CPU0_CPU_THROT_SHIFT	5
#define CTL_LVL0_CPU0_CPU_THROT_MASK	0x3
#define CTL_LVL0_CPU0_CPU_THROT_LIGHT	0x1
#define CTL_LVL0_CPU0_CPU_THROT_HEAVY	0x2
#define CTL_LVL0_CPU0_GPU_THROT_SHIFT	3
#define CTL_LVL0_CPU0_GPU_THROT_MASK	0x3
#define CTL_LVL0_CPU0_GPU_THROT_LIGHT	0x1
#define CTL_LVL0_CPU0_GPU_THROT_HEAVY	0x2
#define CTL_LVL0_CPU0_MEM_THROT_SHIFT	2
#define CTL_LVL0_CPU0_MEM_THROT_MASK	0x1
#define CTL_LVL0_CPU0_STATUS_SHIFT	0
#define CTL_LVL0_CPU0_STATUS_MASK	0x3

#define THERMTRIP			0x80
#define THERMTRIP_ANY_EN_SHIFT		28
#define THERMTRIP_ANY_EN_MASK		0x1
#define THERMTRIP_MEM_EN_SHIFT		27
#define THERMTRIP_MEM_EN_MASK		0x1
#define THERMTRIP_GPU_EN_SHIFT		26
#define THERMTRIP_GPU_EN_MASK		0x1
#define THERMTRIP_CPU_EN_SHIFT		25
#define THERMTRIP_CPU_EN_MASK		0x1
#define THERMTRIP_TSENSE_EN_SHIFT	24
#define THERMTRIP_TSENSE_EN_MASK	0x1
#define THERMTRIP_GPUMEM_THRESH_SHIFT	16
#define THERMTRIP_GPUMEM_THRESH_MASK	0xff
#define THERMTRIP_CPU_THRESH_SHIFT	8
#define THERMTRIP_CPU_THRESH_MASK	0xff
#define THERMTRIP_TSENSE_THRESH_SHIFT	0
#define THERMTRIP_TSENSE_THRESH_MASK	0xff

#define TS_CPU0_CONFIG0				0xc0
#define TS_CPU0_CONFIG0_TALL_SHIFT		8
#define TS_CPU0_CONFIG0_TALL_MASK		0xfffff
#define TS_CPU0_CONFIG0_TCALC_OVER_SHIFT	4
#define TS_CPU0_CONFIG0_TCALC_OVER_MASK		0x1
#define TS_CPU0_CONFIG0_OVER_SHIFT		3
#define TS_CPU0_CONFIG0_OVER_MASK		0x1
#define TS_CPU0_CONFIG0_CPTR_OVER_SHIFT		2
#define TS_CPU0_CONFIG0_CPTR_OVER_MASK		0x1
#define TS_CPU0_CONFIG0_STOP_SHIFT		0
#define TS_CPU0_CONFIG0_STOP_MASK		0x1

#define TS_CPU0_CONFIG1			0xc4
#define TS_CPU0_CONFIG1_EN_SHIFT	31
#define TS_CPU0_CONFIG1_EN_MASK		0x1
#define TS_CPU0_CONFIG1_TIDDQ_SHIFT	15
#define TS_CPU0_CONFIG1_TIDDQ_MASK	0x3f
#define TS_CPU0_CONFIG1_TEN_COUNT_SHIFT	24
#define TS_CPU0_CONFIG1_TEN_COUNT_MASK	0x3f
#define TS_CPU0_CONFIG1_TSAMPLE_SHIFT	0
#define TS_CPU0_CONFIG1_TSAMPLE_MASK	0x3ff

#define TS_CPU0_CONFIG2			0xc8
#define TS_CPU0_CONFIG2_THERM_A_SHIFT	16
#define TS_CPU0_CONFIG2_THERM_A_MASK	0xffff
#define TS_CPU0_CONFIG2_THERM_B_SHIFT	0
#define TS_CPU0_CONFIG2_THERM_B_MASK	0xffff

#define TS_CPU0_STATUS0			0xcc
#define TS_CPU0_STATUS0_VALID_SHIFT	31
#define TS_CPU0_STATUS0_VALID_MASK	0x1
#define TS_CPU0_STATUS0_CAPTURE_SHIFT	0
#define TS_CPU0_STATUS0_CAPTURE_MASK	0xffff

#define TS_CPU0_STATUS1				0xd0
#define TS_CPU0_STATUS1_TEMP_VALID_SHIFT	31
#define TS_CPU0_STATUS1_TEMP_VALID_MASK		0x1
#define TS_CPU0_STATUS1_TEMP_SHIFT		0
#define TS_CPU0_STATUS1_TEMP_MASK		0xffff

#define TS_CPU0_STATUS2			0xd4

#define TS_PDIV				0x1c0
#define TS_PDIV_CPU_SHIFT		12
#define TS_PDIV_CPU_MASK		0xf
#define TS_PDIV_GPU_SHIFT		8
#define TS_PDIV_GPU_MASK		0xf
#define TS_PDIV_MEM_SHIFT		4
#define TS_PDIV_MEM_MASK		0xf
#define TS_PDIV_PLLX_SHIFT		0
#define TS_PDIV_PLLX_MASK		0xf

#define TS_HOTSPOT_OFF			0x1c4
#define TS_HOTSPOT_OFF_CPU_SHIFT	16
#define TS_HOTSPOT_OFF_CPU_MASK		0xff
#define TS_HOTSPOT_OFF_GPU_SHIFT	8
#define TS_HOTSPOT_OFF_GPU_MASK		0xff
#define TS_HOTSPOT_OFF_MEM_SHIFT	0
#define TS_HOTSPOT_OFF_MEM_MASK		0xff

#define TS_TEMP1			0x1c8
#define TS_TEMP1_CPU_TEMP_SHIFT		16
#define TS_TEMP1_CPU_TEMP_MASK		0xffff
#define TS_TEMP1_GPU_TEMP_SHIFT		0
#define TS_TEMP1_GPU_TEMP_MASK		0xffff

#define TS_TEMP2			0x1cc
#define TS_TEMP2_MEM_TEMP_SHIFT		16
#define TS_TEMP2_MEM_TEMP_MASK		0xffff
#define TS_TEMP2_PLLX_TEMP_SHIFT	0
#define TS_TEMP2_PLLX_TEMP_MASK		0xffff

#define TS_TEMP_SW_OVERRIDE		0x1d8

#define TH_INTR_STATUS			0x84
#define TH_INTR_ENABLE			0x88
#define TH_INTR_DISABLE			0x8c

#define LOCK_CTL			0x90

#define TH_INTR_POS_MD3_SHIFT		31
#define TH_INTR_POS_MD3_MASK		0x1
#define TH_INTR_POS_MU3_SHIFT		30
#define TH_INTR_POS_MU3_MASK		0x1
#define TH_INTR_POS_MD2_SHIFT		29
#define TH_INTR_POS_MD2_MASK		0x1
#define TH_INTR_POS_MU2_SHIFT		28
#define TH_INTR_POS_MU2_MASK		0x1
#define TH_INTR_POS_MD1_SHIFT		27
#define TH_INTR_POS_MD1_MASK		0x1
#define TH_INTR_POS_MU1_SHIFT		26
#define TH_INTR_POS_MU1_MASK		0x1
#define TH_INTR_POS_MD0_SHIFT		25
#define TH_INTR_POS_MD0_MASK		0x1
#define TH_INTR_POS_MU0_SHIFT		24
#define TH_INTR_POS_MU0_MASK		0x1
#define TH_INTR_POS_GD3_SHIFT		23
#define TH_INTR_POS_GD3_MASK		0x1
#define TH_INTR_POS_GU3_SHIFT		22
#define TH_INTR_POS_GU3_MASK		0x1
#define TH_INTR_POS_GD2_SHIFT		21
#define TH_INTR_POS_GD2_MASK		0x1
#define TH_INTR_POS_GU2_SHIFT		20
#define TH_INTR_POS_GU2_MASK		0x1
#define TH_INTR_POS_GD1_SHIFT		19
#define TH_INTR_POS_GD1_MASK		0x1
#define TH_INTR_POS_GU1_SHIFT		18
#define TH_INTR_POS_GU1_MASK		0x1
#define TH_INTR_POS_GD0_SHIFT		17
#define TH_INTR_POS_GD0_MASK		0x1
#define TH_INTR_POS_GU0_SHIFT		16
#define TH_INTR_POS_GU0_MASK		0x1
#define TH_INTR_POS_CD3_SHIFT		15
#define TH_INTR_POS_CD3_MASK		0x1
#define TH_INTR_POS_CU3_SHIFT		14
#define TH_INTR_POS_CU3_MASK		0x1
#define TH_INTR_POS_CD2_SHIFT		13
#define TH_INTR_POS_CD2_MASK		0x1
#define TH_INTR_POS_CU2_SHIFT		12
#define TH_INTR_POS_CU2_MASK		0x1
#define TH_INTR_POS_CD1_SHIFT		11
#define TH_INTR_POS_CD1_MASK		0x1
#define TH_INTR_POS_CU1_SHIFT		10
#define TH_INTR_POS_CU1_MASK		0x1
#define TH_INTR_POS_CD0_SHIFT		9
#define TH_INTR_POS_CD0_MASK		0x1
#define TH_INTR_POS_CU0_SHIFT		8
#define TH_INTR_POS_CU0_MASK		0x1
#define TH_INTR_POS_PD3_SHIFT		7
#define TH_INTR_POS_PD3_MASK		0x1
#define TH_INTR_POS_PU3_SHIFT		6
#define TH_INTR_POS_PU3_MASK		0x1
#define TH_INTR_POS_PD2_SHIFT		5
#define TH_INTR_POS_PD2_MASK		0x1
#define TH_INTR_POS_PU2_SHIFT		4
#define TH_INTR_POS_PU2_MASK		0x1
#define TH_INTR_POS_PD1_SHIFT		3
#define TH_INTR_POS_PD1_MASK		0x1
#define TH_INTR_POS_PU1_SHIFT		2
#define TH_INTR_POS_PU1_MASK		0x1
#define TH_INTR_POS_PD0_SHIFT		1
#define TH_INTR_POS_PD0_MASK		0x1
#define TH_INTR_POS_PU0_SHIFT		0
#define TH_INTR_POS_PU0_MASK		0x1


#define UP_STATS_L0		0x10
#define DN_STATS_L0		0x14

#define STATS_CTL		0x94
#define STATS_CTL_CLR_DN	0x8
#define STATS_CTL_EN_DN		0x4
#define STATS_CTL_CLR_UP	0x2
#define STATS_CTL_EN_UP		0x1

#define THROT_GLOBAL_CFG	0x400
#define THROT13_GLOBAL_CFG	0x148
#define THROT_GLOBAL_ENB_SHIFT	0
#define THROT_GLOBAL_ENB_MASK	0x1

#define OC1_CFG				0x310
#define OC1_CFG_LONG_LATENCY_SHIFT	6
#define OC1_CFG_LONG_LATENCY_MASK	0x1
#define OC1_CFG_HW_RESTORE_SHIFT	5
#define OC1_CFG_HW_RESTORE_MASK		0x1
#define OC1_CFG_PWR_GOOD_MASK_SHIFT	4
#define OC1_CFG_PWR_GOOD_MASK_MASK	0x1
#define OC1_CFG_THROTTLE_MODE_SHIFT	2
#define OC1_CFG_THROTTLE_MODE_MASK	0x3
#define OC1_CFG_ALARM_POLARITY_SHIFT	1
#define OC1_CFG_ALARM_POLARITY_MASK	0x1
#define OC1_CFG_EN_THROTTLE_SHIFT	0
#define OC1_CFG_EN_THROTTLE_MASK	0x1

#define OC1_CNT_THRESHOLD		0x314
#define OC1_THROTTLE_PERIOD		0x318
#define OC1_ALARM_COUNT			0x31c
#define OC1_FILTER			0x320

#define OC1_STATS			0x3a8

#define OC_INTR_STATUS			0x39c
#define OC_INTR_ENABLE			0x3a0
#define OC_INTR_DISABLE			0x3a4
#define OC_INTR_POS_OC1_SHIFT		0
#define OC_INTR_POS_OC1_MASK		0x1
#define OC_INTR_POS_OC2_SHIFT		1
#define OC_INTR_POS_OC2_MASK		0x1
#define OC_INTR_POS_OC3_SHIFT		2
#define OC_INTR_POS_OC3_MASK		0x1
#define OC_INTR_POS_OC4_SHIFT		3
#define OC_INTR_POS_OC4_MASK		0x1
#define OC_INTR_POS_OC5_SHIFT		4
#define OC_INTR_POS_OC5_MASK		0x1

#define OC_STATS_CTL			0x3c4
#define OC_STATS_CTL_CLR_ALL		0x2
#define OC_STATS_CTL_EN_ALL		0x1

#define CPU_PSKIP_STATUS			0x418
#define GPU_PSKIP_STATUS			0x41c
#define XPU_PSKIP_STATUS_M_SHIFT		12
#define XPU_PSKIP_STATUS_M_MASK			0xff
#define XPU_PSKIP_STATUS_N_SHIFT		4
#define XPU_PSKIP_STATUS_N_MASK			0xff
#define XPU_PSKIP_STATUS_SW_OVERRIDE_SHIFT	1
#define XPU_PSKIP_STATUS_SW_OVERRIDE_MASK	0x1
#define XPU_PSKIP_STATUS_ENABLED_SHIFT		0
#define XPU_PSKIP_STATUS_ENABLED_MASK		0x1

#define THROT_PRIORITY_LOCK			0x424
#define THROT_PRIORITY_LOCK_PRIORITY_SHIFT	0
#define THROT_PRIORITY_LOCK_PRIORITY_MASK	0xff

#define THROT_STATUS				0x428
#define THROT_STATUS_BREACH_SHIFT		12
#define THROT_STATUS_BREACH_MASK		0x1
#define THROT_STATUS_STATE_SHIFT		4
#define THROT_STATUS_STATE_MASK			0xff
#define THROT_STATUS_ENABLED_SHIFT		0
#define THROT_STATUS_ENABLED_MASK		0x1

#define THROT_PSKIP_CTRL_LITE_CPU		0x430
#define THROT13_PSKIP_CTRL_LOW_CPU		0x154
#define THROT_PSKIP_CTRL_ENABLE_SHIFT		31
#define THROT_PSKIP_CTRL_ENABLE_MASK		0x1
#define THROT_PSKIP_CTRL_VECT_GPU_SHIFT		16
#define THROT_PSKIP_CTRL_VECT_GPU_MASK		0x7
#define THROT_PSKIP_CTRL_VECT_CPU_SHIFT		8
#define THROT_PSKIP_CTRL_VECT_CPU_MASK		0x7
#define THROT_PSKIP_CTRL_DIVIDEND_SHIFT		8
#define THROT_PSKIP_CTRL_DIVIDEND_MASK		0xff
#define THROT_PSKIP_CTRL_DIVISOR_SHIFT		0
#define THROT_PSKIP_CTRL_DIVISOR_MASK		0xff
#define THROT_PSKIP_CTRL_VECT2_CPU_SHIFT	0
#define THROT_PSKIP_CTRL_VECT2_CPU_MASK		0x7

#define THROT_PSKIP_RAMP_LITE_CPU		0x434
#define THROT13_PSKIP_RAMP_LOW_CPU		0x150
#define THROT_PSKIP_RAMP_SEQ_BYPASS_MODE_SHIFT	31
#define THROT_PSKIP_RAMP_SEQ_BYPASS_MODE_MASK	0x1
#define THROT_PSKIP_RAMP_DURATION_SHIFT		8
#define THROT_PSKIP_RAMP_DURATION_MASK		0xffff
#define THROT_PSKIP_RAMP_STEP_SHIFT		0
#define THROT_PSKIP_RAMP_STEP_MASK		0xff

#define THROT_VECT_NONE				0x0 /* 3'b000 */
#define THROT_VECT_LOW				0x1 /* 3'b001 */
#define THROT_VECT_MED				0x3 /* 3'b011 */
#define THROT_VECT_HVY				0x7 /* 3'b111 */

#define THROT_LEVEL_LOW				0
#define THROT_LEVEL_MED				1
#define THROT_LEVEL_HVY				2
#define THROT_LEVEL_NONE			-1 /* invalid */

#define THROT_PRIORITY_LITE			0x444
#define THROT_PRIORITY_LITE_PRIO_SHIFT		0
#define THROT_PRIORITY_LITE_PRIO_MASK		0xff

#define THROT_DELAY_LITE			0x448
#define THROT_DELAY_LITE_DELAY_SHIFT		0
#define THROT_DELAY_LITE_DELAY_MASK		0xff

#define THROT_OFFSET				0x30
#define THROT13_OFFSET				0x0c
#define ALARM_OFFSET				0x14

#define FUSE_TSENSOR_CALIB_FT_SHIFT	13
#define FUSE_TSENSOR_CALIB_FT_MASK	0x1fff
#define FUSE_TSENSOR_CALIB_CP_SHIFT	0
#define FUSE_TSENSOR_CALIB_CP_MASK	0x1fff
#define FUSE_TSENSOR_CALIB_BITS		13

/* car register offsets needed for enabling HW throttling */
#define CAR_SUPER_CCLKG_DIVIDER		0x36c
#define CAR13_SUPER_CCLKG_DIVIDER	0x024
#define CDIVG_ENABLE_SHIFT		31
#define CDIVG_ENABLE_MASK		0x1
#define CDIVG_USE_THERM_CONTROLS_SHIFT	30
#define CDIVG_USE_THERM_CONTROLS_MASK	0x1
#define CDIVG_DIVIDEND_MASK		0xff
#define CDIVG_DIVIDEND_SHIFT		8
#define CDIVG_DIVISOR_MASK		0xff
#define CDIVG_DIVISOR_SHIFT		0

#define CAR_SUPER_CLK_DIVIDER_REGISTER()	(IS_T13X ? \
						 CAR13_SUPER_CCLKG_DIVIDER : \
						 CAR_SUPER_CCLKG_DIVIDER)
#define THROT_PSKIP_CTRL(throt, dev)		(THROT_PSKIP_CTRL_LITE_CPU + \
						(THROT_OFFSET * throt) + \
						(8 * dev))
#define THROT_PSKIP_RAMP(throt, dev)		(THROT_PSKIP_RAMP_LITE_CPU + \
						(THROT_OFFSET * throt) + \
						(8 * dev))
#define THROT13_PSKIP_CTRL_CPU(vect)		(THROT13_PSKIP_CTRL_LOW_CPU + \
						 (THROT13_OFFSET * vect))
#define THROT13_PSKIP_RAMP_CPU(vect)		(THROT13_PSKIP_RAMP_LOW_CPU + \
						 (THROT13_OFFSET * vect))
#define THROT_PRIORITY_CTRL(throt)		(THROT_PRIORITY_LITE + \
						(THROT_OFFSET * throt))
#define THROT_DELAY_CTRL(throt)			(THROT_DELAY_LITE + \
						(THROT_OFFSET * throt))
#define ALARM_CFG(throt)			(OC1_CFG + \
						(ALARM_OFFSET * (throt - \
								THROTTLE_OC1)))
#define ALARM_CNT_THRESHOLD(throt)		(OC1_CNT_THRESHOLD + \
						(ALARM_OFFSET * (throt - \
								THROTTLE_OC1)))
#define ALARM_THROTTLE_PERIOD(throt)		(OC1_THROTTLE_PERIOD + \
						(ALARM_OFFSET * (throt - \
								THROTTLE_OC1)))
#define ALARM_ALARM_COUNT(throt)		(OC1_ALARM_COUNT + \
						(ALARM_OFFSET * (throt - \
								THROTTLE_OC1)))
#define ALARM_FILTER(throt)			(OC1_FILTER + \
						(ALARM_OFFSET * (throt - \
								THROTTLE_OC1)))
#define ALARM_STATS(throt)			(OC1_STATS + \
						(4 * (throt - THROTTLE_OC1)))

#define THROT_DEPTH_DIVIDEND(depth)	((256 * (100 - (depth)) / 100) - 1)
#define THROT_DEPTH(th, dp)		{			\
		(th)->depth    = (dp);				\
		(th)->dividend = THROT_DEPTH_DIVIDEND(dp);	\
		(th)->divisor  = 255;				\
		(th)->duration = 0xff;				\
		(th)->step     = 0xf;				\
	}

#define REG_SET(r, _name, val)	(((r) & ~(_name##_MASK << _name##_SHIFT)) | \
				 (((val) & _name##_MASK) << _name##_SHIFT))
#define REG_GET_BIT(r, _name)	((r) & (_name##_MASK << _name##_SHIFT))
#define REG_GET(r, _name)	(REG_GET_BIT(r, _name) >> _name##_SHIFT)
#define MAKE_SIGNED32(val, nb)	((s32)(val) << (32 - (nb)) >> (32 - (nb)))

#define IS_T11X		(tegra_chip_id == TEGRA_CHIPID_TEGRA11)
#define IS_T14X		(tegra_chip_id == TEGRA_CHIPID_TEGRA14)
#define IS_T12X		(tegra_chip_id == TEGRA_CHIPID_TEGRA12)
#define IS_T13X		(tegra_chip_id == TEGRA_CHIPID_TEGRA13)

static void __iomem *reg_soctherm_base = IOMEM(IO_ADDRESS(TEGRA_SOCTHERM_BASE));
static void __iomem *clk_reset_base = IOMEM(IO_ADDRESS(TEGRA_CLK_RESET_BASE));
static void __iomem *clk13_rst_base = IOMEM(IO_ADDRESS(TEGRA_CLK13_RESET_BASE));

static DEFINE_MUTEX(soctherm_suspend_resume_lock);

static int soctherm_suspend(void);
static int soctherm_resume(void);

static struct soctherm_platform_data plat_data;

/*
 * Remove this flag once this "driver" is structured as a platform driver and
 * the board files calls platform_device_register instead of directly calling
 * tegra11_soctherm_init(). See nvbug 1206311.
 */
static bool soctherm_init_platform_done;
static bool read_hw_temp = true;
static bool soctherm_suspended;
static bool vdd_cpu_low_voltage;
static bool vdd_core_low_voltage;
static u32 tegra_chip_id;

static struct clk *soctherm_clk;
static struct clk *tsensor_clk;

/**
 * soctherm_writel() - Writes a value to a SOC_THERM register
 * @value:		The value to write
 * @reg:		The register offset
 *
 * Writes the @value to @reg if the soctherm device is not suspended.
 */
static inline void soctherm_writel(u32 value, u32 reg)
{
	if (!soctherm_suspended)
		__raw_writel(value, (void __iomem *)
			(reg_soctherm_base + reg));
}

/**
 * soctherm_readl() - reads specified register from SOC_THERM IP block
 * @reg:	register address to be read
 *
 * Return: 0 if SOC_THERM is suspended, else the value of the register
 */
static inline u32 soctherm_readl(u32 reg)
{
	if (soctherm_suspended)
		return 0;
	return __raw_readl(reg_soctherm_base + reg);
}

/* XXX Temporary until CCROC accesses are split out */
static void clk_reset13_writel(u32 value, u32 reg)
{
	BUG_ON(!IS_T13X);
	__raw_writel(value, clk13_rst_base + reg);
	__raw_readl(clk13_rst_base + reg);
}

/* XXX Temporary until CCROC accesses are split out */
static u32 clk_reset13_readl(u32 reg)
{
	BUG_ON(!IS_T13X);
	return __raw_readl(clk13_rst_base + reg);
}

static inline void clk_reset_writel(u32 value, u32 reg)
{
	if (IS_T13X) {
		__raw_writel(value, clk13_rst_base + reg);
		__raw_readl(clk13_rst_base + reg);
	} else
		__raw_writel(value, clk_reset_base + reg);
}

static inline u32 clk_reset_readl(u32 reg)
{
	if (IS_T13X)
		return __raw_readl(clk13_rst_base + reg);
	else
		return __raw_readl(clk_reset_base + reg);
}

/**
 * temp_convert() - convert raw sensor readings to temperature
 * @cap:	raw TSOSC count
 * @a:		slope of count/temperature linear regression
 * @b:		x-intercept of count/temperature linear regression
 *
 * This is a software version of what happens in the hardware when
 * temp_translate() is called. However, when the hardware does the conversion,
 * it cannot do it with the same precision that can be done with software.
 *
 * This function is not in use as long as @read_hw_temp is set to true, however
 * software temperature conversion could be used to monitor temperatures with a
 * higher degree of precision as they near a temperature threshold.
 *
 * Return: temperature in millicelsius.
 */
static inline long temp_convert(int cap, int a, int b)
{
	cap *= a;
	cap >>= 10;
	cap += (b << 3);
	cap *= LOWER_PRECISION_FOR_CONV(500);
	cap /= 8;
	return cap;
}

/**
 * temp_translate_rev() - Translates the given temperature from two's
 * complement to the signed magnitude form used in SOC_THERM registers
 * @temp:	The temperature to be translated
 *
 * The register value returned will have the following bit assignment:
 * 15:7 magnitude of temperature in (1/2 or 1 degree precision) centigrade
 * 0 the sign bit of the temperature
 *
 * This function is the inverse of the temp_translate() function
 *
 * Return: The register value.
 */
static inline u32 temp_translate_rev(long temp)
{
	int sign;
	int low_bit;

	u32 lsb = 0;
	u32 abs = 0;
	u32 reg = 0;
	sign = (temp > 0 ? 1 : -1);
	low_bit = (sign > 0 ? 0 : 1);
	temp *= sign;
	/* high precision only */
	if (!PRECISION_IS_LOWER()) {
		lsb = ((temp % 1000) > 0) ? 1 : 0;
		abs = (temp - 500 * lsb) / 1000;
		abs &= 0xff;
		reg = ((abs << 8) | (lsb << 7) | low_bit);
	}
	return reg;
}

#ifdef CONFIG_THERMAL
static struct thermal_zone_device *soctherm_th_zones[THERM_SIZE];
#endif
struct soctherm_oc_irq_chip_data {
	int			irq_base;
	struct mutex		irq_lock; /* serialize OC IRQs */
	struct irq_chip		irq_chip;
	struct irq_domain	*domain;
	int			irq_enable;
};
static struct soctherm_oc_irq_chip_data soc_irq_cdata;

static u32 fuse_calib_base_cp;
static u32 fuse_calib_base_ft;
static s32 actual_temp_cp;
static s32 actual_temp_ft;

static const char *const therm_names[] = {
	[THERM_CPU] = "CPU",
	[THERM_MEM] = "MEM",
	[THERM_GPU] = "GPU",
	[THERM_PLL] = "PLL",
};

static const char *const throt_names[] = {
	[THROTTLE_LIGHT]   = "light",
	[THROTTLE_HEAVY]   = "heavy",
	[THROTTLE_OC1]     = "oc1",
	[THROTTLE_OC2]     = "oc2",
	[THROTTLE_OC3]     = "oc3",
	[THROTTLE_OC4]     = "oc4",
	[THROTTLE_OC5]     = "oc5", /* reserved */
};

static const char *const throt_dev_names[] = {
	[THROTTLE_DEV_CPU] = "CPU",
	[THROTTLE_DEV_GPU] = "GPU",
};

static const char *const sensor_names[] = {
	[TSENSE_CPU0] = "cpu0",
	[TSENSE_CPU1] = "cpu1",
	[TSENSE_CPU2] = "cpu2",
	[TSENSE_CPU3] = "cpu3",
	[TSENSE_MEM0] = "mem0",
	[TSENSE_MEM1] = "mem1",
	[TSENSE_GPU]  = "gpu0",
	[TSENSE_PLLX] = "pllx",
};

static const int sensor2tsensorcalib[] = {
	[TSENSE_CPU0] = 0,
	[TSENSE_CPU1] = 1,
	[TSENSE_CPU2] = 2,
	[TSENSE_CPU3] = 3,
	[TSENSE_MEM0] = 5,
	[TSENSE_MEM1] = 6,
	[TSENSE_GPU]  = 4,
	[TSENSE_PLLX] = 7,
};

static const int tsensor2therm_map[] = {
	[TSENSE_CPU0] = THERM_CPU,
	[TSENSE_CPU1] = THERM_CPU,
	[TSENSE_CPU2] = THERM_CPU,
	[TSENSE_CPU3] = THERM_CPU,
	[TSENSE_GPU]  = THERM_GPU,
	[TSENSE_MEM0] = THERM_MEM,
	[TSENSE_MEM1] = THERM_MEM,
	[TSENSE_PLLX] = THERM_PLL,
};

static const enum soctherm_throttle_dev_id therm2dev[] = {
	[THERM_CPU] = THROTTLE_DEV_CPU,
	[THERM_MEM] = THROTTLE_DEV_NONE,
	[THERM_GPU] = THROTTLE_DEV_GPU,
	[THERM_PLL] = THROTTLE_DEV_NONE,
};

static const struct soctherm_sensor default_t11x_sensor_params = {
	.tall      = 16300,
	.tiddq     = 1,
	.ten_count = 1,
	.tsample   = 163,
	.tsamp_ate = 655,
	.pdiv      = 10,
	.pdiv_ate  = 10,
};
static const struct soctherm_sensor default_t14x_sensor_params = {
	.tall      = 16300,
	.tiddq     = 1,
	.ten_count = 1,
	.tsample   = 120,
	.tsamp_ate = 481,
	.pdiv      = 8,
	.pdiv_ate  = 8,
};

/* Used for T124 and T132 */
static const struct soctherm_sensor default_t12x_sensor_params = {
	.tall      = 16300,
	.tiddq     = 1,
	.ten_count = 1,
	.tsample   = 120,
	.tsamp_ate = 480,
	.pdiv      = 8,
	.pdiv_ate  = 8,
};

static const unsigned long default_t11x_soctherm_clk_rate = 51000000;
static const unsigned long default_t11x_tsensor_clk_rate = 500000;
static const unsigned long default_t14x_soctherm_clk_rate = 51000000;
static const unsigned long default_t14x_tsensor_clk_rate = 400000;
/* TODO : finalize the default clk rate */
static const unsigned long default_t12x_soctherm_clk_rate = 51000000;
static const unsigned long default_t12x_tsensor_clk_rate = 400000;

/* SOC- OCx to theirt GPIO which is wakeup capable. This is T114 specific */
static int soctherm_ocx_to_wake_gpio[TEGRA_SOC_OC_IRQ_MAX] = {
	TEGRA_GPIO_PEE3,	/* TEGRA_SOC_OC_IRQ_1 */
	TEGRA_GPIO_INVALID,	/* TEGRA_SOC_OC_IRQ_2 */
	TEGRA_GPIO_INVALID,	/* TEGRA_SOC_OC_IRQ_3 */
	TEGRA_GPIO_PJ2,		/* TEGRA_SOC_OC_IRQ_4 */
	TEGRA_GPIO_INVALID,	/* TEGRA_SOC_OC_IRQ_5 */
};

static int sensor2therm_a[TSENSE_SIZE];
static int sensor2therm_b[TSENSE_SIZE];

/**
 * div64_s64_precise() - wrapper for div64_s64()
 * @a:	the dividend
 * @b:	the divisor
 *
 * Implements division with fairly accurate rounding instead of truncation by
 * shifting the dividend to the left by 16 so that the quotient has a
 * much higher precision.
 *
 * Return: the quotient of a / b.
 */

static inline s64 div64_s64_precise(s64 a, s32 b)
{
	s64 r, al;

	/* scale up for increased precision in division */
	al = a << 16;

	r = div64_s64((al * 2) + 1, 2 * b);
	return r >> 16;
}

/**
 * temp_translate() - Converts temperature
 * @readback:		The value from a SOC_THERM sensor temperature
 *			register
 *
 * Converts temperature from the format used in registers to a (signed)
 * long. This function is the inverse of temp_translate_rev().
 *
 * Return: the translated temperature in millicelsius
 */
static inline long temp_translate(int readback)
{
	int abs = readback >> 8;
	int lsb = (readback & 0x80) >> 7;
	int sign = readback & 0x01 ? -1 : 1;

	return (abs * LOWER_PRECISION_FOR_CONV(1000) +
		lsb * LOWER_PRECISION_FOR_CONV(500)) * sign;
}

#ifdef CONFIG_THERMAL

/**
 * soctherm_has_mn_cpu_pskip_status() - does CPU use M,N values for pskip status?
 *
 * If the currently-running SoC reports the CPU thermal throttling
 * pulse skipper status with (M, N) values via SOC_THERM registers,
 * then return true; otherwise, return false.  XXX Temporary - should
 * be replaced by autodetection or DT properties/compatible flags.
 *
 * Return: true if CPU thermal pulse-skipper M,N status values are available via
 * SOC_THERM, or false if not.
 */
static int soctherm_has_mn_cpu_pskip_status(void)
{
	return IS_T11X || IS_T14X || IS_T12X;
}

/**
 * soctherm_get_mn_cpu_pskip_status() - read state of CPU thermal pulse skipper
 * @enabled: pointer to a u8: return 0 if the skipper is disabled, 1 if enabled
 * @sw_override: ptr to a u8: return 0 if sw override is disabled, 1 if enabled
 * @m: pointer to a u16 to return the current pulse skipper ratio numerator into
 * @n: pointer to a u16 to return the current pulse skipper ratio denominator to
 *
 * Read the current status of the thermal throttling pulse skipper
 * attached to the CPU clock, and return the status into the variables
 * pointed to by @enabled, @sw_override, @m, and @n.  The M and N
 * values are not what is stored in the register bitfields, but
 * instead are the actual values used by the pulse skipper -- i.e.,
 * they are the bitfield values _plus one_; they have valid ranges of
 * 1-256.  This function is only defined for chips that report M,N
 * thermal throttling states
 *
 * Return: 0 upon success, -ENOTSUPP if called on a chip that uses
 * CPU-local (i.e., non-SOC_THERM) pulse-skipper status, or -EINVAL if
 * any of the arguments are NULL.
 */
int soctherm_get_mn_cpu_pskip_status(u8 *enabled, u8 *sw_override, u16 *m,
				     u16 *n)
{
	u32 v;

	if (!enabled || !m || !n || !sw_override)
		return -EINVAL;

	/*
	 * XXX should be replaced with an earlier DT property read to
	 * determine the GPU type (or GPU->SOC_THERM integration) in
	 * use
	 */
	if (!soctherm_has_mn_cpu_pskip_status())
		return -ENOTSUPP;

	v = soctherm_readl(CPU_PSKIP_STATUS);
	if (REG_GET(v, XPU_PSKIP_STATUS_ENABLED)) {
		*enabled = 1;
		*sw_override = REG_GET(v, XPU_PSKIP_STATUS_SW_OVERRIDE);
		*m = REG_GET(v, XPU_PSKIP_STATUS_M) + 1;
		*n = REG_GET(v, XPU_PSKIP_STATUS_N) + 1;
	} else {
		*enabled = 0;
	}

	return 0;
}

/**
 * soctherm_has_gpu_pskip_status() - is GPU pskip state readable via SOC_THERM?
 *
 * If the currently-running SoC reports the GPU thermal throttling
 * pulse skipper status via SOC_THERM registers, then return true;
 * otherwise, return false.  XXX Temporary - should be replaced by
 * autodetection or DT properties/compatible flags.
 *
 * Return: true if GPU thermal pulse-skipper status is readable via
 * SOC_THERM, or false if not.
 */
static int soctherm_has_gpu_pskip_status(void)
{
	return IS_T11X || IS_T14X;
}

/**
 * soctherm_get_gpu_pskip_status() - read state of the GPU thermal pulse skipper
 * @enabled: pointer to a u8: return 0 if the skipper is disabled, 1 if enabled
 * @sw_override: ptr to a u8: return 0 if sw override is disabled, 1 if enabled
 * @m: pointer to a u8 to return the current pulse skipper ratio numerator into
 * @n: pointer to a u8 to return the current pulse skipper ratio denominator to
 *
 * Read the current status of the thermal throttling pulse skipper
 * attached to the GPU clock, and return the status into the variables
 * pointed to by @enabled, @sw_override, @m, and @n.  Note that the M
 * and N values are not what is stored in the register bitfields, but
 * instead are the actual values used by the pulse skipper -- i.e., they
 * are the bitfield values _plus one_; they have valid ranges of 1-256.
 *
 * Return: 0 upon success, -ENOTSUPP on chips with GPU-local
 * throttling status (e.g., T124, T132) or -EINVAL if any of the
 * arguments are NULL.
 */
int soctherm_get_gpu_pskip_status(u8 *enabled, u8 *sw_override, u16 *m, u16 *n)
{
	u32 v;

	if (!enabled || !m || !n || !sw_override)
		return -EINVAL;

	/*
	 * XXX should be replaced with an earlier DT property read to
	 * determine the GPU type (or GPU->SOC_THERM integration) in
	 * use
	 */
	if (!soctherm_has_gpu_pskip_status())
		return -ENOTSUPP;

	v = soctherm_readl(GPU_PSKIP_STATUS);
	if (REG_GET(v, XPU_PSKIP_STATUS_ENABLED)) {
		*enabled = 1;
		*sw_override = REG_GET(v, XPU_PSKIP_STATUS_SW_OVERRIDE);
		*m = REG_GET(v, XPU_PSKIP_STATUS_M) + 1;
		*n = REG_GET(v, XPU_PSKIP_STATUS_N) + 1;
	} else {
		*enabled = 0;
	}

	return 0;
}

/**
 * enforce_temp_range() - check and enforce temperature range [min, max]
 * @trip_temp:		The trip temperature to check
 *
 * Checks and enforces the permitted temperature range that SOC_THERM
 * HW can support with 8-bit registers to specify temperature. This is
 * done while taking care of precision.
 *
 * Return: The precsion adjusted capped temperature in millicelsius.
 */
static int enforce_temp_range(long trip_temp)
{
	long temp = LOWER_PRECISION_FOR_TEMP(trip_temp);

	if (temp < MIN_LOW_TEMP) {
		pr_info("soctherm: trip_point temp %ld forced to %d\n",
			trip_temp, LOWER_PRECISION_FOR_CONV(MIN_LOW_TEMP));
		temp = MIN_LOW_TEMP;
	} else if (temp > MAX_HIGH_TEMP) {
		pr_info("soctherm: trip_point temp %ld forced to %d\n",
			trip_temp, LOWER_PRECISION_FOR_CONV(MAX_HIGH_TEMP));
		temp = MAX_HIGH_TEMP;
	}

	return temp;
}

/**
 * prog_hw_shutdown() - Configures the hardware to shut down the
 * system if a given sensor group reaches a given temperature
 * @trip_state:		The trip information. Includes the temperature
 *			at which a trip occurs.
 * @therm:		Int specifying the sensor group.
 *			Should be one of the following:
 *			THERM_CPU, THERM_GPU,
 *			THERM_MEM, or THERM_PPL.
 *
 * Sets the thermal trip threshold of the given sensor group
 * to be the trip temperature of @trip_state.
 * If this threshold is crossed, the hardware will shut down.
 *
 * Return: No return value (void).
 */
static inline void prog_hw_shutdown(struct thermal_trip_info *trip_state,
				    int therm)
{
	u32 r;
	int temp;

	/* Add 1'C to HW shutdown threshold so SW can try to shutdown first */
	temp = trip_state->trip_temp + LOWER_PRECISION_FOR_CONV(1000);

	temp = enforce_temp_range(temp) / 1000;

	r = soctherm_readl(THERMTRIP);
	if (therm == THERM_CPU) {
		r = REG_SET(r, THERMTRIP_CPU_EN, 1);
		r = REG_SET(r, THERMTRIP_CPU_THRESH, temp);
	} else if (therm == THERM_GPU) {
		r = REG_SET(r, THERMTRIP_GPU_EN, 1);
		r = REG_SET(r, THERMTRIP_GPUMEM_THRESH, temp);
	} else if (therm == THERM_PLL) {
		r = REG_SET(r, THERMTRIP_TSENSE_EN, 1);
		r = REG_SET(r, THERMTRIP_TSENSE_THRESH, temp);
	} else if (therm == THERM_MEM) {
		r = REG_SET(r, THERMTRIP_MEM_EN, 1);
		r = REG_SET(r, THERMTRIP_GPUMEM_THRESH, temp);
	}
	r = REG_SET(r, THERMTRIP_ANY_EN, 0);
	soctherm_writel(r, THERMTRIP);
}

/**
 * prog_hw_threshold() - updates hardware temperature threshold
 *	of a particular trip point
 * @trip_state:	setting of a trip point to use to update hardware threshold
 * @therm:	soctherm_therm_id specifying the sensor group to update
 * @throt:	soctherm_throttle_id indicating throttling level to update
 *
 * Configure sensor group @therm to engage a hardware throttling response at
 * the threshold indicated by @trip_state.
 */
static inline void prog_hw_threshold(struct thermal_trip_info *trip_state,
				     int therm, int throt)
{
	u32 r, reg_off;
	int temp;
	int cpu_throt, gpu_throt;

	temp = enforce_temp_range(trip_state->trip_temp) / 1000;

	/* Hardcode LITE on level-1 and HEAVY on level-2 */
	reg_off = TS_THERM_REG_OFFSET(CTL_LVL0_CPU0, throt + 1, therm);

	if (throt == THROTTLE_LIGHT) {
		cpu_throt = CTL_LVL0_CPU0_CPU_THROT_LIGHT;
		gpu_throt = CTL_LVL0_CPU0_GPU_THROT_LIGHT;
	} else {
		cpu_throt = CTL_LVL0_CPU0_CPU_THROT_HEAVY;
		gpu_throt = CTL_LVL0_CPU0_GPU_THROT_HEAVY;
		if (throt != THROTTLE_HEAVY)
			pr_warn("soctherm: invalid throt %d - assuming HEAVY",
				throt);
	}

	r = soctherm_readl(reg_off);
	r = REG_SET(r, CTL_LVL0_CPU0_UP_THRESH, temp);
	r = REG_SET(r, CTL_LVL0_CPU0_DN_THRESH, temp);
	r = REG_SET(r, CTL_LVL0_CPU0_CPU_THROT, cpu_throt);
	r = REG_SET(r, CTL_LVL0_CPU0_GPU_THROT, gpu_throt);
	r = REG_SET(r, CTL_LVL0_CPU0_EN, 1);
	soctherm_writel(r, reg_off);
}

/**
 * soctherm_set_limits() - Configures a sensor group to raise interrupts outside
 * the given temperature range
 * @therm:		ID of the sensor group
 * @lo_limit:		The lowest temperature limit
 * @hi_limit:		The highest temperature limit
 *
 * Configures sensor group @therm to raise an interrupt when temperature goes
 * above @hi_limit or below @lo_limit.
 */
static void soctherm_set_limits(enum soctherm_therm_id therm,
				long lo_limit, long hi_limit)
{
	u32 r, reg_off;
	int rlo_limit, rhi_limit;

	rlo_limit = LOWER_PRECISION_FOR_TEMP(lo_limit) / 1000;
	rhi_limit = LOWER_PRECISION_FOR_TEMP(hi_limit) / 1000;

	reg_off = TS_THERM_REG_OFFSET(CTL_LVL0_CPU0, 0, therm);

	r = soctherm_readl(reg_off);
	r = REG_SET(r, CTL_LVL0_CPU0_DN_THRESH, rlo_limit);
	r = REG_SET(r, CTL_LVL0_CPU0_UP_THRESH, rhi_limit);
	r = REG_SET(r, CTL_LVL0_CPU0_EN, 1);
	soctherm_writel(r, reg_off);

	switch (therm) {
	case THERM_CPU:
		r = REG_SET(0, TH_INTR_POS_CD0, 1);
		r = REG_SET(r, TH_INTR_POS_CU0, 1);
		break;
	case THERM_GPU:
		r = REG_SET(0, TH_INTR_POS_GD0, 1);
		r = REG_SET(r, TH_INTR_POS_GU0, 1);
		break;
	case THERM_PLL:
		r = REG_SET(0, TH_INTR_POS_PD0, 1);
		r = REG_SET(r, TH_INTR_POS_PU0, 1);
		break;
	case THERM_MEM:
		r = REG_SET(0, TH_INTR_POS_MD0, 1);
		r = REG_SET(r, TH_INTR_POS_MU0, 1);
		break;
	default:
		r = 0;
		break;
	}
	soctherm_writel(r, TH_INTR_ENABLE);
}

/**
 * soctherm_update_zone() - Updates the given zone.
 * @zn:		The number of the zone to be updated.
 *		This number correlates to one of the following:
 *		CPU, GPU, MEM, or PLL.
 *
 * Based on current temperature and the trip points associated with
 * this zone, update the temperature thresholds at which hardware will
 * generate interrupts.
 *
 * Return: Nothing is returned (void).
 */
static void soctherm_update_zone(int zn)
{
	long low_temp = 0, high_temp = MAX_HIGH_TEMP;
	long trip_temp, passive_low_temp = MAX_HIGH_TEMP, zone_temp;
	enum thermal_trip_type trip_type;
	struct thermal_trip_info *trip_state;
	struct thermal_zone_device *cur_thz = soctherm_th_zones[zn];
	int count, trips;

	thermal_zone_device_update(cur_thz);

	trips = cur_thz->trips;
	for (count = 0; count < trips; count++) {
		cur_thz->ops->get_trip_type(cur_thz, count, &trip_type);
		if (trip_type == THERMAL_TRIP_HOT)
			continue; /* handled in HW */

		cur_thz->ops->get_trip_temp(cur_thz, count, &trip_temp);

		trip_state = &plat_data.therm[zn].trips[count];
		zone_temp = cur_thz->temperature;

		if (!trip_state->tripped) { /* not tripped? update high */
			if (trip_temp < high_temp)
				high_temp = trip_temp;
		} else { /* tripped? update low */
			if (trip_type != THERMAL_TRIP_PASSIVE) {
				/* get highest ACTIVE and CRITICAL*/
				if (trip_temp > low_temp)
					low_temp = trip_temp;
			} else {
				/* get lowest PASSIVE */
				if (trip_temp < passive_low_temp)
					passive_low_temp = trip_temp;
			}
		}
	}

	if (passive_low_temp != MAX_HIGH_TEMP)
		low_temp = max(low_temp, passive_low_temp);

	soctherm_set_limits(zn, low_temp, high_temp);
}

/**
 * soctherm_update() - updates all thermal zones
 *
 * Will not run if the board-specific data has not been initialized. Loops
 * through all of the thermal zones and makes sure that their high and low
 * temperature limits are updated.
 */
static void soctherm_update(void)
{
	int i;

	if (!soctherm_init_platform_done)
		return;

	for (i = 0; i < THERM_SIZE; i++) {
		if (soctherm_th_zones[i] && soctherm_th_zones[i]->trips)
			soctherm_update_zone(i);
	}
}

/**
  * soctherm_hw_action_get_max_state() - gets the max state for cooling
  *	devices associated with hardware throttling
  * @cdev:       cooling device to get the state
  * @max_state:  pointer where the maximum state will be written to
  *
  * Sets @max_state = 3. See soctherm_hw_action_get_cur_state.
  *
  * Return: 0
  */
static int soctherm_hw_action_get_max_state(struct thermal_cooling_device *cdev,
					    unsigned long *max_state)
{
	struct thermal_trip_info *trip_state = cdev->devdata;

	if (!trip_state)
		return 0;

	*max_state = 3; /* bit 1: CPU  bit 2: GPU */
	return 0;
}

/**
 * soctherm_get_cpu_throt_state - read the current state of the CPU pulse skipper
 * @dividend: pulse skipper numerator value to test against (1-256)
 * @divisor: pulse skipper denominator value to test against (1-256)
 * @cur_state: ptr to the variable that the current throttle state is stored in
 *
 * Determine the current state of the CPU thermal throttling pulse
 * skipper, and if it's enabled and at its configured ending state,
 * set the appropriate 'enabled' bit in the variable pointed to by
 * @cur_state.  This works on T114, T124, and T148 by comparing
 * @dividend and @divisor with the current state of the hardware -
 * though note that @dividend and @divisor must be the actual dividend
 * and divisor values.  That is, they must be in 1-256 range, not the
 * 0-255 range used by the hardware bitfields.
 *
 * FIXME: For T132 switch to Denver:CCROC NV_THERM style status.  Does
 * not currently work on T132.
 *
 * Return: 0 upon success, -ENOTSUPP on T12x and T13x, or -EINVAL if
 * the arguments are invalid or out of range.
 *
 */
static int soctherm_get_cpu_throt_state(u16 dividend, u16 divisor,
					unsigned long *cur_state)
{
	u16 m, n;
	u8 enabled, sw_override;

	if (!cur_state || dividend == 0 || divisor == 0 ||
	    dividend > 256 || divisor > 256)
		return -EINVAL;

	if (soctherm_has_mn_cpu_pskip_status()) {
		soctherm_get_mn_cpu_pskip_status(&enabled, &sw_override, &m, &n);

		if (enabled && m == dividend && n == divisor)
			*cur_state |= (1 << THROTTLE_DEV_CPU);
	} else {
		pr_warn_once("CPU throttling status not yet available on this SoC\n");
		return -EINVAL;
	}

	return 0;
}

/**
 * soctherm_get_gpu_throt_state - read the current state of the GPU pulse skipper
 * @dividend: pulse skipper numerator value to test against (1-256)
 * @divisor: pulse skipper denominator value to test against (1-256)
 * @cur_state: ptr to the variable that the current throttle state is stored in
 *
 * Determine the current state of the GPU thermal throttling pulse
 * skipper, and if it's enabled and at its configured ending state,
 * set the appropriate 'enabled' bit in the variable pointed to by
 * @cur_state.  This works on T114 and T148 by comparing @dividend and
 * @divisor with the current state of the hardware - though note that
 * @dividend and @divisor must be the actual dividend and divisor
 * values.  That is, they must be in 1-256 range, not the 0-255 range used
 * by the hardware bitfields.
 *
 * Unfortunately, on T12x and T13x, the GPU manages its own thermal
 * throttling, and does not report its state to the SOC_THERM IP
 * block.  So on those chips, this function will return an error.
 *
 * Return: 0 upon success, -ENOTSUPP on T12x and T13x, or -EINVAL if
 * the arguments are invalid or out of range.
 */
static int soctherm_get_gpu_throt_state(u16 dividend, u16 divisor,
					unsigned long *cur_state)
{
	u16 m, n;
	u8 enabled, sw_override;
	int r;

	if (!cur_state || dividend == 0 || divisor == 0 ||
	    dividend > 256 || divisor > 256)
		return -EINVAL;

	/*
	 * XXX should be replaced with an earlier DT property read to
	 * determine the GPU type (or GPU->SOC_THERM integration) in
	 * use
	 */
	if (!soctherm_has_gpu_pskip_status())
		return -ENOTSUPP;

	r = soctherm_get_gpu_pskip_status(&enabled, &sw_override, &m, &n);
	if (r) {
		WARN_ON(1);
		return r;
	}

	if (!enabled)
		return 0;

	*cur_state |= (m == dividend && n == divisor) ?
		(1 << THROTTLE_DEV_GPU) : 0;

	return r;
}

/**
 * soctherm_hw_action_get_cur_state() - get the current CPU/GPU throttling state
 * @cdev: ptr to the struct thermal_cooling_device associated with SOC_THERM
 * @cur_state: ptr to a variable to return the throttling state into
 *
 * Query the current state of the SOC_THERM cooling device represented
 * by @cdev, and return its current state into the variable pointed to
 * by @cur_state.  Intended to be used as a thermal framework callback
 * function.
 *
 * Return: 0.
 */
static int soctherm_hw_action_get_cur_state(struct thermal_cooling_device *cdev,
					    unsigned long *cur_state)
{
	struct thermal_trip_info *trip_state = cdev->devdata;
	struct soctherm_throttle_dev *devs;
	int i, r;

	if (!trip_state)
		return 0;

	*cur_state = 0;
	if (trip_state->trip_type != THERMAL_TRIP_HOT)
		return 0;

	for (i = THROTTLE_LIGHT; i <= THROTTLE_HEAVY; i++) {
		if (!strnstr(trip_state->cdev_type, throt_names[i],
			     THERMAL_NAME_LENGTH))
			continue;

		devs = &plat_data.throttle[i].devs[THROTTLE_DEV_CPU];
		if (devs->enable)
			soctherm_get_cpu_throt_state(devs->dividend + 1,
						     devs->divisor + 1,
						     cur_state);

		devs = &plat_data.throttle[i].devs[THROTTLE_DEV_GPU];
		if (devs->enable) {
			r = soctherm_get_gpu_throt_state(devs->dividend + 1,
							 devs->divisor + 1,
							 cur_state);
			/*
			 * On some chips, the GPU thermal throttling
			 * status isn't reported back to the SOC_THERM
			 * hardware.  The ideal situation is for the
			 * GPU driver to register its own cooling
			 * device in that case; however, that code
			 * isn't implemented AFAIK.  On those chips,
			 * Diwakar's preferred approach is for the GPU
			 * throttling status bit to follow the CPU
			 * throttling status bit, since that's the
			 * vendor- recommended thermal configuration.
			 * Diwakar notes: On Tegra12x OC5 is a
			 * reserved alarm. Hence GPU 'PSKIP' state
			 * always shows ON. The real status register
			 * 'NV_THERM_CLK_STATUS' can't be read safely
			 * [from this code - pjw]. So we mirror the
			 * CPU status.
			 */
			if (r == -ENOTSUPP)
				if (*cur_state & (1 << THROTTLE_DEV_CPU))
					*cur_state |= (1 << THROTTLE_DEV_GPU);
		}

	}

	return 0;
}

static int soctherm_hw_action_set_cur_state(struct thermal_cooling_device *cdev,
					    unsigned long cur_state)
{
	return 0; /* hw sets this state */
}

static struct thermal_cooling_device *soctherm_hw_critical_cdev;
static struct thermal_cooling_device *soctherm_hw_heavy_cdev;
static struct thermal_cooling_device *soctherm_hw_light_cdev;
static struct thermal_cooling_device_ops soctherm_hw_action_ops = {
	.get_max_state = soctherm_hw_action_get_max_state,
	.get_cur_state = soctherm_hw_action_get_cur_state,
	.set_cur_state = soctherm_hw_action_set_cur_state,
};

static int soctherm_suspend_get_max_state(struct thermal_cooling_device *cdev,
					  unsigned long *max_state)
{
	*max_state = 1;
	return 0;
}

static int soctherm_suspend_get_cur_state(struct thermal_cooling_device *cdev,
					  unsigned long *cur_state)
{
	*cur_state = !soctherm_suspended;
	return 0;
}

/**
 * soctherm_suspend_set_cur_state() - Resumes or suspends soctherm
 * @cdev:		Thermal cooling device. Currently not being used.
 * @cur_state:		The current state
 *
 * Ensures that the SOC_THERM device is suspended or resumed to match
 * @cur_state. This function is passed to the thermal framework as part of a
 * cooling device. This is a workaround to suspend the SOC_THERM IP block, which
 * is only needed because this is not yet a device driver. Once this code is
 * converted to be a device driver, the soctherm_suspend implementation can
 * be removed
 * Return: 0 (success).
 */
static int soctherm_suspend_set_cur_state(struct thermal_cooling_device *cdev,
					  unsigned long cur_state)
{
	if (!cur_state != soctherm_suspended) {
		if (cur_state)
			soctherm_resume();
		else
			soctherm_suspend();
	}
	return 0;
}

static struct thermal_cooling_device_ops soctherm_suspend_ops = {
	.get_max_state = soctherm_suspend_get_max_state,
	.get_cur_state = soctherm_suspend_get_cur_state,
	.set_cur_state = soctherm_suspend_set_cur_state,
};

/**
 * soctherm_bind() - Binds the given thermal zone's trip
 * points with the given cooling device.
 * @thz:	The thermal zone device to be bound
 * @cdev:	The cooling device to be bound
 *
 * If thermal sensor calibration data is missing from fuses,
 * the cooling devices are not bound.
 *
 * Based on platform-specific configuration associated with this
 * thermal zone, soctherm_bind() binds this cooling device to this
 * thermal zone at various trip points.
 *
 * soctherm_bind is called as a thermal_zone_device_ops bind function.
 *
 * Return: Returns 0 on successful binding. Returns 0 if passed an
 * invalid thermal zone argument, or improperly fused soctherm.
 * In the latter two cases, binding of the cooling device does not
 * occur.
 */
static int soctherm_bind(struct thermal_zone_device *thz,
				struct thermal_cooling_device *cdev)
{
	int i;
	struct soctherm_therm *therm = thz->devdata;
	struct thermal_trip_info *trip_state;

	/* skip binding cooling devices on improperly fused soctherm */
	if (tegra_fuse_calib_base_get_cp(NULL, NULL) < 0 ||
	    tegra_fuse_calib_base_get_ft(NULL, NULL) < 0)
		return 0;

	for (i = 0; i < therm->num_trips; i++) {
		trip_state = &therm->trips[i];
		if (trip_state->cdev_type &&
		    !strncmp(trip_state->cdev_type, cdev->type,
						THERMAL_NAME_LENGTH)) {
			thermal_zone_bind_cooling_device(thz, i, cdev,
							 trip_state->upper,
							 trip_state->lower);
			trip_state->bound = true;
		}
	}

	return 0;
}

/**
 * soctherm_unbind() - unbinds cooling device from a thermal zone.
 * @thz:        thermal zone to be dissociated with a cooling device
 * @cdev:       a cooling device to be dissociated with a thermal zone
 *
 * Dissociates a given cooling device from a given thermal zone.
 * This function will go through every trip point and dissociate
 * cooling device from the thermal zone.
 *
 * Return: 0
 */
static int soctherm_unbind(struct thermal_zone_device *thz,
				struct thermal_cooling_device *cdev)
{
	int i;
	struct soctherm_therm *therm = thz->devdata;
	struct thermal_trip_info *trip_state;

	for (i = 0; i < therm->num_trips; i++) {
		trip_state = &therm->trips[i];
		if (!trip_state->bound)
			continue;
		if (trip_state->cdev_type &&
		    !strncmp(trip_state->cdev_type, cdev->type,
						THERMAL_NAME_LENGTH)) {
			thermal_zone_unbind_cooling_device(thz, 0, cdev);
			trip_state->bound = false;
		}
	}

	return 0;
}

/**
 * soctherm_get_temp() - gets the temperature for the given thermal zone
 * @thz:	the thermal zone from which to get the temperature
 * @temp:	a pointer to where the temperature will be stored
 *
 * Reads the sensor associated with the given thermal zone, converts the
 * reading to millicelcius, and places it into temp. This function is passed
 * to the thermal framework as a callback function when the zone is created and
 * registered.
 *
 * Return: 0
 */
static int soctherm_get_temp(struct thermal_zone_device *thz, long *temp)
{
	struct soctherm_therm *therm = thz->devdata;
	ptrdiff_t index = therm - plat_data.therm;
	u32 r, regv, shft, mask;
	enum soctherm_sense i, j;
	int tt, ti;

	switch (index) {
	case THERM_CPU:
		regv = TS_TEMP1;
		shft = TS_TEMP1_CPU_TEMP_SHIFT;
		mask = TS_TEMP1_CPU_TEMP_MASK;
		i = TSENSE_CPU0;
		j = TSENSE_CPU3;
		break;

	case THERM_GPU:
		regv = TS_TEMP1;
		shft = TS_TEMP1_GPU_TEMP_SHIFT;
		mask = TS_TEMP1_GPU_TEMP_MASK;
		i = TSENSE_GPU;
		j = TSENSE_GPU;
		break;

	case THERM_MEM:
		regv = TS_TEMP2;
		shft = TS_TEMP2_MEM_TEMP_SHIFT;
		mask = TS_TEMP2_MEM_TEMP_MASK;
		i = TSENSE_MEM0;
		j = TSENSE_MEM1;
		break;

	case THERM_PLL:
	default: /* if devdata has error, return PLL temp to be safe */
		regv = TS_TEMP2;
		shft = TS_TEMP2_PLLX_TEMP_SHIFT;
		mask = TS_TEMP2_PLLX_TEMP_MASK;
		i = TSENSE_PLLX;
		j = TSENSE_PLLX;
		break;
	}

	if (read_hw_temp) {
		r = soctherm_readl(regv);
		*temp = temp_translate((r & (mask << shft)) >> shft);
	} else {
		for (tt = 0; i <= j; i++) {
			r = soctherm_readl(TS_TSENSE_REG_OFFSET(
						TS_CPU0_STATUS0, i));
			ti = temp_convert(REG_GET(r, TS_CPU0_STATUS0_CAPTURE),
						sensor2therm_a[i],
						sensor2therm_b[i]);
			*temp = tt = max(tt, ti);
		}
	}
	return 0;
}

/**
 * soctherm_get_trip_type() - Gets the type of a given trip point
 * for a given thermal zone device.
 * @thz:	The thermal zone device
 * @trip:	The trip point index.
 * @type:	The trip type.
 *
 * The trip type will be one of the following values:
 * THERMAL_TRIP_ACTIVE, THERMAL_TRIP_PASSIVE, THERMAL_TRIP_HOT,
 * THERMAL_TRIP_CRITICAL
 *
 * This function is passed to the thermal framework as a callback
 * for each of the SOC_THERM-related thermal zones
 *
 * Return: Returns 0 on success, -EINVAL when passed an invalid argument.
 */
static int soctherm_get_trip_type(struct thermal_zone_device *thz,
				int trip, enum thermal_trip_type *type)
{
	struct soctherm_therm *therm = thz->devdata;

	*type = therm->trips[trip].trip_type;
	return 0;
}

/**
 * soctherm_get_trip_temp() - gets the threshold of a trip point from a zone
 * @thz:	the thermal zone whose trip point temperature will be accessed
 * @trip:	the index of the trip point
 * @temp:	a pointer to where the temperature will be stored
 *
 * Reads the temperature threshold value of the specified trip point from the
 * specified thermal zone (in millicelsius) and places it into temp. It also
 * update's the zone's tripped flag. This function is passed to the thermal
 * framework as a callback function for each thermal zone.
 *
 * Return: 0 if success, otherwise %-EINVAL.
 */

static int soctherm_get_trip_temp(struct thermal_zone_device *thz,
				int trip, long *temp)
{
	struct soctherm_therm *therm = thz->devdata;
	struct thermal_trip_info *trip_state;
	long trip_temp, zone_temp;

	trip_state = &therm->trips[trip];
	trip_temp = trip_state->trip_temp;
	zone_temp = thz->temperature;

	if (zone_temp >= trip_temp) {
		trip_temp -= trip_state->hysteresis;
		trip_state->tripped = true;
	} else if (trip_state->tripped) {
		trip_temp -= trip_state->hysteresis;
		if (zone_temp < trip_temp)
			trip_state->tripped = false;
	}

	*temp = trip_temp;
	return 0;
}

/**
 * soctherm_set_trip_temp() - updates trip temperature
 *	for a particular trip point
 * @thz:	pointer to thermal_zone_device to update trip temperature
 * @trip:	index value of thermal_trip_info in soctherm_therm->trips
 * @temp:	value for new temperature
 *
 * Updates both the software data structure and the hardware threshold
 * for a trip point. This function is passed to the thermal framework
 * as a callback function for each of the thermal zone.
 *
 * Return: 0 if successful else %-EINVAL
 */
static int soctherm_set_trip_temp(struct thermal_zone_device *thz,
				int trip, long temp)
{
	struct soctherm_therm *therm = thz->devdata;
	struct thermal_trip_info *trip_state;
	ptrdiff_t index = therm - plat_data.therm;
	long rem;

	trip_state = &therm->trips[trip];
	trip_state->trip_temp = enforce_temp_range(temp);

	rem = trip_state->trip_temp % LOWER_PRECISION_FOR_CONV(1000);
	if (rem) {
		pr_warn("soctherm: zone%d/trip_point%d %ld mC rounded down\n",
			thz->id, trip, trip_state->trip_temp);
		trip_state->trip_temp -= rem;
	}

	if (trip_state->trip_type == THERMAL_TRIP_HOT) {
		if (strnstr(trip_state->cdev_type,
			    "heavy", THERMAL_NAME_LENGTH))
			prog_hw_threshold(trip_state, index, THROTTLE_HEAVY);
		else if (strnstr(trip_state->cdev_type,
				 "light", THERMAL_NAME_LENGTH))
			prog_hw_threshold(trip_state, index, THROTTLE_LIGHT);
	}

	/* Allow SW to shutdown at 'Critical temperature reached' */
	thermal_notify_framework(thz, trip);

	/* Reprogram HW thermtrip */
	if (trip_state->trip_type == THERMAL_TRIP_CRITICAL)
		prog_hw_shutdown(trip_state, index);

	return 0;
}

/**
 * soctherm_get_crit_temp() - Gets critical temperature of a thermal zone
 * @tzd:		The pointer to thermal zone device
 * @temp:		The pointer to the temperature
 *
 * Iterates through the list of thermal trips for a given @thz, and looks for
 * its critical temperature point @temp to cause a shutdown.
 *
 * Return: 0 if it is able to find a critical temperature point and stores it
 * into the variable pointed by the address in @temp; Otherwise, return -EINVAL.
 */
static int soctherm_get_crit_temp(struct thermal_zone_device *thz, long *temp)
{
	int i;
	struct soctherm_therm *therm = thz->devdata;
	struct thermal_trip_info *trip_state;

	for (i = 0; i < therm->num_trips; i++) {
		trip_state = &therm->trips[i];
		if (trip_state->trip_type != THERMAL_TRIP_CRITICAL)
			continue;
		*temp = trip_state->trip_temp;
		return 0;
	}

	return -EINVAL;
}

/**
 * soctherm_get_trend() - Gets the thermal trend for a given
 * thermal zone device
 * @thz:	The thermal zone device whose trend is being obtained
 * @trip:	The trip point number
 * @trend:	The thermal trend
 *
 * This function is passed to the thermal framework as a callback
 * for SOC_THERM's thermal zone devices
 *
 * The trend will be one of the following:
 * THERMAL_TREND_STABLE: the temperature is stable
 * THERMAL_TREND_RAISING: the temperature is increasing
 * THERMAL_TREND_DROPPING: the temperature is decreasing
 * THERMAL_TREND_RAISE_FULL: apply the highest cooling action
 * THERMAL_TREND_DROP_FULL: apply the lowest cooling action
 *
 * If the trip type of the trip point of the thermal zone device is
 * THERMAL_TRIP_ACTIVE, then the thermal trend is THERMAL_TREND_RAISING.
 * Otherwise, if the device's temperature is higher than its trip
 * temperature, the trend is THERMAL_TREND_RAISING. If the device's
 * temperature is lower, the trend is THERMAL_TREND_DROPPING. Otherwise
 * the trend is stable.
 *
 * Return: 0 on success. Returns -EINVAL if the function was
 * passed an invalid argument.
 */
static int soctherm_get_trend(struct thermal_zone_device *thz,
				int trip,
				enum thermal_trend *trend)
{
	struct soctherm_therm *therm = thz->devdata;
	struct thermal_trip_info *trip_state;
	long trip_temp;

	trip_state = &therm->trips[trip];
	thz->ops->get_trip_temp(thz, trip, &trip_temp);

	switch (trip_state->trip_type) {
	case THERMAL_TRIP_ACTIVE:
		/* aggressive active cooling */
		*trend = THERMAL_TREND_RAISING;
		break;
	case THERMAL_TRIP_PASSIVE:
		if (thz->temperature > trip_state->trip_temp)
			*trend = THERMAL_TREND_RAISING;
		else if (thz->temperature < trip_temp)
			*trend = THERMAL_TREND_DROPPING;
		else
			*trend = THERMAL_TREND_STABLE;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static struct thermal_zone_device_ops soctherm_ops = {
	.bind = soctherm_bind,
	.unbind = soctherm_unbind,
	.get_temp = soctherm_get_temp,
	.get_trip_type = soctherm_get_trip_type,
	.get_trip_temp = soctherm_get_trip_temp,
	.set_trip_temp = soctherm_set_trip_temp,
	.get_crit_temp = soctherm_get_crit_temp,
	.get_trend = soctherm_get_trend,
};

/**
  * soctherm_hot_cdev_register() - registers cooling devices
  *	associated with hardware throttling.
  * @i:		soctherm_therm_id index of the sensor group
  * @trip:	index of thermal_trip_info in soctherm_therm->trips
  *
  * As indicated by platform configuration data, registers with
  * the thermal framework two cooling devices representing
  * SOC_THERM's hardware throttling capability associated with
  * sensor group @i
  *
  * These cooling devices are special. To function properly, they must be
  * bound (with a single trip point) to the thermal zone associated with
  * the same sensor group.
  *
  * Setting the trip point temperature leads to an adjustment of the
  * hardware throttling temperature threshold. Examining the cooling
  * device's cur_state indicates whether hardware throttling is active.
  */
static void __init soctherm_hot_cdev_register(int i, int trip)
{
	struct soctherm_therm *therm;
	int k;

	therm = &plat_data.therm[i];

	for (k = 0; k < THROTTLE_SIZE; k++) {
		if ((therm2dev[i] == THROTTLE_DEV_NONE) ||
		    (!plat_data.throttle[k].devs[therm2dev[i]].enable))
			continue;

		if ((strnstr(therm->trips[trip].cdev_type, "oc1",
			     THERMAL_NAME_LENGTH) && k == THROTTLE_OC1) ||
		    (strnstr(therm->trips[trip].cdev_type, "oc2",
			     THERMAL_NAME_LENGTH) && k == THROTTLE_OC2) ||
		    (strnstr(therm->trips[trip].cdev_type, "oc3",
			     THERMAL_NAME_LENGTH) && k == THROTTLE_OC3) ||
		    (strnstr(therm->trips[trip].cdev_type, "oc4",
			     THERMAL_NAME_LENGTH) && k == THROTTLE_OC4))
			continue;

		if (strnstr(therm->trips[trip].cdev_type,
			    "heavy",
			    THERMAL_NAME_LENGTH) &&
		    k == THROTTLE_HEAVY &&
		    !soctherm_hw_heavy_cdev) {
			soctherm_hw_heavy_cdev =
				thermal_cooling_device_register(
					therm->trips[trip].cdev_type,
					&therm->trips[trip],
					&soctherm_hw_action_ops);
			continue;
		}

		if (strnstr(therm->trips[trip].cdev_type,
			    "light",
			    THERMAL_NAME_LENGTH) &&
		    k == THROTTLE_LIGHT &&
		    !soctherm_hw_light_cdev) {
			soctherm_hw_light_cdev =
				thermal_cooling_device_register(
					therm->trips[trip].cdev_type,
					&therm->trips[trip],
					&soctherm_hw_action_ops);
			continue;
		}
	}
}

/**
 * soctherm_thermal_sys_init() - initializes the SOC_THERM thermal system
 *
 * After the board-specific data has been initalized, this creates a thermal
 * zone device for each enabled sensor and each enabled sensor group.
 * It also creates a cooling zone device for each enabled thermal zone that has
 * a critical trip point. It enables the suspend feature if no over-current
 * alarms are enabled.
 *
 * Once all of the thermal zones have been registered, it runs
 * soctherm_update(), which sets high and low temperature thresholds.
 *
 * Runs at kernel boot-time.
 *
 * Return: 0
 */
static int __init soctherm_thermal_sys_init(void)
{
	char name[THERMAL_NAME_LENGTH];
	struct soctherm_therm *therm;
	bool oc_en = false;
	int i, j;

	if (!soctherm_init_platform_done)
		return 0;

	for (i = 0; i < THERM_SIZE; i++) {
		therm = &plat_data.therm[i];
		if (!therm->zone_enable)
			continue;

		for (j = 0; j < therm->num_trips; j++) {
			switch (therm->trips[j].trip_type) {
			case THERMAL_TRIP_CRITICAL:
				if (soctherm_hw_critical_cdev)
					break;
				soctherm_hw_critical_cdev =
					thermal_cooling_device_register(
						therm->trips[j].cdev_type,
						&therm->trips[j],
						&soctherm_hw_action_ops);
				break;

			case THERMAL_TRIP_HOT:
				soctherm_hot_cdev_register(i, j);
				break;

			case THERMAL_TRIP_PASSIVE:
			case THERMAL_TRIP_ACTIVE:
				break; /* done elsewhere */
			}
		}

		snprintf(name, THERMAL_NAME_LENGTH, "%s-therm", therm_names[i]);
		soctherm_th_zones[i] = thermal_zone_device_register(
						name,
						therm->num_trips,
						(1ULL << therm->num_trips) - 1,
						therm,
						&soctherm_ops,
						therm->tzp,
						therm->passive_delay,
						0);

		for (j = THROTTLE_OC1; !oc_en && j < THROTTLE_SIZE; j++)
			if ((therm2dev[i] != THROTTLE_DEV_NONE) &&
			    (plat_data.throttle[j].devs[therm2dev[i]].enable))
				oc_en = true;
	}

	/* do not enable suspend feature if any OC alarms are enabled */
	if (!oc_en)
		thermal_cooling_device_register("suspend_soctherm", NULL,
						&soctherm_suspend_ops);
	else
		pr_warn("soctherm: Suspend feature CANNOT be enabled %s\n",
			"when any OC alarm is enabled");

	soctherm_update();
	return 0;
}
module_init(soctherm_thermal_sys_init);

#else
static void soctherm_update_zone(int zn)
{
}
static void soctherm_update(void)
{
}
#endif

/**
 * soctherm_thermal_thread_func() - Handles a thermal interrupt request
 * @irq:	The interrupt number being requested; not used
 * @arg:	Opaque pointer to an argument; not used
 *
 * Clears the interrupt status register if there are expected
 * interrupt bits set.
 * The interrupt(s) are then handled by updating the corresponding
 * thermal zones.
 *
 * An error is logged if any unexpected interrupt bits are set.
 *
 * Disabled interrupts are re-enabled.
 *
 * Return: %IRQ_HANDLED. Interrupt was handled and no further processing
 * is needed.
 */
static irqreturn_t soctherm_thermal_thread_func(int irq, void *arg)
{
	u32 st, ex = 0, cp = 0, gp = 0, pl = 0;

	st = soctherm_readl(TH_INTR_STATUS);

	/* deliberately clear expected interrupts handled in SW */
	cp |= REG_GET_BIT(st, TH_INTR_POS_CD0);
	cp |= REG_GET_BIT(st, TH_INTR_POS_CU0);
	ex |= cp;

	gp |= REG_GET_BIT(st, TH_INTR_POS_GD0);
	gp |= REG_GET_BIT(st, TH_INTR_POS_GU0);
	ex |= gp;

	pl |= REG_GET_BIT(st, TH_INTR_POS_PD0);
	pl |= REG_GET_BIT(st, TH_INTR_POS_PU0);
	ex |= pl;

	if (ex) {
		soctherm_writel(ex, TH_INTR_STATUS);
		st &= ~ex;
		if (cp)
			soctherm_update_zone(THERM_CPU);
		if (gp)
			soctherm_update_zone(THERM_GPU);
		if (pl)
			soctherm_update_zone(THERM_PLL);
	}

	/* deliberately ignore expected interrupts NOT handled in SW */
	ex |= REG_GET_BIT(st, TH_INTR_POS_MD0);
	ex |= REG_GET_BIT(st, TH_INTR_POS_MU0);

	ex |= REG_GET_BIT(st, TH_INTR_POS_CD1);
	ex |= REG_GET_BIT(st, TH_INTR_POS_CU1);
	ex |= REG_GET_BIT(st, TH_INTR_POS_CD2);
	ex |= REG_GET_BIT(st, TH_INTR_POS_CU2);
	ex |= REG_GET_BIT(st, TH_INTR_POS_CD3);
	ex |= REG_GET_BIT(st, TH_INTR_POS_CU3);

	ex |= REG_GET_BIT(st, TH_INTR_POS_GD1);
	ex |= REG_GET_BIT(st, TH_INTR_POS_GU1);
	ex |= REG_GET_BIT(st, TH_INTR_POS_GD2);
	ex |= REG_GET_BIT(st, TH_INTR_POS_GU2);
	ex |= REG_GET_BIT(st, TH_INTR_POS_GD3);
	ex |= REG_GET_BIT(st, TH_INTR_POS_GU3);

	ex |= REG_GET_BIT(st, TH_INTR_POS_PD1);
	ex |= REG_GET_BIT(st, TH_INTR_POS_PU1);
	ex |= REG_GET_BIT(st, TH_INTR_POS_PD2);
	ex |= REG_GET_BIT(st, TH_INTR_POS_PU2);
	ex |= REG_GET_BIT(st, TH_INTR_POS_PD3);
	ex |= REG_GET_BIT(st, TH_INTR_POS_PU3);

	ex |= REG_GET_BIT(st, TH_INTR_POS_MD1);
	ex |= REG_GET_BIT(st, TH_INTR_POS_MU1);
	ex |= REG_GET_BIT(st, TH_INTR_POS_MD2);
	ex |= REG_GET_BIT(st, TH_INTR_POS_MU2);
	ex |= REG_GET_BIT(st, TH_INTR_POS_MD3);
	ex |= REG_GET_BIT(st, TH_INTR_POS_MU3);
	st &= ~ex;

	if (st) {
		/* Whine about any other unexpected INTR bits still set */
		pr_err("soctherm: Ignored unexpected INTRs 0x%08x\n", st);
		soctherm_writel(st, TH_INTR_STATUS);
	}

	return IRQ_HANDLED;
}

/**
 * soctherm_oc_intr_enable() - Enables the soctherm over-current interrupt
 * @alarm:		The soctherm throttle id
 * @enable:		Flag indicating enable the soctherm over-current
 *			interrupt or disable it
 *
 * Enables a specific over-current pins @alarm to raise an interrupt if the flag
 * is set and the alarm corresponds to OC1, OC2, OC3, or OC4.
 */
static inline void soctherm_oc_intr_enable(enum soctherm_throttle_id alarm,
					   bool enable)
{
	u32 r;

	if (!enable)
		return;

	r = soctherm_readl(OC_INTR_ENABLE);
	switch (alarm) {
	case THROTTLE_OC1:
		r = REG_SET(r, OC_INTR_POS_OC1, 1);
		break;
	case THROTTLE_OC2:
		r = REG_SET(r, OC_INTR_POS_OC2, 1);
		break;
	case THROTTLE_OC3:
		r = REG_SET(r, OC_INTR_POS_OC3, 1);
		break;
	case THROTTLE_OC4:
		r = REG_SET(r, OC_INTR_POS_OC4, 1);
		break;
	default:
		r = 0;
		break;
	}
	soctherm_writel(r, OC_INTR_ENABLE);
}

/**
 * soctherm_handle_alarm() - Handles soctherm alarms
 * @alarm:		The soctherm throttle id
 *
 * "Handles" over-current alarms (OC1, OC2, OC3, and OC4) by printing
 * a warning or informative message.
 *
 * Return: -EINVAL for @alarm = THROTTLE_OC3, otherwise 0 (success).
 */
static int soctherm_handle_alarm(enum soctherm_throttle_id alarm)
{
	int rv = -EINVAL;

	switch (alarm) {
	case THROTTLE_OC1:
		pr_debug("soctherm: Successfully handled OC1 alarm\n");
		/* add OC1 alarm handling code here */
		rv = 0;
		break;

	case THROTTLE_OC2:
		pr_info("soctherm: Successfully handled OC2 alarm\n");
		/* TODO: add OC2 alarm handling code here */
		rv = 0;
		break;

	case THROTTLE_OC3:
		pr_warn("soctherm: Unexpected OC3 alarm\n");
		/* add OC3 alarm handling code here */
		break;

	case THROTTLE_OC4:
		pr_debug("soctherm: Successfully handled OC4 alarm\n");
		/* TODO: add OC4 alarm handling code here */
		rv = 0;
		break;

	default:
		break;
	}

	if (rv)
		pr_err("soctherm: ERROR in handling %s alarm\n",
		       throt_names[alarm]);

	return rv;
}

/**
 * soctherm_edp_thread_func() - log an over-current interrupt request
 * @irq:	OC irq number. Currently not being used. See description
 * @arg:	a void pointer for callback, currently not being used
 *
 * Over-current events are handled in hardware. This function is called to log
 * and handle any OC events that happened. Additionally, it checks every
 * over-current interrupt registers for registers are set but
 * was not expected (i.e. any discrepancy in interrupt status) by the function,
 * the discrepancy will logged.
 *
 * Return: %IRQ_HANDLED
 */
static irqreturn_t soctherm_edp_thread_func(int irq, void *arg)
{
	u32 st, ex, oc1, oc2, oc3, oc4;

	st = soctherm_readl(OC_INTR_STATUS);

	/* deliberately clear expected interrupts handled in SW */
	oc1 = REG_GET_BIT(st, OC_INTR_POS_OC1);
	oc2 = REG_GET_BIT(st, OC_INTR_POS_OC2);
	oc3 = REG_GET_BIT(st, OC_INTR_POS_OC3);
	oc4 = REG_GET_BIT(st, OC_INTR_POS_OC4);
	ex = oc1 | oc2 | oc3 | oc4;

	if (ex) {
		soctherm_writel(st, OC_INTR_STATUS);
		st &= ~ex;

		if (oc1 && !soctherm_handle_alarm(THROTTLE_OC1))
			soctherm_oc_intr_enable(THROTTLE_OC1, true);

		if (oc2 && !soctherm_handle_alarm(THROTTLE_OC2))
			soctherm_oc_intr_enable(THROTTLE_OC2, true);

		if (oc3 && !soctherm_handle_alarm(THROTTLE_OC3))
			soctherm_oc_intr_enable(THROTTLE_OC3, true);

		if (oc4 && !soctherm_handle_alarm(THROTTLE_OC4))
			soctherm_oc_intr_enable(THROTTLE_OC4, true);

		if (oc1 && soc_irq_cdata.irq_enable & BIT(0))
			handle_nested_irq(
				irq_find_mapping(soc_irq_cdata.domain, 0));

		if (oc2 && soc_irq_cdata.irq_enable & BIT(1))
			handle_nested_irq(
				irq_find_mapping(soc_irq_cdata.domain, 1));

		if (oc3 && soc_irq_cdata.irq_enable & BIT(2))
			handle_nested_irq(
				irq_find_mapping(soc_irq_cdata.domain, 2));

		if (oc4 && soc_irq_cdata.irq_enable & BIT(3))
			handle_nested_irq(
				irq_find_mapping(soc_irq_cdata.domain, 3));
	}

	if (st) {
		pr_err("soctherm: Ignored unexpected OC ALARM 0x%08x\n", st);
		soctherm_writel(st, OC_INTR_STATUS);
	}

	return IRQ_HANDLED;
}

/**
 * soctherm_thermal_isr() - thermal interrupt request handler
 * @irq:	Interrupt request number
 * @arg:	Not used.
 *
 * Reads the thermal interrupt status and then disables any asserted
 * interrupts. The thread woken by this isr services the asserted
 * interrupts and re-enables them.
 *
 * Return: %IRQ_WAKE_THREAD
 */
static irqreturn_t soctherm_thermal_isr(int irq, void *arg)
{
	u32 r;

	r = soctherm_readl(TH_INTR_STATUS);
	soctherm_writel(r, TH_INTR_DISABLE);

	return IRQ_WAKE_THREAD;
}

/**
 * soctherm_edp_isr() - Disables any active interrupts
 * @irq:	The interrupt request number
 * @arg:	Opaque pointer to an argument
 *
 * Writes to the OC_INTR_DISABLE register the over current interrupt status,
 * masking any asserted interrupts. Doing this prevents the same interrupts
 * from triggering this isr repeatedly. The thread woken by this isr will
 * handle asserted interrupts and subsequently unmask/re-enable them.
 *
 * The OC_INTR_DISABLE register indicates which OC interrupts
 * have been disabled.
 *
 * Return: %IRQ_WAKE_THREAD, handler requests to wake the handler thread
 */
static irqreturn_t soctherm_edp_isr(int irq, void *arg)
{
	u32 r;

	r = soctherm_readl(OC_INTR_STATUS);
	soctherm_writel(r, OC_INTR_DISABLE);

	return IRQ_WAKE_THREAD;
}

/**
 * throttlectl_cpu_mn() - program CPU pulse skipper configuration
 * @throt: soctherm_throttle_id describing the level of throttling
 *
 * Pulse skippers are used to throttle clock frequencies.  This
 * function programs the pulse skippers based on @throt and platform
 * data.  This function is used for CPUs that have "remote" pulse
 * skipper control, e.g., the CPU pulse skipper is controlled by the
 * SOC_THERM IP block.  (SOC_THERM is located outside the CPU
 * complex.)
 *
 * Return: boolean true if HW was programmed, or false if the desired
 * configuration is not supported.
 */
static bool throttlectl_cpu_mn(enum soctherm_throttle_id throt)
{
	u32 r;
	struct soctherm_throttle *data = &plat_data.throttle[throt];
	struct soctherm_throttle_dev *dev = &data->devs[THROTTLE_DEV_CPU];

	if (!dev->enable)
		return false;

	r = soctherm_readl(THROT_PSKIP_CTRL(throt, THROTTLE_DEV_CPU));
	r = REG_SET(r, THROT_PSKIP_CTRL_ENABLE, dev->enable);
	r = REG_SET(r, THROT_PSKIP_CTRL_DIVIDEND, dev->dividend);
	r = REG_SET(r, THROT_PSKIP_CTRL_DIVISOR, dev->divisor);
	soctherm_writel(r, THROT_PSKIP_CTRL(throt, THROTTLE_DEV_CPU));

	r = soctherm_readl(THROT_PSKIP_RAMP(throt, THROTTLE_DEV_CPU));
	r = REG_SET(r, THROT_PSKIP_RAMP_DURATION, dev->duration);
	r = REG_SET(r, THROT_PSKIP_RAMP_STEP, dev->step);
	soctherm_writel(r, THROT_PSKIP_RAMP(throt, THROTTLE_DEV_CPU));

	return true;
}

/**
 * throttlectl_cpu_level() - program CPU pulse skipper configuration
 * @throt: soctherm_throttle_id describing the level of throttling
 *
 * Pulse skippers are used to throttle clock frequencies.  This
 * function programs the pulse skippers based on @throt and platform
 * data.  This function is used on SoCs which have CPU-local pulse
 * skipper control, such as T13x. It programs soctherm's interface to
 * Denver:CCROC NV_THERM in terms of Low, Medium and Heavy throttling
 * vectors. PSKIP_BYPASS mode is set as required per HW spec.
 *
 * It's also necessary to set up the CPU-local NV_THERM instance with
 * the M/N values desired for each level.  This function does this
 * also, although it should be handled by a separate driver.
 *
 * Return: boolean true if HW was programmed, or false if the desired
 * configuration is not supported.
 */
static bool throttlectl_cpu_level(enum soctherm_throttle_id throt)
{
	u32 r, throt_vect;
	int throt_level;
	struct soctherm_throttle *data = &plat_data.throttle[throt];
	struct soctherm_throttle_dev *dev = &data->devs[THROTTLE_DEV_CPU];

	if (!dev->enable)
		return false;

	/* Denver:CCROC NV_THERM interface N:3 Mapping */
	if (!strcmp(dev->throttling_depth, "heavy_throttling")) {
		throt_level = THROT_LEVEL_HVY;
		throt_vect = THROT_VECT_HVY;
	} else if (!strcmp(dev->throttling_depth, "medium_throttling")) {
		throt_level = THROT_LEVEL_MED;
		throt_vect = THROT_VECT_MED;
	} else if (!strcmp(dev->throttling_depth, "low_throttling")) {
		throt_level = THROT_LEVEL_LOW;
		throt_vect = THROT_VECT_LOW;
	} else {
		throt_level = THROT_LEVEL_NONE;
		throt_vect = THROT_VECT_NONE;
	}

	if (dev->depth)
		THROT_DEPTH(dev, dev->depth);

	r = soctherm_readl(THROT_PSKIP_CTRL(throt, THROTTLE_DEV_CPU));
	r = REG_SET(r, THROT_PSKIP_CTRL_ENABLE, dev->enable);
	/* setup throttle vector in soctherm register */
	r = REG_SET(r, THROT_PSKIP_CTRL_VECT_CPU, throt_vect);
	r = REG_SET(r, THROT_PSKIP_CTRL_VECT2_CPU, throt_vect);
	soctherm_writel(r, THROT_PSKIP_CTRL(throt, THROTTLE_DEV_CPU));

	/* bypass sequencer in soc_therm as it is programmed in ccroc */
	r = REG_SET(0, THROT_PSKIP_RAMP_SEQ_BYPASS_MODE, 1);
	soctherm_writel(r, THROT_PSKIP_RAMP(throt, THROTTLE_DEV_CPU));

	if (throt_level == THROT_LEVEL_NONE)
		return true;

	/* setup PSKIP in ccroc nv_therm registers */
	r = clk_reset13_readl(THROT13_PSKIP_RAMP_CPU(throt_level));
	r = REG_SET(r, THROT_PSKIP_RAMP_DURATION, dev->duration);
	r = REG_SET(r, THROT_PSKIP_RAMP_STEP, dev->step);
	clk_reset13_writel(r, THROT13_PSKIP_RAMP_CPU(throt_level));

	r = clk_reset13_readl(THROT13_PSKIP_CTRL_CPU(throt_level));
	r = REG_SET(r, THROT_PSKIP_CTRL_ENABLE, dev->enable);
	r = REG_SET(r, THROT_PSKIP_CTRL_DIVIDEND, dev->dividend);
	r = REG_SET(r, THROT_PSKIP_CTRL_DIVISOR, dev->divisor);
	clk_reset13_writel(r, THROT13_PSKIP_CTRL_CPU(throt_level));

	return true;
}

/**
 * throttlectl_gpu_gk20a_nv_therm_style() - programs GK20a NV_THERM config
 * @dev		device struct pointer to GPU device
 * @throt	soctherm_throttle_id describing the level of throttling
 *
 * This function programs soctherm's interface to GK20a NV_THERM in
 * terms of Low, Medium and Heavy throttling preset levels.
 *
 * Return: boolean true if HW was programmed
 */
static bool throttlectl_gpu_gk20a_nv_therm_style(
				struct soctherm_throttle_dev *dev,
				enum soctherm_throttle_id throt)
{
	u32 r, throt_vect;

	/* gk20a nv_therm interface N:3 Mapping */
	if (!strcmp(dev->throttling_depth, "heavy_throttling"))
		throt_vect = THROT_VECT_HVY;
	else if (!strcmp(dev->throttling_depth, "medium_throttling"))
		throt_vect = THROT_VECT_MED;
	else
		throt_vect = THROT_VECT_LOW;

	r = soctherm_readl(THROT_PSKIP_CTRL(throt, THROTTLE_DEV_GPU));
	r = REG_SET(r, THROT_PSKIP_CTRL_ENABLE, dev->enable);
	r = REG_SET(r, THROT_PSKIP_CTRL_VECT_GPU, throt_vect);
	soctherm_writel(r, THROT_PSKIP_CTRL(throt, THROTTLE_DEV_GPU));

	r = soctherm_readl(THROT_PSKIP_RAMP(throt, THROTTLE_DEV_GPU));
	r = REG_SET(r, THROT_PSKIP_RAMP_SEQ_BYPASS_MODE, 1);
	soctherm_writel(r, THROT_PSKIP_RAMP(throt, THROTTLE_DEV_GPU));

	return true;
}

/**
 * throttlectl_gpu() - programs GPU pulse skippers' configuration
 * @throt	soctherm_throttle_id describing the level of throttling
 *
 * Pulse skippers are used to throttle clock frequencies.
 * This function programs the pulse skippers based on @throt and platform data.
 *
 * Return: boolean true if HW was programmed
 */
static bool throttlectl_gpu(enum soctherm_throttle_id throt)
{
	u32 r;
	struct soctherm_throttle *data = &plat_data.throttle[throt];
	struct soctherm_throttle_dev *dev = &data->devs[THROTTLE_DEV_GPU];

	if (!dev->enable)
		return false;

	if (IS_T12X || IS_T13X)
		return throttlectl_gpu_gk20a_nv_therm_style(dev, throt);

	if (dev->depth)
		THROT_DEPTH(dev, dev->depth);

	r = soctherm_readl(THROT_PSKIP_CTRL(throt, THROTTLE_DEV_GPU));
	r = REG_SET(r, THROT_PSKIP_CTRL_ENABLE, dev->enable);
	r = REG_SET(r, THROT_PSKIP_CTRL_DIVIDEND, dev->dividend);
	r = REG_SET(r, THROT_PSKIP_CTRL_DIVISOR, dev->divisor);
	soctherm_writel(r, THROT_PSKIP_CTRL(throt, THROTTLE_DEV_GPU));

	r = soctherm_readl(THROT_PSKIP_RAMP(throt, THROTTLE_DEV_GPU));
	r = REG_SET(r, THROT_PSKIP_RAMP_DURATION, dev->duration);
	r = REG_SET(r, THROT_PSKIP_RAMP_STEP, dev->step);
	soctherm_writel(r, THROT_PSKIP_RAMP(throt, THROTTLE_DEV_GPU));

	return true;
}

/**
 * soctherm_throttle_program() - programs pulse skippers' configuration
 * @throt	soctherm_throttle_id describing the level of throttling
 *
 * Pulse skippers are used to throttle clock frequencies.
 * This function programs the pulse skippers based on @throt and platform data.
 *
 * Return: Nothing is returned (void).
 */
static void soctherm_throttle_program(enum soctherm_throttle_id throt)
{
	u32 r;
	bool throt_enable;
	struct soctherm_throttle *data = &plat_data.throttle[throt];

	throt_enable = (IS_T13X) ? throttlectl_cpu_level(throt) :
		throttlectl_cpu_mn(throt);
	throt_enable |= throttlectl_gpu(throt);

	r = REG_SET(0, THROT_PRIORITY_LITE_PRIO, data->priority);
	soctherm_writel(r, THROT_PRIORITY_CTRL(throt));

	r = REG_SET(0, THROT_DELAY_LITE_DELAY, 0);
	soctherm_writel(r, THROT_DELAY_CTRL(throt));

	r = soctherm_readl(THROT_PRIORITY_LOCK);
	if (r < data->priority) {
		r = REG_SET(0, THROT_PRIORITY_LOCK_PRIORITY, data->priority);
		soctherm_writel(r, THROT_PRIORITY_LOCK);
	}

	/* ----- configure reserved OC5 alarm ----- */
	if (throt == THROTTLE_OC5) {
		r = soctherm_readl(ALARM_CFG(throt));
		r = REG_SET(r, OC1_CFG_THROTTLE_MODE, BRIEF);
		r = REG_SET(r, OC1_CFG_ALARM_POLARITY, 0);
		r = REG_SET(r, OC1_CFG_EN_THROTTLE, 1);
		soctherm_writel(r, ALARM_CFG(throt));

		r = REG_SET(r, OC1_CFG_ALARM_POLARITY, 1);
		soctherm_writel(r, ALARM_CFG(throt));

		r = REG_SET(r, OC1_CFG_ALARM_POLARITY, 0);
		soctherm_writel(r, ALARM_CFG(throt));

		r = REG_SET(r, THROT_PSKIP_CTRL_DIVIDEND, 0);
		r = REG_SET(r, THROT_PSKIP_CTRL_DIVISOR, 0);
		r = REG_SET(r, THROT_PSKIP_CTRL_ENABLE, 1);
		soctherm_writel(r, THROT_PSKIP_CTRL(throt, THROTTLE_DEV_CPU));

		r = soctherm_readl(THROT_PSKIP_RAMP(throt, THROTTLE_DEV_CPU));
		r = REG_SET(r, THROT_PSKIP_RAMP_DURATION, 0xff);
		r = REG_SET(r, THROT_PSKIP_RAMP_STEP, 0xf);
		soctherm_writel(r, THROT_PSKIP_RAMP(throt, THROTTLE_DEV_CPU));

		r = soctherm_readl(THROT_PSKIP_CTRL(throt, THROTTLE_DEV_GPU));
		r = REG_SET(r, THROT_PSKIP_CTRL_ENABLE, 1);
		soctherm_writel(r, THROT_PSKIP_CTRL(throt, THROTTLE_DEV_GPU));

		r = REG_SET(0, THROT_PRIORITY_LITE_PRIO, 1);
		soctherm_writel(r, THROT_PRIORITY_CTRL(throt));
		return;
	}

	if (!throt_enable || (throt < THROTTLE_OC1))
		return;

	/* ----- configure other OC alarms ----- */
	if (!(data->throt_mode == BRIEF || data->throt_mode == STICKY))
		pr_warn("soctherm: Invalid throt_mode in %s\n",
			throt_names[throt]);

	r = soctherm_readl(ALARM_CFG(throt));
	r = REG_SET(r, OC1_CFG_HW_RESTORE, 1);
	r = REG_SET(r, OC1_CFG_PWR_GOOD_MASK, data->pgmask);
	r = REG_SET(r, OC1_CFG_THROTTLE_MODE, data->throt_mode);
	r = REG_SET(r, OC1_CFG_ALARM_POLARITY, data->polarity);
	r = REG_SET(r, OC1_CFG_EN_THROTTLE, 1);
	soctherm_writel(r, ALARM_CFG(throt));

	soctherm_oc_intr_enable(throt, data->intr);

	soctherm_writel(data->period, ALARM_THROTTLE_PERIOD(throt)); /* usec */
	soctherm_writel(data->alarm_cnt_threshold, ALARM_CNT_THRESHOLD(throt));
	if (data->alarm_filter)
		soctherm_writel(data->alarm_filter, ALARM_FILTER(throt));
	else
		soctherm_writel(0xffffffff, ALARM_FILTER(throt));
}

/**
 * soctherm_tsense_program() - Configure sensor timing parameters based on
 * chip-specific data.
 *
 * @sensor:	The temperature sensor. This corresponds to one of the
 *		four CPU sensors, one of the two memory
 *		sensors, or the GPU or PLLX sensor.
 * @data:	Information regarding a sensor. This comes from the platform
 *		data
 *
 * This function is called during initialization. It sets two CPU thermal sensor
 * configuration registers (TS_CPU0_CONFIG0 and TS_CPU0_CONFIG1)
 * to contain the given chip-specific sensor's configuration data.
 *
 * The configuration data affects the sensor's temperature capturing.
 *
 * Return: Nothing is returned (void).
 */
static void soctherm_tsense_program(enum soctherm_sense sensor,
						struct soctherm_sensor *data)
{
	u32 r;

	r = REG_SET(0, TS_CPU0_CONFIG0_TALL, data->tall);
	soctherm_writel(r, TS_TSENSE_REG_OFFSET(TS_CPU0_CONFIG0, sensor));

	r = REG_SET(0, TS_CPU0_CONFIG1_TIDDQ, data->tiddq);
	r = REG_SET(r, TS_CPU0_CONFIG1_EN, 1);
	r = REG_SET(r, TS_CPU0_CONFIG1_TEN_COUNT, data->ten_count);
	r = REG_SET(r, TS_CPU0_CONFIG1_TSAMPLE, data->tsample - 1);
	soctherm_writel(r, TS_TSENSE_REG_OFFSET(TS_CPU0_CONFIG1, sensor));
}

/**
 * soctherm_clk_init() - Initialize SOC_THERM related clocks.
 *
 * The initialization will map clock aliases for SOC_THERM and TSENSE
 * and set their clock rates based on chip-specific defaults or
 * any platform-specific overrides.
 *
 * Return: 0 if successful else %-EINVAL if initialization failed
 */
static int __init soctherm_clk_init(void)
{
	unsigned long default_soctherm_clk_rate;
	unsigned long default_tsensor_clk_rate;

	soctherm_clk = clk_get_sys("soc_therm", NULL);
	tsensor_clk = clk_get_sys("tegra-tsensor", NULL);

	if (IS_ERR(tsensor_clk) || IS_ERR(soctherm_clk)) {
		clk_put(soctherm_clk);
		clk_put(tsensor_clk);
		soctherm_clk = NULL;
		tsensor_clk = NULL;
		return -EINVAL;
	}

	/* initialize default clock rates */
	if (IS_T11X) {
		default_soctherm_clk_rate = default_t11x_soctherm_clk_rate;
		default_tsensor_clk_rate = default_t11x_tsensor_clk_rate;
	} else if (IS_T14X) {
		default_soctherm_clk_rate = default_t14x_soctherm_clk_rate;
		default_tsensor_clk_rate = default_t14x_tsensor_clk_rate;
	} else if ((IS_T12X || IS_T13X)) {
		default_soctherm_clk_rate = default_t12x_soctherm_clk_rate;
		default_tsensor_clk_rate = default_t12x_tsensor_clk_rate;
	} else {
		BUG();
	}

	plat_data.soctherm_clk_rate =
		plat_data.soctherm_clk_rate ?: default_soctherm_clk_rate;
	plat_data.tsensor_clk_rate =
		plat_data.tsensor_clk_rate ?: default_tsensor_clk_rate;

	if (clk_get_rate(soctherm_clk) != plat_data.soctherm_clk_rate)
		if (clk_set_rate(soctherm_clk, plat_data.soctherm_clk_rate))
			return -EINVAL;

	if (clk_get_rate(tsensor_clk) != plat_data.tsensor_clk_rate)
		if (clk_set_rate(tsensor_clk, plat_data.tsensor_clk_rate))
			return -EINVAL;

	return 0;
}

/**
 * soctherm_clk_enable() - enables and disables the clocks
 * @enable:	whether the clocks should be enabled or disabled
 *
 * Enables the SOC_THERM and thermal sensor clocks when SOC_THERM
 * is initialized.
 *
 * Return: 0 if successful, %-EINVAL if either clock hasn't been initialized.
 */
static int soctherm_clk_enable(bool enable)
{
	if (soctherm_clk == NULL || tsensor_clk == NULL)
		return -EINVAL;

	if (enable) {
		clk_enable(soctherm_clk);
		clk_enable(tsensor_clk);
	} else {
		clk_disable(soctherm_clk);
		clk_disable(tsensor_clk);
	}

	return 0;
}

/**
 * soctherm_fuse_read_calib_base() - Calculates calibration base temperature
 *
 * Calculates the nominal temperature used for thermal sensor calibration
 * based on chip type and the value in fuses.
 *
 * Return: 0 (success), otherwise -EINVAL.
 */
static int soctherm_fuse_read_calib_base(void)
{
	s32 calib_cp, calib_ft;
	s32 nominal_calib_cp, nominal_calib_ft;

	if (tegra_fuse_calib_base_get_cp(&fuse_calib_base_cp, &calib_cp) < 0 ||
	    tegra_fuse_calib_base_get_ft(&fuse_calib_base_ft, &calib_ft) < 0) {
		pr_err("soctherm: ERROR: Improper CP or FT calib fuse.\n");
		return -EINVAL;
	}

	nominal_calib_cp = 25;
	if (IS_T11X)
		nominal_calib_ft = 90;
	else if (IS_T14X || IS_T12X || IS_T13X)
		nominal_calib_ft = 105;
	else
		BUG();

	/* use HI precision to calculate: use fuse_temp in 0.5C */
	actual_temp_cp = 2 * nominal_calib_cp + calib_cp;
	actual_temp_ft = 2 * nominal_calib_ft + calib_ft;

	return 0;
}

static struct soctherm_fuse_correction_war no_fuse_war[] = {
	[TSENSE_CPU0] = { 1000000, 0 },
	[TSENSE_CPU1] = { 1000000, 0 },
	[TSENSE_CPU2] = { 1000000, 0 },
	[TSENSE_CPU3] = { 1000000, 0 },
	[TSENSE_MEM0] = { 1000000, 0 },
	[TSENSE_MEM1] = { 1000000, 0 },
	[TSENSE_GPU]  = { 1000000, 0 },
	[TSENSE_PLLX] = { 1000000, 0 },
};

static struct soctherm_fuse_correction_war t11x_fuse_war[] = {
	[TSENSE_CPU0] = { 1196400, -13600000 },
	[TSENSE_CPU1] = { 1196400, -13600000 },
	[TSENSE_CPU2] = { 1196400, -13600000 },
	[TSENSE_CPU3] = { 1196400, -13600000 },
	[TSENSE_MEM0] = { 1000000,  -1000000 },
	[TSENSE_MEM1] = { 1000000,  -1000000 },
	[TSENSE_GPU]  = { 1124500,  -9793100 },
	[TSENSE_PLLX] = { 1224200, -14665000 },
};

static struct soctherm_fuse_correction_war t14x_fuse_war[] = {
	[TSENSE_CPU0] = { 1149000, -16753000 },
	[TSENSE_CPU1] = { 1148800, -16287000 },
	[TSENSE_CPU2] = { 1139100, -12552000 },
	[TSENSE_CPU3] = { 1141800, -11061000 },
	[TSENSE_MEM0] = { 1082300, -11061000 },
	[TSENSE_MEM1] = { 1061800,  -7596500 },
	[TSENSE_GPU]  = { 1078900, -10480000 },
	[TSENSE_PLLX] = { 1125900, -14736000 },
};

/* old CP/FT */
static struct soctherm_fuse_correction_war t12x_fuse_war1[] = {
	[TSENSE_CPU0] = { 1148300, -6572300 },
	[TSENSE_CPU1] = { 1126100, -5794600 },
	[TSENSE_CPU2] = { 1155800, -7462800 },
	[TSENSE_CPU3] = { 1134900, -6810800 },
	[TSENSE_MEM0] = { 1062700, -4463200 },
	[TSENSE_MEM1] = { 1084700, -5603400 },
	[TSENSE_GPU]  = { 1084300, -5111900 },
	[TSENSE_PLLX] = { 1134500, -7410700 },
};

/* new CP1/CP2 */
static struct soctherm_fuse_correction_war t12x_fuse_war2[] = {
	[TSENSE_CPU0] = { 1135400, -6266900 },
	[TSENSE_CPU1] = { 1122220, -5700700 },
	[TSENSE_CPU2] = { 1127000, -6768200 },
	[TSENSE_CPU3] = { 1110900, -6232000 },
	[TSENSE_MEM0] = { 1122300, -5936400 },
	[TSENSE_MEM1] = { 1145700, -7124600 },
	[TSENSE_GPU]  = { 1120100, -6000500 },
	[TSENSE_PLLX] = { 1106500, -6729300 },
};

/* old ATE pattern */
static struct soctherm_fuse_correction_war t13x_fuse_war1[] = {
	[TSENSE_CPU0] = { 1119800,  -6330400 },
	[TSENSE_CPU1] = { 1094100,  -3751800 },
	[TSENSE_CPU2] = { 1108800,  -3835200 },
	[TSENSE_CPU3] = { 1103200,  -5132100 },
	[TSENSE_MEM0] = { 1168400, -11266000 },
	[TSENSE_MEM1] = { 1185600, -10861000 },
	[TSENSE_GPU]  = { 1158500, -10714000 },
	[TSENSE_PLLX] = { 1150000, -11899000 },
};

/* new ATE pattern */
static struct soctherm_fuse_correction_war t13x_fuse_war2[] = {
	[TSENSE_CPU0] = { 1126600, -9433500 },
	[TSENSE_CPU1] = { 1110800, -7383000 },
	[TSENSE_CPU2] = { 1113800, -6215200 },
	[TSENSE_CPU3] = { 1129600, -8196100 },
	[TSENSE_MEM0] = { 1132900, -6755300 },
	[TSENSE_MEM1] = { 1142300, -7374200 },
	[TSENSE_GPU]  = { 1125100, -6350400 },
	[TSENSE_PLLX] = { 1118100, -8208800 },
};

/**
 * soctherm_fuse_read_tsensor() - calculates therm_a and therm_b for a sensor
 * @sensor:	The sensor for which to calculate.
 *
 * Reads the calibration data from the thermal sensor's fuses and then uses
 * that data to calculate the slope of the pulse/temperature
 * relationship, therm_a, and its x-intercept, therm_b. After correcting the
 * values based on their chip ID and whether precision is high or low, it
 * stores them in the sensor's registers so that the hardware can convert the
 * raw TSOSC reading into temperature in celsius.
 *
 * Return: 0 if successful, otherwise %-EINVAL
 */
static int soctherm_fuse_read_tsensor(enum soctherm_sense sensor)
{
	u32 r, value;
	s32 calib, delta_sens, delta_temp;
	s16 therm_a, therm_b;
	s32 div, mult, actual_tsensor_ft, actual_tsensor_cp;
	int fuse_rev;
	struct soctherm_fuse_correction_war *war;

	fuse_rev = tegra_fuse_calib_base_get_cp(NULL, NULL);
	if (fuse_rev < 0)
		return fuse_rev;

	tegra_fuse_get_tsensor_calib(sensor2tsensorcalib[sensor], &value);

	/* Extract bits and convert to signed 2's complement */
	calib = REG_GET(value, FUSE_TSENSOR_CALIB_FT);
	calib = MAKE_SIGNED32(calib, FUSE_TSENSOR_CALIB_BITS);
	actual_tsensor_ft = (fuse_calib_base_ft * 32) + calib;

	calib = REG_GET(value, FUSE_TSENSOR_CALIB_CP);
	calib = MAKE_SIGNED32(calib, FUSE_TSENSOR_CALIB_BITS);
	actual_tsensor_cp = (fuse_calib_base_cp * 64) + calib;

	mult = plat_data.sensor_data[sensor].pdiv *
		plat_data.sensor_data[sensor].tsamp_ate;
	div = plat_data.sensor_data[sensor].tsample *
		plat_data.sensor_data[sensor].pdiv_ate;

	/* first calculate therm_a and therm_b in Hi precision */
	delta_sens = actual_tsensor_ft - actual_tsensor_cp;
	delta_temp = actual_temp_ft - actual_temp_cp;

	therm_a = div64_s64_precise((s64)delta_temp * (1LL << 13) * mult,
				    (s64)delta_sens * div);

	therm_b = div64_s64_precise((((s64)actual_tsensor_ft * actual_temp_cp) -
				     ((s64)actual_tsensor_cp * actual_temp_ft)),
				    (s64)delta_sens);

	/* FUSE correction WARs */
	if (IS_T11X)
		war = PRECISION_IS_LOWER() ?
			&t11x_fuse_war[sensor] : &no_fuse_war[sensor];
	else if (IS_T14X)
		war = PRECISION_IS_LOWER() ?
			&t14x_fuse_war[sensor] : &no_fuse_war[sensor];
	else if (IS_T12X)
		war = fuse_rev ?
			&t12x_fuse_war1[sensor] : &t12x_fuse_war2[sensor];
	else if (IS_T13X)
		war = fuse_rev == 2 ?
			&t13x_fuse_war1[sensor] : &t13x_fuse_war2[sensor];
	else
		war = &no_fuse_war[sensor];

	therm_a = div64_s64_precise((s64)therm_a * war->a,
				    (s64)1000000LL);
	therm_b = div64_s64_precise((s64)therm_b * war->a + war->b,
				    (s64)1000000LL);
	therm_a = LOWER_PRECISION_FOR_TEMP(therm_a);
	therm_b = LOWER_PRECISION_FOR_TEMP(therm_b);

	sensor2therm_a[sensor] = (s16)therm_a;
	sensor2therm_b[sensor] = (s16)therm_b;

	r = REG_SET(0, TS_CPU0_CONFIG2_THERM_A, therm_a);
	r = REG_SET(r, TS_CPU0_CONFIG2_THERM_B, therm_b);
	soctherm_writel(r, TS_TSENSE_REG_OFFSET(TS_CPU0_CONFIG2, sensor));

	return 0;
}

/**
 * soctherm_therm_trip_init() - configure PMC's thermal-shutdown behavior
 * @data:	Power management unit thermal sensor initialization data
 *
 * Takes a given set of data and writes it to SCRATCH54 and SCRATCH55, which
 * PMC will use if SOC_THERM requests a shutdown based on excessive
 * temperature (i.e. a thermtrip).
 */
static void soctherm_therm_trip_init(struct tegra_thermtrip_pmic_data *data)
{
	if (!data)
		return;

	tegra_pmc_enable_thermal_trip();
	tegra_pmc_config_thermal_trip(data);
}

/**
 * soctherm_adjust_cpu_zone() - Adjusts the soctherm CPU zone
 * @therm:	soctherm_therm_id specifying the sensor group to adjust
 *
 * Changes SOC_THERM registers based on the CPU and PLLX temperatures.
 * Programs hotspot offsets per CPU or GPU and PLLX difference of temperature,
 * stops or starts CPUn TSOSCs, and programs hotspot offsets per configuration.
 * This function is called in soctherm_init_platform_data(),
 * tegra_soctherm_adjust_cpu_zone() and tegra_soctherm_adjust_core_zone().
 */
static void soctherm_adjust_zone(int tz)
{
	u32 r, s;
	int i;
	long ztemp, pll_temp, diff;
	bool low_voltage;

	if (soctherm_suspended)
		return;

	if (tz == THERM_CPU)
		low_voltage = vdd_cpu_low_voltage;
	else if (tz == THERM_GPU)
		low_voltage = vdd_core_low_voltage;
	else if (tz == THERM_MEM)
		low_voltage = vdd_core_low_voltage;
	else
		return;

	if (low_voltage) {
		r = soctherm_readl(TS_TEMP1);
		s = soctherm_readl(TS_TEMP2);

		/* get pllx temp */
		pll_temp = temp_translate(REG_GET(s, TS_TEMP2_PLLX_TEMP));
		ztemp = pll_temp; /* initialized */

		/* get therm-zone temp */
		if (tz == THERM_CPU)
			ztemp = temp_translate(REG_GET(r, TS_TEMP1_CPU_TEMP));
		else if (tz == THERM_GPU)
			ztemp = temp_translate(REG_GET(r, TS_TEMP1_GPU_TEMP));
		else if (tz == THERM_MEM)
			ztemp = temp_translate(REG_GET(s, TS_TEMP2_MEM_TEMP));

		if (ztemp > pll_temp)
			diff = ztemp - pll_temp;
		else
			diff = 0;

		/* cap hotspot offset to max offset from pdata */
		if (diff > plat_data.therm[tz].hotspot_offset)
			diff = plat_data.therm[tz].hotspot_offset;

		/* Program hotspot offsets per <tz> ~ PLL diff */
		r = soctherm_readl(TS_HOTSPOT_OFF);
		if (tz == THERM_CPU)
			r = REG_SET(r, TS_HOTSPOT_OFF_CPU, diff / 1000);
		else if (tz == THERM_GPU)
			r = REG_SET(r, TS_HOTSPOT_OFF_GPU, diff / 1000);
		else if (tz == THERM_MEM)
			r = REG_SET(r, TS_HOTSPOT_OFF_MEM, diff / 1000);
		soctherm_writel(r, TS_HOTSPOT_OFF);

		/* Stop all TSENSE's mapped to <tz> */
		for (i = 0; i < TSENSE_SIZE; i++) {
			if (tsensor2therm_map[i] != tz)
				continue;
			r = soctherm_readl(TS_TSENSE_REG_OFFSET
						(TS_CPU0_CONFIG0, i));
			r = REG_SET(r, TS_CPU0_CONFIG0_STOP, 1);
			soctherm_writel(r, TS_TSENSE_REG_OFFSET
						(TS_CPU0_CONFIG0, i));
		}
	} else {
		/* UN-Stop all TSENSE's mapped to <tz> */
		for (i = 0; i < TSENSE_SIZE; i++) {
			if (tsensor2therm_map[i] != tz)
				continue;
			r = soctherm_readl(TS_TSENSE_REG_OFFSET
						(TS_CPU0_CONFIG0, i));
			r = REG_SET(r, TS_CPU0_CONFIG0_STOP, 0);
			soctherm_writel(r, TS_TSENSE_REG_OFFSET
						(TS_CPU0_CONFIG0, i));
		}

		/* default to configured offset for <tz> */
		diff = plat_data.therm[tz].hotspot_offset;

		/* Program hotspot offsets per config */
		r = soctherm_readl(TS_HOTSPOT_OFF);
		if (tz == THERM_CPU)
			r = REG_SET(r, TS_HOTSPOT_OFF_CPU, diff / 1000);
		else if (tz == THERM_GPU)
			r = REG_SET(r, TS_HOTSPOT_OFF_GPU, diff / 1000);
		else if (tz == THERM_MEM)
			r = REG_SET(r, TS_HOTSPOT_OFF_MEM, diff / 1000);
		soctherm_writel(r, TS_HOTSPOT_OFF);
	}
}

/**
 * soctherm_init_platform_data() - Initializes the platform data.
 *
 * Cleans up some platform data in preparation for configuring the
 * hardware and configures the hardware as specified by the cleaned up
 * platform data.
 *
 * Initializes unset parameters for CPU, GPU, MEM, and PLL
 * based on the default values for the sensors on the Tegra chip in use.
 *
 * Sets the temperature sensor PDIV (post divider) register which
 * contains the temperature sensor PDIV for the CPU, GPU, MEM, and PLLX
 *
 * Sets the configurations for each of the sensors.
 *
 * Sanitizes thermal trips for each thermal zone.
 *
 * Writes hotspot offsets to TS_HOTSPOT_OFF register.
 *
 * Checks the throttling priorities and makes sure that they are
 * in the correct order for throttle types THROTTLE_OC1 though THROTTLE_OC4.
 *
 * Initializes PSKIP parameters. These parameters are used during a thermal
 * trip to calculate the amount of throttling of the CPU or GPU for each
 * thermal trip type (i.e. THROTTLE_LIGHT or THROTTLE_HEAVY)
 *
 * Initializes the throttling thresholds for the CPU, GPU, MEM, and PLL
 *
 * Checks if the priorities of heavy and light throttling are in
 * the correct order.
 *
 * Initializes the STATS_CTL and OC_STATS_CTL registers for stat collection
 *
 * Enables PMC shutdown based on the platform data
 *
 * Programs the temperatures at which hardware shutdowns occur.
 *
 * soctherm_init_platform_data ensures that the system will function as
 * expected when it resumes from suspended state or on initial start up.
 *
 * Return: 0 on success. -EINVAL is returned otherwise
 */
static int soctherm_init_platform_data(void)
{
	struct soctherm_therm *therm;
	struct soctherm_sensor *s;
	struct soctherm_sensor sensor_defaults;
	int i, j, k;
	long rem;
	long gsh = MAX_HIGH_TEMP;
	u32 r;

	if (IS_T11X)
		sensor_defaults = default_t11x_sensor_params;
	else if (IS_T14X)
		sensor_defaults = default_t14x_sensor_params;
	else if ((IS_T12X || IS_T13X))
		sensor_defaults = default_t12x_sensor_params;
	else
		BUG();

	/* initialize default values for unspecified params */
	for (i = 0; i < TSENSE_SIZE; i++) {
		therm = &plat_data.therm[tsensor2therm_map[i]];
		s = &plat_data.sensor_data[i];
		s->sensor_enable = s->sensor_enable ?: therm->zone_enable;
		s->tall      = s->tall      ?: sensor_defaults.tall;
		s->tiddq     = s->tiddq     ?: sensor_defaults.tiddq;
		s->ten_count = s->ten_count ?: sensor_defaults.ten_count;
		s->tsample   = s->tsample   ?: sensor_defaults.tsample;
		s->tsamp_ate = s->tsamp_ate ?: sensor_defaults.tsamp_ate;
		s->pdiv      = s->pdiv      ?: sensor_defaults.pdiv;
		s->pdiv_ate  = s->pdiv_ate  ?: sensor_defaults.pdiv_ate;
	}

	/* Pdiv */
	r = soctherm_readl(TS_PDIV);
	r = REG_SET(r, TS_PDIV_CPU, plat_data.sensor_data[TSENSE_CPU0].pdiv);
	r = REG_SET(r, TS_PDIV_GPU, plat_data.sensor_data[TSENSE_GPU].pdiv);
	r = REG_SET(r, TS_PDIV_MEM, plat_data.sensor_data[TSENSE_MEM0].pdiv);
	r = REG_SET(r, TS_PDIV_PLLX, plat_data.sensor_data[TSENSE_PLLX].pdiv);
	soctherm_writel(r, TS_PDIV);

	/* Thermal Sensing programming */
	if (soctherm_fuse_read_calib_base() < 0)
		return -EINVAL;
	for (i = 0; i < TSENSE_SIZE; i++) {
		if (plat_data.sensor_data[i].sensor_enable) {
			soctherm_tsense_program(i, &plat_data.sensor_data[i]);
			if (soctherm_fuse_read_tsensor(i) < 0)
				return -EINVAL;
		}
	}

	soctherm_adjust_zone(THERM_CPU);
	soctherm_adjust_zone(THERM_GPU);
	soctherm_adjust_zone(THERM_MEM);

	/* Sanitize therm trips */
	for (i = 0; i < THERM_SIZE; i++) {
		therm = &plat_data.therm[i];
		if (!therm->zone_enable)
			continue;

		for (j = 0; j < therm->num_trips; j++) {
			rem = therm->trips[j].trip_temp %
				LOWER_PRECISION_FOR_CONV(1000);
			if (rem) {
				pr_warn(
			"soctherm: zone%d/trip_point%d %ld mC rounded down\n",
					i, j, therm->trips[j].trip_temp);
				therm->trips[j].trip_temp -= rem;
			}
		}
	}

	/* Program hotspot offsets per THERM */
	r = REG_SET(0, TS_HOTSPOT_OFF_CPU,
		    plat_data.therm[THERM_CPU].hotspot_offset / 1000);
	r = REG_SET(r, TS_HOTSPOT_OFF_GPU,
		    plat_data.therm[THERM_GPU].hotspot_offset / 1000);
	r = REG_SET(r, TS_HOTSPOT_OFF_MEM,
		    plat_data.therm[THERM_MEM].hotspot_offset / 1000);
	soctherm_writel(r, TS_HOTSPOT_OFF);

	/* Thermal HW throttle programming */
	for (i = 0; i < THROTTLE_SIZE; i++) {
		/* Sanitize HW throttle priority for OC1 - OC4 (not OC5) */
		if ((i != THROTTLE_OC5) && (!plat_data.throttle[i].priority))
			plat_data.throttle[i].priority = 0xE + i;

		/* Setup PSKIP parameters */
		soctherm_throttle_program(i);

		/* Setup throttle thresholds per THERM */
		for (j = 0; j < THERM_SIZE; j++) {
			if ((therm2dev[j] == THROTTLE_DEV_NONE) ||
			    (!plat_data.throttle[i].devs[therm2dev[j]].enable))
				continue;

			therm = &plat_data.therm[j];
			for (k = 0; k < therm->num_trips; k++)
				if ((therm->trips[k].trip_type ==
				     THERMAL_TRIP_HOT) &&
				    strnstr(therm->trips[k].cdev_type,
					    i == THROTTLE_HEAVY ? "heavy" :
					    "light", THERMAL_NAME_LENGTH))
					break;
			if (k < therm->num_trips && i <= THROTTLE_HEAVY)
				prog_hw_threshold(&therm->trips[k], j, i);
		}
	}

	r = REG_SET(0, THROT_GLOBAL_ENB, 1);
	if (IS_T13X)
		clk_reset13_writel(r, THROT13_GLOBAL_CFG);
	else
		soctherm_writel(r, THROT_GLOBAL_CFG);

	if (plat_data.throttle[THROTTLE_HEAVY].priority <
	    plat_data.throttle[THROTTLE_LIGHT].priority)
		pr_err("soctherm: ERROR: Priority of HEAVY less than LIGHT\n");

	/* initialize stats collection */
	r = STATS_CTL_CLR_DN | STATS_CTL_EN_DN |
		STATS_CTL_CLR_UP | STATS_CTL_EN_UP;
	soctherm_writel(r, STATS_CTL);
	soctherm_writel(OC_STATS_CTL_EN_ALL, OC_STATS_CTL);

	/* Enable PMC to shutdown */
	soctherm_therm_trip_init(plat_data.tshut_pmu_trip_data);

	r = clk_reset_readl(CAR_SUPER_CLK_DIVIDER_REGISTER());
	r = REG_SET(r, CDIVG_USE_THERM_CONTROLS, 1);
	clk_reset_writel(r, CAR_SUPER_CLK_DIVIDER_REGISTER());

	/* Thermtrip */
	for (i = 0; i < THERM_SIZE; i++) {
		therm = &plat_data.therm[i];
		if (!therm->zone_enable)
			continue;

		for (j = 0; j < therm->num_trips; j++) {
			if (therm->trips[j].trip_type != THERMAL_TRIP_CRITICAL)
				continue;
			if (i == THERM_GPU) {
				gsh = therm->trips[j].trip_temp;
			} else if ((i == THERM_MEM) &&
				   (gsh != MAX_HIGH_TEMP) &&
				   (therm->trips[j].trip_temp != gsh)) {
				pr_warn("soctherm: Force TRIP temp: MEM = GPU");
				therm->trips[j].trip_temp = gsh;
			}
			prog_hw_shutdown(&therm->trips[j], i);
		}
	}

	return 0;
}

/**
 * soctherm_suspend_locked() - suspends SOC_THERM IP block
 *
 * Note: This function should never be directly called because
 * it's not thread-safe. Instead, soctherm_suspend() should be called.
 * Performs SOC_THERM suspension. It will disable SOC_THERM device interrupts.
 * SOC_THERM will need to be reinitialized.
 *
 */
static void soctherm_suspend_locked(void)
{
	if (!soctherm_suspended) {
		soctherm_writel((u32)-1, TH_INTR_DISABLE);
		soctherm_writel((u32)-1, OC_INTR_DISABLE);
		disable_irq(INT_THERMAL);
		disable_irq(INT_EDP);
		soctherm_init_platform_done = false;
		soctherm_suspended = true;
		/* soctherm_clk_enable(false);*/
	}
}

/**
 * soctherm_suspend() - Suspends the SOC_THERM device
 *
 * Suspends SOC_THERM and prevents interrupts from occurring
 * and SOC_THERM from interrupting the CPU.
 *
 * Return: 0 on success.
 */
static int soctherm_suspend(void)
{
	mutex_lock(&soctherm_suspend_resume_lock);
	soctherm_suspend_locked();
	mutex_unlock(&soctherm_suspend_resume_lock);
	return 0;
}

/**
 * soctherm_resume_locked() - Resumes soctherm if it is suspended
 *
 * Enables device interrupt generation for thermal and EDP when soctherm
 * platform initialization is done.
 */
static void soctherm_resume_locked(void)
{
	if (soctherm_suspended) {
		/* soctherm_clk_enable(true);*/
		soctherm_suspended = false;
		soctherm_init_platform_data();
		soctherm_init_platform_done = true;
		soctherm_update();
		enable_irq(INT_THERMAL);
		enable_irq(INT_EDP);
	}
}

/**
 * soctherm_resume() - wrapper for soctherm_resume_locked()
 *
 * Grabs the soctherm_suspend_resume_lock and then resumes SOC_THERM by running
 * soctherm_resume_locked().
 *
 * Return: 0
 */
static int soctherm_resume(void)
{
	mutex_lock(&soctherm_suspend_resume_lock);
	soctherm_resume_locked();
	mutex_unlock(&soctherm_suspend_resume_lock);
	return 0;
}

/**
 * soctherm_sync() - Syncs soctherm
 *
 * If soctherm is suspended, reinitializes the SOC_THERM IP block registers
 * from the platform data and updates each zone. Otherwise only the
 * latter occurs.
 */
static int soctherm_sync(void)
{
	mutex_lock(&soctherm_suspend_resume_lock);

	if (soctherm_suspended) {
		soctherm_resume_locked();
		soctherm_suspend_locked();
	} else {
		soctherm_update();
	}

	mutex_unlock(&soctherm_suspend_resume_lock);
	return 0;
}
late_initcall_sync(soctherm_sync);

/**
 * soctherm_pm_suspend() - reacts to system PM suspend event
 * @nb:         pointer to notifier_block. Currently not being used
 * @event:      type of action (suspend/resume)
 * @data:       argument for callback, currently not being used
 *
 * Currently supports %PM_SUSPEND_PREPARE. Ignores %PM_POST_SUSPEND
 *
 * Return: %NOTIFY_OK
 */
static int soctherm_pm_suspend(struct notifier_block *nb,
				unsigned long event, void *data)
{
	if (event == PM_SUSPEND_PREPARE) {
		soctherm_suspend();
		pr_info("tegra_soctherm: suspended\n");
	}
	return NOTIFY_OK;
}

/**
 * soctherm_pm_resume() - reacts to system PM resume event
 * @nb:         pointer to notifier_block. Currently not being used
 * @event:      type of action (suspend/resume)
 * @data:       argument for callback, currently not being used
 *
 * Currently supports %PM_POST_SUSPEND. Ignores %PM_SUSPEND_PREPARE
 *
 * Return: %NOTIFY_OK
 */
static int soctherm_pm_resume(struct notifier_block *nb,
				unsigned long event, void *data)
{
	if (event == PM_POST_SUSPEND) {
		soctherm_resume();
		pr_info("tegra_soctherm: resumed\n");
	}
	return NOTIFY_OK;
}

static struct notifier_block soctherm_suspend_nb = {
	.notifier_call = soctherm_pm_suspend,
	.priority = -2,
};

static struct notifier_block soctherm_resume_nb = {
	.notifier_call = soctherm_pm_resume,
	.priority = 2,
};

/**
 * soctherm_oc_irq_lock() - locks the over-current interrupt request
 * @data:	Interrupt request data
 *
 * Looks up the chip data from @data and locks the mutex associated with
 * a particular over-current interrupt request.
 */
static void soctherm_oc_irq_lock(struct irq_data *data)
{
	struct soctherm_oc_irq_chip_data *d = irq_data_get_irq_chip_data(data);

	mutex_lock(&d->irq_lock);
}

/**
 * soctherm_oc_irq_sync_unlock() - Unlocks the OC interrupt request
 * @data:		Interrupt request data
 *
 * Looks up the interrupt request data @data and unlocks the mutex associated
 * with a particular over-current interrupt request.
 */
static void soctherm_oc_irq_sync_unlock(struct irq_data *data)
{
	struct soctherm_oc_irq_chip_data *d = irq_data_get_irq_chip_data(data);

	mutex_unlock(&d->irq_lock);
}

/**
 * soctherm_oc_irq_enable() - Enables the SOC_THERM over-current interrupt queue
 * @data:       irq_data structure of the chip
 *
 * Sets the irq_enable bit of SOC_THERM allowing SOC_THERM
 * to respond to over-current interrupts.
 *
 */
static void soctherm_oc_irq_enable(struct irq_data *data)
{
	struct soctherm_oc_irq_chip_data *d = irq_data_get_irq_chip_data(data);

	d->irq_enable |= BIT(data->hwirq);
}

/**
 * soctherm_oc_irq_disable() - Disables overcurrent interrupt requests
 * @irq_data:	The interrupt request information
 *
 * Clears the interrupt request enable bit of the overcurrent
 * interrupt request chip data.
 *
 * Return: Nothing is returned (void)
 */
static void soctherm_oc_irq_disable(struct irq_data *data)
{
	struct soctherm_oc_irq_chip_data *d = irq_data_get_irq_chip_data(data);

	d->irq_enable &= ~BIT(data->hwirq);
}

static int soctherm_oc_irq_set_type(struct irq_data *data, unsigned int type)
{
	return 0;
}

/**
 * soctherm_oc_irq_set_wake() - Set the overcurrent interrupt request
 * to "wake"
 * @irq_data:	Interrupt request information
 * @on:		Whether to enable or disable power management wakeup
 *
 * Configure the GPIO associated with a SOC_THERM over-current
 * interrupt to wake the system from sleep
 *
 * It may be necessary to wake the system from sleep mode so that
 * SOC_THERM can provide proper over-current throttling.
 *
 * Return: 0 on success, -EINVAL if there is no wakeup support
 * for that given hardware irq, or the gpio number if there is
 * no gpio_to_irq for that gpio.
 */
static int soctherm_oc_irq_set_wake(struct irq_data *data, unsigned int on)
{
	int gpio;
	int gpio_irq;

	gpio = soctherm_ocx_to_wake_gpio[data->hwirq];
	if (!gpio_is_valid(gpio)) {
		pr_err("No wakeup supported for irq %lu\n", data->hwirq);
		return -EINVAL;
	}

	gpio_irq = gpio_to_irq(gpio);
	if (gpio_irq < 0) {
		pr_err("No gpio_to_irq for gpio %d\n", gpio);
		return gpio;
	}

	irq_set_irq_wake(gpio_irq, on);
	return 0;
}

/**
 * soctherm_oc_irq_map() - SOC_THERM interrupt request domain mapper
 * @h:		Interrupt request domain
 * @virq:	Virtual interrupt request number
 * @hw:		Hardware interrupt request number
 *
 * Mapping callback function for SOC_THERM's irq_domain. When a SOC_THERM
 * interrupt request is called, the irq_domain takes the request's virtual
 * request number (much like a virtual memory address) and maps it to a
 * physical hardware request number.
 *
 * When a mapping doesn't already exist for a virtual request number, the
 * irq_domain calls this function to associate the virtual request number with
 * a hardware request number.
 *
 * Return: 0
 */
static int soctherm_oc_irq_map(struct irq_domain *h, unsigned int virq,
		irq_hw_number_t hw)
{
	struct soctherm_oc_irq_chip_data *data = h->host_data;

	irq_set_chip_data(virq, data);
	irq_set_chip(virq, &data->irq_chip);
	irq_set_nested_thread(virq, 1);
	set_irq_flags(virq, IRQF_VALID);
	return 0;
}

static struct irq_domain_ops soctherm_oc_domain_ops = {
	.map	= soctherm_oc_irq_map,
	.xlate	= irq_domain_xlate_twocell,
};

/**
 * tegra11_soctherem_oc_int_init() - Initial enabling of the over
 * current interrupts
 * @irq_base:	The interrupt request base number from platform data
 * @num_irqs:	The number of new interrupt requests
 *
 * Sets the over current interrupt request chip data
 *
 * Return: 0 on success or if overcurrent interrupts are not enabled,
 * -ENOMEM (out of memory), or irq_base if the function failed to
 * allocate the irqs
 */
static int tegra11_soctherem_oc_int_init(int irq_base, int num_irqs)
{
	if (irq_base <= 0 || !num_irqs) {
		pr_info("%s(): OC interrupts are not enabled\n", __func__);
		return 0;
	}

	mutex_init(&soc_irq_cdata.irq_lock);
	soc_irq_cdata.irq_enable = 0;

	soc_irq_cdata.irq_chip.name = "soc_therm_oc";
	soc_irq_cdata.irq_chip.irq_bus_lock = soctherm_oc_irq_lock,
	soc_irq_cdata.irq_chip.irq_bus_sync_unlock =
		soctherm_oc_irq_sync_unlock,
	soc_irq_cdata.irq_chip.irq_disable = soctherm_oc_irq_disable,
	soc_irq_cdata.irq_chip.irq_enable = soctherm_oc_irq_enable,
	soc_irq_cdata.irq_chip.irq_set_type = soctherm_oc_irq_set_type,
	soc_irq_cdata.irq_chip.irq_set_wake = soctherm_oc_irq_set_wake,

	irq_base = irq_alloc_descs(irq_base, 0, num_irqs, 0);
	if (irq_base < 0) {
		pr_err("%s: Failed to allocate IRQs: %d\n", __func__, irq_base);
		return irq_base;
	}

	soc_irq_cdata.domain = irq_domain_add_legacy(NULL, num_irqs,
			irq_base, 0, &soctherm_oc_domain_ops, &soc_irq_cdata);
	if (!soc_irq_cdata.domain) {
		pr_err("%s: Failed to create IRQ domain\n", __func__);
		return -ENOMEM;
	}
	pr_info("%s(): OC interrupts enabled successful\n", __func__);
	return 0;
}

static int core_rail_regulator_notifier_cb(
	struct notifier_block *nb, unsigned long event, void *v)
{
	int uv = (int)((long)v);
	int rv = NOTIFY_DONE;
	int core_vmin_limit_uv;

	if (IS_T12X) {
		core_vmin_limit_uv = 900000;
		if (event & REGULATOR_EVENT_OUT_POSTCHANGE) {
			if (uv >= core_vmin_limit_uv) {
				tegra_soctherm_adjust_core_zone(true);
				rv = NOTIFY_OK;
			}
		} else if (event & REGULATOR_EVENT_OUT_PRECHANGE) {
			if (uv < core_vmin_limit_uv) {
				tegra_soctherm_adjust_core_zone(false);
				rv = NOTIFY_OK;
			}
		}
	}
	return rv;
}

static int __init soctherm_core_rail_notify_init(void)
{
	int ret;
	static struct notifier_block vmin_condition_nb;

	vmin_condition_nb.notifier_call = core_rail_regulator_notifier_cb;
	ret = tegra_dvfs_rail_register_notifier(tegra_core_rail,
						&vmin_condition_nb);
	if (ret) {
		pr_err("%s: Failed to register core rail notifier\n",
		       __func__);
		return ret;
	}

	return 0;
}
late_initcall_sync(soctherm_core_rail_notify_init);

/**
 * tegra11_soctherm_init() - initializes SOC_THERM IP Block
 * @data:       pointer to board-specific information
 *
 * Initialize and enable SOC_THERM clocks, sanitize platform data, configure
 * SOC_THERM according to platform data, and set up interrupt handling for
 * OC events.
 *
 * Return: -1 if initialization failed, 0 otherwise
 */
int __init tegra11_soctherm_init(struct soctherm_platform_data *data)
{
	int ret;

	tegra_chip_id = tegra_get_chip_id();
	if (!(IS_T11X || IS_T14X || IS_T12X || IS_T13X)) {
		pr_err("%s: Unknown chip_id %d", __func__, tegra_chip_id);
		return -1;
	}

	register_pm_notifier(&soctherm_suspend_nb);
	register_pm_notifier(&soctherm_resume_nb);

	if (!data)
		return -1;
	plat_data = *data;

	if (soctherm_clk_init() < 0)
		return -1;

	if (soctherm_clk_enable(true) < 0)
		return -1;

	if (soctherm_init_platform_data() < 0)
		return -1;

	soctherm_init_platform_done = true;

	ret = tegra11_soctherem_oc_int_init(data->oc_irq_base,
			data->num_oc_irqs);
	if (ret < 0) {
		pr_err("soctherem_oc_int_init failed: %d\n", ret);
		return ret;
	}

	if (request_threaded_irq(INT_THERMAL, soctherm_thermal_isr,
				 soctherm_thermal_thread_func, IRQF_ONESHOT,
				 "soctherm_thermal", NULL) < 0)
		return -1;

	if (request_threaded_irq(INT_EDP, soctherm_edp_isr,
				 soctherm_edp_thread_func, IRQF_ONESHOT,
				 "soctherm_edp", NULL) < 0)
		return -1;

	return 0;
}

/**
 * tegra_soctherm_adjust_cpu_zone() - Adjusts the CPU zone of Tegra soctherm
 * @high_voltage_range:		Flag indicating whether or not the system is
 *				within the highest voltage range
 *
 * If a particular VDD_CPU voltage threshold has been crossed (either up or
 * down), invokes soctherm_adjust_cpu_zone().
 * This function should be called by code outside this file when VDD_CPU crosses
 * a particular threshold.
 */
void tegra_soctherm_adjust_cpu_zone(bool high_voltage_range)
{
	if (!vdd_cpu_low_voltage != high_voltage_range) {
		vdd_cpu_low_voltage = !high_voltage_range;
		soctherm_adjust_zone(THERM_CPU);
	}
}

void tegra_soctherm_adjust_core_zone(bool high_voltage_range)
{
	if ((IS_T12X || IS_T13X)) {
		if (!vdd_core_low_voltage != high_voltage_range) {
			vdd_core_low_voltage = !high_voltage_range;
			soctherm_adjust_zone(THERM_GPU);
			soctherm_adjust_zone(THERM_MEM);
		}
	}
}

#ifdef CONFIG_DEBUG_FS

/**
 * regs_show() - show callback for regs debugfs
 * @s:          seq_file for registers values to be written to
 * @data:       a void pointer for callback, currently not being used
 *
 * Gathers various register values and system status, then
 * formats and display the information as a debugfs virtual file.
 * This function allows easy access to debugging information.
 *
 * Return: -1 if fail, 0 otherwise
 */
static int regs_show(struct seq_file *s, void *data)
{
	u32 r;
	u32 state;
	int tcpu[TSENSE_SIZE];
	int i, j, level;
	uint m, n, q;
	char *depth;

	seq_printf(s, "-----TSENSE (precision %s  fuse %d  convert %s)-----\n",
		   PRECISION_TO_STR(), tegra_fuse_calib_base_get_cp(NULL, NULL),
		   read_hw_temp ? "HW" : "SW");

	if (soctherm_suspended) {
		seq_puts(s, "SOC_THERM is SUSPENDED\n");
		return 0;
	}

	for (i = 0; i < TSENSE_SIZE; i++) {
		r = soctherm_readl(TS_TSENSE_REG_OFFSET(TS_CPU0_CONFIG1, i));
		state = REG_GET(r, TS_CPU0_CONFIG1_EN);
		if (!state)
			continue;

		seq_printf(s, "%s: ", sensor_names[i]);

		seq_printf(s, "En(%d) ", state);
		state = REG_GET(r, TS_CPU0_CONFIG1_TIDDQ);
		seq_printf(s, "tiddq(%d) ", state);
		state = REG_GET(r, TS_CPU0_CONFIG1_TEN_COUNT);
		seq_printf(s, "ten_count(%d) ", state);
		state = REG_GET(r, TS_CPU0_CONFIG1_TSAMPLE);
		seq_printf(s, "tsample(%d) ", state + 1);

		r = soctherm_readl(TS_TSENSE_REG_OFFSET(TS_CPU0_STATUS1, i));
		state = REG_GET(r, TS_CPU0_STATUS1_TEMP_VALID);
		seq_printf(s, "Temp(%d/", state);
		state = REG_GET(r, TS_CPU0_STATUS1_TEMP);
		seq_printf(s, "%d) ", tcpu[i] = temp_translate(state));

		r = soctherm_readl(TS_TSENSE_REG_OFFSET(TS_CPU0_STATUS0, i));
		state = REG_GET(r, TS_CPU0_STATUS0_VALID);
		seq_printf(s, "Capture(%d/", state);
		state = REG_GET(r, TS_CPU0_STATUS0_CAPTURE);
		seq_printf(s, "%d) (Converted-temp(%ld) ", state,
			   temp_convert(state, sensor2therm_a[i],
					sensor2therm_b[i]));

		r = soctherm_readl(TS_TSENSE_REG_OFFSET(TS_CPU0_CONFIG0, i));
		state = REG_GET(r, TS_CPU0_CONFIG0_STOP);
		seq_printf(s, "Stop(%d) ", state);
		state = REG_GET(r, TS_CPU0_CONFIG0_TALL);
		seq_printf(s, "Tall(%d) ", state);
		state = REG_GET(r, TS_CPU0_CONFIG0_TCALC_OVER);
		seq_printf(s, "Over(%d/", state);
		state = REG_GET(r, TS_CPU0_CONFIG0_OVER);
		seq_printf(s, "%d/", state);
		state = REG_GET(r, TS_CPU0_CONFIG0_CPTR_OVER);
		seq_printf(s, "%d) ", state);

		r = soctherm_readl(TS_TSENSE_REG_OFFSET(TS_CPU0_CONFIG2, i));
		state = REG_GET(r, TS_CPU0_CONFIG2_THERM_A);
		seq_printf(s, "Therm_A/B(%d/", state);
		state = REG_GET(r, TS_CPU0_CONFIG2_THERM_B);
		seq_printf(s, "%d)\n", (s16)state);
	}

	r = soctherm_readl(TS_PDIV);
	seq_printf(s, "PDIV: 0x%x\n", r);

	seq_puts(s, "\n");
	seq_puts(s, "-----SOC_THERM-----\n");

	r = soctherm_readl(TS_TEMP1);
	state = REG_GET(r, TS_TEMP1_CPU_TEMP);
	seq_printf(s, "Temperatures: CPU(%ld) ", temp_translate(state));
	state = REG_GET(r, TS_TEMP1_GPU_TEMP);
	seq_printf(s, " GPU(%ld) ", temp_translate(state));
	r = soctherm_readl(TS_TEMP2);
	state = REG_GET(r, TS_TEMP2_PLLX_TEMP);
	seq_printf(s, " PLLX(%ld) ", temp_translate(state));
	state = REG_GET(r, TS_TEMP2_MEM_TEMP);
	seq_printf(s, " MEM(%ld)\n", temp_translate(state));

	for (i = 0; i < THERM_SIZE; i++) {
		seq_printf(s, "%s:\n", therm_names[i]);
		for (level = 0; level < 4; level++) {
			r = soctherm_readl(TS_THERM_REG_OFFSET(CTL_LVL0_CPU0,
								level, i));
			state = REG_GET(r, CTL_LVL0_CPU0_UP_THRESH);
			seq_printf(s, "   %d: Up/Dn(%d/", level,
				   LOWER_PRECISION_FOR_CONV(state));
			state = REG_GET(r, CTL_LVL0_CPU0_DN_THRESH);
			seq_printf(s, "%d) ", LOWER_PRECISION_FOR_CONV(state));
			state = REG_GET(r, CTL_LVL0_CPU0_EN);
			seq_printf(s, "En(%d) ", state);

			state = REG_GET(r, CTL_LVL0_CPU0_CPU_THROT);
			seq_puts(s, "CPU Throt");
			seq_printf(s, "(%s) ", state ?
			state == CTL_LVL0_CPU0_CPU_THROT_LIGHT ? "L" :
			state == CTL_LVL0_CPU0_CPU_THROT_HEAVY ? "H" :
				"H+L" : "none");

			state = REG_GET(r, CTL_LVL0_CPU0_GPU_THROT);
			seq_puts(s, "GPU Throt");
			seq_printf(s, "(%s) ", state ?
			state == CTL_LVL0_CPU0_GPU_THROT_LIGHT ? "L" :
			state == CTL_LVL0_CPU0_GPU_THROT_HEAVY ? "H" :
				"H+L" : "none");

			state = REG_GET(r, CTL_LVL0_CPU0_STATUS);
			seq_printf(s, "Status(%s)\n",
				   state == 0 ? "LO" :
				   state == 1 ? "in" :
				   state == 2 ? "??" : "HI");
		}
	}

	r = soctherm_readl(STATS_CTL);
	seq_printf(s, "STATS: Up(%s) Dn(%s)\n",
		   r & STATS_CTL_EN_UP ? "En" : "--",
		   r & STATS_CTL_EN_DN ? "En" : "--");
	for (level = 0; level < 4; level++) {
		r = soctherm_readl(TS_TSENSE_REG_OFFSET(UP_STATS_L0, level));
		seq_printf(s, "  Level_%d Up(%d) ", level, r);
		r = soctherm_readl(TS_TSENSE_REG_OFFSET(DN_STATS_L0, level));
		seq_printf(s, "Dn(%d)\n", r);
	}

	r = soctherm_readl(THERMTRIP);
	state = REG_GET(r, THERMTRIP_ANY_EN);
	seq_printf(s, "ThermTRIP ANY En(%d)\n", state);

	state = REG_GET(r, THERMTRIP_CPU_EN);
	seq_printf(s, "     CPU En(%d) ", state);
	state = REG_GET(r, THERMTRIP_CPU_THRESH);
	seq_printf(s, "Thresh(%d)\n", LOWER_PRECISION_FOR_CONV(state));

	state = REG_GET(r, THERMTRIP_GPU_EN);
	seq_printf(s, "     GPU En(%d) ", state);
	state = REG_GET(r, THERMTRIP_GPUMEM_THRESH);
	seq_printf(s, "Thresh(%d)\n", LOWER_PRECISION_FOR_CONV(state));

	state = REG_GET(r, THERMTRIP_MEM_EN);
	seq_printf(s, "     MEM En(%d) ", state);
	state = REG_GET(r, THERMTRIP_GPUMEM_THRESH);
	seq_printf(s, "Thresh(%d)\n", LOWER_PRECISION_FOR_CONV(state));

	state = REG_GET(r, THERMTRIP_TSENSE_EN);
	seq_printf(s, "    PLLX En(%d) ", state);
	state = REG_GET(r, THERMTRIP_TSENSE_THRESH);
	seq_printf(s, "Thresh(%d)\n", LOWER_PRECISION_FOR_CONV(state));

	r = soctherm_readl(THROT_GLOBAL_CFG);
	seq_printf(s, "GLOBAL THROTTLE CONFIG: 0x%08x\n", r);

	seq_puts(s, "---------------------------------------------------\n");
	r = soctherm_readl(THROT_STATUS);
	state = REG_GET(r, THROT_STATUS_BREACH);
	seq_printf(s, "THROT STATUS: breach(%d) ", state);
	state = REG_GET(r, THROT_STATUS_STATE);
	seq_printf(s, "state(%d) ", state);
	state = REG_GET(r, THROT_STATUS_ENABLED);
	seq_printf(s, "enabled(%d)\n", state);

	r = soctherm_readl(CPU_PSKIP_STATUS);
	if (IS_T13X) {
		state = REG_GET(r, XPU_PSKIP_STATUS_ENABLED);
		seq_printf(s, "%s PSKIP STATUS: ",
			   throt_dev_names[THROTTLE_DEV_CPU]);
		seq_printf(s, "enabled(%d)\n", state);
	} else {
		state = REG_GET(r, XPU_PSKIP_STATUS_M);
		seq_printf(s, "%s PSKIP STATUS: M(%d) ",
			   throt_dev_names[THROTTLE_DEV_CPU], state);
		state = REG_GET(r, XPU_PSKIP_STATUS_N);
		seq_printf(s, "N(%d) ", state);
		state = REG_GET(r, XPU_PSKIP_STATUS_ENABLED);
		seq_printf(s, "enabled(%d)\n", state);
	}

	r = soctherm_readl(GPU_PSKIP_STATUS);
	if ((IS_T12X || IS_T13X)) {
		state = REG_GET(r, XPU_PSKIP_STATUS_ENABLED);
		seq_printf(s, "%s PSKIP STATUS: ",
			   throt_dev_names[THROTTLE_DEV_GPU]);
		seq_printf(s, "enabled(%d)\n", state);
	} else {
		state = REG_GET(r, XPU_PSKIP_STATUS_M);
		seq_printf(s, "%s PSKIP STATUS: M(%d) ",
			   throt_dev_names[THROTTLE_DEV_GPU], state);
		state = REG_GET(r, XPU_PSKIP_STATUS_N);
		seq_printf(s, "N(%d) ", state);
		state = REG_GET(r, XPU_PSKIP_STATUS_ENABLED);
		seq_printf(s, "enabled(%d)\n", state);
	}

	seq_puts(s, "---------------------------------------------------\n");
	seq_puts(s, "THROTTLE control and PSKIP configuration:\n");
	seq_printf(s, "%5s  %3s  %2s  %7s  %8s  %7s  %8s  %4s  %4s  %5s  ",
		   "throt", "dev", "en", " depth ", "dividend", "divisor",
		   "duration", "step", "prio", "delay");
	seq_printf(s, "%2s  %2s  %2s  %2s  %2s  %2s  ",
		   "LL", "HW", "PG", "MD", "01", "EN");
	seq_printf(s, "%8s  %8s  %8s  %8s  %8s\n",
		   "thresh", "period", "count", "filter", "stats");

	/* display throttle_cfg's of all alarms including OC5 */
	for (i = 0; i < THROTTLE_SIZE; i++) {
		for (j = 0; j < THROTTLE_DEV_SIZE; j++) {
			r = soctherm_readl(THROT_PSKIP_CTRL(i, j));
			state = REG_GET(r, THROT_PSKIP_CTRL_ENABLE);
			seq_printf(s, "%5s  %3s  %2d  ",
				   j ? "" : throt_names[i],
				   throt_dev_names[j], state);
			if (!state) {
				seq_puts(s, "\n");
				continue;
			}

			level = THROT_LEVEL_NONE; /* invalid */
			depth = "";
			q = 0;
			if (IS_T13X && j == THROTTLE_DEV_CPU) {
				state = REG_GET(r, THROT_PSKIP_CTRL_VECT_CPU);
				if (state == THROT_VECT_HVY) {
					level = THROT_LEVEL_HVY;
					depth = "hi";
				} else if (state == THROT_VECT_MED) {
					level = THROT_LEVEL_MED;
					depth = "med";
				} else if (state == THROT_VECT_LOW) {
					level = THROT_LEVEL_LOW;
					depth = "low";
				}
			}
			if ((IS_T12X || IS_T13X) && j == THROTTLE_DEV_GPU) {
				state = REG_GET(r, THROT_PSKIP_CTRL_VECT_GPU);
				/* Mapping is hard-coded in gk20a:nv_therm */
				if (state == THROT_VECT_HVY) {
					q = 87;
					depth = "hi";
				} else if (state == THROT_VECT_MED) {
					q = 75;
					depth = "med";
				} else if (state == THROT_VECT_LOW) {
					q = 50;
					depth = "low";
				}
			}

			if (level == THROT_LEVEL_NONE)
				r = 0;
			else if (IS_T13X && j == THROTTLE_DEV_CPU)
				r = clk_reset13_readl(
					THROT13_PSKIP_CTRL_CPU(level));
			else
				r = soctherm_readl(THROT_PSKIP_CTRL(i, j));

			m = REG_GET(r, THROT_PSKIP_CTRL_DIVIDEND);
			n = REG_GET(r, THROT_PSKIP_CTRL_DIVISOR);
			q = q ?: 100 - (((100 * (m+1)) + ((n+1) / 2)) / (n+1));
			seq_printf(s, "%2u%% %3s  ", q, depth);
			seq_printf(s, "%8u  ", m);
			seq_printf(s, "%7u  ", n);

			if (IS_T13X && j == THROTTLE_DEV_CPU)
				r = clk_reset13_readl(
					THROT13_PSKIP_RAMP_CPU(level));
			else
				r = soctherm_readl(THROT_PSKIP_RAMP(i, j));

			state = REG_GET(r, THROT_PSKIP_RAMP_DURATION);
			seq_printf(s, "%8d  ", state);
			state = REG_GET(r, THROT_PSKIP_RAMP_STEP);
			seq_printf(s, "%4d  ", state);

			r = soctherm_readl(THROT_PRIORITY_CTRL(i));
			state = REG_GET(r, THROT_PRIORITY_LITE_PRIO);
			seq_printf(s, "%4d  ", state);

			r = soctherm_readl(THROT_DELAY_CTRL(i));
			state = REG_GET(r, THROT_DELAY_LITE_DELAY);
			seq_printf(s, "%5d  ", state);

			if (i >= THROTTLE_OC1) {
				r = soctherm_readl(ALARM_CFG(i));
				state = REG_GET(r, OC1_CFG_LONG_LATENCY);
				seq_printf(s, "%2d  ", state);
				state = REG_GET(r, OC1_CFG_HW_RESTORE);
				seq_printf(s, "%2d  ", state);
				state = REG_GET(r, OC1_CFG_PWR_GOOD_MASK);
				seq_printf(s, "%2d  ", state);
				state = REG_GET(r, OC1_CFG_THROTTLE_MODE);
				seq_printf(s, "%2d  ", state);
				state = REG_GET(r, OC1_CFG_ALARM_POLARITY);
				seq_printf(s, "%2d  ", state);
				state = REG_GET(r, OC1_CFG_EN_THROTTLE);
				seq_printf(s, "%2d  ", state);

				r = soctherm_readl(ALARM_CNT_THRESHOLD(i));
				seq_printf(s, "%8d  ", r);
				r = soctherm_readl(ALARM_THROTTLE_PERIOD(i));
				seq_printf(s, "%8d  ", r);
				r = soctherm_readl(ALARM_ALARM_COUNT(i));
				seq_printf(s, "%8d  ", r);
				r = soctherm_readl(ALARM_FILTER(i));
				seq_printf(s, "%8d  ", r);
				r = soctherm_readl(ALARM_STATS(i));
				seq_printf(s, "%8d  ", r);
			}
			seq_puts(s, "\n");
		}
	}
	return 0;
}

/**
 * temp_log_show() - "show" callback for temp_log debugfs node
 * @s:		pointer to the seq_file record to write the log through
 * @data:	not used
 *
 * The temperature log contains the time (seconds.nanoseconds), and the
 * temperature of each of the thermal sensors, if there is a valid temperature
 * capture available. Makes sure that SOC_THERM is left in the same state in
 * which it was previously (suspended/resumed).
 *
 * Return: 0
 */
static int temp_log_show(struct seq_file *s, void *data)
{
	u32 r, state;
	int i;
	u64 ts;
	u_long ns;
	bool was_suspended = false;

	ts = cpu_clock(0);
	ns = do_div(ts, 1000000000);
	seq_printf(s, "%6lu.%06lu", (u_long) ts, ns / 1000);

	if (soctherm_suspended) {
		mutex_lock(&soctherm_suspend_resume_lock);
		soctherm_resume_locked();
		was_suspended = true;
	}

	for (i = 0; i < TSENSE_SIZE; i++) {
		r = soctherm_readl(TS_TSENSE_REG_OFFSET(
					TS_CPU0_CONFIG1, i));
		state = REG_GET(r, TS_CPU0_CONFIG1_EN);
		if (!state)
			continue;

		r = soctherm_readl(TS_TSENSE_REG_OFFSET(
					TS_CPU0_STATUS1, i));
		if (!REG_GET(r, TS_CPU0_STATUS1_TEMP_VALID)) {
			seq_puts(s, "\tINVALID");
			continue;
		}

		if (read_hw_temp) {
			state = REG_GET(r, TS_CPU0_STATUS1_TEMP);
			seq_printf(s, "\t%ld", temp_translate(state));
		} else {
			r = soctherm_readl(TS_TSENSE_REG_OFFSET(
						TS_CPU0_STATUS0, i));
			state = REG_GET(r, TS_CPU0_STATUS0_CAPTURE);
			seq_printf(s, "\t%ld",
				   temp_convert(state, sensor2therm_a[i],
						sensor2therm_b[i]));
		}
	}
	seq_puts(s, "\n");

	if (was_suspended) {
		soctherm_suspend_locked();
		mutex_unlock(&soctherm_suspend_resume_lock);
	}
	return 0;
}

/**
 * regs_open() - wraps single_open to associate internal regs_show()
 * @inode:      inode related to the file
 * @file:       pointer to a file to be manipulated with single_open
 *
 * Return: Passes along the return value from single_open().
 */
static int regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, regs_show, inode->i_private);
}

static const struct file_operations regs_fops = {
	.open		= regs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/**
 * convert_get() - indicates software or hardware temperature conversion
 * @data:       argument for callback, currently not being used
 * @val:        pointer to a u64 location to store flag value
 *
 * Stores boolean flag into memory address pointed to by val.
 * The flag indicates whether SOC_THERM is using
 * software or hardware temperature conversion.
 *
 * Return: 0
 */
static int convert_get(void *data, u64 *val)
{
	*val = !read_hw_temp;
	return 0;
}
/**
 * convert_set() - Sets a flag indicating which temperature is
 * being read.
 * @data:	Opaque pointer to data passed in from filesystem layer
 * @val:	The flag value
 *
 * Sets the read_hw_temp file static flag. This flag indicates whether
 * the hardware temperature or the software temperature is being
 * read.
 *
 * Return: 0 on success.
 */
static int convert_set(void *data, u64 val)
{
	read_hw_temp = !val;
	return 0;
}

/**
 * cputemp_get() - gets the CPU temperature.
 * @data:	not used
 * @val:	a pointer in which the temperature will be placed.
 *
 * Reads the temperature of the thermal sensor associated with the CPU.
 *
 * Return: 0
 */
static int cputemp_get(void *data, u64 *val)
{
	u32 reg;

	reg = soctherm_readl(TS_TEMP1);
	*val = temp_translate(REG_GET(reg, TS_TEMP1_CPU_TEMP));
	return 0;
}

/**
 * cputemp_set() - Puts a particular value into the CPU temperature register
 * @data:		The pointer to data. Currently not being used.
 * @temp:		The temperature to be written to the register
 *
 * This function only works if temperature overrides have been enabled.
 * Clears the original register CPU temperature, converts the given
 * temperature to a register value, and writes it to the CPU temp register.
 * Used for debugfs.
 *
 * Return: 0 (success).
 */
static int cputemp_set(void *data, u64 temp)
{
	u32 reg_val = temp_translate_rev(temp);
	u32 reg_orig = soctherm_readl(TS_TEMP1);

	reg_val = (reg_val << 16) | (reg_orig & 0xffff);
	soctherm_writel(reg_val, TS_TEMP1);
	return 0;
}

/**
 * gputemp_get() - retrieve GPU temperature from its register
 * @data:       argument for callback, currently not being used
 * @val:        pointer to a u64 location to store GPU temperature value
 *
 * Reads register value associated with the temperature sensor for the GPU
 * and stores it in the memory address pointed by val.
 *
 * Return: 0
 */
static int gputemp_get(void *data, u64 *val)
{
	u32 reg;

	reg = soctherm_readl(TS_TEMP1);
	*val = temp_translate(REG_GET(reg, TS_TEMP1_GPU_TEMP));
	return 0;
}

/**
 * gputemp_set() - Puts a particular value into the GPU temperature register
 * @data:		The pointer to data. Currently not being used.
 * @temp:		The temperature to be written to the register
 *
 * This function only works if temperature overrides have been enabled.
 * Clears the original GPU temperature register, converts the given
 * temperature to a register value, and writes it to the GPU temp register.
 * The @temp needs to be in the units of the SOC_THERM register temperature
 * bitfield.
 * Used for debugfs.
 *
 * Return: 0 (success).
 */
static int gputemp_set(void *data, u64 temp)
{
	u32 reg_val = temp_translate_rev(temp);
	u32 reg_orig = soctherm_readl(TS_TEMP1);

	reg_val = reg_val | (reg_orig & 0xffff0000);
	soctherm_writel(reg_val, TS_TEMP1);
	return 0;
}

/**
 * memtemp_get() - gets the memory temperature.
 * @data:	not used
 * @val:	a pointer in which the temperature will be placed.
 *
 * Reads the temperature of the thermal sensor associated with the memory.
 *
 * Return: 0
 */
static int memtemp_get(void *data, u64 *val)
{
	u32 reg;

	reg = soctherm_readl(TS_TEMP2);
	*val = temp_translate(REG_GET(reg, TS_TEMP2_MEM_TEMP));
	return 0;
}

/**
 * memtemp_set() - Overrides the memory temperature
 * in hardware
 * @data:	Opaque pointer to data; not used
 * @temp:	The temperature to be written to the register
 *
 * Clears the memory temperature register, converts @temp to
 * a register value, and writes the converted value to the register
 *
 * Function only works when temperature overrides are enabled.
 *
 * This function is called to debug/test temperature
 * trip points regarding MEM temperatures
 *
 * Return: 0 on success.
 */
static int memtemp_set(void *data, u64 temp)
{
	u32 reg_val = temp_translate_rev(temp);
	u32 reg_orig = soctherm_readl(TS_TEMP2);

	reg_val = (reg_val << 16) | (reg_orig & 0xffff);
	soctherm_writel(reg_val, TS_TEMP2);
	return 0;
}

/**
 * plltemp_get() - Gets the phase-locked loop temperature.
 * @data:	Opaque pointer to data
 * @val:	The pll temperature
 *
 * The temperature value is read in from the register.
 * The variable pointed to by @val is set to this temperature value.
 *
 * This function is used in debugfs
 *
 * Return: 0 on success.
 */
static int plltemp_get(void *data, u64 *val)
{
	u32 reg;

	reg = soctherm_readl(TS_TEMP2);
	*val = temp_translate(REG_GET(reg, TS_TEMP2_PLLX_TEMP));
	return 0;
}


/**
 * plltemp_set() - Stores a particular value into the PLLX temperature register
 * @data:		The pointer to data. Currently not being used.
 * @temp:		The temperature to be written to the register
 *
 * This function only works if temperature overrides have been enabled.
 * Clears the original PLLX temperature register, converts the given
 * temperature to a register value, and writes it to the PLLX temp register.
 * Used for debugfs.
 *
 * Return: 0 (success).
 */
static int plltemp_set(void *data, u64 temp)
{
	u32 reg_val = temp_translate_rev(temp);
	u32 reg_orig = soctherm_readl(TS_TEMP2);

	reg_val = reg_val | (reg_orig & 0xffff0000);
	soctherm_writel(reg_val, TS_TEMP2);
	return 0;
}

/**
 * tempoverride_get() - gets the temperature sensor software override value
 * @data:	not used
 * @val:	a pointer in which the value will be placed.
 *
 * Gets whether software override of the temperature is enabled. If it is
 * then TSENSOR_TEMP1/TSENSOR_TEMP2 will be set by the tsense block. If not,
 * then software will have to set it.
 *
 * Return: 0
 */
static int tempoverride_get(void *data, u64 *val)
{
	*val = soctherm_readl(TS_TEMP_SW_OVERRIDE);
	return 0;
}

/**
 * tempoverride_set() - enables or disables software temperature override
 * @data:       argument for callback, currently not being used
 * @val:        val should be 1 to enable override. 0 to disable override
 *
 * For debugging purposes, this function allows or disallow software
 * to override temperature reading. This is useful when testing how SOC_THERM
 * reacts to different temperature.
 *
 * Return: 0
 */
static int tempoverride_set(void *data, u64 val)
{
	soctherm_writel(val, TS_TEMP_SW_OVERRIDE);
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(convert_fops, convert_get, convert_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(cputemp_fops, cputemp_get, cputemp_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(gputemp_fops, gputemp_get, gputemp_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(memtemp_fops, memtemp_get, memtemp_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(plltemp_fops, plltemp_get, plltemp_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(tempoverride_fops, tempoverride_get, tempoverride_set,
			"%llu\n");

static int temp_log_open(struct inode *inode, struct file *file)
{
	return single_open(file, temp_log_show, inode->i_private);
}

static const struct file_operations temp_log_fops = {
	.open		= temp_log_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/**
 * soctherm_debug_init() - initializes the SOC_THERM debugfs files
 *
 * Creates a tegra_soctherm directory in debugfs, then creates all of the
 * debugfs files, setting the functions that are called when each respective
 * file is read or written.
 *
 * Return: 0
 */
static int __init soctherm_debug_init(void)
{
	struct dentry *tegra_soctherm_root;

	tegra_soctherm_root = debugfs_create_dir("tegra_soctherm", NULL);
	debugfs_create_file("regs", 0644, tegra_soctherm_root,
			    NULL, &regs_fops);
	debugfs_create_file("convert", 0644, tegra_soctherm_root,
			    NULL, &convert_fops);
	debugfs_create_file("cputemp", 0644, tegra_soctherm_root,
			    NULL, &cputemp_fops);
	debugfs_create_file("gputemp", 0644, tegra_soctherm_root,
			    NULL, &gputemp_fops);
	debugfs_create_file("memtemp", 0644, tegra_soctherm_root,
			    NULL, &memtemp_fops);
	debugfs_create_file("plltemp", 0644, tegra_soctherm_root,
			    NULL, &plltemp_fops);
	debugfs_create_file("tempoverride", 0644, tegra_soctherm_root,
			    NULL, &tempoverride_fops);
	debugfs_create_file("temp_log", 0644, tegra_soctherm_root,
			    NULL, &temp_log_fops);
	return 0;
}
late_initcall(soctherm_debug_init);

#endif
