/*
 * arch/arm/mach-tegra/pm.c
 *
 * CPU complex suspend & resume functions for Tegra SoCs
 *
 * Copyright (c) 2013-2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/ctype.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/smp.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/cpumask.h>
#include <linux/delay.h>
#include <linux/cpu_pm.h>
#include <linux/err.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/suspend.h>
#include <linux/serial_reg.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/syscore_ops.h>
#include <linux/cpu_pm.h>
#include <linux/clk/tegra.h>
#include <linux/export.h>
#include <linux/vmalloc.h>
#include <linux/console.h>
#include <linux/tegra_audio.h>
#include <linux/clk/tegra.h>
#include <linux/tegra-powergate.h>
#include <linux/tegra-soc.h>
#include <linux/tegra-timer.h>
#include <linux/tegra-cpuidle.h>
#include <linux/irqchip/tegra.h>
#include <linux/tegra-pm.h>
#include <linux/tegra_pm_domains.h>
#include <linux/kmemleak.h>
#include <linux/cpu.h>

#include <trace/events/power.h>
#include <trace/events/nvsecurity.h>
#include <linux/tegra-pmc.h>

#include <asm/cacheflush.h>
#include <asm/idmap.h>
#include <asm/pgalloc.h>
#include <asm/pgtable.h>
#include <asm/tlbflush.h>
#include <asm/suspend.h>
#include <asm/smp_plat.h>
#include <asm/psci.h>

#include <mach/irqs.h>
#include <mach/tegra_smmu.h>

#include "board.h"
#include <linux/platform/tegra/clock.h>
#include "common.h"
#include "iomap.h"
#include "pm.h"
#include "pm-soc.h"
#include "reset.h"
#include "sleep.h"
#include <linux/platform/tegra/dvfs.h>
#include "cpu-tegra.h"
#include "flowctrl.h"
#include "board.h"

#include "pm-tegra132.h"

/* core power request enable */
#define TEGRA_POWER_PWRREQ_OE		(1 << 9)
/* enter LP0 when CPU pwr gated */
#define TEGRA_POWER_EFFECT_LP0		(1 << 14)
/* CPU pwr req enable */
#define TEGRA_POWER_CPU_PWRREQ_OE	(1 << 16)

struct suspend_context {
	/*
	 * The next 7 values are referenced by offset in __restart_plls
	 * in headsmp-t2.S, and should not be moved
	 */
	u32 pllx_misc;
	u32 pllx_base;
	u32 pllp_misc;
	u32 pllp_base;
	u32 pllp_outa;
	u32 pllp_outb;
	u32 pll_timeout;

	u32 cpu_burst;
	u32 clk_csite_src;
	u32 cclk_divider;

	u32 mc[3];
	u8 uart[5];

	struct tegra_twd_context twd;
};
#define PMC_CTRL		0x0

#ifdef CONFIG_PM_SLEEP
static void __iomem *clk_rst = IO_ADDRESS(TEGRA_CLK_RESET_BASE);
static void __iomem *pmc = IO_ADDRESS(TEGRA_PMC_BASE);
static void __iomem *tmrus_reg_base = IO_ADDRESS(TEGRA_TMR1_BASE);
static int tegra_last_pclk;
static u64 resume_time;
static u64 resume_entry_time;
static u64 suspend_time;
static u64 suspend_entry_time;
#endif

struct suspend_context tegra_sctx;

#define TEGRA_POWER_PWRREQ_POLARITY	(1 << 8)   /* core power request polarity */
#define TEGRA_POWER_PWRREQ_OE		(1 << 9)   /* core power request enable */
#define TEGRA_POWER_SYSCLK_POLARITY	(1 << 10)  /* sys clk polarity */
#define TEGRA_POWER_SYSCLK_OE		(1 << 11)  /* system clock enable */
#define TEGRA_POWER_PWRGATE_DIS		(1 << 12)  /* power gate disabled */
#define TEGRA_POWER_EFFECT_LP0		(1 << 14)  /* enter LP0 when CPU pwr gated */
#define TEGRA_POWER_CPU_PWRREQ_POLARITY (1 << 15)  /* CPU power request polarity */
#define TEGRA_POWER_CPU_PWRREQ_OE	(1 << 16)  /* CPU power request enable */
#define TEGRA_POWER_CPUPWRGOOD_EN	(1 << 19)  /* CPU power good enable */

#define TEGRA_DPAD_ORIDE_SYS_CLK_REQ	(1 << 21)

#define PMC_CTRL		0x0
#define PMC_CTRL_LATCH_WAKEUPS	(1 << 5)
#define PMC_WAKE_MASK		0xc
#define PMC_WAKE_LEVEL		0x10
#define PMC_DPAD_ORIDE		0x1C
#define PMC_WAKE_DELAY		0xe0
#define PMC_DPD_SAMPLE		0x20
#if defined(CONFIG_ARCH_TEGRA_14x_SOC) || defined(CONFIG_ARCH_TEGRA_12x_SOC)
#define PMC_DPD_ENABLE		0x24
#endif
#define PMC_IO_DPD_REQ          0x1B8
#define PMC_IO_DPD2_REQ         0x1C0

#define PMC_WAKE_STATUS		0x14
#define PMC_WAKE2_STATUS	0x168
#define PMC_SW_WAKE_STATUS	0x18
#define PMC_COREPWRGOOD_TIMER	0x3c
#define PMC_CPUPWRGOOD_TIMER	0xc8
#define PMC_CPUPWROFF_TIMER	0xcc
#define PMC_COREPWROFF_TIMER	PMC_WAKE_DELAY

#define PMC_PWRGATE_TOGGLE	0x30
#define PWRGATE_TOGGLE_START	(1 << 8)
#define UN_PWRGATE_CPU		\
	(PWRGATE_TOGGLE_START | TEGRA_CPU_POWERGATE_ID(TEGRA_POWERGATE_CPU))

#define PMC_SCRATCH4_WAKE_CLUSTER_MASK	(1<<31)

#define CLK_RESET_CCLK_BURST	0x20
#define CLK_RESET_CCLK_DIVIDER  0x24
#define CLK_RESET_PLLC_BASE	0x80
#define CLK_RESET_PLLM_BASE	0x90
#define CLK_RESET_PLLX_BASE	0xe0
#define CLK_RESET_PLLX_MISC	0xe4
#define CLK_RESET_PLLP_BASE	0xa0
#define CLK_RESET_PLLP_OUTA	0xa4
#define CLK_RESET_PLLP_OUTB	0xa8
#define CLK_RESET_PLLP_MISC	0xac

#define CLK_RESET_SOURCE_CSITE	0x1d4

#define CLK_RESET_CCLK_BURST_POLICY_SHIFT 28
#define CLK_RESET_CCLK_RUN_POLICY_SHIFT    4
#define CLK_RESET_CCLK_IDLE_POLICY_SHIFT   0
#define CLK_RESET_CCLK_IDLE_POLICY	   1
#define CLK_RESET_CCLK_RUN_POLICY	   2
#define CLK_RESET_CCLK_BURST_POLICY_PLLM   3
#define CLK_RESET_CCLK_BURST_POLICY_PLLX   8

#define MC_SECURITY_START	0x6c
#define MC_SECURITY_SIZE	0x70
#define MC_SECURITY_CFG2	0x7c

#ifdef CONFIG_ARCH_TEGRA_HAS_CL_DVFS
static struct clk *tegra_dfll;
#endif
static struct clk *tegra_pclk;
static struct clk *tegra_clk_m;
static struct tegra_suspend_platform_data *pdata;
static enum tegra_suspend_mode current_suspend_mode = TEGRA_SUSPEND_NONE;

void (*tegra_tear_down_cpu)(void);
int (*tegra_sleep_core_finish)(unsigned long v2p);


bool tegra_is_dpd_mode = false;

static bool suspend_in_progress;

bool tegra_suspend_in_progress(void)
{
	smp_rmb();

	return suspend_in_progress;
}

bool tegra_dvfs_is_dfll_bypass(void)
{
#ifdef CONFIG_REGULATOR_TEGRA_DFLL_BYPASS
	return true;
#else
	return false;
#endif
}

#ifdef CONFIG_PM_SLEEP

static const char *tegra_suspend_name[TEGRA_MAX_SUSPEND_MODE] = {
	[TEGRA_SUSPEND_NONE]	= "none",
	[TEGRA_SUSPEND_LP2]	= "lp2",
	[TEGRA_SUSPEND_LP1]	= "lp1",
	[TEGRA_SUSPEND_LP0]	= "lp0",
};

void tegra_log_resume_time(void)
{
	u64 resume_end_time = readl(tmrus_reg_base + TIMERUS_CNTR_1US);

	if (resume_entry_time > resume_end_time)
		resume_end_time |= 1ull<<32;
	resume_time = resume_end_time - resume_entry_time;
}

void tegra_log_suspend_time(void)
{
	suspend_entry_time = readl(tmrus_reg_base + TIMERUS_CNTR_1US);
}

static void tegra_get_suspend_time(void)
{
	u64 suspend_end_time;
	suspend_end_time = readl(tmrus_reg_base + TIMERUS_CNTR_1US);

	if (suspend_entry_time > suspend_end_time)
		suspend_end_time |= 1ull<<32;
	suspend_time = suspend_end_time - suspend_entry_time;
}

unsigned long tegra_cpu_power_good_time(void)
{
	if (WARN_ON_ONCE(!pdata))
		return 5000;

	return pdata->cpu_timer;
}

unsigned long tegra_cpu_power_off_time(void)
{
	if (WARN_ON_ONCE(!pdata))
		return 5000;

	return pdata->cpu_off_timer;
}

static void suspend_cpu_dfll_mode(unsigned int flags)
{
#ifdef CONFIG_ARCH_TEGRA_HAS_CL_DVFS
	/* If DFLL is used as CPU clock source go to open loop mode */
	if (!(flags & TEGRA_POWER_CLUSTER_MASK)) {
		if (tegra_dfll && tegra_dvfs_rail_is_dfll_mode(tegra_cpu_rail))
			tegra_clk_cfg_ex(tegra_dfll, TEGRA_CLK_DFLL_LOCK, 0);
	}

	/* Suspend dfll bypass (safe rail down) on LP or if DFLL is Not used */
	if (pdata && pdata->suspend_dfll_bypass &&
	    (!tegra_dvfs_rail_is_dfll_mode(tegra_cpu_rail)))
		pdata->suspend_dfll_bypass();
#endif
}

static void resume_cpu_dfll_mode(unsigned int flags)
{
#ifdef CONFIG_ARCH_TEGRA_HAS_CL_DVFS
	/* If DFLL is Not used and resume on G restore bypass mode */
	if (pdata && pdata->resume_dfll_bypass &&
	    tegra_is_clk_initialized(tegra_dfll) &&
	    !tegra_dvfs_rail_is_dfll_mode(tegra_cpu_rail))
		pdata->resume_dfll_bypass();

	/* If DFLL is used as CPU clock source restore closed loop mode */
	if (!(flags & TEGRA_POWER_CLUSTER_MASK)) {
		if (tegra_dfll && tegra_dvfs_rail_is_dfll_mode(tegra_cpu_rail))
			tegra_clk_cfg_ex(tegra_dfll, TEGRA_CLK_DFLL_LOCK, 1);
	}
#endif
}

/* ensures that sufficient time is passed for a register write to
 * serialize into the 32KHz domain */
static void pmc_32kwritel(u32 val, unsigned long offs)
{
	writel(val, pmc + offs);
	udelay(130);
}

#if !defined(CONFIG_OF) || !defined(CONFIG_COMMON_CLK)
static void set_power_timers(unsigned long us_on, unsigned long us_off,
			     long rate)
{
	static unsigned long last_us_off;
	unsigned long long ticks;
	unsigned long long pclk;

	if (WARN_ON_ONCE(rate <= 0))
		pclk = 100000000;
	else
		pclk = rate;

	if ((rate != tegra_last_pclk) || (us_off != last_us_off)) {
		ticks = (us_on * pclk) + 999999ull;
		do_div(ticks, 1000000);
		writel((unsigned long)ticks, pmc + PMC_CPUPWRGOOD_TIMER);

		ticks = (us_off * pclk) + 999999ull;
		do_div(ticks, 1000000);
		writel((unsigned long)ticks, pmc + PMC_CPUPWROFF_TIMER);
		wmb();
	}
	tegra_last_pclk = pclk;
	last_us_off = us_off;
}
#endif

/*
 * restore_cpu_complex
 *
 * restores cpu clock setting, clears flow controller
 *
 * Always called on CPU 0.
 */
static void restore_cpu_complex(u32 mode)
{
	int cpu = cpu_logical_map(smp_processor_id());
	unsigned int reg;

	writel(tegra_sctx.clk_csite_src, clk_rst + CLK_RESET_SOURCE_CSITE);

	/* Do not power-gate CPU 0 when flow controlled */
	reg = readl(FLOW_CTRL_CPU_CSR(cpu));
	reg &= ~FLOW_CTRL_CSR_WFE_BITMAP;	/* clear wfe bitmap */
	reg &= ~FLOW_CTRL_CSR_WFI_BITMAP;	/* clear wfi bitmap */
	reg &= ~FLOW_CTRL_CSR_ENABLE;		/* clear enable */
	reg |= FLOW_CTRL_CSR_INTR_FLAG;		/* clear intr */
	reg |= FLOW_CTRL_CSR_EVENT_FLAG;	/* clear event */
	flowctrl_writel(reg, FLOW_CTRL_CPU_CSR(cpu));
}

/*
 * suspend_cpu_complex
 *
 * saves pll state for use by restart_plls, prepares flow controller for
 * transition to suspend state
 *
 * Must always be called on cpu 0.
 */
static void suspend_cpu_complex(u32 mode)
{
	int cpu = cpu_logical_map(smp_processor_id());
	unsigned int reg;
	int i;

	BUG_ON(cpu != 0);

	/* switch coresite to clk_m, save off original source */
	tegra_sctx.clk_csite_src = readl(clk_rst + CLK_RESET_SOURCE_CSITE);
	writel(3<<30, clk_rst + CLK_RESET_SOURCE_CSITE);

	tegra_sctx.cpu_burst = readl(clk_rst + CLK_RESET_CCLK_BURST);
	tegra_sctx.pllx_base = readl(clk_rst + CLK_RESET_PLLX_BASE);
	tegra_sctx.pllx_misc = readl(clk_rst + CLK_RESET_PLLX_MISC);
	tegra_sctx.pllp_base = readl(clk_rst + CLK_RESET_PLLP_BASE);
	tegra_sctx.pllp_outa = readl(clk_rst + CLK_RESET_PLLP_OUTA);
	tegra_sctx.pllp_outb = readl(clk_rst + CLK_RESET_PLLP_OUTB);
	tegra_sctx.pllp_misc = readl(clk_rst + CLK_RESET_PLLP_MISC);
	tegra_sctx.cclk_divider = readl(clk_rst + CLK_RESET_CCLK_DIVIDER);

	reg = readl(FLOW_CTRL_CPU_CSR(cpu));
	reg &= ~FLOW_CTRL_CSR_WFE_BITMAP;	/* clear wfe bitmap */
	reg &= ~FLOW_CTRL_CSR_WFI_BITMAP;	/* clear wfi bitmap */
	reg |= FLOW_CTRL_CSR_INTR_FLAG;		/* clear intr flag */
	reg |= FLOW_CTRL_CSR_EVENT_FLAG;	/* clear event flag */
	reg |= FLOW_CTRL_CSR_WFI_CPU0 << cpu;	/* enable power gating on wfi */
	reg |= FLOW_CTRL_CSR_ENABLE;		/* enable power gating */
	flowctrl_writel(reg, FLOW_CTRL_CPU_CSR(cpu));

	for (i = 0; i < num_possible_cpus(); i++) {
		if (i == cpu)
			continue;
		reg = readl(FLOW_CTRL_CPU_CSR(i));
		reg |= FLOW_CTRL_CSR_EVENT_FLAG;
		reg |= FLOW_CTRL_CSR_INTR_FLAG;
		flowctrl_writel(reg, FLOW_CTRL_CPU_CSR(i));
	}

	tegra_gic_cpu_disable(true);
}

void tegra_psci_suspend_cpu(void *entry_point)
{
	struct psci_power_state pps;

	if (tegra_cpu_is_secure()) {
		if (psci_ops.cpu_suspend) {
			pps.id = TEGRA_ID_CPU_SUSPEND_LP0;
			pps.type = PSCI_POWER_STATE_TYPE_POWER_DOWN;
			pps.affinity_level = TEGRA_PWR_DN_AFFINITY_CLUSTER;

			psci_ops.cpu_suspend(pps, virt_to_phys(entry_point));
		}
	}
}

static void tegra_sleep_core(enum tegra_suspend_mode mode,
			     unsigned long v2p)
{
	if (tegra_cpu_is_secure()) {
		__flush_dcache_area(&tegra_resume_timestamps_start,
					(&tegra_resume_timestamps_end -
					 &tegra_resume_timestamps_start));

		BUG_ON(mode != TEGRA_SUSPEND_LP0);

		trace_smc_sleep_core(NVSEC_SMC_START);
		tegra_psci_suspend_cpu(tegra_resume);
		trace_smc_sleep_core(NVSEC_SMC_DONE);
	}

	tegra_get_suspend_time();
	cpu_suspend(v2p, tegra_sleep_core_finish);
}

static inline void tegra_sleep_cpu(unsigned long v2p)
{
	cpu_suspend(v2p, tegra_sleep_cpu_finish);
}

static int tegra_common_suspend(void)
{
	void __iomem *mc = IO_ADDRESS(TEGRA_MC_BASE);

	tegra_sctx.mc[0] = readl(mc + MC_SECURITY_START);
	tegra_sctx.mc[1] = readl(mc + MC_SECURITY_SIZE);
	tegra_sctx.mc[2] = readl(mc + MC_SECURITY_CFG2);

	return 0;
}

static void tegra_common_resume(void)
{
	void __iomem *mc = IO_ADDRESS(TEGRA_MC_BASE);

	/* Clear DPD Enable */
	writel(0x0, pmc + PMC_DPD_ENABLE);

	writel(tegra_sctx.mc[0], mc + MC_SECURITY_START);
	writel(tegra_sctx.mc[1], mc + MC_SECURITY_SIZE);
	writel(tegra_sctx.mc[2], mc + MC_SECURITY_CFG2);
	writel(0x0, pmc + PMC_SCRATCH41);
}

static void tegra_pm_set(enum tegra_suspend_mode mode)
{
	u32 reg, boot_flag;
	unsigned long rate = 32768;

	reg = readl(pmc + PMC_CTRL);
	reg |= TEGRA_POWER_CPU_PWRREQ_OE;
	if (pdata->combined_req)
		reg &= ~TEGRA_POWER_PWRREQ_OE;
	else
		reg |= TEGRA_POWER_PWRREQ_OE;
	reg &= ~TEGRA_POWER_EFFECT_LP0;

	switch (mode) {
	case TEGRA_SUSPEND_LP0:
		if (pdata->combined_req) {
			reg |= TEGRA_POWER_PWRREQ_OE;
			reg &= ~TEGRA_POWER_CPU_PWRREQ_OE;
		}

		/* Enable DPD sample to trigger sampling pads data and direction
		 * in which pad will be driven during lp0 mode*/
		writel(0x1, pmc + PMC_DPD_SAMPLE);
		writel(0x800fdfff, pmc + PMC_IO_DPD_REQ);
		writel(0x80001fff, pmc + PMC_IO_DPD2_REQ);
		tegra_is_dpd_mode = true;

		/* Set warmboot flag */
		boot_flag = readl(pmc + PMC_SCRATCH0);
		pmc_32kwritel(boot_flag | 1, PMC_SCRATCH0);

		pmc_32kwritel(tegra_lp0_vec_start, PMC_SCRATCH1);

		reg |= TEGRA_POWER_EFFECT_LP0;

		/* No break here. LP0 code falls through to write SCRATCH41 */
	case TEGRA_SUSPEND_LP1:
		__raw_writel(virt_to_phys(tegra_resume), pmc + PMC_SCRATCH41);
		wmb();
		rate = clk_get_rate(tegra_clk_m);
		break;
	case TEGRA_SUSPEND_LP2:
		rate = clk_get_rate(tegra_pclk);
		break;
	case TEGRA_SUSPEND_NONE:
		return;
	default:
		BUG();
	}

#if defined(CONFIG_OF) && defined(CONFIG_COMMON_CLK)
	set_power_timers(pdata->cpu_timer, pdata->cpu_off_timer);
#else
	set_power_timers(pdata->cpu_timer, pdata->cpu_off_timer, rate);
#endif

	pmc_32kwritel(reg, PMC_CTRL);
}

static const char *lp_state[TEGRA_MAX_SUSPEND_MODE] = {
	[TEGRA_SUSPEND_NONE] = "none",
	[TEGRA_SUSPEND_LP2] = "LP2",
	[TEGRA_SUSPEND_LP1] = "LP1",
	[TEGRA_SUSPEND_LP0] = "LP0",
};

static int tegra_suspend_enter(suspend_state_t state)
{
	int ret = 0;
	ktime_t delta;
	struct timespec ts_entry, ts_exit;

	if (pdata && pdata->board_suspend)
		pdata->board_suspend(current_suspend_mode,
			TEGRA_SUSPEND_BEFORE_PERIPHERAL);

	read_persistent_clock(&ts_entry);

	ret = tegra_suspend_dram(current_suspend_mode, 0);
	if (ret) {
		pr_info("Aborting suspend, tegra_suspend_dram error=%d\n", ret);
		goto abort_suspend;
	}

	read_persistent_clock(&ts_exit);

	if (timespec_compare(&ts_exit, &ts_entry) > 0) {
		delta = timespec_to_ktime(timespec_sub(ts_exit, ts_entry));

		tegra_dvfs_rail_pause(tegra_cpu_rail, delta, false);
		tegra_dvfs_rail_pause(tegra_gpu_rail, delta, false);
		if (current_suspend_mode == TEGRA_SUSPEND_LP0)
			tegra_dvfs_rail_pause(tegra_core_rail, delta, false);
		else
			tegra_dvfs_rail_pause(tegra_core_rail, delta, true);
	}

abort_suspend:
	if (pdata && pdata->board_resume)
		pdata->board_resume(current_suspend_mode,
			TEGRA_RESUME_AFTER_PERIPHERAL);

	return ret;
}

static void tegra_suspend_check_pwr_stats(void)
{
	/* cpus and l2 are powered off later */
	unsigned long pwrgate_partid_mask =
		(1 << TEGRA_POWERGATE_HEG)	|
		(1 << TEGRA_POWERGATE_SATA)	|
		(1 << TEGRA_POWERGATE_3D1)	|
		(1 << TEGRA_POWERGATE_3D)	|
		(1 << TEGRA_POWERGATE_VENC)	|
		(1 << TEGRA_POWERGATE_PCIE)	|
		(1 << TEGRA_POWERGATE_VDEC)	|
		(1 << TEGRA_POWERGATE_MPE);

	int partid;

	for (partid = 0; partid < TEGRA_NUM_POWERGATE; partid++)
		if ((1 << partid) & pwrgate_partid_mask)
			if (tegra_powergate_is_powered(partid))
				pr_debug("partition %s is left on before suspend\n",
					tegra_powergate_get_name(partid));

	return;
}

static void tegra_suspend_powergate_control(int partid, bool turn_off)
{
	if (turn_off)
		tegra_powergate_partition(partid);
	else
		tegra_unpowergate_partition(partid);
}

int tegra_suspend_dram(enum tegra_suspend_mode mode, unsigned int flags)
{
	int err = 0;
	u32 scratch37 = 0xDEADBEEF;
	u32 reg;

	bool tegra_suspend_vde_powergated = false;

	if (WARN_ON(mode <= TEGRA_SUSPEND_NONE ||
		mode >= TEGRA_MAX_SUSPEND_MODE)) {
		err = -ENXIO;
		goto fail;
	}

	if ((mode == TEGRA_SUSPEND_LP0) && !tegra_pm_irq_lp0_allowed()) {
		pr_info("LP0 not used due to unsupported wakeup events\n");
		mode = TEGRA_SUSPEND_LP1;
	}

	/* turn off VDE partition in LP1 */
	if (mode == TEGRA_SUSPEND_LP1 &&
		tegra_powergate_is_powered(TEGRA_POWERGATE_VDEC)) {
		pr_info("turning off partition %s in LP1\n",
			tegra_powergate_get_name(TEGRA_POWERGATE_VDEC));
		tegra_suspend_powergate_control(TEGRA_POWERGATE_VDEC, true);
		tegra_suspend_vde_powergated = true;
	}

	tegra_common_suspend();

	tegra_pm_set(mode);

	if (pdata && pdata->board_suspend)
		pdata->board_suspend(mode, TEGRA_SUSPEND_BEFORE_CPU);

	local_fiq_disable();

	trace_cpu_suspend(CPU_SUSPEND_START, tegra_rtc_read_ms());

	if (mode == TEGRA_SUSPEND_LP0) {
		tegra_tsc_suspend();
		tegra_lp0_suspend_mc();
		tegra_tsc_wait_for_suspend();
	}

	cpu_cluster_pm_enter();
	suspend_cpu_complex(flags);

	if (tegra_get_chipid() != TEGRA_CHIPID_TEGRA13)
		flush_cache_all();

	if (mode == TEGRA_SUSPEND_LP2)
		tegra_sleep_cpu(PHYS_OFFSET - PAGE_OFFSET);
	else
		tegra_sleep_core(mode, PHYS_OFFSET - PAGE_OFFSET);

	resume_entry_time = 0;
	if (mode != TEGRA_SUSPEND_LP0)
		resume_entry_time = readl(tmrus_reg_base + TIMERUS_CNTR_1US);

	if (mode == TEGRA_SUSPEND_LP0) {
		tegra_tsc_resume();
		tegra_lp0_resume_mc();
		tegra_tsc_wait_for_resume();
	}

	/* if scratch37 was clobbered during LP1, restore it */
	if (scratch37 != 0xDEADBEEF)
		pmc_32kwritel(scratch37, PMC_SCRATCH37);

	restore_cpu_complex(flags);
	cpu_cluster_pm_exit();

	/* for platforms where the core & CPU power requests are
	 * combined as a single request to the PMU, transition out
	 * of LP0 state by temporarily enabling both requests
	 */
	if (mode == TEGRA_SUSPEND_LP0 && pdata->combined_req) {
		reg = readl(pmc + PMC_CTRL);
		reg |= TEGRA_POWER_CPU_PWRREQ_OE;
		pmc_32kwritel(reg, PMC_CTRL);
		reg &= ~TEGRA_POWER_PWRREQ_OE;
		pmc_32kwritel(reg, PMC_CTRL);
	}

	if (pdata && pdata->board_resume)
		pdata->board_resume(mode, TEGRA_RESUME_AFTER_CPU);

	trace_cpu_suspend(CPU_SUSPEND_DONE, tegra_rtc_read_ms());

	local_fiq_enable();

	suspend_in_progress = false;

	tegra_common_resume();

	/* turn on VDE partition in LP1 */
	if (mode == TEGRA_SUSPEND_LP1 && tegra_suspend_vde_powergated) {
		pr_info("turning on partition %s in LP1\n",
			tegra_powergate_get_name(TEGRA_POWERGATE_VDEC));
		tegra_suspend_powergate_control(TEGRA_POWERGATE_VDEC, false);
	}

fail:
	return err;
}

static int tegra_suspend_valid(suspend_state_t state)
{
	int valid = 1;

	/* LP0 is not supported on T132 A01 silicon */
	if (tegra_get_chipid() == TEGRA_CHIPID_TEGRA13 && \
		tegra_revision == TEGRA_REVISION_A01)
		valid = 0;

	return valid;
}

static int tegra_suspend_prepare_late(void)
{
	if ((current_suspend_mode == TEGRA_SUSPEND_LP0) ||
			(current_suspend_mode == TEGRA_SUSPEND_LP1))
		tegra_suspend_check_pwr_stats();

	return 0;
}

/*
 * Clear the wake status on suspend prepare. If we're starting to suspend,
 * the previous wake reason can be misleading.
 */
static int pm_suspend_clear_status(struct notifier_block *nb,
						unsigned long event, void *data)
{
	u32 temp;
	if (event == PM_SUSPEND_PREPARE) {
		temp = readl(pmc + PMC_WAKE_STATUS);
		if (temp)
			pmc_32kwritel(temp, PMC_WAKE_STATUS);
#ifndef CONFIG_ARCH_TEGRA_2x_SOC
		temp = readl(pmc + PMC_WAKE2_STATUS);
		if (temp)
			pmc_32kwritel(temp, PMC_WAKE2_STATUS);
#endif
	}
	return NOTIFY_OK;
}

/*
 * LP0 WAR: Bring all CPUs online before LP0 so that they can be put into C7 on
 * subsequent __cpu_downs otherwise we end up hanging the system by leaving a
 * core in C6 and requesting LP0 from CPU0
 */
static int __cpuinit pm_suspend_notifier(struct notifier_block *nb,
						unsigned long event, void *data)
{
	int cpu, ret;

	if (event != PM_SUSPEND_PREPARE)
		return NOTIFY_OK;

	suspend_in_progress = true;

	dsb();

	for_each_present_cpu(cpu) {
		if (!cpu)
			continue;
		ret = cpu_up(cpu);

		/*
		 * Error in getting CPU out of C6. Let -EINVAL through as CPU
		 * could have come online
		 */
		if (ret && ret != -EINVAL) {
			pr_err("%s: Couldn't bring up CPU%d on LP0 entry: %d\n",
					__func__, cpu, ret);
			return NOTIFY_BAD;
		}
	}

	return NOTIFY_OK;
}

static void tegra_suspend_finish(void)
{
	if (pdata && pdata->cpu_resume_boost) {
		int ret = tegra_suspended_target(pdata->cpu_resume_boost);
		pr_info("Tegra: resume CPU boost to %u KHz: %s (%d)\n",
			pdata->cpu_resume_boost, ret ? "Failed" : "OK", ret);
	}
}

static const struct platform_suspend_ops tegra_suspend_ops = {
	.prepare_late = tegra_suspend_prepare_late,
	.valid		= tegra_suspend_valid,
	.finish		= tegra_suspend_finish,
	.enter		= tegra_suspend_enter,
};

/*
 * Note: The priority of this notifier needs to be higher than cpu_hotplug's
 * suspend notifier otherwise the subsequent cpu_up operation in
 * pm_suspend_notifier will fail
 */
static struct notifier_block __cpuinitdata suspend_notifier = {
	.notifier_call = pm_suspend_notifier,
	.priority = 1,
};

static struct notifier_block suspend_clear_status = {
	.notifier_call = pm_suspend_clear_status,
};

static ssize_t suspend_mode_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	char *start = buf;
	char *end = buf + PAGE_SIZE;

	start += scnprintf(start, end - start, "%s ",
				tegra_suspend_name[current_suspend_mode]);
	start += scnprintf(start, end - start, "\n");

	return start - buf;
}

static ssize_t suspend_mode_store(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t n)
{
	int len;
	const char *name_ptr;
	enum tegra_suspend_mode new_mode;

	name_ptr = buf;
	while (*name_ptr && !isspace(*name_ptr))
		name_ptr++;
	len = name_ptr - buf;
	if (!len)
		goto bad_name;
	/* TEGRA_SUSPEND_NONE not allowed as suspend state */
	if (!(strncmp(buf, tegra_suspend_name[TEGRA_SUSPEND_NONE], len))
		|| !(strncmp(buf, tegra_suspend_name[TEGRA_SUSPEND_LP2], len))) {
		pr_info("Illegal tegra suspend state: %s\n", buf);
		goto bad_name;
	}

	for (new_mode = TEGRA_SUSPEND_NONE;
			new_mode < TEGRA_MAX_SUSPEND_MODE; ++new_mode) {
		if (!strncmp(buf, tegra_suspend_name[new_mode], len)) {
			current_suspend_mode = new_mode;
			break;
		}
	}

bad_name:
	return n;
}

static struct kobj_attribute suspend_mode_attribute =
	__ATTR(mode, 0644, suspend_mode_show, suspend_mode_store);

static ssize_t suspend_resume_time_show(struct kobject *kobj,
					struct kobj_attribute *attr,
					char *buf)
{
	return sprintf(buf, "%ums\n", ((u32)resume_time / 1000));
}

static struct kobj_attribute suspend_resume_time_attribute =
	__ATTR(resume_time, 0444, suspend_resume_time_show, 0);

static ssize_t suspend_time_show(struct kobject *kobj,
					struct kobj_attribute *attr,
					char *buf)
{
	return sprintf(buf, "%ums\n", ((u32)suspend_time / 1000));
}

static struct kobj_attribute suspend_time_attribute =
	__ATTR(suspend_time, 0444, suspend_time_show, 0);

static struct kobject *suspend_kobj;

static int tegra_pm_enter_suspend(void)
{
	suspend_cpu_dfll_mode(0);
	if (current_suspend_mode == TEGRA_SUSPEND_LP0)
		tegra_lp0_cpu_mode(true);
	pr_info("Entering suspend state %s\n", lp_state[current_suspend_mode]);
	return 0;
}

static void tegra_pm_enter_resume(void)
{
	pr_info("Exited suspend state %s\n", lp_state[current_suspend_mode]);
	if (current_suspend_mode == TEGRA_SUSPEND_LP0)
		tegra_lp0_cpu_mode(false);
	resume_cpu_dfll_mode(0);
}

static void tegra_pm_enter_shutdown(void)
{
	suspend_cpu_dfll_mode(0);
	pr_info("Shutting down tegra ...\n");
}

static struct syscore_ops tegra_pm_enter_syscore_ops = {
	.suspend = tegra_pm_enter_suspend,
	.resume = tegra_pm_enter_resume,
	.save = tegra_pm_enter_suspend,
	.restore = tegra_pm_enter_resume,
	.shutdown = tegra_pm_enter_shutdown,
};

static __init int tegra_pm_enter_syscore_init(void)
{
	register_syscore_ops(&tegra_pm_enter_syscore_ops);
	return 0;
}
subsys_initcall(tegra_pm_enter_syscore_init);
#endif

void __init tegra_init_suspend(struct tegra_suspend_platform_data *plat)
{
	u32 reg;
	u32 mode;
	struct pmc_pm_data *pm_dat;
	bool is_board_pdata = true;

#ifdef CONFIG_ARCH_TEGRA_HAS_CL_DVFS
	tegra_dfll = clk_get_sys(NULL, "dfll_cpu");
	BUG_ON(IS_ERR(tegra_dfll));
#endif
	tegra_pclk = clk_get_sys(NULL, "pclk");
	BUG_ON(IS_ERR(tegra_pclk));
	tegra_clk_m = clk_get_sys(NULL, "clk_m");
	BUG_ON(IS_ERR(tegra_clk_m));

	/* create the pdata from DT information */
	pm_dat = tegra_get_pm_data();
	if (pm_dat) {
		pr_err("PMC dt information non-NULL %s\n", __func__);
		is_board_pdata = false;
		pdata = kzalloc(sizeof(struct tegra_suspend_platform_data),
			GFP_KERNEL);
		if (pm_dat->combined_req != plat->combined_req) {
			pr_err("PMC DT attribute combined_req=%d, board value=%d\n",
				pm_dat->combined_req, plat->combined_req);
			pdata->combined_req = plat->combined_req;
		} else {
			pdata->combined_req = pm_dat->combined_req;
		}
		if (pm_dat->sysclkreq_high != plat->sysclkreq_high) {
			pr_err("PMC DT attribute sysclkreq_high=%d, board value=%d\n",
				pm_dat->sysclkreq_high, plat->sysclkreq_high);
			pdata->sysclkreq_high = plat->sysclkreq_high;
		} else {
			pdata->sysclkreq_high = pm_dat->sysclkreq_high;
		}
		if (pm_dat->corereq_high != plat->corereq_high) {
			pr_err("PMC DT attribute corereq_high=%d, board value=%d\n",
				pm_dat->corereq_high, plat->corereq_high);
			pdata->corereq_high = plat->corereq_high;
		} else {
			pdata->corereq_high = pm_dat->corereq_high;
		}
		if (pm_dat->cpu_off_time != plat->cpu_off_timer) {
			pr_err("PMC DT attribute cpu_off_timer=%d, board value=%ld\n",
				pm_dat->cpu_off_time, plat->cpu_off_timer);
			pdata->cpu_off_timer = plat->cpu_off_timer;
		} else {
			pdata->cpu_off_timer = pm_dat->cpu_off_time;
		}
		if (pm_dat->cpu_good_time != plat->cpu_timer) {
			pr_err("PMC DT attribute cpu_timer=%d, board value=%ld\n",
				pm_dat->cpu_good_time, plat->cpu_timer);
			pdata->cpu_timer = plat->cpu_timer;
		} else {
			pdata->cpu_timer = pm_dat->cpu_good_time;
		}
		if (pm_dat->suspend_mode != plat->suspend_mode) {
			pr_err("PMC DT attribute suspend_mode=%d, board value=%d\n",
				pm_dat->suspend_mode, plat->suspend_mode);
			pdata->suspend_mode = plat->suspend_mode;
		} else {
			pdata->suspend_mode = pm_dat->suspend_mode;
		}
		/* FIXME: pmc_pm_data fields to be reused
		 *	core_osc_time, core_pmu_time, core_off_time
		 *	units of above fields is uSec while
		 *	platform data values are in ticks
		 */
		/* FIXME: pmc_pm_data unused by downstream code
		 *	cpu_pwr_good_en, lp0_vec_size, lp0_vec_phy_addr
		 */
		/* FIXME: add missing DT bindings taken from platform data */
		pdata->core_timer = plat->core_timer;
		pdata->core_off_timer = plat->core_off_timer;
		pdata->board_suspend = plat->board_suspend;
		pdata->board_resume = plat->board_resume;
		pdata->sysclkreq_gpio = plat->sysclkreq_gpio;
		pdata->cpu_lp2_min_residency = plat->cpu_lp2_min_residency;
		pdata->cpu_resume_boost = plat->cpu_resume_boost;
#ifdef CONFIG_ARCH_TEGRA_HAS_SYMMETRIC_CPU_PWR_GATE
		pdata->min_residency_vmin_fmin = plat->min_residency_vmin_fmin;
		pdata->min_residency_ncpu_slow = plat->min_residency_ncpu_slow;
		pdata->min_residency_ncpu_fast = plat->min_residency_ncpu_fast;
		pdata->min_residency_crail = plat->min_residency_crail;
		pdata->crail_up_early = plat->crail_up_early;
#endif
		pdata->min_residency_mclk_stop = plat->min_residency_mclk_stop;
		pdata->usb_vbus_internal_wake = plat->usb_vbus_internal_wake;
		pdata->usb_id_internal_wake = plat->usb_id_internal_wake;
		pdata->suspend_dfll_bypass = plat->suspend_dfll_bypass;
		pdata->resume_dfll_bypass = plat->resume_dfll_bypass;
	} else {
		pr_err("PMC board data used in %s\n", __func__);
		pdata = plat;
	}
	(void)reg;
	(void)mode;

	if (plat->suspend_mode == TEGRA_SUSPEND_LP2)
		plat->suspend_mode = TEGRA_SUSPEND_LP0;

#ifndef CONFIG_PM_SLEEP
	if (plat->suspend_mode != TEGRA_SUSPEND_NONE) {
		pr_warn("%s: Suspend requires CONFIG_PM_SLEEP -- "
			   "disabling suspend\n", __func__);
		plat->suspend_mode = TEGRA_SUSPEND_NONE;
	}
#else

	if (plat->suspend_mode == TEGRA_SUSPEND_LP0 && tegra_lp0_vec_size &&
		tegra_lp0_vec_relocate) {
		unsigned char *reloc_lp0;
		unsigned long tmp;
		void __iomem *orig;
		reloc_lp0 = kmalloc(tegra_lp0_vec_size + L1_CACHE_BYTES - 1,
					GFP_KERNEL);
		WARN_ON(!reloc_lp0);
		if (!reloc_lp0) {
			pr_err("%s: Failed to allocate reloc_lp0\n",
				__func__);
			goto out;
		}

		/* Avoid a kmemleak false positive. The allocated memory
		 * block is later referenced by a physical address (i.e.
		 * tegra_lp0_vec_start) which kmemleak can't detect.
		 */
		kmemleak_not_leak(reloc_lp0);

		orig = ioremap_wc(tegra_lp0_vec_start, tegra_lp0_vec_size);
		WARN_ON(!orig);
		if (!orig) {
			pr_err("%s: Failed to map tegra_lp0_vec_start %llx\n",
				__func__, tegra_lp0_vec_start);
			kfree(reloc_lp0);
			goto out;
		}

		tmp = (unsigned long) reloc_lp0;
		tmp = (tmp + L1_CACHE_BYTES - 1) & ~(L1_CACHE_BYTES - 1);
		reloc_lp0 = (unsigned char *)tmp;
		memcpy(reloc_lp0, orig, tegra_lp0_vec_size);
		wmb();
		iounmap(orig);
		tegra_lp0_vec_start = virt_to_phys(reloc_lp0);
	}

out:
	if (plat->suspend_mode == TEGRA_SUSPEND_LP0 && !tegra_lp0_vec_size) {
		pr_warn("%s: Suspend mode LP0 requested, no lp0_vec "
			   "provided by bootlader -- disabling LP0\n",
			   __func__);
		plat->suspend_mode = TEGRA_SUSPEND_LP1;
	}

	/* Initialize SOC-specific data */
	tegra_soc_suspend_init();
	if (!tegra_tear_down_cpu || !tegra_sleep_core_finish) {
		pr_err("%s: unable to obtain suspend info -- "
				"disabling LP0\n", __func__);
		current_suspend_mode = TEGRA_SUSPEND_LP2;
	}

	/* Always enable CPU power request; just normal polarity is supported */
	reg = readl(pmc + PMC_CTRL);
	BUG_ON(reg & TEGRA_POWER_CPU_PWRREQ_POLARITY);
	reg |= TEGRA_POWER_CPU_PWRREQ_OE;
	pmc_32kwritel(reg, PMC_CTRL);

	/* Configure core power request and system clock control if LP0
	   is supported */
	__raw_writel(pdata->core_timer, pmc + PMC_COREPWRGOOD_TIMER);
	__raw_writel(pdata->core_off_timer, pmc + PMC_COREPWROFF_TIMER);

	reg = readl(pmc + PMC_CTRL);

	if (!pdata->sysclkreq_high)
		reg |= TEGRA_POWER_SYSCLK_POLARITY;
	else
		reg &= ~TEGRA_POWER_SYSCLK_POLARITY;

	if (!pdata->corereq_high)
		reg |= TEGRA_POWER_PWRREQ_POLARITY;
	else
		reg &= ~TEGRA_POWER_PWRREQ_POLARITY;

	/* configure output inverters while the request is tristated */
	pmc_32kwritel(reg, PMC_CTRL);

	/* now enable requests */
	reg |= TEGRA_POWER_SYSCLK_OE;
	if (!pdata->combined_req)
		reg |= TEGRA_POWER_PWRREQ_OE;
	pmc_32kwritel(reg, PMC_CTRL);

	if (pdata->sysclkreq_gpio) {
		reg = readl(pmc + PMC_DPAD_ORIDE);
		reg &= ~TEGRA_DPAD_ORIDE_SYS_CLK_REQ;
		pmc_32kwritel(reg, PMC_DPAD_ORIDE);
	}

	if (pdata->suspend_mode == TEGRA_SUSPEND_LP0)
		tegra_lp0_suspend_init();

	if (register_pm_notifier(&suspend_notifier))
		pr_err("%s: Failed to register suspend notifier\n", __func__);

	if (register_pm_notifier(&suspend_clear_status))
		pr_err("%s: Failed to register suspend status clearer\n", __func__);

	suspend_set_ops(&tegra_suspend_ops);

	/* Create /sys/power/suspend/type */
	suspend_kobj = kobject_create_and_add("suspend", power_kobj);
	if (suspend_kobj) {
		if (sysfs_create_file(suspend_kobj,
						&suspend_mode_attribute.attr))
			pr_err("%s: sysfs_create_file suspend type failed!\n",
								__func__);
		if (sysfs_create_file(suspend_kobj,
					&suspend_resume_time_attribute.attr))
			pr_err("%s: sysfs_create_file resume_time failed!\n",
								__func__);
		if (sysfs_create_file(suspend_kobj,
					&suspend_time_attribute.attr))
			pr_err("%s: sysfs_create_file suspend_time failed!\n",
								__func__);
	}

	/* clear io dpd settings before kernel */
	tegra_bl_io_dpd_cleanup();

#endif
	if (plat->suspend_mode == TEGRA_SUSPEND_NONE)
		tegra_pd_in_idle(false);

	current_suspend_mode = plat->suspend_mode;
}

unsigned long debug_uart_port_base;
EXPORT_SYMBOL(debug_uart_port_base);

static int tegra_debug_uart_suspend(void)
{
	void __iomem *uart;
	u32 lcr;

	if (!debug_uart_port_base)
		return 0;

	uart = IO_ADDRESS(debug_uart_port_base);

	lcr = readb(uart + UART_LCR * 4);

	tegra_sctx.uart[0] = lcr;
	tegra_sctx.uart[1] = readb(uart + UART_MCR * 4);

	/* DLAB = 0 */
	writeb(lcr & ~UART_LCR_DLAB, uart + UART_LCR * 4);

	tegra_sctx.uart[2] = readb(uart + UART_IER * 4);

	/* DLAB = 1 */
	writeb(lcr | UART_LCR_DLAB, uart + UART_LCR * 4);

	tegra_sctx.uart[3] = readb(uart + UART_DLL * 4);
	tegra_sctx.uart[4] = readb(uart + UART_DLM * 4);

	writeb(lcr, uart + UART_LCR * 4);

	return 0;
}

static void tegra_debug_uart_resume(void)
{
	void __iomem *uart;
	u32 lcr;

	if (!debug_uart_port_base)
		return;

	uart = IO_ADDRESS(debug_uart_port_base);

	lcr = tegra_sctx.uart[0];

	writeb(tegra_sctx.uart[1], uart + UART_MCR * 4);

	/* DLAB = 0 */
	writeb(lcr & ~UART_LCR_DLAB, uart + UART_LCR * 4);

	writeb(UART_FCR_ENABLE_FIFO | UART_FCR_T_TRIG_01 | UART_FCR_R_TRIG_01,
			uart + UART_FCR * 4);

	writeb(tegra_sctx.uart[2], uart + UART_IER * 4);

	/* DLAB = 1 */
	writeb(lcr | UART_LCR_DLAB, uart + UART_LCR * 4);

	writeb(tegra_sctx.uart[3], uart + UART_DLL * 4);
	writeb(tegra_sctx.uart[4], uart + UART_DLM * 4);

	writeb(lcr, uart + UART_LCR * 4);
}

static struct syscore_ops tegra_debug_uart_syscore_ops = {
	.suspend = tegra_debug_uart_suspend,
	.resume = tegra_debug_uart_resume,
	.save = tegra_debug_uart_suspend,
	.restore = tegra_debug_uart_resume,
};

struct clk *debug_uart_clk;
EXPORT_SYMBOL(debug_uart_clk);

void tegra_console_uart_suspend(void)
{
	if (console_suspend_enabled && debug_uart_clk)
		tegra_clk_disable_unprepare(debug_uart_clk);
}

void tegra_console_uart_resume(void)
{
	if (console_suspend_enabled && debug_uart_clk)
		tegra_clk_prepare_enable(debug_uart_clk);
}

static int tegra_debug_uart_syscore_init(void)
{
	register_syscore_ops(&tegra_debug_uart_syscore_ops);
	return 0;
}
arch_initcall(tegra_debug_uart_syscore_init);

#ifdef CONFIG_ARM_ARCH_TIMER

static u32 tsc_suspend_start;
static u32 tsc_resume_start;

#define pmc_writel(value, reg) \
		writel(value, pmc + (reg))
#define pmc_readl(reg) \
		readl(pmc + (reg))

#define PMC_DPD_ENABLE			0x24
#define PMC_DPD_ENABLE_TSC_MULT_ENABLE	(1 << 1)
#if defined(CONFIG_ARCH_TEGRA_12x_SOC)
#define PMC_DPD_ENABLE_ON		(1 << 0)
#endif

#define PMC_TSC_MULT			0x2b4
#define PMC_TSC_MULT_FREQ_STS		(1 << 16)

#define TSC_TIMEOUT_US			32

void tegra_tsc_suspend(void)
{
	u32 reg = pmc_readl(PMC_DPD_ENABLE);
	BUG_ON(reg & PMC_DPD_ENABLE_TSC_MULT_ENABLE);
	reg |= PMC_DPD_ENABLE_TSC_MULT_ENABLE;
	pmc_writel(reg, PMC_DPD_ENABLE);
	tsc_suspend_start = timer_readl(TIMERUS_CNTR_1US);
}

void tegra_tsc_resume(void)
{
	u32 reg = pmc_readl(PMC_DPD_ENABLE);
	BUG_ON(!(reg & PMC_DPD_ENABLE_TSC_MULT_ENABLE));
	reg &= ~PMC_DPD_ENABLE_TSC_MULT_ENABLE;
#if defined(CONFIG_ARCH_TEGRA_12x_SOC)
	/*
	 * FIXME: T12x SW WAR -
	 * Resume ensures DPD_ENABLE is 0 when writing
	 * TSC_MULT_ENABLE, else PMC wake status gets reset
	 */
	reg &= ~PMC_DPD_ENABLE_ON;
#endif
	pmc_writel(reg, PMC_DPD_ENABLE);
	tsc_resume_start = timer_readl(TIMERUS_CNTR_1US);
}

void tegra_tsc_wait_for_suspend(void)
{
	while ((timer_readl(TIMERUS_CNTR_1US) - tsc_suspend_start) <
		TSC_TIMEOUT_US) {
		if (pmc_readl(PMC_TSC_MULT) & PMC_TSC_MULT_FREQ_STS)
			break;
		cpu_relax();
	}
}

void tegra_tsc_wait_for_resume(void)
{
	while ((timer_readl(TIMERUS_CNTR_1US) - tsc_resume_start) <
		TSC_TIMEOUT_US) {
		if (!(pmc_readl(PMC_TSC_MULT) & PMC_TSC_MULT_FREQ_STS))
			break;
		cpu_relax();
	}
}
#endif
