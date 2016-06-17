/*
 * arch/arm/mach-tegra/tegra_simon.c
 *
 * Copyright (c) 2013-2014, NVIDIA CORPORATION. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>
#include <linux/thermal.h>
#include <linux/regulator/consumer.h>

#include "tegra_simon.h"
#include <linux/platform/tegra/clock.h>
#include <linux/platform/tegra/dvfs.h>
#include "pm.h"
#include <linux/platform/tegra/tegra_cl_dvfs.h>

static DEFINE_MUTEX(simon_lock);
static RAW_NOTIFIER_HEAD(simon_nh);
static void tegra_simon_grade_notify(struct work_struct *work);

static u32 grading_sec = TEGRA_SIMON_GRADING_INTERVAL_SEC;
static u32 timeout_sec = TEGRA_SIMON_GRADING_TIMEOUT_SEC;

static struct tegra_simon_grader simon_graders[TEGRA_SIMON_DOMAIN_NUM] = {
	[TEGRA_SIMON_DOMAIN_CPU] = {
		.domain_name = "cpu",
		.domain = TEGRA_SIMON_DOMAIN_CPU,
	},
	[TEGRA_SIMON_DOMAIN_GPU] = {
		.domain_name = "gpu",
		.domain = TEGRA_SIMON_DOMAIN_GPU,
	},
};

static void settle_delay(struct tegra_simon_grader *grader)
{
	int us = grader->desc->settle_us;

	if (us < MAX_UDELAY_MS * 1000)
		udelay(us);
	else
		usleep_range(us, us + 100);
}

static inline void mod_wdt_on_grade(struct tegra_simon_grader *grader)
{
	if (grader->grade) {
		/* restart WDT while at high grade */
		struct timespec ts = {timeout_sec, 0};
		mod_timer(&grader->grade_wdt,
			  jiffies + timespec_to_jiffies(&ts));
	}
}

static void tegra_simon_reset_grade(unsigned long data)
{
	unsigned long flags;
	struct tegra_simon_grader *grader = (struct tegra_simon_grader *)data;

	pr_info("%s: %s grade = 0\n", __func__, grader->domain_name);

	spin_lock_irqsave(&grader->grade_lock, flags);
	grader->grade = 0;
	spin_unlock_irqrestore(&grader->grade_lock, flags);

	schedule_work(&grader->grade_update_work);
}

static inline void mod_grading_timer_on_grade(struct tegra_simon_grader *grader)
{
	if (grader->grade) {
		/* restart timer while at high grade */
		struct timespec ts = {grading_sec, 0};
		mod_timer(&grader->grade_timer,
			  jiffies + timespec_to_jiffies(&ts));
	}
}

static void tegra_simon_restart_grader(unsigned long data)
{
	unsigned long flags;
	struct tegra_simon_grader *grader = (struct tegra_simon_grader *)data;

	spin_lock_irqsave(&grader->grade_lock, flags);
	if (grader->grade) {
		/* restart grading while at high grade */
		grader->stop_grading = false;
		pr_info("%s: %s grader restarted\n",
			__func__, grader->domain_name);
	}
	spin_unlock_irqrestore(&grader->grade_lock, flags);
}

static void tegra_simon_grade_set(struct tegra_simon_grader *grader, int grade)
{
	unsigned long flags;

	spin_lock_irqsave(&grader->grade_lock, flags);

	/* once grading is successful, stop grading, and restart timers */
	grader->stop_grading = true;

	if (grader->grade == grade) {
		mod_wdt_on_grade(grader);
		mod_grading_timer_on_grade(grader);
		spin_unlock_irqrestore(&grader->grade_lock, flags);
		return;
	}
	grader->grade = grade;
	mod_wdt_on_grade(grader);
	mod_grading_timer_on_grade(grader);
	spin_unlock_irqrestore(&grader->grade_lock, flags);

	schedule_work(&grader->grade_update_work);
}

/*
 * GPU grading is implemented within vdd_gpu post-change notification chain that
 * guarantees constant voltage during grading. First grading after boot can be
 * executed anytime set voltage is below specified threshold, next grading is
 * always separated by the grading interval from the last successful grading.
 */
static int tegra_simon_gpu_grading_cb(
	struct notifier_block *nb, unsigned long event, void *v)
{
	int mv = (int)((long)v);
	struct tegra_simon_grader *grader = container_of(
		nb, struct tegra_simon_grader, grading_condition_nb);
	unsigned long t;
	int grade = 0;

	if (!(event & REGULATOR_EVENT_OUT_POSTCHANGE))
		return NOTIFY_DONE;

	if (grader->stop_grading)
		return NOTIFY_OK;

	mv = (mv > 0) ? mv / 1000 : mv;
	if ((mv <= 0) || (mv > grader->desc->grading_mv_max))
		return NOTIFY_OK;

	if (grader->tzd->ops->get_temp(grader->tzd, &t)) {
		pr_err("%s: Failed to get %s temperature\n",
		       __func__, grader->domain_name);
		return NOTIFY_OK;
	}

	if (t < grader->desc->grading_temperature_min)
		return NOTIFY_OK;

	if (grader->desc->grade_simon_domain) {
		settle_delay(grader);	/* delay for voltage to settle */
		grade = grader->desc->grade_simon_domain(grader->domain, mv, t);
		if (grade < 0) {
			pr_err("%s: Failed to grade %s\n",
			       __func__, grader->domain_name);
			return NOTIFY_OK;
		}
	}

	grader->last_grading = ktime_get();
	tegra_simon_grade_set(grader, grade);
	pr_info("%s: graded %s: v = %d, t = %lu, grade = %d\n",
		__func__, grader->domain_name, mv, t, grade);
	return NOTIFY_OK;
}

static int __init tegra_simon_init_gpu(void)
{
	int ret;
	struct tegra_simon_grader *grader =
		&simon_graders[TEGRA_SIMON_DOMAIN_GPU];

	spin_lock_init(&grader->grade_lock);
	setup_timer(&grader->grade_timer, tegra_simon_restart_grader,
		    (unsigned long)grader);
	setup_timer(&grader->grade_wdt, tegra_simon_reset_grade,
		    (unsigned long)grader);
	INIT_WORK(&grader->grade_update_work, tegra_simon_grade_notify);

	grader->tzd = thermal_zone_device_find_by_name("GPU-therm");
	if (!grader->tzd) {
		pr_err("%s: Failed to find %s thermal zone\n",
		       __func__, grader->domain_name);
		return -ENOENT;
	}

	grader->grading_condition_nb.notifier_call = tegra_simon_gpu_grading_cb;
	ret = tegra_dvfs_rail_register_notifier(
		tegra_gpu_rail, &grader->grading_condition_nb);
	if (ret) {
		pr_err("%s: Failed to register %s dvfs rail notifier\n",
		       __func__, grader->domain_name);
		return ret;
	}

	return 0;
}

/*
 * CPU grading is implemented within CPU rate post-change notification chain
 * that guarantees constant frequency during grading. Grading is executed only
 * when running on G-CPU, with DFLL as clock source, at rate low enough for DFLL
 * to be close to saturation at minimum voltage. To avoid still possible closed
 * loop voltage fluctuation for sure, DFLL maximum limit is clamped to minimum
 * during measurements.
 *
 * First grading after boot can be executed anytime the conditions above are
 * met, next grading is always separated by the grading interval from the last
 * successful grading.
 */
extern int tegra_cl_dvfs_clamp_at_vmin(struct tegra_cl_dvfs *cld, bool clamp);
static int tegra_simon_cpu_grading_cb(
	struct notifier_block *nb, unsigned long rate, void *v)
{
	struct tegra_simon_grader *grader = container_of(
		nb, struct tegra_simon_grader, grading_condition_nb);
	struct tegra_cl_dvfs *cld;

	unsigned long t;
	int mv;
	int grade = 0;

	if (grader->stop_grading)
		return NOTIFY_OK;

	if (is_lp_cluster() || (rate > grader->desc->grading_rate_max) ||
	    !tegra_dvfs_rail_is_dfll_mode(tegra_cpu_rail))
		return NOTIFY_OK;

	if (grader->tzd->ops->get_temp(grader->tzd, &t)) {
		pr_err("%s: Failed to get %s temperature\n",
		       __func__, grader->domain_name);
		return NOTIFY_OK;
	}

	if (t < grader->desc->grading_temperature_min)
		return NOTIFY_OK;

	cld = tegra_dfll_get_cl_dvfs_data(
		clk_get_parent(clk_get_parent(grader->clk)));
	if (IS_ERR(cld)) {
		pr_err("%s: Failed to get cl_dvfs data for %s\n",
		       __func__, grader->domain_name);
		return NOTIFY_OK;
	}

	mv = tegra_cl_dvfs_clamp_at_vmin(cld, true);
	if (mv < 0) {
		pr_err("%s: Failed to clamp %s voltage\n",
		       __func__, grader->domain_name);
		return NOTIFY_OK;
	}

	if (grader->desc->grade_simon_domain) {
		settle_delay(grader);	/* delay for voltage to settle */
		grade = grader->desc->grade_simon_domain(grader->domain, mv, t);
		if (grade < 0) {
			pr_err("%s: Failed to grade %s\n",
			       __func__, grader->domain_name);
			tegra_cl_dvfs_clamp_at_vmin(cld, false);
			return NOTIFY_OK;
		}

	}
	tegra_cl_dvfs_clamp_at_vmin(cld, false);

	grader->last_grading = ktime_get();
	tegra_simon_grade_set(grader, grade);
	pr_info("%s: graded %s: v = %d, t = %lu, grade = %d\n",
		__func__, grader->domain_name, mv, t, grade);
	return NOTIFY_OK;
}

static int __init tegra_simon_init_cpu(void)
{
	struct tegra_simon_grader *grader =
		&simon_graders[TEGRA_SIMON_DOMAIN_CPU];
	struct clk *c;
	int r;

	spin_lock_init(&grader->grade_lock);
	setup_timer(&grader->grade_timer, tegra_simon_restart_grader,
		    (unsigned long)grader);
	setup_timer(&grader->grade_wdt, tegra_simon_reset_grade,
		    (unsigned long)grader);
	INIT_WORK(&grader->grade_update_work, tegra_simon_grade_notify);

	grader->tzd = thermal_zone_device_find_by_name("CPU-therm");
	if (!grader->tzd) {
		pr_err("%s: Failed to find %s thermal zone\n",
		       __func__, grader->domain_name);
		return -ENOENT;
	}

	c = clk_get_sys("tegra_simon", "cpu");
	if (IS_ERR(c)) {
		pr_err("%s: Failed to get %s clock\n",
		       __func__, grader->domain_name);
		return -ENOENT;
	}

	grader->grading_condition_nb.notifier_call = tegra_simon_cpu_grading_cb;
	r = tegra_register_clk_rate_notifier(c, &grader->grading_condition_nb);
	if (r) {
		pr_err("%s: Failed to register for %s rate change notify\n",
		       __func__, c->name);
		return r;
	}
	grader->clk = c;

	return 0;
}

/*
 * Interface for grader driver to add grader description
 */
int tegra_simon_add_grader(struct tegra_simon_grader_desc *desc)
{
	int ret;
	struct tegra_simon_grader *grader;

	if (!desc || (desc->domain >= TEGRA_SIMON_DOMAIN_NUM)) {
		pr_err("%s: Invalid grader data\n", __func__);
		return -EINVAL;
	}

	grader = &simon_graders[desc->domain];
	if (grader->desc) {
		pr_err("%s: Duplicate grader for %s\n",
		       __func__, grader->domain_name);
		return -EINVAL;
	}

	set_mb(grader->desc, desc);

	switch (grader->domain) {
	case TEGRA_SIMON_DOMAIN_CPU:
		ret = tegra_simon_init_cpu();
		break;
	case TEGRA_SIMON_DOMAIN_GPU:
		ret = tegra_simon_init_gpu();
		break;
	default:
		pr_err("%s: Grader for %s is not supported\n",
		       __func__, grader->domain_name);
		return -EINVAL;
	}

	if (ret) {
		grader->desc = NULL;
		pr_err("%s: Failed to initialize grader for %s\n",
		       __func__, grader->domain_name);
		return ret;
	}

	return 0;
}

/*
 * Grade notification chain
 */
int tegra_register_simon_notifier(struct notifier_block *nb)
{
	int ret;

	mutex_lock(&simon_lock);
	ret = raw_notifier_chain_register(&simon_nh, nb);
	mutex_unlock(&simon_lock);
	return ret;
}

void tegra_unregister_simon_notifier(struct notifier_block *nb)
{
	mutex_lock(&simon_lock);
	raw_notifier_chain_unregister(&simon_nh, nb);
	mutex_unlock(&simon_lock);
}

static void grade_notify(struct tegra_simon_grader *grader)
{
	mutex_lock(&simon_lock);
	raw_notifier_call_chain(&simon_nh, grader->grade,
				(void *)((long)grader->domain));
	mutex_unlock(&simon_lock);
}

static void tegra_simon_grade_notify(struct work_struct *work)
{
	struct tegra_simon_grader *grader = container_of(
		work, struct tegra_simon_grader, grade_update_work);

	grade_notify(grader);
}

#ifdef CONFIG_DEBUG_FS

static int grade_get(void *data, u64 *val)
{
	struct tegra_simon_grader *grader = data;

	if (grader->domain >= TEGRA_SIMON_DOMAIN_NUM) {
		*val = -EINVAL;
		return -EINVAL;
	}

	*val = grader->grade;
	return 0;
}
static int grade_set(void *data, u64 val)
{
	int grade = (int)val;
	struct tegra_simon_grader *grader = data;

	if (grader->domain >= TEGRA_SIMON_DOMAIN_NUM)
		return -EINVAL;

	if (!grader->desc && (grader->grade != grade)) {
		grader->grade = grade;
		grade_notify(grader);
	} else if (grader->desc) {
		tegra_simon_grade_set(grader, grade);
	}
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(grade_fops, grade_get, grade_set, "%llu\n");

static int __init simon_debugfs_init_domain(struct dentry *dir,
					    struct tegra_simon_grader *grader)
{
	struct dentry *d;

	d = debugfs_create_dir(grader->domain_name, dir);
	if (!d)
		return -ENOMEM;

	if (!debugfs_create_file("grade", S_IWUSR | S_IRUGO, d,
				 (void *)grader, &grade_fops))
		return -ENOMEM;

	if (!debugfs_create_bool("grading_stopped", S_IRUGO, d,
				(u32 *)&grader->stop_grading))
		return -ENOMEM;

	return 0;
}

static int __init simon_debugfs_init(void)
{
	int i;
	struct tegra_simon_grader *grader;
	struct dentry *dir;

	dir = debugfs_create_dir("tegra_simon", NULL);
	if (!dir)
		return -ENOMEM;

	if (!debugfs_create_u32("grading_sec", S_IWUSR | S_IRUGO, dir,
				&grading_sec))
		goto err_out;

	if (!debugfs_create_u32("timeout_sec", S_IWUSR | S_IRUGO, dir,
				&timeout_sec))
		goto err_out;

	for (i = 0; i < TEGRA_SIMON_DOMAIN_NUM; i++) {
		grader = &simon_graders[i];

		if (!grader->domain_name)
			continue;

		if (simon_debugfs_init_domain(dir, grader))
			goto err_out;
	}

	return 0;

err_out:
	debugfs_remove_recursive(dir);
	return -ENOMEM;
}

late_initcall(simon_debugfs_init);
#endif

/* FIXME: Add fake graders - to be removed when actual graders are implemnted */
#if CONFIG_ARCH_TEGRA_12x_SOC
#ifndef CONFIG_ARCH_TEGRA_13x_SOC
static int fake_grader(int domain, int mv, int temperature)
{
	return 0;	/* safe low grade */
}

static struct tegra_simon_grader_desc gpu_grader_desc = {
	.domain = TEGRA_SIMON_DOMAIN_GPU,
	.grading_mv_max = 850,
	.grading_temperature_min = 20000,
	.settle_us = 3000,
	.grade_simon_domain = fake_grader,
};

static struct tegra_simon_grader_desc cpu_grader_desc = {
	.domain = TEGRA_SIMON_DOMAIN_CPU,
	.grading_rate_max = 850000000,
	.grading_temperature_min = 20000,
	.settle_us = 3000,
	.grade_simon_domain = fake_grader,
};

static int __init tegra_simon_add_graders(void)
{
	tegra_simon_add_grader(&gpu_grader_desc);
	tegra_simon_add_grader(&cpu_grader_desc);
	return 0;
}
late_initcall_sync(tegra_simon_add_graders);
#endif
#endif
