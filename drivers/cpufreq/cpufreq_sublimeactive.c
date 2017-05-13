/*
 *  drivers/cpufreq/cpufreq_sublimeactive.c
 *
 *  Copyright (C)  2001 Russell King
 *            (C)  2003 Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>.
 *                      Jun Nakajima <jun.nakajima@intel.com>
 *            (C)  2009 Alexander Clouter <alex@digriz.org.uk>
 *            (C)  2015 Dela Anthonio
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/display_state.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/touchboost.h>

#include "cpufreq_governor.h"

/* Sublime_active governor macros */
#define DEF_FREQ_UP_THRESHOLD		(80)
#define DEF_FREQ_DOWN_THRESHOLD		(40)
#define DEF_TOUCHBOOST_MIN_FREQ		(1428000)
#define DEF_TOUCHBOOST_DURATION		(500 * USEC_PER_MSEC)	// 500 ms
#define MAX_TOUCHBOOST_DURATION		(2000 * USEC_PER_MSEC)	// 2 sec
#define DISPLAY_ON_SAMPLING_RATE	(15 * USEC_PER_MSEC)	// 15 ms
#define DISPLAY_OFF_SAMPLING_RATE	(500 * USEC_PER_MSEC)	// 500 ms
#define IGNORE_NICE_LOAD_ON		(1)
#define IGNORE_NICE_LOAD_OFF		(0)
#define MAX_LOAD			(100)
#define MIN_LOAD			(11)

static DEFINE_PER_CPU(struct sa_cpu_dbs_info_s, sa_cpu_dbs_info);

static void sa_def_check_cpu(int cpu, unsigned int load);

static void (*__sa_check_cpu)(int cpu, unsigned int load) = sa_def_check_cpu;

static void sa_check_cpu(int cpu, unsigned int load)
{
	__sa_check_cpu(cpu, load);
}

/**
 * This function is designed for efficient frequency scaling at low sampling rates.
 * For example, if the current busy time exceeds 80% (default), the current frequency
 * will be averaged with the max frequency instead of shooting to the max frequency
 * right away. This helps to make the governor responsive without excessive power use.
 * Likewise if the current busy time is less than 40% (default), the current frequency
 * will be averaged with the minimum frequency.
 * @ cpu the cpu whose frequency should be set.
 * @ load an int between 0 and 100 that represents how busy the cpu is.
 */
static void sa_def_check_cpu(int cpu, unsigned int load)
{
	struct sa_cpu_dbs_info_s const *dbs_info =
		&per_cpu(sa_cpu_dbs_info, cpu);
	struct cpufreq_policy *policy = dbs_info->cdbs.cur_policy;
	struct dbs_data *const dbs_data = policy->governor_data;
	const struct sa_dbs_tuners *const sa_tuners = dbs_data->tuners;
	const unsigned int prev_load = dbs_info->cdbs.prev_load;
	unsigned int freq_target = 0;

	/* Check for frequency decrease */
	if (load < sa_tuners->down_threshold) {
		if (touchboost_is_enabled(sa_tuners->touchboost_dur)) {
			freq_target = sa_tuners->touchboost_min_freq;
			__cpufreq_driver_target(policy, freq_target,
						CPUFREQ_RELATION_H);
		} else {
			freq_target = (policy->cur + policy->min) / 2;
			__cpufreq_driver_target(policy, freq_target,
						CPUFREQ_RELATION_H);
		}

	}

	/* Check for frequency increase */
	else if (load >= max(sa_tuners->up_threshold, prev_load)) {
		freq_target = (policy->max + policy->cur) / 2;
		__cpufreq_driver_target(policy, freq_target,
					CPUFREQ_RELATION_L);
	}
}

/**
 * Scale the cpu freq proportional to the load. This is used when the display is off
 * Due to the interval between cpu samples being increased. There will not be as
 * many spikes in the load that occurs when the sampling inverval is lower.
 * @ cpu the cpu whose frequency should be set.
 * @ load an int between 0 and 100 that represents how busy the cpu is.
 */

static void sa_display_off_check_cpu(int cpu, unsigned int load)
{
	struct sa_cpu_dbs_info_s const *dbs_info =
		&per_cpu(sa_cpu_dbs_info, cpu);
	struct cpufreq_policy *policy = dbs_info->cdbs.cur_policy;
	unsigned int freq_target = policy->min;
	freq_target += (policy->max - policy->min) * load / MAX_LOAD;
	__cpufreq_driver_target(policy, freq_target, CPUFREQ_RELATION_H);
}

static void sa_dbs_timer(struct work_struct *work)
{
	struct sa_cpu_dbs_info_s *dbs_info =
		container_of(work, struct sa_cpu_dbs_info_s, cdbs.work.work);
	unsigned int cpu = dbs_info->cdbs.cur_policy->cpu;
	struct sa_cpu_dbs_info_s *core_dbs_info =
		&per_cpu(sa_cpu_dbs_info, cpu);
	struct dbs_data *dbs_data = dbs_info->cdbs.cur_policy->governor_data;
	struct sa_dbs_tuners *const sa_tuners = dbs_data->tuners;
	int delay = delay_for_sampling_rate(sa_tuners->sampling_rate);
	bool modify_all = true;

	mutex_lock(&core_dbs_info->cdbs.timer_mutex);
	if (!need_load_eval(&core_dbs_info->cdbs, sa_tuners->sampling_rate))
		modify_all = false;
	else
		dbs_check_cpu(dbs_data, cpu);

	gov_queue_work(dbs_data, dbs_info->cdbs.cur_policy, delay, modify_all);
	mutex_unlock(&core_dbs_info->cdbs.timer_mutex);
}

/**
 * Set the sampling rate based on the state of the display.
 * The sampling rate is lowered when the display is off and increased when
 * the display is on.
 */
static int display_notifier(struct notifier_block *nb,
			    unsigned long display_state, void *data)
{
	unsigned int sampling_rate;
	unsigned int ignore_nice_load;
	unsigned int cpu;

	switch (display_state) {
	case DISPLAY_ON:
		__sa_check_cpu = sa_def_check_cpu;
		sampling_rate = DISPLAY_ON_SAMPLING_RATE;
		ignore_nice_load = IGNORE_NICE_LOAD_OFF;
		break;

	case DISPLAY_OFF:
		__sa_check_cpu = sa_display_off_check_cpu;
		sampling_rate = DISPLAY_OFF_SAMPLING_RATE;
		ignore_nice_load = IGNORE_NICE_LOAD_ON;
		break;

	default:
		sampling_rate = DISPLAY_ON_SAMPLING_RATE;
		ignore_nice_load = IGNORE_NICE_LOAD_OFF;

#ifdef CONFIG_TEGRA_DSI_DEBUG
		printk(KERN_WARN "Invalid display state: %u\n", display_state);
#endif
	}

#ifdef CONFIG_TEGRA_DSI_DEBUG
	printk(KERN_INFO "Sampling rate set to %u\n", sampling_rate);
#endif

	for_each_possible_cpu(cpu)
	{
		struct sa_cpu_dbs_info_s *const dbs_info =
			&per_cpu(sa_cpu_dbs_info, cpu);
		struct cpufreq_policy *const policy = dbs_info->cdbs.cur_policy;
		struct dbs_data *const dbs_data = policy->governor_data;
		struct sa_dbs_tuners *const sa_tuners = dbs_data->tuners;
		sa_tuners->sampling_rate = sampling_rate;
		sa_tuners->ignore_nice_load = ignore_nice_load;
	}
	return NOTIFY_OK;
}

static struct notifier_block display_nb = {
	.notifier_call = display_notifier,
};

/************************** sysfs interface ************************/
static struct common_dbs_data sa_dbs_cdata;

static ssize_t store_up_threshold(struct dbs_data *dbs_data, const char *buf,
				  size_t count)
{
	struct sa_dbs_tuners *const sa_tuners = dbs_data->tuners;
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > MAX_LOAD || input <= sa_tuners->down_threshold)
		return -EINVAL;

	sa_tuners->up_threshold = input;
	return count;
}

static ssize_t store_down_threshold(struct dbs_data *dbs_data, const char *buf,
				    size_t count)
{
	struct sa_dbs_tuners *const sa_tuners = dbs_data->tuners;
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input < MIN_LOAD || input >= sa_tuners->up_threshold)
		return -EINVAL;

	sa_tuners->down_threshold = input;
	return count;
}

static ssize_t store_touchboost_min_freq(struct dbs_data *dbs_data,
					 const char *buf, size_t count)
{
	struct sa_dbs_tuners *const sa_tuners = dbs_data->tuners;

	unsigned int input;
	unsigned int cpu;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1)
		return -EINVAL;

	for_each_possible_cpu(cpu)
	{
		struct sa_cpu_dbs_info_s *const dbs_info =
			&per_cpu(sa_cpu_dbs_info, cpu);
		const struct cpufreq_policy *const policy =
			dbs_info->cdbs.cur_policy;

		if (input < policy->min)
			input = policy->min;

		else if (input > policy->max)
			input = policy->max;
	}

	sa_tuners->touchboost_min_freq = input;
	return count;
}

static ssize_t store_touchboost_dur(struct dbs_data *dbs_data, const char *buf,
				    size_t count)
{
	struct sa_dbs_tuners *const sa_tuners = dbs_data->tuners;

	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > MAX_TOUCHBOOST_DURATION)
		return -EINVAL;

	sa_tuners->touchboost_dur = input;
	return count;
}

show_one(sa, sampling_rate);
show_store_one(sa, up_threshold);
show_store_one(sa, down_threshold);
show_store_one(sa, touchboost_min_freq);
show_store_one(sa, touchboost_dur);

gov_sys_pol_attr_ro(sampling_rate);
gov_sys_pol_attr_rw(up_threshold);
gov_sys_pol_attr_rw(down_threshold);
gov_sys_pol_attr_rw(touchboost_min_freq);
gov_sys_pol_attr_rw(touchboost_dur);

static struct attribute *dbs_attributes_gov_sys[] = {
	&sampling_rate_gov_sys.attr,  &up_threshold_gov_sys.attr,
	&down_threshold_gov_sys.attr, &touchboost_min_freq_gov_sys.attr,
	&touchboost_dur_gov_sys.attr, NULL};

static struct attribute_group sa_attr_group_gov_sys = {
	.attrs = dbs_attributes_gov_sys, .name = "sublime_active",
};

static struct attribute *dbs_attributes_gov_pol[] = {
	&sampling_rate_gov_pol.attr,  &up_threshold_gov_pol.attr,
	&down_threshold_gov_pol.attr, &touchboost_min_freq_gov_pol.attr,
	&touchboost_dur_gov_pol.attr, NULL};

static struct attribute_group sa_attr_group_gov_pol = {
	.attrs = dbs_attributes_gov_pol, .name = "sublime_active",
};

/************************** sysfs end ************************/

static int sa_init(struct dbs_data *dbs_data)
{
	struct sa_dbs_tuners *tuners;

	tuners = kzalloc(sizeof(struct sa_dbs_tuners), GFP_KERNEL);
	if (!tuners) {
		pr_err("%s: kzalloc failed\n", __func__);
		return -ENOMEM;
	}
	tuners->up_threshold = DEF_FREQ_UP_THRESHOLD;
	tuners->down_threshold = DEF_FREQ_DOWN_THRESHOLD;
	tuners->touchboost_min_freq = DEF_TOUCHBOOST_MIN_FREQ;
	tuners->touchboost_dur = DEF_TOUCHBOOST_DURATION;
	tuners->ignore_nice_load = IGNORE_NICE_LOAD_OFF;

	dbs_data->tuners = tuners;
	dbs_data->min_sampling_rate = DISPLAY_ON_SAMPLING_RATE;
	mutex_init(&dbs_data->mutex);
	display_state_register_notifier(&display_nb);
	return 0;
}

static void sa_exit(struct dbs_data *dbs_data)
{
	display_state_unregister_notifier(&display_nb);
	kfree(dbs_data->tuners);
}

define_get_cpu_dbs_routines(sa_cpu_dbs_info);

static struct common_dbs_data sa_dbs_cdata = {
	.governor = GOV_SUBLIMEACTIVE,
	.attr_group_gov_sys = &sa_attr_group_gov_sys,
	.attr_group_gov_pol = &sa_attr_group_gov_pol,
	.get_cpu_cdbs = get_cpu_cdbs,
	.get_cpu_dbs_info_s = get_cpu_dbs_info_s,
	.gov_dbs_timer = sa_dbs_timer,
	.gov_check_cpu = sa_check_cpu,
	.gov_ops = NULL,
	.init = sa_init,
	.exit = sa_exit,
};

static int sa_cpufreq_governor_dbs(struct cpufreq_policy *policy,
				   unsigned int event)
{
	return cpufreq_governor_dbs(policy, &sa_dbs_cdata, event);
}

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_SUBLIMEACTIVE
static
#endif
	struct cpufreq_governor cpufreq_gov_sublimeactive = {
		.name = "sublime_active",
		.governor = sa_cpufreq_governor_dbs,
		.max_transition_latency = TRANSITION_LATENCY_LIMIT,
		.owner = THIS_MODULE,
};

static int __init cpufreq_gov_dbs_init(void)
{
	return cpufreq_register_governor(&cpufreq_gov_sublimeactive);
}

static void __exit cpufreq_gov_dbs_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_sublimeactive);
}

MODULE_AUTHOR("Dela Anthonio");
MODULE_DESCRIPTION("'cpufreq_sublime' - A dynamic CPU frequency governor for"
		   "Low latency frequency transition capable processors. "
		   "This governor is optimized for devices which have a"
		   "touchscreen and limited battery capacity");
MODULE_LICENSE("GPL");

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_SUBLIMEACTIVE
fs_initcall(cpufreq_gov_dbs_init);
#else
module_init(cpufreq_gov_dbs_init);
#endif
module_exit(cpufreq_gov_dbs_exit);
