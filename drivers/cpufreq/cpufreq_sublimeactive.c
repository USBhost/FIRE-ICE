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

#include <linux/cpufreq.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kernel_stat.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/percpu-defs.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/types.h>
#include <linux/touchboost.h>

#include "cpufreq_governor.h"

/* Sublime_active governor macros */
#define DEF_FREQUENCY_UP_THRESHOLD           (75)
#define DEF_FREQUENCY_DOWN_THRESHOLD         (30)
#define MAXIMUM_LOAD                         (100)
#define MINIMUM_LOAD                         (11)
#define DEF_INPUT_EVENT_MIN_FREQUENCY        (1428000)
#define DEF_INPUT_EVENT_DURATION             (50000)
#define MAX_INPUT_EVENT_DURATION             (200000)
#define RESISTANCE_OFFSET                    (1)
#define MIN_FREQUENCY_DELTA                  (10000)
#define MINIMUM_SAMPLING_RATE                (15000)

static DEFINE_PER_CPU(struct sa_cpu_dbs_info_s, sa_cpu_dbs_info);


/*
 * Every sampling_rate, if current idle time is less than 30% (default),
 * try to increase the frequency. Every sampling_rate if the current idle
 * time is more than 70% (default), try to decrease the frequency.
 */
static void sa_check_cpu(int cpu, unsigned int load)
{
	struct sa_cpu_dbs_info_s const *dbs_info = &per_cpu(sa_cpu_dbs_info, cpu);
	struct cpufreq_policy *policy = dbs_info->cdbs.cur_policy;
	struct dbs_data* const dbs_data = policy->governor_data;
	const struct sa_dbs_tuners* const sa_tuners = dbs_data->tuners;
	const unsigned int prev_load = dbs_info->cdbs.prev_load;
	const unsigned int freq_cur = policy->cur;
	unsigned int freq_target = 0;
	const bool input_event = input_event_boost(sa_tuners->input_event_duration);

	/* Check for frequency decrease */
	if (load < sa_tuners->down_threshold) {
		const unsigned int freq_min = policy->min;

		// break out early if the frequency is set to the minimum
		if (freq_cur == freq_min)
			return;

		if (input_event)
			freq_target = sa_tuners->input_event_min_freq;

		else
			freq_target = (freq_cur + freq_min) >> RESISTANCE_OFFSET;

		__cpufreq_driver_target(policy, freq_target,
					CPUFREQ_RELATION_L);
	}

	/* Check for frequency increase */
	else if (load >= max(sa_tuners->up_threshold, prev_load)) {
		const unsigned int freq_max = policy->max;

		// stop if the frequency is at the maxmimum value
		if (freq_cur == freq_max)
			return;

		freq_target = (freq_max + freq_cur) >> RESISTANCE_OFFSET;
		if (input_event)
			freq_target = max(freq_target,
				          sa_tuners->input_event_min_freq);

		__cpufreq_driver_target(policy, freq_target,
					 CPUFREQ_RELATION_H);
	}

}

static void sa_dbs_timer(struct work_struct *work)
{
	struct sa_cpu_dbs_info_s *dbs_info = container_of(work,
			struct sa_cpu_dbs_info_s, cdbs.work.work);
	unsigned int cpu = dbs_info->cdbs.cur_policy->cpu;
	struct sa_cpu_dbs_info_s *core_dbs_info = &per_cpu(sa_cpu_dbs_info,
			cpu);
	struct dbs_data *dbs_data = dbs_info->cdbs.cur_policy->governor_data;
	struct sa_dbs_tuners* const sa_tuners = dbs_data->tuners;
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

static int dbs_cpufreq_notifier(struct notifier_block *nb, unsigned long val,
		void *data)
{
	struct cpufreq_freqs *freq = data;
	struct sa_cpu_dbs_info_s *dbs_info =
					&per_cpu(sa_cpu_dbs_info, freq->cpu);
	struct cpufreq_policy *policy;

	if (dbs_info->enable)
		policy = dbs_info->cdbs.cur_policy;

	return 0;
}

/************************** sysfs interface ************************/
static struct common_dbs_data sa_dbs_cdata;

static ssize_t store_sampling_rate(struct dbs_data *dbs_data, const char *buf,
		size_t count)
{
	struct sa_dbs_tuners* const sa_tuners = dbs_data->tuners;
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1)
		return -EINVAL;

	sa_tuners->sampling_rate = max(input, dbs_data->min_sampling_rate);
	return count;
}

static ssize_t store_up_threshold(struct dbs_data *dbs_data, const char *buf,
		size_t count)
{
	struct sa_dbs_tuners* const sa_tuners = dbs_data->tuners;
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > MAXIMUM_LOAD ||
                input <= sa_tuners->down_threshold)
		return -EINVAL;

	sa_tuners->up_threshold = input;
	return count;
}

static ssize_t store_down_threshold(struct dbs_data *dbs_data, const char *buf,
		size_t count)
{
	struct sa_dbs_tuners* const sa_tuners = dbs_data->tuners;
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input < MINIMUM_LOAD ||
                input >= sa_tuners->up_threshold)
		return -EINVAL;

	sa_tuners->down_threshold = input;
	return count;
}

static ssize_t store_input_event_min_freq(struct dbs_data *dbs_data,
					  const char *buf, size_t count)
{
	struct sa_dbs_tuners* const sa_tuners = dbs_data->tuners;

	unsigned int input;
	unsigned int cpu;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1)
		return -EINVAL;

        /* The input should be at most the lowest maximum frequency set among
         * all CPUs and at least the greatest minimum frequency of all CPUs
         */
	for_each_online_cpu(cpu) {
		struct sa_cpu_dbs_info_s* const dbs_info = &per_cpu(sa_cpu_dbs_info, cpu);
		const struct cpufreq_policy* const policy = dbs_info->cdbs.cur_policy;

		if (input < policy->min)
			input = policy->min;

		else if (input > policy->max)
			input  = policy->max;
        }

        sa_tuners->input_event_min_freq = input;
	return count;
}

static ssize_t store_input_event_duration(struct dbs_data *dbs_data,
					  const char *buf, size_t count)
{
	struct sa_dbs_tuners* const sa_tuners = dbs_data->tuners;

	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > MAX_INPUT_EVENT_DURATION)
		return -EINVAL;

	sa_tuners->input_event_duration = input;
	return count;
}

show_store_one(sa, sampling_rate);
show_store_one(sa, up_threshold);
show_store_one(sa, down_threshold);
show_store_one(sa, input_event_min_freq);
show_store_one(sa, input_event_duration);
declare_show_sampling_rate_min(sa);

gov_sys_pol_attr_rw(sampling_rate);
gov_sys_pol_attr_rw(up_threshold);
gov_sys_pol_attr_rw(down_threshold);
gov_sys_pol_attr_rw(input_event_min_freq);
gov_sys_pol_attr_rw(input_event_duration);
gov_sys_pol_attr_ro(sampling_rate_min);

static struct attribute *dbs_attributes_gov_sys[] = {
	&sampling_rate_gov_sys.attr,
	&sampling_rate_min_gov_sys.attr,
	&up_threshold_gov_sys.attr,
	&down_threshold_gov_sys.attr,
	&input_event_min_freq_gov_sys.attr,
	&input_event_duration_gov_sys.attr,
	NULL
};

static struct attribute_group sa_attr_group_gov_sys = {
	.attrs = dbs_attributes_gov_sys,
	.name = "sublime_active",
};

static struct attribute *dbs_attributes_gov_pol[] = {
	&sampling_rate_min_gov_pol.attr,
	&sampling_rate_gov_pol.attr,
	&up_threshold_gov_pol.attr,
	&down_threshold_gov_pol.attr,
	&input_event_min_freq_gov_pol.attr,
	&input_event_duration_gov_pol.attr,
	NULL
};

static struct attribute_group sa_attr_group_gov_pol = {
	.attrs = dbs_attributes_gov_pol,
	.name = "sublime_active",
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
	tuners->up_threshold = DEF_FREQUENCY_UP_THRESHOLD;
	tuners->down_threshold = DEF_FREQUENCY_DOWN_THRESHOLD;
	tuners->input_event_min_freq = DEF_INPUT_EVENT_MIN_FREQUENCY;
	tuners->input_event_duration = DEF_INPUT_EVENT_DURATION;

	dbs_data->tuners = tuners;
	dbs_data->min_sampling_rate = MINIMUM_SAMPLING_RATE;
	mutex_init(&dbs_data->mutex);
	return 0;
}

static void sa_exit(struct dbs_data *dbs_data)
{
	kfree(dbs_data->tuners);
}

define_get_cpu_dbs_routines(sa_cpu_dbs_info);

static struct notifier_block sa_cpufreq_notifier_block = {
	.notifier_call = dbs_cpufreq_notifier,
};

static struct sa_ops sa_ops = {
	.notifier_block = &sa_cpufreq_notifier_block,
};

static struct common_dbs_data sa_dbs_cdata = {
	.governor = GOV_SUBLIMEACTIVE,
	.attr_group_gov_sys = &sa_attr_group_gov_sys,
	.attr_group_gov_pol = &sa_attr_group_gov_pol,
	.get_cpu_cdbs = get_cpu_cdbs,
	.get_cpu_dbs_info_s = get_cpu_dbs_info_s,
	.gov_dbs_timer = sa_dbs_timer,
	.gov_check_cpu = sa_check_cpu,
	.gov_ops = &sa_ops,
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
	.name			= "sublime_active",
	.governor		= sa_cpufreq_governor_dbs,
	.max_transition_latency	= TRANSITION_LATENCY_LIMIT,
	.owner			= THIS_MODULE,
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