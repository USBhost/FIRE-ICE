/*
 * arch/arm/mach-tegra/edp.c
 *
 * Copyright (c) 2011-2014, NVIDIA CORPORATION. All Rights Reserved.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/edp.h>
#include <linux/sysedp.h>
#include <linux/tegra-soc.h>
#include <linux/regulator/consumer.h>
#include <linux/tegra-fuse.h>

#include <mach/edp.h>

#include <linux/platform/tegra/dvfs.h>
#include <linux/platform/tegra/clock.h>
#include <linux/platform/tegra/cpu-tegra.h>
#include <linux/platform/tegra/common.h>

#ifndef CONFIG_CPU_FREQ
#error "CONFIG_CPU_FREQ is required for EDP"
#endif

#define FREQ_STEP 12750000
#define OVERRIDE_DEFAULT 6000

#define IS_T12X		(tegra_chip_id == TEGRA_CHIPID_TEGRA12)
#define IS_T13X		(tegra_chip_id == TEGRA_CHIPID_TEGRA13)
#define IS_T1XX		(IS_T12X || IS_T13X)

static u32 tegra_chip_id = 0xdeadbeef;

static struct tegra_edp_limits *cpu_edp_limits;
static int cpu_edp_limits_size;
static unsigned int cpu_edp_regulator_cur;
/* Value to subtract from regulator current limit */
static unsigned int cpu_edp_reg_override_ma = OVERRIDE_DEFAULT;

static struct tegra_edp_limits *reg_idle_edp_limits;
static int reg_idle_cur;

static struct tegra_system_edp_entry *power_edp_limits;
static int power_edp_limits_size;

/*
 * "Safe entry" to be used when no match for speedo_id /
 * cpu_edp_regulator_cur is found; must be the last one
 */
static struct tegra_edp_limits cpu_edp_default_limits[] = {
	{85, {1000000, 1000000, 1000000, 1000000} },
};

static struct tegra_system_edp_entry power_edp_default_limits[] = {
	{0, 20, {1000000, 1000000, 1000000, 1000000} },
};

/* Constants for EDP calculations */
static const int temperatures[] = { /* degree celcius (C) */
	23, 40, 50, 60, 70, 74, 78, 82, 86, 90, 94, 98, 102,
};

static const int power_cap_levels[] = { /* milliwatts (mW) */
	  500,  1000,  1500,  2000,  2500,  3000,  3500,
	 4000,  4500,  5000,  5500,  6000,  6500,  7000,
	 7500,  8000,  8500,  9000,  9500, 10000, 10500,
	11000, 11500, 12000, 12500, 13000, 13500, 14000,
	14500, 15000, 15500, 16000, 16500, 17000
};

static struct tegra_edp_freq_voltage_table *freq_voltage_lut_saved;
static unsigned int freq_voltage_lut_size_saved;
static struct tegra_edp_freq_voltage_table *freq_voltage_lut;
static unsigned int freq_voltage_lut_size;

static inline s64 edp_pow(s64 val, int pwr)
{
	s64 retval = 1;

	while (val && pwr) {
		if (pwr & 1)
			retval *= val;
		pwr >>= 1;
		if (pwr)
			val *= val;
	}

	return retval;
}


#ifdef CONFIG_TEGRA_CPU_EDP_FIXED_LIMITS
static inline unsigned int cpu_edp_apply_fixed_limits(
				unsigned int in_freq_khz,
				struct tegra_edp_cpu_powermodel_params *params,
				unsigned int cur_effective,
				int temp_c, int n_cores_idx)
{
	unsigned int out_freq_khz = in_freq_khz;
	unsigned int max_cur, max_temp, max_freq;
	int i;

	/* Apply any additional fixed limits */
	for (i = 0; i < 8; i++) {
		max_cur = params->common.max_current_cap[i].max_cur;
		if (max_cur != 0 && cur_effective <= max_cur) {
			max_temp = params->common.max_current_cap[i].max_temp;
			if (max_temp != 0 && temp_c > max_temp) {
				max_freq = params->common.max_current_cap[i].
					max_freq[n_cores_idx];
				if (max_freq && max_freq < out_freq_khz)
					out_freq_khz = max_freq;
			}
		}
	}

	return out_freq_khz;
}
#else
#define cpu_edp_apply_fixed_limits(freq, unused...)	(freq)
#endif

/* Calculate incremental leakage current: common for cpu and gpu */
static s64 common_edp_calculate_leakage_calc_step(
			struct tegra_edp_common_powermodel_params *common,
			int iddq_ma, int temp_c,
			unsigned int voltage_mv, int i, int j, int k)
{
	s64 leakage_calc_step;

	/* iddq raised to i */
	leakage_calc_step = common->leakage_consts_ijk[i][j][k] *
						edp_pow(iddq_ma, i);

	/* Convert (mA)^i to (A)^i */
	leakage_calc_step = div64_s64(leakage_calc_step, edp_pow(1000, i));

	/* voltage raised to j */
	leakage_calc_step *= edp_pow(voltage_mv, j);

	/* Convert (mV)^j to (V)^j */
	leakage_calc_step = div64_s64(leakage_calc_step, edp_pow(1000, j));

	/* temp raised to k */
	leakage_calc_step *= edp_pow(temp_c, k);

	/* Convert (C)^k to (scaled_C)^k */
	leakage_calc_step = div64_s64(leakage_calc_step,
					edp_pow(common->temp_scaled, k));

	/* leakage_consts_ijk was scaled */
	leakage_calc_step = div64_s64(leakage_calc_step,
					common->ijk_scaled);

	return leakage_calc_step;
}

/* Calculate total leakage current: common for cpu and gpu */
static s64 common_edp_calculate_leakage_ma(
			struct tegra_edp_common_powermodel_params *common,
			int iddq_ma, int temp_c,
			unsigned int voltage_mv, int n_cores_idx)
{
	int i, j, k;
	s64 leakage_ma = 0;

	for (i = 0; i <= 3; i++)
		for (j = 0; j <= 3; j++)
			for (k = 0; k <= 3; k++)
				leakage_ma +=
					common_edp_calculate_leakage_calc_step(
						common, iddq_ma,
						temp_c, voltage_mv, i, j, k);

	leakage_ma *= common->leakage_consts_n[n_cores_idx];

	/* leakage_const_n was scaled */
	leakage_ma = div64_s64(leakage_ma, common->consts_scaled);

	/* set floor for leakage current */
	if (leakage_ma <= common->leakage_min)
		leakage_ma = common->leakage_min;

	return leakage_ma;
}

/* Calculate dynamic current: common for cpu and gpu */
static s64 common_edp_calculate_dynamic_ma(
			struct tegra_edp_common_powermodel_params *common,
			unsigned int voltage_mv, int n_cores_idx,
			unsigned int freq_khz)
{
	s64 dyn_ma;

	/* Convert freq to MHz */
	dyn_ma = voltage_mv * freq_khz / 1000;

	/* Convert mV to V */
	dyn_ma = div64_s64(dyn_ma, 1000);
	dyn_ma *= common->dyn_consts_n[n_cores_idx];

	/* dyn_const_n was scaled */
	dyn_ma = div64_s64(dyn_ma, common->dyn_scaled);

	return dyn_ma;
}

/*
 * Find the maximum frequency that results in dynamic and leakage current that
 * is less than the regulator current limit.
 * temp_c - valid or -EINVAL
 * power_mw - valid or -1 (infinite) or -EINVAL
 */
static unsigned int cpu_edp_calculate_maxf(
				struct tegra_edp_cpu_powermodel_params *params,
				int temp_c, int power_mw, unsigned int cur_ma,
				int cpu_iddq_ma,
				int n_cores_idx)
{
	unsigned int voltage_mv, freq_khz = 0;
	unsigned int cur_effective = cpu_edp_regulator_cur -
				     cpu_edp_reg_override_ma;
	int f;
	s64 leakage_ma, dyn_ma;
	s64 leakage_ma1;
	s64 leakage_mw, dyn_mw;

	leakage_ma1 = leakage_ma = dyn_ma = 0;
	voltage_mv = 0;
	/* If current limit is not specified, use max by default */
	cur_ma = cur_ma ? : cur_effective;

	for (f = freq_voltage_lut_size - 1; f >= 0; f--) {
		freq_khz = freq_voltage_lut[f].freq / 1000;
		voltage_mv = freq_voltage_lut[f].voltage_mv;

		/* Constrain Volt-Temp */
		if (params->volt_temp_cap.temperature &&
		    temp_c > params->volt_temp_cap.temperature &&
		    params->volt_temp_cap.voltage_limit_mv &&
		    voltage_mv > params->volt_temp_cap.voltage_limit_mv)
			continue;

		/* Calculate leakage current */
		leakage_ma = common_edp_calculate_leakage_ma(
					&params->common, cpu_iddq_ma,
					temp_c, voltage_mv, n_cores_idx);

		/* Calculate dynamic current */
		dyn_ma = common_edp_calculate_dynamic_ma(&params->common,
					voltage_mv, n_cores_idx, freq_khz);

		if (power_mw != -1) {
			leakage_mw = leakage_ma * voltage_mv;
			dyn_mw = dyn_ma * voltage_mv;
			if (div64_s64(leakage_mw + dyn_mw, 1000) <= power_mw)
				goto end;
		} else if ((leakage_ma + dyn_ma) <= cur_ma) {
			goto end;
		}
		freq_khz = 0;
	}

 end:
	return cpu_edp_apply_fixed_limits(freq_khz, params,
					cur_effective, temp_c, n_cores_idx);
}

static int edp_relate_freq_voltage(struct clk *clk_cpu_g,
			unsigned int cpu_speedo_idx,
			unsigned int freq_volt_lut_size,
			struct tegra_edp_freq_voltage_table *freq_volt_lut)
{
	unsigned int i, j, freq;
	int voltage_mv;

	for (i = 0, j = 0, freq = 0;
		 i < freq_volt_lut_size;
		 i++, freq += FREQ_STEP) {

		/* Predict voltages */
		voltage_mv = tegra_dvfs_predict_peak_millivolts(
			clk_cpu_g, freq);
		if (voltage_mv < 0) {
			pr_err("%s: couldn't predict voltage: freq %u; err %d",
			       __func__, freq, voltage_mv);
			return -EINVAL;
		}

		/* Cache frequency / voltage / voltage constant relationship */
		freq_volt_lut[i].freq = freq;
		freq_volt_lut[i].voltage_mv = voltage_mv;
	}
	return 0;
}

/*
 * Finds the maximum frequency whose corresponding voltage is <= volt
 * If no such frequency is found, the least possible frequency is returned
 */
unsigned int tegra_edp_find_maxf(int volt)
{
	unsigned int i;

	for (i = 0; i < freq_voltage_lut_size_saved; i++) {
		if (freq_voltage_lut_saved[i].voltage_mv > volt) {
			if (!i)
				return freq_voltage_lut_saved[i].freq;
			break;
		}
	}
	return freq_voltage_lut_saved[i - 1].freq;
}


static int edp_find_speedo_idx(int cpu_speedo_id, unsigned int *cpu_speedo_idx)
{
	int i, array_size;
	struct tegra_edp_cpu_powermodel_params *params;

	if (IS_T12X)
		params = tegra12x_get_cpu_powermodel_params(0, &array_size);
	else if (IS_T13X) {
		/* pick edp_params array-index based on package */
		*cpu_speedo_idx = (tegra_package_id() == 1);
		return 0;
	} else
		array_size = 0;

	for (i = 0; i < array_size; i++)
		if (cpu_speedo_id == params[i].cpu_speedo_id) {
			*cpu_speedo_idx = i;
			return 0;
		}

	pr_err("%s: couldn't find cpu speedo id %d in freq/voltage LUT\n",
	       __func__, cpu_speedo_id);
	return -EINVAL;
}

static int init_cpu_edp_limits_calculated(void)
{
	unsigned int max_nr_cpus = num_possible_cpus();
	unsigned int temp_idx, n_cores_idx, pwr_idx;
	unsigned int cpu_g_minf, cpu_g_maxf;
	unsigned int cpu_iddq_ma;
	unsigned int cpu_speedo_idx;
	unsigned int cap, limit;
	struct tegra_edp_limits *cpu_edp_calculated_limits;
	struct tegra_edp_limits *reg_idle_calc_limits;
	struct tegra_system_edp_entry *power_edp_calc_limits;
	struct tegra_edp_cpu_powermodel_params *params;
	int ret;
	struct clk *clk_cpu_g = tegra_get_clock_by_name("cpu_g");
	int cpu_speedo_id = tegra_cpu_speedo_id();
	int idle_cur = reg_idle_cur;

	/* Determine all inputs to EDP formula */
	cpu_iddq_ma = tegra_get_cpu_iddq_value();
	pr_debug("%s: CPU IDDQ value %d\n", __func__, cpu_iddq_ma);
	ret = edp_find_speedo_idx(cpu_speedo_id, &cpu_speedo_idx);
	if (ret)
		return ret;

	params =
		IS_T12X ?
		tegra12x_get_cpu_powermodel_params(cpu_speedo_idx, NULL) :
		IS_T13X ?
		tegra13x_get_cpu_powermodel_params(cpu_speedo_idx, NULL) :
		NULL;
	if (!params)
		return -EINVAL;

	cpu_edp_calculated_limits = kmalloc(sizeof(struct tegra_edp_limits)
					* ARRAY_SIZE(temperatures), GFP_KERNEL);
	BUG_ON(!cpu_edp_calculated_limits);

	reg_idle_calc_limits = kmalloc(sizeof(struct tegra_edp_limits)
				       * ARRAY_SIZE(temperatures), GFP_KERNEL);
	BUG_ON(!reg_idle_calc_limits);

	power_edp_calc_limits = kmalloc(sizeof(struct tegra_system_edp_entry)
				* ARRAY_SIZE(power_cap_levels), GFP_KERNEL);
	BUG_ON(!power_edp_calc_limits);

	cpu_g_minf = 0;
	cpu_g_maxf = clk_get_max_rate(clk_cpu_g);
	freq_voltage_lut_size = (cpu_g_maxf - cpu_g_minf) / FREQ_STEP + 1;
	freq_voltage_lut = kmalloc(sizeof(struct tegra_edp_freq_voltage_table)
				   * freq_voltage_lut_size, GFP_KERNEL);
	if (!freq_voltage_lut) {
		pr_err("%s: failed alloc mem for freq/voltage LUT\n", __func__);
		ret = -ENOMEM;
		goto err;
	}

	ret = edp_relate_freq_voltage(clk_cpu_g, cpu_speedo_idx,
				freq_voltage_lut_size, freq_voltage_lut);
	if (ret)
		goto err;

	if (freq_voltage_lut_size != freq_voltage_lut_size_saved) {
		/* release previous table if present */
		kfree(freq_voltage_lut_saved);
		/* create table to save */
		freq_voltage_lut_saved =
			kmalloc(sizeof(struct tegra_edp_freq_voltage_table) *
			freq_voltage_lut_size, GFP_KERNEL);
		if (!freq_voltage_lut_saved) {
			pr_err("%s: failed alloc mem for freq/voltage LUT\n",
				__func__);
			ret = -ENOMEM;
			goto err;
		}
		freq_voltage_lut_size_saved = freq_voltage_lut_size;
	}
	memcpy(freq_voltage_lut_saved,
		freq_voltage_lut,
		sizeof(struct tegra_edp_freq_voltage_table) *
			freq_voltage_lut_size);

	/* Calculate EDP table */
	for (n_cores_idx = 0; n_cores_idx < max_nr_cpus; n_cores_idx++) {
		for (temp_idx = 0;
		     temp_idx < ARRAY_SIZE(temperatures); temp_idx++) {
			cpu_edp_calculated_limits[temp_idx].temperature =
				temperatures[temp_idx];
			limit = cpu_edp_calculate_maxf(params,
						   temperatures[temp_idx],
						   -1,
						   0,
						   cpu_iddq_ma,
						   n_cores_idx);
			if (limit == -EINVAL) {
				ret = -EINVAL;
				goto err;
			}
			/* apply safety cap if it is specified */
			if (n_cores_idx < 4) {
				cap = params->safety_cap[n_cores_idx];
				if (cap && cap < limit)
					limit = cap;
			}
			cpu_edp_calculated_limits[temp_idx].
				freq_limits[n_cores_idx] = limit;

			/* regulator mode threshold */
			if (!idle_cur)
				continue;
			reg_idle_calc_limits[temp_idx].temperature =
				temperatures[temp_idx];
			limit = cpu_edp_calculate_maxf(params,
						   temperatures[temp_idx],
						   -1,
						   idle_cur,
						   cpu_iddq_ma,
						   n_cores_idx);

			/* remove idle table if any threshold is invalid */
			if (limit == -EINVAL) {
				pr_warn("%s: Invalid idle limit for %dmA\n",
					__func__, idle_cur);
				idle_cur = 0;
				continue;
			}

			/* No mode change below G CPU minimum rate */
			if (limit < clk_get_min_rate(clk_cpu_g) / 1000)
				limit = 0;
			reg_idle_calc_limits[temp_idx].
				freq_limits[n_cores_idx] = limit;
		}

		for (pwr_idx = 0;
		     pwr_idx < ARRAY_SIZE(power_cap_levels); pwr_idx++) {
			power_edp_calc_limits[pwr_idx].power_limit_100mW =
				power_cap_levels[pwr_idx] / 100;
			limit = cpu_edp_calculate_maxf(params,
						   50,
						   -1,
						   power_cap_levels[pwr_idx],
						   cpu_iddq_ma,
						   n_cores_idx);
			if (limit == -EINVAL)
				return -EINVAL;
			power_edp_calc_limits[pwr_idx].
				freq_limits[n_cores_idx] = limit;
		}
	}

	/*
	 * If this is an EDP table update, need to overwrite old table.
	 * The old table's address must remain valid.
	 */
	if (cpu_edp_limits != cpu_edp_default_limits) {
		memcpy(cpu_edp_limits, cpu_edp_calculated_limits,
		       sizeof(struct tegra_edp_limits)
		       * ARRAY_SIZE(temperatures));
		kfree(cpu_edp_calculated_limits);
	} else {
		cpu_edp_limits = cpu_edp_calculated_limits;
		cpu_edp_limits_size = ARRAY_SIZE(temperatures);
	}

	if (idle_cur && reg_idle_edp_limits) {
		memcpy(reg_idle_edp_limits, reg_idle_calc_limits,
		       sizeof(struct tegra_edp_limits)
		       * ARRAY_SIZE(temperatures));
		kfree(reg_idle_calc_limits);
	} else if (idle_cur) {
		reg_idle_edp_limits = reg_idle_calc_limits;
	} else {
		kfree(reg_idle_edp_limits);
		kfree(reg_idle_calc_limits);
		reg_idle_edp_limits = NULL;
	}

	if (power_edp_limits != power_edp_default_limits) {
		memcpy(power_edp_limits, power_edp_calc_limits,
		       sizeof(struct tegra_system_edp_entry)
		       * ARRAY_SIZE(power_cap_levels));
		kfree(power_edp_calc_limits);
	} else {
		power_edp_limits = power_edp_calc_limits;
		power_edp_limits_size = ARRAY_SIZE(power_cap_levels);
	}

	kfree(freq_voltage_lut);
	return 0;

 err:
	kfree(freq_voltage_lut);
	freq_voltage_lut = NULL;
	kfree(cpu_edp_calculated_limits);
	cpu_edp_calculated_limits = NULL;
	kfree(reg_idle_calc_limits);
	reg_idle_calc_limits = NULL;
	kfree(power_edp_calc_limits);
	power_edp_calc_limits = NULL;
	kfree(freq_voltage_lut_saved);
	freq_voltage_lut_saved = NULL;

	return ret;
}

void tegra_recalculate_cpu_edp_limits(void)
{
	if (!IS_T1XX)
		return;
	if (init_cpu_edp_limits_calculated() == 0)
		return;

	/* Revert to default EDP table on error */
	cpu_edp_limits = cpu_edp_default_limits;
	cpu_edp_limits_size = ARRAY_SIZE(cpu_edp_default_limits);

	power_edp_limits = power_edp_default_limits;
	power_edp_limits_size = ARRAY_SIZE(power_edp_default_limits);

	kfree(reg_idle_edp_limits);
	reg_idle_edp_limits = NULL;
	pr_err("%s: Failed to recalculate EDP limits\n", __func__);
}

/*
 * Specify regulator current in mA, e.g. 5000mA
 * Use 0 for default table
 */
void __init tegra_init_cpu_edp_limits(unsigned int regulator_ma)
{
	if (tegra_chip_id == 0xdeadbeef)
		tegra_chip_id = tegra_get_chip_id();

	if (!IS_T1XX)
		return;

	if (!regulator_ma) {
		cpu_edp_limits = cpu_edp_default_limits;
		cpu_edp_limits_size = ARRAY_SIZE(cpu_edp_default_limits);

		power_edp_limits = power_edp_default_limits;
		power_edp_limits_size = ARRAY_SIZE(power_edp_default_limits);
		return;
	}

	cpu_edp_regulator_cur = regulator_ma + OVERRIDE_DEFAULT;
	init_cpu_edp_limits_calculated();
}

void tegra_get_cpu_edp_limits(const struct tegra_edp_limits **limits, int *size)
{
	*limits = cpu_edp_limits;
	*size = cpu_edp_limits_size;
}

void __init tegra_init_cpu_reg_mode_limits(unsigned int regulator_ma,
					   unsigned int mode)
{
	if (mode == REGULATOR_MODE_IDLE) {
		reg_idle_cur = regulator_ma;
		return;
	}
	pr_err("%s: Not supported regulator mode 0x%x\n", __func__, mode);
}

void tegra_get_cpu_reg_mode_limits(const struct tegra_edp_limits **limits,
				   int *size, unsigned int mode)
{
	if (mode == REGULATOR_MODE_IDLE) {
		*limits = reg_idle_edp_limits;
		*size = cpu_edp_limits_size;
	} else {
		*limits = NULL;
		*size = 0;
	}
}

void tegra_platform_edp_init(struct thermal_trip_info *trips,
				int *num_trips, int margin)
{
	const struct tegra_edp_limits *cpu_edp_limits;
	struct thermal_trip_info *trip_state;
	int i, cpu_edp_limits_size;

	if (!trips || !num_trips)
		return;

	/* edp capping */
	tegra_get_cpu_edp_limits(&cpu_edp_limits, &cpu_edp_limits_size);

	if (cpu_edp_limits_size > MAX_THROT_TABLE_SIZE)
		BUG();

	for (i = 0; i < cpu_edp_limits_size-1; i++) {
		trip_state = &trips[*num_trips];

		trip_state->cdev_type = "cpu_edp";
		trip_state->trip_temp =
			(cpu_edp_limits[i].temperature * 1000) - margin;
		trip_state->trip_type = THERMAL_TRIP_ACTIVE;
		trip_state->upper = trip_state->lower = i + 1;

		(*num_trips)++;

		if (*num_trips >= THERMAL_MAX_TRIPS)
			BUG();
	}
}

struct tegra_system_edp_entry *tegra_get_system_edp_entries(int *size)
{
	*size = power_edp_limits_size;
	return power_edp_limits;
}

#ifdef CONFIG_DEBUG_FS

static int cpu_edp_limit_debugfs_show(struct seq_file *s, void *data)
{
	seq_printf(s, "%u\n", tegra_get_edp_limit(NULL));
	return 0;
}

static inline void edp_show_4core_cpu_edp_table(struct seq_file *s, int th_idx)
{
	int i;

	seq_printf(s, "%6s %10s %10s %10s %10s\n",
		   " Temp.", "1-core", "2-cores", "3-cores", "4-cores");
	for (i = 0; i < cpu_edp_limits_size; i++) {
		seq_printf(s, "%c%3dC: %10u %10u %10u %10u\n",
			   i == th_idx ? '>' : ' ',
			   cpu_edp_limits[i].temperature,
			   cpu_edp_limits[i].freq_limits[0],
			   cpu_edp_limits[i].freq_limits[1],
			   cpu_edp_limits[i].freq_limits[2],
			   cpu_edp_limits[i].freq_limits[3]);
	}
}

static inline void edp_show_2core_cpu_edp_table(struct seq_file *s, int th_idx)
{
	int i;

	seq_printf(s, "%6s %10s %10s\n",
		   " Temp.", "1-core", "2-cores");
	for (i = 0; i < cpu_edp_limits_size; i++) {
		seq_printf(s, "%c%3dC: %10u %10u\n",
			   i == th_idx ? '>' : ' ',
			   cpu_edp_limits[i].temperature,
			   cpu_edp_limits[i].freq_limits[0],
			   cpu_edp_limits[i].freq_limits[1]);
	}
}

static inline void edp_show_4core_reg_mode_table(struct seq_file *s, int th_idx)
{
	int i;

	seq_printf(s, "%6s %10s %10s %10s %10s\n",
		   " Temp.", "1-core", "2-cores", "3-cores", "4-cores");
	for (i = 0; i < cpu_edp_limits_size; i++) {
		seq_printf(s, "%c%3dC: %10u %10u %10u %10u\n",
			   i == th_idx ? '>' : ' ',
			   reg_idle_edp_limits[i].temperature,
			   reg_idle_edp_limits[i].freq_limits[0],
			   reg_idle_edp_limits[i].freq_limits[1],
			   reg_idle_edp_limits[i].freq_limits[2],
			   reg_idle_edp_limits[i].freq_limits[3]);
	}
}

static inline void edp_show_2core_reg_mode_table(struct seq_file *s, int th_idx)
{
	int i;

	seq_printf(s, "%6s %10s %10s\n",
		   " Temp.", "1-core", "2-cores");
	for (i = 0; i < cpu_edp_limits_size; i++) {
		seq_printf(s, "%c%3dC: %10u %10u\n",
			   i == th_idx ? '>' : ' ',
			   reg_idle_edp_limits[i].temperature,
			   reg_idle_edp_limits[i].freq_limits[0],
			   reg_idle_edp_limits[i].freq_limits[1]);
	}
}

static int cpu_edp_debugfs_show(struct seq_file *s, void *data)
{
	unsigned int max_nr_cpus = num_possible_cpus();
	int th_idx;

	if (max_nr_cpus != 2 && max_nr_cpus != 4) {
		seq_printf(s, "Unsupported number of CPUs\n");
		return 0;
	}

	tegra_get_edp_limit(&th_idx);

	seq_printf(s, "-- VDD_CPU %sEDP table (%umA = %umA - %umA) --\n",
		   cpu_edp_limits == cpu_edp_default_limits ? "**default** " : "",
		   cpu_edp_regulator_cur - cpu_edp_reg_override_ma,
		   cpu_edp_regulator_cur, cpu_edp_reg_override_ma);

	if (max_nr_cpus == 2)
		edp_show_2core_cpu_edp_table(s, th_idx);
	else if (max_nr_cpus == 4)
		edp_show_4core_cpu_edp_table(s, th_idx);

	if (reg_idle_edp_limits) {
		seq_printf(s, "\n-- Regulator mode thresholds @ %dmA --\n",
			   reg_idle_cur);
		if (max_nr_cpus == 2)
			edp_show_2core_reg_mode_table(s, th_idx);
		else if (max_nr_cpus == 4)
			edp_show_4core_reg_mode_table(s, th_idx);
	}

	return 0;
}

static int cpu_edp_reg_override_show(struct seq_file *s, void *data)
{
	seq_printf(s, "Limit override: %u mA. Effective limit: %u mA\n",
		   cpu_edp_reg_override_ma,
		   cpu_edp_regulator_cur - cpu_edp_reg_override_ma);
	return 0;
}

static ssize_t cpu_edp_reg_override_write(struct file *file,
	const char __user *userbuf, size_t count, loff_t *ppos)
{
	char buf[32];
	unsigned long cpu_edp_reg_override_ma_temp;
	unsigned int cpu_edp_reg_override_ma_prev = cpu_edp_reg_override_ma;

	if (!IS_T1XX)
		return -EINVAL;

	if (sizeof(buf) <= count)
		goto override_err;

	if (copy_from_user(buf, userbuf, count))
		goto override_err;

	/* terminate buffer and trim - white spaces may be appended
	 *  at the end when invoked from shell command line */
	buf[count]='\0';
	strim(buf);

	if (kstrtoul(buf, 10, &cpu_edp_reg_override_ma_temp))
		goto override_err;

	if (cpu_edp_reg_override_ma_temp >= cpu_edp_regulator_cur)
		goto override_err;

	if (cpu_edp_reg_override_ma == cpu_edp_reg_override_ma_temp)
		return count;

	cpu_edp_reg_override_ma = cpu_edp_reg_override_ma_temp;
	if (init_cpu_edp_limits_calculated()) {
		/* Revert to previous override value if new value fails */
		cpu_edp_reg_override_ma = cpu_edp_reg_override_ma_prev;
		goto override_err;
	}

	if (tegra_cpu_set_speed_cap(NULL)) {
		pr_err("FAILED: Set CPU freq cap with new VDD_CPU EDP table\n");
		return -EINVAL;
	}

	pr_info("Reinitialized VDD_CPU EDP table with regulator current limit %u mA\n",
		cpu_edp_regulator_cur - cpu_edp_reg_override_ma);
	return count;

 override_err:
	pr_err("FAILED: Reinitialize VDD_CPU EDP table with override \"%s\"",
	       buf);
	return -EINVAL;
}

static int cpu_edp_debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, cpu_edp_debugfs_show, inode->i_private);
}

static int cpu_edp_limit_debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, cpu_edp_limit_debugfs_show, inode->i_private);
}

static int cpu_edp_reg_override_open(struct inode *inode, struct file *file)
{
	return single_open(file, cpu_edp_reg_override_show, inode->i_private);
}

static const struct file_operations cpu_edp_debugfs_fops = {
	.open		= cpu_edp_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static const struct file_operations cpu_edp_limit_debugfs_fops = {
	.open		= cpu_edp_limit_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static const struct file_operations cpu_edp_reg_override_debugfs_fops = {
	.open		= cpu_edp_reg_override_open,
	.read		= seq_read,
	.write		= cpu_edp_reg_override_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int reg_idle_cur_get(void *data, u64 *val)
{
	*val = reg_idle_cur;
	return 0;
}
static int reg_idle_cur_set(void *data, u64 val)
{
	int ret;

	ret = tegra_cpu_reg_mode_force_normal(true);
	if (ret) {
		pr_err("%s: Failed to force regulator normal mode\n", __func__);
		return ret;
	}

	reg_idle_cur = (int)val;
	tegra_update_cpu_edp_limits();
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(reg_idle_cur_debugfs_fops,
			reg_idle_cur_get, reg_idle_cur_set, "%llu\n");

static int cpucaps_show(struct seq_file *file, void *data)
{
	struct tegra_system_edp_entry *p = power_edp_limits;
	int max_nr_cpus = (int) num_possible_cpus();
	int i, j;
	char ncores[10] = "1-core";

	if (!power_edp_limits)
		return -ENODEV;

	seq_printf(file, "Power");
	for (j = 0; j < max_nr_cpus; j++) {
		if (j != 0)
			snprintf(ncores, 10, "%d-cores", (j+1));
		seq_printf(file, " %10s", ncores );
	}
	seq_printf(file, "\n");

	for (i = 0; i < power_edp_limits_size; i++, p++) {
		seq_printf(file, "%5d", p->power_limit_100mW*100);
		for (j = 0; j < max_nr_cpus; j++) {
			seq_printf(file, " %10u", p->freq_limits[j]);
		}
		seq_printf(file, "\n");
	}

	return 0;
}

static int longattr_open(struct inode *inode, struct file *file)
{
	return single_open(file, inode->i_private, NULL);
}

static const struct file_operations longattr_fops = {
	.open = longattr_open,
	.read = seq_read,
};

#if defined(CONFIG_EDP_FRAMEWORK) || defined(CONFIG_SYSEDP_FRAMEWORK)
static __init struct dentry *tegra_edp_debugfs_dir(void)
{
	if (edp_debugfs_dir)
		return edp_debugfs_dir;
	else
		return debugfs_create_dir("edp", NULL);
}
#else
static __init struct dentry *tegra_edp_debugfs_dir(void)
{
	return debugfs_create_dir("edp", NULL);
}
#endif

static int __init tegra_edp_debugfs_init(void)
{
	struct dentry *d_reg_idle_cur;
	struct dentry *d_edp;
	struct dentry *d_edp_limit;
	struct dentry *d_edp_reg_override;
	struct dentry *edp_dir;
	struct dentry *vdd_cpu_dir;

	if (!tegra_platform_is_silicon())
		return -ENOSYS;

	edp_dir = tegra_edp_debugfs_dir();
	if (!edp_dir)
		goto err_0;

	vdd_cpu_dir = debugfs_create_dir("vdd_cpu", edp_dir);
	if (!vdd_cpu_dir)
		goto err_0;

	d_edp = debugfs_create_file("cpu_edp", S_IRUGO, vdd_cpu_dir, NULL,
				    &cpu_edp_debugfs_fops);
	if (!d_edp)
		goto err_1;

	d_edp_limit = debugfs_create_file("cpu_edp_limit", S_IRUGO, vdd_cpu_dir,
					  NULL, &cpu_edp_limit_debugfs_fops);
	if (!d_edp_limit)
		goto err_2;

	d_edp_reg_override = debugfs_create_file("cpu_edp_reg_override",
					S_IRUGO | S_IWUSR, vdd_cpu_dir, NULL,
					&cpu_edp_reg_override_debugfs_fops);
	if (!d_edp_reg_override)
		goto err_3;

	d_reg_idle_cur = debugfs_create_file("reg_idle_ma",
					S_IRUGO | S_IWUSR, vdd_cpu_dir, NULL,
					&reg_idle_cur_debugfs_fops);
	if (!d_reg_idle_cur)
		goto err_4;

	if (tegra_core_edp_debugfs_init(edp_dir))
		return -ENOMEM;

	return 0;

err_4:
	debugfs_remove(d_edp_reg_override);
err_3:
	debugfs_remove(d_edp_limit);
err_2:
	debugfs_remove(d_edp);
err_1:
	debugfs_remove(vdd_cpu_dir);
err_0:
	return -ENOMEM;
}

late_initcall(tegra_edp_debugfs_init);
#endif /* CONFIG_DEBUG_FS */
