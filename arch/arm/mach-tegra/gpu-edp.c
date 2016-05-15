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
#include <linux/tegra-fuse.h>
#include <linux/hashtable.h>

#include <mach/edp.h>

#include <linux/platform/tegra/dvfs.h>
#include <linux/platform/tegra/clock.h>
#include <linux/platform/tegra/common.h>

struct maxf_cache_entry {
	u32 imax;
	s16 temp_c;
	unsigned maxf;
	struct hlist_node nib; /* next in bucket*/
};

struct vdd_edp {
	char *cdev_name;
	char *cap_clk_name;
	char *edp_name;

	const int n_cores;

	unsigned int fv_lut_size;
	struct tegra_edp_freq_voltage_table *fv_lut;
	const int freq_step;

	struct tegra_edp_common_powermodel_params *params;
	int iddq_ma;

	DECLARE_HASHTABLE(maxf_cache, 7);

	int thermal_idx;

	void (*apply_cap)(struct vdd_edp *);
	struct clk *cap_clk;
	struct mutex edp_lock;

	unsigned int imax_ma;

	const int *temperatures;
	int n_temperatures;
	int temp_from_debugfs;
};

const int temps[] = { /* degree celcius (C) */
	20, 50, 70, 75, 80, 85, 90, 95, 100, 105,
};

static void apply_cap_via_clock(struct vdd_edp *);

static struct vdd_edp s_gpu = {
	.cdev_name = "gpu_edp",
	.cap_clk_name = "edp.gbus",
	.edp_name = "gpu",
	.n_cores = 1,
	.freq_step = 12000000,
	.temperatures = temps,
	.n_temperatures = ARRAY_SIZE(temps),
	.edp_lock = __MUTEX_INITIALIZER(s_gpu.edp_lock),
	.apply_cap = apply_cap_via_clock,
};

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

static int tegra_relate_freq_voltage(struct clk *c, unsigned int step,
			unsigned int freq_volt_lut_size,
			struct tegra_edp_freq_voltage_table *freq_volt_lut)
{
	unsigned int i, j, freq;
	int voltage_mv;

	for (i = 0, j = 0, freq = 0;
		 i < freq_volt_lut_size;
		 i++, freq += step) {

		/* Predict voltages */
		voltage_mv = tegra_dvfs_predict_peak_millivolts(c, freq);
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

static void _init_thermal_trips(struct vdd_edp *ctx,
			 struct thermal_trip_info *trips,
			 int *num_trips, int margin)
{
	const struct tegra_edp_gpu_limits *gpu_edp_limits;
	struct thermal_trip_info *trip_state;
	int i;

	if (!trips || !num_trips)
		return;

	if (ctx->n_temperatures > MAX_THROT_TABLE_SIZE)
		BUG();

	for (i = 0; i < ctx->n_temperatures-1; i++) {
		trip_state = &trips[*num_trips];

		trip_state->cdev_type = ctx->cdev_name;
		trip_state->trip_temp =
			(ctx->temperatures[i] * 1000) - margin;
		trip_state->trip_type = THERMAL_TRIP_ACTIVE;
		trip_state->upper = trip_state->lower = i + 1;

		(*num_trips)++;

		if (*num_trips >= THERMAL_MAX_TRIPS)
			BUG();
	}
}

void tegra_platform_gpu_edp_init(struct thermal_trip_info *trips,
				int *num_trips, int margin)
{
	_init_thermal_trips(&s_gpu, trips, num_trips, margin);
}

static unsigned int edp_calculate_maxf(
			struct tegra_edp_common_powermodel_params *params,
			unsigned int fv_lut_size,
			struct tegra_edp_freq_voltage_table *fv_lut,
			unsigned int cur_effective, int temp_c, int iddq_ma)
{
	unsigned int voltage_mv, freq_khz = 0;
	int f;
	s64 leakage_ma = 0, dyn_ma;

	for (f = fv_lut_size - 1; f >= 0; f--) {
		freq_khz = fv_lut[f].freq / 1000;
		voltage_mv = fv_lut[f].voltage_mv;

		/* Calculate leakage current */
		leakage_ma = common_edp_calculate_leakage_ma(
					params, iddq_ma,
					temp_c, voltage_mv, 0);

		/* Calculate dynamic current */
		dyn_ma = common_edp_calculate_dynamic_ma(params,
					voltage_mv, 0, freq_khz);

		if ((leakage_ma + dyn_ma) <= cur_effective)
			goto end;

		freq_khz = 0;
	}

 end:
	return freq_khz;
}

#define make_key(imax, temp_c) ((imax << 8) ^ temp_c)
static unsigned edp_get_maxf(struct vdd_edp *ctx, int temp_c)
{
	unsigned maxf;
	struct maxf_cache_entry *me;
	u32 key = make_key(ctx->imax_ma, temp_c);

	BUG_ON(!mutex_is_locked(&ctx->edp_lock));

	/* check cache */
	hash_for_each_possible(ctx->maxf_cache, me, nib, key)
		if (me->imax == ctx->imax_ma && me->temp_c == temp_c) {
			maxf = me->maxf;
			return maxf;
		}

	/* service a miss */
	maxf = edp_calculate_maxf(ctx->params,
				  ctx->fv_lut_size,
				  ctx->fv_lut,
				  ctx->imax_ma,
				  temp_c,
				  ctx->iddq_ma);

	/* best effort to cache the result */
	me = kzalloc(sizeof(*me), GFP_KERNEL);
	if (!IS_ERR_OR_NULL(me)) {
		me->imax = ctx->imax_ma;
		me->temp_c = temp_c;
		me->maxf = maxf;
		hash_add(ctx->maxf_cache, &me->nib, key);
	}
	return maxf;
}

static void edp_flush_maxf_cache_locked(struct vdd_edp *ctx)
{
	int bucket;
	struct hlist_node *tmp;
	struct maxf_cache_entry *me;

	hash_for_each_safe(ctx->maxf_cache, bucket, tmp, me, nib) {
		hash_del(&me->nib);
		kfree(me);
	}
}

static int init_fv_lut_locked(struct vdd_edp *ctx)
{
	struct clk *c;
	int minf, maxf, ret = 0;

	kfree(ctx->fv_lut);
	edp_flush_maxf_cache_locked(ctx);

	c = clk_get_parent(ctx->cap_clk);
	minf = 0;
	maxf = clk_get_max_rate(c);

	ctx->fv_lut_size = (maxf - minf) / ctx->freq_step + 1;
	ctx->fv_lut = kmalloc(
		sizeof(struct tegra_edp_freq_voltage_table)
			      * ctx->fv_lut_size, GFP_KERNEL);
	if (!ctx->fv_lut) {
		pr_err("%s: failed alloc mem for freq/voltage LUT\n",
			 __func__);
		ctx->fv_lut_size = 0;
		return -ENOMEM;
	}

	ret = tegra_relate_freq_voltage(c, ctx->freq_step,
				ctx->fv_lut_size, ctx->fv_lut);
	if (ret) {
		kfree(ctx->fv_lut);
		ctx->fv_lut = NULL;
		ctx->fv_lut_size = 0;
	}

	return ret;
}

static int init_fv_lut(struct vdd_edp *ctx)
{
	int ret;

	mutex_lock(&ctx->edp_lock);
	ret = init_fv_lut_locked(ctx);
	mutex_unlock(&ctx->edp_lock);

	return ret;
}

static void __init _init_edp_limits(struct vdd_edp *ctx,
				    unsigned int regulator_ma)
{
	if (ctx->cap_clk_name) {
		ctx->cap_clk = tegra_get_clock_by_name(ctx->cap_clk_name);
		if (!ctx->cap_clk)
			pr_err("%s: cannot get clock:%s\n",
			       __func__, ctx->cap_clk_name);
	}
	ctx->thermal_idx = 0;

	ctx->imax_ma = regulator_ma;

	init_fv_lut(ctx);
}

void __init tegra_init_gpu_edp_limits(unsigned int regulator_ma)
{
	static u32 tegra_chip_id;
	tegra_chip_id = tegra_get_chip_id();

	if (tegra_chip_id == TEGRA_CHIPID_TEGRA12)
		s_gpu.params = tegra12x_get_gpu_powermodel_params();
	else if (tegra_chip_id == TEGRA_CHIPID_TEGRA13)
		s_gpu.params = tegra13x_get_gpu_powermodel_params();
	else
		BUG();

	s_gpu.iddq_ma = tegra_get_gpu_iddq_value();
	pr_debug("%s: %s IDDQ value %d\n",
		 __func__, s_gpu.edp_name, s_gpu.iddq_ma);

	_init_edp_limits(&s_gpu, regulator_ma);
}

static int edp_get_cdev_max_state(struct thermal_cooling_device *cdev,
				       unsigned long *max_state)
{
	struct vdd_edp *ctx = cdev->devdata;
	*max_state = ctx->n_temperatures;
	return 0;
}

static int edp_get_cdev_cur_state(struct thermal_cooling_device *cdev,
				       unsigned long *cur_state)
{
	struct vdd_edp *ctx = cdev->devdata;
	*cur_state = ctx->thermal_idx;
	return 0;
}

static void apply_cap_via_clock(struct vdd_edp *ctx)
{
	unsigned long clk_rate;
	BUG_ON(IS_ERR_OR_NULL(ctx->cap_clk));

	clk_rate = edp_get_maxf(ctx, ctx->temperatures[ctx->thermal_idx]);
	clk_set_rate(ctx->cap_clk, clk_rate * 1000);
}

static int edp_set_cdev_state(struct thermal_cooling_device *cdev,
				   unsigned long cur_state)
{
	struct vdd_edp *ctx = cdev->devdata;

	mutex_lock(&ctx->edp_lock);

	BUG_ON(cur_state >= ctx->n_temperatures);

	ctx->thermal_idx = cur_state;
	if (ctx->apply_cap)
		ctx->apply_cap(ctx);

	mutex_unlock(&ctx->edp_lock);

	return 0;
}

static struct thermal_cooling_device_ops edp_cooling_ops = {
	.get_max_state = edp_get_cdev_max_state,
	.get_cur_state = edp_get_cdev_cur_state,
	.set_cur_state = edp_set_cdev_state,
};

static int __init cdev_register(struct vdd_edp *ctx)
{
	struct thermal_cooling_device *edp_cdev;

	if (!ctx->cdev_name)
		return 0;

	edp_cdev = thermal_cooling_device_register(ctx->cdev_name,
						   ctx,
						   &edp_cooling_ops);
	if (IS_ERR_OR_NULL(edp_cdev))
		pr_err("Failed to register '%s' cooling device\n",
			ctx->cdev_name);

	return PTR_RET(edp_cdev);
}

static int __init gpu_edp_cdev_init(void)
{
	return cdev_register(&s_gpu);
}
module_init(gpu_edp_cdev_init);

#ifdef CONFIG_DEBUG_FS

static int edp_show_maxf(void *data, u64 *val)
{
	struct vdd_edp *ctx = data;

	mutex_lock(&ctx->edp_lock);
	*val = edp_get_maxf(ctx, ctx->temp_from_debugfs);
	mutex_unlock(&ctx->edp_lock);

	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(edp_maxf_fops, edp_show_maxf, NULL, "%llu\n");

static int edp_cache_show(struct seq_file *s, void *data)
{
	int bucket;
	struct maxf_cache_entry *me;
	struct vdd_edp *ctx = s->private;

	seq_printf(s, "%10s %10s %10s\n",
		   "imax [mA]", "temp['C]", "fmax [Hz]");

	hash_for_each(ctx->maxf_cache, bucket, me, nib)
		seq_printf(s, "%10d %10d %10d\n",
			   me->imax, me->temp_c, me->maxf);

	return 0;
}

static int edp_cache_open(struct inode *inode, struct file *file)
{
	return single_open(file, edp_cache_show, inode->i_private);
}

static const struct file_operations edp_cache_fops = {
	.open = edp_cache_open,
	.read = seq_read,
};


static int edp_imax_get(void *data, u64 *val)
{
	struct vdd_edp *ctx = data;
	*val = ctx->imax_ma;
	return 0;
}

static int edp_imax_set(void *data, u64 new_imax)
{
	struct vdd_edp *ctx = data;
	struct thermal_cooling_device hack = {.devdata = ctx};

	ctx->imax_ma = new_imax;

	edp_set_cdev_state(&hack, ctx->thermal_idx);
	pr_info("Reinitialized %s EDP capping with regulator current limit %u mA\n",
		ctx->edp_name, ctx->imax_ma);
	return 0;

}
DEFINE_SIMPLE_ATTRIBUTE(edp_imax_fops, edp_imax_get, edp_imax_set, "%llu\n")

static int __init vdd_edp_debugfs_init(struct vdd_edp *ctx,
				       struct dentry *parent)
{
	parent = debugfs_create_dir(ctx->edp_name, parent);
	if (!parent)
		return -ENOMEM;

	debugfs_create_file("imax_ma",
			    S_IRUGO | S_IWUSR, parent, ctx,
			    &edp_imax_fops);
	debugfs_create_file("edp_cache", S_IRUGO, parent, ctx,
			    &edp_cache_fops);
	debugfs_create_file("maxf", S_IRUGO, parent, ctx,
			    &edp_maxf_fops);
	debugfs_create_u32_array(".vf_lut", S_IRUGO, parent,
				 (u32 *)ctx->fv_lut, 2*ctx->fv_lut_size);
	debugfs_create_u32(".temp_c", S_IRUGO | S_IWUSR, parent,
			   (u32 *)&ctx->temp_from_debugfs);

	return 0;
}

static int __init tegra_edp_debugfs_init(void)
{
	struct dentry *edp_dir;

	if (!tegra_platform_is_silicon())
		return -ENOSYS;

	edp_dir = debugfs_create_dir("edp2", NULL);
	if (!edp_dir)
		return -ENOMEM;

	vdd_edp_debugfs_init(&s_gpu, edp_dir);

	return 0;
}

late_initcall(tegra_edp_debugfs_init);
#endif /* CONFIG_DEBUG_FS */
