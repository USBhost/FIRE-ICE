/*
 * drivers/video/tegra/host/gk20a/platform_gk20a_tegra.c
 *
 * GK20A Tegra Platform Interface
 *
 * Copyright (c) 2014, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/debugfs.h>
#include <linux/tegra-powergate.h>
#include <linux/platform_data/tegra_edp.h>
#include <linux/nvhost_ioctl.h>
#include <linux/dma-buf.h>
#include <linux/nvmap.h>
#include <linux/tegra_pm_domains.h>

#include <mach/irqs.h>

#include "../../../arch/arm/mach-tegra/iomap.h"
#include <linux/platform/tegra/clock.h>
#include <linux/platform/tegra/dvfs.h>
#include <linux/platform/tegra/common.h>
#include <linux/clk/tegra.h>
#include <mach/tegra_emc.h>

#include "gk20a.h"
#include "hal_gk20a.h"
#include "platform_gk20a.h"
#include "gk20a_scale.h"

#define TEGRA_GK20A_INTR		INT_GPU
#define TEGRA_GK20A_INTR_NONSTALL	INT_GPU_NONSTALL

#define TEGRA_GK20A_SIM_BASE 0x538F0000 /*tbd: get from iomap.h */
#define TEGRA_GK20A_SIM_SIZE 0x1000     /*tbd: this is a high-side guess */

#define TEGRA_GK20A_BW_PER_FREQ 32
#define TEGRA_GM20B_BW_PER_FREQ 64
#define TEGRA_DDR3_BW_PER_FREQ 16
#define TEGRA_DDR4_BW_PER_FREQ 16

extern struct device tegra_vpr_dev;
struct gk20a_platform t132_gk20a_tegra_platform;

struct gk20a_emc_params {
	long bw_ratio;
	long freq_last_set;
};

/*
 * 20.12 fixed point arithmetic
 */

static const int FXFRAC = 12;
static const int FX_HALF = (1 << 12) / 2;

#define INT_TO_FX(x) ((x) << FXFRAC)
#define FX_TO_INT(x) ((x) >> FXFRAC)

#define MHZ_TO_HZ(x) ((x) * 1000000)
#define HZ_TO_MHZ(x) ((x) / 1000000)

int FXMUL(int x, int y)
{
	return ((long long) x * (long long) y) >> FXFRAC;
}

int FXDIV(int x, int y)
{
	/* long long div operation not supported, must shift manually. This
	 * would have been
	 *
	 *    return (((long long) x) << FXFRAC) / (long long) y;
	 */
	int pos, t;
	if (x == 0)
		return 0;

	/* find largest allowable right shift to numerator, limit to FXFRAC */
	t = x < 0 ? -x : x;
	pos = 31 - fls(t); /* fls can't be 32 if x != 0 */
	if (pos > FXFRAC)
		pos = FXFRAC;

	y >>= FXFRAC - pos;
	if (y == 0)
		return 0x7FFFFFFF; /* overflow, return MAX_FIXED */

	return (x << pos) / y;
}

static void gk20a_tegra_secure_page_destroy(struct platform_device *pdev,
				       struct secure_page_buffer *secure_buffer)
{
	dma_free_attrs(&tegra_vpr_dev, secure_buffer->size,
			(void *)(uintptr_t)secure_buffer->iova,
			secure_buffer->iova, &secure_buffer->attrs);
}

static int gk20a_tegra_secure_page_alloc(struct platform_device *pdev)
{
	struct gk20a_platform *platform = platform_get_drvdata(pdev);
	struct secure_page_buffer *secure_buffer = &platform->secure_buffer;
	DEFINE_DMA_ATTRS(attrs);
	dma_addr_t iova;
	size_t size = PAGE_SIZE;

	(void)dma_alloc_attrs(&tegra_vpr_dev, size, &iova,
				      DMA_MEMORY_NOMAP, &attrs);
	if (dma_mapping_error(&tegra_vpr_dev, iova))
		return -ENOMEM;

	secure_buffer->size = size;
	secure_buffer->iova = iova;
	secure_buffer->attrs = attrs;
	secure_buffer->destroy = gk20a_tegra_secure_page_destroy;

	return 0;
}

static void gk20a_tegra_secure_destroy(struct platform_device *pdev,
				       struct gr_ctx_buffer_desc *desc)
{
	gk20a_free_sgtable(&desc->sgt);
	dma_free_attrs(&tegra_vpr_dev, desc->size,
			(void *)(uintptr_t)desc->iova,
			desc->iova, &desc->attrs);
}

static int gk20a_tegra_secure_alloc(struct platform_device *pdev,
				    struct gr_ctx_buffer_desc *desc,
				    size_t size)
{
	struct gk20a_platform *platform = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
	DEFINE_DMA_ATTRS(attrs);
	dma_addr_t iova;
	struct sg_table *sgt;
	struct page *page;
	int err = 0;

	if (!platform->secure_alloc_ready)
		return -EINVAL;

	(void)dma_alloc_attrs(&tegra_vpr_dev, size, &iova,
				      DMA_MEMORY_NOMAP, &attrs);
	if (dma_mapping_error(&tegra_vpr_dev, iova))
		return -ENOMEM;

	desc->iova = iova;
	desc->size = size;
	desc->attrs = attrs;
	desc->destroy = gk20a_tegra_secure_destroy;

	sgt = kzalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt) {
		gk20a_err(dev, "failed to allocate memory\n");
		goto fail;
	}
	err = sg_alloc_table(sgt, 1, GFP_KERNEL);
	if (err) {
		gk20a_err(dev, "failed to allocate sg_table\n");
		goto fail_sgt;
	}
	page = phys_to_page(iova);
	sg_set_page(sgt->sgl, page, size, 0);
	sg_dma_address(sgt->sgl) = iova;

	desc->sgt = sgt;

	return err;

fail_sgt:
	kfree(sgt);
fail:
	dma_free_attrs(&tegra_vpr_dev, desc->size,
			(void *)(uintptr_t)&desc->iova,
			desc->iova, &desc->attrs);
	return err;
}

/*
 * gk20a_tegra_get_emc_rate()
 *
 * This function returns the minimum emc clock based on gpu frequency
 */

static unsigned long gk20a_tegra_get_emc_rate(struct gk20a *g,
				struct gk20a_emc_params *emc_params)
{
	unsigned long gpu_freq, gpu_fmax_at_vmin;
	unsigned long emc_rate, emc_scale;

	gpu_freq = clk_get_rate(g->clk.tegra_clk);
	gpu_fmax_at_vmin = tegra_dvfs_get_fmax_at_vmin_safe_t(
		clk_get_parent(g->clk.tegra_clk));

	/* When scaling emc, account for the gpu load when the
	 * gpu frequency is less than or equal to fmax@vmin. */
	if (gpu_freq <= gpu_fmax_at_vmin)
		emc_scale = min(g->pmu.load_avg, g->emc3d_ratio);
	else
		emc_scale = g->emc3d_ratio;

	emc_rate =
		(HZ_TO_MHZ(gpu_freq) * emc_params->bw_ratio * emc_scale) / 1000;

	return MHZ_TO_HZ(emc_rate);
}

/*
 * gk20a_tegra_postscale(profile, freq)
 *
 * This function sets emc frequency based on current gpu frequency
 */

static void gk20a_tegra_postscale(struct platform_device *pdev,
				  unsigned long freq)
{
	struct gk20a_platform *platform = platform_get_drvdata(pdev);
	struct gk20a_scale_profile *profile = platform->g->scale_profile;
	struct gk20a_emc_params *emc_params = profile->private_data;
	struct gk20a *g = get_gk20a(pdev);
	struct clk *emc_clk = platform->clk[2];
	enum tegra_chipid chip_id = tegra_get_chip_id();
	unsigned long emc_target;
	long emc_freq_lower, emc_freq_upper, emc_freq_rounded;

	emc_target = gk20a_tegra_get_emc_rate(g, emc_params);

	switch (chip_id) {
	case TEGRA_CHIPID_TEGRA12:
	case TEGRA_CHIPID_TEGRA13:
		/* T124 and T132 don't apply any rounding. The resulting
		 * emc frequency gets implicitly rounded up after issuing
		 * the clock_set_request.
		 * So explicitly round up the emc target here to achieve
		 * the same outcome. */
		emc_freq_rounded =
			tegra_emc_round_rate_updown(emc_target, true);
		break;

	case TEGRA_CHIPID_TEGRA21:
		emc_freq_lower = tegra_emc_round_rate_updown(emc_target, false);
		emc_freq_upper = tegra_emc_round_rate_updown(emc_target, true);

		/* round to the nearest frequency step */
		if (emc_target < (emc_freq_lower + emc_freq_upper) / 2)
			emc_freq_rounded = emc_freq_lower;
		else
			emc_freq_rounded = emc_freq_upper;
		break;

	case TEGRA_CHIPID_UNKNOWN:
	default:
		/* a proper rounding function needs to be implemented
		 * for emc in t18x */
		emc_freq_rounded = clk_round_rate(emc_clk, emc_target);
		break;
	}

	/* only change the emc clock if new rounded frequency is different
	 * from previously set emc rate */
	if (emc_freq_rounded != emc_params->freq_last_set) {
		clk_set_rate(emc_clk, emc_freq_rounded);
		emc_params->freq_last_set = emc_freq_rounded;
	}
}

/*
 * gk20a_tegra_prescale(profile, freq)
 *
 * This function informs EDP about changed constraints.
 */

static void gk20a_tegra_prescale(struct platform_device *pdev)
{
	struct gk20a *g = get_gk20a(pdev);
	u32 avg = 0;

	gk20a_pmu_load_norm(g, &avg);
	tegra_edp_notify_gpu_load(avg, gk20a_clk_get_rate(g));
}

/*
 * gk20a_tegra_calibrate_emc()
 *
 */

void gk20a_tegra_calibrate_emc(struct platform_device *pdev,
			       struct gk20a_emc_params *emc_params)
{
	enum tegra_chipid cid = tegra_get_chipid();
	long gpu_bw, emc_bw;

	/* store gpu bw based on soc */
	switch (cid) {
	case TEGRA_CHIPID_TEGRA21:
		gpu_bw = TEGRA_GM20B_BW_PER_FREQ;
		break;
	case TEGRA_CHIPID_TEGRA12:
	case TEGRA_CHIPID_TEGRA13:
		gpu_bw = TEGRA_GK20A_BW_PER_FREQ;
		break;
	case TEGRA_CHIPID_UNKNOWN:
	default:
		gpu_bw = 0;
		break;
	}

	/* TODO detect DDR type.
	 * Okay for now since DDR3 and DDR4 have the same BW ratio */
	emc_bw = TEGRA_DDR3_BW_PER_FREQ;

	/* Calculate the bandwidth ratio of gpu_freq <-> emc_freq
	 *   NOTE the ratio must come out as an integer */
	emc_params->bw_ratio = (gpu_bw / emc_bw);
}

/*
 * gk20a_tegra_is_railgated()
 *
 * Check status of gk20a power rail
 */

static bool gk20a_tegra_is_railgated(struct platform_device *pdev)
{
	return !tegra_powergate_is_powered(TEGRA_POWERGATE_GPU);
}

/*
 * gk20a_tegra_railgate()
 *
 * Gate (disable) gk20a power rail
 */

static int gk20a_tegra_railgate(struct platform_device *pdev)
{
	if (tegra_powergate_is_powered(TEGRA_POWERGATE_GPU))
		tegra_powergate_partition(TEGRA_POWERGATE_GPU);
	return 0;
}

/*
 * gk20a_tegra_unrailgate()
 *
 * Ungate (enable) gk20a power rail
 */

static int gk20a_tegra_unrailgate(struct platform_device *pdev)
{
	int ret;
	ret = tegra_unpowergate_partition(TEGRA_POWERGATE_GPU);
	return ret;
}

struct {
	char *name;
	unsigned long default_rate;
} tegra_gk20a_clocks[] = {
	{"PLLG_ref", UINT_MAX},
	{"pwr", 204000000},
	{"emc", UINT_MAX} };

/*
 * gk20a_tegra_get_clocks()
 *
 * This function finds clocks in tegra platform and populates
 * the clock information to gk20a platform data.
 */

static int gk20a_tegra_get_clocks(struct platform_device *pdev)
{
	struct gk20a_platform *platform = platform_get_drvdata(pdev);
	char devname[16];
	int i;
	int ret = 0;

	snprintf(devname, sizeof(devname),
		 (pdev->id <= 0) ? "tegra_%s" : "tegra_%s.%d\n",
		 pdev->name, pdev->id);

	platform->num_clks = 0;
	for (i = 0; i < ARRAY_SIZE(tegra_gk20a_clocks); i++) {
		long rate = tegra_gk20a_clocks[i].default_rate;
		struct clk *c;

		c = clk_get_sys(devname, tegra_gk20a_clocks[i].name);
		if (IS_ERR(c)) {
			ret = PTR_ERR(c);
			goto err_get_clock;
		}
		rate = clk_round_rate(c, rate);
		clk_set_rate(c, rate);
		platform->clk[i] = c;
	}
	platform->num_clks = i;

	return 0;

err_get_clock:

	while (i--)
		clk_put(platform->clk[i]);
	return ret;
}

static void gk20a_tegra_scale_init(struct platform_device *pdev)
{
	struct gk20a_platform *platform = gk20a_get_platform(pdev);
	struct gk20a_scale_profile *profile = platform->g->scale_profile;
	struct gk20a_emc_params *emc_params;

	if (!profile)
		return;

	emc_params = kzalloc(sizeof(*emc_params), GFP_KERNEL);
	if (!emc_params)
		return;

	emc_params->freq_last_set = -1;
	gk20a_tegra_calibrate_emc(pdev, emc_params);

	profile->private_data = emc_params;
}

static void gk20a_tegra_debug_dump(struct platform_device *pdev)
{
	struct gk20a_platform *platform = gk20a_get_platform(pdev);
	struct gk20a *g = platform->g;
	nvhost_debug_dump_device(g->dev);
}

static int gk20a_tegra_probe(struct platform_device *dev)
{
	struct gk20a_platform *platform = gk20a_get_platform(dev);

	if (tegra_get_chipid() == TEGRA_CHIPID_TEGRA13) {
		t132_gk20a_tegra_platform.g = platform->g;
		*platform = t132_gk20a_tegra_platform;
	}

	gk20a_tegra_get_clocks(dev);

	return 0;
}

static int gk20a_tegra_late_probe(struct platform_device *dev)
{
	struct gk20a_platform *platform = gk20a_get_platform(dev);

	/* Make gk20a power domain a subdomain of host1x */
	nvhost_register_client_domain(&platform->g->pd);

	/* Initialise tegra specific scaling quirks */
	gk20a_tegra_scale_init(dev);

	return 0;
}

static int gk20a_tegra_suspend(struct device *dev)
{
	tegra_edp_notify_gpu_load(0, 0);
	return 0;
}

static struct resource gk20a_tegra_resources[] = {
	{
	.start = TEGRA_GK20A_BAR0_BASE,
	.end   = TEGRA_GK20A_BAR0_BASE + TEGRA_GK20A_BAR0_SIZE - 1,
	.flags = IORESOURCE_MEM,
	},
	{
	.start = TEGRA_GK20A_BAR1_BASE,
	.end   = TEGRA_GK20A_BAR1_BASE + TEGRA_GK20A_BAR1_SIZE - 1,
	.flags = IORESOURCE_MEM,
	},
	{ /* Used on ASIM only */
	.start = TEGRA_GK20A_SIM_BASE,
	.end   = TEGRA_GK20A_SIM_BASE + TEGRA_GK20A_SIM_SIZE - 1,
	.flags = IORESOURCE_MEM,
	},
	{
	.start = TEGRA_GK20A_INTR,
	.end   = TEGRA_GK20A_INTR,
	.flags = IORESOURCE_IRQ,
	},
	{
	.start = TEGRA_GK20A_INTR_NONSTALL,
	.end   = TEGRA_GK20A_INTR_NONSTALL,
	.flags = IORESOURCE_IRQ,
	},
};

struct gk20a_platform t132_gk20a_tegra_platform = {
	.has_syncpoints = true,

	/* power management configuration */
	.railgate_delay		= 500,
	.clockgate_delay	= 50,
	.can_railgate		= true,

	.probe = gk20a_tegra_probe,
	.late_probe = gk20a_tegra_late_probe,

	/* power management callbacks */
	.suspend = gk20a_tegra_suspend,
	.railgate = gk20a_tegra_railgate,
	.unrailgate = gk20a_tegra_unrailgate,
	.is_railgated = gk20a_tegra_is_railgated,

	/* frequency scaling configuration */
	.prescale = gk20a_tegra_prescale,
	.postscale = gk20a_tegra_postscale,
	.devfreq_governor = "nvhost_podgov",
	.qos_id = PM_QOS_GPU_FREQ_MIN,

	.secure_alloc = gk20a_tegra_secure_alloc,
	.secure_page_alloc = gk20a_tegra_secure_page_alloc,
	.dump_platform_dependencies = gk20a_tegra_debug_dump,
};

struct gk20a_platform gk20a_tegra_platform = {
	.has_syncpoints = true,

	/* power management configuration */
	.railgate_delay		= 500,
	.clockgate_delay	= 50,
	.can_railgate		= true,

	.probe = gk20a_tegra_probe,
	.late_probe = gk20a_tegra_late_probe,

	/* power management callbacks */
	.suspend = gk20a_tegra_suspend,
	.railgate = gk20a_tegra_railgate,
	.unrailgate = gk20a_tegra_unrailgate,
	.is_railgated = gk20a_tegra_is_railgated,

	/* frequency scaling configuration */
	.prescale = gk20a_tegra_prescale,
	.postscale = gk20a_tegra_postscale,
	.devfreq_governor = "nvhost_podgov",
	.qos_id = PM_QOS_GPU_FREQ_MIN,

	.secure_alloc = gk20a_tegra_secure_alloc,
	.secure_page_alloc = gk20a_tegra_secure_page_alloc,
	.dump_platform_dependencies = gk20a_tegra_debug_dump,
};

struct platform_device tegra_gk20a_device = {
	.name		= "gk20a",
	.resource	= gk20a_tegra_resources,
	.num_resources	= ARRAY_SIZE(gk20a_tegra_resources),
	.dev		= {
		.platform_data = &gk20a_tegra_platform,
	},
};
