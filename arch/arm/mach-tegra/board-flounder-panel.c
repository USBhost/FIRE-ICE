/*
 * arch/arm/mach-tegra/board-flounder-panel.c
 *
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/ioport.h>
#include <linux/fb.h>
#include <linux/nvmap.h>
#include <linux/nvhost.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/tegra_pwm_bl.h>
#include <linux/regulator/consumer.h>
#include <linux/pwm_backlight.h>
#include <linux/of.h>
#include <linux/dma-contiguous.h>
#include <linux/clk.h>

#include <mach/irqs.h>
#include <mach/dc.h>
#include <mach/io_dpd.h>

#include "board.h"
#include "devices.h"
#include "gpio-names.h"
#include "board-flounder.h"
#include "board-panel.h"
#include <linux/platform/tegra/common.h>
#include "iomap.h"
#include "tegra12_host1x_devices.h"
#include <linux/platform/tegra/dvfs.h>

struct platform_device * __init flounder_host1x_init(void)
{
	struct platform_device *pdev = NULL;

#ifdef CONFIG_TEGRA_GRHOST
	pdev = to_platform_device(bus_find_device_by_name(
		&platform_bus_type, NULL, "host1x"));

	if (!pdev) {
		pr_err("host1x devices registration failed\n");
		return NULL;
	}
#endif
	return pdev;
}

/* hdmi related regulators */
static struct regulator *flounder_hdmi_reg;
static struct regulator *flounder_hdmi_pll;
static struct regulator *flounder_hdmi_vddio;

static struct resource flounder_disp1_resources[] = {
	{
		.name	= "irq",
		.start	= INT_DISPLAY_GENERAL,
		.end	= INT_DISPLAY_GENERAL,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "regs",
		.start	= TEGRA_DISPLAY_BASE,
		.end	= TEGRA_DISPLAY_BASE + TEGRA_DISPLAY_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "fbmem",
		.start	= 0, /* Filled in by flounder_panel_init() */
		.end	= 0, /* Filled in by flounder_panel_init() */
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "ganged_dsia_regs",
		.start	= 0, /* Filled in the panel file by init_resources() */
		.end	= 0, /* Filled in the panel file by init_resources() */
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "ganged_dsib_regs",
		.start	= 0, /* Filled in the panel file by init_resources() */
		.end	= 0, /* Filled in the panel file by init_resources() */
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "dsi_regs",
		.start	= 0, /* Filled in the panel file by init_resources() */
		.end	= 0, /* Filled in the panel file by init_resources() */
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "mipi_cal",
		.start	= TEGRA_MIPI_CAL_BASE,
		.end	= TEGRA_MIPI_CAL_BASE + TEGRA_MIPI_CAL_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct resource flounder_disp2_resources[] = {
	{
		.name	= "irq",
		.start	= INT_DISPLAY_B_GENERAL,
		.end	= INT_DISPLAY_B_GENERAL,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name	= "regs",
		.start	= TEGRA_DISPLAY2_BASE,
		.end	= TEGRA_DISPLAY2_BASE + TEGRA_DISPLAY2_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "fbmem",
		.start	= 0, /* Filled in by flounder_panel_init() */
		.end	= 0, /* Filled in by flounder_panel_init() */
		.flags	= IORESOURCE_MEM,
	},
	{
		.name	= "hdmi_regs",
		.start	= TEGRA_HDMI_BASE,
		.end	= TEGRA_HDMI_BASE + TEGRA_HDMI_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
};


static struct tegra_dc_out flounder_disp1_out = {
	.type		= TEGRA_DC_OUT_DSI,
};

static int flounder_hdmi_enable(struct device *dev)
{
	int ret;
	if (!flounder_hdmi_reg) {
		flounder_hdmi_reg = regulator_get(dev, "avdd_hdmi");
		if (IS_ERR_OR_NULL(flounder_hdmi_reg)) {
			pr_err("hdmi: couldn't get regulator avdd_hdmi\n");
			flounder_hdmi_reg = NULL;
			return PTR_ERR(flounder_hdmi_reg);
		}
	}
	ret = regulator_enable(flounder_hdmi_reg);
	if (ret < 0) {
		pr_err("hdmi: couldn't enable regulator avdd_hdmi\n");
		return ret;
	}
	if (!flounder_hdmi_pll) {
		flounder_hdmi_pll = regulator_get(dev, "avdd_hdmi_pll");
		if (IS_ERR_OR_NULL(flounder_hdmi_pll)) {
			pr_err("hdmi: couldn't get regulator avdd_hdmi_pll\n");
			flounder_hdmi_pll = NULL;
			regulator_put(flounder_hdmi_reg);
			flounder_hdmi_reg = NULL;
			return PTR_ERR(flounder_hdmi_pll);
		}
	}
	ret = regulator_enable(flounder_hdmi_pll);
	if (ret < 0) {
		pr_err("hdmi: couldn't enable regulator avdd_hdmi_pll\n");
		return ret;
	}
	return 0;
}

static int flounder_hdmi_disable(void)
{
	if (flounder_hdmi_reg) {
		regulator_disable(flounder_hdmi_reg);
		regulator_put(flounder_hdmi_reg);
		flounder_hdmi_reg = NULL;
	}

	if (flounder_hdmi_pll) {
		regulator_disable(flounder_hdmi_pll);
		regulator_put(flounder_hdmi_pll);
		flounder_hdmi_pll = NULL;
	}
	return 0;
}

static int flounder_hdmi_postsuspend(void)
{
	if (flounder_hdmi_vddio) {
		regulator_disable(flounder_hdmi_vddio);
		regulator_put(flounder_hdmi_vddio);
		flounder_hdmi_vddio = NULL;
	}
	return 0;
}

static int flounder_hdmi_hotplug_init(struct device *dev)
{
	if (!flounder_hdmi_vddio) {
		flounder_hdmi_vddio = regulator_get(dev, "vdd_hdmi_5v0");
		if (WARN_ON(IS_ERR(flounder_hdmi_vddio))) {
			pr_err("%s: couldn't get regulator vdd_hdmi_5v0: %ld\n",
				__func__, PTR_ERR(flounder_hdmi_vddio));
				flounder_hdmi_vddio = NULL;
		} else {
			return regulator_enable(flounder_hdmi_vddio);
		}
	}

	return 0;
}

struct tmds_config flounder_tmds_config[] = {
	{ /* 480p/576p / 25.2MHz/27MHz modes */
	.pclk = 27000000,
	.pll0 = 0x01003110,
	.pll1 = 0x00300F00,
	.pe_current = 0x08080808,
	.drive_current = 0x2e2e2e2e,
	.peak_current = 0x00000000,
	},
	{ /* 720p / 74.25MHz modes */
	.pclk = 74250000,
	.pll0 =  0x01003310,
	.pll1 = 0x10300F00,
	.pe_current = 0x08080808,
	.drive_current = 0x20202020,
	.peak_current = 0x00000000,
	},
	{ /* 1080p / 148.5MHz modes */
	.pclk = 148500000,
	.pll0 = 0x01003310,
	.pll1 = 0x10300F00,
	.pe_current = 0x08080808,
	.drive_current = 0x20202020,
	.peak_current = 0x00000000,
	},
	{
	.pclk = INT_MAX,
	.pll0 = 0x01003310,
	.pll1 = 0x10300F00,
	.pe_current = 0x08080808,
	.drive_current = 0x3A353536, /* lane3 needs a slightly lower current */
	.peak_current = 0x00000000,
	},
};

struct tegra_hdmi_out flounder_hdmi_out = {
	.tmds_config = flounder_tmds_config,
	.n_tmds_config = ARRAY_SIZE(flounder_tmds_config),
};


#ifdef CONFIG_FRAMEBUFFER_CONSOLE
static struct tegra_dc_mode hdmi_panel_modes[] = {
	{
		.pclk =			KHZ2PICOS(25200),
		.h_ref_to_sync =	1,
		.v_ref_to_sync =	1,
		.h_sync_width =		96,	/* hsync_len */
		.v_sync_width =		2,	/* vsync_len */
		.h_back_porch =		48,	/* left_margin */
		.v_back_porch =		33,	/* upper_margin */
		.h_active =		640,	/* xres */
		.v_active =		480,	/* yres */
		.h_front_porch =	16,	/* right_margin */
		.v_front_porch =	10,	/* lower_margin */
	},
};
#endif /* CONFIG_FRAMEBUFFER_CONSOLE */

static struct tegra_dc_out flounder_disp2_out = {
	.type		= TEGRA_DC_OUT_HDMI,
	.flags		= TEGRA_DC_OUT_HOTPLUG_HIGH,
	.parent_clk	= "pll_d2",

	.dcc_bus	= 3,
	.hotplug_gpio	= flounder_hdmi_hpd,
	.hdmi_out	= &flounder_hdmi_out,

	/* TODO: update max pclk to POR */
	.max_pixclock	= KHZ2PICOS(297000),
#ifdef CONFIG_FRAMEBUFFER_CONSOLE
	.modes = hdmi_panel_modes,
	.n_modes = ARRAY_SIZE(hdmi_panel_modes),
	.depth = 24,
#endif /* CONFIG_FRAMEBUFFER_CONSOLE */

	.align		= TEGRA_DC_ALIGN_MSB,
	.order		= TEGRA_DC_ORDER_RED_BLUE,

	.enable		= flounder_hdmi_enable,
	.disable	= flounder_hdmi_disable,
	.postsuspend	= flounder_hdmi_postsuspend,
	.hotplug_init	= flounder_hdmi_hotplug_init,
};

static struct tegra_fb_data flounder_disp1_fb_data = {
	.win		= 0,
	.bits_per_pixel = 32,
	.flags		= TEGRA_FB_FLIP_ON_PROBE,
};

static struct tegra_dc_platform_data flounder_disp1_pdata = {
	.flags		= TEGRA_DC_FLAG_ENABLED,
	.default_out	= &flounder_disp1_out,
	.fb		= &flounder_disp1_fb_data,
	.emc_clk_rate	= 204000000,
#ifdef CONFIG_TEGRA_DC_CMU
	.cmu_enable	= 0,
#endif
};

static struct tegra_fb_data flounder_disp2_fb_data = {
	.win		= 0,
	.xres		= 1280,
	.yres		= 720,
	.bits_per_pixel = 32,
	.flags		= TEGRA_FB_FLIP_ON_PROBE,
};

static struct tegra_dc_platform_data flounder_disp2_pdata = {
	.flags		= TEGRA_DC_FLAG_ENABLED,
	.default_out	= &flounder_disp2_out,
	.fb		= &flounder_disp2_fb_data,
	.emc_clk_rate	= 300000000,
};

static struct platform_device flounder_disp2_device = {
	.name		= "tegradc",
	.id		= 1,
	.resource	= flounder_disp2_resources,
	.num_resources	= ARRAY_SIZE(flounder_disp2_resources),
	.dev = {
		.platform_data = &flounder_disp2_pdata,
	},
};

static struct platform_device flounder_disp1_device = {
	.name		= "tegradc",
	.id		= 0,
	.resource	= flounder_disp1_resources,
	.num_resources	= ARRAY_SIZE(flounder_disp1_resources),
	.dev = {
		.platform_data = &flounder_disp1_pdata,
	},
};

static struct nvmap_platform_carveout flounder_carveouts[] = {
	[0] = {
		.name		= "iram",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_IRAM,
		.base		= TEGRA_IRAM_BASE + TEGRA_RESET_HANDLER_SIZE,
		.size		= TEGRA_IRAM_SIZE - TEGRA_RESET_HANDLER_SIZE,
		.dma_dev	= &tegra_iram_dev,
	},
	[1] = {
		.name		= "generic-0",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_GENERIC,
		.base		= 0, /* Filled in by flounder_panel_init() */
		.size		= 0, /* Filled in by flounder_panel_init() */
		.dma_dev	= &tegra_generic_dev,
	},
	[2] = {
		.name		= "vpr",
		.usage_mask	= NVMAP_HEAP_CARVEOUT_VPR,
		.base		= 0, /* Filled in by flounder_panel_init() */
		.size		= 0, /* Filled in by flounder_panel_init() */
		.dma_dev	= &tegra_vpr_dev,
	},
};

static struct nvmap_platform_data flounder_nvmap_data = {
	.carveouts	= flounder_carveouts,
	.nr_carveouts	= ARRAY_SIZE(flounder_carveouts),
};
static struct platform_device flounder_nvmap_device  = {
	.name	= "tegra-nvmap",
	.id	= -1,
	.dev	= {
		.platform_data = &flounder_nvmap_data,
	},
};

/* can be called multiple times */
static struct tegra_panel *flounder_panel_configure(struct board_info *board_out,
	u8 *dsi_instance_out)
{
	struct tegra_panel *panel = NULL;
	u8 dsi_instance = DSI_INSTANCE_0;
	struct board_info boardtmp;

	if (!board_out)
		board_out = &boardtmp;
	tegra_get_display_board_info(board_out);

	panel = &dsi_j_qxga_8_9;
	dsi_instance = DSI_INSTANCE_0;
	/*tegra_io_dpd_enable(&dsic_io);
	tegra_io_dpd_enable(&dsid_io);*/
	if (board_out->board_id == BOARD_E1813)
		panel = &dsi_s_wqxga_10_1;
	if (dsi_instance_out)
		*dsi_instance_out = dsi_instance;
	return panel;
}

static void flounder_panel_select(void)
{
	struct tegra_panel *panel = NULL;
	u8 dsi_instance;
	struct board_info board;

	panel = flounder_panel_configure(&board, &dsi_instance);

	if (panel) {
		if (panel->init_dc_out) {
			panel->init_dc_out(&flounder_disp1_out);
			if (flounder_disp1_out.type == TEGRA_DC_OUT_DSI) {
				flounder_disp1_out.dsi->dsi_instance =
					dsi_instance;
				flounder_disp1_out.dsi->dsi_panel_rst_gpio =
					DSI_PANEL_RST_GPIO;
				flounder_disp1_out.dsi->dsi_panel_bl_pwm_gpio =
					DSI_PANEL_BL_PWM_GPIO;
				flounder_disp1_out.dsi->te_gpio = TEGRA_GPIO_PR6;
			}
		}

		if (panel->init_fb_data)
			panel->init_fb_data(&flounder_disp1_fb_data);

		if (panel->init_cmu_data)
			panel->init_cmu_data(&flounder_disp1_pdata);

		if (panel->set_disp_device)
			panel->set_disp_device(&flounder_disp1_device);

		if (flounder_disp1_out.type == TEGRA_DC_OUT_DSI) {
			tegra_dsi_resources_init(dsi_instance,
				flounder_disp1_resources,
				ARRAY_SIZE(flounder_disp1_resources));
		}

		if (panel->register_bl_dev)
			panel->register_bl_dev();

		if (panel->register_i2c_bridge)
			panel->register_i2c_bridge();
	}

}

int __init flounder_panel_init(void)
{
	int err = 0;
	struct resource __maybe_unused *res;
	struct platform_device *phost1x = NULL;

#ifdef CONFIG_NVMAP_USE_CMA_FOR_CARVEOUT
	struct dma_declare_info vpr_dma_info;
	struct dma_declare_info generic_dma_info;
#endif
	flounder_panel_select();

#ifdef CONFIG_TEGRA_NVMAP
	flounder_carveouts[1].base = tegra_carveout_start;
	flounder_carveouts[1].size = tegra_carveout_size;
	flounder_carveouts[2].base = tegra_vpr_start;
	flounder_carveouts[2].size = tegra_vpr_size;
#ifdef CONFIG_NVMAP_USE_CMA_FOR_CARVEOUT
	generic_dma_info.name = "generic";
	generic_dma_info.base = tegra_carveout_start;
	generic_dma_info.size = tegra_carveout_size;
	generic_dma_info.resize = false;
	generic_dma_info.cma_dev = NULL;

	vpr_dma_info.name = "vpr";
	vpr_dma_info.base = tegra_vpr_start;
	vpr_dma_info.size = SZ_32M;
	vpr_dma_info.resize = true;
	vpr_dma_info.cma_dev = &tegra_vpr_cma_dev;
	vpr_dma_info.notifier.ops = &vpr_dev_ops;

	carveout_linear_set(&tegra_generic_cma_dev);
	flounder_carveouts[1].cma_dev = &tegra_generic_cma_dev;
	flounder_carveouts[1].resize = false;
	carveout_linear_set(&tegra_vpr_cma_dev);
	flounder_carveouts[2].cma_dev = &tegra_vpr_cma_dev;
	flounder_carveouts[2].resize = true;


	if (tegra_carveout_size) {
		err = dma_declare_coherent_resizable_cma_memory(
				&tegra_generic_dev, &generic_dma_info);
		if (err) {
			pr_err("Generic coherent memory declaration failed\n");
			return err;
		}
	}
	if (tegra_vpr_size) {
		err = dma_declare_coherent_resizable_cma_memory(
				&tegra_vpr_dev, &vpr_dma_info);
		if (err) {
			pr_err("VPR coherent memory declaration failed\n");
			return err;
		}
	}
#endif

	err = platform_device_register(&flounder_nvmap_device);
	if (err) {
		pr_err("nvmap device registration failed\n");
		return err;
	}
#endif

	phost1x = flounder_host1x_init();
	if (!phost1x) {
		pr_err("host1x devices registration failed\n");
		return -EINVAL;
	}

	res = platform_get_resource_byname(&flounder_disp1_device,
					 IORESOURCE_MEM, "fbmem");
	res->start = tegra_fb_start;
	res->end = tegra_fb_start + tegra_fb_size - 1;

	/* Copy the bootloader fb to the fb. */
	if (tegra_bootloader_fb_size)
		__tegra_move_framebuffer(&flounder_nvmap_device,
				tegra_fb_start, tegra_bootloader_fb_start,
				min(tegra_fb_size, tegra_bootloader_fb_size));
	else
		__tegra_clear_framebuffer(&flounder_nvmap_device,
					  tegra_fb_start, tegra_fb_size);

	flounder_disp1_device.dev.parent = &phost1x->dev;
	err = platform_device_register(&flounder_disp1_device);
	if (err) {
		pr_err("disp1 device registration failed\n");
		return err;
	}

	err = tegra_init_hdmi(&flounder_disp2_device, phost1x);
	if (err)
		return err;

#ifdef CONFIG_TEGRA_NVAVP
	nvavp_device.dev.parent = &phost1x->dev;
	err = platform_device_register(&nvavp_device);
	if (err) {
		pr_err("nvavp device registration failed\n");
		return err;
	}
#endif
	return err;
}

int __init flounder_display_init(void)
{
	struct clk *disp1_clk = clk_get_sys("tegradc.0", NULL);
	struct clk *disp2_clk = clk_get_sys("tegradc.1", NULL);
	struct tegra_panel *panel;
	struct board_info board;
	long disp1_rate = 0;
	long disp2_rate;

	if (WARN_ON(IS_ERR(disp1_clk))) {
		if (disp2_clk && !IS_ERR(disp2_clk))
			clk_put(disp2_clk);
		return PTR_ERR(disp1_clk);
	}

	if (WARN_ON(IS_ERR(disp2_clk))) {
		clk_put(disp1_clk);
		return PTR_ERR(disp1_clk);
	}

	panel = flounder_panel_configure(&board, NULL);

	if (panel && panel->init_dc_out) {
		panel->init_dc_out(&flounder_disp1_out);
		if (flounder_disp1_out.n_modes && flounder_disp1_out.modes)
			disp1_rate = flounder_disp1_out.modes[0].pclk;
	} else {
		if (!panel || !panel->init_dc_out)
			printk(KERN_ERR "disp1 panel output not specified!\n");
	}

	printk(KERN_DEBUG "disp1 pclk=%ld\n", disp1_rate);
	if (disp1_rate)
		tegra_dvfs_resolve_override(disp1_clk, disp1_rate);

	/* set up disp2 */
	if (flounder_disp2_out.max_pixclock)
		disp2_rate = PICOS2KHZ(flounder_disp2_out.max_pixclock) * 1000;
	else
		disp2_rate = 297000000; /* HDMI 4K */
	printk(KERN_DEBUG "disp2 pclk=%ld\n", disp2_rate);
	if (disp2_rate)
		tegra_dvfs_resolve_override(disp2_clk, disp2_rate);

	clk_put(disp1_clk);
	clk_put(disp2_clk);
	return 0;
}
