/*
 * arch/arm/mach-tegra/board-p2360.c
 * Based on arch/arm/mach-tegra/board-vcm30_t124.c
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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/platform_device.h>
#include <linux/i2c-tegra.h>
#include <linux/i2c.h>
#include <linux/platform_data/serial-tegra.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/platform_data/tegra_nor.h>
#include <linux/platform_data/tegra_ahci.h>
#include <linux/of_platform.h>
#include <linux/kernel.h>
#include <linux/clocksource.h>
#include <linux/irqchip.h>

#include <mach/tegra_asoc_pdata.h>
#include <mach/io_dpd.h>
#include <asm/mach/arch.h>
#include <mach/isomgr.h>
#include <mach/board_id.h>

#include "iomap.h"
#include "board.h"
#include <linux/platform/tegra/clock.h>
#include "board-p2360.h"
#include "devices.h"
#include "board-common.h"
#include <asm/mach-types.h>
#include <linux/platform/tegra/common.h>

#include "tegra-of-dev-auxdata.h"

static struct board_info board_info, display_board_info;

/*
 * Set clock values as per automotive POR
 * Values taken from VCM30T124 and curtailed for P2360
 */
static __initdata struct tegra_clk_init_table p2360_clk_init_table[] = {
	/* name			parent		rate	enabled (always on)*/

	{ "pll_c",		NULL,		792000000,	true},

	{ "automotive.sclk",	NULL,		316800000,	true},
	{ "automotive.hclk",	NULL,		316800000,	true},
	{ "automotive.pclk",	NULL,		158400000,	true},

	{ "mselect",		"pll_p",	408000000,	true},
	{ "automotive.mselect",	NULL,		408000000,	true},

	{ "se.cbus",		NULL,		432000000,	false},
	{ "msenc.cbus",		NULL,		432000000,	false},
	{ "vde.cbus",		NULL,		432000000,	false},

	{ "vic03.cbus",		NULL,		660000000,	false},
	{ "tsec.cbus",		NULL,		660000000,	false},

	{ "vi.c4bus",		NULL,		600000000,	false},
	{ "isp.c4bus",		NULL,		600000000,	false},

	{ "pll_d_out0",		"pll_d",	474000000,	true},
	{ "disp2",		"pll_d_out0",	474000000,	false},
	{ "disp1",		"pll_d_out0",	474000000,	false},

	{ "pll_d2",		NULL,		297000000,	true},
	{ "hdmi",		"pll_d2",	297000000,	false},

	{ "pll_a_out0",		NULL,		24600000,	true},
	{ "i2s0",		"pll_a_out0",	24600000,	false},
	{ "i2s1",		"pll_a_out0",	24600000,	false},
	{ "i2s2",		"pll_a_out0",	24600000,	false},
	{ "i2s3",		"pll_a_out0",	24600000,	false},
	{ "i2s4",		"pll_a_out0",	24600000,	false},

	{ "dam0",		"pll_p",	19900000,	false},
	{ "dam1",		"pll_p",	19900000,	false},
	{ "dam2",		"pll_p",	19900000,	false},
	{ "adx",		"pll_p",	19900000,	false},
	{ "adx1",		"pll_p",	19900000,	false},
	{ "amx",		"pll_p",	19900000,	false},
	{ "amx1",		"pll_p",	19900000,	false},

	{ "spdif_out",		"pll_a_out0",	24600000,	false},
	{ "hda",		"pll_p",	48000000,	false},
	{ "cilab",		"pll_p",	10200000,	false},
	{ "cilcd",		"pll_p",	10200000,	false},
	{ "cile",		"pll_p",	10200000,	false},

	{ "nor",		"pll_p",	102000000,	false},

	{ "sbc1",		"pll_p",	25000000,	false},
	{ "sbc2",		"pll_p",	25000000,	false},
	{ "sbc3",		"pll_p",	25000000,	false},
	{ "sbc4",		"pll_p",	25000000,	false},
	{ "sbc5",		"pll_p",	25000000,	false},
	{ "sbc6",		"pll_p",	25000000,	false},

	{ "uarta",		"pll_p",	408000000,	false},
	{ "uartb",		"pll_p",	408000000,	false},
	{ "uartc",		"pll_p",	408000000,	false},
	{ "uartd",		"pll_p",	408000000,	false},

	{ "vi_sensor",		"pll_p",	68000000,	false},
	{ "vi_sensor2",		"pll_p",	68000000,	false},

	{ "automotive.host1x",	NULL,		264000000,	true},

	{ NULL,			NULL,		0,		0},
};

#define SET_FIXED_TARGET_RATE(clk_name, fixed_target_rate) \
	{clk_name,	NULL,	fixed_target_rate,	false}

/*
 * FIXME: Need to revisit for following clocks:
 * csi, dsi, dsilp, audio
 */

/*
 * Table of fixed target rates for automotive. parent and enable field are
 * don't care for this table.
 */

static struct tegra_clk_init_table p2360_fixed_target_clk_table[] = {

	/*			name,		fixed target rate*/
	SET_FIXED_TARGET_RATE("pll_m",		792000000),
#ifdef CONFIG_ANDROID
	/* [WAR] : bug 1440706
		There are lots of WARN_ON messages during Video playback.
		Those make some jerky Video playback issues.
		Temporarily changed the expected rate from 316800000
		to 12000000 but it should be fixed correctly.
	*/
	SET_FIXED_TARGET_RATE("sbus",		 12000000),
#else
	SET_FIXED_TARGET_RATE("sbus",		316800000),
#endif

#ifdef CONFIG_TEGRA_DUAL_CBUS
	SET_FIXED_TARGET_RATE("pll_c2",		432000000),
	SET_FIXED_TARGET_RATE("c2bus",		432000000),
	SET_FIXED_TARGET_RATE("pll_c3",		660000000),
	SET_FIXED_TARGET_RATE("c3bus",		660000000),
#endif
	SET_FIXED_TARGET_RATE("pll_c",		792000000),
	SET_FIXED_TARGET_RATE("pll_c4",		600000000),
	SET_FIXED_TARGET_RATE("c4bus",		600000000),
	SET_FIXED_TARGET_RATE("pll_c_out1",	316800000),
	SET_FIXED_TARGET_RATE("pll_p",		408000000),
	SET_FIXED_TARGET_RATE("pll_x",		150000000),
	SET_FIXED_TARGET_RATE("gbus",		600000000),

	SET_FIXED_TARGET_RATE("gk20a.gbus",	600000000),
	SET_FIXED_TARGET_RATE("sclk",		316800000),
	SET_FIXED_TARGET_RATE("hclk",		316800000),
	SET_FIXED_TARGET_RATE("ahb.sclk",	316800000),
	SET_FIXED_TARGET_RATE("pclk",		158400000),
	SET_FIXED_TARGET_RATE("apb.sclk",	158400000),

	SET_FIXED_TARGET_RATE("cpu_g",		150000000),
	SET_FIXED_TARGET_RATE("cpu_lp",		109200000),

	SET_FIXED_TARGET_RATE("vde",		432000000),
	SET_FIXED_TARGET_RATE("se",		432000000),
	SET_FIXED_TARGET_RATE("msenc",		432000000),

	SET_FIXED_TARGET_RATE("tsec",		660000000),
	SET_FIXED_TARGET_RATE("vic03",		660000000),

	SET_FIXED_TARGET_RATE("vi",		600000000),
	SET_FIXED_TARGET_RATE("isp",		600000000),

	SET_FIXED_TARGET_RATE("host1x",		264000000),
	SET_FIXED_TARGET_RATE("mselect",	408000000),

	SET_FIXED_TARGET_RATE("dam0",		40000000),
	SET_FIXED_TARGET_RATE("dam1",		40000000),
	SET_FIXED_TARGET_RATE("dam2",		40000000),

	SET_FIXED_TARGET_RATE("adx",		24600000),
	SET_FIXED_TARGET_RATE("adx1",		24600000),
	SET_FIXED_TARGET_RATE("amx",		24600000),
	SET_FIXED_TARGET_RATE("amx1",		24600000),

	SET_FIXED_TARGET_RATE("spdif_out",	24576000),
	SET_FIXED_TARGET_RATE("hda",		108000000),

	SET_FIXED_TARGET_RATE("cilab",		102000000),
	SET_FIXED_TARGET_RATE("cilcd",		102000000),
	SET_FIXED_TARGET_RATE("cile",		102000000),

	SET_FIXED_TARGET_RATE("uarta",		408000000),
	SET_FIXED_TARGET_RATE("uartb",		408000000),
	SET_FIXED_TARGET_RATE("uartc",		408000000),
#ifndef CONFIG_ANDROID
	SET_FIXED_TARGET_RATE("uartd",		408000000),
#endif
};

static int __init tegra_fixed_target_rate_init(void)
{
	struct clk *c;
	unsigned long flags;
	int i, is_e1860_b00 = 0;

	is_e1860_b00 = tegra_is_board(NULL, "61860", NULL, "300", NULL);

	/* Set POR value for all clocks given in the table */
	for (i = 0; i < ARRAY_SIZE(p2360_fixed_target_clk_table); i++) {

		c = tegra_get_clock_by_name(
				p2360_fixed_target_clk_table[i].name);
		if (c) {
			clk_lock_save(c, &flags);
			c->fixed_target_rate =
				p2360_fixed_target_clk_table[i].rate;
			clk_unlock_restore(c, &flags);
		} else {
			pr_warn("%s: Clock %s not found\n", __func__,
					p2360_fixed_target_clk_table[i].name);
		}
	}

	return 0;
}
late_initcall_sync(tegra_fixed_target_rate_init);

static struct tegra_nor_platform_data p2360_nor_data = {
	.flash = {
		.map_name = "cfi_probe",
		.width = 2,
	},
	.chip_parms = {
		.MuxMode = NorMuxMode_ADNonMux,
		.ReadMode = NorReadMode_Page,
		.PageLength = NorPageLength_8Word,
		.ReadyActive = NorReadyActive_WithData,
		/* FIXME: Need to use characterized value */
		.timing_default = {
			.timing0 = 0x30300273,
			.timing1 = 0x00030302,
		},
		.timing_read = {
			.timing0 = 0x30300273,
			.timing1 = 0x00030302,
		},
	},
};

static struct cs_info p2360_cs_info[] = {
	{
		.cs = CS_0,
		.num_cs_gpio = 0,
		.virt = IO_ADDRESS(TEGRA_NOR_FLASH_BASE),
		.size = SZ_64M,
		.phys = TEGRA_NOR_FLASH_BASE,
	},
};

static void p2360_nor_init(void)
{
	tegra_nor_device.resource[2].end = TEGRA_NOR_FLASH_BASE + SZ_64M - 1;

	p2360_nor_data.info.cs = kzalloc(sizeof(struct cs_info) *
					ARRAY_SIZE(p2360_cs_info),
					GFP_KERNEL);
	if (!p2360_nor_data.info.cs)
		BUG();

	p2360_nor_data.info.num_chips = ARRAY_SIZE(p2360_cs_info);

	memcpy(p2360_nor_data.info.cs, p2360_cs_info,
			sizeof(struct cs_info) * ARRAY_SIZE(p2360_cs_info));

	tegra_nor_device.dev.platform_data = &p2360_nor_data;
	platform_device_register(&tegra_nor_device);
}


/* Register debug UART in old fashion and use DT for all others */
static struct tegra_serial_platform_data p2360_uartb_pdata = {
	.dma_req_selector = 9,
	.modem_interrupt = false,
};

static void __init p2360_uart_init(void)
{
	int debug_port_id;

	if (!is_tegra_debug_uartport_hs()) {
		debug_port_id = uart_console_debug_init(1);
		if (debug_port_id < 0)
			return;

		platform_device_register(uart_console_debug_device);
	} else {
		tegra_uartb_device.dev.platform_data = &p2360_uartb_pdata;
		platform_device_register(&tegra_uartb_device);
	}
}

/* FIXME: Check which devices are needed from the below list */
static struct platform_device *p2360_devices[] __initdata = {
	&tegra_pmu_device,
#if defined(CONFIG_TEGRA_AVP)
	&tegra_avp_device,
#endif
#if defined(CONFIG_CRYPTO_DEV_TEGRA_SE)
	&tegra12_se_device,
#endif
};

static struct tegra_usb_platform_data tegra_udc_pdata = {
#if defined(CONFIG_USB_TEGRA_OTG)
	.port_otg = true,
#else
	.port_otg = false,
#endif
	.has_hostpc = true,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_DEVICE,
	.u_data.dev = {
		.vbus_pmu_irq = 0,
		.vbus_gpio = -1,
		.charging_supported = false,
		.remote_wakeup_supported = false,
	},
	.u_cfg.utmi = {
		.hssync_start_delay = 0,
		.idle_wait_delay = 17,
		.elastic_limit = 16,
		.term_range_adj = 6,
		.xcvr_setup = 63,
		.xcvr_setup_offset = 6,
		.xcvr_use_fuses = 1,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
		.xcvr_use_lsb = 1,
	},
};

static struct tegra_usb_platform_data tegra_ehci1_utmi_pdata = {
#if defined(CONFIG_USB_TEGRA_OTG)
	.port_otg = true,
	.id_det_type = TEGRA_USB_VIRTUAL_ID,
#else
	.port_otg = false,
#endif
	.has_hostpc = true,
	.unaligned_dma_buf_supported = true,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
		.hot_plug = false,
		.remote_wakeup_supported = false,
		.power_off_on_suspend = false,
		.turn_off_vbus_on_lp0 = true,
	},
	.u_cfg.utmi = {
		.hssync_start_delay = 0,
		.elastic_limit = 16,
		.idle_wait_delay = 17,
		.term_range_adj = 6,
		.xcvr_setup = 15,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
		.xcvr_setup_offset = 0,
		.xcvr_use_fuses = 1,
		.vbus_oc_map = 0x4,
	},
};


static struct tegra_usb_platform_data tegra_ehci2_utmi_pdata = {
	.port_otg = false,
	.has_hostpc = true,
	.unaligned_dma_buf_supported = true,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
		.hot_plug = true,
		.remote_wakeup_supported = false,
		.power_off_on_suspend = false,
		.turn_off_vbus_on_lp0 = true,
	},
	.u_cfg.utmi = {
		.hssync_start_delay = 0,
		.elastic_limit = 16,
		.idle_wait_delay = 17,
		.term_range_adj = 6,
		.xcvr_setup = 8,
		.xcvr_lsfslew = 2,
		.xcvr_lsrslew = 2,
		.xcvr_setup_offset = 0,
		.xcvr_use_fuses = 1,
		.vbus_oc_map = 0x5,
	},
};

static struct tegra_usb_otg_data tegra_otg_pdata = {
	.ehci_device = &tegra_ehci1_device,
	.ehci_pdata = &tegra_ehci1_utmi_pdata,
};

static void p2360_usb_init(void)
{
	int usb_port_owner_info = tegra_get_usb_port_owner_info();

	if (!(usb_port_owner_info & UTMI1_PORT_OWNER_XUSB)) {
		tegra_otg_pdata.is_xhci = false;
		tegra_udc_pdata.u_data.dev.is_xhci = false;

#if defined(CONFIG_USB_TEGRA_OTG)
		/* register OTG device */
		tegra_otg_device.dev.platform_data = &tegra_otg_pdata;
		platform_device_register(&tegra_otg_device);

		/* Setup the udc platform data */
		tegra_udc_device.dev.platform_data = &tegra_udc_pdata;
		platform_device_register(&tegra_udc_device);
#else
		/* register host mode */
		tegra_ehci1_device.dev.platform_data = &tegra_ehci1_utmi_pdata;
		platform_device_register(&tegra_ehci1_device);
#endif
	}

	if (!(usb_port_owner_info & UTMI2_PORT_OWNER_XUSB)) {
		tegra_ehci2_device.dev.platform_data = &tegra_ehci2_utmi_pdata;
		platform_device_register(&tegra_ehci2_device);
	}
}

#ifdef CONFIG_USE_OF
struct of_dev_auxdata p2360_auxdata_lookup[] __initdata = {
	OF_DEV_AUXDATA("nvidia,tegra114-hsuart", TEGRA_UARTB_BASE,
				"serial-tegra.1", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-host1x", TEGRA_HOST1X_BASE, "host1x",
		NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-gk20a", TEGRA_GK20A_BAR0_BASE, "gk20a.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-vic", TEGRA_VIC_BASE, "vic03.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-msenc", TEGRA_MSENC_BASE, "msenc",
		NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-vi", TEGRA_VI_BASE, "vi.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-isp", TEGRA_ISP_BASE, "isp.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-isp", TEGRA_ISPB_BASE, "isp.1", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-tsec", TEGRA_TSEC_BASE, "tsec", NULL),
	T124_I2C_OF_DEV_AUXDATA,
	OF_DEV_AUXDATA("nvidia,tegra114-nvavp", 0x60001000, "nvavp",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-efuse", TEGRA_FUSE_BASE, "tegra-fuse",
				NULL),
	{}
};
#endif

static void __init tegra_p2360_early_init(void)
{
	tegra_clk_init_from_table(p2360_clk_init_table);
	tegra_clk_verify_parents();
	tegra_soc_device_init("p2360");
}

static void __init tegra_p2360_late_init(void)
{
	struct board_info board_info;
	tegra_get_board_info(&board_info);
	pr_info("board_info: id:sku:fab:major:minor = 0x%04x:0x%04x:0x%02x:0x%02x:0x%02x\n",
		board_info.board_id, board_info.sku,
		board_info.fab, board_info.major_revision,
		board_info.minor_revision);
	p2360_usb_init();

	p2360_nor_init();

	p2360_uart_init();
	platform_add_devices(p2360_devices, ARRAY_SIZE(p2360_devices));
	tegra_io_dpd_init();
	p2360_sdhci_init();

	isomgr_init();

	p2360_panel_init();
}

static void __init tegra_p2360_dt_init(void)
{
	tegra_get_board_info(&board_info);
	tegra_get_display_board_info(&display_board_info);

	tegra_p2360_early_init();
#ifdef CONFIG_NVMAP_USE_CMA_FOR_CARVEOUT
	carveout_linear_set(&tegra_generic_cma_dev);
	carveout_linear_set(&tegra_vpr_cma_dev);
#endif
#ifdef CONFIG_USE_OF
	of_platform_populate(NULL,
		of_default_bus_match_table, p2360_auxdata_lookup,
		&platform_bus);
#endif

	tegra_p2360_late_init();
}

static void __init tegra_p2360_reserve(void)
{
#if defined(CONFIG_NVMAP_CONVERT_CARVEOUT_TO_IOVMM)
	/* 1920*1200*4*2 = 18432000 bytes */
	tegra_reserve4(0, SZ_16M + SZ_2M, SZ_16M + SZ_2M, 64 * SZ_1M);
#else
	tegra_reserve4(SZ_128M, SZ_16M + SZ_2M, SZ_16M + SZ_2M, 64 * SZ_1M);
#endif
}

static const char * const p2360_dt_board_compat[] = {
	"nvidia,p2360",
	NULL
};

DT_MACHINE_START(P2360, "p2360")
	.atag_offset	= 0x100,
	.smp		= smp_ops(tegra_smp_ops),
	.map_io		= tegra_map_common_io,
	.reserve	= tegra_p2360_reserve,
	.init_early	= tegra12x_init_early,
	.init_irq	= irqchip_init,
	.init_time	= clocksource_of_init,
	.init_machine	= tegra_p2360_dt_init,
	.restart	= tegra_assert_system_reset,
	.dt_compat	= p2360_dt_board_compat,
	.init_late	= tegra_init_late
MACHINE_END
