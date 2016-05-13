/*
 * arch/arm/mach-tegra/board-ardbeg.c
 *
 * Copyright (c) 2013-2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
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
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/serial_8250.h>
#include <linux/i2c.h>
#include <linux/i2c/i2c-hid.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/i2c-tegra.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/spi/spi.h>
#include <linux/spi/rm31080a_ts.h>
#include <linux/maxim_sti.h>
#include <linux/memblock.h>
#include <linux/spi/spi-tegra.h>
#include <linux/nfc/pn544.h>
#include <linux/rfkill-gpio.h>
#include <linux/skbuff.h>
#include <linux/ti_wilink_st.h>
#include <linux/regulator/consumer.h>
#include <linux/smb349-charger.h>
#include <linux/max17048_battery.h>
#include <linux/leds.h>
#include <linux/i2c/at24.h>
#include <linux/of_platform.h>
#include <linux/i2c.h>
#include <linux/i2c-tegra.h>
#include <linux/platform_data/serial-tegra.h>
#include <linux/edp.h>
#include <linux/usb/tegra_usb_phy.h>
#include <linux/mfd/palmas.h>
#include <linux/clk/tegra.h>
#include <media/tegra_dtv.h>
#include <linux/clocksource.h>
#include <linux/irqchip.h>
#include <linux/irqchip/tegra.h>
#include <linux/tegra-soc.h>
#include <linux/tegra_fiq_debugger.h>
#include <linux/platform_data/tegra_usb_modem_power.h>
#include <linux/platform_data/tegra_ahci.h>
#include <linux/irqchip/tegra.h>
#include <sound/max98090.h>

#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/pinmux-t12.h>
#include <mach/io_dpd.h>
#include <mach/i2s.h>
#include <mach/isomgr.h>
#include <mach/tegra_asoc_pdata.h>
#include <mach/dc.h>
#include <mach/tegra_usb_pad_ctrl.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/gpio-tegra.h>
#include <mach/xusb.h>

#include "board.h"
#include "board-ardbeg.h"
#include "board-common.h"
#include "board-touch-raydium.h"
#include "board-touch-maxim_sti.h"
#include <linux/platform/tegra/clock.h>
#include <linux/platform/tegra/common.h>
#include "devices.h"
#include "gpio-names.h"
#include "iomap.h"
#include "pm.h"
#include "tegra-board-id.h"
#include "tegra-of-dev-auxdata.h"

static struct board_info board_info, display_board_info;

static struct resource ardbeg_bluedroid_pm_resources[] = {
	[0] = {
		.name   = "shutdown_gpio",
		.start  = TEGRA_GPIO_PR1,
		.end    = TEGRA_GPIO_PR1,
		.flags  = IORESOURCE_IO,
	},
	[1] = {
		.name = "host_wake",
		.flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	},
	[2] = {
		.name = "gpio_ext_wake",
		.start  = TEGRA_GPIO_PEE1,
		.end    = TEGRA_GPIO_PEE1,
		.flags  = IORESOURCE_IO,
	},
	[3] = {
		.name = "gpio_host_wake",
		.start  = TEGRA_GPIO_PU6,
		.end    = TEGRA_GPIO_PU6,
		.flags  = IORESOURCE_IO,
	},
	[4] = {
		.name = "reset_gpio",
		.start  = TEGRA_GPIO_PX1,
		.end    = TEGRA_GPIO_PX1,
		.flags  = IORESOURCE_IO,
	},
};

static struct platform_device ardbeg_bluedroid_pm_device = {
	.name = "bluedroid_pm",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(ardbeg_bluedroid_pm_resources),
	.resource       = ardbeg_bluedroid_pm_resources,
};

static noinline void __init ardbeg_setup_bluedroid_pm(void)
{
	ardbeg_bluedroid_pm_resources[1].start =
		ardbeg_bluedroid_pm_resources[1].end =
				gpio_to_irq(TEGRA_GPIO_PU6);
	platform_device_register(&ardbeg_bluedroid_pm_device);
}

static struct i2c_board_info __initdata rt5639_board_info = {
	I2C_BOARD_INFO("rt5639", 0x1c),
};

static struct max98090_eq_cfg max98090_eq_cfg[] = {
};

static struct max98090_pdata norrin_max98090_pdata = {
	/* Equalizer Configuration */
	.eq_cfg = max98090_eq_cfg,
	.eq_cfgcnt = ARRAY_SIZE(max98090_eq_cfg),

	/* Microphone Configuration */
	.digmic_left_mode = 1,
	.digmic_right_mode = 1,
};

static struct i2c_board_info __initdata max98090_board_info = {
	I2C_BOARD_INFO("max98090", 0x10),
	.platform_data	= &norrin_max98090_pdata,
};

static __initdata struct tegra_clk_init_table ardbeg_clk_init_table[] = {
	/* name		parent		rate		enabled */
	{ "pll_m",	NULL,		0,		false},
	{ "hda",	"pll_p",	108000000,	false},
	{ "hda2codec_2x", "pll_p",	48000000,	false},
	{ "pwm",	"pll_p",	48000000,	false},
	{ "i2s1",	"pll_a_out0",	0,		false},
	{ "i2s3",	"pll_a_out0",	0,		false},
	{ "i2s4",	"pll_a_out0",	0,		false},
	{ "spdif_out",	"pll_a_out0",	0,		false},
	{ "d_audio",	"pll_a_out0",	12288000,	false},
	{ "dam0",	"clk_m",	12000000,	false},
	{ "dam1",	"clk_m",	12000000,	false},
	{ "dam2",	"clk_m",	12000000,	false},
	{ "audio1",	"i2s1_sync",	0,		false},
	{ "audio3",	"i2s3_sync",	0,		false},
	{ "vi_sensor",	"pll_p",	150000000,	false},
	{ "vi_sensor2",	"pll_p",	150000000,	false},
	{ "cilab",	"pll_p",	150000000,	false},
	{ "cilcd",	"pll_p",	150000000,	false},
	{ "cile",	"pll_p",	150000000,	false},
	{ "i2c1",	"pll_p",	3200000,	false},
	{ "i2c2",	"pll_p",	3200000,	false},
	{ "i2c3",	"pll_p",	3200000,	false},
	{ "i2c4",	"pll_p",	3200000,	false},
	{ "i2c5",	"pll_p",	3200000,	false},
	{ "sbc1",	"pll_p",	25000000,	false},
	{ "sbc2",	"pll_p",	25000000,	false},
	{ "sbc3",	"pll_p",	25000000,	false},
	{ "sbc4",	"pll_p",	25000000,	false},
	{ "sbc5",	"pll_p",	25000000,	false},
	{ "sbc6",	"pll_p",	25000000,	false},
	{ "uarta",	"pll_p",	408000000,	false},
	{ "uartb",	"pll_p",	408000000,	false},
	{ "uartc",	"pll_p",	408000000,	false},
	{ "uartd",	"pll_p",	408000000,	false},
	{ NULL,		NULL,		0,		0},
};

static struct i2c_hid_platform_data i2c_keyboard_pdata = {
	.hid_descriptor_address = 0x0,
};

static struct i2c_board_info __initdata i2c_keyboard_board_info = {
	I2C_BOARD_INFO("hid", 0x3B),
	.platform_data  = &i2c_keyboard_pdata,
};

static struct i2c_hid_platform_data i2c_touchpad_pdata = {
	.hid_descriptor_address = 0x20,
};

static struct i2c_board_info __initdata i2c_touchpad_board_info = {
	I2C_BOARD_INFO("hid", 0x2C),
	.platform_data  = &i2c_touchpad_pdata,
};

static void ardbeg_i2c_init(void)
{
	struct board_info board_info;
	tegra_get_board_info(&board_info);

	if (board_info.board_id == BOARD_PM374) {
		i2c_register_board_info(0, &max98090_board_info, 1);
	} else if (board_info.board_id != BOARD_PM359)
		i2c_register_board_info(0, &rt5639_board_info, 1);

	if (board_info.board_id == BOARD_PM359 ||
		board_info.board_id == BOARD_PM358 ||
		board_info.board_id == BOARD_PM363 ||
		board_info.board_id == BOARD_PM374) {
		i2c_keyboard_board_info.irq = gpio_to_irq(I2C_KB_IRQ);
		i2c_register_board_info(1, &i2c_keyboard_board_info , 1);

		i2c_touchpad_board_info.irq = gpio_to_irq(I2C_TP_IRQ);
		i2c_register_board_info(1, &i2c_touchpad_board_info , 1);
	}
}

#ifndef CONFIG_USE_OF
static struct platform_device *ardbeg_uart_devices[] __initdata = {
	&tegra_uarta_device,
	&tegra_uartb_device,
	&tegra_uartc_device,
};

static struct tegra_serial_platform_data ardbeg_uarta_pdata = {
	.dma_req_selector = 8,
	.modem_interrupt = false,
};

static struct tegra_serial_platform_data ardbeg_uartb_pdata = {
	.dma_req_selector = 9,
	.modem_interrupt = false,
};

static struct tegra_serial_platform_data ardbeg_uartc_pdata = {
	.dma_req_selector = 10,
	.modem_interrupt = false,
};
#endif

static struct tegra_serial_platform_data ardbeg_uartd_pdata = {
	.dma_req_selector = 19,
	.modem_interrupt = false,
};

static struct tegra_asoc_platform_data ardbeg_audio_pdata_rt5639 = {
	.gpio_hp_det = TEGRA_GPIO_HP_DET,
	.gpio_ldo1_en = TEGRA_GPIO_LDO_EN,
	.gpio_spkr_en = -1,
	.gpio_int_mic_en = -1,
	.gpio_ext_mic_en = -1,
	.gpio_hp_mute = -1,
	.gpio_codec1 = -1,
	.gpio_codec2 = -1,
	.gpio_codec3 = -1,
	.i2s_param[HIFI_CODEC]       = {
		.audio_port_id = 1,
		.is_i2s_master = 1,
		.i2s_mode = TEGRA_DAIFMT_I2S,
		.sample_size	= 16,
		.channels       = 2,
		.bit_clk	= 1536000,
	},
	.i2s_param[BT_SCO] = {
		.audio_port_id = 3,
		.is_i2s_master = 1,
		.i2s_mode = TEGRA_DAIFMT_DSP_A,
	},
	.i2s_param[BASEBAND]	= {
		.audio_port_id	= 0,
		.is_i2s_master	= 1,
		.i2s_mode	= TEGRA_DAIFMT_I2S,
		.sample_size	= 16,
		.rate		= 16000,
		.channels	= 2,
		.bit_clk	= 1024000,
	},
};

static struct tegra_asoc_platform_data norrin_audio_pdata_max98090 = {
	.gpio_hp_det		= NORRIN_GPIO_HP_DET,
	.gpio_ext_mic_en	= TEGRA_GPIO_HP_DET,
	.gpio_hp_mute		= -1,
	.edp_support		= true,
	.edp_states		= {1080, 842, 0},
	.i2s_param[HIFI_CODEC]	= {
		.audio_port_id	= 1,
		.is_i2s_master	= 1,
		.i2s_mode	= TEGRA_DAIFMT_I2S,
		.sample_size	= 16,
		.channels	= 2,
		.bit_clk	= 1536000,
	},
	.i2s_param[BT_SCO]	= {
		.audio_port_id	= 3,
		.is_i2s_master	= 1,
		.i2s_mode	= TEGRA_DAIFMT_DSP_A,
		.sample_size	= 16,
		.channels	= 1,
		.bit_clk	= 512000,
	},
};

static void ardbeg_audio_init(void)
{
	struct board_info board_info;
	tegra_get_board_info(&board_info);
	if (board_info.board_id == BOARD_PM359 ||
			board_info.board_id == BOARD_PM358 ||
			board_info.board_id == BOARD_PM370 ||
			board_info.board_id == BOARD_PM374 ||
			board_info.board_id == BOARD_PM375 ||
			board_info.board_id == BOARD_PM363) {
		/*Laguna*/
		ardbeg_audio_pdata_rt5639.gpio_hp_det = TEGRA_GPIO_HP_DET;
		ardbeg_audio_pdata_rt5639.gpio_hp_det_active_high = 1;
		if (board_info.board_id != BOARD_PM363)
			ardbeg_audio_pdata_rt5639.gpio_ldo1_en = -1;
	} else {
		/*Ardbeg*/

		if (board_info.board_id == BOARD_E1762 ||
			board_info.board_id == BOARD_P1761 ||
			board_info.board_id == BOARD_E1922) {
			ardbeg_audio_pdata_rt5639.gpio_hp_det =
				TEGRA_GPIO_CDC_IRQ;
			ardbeg_audio_pdata_rt5639.use_codec_jd_irq = true;
		} else {
			ardbeg_audio_pdata_rt5639.gpio_hp_det =
				TEGRA_GPIO_HP_DET;
			ardbeg_audio_pdata_rt5639.use_codec_jd_irq = false;
		}
		ardbeg_audio_pdata_rt5639.gpio_hp_det_active_high = 0;
		ardbeg_audio_pdata_rt5639.gpio_ldo1_en = TEGRA_GPIO_LDO_EN;
	}

	if (board_info.board_id == BOARD_E1971) {
		ardbeg_audio_pdata_rt5639.gpio_hp_det = TEGRA_GPIO_CDC_IRQ;
		ardbeg_audio_pdata_rt5639.use_codec_jd_irq = true;
		ardbeg_audio_pdata_rt5639.gpio_hp_det_active_high = 0;
		ardbeg_audio_pdata_rt5639.gpio_ldo1_en = TEGRA_GPIO_LDO_EN;
	}

	ardbeg_audio_pdata_rt5639.codec_name = "rt5639.0-001c";
	ardbeg_audio_pdata_rt5639.codec_dai_name = "rt5639-aif1";

	norrin_audio_pdata_max98090.codec_name = "max98090.0-0010";
	norrin_audio_pdata_max98090.codec_dai_name = "HiFi";
}

static struct platform_device ardbeg_audio_device_rt5639 = {
	.name = "tegra-snd-rt5639",
	.id = 0,
	.dev = {
		.platform_data = &ardbeg_audio_pdata_rt5639,
	},
};

static struct platform_device norrin_audio_device_max98090 = {
	.name	= "tegra-snd-max98090",
	.id	= 0,
	.dev	= {
		.platform_data = &norrin_audio_pdata_max98090,
	},
};

static void __init ardbeg_uart_init(void)
{

#ifndef CONFIG_USE_OF
	tegra_uarta_device.dev.platform_data = &ardbeg_uarta_pdata;
	tegra_uartb_device.dev.platform_data = &ardbeg_uartb_pdata;
	tegra_uartc_device.dev.platform_data = &ardbeg_uartc_pdata;
	platform_add_devices(ardbeg_uart_devices,
			ARRAY_SIZE(ardbeg_uart_devices));
#endif
	tegra_uartd_device.dev.platform_data = &ardbeg_uartd_pdata;
	if (!is_tegra_debug_uartport_hs()) {
		int debug_port_id = uart_console_debug_init(3);
		if (debug_port_id < 0)
			return;

#ifdef CONFIG_TEGRA_FIQ_DEBUGGER
		tegra_serial_debug_init_irq_mode(TEGRA_UARTD_BASE, INT_UARTD, NULL, -1, -1);
#else
		platform_device_register(uart_console_debug_device);
#endif
	} else {
		tegra_uartd_device.dev.platform_data = &ardbeg_uartd_pdata;
		platform_device_register(&tegra_uartd_device);
	}
}

static struct resource tegra_rtc_resources[] = {
	[0] = {
		.start = TEGRA_RTC_BASE,
		.end = TEGRA_RTC_BASE + TEGRA_RTC_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = INT_RTC,
		.end = INT_RTC,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device tegra_rtc_device = {
	.name = "tegra_rtc",
	.id   = -1,
	.resource = tegra_rtc_resources,
	.num_resources = ARRAY_SIZE(tegra_rtc_resources),
};

static struct platform_device *ardbeg_devices[] __initdata = {
	&tegra_pmu_device,
	&tegra_rtc_device,
#if defined(CONFIG_TEGRA_WAKEUP_MONITOR)
	&tegratab_tegra_wakeup_monitor_device,
#endif
	&tegra_udc_device,
#if defined(CONFIG_TEGRA_AVP)
	&tegra_avp_device,
#endif
#if defined(CONFIG_CRYPTO_DEV_TEGRA_SE) && !defined(CONFIG_USE_OF)
	&tegra12_se_device,
#endif
	&tegra_ahub_device,
	&tegra_dam_device0,
	&tegra_dam_device1,
	&tegra_dam_device2,
	&tegra_i2s_device0,
	&tegra_i2s_device1,
	&tegra_i2s_device3,
	&tegra_i2s_device4,
	&tegra_spdif_device,
	&spdif_dit_device,
	&bluetooth_dit_device,
	&baseband_dit_device,
	&tegra_hda_device,
	&tegra_offload_device,
	&tegra30_avp_audio_device,
#if defined(CONFIG_CRYPTO_DEV_TEGRA_AES)
	&tegra_aes_device,
#endif
	&tegra_hier_ictlr_device,
};

static struct tegra_usb_platform_data tegra_udc_pdata = {
	.port_otg = true,
	.has_hostpc = true,
	.unaligned_dma_buf_supported = false,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_DEVICE,
	.u_data.dev = {
		.vbus_pmu_irq = 0,
		.vbus_gpio = -1,
		.charging_supported = true,
		.remote_wakeup_supported = false,
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
	},
};

static struct tegra_usb_platform_data tegra_ehci1_utmi_pdata = {
	.port_otg = true,
	.has_hostpc = true,
	.unaligned_dma_buf_supported = true,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
		.hot_plug = false,
		.remote_wakeup_supported = true,
		.power_off_on_suspend = true,
	},
	.u_cfg.utmi = {
		.hssync_start_delay = 0,
		.elastic_limit = 16,
		.idle_wait_delay = 17,
		.term_range_adj = 6,
		.xcvr_setup = 15,
		.xcvr_lsfslew = 0,
		.xcvr_lsrslew = 3,
		.xcvr_setup_offset = 0,
		.xcvr_use_fuses = 1,
		.vbus_oc_map = 0x4,
		.xcvr_hsslew_lsb = 2,
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
		.hot_plug = false,
		.remote_wakeup_supported = true,
		.power_off_on_suspend = true,
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

static struct tegra_usb_platform_data tegra_ehci3_utmi_pdata = {
	.port_otg = false,
	.has_hostpc = true,
	.unaligned_dma_buf_supported = true,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
		.hot_plug = false,
		.remote_wakeup_supported = true,
		.power_off_on_suspend = true,
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

static struct gpio modem_gpios[] = { /* Bruce modem */
	{MODEM_EN, GPIOF_OUT_INIT_HIGH, "MODEM EN"},
	{MDM_RST, GPIOF_OUT_INIT_LOW, "MODEM RESET"},
	{MDM_SAR0, GPIOF_OUT_INIT_LOW, "MODEM SAR0"},
};

static struct tegra_usb_platform_data tegra_ehci2_hsic_baseband_pdata = {
	.port_otg = false,
	.has_hostpc = true,
	.unaligned_dma_buf_supported = true,
	.phy_intf = TEGRA_USB_PHY_INTF_HSIC,
	.op_mode = TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
		.hot_plug = false,
		.remote_wakeup_supported = true,
		.power_off_on_suspend = true,
	},
};

static struct tegra_usb_platform_data tegra_ehci2_hsic_smsc_hub_pdata = {
	.port_otg = false,
	.has_hostpc = true,
	.unaligned_dma_buf_supported = true,
	.phy_intf = TEGRA_USB_PHY_INTF_HSIC,
	.op_mode	= TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
		.hot_plug = false,
		.remote_wakeup_supported = true,
		.power_off_on_suspend = true,
	},
};


static struct tegra_usb_otg_data tegra_otg_pdata = {
	.ehci_device = &tegra_ehci1_device,
	.ehci_pdata = &tegra_ehci1_utmi_pdata,
};

static void ardbeg_usb_init(void)
{
	int usb_port_owner_info = tegra_get_usb_port_owner_info();
	int modem_id = tegra_get_modem_id();
	struct board_info bi;
	tegra_get_pmu_board_info(&bi);

	if (board_info.sku == 1100 || board_info.board_id == BOARD_P1761 ||
					board_info.board_id == BOARD_E1784)
		tegra_ehci1_utmi_pdata.u_data.host.turn_off_vbus_on_lp0 = true;

	if (board_info.board_id == BOARD_PM359 ||
			board_info.board_id == BOARD_PM358 ||
			board_info.board_id == BOARD_PM370 ||
			board_info.board_id == BOARD_PM374 ||
			board_info.board_id == BOARD_PM375 ||
			board_info.board_id == BOARD_PM363) {
		/* Laguna */
		/* Host cable is detected through AMS PMU Interrupt */
		if (board_info.major_revision >= 'A' &&
			board_info.major_revision <= 'D' &&
			board_info.board_id == BOARD_PM375) {
			tegra_udc_pdata.id_det_type = TEGRA_USB_VIRTUAL_ID;
			tegra_ehci1_utmi_pdata.id_det_type =
						TEGRA_USB_VIRTUAL_ID;
		} else {
			tegra_udc_pdata.id_det_type = TEGRA_USB_PMU_ID;
			tegra_ehci1_utmi_pdata.id_det_type = TEGRA_USB_PMU_ID;
		}
		tegra_ehci1_utmi_pdata.id_extcon_dev_name = "as3722-extcon";
	} else {
		/* Ardbeg and TN8 */

		/*
		 * TN8 supports vbus changing and it can handle
		 * vbus voltages larger then 5V.  Enable this.
		 */
		if (board_info.board_id == BOARD_P1761 ||
			board_info.board_id == BOARD_E1784 ||
			board_info.board_id == BOARD_E1780) {

			/*
			 * Set the maximum voltage that can be supplied
			 * over USB vbus that the board supports if we use
			 * a quick charge 2 wall charger.
			 */
			tegra_udc_pdata.qc2_voltage = TEGRA_USB_QC2_9V;
			tegra_udc_pdata.u_data.dev.qc2_current_limit_ma = 1200;

			/* charger needs to be set to 2A - h/w will do 1.8A */
			tegra_udc_pdata.u_data.dev.dcp_current_limit_ma = 2000;
		}

		switch (bi.board_id) {
		case BOARD_E1733:
			/* Host cable is detected through PMU Interrupt */
			tegra_udc_pdata.id_det_type = TEGRA_USB_PMU_ID;
			tegra_ehci1_utmi_pdata.id_det_type = TEGRA_USB_PMU_ID;
			tegra_ehci1_utmi_pdata.id_extcon_dev_name =
							 "as3722-extcon";
			break;
		case BOARD_E1736:
		case BOARD_E1769:
		case BOARD_E1735:
		case BOARD_E1936:
		case BOARD_P1761:
			/* Device cable is detected through PMU Interrupt */
			tegra_udc_pdata.support_pmu_vbus = true;
			tegra_udc_pdata.vbus_extcon_dev_name = "palmas-extcon";
			tegra_ehci1_utmi_pdata.support_pmu_vbus = true;
			tegra_ehci1_utmi_pdata.vbus_extcon_dev_name =
							 "palmas-extcon";
			/* Host cable is detected through PMU Interrupt */
			tegra_udc_pdata.id_det_type = TEGRA_USB_PMU_ID;
			tegra_ehci1_utmi_pdata.id_det_type = TEGRA_USB_PMU_ID;
			tegra_ehci1_utmi_pdata.id_extcon_dev_name =
							 "palmas-extcon";
		}

		/* Enable Y-Cable support */
		if (bi.board_id == BOARD_P1761)
			tegra_ehci1_utmi_pdata.u_data.host.support_y_cable =
							true;
	}

	if (!(usb_port_owner_info & UTMI1_PORT_OWNER_XUSB)) {
		tegra_otg_pdata.is_xhci = false;
		tegra_udc_pdata.u_data.dev.is_xhci = false;
	} else {
		tegra_otg_pdata.is_xhci = true;
		tegra_udc_pdata.u_data.dev.is_xhci = true;
	}
	tegra_otg_device.dev.platform_data = &tegra_otg_pdata;
	platform_device_register(&tegra_otg_device);
	/* Setup the udc platform data */
	tegra_udc_device.dev.platform_data = &tegra_udc_pdata;

	if (!(usb_port_owner_info & UTMI2_PORT_OWNER_XUSB)) {
		if (!modem_id) {
			if ((bi.board_id != BOARD_P1761) &&
			    (bi.board_id != BOARD_E1922) &&
			    (bi.board_id != BOARD_E1784)) {
				tegra_ehci2_device.dev.platform_data =
					&tegra_ehci2_utmi_pdata;
				platform_device_register(&tegra_ehci2_device);
			}
		}
	}

	if (!(usb_port_owner_info & UTMI2_PORT_OWNER_XUSB)) {
		if ((bi.board_id != BOARD_P1761) &&
		    (bi.board_id != BOARD_E1922) &&
		    (bi.board_id != BOARD_E1784)) {
			tegra_ehci3_device.dev.platform_data =
				&tegra_ehci3_utmi_pdata;
			platform_device_register(&tegra_ehci3_device);
		}
	}

}

static struct tegra_xusb_platform_data xusb_pdata = {
	.portmap = TEGRA_XUSB_SS_P0 | TEGRA_XUSB_USB2_P0 | TEGRA_XUSB_SS_P1 |
			TEGRA_XUSB_USB2_P1 | TEGRA_XUSB_USB2_P2,
};

#ifdef CONFIG_TEGRA_XUSB_PLATFORM
static void ardbeg_xusb_init(void)
{
	int usb_port_owner_info = tegra_get_usb_port_owner_info();

	xusb_pdata.lane_owner = (u8) tegra_get_lane_owner_info();

	if (board_info.board_id == BOARD_PM359 ||
			board_info.board_id == BOARD_PM358 ||
			board_info.board_id == BOARD_PM374 ||
			board_info.board_id == BOARD_PM370 ||
			board_info.board_id == BOARD_PM363) {
		if (board_info.board_id == BOARD_PM374 ||
			board_info.board_id == BOARD_PM370)
			pr_info("Norrin. 0x%x\n", board_info.board_id);
		else
			pr_info("Laguna. 0x%x\n", board_info.board_id);

		if (!(usb_port_owner_info & UTMI1_PORT_OWNER_XUSB))
			xusb_pdata.portmap &= ~(TEGRA_XUSB_USB2_P0 |
				TEGRA_XUSB_SS_P0);

		if (!(usb_port_owner_info & UTMI2_PORT_OWNER_XUSB))
			xusb_pdata.portmap &= ~(TEGRA_XUSB_USB2_P1 |
				TEGRA_XUSB_SS_P1 | TEGRA_XUSB_USB2_P2);

		/* FIXME Add for UTMIP2 when have odmdata assigend */
	} else if (board_info.board_id == BOARD_PM375) {
		if (!(usb_port_owner_info & UTMI1_PORT_OWNER_XUSB))
			xusb_pdata.portmap &= ~(TEGRA_XUSB_USB2_P0);
		if (!(usb_port_owner_info & UTMI2_PORT_OWNER_XUSB))
			xusb_pdata.portmap &= ~(TEGRA_XUSB_USB2_P2 |
					TEGRA_XUSB_USB2_P1 | TEGRA_XUSB_SS_P0);
		xusb_pdata.portmap &= ~(TEGRA_XUSB_SS_P1);
	} else {
		/* Ardbeg */
		if (board_info.board_id == BOARD_E1781) {
			pr_info("Shield ERS-S. 0x%x\n", board_info.board_id);
			/* Shield ERS-S */
			if (!(usb_port_owner_info & UTMI1_PORT_OWNER_XUSB))
				xusb_pdata.portmap &= ~(TEGRA_XUSB_USB2_P0);

			if (!(usb_port_owner_info & UTMI2_PORT_OWNER_XUSB))
				xusb_pdata.portmap &= ~(
					TEGRA_XUSB_USB2_P1 | TEGRA_XUSB_SS_P0 |
					TEGRA_XUSB_USB2_P2 | TEGRA_XUSB_SS_P1);
		} else {
			pr_info("Shield ERS 0x%x\n", board_info.board_id);
			/* Shield ERS */
			if (!(usb_port_owner_info & UTMI1_PORT_OWNER_XUSB))
				xusb_pdata.portmap &= ~(TEGRA_XUSB_USB2_P0 |
					TEGRA_XUSB_SS_P0);

			if (!(usb_port_owner_info & UTMI2_PORT_OWNER_XUSB))
				xusb_pdata.portmap &= ~(TEGRA_XUSB_USB2_P1 |
					TEGRA_XUSB_USB2_P2 | TEGRA_XUSB_SS_P1);
		}
		/* FIXME Add for UTMIP2 when have odmdata assigend */
	}

	if (usb_port_owner_info & HSIC1_PORT_OWNER_XUSB)
		xusb_pdata.portmap |= TEGRA_XUSB_HSIC_P0;

	if (usb_port_owner_info & HSIC2_PORT_OWNER_XUSB)
		xusb_pdata.portmap |= TEGRA_XUSB_HSIC_P1;
}
#endif

static int baseband_init(void)
{
	int ret;

	ret = gpio_request_array(modem_gpios, ARRAY_SIZE(modem_gpios));
	if (ret) {
		pr_warn("%s:gpio request failed\n", __func__);
		return ret;
	}

	/* enable pull-down for MDM_COLD_BOOT */
	tegra_pinmux_set_pullupdown(TEGRA_PINGROUP_ULPI_DATA4,
				    TEGRA_PUPD_PULL_DOWN);

	/* Release modem reset to start boot */
	gpio_set_value(MDM_RST, 1);

	/* export GPIO for user space access through sysfs */
	gpio_export(MDM_RST, false);
	gpio_export(MDM_SAR0, false);

	return 0;
}

static const struct tegra_modem_operations baseband_operations = {
	.init = baseband_init,
};

static struct tegra_usb_modem_power_platform_data baseband_pdata = {
	.ops = &baseband_operations,
	.regulator_name = "vdd_wwan_mdm",
	.wake_gpio = -1,
	.boot_gpio = MDM_COLDBOOT,
	.boot_irq_flags = IRQF_TRIGGER_RISING |
				    IRQF_TRIGGER_FALLING |
				    IRQF_ONESHOT,
	.autosuspend_delay = 2000,
	.short_autosuspend_delay = 50,
	.tegra_ehci_device = &tegra_ehci2_device,
	.tegra_ehci_pdata = &tegra_ehci2_hsic_baseband_pdata,
	.mdm_power_report_gpio = MDM_POWER_REPORT,
	.mdm_power_irq_flags = IRQF_TRIGGER_RISING |
				IRQF_TRIGGER_FALLING |
				IRQF_ONESHOT,
};

static struct platform_device icera_bruce_device = {
	.name = "tegra_usb_modem_power",
	.id = -1,
	.dev = {
		.platform_data = &baseband_pdata,
	},
};

static void ardbeg_modem_init(void)
{
	int modem_id = tegra_get_modem_id();
	struct board_info board_info;
	struct board_info pmu_board_info;
	int usb_port_owner_info = tegra_get_usb_port_owner_info();

	tegra_get_board_info(&board_info);
	tegra_get_pmu_board_info(&pmu_board_info);
	pr_info("%s: modem_id = %d\n", __func__, modem_id);

	switch (modem_id) {
	case TEGRA_BB_BRUCE:
		if (!(usb_port_owner_info & HSIC1_PORT_OWNER_XUSB)) {
			/* Set specific USB wake source for Ardbeg */
			if (board_info.board_id == BOARD_E1780)
				tegra_set_wake_source(42, INT_USB2);
			if (pmu_board_info.board_id == BOARD_E1736 ||
				pmu_board_info.board_id == BOARD_E1769 ||
				pmu_board_info.board_id == BOARD_E1936)
				baseband_pdata.regulator_name = NULL;
			platform_device_register(&icera_bruce_device);
		}
		break;
	case TEGRA_BB_HSIC_HUB: /* HSIC hub */
		if (!(usb_port_owner_info & HSIC1_PORT_OWNER_XUSB)) {
			tegra_ehci2_device.dev.platform_data =
				&tegra_ehci2_hsic_smsc_hub_pdata;
			/* Set specific USB wake source for Ardbeg */
			if (board_info.board_id == BOARD_E1780)
				tegra_set_wake_source(42, INT_USB2);
			platform_device_register(&tegra_ehci2_device);
		} else
			xusb_pdata.pretend_connect_0 = true;
		break;
	default:
		return;
	}
}

#ifdef CONFIG_USE_OF
static struct of_dev_auxdata ardbeg_auxdata_lookup[] __initdata = {
	T124_SPI_OF_DEV_AUXDATA,
	OF_DEV_AUXDATA("nvidia,tegra124-apbdma", 0x60020000, "tegra-apbdma",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-se", 0x70012000, "tegra12-se", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-host1x", TEGRA_HOST1X_BASE, "host1x",
		NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-gk20a", TEGRA_GK20A_BAR0_BASE,
		"gk20a.0", NULL),
#ifdef CONFIG_ARCH_TEGRA_VIC
	OF_DEV_AUXDATA("nvidia,tegra124-vic", TEGRA_VIC_BASE, "vic03.0", NULL),
#endif
	OF_DEV_AUXDATA("nvidia,tegra124-msenc", TEGRA_MSENC_BASE, "msenc",
		NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-vi", TEGRA_VI_BASE, "vi.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-isp", TEGRA_ISP_BASE, "isp.0", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-isp", TEGRA_ISPB_BASE, "isp.1", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-tsec", TEGRA_TSEC_BASE, "tsec", NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-hsuart", 0x70006000, "serial-tegra.0",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-hsuart", 0x70006040, "serial-tegra.1",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-hsuart", 0x70006200, "serial-tegra.2",
				NULL),
	T124_I2C_OF_DEV_AUXDATA,
	OF_DEV_AUXDATA("nvidia,tegra124-xhci", 0x70090000, "tegra-xhci",
				&xusb_pdata),
	OF_DEV_AUXDATA("nvidia,tegra124-dc", TEGRA_DISPLAY_BASE, "tegradc.0",
		NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-dc", TEGRA_DISPLAY2_BASE, "tegradc.1",
		NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-nvavp", 0x60001000, "nvavp",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-pwm", 0x7000a000, "tegra-pwm", NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-dfll", 0x70110000, "tegra_cl_dvfs",
		NULL),
	OF_DEV_AUXDATA("nvidia,tegra132-dfll", 0x70040084, "tegra_cl_dvfs",
		NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-efuse", TEGRA_FUSE_BASE, "tegra-fuse",
		NULL),
	OF_DEV_AUXDATA("nvidia,tegra124-camera", 0, "pcl-generic",
				NULL),
	OF_DEV_AUXDATA("nvidia,tegra114-ahci-sata", 0x70027000, "tegra-sata.0",
		NULL),
	{}
};
#endif

struct maxim_sti_pdata maxim_sti_pdata = {
	.touch_fusion         = "/vendor/bin/touch_fusion",
	.config_file          = "/vendor/firmware/touch_fusion.cfg",
	.fw_name              = "maxim_fp35.bin",
	.nl_family            = TF_FAMILY_NAME,
	.nl_mc_groups         = 5,
	.chip_access_method   = 2,
	.default_reset_state  = 0,
	.tx_buf_size          = 4100,
	.rx_buf_size          = 4100,
	.gpio_reset           = TOUCH_GPIO_RST_MAXIM_STI_SPI,
	.gpio_irq             = TOUCH_GPIO_IRQ_MAXIM_STI_SPI
};

struct maxim_sti_pdata maxim_sti_pdata_rd = {
	.touch_fusion         = "/vendor/bin/touch_fusion_rd",
	.config_file          = "/vendor/firmware/touch_fusion.cfg",
	.fw_name              = "maxim_fp35.bin",
	.nl_family            = TF_FAMILY_NAME,
	.nl_mc_groups         = 5,
	.chip_access_method   = 2,
	.default_reset_state  = 0,
	.tx_buf_size          = 4100,
	.rx_buf_size          = 4100,
	.gpio_reset           = TOUCH_GPIO_RST_MAXIM_STI_SPI,
	.gpio_irq             = TOUCH_GPIO_IRQ_MAXIM_STI_SPI
};

static struct tegra_spi_device_controller_data maxim_dev_cdata = {
	.rx_clk_tap_delay = 0,
	.is_hw_based_cs = true,
	.tx_clk_tap_delay = 0,
};

struct spi_board_info maxim_sti_spi_board = {
	.modalias = MAXIM_STI_NAME,
	.bus_num = TOUCH_SPI_ID,
	.chip_select = TOUCH_SPI_CS,
	.max_speed_hz = 12 * 1000 * 1000,
	.mode = SPI_MODE_0,
	.platform_data = &maxim_sti_pdata,
	.controller_data = &maxim_dev_cdata,
};

static __initdata struct tegra_clk_init_table touch_clk_init_table[] = {
	/* name         parent          rate            enabled */
	{ "extern2",    "pll_p",        41000000,       false},
	{ "clk_out_2",  "extern2",      40800000,       false},
	{ NULL,         NULL,           0,              0},
};

static struct rm_spi_ts_platform_data rm31080ts_ardbeg_data = {
	.gpio_reset = TOUCH_GPIO_RST_RAYDIUM_SPI,
	.config = 0,
	.platform_id = RM_PLATFORM_A010,
	.name_of_clock = "clk_out_2",
	.name_of_clock_con = "extern2",
};

static struct rm_spi_ts_platform_data rm31080ts_tn8_data = {
	.gpio_reset = TOUCH_GPIO_RST_RAYDIUM_SPI,
	.config = 0,
	.platform_id = RM_PLATFORM_T008,
	.name_of_clock = "clk_out_2",
	.name_of_clock_con = "extern2",
};

static struct rm_spi_ts_platform_data rm31080ts_tn8_p1765_data = {
	.gpio_reset = TOUCH_GPIO_RST_RAYDIUM_SPI,
	.config = 0,
	.platform_id = RM_PLATFORM_T008_2,
	.name_of_clock = "clk_out_2",
	.name_of_clock_con = "extern2",
};

static struct rm_spi_ts_platform_data rm31080ts_norrin_data = {
	.gpio_reset = TOUCH_GPIO_RST_RAYDIUM_SPI,
	.config = 0,
	.platform_id = RM_PLATFORM_P140,
	.name_of_clock = "clk_out_2",
	.name_of_clock_con = "extern2",
};

struct rm_spi_ts_platform_data rm31080ts_t132loki_data = {
	.gpio_reset = TOUCH_GPIO_RST_RAYDIUM_SPI,
	.config = 0,
	.platform_id = RM_PLATFORM_L005,
	.name_of_clock = "clk_out_2",
	.name_of_clock_con = "extern2",
};

struct rm_spi_ts_platform_data rm31080ts_t132loki_data_jdi_5 = {
	.gpio_reset = TOUCH_GPIO_RST_RAYDIUM_SPI,
	.config = 0,
	.platform_id = RM_PLATFORM_L005,
	.name_of_clock = "clk_out_2",
	.name_of_clock_con = "extern2",
	.gpio_sensor_select0 = false,
	.gpio_sensor_select1 = true,
};
static struct tegra_spi_device_controller_data dev_cdata = {
	.rx_clk_tap_delay = 0,
	.tx_clk_tap_delay = 16,
};

static struct spi_board_info rm31080a_ardbeg_spi_board[1] = {
	{
		.modalias = "rm_ts_spidev",
		.bus_num = TOUCH_SPI_ID,
		.chip_select = TOUCH_SPI_CS,
		.max_speed_hz = 12 * 1000 * 1000,
		.mode = SPI_MODE_0,
		.controller_data = &dev_cdata,
		.platform_data = &rm31080ts_ardbeg_data,
	},
};

static struct spi_board_info rm31080a_tn8_spi_board[1] = {
	{
		.modalias = "rm_ts_spidev",
		.bus_num = TOUCH_SPI_ID,
		.chip_select = TOUCH_SPI_CS,
		.max_speed_hz = 18 * 1000 * 1000,
		.mode = SPI_MODE_0,
		.controller_data = &dev_cdata,
		.platform_data = &rm31080ts_tn8_data,
	},
};

static struct spi_board_info rm31080a_tn8_p1765_spi_board[1] = {
	{
		.modalias = "rm_ts_spidev",
		.bus_num = TOUCH_SPI_ID,
		.chip_select = TOUCH_SPI_CS,
		.max_speed_hz = 18 * 1000 * 1000,
		.mode = SPI_MODE_0,
		.controller_data = &dev_cdata,
		.platform_data = &rm31080ts_tn8_p1765_data,
	},
};

static struct spi_board_info rm31080a_norrin_spi_board[1] = {
	{
		.modalias = "rm_ts_spidev",
		.bus_num = NORRIN_TOUCH_SPI_ID,
		.chip_select = NORRIN_TOUCH_SPI_CS,
		.max_speed_hz = 12 * 1000 * 1000,
		.mode = SPI_MODE_0,
		.controller_data = &dev_cdata,
		.platform_data = &rm31080ts_norrin_data,
	},
};

static int __init ardbeg_touch_init(void)
{
	tegra_get_board_info(&board_info);

	if (tegra_get_touch_vendor_id() == MAXIM_TOUCH) {
		pr_info("%s init maxim touch\n", __func__);
#if defined(CONFIG_TOUCHSCREEN_MAXIM_STI) || \
	defined(CONFIG_TOUCHSCREEN_MAXIM_STI_MODULE)
		if (tegra_get_touch_panel_id() == TOUCHPANEL_TN7)
			maxim_sti_spi_board.platform_data = &maxim_sti_pdata_rd;
		(void)touch_init_maxim_sti(&maxim_sti_spi_board);
#endif
	} else if (tegra_get_touch_vendor_id() == RAYDIUM_TOUCH) {
		pr_info("%s init raydium touch\n", __func__);
		tegra_clk_init_from_table(touch_clk_init_table);
		if (board_info.board_id == BOARD_PM374) {
			rm31080a_norrin_spi_board[0].irq =
				gpio_to_irq(TOUCH_GPIO_IRQ_RAYDIUM_SPI);
			touch_init_raydium(TOUCH_GPIO_IRQ_RAYDIUM_SPI,
					TOUCH_GPIO_RST_RAYDIUM_SPI,
					&rm31080ts_norrin_data,
					&rm31080a_norrin_spi_board[0],
					ARRAY_SIZE(rm31080a_norrin_spi_board));
		} else if ((board_info.board_id == BOARD_P2530) ||
			(board_info.board_id == BOARD_E2548)) {
			if (board_info.sku == BOARD_SKU_FOSTER)
				return 0;
			if (tegra_get_touch_panel_id() == TOUCHPANEL_LOKI_JDI5)
				rm31080a_ardbeg_spi_board[0].platform_data =
					&rm31080ts_t132loki_data_jdi_5;
			else
				rm31080a_ardbeg_spi_board[0].platform_data =
					&rm31080ts_t132loki_data;
			if (board_info.fab >= 0xa3) {
				rm31080ts_t132loki_data.name_of_clock = NULL;
				rm31080ts_t132loki_data.name_of_clock_con = NULL;
			} else
				tegra_clk_init_from_table(touch_clk_init_table);
				rm31080a_ardbeg_spi_board[0].irq =
					gpio_to_irq(TOUCH_GPIO_IRQ_RAYDIUM_SPI);
				touch_init_raydium(TOUCH_GPIO_IRQ_RAYDIUM_SPI,
					TOUCH_GPIO_RST_RAYDIUM_SPI,
					&rm31080ts_t132loki_data,
					&rm31080a_ardbeg_spi_board[0],
					ARRAY_SIZE(rm31080a_ardbeg_spi_board));
		} else if (board_info.board_id == BOARD_P1761) {
			rm31080a_tn8_spi_board[0].irq =
				gpio_to_irq(TOUCH_GPIO_IRQ_RAYDIUM_SPI);
			touch_init_raydium(TOUCH_GPIO_IRQ_RAYDIUM_SPI,
				TOUCH_GPIO_RST_RAYDIUM_SPI,
				&rm31080ts_tn8_data,
				&rm31080a_tn8_spi_board[0],
				ARRAY_SIZE(rm31080a_tn8_spi_board));
                } else if (board_info.board_id == BOARD_P1765) {
                        rm31080a_tn8_p1765_spi_board[0].irq =
                                gpio_to_irq(TOUCH_GPIO_IRQ_RAYDIUM_SPI);
                        touch_init_raydium(TOUCH_GPIO_IRQ_RAYDIUM_SPI,
                                TOUCH_GPIO_RST_RAYDIUM_SPI,
                                &rm31080ts_tn8_p1765_data,
                                &rm31080a_tn8_p1765_spi_board[0],
                                ARRAY_SIZE(rm31080a_tn8_p1765_spi_board));
		} else {
			rm31080a_ardbeg_spi_board[0].irq =
				gpio_to_irq(TOUCH_GPIO_IRQ_RAYDIUM_SPI);
			touch_init_raydium(TOUCH_GPIO_IRQ_RAYDIUM_SPI,
					TOUCH_GPIO_RST_RAYDIUM_SPI,
					&rm31080ts_ardbeg_data,
					&rm31080a_ardbeg_spi_board[0],
					ARRAY_SIZE(rm31080a_ardbeg_spi_board));
		}
	}
	return 0;
}

static void __init ardbeg_sysedp_init(void)
{
	struct board_info bi;

	tegra_get_board_info(&bi);

	switch (bi.board_id) {
	case BOARD_E1780:
		if (bi.sku == 1100) {
			tn8_new_sysedp_init();
		}
		else
			shield_new_sysedp_init();
		break;
	case BOARD_E1971:
	case BOARD_E1922:
	case BOARD_E1784:
	case BOARD_P1761:
	case BOARD_P1765:
		tn8_new_sysedp_init();
		break;
	case BOARD_PM358:
	case BOARD_PM359:
	case BOARD_PM375:
	default:
		break;
	}
}

static void __init ardbeg_sysedp_dynamic_capping_init(void)
{
	struct board_info bi;

	tegra_get_board_info(&bi);

	switch (bi.board_id) {
	case BOARD_E1780:
		if (bi.sku == 1100)
			tn8_sysedp_dynamic_capping_init();
		else
			shield_sysedp_dynamic_capping_init();
		break;
	case BOARD_E1971:
	case BOARD_E1922:
	case BOARD_E1784:
	case BOARD_P1761:
	case BOARD_P1765:
		tn8_sysedp_dynamic_capping_init();
		break;
	case BOARD_PM358:
	case BOARD_PM359:
	case BOARD_PM375:
	default:
		break;
	}
}

static void __init ardbeg_sysedp_batmon_init(void)
{
	struct board_info bi;

	if (!IS_ENABLED(CONFIG_SYSEDP_FRAMEWORK))
		return;

	tegra_get_board_info(&bi);

	switch (bi.board_id) {
	case BOARD_E1780:
		if (bi.sku != 1100)
			shield_sysedp_batmon_init();
		break;
	case BOARD_PM358:
	case BOARD_PM359:
	case BOARD_PM375:
	default:
		break;
	}
}



static void __init edp_init(void)
{
	struct board_info bi;

	tegra_get_board_info(&bi);

	switch (bi.board_id) {
	case BOARD_E1780:
		if (bi.sku == 1100)
			tn8_edp_init();
		else
			ardbeg_edp_init();
		break;
	case BOARD_P1761:
			tn8_edp_init();
			break;
	case BOARD_PM358:
	case BOARD_PM359:
	case BOARD_PM375:
			laguna_edp_init();
			break;
	case BOARD_P2530:
	case BOARD_E2548:
			loki_edp_init();
			break;
	default:
			ardbeg_edp_init();
			break;
	}
}

static void __init tegra_ardbeg_early_init(void)
{
	ardbeg_sysedp_init();
	tegra_clk_init_from_table(ardbeg_clk_init_table);
	tegra_clk_verify_parents();
	if (of_machine_is_compatible("nvidia,jetson-tk1"))
		tegra_soc_device_init("jetson-tk1");
	else if (of_machine_is_compatible("nvidia,laguna"))
		tegra_soc_device_init("laguna");
	else if (of_machine_is_compatible("nvidia,tn8"))
		tegra_soc_device_init("tn8");
	else if (of_machine_is_compatible("nvidia,ardbeg_sata"))
		tegra_soc_device_init("ardbeg_sata");
	else if (of_machine_is_compatible("nvidia,norrin"))
		tegra_soc_device_init("norrin");
	else if (of_machine_is_compatible("nvidia,bowmore"))
		tegra_soc_device_init("bowmore");
	else if (of_machine_is_compatible("nvidia,t132loki"))
		tegra_soc_device_init("t132loki");
	else
		tegra_soc_device_init("ardbeg");
}

static struct tegra_dtv_platform_data ardbeg_dtv_pdata = {
	.dma_req_selector = 11,
};

static void __init ardbeg_dtv_init(void)
{
	tegra_dtv_device.dev.platform_data = &ardbeg_dtv_pdata;
	platform_device_register(&tegra_dtv_device);
}

static struct tegra_io_dpd pexbias_io = {
	.name			= "PEX_BIAS",
	.io_dpd_reg_index	= 0,
	.io_dpd_bit		= 4,
};
static struct tegra_io_dpd pexclk1_io = {
	.name			= "PEX_CLK1",
	.io_dpd_reg_index	= 0,
	.io_dpd_bit		= 5,
};
static struct tegra_io_dpd pexclk2_io = {
	.name			= "PEX_CLK2",
	.io_dpd_reg_index	= 0,
	.io_dpd_bit		= 6,
};

static void __init tegra_ardbeg_late_init(void)
{
	struct board_info board_info;
	tegra_get_board_info(&board_info);
	pr_info("board_info: id:sku:fab:major:minor = 0x%04x:0x%04x:0x%02x:0x%02x:0x%02x\n",
		board_info.board_id, board_info.sku,
		board_info.fab, board_info.major_revision,
		board_info.minor_revision);

	if (board_info.board_id == BOARD_E2548 ||
			board_info.board_id == BOARD_P2530)
		loki_pinmux_init();
#ifndef CONFIG_MACH_EXUMA
	ardbeg_display_init();
#endif
	ardbeg_uart_init();
	ardbeg_usb_init();
	ardbeg_modem_init();
#ifdef CONFIG_TEGRA_XUSB_PLATFORM
	ardbeg_xusb_init();
#endif
	ardbeg_i2c_init();
	ardbeg_audio_init();
	platform_add_devices(ardbeg_devices, ARRAY_SIZE(ardbeg_devices));
	if (board_info.board_id == BOARD_PM374)	/* Norrin ERS */
		platform_device_register(&norrin_audio_device_max98090);
	else if (board_info.board_id != BOARD_PM359)
		platform_device_register(&ardbeg_audio_device_rt5639);
	tegra_io_dpd_init();
	if (board_info.board_id == BOARD_E2548 ||
			board_info.board_id == BOARD_P2530)
		loki_sdhci_init();
	else
		ardbeg_sdhci_init();
	if (board_info.board_id == BOARD_PM359 ||
			board_info.board_id == BOARD_PM358 ||
			board_info.board_id == BOARD_PM370 ||
			board_info.board_id == BOARD_PM375 ||
			board_info.board_id == BOARD_PM363)
		laguna_regulator_init();
	else if (board_info.board_id == BOARD_PM374)
		norrin_regulator_init();
	else if (board_info.board_id == BOARD_E2548 ||
			board_info.board_id == BOARD_P2530)
		loki_regulator_init();
	else
		ardbeg_regulator_init();
	ardbeg_dtv_init();
	ardbeg_suspend_init();

	if ((board_info.board_id == BOARD_PM374) ||
		(board_info.board_id == BOARD_E1971) ||
		(board_info.board_id == BOARD_E1973))
		norrin_emc_init();
	else if (board_info.board_id == BOARD_E2548 ||
			board_info.board_id == BOARD_P2530)
		loki_emc_init();
	else
		ardbeg_emc_init();

	edp_init();
	isomgr_init();
	ardbeg_touch_init();
	if (board_info.board_id == BOARD_E2548 ||
			board_info.board_id == BOARD_P2530)
		loki_panel_init();
	else
		ardbeg_panel_init();

		/* put PEX pads into DPD mode to save additional power */
		tegra_io_dpd_enable(&pexbias_io);
		tegra_io_dpd_enable(&pexclk1_io);
		tegra_io_dpd_enable(&pexclk2_io);

	if (board_info.board_id == BOARD_E2548 ||
			board_info.board_id == BOARD_P2530)
		loki_kbc_init();

#ifdef CONFIG_TEGRA_WDT_RECOVERY
	tegra_wdt_recovery_init();
#endif


	if (board_info.board_id == BOARD_PM374 ||
		board_info.board_id == BOARD_PM359 ||
		board_info.board_id == BOARD_PM358 ||
		board_info.board_id == BOARD_PM370 ||
		board_info.board_id == BOARD_PM375 ||
		board_info.board_id == BOARD_PM363) {
		ardbeg_sensors_init();
		norrin_soctherm_init();
	}	else if (board_info.board_id == BOARD_E2548 ||
			board_info.board_id == BOARD_P2530) {
		loki_sensors_init();
		loki_fan_init();
		loki_soctherm_init();
	}	else {
		ardbeg_sensors_init();
		ardbeg_soctherm_init();
	}

	ardbeg_setup_bluedroid_pm();
	ardbeg_sysedp_dynamic_capping_init();
	ardbeg_sysedp_batmon_init();
}

static void __init tegra_ardbeg_init_early(void)
{
	tegra_get_board_info(&board_info);
	if (board_info.board_id == BOARD_E2548 ||
			board_info.board_id == BOARD_P2530)
		loki_rail_alignment_init();
	else
		ardbeg_rail_alignment_init();
	tegra12x_init_early();
}

static void __init tegra_ardbeg_dt_init(void)
{
	tegra_get_board_info(&board_info);
	tegra_get_display_board_info(&display_board_info);

	tegra_ardbeg_early_init();
#ifdef CONFIG_NVMAP_USE_CMA_FOR_CARVEOUT
	carveout_linear_set(&tegra_generic_cma_dev);
	carveout_linear_set(&tegra_vpr_cma_dev);
#endif
#ifdef CONFIG_USE_OF
	ardbeg_camera_auxdata(ardbeg_auxdata_lookup);
	of_platform_populate(NULL,
		of_default_bus_match_table, ardbeg_auxdata_lookup,
		&platform_bus);
#endif

	tegra_ardbeg_late_init();
}

static void __init tegra_ardbeg_reserve(void)
{
#ifdef CONFIG_TEGRA_HDMI_PRIMARY
	ulong tmp;
#endif /* CONFIG_TEGRA_HDMI_PRIMARY */

#if defined(CONFIG_NVMAP_CONVERT_CARVEOUT_TO_IOVMM) || \
		defined(CONFIG_TEGRA_NO_CARVEOUT)
	ulong carveout_size = 0;
	ulong fb2_size = SZ_16M;
#else
	ulong carveout_size = SZ_1G;
	ulong fb2_size = SZ_4M;
#endif
	ulong fb1_size = SZ_16M + SZ_2M;
	ulong vpr_size = 186 * SZ_1M;

#ifdef CONFIG_FRAMEBUFFER_CONSOLE
	/* support FBcon on 4K monitors */
	fb2_size = SZ_64M + SZ_8M;	/* 4096*2160*4*2 = 70778880 bytes */
#endif /* CONFIG_FRAMEBUFFER_CONSOLE */

#ifdef CONFIG_TEGRA_HDMI_PRIMARY
	tmp = fb1_size;
	fb1_size = fb2_size;
	fb2_size = tmp;
#endif /* CONFIG_TEGRA_HDMI_PRIMARY */

	tegra_reserve4(carveout_size, fb1_size, fb2_size, vpr_size);
}

static const char * const ardbeg_dt_board_compat[] = {
	"nvidia,ardbeg",
	NULL
};

static const char * const laguna_dt_board_compat[] = {
	"nvidia,laguna",
	NULL
};

static const char * const tn8_dt_board_compat[] = {
	"nvidia,tn8",
	NULL
};

static const char * const ardbeg_sata_dt_board_compat[] = {
	"nvidia,ardbeg_sata",
	NULL
};

static const char * const norrin_dt_board_compat[] = {
	"nvidia,norrin",
	NULL
};

static const char * const bowmore_dt_board_compat[] = {
	"nvidia,bowmore",
	NULL
};

static const char * const loki_dt_board_compat[] = {
	"nvidia,t132loki",
	NULL
};

static const char * const jetson_dt_board_compat[] = {
	"nvidia,jetson-tk1",
	NULL
};

#ifdef CONFIG_ARCH_TEGRA_13x_SOC
DT_MACHINE_START(LOKI, "t132loki")
	.atag_offset	= 0x100,
	.smp		= smp_ops(tegra_smp_ops),
	.map_io		= tegra_map_common_io,
	.reserve	= tegra_ardbeg_reserve,
	.init_early	= tegra_ardbeg_init_early,
	.init_irq	= irqchip_init,
	.init_time	= clocksource_of_init,
	.init_machine	= tegra_ardbeg_dt_init,
	.restart	= tegra_assert_system_reset,
	.dt_compat	= loki_dt_board_compat,
	.init_late      = tegra_init_late
MACHINE_END
#endif

DT_MACHINE_START(LAGUNA, "laguna")
	.atag_offset	= 0x100,
	.smp		= smp_ops(tegra_smp_ops),
	.map_io		= tegra_map_common_io,
	.reserve	= tegra_ardbeg_reserve,
	.init_early	= tegra_ardbeg_init_early,
	.init_irq	= irqchip_init,
	.init_time	= clocksource_of_init,
	.init_machine	= tegra_ardbeg_dt_init,
	.restart	= tegra_assert_system_reset,
	.dt_compat	= laguna_dt_board_compat,
	.init_late      = tegra_init_late
MACHINE_END

DT_MACHINE_START(TN8, "tn8")
	.atag_offset	= 0x100,
	.smp		= smp_ops(tegra_smp_ops),
	.map_io		= tegra_map_common_io,
	.reserve	= tegra_ardbeg_reserve,
	.init_early	= tegra_ardbeg_init_early,
	.init_irq	= irqchip_init,
	.init_time	= clocksource_of_init,
	.init_machine	= tegra_ardbeg_dt_init,
	.restart	= tegra_assert_system_reset,
	.dt_compat	= tn8_dt_board_compat,
	.init_late      = tegra_init_late
MACHINE_END

DT_MACHINE_START(NORRIN, "norrin")
	.atag_offset	= 0x100,
	.smp		= smp_ops(tegra_smp_ops),
	.map_io		= tegra_map_common_io,
	.reserve	= tegra_ardbeg_reserve,
	.init_early	= tegra_ardbeg_init_early,
	.init_irq	= irqchip_init,
	.init_time	= clocksource_of_init,
	.init_machine	= tegra_ardbeg_dt_init,
	.restart	= tegra_assert_system_reset,
	.dt_compat	= norrin_dt_board_compat,
	.init_late      = tegra_init_late
MACHINE_END

DT_MACHINE_START(BOWMORE, "bowmore")
	.atag_offset	= 0x100,
	.smp		= smp_ops(tegra_smp_ops),
	.map_io		= tegra_map_common_io,
	.reserve	= tegra_ardbeg_reserve,
	.init_early	= tegra_ardbeg_init_early,
	.init_irq	= irqchip_init,
	.init_time	= clocksource_of_init,
	.init_machine	= tegra_ardbeg_dt_init,
	.restart	= tegra_assert_system_reset,
	.dt_compat	= bowmore_dt_board_compat,
	.init_late      = tegra_init_late
MACHINE_END

DT_MACHINE_START(ARDBEG, "ardbeg")
	.atag_offset	= 0x100,
	.smp		= smp_ops(tegra_smp_ops),
	.map_io		= tegra_map_common_io,
	.reserve	= tegra_ardbeg_reserve,
	.init_early	= tegra_ardbeg_init_early,
	.init_irq	= irqchip_init,
	.init_time	= clocksource_of_init,
	.init_machine	= tegra_ardbeg_dt_init,
	.restart	= tegra_assert_system_reset,
	.dt_compat	= ardbeg_dt_board_compat,
	.init_late      = tegra_init_late
MACHINE_END

DT_MACHINE_START(ARDBEG_SATA, "ardbeg_sata")
	.atag_offset	= 0x100,
	.smp		= smp_ops(tegra_smp_ops),
	.map_io		= tegra_map_common_io,
	.reserve	= tegra_ardbeg_reserve,
	.init_early	= tegra_ardbeg_init_early,
	.init_irq	= irqchip_init,
	.init_time	= clocksource_of_init,
	.init_machine	= tegra_ardbeg_dt_init,
	.restart	= tegra_assert_system_reset,
	.dt_compat	= ardbeg_sata_dt_board_compat,
	.init_late      = tegra_init_late

MACHINE_END

DT_MACHINE_START(JETSON_TK1, "jetson-tk1")
	.atag_offset	= 0x100,
	.smp		= smp_ops(tegra_smp_ops),
	.map_io		= tegra_map_common_io,
	.reserve	= tegra_ardbeg_reserve,
	.init_early	= tegra_ardbeg_init_early,
	.init_irq	= irqchip_init,
	.init_time	= clocksource_of_init,
	.init_machine	= tegra_ardbeg_dt_init,
	.restart	= tegra_assert_system_reset,
	.dt_compat	= jetson_dt_board_compat,
	.init_late      = tegra_init_late
MACHINE_END
