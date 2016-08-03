/*
 * drivers/video/tegra/dc/dc.c
 *
 * Copyright (C) 2010 Google, Inc.
 * Author: Erik Gilling <konkers@android.com>
 *
 * Copyright (c) 2010-2014, NVIDIA CORPORATION, All rights reserved.
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/workqueue.h>
#include <linux/ktime.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/backlight.h>
#include <linux/gpio.h>
#include <linux/nvhost.h>
#include <linux/clk/tegra.h>
#include <video/tegrafb.h>
#include <drm/drm_fixed.h>
#ifdef CONFIG_SWITCH
#include <linux/switch.h>
#endif
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/tegra_pm_domains.h>

#define CREATE_TRACE_POINTS
#include <trace/events/display.h>

#include <mach/dc.h>
#include <mach/fb.h>
#include <mach/mc.h>
#include <linux/nvhost.h>
#include <linux/nvhost_ioctl.h>
#include <mach/latency_allowance.h>

#include "dc_reg.h"
#include "dc_config.h"
#include "dc_priv.h"
#include "dev.h"
#include "nvhost_sync.h"
#include "nvsd.h"
#include "dp.h"
#include "nvsr.h"

#ifdef CONFIG_ADF_TEGRA
#include "tegra_adf.h"
#endif

#ifdef CONFIG_FRAMEBUFFER_CONSOLE
#include "hdmi.h"
#endif /* CONFIG_FRAMEBUFFER_CONSOLE */

/* HACK! This needs to come from DT */
#include "../../../../arch/arm/mach-tegra/iomap.h"

#define TEGRA_CRC_LATCHED_DELAY		34

#define DC_COM_PIN_OUTPUT_POLARITY1_INIT_VAL	0x01000000
#define DC_COM_PIN_OUTPUT_POLARITY3_INIT_VAL	0x0

static struct fb_videomode tegra_dc_vga_mode = {
	.refresh = 60,
	.xres = 640,
	.yres = 480,
	.pixclock = KHZ2PICOS(25200),
	.hsync_len = 96,	/* h_sync_width */
	.vsync_len = 2,		/* v_sync_width */
	.left_margin = 48,	/* h_back_porch */
	.upper_margin = 33,	/* v_back_porch */
	.right_margin = 16,	/* h_front_porch */
	.lower_margin = 10,	/* v_front_porch */
	.vmode = 0,
	.sync = 0,
};

static struct tegra_dc_mode override_disp_mode[3];

static void _tegra_dc_controller_disable(struct tegra_dc *dc);

struct tegra_dc *tegra_dcs[TEGRA_MAX_DC];

DEFINE_MUTEX(tegra_dc_lock);
DEFINE_MUTEX(shared_lock);

static struct device_dma_parameters tegra_dc_dma_parameters = {
	.max_segment_size = UINT_MAX,
};

#ifdef CONFIG_TEGRA_DC_CMU
static struct tegra_dc_cmu default_cmu = {
	/* lut1 maps sRGB to linear space. */
	{
		0,    1,    2,    4,    5,    6,    7,    9,
		10,   11,   12,   14,   15,   16,   18,   20,
		21,   23,   25,   27,   29,   31,   33,   35,
		37,   40,   42,   45,   48,   50,   53,   56,
		59,   62,   66,   69,   72,   76,   79,   83,
		87,   91,   95,   99,   103,  107,  112,  116,
		121,  126,  131,  136,  141,  146,  151,  156,
		162,  168,  173,  179,  185,  191,  197,  204,
		210,  216,  223,  230,  237,  244,  251,  258,
		265,  273,  280,  288,  296,  304,  312,  320,
		329,  337,  346,  354,  363,  372,  381,  390,
		400,  409,  419,  428,  438,  448,  458,  469,
		479,  490,  500,  511,  522,  533,  544,  555,
		567,  578,  590,  602,  614,  626,  639,  651,
		664,  676,  689,  702,  715,  728,  742,  755,
		769,  783,  797,  811,  825,  840,  854,  869,
		884,  899,  914,  929,  945,  960,  976,  992,
		1008, 1024, 1041, 1057, 1074, 1091, 1108, 1125,
		1142, 1159, 1177, 1195, 1213, 1231, 1249, 1267,
		1286, 1304, 1323, 1342, 1361, 1381, 1400, 1420,
		1440, 1459, 1480, 1500, 1520, 1541, 1562, 1582,
		1603, 1625, 1646, 1668, 1689, 1711, 1733, 1755,
		1778, 1800, 1823, 1846, 1869, 1892, 1916, 1939,
		1963, 1987, 2011, 2035, 2059, 2084, 2109, 2133,
		2159, 2184, 2209, 2235, 2260, 2286, 2312, 2339,
		2365, 2392, 2419, 2446, 2473, 2500, 2527, 2555,
		2583, 2611, 2639, 2668, 2696, 2725, 2754, 2783,
		2812, 2841, 2871, 2901, 2931, 2961, 2991, 3022,
		3052, 3083, 3114, 3146, 3177, 3209, 3240, 3272,
		3304, 3337, 3369, 3402, 3435, 3468, 3501, 3535,
		3568, 3602, 3636, 3670, 3705, 3739, 3774, 3809,
		3844, 3879, 3915, 3950, 3986, 4022, 4059, 4095,
	},
	/* csc */
	{
		0x100, 0x0,   0x0,
		0x0,   0x100, 0x0,
		0x0,   0x0,   0x100,
	},
	/* lut2 maps linear space to sRGB*/
	{
		0,    1,    2,    2,    3,    4,    5,    6,
		6,    7,    8,    9,    10,   10,   11,   12,
		13,   13,   14,   15,   15,   16,   16,   17,
		18,   18,   19,   19,   20,   20,   21,   21,
		22,   22,   23,   23,   23,   24,   24,   25,
		25,   25,   26,   26,   27,   27,   27,   28,
		28,   29,   29,   29,   30,   30,   30,   31,
		31,   31,   32,   32,   32,   33,   33,   33,
		34,   34,   34,   34,   35,   35,   35,   36,
		36,   36,   37,   37,   37,   37,   38,   38,
		38,   38,   39,   39,   39,   40,   40,   40,
		40,   41,   41,   41,   41,   42,   42,   42,
		42,   43,   43,   43,   43,   43,   44,   44,
		44,   44,   45,   45,   45,   45,   46,   46,
		46,   46,   46,   47,   47,   47,   47,   48,
		48,   48,   48,   48,   49,   49,   49,   49,
		49,   50,   50,   50,   50,   50,   51,   51,
		51,   51,   51,   52,   52,   52,   52,   52,
		53,   53,   53,   53,   53,   54,   54,   54,
		54,   54,   55,   55,   55,   55,   55,   55,
		56,   56,   56,   56,   56,   57,   57,   57,
		57,   57,   57,   58,   58,   58,   58,   58,
		58,   59,   59,   59,   59,   59,   59,   60,
		60,   60,   60,   60,   60,   61,   61,   61,
		61,   61,   61,   62,   62,   62,   62,   62,
		62,   63,   63,   63,   63,   63,   63,   64,
		64,   64,   64,   64,   64,   64,   65,   65,
		65,   65,   65,   65,   66,   66,   66,   66,
		66,   66,   66,   67,   67,   67,   67,   67,
		67,   67,   68,   68,   68,   68,   68,   68,
		68,   69,   69,   69,   69,   69,   69,   69,
		70,   70,   70,   70,   70,   70,   70,   71,
		71,   71,   71,   71,   71,   71,   72,   72,
		72,   72,   72,   72,   72,   72,   73,   73,
		73,   73,   73,   73,   73,   74,   74,   74,
		74,   74,   74,   74,   74,   75,   75,   75,
		75,   75,   75,   75,   75,   76,   76,   76,
		76,   76,   76,   76,   77,   77,   77,   77,
		77,   77,   77,   77,   78,   78,   78,   78,
		78,   78,   78,   78,   78,   79,   79,   79,
		79,   79,   79,   79,   79,   80,   80,   80,
		80,   80,   80,   80,   80,   81,   81,   81,
		81,   81,   81,   81,   81,   81,   82,   82,
		82,   82,   82,   82,   82,   82,   83,   83,
		83,   83,   83,   83,   83,   83,   83,   84,
		84,   84,   84,   84,   84,   84,   84,   84,
		85,   85,   85,   85,   85,   85,   85,   85,
		85,   86,   86,   86,   86,   86,   86,   86,
		86,   86,   87,   87,   87,   87,   87,   87,
		87,   87,   87,   88,   88,   88,   88,   88,
		88,   88,   88,   88,   88,   89,   89,   89,
		89,   89,   89,   89,   89,   89,   90,   90,
		90,   90,   90,   90,   90,   90,   90,   90,
		91,   91,   91,   91,   91,   91,   91,   91,
		91,   91,   92,   92,   92,   92,   92,   92,
		92,   92,   92,   92,   93,   93,   93,   93,
		93,   93,   93,   93,   93,   93,   94,   94,
		94,   94,   94,   94,   94,   94,   94,   94,
		95,   95,   95,   95,   95,   95,   95,   95,
		95,   95,   96,   96,   96,   96,   96,   96,
		96,   96,   96,   96,   96,   97,   97,   97,
		97,   97,   97,   97,   97,   97,   97,   98,
		98,   98,   98,   98,   98,   98,   98,   98,
		98,   98,   99,   99,   99,   99,   99,   99,
		99,   100,  101,  101,  102,  103,  103,  104,
		105,  105,  106,  107,  107,  108,  109,  109,
		110,  111,  111,  112,  113,  113,  114,  115,
		115,  116,  116,  117,  118,  118,  119,  119,
		120,  120,  121,  122,  122,  123,  123,  124,
		124,  125,  126,  126,  127,  127,  128,  128,
		129,  129,  130,  130,  131,  131,  132,  132,
		133,  133,  134,  134,  135,  135,  136,  136,
		137,  137,  138,  138,  139,  139,  140,  140,
		141,  141,  142,  142,  143,  143,  144,  144,
		145,  145,  145,  146,  146,  147,  147,  148,
		148,  149,  149,  150,  150,  150,  151,  151,
		152,  152,  153,  153,  153,  154,  154,  155,
		155,  156,  156,  156,  157,  157,  158,  158,
		158,  159,  159,  160,  160,  160,  161,  161,
		162,  162,  162,  163,  163,  164,  164,  164,
		165,  165,  166,  166,  166,  167,  167,  167,
		168,  168,  169,  169,  169,  170,  170,  170,
		171,  171,  172,  172,  172,  173,  173,  173,
		174,  174,  174,  175,  175,  176,  176,  176,
		177,  177,  177,  178,  178,  178,  179,  179,
		179,  180,  180,  180,  181,  181,  182,  182,
		182,  183,  183,  183,  184,  184,  184,  185,
		185,  185,  186,  186,  186,  187,  187,  187,
		188,  188,  188,  189,  189,  189,  189,  190,
		190,  190,  191,  191,  191,  192,  192,  192,
		193,  193,  193,  194,  194,  194,  195,  195,
		195,  196,  196,  196,  196,  197,  197,  197,
		198,  198,  198,  199,  199,  199,  200,  200,
		200,  200,  201,  201,  201,  202,  202,  202,
		202,  203,  203,  203,  204,  204,  204,  205,
		205,  205,  205,  206,  206,  206,  207,  207,
		207,  207,  208,  208,  208,  209,  209,  209,
		209,  210,  210,  210,  211,  211,  211,  211,
		212,  212,  212,  213,  213,  213,  213,  214,
		214,  214,  214,  215,  215,  215,  216,  216,
		216,  216,  217,  217,  217,  217,  218,  218,
		218,  219,  219,  219,  219,  220,  220,  220,
		220,  221,  221,  221,  221,  222,  222,  222,
		223,  223,  223,  223,  224,  224,  224,  224,
		225,  225,  225,  225,  226,  226,  226,  226,
		227,  227,  227,  227,  228,  228,  228,  228,
		229,  229,  229,  229,  230,  230,  230,  230,
		231,  231,  231,  231,  232,  232,  232,  232,
		233,  233,  233,  233,  234,  234,  234,  234,
		235,  235,  235,  235,  236,  236,  236,  236,
		237,  237,  237,  237,  238,  238,  238,  238,
		239,  239,  239,  239,  240,  240,  240,  240,
		240,  241,  241,  241,  241,  242,  242,  242,
		242,  243,  243,  243,  243,  244,  244,  244,
		244,  244,  245,  245,  245,  245,  246,  246,
		246,  246,  247,  247,  247,  247,  247,  248,
		248,  248,  248,  249,  249,  249,  249,  249,
		250,  250,  250,  250,  251,  251,  251,  251,
		251,  252,  252,  252,  252,  253,  253,  253,
		253,  253,  254,  254,  254,  254,  255,  255,
	},
};

static struct tegra_dc_cmu default_limited_cmu = {
	/* lut1 maps sRGB to linear space. */
	{
		0,    1,    2,    4,    5,    6,    7,    9,
		10,   11,   12,   14,   15,   16,   18,   20,
		21,   23,   25,   27,   29,   31,   33,   35,
		37,   40,   42,   45,   48,   50,   53,   56,
		59,   62,   66,   69,   72,   76,   79,   83,
		87,   91,   95,   99,   103,  107,  112,  116,
		121,  126,  131,  136,  141,  146,  151,  156,
		162,  168,  173,  179,  185,  191,  197,  204,
		210,  216,  223,  230,  237,  244,  251,  258,
		265,  273,  280,  288,  296,  304,  312,  320,
		329,  337,  346,  354,  363,  372,  381,  390,
		400,  409,  419,  428,  438,  448,  458,  469,
		479,  490,  500,  511,  522,  533,  544,  555,
		567,  578,  590,  602,  614,  626,  639,  651,
		664,  676,  689,  702,  715,  728,  742,  755,
		769,  783,  797,  811,  825,  840,  854,  869,
		884,  899,  914,  929,  945,  960,  976,  992,
		1008, 1024, 1041, 1057, 1074, 1091, 1108, 1125,
		1142, 1159, 1177, 1195, 1213, 1231, 1249, 1267,
		1286, 1304, 1323, 1342, 1361, 1381, 1400, 1420,
		1440, 1459, 1480, 1500, 1520, 1541, 1562, 1582,
		1603, 1625, 1646, 1668, 1689, 1711, 1733, 1755,
		1778, 1800, 1823, 1846, 1869, 1892, 1916, 1939,
		1963, 1987, 2011, 2035, 2059, 2084, 2109, 2133,
		2159, 2184, 2209, 2235, 2260, 2286, 2312, 2339,
		2365, 2392, 2419, 2446, 2473, 2500, 2527, 2555,
		2583, 2611, 2639, 2668, 2696, 2725, 2754, 2783,
		2812, 2841, 2871, 2901, 2931, 2961, 2991, 3022,
		3052, 3083, 3114, 3146, 3177, 3209, 3240, 3272,
		3304, 3337, 3369, 3402, 3435, 3468, 3501, 3535,
		3568, 3602, 3636, 3670, 3705, 3739, 3774, 3809,
		3844, 3879, 3915, 3950, 3986, 4022, 4059, 4095,
	},
	/* csc */
	{
		0x100, 0x000, 0x000,
		0x000, 0x100, 0x000,
		0x000, 0x000, 0x100,
	},
	/*
	 * lut2 maps linear space back to sRGB, where
	 * the output range is [16...235] (limited).
	 */
	{
		16,  17,  17,  18,  19,  19,  20,  21,
		22,  22,  23,  24,  24,  25,  26,  26,
		27,  27,  28,  29,  29,  30,  30,  31,
		31,  32,  32,  32,  33,  33,  34,  34,
		35,  35,  35,  36,  36,  36,  37,  37,
		38,  38,  38,  39,  39,  39,  40,  40,
		40,  41,  41,  41,  41,  42,  42,  42,
		43,  43,  43,  43,  44,  44,  44,  45,
		45,  45,  45,  46,  46,  46,  46,  47,
		47,  47,  47,  48,  48,  48,  48,  49,
		49,  49,  49,  49,  50,  50,  50,  50,
		51,  51,  51,  51,  51,  52,  52,  52,
		52,  53,  53,  53,  53,  53,  54,  54,
		54,  54,  54,  55,  55,  55,  55,  55,
		56,  56,  56,  56,  56,  56,  57,  57,
		57,  57,  57,  58,  58,  58,  58,  58,
		58,  59,  59,  59,  59,  59,  60,  60,
		60,  60,  60,  60,  61,  61,  61,  61,
		61,  61,  62,  62,  62,  62,  62,  62,
		63,  63,  63,  63,  63,  63,  63,  64,
		64,  64,  64,  64,  64,  65,  65,  65,
		65,  65,  65,  65,  66,  66,  66,  66,
		66,  66,  67,  67,  67,  67,  67,  67,
		67,  68,  68,  68,  68,  68,  68,  68,
		69,  69,  69,  69,  69,  69,  69,  69,
		70,  70,  70,  70,  70,  70,  70,  71,
		71,  71,  71,  71,  71,  71,  72,  72,
		72,  72,  72,  72,  72,  72,  73,  73,
		73,  73,  73,  73,  73,  73,  74,  74,
		74,  74,  74,  74,  74,  74,  75,  75,
		75,  75,  75,  75,  75,  75,  76,  76,
		76,  76,  76,  76,  76,  76,  76,  77,
		77,  77,  77,  77,  77,  77,  77,  78,
		78,  78,  78,  78,  78,  78,  78,  78,
		79,  79,  79,  79,  79,  79,  79,  79,
		80,  80,  80,  80,  80,  80,  80,  80,
		80,  81,  81,  81,  81,  81,  81,  81,
		81,  81,  81,  82,  82,  82,  82,  82,
		82,  82,  82,  82,  83,  83,  83,  83,
		83,  83,  83,  83,  83,  84,  84,  84,
		84,  84,  84,  84,  84,  84,  84,  85,
		85,  85,  85,  85,  85,  85,  85,  85,
		85,  86,  86,  86,  86,  86,  86,  86,
		86,  86,  86,  87,  87,  87,  87,  87,
		87,  87,  87,  87,  87,  88,  88,  88,
		88,  88,  88,  88,  88,  88,  88,  89,
		89,  89,  89,  89,  89,  89,  89,  89,
		89,  89,  90,  90,  90,  90,  90,  90,
		90,  90,  90,  90,  91,  91,  91,  91,
		91,  91,  91,  91,  91,  91,  91,  92,
		92,  92,  92,  92,  92,  92,  92,  92,
		92,  92,  93,  93,  93,  93,  93,  93,
		93,  93,  93,  93,  93,  94,  94,  94,
		94,  94,  94,  94,  94,  94,  94,  94,
		94,  95,  95,  95,  95,  95,  95,  95,
		95,  95,  95,  95,  96,  96,  96,  96,
		96,  96,  96,  96,  96,  96,  96,  96,
		97,  97,  97,  97,  97,  97,  97,  97,
		97,  97,  97,  97,  98,  98,  98,  98,
		98,  98,  98,  98,  98,  98,  98,  98,
		99,  99,  99,  99,  99,  99,  99,  99,
		99,  99,  99,  99, 100, 100, 100, 100,
		100, 100, 100, 100, 100, 100, 100, 100,
		100, 101, 101, 101, 101, 101, 101, 101,
		102, 102, 103, 104, 104, 105, 105, 106,
		107, 107, 108, 108, 109, 109, 110, 111,
		111, 112, 112, 113, 113, 114, 114, 115,
		115, 116, 116, 117, 117, 118, 118, 119,
		119, 120, 120, 121, 121, 122, 122, 123,
		123, 124, 124, 125, 125, 126, 126, 127,
		127, 127, 128, 128, 129, 129, 130, 130,
		131, 131, 131, 132, 132, 133, 133, 134,
		134, 134, 135, 135, 136, 136, 136, 137,
		137, 138, 138, 139, 139, 139, 140, 140,
		141, 141, 141, 142, 142, 142, 143, 143,
		144, 144, 144, 145, 145, 145, 146, 146,
		147, 147, 147, 148, 148, 148, 149, 149,
		150, 150, 150, 151, 151, 151, 152, 152,
		152, 153, 153, 153, 154, 154, 154, 155,
		155, 155, 156, 156, 156, 157, 157, 157,
		158, 158, 158, 159, 159, 159, 160, 160,
		160, 161, 161, 161, 162, 162, 162, 163,
		163, 163, 164, 164, 164, 165, 165, 165,
		166, 166, 166, 166, 167, 167, 167, 168,
		168, 168, 169, 169, 169, 169, 170, 170,
		170, 171, 171, 171, 172, 172, 172, 172,
		173, 173, 173, 174, 174, 174, 174, 175,
		175, 175, 176, 176, 176, 176, 177, 177,
		177, 178, 178, 178, 178, 179, 179, 179,
		180, 180, 180, 180, 181, 181, 181, 181,
		182, 182, 182, 183, 183, 183, 183, 184,
		184, 184, 184, 185, 185, 185, 185, 186,
		186, 186, 187, 187, 187, 187, 188, 188,
		188, 188, 189, 189, 189, 189, 190, 190,
		190, 190, 191, 191, 191, 191, 192, 192,
		192, 192, 193, 193, 193, 193, 194, 194,
		194, 194, 195, 195, 195, 195, 196, 196,
		196, 196, 197, 197, 197, 197, 198, 198,
		198, 198, 199, 199, 199, 199, 199, 200,
		200, 200, 200, 201, 201, 201, 201, 202,
		202, 202, 202, 203, 203, 203, 203, 203,
		204, 204, 204, 204, 205, 205, 205, 205,
		206, 206, 206, 206, 206, 207, 207, 207,
		207, 208, 208, 208, 208, 208, 209, 209,
		209, 209, 210, 210, 210, 210, 210, 211,
		211, 211, 211, 212, 212, 212, 212, 212,
		213, 213, 213, 213, 213, 214, 214, 214,
		214, 215, 215, 215, 215, 215, 216, 216,
		216, 216, 216, 217, 217, 217, 217, 218,
		218, 218, 218, 218, 219, 219, 219, 219,
		219, 220, 220, 220, 220, 220, 221, 221,
		221, 221, 221, 222, 222, 222, 222, 222,
		223, 223, 223, 223, 224, 224, 224, 224,
		224, 225, 225, 225, 225, 225, 226, 226,
		226, 226, 226, 227, 227, 227, 227, 227,
		227, 228, 228, 228, 228, 228, 229, 229,
		229, 229, 229, 230, 230, 230, 230, 230,
		231, 231, 231, 231, 231, 232, 232, 232,
		232, 232, 233, 233, 233, 233, 233, 233,
		234, 234, 234, 234, 234, 235, 235, 235,
	},
};
#endif

void tegra_dc_clk_enable(struct tegra_dc *dc)
{
	clk_prepare_enable(dc->clk);
	tegra_dvfs_set_rate(dc->clk, dc->mode.pclk);
}

void tegra_dc_clk_disable(struct tegra_dc *dc)
{
	clk_disable_unprepare(dc->clk);
	tegra_dvfs_set_rate(dc->clk, 0);
}

void tegra_dc_get(struct tegra_dc *dc)
{
	tegra_dc_io_start(dc);

	/* extra reference to dc clk */
	clk_prepare_enable(dc->clk);
}

void tegra_dc_put(struct tegra_dc *dc)
{
	/* balance extra dc clk reference */
	clk_disable_unprepare(dc->clk);

	tegra_dc_io_end(dc);
}

void tegra_dc_hold_dc_out(struct tegra_dc *dc)
{
	if (1 == atomic_inc_return(&dc->holding)) {
		tegra_dc_get(dc);
		if (dc->out_ops && dc->out_ops->hold)
			dc->out_ops->hold(dc);
	}
}

void tegra_dc_release_dc_out(struct tegra_dc *dc)
{
	if (0 == atomic_dec_return(&dc->holding)) {
		if (dc->out_ops && dc->out_ops->release)
			dc->out_ops->release(dc);
		tegra_dc_put(dc);
	}
}

#define DUMP_REG(a) do {			\
	snprintf(buff, sizeof(buff), "%-32s\t%03x\t%08lx\n",  \
		 #a, a, tegra_dc_readl(dc, a));		      \
	print(data, buff);				      \
	} while (0)

static void _dump_regs(struct tegra_dc *dc, void *data,
		       void (* print)(void *data, const char *str))
{
	int i;
	char buff[256];
	const char winname[] = "ABCDHT";
	/* for above, see also: DC_CMD_DISPLAY_WINDOW_HEADER and DC_N_WINDOWS */

	/* If gated, quietly return. */
	if (!tegra_powergate_is_powered(dc->powergate_id))
		return;

	mutex_lock(&dc->lock);
	tegra_dc_get(dc);
	tegra_dc_writel(dc, WRITE_MUX_ACTIVE | READ_MUX_ACTIVE,
		DC_CMD_STATE_ACCESS);

	DUMP_REG(DC_CMD_DISPLAY_COMMAND_OPTION0);
	DUMP_REG(DC_CMD_DISPLAY_COMMAND);
	DUMP_REG(DC_CMD_SIGNAL_RAISE);
	DUMP_REG(DC_CMD_INT_STATUS);
	DUMP_REG(DC_CMD_INT_MASK);
	DUMP_REG(DC_CMD_INT_ENABLE);
	DUMP_REG(DC_CMD_INT_TYPE);
	DUMP_REG(DC_CMD_INT_POLARITY);
	DUMP_REG(DC_CMD_SIGNAL_RAISE1);
	DUMP_REG(DC_CMD_SIGNAL_RAISE2);
	DUMP_REG(DC_CMD_SIGNAL_RAISE3);
	DUMP_REG(DC_CMD_STATE_ACCESS);
	DUMP_REG(DC_CMD_STATE_CONTROL);
	DUMP_REG(DC_CMD_DISPLAY_WINDOW_HEADER);
	DUMP_REG(DC_CMD_REG_ACT_CONTROL);

	DUMP_REG(DC_DISP_DISP_SIGNAL_OPTIONS0);
	DUMP_REG(DC_DISP_DISP_SIGNAL_OPTIONS1);
	DUMP_REG(DC_DISP_DISP_WIN_OPTIONS);
	DUMP_REG(DC_DISP_MEM_HIGH_PRIORITY);
	DUMP_REG(DC_DISP_MEM_HIGH_PRIORITY_TIMER);
	DUMP_REG(DC_DISP_DISP_TIMING_OPTIONS);
	DUMP_REG(DC_DISP_REF_TO_SYNC);
	DUMP_REG(DC_DISP_SYNC_WIDTH);
	DUMP_REG(DC_DISP_BACK_PORCH);
	DUMP_REG(DC_DISP_DISP_ACTIVE);
	DUMP_REG(DC_DISP_FRONT_PORCH);
	DUMP_REG(DC_DISP_H_PULSE0_CONTROL);
	DUMP_REG(DC_DISP_H_PULSE0_POSITION_A);
	DUMP_REG(DC_DISP_H_PULSE0_POSITION_B);
	DUMP_REG(DC_DISP_H_PULSE0_POSITION_C);
	DUMP_REG(DC_DISP_H_PULSE0_POSITION_D);
	DUMP_REG(DC_DISP_H_PULSE1_CONTROL);
	DUMP_REG(DC_DISP_H_PULSE1_POSITION_A);
	DUMP_REG(DC_DISP_H_PULSE1_POSITION_B);
	DUMP_REG(DC_DISP_H_PULSE1_POSITION_C);
	DUMP_REG(DC_DISP_H_PULSE1_POSITION_D);
	DUMP_REG(DC_DISP_H_PULSE2_CONTROL);
	DUMP_REG(DC_DISP_H_PULSE2_POSITION_A);
	DUMP_REG(DC_DISP_H_PULSE2_POSITION_B);
	DUMP_REG(DC_DISP_H_PULSE2_POSITION_C);
	DUMP_REG(DC_DISP_H_PULSE2_POSITION_D);
	DUMP_REG(DC_DISP_V_PULSE0_CONTROL);
	DUMP_REG(DC_DISP_V_PULSE0_POSITION_A);
	DUMP_REG(DC_DISP_V_PULSE0_POSITION_B);
	DUMP_REG(DC_DISP_V_PULSE0_POSITION_C);
	DUMP_REG(DC_DISP_V_PULSE1_CONTROL);
	DUMP_REG(DC_DISP_V_PULSE1_POSITION_A);
	DUMP_REG(DC_DISP_V_PULSE1_POSITION_B);
	DUMP_REG(DC_DISP_V_PULSE1_POSITION_C);
	DUMP_REG(DC_DISP_V_PULSE2_CONTROL);
	DUMP_REG(DC_DISP_V_PULSE2_POSITION_A);
	DUMP_REG(DC_DISP_V_PULSE3_CONTROL);
	DUMP_REG(DC_DISP_V_PULSE3_POSITION_A);
	DUMP_REG(DC_DISP_M0_CONTROL);
	DUMP_REG(DC_DISP_M1_CONTROL);
	DUMP_REG(DC_DISP_DI_CONTROL);
	DUMP_REG(DC_DISP_PP_CONTROL);
	DUMP_REG(DC_DISP_PP_SELECT_A);
	DUMP_REG(DC_DISP_PP_SELECT_B);
	DUMP_REG(DC_DISP_PP_SELECT_C);
	DUMP_REG(DC_DISP_PP_SELECT_D);
	DUMP_REG(DC_DISP_DISP_CLOCK_CONTROL);
	DUMP_REG(DC_DISP_DISP_INTERFACE_CONTROL);
	DUMP_REG(DC_DISP_DISP_COLOR_CONTROL);
	DUMP_REG(DC_DISP_SHIFT_CLOCK_OPTIONS);
	DUMP_REG(DC_DISP_DATA_ENABLE_OPTIONS);
	DUMP_REG(DC_DISP_SERIAL_INTERFACE_OPTIONS);
	DUMP_REG(DC_DISP_LCD_SPI_OPTIONS);
#if !defined(CONFIG_TEGRA_DC_BLENDER_GEN2)
	DUMP_REG(DC_DISP_BORDER_COLOR);
#endif
	DUMP_REG(DC_DISP_COLOR_KEY0_LOWER);
	DUMP_REG(DC_DISP_COLOR_KEY0_UPPER);
	DUMP_REG(DC_DISP_COLOR_KEY1_LOWER);
	DUMP_REG(DC_DISP_COLOR_KEY1_UPPER);
	DUMP_REG(DC_DISP_CURSOR_FOREGROUND);
	DUMP_REG(DC_DISP_CURSOR_BACKGROUND);
	DUMP_REG(DC_DISP_CURSOR_START_ADDR);
	DUMP_REG(DC_DISP_CURSOR_START_ADDR_NS);
#if defined(CONFIG_ARCH_TEGRA_12x_SOC)
	DUMP_REG(DC_DISP_CURSOR_START_ADDR_HI);
	DUMP_REG(DC_DISP_CURSOR_START_ADDR_HI_NS);
#endif
	DUMP_REG(DC_DISP_CURSOR_POSITION);
	DUMP_REG(DC_DISP_CURSOR_POSITION_NS);
	DUMP_REG(DC_DISP_INIT_SEQ_CONTROL);
	DUMP_REG(DC_DISP_SPI_INIT_SEQ_DATA_A);
	DUMP_REG(DC_DISP_SPI_INIT_SEQ_DATA_B);
	DUMP_REG(DC_DISP_SPI_INIT_SEQ_DATA_C);
	DUMP_REG(DC_DISP_SPI_INIT_SEQ_DATA_D);
	DUMP_REG(DC_DISP_DC_MCCIF_FIFOCTRL);
	DUMP_REG(DC_DISP_MCCIF_DISPLAY0A_HYST);
	DUMP_REG(DC_DISP_MCCIF_DISPLAY0B_HYST);
	DUMP_REG(DC_DISP_MCCIF_DISPLAY0C_HYST);
#if defined(CONFIG_ARCH_TEGRA_2x_SOC) || defined(CONFIG_ARCH_TEGRA_3x_SOC)
	DUMP_REG(DC_DISP_MCCIF_DISPLAY1B_HYST);
#endif
	DUMP_REG(DC_DISP_DAC_CRT_CTRL);
	DUMP_REG(DC_DISP_DISP_MISC_CONTROL);
#if defined(CONFIG_ARCH_TEGRA_12x_SOC) && defined(CONFIG_TEGRA_DC_INTERLACE)
	DUMP_REG(DC_DISP_INTERLACE_CONTROL);
	DUMP_REG(DC_DISP_INTERLACE_FIELD2_REF_TO_SYNC);
	DUMP_REG(DC_DISP_INTERLACE_FIELD2_SYNC_WIDTH);
	DUMP_REG(DC_DISP_INTERLACE_FIELD2_BACK_PORCH);
	DUMP_REG(DC_DISP_INTERLACE_FIELD2_FRONT_PORCH);
	DUMP_REG(DC_DISP_INTERLACE_FIELD2_DISP_ACTIVE);
#endif
	for_each_set_bit(i, &dc->valid_windows, DC_N_WINDOWS) {
		print(data, "\n");
		snprintf(buff, sizeof(buff), "WINDOW %c:\n", winname[i]);
		print(data, buff);

		tegra_dc_writel(dc, WINDOW_A_SELECT << i,
				DC_CMD_DISPLAY_WINDOW_HEADER);
		DUMP_REG(DC_CMD_DISPLAY_WINDOW_HEADER);
		DUMP_REG(DC_WIN_WIN_OPTIONS);
		DUMP_REG(DC_WIN_BYTE_SWAP);
		DUMP_REG(DC_WIN_BUFFER_CONTROL);
		DUMP_REG(DC_WIN_COLOR_DEPTH);
		DUMP_REG(DC_WIN_POSITION);
		DUMP_REG(DC_WIN_SIZE);
		DUMP_REG(DC_WIN_PRESCALED_SIZE);
		DUMP_REG(DC_WIN_H_INITIAL_DDA);
		DUMP_REG(DC_WIN_V_INITIAL_DDA);
		DUMP_REG(DC_WIN_DDA_INCREMENT);
		DUMP_REG(DC_WIN_LINE_STRIDE);
#if defined(CONFIG_ARCH_TEGRA_2x_SOC) || defined(CONFIG_ARCH_TEGRA_3x_SOC)
		DUMP_REG(DC_WIN_BUF_STRIDE);
		DUMP_REG(DC_WIN_UV_BUF_STRIDE);
#endif
		DUMP_REG(DC_WIN_BLEND_NOKEY);
		DUMP_REG(DC_WIN_BLEND_1WIN);
		DUMP_REG(DC_WIN_BLEND_2WIN_X);
		DUMP_REG(DC_WIN_BLEND_2WIN_Y);
		DUMP_REG(DC_WIN_BLEND_3WIN_XY);
		DUMP_REG(DC_WINBUF_START_ADDR);
		DUMP_REG(DC_WINBUF_START_ADDR_U);
		DUMP_REG(DC_WINBUF_START_ADDR_V);
		DUMP_REG(DC_WINBUF_ADDR_H_OFFSET);
		DUMP_REG(DC_WINBUF_ADDR_V_OFFSET);
#if defined(CONFIG_ARCH_TEGRA_12x_SOC)
		DUMP_REG(DC_WINBUF_START_ADDR_HI);
		DUMP_REG(DC_WINBUF_START_ADDR_HI_U);
		DUMP_REG(DC_WINBUF_START_ADDR_HI_V);
		DUMP_REG(DC_WINBUF_START_ADDR_FIELD2);
		DUMP_REG(DC_WINBUF_START_ADDR_FIELD2_U);
		DUMP_REG(DC_WINBUF_START_ADDR_FIELD2_V);
		DUMP_REG(DC_WINBUF_START_ADDR_FIELD2_HI);
		DUMP_REG(DC_WINBUF_START_ADDR_FIELD2_HI_U);
		DUMP_REG(DC_WINBUF_START_ADDR_FIELD2_HI_V);
		DUMP_REG(DC_WINBUF_ADDR_H_OFFSET_FIELD2);
		DUMP_REG(DC_WINBUF_ADDR_V_OFFSET_FIELD2);
#endif
		DUMP_REG(DC_WINBUF_UFLOW_STATUS);
		DUMP_REG(DC_WIN_CSC_YOF);
		DUMP_REG(DC_WIN_CSC_KYRGB);
		DUMP_REG(DC_WIN_CSC_KUR);
		DUMP_REG(DC_WIN_CSC_KVR);
		DUMP_REG(DC_WIN_CSC_KUG);
		DUMP_REG(DC_WIN_CSC_KVG);
		DUMP_REG(DC_WIN_CSC_KUB);
		DUMP_REG(DC_WIN_CSC_KVB);
	}

	DUMP_REG(DC_CMD_DISPLAY_POWER_CONTROL);
	DUMP_REG(DC_COM_PIN_OUTPUT_ENABLE2);
	DUMP_REG(DC_COM_PIN_OUTPUT_POLARITY2);
	DUMP_REG(DC_COM_PIN_OUTPUT_DATA2);
	DUMP_REG(DC_COM_PIN_INPUT_ENABLE2);
	DUMP_REG(DC_COM_PIN_OUTPUT_SELECT5);
	DUMP_REG(DC_DISP_DISP_SIGNAL_OPTIONS0);
	DUMP_REG(DC_DISP_M1_CONTROL);
	DUMP_REG(DC_COM_PM1_CONTROL);
	DUMP_REG(DC_COM_PM1_DUTY_CYCLE);
	DUMP_REG(DC_DISP_SD_CONTROL);
#ifdef CONFIG_TEGRA_DC_CMU
	DUMP_REG(DC_COM_CMU_CSC_KRR);
	DUMP_REG(DC_COM_CMU_CSC_KGR);
	DUMP_REG(DC_COM_CMU_CSC_KBR);
	DUMP_REG(DC_COM_CMU_CSC_KRG);
	DUMP_REG(DC_COM_CMU_CSC_KGG);
	DUMP_REG(DC_COM_CMU_CSC_KBR);
	DUMP_REG(DC_COM_CMU_CSC_KRB);
	DUMP_REG(DC_COM_CMU_CSC_KGB);
	DUMP_REG(DC_COM_CMU_CSC_KBB);
#endif

	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);
}

#undef DUMP_REG

#ifdef DEBUG
static void dump_regs_print(void *data, const char *str)
{
	struct tegra_dc *dc = data;
	dev_dbg(&dc->ndev->dev, "%s", str);
}

static void dump_regs(struct tegra_dc *dc)
{
	_dump_regs(dc, dc, dump_regs_print);
}
#else /* !DEBUG */

static void dump_regs(struct tegra_dc *dc) {}

#endif /* DEBUG */

#ifdef CONFIG_DEBUG_FS

static void dbg_regs_print(void *data, const char *str)
{
	struct seq_file *s = data;

	seq_printf(s, "%s", str);
}

#undef DUMP_REG

static int dbg_dc_show(struct seq_file *s, void *unused)
{
	struct tegra_dc *dc = s->private;

	_dump_regs(dc, s, dbg_regs_print);

	return 0;
}


static int dbg_dc_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_dc_show, inode->i_private);
}

static const struct file_operations regs_fops = {
	.open		= dbg_dc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int dbg_dc_mode_show(struct seq_file *s, void *unused)
{
	struct tegra_dc *dc = s->private;
	struct tegra_dc_mode *m;

	mutex_lock(&dc->lock);
	m = &dc->mode;
	seq_printf(s,
		"pclk: %d\n"
		"h_ref_to_sync: %d\n"
		"v_ref_to_sync: %d\n"
		"h_sync_width: %d\n"
		"v_sync_width: %d\n"
		"h_back_porch: %d\n"
		"v_back_porch: %d\n"
		"h_active: %d\n"
		"v_active: %d\n"
		"h_front_porch: %d\n"
		"v_front_porch: %d\n"
		"stereo_mode: %d\n",
		m->pclk, m->h_ref_to_sync, m->v_ref_to_sync,
		m->h_sync_width, m->v_sync_width,
		m->h_back_porch, m->v_back_porch,
		m->h_active, m->v_active,
		m->h_front_porch, m->v_front_porch,
		m->stereo_mode);
	mutex_unlock(&dc->lock);
	return 0;
}

static int dbg_dc_mode_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_dc_mode_show, inode->i_private);
}

static const struct file_operations mode_fops = {
	.open		= dbg_dc_mode_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int dbg_dc_stats_show(struct seq_file *s, void *unused)
{
	struct tegra_dc *dc = s->private;

	mutex_lock(&dc->lock);
	seq_printf(s,
		"underflows: %llu\n"
		"underflows_a: %llu\n"
		"underflows_b: %llu\n"
		"underflows_c: %llu\n",
		dc->stats.underflows,
		dc->stats.underflows_a,
		dc->stats.underflows_b,
		dc->stats.underflows_c);
#if defined(CONFIG_ARCH_TEGRA_14x_SOC) || defined(CONFIG_ARCH_TEGRA_12x_SOC)
	seq_printf(s,
		"underflows_d: %llu\n"
		"underflows_h: %llu\n"
		"underflows_t: %llu\n",
		dc->stats.underflows_d,
		dc->stats.underflows_h,
		dc->stats.underflows_t);
#endif
	mutex_unlock(&dc->lock);

	return 0;
}

static int dbg_dc_stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_dc_stats_show, inode->i_private);
}

static int dbg_dc_event_inject_show(struct seq_file *s, void *unused)
{
	return 0;
}

static ssize_t dbg_dc_event_inject_write(struct file *file,
	const char __user *addr, size_t len, loff_t *pos)
{
	struct seq_file *m = file->private_data; /* single_open() initialized */
	struct tegra_dc *dc = m ? m->private : NULL;
	long event;
	int ret;

	if (!dc)
		return -EINVAL;

	ret = kstrtol_from_user(addr, len, 10, &event);
	if (ret < 0)
		return ret;

	/*
	 * ADF has two seperate events for hotplug connect and disconnect.
	 * We map event 0x0, and 0x1 for them accordingly.  For DC_EXT,
	 * both events map to HOTPLUG.
	 */
#ifdef CONFIG_ADF_TEGRA
	if (event == 0x0)
		tegra_adf_process_hotplug_connected(dc->adf, NULL);
	else if (event == 0x1)
		tegra_adf_process_hotplug_disconnected(dc->adf);
	else if (event == 0x2)
		tegra_adf_process_bandwidth_renegotiate(dc->adf, 0);
	else {
		dev_err(&dc->ndev->dev, "Unknown event 0x%lx\n", event);
		return -EINVAL; /* unknown event number */
	}
#endif
#ifdef CONFIG_TEGRA_DC_EXTENSIONS
	if (event == 0x0 || event == 0x1) /* TEGRA_DC_EXT_EVENT_HOTPLUG */
		tegra_dc_ext_process_hotplug(dc->ndev->id);
	else if (event == 0x2) /* TEGRA_DC_EXT_EVENT_BANDWIDTH_DEC */
		tegra_dc_ext_process_bandwidth_renegotiate(
				dc->ndev->id, NULL);
	else {
		dev_err(&dc->ndev->dev, "Unknown event 0x%lx\n", event);
		return -EINVAL; /* unknown event number */
	}
#endif
	return len;
}

static const struct file_operations stats_fops = {
	.open		= dbg_dc_stats_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int dbg_dc_event_inject_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_dc_event_inject_show, inode->i_private);
}

static const struct file_operations event_inject_fops = {
	.open		= dbg_dc_event_inject_open,
	.read		= seq_read,
	.write		= dbg_dc_event_inject_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void tegra_dc_remove_debugfs(struct tegra_dc *dc)
{
	if (dc->debugdir)
		debugfs_remove_recursive(dc->debugdir);
	dc->debugdir = NULL;
}

static void tegra_dc_create_debugfs(struct tegra_dc *dc)
{
	struct dentry *retval;

	dc->debugdir = debugfs_create_dir(dev_name(&dc->ndev->dev), NULL);
	if (!dc->debugdir)
		goto remove_out;

	retval = debugfs_create_file("regs", S_IRUGO, dc->debugdir, dc,
		&regs_fops);
	if (!retval)
		goto remove_out;

	retval = debugfs_create_file("mode", S_IRUGO, dc->debugdir, dc,
		&mode_fops);
	if (!retval)
		goto remove_out;

	retval = debugfs_create_file("stats", S_IRUGO, dc->debugdir, dc,
		&stats_fops);
	if (!retval)
		goto remove_out;

	retval = debugfs_create_file("event_inject", S_IRUGO, dc->debugdir, dc,
		&event_inject_fops);
	if (!retval)
		goto remove_out;

	return;
remove_out:
	dev_err(&dc->ndev->dev, "could not create debugfs\n");
	tegra_dc_remove_debugfs(dc);
}

#else /* !CONFIG_DEBUGFS */
static inline void tegra_dc_create_debugfs(struct tegra_dc *dc) { };
static inline void tegra_dc_remove_debugfs(struct tegra_dc *dc) { };
#endif /* CONFIG_DEBUGFS */

unsigned long tegra_dc_poll_register(struct tegra_dc *dc, u32 reg, u32 mask,
		u32 exp_val, u32 poll_interval_us, u32 timeout_ms)
{
	unsigned long timeout_jf = jiffies + msecs_to_jiffies(timeout_ms);
	u32 reg_val = 0;

	do {
		usleep_range(poll_interval_us, poll_interval_us << 1);
		reg_val = tegra_dc_readl(dc, reg);
	} while (((reg_val & mask) != exp_val) &&
		time_after(timeout_jf, jiffies));

	if ((reg_val & mask) == exp_val)
		return 0;       /* success */
	dev_err(&dc->ndev->dev,
		"dc_poll_register 0x%x: timeout\n", reg);
	return jiffies - timeout_jf + 1;
}

static int tegra_dc_set(struct tegra_dc *dc, int index)
{
	int ret = 0;

	mutex_lock(&tegra_dc_lock);
	if (index >= TEGRA_MAX_DC) {
		ret = -EINVAL;
		goto out;
	}

	if (dc != NULL && tegra_dcs[index] != NULL) {
		ret = -EBUSY;
		goto out;
	}

	tegra_dcs[index] = dc;

out:
	mutex_unlock(&tegra_dc_lock);

	return ret;
}

unsigned int tegra_dc_has_multiple_dc(void)
{
	unsigned int idx;
	unsigned int cnt = 0;
	struct tegra_dc *dc;

	mutex_lock(&tegra_dc_lock);
	for (idx = 0; idx < TEGRA_MAX_DC; idx++)
		cnt += ((dc = tegra_dcs[idx]) != NULL && dc->enabled) ? 1 : 0;
	mutex_unlock(&tegra_dc_lock);

	return (cnt > 1);
}

/* get the stride size of a window.
 * return: stride size in bytes for window win. or 0 if unavailble. */
int tegra_dc_get_stride(struct tegra_dc *dc, unsigned win)
{
	u32 stride;

	if (!dc->enabled)
		return 0;
	BUG_ON(win > DC_N_WINDOWS);
	mutex_lock(&dc->lock);
	tegra_dc_get(dc);
	tegra_dc_writel(dc, WINDOW_A_SELECT << win,
		DC_CMD_DISPLAY_WINDOW_HEADER);
	stride = tegra_dc_readl(dc, DC_WIN_LINE_STRIDE);
	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);
	return GET_LINE_STRIDE(stride);
}
EXPORT_SYMBOL(tegra_dc_get_stride);

struct tegra_dc *tegra_dc_get_dc(unsigned idx)
{
	if (idx < TEGRA_MAX_DC)
		return tegra_dcs[idx];
	else
		return NULL;
}
EXPORT_SYMBOL(tegra_dc_get_dc);

struct tegra_dc_win *tegra_dc_get_window(struct tegra_dc *dc, unsigned win)
{
	if (win >= DC_N_WINDOWS || !test_bit(win, &dc->valid_windows))
		return NULL;

	return &dc->windows[win];
}
EXPORT_SYMBOL(tegra_dc_get_window);

bool tegra_dc_get_connected(struct tegra_dc *dc)
{
	return dc->connected;
}
EXPORT_SYMBOL(tegra_dc_get_connected);

bool tegra_dc_hpd(struct tegra_dc *dc)
{
	int sense;
	int level;
	int hpd;

	if (WARN_ON(!dc || !dc->out))
		return false;

	if (dc->out->hotplug_state != 0) {
		if (dc->out->hotplug_state == 1) /* force on */
			return true;
		if (dc->out->hotplug_state == -1) /* force off */
			return false;
	}
	level = gpio_get_value_cansleep(dc->out->hotplug_gpio);

	sense = dc->out->flags & TEGRA_DC_OUT_HOTPLUG_MASK;

	hpd = (sense == TEGRA_DC_OUT_HOTPLUG_HIGH && level) ||
		(sense == TEGRA_DC_OUT_HOTPLUG_LOW && !level);

	if (dc->out->hotplug_report)
		dc->out->hotplug_report(hpd);

	return hpd;
}
EXPORT_SYMBOL(tegra_dc_hpd);

static void tegra_dc_set_scaling_filter(struct tegra_dc *dc)
{
	unsigned i;
	unsigned v0 = 128;
	unsigned v1 = 0;

	/* linear horizontal and vertical filters */
	for (i = 0; i < 16; i++) {
		tegra_dc_writel(dc, (v1 << 16) | (v0 << 8),
				DC_WIN_H_FILTER_P(i));

		tegra_dc_writel(dc, v0,
				DC_WIN_V_FILTER_P(i));
		v0 -= 8;
		v1 += 8;
	}
}

#ifdef CONFIG_TEGRA_DC_CMU
static void tegra_dc_cache_cmu(struct tegra_dc *dc,
					struct tegra_dc_cmu *src_cmu)
{
	if (&dc->cmu != src_cmu) /* ignore if it would require memmove() */
		memcpy(&dc->cmu, src_cmu, sizeof(*src_cmu));
	dc->cmu_dirty = true;
}

static void tegra_dc_set_cmu(struct tegra_dc *dc, struct tegra_dc_cmu *cmu)
{
	u32 val;
	u32 i;

	for (i = 0; i < 256; i++) {
		val = LUT1_ADDR(i) | LUT1_DATA(cmu->lut1[i]);
		tegra_dc_writel(dc, val, DC_COM_CMU_LUT1);
	}

	tegra_dc_writel(dc, cmu->csc.krr, DC_COM_CMU_CSC_KRR);
	tegra_dc_writel(dc, cmu->csc.kgr, DC_COM_CMU_CSC_KGR);
	tegra_dc_writel(dc, cmu->csc.kbr, DC_COM_CMU_CSC_KBR);
	tegra_dc_writel(dc, cmu->csc.krg, DC_COM_CMU_CSC_KRG);
	tegra_dc_writel(dc, cmu->csc.kgg, DC_COM_CMU_CSC_KGG);
	tegra_dc_writel(dc, cmu->csc.kbg, DC_COM_CMU_CSC_KBG);
	tegra_dc_writel(dc, cmu->csc.krb, DC_COM_CMU_CSC_KRB);
	tegra_dc_writel(dc, cmu->csc.kgb, DC_COM_CMU_CSC_KGB);
	tegra_dc_writel(dc, cmu->csc.kbb, DC_COM_CMU_CSC_KBB);

	for (i = 0; i < 960; i++) {
		val = LUT2_ADDR(i) | LUT1_DATA(cmu->lut2[i]);
		tegra_dc_writel(dc, val, DC_COM_CMU_LUT2);
	}

	dc->cmu_dirty = false;
}

static void _tegra_dc_update_cmu(struct tegra_dc *dc, struct tegra_dc_cmu *cmu)
{
	u32 val;

	if (!dc->cmu_enabled)
		return;

	tegra_dc_cache_cmu(dc, cmu);

	if (dc->cmu_dirty) {
		/* Disable CMU to avoid programming it while it is in use */
		val = tegra_dc_readl(dc, DC_DISP_DISP_COLOR_CONTROL);
		if (val & CMU_ENABLE) {
			val &= ~CMU_ENABLE;
			tegra_dc_writel(dc, val,
					DC_DISP_DISP_COLOR_CONTROL);
			val = GENERAL_ACT_REQ;
			tegra_dc_writel(dc, val, DC_CMD_STATE_CONTROL);
			/*TODO: Sync up with vsync */
			mdelay(20);
		}
		dev_dbg(&dc->ndev->dev, "updating CMU cmu_dirty=%d\n",
			dc->cmu_dirty);

		tegra_dc_set_cmu(dc, &dc->cmu);
	}
}

int tegra_dc_update_cmu(struct tegra_dc *dc, struct tegra_dc_cmu *cmu)
{
	mutex_lock(&dc->lock);
	if (!dc->enabled) {
		mutex_unlock(&dc->lock);
		return 0;
	}

	tegra_dc_get(dc);

	_tegra_dc_update_cmu(dc, cmu);
	tegra_dc_set_color_control(dc);

	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);

	return 0;
}
EXPORT_SYMBOL(tegra_dc_update_cmu);

static struct tegra_dc_cmu *tegra_dc_get_cmu(struct tegra_dc *dc)
{
	if (dc->pdata->cmu)
		return dc->pdata->cmu;
	else if (dc->out->type == TEGRA_DC_OUT_HDMI)
		return &default_limited_cmu;
	else
		return &default_cmu;
}

void tegra_dc_cmu_enable(struct tegra_dc *dc, bool cmu_enable)
{
	dc->cmu_enabled = cmu_enable;
	tegra_dc_update_cmu(dc, tegra_dc_get_cmu(dc));
}
#else
#define tegra_dc_cache_cmu(dc, src_cmu)
#define tegra_dc_set_cmu(dc, cmu)
#define tegra_dc_update_cmu(dc, cmu)
#endif

/* disable_irq() blocks until handler completes, calling this function while
 * holding dc->lock can deadlock. */
static inline void disable_dc_irq(const struct tegra_dc *dc)
{
	disable_irq(dc->irq);
}

u32 tegra_dc_get_syncpt_id(const struct tegra_dc *dc, int i)
{
	return dc->syncpt[i].id;
}
EXPORT_SYMBOL(tegra_dc_get_syncpt_id);

u32 tegra_dc_incr_syncpt_max(struct tegra_dc *dc, int i)
{
	u32 max;

	mutex_lock(&dc->lock);
	tegra_dc_get(dc);
	max = nvhost_syncpt_incr_max_ext(dc->ndev,
		dc->syncpt[i].id, ((dc->enabled) ? 1 : 0));
	dc->syncpt[i].max = max;
	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);

	return max;
}

void tegra_dc_incr_syncpt_min(struct tegra_dc *dc, int i, u32 val)
{
	mutex_lock(&dc->lock);

	tegra_dc_get(dc);
	while (dc->syncpt[i].min < val) {
		dc->syncpt[i].min++;
		nvhost_syncpt_cpu_incr_ext(dc->ndev, dc->syncpt[i].id);
		}
	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);
}

struct sync_fence *tegra_dc_create_fence(struct tegra_dc *dc, int i, u32 val)
{
	struct nvhost_ctrl_sync_fence_info syncpt;
	u32 id = tegra_dc_get_syncpt_id(dc, i);

	syncpt.id = id;
	syncpt.thresh = val;
	return nvhost_sync_create_fence(
			to_platform_device(dc->ndev->dev.parent),
			&syncpt, 1, dev_name(&dc->ndev->dev));
}

void
tegra_dc_config_pwm(struct tegra_dc *dc, struct tegra_dc_pwm_params *cfg)
{
	unsigned int ctrl;
	unsigned long out_sel;
	unsigned long cmd_state;

	mutex_lock(&dc->lock);
	if (!dc->enabled) {
		mutex_unlock(&dc->lock);
		return;
	}

	tegra_dc_get(dc);

	ctrl = ((cfg->period << PM_PERIOD_SHIFT) |
		(cfg->clk_div << PM_CLK_DIVIDER_SHIFT) |
		cfg->clk_select);

	/* The new value should be effected immediately */
	cmd_state = tegra_dc_readl(dc, DC_CMD_STATE_ACCESS);
	tegra_dc_writel(dc, (cmd_state | (1 << 2)), DC_CMD_STATE_ACCESS);

	switch (cfg->which_pwm) {
	case TEGRA_PWM_PM0:
		/* Select the LM0 on PM0 */
		out_sel = tegra_dc_readl(dc, DC_COM_PIN_OUTPUT_SELECT5);
		out_sel &= ~(7 << 0);
		out_sel |= (3 << 0);
		tegra_dc_writel(dc, out_sel, DC_COM_PIN_OUTPUT_SELECT5);
		tegra_dc_writel(dc, ctrl, DC_COM_PM0_CONTROL);
		tegra_dc_writel(dc, cfg->duty_cycle, DC_COM_PM0_DUTY_CYCLE);
		break;
	case TEGRA_PWM_PM1:
		/* Select the LM1 on PM1 */
		out_sel = tegra_dc_readl(dc, DC_COM_PIN_OUTPUT_SELECT5);
		out_sel &= ~(7 << 4);
		out_sel |= (3 << 4);
		tegra_dc_writel(dc, out_sel, DC_COM_PIN_OUTPUT_SELECT5);
		tegra_dc_writel(dc, ctrl, DC_COM_PM1_CONTROL);
		tegra_dc_writel(dc, cfg->duty_cycle, DC_COM_PM1_DUTY_CYCLE);
		break;
	default:
		dev_err(&dc->ndev->dev, "Error: Need which_pwm\n");
		break;
	}
	tegra_dc_writel(dc, cmd_state, DC_CMD_STATE_ACCESS);
	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);
}
EXPORT_SYMBOL(tegra_dc_config_pwm);

void tegra_dc_set_out_pin_polars(struct tegra_dc *dc,
				const struct tegra_dc_out_pin *pins,
				const unsigned int n_pins)
{
	unsigned int i;

	int name;
	int pol;

	u32 pol1, pol3;

	u32 set1, unset1;
	u32 set3, unset3;

	set1 = set3 = unset1 = unset3 = 0;

	for (i = 0; i < n_pins; i++) {
		name = (pins + i)->name;
		pol  = (pins + i)->pol;

		/* set polarity by name */
		switch (name) {
		case TEGRA_DC_OUT_PIN_DATA_ENABLE:
			if (pol == TEGRA_DC_OUT_PIN_POL_LOW)
				set3 |= LSPI_OUTPUT_POLARITY_LOW;
			else
				unset3 |= LSPI_OUTPUT_POLARITY_LOW;
			break;
		case TEGRA_DC_OUT_PIN_H_SYNC:
			if (pol == TEGRA_DC_OUT_PIN_POL_LOW)
				set1 |= LHS_OUTPUT_POLARITY_LOW;
			else
				unset1 |= LHS_OUTPUT_POLARITY_LOW;
			break;
		case TEGRA_DC_OUT_PIN_V_SYNC:
			if (pol == TEGRA_DC_OUT_PIN_POL_LOW)
				set1 |= LVS_OUTPUT_POLARITY_LOW;
			else
				unset1 |= LVS_OUTPUT_POLARITY_LOW;
			break;
		case TEGRA_DC_OUT_PIN_PIXEL_CLOCK:
			if (pol == TEGRA_DC_OUT_PIN_POL_LOW)
				set1 |= LSC0_OUTPUT_POLARITY_LOW;
			else
				unset1 |= LSC0_OUTPUT_POLARITY_LOW;
			break;
		default:
			printk("Invalid argument in function %s\n",
			       __FUNCTION__);
			break;
		}
	}

	pol1 = DC_COM_PIN_OUTPUT_POLARITY1_INIT_VAL;
	pol3 = DC_COM_PIN_OUTPUT_POLARITY3_INIT_VAL;

	pol1 |= set1;
	pol1 &= ~unset1;

	pol3 |= set3;
	pol3 &= ~unset3;

	tegra_dc_writel(dc, pol1, DC_COM_PIN_OUTPUT_POLARITY1);
	tegra_dc_writel(dc, pol3, DC_COM_PIN_OUTPUT_POLARITY3);
}

static struct tegra_dc_mode *tegra_dc_get_override_mode(struct tegra_dc *dc)
{
	if (dc->out->type == TEGRA_DC_OUT_RGB ||
		dc->out->type == TEGRA_DC_OUT_HDMI ||
		dc->out->type == TEGRA_DC_OUT_DSI)
		return override_disp_mode[dc->out->type].pclk ?
			&override_disp_mode[dc->out->type] : NULL;
	else
		return NULL;
}

static int tegra_dc_set_out(struct tegra_dc *dc, struct tegra_dc_out *out)
{
	struct tegra_dc_mode *mode;
	int err = 0;

	dc->out = out;
	mode = tegra_dc_get_override_mode(dc);

	if (mode) {
		tegra_dc_set_mode(dc, mode);

		/*
		 * Bootloader should and should only pass disp_params if
		 * it has initialized display controller.  Whenever we see
		 * override modes, we should skip things cause display resets.
		 */
		dev_info(&dc->ndev->dev, "Bootloader disp_param detected. "
				"Detected mode: %dx%d (on %dx%dmm) pclk=%d\n",
				dc->mode.h_active, dc->mode.v_active,
				dc->out->h_size, dc->out->v_size,
				dc->mode.pclk);
		dc->initialized = true;

#ifdef CONFIG_TEGRA_DC_CMU
		/*
		 * If the bootloader already set the mode, assume the CMU
		 * parameters are also correctly set. It would be better to
		 * read them, but unfortunately there is no reliable and
		 * flicker-free way to do this!
		 */
		tegra_dc_cache_cmu(dc, tegra_dc_get_cmu(dc));
#endif
	} else if (out->n_modes > 0)
		tegra_dc_set_mode(dc, &dc->out->modes[0]);

	switch (out->type) {
	case TEGRA_DC_OUT_RGB:
		dc->out_ops = &tegra_dc_rgb_ops;
		break;

	case TEGRA_DC_OUT_HDMI:
		dc->out_ops = &tegra_dc_hdmi_ops;
		break;

	case TEGRA_DC_OUT_DSI:
		dc->out_ops = &tegra_dc_dsi_ops;
		break;

#ifdef CONFIG_TEGRA_DP
	case TEGRA_DC_OUT_DP:
		dc->out_ops = &tegra_dc_dp_ops;
		break;
#ifdef CONFIG_TEGRA_NVSR
	case TEGRA_DC_OUT_NVSR_DP:
		dc->out_ops = &tegra_dc_nvsr_ops;
		break;
#endif
#endif
#ifdef CONFIG_TEGRA_LVDS
	case TEGRA_DC_OUT_LVDS:
		dc->out_ops = &tegra_dc_lvds_ops;
		break;
#endif
	default:
		dc->out_ops = NULL;
		break;
	}

	if (dc->out_ops && dc->out_ops->init) {
		err = dc->out_ops->init(dc);
		if (err < 0) {
			dc->out = NULL;
			dc->out_ops = NULL;
			dev_err(&dc->ndev->dev,
				"Error: out->type:%d out_ops->init() failed\n",
				out->type);
			return err;
		}
	}

	return err;
}

/* returns on error: -EINVAL
 * on success: TEGRA_DC_OUT_RGB, TEGRA_DC_OUT_HDMI, or TEGRA_DC_OUT_DSI. */
int tegra_dc_get_out(const struct tegra_dc *dc)
{
	if (dc && dc->out)
		return dc->out->type;
	return -EINVAL;
}

unsigned tegra_dc_get_out_height(const struct tegra_dc *dc)
{
	if (dc->out)
		return dc->out->height;
	else
		return 0;
}
EXPORT_SYMBOL(tegra_dc_get_out_height);

unsigned tegra_dc_get_out_width(const struct tegra_dc *dc)
{
	if (dc->out)
		return dc->out->width;
	else
		return 0;
}
EXPORT_SYMBOL(tegra_dc_get_out_width);

unsigned tegra_dc_get_out_max_pixclock(const struct tegra_dc *dc)
{
	if (dc && dc->out)
		return dc->out->max_pixclock;
	else
		return 0;
}
EXPORT_SYMBOL(tegra_dc_get_out_max_pixclock);

void tegra_dc_enable_crc(struct tegra_dc *dc)
{
	u32 val;

	mutex_lock(&dc->lock);
	tegra_dc_get(dc);

	val = CRC_ALWAYS_ENABLE | CRC_INPUT_DATA_ACTIVE_DATA |
		CRC_ENABLE_ENABLE;
	tegra_dc_writel(dc, val, DC_COM_CRC_CONTROL);
	tegra_dc_writel(dc, GENERAL_ACT_REQ, DC_CMD_STATE_CONTROL);
	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);

	/* Register a client of frame_end interrupt */
	tegra_dc_config_frame_end_intr(dc, true);
}

void tegra_dc_disable_crc(struct tegra_dc *dc)
{
	/* Unregister a client of frame_end interrupt */
	tegra_dc_config_frame_end_intr(dc, false);

	mutex_lock(&dc->lock);
	tegra_dc_get(dc);
	tegra_dc_writel(dc, 0x0, DC_COM_CRC_CONTROL);
	tegra_dc_writel(dc, GENERAL_ACT_REQ, DC_CMD_STATE_CONTROL);

	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);
}

u32 tegra_dc_read_checksum_latched(struct tegra_dc *dc)
{
	int crc = 0;

	if (!dc) {
		pr_err("Failed to get dc: NULL parameter.\n");
		goto crc_error;
	}

	/* If gated quitely return */
	if (!tegra_dc_is_powered(dc))
		return 0;

	INIT_COMPLETION(dc->crc_complete);
	if (dc->crc_pending &&
	    wait_for_completion_interruptible(&dc->crc_complete)) {
		pr_err("CRC read interrupted.\n");
		goto crc_error;
	}

	mutex_lock(&dc->lock);
	tegra_dc_get(dc);
	crc = tegra_dc_readl(dc, DC_COM_CRC_CHECKSUM_LATCHED);
	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);
crc_error:
	return crc;
}

static bool tegra_dc_windows_are_dirty(struct tegra_dc *dc)
{
	u32 val;

	if (tegra_platform_is_linsim())
		return false;

	val = tegra_dc_readl(dc, DC_CMD_STATE_CONTROL);
	if (val & WIN_ALL_ACT_REQ)
		return true;

	return false;
}

static inline void enable_dc_irq(const struct tegra_dc *dc)
{
	if (tegra_platform_is_fpga())
		/* Always disable DC interrupts on FPGA. */
		disable_irq(dc->irq);
	else
		enable_irq(dc->irq);
}

bool tegra_dc_has_vsync(struct tegra_dc *dc)
{
	return !!(dc->out->flags & TEGRA_DC_OUT_ONE_SHOT_MODE);
}

int tegra_dc_wait_for_vsync(struct tegra_dc *dc)
{
	int ret = -ENOTTY;

	/* Generally vsync comes at 60Hz (~16.67ms per cycle).
	 * 2 time periods should be good enough for the timeout.
	 * We add some margin here for the default timeout value.
	 */
	unsigned long timeout_ms = 40;

	if (dc->out->type == TEGRA_DC_OUT_DSI &&
		dc->out->dsi->rated_refresh_rate != 0)
		timeout_ms = 2 * DIV_ROUND_UP(1000,
			 dc->out->dsi->rated_refresh_rate);

	mutex_lock(&dc->one_shot_lp_lock);

	if (!(dc->out->flags & TEGRA_DC_OUT_ONE_SHOT_MODE) || !dc->enabled) {
		mutex_unlock(&dc->one_shot_lp_lock);
		return ret;
	}

	tegra_dc_get(dc);
	if (dc->out_ops && dc->out_ops->hold)
		dc->out_ops->hold(dc);

	/*
	 * Logic is as follows
	 * a) Indicate we need a vblank.
	 * b) Wait for completion to be signalled from isr.
	 * c) Initialize completion for next iteration.
	 */

	dc->out->user_needs_vblank = true;

	mutex_lock(&dc->lock);
	tegra_dc_unmask_interrupt(dc, MSF_INT);
	mutex_unlock(&dc->lock);

	ret = wait_for_completion_interruptible_timeout(
		&dc->out->user_vblank_comp, msecs_to_jiffies(timeout_ms));
	init_completion(&dc->out->user_vblank_comp);

	mutex_lock(&dc->lock);
	tegra_dc_mask_interrupt(dc, MSF_INT);
	mutex_unlock(&dc->lock);

	if (dc->out_ops && dc->out_ops->release)
		dc->out_ops->release(dc);
	tegra_dc_put(dc);

	mutex_unlock(&dc->one_shot_lp_lock);
	return ret;
}

static int _tegra_dc_config_frame_end_intr(struct tegra_dc *dc, bool enable)
{
	tegra_dc_io_start(dc);
	if (enable) {
		atomic_inc(&dc->frame_end_ref);
		tegra_dc_unmask_interrupt(dc, FRAME_END_INT);
	} else if (!atomic_dec_return(&dc->frame_end_ref))
		tegra_dc_mask_interrupt(dc, FRAME_END_INT);
	tegra_dc_io_end(dc);

	return 0;
}

int _tegra_dc_wait_for_frame_end(struct tegra_dc *dc,
	u32 timeout_ms)
{
	int ret;

	INIT_COMPLETION(dc->frame_end_complete);

	tegra_dc_get(dc);

	tegra_dc_flush_interrupt(dc, FRAME_END_INT);
	/* unmask frame end interrupt */
	_tegra_dc_config_frame_end_intr(dc, true);

	ret = wait_for_completion_interruptible_timeout(
			&dc->frame_end_complete,
			msecs_to_jiffies(timeout_ms));

	_tegra_dc_config_frame_end_intr(dc, false);

	tegra_dc_put(dc);

	return ret;
}

static void tegra_dc_prism_update_backlight(struct tegra_dc *dc)
{
	/* Do the actual brightness update outside of the mutex dc->lock */
	if (dc->out->sd_settings && !dc->out->sd_settings->bl_device &&
		dc->out->sd_settings->bl_device_name) {
		char *bl_device_name =
			dc->out->sd_settings->bl_device_name;
		dc->out->sd_settings->bl_device =
			get_backlight_device_by_name(bl_device_name);
	}

	if (dc->out->sd_settings && dc->out->sd_settings->bl_device) {
		struct backlight_device *bl = dc->out->sd_settings->bl_device;
		backlight_update_status(bl);
	}
}

void tegra_dc_vsync_enable(struct tegra_dc *dc)
{
	mutex_lock(&dc->lock);
	set_bit(V_BLANK_USER, &dc->vblank_ref_count);
	tegra_dc_unmask_interrupt(dc, V_BLANK_INT);
	mutex_unlock(&dc->lock);
}

void tegra_dc_vsync_disable(struct tegra_dc *dc)
{
	mutex_lock(&dc->lock);
	clear_bit(V_BLANK_USER, &dc->vblank_ref_count);
	if (!dc->vblank_ref_count)
		tegra_dc_mask_interrupt(dc, V_BLANK_INT);
	mutex_unlock(&dc->lock);
}

static void tegra_dc_vblank(struct work_struct *work)
{
	struct tegra_dc *dc = container_of(work, struct tegra_dc, vblank_work);
	bool nvsd_updated = false;

	mutex_lock(&dc->lock);

	if (!dc->enabled) {
		mutex_unlock(&dc->lock);
		return;
	}

	tegra_dc_get(dc);

	/* Clear the V_BLANK_FLIP bit of vblank ref-count if update is clean. */
	if (!tegra_dc_windows_are_dirty(dc))
		clear_bit(V_BLANK_FLIP, &dc->vblank_ref_count);

	/* Update the SD brightness */
	if (dc->out->sd_settings && !dc->out->sd_settings->use_vpulse2) {
		nvsd_updated = nvsd_update_brightness(dc);
		/* Ref-count vblank if nvsd is on-going. Otherwise, clean the
		 * V_BLANK_NVSD bit of vblank ref-count. */
		if (nvsd_updated) {
			set_bit(V_BLANK_NVSD, &dc->vblank_ref_count);
			tegra_dc_unmask_interrupt(dc, V_BLANK_INT);
		} else {
			clear_bit(V_BLANK_NVSD, &dc->vblank_ref_count);
		}
	}

	/* Mask vblank interrupt if ref-count is zero. */
	if (!dc->vblank_ref_count)
		tegra_dc_mask_interrupt(dc, V_BLANK_INT);

	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);

	/* Do the actual brightness update outside of the mutex dc->lock */
	if (nvsd_updated)
		tegra_dc_prism_update_backlight(dc);
}

static void tegra_dc_one_shot_worker(struct work_struct *work)
{
	struct tegra_dc *dc = container_of(
		to_delayed_work(work), struct tegra_dc, one_shot_work);
	mutex_lock(&dc->lock);

	/* memory client has gone idle */
	tegra_dc_clear_bandwidth(dc);

	if (dc->out_ops && dc->out_ops->idle) {
		tegra_dc_io_start(dc);
		dc->out_ops->idle(dc);
		tegra_dc_io_end(dc);
	}

	mutex_unlock(&dc->lock);
}

/* return an arbitrarily large number if count overflow occurs.
 * make it a nice base-10 number to show up in stats output */
static u64 tegra_dc_underflow_count(struct tegra_dc *dc, unsigned reg)
{
	unsigned count = tegra_dc_readl(dc, reg);

	tegra_dc_writel(dc, 0, reg);
	return ((count & 0x80000000) == 0) ? count : 10000000000ll;
}

static void tegra_dc_underflow_handler(struct tegra_dc *dc)
{
	const u32 masks[] = {
		WIN_A_UF_INT,
		WIN_B_UF_INT,
		WIN_C_UF_INT,
#if defined(CONFIG_ARCH_TEGRA_14x_SOC) || defined(CONFIG_ARCH_TEGRA_12x_SOC)
		WIN_D_UF_INT,
		HC_UF_INT,
		WIN_T_UF_INT,
#endif
	};
	int i;

	dc->stats.underflows++;
	if (dc->underflow_mask & WIN_A_UF_INT)
		dc->stats.underflows_a += tegra_dc_underflow_count(dc,
			DC_WINBUF_AD_UFLOW_STATUS);
	if (dc->underflow_mask & WIN_B_UF_INT)
		dc->stats.underflows_b += tegra_dc_underflow_count(dc,
			DC_WINBUF_BD_UFLOW_STATUS);
	if (dc->underflow_mask & WIN_C_UF_INT)
		dc->stats.underflows_c += tegra_dc_underflow_count(dc,
			DC_WINBUF_CD_UFLOW_STATUS);
#if defined(CONFIG_ARCH_TEGRA_14x_SOC) || defined(CONFIG_ARCH_TEGRA_12x_SOC)
	if (dc->underflow_mask & HC_UF_INT)
		dc->stats.underflows_h += tegra_dc_underflow_count(dc,
			DC_WINBUF_HD_UFLOW_STATUS);
	if (dc->underflow_mask & WIN_D_UF_INT)
		dc->stats.underflows_d += tegra_dc_underflow_count(dc,
			DC_WINBUF_DD_UFLOW_STATUS);
	if (dc->underflow_mask & WIN_T_UF_INT)
		dc->stats.underflows_t += tegra_dc_underflow_count(dc,
			DC_WINBUF_TD_UFLOW_STATUS);
#endif

	/* Check for any underflow reset conditions */
	for_each_set_bit(i, &dc->valid_windows, DC_N_WINDOWS) {
		if (WARN_ONCE(i >= ARRAY_SIZE(masks),
			"underflow stats unsupported"))
			break; /* bail if the table above is missing entries */
		if (!masks[i])
			continue; /* skip empty entries */

		if (dc->underflow_mask & masks[i]) {
			dc->windows[i].underflows++;

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
			if (i < 3 && dc->windows[i].underflows > 4) {
				schedule_work(&dc->reset_work);
				/* reset counter */
				dc->windows[i].underflows = 0;
				trace_display_reset(dc);
			}
#endif
#ifdef CONFIG_ARCH_TEGRA_3x_SOC
			if (i < 3 && dc->windows[i].underflows > 4) {
				trace_display_reset(dc);
				tegra_dc_writel(dc, UF_LINE_FLUSH,
						DC_DISP_DISP_MISC_CONTROL);
				tegra_dc_writel(dc, GENERAL_ACT_REQ,
						DC_CMD_STATE_CONTROL);

				tegra_dc_writel(dc, 0,
						DC_DISP_DISP_MISC_CONTROL);
				tegra_dc_writel(dc, GENERAL_ACT_REQ,
						DC_CMD_STATE_CONTROL);
			}
#endif
		} else {
			dc->windows[i].underflows = 0;
		}
	}

	/* Clear the underflow mask now that we've checked it. */
	tegra_dc_writel(dc, dc->underflow_mask, DC_CMD_INT_STATUS);
	dc->underflow_mask = 0;
	tegra_dc_unmask_interrupt(dc, ALL_UF_INT());
	trace_underflow(dc);
}

#if !defined(CONFIG_ARCH_TEGRA_2x_SOC) && !defined(CONFIG_ARCH_TEGRA_3x_SOC)
static void tegra_dc_vpulse2(struct work_struct *work)
{
	struct tegra_dc *dc = container_of(work, struct tegra_dc, vpulse2_work);
	bool nvsd_updated = false;

	mutex_lock(&dc->lock);

	if (!dc->enabled) {
		mutex_unlock(&dc->lock);
		return;
	}

	tegra_dc_get(dc);

	/* Clear the V_PULSE2_FLIP if no update */
	if (!tegra_dc_windows_are_dirty(dc))
		clear_bit(V_PULSE2_FLIP, &dc->vpulse2_ref_count);

	/* Update the SD brightness */
	if (dc->out->sd_settings && dc->out->sd_settings->use_vpulse2) {
		nvsd_updated = nvsd_update_brightness(dc);
		if (nvsd_updated) {
			set_bit(V_PULSE2_NVSD, &dc->vpulse2_ref_count);
			tegra_dc_unmask_interrupt(dc, V_PULSE2_INT);
		} else {
			clear_bit(V_PULSE2_NVSD, &dc->vpulse2_ref_count);
		}
	}

	/* Mask vpulse2 interrupt if ref-count is zero. */
	if (!dc->vpulse2_ref_count)
		tegra_dc_mask_interrupt(dc, V_PULSE2_INT);

	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);

	/* Do the actual brightness update outside of the mutex dc->lock */
	if (nvsd_updated)
		tegra_dc_prism_update_backlight(dc);
}
#endif

static void tegra_dc_process_vblank(struct tegra_dc *dc, ktime_t timestamp)
{
	if (test_bit(V_BLANK_USER, &dc->vblank_ref_count)) {
#ifdef CONFIG_ADF_TEGRA
		tegra_adf_process_vblank(dc->adf, timestamp);
#endif
#ifdef CONFIG_TEGRA_DC_EXTENSIONS
		tegra_dc_ext_process_vblank(dc->ndev->id, timestamp);
#endif
	}
}

int tegra_dc_config_frame_end_intr(struct tegra_dc *dc, bool enable)
{
	int ret;

	mutex_lock(&dc->lock);
	ret = _tegra_dc_config_frame_end_intr(dc, enable);
	mutex_unlock(&dc->lock);

	return ret;
}

static void tegra_dc_one_shot_irq(struct tegra_dc *dc, unsigned long status,
				ktime_t timestamp)
{
	/* pending user vblank, so wakeup */
	if (status & (V_BLANK_INT | MSF_INT)) {
		if (dc->out->user_needs_vblank) {
			dc->out->user_needs_vblank = false;
			complete(&dc->out->user_vblank_comp);
		}
		tegra_dc_process_vblank(dc, timestamp);
	}

	if (status & V_BLANK_INT) {
		/* Sync up windows. */
		tegra_dc_trigger_windows(dc);

		/* Schedule any additional bottom-half vblank actvities. */
		queue_work(system_freezable_wq, &dc->vblank_work);
	}

	if (status & FRAME_END_INT) {
		/* Mark the frame_end as complete. */
		dc->crc_pending = false;
		if (!completion_done(&dc->frame_end_complete))
			complete(&dc->frame_end_complete);
		if (!completion_done(&dc->crc_complete))
			complete(&dc->crc_complete);

		if (dc->out->flags & TEGRA_DC_OUT_ONE_SHOT_MODE && !dc->nvsr)
			tegra_dc_put(dc);
	}

#if !defined(CONFIG_ARCH_TEGRA_2x_SOC) && !defined(CONFIG_ARCH_TEGRA_3x_SOC)
	if (status & V_PULSE2_INT)
		queue_work(system_freezable_wq, &dc->vpulse2_work);
#endif
}

static void tegra_dc_continuous_irq(struct tegra_dc *dc, unsigned long status,
				ktime_t timestamp)
{
	/* Schedule any additional bottom-half vblank actvities. */
	if (status & V_BLANK_INT)
		queue_work(system_freezable_wq, &dc->vblank_work);

	if (status & (V_BLANK_INT | MSF_INT))
		tegra_dc_process_vblank(dc, timestamp);

	if (status & FRAME_END_INT) {
		struct timespec tm = CURRENT_TIME;
		dc->frame_end_timestamp = timespec_to_ns(&tm);
		wake_up(&dc->timestamp_wq);

		/* Mark the frame_end as complete. */
		if (!completion_done(&dc->frame_end_complete))
			complete(&dc->frame_end_complete);
		if (!completion_done(&dc->crc_complete))
			complete(&dc->crc_complete);

		tegra_dc_trigger_windows(dc);
	}

#if !defined(CONFIG_ARCH_TEGRA_2x_SOC) && !defined(CONFIG_ARCH_TEGRA_3x_SOC)
	if (status & V_PULSE2_INT)
		queue_work(system_freezable_wq, &dc->vpulse2_work);
#endif
}

/* XXX: Not sure if we limit look ahead to 1 frame */
bool tegra_dc_is_within_n_vsync(struct tegra_dc *dc, s64 ts)
{
	BUG_ON(!dc->frametime_ns);
	return ((ts - dc->frame_end_timestamp) < dc->frametime_ns);
}

bool tegra_dc_does_vsync_separate(struct tegra_dc *dc, s64 new_ts, s64 old_ts)
{
	BUG_ON(!dc->frametime_ns);
	return (((new_ts - old_ts) > dc->frametime_ns)
		|| (div_s64((new_ts - dc->frame_end_timestamp), dc->frametime_ns)
			!= div_s64((old_ts - dc->frame_end_timestamp),
				dc->frametime_ns)));
}

static irqreturn_t tegra_dc_irq(int irq, void *ptr)
{
	ktime_t timestamp = ktime_get();
	struct tegra_dc *dc = ptr;
	unsigned long status;
	unsigned long underflow_mask;
	u32 val;
	int need_disable = 0;

	if (tegra_platform_is_fpga())
		return IRQ_NONE;

	mutex_lock(&dc->lock);
	if (!tegra_dc_is_powered(dc)) {
		mutex_unlock(&dc->lock);
		return IRQ_HANDLED;
	}

	tegra_dc_get(dc);

	if (!dc->enabled || !nvhost_module_powered_ext(dc->ndev)) {
		dev_dbg(&dc->ndev->dev, "IRQ when DC not powered!\n");
		status = tegra_dc_readl(dc, DC_CMD_INT_STATUS);
		tegra_dc_writel(dc, status, DC_CMD_INT_STATUS);
		tegra_dc_put(dc);
		mutex_unlock(&dc->lock);
		return IRQ_HANDLED;
	}

	/* clear all status flags except underflow, save those for the worker */
	status = tegra_dc_readl(dc, DC_CMD_INT_STATUS);
	tegra_dc_writel(dc, status & ~ALL_UF_INT(), DC_CMD_INT_STATUS);
	val = tegra_dc_readl(dc, DC_CMD_INT_MASK);
	tegra_dc_writel(dc, val & ~ALL_UF_INT(), DC_CMD_INT_MASK);

	/*
	 * Overlays can get thier internal state corrupted during and underflow
	 * condition.  The only way to fix this state is to reset the DC.
	 * if we get 4 consecutive frames with underflows, assume we're
	 * hosed and reset.
	 */
	underflow_mask = status & ALL_UF_INT();

	/* Check underflow */
	if (underflow_mask) {
		dc->underflow_mask |= underflow_mask;
		schedule_delayed_work(&dc->underflow_work,
			msecs_to_jiffies(1));
	}

	if (dc->out->flags & TEGRA_DC_OUT_ONE_SHOT_MODE)
		tegra_dc_one_shot_irq(dc, status, timestamp);
	else
		tegra_dc_continuous_irq(dc, status, timestamp);

	if (dc->nvsr)
		tegra_dc_nvsr_irq(dc->nvsr, status);

	/* update video mode if it has changed since the last frame */
	if (status & (FRAME_END_INT | V_BLANK_INT))
		if (tegra_dc_update_mode(dc))
			need_disable = 1; /* force display off on error */

	if (status & FRAME_END_INT)
		if (dc->disp_active_dirty) {
			tegra_dc_writel(dc, dc->mode.h_active |
				(dc->mode.v_active << 16), DC_DISP_DISP_ACTIVE);
			tegra_dc_writel(dc,
				GENERAL_ACT_REQ, DC_CMD_STATE_CONTROL);

			dc->disp_active_dirty = false;
		}

	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);

	if (need_disable)
		tegra_dc_disable(dc);
	return IRQ_HANDLED;
}

void tegra_dc_set_color_control(struct tegra_dc *dc)
{
	u32 color_control;

	switch (dc->out->depth) {
	case 3:
		color_control = BASE_COLOR_SIZE111;
		break;

	case 6:
		color_control = BASE_COLOR_SIZE222;
		break;

	case 8:
		color_control = BASE_COLOR_SIZE332;
		break;

	case 9:
		color_control = BASE_COLOR_SIZE333;
		break;

	case 12:
		color_control = BASE_COLOR_SIZE444;
		break;

	case 15:
		color_control = BASE_COLOR_SIZE555;
		break;

	case 16:
		color_control = BASE_COLOR_SIZE565;
		break;

	case 18:
		color_control = BASE_COLOR_SIZE666;
		break;

	default:
		color_control = BASE_COLOR_SIZE888;
		break;
	}

	switch (dc->out->dither) {
	case TEGRA_DC_UNDEFINED_DITHER:
	case TEGRA_DC_DISABLE_DITHER:
		color_control |= DITHER_CONTROL_DISABLE;
		break;
	case TEGRA_DC_ORDERED_DITHER:
		color_control |= DITHER_CONTROL_ORDERED;
		break;
#ifdef CONFIG_TEGRA_DC_TEMPORAL_DITHER
	case TEGRA_DC_TEMPORAL_DITHER:
		color_control |= DITHER_CONTROL_TEMPORAL;
		break;
#else
	case TEGRA_DC_ERRDIFF_DITHER:
		/* The line buffer for error-diffusion dither is limited
		 * to 1280 pixels per line. This limits the maximum
		 * horizontal active area size to 1280 pixels when error
		 * diffusion is enabled.
		 */
		BUG_ON(dc->mode.h_active > 1280);
		color_control |= DITHER_CONTROL_ERRDIFF;
		break;
#endif
	default:
		dev_err(&dc->ndev->dev, "Error: Unsupported dithering mode\n");
	}

#ifdef CONFIG_TEGRA_DC_CMU
	if (dc->cmu_enabled)
		color_control |= CMU_ENABLE;
#endif

	tegra_dc_writel(dc, color_control, DC_DISP_DISP_COLOR_CONTROL);
}

static u32 get_syncpt(struct tegra_dc *dc, int idx)
{
	if (idx >= 0 && idx < ARRAY_SIZE(dc->win_syncpt))
		return dc->win_syncpt[idx];
	BUG();
}

static void tegra_dc_init_vpulse2_int(struct tegra_dc *dc)
{
#if !defined(CONFIG_ARCH_TEGRA_2x_SOC) && !defined(CONFIG_ARCH_TEGRA_3x_SOC)
	u32 start, end;
	unsigned long val;

	val = V_PULSE2_H_POSITION(0) | V_PULSE2_LAST(0x1);
	tegra_dc_writel(dc, val, DC_DISP_V_PULSE2_CONTROL);

	start = dc->mode.v_ref_to_sync + dc->mode.v_sync_width +
		dc->mode.v_back_porch +	dc->mode.v_active;
	end = start + 1;
	val = V_PULSE2_START_A(start) + V_PULSE2_END_A(end);
	tegra_dc_writel(dc, val, DC_DISP_V_PULSE2_POSITION_A);

	val = tegra_dc_readl(dc, DC_CMD_INT_ENABLE);
	val |= V_PULSE2_INT;
	tegra_dc_writel(dc, val , DC_CMD_INT_ENABLE);

	tegra_dc_mask_interrupt(dc, V_PULSE2_INT);
	tegra_dc_writel(dc, V_PULSE_2_ENABLE, DC_DISP_DISP_SIGNAL_OPTIONS0);
#endif
}

static int tegra_dc_init(struct tegra_dc *dc)
{
	int i;
	int int_enable;
	u32 val;

	tegra_dc_io_start(dc);
	tegra_dc_writel(dc, 0x00000100, DC_CMD_GENERAL_INCR_SYNCPT_CNTRL);
	if (dc->ndev->id == 0) {
		tegra_mc_set_priority(TEGRA_MC_CLIENT_DISPLAY0A,
				      TEGRA_MC_PRIO_MED);
		tegra_mc_set_priority(TEGRA_MC_CLIENT_DISPLAY0B,
				      TEGRA_MC_PRIO_MED);
		tegra_mc_set_priority(TEGRA_MC_CLIENT_DISPLAY0C,
				      TEGRA_MC_PRIO_MED);
#if defined(CONFIG_ARCH_TEGRA_2x_SOC) || defined(CONFIG_ARCH_TEGRA_3x_SOC)
		/* only present on Tegra2 and 3 */
		tegra_mc_set_priority(TEGRA_MC_CLIENT_DISPLAY1B,
				      TEGRA_MC_PRIO_MED);
#endif
		tegra_mc_set_priority(TEGRA_MC_CLIENT_DISPLAYHC,
				      TEGRA_MC_PRIO_HIGH);
	} else if (dc->ndev->id == 1) {
		tegra_mc_set_priority(TEGRA_MC_CLIENT_DISPLAY0AB,
				      TEGRA_MC_PRIO_MED);
		tegra_mc_set_priority(TEGRA_MC_CLIENT_DISPLAY0BB,
				      TEGRA_MC_PRIO_MED);
		tegra_mc_set_priority(TEGRA_MC_CLIENT_DISPLAY0CB,
				      TEGRA_MC_PRIO_MED);
#if defined(CONFIG_ARCH_TEGRA_2x_SOC) || defined(CONFIG_ARCH_TEGRA_3x_SOC)
		/* only present on Tegra2 and 3 */
		tegra_mc_set_priority(TEGRA_MC_CLIENT_DISPLAY1BB,
				      TEGRA_MC_PRIO_MED);
#endif
		tegra_mc_set_priority(TEGRA_MC_CLIENT_DISPLAYHCB,
				      TEGRA_MC_PRIO_HIGH);
	}
	tegra_dc_writel(dc, 0x00000100 | dc->vblank_syncpt,
			DC_CMD_CONT_SYNCPT_VSYNC);

	tegra_dc_writel(dc, 0x00004700, DC_CMD_INT_TYPE);
#if defined(CONFIG_ARCH_TEGRA_14x_SOC) || defined(CONFIG_ARCH_TEGRA_12x_SOC)
	tegra_dc_writel(dc, WIN_A_OF_INT | WIN_B_OF_INT | WIN_C_OF_INT |
		WIN_T_UF_INT | WIN_D_UF_INT | HC_UF_INT |
		WIN_A_UF_INT | WIN_B_UF_INT | WIN_C_UF_INT,
		DC_CMD_INT_POLARITY);
#else
	tegra_dc_writel(dc, WIN_A_OF_INT | WIN_B_OF_INT | WIN_C_OF_INT |
		WIN_A_UF_INT | WIN_B_UF_INT | WIN_C_UF_INT,
		DC_CMD_INT_POLARITY);
#endif
	tegra_dc_writel(dc, 0x00202020, DC_DISP_MEM_HIGH_PRIORITY);
	tegra_dc_writel(dc, 0x00010101, DC_DISP_MEM_HIGH_PRIORITY_TIMER);
#ifdef CONFIG_ARCH_TEGRA_3x_SOC
	tegra_dc_writel(dc, 0x00000000, DC_DISP_DISP_MISC_CONTROL);
#endif
	/* enable interrupts for vblank, frame_end and underflows */
	int_enable = (FRAME_END_INT | V_BLANK_INT | ALL_UF_INT());
	/* for panels with one-shot mode enable tearing effect interrupt */
	if (dc->out->flags & TEGRA_DC_OUT_ONE_SHOT_MODE)
		int_enable |= MSF_INT;

	tegra_dc_writel(dc, int_enable, DC_CMD_INT_ENABLE);
	tegra_dc_writel(dc, ALL_UF_INT(), DC_CMD_INT_MASK);
	tegra_dc_init_vpulse2_int(dc);

#if !defined(CONFIG_TEGRA_DC_BLENDER_GEN2)
	tegra_dc_writel(dc, 0x00000000, DC_DISP_BORDER_COLOR);
#else
	tegra_dc_writel(dc, 0x00000000, DC_DISP_BLEND_BACKGROUND_COLOR);
#endif

#ifdef CONFIG_TEGRA_DC_CMU
	_tegra_dc_update_cmu(dc, tegra_dc_get_cmu(dc));
#endif
	tegra_dc_set_color_control(dc);
	for_each_set_bit(i, &dc->valid_windows, DC_N_WINDOWS) {
		struct tegra_dc_win *win = &dc->windows[i];
		tegra_dc_writel(dc, WINDOW_A_SELECT << i,
				DC_CMD_DISPLAY_WINDOW_HEADER);
		tegra_dc_set_csc(dc, &win->csc);
		tegra_dc_set_lut(dc, win);
		tegra_dc_set_scaling_filter(dc);
	}

#ifdef CONFIG_TEGRA_DC_WIN_H
	/* Window H is set to window mode by default for t14x. */
	tegra_dc_writel(dc, WINH_CURS_SELECT(1),
			DC_DISP_BLEND_CURSOR_CONTROL);
#endif

	for_each_set_bit(i, &dc->valid_windows, DC_N_WINDOWS) {
		u32 syncpt = get_syncpt(dc, i);

		/* refuse to operate on invalid syncpts */
		if (WARN_ON(syncpt == NVSYNCPT_INVALID))
			continue;

		dc->syncpt[i].id = syncpt;

		if (!nvhost_syncpt_read_ext_check(dc->ndev, syncpt, &val))
			dc->syncpt[i].min = dc->syncpt[i].max = val;
	}

	dc->crc_pending = false;

	trace_display_mode(dc, &dc->mode);

	if (dc->mode.pclk) {
		if (!dc->initialized) {
			if (tegra_dc_program_mode(dc, &dc->mode)) {
				tegra_dc_io_end(dc);
				dev_warn(&dc->ndev->dev,
					"%s: tegra_dc_program_mode failed\n",
					__func__);
				return -EINVAL;
			}
		} else {
			dev_info(&dc->ndev->dev, "DC initialized, "
					"skipping tegra_dc_program_mode.\n");
		}
	}

	/* Initialize SD AFTER the modeset.
	   nvsd_init handles the sd_settings = NULL case. */
	nvsd_init(dc, dc->out->sd_settings);

	tegra_dc_io_end(dc);

	return 0;
}

static bool _tegra_dc_controller_enable(struct tegra_dc *dc)
{
	int failed_init = 0;
	int i;

	if (WARN_ON(!dc || !dc->out || !dc->out_ops))
		return false;

	tegra_dc_unpowergate_locked(dc);

	if (dc->out->enable)
		dc->out->enable(&dc->ndev->dev);

	tegra_dc_setup_clk(dc, dc->clk);

	/* dc clk always on for continuous mode */
	if (!(dc->out->flags & TEGRA_DC_OUT_ONE_SHOT_MODE))
		tegra_dc_clk_enable(dc);
	else
		tegra_dvfs_set_rate(dc->clk, dc->mode.pclk);

	tegra_dc_get(dc);

	tegra_dc_power_on(dc);

	/* do not accept interrupts during initialization */
	tegra_dc_writel(dc, 0, DC_CMD_INT_MASK);

	enable_dc_irq(dc);

	failed_init = tegra_dc_init(dc);
	if (failed_init) {
		tegra_dc_writel(dc, 0, DC_CMD_INT_MASK);
		disable_irq_nosync(dc->irq);
		tegra_dc_clear_bandwidth(dc);
		if (dc->out && dc->out->disable)
			dc->out->disable();
		tegra_dc_put(dc);
		if (!(dc->out->flags & TEGRA_DC_OUT_ONE_SHOT_MODE))
			tegra_dc_clk_disable(dc);
		else
			tegra_dvfs_set_rate(dc->clk, 0);
		dev_warn(&dc->ndev->dev,
			"%s: tegra_dc_init failed\n", __func__);
		return false;
	}

	tegra_dpaux_pad_power(dc, false);

	if (dc->out_ops && dc->out_ops->enable)
		dc->out_ops->enable(dc);

	/* force a full blending update */
	for (i = 0; i < DC_N_WINDOWS; i++)
		dc->blend.z[i] = -1;

#ifdef CONFIG_TEGRA_DC_EXTENSIONS
	tegra_dc_ext_enable(dc->ext);
#endif

	/* initialize cursor to defaults, as driver depends on HW state */
	tegra_dc_writel(dc, 0, DC_DISP_CURSOR_START_ADDR);
	tegra_dc_writel(dc, 0, DC_DISP_CURSOR_START_ADDR_NS);
#if defined(CONFIG_ARCH_TEGRA_12x_SOC)
	tegra_dc_writel(dc, 0, DC_DISP_CURSOR_START_ADDR_HI);
	tegra_dc_writel(dc, 0, DC_DISP_CURSOR_START_ADDR_HI_NS);
#endif
	tegra_dc_writel(dc, 0, DC_DISP_CURSOR_POSITION);
	tegra_dc_writel(dc, 0, DC_DISP_CURSOR_POSITION_NS);
	tegra_dc_writel(dc, 0xffffff, DC_DISP_CURSOR_FOREGROUND); /* white */
	tegra_dc_writel(dc, 0x000000, DC_DISP_CURSOR_BACKGROUND); /* black */
	tegra_dc_writel(dc, 0, DC_DISP_BLEND_CURSOR_CONTROL);

	trace_display_enable(dc);

	tegra_dc_writel(dc, GENERAL_ACT_REQ, DC_CMD_STATE_CONTROL);

	if (dc->out->postpoweron)
		dc->out->postpoweron(&dc->ndev->dev);

	if (dc->out_ops && dc->out_ops->postpoweron)
		dc->out_ops->postpoweron(dc);

	tegra_log_resume_time();
	/*
	 * We will need to reinitialize the display the next time panel
	 * is enabled.
	 */
	dc->out->flags &= ~TEGRA_DC_OUT_INITIALIZED_MODE;

	tegra_dc_put(dc);

	return true;
}

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
static bool _tegra_dc_controller_reset_enable(struct tegra_dc *dc)
{
	bool ret = true;

	if (WARN_ON(!dc || !dc->out || !dc->out_ops))
		return false;

	if (dc->out->enable)
		dc->out->enable(&dc->ndev->dev);

	tegra_dc_setup_clk(dc, dc->clk);
	tegra_dc_clk_enable(dc);

	if (dc->ndev->id == 0 && tegra_dcs[1] != NULL) {
		mutex_lock(&tegra_dcs[1]->lock);
		disable_irq_nosync(tegra_dcs[1]->irq);
	} else if (dc->ndev->id == 1 && tegra_dcs[0] != NULL) {
		mutex_lock(&tegra_dcs[0]->lock);
		disable_irq_nosync(tegra_dcs[0]->irq);
	}

	msleep(5);
	tegra_periph_reset_assert(dc->clk);
	msleep(2);
	if (tegra_platform_is_silicon()) {
		tegra_periph_reset_deassert(dc->clk);
		msleep(1);
	}

	if (dc->ndev->id == 0 && tegra_dcs[1] != NULL) {
		enable_dc_irq(tegra_dcs[1]);
		mutex_unlock(&tegra_dcs[1]->lock);
	} else if (dc->ndev->id == 1 && tegra_dcs[0] != NULL) {
		enable_dc_irq(tegra_dcs[0]);
		mutex_unlock(&tegra_dcs[0]->lock);
	}

	enable_dc_irq(dc);

	if (tegra_dc_init(dc)) {
		dev_err(&dc->ndev->dev, "cannot initialize\n");
		ret = false;
	}

	if (dc->out_ops && dc->out_ops->enable)
		dc->out_ops->enable(dc);

	if (dc->out->postpoweron)
		dc->out->postpoweron(&dc->ndev->dev);

	/* force a full blending update */
	dc->blend.z[0] = -1;

#ifdef CONFIG_TEGRA_DC_EXTENSIONS
	tegra_dc_ext_enable(dc->ext);
#endif

	if (!ret) {
		dev_err(&dc->ndev->dev, "initialization failed,disabling");
		_tegra_dc_controller_disable(dc);
	}

	trace_display_reset(dc);
	return ret;
}
#endif

static int _tegra_dc_set_default_videomode(struct tegra_dc *dc)
{
	if (dc->mode.pclk == 0) {
		switch (dc->out->type) {
		case TEGRA_DC_OUT_HDMI:
		/* If DC is enable called, and HDMI is connected,
		 * but DC is not initialized by bootloader and no
		 * mode is set up, then set a fallback mode.
		 */
		if (tegra_dc_hpd(dc) && (!dc->initialized)) {
			return tegra_dc_set_fb_mode(dc, &tegra_dc_vga_mode, 0);
		} else
			return false;

		break;

		case TEGRA_DC_OUT_DP:
		case TEGRA_DC_OUT_NVSR_DP:
			return tegra_dc_set_fb_mode(dc, &tegra_dc_vga_mode, 0);

		/* Do nothing for other outputs for now */
		case TEGRA_DC_OUT_RGB:

		case TEGRA_DC_OUT_DSI:

		default:
			return false;
		}
	}

	return false;
}

int tegra_dc_set_default_videomode(struct tegra_dc *dc)
{
	return _tegra_dc_set_default_videomode(dc);
}

static bool _tegra_dc_enable(struct tegra_dc *dc)
{
	if (dc->mode.pclk == 0)
		return false;

	if (!dc->out)
		return false;

	if (dc->enabled)
		return true;

	pm_runtime_get_sync(&dc->ndev->dev);

	if (dc->out->type == TEGRA_DC_OUT_HDMI && !tegra_dc_hpd(dc))
		return false;

	if (!_tegra_dc_controller_enable(dc)) {
		pm_runtime_put_sync(&dc->ndev->dev);
		return false;
	}

	return true;
}

void tegra_dc_enable(struct tegra_dc *dc)
{
	if (WARN_ON(!dc || !dc->out || !dc->out_ops))
		return;

	mutex_lock(&dc->lock);

	if (!dc->enabled)
		dc->enabled = _tegra_dc_enable(dc);

	mutex_unlock(&dc->lock);
	trace_display_mode(dc, &dc->mode);
}

void tegra_dc_disable_window(struct tegra_dc *dc, unsigned win)
{
	struct tegra_dc_win *w = &dc->windows[win];

	/* reset window bandwidth */
	w->bandwidth = 0;
	w->new_bandwidth = 0;

	/* disable windows */
	w->flags &= ~TEGRA_WIN_FLAG_ENABLED;

	/* refuse to operate on invalid syncpts */
	if (WARN_ON(dc->syncpt[win].id == NVSYNCPT_INVALID))
		return;

	/* flush any pending syncpt waits */
	dc->syncpt[win].max += 1;
	while (dc->syncpt[win].min < dc->syncpt[win].max) {
		trace_display_syncpt_flush(dc, dc->syncpt[win].id,
			dc->syncpt[win].min, dc->syncpt[win].max);
		dc->syncpt[win].min++;
		nvhost_syncpt_cpu_incr_ext(dc->ndev, dc->syncpt[win].id);
	}
}

static void _tegra_dc_controller_disable(struct tegra_dc *dc)
{
	unsigned i;

	tegra_dc_get(dc);

	if (atomic_read(&dc->holding)) {
		/* Force release all refs but the last one */
		atomic_set(&dc->holding, 1);
		tegra_dc_release_dc_out(dc);
	}

	if (dc->out && dc->out->prepoweroff)
		dc->out->prepoweroff();

	if (dc->out_ops && dc->out_ops->disable)
		dc->out_ops->disable(dc);

	if (tegra_powergate_is_powered(dc->powergate_id))
		tegra_dc_writel(dc, 0, DC_CMD_INT_MASK);

	disable_irq_nosync(dc->irq);

	tegra_dc_clear_bandwidth(dc);

	if (dc->out && dc->out->disable)
		dc->out->disable();

	for_each_set_bit(i, &dc->valid_windows, DC_N_WINDOWS) {
		tegra_dc_disable_window(dc, i);
	}
	trace_display_disable(dc);

	if (dc->out_ops && dc->out_ops->postpoweroff)
		dc->out_ops->postpoweroff(dc);

	tegra_dc_put(dc);

	/* disable always on dc clk in continuous mode */
	if (!(dc->out->flags & TEGRA_DC_OUT_ONE_SHOT_MODE))
		tegra_dc_clk_disable(dc);
	else
		tegra_dvfs_set_rate(dc->clk, 0);
}

void tegra_dc_stats_enable(struct tegra_dc *dc, bool enable)
{
#if 0 /* underflow interrupt is already enabled by dc reset worker */
	u32 val;
	if (dc->enabled)  {
		val = tegra_dc_readl(dc, DC_CMD_INT_ENABLE);
		if (enable)
			val |= (WIN_A_UF_INT | WIN_B_UF_INT | WIN_C_UF_INT);
		else
			val &= ~(WIN_A_UF_INT | WIN_B_UF_INT | WIN_C_UF_INT);
		tegra_dc_writel(dc, val, DC_CMD_INT_ENABLE);
	}
#endif
}

bool tegra_dc_stats_get(struct tegra_dc *dc)
{
#if 0 /* right now it is always enabled */
	u32 val;
	bool res;

	if (dc->enabled)  {
		val = tegra_dc_readl(dc, DC_CMD_INT_ENABLE);
		res = !!(val & (WIN_A_UF_INT | WIN_B_UF_INT | WIN_C_UF_INT));
	} else {
		res = false;
	}

	return res;
#endif
	return true;
}

/* blank selected windows by disabling them */
void tegra_dc_blank(struct tegra_dc *dc, unsigned windows)
{
	struct tegra_dc_win *dcwins[DC_N_WINDOWS];
	unsigned i;
	unsigned long int blank_windows;
	int nr_win = 0;

	blank_windows = windows & dc->valid_windows;

	if (!blank_windows)
		return;

	for_each_set_bit(i, &blank_windows, DC_N_WINDOWS) {
		dcwins[nr_win] = tegra_dc_get_window(dc, i);
		if (!dcwins[nr_win])
			continue;
		dcwins[nr_win++]->flags &= ~TEGRA_WIN_FLAG_ENABLED;
	}

	tegra_dc_update_windows(dcwins, nr_win, NULL);
	tegra_dc_sync_windows(dcwins, nr_win);
	tegra_dc_program_bandwidth(dc, true);
}

int tegra_dc_restore(struct tegra_dc *dc)
{
#ifdef CONFIG_TEGRA_DC_EXTENSIONS
	return tegra_dc_ext_restore(dc->ext);
#else
	return 0;
#endif
}

static void _tegra_dc_disable(struct tegra_dc *dc)
{
#ifdef CONFIG_TEGRA_DC_CMU
	/* power down resets the registers, setting to true
	 * causes CMU to be restored in tegra_dc_init(). */
	dc->cmu_dirty = true;
#endif

	if (dc->out->flags & TEGRA_DC_OUT_ONE_SHOT_MODE) {
		mutex_lock(&dc->one_shot_lock);
		cancel_delayed_work_sync(&dc->one_shot_work);
	}

	tegra_dc_io_start(dc);
	_tegra_dc_controller_disable(dc);
	tegra_dc_io_end(dc);

	tegra_dc_powergate_locked(dc);

	if (dc->out->flags & TEGRA_DC_OUT_ONE_SHOT_MODE)
		mutex_unlock(&dc->one_shot_lock);
	pm_runtime_put(&dc->ndev->dev);

	tegra_log_suspend_time();
}

void tegra_dc_disable(struct tegra_dc *dc)
{
	if (WARN_ON(!dc || !dc->out || !dc->out_ops))
		return;

#ifdef CONFIG_TEGRA_DC_EXTENSIONS
	tegra_dc_ext_disable(dc->ext);
#endif

	/* it's important that new underflow work isn't scheduled before the
	 * lock is acquired. */
	cancel_delayed_work_sync(&dc->underflow_work);

	if (dc->out->flags & TEGRA_DC_OUT_ONE_SHOT_MODE)
		mutex_lock(&dc->one_shot_lp_lock);
	mutex_lock(&dc->lock);

	if (dc->enabled) {
		dc->enabled = false;
		dc->blanked = false;

		if (!dc->suspended)
			_tegra_dc_disable(dc);
	}

#ifdef CONFIG_SWITCH
	switch_set_state(&dc->modeset_switch, 0);
#endif
	mutex_unlock(&dc->lock);
	if (dc->out->flags & TEGRA_DC_OUT_ONE_SHOT_MODE)
		mutex_unlock(&dc->one_shot_lp_lock);
	synchronize_irq(dc->irq);
	trace_display_mode(dc, &dc->mode);

	/* disable pending clks due to uncompleted frames */
	while (tegra_is_clk_enabled(dc->clk))
		tegra_dc_put(dc);
}

#ifdef CONFIG_ARCH_TEGRA_2x_SOC
static void tegra_dc_reset_worker(struct work_struct *work)
{
	struct tegra_dc *dc =
		container_of(work, struct tegra_dc, reset_work);

	unsigned long val = 0;

	mutex_lock(&shared_lock);

	dev_warn(&dc->ndev->dev,
		"overlay stuck in underflow state.  resetting.\n");

#ifdef CONFIG_TEGRA_DC_EXTENSIONS
	tegra_dc_ext_disable(dc->ext);
#endif

	mutex_lock(&dc->lock);

	if (dc->enabled == false)
		goto unlock;

	dc->enabled = false;

	/*
	 * off host read bus
	 */
	val = tegra_dc_readl(dc, DC_CMD_CONT_SYNCPT_VSYNC);
	val &= ~(0x00000100);
	tegra_dc_writel(dc, val, DC_CMD_CONT_SYNCPT_VSYNC);

	/*
	 * set DC to STOP mode
	 */
	tegra_dc_writel(dc, DISP_CTRL_MODE_STOP, DC_CMD_DISPLAY_COMMAND);

	msleep(10);

	_tegra_dc_controller_disable(dc);

	/* _tegra_dc_controller_reset_enable deasserts reset */
	_tegra_dc_controller_reset_enable(dc);

	dc->enabled = true;

	/* reopen host read bus */
	val = tegra_dc_readl(dc, DC_CMD_CONT_SYNCPT_VSYNC);
	val &= ~(0x00000100);
	val |= 0x100;
	tegra_dc_writel(dc, val, DC_CMD_CONT_SYNCPT_VSYNC);

unlock:
	mutex_unlock(&dc->lock);
	mutex_unlock(&shared_lock);
	trace_display_reset(dc);
}
#endif

static void tegra_dc_underflow_worker(struct work_struct *work)
{
	struct tegra_dc *dc = container_of(
		to_delayed_work(work), struct tegra_dc, underflow_work);

	mutex_lock(&dc->lock);
	tegra_dc_get(dc);

	if (dc->enabled) {
		tegra_dc_underflow_handler(dc);
	}
	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);
}

static void (*flip_callback)(void);
static spinlock_t flip_callback_lock;
static bool init_tegra_dc_flip_callback_called;

static int __init init_tegra_dc_flip_callback(void)
{
	spin_lock_init(&flip_callback_lock);
	init_tegra_dc_flip_callback_called = true;
	return 0;
}

pure_initcall(init_tegra_dc_flip_callback);

int tegra_dc_set_flip_callback(void (*callback)(void))
{
	WARN_ON(!init_tegra_dc_flip_callback_called);

	spin_lock(&flip_callback_lock);
	flip_callback = callback;
	spin_unlock(&flip_callback_lock);

	return 0;
}
EXPORT_SYMBOL(tegra_dc_set_flip_callback);

int tegra_dc_unset_flip_callback()
{
	spin_lock(&flip_callback_lock);
	flip_callback = NULL;
	spin_unlock(&flip_callback_lock);

	return 0;
}
EXPORT_SYMBOL(tegra_dc_unset_flip_callback);

void tegra_dc_call_flip_callback()
{
	spin_lock(&flip_callback_lock);
	if (flip_callback)
		flip_callback();
	spin_unlock(&flip_callback_lock);
}
EXPORT_SYMBOL(tegra_dc_call_flip_callback);

#ifdef CONFIG_SWITCH
static ssize_t switch_modeset_print_mode(struct switch_dev *sdev, char *buf)
{
	struct tegra_dc *dc =
		container_of(sdev, struct tegra_dc, modeset_switch);

	if (!sdev->state)
		return sprintf(buf, "offline\n");

	return sprintf(buf, "%dx%d\n", dc->mode.h_active, dc->mode.v_active);
}
#endif

static int tegra_dc_probe(struct platform_device *ndev)
{
	struct tegra_dc *dc;
	struct tegra_dc_mode *mode;
	struct tegra_dc_platform_data *dt_pdata = NULL;
	struct clk *clk;
#ifndef CONFIG_TEGRA_ISOMGR
	struct clk *emc_clk;
#else
	int isomgr_client_id = -1;
#endif
	struct device_node *np = ndev->dev.of_node;
	struct resource *res;
	struct resource dt_res;
	struct resource *base_res;
	struct resource *fb_mem = NULL;
	int ret = 0;
	void __iomem *base;
	int irq;
	int i;

	if (!np && !ndev->dev.platform_data) {
		dev_err(&ndev->dev, "no platform data\n");
		return -ENOENT;
	}

	/* Specify parameters for the maximum physical segment size. */
	ndev->dev.dma_parms = &tegra_dc_dma_parameters;

	dc = kzalloc(sizeof(struct tegra_dc), GFP_KERNEL);
	if (!dc) {
		dev_err(&ndev->dev, "can't allocate memory for tegra_dc\n");
		return -ENOMEM;
	}

	if (np) {
		dt_pdata = of_dc_parse_platform_data(ndev);
		if (dt_pdata == NULL)
			goto err_free;

#ifdef CONFIG_OF
		irq = of_irq_to_resource(np, 0, NULL);
		if (!irq)
			goto err_free;
#endif

		ret = of_address_to_resource(np, 0, &dt_res);
		if (ret)
			goto err_free;

		if (dt_res.start == TEGRA_DISPLAY_BASE)
			ndev->id = 0;
		else if (dt_res.start == TEGRA_DISPLAY2_BASE)
			ndev->id = 1;
		else
			goto err_free;

		res = &dt_res;
	} else {
		irq = platform_get_irq_byname(ndev, "irq");
		if (irq <= 0) {
			dev_err(&ndev->dev, "no irq\n");
			ret = -ENOENT;
			goto err_free;
		}

		res = platform_get_resource_byname(ndev,
			IORESOURCE_MEM, "regs");
		if (!res) {
			dev_err(&ndev->dev, "no mem resource\n");
			ret = -ENOENT;
			goto err_free;
		}
	}

	base_res = request_mem_region(res->start, resource_size(res),
		ndev->name);
	if (!base_res) {
		dev_err(&ndev->dev, "request_mem_region failed\n");
		ret = -EBUSY;
		goto err_free;
	}

	base = ioremap(res->start, resource_size(res));
	if (!base) {
		dev_err(&ndev->dev, "registers can't be mapped\n");
		ret = -EBUSY;
		goto err_release_resource_reg;
	}

	for (i = 0; i < DC_N_WINDOWS; i++)
		dc->win_syncpt[i] = NVSYNCPT_INVALID;

	if (TEGRA_DISPLAY_BASE == res->start) {
		dc->vblank_syncpt = NVSYNCPT_VBLANK0;
		dc->win_syncpt[0] = nvhost_get_syncpt_client_managed("disp0_a");
		dc->win_syncpt[1] = nvhost_get_syncpt_client_managed("disp0_b");
		dc->win_syncpt[2] = nvhost_get_syncpt_client_managed("disp0_c");
		dc->valid_windows = 0x07;
#ifdef CONFIG_ARCH_TEGRA_14x_SOC
		dc->win_syncpt[3] = nvhost_get_syncpt_client_managed("disp0_d");
		dc->win_syncpt[4] = nvhost_get_syncpt_client_managed("disp0_h");
		dc->valid_windows |= 0x18;
#elif defined(CONFIG_ARCH_TEGRA_12x_SOC)
		dc->win_syncpt[3] = nvhost_get_syncpt_client_managed("disp0_d");
		dc->valid_windows |= 0x08;
#endif
		dc->powergate_id = TEGRA_POWERGATE_DISA;
#ifdef CONFIG_TEGRA_ISOMGR
		isomgr_client_id = TEGRA_ISO_CLIENT_DISP_0;
#endif
	} else if (TEGRA_DISPLAY2_BASE == res->start) {
		dc->vblank_syncpt = NVSYNCPT_VBLANK1;
		dc->win_syncpt[0] = nvhost_get_syncpt_client_managed("disp1_a");
		dc->win_syncpt[1] = nvhost_get_syncpt_client_managed("disp1_b");
		dc->win_syncpt[2] = nvhost_get_syncpt_client_managed("disp1_c");
		dc->valid_windows = 0x07;
#ifdef CONFIG_ARCH_TEGRA_14x_SOC
		dc->win_syncpt[4] = nvhost_get_syncpt_client_managed("disp1_h");
		dc->valid_windows |= 0x10;
#endif
		dc->powergate_id = TEGRA_POWERGATE_DISB;
#ifdef CONFIG_TEGRA_ISOMGR
		isomgr_client_id = TEGRA_ISO_CLIENT_DISP_1;
#endif
	} else {
		dev_err(&ndev->dev,
			"Unknown base address %llx: unable to assign syncpt\n",
			(u64)res->start);
	}

	if (np) {
		struct resource of_fb_res;
		int err;
		if (ndev->id == 0)
			err = tegra_get_fb_resource(&of_fb_res);
		else /*ndev->id == 1*/
			err = tegra_get_fb2_resource(&of_fb_res);

		if (err < 0) {
			dev_dbg(&ndev->dev, "failed to get fb resource: %d\n",
					-err);
		} else {
			fb_mem = kzalloc(sizeof(struct resource), GFP_KERNEL);
			if (fb_mem == NULL) {
				ret = -ENOMEM;
				goto err_iounmap_reg;
			}
			fb_mem->name = "fbmem";
			fb_mem->flags = IORESOURCE_MEM;
			fb_mem->start = (resource_size_t)of_fb_res.start;
			fb_mem->end = (resource_size_t)of_fb_res.end;
		}
	} else {
		fb_mem = platform_get_resource_byname(ndev,
			IORESOURCE_MEM, "fbmem");
	}

	clk = clk_get(&ndev->dev, NULL);
	if (IS_ERR_OR_NULL(clk)) {
		dev_err(&ndev->dev, "can't get clock\n");
		ret = -ENOENT;
		goto err_iounmap_reg;
	}

	dc->clk = clk;
	dc->shift_clk_div.mul = dc->shift_clk_div.div = 1;
	/* Initialize one shot work delay, it will be assigned by dsi
	 * according to refresh rate later. */
	dc->one_shot_delay_ms = 40;

	dc->base_res = base_res;
	dc->base = base;
	dc->irq = irq;
	dc->ndev = ndev;
	dc->fb_mem = fb_mem;

	if (!np)
		dc->pdata = ndev->dev.platform_data;
	else
		dc->pdata = dt_pdata;

	dc->bw_kbps = 0;

	mutex_init(&dc->lock);
	mutex_init(&dc->one_shot_lock);
	mutex_init(&dc->one_shot_lp_lock);
	init_completion(&dc->frame_end_complete);
	init_completion(&dc->crc_complete);
	init_waitqueue_head(&dc->wq);
	init_waitqueue_head(&dc->timestamp_wq);
#ifdef CONFIG_ARCH_TEGRA_2x_SOC
	INIT_WORK(&dc->reset_work, tegra_dc_reset_worker);
#endif
	INIT_WORK(&dc->vblank_work, tegra_dc_vblank);
	dc->vblank_ref_count = 0;
#if !defined(CONFIG_ARCH_TEGRA_2x_SOC) && !defined(CONFIG_ARCH_TEGRA_3x_SOC)
	INIT_WORK(&dc->vpulse2_work, tegra_dc_vpulse2);
#endif
	dc->vpulse2_ref_count = 0;
	INIT_DELAYED_WORK(&dc->underflow_work, tegra_dc_underflow_worker);
	INIT_DELAYED_WORK(&dc->one_shot_work, tegra_dc_one_shot_worker);

	tegra_dc_init_lut_defaults(&dc->fb_lut);

	dc->n_windows = DC_N_WINDOWS;
	for (i = 0; i < DC_N_WINDOWS; i++) {
		struct tegra_dc_win *win = &dc->windows[i];
		struct tegra_dc_win *tmp_win = &dc->tmp_wins[i];
		if (!test_bit(i, &dc->valid_windows))
			win->flags |= TEGRA_WIN_FLAG_INVALID;
		win->idx = i;
		win->dc = dc;
		tmp_win->idx = i;
		tmp_win->dc = dc;
		tegra_dc_init_csc_defaults(&win->csc);
		tegra_dc_init_lut_defaults(&win->lut);
	}

	ret = tegra_dc_set(dc, ndev->id);
	if (ret < 0) {
		dev_err(&ndev->dev, "can't add dc\n");
		goto err_put_clk;
	}

	platform_set_drvdata(ndev, dc);

#ifdef CONFIG_SWITCH
	dc->modeset_switch.name = dev_name(&ndev->dev);
	dc->modeset_switch.state = 0;
	dc->modeset_switch.print_state = switch_modeset_print_mode;
	ret = switch_dev_register(&dc->modeset_switch);
	if (ret < 0)
		dev_err(&ndev->dev, "failed to register switch driver\n");
#endif

	tegra_dc_feature_register(dc);

	if (dc->pdata->default_out) {
#ifdef CONFIG_FRAMEBUFFER_CONSOLE
		if (dc->pdata->default_out->hotplug_init)
			dc->pdata->default_out->hotplug_init(&dc->ndev->dev);
#endif /* CONFIG_FRAMEBUFFER_CONSOLE */
		ret = tegra_dc_set_out(dc, dc->pdata->default_out);
		if (ret < 0) {
			dev_err(&dc->ndev->dev, "failed to initialize DC out ops\n");
			goto err_put_clk;
		}
	} else {
		dev_err(&ndev->dev,
			"No default output specified.  Leaving output disabled.\n");
	}
	dc->mode_dirty = false; /* ignore changes tegra_dc_set_out has done */

#ifdef CONFIG_FRAMEBUFFER_CONSOLE
	if (dc->out && dc->out->n_modes &&
	    (dc->out->type == TEGRA_DC_OUT_HDMI)) {
		struct fb_monspecs specs;
		struct tegra_dc_hdmi_data *hdmi = tegra_dc_get_outdata(dc);
		if (!tegra_edid_get_monspecs(hdmi->edid, &specs)) {
			struct tegra_dc_mode *dcmode = &dc->out->modes[0];
			dcmode->pclk          = specs.modedb->pixclock;
			dcmode->pclk          = PICOS2KHZ(dcmode->pclk);
			dcmode->pclk         *= 1000;
			dcmode->h_ref_to_sync = 1;
			dcmode->v_ref_to_sync = 1;
			dcmode->h_sync_width  = specs.modedb->hsync_len;
			dcmode->v_sync_width  = specs.modedb->vsync_len;
			dcmode->h_back_porch  = specs.modedb->left_margin;
			dcmode->v_back_porch  = specs.modedb->upper_margin;
			dcmode->h_active      = specs.modedb->xres;
			dcmode->v_active      = specs.modedb->yres;
			dcmode->h_front_porch = specs.modedb->right_margin;
			dcmode->v_front_porch = specs.modedb->lower_margin;
			tegra_dc_set_mode(dc, dcmode);
			dc->pdata->fb->xres = dcmode->h_active;
			dc->pdata->fb->yres = dcmode->v_active;
		}
	}
#endif /* CONFIG_FRAMEBUFFER_CONSOLE */

#ifndef CONFIG_TEGRA_ISOMGR
		/*
		 * The emc is a shared clock, it will be set based on
		 * the requirements for each user on the bus.
		 */
		emc_clk = clk_get(&ndev->dev, "emc");
		if (IS_ERR_OR_NULL(emc_clk)) {
			dev_err(&ndev->dev, "can't get emc clock\n");
			ret = -ENOENT;
			goto err_put_clk;
		}
		dc->emc_clk = emc_clk;
#endif

#ifdef CONFIG_TEGRA_DC_EXTENSIONS
	dc->ext = tegra_dc_ext_register(ndev, dc);
	if (IS_ERR_OR_NULL(dc->ext)) {
		dev_warn(&ndev->dev, "Failed to enable Tegra DC extensions.\n");
		dc->ext = NULL;
	}
#endif

	/* interrupt handler must be registered before tegra_fb_register() */
	if (request_threaded_irq(irq, NULL, tegra_dc_irq, IRQF_ONESHOT,
			dev_name(&ndev->dev), dc)) {
		dev_err(&ndev->dev, "request_irq %d failed\n", irq);
		ret = -EBUSY;
		goto err_disable_dc;
	}
	disable_dc_irq(dc);

	tegra_pd_add_device(&ndev->dev);
	pm_runtime_use_autosuspend(&ndev->dev);
	pm_runtime_set_autosuspend_delay(&ndev->dev, 100);
	pm_runtime_enable(&ndev->dev);

#ifdef CONFIG_TEGRA_DC_CMU
	/* if bootloader leaves this head enabled, then skip CMU programming. */
	dc->cmu_dirty = (dc->pdata->flags & TEGRA_DC_FLAG_ENABLED) == 0;
	dc->cmu_enabled = dc->pdata->cmu_enable;
#endif

	if (dc->pdata->flags & TEGRA_DC_FLAG_ENABLED) {
		/* WAR: BL is putting DC in bad state for EDP configuration */
		if (dc->out->type == TEGRA_DC_OUT_DP ||
			dc->out->type == TEGRA_DC_OUT_NVSR_DP) {
			clk_prepare_enable(dc->clk);
			tegra_periph_reset_assert(dc->clk);
			udelay(10);
			tegra_periph_reset_deassert(dc->clk);
			udelay(10);
			clk_disable_unprepare(dc->clk);
		}
		_tegra_dc_set_default_videomode(dc);
		dc->enabled = _tegra_dc_enable(dc);

#if !defined(CONFIG_ARCH_TEGRA_11x_SOC) && !defined(CONFIG_ARCH_TEGRA_14x_SOC)
		/* BL or PG init will keep DISA unpowergated after booting.
		 * Adding an extra powergate to balance the refcount
		 * since _tegra_dc_enable() increases the refcount.
		 */
		if (dc->powergate_id == TEGRA_POWERGATE_DISA)
			tegra_dc_powergate_locked(dc);
#endif
	}

#ifdef CONFIG_TEGRA_ISOMGR
	if (isomgr_client_id == -1) {
		dc->isomgr_handle = NULL;
	} else {
		dc->isomgr_handle = tegra_isomgr_register(isomgr_client_id,
			tegra_dc_calc_min_bandwidth(dc),
			tegra_dc_bandwidth_renegotiate, dc);
		if (IS_ERR(dc->isomgr_handle)) {
			dev_err(&dc->ndev->dev,
				"could not register isomgr. err=%ld\n",
				PTR_ERR(dc->isomgr_handle));
			ret = -ENOENT;
			goto err_put_clk;
		}
		dc->reserved_bw = tegra_dc_calc_min_bandwidth(dc);
		/*
		 * Use maximum value so we can try to reserve as much as
		 * needed until we are told by isomgr to backoff.
		 */
		dc->available_bw = UINT_MAX;
	}
#endif

	tegra_dc_create_debugfs(dc);

	dev_info(&ndev->dev, "probed\n");

	if (dc->pdata->fb) {
		if (dc->enabled && dc->pdata->fb->bits_per_pixel == -1) {
			unsigned long fmt;
			tegra_dc_writel(dc,
					WINDOW_A_SELECT << dc->pdata->fb->win,
					DC_CMD_DISPLAY_WINDOW_HEADER);

			fmt = tegra_dc_readl(dc, DC_WIN_COLOR_DEPTH);
			dc->pdata->fb->bits_per_pixel =
				tegra_dc_fmt_bpp(fmt);
		}

		mode = tegra_dc_get_override_mode(dc);
		if (mode) {
			dc->pdata->fb->xres = mode->h_active;
			dc->pdata->fb->yres = mode->v_active;
		}

#ifdef CONFIG_ADF_TEGRA
		tegra_dc_io_start(dc);
		dc->adf = tegra_adf_init(ndev, dc, dc->pdata->fb, fb_mem);
		tegra_dc_io_end(dc);

		if (IS_ERR(dc->adf)) {
			tegra_dc_io_start(dc);
			dc->fb = tegra_fb_register(ndev, dc, dc->pdata->fb,
					fb_mem);
			tegra_dc_io_end(dc);
			if (IS_ERR_OR_NULL(dc->fb)) {
				dc->fb = NULL;
				dev_err(&ndev->dev, "failed to register fb\n");
				goto err_remove_debugfs;
			}
		}
#endif
#ifdef CONFIG_TEGRA_DC_EXTENSIONS
		tegra_dc_io_start(dc);
		dc->fb = tegra_fb_register(ndev, dc, dc->pdata->fb, fb_mem);
		tegra_dc_io_end(dc);
		if (IS_ERR_OR_NULL(dc->fb)) {
			dc->fb = NULL;
			dev_err(&ndev->dev, "failed to register fb\n");
			goto err_remove_debugfs;
		}
#endif
	}

#ifndef CONFIG_FRAMEBUFFER_CONSOLE
	if (dc->out && dc->out->hotplug_init)
		dc->out->hotplug_init(&ndev->dev);
#endif /* !CONFIG_FRAMEBUFFER_CONSOLE */

	if (dc->out_ops) {
		if (dc->out_ops->detect)
			dc->connected = dc->out_ops->detect(dc);
		else
			dc->connected = true;
	}
	else
		dc->connected = false;

	/* Powergate display module when it's unconnected. */
	/* detect() function, if presetns, responsible for the powergate */
	if (!tegra_dc_get_connected(dc) &&
			!(dc->out_ops && dc->out_ops->detect))
		tegra_dc_powergate_locked(dc);

	tegra_dc_create_sysfs(&dc->ndev->dev);

	/*
	 * Overriding the display mode only applies for modes set up during
	 * boot. It should not apply for e.g. HDMI hotplug.
	 */
	dc->initialized = false;

	return 0;

err_remove_debugfs:
	tegra_dc_remove_debugfs(dc);
	free_irq(irq, dc);
err_disable_dc:
#ifdef CONFIG_TEGRA_DC_EXTENSIONS
	if (dc->ext) {
		tegra_dc_ext_disable(dc->ext);
		tegra_dc_ext_unregister(dc->ext);
	}
#endif
	mutex_lock(&dc->lock);
	if (dc->enabled)
		_tegra_dc_disable(dc);
	dc->enabled = false;
	mutex_unlock(&dc->lock);
#ifdef CONFIG_SWITCH
	switch_dev_unregister(&dc->modeset_switch);
#endif
#ifdef CONFIG_TEGRA_ISOMGR
	tegra_isomgr_unregister(dc->isomgr_handle);
#else
	clk_put(emc_clk);
#endif
err_put_clk:
	clk_put(clk);
err_iounmap_reg:
	iounmap(base);
	if (fb_mem) {
		if (!np)
			release_resource(fb_mem);
		else
			kfree(fb_mem);
	}
err_release_resource_reg:
	release_resource(base_res);
err_free:
	kfree(dc);

	return ret;
}

static int tegra_dc_remove(struct platform_device *ndev)
{
	struct tegra_dc *dc = platform_get_drvdata(ndev);
	struct device_node *np = ndev->dev.of_node;

	tegra_dc_remove_sysfs(&dc->ndev->dev);
	tegra_dc_remove_debugfs(dc);

	if (dc->fb) {
		tegra_fb_unregister(dc->fb);
		if (dc->fb_mem) {
			if (!np)
				release_resource(dc->fb_mem);
			else
				kfree(dc->fb_mem);
		}
	}

#ifdef CONFIG_ADF_TEGRA
	if (dc->adf)
		tegra_adf_unregister(dc->adf);
#endif
#ifdef CONFIG_TEGRA_DC_EXTENSIONS
	if (dc->ext) {
		tegra_dc_ext_disable(dc->ext);
		tegra_dc_ext_unregister(dc->ext);
	}
#endif

	mutex_lock(&dc->lock);
	if (dc->enabled)
		_tegra_dc_disable(dc);
	dc->enabled = false;
	mutex_unlock(&dc->lock);
	synchronize_irq(dc->irq); /* wait for IRQ handlers to finish */

#ifdef CONFIG_SWITCH
	switch_dev_unregister(&dc->modeset_switch);
#endif
	free_irq(dc->irq, dc);
#ifdef CONFIG_TEGRA_ISOMGR
	if (dc->isomgr_handle) {
		tegra_isomgr_unregister(dc->isomgr_handle);
		dc->isomgr_handle = NULL;
	}
#else
	clk_put(dc->emc_clk);
#endif
	clk_put(dc->clk);
	iounmap(dc->base);
	if (dc->fb_mem)
		release_resource(dc->base_res);
	kfree(dc);
	tegra_dc_set(NULL, ndev->id);

	return 0;
}

#ifdef CONFIG_PM
static int tegra_dc_suspend(struct platform_device *ndev, pm_message_t state)
{
	struct tegra_dc *dc = platform_get_drvdata(ndev);
	int ret = 0;

	trace_display_suspend(dc);
	dev_dbg(&ndev->dev, "suspend\n");

#ifdef CONFIG_TEGRA_DC_EXTENSIONS
	tegra_dc_ext_disable(dc->ext);
#endif

	tegra_dc_cursor_suspend(dc);
	mutex_lock(&dc->lock);
	ret = tegra_dc_io_start(dc);

	if (dc->out_ops && dc->out_ops->suspend)
		dc->out_ops->suspend(dc);

	if (dc->enabled) {
		_tegra_dc_disable(dc);

		dc->suspended = true;
	}

	if (dc->out && dc->out->postsuspend) {
		dc->out->postsuspend();
		/* avoid resume event due to voltage falling on interfaces that
		 * support hotplug wake. And only do this if a panel is
		 * connected, if we are already disconnected, then no phantom
		 * hotplug can occur by disabling the voltage.
		 */
		if ((dc->out->flags & TEGRA_DC_OUT_HOTPLUG_WAKE_LP0)
			&& tegra_dc_get_connected(dc))
			msleep(100);
	}

	if (!ret)
		tegra_dc_io_end(dc);

#ifdef CONFIG_TEGRA_DC_CMU
	/*
	 * CMU settings are lost when the DC goes to sleep. User-space will
	 * perform a blank ioctl upon resume which will call tegra_dc_init()
	 * and apply CMU settings again, but only if the cached values are
	 * different from those specified. Clearing the cache here ensures
	 * that this will happen.
	 *
	 * It would be better to reapply the CMU settings in tegra_dc_resume(),
	 * but color corruption sometimes happens if we do so and
	 * tegra_dc_init() seems to be the only safe place for this.
	 */
	memset(&dc->cmu, 0, sizeof(dc->cmu));
#endif

	mutex_unlock(&dc->lock);
	synchronize_irq(dc->irq); /* wait for IRQ handlers to finish */

	return 0;
}

static int tegra_dc_resume(struct platform_device *ndev)
{
	struct tegra_dc *dc = platform_get_drvdata(ndev);

	trace_display_resume(dc);
	dev_dbg(&ndev->dev, "resume\n");

	mutex_lock(&dc->lock);
	dc->suspended = false;

	/* To pan the fb on resume */
	tegra_fb_pan_display_reset(dc->fb);

	if (dc->enabled) {
		dc->enabled = false;
		_tegra_dc_set_default_videomode(dc);
		dc->enabled = _tegra_dc_enable(dc);
	}

	if (dc->out && dc->out->hotplug_init)
		dc->out->hotplug_init(&ndev->dev);

	if (dc->out_ops && dc->out_ops->resume)
		dc->out_ops->resume(dc);

	mutex_unlock(&dc->lock);
	tegra_dc_cursor_resume(dc);

	return 0;
}

#endif /* CONFIG_PM */

static void tegra_dc_shutdown(struct platform_device *ndev)
{
	struct tegra_dc *dc = platform_get_drvdata(ndev);

	if (WARN_ON(!dc || !dc->out || !dc->out_ops))
		return;

	if (!dc->enabled)
		return;

	tegra_dc_disable(dc);
}

extern int suspend_set(const char *val, struct kernel_param *kp)
{
	if (!strcmp(val, "dump"))
		dump_regs(tegra_dcs[0]);
#ifdef CONFIG_PM
	else if (!strcmp(val, "suspend"))
		tegra_dc_suspend(tegra_dcs[0]->ndev, PMSG_SUSPEND);
	else if (!strcmp(val, "resume"))
		tegra_dc_resume(tegra_dcs[0]->ndev);
#endif

	return 0;
}

extern int suspend_get(char *buffer, struct kernel_param *kp)
{
	return 0;
}

int suspend;

module_param_call(suspend, suspend_set, suspend_get, &suspend, 0644);


#ifdef CONFIG_OF
static struct of_device_id tegra_display_of_match[] = {
	{.compatible = "nvidia,tegra114-dc", },
	{.compatible = "nvidia,tegra124-dc", },
	{ },
};
#endif

struct platform_driver tegra_dc_driver = {
	.driver = {
		.name = "tegradc",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table =
			of_match_ptr(tegra_display_of_match),
#endif
	},
	.probe = tegra_dc_probe,
	.remove = tegra_dc_remove,
#ifdef CONFIG_PM
	.suspend = tegra_dc_suspend,
	.resume = tegra_dc_resume,
#endif
	.shutdown = tegra_dc_shutdown,
};

#ifndef MODULE
static int __init parse_disp_params(char *options, struct tegra_dc_mode *mode)
{
	int i, params[11];
	char *p;

	for (i = 0; i < ARRAY_SIZE(params); i++) {
		if ((p = strsep(&options, ",")) != NULL) {
			if (*p)
				params[i] = simple_strtoul(p, &p, 10);
		} else
			return -EINVAL;
	}

	if ((mode->pclk = params[0]) == 0)
		return -EINVAL;

	mode->h_active      = params[1];
	mode->v_active      = params[2];
	mode->h_ref_to_sync = params[3];
	mode->v_ref_to_sync = params[4];
	mode->h_sync_width  = params[5];
	mode->v_sync_width  = params[6];
	mode->h_back_porch  = params[7];
	mode->v_back_porch  = params[8];
	mode->h_front_porch = params[9];
	mode->v_front_porch = params[10];

	return 0;
}

static int __init tegra_dc_mode_override(char *str)
{
	char *p = str, *options;

	if (!p || !*p)
		return -EINVAL;

	p = strstr(str, "hdmi:");
	if (p) {
		p += 5;
		options = strsep(&p, ";");
		if (parse_disp_params(options, &override_disp_mode[TEGRA_DC_OUT_HDMI]))
			return -EINVAL;
	}

	p = strstr(str, "rgb:");
	if (p) {
		p += 4;
		options = strsep(&p, ";");
		if (parse_disp_params(options, &override_disp_mode[TEGRA_DC_OUT_RGB]))
			return -EINVAL;
	}

	p = strstr(str, "dsi:");
	if (p) {
		p += 4;
		options = strsep(&p, ";");
		if (parse_disp_params(options, &override_disp_mode[TEGRA_DC_OUT_DSI]))
			return -EINVAL;
	}

	return 0;
}

__setup("disp_params=", tegra_dc_mode_override);
#endif

static int __init tegra_dc_module_init(void)
{
#ifdef CONFIG_TEGRA_DC_EXTENSIONS
	int ret = tegra_dc_ext_module_init();
	if (ret)
		return ret;
#endif
	return platform_driver_register(&tegra_dc_driver);
}

static void __exit tegra_dc_module_exit(void)
{
	platform_driver_unregister(&tegra_dc_driver);
#ifdef CONFIG_TEGRA_DC_EXTENSIONS
	tegra_dc_ext_module_exit();
#endif
}

module_exit(tegra_dc_module_exit);
module_init(tegra_dc_module_init);
