/* include/linux/wlan_plat.h
 *
 * Copyright (c) 2010 Google, Inc.
 * Copyright (c) 2013 NVIDIA Corporation. All rights reserved.
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
#ifndef _LINUX_WLAN_PLAT_H_
#define _LINUX_WLAN_PLAT_H_

#include <linux/sysedp.h>

#define WLAN_PLAT_NODFS_FLAG	0x01

struct wifi_platform_data {
	int (*set_power)(int val);
	int (*set_reset)(int val);
	int (*set_carddetect)(int val);
	void *(*mem_prealloc)(int section, unsigned long size);
	int (*get_mac_addr)(unsigned char *buf);
	void *(*get_country_code)(char *ccode, u32 flags);
	struct sysedp_consumer *sysedpc;
};

#endif
