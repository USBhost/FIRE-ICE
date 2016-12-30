/*
 *  include/linux/touchboost.h
 *
 * franciscofranco.1990@gmail.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _LINUX_TOUCHBOOST_H
#define _LINUX_TOUCHBOOST_H

#include <linux/types.h>

/**
 * Determines if touchboost is active for a local module.
 * @local_dur the touchboost duration of a querying module in us.
 */
bool touchboost_is_enabled(u64 local_duration);
#endif
