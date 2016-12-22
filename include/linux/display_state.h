/*
 *  include/linux/display_state.h
 *
 *  Copyright (C)  2016 Dela Anthonio
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/notifier.h>

#define DISPLAY_OFF 0
#define DISPLAY_ON 1

/**
 * Registers a new notifier block
 */
 void display_state_register_notifier(struct notifier_block *nb);

 /**
  * Unregisters a notifier block
  */
 void display_state_unregister_notifier(struct notifier_block *nb);