/*
 * virtual.c - Virtual kernel driver
 *
 * Copyright (c) 2013-2016, NVIDIA CORPORATION.  All rights reserved.

 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.

 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define CAMERA_DEVICE_INTERNAL

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/clk.h>

#include <media/nvc.h>
#include <media/camera.h>

struct chip_config {
	int clk_num;
	int gpio_num;
	int reg_num;
	struct camera_reg *seq_power_on;
	struct camera_reg *seq_power_off;
	char *reg_names[];
};

static int virtual_update(
	struct camera_device *cdev, struct cam_update *upd, u32 num)
{
	int err = 0;
	u32 idx;

	dev_dbg(cdev->dev, "%s %d\n", __func__, num);
	mutex_lock(&cdev->mutex);
	for (idx = 0; idx < num; idx++) {
		switch (upd[idx].type) {
		case UPDATE_CLOCK:
		{
			struct clk *ck;
			char *clk_name;

			if (!cdev->num_clk) {
				dev_err(cdev->dev, "NO clock needed.\n");
				err = -ENODEV;
				break;
			}
			if (upd[idx].index >= cdev->num_clk) {
				dev_err(cdev->dev,
					"clock index %d out of range.\n",
					upd[idx].index);
				err = -ENODEV;
				break;
			}

			clk_name = (void *)upd[idx].args;
			dev_dbg(cdev->dev, "%s UPDATE_CLOCK %d of %d, %s\n",
				__func__, upd[idx].index,
				cdev->num_clk, clk_name);
			ck = devm_clk_get(cdev->dev, clk_name);
			if (IS_ERR(ck)) {
				dev_err(cdev->dev, "%s: get clock %s FAILED.\n",
					__func__, clk_name);
				return PTR_ERR(ck);
			}
			cdev->clks[upd[idx].index] = ck;
			dev_dbg(cdev->dev, "UPDATE_CLOCK: %d %s\n",
				upd[idx].index, clk_name);
			break;
		}
		case UPDATE_PINMUX:
		{
			u32 *pinmux;

			if (!cdev->pinmux_num) {
				dev_err(cdev->dev, "NO pinmux available.\n");
				err = -ENODEV;
				break;
			}
			if (upd[idx].arg >= cdev->pinmux_num) {
				dev_err(cdev->dev,
					"pinmux index %u out of range.\n",
					upd[idx].arg);
				err = -ENODEV;
				break;
			}

			dev_dbg(cdev->dev, "UPDATE_PINMUX: %d %u\n",
				upd[idx].index, upd[idx].arg);
			if (!upd[idx].index)
				pinmux = &cdev->mclk_enable_idx;
			else
				pinmux = &cdev->mclk_disable_idx;
			*pinmux = upd[idx].arg;
			break;
		}
		default:
			dev_err(cdev->dev,
				"unsupported upd type %d\n", upd[idx].type);
			break;
		}

		if (err)
			break;
	}
	mutex_unlock(&cdev->mutex);
	return err;
}

static int virtual_power_on(struct camera_device *cdev)
{
	struct chip_config *c_info = cdev->chip->private;
	struct camera_reg *pwr_seq = c_info->seq_power_on;
	int err = 0;

	dev_dbg(cdev->dev, "%s %x %p\n",
		__func__, cdev->is_power_on, pwr_seq);
	if (cdev->is_power_on || !pwr_seq)
		return 0;

	mutex_lock(&cdev->mutex);
	err = camera_dev_wr_table(cdev, pwr_seq, NULL);
	if (!err)
		cdev->is_power_on = 1;
	mutex_unlock(&cdev->mutex);
	return err;
}

static int virtual_power_off(struct camera_device *cdev)
{
	struct chip_config *c_info = cdev->chip->private;
	struct camera_reg *pwr_seq = c_info->seq_power_off;
	int err = 0;

	dev_dbg(cdev->dev, "%s %x %p\n",
		__func__, cdev->is_power_on, pwr_seq);
	if (!cdev->is_power_on || !pwr_seq)
		return 0;

	mutex_lock(&cdev->mutex);
	err = camera_dev_wr_table(cdev, pwr_seq, NULL);
	if (!err)
		cdev->is_power_on = 0;
	mutex_unlock(&cdev->mutex);

	return err;
}

static int virtual_shutdown(struct camera_device *cdev)
{
	int err = 0;

	dev_dbg(cdev->dev, "%s %x\n", __func__, cdev->is_power_on);
	if (!cdev->is_power_on)
		return 0;

	if (!err)
		err = virtual_power_off(cdev);

	return err;
}

static int virtual_instance_destroy(struct camera_device *cdev)
{
	void *buf;
	u32 idx;

	dev_dbg(cdev->dev, "%s\n", __func__);

	if (!cdev->dev)
		return 0;

	buf = dev_get_drvdata(cdev->dev);
	dev_set_drvdata(cdev->dev, NULL);

	for (idx = 0; idx < cdev->num_reg; idx++)
		if (likely(cdev->regs[idx].vreg))
			regulator_put(cdev->regs[idx].vreg);

	cdev->num_gpio = 0;
	cdev->num_reg = 0;
	kfree(buf);
	return 0;
}

static int virtual_instance_create(struct camera_device *cdev, void *pdata)
{
	struct chip_config *c_info = cdev->chip->private;
	u32 idx;

	dev_dbg(cdev->dev, "%s\n", __func__);
	cdev->gpios = kzalloc(c_info->gpio_num * sizeof(*cdev->gpios) +
		c_info->reg_num * sizeof(*cdev->regs) +
		c_info->clk_num * sizeof(*cdev->clks),
		GFP_KERNEL);
	if (cdev->gpios == NULL) {
		dev_err(cdev->dev, "%s memory low!\n", __func__);
		return -ENOMEM;
	}

	cdev->pinmux_num = ((struct camera_platform_data *)pdata)->pinmux_num;
	cdev->pinmux_tbl = ((struct camera_platform_data *)pdata)->pinmux;
	cdev->num_gpio = c_info->gpio_num;
	cdev->regs = (void *)cdev->gpios +
		c_info->gpio_num * sizeof(*cdev->gpios);
	cdev->num_reg = c_info->reg_num;
	cdev->clks = (void *)cdev->regs + c_info->reg_num * sizeof(*cdev->regs);
	cdev->num_clk = c_info->clk_num;
	cdev->mclk_enable_idx = CAMDEV_INVALID;
	cdev->mclk_disable_idx = CAMDEV_INVALID;

	for (idx = 0; idx < cdev->num_gpio; idx++)
		cdev->gpios[idx].valid = false;

	for (idx = 0; idx < cdev->num_reg; idx++) {
		cdev->regs[idx].vreg_name =
			(const char *)c_info->reg_names[idx];
		camera_regulator_get(cdev->dev, &cdev->regs[idx],
			(char *)cdev->regs[idx].vreg_name);
	}

	dev_set_drvdata(cdev->dev, cdev->gpios);
	return 0;
}

static struct regmap_config regmap_cfg_default = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_NONE,
};

static int virtual_device_sanity_check(
	struct device *dev, struct virtual_device *dev_info, int *len)
{
	u32 num;
	u8 *nptr;
	int n;

	dev_dbg(dev, "%s: %s, bus type %d, addr bits %d, val bits %d\n",
		__func__, dev_info->name, dev_info->bus_type,
		dev_info->regmap_cfg.addr_bits, dev_info->regmap_cfg.val_bits);
	dev_dbg(dev, "gpios %d, regs %d, clks %d\n",
		dev_info->gpio_num, dev_info->reg_num, dev_info->clk_num);
	if (dev_info->name[0] == '\0') {
		dev_err(dev, "%s need a device name!\n", __func__);
		return -ENODEV;
	}

	if (dev_info->bus_type != CAMERA_DEVICE_TYPE_I2C) {
		dev_err(dev, "%s unsupported device type %d!\n",
		__func__, dev_info->bus_type);
		return -ENODEV;
	}

	if (dev_info->regmap_cfg.addr_bits != 0 &&
		dev_info->regmap_cfg.addr_bits != 8 &&
		dev_info->regmap_cfg.addr_bits != 16) {
		dev_err(dev, "%s unsupported address bits %d!\n",
		__func__, dev_info->regmap_cfg.addr_bits);
		return -ENODEV;
	}

	if (dev_info->regmap_cfg.val_bits != 8 &&
		dev_info->regmap_cfg.val_bits != 16) {
		dev_err(dev, "%s unsupported data bits %d!\n",
		__func__, dev_info->regmap_cfg.val_bits);
		return -ENODEV;
	}

	if (dev_info->regmap_cfg.cache_type != REGCACHE_NONE &&
		dev_info->regmap_cfg.cache_type != REGCACHE_RBTREE &&
		dev_info->regmap_cfg.cache_type != REGCACHE_COMPRESSED) {
		dev_err(dev, "%s unsupported cache type %d!\n",
		__func__, dev_info->regmap_cfg.cache_type);
		return -ENODEV;
	}

	if (dev_info->gpio_num >= ARCH_NR_GPIOS) {
		dev_err(dev, "%s too many gpios %d!\n",
		__func__, dev_info->gpio_num);
		return -ENODEV;
	}

	if (dev_info->gpio_num >= 6) {
		dev_notice(dev, "%s WHAT?! Are you sure you need %d gpios?\n",
			__func__, dev_info->gpio_num);
	}

	if (dev_info->clk_num >= 5) {
		dev_notice(dev, "%s WHAT?! Are you sure you need %d clocks?\n",
			__func__, dev_info->clk_num);
	}

	*len = 0;
	num = dev_info->reg_num;
	nptr = &dev_info->reg_names[0];
	while (num) {
		n = strlen(nptr);
		if (!n) {
			dev_err(dev, "%s NULL reg name @ %d\n",
				__func__, dev_info->reg_num - num);
			return -ENODEV;
		}
		*len += n + 1;
		nptr += CAMERA_MAX_NAME_LENGTH;
		num--;
	}
	dev_dbg(dev, "regulator name size: %d\n", *len);

	if (dev_info->pwr_on_size > VIRTUAL_DEV_MAX_POWER_SIZE) {
		dev_err(dev, "%s power on function size too big %d!\n",
		__func__, dev_info->pwr_on_size);
		return -ENODEV;
	}

	if (dev_info->pwr_off_size > VIRTUAL_DEV_MAX_POWER_SIZE) {
		dev_err(dev, "%s power off function size too big %d!\n",
		__func__, dev_info->pwr_off_size);
		return -ENODEV;
	}

	return 0;
}

static int virtual_chip_config(
	struct device *dev,
	struct virtual_device *dev_info,
	struct chip_config *c_info)
{
	char *rptr = (void *)c_info;
	char *nptr = dev_info->reg_names;
	int len;
	u32 idx;

	dev_dbg(dev, "%s regulators:\n", __func__);
	c_info->clk_num = dev_info->clk_num;
	c_info->gpio_num = dev_info->gpio_num;
	c_info->reg_num = dev_info->reg_num;
	rptr += sizeof(*c_info) + sizeof(char *) * c_info->reg_num;
	for (idx = 0; idx < c_info->reg_num; idx++) {
		c_info->reg_names[idx] = rptr;
		len = strlen(nptr);
		dev_dbg(dev, "#%d %s len %d\n", idx, nptr, len);
		strcpy(rptr, nptr);
		rptr[len] = '\0';
		rptr += len + 1;
		nptr += CAMERA_MAX_NAME_LENGTH;
		dev_dbg(dev, "#%d - %s\n", idx, c_info->reg_names[idx]);
	}

	c_info->seq_power_on = (void *)rptr;
	if (copy_from_user(
		c_info->seq_power_on,
		(const void __user *)(unsigned long)dev_info->power_on,
		sizeof(struct camera_reg) * dev_info->pwr_on_size)) {
		dev_err(dev, "%s copy_from_user err line %d\n",
			__func__, __LINE__);
			return -EFAULT;
	}

	c_info->seq_power_off = (void *)c_info->seq_power_on +
		sizeof(struct camera_reg) * dev_info->pwr_on_size;
	if (copy_from_user(
		c_info->seq_power_off,
		(const void __user *)(unsigned long)dev_info->power_off,
		sizeof(struct camera_reg) * dev_info->pwr_off_size)) {
		dev_err(dev, "%s copy_from_user err line %d\n",
			__func__, __LINE__);
			return -EFAULT;
	}

	return 0;
}

int virtual_device_add(struct device *dev, unsigned long arg)
{
	struct virtual_device dev_info;
	struct camera_chip *v_chip;
	struct chip_config *c_info;
	struct regmap_config *p_regmap;
	int buf_len;
	int err = 0;

#ifdef CONFIG_COMPAT
	struct virtual_device_32 dvi32;
	if (copy_from_user(
		&dvi32, (const void __user *)arg, sizeof(dvi32))) {
		dev_err(dev, "%s copy_from_user err line %d\n",
			__func__, __LINE__);
			return -EFAULT;
	}
	dev_info.power_on = (void *)((unsigned long)dvi32.power_on);
	dev_info.power_off = (void *)((unsigned long)dvi32.power_off);
	dev_info.regmap_cfg = dvi32.regmap_cfg;
	dev_info.bus_type = dvi32.bus_type;
	dev_info.gpio_num = dvi32.gpio_num;
	dev_info.reg_num = dvi32.reg_num;
	dev_info.pwr_on_size = dvi32.pwr_on_size;
	dev_info.pwr_off_size = dvi32.pwr_off_size;
	dev_info.clk_num = dvi32.clk_num;
	memcpy(dev_info.name, dvi32.name, sizeof(dvi32.name));
	memcpy(dev_info.reg_names, dvi32.reg_names,
		sizeof(dvi32.reg_names));
#else
	if (copy_from_user(
		&dev_info, (const void __user *)arg, sizeof(dev_info))) {
		dev_err(dev, "%s copy_from_user err line %d\n",
			__func__, __LINE__);
			return -EFAULT;
	}
#endif

	dev_info(dev, "%s\n", __func__);
	err = virtual_device_sanity_check(dev, &dev_info, &buf_len);
	if (err)
		return err;

	buf_len += sizeof(char *) * dev_info.reg_num +
		sizeof(struct camera_reg) * dev_info.pwr_on_size +
		sizeof(struct camera_reg) * dev_info.pwr_off_size;
	v_chip = kzalloc(
		sizeof(*v_chip) + sizeof(*c_info) + buf_len, GFP_KERNEL);
	if (!v_chip) {
		dev_err(dev, "%s unable to allocate memory!\n", __func__);
		return -ENOMEM;
	}

	c_info = (void *)v_chip + sizeof(*v_chip);
	err = virtual_chip_config(dev, &dev_info, c_info);
	if (err) {
		kfree(v_chip);
		return err;
	}

	strncpy((u8 *)v_chip->name, (u8 const *)dev_info.name,
		sizeof(v_chip->name));

	p_regmap = (struct regmap_config *)&v_chip->regmap_cfg;
	memcpy(p_regmap, &regmap_cfg_default, sizeof(*p_regmap));
	v_chip->type = dev_info.bus_type;
	if (dev_info.regmap_cfg.addr_bits)
		p_regmap->reg_bits = dev_info.regmap_cfg.addr_bits;
	if (dev_info.regmap_cfg.val_bits)
		p_regmap->val_bits = dev_info.regmap_cfg.val_bits;
	p_regmap->cache_type = dev_info.regmap_cfg.cache_type;

	INIT_LIST_HEAD(&v_chip->list);
	v_chip->private = c_info;
	v_chip->init = virtual_instance_create;
	v_chip->release = virtual_instance_destroy,
	v_chip->power_on = virtual_power_on,
	v_chip->power_off = virtual_power_off,
	v_chip->shutdown = virtual_shutdown,
	v_chip->update = virtual_update,

	err = camera_chip_add(v_chip);
	if (err) {
		kfree(v_chip);
		if (err == -EEXIST)
			err = 0;
	}

	return err;
}

static int __init virtual_init(void)
{
	pr_info("%s\n", __func__);
	return 0;
}
device_initcall(virtual_init);

static void __exit virtual_exit(void)
{
}
module_exit(virtual_exit);

MODULE_DESCRIPTION("virtual sensor device");
MODULE_AUTHOR("Charlie Huang <chahuang@nvidia.com>");
MODULE_LICENSE("GPL v2");
