/*
 * camera.c - generic camera device driver
 *
 * Copyright (c) 2013-2014, NVIDIA CORPORATION.  All rights reserved.
 *
 * Contributors:
 *	Charlie Huang <chahuang@nvidia.com>
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

/* Implementation
 * --------------
 * The board level details about the device are to be provided in the board
 * file with the <device>_platform_data structure.
 * Standard among NVC kernel drivers in this structure is:
 * .dev_name = The MISC driver name the device registers as.  If not used,
 *	     then the part number of the device is used for the driver name.
 *	     If using the NVC user driver then use the name found in this
 *	     driver under _default_pdata.
 */

#define CAMERA_DEVICE_INTERNAL
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/debugfs.h>
#include <linux/clk.h>

#include "t124/t124.h"
#include <isp.h>
#include <media/camera.h>

static struct camera_platform_data camera_dflt_pdata = {
	.cfg = 0,
};

static atomic_t cam_ref;
static DEFINE_MUTEX(cam_mutex);

static DEFINE_MUTEX(app_mutex);
static DEFINE_MUTEX(dev_mutex);
static DEFINE_MUTEX(chip_mutex);
static LIST_HEAD(app_list);
static LIST_HEAD(dev_list);
static LIST_HEAD(chip_list);

static struct camera_platform_info cam_desc = {
	.in_use = ATOMIC_INIT(0),
	.u_mutex = &app_mutex,
	.d_mutex = &dev_mutex,
	.c_mutex = &chip_mutex,
	.app_list = &app_list,
	.dev_list = &dev_list,
	.chip_list = &chip_list,
};

static inline void camera_ref_init(void)
{
	atomic_set(&cam_ref, 0);
}

static int camera_ref_raise(void)
{
	mutex_lock(&cam_mutex);
	if (atomic_read(&cam_ref) < 0) {
		mutex_unlock(&cam_mutex);
		dev_err(cam_desc.dev, "%s - CAMERA DOWN.\n", __func__);
		return -ENOTTY;
	}
	atomic_inc(&cam_ref);
	mutex_unlock(&cam_mutex);

	return 0;
}

static inline void camera_ref_down(void)
{
	atomic_dec(&cam_ref);
}

static void camera_ref_lock(void)
{
	int ref;

	do {
		mutex_lock(&cam_mutex);
		ref = atomic_read(&cam_ref);
		if (ref <= 0) {
			atomic_set(&cam_ref, -1);
			mutex_unlock(&cam_mutex);
			break;
		}
		mutex_unlock(&cam_mutex);
		usleep_range(10000, 10200);
	} while (true);
}

#ifdef CONFIG_COMPAT
int camera_copy_user_params(unsigned long arg, struct nvc_param *prm)
{
	struct nvc_param_32 p32;

	memcpy(&p32, prm, sizeof(p32));
	p32.p_value = (u32)prm->p_value;

	return copy_to_user(
		(void __user *)arg, (const void *)&p32, sizeof(p32));
}
#else
int camera_copy_user_params(unsigned long arg, struct nvc_param *prm)
{
	return copy_to_user(
		MAKE_USER_PTR(arg), (const void *)prm, sizeof(*prm));
}
#endif

int __camera_get_params(
	struct camera_info *cam, unsigned long arg, int u_size,
	struct nvc_param *prm, void **data, bool zero_size_ok)
{
	void *buf;
	size_t size;

#ifdef CONFIG_COMPAT
	memset(prm, 0, sizeof(*prm));
	if (copy_from_user(
		prm, (const void __user *)arg, sizeof(struct nvc_param_32))) {
		dev_err(cam->dev, "%s copy_from_user err line %d\n",
			__func__, __LINE__);
		return -EFAULT;
	}
#else
	if (copy_from_user(prm, (const void __user *)arg, sizeof(*prm))) {
		dev_err(cam->dev, "%s copy_from_user err line %d\n",
			__func__, __LINE__);
		return -EFAULT;
	}
#endif
	if (!data)
		return 0;

	if (zero_size_ok && prm->sizeofvalue == 0) {
		*data = ZERO_SIZE_PTR;
		return 0;
	}

	size = prm->sizeofvalue * u_size;
	buf = kcalloc(prm->sizeofvalue, u_size, GFP_KERNEL);
	if (ZERO_OR_NULL_PTR(buf)) {
		dev_err(cam->dev, "%s allocate memory failed!\n", __func__);
		return -ENOMEM;
	}
	if (copy_from_user(buf, MAKE_CONSTUSER_PTR(prm->p_value), size)) {
		dev_err(cam->dev, "%s copy_from_user err line %d\n",
			__func__, __LINE__);
		kfree(buf);
		return -EFAULT;
	}
	*data = buf;

	return 0;
}

static int camera_validate_p_i2c_table(struct camera_info *cam,
		const struct nvc_param *params,
		const struct camera_reg *p_i2c_table, const char *caller)
{
	u32 idx, last_idx = params->sizeofvalue / sizeof(p_i2c_table[0]);

	for (idx = 0; idx < last_idx; idx++)
		if (p_i2c_table[idx].addr == CAMERA_TABLE_END)
			return 0;

	dev_err(cam->dev, "%s: table is not properly terminated\n", caller);
	return -EINVAL;
}

static int camera_seq_rd(struct camera_info *cam, unsigned long arg)
{
	struct nvc_param params;
	struct camera_reg *p_i2c_table;
	int err;

	dev_dbg(cam->dev, "%s %lx\n", __func__, arg);
	err = camera_get_params(cam, arg, 1, &params, (void **)&p_i2c_table);
	if (err)
		return err;

	err = camera_validate_p_i2c_table(cam, &params, p_i2c_table, __func__);
	if (err)
		goto seq_rd_end;

	err = camera_dev_rd_table(cam->cdev, p_i2c_table);
	if (!err && copy_to_user(MAKE_USER_PTR(params.p_value),
		p_i2c_table, params.sizeofvalue)) {
		dev_err(cam->dev, "%s copy_to_user err line %d\n",
			__func__, __LINE__);
		err = -EINVAL;
	}

seq_rd_end:
	kfree(p_i2c_table);
	return err;
}

static int camera_seq_wr(struct camera_info *cam, unsigned long arg)
{
	struct nvc_param params;
	struct camera_device *cdev = cam->cdev;
	struct camera_reg *p_i2c_table = NULL;
	struct camera_seq_status seqs;
	u8 pfree = 0;
	int err = 0;
	int idx;

	dev_dbg(cam->dev, "%s %lx", __func__, arg);

	err = camera_get_params(cam, arg, 0, &params, NULL);
	if (err)
		return err;

	dev_dbg(cam->dev, "param: %x, size %d\n", params.param,
		params.sizeofvalue);
	if (params.param == CAMERA_SEQ_EXIST) {
		idx = params.variant & CAMERA_SEQ_INDEX_MASK;
		if (idx >= NUM_OF_SEQSTACK) {
			dev_err(cam->dev, "%s seq index out of range %d\n",
				__func__, idx);
			return -EFAULT;
		}
		p_i2c_table = cdev->seq_stack[idx];
		if (p_i2c_table == NULL) {
			dev_err(cam->dev, "%s seq index empty! %d\n",
				__func__, idx);
			return -EEXIST;
		}
		goto seq_wr_table;
	}

	p_i2c_table = devm_kzalloc(cdev->dev, params.sizeofvalue, GFP_KERNEL);
	if (ZERO_OR_NULL_PTR(p_i2c_table)) {
		dev_err(cam->dev, "%s devm_kzalloc err line %d\n",
			__func__, __LINE__);
		return -ENOMEM;
	}
	pfree = 1;

	if (copy_from_user(p_i2c_table,
		MAKE_CONSTUSER_PTR(params.p_value), params.sizeofvalue)) {
		dev_err(cam->dev, "%s copy_from_user err line %d\n",
			__func__, __LINE__);
		err = -EFAULT;
		goto seq_wr_end;
	}

	err = camera_validate_p_i2c_table(cam, &params, p_i2c_table, __func__);
	if (err)
		goto seq_wr_end;

	switch (params.param) {
	case CAMERA_SEQ_REGISTER_EXEC:
	case CAMERA_SEQ_REGISTER_ONLY:
		for (idx = 0; idx < NUM_OF_SEQSTACK; idx++) {
			if (!cdev->seq_stack[idx]) {
				cdev->seq_stack[idx] = p_i2c_table;
				pfree = 0;
				break;
			}
		}
		if (idx >= NUM_OF_SEQSTACK) {
			dev_err(cam->dev, "%s seq index full!\n", __func__);
			err = -EINVAL;
			goto seq_wr_end;
		} else {
			params.variant = idx;
			goto seq_wr_upd;
		}
		if (params.param == CAMERA_SEQ_REGISTER_EXEC)
			goto seq_wr_table;
		break;
	case CAMERA_SEQ_EXEC:
		break;
	}

seq_wr_table:
	if (err < 0)
		goto seq_wr_end;

	memset(&seqs, 0, sizeof(seqs));
	mutex_lock(&cdev->mutex);
	err = camera_dev_wr_table(cdev, p_i2c_table, &seqs);
	mutex_unlock(&cdev->mutex);
	if (err < 0) {
		params.param = CAMERA_SEQ_STATUS_MASK | seqs.idx;
		params.variant = seqs.status;
	}

seq_wr_upd:
	if (camera_copy_user_params(arg, &params)) {
		dev_err(cam->dev, "%s copy_to_user err line %d\n",
			__func__, __LINE__);
		err = -EFAULT;
	}

seq_wr_end:
	if (pfree) {
		/* if table has been updated, send it back */
		if (err > 0 && copy_to_user(MAKE_USER_PTR(params.p_value),
			p_i2c_table, params.sizeofvalue)) {
			dev_err(cam->dev, "%s copy_to_user err line %d\n",
				__func__, __LINE__);
			err = -EFAULT;
		}
		devm_kfree(cdev->dev, p_i2c_table);
	}
	return err;
}

static int camera_dev_pwr_set(struct camera_info *cam, unsigned long pwr)
{
	struct camera_device *cdev = cam->cdev;
	struct camera_chip *chip = cdev->chip;
	int err = 0;

	dev_dbg(cam->dev, "%s %lu %d\n", __func__, pwr, cdev->pwr_state);
	if (pwr == cdev->pwr_state)	/* power state no change */
		goto dev_power_end;

	switch (pwr) {
	case NVC_PWR_OFF:
	case NVC_PWR_STDBY_OFF:
		if (chip->power_off)
			err |= chip->power_off(cdev);
		camera_edp_lowest(cdev);
		break;
	case NVC_PWR_STDBY:
	case NVC_PWR_COMM:
	case NVC_PWR_ON:
		if (chip->power_on)
			err = chip->power_on(cdev);
		break;
	default:
		err = -EINVAL;
		break;
	}

	if (err < 0) {
		dev_err(cam->dev, "%s error\n", __func__);
		pwr = NVC_PWR_ERR;
	}

	cdev->pwr_state = pwr;
	if (err > 0)
		err = 0;

dev_power_end:
	return err;
}

static int camera_dev_pwr_get(struct camera_info *cam, unsigned long arg)
{
	int pwr;
	int err = 0;

	pwr = cam->cdev->pwr_state;
	dev_dbg(cam->dev, "%s PWR_RD: %d\n", __func__, pwr);
	if (copy_to_user((void __user *)arg, (const void *)&pwr,
		sizeof(pwr))) {
		dev_err(cam->dev, "%s copy_to_user err line %d\n",
			__func__, __LINE__);
		err = -EFAULT;
	}

	return err;
}

static struct camera_chip *camera_chip_chk(char *name)
{
	struct camera_chip *ccp;

	mutex_lock(cam_desc.c_mutex);
	list_for_each_entry(ccp, cam_desc.chip_list, list)
		if (!strcmp(ccp->name, name)) {
			dev_dbg(cam_desc.dev, "%s device %s found.\n",
				__func__, name);
			mutex_unlock(cam_desc.c_mutex);
			return ccp;
		}
	mutex_unlock(cam_desc.c_mutex);
	return NULL;
}

int camera_chip_add(struct camera_chip *chip)
{
	struct camera_chip *ccp;
	int err = 0;

	dev_dbg(cam_desc.dev, "%s add chip: %s\n", __func__, chip->name);
	mutex_lock(cam_desc.c_mutex);
	list_for_each_entry(ccp, cam_desc.chip_list, list)
		if (!strcmp(ccp->name, chip->name)) {
			dev_notice(cam_desc.dev, "%s chip %s already added.\n",
				__func__, chip->name);
			mutex_unlock(cam_desc.c_mutex);
			return -EEXIST;
		}

	list_add_tail(&chip->list, cam_desc.chip_list);
	mutex_unlock(cam_desc.c_mutex);
	return err;
}
EXPORT_SYMBOL_GPL(camera_chip_add);

static int camera_remove_device(struct camera_device *cdev, bool ref_dec)
{
	dev_dbg(cdev->dev, "%s %s\n", __func__, cdev->name);
	if (!cdev) {
		dev_err(cam_desc.dev, "%s cdev is NULL!\n", __func__);
		return -EFAULT;
	}

	WARN_ON(atomic_xchg(&cdev->in_use, 0));
	if (cdev->cam) {
		struct camera_info *cam = cdev->cam;
		cam->cdev = NULL;
		cam->dev = cam_desc.dev;
		atomic_set(&cam->in_use, 0);
	}
	if (cdev->chip) {
		(cdev->chip->release)(cdev);
		if (ref_dec)
			atomic_dec(&cdev->chip->ref_cnt);
	}
	if (cdev->dev)
		i2c_unregister_device(to_i2c_client(cdev->dev));
	kfree(cdev);
	return 0;
}

static int camera_free_device(struct camera_info *cam, unsigned long arg)
{
	struct camera_device *cdev = cam->cdev;

	dev_dbg(cam->dev, "%s %lx", __func__, arg);
	mutex_lock(cam_desc.d_mutex);
	atomic_set(&cdev->in_use, 0);
	mutex_unlock(cam_desc.d_mutex);

	return 0;
}

static int camera_new_device(struct camera_info *cam, unsigned long arg)
{
	struct camera_device_info dev_info;
	struct camera_device *new_dev;
	struct camera_device *next_dev;
	struct camera_chip *c_chip;
	struct i2c_adapter *adap;
	struct i2c_board_info brd;
	struct i2c_client *client;
	int err = 0;

	dev_dbg(cam->dev, "%s %lx", __func__, arg);
	if (copy_from_user(
		&dev_info, (const void __user *)arg, sizeof(dev_info))) {
		dev_err(cam_desc.dev, "%s copy_from_user err line %d\n",
			__func__, __LINE__);
		err = -EFAULT;
		goto new_device_end;
	}

	dev_dbg(cam->dev, "%s - %d %d %x\n",
		dev_info.name, dev_info.type, dev_info.bus, dev_info.addr);

	c_chip = camera_chip_chk(dev_info.name);
	if (c_chip == NULL) {
		dev_err(cam->dev, "%s device %s not found\n",
			__func__, dev_info.name);
		err = -ENODEV;
		goto new_device_end;
	}

	adap = i2c_get_adapter(dev_info.bus);
	if (!adap) {
		dev_err(cam_desc.dev, "%s no such i2c bus %d\n",
			__func__, dev_info.bus);
		err = -ENODEV;
		goto new_device_end;
	}

	new_dev = kzalloc(sizeof(*new_dev), GFP_KERNEL);
	if (!new_dev) {
		dev_err(cam_desc.dev, "%s memory low!\n", __func__);
		err = -ENOMEM;
		goto new_device_end;
	}

	new_dev->chip = c_chip;
	memset(&brd, 0, sizeof(brd));
	strncpy(brd.type, dev_info.name, sizeof(brd.type));
	brd.addr = dev_info.addr;

	mutex_lock(cam_desc.d_mutex);

	/* check if device exists, if yes and not occupied pick it and exit */
	list_for_each_entry(next_dev, cam_desc.dev_list, list) {
		if (next_dev->client &&
			next_dev->client->adapter == adap &&
			next_dev->client->addr == dev_info.addr) {
			dev_dbg(cam_desc.dev,
				"%s: device already exists.\n", __func__);
			camera_remove_device(new_dev, false);
			if (atomic_xchg(&next_dev->in_use, 1)) {
				dev_err(cam_desc.dev, "%s device %s BUSY\n",
					__func__, next_dev->name);
				err = -EBUSY;
				goto new_device_err;
			}
			new_dev = next_dev;
			goto new_device_done;
		}
	}

	/* device is not present in the dev_list, add it */
	client = i2c_new_device(adap, &brd);
	if (!client) {
		dev_err(cam_desc.dev,
			"%s cannot allocate client: %s bus %d, %x\n",
			__func__, dev_info.name, dev_info.bus, dev_info.addr);
		err = -EINVAL;
		goto new_device_err;
	}
	new_dev->dev = &client->dev;

	new_dev->regmap = devm_regmap_init_i2c(client, &c_chip->regmap_cfg);
	if (IS_ERR(new_dev->regmap)) {
		dev_err(new_dev->dev, "%s regmap init failed: %ld\n",
			__func__, PTR_ERR(new_dev->regmap));
		err = -ENODEV;
		goto new_device_err;
	}
	strncpy(new_dev->name, dev_info.name, sizeof(new_dev->name));
	INIT_LIST_HEAD(&new_dev->list);
	mutex_init(&new_dev->mutex);
	new_dev->client = client;

	err = (new_dev->chip->init)(new_dev, cam_desc.pdata);
	if (err)
		goto new_device_err;

	atomic_inc(&c_chip->ref_cnt);
	atomic_set(&new_dev->in_use, 1);

	list_add(&new_dev->list, cam_desc.dev_list);

new_device_done:
	mutex_unlock(cam_desc.d_mutex);

	cam->cdev = new_dev;
	cam->dev = new_dev->dev;
	new_dev->cam = cam;
	goto new_device_end;

new_device_err:
	mutex_unlock(cam_desc.d_mutex);
	camera_remove_device(new_dev, false);

new_device_end:
	return err;
}

static void camera_app_remove(struct camera_info *cam, bool ref_chk)
{
	dev_dbg(cam->dev, "%s\n", __func__);

	if (ref_chk)
		WARN_ON(atomic_xchg(&cam->in_use, 0));
	if (cam->cdev) {
		if (ref_chk)
			WARN_ON(atomic_xchg(&cam->cdev->in_use, 0));
		cam->cdev->cam = NULL;
	}
	kfree(cam);
}

static int camera_update(struct camera_info *cam, unsigned long arg)
{
	struct camera_device *cdev = cam->cdev;
	struct camera_chip *chip = cdev->chip;
	struct nvc_param param;
	struct cam_update *upd = NULL;
	int err = 0;

	dev_dbg(cam->dev, "%s %lx", __func__, arg);
	if (!chip->update) {
		dev_dbg(cam->dev, "no update pointer.\n");
		return err;
	}

	err = __camera_get_params(cam, arg, sizeof(*upd), &param, (void **)&upd,
			true);
	if (err)
		return err;

	err = chip->update(cdev, upd, param.sizeofvalue);

	kfree(upd);
	return err;
}

/* need this feature for auto detect to display notifications */
static int camera_msg(struct camera_info *cam, unsigned long arg)
{
	struct nvc_param param;
	char *str;
	int err = 0;

	dev_dbg(cam->dev, "%s %lx", __func__, arg);
	err = camera_get_params(cam, arg, 1, &param, (void **)&str);
	if (err)
		return err;
	if (str[param.sizeofvalue - 1] == '\0')
		dev_info(cam->dev, "%s\n", str);
	kfree(str);

	return 0;
}

static int camera_layout_update(struct camera_info *cam, unsigned long arg)
{
	struct nvc_param param;
	void *upd = NULL;
	int err = 0;

	dev_dbg(cam->dev, "%s %lx", __func__, arg);
	mutex_lock(cam_desc.u_mutex);
	if (cam_desc.layout) {
		dev_notice(cam->dev, "layout already there.\n");
		err = -EEXIST;
		goto layout_end;
	}

	err = camera_get_params(cam, arg, 1, &param, &upd);
	if (err)
		goto layout_end;

	cam_desc.layout = upd;
	cam_desc.size_layout = param.sizeofvalue;

layout_end:
	mutex_unlock(cam_desc.u_mutex);
	return err;
}

static int camera_layout_get(struct camera_info *cam, unsigned long arg)
{
	struct nvc_param param;
	int len;
	int err = 0;

	dev_dbg(cam->dev, "%s %lx", __func__, arg);
	if (!cam_desc.layout) {
		dev_notice(cam->dev, "layout empty.\n");
		err = -EEXIST;
		goto getlayout_end;
	}

	err = camera_get_params(cam, arg, 0, &param, NULL);
	if (err)
		return err;

	if (param.variant > MAX_PARAM_VARIANT) {
		dev_err(cam->dev, "%s param variant is too large: %u\n",
		__func__, param.variant);
		return -EINVAL;
	}
	if (param.sizeofvalue > MAX_PARAM_SIZE_OF_VALUE) {
		dev_err(cam->dev, "%s size of param value is too large: %u\n",
		__func__, param.sizeofvalue);
		return -EINVAL;
	}

	len = (int)cam_desc.size_layout - param.variant;
	if (len <= 0) {
		dev_err(cam->dev, "%s invalid offset %u\n",
			__func__, param.variant);
		err = -EINVAL;
		goto getlayout_end;
	}
	if (len > param.sizeofvalue) {
		len = param.sizeofvalue;
		err = -EAGAIN;
	}
	if (copy_to_user(MAKE_USER_PTR(param.p_value),
		cam_desc.layout + param.variant, len)) {
		dev_err(cam->dev, "%s copy_to_user err line %d\n",
			__func__, __LINE__);
		err = -EFAULT;
		goto getlayout_end;
	}

	param.sizeofvalue = len;
	param.variant = cam_desc.size_layout;
	if (camera_copy_user_params(arg, &param)) {
		dev_err(cam->dev, "%s copy_to_user err line %d\n",
			__func__, __LINE__);
		err = -EFAULT;
	}

getlayout_end:
	return err;
}

static int camera_add_dev_drv(
	struct camera_info *cam,
	struct i2c_adapter *adap,
	struct camera_board *cb)
{
	struct i2c_client *client = NULL;
	struct i2c_board_info *bi = NULL;

	bi = cb->bi;
	if (bi && strlen(bi->type)) {
		if (!adap)
			adap = i2c_get_adapter(cb->busnum);
		dev_dbg(cam->dev, "%s: installing %s @ %d-%04x\n",
			__func__, bi->type, adap->nr, bi->addr);
		client = i2c_new_device(adap, bi);
		if (!client) {
			dev_err(cam->dev, "%s add driver %s fail\n",
				__func__, bi->type);
			return -EFAULT;
		}
	}
	return 0;
}

static int camera_add_drv_by_sensor_name(
	struct camera_info *cam, struct nvc_param *param)
{
	struct camera_module *cm = cam_desc.pdata->modules;
	struct i2c_board_info *bi = NULL;
	char ref_name[CAMERA_MAX_NAME_LENGTH];
	int err = 0;

	if (param->sizeofvalue > sizeof(ref_name) - 1) {
		dev_err(cam->dev, "%s driver name too long %d\n",
			__func__, param->sizeofvalue);
		err = -EFAULT;
		goto add_sensor_driver_end;
	}

	memset(ref_name, 0, sizeof(ref_name));
	if (copy_from_user(ref_name, MAKE_CONSTUSER_PTR(param->p_value),
		param->sizeofvalue)) {
		dev_err(cam->dev, "%s copy_from_user err line %d\n",
			__func__, __LINE__);
		err = -EFAULT;
		goto add_sensor_driver_end;
	}

	while (cm) {
		bi = cm->sensor.bi;
		if (!bi || !strlen(bi->type))
			break;

		dev_dbg(cam->dev, "%s\n", bi->type);
		if (!strcmp(bi->type, ref_name)) {
			err = camera_add_dev_drv(cam, NULL, &cm->sensor);
			if (err)
				break;

			err = camera_add_dev_drv(cam, NULL, &cm->focuser);
			if (err)
				break;

			err = camera_add_dev_drv(cam, NULL, &cm->flash);
			if (err)
				break;
		}
		cm++;
	}

add_sensor_driver_end:
	return err;
}

static int camera_add_drv_by_module(
	struct camera_info *cam, struct nvc_param *param)
{
	struct camera_module *cm;
	int err;

	if (!cam_desc.pdata->modules ||
		param->variant >= cam_desc.pdata->mod_num) {
		dev_err(cam->dev, "%s module %p %d not exists\n",
			__func__, cam_desc.pdata->modules, param->variant);
		return -ENODEV;
	}
	dev_dbg(cam->dev, "%s install module %d\n", __func__, param->variant);
	cm = &cam_desc.pdata->modules[param->variant];

	err = camera_add_dev_drv(cam, NULL, &cm->sensor);
	if (err)
		return err;

	err = camera_add_dev_drv(cam, NULL, &cm->focuser);
	if (err)
		return err;

	err = camera_add_dev_drv(cam, NULL, &cm->flash);
	return err;
}

static int camera_add_drivers(struct camera_info *cam, unsigned long arg)
{
	struct nvc_param param;
	int err;

	dev_dbg(cam->dev, "%s %lx", __func__, arg);
	err = camera_get_params(cam, arg, 0, &param, NULL);
	if (err)
		return err;

	if (param.param == 0)
		return camera_add_drv_by_sensor_name(cam, &param);
	return camera_add_drv_by_module(cam, &param);
}

static long camera_ioctl(struct file *file,
			 unsigned int cmd,
			 unsigned long arg)
{
	struct camera_info *cam;
	int err = 0;

	if (camera_ref_raise())
		return -ENOTTY;

	cam = file->private_data;
	if (!cam->cdev && ((cmd == PCLLK_IOCTL_SEQ_WR) ||
		(cmd == PCLLK_IOCTL_PWR_WR) ||
		(cmd == PCLLK_IOCTL_PWR_RD))) {
		dev_err(cam_desc.dev, "%s %x - no device activated.\n",
			__func__, cmd);
		err = -ENODEV;
		goto ioctl_end;
	}

	/* command distributor */
	switch (cmd) {
	case PCLLK_IOCTL_CHIP_REG:
		err = virtual_device_add(cam_desc.dev, arg);
		break;
	case PCLLK_IOCTL_DEV_REG:
		err = camera_new_device(cam, arg);
		break;
	case PCLLK_IOCTL_DEV_DEL:
		mutex_lock(cam_desc.d_mutex);
		if (!cam->cdev) {
			err = -ENODEV;
			mutex_unlock(cam_desc.d_mutex);
			break;
		}
		list_del(&cam->cdev->list);
		camera_remove_device(cam->cdev, true);
		mutex_unlock(cam_desc.d_mutex);
		break;
	case PCLLK_IOCTL_DEV_FREE:
		err = camera_free_device(cam, arg);
		break;
	case PCLLK_IOCTL_SEQ_WR:
		mutex_lock(cam_desc.d_mutex);
		err = camera_seq_wr(cam, arg);
		mutex_unlock(cam_desc.d_mutex);
		break;
	case PCLLK_IOCTL_SEQ_RD:
		mutex_lock(cam_desc.d_mutex);
		err = camera_seq_rd(cam, arg);
		mutex_unlock(cam_desc.d_mutex);
		break;
	case PCLLK_IOCTL_PARAM_RD:
		/* err = camera_param_rd(cam, arg); */
		break;
	case PCLLK_IOCTL_PWR_WR:
		/* This is a Guaranteed Level of Service (GLOS) call */
		mutex_lock(cam_desc.d_mutex);
		err = camera_dev_pwr_set(cam, arg);
		mutex_unlock(cam_desc.d_mutex);
		break;
	case PCLLK_IOCTL_PWR_RD:
		mutex_lock(cam_desc.d_mutex);
		err = camera_dev_pwr_get(cam, arg);
		mutex_unlock(cam_desc.d_mutex);
		break;
	case PCLLK_IOCTL_UPDATE:
		mutex_lock(cam_desc.d_mutex);
		err = camera_update(cam, arg);
		mutex_unlock(cam_desc.d_mutex);
		break;
	case PCLLK_IOCTL_LAYOUT_WR:
		err = camera_layout_update(cam, arg);
		break;
	case PCLLK_IOCTL_LAYOUT_RD:
		err = camera_layout_get(cam, arg);
		break;
	case PCLLK_IOCTL_DRV_ADD:
		err = camera_add_drivers(cam, arg);
		break;
	case PCLLK_IOCTL_DT_GET:
		err = of_camera_get_property(cam, arg);
		break;
	case PCLLK_IOCTL_MSG:
		err = camera_msg(cam, arg);
		break;
#ifdef CONFIG_COMPAT
	case PCLLK_IOCTL_32_CHIP_REG:
		err = virtual_device_add(cam_desc.dev, arg);
		break;
	case PCLLK_IOCTL_32_SEQ_WR:
		mutex_lock(cam_desc.d_mutex);
		err = camera_seq_wr(cam, arg);
		mutex_unlock(cam_desc.d_mutex);
		break;
	case PCLLK_IOCTL_32_SEQ_RD:
		mutex_lock(cam_desc.d_mutex);
		err = camera_seq_rd(cam, arg);
		mutex_unlock(cam_desc.d_mutex);
		break;
	case PCLLK_IOCTL_32_PARAM_RD:
		/* err = camera_param_rd(cam, arg); */
		break;
	case PCLLK_IOCTL_32_UPDATE:
		mutex_lock(cam_desc.d_mutex);
		err = camera_update(cam, arg);
		mutex_unlock(cam_desc.d_mutex);
		break;
	case PCLLK_IOCTL_32_LAYOUT_WR:
		err = camera_layout_update(cam, arg);
		break;
	case PCLLK_IOCTL_32_LAYOUT_RD:
		err = camera_layout_get(cam, arg);
		break;
	case PCLLK_IOCTL_32_DRV_ADD:
		err = camera_add_drivers(cam, arg);
		break;
	case PCLLK_IOCTL_32_DT_GET:
		err = of_camera_get_property(cam, arg);
		break;
	case PCLLK_IOCTL_32_MSG:
		err = camera_msg(cam, arg);
		break;
#endif
	default:
		dev_err(cam->dev, "%s unsupported ioctl: %x\n",
			__func__, cmd);
		err = -EINVAL;
	}

ioctl_end:
	camera_ref_down();
	if (err)
		dev_dbg(cam->dev, "err = %d\n", err);

	return err;
}

static int camera_open(struct inode *inode, struct file *file)
{
	struct camera_info *cam;

	if (camera_ref_raise())
		return -ENOTTY;

	cam = kzalloc(sizeof(*cam), GFP_KERNEL);
	if (!cam) {
		camera_ref_down();
		dev_err(cam_desc.dev,
			"%s unable to allocate memory!\n", __func__);
		return -ENOMEM;
	}

	mutex_init(&cam->k_mutex);
	atomic_set(&cam->in_use, 0);
	INIT_LIST_HEAD(&cam->list);
	cam->dev = cam_desc.dev;
	file->private_data = cam;

	mutex_lock(cam_desc.u_mutex);
	list_add(&cam->list, cam_desc.app_list);
	mutex_unlock(cam_desc.u_mutex);

	camera_ref_down();
	dev_dbg(cam_desc.dev, "%s\n", __func__);
	return 0;
}

static int camera_release(struct inode *inode, struct file *file)
{
	struct camera_info *cam;

	dev_dbg(cam_desc.dev, "%s\n", __func__);

	if (camera_ref_raise())
		return -ENOTTY;

	cam = file->private_data;
	mutex_lock(cam_desc.u_mutex);
	list_del(&cam->list);
	mutex_unlock(cam_desc.u_mutex);

	camera_app_remove(cam, true);

	camera_ref_down();
	file->private_data = NULL;
	return 0;
}

static const struct file_operations camera_fileops = {
	.owner = THIS_MODULE,
	.open = camera_open,
	.unlocked_ioctl = camera_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = camera_ioctl,
#endif
	.release = camera_release,
};

static int camera_remove(struct platform_device *dev)
{
	struct camera_info *cam;
	struct camera_device *cdev;

	dev_dbg(cam_desc.dev, "%s\n", __func__);

	camera_ref_lock();

	atomic_xchg(&cam_desc.in_use, 0);
	misc_deregister(&cam_desc.miscdev);

	list_for_each_entry(cam, cam_desc.app_list, list) {
		mutex_lock(cam_desc.u_mutex);
		list_del(&cam->list);
		mutex_unlock(cam_desc.u_mutex);
		camera_app_remove(cam, false);
	}

	list_for_each_entry(cdev, cam_desc.dev_list, list) {
		mutex_lock(cam_desc.d_mutex);
		list_del(&cdev->list);
		mutex_unlock(cam_desc.d_mutex);
		camera_remove_device(cdev, true);
	}

#ifdef TEGRA_12X_OR_HIGHER_CONFIG
	tegra_isp_unregister_mfi_cb();
#endif
	camera_debugfs_remove();

	kfree(cam_desc.layout);
	cam_desc.layout = NULL;
	cam_desc.size_layout = 0;
	if (cam_desc.pdata->freeable)
		kfree(cam_desc.pdata);
	cam_desc.pdata = NULL;
	return 0;
}

static int camera_probe(struct platform_device *dev)
{
	struct camera_platform_data *pd;

	dev_dbg(&dev->dev, "%s\n", __func__);
	if (atomic_xchg(&cam_desc.in_use, 1)) {
		dev_err(&dev->dev, "%s OCCUPIED!\n", __func__);
		return -EBUSY;
	}

	camera_ref_lock();
	cam_desc.dev = &dev->dev;
	if (dev->dev.of_node) {
		pd = of_camera_create_pdata(dev);
		if (IS_ERR(pd))
			return (int)pd;
		cam_desc.pdata = pd;
	} else if (dev->dev.platform_data) {
		cam_desc.pdata = dev->dev.platform_data;
	} else {
		cam_desc.pdata = &camera_dflt_pdata;
		dev_dbg(cam_desc.dev, "%s No platform data.  Using defaults.\n",
			__func__);
	}
	dev_dbg(&dev->dev, "%x\n", cam_desc.pdata->cfg);

	strcpy(cam_desc.dname, "camera.pcl");
	dev_set_drvdata(&dev->dev, &cam_desc);

#ifdef TEGRA_12X_OR_HIGHER_CONFIG
	camera_dev_sync_init();
	tegra_isp_register_mfi_cb(camera_dev_sync_cb, NULL);
#endif
	of_camera_init(&cam_desc);

	cam_desc.miscdev.name = cam_desc.dname;
	cam_desc.miscdev.fops = &camera_fileops;
	cam_desc.miscdev.minor = MISC_DYNAMIC_MINOR;
	if (misc_register(&cam_desc.miscdev)) {
		dev_err(cam_desc.dev, "%s unable to register misc device %s\n",
			__func__, cam_desc.dname);
		return -ENODEV;
	}

	camera_debugfs_init(&cam_desc);
	camera_ref_init();
	return 0;
}

static void camera_shutdown(struct platform_device *dev)
{
	dev_dbg(&dev->dev, "%s ...\n", __func__);

	camera_ref_lock();
	atomic_xchg(&cam_desc.in_use, 0);
	dev_info(&dev->dev, "%s locked.\n", __func__);
}

static const struct platform_device_id camera_id[] = {
	{ "pcl-generic", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, camera_id);

static struct platform_driver camera_driver = {
	.driver = {
		.name = "pcl-generic",
		.owner = THIS_MODULE,
	},
	.id_table = camera_id,
	.probe = camera_probe,
	.remove = camera_remove,
	.shutdown = camera_shutdown,
};

module_platform_driver(camera_driver);

MODULE_DESCRIPTION("Generic Camera Device Driver");
MODULE_AUTHOR("Charlie Huang <chahuang@nvidia.com>");
MODULE_LICENSE("GPL v2");
