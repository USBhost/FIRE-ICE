/* CwMcuSensor.c - driver file for HTC SensorHUB
 *
 * Copyright (C) 2014 HTC Ltd.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/CwMcuSensor.h>
#include <linux/gpio.h>
#include <linux/wakelock.h>
#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif
#include <linux/mutex.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>

#include <linux/regulator/consumer.h>

#include <linux/firmware.h>

#include <linux/notifier.h>

#include <linux/sensor_hub.h>

#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/irq_work.h>

/*#include <mach/gpiomux.h>*/
#define D(x...) pr_debug("[S_HUB][CW_MCU] " x)
#define I(x...) pr_info("[S_HUB][CW_MCU] " x)
#define E(x...) pr_err("[S_HUB][CW_MCU] " x)

#define RETRY_TIMES 20
#define LATCH_TIMES  1
#define CHECK_FW_VER_TIMES  3
#define UPDATE_FIRMWARE_RETRY_TIMES 5
#define FW_ERASE_MIN 9000
#define FW_ERASE_MAX 12000
#define LATCH_ERROR_NO (-110)
#define ACTIVE_RETRY_TIMES 10
#define DPS_MAX			(1 << (16 - 1))
/* ========================================================================= */

#define TOUCH_LOG_DELAY		5000
#define CWMCU_BATCH_TIMEOUT_MIN 200
#define MS_TO_PERIOD (1000 * 99 / 100)

/* ========================================================================= */
#define rel_significant_motion REL_WHEEL

#define ACC_CALIBRATOR_LEN 3
#define ACC_CALIBRATOR_RL_LEN 12
#define MAG_CALIBRATOR_LEN 26
#define GYRO_CALIBRATOR_LEN 3
#define LIGHT_CALIBRATOR_LEN 4
#define PRESSURE_CALIBRATOR_LEN 4

#define REPORT_EVENT_COMMON_LEN 3

#define FW_VER_INFO_LEN 31
#define FW_VER_HEADER_LEN 7
#define FW_VER_COUNT 6
#define FW_RESPONSE_CODE 0x79
#define FW_I2C_LEN_LIMIT 60

#define REACTIVATE_PERIOD (10*HZ)
#define RESET_PERIOD (30*HZ)
#define SYNC_ACK_MAGIC  0x66
#define EXHAUSTED_MAGIC 0x77

#define CALIBRATION_DATA_PATH "/calibration_data"
#define G_SENSOR_FLASH_DATA "gs_flash"
#define GYRO_SENSOR_FLASH_DATA "gyro_flash"
#define LIGHT_SENSOR_FLASH_DATA "als_flash"
#define BARO_SENSOR_FLASH_DATA "bs_flash"

#ifdef CONFIG_CWSTM32_DEBUG  /* Remove this from defconfig when release */

static int DEBUG_FLAG_GSENSOR;
module_param(DEBUG_FLAG_GSENSOR, int, 0600);

#else

#define DEBUG_FLAG_GSENSOR 0

#endif


static int DEBUG_DISABLE;
module_param(DEBUG_DISABLE, int, 0660);
MODULE_PARM_DESC(DEBUG_DISABLE, "disable " CWMCU_I2C_NAME " driver") ;

struct cwmcu_data {
	struct i2c_client *client;
	atomic_t delay;

	/* mutex_lock protect:
	 * mcu_data->suspended,
	 * cw_set_pseudo_irq(indio_dev, state);
	 * iio_push_to_buffers(mcu_data->indio_dev, event);
	 */
	struct mutex mutex_lock;

	/* group_i2c_lock protect:
	 * set_calibrator_en(),
	 * set_k_value(),
	 * get_light_polling(),
	 * CWMCU_i2c_multi_write()
	 */
	struct mutex group_i2c_lock;

	/* activated_i2c_lock protect:
	 * CWMCU_i2c_write(),
	 * CWMCU_i2c_read(),
	 * reset_hub(),
	 * mcu_data->i2c_total_retry,
	 * mcu_data->i2c_latch_retry,
	 * mcu_data->i2c_jiffies
	 */
	struct mutex activated_i2c_lock;

	/* power_mode_lock protect:
	 * mcu_data->power_on_counter
	 */
	struct mutex power_mode_lock;

	struct iio_trigger  *trig;
	atomic_t pseudo_irq_enable;
	struct mutex lock;

	struct timeval now;
	struct class *sensor_class;
	struct device *sensor_dev;
	u8	acceleration_axes;
	u8	magnetic_axes;
	u8	gyro_axes;

	u64	enabled_list; /* Bit mask for sensor enable status */
	u64	batched_list; /* Bit mask for FIFO usage, 32MSB is wake up */

	/* report time */
	s64	sensors_time[num_sensors];
	s64	time_diff[num_sensors];
	s32	report_period[num_sensors]; /* Microseconds * 0.99 */
	u64	update_list;
	u64	pending_flush;
	s64	batch_timeout[num_sensors];
	int	IRQ;
	struct delayed_work	work;
	struct work_struct	one_shot_work;
	/* Remember to add flag in cwmcu_resume() when add new flag */
	bool w_activated_i2c;
	bool w_re_init;
	bool w_facedown_set;
	bool w_flush_fifo;
	bool w_clear_fifo;
	bool w_clear_fifo_running;
	bool w_report_meta;

	bool suspended;
	bool probe_success;
	bool is_block_i2c;

	u32 gpio_wake_mcu;
	u32 gpio_reset;
	u32 gpio_chip_mode;
	u32 gpio_mcu_irq;
	s32 gs_chip_layout;
	u32 gs_kvalue;
	s16 gs_kvalue_R1;
	s16 gs_kvalue_R2;
	s16 gs_kvalue_R3;
	s16 gs_kvalue_L1;
	s16 gs_kvalue_L2;
	s16 gs_kvalue_L3;
	u32 gy_kvalue;
	u32 als_kvalue;
	u32 bs_kvalue;
	u8  bs_kheader;
	u8  gs_calibrated;
	u8  ls_calibrated;
	u8  bs_calibrated;
	u8  gy_calibrated;

	s32 i2c_total_retry;
	s32 i2c_latch_retry;
	unsigned long i2c_jiffies;
	unsigned long reset_jiffies;

	int disable_access_count;

	s32 iio_data[6];
	struct iio_dev *indio_dev;
	struct irq_work iio_irq_work;

	/* power status */
	int power_on_counter;

	struct input_dev *input;
	u16 light_last_data[REPORT_EVENT_COMMON_LEN];
	u64 time_base;
	u64 wake_fifo_time_base;
	u64 step_counter_base;

	struct workqueue_struct *mcu_wq;
	struct wake_lock significant_wake_lock;
	struct wake_lock report_wake_lock;

	int fw_update_status;
	u16 erase_fw_wait;
};

BLOCKING_NOTIFIER_HEAD(double_tap_notifier_list);

int register_notifier_by_facedown(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&double_tap_notifier_list, nb);
}
EXPORT_SYMBOL(register_notifier_by_facedown);

int unregister_notifier_by_facedown(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&double_tap_notifier_list,
						  nb);
}
EXPORT_SYMBOL(unregister_notifier_by_facedown);

static int CWMCU_i2c_read(struct cwmcu_data *mcu_data,
			u8 reg_addr, void *data, u8 len);
static int CWMCU_i2c_read_power(struct cwmcu_data *mcu_data,
			 u8 reg_addr, void *data, u8 len);
static int CWMCU_i2c_write(struct cwmcu_data *mcu_data,
			u8 reg_addr, const void *data, u8 len);
static int CWMCU_i2c_write_power(struct cwmcu_data *mcu_data,
			u8 reg_addr, const void *data, u8 len);
static int firmware_odr(struct cwmcu_data *mcu_data, int sensors_id,
			int delay_ms);
static void cwmcu_batch_read(struct cwmcu_data *mcu_data);

static void gpio_make_falling_edge(int gpio)
{
	if (!gpio_get_value(gpio))
		gpio_set_value(gpio, 1);
	gpio_set_value(gpio, 0);
}

static void cwmcu_powermode_switch(struct cwmcu_data *mcu_data, int onoff)
{
	mutex_lock(&mcu_data->power_mode_lock);
	if (onoff) {
		if (mcu_data->power_on_counter == 0) {
			gpio_make_falling_edge(mcu_data->gpio_wake_mcu);
			udelay(10);
			gpio_set_value(mcu_data->gpio_wake_mcu, 1);
			udelay(10);
			gpio_set_value(mcu_data->gpio_wake_mcu, 0);
			D("%s: 11 onoff = %d\n", __func__, onoff);
			usleep_range(500, 600);
		}
		mcu_data->power_on_counter++;
	} else {
		mcu_data->power_on_counter--;
		if (mcu_data->power_on_counter <= 0) {
			mcu_data->power_on_counter = 0;
			gpio_set_value(mcu_data->gpio_wake_mcu, 1);
			D("%s: 22 onoff = %d\n", __func__, onoff);
		}
	}
	mutex_unlock(&mcu_data->power_mode_lock);
	D("%s: onoff = %d, power_counter = %d\n", __func__, onoff,
	  mcu_data->power_on_counter);
}

static int cw_send_event(struct cwmcu_data *mcu_data, u8 id, u16 *data,
			 s64 timestamp)
{
	u8 event[21];/* Sensor HAL uses fixed 21 bytes */

	event[0] = id;
	memcpy(&event[1], data, sizeof(u16)*REPORT_EVENT_COMMON_LEN);
	memset(&event[7], 0, sizeof(u16)*REPORT_EVENT_COMMON_LEN);
	memcpy(&event[13], &timestamp, sizeof(s64));

	D("%s: active_scan_mask = 0x%p, masklength = %u, data(x, y, z) ="
	  "(%d, %d, %d)\n",
	  __func__, mcu_data->indio_dev->active_scan_mask,
	  mcu_data->indio_dev->masklength,
	  *(s16 *)&event[1], *(s16 *)&event[3], *(s16 *)&event[5]);

	if (mcu_data->indio_dev->active_scan_mask &&
	    (!bitmap_empty(mcu_data->indio_dev->active_scan_mask,
			   mcu_data->indio_dev->masklength))) {
		mutex_lock(&mcu_data->mutex_lock);
		if (!mcu_data->w_clear_fifo_running)
			iio_push_to_buffers(mcu_data->indio_dev, event);
		else {
			D(
			  "%s: Drop data(0, 1, 2) = (0x%x, 0x%x, 0x%x)\n",
			  __func__, data[0], data[1], data[2]);
		}
		mutex_unlock(&mcu_data->mutex_lock);
		return 0;
	} else if (mcu_data->indio_dev->active_scan_mask == NULL)
		D("%s: active_scan_mask = NULL, event might be missing\n",
		  __func__);

	return -EIO;
}

static int cw_send_event_special(struct cwmcu_data *mcu_data, u8 id, u16 *data,
				 u16 *bias, s64 timestamp)
{
	u8 event[1+(2*sizeof(u16)*REPORT_EVENT_COMMON_LEN)+sizeof(timestamp)];

	event[0] = id;
	memcpy(&event[1], data, sizeof(u16)*REPORT_EVENT_COMMON_LEN);
	memcpy(&event[1+sizeof(u16)*REPORT_EVENT_COMMON_LEN], bias,
	       sizeof(u16)*REPORT_EVENT_COMMON_LEN);
	memcpy(&event[1+(2*sizeof(u16)*REPORT_EVENT_COMMON_LEN)], &timestamp,
	       sizeof(timestamp));

	if (mcu_data->indio_dev->active_scan_mask &&
	    (!bitmap_empty(mcu_data->indio_dev->active_scan_mask,
			   mcu_data->indio_dev->masklength))) {
		mutex_lock(&mcu_data->mutex_lock);
		if (!mcu_data->w_clear_fifo_running)
			iio_push_to_buffers(mcu_data->indio_dev, event);
		else {
			D(
			  "%s: Drop data(0, 1, 2) = (0x%x, 0x%x, 0x%x)\n",
			  __func__, data[0], data[1], data[2]);
		}
		mutex_unlock(&mcu_data->mutex_lock);
		return 0;
	} else if (mcu_data->indio_dev->active_scan_mask == NULL)
		D("%s: active_scan_mask = NULL, event might be missing\n",
		  __func__);

	return -EIO;
}

static int cwmcu_get_calibrator_status(struct cwmcu_data *mcu_data,
				       u8 sensor_id, u8 *data)
{
	int error_msg = 0;

	if (sensor_id == CW_ACCELERATION)
		error_msg = CWMCU_i2c_read_power(mcu_data,
				CW_I2C_REG_SENSORS_CALIBRATOR_STATUS_ACC,
				data, 1);
	else if (sensor_id == CW_MAGNETIC)
		error_msg = CWMCU_i2c_read_power(mcu_data,
				CW_I2C_REG_SENSORS_CALIBRATOR_STATUS_MAG,
				data, 1);
	else if (sensor_id == CW_GYRO)
		error_msg = CWMCU_i2c_read_power(mcu_data,
				CW_I2C_REG_SENSORS_CALIBRATOR_STATUS_GYRO,
				data, 1);

	return error_msg;
}

static int cwmcu_get_calibrator(struct cwmcu_data *mcu_data, u8 sensor_id,
				s8 *data, u8 len)
{
	int error_msg = 0;

	if ((sensor_id == CW_ACCELERATION) && (len == ACC_CALIBRATOR_LEN))
		error_msg = CWMCU_i2c_read_power(mcu_data,
				CW_I2C_REG_SENSORS_CALIBRATOR_GET_DATA_ACC,
				data, len);
	else if ((sensor_id == CW_MAGNETIC) && (len == MAG_CALIBRATOR_LEN))
		error_msg = CWMCU_i2c_read_power(mcu_data,
				CW_I2C_REG_SENSORS_CALIBRATOR_GET_DATA_MAG,
				data, len);
	else if ((sensor_id == CW_GYRO) && (len == GYRO_CALIBRATOR_LEN))
		error_msg = CWMCU_i2c_read_power(mcu_data,
				CW_I2C_REG_SENSORS_CALIBRATOR_GET_DATA_GYRO,
				data, len);
	else if ((sensor_id == CW_LIGHT) && (len == LIGHT_CALIBRATOR_LEN))
		error_msg = CWMCU_i2c_read_power(mcu_data,
				CW_I2C_REG_SENSORS_CALIBRATOR_GET_DATA_LIGHT,
				data, len);
	else if ((sensor_id == CW_PRESSURE) && (len == PRESSURE_CALIBRATOR_LEN))
		error_msg = CWMCU_i2c_read_power(mcu_data,
				CW_I2C_REG_SENSORS_CALIBRATOR_GET_DATA_PRESSURE,
				data, len);
	else
		E("%s: invalid arguments, sensor_id = %u, len = %u\n",
		  __func__, sensor_id, len);

	D("sensors_id = %u, calibrator data = (%d, %d, %d)\n", sensor_id,
	  data[0], data[1], data[2]);
	return error_msg;
}

static ssize_t set_calibrator_en(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	u8 data;
	u8 data2;
	unsigned long sensors_id;
	int error;

	error = kstrtoul(buf, 10, &sensors_id);
	if (error) {
		E("%s: kstrtoul fails, error = %d\n", __func__, error);
		return error;
	}

	/* sensor_id at least should between 0 ~ 31 */
	data = (u8)sensors_id;
	D("%s: data(sensors_id) = %u\n", __func__, data);

	cwmcu_powermode_switch(mcu_data, 1);

	mutex_lock(&mcu_data->group_i2c_lock);

	switch (data) {
	case 1:
		error = CWMCU_i2c_read(mcu_data, G_SENSORS_STATUS, &data2, 1);
		if (error < 0)
			goto i2c_fail;
		data = data2 | 16;
		error = CWMCU_i2c_write(mcu_data, G_SENSORS_STATUS, &data, 1);
		if (error < 0)
			goto i2c_fail;
		break;
	case 2:
		error = CWMCU_i2c_read(mcu_data, ECOMPASS_SENSORS_STATUS,
				       &data2, 1);
		if (error < 0)
			goto i2c_fail;
		data = data2 | 16;
		error = CWMCU_i2c_write(mcu_data, ECOMPASS_SENSORS_STATUS,
					&data, 1);
		if (error < 0)
			goto i2c_fail;
		break;
	case 4:
		error = CWMCU_i2c_read(mcu_data, GYRO_SENSORS_STATUS,
				       &data2, 1);
		if (error < 0)
			goto i2c_fail;
		data = data2 | 16;
		error = CWMCU_i2c_write(mcu_data, GYRO_SENSORS_STATUS,
					&data, 1);
		if (error < 0)
			goto i2c_fail;
		break;
	case 7:
		error = CWMCU_i2c_read(mcu_data, LIGHT_SENSORS_STATUS,
				       &data2, 1);
		if (error < 0)
			goto i2c_fail;
		data = data2 | 16;
		error = CWMCU_i2c_write(mcu_data, LIGHT_SENSORS_STATUS,
					&data, 1);
		if (error < 0)
			goto i2c_fail;
		break;
	case 9:
		data = 2; /* X- R calibration */
		error = CWMCU_i2c_write(mcu_data,
				CW_I2C_REG_SENSORS_CALIBRATOR_TARGET_ACC,
				&data, 1);
		error = CWMCU_i2c_read(mcu_data, G_SENSORS_STATUS, &data2, 1);
		if (error < 0)
			goto i2c_fail;
		data = data2 | 16;
		error = CWMCU_i2c_write(mcu_data, G_SENSORS_STATUS, &data, 1);
		if (error < 0)
			goto i2c_fail;
		break;
	case 10:
		data = 1; /* X+ L calibration */
		error = CWMCU_i2c_write(mcu_data,
				CW_I2C_REG_SENSORS_CALIBRATOR_TARGET_ACC,
				&data, 1);
		error = CWMCU_i2c_read(mcu_data, G_SENSORS_STATUS, &data2, 1);
		if (error < 0)
			goto i2c_fail;
		data = data2 | 16;
		error = CWMCU_i2c_write(mcu_data, G_SENSORS_STATUS, &data, 1);
		if (error < 0)
			goto i2c_fail;
		break;
	case 11:
		data = 0; /* Z+ */
		error = CWMCU_i2c_write(mcu_data,
				CW_I2C_REG_SENSORS_CALIBRATOR_TARGET_ACC,
				&data, 1);
		if (error < 0)
			goto i2c_fail;
		break;
	case 12:
		mcu_data->step_counter_base = 0;
		D("%s: Reset step counter\n", __func__);
		break;
	default:
		mutex_unlock(&mcu_data->group_i2c_lock);
		cwmcu_powermode_switch(mcu_data, 0);
		E("%s: Improper sensor_id = %u\n", __func__, data);
		return -EINVAL;
	}

	error = count;

i2c_fail:
	mutex_unlock(&mcu_data->group_i2c_lock);

	cwmcu_powermode_switch(mcu_data, 0);

	D("%s--: data2 = 0x%x, rc = %d\n", __func__, data2, error);
	return error;
}

static void print_hex_data(char *buf, u32 index, u8 *data, size_t len)
{
	int i;
	int rc;
	char *buf_start;
	size_t buf_remaining =
		3*EXCEPTION_BLOCK_LEN; /* 3 characters per data */

	buf_start = buf;

	for (i = 0; i < len; i++) {
		rc = scnprintf(buf, buf_remaining, "%02x%c", data[i],
				(i == len - 1) ? '\0' : ' ');
		buf += rc;
		buf_remaining -= rc;
	}

	printk(KERN_ERR "[S_HUB][CW_MCU] Exception Buffer[%d] = %.*s\n",
			index * EXCEPTION_BLOCK_LEN,
			(int)(buf - buf_start),
			buf_start);
}

static ssize_t sprint_data(char *buf, s8 *data, ssize_t len)
{
	int i;
	int rc;
	size_t buf_remaining = PAGE_SIZE;

	for (i = 0; i < len; i++) {
		rc = scnprintf(buf, buf_remaining, "%d%c", data[i],
				(i == len - 1) ? '\n' : ' ');
		buf += rc;
		buf_remaining -= rc;
	}
	return PAGE_SIZE - buf_remaining;
}

static ssize_t show_calibrator_status_acc(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	u8 data[6] = {0};

	if (cwmcu_get_calibrator_status(mcu_data, CW_ACCELERATION, data) >= 0)
		return scnprintf(buf, PAGE_SIZE, "0x%x\n", data[0]);

	return scnprintf(buf, PAGE_SIZE, "0x1\n");
}

static ssize_t show_calibrator_status_mag(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	u8 data[6] = {0};

	if (cwmcu_get_calibrator_status(mcu_data, CW_MAGNETIC, data) >= 0)
		return scnprintf(buf, PAGE_SIZE, "0x%x\n", data[0]);

	return scnprintf(buf, PAGE_SIZE, "0x1\n");
}

static ssize_t show_calibrator_status_gyro(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	u8 data[6] = {0};

	if (cwmcu_get_calibrator_status(mcu_data, CW_GYRO, data) >= 0)
		return scnprintf(buf, PAGE_SIZE, "0x%x\n", data[0]);

	return scnprintf(buf, PAGE_SIZE, "0x1\n");
}

static ssize_t set_k_value(struct cwmcu_data *mcu_data, const char *buf,
			   size_t count, u8 reg_addr, u8 len)
{
	int i;
	long data_temp[len];
	char *str_buf;
	char *running;
	int error;

	D(
	  "%s: count = %lu, strlen(buf) = %lu, PAGE_SIZE = %lu,"
	  " reg_addr = 0x%x\n",
	  __func__, count, strlen(buf), PAGE_SIZE, reg_addr);

	str_buf = kstrndup(buf, count, GFP_KERNEL);
	if (str_buf == NULL) {
		E("%s: cannot allocate buffer\n", __func__);
		return -ENOMEM;
	}
	running = str_buf;

	for (i = 0; i < len; i++) {
		int error;
		char *token;

		token = strsep(&running, " ");

		if (token == NULL) {
			D("%s: i = %d\n", __func__, i);
			break;
		} else {
			if (reg_addr ==
			    CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PRESSURE)
				error = kstrtol(token, 16, &data_temp[i]);
			else
				error = kstrtol(token, 10, &data_temp[i]);
			if (error) {
				E("%s: kstrtol fails, error = %d, i = %d\n",
				  __func__, error, i);
				kfree(str_buf);
				return error;
			}
		}
	}
	kfree(str_buf);

	D("Set calibration by attr (%ld, %ld, %ld), len = %u, reg_addr = 0x%x\n"
	  , data_temp[0], data_temp[1], data_temp[2], len, reg_addr);

	cwmcu_powermode_switch(mcu_data, 1);

	mutex_lock(&mcu_data->group_i2c_lock);
	for (i = 0; i < len; i++) {
		u8 data = (u8)(data_temp[i]);
		/* Firmware can't write multi bytes */
		error = CWMCU_i2c_write(mcu_data, reg_addr, &data, 1);
		if (error < 0) {
			mutex_unlock(&mcu_data->group_i2c_lock);
			cwmcu_powermode_switch(mcu_data, 0);
			E("%s: error = %d, i = %d\n", __func__, error, i);
			return -EIO;
		}
	}
	mutex_unlock(&mcu_data->group_i2c_lock);

	cwmcu_powermode_switch(mcu_data, 0);

	return count;
}

static ssize_t set_k_value_acc_f(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);

	return set_k_value(mcu_data, buf, count,
			   CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_ACC,
			   ACC_CALIBRATOR_LEN);
}


static ssize_t set_k_value_mag_f(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);

	return set_k_value(mcu_data, buf, count,
			   CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_MAG,
			   MAG_CALIBRATOR_LEN);
}

static ssize_t set_k_value_gyro_f(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);

	return set_k_value(mcu_data, buf, count,
			   CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_GYRO,
			   GYRO_CALIBRATOR_LEN);
}

static ssize_t set_k_value_barometer_f(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);

	return set_k_value(mcu_data, buf, count,
			   CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PRESSURE,
			   PRESSURE_CALIBRATOR_LEN);
}

static ssize_t led_enable(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	int error;
	u8 data;
	long data_temp = 0;

	error = kstrtol(buf, 10, &data_temp);
	if (error) {
		E("%s: kstrtol fails, error = %d\n", __func__, error);
		return error;
	}

	data = data_temp ? 2 : 4;

	I("LED %s\n", (data == 2) ? "ENABLE" : "DISABLE");

	error = CWMCU_i2c_write_power(mcu_data, 0xD0, &data, 1);
	if (error < 0) {
		E("%s: error = %d\n", __func__, error);
		return -EIO;
	}

	return count;
}

static ssize_t get_k_value(struct cwmcu_data *mcu_data, int type, char *buf,
			   char *data, unsigned len)
{
	if (cwmcu_get_calibrator(mcu_data, type, data, len) < 0)
		memset(data, 0, len);

	return sprint_data(buf, data, len);
}

static ssize_t get_k_value_acc_f(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	u8 data[ACC_CALIBRATOR_LEN] = {0};

	return get_k_value(mcu_data, CW_ACCELERATION, buf, data, sizeof(data));
}

static ssize_t get_k_value_acc_rl_f(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	u8 data[ACC_CALIBRATOR_RL_LEN] = {0};

	if (CWMCU_i2c_read_power(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_RESULT_RL_ACC
			   , data, sizeof(data)) >= 0) {

		if (DEBUG_FLAG_GSENSOR == 1) {
			int i;

			for (i = 0; i < sizeof(data); i++)
				D("data[%d]: %u\n", i, data[i]);
		}

		mcu_data->gs_kvalue_L1 = ((s8)data[1] << 8) | data[0];
		mcu_data->gs_kvalue_L2 = ((s8)data[3] << 8) | data[2];
		mcu_data->gs_kvalue_L3 = ((s8)data[5] << 8) | data[4];
		mcu_data->gs_kvalue_R1 = ((s8)data[7] << 8) | data[6];
		mcu_data->gs_kvalue_R2 = ((s8)data[9] << 8) | data[8];
		mcu_data->gs_kvalue_R3 = ((s8)data[11] << 8) | data[10];
	}

	return sprint_data(buf, data, sizeof(data));
}

static ssize_t ap_get_k_value_acc_rl_f(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d %d %d %d %d %d\n",
			 (s16)mcu_data->gs_kvalue_L1,
			 (s16)mcu_data->gs_kvalue_L2,
			 (s16)mcu_data->gs_kvalue_L3,
			 (s16)mcu_data->gs_kvalue_R1,
			 (s16)mcu_data->gs_kvalue_R2,
			 (s16)mcu_data->gs_kvalue_R3);
}

static ssize_t get_k_value_mag_f(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	u8 data[MAG_CALIBRATOR_LEN] = {0};

	return get_k_value(mcu_data, CW_MAGNETIC, buf, data, sizeof(data));
}

static ssize_t get_k_value_gyro_f(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	u8 data[GYRO_CALIBRATOR_LEN] = {0};

	return get_k_value(mcu_data, CW_GYRO, buf, data, sizeof(data));
}

static ssize_t get_k_value_light_f(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	u8 data[LIGHT_CALIBRATOR_LEN] = {0};

	if (cwmcu_get_calibrator(mcu_data, CW_LIGHT, data, sizeof(data)) < 0) {
		E("%s: Get LIGHT Calibrator fails\n", __func__);
		return -EIO;
	}
	return scnprintf(buf, PAGE_SIZE, "%x %x %x %x\n", data[0], data[1],
			 data[2], data[3]);
}

static ssize_t get_k_value_barometer_f(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	u8 data[PRESSURE_CALIBRATOR_LEN] = {0};

	return get_k_value(mcu_data, CW_PRESSURE, buf, data, sizeof(data));
}

static int CWMCU_i2c_read_power(struct cwmcu_data *mcu_data,
			 u8 reg_addr, void *data, u8 len)
{
	int ret;

	cwmcu_powermode_switch(mcu_data, 1);
	ret = CWMCU_i2c_read(mcu_data, reg_addr, data, len);
	cwmcu_powermode_switch(mcu_data, 0);
	return ret;
}

static int CWMCU_i2c_write_power(struct cwmcu_data *mcu_data,
				 u8 reg_addr, const void *data, u8 len)
{
	int ret;

	cwmcu_powermode_switch(mcu_data, 1);
	ret = CWMCU_i2c_write(mcu_data, reg_addr, data, len);
	cwmcu_powermode_switch(mcu_data, 0);
	return ret;
}

static ssize_t get_light_kadc(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	u8 data[4] = {0};
	u16 light_gadc;
	u16 light_kadc;

	CWMCU_i2c_read_power(mcu_data, LIGHT_SENSORS_CALIBRATION_DATA, data,
			     sizeof(data));

	light_gadc = (data[1] << 8) | data[0];
	light_kadc = (data[3] << 8) | data[2];
	return scnprintf(buf, PAGE_SIZE, "gadc = 0x%x, kadc = 0x%x", light_gadc,
			 light_kadc);
}

static ssize_t get_firmware_version(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	u8 firmware_version[FW_VER_COUNT] = {0};

	CWMCU_i2c_read_power(mcu_data, FIRMWARE_VERSION, firmware_version,
			     sizeof(firmware_version));

	return scnprintf(buf, PAGE_SIZE,
			 "Firmware Architecture version %u, "
			 "Sense version %u, Cywee lib version %u,"
			 " Water number %u"
			 ", Active Engine %u, Project Mapping %u\n",
			 firmware_version[0], firmware_version[1],
			 firmware_version[2], firmware_version[3],
			 firmware_version[4], firmware_version[5]);
}

static ssize_t get_hall_sensor(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	u8 hall_sensor = 0;

	CWMCU_i2c_read_power(mcu_data, CWSTM32_READ_Hall_Sensor,
			     &hall_sensor, 1);

	return scnprintf(buf, PAGE_SIZE,
			 "Hall_1(S, N) = (%u, %u), Hall_2(S, N)"
			 " = (%u, %u), Hall_3(S, N) = (%u, %u)\n",
			 !!(hall_sensor & 0x1), !!(hall_sensor & 0x2),
			 !!(hall_sensor & 0x4), !!(hall_sensor & 0x8),
			 !!(hall_sensor & 0x10), !!(hall_sensor & 0x20));
}

static ssize_t get_barometer(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	u8 data[6] = {0};

	CWMCU_i2c_read_power(mcu_data, CWSTM32_READ_Pressure, data,
			     sizeof(data));

	return scnprintf(buf, PAGE_SIZE, "%x %x %x %x\n", data[0], data[1],
					 data[2], data[3]);
}

static ssize_t get_light_polling(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	u8 data[REPORT_EVENT_COMMON_LEN] = {0};
	u8 data_polling_enable;
	u16 light_adc;
	int rc;

	data_polling_enable = CW_MCU_BIT_LIGHT_POLLING;

	cwmcu_powermode_switch(mcu_data, 1);

	mutex_lock(&mcu_data->group_i2c_lock);
	rc = CWMCU_i2c_write(mcu_data, LIGHT_SENSORS_STATUS,
			&data_polling_enable, 1);
	if (rc < 0) {
		mutex_unlock(&mcu_data->group_i2c_lock);
		cwmcu_powermode_switch(mcu_data, 0);
		E("%s: write fail, rc = %d\n", __func__, rc);
		return rc;
	}
	CWMCU_i2c_read(mcu_data, CWSTM32_READ_Light, data, sizeof(data));
	if (rc < 0) {
		mutex_unlock(&mcu_data->group_i2c_lock);
		cwmcu_powermode_switch(mcu_data, 0);
		E("%s: read fail, rc = %d\n", __func__, rc);
		return rc;
	}
	mutex_unlock(&mcu_data->group_i2c_lock);

	cwmcu_powermode_switch(mcu_data, 0);

	light_adc = (data[2] << 8) | data[1];

	I("poll light[%x]=%u\n", light_adc, data[0]);

	return scnprintf(buf, PAGE_SIZE, "ADC[0x%04X] => level %u\n", light_adc,
					 data[0]);
}


static ssize_t read_mcu_data(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	int i;
	u8 reg_addr;
	u8 len;
	long data_temp[2] = {0};
	u8 mcu_rdata[128] = {0};
	char *str_buf;
	char *running;

	str_buf = kstrndup(buf, count, GFP_KERNEL);
	if (str_buf == NULL) {
		E("%s: cannot allocate buffer\n", __func__);
		return -ENOMEM;
	}
	running = str_buf;

	for (i = 0; i < ARRAY_SIZE(data_temp); i++) {
		int error;
		char *token;

		token = strsep(&running, " ");

		if (i == 0)
			error = kstrtol(token, 16, &data_temp[i]);
		else {
			if (token == NULL) {
				data_temp[i] = 1;
				D("%s: token 2 missing\n", __func__);
				break;
			} else
				error = kstrtol(token, 10, &data_temp[i]);
		}
		if (error) {
			E("%s: kstrtol fails, error = %d, i = %d\n",
			  __func__, error, i);
			kfree(str_buf);
			return error;
		}
	}
	kfree(str_buf);

	/* TESTME for changing array to variable */
	reg_addr = (u8)(data_temp[0]);
	len = (u8)(data_temp[1]);

	if (len < sizeof(mcu_rdata)) {
		CWMCU_i2c_read_power(mcu_data, reg_addr, mcu_rdata, len);

		for (i = 0; i < len; i++)
			D("read mcu reg_addr = 0x%x, reg[%u] = 0x%x\n",
				reg_addr, (reg_addr + i), mcu_rdata[i]);
	} else
		E("%s: len = %u, out of range\n", __func__, len);

	return count;
}

static inline bool retry_exhausted(struct cwmcu_data *mcu_data)
{
	return ((mcu_data->i2c_total_retry > RETRY_TIMES) ||
		(mcu_data->i2c_latch_retry > LATCH_TIMES));
}

static inline void retry_reset(struct cwmcu_data *mcu_data)
{
	mcu_data->i2c_total_retry = 0;
	mcu_data->i2c_latch_retry = 0;
}

static int CWMCU_i2c_write(struct cwmcu_data *mcu_data,
			  u8 reg_addr, const void *data, u8 len)
{
	s32 write_res;
	int i;
	const u8 *u8_data = data;

	if (DEBUG_DISABLE) {
		mcu_data->disable_access_count++;
		if ((mcu_data->disable_access_count % 100) == 0)
			I("%s: DEBUG_DISABLE = %d\n", __func__, DEBUG_DISABLE);
		return len;
	}

	if (mcu_data->is_block_i2c) {
		if (time_after(jiffies,
			       mcu_data->reset_jiffies + RESET_PERIOD))
			mcu_data->is_block_i2c = 0;
		return len;
	}

	mutex_lock(&mcu_data->mutex_lock);
	if (mcu_data->suspended) {
		mutex_unlock(&mcu_data->mutex_lock);
		return len;
	}
	mutex_unlock(&mcu_data->mutex_lock);

	mutex_lock(&mcu_data->activated_i2c_lock);
	if (retry_exhausted(mcu_data)) {
		mutex_unlock(&mcu_data->activated_i2c_lock);
		D("%s: mcu_data->i2c_total_retry = %d, i2c_latch_retry = %d\n",
		  __func__,
		  mcu_data->i2c_total_retry, mcu_data->i2c_latch_retry);
		/* Try to recover HUB in low CPU utilization */
		mcu_data->w_activated_i2c = true;
		queue_work(mcu_data->mcu_wq, &mcu_data->one_shot_work);
		return -EIO;
	}

	for (i = 0; i < len; i++) {
		while (!retry_exhausted(mcu_data)) {
			write_res = i2c_smbus_write_byte_data(mcu_data->client,
						  reg_addr, u8_data[i]);
			if (write_res >= 0) {
				retry_reset(mcu_data);
				break;
			}
			gpio_make_falling_edge(mcu_data->gpio_wake_mcu);
			if (write_res == LATCH_ERROR_NO)
				mcu_data->i2c_latch_retry++;
			mcu_data->i2c_total_retry++;
			E(
			  "%s: i2c write error, write_res = %d, total_retry ="
			  " %d, latch_retry = %d, addr = 0x%x, val = 0x%x\n",
			  __func__, write_res, mcu_data->i2c_total_retry,
			  mcu_data->i2c_latch_retry, reg_addr, u8_data[i]);
		}

		if (retry_exhausted(mcu_data)) {
			mutex_unlock(&mcu_data->activated_i2c_lock);
			E("%s: mcu_data->i2c_total_retry = %d, "
			  "i2c_latch_retry = %d, EIO\n", __func__,
			  mcu_data->i2c_total_retry, mcu_data->i2c_latch_retry);
			return -EIO;
		}
	}

	mutex_unlock(&mcu_data->activated_i2c_lock);

	return 0;
}

static int CWMCU_i2c_multi_write(struct cwmcu_data *mcu_data,
			  u8 reg_addr, const void *data, u8 len)
{
	int rc, i;
	const u8 *u8_data = data;

	mutex_lock(&mcu_data->group_i2c_lock);

	for (i = 0; i < len; i++) {
		rc = CWMCU_i2c_write(mcu_data, reg_addr, &u8_data[i], 1);
		if (rc) {
			mutex_unlock(&mcu_data->group_i2c_lock);
			E("%s: CWMCU_i2c_write fails, rc = %d, i = %d\n",
			  __func__, rc, i);
			return -EIO;
		}
	}

	mutex_unlock(&mcu_data->group_i2c_lock);
	return 0;
}

static int cwmcu_set_sensor_kvalue(struct cwmcu_data *mcu_data)
{
	/* Write single Byte because firmware can't write multi bytes now */
	u8 *gs_data = (u8 *)&mcu_data->gs_kvalue; /* gs_kvalue is u32 */
	u8 *gy_data = (u8 *)&mcu_data->gy_kvalue; /* gy_kvalue is u32 */
	u8 *bs_data = (u8 *)&mcu_data->bs_kvalue; /* bs_kvalue is u32 */
	u8 firmware_version[FW_VER_COUNT] = {0};

	mcu_data->gs_calibrated = 0;
	mcu_data->gy_calibrated = 0;
	mcu_data->ls_calibrated = 0;
	mcu_data->bs_calibrated = 0;

	CWMCU_i2c_read(mcu_data, FIRMWARE_VERSION, firmware_version,
		       sizeof(firmware_version));
	I(
	  "Firmware Architecture version %u, Sense version %u,"
	  " Cywee lib version %u, Water number %u"
	  ", Active Engine %u, Project Mapping %u\n",
		firmware_version[0], firmware_version[1], firmware_version[2],
		firmware_version[3], firmware_version[4], firmware_version[5]);

	if (gs_data[3] == 0x67) {
		__be32 be32_gs_data = cpu_to_be32(mcu_data->gs_kvalue);
		gs_data = (u8 *)&be32_gs_data;

		CWMCU_i2c_write(mcu_data,
			CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_ACC,
			gs_data + 1, ACC_CALIBRATOR_LEN);
		mcu_data->gs_calibrated = 1;
		D("Set g-sensor kvalue (x, y, z) = (0x%x, 0x%x, 0x%x)\n",
			gs_data[1], gs_data[2], gs_data[3]);
	}

	if (gy_data[3] == 0x67) {
		__be32 be32_gy_data = cpu_to_be32(mcu_data->gy_kvalue);
		gy_data = (u8 *)&be32_gy_data;

		CWMCU_i2c_write(mcu_data,
			CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_GYRO,
			gy_data + 1, GYRO_CALIBRATOR_LEN);
		mcu_data->gy_calibrated = 1;
		D("Set gyro-sensor kvalue (x, y, z) = (0x%x, 0x%x, 0x%x)\n",
			gy_data[1], gy_data[2], gy_data[3]);
	}

	if ((mcu_data->als_kvalue & 0x6DA50000) == 0x6DA50000) {
		__le16 als_data[2];
		als_data[0] = cpu_to_le16(0x0a38);
		als_data[1] = cpu_to_le16(mcu_data->als_kvalue);
		CWMCU_i2c_write(mcu_data,
			CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_LIGHT,
			als_data, LIGHT_CALIBRATOR_LEN);
		mcu_data->ls_calibrated = 1;
		D("Set light-sensor kvalue = 0x%x\n", als_data[1]);
	}

	if (mcu_data->bs_kheader == 0x67) {
		__be32 be32_bs_data = cpu_to_be32(mcu_data->bs_kvalue);

		CWMCU_i2c_write(mcu_data,
			CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PRESSURE,
			&be32_bs_data, PRESSURE_CALIBRATOR_LEN);
		mcu_data->bs_calibrated = 1;
		D(
		  "Set barometer kvalue (a, b, c, d) = "
		  "(0x%x, 0x%x, 0x%x, 0x%x)\n",
		  bs_data[3], bs_data[2], bs_data[1], bs_data[0]);
	}
	I("Sensor calibration matrix is (gs %u gy %u ls %u bs %u)\n",
		mcu_data->gs_calibrated, mcu_data->gy_calibrated,
		mcu_data->ls_calibrated, mcu_data->bs_calibrated);
	return 0;
}


static int cwmcu_sensor_placement(struct cwmcu_data *mcu_data)
{
	D("Set Sensor Placement\n");
	CWMCU_i2c_write(mcu_data, GENSOR_POSITION, &mcu_data->acceleration_axes,
			1);
	CWMCU_i2c_write(mcu_data, COMPASS_POSITION, &mcu_data->magnetic_axes,
			1);
	CWMCU_i2c_write(mcu_data, GYRO_POSITION, &mcu_data->gyro_axes, 1);

	return 0;
}

static void cwmcu_i2c_write_group(struct cwmcu_data *mcu_data, u8 write_addr,
				  u32 enable_list)
{
	int i;
	__le32 buf = cpu_to_le32(enable_list);
	u8 *data = (u8 *)&buf;

	for (i = 0; i < sizeof(buf); ++i) {
		D("%s: write_addr = 0x%x, write_val = 0x%x\n",
		  __func__, write_addr + i, data[i]);
		CWMCU_i2c_write(mcu_data, write_addr + i, data + i, 1);
	}
}

static int cwmcu_restore_status(struct cwmcu_data *mcu_data)
{
	int i, rc;
	u8 data;
	u8 reg_value = 0;
	int delay_ms;

	D("Restore status\n");

	mcu_data->enabled_list |= (1LL << HTC_MAGIC_COVER);

	cwmcu_i2c_write_group(mcu_data, CWSTM32_ENABLE_REG,
			      mcu_data->enabled_list
			      | (mcu_data->enabled_list >> 32));
	cwmcu_i2c_write_group(mcu_data, CW_BATCH_ENABLE_REG,
			      mcu_data->batched_list);
	cwmcu_i2c_write_group(mcu_data, CW_WAKE_UP_BATCH_ENABLE_REG,
			      mcu_data->batched_list >> 32);

	D("%s: enable_list = 0x%llx\n", __func__, mcu_data->enabled_list);

	for (i = 0; i < CW_SENSORS_ID_TOTAL; i++) {
		delay_ms = mcu_data->report_period[i] / MS_TO_PERIOD;

		rc = firmware_odr(mcu_data, i, delay_ms);
		if (rc) {
			E("%s: firmware_odr fails, rc = %d, i = %d\n",
			  __func__, rc, i);
			return -EIO;
		}
	}

#ifdef MCU_WARN_MSGS
	reg_value = 1;
	rc = CWMCU_i2c_write(mcu_data, CW_I2C_REG_WARN_MSG_ENABLE,
			     &reg_value, 1);
	if (rc) {
		E("%s: CWMCU_i2c_write(WARN_MSG) fails, rc = %d, i = %d\n",
		  __func__, rc, i);
		return -EIO;
	}
	D("%s: WARN_MSGS enabled\n", __func__);
#endif

	reg_value = 1;
	rc = CWMCU_i2c_write(mcu_data, CW_I2C_REG_WATCH_DOG_ENABLE,
			     &reg_value, 1);
	if (rc) {
		E("%s: CWMCU_i2c_write(WATCH_DOG) fails, rc = %d\n",
		  __func__, rc);
		return -EIO;
	}
	D("%s: Watch dog enabled\n", __func__);

	/* Inform SensorHUB that CPU is going to resume */
	data = 1;
	CWMCU_i2c_write(mcu_data, CW_CPU_STATUS_REG, &data, 1);
	D("%s: write_addr = 0x%x, write_data = 0x%x\n", __func__,
	  CW_CPU_STATUS_REG, data);

	return 0;
}

static int check_fw_version(struct cwmcu_data *mcu_data,
			    const struct firmware *fw)
{
	u8 firmware_version[FW_VER_COUNT] = {0};
	u8 fw_version[FW_VER_COUNT];
	char char_ver[4];
	unsigned long ul_ver;
	int i;
	int rc;

	if (mcu_data->fw_update_status & (FW_FLASH_FAILED | FW_ERASE_FAILED))
		return 1;

	CWMCU_i2c_read(mcu_data, FIRMWARE_VERSION, firmware_version,
		       sizeof(firmware_version));

	/* Version example: HTCSHUB001.000.001.005.000.001 */
	if (!strncmp(&fw->data[fw->size - FW_VER_INFO_LEN], "HTCSHUB",
		     sizeof("HTCSHUB") - 1)) {
		for (i = 0; i < FW_VER_COUNT; i++) {
			memcpy(char_ver, &fw->data[fw->size - FW_VER_INFO_LEN +
						   FW_VER_HEADER_LEN + (i * 4)],
			       sizeof(char_ver));
			char_ver[sizeof(char_ver) - 1] = 0;
			rc = kstrtol(char_ver, 10, &ul_ver);
			if (rc) {
				E("%s: kstrtol fails, rc = %d, i = %d\n",
					__func__, rc, i);
				return rc;
			}
			fw_version[i] = ul_ver;
			D(
			  "%s: fw_version[%d] = %u, firmware_version[%d] ="
			  " %u\n", __func__, i, fw_version[i]
			  , i, firmware_version[i]);
		}

		if (memcmp(firmware_version, fw_version,
			   sizeof(firmware_version))) {
			I("%s: Sensor HUB firmware update is required\n",
			  __func__);
			return 1;
		} else {
			I("%s: Sensor HUB firmware is up-to-date\n", __func__);
			return 0;
		}

	} else {
		E("%s: fw version incorrect!\n", __func__);
		return -ESPIPE;
	}
	return 0;
}

static int i2c_rx_bytes_locked(struct cwmcu_data *mcu_data, u8 *data,
			 u16 length)
{
	int retry;

	struct i2c_msg msg[] = {
		{
			.addr = mcu_data->client->addr,
			.flags = I2C_M_RD,
			.len = length ,
			.buf = data,
		}
	};

	for (retry = 0; retry < UPDATE_FIRMWARE_RETRY_TIMES; retry++) {
		if (__i2c_transfer(mcu_data->client->adapter, msg, 1) == 1)
			break;
		mdelay(10);
	}

	if (retry == UPDATE_FIRMWARE_RETRY_TIMES) {
		E("%s: Retry over %d\n", __func__,
				UPDATE_FIRMWARE_RETRY_TIMES);
		return -EIO;
	}
	return 0;
}

static int i2c_tx_bytes_locked(struct cwmcu_data *mcu_data, u8 *data,
			 u16 length)
{
	int retry;

	struct i2c_msg msg[] = {
		{
			.addr = mcu_data->client->addr,
			.flags = 0,
			.len = length ,
			.buf = data,
		}
	};

	for (retry = 0; retry < UPDATE_FIRMWARE_RETRY_TIMES; retry++) {
		if (__i2c_transfer(mcu_data->client->adapter, msg, 1) == 1)
			break;
		mdelay(10);
	}

	if (retry == UPDATE_FIRMWARE_RETRY_TIMES) {
		E("%s: Retry over %d\n", __func__,
				UPDATE_FIRMWARE_RETRY_TIMES);
		return -EIO;
	}
	return 0;
}

static int erase_mcu_flash_mem(struct cwmcu_data *mcu_data)
{
	u8 i2c_data[3] = {0};
	int rc;

	i2c_data[0] = 0x44;
	i2c_data[1] = 0xBB;
	rc = i2c_tx_bytes_locked(mcu_data, i2c_data, 2);
	if (rc) {
		E("%s: Failed to write 0xBB44, rc = %d\n", __func__, rc);
		return rc;
	}

	rc = i2c_rx_bytes_locked(mcu_data, i2c_data, 1);
	if (rc) {
		E("%s: Failed to read, rc = %d\n", __func__, rc);
		return rc;
	}

	if (i2c_data[0] != FW_RESPONSE_CODE) {
		E("%s: FW NACK, i2c_data = 0x%x\n", __func__, i2c_data[0]);
		return 1;
	}


	i2c_data[0] = 0xFF;
	i2c_data[1] = 0xFF;
	i2c_data[2] = 0;
	rc = i2c_tx_bytes_locked(mcu_data, i2c_data, 3);
	if (rc) {
		E("%s: Failed to write_2, rc = %d\n", __func__, rc);
		return rc;
	}

	D("%s: Tx size = %d\n", __func__, 3);
	/* Erase needs 9 sec in worst case */
	msleep(mcu_data->erase_fw_wait + FW_ERASE_MIN);
	D("%s: After delay, Tx size = %d\n", __func__, 3);

	return 0;
}

static int update_mcu_flash_mem_block(struct cwmcu_data *mcu_data,
				      u32 start_address,
				      u8 write_buf[],
				      int numberofbyte)
{
	u8 i2c_data[FW_I2C_LEN_LIMIT+2] = {0};
	__be32 to_i2c_command;
	int data_len, checksum;
	int i;
	int rc;

	i2c_data[0] = 0x31;
	i2c_data[1] = 0xCE;
	rc = i2c_tx_bytes_locked(mcu_data, i2c_data, 2);
	if (rc) {
		E("%s: Failed to write 0xCE31, rc = %d\n", __func__, rc);
		return rc;
	}

	rc = i2c_rx_bytes_locked(mcu_data, i2c_data, 1);
	if (rc) {
		E("%s: Failed to read, rc = %d\n", __func__, rc);
		return rc;
	}

	if (i2c_data[0] != FW_RESPONSE_CODE) {
		E("%s: FW NACK, i2c_data = 0x%x\n", __func__, i2c_data[0]);
		return 1;
	}


	to_i2c_command = cpu_to_be32(start_address);
	memcpy(i2c_data, &to_i2c_command, sizeof(__be32));
	i2c_data[4] = i2c_data[0] ^ i2c_data[1] ^ i2c_data[2] ^ i2c_data[3];
	rc = i2c_tx_bytes_locked(mcu_data, i2c_data, 5);
	if (rc) {
		E("%s: Failed to write_2, rc = %d\n", __func__, rc);
		return rc;
	}

	rc = i2c_rx_bytes_locked(mcu_data, i2c_data, 1);
	if (rc) {
		E("%s: Failed to read_2, rc = %d\n", __func__, rc);
		return rc;
	}

	if (i2c_data[0] != FW_RESPONSE_CODE) {
		E("%s: FW NACK_2, i2c_data = 0x%x\n", __func__, i2c_data[0]);
		return 1;
	}


	checksum = 0x0;
	data_len = numberofbyte + 2;

	i2c_data[0] = numberofbyte - 1;

	for (i = 0; i < numberofbyte; i++)
		i2c_data[i+1] = write_buf[i];

	for (i = 0; i < (data_len - 1); i++)
		checksum ^= i2c_data[i];

	i2c_data[i] = checksum;
	rc = i2c_tx_bytes_locked(mcu_data, i2c_data, data_len);
	if (rc) {
		E("%s: Failed to write_3, rc = %d\n", __func__, rc);
		return rc;
	}

	i = numberofbyte * 35;
	usleep_range(i, i + 1000);

	rc = i2c_rx_bytes_locked(mcu_data, i2c_data, 1);
	if (rc) {
		E("%s: Failed to read_3, rc = %d\n", __func__, rc);
		return rc;
	}

	if (i2c_data[0] != FW_RESPONSE_CODE) {
		E("%s: FW NACK_3, i2c_data = 0x%x\n", __func__, i2c_data[0]);
		return 1;
	}

	return 0;
}

static void update_firmware(const struct firmware *fw, void *context)
{
	struct cwmcu_data *mcu_data = context;
	int  ret;
	u8 write_buf[FW_I2C_LEN_LIMIT] = {0};
	int block_size, data_len;
	u32 address_point;
	int i;

	cwmcu_powermode_switch(mcu_data, 1);

	if (!fw) {
		E("%s: fw does not exist\n", __func__);
		mcu_data->fw_update_status |= FW_DOES_NOT_EXIST;
		goto fast_exit;
	}

	D("%s: firmware size = %lu\n", __func__, fw->size);

	ret = check_fw_version(mcu_data, fw);
	if (ret == 1) { /* Perform firmware update */

		mutex_lock(&mcu_data->activated_i2c_lock);
		i2c_lock_adapter(mcu_data->client->adapter);

		mcu_data->client->addr = 0x39;

		gpio_direction_output(mcu_data->gpio_chip_mode, 1);
		mdelay(10);
		gpio_direction_output(mcu_data->gpio_reset, 0);
		mdelay(10);
		gpio_direction_output(mcu_data->gpio_reset, 1);
		mdelay(41);

		mcu_data->fw_update_status |= FW_FLASH_FAILED;

		ret = erase_mcu_flash_mem(mcu_data);
		if (ret) {
			E("%s: erase mcu flash memory fails, ret = %d\n",
			  __func__, ret);
			mcu_data->fw_update_status |= FW_ERASE_FAILED;
		} else {
			mcu_data->fw_update_status &= ~FW_ERASE_FAILED;
		}

		D("%s: Start writing firmware\n", __func__);

		block_size = fw->size / FW_I2C_LEN_LIMIT;
		data_len = fw->size % FW_I2C_LEN_LIMIT;
		address_point = 0x08000000;

		for (i = 0; i < block_size; i++) {
			memcpy(write_buf, &fw->data[FW_I2C_LEN_LIMIT*i],
			       FW_I2C_LEN_LIMIT);
			ret = update_mcu_flash_mem_block(mcu_data,
							 address_point,
							 write_buf,
							 FW_I2C_LEN_LIMIT);
			if (ret) {
				E("%s: update_mcu_flash_mem_block fails,"
				  "ret = %d, i = %d\n", __func__, ret, i);
				goto out;
			}
			address_point += FW_I2C_LEN_LIMIT;
		}

		if (data_len != 0) {
			memcpy(write_buf, &fw->data[FW_I2C_LEN_LIMIT*i],
			       data_len);
			ret = update_mcu_flash_mem_block(mcu_data,
							 address_point,
							 write_buf,
							 data_len);
			if (ret) {
				E("%s: update_mcu_flash_mem_block fails_2,"
				  "ret = %d\n", __func__, ret);
				goto out;
			}
		}
		mcu_data->fw_update_status &= ~FW_FLASH_FAILED;

out:
		D("%s: End writing firmware\n", __func__);

		gpio_direction_output(mcu_data->gpio_chip_mode, 0);
		mdelay(10);
		gpio_direction_output(mcu_data->gpio_reset, 0);
		mdelay(10);
		gpio_direction_output(mcu_data->gpio_reset, 1);

		/* HUB need at least 500ms to be ready */
		usleep_range(500000, 1000000);

		mcu_data->client->addr = 0x72;

		i2c_unlock_adapter(mcu_data->client->adapter);
		mutex_unlock(&mcu_data->activated_i2c_lock);

	}
	release_firmware(fw);

fast_exit:
	mcu_data->w_re_init = true;
	queue_work(mcu_data->mcu_wq, &mcu_data->one_shot_work);

	cwmcu_powermode_switch(mcu_data, 0);

	mcu_data->fw_update_status &= ~FW_UPDATE_QUEUED;
	D("%s: fw_update_status = 0x%x\n", __func__,
	  mcu_data->fw_update_status);

	if (mcu_data->erase_fw_wait <= (FW_ERASE_MAX - FW_ERASE_MIN - 1000))
		mcu_data->erase_fw_wait += 1000;
}

/* Returns the number of read bytes on success */
static int CWMCU_i2c_read(struct cwmcu_data *mcu_data,
			 u8 reg_addr, void *data, u8 len)
{
	s32 rc = 0;
	u8 *u8_data = data;

	D("%s++: reg_addr = 0x%x, len = %d\n", __func__, reg_addr, len);

	if (DEBUG_DISABLE) {
		mcu_data->disable_access_count++;
		if ((mcu_data->disable_access_count % 100) == 0)
			I("%s: DEBUG_DISABLE = %d\n", __func__, DEBUG_DISABLE);
		return len;
	}

	if (mcu_data->is_block_i2c) {
		if (time_after(jiffies,
			       mcu_data->reset_jiffies + RESET_PERIOD))
			mcu_data->is_block_i2c = 0;
		return len;
	}

	mutex_lock(&mcu_data->mutex_lock);
	if (mcu_data->suspended) {
		mutex_unlock(&mcu_data->mutex_lock);
		return len;
	}
	mutex_unlock(&mcu_data->mutex_lock);

	mutex_lock(&mcu_data->activated_i2c_lock);
	if (retry_exhausted(mcu_data)) {
		memset(u8_data, 0, len); /* Assign data to 0 when chip NACK */

		/* Try to recover HUB in low CPU utilization */
		D(
		  "%s: mcu_data->i2c_total_retry = %d, "
		  "mcu_data->i2c_latch_retry = %d\n", __func__,
		  mcu_data->i2c_total_retry,
		  mcu_data->i2c_latch_retry);
		mcu_data->w_activated_i2c = true;
		queue_work(mcu_data->mcu_wq, &mcu_data->one_shot_work);

		mutex_unlock(&mcu_data->activated_i2c_lock);
		return len;
	}

	while (!retry_exhausted(mcu_data)) {
		rc = i2c_smbus_read_i2c_block_data(mcu_data->client, reg_addr,
						   len, u8_data);
		if (rc == len) {
			retry_reset(mcu_data);
			break;
		} else {
			gpio_make_falling_edge(mcu_data->gpio_wake_mcu);
			mcu_data->i2c_total_retry++;
			if (rc == LATCH_ERROR_NO)
				mcu_data->i2c_latch_retry++;
			E("%s: rc = %d, total_retry = %d, latch_retry = %d\n",
			  __func__,
			  rc, mcu_data->i2c_total_retry,
			  mcu_data->i2c_latch_retry);
		}
	}

	if (retry_exhausted(mcu_data)) {
		E("%s: total_retry = %d, latch_retry = %d, return\n",
		  __func__, mcu_data->i2c_total_retry,
		  mcu_data->i2c_latch_retry);
	}

	mutex_unlock(&mcu_data->activated_i2c_lock);

	return rc;
}

static bool reset_hub(struct cwmcu_data *mcu_data)
{
	if (time_after(jiffies, mcu_data->reset_jiffies + RESET_PERIOD)) {
		gpio_direction_output(mcu_data->gpio_reset, 0);
		D("%s: gpio_reset = %d\n", __func__,
		  gpio_get_value_cansleep(mcu_data->gpio_reset));
		usleep_range(10000, 15000);
		gpio_direction_output(mcu_data->gpio_reset, 1);
		D("%s: gpio_reset = %d\n", __func__,
		  gpio_get_value_cansleep(mcu_data->gpio_reset));

		retry_reset(mcu_data);
		mcu_data->i2c_jiffies = jiffies;

		/* HUB need at least 500ms to be ready */
		usleep_range(500000, 1000000);
		mcu_data->is_block_i2c = false;
	} else
		mcu_data->is_block_i2c = true;

	mcu_data->reset_jiffies = jiffies;
	return !mcu_data->is_block_i2c;
}

/* This informs firmware for Output Data Rate of each sensor.
 * Need powermode held by caller */
static int firmware_odr(struct cwmcu_data *mcu_data, int sensors_id,
			int delay_ms)
{
	u8 reg_addr;
	u8 reg_value;
	int rc;

	switch (sensors_id) {
	case CW_ACCELERATION:
		reg_addr = ACCE_UPDATE_RATE;
		break;
	case CW_MAGNETIC:
		reg_addr = MAGN_UPDATE_RATE;
		break;
	case CW_GYRO:
		reg_addr = GYRO_UPDATE_RATE;
		break;
	case CW_ORIENTATION:
		reg_addr = ORIE_UPDATE_RATE;
		break;
	case CW_ROTATIONVECTOR:
		reg_addr = ROTA_UPDATE_RATE;
		break;
	case CW_LINEARACCELERATION:
		reg_addr = LINE_UPDATE_RATE;
		break;
	case CW_GRAVITY:
		reg_addr = GRAV_UPDATE_RATE;
		break;
	case CW_MAGNETIC_UNCALIBRATED:
		reg_addr = MAGN_UNCA_UPDATE_RATE;
		break;
	case CW_GYROSCOPE_UNCALIBRATED:
		reg_addr = GYRO_UNCA_UPDATE_RATE;
		break;
	case CW_GAME_ROTATION_VECTOR:
		reg_addr = GAME_ROTA_UPDATE_RATE;
		break;
	case CW_GEOMAGNETIC_ROTATION_VECTOR:
		reg_addr = GEOM_ROTA_UPDATE_RATE;
		break;
	case CW_SIGNIFICANT_MOTION:
		reg_addr = SIGN_UPDATE_RATE;
		break;
	case CW_PRESSURE:
		reg_addr = PRESSURE_UPDATE_RATE;
		break;
	case CW_STEP_COUNTER:
		reg_addr = STEP_COUNTER_UPDATE_PERIOD;
		break;
	case CW_ACCELERATION_W:
		reg_addr = ACCE_WAKE_UPDATE_RATE;
		break;
	case CW_MAGNETIC_W:
		reg_addr = MAGN_WAKE_UPDATE_RATE;
		break;
	case CW_GYRO_W:
		reg_addr = GYRO_WAKE_UPDATE_RATE;
		break;
	case CW_PRESSURE_W:
		reg_addr = PRESSURE_WAKE_UPDATE_RATE;
		break;
	case CW_ORIENTATION_W:
		reg_addr = ORIE_WAKE_UPDATE_RATE;
		break;
	case CW_ROTATIONVECTOR_W:
		reg_addr = ROTA_WAKE_UPDATE_RATE;
		break;
	case CW_LINEARACCELERATION_W:
		reg_addr = LINE_WAKE_UPDATE_RATE;
		break;
	case CW_GRAVITY_W:
		reg_addr = GRAV_WAKE_UPDATE_RATE;
		break;
	case CW_MAGNETIC_UNCALIBRATED_W:
		reg_addr = MAGN_UNCA_WAKE_UPDATE_RATE;
		break;
	case CW_GYROSCOPE_UNCALIBRATED_W:
		reg_addr = GYRO_UNCA_WAKE_UPDATE_RATE;
		break;
	case CW_GAME_ROTATION_VECTOR_W:
		reg_addr = GAME_ROTA_WAKE_UPDATE_RATE;
		break;
	case CW_GEOMAGNETIC_ROTATION_VECTOR_W:
		reg_addr = GEOM_ROTA_WAKE_UPDATE_RATE;
		break;
	case CW_STEP_COUNTER_W:
		reg_addr = STEP_COUNTER_UPDATE_PERIOD;
		break;
	default:
		reg_addr = 0;
		D(
		  "%s: Only report_period changed, sensors_id = %d,"
		  " delay_us = %6d\n",
		  __func__, sensors_id,
		  mcu_data->report_period[sensors_id]);
		return 0;
	}

	if (delay_ms >= 200)
		reg_value = UPDATE_RATE_NORMAL;
	else if (delay_ms >= 100)
		reg_value = UPDATE_RATE_RATE_10Hz;
	else if (delay_ms >= 60)
		reg_value = UPDATE_RATE_UI;
	else if (delay_ms >= 40)
		reg_value = UPDATE_RATE_RATE_25Hz;
	else if (delay_ms >= 20)
		reg_value = UPDATE_RATE_GAME;
	else
		reg_value = UPDATE_RATE_FASTEST;


	if ((sensors_id != CW_STEP_COUNTER) && (sensors_id != CW_LIGHT) &&
	    (sensors_id != CW_STEP_COUNTER_W)) {
		D("%s: reg_addr = 0x%x, reg_value = 0x%x\n",
		  __func__, reg_addr, reg_value);

		rc = CWMCU_i2c_write(mcu_data, reg_addr, &reg_value, 1);
		if (rc) {
			E("%s: CWMCU_i2c_write fails, rc = %d\n", __func__, rc);
			return -EIO;
		}
	} else {
		__le32 period_data;

		period_data = cpu_to_le32(delay_ms);

		D("%s: reg_addr = 0x%x, period_data = 0x%x\n",
		  __func__, reg_addr, period_data);

		rc = CWMCU_i2c_multi_write(mcu_data, reg_addr,
					   &period_data,
					   sizeof(period_data));
		if (rc) {
			E("%s: CWMCU_i2c_multi_write fails, rc = %d\n",
			  __func__, rc);
			return -EIO;
		}
	}

	return 0;
}

int is_continuous_sensor(int sensors_id)
{
	switch (sensors_id) {
	case CW_ACCELERATION:
	case CW_MAGNETIC:
	case CW_GYRO:
	case CW_PRESSURE:
	case CW_ORIENTATION:
	case CW_ROTATIONVECTOR:
	case CW_LINEARACCELERATION:
	case CW_GRAVITY:
	case CW_MAGNETIC_UNCALIBRATED:
	case CW_GYROSCOPE_UNCALIBRATED:
	case CW_GAME_ROTATION_VECTOR:
	case CW_GEOMAGNETIC_ROTATION_VECTOR:
	case CW_ACCELERATION_W:
	case CW_MAGNETIC_W:
	case CW_GYRO_W:
	case CW_PRESSURE_W:
	case CW_ORIENTATION_W:
	case CW_ROTATIONVECTOR_W:
	case CW_LINEARACCELERATION_W:
	case CW_GRAVITY_W:
	case CW_MAGNETIC_UNCALIBRATED_W:
	case CW_GYROSCOPE_UNCALIBRATED_W:
	case CW_GAME_ROTATION_VECTOR_W:
	case CW_GEOMAGNETIC_ROTATION_VECTOR_W:
		return 1;
		break;
	default:
		return 0;
		break;
	}
}

static void setup_delay(struct cwmcu_data *mcu_data)
{
	u8 i;
	int delay_ms;
	int delay_candidate_ms;

	delay_candidate_ms = CWMCU_NO_POLLING_DELAY;
	for (i = 0; i < CW_SENSORS_ID_TOTAL; i++) {
		D("%s: batch_timeout[%d] = %lld\n", __func__, i,
		  mcu_data->batch_timeout[i]);
		if ((mcu_data->enabled_list & (1LL << i)) &&
		    is_continuous_sensor(i) &&
		    (mcu_data->batch_timeout[i] == 0)) {
			D("%s: report_period[%d] = %d\n", __func__, i,
			  mcu_data->report_period[i]);

			/* report_period is actual delay(us) * 0.99), convert to
			 * microseconds */
			delay_ms = mcu_data->report_period[i] /
					MS_TO_PERIOD;
			if (delay_ms > CWMCU_MAX_DELAY)
				delay_ms = CWMCU_MAX_DELAY;

			if (delay_candidate_ms > delay_ms)
				delay_candidate_ms = delay_ms;
		}
	}

	if (delay_candidate_ms != atomic_read(&mcu_data->delay)) {
		cancel_delayed_work_sync(&mcu_data->work);
		if (mcu_data->enabled_list & IIO_SENSORS_MASK) {
			atomic_set(&mcu_data->delay, delay_candidate_ms);
			queue_delayed_work(mcu_data->mcu_wq, &mcu_data->work,
					   0);
		} else
			atomic_set(&mcu_data->delay, CWMCU_MAX_DELAY + 1);
	}

	D("%s: Minimum delay = %dms\n", __func__,
	  atomic_read(&mcu_data->delay));

}

static int handle_batch_list(struct cwmcu_data *mcu_data, int sensors_id,
			     bool is_wake)
{
	int rc;
	u8 i;
	u8 data;
	u64 sensors_bit;
	u8 write_addr;

	if ((sensors_id == CW_LIGHT) || (sensors_id == CW_SIGNIFICANT_MOTION))
		return 0;

	sensors_bit = (1LL << sensors_id);
	mcu_data->batched_list &= ~sensors_bit;
	mcu_data->batched_list |= (mcu_data->enabled_list & sensors_bit)
					? sensors_bit : 0;

	D("%s: sensors_bit = 0x%llx, batched_list = 0x%llx\n", __func__,
	  sensors_bit, mcu_data->batched_list);

	i = (sensors_id / 8);
	data = (u8)(mcu_data->batched_list >> (i*8));

	write_addr = (is_wake) ? CW_WAKE_UP_BATCH_ENABLE_REG :
				 CW_BATCH_ENABLE_REG;

	if (i > 3)
		i = (i - 4);

	D("%s: Writing, addr = 0x%x, data = 0x%x\n", __func__,
	  (write_addr+i), data);

	rc = CWMCU_i2c_write_power(mcu_data, write_addr+i, &data, 1);
	if (rc)
		E("%s: CWMCU_i2c_write fails, rc = %d\n",
		  __func__, rc);

	return rc;
}

static int setup_batch_timeout(struct cwmcu_data *mcu_data, bool is_wake)
{
	__le32 timeout_data;
	s64 current_timeout;
	u32 continuous_sensor_count;
	u8 i;
	u8 write_addr;
	int rc;
	int scan_limit;

	current_timeout = 0;
	if (is_wake) {
		i = CW_ACCELERATION_W;
		scan_limit = CW_SENSORS_ID_TOTAL;
	} else {
		i = CW_ACCELERATION;
		scan_limit = CW_SENSORS_ID_FW;
	}
	for (continuous_sensor_count = 0; i < scan_limit; i++) {
		if (mcu_data->batch_timeout[i] != 0) {
			if ((current_timeout >
			     mcu_data->batch_timeout[i]) ||
			    (current_timeout == 0)) {
				current_timeout =
					mcu_data->batch_timeout[i];
			}
			D("sensorid = %d, current_timeout = %lld\n",
			  i, current_timeout);
		} else
			continuous_sensor_count++;
	}

	if (continuous_sensor_count == scan_limit)
		current_timeout = 0;

	timeout_data = cpu_to_le32(current_timeout);

	write_addr = (is_wake) ? CWSTM32_WAKE_UP_BATCH_MODE_TIMEOUT :
				 CWSTM32_BATCH_MODE_TIMEOUT;

	D(
	  "%s: Writing, write_addr = 0x%x, current_timeout = %lld,"
	  " timeout_data = 0x%x\n",
	  __func__, write_addr, current_timeout, timeout_data);

	cwmcu_powermode_switch(mcu_data, 1);
	rc = CWMCU_i2c_multi_write(mcu_data, write_addr,
				   &timeout_data,
				   sizeof(timeout_data));
	cwmcu_powermode_switch(mcu_data, 0);
	if (rc)
		E("%s: CWMCU_i2c_write fails, rc = %d\n", __func__, rc);

	return rc;
}

static u64 report_step_counter(struct cwmcu_data *mcu_data, u32 fw_step,
			       u64 timestamp, bool is_wake)
{
	u16 u16_data_buff[REPORT_EVENT_COMMON_LEN * 2];
	u64 step_counter_buff;

	mcu_data->sensors_time[CW_STEP_COUNTER] = 0;

	step_counter_buff = mcu_data->step_counter_base + fw_step;

	u16_data_buff[0] = step_counter_buff & 0xFFFF;
	u16_data_buff[1] = (step_counter_buff >> 16) & 0xFFFF;
	u16_data_buff[2] = 0;
	u16_data_buff[3] = (step_counter_buff >> 32) & 0xFFFF;
	u16_data_buff[4] = (step_counter_buff >> 48) & 0xFFFF;
	u16_data_buff[5] = 0;

	cw_send_event_special(mcu_data, (is_wake) ? CW_STEP_COUNTER_W
						  : CW_STEP_COUNTER,
			      u16_data_buff,
			      u16_data_buff + REPORT_EVENT_COMMON_LEN,
			      timestamp);

	return step_counter_buff;
}

static ssize_t active_set(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	long enabled = 0;
	long sensors_id = 0;
	u8 data;
	u8 i;
	char *str_buf;
	char *running;
	u64 sensors_bit;
	int rc;
	bool is_wake;
	bool non_wake_bit;
	bool wake_bit;
	u32 write_list;

	str_buf = kstrndup(buf, count, GFP_KERNEL);
	if (str_buf == NULL) {
		E("%s: cannot allocate buffer\n", __func__);
		return -ENOMEM;
	}
	running = str_buf;

	for (i = 0; i < 2; i++) {
		int error;
		char *token;

		token = strsep(&running, " ");

		if (i == 0)
			error = kstrtol(token, 10, &sensors_id);
		else {
			if (token == NULL) {
				enabled = sensors_id;
				sensors_id = 0;
			} else
				error = kstrtol(token, 10, &enabled);
		}
		if (error) {
			E("%s: kstrtol fails, error = %d, i = %d\n",
				__func__, error, i);
			kfree(str_buf);
			return error;
		}
	}
	kfree(str_buf);

	if (!mcu_data->probe_success)
		return -EBUSY;

	if ((sensors_id >= CW_SENSORS_ID_TOTAL) ||
	    (sensors_id < 0)
	   ) {
		E("%s: Invalid sensors_id = %ld\n", __func__, sensors_id);
		return -EINVAL;
	}

	sensors_bit = 1LL << sensors_id;

	is_wake = (sensors_id >= CW_ACCELERATION_W) &&
		  (sensors_id <= CW_STEP_COUNTER_W);
	if (is_wake) {
		wake_bit = (mcu_data->enabled_list & sensors_bit);
		non_wake_bit = (mcu_data->enabled_list & (sensors_bit >> 32));
	} else {
		wake_bit = (mcu_data->enabled_list & (sensors_bit << 32));
		non_wake_bit = (mcu_data->enabled_list & sensors_bit);
	}

	mcu_data->enabled_list &= ~sensors_bit;
	mcu_data->enabled_list |= enabled ? sensors_bit : 0;

	/* clean batch parameters if sensor turn off */
	if (!enabled) {
		mcu_data->batch_timeout[sensors_id] = 0;
		mcu_data->batched_list &= ~sensors_bit;
		mcu_data->sensors_time[sensors_id] = 0;
		setup_batch_timeout(mcu_data, is_wake);
		mcu_data->report_period[sensors_id] = 200000 * MS_TO_PERIOD;
		mcu_data->pending_flush &= ~(sensors_bit);
	} else {
		do_gettimeofday(&mcu_data->now);
		mcu_data->sensors_time[sensors_id] =
			(mcu_data->now.tv_sec * NS_PER_US) +
			mcu_data->now.tv_usec;
	}

	write_list = mcu_data->enabled_list | (mcu_data->enabled_list >> 32);

	i = ((is_wake) ? (sensors_id - 32) : sensors_id) / 8;
	data = (u8)(write_list >> (i*8));

	if (enabled
	    ? !(wake_bit | non_wake_bit)
	    : (wake_bit ^ non_wake_bit)) {
		D("%s: Writing: CWSTM32_ENABLE_REG+i = 0x%x, data = 0x%x\n",
		  __func__, CWSTM32_ENABLE_REG+i, data);
		rc = CWMCU_i2c_write_power(mcu_data, CWSTM32_ENABLE_REG+i,
					   &data, 1);
		if (rc) {
			E("%s: CWMCU_i2c_write fails, rc = %d\n",
			  __func__, rc);
			return -EIO;
		}

		/* Disabling Step counter and no other step counter enabled */
		if (((sensors_id == CW_STEP_COUNTER) ||
		     (sensors_id == CW_STEP_COUNTER_W))
		    && !enabled
		    && !(mcu_data->enabled_list & STEP_COUNTER_MASK)) {
			__le32 data[3];

			rc = CWMCU_i2c_read_power(mcu_data,
					    CWSTM32_READ_STEP_COUNTER,
					    data, sizeof(data));
			if (rc >= 0) {
				mcu_data->step_counter_base +=
							le32_to_cpu(data[2]);
				D("%s: Record step = %llu\n",
				  __func__, mcu_data->step_counter_base);
			} else {
				D("%s: Step Counter i2c read fails, rc = %d\n",
				  __func__, rc);
			}
		}

		if (!enabled
		    && (!(mcu_data->enabled_list & IIO_CONTINUOUS_MASK))) {
			mutex_lock(&mcu_data->mutex_lock);
			mcu_data->w_clear_fifo_running = true;
			mcu_data->w_clear_fifo = true;
			mutex_unlock(&mcu_data->mutex_lock);
			queue_work(mcu_data->mcu_wq, &mcu_data->one_shot_work);
		}

	}

	cwmcu_powermode_switch(mcu_data, 1);
	rc = firmware_odr(mcu_data, sensors_id,
			  mcu_data->report_period[sensors_id] / MS_TO_PERIOD);
	cwmcu_powermode_switch(mcu_data, 0);
	if (rc) {
		E("%s: firmware_odr fails, rc = %d\n", __func__, rc);
	}

	if ((sensors_id == CW_LIGHT) && (!!enabled)) {
		D("%s: Initial lightsensor = %d\n",
		  __func__, mcu_data->light_last_data[0]);
		cw_send_event(mcu_data, CW_LIGHT,
			      mcu_data->light_last_data, 0);
	}

	setup_delay(mcu_data);

	rc = handle_batch_list(mcu_data, sensors_id, is_wake);
	if (rc) {
		E("%s: handle_batch_list fails, rc = %d\n", __func__,
		  rc);
		return rc;
	}

	D("%s: sensors_id = %ld, enable = %ld, enable_list = 0x%llx\n",
		__func__, sensors_id, enabled, mcu_data->enabled_list);

	return count;
}

static ssize_t active_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	u32 data = 0;

	CWMCU_i2c_read_power(mcu_data, CWSTM32_ENABLE_REG, &data, sizeof(data));

	D("%s: enable = 0x%x\n", __func__, data);

	return scnprintf(buf, PAGE_SIZE, "0x%llx, 0x%x\n",
			 mcu_data->enabled_list, data);
}

static ssize_t interval_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&mcu_data->delay));
}

static ssize_t interval_set(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	long val = 0;
	long sensors_id = 0;
	int i, rc;
	char *str_buf;
	char *running;

	str_buf = kstrndup(buf, count, GFP_KERNEL);
	if (str_buf == NULL) {
		E("%s: cannot allocate buffer\n", __func__);
		return -ENOMEM;
	}
	running = str_buf;

	for (i = 0; i < 2; i++) {
		int error;
		char *token;

		token = strsep(&running, " ");

		if (i == 0)
			error = kstrtol(token, 10, &sensors_id);
		else {
			if (token == NULL) {
				val = 66;
				D("%s: delay set to 66\n", __func__);
			} else
				error = kstrtol(token, 10, &val);
		}
		if (error) {
			E("%s: kstrtol fails, error = %d, i = %d\n",
				__func__, error, i);
			kfree(str_buf);
			return error;
		}
	}
	kfree(str_buf);

	if ((sensors_id < 0) || (sensors_id >= num_sensors)) {
		D("%s: Invalid sensors_id = %ld\n", __func__, sensors_id);
		return -EINVAL;
	}

	if (mcu_data->report_period[sensors_id] != val * MS_TO_PERIOD) {
		/* period is actual delay(us) * 0.99 */
		mcu_data->report_period[sensors_id] = val * MS_TO_PERIOD;

		setup_delay(mcu_data);

		cwmcu_powermode_switch(mcu_data, 1);
		rc = firmware_odr(mcu_data, sensors_id, val);
		cwmcu_powermode_switch(mcu_data, 0);
		if (rc) {
			E("%s: firmware_odr fails, rc = %d\n", __func__, rc);
			return rc;
		}
	}

	return count;
}


static ssize_t batch_set(struct device *dev,
		     struct device_attribute *attr,
		     const char *buf, size_t count)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	s64 timeout = 0;
	int sensors_id = 0, flag = 0, delay_ms = 0;
	u8 i;
	int retry;
	int rc;
	char *token;
	char *str_buf;
	char *running;
	long input_val;
	unsigned long long input_val_l;
	bool need_update_fw_odr;
	s32 period;
	bool is_wake;

	if (!mcu_data->probe_success) {
		E("%s: probe_success = %d\n", __func__,
		  mcu_data->probe_success);
		return -1;
	}

	for (retry = 0; retry < ACTIVE_RETRY_TIMES; retry++) {
		mutex_lock(&mcu_data->mutex_lock);
		if (mcu_data->suspended) {
			mutex_unlock(&mcu_data->mutex_lock);
			D("%s: suspended, retry = %d\n",
				__func__, retry);
			usleep_range(5000, 10000);
		} else {
			mutex_unlock(&mcu_data->mutex_lock);
			break;
		}
	}
	if (retry >= ACTIVE_RETRY_TIMES) {
		D("%s: resume not completed, retry = %d, retry fails!\n",
			__func__, retry);
		return -ETIMEDOUT;
	}

	str_buf = kstrndup(buf, count, GFP_KERNEL);
	if (str_buf == NULL) {
		E("%s: cannot allocate buffer\n", __func__);
		return -1;
	}
	running = str_buf;

	for (i = 0; i < 4; i++) {
		token = strsep(&running, " ");
		if (token == NULL) {
			E("%s: token = NULL, i = %d\n", __func__, i);
			break;
		}

		switch (i) {
		case 0:
			rc = kstrtol(token, 10, &input_val);
			sensors_id = (int)input_val;
			break;
		case 1:
			rc = kstrtol(token, 10, &input_val);
			flag = (int)input_val;
			break;
		case 2:
			rc = kstrtol(token, 10, &input_val);
			delay_ms = (int)input_val;
			break;
		case 3:
			rc = kstrtoull(token, 10, &input_val_l);
			timeout = (s64)input_val_l;
			break;
		default:
			E("%s: Unknown i = %d\n", __func__, i);
			break;
		}

		if (rc) {
			E("%s: kstrtol fails, rc = %d, i = %d\n",
			  __func__, rc, i);
			kfree(str_buf);
			return rc;
		}
	}
	kfree(str_buf);

	if ((sensors_id < 0) || (sensors_id >= num_sensors)) {
		D("%s: Invalid sensors_id = %d\n", __func__, sensors_id);
		return -EINVAL;
	}

	D("%s: sensors_id = 0x%x, flag = %d, delay_ms = %d, timeout = %lld\n",
	  __func__, sensors_id, flag, delay_ms, timeout);

	is_wake = (CW_ACCELERATION_W <= sensors_id) &&
				       (sensors_id <= CW_STEP_COUNTER_W);

	/* period is actual delay(us) * 0.99 */
	period = delay_ms * MS_TO_PERIOD;
	need_update_fw_odr = mcu_data->report_period[sensors_id] != period;
	D("%s: period = %d, report_period[%d] = %d\n",
	  __func__, period, sensors_id, mcu_data->report_period[sensors_id]);
	mcu_data->report_period[sensors_id] = period;

	switch (sensors_id) {
	case CW_ACCELERATION:
	case CW_MAGNETIC:
	case CW_GYRO:
	case CW_PRESSURE:
	case CW_ORIENTATION:
	case CW_ROTATIONVECTOR:
	case CW_LINEARACCELERATION:
	case CW_GRAVITY:
	case CW_MAGNETIC_UNCALIBRATED:
	case CW_GYROSCOPE_UNCALIBRATED:
	case CW_GAME_ROTATION_VECTOR:
	case CW_GEOMAGNETIC_ROTATION_VECTOR:
	case CW_STEP_DETECTOR:
	case CW_STEP_COUNTER:
	case CW_ACCELERATION_W:
	case CW_MAGNETIC_W:
	case CW_GYRO_W:
	case CW_PRESSURE_W:
	case CW_ORIENTATION_W:
	case CW_ROTATIONVECTOR_W:
	case CW_LINEARACCELERATION_W:
	case CW_GRAVITY_W:
	case CW_MAGNETIC_UNCALIBRATED_W:
	case CW_GYROSCOPE_UNCALIBRATED_W:
	case CW_GAME_ROTATION_VECTOR_W:
	case CW_GEOMAGNETIC_ROTATION_VECTOR_W:
	case CW_STEP_DETECTOR_W:
	case CW_STEP_COUNTER_W:
		break;
	case CW_LIGHT:
	case CW_SIGNIFICANT_MOTION:
	default:
		D("%s: Batch not supported for this sensor_id = 0x%x\n",
		  __func__, sensors_id);
		return count;
	}

	mcu_data->batch_timeout[sensors_id] = timeout;

	setup_delay(mcu_data);

	rc = setup_batch_timeout(mcu_data, is_wake);
	if (rc) {
		E("%s: setup_batch_timeout fails, rc = %d\n", __func__, rc);
		return rc;
	}

	if ((need_update_fw_odr == true) &&
	    (mcu_data->enabled_list & (1LL << sensors_id))) {
		int odr_sensors_id;

		odr_sensors_id = (is_wake) ? (sensors_id + 32) : sensors_id;

		cwmcu_powermode_switch(mcu_data, 1);
		rc = firmware_odr(mcu_data, odr_sensors_id, delay_ms);
		cwmcu_powermode_switch(mcu_data, 0);
		if (rc) {
			E("%s: firmware_odr fails, rc = %d\n", __func__, rc);
		}
	}

	D(
	  "%s: sensors_id = %d, timeout = %lld, batched_list = 0x%llx,"
	  " delay_ms = %d\n",
	  __func__, sensors_id, timeout, mcu_data->batched_list,
	  delay_ms);

	return (rc) ? rc : count;
}

static ssize_t batch_show(struct device *dev, struct device_attribute *attr,
		      char *buf)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	u64 timestamp = 0;
	struct timespec kt;
	u64 k_timestamp;

	kt = current_kernel_time();

	CWMCU_i2c_read_power(mcu_data, CW_I2C_REG_MCU_TIME, &timestamp,
			     sizeof(timestamp));

	le64_to_cpus(&timestamp);

	k_timestamp = (u64)(kt.tv_sec*NSEC_PER_SEC) + (u64)kt.tv_nsec;

	return scnprintf(buf, PAGE_SIZE, "%llu", timestamp);
}


static ssize_t flush_show(struct device *dev, struct device_attribute *attr,
		      char *buf)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	int ret;
	u8 data[4] = {0};

	ret = CWMCU_i2c_read_power(mcu_data, CWSTM32_BATCH_MODE_DATA_COUNTER,
			     data, sizeof(data));
	if (ret < 0)
		D("%s: Read Counter fail, ret = %d\n", __func__, ret);

	D("%s: DEBUG: Queue counter = %d\n", __func__,
	  *(u32 *)&data[0]);

	return scnprintf(buf, PAGE_SIZE, "Queue counter = %d\n",
			 *(u32 *)&data[0]);
}

static void cwmcu_send_flush(struct cwmcu_data *mcu_data, int id)
{
	u8 type = CW_META_DATA;
	u16 data[REPORT_EVENT_COMMON_LEN];
	s64 timestamp = 0;
	int rc;

	data[0] = (u16)id;
	data[1] = data[2] = 0;

	D("%s: flush sensor: %d!!\n", __func__, id);

	rc = cw_send_event(mcu_data, type, data, timestamp);
	if (rc < 0)
		E("%s: send_event fails, rc = %d\n", __func__, rc);

	D("%s--:\n", __func__);
}

static ssize_t flush_set(struct device *dev, struct device_attribute *attr,
		     const char *buf, size_t count)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	u8 data;
	unsigned long handle;
	int rc;

	rc = kstrtoul(buf, 10, &handle);
	if (rc) {
		E("%s: kstrtoul fails, rc = %d\n", __func__, rc);
		return rc;
	}

	D("%s: handle = %lu\n", __func__, handle);

	data = handle;

	D("%s: addr = 0x%x, data = 0x%x\n", __func__,
	  CWSTM32_BATCH_FLUSH, data);

	rc = CWMCU_i2c_write_power(mcu_data, CWSTM32_BATCH_FLUSH, &data, 1);
	if (rc)
		E("%s: CWMCU_i2c_write fails, rc = %d\n", __func__, rc);

	mcu_data->w_flush_fifo = true;
	queue_work(mcu_data->mcu_wq, &mcu_data->one_shot_work);

	if ((handle == CW_LIGHT) || (handle == CW_SIGNIFICANT_MOTION)) {
		mutex_lock(&mcu_data->lock);
		cwmcu_send_flush(mcu_data, handle);
		mutex_unlock(&mcu_data->lock);
	} else
		mcu_data->pending_flush |= (1LL << handle);

	D("%s: mcu_data->pending_flush = 0x%llx\n", __func__,
	  mcu_data->pending_flush);

	return count;
}

static ssize_t facedown_set(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t size)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	bool on;

	if (strtobool(buf, &on) < 0)
		return -EINVAL;

	if (!!on == !!(mcu_data->enabled_list &
			(1LL << HTC_FACEDOWN_DETECTION)))
		return size;

	if (on)
		mcu_data->enabled_list |= (1LL << HTC_FACEDOWN_DETECTION);
	else
		mcu_data->enabled_list &= ~(1LL << HTC_FACEDOWN_DETECTION);

	mcu_data->w_facedown_set = true;
	queue_work(mcu_data->mcu_wq, &mcu_data->one_shot_work);

	return size;
}

static ssize_t facedown_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n",
		!!(mcu_data->enabled_list & (1U << HTC_FACEDOWN_DETECTION)));
}

/* Return if META is read out */
static bool report_iio(struct cwmcu_data *mcu_data, int *i, u8 *data,
		       __le64 *data64, u32 *event_count, bool is_wake)
{
	s32 ret;
	u8 data_buff;
	u16 data_event[REPORT_EVENT_COMMON_LEN] = {0};
	u16 bias_event[REPORT_EVENT_COMMON_LEN] = {0};
	u16 timestamp_event;
	u64 *handle_time_base;
	bool is_meta_read = false;

	if (is_wake) {
		wake_lock_timeout(&mcu_data->report_wake_lock,
				  msecs_to_jiffies(200));
	}

	if (data[0] == CW_META_DATA) {
		__le16 *data16 = (__le16 *)(data + 1);

		data_event[0] = le16_to_cpup(data16 + 1);
		cw_send_event(mcu_data, data[0], data_event, 0);
		mcu_data->pending_flush &= ~(1LL << data_event[0]);
		D(
		  "total count = %u, current_count = %d, META from firmware,"
		  " event_id = %d, pending_flush = 0x%llx\n", *event_count, *i,
		  data_event[0], mcu_data->pending_flush);
		is_meta_read = true;
	} else if (data[0] == CW_TIME_BASE) {
		u64 timestamp;

		timestamp = le64_to_cpup(data64 + 1);

		handle_time_base = (is_wake) ? &mcu_data->wake_fifo_time_base :
					       &mcu_data->time_base;

		D(
		  "total count = %u, current_count = %d, CW_TIME_BASE = %llu,"
		  " is_wake = %d\n", *event_count, *i, timestamp, is_wake);
		*handle_time_base = timestamp;

	} else if (data[0] == CW_STEP_DETECTOR) {
		__le16 *data16 = (__le16 *)(data + 1);

		timestamp_event = le16_to_cpup(data16);

		data_event[0] = 1;
		handle_time_base = (is_wake) ?
					&mcu_data->wake_fifo_time_base :
					&mcu_data->time_base;
		cw_send_event(mcu_data,
			      (is_wake)
				? CW_STEP_DETECTOR_W
				: CW_STEP_DETECTOR
			      , data_event
			      , timestamp_event + *handle_time_base);

		D(
		  "Batch data: total count = %u, current count = %d, "
		  "STEP_DETECTOR%s, timediff = %d, time_base = %llu,"
		  " r_time = %llu\n"
		  , *event_count, *i, (is_wake) ? "_W" : ""
		  , timestamp_event
		  , *handle_time_base
		  , *handle_time_base + timestamp_event
		  );

	} else if (data[0] == CW_STEP_COUNTER) {
		__le16 *data16 = (__le16 *)(data + 1);
		__le32 *data32 = (__le32 *)(data + 3);

		timestamp_event = le16_to_cpup(data16);

		handle_time_base = (is_wake) ?
					&mcu_data->wake_fifo_time_base :
					&mcu_data->time_base;
		report_step_counter(mcu_data,
				    le32_to_cpu(*data32),
				    timestamp_event + *handle_time_base,
				    is_wake);

		D(
		  "Batch data: total count = %u, current count = %d, "
		  "STEP_COUNTER%s, step = %d, "
		  "timediff = %d, time_base = %llu, r_time = %llu\n"
		  , *event_count, *i, (is_wake) ? "_W" : ""
		  , le32_to_cpu(*data32)
		  , timestamp_event
		  , *handle_time_base
		  , *handle_time_base + timestamp_event
		  );

	} else if ((data[0] == CW_MAGNETIC_UNCALIBRATED_BIAS) ||
		   (data[0] == CW_GYROSCOPE_UNCALIBRATED_BIAS)) {
		__le16 *data16 = (__le16 *)(data + 1);
		u8 read_addr;

		data_buff = (data[0] == CW_MAGNETIC_UNCALIBRATED_BIAS) ?
				CW_MAGNETIC_UNCALIBRATED :
				CW_GYROSCOPE_UNCALIBRATED;
		data_buff += (is_wake) ? 32 : 0;

		bias_event[0] = le16_to_cpup(data16 + 1);
		bias_event[1] = le16_to_cpup(data16 + 2);
		bias_event[2] = le16_to_cpup(data16 + 3);

		read_addr = (is_wake) ? CWSTM32_WAKE_UP_BATCH_MODE_DATA_QUEUE :
					CWSTM32_BATCH_MODE_DATA_QUEUE;
		ret = CWMCU_i2c_read(mcu_data, read_addr, data, 9);
		if (ret >= 0) {
			(*i)++;
			timestamp_event = le16_to_cpup(data16);
			data_event[0] = le16_to_cpup(data16 + 1);
			data_event[1] = le16_to_cpup(data16 + 2);
			data_event[2] = le16_to_cpup(data16 + 3);

			D(
			  "Batch data: total count = %u, current "
			  "count = %d, event_id = %d, data(x, y, z) = "
			  "(%d, %d, %d), bias(x, y,  z) = "
			  "(%d, %d, %d)\n"
			  , *event_count, *i, data_buff
			  , data_event[0], data_event[1], data_event[2]
			  , bias_event[0], bias_event[1]
			  , bias_event[2]);

			handle_time_base = (is_wake) ?
						&mcu_data->wake_fifo_time_base :
						&mcu_data->time_base;
			cw_send_event_special(mcu_data, data_buff,
					      data_event,
					      bias_event,
					      timestamp_event +
					      *handle_time_base);
		} else {
			E("Read Uncalibrated data fails, ret = %d\n", ret);
		}
	} else {
		__le16 *data16 = (__le16 *)(data + 1);

		timestamp_event = le16_to_cpup(data16);
		data_event[0] = le16_to_cpup(data16 + 1);
		data_event[1] = le16_to_cpup(data16 + 2);
		data_event[2] = le16_to_cpup(data16 + 3);

		data[0] += (is_wake) ? 32 : 0;

		handle_time_base = (is_wake) ?
					&mcu_data->wake_fifo_time_base :
					&mcu_data->time_base;

		D(
		  "Batch data: total count = %u, current count = %d, "
		  "event_id = %d, data(x, y, z) = (%d, %d, %d), "
		  "timediff = %d, time_base = %llu, r_time = %llu\n"
		  , *event_count, *i, data[0]
		  , data_event[0], data_event[1], data_event[2]
		  , timestamp_event
		  , *handle_time_base
		  , *handle_time_base + timestamp_event
		  );

		if ((data[0] == CW_MAGNETIC) || (data[0] == CW_ORIENTATION)) {
			int rc;
			u8 accuracy;

			rc = CWMCU_i2c_read(mcu_data,
					    CW_I2C_REG_SENSORS_ACCURACY_MAG,
					    &accuracy, 1);
			if (rc < 0) {
				E(
				  "%s: read ACCURACY_MAG fails, rc = "
				  "%d\n", __func__, rc);
				accuracy = 3;
			}
			bias_event[0] = accuracy;

			cw_send_event_special(mcu_data, data[0], data_event,
					      bias_event,
					      timestamp_event +
					      *handle_time_base);
		} else {
			cw_send_event(mcu_data, data[0], data_event,
				      timestamp_event + *handle_time_base);
		}
	}
	return is_meta_read;
}

/* Return if META is read out */
static bool cwmcu_batch_fifo_read(struct cwmcu_data *mcu_data, int queue_id)
{
	s32 ret;
	int i;
	u32 *event_count;
	u8 event_count_data[4] = {0};
	u8 reg_addr;
	bool is_meta_read = false;

	mutex_lock(&mcu_data->lock);

	reg_addr = (queue_id)
			? CWSTM32_WAKE_UP_BATCH_MODE_DATA_COUNTER
			: CWSTM32_BATCH_MODE_DATA_COUNTER;

	ret = CWMCU_i2c_read(mcu_data, reg_addr, event_count_data,
			     sizeof(event_count_data));
	if (ret < 0) {
		D(
		  "Read Batched data Counter fail, ret = %d, queue_id"
		  " = %d\n", ret, queue_id);
	}

	event_count = (u32 *)(&event_count_data[0]);
	if (*event_count > MAX_EVENT_COUNT) {
		I("%s: event_count = %u, strange, queue_id = %d\n",
		  __func__, *event_count, queue_id);
		*event_count = 0;
	}

	D("%s: event_count = %u, queue_id = %d\n", __func__,
	  *event_count, queue_id);

	reg_addr = (queue_id) ? CWSTM32_WAKE_UP_BATCH_MODE_DATA_QUEUE :
		 CWSTM32_BATCH_MODE_DATA_QUEUE;

	for (i = 0; i < *event_count; i++) {
		__le64 data64[2];
		u8 *data = (u8 *)data64;

		data = data + 7;

		ret = CWMCU_i2c_read(mcu_data, reg_addr, data, 9);
		if (ret >= 0) {
			/* check if there are no data from queue */
			if (data[0] != CWMCU_NODATA) {
				is_meta_read = report_iio(mcu_data, &i, data,
							  data64,
							  event_count,
							  queue_id);
			}
		} else {
			E("Read Queue fails, ret = %d, queue_id = %d\n",
			  ret, queue_id);
		}
	}

	mutex_unlock(&mcu_data->lock);

	return is_meta_read;
}

static void cwmcu_meta_read(struct cwmcu_data *mcu_data)
{
	int i;
	int queue_id;

	for (i = 0; (i < 3) && mcu_data->pending_flush; i++) {
		D("%s: mcu_data->pending_flush = 0x%llx, i = %d\n", __func__,
		  mcu_data->pending_flush, i);

		queue_id = (mcu_data->pending_flush & 0xFFFFFFFF)
				? 0 : 1;
		if (cwmcu_batch_fifo_read(mcu_data, queue_id))
			break;

		if (mcu_data->pending_flush)
			usleep_range(6000, 9000);
		else
			break;
	}
	if (mcu_data->pending_flush && (i == 3))
		D("%s: Fail to get META!!\n", __func__);

}

/* cwmcu_powermode_switch() must be held by caller */
static void cwmcu_batch_read(struct cwmcu_data *mcu_data)
{
	int j;
	u32 *non_wake_batch_list = (u32 *)&mcu_data->batched_list;
	u32 *wake_batch_list = (non_wake_batch_list + 1);

	D("%s++: batched_list = 0x%llx\n", __func__, mcu_data->batched_list);

	for (j = 0; j < 2; j++) {
		if ((!(*non_wake_batch_list) && (j == 0)) ||
		    (!(*wake_batch_list) && (j == 1))) {
			D(
			  "%s++: nw_batched_list = 0x%x, w_batched_list = 0x%x,"
			  " j = %d, continue\n",
			   __func__, *non_wake_batch_list, *wake_batch_list, j);
			continue;
		}

		cwmcu_batch_fifo_read(mcu_data, j);
	}

	D("%s--: batched_list = 0x%llx\n", __func__, mcu_data->batched_list);
}

static void cwmcu_check_sensor_update(struct cwmcu_data *mcu_data)
{
	int id;
	s64 temp;

	do_gettimeofday(&mcu_data->now);
	temp = (mcu_data->now.tv_sec * NS_PER_US) + mcu_data->now.tv_usec;

	for (id = 0; id < CW_SENSORS_ID_TOTAL; id++) {
		mcu_data->time_diff[id] = temp - mcu_data->sensors_time[id];

		if ((mcu_data->time_diff[id] >= mcu_data->report_period[id])
		    && (mcu_data->enabled_list & (1LL << id))) {
			mcu_data->sensors_time[id] = temp;
			mcu_data->update_list |= (1LL << id);
		} else
			mcu_data->update_list &= ~(1LL << id);
	}
}

static void cwmcu_read(struct cwmcu_data *mcu_data, struct iio_poll_func *pf)
{
	int id_check;

	if (!mcu_data->probe_success) {
		E("%s: probe_success = %d\n", __func__,
		  mcu_data->probe_success);
		return;
	}

	if (mcu_data->enabled_list) {

		cwmcu_check_sensor_update(mcu_data);

		for (id_check = 0 ;
		     (id_check < CW_SENSORS_ID_TOTAL)
		     ; id_check++) {
			if ((is_continuous_sensor(id_check)) &&
			    (mcu_data->update_list & (1LL<<id_check)) &&
			    (mcu_data->batch_timeout[id_check] == 0)) {
				cwmcu_powermode_switch(mcu_data, 1);
				cwmcu_batch_fifo_read(mcu_data,
						id_check > CW_SENSORS_ID_FW);
				cwmcu_powermode_switch(mcu_data, 0);
			}
		}
	}

}

static int cwmcu_suspend(struct device *dev)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	int i;
	u8 data;

	D("[CWMCU] %s\n", __func__);

	cancel_work_sync(&mcu_data->one_shot_work);
	cancel_delayed_work_sync(&mcu_data->work);

	disable_irq(mcu_data->IRQ);

	/* Inform SensorHUB that CPU is going to suspend */
	data = 0;
	CWMCU_i2c_write_power(mcu_data, CW_CPU_STATUS_REG, &data, 1);
	D("%s: write_addr = 0x%x, write_data = 0x%x\n", __func__,
	  CW_CPU_STATUS_REG, data);

	mutex_lock(&mcu_data->mutex_lock);
	mcu_data->suspended = true;
	mutex_unlock(&mcu_data->mutex_lock);

	for (i = 0; (mcu_data->power_on_counter != 0) &&
		    (gpio_get_value(mcu_data->gpio_wake_mcu) != 1) &&
		    (i < ACTIVE_RETRY_TIMES); i++)
		usleep_range(10, 20);

	gpio_set_value(mcu_data->gpio_wake_mcu, 1);
	mcu_data->power_on_counter = 0;

	return 0;
}


static int cwmcu_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct cwmcu_data *mcu_data = i2c_get_clientdata(client);
	u8 data;

	D("[CWMCU] %s++\n", __func__);

	mutex_lock(&mcu_data->mutex_lock);
	mcu_data->suspended = false;
	mutex_unlock(&mcu_data->mutex_lock);

	/* Inform SensorHUB that CPU is going to resume */
	data = 1;
	CWMCU_i2c_write_power(mcu_data, CW_CPU_STATUS_REG, &data, 1);
	D("%s: write_addr = 0x%x, write_data = 0x%x\n", __func__,
	  CW_CPU_STATUS_REG, data);

	enable_irq(mcu_data->IRQ);

	if (mcu_data->w_activated_i2c
	    || mcu_data->w_re_init
	    || mcu_data->w_facedown_set
	    || mcu_data->w_clear_fifo
	    || mcu_data->w_flush_fifo
	    || mcu_data->w_report_meta)
		queue_work(mcu_data->mcu_wq, &mcu_data->one_shot_work);

	if (mcu_data->enabled_list & IIO_SENSORS_MASK) {
		queue_delayed_work(mcu_data->mcu_wq, &mcu_data->work,
			msecs_to_jiffies(atomic_read(&mcu_data->delay)));
	}

	D("[CWMCU] %s--\n", __func__);
	return 0;
}


#ifdef MCU_WARN_MSGS
static void print_warn_msg(struct cwmcu_data *mcu_data,
			   char *buf, u32 len, u32 index)
{
	int ret;
	char *buf_start = buf;

	while ((buf - buf_start) < len) {
		ret = min((u32)WARN_MSG_BLOCK_LEN,
			  (u32)(len - (buf - buf_start)));
		ret = CWMCU_i2c_read(mcu_data,
				     CW_I2C_REG_WARN_MSG_BUFFER,
				     buf, ret);
		if (ret == 0) {
			break;
		} else if (ret < 0) {
			E("%s: warn i2c_read: ret = %d\n", __func__, ret);
			break;
		} else
			buf += ret;
	}
	printk(KERN_WARNING "[S_HUB][CW_MCU] Warning MSG[%d] = %.*s",
			index, (int)(buf - buf_start), buf_start);
}
#endif

void magic_cover_report_input(struct cwmcu_data *mcu_data, u8 val)
{
	u32 data = ((val >> 6) & 0x3);

	if ((data == 1) || (data == 2)) {
		input_report_switch(mcu_data->input, SW_LID, (data - 1));
		input_sync(mcu_data->input);
	} else if ((data == 3) || (data == 0)) {
		input_report_switch(mcu_data->input, SW_CAMERA_LENS_COVER,
				    !data);
		input_sync(mcu_data->input);
	}
	return;
}

void activate_double_tap(u8 facedown)
{
	blocking_notifier_call_chain(&double_tap_notifier_list, facedown, NULL);
	return;
}

static irqreturn_t cwmcu_irq_handler(int irq, void *handle)
{
	struct cwmcu_data *mcu_data = handle;
	s32 ret;
	u8 INT_st1, INT_st2, INT_st3, INT_st4, err_st, batch_st;
	u8 clear_intr;
	u16 light_adc = 0;

	if (!mcu_data->probe_success) {
		D("%s: probe not completed\n", __func__);
		return IRQ_HANDLED;
	}

	D("[CWMCU] %s\n", __func__);

	cwmcu_powermode_switch(mcu_data, 1);

	CWMCU_i2c_read(mcu_data, CWSTM32_INT_ST1, &INT_st1, 1);
	CWMCU_i2c_read(mcu_data, CWSTM32_INT_ST2, &INT_st2, 1);
	CWMCU_i2c_read(mcu_data, CWSTM32_INT_ST3, &INT_st3, 1);
	CWMCU_i2c_read(mcu_data, CWSTM32_INT_ST4, &INT_st4, 1);
	CWMCU_i2c_read(mcu_data, CWSTM32_ERR_ST, &err_st, 1);
	CWMCU_i2c_read(mcu_data, CWSTM32_BATCH_MODE_COMMAND, &batch_st, 1);

	D(
	  "%s: INT_st(1, 2, 3, 4) = (0x%x, 0x%x, 0x%x, 0x%x), err_st = 0x%x"
	  ", batch_st = 0x%x\n",
	  __func__, INT_st1, INT_st2, INT_st3, INT_st4, err_st, batch_st);

	/* INT_st1: bit 3 */
	if (INT_st1 & CW_MCU_INT_BIT_LIGHT) {
		u8 data[REPORT_EVENT_COMMON_LEN] = {0};
		u16 data_buff[REPORT_EVENT_COMMON_LEN] = {0};

		if (mcu_data->enabled_list & (1LL << CW_LIGHT)) {
			CWMCU_i2c_read(mcu_data, CWSTM32_READ_Light, data, 3);
			if (data[0] < 11) {
				mcu_data->sensors_time[CW_LIGHT] =
					mcu_data->sensors_time[CW_LIGHT] -
					mcu_data->report_period[CW_LIGHT];
				light_adc = (data[2] << 8) | data[1];

				data_buff[0] = data[0];
				mcu_data->light_last_data[0] = data_buff[0];
				cw_send_event(mcu_data, CW_LIGHT, data_buff, 0);
				D(
				  "light interrupt occur value is %u, adc "
				  "is %x ls_calibration is %u\n",
					data[0], light_adc,
					mcu_data->ls_calibrated);
			} else {
				light_adc = (data[2] << 8) | data[1];
				D(
				  "light interrupt occur value is %u, adc is"
				  " %x ls_calibration is %u (message only)\n",
					data[0], light_adc,
					mcu_data->ls_calibrated);
			}
			I("intr light[%x]=%u\n", light_adc, data[0]);
		}
		if (data[0] < 11) {
			clear_intr = CW_MCU_INT_BIT_LIGHT;
			CWMCU_i2c_write(mcu_data, CWSTM32_INT_ST1, &clear_intr,
					1);
		}
	}

	/* INT_st2: bit 4 */
	if (INT_st2 & CW_MCU_INT_BIT_MAGIC_COVER) {
		if (mcu_data->enabled_list & (1LL << HTC_MAGIC_COVER)) {
			u8 data;

			ret = CWMCU_i2c_read(mcu_data, CWSTM32_READ_Hall_Sensor,
					     &data, 1);
			if (ret >= 0) {
				I("%s: MAGIC COVER = 0x%x\n", __func__,
				  ((data >> 6) & 0x3));
				magic_cover_report_input(mcu_data, data);
			} else {
				E("%s: MAGIC COVER read fails, ret = %d\n",
				  __func__, ret);
			}

			clear_intr = CW_MCU_INT_BIT_MAGIC_COVER;
			CWMCU_i2c_write(mcu_data, CWSTM32_INT_ST2, &clear_intr,
					1);
		}
	}

	/* INT_st3: bit 4 */
	if (INT_st3 & CW_MCU_INT_BIT_SIGNIFICANT_MOTION) {
		if (mcu_data->enabled_list & (1LL << CW_SIGNIFICANT_MOTION)) {
			u16 data_buff[REPORT_EVENT_COMMON_LEN] = {0};
			__le64 data64[2];
			u8 *data = (u8 *)data64;

			data = data + sizeof(__le64) - sizeof(u8);

			ret = CWMCU_i2c_read(mcu_data,
					     CWSTM32_READ_SIGNIFICANT_MOTION,
					     data, sizeof(u8) + sizeof(__le64));
			if (ret >= 0) {
				u64 timestamp_event;
				__le64 *le64_timestamp = data64 + 1;

				timestamp_event = le64_to_cpu(*le64_timestamp);

				mcu_data->sensors_time[CW_SIGNIFICANT_MOTION]
						= 0;

				wake_lock_timeout(
					&mcu_data->significant_wake_lock, HZ);

				data_buff[0] = 1;
				cw_send_event(mcu_data, CW_SIGNIFICANT_MOTION,
					      data_buff, timestamp_event);

				D("%s: Significant timestamp = %llu\n"
					  , __func__, timestamp_event);
			} else {
				E(
				  "Read CWSTM32_READ_SIGNIFICANT_MOTION fails,"
				  " ret = %d\n", ret);
			}
		}
		clear_intr = CW_MCU_INT_BIT_SIGNIFICANT_MOTION;
		CWMCU_i2c_write(mcu_data, CWSTM32_INT_ST3, &clear_intr, 1);
	}

	/* INT_st3: bit 5 */
	if (INT_st3 & CW_MCU_INT_BIT_STEP_DETECTOR) {
		if (mcu_data->enabled_list & ((1ULL << CW_STEP_DETECTOR) |
					      (1ULL << CW_STEP_DETECTOR_W)))
			cwmcu_batch_read(mcu_data);

		clear_intr = CW_MCU_INT_BIT_STEP_DETECTOR;
		CWMCU_i2c_write(mcu_data, CWSTM32_INT_ST3, &clear_intr, 1);
	}

	/* INT_st3: bit 6 */
	if (INT_st3 & CW_MCU_INT_BIT_STEP_COUNTER) {
		if (mcu_data->enabled_list & (1LL << CW_STEP_COUNTER_W))
			cwmcu_batch_read(mcu_data);

		if (mcu_data->enabled_list & (1LL << CW_STEP_COUNTER)) {
			__le64 data64[2];
			u8 *data = (u8 *)data64;
			__le32 step_fw;

			ret = CWMCU_i2c_read(mcu_data,
					     CWSTM32_READ_STEP_COUNTER,
					     data, 12);
			if (ret >= 0) {
				step_fw = *(__le32 *)(data + 8);
				D("%s: From Firmware, step = %u\n",
				  __func__, le32_to_cpu(step_fw));

				mcu_data->sensors_time[CW_STEP_COUNTER]
					= 0;

				report_step_counter(mcu_data,
						    le32_to_cpu(step_fw)
						    , le64_to_cpu(
							data64[0])
						    , false);

				D(
				  "%s: Step Counter INT, step = %llu"
				  ", timestamp = %llu\n"
				  , __func__
				  , mcu_data->step_counter_base
				    + le32_to_cpu(step_fw)
				  , le64_to_cpu(data64[0]));
			} else {
				E(
				  "%s: Step Counter i2c read fails, "
				  "ret = %d\n", __func__, ret);
			}
		}
		clear_intr = CW_MCU_INT_BIT_STEP_COUNTER;
		CWMCU_i2c_write(mcu_data, CWSTM32_INT_ST3, &clear_intr, 1);
	}

	/* INT_st3: bit 7 */
	if (INT_st3 & CW_MCU_INT_BIT_FACEDOWN_DETECTION) {
		if (mcu_data->enabled_list & (1LL << HTC_FACEDOWN_DETECTION)) {
			u8 data;

			ret = CWMCU_i2c_read(mcu_data,
					     CWSTM32_READ_FACEDOWN_DETECTION,
					     &data, sizeof(data));
			if (ret >= 0) {
				D("%s: FACEDOWN = %u\n", __func__, data);
				activate_double_tap(data);
			} else
				E("%s: FACEDOWN i2c read fails, ret = %d\n",
						__func__, ret);

		}
		clear_intr = CW_MCU_INT_BIT_FACEDOWN_DETECTION;
		CWMCU_i2c_write(mcu_data, CWSTM32_INT_ST3, &clear_intr, 1);
	}

#ifdef MCU_WARN_MSGS
	/* err_st: bit 5 */
	if (err_st & CW_MCU_INT_BIT_ERROR_WARN_MSG) {
		u8 buf_len[WARN_MSG_BUFFER_LEN_SIZE] = {0};

		ret = CWMCU_i2c_read(mcu_data, CW_I2C_REG_WARN_MSG_BUFFER_LEN,
				     buf_len, sizeof(buf_len));
		if (ret >= 0) {
			int i;
			char buf[WARN_MSG_PER_ITEM_LEN];

			for (i = 0; i < WARN_MSG_BUFFER_LEN_SIZE; i++) {
				if (buf_len[i] <= WARN_MSG_PER_ITEM_LEN)
					print_warn_msg(mcu_data, buf,
						       buf_len[i], i);
			}
		} else {
			E("%s: Warn MSG read fails, ret = %d\n",
						__func__, ret);
		}
		clear_intr = CW_MCU_INT_BIT_ERROR_WARN_MSG;
		ret = CWMCU_i2c_write(mcu_data, CWSTM32_ERR_ST, &clear_intr, 1);
	}
#endif

	/* err_st: bit 6 */
	if (err_st & CW_MCU_INT_BIT_ERROR_MCU_EXCEPTION) {
		u8 buf_len[EXCEPTION_BUFFER_LEN_SIZE] = {0};
		bool reset_done;

		ret = CWMCU_i2c_read(mcu_data, CW_I2C_REG_EXCEPTION_BUFFER_LEN,
				     buf_len, sizeof(buf_len));
		if (ret >= 0) {
			__le32 exception_len;
			u8 data[EXCEPTION_BLOCK_LEN];
			int i;

			exception_len = cpu_to_le32p((u32 *)&buf_len[0]);
			E("%s: exception_len = %u\n", __func__, exception_len);

			for (i = 0; exception_len >= EXCEPTION_BLOCK_LEN; i++) {
				memset(data, 0, sizeof(data));
				ret = CWMCU_i2c_read(mcu_data,
						    CW_I2C_REG_EXCEPTION_BUFFER,
						    data, sizeof(data));
				if (ret >= 0) {
					char buf[3*EXCEPTION_BLOCK_LEN];

					print_hex_data(buf, i, data,
							EXCEPTION_BLOCK_LEN);
					exception_len -= EXCEPTION_BLOCK_LEN;
				} else {
					E(
					  "%s: i = %d, excp1 i2c_read: ret = %d"
					  "\n", __func__, i, ret);
					goto exception_end;
				}
			}
			if ((exception_len > 0) &&
			    (exception_len < sizeof(data))) {
				ret = CWMCU_i2c_read(mcu_data,
						    CW_I2C_REG_EXCEPTION_BUFFER,
						    data, exception_len);
				if (ret >= 0) {
					char buf[3*EXCEPTION_BLOCK_LEN];

					print_hex_data(buf, i, data,
						       exception_len);
				} else {
					E(
					  "%s: i = %d, excp2 i2c_read: ret = %d"
					  "\n", __func__, i, ret);
				}
			}
		} else {
			E("%s: Exception status dump fails, ret = %d\n",
						__func__, ret);
		}
exception_end:
		mutex_lock(&mcu_data->activated_i2c_lock);
		reset_done = reset_hub(mcu_data);
		mutex_unlock(&mcu_data->activated_i2c_lock);

		if (reset_done) {
			mcu_data->w_re_init = true;
			mcu_data->w_report_meta = true;
			queue_work(mcu_data->mcu_wq, &mcu_data->one_shot_work);
			E("%s: reset after exception done\n", __func__);
		}

		clear_intr = CW_MCU_INT_BIT_ERROR_MCU_EXCEPTION;
		ret = CWMCU_i2c_write(mcu_data, CWSTM32_ERR_ST, &clear_intr, 1);
	}

	/* err_st: bit 7 */
	if (err_st & CW_MCU_INT_BIT_ERROR_WATCHDOG_RESET) {
		u8 data[WATCHDOG_STATUS_LEN] = {0};

		E("[CWMCU] Watch Dog Reset\n");
		msleep(5);

		ret = CWMCU_i2c_read(mcu_data, CW_I2C_REG_WATCHDOG_STATUS,
				     data, WATCHDOG_STATUS_LEN);
		if (ret >= 0) {
			int i;

			for (i = 0; i < WATCHDOG_STATUS_LEN; i++) {
				E("%s: Watchdog Status[%d] = 0x%x\n",
					__func__, i, data[i]);
			}
		} else {
			E("%s: Watchdog status dump fails, ret = %d\n",
						__func__, ret);
		}

		mcu_data->w_re_init = true;
		mcu_data->w_report_meta = true;
		queue_work(mcu_data->mcu_wq, &mcu_data->one_shot_work);

		clear_intr = CW_MCU_INT_BIT_ERROR_WATCHDOG_RESET;
		ret = CWMCU_i2c_write(mcu_data, CWSTM32_ERR_ST, &clear_intr, 1);
	}

	/* batch_st */
	if (batch_st & CW_MCU_INT_BIT_BATCH_INT_MASK) {
		cwmcu_batch_read(mcu_data);

		clear_intr = CW_MCU_INT_BIT_BATCH_INT_MASK;

		D("%s: clear_intr = 0x%x, write_addr = 0x%x", __func__,
		  clear_intr, CWSTM32_BATCH_MODE_COMMAND);

		ret = CWMCU_i2c_write(mcu_data,
				      CWSTM32_BATCH_MODE_COMMAND,
				      &clear_intr, 1);
	}

	cwmcu_powermode_switch(mcu_data, 0);

	return IRQ_HANDLED;
}

/*=======iio device reg=========*/
static void iio_trigger_work(struct irq_work *work)
{
	struct cwmcu_data *mcu_data = container_of((struct irq_work *)work,
					struct cwmcu_data, iio_irq_work);

	iio_trigger_poll(mcu_data->trig, iio_get_time_ns());
}

static irqreturn_t cw_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct cwmcu_data *mcu_data = iio_priv(indio_dev);

	cwmcu_read(mcu_data, pf);

	mutex_lock(&mcu_data->lock);
	iio_trigger_notify_done(mcu_data->indio_dev->trig);
	mutex_unlock(&mcu_data->lock);

	return IRQ_HANDLED;
}

static const struct iio_buffer_setup_ops cw_buffer_setup_ops = {
	.preenable = &iio_sw_buffer_preenable,
	.postenable = &iio_triggered_buffer_postenable,
	.predisable = &iio_triggered_buffer_predisable,
};

static int cw_pseudo_irq_enable(struct iio_dev *indio_dev)
{
	struct cwmcu_data *mcu_data = iio_priv(indio_dev);

	if (!atomic_cmpxchg(&mcu_data->pseudo_irq_enable, 0, 1)) {
		D("%s:\n", __func__);
		cancel_delayed_work_sync(&mcu_data->work);
		queue_delayed_work(mcu_data->mcu_wq, &mcu_data->work, 0);
	}

	return 0;
}

static int cw_pseudo_irq_disable(struct iio_dev *indio_dev)
{
	struct cwmcu_data *mcu_data = iio_priv(indio_dev);

	if (atomic_cmpxchg(&mcu_data->pseudo_irq_enable, 1, 0)) {
		cancel_delayed_work_sync(&mcu_data->work);
		D("%s:\n", __func__);
	}
	return 0;
}

static int cw_set_pseudo_irq(struct iio_dev *indio_dev, int enable)
{
	if (enable)
		cw_pseudo_irq_enable(indio_dev);
	else
		cw_pseudo_irq_disable(indio_dev);

	return 0;
}

static int cw_data_rdy_trigger_set_state(struct iio_trigger *trig,
		bool state)
{
	struct iio_dev *indio_dev =
			(struct iio_dev *)iio_trigger_get_drvdata(trig);
	struct cwmcu_data *mcu_data = iio_priv(indio_dev);

	mutex_lock(&mcu_data->mutex_lock);
	cw_set_pseudo_irq(indio_dev, state);
	mutex_unlock(&mcu_data->mutex_lock);

	return 0;
}

static const struct iio_trigger_ops cw_trigger_ops = {
	.owner = THIS_MODULE,
	.set_trigger_state = &cw_data_rdy_trigger_set_state,
};

static int cw_probe_trigger(struct iio_dev *iio_dev)
{
	struct cwmcu_data *mcu_data = iio_priv(iio_dev);
	int ret;

	iio_dev->pollfunc = iio_alloc_pollfunc(&iio_pollfunc_store_time,
			&cw_trigger_handler, IRQF_ONESHOT, iio_dev,
			"%s_consumer%d", iio_dev->name, iio_dev->id);
	if (iio_dev->pollfunc == NULL) {
		ret = -ENOMEM;
		goto error_ret;
	}
	mcu_data->trig = iio_trigger_alloc("%s-dev%d",
			iio_dev->name,
			iio_dev->id);
	if (!mcu_data->trig) {
		ret = -ENOMEM;
		goto error_dealloc_pollfunc;
	}
	mcu_data->trig->dev.parent = &mcu_data->client->dev;
	mcu_data->trig->ops = &cw_trigger_ops;
	iio_trigger_set_drvdata(mcu_data->trig, iio_dev);

	ret = iio_trigger_register(mcu_data->trig);
	if (ret)
		goto error_free_trig;

	return 0;

error_free_trig:
	iio_trigger_free(mcu_data->trig);
error_dealloc_pollfunc:
	iio_dealloc_pollfunc(iio_dev->pollfunc);
error_ret:
	return ret;
}

static int cw_probe_buffer(struct iio_dev *iio_dev)
{
	int ret;
	struct iio_buffer *buffer;

	buffer = iio_kfifo_allocate(iio_dev);
	if (!buffer) {
		ret = -ENOMEM;
		goto error_ret;
	}

	buffer->scan_timestamp = true;
	iio_dev->buffer = buffer;
	iio_dev->setup_ops = &cw_buffer_setup_ops;
	iio_dev->modes |= INDIO_BUFFER_TRIGGERED;
	ret = iio_buffer_register(iio_dev, iio_dev->channels,
				  iio_dev->num_channels);
	if (ret)
		goto error_free_buf;

	iio_scan_mask_set(iio_dev, iio_dev->buffer, CW_SCAN_ID);
	iio_scan_mask_set(iio_dev, iio_dev->buffer, CW_SCAN_X);
	iio_scan_mask_set(iio_dev, iio_dev->buffer, CW_SCAN_Y);
	iio_scan_mask_set(iio_dev, iio_dev->buffer, CW_SCAN_Z);
	return 0;

error_free_buf:
	iio_kfifo_free(iio_dev->buffer);
error_ret:
	return ret;
}


static int cw_read_raw(struct iio_dev *indio_dev,
		       struct iio_chan_spec const *chan,
		       int *val,
		       int *val2,
		       long mask) {
	struct cwmcu_data *mcu_data = iio_priv(indio_dev);
	int ret = -EINVAL;

	if (chan->type != IIO_ACCEL)
		return ret;

	mutex_lock(&mcu_data->lock);
	switch (mask) {
	case 0:
		*val = mcu_data->iio_data[chan->channel2 - IIO_MOD_X];
		ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_SCALE:
		/* Gain : counts / uT = 1000 [nT] */
		/* Scaling factor : 1000000 / Gain = 1000 */
		*val = 0;
		*val2 = 1000;
		ret = IIO_VAL_INT_PLUS_MICRO;
		break;
	}
	mutex_unlock(&mcu_data->lock);

	return ret;
}

#define CW_CHANNEL(axis)			\
{						\
	.type = IIO_ACCEL,			\
	.modified = 1,				\
	.channel2 = axis+1,			\
	.info_mask = BIT(IIO_CHAN_INFO_SCALE),	\
	.scan_index = axis,			\
	.scan_type = IIO_ST('u', 32, 32, 0)	\
}

static const struct iio_chan_spec cw_channels[] = {
	CW_CHANNEL(CW_SCAN_ID),
	CW_CHANNEL(CW_SCAN_X),
	CW_CHANNEL(CW_SCAN_Y),
	CW_CHANNEL(CW_SCAN_Z),
	IIO_CHAN_SOFT_TIMESTAMP(CW_SCAN_TIMESTAMP)
};

static const struct iio_info cw_info = {
	.read_raw = &cw_read_raw,
	.driver_module = THIS_MODULE,
};

static int mcu_parse_dt(struct device *dev, struct cwmcu_data *pdata)
{
	struct property *prop;
	struct device_node *dt = dev->of_node;
	u32 buf = 0;
	struct device_node *g_sensor_offset;
	int g_sensor_cali_size = 0;
	unsigned char *g_sensor_cali_data = NULL;
	struct device_node *gyro_sensor_offset;
	int gyro_sensor_cali_size = 0;
	unsigned char *gyro_sensor_cali_data = NULL;
	struct device_node *light_sensor_offset = NULL;
	int light_sensor_cali_size = 0;
	unsigned char *light_sensor_cali_data = NULL;
	struct device_node *baro_sensor_offset;
	int baro_sensor_cali_size = 0;
	unsigned char *baro_sensor_cali_data = NULL;

	int i;

	g_sensor_offset = of_find_node_by_path(CALIBRATION_DATA_PATH);
	if (g_sensor_offset) {
		g_sensor_cali_data = (unsigned char *)
				     of_get_property(g_sensor_offset,
						     G_SENSOR_FLASH_DATA,
						     &g_sensor_cali_size);
		D("%s: cali_size = %d\n", __func__, g_sensor_cali_size);
		if (g_sensor_cali_data) {
			for (i = 0; (i < g_sensor_cali_size) && (i < 4); i++) {
				D("g sensor cali_data[%d] = %02x\n", i,
						g_sensor_cali_data[i]);
				pdata->gs_kvalue |= (g_sensor_cali_data[i] <<
						    (i * 8));
			}
		}

	} else
		E("%s: G-sensor Calibration data offset not found\n", __func__);

	gyro_sensor_offset = of_find_node_by_path(CALIBRATION_DATA_PATH);
	if (gyro_sensor_offset) {
		gyro_sensor_cali_data = (unsigned char *)
					of_get_property(gyro_sensor_offset,
							GYRO_SENSOR_FLASH_DATA,
							&gyro_sensor_cali_size);
		D("%s:gyro cali_size = %d\n", __func__, gyro_sensor_cali_size);
		if (gyro_sensor_cali_data) {
			for (i = 0; (i < gyro_sensor_cali_size) && (i < 4);
				     i++) {
				D("gyro sensor cali_data[%d] = %02x\n", i,
					gyro_sensor_cali_data[i]);
				pdata->gy_kvalue |= (gyro_sensor_cali_data[i] <<
						     (i * 8));
			}
			pdata->gs_kvalue_L1 = (gyro_sensor_cali_data[5] << 8) |
						gyro_sensor_cali_data[4];
			D("g sensor cali_data L1 = %x\n", pdata->gs_kvalue_L1);
			pdata->gs_kvalue_L2 = (gyro_sensor_cali_data[7] << 8) |
						gyro_sensor_cali_data[6];
			D("g sensor cali_data L2 = %x\n", pdata->gs_kvalue_L2);
			pdata->gs_kvalue_L3 = (gyro_sensor_cali_data[9] << 8) |
						gyro_sensor_cali_data[8];
			D("g sensor cali_data L3 = %x\n", pdata->gs_kvalue_L3);
			pdata->gs_kvalue_R1 = (gyro_sensor_cali_data[11] << 8) |
						gyro_sensor_cali_data[10];
			D("g sensor cali_data R1 = %x\n", pdata->gs_kvalue_R1);
			pdata->gs_kvalue_R2 = (gyro_sensor_cali_data[13] << 8) |
						gyro_sensor_cali_data[12];
			D("g sensor cali_data R2 = %x\n", pdata->gs_kvalue_R2);
			pdata->gs_kvalue_R3 = (gyro_sensor_cali_data[15] << 8) |
						gyro_sensor_cali_data[14];
			D("g sensor cali_data R3 = %x\n", pdata->gs_kvalue_R3);
		}

	} else
		E("%s: GYRO-sensor Calibration data offset not found\n",
				__func__);

	light_sensor_offset = of_find_node_by_path(CALIBRATION_DATA_PATH);
	if (light_sensor_offset) {
		light_sensor_cali_data = (unsigned char *)
					 of_get_property(light_sensor_offset,
						       LIGHT_SENSOR_FLASH_DATA,
						       &light_sensor_cali_size);
		D("%s:light cali_size = %d\n", __func__,
				light_sensor_cali_size);
		if (light_sensor_cali_data) {
			for (i = 0; (i < light_sensor_cali_size) && (i < 4);
			     i++) {
				D("light sensor cali_data[%d] = %02x\n", i,
					light_sensor_cali_data[i]);
				pdata->als_kvalue |=
					(light_sensor_cali_data[i] << (i * 8));
			}
		}
	} else
		E("%s: LIGHT-sensor Calibration data offset not found\n",
			__func__);

	baro_sensor_offset = of_find_node_by_path(CALIBRATION_DATA_PATH);
	if (baro_sensor_offset) {
		baro_sensor_cali_data = (unsigned char *)
					of_get_property(baro_sensor_offset,
							BARO_SENSOR_FLASH_DATA,
							&baro_sensor_cali_size);
		D("%s: cali_size = %d\n", __func__, baro_sensor_cali_size);
		if (baro_sensor_cali_data) {
			for (i = 0; (i < baro_sensor_cali_size) &&
			     (i < 5); i++) {
				D("baro sensor cali_data[%d] = %02x\n", i,
					baro_sensor_cali_data[i]);
				if (i == baro_sensor_cali_size - 1)
					pdata->bs_kheader =
						baro_sensor_cali_data[i];
				else
					pdata->bs_kvalue |=
						(baro_sensor_cali_data[i] <<
						(i * 8));
			}
		}
	} else
		E("%s: Barometer-sensor Calibration data offset not found\n",
			__func__);

	pdata->gpio_wake_mcu = of_get_named_gpio(dt, "mcu,Cpu_wake_mcu-gpio",
					0);
	if (!gpio_is_valid(pdata->gpio_wake_mcu))
		E("DT:gpio_wake_mcu value is not valid\n");
	else
		D("DT:gpio_wake_mcu=%d\n", pdata->gpio_wake_mcu);

	pdata->gpio_mcu_irq = of_get_named_gpio(dt, "mcu,intr-gpio", 0);
	if (!gpio_is_valid(pdata->gpio_mcu_irq))
		E("DT:gpio_mcu_irq value is not valid\n");
	else
		D("DT:gpio_mcu_irq=%d\n", pdata->gpio_mcu_irq);

	pdata->gpio_reset = of_get_named_gpio(dt, "mcu,Reset-gpio", 0);
	if (!gpio_is_valid(pdata->gpio_reset))
		E("DT:gpio_reset value is not valid\n");
	else
		D("DT:gpio_reset=%d\n", pdata->gpio_reset);

	pdata->gpio_chip_mode = of_get_named_gpio(dt, "mcu,Chip_mode-gpio", 0);
	if (!gpio_is_valid(pdata->gpio_chip_mode))
		E("DT:gpio_chip_mode value is not valid\n");
	else
		D("DT:gpio_chip_mode=%d\n", pdata->gpio_chip_mode);

	prop = of_find_property(dt, "mcu,gs_chip_layout", NULL);
	if (prop) {
		of_property_read_u32(dt, "mcu,gs_chip_layout", &buf);
		pdata->gs_chip_layout = buf;
		D("%s: chip_layout = %d\n", __func__, pdata->gs_chip_layout);
	} else
		E("%s: g_sensor,chip_layout not found", __func__);

	prop = of_find_property(dt, "mcu,acceleration_axes", NULL);
	if (prop) {
		of_property_read_u32(dt, "mcu,acceleration_axes", &buf);
		pdata->acceleration_axes = buf;
		I("%s: acceleration axes = %u\n", __func__,
			pdata->acceleration_axes);
	} else
		E("%s: g_sensor axes not found", __func__);

	prop = of_find_property(dt, "mcu,magnetic_axes", NULL);
	if (prop) {
		of_property_read_u32(dt, "mcu,magnetic_axes", &buf);
		pdata->magnetic_axes = buf;
		I("%s: Compass axes = %u\n", __func__, pdata->magnetic_axes);
	} else
		E("%s: Compass axes not found", __func__);

	prop = of_find_property(dt, "mcu,gyro_axes", NULL);
	if (prop) {
		of_property_read_u32(dt, "mcu,gyro_axes", &buf);
		pdata->gyro_axes = buf;
		I("%s: gyro axes = %u\n", __func__, pdata->gyro_axes);
	} else
		E("%s: gyro axes not found", __func__);

	return 0;
}

static struct device_attribute attributes[] = {

	__ATTR(enable, 0666, active_show,
			active_set),
	__ATTR(batch_enable, 0666, batch_show, batch_set),
	__ATTR(delay_ms, 0666, interval_show,
			interval_set),
	__ATTR(flush, 0666, flush_show, flush_set),
	__ATTR(calibrator_en, 0220, NULL, set_calibrator_en),
	__ATTR(calibrator_status_acc, 0440, show_calibrator_status_acc, NULL),
	__ATTR(calibrator_status_mag, 0440, show_calibrator_status_mag, NULL),
	__ATTR(calibrator_status_gyro, 0440, show_calibrator_status_gyro, NULL),
	__ATTR(calibrator_data_acc, 0666, get_k_value_acc_f, set_k_value_acc_f),
	__ATTR(calibrator_data_acc_rl, 0440, get_k_value_acc_rl_f, NULL),
	__ATTR(ap_calibrator_data_acc_rl, 0440, ap_get_k_value_acc_rl_f, NULL),
	__ATTR(calibrator_data_mag, 0666, get_k_value_mag_f, set_k_value_mag_f),
	__ATTR(calibrator_data_gyro, 0666, get_k_value_gyro_f,
			set_k_value_gyro_f),
	__ATTR(calibrator_data_light, 0440, get_k_value_light_f, NULL),
	__ATTR(calibrator_data_barometer, 0666, get_k_value_barometer_f,
			set_k_value_barometer_f),
	__ATTR(data_barometer, 0440, get_barometer, NULL),
	__ATTR(data_light_polling, 0440, get_light_polling, NULL),
	__ATTR(sensor_hub_rdata, 0220, NULL, read_mcu_data),
	__ATTR(data_light_kadc, 0440, get_light_kadc, NULL),
	__ATTR(firmware_version, 0440, get_firmware_version, NULL),
	__ATTR(hall_sensor, 0440, get_hall_sensor, NULL),
	__ATTR(led_en, 0220, NULL, led_enable),
	__ATTR(facedown_enabled, 0660, facedown_show, facedown_set),
};


static int create_sysfs_interfaces(struct cwmcu_data *mcu_data)
{
	int i;
	int res;

	mcu_data->sensor_class = class_create(THIS_MODULE, "htc_sensorhub");
	if (IS_ERR(mcu_data->sensor_class))
		return PTR_ERR(mcu_data->sensor_class);

	mcu_data->sensor_dev = device_create(mcu_data->sensor_class, NULL, 0,
					     "%s", "sensor_hub");
	if (IS_ERR(mcu_data->sensor_dev)) {
		res = PTR_ERR(mcu_data->sensor_dev);
		goto err_device_create;
	}

	res = dev_set_drvdata(mcu_data->sensor_dev, mcu_data);
	if (res)
		goto err_set_drvdata;

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(mcu_data->sensor_dev, attributes + i))
			goto error;

	res = sysfs_create_link(&mcu_data->sensor_dev->kobj,
				&mcu_data->indio_dev->dev.kobj, "iio");
	if (res < 0)
		goto error;

	return 0;

error:
	while (--i >= 0)
		device_remove_file(mcu_data->sensor_dev, attributes + i);
err_set_drvdata:
	put_device(mcu_data->sensor_dev);
	device_unregister(mcu_data->sensor_dev);
err_device_create:
	class_destroy(mcu_data->sensor_class);
	return res;
}

static void destroy_sysfs_interfaces(struct cwmcu_data *mcu_data)
{
	int i;

	sysfs_remove_link(&mcu_data->sensor_dev->kobj, "iio");
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(mcu_data->sensor_dev, attributes + i);
	put_device(mcu_data->sensor_dev);
	device_unregister(mcu_data->sensor_dev);
	class_destroy(mcu_data->sensor_class);
}

static void cwmcu_remove_trigger(struct iio_dev *indio_dev)
{
	struct cwmcu_data *mcu_data = iio_priv(indio_dev);

	iio_trigger_unregister(mcu_data->trig);
	iio_trigger_free(mcu_data->trig);
	iio_dealloc_pollfunc(indio_dev->pollfunc);
}
static void cwmcu_remove_buffer(struct iio_dev *indio_dev)
{
	iio_buffer_unregister(indio_dev);
	iio_kfifo_free(indio_dev->buffer);
}

static void cwmcu_one_shot(struct work_struct *work)
{
	struct cwmcu_data *mcu_data = container_of((struct work_struct *)work,
			struct cwmcu_data, one_shot_work);

	if (mcu_data->w_activated_i2c == true) {
		mcu_data->w_activated_i2c = false;

		mutex_lock(&mcu_data->activated_i2c_lock);
		if (retry_exhausted(mcu_data) &&
		    time_after(jiffies, mcu_data->i2c_jiffies +
					REACTIVATE_PERIOD)) {
			bool reset_done;

			reset_done = reset_hub(mcu_data);

			I("%s: fw_update_status = 0x%x\n", __func__,
			  mcu_data->fw_update_status);

			if (reset_done &&
			    (!(mcu_data->fw_update_status &
			    (FW_DOES_NOT_EXIST | FW_UPDATE_QUEUED)))) {
				mcu_data->fw_update_status |= FW_UPDATE_QUEUED;
				request_firmware_nowait(THIS_MODULE,
					FW_ACTION_HOTPLUG,
					"sensor_hub.img",
					&mcu_data->client->dev,
					GFP_KERNEL, mcu_data, update_firmware);
			}

		}

		if (retry_exhausted(mcu_data)) {
			D("%s: i2c_total_retry = %d, i2c_latch_retry = %d\n",
			  __func__, mcu_data->i2c_total_retry,
			  mcu_data->i2c_latch_retry);
			mutex_unlock(&mcu_data->activated_i2c_lock);
			return;
		}

		/* record the failure */
		mcu_data->i2c_total_retry++;
		mcu_data->i2c_jiffies = jiffies;

		mutex_unlock(&mcu_data->activated_i2c_lock);
		D(
		  "%s--: mcu_data->i2c_total_retry = %d, "
		  "mcu_data->i2c_latch_retry = %d\n", __func__,
		  mcu_data->i2c_total_retry, mcu_data->i2c_latch_retry);
	}

	if (mcu_data->w_re_init == true) {
		mcu_data->w_re_init = false;

		cwmcu_powermode_switch(mcu_data, 1);

		cwmcu_sensor_placement(mcu_data);
		cwmcu_set_sensor_kvalue(mcu_data);
		cwmcu_restore_status(mcu_data);

		cwmcu_powermode_switch(mcu_data, 0);
	}

	if (mcu_data->w_facedown_set == true) {
		u8 data;
		int i;

		mcu_data->w_facedown_set = false;

		i = (HTC_FACEDOWN_DETECTION / 8);

		data = (u8)(mcu_data->enabled_list >> (i * 8));
		CWMCU_i2c_write_power(mcu_data, CWSTM32_ENABLE_REG + i, &data,
				      1);
	}

	if (mcu_data->w_flush_fifo == true) {
		int j;
		bool is_meta_read = false;

		mcu_data->w_flush_fifo = false;

		cwmcu_powermode_switch(mcu_data, 1);

		for (j = 0; j < 2; j++) {
			is_meta_read = cwmcu_batch_fifo_read(mcu_data, j);
			if (is_meta_read)
				break;
		}

		if (!is_meta_read)
			cwmcu_meta_read(mcu_data);

		cwmcu_powermode_switch(mcu_data, 0);

		if (mcu_data->pending_flush && !mcu_data->w_flush_fifo) {
			mcu_data->w_flush_fifo = true;
			queue_work(mcu_data->mcu_wq, &mcu_data->one_shot_work);
		}
	}

	mutex_lock(&mcu_data->mutex_lock);
	if (mcu_data->w_clear_fifo == true) {
		int j;

		mcu_data->w_clear_fifo = false;
		mutex_unlock(&mcu_data->mutex_lock);

		cwmcu_powermode_switch(mcu_data, 1);

		for (j = 0; j < 2; j++)
			cwmcu_batch_fifo_read(mcu_data, j);

		cwmcu_powermode_switch(mcu_data, 0);

		mutex_lock(&mcu_data->mutex_lock);
		if (!mcu_data->w_clear_fifo)
			mcu_data->w_clear_fifo_running = false;
	} else
		mcu_data->w_clear_fifo_running = false;
	mutex_unlock(&mcu_data->mutex_lock);

	if (mcu_data->w_report_meta == true) {
		int j;
		u16 data_event[REPORT_EVENT_COMMON_LEN] = {0};

		mcu_data->w_report_meta = false;

		for (j = 0;
		     (j < CW_SENSORS_ID_TOTAL) && mcu_data->pending_flush;
		     j++) {
			if (mcu_data->enabled_list & (1LL << j)) {
				data_event[0] = j;
				cw_send_event(mcu_data, CW_META_DATA,
					      data_event, 0);
				I("%s: Reported META = %d from driver\n",
				  __func__, j);
			}
			mcu_data->pending_flush &= ~(1LL << j);
		}
	}
}


static void cwmcu_work_report(struct work_struct *work)
{
	struct cwmcu_data *mcu_data = container_of((struct delayed_work *)work,
			struct cwmcu_data, work);

	if (atomic_read(&mcu_data->pseudo_irq_enable)) {
		unsigned long jiff;

		jiff = msecs_to_jiffies(atomic_read(&mcu_data->delay));
		if (!jiff)
			jiff = 1;
		D("%s: jiff = %lu\n", __func__, jiff);
		irq_work_queue(&mcu_data->iio_irq_work);
		queue_delayed_work(mcu_data->mcu_wq, &mcu_data->work, jiff);
	}
}

static int cwmcu_input_init(struct input_dev **input)
{
	int err;

	*input = input_allocate_device();
	if (!*input)
		return -ENOMEM;

	set_bit(EV_SW, (*input)->evbit);

	input_set_capability(*input, EV_SW, SW_LID);
	input_set_capability(*input, EV_SW, SW_CAMERA_LENS_COVER);

	(*input)->name = CWMCU_I2C_NAME;

	err = input_register_device(*input);
	if (err) {
		input_free_device(*input);
		return err;
	}

	return err;
}

static int CWMCU_i2c_probe(struct i2c_client *client,
				       const struct i2c_device_id *id)
{
	struct cwmcu_data *mcu_data;
	struct iio_dev *indio_dev;
	int error;
	int i;

	I("%s++: Report pending META when FW exceptions\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "i2c_check_functionality error\n");
		return -EIO;
	}

	D("%s: sizeof(*mcu_data) = %lu\n", __func__, sizeof(*mcu_data));

	indio_dev = iio_device_alloc(sizeof(*mcu_data));
	if (!indio_dev) {
		I("%s: iio_device_alloc failed\n", __func__);
		return -ENOMEM;
	}

	i2c_set_clientdata(client, indio_dev);

	indio_dev->name = CWMCU_I2C_NAME;
	indio_dev->dev.parent = &client->dev;
	indio_dev->info = &cw_info;
	indio_dev->channels = cw_channels;
	indio_dev->num_channels = ARRAY_SIZE(cw_channels);
	indio_dev->modes |= INDIO_BUFFER_TRIGGERED | INDIO_KFIFO_USE_VMALLOC;

	mcu_data = iio_priv(indio_dev);
	mcu_data->client = client;
	mcu_data->indio_dev = indio_dev;

	if (client->dev.of_node) {
		D("Device Tree parsing.");

		error = mcu_parse_dt(&client->dev, mcu_data);
		if (error) {
			dev_err(&client->dev,
				"%s: mcu_parse_dt for pdata failed. err = %d\n"
					, __func__, error);
			goto exit_mcu_parse_dt_fail;
		}
	} else {
		if (client->dev.platform_data != NULL) {
			mcu_data->acceleration_axes =
				((struct cwmcu_platform_data *)
				 mcu_data->client->dev.platform_data)
				->acceleration_axes;
			mcu_data->magnetic_axes =
				((struct cwmcu_platform_data *)
				 mcu_data->client->dev.platform_data)
				->magnetic_axes;
			mcu_data->gyro_axes =
				((struct cwmcu_platform_data *)
				 mcu_data->client->dev.platform_data)
				->gyro_axes;
			mcu_data->gpio_wake_mcu =
				((struct cwmcu_platform_data *)
				 mcu_data->client->dev.platform_data)
				->gpio_wake_mcu;
		}
	}

	error = gpio_request(mcu_data->gpio_reset, "cwmcu_reset");
	if (error)
		E("%s : request reset gpio fail\n", __func__);

	error = gpio_request(mcu_data->gpio_wake_mcu, "cwmcu_CPU2MCU");
	if (error)
		E("%s : request gpio_wake_mcu gpio fail\n", __func__);

	error = gpio_request(mcu_data->gpio_chip_mode, "cwmcu_hub_boot_mode");
	if (error)
		E("%s : request ghip mode gpio fail\n", __func__);

	gpio_direction_output(mcu_data->gpio_reset, 1);
	gpio_direction_output(mcu_data->gpio_wake_mcu, 0);
	gpio_direction_output(mcu_data->gpio_chip_mode, 0);

	error = gpio_request(mcu_data->gpio_mcu_irq, "cwmcu_int");
	if (error) {
		E("%s : request irq gpio fail\n", __func__);
	}

	mutex_init(&mcu_data->mutex_lock);
	mutex_init(&mcu_data->group_i2c_lock);
	mutex_init(&mcu_data->activated_i2c_lock);
	mutex_init(&mcu_data->power_mode_lock);
	mutex_init(&mcu_data->lock);

	INIT_DELAYED_WORK(&mcu_data->work, cwmcu_work_report);
	INIT_WORK(&mcu_data->one_shot_work, cwmcu_one_shot);

	error = cw_probe_buffer(indio_dev);
	if (error) {
		E("%s: iio yas_probe_buffer failed\n", __func__);
		goto error_free_dev;
	}
	error = cw_probe_trigger(indio_dev);
	if (error) {
		E("%s: iio yas_probe_trigger failed\n", __func__);
		goto error_remove_buffer;
	}
	error = iio_device_register(indio_dev);
	if (error) {
		E("%s: iio iio_device_register failed\n", __func__);
		goto error_remove_trigger;
	}

	error = create_sysfs_interfaces(mcu_data);
	if (error)
		goto err_free_mem;

	for (i = 0; i < num_sensors; i++) {
		mcu_data->sensors_time[i] = 0;
		mcu_data->report_period[i] = 200000 * MS_TO_PERIOD;
	}

	wake_lock_init(&mcu_data->significant_wake_lock, WAKE_LOCK_SUSPEND,
		       "significant_wake_lock");
	wake_lock_init(&mcu_data->report_wake_lock, WAKE_LOCK_SUSPEND,
		       "report_wake_lock");

	atomic_set(&mcu_data->delay, CWMCU_MAX_DELAY);
	init_irq_work(&mcu_data->iio_irq_work, iio_trigger_work);

	mcu_data->mcu_wq = create_singlethread_workqueue("htc_mcu");
	i2c_set_clientdata(client, mcu_data);
	pm_runtime_enable(&client->dev);

	client->irq = gpio_to_irq(mcu_data->gpio_mcu_irq);

	mcu_data->IRQ = client->irq;
	D("Requesting irq = %d\n", mcu_data->IRQ);
	error = request_threaded_irq(mcu_data->IRQ, NULL, cwmcu_irq_handler,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT, "cwmcu", mcu_data);
	if (error)
		E("[CWMCU] could not request irq %d\n", error);
	error = enable_irq_wake(mcu_data->IRQ);
	if (error < 0)
		E("[CWMCU] could not enable irq as wakeup source %d\n", error);

	mutex_lock(&mcu_data->mutex_lock);
	mcu_data->suspended = false;
	mutex_unlock(&mcu_data->mutex_lock);

	request_firmware_nowait(THIS_MODULE, FW_ACTION_HOTPLUG,
			"sensor_hub.img", &client->dev, GFP_KERNEL, mcu_data,
			update_firmware);

	error = cwmcu_input_init(&mcu_data->input);
	if (error) {
		E("%s: input_dev register failed", __func__);
		goto err_register_input;
	}
	input_set_drvdata(mcu_data->input, mcu_data);

	mcu_data->probe_success = true;
	I("CWMCU_i2c_probe success!\n");

	return 0;

err_register_input:
	free_irq(mcu_data->IRQ, mcu_data);
err_free_mem:
	if (indio_dev)
		iio_device_unregister(indio_dev);
error_remove_trigger:
	if (indio_dev)
		cwmcu_remove_trigger(indio_dev);
error_remove_buffer:
	if (indio_dev)
		cwmcu_remove_buffer(indio_dev);
error_free_dev:
	if (client->dev.of_node &&
	    ((struct cwmcu_platform_data *)mcu_data->client->dev.platform_data))
		kfree(mcu_data->client->dev.platform_data);
exit_mcu_parse_dt_fail:
	if (indio_dev)
		iio_device_free(indio_dev);
	i2c_set_clientdata(client, NULL);

	return error;
}


static int CWMCU_i2c_remove(struct i2c_client *client)
{
	struct cwmcu_data *mcu_data = i2c_get_clientdata(client);

	gpio_set_value(mcu_data->gpio_wake_mcu, 1);

	wake_lock_destroy(&mcu_data->significant_wake_lock);
	wake_lock_destroy(&mcu_data->report_wake_lock);
	destroy_sysfs_interfaces(mcu_data);
	kfree(mcu_data);
	return 0;
}

static const struct dev_pm_ops cwmcu_pm_ops = {
	.suspend = cwmcu_suspend,
	.resume = cwmcu_resume
};


static const struct i2c_device_id cwmcu_id[] = {
	{CWMCU_I2C_NAME, 0},
	{ }
};
#ifdef CONFIG_OF
static struct of_device_id mcu_match_table[] = {
	{.compatible = "htc_mcu" },
	{},
};
#else
#define mcu_match_table NULL
#endif

MODULE_DEVICE_TABLE(i2c, cwmcu_id);

static struct i2c_driver cwmcu_driver = {
	.driver = {
		.name = CWMCU_I2C_NAME,
		   .owner = THIS_MODULE,
		.pm = &cwmcu_pm_ops,
		.of_match_table = mcu_match_table,
	},
	.probe    = CWMCU_i2c_probe,
	.remove   = CWMCU_i2c_remove,
	.id_table = cwmcu_id,
};

static int __init CWMCU_i2c_init(void)
{
	return i2c_add_driver(&cwmcu_driver);
}
module_init(CWMCU_i2c_init);

static void __exit CWMCU_i2c_exit(void)
{
	i2c_del_driver(&cwmcu_driver);
}
module_exit(CWMCU_i2c_exit);

MODULE_DESCRIPTION("CWMCU I2C Bus Driver V1.6");
MODULE_AUTHOR("CyWee Group Ltd.");
MODULE_LICENSE("GPL");
