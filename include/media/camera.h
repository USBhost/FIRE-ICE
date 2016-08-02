/*
 * Copyright (c) 2013-2014, NVIDIA CORPORATION.  All rights reserved.

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

#ifndef __CAMERA_H__
#define __CAMERA_H__

#ifdef __KERNEL__
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/sysedp.h>
#include <media/nvc.h>
#endif

#define CAMERA_INT_MASK			0xf0000000
#define CAMERA_TABLE_WAIT_US		(CAMERA_INT_MASK | 1)
#define CAMERA_TABLE_WAIT_MS		(CAMERA_INT_MASK | 2)
#define CAMERA_TABLE_END		(CAMERA_INT_MASK | 9)
#define CAMERA_TABLE_PWR		(CAMERA_INT_MASK | 20)
#define CAMERA_TABLE_PINMUX		(CAMERA_INT_MASK | 25)
#define CAMERA_TABLE_INX_PINMUX		(CAMERA_INT_MASK | 26)
#define CAMERA_TABLE_GPIO_ACT		(CAMERA_INT_MASK | 30)
#define CAMERA_TABLE_GPIO_DEACT		(CAMERA_INT_MASK | 31)
#define CAMERA_TABLE_GPIO_INX_ACT	(CAMERA_INT_MASK | 32)
#define CAMERA_TABLE_GPIO_INX_DEACT	(CAMERA_INT_MASK | 33)
#define CAMERA_TABLE_REG_NEW_POWER	(CAMERA_INT_MASK | 40)
#define CAMERA_TABLE_INX_POWER		(CAMERA_INT_MASK | 41)
#define CAMERA_TABLE_INX_CLOCK		(CAMERA_INT_MASK | 50)
#define CAMERA_TABLE_INX_CGATE		(CAMERA_INT_MASK | 51)
#define CAMERA_TABLE_EDP_STATE		(CAMERA_INT_MASK | 60)

#define CAMERA_TABLE_DEV_READ		0xe0000000

#define CAMERA_TABLE_PWR_FLAG_MASK	0xf0000000
#define CAMERA_TABLE_PWR_FLAG_ON	0x80000000
#define CAMERA_TABLE_PINMUX_FLAG_MASK	0xf0000000
#define CAMERA_TABLE_PINMUX_FLAG_ON	0x80000000
#define CAMERA_TABLE_CLOCK_VALUE_BITS	24
#define CAMERA_TABLE_CLOCK_VALUE_MASK	\
			((u32)(-1) >> (32 - CAMERA_TABLE_CLOCK_VALUE_BITS))
#define CAMERA_TABLE_CLOCK_INDEX_BITS	(32 - CAMERA_TABLE_CLOCK_VALUE_BITS)
#define CAMERA_TABLE_CLOCK_INDEX_MASK	\
			((u32)(-1) << (32 - CAMERA_TABLE_CLOCK_INDEX_BITS))

#define PCLLK_IOCTL_CHIP_REG	_IOW('o', 100, struct virtual_device)
#define PCLLK_IOCTL_DEV_REG	_IOW('o', 104, struct camera_device_info)
#define PCLLK_IOCTL_DEV_DEL	_IOW('o', 105, int)
#define PCLLK_IOCTL_DEV_FREE	_IOW('o', 106, int)
#define PCLLK_IOCTL_PWR_WR	_IOW('o', 108, int)
#define PCLLK_IOCTL_PWR_RD	_IOR('o', 109, int)
#define PCLLK_IOCTL_SEQ_WR	_IOWR('o', 112, struct nvc_param)
#define PCLLK_IOCTL_SEQ_RD	_IOWR('o', 113, struct nvc_param)
#define PCLLK_IOCTL_UPDATE	_IOW('o', 116, struct nvc_param)
#define PCLLK_IOCTL_LAYOUT_WR	_IOW('o', 120, struct nvc_param)
#define PCLLK_IOCTL_LAYOUT_RD	_IOWR('o', 121, struct nvc_param)
#define PCLLK_IOCTL_PARAM_WR	_IOWR('o', 140, struct nvc_param)
#define PCLLK_IOCTL_PARAM_RD	_IOWR('o', 141, struct nvc_param)
#define PCLLK_IOCTL_DRV_ADD	_IOW('o', 150, struct nvc_param)
#define PCLLK_IOCTL_DT_GET	_IOWR('o', 160, struct nvc_param)
#define PCLLK_IOCTL_MSG		_IOWR('o', 170, struct nvc_param)

#ifdef CONFIG_COMPAT
/* IOCTL commands that pass 32 bit pointers from user space.
   CAUTION: the nr number of these commands MUST be the same value as the
   nr number of the related normal commands. */
#define PCLLK_IOCTL_32_CHIP_REG	_IOW('o', 100, struct virtual_device_32)
#define PCLLK_IOCTL_32_SEQ_WR	_IOWR('o', 112, struct nvc_param_32)
#define PCLLK_IOCTL_32_SEQ_RD	_IOWR('o', 113, struct nvc_param_32)
#define PCLLK_IOCTL_32_UPDATE	_IOW('o', 116, struct nvc_param_32)
#define PCLLK_IOCTL_32_LAYOUT_WR	_IOW('o', 120, struct nvc_param_32)
#define PCLLK_IOCTL_32_LAYOUT_RD	_IOWR('o', 121, struct nvc_param_32)
#define PCLLK_IOCTL_32_PARAM_WR	_IOWR('o', 140, struct nvc_param_32)
#define PCLLK_IOCTL_32_PARAM_RD	_IOWR('o', 141, struct nvc_param_32)
#define PCLLK_IOCTL_32_DRV_ADD	_IOW('o', 150, struct nvc_param_32)
#define PCLLK_IOCTL_32_DT_GET	_IOWR('o', 160, struct nvc_param_32)
#define PCLLK_IOCTL_32_MSG		_IOWR('o', 170, struct nvc_param_32)
#endif

#define CAMERA_MAX_EDP_ENTRIES  16
#define CAMERA_MAX_NAME_LENGTH	32
#define CAMDEV_INVALID		0xffffffff

#define	CAMERA_SEQ_STATUS_MASK	0xf0000000
#define	CAMERA_SEQ_INDEX_MASK	0x0000ffff
#define	CAMERA_SEQ_FLAG_MASK	(~CAMERA_SEQ_INDEX_MASK)
#define	CAMERA_SEQ_FLAG_EDP	0x80000000

#define CAMERA_DT_HANDLE_MASK		0xffff00
#define CAMERA_DT_HANDLE_PROFILE	0x000000
#define CAMERA_DT_HANDLE_PHANDLE	0x800000
#define CAMERA_DT_HANDLE_SENSOR		0x400000
#define CAMERA_DT_HANDLE_FOCUSER	0x200000
#define CAMERA_DT_HANDLE_FLASH		0x100000
#define CAMERA_DT_HANDLE_MODULE		0x080000

#define CAMERA_DT_TYPE_MASK	0xff
#define CAMERA_DT_QUERY		0
#define CAMERA_DT_STRING	11
#define CAMERA_DT_DATA_U8	12
#define CAMERA_DT_DATA_U16	13
#define CAMERA_DT_DATA_U32	14
#define CAMERA_DT_DATA_U64	15
#define CAMERA_DT_ARRAY_U8	21
#define CAMERA_DT_ARRAY_U16	22
#define CAMERA_DT_ARRAY_U32	23

#define MAX_PARAM_SIZE_OF_VALUE 1024
#define MAX_PARAM_VARIANT 4096

enum {
	CAMERA_SEQ_EXEC,
	CAMERA_SEQ_REGISTER_EXEC,
	CAMERA_SEQ_REGISTER_ONLY,
	CAMERA_SEQ_EXIST,
	CAMERA_SEQ_MAX_NUM,
};

enum {
	CAMERA_DEVICE_TYPE_I2C,
	CAMERA_DEVICE_TYPE_MAX_NUM,
};

struct camera_device_info {
	__u8 name[CAMERA_MAX_NAME_LENGTH];
	__u32 type;
	__u8 bus;
	__u8 addr;
};

struct camera_reg {
	__u32 addr;
	__u32 val;
};

struct regmap_cfg {
	int addr_bits;
	int val_bits;
	__u32 cache_type;
};

struct gpio_cfg {
	int gpio;
	__u8 own;
	__u8 active_high;
	__u8 flag;
	__u8 reserved;
};

struct edp_cfg {
	uint estates[CAMERA_MAX_EDP_ENTRIES];
	uint num;
};

#define VIRTUAL_DEV_MAX_REGULATORS	8
#define VIRTUAL_DEV_MAX_GPIOS		8
#define VIRTUAL_DEV_MAX_POWER_SIZE	32
#define VIRTUAL_REGNAME_SIZE		(VIRTUAL_DEV_MAX_REGULATORS * \
						CAMERA_MAX_NAME_LENGTH)
#ifdef CONFIG_COMPAT
struct virtual_device_32 {
	__u32 power_on;
	__u32 power_off;
	struct regmap_cfg regmap_cfg;
	__u32 bus_type;
	__u32 gpio_num;
	__u32 reg_num;
	__u32 pwr_on_size;
	__u32 pwr_off_size;
	__u32 clk_num;
	__u8 name[32];
	__u8 reg_names[VIRTUAL_REGNAME_SIZE];
};
#endif

struct virtual_device {
	void *power_on;
	void *power_off;
	struct regmap_cfg regmap_cfg;
	__u32 bus_type;
	__u32 gpio_num;
	__u32 reg_num;
	__u32 pwr_on_size;
	__u32 pwr_off_size;
	__u32 clk_num;
	__u8 name[32];
	__u8 reg_names[VIRTUAL_REGNAME_SIZE];
};

enum {
	UPDATE_PINMUX,
	UPDATE_GPIO,
	UPDATE_POWER,
	UPDATE_CLOCK,
	UPDATE_MAX_NUM,
};

struct cam_update {
	__u32 type;
	__u32 index;
	__u32 size;
	__u32 arg;
	__u32 args[28];
};

enum {
	DEVICE_SENSOR,
	DEVICE_FOCUSER,
	DEVICE_FLASH,
	DEVICE_ROM,
	DEVICE_OTHER,
	DEVICE_OTHER2,
	DEVICE_OTHER3,
	DEVICE_OTHER4,
	DEVICE_MAX_NUM,
};

struct cam_device_layout {
	__u64 guid;
	__u8 name[CAMERA_MAX_NAME_LENGTH];
	__u8 type;
	__u8 alt_name[CAMERA_MAX_NAME_LENGTH];
	__u8 pos;
	__u8 bus;
	__u8 addr;
	__u8 addr_byte;
	__u32 dev_id;
	__u32 index;
	__u32 reserved1;
	__u32 reserved2;
};

struct camera_property_info {
	__u8 name[CAMERA_MAX_NAME_LENGTH];
	__u32 type;
};

#ifdef __KERNEL__

#define NUM_OF_SEQSTACK		16
#define SIZEOF_I2C_BUF		32
#define CAMERA_REGCACHE_MAX (128)

struct camera_device;

struct camera_data_blob {
	char *name;
	void *data;
};

struct camera_board {
	int busnum;
	struct i2c_board_info *bi;
	struct device_node *of_node;
};

struct camera_module {
	struct camera_board sensor;
	struct camera_board focuser;
	struct camera_board flash;
	struct device_node *of_node;
};

struct camera_platform_data {
	unsigned cfg;
	int pinmux_num;
	struct tegra_pingroup_config **pinmux;
	struct camera_module *modules;
	struct camera_data_blob *lut;
	struct device_node *of_profiles;
	uint prof_num;
	uint mod_num;
	uint max_blob_size;
	bool freeable;
};

struct camera_edp_cfg {
	struct sysedp_consumer *edp_client;
	unsigned edp_state;
	uint estates[CAMERA_MAX_EDP_ENTRIES];
	uint num;
};

struct camera_seq_status {
	u32 idx;
	u32 status;
};

struct camera_device {
	struct list_head list;
	u8 name[CAMERA_MAX_NAME_LENGTH];
	struct device *dev;
	struct i2c_client *client;
	struct camera_chip *chip;
	struct regmap *regmap;
	struct camera_info *cam;
	atomic_t in_use;
	struct mutex mutex;
	struct camera_edp_cfg edpc;
	struct clk **clks;
	u32 num_clk;
	struct nvc_regulator *regs;
	u32 num_reg;
	struct nvc_gpio *gpios;
	u32 num_gpio;
	struct tegra_pingroup_config **pinmux_tbl;
	u32 pinmux_num;
	u32 mclk_enable_idx;
	u32 mclk_disable_idx;
	struct regulator *ext_regs;
	struct camera_reg *seq_stack[NUM_OF_SEQSTACK];
	int pwr_state;
	u8 is_power_on;
	u8 i2c_buf[SIZEOF_I2C_BUF];
};

struct camera_chip {
	const u8			name[CAMERA_MAX_NAME_LENGTH];
	u32				type;
	const struct regmap_config	regmap_cfg;
	struct list_head		list;
	atomic_t			ref_cnt;
	void				*private;
	/* function pointers */
	int	(*init)(struct camera_device *cdev, void *);
	int	(*release)(struct camera_device *cdev);
	int	(*power_on)(struct camera_device *cdev);
	int	(*power_off)(struct camera_device *cdev);
	int	(*shutdown)(struct camera_device *cdev);
	int	(*update)(struct camera_device *cdev,
			struct cam_update *upd, u32 num);
};

struct camera_sync_dev {
	char name[CAMERA_MAX_NAME_LENGTH];
	struct regmap *regmap;
	struct camera_reg reg[CAMERA_REGCACHE_MAX];
	u32 num_used;
	struct list_head list;
};

int camera_chip_add(struct camera_chip *chip);

int camera_dev_sync_init(void);
void camera_dev_sync_cb(void *stub);
extern int camera_dev_sync_clear(struct camera_sync_dev *csyncdev);
extern int camera_dev_sync_wr_add(
	struct camera_sync_dev *csyncdev, u32 offset, u32 val);
extern int camera_dev_add_regmap(
	struct camera_sync_dev **csyncdev, u8 *name, struct regmap *regmap);

#ifdef CAMERA_DEVICE_INTERNAL

struct camera_info {
	struct list_head list;
	atomic_t in_use;
	struct device *dev;
	struct mutex k_mutex;
	struct camera_device *cdev;
};

struct camera_platform_info {
	char dname[CAMERA_MAX_NAME_LENGTH];
	struct miscdevice miscdev;
	atomic_t in_use;
	struct device *dev;
	struct camera_platform_data *pdata;
	struct mutex *u_mutex;
	struct list_head *app_list;
	struct mutex *d_mutex;
	struct list_head *dev_list;
	struct mutex *c_mutex;
	struct list_head *chip_list;
	struct dentry *d_entry;
	void *layout;
	size_t size_layout;
};

/* common functions */
int __camera_get_params(
	struct camera_info *, unsigned long, int, struct nvc_param *, void **,
	bool);
static inline int camera_get_params(struct camera_info *cam, unsigned long arg,
		int u_size, struct nvc_param *prm, void **data)
{
	return __camera_get_params(cam, arg, u_size, prm, data, false);
}
int camera_copy_user_params(unsigned long, struct nvc_param *);

int virtual_device_add(struct device *, unsigned long);
int camera_regulator_get(struct device *, struct nvc_regulator *, char *);

/* device access functions */
int camera_dev_parser(
	struct camera_device *, u32, u32 *, struct camera_seq_status *
);
int camera_dev_wr_table(
	struct camera_device *, struct camera_reg *, struct camera_seq_status *
);
int camera_dev_rd_table(struct camera_device *, struct camera_reg *);

/* edp functions */
void camera_edp_register(
	struct camera_device *
);
int camera_edp_req(
	struct camera_device *, unsigned
);
void camera_edp_lowest(
	struct camera_device *
);

/* debugfs functions */
int camera_debugfs_init(
	struct camera_platform_info *
);
int camera_debugfs_remove(void);

/* device tree parser functions */
int of_camera_init(struct camera_platform_info *);
int of_camera_get_property(struct camera_info *, unsigned long);
struct camera_platform_data *of_camera_create_pdata(struct platform_device *);

#endif

#endif
#endif
/* __CAMERA_H__ */
