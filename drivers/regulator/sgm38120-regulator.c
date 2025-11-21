// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2025 Eli
 * SGM38120 Camera LDO Driver for MTK Android Platform
 */

#include <linux/crc8.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/driver.h>
#include <linux/slab.h>

#define GENERIC_DEBUGFS 1

#if GENERIC_DEBUGFS
#include <linux/debugfs.h>
#endif /* GENERIC_DEBUGFS */

#define SGM38120_DRV_VERSION "1.0.1_MTK"

#define SGM38120_REG_CHIP_ID        0x00
#define SGM38120_REG_ENABLE         0x03
#define SGM38120_REG_VSEL_BASE      0x04
#define SGM38120_REG_LDO5_VSEL      0x08
#define SGM38120_REG_LDO7_VSEL      0x0A
#define SGM38120_REG_UVP_INT        0x15
#define SGM38120_REG_OCP_INT        0x16
#define SGM38120_REG_TSD_UVLO_INT   0x17
#define SGM38120_REG_EXTRA_VOLT     0x1F

#define SGM38120_VENDOR_ID_MASK     GENMASK(7, 0)
#define SGM38120_VENDOR_ID          0xD9
#define SGM38120_LDO_ENABLE_MASK    BIT(7)
#define SGM38120_LDO_VSEL_MASK      GENMASK(7, 0)
#define SGM38120_LDO5_EXTRA_MASK    BIT(0)
#define SGM38120_LDO7_EXTRA_MASK    BIT(1)

#define SGM38120_LDO_UVP_EVT_MASK   GENMASK(6, 0)
#define SGM38120_LDO_OCP_EVT_MASK   GENMASK(6, 0)
#define SGM38120_TSD_UVLO_EVT_MASK  GENMASK(1, 0)
#define SGM38120_INTR_CLR_MASK      GENMASK(6, 0)
#define SGM38120_INTR_BYTE_NR       1

#define SGM38120_CRC8_POLYNOMIAL    0x7
#define SGM38120_I2C_ADDR_LEN       1
#define SGM38120_PREDATA_LEN        2
#define SGM38120_REG_ADDR_LEN       1
#define SGM38120_I2C_DUMMY_LEN      1

/* Voltage parameters */
#define LDO12_MIN_UV        528000
#define LDO12_STEP_UV       8000
#define LDO12_N_VOLTAGES    123 /* (1.504 - 0.528) / 0.008 + 1 */

#define LDO346_MIN_UV       1504000
#define LDO346_STEP_UV      8000
#define LDO346_N_VOLTAGES   256 /* (3.544 - 1.504) / 0.008 + 1 */

#define LDO57_EXTRA_UV      1200000
#define LDO57_N_VOLTAGES    257 /* 1.2V + 256 steps from 1.504V to 3.544V */

#if GENERIC_DEBUGFS
struct dbg_internal {
    struct dentry *rt_root;
    struct dentry *ic_root;
    bool rt_dir_create;
    struct mutex io_lock;
    u16 reg;
    u16 size;
    u16 data_buffer_size;
    void *data_buffer;
    bool access_lock;
};

struct dbg_info {
    const char *dirname;
    const char *devname;
    const char *typestr;
    void *io_drvdata;
    int (*io_read)(void *drvdata, u16 reg, void *val, u16 size);
    int (*io_write)(void *drvdata, u16 reg, const void *val, u16 size);
    struct dbg_internal internal;
};

#ifdef CONFIG_DEBUG_FS
#define PREALLOC_RBUFFER_SIZE (32)
#define PREALLOC_WBUFFER_SIZE (1000)

static int data_debug_show(struct seq_file *s, void *data)
{
    struct dbg_info *di = s->private;
    struct dbg_internal *d = &di->internal;
    void *buffer;
    u8 *pdata;
    int i, ret;

    if (d->data_buffer_size < d->size) {
        buffer = kzalloc(d->size, GFP_KERNEL);
        if (!buffer)
            return -ENOMEM;
        kfree(d->data_buffer);
        d->data_buffer = buffer;
        d->data_buffer_size = d->size;
    }

    if (!di->io_read)
        return -EPERM;
    ret = di->io_read(di->io_drvdata, d->reg, d->data_buffer, d->size);
    if (ret < 0)
        return ret;

    pdata = d->data_buffer;
    seq_puts(s, "0x");
    for (i = 0; i < d->size; i++)
        seq_printf(s, "%02x,", *(pdata + i));
    seq_puts(s, "\n");

    return 0;
}

static int data_debug_open(struct inode *inode, struct file *file)
{
    return single_open(file, data_debug_show, inode->i_private);
}

static ssize_t data_debug_write(struct file *file,
                                const char __user *user_buf,
                                size_t cnt, loff_t *loff)
{
    struct seq_file *seq = file->private_data;
    struct dbg_info *di = seq->private;
    struct dbg_internal *d = &di->internal;
    void *buffer;
    u8 *pdata;
    char buf[PREALLOC_WBUFFER_SIZE + 1], *token, *cur;
    int val_cnt = 0, ret;

    if (cnt > PREALLOC_WBUFFER_SIZE)
        return -ENOMEM;
    if (copy_from_user(buf, user_buf, cnt))
        return -EFAULT;
    buf[cnt] = 0;

    if (d->data_buffer_size < d->size) {
        buffer = kzalloc(d->size, GFP_KERNEL);
        if (!buffer)
            return -ENOMEM;
        kfree(d->data_buffer);
        d->data_buffer = buffer;
        d->data_buffer_size = d->size;
    }

    cur = buf;
    pdata = d->data_buffer;
    while ((token = strsep(&cur, ",\n")) != NULL) {
        if (!*token)
            break;
        if (val_cnt++ >= d->size)
            break;
        if (kstrtou8(token, 16, pdata++))
            return -EINVAL;
    }
    if (val_cnt != d->size)
        return -EINVAL;

    if (!di->io_write)
        return -EPERM;
    ret = di->io_write(di->io_drvdata, d->reg, d->data_buffer, d->size);
    return (ret < 0) ? ret : cnt;
}

static const struct file_operations data_debug_fops = {
    .open = data_debug_open,
    .read = seq_read,
    .write = data_debug_write,
    .llseek = seq_lseek,
    .release = single_release,
};

static int type_debug_show(struct seq_file *s, void *data)
{
    struct dbg_info *di = s->private;

    seq_printf(s, "%s,%s\n", di->typestr, di->devname);
    return 0;
}

static int type_debug_open(struct inode *inode, struct file *file)
{
    return single_open(file, type_debug_show, inode->i_private);
}

static const struct file_operations type_debug_fops = {
    .open = type_debug_open,
    .read = seq_read,
    .llseek = seq_lseek,
    .release = single_release,
};

static ssize_t lock_debug_read(struct file *file,
                               char __user *user_buf, size_t cnt, loff_t *loff)
{
    struct dbg_info *di = file->private_data;
    struct dbg_internal *d = &di->internal;
    char buf[10];
    bool lock;
    int ret = 0;

    mutex_lock(&d->io_lock);
    lock = d->access_lock;
    mutex_unlock(&d->io_lock);

    ret = snprintf(buf, sizeof(buf), "%d\n", lock);
    if (ret < 0)
        pr_debug("%s snprintf failed\n", __func__);
    return simple_read_from_buffer(user_buf, cnt, loff, buf, strlen(buf));
}

static ssize_t lock_debug_write(struct file *file,
                                const char __user *user_buf,
                                size_t cnt, loff_t *loff)
{
    struct dbg_info *di = file->private_data;
    struct dbg_internal *d = &di->internal;
    u32 lock;
    int ret;

    ret = kstrtou32_from_user(user_buf, cnt, 0, &lock);
    if (ret < 0)
        return ret;
    mutex_lock(&d->io_lock);
    if (!!lock == d->access_lock)
        ret = -EFAULT;
    d->access_lock = !!lock;
    mutex_unlock(&d->io_lock);
    return (ret < 0) ? ret : cnt;
}

static const struct file_operations lock_debug_fops = {
    .open = simple_open,
    .read = lock_debug_read,
    .write = lock_debug_write,
};

static int generic_debugfs_init(struct dbg_info *di)
{
    struct dbg_internal *d = &di->internal;

    if (!di->dirname || !di->devname || !di->typestr)
        return -EINVAL;
    d->data_buffer_size = PREALLOC_RBUFFER_SIZE;
    d->data_buffer = kzalloc(PREALLOC_RBUFFER_SIZE, GFP_KERNEL);
    if (!d->data_buffer)
        return -ENOMEM;

    d->rt_root = debugfs_lookup("ext_dev_io", NULL);
    if (!d->rt_root) {
        d->rt_root = debugfs_create_dir("ext_dev_io", NULL);
        if (!d->rt_root)
            return -ENODEV;
        d->rt_dir_create = true;
    }
    mutex_init(&d->io_lock);
    d->ic_root = debugfs_create_dir(di->dirname, d->rt_root);
    if (!d->ic_root)
        goto err_cleanup_rt;
    if (!debugfs_create_u16("reg", 0644, d->ic_root, &d->reg))
        goto err_cleanup_ic;
    if (!debugfs_create_u16("size", 0644, d->ic_root, &d->size))
        goto err_cleanup_ic;
    if (!debugfs_create_file("data", 0644, d->ic_root, di, &data_debug_fops))
        goto err_cleanup_ic;
    if (!debugfs_create_file("type", 0444, d->ic_root, di, &type_debug_fops))
        goto err_cleanup_ic;
    if (!debugfs_create_file("lock", 0644, d->ic_root, di, &lock_debug_fops))
        goto err_cleanup_ic;
    return 0;

err_cleanup_ic:
    debugfs_remove_recursive(d->ic_root);
err_cleanup_rt:
    mutex_destroy(&d->io_lock);
    if (d->rt_dir_create)
        debugfs_remove_recursive(d->rt_root);
    kfree(d->data_buffer);
    return -ENODEV;
}

static inline void generic_debugfs_exit(struct dbg_info *di)
{
    struct dbg_internal *d = &di->internal;

    debugfs_remove_recursive(d->ic_root);
    mutex_destroy(&d->io_lock);
    if (d->rt_dir_create)
        debugfs_remove_recursive(d->rt_root);
    kfree(d->data_buffer);
}
#else
static inline int generic_debugfs_init(struct dbg_info *di)
{
    return 0;
}

static inline void generic_debugfs_exit(struct dbg_info *di) {}
#endif
#endif /* GENERIC_DEBUGFS */

enum {
    SGM38120_REGULATOR_LDO1 = 0,
    SGM38120_REGULATOR_LDO2,
    SGM38120_REGULATOR_LDO3,
    SGM38120_REGULATOR_LDO4,
    SGM38120_REGULATOR_LDO5,
    SGM38120_REGULATOR_LDO6,
    SGM38120_REGULATOR_LDO7,
    SGM38120_REGULATOR_MAX
};

struct sgm38120_priv {
    struct device *dev;
    struct i2c_client *client;
    struct gpio_desc *enable_gpio;
    struct regulator_dev *rdev[SGM38120_REGULATOR_MAX];
#if GENERIC_DEBUGFS
    struct dbg_info dbg_info;
#endif /* GENERIC_DEBUGFS */
};

//u8 crc8_tbls[CRC8_TABLE_SIZE];

/* Helper functions for I2C read/write with CRC */

static int sgm38120_i2c_read_byte(struct i2c_client *client, u8 reg, u8 *val)
{
	int stat;
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 1,
			.buf = &reg,
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = val,
		}
	};

	stat = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (stat < 0) {
		printk("i2c read error: %d\n", stat);
	} else if (stat != ARRAY_SIZE(msgs)) {
		printk("i2c read N mismatch: %d\n", stat);
		stat = -EIO;
	} else {
		stat = 0;
	}

	return stat;
}

static int sgm38120_i2c_write_byte(struct i2c_client *client, u8 reg, u8 val)
{
	int stat;
	uint8_t txbuf[2] = { reg, val };
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = 2,
			.buf = txbuf,
		}
	};

	stat = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));

	if (stat < 0) {
		printk("i2c send error: %d\n", stat);
	} else if (stat != ARRAY_SIZE(msgs)) {
		printk("i2c send N mismatch: %d\n", stat);
		stat = -EIO;
	} else {
		stat = 0;
	}

	return stat;
}

static int sgm38120_i2c_update_bits(struct i2c_client *client, u8 reg, u8 mask, u8 val)
{
    u8 old_val, new_val;
    int ret;

    ret = sgm38120_i2c_read_byte(client, reg, &old_val);
    if (ret)
        return ret;

    new_val = (old_val & ~mask) | (val & mask);
    if (new_val == old_val)
        return 0;

    return sgm38120_i2c_write_byte(client, reg, new_val);
}

/* Custom voltage operations for LDO5 and LDO7 */
static int sgm38120_ldo5_7_list_voltage(struct regulator_dev *rdev, unsigned sel)
{
    //printk(" %s:sel=%d\n", __FUNCTION__ , sel);
    if (sel == 0)
        return LDO57_EXTRA_UV;
    else if (sel <= 256)
        return LDO346_MIN_UV + (sel - 1) * LDO346_STEP_UV;
    else
        return -EINVAL;
}

static int sgm38120_ldo5_7_set_voltage_sel(struct regulator_dev *rdev, unsigned sel)
{
    struct sgm38120_priv *priv = rdev_get_drvdata(rdev);
    int id = rdev_get_id(rdev);
    u8 mask = (id == SGM38120_REGULATOR_LDO5) ? SGM38120_LDO5_EXTRA_MASK : SGM38120_LDO7_EXTRA_MASK;
    u8 reg = (id == SGM38120_REGULATOR_LDO5) ? SGM38120_REG_LDO5_VSEL : SGM38120_REG_LDO7_VSEL;
    int ret;
    printk(" %s:id=%d,sel=%d\n", __FUNCTION__ , id, sel);
    if (sel == 0) {
        ret = sgm38120_i2c_update_bits(priv->client, SGM38120_REG_EXTRA_VOLT, mask, mask);
    } else {
        ret = sgm38120_i2c_update_bits(priv->client, SGM38120_REG_EXTRA_VOLT, mask, 0);
        if (!ret)
            ret = sgm38120_i2c_write_byte(priv->client, reg, sel - 1);
    }
    return ret;
}

static int sgm38120_ldo5_7_get_voltage_sel(struct regulator_dev *rdev)
{
    struct sgm38120_priv *priv = rdev_get_drvdata(rdev);
    int id = rdev_get_id(rdev);
    u8 mask = (id == SGM38120_REGULATOR_LDO5) ? SGM38120_LDO5_EXTRA_MASK : SGM38120_LDO7_EXTRA_MASK;
    u8 val;
    int ret;
    //printk(" %s:mask=0x%x\n", __FUNCTION__ , mask);
    ret = sgm38120_i2c_read_byte(priv->client, SGM38120_REG_EXTRA_VOLT, &val);
    if (ret)
        return ret;
    if (val & mask)
        return 0; /* 1.2V */
    ret = sgm38120_i2c_read_byte(priv->client, (id == SGM38120_REGULATOR_LDO5) ? SGM38120_REG_LDO5_VSEL : SGM38120_REG_LDO7_VSEL, &val);
    if (ret)
        return ret;
    return val + 1;
}

/* Regulator operations */
static int sgm38120_set_voltage_sel(struct regulator_dev *rdev, unsigned sel)
{
    struct sgm38120_priv *priv = rdev_get_drvdata(rdev);
    int id = rdev_get_id(rdev);
    u8 reg = SGM38120_REG_VSEL_BASE + id;
    //printk(" %s:sel=0x%x, reg=0x%x\n", __FUNCTION__ , sel,reg);
    return sgm38120_i2c_write_byte(priv->client, reg, sel);
}

static int sgm38120_get_voltage_sel(struct regulator_dev *rdev)
{
    struct sgm38120_priv *priv = rdev_get_drvdata(rdev);
    int id = rdev_get_id(rdev);
    u8 reg = SGM38120_REG_VSEL_BASE + id;
    u8 val;
    int ret;
    printk(" %s:id=%d,reg=0x%x\n", __FUNCTION__ , id, reg);
    ret = sgm38120_i2c_read_byte(priv->client, reg, &val);
    if (ret)
        return ret;
    return val;
}

static int sgm38120_enable(struct regulator_dev *rdev)
{
    struct sgm38120_priv *priv = rdev_get_drvdata(rdev);
    int id = rdev_get_id(rdev);
    u8 mask = BIT(id);
    printk(" %s:id=%d\n", __FUNCTION__ , id);
    sgm38120_i2c_update_bits(priv->client, SGM38120_REG_ENABLE, 1<<7, 1<<7);
    return sgm38120_i2c_update_bits(priv->client, SGM38120_REG_ENABLE, mask, mask);
 //   return sgm38120_i2c_update_bits(priv->client, SGM38120_REG_ENABLE, 0XFF, 0XFF);

}

static int sgm38120_disable(struct regulator_dev *rdev)
{
    struct sgm38120_priv *priv = rdev_get_drvdata(rdev);
    int id = rdev_get_id(rdev);
    u8 mask = BIT(id);
    printk(" %s:id=%d\n", __FUNCTION__ , id);
    
    return sgm38120_i2c_update_bits(priv->client, SGM38120_REG_ENABLE, mask, 0);
}

static int sgm38120_is_enabled(struct regulator_dev *rdev)
{
    struct sgm38120_priv *priv = rdev_get_drvdata(rdev);
    int id = rdev_get_id(rdev);
    u8 val;
    int ret;
    //printk(" %s:%d\n", __FUNCTION__ , __LINE__);
    ret = sgm38120_i2c_read_byte(priv->client, SGM38120_REG_ENABLE, &val);
    if (ret)
        return ret;
    return !!(val & BIT(id));
}

static const struct regulator_ops sgm38120_ldo_ops = {
    .set_voltage_sel = sgm38120_set_voltage_sel,
    .get_voltage_sel = sgm38120_get_voltage_sel,
    .list_voltage = regulator_list_voltage_linear,
    .enable = sgm38120_enable,
    .disable = sgm38120_disable,
    .is_enabled = sgm38120_is_enabled,
};

static const struct regulator_ops sgm38120_ldo5_7_ops = {
    .set_voltage_sel = sgm38120_ldo5_7_set_voltage_sel,
    .get_voltage_sel = sgm38120_ldo5_7_get_voltage_sel,
    .list_voltage = sgm38120_ldo5_7_list_voltage,
    .enable = sgm38120_enable,
    .disable = sgm38120_disable,
    .is_enabled = sgm38120_is_enabled,
};

static int sgm38120_of_parse_cb(struct device_node *node,
                                const struct regulator_desc *desc,
                                struct regulator_config *config)
{
    struct sgm38120_priv *priv = config->driver_data;
    unsigned int base_addr = SGM38120_REG_VSEL_BASE + desc->id;
    int ret;
    unsigned int val;

    if (desc->id == SGM38120_REGULATOR_LDO5 || desc->id == SGM38120_REGULATOR_LDO7)
        base_addr = (desc->id == SGM38120_REGULATOR_LDO5) ? SGM38120_REG_LDO5_VSEL : SGM38120_REG_LDO7_VSEL;
    ret = of_property_read_u32(node, "soft_start_time_sel", &val);
    if (!ret && val <= 3) {
        dev_info(priv->dev, "Soft start time for %s set to %u\n", desc->name, val);
    }
    
    //printk(" %s:val=%d\n", __FUNCTION__ , val);

    return 0;
}

#define SGM38120_REGULATOR_DESC(_name, _id, _ops, _n_voltages, _min_uV, _step_uV, _supply) \
{ \
    .name = "sgm38120-" #_name, \
    .id = SGM38120_REGULATOR_##_name, \
    .of_match = of_match_ptr(#_name), \
    .regulators_node = of_match_ptr("regulators"), \
    .supply_name = _supply, \
    .of_parse_cb = sgm38120_of_parse_cb, \
    .type = REGULATOR_VOLTAGE, \
    .owner = THIS_MODULE, \
    .ops = _ops, \
    .n_voltages = _n_voltages, \
    .min_uV = _min_uV, \
    .uV_step = _step_uV, \
    .enable_reg = SGM38120_REG_ENABLE, \
    .enable_mask = BIT(_id), \
    .enable_time = 2000, \
    .vsel_reg = SGM38120_REG_VSEL_BASE + _id, \
    .vsel_mask = SGM38120_LDO_VSEL_MASK, \
}

static const struct regulator_desc sgm38120_regulators[] = {
    SGM38120_REGULATOR_DESC(LDO1, 0, &sgm38120_ldo_ops, LDO12_N_VOLTAGES, LDO12_MIN_UV, LDO12_STEP_UV, "sgm38120-base"),
    SGM38120_REGULATOR_DESC(LDO2, 1, &sgm38120_ldo_ops, LDO12_N_VOLTAGES, LDO12_MIN_UV, LDO12_STEP_UV, "sgm38120-base"),
    SGM38120_REGULATOR_DESC(LDO3, 2, &sgm38120_ldo_ops, LDO346_N_VOLTAGES, LDO346_MIN_UV, LDO346_STEP_UV, "sgm38120-base"),
    SGM38120_REGULATOR_DESC(LDO4, 3, &sgm38120_ldo_ops, LDO346_N_VOLTAGES, LDO346_MIN_UV, LDO346_STEP_UV, "sgm38120-base"),
    {
        .name = "sgm38120-LDO5",
        .id = SGM38120_REGULATOR_LDO5,
        .of_match = of_match_ptr("LDO5"),
        .regulators_node = of_match_ptr("regulators"),
        .supply_name = "sgm38120-base",
        .of_parse_cb = sgm38120_of_parse_cb,
        .type = REGULATOR_VOLTAGE,
        .owner = THIS_MODULE,
        .ops = &sgm38120_ldo5_7_ops,
        .n_voltages = LDO57_N_VOLTAGES,
        .enable_reg = SGM38120_REG_ENABLE,
        .enable_mask = BIT(4),
        .enable_time = 2000,
    },
    SGM38120_REGULATOR_DESC(LDO6, 5, &sgm38120_ldo_ops, LDO346_N_VOLTAGES, LDO346_MIN_UV, LDO346_STEP_UV, "sgm38120-base"),
    {
        .name = "sgm38120-LDO7",
        .id = SGM38120_REGULATOR_LDO7,
        .of_match = of_match_ptr("LDO7"),
        .regulators_node = of_match_ptr("regulators"),
        .supply_name = "sgm38120-base",
        .of_parse_cb = sgm38120_of_parse_cb,
        .type = REGULATOR_VOLTAGE,
        .owner = THIS_MODULE,
        .ops = &sgm38120_ldo5_7_ops,
        .n_voltages = LDO57_N_VOLTAGES,
        .enable_reg = SGM38120_REG_ENABLE,
        .enable_mask = BIT(6),
        .enable_time = 2000,
    },
};

static irqreturn_t sgm38120_intr_handler(int irq_number, void *data)
{
    struct sgm38120_priv *priv = data;
    u8 uvp_status, ocp_status, tsd_uvlo_status;
    int i, ret;

    ret = sgm38120_i2c_read_byte(priv->client, SGM38120_REG_UVP_INT, &uvp_status);
    if (ret)
        goto out_intr_handler;
    if (uvp_status & SGM38120_LDO_UVP_EVT_MASK) {
        for (i = 0; i < SGM38120_REGULATOR_MAX; i++) {
            if (uvp_status & BIT(i))
                regulator_notifier_call_chain(priv->rdev[i], REGULATOR_EVENT_UNDER_VOLTAGE, &i);
        }
    }

    ret = sgm38120_i2c_read_byte(priv->client, SGM38120_REG_OCP_INT, &ocp_status);
    if (ret)
        goto out_intr_handler;
    if (ocp_status & SGM38120_LDO_OCP_EVT_MASK) {
        for (i = 0; i < SGM38120_REGULATOR_MAX; i++) {
            if (ocp_status & BIT(i))
                regulator_notifier_call_chain(priv->rdev[i], REGULATOR_EVENT_OVER_CURRENT, &i);
        }
    }

    ret = sgm38120_i2c_read_byte(priv->client, SGM38120_REG_TSD_UVLO_INT, &tsd_uvlo_status);
    if (ret)
        goto out_intr_handler;
    if (tsd_uvlo_status & SGM38120_TSD_UVLO_EVT_MASK) {
        dev_info(priv->dev, "TSD/UVLO event: 0x%x\n", tsd_uvlo_status);
    }

    ret = sgm38120_i2c_write_byte(priv->client, SGM38120_REG_UVP_INT, uvp_status);
    if (ret)
        goto out_intr_handler;
    ret = sgm38120_i2c_write_byte(priv->client, SGM38120_REG_OCP_INT, ocp_status);
    if (ret)
        goto out_intr_handler;
    ret = sgm38120_i2c_write_byte(priv->client, SGM38120_REG_TSD_UVLO_INT, tsd_uvlo_status);
    if (ret)
        goto out_intr_handler;

    return IRQ_HANDLED;

out_intr_handler:
    dev_err(priv->dev, "Interrupt handling failed: %d\n", ret);
    return IRQ_NONE;
}

static int sgm38120_enable_interrupts(int irq_no, struct sgm38120_priv *priv)
{
    u8 mask = SGM38120_INTR_CLR_MASK;
    int ret;

    ret = sgm38120_i2c_write_byte(priv->client, SGM38120_REG_UVP_INT, mask);
    if (ret) {
        dev_err(priv->dev, "Failed to clear UVP interrupts\n");
        return ret;
    }
    ret = sgm38120_i2c_write_byte(priv->client, SGM38120_REG_OCP_INT, mask);
    if (ret) {
        dev_err(priv->dev, "Failed to clear OCP interrupts\n");
        return ret;
    }
    ret = sgm38120_i2c_write_byte(priv->client, SGM38120_REG_TSD_UVLO_INT, mask);
    if (ret) {
        dev_err(priv->dev, "Failed to clear TSD/UVLO interrupts\n");
        return ret;
    }

    return devm_request_threaded_irq(priv->dev, irq_no, NULL,
                                     sgm38120_intr_handler, IRQF_ONESHOT,
                                     dev_name(priv->dev), priv);
}

#if GENERIC_DEBUGFS
static int sgm38120_dbg_io_read(void *drvdata, u16 reg, void *val, u16 size)
{
    struct i2c_client *client = drvdata;
    u8 *buf = val;
    int i, ret;

    for (i = 0; i < size; i++) {
        ret = sgm38120_i2c_read_byte(client, reg + i, &buf[i]);
        if (ret)
            return ret;
    }
    return 0;
}

static int sgm38120_dbg_io_write(void *drvdata, u16 reg, const void *val, u16 size)
{
    struct i2c_client *client = drvdata;
    const u8 *buf = val;
    int i, ret;

    for (i = 0; i < size; i++) {
        ret = sgm38120_i2c_write_byte(client, reg + i, buf[i]);
        if (ret)
            return ret;
    }
    return 0;
}
#endif /* GENERIC_DEBUGFS */

static int sgm38120_validate_vendor_info(struct sgm38120_priv *priv)
{
    u8 val = 0;
    int ret;

    ret = sgm38120_i2c_read_byte(priv->client, SGM38120_REG_CHIP_ID, &val);
    //printk(" %s:ret =%d, val=0x%x\n", __FUNCTION__ , ret ,(val & SGM38120_VENDOR_ID_MASK));
    if (ret)
        return ret;

    if ((val & SGM38120_VENDOR_ID_MASK) != SGM38120_VENDOR_ID)
        return -ENODEV;

    return 0;
}


static int sgm38120_probe(struct i2c_client *i2c)
{
    struct sgm38120_priv *priv;
    struct regulator_config config = {0};
    int i, ret;

    dev_info(&i2c->dev, "%s start(%s)\n", __func__, SGM38120_DRV_VERSION);
    priv = devm_kzalloc(&i2c->dev, sizeof(*priv), GFP_KERNEL);
    if (!priv)
        return -ENOMEM;

    priv->dev = &i2c->dev; 
	priv->client = i2c;

    priv->enable_gpio = devm_gpiod_get_optional(&i2c->dev, "enable", GPIOD_OUT_HIGH);
    if (IS_ERR(priv->enable_gpio)) {
        dev_err(&i2c->dev, "Failed to request enable GPIO\n");
        return PTR_ERR(priv->enable_gpio);
    }

#if GENERIC_DEBUGFS
    priv->dbg_info.dirname = devm_kasprintf(&i2c->dev, GFP_KERNEL, "SGM38120.%s", dev_name(&i2c->dev));
    priv->dbg_info.devname = dev_name(&i2c->dev);
    priv->dbg_info.typestr = devm_kasprintf(&i2c->dev, GFP_KERNEL, "I2C,SGM38120");
    priv->dbg_info.io_drvdata = priv->client;
    priv->dbg_info.io_read = sgm38120_dbg_io_read;
    priv->dbg_info.io_write = sgm38120_dbg_io_write;

    ret = generic_debugfs_init(&priv->dbg_info);
    if (ret < 0)
        return ret;
#endif /* GENERIC_DEBUGFS */

    ret = sgm38120_validate_vendor_info(priv);
    if (ret) {
        dev_err(&i2c->dev, "Failed to check vendor info [%d]\n", ret);
        return ret;
    }

    config.dev = &i2c->dev;
    config.driver_data = priv;

    for (i = 0; i < SGM38120_REGULATOR_MAX; i++) {
        priv->rdev[i] = devm_regulator_register(&i2c->dev, &sgm38120_regulators[i], &config);
        if (IS_ERR(priv->rdev[i])) {
            dev_err(&i2c->dev, "Failed to register [%d] regulator\n", i);
            return PTR_ERR(priv->rdev[i]);
        }
    }

    ret = sgm38120_enable_interrupts(i2c->irq, priv);
    if (ret) {
        dev_err(&i2c->dev, "Enable interrupt failed\n");
       // return ret;
    }

    dev_info(&i2c->dev, "%s done.\n", __func__);
    return 0;
}

static const struct of_device_id sgm38120_ofid_tbls[] = {
    { .compatible = "sgmicro,sgm38120", },
    { }
};
MODULE_DEVICE_TABLE(of, sgm38120_ofid_tbls);

static struct i2c_driver sgm38120_driver = {
    .driver = {
        .name = "sgm38120",
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(sgm38120_ofid_tbls),
    },
    .probe_new = sgm38120_probe,
};
module_i2c_driver(sgm38120_driver);

MODULE_AUTHOR("Eli oywj321@gmail.com");
MODULE_DESCRIPTION("SGM38120 Camera LDO Driver for MTK Android Platform");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(SGM38120_DRV_VERSION);
