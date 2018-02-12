/* drivers/input/touchscreen/pixart.c
 *
 * Copyright (C) 2012 Pixart Imaging Inc.
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
#include <linux/init.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/time.h>
#include <asm/irq.h>

#include <linux/i2c/pixart.h>
#include "pixart-fw.h"
#include "ts_ctrl.h"
#ifdef CONFIG_OF
#include "board-8974-touch.h"
#endif

#include <linux/input/mt.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <asm/uaccess.h>

#include <linux/regulator/consumer.h>

#define PIXART_DRIVER_MOD_TIME "2012/10/30"
#define PIXART_DRIVER_VERSION "Pixart AMRI/PAP1100 v2.10"

#define MODULE_NAME "pixart: "

#define PIX_DEV_DBG(fmt, arg...)	if(ts_log_level & 0x01) printk(fmt, ## arg)
#define PIX_DEV_INFO(fmt, arg...)	if(ts_log_level & 0x02) printk(fmt, ## arg)
#define PIX_DEV_TOUCH(fmt, arg...)	if(ts_log_level & 0x04) printk(fmt, ## arg)
#define PIX_DEV_I2C(fmt, arg...)	if(ts_log_level & 0x08) printk(fmt, ## arg)

/**
 *	Variables
 */
/* static struct regulator *reg_touch; */
static uint8_t pid = 0;
static uint8_t new_fw_G2 = 0;
static int pixart_debug_message_mask = 0;
static int disable_count;
static struct workqueue_struct *pixart_wq;
static int Touch_Status = TOUCH_POWERON;
static struct sysfs_data_ {
	uint8_t command;
	uint8_t reg[3];
	uint8_t data;
	uint8_t number;
	uint8_t val0;
	uint8_t val1;
} sdata;

struct pixart_data {
    struct i2c_client *client;
    struct input_dev *input;
    unsigned int irq;
    struct delayed_work esdwork;
    uint8_t touch_number;
#if defined(CONFIG_FB)
    struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend early_suspend;
#endif
    struct mutex lock;
    struct cdev device_cdev;
    struct completion init_done;
    struct pixart_touch_data_gen2 touch_data;
    enum ts_config_type config_status;
    enum ts_config_type last_config_status;
    struct ts_config_nv config_nv[TS_CONFIG_MAX];
    bool is_enable;
    bool is_set;
};

/**
 * Function prototypes
 */
static int pixart_init_panel(struct pixart_data *ts);
static int pixart_load_fw(struct i2c_client *client);
#if 0
static int pixart_set_nv(struct pixart_data *ts);
#endif
static int pixart_flash_save(struct pixart_data *ts);
static void pixart_report_clear(struct pixart_data *ts);

/* Explanation of sysfs debug interface
 *
 * With root privilege, use adb and type
 * cd /sys/bus/spi/drivers/pixart-pt
 * cd spi1.0
 * This follows a symbolic link which generally gets you to
 * /sys/devices/platform/omap2_mcspi.1/spi1.0
 *
 * From there you can do several things
 *
 *  Register dump:
 *  echo "r 00 80" > m
 *  cat m
 *  (the first parameter is a starting address (hex) from 0 to 7f
 *   and the 2nd parameter is a length (hex) from 1 to 80
 *
 * Write to a register (one byte):
 *  echo "w 22 95" > m
 *  (the first parameter is the address (hex) from 0 to 7f
 *   the 2nd parameter is the value (hex) from 0 to ff
 * Write to multiple registers:
 *  echo "w a1 d1 a2 d2 a3 d3"   > m
 *  (up to 64 pairs of address, value both in hex)
 *
 * Burst Write up to 128 bytes to register 0x0b
 *  echo "b 0b 55 56  a6.... aa 55" > m
 *
 * Disable navigation for 10000 reports
 *  echo "d" > m
 *
 * Enable navigation
 *  echo "e" > m
 *
 * Triple register read repeats
 *  echo "t ra0 ra1 ra2 n" > m
 *   (ra0 is the hex address of the 1st register to repeatedly read,
 *	ra1 is the hex address of the 2nd register to repeatedly read,
 *	ra2 is the hex address of the 3rd register to repeatedly read,
 *	 n  is the hex number of times to read these 3 registers (max may be near 14A))
 *
 * Write 2 then read 1 repeatedly:
 *  echo "m wa0 val0 wa1 val1 ra n " > m
 *  cat m
 *   (wa0 is the hex address of the 1st register to write once
 *	val0 is the hex value to write once to the 1st register
 *	wa1 is the hex address of the 2nd register to write once
 *	val1 is the hex value to write once to the 2nd register
 *	ra is the hex address of the  register to repeatedly read,
 *	 n  is the hex number of times to read this register (max may be near 3E8))
 *
 * Find driver version string
 *  echo "v" > m
 *  cat m
 *
 * Reload firmware
 *  echo "f" > m
 *
 * Kernel debug message level
 *  echo "k 1" > m
 *
 *  NRST gpio pin raise/lower
 *	echo "g 0" > m
 *	echo "g 1" > m
 *
 * These can be performed from the command line (outside adb) with this syntax (examples)
 * adb shell "echo r 00 80 > /sys/devices/platform/omap2_mcspi.1/spi1.0/m"
 * adb shell "cat /sys/devices/platform/omap2_mcspi.1/spi1.0/m"
 *
 * adb shell "echo w 22 95 > /sys/devices/platform/omap2_mcspi.1/spi1.0/m"
 *
 * adb shell "echo d > /sys/devices/platform/omap2_mcspi.1/spi1.0/m"
 * adb shell "echo e > /sys/devices/platform/omap2_mcspi.1/spi1.0/m"
 *
 * adb shell "echo v > /sys/devices/platform/omap2_mcspi.1/spi1.0/m"
 * adb shell "cat /sys/devices/platform/omap2_mcspi.1/spi1.0/m"
 *
 * Following true for I2C based Drver, only:
 * adb shell "echo r 00 80 > /sys/devices/i2c-3/3-006b/m"
 * adb shell "cat /sys/devices/i2c-3/3-006b/m"
 *
 * adb shell "echo w 22 95 > /sys/devices/i2c-3/3-006b/m"
 *
 * adb shell "echo d > /sys/devices/i2c-3/3-006b/m"
 * adb shell "echo e > /sys/devices/i2c-3/3-006b/m"
 *
 * adb shell "echo v > /sys/devices/i2c-3/3-006b/m"
 * adb shell "cat /sys/devices/i2c-3/3-006b/m"
 */

/**
 *	Function Definitions
 */
static uint8_t pixart_r_reg_byte(struct i2c_client *client, uint8_t reg)
{
	int error = 0;
	unsigned char data[1];

	if (!client->adapter)
		return -ENODEV;

	error = i2c_smbus_read_i2c_block_data(client, reg, 1, (uint8_t *) data);
	if (error != 1)
		pr_err("PIXART_READ_REG_BYTE error =0x%02x\n", error);

	PIX_DEV_I2C("%s: add: %02x, val: %02x\n",__func__, reg , data[0]);

	return data[0];
}

static uint8_t pixart_r_reg_nbytes(struct i2c_client *client,
		uint8_t *readAddress, uint8_t *pdata, uint8_t len)
{
	if (!client->adapter)
		return -ENODEV;

	return i2c_smbus_read_i2c_block_data(client, readAddress[0], len, pdata);
}

static int pixart_w_reg_byte(struct i2c_client *client,
		uint8_t reg, uint8_t val)
{
	int error;

	if (!client->adapter)
		return -ENODEV;

	error = i2c_smbus_write_i2c_block_data(client, reg, 1, &val);
	PIX_DEV_I2C("%s: add: %02x, val: %02x\n",__func__, reg , val);
	udelay(PIXART_DELAY_US_BETWEEN_I2C_WRITES);

	return error;
}

static int pixart_w_na_reg(struct i2c_client *client,
		uint8_t index, uint8_t val)
{
	int error;

	if (!client->adapter)
		return -ENODEV;

	error = pixart_w_reg_byte(client,
				  PIXART_REG_EXTENDED_REG_GEN2,
				  PIXART_EX_NA_CHOOSE_ACTION_INDEX);
	if (error)
		return error;

	error = pixart_w_reg_byte(client,
				  PIXART_REG_EXTENDED_VAL_GEN2,
				  index);
	if (error)
		return error;

	error = pixart_w_reg_byte(client,
				  PIXART_REG_EXTENDED_REG_GEN2,
				  PIXART_EX_NA_CHOOSE_NA);
	if (error)
		return error;

	error = pixart_w_reg_byte(client,
				  PIXART_REG_EXTENDED_VAL_GEN2,
				  val);

	return error;
}

#if 0
static int pixart_r_na_reg(struct i2c_client *client,
		uint8_t index, uint8_t *val)
{
	int error;

	if (!client->adapter)
		return -ENODEV;

	error = pixart_w_reg_byte(client,
				  PIXART_REG_EXTENDED_REG_GEN2,
				  PIXART_EX_NA_CHOOSE_ACTION_INDEX);
	if (error)
		return error;

	error = pixart_w_reg_byte(client,
				  PIXART_REG_EXTENDED_VAL_GEN2,
				  index);
	if (error)
		return error;

	error = pixart_w_reg_byte(client,
				  PIXART_REG_EXTENDED_REG_GEN2,
				  PIXART_EX_NA_CHOOSE_NA);
	if (error)
		return error;

	*val = pixart_r_reg_byte(client, PIXART_REG_EXTENDED_VAL_GEN2);

	return error;
}
#endif

static int pixart_enable_irq(struct pixart_data *ts)
{
	PIX_DEV_DBG("%s() is called.\n",__func__);
	mutex_lock(&ts->lock);
	if (ts->is_enable)
		goto done;

	ts->is_enable = true;
	enable_irq(ts->client->irq);
done:
	mutex_unlock(&ts->lock);
	return 0;
}

static void pixart_disable_irq(struct pixart_data *ts)
{
	PIX_DEV_DBG("%s() is called.\n",__func__);
	mutex_lock(&ts->lock);
	if (!ts->is_enable)
		goto done;

	disable_irq_nosync(ts->client->irq);
	ts->is_enable = false;
done:
	mutex_unlock(&ts->lock);
}

static int pixart_set_config(struct pixart_data *ts)
{
	const struct pixart_platform_data *pdata = ts->client->dev.platform_data;
	int ret, i;
	uint8_t val;
	u8 flash;

	for(i=0; i<pdata->config_length; ){
		val = pixart_r_reg_byte(ts->client, pdata->config[i]);
		if(val != pdata->config[i+2]){
			printk("%s: Add->%02x, Value->%02x\n",__func__,pdata->config[i], pdata->config[i+2]);
			/* shuusei ga hituyou */
			ret = pixart_w_reg_byte(ts->client, pdata->config[i], pdata->config[i+2]);
			i += 3;
			flash = 1;
			if(ret){
				pr_err("%s: Error Write register\n",__func__);
				return ret;
			}
		}
		else
			i += 3;
	}

	pixart_w_na_reg(ts->client, PIXART_NA_DISABLE, 0x08);
	pixart_w_na_reg(ts->client, PIXART_NA_OPTIONS_MISC, 0x00);
	pixart_w_na_reg(ts->client, PIXART_NA_DRIVE_FREQ_INIT, 0x47);
	pixart_w_na_reg(ts->client, PIXART_NOISE_THRESH, 0x0A);
	pixart_w_na_reg(ts->client, PIXART_K1LOG2, 0x1C);
	pixart_w_na_reg(ts->client, PIXART_K2LOG2, 0x1C);
	pixart_w_na_reg(ts->client, PIXART_MAX_CORR_DIST_PER_REPORT_LSB, 0xFA);
	pixart_w_na_reg(ts->client, PIXART_MAX_CORR_DIST_PER_REPORT_MSB, 0x00);
	pixart_w_na_reg(ts->client, PIXART_MAX_CORR_DIST_PER_REPORT_LSB_CHARG, 0xF0);
	pixart_w_na_reg(ts->client, PIXART_MAX_CORR_DIST_PER_REPORT_MSB_CHARG, 0x00);
	pixart_w_na_reg(ts->client, PIXART_TOUCH_DELAY_T3, 0x04);
	pixart_w_na_reg(ts->client, PIXART_TOUCH_DELAY_T3_CHARG, 0x0A);
	pixart_w_na_reg(ts->client, PIXART_TOUCH_DELAY_T4, 0x03);
	pixart_w_na_reg(ts->client, PIXART_TOUCH_DELAY_T4_CHARG, 0x05);
	pixart_w_na_reg(ts->client, PIXART_TOUCH_DELAY_T5, 0x02);
	pixart_w_na_reg(ts->client, PIXART_TOUCH_DELAY_T5_CHARG, 0x03);
	pixart_w_na_reg(ts->client, PIXART_PROLONG_DELAY, 0x02);
	pixart_w_na_reg(ts->client, PIXART_PROLONG_DELAY_CHARG, 0x02);
	pixart_w_na_reg(ts->client, PIXART_NA_RESUME, 0x04);
	pixart_w_na_reg(ts->client, PIXART_ALC_CONTROL_ENABLE_BITS, 0x0B);
	pixart_w_na_reg(ts->client, PIXART_ALC_THRESH_2, 0x0C);
	pixart_w_na_reg(ts->client, PIXART_ALC_THRESH_3, 0x0A);

#if 0
	pixart_r_na_reg(ts->client, PIXART_NA_DISABLE, (uint8_t *)&val);
	printk("%s: PIXART_NA_DISABLE = 0x%X\n",__func__, val);
	pixart_r_na_reg(ts->client, PIXART_NA_OPTIONS_MISC, (uint8_t *)&val);
	printk("%s: PIXART_NA_OPTIONS_MISC = 0x%X\n",__func__, val);
	pixart_r_na_reg(ts->client, PIXART_NA_DRIVE_FREQ_INIT, (uint8_t *)&val);
	printk("%s: PIXART_NA_DRIVE_FREQ_INIT = 0x%X\n",__func__, val);
	pixart_r_na_reg(ts->client, PIXART_NOISE_THRESH, (uint8_t *)&val);
	printk("%s: PIXART_NOISE_THRESH = 0x%X\n",__func__, val);
	pixart_r_na_reg(ts->client, PIXART_K1LOG2, (uint8_t *)&val);
	printk("%s: PIXART_K1LOG2 = 0x%X\n",__func__, val);
	pixart_r_na_reg(ts->client, PIXART_K2LOG2, (uint8_t *)&val);
	printk("%s: PIXART_K2LOG2 = 0x%X\n",__func__, val);
	pixart_r_na_reg(ts->client, PIXART_MAX_CORR_DIST_PER_REPORT_LSB, (uint8_t *)&val);
	printk("%s: PIXART_MAX_CORR_DIST_PER_REPORT_LSB = 0x%X\n",__func__, val);
	pixart_r_na_reg(ts->client, PIXART_MAX_CORR_DIST_PER_REPORT_MSB, (uint8_t *)&val);
	printk("%s: PIXART_MAX_CORR_DIST_PER_REPORT_MSB = 0x%X\n",__func__, val);
	pixart_r_na_reg(ts->client, PIXART_MAX_CORR_DIST_PER_REPORT_LSB_CHARG, (uint8_t *)&val);
	printk("%s: PIXART_MAX_CORR_DIST_PER_REPORT_LSB_CHARG = 0x%X\n",__func__, val);
	pixart_r_na_reg(ts->client, PIXART_MAX_CORR_DIST_PER_REPORT_MSB_CHARG, (uint8_t *)&val);
	printk("%s: PIXART_MAX_CORR_DIST_PER_REPORT_MSB_CHARG = 0x%X\n",__func__, val);
	pixart_r_na_reg(ts->client, PIXART_TOUCH_DELAY_T3, (uint8_t *)&val);
	printk("%s: PIXART_TOUCH_DELAY_T3 = 0x%X\n",__func__, val);
	pixart_r_na_reg(ts->client, PIXART_TOUCH_DELAY_T3_CHARG, (uint8_t *)&val);
	printk("%s: PIXART_TOUCH_DELAY_T3_CHARG = 0x%X\n",__func__, val);
	pixart_r_na_reg(ts->client, PIXART_TOUCH_DELAY_T4, (uint8_t *)&val);
	printk("%s: PIXART_TOUCH_DELAY_T4 = 0x%X\n",__func__, val);
	pixart_r_na_reg(ts->client, PIXART_TOUCH_DELAY_T4_CHARG, (uint8_t *)&val);
	printk("%s: PIXART_TOUCH_DELAY_T4_CHARG = 0x%X\n",__func__, val);
	pixart_r_na_reg(ts->client, PIXART_TOUCH_DELAY_T5, (uint8_t *)&val);
	printk("%s: PIXART_TOUCH_DELAY_T5 = 0x%X\n",__func__, val);
	pixart_r_na_reg(ts->client, PIXART_TOUCH_DELAY_T5_CHARG, (uint8_t *)&val);
	printk("%s: PIXART_TOUCH_DELAY_T5_CHARG = 0x%X\n",__func__, val);
	pixart_r_na_reg(ts->client, PIXART_PROLONG_DELAY, (uint8_t *)&val);
	printk("%s: PIXART_PROLONG_DELAY = 0x%X\n",__func__, val);
	pixart_r_na_reg(ts->client, PIXART_PROLONG_DELAY_CHARG, (uint8_t *)&val);
	printk("%s: PIXART_PROLONG_DELAY_CHARG = 0x%X\n",__func__, val);
	pixart_r_na_reg(ts->client, PIXART_NA_RESUME, (uint8_t *)&val);
	printk("%s: PIXART_NA_RESUME = 0x%X\n",__func__, val);
	pixart_r_na_reg(ts->client, PIXART_ALC_CONTROL_ENABLE_BITS, (uint8_t *)&val);
	printk("%s: PIXART_ALC_CONTROL_ENABLE_BITS = 0x%X\n",__func__, val);
	pixart_r_na_reg(ts->client, PIXART_ALC_THRESH_PCT, (uint8_t *)&val);
	printk("%s: PIXART_ALC_THRESH_PCT = 0x%X\n",__func__, val);
	pixart_r_na_reg(ts->client, PIXART_ALC_THRESH_2, (uint8_t *)&val);
	printk("%s: PIXART_ALC_THRESH_2 = 0x%X\n",__func__, val);
	pixart_r_na_reg(ts->client, PIXART_ALC_THRESH_3, (uint8_t *)&val);
	printk("%s: PIXART_ALC_THRESH_3 = 0x%X\n",__func__, val);
	pixart_r_na_reg(ts->client, PIXART_ALC_FAST_ADAPT_RATE, (uint8_t *)&val);
	printk("%s: PIXART_ALC_FAST_ADAPT_RATE = 0x%X\n",__func__, val);
	pixart_r_na_reg(ts->client, PIXART_ALC_VERY_FAST_ADAPT_RATE, (uint8_t *)&val);
	printk("%s: PIXART_ALC_VERY_FAST_ADAPT_RATE = 0x%X\n",__func__, val);
#endif

	ts->is_set = false;
	if(flash){
		ret = pixart_flash_save(ts);	/* IC Access kinshi */
		if(ret == 0){
			pr_err("%s: Success Backup\n",__func__);
			ts->is_set = true;
		}
		else{
			pr_err("%s: Fail Backup\n",__func__);
			ts->is_set = false;
			return ret;
		}
	}

	return ret;
}

static int pixart_reset_and_wait(struct pixart_data *ts)
{
	const struct pixart_platform_data *pdata = ts->client->dev.platform_data;
	int retry, err;

	PIX_DEV_DBG("%s() is called.\n",__func__);
	if (pdata->reset_hw)
		pdata->reset_hw();

	for(retry = 0; retry < 5; retry++){
		msleep(10);
		pid = pixart_r_reg_byte(ts->client, PIXART_REG_PID);
		if(pid == PIXART_5305_PAP1100QN_PID){	// PIXART_5305_PAP1100QN_PID = 0x89
			break;
		}
		if(retry == 4) {
			pr_err("%s: No hardware detected.\n",__func__);
			return 1;
		}
	}

	retry = 10;
	while (retry > 0) {
		if (pid == PIXART_5305_PAP1100QN_PID || pid == PIXART_5311_PID)		// 0x89 or 0x8B
			err = pixart_r_reg_byte(ts->client, PIXART_REG_BOOT_STAT_GEN2);
		else
			err = pixart_r_reg_byte(ts->client, PIXART_REG_BOOT_STAT);

		if (err & 0x01)
			break;

		/* wait 10ms before check */
		msleep(10);
		retry--;
	}

	if (retry == 0) {
		pr_err("%s: Reset failed\n",__func__);
		return 1;
	}

	return 0;

}

static int pixart_reset_status(struct pixart_data *ts)
{
	int ret = 0;

	PIX_DEV_DBG("%s() is called.\n",__func__);
	pixart_report_clear(ts);
#if 0
	if(ts->config_nv[TS_CHARGE_CABLE].data)
		ret = pixart_set_nv(ts);
	else
#endif
		ret = pixart_set_config(ts);
	if(!ret){
		PIX_DEV_DBG("%s: Success\n",__func__);
	}else{
		pr_err("%s: error\n",__func__);
	}
	return ret;
}

static ssize_t pixart_mode_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{

	struct pixart_data *ts = i2c_get_clientdata(to_i2c_client(dev));
	ssize_t ret = 0;
	uint8_t i, len;
	static uint8_t data[128];

	len = sdata.number;
	/* command 'r sdata.reg sdata.number' */
	switch (sdata.command) {
	case SYSFS_READ:
		sprintf(buf, "	   ");
		for (i = 0; i < 16; i++) {
			sprintf(data, "%2x ", i);
			strcat(buf, data);
		}

		i = sdata.reg[0] & 0xf0;
		for (; i < ((sdata.reg[0] + len + 15) & 0xf0); i++) {
			/* start at 10 20 ... n0, \t = 4 chars */
			if (!(i & 0xf)) {
				sprintf(data, "\n%02x :\t", i);
				strcat(buf, data);
			}
			if (i >= sdata.reg[0] && i < sdata.reg[0] + len) {
				sprintf(data, "%02x ", pixart_r_reg_byte(ts->client, i));
				strcat(buf, data);
			}
			else {
				sprintf(data, "   ");
				strcat(buf, data);
			}
		}

		strcat(buf, "\n");
		ret = strlen(buf) + 1;
		break;

	case SYSFS_READ3N:
		for (i = 0; i < len; i++) {
			pixart_r_reg_nbytes(ts->client, sdata.reg, data, 3);
			sprintf(&data[3], " %02x %02x %02x", data[0],data[1],data[2]);
			strcat(buf, &data[3]);
			if (pixart_debug_message_mask & 0x10)
				pr_debug(MODULE_NAME "triple read: %02x %02x %02x\n",
					data[0],data[1],data[2]);
		}

		ret = strlen(buf) + 1;
		break;

	case SYSFS_WRITE_READ:
		pixart_w_reg_byte(ts->client, sdata.reg[0], sdata.val0);
		pixart_w_reg_byte(ts->client, sdata.reg[1], sdata.val1);
		sprintf(buf, " ");
		for (i = 0; i < len; i++) {
			sprintf(data, "%02x ", pixart_r_reg_byte(ts->client, sdata.reg[2]));
			strcat(buf, data);
		}
		ret = strlen(buf) + 1;
		break;

	case SYSFS_VERSION:
		sprintf(buf, " ");
		sprintf(data, "%s", PIXART_DRIVER_VERSION);
		strcat(buf, data);
		strcat(buf, "\n ");
		break;

	case SYSFS_FIRMWARE:
		sprintf(buf, " ");
		sprintf(data, "%02x", sdata.val0);
		strcat(buf, data);
		strcat(buf, "\n ");
		break;

	case SYSFS_KDBGLEVEL:
		sprintf(buf, " ");
		sprintf(data, "%02x", pixart_debug_message_mask);
		strcat(buf, data);
		strcat(buf, "\n ");
		break;

	default:
		break;
	}

	return ret;
}

static ssize_t pixart_mode_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct pixart_data *ts = i2c_get_clientdata(to_i2c_client(dev));
	static char data[128];
	const char *p;
	char temp;
	int n;

	PIX_DEV_DBG("%s() is called.\n",__func__);
	switch (buf[0]) {
	case 'r':
		sdata.command = SYSFS_READ;
		sscanf(buf, "%c %x %x", data, (unsigned int *) &sdata.reg[0],
				(unsigned int *) &sdata.number);
		PIX_DEV_DBG("%s: read mode: %02x %02x \n",__func__, sdata.reg[0],
				sdata.number);
		break;

	case 'w':
		sdata.command = SYSFS_WRITE;
		p = buf + 1;
		while (sscanf(p, "%x %x%n", (unsigned int *) &sdata.reg[0],
				(unsigned int *) &sdata.data, &n) == 2) {
			p += n;
			pixart_w_reg_byte(ts->client, sdata.reg[0], sdata.data);
			PIX_DEV_DBG("%s: write mode: %02x %02x \n",__func__,
				sdata.reg[0], sdata.data);
		}
		break;

	case 'b':
		p = buf + 1;
		sdata.command = SYSFS_BURST;
		/* find out what address to write to */
		/* %n how many chars consumed this time */
		sscanf(p, "%x%n", (unsigned int *) &sdata.reg[0], &n);
		p += n;
		/* write the data bytes which follow */
		while (sscanf(p, "%x%n", (unsigned int *) &temp, &n) == 1) {
			p += n;
			pixart_w_reg_byte(ts->client, sdata.reg[0], temp);
			PIX_DEV_DBG("%s: burst write %02x %02x \n",__func__, sdata.reg[0], temp);
		}
		break;

	case 'd':
		sdata.command = SYSFS_DISABLE;
		PIX_DEV_DBG("%s: Navigation Disabled\n",__func__);
		disable_count = 10000;
		break;
/*
	case 'e':
		sdata.command = SYSFS_ENABLE;
		PIX_DEV_DBG("%s: Navigation Enabled\n",__func__);
		disable_count = 100;
		break;
*/
	case 't':
		sdata.command = SYSFS_READ3N;
		sscanf(buf, "%c %x %x %x %x", data, (unsigned int *) &sdata.reg[0],
			   (unsigned int *) &sdata.reg[1],
			   (unsigned int *) &sdata.reg[2],
			   (unsigned int *) &sdata.number);
		PIX_DEV_DBG("%s: triple read mode: 0x%02x 0x%02x 0x%02x x 0x%02x times\n"
				,__func__ , sdata.reg[0], sdata.reg[1], sdata.reg[2], sdata.number);
		break;

	case 'm':
		sdata.command = SYSFS_WRITE_READ;
		sscanf(buf, "%c %x %x %x %x %x %x", data, (unsigned int *) &sdata.reg[0],
			   (unsigned int *) &sdata.val0,
			   (unsigned int *) &sdata.reg[1],
			   (unsigned int *) &sdata.val1,
			   (unsigned int *) &sdata.reg[2],
			   (unsigned int *) &sdata.number);
		PIX_DEV_DBG("%s: write 2, read 1 values set\n",__func__);
		break;

	case 'v':
		sdata.command = SYSFS_VERSION;
		pr_info(MODULE_NAME "Ver: %s\n", PIXART_DRIVER_VERSION);
		break;

	case 'f':
		sdata.command = SYSFS_FIRMWARE;
		sdata.val0 = (uint8_t) pixart_init_panel(ts);
		break;

	case 'l':
		sdata.command = SYSFS_KDBGLEVEL;
		sscanf(buf, "%c %x ", data, (unsigned int *)&ts_log_level);
		PIX_DEV_DBG("%s: touch debug level changed : 0x%02x\n",__func__,ts_log_level);
		break;

	case 'p':
		pr_info(MODULE_NAME "trigger kernel panic\n");
		BUG_ON(1);
		break;

	case 'e':
		p = buf + 1;
		sscanf(p, "%x", (int *)&ts_esd_recovery);
		PIX_DEV_DBG("%s: ts_esd_recovery = %x\n",__func__, (int)ts_esd_recovery);
		break;

	default:
		break;
	}

	return count;
}

static DEVICE_ATTR(ctrl, S_IRUGO|S_IWUSR, pixart_mode_show, pixart_mode_store);

static struct attribute *tp_attributes[] = {
	&dev_attr_ctrl.attr,
	NULL
};

static const struct attribute_group tp_attr_group = {
	.attrs = tp_attributes,
};

static int pixart_sysfs_init(struct pixart_data *ts)
{
	int ret;

	/* Register sysfs hooks */
	ret = sysfs_create_group(&ts->client->dev.kobj, &tp_attr_group);
	if (ret) {
		pr_err( "%s: subsystem_register failed\n",__func__);
		ret = -ENOMEM;
		goto err;
	}

	return 0;

err:
	return ret;
}

uint8_t g_sys_fs_command_buf[4] = {0, 0, 0, 0};

static ssize_t pixart_mode_show_fs(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	uint8_t i = 0;

	pr_debug(MODULE_NAME "file read mode  %c %c %x %x\n",
		   g_sys_fs_command_buf[0], g_sys_fs_command_buf[1],
		   g_sys_fs_command_buf[2], g_sys_fs_command_buf[3]);

	for (i = 0; i<sizeof (g_sys_fs_command_buf); i++)
		buf[i] = (char) g_sys_fs_command_buf[i];

	pr_debug(MODULE_NAME "file read mode %x %c %c %x %x\n", i, buf[0], buf[1],
		buf[2], buf[3]);

	return i;
}

static ssize_t pixart_mode_store_fs(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int i = 0;
	int size = sizeof (g_sys_fs_command_buf) / sizeof (g_sys_fs_command_buf[0]);

	struct pixart_data *ts = i2c_get_clientdata(to_i2c_client(dev));

	for (i = 0; i < size; i++)
		g_sys_fs_command_buf[i] = (uint8_t) (buf[i]);

	if (g_sys_fs_command_buf[0] == 'f') {
		if (g_sys_fs_command_buf[1] == 'w') {
			pixart_w_reg_byte(ts->client, (uint8_t) g_sys_fs_command_buf[2],
				(uint8_t) g_sys_fs_command_buf[3]);
			pr_debug(MODULE_NAME "file write mode %c %c %02x %02x \n",
				   g_sys_fs_command_buf[0], g_sys_fs_command_buf[1],
				   g_sys_fs_command_buf[2], g_sys_fs_command_buf[3]);
		} else if (g_sys_fs_command_buf[1] == 'r') {
			g_sys_fs_command_buf[3] = (pixart_r_reg_byte(ts->client,
				(uint8_t) g_sys_fs_command_buf[2]));
			pr_debug(MODULE_NAME "file read mode  %c %c %02x %02x\n",
				   g_sys_fs_command_buf[0], g_sys_fs_command_buf[1],
				   g_sys_fs_command_buf[2], g_sys_fs_command_buf[3]);
		}
	}

	return count;
}

static DEVICE_ATTR(pixart_sysfs_attr,
	S_IWUSR | S_IWGRP | S_IWOTH | S_IRUSR | S_IRGRP | S_IROTH,
	pixart_mode_show_fs, pixart_mode_store_fs);

static struct attribute *tp_attributes_fs[] = {
	& dev_attr_pixart_sysfs_attr.attr,
	NULL
};

static const struct attribute_group tp_attr_group_fs = {
	.attrs = tp_attributes_fs,
};

static int pixart_sysfs_init_fs(struct pixart_data *ts)
{
	int ret;

	/* Register sysfs hooks */
	ret = sysfs_create_group(&ts->client->dev.kobj, &tp_attr_group_fs);
	if (ret) {
		pr_err( "%s: subsystem_register failed\n",__func__);
		ret = -ENOMEM;
		goto err;
	}
	return 0;

err:
	return ret;
}

static int pixart_flash_save(struct pixart_data *ts)
{
	u8 val;
	int i;

	PIX_DEV_DBG("%s() is called.\n",__func__);
	return 0;

	pixart_w_reg_byte(ts->client, PIXART_REG_FLASH_ENABLE, PIXART_FLASH_ENABLE);
	msleep(1);
	pixart_w_reg_byte(ts->client, PIXART_REG_FLASH_CTL_GEN2, PIXART_REG_FLASH_SAVE_CUST_REGS);
	msleep(300);

	i = 0;
	val = pixart_r_reg_byte(ts->client, PIXART_REG_FLASH_CTL_GEN2);
	while((val & 0x01) && (i*10 < PIXART_TIMEOUT_MS_AFTER_FLASH_ENABLE_GEN2)){
		i++;
		mdelay(10);
		val = pixart_r_reg_byte(ts->client, PIXART_REG_FLASH_CTL_GEN2);
	}

	if(i == 50)
		return 1;

	return 0;
}

static int pixart_load_fw(struct i2c_client *client)
{
	int i, len;
	const struct pixart_platform_data *pdata = client->dev.platform_data;
	volatile uint8_t val, ErrorID, BootStat, Stat, fw_id, crc_hi_cs, crc_lo_cs;
	int cs_same; /* Customer Settings block CRC same flag */
	uint8_t *pfw;
	uint8_t desired_fw_id = 0xff;
	int while_count = 0;

	printk("pixart_load_fw is not run!\n");
	new_fw_G2 = 1;
	return 0;

	/*
	 * General overview of firmware loading for AMRI-5000 and AMRI-5100
	 * -------------------------------------------------
	 * disable watchdog
	 * clear boot status register
	 * initiate firmware load by writing to IODL_CTL
	 * send firmware bytes one at a time to IODL_DATA
	 * read boot status register until bits 0,1 are set
	 * write partial reset
	 * delay 1ms
	 * enable watchdog
	 * read REV ID to confirm changes from baseline
	 *
	 * General overview of firmware loading for AMRI-5305
	 * -------------------------------------------------
	 * Steps: 1 - 32 numerically identified prior to each line of C code
	 *
	 */
	pr_info(MODULE_NAME "device version 0x%02x\n", pid);
	switch (pid) {
	case PIXART_5000_PID:
		len = sizeof (firmware5000);
		pfw = firmware5000;
		desired_fw_id = AMRI5000_FW_VERSION;
		pr_info(MODULE_NAME "AMRI5000 detected \n");
		break;

	case PIXART_5100_PID:
		len = sizeof (firmware5100);
		pfw = firmware5100;
		desired_fw_id = AMRI5100_FW_VERSION;
		pr_info(MODULE_NAME "AMRI5100 detected\n");
		break;

	case PIXART_5305_PAP1100QN_PID:
		/* Get h/w ID */
		printk("pixart_load_FW - 5305_PAP1100QN_PID\n");
		val = pixart_r_reg_byte(client, 0x01);
		if (val == 0x10) {
			printk("pixart_load_FW - HW_ID is 0x10\n");
			len = sizeof (firmware5305_10);
			pfw = firmware5305_10;
			desired_fw_id = AMRI5305_ROM10_FW_VERSION;
			pr_info(MODULE_NAME "AMRI5305 0x10 detected \n");
		} else if (val == 0x11) {
			printk("pixart_load_FW - HW_ID is 0x11\n");
			len = sizeof (firmware5305_11);
			pfw = firmware5305_11;
			desired_fw_id = AMRI5305_ROM11_FW_VERSION;
			pr_info(MODULE_NAME "AMRI5305 0x11 detected \n");
		} else if (val == 0x12) {
			printk("pixart_load_FW - HW_ID is 0x12\n");
            len = sizeof (firmware_pap1100qn);
            pfw = firmware_pap1100qn;
            desired_fw_id = PAP1100QN_FW_VERSION;
			pr_info(MODULE_NAME "PAP1100QN 0x12 detected \n");
        } else {
			pr_err(MODULE_NAME "HW version not recognized\n");
			return -1;
		}
		break;

	case PIXART_5311_PID:
		len = sizeof (firmware5311);
		pfw = firmware5311;
		desired_fw_id = AMRI5311_FW_VERSION;
		pr_info(MODULE_NAME "AMRI5311 detected \n");
		break;

	default:
		pr_err(MODULE_NAME "could not detect AMRI5x00\n");
		return -1;
		break;
	}

	if (pid == PIXART_5305_PAP1100QN_PID || pid == PIXART_5311_PID) {
		fw_id = pixart_r_reg_byte(client, PIXART_REG_FW_REVID_GEN2);
/* This flag is used for driver verification only */
#ifndef CONFIG_PIXART_FW_OVERWRITE
		if (fw_id >= desired_fw_id) {
			new_fw_G2 = 0;
			printk("%s: load_fw 1111\n",__func__);
			pr_info(MODULE_NAME "Acceptable FW version already present "
				"(rev 0x%x). No reflash needed.\n", fw_id);
			udelay(PIXART_DELAY_US_AFTER_BOOT_GEN2);
			return 0;
		}
#endif
		new_fw_G2 = 1;
		crc_hi_cs = 0;
		crc_lo_cs = 0;
		cs_same = 1;

		printk("pixart_load_FW - clear Boot_Stat\n");

		/* clear Boot_Stat */
		pixart_w_reg_byte(client, PIXART_REG_BOOT_STAT_GEN2, 0x00);		//
		val = pixart_r_reg_byte(client, PIXART_REG_BOOT_STAT_GEN2);
		pr_info(MODULE_NAME "After clear bootstat = 0x%x\n", val);

		/* To Check if Flash is corrupted or erased, do a reset, anyways */
		pixart_w_reg_byte(client, PIXART_REG_SHUTDOWN, PIXART_SHUTDOWN);
		pixart_w_reg_byte(client, PIXART_REG_SHUTDOWN, PIXART_RESET);

		/* wait 10 ms before polling */
		mdelay(10);

		/* check boot_stat bit 0 going to 1 first before checking for fw id */
		i = 0, val = 0;
		printk("pixart_load_FW - check boot stat bit 0 going to 1\n");
		while (((val & 0x01) == 0) && (i * 10 < PIXART_TIMEOUT_MS_BOOT_GEN2)) {
			i++;
			mdelay(10);
			val = pixart_r_reg_byte(client, PIXART_REG_BOOT_STAT_GEN2);
		}

		if (val & 0x01)
			pr_info(MODULE_NAME "correct bootstat after reset in %d ms\n",
				i * 10);
		else
			pr_info(MODULE_NAME "timeout waiting for correct bootstat after "
				"reset. Got boot_stat = 0x%x i = 0x%x.\n", val, i);

		/* If fw_id is still 0x00, Flash is corrupted...go to initial
			download procedure */
		fw_id = pixart_r_reg_byte(client, PIXART_REG_FW_REVID_GEN2);
		if (fw_id == 0) {
			pr_info(MODULE_NAME "flash is corrupted/erased..going to "
				"flash_init_dl\n");
			goto flash_init_dl;
		}

		pr_info(MODULE_NAME "Establish new firwmare defaults after "
			"next reset\n");

		printk("pixart_load_FW - Enable the Flash\n");
		/* 1.  Enable the Flash for erase/program by setting FLASH_ENABLE
			(0x7C) to Enable */
		/* Erase/Program (0xFE) */
		pixart_w_reg_byte(client, PIXART_REG_FLASH_ENABLE, PIXART_FLASH_ENABLE);

		/* 2.  Delay 1ms before continuing. */
		/* 1ms delay after flash enable */
		msleep(1);

		/* 3.  Write 0x27 with 0x02 to disable restoring customer settings
			from Flash */
		/* after the next reset, allowing new firmware defaults to be
			established. */
		pixart_w_reg_byte(client, PIXART_REG_FLASH_CTL_GEN2,
			PIXART_REG_FLASH_NO_SAVE_CUST_REGS);

		/* 4.  Delay for 1 ms ( for fw >= 0x06 ) or 100 ms (for fw < 0x06)
			before continuing. */
		if (fw_id <= 0x05 || fw_id == 0xff)
			msleep(100);
		else
			msleep(1);

		/* 5. Poll bit 0 of register 0x27 every 10ms and wait for it to be
			reset to 0 (this will */
		/* take about 70 ms, 50 for the page erase and 20 ms to re-program the
			page), indicating */
		/* AMRI-5305 is ready to continue. */
		i = 0;
		val = pixart_r_reg_byte(client, PIXART_REG_FLASH_CTL_GEN2);
		while ((val & 0x01) && (i * 10 < PIXART_TIMEOUT_MS_AFTER_FLASH_ENABLE_GEN2)) {
			i++;
			msleep(10);
			val = pixart_r_reg_byte(client, PIXART_REG_FLASH_CTL_GEN2);
		}

		if ((val & 0x01)) {
			printk("pixart_load_FW - Error Check 0x0D\n");
			/* 6.  Read register ERROR_ID (0x0D). If not zero then an error
				occurred. */
			ErrorID = pixart_r_reg_byte(client, PIXART_REG_ERROR_ID_GEN2);
			BootStat = pixart_r_reg_byte(client, PIXART_REG_BOOT_STAT_GEN2);
			Stat = pixart_r_reg_byte(client, PIXART_REG_STATUS_GEN2);
			fw_id = pixart_r_reg_byte(client, PIXART_REG_FW_REVID_GEN2);
			pr_info(MODULE_NAME "Before flash_ctl poll routine. val = 0x%x, "
				"ErrorID = 0x%x, BootStat = 0x%x  i = 0x%x Status = 0x%x "
				"fw_id = 0x%x\n", val, ErrorID, BootStat, i, Stat, fw_id);
		} else
			pr_info(MODULE_NAME " Flash fw  block erase complete in "
				"%d ms\n", i);


		/* 7.  Write 0xAA to register SHUTDOWN (0x7A) to put AMRI-5305 on
			standby. */
		pixart_w_reg_byte(client, PIXART_REG_SHUTDOWN, PIXART_SHUTDOWN);


		/* 8.  Write 0xCC to register SHUTDOWN to reboot AMRI-5305 to
			run from ROM. */
		/* To write to Flash, AMRI-5305 needs to be executing from ROM. */
		pixart_w_reg_byte(client, PIXART_REG_SHUTDOWN, PIXART_RESET_TO_ROM_GEN2);


		/* 9.  To Verify AMRI-5305 is running from ROM, read register FW_REV_ID
			(0x02). */
		/* The return value should be 0x00. Any other value indicates that
			AMRI-5305 is */
		/* still running from Flash. */
		/* check boot_stat bit 0 going to 1 first before checking for fw id */
		i = 0, val = 0;
		while (((val & 0x01) == 0) && (i * 10 < PIXART_TIMEOUT_MS_BOOT_GEN2)) {
			i++;
			mdelay(10);
			val = pixart_r_reg_byte(client, PIXART_REG_BOOT_STAT_GEN2);
		}

		if (!(val & 0x01)) {
			pr_info(MODULE_NAME "timeout waiting for correct bootstat after "
				"reset to ROM. Got 0x%x instead.\n", val);
			return -1;
		} else {
			pr_info(MODULE_NAME "correct bootstat after reset to ROM in %d "
				"ms\n", i * 10);
			fw_id = pixart_r_reg_byte(client, PIXART_REG_FW_REVID_GEN2);
			if (fw_id != 00) {
				pr_info(MODULE_NAME "step-9: did not reset to ROM. Got Fw "
					"version: 0x%x instead.\n", val);
				return 0;
			}
		}

		/* 10. Write 0xAD to register WD_DISABLE (0x7D), which disables the
			watchdog. */

flash_init_dl:
		printk("pixart_load_FW - flash_init_dl\n");
		pixart_w_reg_byte(client, PIXART_REG_WD_DISABLE, PIXART_WD_DISABLE);

		/* 11. Enable the Flash for erase/program by setting FLASH_ENABLE
			(0x7C) to Enable Erase/Program (0xFE). */
		pixart_w_reg_byte(client, PIXART_REG_FLASH_ENABLE, PIXART_FLASH_ENABLE);


		/* 12.  Delay 1ms before continuing. */
		msleep(1);

		/* 13. (Optional) Write 0x00 to 0x03 to clear register BOOT_STAT.
			This ensures correct reads in step 19. */
		pixart_w_reg_byte(client, PIXART_REG_BOOT_STAT_GEN2, 0x00);


		/* 14. Write 0x2E (Flash Configuration type) to register DOWNLOAD_CTL
			(0x0A) to enable the Flash download routine. */
		pixart_w_reg_byte(client, PIXART_REG_IODL_CTL_GEN2, PIXART_ENABLE_DL_GEN2);

		/* 15. Delay for 1 ms before continuing. */
		msleep(1);

		/* soon the PIXART_REG_IODL_CTL_GEN2 register MSB busy bit will be
			set, but don't check it until after at least 10ms, just assume
			it is already busy */

		/* 16. Poll bit 7 of register 0x0A every 10 ms and wait for it to be
			reset to 0 (this will take about 1 sec), indicating AMRI-5305
			is ready to receive download data. */
		val = 0x80, i = 0;
		while (((val & 0x80) == 0x80)
				&& (i * 10 < PIXART_TIMEOUT_MS_FW_INIT_GEN2)) {
			i++;
			msleep(10);
			val = pixart_r_reg_byte(client, PIXART_REG_IODL_CTL_GEN2);
		}

		if ((val & 0x80) == 0x80) {
			ErrorID = pixart_r_reg_byte(client, PIXART_REG_ERROR_ID_GEN2);
			pr_info(MODULE_NAME "REG_IODL_CTL_GEN2 timeout waiting for MSB "
				"clear. Got 0x%x instead, Error_ID = 0x%x\n", val, ErrorID);
		} else
			pr_info(MODULE_NAME "REG_IODL_CTL_GEN2 done erasing"
				"(value = 0x%x ) in >~%d ms \n", val, i * (10));

		/* 17. Read register ERROR_ID (0x0D) and discard value to clear it. */
		val = pixart_r_reg_byte(client, PIXART_REG_ERROR_ID_GEN2);

		/* 18. Write the entire firmware file, byte by byte, to register
			0x0B (DOWNLOAD_DATA). */

		i = 0;
		printk("pixart_load_FW - Here is Write FW\n");
		while (i < len) {
			pixart_w_reg_byte(client, PIXART_REG_IODL_DATA_GEN2, *(pfw + i));
			/* 5311 firmware has two 128bytes buffer and do the flash-write
				every 128bytes 128 x 20us = 2.56ms, while wite 128bytes flash
				 need 1.24ms. so we can optimize around 1ms for every
				128bytes */
			udelay(PIXART_DELAY_US_BETWEEN_FLASH_WRITES);
			i++;
		}

		/* 19. After completion of all bytes written, read register 0x03 until
			bits 0 and 1 are set. Start over on timeout or error. */
		i = 0, val = 0;
		while (((val & 0x03) == 0) && (i * 10 < PIXART_TIMEOUT_MS_BOOT_GEN2)) {
			i++;
			mdelay(10);
			val = pixart_r_reg_byte(client, PIXART_REG_BOOT_STAT_GEN2);
		}

		/* Bit 1 - is load succesfull, Bit 2  - load failure */
		if ((val & 0x04) || !(val & 0x02)) {
			ErrorID = pixart_r_reg_byte(client, PIXART_REG_ERROR_ID_GEN2);
			pr_info(MODULE_NAME "post fw DLD BOOT_STAT timeout waiting for "
				"correct bit values. Got 0x%x instead, ErrorID = 0x%x\n",
				val, ErrorID);
		} else
			pr_info(MODULE_NAME "Post Fw DL BOOT_STAT success (value"
				" = 0x%x ) in %d ms \n", val, i * 10);

		/* 20. Reset AMRI-5305 using pin 5 (NRST_NSHD) -- optional for client
			I/F */
		/* 21. Wait for reset to complete; poll 0x03, BOOT_STAT until bit 0
			is set (Boot completed). */
		i = 0, val = 0;
		while (((val & 0x01) == 0) && (i * 10 < PIXART_TIMEOUT_MS_BOOT_GEN2)) {
			i++;
			mdelay(10);
			val = pixart_r_reg_byte(client, PIXART_REG_BOOT_STAT_GEN2);
		}

		ErrorID = pixart_r_reg_byte(client, PIXART_REG_ERROR_ID_GEN2);
		if (!(val & 0x01)) {
			pr_info(MODULE_NAME " timeout waiting for correct bootstat after "
			"reset.  Got 0x%x instead, ErrorID = 0x%02x\n", val, ErrorID);

		} else
			pr_info(MODULE_NAME " correct bootstat after reset in %d ms "
				"(ErrorID= 0x%02x)\n", i * 10,ErrorID);


		/* 22. Read 0x02 to confirm firmware revision ID. */
		fw_id = pixart_r_reg_byte(client, PIXART_REG_FW_REVID_GEN2);
		if (fw_id == 0) {
			pr_info(MODULE_NAME "FW rev check failed! Wanted 0x%x, got "
				"0x%02x\n", desired_fw_id, fw_id);
			pr_info(MODULE_NAME "will be using native ROM. Performance "
				"may not be optimal. \n");
			return 0;
		} else
			pr_info(MODULE_NAME "FW updated & rebooted w/ default flash "
				"settings. FW rev = 0x%02x\n", fw_id);

		/* 23. check Customer Settings CRC */
		/* : FLC1 - Save Customer Registers to Flash */
		printk("pixart_load_FW - Check_CRC_23\n");
		pixart_w_reg_byte(client, PIXART_REG_FLASH_CRC_CTL_GEN2, 0x02);
		if (fw_id < 0x06)
			msleep(100);

		/* wait for bit 0 of PIXART_REG_FLASH_CRC_CTL to clear */
		i = 0;
		while ((val & 0x01) || i * 10 < 20000) {
			val = pixart_r_reg_byte(client, PIXART_REG_FLASH_CRC_CTL_GEN2);
			udelay(10);
			i++;
		}

		crc_hi_cs = pixart_r_reg_byte(client, PIXART_REG_CRC_HI_GEN2);
		crc_lo_cs = pixart_r_reg_byte(client, PIXART_REG_CRC_LO_GEN2);

		if (!(val & 0x01))
			pr_info(MODULE_NAME "crc_hi_cs = 0x%x crc_lo_cs = 0x%x \n",
				   crc_hi_cs, crc_lo_cs);
		else
			pr_info(MODULE_NAME "timeout waiting for CRC_CTL bit 0 to "
				"clear: crc_hi_cs = 0x%x crc_lo_cs = 0x%x \n",
				crc_hi_cs, crc_lo_cs);


		if (crc_hi_cs != CRC_HI_CS_REGS || crc_lo_cs != CRC_LO_CS_REGS) {
			cs_same = 0;
			pr_info(MODULE_NAME "Customer Settings block CRC mismatch.\n");

			/* Write 0x27 with 0x02 to disable restoring customer settings
				from Flash after the next reset, allowing new firmware
				defaults to be established. */
			pixart_w_reg_byte(client, PIXART_REG_FLASH_CTL_GEN2,
				PIXART_REG_FLASH_NO_SAVE_CUST_REGS);

			/* Delay for 1 ms ( for fw >= 0x06 ) or 100 ms (for fw < 0x06)
				before continuing. */
			if (fw_id <= 0x05 || fw_id == 0xff)
				msleep(100);
			else
				msleep(1);

			/* Poll bit 0 of register 0x27 every 10ms and wait for it to
				be reset to 0 (this will take about 70 ms, 50 for the page
				erase and 20 ms to re-program the page), indicating AMRI-5305
				is ready to continue. */
			i = 0;
			val = pixart_r_reg_byte(client, PIXART_REG_FLASH_CTL_GEN2);
			while ((val & 0x01)
					&& (i * 10 < PIXART_TIMEOUT_MS_AFTER_FLASH_ENABLE_GEN2)) {
				i++;
				msleep(10);
				val = pixart_r_reg_byte(client, PIXART_REG_FLASH_CTL_GEN2);
			}

			if ((val & 0x01)) {
				/* Read register ERROR_ID (0x0D). If not zero then an error
					occurred. */
				ErrorID = pixart_r_reg_byte(client, PIXART_REG_ERROR_ID_GEN2);
				BootStat = pixart_r_reg_byte(client, PIXART_REG_BOOT_STAT_GEN2);
				Stat = pixart_r_reg_byte(client, PIXART_REG_STATUS_GEN2);
				fw_id = pixart_r_reg_byte(client, PIXART_REG_FW_REVID_GEN2);
				pr_info(MODULE_NAME "Before flash_ctl poll routine. val = 0x%x,"
					" ErrorID = 0x%x, BootStat = 0x%x  i = 0x%x Status = 0x%x"
					" fw_id = 0x%x\n", val, ErrorID, BootStat, i, Stat, fw_id);
			} else
				pr_info(MODULE_NAME " Flash fw  block erase complete in "
					"%d ms\n", i * 10);
		}

		/* 24. Reset AMRI-5305 */
		pixart_w_reg_byte(client, PIXART_REG_SHUTDOWN, PIXART_SHUTDOWN);
		pixart_w_reg_byte(client, PIXART_REG_SHUTDOWN, PIXART_RESET);
		i = 0, val = 0;
		/* check boot_stat bit 0 going to 1 first */
		while (((val & 0x01) == 0) && (i * 10 < PIXART_TIMEOUT_MS_BOOT_GEN2)) {
			i++;
			mdelay(10);
			val = pixart_r_reg_byte(client, PIXART_REG_BOOT_STAT_GEN2);
		}
		if (val & 0x01)
			pr_info(MODULE_NAME "correct bootstat after reset in %d ms\n",
				i * 10);
		else {
			pr_info(MODULE_NAME "timeout waiting for correct bootstat after "
				"reset. Got boot_stat = 0x%x i = 0x%x.\n", val, i);
			return 0;
		}


		/* 25. Restore any Drive/Sense map values & customer settings */
		/* Steps a) through l) are for downloading Drive/Sense map to
			AMRI-5305 */
		/* a)	Write 0xAD to register WD_DISABLE (0x7D) which disables
			the watchdog. */
		pixart_w_reg_byte(client, PIXART_REG_WD_DISABLE, PIXART_WD_DISABLE);

		/* b)	Write 0x00 to register 0x03 to clear register BOOT_STAT.
			This ensures correct reads in step i) */
		pixart_w_reg_byte(client, PIXART_REG_BOOT_STAT_GEN2, 0x00);

		/* c) Write desired download control command to register DOWNLOAD_CTL
			(0x0A) to enable the appropriate download routine. */
		/*  Please refer to above DOWNLOAD_CTL register description for the
			different values of the control command of */
		/*  0x29, 0x2A, 0x2C or 0x2D for each type of data download. */
		/*  0x2c: initiate Sense/Drive Data Download */
		pixart_w_reg_byte(client, PIXART_REG_IODL_CTL_GEN2, 0x2C);

		/* d) Delay for 1ms before continuing.. */
		mdelay(1);

		/* e) Poll bit 7 of register 0x0A and wait for it to be reset to 0,
			indicating AMRI-5305 is ready to receive download data. */
		/* If bit 7 not reset to 0 within a 2000 ms timeout, an error has
			occurred. */
		val = 0x80, i = 0;
		while (((val & 0x80) == 0x80)
				&& (i * 10 < PIXART_TIMEOUT_MS_FW_INIT_GEN2)) {
			i++;
			msleep(10);
			val = pixart_r_reg_byte(client, PIXART_REG_IODL_CTL_GEN2);
		}

		if ((val & 0x80) == 0x80)
			pr_info(MODULE_NAME "REG_IODL_CTL_GEN2 timeout waiting for "
				"MSB clear.  Got 0x%x instead.\n", val);
		else
			pr_info(MODULE_NAME "REG_IODL_CTL_GEN2 done erasing"
				"(value = 0x%x ) in >~%d ms \n", val, i * (10));

		/* f) delay by 100us before continuiing */
		udelay(100);

		/* g) Read X_ERROR_ID (0x0D) and discard value to clear it.
			A non-zero value will prevent successful download. */
		pixart_r_reg_byte(client, PIXART_REG_ERROR_ID_GEN2);

		/* h)(unneccessary) If sense/drive download (0x2C) or delta
			compensation data download (0x2D), clear bit 7 of BOOT_STAT
			(register 0x03) in preparation for later read of this bit
			in step l). */
		pixart_w_reg_byte(client, PIXART_REG_BOOT_STAT_GEN2, 0x00);


		/* i)Write all the bytes in the CONFIGUATION FILE to register
			DOWNLOAD_DATA (0x0B). For each byte written, maintain a minimum
			delay time of 10us or more in between. */
		printk("pixart_load_FW - Write Configuration file\n");
		/* rows are swapped */
		pixart_w_reg_byte(client, PIXART_REG_IODL_DATA_GEN2, 0x10); // reset reference frame
		pixart_w_reg_byte(client, PIXART_REG_IODL_DATA_GEN2, 0x0C); // 12 rows
		pixart_w_reg_byte(client, PIXART_REG_IODL_DATA_GEN2, 0x14); // 20 columns

		/* rows are swapped */
		pixart_w_reg_byte(client, PIXART_REG_IODL_DATA_GEN2, 0xab); // row 10 & 11
		pixart_w_reg_byte(client, PIXART_REG_IODL_DATA_GEN2, 0x89); // row 8 & 9
		pixart_w_reg_byte(client, PIXART_REG_IODL_DATA_GEN2, 0x67); // row 6 & 7
		pixart_w_reg_byte(client, PIXART_REG_IODL_DATA_GEN2, 0x45); // row 4 & 5
		pixart_w_reg_byte(client, PIXART_REG_IODL_DATA_GEN2, 0x23); // row 2 & 3
		pixart_w_reg_byte(client, PIXART_REG_IODL_DATA_GEN2, 0x01); // row 0 & 1

		/* columns are mapped alternate odd/even */
		pixart_w_reg_byte(client, PIXART_REG_IODL_DATA_GEN2, 0x00); // col# 0 -> AMRI-5305 col 0
		pixart_w_reg_byte(client, PIXART_REG_IODL_DATA_GEN2, 0x02);// col# 2 -> AMRI-5305 col 1
		pixart_w_reg_byte(client, PIXART_REG_IODL_DATA_GEN2, 0x04);// col# 4 -> AMRI-5305 col 2
		pixart_w_reg_byte(client, PIXART_REG_IODL_DATA_GEN2, 0x06);// col# 6 -> AMRI-5305 col 3
		pixart_w_reg_byte(client, PIXART_REG_IODL_DATA_GEN2, 0x08);// col# 8 -> AMRI-5305 col 4
		pixart_w_reg_byte(client, PIXART_REG_IODL_DATA_GEN2, 0x0A);// col# 10 ->  AMRI-5305 col 5
		pixart_w_reg_byte(client, PIXART_REG_IODL_DATA_GEN2, 0x0C);// col# 12 -> AMRI-5305 col 6
		pixart_w_reg_byte(client, PIXART_REG_IODL_DATA_GEN2, 0x0E);// col#14 -> AMRI-5305 col 7
		pixart_w_reg_byte(client, PIXART_REG_IODL_DATA_GEN2, 0x10);// col# 16 -> AMRI-5305 col 8
		pixart_w_reg_byte(client, PIXART_REG_IODL_DATA_GEN2, 0x12);// col# 18 -> AMRI-5305 col 9
		pixart_w_reg_byte(client, PIXART_REG_IODL_DATA_GEN2, 0x01);// col# 1 -> AMRI-5305 col 10
		pixart_w_reg_byte(client, PIXART_REG_IODL_DATA_GEN2, 0x03);// col# 3 -> AMRI-5305 col 11
		pixart_w_reg_byte(client, PIXART_REG_IODL_DATA_GEN2, 0x05);// col# 5 -> AMRI-5305 col 12
		pixart_w_reg_byte(client, PIXART_REG_IODL_DATA_GEN2, 0x07);// col# 7 -> AMRI-5305 col 13
		pixart_w_reg_byte(client, PIXART_REG_IODL_DATA_GEN2, 0x09);// col# 9 -> AMRI-5305 col 14
		pixart_w_reg_byte(client, PIXART_REG_IODL_DATA_GEN2, 0x0B);// col# 11 -> AMRI-5305 col 15
		pixart_w_reg_byte(client, PIXART_REG_IODL_DATA_GEN2, 0x0D);// col# 13 -> AMRI-5305 col 16
		pixart_w_reg_byte(client, PIXART_REG_IODL_DATA_GEN2, 0x0F);// col# 15 -> AMRI-5305 col 17
		pixart_w_reg_byte(client, PIXART_REG_IODL_DATA_GEN2, 0x11); // col# 17 -> AMRI-5305 col 18
		pixart_w_reg_byte(client, PIXART_REG_IODL_DATA_GEN2, 0x13); // col# 19 -> AMRI-5305 col 19

		/* j) After completion of all bytes written, read register 0x03 until
			bit 6 is set (1) and bit 5 is reset (0). */
		/* Start over on timeout or error.  A timeout of 500 ms is sufficient
			in determining if an error occurred. A detected error will be
			reported in X_ERROR_ID (0x0D).  If no timeout or error, continue
			to step 10. */
		/* poll boot_stat bit 6 for data down load completion */
		/* wait for either bit 6 or 5 to set (bit 6 = success, bit 5 = fail) */
		i = 0, val = 0;
		while (((val & 0x60) == 0x00)
				&& (i * 10 < PIXART_TIMEOUT_MS_BOOT_GEN2)) {
			i++;
			mdelay(10);
			val = pixart_r_reg_byte(client, PIXART_REG_BOOT_STAT_GEN2);
		}

		/* Bit 6 - load is succesfull */
		if (val & 0x40)
			pr_info(MODULE_NAME "Post Data DL BOOT_STAT success "
				"(value = 0x%x ) in %d ms \n", val, i * 10);
		else {
			ErrorID = pixart_r_reg_byte(client, PIXART_REG_ERROR_ID_GEN2);
			pr_info(MODULE_NAME "post Data DLD BOOT_STAT timeout or error "
				"waiting for correct bit values. Got 0x%x instead, Error_ID"
				" = 0x%x\n", val, ErrorID);
		}

		/* k)	Write 0x00 to 0x7D which enables the watchdog. */
		pixart_w_reg_byte(client, PIXART_REG_WD_DISABLE, PIXART_WD_ENABLE);

		/* following restores customer settings values */
		pixart_w_reg_byte(client, PIXART_REG_ROW_GEN2, CTP_ROWS); // 12
		pixart_w_reg_byte(client, PIXART_REG_COL_GEN2, CTP_COLS); // 20
		pixart_w_reg_byte(client, PIXART_REG_WIN_MAX_Y_GEN2, CTP_ROWS-1); // last row # = 11
		pixart_w_reg_byte(client, PIXART_REG_WIN_MAX_X_GEN2, CTP_COLS-1); // last col # = 19
		pixart_w_reg_byte(client, PIXART_REG_REPORT_POINTS_GEN2, PIXART_TOUCH_NUMBER_GEN2);
		pixart_w_reg_byte(client, PIXART_REG_MOTION_REPORT_CTL_GEN2, PIXART_MOTION_DISABLE_HOVER_GEN2);
		pixart_w_reg_byte(client, PIXART_REG_TOUCH_REF_LO_GEN2, PIXART_VALUE_TOUCH_REF_LO_GEN2);
		pixart_w_reg_byte(client, PIXART_REG_TOUCH_REF_HI_GEN2, PIXART_VALUE_TOUCH_REF_HI_GEN2);
		pixart_w_reg_byte(client, PIXART_REG_ORIENTATION_GEN2, pdata->orient);
		pixart_w_reg_byte(client, PIXART_REG_HEGHT_HI_GEN2, (pdata->y_size >> 8)& 0xFF);
		pixart_w_reg_byte(client, PIXART_REG_HEIGHT_LO_GEN2, pdata->y_size & 0xFF);
		pixart_w_reg_byte(client, PIXART_REG_WIDTH_HI_GEN2, (pdata->x_size >> 8)& 0xFF);
		pixart_w_reg_byte(client, PIXART_REG_WIDTH_LO_GEN2, pdata->x_size & 0xFF);

		/* save settings to flash */
		pr_info(MODULE_NAME "saving Row/Col mappings, customer settings, "
			"and new FW defaults \n");

		/* 26. Enable the Flash for erase/program by setting FLASH_ENABLE
			(0x7C) to Enable Erase/Program (0xFE). */
		pixart_w_reg_byte(client, PIXART_REG_FLASH_ENABLE, PIXART_FLASH_ENABLE);

		/* 27. Delay 1ms before continuing. */
		msleep(1);

		/* 28. Write 0x27 with 0x86 (save D/S Map & customer settings and
			restore them from Flash after the next reset). */
		pixart_w_reg_byte(client, PIXART_REG_FLASH_CTL_GEN2,
			PIXART_REG_FLASH_SAVE_CUST_REGS_DS_MAP);

		/* 29. Delay for 1 ms ( for fw >= 0x06 ) or 100 ms per bit
			(for fw < 0x06) before continuing. */
		/* 100 ms per bit delay(total delay = 2x100ms) */
		if (fw_id <= 0x05 || fw_id == 0xff)
			msleep(200);
		else
			msleep(1);

		/* 30. Poll bit 0 of register 0x27 every 10ms and wait for it to be
			reset to 0 (this will take about 70 ms, 50 for the page erase
			and 20 ms to re-program the page), indicating */
		/* AMRI-5305 is ready to continue. */
		i = 0;
		while ((val & 0x01)
				&& (i * 10 < PIXART_TIMEOUT_MS_AFTER_FLASH_ENABLE_GEN2)) {
			i++;
			mdelay(10);
			val = pixart_r_reg_byte(client, PIXART_REG_FLASH_CTL_GEN2);
		}

		/* Enable Watchdog... */
		pixart_w_reg_byte(client, PIXART_REG_WD_DISABLE, PIXART_WD_ENABLE);

		if (val & 0x01) {
			/* 31. Read register ERROR_ID (0x0D). If not zero then
				an error occurred. */
			ErrorID = pixart_r_reg_byte(client, PIXART_REG_ERROR_ID_GEN2);
			BootStat = pixart_r_reg_byte(client, PIXART_REG_BOOT_STAT_GEN2);
			Stat = pixart_r_reg_byte(client, PIXART_REG_STATUS_GEN2);
			fw_id = pixart_r_reg_byte(client, PIXART_REG_FW_REVID_GEN2);
			pr_info(MODULE_NAME "timeout waiting for flash block write. "
				"Status = 0x%x Fw_id = 0x%x ErrorID = 0x%x, i = 0x%x "
				"Flash-Ctl = 0x%x \n", Stat, fw_id, ErrorID, i, val);
		} else
			pr_info(MODULE_NAME "Flash block write complete in %d ms\n", i);


		/* l) If sense/drive download (0x2C) or delta compensation data
			download (0x2D), poll bit 7 of BOOT_STAT (register 0x03) for a
			to confirm navigation ready. This could take up to
			1500 ms maximum. This should be done at the end before succesfull
			download return */

		/* poll boot_stat bit 7 to be 1 for Nav ready */
		i = 0, val = 0;
		/* wait for either bit 6 or 5 to set (bit 6 = success, bit 5 = fail) */
		while (((val & 0x80) == 0x00)
				&& (i * 10 < PIXART_TIMEOUT_MS_FW_INIT_GEN2)) {
			i++;
			mdelay(10);
			val = pixart_r_reg_byte(client, PIXART_REG_BOOT_STAT_GEN2);
		}


		/* Bit 7 - Nav is ready! */
		if (val & 0x80)
			pr_info(MODULE_NAME "Post Data DL BOOT_STAT success "
				"(value = 0x%x ) in %d ms \n", val, i * 10);
		else {
			ErrorID = pixart_r_reg_byte(client, PIXART_REG_ERROR_ID_GEN2);
			pr_info(MODULE_NAME "post Data DLD BOOT_STAT timeout or error"
				" waiting for correct Nav Ready bit. Got 0x%x instead, "
				"ErrorID = 0x%x\n", val, ErrorID);
		}

		return 0;

	} else {
		printk("pixart_load_FW - Old 5100 and 5200 device\n");
		/* the following is for 5100 & 5200 devices, only... */
		pixart_w_reg_byte(client, PIXART_REG_WD_DISABLE, PIXART_WD_DISABLE);
		pixart_w_reg_byte(client, PIXART_REG_BOOT_STAT, 0x00);
		pixart_w_reg_byte(client, PIXART_REG_IODL_CTL, PIXART_ENABLE_DL);

		i = 0;
		while (i < len) {
			pixart_w_reg_byte(client, PIXART_REG_IODL_DATA, *(pfw + i));
			i++;
		}

		i = 0;
		val = pixart_r_reg_byte(client, PIXART_REG_BOOT_STAT);
		while (((val & 0x03) != 0x03) && i < PIXART_TIMEOUT_MS_FW_INIT_GEN1) {
			i++;
			udelay(1000);
			val = pixart_r_reg_byte(client, PIXART_REG_BOOT_STAT);
			if (i >= PIXART_TIMEOUT_MS_FW_INIT_GEN1)
				pr_info(MODULE_NAME "REG_BOOT_STAT timeout waiting for "
					"correct value\n");
		}

		val = 0;
		val = pixart_r_reg_byte(client, PIXART_REG_PID);
		/* too much delay while inquiring rev_ID - why is being read > 1
			time? */
		while (val != desired_fw_id) {
			udelay(5);
			val = pixart_r_reg_byte(client, PIXART_REG_REV_ID);
			while_count++;
			if (while_count >= 0x2ffff) {
				pr_info(MODULE_NAME " firmware check failed!\n");
				break;
			}
		}
		pr_info(MODULE_NAME " firmware version %x!\n", val);

		return 0;
	}

	/* should not be able to get here */
	return -1;
}

static int pixart_init_panel(struct pixart_data *ts)
{
	int i = 0;
	const struct pixart_platform_data *pdata = ts->client->dev.platform_data;
	volatile uint8_t val = 0, ErrorID, BootStat, Stat, fw_id,
		crc_hi_ds, crc_lo_ds, crc_hi_cs, crc_lo_cs;
	int cs_same = 1; /* Customer Settings block CRC same flag */
	int ds_same = 1; /* Drive/Sense map block CRC same flag */
//	u8 crc_test;

	if (pixart_load_fw(ts->client) != 0)
		return -1;

	switch (pid) {
	case PIXART_5311_PID:
		printk("Pix_init_panel FW is 0x8B\n");
		fw_id = pixart_r_reg_byte(ts->client, PIXART_REG_FW_REVID_GEN2);
		pixart_w_reg_byte(ts->client, PIXART_REG_HEGHT_HI_GEN2, (pdata->y_size >> 8)& 0xFF);
		pixart_w_reg_byte(ts->client, PIXART_REG_HEIGHT_LO_GEN2, pdata->y_size & 0xFF);
		pixart_w_reg_byte(ts->client, PIXART_REG_WIDTH_HI_GEN2, (pdata->x_size >> 8)& 0xFF);
		pixart_w_reg_byte(ts->client, PIXART_REG_WIDTH_LO_GEN2,  pdata->x_size & 0xFF);
		break;

	case PIXART_5305_PAP1100QN_PID:
		printk("Pix_init_panel FW is 0x89. Normal Route\n");
		if (!(new_fw_G2)) {
			printk("pixart ner_fw_G2 Route\n");
			crc_hi_ds = crc_lo_ds = crc_hi_cs = crc_lo_cs = 0;
			fw_id = pixart_r_reg_byte(ts->client, PIXART_REG_FW_REVID_GEN2);
			pr_info(MODULE_NAME "New FW load not detected..may update "
				"customer settings & D/s maps \n");

			/* check if DS Map in flash is corrupted via CRC registers */
			pixart_w_reg_byte(ts->client, PIXART_REG_FLASH_CRC_CTL_GEN2, 0x04);
			/* delay 100 ms ( only needed for fw < 0x06) before polling
				bit 0 of CRC_CTL */
			if (fw_id < 0x06)
				msleep(100);

			/* wait for bit 0 of PIXART_REG_FLASH_CRC_CTL to clear */
			/* timeout after 20 ms */
			i = 0, val = 0x01;
			while ((val & 0x01) || i * 10 < 20000) {
				val = pixart_r_reg_byte(ts->client,
					PIXART_REG_FLASH_CRC_CTL_GEN2);
				udelay(10);
				i++;
			}

			crc_hi_ds = pixart_r_reg_byte(ts->client, PIXART_REG_CRC_HI_GEN2);
			crc_lo_ds = pixart_r_reg_byte(ts->client, PIXART_REG_CRC_LO_GEN2);

			if (!(val & 0x01))
				pr_info(MODULE_NAME "crc_hi_ds = 0x%x crc_lo_ds = 0x%x \n",
					   crc_hi_ds, crc_lo_ds);
			else
				pr_info(MODULE_NAME "timeout waiting for CRC_CTL bit 0 to "
					"clear: crc_hi_ds = 0x%x crc_lo_ds = 0x%x \n",
					crc_hi_ds, crc_lo_ds);

			if (fw_id < 0x06)
				msleep(100);

			/* check for CS regs map crc */
			pixart_w_reg_byte(ts->client, PIXART_REG_FLASH_CRC_CTL_GEN2, 0x02);

			if (fw_id < 0x06)
				msleep(100);

			/* wait for bit 0 of PIXART_REG_FLASH_CRC_CTL to clear */
			i = 0, val = 0x01;
			while ((val & 0x01) || i * 10 < 20000) {
				val = pixart_r_reg_byte(ts->client,
					PIXART_REG_FLASH_CRC_CTL_GEN2);
				udelay(10);
				i++;
			}

			crc_hi_cs = pixart_r_reg_byte(ts->client, PIXART_REG_CRC_HI_GEN2);
			crc_lo_cs = pixart_r_reg_byte(ts->client, PIXART_REG_CRC_LO_GEN2);
			if (!(val & 0x01))
				pr_info(MODULE_NAME "crc_hi_cs = 0x%x crc_lo_cs = 0x%x \n",
					   crc_hi_cs, crc_lo_cs);
			else
				pr_info(MODULE_NAME "timeout waiting for CRC_CTL bit 0 to "
					"clear: crc_hi_cs = 0x%x crc_lo_cs = 0x%x \n",
					crc_hi_cs, crc_lo_cs);


			if (crc_hi_ds != CRC_HI_DS_MAP || crc_lo_ds != CRC_LO_DS_MAP) {
				/* D/S map block is overwritten - need to reload DS map
					& reflash */
				ds_same = 0;
				pr_info(MODULE_NAME " D/S map block is overwritten - need "
					"to reload DS map & reflash.\n");

				/* Steps a) through l) are for downloading Drive/Sense map
					to AMRI-5305 */
				/* a)	Write 0xAD to register WD_DISABLE (0x7D) which disables
					the watchdog. */
				pixart_w_reg_byte(ts->client, PIXART_REG_WD_DISABLE,
						PIXART_WD_DISABLE);

				/* b)	Write 0x00 to register 0x03 to clear register BOOT_STAT.
					 This ensures correct reads in step i) */
				/* clear Boot_stat */
				pixart_w_reg_byte(ts->client, PIXART_REG_BOOT_STAT_GEN2, 0x00);

				/* c) Write desired download control command to register
					DOWNLOAD_CTL (0x0A) to enable the appropriate download
					routine. */
				/* Please refer to above DOWNLOAD_CTL register description
					for the different values of the control command of */
				/* 0x29, 0x2A, 0x2C or 0x2D for each type of data download. */
				pixart_w_reg_byte(ts->client, PIXART_REG_IODL_CTL_GEN2,
						PIXART_VALUE_DS_MAP_DL);

				/* d) Delay for 1ms before continuing.. */
				mdelay(1);

				/* e) Poll bit 7 of register 0x0A and wait for it to be reset
					to 0, indicating AMRI-5305 is ready to receive download
					data. */
				/* If bit 7 not reset to 0 within a 2000 ms timeout, an error
					has occurred. */
				val = 0x80, i = 0;
				while (((val & 0x80) == 0x80)
						&& (i * 10 < PIXART_TIMEOUT_MS_FW_INIT_GEN2)) {
					i++;
					msleep(10);
					val = pixart_r_reg_byte(ts->client,
							PIXART_REG_IODL_CTL_GEN2);
				}

				if ((val & 0x80) == 0x80) {
					pr_info(MODULE_NAME "REG_IODL_CTL_GEN2 timeout waiting for "
						"MSB clear.  Got 0x%x instead.\n", val);
					return -1;
				} else {
					pr_info(MODULE_NAME "REG_IODL_CTL_GEN2 done erasing(value"
						" = 0x%x ) in >~%d ms \n", val, i * (10));
					/* machine dependent estimate */
				}

				/* f) delay by 100us before continuiing */
				udelay(100);

				/* g) Read X_ERROR_ID (0x0D) and discard value to clear it.
					A non-zero value will prevent successful download. */
				pixart_r_reg_byte(ts->client, PIXART_REG_ERROR_ID_GEN2);

				/* h)(unneccessary) If sense/drive download (0x2C) or delta
					compensation data download (0x2D), */
				/* clear bit 7 of BOOT_STAT (register 0x03) in preparation for
					later read of this bit in step 12. */
				/* clear Boot_stat */
				pixart_w_reg_byte(ts->client, PIXART_REG_BOOT_STAT_GEN2, 0x00);

				/* i)Write all the bytes in the CONFIGUATION FILE to register
					DOWNLOAD_DATA (0x0B). */
				/* For each byte written, maintain a minimum delay time of
					10us or more in between. */

				/* rows are swapped */
				/* reset reference frame */
				pixart_w_reg_byte(ts->client, PIXART_REG_IODL_DATA_GEN2, 0x10); // reset reference frame
				pixart_w_reg_byte(ts->client, PIXART_REG_IODL_DATA_GEN2, 0x0C); // 12 rows
				pixart_w_reg_byte(ts->client, PIXART_REG_IODL_DATA_GEN2, 0x14); // 20 columns

				/* rows are swapped */
				pixart_w_reg_byte(ts->client, PIXART_REG_IODL_DATA_GEN2, 0xab); // row 10 & 11
				pixart_w_reg_byte(ts->client, PIXART_REG_IODL_DATA_GEN2, 0x89); // row 8 & 9
				pixart_w_reg_byte(ts->client, PIXART_REG_IODL_DATA_GEN2, 0x67); // row 6 & 7
				pixart_w_reg_byte(ts->client, PIXART_REG_IODL_DATA_GEN2, 0x45); // row 4 & 5
				pixart_w_reg_byte(ts->client, PIXART_REG_IODL_DATA_GEN2, 0x23); // row 2 & 3
				pixart_w_reg_byte(ts->client, PIXART_REG_IODL_DATA_GEN2, 0x01); // row 0 & 1

				/* columns are mapped alternate odd/even */
				pixart_w_reg_byte(ts->client, PIXART_REG_IODL_DATA_GEN2, 0x00); // col# 0 -> AMRI-5305 col 0
				pixart_w_reg_byte(ts->client, PIXART_REG_IODL_DATA_GEN2, 0x02);// col# 2 -> AMRI-5305 col 1
				pixart_w_reg_byte(ts->client, PIXART_REG_IODL_DATA_GEN2, 0x04);// col# 4 -> AMRI-5305 col 2
				pixart_w_reg_byte(ts->client, PIXART_REG_IODL_DATA_GEN2, 0x06);// col# 6 -> AMRI-5305 col 3
				pixart_w_reg_byte(ts->client, PIXART_REG_IODL_DATA_GEN2, 0x08);// col# 8 -> AMRI-5305 col 4
				pixart_w_reg_byte(ts->client, PIXART_REG_IODL_DATA_GEN2, 0x0A);// col# 10 ->  AMRI-5305 col 5
				pixart_w_reg_byte(ts->client, PIXART_REG_IODL_DATA_GEN2, 0x0C);// col# 12 -> AMRI-5305 col 6
				pixart_w_reg_byte(ts->client, PIXART_REG_IODL_DATA_GEN2, 0x0E);// col#14 -> AMRI-5305 col 7
				pixart_w_reg_byte(ts->client, PIXART_REG_IODL_DATA_GEN2, 0x10);// col# 16 -> AMRI-5305 col 8
				pixart_w_reg_byte(ts->client, PIXART_REG_IODL_DATA_GEN2, 0x12);// col# 18 -> AMRI-5305 col 9
				pixart_w_reg_byte(ts->client, PIXART_REG_IODL_DATA_GEN2, 0x01);// col# 1 -> AMRI-5305 col 10
				pixart_w_reg_byte(ts->client, PIXART_REG_IODL_DATA_GEN2, 0x03);// col# 3 -> AMRI-5305 col 11
				pixart_w_reg_byte(ts->client, PIXART_REG_IODL_DATA_GEN2, 0x05);// col# 5 -> AMRI-5305 col 12
				pixart_w_reg_byte(ts->client, PIXART_REG_IODL_DATA_GEN2, 0x07);// col# 7 -> AMRI-5305 col 13
				pixart_w_reg_byte(ts->client, PIXART_REG_IODL_DATA_GEN2, 0x09);// col# 9 -> AMRI-5305 col 14
				pixart_w_reg_byte(ts->client, PIXART_REG_IODL_DATA_GEN2, 0x0B);// col# 11 -> AMRI-5305 col 15
				pixart_w_reg_byte(ts->client, PIXART_REG_IODL_DATA_GEN2, 0x0D);// col# 13 -> AMRI-5305 col 16
				pixart_w_reg_byte(ts->client, PIXART_REG_IODL_DATA_GEN2, 0x0F);// col# 15 -> AMRI-5305 col 17
				pixart_w_reg_byte(ts->client, PIXART_REG_IODL_DATA_GEN2, 0x11); // col# 17 -> AMRI-5305 col 18
				pixart_w_reg_byte(ts->client, PIXART_REG_IODL_DATA_GEN2, 0x13); // col# 19 -> AMRI-5305 col 19

				/* j) After completion of all bytes written, read register
					0x03 until bit 6 is set (1) and bit 5 is reset (0). */
				/* Start over on timeout or error.  A timeout of 500 ms is
					sufficient in determining if an error occurred. */
				/* A detected error will be reported in X_ERROR_ID (0x0D).
					If no timeout or error, continue to step 10. */
				/* poll boot_stat bit 6 for data down load completion */
				i = 0, val = 0;
				/* wait for either bit 6 or 5 to set (bit 6 = success,
					bit 5 = fail) */
				while (((val & 0x60) == 0x00)
						&& (i * 10 < PIXART_TIMEOUT_MS_BOOT_GEN2)) {
					i++;
					mdelay(10); /* 10 ms before check */
					val = pixart_r_reg_byte(ts->client,
						PIXART_REG_BOOT_STAT_GEN2);
				}

				/* Bit 6 - load is succesfull */
				if (val & 0x40)
					pr_info(MODULE_NAME "Post Data DL BOOT_STAT success "
						"(value = 0x%x ) in %d ms \n", val, i * 10);
				else {
					pr_info(MODULE_NAME "post Data DLD BOOT_STAT timeout "
						"or error waiting for correct bit values. Got "
						"0x%x instead.\n", val);
					val = pixart_r_reg_byte(ts->client,
						PIXART_REG_ERROR_ID_GEN2);
					pr_info(MODULE_NAME "post Data DL REG_ERROR_ID register "
						"value is 0x%x \n", val);
					return -1;
				}

				/* enable orienation */
				/* pixart_w_reg_byte(ts->client, PIXART_REG_ORIENTATION_GEN2,
					0x85); */
				/* k)  Write 0x00 to 0x7D which enables the watchdog. */
				pixart_w_reg_byte(ts->client, PIXART_REG_WD_DISABLE,
					PIXART_WD_ENABLE);
			}

			/* check for customer specific settings and flash if mismatch */
			if (crc_hi_cs != CRC_HI_CS_REGS || crc_lo_cs != CRC_LO_CS_REGS) {
				/* set settings mismatch flag */
				cs_same = 0;
				pr_info(MODULE_NAME "Customer Settings block CRC mismatch.\n");

				pixart_w_reg_byte(ts->client, PIXART_REG_ROW_GEN2, CTP_ROWS); // 12
				pixart_w_reg_byte(ts->client, PIXART_REG_COL_GEN2, CTP_COLS); // 20
				pixart_w_reg_byte(ts->client, PIXART_REG_WIN_MAX_Y_GEN2, CTP_ROWS-1); // last row # = 11
				pixart_w_reg_byte(ts->client, PIXART_REG_WIN_MAX_X_GEN2, CTP_COLS-1); // last col # = 19
				pixart_w_reg_byte(ts->client, PIXART_REG_REPORT_POINTS_GEN2, PIXART_TOUCH_NUMBER_GEN2);
				pixart_w_reg_byte(ts->client, PIXART_REG_MOTION_REPORT_CTL_GEN2, PIXART_MOTION_DISABLE_HOVER_GEN2);
				pixart_w_reg_byte(ts->client, PIXART_REG_TOUCH_REF_LO_GEN2, PIXART_VALUE_TOUCH_REF_LO_GEN2);
				pixart_w_reg_byte(ts->client, PIXART_REG_TOUCH_REF_HI_GEN2, PIXART_VALUE_TOUCH_REF_HI_GEN2);
				pixart_w_reg_byte(ts->client, PIXART_REG_ORIENTATION_GEN2, pdata->orient);
				pixart_w_reg_byte(ts->client, PIXART_REG_HEGHT_HI_GEN2, (pdata->y_size >> 8)& 0xFF);
				pixart_w_reg_byte(ts->client, PIXART_REG_HEIGHT_LO_GEN2, pdata->y_size & 0xFF);
				pixart_w_reg_byte(ts->client, PIXART_REG_WIDTH_HI_GEN2, (pdata->x_size >> 8)& 0xFF);
				pixart_w_reg_byte(ts->client, PIXART_REG_WIDTH_LO_GEN2, pdata->x_size & 0xFF);
			}

			/* save settings to flash if there's a mismatch of expected values
				for Customer Settings or DS map */
			if (!cs_same || !ds_same) {
				pr_info(MODULE_NAME "settings mismatch - re-flashing...\n");

				/* WD Dsiable before saving Data in Flash */
				pixart_w_reg_byte(ts->client, PIXART_REG_WD_DISABLE,
					PIXART_WD_DISABLE);
				pixart_w_reg_byte(ts->client, PIXART_REG_FLASH_ENABLE,
					PIXART_FLASH_ENABLE);

				msleep(1); /* 1 ms delay after flash enable. */

				if (!cs_same & !ds_same) {
					/* Both Customer Settings & DS map are corrupted - save
						both blocks to flash. */
					pixart_w_reg_byte(ts->client, PIXART_REG_FLASH_CTL_GEN2,
						PIXART_REG_FLASH_SAVE_CUST_REGS_DS_MAP);
					if (fw_id <= 0x05)
						msleep(200);
					else
						msleep(1);
					/* 1 ms per bit delay before reading flash_ctl */
				} else if (!cs_same) {
					/* Customer Settings block is corrupted - save customer
						settings block to flash, only */
					pixart_w_reg_byte(ts->client, PIXART_REG_FLASH_CTL_GEN2,
						PIXART_REG_FLASH_SAVE_CUST_REGS);

					if (fw_id <= 0x05)
						msleep(100);
					else
						msleep(1);
				}
				else if (!ds_same) {
					/* DS map block is corrupted - save DS map block to flash,
						only */
					pixart_w_reg_byte(ts->client, PIXART_REG_FLASH_CTL_GEN2,
						PIXART_REG_FLASH_SAVE_DS_MAP);

					if (fw_id <= 0x05)
						msleep(100);
					else
						msleep(1);
				}

				i = 0;
				val = pixart_r_reg_byte(ts->client, PIXART_REG_FLASH_CTL_GEN2);
				while ((val & 0x01)
					&& (i * 10 < PIXART_TIMEOUT_MS_AFTER_FLASH_ENABLE_GEN2)) {

					i++;
					mdelay(10);
					val = pixart_r_reg_byte(ts->client,
						PIXART_REG_FLASH_CTL_GEN2);
				}

				/* Enable Watchdog... */
				pixart_w_reg_byte(ts->client, PIXART_REG_WD_DISABLE,
					PIXART_WD_ENABLE);

				if ((val & 0x01)) {
					ErrorID = pixart_r_reg_byte(ts->client,
						PIXART_REG_ERROR_ID_GEN2);
					BootStat = pixart_r_reg_byte(ts->client,
						PIXART_REG_BOOT_STAT_GEN2);
					Stat = pixart_r_reg_byte(ts->client,
						PIXART_REG_STATUS_GEN2);
					fw_id = pixart_r_reg_byte(ts->client,
						PIXART_REG_FW_REVID_GEN2);
					pr_info(MODULE_NAME "timeout waiting for flash block write."
						" val = 0x%x, ErrorID = 0x%x, BootStat = 0x%x  i = 0x%x"
						" Status = 0x%x fw_id = 0x%x\n",
						val, ErrorID, BootStat, i, Stat, fw_id);
				} else
					pr_info(MODULE_NAME " Flash block write complete in"
						" %d ms\n", i * 10);

				if (!ds_same) {
					/* l) poll boot_stat bit 7 to be 1 for Nav ready after
						D/S Map download */
					i = 0, val = 0;
					/* wait for either bit 6 or 5 to set (bit 6 = success,
						bit 5 = fail) */
					while (((val & 0x80) == 0x00)
							&& (i * 10 < PIXART_TIMEOUT_MS_FW_INIT_GEN2)) {
						i++;
						mdelay(10);
						val = pixart_r_reg_byte(ts->client,
							PIXART_REG_BOOT_STAT_GEN2);
					}

					/* Bit 7 - Nav is ready! */
					if (val & 0x80)
						pr_info(MODULE_NAME "Post Data DL BOOT_STAT success "
							"(value = 0x%x ) in %d ms \n", val, i * 10);
					else {
						ErrorID = pixart_r_reg_byte(ts->client,
							PIXART_REG_ERROR_ID_GEN2);
						pr_info(MODULE_NAME "post Data DLD BOOT_STAT timeout "
							"or error waiting for correct Nav Ready bit. Got "
							"0x%x instead. ErrorID = 0x%x\n", val, ErrorID);
					}
				}
			} else
				pr_info(MODULE_NAME "settings match - no re-flash "
					"required.. \n");

		} else{ /* end of if block for not new fw loaded condition */
			printk("%s: pixart Panel Config else route\n",__func__);
			pixart_set_config(ts);
		}
		break;

	default: /* following is for 5100 & 5200 devices only.. */
		printk("Pix_init_panel FW is not new. Default\n");
		pixart_w_reg_byte(ts->client, PIXART_REG_POINTS,
			PIXART_TOUCH_NUMBER_GEN1);

#ifdef PIXART_SET_TOUCH_THRESHOLD
		pixart_w_reg_byte(ts->client, PIXART_REG_TOUCH_1_HIGH,
			PIXART_VALUE_TOUCH_1_HIGH);
		pixart_w_reg_byte(ts->client, PIXART_REG_TOUCH_1_LO,
			PIXART_VALUE_TOUCH_1_LO);
		pixart_w_reg_byte(ts->client, PIXART_REG_TOUCH_2_HIGH,
			PIXART_VALUE_TOUCH_2_HIGH);
		pixart_w_reg_byte(ts->client, PIXART_REG_TOUCH_2_LO,
			PIXART_VALUE_TOUCH_2_LO);
		pixart_w_reg_byte(ts->client, PIXART_REG_TOUCH_3_HIGH,
			PIXART_VALUE_TOUCH_3_HIGH);
		pixart_w_reg_byte(ts->client, PIXART_REG_TOUCH_3_LO,
			PIXART_VALUE_TOUCH_3_LO);
#endif

		pixart_w_reg_byte(ts->client, PIXART_REG_ORIENTATION, pdata->orient);
		pixart_w_reg_byte(ts->client, PIXART_REG_HEIGHT_HI,
			(pdata->y_size >> 8)& 0xFF);
		pixart_w_reg_byte(ts->client, PIXART_REG_HEGHT_LO,
			pdata->y_size & 0xFF);
		pixart_w_reg_byte(ts->client, PIXART_REG_WIDTH_HI,
			(pdata->x_size >> 8)& 0xFF);
		pixart_w_reg_byte(ts->client, PIXART_REG_WIDTH_LO,
			pdata->x_size & 0xFF);
		udelay(500);
		break;
	}

	return 0;
}

static irqreturn_t pixart_interrupt(int irq, void *dev_id)
{
	int i, j;
	struct pixart_data *ts = dev_id;
	struct pixart_touch_data_gen2 *touch_data = (struct pixart_touch_data_gen2 *)&ts->touch_data;
	uint8_t *pdata, status, val;
	uint8_t move = 0;
	uint8_t readFrom[] = {
			PIXART_REG_MOTION_REPORT_DATA_GEN2,
			PIXART_REG_MOTION_REPORT_DATA_GEN2,
			PIXART_REG_MOTION_REPORT_DATA_GEN2,
			PIXART_REG_MOTION_REPORT_DATA_GEN2,
			PIXART_REG_MOTION_REPORT_DATA_GEN2,
			PIXART_REG_MOTION_REPORT_DATA_GEN2,
			PIXART_REG_MOTION_REPORT_DATA_GEN2,
			PIXART_REG_MOTION_REPORT_DATA_GEN2,
			PIXART_REG_MOTION_REPORT_DATA_GEN2
	};

	mutex_lock(&ts->lock);
	ts->touch_number = 0;

	if (Touch_Status >= TOUCH_POWEROFF)
		goto end_of_interrupt;

	if(ts_event_control)
		goto end_of_interrupt;

	if (disable_count) {
		disable_count--;
		goto end_of_interrupt;
	}

	status = pixart_r_reg_byte(ts->client, PIXART_REG_STATUS_GEN2);
	if (status & PIXART_REG_STATUS_GEN2_ERROR){
		val = pixart_r_reg_byte(ts->client, PIXART_REG_ERROR_ID_GEN2);
		pr_err("%s: Error Status. ID = 0x%02x\n",__func__, val);
		if(val != 0x03){
			switch (val) {
			case 0x01:
			case 0x02:
			case 0x06:
			case 0x07:
			case 0x08:
			case 0x09:
			case 0x0A:
			case 0x0B:
			case 0x0C:
			case 0x0D:
			case 0x0E:
			case 0x0F:
			case 0x10:
			case 0x11:
			case 0x12:
			case 0x13:
			case 0x14:
			case 0x15:
			case 0x23:
				pr_err("%s: IC Reset\n",__func__);
				if (ts->is_enable) {
					disable_irq_nosync(ts->irq);
					ts->is_enable = false;
				}
				if (pixart_reset_and_wait(ts))
					pr_err("%s: Failed to restart!\n",__func__);
				if (pixart_reset_status(ts))
					pr_err("%s: Failed to reset status!\n",__func__);
				if (!ts->is_enable) {
					enable_irq(ts->irq);
					ts->is_enable = true;
				}
				break;
			default:
				break;
			}
			goto end_of_interrupt;
		}
	}
	if (!(status & PIXART_REG_STATUS_GEN2_DATA_READY)){
		pr_err("%s: Error End\n",__func__);
		goto end_of_interrupt;
	}
	if (!(status & PIXART_REG_STATUS_GEN2_TOUCH))
		PIX_DEV_TOUCH("%s: decision No Touch\n",__func__);

	pixart_w_reg_byte(ts->client, PIXART_REG_MOTION_REPORT_CTL_GEN2,
		PIXART_MOTION_MARK_READ_GEN2 | PIXART_MOTION_DISABLE_HOVER_GEN2);	//0x84

	pdata = (uint8_t *)touch_data;
	pixart_r_reg_nbytes(ts->client, readFrom, pdata, 2);	/* status, total touch */
	if (*pdata == 0xff) {
		pr_err("%s: ignoring report w/ status = 0xff.\n",__func__);
		pixart_w_reg_byte(ts->client, PIXART_REG_MOTION_REPORT_CTL_GEN2,0x04);
		goto end_of_interrupt;
	}
	pdata += 2;
	pixart_r_reg_nbytes(ts->client, readFrom, pdata, 9); /* id, x0, y0, f0, a0 */
	pdata += 9;
	if (touch_data->total_touch > PIXART_TOUCH_NUMBER_GEN2)
		touch_data->total_touch = PIXART_TOUCH_NUMBER_GEN2;

	for (i = 1; i < touch_data->total_touch; i++) {
		pixart_r_reg_nbytes(ts->client, readFrom, pdata, 9); /* idn,xn,yn */
		pdata += 9;
	}

	pixart_w_reg_byte(ts->client, PIXART_REG_MOTION_REPORT_CTL_GEN2,0x04);

	for(j = 0; j < 10; j++){
		if(touch_data->slot_id[j] == 0)
			continue;
		val = j;
		for(i = 0; i < touch_data->total_touch; i++){
			if(touch_data->slot_id[j] == (touch_data->point_data[i].id & 0x0f)){
				val = 255;
				break;
			}
		}
		if(val != 255){
			PIX_DEV_TOUCH("%s RELEASE j=%d\n",__func__,j);
			input_mt_slot(ts->input, j);
			input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, 0);
			touch_data->slot_id[j] = 0;
		}
	}

	for (i = 0; i < touch_data->total_touch; i++) {
		if(touch_data->point_data[i].id < 129 || touch_data->point_data[i].id > 139){
			printk("%s: Error id = %d\n",__func__, touch_data->point_data[i].id);
			continue;
		}		
		for(j = 0; j < 10; j++){
			if(touch_data->slot_id[j] == (touch_data->point_data[i].id & 0x0F) && (touch_data->point_data[i].id & 0x0F) != 0){
				PIX_DEV_TOUCH("%s: MOVE j=%d , touch_id = %d\n",
							__func__,j,(touch_data->point_data[i].id & 0x0F));
				move = 1;
				break;
			}
		}
		if(!move){
			for(j = 0; j < 10; j++){
				if(touch_data->slot_id[j] == 0 && (touch_data->point_data[i].id & 0x0F) != 0){
					touch_data->slot_id[j] = (touch_data->point_data[i].id & 0x0F);
					PIX_DEV_TOUCH("%s: PUSH j=%d , touch_id = %d\n",
							__func__,j,(touch_data->point_data[i].id & 0x0F));
					break;
				}
			}
		}
		move = 0;

		input_mt_slot(ts->input, j);
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, (touch_data->point_data[i].id & 0x0F) != 0);

		ts->touch_number++;
		input_report_abs(ts->input, ABS_MT_POSITION_X,	touch_data->point_data[i].x);
		input_report_abs(ts->input, ABS_MT_POSITION_Y,	touch_data->point_data[i].y);
		input_report_abs(ts->input, ABS_MT_PRESSURE,	touch_data->point_data[i].force);
		input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR,	touch_data->point_data[i].area);
		input_report_abs(ts->input, ABS_MT_WIDTH_MAJOR,	0x14);
		PIX_DEV_TOUCH("%s: [%02d] id:%02d , (x,y) = ( %d	,	%d )\n",__func__,j,
			touch_data->point_data[i].id,touch_data->point_data[i].x,touch_data->point_data[i].y);
	}

	input_report_key(ts->input, BTN_TOUCH, ts->touch_number > 0);
	input_sync(ts->input);

	mutex_unlock(&ts->lock);
	return IRQ_HANDLED;

end_of_interrupt:
	mutex_unlock(&ts->lock);
	PIX_DEV_TOUCH("%s: end of work func\n",__func__);
	return IRQ_HANDLED;
}

static int pixart_ts_open(struct inode *inode, struct file *file)
{
	struct pixart_data *ts = container_of(inode->i_cdev, struct pixart_data, device_cdev);

	file->private_data = ts;
	return 0;
};

static int pixart_ts_release(struct inode *inode, struct file *file)
{
	return 0;
};

#if 0
static int pixart_atoi(struct device *dev, const char *src, u8 *dst)
{
	u8 val = 0;
	int cnt = 0;

	for (;; src++) {
		switch (*src) {
		case '0' ... '9':
			val = 16 * val + (*src - '0');
			break;
		case 'A' ... 'F':
			val = 16 * val + (*src - '7');
			break;
		case 'a' ... 'f':
			val = 16 * val + (*src - 'W');
			break;
		default:
			return 0;
		}
		if ((cnt % 2) == 1) {
			*dst = val;
			dst++;
			val = 0;
		}
		cnt++;
	}
	return -1;
}

static long pixart_get_property(struct device *dev, unsigned long arg,
								enum ts_config_type type)
{
	struct pixart_data *ts = dev_get_drvdata(dev);
	struct ts_nv_data *dp = (struct ts_nv_data *)arg;
	struct ts_config_nv *config_nv = &ts->config_nv[type];
	long ret = 0;
	long err;
	size_t size;
	char *str;
	char ver[5];
	char *p;

	err = copy_from_user(&size, (void __user *)&dp->size, sizeof(size_t));
	if (err){
		ret = -EFAULT;
		pr_err("%s: copy_from_user error\n", __func__);
		goto done;
	}
	str = kcalloc(size, sizeof(char), GFP_KERNEL);
	if (!str) {
		pr_err("%s: Failed to allocate memory\n", __func__);
		ret = -ENOMEM;
		goto done;
	}
	err = copy_from_user(str, (void __user *)dp->data, size);
	if (err){
		ret = -EFAULT;
		pr_err("%s: copy_from_user error\n", __func__);
		goto done;
	}

	mutex_lock(&ts->lock);
	config_nv->size = size / 2 - 2;

	if (!config_nv->data) {
		config_nv->data = kcalloc(config_nv->size, sizeof(char), GFP_KERNEL);
		if (!config_nv->data) {
			pr_err("%s: Failed to allocate memory\n", __func__);
			ret = -ENOMEM;
			goto err_free_str;
		}
	} else
		pr_err("%s: config_nv->data has been allocated.\n", __func__);

	memset(ver, '\0', sizeof(ver));
	memcpy(ver, str, 4);
	pixart_atoi(dev, ver, (u8 *)&config_nv->ver);

	p = str + 4;
	pixart_atoi(dev, p, config_nv->data);
	PIX_DEV_DBG("%s: type = %d, size = %d\n", __func__, type, size);

err_free_str:
	mutex_unlock(&ts->lock);
	kfree(str);

done:
	return ret;
}

static long pixart_set_nv(struct pixart_data *ts)
{
	struct i2c_client *client = ts->client;
	struct ts_config_nv *config_nv;
	long err;
	u8 i, val, ret;
	u8 flash = 0;

	PIX_DEV_DBG("%s() is called.\n",__func__);
	config_nv = &ts->config_nv[TS_CHARGE_CABLE];		/* INITIAL_VALUE */

	if (!config_nv) {
		pr_err("%s: No nv data. Skipping set nv.\n",__func__);
		return 0;
	}

	err = wait_for_completion_interruptible_timeout(&ts->init_done,
			msecs_to_jiffies(5 * MSEC_PER_SEC));

	if (err < 0) {
		pr_err("%s: Error waiting device init (%d)!\n", __func__, (int)err);
		return -ENXIO;
	} else if (err == 0) {
		pr_err("%s: Timedout while waiting for device init!\n",__func__);
		return -ENXIO;
	}

	mutex_lock(&ts->lock);
	if (ts->is_enable) {
		disable_irq_nosync(client->irq);
		ts->is_enable = false;
	}

	for(i = 0; i < config_nv->size; ){
		if(config_nv->data[i+1] == 0){
			PIX_DEV_DBG("%s: config is over\n",\n);
			break;
		}
		val = pixart_r_reg_byte(ts->client, config_nv->data[i]);
		if(val != config_nv->data[i+2]){
			PIX_DEV_DBG("%s: val=%02x, Write 0x%02x -> 0x%02x\n",__func__,val,config_nv->data[i], config_nv->data[i+2]);
			pixart_w_reg_byte(ts->client, config_nv->data[i], config_nv->data[i+2]);
			flash = 1;
		}
		i += 3;
	}

	if(flash){
		ret = pixart_flash_save(ts);
		if(ret == 0)
			ts->is_set = true;
		else{
			pr_err("%s: Fail Backup\n",__func__);
			ts->is_set = false;
		}
	}

	if (!ts->is_enable) {
		enable_irq(client->irq);
		ts->is_enable = true;
	}

	mutex_unlock(&ts->lock);
	return 0;
}

static long pixart_switch_config(struct device *dev)
{
	struct pixart_data *ts = dev_get_drvdata(dev);
	struct ts_config_nv *config_nv;
	int i, err;

	config_nv = &ts->config_nv[ts->config_status];
	PIX_DEV_DBG("%s: config status is %d\n",__func__ ,ts->config_status);

	if(ts_log_level 0x01){
		printk("%s: config is = ",__func__);
		for(i = 0; i < config_nv->size; i++){
			printk("%02x",config_nv->data[i]);
		}
		printk("\n");
	}

	if(ts->last_config_status == ts->config_status){
		PIX_DEV_DBG("%s: Skip. Same status.\n",__func__);
		return 0;
	}

	if(!config_nv->data){
		pr_err("%s: No nv data.\n",__func__);
		return 0;
	}
	for(i = 0; i < config_nv->size; ){
		if(config_nv->data[i+1] == 0){
			PIX_DEV_DBG("%s: config is over\n",__func__);
			break;
		}
		err = pixart_w_reg_byte(ts->client, config_nv->data[i], config_nv->data[i+2]);
		i += 3;
		if(err){
			pr_err("%s: Error Write register\n",__func__);
			return err;
		}
	}

	ts->last_config_status = ts->config_status;
	return 0;
}
#endif

static long pixart_ts_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct pixart_data *ts = (struct pixart_data *)file->private_data;
	struct device *dev = &ts->client->dev;
	long err = 0;

	switch (cmd) {
	case IOCTL_SET_CONF_STAT:
#if 0
		if (ts->client->irq == -1) {
			printk("%s: driver is abnormal status.\n",__func__);
			return -1;
		}
		if (!access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd))) {
			err = -EFAULT;
			dev_err(dev, "%s: invalid access\n", __func__);
			goto done;
		}
		mutex_lock(&ts->lock);
		if (copy_from_user(&ts->config_status, (void __user *)arg, sizeof(ts->config_status))) {
			err = -EFAULT;
			dev_err(dev, "%s: copy_from_user error\n", __func__);
			mutex_unlock(&ts->lock);
			goto done;
		}

		err = pixart_switch_config(dev);
		mutex_unlock(&ts->lock);
		break;
#else
		printk("%s: IOCTL_SET_CONF_STAT skip\n",__func__);
		return  0;
#endif
	case IOCTL_GET_CONF_STAT:
	case IOCTL_SET_LOG:
	case IOCTL_DIAG_START:
	case IOCTL_MULTI_GET:
	case IOCTL_COODINATE_GET:
	case IOCTL_DIAG_END:
		return 0;
	case IOCTL_DIAG_EVENT_CTRL:
		if (ts->client->irq == -1) {
			printk("%s: driver is abnormal status.\n",__func__);
			return -1;
		}
		if (!access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd))) {
			err = -EFAULT;
			dev_err(dev, "%s: invalid access\n", __func__);
			goto done;
		}
		err = copy_from_user(&ts_event_control, (void __user *)arg,
						sizeof(unsigned char));
		if (err){
			dev_err(dev, "%s: copy_from_user error\n", __func__);
			return -EFAULT;
		}
		return 0;
	case IOCTL_LOAD_CHARGE_C_NV:
#if 0
		if (!access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd))) {
			err = -EFAULT;
			dev_err(dev, "%s: invalid access\n", __func__);
			goto done;
		}
		err = pixart_get_property(dev, arg, TS_CHARGE_CABLE);
		break;
#else
		printk("%s: IOCTL_LOAD_CHARGE_C_NV skip\n",__func__);
		return  0;
#endif
	case IOCTL_LOAD_CHARGE_A_S1_NV:
	case IOCTL_LOAD_CHARGE_A_S2_NV:
		return 0;
	case IOCTL_LOAD_DISCHARGE_NV:
#if 0
		printk("%s: IOCTL_LOAD_DISCHARGE\n", __func__);
		if (!access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd))) {
			err = -EFAULT;
			dev_err(dev, "%s: invalid access\n", __func__);
			goto done;
		}
		err = pixart_get_property(dev, arg, TS_DISCHARGE);
		break;
#else
		printk("%s: IOCTL_LOAD_DISCHARGE_NV skip\n",__func__);
		return  0;
#endif
	case IOCTL_LOAD_WS_NV:
		return 0;
	case IOCTL_SET_NV:
#if 0
		printk("%s: IOCTL_SET_NV\n", __func__);
		if (ts->client->irq == -1) {
			printk("%s: driver is abnormal status.\n",__func__);
			return -1;
		}
		err = pixart_set_nv(ts);
		break;
#else
		printk("%s: IOCTL_SET_NV skip\n",__func__);
		return  0;
#endif
	case IOCTL_DIAG_LOG_LEVEL:
	case IOCTL_LOAD_DEBUG_FLAG_NV:
	case IOCTL_DIAG_RESET_HW:
	case IOCTL_GET_GOLDEN_REFERENCE:
	case IOCTL_DIAG_GET_C_REFERENCE:
	case IOCTL_DIAG_GET_DELTA:
	case IOCTL_CHECK_FW:
	case IOCTL_GET_INFO:
		return 0;
	default:
		return -EINVAL;
		break;
	}
done:
	return err;
}

const struct file_operations pixart_ts_fops = {
	.owner = THIS_MODULE,
	.open = pixart_ts_open,
	.unlocked_ioctl = pixart_ts_ioctl,
	.release = pixart_ts_release,
};

static int pixart_input_open(struct input_dev *dev)
{
	struct pixart_data *ts = input_get_drvdata(dev);
	int error;

	PIX_DEV_DBG("%s() is called.\n",__func__);
	error = wait_for_completion_interruptible_timeout(&ts->init_done,
			msecs_to_jiffies(5 * MSEC_PER_SEC));

	if (error > 0) {
		if (ts->client->irq != -1){
			error = pixart_enable_irq(ts);
		}
		else {
			pr_err("%s: Can't enable irq.\n",__func__);
			error = -ENXIO;
		}
	} else if (error < 0) {
		pr_err("%s: Error while waiting for device init (%d)!\n",__func__, error);
		error = -ENXIO;
	} else if (error == 0) {
		pr_err("%s: Timedout while waiting for device init!\n",__func__);
		error = -ENXIO;
	}
/*
	reg_touch = regulator_get(NULL, "8921_l17");
	if(IS_ERR(reg_touch)){
		pr_err("%s regulator is not get\n",__func__);
	}
*/
	return error;
}

static void pixart_input_close(struct input_dev *dev)
{
	struct pixart_data *ts = input_get_drvdata(dev);

	PIX_DEV_DBG("%s() is called.\n",__func__);
	if (ts->client->irq != -1){
		pixart_disable_irq(ts);
	}
}


static int pixart_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pixart_data *ts = i2c_get_clientdata(client);

	PIX_DEV_DBG("%s() is called.\n",__func__);

	if(pixart_wq){
		cancel_delayed_work_sync(&ts->esdwork);
		flush_workqueue(pixart_wq);
	}

	mutex_lock(&ts->lock);

	disable_irq(client->irq);
	Touch_Status = TOUCH_POWEROFF;

	pixart_w_reg_byte(client, 0x7a, 0xaa);

	mutex_unlock(&ts->lock);
	return 0;
}

static void pixart_report_clear(struct pixart_data *ts)
{
	int i;

	PIX_DEV_DBG("%s() is called.\n",__func__);
	for(i = 0; i < 10; i++){
		if(ts->touch_data.slot_id[i] == 0)
			continue;

		PIX_DEV_DBG("%s: [%d] released\n",__func__, i);
		input_mt_slot(ts->input, i);
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, 0);
		ts->touch_data.slot_id[i] = 0;
	}

	input_report_key(ts->input, BTN_TOUCH, 0);
	input_sync(ts->input);
}

static int pixart_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pixart_data *ts = i2c_get_clientdata(client);
	int i;
	u8 err;

	PIX_DEV_DBG("%s() is called.\n",__func__);
	mutex_lock(&ts->lock);

	pixart_report_clear(ts);

	pixart_w_reg_byte(client, 0x7a, 0xdd);

	for (i = 0; i < 750; i++) {
		err = pixart_r_reg_byte(client, 0x03);
		if ((err & 0x91) == 0x91){
			PIX_DEV_DBG("%s: Success. Read retry = %d\n",__func__, i);
			break;
		}
		udelay(1000);
	}
	if(i >= 750)
		pr_err("%s: Read error\n",__func__);

	enable_irq(client->irq);
	Touch_Status = TOUCH_POWERON;

	if(pixart_wq){
		queue_delayed_work(pixart_wq, &ts->esdwork,
				   msecs_to_jiffies(5000));
	}

	mutex_unlock(&ts->lock);
	return 0;
}

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct pixart_data *ts =
		container_of(self, struct pixart_data, fb_notif);

	if (evdata && evdata->data && event == FB_EVENT_BLANK && ts && ts->client) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
			pixart_resume(&ts->client->dev);
		else if (*blank == FB_BLANK_POWERDOWN)
			pixart_suspend(&ts->client->dev);
	}

	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void pixart_early_suspend(struct early_suspend *h)
{
	struct pixart_data *ts = container_of(h, struct pixart_data, early_suspend);
	pixart_suspend(&ts->client->dev);
}

static void pixart_late_resume(struct early_suspend *h)
{
	struct pixart_data *ts = container_of(h, struct pixart_data, early_suspend);
	pixart_resume(&ts->client->dev);
}
#endif

static const struct dev_pm_ops pix_pm_ops = {
#if (!defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND))
	.suspend	= pixart_suspend,
	.resume		= pixart_resume,
#endif
};


static void pixart_esd_work(struct work_struct *work)
{
	struct pixart_data *ts = container_of(work, struct pixart_data, esdwork.work);
	u8 val;

	if((ts_esd_recovery != 0) && mutex_trylock(&ts->lock)) {
		val = pixart_r_reg_byte(ts->client, PIXART_REG_PID);
		if(pid != val){
			pr_err("%s: error\n",__func__);
			if (ts->is_enable) {
				disable_irq_nosync(ts->irq);
				ts->is_enable = false;
			}
			if (pixart_reset_and_wait(ts))
				pr_err("%s: Failed to restart!\n",__func__);
			if (pixart_reset_status(ts))
				pr_err("%s: Failed to reset status!\n",__func__);
			if (!ts->is_enable) {
				enable_irq(ts->irq);
				ts->is_enable = true;
			}
		}else
			PIX_DEV_DBG("%s: Correct PROD_ID [0x%02x]\n",__func__, val);
		mutex_unlock(&ts->lock);
	}
	if(pixart_wq){
		queue_delayed_work(pixart_wq, &ts->esdwork,
					msecs_to_jiffies(5000));
	}
}

static int __devinit pixart_ts_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	const struct pixart_platform_data *pdata;
	struct pixart_data *ts;
	struct input_dev *input_dev;
	int ret, err = 0;

#ifdef CONFIG_OF
	client->dev.platform_data = &pixart_platform_data;
#endif
	pdata = client->dev.platform_data;

	disable_count = 0;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s: need I2C_FUNC_I2C\n",__func__);
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof (struct pixart_data), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!ts || !input_dev) {
		pr_err("%s: Failed to allocate memory\n",__func__);
		err = -ENOMEM;
		goto err_free_mem;
	}

	ts->client = client;
	ts->input = input_dev;
	ts->touch_number = 0;
	ts->irq = client->irq;

	for(ret = 0; ret < 10; ret++){
		ts->touch_data.slot_id[ret] = 0;
	}

	input_dev->name = PIXART_I2C_NAME;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;
	input_dev->open = pixart_input_open;
	input_dev->close = pixart_input_close;

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(EV_SYN, input_dev->evbit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	input_mt_init_slots(input_dev, PIXART_MAX_FINGER);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,  0, pdata->y_size, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,  0, pdata->x_size, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, AMRI5K_FORCE_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, AMRI5K_AREA_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE,    0, AMRI5K_MAX_PRESSURE, 0, 0);

	mutex_init(&ts->lock);
	init_completion(&ts->init_done);

	input_set_drvdata(input_dev, ts);
	i2c_set_clientdata(client, ts);

	if (pdata->init_hw)
		err = pdata->init_hw();

	if(err){
		pr_err("%s: Failed to initialize hardware\n",__func__);
		goto err_free_mem;
	}

	err = pixart_reset_and_wait(ts);
	if (err) {
		pr_err("%s: Reset and wait failed\n",__func__);
		goto err_free_mem;
	}

	if (pid < PIXART_5000_PID) {
		pr_err("%s: Failed. PID reported was 0x%x\n",__func__, pid);
		goto err_free_mem;
	}

	err = pixart_init_panel(ts);
	if (err != 0) {
		pr_err("%s: failed.\n",__func__);
		goto err_free_mem;
	}

	if (request_threaded_irq(client->irq, NULL, pixart_interrupt,
				pdata->irqflags, client->dev.driver->name, ts)) {
		pr_err("%s: Failed to register interrupt\n",__func__);
		goto err_free_mem;
	}

	disable_irq(client->irq);
	complete_all(&ts->init_done);

	if(pixart_wq) {
		INIT_DELAYED_WORK(&ts->esdwork, pixart_esd_work);
		queue_delayed_work(pixart_wq, &ts->esdwork,
				   msecs_to_jiffies(10000));
	}

	err = input_register_device(input_dev);
	if (err) {
		pr_err("%s: input_register_device\n",__func__);
		goto err_free_irq;
	}

	pixart_sysfs_init(ts);
	pixart_sysfs_init_fs(ts);

	ts_ctrl_init(&(ts->device_cdev), &pixart_ts_fops);

#if defined(CONFIG_FB)
	ts->fb_notif.notifier_call = fb_notifier_callback;
	err = fb_register_client(&ts->fb_notif);
	if (err)
		dev_err(&client->dev, "Unable to register fb_notifier: %d\n", err);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = pixart_early_suspend;
	ts->early_suspend.resume = pixart_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	return 0;

err_free_irq:
	free_irq(client->irq, ts);

err_check_functionality_failed:
	return ret;

err_free_mem:
	input_free_device(input_dev);
	kfree(ts);

	return err;
}

static int pixart_ts_remove(struct i2c_client *client)
{
	struct pixart_data *ts = i2c_get_clientdata(client);

	PIX_DEV_DBG("%s() is called.\n",__func__);
	free_irq(ts->client->irq, ts);
	input_unregister_device(ts->input);

	kfree(ts);
	pr_info("unregistered touchscreen\n");
	return 0;
}


static const struct i2c_device_id pixart_ts_id[] = {
	{ PIXART_I2C_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static struct of_device_id pixart_match_table[] = {
	{ .compatible = "pixart,pt",},
	{ },
};
#else
#define pixart_match_table NULL
#endif

static struct i2c_driver pixart_ts_driver = {
	.probe		= pixart_ts_probe,
	.remove		= pixart_ts_remove,
	.id_table	= pixart_ts_id,
	.driver = {
		.name	= PIXART_I2C_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = pixart_match_table,
	},
};

static int __init pixart_init(void)
{
	pr_info(MODULE_NAME "%s\n", __func__);

	pixart_wq = alloc_workqueue("pixart_wq", WQ_MEM_RECLAIM, 1);
	if (!pixart_wq){
		pr_info(MODULE_NAME "%s workqueue Error\n", __func__);
		return -ENOMEM;
	}

	return i2c_add_driver(&pixart_ts_driver);
}

static void __exit pixart_exit(void)
{
	PIX_DEV_DBG("%s() is called.\n",__func__);
	if (pixart_wq)
		destroy_workqueue(pixart_wq);

	i2c_del_driver(&pixart_ts_driver);
}
module_init(pixart_init);
module_exit(pixart_exit);

MODULE_AUTHOR("KYOCERA Corporation");
MODULE_DESCRIPTION("Pixart Touchscreen Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("pixart");
