/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 * (C) 2013 KYOCERA Corporation
 */
/*
 * Copyright (c) 2010-2012 Yamaha Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA  02110-1301, USA.
 */

#include <asm/atomic.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

#include <linux/yas.h>

#include <linux/sensor_power.h>
static struct i2c_client *this_client = {0};

#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS529
#include "yas_mag_driver-yas529.c"
#elif YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS530
#include "yas_mag_driver-yas53x.c"
#elif YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS532
#include "yas_mag_driver-yas53x.c"
#elif YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS533
#include "yas_mag_driver-yas53x.c"
#else
#include "yas_mag_driver-none.c"
#endif

#define GEOMAGNETIC_I2C_DEVICE_NAME	"YAS532B"
#define GEOMAGNETIC_INPUT_NAME		"geomagnetic"
#define GEOMAGNETIC_INPUT_RAW_NAME	"geomagnetic_raw"
#undef GEOMAGNETIC_PLATFORM_API

#define ABS_STATUS			(ABS_BRAKE)
#define ABS_WAKE			(ABS_MISC)

#define ABS_RAW_DISTORTION		(ABS_THROTTLE)
#define ABS_RAW_THRESHOLD		(ABS_RUDDER)
#define ABS_RAW_SHAPE			(ABS_WHEEL)
#define ABS_RAW_MODE			(ABS_HAT0X)
#define ABS_RAW_GYRO_THRESHOLD		(ABS_HAT0Y)
#define ABS_RAW_REPORT			(ABS_GAS)

#define GEO_EV_UPDATED_OVERFLOW         0x00000001
#define GEO_EV_UPDATED_HARD_OFFSET      0x00000002
#define GEO_EV_UPDATED_CALIB_OFFSET     0x00000004
#define GEO_EV_UPDATED_ACCURACY         0x00000008
#define GEO_EV_UPDATED_THRESHOLD        0x00000010
#define GEO_EV_UPDATED_DISTORTION       0x00000020

struct geomagnetic_data {
	struct input_dev *input_data;
	struct input_dev *input_raw;
	struct delayed_work work;
	struct semaphore driver_lock;
	struct semaphore multi_lock;
	atomic_t last_data[3];
	atomic_t last_raw[3];
	atomic_t last_status;
	atomic_t enable;
	int filter_enable;
	int filter_len;
	int32_t filter_noise[3];
	int32_t filter_threshold;
	int delay;
	int32_t mag_noise;
	int32_t gyro_noise;
	int32_t threshold;
	int32_t distortion[3];
	int32_t shape;
	int32_t ellipsoid_mode;
	struct yas_mag_offset driver_offset;
#if DEBUG
	int suspend;
#endif
#ifdef YAS_MAG_MANUAL_OFFSET
	struct yas_vector manual_offset;
#endif
	struct yas_matrix static_matrix;
	struct yas_matrix dynamic_matrix;
#ifdef YAS_SENSOR_KERNEL_DEVFILE_INTERFACE
	struct list_head devfile_list;
	struct list_head raw_devfile_list;
#endif
	int32_t input_event_log;
	int32_t input_event_updated;
};
static struct workqueue_struct *mag_polling_wq;

static int32_t kc_geomagnetic_set_reset(int32_t reset);

#ifdef YAS_SENSOR_KERNEL_DEVFILE_INTERFACE

#include <linux/miscdevice.h>
#define MAX_COUNT (64)
#define SENSOR_NAME "geomagnetic"
#define SENSOR_RAW_NAME "geomagnetic_raw"

struct sensor_device {
	struct list_head list;
	struct mutex lock;
	wait_queue_head_t waitq;
	struct input_event events[MAX_COUNT];
	int head, num_event;
};

static void
get_time_stamp(struct timeval *tv)
{
	struct timespec ts;
	ktime_get_ts(&ts);
	tv->tv_sec = ts.tv_sec;
	tv->tv_usec = ts.tv_nsec / 1000;
}

static void
mkev(struct input_event *ev, int type, int code, int value)
{
	struct timeval tv;
	get_time_stamp(&tv);
	ev->type = type;
	ev->code = code;
	ev->value = value;
	ev->time = tv;
}

static void
mkev_with_time(struct input_event *ev, int type, int code, int value,
	struct timeval *tv)
{
	ev->type = type;
	ev->code = code;
	ev->value = value;
	ev->time = *tv;
}

static void
sensor_enq(struct sensor_device *kdev, struct input_event *ev)
{
	int idx;

	idx = kdev->head + kdev->num_event;
	if (MAX_COUNT <= idx)
		idx -= MAX_COUNT;
	kdev->events[idx] = *ev;
	kdev->num_event++;
	if (MAX_COUNT < kdev->num_event) {
		kdev->num_event = MAX_COUNT;
		kdev->head++;
		if (MAX_COUNT <= kdev->head)
			kdev->head -= MAX_COUNT;
	}
}

static int
sensor_deq(struct sensor_device *kdev, struct input_event *ev)
{
	if (kdev->num_event == 0)
		return 0;

	*ev = kdev->events[kdev->head];
	kdev->num_event--;
	kdev->head++;
	if (MAX_COUNT <= kdev->head)
		kdev->head -= MAX_COUNT;
	return 1;
}

static void
sensor_event(struct list_head *devlist, struct input_event *ev, int num)
{
	struct sensor_device *kdev;
	int i;

	list_for_each_entry(kdev, devlist, list) {
		mutex_lock(&kdev->lock);
		for (i = 0; i < num; i++)
			sensor_enq(kdev, &ev[i]);
		mutex_unlock(&kdev->lock);
		wake_up_interruptible(&kdev->waitq);
	}
}

static ssize_t
sensor_write(struct file *f, const char __user *buf, size_t count, loff_t *pos)
{
	struct geomagnetic_data *data = i2c_get_clientdata(this_client);
	struct sensor_device *kdev;
	struct input_event ev[MAX_COUNT];
	int num, i;

	if (count < sizeof(struct input_event))
		return -EINVAL;
	num = count / sizeof(struct input_event);
	if (MAX_COUNT < num)
		num = MAX_COUNT;
	if (copy_from_user(ev, buf, num * sizeof(struct input_event)))
		return -EFAULT;

	list_for_each_entry(kdev, &data->devfile_list, list) {
		mutex_lock(&kdev->lock);
		for (i = 0; i < num; i++)
			sensor_enq(kdev, &ev[i]);
		mutex_unlock(&kdev->lock);
		wake_up_interruptible(&kdev->waitq);
	}

	return count;
}

static ssize_t
sensor_read(struct file *f, char __user *buf, size_t count, loff_t *pos)
{
	struct sensor_device *kdev = f->private_data;
	int rt, num;
	struct input_event ev[MAX_COUNT];

	if (count < sizeof(struct input_event))
		return -EINVAL;

	rt = wait_event_interruptible(kdev->waitq, kdev->num_event != 0);
	if (rt)
		return rt;

	mutex_lock(&kdev->lock);
	for (num = 0; num < count / sizeof(struct input_event); num++)
		if (!sensor_deq(kdev, &ev[num]))
			break;
	mutex_unlock(&kdev->lock);

	if (copy_to_user(buf, ev, num * sizeof(struct input_event)))
		return -EFAULT;

	return num * sizeof(struct input_event);
}

static unsigned int
sensor_poll(struct file *f, struct poll_table_struct *wait)
{
	struct sensor_device *kdev = f->private_data;

	poll_wait(f, &kdev->waitq, wait);
	if (kdev->num_event != 0)
		return POLLIN | POLLRDNORM;

	return 0;
}

static int sensor_open(struct inode *inode, struct file *f)
{
	struct geomagnetic_data *data = i2c_get_clientdata(this_client);
	struct sensor_device *kdev;

	kdev = kzalloc(sizeof(struct sensor_device), GFP_KERNEL);
	if (!kdev)
		return -ENOMEM;

	mutex_init(&kdev->lock);
	init_waitqueue_head(&kdev->waitq);
	f->private_data = kdev;
	kdev->head = 0;
	kdev->num_event = 0;
	list_add(&kdev->list, &data->devfile_list);

	return 0;
}

static int sensor_release(struct inode *inode, struct file *f)
{
	struct sensor_device *kdev = f->private_data;

	list_del(&kdev->list);
	kfree(kdev);

	return 0;
}

static int sensor_raw_open(struct inode *inode, struct file *f)
{
	struct geomagnetic_data *data = i2c_get_clientdata(this_client);
	struct sensor_device *kdev;

	kdev = kzalloc(sizeof(struct sensor_device), GFP_KERNEL);
	if (!kdev)
		return -ENOMEM;

	mutex_init(&kdev->lock);
	init_waitqueue_head(&kdev->waitq);
	f->private_data = kdev;
	kdev->head = 0;
	kdev->num_event = 0;
	list_add(&kdev->list, &data->raw_devfile_list);

	return 0;
}

static const struct file_operations sensor_fops = {
	.owner = THIS_MODULE,
	.open = sensor_open,
	.release = sensor_release,
	.write = sensor_write,
	.read = sensor_read,
	.poll = sensor_poll,
};

static struct miscdevice sensor_devfile = {
	.name = SENSOR_NAME,
	.fops = &sensor_fops,
	.minor = MISC_DYNAMIC_MINOR,
};

static const struct file_operations sensor_raw_fops = {
	.owner = THIS_MODULE,
	.open = sensor_raw_open,
	.release = sensor_release,
	.write = sensor_write,
	.read = sensor_read,
	.poll = sensor_poll,
};

static struct miscdevice sensor_raw_devfile = {
	.name = SENSOR_RAW_NAME,
	.fops = &sensor_raw_fops,
	.minor = MISC_DYNAMIC_MINOR,
};

#endif

static int
geomagnetic_i2c_open(void)
{
	return 0;
}

static int
geomagnetic_i2c_close(void)
{
	return 0;
}

#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS529
static int
geomagnetic_i2c_write(const uint8_t *buf, int len)
{
	if (i2c_master_send(this_client, buf, len) < 0)
		return -1;
#if DEBUG
	YLOGD(("[W] [%02x]\n", buf[0]));
#endif

	return 0;
}

static int
geomagnetic_i2c_read(uint8_t *buf, int len)
{
	if (i2c_master_recv(this_client, buf, len) < 0)
		return -1;

#if DEBUG
	if (len == 1) {
		YLOGD(("[R] [%02x]\n", buf[0]));
	} else if (len == 6) {
		YLOGD(("[R] "
		"[%02x%02x%02x%02x%02x%02x]\n",
		buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]));
	} else if (len == 8) {
		YLOGD(("[R] "
		"[%02x%02x%02x%02x%02x%02x%02x%02x]\n",
		buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6],
		buf[7]));
	} else if (len == 9) {
		YLOGD(("[R] "
		"[%02x%02x%02x%02x%02x%02x%02x%02x%02x]\n",
		buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7],
		buf[8]));
	} else if (len == 14) {
		YLOGD(("[R] addr[%02x] "
		"[%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x]\n",
		addr,
		buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7],
		buf[8], buf[9], buf[10], buf[11], buf[12], buf[13]));
	} else if (len == 16) {
		YLOGD(("[R] "
		"[%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x"
		"%02x]\n",
		buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7],
		buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14],
		buf[15]));
	}
#endif

	return 0;
}

#else

static int
geomagnetic_i2c_write(uint8_t addr, const uint8_t *buf, int len)
{
	uint8_t tmp[16];

	if (sizeof(tmp) - 1 < len)
		return -1;

	tmp[0] = addr;
	memcpy(&tmp[1], buf, len);

	if (i2c_master_send(this_client, tmp, len + 1) < 0)
		return -1;
#if DEBUG
	YLOGD(("[W] addr[%02x] [%02x]\n", addr, buf[0]));
#endif

	return 0;
}

static int
geomagnetic_i2c_read(uint8_t addr, uint8_t *buf, int len)
{
	struct i2c_msg msg[2];
	int err;

	msg[0].addr = this_client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &addr;
	msg[1].addr = this_client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = buf;

	err = i2c_transfer(this_client->adapter, msg, 2);
	if (err != 2) {
		dev_err(&this_client->dev,
				"i2c_transfer() read error: "
				"slave_addr=%02x, reg_addr=%02x, err=%d\n",
				this_client->addr, addr, err);
		return err;
	}


#if DEBUG
	if (len == 1) {
		YLOGD(("[R] addr[%02x] [%02x]\n", addr, buf[0]));
	} else if (len == 6) {
		YLOGD(("[R] addr[%02x] "
		"[%02x%02x%02x%02x%02x%02x]\n",
		addr, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]));
	} else if (len == 8) {
		YLOGD(("[R] addr[%02x] "
		"[%02x%02x%02x%02x%02x%02x%02x%02x]\n",
		addr, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6],
		buf[7]));
	} else if (len == 9) {
		YLOGD(("[R] addr[%02x] "
		"[%02x%02x%02x%02x%02x%02x%02x%02x%02x]\n",
		addr, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6],
		buf[7], buf[8]));
	} else if (len == 16) {
		YLOGD(("[R] addr[%02x] "
		"[%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x"
		"%02x]\n",
		addr,
		buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7],
		buf[8], buf[9], buf[10], buf[11], buf[12], buf[13], buf[14],
		buf[15]));
	}
#endif

	return 0;
}

#endif

static int
geomagnetic_lock(void)
{
	struct geomagnetic_data *data = NULL;
	int rt;

	if (this_client == NULL)
		return -1;

	data = i2c_get_clientdata(this_client);
	rt = down_interruptible(&data->driver_lock);
	if (rt < 0)
		up(&data->driver_lock);
	return rt;
}

static int
geomagnetic_unlock(void)
{
	struct geomagnetic_data *data = NULL;

	if (this_client == NULL)
		return -1;

	data = i2c_get_clientdata(this_client);
	up(&data->driver_lock);
	return 0;
}

static void
geomagnetic_msleep(int ms)
{
	msleep(ms);
}

static void
geomagnetic_current_time(int32_t *sec, int32_t *msec)
{
	struct timeval tv;

	do_gettimeofday(&tv);

	*sec = tv.tv_sec;
	*msec = tv.tv_usec / 1000;
}

static struct yas_mag_driver hwdriver = {
	.callback = {
		.lock		= geomagnetic_lock,
		.unlock		= geomagnetic_unlock,
		.device_open	= geomagnetic_i2c_open,
		.device_close	= geomagnetic_i2c_close,
		.device_read	= geomagnetic_i2c_read,
		.device_write	= geomagnetic_i2c_write,
		.msleep		= geomagnetic_msleep,
		.current_time	= geomagnetic_current_time,
		.device_set_reset	= kc_geomagnetic_set_reset,
	},
};

static int
geomagnetic_multi_lock(void)
{
	struct geomagnetic_data *data = NULL;
	int rt;

	if (this_client == NULL)
		return -1;

	data = i2c_get_clientdata(this_client);
	rt = down_interruptible(&data->multi_lock);
	if (rt < 0)
		up(&data->multi_lock);
	return rt;
}

static int
geomagnetic_multi_unlock(void)
{
	struct geomagnetic_data *data = NULL;

	if (this_client == NULL)
		return -1;

	data = i2c_get_clientdata(this_client);
	up(&data->multi_lock);
	return 0;
}

static int
geomagnetic_enable(struct geomagnetic_data *data)
{
	YLOGI(("%s(): start\n",__func__));
	if (!atomic_cmpxchg(&data->enable, 0, 1))
		queue_delayed_work(mag_polling_wq, &data->work, 0);

	YLOGI(("%s(): end\n",__func__));
	return 0;
}

static int
geomagnetic_disable(struct geomagnetic_data *data)
{
	YLOGI(("%s(): start\n",__func__));
	if (atomic_cmpxchg(&data->enable, 1, 0))
		cancel_delayed_work_sync(&data->work);

	YLOGI(("%s(): end\n",__func__));
	return 0;
}

static int32_t
kc_geomagnetic_disable(struct geomagnetic_data *data)
{
	YLOGI(("%s(): start\n",__func__));
	if (atomic_cmpxchg(&data->enable, 1, 0))
		cancel_delayed_work(&data->work);

	YLOGI(("%s(): end\n",__func__));
	return 0;
}

static int32_t kc_geomagnetic_set_reset(int32_t reset)
{
	struct geomagnetic_data *data = i2c_get_clientdata(this_client);
	YLOGI(("%s(): start reset=%d\n",__func__,reset));

	if (reset) {
		if (hwdriver.get_enable()) {
			hwdriver.set_reset(1);
			geomagnetic_enable(data);
		}
	} else {
		kc_geomagnetic_disable(data);
		hwdriver.set_reset(0);
	}

	YLOGI(("%s(): end\n",__func__));
	return 0;
}

/* Sysfs interface */
static ssize_t
geomagnetic_delay_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct geomagnetic_data *data = input_get_drvdata(input_data);
	int delay;

	YLOGI(("%s(): start\n",__func__));
	geomagnetic_multi_lock();

	delay = data->delay;

	geomagnetic_multi_unlock();
	YLOGI(("%s(): end\n",__func__));

	return sprintf(buf, "%d\n", delay);
}

static ssize_t
geomagnetic_delay_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct geomagnetic_data *data = input_get_drvdata(input_data);
	long value;

	YLOGI(("%s(): start\n",__func__));
	if (hwdriver.set_delay == NULL)
		return -ENOTTY;
	if (strict_strtol(buf, 10, &value) < 0)
		return -EINVAL;

	geomagnetic_multi_lock();

	if (hwdriver.set_delay(value) == 0)
		data->delay = value;

	geomagnetic_multi_unlock();

	YLOGI(("%s(): end\n",__func__));
	return count;
}

static ssize_t
geomagnetic_enable_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct geomagnetic_data *data = input_get_drvdata(input_data);
	YLOGI(("%s(): start\n",__func__));

	return sprintf(buf, "%d\n", atomic_read(&data->enable));
}

static ssize_t
geomagnetic_enable_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct geomagnetic_data *data = input_get_drvdata(input_data);
	long value;
	YLOGI(("%s(): [IN]\n",__func__));

	if (hwdriver.set_enable == NULL)
		return -ENOTTY;
	if (strict_strtol(buf, 10, &value) < 0)
		return -EINVAL;
	value = !!value;

	YLOGI(("%s(): enable = %ld\n",__func__,value));
	if (geomagnetic_multi_lock() < 0)
		return count;
	if (hwdriver.set_enable(value) == 0) {
		if (value)
			geomagnetic_enable(data);
		else
			geomagnetic_disable(data);
	}

	geomagnetic_multi_unlock();
	YLOGI(("%s(): [OUT]\n",__func__));

	return count;
}

static ssize_t
geomagnetic_filter_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct geomagnetic_data *data = input_get_drvdata(input_data);
	int filter_enable;
	YLOGI(("%s(): start\n",__func__));

	geomagnetic_multi_lock();

	filter_enable = data->filter_enable;

	geomagnetic_multi_unlock();
	YLOGI(("%s(): end filter_enable = %d\n",__func__,filter_enable));

	return sprintf(buf, "%d\n", filter_enable);
}

static ssize_t
geomagnetic_filter_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct geomagnetic_data *data = input_get_drvdata(input_data);
	long value;
	YLOGI(("%s(): start\n",__func__));

	if (hwdriver.set_filter_enable == NULL)
		return -ENOTTY;
	if (strict_strtol(buf, 10, &value) < 0)
		return -EINVAL;

	if (geomagnetic_multi_lock() < 0)
		return count;

	if (hwdriver.set_filter_enable(value) == 0)
		data->filter_enable = !!value;

	geomagnetic_multi_unlock();
	YLOGI(("%s(): end\n",__func__));

	return count;
}

static ssize_t
geomagnetic_filter_len_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct geomagnetic_data *data = input_get_drvdata(input_data);
	int filter_len;
	YLOGI(("%s(): start\n",__func__));

	geomagnetic_multi_lock();

	filter_len = data->filter_len;

	geomagnetic_multi_unlock();
	YLOGI(("%s(): end\n",__func__));

	return sprintf(buf, "%d\n", filter_len);
}

static ssize_t
geomagnetic_filter_len_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct geomagnetic_data *data = input_get_drvdata(input_data);
	struct yas_mag_filter filter;
	long value;
	YLOGI(("%s(): start\n",__func__));

	if (hwdriver.get_filter == NULL || hwdriver.set_filter == NULL)
		return -ENOTTY;
	if (strict_strtol(buf, 10, &value) < 0)
		return -EINVAL;

	if (geomagnetic_multi_lock() < 0)
		return count;

	hwdriver.get_filter(&filter);
	filter.len = value;
	if (hwdriver.set_filter(&filter) == 0)
		data->filter_len = value;

	geomagnetic_multi_unlock();
	YLOGI(("%s(): end\n",__func__));

	return count;
}

static ssize_t
geomagnetic_filter_noise_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	struct input_dev *input_raw = to_input_dev(dev);
	struct geomagnetic_data *data = input_get_drvdata(input_raw);
	int rt;

	YLOGI(("%s(): start\n",__func__));
	geomagnetic_multi_lock();

	rt = sprintf(buf, "%d %d %d\n", data->filter_noise[0],
			data->filter_noise[1], data->filter_noise[2]);

	geomagnetic_multi_unlock();
	YLOGI(("%s(): end\n",__func__));

	return rt;
}

static ssize_t
geomagnetic_filter_noise_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct input_dev *input_raw = to_input_dev(dev);
	struct yas_mag_filter filter;
	struct geomagnetic_data *data = input_get_drvdata(input_raw);
	int32_t noise[3];
	YLOGI(("%s(): start\n",__func__));

	geomagnetic_multi_lock();

	sscanf(buf, "%d %d %d", &noise[0], &noise[1], &noise[2]);
	hwdriver.get_filter(&filter);
	memcpy(filter.noise, noise, sizeof(filter.noise));
	if (hwdriver.set_filter(&filter) == 0)
		memcpy(data->filter_noise, noise, sizeof(data->filter_noise));

	geomagnetic_multi_unlock();
	YLOGI(("%s(): end\n",__func__));

	return count;
}

static ssize_t
geomagnetic_filter_threshold_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct geomagnetic_data *data = input_get_drvdata(input_data);
	int32_t filter_threshold;
	YLOGI(("%s(): start\n",__func__));

	geomagnetic_multi_lock();

	filter_threshold = data->filter_threshold;

	geomagnetic_multi_unlock();
	YLOGI(("%s(): end\n",__func__));

	return sprintf(buf, "%d\n", filter_threshold);
}

static ssize_t
geomagnetic_filter_threshold_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct geomagnetic_data *data = input_get_drvdata(input_data);
	struct yas_mag_filter filter;
	long value;
	YLOGI(("%s(): start\n",__func__));

	if (hwdriver.get_filter == NULL || hwdriver.set_filter == NULL)
		return -ENOTTY;
	if (strict_strtol(buf, 10, &value) < 0)
		return -EINVAL;

	if (geomagnetic_multi_lock() < 0)
		return count;

	hwdriver.get_filter(&filter);
	filter.threshold = value;
	if (hwdriver.set_filter(&filter) == 0)
		data->filter_threshold = value;

	geomagnetic_multi_unlock();
	YLOGI(("%s(): end\n",__func__));

	return count;
}

static ssize_t
geomagnetic_position_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	if (hwdriver.get_position == NULL)
		return -ENOTTY;
	YLOGI(("%s(): start\n",__func__));
	return sprintf(buf, "%d\n", hwdriver.get_position());
}

static ssize_t
geomagnetic_position_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	long value;
	YLOGI(("%s(): start\n",__func__));

	if (hwdriver.set_position == NULL)
		return -ENOTTY;
	if (strict_strtol(buf, 10, &value) < 0)
		return -EINVAL;

	hwdriver.set_position(value);

	YLOGI(("%s(): end\n",__func__));
	return count;
}

static ssize_t
geomagnetic_data_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct geomagnetic_data *data = input_get_drvdata(input_data);
	int rt;
	YLOGI(("%s(): start\n",__func__));

	rt = sprintf(buf, "%d %d %d\n", atomic_read(&data->last_data[0]),
			atomic_read(&data->last_data[1]),
			atomic_read(&data->last_data[2]));

	return rt;
}

static ssize_t
geomagnetic_status_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct geomagnetic_data *data = input_get_drvdata(input_data);
	int rt;
	YLOGI(("%s(): start\n",__func__));

	rt = sprintf(buf, "%d\n", atomic_read(&data->last_status));

	return rt;
}

static ssize_t
geomagnetic_status_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct geomagnetic_data *data = input_get_drvdata(input_data);
	static int16_t cnt = 1;
#ifdef YAS_SENSOR_KERNEL_DEVFILE_INTERFACE
	struct input_event ev;
#endif
	int accuracy = 0;
	int code = 0;
	int value = 0;

	geomagnetic_multi_lock();

	sscanf(buf, "%d", &accuracy);
	if (0 <= accuracy && accuracy <= 3)
		atomic_set(&data->last_status, accuracy);
	code |= YAS_REPORT_CALIB_OFFSET_CHANGED;
	value = (cnt++ << 16) | (code);

#ifdef YAS_SENSOR_KERNEL_DEVFILE_INTERFACE
	mkev(&ev, EV_ABS, ABS_RAW_REPORT, value);
	sensor_event(&data->raw_devfile_list, &ev, 1);
#else
	input_report_abs(data->input_raw, ABS_RAW_REPORT, value);
	input_sync(data->input_raw);
#endif

	geomagnetic_multi_unlock();

	return count;
}

static ssize_t
geomagnetic_wake_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct geomagnetic_data *data = input_get_drvdata(input_data);
	static int16_t cnt = 1;
	YLOGI(("%s(): start\n",__func__));
#ifdef YAS_SENSOR_KERNEL_DEVFILE_INTERFACE
	struct input_event ev[1];
	mkev(ev, EV_ABS, ABS_WAKE, cnt++);
	sensor_event(&data->devfile_list, ev, 1);
#else
	input_report_abs(data->input_data, ABS_WAKE, cnt++);
	input_sync(data->input_data);
#endif

	return count;
}

#if DEBUG

static int geomagnetic_suspend(struct i2c_client *client, pm_message_t mesg);
static int geomagnetic_resume(struct i2c_client *client);

static ssize_t
geomagnetic_debug_suspend_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct geomagnetic_data *data = input_get_drvdata(input);

	return sprintf(buf, "%d\n", data->suspend);
}

static ssize_t
geomagnetic_debug_suspend_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	long suspend;

	if (strict_strtol(buf, 10, &suspend) < 0)
		return -EINVAL;
	if (suspend) {
		pm_message_t msg;
		memset(&msg, 0, sizeof(msg));
		geomagnetic_suspend(this_client, msg);
	} else {
		geomagnetic_resume(this_client);
	}

	return count;
}

#endif /* DEBUG */

static DEVICE_ATTR(delay, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
	geomagnetic_delay_show, geomagnetic_delay_store);
static DEVICE_ATTR(enable, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
	geomagnetic_enable_show, geomagnetic_enable_store);
static DEVICE_ATTR(filter_enable, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
	geomagnetic_filter_enable_show,
	geomagnetic_filter_enable_store);
static DEVICE_ATTR(filter_len, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
	geomagnetic_filter_len_show, geomagnetic_filter_len_store);
static DEVICE_ATTR(filter_threshold, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
	geomagnetic_filter_threshold_show,
	geomagnetic_filter_threshold_store);
static DEVICE_ATTR(filter_noise, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
	geomagnetic_filter_noise_show, geomagnetic_filter_noise_store);
static DEVICE_ATTR(data, S_IRUSR|S_IRGRP, geomagnetic_data_show, NULL);
static DEVICE_ATTR(status, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP, geomagnetic_status_show,
		geomagnetic_status_store);
static DEVICE_ATTR(wake, S_IWUSR|S_IWGRP, NULL, geomagnetic_wake_store);
static DEVICE_ATTR(position, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
	geomagnetic_position_show, geomagnetic_position_store);
#if DEBUG
static DEVICE_ATTR(debug_suspend, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
	geomagnetic_debug_suspend_show, geomagnetic_debug_suspend_store);
#endif /* DEBUG */

static struct attribute *geomagnetic_attributes[] = {
	&dev_attr_delay.attr,
	&dev_attr_enable.attr,
	&dev_attr_filter_enable.attr,
	&dev_attr_filter_len.attr,
	&dev_attr_filter_threshold.attr,
	&dev_attr_filter_noise.attr,
	&dev_attr_data.attr,
	&dev_attr_status.attr,
	&dev_attr_wake.attr,
	&dev_attr_position.attr,
#if DEBUG
	&dev_attr_debug_suspend.attr,
#endif /* DEBUG */
	NULL
};

static struct attribute_group geomagnetic_attribute_group = {
	.attrs = geomagnetic_attributes
};

static ssize_t
geomagnetic_raw_data_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct geomagnetic_data *data = input_get_drvdata(input_data);
	int rt;

	rt = sprintf(buf, "%d %d %d\n", atomic_read(&data->last_raw[0]),
			atomic_read(&data->last_raw[1]),
			atomic_read(&data->last_raw[2]));

	return rt;
}

static ssize_t
geomagnetic_raw_gyro_threshold_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct input_dev *input_raw = to_input_dev(dev);
	struct geomagnetic_data *data = input_get_drvdata(input_raw);
	int mag_noise, gyro_noise;

	geomagnetic_multi_lock();

	mag_noise = data->mag_noise;
	gyro_noise = data->gyro_noise;

	geomagnetic_multi_unlock();

	return sprintf(buf, "%d %d\n", mag_noise, gyro_noise);
}

static ssize_t
geomagnetic_raw_gyro_threshold_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct input_dev *input_raw = to_input_dev(dev);
	struct geomagnetic_data *data = input_get_drvdata(input_raw);
	static int cnt = 1;
	int32_t mag_noise = 0, gyro_noise = 0;
#ifdef YAS_SENSOR_KERNEL_DEVFILE_INTERFACE
	struct input_event ev[1];
#endif

	sscanf(buf, "%d %d", &mag_noise, &gyro_noise);
	if (mag_noise < 0 || SHRT_MAX < mag_noise
			|| gyro_noise < 0 || SHRT_MAX < gyro_noise)
		return -EINVAL;

	geomagnetic_multi_lock();

	data->mag_noise = mag_noise;
	data->gyro_noise = gyro_noise;
#ifdef YAS_SENSOR_KERNEL_DEVFILE_INTERFACE
	mkev(ev, EV_ABS, ABS_RAW_GYRO_THRESHOLD, cnt++);
	sensor_event(&data->raw_devfile_list, ev, 1);
#else
	input_report_abs(data->input_raw, ABS_RAW_GYRO_THRESHOLD, cnt++);
	input_sync(data->input_raw);
#endif

	geomagnetic_multi_unlock();

	return count;
}

static ssize_t
geomagnetic_raw_threshold_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct input_dev *input_raw = to_input_dev(dev);
	struct geomagnetic_data *data = input_get_drvdata(input_raw);
	int threshold;

	geomagnetic_multi_lock();

	threshold = data->threshold;

	geomagnetic_multi_unlock();

	return sprintf(buf, "%d\n", threshold);
}

static ssize_t
geomagnetic_raw_threshold_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct input_dev *input_raw = to_input_dev(dev);
	struct geomagnetic_data *data = input_get_drvdata(input_raw);
	static int cnt = 1;
	long value;
#ifdef YAS_SENSOR_KERNEL_DEVFILE_INTERFACE
	struct input_event ev[1];
#endif

	if (strict_strtol(buf, 10, &value) < 0)
		return -EINVAL;
	if (value < 0 || 2 <  value)
		return -EINVAL;

	geomagnetic_multi_lock();

	data->threshold = value;
#ifdef YAS_SENSOR_KERNEL_DEVFILE_INTERFACE
	mkev(ev, EV_ABS, ABS_RAW_THRESHOLD, cnt++);
	sensor_event(&data->raw_devfile_list, ev, 1);
#else
	input_report_abs(data->input_raw, ABS_RAW_THRESHOLD, cnt++);
	input_sync(data->input_raw);
	data->input_event_updated |= GEO_EV_UPDATED_THRESHOLD;
	if (data->input_event_log)
	{
		printk("[YAS_MAG_EVENT] ABS_RAW_THRESHOLD value = %ld \n",value);
	}
#endif

	geomagnetic_multi_unlock();

	return count;
}

static ssize_t
geomagnetic_raw_distortion_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct input_dev *input_raw = to_input_dev(dev);
	struct geomagnetic_data *data = input_get_drvdata(input_raw);
	int rt;

	geomagnetic_multi_lock();

	rt = sprintf(buf, "%d %d %d\n", data->distortion[0],
			data->distortion[1], data->distortion[2]);

	geomagnetic_multi_unlock();

	return rt;
}

static ssize_t
geomagnetic_raw_distortion_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct input_dev *input_raw = to_input_dev(dev);
	struct geomagnetic_data *data = input_get_drvdata(input_raw);
	int32_t distortion[3];
	static int32_t cnt = 1;
	int i;
#ifdef YAS_SENSOR_KERNEL_DEVFILE_INTERFACE
	struct input_event ev[1];
#endif

	sscanf(buf, "%d %d %d", &distortion[0], &distortion[1], &distortion[2]);
	if (distortion[0] <= 0 || distortion[1] <= 0 || distortion[2] <= 0)
		return -EINVAL;

	geomagnetic_multi_lock();

	for (i = 0; i < 3; i++)
		data->distortion[i] = distortion[i];
#ifdef YAS_SENSOR_KERNEL_DEVFILE_INTERFACE
	mkev(ev, EV_ABS, ABS_RAW_DISTORTION, cnt++);
	sensor_event(&data->raw_devfile_list, ev, 1);
#else
	input_report_abs(data->input_raw, ABS_RAW_DISTORTION, cnt++);
	input_sync(data->input_raw);
	data->input_event_updated |= GEO_EV_UPDATED_DISTORTION;
	if (data->input_event_log)
	{
		printk("[YAS_MAG_EVENT] ABS_RAW_DISTORTION dis1 = %d, dis2 = %d, dis3 = %d \n",
		       distortion[0], distortion[1], distortion[2]);
	}
#endif

	geomagnetic_multi_unlock();

	return count;
}

static ssize_t
geomagnetic_raw_shape_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	struct input_dev *input_raw = to_input_dev(dev);
	struct geomagnetic_data *data = input_get_drvdata(input_raw);
	int shape;

	YLOGI(("%s(): start\n",__func__));
	geomagnetic_multi_lock();

	shape = data->shape;

	geomagnetic_multi_unlock();

	YLOGI(("%s(): end\n",__func__));
	return sprintf(buf, "%d\n", shape);
}

static ssize_t
geomagnetic_raw_shape_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct input_dev *input_raw = to_input_dev(dev);
	struct geomagnetic_data *data = input_get_drvdata(input_raw);
	long value;
	static int cnt = 1;
#ifdef YAS_SENSOR_KERNEL_DEVFILE_INTERFACE
	struct input_event ev[1];
#endif

	YLOGI(("%s(): start\n",__func__));
	if (strict_strtol(buf, 10, &value) < 0)
		return -EINVAL;
	if (value < 0 || 1 < value)
		return -EINVAL;

	geomagnetic_multi_lock();

	data->shape = value;
#ifdef YAS_SENSOR_KERNEL_DEVFILE_INTERFACE
	mkev(ev, EV_ABS, ABS_RAW_SHAPE, cnt++);
	sensor_event(&data->raw_devfile_list, ev, 1);
#else
	input_report_abs(data->input_raw, ABS_RAW_SHAPE, cnt++);
	input_sync(data->input_raw);
#endif

	geomagnetic_multi_unlock();

	YLOGI(("%s(): end\n",__func__));
	return count;
}

static ssize_t
geomagnetic_raw_offsets_show(struct device *dev, struct device_attribute *attr,
	char *buf)
{
	struct input_dev *input_raw = to_input_dev(dev);
	struct geomagnetic_data *data = input_get_drvdata(input_raw);
	struct yas_mag_offset offset;
	int accuracy;

	YLOGI(("%s(): start\n",__func__));
	geomagnetic_multi_lock();

	offset = data->driver_offset;
	accuracy = atomic_read(&data->last_status);

	geomagnetic_multi_unlock();

	YLOGI(("%s(): end\n",__func__));
	return sprintf(buf, "%d %d %d %d %d %d %d\n", offset.hard_offset[0],
			offset.hard_offset[1], offset.hard_offset[2],
			offset.calib_offset.v[0], offset.calib_offset.v[1],
			offset.calib_offset.v[2], accuracy);
}

static ssize_t
geomagnetic_raw_offsets_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct input_dev *input_raw = to_input_dev(dev);
	struct geomagnetic_data *data = input_get_drvdata(input_raw);
	struct yas_mag_offset offset;
	int32_t hard_offset[3];
	int i, accuracy;
	YLOGI(("%s(): start\n",__func__));

	geomagnetic_multi_lock();

	sscanf(buf, "%d %d %d %d %d %d %d", &hard_offset[0], &hard_offset[1],
			&hard_offset[2], &offset.calib_offset.v[0],
			&offset.calib_offset.v[1], &offset.calib_offset.v[2],
			&accuracy);
	if (0 <= accuracy && accuracy <= 3) {
		for (i = 0; i < 3; i++)
			offset.hard_offset[i] = (int8_t)hard_offset[i];
		if (hwdriver.set_offset(&offset) == 0) {
			atomic_set(&data->last_status, accuracy);
			data->driver_offset = offset;
		}
	}

	geomagnetic_multi_unlock();
	YLOGD(("geomagnetic_raw_offsets_store[%d][%d][%d][%d][%d][%d][%d]\n",
				hard_offset[0], hard_offset[1], hard_offset[2],
				offset.calib_offset.v[0],
				offset.calib_offset.v[1],
				offset.calib_offset.v[2],
				accuracy));
	YLOGI(("%s(): end\n",__func__));

	return count;
}

#ifdef YAS_MAG_MANUAL_OFFSET
static ssize_t
geomagnetic_raw_manual_offsets_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct input_dev *input_raw = to_input_dev(dev);
	struct geomagnetic_data *data = input_get_drvdata(input_raw);
	struct yas_vector offset;

	YLOGI(("%s(): start\n",__func__));
	geomagnetic_multi_lock();

	offset = data->manual_offset;

	geomagnetic_multi_unlock();

	YLOGI(("%s(): end\n",__func__));
	return sprintf(buf, "%d %d %d\n", offset.v[0], offset.v[1],
			offset.v[2]);
}

static ssize_t
geomagnetic_raw_manual_offsets_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct input_dev *input_raw = to_input_dev(dev);
	struct geomagnetic_data *data = input_get_drvdata(input_raw);
	struct yas_vector offset;
	YLOGI(("%s(): start\n",__func__));

	geomagnetic_multi_lock();

	sscanf(buf, "%d %d %d", &offset.v[0], &offset.v[1], &offset.v[2]);
	if (hwdriver.set_manual_offset(&offset) == 0)
		data->manual_offset = offset;

	geomagnetic_multi_unlock();

	YLOGI(("%s(): end\n",__func__));
	return count;
}
#endif

static ssize_t
geomagnetic_raw_static_matrix_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct input_dev *input_raw = to_input_dev(dev);
	struct geomagnetic_data *data = input_get_drvdata(input_raw);
	struct yas_matrix matrix;

	YLOGI(("%s(): start\n",__func__));
	geomagnetic_multi_lock();

	matrix = data->static_matrix;

	geomagnetic_multi_unlock();

	YLOGI(("%s(): end\n",__func__));
	return sprintf(buf, "%d %d %d %d %d %d %d %d %d\n", matrix.matrix[0],
			matrix.matrix[1], matrix.matrix[2], matrix.matrix[3],
			matrix.matrix[4], matrix.matrix[5], matrix.matrix[6],
			matrix.matrix[7], matrix.matrix[8]);
}

static ssize_t
geomagnetic_raw_static_matrix_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct input_dev *input_raw = to_input_dev(dev);
	struct geomagnetic_data *data = input_get_drvdata(input_raw);
	struct yas_matrix matrix;

	YLOGI(("%s(): start\n",__func__));
	geomagnetic_multi_lock();

	sscanf(buf, "%d %d %d %d %d %d %d %d %d", &matrix.matrix[0],
			&matrix.matrix[1], &matrix.matrix[2],
			&matrix.matrix[3], &matrix.matrix[4], &matrix.matrix[5],
			&matrix.matrix[6], &matrix.matrix[7],
			&matrix.matrix[8]);
	if (hwdriver.set_static_matrix(&matrix) == 0)
		data->static_matrix = matrix;

	geomagnetic_multi_unlock();

	YLOGI(("%s(): end\n",__func__));
	return count;
}

static ssize_t
geomagnetic_raw_dynamic_matrix_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct input_dev *input_raw = to_input_dev(dev);
	struct geomagnetic_data *data = input_get_drvdata(input_raw);
	struct yas_matrix matrix;

	YLOGI(("%s(): start\n",__func__));
	geomagnetic_multi_lock();

	matrix = data->dynamic_matrix;

	geomagnetic_multi_unlock();

	YLOGI(("%s(): end\n",__func__));
	return sprintf(buf, "%d %d %d %d %d %d %d %d %d\n", matrix.matrix[0],
			matrix.matrix[1], matrix.matrix[2], matrix.matrix[3],
			matrix.matrix[4], matrix.matrix[5], matrix.matrix[6],
			matrix.matrix[7], matrix.matrix[8]);
}

static ssize_t
geomagnetic_raw_dynamic_matrix_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct input_dev *input_raw = to_input_dev(dev);
	struct geomagnetic_data *data = input_get_drvdata(input_raw);
	struct yas_matrix matrix;

	YLOGI(("%s(): start\n",__func__));
	geomagnetic_multi_lock();

	sscanf(buf, "%d %d %d %d %d %d %d %d %d", &matrix.matrix[0],
			&matrix.matrix[1], &matrix.matrix[2],
			&matrix.matrix[3], &matrix.matrix[4], &matrix.matrix[5],
			&matrix.matrix[6], &matrix.matrix[7],
			&matrix.matrix[8]);
	if (hwdriver.set_dynamic_matrix(&matrix) == 0)
		data->dynamic_matrix = matrix;

	geomagnetic_multi_unlock();

	YLOGI(("%s(): end\n",__func__));
	return count;
}

static ssize_t
geomagnetic_raw_ellipsoid_mode_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct input_dev *input_raw = to_input_dev(dev);
	struct geomagnetic_data *data = input_get_drvdata(input_raw);
	int ellipsoid_mode;

	YLOGI(("%s(): start\n",__func__));
	geomagnetic_multi_lock();

	ellipsoid_mode = data->ellipsoid_mode;

	geomagnetic_multi_unlock();

	YLOGI(("%s(): end\n",__func__));
	return sprintf(buf, "%d\n", ellipsoid_mode);
}

static ssize_t
geomagnetic_raw_ellipsoid_mode_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct input_dev *input_raw = to_input_dev(dev);
	struct geomagnetic_data *data = input_get_drvdata(input_raw);
	long value;
	static int cnt = 1;
#ifdef YAS_SENSOR_KERNEL_DEVFILE_INTERFACE
	struct input_event ev[1];
#endif

	YLOGI(("%s(): start\n",__func__));
	if (strict_strtol(buf, 10, &value) < 0)
		return -EINVAL;
	if (value < 0 || 4 < value)
		return -EINVAL;

	geomagnetic_multi_lock();

	data->ellipsoid_mode = value;
#ifdef YAS_SENSOR_KERNEL_DEVFILE_INTERFACE
	mkev(ev, EV_ABS, ABS_RAW_MODE, cnt++);
	sensor_event(&data->raw_devfile_list, ev, 1);
#else
	input_report_abs(data->input_raw, ABS_RAW_MODE, cnt++);
	input_sync(data->input_raw);
#endif

	geomagnetic_multi_unlock();

	YLOGI(("%s(): end\n",__func__));
	return count;
}

static ssize_t geomagnetic_raw_data_all_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct yas_mag_data magdata;
	int time_delay_ms = 100;
	int rt;
	YLOGI(("%s(): [IN]\n",__func__));

	rt = hwdriver.measure(&magdata, &time_delay_ms);
	if (rt < 0) {
		YLOGE(("measure failed[%d]\n", rt));
	}
	YLOGI(("%s(): [OUT]\n",__func__));
	return sprintf(buf, "%d %d %d %d %d %d %d %d %d\n",
		magdata.xyz.v[0],magdata.xyz.v[1],magdata.xyz.v[2],
		magdata.raw.v[0],magdata.raw.v[1],magdata.raw.v[2],
		magdata.xy1y2.v[0], magdata.xy1y2.v[1],magdata.xy1y2.v[2]);
}

static ssize_t geomagnetic_ope_device_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int number;
	YLOGI(("%s(): [IN]\n",__func__));

	sscanf(buf, "%d", &number);
	YLOGD(("geomagnetic_ope_device_store [number = %d]\n", number));

	switch (number)
	{
	case KC_SENSOR_COMMON_POWER_ON:
		hwdriver.set_vsensor(1);
		break;
	case KC_SENSOR_COMMON_POWER_OFF:
		hwdriver.set_vsensor(0);
		break;
	case KC_SENSOR_COMMON_INIT:
		hwdriver.init_sequential();
		break;
	default:
		break;
	}
	YLOGI(("%s(): [OUT]\n",__func__));
	return count;

}

static ssize_t geomagnetic_event_log_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct input_dev *input_raw = to_input_dev(dev);
	struct geomagnetic_data *data = input_get_drvdata(input_raw);
	int32_t value;
	
	YLOGI(("%s(): [IN]\n",__func__));
	value = data->input_event_log;
	YLOGI(("%s(): [OUT]\n",__func__));

	return sprintf(buf, "%d\n",value);
}

static ssize_t geomagnetic_event_log_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct input_dev *input_data = to_input_dev(dev);
	struct geomagnetic_data *data = input_get_drvdata(input_data);
	int32_t value;

	YLOGI(("%s(): start\n",__func__));
	sscanf(buf, "%d", &value);
	data->input_event_log = value;
	YLOGI(("%s(): end\n",__func__));
	return count;
}

static ssize_t geomagnetic_event_updated_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	struct input_dev *input_raw = to_input_dev(dev);
	struct geomagnetic_data *data = input_get_drvdata(input_raw);
	int32_t value;
	
	YLOGI(("%s(): [IN]\n",__func__));
	value = data->input_event_updated;
	YLOGI(("%s(): [OUT]\n",__func__));

	return sprintf(buf, "%d\n",value);
}

static ssize_t geomagnetic_pcb_data_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	ssize_t ret=0;
    char tmp_buf1[30];
    char tmp_buf2[30];
    char tmp_buf3[30];

	YLOGI(("%s(): [IN]\n",__func__));

	memset(tmp_buf1, 0, sizeof(tmp_buf1));
	memset(tmp_buf2, 0, sizeof(tmp_buf2));
	memset(tmp_buf3, 0, sizeof(tmp_buf3));

	geomagnetic_i2c_read(0x80, tmp_buf1, 1);

	geomagnetic_i2c_read(0x90, tmp_buf2, 14);
	geomagnetic_i2c_read(0x90, tmp_buf2, 14);

	geomagnetic_i2c_read(0xb0, tmp_buf3, 8);

	ret = sprintf(buf, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d ",
	tmp_buf1[0], 
	tmp_buf2[0],tmp_buf2[1],tmp_buf2[2],tmp_buf2[3],tmp_buf2[4],tmp_buf2[5],
	tmp_buf2[6],tmp_buf2[7],tmp_buf2[8],tmp_buf2[9],tmp_buf2[10],tmp_buf2[11],
	tmp_buf2[12],tmp_buf2[13],
	tmp_buf3[0],tmp_buf3[1],tmp_buf3[2], tmp_buf3[3],tmp_buf3[4],tmp_buf3[5],
	tmp_buf3[6],tmp_buf3[7]);

    YLOGI(("%s(): [OUT]\n",__func__));

	return ret;
}
static ssize_t geomagnetic_pcb_data_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int ret=0;
	int req[4] = {0};

	YLOGI(("%s(): [IN]\n",__func__));

	sscanf(buf, "%d %d %d %d", &req[0], &req[1], &req[2], &req[3]);
    YLOGI(("%s:LINE=%d req[0]=%d req[1]=%d req[2]=%d req[3]=%d \n",
			__FUNCTION__,__LINE__,req[0],req[1],req[2],req[3]));

	switch (req[0])
	{
	case 0:
		ret = geomagnetic_i2c_open();
		break;
	case 1:
		ret = geomagnetic_i2c_close();
		break;
	case 2:
		ret = geomagnetic_i2c_write(req[1], (const uint8_t *)&req[3], req[2]);
		break;
	case 3:
		geomagnetic_msleep(req[1]);
		break;
	default:
		break;
	}

	YLOGI(("%s(): [OUT]\n",__func__));
	return count;
}

static DEVICE_ATTR(data_raw, S_IRUSR|S_IRGRP,
	geomagnetic_raw_data_show, NULL);
static DEVICE_ATTR(threshold, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
	geomagnetic_raw_threshold_show, geomagnetic_raw_threshold_store);
static DEVICE_ATTR(distortion, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
	geomagnetic_raw_distortion_show, geomagnetic_raw_distortion_store);
static DEVICE_ATTR(shape, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
	geomagnetic_raw_shape_show, geomagnetic_raw_shape_store);
static DEVICE_ATTR(offsets, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
	geomagnetic_raw_offsets_show, geomagnetic_raw_offsets_store);
static DEVICE_ATTR(gyro_threshold, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
	geomagnetic_raw_gyro_threshold_show,
	geomagnetic_raw_gyro_threshold_store);
#ifdef YAS_MAG_MANUAL_OFFSET
static DEVICE_ATTR(manual_offsets, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
	geomagnetic_raw_manual_offsets_show,
	geomagnetic_raw_manual_offsets_store);
#endif
static DEVICE_ATTR(static_matrix, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
	geomagnetic_raw_static_matrix_show,
	geomagnetic_raw_static_matrix_store);
static DEVICE_ATTR(dynamic_matrix, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
	geomagnetic_raw_dynamic_matrix_show,
	geomagnetic_raw_dynamic_matrix_store);
static DEVICE_ATTR(ellipsoid_mode, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
	geomagnetic_raw_ellipsoid_mode_show,
	geomagnetic_raw_ellipsoid_mode_store);

static DEVICE_ATTR(raw_data, S_IRUSR|S_IRGRP,
	geomagnetic_raw_data_all_show,
	NULL);
static DEVICE_ATTR(ope_device, S_IWUSR|S_IWGRP,
	NULL,
	geomagnetic_ope_device_store);
static DEVICE_ATTR(event_log, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
	geomagnetic_event_log_show,
	geomagnetic_event_log_store);
static DEVICE_ATTR(event_updated, S_IRUSR|S_IRGRP,
	geomagnetic_event_updated_show,
	NULL);
static DEVICE_ATTR(pcb_test, 0660,
	geomagnetic_pcb_data_show,
	geomagnetic_pcb_data_store);

static struct attribute *geomagnetic_raw_attributes[] = {
	&dev_attr_data_raw.attr,
	&dev_attr_gyro_threshold.attr,
	&dev_attr_threshold.attr,
	&dev_attr_distortion.attr,
	&dev_attr_shape.attr,
	&dev_attr_offsets.attr,
#ifdef YAS_MAG_MANUAL_OFFSET
	&dev_attr_manual_offsets.attr,
#endif
	&dev_attr_static_matrix.attr,
	&dev_attr_dynamic_matrix.attr,
	&dev_attr_ellipsoid_mode.attr,
	&dev_attr_raw_data.attr,
	&dev_attr_ope_device.attr,
	&dev_attr_event_log.attr,
	&dev_attr_event_updated.attr,
	&dev_attr_pcb_test.attr,
	NULL
};

static struct attribute_group geomagnetic_raw_attribute_group = {
	.attrs = geomagnetic_raw_attributes
};

/* Interface Functions for Lower Layer */

static int
geomagnetic_work(struct yas_mag_data *magdata)
{
	struct geomagnetic_data *data = i2c_get_clientdata(this_client);
	uint32_t time_delay_ms = 100;
	static int cnt;
	int rt, i, accuracy;
#ifdef YAS_SENSOR_KERNEL_DEVFILE_INTERFACE
	struct input_event ev[5];
	struct timeval tv;
#endif

	if (hwdriver.measure == NULL || hwdriver.get_offset == NULL)
		return time_delay_ms;

	rt = hwdriver.measure(magdata, &time_delay_ms);
	if (rt < 0) {
		YLOGE(("measure failed[%d]\n", rt));
		return 100;
	}
	YLOGD(("[%d] xy1y2 [%d][%d][%d] raw[%d][%d][%d]\n",
				rt,
				magdata->xy1y2.v[0], magdata->xy1y2.v[1],
				magdata->xy1y2.v[2], magdata->raw.v[0],
				magdata->raw.v[1], magdata->raw.v[2]));

	accuracy = atomic_read(&data->last_status);

	if ((rt & YAS_REPORT_OVERFLOW_OCCURED)
			|| (rt & YAS_REPORT_HARD_OFFSET_CHANGED)
			|| (rt & YAS_REPORT_CALIB_OFFSET_CHANGED)) {
		static uint16_t count = 1;
		int code = 0;
		int value = 0;

		hwdriver.get_offset(&data->driver_offset);
		if (rt & YAS_REPORT_OVERFLOW_OCCURED) {
			atomic_set(&data->last_status, 0);
			accuracy = 0;
		}

		/* report event */
		code |= (rt & YAS_REPORT_OVERFLOW_OCCURED);
		code |= (rt & YAS_REPORT_HARD_OFFSET_CHANGED);
		code |= (rt & YAS_REPORT_CALIB_OFFSET_CHANGED);
		value = (count++ << 16) | (code);
#ifdef YAS_SENSOR_KERNEL_DEVFILE_INTERFACE
		mkev(ev, EV_ABS, ABS_RAW_REPORT, value);
		sensor_event(&data->raw_devfile_list, ev, 1);
#else
		input_report_abs(data->input_raw, ABS_RAW_REPORT, value);
		data->input_event_updated |= (rt & YAS_REPORT_OVERFLOW_OCCURED) ? GEO_EV_UPDATED_OVERFLOW : 0x00;
		data->input_event_updated |= (rt & YAS_REPORT_HARD_OFFSET_CHANGED) ? GEO_EV_UPDATED_HARD_OFFSET : 0x00;
		data->input_event_updated |= (rt & YAS_REPORT_CALIB_OFFSET_CHANGED) ? GEO_EV_UPDATED_CALIB_OFFSET : 0x00;
		if (data->input_event_log)
		{
			printk("[YAS_MAG_EVENT] ABS_RAW_REPORT value = %d \n",value);
		}
#endif
	}

	if (rt & YAS_REPORT_DATA) {
#ifdef YAS_SENSOR_KERNEL_DEVFILE_INTERFACE
		get_time_stamp(&tv);
		mkev_with_time(&ev[0], EV_ABS, ABS_X, magdata->xyz.v[0], &tv);
		mkev_with_time(&ev[1], EV_ABS, ABS_Y, magdata->xyz.v[1], &tv);
		mkev_with_time(&ev[2], EV_ABS, ABS_Z, magdata->xyz.v[2], &tv);
		mkev_with_time(&ev[3], EV_ABS, ABS_STATUS, accuracy, &tv);
		mkev_with_time(&ev[4], EV_SYN, 0, 0, &tv);
		sensor_event(&data->devfile_list, ev, 5);
#else
		/* report magnetic data in [nT] */
		input_report_abs(data->input_data, ABS_X, magdata->xyz.v[0]);
		input_report_abs(data->input_data, ABS_Y, magdata->xyz.v[1]);
		input_report_abs(data->input_data, ABS_Z, magdata->xyz.v[2]);
		if (atomic_read(&data->last_data[0]) == magdata->xyz.v[0]
				&& atomic_read(&data->last_data[1])
				== magdata->xyz.v[1]
				&& atomic_read(&data->last_data[2])
				== magdata->xyz.v[2]) {
			input_report_abs(data->input_data, ABS_RUDDER, cnt++);
		}
		input_report_abs(data->input_data, ABS_STATUS, accuracy);
		input_sync(data->input_data);
		YLOGD(("input_report_abs accuracy[%d]\n", accuracy));
		data->input_event_updated |= GEO_EV_UPDATED_ACCURACY;
		if (data->input_event_log)
		{
			printk("[YAS_MAG_EVENT] ABS_STATUS value = %d \n",accuracy);
		}
#endif

		for (i = 0; i < 3; i++)
			atomic_set(&data->last_data[i], magdata->xyz.v[i]);
	}

	if (rt & YAS_REPORT_CALIB) {
#ifdef YAS_SENSOR_KERNEL_DEVFILE_INTERFACE
		get_time_stamp(&tv);
		mkev_with_time(&ev[0], EV_ABS, ABS_X, magdata->raw.v[0], &tv);
		mkev_with_time(&ev[1], EV_ABS, ABS_Y, magdata->raw.v[1], &tv);
		mkev_with_time(&ev[2], EV_ABS, ABS_Z, magdata->raw.v[2], &tv);
		mkev_with_time(&ev[3], EV_SYN, 0, 0, &tv);
		sensor_event(&data->raw_devfile_list, ev, 4);
#else
		/* report raw magnetic data */
		input_report_abs(data->input_raw, ABS_X, magdata->raw.v[0]);
		input_report_abs(data->input_raw, ABS_Y, magdata->raw.v[1]);
		input_report_abs(data->input_raw, ABS_Z, magdata->raw.v[2]);
		input_sync(data->input_raw);
#endif
		for (i = 0; i < 3; i++)
			atomic_set(&data->last_raw[i], magdata->raw.v[i]);
	}

	return time_delay_ms;

}

static void
geomagnetic_input_work_func(struct work_struct *work)
{
	struct geomagnetic_data *data
		= container_of((struct delayed_work *)work,
			struct geomagnetic_data, work);
	uint32_t delay;
	struct yas_mag_data magdata;

	delay = geomagnetic_work(&magdata);
	if (delay > 0)
		queue_delayed_work(mag_polling_wq, &data->work, msecs_to_jiffies(delay) + 1);
	else
		queue_delayed_work(mag_polling_wq, &data->work, 0);
}

static int
geomagnetic_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct geomagnetic_data *data = i2c_get_clientdata(client);
	YLOGI(("%s(): [IN]\n",__func__));

	if (hwdriver.set_enable == NULL)
		return -1;

	if (geomagnetic_multi_lock() < 0)
		return -1;

	if (atomic_read(&data->enable)) {
		cancel_delayed_work_sync(&data->work);
		hwdriver.set_enable(0);
	}
#if DEBUG
	data->suspend = 1;
#endif

	geomagnetic_multi_unlock();

	YLOGI(("%s(): [OUT]\n",__func__));
	return 0;
}

static int
geomagnetic_resume(struct i2c_client *client)
{
	struct geomagnetic_data *data = i2c_get_clientdata(client);
	YLOGI(("%s(): [IN]\n",__func__));

	if (hwdriver.set_enable == NULL)
		return -1;

	if (geomagnetic_multi_lock() < 0)
		return -1;

	if (atomic_read(&data->enable)) {
		hwdriver.set_enable(1);
		queue_delayed_work(mag_polling_wq,&data->work,0);
	}

#if DEBUG
	data->suspend = 0;
#endif

	geomagnetic_multi_unlock();

	YLOGI(("%s(): [OUT]\n",__func__));
	return 0;
}


static int
geomagnetic_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct geomagnetic_data *data = NULL;
	struct input_dev *input_data = NULL, *input_raw = NULL;
	int rt, sysfs_created = 0, sysfs_raw_created = 0;
	int data_registered = 0, raw_registered = 0, i;
	struct yas_mag_filter filter;
	int32_t data_allocated = 0;
	int32_t raw_allocated = 0;

	YLOGI(("%s(): start\n",__func__));
	i2c_set_clientdata(client, NULL);
	data = kzalloc(sizeof(struct geomagnetic_data), GFP_KERNEL);
	if (data == NULL) {
		rt = -ENOMEM;
		YLOGE(("%s(): err kzalloc\n",__func__));
		goto err;
	}
	data->ellipsoid_mode = YAS_DEFAULT_MAGCALIB_MODE;
	data->threshold = YAS_DEFAULT_MAGCALIB_THRESHOLD;
	data->distortion[0] = YAS_DEFAULT_MAGCALIB_DISTORTION_1;
	data->distortion[1] = YAS_DEFAULT_MAGCALIB_DISTORTION_2;
	data->distortion[2] = YAS_DEFAULT_MAGCALIB_DISTORTION_3;
	data->shape = 0;
	data->mag_noise = YAS_DEFAULT_GYROCALIB_DISTORTION_MAG;
	data->gyro_noise = YAS_DEFAULT_GYROCALIB_DISTORTION_GYRO;
	atomic_set(&data->enable, 0);
	for (i = 0; i < 3; i++)
		atomic_set(&data->last_data[i], 0);
	for (i = 0; i < 3; i++)
		atomic_set(&data->last_raw[i], 0);
	atomic_set(&data->last_status, 0);
	INIT_DELAYED_WORK(&data->work, geomagnetic_input_work_func);
	sema_init(&data->driver_lock, 1);
	sema_init(&data->multi_lock, 1);

	input_data = input_allocate_device();
	if (input_data == NULL) {
		rt = -ENOMEM;
		YLOGE(("geomagnetic_probe: Failed to allocate input_data "
					"device\n"));
		goto err;
	}
	data_allocated = 1;

	input_data->name = GEOMAGNETIC_INPUT_NAME;
	input_data->id.bustype = BUS_I2C;
	set_bit(EV_ABS, input_data->evbit);
	input_set_abs_params(input_data, ABS_X, INT_MIN, INT_MAX, 0, 0);
	input_set_abs_params(input_data, ABS_Y, INT_MIN, INT_MAX, 0, 0);
	input_set_abs_params(input_data, ABS_Z, INT_MIN, INT_MAX, 0, 0);
	input_set_abs_params(input_data, ABS_RUDDER, INT_MIN, INT_MAX, 0, 0);
	input_set_abs_params(input_data, ABS_STATUS, 0, 3, 0, 0);
	input_set_abs_params(input_data, ABS_WAKE, INT_MIN, INT_MAX, 0, 0);
	input_data->dev.parent = &client->dev;

	rt = input_register_device(input_data);
	if (rt) {
		YLOGE(("geomagnetic_probe: Unable to register input_data "
					"device: %s\n", input_data->name));
		goto err;
	}
	data_registered = 1;

	rt = sysfs_create_group(&input_data->dev.kobj,
			&geomagnetic_attribute_group);
	if (rt) {
		YLOGE(("geomagnetic_probe: sysfs_create_group failed[%s]\n",
					input_data->name));
		goto err;
	}
	sysfs_created = 1;

	input_raw = input_allocate_device();
	if (input_raw == NULL) {
		rt = -ENOMEM;
		YLOGE(("geomagnetic_probe: Failed to allocate input_raw "
					"device\n"));
		goto err;
	}
	raw_allocated = 1;

	input_raw->name = GEOMAGNETIC_INPUT_RAW_NAME;
	input_raw->id.bustype = BUS_I2C;
	set_bit(EV_ABS, input_raw->evbit);
	input_set_abs_params(input_raw, ABS_X, INT_MIN, INT_MAX, 0, 0);
	input_set_abs_params(input_raw, ABS_Y, INT_MIN, INT_MAX, 0, 0);
	input_set_abs_params(input_raw, ABS_Z, INT_MIN, INT_MAX, 0, 0);
	input_set_abs_params(input_raw, ABS_RAW_DISTORTION, 0, INT_MAX, 0, 0);
	input_set_abs_params(input_raw, ABS_RAW_THRESHOLD, 0, 2, 0, 0);
	input_set_abs_params(input_raw, ABS_RAW_SHAPE, 0, 1, 0, 0);
	input_set_abs_params(input_raw, ABS_RAW_MODE, 0, 4, 0, 0);
	input_set_abs_params(input_raw, ABS_RAW_GYRO_THRESHOLD, INT_MIN,
			INT_MAX, 0, 0);
	input_set_abs_params(input_raw, ABS_RAW_REPORT, INT_MIN, INT_MAX, 0, 0);
	input_raw->dev.parent = &client->dev;

	rt = input_register_device(input_raw);
	if (rt) {
		YLOGE(("geomagnetic_probe: Unable to register input_raw device:"
					" %s\n", input_raw->name));
		goto err;
	}
	raw_registered = 1;

	rt = sysfs_create_group(&input_raw->dev.kobj,
			&geomagnetic_raw_attribute_group);
	if (rt) {
		YLOGE(("geomagnetic_probe: sysfs_create_group failed[%s]\n",
					input_data->name));
		goto err;
	}
	sysfs_raw_created = 1;

	this_client = client;
	data->input_raw = input_raw;
	data->input_data = input_data;
	input_set_drvdata(input_data, data);
	input_set_drvdata(input_raw, data);
	i2c_set_clientdata(client, data);

	rt = yas_mag_driver_init(&hwdriver);
	if (rt < 0) {
		YLOGE(("yas_mag_driver_init failed[%d]\n", rt));
		goto err;
	}
	if (hwdriver.init != NULL) {
		rt = hwdriver.init();
		if (rt < 0) {
			YLOGE(("hwdriver.init() failed[%d]\n", rt));
			goto err;
		}
	}
	if (hwdriver.set_position != NULL) {
		if (hwdriver.set_position(
					CONFIG_INPUT_YAS_MAGNETOMETER_POSITION)
				< 0) {
			YLOGE(("hwdriver.set_position() failed[%d]\n", rt));
			goto err;
		}
	}
	if (hwdriver.get_offset != NULL) {
		if (hwdriver.get_offset(&data->driver_offset) < 0) {
			YLOGE(("hwdriver get_driver_state failed\n"));
			goto err;
		}
	}
	if (hwdriver.get_delay != NULL)
		data->delay = hwdriver.get_delay();
	if (hwdriver.set_filter_enable != NULL) {
		/* default to enable */
		if (hwdriver.set_filter_enable(1) == 0)
			data->filter_enable = 1;
	}
	if (hwdriver.get_filter != NULL) {
		if (hwdriver.get_filter(&filter) < 0) {
			YLOGE(("hwdriver get_filter failed\n"));
			goto err;
		}
		data->filter_len = filter.len;
		for (i = 0; i < 3; i++)
			data->filter_noise[i] = filter.noise[i];
		data->filter_threshold = filter.threshold;
	}
	if (hwdriver.get_static_matrix != NULL)
		hwdriver.get_static_matrix(&data->static_matrix);
	if (hwdriver.get_dynamic_matrix != NULL)
		hwdriver.get_dynamic_matrix(&data->dynamic_matrix);

#ifdef YAS_SENSOR_KERNEL_DEVFILE_INTERFACE
	INIT_LIST_HEAD(&data->devfile_list);
	INIT_LIST_HEAD(&data->raw_devfile_list);
	if (misc_register(&sensor_devfile) < 0)
	{
		YLOGE(("%s(): err misc_register\n",__func__));
		goto err;
	}
	if (misc_register(&sensor_raw_devfile) < 0) {
		misc_deregister(&sensor_devfile);
		YLOGE(("%s(): err misc_deregister\n",__func__));
		goto err;
	}
#endif

	data->input_event_log = 0;
	data->input_event_updated = 0;
	YLOGI(("%s(): end\n",__func__));
	return 0;

err:
	if (data != NULL) {
		if (input_raw != NULL) {
			if (sysfs_raw_created) {
				sysfs_remove_group(&input_raw->dev.kobj,
					&geomagnetic_raw_attribute_group);
				YLOGE(("%s(): sysfs_remove_group input_raw\n",__func__));
			}
			if (raw_registered) {
				input_unregister_device(input_raw);
				YLOGE(("%s(): input_unregister_device input_raw\n",__func__));
			}
			if (raw_allocated) {
				input_free_device(input_raw);
				YLOGE(("%s(): input_free_device input_raw\n",__func__));
			}
		}
		if (input_data != NULL) {
			if (sysfs_created) {
				sysfs_remove_group(&input_data->dev.kobj,
					&geomagnetic_attribute_group);
				YLOGE(("%s(): sysfs_remove_group input_data\n",__func__));
			}
			if (data_registered) {
				input_unregister_device(input_data);
				YLOGE(("%s(): input_unregister_device input_data\n",__func__));
			}
			if (data_allocated) {
				input_free_device(input_data);
				YLOGE(("%s(): input_free_device input_data\n",__func__));
			}
		}
		kfree(data);
		YLOGE(("%s(): kfree\n",__func__));
	}

	YLOGI(("%s(): end rt=%d\n",__func__,rt));
	return rt;
}

static int
geomagnetic_remove(struct i2c_client *client)
{
	struct geomagnetic_data *data = i2c_get_clientdata(client);

#ifdef YAS_SENSOR_KERNEL_DEVFILE_INTERFACE
	misc_deregister(&sensor_devfile);
	misc_deregister(&sensor_raw_devfile);
#endif
	if (data != NULL) {
		geomagnetic_disable(data);
		if (hwdriver.term != NULL)
			hwdriver.term();

		input_unregister_device(data->input_raw);
		sysfs_remove_group(&data->input_data->dev.kobj,
				&geomagnetic_attribute_group);
		sysfs_remove_group(&data->input_raw->dev.kobj,
				&geomagnetic_raw_attribute_group);
		input_unregister_device(data->input_data);
		kfree(data);
	}

	return 0;
}

/* I2C Device Driver */
static struct i2c_device_id geomagnetic_idtable[] = {
	{GEOMAGNETIC_I2C_DEVICE_NAME, 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, geomagnetic_idtable);

static struct i2c_driver geomagnetic_i2c_driver = {
	.driver = {
		.name		= GEOMAGNETIC_I2C_DEVICE_NAME,
		.owner		= THIS_MODULE,
	},

	.id_table		= geomagnetic_idtable,
	.probe			= geomagnetic_probe,
	.remove			= geomagnetic_remove,
	.suspend		= geomagnetic_suspend,
	.resume			= geomagnetic_resume,
};

static int __init
geomagnetic_init(void)
{
    int32_t rc;
	YLOGI(("%s(): start\n",__func__));

    mag_polling_wq = create_singlethread_workqueue("mag_polling_wq");
    if(!mag_polling_wq)
    {
		pr_err("can't create queue : mag_polling_wq \n");
		rc = -ENOTSUPP;
    }

    rc = i2c_add_driver(&geomagnetic_i2c_driver);
    if (rc != 0) {
        pr_err("can't add i2c driver\n");
		if(mag_polling_wq != NULL){
			flush_workqueue(mag_polling_wq);
			destroy_workqueue(mag_polling_wq);
			mag_polling_wq = NULL;
		}
        rc = -ENOTSUPP;
    }

	YLOGI(("%s(): end\n",__func__));
    return rc;
}

static void __exit
geomagnetic_term(void)
{
    if(mag_polling_wq != NULL){
        flush_workqueue(mag_polling_wq);
        destroy_workqueue(mag_polling_wq);
        mag_polling_wq = NULL;
    }

	i2c_del_driver(&geomagnetic_i2c_driver);
	i2c_unregister_device(this_client);
	this_client = NULL;
}

#ifdef GEOMAGNETIC_PLATFORM_API
static int
geomagnetic_api_enable(int enable)
{
	struct geomagnetic_data *data = i2c_get_clientdata(this_client);
	int rt;

	if (geomagnetic_multi_lock() < 0)
		return -1;
	enable = !!enable;

	rt = hwdriver.set_enable(enable);
	if (rt == 0) {
		atomic_set(&data->enable, enable);
		if (enable)
			rt = hwdriver.set_delay(20);
	}

	geomagnetic_multi_unlock();

	return rt;
}

int
geomagnetic_api_resume(void)
{
	return geomagnetic_api_enable(1);
}
EXPORT_SYMBOL(geomagnetic_api_resume);

int
geomagnetic_api_suspend(void)
{
	return geomagnetic_api_enable(0);
}
EXPORT_SYMBOL(geomagnetic_api_suspend);

int
geomagnetic_api_read(int *xyz, int *raw, int *xy1y2, int *accuracy)
{
	struct geomagnetic_data *data = i2c_get_clientdata(this_client);
	struct yas_mag_data magdata;
	int i;

	geomagnetic_work(&magdata);
	if (xyz != NULL)
		for (i = 0; i < 3; i++)
			xyz[i] = magdata.xyz.v[i];
	if (raw != NULL)
		for (i = 0; i < 3; i++)
			raw[i] = magdata.raw.v[i];
	if (xy1y2 != NULL)
		for (i = 0; i < 3; i++)
			xy1y2[i] = magdata.xy1y2.v[i];
	if (accuracy != NULL)
		*accuracy = atomic_read(&data->last_status);

	return 0;
}
EXPORT_SYMBOL(geomagnetic_api_read);
#endif

module_init(geomagnetic_init);
module_exit(geomagnetic_term);

MODULE_AUTHOR("Yamaha Corporation");
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS529
MODULE_DESCRIPTION("YAS529 Geomagnetic Sensor Driver");
#elif YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS530
MODULE_DESCRIPTION("YAS530 Geomagnetic Sensor Driver");
#elif YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS532
MODULE_DESCRIPTION("YAS532 Geomagnetic Sensor Driver");
#elif YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS533
MODULE_DESCRIPTION("YAS533 Geomagnetic Sensor Driver");
#endif
MODULE_LICENSE("GPL");
MODULE_VERSION("4.17.909");
