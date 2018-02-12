/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 * (C) 2013 KYOCERA Corporation
 */
/* include/asm/mach-msm/htc_pwrsink.h
 *
 * Copyright (C) 2008 HTC Corporation.
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2011 Code Aurora Forum. All rights reserved.
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include "board-8960.h"
#include "timed_output.h"

enum vib_status
{
    VIB_STANDBY,
    VIB_ON,
    VIB_OFF,
};
enum vib_add_time_flag
{
    VIB_ADD_TIME_FLAG_OFF,
    VIB_ADD_TIME_FLAG_ON,
};

//#define DEBUG_VIB_LC8983XX 1

#define VIB_DRV_NAME                    "LC8983XX"
#define VIB_ON_WORK_NUM                 (5)
#define I2C_RETRIES_NUM                 (5)
#define I2C_WRITE_MSG_NUM               (1)
#define I2C_READ_MSG_NUM                (2)
#define VIB_STANDBY_DELAY_TIME          (1000)
#define VIB_TIME_MIN                    (25)
#define VIB_TIME_MAX                    (15000)
#define VIB_WRITE_ALL_LEN               (10)
#define VIB_WRITE_CHG_LEN               (2)

#define VIB_GPIO_RST                    (83)
#define VIB_GPIO_EN                     (84)

#define VIB_INTENSITY_LOW_ON            (0x00010000)
#define VIB_INTENSITY_MASK              (0x0000FFFF)

struct vib_on_work_data
{
    struct work_struct  work_vib_on;
    int                 time;
    int                 intensity;
};
struct lc8983xx_work_data {
    struct vib_on_work_data vib_on_work_data[VIB_ON_WORK_NUM];
    struct work_struct work_vib_off;
    struct work_struct work_vib_standby;
};
struct lc8983xx_data_t {
    struct i2c_client *lc8983xx_i2c_client;
    struct hrtimer vib_off_timer;
    struct hrtimer vib_standby_timer;
    int work_vib_on_pos;
    enum vib_status vib_cur_status;
    enum vib_add_time_flag add_time_flag;
    struct regulator *vib_regulator;
};

static struct mutex vib_mutex;
                                                   /* addr  0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09 */
static u8 write_all_intensity_3[VIB_WRITE_ALL_LEN] = {0x01, 0x06, 0x0B, 0x03, 0x18, 0x18, 0x60, 0x60, 0x20, 0x00};
static u8 write_all_intensity_4[VIB_WRITE_ALL_LEN] = {0x01, 0x0B, 0x0B, 0x03, 0x18, 0x18, 0x60, 0x60, 0x20, 0x00};
static u8 write_chg_intensity_3[VIB_WRITE_CHG_LEN] = {0x01, 0x06};
static u8 write_chg_intensity_4[VIB_WRITE_CHG_LEN] = {0x01, 0x0B};
                                                       /* addr  0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09 */
static u8 write_all_intensity_3_low[VIB_WRITE_ALL_LEN] = {0x01, 0x05, 0x0B, 0x03, 0x18, 0x18, 0x60, 0x60, 0x20, 0x00};
static u8 write_all_intensity_4_low[VIB_WRITE_ALL_LEN] = {0x01, 0x08, 0x0B, 0x03, 0x18, 0x18, 0x60, 0x60, 0x20, 0x00};
static u8 write_chg_intensity_3_low[VIB_WRITE_CHG_LEN] = {0x01, 0x05};
static u8 write_chg_intensity_4_low[VIB_WRITE_CHG_LEN] = {0x01, 0x08};

#ifdef DEBUG_VIB_LC8983XX
static u8 read_buf[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
#endif /* DEBUG_VIB_LC8983XX */
struct lc8983xx_work_data lc8983xx_work;
struct lc8983xx_data_t lc8983xx_data;

#define VIB_LOG(md, fmt, ... ) \
printk(md "[VIB]%s(%d): " fmt, __func__, __LINE__, ## __VA_ARGS__)
#ifdef DEBUG_VIB_LC8983XX
#define VIB_DEBUG_LOG(md, fmt, ... ) \
printk(md "[VIB]%s(%d): " fmt, __func__, __LINE__, ## __VA_ARGS__)
#else
#define VIB_DEBUG_LOG(md, fmt, ... )
#endif /* DEBUG_VIB_LC8983XX */

#define VIB 'V'
#define VIB_SET_INTENSITY       _IOW(VIB, 0, T_VIB_INTENSITY_IOCTL)
typedef struct _t_vib_ioctl {
    uint32_t data;
}T_VIB_INTENSITY_IOCTL;

enum vib_intensity{
    VIB_INTENSITY_1 = 1,
    VIB_INTENSITY_2,
    VIB_INTENSITY_3,
    VIB_INTENSITY_4,
    VIB_INTENSITY_5
};

static atomic_t g_vib_intensity = ATOMIC_INIT(VIB_INTENSITY_3);

static int lc8983xx_i2c_write_data(struct i2c_client *client, u8 *buf, u16 len)
{
    int ret = 0;
    int retry = 0;
    struct i2c_msg msg[I2C_WRITE_MSG_NUM];
#ifdef DEBUG_VIB_LC8983XX
    int i = 0;
#endif /* DEBUG_VIB_LC8983XX */

    VIB_DEBUG_LOG(KERN_INFO, "called. len=%d\n", (int)len);
    if (client == NULL || buf == NULL)
    {
        VIB_LOG(KERN_ERR, "client=0x%08x,buf=0x%08x\n",
                (unsigned int)client, (unsigned int)buf);
        return 0;
    }

    VIB_DEBUG_LOG(KERN_INFO, "addr=0x%02x,adapter=0x%08x\n",
                  (unsigned int)client->addr,
                  (unsigned int)client->adapter);

    msg[0].addr = client->addr;
    msg[0].flags = 0;
    msg[0].len = len;
    msg[0].buf = buf;

    do
    {
        ret = i2c_transfer(client->adapter, msg, I2C_WRITE_MSG_NUM);
        VIB_DEBUG_LOG(KERN_INFO, "i2c_transfer(write) ret=%d\n", ret);
    } while ((ret != I2C_WRITE_MSG_NUM) && (++retry < I2C_RETRIES_NUM));

    if (ret != I2C_WRITE_MSG_NUM)
    {
        ret = -1;
        VIB_LOG(KERN_ERR, "i2c write error (try:%d)\n", retry);
    }
    else
    {
        ret = 0;
        VIB_DEBUG_LOG(KERN_INFO, "i2c write success\n");
#ifdef DEBUG_VIB_LC8983XX
        for (i = 1; i < len; i++)
        {
            VIB_DEBUG_LOG(KERN_INFO, "i2c write reg=0x%02x,value=0x%02x\n",
                          (unsigned int)(*buf + i), (unsigned int)*(buf + i));
        }
#endif /* DEBUG_VIB_LC8983XX */
    }

    VIB_DEBUG_LOG(KERN_INFO, "finish. ret=%d\n", ret);
    return ret;
}

#ifdef DEBUG_VIB_LC8983XX
static int lc8983xx_i2c_read_data(struct i2c_client *client, u8 reg, u8 *buf, u16 len)
{
    int ret = 0;
    int retry = 0;
    u8 start_reg = 0;
    struct i2c_msg msg[I2C_READ_MSG_NUM];
    int i = 0;

    VIB_DEBUG_LOG(KERN_INFO, "called. reg=0x%02x, len=%d\n", (int)reg, (int)len);
    if (client == NULL || buf == NULL)
    {
        VIB_LOG(KERN_ERR, "client=0x%08x\n",
                (unsigned int)client);
        return 0;
    }

    VIB_DEBUG_LOG(KERN_INFO, "addr=0x%02x,adapter=0x%08x\n",
                  (unsigned int)client->addr,
                  (unsigned int)client->adapter);

    msg[0].addr = client->addr;
    msg[0].flags = 0;
    msg[0].len = 1;
    msg[0].buf = &start_reg;
    start_reg = reg;

    msg[1].addr = client->addr;
    msg[1].flags = I2C_M_RD;
    msg[1].len = len;
    msg[1].buf = buf;

    do
    {
        ret = i2c_transfer(client->adapter, msg, I2C_READ_MSG_NUM);
        VIB_DEBUG_LOG(KERN_INFO, "i2c_transfer(read) reg=0x%02x,ret=%d\n",
                      (unsigned int)reg, ret);
    } while ((ret != I2C_READ_MSG_NUM) && (++retry < I2C_RETRIES_NUM));

    if(ret != I2C_READ_MSG_NUM)
    {
        ret = -1;
        VIB_LOG(KERN_ERR, "i2c read error (try:%d)\n", retry);
    }
    else
    {
        ret = 0;
        VIB_DEBUG_LOG(KERN_INFO, "i2c read success\n");
        for (i = 0; i < len; i++)
        {
            VIB_DEBUG_LOG(KERN_INFO, "i2c read reg=0x%02x,value=0x%02x\n",
                          (unsigned int)(reg + i), (unsigned int)*(buf + i));
        }
    }

    VIB_DEBUG_LOG(KERN_INFO, "finish. ret=%d\n", ret);
    return ret;
}
#endif /* DEBUG_VIB_LC8983XX */

static void lc8983xx_set_vib(enum vib_status status, int time, int intensity_param)
{
    enum vib_status cur_status = VIB_STANDBY;
    int ret = 0;
    u8 *write_buf;
    int intensity = (intensity_param & VIB_INTENSITY_MASK);

    mutex_lock(&vib_mutex);

    cur_status = lc8983xx_data.vib_cur_status;
    VIB_DEBUG_LOG(KERN_INFO, "called. status=%d,time=%d,intensity_param=%d,cur_status=%d\n",
                              status, time, intensity_param, cur_status);

    switch (status) {
        case VIB_ON:
            VIB_DEBUG_LOG(KERN_INFO, "VIB_ON\n");
            if (cur_status == VIB_STANDBY)
            {
                ret = gpio_request(VIB_GPIO_RST, "LC8983XX RST");
                VIB_DEBUG_LOG(KERN_INFO, "gpio_request(GPIO83) ret=%d\n", ret);
                if (ret != 0)
                {
                    VIB_LOG(KERN_ERR, "gpio_request(GPIO83) ret=%d\n", ret);
                    goto gpio_request_error;
                }
                gpio_set_value_cansleep(VIB_GPIO_RST, 1);

                VIB_DEBUG_LOG(KERN_INFO, "udelay(200) start.\n");
                udelay(200);
                VIB_DEBUG_LOG(KERN_INFO, "udelay(200) end.\n");

                if(intensity_param & VIB_INTENSITY_LOW_ON)
                {
                    if(intensity == VIB_INTENSITY_4)
                    {
                        write_buf = write_all_intensity_4_low;
                    }
                    else
                    {
                        write_buf = write_all_intensity_3_low;
                    }
                }
                else
                {
                    if(intensity == VIB_INTENSITY_4)
                    {
                        write_buf = write_all_intensity_4;
                    }
                    else
                    {
                        write_buf = write_all_intensity_3;
                    }
                }
                lc8983xx_i2c_write_data(lc8983xx_data.lc8983xx_i2c_client,
                                        (u8 *)write_buf,
                                        VIB_WRITE_ALL_LEN);

#ifdef DEBUG_VIB_LC8983XX
                lc8983xx_i2c_read_data(lc8983xx_data.lc8983xx_i2c_client,
                                        0x01,
                                        (u8 *)read_buf,
                                        sizeof(read_buf));
#endif /* DEBUG_VIB_LC8983XX */
            }
            else
            {
                if(intensity_param & VIB_INTENSITY_LOW_ON)
                {
                    if(intensity == VIB_INTENSITY_4)
                    {
                        write_buf = write_chg_intensity_4_low;
                    }
                    else
                    {
                        write_buf = write_chg_intensity_3_low;
                    }
                }
                else
                {
                    if(intensity == VIB_INTENSITY_4)
                    {
                        write_buf = write_chg_intensity_4;
                    }
                    else
                    {
                        write_buf = write_chg_intensity_3;
                    }
                }
                lc8983xx_i2c_write_data(lc8983xx_data.lc8983xx_i2c_client,
                                        (u8 *)write_buf,
                                        VIB_WRITE_CHG_LEN);

#ifdef DEBUG_VIB_LC8983XX
                lc8983xx_i2c_read_data(lc8983xx_data.lc8983xx_i2c_client,
                                        0x01,
                                        (u8 *)read_buf,
                                        VIB_WRITE_CHG_LEN-1);
#endif /* DEBUG_VIB_LC8983XX */
                VIB_DEBUG_LOG(KERN_INFO, "VIB_ON standby cancel skip.\n");
            }

            if (cur_status != VIB_ON)
            {
                ret = gpio_request(VIB_GPIO_EN, "LC8983XX EN");
                VIB_DEBUG_LOG(KERN_INFO, "gpio_request(GPIO84) ret=%d\n", ret);
                if (ret != 0)
                {
                    VIB_LOG(KERN_ERR, "gpio_request(GPIO84) ret=%d\n", ret);
                    goto gpio_request_error;
                }
                gpio_set_value_cansleep(VIB_GPIO_EN, 1);
            }
            else
            {
                VIB_DEBUG_LOG(KERN_INFO, "VIB_ON skip.\n");
            }
            VIB_DEBUG_LOG(KERN_INFO, "hrtimer_start(vib_off_timer). time=%d\n", time);
            hrtimer_start(&lc8983xx_data.vib_off_timer,
                          ktime_set(time / 1000, 
                          (time % 1000) * 1000000),
                          HRTIMER_MODE_REL);
            
            lc8983xx_data.vib_cur_status = status;
            VIB_DEBUG_LOG(KERN_INFO, "set cur_status=%d\n", lc8983xx_data.vib_cur_status);
            break;
        case VIB_OFF:
            VIB_DEBUG_LOG(KERN_INFO, "VIB_OFF\n");
            if (cur_status == VIB_ON)
            {
                gpio_set_value_cansleep(VIB_GPIO_EN, 0);
                gpio_free(VIB_GPIO_EN);
                VIB_DEBUG_LOG(KERN_INFO, "gpio_free(GPIO84)\n");

                lc8983xx_data.vib_cur_status = status;
                VIB_DEBUG_LOG(KERN_INFO, "set cur_status=%d\n", lc8983xx_data.vib_cur_status);

                VIB_DEBUG_LOG(KERN_INFO, "hrtimer_start(vib_standby_timer).\n");
                hrtimer_start(&lc8983xx_data.vib_standby_timer,
                              ktime_set(VIB_STANDBY_DELAY_TIME / 1000, 
                              (VIB_STANDBY_DELAY_TIME % 1000) * 1000000),
                              HRTIMER_MODE_REL);
            }
            else
            {
                VIB_DEBUG_LOG(KERN_INFO, "VIB_OFF skip.\n");
            }
            break;
        case VIB_STANDBY:
            VIB_DEBUG_LOG(KERN_INFO, "VIB_STANDBY\n");
            if (cur_status == VIB_OFF)
            {
                gpio_set_value_cansleep(VIB_GPIO_RST, 0);
                gpio_free(VIB_GPIO_RST);
                VIB_DEBUG_LOG(KERN_INFO, "gpio_free(GPIO83)\n");

                lc8983xx_data.vib_cur_status = status;
                VIB_DEBUG_LOG(KERN_INFO, "set cur_status=%d\n", lc8983xx_data.vib_cur_status);
            }
            else
            {
                VIB_DEBUG_LOG(KERN_INFO, "VIB_STANDBY skip.\n");
            }
            break;
        default:
            VIB_LOG(KERN_ERR, "parameter error. status=%d\n", status);
            break;
    }
gpio_request_error:
    mutex_unlock(&vib_mutex);
    VIB_DEBUG_LOG(KERN_INFO, "end.\n");
    return;
}

static void lc8983xx_vib_on(struct work_struct *work)
{
    struct vib_on_work_data *work_data = container_of
                                  (work, struct vib_on_work_data, work_vib_on);

    VIB_DEBUG_LOG(KERN_INFO, "called. work=0x%08x\n", (unsigned int)work);
    VIB_DEBUG_LOG(KERN_INFO, "work_data=0x%08x,time=%d\n",
                  (unsigned int)work_data, work_data->time);
    lc8983xx_set_vib(VIB_ON, work_data->time, work_data->intensity);

    return;
}

static void lc8983xx_vib_off(struct work_struct *work)
{
    VIB_DEBUG_LOG(KERN_INFO, "called. work=0x%08x\n", (unsigned int)work);
    lc8983xx_set_vib(VIB_OFF, 0, 0);
    return;
}

static void lc8983xx_vib_standby(struct work_struct *work)
{
    VIB_DEBUG_LOG(KERN_INFO, "called. work=0x%08x\n", (unsigned int)work);

    lc8983xx_set_vib(VIB_STANDBY, 0, 0);
    return;
}

static void lc8983xx_timed_vib_on(struct timed_output_dev *dev, int timeout_val)
{
    int ret = 0;

    VIB_DEBUG_LOG(KERN_INFO, "called. dev=0x%08x, timeout_val=%d\n",
                  (unsigned int)dev, timeout_val);

    lc8983xx_work.vib_on_work_data[lc8983xx_data.work_vib_on_pos].time = timeout_val;
    lc8983xx_work.vib_on_work_data[lc8983xx_data.work_vib_on_pos].intensity =
                                            (int)atomic_read(&g_vib_intensity);

    VIB_DEBUG_LOG(KERN_INFO, "called. g_vib_intensity=0x%08x\n",
                    lc8983xx_work.vib_on_work_data[lc8983xx_data.work_vib_on_pos].intensity);

    ret = schedule_work
          (&(lc8983xx_work.vib_on_work_data[lc8983xx_data.work_vib_on_pos].work_vib_on));
    if (ret != 0)
    {
        lc8983xx_data.work_vib_on_pos++;
        if (lc8983xx_data.work_vib_on_pos >= VIB_ON_WORK_NUM) {
            lc8983xx_data.work_vib_on_pos = 0;
        }
        VIB_DEBUG_LOG(KERN_INFO, "schedule_work(). work_vib_on_pos=%d\n",
                      lc8983xx_data.work_vib_on_pos);
        VIB_DEBUG_LOG(KERN_INFO, "vib_on_work_data[%d].time=%d intensity=0x%08x\n",
                      lc8983xx_data.work_vib_on_pos,
                      lc8983xx_work.vib_on_work_data[lc8983xx_data.work_vib_on_pos].time,
                      lc8983xx_work.vib_on_work_data[lc8983xx_data.work_vib_on_pos].intensity);
    }
    return;
}

static void lc8983xx_timed_vib_off(struct timed_output_dev *dev)
{
    int ret = 0;

    VIB_DEBUG_LOG(KERN_INFO, "called. dev=0x%08x\n", (unsigned int)dev);
    ret = schedule_work(&lc8983xx_work.work_vib_off);
    if  (ret == 0)
    {
        VIB_LOG(KERN_ERR, "schedule_work error. ret=%d\n",ret);
    }
    return;
}

static void lc8983xx_timed_vib_standby(struct timed_output_dev *dev)
{
    int ret = 0;

    VIB_DEBUG_LOG(KERN_INFO, "called. dev=0x%08x\n", (unsigned int)dev);
    ret = schedule_work(&lc8983xx_work.work_vib_standby);
    if  (ret == 0)
    {
        VIB_LOG(KERN_ERR, "schedule_work error. ret=%d\n",ret);
    }
    return;
}

static void lc8983xx_enable(struct timed_output_dev *dev, int value)
{
    VIB_DEBUG_LOG(KERN_INFO, "called. dev=0x%08x,value=%d\n", (unsigned int)dev, value);
    VIB_DEBUG_LOG(KERN_INFO, "add_time_flag=%d\n", lc8983xx_data.add_time_flag);
    if ((value <= 0) && (lc8983xx_data.add_time_flag == VIB_ADD_TIME_FLAG_ON))
    {
        VIB_DEBUG_LOG(KERN_INFO, "skip. value=%d,add_time_flag=%d\n",
                      value, lc8983xx_data.add_time_flag);
        return;
    }

    VIB_DEBUG_LOG(KERN_INFO, "hrtimer_cancel(vib_off_timer)\n");
    hrtimer_cancel(&lc8983xx_data.vib_off_timer);

    if (value <= 0)
    {
        lc8983xx_timed_vib_off(dev);
    }
    else
    {
        VIB_DEBUG_LOG(KERN_INFO, "hrtimer_cancel(vib_standby_timer)\n");
        hrtimer_cancel(&lc8983xx_data.vib_standby_timer);
        if (value < VIB_TIME_MIN)
        {
            value = VIB_TIME_MIN;
            lc8983xx_data.add_time_flag = VIB_ADD_TIME_FLAG_ON;
            VIB_DEBUG_LOG(KERN_INFO, "set add_time_flag=%d\n", lc8983xx_data.add_time_flag);
        }
        else
        {
            lc8983xx_data.add_time_flag = VIB_ADD_TIME_FLAG_OFF;
            VIB_DEBUG_LOG(KERN_INFO, "set add_time_flag=%d\n", lc8983xx_data.add_time_flag);
        }
        lc8983xx_timed_vib_on(dev, value);
    }
    return;
}

static int lc8983xx_get_vib_time(struct timed_output_dev *dev)
{
    int ret = 0;

    VIB_DEBUG_LOG(KERN_INFO, "called. dev=0x%08x\n", (unsigned int)dev);
    mutex_lock(&vib_mutex);

    ret = hrtimer_active(&lc8983xx_data.vib_off_timer);
    if (ret != 0)
    {
        ktime_t r = hrtimer_get_remaining(&lc8983xx_data.vib_off_timer);
        struct timeval t = ktime_to_timeval(r);
        mutex_unlock(&vib_mutex);

        return t.tv_sec * 1000 + t.tv_usec / 1000;
    }
    mutex_unlock(&vib_mutex);
    return 0;
}

static enum hrtimer_restart lc8983xx_off_timer_func(struct hrtimer *timer)
{
    VIB_DEBUG_LOG(KERN_INFO, "called. timer=0x%08x\n", (unsigned int)timer);
    lc8983xx_data.add_time_flag = VIB_ADD_TIME_FLAG_OFF;
    VIB_DEBUG_LOG(KERN_INFO, "set add_time_flag=%d\n", lc8983xx_data.add_time_flag);

    lc8983xx_timed_vib_off(NULL);
    return HRTIMER_NORESTART;
}

static enum hrtimer_restart lc8983xx_standby_timer_func(struct hrtimer *timer)
{
    VIB_DEBUG_LOG(KERN_INFO, "called. timer=0x%08x\n", (unsigned int)timer);
    lc8983xx_timed_vib_standby(NULL);
    return HRTIMER_NORESTART;
}

static struct timed_output_dev lc8983xx_output_dev = {
    .name = "vibrator",
    .get_time = lc8983xx_get_vib_time,
    .enable = lc8983xx_enable,
};

static int32_t vib_open(struct inode* inode, struct file* filp)
{
    VIB_DEBUG_LOG(KERN_INFO, "called.\n");
    return 0;
}

static int32_t vib_release(struct inode* inode, struct file* filp)
{
    VIB_DEBUG_LOG(KERN_INFO, "called.\n");
    return 0;
}

static long vib_ioctl (struct file *filp, unsigned int cmd, unsigned long arg)
{
    void __user *argp = (void __user *)arg;
    int32_t ret = -1;
    T_VIB_INTENSITY_IOCTL st_ioctl;

    VIB_DEBUG_LOG(KERN_INFO, "called.\n");
    switch (cmd) {
    case VIB_SET_INTENSITY:
        VIB_DEBUG_LOG(KERN_INFO, "VIB_SET_INTENSITY.\n");
        ret = copy_from_user(&st_ioctl,
                    argp,
                    sizeof(T_VIB_INTENSITY_IOCTL));
        if (ret) {
            VIB_DEBUG_LOG(KERN_INFO, "ERROR.\n");
            return -EFAULT;
        }
        atomic_set(&g_vib_intensity, (int)st_ioctl.data);
        VIB_DEBUG_LOG(KERN_INFO, "g_vib_intensity = %d.\n", (int)st_ioctl.data);
        break;
    default:
            VIB_DEBUG_LOG(KERN_INFO, "default.\n");
        return -ENOTTY;
    }
    return 0;
}

static struct file_operations vib_fops = {
    .owner      = THIS_MODULE,
    .open       = vib_open,
    .release    = vib_release,
    .unlocked_ioctl = vib_ioctl,
};

static struct miscdevice vib_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name  = "vib-intensity",
    .fops  = &vib_fops,
};

static int __devinit lc8983xx_probe(struct i2c_client *client, const struct i2c_device_id *id)
{

    int ret = 0;
    int count = 0;
    bool bret = true;
    
    VIB_DEBUG_LOG(KERN_INFO, "called. id=0x%08x\n", (unsigned int)id);

    lc8983xx_data.lc8983xx_i2c_client = client;
    lc8983xx_data.vib_cur_status = VIB_STANDBY;
    lc8983xx_data.add_time_flag = VIB_ADD_TIME_FLAG_OFF;

    mutex_init(&vib_mutex);

    for (count = 0; count < VIB_ON_WORK_NUM; count++)
    {
        INIT_WORK(&(lc8983xx_work.vib_on_work_data[count].work_vib_on),
                  lc8983xx_vib_on);
        lc8983xx_work.vib_on_work_data[count].time = 0;
        lc8983xx_work.vib_on_work_data[count].intensity = VIB_INTENSITY_3;
    }
    lc8983xx_data.work_vib_on_pos = 0;
    INIT_WORK(&lc8983xx_work.work_vib_off, lc8983xx_vib_off);
    INIT_WORK(&lc8983xx_work.work_vib_standby, lc8983xx_vib_standby);

    hrtimer_init(&lc8983xx_data.vib_off_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    lc8983xx_data.vib_off_timer.function = lc8983xx_off_timer_func;
    hrtimer_init(&lc8983xx_data.vib_standby_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    lc8983xx_data.vib_standby_timer.function = lc8983xx_standby_timer_func;

    ret = timed_output_dev_register(&lc8983xx_output_dev);
    VIB_DEBUG_LOG(KERN_INFO, "timed_output_dev_register() ret=%d\n", ret);
    if (ret != 0)
    {
        VIB_LOG(KERN_ERR, "timed_output_dev_register() ret=%d\n", ret);
        goto probe_dev_register_error;
    }

    bret = gpio_is_valid(VIB_GPIO_RST);
    if (!bret)
    {
        VIB_LOG(KERN_ERR, "gpio_is_valid error (LC8983XX RST)\n");
        ret = -ENODEV;
        goto probe_gpio_is_valid_error;
    }

    bret = gpio_is_valid(VIB_GPIO_EN);
    if (!bret)
    {
        VIB_LOG(KERN_ERR, "gpio_is_valid error (LC8983XX EN)\n");
        ret = -ENODEV;
        goto probe_gpio_is_valid_error;
    }
    misc_register(&vib_device);
    return 0;

probe_gpio_is_valid_error:
    timed_output_dev_unregister(&lc8983xx_output_dev);
probe_dev_register_error:
    mutex_destroy(&vib_mutex);
    return ret;

}

static int32_t __devexit lc8983xx_remove(struct i2c_client *pst_client)
{
    int ret = 0;

    VIB_DEBUG_LOG(KERN_INFO, "called. pst_client=0x%08x\n", (unsigned int)pst_client);
    gpio_free(VIB_GPIO_RST);
    gpio_free(VIB_GPIO_EN);

    timed_output_dev_unregister(&lc8983xx_output_dev);

    mutex_destroy(&vib_mutex);
    return ret;
}

static int32_t lc8983xx_suspend(struct i2c_client *pst_client, pm_message_t mesg)
{
    VIB_DEBUG_LOG(KERN_INFO, "called. pst_client=0x%08x,mesg=%d\n",
                  (unsigned int)pst_client, mesg.event);
    VIB_DEBUG_LOG(KERN_INFO, "end.\n");
    return 0;
}

static int32_t lc8983xx_resume(struct i2c_client *pst_client)
{
    VIB_DEBUG_LOG(KERN_INFO, "called. pst_client=0x%08x\n", (unsigned int)pst_client);
    VIB_DEBUG_LOG(KERN_INFO, "end.\n");
    return 0;
}

#ifdef CONFIG_OF
static struct of_device_id lc8983xx_match_table[] = {
    { .compatible = "kc,vibrator" },
    { },
};
#else
#define lc8983xx_match_table NULL
#endif

static const struct i2c_device_id lc8983xx_idtable[] = {
    { "LC8983XX", 0 },
    { },
};
MODULE_DEVICE_TABLE(i2c, lc8983xx_idtable);

static struct i2c_driver lc8983xx_driver = {
    .driver     = {
        .name   = VIB_DRV_NAME,
        .owner = THIS_MODULE,
        .of_match_table = lc8983xx_match_table,
    },
    .probe      = lc8983xx_probe,
    .remove     = __devexit_p(lc8983xx_remove),
    .suspend    = lc8983xx_suspend,
    .resume     = lc8983xx_resume,
    .id_table   = lc8983xx_idtable,
};

static int __init lc8983xx_init(void)
{

    int ret = 0;

    VIB_DEBUG_LOG(KERN_INFO, "called.\n");

    ret = i2c_add_driver(&lc8983xx_driver);
    VIB_DEBUG_LOG(KERN_INFO, "i2c_add_driver() ret=%d\n", ret);
    if (ret != 0)
    {
        VIB_LOG(KERN_ERR, "i2c_add_driver() ret=%d\n", ret);
    }

    return ret;
}

static void __exit lc8983xx_exit(void)
{
    VIB_DEBUG_LOG(KERN_INFO, "called.\n");
    i2c_del_driver(&lc8983xx_driver);
    VIB_DEBUG_LOG(KERN_INFO, "i2c_del_driver()\n");

    return;
}

module_init(lc8983xx_init);
module_exit(lc8983xx_exit);
MODULE_DESCRIPTION("timed output LC8983XX vibrator device");
MODULE_LICENSE("GPL");
