/* This program is free software; you can redistribute it and/or
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
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2013 KYOCERA Corporation
 */
#include <mach/gpiomux.h>
#include "msm_sensor.h"
#include "msm_cci.h"
#include "msm_camera_io_util.h"
#include "camera.h"
#include "msm.h"
#define S5K8AAY_SENSOR_NAME "s5k8aay"
#define PLATFORM_DRIVER_NAME "msm_camera_s5k8aay"
#define s5k8aay_obj s5k8aay_##obj

#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

#define S5K8AAY_EXPOSURE_SETTING_SIZE 4
#define S5K8AAY_EXPOSURE_VALUE_STEP 1
#define S5K8AAY_EXPOSURE_VALUE_MAX 6
#define S5K8AAY_EXPOSURE_TBL_SIZE (((S5K8AAY_EXPOSURE_VALUE_MAX) * 2) + 1)

#define S5K8AAY_WB_AUTO             0
#define S5K8AAY_WB_DAYLIGHT         1
#define S5K8AAY_WB_FLUORESCENT      2
#define S5K8AAY_WB_WARM_FLUORESCENT 3
#define S5K8AAY_WB_INCANDESCENT     4
#define S5K8AAY_WB_MAX              5
#define S5K8AAY_WB_SETTING_SIZE 11

#define S5K8AAY_REG_READ_ADDR 0x0F12

#define S5K8AAY_EXP_TIME_DIVISOR 400

#define S5K8AAY_ERR_CHK_TBL_SIZE 2
#define S5K8AAY_ERR_CHK_TBL_REG_SIZE 3
#define S5K8AAY_ERR_CHK_VAL 0x0000

#define S5K8AAY_RESOLUTION_TBL_SIZE 13
#define S5K8AAY_RESOLUTION_TBL_REG_SIZE 55
#define S5K8AAY_OUTPUT_RESOLUTION_SXGA 0

#define S5K8AAY_EFFECT_OFF          0
#define S5K8AAY_EFFECT_MONO         1
#define S5K8AAY_EFFECT_SEPIA        2
#define S5K8AAY_EFFECT_NEGATIVE     3
#define S5K8AAY_EFFECT_MAX          4
#define S5K8AAY_EFFECT_SETTING_SIZE 8

#define S5K8AAY_EXP_THRESHOLD_5LX    0x2EDF
#define S5K8AAY_AGAIN_THRESHOLD_5LX  0x0A76
#define S5K8AAY_EXP_THRESHOLD_50LX   0x2EDF
#define S5K8AAY_AGAIN_THRESHOLD_50LX 0x0A76

DEFINE_MSM_MUTEX(s5k8aay_mut);
static struct msm_sensor_ctrl_t s5k8aay_s_ctrl;

static int8_t s5k8aay_wb = CAMERA_WB_MODE_AUTO;
static int8_t s5k8aay_effect = CAMERA_EFFECT_MODE_OFF;

typedef struct {
	struct work_struct work;
	struct msm_sensor_ctrl_t *s_ctrl;
} s5k8aay_work_t;

static struct workqueue_struct *s5k8aay_err_chk_wq;
static s5k8aay_work_t *s5k8aay_err_chk_work;
static volatile int s5k8aay_power_on_flg = 0;
static volatile int s5k8aay_err_chk_work_flg = 0;
static int32_t pre_exp_setting = 0;
static uint8_t s5k8aay_skip_frame_flg = 0;
static uint32_t s5k8aay_g_exposure_time = 0;
static int32_t s5k8aay_power_on_first = 0;

/* Prototype Declaration */
//static int msm_sensor_s5k8aay_sensor_config(struct msm_sensor_ctrl_t *s_ctrl,
 int32_t msm_sensor_s5k8aay_sensor_config(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp);
static int msm_sensor_s5k8aay_power_up(struct msm_sensor_ctrl_t *s_ctrl);
static int msm_sensor_s5k8aay_power_down(struct msm_sensor_ctrl_t *s_ctrl);
static int32_t msm_sensor_s5k8aay_match_id(struct msm_sensor_ctrl_t *s_ctrl);
static int msm_sensor_s5k8aay_start_stream(struct msm_sensor_ctrl_t *s_ctrl);
static int32_t msm_sensor_s5k8aay_stop_stream(struct msm_sensor_ctrl_t *s_ctrl);
static int32_t msm_sensor_s5k8aay_write_exp(struct msm_sensor_ctrl_t *s_ctrl, int32_t exp_value);
static int32_t msm_sensor_s5k8aay_set_white_balance(struct msm_sensor_ctrl_t *s_ctrl, int32_t wb);
static int32_t msm_sensor_s5k8aay_get_exp_time(struct msm_sensor_ctrl_t *s_ctrl, uint32_t *exposure_time);
static void msm_sensor_s5k8aay_error_check_work(struct work_struct *work);
static void msm_sensor_s5k8aay_set_skip_frame(void);
#if 0
static void msm_sensor_s5k8aay_get_frame_skip_flg(struct msm_sensor_ctrl_t *s_ctrl, uint8_t *flag);
#endif /* #if 0 */
static int32_t msm_sensor_s5k8aay_set_effect(struct msm_sensor_ctrl_t *s_ctrl, int32_t effect);
static int32_t msm_sensor_s5k8aay_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id);
static int32_t msm_sensor_s5k8aay_platform_probe(struct platform_device *pdev);
static void __exit msm_sensor_s5k8aay_exit_module(void);
static int __init msm_sensor_s5k8aay_init_module(void);
static void msm_sensor_s5k8aay_set_skip_frame(void);

static struct msm_sensor_power_setting s5k8aay_power_setting[] = {
	{
#if 0
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VAMA,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
#else  /* #if 0 */
		.seq_type = SENSOR_VREG,
//		.seq_val = CAM_VANA,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
#endif /* #if 0 */
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VDD,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{
#if 0
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VIO,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
#else  /* #if 0 */
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 0,
#endif /* #if 0 */
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 0,
		.delay = 50,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 50,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
};
static struct msm_camera_i2c_reg_conf s5k8aay_recommend_settings1[] = {
	{0xFCFC, 0xD000,},/* reserved */
	{0x0010, 0x0001,},/* S/W Reset */
	{0xFCFC, 0x0000,},/* reserved */
	{0x0000, 0x0000,},/* reserved */
	{0xFCFC, 0xD000,},/* reserved */
	{0x1030, 0x0000,},/* contint_host_int */
	{0x0014, 0x0001,},/* sw_load_complete */
};
static struct msm_camera_i2c_reg_conf s5k8aay_recommend_settings2[] = {
	{0x0028, 0x7000,},/* reserved */
	{0x002A, 0x2470,},/* reserved */
	{0x0F12, 0xB510,},/* reserved */
	{0x0F12, 0x490E,},/* reserved */
	{0x0F12, 0x480E,},/* reserved */
	{0x0F12, 0xF000,},/* reserved */
	{0x0F12, 0xF9ED,},/* reserved */
	{0x0F12, 0x490E,},/* reserved */
	{0x0F12, 0x480E,},/* reserved */
	{0x0F12, 0xF000,},/* reserved */
	{0x0F12, 0xF9E9,},/* reserved */
	{0x0F12, 0x490E,},/* reserved */
	{0x0F12, 0x480E,},/* reserved */
	{0x0F12, 0x6341,},/* reserved */
	{0x0F12, 0x490E,},/* reserved */
	{0x0F12, 0x480F,},/* reserved */
	{0x0F12, 0xF000,},/* reserved */
	{0x0F12, 0xF9E2,},/* reserved */
	{0x0F12, 0x490E,},/* reserved */
	{0x0F12, 0x480F,},/* reserved */
	{0x0F12, 0xF000,},/* reserved */
	{0x0F12, 0xF9DE,},/* reserved */
	{0x0F12, 0x490E,},/* reserved */
	{0x0F12, 0x480F,},/* reserved */
	{0x0F12, 0xF000,},/* reserved */
	{0x0F12, 0xF9DA,},/* reserved */
	{0x0F12, 0x480E,},/* reserved */
	{0x0F12, 0x490F,},/* reserved */
	{0x0F12, 0x6448,},/* reserved */
	{0x0F12, 0xBC10,},/* reserved */
	{0x0F12, 0xBC08,},/* reserved */
	{0x0F12, 0x4718,},/* reserved */
	{0x0F12, 0x27CC,},/* reserved */
	{0x0F12, 0x7000,},/* reserved */
	{0x0F12, 0x8EDD,},/* reserved */
	{0x0F12, 0x0000,},/* reserved */
	{0x0F12, 0x2744,},/* reserved */
	{0x0F12, 0x7000,},/* reserved */
	{0x0F12, 0x8725,},/* reserved */
	{0x0F12, 0x0000,},/* reserved */
	{0x0F12, 0x26E4,},/* reserved */
	{0x0F12, 0x7000,},/* reserved */
	{0x0F12, 0x0080,},/* reserved */
	{0x0F12, 0x7000,},/* reserved */
	{0x0F12, 0x2638,},/* reserved */
	{0x0F12, 0x7000,},/* reserved */
	{0x0F12, 0xA6EF,},/* reserved */
	{0x0F12, 0x0000,},/* reserved */
	{0x0F12, 0x2604,},/* reserved */
	{0x0F12, 0x7000,},/* reserved */
	{0x0F12, 0xA0F1,},/* reserved */
	{0x0F12, 0x0000,},/* reserved */
	{0x0F12, 0x25D0,},/* reserved */
	{0x0F12, 0x7000,},/* reserved */
	{0x0F12, 0x058F,},/* reserved */
	{0x0F12, 0x0000,},/* reserved */
	{0x0F12, 0x24E4,},/* reserved */
	{0x0F12, 0x7000,},/* reserved */
	{0x0F12, 0x0000,},/* reserved */
	{0x0F12, 0x7000,},/* reserved */
	{0x0F12, 0x403E,},/* reserved */
	{0x0F12, 0xE92D,},/* reserved */
	{0x0F12, 0x00DD,},/* reserved */
	{0x0F12, 0xEB00,},/* reserved */
	{0x0F12, 0x2000,},/* reserved */
	{0x0F12, 0xE3A0,},/* reserved */
	{0x0F12, 0x1002,},/* reserved */
	{0x0F12, 0xE3A0,},/* reserved */
	{0x0F12, 0x0F86,},/* reserved */
	{0x0F12, 0xE3A0,},/* reserved */
	{0x0F12, 0x00DC,},/* reserved */
	{0x0F12, 0xEB00,},/* reserved */
	{0x0F12, 0x200A,},/* reserved */
	{0x0F12, 0xE3A0,},/* reserved */
	{0x0F12, 0x1000,},/* reserved */
	{0x0F12, 0xE28D,},/* reserved */
	{0x0F12, 0x0E3F,},/* reserved */
	{0x0F12, 0xE3A0,},/* reserved */
	{0x0F12, 0x00DB,},/* reserved */
	{0x0F12, 0xEB00,},/* reserved */
	{0x0F12, 0x2001,},/* reserved */
	{0x0F12, 0xE3A0,},/* reserved */
	{0x0F12, 0x1002,},/* reserved */
	{0x0F12, 0xE3A0,},/* reserved */
	{0x0F12, 0x0F86,},/* reserved */
	{0x0F12, 0xE3A0,},/* reserved */
	{0x0F12, 0x00D4,},/* reserved */
	{0x0F12, 0xEB00,},/* reserved */
	{0x0F12, 0x0000,},/* reserved */
	{0x0F12, 0xE5DD,},/* reserved */
	{0x0F12, 0x00C3,},/* reserved */
	{0x0F12, 0xE350,},/* reserved */
	{0x0F12, 0x0027,},/* reserved */
	{0x0F12, 0x1A00,},/* reserved */
	{0x0F12, 0x0001,},/* reserved */
	{0x0F12, 0xE5DD,},/* reserved */
	{0x0F12, 0x003C,},/* reserved */
	{0x0F12, 0xE350,},/* reserved */
	{0x0F12, 0x0024,},/* reserved */
	{0x0F12, 0x1A00,},/* reserved */
	{0x0F12, 0x02E0,},/* reserved */
	{0x0F12, 0xE59F,},/* reserved */
	{0x0F12, 0x1000,},/* reserved */
	{0x0F12, 0xE5D0,},/* reserved */
	{0x0F12, 0x0000,},/* reserved */
	{0x0F12, 0xE351,},/* reserved */
	{0x0F12, 0x0003,},/* reserved */
	{0x0F12, 0x1A00,},/* reserved */
	{0x0F12, 0x12D4,},/* reserved */
	{0x0F12, 0xE59F,},/* reserved */
	{0x0F12, 0x10B8,},/* reserved */
	{0x0F12, 0xE1D1,},/* reserved */
	{0x0F12, 0x0000,},/* reserved */
	{0x0F12, 0xE351,},/* reserved */
	{0x0F12, 0x001C,},/* reserved */
	{0x0F12, 0x0A00,},/* reserved */
	{0x0F12, 0x1000,},/* reserved */
	{0x0F12, 0xE3A0,},/* reserved */
	{0x0F12, 0x1000,},/* reserved */
	{0x0F12, 0xE5C0,},/* reserved */
	{0x0F12, 0x0000,},/* reserved */
	{0x0F12, 0xE3A0,},/* reserved */
	{0x0F12, 0x1002,},/* reserved */
	{0x0F12, 0xE28D,},/* reserved */
	{0x0F12, 0x0015,},/* reserved */
	{0x0F12, 0xEA00,},/* reserved */
	{0x0F12, 0x2000,},/* reserved */
	{0x0F12, 0xE5D1,},/* reserved */
	{0x0F12, 0x3001,},/* reserved */
	{0x0F12, 0xE5D1,},/* reserved */
	{0x0F12, 0x3403,},/* reserved */
	{0x0F12, 0xE182,},/* reserved */
	{0x0F12, 0xC2A8,},/* reserved */
	{0x0F12, 0xE59F,},/* reserved */
	{0x0F12, 0x2080,},/* reserved */
	{0x0F12, 0xE08C,},/* reserved */
	{0x0F12, 0xE7B4,},/* reserved */
	{0x0F12, 0xE1D2,},/* reserved */
	{0x0F12, 0x039E,},/* reserved */
	{0x0F12, 0xE004,},/* reserved */
	{0x0F12, 0xE80F,},/* reserved */
	{0x0F12, 0xE3E0,},/* reserved */
	{0x0F12, 0x4624,},/* reserved */
	{0x0F12, 0xE00E,},/* reserved */
	{0x0F12, 0x47B4,},/* reserved */
	{0x0F12, 0xE1C2,},/* reserved */
	{0x0F12, 0x4004,},/* reserved */
	{0x0F12, 0xE280,},/* reserved */
	{0x0F12, 0xC084,},/* reserved */
	{0x0F12, 0xE08C,},/* reserved */
	{0x0F12, 0x47B4,},/* reserved */
	{0x0F12, 0xE1DC,},/* reserved */
	{0x0F12, 0x0493,},/* reserved */
	{0x0F12, 0xE004,},/* reserved */
	{0x0F12, 0x4624,},/* reserved */
	{0x0F12, 0xE00E,},/* reserved */
	{0x0F12, 0x47B4,},/* reserved */
	{0x0F12, 0xE1CC,},/* reserved */
	{0x0F12, 0xC8B4,},/* reserved */
	{0x0F12, 0xE1D2,},/* reserved */
	{0x0F12, 0x039C,},/* reserved */
	{0x0F12, 0xE003,},/* reserved */
	{0x0F12, 0x3623,},/* reserved */
	{0x0F12, 0xE00E,},/* reserved */
	{0x0F12, 0x38B4,},/* reserved */
	{0x0F12, 0xE1C2,},/* reserved */
	{0x0F12, 0x0001,},/* reserved */
	{0x0F12, 0xE280,},/* reserved */
	{0x0F12, 0x1002,},/* reserved */
	{0x0F12, 0xE281,},/* reserved */
	{0x0F12, 0x0004,},/* reserved */
	{0x0F12, 0xE350,},/* reserved */
	{0x0F12, 0xFFE7,},/* reserved */
	{0x0F12, 0xBAFF,},/* reserved */
	{0x0F12, 0x403E,},/* reserved */
	{0x0F12, 0xE8BD,},/* reserved */
	{0x0F12, 0xFF1E,},/* reserved */
	{0x0F12, 0xE12F,},/* reserved */
	{0x0F12, 0x4010,},/* reserved */
	{0x0F12, 0xE92D,},/* reserved */
	{0x0F12, 0x00AB,},/* reserved */
	{0x0F12, 0xEB00,},/* reserved */
	{0x0F12, 0x0248,},/* reserved */
	{0x0F12, 0xE59F,},/* reserved */
	{0x0F12, 0x00B2,},/* reserved */
	{0x0F12, 0xE1D0,},/* reserved */
	{0x0F12, 0x0000,},/* reserved */
	{0x0F12, 0xE350,},/* reserved */
	{0x0F12, 0x0004,},/* reserved */
	{0x0F12, 0x0A00,},/* reserved */
	{0x0F12, 0x0080,},/* reserved */
	{0x0F12, 0xE310,},/* reserved */
	{0x0F12, 0x0002,},/* reserved */
	{0x0F12, 0x1A00,},/* reserved */
	{0x0F12, 0x1234,},/* reserved */
	{0x0F12, 0xE59F,},/* reserved */
	{0x0F12, 0x0001,},/* reserved */
	{0x0F12, 0xE3A0,},/* reserved */
	{0x0F12, 0x0DB2,},/* reserved */
	{0x0F12, 0xE1C1,},/* reserved */
	{0x0F12, 0x4010,},/* reserved */
	{0x0F12, 0xE8BD,},/* reserved */
	{0x0F12, 0xFF1E,},/* reserved */
	{0x0F12, 0xE12F,},/* reserved */
	{0x0F12, 0x4010,},/* reserved */
	{0x0F12, 0xE92D,},/* reserved */
	{0x0F12, 0x4000,},/* reserved */
	{0x0F12, 0xE590,},/* reserved */
	{0x0F12, 0x0004,},/* reserved */
	{0x0F12, 0xE1A0,},/* reserved */
	{0x0F12, 0x009F,},/* reserved */
	{0x0F12, 0xEB00,},/* reserved */
	{0x0F12, 0x0214,},/* reserved */
	{0x0F12, 0xE59F,},/* reserved */
	{0x0F12, 0x0000,},/* reserved */
	{0x0F12, 0xE5D0,},/* reserved */
	{0x0F12, 0x0000,},/* reserved */
	{0x0F12, 0xE350,},/* reserved */
	{0x0F12, 0x0002,},/* reserved */
	{0x0F12, 0x0A00,},/* reserved */
	{0x0F12, 0x0004,},/* reserved */
	{0x0F12, 0xE594,},/* reserved */
	{0x0F12, 0x00A0,},/* reserved */
	{0x0F12, 0xE1A0,},/* reserved */
	{0x0F12, 0x0004,},/* reserved */
	{0x0F12, 0xE584,},/* reserved */
	{0x0F12, 0x4010,},/* reserved */
	{0x0F12, 0xE8BD,},/* reserved */
	{0x0F12, 0xFF1E,},/* reserved */
	{0x0F12, 0xE12F,},/* reserved */
	{0x0F12, 0x4070,},/* reserved */
	{0x0F12, 0xE92D,},/* reserved */
	{0x0F12, 0x0000,},/* reserved */
	{0x0F12, 0xE590,},/* reserved */
	{0x0F12, 0x0800,},/* reserved */
	{0x0F12, 0xE1A0,},/* reserved */
	{0x0F12, 0x0820,},/* reserved */
	{0x0F12, 0xE1A0,},/* reserved */
	{0x0F12, 0x4041,},/* reserved */
	{0x0F12, 0xE280,},/* reserved */
	{0x0F12, 0x01E0,},/* reserved */
	{0x0F12, 0xE59F,},/* reserved */
	{0x0F12, 0x11B8,},/* reserved */
	{0x0F12, 0xE1D0,},/* reserved */
	{0x0F12, 0x51B6,},/* reserved */
	{0x0F12, 0xE1D0,},/* reserved */
	{0x0F12, 0x0005,},/* reserved */
	{0x0F12, 0xE041,},/* reserved */
	{0x0F12, 0x0094,},/* reserved */
	{0x0F12, 0xE000,},/* reserved */
	{0x0F12, 0x1D11,},/* reserved */
	{0x0F12, 0xE3A0,},/* reserved */
	{0x0F12, 0x008D,},/* reserved */
	{0x0F12, 0xEB00,},/* reserved */
	{0x0F12, 0x11C0,},/* reserved */
	{0x0F12, 0xE59F,},/* reserved */
	{0x0F12, 0x1000,},/* reserved */
	{0x0F12, 0xE5D1,},/* reserved */
	{0x0F12, 0x0000,},/* reserved */
	{0x0F12, 0xE351,},/* reserved */
	{0x0F12, 0x0000,},/* reserved */
	{0x0F12, 0x0A00,},/* reserved */
	{0x0F12, 0x00A0,},/* reserved */
	{0x0F12, 0xE1A0,},/* reserved */
	{0x0F12, 0x21A8,},/* reserved */
	{0x0F12, 0xE59F,},/* reserved */
	{0x0F12, 0x3FB0,},/* reserved */
	{0x0F12, 0xE1D2,},/* reserved */
	{0x0F12, 0x0000,},/* reserved */
	{0x0F12, 0xE353,},/* reserved */
	{0x0F12, 0x0003,},/* reserved */
	{0x0F12, 0x0A00,},/* reserved */
	{0x0F12, 0x31A4,},/* reserved */
	{0x0F12, 0xE59F,},/* reserved */
	{0x0F12, 0x5BB2,},/* reserved */
	{0x0F12, 0xE1C3,},/* reserved */
	{0x0F12, 0xC000,},/* reserved */
	{0x0F12, 0xE085,},/* reserved */
	{0x0F12, 0xCBB4,},/* reserved */
	{0x0F12, 0xE1C3,},/* reserved */
	{0x0F12, 0x0000,},/* reserved */
	{0x0F12, 0xE351,},/* reserved */
	{0x0F12, 0x0000,},/* reserved */
	{0x0F12, 0x0A00,},/* reserved */
	{0x0F12, 0x0080,},/* reserved */
	{0x0F12, 0xE1A0,},/* reserved */
	{0x0F12, 0x1DBC,},/* reserved */
	{0x0F12, 0xE1D2,},/* reserved */
	{0x0F12, 0x3EB4,},/* reserved */
	{0x0F12, 0xE1D2,},/* reserved */
	{0x0F12, 0x2EB2,},/* reserved */
	{0x0F12, 0xE1D2,},/* reserved */
	{0x0F12, 0x0193,},/* reserved */
	{0x0F12, 0xE001,},/* reserved */
	{0x0F12, 0x0092,},/* reserved */
	{0x0F12, 0xE000,},/* reserved */
	{0x0F12, 0x2811,},/* reserved */
	{0x0F12, 0xE3A0,},/* reserved */
	{0x0F12, 0x0194,},/* reserved */
	{0x0F12, 0xE001,},/* reserved */
	{0x0F12, 0x0092,},/* reserved */
	{0x0F12, 0xE000,},/* reserved */
	{0x0F12, 0x11A1,},/* reserved */
	{0x0F12, 0xE1A0,},/* reserved */
	{0x0F12, 0x01A0,},/* reserved */
	{0x0F12, 0xE1A0,},/* reserved */
	{0x0F12, 0x0072,},/* reserved */
	{0x0F12, 0xEB00,},/* reserved */
	{0x0F12, 0x1160,},/* reserved */
	{0x0F12, 0xE59F,},/* reserved */
	{0x0F12, 0x02B4,},/* reserved */
	{0x0F12, 0xE1C1,},/* reserved */
	{0x0F12, 0x4070,},/* reserved */
	{0x0F12, 0xE8BD,},/* reserved */
	{0x0F12, 0xFF1E,},/* reserved */
	{0x0F12, 0xE12F,},/* reserved */
	{0x0F12, 0x4010,},/* reserved */
	{0x0F12, 0xE92D,},/* reserved */
	{0x0F12, 0x006E,},/* reserved */
	{0x0F12, 0xEB00,},/* reserved */
	{0x0F12, 0x2148,},/* reserved */
	{0x0F12, 0xE59F,},/* reserved */
	{0x0F12, 0x14B0,},/* reserved */
	{0x0F12, 0xE1D2,},/* reserved */
	{0x0F12, 0x0080,},/* reserved */
	{0x0F12, 0xE311,},/* reserved */
	{0x0F12, 0x0005,},/* reserved */
	{0x0F12, 0x0A00,},/* reserved */
	{0x0F12, 0x013C,},/* reserved */
	{0x0F12, 0xE59F,},/* reserved */
	{0x0F12, 0x00B0,},/* reserved */
	{0x0F12, 0xE1D0,},/* reserved */
	{0x0F12, 0x0001,},/* reserved */
	{0x0F12, 0xE350,},/* reserved */
	{0x0F12, 0x0001,},/* reserved */
	{0x0F12, 0x9A00,},/* reserved */
	{0x0F12, 0x0001,},/* reserved */
	{0x0F12, 0xE3A0,},/* reserved */
	{0x0F12, 0x0000,},/* reserved */
	{0x0F12, 0xEA00,},/* reserved */
	{0x0F12, 0x0000,},/* reserved */
	{0x0F12, 0xE3A0,},/* reserved */
	{0x0F12, 0x3110,},/* reserved */
	{0x0F12, 0xE59F,},/* reserved */
	{0x0F12, 0x0000,},/* reserved */
	{0x0F12, 0xE5C3,},/* reserved */
	{0x0F12, 0x0000,},/* reserved */
	{0x0F12, 0xE5D3,},/* reserved */
	{0x0F12, 0x0000,},/* reserved */
	{0x0F12, 0xE350,},/* reserved */
	{0x0F12, 0x0003,},/* reserved */
	{0x0F12, 0x0A00,},/* reserved */
	{0x0F12, 0x0080,},/* reserved */
	{0x0F12, 0xE3C1,},/* reserved */
	{0x0F12, 0x110C,},/* reserved */
	{0x0F12, 0xE59F,},/* reserved */
	{0x0F12, 0x04B0,},/* reserved */
	{0x0F12, 0xE1C2,},/* reserved */
	{0x0F12, 0x00B2,},/* reserved */
	{0x0F12, 0xE1C1,},/* reserved */
	{0x0F12, 0x4010,},/* reserved */
	{0x0F12, 0xE8BD,},/* reserved */
	{0x0F12, 0xFF1E,},/* reserved */
	{0x0F12, 0xE12F,},/* reserved */
	{0x0F12, 0x41F0,},/* reserved */
	{0x0F12, 0xE92D,},/* reserved */
	{0x0F12, 0x1000,},/* reserved */
	{0x0F12, 0xE590,},/* reserved */
	{0x0F12, 0xC801,},/* reserved */
	{0x0F12, 0xE1A0,},/* reserved */
	{0x0F12, 0xC82C,},/* reserved */
	{0x0F12, 0xE1A0,},/* reserved */
	{0x0F12, 0x1004,},/* reserved */
	{0x0F12, 0xE590,},/* reserved */
	{0x0F12, 0x1801,},/* reserved */
	{0x0F12, 0xE1A0,},/* reserved */
	{0x0F12, 0x1821,},/* reserved */
	{0x0F12, 0xE1A0,},/* reserved */
	{0x0F12, 0x4008,},/* reserved */
	{0x0F12, 0xE590,},/* reserved */
	{0x0F12, 0x500C,},/* reserved */
	{0x0F12, 0xE590,},/* reserved */
	{0x0F12, 0x2004,},/* reserved */
	{0x0F12, 0xE1A0,},/* reserved */
	{0x0F12, 0x3005,},/* reserved */
	{0x0F12, 0xE1A0,},/* reserved */
	{0x0F12, 0x000C,},/* reserved */
	{0x0F12, 0xE1A0,},/* reserved */
	{0x0F12, 0x004E,},/* reserved */
	{0x0F12, 0xEB00,},/* reserved */
	{0x0F12, 0x60A0,},/* reserved */
	{0x0F12, 0xE59F,},/* reserved */
	{0x0F12, 0x00B2,},/* reserved */
	{0x0F12, 0xE1D6,},/* reserved */
	{0x0F12, 0x0000,},/* reserved */
	{0x0F12, 0xE350,},/* reserved */
	{0x0F12, 0x000E,},/* reserved */
	{0x0F12, 0x0A00,},/* reserved */
	{0x0F12, 0x00B8,},/* reserved */
	{0x0F12, 0xE59F,},/* reserved */
	{0x0F12, 0x05B4,},/* reserved */
	{0x0F12, 0xE1D0,},/* reserved */
	{0x0F12, 0x0002,},/* reserved */
	{0x0F12, 0xE350,},/* reserved */
	{0x0F12, 0x000A,},/* reserved */
	{0x0F12, 0x1A00,},/* reserved */
	{0x0F12, 0x70AC,},/* reserved */
	{0x0F12, 0xE59F,},/* reserved */
	{0x0F12, 0x10F4,},/* reserved */
	{0x0F12, 0xE1D6,},/* reserved */
	{0x0F12, 0x26B0,},/* reserved */
	{0x0F12, 0xE1D7,},/* reserved */
	{0x0F12, 0x00F0,},/* reserved */
	{0x0F12, 0xE1D4,},/* reserved */
	{0x0F12, 0x0044,},/* reserved */
	{0x0F12, 0xEB00,},/* reserved */
	{0x0F12, 0x00B0,},/* reserved */
	{0x0F12, 0xE1C4,},/* reserved */
	{0x0F12, 0x26B0,},/* reserved */
	{0x0F12, 0xE1D7,},/* reserved */
	{0x0F12, 0x10F6,},/* reserved */
	{0x0F12, 0xE1D6,},/* reserved */
	{0x0F12, 0x00F0,},/* reserved */
	{0x0F12, 0xE1D5,},/* reserved */
	{0x0F12, 0x003F,},/* reserved */
	{0x0F12, 0xEB00,},/* reserved */
	{0x0F12, 0x00B0,},/* reserved */
	{0x0F12, 0xE1C5,},/* reserved */
	{0x0F12, 0x41F0,},/* reserved */
	{0x0F12, 0xE8BD,},/* reserved */
	{0x0F12, 0xFF1E,},/* reserved */
	{0x0F12, 0xE12F,},/* reserved */
	{0x0F12, 0x4010,},/* reserved */
	{0x0F12, 0xE92D,},/* reserved */
	{0x0F12, 0x4000,},/* reserved */
	{0x0F12, 0xE1A0,},/* reserved */
	{0x0F12, 0x1004,},/* reserved */
	{0x0F12, 0xE594,},/* reserved */
	{0x0F12, 0x0040,},/* reserved */
	{0x0F12, 0xE59F,},/* reserved */
	{0x0F12, 0x00B0,},/* reserved */
	{0x0F12, 0xE1D0,},/* reserved */
	{0x0F12, 0x0000,},/* reserved */
	{0x0F12, 0xE350,},/* reserved */
	{0x0F12, 0x0008,},/* reserved */
	{0x0F12, 0x0A00,},/* reserved */
	{0x0F12, 0x005C,},/* reserved */
	{0x0F12, 0xE59F,},/* reserved */
	{0x0F12, 0x3001,},/* reserved */
	{0x0F12, 0xE1A0,},/* reserved */
	{0x0F12, 0x2068,},/* reserved */
	{0x0F12, 0xE590,},/* reserved */
	{0x0F12, 0x0054,},/* reserved */
	{0x0F12, 0xE59F,},/* reserved */
	{0x0F12, 0x1005,},/* reserved */
	{0x0F12, 0xE3A0,},/* reserved */
	{0x0F12, 0x0032,},/* reserved */
	{0x0F12, 0xEB00,},/* reserved */
	{0x0F12, 0x0000,},/* reserved */
	{0x0F12, 0xE584,},/* reserved */
	{0x0F12, 0x4010,},/* reserved */
	{0x0F12, 0xE8BD,},/* reserved */
	{0x0F12, 0xFF1E,},/* reserved */
	{0x0F12, 0xE12F,},/* reserved */
	{0x0F12, 0x0000,},/* reserved */
	{0x0F12, 0xE594,},/* reserved */
	{0x0F12, 0x0030,},/* reserved */
	{0x0F12, 0xEB00,},/* reserved */
	{0x0F12, 0x0000,},/* reserved */
	{0x0F12, 0xE584,},/* reserved */
	{0x0F12, 0xFFF9,},/* reserved */
	{0x0F12, 0xEAFF,},/* reserved */
	{0x0F12, 0x28E8,},/* reserved */
	{0x0F12, 0x7000,},/* reserved */
	{0x0F12, 0x3370,},/* reserved */
	{0x0F12, 0x7000,},/* reserved */
	{0x0F12, 0x1272,},/* reserved */
	{0x0F12, 0x7000,},/* reserved */
	{0x0F12, 0x1728,},/* reserved */
	{0x0F12, 0x7000,},/* reserved */
	{0x0F12, 0x112C,},/* reserved */
	{0x0F12, 0x7000,},/* reserved */
	{0x0F12, 0x28EC,},/* reserved */
	{0x0F12, 0x7000,},/* reserved */
	{0x0F12, 0x122C,},/* reserved */
	{0x0F12, 0x7000,},/* reserved */
	{0x0F12, 0xF200,},/* reserved */
	{0x0F12, 0xD000,},/* reserved */
	{0x0F12, 0x2340,},/* reserved */
	{0x0F12, 0x7000,},/* reserved */
	{0x0F12, 0x0E2C,},/* reserved */
	{0x0F12, 0x7000,},/* reserved */
	{0x0F12, 0xF400,},/* reserved */
	{0x0F12, 0xD000,},/* reserved */
	{0x0F12, 0x0CDC,},/* reserved */
	{0x0F12, 0x7000,},/* reserved */
	{0x0F12, 0x20D4,},/* reserved */
	{0x0F12, 0x7000,},/* reserved */
	{0x0F12, 0x06D4,},/* reserved */
	{0x0F12, 0x7000,},/* reserved */
	{0x0F12, 0x4778,},/* reserved */
	{0x0F12, 0x46C0,},/* reserved */
	{0x0F12, 0xC000,},/* reserved */
	{0x0F12, 0xE59F,},/* reserved */
	{0x0F12, 0xFF1C,},/* reserved */
	{0x0F12, 0xE12F,},/* reserved */
	{0x0F12, 0xC091,},/* reserved */
	{0x0F12, 0x0000,},/* reserved */
	{0x0F12, 0xC000,},/* reserved */
	{0x0F12, 0xE59F,},/* reserved */
	{0x0F12, 0xFF1C,},/* reserved */
	{0x0F12, 0xE12F,},/* reserved */
	{0x0F12, 0x0467,},/* reserved */
	{0x0F12, 0x0000,},/* reserved */
	{0x0F12, 0xC000,},/* reserved */
	{0x0F12, 0xE59F,},/* reserved */
	{0x0F12, 0xFF1C,},/* reserved */
	{0x0F12, 0xE12F,},/* reserved */
	{0x0F12, 0x2FA7,},/* reserved */
	{0x0F12, 0x0000,},/* reserved */
	{0x0F12, 0xC000,},/* reserved */
	{0x0F12, 0xE59F,},/* reserved */
	{0x0F12, 0xFF1C,},/* reserved */
	{0x0F12, 0xE12F,},/* reserved */
	{0x0F12, 0xCB1F,},/* reserved */
	{0x0F12, 0x0000,},/* reserved */
	{0x0F12, 0xC000,},/* reserved */
	{0x0F12, 0xE59F,},/* reserved */
	{0x0F12, 0xFF1C,},/* reserved */
	{0x0F12, 0xE12F,},/* reserved */
	{0x0F12, 0x058F,},/* reserved */
	{0x0F12, 0x0000,},/* reserved */
	{0x0F12, 0xC000,},/* reserved */
	{0x0F12, 0xE59F,},/* reserved */
	{0x0F12, 0xFF1C,},/* reserved */
	{0x0F12, 0xE12F,},/* reserved */
	{0x0F12, 0xA0F1,},/* reserved */
	{0x0F12, 0x0000,},/* reserved */
	{0x0F12, 0xF004,},/* reserved */
	{0x0F12, 0xE51F,},/* reserved */
	{0x0F12, 0xD14C,},/* reserved */
	{0x0F12, 0x0000,},/* reserved */
	{0x0F12, 0xC000,},/* reserved */
	{0x0F12, 0xE59F,},/* reserved */
	{0x0F12, 0xFF1C,},/* reserved */
	{0x0F12, 0xE12F,},/* reserved */
	{0x0F12, 0x2B43,},/* reserved */
	{0x0F12, 0x0000,},/* reserved */
	{0x0F12, 0xC000,},/* reserved */
	{0x0F12, 0xE59F,},/* reserved */
	{0x0F12, 0xFF1C,},/* reserved */
	{0x0F12, 0xE12F,},/* reserved */
	{0x0F12, 0x8725,},/* reserved */
	{0x0F12, 0x0000,},/* reserved */
	{0x0F12, 0xC000,},/* reserved */
	{0x0F12, 0xE59F,},/* reserved */
	{0x0F12, 0xFF1C,},/* reserved */
	{0x0F12, 0xE12F,},/* reserved */
	{0x0F12, 0x6777,},/* reserved */
	{0x0F12, 0x0000,},/* reserved */
	{0x0F12, 0xC000,},/* reserved */
	{0x0F12, 0xE59F,},/* reserved */
	{0x0F12, 0xFF1C,},/* reserved */
	{0x0F12, 0xE12F,},/* reserved */
	{0x0F12, 0x8E49,},/* reserved */
	{0x0F12, 0x0000,},/* reserved */
	{0x0F12, 0xC000,},/* reserved */
	{0x0F12, 0xE59F,},/* reserved */
	{0x0F12, 0xFF1C,},/* reserved */
	{0x0F12, 0xE12F,},/* reserved */
	{0x0F12, 0x8EDD,},/* reserved */
	{0x0F12, 0x0000,},/* reserved */
	{0x0F12, 0x96FF,},/* reserved */
	{0x0F12, 0x0000,},/* reserved */
	{0x0F12, 0x0001,},/* reserved */
	{0x0F12, 0x0000,},/* reserved */
	{0x0028, 0x7000,},/* reserved */
	{0x002A, 0x0E38,},/* reserved */
	{0x0F12, 0x0476,},/* senHal_RegCompBiasNormSf */
	{0x0F12, 0x0476,},/* senHal_RegCompBiasYAv */
	{0x002A, 0x0AA0,},/* reserved */
	{0x0F12, 0x0001,},/* setot_bUseDigitalHbin */
	{0x002A, 0x0E2C,},/* reserved */
	{0x0F12, 0x0001,},/* senHal_bUseAnalogVerAv */
	{0x002A, 0x0E66,},/* reserved */
	{0x0F12, 0x0001,},/* senHal_RegBlstEnNorm */
	{0x002A, 0x1250,},/* reserved */
	{0x0F12, 0xFFFF,},/* senHal_Bls_nSpExpLines */
	{0x002A, 0x1202,},/* reserved */
	{0x0F12, 0x0010,},/* senHal_Dblr_VcoFreqMHZ */
	{0x002A, 0x1288,},/* reserved */
	{0x0F12, 0x020F,},/* gisp_dadlc_ResetFilterValue */
	{0x0F12, 0x1C02,},/* gisp_dadlc_SteadyFilterValue */
	{0x0F12, 0x0006,},/* gisp_dadlc_NResetIIrFrames */
	{0x002A, 0x3378,},/* reserved */
	{0x0F12, 0x0000,},/* Tune_TP_bReMultGainsByNvm */
	{0x002A, 0x1326,},/* reserved */
	{0x0F12, 0x0000,},/* gisp_gos_Enable */
	{0x002A, 0x063A,},/* reserved */
	{0x0F12, 0x00E0,},/* TVAR_ash_GASAlpha_0__0_ */
	{0x0F12, 0x0100,},/* TVAR_ash_GASAlpha_0__1_ */
	{0x0F12, 0x0100,},/* TVAR_ash_GASAlpha_0__2_ */
	{0x0F12, 0x0100,},/* TVAR_ash_GASAlpha_0__3_ */
	{0x0F12, 0x00E0,},/* TVAR_ash_GASAlpha_1__0_ */
	{0x0F12, 0x0100,},/* TVAR_ash_GASAlpha_1__1_ */
	{0x0F12, 0x0100,},/* TVAR_ash_GASAlpha_1__2_ */
	{0x0F12, 0x0100,},/* TVAR_ash_GASAlpha_1__3_ */
	{0x0F12, 0x00C8,},/* TVAR_ash_GASAlpha_2__0_ */
	{0x0F12, 0x0100,},/* TVAR_ash_GASAlpha_2__1_ */
	{0x0F12, 0x0100,},/* TVAR_ash_GASAlpha_2__2_ */
	{0x0F12, 0x0100,},/* TVAR_ash_GASAlpha_2__3_ */
	{0x0F12, 0x00E8,},/* TVAR_ash_GASAlpha_3__0_ */
	{0x0F12, 0x0100,},/* TVAR_ash_GASAlpha_3__1_ */
	{0x0F12, 0x0100,},/* TVAR_ash_GASAlpha_3__2_ */
	{0x0F12, 0x0100,},/* TVAR_ash_GASAlpha_3__3_ */
	{0x0F12, 0x00E8,},/* TVAR_ash_GASAlpha_4__0_ */
	{0x0F12, 0x00F8,},/* TVAR_ash_GASAlpha_4__1_ */
	{0x0F12, 0x00F8,},/* TVAR_ash_GASAlpha_4__2_ */
	{0x0F12, 0x0100,},/* TVAR_ash_GASAlpha_4__3_ */
	{0x0F12, 0x00F0,},/* TVAR_ash_GASAlpha_5__0_ */
	{0x0F12, 0x0100,},/* TVAR_ash_GASAlpha_5__1_ */
	{0x0F12, 0x0100,},/* TVAR_ash_GASAlpha_5__2_ */
	{0x0F12, 0x0100,},/* TVAR_ash_GASAlpha_5__3_ */
	{0x0F12, 0x00F0,},/* TVAR_ash_GASAlpha_6__0_ */
	{0x0F12, 0x0100,},/* TVAR_ash_GASAlpha_6__1_ */
	{0x0F12, 0x0100,},/* TVAR_ash_GASAlpha_6__2_ */
	{0x0F12, 0x0100,},/* TVAR_ash_GASAlpha_6__3_ */
	{0x002A, 0x067A,},/* reserved */
	{0x0F12, 0x0000,},/* ash_GASBeta_0__0_ */
	{0x0F12, 0x0000,},/* ash_GASBeta_0__1_ */
	{0x0F12, 0x0000,},/* ash_GASBeta_0__2_ */
	{0x0F12, 0x0000,},/* ash_GASBeta_0__3_ */
	{0x0F12, 0x0000,},/* ash_GASBeta_1__0_ */
	{0x0F12, 0x0000,},/* ash_GASBeta_1__1_ */
	{0x0F12, 0x0000,},/* ash_GASBeta_1__2_ */
	{0x0F12, 0x0000,},/* ash_GASBeta_1__3_ */
	{0x0F12, 0x0000,},/* ash_GASBeta_2__0_ */
	{0x0F12, 0x0000,},/* ash_GASBeta_2__1_ */
	{0x0F12, 0x0000,},/* ash_GASBeta_2__2_ */
	{0x0F12, 0x0000,},/* ash_GASBeta_2__3_ */
	{0x0F12, 0x0000,},/* ash_GASBeta_3__0_ */
	{0x0F12, 0x0000,},/* ash_GASBeta_3__1_ */
	{0x0F12, 0x0000,},/* ash_GASBeta_3__2_ */
	{0x0F12, 0x0000,},/* ash_GASBeta_3__3_ */
	{0x0F12, 0x0000,},/* ash_GASBeta_4__0_ */
	{0x0F12, 0x0000,},/* ash_GASBeta_4__1_ */
	{0x0F12, 0x0000,},/* ash_GASBeta_4__2_ */
	{0x0F12, 0x0000,},/* ash_GASBeta_4__3_ */
	{0x0F12, 0x0000,},/* ash_GASBeta_5__0_ */
	{0x0F12, 0x0000,},/* ash_GASBeta_5__1_ */
	{0x0F12, 0x0000,},/* ash_GASBeta_5__2_ */
	{0x0F12, 0x0000,},/* ash_GASBeta_5__3_ */
	{0x0F12, 0x0000,},/* ash_GASBeta_6__0_ */
	{0x0F12, 0x0000,},/* ash_GASBeta_6__1_ */
	{0x0F12, 0x0000,},/* ash_GASBeta_6__2_ */
	{0x0F12, 0x0000,},/* ash_GASBeta_6__3_ */
	{0x002A, 0x06BA,},/* reserved */
	{0x0F12, 0x0001,},/* ash_bLumaMode */
	{0x002A, 0x0632,},/* reserved */
	{0x0F12, 0x0100,},/* ash_CGrasAlphas_0_ */
	{0x0F12, 0x0100,},/* ash_CGrasAlphas_1_ */
	{0x0F12, 0x0100,},/* ash_CGrasAlphas_2_ */
	{0x0F12, 0x0100,},/* ash_CGrasAlphas_3_ */
	{0x002A, 0x0672,},/* reserved */
	{0x0F12, 0x0100,},/* TVAR_ash_GASOutdoorAlpha_0_ */
	{0x0F12, 0x0100,},/* TVAR_ash_GASOutdoorAlpha_1_ */
	{0x0F12, 0x0100,},/* TVAR_ash_GASOutdoorAlpha_2_ */
	{0x0F12, 0x0100,},/* TVAR_ash_GASOutdoorAlpha_3_ */
	{0x002A, 0x06B2,},/* reserved */
	{0x0F12, 0x0000,},/* ash_GASOutdoorBeta_0_ */
	{0x0F12, 0x0000,},/* ash_GASOutdoorBeta_1_ */
	{0x0F12, 0x0000,},/* ash_GASOutdoorBeta_2_ */
	{0x0F12, 0x0000,},/* ash_GASOutdoorBeta_3_ */
	{0x002A, 0x06D0,},/* reserved */
	{0x0F12, 0x000D,},/* ash_uParabolicScalingA */
	{0x0F12, 0x000F,},/* ash_uParabolicScalingB */
	{0x002A, 0x06CC,},/* reserved */
	{0x0F12, 0x0280,},/* ash_uParabolicCenterX */
	{0x0F12, 0x01E0,},/* ash_uParabolicCenterY */
	{0x002A, 0x06C6,},/* reserved */
	{0x0F12, 0x0001,},/* ash_bParabolicEstimation */
	{0x002A, 0x0624,},/* reserved */
	{0x0F12, 0x009D,},/* TVAR_ash_AwbAshCord_0_ */
	{0x0F12, 0x00D5,},/* TVAR_ash_AwbAshCord_1_ */
	{0x0F12, 0x0103,},/* TVAR_ash_AwbAshCord_2_ */
	{0x0F12, 0x0128,},/* TVAR_ash_AwbAshCord_3_ */
	{0x0F12, 0x0166,},/* TVAR_ash_AwbAshCord_4_ */
	{0x0F12, 0x0193,},/* TVAR_ash_AwbAshCord_5_ */
	{0x0F12, 0x01A0,},/* TVAR_ash_AwbAshCord_6_ */
	{0x002A, 0x347C,},/* reserved */
	{0x0F12, 0x013B,},/* Tune_wbt_GAS_0_ */
	{0x0F12, 0x0116,},/* Tune_wbt_GAS_1_ */
	{0x0F12, 0x00D9,},/* Tune_wbt_GAS_2_ */
	{0x0F12, 0x00A6,},/* Tune_wbt_GAS_3_ */
	{0x0F12, 0x0082,},/* Tune_wbt_GAS_4_ */
	{0x0F12, 0x006C,},/* Tune_wbt_GAS_5_ */
	{0x0F12, 0x0065,},/* Tune_wbt_GAS_6_ */
	{0x0F12, 0x006C,},/* Tune_wbt_GAS_7_ */
	{0x0F12, 0x0080,},/* Tune_wbt_GAS_8_ */
	{0x0F12, 0x00A3,},/* Tune_wbt_GAS_9_ */
	{0x0F12, 0x00D4,},/* Tune_wbt_GAS_10_ */
	{0x0F12, 0x010D,},/* Tune_wbt_GAS_11_ */
	{0x0F12, 0x012E,},/* Tune_wbt_GAS_12_ */
	{0x0F12, 0x0138,},/* Tune_wbt_GAS_13_ */
	{0x0F12, 0x0104,},/* Tune_wbt_GAS_14_ */
	{0x0F12, 0x00BE,},/* Tune_wbt_GAS_15_ */
	{0x0F12, 0x0088,},/* Tune_wbt_GAS_16_ */
	{0x0F12, 0x0062,},/* Tune_wbt_GAS_17_ */
	{0x0F12, 0x004D,},/* Tune_wbt_GAS_18_ */
	{0x0F12, 0x0046,},/* Tune_wbt_GAS_19_ */
	{0x0F12, 0x004C,},/* Tune_wbt_GAS_20_ */
	{0x0F12, 0x0060,},/* Tune_wbt_GAS_21_ */
	{0x0F12, 0x0084,},/* Tune_wbt_GAS_22_ */
	{0x0F12, 0x00B8,},/* Tune_wbt_GAS_23_ */
	{0x0F12, 0x00F9,},/* Tune_wbt_GAS_24_ */
	{0x0F12, 0x012C,},/* Tune_wbt_GAS_25_ */
	{0x0F12, 0x011A,},/* Tune_wbt_GAS_26_ */
	{0x0F12, 0x00DB,},/* Tune_wbt_GAS_27_ */
	{0x0F12, 0x0093,},/* Tune_wbt_GAS_28_ */
	{0x0F12, 0x005F,},/* Tune_wbt_GAS_29_ */
	{0x0F12, 0x003C,},/* Tune_wbt_GAS_30_ */
	{0x0F12, 0x0027,},/* Tune_wbt_GAS_31_ */
	{0x0F12, 0x0020,},/* Tune_wbt_GAS_32_ */
	{0x0F12, 0x0026,},/* Tune_wbt_GAS_33_ */
	{0x0F12, 0x003A,},/* Tune_wbt_GAS_34_ */
	{0x0F12, 0x005C,},/* Tune_wbt_GAS_35_ */
	{0x0F12, 0x008E,},/* Tune_wbt_GAS_36_ */
	{0x0F12, 0x00D2,},/* Tune_wbt_GAS_37_ */
	{0x0F12, 0x010E,},/* Tune_wbt_GAS_38_ */
	{0x0F12, 0x0101,},/* Tune_wbt_GAS_39_ */
	{0x0F12, 0x00BF,},/* Tune_wbt_GAS_40_ */
	{0x0F12, 0x0077,},/* Tune_wbt_GAS_41_ */
	{0x0F12, 0x0044,},/* Tune_wbt_GAS_42_ */
	{0x0F12, 0x0023,},/* Tune_wbt_GAS_43_ */
	{0x0F12, 0x0011,},/* Tune_wbt_GAS_44_ */
	{0x0F12, 0x000C,},/* Tune_wbt_GAS_45_ */
	{0x0F12, 0x0010,},/* Tune_wbt_GAS_46_ */
	{0x0F12, 0x0022,},/* Tune_wbt_GAS_47_ */
	{0x0F12, 0x0043,},/* Tune_wbt_GAS_48_ */
	{0x0F12, 0x0074,},/* Tune_wbt_GAS_49_ */
	{0x0F12, 0x00B7,},/* Tune_wbt_GAS_50_ */
	{0x0F12, 0x00F7,},/* Tune_wbt_GAS_51_ */
	{0x0F12, 0x00FC,},/* Tune_wbt_GAS_52_ */
	{0x0F12, 0x00B7,},/* Tune_wbt_GAS_53_ */
	{0x0F12, 0x006F,},/* Tune_wbt_GAS_54_ */
	{0x0F12, 0x003C,},/* Tune_wbt_GAS_55_ */
	{0x0F12, 0x001C,},/* Tune_wbt_GAS_56_ */
	{0x0F12, 0x000A,},/* Tune_wbt_GAS_57_ */
	{0x0F12, 0x0004,},/* Tune_wbt_GAS_58_ */
	{0x0F12, 0x000A,},/* Tune_wbt_GAS_59_ */
	{0x0F12, 0x001B,},/* Tune_wbt_GAS_60_ */
	{0x0F12, 0x003B,},/* Tune_wbt_GAS_61_ */
	{0x0F12, 0x006C,},/* Tune_wbt_GAS_62_ */
	{0x0F12, 0x00B0,},/* Tune_wbt_GAS_63_ */
	{0x0F12, 0x00F2,},/* Tune_wbt_GAS_64_ */
	{0x0F12, 0x00EF,},/* Tune_wbt_GAS_65_ */
	{0x0F12, 0x00AB,},/* Tune_wbt_GAS_66_ */
	{0x0F12, 0x0065,},/* Tune_wbt_GAS_67_ */
	{0x0F12, 0x0034,},/* Tune_wbt_GAS_68_ */
	{0x0F12, 0x0015,},/* Tune_wbt_GAS_69_ */
	{0x0F12, 0x0004,},/* Tune_wbt_GAS_70_ */
	{0x0F12, 0x0000,},/* Tune_wbt_GAS_71_ */
	{0x0F12, 0x0004,},/* Tune_wbt_GAS_72_ */
	{0x0F12, 0x0013,},/* Tune_wbt_GAS_73_ */
	{0x0F12, 0x0033,},/* Tune_wbt_GAS_74_ */
	{0x0F12, 0x0063,},/* Tune_wbt_GAS_75_ */
	{0x0F12, 0x00A5,},/* Tune_wbt_GAS_76_ */
	{0x0F12, 0x00E5,},/* Tune_wbt_GAS_77_ */
	{0x0F12, 0x00F7,},/* Tune_wbt_GAS_78_ */
	{0x0F12, 0x00B4,},/* Tune_wbt_GAS_79_ */
	{0x0F12, 0x006D,},/* Tune_wbt_GAS_80_ */
	{0x0F12, 0x003C,},/* Tune_wbt_GAS_81_ */
	{0x0F12, 0x001C,},/* Tune_wbt_GAS_82_ */
	{0x0F12, 0x000B,},/* Tune_wbt_GAS_83_ */
	{0x0F12, 0x0005,},/* Tune_wbt_GAS_84_ */
	{0x0F12, 0x000A,},/* Tune_wbt_GAS_85_ */
	{0x0F12, 0x001B,},/* Tune_wbt_GAS_86_ */
	{0x0F12, 0x003B,},/* Tune_wbt_GAS_87_ */
	{0x0F12, 0x006B,},/* Tune_wbt_GAS_88_ */
	{0x0F12, 0x00AD,},/* Tune_wbt_GAS_89_ */
	{0x0F12, 0x00ED,},/* Tune_wbt_GAS_90_ */
	{0x0F12, 0x010B,},/* Tune_wbt_GAS_91_ */
	{0x0F12, 0x00CB,},/* Tune_wbt_GAS_92_ */
	{0x0F12, 0x0085,},/* Tune_wbt_GAS_93_ */
	{0x0F12, 0x0051,},/* Tune_wbt_GAS_94_ */
	{0x0F12, 0x002F,},/* Tune_wbt_GAS_95_ */
	{0x0F12, 0x001C,},/* Tune_wbt_GAS_96_ */
	{0x0F12, 0x0016,},/* Tune_wbt_GAS_97_ */
	{0x0F12, 0x001C,},/* Tune_wbt_GAS_98_ */
	{0x0F12, 0x002E,},/* Tune_wbt_GAS_99_ */
	{0x0F12, 0x004F,},/* Tune_wbt_GAS_100_ */
	{0x0F12, 0x0081,},/* Tune_wbt_GAS_101_ */
	{0x0F12, 0x00C4,},/* Tune_wbt_GAS_102_ */
	{0x0F12, 0x0102,},/* Tune_wbt_GAS_103_ */
	{0x0F12, 0x0119,},/* Tune_wbt_GAS_104_ */
	{0x0F12, 0x00DF,},/* Tune_wbt_GAS_105_ */
	{0x0F12, 0x009B,},/* Tune_wbt_GAS_106_ */
	{0x0F12, 0x0067,},/* Tune_wbt_GAS_107_ */
	{0x0F12, 0x0045,},/* Tune_wbt_GAS_108_ */
	{0x0F12, 0x0030,},/* Tune_wbt_GAS_109_ */
	{0x0F12, 0x0029,},/* Tune_wbt_GAS_110_ */
	{0x0F12, 0x002F,},/* Tune_wbt_GAS_111_ */
	{0x0F12, 0x0043,},/* Tune_wbt_GAS_112_ */
	{0x0F12, 0x0066,},/* Tune_wbt_GAS_113_ */
	{0x0F12, 0x0098,},/* Tune_wbt_GAS_114_ */
	{0x0F12, 0x00D9,},/* Tune_wbt_GAS_115_ */
	{0x0F12, 0x010F,},/* Tune_wbt_GAS_116_ */
	{0x0F12, 0x0138,},/* Tune_wbt_GAS_117_ */
	{0x0F12, 0x010C,},/* Tune_wbt_GAS_118_ */
	{0x0F12, 0x00CB,},/* Tune_wbt_GAS_119_ */
	{0x0F12, 0x0097,},/* Tune_wbt_GAS_120_ */
	{0x0F12, 0x0073,},/* Tune_wbt_GAS_121_ */
	{0x0F12, 0x005C,},/* Tune_wbt_GAS_122_ */
	{0x0F12, 0x0054,},/* Tune_wbt_GAS_123_ */
	{0x0F12, 0x005B,},/* Tune_wbt_GAS_124_ */
	{0x0F12, 0x0070,},/* Tune_wbt_GAS_125_ */
	{0x0F12, 0x0096,},/* Tune_wbt_GAS_126_ */
	{0x0F12, 0x00C9,},/* Tune_wbt_GAS_127_ */
	{0x0F12, 0x0106,},/* Tune_wbt_GAS_128_ */
	{0x0F12, 0x012D,},/* Tune_wbt_GAS_129_ */
	{0x0F12, 0x0147,},/* Tune_wbt_GAS_130_ */
	{0x0F12, 0x012F,},/* Tune_wbt_GAS_131_ */
	{0x0F12, 0x00F8,},/* Tune_wbt_GAS_132_ */
	{0x0F12, 0x00C5,},/* Tune_wbt_GAS_133_ */
	{0x0F12, 0x00A1,},/* Tune_wbt_GAS_134_ */
	{0x0F12, 0x008B,},/* Tune_wbt_GAS_135_ */
	{0x0F12, 0x0083,},/* Tune_wbt_GAS_136_ */
	{0x0F12, 0x008B,},/* Tune_wbt_GAS_137_ */
	{0x0F12, 0x00A0,},/* Tune_wbt_GAS_138_ */
	{0x0F12, 0x00C2,},/* Tune_wbt_GAS_139_ */
	{0x0F12, 0x00F3,},/* Tune_wbt_GAS_140_ */
	{0x0F12, 0x0124,},/* Tune_wbt_GAS_141_ */
	{0x0F12, 0x0139,},/* Tune_wbt_GAS_142_ */
	{0x0F12, 0x0093,},/* Tune_wbt_GAS_143_ */
	{0x0F12, 0x007E,},/* Tune_wbt_GAS_144_ */
	{0x0F12, 0x0062,},/* Tune_wbt_GAS_145_ */
	{0x0F12, 0x004D,},/* Tune_wbt_GAS_146_ */
	{0x0F12, 0x003E,},/* Tune_wbt_GAS_147_ */
	{0x0F12, 0x0034,},/* Tune_wbt_GAS_148_ */
	{0x0F12, 0x0030,},/* Tune_wbt_GAS_149_ */
	{0x0F12, 0x0032,},/* Tune_wbt_GAS_150_ */
	{0x0F12, 0x003B,},/* Tune_wbt_GAS_151_ */
	{0x0F12, 0x0049,},/* Tune_wbt_GAS_152_ */
	{0x0F12, 0x005C,},/* Tune_wbt_GAS_153_ */
	{0x0F12, 0x0077,},/* Tune_wbt_GAS_154_ */
	{0x0F12, 0x008A,},/* Tune_wbt_GAS_155_ */
	{0x0F12, 0x0093,},/* Tune_wbt_GAS_156_ */
	{0x0F12, 0x0077,},/* Tune_wbt_GAS_157_ */
	{0x0F12, 0x0059,},/* Tune_wbt_GAS_158_ */
	{0x0F12, 0x0042,},/* Tune_wbt_GAS_159_ */
	{0x0F12, 0x0032,},/* Tune_wbt_GAS_160_ */
	{0x0F12, 0x0027,},/* Tune_wbt_GAS_161_ */
	{0x0F12, 0x0024,},/* Tune_wbt_GAS_162_ */
	{0x0F12, 0x0026,},/* Tune_wbt_GAS_163_ */
	{0x0F12, 0x002F,},/* Tune_wbt_GAS_164_ */
	{0x0F12, 0x003D,},/* Tune_wbt_GAS_165_ */
	{0x0F12, 0x0052,},/* Tune_wbt_GAS_166_ */
	{0x0F12, 0x006E,},/* Tune_wbt_GAS_167_ */
	{0x0F12, 0x008B,},/* Tune_wbt_GAS_168_ */
	{0x0F12, 0x0083,},/* Tune_wbt_GAS_169_ */
	{0x0F12, 0x0064,},/* Tune_wbt_GAS_170_ */
	{0x0F12, 0x0046,},/* Tune_wbt_GAS_171_ */
	{0x0F12, 0x0030,},/* Tune_wbt_GAS_172_ */
	{0x0F12, 0x0020,},/* Tune_wbt_GAS_173_ */
	{0x0F12, 0x0016,},/* Tune_wbt_GAS_174_ */
	{0x0F12, 0x0011,},/* Tune_wbt_GAS_175_ */
	{0x0F12, 0x0014,},/* Tune_wbt_GAS_176_ */
	{0x0F12, 0x001E,},/* Tune_wbt_GAS_177_ */
	{0x0F12, 0x002D,},/* Tune_wbt_GAS_178_ */
	{0x0F12, 0x0041,},/* Tune_wbt_GAS_179_ */
	{0x0F12, 0x005D,},/* Tune_wbt_GAS_180_ */
	{0x0F12, 0x007C,},/* Tune_wbt_GAS_181_ */
	{0x0F12, 0x0077,},/* Tune_wbt_GAS_182_ */
	{0x0F12, 0x0057,},/* Tune_wbt_GAS_183_ */
	{0x0F12, 0x0039,},/* Tune_wbt_GAS_184_ */
	{0x0F12, 0x0024,},/* Tune_wbt_GAS_185_ */
	{0x0F12, 0x0014,},/* Tune_wbt_GAS_186_ */
	{0x0F12, 0x000A,},/* Tune_wbt_GAS_187_ */
	{0x0F12, 0x0007,},/* Tune_wbt_GAS_188_ */
	{0x0F12, 0x0009,},/* Tune_wbt_GAS_189_ */
	{0x0F12, 0x0012,},/* Tune_wbt_GAS_190_ */
	{0x0F12, 0x0021,},/* Tune_wbt_GAS_191_ */
	{0x0F12, 0x0036,},/* Tune_wbt_GAS_192_ */
	{0x0F12, 0x0051,},/* Tune_wbt_GAS_193_ */
	{0x0F12, 0x0070,},/* Tune_wbt_GAS_194_ */
	{0x0F12, 0x0077,},/* Tune_wbt_GAS_195_ */
	{0x0F12, 0x0056,},/* Tune_wbt_GAS_196_ */
	{0x0F12, 0x0038,},/* Tune_wbt_GAS_197_ */
	{0x0F12, 0x0022,},/* Tune_wbt_GAS_198_ */
	{0x0F12, 0x0013,},/* Tune_wbt_GAS_199_ */
	{0x0F12, 0x0009,},/* Tune_wbt_GAS_200_ */
	{0x0F12, 0x0005,},/* Tune_wbt_GAS_201_ */
	{0x0F12, 0x0008,},/* Tune_wbt_GAS_202_ */
	{0x0F12, 0x0011,},/* Tune_wbt_GAS_203_ */
	{0x0F12, 0x0020,},/* Tune_wbt_GAS_204_ */
	{0x0F12, 0x0035,},/* Tune_wbt_GAS_205_ */
	{0x0F12, 0x0051,},/* Tune_wbt_GAS_206_ */
	{0x0F12, 0x0071,},/* Tune_wbt_GAS_207_ */
	{0x0F12, 0x006E,},/* Tune_wbt_GAS_208_ */
	{0x0F12, 0x004E,},/* Tune_wbt_GAS_209_ */
	{0x0F12, 0x0032,},/* Tune_wbt_GAS_210_ */
	{0x0F12, 0x001C,},/* Tune_wbt_GAS_211_ */
	{0x0F12, 0x000D,},/* Tune_wbt_GAS_212_ */
	{0x0F12, 0x0004,},/* Tune_wbt_GAS_213_ */
	{0x0F12, 0x0000,},/* Tune_wbt_GAS_214_ */
	{0x0F12, 0x0003,},/* Tune_wbt_GAS_215_ */
	{0x0F12, 0x000B,},/* Tune_wbt_GAS_216_ */
	{0x0F12, 0x001A,},/* Tune_wbt_GAS_217_ */
	{0x0F12, 0x002F,},/* Tune_wbt_GAS_218_ */
	{0x0F12, 0x0049,},/* Tune_wbt_GAS_219_ */
	{0x0F12, 0x0068,},/* Tune_wbt_GAS_220_ */
	{0x0F12, 0x0072,},/* Tune_wbt_GAS_221_ */
	{0x0F12, 0x0053,},/* Tune_wbt_GAS_222_ */
	{0x0F12, 0x0037,},/* Tune_wbt_GAS_223_ */
	{0x0F12, 0x0021,},/* Tune_wbt_GAS_224_ */
	{0x0F12, 0x0012,},/* Tune_wbt_GAS_225_ */
	{0x0F12, 0x0009,},/* Tune_wbt_GAS_226_ */
	{0x0F12, 0x0005,},/* Tune_wbt_GAS_227_ */
	{0x0F12, 0x0008,},/* Tune_wbt_GAS_228_ */
	{0x0F12, 0x0010,},/* Tune_wbt_GAS_229_ */
	{0x0F12, 0x001F,},/* Tune_wbt_GAS_230_ */
	{0x0F12, 0x0034,},/* Tune_wbt_GAS_231_ */
	{0x0F12, 0x004E,},/* Tune_wbt_GAS_232_ */
	{0x0F12, 0x006C,},/* Tune_wbt_GAS_233_ */
	{0x0F12, 0x007F,},/* Tune_wbt_GAS_234_ */
	{0x0F12, 0x0060,},/* Tune_wbt_GAS_235_ */
	{0x0F12, 0x0043,},/* Tune_wbt_GAS_236_ */
	{0x0F12, 0x002D,},/* Tune_wbt_GAS_237_ */
	{0x0F12, 0x001D,},/* Tune_wbt_GAS_238_ */
	{0x0F12, 0x0013,},/* Tune_wbt_GAS_239_ */
	{0x0F12, 0x0010,},/* Tune_wbt_GAS_240_ */
	{0x0F12, 0x0013,},/* Tune_wbt_GAS_241_ */
	{0x0F12, 0x001C,},/* Tune_wbt_GAS_242_ */
	{0x0F12, 0x002B,},/* Tune_wbt_GAS_243_ */
	{0x0F12, 0x0040,},/* Tune_wbt_GAS_244_ */
	{0x0F12, 0x005A,},/* Tune_wbt_GAS_245_ */
	{0x0F12, 0x0079,},/* Tune_wbt_GAS_246_ */
	{0x0F12, 0x0082,},/* Tune_wbt_GAS_247_ */
	{0x0F12, 0x0066,},/* Tune_wbt_GAS_248_ */
	{0x0F12, 0x0049,},/* Tune_wbt_GAS_249_ */
	{0x0F12, 0x0035,},/* Tune_wbt_GAS_250_ */
	{0x0F12, 0x0025,},/* Tune_wbt_GAS_251_ */
	{0x0F12, 0x001B,},/* Tune_wbt_GAS_252_ */
	{0x0F12, 0x0017,},/* Tune_wbt_GAS_253_ */
	{0x0F12, 0x0019,},/* Tune_wbt_GAS_254_ */
	{0x0F12, 0x0023,},/* Tune_wbt_GAS_255_ */
	{0x0F12, 0x0033,},/* Tune_wbt_GAS_256_ */
	{0x0F12, 0x0046,},/* Tune_wbt_GAS_257_ */
	{0x0F12, 0x0060,},/* Tune_wbt_GAS_258_ */
	{0x0F12, 0x007B,},/* Tune_wbt_GAS_259_ */
	{0x0F12, 0x0092,},/* Tune_wbt_GAS_260_ */
	{0x0F12, 0x007C,},/* Tune_wbt_GAS_261_ */
	{0x0F12, 0x0060,},/* Tune_wbt_GAS_262_ */
	{0x0F12, 0x004B,},/* Tune_wbt_GAS_263_ */
	{0x0F12, 0x003C,},/* Tune_wbt_GAS_264_ */
	{0x0F12, 0x0032,},/* Tune_wbt_GAS_265_ */
	{0x0F12, 0x002D,},/* Tune_wbt_GAS_266_ */
	{0x0F12, 0x0030,},/* Tune_wbt_GAS_267_ */
	{0x0F12, 0x0039,},/* Tune_wbt_GAS_268_ */
	{0x0F12, 0x0049,},/* Tune_wbt_GAS_269_ */
	{0x0F12, 0x005D,},/* Tune_wbt_GAS_270_ */
	{0x0F12, 0x0076,},/* Tune_wbt_GAS_271_ */
	{0x0F12, 0x008C,},/* Tune_wbt_GAS_272_ */
	{0x0F12, 0x009F,},/* Tune_wbt_GAS_273_ */
	{0x0F12, 0x008F,},/* Tune_wbt_GAS_274_ */
	{0x0F12, 0x0077,},/* Tune_wbt_GAS_275_ */
	{0x0F12, 0x0061,},/* Tune_wbt_GAS_276_ */
	{0x0F12, 0x0052,},/* Tune_wbt_GAS_277_ */
	{0x0F12, 0x0048,},/* Tune_wbt_GAS_278_ */
	{0x0F12, 0x0043,},/* Tune_wbt_GAS_279_ */
	{0x0F12, 0x0047,},/* Tune_wbt_GAS_280_ */
	{0x0F12, 0x0050,},/* Tune_wbt_GAS_281_ */
	{0x0F12, 0x005E,},/* Tune_wbt_GAS_282_ */
	{0x0F12, 0x0071,},/* Tune_wbt_GAS_283_ */
	{0x0F12, 0x0086,},/* Tune_wbt_GAS_284_ */
	{0x0F12, 0x0097,},/* Tune_wbt_GAS_285_ */
	{0x0F12, 0x0093,},/* Tune_wbt_GAS_286_ */
	{0x0F12, 0x007C,},/* Tune_wbt_GAS_287_ */
	{0x0F12, 0x005F,},/* Tune_wbt_GAS_288_ */
	{0x0F12, 0x0049,},/* Tune_wbt_GAS_289_ */
	{0x0F12, 0x003A,},/* Tune_wbt_GAS_290_ */
	{0x0F12, 0x0030,},/* Tune_wbt_GAS_291_ */
	{0x0F12, 0x002C,},/* Tune_wbt_GAS_292_ */
	{0x0F12, 0x002F,},/* Tune_wbt_GAS_293_ */
	{0x0F12, 0x0037,},/* Tune_wbt_GAS_294_ */
	{0x0F12, 0x0045,},/* Tune_wbt_GAS_295_ */
	{0x0F12, 0x005A,},/* Tune_wbt_GAS_296_ */
	{0x0F12, 0x0075,},/* Tune_wbt_GAS_297_ */
	{0x0F12, 0x008A,},/* Tune_wbt_GAS_298_ */
	{0x0F12, 0x0094,},/* Tune_wbt_GAS_299_ */
	{0x0F12, 0x0077,},/* Tune_wbt_GAS_300_ */
	{0x0F12, 0x0057,},/* Tune_wbt_GAS_301_ */
	{0x0F12, 0x0040,},/* Tune_wbt_GAS_302_ */
	{0x0F12, 0x002F,},/* Tune_wbt_GAS_303_ */
	{0x0F12, 0x0024,},/* Tune_wbt_GAS_304_ */
	{0x0F12, 0x0020,},/* Tune_wbt_GAS_305_ */
	{0x0F12, 0x0023,},/* Tune_wbt_GAS_306_ */
	{0x0F12, 0x002D,},/* Tune_wbt_GAS_307_ */
	{0x0F12, 0x003B,},/* Tune_wbt_GAS_308_ */
	{0x0F12, 0x0051,},/* Tune_wbt_GAS_309_ */
	{0x0F12, 0x006E,},/* Tune_wbt_GAS_310_ */
	{0x0F12, 0x008C,},/* Tune_wbt_GAS_311_ */
	{0x0F12, 0x0085,},/* Tune_wbt_GAS_312_ */
	{0x0F12, 0x0066,},/* Tune_wbt_GAS_313_ */
	{0x0F12, 0x0046,},/* Tune_wbt_GAS_314_ */
	{0x0F12, 0x002F,},/* Tune_wbt_GAS_315_ */
	{0x0F12, 0x001F,},/* Tune_wbt_GAS_316_ */
	{0x0F12, 0x0014,},/* Tune_wbt_GAS_317_ */
	{0x0F12, 0x000F,},/* Tune_wbt_GAS_318_ */
	{0x0F12, 0x0012,},/* Tune_wbt_GAS_319_ */
	{0x0F12, 0x001C,},/* Tune_wbt_GAS_320_ */
	{0x0F12, 0x002B,},/* Tune_wbt_GAS_321_ */
	{0x0F12, 0x0040,},/* Tune_wbt_GAS_322_ */
	{0x0F12, 0x005C,},/* Tune_wbt_GAS_323_ */
	{0x0F12, 0x007D,},/* Tune_wbt_GAS_324_ */
	{0x0F12, 0x007A,},/* Tune_wbt_GAS_325_ */
	{0x0F12, 0x005A,},/* Tune_wbt_GAS_326_ */
	{0x0F12, 0x003A,},/* Tune_wbt_GAS_327_ */
	{0x0F12, 0x0024,},/* Tune_wbt_GAS_328_ */
	{0x0F12, 0x0014,},/* Tune_wbt_GAS_329_ */
	{0x0F12, 0x0009,},/* Tune_wbt_GAS_330_ */
	{0x0F12, 0x0006,},/* Tune_wbt_GAS_331_ */
	{0x0F12, 0x0008,},/* Tune_wbt_GAS_332_ */
	{0x0F12, 0x0011,},/* Tune_wbt_GAS_333_ */
	{0x0F12, 0x0020,},/* Tune_wbt_GAS_334_ */
	{0x0F12, 0x0036,},/* Tune_wbt_GAS_335_ */
	{0x0F12, 0x0051,},/* Tune_wbt_GAS_336_ */
	{0x0F12, 0x0072,},/* Tune_wbt_GAS_337_ */
	{0x0F12, 0x007B,},/* Tune_wbt_GAS_338_ */
	{0x0F12, 0x0059,},/* Tune_wbt_GAS_339_ */
	{0x0F12, 0x003A,},/* Tune_wbt_GAS_340_ */
	{0x0F12, 0x0023,},/* Tune_wbt_GAS_341_ */
	{0x0F12, 0x0012,},/* Tune_wbt_GAS_342_ */
	{0x0F12, 0x0008,},/* Tune_wbt_GAS_343_ */
	{0x0F12, 0x0004,},/* Tune_wbt_GAS_344_ */
	{0x0F12, 0x0007,},/* Tune_wbt_GAS_345_ */
	{0x0F12, 0x000F,},/* Tune_wbt_GAS_346_ */
	{0x0F12, 0x001F,},/* Tune_wbt_GAS_347_ */
	{0x0F12, 0x0035,},/* Tune_wbt_GAS_348_ */
	{0x0F12, 0x0051,},/* Tune_wbt_GAS_349_ */
	{0x0F12, 0x0072,},/* Tune_wbt_GAS_350_ */
	{0x0F12, 0x0073,},/* Tune_wbt_GAS_351_ */
	{0x0F12, 0x0053,},/* Tune_wbt_GAS_352_ */
	{0x0F12, 0x0034,},/* Tune_wbt_GAS_353_ */
	{0x0F12, 0x001D,},/* Tune_wbt_GAS_354_ */
	{0x0F12, 0x000E,},/* Tune_wbt_GAS_355_ */
	{0x0F12, 0x0004,},/* Tune_wbt_GAS_356_ */
	{0x0F12, 0x0000,},/* Tune_wbt_GAS_357_ */
	{0x0F12, 0x0002,},/* Tune_wbt_GAS_358_ */
	{0x0F12, 0x000A,},/* Tune_wbt_GAS_359_ */
	{0x0F12, 0x001A,},/* Tune_wbt_GAS_360_ */
	{0x0F12, 0x002F,},/* Tune_wbt_GAS_361_ */
	{0x0F12, 0x004A,},/* Tune_wbt_GAS_362_ */
	{0x0F12, 0x006A,},/* Tune_wbt_GAS_363_ */
	{0x0F12, 0x0077,},/* Tune_wbt_GAS_364_ */
	{0x0F12, 0x0058,},/* Tune_wbt_GAS_365_ */
	{0x0F12, 0x0039,},/* Tune_wbt_GAS_366_ */
	{0x0F12, 0x0022,},/* Tune_wbt_GAS_367_ */
	{0x0F12, 0x0012,},/* Tune_wbt_GAS_368_ */
	{0x0F12, 0x0008,},/* Tune_wbt_GAS_369_ */
	{0x0F12, 0x0004,},/* Tune_wbt_GAS_370_ */
	{0x0F12, 0x0007,},/* Tune_wbt_GAS_371_ */
	{0x0F12, 0x000F,},/* Tune_wbt_GAS_372_ */
	{0x0F12, 0x001E,},/* Tune_wbt_GAS_373_ */
	{0x0F12, 0x0034,},/* Tune_wbt_GAS_374_ */
	{0x0F12, 0x004F,},/* Tune_wbt_GAS_375_ */
	{0x0F12, 0x006F,},/* Tune_wbt_GAS_376_ */
	{0x0F12, 0x0083,},/* Tune_wbt_GAS_377_ */
	{0x0F12, 0x0064,},/* Tune_wbt_GAS_378_ */
	{0x0F12, 0x0045,},/* Tune_wbt_GAS_379_ */
	{0x0F12, 0x002E,},/* Tune_wbt_GAS_380_ */
	{0x0F12, 0x001D,},/* Tune_wbt_GAS_381_ */
	{0x0F12, 0x0012,},/* Tune_wbt_GAS_382_ */
	{0x0F12, 0x000F,},/* Tune_wbt_GAS_383_ */
	{0x0F12, 0x0011,},/* Tune_wbt_GAS_384_ */
	{0x0F12, 0x001A,},/* Tune_wbt_GAS_385_ */
	{0x0F12, 0x002A,},/* Tune_wbt_GAS_386_ */
	{0x0F12, 0x003F,},/* Tune_wbt_GAS_387_ */
	{0x0F12, 0x005B,},/* Tune_wbt_GAS_388_ */
	{0x0F12, 0x007B,},/* Tune_wbt_GAS_389_ */
	{0x0F12, 0x0087,},/* Tune_wbt_GAS_390_ */
	{0x0F12, 0x006A,},/* Tune_wbt_GAS_391_ */
	{0x0F12, 0x004B,},/* Tune_wbt_GAS_392_ */
	{0x0F12, 0x0036,},/* Tune_wbt_GAS_393_ */
	{0x0F12, 0x0025,},/* Tune_wbt_GAS_394_ */
	{0x0F12, 0x0019,},/* Tune_wbt_GAS_395_ */
	{0x0F12, 0x0015,},/* Tune_wbt_GAS_396_ */
	{0x0F12, 0x0017,},/* Tune_wbt_GAS_397_ */
	{0x0F12, 0x0022,},/* Tune_wbt_GAS_398_ */
	{0x0F12, 0x0031,},/* Tune_wbt_GAS_399_ */
	{0x0F12, 0x0045,},/* Tune_wbt_GAS_400_ */
	{0x0F12, 0x0060,},/* Tune_wbt_GAS_401_ */
	{0x0F12, 0x007D,},/* Tune_wbt_GAS_402_ */
	{0x0F12, 0x0096,},/* Tune_wbt_GAS_403_ */
	{0x0F12, 0x007F,},/* Tune_wbt_GAS_404_ */
	{0x0F12, 0x0061,},/* Tune_wbt_GAS_405_ */
	{0x0F12, 0x004B,},/* Tune_wbt_GAS_406_ */
	{0x0F12, 0x003B,},/* Tune_wbt_GAS_407_ */
	{0x0F12, 0x002F,},/* Tune_wbt_GAS_408_ */
	{0x0F12, 0x002A,},/* Tune_wbt_GAS_409_ */
	{0x0F12, 0x002D,},/* Tune_wbt_GAS_410_ */
	{0x0F12, 0x0036,},/* Tune_wbt_GAS_411_ */
	{0x0F12, 0x0046,},/* Tune_wbt_GAS_412_ */
	{0x0F12, 0x005B,},/* Tune_wbt_GAS_413_ */
	{0x0F12, 0x0075,},/* Tune_wbt_GAS_414_ */
	{0x0F12, 0x008D,},/* Tune_wbt_GAS_415_ */
	{0x0F12, 0x00A1,},/* Tune_wbt_GAS_416_ */
	{0x0F12, 0x0091,},/* Tune_wbt_GAS_417_ */
	{0x0F12, 0x0077,},/* Tune_wbt_GAS_418_ */
	{0x0F12, 0x0060,},/* Tune_wbt_GAS_419_ */
	{0x0F12, 0x0050,},/* Tune_wbt_GAS_420_ */
	{0x0F12, 0x0044,},/* Tune_wbt_GAS_421_ */
	{0x0F12, 0x0040,},/* Tune_wbt_GAS_422_ */
	{0x0F12, 0x0043,},/* Tune_wbt_GAS_423_ */
	{0x0F12, 0x004C,},/* Tune_wbt_GAS_424_ */
	{0x0F12, 0x005A,},/* Tune_wbt_GAS_425_ */
	{0x0F12, 0x006D,},/* Tune_wbt_GAS_426_ */
	{0x0F12, 0x0084,},/* Tune_wbt_GAS_427_ */
	{0x0F12, 0x0094,},/* Tune_wbt_GAS_428_ */
	{0x0F12, 0x0072,},/* Tune_wbt_GAS_429_ */
	{0x0F12, 0x0063,},/* Tune_wbt_GAS_430_ */
	{0x0F12, 0x004C,},/* Tune_wbt_GAS_431_ */
	{0x0F12, 0x003A,},/* Tune_wbt_GAS_432_ */
	{0x0F12, 0x002D,},/* Tune_wbt_GAS_433_ */
	{0x0F12, 0x0025,},/* Tune_wbt_GAS_434_ */
	{0x0F12, 0x0023,},/* Tune_wbt_GAS_435_ */
	{0x0F12, 0x0025,},/* Tune_wbt_GAS_436_ */
	{0x0F12, 0x002C,},/* Tune_wbt_GAS_437_ */
	{0x0F12, 0x0038,},/* Tune_wbt_GAS_438_ */
	{0x0F12, 0x004A,},/* Tune_wbt_GAS_439_ */
	{0x0F12, 0x005F,},/* Tune_wbt_GAS_440_ */
	{0x0F12, 0x006B,},/* Tune_wbt_GAS_441_ */
	{0x0F12, 0x0079,},/* Tune_wbt_GAS_442_ */
	{0x0F12, 0x0065,},/* Tune_wbt_GAS_443_ */
	{0x0F12, 0x004A,},/* Tune_wbt_GAS_444_ */
	{0x0F12, 0x0037,},/* Tune_wbt_GAS_445_ */
	{0x0F12, 0x0029,},/* Tune_wbt_GAS_446_ */
	{0x0F12, 0x0021,},/* Tune_wbt_GAS_447_ */
	{0x0F12, 0x001D,},/* Tune_wbt_GAS_448_ */
	{0x0F12, 0x001F,},/* Tune_wbt_GAS_449_ */
	{0x0F12, 0x0027,},/* Tune_wbt_GAS_450_ */
	{0x0F12, 0x0033,},/* Tune_wbt_GAS_451_ */
	{0x0F12, 0x0044,},/* Tune_wbt_GAS_452_ */
	{0x0F12, 0x005E,},/* Tune_wbt_GAS_453_ */
	{0x0F12, 0x006E,},/* Tune_wbt_GAS_454_ */
	{0x0F12, 0x006A,},/* Tune_wbt_GAS_455_ */
	{0x0F12, 0x0055,},/* Tune_wbt_GAS_456_ */
	{0x0F12, 0x003A,},/* Tune_wbt_GAS_457_ */
	{0x0F12, 0x0028,},/* Tune_wbt_GAS_458_ */
	{0x0F12, 0x001A,},/* Tune_wbt_GAS_459_ */
	{0x0F12, 0x0011,},/* Tune_wbt_GAS_460_ */
	{0x0F12, 0x000D,},/* Tune_wbt_GAS_461_ */
	{0x0F12, 0x000F,},/* Tune_wbt_GAS_462_ */
	{0x0F12, 0x0017,},/* Tune_wbt_GAS_463_ */
	{0x0F12, 0x0024,},/* Tune_wbt_GAS_464_ */
	{0x0F12, 0x0035,},/* Tune_wbt_GAS_465_ */
	{0x0F12, 0x004E,},/* Tune_wbt_GAS_466_ */
	{0x0F12, 0x0061,},/* Tune_wbt_GAS_467_ */
	{0x0F12, 0x0061,},/* Tune_wbt_GAS_468_ */
	{0x0F12, 0x004A,},/* Tune_wbt_GAS_469_ */
	{0x0F12, 0x0031,},/* Tune_wbt_GAS_470_ */
	{0x0F12, 0x001E,},/* Tune_wbt_GAS_471_ */
	{0x0F12, 0x0011,},/* Tune_wbt_GAS_472_ */
	{0x0F12, 0x0008,},/* Tune_wbt_GAS_473_ */
	{0x0F12, 0x0005,},/* Tune_wbt_GAS_474_ */
	{0x0F12, 0x0007,},/* Tune_wbt_GAS_475_ */
	{0x0F12, 0x000E,},/* Tune_wbt_GAS_476_ */
	{0x0F12, 0x001B,},/* Tune_wbt_GAS_477_ */
	{0x0F12, 0x002D,},/* Tune_wbt_GAS_478_ */
	{0x0F12, 0x0045,},/* Tune_wbt_GAS_479_ */
	{0x0F12, 0x0059,},/* Tune_wbt_GAS_480_ */
	{0x0F12, 0x0062,},/* Tune_wbt_GAS_481_ */
	{0x0F12, 0x004B,},/* Tune_wbt_GAS_482_ */
	{0x0F12, 0x0031,},/* Tune_wbt_GAS_483_ */
	{0x0F12, 0x001E,},/* Tune_wbt_GAS_484_ */
	{0x0F12, 0x0010,},/* Tune_wbt_GAS_485_ */
	{0x0F12, 0x0008,},/* Tune_wbt_GAS_486_ */
	{0x0F12, 0x0004,},/* Tune_wbt_GAS_487_ */
	{0x0F12, 0x0006,},/* Tune_wbt_GAS_488_ */
	{0x0F12, 0x000E,},/* Tune_wbt_GAS_489_ */
	{0x0F12, 0x001B,},/* Tune_wbt_GAS_490_ */
	{0x0F12, 0x002E,},/* Tune_wbt_GAS_491_ */
	{0x0F12, 0x0046,},/* Tune_wbt_GAS_492_ */
	{0x0F12, 0x005A,},/* Tune_wbt_GAS_493_ */
	{0x0F12, 0x005B,},/* Tune_wbt_GAS_494_ */
	{0x0F12, 0x0045,},/* Tune_wbt_GAS_495_ */
	{0x0F12, 0x002C,},/* Tune_wbt_GAS_496_ */
	{0x0F12, 0x001A,},/* Tune_wbt_GAS_497_ */
	{0x0F12, 0x000C,},/* Tune_wbt_GAS_498_ */
	{0x0F12, 0x0003,},/* Tune_wbt_GAS_499_ */
	{0x0F12, 0x0000,},/* Tune_wbt_GAS_500_ */
	{0x0F12, 0x0002,},/* Tune_wbt_GAS_501_ */
	{0x0F12, 0x0009,},/* Tune_wbt_GAS_502_ */
	{0x0F12, 0x0016,},/* Tune_wbt_GAS_503_ */
	{0x0F12, 0x0029,},/* Tune_wbt_GAS_504_ */
	{0x0F12, 0x0040,},/* Tune_wbt_GAS_505_ */
	{0x0F12, 0x0054,},/* Tune_wbt_GAS_506_ */
	{0x0F12, 0x005F,},/* Tune_wbt_GAS_507_ */
	{0x0F12, 0x004A,},/* Tune_wbt_GAS_508_ */
	{0x0F12, 0x0031,},/* Tune_wbt_GAS_509_ */
	{0x0F12, 0x001F,},/* Tune_wbt_GAS_510_ */
	{0x0F12, 0x0010,},/* Tune_wbt_GAS_511_ */
	{0x0F12, 0x0008,},/* Tune_wbt_GAS_512_ */
	{0x0F12, 0x0004,},/* Tune_wbt_GAS_513_ */
	{0x0F12, 0x0007,},/* Tune_wbt_GAS_514_ */
	{0x0F12, 0x000E,},/* Tune_wbt_GAS_515_ */
	{0x0F12, 0x001B,},/* Tune_wbt_GAS_516_ */
	{0x0F12, 0x002E,},/* Tune_wbt_GAS_517_ */
	{0x0F12, 0x0045,},/* Tune_wbt_GAS_518_ */
	{0x0F12, 0x0059,},/* Tune_wbt_GAS_519_ */
	{0x0F12, 0x006C,},/* Tune_wbt_GAS_520_ */
	{0x0F12, 0x0057,},/* Tune_wbt_GAS_521_ */
	{0x0F12, 0x003E,},/* Tune_wbt_GAS_522_ */
	{0x0F12, 0x002A,},/* Tune_wbt_GAS_523_ */
	{0x0F12, 0x001B,},/* Tune_wbt_GAS_524_ */
	{0x0F12, 0x0012,},/* Tune_wbt_GAS_525_ */
	{0x0F12, 0x000F,},/* Tune_wbt_GAS_526_ */
	{0x0F12, 0x0011,},/* Tune_wbt_GAS_527_ */
	{0x0F12, 0x0019,},/* Tune_wbt_GAS_528_ */
	{0x0F12, 0x0027,},/* Tune_wbt_GAS_529_ */
	{0x0F12, 0x0039,},/* Tune_wbt_GAS_530_ */
	{0x0F12, 0x0050,},/* Tune_wbt_GAS_531_ */
	{0x0F12, 0x0063,},/* Tune_wbt_GAS_532_ */
	{0x0F12, 0x006F,},/* Tune_wbt_GAS_533_ */
	{0x0F12, 0x005C,},/* Tune_wbt_GAS_534_ */
	{0x0F12, 0x0044,},/* Tune_wbt_GAS_535_ */
	{0x0F12, 0x0031,},/* Tune_wbt_GAS_536_ */
	{0x0F12, 0x0023,},/* Tune_wbt_GAS_537_ */
	{0x0F12, 0x0019,},/* Tune_wbt_GAS_538_ */
	{0x0F12, 0x0016,},/* Tune_wbt_GAS_539_ */
	{0x0F12, 0x0017,},/* Tune_wbt_GAS_540_ */
	{0x0F12, 0x0020,},/* Tune_wbt_GAS_541_ */
	{0x0F12, 0x002E,},/* Tune_wbt_GAS_542_ */
	{0x0F12, 0x0040,},/* Tune_wbt_GAS_543_ */
	{0x0F12, 0x0055,},/* Tune_wbt_GAS_544_ */
	{0x0F12, 0x0064,},/* Tune_wbt_GAS_545_ */
	{0x0F12, 0x007E,},/* Tune_wbt_GAS_546_ */
	{0x0F12, 0x0071,},/* Tune_wbt_GAS_547_ */
	{0x0F12, 0x0059,},/* Tune_wbt_GAS_548_ */
	{0x0F12, 0x0046,},/* Tune_wbt_GAS_549_ */
	{0x0F12, 0x0039,},/* Tune_wbt_GAS_550_ */
	{0x0F12, 0x002F,},/* Tune_wbt_GAS_551_ */
	{0x0F12, 0x002A,},/* Tune_wbt_GAS_552_ */
	{0x0F12, 0x002D,},/* Tune_wbt_GAS_553_ */
	{0x0F12, 0x0035,},/* Tune_wbt_GAS_554_ */
	{0x0F12, 0x0043,},/* Tune_wbt_GAS_555_ */
	{0x0F12, 0x0054,},/* Tune_wbt_GAS_556_ */
	{0x0F12, 0x0069,},/* Tune_wbt_GAS_557_ */
	{0x0F12, 0x0074,},/* Tune_wbt_GAS_558_ */
	{0x0F12, 0x0083,},/* Tune_wbt_GAS_559_ */
	{0x0F12, 0x007D,},/* Tune_wbt_GAS_560_ */
	{0x0F12, 0x0068,},/* Tune_wbt_GAS_561_ */
	{0x0F12, 0x0055,},/* Tune_wbt_GAS_562_ */
	{0x0F12, 0x0048,},/* Tune_wbt_GAS_563_ */
	{0x0F12, 0x003E,},/* Tune_wbt_GAS_564_ */
	{0x0F12, 0x003A,},/* Tune_wbt_GAS_565_ */
	{0x0F12, 0x003D,},/* Tune_wbt_GAS_566_ */
	{0x0F12, 0x0045,},/* Tune_wbt_GAS_567_ */
	{0x0F12, 0x0051,},/* Tune_wbt_GAS_568_ */
	{0x0F12, 0x0061,},/* Tune_wbt_GAS_569_ */
	{0x0F12, 0x0072,},/* Tune_wbt_GAS_570_ */
	{0x0F12, 0x0077,},/* Tune_wbt_GAS_571_ */
	{0x002A, 0x1348,},/* reserved */
	{0x0F12, 0x0001,},/* gisp_gras_Enable */
	{0x002A, 0x1278,},/* reserved */
	{0x0F12, 0xAAF0,},/* gisp_dadlc_config */
	{0x002A, 0x3370,},/* reserved */
	{0x0F12, 0x0000,},/* afit_bUseNormBrForAfit */
	{0x002A, 0x0D2A,},/* reserved */
	{0x0F12, 0x0450,},/* awbb_GainsInit_0 */
	{0x0F12, 0x0400,},/* awbb_GainsInit_1 */
	{0x0F12, 0x0900,},/* awbb_GainsInit_2 */
	{0x002A, 0x0B36,},/* reserved */
	{0x0F12, 0x0005,},/* awbb_IndoorGrZones_ZInfo_m_GridStep */
	{0x002A, 0x0B3A,},/* reserved */
	{0x0F12, 0x00B4,},/* awbb_IndoorGrZones_ZInfo_m_BMin */
	{0x0F12, 0x02C1,},/* awbb_IndoorGrZones_ZInfo_m_BMax */
	{0x002A, 0x0B38,},/* reserved */
	{0x0F12, 0x0012,},/* awbb_IndoorGrZones_ZInfo_m_GridSz */
	{0x002A, 0x0AE6,},/* reserved */
	{0x0F12, 0x03EF,},/* awbb_IndoorGrZones_m_BGrid_0__m_left */
	{0x0F12, 0x0434,},/* awbb_IndoorGrZones_m_BGrid_0__m_right */
	{0x0F12, 0x03A1,},/* awbb_IndoorGrZones_m_BGrid_1__m_left */
	{0x0F12, 0x0439,},/* awbb_IndoorGrZones_m_BGrid_1__m_right */
	{0x0F12, 0x0379,},/* awbb_IndoorGrZones_m_BGrid_2__m_left */
	{0x0F12, 0x043E,},/* awbb_IndoorGrZones_m_BGrid_2__m_right */
	{0x0F12, 0x0350,},/* awbb_IndoorGrZones_m_BGrid_3__m_left */
	{0x0F12, 0x0429,},/* awbb_IndoorGrZones_m_BGrid_3__m_right */
	{0x0F12, 0x0327,},/* awbb_IndoorGrZones_m_BGrid_4__m_left */
	{0x0F12, 0x03E5,},/* awbb_IndoorGrZones_m_BGrid_4__m_right */
	{0x0F12, 0x02F9,},/* awbb_IndoorGrZones_m_BGrid_5__m_left */
	{0x0F12, 0x03BE,},/* awbb_IndoorGrZones_m_BGrid_5__m_right */
	{0x0F12, 0x02D2,},/* awbb_IndoorGrZones_m_BGrid_6__m_left */
	{0x0F12, 0x036D,},/* awbb_IndoorGrZones_m_BGrid_6__m_right */
	{0x0F12, 0x02C1,},/* awbb_IndoorGrZones_m_BGrid_7__m_left */
	{0x0F12, 0x0324,},/* awbb_IndoorGrZones_m_BGrid_7__m_right */
	{0x0F12, 0x02AB,},/* awbb_IndoorGrZones_m_BGrid_8__m_left */
	{0x0F12, 0x02FF,},/* awbb_IndoorGrZones_m_BGrid_8__m_right */
	{0x0F12, 0x028C,},/* awbb_IndoorGrZones_m_BGrid_9__m_left */
	{0x0F12, 0x02F1,},/* awbb_IndoorGrZones_m_BGrid_9__m_right */
	{0x0F12, 0x0276,},/* awbb_IndoorGrZones_m_BGrid_10__m_left */
	{0x0F12, 0x02D7,},/* awbb_IndoorGrZones_m_BGrid_10__m_right */
	{0x0F12, 0x0261,},/* awbb_IndoorGrZones_m_BGrid_11__m_left */
	{0x0F12, 0x02C4,},/* awbb_IndoorGrZones_m_BGrid_11__m_right */
	{0x0F12, 0x023A,},/* awbb_IndoorGrZones_m_BGrid_12__m_left */
	{0x0F12, 0x02A5,},/* awbb_IndoorGrZones_m_BGrid_12__m_right */
	{0x0F12, 0x020C,},/* awbb_IndoorGrZones_m_BGrid_13__m_left */
	{0x0F12, 0x0288,},/* awbb_IndoorGrZones_m_BGrid_13__m_right */
	{0x0F12, 0x01E5,},/* awbb_IndoorGrZones_m_BGrid_14__m_left */
	{0x0F12, 0x026A,},/* awbb_IndoorGrZones_m_BGrid_14__m_right */
	{0x0F12, 0x01D8,},/* awbb_IndoorGrZones_m_BGrid_15__m_left */
	{0x0F12, 0x0250,},/* awbb_IndoorGrZones_m_BGrid_15__m_right */
	{0x0F12, 0x01DF,},/* awbb_IndoorGrZones_m_BGrid_16__m_left */
	{0x0F12, 0x022C,},/* awbb_IndoorGrZones_m_BGrid_16__m_right */
	{0x0F12, 0x0209,},/* awbb_IndoorGrZones_m_BGrid_17__m_left */
	{0x0F12, 0x01D3,},/* awbb_IndoorGrZones_m_BGrid_17__m_right */
	{0x0F12, 0x0000,},/* awbb_IndoorGrZones_m_BGrid_18__m_left */
	{0x0F12, 0x0000,},/* awbb_IndoorGrZones_m_BGrid_18__m_right */
	{0x0F12, 0x0000,},/* awbb_IndoorGrZones_m_BGrid_19__m_left */
	{0x0F12, 0x0000,},/* awbb_IndoorGrZones_m_BGrid_19__m_right */
	{0x002A, 0x0BAA,},/* reserved */
	{0x0F12, 0x0006,},/* awbb_LowBrGrZones_ZInfo_m_GridStep */
	{0x002A, 0x0BAE,},/* reserved */
	{0x0F12, 0x0095,},/* awbb_LowBrGrZones_ZInfo_m_BMin */
	{0x0F12, 0x02E0,},/* awbb_LowBrGrZones_ZInfo_m_BMax */
	{0x002A, 0x0BAC,},/* reserved */
	{0x0F12, 0x000B,},/* awbb_LowBrGrZones_ZInfo_m_GridSz */
	{0x002A, 0x0B7A,},/* reserved */
	{0x0F12, 0x0401,},/* awbb_LowBrGrZones_m_BGrid_0__m_left */
	{0x0F12, 0x0446,},/* awbb_LowBrGrZones_m_BGrid_0__m_right */
	{0x0F12, 0x037F,},/* awbb_LowBrGrZones_m_BGrid_1__m_left */
	{0x0F12, 0x044C,},/* awbb_LowBrGrZones_m_BGrid_1__m_right */
	{0x0F12, 0x033B,},/* awbb_LowBrGrZones_m_BGrid_2__m_left */
	{0x0F12, 0x0446,},/* awbb_LowBrGrZones_m_BGrid_2__m_right */
	{0x0F12, 0x02D7,},/* awbb_LowBrGrZones_m_BGrid_3__m_left */
	{0x0F12, 0x03FD,},/* awbb_LowBrGrZones_m_BGrid_3__m_right */
	{0x0F12, 0x02A6,},/* awbb_LowBrGrZones_m_BGrid_4__m_left */
	{0x0F12, 0x03C0,},/* awbb_LowBrGrZones_m_BGrid_4__m_right */
	{0x0F12, 0x026A,},/* awbb_LowBrGrZones_m_BGrid_5__m_left */
	{0x0F12, 0x0335,},/* awbb_LowBrGrZones_m_BGrid_5__m_right */
	{0x0F12, 0x0229,},/* awbb_LowBrGrZones_m_BGrid_6__m_left */
	{0x0F12, 0x0300,},/* awbb_LowBrGrZones_m_BGrid_6__m_right */
	{0x0F12, 0x01D6,},/* awbb_LowBrGrZones_m_BGrid_7__m_left */
	{0x0F12, 0x02C3,},/* awbb_LowBrGrZones_m_BGrid_7__m_right */
	{0x0F12, 0x01C6,},/* awbb_LowBrGrZones_m_BGrid_8__m_left */
	{0x0F12, 0x0270,},/* awbb_LowBrGrZones_m_BGrid_8__m_right */
	{0x0F12, 0x01D5,},/* awbb_LowBrGrZones_m_BGrid_9__m_left */
	{0x0F12, 0x0231,},/* awbb_LowBrGrZones_m_BGrid_9__m_right */
	{0x0F12, 0x029D,},/* awbb_LowBrGrZones_m_BGrid_10__m_left */
	{0x0F12, 0x016F,},/* awbb_LowBrGrZones_m_BGrid_10__m_right */
	{0x0F12, 0x0000,},/* awbb_LowBrGrZones_m_BGrid_11__m_left */
	{0x0F12, 0x0000,},/* awbb_LowBrGrZones_m_BGrid_11__m_right */
	{0x002A, 0x0B70,},/* reserved */
	{0x0F12, 0x0005,},/* awbb_OutdoorGrZones_ZInfo_m_GridStep */
	{0x002A, 0x0B74,},/* reserved */
	{0x0F12, 0x022B,},/* awbb_OutdoorGrZones_ZInfo_m_BMin */
	{0x0F12, 0x0294,},/* awbb_OutdoorGrZones_ZInfo_m_BMax */
	{0x002A, 0x0B72,},/* reserved */
	{0x0F12, 0x0005,},/* awbb_OutdoorGrZones_ZInfo_m_GridSz */
	{0x002A, 0x0B40,},/* reserved */
	{0x0F12, 0x0280,},/* awbb_OutdoorGrZones_m_BGrid_0__m_left */
	{0x0F12, 0x0297,},/* awbb_OutdoorGrZones_m_BGrid_0__m_right */
	{0x0F12, 0x0259,},/* awbb_OutdoorGrZones_m_BGrid_1__m_left */
	{0x0F12, 0x028E,},/* awbb_OutdoorGrZones_m_BGrid_1__m_right */
	{0x0F12, 0x023D,},/* awbb_OutdoorGrZones_m_BGrid_2__m_left */
	{0x0F12, 0x0275,},/* awbb_OutdoorGrZones_m_BGrid_2__m_right */
	{0x0F12, 0x0235,},/* awbb_OutdoorGrZones_m_BGrid_3__m_left */
	{0x0F12, 0x0257,},/* awbb_OutdoorGrZones_m_BGrid_3__m_right */
	{0x0F12, 0x0240,},/* awbb_OutdoorGrZones_m_BGrid_4__m_left */
	{0x0F12, 0x0220,},/* awbb_OutdoorGrZones_m_BGrid_4__m_right */
	{0x0F12, 0x0000,},/* awbb_OutdoorGrZones_m_BGrid_5__m_left */
	{0x0F12, 0x0000,},/* awbb_OutdoorGrZones_m_BGrid_5__m_right */
	{0x0F12, 0x0000,},/* awbb_OutdoorGrZones_m_BGrid_6__m_left */
	{0x0F12, 0x0000,},/* awbb_OutdoorGrZones_m_BGrid_6__m_right */
	{0x0F12, 0x0000,},/* awbb_OutdoorGrZones_m_BGrid_7__m_left */
	{0x0F12, 0x0000,},/* awbb_OutdoorGrZones_m_BGrid_7__m_right */
	{0x0F12, 0x0000,},/* awbb_OutdoorGrZones_m_BGrid_8__m_left */
	{0x0F12, 0x0000,},/* awbb_OutdoorGrZones_m_BGrid_8__m_right */
	{0x0F12, 0x0000,},/* awbb_OutdoorGrZones_m_BGrid_9__m_left */
	{0x0F12, 0x0000,},/* awbb_OutdoorGrZones_m_BGrid_9__m_right */
	{0x0F12, 0x0000,},/* awbb_OutdoorGrZones_m_BGrid_10__m_left */
	{0x0F12, 0x0000,},/* awbb_OutdoorGrZones_m_BGrid_10__m_right */
	{0x0F12, 0x0000,},/* awbb_OutdoorGrZones_m_BGrid_11__m_left */
	{0x0F12, 0x0000,},/* awbb_OutdoorGrZones_m_BGrid_11__m_right */
	{0x002A, 0x0BC8,},/* reserved */
	{0x0F12, 0x0005,},/* awbb_CWSkinZone_ZInfo_m_GridStep */
	{0x002A, 0x0BCC,},/* reserved */
	{0x0F12, 0x0145,},/* awbb_CWSkinZone_ZInfo_m_BMin */
	{0x0F12, 0x01A5,},/* awbb_CWSkinZone_ZInfo_m_BMax */
	{0x002A, 0x0BCA,},/* reserved */
	{0x0F12, 0x0004,},/* awbb_CWSkinZone_ZInfo_m_GridSz */
	{0x002A, 0x0BB4,},/* reserved */
	{0x0F12, 0x03C2,},/* awbb_CWSkinZone_m_BGrid_0__m_left */
	{0x0F12, 0x03E6,},/* awbb_CWSkinZone_m_BGrid_0__m_right */
	{0x0F12, 0x0384,},/* awbb_CWSkinZone_m_BGrid_1__m_left */
	{0x0F12, 0x03E5,},/* awbb_CWSkinZone_m_BGrid_1__m_right */
	{0x0F12, 0x034A,},/* awbb_CWSkinZone_m_BGrid_2__m_left */
	{0x0F12, 0x03B3,},/* awbb_CWSkinZone_m_BGrid_2__m_right */
	{0x0F12, 0x0340,},/* awbb_CWSkinZone_m_BGrid_3__m_left */
	{0x0F12, 0x0362,},/* awbb_CWSkinZone_m_BGrid_3__m_right */
	{0x0F12, 0x0000,},/* awbb_CWSkinZone_m_BGrid_4__m_left */
	{0x0F12, 0x0000,},/* awbb_CWSkinZone_m_BGrid_4__m_right */
	{0x002A, 0x0BE6,},/* reserved */
	{0x0F12, 0x0006,},/* awbb_DLSkinZone_ZInfo_m_GridStep */
	{0x002A, 0x0BEA,},/* reserved */
	{0x0F12, 0x01BF,},/* awbb_DLSkinZone_ZInfo_m_BMin */
	{0x0F12, 0x022A,},/* awbb_DLSkinZone_ZInfo_m_BMax */
	{0x002A, 0x0BE8,},/* reserved */
	{0x0F12, 0x0003,},/* awbb_DLSkinZone_ZInfo_m_GridSz */
	{0x002A, 0x0BD2,},/* reserved */
	{0x0F12, 0x030B,},/* awbb_DLSkinZone_m_BGrid_0__m_left */
	{0x0F12, 0x032F,},/* awbb_DLSkinZone_m_BGrid_0__m_right */
	{0x0F12, 0x02C0,},/* awbb_DLSkinZone_m_BGrid_1__m_left */
	{0x0F12, 0x0305,},/* awbb_DLSkinZone_m_BGrid_1__m_right */
	{0x0F12, 0x029E,},/* awbb_DLSkinZone_m_BGrid_2__m_left */
	{0x0F12, 0x02AC,},/* awbb_DLSkinZone_m_BGrid_2__m_right */
	{0x0F12, 0x0000,},/* awbb_DLSkinZone_m_BGrid_3__m_left */
	{0x0F12, 0x0000,},/* awbb_DLSkinZone_m_BGrid_3__m_right */
	{0x0F12, 0x0000,},/* awbb_DLSkinZone_m_BGrid_4__m_left */
	{0x0F12, 0x0000,},/* awbb_DLSkinZone_m_BGrid_4__m_right */
	{0x002A, 0x0C2C,},/* reserved */
	{0x0F12, 0x0139,},/* awbb_IntcR */
	{0x0F12, 0x0122,},/* awbb_IntcB */
	{0x002A, 0x0BFC,},/* reserved */
	{0x0F12, 0x03AD,},/* awbb_IndoorWP_0__r */
	{0x0F12, 0x013F,},/* awbb_IndoorWP_0__b */
	{0x0F12, 0x0341,},/* awbb_IndoorWP_1__r */
	{0x0F12, 0x017B,},/* awbb_IndoorWP_1__b */
	{0x0F12, 0x038D,},/* awbb_IndoorWP_2__r */
	{0x0F12, 0x014B,},/* awbb_IndoorWP_2__b */
	{0x0F12, 0x02C3,},/* awbb_IndoorWP_3__r */
	{0x0F12, 0x01CC,},/* awbb_IndoorWP_3__b */
	{0x0F12, 0x0241,},/* awbb_IndoorWP_4__r */
	{0x0F12, 0x027F,},/* awbb_IndoorWP_4__b */
	{0x0F12, 0x0241,},/* awbb_IndoorWP_5__r */
	{0x0F12, 0x027F,},/* awbb_IndoorWP_5__b */
	{0x0F12, 0x0214,},/* awbb_IndoorWP_6__r */
	{0x0F12, 0x02A8,},/* awbb_IndoorWP_6__b */
	{0x0F12, 0x0295,},/* awbb_OutdoorWP_r */
	{0x0F12, 0x0210,},/* awbb_OutdoorWP_b */
	{0x002A, 0x0C4C,},/* reserved */
	{0x0F12, 0x0472,},/* awbb_MvEq_RBthresh */
	{0x002A, 0x0C58,},/* reserved */
	{0x0F12, 0x057C,},/* awbb_LowTempRB */
	{0x002A, 0x0BF8,},/* reserved */
	{0x0F12, 0x018A,},/* awbb_LowTSep_m_RminusB */
	{0x002A, 0x0BF4,},/* reserved */
	{0x0F12, 0x0001,},/* awbb_LowBrYThresh_y_low */
	{0x002A, 0x0CAC,},/* reserved */
	{0x0F12, 0x0050,},/* awbb_OutDMaxIncr */
	{0x002A, 0x0C28,},/* reserved */
	{0x0F12, 0x0000,},/* awbb_SkinPreference */
	{0x002A, 0x20BA,},/* reserved */
	{0x0F12, 0x0006,},/* Lowtemp bypass */
	{0x002A, 0x0D0E,},/* reserved */
	{0x0F12, 0x00B5,},/* awbb_GridCoeff_R_2 */
	{0x0F12, 0x00B5,},/* awbb_GridCoeff_B_2 */
	{0x002A, 0x0CFE,},/* reserved */
	{0x0F12, 0x0E58,},/* awbb_GridConst_2_0_ */
	{0x0F12, 0x0F2C,},/* awbb_GridConst_2_1_ */
	{0x0F12, 0x1000,},/* awbb_GridConst_2_2_ */
	{0x0F12, 0x10D4,},/* awbb_GridConst_2_3_ */
	{0x0F12, 0x11A8,},/* awbb_GridConst_2_4_ */
	{0x0F12, 0x127C,},/* awbb_GridConst_2_5_ */
	{0x0F12, 0x00B5,},/* awbb_GridCoeff_R_1  */
	{0x0F12, 0x00B5,},/* awbb_GridCoeff_B_1  */
	{0x002A, 0x0CF8,},/* reserved */
	{0x0F12, 0x027C,},/* awbb_GridConst_1_0_ */
	{0x0F12, 0x0351,},/* awbb_GridConst_1_1_ */
	{0x0F12, 0x0425,},/* awbb_GridConst_1_2_ */
	{0x002A, 0x0CB0,},/* reserved */
	{0x0F12, 0x0000,},/* awbb_GridCorr_R_0__0_ */
	{0x0F12, 0x0000,},/* awbb_GridCorr_R_0__1_ */
	{0x0F12, 0x0000,},/* awbb_GridCorr_R_0__2_ */
	{0x0F12, 0x0000,},/* awbb_GridCorr_R_0__3_ */
	{0x0F12, 0x0000,},/* awbb_GridCorr_R_0__4_ */
	{0x0F12, 0x0000,},/* awbb_GridCorr_R_0__5_ */
	{0x0F12, 0x0000,},/* awbb_GridCorr_R_1__0_ */
	{0x0F12, 0x0000,},/* awbb_GridCorr_R_1__1_ */
	{0x0F12, 0x0000,},/* awbb_GridCorr_R_1__2_ */
	{0x0F12, 0x0000,},/* awbb_GridCorr_R_1__3_ */
	{0x0F12, 0x0000,},/* awbb_GridCorr_R_1__4_ */
	{0x0F12, 0x0000,},/* awbb_GridCorr_R_1__5_ */
	{0x0F12, 0x0000,},/* awbb_GridCorr_R_2__0_ */
	{0x0F12, 0x0000,},/* awbb_GridCorr_R_2__1_ */
	{0x0F12, 0x0000,},/* awbb_GridCorr_R_2__2_ */
	{0x0F12, 0x0000,},/* awbb_GridCorr_R_2__3_ */
	{0x0F12, 0x0000,},/* awbb_GridCorr_R_2__4_ */
	{0x0F12, 0x0000,},/* awbb_GridCorr_R_2__5_ */
	{0x0F12, 0x0000,},/* awbb_GridCorr_B_0__0_ */
	{0x0F12, 0x0000,},/* awbb_GridCorr_B_0__1_ */
	{0x0F12, 0x0000,},/* awbb_GridCorr_B_0__2_ */
	{0x0F12, 0x0000,},/* awbb_GridCorr_B_0__3_ */
	{0x0F12, 0x0000,},/* awbb_GridCorr_B_0__4_ */
	{0x0F12, 0x0000,},/* awbb_GridCorr_B_0__5_ */
	{0x0F12, 0x0000,},/* awbb_GridCorr_B_1__0_ */
	{0x0F12, 0x0000,},/* awbb_GridCorr_B_1__1_ */
	{0x0F12, 0x0000,},/* awbb_GridCorr_B_1__2_ */
	{0x0F12, 0x0000,},/* awbb_GridCorr_B_1__3_ */
	{0x0F12, 0x0000,},/* awbb_GridCorr_B_1__4_ */
	{0x0F12, 0x0000,},/* awbb_GridCorr_B_1__5_ */
	{0x0F12, 0x0000,},/* awbb_GridCorr_B_2__0_ */
	{0x0F12, 0x0000,},/* awbb_GridCorr_B_2__1_ */
	{0x0F12, 0x0000,},/* awbb_GridCorr_B_2__2_ */
	{0x0F12, 0x0000,},/* awbb_GridCorr_B_2__3_ */
	{0x0F12, 0x0000,},/* awbb_GridCorr_B_2__4_ */
	{0x0F12, 0x0000,},/* awbb_GridCorr_B_2__5_ */
	{0x002A, 0x0D30,},/* reserved */
	{0x0F12, 0x0002,},/* awbb_GridEnable */
	{0x002A, 0x3372,},/* reserved */
	{0x0F12, 0x0001,},/* awbb_bUseOutdoorGrid */
	{0x0F12, 0x0000,},/* awbb_OutdoorGridCorr_R */
	{0x0F12, 0x0000,},/* awbb_OutdoorGridCorr_B */
	{0x002A, 0x0C86,},/* reserved */
	{0x0F12, 0x0005,},/* awbb_OutdoorDetectionZone_ZInfo_m_GridSz */
	{0x002A, 0x0C70,},/* reserved */
	{0x0F12, 0xFF7B,},/* awbb_OutdoorDetectionZone_m_BGrid_0__m_left */
	{0x0F12, 0x00CE,},/* awbb_OutdoorDetectionZone_m_BGrid_0__m_right */
	{0x0F12, 0xFF23,},/* awbb_OutdoorDetectionZone_m_BGrid_1__m_left */
	{0x0F12, 0x010D,},/* awbb_OutdoorDetectionZone_m_BGrid_1__m_right */
	{0x0F12, 0xFEF3,},/* awbb_OutdoorDetectionZone_m_BGrid_2__m_left */
	{0x0F12, 0x012C,},/* awbb_OutdoorDetectionZone_m_BGrid_2__m_right */
	{0x0F12, 0xFED7,},/* awbb_OutdoorDetectionZone_m_BGrid_3__m_left */
	{0x0F12, 0x014E,},/* awbb_OutdoorDetectionZone_m_BGrid_3__m_right */
	{0x0F12, 0xFEBB,},/* awbb_OutdoorDetectionZone_m_BGrid_4__m_left */
	{0x0F12, 0x0162,},/* awbb_OutdoorDetectionZone_m_BGrid_4__m_right */
	{0x0F12, 0x1388,},/* awbb_OutdoorDetectionZone_ZInfo_m_AbsGridStep */
	{0x002A, 0x0C8A,},/* reserved */
	{0x0F12, 0x4ACB,},/* awbb_OutdoorDetectionZone_ZInfo_m_MaxNB */
	{0x002A, 0x0C88,},/* reserved */
	{0x0F12, 0x0A7C,},/* awbb_OutdoorDetectionZone_ZInfo_m_NBoffs */
	{0x002A, 0x0CA0,},/* reserved */
	{0x0F12, 0x0030,},/* awbb_GnCurPntImmunity */
	{0x002A, 0x0CA4,},/* reserved */
	{0x0F12, 0x0030,},/* awbb_GnCurPntLongJump */
	{0x0F12, 0x0180,},/* awbb_GainsMaxMove */
	{0x0F12, 0x0002,},/* awbb_GnMinMatchToJump */
	{0x002A, 0x012E,},/* reserved */
	{0x0F12, 0x5DC0,},/* REG_TC_IPRM_InClockLSBs */
	{0x0F12, 0x0000,},/* REG_TC_IPRM_InClockMSBs */
	{0x002A, 0x0146,},/* reserved */
	{0x0F12, 0x0000,},/* REG_TC_IPRM_UseNPviClocks */
	{0x0F12, 0x0001,},/* REG_TC_IPRM_UseNMipiClocks */
	{0x002A, 0x014C,},/* reserved */
	{0x0F12, 0x34BC,},/* REG_TC_IPRM_OpClk4KHz_0 */
	{0x002A, 0x0152,},/* reserved */
	{0x0F12, 0x6978,},/* REG_TC_IPRM_MinOutRate4KHz_0 */
	{0x002A, 0x014E,},/* reserved */
	{0x0F12, 0x6979,},/* REG_TC_IPRM_MaxOutRate4KHz_0 */
	{0x002A, 0x0164,},/* reserved */
	{0x0F12, 0x0001,},/* REG_TC_IPRM_InitParamsUpdated */
	{0x002A, 0x0ABC,},/* reserved */
	{0x0F12, 0x0000,},/* REG_SF_USER_FlickerQuant */
	{0x002A, 0x0AD0,},/* reserved */
	{0x0F12, 0x0080,},/* AFC_SCD_Decay */
	{0x002A, 0x0408,},/* reserved */
	{0x0F12, 0x067F,},/* REG_TC_DBG_AutoAlgEnBits */
	{0x002A, 0x0D40,},/* reserved */
	{0x0F12, 0x0039,},/* TVAR_ae_BrAve */
	{0x002A, 0x0D46,},/* reserved */
	{0x0F12, 0x000F,},/* ae_StatMode */
	{0x002A, 0x0440,},/* reserved */
	{0x0F12, 0x3410,},/* lt_uMaxExp_0_ */
	{0x002A, 0x0444,},/* reserved */
	{0x0F12, 0x6820,},/* lt_uMaxExp_1_ */
	{0x002A, 0x0448,},/* reserved */
	{0x0F12, 0x8227,},/* lt_uMaxExp_2_ */
	{0x002A, 0x044C,},/* reserved */
	{0x0F12, 0xD090,},/* lt_uMaxExp_3_ */
	{0x0F12, 0x0003,},/* lt_uMaxExp_3_ */
	{0x002A, 0x0450,},/* reserved */
	{0x0F12, 0x3410,},/* lt_uCapMaxExp_0_ */
	{0x002A, 0x0454,},/* reserved */
	{0x0F12, 0x6820,},/* lt_uCapMaxExp_1_ */
	{0x002A, 0x0458,},/* reserved */
	{0x0F12, 0x8227,},/* lt_uCapMaxExp_2_ */
	{0x002A, 0x045C,},/* reserved */
	{0x0F12, 0xD090,},/* lt_uCapMaxExp_3_ */
	{0x0F12, 0x0003,},/* lt_uCapMaxExp_3_ */
	{0x002A, 0x0460,},/* reserved */
	{0x0F12, 0x01B0,},/* lt_uMaxAnGain_0_ */
	{0x0F12, 0x01B0,},/* lt_uMaxAnGain_1_ */
	{0x0F12, 0x0280,},/* lt_uMaxAnGain_2_ */
	{0x0F12, 0x0A80,},/* lt_uMaxAnGain_3_ */
	{0x0F12, 0x0100,},/* lt_uMaxDigGain */
	{0x0F12, 0x3000,},/* lt_uMaxTotGain */
	{0x002A, 0x042E,},/* reserved */
	{0x0F12, 0x010E,},/* lt_uMaxTotGain */
	{0x0F12, 0x00F5,},/* lt_uLimitLow */
	{0x002A, 0x0DE0,},/* reserved */
	{0x0F12, 0x0002,},/* ae_Fade2BlackEnable */
	{0x002A, 0x0D4E,},/* reserved */
	{0x0F12, 0x0000,},/* ae_WeightTbl_16_0_ */
	{0x0F12, 0x0101,},/* ae_WeightTbl_16_1_ */
	{0x0F12, 0x0101,},/* ae_WeightTbl_16_2_ */
	{0x0F12, 0x0000,},/* ae_WeightTbl_16_3_ */
	{0x0F12, 0x0101,},/* ae_WeightTbl_16_4_ */
	{0x0F12, 0x0101,},/* ae_WeightTbl_16_5_ */
	{0x0F12, 0x0101,},/* ae_WeightTbl_16_6_ */
	{0x0F12, 0x0101,},/* ae_WeightTbl_16_7_ */
	{0x0F12, 0x0201,},/* ae_WeightTbl_16_8_ */
	{0x0F12, 0x0303,},/* ae_WeightTbl_16_9_ */
	{0x0F12, 0x0303,},/* ae_WeightTbl_16_10_ */
	{0x0F12, 0x0102,},/* ae_WeightTbl_16_11_ */
	{0x0F12, 0x0201,},/* ae_WeightTbl_16_12_ */
	{0x0F12, 0x0403,},/* ae_WeightTbl_16_13_ */
	{0x0F12, 0x0304,},/* ae_WeightTbl_16_14_ */
	{0x0F12, 0x0102,},/* ae_WeightTbl_16_15_ */
	{0x0F12, 0x0201,},/* ae_WeightTbl_16_16_ */
	{0x0F12, 0x0403,},/* ae_WeightTbl_16_17_ */
	{0x0F12, 0x0304,},/* ae_WeightTbl_16_18_ */
	{0x0F12, 0x0102,},/* ae_WeightTbl_16_19_ */
	{0x0F12, 0x0201,},/* ae_WeightTbl_16_20_ */
	{0x0F12, 0x0403,},/* ae_WeightTbl_16_21_ */
	{0x0F12, 0x0304,},/* ae_WeightTbl_16_22_ */
	{0x0F12, 0x0102,},/* ae_WeightTbl_16_23_ */
	{0x0F12, 0x0201,},/* ae_WeightTbl_16_24_ */
	{0x0F12, 0x0303,},/* ae_WeightTbl_16_25_ */
	{0x0F12, 0x0303,},/* ae_WeightTbl_16_26_ */
	{0x0F12, 0x0102,},/* ae_WeightTbl_16_27_ */
	{0x0F12, 0x0201,},/* ae_WeightTbl_16_28_ */
	{0x0F12, 0x0202,},/* ae_WeightTbl_16_29_ */
	{0x0F12, 0x0202,},/* ae_WeightTbl_16_30_ */
	{0x0F12, 0x0102,},/* ae_WeightTbl_16_31_ */
	{0x002A, 0x33A4,},/* reserved */
	{0x0F12, 0x01D0,},/* Tune_wbt_BaseCcms_0__0_ */
	{0x0F12, 0xFFA1,},/* Tune_wbt_BaseCcms_0__1_ */
	{0x0F12, 0xFFFA,},/* Tune_wbt_BaseCcms_0__2_ */
	{0x0F12, 0xFF6F,},/* Tune_wbt_BaseCcms_0__3_ */
	{0x0F12, 0x0140,},/* Tune_wbt_BaseCcms_0__4_ */
	{0x0F12, 0xFF49,},/* Tune_wbt_BaseCcms_0__5_ */
	{0x0F12, 0xFFC1,},/* Tune_wbt_BaseCcms_0__6_ */
	{0x0F12, 0x001F,},/* Tune_wbt_BaseCcms_0__7_ */
	{0x0F12, 0x01BD,},/* Tune_wbt_BaseCcms_0__8_ */
	{0x0F12, 0x013F,},/* Tune_wbt_BaseCcms_0__9_ */
	{0x0F12, 0x00E1,},/* Tune_wbt_BaseCcms_0__10_ */
	{0x0F12, 0xFF43,},/* Tune_wbt_BaseCcms_0__11_ */
	{0x0F12, 0x0191,},/* Tune_wbt_BaseCcms_0__12_ */
	{0x0F12, 0xFFC0,},/* Tune_wbt_BaseCcms_0__13_ */
	{0x0F12, 0x01B7,},/* Tune_wbt_BaseCcms_0__14_ */
	{0x0F12, 0xFF30,},/* Tune_wbt_BaseCcms_0__15_ */
	{0x0F12, 0x015F,},/* Tune_wbt_BaseCcms_0__16_ */
	{0x0F12, 0x0106,},/* Tune_wbt_BaseCcms_0__17_ */
	{0x0F12, 0x01D0,},/* Tune_wbt_BaseCcms_1__0_ */
	{0x0F12, 0xFFA1,},/* Tune_wbt_BaseCcms_1__1_ */
	{0x0F12, 0xFFFA,},/* Tune_wbt_BaseCcms_1__2_ */
	{0x0F12, 0xFF6F,},/* Tune_wbt_BaseCcms_1__3_ */
	{0x0F12, 0x0140,},/* Tune_wbt_BaseCcms_1__4_ */
	{0x0F12, 0xFF49,},/* Tune_wbt_BaseCcms_1__5_ */
	{0x0F12, 0xFFC1,},/* Tune_wbt_BaseCcms_1__6_ */
	{0x0F12, 0x001F,},/* Tune_wbt_BaseCcms_1__7_ */
	{0x0F12, 0x01BD,},/* Tune_wbt_BaseCcms_1__8_ */
	{0x0F12, 0x013F,},/* Tune_wbt_BaseCcms_1__9_ */
	{0x0F12, 0x00E1,},/* Tune_wbt_BaseCcms_1__10_ */
	{0x0F12, 0xFF43,},/* Tune_wbt_BaseCcms_1__11_ */
	{0x0F12, 0x0191,},/* Tune_wbt_BaseCcms_1__12_ */
	{0x0F12, 0xFFC0,},/* Tune_wbt_BaseCcms_1__13_ */
	{0x0F12, 0x01B7,},/* Tune_wbt_BaseCcms_1__14_ */
	{0x0F12, 0xFF30,},/* Tune_wbt_BaseCcms_1__15_ */
	{0x0F12, 0x015F,},/* Tune_wbt_BaseCcms_1__16_ */
	{0x0F12, 0x0106,},/* Tune_wbt_BaseCcms_1__17_ */
	{0x0F12, 0x01D0,},/* Tune_wbt_BaseCcms_2__0_ */
	{0x0F12, 0xFFA1,},/* Tune_wbt_BaseCcms_2__1_ */
	{0x0F12, 0xFFFA,},/* Tune_wbt_BaseCcms_2__2_ */
	{0x0F12, 0xFF6F,},/* Tune_wbt_BaseCcms_2__3_ */
	{0x0F12, 0x0140,},/* Tune_wbt_BaseCcms_2__4_ */
	{0x0F12, 0xFF49,},/* Tune_wbt_BaseCcms_2__5_ */
	{0x0F12, 0xFFC1,},/* Tune_wbt_BaseCcms_2__6_ */
	{0x0F12, 0x001F,},/* Tune_wbt_BaseCcms_2__7_ */
	{0x0F12, 0x01BD,},/* Tune_wbt_BaseCcms_2__8_ */
	{0x0F12, 0x013F,},/* Tune_wbt_BaseCcms_2__9_ */
	{0x0F12, 0x00E1,},/* Tune_wbt_BaseCcms_2__10_ */
	{0x0F12, 0xFF43,},/* Tune_wbt_BaseCcms_2__11_ */
	{0x0F12, 0x0191,},/* Tune_wbt_BaseCcms_2__12_ */
	{0x0F12, 0xFFC0,},/* Tune_wbt_BaseCcms_2__13_ */
	{0x0F12, 0x01B7,},/* Tune_wbt_BaseCcms_2__14_ */
	{0x0F12, 0xFF30,},/* Tune_wbt_BaseCcms_2__15_ */
	{0x0F12, 0x015F,},/* Tune_wbt_BaseCcms_2__16_ */
	{0x0F12, 0x0106,},/* Tune_wbt_BaseCcms_2__17_ */
	{0x0F12, 0x01D0,},/* Tune_wbt_BaseCcms_3__0_ */
	{0x0F12, 0xFFA1,},/* Tune_wbt_BaseCcms_3__1_ */
	{0x0F12, 0xFFFA,},/* Tune_wbt_BaseCcms_3__2_ */
	{0x0F12, 0xFF6F,},/* Tune_wbt_BaseCcms_3__3_ */
	{0x0F12, 0x0140,},/* Tune_wbt_BaseCcms_3__4_ */
	{0x0F12, 0xFF49,},/* Tune_wbt_BaseCcms_3__5_ */
	{0x0F12, 0xFFC1,},/* Tune_wbt_BaseCcms_3__6_ */
	{0x0F12, 0x001F,},/* Tune_wbt_BaseCcms_3__7_ */
	{0x0F12, 0x01BD,},/* Tune_wbt_BaseCcms_3__8_ */
	{0x0F12, 0x013F,},/* Tune_wbt_BaseCcms_3__9_ */
	{0x0F12, 0x00E1,},/* Tune_wbt_BaseCcms_3__10_ */
	{0x0F12, 0xFF43,},/* Tune_wbt_BaseCcms_3__11_ */
	{0x0F12, 0x0191,},/* Tune_wbt_BaseCcms_3__12_ */
	{0x0F12, 0xFFC0,},/* Tune_wbt_BaseCcms_3__13_ */
	{0x0F12, 0x01B7,},/* Tune_wbt_BaseCcms_3__14_ */
	{0x0F12, 0xFF30,},/* Tune_wbt_BaseCcms_3__15_ */
	{0x0F12, 0x015F,},/* Tune_wbt_BaseCcms_3__16_ */
	{0x0F12, 0x0106,},/* Tune_wbt_BaseCcms_3__17_ */
	{0x0F12, 0x01BF,},/* Tune_wbt_BaseCcms_4__0_ */
	{0x0F12, 0xFFBF,},/* Tune_wbt_BaseCcms_4__1_ */
	{0x0F12, 0xFFFE,},/* Tune_wbt_BaseCcms_4__2_ */
	{0x0F12, 0xFF6D,},/* Tune_wbt_BaseCcms_4__3_ */
	{0x0F12, 0x01B4,},/* Tune_wbt_BaseCcms_4__4_ */
	{0x0F12, 0xFF66,},/* Tune_wbt_BaseCcms_4__5_ */
	{0x0F12, 0xFFCA,},/* Tune_wbt_BaseCcms_4__6_ */
	{0x0F12, 0xFFCE,},/* Tune_wbt_BaseCcms_4__7_ */
	{0x0F12, 0x017B,},/* Tune_wbt_BaseCcms_4__8_ */
	{0x0F12, 0x0136,},/* Tune_wbt_BaseCcms_4__9_ */
	{0x0F12, 0x0132,},/* Tune_wbt_BaseCcms_4__10_ */
	{0x0F12, 0xFF85,},/* Tune_wbt_BaseCcms_4__11_ */
	{0x0F12, 0x018B,},/* Tune_wbt_BaseCcms_4__12_ */
	{0x0F12, 0xFF73,},/* Tune_wbt_BaseCcms_4__13_ */
	{0x0F12, 0x0191,},/* Tune_wbt_BaseCcms_4__14_ */
	{0x0F12, 0xFF3F,},/* Tune_wbt_BaseCcms_4__15_ */
	{0x0F12, 0x015B,},/* Tune_wbt_BaseCcms_4__16_ */
	{0x0F12, 0x00D0,},/* Tune_wbt_BaseCcms_4__17_ */
	{0x0F12, 0x01BF,},/* Tune_wbt_BaseCcms_5__0_ */
	{0x0F12, 0xFFBF,},/* Tune_wbt_BaseCcms_5__1_ */
	{0x0F12, 0xFFFE,},/* Tune_wbt_BaseCcms_5__2_ */
	{0x0F12, 0xFF6D,},/* Tune_wbt_BaseCcms_5__3_ */
	{0x0F12, 0x01B4,},/* Tune_wbt_BaseCcms_5__4_ */
	{0x0F12, 0xFF66,},/* Tune_wbt_BaseCcms_5__5_ */
	{0x0F12, 0xFFCA,},/* Tune_wbt_BaseCcms_5__6_ */
	{0x0F12, 0xFFCE,},/* Tune_wbt_BaseCcms_5__7_ */
	{0x0F12, 0x017B,},/* Tune_wbt_BaseCcms_5__8_ */
	{0x0F12, 0x0136,},/* Tune_wbt_BaseCcms_5__9_ */
	{0x0F12, 0x0132,},/* Tune_wbt_BaseCcms_5__10_ */
	{0x0F12, 0xFF85,},/* Tune_wbt_BaseCcms_5__11_ */
	{0x0F12, 0x018B,},/* Tune_wbt_BaseCcms_5__12_ */
	{0x0F12, 0xFF73,},/* Tune_wbt_BaseCcms_5__13_ */
	{0x0F12, 0x0191,},/* Tune_wbt_BaseCcms_5__14_ */
	{0x0F12, 0xFF3F,},/* Tune_wbt_BaseCcms_5__15_ */
	{0x0F12, 0x015B,},/* Tune_wbt_BaseCcms_5__16_ */
	{0x0F12, 0x00D0,},/* Tune_wbt_BaseCcms_5__17_ */
	{0x002A, 0x3380,},/* reserved */
	{0x0F12, 0x01AC,},/* Tune_wbt_OutdoorCcm_0_ */
	{0x0F12, 0xFFD7,},/* Tune_wbt_OutdoorCcm_1_ */
	{0x0F12, 0x0019,},/* Tune_wbt_OutdoorCcm_2_ */
	{0x0F12, 0xFF49,},/* Tune_wbt_OutdoorCcm_3_ */
	{0x0F12, 0x01D9,},/* Tune_wbt_OutdoorCcm_4_ */
	{0x0F12, 0xFF63,},/* Tune_wbt_OutdoorCcm_5_ */
	{0x0F12, 0xFFCA,},/* Tune_wbt_OutdoorCcm_6_ */
	{0x0F12, 0xFFCE,},/* Tune_wbt_OutdoorCcm_7_ */
	{0x0F12, 0x017B,},/* Tune_wbt_OutdoorCcm_8_ */
	{0x0F12, 0x0132,},/* Tune_wbt_OutdoorCcm_9_ */
	{0x0F12, 0x012E,},/* Tune_wbt_OutdoorCcm_10_ */
	{0x0F12, 0xFF8D,},/* Tune_wbt_OutdoorCcm_11_ */
	{0x0F12, 0x018B,},/* Tune_wbt_OutdoorCcm_12_ */
	{0x0F12, 0xFF73,},/* Tune_wbt_OutdoorCcm_13_ */
	{0x0F12, 0x0191,},/* Tune_wbt_OutdoorCcm_14_ */
	{0x0F12, 0xFF3F,},/* Tune_wbt_OutdoorCcm_15_ */
	{0x0F12, 0x015B,},/* Tune_wbt_OutdoorCcm_16_ */
	{0x0F12, 0x00D0,},/* Tune_wbt_OutdoorCcm_17_ */
	{0x002A, 0x0612,},/* reserved */
	{0x0F12, 0x009D,},/* SARR_AwbCcmCord_0_ */
	{0x0F12, 0x00D5,},/* SARR_AwbCcmCord_1_ */
	{0x0F12, 0x0103,},/* SARR_AwbCcmCord_2_ */
	{0x0F12, 0x0128,},/* SARR_AwbCcmCord_3_ */
	{0x0F12, 0x0166,},/* SARR_AwbCcmCord_4_ */
	{0x0F12, 0x0193,},/* SARR_AwbCcmCord_5_ */
	{0x002A, 0x0538,},/* reserved */
	{0x0F12, 0x0000,},/* seti_uGammaLutPreDemNoBin_0_ */
	{0x0F12, 0x001F,},/* seti_uGammaLutPreDemNoBin_1_ */
	{0x0F12, 0x0035,},/* seti_uGammaLutPreDemNoBin_2_ */
	{0x0F12, 0x005A,},/* seti_uGammaLutPreDemNoBin_3_ */
	{0x0F12, 0x0095,},/* seti_uGammaLutPreDemNoBin_4_ */
	{0x0F12, 0x00E6,},/* seti_uGammaLutPreDemNoBin_5_ */
	{0x0F12, 0x0121,},/* seti_uGammaLutPreDemNoBin_6_ */
	{0x0F12, 0x0139,},/* seti_uGammaLutPreDemNoBin_7_ */
	{0x0F12, 0x0150,},/* seti_uGammaLutPreDemNoBin_8_ */
	{0x0F12, 0x0177,},/* seti_uGammaLutPreDemNoBin_9_ */
	{0x0F12, 0x019A,},/* seti_uGammaLutPreDemNoBin_10_ */
	{0x0F12, 0x01BB,},/* seti_uGammaLutPreDemNoBin_11_ */
	{0x0F12, 0x01DC,},/* seti_uGammaLutPreDemNoBin_12_ */
	{0x0F12, 0x0219,},/* seti_uGammaLutPreDemNoBin_13_ */
	{0x0F12, 0x0251,},/* seti_uGammaLutPreDemNoBin_14_ */
	{0x0F12, 0x02B3,},/* seti_uGammaLutPreDemNoBin_15_ */
	{0x0F12, 0x030A,},/* seti_uGammaLutPreDemNoBin_16_ */
	{0x0F12, 0x035F,},/* seti_uGammaLutPreDemNoBin_17_ */
	{0x0F12, 0x03B1,},/* seti_uGammaLutPreDemNoBin_18_ */
	{0x0F12, 0x03FF,},/* seti_uGammaLutPreDemNoBin_19_ */
	{0x0F12, 0x0000,},/* seti_uGammaLutPostDemNoBin_0_ */
	{0x0F12, 0x0001,},/* seti_uGammaLutPostDemNoBin_1_ */
	{0x0F12, 0x0001,},/* seti_uGammaLutPostDemNoBin_2_ */
	{0x0F12, 0x0002,},/* seti_uGammaLutPostDemNoBin_3_ */
	{0x0F12, 0x0004,},/* seti_uGammaLutPostDemNoBin_4_ */
	{0x0F12, 0x000A,},/* seti_uGammaLutPostDemNoBin_5_ */
	{0x0F12, 0x0012,},/* seti_uGammaLutPostDemNoBin_6_ */
	{0x0F12, 0x0016,},/* seti_uGammaLutPostDemNoBin_7_ */
	{0x0F12, 0x001A,},/* seti_uGammaLutPostDemNoBin_8_ */
	{0x0F12, 0x0024,},/* seti_uGammaLutPostDemNoBin_9_ */
	{0x0F12, 0x0031,},/* seti_uGammaLutPostDemNoBin_10_ */
	{0x0F12, 0x003E,},/* seti_uGammaLutPostDemNoBin_11_ */
	{0x0F12, 0x004E,},/* seti_uGammaLutPostDemNoBin_12_ */
	{0x0F12, 0x0075,},/* seti_uGammaLutPostDemNoBin_13_ */
	{0x0F12, 0x00A8,},/* seti_uGammaLutPostDemNoBin_14_ */
	{0x0F12, 0x0126,},/* seti_uGammaLutPostDemNoBin_15_ */
	{0x0F12, 0x01BE,},/* seti_uGammaLutPostDemNoBin_16_ */
	{0x0F12, 0x0272,},/* seti_uGammaLutPostDemNoBin_17_ */
	{0x0F12, 0x0334,},/* seti_uGammaLutPostDemNoBin_18_ */
	{0x0F12, 0x03FF,},/* seti_uGammaLutPostDemNoBin_19_ */
	{0x002A, 0x0498,},/* reserved */
	{0x0F12, 0x0000,},/* SARR_usDualGammaLutRGBIndoor_0__0_ */
	{0x0F12, 0x0002,},/* SARR_usDualGammaLutRGBIndoor_0__1_ */
	{0x0F12, 0x0007,},/* SARR_usDualGammaLutRGBIndoor_0__2_ */
	{0x0F12, 0x001D,},/* SARR_usDualGammaLutRGBIndoor_0__3_ */
	{0x0F12, 0x006E,},/* SARR_usDualGammaLutRGBIndoor_0__4_ */
	{0x0F12, 0x00D3,},/* SARR_usDualGammaLutRGBIndoor_0__5_ */
	{0x0F12, 0x0127,},/* SARR_usDualGammaLutRGBIndoor_0__6_ */
	{0x0F12, 0x014C,},/* SARR_usDualGammaLutRGBIndoor_0__7_ */
	{0x0F12, 0x016E,},/* SARR_usDualGammaLutRGBIndoor_0__8_ */
	{0x0F12, 0x01A5,},/* SARR_usDualGammaLutRGBIndoor_0__9_ */
	{0x0F12, 0x01D3,},/* SARR_usDualGammaLutRGBIndoor_0__10_ */
	{0x0F12, 0x01FB,},/* SARR_usDualGammaLutRGBIndoor_0__11_ */
	{0x0F12, 0x021F,},/* SARR_usDualGammaLutRGBIndoor_0__12_ */
	{0x0F12, 0x0260,},/* SARR_usDualGammaLutRGBIndoor_0__13_ */
	{0x0F12, 0x029A,},/* SARR_usDualGammaLutRGBIndoor_0__14_ */
	{0x0F12, 0x02F7,},/* SARR_usDualGammaLutRGBIndoor_0__15_ */
	{0x0F12, 0x034D,},/* SARR_usDualGammaLutRGBIndoor_0__16_ */
	{0x0F12, 0x0395,},/* SARR_usDualGammaLutRGBIndoor_0__17_ */
	{0x0F12, 0x03CE,},/* SARR_usDualGammaLutRGBIndoor_0__18_ */
	{0x0F12, 0x03FF,},/* SARR_usDualGammaLutRGBIndoor_0__19_ */
	{0x0F12, 0x0000,},/* SARR_usDualGammaLutRGBOutdoor_0__0_ */
	{0x0F12, 0x0004,},/* SARR_usDualGammaLutRGBOutdoor_0__1_ */
	{0x0F12, 0x000C,},/* SARR_usDualGammaLutRGBOutdoor_0__2_ */
	{0x0F12, 0x0024,},/* SARR_usDualGammaLutRGBOutdoor_0__3_ */
	{0x0F12, 0x006E,},/* SARR_usDualGammaLutRGBOutdoor_0__4_ */
	{0x0F12, 0x00D1,},/* SARR_usDualGammaLutRGBOutdoor_0__5_ */
	{0x0F12, 0x0119,},/* SARR_usDualGammaLutRGBOutdoor_0__6_ */
	{0x0F12, 0x0139,},/* SARR_usDualGammaLutRGBOutdoor_0__7_ */
	{0x0F12, 0x0157,},/* SARR_usDualGammaLutRGBOutdoor_0__8_ */
	{0x0F12, 0x018E,},/* SARR_usDualGammaLutRGBOutdoor_0__9_ */
	{0x0F12, 0x01C3,},/* SARR_usDualGammaLutRGBOutdoor_0__10_ */
	{0x0F12, 0x01F3,},/* SARR_usDualGammaLutRGBOutdoor_0__11_ */
	{0x0F12, 0x021F,},/* SARR_usDualGammaLutRGBOutdoor_0__12_ */
	{0x0F12, 0x0269,},/* SARR_usDualGammaLutRGBOutdoor_0__13_ */
	{0x0F12, 0x02A6,},/* SARR_usDualGammaLutRGBOutdoor_0__14_ */
	{0x0F12, 0x02FF,},/* SARR_usDualGammaLutRGBOutdoor_0__15_ */
	{0x0F12, 0x0351,},/* SARR_usDualGammaLutRGBOutdoor_0__16_ */
	{0x0F12, 0x0395,},/* SARR_usDualGammaLutRGBOutdoor_0__17_ */
	{0x0F12, 0x03CE,},/* SARR_usDualGammaLutRGBOutdoor_0__18_ */
	{0x0F12, 0x03FF,},/* SARR_usDualGammaLutRGBOutdoor_0__19_ */
	{0x002A, 0x06D4,},/* reserved */
	{0x0F12, 0x0032,},/* afit_uNoiseIndInDoor_0_ */
	{0x0F12, 0x0078,},/* afit_uNoiseIndInDoor_1_ */
	{0x0F12, 0x00C8,},/* afit_uNoiseIndInDoor_2_ */
	{0x0F12, 0x0190,},/* afit_uNoiseIndInDoor_3_ */
	{0x0F12, 0x028C,},/* afit_uNoiseIndInDoor_4_ */
	{0x002A, 0x0734,},/* reserved */
	{0x0F12, 0x0000,},/* AfitBaseVals_0__0_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_0__1_ */
	{0x0F12, 0x000F,},/* AfitBaseVals_0__2_ */
	{0x0F12, 0x0005,},/* AfitBaseVals_0__3_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_0__4_ */
	{0x0F12, 0x0078,},/* AfitBaseVals_0__5_ */
	{0x0F12, 0x012C,},/* AfitBaseVals_0__6_ */
	{0x0F12, 0x03FF,},/* AfitBaseVals_0__7_ */
	{0x0F12, 0x0014,},/* AfitBaseVals_0__8_ */
	{0x0F12, 0x0064,},/* AfitBaseVals_0__9_ */
	{0x0F12, 0x000C,},/* AfitBaseVals_0__10_ */
	{0x0F12, 0x0010,},/* AfitBaseVals_0__11_ */
	{0x0F12, 0x01E6,},/* AfitBaseVals_0__12_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_0__13_ */
	{0x0F12, 0x0070,},/* AfitBaseVals_0__14_ */
	{0x0F12, 0x01FF,},/* AfitBaseVals_0__15_ */
	{0x0F12, 0x0144,},/* AfitBaseVals_0__16_ */
	{0x0F12, 0x000F,},/* AfitBaseVals_0__17_ */
	{0x0F12, 0x000A,},/* AfitBaseVals_0__18_ */
	{0x0F12, 0x0073,},/* AfitBaseVals_0__19_ */
	{0x0F12, 0x0087,},/* AfitBaseVals_0__20_ */
	{0x0F12, 0x0014,},/* AfitBaseVals_0__21_ */
	{0x0F12, 0x000A,},/* AfitBaseVals_0__22_ */
	{0x0F12, 0x0023,},/* AfitBaseVals_0__23_ */
	{0x0F12, 0x001E,},/* AfitBaseVals_0__24_ */
	{0x0F12, 0x0014,},/* AfitBaseVals_0__25_ */
	{0x0F12, 0x000A,},/* AfitBaseVals_0__26_ */
	{0x0F12, 0x0023,},/* AfitBaseVals_0__27_ */
	{0x0F12, 0x0046,},/* AfitBaseVals_0__28_ */
	{0x0F12, 0x2B32,},/* AfitBaseVals_0__29_ */
	{0x0F12, 0x0601,},/* AfitBaseVals_0__30_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_0__31_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_0__32_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_0__33_ */
	{0x0F12, 0x00FF,},/* AfitBaseVals_0__34_ */
	{0x0F12, 0x07FF,},/* AfitBaseVals_0__35_ */
	{0x0F12, 0xFFFF,},/* AfitBaseVals_0__36_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_0__37_ */
	{0x0F12, 0x050D,},/* AfitBaseVals_0__38_ */
	{0x0F12, 0x1E80,},/* AfitBaseVals_0__39_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_0__40_ */
	{0x0F12, 0x1408,},/* AfitBaseVals_0__41_ */
	{0x0F12, 0x0214,},/* AfitBaseVals_0__42_ */
	{0x0F12, 0xFF01,},/* AfitBaseVals_0__43_ */
	{0x0F12, 0x180F,},/* AfitBaseVals_0__44_ */
	{0x0F12, 0x0001,},/* AfitBaseVals_0__45_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_0__46_ */
	{0x0F12, 0x8003,},/* AfitBaseVals_0__47_ */
	{0x0F12, 0x0080,},/* AfitBaseVals_0__48_ */
	{0x0F12, 0x0080,},/* AfitBaseVals_0__49_ */
	{0x0F12, 0x0180,},/* AfitBaseVals_0__50_ */
	{0x0F12, 0x0308,},/* AfitBaseVals_0__51_ */
	{0x0F12, 0xFFFF,},/* AfitBaseVals_0__52_ */
	{0x0F12, 0xFFFF,},/* AfitBaseVals_0__53_ */
	{0x0F12, 0x0A02,},/* AfitBaseVals_0__54_ */
	{0x0F12, 0x080A,},/* AfitBaseVals_0__55_ */
	{0x0F12, 0x0500,},/* AfitBaseVals_0__56_ */
	{0x0F12, 0x032D,},/* AfitBaseVals_0__57_ */
	{0x0F12, 0x324E,},/* AfitBaseVals_0__58_ */
	{0x0F12, 0xFF1E,},/* AfitBaseVals_0__59_ */
	{0x0F12, 0x02FF,},/* AfitBaseVals_0__60_ */
	{0x0F12, 0x0103,},/* AfitBaseVals_0__61_ */
	{0x0F12, 0x010C,},/* AfitBaseVals_0__62_ */
	{0x0F12, 0x9696,},/* AfitBaseVals_0__63_ */
	{0x0F12, 0x4646,},/* AfitBaseVals_0__64_ */
	{0x0F12, 0x0802,},/* AfitBaseVals_0__65_ */
	{0x0F12, 0x0802,},/* AfitBaseVals_0__66_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_0__67_ */
	{0x0F12, 0x030F,},/* AfitBaseVals_0__68_ */
	{0x0F12, 0x3202,},/* AfitBaseVals_0__69_ */
	{0x0F12, 0x0F1E,},/* AfitBaseVals_0__70_ */
	{0x0F12, 0x020F,},/* AfitBaseVals_0__71_ */
	{0x0F12, 0x0103,},/* AfitBaseVals_0__72_ */
	{0x0F12, 0x010C,},/* AfitBaseVals_0__73_ */
	{0x0F12, 0x9696,},/* AfitBaseVals_0__74_ */
	{0x0F12, 0x4646,},/* AfitBaseVals_0__75_ */
	{0x0F12, 0x0802,},/* AfitBaseVals_0__76_ */
	{0x0F12, 0x0802,},/* AfitBaseVals_0__77_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_0__78_ */
	{0x0F12, 0x030F,},/* AfitBaseVals_0__79_ */
	{0x0F12, 0x3202,},/* AfitBaseVals_0__80_ */
	{0x0F12, 0x0F1E,},/* AfitBaseVals_0__81_ */
	{0x0F12, 0x020F,},/* AfitBaseVals_0__82_ */
	{0x0F12, 0x0003,},/* AfitBaseVals_0__83_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_1__0_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_1__1_ */
	{0x0F12, 0x000F,},/* AfitBaseVals_1__2_ */
	{0x0F12, 0x0005,},/* AfitBaseVals_1__3_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_1__4_ */
	{0x0F12, 0x006A,},/* AfitBaseVals_1__5_ */
	{0x0F12, 0x012C,},/* AfitBaseVals_1__6_ */
	{0x0F12, 0x03FF,},/* AfitBaseVals_1__7_ */
	{0x0F12, 0x0014,},/* AfitBaseVals_1__8_ */
	{0x0F12, 0x0064,},/* AfitBaseVals_1__9_ */
	{0x0F12, 0x000C,},/* AfitBaseVals_1__10_ */
	{0x0F12, 0x0010,},/* AfitBaseVals_1__11_ */
	{0x0F12, 0x01E6,},/* AfitBaseVals_1__12_ */
	{0x0F12, 0x03FF,},/* AfitBaseVals_1__13_ */
	{0x0F12, 0x0070,},/* AfitBaseVals_1__14_ */
	{0x0F12, 0x007D,},/* AfitBaseVals_1__15_ */
	{0x0F12, 0x0064,},/* AfitBaseVals_1__16_ */
	{0x0F12, 0x0014,},/* AfitBaseVals_1__17_ */
	{0x0F12, 0x000A,},/* AfitBaseVals_1__18_ */
	{0x0F12, 0x0073,},/* AfitBaseVals_1__19_ */
	{0x0F12, 0x0087,},/* AfitBaseVals_1__20_ */
	{0x0F12, 0x0014,},/* AfitBaseVals_1__21_ */
	{0x0F12, 0x000A,},/* AfitBaseVals_1__22_ */
	{0x0F12, 0x0023,},/* AfitBaseVals_1__23_ */
	{0x0F12, 0x001E,},/* AfitBaseVals_1__24_ */
	{0x0F12, 0x0014,},/* AfitBaseVals_1__25_ */
	{0x0F12, 0x000A,},/* AfitBaseVals_1__26_ */
	{0x0F12, 0x0023,},/* AfitBaseVals_1__27_ */
	{0x0F12, 0x001E,},/* AfitBaseVals_1__28_ */
	{0x0F12, 0x2B32,},/* AfitBaseVals_1__29_ */
	{0x0F12, 0x0601,},/* AfitBaseVals_1__30_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_1__31_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_1__32_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_1__33_ */
	{0x0F12, 0x00FF,},/* AfitBaseVals_1__34_ */
	{0x0F12, 0x07FF,},/* AfitBaseVals_1__35_ */
	{0x0F12, 0xFFFF,},/* AfitBaseVals_1__36_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_1__37_ */
	{0x0F12, 0x050D,},/* AfitBaseVals_1__38_ */
	{0x0F12, 0x1E80,},/* AfitBaseVals_1__39_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_1__40_ */
	{0x0F12, 0x1408,},/* AfitBaseVals_1__41_ */
	{0x0F12, 0x0214,},/* AfitBaseVals_1__42_ */
	{0x0F12, 0xFF01,},/* AfitBaseVals_1__43_ */
	{0x0F12, 0x180F,},/* AfitBaseVals_1__44_ */
	{0x0F12, 0x0002,},/* AfitBaseVals_1__45_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_1__46_ */
	{0x0F12, 0x8003,},/* AfitBaseVals_1__47_ */
	{0x0F12, 0x008C,},/* AfitBaseVals_1__48_ */
	{0x0F12, 0x0080,},/* AfitBaseVals_1__49_ */
	{0x0F12, 0x0180,},/* AfitBaseVals_1__50_ */
	{0x0F12, 0x0308,},/* AfitBaseVals_1__51_ */
	{0x0F12, 0x1E65,},/* AfitBaseVals_1__52_ */
	{0x0F12, 0x1A24,},/* AfitBaseVals_1__53_ */
	{0x0F12, 0x0A03,},/* AfitBaseVals_1__54_ */
	{0x0F12, 0x080A,},/* AfitBaseVals_1__55_ */
	{0x0F12, 0x0500,},/* AfitBaseVals_1__56_ */
	{0x0F12, 0x032D,},/* AfitBaseVals_1__57_ */
	{0x0F12, 0x324D,},/* AfitBaseVals_1__58_ */
	{0x0F12, 0xFF1E,},/* AfitBaseVals_1__59_ */
	{0x0F12, 0x02FF,},/* AfitBaseVals_1__60_ */
	{0x0F12, 0x0103,},/* AfitBaseVals_1__61_ */
	{0x0F12, 0x010C,},/* AfitBaseVals_1__62_ */
	{0x0F12, 0x9696,},/* AfitBaseVals_1__63_ */
	{0x0F12, 0x2F34,},/* AfitBaseVals_1__64_ */
	{0x0F12, 0x0504,},/* AfitBaseVals_1__65_ */
	{0x0F12, 0x080F,},/* AfitBaseVals_1__66_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_1__67_ */
	{0x0F12, 0x030F,},/* AfitBaseVals_1__68_ */
	{0x0F12, 0x3208,},/* AfitBaseVals_1__69_ */
	{0x0F12, 0x0F1E,},/* AfitBaseVals_1__70_ */
	{0x0F12, 0x020F,},/* AfitBaseVals_1__71_ */
	{0x0F12, 0x0103,},/* AfitBaseVals_1__72_ */
	{0x0F12, 0x010C,},/* AfitBaseVals_1__73_ */
	{0x0F12, 0x9696,},/* AfitBaseVals_1__74_ */
	{0x0F12, 0x1414,},/* AfitBaseVals_1__75_ */
	{0x0F12, 0x0504,},/* AfitBaseVals_1__76_ */
	{0x0F12, 0x080F,},/* AfitBaseVals_1__77_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_1__78_ */
	{0x0F12, 0x030F,},/* AfitBaseVals_1__79_ */
	{0x0F12, 0x3208,},/* AfitBaseVals_1__80_ */
	{0x0F12, 0x0F1E,},/* AfitBaseVals_1__81_ */
	{0x0F12, 0x020F,},/* AfitBaseVals_1__82_ */
	{0x0F12, 0x0003,},/* AfitBaseVals_1__83_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_2__0_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_2__1_ */
	{0x0F12, 0x000F,},/* AfitBaseVals_2__2_ */
	{0x0F12, 0x0005,},/* AfitBaseVals_2__3_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_2__4_ */
	{0x0F12, 0x0064,},/* AfitBaseVals_2__5_ */
	{0x0F12, 0x012C,},/* AfitBaseVals_2__6_ */
	{0x0F12, 0x03FF,},/* AfitBaseVals_2__7_ */
	{0x0F12, 0x0014,},/* AfitBaseVals_2__8_ */
	{0x0F12, 0x0064,},/* AfitBaseVals_2__9_ */
	{0x0F12, 0x000C,},/* AfitBaseVals_2__10_ */
	{0x0F12, 0x0010,},/* AfitBaseVals_2__11_ */
	{0x0F12, 0x01E6,},/* AfitBaseVals_2__12_ */
	{0x0F12, 0x03FF,},/* AfitBaseVals_2__13_ */
	{0x0F12, 0x0070,},/* AfitBaseVals_2__14_ */
	{0x0F12, 0x007D,},/* AfitBaseVals_2__15_ */
	{0x0F12, 0x0064,},/* AfitBaseVals_2__16_ */
	{0x0F12, 0x0032,},/* AfitBaseVals_2__17_ */
	{0x0F12, 0x0096,},/* AfitBaseVals_2__18_ */
	{0x0F12, 0x0073,},/* AfitBaseVals_2__19_ */
	{0x0F12, 0x0087,},/* AfitBaseVals_2__20_ */
	{0x0F12, 0x0014,},/* AfitBaseVals_2__21_ */
	{0x0F12, 0x0019,},/* AfitBaseVals_2__22_ */
	{0x0F12, 0x0023,},/* AfitBaseVals_2__23_ */
	{0x0F12, 0x001E,},/* AfitBaseVals_2__24_ */
	{0x0F12, 0x0014,},/* AfitBaseVals_2__25_ */
	{0x0F12, 0x0019,},/* AfitBaseVals_2__26_ */
	{0x0F12, 0x0023,},/* AfitBaseVals_2__27_ */
	{0x0F12, 0x001E,},/* AfitBaseVals_2__28_ */
	{0x0F12, 0x2B32,},/* AfitBaseVals_2__29_ */
	{0x0F12, 0x0601,},/* AfitBaseVals_2__30_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_2__31_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_2__32_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_2__33_ */
	{0x0F12, 0x00FF,},/* AfitBaseVals_2__34_ */
	{0x0F12, 0x07FF,},/* AfitBaseVals_2__35_ */
	{0x0F12, 0xFFFF,},/* AfitBaseVals_2__36_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_2__37_ */
	{0x0F12, 0x050D,},/* AfitBaseVals_2__38_ */
	{0x0F12, 0x1E80,},/* AfitBaseVals_2__39_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_2__40_ */
	{0x0F12, 0x0A08,},/* AfitBaseVals_2__41_ */
	{0x0F12, 0x0200,},/* AfitBaseVals_2__42_ */
	{0x0F12, 0xFF01,},/* AfitBaseVals_2__43_ */
	{0x0F12, 0x180F,},/* AfitBaseVals_2__44_ */
	{0x0F12, 0x0002,},/* AfitBaseVals_2__45_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_2__46_ */
	{0x0F12, 0x8003,},/* AfitBaseVals_2__47_ */
	{0x0F12, 0x008C,},/* AfitBaseVals_2__48_ */
	{0x0F12, 0x0080,},/* AfitBaseVals_2__49_ */
	{0x0F12, 0x0180,},/* AfitBaseVals_2__50_ */
	{0x0F12, 0x0208,},/* AfitBaseVals_2__51_ */
	{0x0F12, 0x1E4B,},/* AfitBaseVals_2__52_ */
	{0x0F12, 0x1A24,},/* AfitBaseVals_2__53_ */
	{0x0F12, 0x0A05,},/* AfitBaseVals_2__54_ */
	{0x0F12, 0x080A,},/* AfitBaseVals_2__55_ */
	{0x0F12, 0x0500,},/* AfitBaseVals_2__56_ */
	{0x0F12, 0x032D,},/* AfitBaseVals_2__57_ */
	{0x0F12, 0x324D,},/* AfitBaseVals_2__58_ */
	{0x0F12, 0x001E,},/* AfitBaseVals_2__59_ */
	{0x0F12, 0x0200,},/* AfitBaseVals_2__60_ */
	{0x0F12, 0x0103,},/* AfitBaseVals_2__61_ */
	{0x0F12, 0x010C,},/* AfitBaseVals_2__62_ */
	{0x0F12, 0x9696,},/* AfitBaseVals_2__63_ */
	{0x0F12, 0x1E23,},/* AfitBaseVals_2__64_ */
	{0x0F12, 0x0505,},/* AfitBaseVals_2__65_ */
	{0x0F12, 0x080F,},/* AfitBaseVals_2__66_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_2__67_ */
	{0x0F12, 0x030F,},/* AfitBaseVals_2__68_ */
	{0x0F12, 0x3208,},/* AfitBaseVals_2__69_ */
	{0x0F12, 0x0F1E,},/* AfitBaseVals_2__70_ */
	{0x0F12, 0x020F,},/* AfitBaseVals_2__71_ */
	{0x0F12, 0x0103,},/* AfitBaseVals_2__72_ */
	{0x0F12, 0x010C,},/* AfitBaseVals_2__73_ */
	{0x0F12, 0x9696,},/* AfitBaseVals_2__74_ */
	{0x0F12, 0x1E23,},/* AfitBaseVals_2__75_ */
	{0x0F12, 0x0505,},/* AfitBaseVals_2__76_ */
	{0x0F12, 0x080F,},/* AfitBaseVals_2__77_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_2__78_ */
	{0x0F12, 0x030F,},/* AfitBaseVals_2__79_ */
	{0x0F12, 0x3208,},/* AfitBaseVals_2__80_ */
	{0x0F12, 0x0F1E,},/* AfitBaseVals_2__81_ */
	{0x0F12, 0x020F,},/* AfitBaseVals_2__82_ */
	{0x0F12, 0x0003,},/* AfitBaseVals_2__83_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_3__0_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_3__1_ */
	{0x0F12, 0x000F,},/* AfitBaseVals_3__2_ */
	{0x0F12, 0x0007,},/* AfitBaseVals_3__3_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_3__4_ */
	{0x0F12, 0x0064,},/* AfitBaseVals_3__5_ */
	{0x0F12, 0x012C,},/* AfitBaseVals_3__6_ */
	{0x0F12, 0x03FF,},/* AfitBaseVals_3__7_ */
	{0x0F12, 0x0014,},/* AfitBaseVals_3__8_ */
	{0x0F12, 0x0064,},/* AfitBaseVals_3__9_ */
	{0x0F12, 0x000C,},/* AfitBaseVals_3__10_ */
	{0x0F12, 0x0010,},/* AfitBaseVals_3__11_ */
	{0x0F12, 0x01E6,},/* AfitBaseVals_3__12_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_3__13_ */
	{0x0F12, 0x0070,},/* AfitBaseVals_3__14_ */
	{0x0F12, 0x007D,},/* AfitBaseVals_3__15_ */
	{0x0F12, 0x0064,},/* AfitBaseVals_3__16_ */
	{0x0F12, 0x0032,},/* AfitBaseVals_3__17_ */
	{0x0F12, 0x0096,},/* AfitBaseVals_3__18_ */
	{0x0F12, 0x0073,},/* AfitBaseVals_3__19_ */
	{0x0F12, 0x009F,},/* AfitBaseVals_3__20_ */
	{0x0F12, 0x0028,},/* AfitBaseVals_3__21_ */
	{0x0F12, 0x0028,},/* AfitBaseVals_3__22_ */
	{0x0F12, 0x0023,},/* AfitBaseVals_3__23_ */
	{0x0F12, 0x0037,},/* AfitBaseVals_3__24_ */
	{0x0F12, 0x0028,},/* AfitBaseVals_3__25_ */
	{0x0F12, 0x0028,},/* AfitBaseVals_3__26_ */
	{0x0F12, 0x0023,},/* AfitBaseVals_3__27_ */
	{0x0F12, 0x0037,},/* AfitBaseVals_3__28_ */
	{0x0F12, 0x2B32,},/* AfitBaseVals_3__29_ */
	{0x0F12, 0x0601,},/* AfitBaseVals_3__30_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_3__31_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_3__32_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_3__33_ */
	{0x0F12, 0x00FF,},/* AfitBaseVals_3__34_ */
	{0x0F12, 0x07A0,},/* AfitBaseVals_3__35_ */
	{0x0F12, 0xFFFF,},/* AfitBaseVals_3__36_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_3__37_ */
	{0x0F12, 0x050D,},/* AfitBaseVals_3__38_ */
	{0x0F12, 0x1E80,},/* AfitBaseVals_3__39_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_3__40_ */
	{0x0F12, 0x0A08,},/* AfitBaseVals_3__41_ */
	{0x0F12, 0x0200,},/* AfitBaseVals_3__42_ */
	{0x0F12, 0xFF01,},/* AfitBaseVals_3__43_ */
	{0x0F12, 0x180F,},/* AfitBaseVals_3__44_ */
	{0x0F12, 0x0001,},/* AfitBaseVals_3__45_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_3__46_ */
	{0x0F12, 0x8003,},/* AfitBaseVals_3__47_ */
	{0x0F12, 0x008C,},/* AfitBaseVals_3__48_ */
	{0x0F12, 0x0080,},/* AfitBaseVals_3__49_ */
	{0x0F12, 0x0180,},/* AfitBaseVals_3__50_ */
	{0x0F12, 0x0108,},/* AfitBaseVals_3__51_ */
	{0x0F12, 0x1E32,},/* AfitBaseVals_3__52_ */
	{0x0F12, 0x1A24,},/* AfitBaseVals_3__53_ */
	{0x0F12, 0x0A05,},/* AfitBaseVals_3__54_ */
	{0x0F12, 0x080A,},/* AfitBaseVals_3__55_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_3__56_ */
	{0x0F12, 0x0328,},/* AfitBaseVals_3__57_ */
	{0x0F12, 0x324C,},/* AfitBaseVals_3__58_ */
	{0x0F12, 0x001E,},/* AfitBaseVals_3__59_ */
	{0x0F12, 0x0200,},/* AfitBaseVals_3__60_ */
	{0x0F12, 0x0103,},/* AfitBaseVals_3__61_ */
	{0x0F12, 0x010C,},/* AfitBaseVals_3__62_ */
	{0x0F12, 0x9696,},/* AfitBaseVals_3__63_ */
	{0x0F12, 0x0F0F,},/* AfitBaseVals_3__64_ */
	{0x0F12, 0x0307,},/* AfitBaseVals_3__65_ */
	{0x0F12, 0x080F,},/* AfitBaseVals_3__66_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_3__67_ */
	{0x0F12, 0x030F,},/* AfitBaseVals_3__68_ */
	{0x0F12, 0x3208,},/* AfitBaseVals_3__69_ */
	{0x0F12, 0x0F1E,},/* AfitBaseVals_3__70_ */
	{0x0F12, 0x020F,},/* AfitBaseVals_3__71_ */
	{0x0F12, 0x0103,},/* AfitBaseVals_3__72_ */
	{0x0F12, 0x010C,},/* AfitBaseVals_3__73_ */
	{0x0F12, 0x9696,},/* AfitBaseVals_3__74_ */
	{0x0F12, 0x0F0F,},/* AfitBaseVals_3__75_ */
	{0x0F12, 0x0307,},/* AfitBaseVals_3__76_ */
	{0x0F12, 0x080F,},/* AfitBaseVals_3__77_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_3__78_ */
	{0x0F12, 0x030F,},/* AfitBaseVals_3__79_ */
	{0x0F12, 0x3208,},/* AfitBaseVals_3__80_ */
	{0x0F12, 0x0F1E,},/* AfitBaseVals_3__81_ */
	{0x0F12, 0x020F,},/* AfitBaseVals_3__82_ */
	{0x0F12, 0x0003,},/* AfitBaseVals_3__83_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_4__0_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_4__1_ */
	{0x0F12, 0x000F,},/* AfitBaseVals_4__2_ */
	{0x0F12, 0x0007,},/* AfitBaseVals_4__3_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_4__4_ */
	{0x0F12, 0x0028,},/* AfitBaseVals_4__5_ */
	{0x0F12, 0x012C,},/* AfitBaseVals_4__6_ */
	{0x0F12, 0x03FF,},/* AfitBaseVals_4__7_ */
	{0x0F12, 0x0014,},/* AfitBaseVals_4__8_ */
	{0x0F12, 0x0064,},/* AfitBaseVals_4__9_ */
	{0x0F12, 0x000C,},/* AfitBaseVals_4__10_ */
	{0x0F12, 0x0010,},/* AfitBaseVals_4__11_ */
	{0x0F12, 0x01E6,},/* AfitBaseVals_4__12_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_4__13_ */
	{0x0F12, 0x0070,},/* AfitBaseVals_4__14_ */
	{0x0F12, 0x0087,},/* AfitBaseVals_4__15_ */
	{0x0F12, 0x0073,},/* AfitBaseVals_4__16_ */
	{0x0F12, 0x0032,},/* AfitBaseVals_4__17_ */
	{0x0F12, 0x0096,},/* AfitBaseVals_4__18_ */
	{0x0F12, 0x0073,},/* AfitBaseVals_4__19_ */
	{0x0F12, 0x00B4,},/* AfitBaseVals_4__20_ */
	{0x0F12, 0x0028,},/* AfitBaseVals_4__21_ */
	{0x0F12, 0x0028,},/* AfitBaseVals_4__22_ */
	{0x0F12, 0x0023,},/* AfitBaseVals_4__23_ */
	{0x0F12, 0x0046,},/* AfitBaseVals_4__24_ */
	{0x0F12, 0x0028,},/* AfitBaseVals_4__25_ */
	{0x0F12, 0x0028,},/* AfitBaseVals_4__26_ */
	{0x0F12, 0x0023,},/* AfitBaseVals_4__27_ */
	{0x0F12, 0x0046,},/* AfitBaseVals_4__28_ */
	{0x0F12, 0x2B23,},/* AfitBaseVals_4__29_ */
	{0x0F12, 0x0601,},/* AfitBaseVals_4__30_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_4__31_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_4__32_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_4__33_ */
	{0x0F12, 0x00FF,},/* AfitBaseVals_4__34_ */
	{0x0F12, 0x0B84,},/* AfitBaseVals_4__35_ */
	{0x0F12, 0xFFFF,},/* AfitBaseVals_4__36_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_4__37_ */
	{0x0F12, 0x050D,},/* AfitBaseVals_4__38_ */
	{0x0F12, 0x1E80,},/* AfitBaseVals_4__39_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_4__40_ */
	{0x0F12, 0x0A08,},/* AfitBaseVals_4__41_ */
	{0x0F12, 0x0200,},/* AfitBaseVals_4__42_ */
	{0x0F12, 0xFF01,},/* AfitBaseVals_4__43_ */
	{0x0F12, 0x180F,},/* AfitBaseVals_4__44_ */
	{0x0F12, 0x0001,},/* AfitBaseVals_4__45_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_4__46_ */
	{0x0F12, 0x8003,},/* AfitBaseVals_4__47_ */
	{0x0F12, 0x008C,},/* AfitBaseVals_4__48_ */
	{0x0F12, 0x0080,},/* AfitBaseVals_4__49_ */
	{0x0F12, 0x0180,},/* AfitBaseVals_4__50_ */
	{0x0F12, 0x0108,},/* AfitBaseVals_4__51_ */
	{0x0F12, 0x1E1E,},/* AfitBaseVals_4__52_ */
	{0x0F12, 0x1419,},/* AfitBaseVals_4__53_ */
	{0x0F12, 0x0A0A,},/* AfitBaseVals_4__54_ */
	{0x0F12, 0x0800,},/* AfitBaseVals_4__55_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_4__56_ */
	{0x0F12, 0x0328,},/* AfitBaseVals_4__57_ */
	{0x0F12, 0x324C,},/* AfitBaseVals_4__58_ */
	{0x0F12, 0x001E,},/* AfitBaseVals_4__59_ */
	{0x0F12, 0x0200,},/* AfitBaseVals_4__60_ */
	{0x0F12, 0x0103,},/* AfitBaseVals_4__61_ */
	{0x0F12, 0x010C,},/* AfitBaseVals_4__62_ */
	{0x0F12, 0x6464,},/* AfitBaseVals_4__63_ */
	{0x0F12, 0x0F0F,},/* AfitBaseVals_4__64_ */
	{0x0F12, 0x0307,},/* AfitBaseVals_4__65_ */
	{0x0F12, 0x080F,},/* AfitBaseVals_4__66_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_4__67_ */
	{0x0F12, 0x030F,},/* AfitBaseVals_4__68_ */
	{0x0F12, 0x3208,},/* AfitBaseVals_4__69_ */
	{0x0F12, 0x0F1E,},/* AfitBaseVals_4__70_ */
	{0x0F12, 0x020F,},/* AfitBaseVals_4__71_ */
	{0x0F12, 0x0103,},/* AfitBaseVals_4__72_ */
	{0x0F12, 0x010C,},/* AfitBaseVals_4__73_ */
	{0x0F12, 0x6464,},/* AfitBaseVals_4__74_ */
	{0x0F12, 0x0F0F,},/* AfitBaseVals_4__75_ */
	{0x0F12, 0x0307,},/* AfitBaseVals_4__76_ */
	{0x0F12, 0x080F,},/* AfitBaseVals_4__77_ */
	{0x0F12, 0x0000,},/* AfitBaseVals_4__78_ */
	{0x0F12, 0x030F,},/* AfitBaseVals_4__79_ */
	{0x0F12, 0x3208,},/* AfitBaseVals_4__80_ */
	{0x0F12, 0x0F1E,},/* AfitBaseVals_4__81_ */
	{0x0F12, 0x020F,},/* AfitBaseVals_4__82_ */
	{0x0F12, 0x0003,},/* AfitBaseVals_4__83_ */
	{0x0F12, 0x7F5E,},/* ConstAfitBaseVals_0_ */
	{0x0F12, 0xFEEE,},/* ConstAfitBaseVals_1_ */
	{0x0F12, 0xD9B7,},/* ConstAfitBaseVals_2_ */
	{0x0F12, 0x0472,},/* ConstAfitBaseVals_3_ */
	{0x0F12, 0x0001,},/* ConstAfitBaseVals_4_ */
	{0x002A, 0x01CA,},/* reserved */
	{0x0F12, 0x0500,},/* REG_0TC_PCFG_Cfg_Input_Sizes_usWidth  */
	{0x0F12, 0x03C0,},/* REG_0TC_PCFG_Cfg_Input_Sizes_usHeight */
	{0x0F12, 0x0000,},/* REG_0TC_PCFG_Cfg_Input_Ofs_usWidth */
	{0x0F12, 0x0000,},/* REG_0TC_PCFG_Cfg_Input_Ofs_usHeight */
	{0x002A, 0x02BC,},/* reserved */
	{0x0F12, 0x0500,},/* REG_0TC_CCFG_Cfg_Input_Sizes_usWidth  */
	{0x0F12, 0x03C0,},/* REG_0TC_CCFG_Cfg_Input_Sizes_usHeight */
	{0x0F12, 0x0000,},/* REG_0TC_CCFG_Cfg_Input_Ofs_usWidth */
	{0x0F12, 0x0000,},/* REG_0TC_CCFG_Cfg_Input_Ofs_usHeight */
	{0x002A, 0x01BE,},/* reserved */
	{0x0F12, 0x0500,},/* REG_0TC_PCFG_usWidth */
	{0x0F12, 0x03C0,},/* REG_0TC_PCFG_usHeight */
	{0x0F12, 0x0005,},/* REG_0TC_PCFG_Format */
	{0x002A, 0x01C8,},/* reserved */
	{0x0F12, 0x0000,},/* REG_0TC_PCFG_uClockInd */
	{0x002A, 0x01C4,},/* reserved */
	{0x0F12, 0x0042,},/* REG_0TC_PCFG_PVIMask */
	{0x002A, 0x01D4,},/* reserved */
	{0x0F12, 0x0002,},/* REG_0TC_PCFG_FrRateQualityType */
	{0x002A, 0x01D2,},/* reserved */
	{0x0F12, 0x0001,},/* REG_0TC_PCFG_usFrTimeType */
	{0x002A, 0x01D8,},/* reserved */
	{0x0F12, 0x014D,},/* REG_0TC_PCFG_usMaxFrTimeMsecMult10 */
	{0x002A, 0x01D6,},/* reserved */
	{0x0F12, 0x0000,},/* REG_0TC_PCFG_usMinFrTimeMsecMult10 */
	{0x002A, 0x01E8,},/* reserved */
	{0x0F12, 0x0033,},/* REG_0TC_PCFG_uPrevMirror */
	{0x002A, 0x02B0,},/* reserved */
	{0x0F12, 0x0500,},/* REG_0TC_CCFG_usWidth */
	{0x0F12, 0x03C0,},/* REG_0TC_CCFG_usHeight */
	{0x0F12, 0x0005,},/* REG_0TC_CCFG_Format */
	{0x002A, 0x02BA,},/* reserved */
	{0x0F12, 0x0000,},/* REG_0TC_CCFG_uClockInd */
	{0x002A, 0x02B6,},/* reserved */
	{0x0F12, 0x0042,},/* REG_0TC_CCFG_PVIMask */
	{0x002A, 0x02C6,},/* reserved */
	{0x0F12, 0x0002,},/* REG_0TC_CCFG_FrRateQualityType */
	{0x002A, 0x02C4,},/* reserved */
	{0x0F12, 0x0001,},/* REG_0TC_CCFG_usFrTimeType */
	{0x002A, 0x02CA,},/* reserved */
	{0x0F12, 0x014D,},/* REG_0TC_CCFG_usMaxFrTimeMsecMult10 */
	{0x002A, 0x02C8,},/* reserved */
	{0x0F12, 0x0000,},/* REG_0TC_CCFG_usMinFrTimeMsecMult10 */
	{0x002A, 0x01A8,},/* reserved */
	{0x0F12, 0x0000,},/* REG_TC_GP_ActivePrevConfig */
	{0x002A, 0x01B0,},/* reserved */
	{0x0F12, 0x0000,},/* REG_TC_GP_ActiveCapConfig */
};

static struct msm_camera_i2c_reg_conf s5k8aay_config_exposure_settings[S5K8AAY_EXPOSURE_TBL_SIZE][S5K8AAY_EXPOSURE_SETTING_SIZE] = {
	/* -6 */
	{
		{0xFCFC, 0xD000},
		{0x0028, 0x7000},
		{0x002A, 0x018E},
		{0x0F12, 0xFF94},
	},
	/* -5 */
	{
		{0xFCFC, 0xD000},
		{0x0028, 0x7000},
		{0x002A, 0x018E},
		{0x0F12, 0xFFA6},
	},
	/* -4 */
	{
		{0xFCFC, 0xD000},
		{0x0028, 0x7000},
		{0x002A, 0x018E},
		{0x0F12, 0xFFB8},
	},
	/* -3 */
	{
		{0xFCFC, 0xD000},
		{0x0028, 0x7000},
		{0x002A, 0x018E},
		{0x0F12, 0xFFCA},
	},
	/* -2 */
	{
		{0xFCFC, 0xD000},
		{0x0028, 0x7000},
		{0x002A, 0x018E},
		{0x0F12, 0xFFDC},
	},
	/* -1 */
	{
		{0xFCFC, 0xD000},
		{0x0028, 0x7000},
		{0x002A, 0x018E},
		{0x0F12, 0xFFEE},
	},
	/* 0 */
	{
		{0xFCFC, 0xD000},
		{0x0028, 0x7000},
		{0x002A, 0x018E},
		{0x0F12, 0x0000},
	},
	/* 1 */
	{
		{0xFCFC, 0xD000},
		{0x0028, 0x7000},
		{0x002A, 0x018E},
		{0x0F12, 0x0012},
	},
	/* 2 */
	{
		{0xFCFC, 0xD000},
		{0x0028, 0x7000},
		{0x002A, 0x018E},
		{0x0F12, 0x0024},
	},
	/* 3 */
	{
		{0xFCFC, 0xD000},
		{0x0028, 0x7000},
		{0x002A, 0x018E},
		{0x0F12, 0x0036},
	},
	/* 4 */
	{
		{0xFCFC, 0xD000},
		{0x0028, 0x7000},
		{0x002A, 0x018E},
		{0x0F12, 0x0048},
	},
	/* 5 */
	{
		{0xFCFC, 0xD000},
		{0x0028, 0x7000},
		{0x002A, 0x018E},
		{0x0F12, 0x005A},
	},
	/* 6 */
	{
		{0xFCFC, 0xD000},
		{0x0028, 0x7000},
		{0x002A, 0x018E},
		{0x0F12, 0x006C},
	},
};

static struct msm_camera_i2c_reg_conf s5k8aay_config_white_balance_settings[S5K8AAY_WB_MAX][S5K8AAY_WB_SETTING_SIZE] = {
	/* S5K8AAY_WB_AUTO */
	{
		{0xFCFC, 0xD000},
		{0x0028, 0x7000},
		{0x002A, 0x2162},
		{0x0F12, 0x0001},
		{0x002A, 0x03DA},
		{0x0F12, 0x0000},
		{0x0F12, 0x0000},
		{0x0F12, 0x0000},
		{0x0F12, 0x0000},
		{0x0F12, 0x0000},
		{0x0F12, 0x0000},
	},
	/* S5K8AAY_WB_DAYLIGHT */
	{
		{0xFCFC, 0xD000},
		{0x0028, 0x7000},
		{0x002A, 0x2162},
		{0x0F12, 0x0000},
		{0x002A, 0x03DA},
		{0x0F12, 0x05D0},
		{0x0F12, 0x0001},
		{0x0F12, 0x0400},
		{0x0F12, 0x0001},
		{0x0F12, 0x05A0},
		{0x0F12, 0x0001},
	},
	/* S5K8AAY_WB_FLUORESCENT */
	{
		{0xFCFC, 0xD000},
		{0x0028, 0x7000},
		{0x002A, 0x2162},
		{0x0F12, 0x0000},
		{0x002A, 0x03DA},
		{0x0F12, 0x04C8},
		{0x0F12, 0x0001},
		{0x0F12, 0x0400},
		{0x0F12, 0x0001},
		{0x0F12, 0x0818},
		{0x0F12, 0x0001},
	},
	/* S5K8AAY_WB_WARM_FLUORESCENT */
	{
		{0xFCFC, 0xD000},
		{0x0028, 0x7000},
		{0x002A, 0x2162},
		{0x0F12, 0x0000},
		{0x002A, 0x03DA},
		{0x0F12, 0x0500},
		{0x0F12, 0x0001},
		{0x0F12, 0x0400},
		{0x0F12, 0x0001},
		{0x0F12, 0x0600},
		{0x0F12, 0x0001},
	},
	/* S5K8AAY_WB_INCANDESCENT */
	{
		{0xFCFC, 0xD000},
		{0x0028, 0x7000},
		{0x002A, 0x2162},
		{0x0F12, 0x0000},
		{0x002A, 0x03DA},
		{0x0F12, 0x0400},
		{0x0F12, 0x0001},
		{0x0F12, 0x04B1},
		{0x0F12, 0x0001},
		{0x0F12, 0x0C30},
		{0x0F12, 0x0001},
	},
};

static struct msm_camera_i2c_reg_conf s5k8aay_config_effect_settings[S5K8AAY_EFFECT_MAX][S5K8AAY_EFFECT_SETTING_SIZE] = {
	/* S5K8AAY_EFFECT_OFF */
	{
		{0xFCFC, 0xD000},
		{0x0028, 0x7000},
		{0x002A, 0x019C},
		{0x0F12, 0x0000},
		{0xFCFC, 0xD000},
		{0x0028, 0x7000},
		{0x002A, 0x0192},
		{0x0F12, 0x0000},
	},
	/* S5K8AAY_EFFECT_MONO */
	{
		{0xFCFC, 0xD000},
		{0x0028, 0x7000},
		{0x002A, 0x019C},
		{0x0F12, 0x0001},
		{0xFCFC, 0xD000},
		{0x0028, 0x7000},
		{0x002A, 0x0192},
		{0x0F12, 0x0000},
	},
	/* S5K8AAY_EFFECT_SEPIA */
	{
		{0xFCFC, 0xD000},
		{0x0028, 0x7000},
		{0x002A, 0x019C},
		{0x0F12, 0x0004},
		{0xFCFC, 0xD000},
		{0x0028, 0x7000},
		{0x002A, 0x0192},
		{0x0F12, 0x0000},
	},
	/* S5K8AAY_EFFECT_NEGATIVE */
	{
		{0xFCFC, 0xD000},
		{0x0028, 0x7000},
		{0x002A, 0x019C},
		{0x0F12, 0x0003},
		{0xFCFC, 0xD000},
		{0x0028, 0x7000},
		{0x002A, 0x0192},
		{0x0F12, 0x0000},
	},
};

static struct msm_camera_i2c_reg_conf s5k8aay_start_stream_settings[] = {
	{0xFCFC, 0xD000,},
	{0x0028, 0x7000,},
	{0x002A, 0x019E,},
	{0x0F12, 0x0001,},
	{0x002A, 0x01A0,},
	{0x0F12, 0x0001,},
};
static struct msm_camera_i2c_reg_conf s5k8aay_stop_stream_settings[] = {
	{0xFCFC, 0xD000,},
	{0x0028, 0x7000,},
	{0x002A, 0x019E,},
	{0x0F12, 0x0000,},
	{0x002A, 0x01A0,},
	{0x0F12, 0x0001,},

};

static struct msm_camera_i2c_reg_conf s5k8aay_id_read_settings[] = {
	{0xFCFC, 0xD000},
	{0x002C, 0x0000},
	{0x002E, 0x0040},
};

static struct msm_camera_i2c_reg_conf s5k8aay_exp_time_settings[] = {
	{0xFCFC, 0xD000},
	{0x002C, 0x7000},
	{0x002E, 0x210C},
};

static struct msm_camera_i2c_reg_conf s5k8aay_error_check_write_reg[] = {
	{0xFCFC, 0xD000},
	{0x0028, 0x7000},
	{0x002A, 0x0108},
	{0x0F12, 0x0000},
	{0x0028, 0xD000},
	{0x002A, 0x1002},
	{0x0F12, 0x0000},
};
static struct msm_camera_i2c_reg_conf s5k8aay_error_check_reg_1[] = {
	{0xFCFC, 0xD000},
	{0x002C, 0x7000},
	{0x002E, 0x0108},
};
static struct msm_camera_i2c_reg_conf s5k8aay_error_check_reg_2[] = {
	{0xFCFC, 0xD000},
	{0x002C, 0xD000},
	{0x002E, 0x1002},
};

#if 0
static struct msm_camera_i2c_reg_conf s5k8aay_resolution_settings[S5K8AAY_RESOLUTION_TBL_SIZE][S5K8AAY_RESOLUTION_TBL_REG_SIZE] = {
	{
		/* SXGA(1280x960) */
		{0xFCFC, 0xD000},
		{0x0028, 0x7000},
		{0x002A, 0x01CA},
		{0x0F12, 0x0500},
		{0x0F12, 0x03C0},
		{0x0F12, 0x0000},
		{0x0F12, 0x0000},
		{0x002A, 0x02BC},
		{0x0F12, 0x0500},
		{0x0F12, 0x03C0},
		{0x0F12, 0x0000},
		{0x0F12, 0x0000},
		{0x002A, 0x01BE},
		{0x0F12, 0x0500},
		{0x0F12, 0x03C0},
		{0x0F12, 0x0005},
		{0x002A, 0x01C8},
		{0x0F12, 0x0000},
		{0x002A, 0x01C4},
		{0x0F12, 0x0042},
		{0x002A, 0x01D4},
		{0x0F12, 0x0002},
		{0x002A, 0x01D2},
		{0x0F12, 0x0001},
		{0x002A, 0x01D8},
		{0x0F12, 0x014D},
		{0x002A, 0x01D6},
		{0x0F12, 0x0000},
		{0x002A, 0x01E8},
		{0x0F12, 0x0033},
		{0x002A, 0x02B0},
		{0x0F12, 0x0500},
		{0x0F12, 0x03C0},
		{0x0F12, 0x0005},
		{0x002A, 0x02BA},
		{0x0F12, 0x0000},
		{0x002A, 0x02B6},
		{0x0F12, 0x0042},
		{0x002A, 0x02C6},
		{0x0F12, 0x0002},
		{0x002A, 0x02C4},
		{0x0F12, 0x0001},
		{0x002A, 0x02CA},
		{0x0F12, 0x014D},
		{0x002A, 0x02C8},
		{0x0F12, 0x0000},
		{0x002A, 0x01A8},
		{0x0F12, 0x0000},
		{0x0F12, 0x0001},
		{0x002A, 0x01B0},
		{0x0F12, 0x0000},
		{0x002A, 0x019E},
		{0x0F12, 0x0001},
		{0x002A, 0x01A0},
		{0x0F12, 0x0001},
	},
	{
		/* VGA(640x480) */
		{0xFCFC, 0xD000},
		{0x0028, 0x7000},
		{0x002A, 0x01CA},
		{0x0F12, 0x0500},
		{0x0F12, 0x03C0},
		{0x0F12, 0x0000},
		{0x0F12, 0x0000},
		{0x002A, 0x02BC},
		{0x0F12, 0x0500},
		{0x0F12, 0x03C0},
		{0x0F12, 0x0000},
		{0x0F12, 0x0000},
		{0x002A, 0x01BE},
		{0x0F12, 0x0280},
		{0x0F12, 0x01E0},
		{0x0F12, 0x0005},
		{0x002A, 0x01C8},
		{0x0F12, 0x0000},
		{0x002A, 0x01C4},
		{0x0F12, 0x0042},
		{0x002A, 0x01D4},
		{0x0F12, 0x0002},
		{0x002A, 0x01D2},
		{0x0F12, 0x0001},
		{0x002A, 0x01D8},
		{0x0F12, 0x014D},
		{0x002A, 0x01D6},
		{0x0F12, 0x0000},
		{0x002A, 0x01E8},
		{0x0F12, 0x0033},
		{0x002A, 0x02B0},
		{0x0F12, 0x0280},
		{0x0F12, 0x01E0},
		{0x0F12, 0x0005},
		{0x002A, 0x02BA},
		{0x0F12, 0x0000},
		{0x002A, 0x02B6},
		{0x0F12, 0x0042},
		{0x002A, 0x02C6},
		{0x0F12, 0x0002},
		{0x002A, 0x02C4},
		{0x0F12, 0x0001},
		{0x002A, 0x02CA},
		{0x0F12, 0x014D},
		{0x002A, 0x02C8},
		{0x0F12, 0x0000},
		{0x002A, 0x01A8},
		{0x0F12, 0x0000},
		{0x0F12, 0x0001},
		{0x002A, 0x01B0},
		{0x0F12, 0x0000},
		{0x002A, 0x019E},
		{0x0F12, 0x0001},
		{0x002A, 0x01A0},
		{0x0F12, 0x0001},
	},
	{
		/* WVGA(800x480) */
		{0xFCFC, 0xD000},
		{0x0028, 0x7000},
		{0x002A, 0x01CA},
		{0x0F12, 0x0500},
		{0x0F12, 0x0300},
		{0x0F12, 0x0000},
		{0x0F12, 0x0060},
		{0x002A, 0x02BC},
		{0x0F12, 0x0500},
		{0x0F12, 0x0300},
		{0x0F12, 0x0000},
		{0x0F12, 0x0060},
		{0x002A, 0x01BE},
		{0x0F12, 0x0320},
		{0x0F12, 0x01E0},
		{0x0F12, 0x0005},
		{0x002A, 0x01C8},
		{0x0F12, 0x0000},
		{0x002A, 0x01C4},
		{0x0F12, 0x0042},
		{0x002A, 0x01D4},
		{0x0F12, 0x0002},
		{0x002A, 0x01D2},
		{0x0F12, 0x0001},
		{0x002A, 0x01D8},
		{0x0F12, 0x014D},
		{0x002A, 0x01D6},
		{0x0F12, 0x0000},
		{0x002A, 0x01E8},
		{0x0F12, 0x0033},
		{0x002A, 0x02B0},
		{0x0F12, 0x0320},
		{0x0F12, 0x01E0},
		{0x0F12, 0x0005},
		{0x002A, 0x02BA},
		{0x0F12, 0x0000},
		{0x002A, 0x02B6},
		{0x0F12, 0x0042},
		{0x002A, 0x02C6},
		{0x0F12, 0x0002},
		{0x002A, 0x02C4},
		{0x0F12, 0x0001},
		{0x002A, 0x02CA},
		{0x0F12, 0x014D},
		{0x002A, 0x02C8},
		{0x0F12, 0x0000},
		{0x002A, 0x01A8},
		{0x0F12, 0x0000},
		{0x0F12, 0x0001},
		{0x002A, 0x01B0},
		{0x0F12, 0x0000},
		{0x002A, 0x019E},
		{0x0F12, 0x0001},
		{0x002A, 0x01A0},
		{0x0F12, 0x0001},
	},
	{
		/* Display(960x720) */
		{0xFCFC, 0xD000},
		{0x0028, 0x7000},
		{0x002A, 0x01CA},
		{0x0F12, 0x0500},
		{0x0F12, 0x03C0},
		{0x0F12, 0x0000},
		{0x0F12, 0x0000},
		{0x002A, 0x02BC},
		{0x0F12, 0x0500},
		{0x0F12, 0x03C0},
		{0x0F12, 0x0000},
		{0x0F12, 0x0000},
		{0x002A, 0x01BE},
		{0x0F12, 0x03C0},
		{0x0F12, 0x02D0},
		{0x0F12, 0x0005},
		{0x002A, 0x01C8},
		{0x0F12, 0x0000},
		{0x002A, 0x01C4},
		{0x0F12, 0x0042},
		{0x002A, 0x01D4},
		{0x0F12, 0x0002},
		{0x002A, 0x01D2},
		{0x0F12, 0x0001},
		{0x002A, 0x01D8},
		{0x0F12, 0x014D},
		{0x002A, 0x01D6},
		{0x0F12, 0x0000},
		{0x002A, 0x01E8},
		{0x0F12, 0x0033},
		{0x002A, 0x02B0},
		{0x0F12, 0x03C0},
		{0x0F12, 0x02D0},
		{0x0F12, 0x0005},
		{0x002A, 0x02BA},
		{0x0F12, 0x0000},
		{0x002A, 0x02B6},
		{0x0F12, 0x0042},
		{0x002A, 0x02C6},
		{0x0F12, 0x0002},
		{0x002A, 0x02C4},
		{0x0F12, 0x0001},
		{0x002A, 0x02CA},
		{0x0F12, 0x014D},
		{0x002A, 0x02C8},
		{0x0F12, 0x0000},
		{0x002A, 0x01A8},
		{0x0F12, 0x0000},
		{0x0F12, 0x0001},
		{0x002A, 0x01B0},
		{0x0F12, 0x0000},
		{0x002A, 0x019E},
		{0x0F12, 0x0001},
		{0x002A, 0x01A0},
		{0x0F12, 0x0001},
	},
	{
		/* HD(1280x720) */
		{0xFCFC, 0xD000},
		{0x0028, 0x7000},
		{0x002A, 0x01CA},
		{0x0F12, 0x0500},
		{0x0F12, 0x02D0},
		{0x0F12, 0x0000},
		{0x0F12, 0x0078},
		{0x002A, 0x02BC},
		{0x0F12, 0x0500},
		{0x0F12, 0x02D0},
		{0x0F12, 0x0000},
		{0x0F12, 0x0078},
		{0x002A, 0x01BE},
		{0x0F12, 0x0500},
		{0x0F12, 0x02D0},
		{0x0F12, 0x0005},
		{0x002A, 0x01C8},
		{0x0F12, 0x0000},
		{0x002A, 0x01C4},
		{0x0F12, 0x0042},
		{0x002A, 0x01D4},
		{0x0F12, 0x0002},
		{0x002A, 0x01D2},
		{0x0F12, 0x0001},
		{0x002A, 0x01D8},
		{0x0F12, 0x014D},
		{0x002A, 0x01D6},
		{0x0F12, 0x0000},
		{0x002A, 0x01E8},
		{0x0F12, 0x0033},
		{0x002A, 0x02B0},
		{0x0F12, 0x0500},
		{0x0F12, 0x02D0},
		{0x0F12, 0x0005},
		{0x002A, 0x02BA},
		{0x0F12, 0x0000},
		{0x002A, 0x02B6},
		{0x0F12, 0x0042},
		{0x002A, 0x02C6},
		{0x0F12, 0x0002},
		{0x002A, 0x02C4},
		{0x0F12, 0x0001},
		{0x002A, 0x02CA},
		{0x0F12, 0x014D},
		{0x002A, 0x02C8},
		{0x0F12, 0x0000},
		{0x002A, 0x01A8},
		{0x0F12, 0x0000},
		{0x0F12, 0x0001},
		{0x002A, 0x01B0},
		{0x0F12, 0x0000},
		{0x002A, 0x019E},
		{0x0F12, 0x0001},
		{0x002A, 0x01A0},
		{0x0F12, 0x0001},
	},
	{
		/* QCIF VIDEO(208x176) */
		{0xFCFC, 0xD000},
		{0x0028, 0x7000},
		{0x002A, 0x01CA},
		{0x0F12, 0x0496},
		{0x0F12, 0x03C0},
		{0x0F12, 0x0035},
		{0x0F12, 0x0000},
		{0x002A, 0x02BC},
		{0x0F12, 0x0496},
		{0x0F12, 0x03C0},
		{0x0F12, 0x0035},
		{0x0F12, 0x0000},
		{0x002A, 0x01BE},
		{0x0F12, 0x00D0},
		{0x0F12, 0x00B0},
		{0x0F12, 0x0005},
		{0x002A, 0x01C8},
		{0x0F12, 0x0000},
		{0x002A, 0x01C4},
		{0x0F12, 0x0042},
		{0x002A, 0x01D4},
		{0x0F12, 0x0002},
		{0x002A, 0x01D2},
		{0x0F12, 0x0001},
		{0x002A, 0x01D8},
		{0x0F12, 0x014D},
		{0x002A, 0x01D6},
		{0x0F12, 0x0000},
		{0x002A, 0x01E8},
		{0x0F12, 0x0033},
		{0x002A, 0x02B0},
		{0x0F12, 0x00D0},
		{0x0F12, 0x00B0},
		{0x0F12, 0x0005},
		{0x002A, 0x02BA},
		{0x0F12, 0x0000},
		{0x002A, 0x02B6},
		{0x0F12, 0x0042},
		{0x002A, 0x02C6},
		{0x0F12, 0x0002},
		{0x002A, 0x02C4},
		{0x0F12, 0x0001},
		{0x002A, 0x02CA},
		{0x0F12, 0x014D},
		{0x002A, 0x02C8},
		{0x0F12, 0x0000},
		{0x002A, 0x01A8},
		{0x0F12, 0x0000},
		{0x0F12, 0x0001},
		{0x002A, 0x01B0},
		{0x0F12, 0x0000},
		{0x002A, 0x019E},
		{0x0F12, 0x0001},
		{0x002A, 0x01A0},
		{0x0F12, 0x0001},
	},
	{
		/* QVGA VIDEO(384x288) */
		{0xFCFC, 0xD000},
		{0x0028, 0x7000},
		{0x002A, 0x01CA},
		{0x0F12, 0x0500},
		{0x0F12, 0x03C0},
		{0x0F12, 0x0000},
		{0x0F12, 0x0000},
		{0x002A, 0x02BC},
		{0x0F12, 0x0500},
		{0x0F12, 0x03C0},
		{0x0F12, 0x0000},
		{0x0F12, 0x0000},
		{0x002A, 0x01BE},
		{0x0F12, 0x0180},
		{0x0F12, 0x0120},
		{0x0F12, 0x0005},
		{0x002A, 0x01C8},
		{0x0F12, 0x0000},
		{0x002A, 0x01C4},
		{0x0F12, 0x0042},
		{0x002A, 0x01D4},
		{0x0F12, 0x0002},
		{0x002A, 0x01D2},
		{0x0F12, 0x0001},
		{0x002A, 0x01D8},
		{0x0F12, 0x014D},
		{0x002A, 0x01D6},
		{0x0F12, 0x0000},
		{0x002A, 0x01E8},
		{0x0F12, 0x0033},
		{0x002A, 0x02B0},
		{0x0F12, 0x0180},
		{0x0F12, 0x0120},
		{0x0F12, 0x0005},
		{0x002A, 0x02BA},
		{0x0F12, 0x0000},
		{0x002A, 0x02B6},
		{0x0F12, 0x0042},
		{0x002A, 0x02C6},
		{0x0F12, 0x0002},
		{0x002A, 0x02C4},
		{0x0F12, 0x0001},
		{0x002A, 0x02CA},
		{0x0F12, 0x014D},
		{0x002A, 0x02C8},
		{0x0F12, 0x0000},
		{0x002A, 0x01A8},
		{0x0F12, 0x0000},
		{0x0F12, 0x0001},
		{0x002A, 0x01B0},
		{0x0F12, 0x0000},
		{0x002A, 0x019E},
		{0x0F12, 0x0001},
		{0x002A, 0x01A0},
		{0x0F12, 0x0001},
	},
	{
		/* VGA VIDEO(768x576) */
		{0xFCFC, 0xD000},
		{0x0028, 0x7000},
		{0x002A, 0x01CA},
		{0x0F12, 0x0500},
		{0x0F12, 0x03C0},
		{0x0F12, 0x0000},
		{0x0F12, 0x0000},
		{0x002A, 0x02BC},
		{0x0F12, 0x0500},
		{0x0F12, 0x03C0},
		{0x0F12, 0x0000},
		{0x0F12, 0x0000},
		{0x002A, 0x01BE},
		{0x0F12, 0x0300},
		{0x0F12, 0x0240},
		{0x0F12, 0x0005},
		{0x002A, 0x01C8},
		{0x0F12, 0x0000},
		{0x002A, 0x01C4},
		{0x0F12, 0x0042},
		{0x002A, 0x01D4},
		{0x0F12, 0x0002},
		{0x002A, 0x01D2},
		{0x0F12, 0x0001},
		{0x002A, 0x01D8},
		{0x0F12, 0x014D},
		{0x002A, 0x01D6},
		{0x0F12, 0x0000},
		{0x002A, 0x01E8},
		{0x0F12, 0x0033},
		{0x002A, 0x02B0},
		{0x0F12, 0x0300},
		{0x0F12, 0x0240},
		{0x0F12, 0x0005},
		{0x002A, 0x02BA},
		{0x0F12, 0x0000},
		{0x002A, 0x02B6},
		{0x0F12, 0x0042},
		{0x002A, 0x02C6},
		{0x0F12, 0x0002},
		{0x002A, 0x02C4},
		{0x0F12, 0x0001},
		{0x002A, 0x02CA},
		{0x0F12, 0x014D},
		{0x002A, 0x02C8},
		{0x0F12, 0x0000},
		{0x002A, 0x01A8},
		{0x0F12, 0x0000},
		{0x0F12, 0x0001},
		{0x002A, 0x01B0},
		{0x0F12, 0x0000},
		{0x002A, 0x019E},
		{0x0F12, 0x0001},
		{0x002A, 0x01A0},
		{0x0F12, 0x0001},
	},
	{
		/* WVGA VIDEO(960x576) */
		{0xFCFC, 0xD000},
		{0x0028, 0x7000},
		{0x002A, 0x01CA},
		{0x0F12, 0x0500},
		{0x0F12, 0x0300},
		{0x0F12, 0x0000},
		{0x0F12, 0x0060},
		{0x002A, 0x02BC},
		{0x0F12, 0x0500},
		{0x0F12, 0x0300},
		{0x0F12, 0x0000},
		{0x0F12, 0x0060},
		{0x002A, 0x01BE},
		{0x0F12, 0x03C0},
		{0x0F12, 0x0240},
		{0x0F12, 0x0005},
		{0x002A, 0x01C8},
		{0x0F12, 0x0000},
		{0x002A, 0x01C4},
		{0x0F12, 0x0042},
		{0x002A, 0x01D4},
		{0x0F12, 0x0002},
		{0x002A, 0x01D2},
		{0x0F12, 0x0001},
		{0x002A, 0x01D8},
		{0x0F12, 0x014D},
		{0x002A, 0x01D6},
		{0x0F12, 0x0000},
		{0x002A, 0x01E8},
		{0x0F12, 0x0033},
		{0x002A, 0x02B0},
		{0x0F12, 0x03C0},
		{0x0F12, 0x0240},
		{0x0F12, 0x0005},
		{0x002A, 0x02BA},
		{0x0F12, 0x0000},
		{0x002A, 0x02B6},
		{0x0F12, 0x0042},
		{0x002A, 0x02C6},
		{0x0F12, 0x0002},
		{0x002A, 0x02C4},
		{0x0F12, 0x0001},
		{0x002A, 0x02CA},
		{0x0F12, 0x014D},
		{0x002A, 0x02C8},
		{0x0F12, 0x0000},
		{0x002A, 0x01A8},
		{0x0F12, 0x0000},
		{0x0F12, 0x0001},
		{0x002A, 0x01B0},
		{0x0F12, 0x0000},
		{0x002A, 0x019E},
		{0x0F12, 0x0001},
		{0x002A, 0x01A0},
		{0x0F12, 0x0001},
	},
	{
		/* diaplay VIDEO(1152x864) */
		{0xFCFC, 0xD000},
		{0x0028, 0x7000},
		{0x002A, 0x01CA},
		{0x0F12, 0x0500},
		{0x0F12, 0x03C0},
		{0x0F12, 0x0000},
		{0x0F12, 0x0000},
		{0x002A, 0x02BC},
		{0x0F12, 0x0500},
		{0x0F12, 0x03C0},
		{0x0F12, 0x0000},
		{0x0F12, 0x0000},
		{0x002A, 0x01BE},
		{0x0F12, 0x0480},
		{0x0F12, 0x0360},
		{0x0F12, 0x0005},
		{0x002A, 0x01C8},
		{0x0F12, 0x0000},
		{0x002A, 0x01C4},
		{0x0F12, 0x0042},
		{0x002A, 0x01D4},
		{0x0F12, 0x0002},
		{0x002A, 0x01D2},
		{0x0F12, 0x0001},
		{0x002A, 0x01D8},
		{0x0F12, 0x014D},
		{0x002A, 0x01D6},
		{0x0F12, 0x0000},
		{0x002A, 0x01E8},
		{0x0F12, 0x0033},
		{0x002A, 0x02B0},
		{0x0F12, 0x0480},
		{0x0F12, 0x0360},
		{0x0F12, 0x0005},
		{0x002A, 0x02BA},
		{0x0F12, 0x0000},
		{0x002A, 0x02B6},
		{0x0F12, 0x0042},
		{0x002A, 0x02C6},
		{0x0F12, 0x0002},
		{0x002A, 0x02C4},
		{0x0F12, 0x0001},
		{0x002A, 0x02CA},
		{0x0F12, 0x014D},
		{0x002A, 0x02C8},
		{0x0F12, 0x0000},
		{0x002A, 0x01A8},
		{0x0F12, 0x0000},
		{0x0F12, 0x0001},
		{0x002A, 0x01B0},
		{0x0F12, 0x0000},
		{0x002A, 0x019E},
		{0x0F12, 0x0001},
		{0x002A, 0x01A0},
		{0x0F12, 0x0001},
	},
	{
		/* QCIF(176x144) */
		{0xFCFC, 0xD000},
		{0x0028, 0x7000},
		{0x002A, 0x01CA},
		{0x0F12, 0x0496},
		{0x0F12, 0x03C0},
		{0x0F12, 0x0035},
		{0x0F12, 0x0000},
		{0x002A, 0x02BC},
		{0x0F12, 0x0496},
		{0x0F12, 0x03C0},
		{0x0F12, 0x0035},
		{0x0F12, 0x0000},
		{0x002A, 0x01BE},
		{0x0F12, 0x00B0},
		{0x0F12, 0x0090},
		{0x0F12, 0x0005},
		{0x002A, 0x01C8},
		{0x0F12, 0x0000},
		{0x002A, 0x01C4},
		{0x0F12, 0x0042},
		{0x002A, 0x01D4},
		{0x0F12, 0x0002},
		{0x002A, 0x01D2},
		{0x0F12, 0x0001},
		{0x002A, 0x01D8},
		{0x0F12, 0x014D},
		{0x002A, 0x01D6},
		{0x0F12, 0x0000},
		{0x002A, 0x01E8},
		{0x0F12, 0x0033},
		{0x002A, 0x02B0},
		{0x0F12, 0x00B0},
		{0x0F12, 0x0090},
		{0x0F12, 0x0005},
		{0x002A, 0x02BA},
		{0x0F12, 0x0000},
		{0x002A, 0x02B6},
		{0x0F12, 0x0042},
		{0x002A, 0x02C6},
		{0x0F12, 0x0002},
		{0x002A, 0x02C4},
		{0x0F12, 0x0001},
		{0x002A, 0x02CA},
		{0x0F12, 0x0535},
		{0x002A, 0x02C8},
		{0x0F12, 0x0000},
		{0x002A, 0x01A8},
		{0x0F12, 0x0000},
		{0x0F12, 0x0001},
		{0x002A, 0x01B0},
		{0x0F12, 0x0000},
		{0x002A, 0x019E},
		{0x0F12, 0x0001},
		{0x002A, 0x01A0},
		{0x0F12, 0x0001},
	},
	{
		/* QVGA(320x240) */
		{0xFCFC, 0xD000},
		{0x0028, 0x7000},
		{0x002A, 0x01CA},
		{0x0F12, 0x0500},
		{0x0F12, 0x03C0},
		{0x0F12, 0x0000},
		{0x0F12, 0x0000},
		{0x002A, 0x02BC},
		{0x0F12, 0x0500},
		{0x0F12, 0x03C0},
		{0x0F12, 0x0000},
		{0x0F12, 0x0000},
		{0x002A, 0x01BE},
		{0x0F12, 0x0140},
		{0x0F12, 0x00F0},
		{0x0F12, 0x0005},
		{0x002A, 0x01C8},
		{0x0F12, 0x0000},
		{0x002A, 0x01C4},
		{0x0F12, 0x0042},
		{0x002A, 0x01D4},
		{0x0F12, 0x0002},
		{0x002A, 0x01D2},
		{0x0F12, 0x0001},
		{0x002A, 0x01D8},
		{0x0F12, 0x014D},
		{0x002A, 0x01D6},
		{0x0F12, 0x0000},
		{0x002A, 0x01E8},
		{0x0F12, 0x0033},
		{0x002A, 0x02B0},
		{0x0F12, 0x0140},
		{0x0F12, 0x00F0},
		{0x0F12, 0x0005},
		{0x002A, 0x02BA},
		{0x0F12, 0x0000},
		{0x002A, 0x02B6},
		{0x0F12, 0x0042},
		{0x002A, 0x02C6},
		{0x0F12, 0x0002},
		{0x002A, 0x02C4},
		{0x0F12, 0x0001},
		{0x002A, 0x02CA},
		{0x0F12, 0x0535},
		{0x002A, 0x02C8},
		{0x0F12, 0x0000},
		{0x002A, 0x01A8},
		{0x0F12, 0x0000},
		{0x0F12, 0x0001},
		{0x002A, 0x01B0},
		{0x0F12, 0x0000},
		{0x002A, 0x019E},
		{0x0F12, 0x0001},
		{0x002A, 0x01A0},
		{0x0F12, 0x0001},
	},
	{
		/* 1:1(960x960) */
		{0xFCFC, 0xD000},
		{0x0028, 0x7000},
		{0x002A, 0x01CA},
		{0x0F12, 0x03C0},
		{0x0F12, 0x03C0},
		{0x0F12, 0x00A0},
		{0x0F12, 0x0000},
		{0x002A, 0x02BC},
		{0x0F12, 0x03C0},
		{0x0F12, 0x03C0},
		{0x0F12, 0x00A0},
		{0x0F12, 0x0000},
		{0x002A, 0x01BE},
		{0x0F12, 0x03C0},
		{0x0F12, 0x03C0},
		{0x0F12, 0x0005},
		{0x002A, 0x01C8},
		{0x0F12, 0x0000},
		{0x002A, 0x01C4},
		{0x0F12, 0x0042},
		{0x002A, 0x01D4},
		{0x0F12, 0x0002},
		{0x002A, 0x01D2},
		{0x0F12, 0x0001},
		{0x002A, 0x01D8},
		{0x0F12, 0x014D},
		{0x002A, 0x01D6},
		{0x0F12, 0x0000},
		{0x002A, 0x01E8},
		{0x0F12, 0x0033},
		{0x002A, 0x02B0},
		{0x0F12, 0x03C0},
		{0x0F12, 0x03C0},
		{0x0F12, 0x0005},
		{0x002A, 0x02BA},
		{0x0F12, 0x0000},
		{0x002A, 0x02B6},
		{0x0F12, 0x0042},
		{0x002A, 0x02C6},
		{0x0F12, 0x0002},
		{0x002A, 0x02C4},
		{0x0F12, 0x0001},
		{0x002A, 0x02CA},
		{0x0F12, 0x014D},
		{0x002A, 0x02C8},
		{0x0F12, 0x0000},
		{0x002A, 0x01A8},
		{0x0F12, 0x0000},
		{0x0F12, 0x0001},
		{0x002A, 0x01B0},
		{0x0F12, 0x0000},
		{0x002A, 0x019E},
		{0x0F12, 0x0001},
		{0x002A, 0x01A0},
		{0x0F12, 0x0001},
	},
};
#endif /* #if 0 */
static struct v4l2_subdev_info s5k8aay_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_YUYV8_2X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static const struct of_device_id s5k8aay_dt_match[] = {
	{.compatible = "qcom,s5k8aay", .data = &s5k8aay_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, s5k8aay_dt_match);

static struct platform_driver s5k8aay_platform_driver = {
	.driver = {
		.name = "qcom,s5k8aay",
		.owner = THIS_MODULE,
		.of_match_table = s5k8aay_dt_match,
	},
};


//static int msm_sensor_s5k8aay_sensor_config(struct msm_sensor_ctrl_t *s_ctrl,
int32_t msm_sensor_s5k8aay_sensor_config(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp)
{
	struct sensorb_cfg_data *cdata = (struct sensorb_cfg_data *)argp;
	long rc = 0;
	int32_t i = 0;
	mutex_lock(s_ctrl->msm_sensor_mutex);
	CDBG("%s:%d %s cfgtype = %d\n", __func__, __LINE__,
		s_ctrl->sensordata->sensor_name, cdata->cfgtype);


	switch (cdata->cfgtype) {
	case CFG_GET_SENSOR_INFO:
		memcpy(cdata->cfg.sensor_info.sensor_name,
			s_ctrl->sensordata->sensor_name,
			sizeof(cdata->cfg.sensor_info.sensor_name));
		cdata->cfg.sensor_info.session_id =
			s_ctrl->sensordata->sensor_info->session_id;
		for (i = 0; i < SUB_MODULE_MAX; i++)
			cdata->cfg.sensor_info.subdev_id[i] =
				s_ctrl->sensordata->sensor_info->subdev_id[i];
		CDBG("%s:%d sensor name %s\n", __func__, __LINE__,
			cdata->cfg.sensor_info.sensor_name);
		CDBG("%s:%d session id %d\n", __func__, __LINE__,
			cdata->cfg.sensor_info.session_id);
		for (i = 0; i < SUB_MODULE_MAX; i++)
			CDBG("%s:%d subdev_id[%d] %d\n", __func__, __LINE__, i,
				cdata->cfg.sensor_info.subdev_id[i]);

		break;
	case CFG_SET_INIT_SETTING:
		/* 1. Write Recommend settings */
		/* 2. Write change settings */
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(
			s_ctrl->sensor_i2c_client, s5k8aay_recommend_settings1,
			ARRAY_SIZE(s5k8aay_recommend_settings1),
			MSM_CAMERA_I2C_WORD_DATA);
		if (rc < 0) {
			pr_err("%s: i2c read err \n", __func__);
			msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
		}
		usleep_range(1000, 1000);
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(
			s_ctrl->sensor_i2c_client, s5k8aay_recommend_settings2,
			ARRAY_SIZE(s5k8aay_recommend_settings2),
			MSM_CAMERA_I2C_WORD_DATA);
		if (rc < 0) {
			pr_err("%s: i2c read err \n", __func__);
			msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
		}
		usleep_range(10000, 10000);
		break;
	case CFG_SET_RESOLUTION:
#if 0
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(
//			s_ctrl->sensor_i2c_client, s5k8aay_vga_settings,
//			ARRAY_SIZE(s5k8aay_vga_settings),
			s_ctrl->sensor_i2c_client, s5k8aay_sxga_settings,
			ARRAY_SIZE(s5k8aay_sxga_settings),
			MSM_CAMERA_I2C_WORD_DATA);
		usleep_range(10000, 10000);

		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(
			s_ctrl->sensor_i2c_client, s5k8aay_stop_stream_settings,
			ARRAY_SIZE(s5k8aay_stop_stream_settings),
			MSM_CAMERA_I2C_WORD_DATA);
		usleep_range(10000, 10000);
#endif
		break;
	case CFG_SET_STOP_STREAM:
		rc = msm_sensor_s5k8aay_stop_stream(s_ctrl);
		break;
	case CFG_SET_START_STREAM:
		if (s_ctrl->func_tbl->sensor_start_stream)
			rc = s_ctrl->func_tbl->sensor_start_stream(s_ctrl);
		else
			rc = -EFAULT;

		break;
	case CFG_GET_SENSOR_INIT_PARAMS:
		cdata->cfg.sensor_init_params =
			*s_ctrl->sensordata->sensor_init_params;
		CDBG("%s:%d init params mode %d pos %d mount %d\n", __func__,
			__LINE__,
			cdata->cfg.sensor_init_params.modes_supported,
			cdata->cfg.sensor_init_params.position,
			cdata->cfg.sensor_init_params.sensor_mount_angle);
		break;
	case CFG_SET_SLAVE_INFO: {
		struct msm_camera_sensor_slave_info sensor_slave_info;
		struct msm_sensor_power_setting_array *power_setting_array;
		int slave_index = 0;
		if (copy_from_user(&sensor_slave_info,
		    (void *)cdata->cfg.setting,
		    sizeof(struct msm_camera_sensor_slave_info))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		/* Update sensor slave address */
		if (sensor_slave_info.slave_addr) {
			s_ctrl->sensor_i2c_client->cci_client->sid =
				sensor_slave_info.slave_addr >> 1;
		}

		/* Update sensor address type */
		s_ctrl->sensor_i2c_client->addr_type =
			sensor_slave_info.addr_type;

		/* Update power up / down sequence */
		s_ctrl->power_setting_array =
			sensor_slave_info.power_setting_array;
		power_setting_array = &s_ctrl->power_setting_array;
		power_setting_array->power_setting = kzalloc(
			power_setting_array->size *
			sizeof(struct msm_sensor_power_setting), GFP_KERNEL);
		if (!power_setting_array->power_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(power_setting_array->power_setting,
		    (void *)sensor_slave_info.power_setting_array.power_setting,
		    power_setting_array->size *
		    sizeof(struct msm_sensor_power_setting))) {
			kfree(power_setting_array->power_setting);
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		s_ctrl->free_power_setting = true;
		CDBG("%s sensor id %x\n", __func__,
			sensor_slave_info.slave_addr);
		CDBG("%s sensor addr type %d\n", __func__,
			sensor_slave_info.addr_type);
		CDBG("%s sensor reg %x\n", __func__,
			sensor_slave_info.sensor_id_info.sensor_id_reg_addr);
		CDBG("%s sensor id %x\n", __func__,
			sensor_slave_info.sensor_id_info.sensor_id);
		for (slave_index = 0; slave_index <
			power_setting_array->size; slave_index++) {
			CDBG("%s i %d power setting %d %d %ld %d\n", __func__,
				slave_index,
				power_setting_array->power_setting[slave_index].
				seq_type,
				power_setting_array->power_setting[slave_index].
				seq_val,
				power_setting_array->power_setting[slave_index].
				config_val,
				power_setting_array->power_setting[slave_index].
				delay);
		}
		kfree(power_setting_array->power_setting);
		break;
	}
	case CFG_WRITE_I2C_ARRAY: {
		struct msm_camera_i2c_reg_setting conf_array;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;

		if (copy_from_user(&conf_array,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
			conf_array.size *
			sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_table(
			s_ctrl->sensor_i2c_client, &conf_array);
		if (rc < 0) {
			pr_err("%s: i2c write err \n", __func__);
			msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
		}
		kfree(reg_setting);
		break;
	}
	case CFG_WRITE_I2C_SEQ_ARRAY: {
		struct msm_camera_i2c_seq_reg_setting conf_array;
		struct msm_camera_i2c_seq_reg_array *reg_setting = NULL;

		if (copy_from_user(&conf_array,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_seq_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_seq_reg_array)),
			GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
			conf_array.size *
			sizeof(struct msm_camera_i2c_seq_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_seq_table(s_ctrl->sensor_i2c_client,
			&conf_array);
		if (rc < 0) {
			pr_err("%s: i2c read err \n", __func__);
			msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
		}
		kfree(reg_setting);
		break;
	}

	case CFG_POWER_UP:
		if (s_ctrl->func_tbl->sensor_power_up)
			rc = s_ctrl->func_tbl->sensor_power_up(s_ctrl);
		else
			rc = -EFAULT;
		break;

	case CFG_POWER_DOWN:
		if (s_ctrl->func_tbl->sensor_power_down)
			rc = s_ctrl->func_tbl->sensor_power_down(
				s_ctrl);
		else
			rc = -EFAULT;
		break;

	case CFG_SET_STOP_STREAM_SETTING: {
		struct msm_camera_i2c_reg_setting *stop_setting =
			&s_ctrl->stop_setting;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;
		if (copy_from_user(stop_setting, (void *)cdata->cfg.setting,
		    sizeof(struct msm_camera_i2c_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = stop_setting->reg_setting;
		stop_setting->reg_setting = kzalloc(stop_setting->size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!stop_setting->reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(stop_setting->reg_setting,
		    (void *)reg_setting, stop_setting->size *
		    sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(stop_setting->reg_setting);
			stop_setting->reg_setting = NULL;
			stop_setting->size = 0;
			rc = -EFAULT;
			break;
		}
		break;
	}

	case CFG_SET_EXPOSURE_COMPENSATION:
		rc = msm_sensor_s5k8aay_write_exp(s_ctrl, cdata->cfg.exposure);
		break;

	case CFG_SET_WHITE_BALANCE:
		rc = msm_sensor_s5k8aay_set_white_balance(s_ctrl, cdata->cfg.wb);
		break;

	case CFG_SET_EFFECT:
		rc = msm_sensor_s5k8aay_set_effect(s_ctrl, cdata->cfg.effect);
		break;

	case CFG_GET_EXP_TIME:
		// default = 1000ms
		cdata->cfg.exp_time = 1000;

		rc = msm_sensor_s5k8aay_get_exp_time(s_ctrl, &cdata->cfg.exp_time);

		argp = (void *)cdata;


		break;

	case CFG_SET_TIMEOUT_ERROR:
		msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_TIMEOUT_ERROR);
		break;

	default:
		rc = -EFAULT;
		break;
	}

	mutex_unlock(s_ctrl->msm_sensor_mutex);

	return rc;
}

static int msm_sensor_s5k8aay_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	struct msm_sensor_power_setting_array *power_setting_array = NULL;
	struct msm_sensor_power_setting *power_setting = NULL;
	struct msm_camera_sensor_board_info *data = s_ctrl->sensordata;

	CDBG("%s:%d\n", __func__, __LINE__);
	power_setting_array = &s_ctrl->power_setting_array;

	if (data->gpio_conf->cam_gpiomux_conf_tbl != NULL) {
		pr_err("%s:%d mux install\n", __func__, __LINE__);
		msm_gpiomux_install(
			(struct msm_gpiomux_config *)
			data->gpio_conf->cam_gpiomux_conf_tbl,
			data->gpio_conf->cam_gpiomux_conf_tbl_size);
	}

	rc = msm_camera_request_gpio_table(
		data->gpio_conf->cam_gpio_req_tbl,
		data->gpio_conf->cam_gpio_req_tbl_size, 1);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		return rc;
	}

#if 0
	gpio_set_value_cansleep(26, GPIO_OUT_HIGH);       /* VSCAMA High */
#else  /* #if 0 */
	power_setting = &power_setting_array->power_setting[0];
	msm_camera_config_single_vreg(s_ctrl->dev,
		&data->cam_vreg[power_setting->seq_val],
		(struct regulator **)&power_setting->data[0],
		1);
#endif /* #if 0 */

	usleep_range(1000, 1000);

	gpio_set_value_cansleep(18, GPIO_OUT_HIGH);       /* VSCAMD High*/

	usleep_range(1000, 1000);

#if 0
	gpio_set_value_cansleep(9, GPIO_OUT_HIGH);       /* VSCAMIO High */
#else  /* #if 0 */
	power_setting = &power_setting_array->power_setting[2];
	msm_camera_config_single_vreg(s_ctrl->dev,
		&data->cam_vreg[power_setting->seq_val],
		(struct regulator **)&power_setting->data[0],
		1);
#endif /* #if 0 */

	usleep_range(1000, 1000);

	power_setting = &power_setting_array->power_setting[3];

	if (power_setting->seq_val >= s_ctrl->clk_info_size) {
		pr_err("%s clk index %d >= max %d\n", __func__,
			power_setting->seq_val,
			s_ctrl->clk_info_size);
		goto power_up_failed;
	}

	rc = msm_cam_clk_enable(s_ctrl->dev,
		&s_ctrl->clk_info[0],
		(struct clk **)&power_setting->data[0],
		s_ctrl->clk_info_size,
		1);
	if (rc < 0) {
		pr_err("%s: clk enable failed\n",
			__func__);
		goto power_up_failed;
	}

	usleep_range(50, 50);

	gpio_set_value_cansleep(4, GPIO_OUT_HIGH);       /* Standby High */

	usleep_range(50, 50);

	gpio_set_value_cansleep(14, GPIO_OUT_HIGH);       /* Reset High */

	usleep_range(500, 500);

	if (s_ctrl->sensor_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_util(
			s_ctrl->sensor_i2c_client, MSM_CCI_INIT);
		if (rc < 0) {
			pr_err("%s cci_init failed\n", __func__);
			msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
			goto power_up_failed;
		}
	}

	if (s_ctrl->func_tbl->sensor_match_id)
		rc = s_ctrl->func_tbl->sensor_match_id(s_ctrl);
	else
		rc = msm_sensor_match_id(s_ctrl);
	if (rc < 0) {
		pr_err("%s:%d match id failed rc %d\n", __func__, __LINE__, rc);
		msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
	}
	s5k8aay_wb = CAMERA_WB_MODE_AUTO;
	s5k8aay_effect = CAMERA_EFFECT_MODE_OFF;
	s5k8aay_power_on_first = 1;

	CDBG("%s exit\n", __func__);
	return 0;
power_up_failed:
	pr_err("%s:%d failed\n", __func__, __LINE__);
	return rc;
}

static int msm_sensor_s5k8aay_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
	struct msm_sensor_power_setting_array *power_setting_array = NULL;
	struct msm_sensor_power_setting *power_setting = NULL;
	struct msm_camera_sensor_board_info *data = s_ctrl->sensordata;

	CDBG("%s:%d\n", __func__, __LINE__);
	power_setting_array = &s_ctrl->power_setting_array;

	if (s_ctrl->sensor_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_util(
			s_ctrl->sensor_i2c_client, MSM_CCI_RELEASE);
	}

	gpio_set_value_cansleep(4, GPIOF_OUT_INIT_LOW);       /* Standby Low */

	usleep_range(100000, 100000);

	gpio_set_value_cansleep(14,GPIOF_OUT_INIT_LOW);       /* Reset Low*/

	usleep_range(500, 500);

	power_setting = &power_setting_array->power_setting[3];

	msm_cam_clk_enable(s_ctrl->dev,
		&s_ctrl->clk_info[0],
		(struct clk **)&power_setting->data[0],
		s_ctrl->clk_info_size,
		0);

	usleep_range(500, 500);

#if 0
	gpio_set_value_cansleep(9, GPIOF_OUT_INIT_LOW);       /* VSCAMIO Low */
#else  /* #if 0 */
	power_setting = &power_setting_array->power_setting[2];
	msm_camera_config_single_vreg(s_ctrl->dev,
		&data->cam_vreg[power_setting->seq_val],
		(struct regulator **)&power_setting->data[0],
		0);
#endif /* #if 0 */

	usleep_range(1000, 1000);

	gpio_set_value_cansleep(18,GPIOF_OUT_INIT_LOW);       /* VSCAMD Low*/

	usleep_range(1000, 1000);

#if 0
	gpio_set_value_cansleep(26, GPIOF_OUT_INIT_LOW);       /* VSCAMA Low */
	usleep_range(1000, 1000);
#else  /* #if 0 */
	power_setting = &power_setting_array->power_setting[0];
	msm_camera_config_single_vreg(s_ctrl->dev,
		&data->cam_vreg[power_setting->seq_val],
		(struct regulator **)&power_setting->data[0],
		0);
#endif /* #if 0 */

	msm_camera_request_gpio_table(
		data->gpio_conf->cam_gpio_req_tbl,
		data->gpio_conf->cam_gpio_req_tbl_size, 0);
	s5k8aay_power_on_first = 0;

	CDBG("%s exit\n", __func__);
	return 0;
}

static int32_t msm_sensor_s5k8aay_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	uint16_t chip_id = 0;

	CDBG("%s: E\n", __func__);

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
		i2c_write_conf_tbl(
		s_ctrl->sensor_i2c_client,
		s5k8aay_id_read_settings,
		ARRAY_SIZE(s5k8aay_id_read_settings),
		MSM_CAMERA_I2C_WORD_DATA);
	if (rc < 0) {
		pr_err("%s: i2c write err \n", __func__);
		msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
		goto failure;
	}
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
		i2c_read(
		s_ctrl->sensor_i2c_client,
		S5K8AAY_REG_READ_ADDR,
		&chip_id,
		MSM_CAMERA_I2C_WORD_DATA);
	if (rc < 0) {
		pr_err("%s: i2c read err \n", __func__);
		msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
		goto failure;
	}
	CDBG("%s: chip_id = %x %x\n", __func__, chip_id, s_ctrl->sensordata->slave_info->sensor_id);
	if (chip_id != s_ctrl->sensordata->slave_info->sensor_id) {
		pr_err("msm_sensor_match_id chip id doesnot match\n");
		rc = -ENODEV;
		goto failure;
	}

failure:
	CDBG("%s: X rc = %d\n", __func__, rc);
	return rc;
}

static int msm_sensor_s5k8aay_start_stream(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	static struct msm_camera_i2c_reg_conf s5k8aay_power_on_first_setting[] = {
		{0x002A, 0x019E},
		{0x0F12, 0x0001},
		{0x002A, 0x01A0},
		{0x0F12, 0x0001},
		{0x0028, 0xD000},
		{0x002A, 0x1000},
		{0x0F12, 0x0001},
	};

	CDBG("%s: E\n", __func__);

    if(s5k8aay_power_on_first)
	{
		s5k8aay_power_on_first = 0;

		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(
			s_ctrl->sensor_i2c_client,
			s5k8aay_power_on_first_setting,
			ARRAY_SIZE(s5k8aay_power_on_first_setting),
			MSM_CAMERA_I2C_WORD_DATA);
		if (rc < 0) {
			pr_err("%s: i2c write err \n", __func__);
			msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
		}
		usleep_range(10000, 10000);
    }

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
		i2c_write_conf_tbl(
		s_ctrl->sensor_i2c_client, s5k8aay_start_stream_settings,
		ARRAY_SIZE(s5k8aay_start_stream_settings),
		MSM_CAMERA_I2C_WORD_DATA);
	if (rc < 0) {
		pr_err("%s: i2c read err \n", __func__);
		msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
	}

	s5k8aay_skip_frame_flg = 0;

	s5k8aay_power_on_flg = 1;
	if (s5k8aay_err_chk_work_flg == 0) {
		s5k8aay_err_chk_work_flg = 1;
		s5k8aay_err_chk_work->s_ctrl = s_ctrl;
		queue_work( s5k8aay_err_chk_wq, (struct work_struct *)s5k8aay_err_chk_work );
	}

	CDBG("%s: X\n", __func__);
	return rc;
}

static int32_t msm_sensor_s5k8aay_stop_stream(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;

	CDBG("%s: E\n", __func__);

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
		i2c_write_conf_tbl(
		s_ctrl->sensor_i2c_client, s5k8aay_stop_stream_settings,
		ARRAY_SIZE(s5k8aay_stop_stream_settings),
		MSM_CAMERA_I2C_WORD_DATA);
	if (rc < 0) {
		pr_err("%s: i2c read err \n", __func__);
		msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
	}

	s5k8aay_power_on_flg = 0;

	CDBG("%s: X\n", __func__);
	return rc;
}

static int32_t msm_sensor_s5k8aay_write_exp(struct msm_sensor_ctrl_t *s_ctrl, int32_t exp_value)
{
	int32_t exp_setting;
	int32_t rc = 0;

	exp_setting = exp_value / S5K8AAY_EXPOSURE_VALUE_STEP + S5K8AAY_EXPOSURE_VALUE_MAX;
	if (exp_setting < 0) {
		exp_setting = 0;
	}
	else if (exp_setting > (S5K8AAY_EXPOSURE_VALUE_MAX * 2)) {
		exp_setting = S5K8AAY_EXPOSURE_VALUE_MAX * 2;
	}

	CDBG("%s: exp_value = %d, exp_setting = %d\n", __func__, exp_value, exp_setting);

	if (pre_exp_setting == exp_setting) {
		CDBG("%s: exp_setting no change \n", __func__);
		return 0;
	}
	pre_exp_setting = exp_setting;

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(
		s_ctrl->sensor_i2c_client,
		&s5k8aay_config_exposure_settings[exp_setting][0],
		ARRAY_SIZE(s5k8aay_config_exposure_settings[exp_setting]),
		MSM_CAMERA_I2C_WORD_DATA);
	if (rc < 0) {
		pr_err("%s: i2c write err \n", __func__);
		msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
	}

	msm_sensor_s5k8aay_set_skip_frame();

	CDBG("%s: rc = %d\n", __func__, rc);
	return rc;
}

static int32_t msm_sensor_s5k8aay_set_white_balance(struct msm_sensor_ctrl_t *s_ctrl, int32_t wb)
{
	int32_t index = 0;
	int32_t rc = 0;

	CDBG("%s: wb = %d\n", __func__, wb);

	if (s5k8aay_wb == wb) {
		CDBG("%s: wb no change \n", __func__);
		return 0;
	}

	s5k8aay_wb = wb;

	switch (wb) {
	case CAMERA_WB_MODE_DAYLIGHT:
		index = S5K8AAY_WB_DAYLIGHT;
		break;

	case CAMERA_WB_MODE_FLUORESCENT:
		index = S5K8AAY_WB_FLUORESCENT;
		break;

	case CAMERA_WB_MODE_WARM_FLUORESCENT:
		index = S5K8AAY_WB_WARM_FLUORESCENT;
		break;

	case CAMERA_WB_MODE_INCANDESCENT:
		index = S5K8AAY_WB_INCANDESCENT;
		break;

	case CAMERA_WB_MODE_AUTO:
	default:
		index = S5K8AAY_WB_AUTO;
		break;
	}

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(
		s_ctrl->sensor_i2c_client,
		&s5k8aay_config_white_balance_settings[index][0],
		ARRAY_SIZE(s5k8aay_config_white_balance_settings[index]),
		MSM_CAMERA_I2C_WORD_DATA);
	if (rc < 0) {
		pr_err("%s: i2c write err \n", __func__);
		msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
	}

	msm_sensor_s5k8aay_set_skip_frame();

	CDBG("%s: rc = %d\n", __func__, rc);
	return rc;
}

static int32_t msm_sensor_s5k8aay_get_exp_time(struct msm_sensor_ctrl_t *s_ctrl, uint32_t *exposure_time)
{

	int32_t rc = 0;
	int32_t i;
	uint16_t data[2];

	CDBG("%s: E\n", __func__);
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(
		s_ctrl->sensor_i2c_client,
		&s5k8aay_exp_time_settings[0],
		ARRAY_SIZE(s5k8aay_exp_time_settings),
		MSM_CAMERA_I2C_WORD_DATA);
	if (rc < 0) {
		pr_err("%s: i2c write err \n", __func__);
		msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
		goto failure;
	}

	for (i=0; i<2; i++) {
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_read(
			s_ctrl->sensor_i2c_client,
			S5K8AAY_REG_READ_ADDR,
			&data[i],
			MSM_CAMERA_I2C_WORD_DATA);
		if (rc < 0) {
			pr_err("%s: i2c read err \n", __func__);
			msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
			goto failure;
		}
	}

	s5k8aay_g_exposure_time = ((data[1]<<16) | data[0]) / S5K8AAY_EXP_TIME_DIVISOR;
	*exposure_time = s5k8aay_g_exposure_time;

failure:

	CDBG("%s: X rc = %d\n", __func__, rc);
	return rc;
}

static void msm_sensor_s5k8aay_error_check_work(struct work_struct *work)
{
	int32_t i2c_rc = 0;
	int32_t reg_rc = 0;
	uint16_t read_data;
	s5k8aay_work_t *tmp_work = (s5k8aay_work_t *)work;
	struct msm_sensor_ctrl_t *s_ctrl = tmp_work->s_ctrl;

	CDBG("%s: E\n", __func__);

	if (s5k8aay_power_on_flg) {
		mutex_lock(s_ctrl->msm_sensor_mutex);

		i2c_rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(
			s_ctrl->sensor_i2c_client,
			s5k8aay_error_check_write_reg,
			ARRAY_SIZE(s5k8aay_error_check_write_reg),
			MSM_CAMERA_I2C_WORD_DATA);

		mutex_unlock(s_ctrl->msm_sensor_mutex);
	}

	if (i2c_rc < 0) {
		pr_err("%s: i2c write err \n", __func__);
		msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
	}

	while (s5k8aay_power_on_flg) {

		if (s5k8aay_power_on_flg) {
			mutex_lock(s_ctrl->msm_sensor_mutex);

			i2c_rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
				i2c_write_conf_tbl(
				s_ctrl->sensor_i2c_client,
				s5k8aay_error_check_reg_1,
				ARRAY_SIZE(s5k8aay_error_check_reg_1),
				MSM_CAMERA_I2C_WORD_DATA);
			if (i2c_rc < 0) {
				pr_err("%s: i2c write err \n", __func__);
			}

			i2c_rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
				i2c_read(
				s_ctrl->sensor_i2c_client,
				0x0F12,
				&read_data,
				MSM_CAMERA_I2C_WORD_DATA);
			if (i2c_rc < 0) {
				pr_err("%s: i2c read err \n", __func__);
			}
			if (read_data != S5K8AAY_ERR_CHK_VAL) {
				pr_err("%s: read_data err \n", __func__);
				reg_rc = -1;
			}
			i2c_rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
				i2c_write_conf_tbl(
				s_ctrl->sensor_i2c_client,
				s5k8aay_error_check_reg_2,
				ARRAY_SIZE(s5k8aay_error_check_reg_2),
				MSM_CAMERA_I2C_WORD_DATA);
			if (i2c_rc < 0) {
				pr_err("%s: i2c write err \n", __func__);
			}

			i2c_rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
				i2c_read(
				s_ctrl->sensor_i2c_client,
				0x0F12,
				&read_data,
				MSM_CAMERA_I2C_WORD_DATA);
			if (i2c_rc < 0) {
				pr_err("%s: i2c read err \n", __func__);
			}
			if (read_data != S5K8AAY_ERR_CHK_VAL) {
				pr_err("%s: read_data err \n", __func__);
				reg_rc = -1;
			}
			mutex_unlock(s_ctrl->msm_sensor_mutex);
		}

		if (i2c_rc < 0) {
			pr_err("%s: X i2c_rc = %d\n", __func__, i2c_rc);
			msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
			break;
		}
		else if (reg_rc < 0) {
			pr_err("%s: X reg_rc = %d\n", __func__, reg_rc);
			msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
			break;
		}
		else {
			CDBG("%s: OK\n", __func__);
		}

		msleep(2000);
	}

	s5k8aay_err_chk_work_flg = 0;
	CDBG("%s: X\n", __func__);
	return;
}

static void msm_sensor_s5k8aay_set_skip_frame(void) {
	CDBG("%s: E\n", __func__);
	s5k8aay_skip_frame_flg = 1;
    /* Wait:35ms For 1frame Mask */
	usleep_range(35000, 35000);
}

#if 0
static void msm_sensor_s5k8aay_get_frame_skip_flg(struct msm_sensor_ctrl_t *s_ctrl, uint8_t *flag)
{
	CDBG("%s: E\n", __func__);
	*flag = s5k8aay_skip_frame_flg;
	s5k8aay_skip_frame_flg = 0;
}
#endif /* #if 0 */

static int32_t msm_sensor_s5k8aay_set_effect(struct msm_sensor_ctrl_t *s_ctrl, int32_t effect)
{
	int32_t index = 0;
	int32_t rc = 0;

	CDBG("%s: effect = %d\n", __func__, effect);

	if (s5k8aay_effect == effect) {
		CDBG("%s: effect no change \n", __func__);
		return 0;
	}

	s5k8aay_effect = effect;

	switch (effect) {
	case CAMERA_EFFECT_MODE_MONO:
		index = S5K8AAY_EFFECT_MONO;
		break;

	case CAMERA_EFFECT_MODE_SEPIA:
		index = S5K8AAY_EFFECT_SEPIA;
		break;

	case CAMERA_EFFECT_MODE_NEGATIVE:
		index = S5K8AAY_EFFECT_NEGATIVE;
		break;
	case CAMERA_EFFECT_MODE_OFF:
	default:
		index = S5K8AAY_EFFECT_OFF;
		break;
	}

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_conf_tbl(
		s_ctrl->sensor_i2c_client,
		&s5k8aay_config_effect_settings[index][0],
		ARRAY_SIZE(s5k8aay_config_effect_settings[index]),
		MSM_CAMERA_I2C_WORD_DATA);
	if (rc < 0) {
		pr_err("%s: i2c write err \n", __func__);
		msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
	}

	msm_sensor_s5k8aay_set_skip_frame();

	CDBG("%s: rc = %d\n", __func__, rc);
	return rc;
}

static const struct i2c_device_id s5k8aay_i2c_id[] = {
	{S5K8AAY_SENSOR_NAME, (kernel_ulong_t)&s5k8aay_s_ctrl},
	{ }
};

static int32_t msm_sensor_s5k8aay_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &s5k8aay_s_ctrl);
}

static struct i2c_driver s5k8aay_i2c_driver = {
	.id_table = s5k8aay_i2c_id,
	.probe  = msm_sensor_s5k8aay_i2c_probe,
	.driver = {
		.name = S5K8AAY_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client s5k8aay_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static int32_t msm_sensor_s5k8aay_platform_probe(struct platform_device *pdev)
{
	int32_t rc;
	const struct of_device_id *match;
	match = of_match_device(s5k8aay_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static void __exit msm_sensor_s5k8aay_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (s5k8aay_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&s5k8aay_s_ctrl);
		platform_driver_unregister(&s5k8aay_platform_driver);
	} else
		i2c_del_driver(&s5k8aay_i2c_driver);
	return;
}

static int __init msm_sensor_s5k8aay_init_module(void)
{
	int32_t rc;
	pr_info("%s:%d\n", __func__, __LINE__);
	s5k8aay_err_chk_wq = create_workqueue("s5k8aay_err_chk_queue");
	if (s5k8aay_err_chk_wq) {
		s5k8aay_err_chk_work = (s5k8aay_work_t *)kmalloc(sizeof(s5k8aay_work_t), GFP_KERNEL);
		if (s5k8aay_err_chk_work) {
			INIT_WORK( (struct work_struct *)s5k8aay_err_chk_work, msm_sensor_s5k8aay_error_check_work );
		}
	}

	rc = platform_driver_probe(&s5k8aay_platform_driver,
		msm_sensor_s5k8aay_platform_probe);
	if (!rc)
		return rc;
	return i2c_add_driver(&s5k8aay_i2c_driver);
}

static struct msm_sensor_fn_t s5k8aay_sensor_func_tbl = {
	.sensor_config = msm_sensor_s5k8aay_sensor_config,
	.sensor_power_up = msm_sensor_s5k8aay_power_up,
	.sensor_power_down = msm_sensor_s5k8aay_power_down,
	.sensor_match_id = msm_sensor_s5k8aay_match_id,
	.sensor_start_stream = msm_sensor_s5k8aay_start_stream,
};

static struct msm_sensor_ctrl_t s5k8aay_s_ctrl = {
	.sensor_i2c_client = &s5k8aay_sensor_i2c_client,
	.power_setting_array.power_setting = s5k8aay_power_setting,
	.power_setting_array.size = ARRAY_SIZE(s5k8aay_power_setting),
	.msm_sensor_mutex = &s5k8aay_mut,
	.sensor_v4l2_subdev_info = s5k8aay_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(s5k8aay_subdev_info),
	.func_tbl = &s5k8aay_sensor_func_tbl,
};

module_init(msm_sensor_s5k8aay_init_module);
module_exit(msm_sensor_s5k8aay_exit_module);
MODULE_DESCRIPTION("Samsung 1.2MP YUV sensor driver");
MODULE_LICENSE("GPL v2");
