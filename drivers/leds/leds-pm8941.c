/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2013 KYOCERA Corporation
*/
/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

//#define DISABLE_DISP_DETECT 1

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/leds.h>
#include <linux/err.h>
#include <linux/spinlock.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <linux/spmi.h>
#include <linux/qpnp/pwm.h>
#include <linux/workqueue.h>
#include <linux/kc_led.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/regulator/consumer.h>
#include <linux/power_supply.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/delay.h>

#define LED_DEBUG 0

#if LED_DEBUG
#define DEBUG_PRINT(arg...)    printk(KERN_INFO "[LEDDRV]:" arg)
#define LED_DUMP_REGISTER(led, reg_array, array_size)    led_dump_register(led, reg_array, array_size)
#else
#define DEBUG_PRINT(arg...)
#define LED_DUMP_REGISTER(led, reg_array, array_size)
#endif

#define DEBUG_PRINT_ERR(arg...)    printk(KERN_ERR "[LEDDRV]:" arg)

#define LED_RESET_ON    0x01
#define LED_RESET_OFF    0x00

#define ALL_OFF_STATE           0x00000000
#define BACK_LIGHT_STATE        0x00000001
#define LED_RED_STATE           0x00000004
#define LED_GREEN_STATE         0x00000008
#define LED_BLUE_STATE          0x00000010
#define MOBILE_LIGHT_STATE      0x00000020
#define RGB_LED_STATE           (LED_RED_STATE|LED_GREEN_STATE|LED_BLUE_STATE)
#define LED_CHARGEPUMP_STATE    (RGB_LED_STATE|MOBILE_LIGHT_STATE)

#define RGB_LED_INFO          "ledinfo"
#define MOBILELIGHT_INFO      "mobilelightinfo"
#define BACKLIGHT_INFO        "backlightinfo"

#define LABEL_RGB_LED         "rgb"
#define LABEL_MOBILELIGHT     "flash"
#define LABEL_BACKLIGHT       "wled"

#define REG_MASK_VALUE    0xFF

#define RGB_LED_MAX_BRIGHT_VAL        0xFFFFFFFFu
#define MOBILELIGHT_MAX_BRIGHT_VAL    0xFFFFFFFFu
#define BACKLIGHT_MAX_BRIGHT_VAL      0xFFFFFFFFu

#define LED_COL_BLACK    0x00000000

#define LSB_8_BIT_MASK    0xFF
#define MSB_4_BIT_MASK    0x0F
#define MSB_5_BIT_MASK    0x1F
#define MSB_8_BIT_SHFT    0x08

#define GET_MSB(val)         ((val >> MSB_8_BIT_SHFT) & MSB_4_BIT_MASK)
#define GET_MSB_5BIT(val)    ((val >> MSB_8_BIT_SHFT) & MSB_5_BIT_MASK)
#define GET_LSB(val)         (val & LSB_8_BIT_MASK)

#define VALUE_TRANSFORMATION(max_val, trans_val, base_val) ((max_val * trans_val) / base_val)

#define LEDLIGHT_BLINK_NUM        4

#define LEDLIGHT            'L'
#define LEDLIGHT_SET_BLINK            _IOW(LEDLIGHT, 0, T_LEDLIGHT_IOCTL)
#define LEDLIGHT_SET_TEMPERTURE_DM    _IOW(LEDLIGHT, 1, T_LEDLIGHT_IOCTL_DM)

#define SPMI_RETRIES_NUM    5
#define SPMI_NO_ERROR       0

#define BACKLIGHT_NUM        2

#define LPG_LUT_LSB_L(offset)    (0xB060 +(0x20 * offset))
#define LPG_LUT_MSB_L(offset)    (0xB061 +(0x20 * offset))
#define LPG_LUT_LSB_H(offset)    (0xB062 +(0x20 * offset))
#define LPG_LUT_MSB_H(offset)    (0xB063 +(0x20 * offset))

#define LPG_LUT_HI_INDEX_VAL(offset)        (0x11 + (0x10 * offset))
#define LPG_LUT_LO_INDEX_VAL(offset)        (0x10 + (0x10 * offset))

#define LPG_LUT_RAMP_CONTROL(offset)        (0x10 << offset)
#define LPG_LUT_RAMP_CONTROL_OFF            0x00

#define RGB_LED_GET_COL_B    0
#define RGB_LED_GET_COL_G    8
#define RGB_LED_GET_COL_R    16

#define RGB_LED_MIXED_COL       0x00FFFFFF
#define RGB_LED_SINGLE_COL_B    0x000000FF
#define RGB_LED_SINGLE_COL_G    0x0000FF00
#define RGB_LED_SINGLE_COL_R    0x00FF0000
#define RGB_LED_SINGLE_COL      0xFF

#define RGB_LED_MAX_LEVEL       0x1FF

#define RGB_LED_RAMP_DURATION    50
#define MAX_PAUSE_CODE           0x1FFF

#define RGB_LED_PATTERN_VAL_MAX_MSB      0x0F
#define RGB_LED_PATTERN_VAL_MAX_R_MSB     0x00
#define RGB_LED_PATTERN_VAL_MID_R_MSB     0x00
#define RGB_LED_PATTERN_VAL_MIN_R_MSB     0x00
#define RGB_LED_PATTERN_VAL_MAX_G_MSB     0x00
#define RGB_LED_PATTERN_VAL_MIN_G_MSB     0x00
#define RGB_LED_PATTERN_VAL_MAX_B_MSB     0x00
#define RGB_LED_PATTERN_VAL_MID1_B_MSB    0x00
#define RGB_LED_PATTERN_VAL_MID2_B_MSB    0x00
#define RGB_LED_PATTERN_VAL_MIN_B_MSB     0x00
#define RGB_LED_PATTERN_VAL_OFF_MSB      0x00

#define RGB_LED_PATTERN_VAL_MAX_LSB       0xFF
#define RGB_LED_PATTERN_VAL_MAX_R_LSB     0x2A
#define RGB_LED_PATTERN_VAL_MID_R_LSB     0x22
#define RGB_LED_PATTERN_VAL_MIN_R_LSB     0x19
#define RGB_LED_PATTERN_VAL_MAX_G_LSB     0x17
#define RGB_LED_PATTERN_VAL_MIN_G_LSB     0x15
#define RGB_LED_PATTERN_VAL_MAX_B_LSB     0x2A
#define RGB_LED_PATTERN_VAL_MID1_B_LSB    0x22
#define RGB_LED_PATTERN_VAL_MID2_B_LSB    0x19
#define RGB_LED_PATTERN_VAL_MIN_B_LSB     0x16
#define RGB_LED_PATTERN_VAL_OFF_LSB       0x00

#define RGB_LED_REG_B0C8    0xB0C8
#define RGB_LED_REG_D045    0xD045
#define RGB_LED_REG_D046    0xD046
#define RGB_LED_REG_D047    0xD047

#define RGB_LED_B_REG_B541    0xB541
#define RGB_LED_B_REG_B542    0xB542

#define RGB_LED_G_REG_B641    0xB641
#define RGB_LED_G_REG_B642    0xB642

#define RGB_LED_R_REG_B741    0xB741
#define RGB_LED_R_REG_B742    0xB742

#define LPG_CHAN_ENABLE_CONTROL(offset)             (0xB546 + (offset * 0x100))
#define LPG_CHAN_PWM_VALUE_MSB(offset)              (0xB545 + (offset * 0x100))
#define LPG_CHAN_PWM_VALUE_LSB(offset)              (0xB544 + (offset * 0x100))
#define LPG_CHAN_LPG_PATTERN_CONFIG(offset)         (0xB540 + (offset * 0x100))
#define LPG_CHAN_RAMP_STEP_DURATION_MSB(offset)     (0xB551 + (offset * 0x100))
#define LPG_CHAN_RAMP_STEP_DURATION_LSB(offset)     (0xB550 + (offset * 0x100))
#define LPG_CHAN_PAUSE_HI_MULTIPLIER_MSB(offset)    (0xB553 + (offset * 0x100))
#define LPG_CHAN_PAUSE_HI_MULTIPLIER_LSB(offset)    (0xB552 + (offset * 0x100))
#define LPG_CHAN_PAUSE_LO_MULTIPLIER_MSB(offset)    (0xB555 + (offset * 0x100))
#define LPG_CHAN_PAUSE_LO_MULTIPLIER_LSB(offset)    (0xB554 + (offset * 0x100))
#define LPG_CHAN_HI_INDEX(offset)                   (0xB556 + (offset * 0x100))
#define LPG_CHAN_LO_INDEX(offset)                   (0xB557 + (offset * 0x100))

#define RGB_LED_REG_D045_INIT    0x01
#define RGB_LED_REG_D047_INIT    0xC0

#define RGB_LED_B_REG_B541_INIT    0x33
#define RGB_LED_B_REG_B542_INIT    0x43

#define RGB_LED_G_REG_B641_INIT    0x33
#define RGB_LED_G_REG_B642_INIT    0x43

#define RGB_LED_R_REG_B741_INIT    0x33
#define RGB_LED_R_REG_B742_INIT    0x43

#define TRI_LED_EN_CTL_VAL(offset)    (0x20 << offset)
#define TRI_LED_EN_CTL_VAL_OFF        0x00

#define LPG_CHAN_ENABLE_CONTROL_ON       0xE4
#define LPG_CHAN_ENABLE_CONTROL_BLINK    0xE2
#define LPG_CHAN_ENABLE_CONTROL_OFF      0x04

#define LPG_CHAN_PWM_VALUE_MSB_OFF       0x00
#define LPG_CHAN_PWM_VALUE_LSB_OFF       0x00

#define LPG_CHAN_LPG_PATTERN_CONFIG_VAL    0x1F

#define MOBILELIGHT_REG_D341    0xD341
#define MOBILELIGHT_REG_D342    0xD342
#define MOBILELIGHT_REG_D343    0xD343
#define MOBILELIGHT_REG_D344    0xD344
#define MOBILELIGHT_REG_D346    0xD346
#define MOBILELIGHT_REG_D347    0xD347
#define MOBILELIGHT_REG_D348    0xD348
#define MOBILELIGHT_REG_D349    0xD349
#define MOBILELIGHT_REG_D34A    0xD34A
#define MOBILELIGHT_REG_D34B    0xD34B
#define MOBILELIGHT_REG_D34C    0xD34C
#define MOBILELIGHT_REG_D34D    0xD34D
#define MOBILELIGHT_REG_D34E    0xD34E
#define MOBILELIGHT_REG_D34F    0xD34F
#define MOBILELIGHT_REG_D351    0xD351
#define MOBILELIGHT_REG_D352    0xD352
#define MOBILELIGHT_REG_D356    0xD356

#define MOBILELIGHT_REG_D341_INIT    0x0F
#define MOBILELIGHT_REG_D342_INIT    0x01
#define MOBILELIGHT_REG_D343_INIT    0x00
#define MOBILELIGHT_REG_D344_INIT    0x0F
#define MOBILELIGHT_REG_D346_INIT    0x00
#define MOBILELIGHT_REG_D347_INIT    0x00
#define MOBILELIGHT_REG_D348_INIT    0x03
#define MOBILELIGHT_REG_D349_INIT    0x1F
#define MOBILELIGHT_REG_D34A_INIT    0x01
#define MOBILELIGHT_REG_D34B_INIT    0x01
#define MOBILELIGHT_REG_D34C_INIT    0x20
#define MOBILELIGHT_REG_D34D_INIT    0x00
#define MOBILELIGHT_REG_D34E_INIT    0x02
#define MOBILELIGHT_REG_D34F_INIT    0x80
#define MOBILELIGHT_REG_D351_INIT    0x80
#define MOBILELIGHT_REG_D352_INIT    0x00

#define MOBILELIGHT_REG_D342_ON_LOW    0x01
#define MOBILELIGHT_REG_D343_ON_LOW    0x00
#define MOBILELIGHT_REG_D34D_ON_LOW    0x00
#define MOBILELIGHT_REG_D34E_ON_LOW    0x02

#define MOBILELIGHT_REG_D342_ON_HIGH    0x00
#define MOBILELIGHT_REG_D343_ON_HIGH    0x00
#define MOBILELIGHT_REG_D34D_ON_HIGH    0x00
#define MOBILELIGHT_REG_D34E_ON_HIGH    0x00

#define MOBILELIGHT_REG_D346_ON    0x80
#define MOBILELIGHT_REG_D347_ON    0xC0

#define MOBILELIGHT_REG_D342_OFF    0x01
#define MOBILELIGHT_REG_D343_OFF    0x00
#define MOBILELIGHT_REG_D34D_OFF    0x00
#define MOBILELIGHT_REG_D34E_OFF    0x02
#define MOBILELIGHT_REG_D346_OFF    0x00
#define MOBILELIGHT_REG_D347_OFF    0x00

#define MOBILELIGHT_REG_D356_RESET_WDT    0x80
#define MOBILELIGHT_WDT_RESET_TIME        30
#define MOBILELIGHT_CAMERATERM_LOW_VAL    0x00
#define MOBILELIGHT_CAMERATERM_HIGH_VAL   0x01
#define MOBILELIGHT_THRESHOLD_TEMPVAL     0x41

#define MOBILELIGHT_ON    0x01
#define MOBILELIGHT_OFF   0x00

#define BACKLIGHT_BASE_LEVEL    0xFF
#define BACKLIGHT_MAX_LEVEL     0xFFF

#define BACKLIGHT_REG_D846    0xD846
#define BACKLIGHT_REG_D847    0xD847

#define BACKLIGHT_REG_D848    0xD848
#define BACKLIGHT_REG_D84C    0xD84C
#define BACKLIGHT_REG_D84D    0xD84D
#define BACKLIGHT_REG_D84E    0xD84E
#define BACKLIGHT_REG_D84F    0xD84F
#define BACKLIGHT_REG_D858    0xD858
#define BACKLIGHT_REG_D860    0xD860
#define BACKLIGHT_REG_D862    0xD862
#define BACKLIGHT_REG_D863    0xD863
#define BACKLIGHT_REG_D866    0xD866
#define BACKLIGHT_REG_D870    0xD870
#define BACKLIGHT_REG_D872    0xD872
#define BACKLIGHT_REG_D873    0xD873
#define BACKLIGHT_REG_D876    0xD876
#define BACKLIGHT_REG_D880    0xD880
#define BACKLIGHT_REG_D882    0xD882
#define BACKLIGHT_REG_D883    0xD883
#define BACKLIGHT_REG_D886    0xD886

#define BACKLIGHT_REG_D848_INIT    0x00
#define BACKLIGHT_REG_D84C_INIT    0x0B
#define BACKLIGHT_REG_D84D_INIT    0x03
#define BACKLIGHT_REG_D84E_INIT    0x03
#define BACKLIGHT_REG_D84F_INIT    0x60
#define BACKLIGHT_REG_D858_INIT    0x00
#define BACKLIGHT_REG_D860_INIT    0x80
#define BACKLIGHT_REG_D862_INIT    0x14
#define BACKLIGHT_REG_D863_INIT    0x00
#define BACKLIGHT_REG_D866_INIT    0x80
#define BACKLIGHT_REG_D870_INIT    0x80
#define BACKLIGHT_REG_D872_INIT    0x14
#define BACKLIGHT_REG_D873_INIT    0x00
#define BACKLIGHT_REG_D876_INIT    0x80
#define BACKLIGHT_REG_D880_INIT    0x00
#define BACKLIGHT_REG_D882_INIT    0x00
#define BACKLIGHT_REG_D883_INIT    0x00
#define BACKLIGHT_REG_D886_INIT    0x00

#define BACKLIGHT_REG_D846_ON      0x80
#define BACKLIGHT_REG_D846_OFF     0x00
#define BACKLIGHT_REG_D847_ON      0x07
#define BACKLIGHT_REG_D847_OFF     0x00

#define BACKLIGHT_BRIGHTNESS_SETTING_MSB(offset) (0xD841 + (0x02 * offset))
#define BACKLIGHT_BRIGHTNESS_SETTING_LSB(offset) (0xD840 + (0x02 * offset))

#define BACKLIGHT_ON                    0x01
#define BACKLIGHT_OFF                   0x00
#define BACKLIGHT_THRESHOLD_TEMPVAL     0x3C
#define BACKLIGHT_THRESHOLD_CURVAL      0xFFF


/**
 * enum qpnp_leds - QPNP supported led ids
 * @QPNP_ID_WLED - White led backlight
 */
enum qpnp_leds {
    ID_RGB_LED=0,
    ID_MOBILELIGHT,
    ID_BACKLIGHT,
    ID_LED_MAX,
};

enum led_index_enum{
    RGB_LED_B=0,
    RGB_LED_G,
    RGB_LED_R,
    RGB_LED_MAX,
};

enum lut_table_index_enum{
    LUT_MSB_L=0,
    LUT_LSB_L,
    LUT_MSB_H,
    LUT_LSB_H,
    LUT_TABLE_MAX,
};

enum color_pattern_enum{
    COLOR_PATTERN_1 = 0,
    COLOR_PATTERN_2,
    COLOR_PATTERN_3,
    COLOR_PATTERN_4,
    COLOR_PATTERN_5,
    COLOR_PATTERN_6,
    COLOR_PATTERN_7,
    COLOR_PATTERN_MAX
};

enum light_index_enum{
    MOBILELIGHT_INDEX = 0,
    LEDLIGHT_INDEX,
    BACKLIGHT_INDEX,
    LIGHT_INDEX_MAX
};

enum blink_control_enum{
    NO_BLINK_REQUEST = 0,
    BLINK_REQUEST,
};

#if LED_DEBUG
static u16 backlight_debug_regs[] = {
    0xD846, 0xD847, 0xD848, 0xD84c, 0xD84d, 0xD84e, 0xD84f, 0xD858,
    0xD860, 0xD862, 0xD863, 0xD866,
    0xD870, 0xD872, 0xD873, 0xD876,
    0xD880, 0xD882, 0xD883, 0xD886,
};

static u16 mobilelight_debug_regs[] = {
    0xA046, 
    0xD341, 0xD342, 0xD343, 0xD344, 0xD346, 0xD347, 0xD348, 0xD349, 0xD34a, 0xD34b, 0xD34c, 0xD34f,
    0xD351,
};

static u16 rgb_led_debug_regs[] = {
    0xB040, 0xB042, 0xB044, 0xB046, 0xB048, 0xB04A, 0xB04C, 0xB04E,
    0xB050, 0xB052, 0xB054, 0xB056, 0xB058, 0xB05A, 0xB05C, 0xB05E,
    0xB060,
    0xB0C8,
    0xD045, 0xD046, 0xD047,
    0xB540, 0xB541, 0xB542, 0xB544, 0xB545, 0xB546,
    0xB550, 0xB551, 0xB552, 0xB553, 0xB554, 0xB555, 0xB556, 0xB557,
    0xB640, 0xB641, 0xB642, 0xB644, 0xB645, 0xB646,
    0xB650, 0xB651, 0xB652, 0xB653, 0xB654, 0xB655, 0xB656, 0xB657,
    0xB740, 0xB741, 0xB742, 0xB744, 0xB745, 0xB746,
    0xB750, 0xB751, 0xB752, 0xB753, 0xB754, 0xB755, 0xB756, 0xB757,
};
#endif

static uint32_t const led_pattern_value[COLOR_PATTERN_MAX] = {
    0x000000FF,
    0x00FF0000,
    0x0000FF00,
    0x00FF00FF,
    0x0000FFFF,
    0x00FFFF00,
    0x00C0C0C0
};

static uint32_t const led_pattern_val_msb[COLOR_PATTERN_MAX][RGB_LED_MAX] = {
    {RGB_LED_PATTERN_VAL_MAX_B_MSB  ,RGB_LED_PATTERN_VAL_OFF_MSB   ,RGB_LED_PATTERN_VAL_OFF_MSB   },
    {RGB_LED_PATTERN_VAL_OFF_MSB    ,RGB_LED_PATTERN_VAL_OFF_MSB   ,RGB_LED_PATTERN_VAL_MAX_R_MSB },
    {RGB_LED_PATTERN_VAL_OFF_MSB    ,RGB_LED_PATTERN_VAL_MIN_G_MSB ,RGB_LED_PATTERN_VAL_OFF_MSB   },
    {RGB_LED_PATTERN_VAL_MID2_B_MSB ,RGB_LED_PATTERN_VAL_OFF_MSB   ,RGB_LED_PATTERN_VAL_MIN_R_MSB },
    {RGB_LED_PATTERN_VAL_MID1_B_MSB ,RGB_LED_PATTERN_VAL_MIN_G_MSB ,RGB_LED_PATTERN_VAL_OFF_MSB   },
    {RGB_LED_PATTERN_VAL_OFF_MSB    ,RGB_LED_PATTERN_VAL_MIN_G_MSB ,RGB_LED_PATTERN_VAL_MID_R_MSB },
    {RGB_LED_PATTERN_VAL_MIN_B_MSB  ,RGB_LED_PATTERN_VAL_MAX_G_MSB ,RGB_LED_PATTERN_VAL_MIN_R_MSB }
};

static uint32_t const led_pattern_val_lsb[COLOR_PATTERN_MAX][RGB_LED_MAX] = {
    {RGB_LED_PATTERN_VAL_MAX_B_LSB  ,RGB_LED_PATTERN_VAL_OFF_LSB   ,RGB_LED_PATTERN_VAL_OFF_LSB   },
    {RGB_LED_PATTERN_VAL_OFF_LSB    ,RGB_LED_PATTERN_VAL_OFF_LSB   ,RGB_LED_PATTERN_VAL_MAX_R_LSB },
    {RGB_LED_PATTERN_VAL_OFF_LSB    ,RGB_LED_PATTERN_VAL_MIN_G_LSB ,RGB_LED_PATTERN_VAL_OFF_LSB   },
    {RGB_LED_PATTERN_VAL_MID2_B_LSB ,RGB_LED_PATTERN_VAL_OFF_LSB   ,RGB_LED_PATTERN_VAL_MIN_R_LSB },
    {RGB_LED_PATTERN_VAL_MID1_B_LSB ,RGB_LED_PATTERN_VAL_MIN_G_LSB ,RGB_LED_PATTERN_VAL_OFF_LSB   },
    {RGB_LED_PATTERN_VAL_OFF_LSB    ,RGB_LED_PATTERN_VAL_MIN_G_LSB ,RGB_LED_PATTERN_VAL_MID_R_LSB },
    {RGB_LED_PATTERN_VAL_MIN_B_LSB  ,RGB_LED_PATTERN_VAL_MAX_G_LSB ,RGB_LED_PATTERN_VAL_MIN_R_LSB }

};

struct light_led_data_type {
    struct led_classdev    cdev;
    struct spmi_device     *spmi_dev;
    struct work_struct     work;
    u8                     num_leds;
    struct mutex           lock;
    uint32_t               ul_value;
    uint32_t               blink_control; 
    uint32_t               blink_ramp_duration;
    uint32_t               blink_low_pause_time;
    uint32_t               blink_high_pause_time;
    uint32_t               blink_off_color;
    uint8_t                rgb_led_val[RGB_LED_MAX];
};

typedef struct write_reg_data_init {
    u16    addr;
    u16    value;
} write_reg_led_data_init_type;

typedef struct _t_ledlight_ioctl {
    uint32_t data[LEDLIGHT_BLINK_NUM];
}T_LEDLIGHT_IOCTL;

typedef struct _t_ledlight_ioctl_dm {
    uint32_t dm_data;
}T_LEDLIGHT_IOCTL_DM;

static const write_reg_led_data_init_type backlight_init_reg_data[] = {
    { BACKLIGHT_REG_D848, BACKLIGHT_REG_D848_INIT },

    { BACKLIGHT_REG_D84C, BACKLIGHT_REG_D84C_INIT },
    { BACKLIGHT_REG_D84D, BACKLIGHT_REG_D84D_INIT },
    { BACKLIGHT_REG_D84E, BACKLIGHT_REG_D84E_INIT },
    { BACKLIGHT_REG_D84F, BACKLIGHT_REG_D84F_INIT },
    { BACKLIGHT_REG_D858, BACKLIGHT_REG_D858_INIT },

    { BACKLIGHT_REG_D860, BACKLIGHT_REG_D860_INIT },
    { BACKLIGHT_REG_D862, BACKLIGHT_REG_D862_INIT },
    { BACKLIGHT_REG_D863, BACKLIGHT_REG_D863_INIT },
    { BACKLIGHT_REG_D866, BACKLIGHT_REG_D866_INIT },
    { BACKLIGHT_REG_D870, BACKLIGHT_REG_D870_INIT },
    { BACKLIGHT_REG_D872, BACKLIGHT_REG_D872_INIT },
    { BACKLIGHT_REG_D873, BACKLIGHT_REG_D873_INIT },
    { BACKLIGHT_REG_D876, BACKLIGHT_REG_D876_INIT },
    { BACKLIGHT_REG_D880, BACKLIGHT_REG_D880_INIT },
    { BACKLIGHT_REG_D882, BACKLIGHT_REG_D882_INIT },
    { BACKLIGHT_REG_D883, BACKLIGHT_REG_D883_INIT },
    { BACKLIGHT_REG_D886, BACKLIGHT_REG_D886_INIT },
};

static const write_reg_led_data_init_type mobilelight_init_reg_data[] = {
    { MOBILELIGHT_REG_D341, MOBILELIGHT_REG_D341_INIT },
    { MOBILELIGHT_REG_D342, MOBILELIGHT_REG_D342_INIT },
    { MOBILELIGHT_REG_D343, MOBILELIGHT_REG_D343_INIT },
    { MOBILELIGHT_REG_D344, MOBILELIGHT_REG_D344_INIT },
    { MOBILELIGHT_REG_D346, MOBILELIGHT_REG_D346_INIT },
    { MOBILELIGHT_REG_D347, MOBILELIGHT_REG_D347_INIT },
    { MOBILELIGHT_REG_D348, MOBILELIGHT_REG_D348_INIT },
    { MOBILELIGHT_REG_D349, MOBILELIGHT_REG_D349_INIT },
    { MOBILELIGHT_REG_D34A, MOBILELIGHT_REG_D34A_INIT },
    { MOBILELIGHT_REG_D34B, MOBILELIGHT_REG_D34B_INIT },
    { MOBILELIGHT_REG_D34C, MOBILELIGHT_REG_D34C_INIT },
    { MOBILELIGHT_REG_D34D, MOBILELIGHT_REG_D34D_INIT },
    { MOBILELIGHT_REG_D34E, MOBILELIGHT_REG_D34E_INIT },
    { MOBILELIGHT_REG_D34F, MOBILELIGHT_REG_D34F_INIT },
    { MOBILELIGHT_REG_D351, MOBILELIGHT_REG_D351_INIT },
    { MOBILELIGHT_REG_D352, MOBILELIGHT_REG_D352_INIT },
};

static const write_reg_led_data_init_type rgb_led_init_reg_data[] = {
    { RGB_LED_B_REG_B541, RGB_LED_B_REG_B541_INIT },
    { RGB_LED_B_REG_B542, RGB_LED_B_REG_B542_INIT },
    { RGB_LED_G_REG_B641, RGB_LED_G_REG_B641_INIT },
    { RGB_LED_G_REG_B642, RGB_LED_G_REG_B642_INIT },
    { RGB_LED_R_REG_B741, RGB_LED_R_REG_B741_INIT },
    { RGB_LED_R_REG_B742, RGB_LED_R_REG_B742_INIT },
    { RGB_LED_REG_D045, RGB_LED_REG_D045_INIT },
    { RGB_LED_REG_D047, RGB_LED_REG_D047_INIT },
};

static uint32_t guc_light_dm = 0;

#ifndef DISABLE_DISP_DETECT
static atomic_t g_disp_status = ATOMIC_INIT(0);
static atomic_t g_display_detect = ATOMIC_INIT(0);
static struct mutex led_disp_lock;
#endif  /* DISABLE_DISP_DETECT */

static struct light_led_data_type *grgb_led = NULL;
static struct light_led_data_type *gbacklight = NULL;
static struct light_led_data_type *gmobilelight = NULL;

static struct regulator *boost_boost_enable;

static struct hrtimer mobilelight_wdt;

static int get_substrate_therm(enum power_supply_property get_therm)
{
    struct power_supply *battery_psy = NULL;
    union power_supply_propval ret = {0,};

    battery_psy = power_supply_get_by_name("battery");
    
    if(battery_psy)
        battery_psy->get_property(battery_psy, get_therm, &ret);

    return ret.intval;
}

static int led_reg_write(struct light_led_data_type *led, u16 addr, u8 val)
{
    int rc=0;
    int retry=0;

    do
    {
        rc = spmi_ext_register_writel(led->spmi_dev->ctrl, led->spmi_dev->sid, addr, &val, 1);
        if (rc) {
            DEBUG_PRINT_ERR("%s(): Unable to write to dev=[0x%08x] addr=[%04x] val=[0x%02x] rc=[%d] retry=[%d] \n",
                __func__, (unsigned int)&led->spmi_dev->dev, addr, val, rc, retry);
        }
    }while ((rc != SPMI_NO_ERROR) && (++retry < SPMI_RETRIES_NUM));

    DEBUG_PRINT("%s(): Write addr=[0x%04x] val=[0x%02x] \n", __func__, addr, val);

    return rc;
}

static int led_reg_read(struct light_led_data_type *led, u16 addr, u8 *val)
{
    int rc=0;
    int retry=0;
    u8 reg=0;

    do
    {
        rc = spmi_ext_register_readl(led->spmi_dev->ctrl, led->spmi_dev->sid, addr, &reg, 1);
        if (rc) {
            DEBUG_PRINT_ERR("%s(): Unable to read from dev=[0x%08x] addr=[%04x] val=[0x%02x] rc=[%d] retry=[%d] \n",
                __func__, (unsigned int)&led->spmi_dev->dev, addr, reg, rc, retry);
        }
    }while ((rc != SPMI_NO_ERROR) && (++retry < SPMI_RETRIES_NUM));

    *val = reg;

    DEBUG_PRINT("%s():  Read  addr=[0x%04x] val=[0x%02x] \n", __func__, addr, *val);

    return rc;
}

static int led_masked_reg_write(struct light_led_data_type *led, u16 addr, u8 mask, u8 val)
{
    int rc=0;
    u8 reg=0;

    rc = led_reg_read(led, addr, &reg);
    if (rc != SPMI_NO_ERROR)
        goto FAIL_SPMI_CONTROL;

    reg &= ~mask;
    reg |= val;

    rc = led_reg_write(led, addr, reg);
    if (rc != SPMI_NO_ERROR)
        goto FAIL_SPMI_CONTROL;

    return rc;

FAIL_SPMI_CONTROL:
    DEBUG_PRINT_ERR("%s(): Failed to SPMI control rc=[%d] \n", __func__, rc);
    return rc;
}

#if LED_DEBUG
static void led_dump_register(struct light_led_data_type *led, u16 regs[], u16 array_size)
{
    int i;
    u8 val;

    DEBUG_PRINT("%s(): start \n", __func__);

    DEBUG_PRINT("%s(): ===== [%s] register dump start ===== \n", __func__, led->cdev.name);
    for (i = 0; i < array_size; i++) {
        spmi_ext_register_readl(led->spmi_dev->ctrl,
                    led->spmi_dev->sid,
                    regs[i],
                    &val, sizeof(val));
        DEBUG_PRINT("%s(): addr=[0x%04x] val=[0x%02x] \n", __func__, regs[i], val);
    }
    DEBUG_PRINT("%s(): ===== [%s] register dump end ===== \n", __func__,led->cdev.name);

    DEBUG_PRINT("%s(): end \n", __func__);
}
#endif

static int32_t rgb_led_color_pattern_check(uint32_t ul_colorval)
{
    int32_t lret = COLOR_PATTERN_MAX;
    int32_t lmatch = 0;

    DEBUG_PRINT("%s(): start \n", __func__);

    for (lmatch = 0; lmatch < COLOR_PATTERN_MAX; lmatch++) {
        if (ul_colorval == led_pattern_value[lmatch]) {
            DEBUG_PRINT("%s(): color pattern match [%d] \n", __func__, lmatch);
            lret = lmatch;
            break;
        }
    }

    DEBUG_PRINT("%s(): end \n", __func__);
    return lret;
}

static int32_t rgb_led_lut_table_reg(struct light_led_data_type *led, uint32_t ul_pattern_val)
{
    int rc=0, i;
    uint32_t led_val[RGB_LED_MAX], led_off_val[RGB_LED_MAX];
    uint32_t ul_pattern_val_off;
    write_reg_led_data_init_type blink_col_table[LUT_TABLE_MAX * RGB_LED_MAX];
    
    DEBUG_PRINT("%s(): start \n", __func__);
    
    if (ul_pattern_val != COLOR_PATTERN_MAX) {
        for (i = 0; i < RGB_LED_MAX; i++) {
            blink_col_table[(i * LUT_TABLE_MAX) + LUT_MSB_L].addr  = LPG_LUT_MSB_L(i);
            blink_col_table[(i * LUT_TABLE_MAX) + LUT_MSB_L].value = led_pattern_val_msb[ul_pattern_val][i];
            
            blink_col_table[(i * LUT_TABLE_MAX) + LUT_LSB_L].addr  = LPG_LUT_LSB_L(i);
            blink_col_table[(i * LUT_TABLE_MAX) + LUT_LSB_L].value = led_pattern_val_lsb[ul_pattern_val][i];
        }
    } else {
        for (i = 0; i < RGB_LED_MAX; i++) {
            led_val[i]  = VALUE_TRANSFORMATION(RGB_LED_MAX_LEVEL, led->rgb_led_val[i],  RGB_LED_SINGLE_COL);
            
            blink_col_table[(i * LUT_TABLE_MAX) + LUT_MSB_L].addr  = LPG_LUT_MSB_L(i);
            blink_col_table[(i * LUT_TABLE_MAX) + LUT_MSB_L].value = GET_MSB(led_val[i]);
            
            blink_col_table[(i * LUT_TABLE_MAX) + LUT_LSB_L].addr  = LPG_LUT_LSB_L(i);
            blink_col_table[(i * LUT_TABLE_MAX) + LUT_LSB_L].value = GET_LSB(led_val[i]);
        }
    }
    ul_pattern_val_off = rgb_led_color_pattern_check(grgb_led->blink_off_color & RGB_LED_MIXED_COL);
    DEBUG_PRINT("%s(): Debug info ul_pattern_val_off=[%i] \n", __func__, ul_pattern_val_off);

    if (ul_pattern_val_off != COLOR_PATTERN_MAX) {
        for (i = 0; i < RGB_LED_MAX; i++) {
            blink_col_table[(i * LUT_TABLE_MAX) + LUT_MSB_H].addr  = LPG_LUT_MSB_H(i);
            blink_col_table[(i * LUT_TABLE_MAX) + LUT_MSB_H].value = led_pattern_val_msb[ul_pattern_val_off][i];
            
            blink_col_table[(i * LUT_TABLE_MAX) + LUT_LSB_H].addr  = LPG_LUT_LSB_H(i);
            blink_col_table[(i * LUT_TABLE_MAX) + LUT_LSB_H].value = led_pattern_val_lsb[ul_pattern_val_off][i];
        }

    } else {
        led_off_val[RGB_LED_R] = (uint8_t)((grgb_led->blink_off_color >> (RGB_LED_GET_COL_R)) & RGB_LED_SINGLE_COL);
        led_off_val[RGB_LED_G] = (uint8_t)((grgb_led->blink_off_color >> (RGB_LED_GET_COL_G)) & RGB_LED_SINGLE_COL);
        led_off_val[RGB_LED_B] = (uint8_t)((grgb_led->blink_off_color >> (RGB_LED_GET_COL_B)) & RGB_LED_SINGLE_COL);

        for (i = 0; i < RGB_LED_MAX; i++) {
            led_off_val[i]  = VALUE_TRANSFORMATION(RGB_LED_MAX_LEVEL, led_off_val[i],  RGB_LED_SINGLE_COL);
            
            blink_col_table[(i * LUT_TABLE_MAX) + LUT_MSB_H].addr  = LPG_LUT_MSB_H(i);
            blink_col_table[(i * LUT_TABLE_MAX) + LUT_MSB_H].value = GET_MSB(led_off_val[i]);
            
            blink_col_table[(i * LUT_TABLE_MAX) + LUT_LSB_H].addr  = LPG_LUT_LSB_H(i);
            blink_col_table[(i * LUT_TABLE_MAX) + LUT_LSB_H].value = GET_LSB(led_off_val[i]);
        }
    }

    for (i = 0; i < (LUT_TABLE_MAX * RGB_LED_MAX); i++) {
        rc = led_masked_reg_write(led, blink_col_table[i].addr, REG_MASK_VALUE, blink_col_table[i].value);
        if (rc != SPMI_NO_ERROR)
            goto FAIL_SPMI_CONTROL;
    }

    DEBUG_PRINT("%s(): end \n", __func__);

    return rc;

FAIL_SPMI_CONTROL:
    DEBUG_PRINT_ERR("%s(): Failed to SPMI control rc=[%d] \n", __func__, rc);
    return rc;
}
static int32_t rgb_led_blink_request(struct light_led_data_type *led, uint32_t hi_pause_code, uint32_t lo_pause_code, int col)
{
    int rc=0;

    DEBUG_PRINT("%s(): start \n", __func__);
    DEBUG_PRINT("%s(): Debug info Regster color [%d(R:2/G:1/B:0)] \n", __func__, col);

    rc = led_masked_reg_write(led, LPG_CHAN_LPG_PATTERN_CONFIG(col), REG_MASK_VALUE, LPG_CHAN_LPG_PATTERN_CONFIG_VAL);
    if (rc != SPMI_NO_ERROR)
        goto FAIL_SPMI_CONTROL;

    rc = led_masked_reg_write(led, LPG_CHAN_RAMP_STEP_DURATION_MSB(col), REG_MASK_VALUE, GET_MSB(grgb_led->blink_ramp_duration));
    if (rc != SPMI_NO_ERROR)
        goto FAIL_SPMI_CONTROL;

    rc = led_masked_reg_write(led, LPG_CHAN_RAMP_STEP_DURATION_LSB(col), REG_MASK_VALUE, GET_LSB(grgb_led->blink_ramp_duration));
    if (rc != SPMI_NO_ERROR)
        goto FAIL_SPMI_CONTROL;

    rc = led_masked_reg_write(led, LPG_CHAN_PAUSE_HI_MULTIPLIER_MSB(col), REG_MASK_VALUE, GET_MSB_5BIT(hi_pause_code));
    if (rc != SPMI_NO_ERROR)
        goto FAIL_SPMI_CONTROL;

    rc = led_masked_reg_write(led, LPG_CHAN_PAUSE_HI_MULTIPLIER_LSB(col), REG_MASK_VALUE, GET_LSB(hi_pause_code));
    if (rc != SPMI_NO_ERROR)
        goto FAIL_SPMI_CONTROL;

    rc = led_masked_reg_write(led, LPG_CHAN_PAUSE_LO_MULTIPLIER_MSB(col), REG_MASK_VALUE, GET_MSB_5BIT(lo_pause_code));
    if (rc != SPMI_NO_ERROR)
        goto FAIL_SPMI_CONTROL;

    rc = led_masked_reg_write(led, LPG_CHAN_PAUSE_LO_MULTIPLIER_LSB(col), REG_MASK_VALUE, GET_LSB(lo_pause_code));
    if (rc != SPMI_NO_ERROR)
        goto FAIL_SPMI_CONTROL;

    rc = led_masked_reg_write(led, LPG_CHAN_HI_INDEX(col), REG_MASK_VALUE, LPG_LUT_HI_INDEX_VAL(col));
    if (rc != SPMI_NO_ERROR)
        goto FAIL_SPMI_CONTROL;

    rc = led_masked_reg_write(led, LPG_CHAN_LO_INDEX(col), REG_MASK_VALUE, LPG_LUT_LO_INDEX_VAL(col));
    if (rc != SPMI_NO_ERROR)
        goto FAIL_SPMI_CONTROL;

    DEBUG_PRINT("%s(): end \n", __func__);

    return rc;

FAIL_SPMI_CONTROL:
    DEBUG_PRINT_ERR("%s(): Failed to SPMI control rc=[%d] \n", __func__, rc);
    return rc;
}

static int32_t rgb_led_light_on(int col, uint8_t msb, uint8_t lsb, struct light_led_data_type *led)
{
    int rc=0;

    DEBUG_PRINT("%s(): start \n", __func__);
    DEBUG_PRINT("%s(): Debug info Regster color [%d(R:2/G:1/B:0)] \n", __func__, col);

    rc = led_masked_reg_write(led, LPG_CHAN_ENABLE_CONTROL(col), REG_MASK_VALUE, LPG_CHAN_ENABLE_CONTROL_ON);
    if (rc != SPMI_NO_ERROR)
        goto FAIL_SPMI_CONTROL;

    rc = led_masked_reg_write(led, LPG_CHAN_PWM_VALUE_MSB(col), REG_MASK_VALUE, msb);
    if (rc != SPMI_NO_ERROR)
        goto FAIL_SPMI_CONTROL;

    rc = led_masked_reg_write(led, LPG_CHAN_PWM_VALUE_LSB(col), REG_MASK_VALUE, lsb);
    if (rc != SPMI_NO_ERROR)
        goto FAIL_SPMI_CONTROL;

    DEBUG_PRINT("%s(): end \n", __func__);

    return rc;

FAIL_SPMI_CONTROL:
    DEBUG_PRINT_ERR("%s(): Failed to SPMI control rc=[%d] \n", __func__, rc);
    return rc;
}

static int32_t rgb_led_light_off(struct light_led_data_type *led)
{
    int rc=0, i;

    DEBUG_PRINT("%s(): start \n", __func__);

    for (i = 0; i < RGB_LED_MAX; i++) {

        rc = led_masked_reg_write(led, LPG_CHAN_ENABLE_CONTROL(i), REG_MASK_VALUE, LPG_CHAN_ENABLE_CONTROL_OFF);
        if (rc != SPMI_NO_ERROR)
            goto FAIL_SPMI_CONTROL;

        rc = led_masked_reg_write(led, LPG_CHAN_PWM_VALUE_MSB(i), REG_MASK_VALUE, LPG_CHAN_PWM_VALUE_MSB_OFF);
        if (rc != SPMI_NO_ERROR)
            goto FAIL_SPMI_CONTROL;

        rc = led_masked_reg_write(led, LPG_CHAN_PWM_VALUE_LSB(i), REG_MASK_VALUE, LPG_CHAN_PWM_VALUE_LSB_OFF);
        if (rc != SPMI_NO_ERROR)
            goto FAIL_SPMI_CONTROL;

    }

    rc = led_reg_write(led, RGB_LED_REG_B0C8, LPG_LUT_RAMP_CONTROL_OFF);
    if (rc != SPMI_NO_ERROR)
        goto FAIL_SPMI_CONTROL;

    rc = led_masked_reg_write(led, RGB_LED_REG_D046, REG_MASK_VALUE, TRI_LED_EN_CTL_VAL_OFF);
    if (rc != SPMI_NO_ERROR)
        goto FAIL_SPMI_CONTROL;

    DEBUG_PRINT("%s(): end \n", __func__);

    return rc;

FAIL_SPMI_CONTROL:
    DEBUG_PRINT_ERR("%s(): Failed to SPMI control rc=[%d] \n", __func__, rc);
    return rc;
}

static int32_t rgb_led_light_set(struct light_led_data_type *led)
{
    int rc=0, i;
    uint8_t ctl_val=0, lut_val = 0;
    uint32_t led_val[RGB_LED_MAX];
    uint32_t hi_pause_code, lo_pause_code;
    uint32_t ul_pattern_val;

    DEBUG_PRINT("%s(): start \n", __func__);

    ul_pattern_val = rgb_led_color_pattern_check(led->ul_value & RGB_LED_MIXED_COL);
    DEBUG_PRINT("%s(): Debug info ul_pattern_val=[%i] \n", __func__, ul_pattern_val);

    if (led->blink_control == BLINK_REQUEST) {
        rc = rgb_led_lut_table_reg(led, ul_pattern_val);
        if (rc != SPMI_NO_ERROR)
            goto FAIL_SPMI_CONTROL;

        hi_pause_code = (grgb_led->blink_high_pause_time / grgb_led->blink_ramp_duration);
        lo_pause_code = (grgb_led->blink_low_pause_time  / grgb_led->blink_ramp_duration);

        if (hi_pause_code != 0)
            hi_pause_code = hi_pause_code - 1;

        if (lo_pause_code != 0)
            lo_pause_code = lo_pause_code - 1;

        if (hi_pause_code > MAX_PAUSE_CODE)
            hi_pause_code = MAX_PAUSE_CODE;

        if (lo_pause_code > MAX_PAUSE_CODE)
            lo_pause_code = MAX_PAUSE_CODE;

        DEBUG_PRINT("%s(): Debug info hi_pause_code=[%i] lo_pause_code=[%i]", __func__, hi_pause_code, lo_pause_code);

        if ((led->ul_value & RGB_LED_SINGLE_COL_B) || (grgb_led->blink_off_color & RGB_LED_SINGLE_COL_B)) {

            rc = rgb_led_blink_request(led, hi_pause_code, lo_pause_code, RGB_LED_B);
            if (rc != SPMI_NO_ERROR)
                goto FAIL_SPMI_CONTROL;

            rc = led_masked_reg_write(led, LPG_CHAN_ENABLE_CONTROL(RGB_LED_B), REG_MASK_VALUE, LPG_CHAN_ENABLE_CONTROL_BLINK);
            if (rc != SPMI_NO_ERROR)
                goto FAIL_SPMI_CONTROL;

            ctl_val |= TRI_LED_EN_CTL_VAL(RGB_LED_B);
            lut_val |= LPG_LUT_RAMP_CONTROL(RGB_LED_B);
        }

        if ((led->ul_value & RGB_LED_SINGLE_COL_G) || (grgb_led->blink_off_color & RGB_LED_SINGLE_COL_G)) {

            rc = rgb_led_blink_request(led, hi_pause_code, lo_pause_code, RGB_LED_G);
            if (rc != SPMI_NO_ERROR)
                goto FAIL_SPMI_CONTROL;

            rc = led_masked_reg_write(led, LPG_CHAN_ENABLE_CONTROL(RGB_LED_G), REG_MASK_VALUE, LPG_CHAN_ENABLE_CONTROL_BLINK);
            if (rc != SPMI_NO_ERROR)
                goto FAIL_SPMI_CONTROL;

            ctl_val |= TRI_LED_EN_CTL_VAL(RGB_LED_G);
            lut_val |= LPG_LUT_RAMP_CONTROL(RGB_LED_G);
        }

        if ((led->ul_value & RGB_LED_SINGLE_COL_R) || (grgb_led->blink_off_color & RGB_LED_SINGLE_COL_R)) {

            rc = rgb_led_blink_request(led, hi_pause_code, lo_pause_code, RGB_LED_R);
            if (rc != SPMI_NO_ERROR)
                goto FAIL_SPMI_CONTROL;

            rc = led_masked_reg_write(led, LPG_CHAN_ENABLE_CONTROL(RGB_LED_R), REG_MASK_VALUE, LPG_CHAN_ENABLE_CONTROL_BLINK);
            if (rc != SPMI_NO_ERROR)
                goto FAIL_SPMI_CONTROL;

            ctl_val |= TRI_LED_EN_CTL_VAL(RGB_LED_R);
            lut_val |= LPG_LUT_RAMP_CONTROL(RGB_LED_R);
        }

        if (ctl_val == 0 ) {
            DEBUG_PRINT_ERR("%s(): TRI_LED_EN_CTL(0x1D046) Must be set a value \n", __func__);
            rc = -1;
            goto PARAMETER_ERR;
        } else {
            rc = led_masked_reg_write(led, RGB_LED_REG_D046, REG_MASK_VALUE, ctl_val);
            if (rc != SPMI_NO_ERROR)
                goto FAIL_SPMI_CONTROL;
        }

        if (lut_val == 0) {
            DEBUG_PRINT_ERR("%s(): LPG_LUT_RAMP_CONTROL(0x1B0C8) Must be set a value \n", __func__);
            rc = -1;
            goto PARAMETER_ERR;
        } else {
            rc = led_reg_write(led, RGB_LED_REG_B0C8, lut_val);
            if (rc != SPMI_NO_ERROR)
                goto FAIL_SPMI_CONTROL;
        }
    } 
    else {
        if (ul_pattern_val != COLOR_PATTERN_MAX) {
            for(i =0; i < RGB_LED_MAX; i++) {
                rc = rgb_led_light_on(i, led_pattern_val_msb[ul_pattern_val][i], led_pattern_val_lsb[ul_pattern_val][i], led);
                if (rc != SPMI_NO_ERROR)
                    goto FAIL_SPMI_CONTROL;

                if ((led_pattern_val_msb[ul_pattern_val][i] != RGB_LED_PATTERN_VAL_OFF_MSB) ||
                    (led_pattern_val_lsb[ul_pattern_val][i] != RGB_LED_PATTERN_VAL_OFF_LSB))
                        ctl_val |= TRI_LED_EN_CTL_VAL(i);
            }
        } else {
            for (i = 0; i < RGB_LED_MAX; i++) {

                led_val[i]  = VALUE_TRANSFORMATION(RGB_LED_MAX_LEVEL, led->rgb_led_val[i],  RGB_LED_SINGLE_COL);
                DEBUG_PRINT("%s(): Debug info RGB_LED=[%d(R:2/G:1/B:0)] Value=[0x%04X] \n", __func__, i, led_val[i]);

                rc = rgb_led_light_on(i, GET_MSB(led_val[i]), GET_LSB(led_val[i]), led);
                if (rc != SPMI_NO_ERROR)
                    goto FAIL_SPMI_CONTROL;

                if (led_val[i] != LED_COL_BLACK)
                    ctl_val |= TRI_LED_EN_CTL_VAL(i);
            }
        }

        if (ctl_val == 0) {
            DEBUG_PRINT_ERR("%s(): TRI_LED_EN_CTL(0x1D046) Must be set a value \n", __func__);
            rc = -1;
            goto PARAMETER_ERR;
        } else {
            rc = led_masked_reg_write(led, RGB_LED_REG_D046, REG_MASK_VALUE, ctl_val);
            if (rc != SPMI_NO_ERROR)
                goto FAIL_SPMI_CONTROL;
        }
    }

    DEBUG_PRINT("%s(): end \n", __func__);

    return rc;

PARAMETER_ERR:
    DEBUG_PRINT_ERR("%s(): Error end rc=[%d] \n", __func__, rc);
    return rc;

FAIL_SPMI_CONTROL:
    DEBUG_PRINT_ERR("%s(): Failed to SPMI control rc=[%d] \n", __func__, rc);
    return rc;
}

static int rgb_led_set(struct light_led_data_type *led)
{
    int rc=0;

    mutex_lock(&grgb_led->lock);
    DEBUG_PRINT("%s(): mutex_lock \n", __func__);

    DEBUG_PRINT("%s(): start Value=[0x%08x] \n", __func__, led->ul_value);

    rc = rgb_led_light_off(led);
    if (rc != SPMI_NO_ERROR)
        goto FAIL_SPMI_CONTROL;

    if (LED_COL_BLACK != (led->ul_value & RGB_LED_MIXED_COL)){
        led->rgb_led_val[RGB_LED_R] = (u8)((led->ul_value >> RGB_LED_GET_COL_R) & RGB_LED_SINGLE_COL);
        led->rgb_led_val[RGB_LED_G] = (u8)((led->ul_value >> RGB_LED_GET_COL_G) & RGB_LED_SINGLE_COL);
        led->rgb_led_val[RGB_LED_B] = (u8)((led->ul_value >> RGB_LED_GET_COL_B) & RGB_LED_SINGLE_COL);

        DEBUG_PRINT("%s(): RGB color Shift and Masked R:[0x%04x] G:[0x%04x] B:[0x%04x] \n", 
            __func__, led->rgb_led_val[RGB_LED_R], led->rgb_led_val[RGB_LED_G], led->rgb_led_val[RGB_LED_B]);

        rc = rgb_led_light_set(led);
        if (rc != SPMI_NO_ERROR)
            goto FAIL_SPMI_CONTROL;
    }

    mutex_unlock(&grgb_led->lock);
    DEBUG_PRINT("%s(): mutex_unlock \n", __func__);

    DEBUG_PRINT("%s(): end \n", __func__);

    return rc;

FAIL_SPMI_CONTROL:
    mutex_unlock(&grgb_led->lock);
    DEBUG_PRINT("%s(): mutex_unlock \n", __func__);
    DEBUG_PRINT_ERR("%s(): Failed to SPMI control rc=[%d] \n", __func__, rc);
    return rc;
}

static enum hrtimer_restart mobilelight_wdt_reset(struct hrtimer *timer)
{
    int rc=0;

    DEBUG_PRINT("%s(): start \n", __func__);

    rc = led_reg_write(gmobilelight, MOBILELIGHT_REG_D356, MOBILELIGHT_REG_D356_RESET_WDT);
    if (rc != SPMI_NO_ERROR)
        goto FAIL_SPMI_CONTROL;

    hrtimer_start(&mobilelight_wdt, ktime_set(MOBILELIGHT_WDT_RESET_TIME, 0), HRTIMER_MODE_REL);

    DEBUG_PRINT("%s(): Mobilelight WatchDogTimer Reset and Timer Restart \n", __func__);

    DEBUG_PRINT("%s(): end \n", __func__);

    return HRTIMER_NORESTART;

FAIL_SPMI_CONTROL:
    DEBUG_PRINT_ERR("%s(): Failed to SPMI control rc=[%d] \n", __func__, rc);
    DEBUG_PRINT_ERR("%s(): Mobilelight WatchDogTimer can't reset and timer not restart \n", __func__);
    return HRTIMER_NORESTART;
}

static int mobilelight_set(struct light_led_data_type *led)
{
    int rc=0;
    uint8_t uc_mobilelight_val;
    int32_t camera_temp = 0;
    char *err_msg="SPMI";
    static int32_t mobilelight_status = MOBILELIGHT_OFF;

    DEBUG_PRINT("%s(): start mobilelight_status[%d]\n", __func__, mobilelight_status);

    mutex_lock(&gmobilelight->lock);
    DEBUG_PRINT("%s(): mutex_lock \n", __func__);

    if (led->ul_value > LED_COL_BLACK) {
        if (mobilelight_status == MOBILELIGHT_OFF) {

            DEBUG_PRINT("%s(): dm_val=[%d] \n", __func__, guc_light_dm);
            if (guc_light_dm == 1) {
                uc_mobilelight_val = MOBILELIGHT_CAMERATERM_HIGH_VAL;
                DEBUG_PRINT("%s(): mobilelight_val=[%d] \n", __func__, uc_mobilelight_val);
            }
            else if (guc_light_dm == 2) {
                uc_mobilelight_val = MOBILELIGHT_CAMERATERM_LOW_VAL;
                DEBUG_PRINT("%s(): mobilelight_val=[%d] \n", __func__, uc_mobilelight_val);
            }
            else {
                camera_temp = get_substrate_therm(POWER_SUPPLY_PROP_OEM_CAMERA_THERM);
                DEBUG_PRINT("%s(): Get CameraTemp=[%d] \n", __func__, camera_temp);

                if (camera_temp >= MOBILELIGHT_THRESHOLD_TEMPVAL) {
                    DEBUG_PRINT_ERR("%s(): CameraTemp is over threshold. CameraTemp=[%d] \n", __func__, camera_temp);
                    uc_mobilelight_val = MOBILELIGHT_CAMERATERM_HIGH_VAL;
                }
                else {
                    uc_mobilelight_val = MOBILELIGHT_CAMERATERM_LOW_VAL;
                }

                DEBUG_PRINT("%s(): uc_mobilelight_val=[%d] \n",  __func__, uc_mobilelight_val);
            }

            if (uc_mobilelight_val == MOBILELIGHT_CAMERATERM_LOW_VAL) {
                rc = led_masked_reg_write(led, MOBILELIGHT_REG_D342, REG_MASK_VALUE, MOBILELIGHT_REG_D342_ON_LOW);
                if (rc != SPMI_NO_ERROR)
                    goto FAIL_SPMI_CONTROL;

                rc = led_masked_reg_write(led, MOBILELIGHT_REG_D343, REG_MASK_VALUE, MOBILELIGHT_REG_D343_ON_LOW);
                if (rc != SPMI_NO_ERROR)
                    goto FAIL_SPMI_CONTROL;

                rc = led_masked_reg_write(led, MOBILELIGHT_REG_D34D, REG_MASK_VALUE, MOBILELIGHT_REG_D34D_ON_LOW);
                if (rc != SPMI_NO_ERROR)
                    goto FAIL_SPMI_CONTROL;

                rc = led_masked_reg_write(led, MOBILELIGHT_REG_D34E, REG_MASK_VALUE, MOBILELIGHT_REG_D34E_ON_LOW);
                if (rc != SPMI_NO_ERROR)
                    goto FAIL_SPMI_CONTROL;
            } else {
                rc = led_masked_reg_write(led, MOBILELIGHT_REG_D342, REG_MASK_VALUE, MOBILELIGHT_REG_D342_ON_HIGH);
                if (rc != SPMI_NO_ERROR)
                    goto FAIL_SPMI_CONTROL;

                rc = led_masked_reg_write(led, MOBILELIGHT_REG_D343, REG_MASK_VALUE, MOBILELIGHT_REG_D343_ON_HIGH);
                if (rc != SPMI_NO_ERROR)
                    goto FAIL_SPMI_CONTROL;

                rc = led_masked_reg_write(led, MOBILELIGHT_REG_D34D, REG_MASK_VALUE, MOBILELIGHT_REG_D34D_ON_HIGH);
                if (rc != SPMI_NO_ERROR)
                    goto FAIL_SPMI_CONTROL;

                rc = led_masked_reg_write(led, MOBILELIGHT_REG_D34E, REG_MASK_VALUE, MOBILELIGHT_REG_D34E_ON_HIGH);
                if (rc != SPMI_NO_ERROR)
                    goto FAIL_SPMI_CONTROL;
            }

            rc = regulator_enable(boost_boost_enable);
            if (rc != 0) {
                err_msg = "Regulator";
                goto FAIL_SPMI_CONTROL;
            }

            rc = led_masked_reg_write(led, MOBILELIGHT_REG_D346, REG_MASK_VALUE, MOBILELIGHT_REG_D346_ON);
            if (rc != SPMI_NO_ERROR)
                goto FAIL_SPMI_CONTROL;

            rc = led_masked_reg_write(led, MOBILELIGHT_REG_D347, REG_MASK_VALUE, MOBILELIGHT_REG_D347_ON);
            if (rc != SPMI_NO_ERROR)
                goto FAIL_SPMI_CONTROL;

            hrtimer_start(&mobilelight_wdt, ktime_set(MOBILELIGHT_WDT_RESET_TIME, 0), HRTIMER_MODE_REL);

            DEBUG_PRINT("%s(): Mobilelight WatchDogTimer start \n", __func__);

            mobilelight_status = MOBILELIGHT_ON;
        } else {
            DEBUG_PRINT("%s(): Mobilelight ON skip \n", __func__);
        }
    } else {
        if (mobilelight_status == MOBILELIGHT_ON) {

            hrtimer_cancel(&mobilelight_wdt);
            DEBUG_PRINT("%s(): Mobilelight WatchDogTimer cancel \n", __func__);

            rc = led_masked_reg_write(led, MOBILELIGHT_REG_D342, REG_MASK_VALUE, MOBILELIGHT_REG_D342_OFF);
            if (rc != SPMI_NO_ERROR)
                goto FAIL_SPMI_CONTROL;

            rc = led_masked_reg_write(led, MOBILELIGHT_REG_D343, REG_MASK_VALUE, MOBILELIGHT_REG_D343_OFF);
            if (rc != SPMI_NO_ERROR)
                goto FAIL_SPMI_CONTROL;

            rc = led_masked_reg_write(led, MOBILELIGHT_REG_D34D, REG_MASK_VALUE, MOBILELIGHT_REG_D34D_OFF);
            if (rc != SPMI_NO_ERROR)
                goto FAIL_SPMI_CONTROL;

            rc = led_masked_reg_write(led, MOBILELIGHT_REG_D34E, REG_MASK_VALUE, MOBILELIGHT_REG_D34E_OFF);
            if (rc != SPMI_NO_ERROR)
                goto FAIL_SPMI_CONTROL;

            rc = led_masked_reg_write(led, MOBILELIGHT_REG_D347, REG_MASK_VALUE, MOBILELIGHT_REG_D347_OFF);
            if (rc != SPMI_NO_ERROR)
                goto FAIL_SPMI_CONTROL;

            rc = led_masked_reg_write(led, MOBILELIGHT_REG_D346, REG_MASK_VALUE, MOBILELIGHT_REG_D346_OFF);
            if (rc != SPMI_NO_ERROR)
                goto FAIL_SPMI_CONTROL;

            rc = regulator_disable(boost_boost_enable);
            if (rc != 0) {
                err_msg = "Regulator";
                goto FAIL_SPMI_CONTROL;
            }

            mobilelight_status = MOBILELIGHT_OFF;
        } else {
            DEBUG_PRINT("%s(): Mobilelight OFF skip \n", __func__);
        }
    }

    mutex_unlock(&gmobilelight->lock);
    DEBUG_PRINT("%s(): mutex_unlock \n", __func__);

    DEBUG_PRINT("%s(): end \n", __func__);

    return rc;

FAIL_SPMI_CONTROL:
    mutex_unlock(&gmobilelight->lock);
    DEBUG_PRINT("%s(): mutex_unlock \n", __func__);
    DEBUG_PRINT_ERR("%s(): Failed to %s control rc=[%d] \n", __func__, err_msg, rc);
    return rc;
}

static int backlight_switch(struct light_led_data_type *led, uint8_t val)
{
    int rc=0;

    DEBUG_PRINT("%s(): start switch to [0x%02x(ON:0x80/OFF:0x00)] \n", __func__, val);

    rc = led_masked_reg_write(led, BACKLIGHT_REG_D847, REG_MASK_VALUE, BACKLIGHT_REG_D847_ON);
    if (rc != SPMI_NO_ERROR)
        goto FAIL_SPMI_CONTROL;

    rc = led_masked_reg_write(led, BACKLIGHT_REG_D847, REG_MASK_VALUE, BACKLIGHT_REG_D847_OFF);
    if (rc != SPMI_NO_ERROR)
        goto FAIL_SPMI_CONTROL;

    udelay(10);
    rc = led_masked_reg_write(led, BACKLIGHT_REG_D846, REG_MASK_VALUE, val);
    if (rc != SPMI_NO_ERROR)
        goto FAIL_SPMI_CONTROL;

    DEBUG_PRINT("%s(): end \n", __func__);

    return rc;

FAIL_SPMI_CONTROL:
    DEBUG_PRINT_ERR("%s(): Failed to SPMI control rc=[%d] \n", __func__, rc);
    return rc;
}

static int backlight_set(struct light_led_data_type *led)
{
    int32_t rc=0, level;
    uint8_t i;
    uint8_t uc_backlightval;
    uint32_t ul_val;
    static int32_t backlight_status = BACKLIGHT_OFF;
    int32_t batt_temp = 0;
#ifndef DISABLE_DISP_DETECT
    int32_t display_detect;
#endif  /* DISABLE_DISP_DETECT */

    DEBUG_PRINT("%s(): start backlight_status=[%d] \n", __func__, backlight_status);

    mutex_lock(&gbacklight->lock);
    DEBUG_PRINT("%s(): mutex_lock \n", __func__);

    ul_val = (led->ul_value & 0xffffffff);
    uc_backlightval  = (u8)(ul_val & BACKLIGHT_BASE_LEVEL);
    DEBUG_PRINT("%s(): backlight val=[%d] \n", __func__, uc_backlightval);

#ifndef DISABLE_DISP_DETECT
    display_detect = atomic_read(&g_display_detect);
#endif  /* DISABLE_DISP_DETECT */

    level = VALUE_TRANSFORMATION(BACKLIGHT_MAX_LEVEL, uc_backlightval, BACKLIGHT_BASE_LEVEL);

    if (backlight_status == BACKLIGHT_OFF) {
        if (level >= BACKLIGHT_THRESHOLD_CURVAL) {
            batt_temp = get_substrate_therm(POWER_SUPPLY_PROP_OEM_SUBSTRATE_THERM);
            DEBUG_PRINT("%s(): Get BatteryTemp=[%d] \n", __func__, batt_temp);

            if (batt_temp >= BACKLIGHT_THRESHOLD_TEMPVAL) {
                level = BACKLIGHT_THRESHOLD_CURVAL;
                DEBUG_PRINT_ERR("%s(): BatteryTemp is over threshold. BatteryTemp=[%d] level=[%d] \n", __func__, batt_temp, level);
            }

            if (guc_light_dm == 1) {
                level = BACKLIGHT_THRESHOLD_CURVAL;
                DEBUG_PRINT("%s(): backlight val=[%d] dm=[%d] \n", __func__, level, guc_light_dm);
            }
            else if(guc_light_dm == 2) {
                level = batt_temp;
                DEBUG_PRINT("%s(): backlight val=[%d] dm=[%d] \n", __func__, level, guc_light_dm);
            }
        }
    }

    /* program brightness control registers */
    for (i = 0; i < BACKLIGHT_NUM; i++) {
        rc = led_masked_reg_write(led, BACKLIGHT_BRIGHTNESS_SETTING_MSB(i), REG_MASK_VALUE, GET_MSB(level));
        if (rc != SPMI_NO_ERROR)
            goto FAIL_SPMI_CONTROL;

        rc = led_masked_reg_write(led, BACKLIGHT_BRIGHTNESS_SETTING_LSB(i), REG_MASK_VALUE, GET_LSB(level));
        if (rc != SPMI_NO_ERROR)
            goto FAIL_SPMI_CONTROL;
    }

    if (uc_backlightval == LED_COL_BLACK) {
#ifndef DISABLE_DISP_DETECT
        if (display_detect == 0) {
            rc = light_led_disp_set(LIGHT_MAIN_WLED_LED_DIS);
        }
        else if (display_detect == 1) {
#endif  /* DISABLE_DISP_DETECT */
            rc = backlight_switch(led, BACKLIGHT_REG_D846_OFF);

#ifndef DISABLE_DISP_DETECT
        }
#endif  /* DISABLE_DISP_DETECT */
        if (rc != SPMI_NO_ERROR)
            goto FAIL_SPMI_CONTROL;

        backlight_status = BACKLIGHT_OFF;
    } else {
#ifndef DISABLE_DISP_DETECT
        if (display_detect == 0) {
            rc = light_led_disp_set(LIGHT_MAIN_WLED_LED_EN);
            if (rc != SPMI_NO_ERROR)
                goto FAIL_SPMI_CONTROL;
        }
        else if(display_detect == 1) {
#endif  /* DISABLE_DISP_DETECT */
            rc = backlight_switch(led, BACKLIGHT_REG_D846_ON);
            if (rc != SPMI_NO_ERROR)
                goto FAIL_SPMI_CONTROL;
#ifndef DISABLE_DISP_DETECT
        }
        else {
            DEBUG_PRINT_ERR("%s(): No set display display_detect=[%x] \n", __func__, (int32_t)display_detect);
        }
#endif  /* DISABLE_DISP_DETECT */
        backlight_status = BACKLIGHT_ON;
    }

    mutex_unlock(&gbacklight->lock);
    DEBUG_PRINT("%s(): mutex_unlock \n", __func__);

    DEBUG_PRINT("%s(): end \n", __func__);

    return rc;

FAIL_SPMI_CONTROL:
    mutex_unlock(&gbacklight->lock);
    DEBUG_PRINT("%s(): mutex_unlock \n", __func__);
    DEBUG_PRINT_ERR("%s(): Failed to SPMI control rc=[%d] \n", __func__, rc);
    return rc;
}

static long leds_ioctl (struct file *filp, unsigned int cmd, unsigned long arg)
{
    void __user *argp = (void __user *)arg;
    int32_t ret = -1;
    T_LEDLIGHT_IOCTL st_ioctl;
    T_LEDLIGHT_IOCTL_DM st_ioctl_dm;

    DEBUG_PRINT("%s(): start \n",__func__);
    switch (cmd) {
    case LEDLIGHT_SET_BLINK:
        DEBUG_PRINT("%s(): LEDLIGHT_SET_BLINK \n",__func__);
        ret = copy_from_user(&st_ioctl,
                    argp,
                    sizeof(T_LEDLIGHT_IOCTL));
        if (ret) {
            DEBUG_PRINT_ERR("%s(): Error leds_ioctl(cmd = LEDLIGHT_SET_BLINK) \n", __func__);
            return -EFAULT;
        }
        DEBUG_PRINT("%s(): st_ioctl data[0]=[%d] data[1]=[%d] data[2]=[%d] data[3]=[0x%08x] \n",
            __func__, st_ioctl.data[0], st_ioctl.data[1], st_ioctl.data[2], st_ioctl.data[3]);

        mutex_lock(&grgb_led->lock);
        DEBUG_PRINT("%s(): mutex_lock \n", __func__);

        grgb_led->blink_control         = st_ioctl.data[0];
        grgb_led->blink_ramp_duration   = RGB_LED_RAMP_DURATION;
        grgb_led->blink_low_pause_time  = st_ioctl.data[1];
        grgb_led->blink_high_pause_time = st_ioctl.data[2];
        grgb_led->blink_off_color       = st_ioctl.data[3];

        DEBUG_PRINT("%s(): grgb_led blink_control=[%d] blink_ramp_duration=[%d] blink_low_pause_time=[%d] blink_high_pause_time=[%d] blink_off_color=[0x%08x] \n",
            __func__, grgb_led->blink_control, grgb_led->blink_ramp_duration, grgb_led->blink_low_pause_time, grgb_led->blink_high_pause_time, grgb_led->blink_off_color);

        mutex_unlock(&grgb_led->lock);
        DEBUG_PRINT("%s(): mutex_unlock \n", __func__);
        break;
    case LEDLIGHT_SET_TEMPERTURE_DM:
        DEBUG_PRINT("%s(): LEDLIGHT_SET_TEMPERTURE_DM \n",__func__);
        ret = copy_from_user(&st_ioctl_dm,
                    argp,
                    sizeof(T_LEDLIGHT_IOCTL_DM));
        if (ret) {
            DEBUG_PRINT_ERR("%s(): Error  st_ioctl_dm(cmd = LEDLIGHT_SET_TEMPERTURE_DM) \n", __func__);
            return -EFAULT;
        }
        mutex_lock(&grgb_led->lock);
        DEBUG_PRINT("%s(): mutex_lock \n", __func__);
        guc_light_dm = st_ioctl_dm.dm_data;
        DEBUG_PRINT("%s(): guc_light_dm=[%d] \n", __func__, guc_light_dm);
        mutex_unlock(&grgb_led->lock);
        DEBUG_PRINT("%s(): mutex_unlock \n", __func__);
        break;
    default:
        DEBUG_PRINT("%s(): default \n", __func__);
        return -ENOTTY;
    }

    DEBUG_PRINT("%s(): end \n", __func__);

    return 0;
}

static void led_work(struct work_struct *work)
{
    int rc;
    struct light_led_data_type *led;

    DEBUG_PRINT("%s(): start \n", __func__);

    if (work == NULL) {
        DEBUG_PRINT_ERR("%s(): Error work is NULL \n", __func__);
        return;
    }

    led = container_of(work, struct light_led_data_type, work);
    if ((led != grgb_led) && (led != gbacklight) && (led != gmobilelight)) {
        DEBUG_PRINT_ERR("%s(): Error in container_of led=[0x%08x] \n", __func__, (unsigned int)led);
        return;
    }

    DEBUG_PRINT("%s(): LED set info Dev=[0x%08x] DevName=[%s] Value=[0x%08x] \n", 
        __func__, (unsigned int)&led->spmi_dev->dev, led->cdev.name, led->ul_value);

    if (!strcmp(led->cdev.name, RGB_LED_INFO)) {
        rc = rgb_led_set(led);
        if (rc != SPMI_NO_ERROR)
            DEBUG_PRINT_ERR("%s(): RGB_LED set brightness failed rc=[%d] Dev=[0x%08x] \n", __func__, rc, (unsigned int)&led->spmi_dev->dev);
    } else if (!strcmp(led->cdev.name, MOBILELIGHT_INFO)) {
        rc = mobilelight_set(led);
        if (rc != SPMI_NO_ERROR)
            DEBUG_PRINT_ERR("%s(): MOBILELIGHT set brightness failed rc=[%d] Dev=[0x%08x] \n", __func__, rc, (unsigned int)&led->spmi_dev->dev);
    } else if (!strcmp(led->cdev.name, BACKLIGHT_INFO)) {
        rc = backlight_set(led);
        if (rc != SPMI_NO_ERROR)
            DEBUG_PRINT_ERR("%s(): BACKLIGHT set brightness failed rc=[%d] Dev=[0x%08x] \n", __func__, rc, (unsigned int)&led->spmi_dev->dev);
    } else {
        DEBUG_PRINT_ERR("%s(): No LED matched DevName=[%s] \n", __func__, led->cdev.name);
    }

    DEBUG_PRINT("%s(): end \n", __func__);
}

static void led_set(struct led_classdev *led_cdev,
                enum led_brightness value)
{
    struct light_led_data_type *led;

    DEBUG_PRINT("%s(): start value=[0x%08x] \n", __func__, value);

    if (led_cdev == NULL) {
        DEBUG_PRINT_ERR("%s(): Error led_cdev is NULL \n", __func__);
        return;
    }

    led = container_of(led_cdev, struct light_led_data_type, cdev);
    if ((led != grgb_led) && (led != gbacklight) && (led != gmobilelight)) {
        DEBUG_PRINT_ERR("%s(): Error in container_of led=[0x%08x] \n", __func__, (unsigned int)led);
        return;
    }

    if ((value < LED_OFF) || (value > led->cdev.max_brightness)) {
        DEBUG_PRINT_ERR("%s(): Invalid brightness value=[%d] Dev=[0x%08x] \n", __func__, value, (unsigned int)&led->spmi_dev->dev);
        return;
    }

    mutex_lock(&grgb_led->lock);
    DEBUG_PRINT("%s(): mutex_lock \n", __func__);

    led->ul_value = value;

    DEBUG_PRINT("%s(): LED set info Dev=[0x%08x] DevName=[%s] Value=[0x%08x] \n", 
        __func__, (unsigned int)&led->spmi_dev->dev, led->cdev.name, led->ul_value);

    mutex_unlock(&grgb_led->lock);
    DEBUG_PRINT("%s(): mutex_unlock \n", __func__);

    schedule_work(&led->work);

    DEBUG_PRINT("%s(): end \n", __func__);
}

static enum led_brightness led_get(struct led_classdev *led_cdev)
{
    struct light_led_data_type *led;

    DEBUG_PRINT("%s(): start \n", __func__);

    if (led_cdev == NULL) {
        DEBUG_PRINT_ERR("%s(): led_cdev is NULL \n", __func__);
        return 0;
    }

    led = container_of(led_cdev, struct light_led_data_type, cdev);
    if ((led != grgb_led) && (led != gbacklight) && (led != gmobilelight)) {
        DEBUG_PRINT_ERR("%s(): Error in container_of led=[0x%08x] \n", __func__, (unsigned int)led);
        return 0;
    }

    DEBUG_PRINT("%s(): end value=[%i] \n", __func__, led->ul_value);

    return led->ul_value;
}

static int __devinit rgb_led_init(struct light_led_data_type *led)
{
    int rc=0, i;

    DEBUG_PRINT("%s(): start \n", __func__);

    for (i = 0; i < (sizeof(rgb_led_init_reg_data)/sizeof(write_reg_led_data_init_type)); i++) {
        rc = led_masked_reg_write(led, rgb_led_init_reg_data[i].addr, REG_MASK_VALUE, rgb_led_init_reg_data[i].value);
        if (rc != SPMI_NO_ERROR)
            goto FAIL_SPMI_CONTROL;
    }

    LED_DUMP_REGISTER(led, rgb_led_debug_regs, ARRAY_SIZE(rgb_led_debug_regs));

    DEBUG_PRINT("%s(): end \n", __func__);

    return rc;

FAIL_SPMI_CONTROL:
    DEBUG_PRINT_ERR("%s(): Failed to SPMI control rc=[%d] \n", __func__, rc);
    return rc;
}

static int __devinit mobilelight_init(struct light_led_data_type *led)
{
    int rc=0, i;

    DEBUG_PRINT("%s(): start \n", __func__);

    for (i = 0; i < (sizeof(mobilelight_init_reg_data)/sizeof(write_reg_led_data_init_type)); i++) {
        rc = led_masked_reg_write(led, mobilelight_init_reg_data[i].addr, REG_MASK_VALUE, mobilelight_init_reg_data[i].value);
        if (rc != SPMI_NO_ERROR)
            goto FAIL_SPMI_CONTROL;
    }

    LED_DUMP_REGISTER(led, mobilelight_debug_regs, ARRAY_SIZE(mobilelight_debug_regs));

    DEBUG_PRINT("%s(): end \n", __func__);

    return rc;

FAIL_SPMI_CONTROL:
    DEBUG_PRINT_ERR("%s(): Failed to SPMI control rc=[%d] \n", __func__, rc);
    return rc;
}

static int __devinit backlight_init(struct light_led_data_type *led)
{
    int rc=0, i;

    DEBUG_PRINT("%s(): start\n", __func__);

    for (i = 0; i < (sizeof(backlight_init_reg_data)/sizeof(write_reg_led_data_init_type)); i++) {
        rc = led_masked_reg_write(led, backlight_init_reg_data[i].addr, REG_MASK_VALUE, backlight_init_reg_data[i].value);
        if (rc != SPMI_NO_ERROR)
            goto FAIL_SPMI_CONTROL;
    }

    LED_DUMP_REGISTER(led, backlight_debug_regs, ARRAY_SIZE(backlight_debug_regs));

    DEBUG_PRINT("%s(): end \n", __func__);

    return rc;

FAIL_SPMI_CONTROL:
    DEBUG_PRINT_ERR("%s(): Failed to SPMI control rc=[%d] \n", __func__, rc);
    return rc;
}


int32_t light_led_disp_set(e_light_main_wled_disp disp_status)
{
    int32_t ret=0;
#ifndef DISABLE_DISP_DETECT
    e_light_main_wled_disp status;
    
    mutex_lock(&led_disp_lock);
    DEBUG_PRINT("%s(): mutex_lock \n", __func__);
    status = (e_light_main_wled_disp)atomic_read(&g_disp_status);
    DEBUG_PRINT("%s(): start status=[0x%x] disp_status=[0x%x] \n", __func__, (uint32_t)status, (uint32_t)disp_status);

    if((atomic_read(&g_display_detect)) != 0){
        mutex_unlock(&led_disp_lock);
        DEBUG_PRINT("%s(): mutex_unlock \n", __func__);
        DEBUG_PRINT("%s(): [end]Already set g_display_detect=[%d] \n", __func__, (int32_t)atomic_read(&g_display_detect));
        return ret;
    }

    switch(disp_status) {
        case LIGHT_MAIN_WLED_LCD_EN:
        case LIGHT_MAIN_WLED_LED_EN:
            status |= disp_status;
            if(LIGHT_MAIN_WLED_EN == status) {
                ret = backlight_switch(gbacklight, BACKLIGHT_REG_D846_ON);
                atomic_set(&g_display_detect,1);
                DEBUG_PRINT("%s(): Set display detect status=[0x%x] \n", __func__, (uint32_t)status);
            }
            break;
        case LIGHT_MAIN_WLED_LCD_DIS:
            atomic_set(&g_display_detect,-1);
            DEBUG_PRINT_ERR("%s(): No set display disp_status=[0x%x] \n", __func__, (uint32_t)disp_status);
        case LIGHT_MAIN_WLED_LED_DIS:
            status &= ~(disp_status>>4);
            DEBUG_PRINT("%s(): status=[0x%x] \n", __func__, (uint32_t)status);
            break;
        default:
            break;
    }
    DEBUG_PRINT("%s(): status=[0x%x] g_display_detect=[%d] \n", __func__, (uint32_t)status, (int32_t)atomic_read(&g_display_detect));
    atomic_set(&g_disp_status,(uint32_t)status);
    mutex_unlock(&led_disp_lock);
    DEBUG_PRINT("%s(): mutex_unlock \n", __func__);
    DEBUG_PRINT("%s(): end ret=[%d] \n", __func__, ret);
#endif  /* DISABLE_DISP_DETECT */
    return ret;
}
EXPORT_SYMBOL(light_led_disp_set);


static int32_t leds_open(struct inode* inode, struct file* filp)
{
    DEBUG_PRINT("%s(): start \n", __func__);
    DEBUG_PRINT("%s(): end \n", __func__);
    return 0;
}

static int32_t leds_release(struct inode* inode, struct file* filp)
{
    DEBUG_PRINT("%s(): start \n", __func__);
    DEBUG_PRINT("%s(): end \n", __func__);
    return 0;
}

static struct file_operations leds_fops = {
    .owner        = THIS_MODULE,
    .open        = leds_open,
    .release    = leds_release,
    .unlocked_ioctl = leds_ioctl,
};

static struct miscdevice leds_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name  = "leds-ledlight",
    .fops  = &leds_fops,
};

static int __devinit leds_probe(struct spmi_device *spmi)
{
    struct light_led_data_type *led, *led_array;
    struct resource *led_resource;
    struct device_node *node, *temp;
    int rc, num_leds = 0, parsed_leds;
    const char *led_label;

    if (spmi == NULL){
        DEBUG_PRINT_ERR("%s(): Received spmi is NULL \n", __func__);
        return -ENODEV;
    }

    DEBUG_PRINT("%s(): start spmi=[0x%08x] \n", __func__, (unsigned int)spmi);

    node = spmi->dev.of_node;
    if (node == NULL)
        return -ENODEV;

    temp = NULL;
    while ((temp = of_get_next_child(node, temp)))
        num_leds++;
    num_leds = 1;

    led_array = devm_kzalloc(&spmi->dev, (sizeof(struct light_led_data_type) * num_leds), GFP_KERNEL);
    if (!led_array) {
        DEBUG_PRINT_ERR("%s(): Unable to allocate memory Dev=[0x%08x] \n", __func__, (unsigned int)&spmi->dev);
        return -ENOMEM;
    }

#ifndef DISABLE_DISP_DETECT
    mutex_init(&led_disp_lock);
#endif  /* DISABLE_DISP_DETECT */

    for (parsed_leds=0; parsed_leds < num_leds; parsed_leds++) {

        led = &led_array[parsed_leds];
        led->num_leds = num_leds;
        led->spmi_dev = spmi;

        temp = of_get_next_child(node, temp);

        led_resource = spmi_get_resource(spmi, NULL, IORESOURCE_MEM, 0);
        if (!led_resource) {
            DEBUG_PRINT_ERR("%s(): Unable to get LED base address Dev=[0x%08x] \n", __func__, (unsigned int)&spmi->dev);
            rc = -ENXIO;
            goto FAILED_IN_PROBE;
        }

        rc = of_property_read_string(temp, "label", &led_label);
        if (rc < 0) {
            DEBUG_PRINT_ERR("%s(): Failure reading label, Dev=[0x%08x] rc=[%d] \n", __func__, (unsigned int)&led->spmi_dev->dev, rc);
            goto FAILED_IN_PROBE;
        }

        mutex_init(&led->lock);
        INIT_WORK(&led->work, led_work);

        if (strcmp(led_label, LABEL_RGB_LED) == 0) {
            DEBUG_PRINT("%s(): Now start RGB_LED probe \n", __func__);

            led->cdev.name   = RGB_LED_INFO;
            led->cdev.max_brightness = (unsigned int)RGB_LED_MAX_BRIGHT_VAL;

            rc = rgb_led_init(led);
            if (rc != SPMI_NO_ERROR) {
                DEBUG_PRINT_ERR("%s(): RGB_LED initialize failed Dev=[0x%08x] rc=[%d] \n", __func__, (unsigned int)&led->spmi_dev->dev, rc);
                goto FAILED_IN_PROBE;
            }

            grgb_led = led;
            misc_register(&leds_device);

        } else if (strcmp(led_label, LABEL_MOBILELIGHT) == 0) {
            DEBUG_PRINT("%s(): Now start MOBILELIGHT probe \n", __func__);

            led->cdev.name   = MOBILELIGHT_INFO;
            led->cdev.max_brightness = (unsigned int)MOBILELIGHT_MAX_BRIGHT_VAL;

            rc = mobilelight_init(led);
            if (rc != SPMI_NO_ERROR) {
                DEBUG_PRINT_ERR("%s(): MOBILELIGHT initialize failed Dev=[0x%08x] rc=[%d] \n", __func__, (unsigned int)&led->spmi_dev->dev, rc);
                goto FAILED_IN_PROBE;
            }

            gmobilelight = led;

            boost_boost_enable = regulator_get(&spmi->dev, "parent");
            if (IS_ERR(boost_boost_enable)) {
                rc = -1;
                DEBUG_PRINT_ERR("%s(): Cannot get regulator Dev=[0x%08x] \n", __func__, (unsigned int)&spmi->dev);
                goto FAILED_IN_PROBE;
            } else {
                DEBUG_PRINT("%s(): Success in getting regulator Dev=[0x%08x] \n", __func__, (unsigned int)&spmi->dev);
            }

            hrtimer_init(&mobilelight_wdt, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
            mobilelight_wdt.function = mobilelight_wdt_reset;
            
        } else if (strcmp(led_label, LABEL_BACKLIGHT) == 0) {
            DEBUG_PRINT("%s(): Now start BACKLIGHT probe \n", __func__);

            led->cdev.name   = BACKLIGHT_INFO;
            led->cdev.max_brightness = (unsigned int)BACKLIGHT_MAX_BRIGHT_VAL;

            rc = backlight_init(led);
            if (rc != SPMI_NO_ERROR) {
                DEBUG_PRINT_ERR("%s(): BACKLIGHT initialize failed Dev=[0x%08x] rc=[%d] \n", __func__, (unsigned int)&led->spmi_dev->dev, rc);
                goto FAILED_IN_PROBE;
            }

            gbacklight = led;

        } else {
            DEBUG_PRINT_ERR("%s(): No LED matched label=[%s] \n", __func__, led_label);
            rc = -EINVAL;
            goto FAILED_IN_PROBE;
        }

        led->cdev.brightness_set    = led_set;
        led->cdev.brightness_get    = led_get;
        led->cdev.brightness        = LED_COL_BLACK;
        led->cdev.flags             = 0;

        rc = led_classdev_register(&led->spmi_dev->dev, &led->cdev);
        if (rc) {
            DEBUG_PRINT_ERR("%s(): Unable to register led name=[%s] Dev=[0x%08x] rc=[%d] \n", __func__, led_label, (unsigned int)&spmi->dev, rc);
            goto FAILED_IN_PROBE;
        }

        led->blink_control         = NO_BLINK_REQUEST; 
        led->blink_ramp_duration   = 0;
        led->blink_low_pause_time  = 0;
        led->blink_high_pause_time = 0;
        led->blink_off_color       = LED_COL_BLACK;
    }
    dev_set_drvdata(&spmi->dev, led_array);

    DEBUG_PRINT("%s(): end \n",__func__);

    return 0;

FAILED_IN_PROBE:
    DEBUG_PRINT_ERR("%s(): Failed in device probe rc=[%d] \n", __func__, rc);
    return rc;
}

static int __devexit qpnp_leds_remove(struct spmi_device *spmi)
{
    DEBUG_PRINT("%s(): start \n", __func__);
    DEBUG_PRINT("%s(): end \n", __func__);
    
    return 0;
}
static struct of_device_id spmi_match_table[] = {
    {    .compatible = "qcom,leds-qpnp",
    }
};

static struct spmi_driver qpnp_leds_driver = {
    .driver        = {
        .name    = "qcom,leds-qpnp",
        .of_match_table = spmi_match_table,
    },
    .probe        = leds_probe,
    .remove        = __devexit_p(qpnp_leds_remove),
};

static int __init qpnp_led_init(void)
{
    return spmi_driver_register(&qpnp_leds_driver);
}
module_init(qpnp_led_init);

static void __exit qpnp_led_exit(void)
{
    spmi_driver_unregister(&qpnp_leds_driver);
}
module_exit(qpnp_led_exit);

MODULE_DESCRIPTION("QPNP LEDs driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("leds:leds-qpnp");

