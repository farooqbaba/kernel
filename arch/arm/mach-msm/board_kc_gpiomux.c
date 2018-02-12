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
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*
*/
/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2013 KYOCERA Corporation
 */

#include <linux/init.h>
#include <linux/gpio.h>
#include <mach/gpiomux.h>
#include <mach/board_kc_gpiomux.h>

/* suspended */
static struct gpiomux_setting sus_gpio_np_2ma_in_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};
static struct gpiomux_setting sus_gpio_np_2ma_out_high_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_HIGH,
};
static struct gpiomux_setting sus_gpio_np_2ma_out_low_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_LOW,
};
static struct gpiomux_setting sus_gpio_pd_2ma_in_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_IN,
};
static struct gpiomux_setting sus_gpio_pu_2ma_in_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_IN,
};
static struct gpiomux_setting sus_func1_np_2ma_out_low_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_LOW,
};
static struct gpiomux_setting sus_func1_np_2ma_in_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};
static struct gpiomux_setting sus_func3_np_2ma_in_cfg = {
	.func = GPIOMUX_FUNC_3,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};
static struct gpiomux_setting sus_func1_pd_2ma_in_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_IN,
};
static struct gpiomux_setting sus_func1_np_8ma_out_low_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_LOW,
};
static struct gpiomux_setting sus_func3_pu_2ma_in_cfg = {
	.func = GPIOMUX_FUNC_3,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_IN,
};
static struct gpiomux_setting sus_func3_np_2ma_out_low_cfg = {
	.func = GPIOMUX_FUNC_3,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_LOW,
};
static struct gpiomux_setting sus_func4_np_4ma_out_high_cfg = {
	.func = GPIOMUX_FUNC_4,
	.drv = GPIOMUX_DRV_4MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_HIGH,
};

/* active */
static struct gpiomux_setting act_gpio_np_2ma_in_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};
static struct gpiomux_setting act_gpio_np_2ma_out_high_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_HIGH,
};
static struct gpiomux_setting act_gpio_np_2ma_out_low_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_LOW,
};
static struct gpiomux_setting act_gpio_pd_2ma_in_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_IN,
};
static struct gpiomux_setting act_gpio_pu_2ma_in_cfg = {
	.func = GPIOMUX_FUNC_GPIO,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_IN,
};
static struct gpiomux_setting act_func3_np_2ma_in_cfg = {
	.func = GPIOMUX_FUNC_3,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};
static struct gpiomux_setting act_func1_pd_2ma_in_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_IN,
};
static struct gpiomux_setting act_func1_np_8ma_out_low_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_8MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_LOW,
};
static struct gpiomux_setting act_func1_np_2ma_in_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_IN,
};
static struct gpiomux_setting act_func3_pu_2ma_in_cfg = {
	.func = GPIOMUX_FUNC_3,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_UP,
	.dir = GPIOMUX_IN,
};
static struct gpiomux_setting act_func1_np_2ma_out_low_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_LOW,
};
static struct gpiomux_setting act_func3_np_2ma_out_low_cfg = {
	.func = GPIOMUX_FUNC_3,
	.drv = GPIOMUX_DRV_2MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_LOW,
};
static struct gpiomux_setting act_func1_pd_6ma_in_cfg = {
	.func = GPIOMUX_FUNC_1,
	.drv = GPIOMUX_DRV_6MA,
	.pull = GPIOMUX_PULL_DOWN,
	.dir = GPIOMUX_IN,
};
static struct gpiomux_setting act_func4_np_4ma_out_high_cfg = {
	.func = GPIOMUX_FUNC_4,
	.drv = GPIOMUX_DRV_4MA,
	.pull = GPIOMUX_PULL_NONE,
	.dir = GPIOMUX_OUT_HIGH,
};

static struct msm_gpiomux_config msm8974_gpio_configs[] __initdata = {
    {
        .gpio = 0,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 1,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 2,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 3,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 4,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_np_2ma_out_low_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_np_2ma_out_low_cfg,
        },
    },
    {
        .gpio = 5,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 6,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_func3_np_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_func3_np_2ma_in_cfg,
        },
    },
    {
        .gpio = 7,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_func3_np_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_func3_np_2ma_in_cfg,
        },
    },
    {
        .gpio = 8,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 9,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_np_2ma_out_low_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_np_2ma_out_low_cfg,
        },
    },
    {
        .gpio = 10,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    /* gpio = 11 No Setting*/
    {
        .gpio = 12,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 13,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 14,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_np_2ma_out_low_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_np_2ma_out_low_cfg,
        },
    },
    {
        .gpio = 15,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_func1_np_8ma_out_low_cfg,
            [GPIOMUX_SUSPENDED] = &sus_func1_np_8ma_out_low_cfg,
        },
    },
    {
        .gpio = 16,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_np_2ma_out_low_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_np_2ma_out_low_cfg,
        },
    },
    {
        .gpio = 17,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_func1_np_8ma_out_low_cfg,
            [GPIOMUX_SUSPENDED] = &sus_func1_np_8ma_out_low_cfg,
        },
    },
    {
        .gpio = 18,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_np_2ma_out_low_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_np_2ma_out_low_cfg,
        },
    },
    {
        .gpio = 19,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_func1_np_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_func1_np_2ma_in_cfg,
        },
    },
    {
        .gpio = 20,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_func1_np_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_func1_np_2ma_in_cfg,
        },
    },
    {
        .gpio = 21,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_func1_np_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_func1_np_2ma_in_cfg,
        },
    },
    {
        .gpio = 22,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_func1_np_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_func1_np_2ma_in_cfg,
        },
    },
    {
        .gpio = 23,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_np_2ma_out_low_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_np_2ma_out_low_cfg,
        },
    },
    {
        .gpio = 24,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pu_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pu_2ma_in_cfg,
        },
    },
    {
        .gpio = 25,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_np_2ma_out_low_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_np_2ma_out_low_cfg,
        },
    },
    {
        .gpio = 26,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 27,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_func4_np_4ma_out_high_cfg,
            [GPIOMUX_SUSPENDED] = &sus_func4_np_4ma_out_high_cfg,
        },
    },
    {
        .gpio = 28,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_func3_pu_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_func3_pu_2ma_in_cfg,
        },
    },
    {
        .gpio = 29,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_func3_np_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_func3_np_2ma_in_cfg,
        },
    },
    {
        .gpio = 30,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_func3_np_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_func3_np_2ma_in_cfg,
        },
    },
    {
        .gpio = 31,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 32,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_func1_np_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_np_2ma_in_cfg,
        },
    },
    {
        .gpio = 33,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_func1_np_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_np_2ma_in_cfg,
        },
    },
    {
        .gpio = 34,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_func1_np_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_np_2ma_in_cfg,
        },
    },
    {
        .gpio = 35,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_func1_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 36,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_func1_pd_6ma_in_cfg,
        },
    },
    {
        .gpio = 37,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_func1_pd_6ma_in_cfg,
        },
    },
    {
        .gpio = 38,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_func1_pd_6ma_in_cfg,
        },
    },
    {
        .gpio = 39,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_func1_pd_6ma_in_cfg,
        },
    },
    {
        .gpio = 40,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_func1_pd_6ma_in_cfg,
        },
    },
    {
        .gpio = 41,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 42,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    /* gpio = 43 No Setting*/
    /* gpio = 44 No Setting*/
    {
        .gpio = 45,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_np_2ma_out_low_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_np_2ma_out_low_cfg,
        },
    },
    {
        .gpio = 46,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 47,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 48,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 49,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_func3_np_2ma_out_low_cfg,
            [GPIOMUX_SUSPENDED] = &sus_func3_np_2ma_out_low_cfg,
        },
    },
    {
        .gpio = 50,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_func3_pu_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_func3_pu_2ma_in_cfg,
        },
    },
    {
        .gpio = 51,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_np_2ma_out_high_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_np_2ma_out_high_cfg,
        },
    },
    {
        .gpio = 52,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_np_2ma_out_high_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_np_2ma_out_high_cfg,
        },
    },
    {
        .gpio = 53,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_np_2ma_out_low_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_np_2ma_out_low_cfg,
        },
    },
    {
        .gpio = 54,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pu_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pu_2ma_in_cfg,
        },
    },
    {
        .gpio = 55,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 56,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 57,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_np_2ma_out_low_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_np_2ma_out_low_cfg,
        },
    },
    /* gpio = 58 No Setting*/
    {
        .gpio = 59,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pu_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pu_2ma_in_cfg,
        },
    },
    {
        .gpio = 60,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_np_2ma_out_low_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_np_2ma_out_low_cfg,
        },
    },
    {
        .gpio = 61,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pu_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pu_2ma_in_cfg,
        },
    },
    {
        .gpio = 62,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_np_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_np_2ma_in_cfg,
        },
    },
    {
        .gpio = 63,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_np_2ma_out_low_cfg,
        },
    },
    {
        .gpio = 64,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pu_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pu_2ma_in_cfg,
        },
    },
    {
        .gpio = 65,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 66,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 67,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 68,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 69,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    /* gpio = 70 No Setting*/
    /* gpio = 71 No Setting*/
    /* gpio = 72 No Setting*/
    {
        .gpio = 73,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 74,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pu_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pu_2ma_in_cfg,
        },
    },
    {
        .gpio = 75,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 76,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_np_2ma_out_low_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_np_2ma_out_low_cfg,
        },
    },
    {
        .gpio = 77,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 78,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 79,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pu_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pu_2ma_in_cfg,
        },
    },
    {
        .gpio = 80,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pu_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pu_2ma_in_cfg,
        },
    },
    {
        .gpio = 81,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_np_2ma_out_low_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_np_2ma_out_low_cfg,
        },
    },
    {
        .gpio = 82,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 83,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_np_2ma_out_low_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_np_2ma_out_low_cfg,
        },
    },
    {
        .gpio = 84,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_np_2ma_out_low_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_np_2ma_out_low_cfg,
        },
    },
    {
        .gpio = 85,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_np_2ma_out_low_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_np_2ma_out_low_cfg,
        },
    },
    {
        .gpio = 86,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 87,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_func3_np_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_func3_np_2ma_in_cfg,
        },
    },
    {
        .gpio = 88,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_func3_np_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_func3_np_2ma_in_cfg,
        },
    },
    {
        .gpio = 89,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 90,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 91,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 92,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    /* gpio = 93 No Setting*/
    {
        .gpio = 94,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 95,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 96,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 97,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_func1_np_2ma_out_low_cfg,
            [GPIOMUX_SUSPENDED] = &sus_func1_np_2ma_out_low_cfg,
        },
    },
    {
        .gpio = 98,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_func1_np_2ma_out_low_cfg,
            [GPIOMUX_SUSPENDED] = &sus_func1_np_2ma_out_low_cfg,
        },
    },
    {
        .gpio = 99,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_np_2ma_out_low_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_np_2ma_out_low_cfg,
        },
    },
    {
        .gpio = 100,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 101,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_np_2ma_out_low_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_np_2ma_out_low_cfg,
        },
    },
    {
        .gpio = 102,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_np_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_np_2ma_in_cfg,
        },
    },
    {
        .gpio = 103,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_func1_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_func1_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 104,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_func1_np_2ma_out_low_cfg,
        },
    },
    {
        .gpio = 105,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 106,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 107,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 108,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_func1_np_2ma_out_low_cfg,
        },
    },
    {
        .gpio = 109,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 110,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_func1_np_2ma_out_low_cfg,
        },
    },
    {
        .gpio = 111,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_func1_np_2ma_out_low_cfg,
        },
    },
    {
        .gpio = 112,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 113,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_func1_np_2ma_out_low_cfg,
        },
    },
    {
        .gpio = 114,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 115,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 116,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_np_2ma_out_low_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_np_2ma_out_low_cfg,
        },
    },
    {
        .gpio = 117,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 118,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_func1_np_2ma_out_low_cfg,
        },
    },
    {
        .gpio = 119,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 120,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 121,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 122,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 123,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_func1_np_2ma_out_low_cfg,
        },
    },
    {
        .gpio = 124,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 125,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 126,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 127,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_func1_np_2ma_out_low_cfg,
        },
    },
    {
        .gpio = 128,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_np_2ma_out_low_cfg,
        },
    },
    {
        .gpio = 129,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 130,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 131,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 132,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    /* gpio = 133 No Setting*/
    /* gpio = 134 No Setting*/
    {
        .gpio = 135,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 136,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 137,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    /* gpio = 138 No Setting*/
    /* gpio = 139 No Setting*/
    /* gpio = 140 No Setting*/
    /* gpio = 141 No Setting*/
    /* gpio = 142 No Setting*/
    /* gpio = 143 No Setting*/
    {
        .gpio = 144,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
    {
        .gpio = 145,
        .settings = {
            [GPIOMUX_ACTIVE]    = &act_gpio_pd_2ma_in_cfg,
            [GPIOMUX_SUSPENDED] = &sus_gpio_pd_2ma_in_cfg,
        },
    },
};

void __init msm8960_kc_init_gpiomux(void)
{
    msm_gpiomux_install(msm8974_gpio_configs, ARRAY_SIZE(msm8974_gpio_configs));

    return;
}
