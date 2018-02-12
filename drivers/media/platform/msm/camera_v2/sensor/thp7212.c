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
#include "msm_sensor.h"
#include "msm_cci.h"
#include "msm_camera_io_util.h"
#include "msm.h"
#define THP7212_SENSOR_NAME "thp7212"
#define PLATFORM_DRIVER_NAME "msm_camera_thp7212"
#define thp7212_obj thp7212_##obj

//#define CONFIG_MSMB_CAMERA_DEBUG
#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

DEFINE_MSM_MUTEX(thp7212_mut);
static struct msm_sensor_ctrl_t thp7212_s_ctrl;
static int32_t g_res=0;
static int32_t g_prev_w, g_prev_h;
static int32_t g_thp7212_bracket_mode = 0;
static int32_t g_thp7217_module_bracket = 0;
static int32_t g_thp7212_af_roi = 0;
static int32_t g_thp7212_af_mode = 0;
static int32_t g_snapshot_flg = 0;
static int32_t g_thp7217_multi_frame_save = 0;
static int32_t g_thp7217_multi_frame_save_parm = 0;

static struct msm_sensor_power_setting thp7212_power_setting[] = {
    {
        .seq_type = SENSOR_VREG,
        .seq_val = CAM_VANA,
        .config_val = 0,
        .delay = 0,
    },
    {
        .seq_type = SENSOR_VREG,
        .seq_val = CAM_VIO,
        .config_val = 0,
        .delay = 0,
    },
    {
        .seq_type = SENSOR_GPIO,
        .seq_val = SENSOR_GPIO_VISP,
        .config_val = GPIO_OUT_HIGH,
        .delay = 0,
    },
    {
        .seq_type = SENSOR_VREG,
        .seq_val = CAM_VDIG,
        .config_val = 0,
        .delay = 0,
    },
    {
        .seq_type = SENSOR_VREG,
        .seq_val = CAM_VAF,
        .config_val = 0,
        .delay = 10,
    },
    {
        .seq_type = SENSOR_CLK,
        .seq_val = SENSOR_CAM_MCLK,
        .config_val = 0,
        .delay = 1,
    },
    {
        .seq_type = SENSOR_GPIO,
        .seq_val = SENSOR_GPIO_RESET,
        .config_val = GPIO_OUT_HIGH,
        .delay = 1,
    },
    {
        .seq_type = SENSOR_I2C_MUX,
        .seq_val = 0,
        .config_val = 0,
        .delay = 0,
    },
};

struct thp7212_mapping {
	int32_t desc;
	int32_t val;
};

static struct thp7212_mapping thp7212_map_effect[] = {
    {CAMERA_EFFECT_MODE_OFF     , 0x0},
    {CAMERA_EFFECT_MODE_MONO    , 0x1},
    {CAMERA_EFFECT_MODE_SEPIA   , 0x2},
    {CAMERA_EFFECT_MODE_NEGATIVE, 0x3},
};

static struct thp7212_mapping thp7212_map_wb[] = {
    {CAMERA_WB_MODE_AUTO            , 0x0},
    {CAMERA_WB_MODE_INCANDESCENT    , 0x1},
    {CAMERA_WB_MODE_DAYLIGHT        , 0x2},
    {CAMERA_WB_MODE_CLOUDY_DAYLIGHT , 0x3},
    {CAMERA_WB_MODE_WARM_FLUORESCENT, 0x4},
    {CAMERA_WB_MODE_FLUORESCENT     , 0x5},
};

static struct thp7212_mapping thp7212_map_bestshot[] = {
    {CAMERA_SCENE_MODE_OFF        , 0x0},
    {CAMERA_SCENE_MODE_PORTRAIT   , 0x1},
    {CAMERA_SCENE_MODE_PARTY_1    , 0x2},
    {CAMERA_SCENE_MODE_PARTY_2    , 0x3},
    {CAMERA_SCENE_MODE_PARTY_3    , 0x4},
    {CAMERA_SCENE_MODE_SPORTS     , 0x5},
    {CAMERA_SCENE_MODE_LANDSCAPE  , 0x6},
    {CAMERA_SCENE_MODE_NIGHT      , 0x7},
    {CAMERA_SCENE_MODE_BACKLIGHT  , 0x8},
    {CAMERA_SCENE_MODE_MAPTEXT    , 0x9},
    {CAMERA_SCENE_MODE_BABY       , 0xA},
    {CAMERA_SCENE_MODE_SUNSET     , 0xB},
    {CAMERA_SCENE_MODE_PARTY      , 0xC},
    {CAMERA_SCENE_MODE_AUTO       , 0xFF},
};

/* Focus Control */
static struct msm_camera_i2c_reg_conf thp7212_focus_single[] = {
    {0xF041, 0x00},
    {0xF040, 0x01},
};

static struct msm_camera_i2c_reg_conf thp7212_focus_cancel[] = {
    {0xF040, 0x80},
};

static struct msm_camera_i2c_reg_conf thp7212_focus_infinity[] = {
    {0xF040, 0x03},
};

static struct msm_camera_i2c_reg_conf thp7212_focus_macro[] = {
    {0xF040, 0x04},
};

static struct msm_camera_i2c_reg_conf thp7212_focus_caf_video[] = {
    {0xF041, 0x12},
    {0xF040, 0x01},
};

static struct msm_camera_i2c_reg_conf thp7212_focus_caf_pict[] = {
    {0xF041, 0x32},
    {0xF040, 0x01},
};

static struct msm_camera_i2c_reg_conf thp7212_af_roi_parm[] = {
    {0xF046, 0x00},
    {0xF047, 0x00},
    {0xF048, 0x00},
    {0xF049, 0x00},
    {0xF04A, 0x00},
    {0xF04B, 0x00},
    {0xF04C, 0x00},
    {0xF04D, 0x00},
    {0xF041, 0x02},
    {0xF040, 0x02},
};

static struct msm_camera_i2c_reg_conf thp7212_af_roi_parm_vcaf[] = {
    {0xF046, 0x00},
    {0xF047, 0x00},
    {0xF048, 0x00},
    {0xF049, 0x00},
    {0xF04A, 0x00},
    {0xF04B, 0x00},
    {0xF04C, 0x00},
    {0xF04D, 0x00},
    {0xF041, 0x13},
    {0xF040, 0x01},
};

/* Output Control */
static struct msm_camera_i2c_reg_conf thp7212_output_ctrl_disable[] = {
    {0xF008, 0x00},
};
static struct msm_camera_i2c_reg_conf thp7212_output_ctrl_enable[] = {
    {0xF008, 0x01},
};

struct thp7212_conf_array_setting_size {
    void *conf;
    int32_t size;
    uint8_t e_data;
};

static struct msm_camera_i2c_reg_conf thp7212_full_size_13m[] = {
    {0xF00D, 0x14},
    {0xF00E, 0x14},
    {0xF008, 0x00},
    {0xF010, 0x01},
};
static struct msm_camera_i2c_reg_conf thp7212_full_size_9m[] = {
    {0xF00D, 0x13},
    {0xF00E, 0x13},
    {0xF008, 0x00},
    {0xF010, 0x01},
};
static struct msm_camera_i2c_reg_conf thp7212_full_size_8m[] = {
    {0xF00D, 0x12},
    {0xF00E, 0x12},
    {0xF008, 0x00},
    {0xF010, 0x01},
};
static struct msm_camera_i2c_reg_conf thp7212_full_size_3m[] = {
    {0xF00D, 0x10},
    {0xF00E, 0x10},
    {0xF008, 0x00},
    {0xF010, 0x01},
};
static struct msm_camera_i2c_reg_conf thp7212_full_size_fhd[] = {
    {0xF00D, 0x0C},
    {0xF00E, 0x0C},
    {0xF008, 0x00},
    {0xF010, 0x01},
};
static struct msm_camera_i2c_reg_conf thp7212_full_size_fhd_plus[] = {
    {0xF00D, 0x0B},
    {0xF00E, 0x0B},
    {0xF008, 0x00},
    {0xF010, 0x01},
};
static struct msm_camera_i2c_reg_conf thp7212_full_size_wvga[] = {
    {0xF008, 0x00},
    {0xF010, 0x07},
};
static struct msm_camera_i2c_reg_conf thp7212_full_size_3m_hdr[] = {
    {0xF00D, 0x10},
    {0xF008, 0x00},
    {0xF010, 0x02},
};
static struct msm_camera_i2c_reg_conf thp7212_full_size_fhd_plus_hdr[] = {
    {0xF00D, 0x0B},
    {0xF008, 0x00},
    {0xF010, 0x02},
};

static struct thp7212_conf_array_setting_size thp7212_confs[] = {
    {&thp7212_full_size_13m,          4,  0x01},
    {&thp7212_full_size_9m,           4,  0x01},
    {&thp7212_full_size_8m,           4,  0x01},
    {&thp7212_full_size_3m,           4,  0x01},
    {&thp7212_full_size_fhd,          4,  0x01},
    {&thp7212_full_size_fhd_plus,     4,  0x01},
    {&thp7212_full_size_wvga,         2,  0x07},
    {&thp7212_full_size_3m_hdr,       3,  0x02},
    {&thp7212_full_size_fhd_plus_hdr, 3,  0x02},
};

struct thp7212_sensor_size {
	int32_t width;
	int32_t height;
};
static struct thp7212_sensor_size thp7212_sensor_output_size[] = {
    { 4160, 3120 },
    { 3920, 2204 },
    { 3264, 2448 },
    { 2048, 1536 },
    { 1920, 1080 },
    { 2048, 1152 },
    {  800,  450 },
    { 2048, 1536 },
    { 2048, 1152 },
};
static struct msm_camera_i2c_reg_conf thp7212_ae_bracket_parm[] = {
    {0xF012, 0x01},
    {0xF014, 0x00},
    {0xF015, 0x00},
    {0xF016, 0x00},
    {0xF017, 0x00},
};

static struct v4l2_subdev_info thp7212_subdev_info[] = {
    {
        .code   = V4L2_MBUS_FMT_YUYV8_2X8,
        .colorspace = V4L2_COLORSPACE_JPEG,
        .fmt    = 1,
        .order    = 0,
    },
};

static const struct i2c_device_id thp7212_i2c_id[] = {
    {THP7212_SENSOR_NAME, (kernel_ulong_t)&thp7212_s_ctrl},
    { }
};

static struct msm_camera_i2c_client thp7212_sensor_i2c_client = {
    .addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id thp7212_dt_match[] = {
    {.compatible = "qcom,thp7212", .data = &thp7212_s_ctrl},
    {}
};

MODULE_DEVICE_TABLE(of, thp7212_dt_match);

static struct platform_driver thp7212_platform_driver = {
    .driver = {
        .name = "qcom,thp7212",
        .owner = THIS_MODULE,
        .of_match_table = thp7212_dt_match,
    },
};

extern int thp7212_spi_transfer(void);
extern int thp7212_fw_update(void*);

static int32_t thp7212_fwdl(struct msm_sensor_ctrl_t *s_ctrl)
{
    int32_t rc;
    int32_t i;
    uint16_t data=0;

    rc = thp7212_spi_transfer();
    if (rc) {
        pr_err("%s:%d failed rc:%d\n", __func__, __LINE__, rc);
        return rc;
    }

    usleep_range(70000, 70000);
    for (i=0; i<23; i++){//timeout total 300ms
        rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
                s_ctrl->sensor_i2c_client,
                0xF001, &data, MSM_CAMERA_I2C_BYTE_DATA);
        if(data == 0x80)
            break;
        usleep_range(10000, 10000);
    }
    if(i>=23)
        return -EIO;

    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
            s_ctrl->sensor_i2c_client,
            0xF000, &data, MSM_CAMERA_I2C_BYTE_DATA);
    pr_info("%s: fw ver %02x", __func__, data);
    if (rc) {
        pr_err("%s:%d: i2c_write failed\n", __func__, __LINE__);
        msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
        return rc;
    }
    //fd enable
    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
        s_ctrl->sensor_i2c_client,
        0xF064, 0x0001, MSM_CAMERA_I2C_BYTE_DATA);
    if (rc) {
        pr_err("%s:%d: i2c_write failed\n", __func__, __LINE__);
        msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
        return rc;
    }

    return 0;
}

static int32_t thp7212_set_resolution(struct msm_sensor_ctrl_t *s_ctrl, int32_t res, uint32_t stream_mask, int32_t prev_w, int32_t prev_h)
{
    int32_t rc=0;
    int32_t i=0;
    uint16_t data;
    uint16_t e_data;

    pr_info("CFG_SET_RESOLUTION res:%d, mask:0x%02x Bra:%d\n", res, stream_mask, g_thp7212_bracket_mode);
    g_prev_w = prev_w;
    g_prev_h = prev_h;
    if(g_res == res)
    {
      if(g_thp7217_module_bracket == g_thp7212_bracket_mode)
      {
        if(g_thp7212_bracket_mode == CAMERA_BRACKET_MODE_ON) {
          if((stream_mask == 0x0C) && (g_thp7217_module_bracket == 1)) {
            return 0;
          }
        } else
          return 0;
      }
    }

    pr_info("CFG_SET_RESOLUTION size:%d\n", thp7212_confs[res].size);

    if((g_thp7212_bracket_mode == CAMERA_BRACKET_MODE_ON) && (stream_mask == 0x0C) && (g_thp7217_module_bracket == 0)) {
        thp7212_full_size_13m[3].reg_data = 0x05;
        thp7212_full_size_9m[3].reg_data = 0x05;
        thp7212_full_size_8m[3].reg_data = 0x05;
        thp7212_full_size_3m[3].reg_data = 0x05;
        thp7212_full_size_fhd[3].reg_data = 0x05;
        thp7212_full_size_fhd_plus[3].reg_data = 0x05;
        g_thp7217_module_bracket = 1;
    } else {
        thp7212_full_size_13m[3].reg_data = 0x01;
        thp7212_full_size_9m[3].reg_data = 0x01;
        thp7212_full_size_8m[3].reg_data = 0x01;
        thp7212_full_size_3m[3].reg_data = 0x01;
        thp7212_full_size_fhd[3].reg_data = 0x01;
        thp7212_full_size_fhd_plus[3].reg_data = 0x01;
        g_thp7217_module_bracket = 0;
    }

    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_conf_tbl(
        s_ctrl->sensor_i2c_client,
        (struct msm_camera_i2c_reg_conf *)thp7212_confs[res].conf,
        thp7212_confs[res].size,
        MSM_CAMERA_I2C_BYTE_DATA);

    e_data = thp7212_confs[res].e_data;

    if((g_thp7217_module_bracket == 1) &&
        (res <= MSM_SENSOR_RES_5)) {
        e_data = 0x05;
    }

#if 0
    for (i=0; i<100; i++) {
        rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
                s_ctrl->sensor_i2c_client,
                0xF002, &data, MSM_CAMERA_I2C_BYTE_DATA);
        if(data == (e_data | 0x80))
            break;
        usleep_range(10000, 10000);
    }
    if(i==100 || rc<0) {
        pr_err("%s:%d failed\n", __func__, __LINE__);
        msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
        return -EIO;
    }
#endif
    if(stream_mask != 0x0C) {
      for (i=0; i<100; i++) {
          rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
                  s_ctrl->sensor_i2c_client,
                  0xF011, &data, MSM_CAMERA_I2C_BYTE_DATA);
         if(data == e_data)
              break;
          usleep_range(10000, 10000);
      }
      if(i==100 || rc<0) {
          pr_err("%s:%d failed\n", __func__, __LINE__);
          msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
          return -EIO;
      }
    }else
      g_snapshot_flg = 1;
    if(g_thp7217_multi_frame_save)
    {
        pr_err("multi_frame set val:%d",g_thp7217_multi_frame_save_parm);
        rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
            s_ctrl->sensor_i2c_client,
            0xF01A, (uint16_t)g_thp7217_multi_frame_save_parm, MSM_CAMERA_I2C_BYTE_DATA);
        if (rc) {
            pr_err("%s:%d failed\n", __func__, __LINE__);
            msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
        }
        g_thp7217_multi_frame_save = 0;
    }

    g_res = res;
    return 0;
}

static int32_t thp7212_start_stream(struct msm_sensor_ctrl_t *s_ctrl)
{
    int32_t rc=0;
    int32_t i;
    uint16_t data=0;
    uint16_t e_data;

    if(g_snapshot_flg == 1){
      e_data = thp7212_confs[g_res].e_data;
      if((g_thp7217_module_bracket == 1) &&
          (g_res <= MSM_SENSOR_RES_5)) {
          e_data = 0x05;
      }

      for (i=0; i<100; i++) {
          rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
                  s_ctrl->sensor_i2c_client,
                  0xF011, &data, MSM_CAMERA_I2C_BYTE_DATA);
         if(data == e_data)
              break;
          usleep_range(10000, 10000);
      }
      g_snapshot_flg = 0;
      if(i==100 || rc<0) {
          pr_err("%s:%d failed\n", __func__, __LINE__);
          msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
          return -EIO;
      }
    }

    //Clear Mask
    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_conf_tbl(
        s_ctrl->sensor_i2c_client,
        thp7212_output_ctrl_enable,
        ARRAY_SIZE(thp7212_output_ctrl_enable),
        MSM_CAMERA_I2C_BYTE_DATA);
    if (rc) {
        pr_err("%s:%d failed\n", __func__, __LINE__);
        msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
    }
    return rc;
}

static int32_t thp7212_stop_stream(struct msm_sensor_ctrl_t *s_ctrl)
{
    int32_t rc=0;

    //Set Mask
    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_conf_tbl(
        s_ctrl->sensor_i2c_client,
        thp7212_output_ctrl_disable,
        ARRAY_SIZE(thp7212_output_ctrl_disable),
        MSM_CAMERA_I2C_BYTE_DATA);

    if (rc) {
        pr_err("%s:%d failed\n", __func__, __LINE__);
        msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
    }
    return rc;
}

static int32_t thp7212_set_white_balance(struct msm_sensor_ctrl_t *s_ctrl, int32_t wb)
{
    int32_t rc=0, i=0;
    pr_info("%s val:%d", __func__, wb);

    //current state check

    for(i=0 ; i<(sizeof(thp7212_map_wb)/sizeof(struct thp7212_mapping)) ; i++){
        if(wb == thp7212_map_wb[i].desc){
            rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
                s_ctrl->sensor_i2c_client,
                0xF039, thp7212_map_wb[i].val, MSM_CAMERA_I2C_BYTE_DATA);
            break;
        }
    }
    if (rc) {
        pr_err("%s:%d failed\n", __func__, __LINE__);
        msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
    }
    return rc;
}

static int32_t thp7212_set_effect(struct msm_sensor_ctrl_t *s_ctrl, int32_t effect)
{
    int32_t rc=0, i=0;
    pr_info("%s val:%d", __func__, effect);

    //current state check

    for(i=0 ; i<(sizeof(thp7212_map_effect)/sizeof(struct thp7212_mapping)) ; i++){
        if(effect == thp7212_map_effect[i].desc){
            rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
                s_ctrl->sensor_i2c_client,
                0xF051, thp7212_map_effect[i].val, MSM_CAMERA_I2C_BYTE_DATA);
            break;
        }
    }
    if (rc) {
        pr_err("%s:%d failed\n", __func__, __LINE__);
        msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
    }
    return rc;
}

static int32_t thp7212_set_exposure_compensation(struct msm_sensor_ctrl_t *s_ctrl, int32_t exp)
{
    int32_t rc=0;
    uint16_t val=0;
    pr_info("%s val:%d", __func__, exp);
    
    //current state check

    val = exp + 6;
    pr_info("%s setting val:%d", __func__, val);
    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
        s_ctrl->sensor_i2c_client,
        0xF022, val, MSM_CAMERA_I2C_BYTE_DATA);
    if (rc) {
        pr_err("%s:%d failed\n", __func__, __LINE__);
        msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
    }
    return rc;
}

static int32_t thp7212_set_bestshot(struct msm_sensor_ctrl_t *s_ctrl, int32_t bestshot)
{
    int32_t rc=0, i=0;
    pr_info("%s val:%d", __func__, bestshot);

    //current state check

    for(i=0 ; i<(sizeof(thp7212_map_bestshot)/sizeof(struct thp7212_mapping)) ; i++){
        if(bestshot == thp7212_map_bestshot[i].desc){
            rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
                s_ctrl->sensor_i2c_client,
                0xF018, thp7212_map_bestshot[i].val, MSM_CAMERA_I2C_BYTE_DATA);
            break;
        }
    }
    if (rc) {
        pr_err("%s:%d failed\n", __func__, __LINE__);
        msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
    }
    return rc;
}

static int32_t thp7212_set_focus_mode(struct msm_sensor_ctrl_t *s_ctrl, struct sensor_af_info_t *af_info)
{
    int32_t rc=0;
    pr_info("%s val:%d %d", __func__, af_info->mode, af_info->fd_enable);

    if( (af_info->mode != FOCUS_MODE_AUTO) && (af_info->mode == g_thp7212_af_mode))
        return 0;

    if( af_info->mode == FOCUS_MODE_OFF ){
        rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_conf_tbl(
            s_ctrl->sensor_i2c_client,
            thp7212_focus_cancel,
            ARRAY_SIZE(thp7212_focus_cancel),
            MSM_CAMERA_I2C_BYTE_DATA);
    }
    else if( af_info->mode == FOCUS_MODE_AUTO ){
        if(g_thp7212_af_roi == 1){
            rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_conf_tbl(
                s_ctrl->sensor_i2c_client,
                thp7212_af_roi_parm,
                ARRAY_SIZE(thp7212_af_roi_parm),
                MSM_CAMERA_I2C_BYTE_DATA);
                g_thp7212_af_roi = 0;
        }
        else{
            rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_conf_tbl(
                s_ctrl->sensor_i2c_client,
                thp7212_focus_single,
                ARRAY_SIZE(thp7212_focus_single),
                MSM_CAMERA_I2C_BYTE_DATA);
        }
    }
    else if( af_info->mode == FOCUS_MODE_INFINITY ){
        rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_conf_tbl(
            s_ctrl->sensor_i2c_client,
            thp7212_focus_infinity,
            ARRAY_SIZE(thp7212_focus_infinity),
            MSM_CAMERA_I2C_BYTE_DATA);
    }
    else if( af_info->mode == FOCUS_MODE_MACRO ){
        rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_conf_tbl(
            s_ctrl->sensor_i2c_client,
            thp7212_focus_macro,
            ARRAY_SIZE(thp7212_focus_macro),
            MSM_CAMERA_I2C_BYTE_DATA);
    }
    else if( af_info->mode == FOCUS_MODE_CONTINOUS_VIDEO ){
        rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_conf_tbl(
            s_ctrl->sensor_i2c_client,
            thp7212_focus_caf_video,
            ARRAY_SIZE(thp7212_focus_caf_video),
            MSM_CAMERA_I2C_BYTE_DATA);
    }
    else if( af_info->mode == FOCUS_MODE_CONTINOUS_PICTURE ){
        rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_conf_tbl(
            s_ctrl->sensor_i2c_client,
            thp7212_focus_caf_pict,
            ARRAY_SIZE(thp7212_focus_caf_pict),
            MSM_CAMERA_I2C_BYTE_DATA);
    }
    if (rc) {
        pr_err("%s:%d failed\n", __func__, __LINE__);
        msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
    }
    g_thp7212_af_mode = af_info->mode;
    return rc;
}

static int32_t thp7212_set_af_roi(struct msm_sensor_ctrl_t *s_ctrl, struct sensor_af_roi_info_t *af_roi)
{
    int32_t rc=0;
    int16_t left, top, right, bottom;

    pr_info("%s",__func__);
    pr_info("%08x %08x %08x %08x", af_roi->left, af_roi->top, af_roi->width, af_roi->height);

    left   = af_roi->left;
    top    = af_roi->top;
    right  = af_roi->left + af_roi->width;
    bottom = af_roi->top + af_roi->height;
    
    if(g_thp7212_af_mode == FOCUS_MODE_CONTINOUS_VIDEO) {
      thp7212_af_roi_parm_vcaf[0].reg_data = (left >> 8) & 0xff;
      thp7212_af_roi_parm_vcaf[1].reg_data = left & 0xff;
      thp7212_af_roi_parm_vcaf[2].reg_data = (top >> 8) & 0xff;
      thp7212_af_roi_parm_vcaf[3].reg_data = top & 0xff;
      thp7212_af_roi_parm_vcaf[4].reg_data = (right >> 8) & 0xff;
      thp7212_af_roi_parm_vcaf[5].reg_data = right & 0xff;
      thp7212_af_roi_parm_vcaf[6].reg_data = (bottom >> 8) & 0xff;
      thp7212_af_roi_parm_vcaf[7].reg_data = bottom & 0xff;

      rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_conf_tbl(
          s_ctrl->sensor_i2c_client,
          thp7212_af_roi_parm_vcaf,
          ARRAY_SIZE(thp7212_af_roi_parm_vcaf),
          MSM_CAMERA_I2C_BYTE_DATA);
      if (rc) {
          pr_err("%s:%d failed\n", __func__, __LINE__);
          msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
      }
    } else {
      thp7212_af_roi_parm[0].reg_data = (left >> 8) & 0xff;
      thp7212_af_roi_parm[1].reg_data = left & 0xff;
      thp7212_af_roi_parm[2].reg_data = (top >> 8) & 0xff;
      thp7212_af_roi_parm[3].reg_data = top & 0xff;
      thp7212_af_roi_parm[4].reg_data = (right >> 8) & 0xff;
      thp7212_af_roi_parm[5].reg_data = right & 0xff;
      thp7212_af_roi_parm[6].reg_data = (bottom >> 8) & 0xff;
      thp7212_af_roi_parm[7].reg_data = bottom & 0xff;

      g_thp7212_af_roi = 1;
    }
    return rc;
}

static int32_t thp7212_set_direction_of_phone(struct msm_sensor_ctrl_t *s_ctrl, int32_t direction)
{
    int32_t rc=0;
    pr_info("%s val:%d", __func__, direction);
    
    //current state check

    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
        s_ctrl->sensor_i2c_client,
        0xF025, (uint16_t)direction, MSM_CAMERA_I2C_BYTE_DATA);
    if (rc) {
        pr_err("%s:%d failed\n", __func__, __LINE__);
        msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
    }
    return rc;
}

static int32_t thp7212_set_multi_frame(struct msm_sensor_ctrl_t *s_ctrl, int32_t multi_frame)
{
    int32_t rc=0;
    pr_info("%s val:%d", __func__, multi_frame);
    
    //current state check

    if(g_res==6)
    {
        g_thp7217_multi_frame_save = 1;
        g_thp7217_multi_frame_save_parm = multi_frame;
        pr_err("%s save val:%d", __func__, multi_frame);
        return rc;
    }

    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
        s_ctrl->sensor_i2c_client,
        0xF01A, (uint16_t)multi_frame, MSM_CAMERA_I2C_BYTE_DATA);
    if (rc) {
        pr_err("%s:%d failed\n", __func__, __LINE__);
        msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
    }
    return rc;
}

static int32_t thp7212_set_ae_bracket(struct msm_sensor_ctrl_t *s_ctrl, struct sensor_bracket_info_t *bracket_info)
{
    int32_t rc=0, i=0;
    pr_info("%s val:%d %d", __func__, bracket_info->mode, bracket_info->exp_num);

    //current state check
    
    g_thp7212_bracket_mode = bracket_info->mode;
    if( bracket_info->exp_num < 4)
      thp7212_ae_bracket_parm[0].reg_data = bracket_info->exp_num + 1;
    else
      thp7212_ae_bracket_parm[0].reg_data = bracket_info->exp_num;

    if(g_thp7212_bracket_mode == CAMERA_BRACKET_MODE_ON) {
        for(i=0; i < bracket_info->exp_num; i++) {
            pr_info("%s exp_val:%d %d", __func__, i, bracket_info->exp_values[i]);
            thp7212_ae_bracket_parm[i+1].reg_data = 6 + bracket_info->exp_values[i] / 2;
        }
        if( bracket_info->exp_num < 4)
          thp7212_ae_bracket_parm[i+1].reg_data = 6 + bracket_info->exp_values[i-1] / 2;
    }

    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_conf_tbl(
            s_ctrl->sensor_i2c_client,
            thp7212_ae_bracket_parm,
            ARRAY_SIZE(thp7212_ae_bracket_parm),
            MSM_CAMERA_I2C_BYTE_DATA);
    if (rc) {
        pr_err("%s:%d failed\n", __func__, __LINE__);
        msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
    }
    return rc;
}

static int32_t thp7212_set_zoom(struct msm_sensor_ctrl_t *s_ctrl, int32_t zoom)
{
    int32_t rc=0;
    pr_info("%s val:%d", __func__, zoom);

    //current state check

    zoom /= 2;

    if(zoom > 30)
      return 0;

    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
        s_ctrl->sensor_i2c_client,
        0xF058, zoom, MSM_CAMERA_I2C_BYTE_DATA);
    if (rc) {
        pr_err("%s:%d failed\n", __func__, __LINE__);
        msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
    }
    return rc;
}

static int32_t thp7212_set_zoom_ex(struct msm_sensor_ctrl_t *s_ctrl, int32_t zoom)
{
    int32_t rc=0;
    pr_info("%s val:%d", __func__, zoom);

    //current state check

    if(zoom > 30)
      return 0;

    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
        s_ctrl->sensor_i2c_client,
        0xF061, zoom, MSM_CAMERA_I2C_BYTE_DATA);
    if (rc) {
        pr_err("%s:%d failed\n", __func__, __LINE__);
        msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
    }
    return rc;
}

static int32_t thp7212_set_antibanding(struct msm_sensor_ctrl_t *s_ctrl, int32_t antibanding)
{
    int32_t rc=0;
    pr_info("%s val:%d", __func__, antibanding);

    //current state check

    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
        s_ctrl->sensor_i2c_client,
        0xF023, antibanding, MSM_CAMERA_I2C_BYTE_DATA);
    if (rc) {
        pr_err("%s:%d failed\n", __func__, __LINE__);
        msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
    }
    return rc;
}

static int32_t thp7212_set_aec_lock(struct msm_sensor_ctrl_t *s_ctrl, int32_t lock)
{
    int32_t rc=0;
    pr_info("%s val:%d", __func__, lock);

    //current state check

    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
        s_ctrl->sensor_i2c_client,
        0xF020, lock, MSM_CAMERA_I2C_BYTE_DATA);
    if (rc) {
        pr_err("%s:%d failed\n", __func__, __LINE__);
        msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
    }
    return rc;
}

static int32_t thp7212_set_awb_lock(struct msm_sensor_ctrl_t *s_ctrl, int32_t lock)
{
    int32_t rc=0;
    pr_info("%s val:%d", __func__, lock);

    //current state check

    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
        s_ctrl->sensor_i2c_client,
        0xF038, lock, MSM_CAMERA_I2C_BYTE_DATA);
    if (rc) {
        pr_err("%s:%d failed\n", __func__, __LINE__);
        msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
    }
    return rc;
}

static int32_t thp7212_get_detect_scene(struct msm_sensor_ctrl_t *s_ctrl, void *setting)
{
    int32_t rc=0;
    uint16_t data = 0;
    
    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
            s_ctrl->sensor_i2c_client,
            0xF019, &data, MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
        pr_err("%s:%d: i2c_read failed\n", __func__, __LINE__);
        msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
        return rc;
    }
    pr_info("%s : %d", __func__, data);
    if (copy_to_user(setting, (void *)&data, sizeof(uint16_t))) {
        pr_err("%s:%d copy failed\n", __func__, __LINE__);
        rc = -EFAULT;
    }

    return rc;
}

static int32_t thp7212_get_face_detect_ex(struct msm_sensor_ctrl_t *s_ctrl, void *setting)
{
    int32_t rc=0;
    uint16_t i, j, fd_num;
    uint8_t fd_data[8];
    struct sensor_face_ex_info_t fd_ex_parm;
    int32_t coef_x, coef_y, correction;

    /* Face num read */
    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
                s_ctrl->sensor_i2c_client,
                0xF065, &fd_num, MSM_CAMERA_I2C_BYTE_DATA);
    if(fd_num > 4) fd_num = 4;
    fd_ex_parm.num_faces_detected = 0;
    pr_debug("##### %s num=%d\n", __func__, fd_num);
    if (rc < 0) {
        pr_err("%s:%d: i2c_read failed\n", __func__, __LINE__);
        msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
        return rc;
    }
    if (g_prev_w == g_prev_h) {
        coef_y = 1000 * thp7212_sensor_output_size[g_res].height / g_prev_h;
        coef_x = coef_y;
    } else {
        coef_x = 1000 * thp7212_sensor_output_size[g_res].width  / g_prev_w;
        coef_y = 1000 * thp7212_sensor_output_size[g_res].height / g_prev_h;
    }
    correction = (thp7212_sensor_output_size[g_res].width - thp7212_sensor_output_size[g_res].height) / 2;
    /* Face pos read */
    for(i=0,j=0; i<fd_num; i++)
    {
        rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read_seq(
                s_ctrl->sensor_i2c_client,
                0xF066+i*8, fd_data, sizeof(fd_data));
        if (rc < 0) {
            pr_err("%s:%d: i2c_read failed\n", __func__, __LINE__);
            msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
            return rc;
        }
        if (g_prev_w == g_prev_h) {
            if( ((int16_t)(fd_data[0] << 8 | fd_data[1]) >= correction) &&
                ((int16_t)(fd_data[2] << 8 | fd_data[3]) >= 0) &&
                ((fd_data[4] << 8 | fd_data[5]) <= (thp7212_sensor_output_size[g_res].width-correction)) &&
                ((fd_data[6] << 8 | fd_data[7]) <= thp7212_sensor_output_size[g_res].height) ) {

                fd_ex_parm.roi[j].x  = 1000 * ((fd_data[0] << 8 | fd_data[1])-correction) / coef_x;
                fd_ex_parm.roi[j].y  = 1000 * (fd_data[2] << 8 | fd_data[3]) / coef_y;
                fd_ex_parm.roi[j].dx = 1000 * ((fd_data[4] << 8 | fd_data[5])-correction) / coef_x;
                fd_ex_parm.roi[j].dy = 1000 * (fd_data[6] << 8 | fd_data[7]) / coef_y;
                fd_ex_parm.roi[j].dx -= fd_ex_parm.roi[j].x;
                fd_ex_parm.roi[j].dy -= fd_ex_parm.roi[j].y;
                fd_ex_parm.num_faces_detected = ++j;
            }
        } else {
            if( ((int16_t)(fd_data[0] << 8 | fd_data[1]) >= 0) &&
                ((int16_t)(fd_data[2] << 8 | fd_data[3]) >= 0) &&
                ((fd_data[4] << 8 | fd_data[5]) <= thp7212_sensor_output_size[g_res].width) &&
                ((fd_data[6] << 8 | fd_data[7]) <= thp7212_sensor_output_size[g_res].height) ) {

                fd_ex_parm.roi[j].x  = 1000 * (fd_data[0] << 8 | fd_data[1]) / coef_x;
                fd_ex_parm.roi[j].y  = 1000 * (fd_data[2] << 8 | fd_data[3]) / coef_y;
                fd_ex_parm.roi[j].dx = 1000 * (fd_data[4] << 8 | fd_data[5]) / coef_x;
                fd_ex_parm.roi[j].dy = 1000 * (fd_data[6] << 8 | fd_data[7]) / coef_y;
                fd_ex_parm.roi[j].dx -= fd_ex_parm.roi[j].x;
                fd_ex_parm.roi[j].dy -= fd_ex_parm.roi[j].y;
                fd_ex_parm.num_faces_detected = ++j;
            }
        }
        pr_debug("##### %s %d:[%d,%d],[%d,%d]\n", __func__, j, 
            fd_ex_parm.roi[j].x, fd_ex_parm.roi[j].y, 
            fd_ex_parm.roi[j].dx, fd_ex_parm.roi[j].dy);

    }

    if (copy_to_user(setting, (void *)&fd_ex_parm, sizeof(struct sensor_face_ex_info_t))) {
        pr_err("%s:%d copy failed\n", __func__, __LINE__);
        rc = -EFAULT;
    }

    return rc;
}
static int32_t thp7212_get_maker_note(struct msm_sensor_ctrl_t *s_ctrl, void *setting)
{
    int32_t rc=0;
    uint8_t data[128];

    memset(data, 0x00 , sizeof(data));

    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read_seq(
		s_ctrl->sensor_i2c_client, 0xFC80, data, sizeof(data));
    if (rc < 0) {
        pr_err("%s:%d: i2c_read failed\n", __func__, __LINE__);
        msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
        return rc;
    }

    if (copy_to_user(setting, (void *)data, sizeof(data))) {
        pr_err("%s:%d copy failed\n", __func__, __LINE__);
        rc = -EFAULT;
    }
    return rc;
}

static int32_t thp7212_get_exp_time(struct msm_sensor_ctrl_t *s_ctrl, uint32_t *exposure_time)
{
    int32_t rc=0;
    uint8_t data[2]={0x27,0x10};//default 1000ms

    pr_info("%s [%02x][%02x]\n", __func__, data[0], data[1]);
    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read_seq(
		s_ctrl->sensor_i2c_client, 0xF026, data, sizeof(data));
    if (rc < 0) {
        pr_err("%s:%d: i2c_read failed\n", __func__, __LINE__);
        msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
        return rc;
    }
  	*exposure_time = ((data[0]<<8) | data[1]);
    pr_info("%s [%d]\n", __func__, *exposure_time );

    return rc;
}

static int32_t thp7212_get_illumi_estimate(struct msm_sensor_ctrl_t *s_ctrl, void *setting)
{
    int32_t rc=0;
    uint16_t data;

    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
		s_ctrl->sensor_i2c_client, 0xF031, &data, MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
        pr_err("%s:%d: i2c_read failed\n", __func__, __LINE__);
        msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
        return rc;
    }
    pr_info("%s [%d]\n", __func__, data);

    if (copy_to_user(setting, (void *)&data, sizeof(uint16_t))) {
        pr_err("%s:%d copy failed\n", __func__, __LINE__);
        rc = -EFAULT;
    }
    return rc;
}

static int32_t thp7212_get_lens_position(struct msm_sensor_ctrl_t *s_ctrl, void *setting)
{
    int32_t rc=0;
    uint16_t data;

    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
		s_ctrl->sensor_i2c_client, 0xF043, &data, MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
        pr_err("%s:%d: i2c_read failed\n", __func__, __LINE__);
        msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
        return rc;
    }
    data = ~data & 0x0001;

    if (copy_to_user(setting, (void *)&data, sizeof(uint16_t))) {
        pr_err("%s:%d copy failed\n", __func__, __LINE__);
        rc = -EFAULT;
    }
    return rc;
}

static int32_t thp7212_get_af_status(struct msm_sensor_ctrl_t *s_ctrl, void *setting)
{
    int32_t rc=0;
    uint16_t data;

    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
		s_ctrl->sensor_i2c_client, 0xF042, &data, MSM_CAMERA_I2C_BYTE_DATA);
    if (rc < 0) {
        pr_err("%s:%d: i2c_read failed\n", __func__, __LINE__);
        msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
        return rc;
    }

    if (copy_to_user(setting, (void *)&data, sizeof(uint16_t))) {
        pr_err("%s:%d copy failed\n", __func__, __LINE__);
        rc = -EFAULT;
    }
    return rc;
}

static int32_t thp7212_get_fw_ver(struct msm_sensor_ctrl_t *s_ctrl, void *setting)
{
    int32_t rc=0;
    uint16_t ver_data;

    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
                s_ctrl->sensor_i2c_client,
                0xF000, &ver_data, MSM_CAMERA_I2C_BYTE_DATA);

    pr_err("##### FW_ver = 0x%02x\n", ver_data);

    if (rc) {
        pr_err("%s:%d failed\n", __func__, __LINE__);
        msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
    }
    return rc;
}

static int32_t thp7212_get_maker_note_dm(struct msm_sensor_ctrl_t *s_ctrl, void *setting)
{
    int32_t rc=0;
    int16_t i;
    uint8_t data[128];

    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read_seq(
                s_ctrl->sensor_i2c_client,
                0xFC80, data, sizeof(data));

    for(i=0; i<sizeof(data); i++)
    {
      pr_err("##### MakerNote(0x%02x) = 0x%02x\n", i, data[i]);
    }

    if (rc) {
        pr_err("%s:%d failed\n", __func__, __LINE__);
        msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
    }
    return rc;
}

static int32_t thp7212_set_cci_write(struct msm_sensor_ctrl_t *s_ctrl, void *setting)
{
    int32_t rc=0;
    uint8_t i2c_data[3];
    uint16_t addr;

    if (copy_from_user(i2c_data,
        (void *)setting,
        sizeof(i2c_data))) {
        return -EFAULT;
    }

    addr = (uint16_t)i2c_data[0] << 8 | i2c_data[1];

    pr_err("##### cci_write: 0x%04x,0x%02x\n", addr, i2c_data[2]);

    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
        s_ctrl->sensor_i2c_client,
        addr, i2c_data[2], MSM_CAMERA_I2C_BYTE_DATA);

    if (rc) {
        pr_err("%s:%d failed\n", __func__, __LINE__);
        msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
    }
    return rc;
}

static int32_t thp7212_set_cci_read(struct msm_sensor_ctrl_t *s_ctrl, void *setting)
{
    int32_t rc=0;
    uint8_t  i2c_data[3];
    uint16_t i,addr;
    uint8_t  read_data[255];

    if (copy_from_user(i2c_data,
        (void *)setting,
        sizeof(i2c_data))) {
        return -EFAULT;
    }

    addr = (uint16_t)i2c_data[0] << 8 | i2c_data[1];

    pr_err("##### cci_read: 0x%04x,0x%02x\n", addr, i2c_data[2]);

    rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read_seq(
                s_ctrl->sensor_i2c_client,
                addr, read_data, i2c_data[2]);

    for(i=0; i<i2c_data[2]; i++)
    {
      pr_err("##### cci_read_data(0x%02x) = 0x%02x\n", i, read_data[i]);
    }

    if (rc) {
        pr_err("%s:%d failed\n", __func__, __LINE__);
        msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_I2C_ERROR);
    }
    return rc;
}

static int32_t thp7212_platform_probe(struct platform_device *pdev)
{
    int32_t rc;
    const struct of_device_id *match;
    match = of_match_device(thp7212_dt_match, &pdev->dev);
    rc = msm_sensor_platform_probe(pdev, match->data);
    return rc;
}

static int __init thp7212_init_module(void)
{
    int32_t rc;
    pr_info("%s:%d\n", __func__, __LINE__);
    rc = platform_driver_probe(&thp7212_platform_driver, thp7212_platform_probe);
    pr_info("%s:%d rc %d\n", __func__, __LINE__, rc);
    return rc;
}

static void __exit thp7212_exit_module(void)
{
    pr_info("%s:%d\n", __func__, __LINE__);
    msm_sensor_free_sensor_data(&thp7212_s_ctrl);
    platform_driver_unregister(&thp7212_platform_driver);
    return;
}

int32_t thp7212_sensor_config(struct msm_sensor_ctrl_t *s_ctrl,
    void __user *argp)
{
    struct sensorb_cfg_data *cdata = (struct sensorb_cfg_data *)argp;
    long rc = 0;
    int32_t i = 0;
    mutex_lock(s_ctrl->msm_sensor_mutex);
    pr_info("%s:%d %s cfgtype = %d\n", __func__, __LINE__,
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
          rc = thp7212_fwdl(s_ctrl);
        break;

    case CFG_SET_RESOLUTION: {
        struct sensor_res_info_t *res_val;
        res_val = (struct sensor_res_info_t *)&cdata->cfg.res_info;
        rc = thp7212_set_resolution(s_ctrl, res_val->res, res_val->stream_mask, res_val->prev_width, res_val->prev_height);
        break;
    }
    case CFG_SET_STOP_STREAM:
        rc = thp7212_stop_stream(s_ctrl);
        break;
    case CFG_SET_START_STREAM:
        rc = thp7212_start_stream(s_ctrl);
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
        kfree(reg_setting);
        break;
    }

    case CFG_POWER_UP:
        if (s_ctrl->func_tbl->sensor_power_up){
            rc = s_ctrl->func_tbl->sensor_power_up(s_ctrl);
        }
        else
            rc = -EFAULT;
        g_res = 0;
        g_thp7212_af_roi = 0;
        g_thp7212_af_mode = 0;
        g_snapshot_flg = 0;
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

    case CFG_SET_WHITE_BALANCE:
        rc = thp7212_set_white_balance(s_ctrl, cdata->cfg.wb);
        break;
    case CFG_SET_EFFECT:
        rc = thp7212_set_effect(s_ctrl, cdata->cfg.effect);
        break;
    case CFG_SET_EXPOSURE_COMPENSATION:
        rc = thp7212_set_exposure_compensation(s_ctrl, cdata->cfg.exposure);
        break;
    case CFG_SET_BESTSHOT:
        rc = thp7212_set_bestshot(s_ctrl, cdata->cfg.bestshot);
        break;
    case CFG_SET_FOCUS_MODE:{
        struct sensor_af_info_t *af_info;
        af_info = (struct sensor_af_info_t *)&cdata->cfg.af_info;
        rc = thp7212_set_focus_mode(s_ctrl, af_info);
        break;
    }
    case CFG_SET_AF_ROI:{
        struct sensor_af_roi_info_t *af_roi;
        af_roi = (struct sensor_af_roi_info_t *)&cdata->cfg.af_roi;
        rc = thp7212_set_af_roi(s_ctrl, af_roi);
        break;
    }
    case CFG_SET_DIRECTION_OF_PHONE:
        rc = thp7212_set_direction_of_phone(s_ctrl, cdata->cfg.direction);
        break;
    case CFG_SET_MULTI_FRAME:
        rc = thp7212_set_multi_frame(s_ctrl, cdata->cfg.multi_frame);
        break;
    case CFG_SET_AE_BRACKET:{
        struct sensor_bracket_info_t *bracket_info;
        bracket_info = (struct sensor_bracket_info_t *)&cdata->cfg.bracket_info;
        rc = thp7212_set_ae_bracket(s_ctrl, bracket_info);
        break;
    }
    case CFG_SET_FW_UPDATE:
        rc = thp7212_fw_update(cdata->cfg.setting);
        break;
    case CFG_SET_ZOOM:
        rc = thp7212_set_zoom(s_ctrl, cdata->cfg.zoom);
        break;
    case CFG_SET_ZOOM_EX:
        rc = thp7212_set_zoom_ex(s_ctrl, cdata->cfg.zoom);
        break;
    case CFG_SET_ANTIBANDING:
        rc = thp7212_set_antibanding(s_ctrl, cdata->cfg.antibanding);
        break;
    case CFG_SET_AEC_LOCK:
        rc = thp7212_set_aec_lock(s_ctrl, cdata->cfg.lock);
        break;
    case CFG_SET_AWB_LOCK:
        rc = thp7212_set_awb_lock(s_ctrl, cdata->cfg.lock);
        break;
    case CFG_GET_DETECT_SCENE:
        rc = thp7212_get_detect_scene(s_ctrl, cdata->cfg.setting);
        break;
    case CFG_GET_FD_EX:
        rc = thp7212_get_face_detect_ex(s_ctrl, cdata->cfg.setting);
        break;
    case CFG_GET_MAKER_NOTE:
        rc = thp7212_get_maker_note(s_ctrl, cdata->cfg.setting);
        break;
    case CFG_GET_EXP_TIME:
        rc = thp7212_get_exp_time(s_ctrl, &cdata->cfg.exp_time);
        argp = (void *)cdata;
        break;
    case CFG_GET_ILLUMI_ESTIMATE:
        rc = thp7212_get_illumi_estimate(s_ctrl, cdata->cfg.setting);
        break;
    case CFG_GET_LENS_POSITION:
        rc = thp7212_get_lens_position(s_ctrl, cdata->cfg.setting);
        break;
    case CFG_GET_AF_STATUS:
        rc = thp7212_get_af_status(s_ctrl, cdata->cfg.setting);
        break;
    case CFG_GET_FW_VER:
        rc = thp7212_get_fw_ver(s_ctrl, cdata->cfg.setting);
        break;
    case CFG_GET_MAKER_NOTE_DM:
        rc = thp7212_get_maker_note_dm(s_ctrl, cdata->cfg.setting);
        break;
    case CFG_SET_CCI_WRITE:
        rc = thp7212_set_cci_write(s_ctrl, cdata->cfg.setting);
        break;
    case CFG_SET_CCI_READ:
        rc = thp7212_set_cci_read(s_ctrl, cdata->cfg.setting);
        break;
    case CFG_SET_TIMEOUT_ERROR:
        msm_error_notify((unsigned int)s_ctrl->sensordata->sensor_info->session_id, MSM_CAMERA_PRIV_TIMEOUT_ERROR);
        break;
    default:
        break;
    }

    mutex_unlock(s_ctrl->msm_sensor_mutex);

    return rc;
}

static struct msm_sensor_fn_t thp7212_sensor_func_tbl = {
    .sensor_config = thp7212_sensor_config,
    .sensor_power_up = msm_sensor_power_up,
    .sensor_power_down = msm_sensor_power_down,
    .sensor_match_id = msm_sensor_match_id,
};

static struct msm_sensor_ctrl_t thp7212_s_ctrl = {
    .sensor_i2c_client = &thp7212_sensor_i2c_client,
    .power_setting_array.power_setting = thp7212_power_setting,
    .power_setting_array.size = ARRAY_SIZE(thp7212_power_setting),
    .msm_sensor_mutex = &thp7212_mut,
    .sensor_v4l2_subdev_info = thp7212_subdev_info,
    .sensor_v4l2_subdev_info_size = ARRAY_SIZE(thp7212_subdev_info),
    .func_tbl = &thp7212_sensor_func_tbl,
};

module_init(thp7212_init_module);
module_exit(thp7212_exit_module);
MODULE_DESCRIPTION("Sony 13M YUV sensor driver");
MODULE_LICENSE("GPL v2");
