/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 * (C) 2013 KYOCERA Corporation
 *
 * drivers/video/msm/disp_ext.h
 *
 * Copyright (c) 2008-2010, The Linux Foundation. All rights reserved.
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
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
*/
#ifndef DISP_EXT_H
#define DISP_EXT_H
#include <linux/msm_mdp.h>
#include "mdss_fb.h"
#include "mdss_dsi.h"

#define FEATURE_DISP_LOCAL_LOG
/* #undef FEATURE_DISP_LOCAL_LOG */
extern uint8_t fb_dbg_msg_level;
extern u32 msm_fb_msg_level;
#ifdef FEATURE_DISP_LOCAL_LOG
#define DISP_LOCAL_LOG_EMERG(msg, ...) if (fb_dbg_msg_level>0) printk(KERN_EMERG msg, ## __VA_ARGS__);
#define MSM_FB_EMERG(msg, ...)    \
	if (msm_fb_msg_level > 0)  \
		printk(KERN_EMERG msg, ## __VA_ARGS__);
#define MSM_FB_ALERT(msg, ...)    \
	if (msm_fb_msg_level > 1)  \
		printk(KERN_ALERT msg, ## __VA_ARGS__);
#define MSM_FB_CRIT(msg, ...)    \
	if (msm_fb_msg_level > 2)  \
		printk(KERN_CRIT msg, ## __VA_ARGS__);
#define MSM_FB_ERR(msg, ...)    \
	if (msm_fb_msg_level > 3)  \
		printk(KERN_ERR msg, ## __VA_ARGS__);
#define MSM_FB_WARNING(msg, ...)    \
	if (msm_fb_msg_level > 4)  \
		printk(KERN_WARNING msg, ## __VA_ARGS__);
#define MSM_FB_NOTICE(msg, ...)    \
	if (msm_fb_msg_level > 5)  \
		printk(KERN_NOTICE msg, ## __VA_ARGS__);
#define MSM_FB_INFO(msg, ...)    \
	if (msm_fb_msg_level > 6)  \
		printk(KERN_INFO msg, ## __VA_ARGS__);
#define MSM_FB_DEBUG(msg, ...)    \
	if (msm_fb_msg_level > 7)  \
		printk(KERN_INFO msg, ## __VA_ARGS__);
#else
#define DISP_LOCAL_LOG_EMERG(msg, ...) (void)0;
#define MSM_FB_EMERG(msg, ...)   (void)0;
#define MSM_FB_ALERT(msg, ...)   (void)0;
#define MSM_FB_CRIT(msg, ...)    (void)0;
#define MSM_FB_ERR(msg, ...)     (void)0;
#define MSM_FB_WARNING(msg, ...) (void)0;
#define MSM_FB_NOTICE(msg, ...)  (void)0;
#define MSM_FB_INFO(msg, ...)    (void)0;
#define MSM_FB_DEBUG(msg, ...)   (void)0;
#endif

void disp_ext_set_dmflag(int dmflag);
int disp_ext_is_invalid(void);

#ifdef CONFIG_DISP_EXT_REFRESH
void disp_ext_refresh_te_monitor_timer_set( void );
void disp_ext_refresh_te_monitor_timer_release( void );
void disp_ext_refresh_set_te_monitor_init(void);
void disp_ext_refresh_seq( struct msm_fb_data_type *mfd, unsigned int cmd );
void disp_ext_refresh_err_report_check(struct msm_fb_data_type *mfd);
irqreturn_t disp_ext_refresh_hw_vsync_irq(int irq, void *data);
u8 disp_ext_reflesh_get_enable(void);
void disp_ext_reflesh_set_enable(u8 flag);
u8 disp_ext_reflesh_get_start(void);
void disp_ext_reflesh_set_start(u8 flag);
void disp_ext_reflesh_before_te_run_count_init(void);
void disp_ext_reflesh_init(void);
u8 disp_ext_reflesh_get_sw(void);
void disp_ext_reflesh_set_sw(u8 sw);
void disp_ext_reflesh_start(void);
#endif /* CONFIG_DISP_EXT_REFRESH */

#ifdef CONFIG_DISP_EXT_BOARD
int disp_ext_board_detect_board(struct mdss_dsi_ctrl_pdata *ctrl);
int disp_ext_board_get_panel_detect(void);
void disp_ext_board_set_panel_detect(int flag);
#endif /* CONFIG_DISP_EXT_BOARD */

#ifdef CONFIG_DISP_EXT_BLC
u8 disp_ext_blc_get_select_mode(void);
void disp_ext_blc_set_select_mode(u8 sw);
void disp_ext_blc_init(void);

void disp_ext_util_cabc_lock( void );
void disp_ext_util_cabc_unlock( void );
#endif /* CONFIG_DISP_EXT_BLC */

#ifdef CONFIG_DISP_EXT_DIAG
int disp_ext_diag_get_err_crc(struct fb_info *info, unsigned int cmd, unsigned long arg);
int disp_ext_diag_reg_write(struct fb_info *info, unsigned int cmd, unsigned long arg);
int disp_ext_diag_reg_read(struct fb_info *info, unsigned int cmd, unsigned long arg);
int disp_ext_diag_standby(struct fb_info *info, unsigned int cmd, unsigned long arg);
int disp_ext_diag_tx_rate(struct fb_info *info, unsigned int cmd, unsigned long arg);
void disp_ext_diag_init(void);

uint32_t disp_ext_util_get_crc_error(void);
void disp_ext_util_set_crc_error(uint32_t count);
void disp_ext_util_crc_countup(void);
#endif /* CONFIG_DISP_EXT_DIAG */

#ifdef CONFIG_DISP_EXT_PROPERTY
void disp_ext_util_set_kcjprop(struct mdss_panel_data *pdata, struct fb_kcjprop_data* kcjprop_data);
#endif /* CONFIG_DISP_EXT_PROPERTY */

#ifdef CONFIG_DISP_EXT_UTIL
local_disp_state_enum disp_ext_util_get_disp_state(void);
void disp_ext_util_set_disp_state(local_disp_state_enum state);
struct local_disp_state_type *disp_ext_util_get_disp_info(void);
void disp_ext_util_disp_local_init( void );
void disp_ext_util_disp_local_lock( void );
void disp_ext_util_disp_local_unlock( void );
void disp_ext_util_mipitx_lock( void );
void disp_ext_util_mipitx_unlock( void );
#ifdef CONFIG_DISP_EXT_UTIL_GET_RATE
u32 disp_ext_util_get_refresh_rate( void );
#endif /*CONFIG_DISP_EXT_UTIL_GET_RATE*/
#ifdef CONFIG_DISP_UTIL_DSI_CLK_OFF
void disp_ext_util_dsi_clk_off( void );
#endif /*CONFIG_DISP_UTIL_DSI_CLK_OFF*/
#ifdef CONFIG_DISP_EXT_UTIL_VSYNC
u32 disp_ext_util_get_total_line( void );
u32 disp_ext_util_get_vsync_count( void );
u32 disp_ext_util_vsync_cal_start( u32 start_y );
#endif /*CONFIG_DISP_EXT_UTIL_VSYNC*/
#endif /* CONFIG_DISP_EXT_UTIL */

#endif /* DISP_EXT_H */
