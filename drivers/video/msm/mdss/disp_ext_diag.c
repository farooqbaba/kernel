/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 * (C) 2013 KYOCERA Corporation
 *
 * drivers/video/msm/disp_ext_diag.c
 *
 * Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
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
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include "mdss_fb.h"
#include "mdss_dsi.h"
#include "mdss_mdp.h"
#include "disp_ext.h"

int disp_ext_diag_get_err_crc(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
	void __user *argp = (void __user *)arg;
	struct dsi_cmd_desc dm_dsi_cmds;
	struct dcs_cmd_req cmdreq;
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	char mipi_reg;
	u32 err_data;
	u32 rd_data, timeout_cnt = 0x00;
	int ret;

	/* MIPI 05H register read Start*/
	memset(&dm_dsi_cmds, 0x00, sizeof(dm_dsi_cmds));
	mipi_reg = 0x05; /* RDNUMED reg(CRC ERR count read) */
	dm_dsi_cmds.dchdr.dtype   = DTYPE_DCS_READ; /* Command type */
	dm_dsi_cmds.dchdr.last    = 1; /* Last command */
	dm_dsi_cmds.dchdr.vc      = 0; /* Virtual Channel */
	dm_dsi_cmds.dchdr.ack     = 1; /* Don't care, dsi_host default ON set */
	dm_dsi_cmds.dchdr.wait    = 0; /* wait response by msleep() */
	dm_dsi_cmds.dchdr.dlen    = 1; /* Data length */
	dm_dsi_cmds.payload = &mipi_reg;   /* Data */

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &dm_dsi_cmds;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT;
	cmdreq.rlen = 1;
	cmdreq.cb = NULL;

	pdata = dev_get_platdata(&mfd->pdev->dev);
	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	mdss_dsi_cmdlist_put(ctrl_pdata, &cmdreq);

	rd_data = (u32)(*(ctrl_pdata->rx_buf.data));
	timeout_cnt = (u32)disp_ext_util_get_crc_error();

	rd_data = ((rd_data & 0xFF00) >> 8) | ((rd_data & 0x00FF) << 8);
	timeout_cnt = ((timeout_cnt & 0xFF00) >> 8) | ((timeout_cnt & 0x00FF) << 8);
	err_data = ((0x0000FFFF & timeout_cnt) << 16) | (0x0000FFFF & rd_data);

	ret = copy_to_user(argp, &err_data, 4);

	return ret;
}

int disp_ext_diag_reg_write(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
	void __user *argp = (void __user *)arg;
	int ret;
	struct dsi_cmd_desc dm_dsi_cmds;
	struct disp_diag_mipi_reg_type mipi_reg_data;
	struct dcs_cmd_req cmdreq;
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	ret = copy_from_user(&mipi_reg_data, argp, sizeof(mipi_reg_data));
	if (ret) {
		pr_err("MSMFB_MIPI_REG_WRITE: error 1[%d] \n", ret);
		return ret;
	}

	if ((mipi_reg_data.type != DTYPE_DCS_WRITE)
	 && (mipi_reg_data.type != DTYPE_DCS_WRITE1)
	 && (mipi_reg_data.type != DTYPE_DCS_LWRITE)
	 && (mipi_reg_data.type != DTYPE_GEN_WRITE)
	 && (mipi_reg_data.type != DTYPE_GEN_WRITE1)
	 && (mipi_reg_data.type != DTYPE_GEN_WRITE2)
	 && (mipi_reg_data.type != DTYPE_GEN_LWRITE)) {
		pr_err("MSMFB_MIPI_REG_WRITE: error 2[%d] \n", mipi_reg_data.type);
		return -EINVAL;
	}

	/* Tx command send */
	memset(&dm_dsi_cmds, 0x00, sizeof(dm_dsi_cmds));
	dm_dsi_cmds.dchdr.dtype = mipi_reg_data.type;     /* Command type */
	dm_dsi_cmds.dchdr.last = 1;   /* Last command */
	dm_dsi_cmds.dchdr.vc = 0;     /* Virtual Channel */
	dm_dsi_cmds.dchdr.ack = 0;    /* No ACK trigger msg from peripeheral */
	dm_dsi_cmds.dchdr.wait = mipi_reg_data.wait;      /* wait response by msleep() */
	dm_dsi_cmds.dchdr.dlen = mipi_reg_data.len;       /* Data length */
	dm_dsi_cmds.payload = (char *)mipi_reg_data.data;   /* Data */
	pr_info("@@@ Tx command\n");
	pr_info("    - dtype = 0x%08X\n", dm_dsi_cmds.dchdr.dtype);
	pr_info("    - last  = %d\n", dm_dsi_cmds.dchdr.last);
	pr_info("    - vc    = %d\n", dm_dsi_cmds.dchdr.vc);
	pr_info("    - ack   = %d\n", dm_dsi_cmds.dchdr.ack);
	pr_info("    - wait  = %d\n", dm_dsi_cmds.dchdr.wait);
	pr_info("    - dlen  = %d\n", dm_dsi_cmds.dchdr.dlen);
	pr_info("    - payload:%02X %02X %02X %02X\n",
		dm_dsi_cmds.payload[0], dm_dsi_cmds.payload[1],
		dm_dsi_cmds.payload[2], dm_dsi_cmds.payload[3]);
	pr_info("    -         %02X %02X %02X %02X\n",
		dm_dsi_cmds.payload[4], dm_dsi_cmds.payload[5],
		dm_dsi_cmds.payload[6], dm_dsi_cmds.payload[7]);

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &dm_dsi_cmds;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_COMMIT;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	pdata = dev_get_platdata(&mfd->pdev->dev);
	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	mdss_dsi_cmdlist_put(ctrl_pdata, &cmdreq);

	return ret;
}

int disp_ext_diag_reg_read(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
	void __user *argp = (void __user *)arg;
	int ret;
	struct dsi_cmd_desc dm_dsi_cmds;
	struct disp_diag_mipi_reg_type mipi_reg_data;
	struct dcs_cmd_req cmdreq;
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	ret = copy_from_user(&mipi_reg_data, argp, sizeof(mipi_reg_data));
	if (ret) {
		pr_err("MSMFB_MIPI_REG_READ: error 1[%d] \n", ret);
		return ret;
	}

	if ((mipi_reg_data.type != DTYPE_DCS_READ)
	 && (mipi_reg_data.type != DTYPE_GEN_READ)
	 && (mipi_reg_data.type != DTYPE_GEN_READ1)
	 && (mipi_reg_data.type != DTYPE_GEN_READ2)) {
		pr_err("MSMFB_MIPI_REG_READ: error 2[%d] \n", mipi_reg_data.type);
		return -EINVAL;
	}

	/* Rx command send */
	memset(&dm_dsi_cmds, 0x00, sizeof(dm_dsi_cmds));
	dm_dsi_cmds.dchdr.dtype = mipi_reg_data.type; /* Command type */
	dm_dsi_cmds.dchdr.last = 1; /* Last command */
	dm_dsi_cmds.dchdr.vc = 0; /* Virtual Channel */
	dm_dsi_cmds.dchdr.ack = 1; /* Don't care, dsi_host default ON set */
	dm_dsi_cmds.dchdr.wait = mipi_reg_data.wait; /* wait response by msleep() */
	dm_dsi_cmds.dchdr.dlen = mipi_reg_data.len; /* Data length */
	dm_dsi_cmds.payload = (char *)mipi_reg_data.data; /* Data */
	pr_info("@@@ Rx command\n");
	pr_info("    - dtype = 0x%08X\n", dm_dsi_cmds.dchdr.dtype);
	pr_info("    - last  = %d\n", dm_dsi_cmds.dchdr.last);
	pr_info("    - vc    = %d\n", dm_dsi_cmds.dchdr.vc);
	pr_info("    - ack   = %d\n", dm_dsi_cmds.dchdr.ack);
	pr_info("    - wait  = %d\n", dm_dsi_cmds.dchdr.wait);
	pr_info("    - dlen  = %d\n", dm_dsi_cmds.dchdr.dlen);
	pr_info("    - payload:%02X %02X %02X %02X\n",
		dm_dsi_cmds.payload[0], dm_dsi_cmds.payload[1],
		dm_dsi_cmds.payload[2], dm_dsi_cmds.payload[3]);
	pr_info("    -         %02X %02X %02X %02X\n",
		dm_dsi_cmds.payload[4], dm_dsi_cmds.payload[5],
		dm_dsi_cmds.payload[6], dm_dsi_cmds.payload[7]);

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = &dm_dsi_cmds;
	cmdreq.cmds_cnt = 1;
	cmdreq.flags = CMD_REQ_RX | CMD_REQ_COMMIT;
	cmdreq.rlen = 1;
	cmdreq.cb = NULL;

	pdata = dev_get_platdata(&mfd->pdev->dev);
	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	mdss_dsi_cmdlist_put(ctrl_pdata, &cmdreq);

	memcpy(mipi_reg_data.data, ctrl_pdata->rx_buf.data, ctrl_pdata->rx_buf.len);

	ret = copy_to_user(argp, &mipi_reg_data, sizeof(mipi_reg_data));
	if (ret) {
		pr_err("MSMFB_MIPI_REG_READ: error 3[%d] \n", ret);
	}

	return ret;
}

int disp_ext_diag_tx_rate(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
    struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
    void __user *argp = (void __user *)arg;
    int ret;
    u32 input_data;
    u32 msmfb_rate;
    int new_fps;

    /* Get arg */
    ret = copy_from_user(&input_data, argp, sizeof(uint));
    if (ret) {
        pr_err("MSMFB_CHANGE_TRANSFER_RATE: error 1[%d] \n", ret);
        /* Error */
        return ret;
    }
    msmfb_rate = ((input_data << 8) & 0xFF00) | ((input_data >> 8) & 0x00FF) * 1000000;

    new_fps = mfd->panel_info->mipi.frame_rate * msmfb_rate / mfd->panel_info->clk_rate;

    pr_info("%s: new_fps =%d\n", __func__, new_fps);

    ret = mdss_dsi_clk_div_config(mfd->panel_info, new_fps);
    if (ret) {
        pr_err("MSMFB_CHANGE_TRANSFER_RATE: error 2[%d] \n", ret);
        /* Error */
        return ret;
    }

    return 0;
}

void disp_ext_diag_init(void)
{
}
