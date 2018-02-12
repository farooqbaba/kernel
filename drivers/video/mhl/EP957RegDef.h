/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2013 KYOCERA Corporation
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */
/******************************************************************************\

          (c) Copyright Explore Semiconductor, Inc. Limited 2005
                           ALL RIGHTS RESERVED 

--------------------------------------------------------------------------------

 Please review the terms of the license agreement before using this file.
 If you are not an authorized user, please destroy this source code file  
 and notify Explore Semiconductor Inc. immediately that you inadvertently 
 received an unauthorized copy.  

--------------------------------------------------------------------------------

  File        :  EP957RegDef.h

  Description :  Register Address definitions of EP957B revision D and EP958B.

\******************************************************************************/

#ifndef EP957REGDEF_H
#define EP957REGDEF_H

// Registers								Word	BitMask
#define EP957_TX_PHY_Control_0				0x00
#define EP957_TX_PHY_Control_0__TERM1_DIS			0x80
#define EP957_TX_PHY_Control_0__TERM2_EN			0x40

#define EP957_TX_PHY_Control_1				0x01
#define EP957_TX_PHY_Control_1__RSEN_DIS			0x40
#define EP957_TX_PHY_Control_1__VCO_Gain			0x0C
#define EP957_TX_PHY_Control_1__PHD_CUR				0x03

#define EP957_RX_PHY_Control_0				0x02
#define EP957_RX_PHY_Control_0__PLL_PHD				0x80
#define EP957_RX_PHY_Control_0__PLL_REG				0x40
#define EP957_RX_PHY_Control_0__EQ_BIAS				0x30
#define EP957_RX_PHY_Control_0__PLL_PUMP			0x0C
#define EP957_RX_PHY_Control_0__PLL_BW				0x02
#define EP957_RX_PHY_Control_0__EQ_GAIN				0x01

#define EP957_General_Control_1				0x08
#define EP957_General_Control_1__INT_POL			0x80
#define EP957_General_Control_1__INT_OD				0x40
#define EP957_General_Control_1__INT_PUL			0x20
#define EP957_General_Control_1__HPD				0x10
#define EP957_General_Control_1__EDID_EN			0x08
#define EP957_General_Control_1__TX_PU				0x04
#define EP957_General_Control_1__RX_PU				0x02
#define EP957_General_Control_1__OSC_PU				0x01

#define EP957_General_Control_2				0x09
#define EP957_General_Control_2__RSEN_IE			0x40
#define EP957_General_Control_2__AKSV_IE			0x20
#define EP957_General_Control_2__RSEN				0x08
#define EP957_General_Control_2__RSEN_F				0x02
#define EP957_General_Control_2__AKSV_F				0x01

#define EP957_General_Control_3				0x0A
#define EP957_General_Control_3__DK					0xE0
#define EP957_General_Control_3__USB_Pol			0x10 	// New added in version D
#define EP957_General_Control_3__MUX_EN				0x08
#define EP957_General_Control_3__PP_MODE			0x04
//#define EP957_General_Control_3__DDC_PUL			0x02
#define EP957_General_Control_3__OSC_Sel			0x01 	// New added in version D

#define EP957_General_Control_4				0x0B
#define EP957_General_Control_4__T_CYCLE			0xE0
#define EP957_General_Control_4__LINK_RST_EN		0x10
#define EP957_General_Control_4__VBUS				0x08
#define EP957_General_Control_4__HDMI				0x04
#define EP957_General_Control_4__DE_VALID			0x02
#define EP957_General_Control_4__LINK_ON			0x01

#define EP957_AKSV							0x10			// AKSV1-AKSV5 0x10-0x14

#define EP957_AINFO							0x15			// AINFO

#define EP957_AN							0x16			// AN1-AN8 0x16-0x1D

#define EP957_RI							0x1E			// RI1-RI2 0x1E-0x1F

#define EP957_BKSV							0x20			// BKSV1-BKSV5 0x20-0x24

#define EP957_BCAP							0x25			// 0x25

#define EP957_BSTATUS						0x26			// 0x26-0x27

#define EP957_V_Registers					0x28			// SHA 0x28-0x3B

#define EP957_KSV_FIFO						0x3C			// FIFO 0x3C-0x63

#define EP957_CBUS_MSC_Dec_Capability		0xA0
#define EP957_CBUS_MSC_Dec_SrcPad			0xB0
#define EP957_CBUS_MSC_Dec_Interrupt		0xC0
#define EP957_CBUS_MSC_Dec_Status			0xC4
#define EP957_CBUS_MSC_RAP_RCP				0xC8

#define EP957_CBUS_MSC_Interrupt			0xCA
#define EP957_CBUS_MSC_Interrupt__RQ_T1_EN			0x80	// New added in version D
#define EP957_CBUS_MSC_Interrupt__RTY_T1_EN			0x40	// New added in version D
#define EP957_CBUS_MSC_Interrupt__HPD_IE			0x08	// Moved added in version D
#define EP957_CBUS_MSC_Interrupt__MSG_IE			0x04	// Moved added in version D
#define EP957_CBUS_MSC_Interrupt__SCR_IE			0x02	// Moved added in version D
#define EP957_CBUS_MSC_Interrupt__INT_IE			0x01	// Moved added in version D
#define EP957_CBUS_MSC_Interrupt__HPD_S				0x08
#define EP957_CBUS_MSC_Interrupt__MSG_F				0x04
#define EP957_CBUS_MSC_Interrupt__SCR_F				0x02
#define EP957_CBUS_MSC_Interrupt__INT_F				0x01

#define EP957_CBUS_RQ_Control				0xCB
#define EP957_CBUS_RQ_Control__RQ_DONE				0x80
#define EP957_CBUS_RQ_Control__RQ_ERR				0x40
#define EP957_CBUS_RQ_Control__CBUS_STATE			0x20 	// New added in version D
#define EP957_CBUS_RQ_Control__WB_SPT				0x10	// New added in version D
#define EP957_CBUS_RQ_Control__RQ_RI_EN				0x08
#define EP957_CBUS_RQ_Control__CBUS_TRI				0x04
#define EP957_CBUS_RQ_Control__RQ_ABORT				0x02
#define EP957_CBUS_RQ_Control__RQ_START				0x01

#define EP957_CBUS_RQ_SIZE					0xCC
#define EP957_CBUS_RQ_SIZE__RX_SIZE					0x60
#define EP957_CBUS_RQ_SIZE__TX_SIZE					0x1F
#define EP957_CBUS_RQ_HEADER				0xCD
#define EP957_CBUS_RQ_HEADER__DDC_Packet			0x00
#define EP957_CBUS_RQ_HEADER__VS_Packet				0x02
#define EP957_CBUS_RQ_HEADER__MSC_Packet			0x04
#define EP957_CBUS_RQ_HEADER__isCommand				0x01
#define EP957_CBUS_RQ_HEADER__TD0_isCommand			0x08
#define EP957_CBUS_RQ_HEADER__TDn_isCommand			0x10
#define EP957_CBUS_RQ_CMD					0xCE
#define EP957_CBUS_RQ_TD					0xCF

#define EP957_CBUS_RQ_ACT_RX_SIZE			0xE1
#define EP957_CBUS_RQ_RD					0xE2

#define EP957_CBUS_Connection				0xE6
#define EP957_CBUS_Connection__CONNECTED			0x80
#define EP957_CBUS_Connection__CON_DONE				0x40
#define EP957_CBUS_Connection__ZM_RDY				0x20
#define EP957_CBUS_Connection__ZM_IE				0x10	// The read of this bit is fixed in version D
#define EP957_CBUS_Connection__ZM_F					0x08
#define EP957_CBUS_Connection__CON_BREAK			0x04
#define EP957_CBUS_Connection__CON_START			0x02
#define EP957_CBUS_Connection__ZM_EN				0x01

#define EP957_CBUS_Vendor_ID				0xE4
#define EP957_CBUS_BR_ADJ					0xE5

#define EP957_CBUS_TX_Re_Try				0xE7 			// New added in version D

#define EP957_CBUS_Time_Out					0xE8 			// New added in version D

#define EP957_EDID_Data_Addr	 			0xFE			// 1+256 Byte

#endif