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

  File        :  EP957_if.h 

  Description :  Head file of EP957 Interface  

\******************************************************************************/

#ifndef EP957_IF_H
#define EP957_IF_H

#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/syscalls.h> 
#include <linux/fcntl.h> 
#include <linux/regulator/consumer.h>

#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/spinlock.h>

#include "EP957RegDef.h"

#include <linux/string.h>

//==================================================================================================
//
// Protected Data Member
//

#if 1
#define EP_LOG_FUNCTION_NAME_ENTRY	printk(KERN_INFO		"[MHL] ## %s() ++ ##\n",  __func__);
#define EP_LOG_FUNCTION_NAME_EXIT	printk(KERN_INFO		"[MHL] ## %s() -- ##\n",  __func__);
#define EP_DEV_DBG_ERROR(fmt, args...)	printk(KERN_INFO		"[MHL] Exception [%s][%d] :: "fmt, __func__, __LINE__, ##args);
#define EP_DEV_DBG(fmt, args...) 			printk(KERN_DEBUG	"[MHL] [%s] :: " fmt, __func__, ##args);
#else
#define EP_LOG_FUNCTION_NAME_ENTRY 
#define EP_LOG_FUNCTION_NAME_EXIT  
#define EP_DEV_DBG_ERROR(fmt, args...)
#define EP_DEV_DBG(fmt, args...)
#endif


//
// Type Definition
//
#define FALSE 0
#define TRUE  1

#define STATUS_DISCONNECT 0
#define STATUS_CONNECT 1

#define MHL_DRIVER_MINOR_MAX   1
#define MHL_DRIVER_NAME "EP957_Driver"

#define MHL_DEBUG_FLAG 1

typedef char BOOL;
typedef unsigned char BYTE;
typedef BYTE *PBYTE;
typedef unsigned int WORD;
typedef WORD *PWORD;
typedef unsigned long DWORD;
typedef DWORD *PDWORD;

// Command and Data for the function 

// MHL_MSC_Cmd_READ_DEVICE_CAP(BYTE offset, PBYTE pValue)
#define MSC_DEV_CAT					0X02
#define 	POW								0x10
#define		DEV_TYPE						0x0F

#define ADOPTER_ID_H				0x03
#define ADOPTER_ID_L				0x04

#define LOG_DEV_MAP					0x08
#define 	LD_DISPLAY						0X01
#define 	LD_VIDEO						0X02
#define 	LD_AUDIO						0X04
#define 	LD_MEDIA						0X08
#define 	LD_TUNER						0X10
#define 	LD_RECORD						0X20
#define 	LD_SPEAKER						0X40
#define 	LD_GUI							0X80

#define FEATURE_FLAG				0X0A
#define 	RCP_SUPPORT						0X01
#define 	RAP_SUPPORT						0X02
#define 	SP_SUPPORT						0X04

#define DEVICE_ID_H					0x0B
#define DEVICE_ID_L					0x0C

#define SCRATCHPAD_SIZE				0X0D

// MHL_MSC_Reg_Read(BYTE offset)
// MHL_MSC_Cmd_WRITE_STATE(BYTE offset, BYTE value)
#define MSC_RCHANGE_INT				0X20
#define 	DCAP_CHG						0x01
#define 	DSCR_CHG						0x02
#define 	REQ_WRT							0x04
#define 	GRT_WRT							0x08

#define MSC_DCHANGE_INT				0X21
#define 	EDID_CHG						0x02

#define MSC_STATUS_CONNECTED_RDY	0X30
#define 	DCAP_RDY						0x01

#define MSC_STATUS_LINK_MODE		0X31
#define 	CLK_MODE						0x07
#define 	CLK_MODE__Normal						0x03
#define 	CLK_MODE__PacketPixel					0x02
#define 	PATH_EN							0x08

#define MSC_SCRATCHPAD				0x40


//MHL_MSC_Cmd_MSC_MSG(BYTE SubCmd, BYTE ActionCode)
#define MSC_RAP						0x20
#define		RAP_POLL						0x00
#define		RAP_CONTENT_ON					0x10
#define		RAP_CONTENT_OFF					0x11

#define MSC_RAPK					0x21
#define		RAPK_No_Error					0x00
#define		RAPK_Unrecognized_Code			0x01
#define		RAPK_Unsupported_Code			0x02
#define		RAPK_Responder_Busy				0x03

#define MSC_RCP						0x10
#define 	RCP_Select						0x00
#define 	RCP_Up							0x01
#define 	RCP_Down						0x02
#define 	RCP_Left						0x03
#define 	RCP_Right						0x04
#define 	RCP_Right_Up					0x05
#define 	RCP_Right_Down					0x06
#define 	RCP_Left_Up						0x07
#define 	RCP_Left_Down					0x08
#define 	RCP_Root_Menu					0x09
#define 	RCP_Setup_Menu					0x0A
#define 	RCP_Contents_Menu				0x0B
#define 	RCP_Favorite_Menu				0x0C
#define 	RCP_Exit						0x0D

#define 	RCP_Numeric_0					0x20
#define 	RCP_Numeric_1					0x21
#define 	RCP_Numeric_2					0x22
#define 	RCP_Numeric_3					0x23
#define 	RCP_Numeric_4					0x24
#define 	RCP_Numeric_5					0x25
#define 	RCP_Numeric_6					0x26
#define 	RCP_Numeric_7					0x27
#define 	RCP_Numeric_8					0x28
#define 	RCP_Numeric_9					0x29
#define 	RCP_Dot							0x2A
#define 	RCP_Enter						0x2B
#define 	RCP_Clear						0x2C

#define 	RCP_Channel_Up					0x30
#define 	RCP_Channel_Down				0x31
#define 	RCP_Previous_Channel			0x32

#define 	RCP_Volume_Up					0x41
#define 	RCP_Volume_Down					0x42
#define 	RCP_Mute						0x43
#define 	RCP_Play						0x44
#define 	RCP_Stop						0x45
#define 	RCP_Pause						0x46
#define 	RCP_Record						0x47
#define 	RCP_Rewind						0x48
#define 	RCP_Fast_Forward				0x49
#define 	RCP_Eject						0x4A
#define 	RCP_Forward						0x4B
#define 	RCP_Backward					0x4C

#define 	RCP_Angle						0x50
#define 	RCP_Subpicture					0x51

#define 	RCP_Play_Function				0x60
#define 	RCP_Pause_Play_Function			0x61
#define 	RCP_Record_Function				0x62
#define 	RCP_Pause_Record_Function		0x63
#define 	RCP_Stop_Function				0x64
#define 	RCP_Mute_Function				0x65
#define 	RCP_Restore_Volume_Function		0x66
#define 	RCP_Tune_Function				0x67
#define 	RCP_Select_Media_Function		0x68

#define 	RCP_F1							0x71
#define 	RCP_F2							0x72
#define 	RCP_F3							0x73
#define 	RCP_F4							0x74

#define 	RCP_F5							0x75
#define 	RCP_Vendor_Specific				0x7E

#define MSC_RCPK					0x11
#define MSC_RCPE					0x12
#define 	RCPK_No_Error					0x00
#define 	RCPK_Ineffective_Code			0x01
#define 	RCPK_Responder_Busy				0x02

#define MSC_UCP						0x30
#define MSC_UCPK					0x31
#define MSC_UCPE					0x32
#define 	UCPK_No_Error					0x00
#define 	UCPK_Ineffective_Code			0x01

// Return value for the function
// MHL_Tx_Connection_Status();
typedef enum {
	MHL_CBUS_CONNECTING = 0,	// CBUS is under connecting.
	MHL_NOT_CONNECTED, 			// CBUS connect fail.
	MHL_CBUS_CONNECTED, 		// CBUS is connected.
	MHL_HOTPLUG_DETECT	 		// Hot-Plug signal is detected. 
} MHL_CONNECTION_STATUS;

extern BYTE Device_Capability_Default[];

typedef enum {
	// Master
	SMBUS_STATUS_Success = 0x00,
	SMBUS_STATUS_Pending,//	SMBUS_STATUS_Abort,
	SMBUS_STATUS_NoAct = 0x02,
	SMBUS_STATUS_TimeOut,
	SMBUS_STATUS_ArbitrationLoss = 0x04
} SMBUS_STATUS;

typedef enum {
	SMBUS_Normal = 0,
	SMBUS_SkipStop,
	SMBUS_SkipStart,
	SMBUS_DataOnly		// 3 = SMBUS_SkipStop | SMBUS_SkipStart
} SMBUS_MODE;

#define MHL_EVENT_CONNECTED 	"MHLEVENT=connected"
#define MHL_EVENT_DISCONNECTED 	"MHLEVENT=disconnected"
#define MHL_EVENT_RECEIVED_RCP 	"MHLEVENT=received_RCP"
#define MHL_EVENT_1kOHM		"MHLEVENT=1kOHM_DET"
//==================================================================================================
//
// Public Functions
//

//--------------------------------------------------------------------------------------------------
//
// General
//

// All Interface Inital
extern void EP957_If_Reset(void);

//--------------------------------------------------------------------------------------------------
//
// MHL Transmiter Interface
//

// Common
extern void MHL_Tx_Power_Down(void);
extern void MHL_Tx_Power_Up(void);
extern BOOL MHL_Tx_RSEN(void);

// Link
extern void MHL_Tx_USB_Mode(BOOL USB_Mode);
extern BOOL MHL_Tx_VBUS_Power(void);
extern BOOL MHL_Tx_MEASURE_1KOHM(void);
extern BYTE MHL_MSC_Get_Flags(void);
extern void MHL_Tx_CBUS_Connect(void);
extern void MHL_Tx_CBUS_Disconnect(void);
extern MHL_CONNECTION_STATUS MHL_Tx_Connection_Status(void);
extern void MHL_Clock_Mode(BOOL Packed_Pixel_Mode);

// Read/Write Reg (CBus Responder)
extern void MHL_MSC_Reg_Update(BYTE offset, BYTE value); // Update Device Cap
extern BYTE MHL_MSC_Reg_Read(BYTE offset); // Read Status, Read Interrupt
extern void MHL_RCP_RAP_Read(PBYTE pData);

// Send Command (CBus Requester)
extern BOOL MHL_MSC_Cmd_READ_DEVICE_CAP(BYTE offset, PBYTE pValue);
extern BOOL MHL_MSC_Cmd_WRITE_STATE(BYTE offset, BYTE value); // same as the SET_INT command
extern BOOL MHL_MSC_Cmd_MSC_MSG(BYTE SubCmd, BYTE ActionCode);
extern BOOL MHL_MSC_Cmd_WRITE_BURST(BYTE offset, PBYTE pData, BYTE size);

// Protected Functions
extern void MHL_MSC_Cmd_ACK(void);
extern void MHL_MSC_Cmd_ABORT(void);

// Access DDC
SMBUS_STATUS MHL_Tx_DDC_Cmd(BYTE Addr, BYTE *pDatas, WORD Length, SMBUS_MODE Mode);


//--------------------------------------------------------------------------------------------------
//
// HDMI Receiver Interface
//

// Common
extern void HDMI_Rx_Power_Down(void);
extern void HDMI_Rx_Power_Up(void);
extern BOOL HDMI_Rx_Signal(void);

extern void HDMI_Rx_write_EDID(PBYTE pData);
extern void HDMI_Rx_HotPlug_Set(void);
extern void HDMI_Rx_HotPlug_Clear(void);

// HDCP
extern void HDMI_Rx_write_BKSV(PBYTE pBKSV);
extern void HDMI_Rx_write_BCAPS(BYTE BCaps);
extern BYTE HDMI_Rx_HDCP_Get_Flags(void);

extern void HDMI_Rx_read_AINFO(PBYTE pAINFO);
extern void HDMI_Rx_read_AN(PBYTE pAN);
extern void HDMI_Rx_read_AKSV(PBYTE pAKSV);

extern void HDMI_Rx_write_BSTATUS(PBYTE pBSTATUS);
extern void HDMI_Rx_write_FIFO(PBYTE pFIFO, BYTE Index);
extern void HDMI_Rx_write_V(PBYTE pV);

// Special
extern void HDMI_RQ_RI_Enable(BOOL Enable);


//--------------------------------------------------------------------------------------------------
//
// Hardware Interface (IIC Interface to access the EP957 registers)
//

// EP957 Register Interface
extern SMBUS_STATUS EP957_Reg_Read(BYTE ByteAddr, BYTE*  Data, WORD Size);
extern SMBUS_STATUS EP957_Reg_Write(BYTE ByteAddr, BYTE*  Data, WORD Size);
extern SMBUS_STATUS EP957_Reg_Set_Bit(BYTE ByteAddr, BYTE BitMask);
extern SMBUS_STATUS EP957_Reg_Clear_Bit(BYTE ByteAddr, BYTE BitMask);

//--------------------------------------------------------------------------------------------------
//
// Charging Interface
//
extern void MHL_set_Charge_Mode(BOOL Charge_Mode);

//==================================================================================================
//
// Protected Data Member
//

// EDID status error code
typedef enum {
	// Master
	EDID_STATUS_Success = 0x00,
	EDID_STATUS_Pending,//	SMBUS_STATUS_Abort,
	EDID_STATUS_NoAct = 0x02,
	EDID_STATUS_TimeOut,
	EDID_STATUS_ArbitrationLoss = 0x04,
	EDID_STATUS_ExtensionOverflow,
	EDID_STATUS_ChecksumError
} EDID_STATUS;


//==================================================================================================
//
// Public Functions
//

//--------------------------------------------------------------------------------------------------
//
// General
//

// All Interface Inital


//--------------------------------------------------------------------------------------------------
//
// Downstream HDCP Control Interface
//
BOOL Downstream_Rx_read_BKSV(BYTE* pBKSV);
BYTE Downstream_Rx_BCAPS(void);
void Downstream_Rx_write_AINFO(char ainfo);
SMBUS_STATUS Downstream_Rx_write_AN(BYTE* pAN);     // modify 130408
SMBUS_STATUS Downstream_Rx_write_AKSV(BYTE* pAKSV); // modify 130408
BOOL Downstream_Rx_read_RI(BYTE* pRI);
void Downstream_Rx_read_BSTATUS(BYTE* pBSTATUS);
void Downstream_Rx_read_SHA1_HASH(BYTE* pSHA);
BOOL Downstream_Rx_read_KSV_FIFO(BYTE* pBKSV, BYTE Index, BYTE DevCount);


//--------------------------------------------------------------------------------------------------
//
// Downstream EDID Control Interface
//

BOOL Downstream_Rx_poll_EDID(void);
EDID_STATUS Downstream_Rx_read_EDID(BYTE* pEDID);


//==================================================================================================
//
// 
// Public Functions
//

//--------------------------------------------------------------------------------------------------
//
// General
//

// All Interface Inital

//--------------------------------------------------------------------------------------------------
//
// Notify Status Interface
//

// Connect Status

extern void Notify_Connect_Status(BOOL is_connected);

#endif // EP957_IF_H


