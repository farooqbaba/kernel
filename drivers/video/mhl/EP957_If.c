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

  File        :  EP957_If.c

  Description :  EP957 IIC Interface. Revision D.

\******************************************************************************/

#include "EP957_If.h"
#include "EP957Controller.h"
#ifdef CONFIG_EP957B_MHL
#include <linux/usb/otg.h>
#endif/*CONFIG_EP957B_MHL*/

#include <linux/mfd/pm8xxx/pm8921-charger.h>
#include <linux/power_supply.h>
#include <linux/usb/msm_hsusb.h>

//--------------------------------------------------------------------------------------------------

//#define MSC_DBG
//#define DDC_DBG

//#define EP957_LINK_TIMER_TIME			55	// 100(msec)

#define MHL_CMD_RETRY_TIME		4
#define CBUS_TIME_OUT_CheckErr	0x50 // 0x01 = 1ms
#define CBUS_TIME_OUT_Normal	0x6E // default = 110ms

#define EVENT_STR_LEN_MAX		40

typedef enum {
	RQ_STATUS_Success = 0,
	RQ_STATUS_Abort,
	RQ_STATUS_Timeout,
	RQ_STATUS_Error
} RQ_STATUS;

//--------------------------------------------------------------------------------------------------

BYTE Device_Capability_Default[16] = 
{
	0x00, // 0: DEV_STATE		x
	0x12, // 1: MHL_VERSION		-
	0x02, // 2: DEV_CAT			- POW[4], DEV_TYPE[3:0]
// ADAPTOE_ID_H, L should be modified by customers
	0x04, // 3: ADOPTER_ID_H	- Explore ADOPTER ID is 263, then ADOPTER_ID_H = 0x01
	0x88, // 4: ADOPTER_ID_L	- Explore ADOPTER ID is 263, then ADOPTER_ID_L = 0x07.
	0x21, // 5: VID_LINK_MODE	- SUPP_VGA[5], SUPP_ISLANDS[4], SUPP_PPIXEL[3], SUPP_YCBCR422[2], SUPP_YCBCR444[1], SUPP_RGB444[0]
	0x03, // 6: AUD_LINK_MODE	- AUD_8CH[1], AUD_2CH[0]
	0x00, // 7: VIDEO_TYPE		x SUPP_VT[7], VT_GAME[3], VT_CINEMA[2], VT_PHOTO[1], VT_GRAPHICS[0]
	0x80, // 8: LOG_DEV_MAP		- LD_GUI[7], LD_SPEAKER[6], LD_RECORD[5], LD_TUNER[4], LD_MEDIA[3], LD_AUDIO[2], LD_VIDEO[1], LD_DISPLAY[0]
	0x0F, // 9: BANDWIDTH		x
// FEATURE_FLAG should be modified by customers	
	0x07, // A: FEATURE_FLAG	- UCP_RECV_SUPPORT[4], UCP_SEND_SUPPORT[3], SP_SUPPORT[2], RAP_SUPPORT[1], RCP_SUPPORT[0]
// DEVICE_ID_H, L should be modified by customers	
	0x95, // B: DEVICE_ID_H		-
	0x8B, // C: DEVICE_ID_L		-
	0x10, // D: SCRATCHPAD_SIZE	-
	0x33, // E: INT_STAT_SIZE	- STAT_SIZE[7:4], INT_SIZE[3:0]
	0x00, // F: Reserved
};

//--------------------------------------------------------------------------------------------------

#ifdef CONFIG_EP957B_MHL
static struct workqueue_struct         *initializeWorkQueue;
struct work_struct             workItem;
struct usb_mhl_event_callback mhlcb;
//static int vbus_mode = 0;
//static int vbus_cnt = 0;
//#define MAX_CNT	3
#endif/*CONFIG_EP957B_MHL*/

// Temp
static int i, j;
static SMBUS_STATUS status;
static BYTE Temp_Data[16];

static BOOL is_Connected_backup;

static BOOL RI_is_Enable = FALSE; // modify 130408 

static DEFINE_MUTEX(mhl_lock);

typedef enum {
	NO_CONNECT = 0,
	isUSB_MODE,
	isMHL_MODE
} CURRENTMODESTATUS;
static CURRENTMODESTATUS is_CurrentMode = NO_CONNECT;

BYTE Chip_Revision;

// Global date for HDMI Transmiter
BYTE Cache_EP957_Flags, Cache_EP957_MHL_Flags;


// Private Functions
SMBUS_STATUS EP957_Reg_Read(BYTE ByteAddr, BYTE*  Data, WORD Size);
SMBUS_STATUS EP957_Reg_Write(BYTE ByteAddr, BYTE*  Data, WORD Size);
SMBUS_STATUS EP957_Reg_Set_Bit(BYTE ByteAddr, BYTE BitMask);
SMBUS_STATUS EP957_Reg_Clear_Bit(BYTE ByteAddr, BYTE BitMask);

// CBus Requester
RQ_STATUS MHL_CBus_RQ_Go(void);
RQ_STATUS MHL_CBus_RQ_Go_DDC(void); // modify 130417 - add function for pass test read EDID
RQ_STATUS MHL_CBus_RQ_Check(BYTE Size, PBYTE pData);

// Android
void callback_work_Int(struct work_struct *p);
void callback_work_Link_Task(struct work_struct *p);	// modify 130218
void callback_work_PPG(struct work_struct *p);

// HiRes Timer
static struct hrtimer pTimer_Link_Task; // modify 130218
// Work Queue
struct work_struct work_Int;
struct work_struct work_Link_Task; // modify 130218
struct delayed_work work_PPG;

void Send_RCP_Key_Code(BYTE rcp_key_code);
void Send_Key_Event(unsigned int key_event);
inline int Init_Keyboard(void);
void Send_Kobject_Uevent(char str[EVENT_STR_LEN_MAX]);

// IIC Client
struct i2c_client *EP957_i2c_client = NULL;

// Link State
static CBUS_LINK_STATE Link_State = 0;

struct class *mhl_class;
struct device *mhl_dev;
static int32_t devMajor = 0;
static struct cdev mhlCdev;

static struct input_dev *dev_keyboard;

//==================================================================================================
//
// Charging Function Implementation
//
#define MAX_CURRENT 1500000

static int mhl_current_val = 0;
static bool mhl_vbus_active = 0;
static struct power_supply mhl_psy;
static struct usb_ext_notification mhl_info;
static int mhl_charge_mode = 0;

void (*mhl_notify_usb_online)(int online) = NULL;

static char *mhl_pm_power_supplied_to[] = {
	"usb",
};

static enum power_supply_property mhl_pm_power_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

atomic_t mhl_device_available = ATOMIC_INIT(1);

static int mhl_power_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	if(NULL == val) {
		return -EINVAL;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = mhl_current_val;
		EP_DEV_DBG("%s:get current max=%d",__func__,mhl_current_val);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		EP_DEV_DBG("%s:get vbus_active=%d",__func__,mhl_vbus_active);
		val->intval = mhl_vbus_active;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = mhl_vbus_active && mhl_charge_mode;
		EP_DEV_DBG("%s:get vbus_active=%d, mhl_mode=%d\n",__func__,mhl_vbus_active,mhl_charge_mode);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int mhl_power_set_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  const union power_supply_propval *val)
{
	if((NULL == psy) || (NULL == val)) {
		return -EINVAL;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		mhl_vbus_active = val->intval;
		if (mhl_vbus_active)
			mhl_current_val = MAX_CURRENT;
		else
			mhl_current_val = 0;

		EP_DEV_DBG("%s:set current max=%d",__func__,mhl_current_val);
		power_supply_changed(psy);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int mhl_device_discovery(void *data, int id,
			     void (*usb_notify_cb)(int online))
{

	if(usb_notify_cb) {
		mhl_notify_usb_online = usb_notify_cb;
	} else {
		EP_DEV_DBG_ERROR("%s:usb_notify_cb is NULL\n",__func__);
	}

	return 0;
}

//--------------------------------------------------------------------------------------------------
// Charging Interface
void MHL_set_Charge_Mode(BOOL Charge_Mode)
{
	if(mhl_notify_usb_online){
		if(Charge_Mode) {
			EP_DEV_DBG("###MHL_set_Charge_Mode on \n");
			mhl_charge_mode = 1;
			mhl_notify_usb_online(1);
		} else {
			EP_DEV_DBG("###MHL_set_Charge_Mode off \n");
			mhl_charge_mode = 0;
			mhl_notify_usb_online(0);
		}
	}
}


//==================================================================================================
//
// Public Function Implementation
//

//--------------------------------------------------------------------------------------------------
// Hardware Interface

void EP957_If_Reset(void)
{
	// Global date for HDMI Transmiter
EP_LOG_FUNCTION_NAME_ENTRY	
	Cache_EP957_Flags = 0x00;
	Cache_EP957_MHL_Flags = 0x00;
	
	// Set the CBUS Time-Out time
	Temp_Data[0] = CBUS_TIME_OUT_Normal;
	EP957_Reg_Write(EP957_CBUS_Time_Out, &Temp_Data[0], 1);
	
	// Check chip version
	EP957_Reg_Read(EP957_CBUS_Time_Out, &Temp_Data[0], 1);
	if(Temp_Data[0] == CBUS_TIME_OUT_Normal) {
		// Chip revision D
		EP_DEV_DBG("EP957 Revision D\n");
		Chip_Revision = 0x0D;
	}
	else {
		// Chip revision C
		EP_DEV_DBG("EP957 Revision C\n");
		Chip_Revision = 0x0C;
	}

	// Initial Settings
	EP957_Reg_Set_Bit(EP957_General_Control_1, EP957_General_Control_1__INT_OD | EP957_General_Control_1__OSC_PU);
	EP957_Reg_Clear_Bit(EP957_General_Control_1, EP957_General_Control_1__HPD);

	// Set the TX PHY Control 0
	Temp_Data[0] = 0x41; // 
	EP957_Reg_Write(EP957_TX_PHY_Control_0, &Temp_Data[0], 1);

	// Set the TX PHY Control 1
	Temp_Data[0] = 0x6F; // 
	EP957_Reg_Write(EP957_TX_PHY_Control_1, &Temp_Data[0], 1);

	// Set the RX PHY Control
	Temp_Data[0] = 0x0A; // 
	EP957_Reg_Write(EP957_RX_PHY_Control_0, &Temp_Data[0], 1);

	// Set Control Register 3
	Temp_Data[0] = 0xE3; // 
	EP957_Reg_Write(EP957_General_Control_3, &Temp_Data[0], 1);

	// Interrupt Enable
	Temp_Data[0] = EP957_General_Control_2__AKSV_IE; // EP957_General_Control_2__RSEN_IE | EP957_General_Control_2__AKSV_IE
	EP957_Reg_Write(EP957_General_Control_2, &Temp_Data[0], 1);
	Temp_Data[0] = EP957_CBUS_MSC_Interrupt__MSG_IE | EP957_CBUS_MSC_Interrupt__INT_IE | EP957_CBUS_MSC_Interrupt__HPD_IE; // modify 130205
	if(Chip_Revision == 0x0C) Temp_Data[0] <<= 4;
	EP957_Reg_Write(EP957_CBUS_MSC_Interrupt, &Temp_Data[0], 1);

	// Check SP_SUPPORT bit
	if(Device_Capability_Default[0x0A] | 0x04) {
		// Enable Write Burst support
		EP957_Reg_Set_Bit(EP957_CBUS_RQ_Control, EP957_CBUS_RQ_Control__WB_SPT); 
	}

	// Set the BR Adj
	//Temp_Data[0] = 50; // default 47, set to 50 for D version
#ifdef CONFIG_EP957B_MHL_A
	EP_DEV_DBG("CONFIG_EP957B_MHL_A\n");
	Temp_Data[0] = 42; // default 47, set to 42 for E version
#endif
#ifdef CONFIG_EP957B_MHL_B
	EP_DEV_DBG("CONFIG_EP957B_MHL_B\n");
	Temp_Data[0] = 42; // default 47, set to 42 for E version
#endif
	EP957_Reg_Write(EP957_CBUS_BR_ADJ, &Temp_Data[0], 1);

	// Set the CBUS Re-try time
	Temp_Data[0] = 0x20; 
	EP957_Reg_Write(EP957_CBUS_TX_Re_Try, &Temp_Data[0], 1);



	//
	// Set Default Device Capability
	//
	memcpy(Temp_Data, Device_Capability_Default, sizeof(Device_Capability_Default));
	EP957_Reg_Write(EP957_CBUS_MSC_Dec_Capability, Temp_Data, sizeof(Device_Capability_Default) );	
EP_LOG_FUNCTION_NAME_EXIT
}

void EP957_term_enable_exec(bool enable)
{
	BYTE mask = 0;
	BYTE o_val, n_val;

	mask = EP957_TX_PHY_Control_0__TERM1_DIS | EP957_TX_PHY_Control_0__TERM2_EN;
	EP957_Reg_Read(EP957_TX_PHY_Control_0, &o_val, 1);
	n_val = o_val & ~mask;

	if (enable) {
		n_val |= ~EP957_TX_PHY_Control_0__TERM1_DIS & EP957_TX_PHY_Control_0__TERM2_EN;
	} else {
		if (MHL_Tx_RSEN() && (EP957_Read_LINK_STATE() > CBUS_LINK_STATE__USB_Mode))
			n_val |= EP957_TX_PHY_Control_0__TERM1_DIS & ~EP957_TX_PHY_Control_0__TERM2_EN;
		else
			n_val = o_val;
	}
	if (o_val != n_val) {
		EP_DEV_DBG("EP957_TX_PHY_Control_0 = 0x%d\n", n_val);
		EP957_Reg_Write(EP957_TX_PHY_Control_0, &n_val, 1);
	}

	return;
}

//--------------------------------------------------------------------------------------------------
//
// MHL Transmiter (EP957-Tx Implementation)
//

void MHL_Tx_Power_Down(void)
{
	// Software power down
	EP957_Reg_Clear_Bit(EP957_General_Control_1, EP957_General_Control_1__TX_PU);	
	EP_DEV_DBG("Set TX Power Down\n");
}

void MHL_Tx_Power_Up(void)
{
	// Software power up
	EP957_Reg_Set_Bit(EP957_General_Control_1, EP957_General_Control_1__TX_PU);	
	EP_DEV_DBG("Set TX Power Up\n");	
}

BOOL MHL_Tx_RSEN(void)
{
	// RSEN Detect
	EP957_Reg_Read(EP957_General_Control_2, &Temp_Data[0], 1);
	Cache_EP957_Flags |= Temp_Data[0] & 0x03;

	return (Temp_Data[0] & EP957_General_Control_2__RSEN)? TRUE:FALSE;
}

void MHL_Tx_USB_Mode(BOOL USB_Mode)
{
	char status_str[EVENT_STR_LEN_MAX];
	CURRENTMODESTATUS new_CurrentMode;
	if(USB_Mode) {
		new_CurrentMode = isUSB_MODE;
	}
	else {
		new_CurrentMode = isMHL_MODE;
	}

	if(is_CurrentMode == new_CurrentMode) {
		return;
	}
	else {
		is_CurrentMode = new_CurrentMode;
	}

	if(USB_Mode) {
		// Enable the USB Mux to the USB mode
		EP957_Reg_Set_Bit(EP957_General_Control_3, EP957_General_Control_3__MUX_EN);
		EP957_Reg_Clear_Bit(EP957_General_Control_3, EP957_General_Control_3__OSC_Sel);
		EP957_Reg_Clear_Bit(EP957_General_Control_1, EP957_General_Control_1__OSC_PU);
		
		// Disable RSEN detect
		EP957_Reg_Set_Bit(EP957_TX_PHY_Control_1, EP957_TX_PHY_Control_1__RSEN_DIS);
		
	}
	else {
		// Disable the USB Mux to the MHL mode
		EP957_Reg_Clear_Bit(EP957_General_Control_3, EP957_General_Control_3__MUX_EN);
		EP957_Reg_Set_Bit(EP957_General_Control_3, EP957_General_Control_3__OSC_Sel);
		EP957_Reg_Set_Bit(EP957_General_Control_1, EP957_General_Control_1__OSC_PU);
		
		// Enable RSEN detect
		EP957_Reg_Clear_Bit(EP957_TX_PHY_Control_1, EP957_TX_PHY_Control_1__RSEN_DIS);
		//1kOHM detect
		strncpy(status_str,
			MHL_EVENT_1kOHM,
			EVENT_STR_LEN_MAX);
		Send_Kobject_Uevent(status_str);
	}
}

BOOL MHL_Tx_VBUS_Power(void)
{
	EP957_Reg_Read(EP957_General_Control_4, &Temp_Data[0], 1);
	return (Temp_Data[0] & EP957_General_Control_4__VBUS)? TRUE:FALSE;
	
}

BOOL MHL_Tx_MEASURE_1KOHM(void)
{
	BOOL Imp_Match = FALSE;
	
	EP957_Reg_Set_Bit(EP957_CBUS_Connection, EP957_CBUS_Connection__CON_BREAK); // Disconnect first

	EP957_Reg_Set_Bit(EP957_CBUS_Connection, EP957_CBUS_Connection__ZM_EN);

	EP957_Reg_Read(EP957_CBUS_Connection, &Temp_Data[0], 1);
	if(Temp_Data[0] & EP957_CBUS_Connection__ZM_RDY) {
		// 1K Ohm found
		Imp_Match = TRUE;
	}

	EP957_Reg_Clear_Bit(EP957_CBUS_Connection, EP957_CBUS_Connection__ZM_EN);

	return Imp_Match;
}

BYTE MHL_MSC_Get_Flags(void)
{
	EP957_Reg_Read(EP957_CBUS_MSC_Interrupt, &Temp_Data[0], 1);
	Temp_Data[0] |= Cache_EP957_MHL_Flags;
	Cache_EP957_MHL_Flags = 0;
	return Temp_Data[0];
}

void MHL_Tx_CBUS_Connect(void)
{
	EP957_Reg_Set_Bit(EP957_CBUS_Connection, EP957_CBUS_Connection__CON_BREAK); // Disconnect first
	EP957_Reg_Set_Bit(EP957_CBUS_Connection, EP957_CBUS_Connection__CON_START);
}

void MHL_Tx_CBUS_Disconnect(void)
{
	EP957_term_enable_exec(true);
	EP957_Reg_Set_Bit(EP957_CBUS_Connection, EP957_CBUS_Connection__CON_BREAK);
	MhlCharge(FALSE);
}

MHL_CONNECTION_STATUS MHL_Tx_Connection_Status(void)
{
	// CBUS Connections Status
	EP957_Reg_Read(EP957_CBUS_Connection, &Temp_Data[0], 1);

	if(Temp_Data[0] & EP957_CBUS_Connection__CON_DONE) {
		if(Temp_Data[0] & EP957_CBUS_Connection__CONNECTED) {

			// Hot-Plug Detect
			EP957_Reg_Read(EP957_CBUS_MSC_Interrupt, &Temp_Data[0], 1);
			Cache_EP957_MHL_Flags |= Temp_Data[0] & 0x07;
		
			if(Temp_Data[0] & EP957_CBUS_MSC_Interrupt__HPD_S) {
				return MHL_HOTPLUG_DETECT;
			}
			else {
				return MHL_CBUS_CONNECTED;
			}
		}
		return MHL_NOT_CONNECTED;
	}
	return MHL_CBUS_CONNECTING;
}

void MHL_Clock_Mode(BOOL Packed_Pixel_Mode)
{
	if(Packed_Pixel_Mode) {
		EP957_Reg_Set_Bit(EP957_General_Control_3, EP957_General_Control_3__PP_MODE);
		EP_DEV_DBG("Set Packed Pixel Mode On\n");
	}
	else {
		EP957_Reg_Clear_Bit(EP957_General_Control_3, EP957_General_Control_3__PP_MODE);
		EP_DEV_DBG("Set Packed Pixel Mode Off\n");
	}
}

void MHL_MSC_Reg_Update(BYTE offset, BYTE value)
{
	if(offset >= 0x40) {
		offset -= 0x40;
		// Scratchpad
		EP957_Reg_Write(EP957_CBUS_MSC_Dec_SrcPad + offset, &value, 1);
	}
	else if(offset >= 0x30) {
		offset -= 0x30;
		// Status Registers
		EP957_Reg_Write(EP957_CBUS_MSC_Dec_Status + offset, &value, 1);
	}
	else if(offset >= 0x20) {
		offset -= 0x20;
		// Interrupt Registers
		EP957_Reg_Write(EP957_CBUS_MSC_Dec_Interrupt + offset, &value, 1);
	}
	else {
		// Capability Registers
		EP957_Reg_Write(EP957_CBUS_MSC_Dec_Capability + offset, &value, 1);
	}
}

BYTE MHL_MSC_Reg_Read(BYTE offset)
{
	if(offset >= 0x40) {
		offset -= 0x40;
		// Scratchpad
		EP957_Reg_Read(EP957_CBUS_MSC_Dec_SrcPad + offset, &Temp_Data[0], 1);
	}
	else if(offset >= 0x30) {
		offset -= 0x30;
		// Status Registers
		EP957_Reg_Read(EP957_CBUS_MSC_Dec_Status + offset, &Temp_Data[0], 1);
	}
	else if(offset >= 0x20) {
		offset -= 0x20;
		// Interrupt Registers
		EP957_Reg_Read(EP957_CBUS_MSC_Dec_Interrupt + offset, &Temp_Data[0], 1);
	}
	else {
		// Capability Registers
		EP957_Reg_Read(EP957_CBUS_MSC_Dec_Capability + offset, &Temp_Data[0], 1);
	}
	return Temp_Data[0];
}

void MHL_RCP_RAP_Read(BYTE* pData)
{
	EP957_Reg_Read(EP957_CBUS_MSC_RAP_RCP, pData, 2);
}

BOOL MHL_MSC_Cmd_READ_DEVICE_CAP(BYTE offset, PBYTE pValue)
{
	RQ_STATUS error;
        int i;

	for(i=0; i<MHL_CMD_RETRY_TIME; ++i) {

		//
		// Fill in the Command
		//

		// Size (TX Size is not including the Header)
		Temp_Data[1] = 1 | (2<<5); // TX Size | (RX Size << 5)

		// Header
		Temp_Data[2] = EP957_CBUS_RQ_HEADER__MSC_Packet | EP957_CBUS_RQ_HEADER__isCommand;

		// Command
		Temp_Data[3] = 0x61; // MSC_READ_DEVCAP

		// Data
		Temp_Data[4] = offset; // offset

		EP957_Reg_Write(EP957_CBUS_RQ_SIZE, &Temp_Data[1], 4);
	
	
		//
		// Start to Send the Command
		//
	
		// Set the CBUS Time-Out time
		Temp_Data[0] = CBUS_TIME_OUT_CheckErr;
		EP957_Reg_Write(EP957_CBUS_Time_Out, &Temp_Data[0], 1);

		error = MHL_CBus_RQ_Go();
		
		// Set the CBUS Time-Out time
		Temp_Data[0] = CBUS_TIME_OUT_Normal;
		EP957_Reg_Write(EP957_CBUS_Time_Out, &Temp_Data[0], 1);

		//if(!error) {
			error = MHL_CBus_RQ_Check(1, pValue);
			if(!error) {
				if(i>0) EP_DEV_DBG("Retry %d READ_DEVICE_CAP Success\n", i);

				return TRUE;
			}
		//}	
	}

	EP_DEV_DBG("READ_DEVICE_CAP Fail, offset = 0x%X\n", offset);
	return FALSE;
}

BOOL MHL_MSC_Cmd_WRITE_STATE(BYTE offset, BYTE value)
{
	RQ_STATUS error;
        int i;

	for(i=0; i<MHL_CMD_RETRY_TIME; ++i) {

		//
		// Fill in the Command and Parameters
		//

		// Size (TX Size is not including the Header)
		Temp_Data[1] = 2 | (1<<5); // TX Size | (RX Size << 5)

		// Header
		Temp_Data[2] = EP957_CBUS_RQ_HEADER__MSC_Packet | EP957_CBUS_RQ_HEADER__isCommand;

		// Command
		Temp_Data[3] = 0x60; // MSC_WRITE_STATE

		// Data
		Temp_Data[4] = offset; // offset
		Temp_Data[5] = value; // value

		EP957_Reg_Write(EP957_CBUS_RQ_SIZE, &Temp_Data[1], 5);


		//
		// Start to Send the Command
		//

		error = MHL_CBus_RQ_Go();
		//if(!error) {
			error = MHL_CBus_RQ_Check(0, NULL);
			if(!error) {
				if(i>0) EP_DEV_DBG("Retry %d WRITE_STATE Success\n", i);

				return TRUE;
			}
		//}
	}
	EP_DEV_DBG("WRITE_STATE Fail\n");
	return FALSE;
}

BOOL MHL_MSC_Cmd_MSC_MSG(BYTE SubCmd, BYTE ActionCode)
{
	RQ_STATUS error;
        int i;

	for(i=0; i<MHL_CMD_RETRY_TIME; ++i) {

		//
		// Fill in the Command and Parameters
		//

		// Size (TX Size is not including the Header)
		Temp_Data[1] = 2 | (1<<5); // TX Size | (RX Size << 5)

		// Header
		Temp_Data[2] = EP957_CBUS_RQ_HEADER__MSC_Packet | EP957_CBUS_RQ_HEADER__isCommand;

		// Command
		Temp_Data[3] = 0x68; // MSC_MSG

		// Data
		Temp_Data[4] = SubCmd; // SubCmd
		Temp_Data[5] = ActionCode; // ActionCode

		EP957_Reg_Write(EP957_CBUS_RQ_SIZE, &Temp_Data[1], 5);


		//
		// Start to Send the Command
		//

		error = MHL_CBus_RQ_Go();
		//if(!error) {
			error = MHL_CBus_RQ_Check(0, NULL);
			if(!error) {
				if(i>0) EP_DEV_DBG("Retry %d MSC_MSG Success\n", i);

				return TRUE;
			}
		//}
	}
	EP_DEV_DBG("MSC_MSG Fail\n");
	return FALSE;
}

BOOL MHL_MSC_Cmd_WRITE_BURST(BYTE offset, PBYTE pData, BYTE size)
{
	RQ_STATUS error;
        int i;
	size = min((int)size, 16);

if(Chip_Revision == 0x0C) {

	// The code for Chip Revision C
	for(i=0; i<MHL_CMD_RETRY_TIME; ++i) {

		//
		// Fill in the Command and Parameters
		//

		// Size (TX Size is not including the Header)
		Temp_Data[0] = (1+size) | (0<<5); // TX Size | (RX Size << 5)

		// Header
		Temp_Data[1] = EP957_CBUS_RQ_HEADER__MSC_Packet | EP957_CBUS_RQ_HEADER__isCommand;

		// Command
		Temp_Data[2] = 0x6C; // MSC_WRITE_BURST

		// Data[0]
		Temp_Data[3] = offset; // offset
		
		EP957_Reg_Write(EP957_CBUS_RQ_SIZE, Temp_Data, 4);
		
		// Data[1] - [16]
		EP957_Reg_Write(EP957_CBUS_RQ_TD+1, pData, size);


		//
		// Start to Send the Command
		//

		error = MHL_CBus_RQ_Go();
		if(!error) {
		
			//
			// Fill in the Command and Parameters
			//
	
			// Size (TX Size is not including the Header)
			Temp_Data[0] = (0) | (1<<5); // TX Size | (RX Size << 5)
	
			// Header
			Temp_Data[1] = EP957_CBUS_RQ_HEADER__MSC_Packet | EP957_CBUS_RQ_HEADER__isCommand;
	
			// Command
			Temp_Data[2] = 0x32; // EOF
	
			EP957_Reg_Write(EP957_CBUS_RQ_SIZE, Temp_Data, 3);
	
	
			//
			// Start to Send the Command
			//
	
			error = MHL_CBus_RQ_Go();
			if(!error) {
				error = MHL_CBus_RQ_Check(0, NULL);
				if(!error) {
					if(i>0) EP_DEV_DBG("Retry %d WRITE_BURST Success\n", i);
					return TRUE;
				}
			}
		}
		EP_DEV_DBG("WRITE_BURST error\n");
	}
	
}
else {

	// The code for Chip Revision D
	
	for(i=0; i<MHL_CMD_RETRY_TIME; ++i) {

		//
		// Fill in the Command and Parameters
		//

		// Size (TX Size is not including the Header)
		Temp_Data[0] = (2+size) | (1<<5); // TX Size | (RX Size << 5)

		// Header
		Temp_Data[1] = EP957_CBUS_RQ_HEADER__MSC_Packet | EP957_CBUS_RQ_HEADER__isCommand | EP957_CBUS_RQ_HEADER__TDn_isCommand;

		// Command
		Temp_Data[2] = 0x6C; // MSC_WRITE_BURST
		
		// Data[0]
		Temp_Data[3] = offset; // offset
		
		EP957_Reg_Write(EP957_CBUS_RQ_SIZE, Temp_Data, 4);
		
		// Data[1] - [16]
		EP957_Reg_Write(EP957_CBUS_RQ_TD+1, pData, size);
		
		// EOF
		Temp_Data[0] = 0x32; // EOF
		EP957_Reg_Write(EP957_CBUS_RQ_TD+1+size, Temp_Data, 1);


		//
		// Start to Send the Command
		//

		error = MHL_CBus_RQ_Go();
		//if(!error) {
			error = MHL_CBus_RQ_Check(0, NULL);
			if(!error) {
				if(i>0) EP_DEV_DBG("Retry %d WRITE_BURST Success\n", i);
				return TRUE;
			}
		//}
	}
}

	EP_DEV_DBG("WRITE_BURST Fail\n");
	return FALSE;
}

//--------------------------------------------------------------------------------------------------
// MHL Protected(Internal) Commands

void MHL_MSC_Cmd_ABORT(void)
{
	//
	// Fill in the Command
	//

	// Size (TX Size is not including the Header)
	Temp_Data[1] = 0 | (0<<5); // TX Size | (RX Size << 5)

	// Header
	Temp_Data[2] = EP957_CBUS_RQ_HEADER__MSC_Packet | EP957_CBUS_RQ_HEADER__isCommand;

	// Command
	Temp_Data[3] = 0x35; // MSC_ABORT

	EP957_Reg_Write(EP957_CBUS_RQ_SIZE, &Temp_Data[1], 3);


	//
	// Start to Send the Command
	//

	MHL_CBus_RQ_Go();

	mdelay(110); // delay 100ms
}

void MHL_DDC_Cmd_ABORT(void)
{
	//
	// Fill in the Command
	//		   

	// Size (TX Size is not including the Header)
	Temp_Data[1] = 0 | (0<<5); // TX Size | (RX Size << 5)

	// Header
	Temp_Data[2] = EP957_CBUS_RQ_HEADER__DDC_Packet | EP957_CBUS_RQ_HEADER__isCommand;

	// Command
	Temp_Data[3] = 0x35; // DDC_ABORT

	EP957_Reg_Write(EP957_CBUS_RQ_SIZE, &Temp_Data[1], 3);


	//
	// Start to Send the Command
	//

	MHL_CBus_RQ_Go();
}

void MHL_DDC_Cmd_EOF(void)
{
	//
	// Fill in the Command
	//		   

	// Size (TX Size is not including the Header)
	Temp_Data[1] = 0 | (0<<5); // TX Size | (RX Size << 5)

	// Header
	Temp_Data[2] = EP957_CBUS_RQ_HEADER__DDC_Packet | EP957_CBUS_RQ_HEADER__isCommand;

	// Command
	Temp_Data[3] = 0x32; // DDC_EOF

	EP957_Reg_Write(EP957_CBUS_RQ_SIZE, &Temp_Data[1], 3);


	//
	// Start to Send the Command
	//

	MHL_CBus_RQ_Go();
}

//--------------------------------------------------------------------------------------------------
// MHL Access DDC

SMBUS_STATUS MHL_Tx_DDC_Cmd(BYTE Addr, BYTE *pDatas, WORD Length, SMBUS_MODE Mode)
{
	WORD i=0;
	SMBUS_STATUS status = SMBUS_STATUS_Success;
	RQ_STATUS error;
	
	// modify 130419 start
	BOOL is_READ_EDID;
	BYTE Check_Addr;

	Check_Addr = Addr & 0xFE;
	if( (Check_Addr == 0x60) || (Check_Addr == 0xA0) )
	{
		is_READ_EDID = TRUE;	
	}
	else
	{
		is_READ_EDID = FALSE;	
	}
	// modify 130419 end

	if(!(Mode & SMBUS_SkipStart)) {

		//
		// Fill in the Command and Parameters
		//

		// Size
		Temp_Data[1] = 1 | (1<<5); // TX Size | (RX Size << 5)
	
		// Header
		Temp_Data[2] = EP957_CBUS_RQ_HEADER__DDC_Packet | EP957_CBUS_RQ_HEADER__isCommand;
	
		// Command
		Temp_Data[3] = 0x30; // DDC SOF
	
		// Data
		Temp_Data[4] = Addr; // Addr
	
		EP957_Reg_Write(EP957_CBUS_RQ_SIZE, &Temp_Data[1], 4);


		//
		// Start to Send the Command
		//

	// modify 130419 start
		if(is_READ_EDID)
		{
			error = MHL_CBus_RQ_Go_DDC(); // modify 130417 - for pass test read EDID
		}
		else
		{
			error = MHL_CBus_RQ_Go(); 
		}
	// modify 130419 end

		//if(!error) {
		
			//
			// Check the read data
			//
		
			EP957_Reg_Read(EP957_CBUS_RQ_ACT_RX_SIZE, &Temp_Data[0], 2);
			
			if(!(Temp_Data[0] & 0x40)) { // CMD0 bit (shall be command but not)
#ifdef DDC_DBG
				EP_DEV_DBG("Err: CBUS DDC1 - CMD_BIT_ERR, 0x%0X\n", Temp_Data[0]);
#endif
				status = SMBUS_STATUS_ArbitrationLoss;
				MHL_DDC_Cmd_ABORT();
			}
			else if( (Temp_Data[0] & 0x03) != 1) { // ACT_RX_SIZE
#ifdef DDC_DBG
				EP_DEV_DBG("Err: CBUS DDC1 - RX_SIZE_ERR\n");	
#endif
				status = SMBUS_STATUS_ArbitrationLoss;
				MHL_DDC_Cmd_ABORT();
			}
			else if(Temp_Data[1] == 0x33) { // ACK
			}
			else if(Temp_Data[1] == 0x34) { // NACK
#ifdef DDC_DBG
				EP_DEV_DBG("Err: CBUS DDC1 - NO_ACK\n");	
#endif	
				status = SMBUS_STATUS_NoAct;
				MHL_DDC_Cmd_EOF();
			}
			else if(Temp_Data[1] == 0x35) { // ABORT
#ifdef DDC_DBG
				EP_DEV_DBG("Err: CBUS DDC1 - ABORT\n");	
#endif	
				Mode |= SMBUS_SkipStop;
				status = SMBUS_STATUS_Pending;
			}
			else {
				status = SMBUS_STATUS_ArbitrationLoss;
				MHL_DDC_Cmd_ABORT();
			}

		//}
		//else {
		//	status = SMBUS_STATUS_TimeOut;
		//}			  
	}

	if(status == SMBUS_STATUS_Success) {
		if(Addr & 0x01) {	// Read
//			for(i = 0; (i < Length) && (status == 0); ++i) { //modify 130314

				//
				// Fill in the Command and Parameters
				//
		
				// Size
				Temp_Data[1] = 0 | (1<<5);
			
				// Header
				Temp_Data[2] = EP957_CBUS_RQ_HEADER__DDC_Packet | EP957_CBUS_RQ_HEADER__isCommand;
			
				// Command
				Temp_Data[3] = 0x50; // DDC CONT
			
				EP957_Reg_Write(EP957_CBUS_RQ_SIZE, &Temp_Data[1], 3);
		
		  for(i = 0; (i < Length) && (status == 0); ++i) { //modify 130314
		
				//
				// Start to Send the Command
				//
				
				// modify 130419 start
					if(is_READ_EDID)
					{
						error = MHL_CBus_RQ_Go_DDC(); // modify 130417 - for pass test read EDID
					}
					else
					{
						error = MHL_CBus_RQ_Go(); 
					}
				// modify 130419 end
				
				//if(!error) {
				
					//
					// Check the read data
					//
				
					EP957_Reg_Read(EP957_CBUS_RQ_ACT_RX_SIZE, &Temp_Data[0], 2);

					if( (Temp_Data[0] & 0x03) != 1) { // ACT_RX_SIZE
#ifdef DDC_DBG
						EP_DEV_DBG("Err: CBUS DDC2 #%d - RX_SIZE_ERR\n", i);	
#endif	
						status = SMBUS_STATUS_ArbitrationLoss;
						MHL_DDC_Cmd_ABORT();
					}
					if(Temp_Data[0] & 0x40) { // CMD0 bit (shall NOT be command)
#ifdef DDC_DBG
						EP_DEV_DBG("Err: CBUS DDC2 #%d - CMD_BIT_ERR\n", i);
#endif
						if(Temp_Data[1] == 0x35) { // ABORT
							Mode |= SMBUS_SkipStop;
							status = SMBUS_STATUS_Pending;
						}
						else {
							status = SMBUS_STATUS_ArbitrationLoss;
							MHL_DDC_Cmd_ABORT();
						}
					}

					pDatas[i] = Temp_Data[1];
				//}
				//else {
				//	status = SMBUS_STATUS_TimeOut;
				//}							
			}
		}
		else {
			for(i = 0; (i < Length) && (status == 0); ++i) {
				
				//
				// Fill in the Command and Parameters
				//
		
				// Size
				Temp_Data[1] = 0 | (1<<5);
			
				// Header
				Temp_Data[2] = EP957_CBUS_RQ_HEADER__DDC_Packet;
			
				// Command
				Temp_Data[3] = pDatas[i]; // DDC OFFSET / DATA
			
				EP957_Reg_Write(EP957_CBUS_RQ_SIZE, &Temp_Data[1], 3);
		
		
				//
				// Start to Send the Command
				//
				
				// modify 130419 start
					if(is_READ_EDID)
					{
						error = MHL_CBus_RQ_Go_DDC(); // modify 130417 - for pass test read EDID
					}
					else
					{
						error = MHL_CBus_RQ_Go(); 
					}
				// modify 130419 end
				
				//if(!error) {
				
					//
					// Check the read data
					//
				
					EP957_Reg_Read(EP957_CBUS_RQ_ACT_RX_SIZE, &Temp_Data[0], 2);

					if(!(Temp_Data[0] & 0x40)) { // CMD0 bit (shall be command but not)
		#ifdef DDC_DBG
						EP_DEV_DBG("Err: CBUS DDC1 - CMD_BIT_ERR, 0x%0X\n", Temp_Data[0]);
		#endif
						status = SMBUS_STATUS_ArbitrationLoss;
						MHL_DDC_Cmd_ABORT();
					}
					else if( (Temp_Data[0] & 0x03) != 1) { // ACT_RX_SIZE
		#ifdef DDC_DBG
						EP_DEV_DBG("Err: CBUS DDC1 - RX_SIZE_ERR\n");	
		#endif
						status = SMBUS_STATUS_ArbitrationLoss;
						MHL_DDC_Cmd_ABORT();
					}
					else if(Temp_Data[1] == 0x33) { // ACK
					}
					else if(Temp_Data[1] == 0x34) { // NACK
		#ifdef DDC_DBG
						EP_DEV_DBG("Err: CBUS DDC1 - NO_ACK\n");	
		#endif	
						status = SMBUS_STATUS_NoAct;
						MHL_DDC_Cmd_EOF();
					}
					else if(Temp_Data[1] == 0x35) { // ABORT
		#ifdef DDC_DBG
						EP_DEV_DBG("Err: CBUS DDC1 - ABORT\n");	
		#endif	
						Mode |= SMBUS_SkipStop;
						status = SMBUS_STATUS_Pending;
					}
					else {
						status = SMBUS_STATUS_ArbitrationLoss;
						MHL_DDC_Cmd_ABORT();
					}

				//}
				//else {
				//	status = SMBUS_STATUS_TimeOut;
				//}							
			}
		}
	}
	
	if(!(Mode & SMBUS_SkipStop)) {
	
// EP957 has a speed up function to send two command at once.

		//
		// Fill in the Command and Parameters
		//

		// Size
		Temp_Data[1] = 1 | (0<<5);
	
		// Header
		Temp_Data[2] = EP957_CBUS_RQ_HEADER__DDC_Packet | EP957_CBUS_RQ_HEADER__isCommand | EP957_CBUS_RQ_HEADER__TD0_isCommand;
	
		// Command
		Temp_Data[3] = 0x51; // DDC STOP
		Temp_Data[4] = 0x32; // DDC EOF
	
		EP957_Reg_Write(EP957_CBUS_RQ_SIZE, &Temp_Data[1], 4);


		//
		// Start to Send the Command
		//
		
		// modify 130419 start
			if(is_READ_EDID)
			{
				error = MHL_CBus_RQ_Go_DDC(); // modify 130417 - for pass test read EDID
			}
			else
			{
				error = MHL_CBus_RQ_Go(); 
			}
		// modify 130419 end
		
		//if(error) {
		//	status = SMBUS_STATUS_TimeOut;
		//}
	}

	return status;
}

//--------------------------------------------------------------------------------------------------
//
// HDMI Receiver (EP957-Rx Implementation)
//

void HDMI_Rx_Power_Down(void)
{
	// Software power down
	EP957_Reg_Clear_Bit(EP957_General_Control_1, EP957_General_Control_1__RX_PU);	
	EP_DEV_DBG("Set RX Power Down\n");
}

void HDMI_Rx_Power_Up(void)
{
	// Software power up
	EP957_Reg_Set_Bit(EP957_General_Control_1, EP957_General_Control_1__RX_PU);	
	EP_DEV_DBG("Set RX Power Up\n");	
}

BOOL HDMI_Rx_Signal(void)
{
	EP957_Reg_Read(EP957_General_Control_4, &Temp_Data[0], 1);
	if(Temp_Data[0] & EP957_General_Control_4__LINK_ON) {
		if(Temp_Data[0] & EP957_General_Control_4__DE_VALID) {
			return TRUE;
		}
	}
	return FALSE;
}

void HDMI_Rx_write_EDID(PBYTE pData)
{	
	int i;

	for(i=0; i<256; i+=16) {
		Temp_Data[0] = i; // start address
		EP957_Reg_Write(EP957_EDID_Data_Addr, Temp_Data, 1);
		EP957_Reg_Write(EP957_EDID_Data_Addr+1, &pData[i], 16);
	}
	EP_DEV_DBG("Write EDID to HDMI Rx\n");
}

void HDMI_Rx_HotPlug_Set(void)
{
	EP957_Reg_Set_Bit(EP957_General_Control_1, EP957_General_Control_1__HPD | EP957_General_Control_1__EDID_EN);
	EP_DEV_DBG("Set HotPlug to HDMI Rx\n");
}

void HDMI_Rx_HotPlug_Clear(void)
{
	EP957_Reg_Clear_Bit(EP957_General_Control_1, EP957_General_Control_1__HPD | EP957_General_Control_1__EDID_EN);
	EP_DEV_DBG("Clear HotPlug to HDMI Rx\n");
}

//------------------------------------
// HDCP Propagation

void HDMI_Rx_write_BKSV(PBYTE pBKSV)
{
	EP957_Reg_Write(EP957_BKSV, pBKSV, 5);
	EP_DEV_DBG("Write BKSV to HDMI Rx\n");
}

void HDMI_Rx_write_BCAPS(BYTE BCaps)
{
	EP957_Reg_Write(EP957_BCAP, &BCaps, 1);
	EP_DEV_DBG("Write BCAPS to HDMI Rx\n");
}

BYTE HDMI_Rx_HDCP_Get_Flags(void)
{
	EP957_Reg_Read(EP957_General_Control_2, &Temp_Data[0], 1);
	Temp_Data[0] |= Cache_EP957_Flags;
	Cache_EP957_Flags = 0;
	return Temp_Data[0];
}

void HDMI_Rx_read_AINFO(PBYTE pAINFO)
{
	EP957_Reg_Read(EP957_AINFO, pAINFO, 1);
	EP_DEV_DBG("Read AINFO from HDMI Rx\n");
}

void HDMI_Rx_read_AN(PBYTE pAN)
{
	EP957_Reg_Read(EP957_AN, pAN, 8);
	EP_DEV_DBG("Read AN from HDMI Rx\n");
}

void HDMI_Rx_read_AKSV(BYTE *pAKSV)
{
	EP957_Reg_Read(EP957_AKSV, pAKSV, 5);
	EP_DEV_DBG("Read AKSV from HDMI Rx\n");	
}

void HDMI_Rx_write_BSTATUS(PBYTE pBSTATUS)
{
	EP957_Reg_Write(EP957_BSTATUS, pBSTATUS, 2);
	EP_DEV_DBG("Write BSTATUS to HDMI Rx\n");	
}

void HDMI_Rx_write_FIFO(PBYTE pKSV, BYTE Index)
{
	EP957_Reg_Write(EP957_KSV_FIFO+(Index*5), pKSV, 5);
	EP_DEV_DBG("Write FIFO to HDMI Rx\n");	
}

void HDMI_Rx_write_V(PBYTE pV)
{
	EP957_Reg_Write(EP957_V_Registers, pV, 20);
	EP_DEV_DBG("Write V to HDMI Rx\n");	
}

//
// Special
//

void HDMI_RQ_RI_Enable(BOOL Enable)
{
	// modify 130408 start
	if((!RI_is_Enable) && Enable) {
		RI_is_Enable = TRUE;			   
		
		EP957_Reg_Set_Bit(EP957_CBUS_RQ_Control, EP957_CBUS_RQ_Control__RQ_RI_EN);
	}
	if(RI_is_Enable && (!Enable)) {
		RI_is_Enable = FALSE;
		
		EP957_Reg_Clear_Bit(EP957_CBUS_RQ_Control, EP957_CBUS_RQ_Control__RQ_RI_EN);
		
		MHL_DDC_Cmd_ABORT();
	}
	// modify 130408 end
}


//--------------------------------------------------------------------------------------------------
//
// Hardware Interface
//

SMBUS_STATUS EP957_Reg_Read(BYTE RegAddr, BYTE*  Data, WORD Size)
{
	int nbyte=0;
	nbyte = i2c_smbus_read_i2c_block_data(EP957_i2c_client, RegAddr, Size, Data);
	if(nbyte<0) EP_DEV_DBG( "less than Size. read bytes : %d", nbyte);
	return 0; // Success
}

SMBUS_STATUS EP957_Reg_Write(BYTE RegAddr, BYTE*  Data, WORD Size)
{
	int result;
	result = i2c_smbus_write_i2c_block_data(EP957_i2c_client, RegAddr, Size, Data);
	if(result!=0) EP_DEV_DBG("write result = %d", result);
	return 0;
}

SMBUS_STATUS EP957_Reg_Set_Bit(BYTE RegAddr, BYTE BitMask)
{
	BYTE reg_data_tmp;

	EP957_Reg_Read(RegAddr, &reg_data_tmp, 1);

	// Write back to Reg Reg_Addr
	reg_data_tmp |= BitMask;
	
	return EP957_Reg_Write(RegAddr, &reg_data_tmp, 1);
}

SMBUS_STATUS EP957_Reg_Clear_Bit(BYTE RegAddr, BYTE BitMask)
{
	BYTE reg_data_tmp;

	EP957_Reg_Read(RegAddr, &reg_data_tmp, 1);

	// Write back to Reg Reg_Addr
	reg_data_tmp &= ~BitMask;
	
	return EP957_Reg_Write(RegAddr, &reg_data_tmp, 1);
}


//==================================================================================================
//
// Private Functions
//

RQ_STATUS MHL_CBus_RQ_Go(void)
{
	int i;
	BYTE RQ_Check;

	// Set the CBUS Re-try time
	Temp_Data[0] = 0x00; 
	EP957_Reg_Write(EP957_CBUS_TX_Re_Try, &Temp_Data[0], 1);


	//
	// Start to Send the Command
	//

	EP957_Reg_Set_Bit(EP957_CBUS_RQ_Control, EP957_CBUS_RQ_Control__RQ_START);


	//
	// Wait for Complete
	//

	// When IIC clock speed = 170KHz => 400 round = 100ms
	// When IIC clock speed = 200KHz => 470 round = 100ms
	// When IIC clock speed = 400KHz => 940 round = 100ms
	for(i=0; i<940; i++) { // > 100ms timeout
		//delay5us();
		EP957_Reg_Read(EP957_CBUS_RQ_Control, &RQ_Check, 1);
		if(!(RQ_Check & EP957_CBUS_RQ_Control__RQ_START)) {
			if(RQ_Check & EP957_CBUS_RQ_Control__RQ_DONE) {
				break;
			}
		}
	}
	
	// Set the CBUS Re-try time
	Temp_Data[0] = 0x20; 
	EP957_Reg_Write(EP957_CBUS_TX_Re_Try, &Temp_Data[0], 1);


	// Check Error
	if(RQ_Check & EP957_CBUS_RQ_Control__RQ_DONE) {
		if(!(RQ_Check & EP957_CBUS_RQ_Control__RQ_ERR)) {
			return RQ_STATUS_Success; // No error
		}
	}
	else {
		EP957_Reg_Set_Bit(EP957_CBUS_RQ_Control, EP957_CBUS_RQ_Control__RQ_ABORT);
//			EP_DEV_DBG("Err: CBUS RQ Start - RQ_Timeout\n");
		return RQ_STATUS_Timeout;
	}

//	EP_DEV_DBG("Err: CBUS RQ Start - RQ_ERR\n");
	return RQ_STATUS_Error;
}

// modify 130417 start
RQ_STATUS MHL_CBus_RQ_Go_DDC(void) // for pass test read EDID
{
	int i;
	BYTE RQ_Check;

	// Start to Send the Command
	
	Temp_Data[0] = 0x01; 
	EP957_Reg_Write(EP957_CBUS_RQ_Control, &Temp_Data[0], 1);

	// Wait for Complete

	for(i=0; i<940; i++) { // > 100ms timeout
		//delay5us();
		EP957_Reg_Read(EP957_CBUS_RQ_Control, &RQ_Check, 1);
		if(!(RQ_Check & EP957_CBUS_RQ_Control__RQ_START)) {
			if(RQ_Check & EP957_CBUS_RQ_Control__RQ_DONE) {
				break;
			}
		}
	}	

	// Check Error
	if(RQ_Check & EP957_CBUS_RQ_Control__RQ_DONE) {
		if(!(RQ_Check & EP957_CBUS_RQ_Control__RQ_ERR)) {
			return RQ_STATUS_Success; // No error
		}
	}
	else {
		EP957_Reg_Set_Bit(EP957_CBUS_RQ_Control, EP957_CBUS_RQ_Control__RQ_ABORT);
//			EP_DEV_DBG("Err: CBUS RQ Start - RQ_Timeout\n");
		return RQ_STATUS_Timeout;
	}

//	EP_DEV_DBG("Err: CBUS RQ Start - RQ_ERR\n");
	return RQ_STATUS_Error;
}
// modify 130417 end

RQ_STATUS MHL_CBus_RQ_Check(BYTE Size, BYTE* pData)
{
	RQ_STATUS error = RQ_STATUS_Success;

	//
	// Check the read data
	//

	EP957_Reg_Read(EP957_CBUS_RQ_ACT_RX_SIZE, &Temp_Data[0], Size + 2);	   

	if( (Temp_Data[0]&0x03) == 0 ) {
		error = RQ_STATUS_Error;
#ifdef MSC_DBG
		EP_DEV_DBG("Err: CBUS RQ - No Data Received\n");
#endif
	}
	else if(!(Temp_Data[0] & 0x40)) { // CMD0 bit (shall be command)
#ifdef MSC_DBG
		EP_DEV_DBG("Err: CBUS RQ - CMD_BIT0_ERR, 0x%X\n", Temp_Data[0]);
#endif
		error = RQ_STATUS_Error;
		MHL_MSC_Cmd_ABORT();
	}
	else {
		if( (Temp_Data[0]&0x03) != (Size + 1) ) { // ACT_RX_SIZE
#ifdef MSC_DBG
		EP_DEV_DBG("Err: CBUS RQ - RX_SIZE_ERR, 0x%X\n", Temp_Data[0]);
#endif
			error = RQ_STATUS_Error;
			mdelay(30); // delay 30ms
			MHL_MSC_Cmd_ABORT();					 
		}
	}
	if(!error) {
		if(Temp_Data[1] == 0x35) { // ABORT
#ifdef MSC_DBG
		EP_DEV_DBG("Err: CBUS RQ - ABORT, 0x%X\n", Temp_Data[1]);
#endif
		mdelay(2100); // Delay 2 second
			error = RQ_STATUS_Abort;
		}
		else if(Temp_Data[1] != 0x33) { // ACK
#ifdef MSC_DBG
		EP_DEV_DBG("Err: CBUS RQ - NOT_ACK, 0x%X\n", Temp_Data[1]);
#endif
			error = RQ_STATUS_Abort;
		}
	}
	if((Temp_Data[0] & 0x03) > 1) {
		if((Temp_Data[0] & 0x80)) { // CMD1 bit (shall not be command)
#ifdef MSC_DBG
			EP_DEV_DBG("Err: CBUS RQ - CMD_BIT1_ERR, 0x%0X\n", Temp_Data[0]);
#endif
			error = RQ_STATUS_Error;
			MHL_MSC_Cmd_ABORT();
		}
	}

	if(!error) { // if No error
		// Copy the data
		if(pData) memcpy(pData, &Temp_Data[2], Size);
	}

	return error;
}


//==================================================================================================
//
// DDC Implementation
//
//==================================================================================================


#define HDCP_RX_ADDR            0x74     // HDCP RX Address
#define EDID_ADDR       		0xA0     // EDID Address
#define EDID_SEGMENT_PTR		0x60

#define HDCP_RX_BKSV_ADDR       0x00     // HDCP RX, BKSV Register Address
#define HDCP_RX_RI_ADDR         0x08     // HDCP RX, RI Register Address
#define HDCP_RX_AKSV_ADDR       0x10     // HDCP RX, AKSV Register Address
#define HDCP_RX_AINFO_ADDR      0x15     // HDCP RX, AINFO Register Address
#define HDCP_RX_AN_ADDR         0x18     // HDCP RX, AN Register Address
#define HDCP_RX_SHA1_HASH_ADDR  0x20     // HDCP RX, SHA-1 Hash Value Start Address
#define HDCP_RX_BCAPS_ADDR      0x40     // HDCP RX, BCAPS Register Address
#define HDCP_RX_BSTATUS_ADDR    0x41     // HDCP RX, BSTATUS Register Address
#define HDCP_RX_KSV_FIFO_ADDR   0x43     // HDCP RX, KSV FIFO Start Address

#define EDID_READ_STEP	16

//--------------------------------------------------------------------------------------------------

// Temp Data
static int i, j;
static BYTE DDC_Data[128]; //modify 130314
static BYTE TempBit;

// Private Functions
SMBUS_STATUS DDC_Access(BYTE IICAddr, BYTE RegAddr, BYTE*  Data, WORD Size);
#define DDC_Write DDC_Access
SMBUS_STATUS DDC_Read(BYTE IICAddr, BYTE RegAddr, BYTE*  Data, WORD Size);
void DDC_Segment_Write(BYTE Segment);

//==================================================================================================
//
// Public Function Implementation
//

//--------------------------------------------------------------------------------------------------
// Hardware Interface

//--------------------------------------------------------------------------------------------------
//
// Downstream HDCP Control
//

BOOL Downstream_Rx_read_BKSV(BYTE *pBKSV)
{
	status = DDC_Read(HDCP_RX_ADDR, HDCP_RX_BKSV_ADDR, pBKSV, 5);
	if(status != SMBUS_STATUS_Success) {
		EP_DEV_DBG("Err: BKSV read - DN DDC %X\n", status);
		return FALSE;
	}

	i = 0;
	j = 0;
	while (i < 5) {
		TempBit = 1;
		while (TempBit) {
			if (pBKSV[i] & TempBit) j++;
			TempBit <<= 1;
		}
		i++;
	}
	if(j != 20) {
		EP_DEV_DBG("Err: BKSV read - Key Wrong\n");
		EP_DEV_DBG("Err: BKSV = 0x%X,0x%X,0x%X,0x%X,0x%X\n", pBKSV[0], pBKSV[1], pBKSV[2], pBKSV[3], pBKSV[4]);
		return FALSE;
	}
	return TRUE;
}

BYTE Downstream_Rx_BCAPS(void)
{
	DDC_Read(HDCP_RX_ADDR, HDCP_RX_BCAPS_ADDR, &DDC_Data[0], 1);
	return DDC_Data[0];
}

void Downstream_Rx_write_AINFO(char ainfo) 
{
	DDC_Write(HDCP_RX_ADDR, HDCP_RX_AINFO_ADDR, &ainfo, 1);
}

SMBUS_STATUS Downstream_Rx_write_AN(BYTE *pAN) // modify 130408  
{
	return DDC_Write(HDCP_RX_ADDR, HDCP_RX_AN_ADDR, pAN, 8); // modify 130408 (need return status)
}

SMBUS_STATUS Downstream_Rx_write_AKSV(BYTE *pAKSV) // modify 130408 
{
	return DDC_Write(HDCP_RX_ADDR, HDCP_RX_AKSV_ADDR, pAKSV, 5); //modify 130408 (need return status)
}

BOOL Downstream_Rx_read_RI(BYTE *pRI)
{
	// Not to use the Short Read
	status = DDC_Read(HDCP_RX_ADDR, HDCP_RX_RI_ADDR, pRI, 2);
	if(status != SMBUS_STATUS_Success) {
		EP_DEV_DBG("Err: Rx Ri read\n");
		return FALSE;
	}
	return TRUE;
}

void Downstream_Rx_read_BSTATUS(BYTE *pBSTATUS)
{
	DDC_Read(HDCP_RX_ADDR, HDCP_RX_BSTATUS_ADDR, pBSTATUS, 2);
}

void Downstream_Rx_read_SHA1_HASH(BYTE *pSHA) 
{
	DDC_Read(HDCP_RX_ADDR, HDCP_RX_SHA1_HASH_ADDR, pSHA, 20);
}

// Retrive 5 bytes KSV at "Index" from FIFO
BOOL Downstream_Rx_read_KSV_FIFO(BYTE *pBKSV, BYTE Index, BYTE DevCount)
{
	// Try not to re-read the previous KSV
	// But, when? to send STOP is can not be determined
	if(Index == 0) { // Start 
		DDC_Data[0] = HDCP_RX_KSV_FIFO_ADDR;
		MHL_Tx_DDC_Cmd(HDCP_RX_ADDR, &DDC_Data[0], 1, SMBUS_SkipStop);
		if(Index == DevCount-1) {
			MHL_Tx_DDC_Cmd(HDCP_RX_ADDR+1, pBKSV, 5, SMBUS_Normal); // Start also End
		}
		else {
			MHL_Tx_DDC_Cmd(HDCP_RX_ADDR+1, pBKSV, 5, SMBUS_SkipStop); // Start only
		}
	}
	else if(Index == DevCount-1) { // End
		MHL_Tx_DDC_Cmd(HDCP_RX_ADDR+1, pBKSV, 5, SMBUS_SkipStart);
	}
	else { // Mid
		MHL_Tx_DDC_Cmd(HDCP_RX_ADDR+1, pBKSV, 5, SMBUS_DataOnly);
	}

	if(status != SMBUS_STATUS_Success) {
		EP_DEV_DBG("Err: KSV FIFO read - DN DDC %X\n", status );
		return FALSE;
	}

	i = 0;
	j = 0;
	while (i < 5) {
		TempBit = 1;
		while (TempBit) {
			if (pBKSV[i] & TempBit) j++;
			TempBit <<= 1;
		}
		i++;
	}
	if(j != 20) {
		EP_DEV_DBG("Err: KSV FIFO read - Key Wrong\n");
		MHL_Tx_DDC_Cmd(HDCP_RX_ADDR+1, pBKSV, 0, SMBUS_SkipStart); // Stop
		return FALSE;
	}	
	return TRUE;
}


//--------------------------------------------------------------------------------------------------
//
// Downstream EDID Control
//

// Read the EDID test
BOOL Downstream_Rx_poll_EDID(void)
{
	// Segment Pointer Address
	DDC_Segment_Write(0);

	// Base Address and Read 1
	status = DDC_Read(EDID_ADDR, 0, DDC_Data, 1);
	if(status == SMBUS_STATUS_Success) {
		return TRUE;
	}
	return FALSE;
}

EDID_STATUS Downstream_Rx_read_EDID(BYTE *pEDID)
{
	BYTE seg_ptr, BlockCount, Block1Found, ChkSum;

// modify 130314 
// change note : 
// 1. doesn't use for loop to read 128 byte EDID, direct read 128 byte at once.
// 2. printf EDID for debug is move to after read EDID process done
	// =========================================================
	// I. Read the block 0

	// Segment Pointer Address
	DDC_Segment_Write(0); 

	// Read EDID data
	status = DDC_Read(EDID_ADDR, 0, &pEDID[0], 128);
	if(status != SMBUS_STATUS_Success) {
		EP_DEV_DBG("Err: EDID b0-%d read - DN DDC %d\n", i, status);
		return status;	
	}

	// Check EDID
	if(pEDID[126] > 8) {
		EP_DEV_DBG("warning: EDID Check failed, pEDID[126]=0x%X > 8\n\r", pEDID[126] );
		//return EDID_STATUS_ExtensionOverflow;
		pEDID[126] = 1;
	}

	// =========================================================
	// II. Read other blocks and find Timing Extension Block

	BlockCount = pEDID[126];
	Block1Found = 0;
	for (seg_ptr = 1; seg_ptr <= BlockCount; ++seg_ptr) {

		// Segment Pointer Address
		DDC_Segment_Write(seg_ptr >> 1); 
			
		// Base Address and Read 128
		if(Block1Found) {		
			// Read EDID data
			status = DDC_Read(EDID_ADDR, ((seg_ptr & 0x01) << 7), DDC_Data, 128);
			if(status != SMBUS_STATUS_Success) {
				EP_DEV_DBG("Err: EDID 0x%x-%d read\n", seg_ptr, i); 
				return status;
			}
		}
		else {	
			
			// Read EDID data
			status = DDC_Read(EDID_ADDR, ((seg_ptr & 0x01) << 7), &pEDID[128], 128);
			if(status != SMBUS_STATUS_Success) {
				EP_DEV_DBG("Err: EDID 0x%x-%d read\n", seg_ptr, i);
				return status;
			}
		}

		if(pEDID[128] == 0x02 && Block1Found == 0) {
			Block1Found = 1;
		}
	}
	

	pEDID[0x00] = 0x00; 
	pEDID[0x01] = 0xFF; 
	pEDID[0x02] = 0xFF; 
	pEDID[0x03] = 0xFF; 
	pEDID[0x04] = 0xFF; 
	pEDID[0x05] = 0xFF; 
	pEDID[0x06] = 0xFF; 
	pEDID[0x07] = 0x00; 	

	// Check CheckSum
	ChkSum = 0;
	for(i=0; i<((Block1Found)?256:128); ++i) {
		ChkSum += pEDID[i];
	}
	if(ChkSum != 0) {
		EP_DEV_DBG("warning: EDID Check failed, ChkSum=0x%X  \n\r", ChkSum );
		return EDID_STATUS_ChecksumError;
	}

	// calsulate CheckSum
	ChkSum = 0;
	for(i=0; i<127; ++i) 
	{
		ChkSum += pEDID[i];
	}
	pEDID[127] = ~(ChkSum - 1);
		
	// If have Block 1
	if(pEDID[126] == 0x01) 
	{
		pEDID[128+126] = 0x00;	// We can not have block2
					
		ChkSum = 0;
		for(i=0; i<127; ++i) 
		{
			ChkSum += pEDID[128+i];
		}
		pEDID[128+127] = ~(ChkSum - 1);
	}

	//
	if(Block1Found) {
		pEDID[126] = 1;
	}
	else {
		pEDID[126] = 0;
	}

	// printf EDID data for debug--------------
	EP_DEV_DBG("EDID b0 read:");
#if 0
	for(i=0; i<128; ++i) {
		if(i%16 == 0) printk("\n");		
		if(i%8 == 0) printk(" ");		
		printk("0x%02x,", pEDID[i] );
	}
#endif
			
	EP_DEV_DBG("EDID b1 read:");
#if 0
	for(i=0; i<128; ++i) {
		if(i%16 == 0) printk("\n"); 	
		if(i%8 == 0) printk(" ");		
		printk("0x%02x,", pEDID[128+i]);
	}
#endif
	//----------------------------------
	
	return EDID_STATUS_Success;
}

//==================================================================================================
//
// Private Functions
//

SMBUS_STATUS DDC_Access(BYTE IICAddr, BYTE RegAddr, BYTE*  Data, WORD Size)
{
	if(IICAddr & 0x01) { // Read
		status = MHL_Tx_DDC_Cmd(IICAddr-1, &RegAddr, 1, SMBUS_SkipStop);
		if(!status) { // modify 130408 (if communication error, need stop)
			status |= MHL_Tx_DDC_Cmd(IICAddr, Data, Size, SMBUS_Normal);
		} // modify 130408
	}
	else { // Write
		status = MHL_Tx_DDC_Cmd(IICAddr, &RegAddr, 1, SMBUS_SkipStop);
		if(!status) { // modify 130408 (if communication error, need stop)
			status |= MHL_Tx_DDC_Cmd(IICAddr, Data, Size, SMBUS_SkipStart);
		} // modify 130408
	}
	if(status) { // failed and retry
		EP_DEV_DBG("Err: DDC failed %X, IICAddr=0x%X, RegAddr=0x%X\n", status, IICAddr, RegAddr);
	
	}
	return status;
}

SMBUS_STATUS DDC_Read(BYTE IICAddr, BYTE RegAddr, BYTE*  Data, WORD Size)
{
	if(!(IICAddr & 0x01)) IICAddr |= 1;
	return DDC_Access(IICAddr, RegAddr, Data, Size);
}

void DDC_Segment_Write(BYTE Segment)
{
	SMBUS_STATUS status;
	status = MHL_Tx_DDC_Cmd(EDID_SEGMENT_PTR, &Segment, 0, SMBUS_SkipStop);
	if(!status) { // if no error
		MHL_Tx_DDC_Cmd(EDID_SEGMENT_PTR, &Segment, 1, SMBUS_DataOnly);
	}
}



//==================================================================================================
//
// Android Driver Implementation
//
//==================================================================================================


//------------------------------------------------------------------------------------------------
// Call Back and Interrupt Functions
//------------------------------------------------------------------------------------------------

static enum hrtimer_restart callback_timer(struct hrtimer *timer)
{
	schedule_work(&work_Link_Task); // modify 130218
	
	return HRTIMER_NORESTART;
}

irqreturn_t callback_irq(int irq, void *handle)
{
	schedule_work(&work_Int);
	return IRQ_HANDLED;
}

void callback_work_Link_Task(struct work_struct *p) // modify 130218
{
	if(EP957_i2c_client == NULL) return;
	
	cancel_delayed_work_sync(&work_PPG); // modify 130408 // prevent PPG Task run again when Link Task is running

	// EP957 Link Task to check the CBUS connection every 55 ms
	// Set a timer to run this task every 55 ms 

	mutex_lock(&mhl_lock);

	Link_State = EP957Control_Link_Task();

	if(Link_State == CBUS_LINK_STATE__USB_Mode) 
	{
		// USB Mode
		hrtimer_start(&pTimer_Link_Task, ktime_set(0, 1000*1000000), HRTIMER_MODE_REL); // 1 sec // modify 130218
	}
	else {
		hrtimer_start(&pTimer_Link_Task, ktime_set(0, 55*1000000), HRTIMER_MODE_REL); // 55 ms // modify 130218
	}

	if(Link_State == CBUS_LINK_STATE__Connected) 
	{
		///////////////////////////////////////////////////////////////
		// Start a thread to run EP957Control_Propagation_Task();
			schedule_delayed_work(&work_PPG, 0);
		///////////////////////////////////////////////////////////////
	}
	mutex_unlock(&mhl_lock);
}

void callback_work_PPG(struct work_struct *p)
{
	if(EP957_i2c_client == NULL) return;

	/////////////////////////////////////////////////////////////////
	// EP957 Propagation Task, run the task in a loop asap.
	// The Propagation Task will handle the EDID and HDCP propegation
	mutex_lock(&mhl_lock);
	{
		PPG_STATE PPG_State = EP957Control_Propagation_Task();

		if((PPG_State != PPGS_HDCP_Done) && (Link_State == CBUS_LINK_STATE__Connected) )
		{
			///////////////////////////////////////////////////////////////
			// Continue the thread that run EP957Control_Propagation_Task();
			///////////////////////////////////////////////////////////////

			//queue_delayed_work(MHL_wq, &work_PPG, usecs_to_jiffies(100));
			schedule_delayed_work(&work_PPG, usecs_to_jiffies(100));
		}
	}
	mutex_unlock(&mhl_lock);
	/////////////////////////////////////////////////////////////////
}

void callback_work_Int(struct work_struct *p)
{
	if(EP957_i2c_client == NULL) return;

	if (!atomic_read(&mhl_device_available)) {
		EP_DEV_DBG("Skip(%d).\n", gpio_get_value(59));
		return;
	}
	cancel_delayed_work_sync(&work_PPG); // modify 130408 // prevent PPG Task run again when PPG Task is running

	EP_DEV_DBG("ep957_interrupt_event_work() is called\n");
	
	/////////////////////////////////////////////////////////////////
	// Start a thread that run EP957Control_Propagation_Task() ASAP
	// if the Source will do the HDCP Authentication second time.
	if(Link_State == CBUS_LINK_STATE__Connected) 
	{
		///////////////////////////////////////////////////////////////
		// Start a thread to run EP957Control_Propagation_Task();
		if(work_pending(&work_Link_Task) == 0)  // modify 130408 // prevent PPG Task run when Link Task running
		{
			schedule_delayed_work(&work_PPG, 0);
		}
	}
	/////////////////////////////////////////////////////////////////
	
	/////////////////////////////////////////////////////////////////
	// Check the RCP code when Interrupt is detected.
	// Or polling every < 200ms is also available.
	mutex_lock(&mhl_lock);
	{
		BYTE RCP_RAP_Code[2];

		if(EP957Control_RCP_RAP_Read(RCP_RAP_Code) == TRUE) 
		{
			if(RCP_RAP_Code[0] == MSC_RAP)
			{
				//MHL_RAP_Action(RCP_RAP_Code[1]);
				EP_DEV_DBG("RAP Code =0x%x", RCP_RAP_Code[1]);
				//switch(RCP_RAP_Code[1]) 
				//{
				//	case RAP_CONTENT_ON:
				//		EP_DEV_DBG("RAP: Content On\n");
				//		break;
				//	case RAP_CONTENT_OFF:
				//		EP_DEV_DBG("RAP: Content Off\n");
				//		break;
				//}
			}
			if(RCP_RAP_Code[0] == MSC_RCP)
			{
				//MHL_RCP_Action(RCP_RAP_Code[1]);
				EP_DEV_DBG("RCP Code =0x%x", RCP_RAP_Code[1]);
				//switch(RCP_RAP_Code[1]) 
				//{
				//	case RCP_Select:
				//		EP_DEV_DBG("RCP: Select\n");
				//		break;
				//	case RCP_Up:
				//		EP_DEV_DBG("RCP: Up\n");
				//		break;
				//	case RCP_Down:
				//		EP_DEV_DBG("RCP: Down\n");
				//		break;
				//	case RCP_Left:
				//		EP_DEV_DBG("RCP: Left\n");
				//		break;
				//	case RCP_Right:
				//		EP_DEV_DBG("RCP: Right\n");
				//		break;
				//}
				if(!(RCP_RAP_Code[1] & 0x80))
				{
					Send_RCP_Key_Code(RCP_RAP_Code[1] & 0x7F);
				}
			}
			if(RCP_RAP_Code[0] == MSC_UCP)
			{
				//MHL_UCP_Action(RCP_RAP_Code[1]);
				EP_DEV_DBG("UCP Code =0x%x", RCP_RAP_Code[1]);
			}
		}
	}
	mutex_unlock(&mhl_lock);
	/////////////////////////////////////////////////////////////////
}

//------------------------------------------------------------------------------------------------

#ifdef CONFIG_EP957B_MHL
void mhl_notify_vbus_valid(int event)
{
	queue_work(initializeWorkQueue, &workItem);
//	vbus_mode = event;
//	EP_DEV_DBG("start event=%d\n", event);
}
static void mhl_device_monitoring(struct work_struct *work)
{
	if (!atomic_read(&mhl_device_available)) {
		EP_DEV_DBG("Skip.\n");
		return;
	}

       /*EP957B Monitoling*/
	EP957_Reg_Read(EP957_TX_PHY_Control_1, &Temp_Data[0], 1);
//	EP_DEV_DBG("EP957_TX_PHY_Control_0 = 0x%0X\n",Temp_Data[0]);
	if((Temp_Data[0] & 0x0F) == 0) {
		EP_DEV_DBG("EP957B Recovery\n");
		EP957Control_Reset();

		EP957_If_Reset();
	}
}
void	mhl_device_monitoring_init(void)
{
	mhlcb.fn = mhl_notify_vbus_valid;
	usb_mhl_reg_cbfunc(&mhlcb);
	initializeWorkQueue = create_workqueue("EP957B_initialize_work");
	if(initializeWorkQueue == NULL)
		EP_DEV_DBG("Initialization of workqueue failed,\n");
	INIT_WORK(&workItem, mhl_device_monitoring);
}
void	mhl_device_monitoring_exit(void)
{
	usb_mhl_unreg_cbfunc(&mhlcb);
	destroy_workqueue(initializeWorkQueue);
	initializeWorkQueue = NULL;
}
#endif/*CONFIG_EP957B_MHL*/

ssize_t EP957ShowConnectionState(struct device *dev, struct device_attribute *attr,
							char *buf)
{
	
	if(EP957_Read_LINK_STATE() > CBUS_LINK_STATE__USB_Mode) {
		EP_DEV_DBG("connected\n");
		return scnprintf(buf, PAGE_SIZE, "connected");
	} else {
		EP_DEV_DBG("not connected\n");
		return scnprintf(buf, PAGE_SIZE, "not connected");
	}
}

bool mhl_power = true;
void EP957_MHL_module_reset(bool on)
{
	EP_DEV_DBG("(%d)\n", on);
	if (on) {
		if (!mhl_power) {
			cancel_delayed_work_sync(&work_PPG);
			hrtimer_cancel(&pTimer_Link_Task);

			mutex_lock(&mhl_lock);

			if (EP957_Read_LINK_STATE() > CBUS_LINK_STATE__USB_Mode) {
				EP_DEV_DBG("On process\n");

				EP957Control_Reset();

				// EP957 Controller initial.
				Link_State = 0;
				is_Connected_backup = 0;
				is_CurrentMode = NO_CONNECT;

				EP957Control_Initial();
				// EP957 Controller settings.
				EP957Control_ContentOn();

			} else {
				EP_DEV_DBG("On process Skip: mhl not connected.\n");
			}

			mutex_unlock(&mhl_lock);

			hrtimer_start(&pTimer_Link_Task, ktime_set(0, 55*1000000), HRTIMER_MODE_REL);
			mhl_power = true;
		} else {
			EP_DEV_DBG("On process Skip: already on.\n");
		}
	} else {
		if (mhl_power)
			mhl_power = false;
	}

	return;
}
EXPORT_SYMBOL_GPL(EP957_MHL_module_reset);

/*
 * Sysfs attribute files supported by this driver.
 */
struct device_attribute driver_attribs[] = {
		__ATTR(connection_state, 0444, EP957ShowConnectionState, NULL),
		__ATTR_NULL
};

static int EP957_MHL_module_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	/* int retval; */
	int ret;
    EP_LOG_FUNCTION_NAME_ENTRY;
	
	/* rest of the initialisation goes here. */	
	EP_DEV_DBG("EP957 attach success!!!\n");

	EP957_i2c_client = client;

	//guarantee i2c is initialized.. then i2c is accessible.

	mhl_class = class_create(THIS_MODULE, "mhl");
	if (IS_ERR(mhl_class))
	{
		EP_DEV_DBG_ERROR("Failed to create class(mhl)!\n");
		//ret = PTR_ERR(mhlCdev);
		cdev_del(&mhlCdev);
		//return ret;
	}

	mhl_class->dev_attrs = driver_attribs;

	mhl_dev = device_create(mhl_class, NULL, MKDEV(devMajor, 0), NULL, "EP957_Device");
	if (IS_ERR(mhl_dev))
	{
		EP_DEV_DBG_ERROR("Failed to create device(mhl_dev)!\n");
		//ret = PTR_ERR(mhl_dev);
		class_destroy(mhl_class);
		//return ret;
	}
	
	ret = Init_Keyboard();
	if (ret) 
	{
		EP_DEV_DBG_ERROR("Failed to install keyboard\n");
		//return ret;
	}

	INIT_WORK(&work_Int, callback_work_Int);
	INIT_WORK(&work_Link_Task, callback_work_Link_Task); // modify 130218
	INIT_DELAYED_WORK(&work_PPG, callback_work_PPG);
	
	ret = request_irq(client->irq, callback_irq, IRQF_TRIGGER_FALLING , "MHL_INT", NULL); 
	if (ret) {
		EP_DEV_DBG_ERROR("unable to request irq MHL_INT err:: %d\n", ret);	
		return 0;
	}
	EP_DEV_DBG("MHL int reques successful %d\n", ret);
	
	EP957Control_Reset();

	// EP957 Controller initial.
	Link_State = 0;
	is_Connected_backup = 0;

	EP957Control_Initial();

	// EP957 Controller settings.
	EP957Control_ContentOn();

	hrtimer_init(&pTimer_Link_Task, 1, HRTIMER_MODE_REL); // modify 130218
	pTimer_Link_Task.function = &callback_timer;	// modify 130218	
	hrtimer_start(&pTimer_Link_Task, ktime_set(0, 55*1000000), HRTIMER_MODE_REL); // modify 130218

	{
		int rc = 0;

		mhl_psy.name = "ext-vbus";
		mhl_psy.type = POWER_SUPPLY_TYPE_USB_DCP;
		mhl_psy.supplied_to = mhl_pm_power_supplied_to;
		mhl_psy.num_supplicants = ARRAY_SIZE(mhl_pm_power_supplied_to);
		mhl_psy.properties = mhl_pm_power_props;
		mhl_psy.num_properties = ARRAY_SIZE(mhl_pm_power_props);
		mhl_psy.get_property = mhl_power_get_property;
		mhl_psy.set_property = mhl_power_set_property;

		rc = power_supply_register(&client->dev, &mhl_psy);
		if (rc < 0) {
			EP_DEV_DBG_ERROR("%s:power_supply_register ext_vbus_psy failed\n",
							__func__);
			goto failed_probe_pwr;
		}
	}

	mhl_info.ctxt = NULL;
	mhl_info.notify = mhl_device_discovery;
	if (msm_register_usb_ext_notification(&mhl_info)) {
		EP_DEV_DBG_ERROR("%s: register for usb notifcn failed\n", __func__);
		power_supply_unregister(&mhl_psy);
		goto failed_probe_pwr;
	}

failed_probe_pwr:
#ifdef CONFIG_EP957B_MHL
	mhl_device_monitoring_init();
#endif/*CONFIG_EP957B_MHL*/

    EP_LOG_FUNCTION_NAME_EXIT;
	return 0;
}

static int EP957_MHL_module_remove(struct i2c_client *client)
{
	EP957_i2c_client = NULL;
	mhl_device_monitoring_exit();
	device_destroy(mhl_class, 0);
    class_destroy(mhl_class);
    unregister_chrdev_region(MKDEV(devMajor, 0), MHL_DRIVER_MINOR_MAX);
	return 0;
}

static int EP957_MHL_module_suspend(struct i2c_client *client, pm_message_t state)
{
	//timer suspend
	hrtimer_cancel(&pTimer_Link_Task); // modify 130221
	if (atomic_read(&mhl_device_available)) {
		EP_DEV_DBG("unavailable\n");
		atomic_dec(&mhl_device_available);
	}
	return 0;
}
static int EP957_MHL_module_resume(struct i2c_client *client)
{
	//timer resume
	hrtimer_start(&pTimer_Link_Task, ktime_set(0, 55*1000000), HRTIMER_MODE_REL); // 55 ms // modify 130218
	if (!atomic_read(&mhl_device_available)) {
		EP_DEV_DBG("available\n");
		atomic_inc(&mhl_device_available);
	}
	return 0;
}

static struct i2c_device_id EP957_MHL_module_id[] = {
	{"EP957", 0}, // WARNING! Must match I2C_BOARD_INFO() setting in board-omap3beagle.c
	{}
};

struct i2c_driver EP957_MHL_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "EP957_Driver",
	},
	.probe	    = EP957_MHL_module_probe,
	.remove	    = EP957_MHL_module_remove,
	.suspend	= EP957_MHL_module_suspend,
	.resume	    = EP957_MHL_module_resume,
	.id_table	= EP957_MHL_module_id,
	.command    = NULL,
};

static int MHL_open(struct inode *ip, struct file *fp)
{
	EP_DEV_DBG("\n");
	return 0;
}

static int MHL_release(struct inode *ip, struct file *fp)
{
	EP_DEV_DBG("\n");
	return 0;
}
#if 0
static int MHL_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	EP_DEV_DBG("\n");
	return 0;
}
#endif
static struct file_operations MHL_fops = {
	.owner  = THIS_MODULE,
	.open   = MHL_open,
    .release = MHL_release,
    //.ioctl = MHL_ioctl,
};

void EP957_MHL_term_enable(bool enable)
{
	EP_DEV_DBG("%s(%d)\n", __func__, enable);
	mutex_lock(&mhl_lock);

	EP957_term_enable_exec(enable);

	mutex_unlock(&mhl_lock);

	return;
}
EXPORT_SYMBOL_GPL(EP957_MHL_term_enable);

static int __init EP957_MHL_module_init(void)
{
	int ret;
	dev_t devno;
	
    EP_LOG_FUNCTION_NAME_ENTRY;
	
	if (devMajor) {
		devno = MKDEV(devMajor, 0);
		ret = register_chrdev_region(devno,
						MHL_DRIVER_MINOR_MAX,
						MHL_DRIVER_NAME);
	} else {
		ret = alloc_chrdev_region(&devno,
						0,
						MHL_DRIVER_MINOR_MAX,
						MHL_DRIVER_NAME);
		devMajor = MAJOR(devno);
	}
	if (ret < 0) {
		//error
		EP_DEV_DBG_ERROR("Failed to register Major number %d, error code: %d\n", devMajor, ret);
		return ret;
	}
	
	cdev_init(&mhlCdev, &MHL_fops);
	mhlCdev.owner = THIS_MODULE;
	ret = cdev_add(&mhlCdev, devno, MHL_DRIVER_MINOR_MAX);
	if (ret < 0) {
		//error
		EP_DEV_DBG_ERROR("can't add EP957 MHL driver[cdev], error code: %d\n", ret);
		unregister_chrdev_region(MKDEV(devMajor, 0), MHL_DRIVER_MINOR_MAX);
		return ret;
	}

	ret = i2c_add_driver(&EP957_MHL_driver);
	if (ret != 0)
	{
	    EP_DEV_DBG_ERROR("can't add EP957 MHL driver[i2c]\n");	
    }
	else
	{
	    EP_DEV_DBG("add EP957 MHL driver\n");
    }    

	return ret;	
}

static void __exit EP957_MHL_module_exit(void)
{
	i2c_del_driver(&EP957_MHL_driver);
};

void Send_RCP_Key_Code(BYTE rcp_key_code)
{
	char rcp_str[EVENT_STR_LEN_MAX];
	
	if (rcp_key_code > 0x7F || rcp_key_code < 0x00)
	{
		EP_DEV_DBG_ERROR("get rcp_key_code error: rcp_key_code = 0x%x\n",rcp_key_code);
		return;
	}
	
	//strncpy_s(rcp_str, EVENT_STR_LEN_MAX,
	//		MHL_EVENT_RECEIVED_RCP,
	//		(EVENT_STR_LEN_MAX - 1));
	//strncpy(rcp_str,
	//		MHL_EVENT_RECEIVED_RCP,
	//		EVENT_STR_LEN_MAX);
	snprintf(rcp_str, EVENT_STR_LEN_MAX,
			"MHLEVENT=received_RCP key code=0x%02x", rcp_key_code);
	Send_Kobject_Uevent(rcp_str);
	
	switch(rcp_key_code) 
	{
		case RCP_Select:
			EP_DEV_DBG("RCP: Select\n");
			Send_Key_Event(KEY_ENTER);
			break;
		case RCP_Up:
			EP_DEV_DBG("RCP: Up\n");
			Send_Key_Event(KEY_UP);
			break;
		case RCP_Down:
			EP_DEV_DBG("RCP: Down\n");
			Send_Key_Event(KEY_DOWN);
			break;
		case RCP_Left:
			EP_DEV_DBG("RCP: Left\n");
			Send_Key_Event(KEY_LEFT);
			break;
		case RCP_Right:
			EP_DEV_DBG("RCP: Right\n");
			Send_Key_Event(KEY_RIGHT);
			break;
#if 0
		case RCP_Right_Up:
			EP_DEV_DBG("RCP: Right_Up\n");
			break;
		case RCP_Right_Down:
			EP_DEV_DBG("RCP: Right_Down\n");
			break;
		case RCP_Left_Up:
			EP_DEV_DBG("RCP: Left_Up\n");
			break;
		case RCP_Left_Down:
			EP_DEV_DBG("RCP: Left_Down\n");
			break;
#endif
		case RCP_Root_Menu:
			EP_DEV_DBG("RCP: Root_Menu\n");
			Send_Key_Event(KEY_HOMEPAGE);
			break;
#if 0
		case RCP_Setup_Menu:
			EP_DEV_DBG("RCP: Setup_Menu\n");
			break;
		case RCP_Contents_Menu:
			EP_DEV_DBG("RCP: Contents_Menu\n");
			break;
		case RCP_Favorite_Menu:
			EP_DEV_DBG("RCP: Favorite_Menu\n");
			break;
#endif
		case RCP_Exit:
			EP_DEV_DBG("RCP: Exit\n");
			Send_Key_Event(KEY_BACK);
			break;
			
#if 0
		case RCP_Numeric_0:
			EP_DEV_DBG("RCP: Numeric_0\n");
			break;
		case RCP_Numeric_1:
			EP_DEV_DBG("RCP: Numeric_1\n");
			break;
		case RCP_Numeric_2:
			EP_DEV_DBG("RCP: Numeric_2\n");
			break;
		case RCP_Numeric_3:
			EP_DEV_DBG("RCP: Numeric_3\n");
			break;
		case RCP_Numeric_4:
			EP_DEV_DBG("RCP: Numeric_4\n");
			break;
		case RCP_Numeric_5:
			EP_DEV_DBG("RCP: Numeric_5\n");
			break;
		case RCP_Numeric_6:
			EP_DEV_DBG("RCP: Numeric_6\n");
			break;
		case RCP_Numeric_7:
			EP_DEV_DBG("RCP: Numeric_7\n");
			break;
		case RCP_Numeric_8:
			EP_DEV_DBG("RCP: Numeric_8\n");
			break;
		case RCP_Numeric_9:
			EP_DEV_DBG("RCP: Numeric_9\n");
			break;
		case RCP_Dot:
			EP_DEV_DBG("RCP: Dot\n");
			break;
		case RCP_Enter:
			EP_DEV_DBG("RCP: Enter\n");
			break;
		case RCP_Clear:
			EP_DEV_DBG("RCP: Clear\n");
			break;
			
		case RCP_Channel_Up:
			EP_DEV_DBG("RCP: Channel_Up\n");
			break;
		case RCP_Channel_Down:
			EP_DEV_DBG("RCP: Channel_Down\n");
			break;
		case RCP_Previous_Channel:
			EP_DEV_DBG("RCP: Previous_Channel\n");
			break;
			
		case RCP_Volume_Up:
			EP_DEV_DBG("RCP: Volume_Up\n");
			break;
		case RCP_Volume_Down:
			EP_DEV_DBG("RCP: Volume_Down\n");
			break;
		case RCP_Mute:
			EP_DEV_DBG("RCP: Mute\n");
			break;
#endif
		case RCP_Play:
			EP_DEV_DBG("RCP: Play\n");
			Send_Key_Event(KEY_PLAYCD);
			break;
		case RCP_Stop:
			EP_DEV_DBG("RCP: Stop\n");
			Send_Key_Event(KEY_STOPCD);
			break;
		case RCP_Pause:
			EP_DEV_DBG("RCP: Pause\n");
			Send_Key_Event(KEY_PLAYPAUSE);
			break;
#if 0
		case RCP_Record:
			EP_DEV_DBG("RCP: Record\n");
			break;
#endif
		case RCP_Rewind:
			EP_DEV_DBG("RCP: Rewind\n");
			Send_Key_Event(KEY_REWIND);
			break;
		case RCP_Fast_Forward:
			EP_DEV_DBG("RCP: Fast_Forward\n");
			Send_Key_Event(KEY_FASTFORWARD);
			break;
#if 0
		case RCP_Eject:
			EP_DEV_DBG("RCP: Eject\n");
			break;
#endif
		case RCP_Forward:
			EP_DEV_DBG("RCP: Forward\n");
			Send_Key_Event(KEY_NEXTSONG);
			break;
		case RCP_Backward:
			EP_DEV_DBG("RCP: Backward\n");
			Send_Key_Event(KEY_PREVIOUSSONG);
			break;
#if 0
		case RCP_Angle:
			EP_DEV_DBG("RCP: Angle\n");
			break;
		case RCP_Subpicture:
			EP_DEV_DBG("RCP: Subpicture\n");
			break;
#endif
		case RCP_Play_Function:
			EP_DEV_DBG("RCP: Play_Function\n");
			Send_Key_Event(KEY_PLAYCD);
			break;
		case RCP_Pause_Play_Function:
			EP_DEV_DBG("RCP: Pause_Play_Function\n");
			Send_Key_Event(KEY_PLAYPAUSE);
			break;
#if 0
		case RCP_Record_Function:
			EP_DEV_DBG("RCP: Record_Function\n");
			break;
		case RCP_Pause_Record_Function:
			EP_DEV_DBG("RCP: Pause_Record_Function\n");
			break;
#endif
		case RCP_Stop_Function:
			EP_DEV_DBG("RCP: Stop_Function\n");
			Send_Key_Event(KEY_STOPCD);
			break;
#if 0
		case RCP_Mute_Function:
			EP_DEV_DBG("RCP: Mute_Function\n");
			break;
		case RCP_Restore_Volume_Function:
			EP_DEV_DBG("RCP: Restore_Volume_Function\n");
			break;
		case RCP_Tune_Function:
			EP_DEV_DBG("RCP: Tune_Function\n");
			break;
		case RCP_Select_Media_Function:
			EP_DEV_DBG("RCP: Select_Media_Function\n");
			break;
#endif
		// Color Keys are not defined in MHL 1.1
		case RCP_F1:
			EP_DEV_DBG("RCP: F1(Blue)\n");
			Send_Key_Event(KEY_BACK);
			break;
		case RCP_F2:
			EP_DEV_DBG("RCP: F2(Red)\n");
			break;
		case RCP_F3:
			EP_DEV_DBG("RCP: F3(Green)\n");
			Send_Key_Event(KEY_MENU);
			break;
		case RCP_F4:
			EP_DEV_DBG("RCP: F4(Yellow)\n");
			break;
#if 0
		case RCP_F5:
			EP_DEV_DBG("RCP: F5\n");
			break;
		case RCP_Vendor_Specific:
			EP_DEV_DBG("RCP: Vendor_Specific\n");
			break;
#endif
		default:
			EP_DEV_DBG_ERROR("RCP: Unknown Key Code = 0x%x\n", rcp_key_code);
			break;
	}
}

void Send_Key_Event(unsigned int key_event)
{
	if (key_event < 0)
	{
		EP_DEV_DBG_ERROR("get key_event error: key_event = %d\n",key_event);
		return;
	}
	
	EP_DEV_DBG("send key_event as input key event: key_event = %d\n",key_event);
	
	input_report_key(dev_keyboard, key_event, 1);
	input_sync(dev_keyboard);
	input_report_key(dev_keyboard, key_event, 0);
	input_sync(dev_keyboard);
}

inline int Init_Keyboard(void)
{
    int error;

	dev_keyboard = input_allocate_device();
	if (!dev_keyboard)
	{
		EP_DEV_DBG_ERROR("dev_keyboard: Not enough memory\n");
		error = -ENOMEM;
		goto err_free_irq;
	}

	set_bit(EV_KEY, dev_keyboard->evbit);
	set_bit(EV_REP, dev_keyboard->evbit);

	dev_keyboard->phys = "atakbd/input0";
	dev_keyboard->id.bustype = BUS_HOST;

	set_bit(KEY_UP, dev_keyboard->keybit);
	set_bit(KEY_DOWN, dev_keyboard->keybit);
	set_bit(KEY_LEFT, dev_keyboard->keybit);
	set_bit(KEY_RIGHT, dev_keyboard->keybit);
	set_bit(KEY_ENTER, dev_keyboard->keybit);
	set_bit(KEY_BACK, dev_keyboard->keybit);
	set_bit(KEY_PLAYPAUSE, dev_keyboard->keybit);
	set_bit(KEY_STOP, dev_keyboard->keybit);
	set_bit(KEY_SELECT, dev_keyboard->keybit);
	set_bit(KEY_OK,dev_keyboard->keybit);
	set_bit(KEY_REPLY,dev_keyboard->keybit);
	set_bit(KEY_PLAYCD,dev_keyboard->keybit);
	set_bit(KEY_STOPCD,dev_keyboard->keybit);

	set_bit(BTN_LEFT,dev_keyboard->keybit);
	set_bit(BTN_SELECT,dev_keyboard->keybit);

	set_bit(KEY_HOMEPAGE,dev_keyboard->keybit);
	set_bit(KEY_REWIND,dev_keyboard->keybit);
	set_bit(KEY_FASTFORWARD,dev_keyboard->keybit);
	set_bit(KEY_NEXTSONG,dev_keyboard->keybit);
	set_bit(KEY_PREVIOUSSONG,dev_keyboard->keybit);
	set_bit(KEY_MENU,dev_keyboard->keybit);

	dev_keyboard->id.bustype = BUS_USB;
	dev_keyboard->id.vendor  = 0x0000;
	dev_keyboard->id.product = 0x0000;
	dev_keyboard->id.version = 0xA;				//use version to distinguish mouse from keyboard

	error = input_register_device(dev_keyboard);
	if (error) {
			EP_DEV_DBG_ERROR("dev_keyboard: Failed to register device\n");
			goto err_free_dev;
	}

	EP_DEV_DBG("dev_keyboard: driver loaded\n");

	return 0;

 err_free_dev:
         input_free_device(dev_keyboard);
 err_free_irq:
         return error;
}

void Send_Kobject_Uevent(char str[EVENT_STR_LEN_MAX])
{
	char event_str[EVENT_STR_LEN_MAX];
	char *env[] = {event_str, NULL};
	int ret;
	
	//strncpy(event_str, str, EVENT_STR_LEN_MAX);
	//strncpy_s(event_str, EVENT_STR_LEN_MAX, str, (EVENT_STR_LEN_MAX - 1));
	strncpy(event_str, str, EVENT_STR_LEN_MAX);
	
	ret = kobject_uevent_env(&mhl_dev->kobj, KOBJ_CHANGE, env);
	if(ret != 0)
	{
		EP_DEV_DBG_ERROR("send kobject_uevent_env error: ret = %d, event_str = '%s'\n", ret, event_str);
		return;
	}
	
	EP_DEV_DBG("send kobject_uevent_env: ret = %d, event_str = '%s'\n", ret, event_str);
	return;
}

void Notify_Connect_Status(BOOL is_connected)
{
	char status_str[EVENT_STR_LEN_MAX];
	/*
	if (is_connected < STATUS_DISCONNECT || is_connected > STATUS_CONNECT) 
	{
		// error
		EP_DEV_DBG_ERROR("get Connect Status error: is_connected = %d\n", is_connected);
		return;
	}
	*/
	if (is_Connected_backup != is_connected) 
	{
		// Save new status
		is_Connected_backup = is_connected;
		EP_DEV_DBG("send kobject_uevent_env: is_connected = %d\n", is_connected);
		
		if (is_connected == STATUS_CONNECT) 
		{
			strncpy(status_str,
				MHL_EVENT_CONNECTED,
				EVENT_STR_LEN_MAX);
		}
		else if (is_connected == STATUS_DISCONNECT)
		{
			strncpy(status_str,
				MHL_EVENT_DISCONNECTED,
				EVENT_STR_LEN_MAX);
		}
		
		Send_Kobject_Uevent(status_str);
	}
	else
	{
		// Connection Status no change
	}
}

module_init(EP957_MHL_module_init);
module_exit(EP957_MHL_module_exit);

MODULE_DESCRIPTION("EP957 MHL driver");
MODULE_AUTHOR("Explore Semiconductor <http://www.epmi.com.tw/>");
MODULE_LICENSE("GPL");
