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

  File        :  EP957Controller.c 

  Description :  EP957Controller program 
                 Handle the HDMI(MHL) Link State.

  Codeing     :  Shihken

  2011.06.13  :  1) Set the Version Number to 1.30 Beta 30
                 

\******************************************************************************/

#include "EP957Controller.h"
#include <linux/mfd/pm8xxx/pm8921-charger.h>
#include <linux/power_supply.h>

//
// Global State and Flags
//

// System flags
BOOL is_Connected;
BOOL is_Source_Ready;
BOOL is_ReceiverSense;
BOOL is_RSEN_bkup;
BOOL is_VBus_Power;
BOOL is_CBUS_OK;
BOOL is_PATH_EN;
BOOL is_PackedPixelMode;
BOOL is_Repeater;
BOOL is_First_Auth;
BOOL is_DACP_Change;


//
// Global Data
//

// Temp Data
static int i;
static BYTE Temp_Byte[256];

// System Data
CBUS_LINK_STATE ConnectionState;
BYTE TimeOut_Count, RSEN_ChangeCount;
PPG_STATE PPG_State;
WORD ReadEDID_TimeCount;


//
// Private Functions
//

void Exchange_DCAP(void);
void Set_DCAP_Ready(void);
void EP957Control_HDCP_Proc1(void);
void PPGS_RollBack_Wait_Upstream(void);
void PPGS_RollBack_Stream(void);
void Set_Gpio_Reset(int state);
void Wait_Reset_Timer(int wait_time_m_sec);
void Set_Gpio_Init(void);

//--------------------------------------------------------------------------------------------------
// Public Functions
//--------------------------------------------------------------------------------------------------

// The initial function that should be called once.
void EP957Control_Initial(void)
{
	//
	// Reset EP957 Control Program
	//
	
	// EP957 Interface Reset
	EP957_If_Reset();

	// VBus Interface Reset
//	VBUS_If_Reset();		


	// Reset Variables

	// bit
	is_Connected = 0;
	is_Source_Ready = 0;
	is_ReceiverSense = 0;
	is_RSEN_bkup = 0;
	is_VBus_Power = 0;
	is_PATH_EN = 0;
	is_PackedPixelMode = 0;
	is_Repeater = 0;
	is_First_Auth = 0;
	is_DACP_Change = 0;

	// data
	ConnectionState = CBUS_LINK_STATE__USB_Mode;
	TimeOut_Count = 0;
	RSEN_ChangeCount = 0;
	PPG_State = PPGS_Search_EDID;
	ReadEDID_TimeCount = 0;
}

// The looping task that check the CBUS connection.
CBUS_LINK_STATE EP957Control_Link_Task(void)
{
	//
	// Check the basic CBUS connection status
	//

	// Check RSEN
	if(MHL_Tx_RSEN()) {
		is_ReceiverSense = 1;
		RSEN_ChangeCount = 0;
	}
	else {
		// RSEN Deglitch timing: min = 100 ms; Max = 200 ms
		if(++RSEN_ChangeCount > (150/LINK_TASK_TIME_BASE)) { // Accept continuous 1 error = 80 ms
			RSEN_ChangeCount = 0;
			is_ReceiverSense = 0;
		}
	}

	// for debug
	if(is_RSEN_bkup != is_ReceiverSense) {
		is_RSEN_bkup = is_ReceiverSense;
		EP_DEV_DBG("RSEN = 0x%X\n", is_ReceiverSense);
	}

	// CBUS connections state 
	// The delay between each state must be > 50 ms.
	switch(ConnectionState) {

		case CBUS_LINK_STATE__USB_Mode: // Measure the CBUS Impedance
			if( MHL_Tx_MEASURE_1KOHM() ) {
				EP_DEV_DBG("CBUS Connection: 1KOhm Detected\n");						
				MHL_Tx_USB_Mode(FALSE);
//				is_VBus_Power = VBUS_Power(0);
				ConnectionState = CBUS_LINK_STATE__1KOHM_Detected;
			}
			else {
				// USB Mode	
				MHL_Tx_USB_Mode(TRUE);
			}
			break;

		case CBUS_LINK_STATE__1KOHM_Detected: // Try VBus On
//			is_VBus_Power = VBUS_Power(1);
			MHL_Tx_CBUS_Connect();
			EP_DEV_DBG("CBUS Connection: Start Connect\n");
			ConnectionState = CBUS_LINK_STATE__Start_Connect;
			TimeOut_Count = 0;
			break;

		case CBUS_LINK_STATE__Start_Connect: // Check CBUS Connection status
			if( MHL_Tx_Connection_Status() ) {
			
				if( MHL_Tx_Connection_Status() == MHL_NOT_CONNECTED) {
					MHL_Tx_CBUS_Disconnect();
					
//					is_VBus_Power = VBUS_Power(0); // VBUS Off
					
					EP_DEV_DBG("CBUS Connection: Connect Fail\n");
					ConnectionState = CBUS_LINK_STATE__USB_Mode;
				}
				else {
					is_CBUS_OK = 0;
					is_DACP_Change = 0;
	
					EP_DEV_DBG("CBUS Connection: Connected\n");
					ConnectionState = CBUS_LINK_STATE__Check_DCAP;
					TimeOut_Count = 0;
				}
			}
			else if(TimeOut_Count >= (2000/LINK_TASK_TIME_BASE)) { // 2 sec timeout
				MHL_Tx_CBUS_Disconnect();
//				is_VBus_Power = VBUS_Power(0); // VBUS Off

				EP_DEV_DBG("CBUS Connection: Connection Timeout\n");
				ConnectionState = CBUS_LINK_STATE__USB_Mode;
			}
			else {
				TimeOut_Count++;
/*				
				if( !MHL_Tx_VBUS_Power() ) {
					MHL_Tx_CBUS_Disconnect();
//					is_VBus_Power = VBUS_Power(0); // VBUS Off
					
					EP_DEV_DBG("CBUS Connection: VBUS missing");
					ConnectionState = CBUS_LINK_STATE__USB_Mode;
				}
*/
			}
			break;
	
		case CBUS_LINK_STATE__Check_DCAP: // Check DCAP 

			//if(MHL_MSC_Reg_Read(MSC_RCHANGE_INT) & DCAP_CHG) {
			if(is_DACP_Change) { // Use interrupt pin to detect the DCAP_CHG event

				if(MHL_MSC_Reg_Read(MSC_STATUS_CONNECTED_RDY) & DCAP_RDY) {
					
					// Read Device Cap
					EP_DEV_DBG("Device Cap Change\n");
					Exchange_DCAP();
					Set_DCAP_Ready();
					ConnectionState = CBUS_LINK_STATE__Connected;
				}
			}
			else {
				if(MHL_MSC_Reg_Read(MSC_STATUS_CONNECTED_RDY) & DCAP_RDY) {
				
					// Read Device Cap
					EP_DEV_DBG("Device Cap Ready\n");
					Exchange_DCAP();
					Set_DCAP_Ready();
					ConnectionState = CBUS_LINK_STATE__Connected;
				}
			}	

			// Make sure the RSEN is OK
			if(TimeOut_Count >= (100/LINK_TASK_TIME_BASE)) { // RXRSEN_CHK = 300 - 500 ms
				if(!is_ReceiverSense) {
		
					MHL_Tx_CBUS_Disconnect();
//					is_VBus_Power = VBUS_Power(0); // VBus Off
		
					EP_DEV_DBG("CBUS Connection: RSEN missing\n");
					ConnectionState = CBUS_LINK_STATE__USB_Mode;
				}
			}
			
			// Time-out
			if(TimeOut_Count >= (2000/LINK_TASK_TIME_BASE)) { // 2 sec timeout
			
				// Read Device Cap
				EP_DEV_DBG("Device Cap waiting Timeout\n");
				//Exchange_DCAP(); // Force to read DCAP or Skip? (Skip it for 6.3.6.4 - 3 test)
				Set_DCAP_Ready();
				ConnectionState = CBUS_LINK_STATE__Connected;
			}

			TimeOut_Count++;
			break;

		case CBUS_LINK_STATE__Connected: // CBus Connected

			// Check Hot-Plug / EDID Change Interrupt
			if( MHL_Tx_Connection_Status() == MHL_HOTPLUG_DETECT ) { // CBUS Connect && MSC_HPD detect
				if(MHL_MSC_Reg_Read(MSC_DCHANGE_INT) & EDID_CHG) {
					is_Connected = 0;
					Notify_Connect_Status(is_Connected);
				}
				else {
					is_Connected = 1;
					Notify_Connect_Status(is_Connected);
				}
			}
			else {
				is_Connected = 0;
				Notify_Connect_Status(is_Connected);
			}

			// Check Receiver Sense
			if(!is_ReceiverSense) {

				is_Connected = 0;
				Notify_Connect_Status(is_Connected);

				MHL_Tx_CBUS_Disconnect();
//				is_VBus_Power = VBUS_Power(0); // VBus Off

				EP_DEV_DBG("CBUS Connection: Disconnect \n");
				ConnectionState = CBUS_LINK_STATE__USB_Mode;
				
				// Reset Propagation Task to Power Down TX and RX both.
				PPGS_RollBack_Stream();
				PPGS_RollBack_Wait_Upstream();
				break; // modify 130205
			}
			// Check PATH_EN
			if(MHL_MSC_Reg_Read(MSC_STATUS_LINK_MODE) & PATH_EN) {
				if(!is_PATH_EN) {
					EP_DEV_DBG("CBUS Connection: Get PATH_EN = 1\n");
					is_PATH_EN = 1;
					
					if(is_PackedPixelMode) 
						MHL_MSC_Cmd_WRITE_STATE(MSC_STATUS_LINK_MODE, CLK_MODE__PacketPixel | PATH_EN);
					else
						MHL_MSC_Cmd_WRITE_STATE(MSC_STATUS_LINK_MODE, CLK_MODE__Normal    | PATH_EN);
				}
			}
			else {
				if(is_PATH_EN) {
					EP_DEV_DBG("CBUS Connection: Get PATH_EN = 0\n");
					is_PATH_EN = 0;
		
					if(is_PackedPixelMode) 
						MHL_MSC_Cmd_WRITE_STATE(MSC_STATUS_LINK_MODE, CLK_MODE__PacketPixel);
					else
						MHL_MSC_Cmd_WRITE_STATE(MSC_STATUS_LINK_MODE, CLK_MODE__Normal);
				}
			}
			break;
	}

	return ConnectionState;
}

// Tis function require a fast looping to handle the EDID and HDCP propagation job.
PPG_STATE EP957Control_Propagation_Task(void)
{
	//
	// Handle the main HDMI Link State and HDCP Authentication Process
	// The state transition: [Search_EDID] => [Wait_Upstream] => [Stream] => [HDCP]
	//		   			

	switch(PPG_State) {
		case PPGS_Search_EDID:
			if(is_Connected && is_ReceiverSense && is_PATH_EN) {
				unsigned char EDID_DDC_Status;

				// Disable RI Auto Update
				HDMI_RQ_RI_Enable(FALSE);

				// Read EDID
					EP_DEV_DBG("State Transist: Read EDID -> [PPGS_Wait_Upstream]\n");				
				memset(Temp_Byte, 0xFF, 256);
				EDID_DDC_Status = Downstream_Rx_read_EDID(Temp_Byte);
				if(EDID_DDC_Status) {
						EP_DEV_DBG("Err: EDID read failed 0x%X\n", EDID_DDC_Status);
					if(++ReadEDID_TimeCount <= 20) break;
				}
				ReadEDID_TimeCount = 0;

				// Write EDID
				HDMI_Rx_write_EDID(Temp_Byte);

				// Power Up Rx
				HDMI_Rx_Power_Up();

				// Enable Hot-Plug to the Source
				HDMI_Rx_HotPlug_Set();

				PPG_State = PPGS_Wait_Upstream;
			}
			else {	
				ReadEDID_TimeCount = 0;
			}
			break;
			
		case PPGS_Wait_Upstream:

			if(!is_Connected || !is_ReceiverSense || !is_PATH_EN) {

				// for debug ==============================================================
				if(!is_Connected)		EP_DEV_DBG("PPGS_Wait_Upstream: check is_Connected = 0\n");
				if(!is_ReceiverSense)	EP_DEV_DBG("PPGS_Wait_Upstream: check is_ReceiverSense = 0\n");
				if(!is_PATH_EN)			EP_DEV_DBG("PPGS_Wait_Upstream: check is_PATH_EN = 0\n");
				//=====================================================================
				
				PPGS_RollBack_Stream();
				PPGS_RollBack_Wait_Upstream();
			}
			else if(is_Source_Ready) {

				if(HDMI_Rx_Signal()) {
						EP_DEV_DBG("State Transist: Power Up -> [PPGS_Wait_Authentication]\n");	
						
					// Disable RI Auto Update
					//HDMI_RQ_RI_Enable(FALSE); //modify 130205
	
					// Power Up Tx
					MHL_Tx_Power_Up();
					
					// Clear flags
					HDMI_Rx_HDCP_Get_Flags(); 

					// Copy HDCP registers to Source
					// BKSV, BCaps
					if(!Downstream_Rx_read_BKSV(Temp_Byte)) {
						if(!Downstream_Rx_read_BKSV(Temp_Byte)) {
								EP_DEV_DBG("Err: BKSV read fail\n");
								break; //modify 130205	
						}
					}
					HDMI_Rx_write_BKSV(Temp_Byte);
					Temp_Byte[0] = Downstream_Rx_BCAPS();
					HDMI_Rx_write_BCAPS(Temp_Byte[0]);
					if(Temp_Byte[0] & 0x40) {
						is_Repeater = 1;
					}
					else { //modify 130205
						is_Repeater = 0; //modify 130205
					}	//modify 130205

					is_First_Auth = 0;
					PPG_State = PPGS_Wait_Authentication;
				}
			}
			else {
			}
			break;

		case PPGS_Wait_Authentication:

			if(!is_Connected || !is_ReceiverSense || !is_PATH_EN) {

				// for debug ==============================================================
				if(!is_Connected)		EP_DEV_DBG("PPGS_Wait_Authentication: check is_Connected = 0\n");
				if(!is_ReceiverSense)	EP_DEV_DBG("PPGS_Wait_Authentication: check is_ReceiverSense = 0\n");
				if(!is_PATH_EN)			EP_DEV_DBG("PPGS_Wait_Authentication: check is_PATH_EN = 0\n");
				//=====================================================================

				PPGS_RollBack_Stream();
				PPGS_RollBack_Wait_Upstream();
			}
			else if(!is_Source_Ready || !HDMI_Rx_Signal()) {

				// for debug ==============================================================
				if(!is_Source_Ready) {
					EP_DEV_DBG("PPGS_Wait_Authentication: check is_Source_Ready = 0\n");
				} else {
					EP_DEV_DBG("PPGS_Wait_Authentication: check HDMI_Rx_Signal = 0\n");
				}
				//=====================================================================

				PPGS_RollBack_Stream();
			}
			else {

				// Detect the AKSV write
				if(HDMI_Rx_HDCP_Get_Flags() & EP957_General_Control_2__AKSV_F) {

					// Copy HDCP registers to Sink
					// AINFO, AN, AKSV
					EP957Control_HDCP_Proc1();

					// Next State
					if(!is_Repeater) {
						PPG_State = PPGS_HDCP_Done;
					}

					is_First_Auth = 1;
				}										

				// FIFO, V, BSTATUS, BCAPS
				if(is_First_Auth && is_Repeater) {

					// Wait for the Ready bit
					Temp_Byte[0] = Downstream_Rx_BCAPS();
					if(Temp_Byte[0] & 0x20) {

						// Read Device Count and Device Depth
						Downstream_Rx_read_BSTATUS(&Temp_Byte[1]);
						HDMI_Rx_write_BSTATUS(&Temp_Byte[1]);
                                                // modify 130205
						for(i=0; i<(Temp_Byte[1] & 0x7F) && i<8; ++i) {// modify 130417 - fix typo
							// Read FIFO
							if(!Downstream_Rx_read_KSV_FIFO(&Temp_Byte[3], i, (Temp_Byte[1] & 0x7F))) {//modify 130417 - fix typo
								break; // modify 130205
							}
							HDMI_Rx_write_FIFO(&Temp_Byte[3], i);
						}
						// Read V (SHA)
						Downstream_Rx_read_SHA1_HASH(&Temp_Byte[1]);
						HDMI_Rx_write_V(&Temp_Byte[1]);

						// Assert the Ready bit to HDMI source
						HDMI_Rx_write_BCAPS(Temp_Byte[0]);

							EP_DEV_DBG("HDCP Propagation Part 2 Done\n");

						// Next State
						PPG_State = PPGS_HDCP_Done;
					}								   
				}
			}
			break;
			
		case PPGS_HDCP_Done:

			if(!is_Connected || !is_ReceiverSense || !is_PATH_EN) {

				// for debug ==============================================================
				if(!is_Connected)		EP_DEV_DBG("PPGS_HDCP_Done: check is_Connected = 0\n");
				if(!is_ReceiverSense)	EP_DEV_DBG("PPGS_HDCP_Done: check is_ReceiverSense = 0\n");
				if(!is_PATH_EN)			EP_DEV_DBG("PPGS_HDCP_Done: check is_PATH_EN = 0\n");
				//=====================================================================

				PPGS_RollBack_Stream();
				PPGS_RollBack_Wait_Upstream();
			}
			else if(!is_Source_Ready) {

				// for debug ==============================================================
				if(!is_Source_Ready)	EP_DEV_DBG("PPGS_HDCP_Done: check is_Source_Ready = 0\n");
				//=====================================================================

				PPGS_RollBack_Stream();
			}
			else {

				// Detect the AKSV write
				if(HDMI_Rx_HDCP_Get_Flags() & EP957_General_Control_2__AKSV_F) {

					// Copy HDCP registers to Sink
					// AINFO, AN, AKSV
					EP957Control_HDCP_Proc1();

					// Next State
					if(!is_Repeater) {
						//PPG_State = PPGS_HDCP_Done;
					}
					else {
						PPG_State = PPGS_Wait_Authentication;
					}
				}
			}
			break;
	}

	return PPG_State;
}

// This functions should be called when interrupt is detected.
// Polling this function every 200ms is also avaliable.
// The function return TRUE when it get the RCP command.
BOOL EP957Control_RCP_RAP_Read(PBYTE RCP_RAP_Code) 
{
	BYTE IntFlag_Byte;

	// MHL Interrupt
	IntFlag_Byte = MHL_MSC_Get_Flags();
        
        // modify 130205 
	if(!(IntFlag_Byte & EP957_CBUS_MSC_Interrupt__HPD_S)) {
		// RQ_RI_EN
		HDMI_RQ_RI_Enable(FALSE);					 
		EP_DEV_DBG("No HPD\n");
	}
	if(IntFlag_Byte & EP957_CBUS_MSC_Interrupt__MSG_F) {
		
		MHL_RCP_RAP_Read(RCP_RAP_Code);

		switch(RCP_RAP_Code[0]) {

			// RAP
			case MSC_RAP:
				EP_DEV_DBG("RAP Received: 0x%02X\n", RCP_RAP_Code[1]);
				MHL_MSC_Cmd_MSC_MSG(MSC_RAPK, RAPK_No_Error);
				break;

			case MSC_RAPK:
				if(RCP_RAP_Code[1])	EP_DEV_DBG("RAP Send Err: 0x%02X\n", RCP_RAP_Code[1]);
				break;

			// RCP
			case MSC_RCP:
				EP_DEV_DBG("RCP Received: 0x%02X\n", RCP_RAP_Code[1]);
				MHL_MSC_Cmd_MSC_MSG(MSC_RCPK, RCP_RAP_Code[1]);
				break;

			case MSC_RCPE:
				if(RCP_RAP_Code[1])	EP_DEV_DBG("RCP Send Err: 0x%02X\n", RCP_RAP_Code[1]);
				break;
				
			// UCP
			case MSC_UCP:
				EP_DEV_DBG("UCP Received: 0x%02X\n", RCP_RAP_Code[1]);
				MHL_MSC_Cmd_MSC_MSG(MSC_UCPK, RCP_RAP_Code[1]);
				break;

			case MSC_UCPE:
				if(RCP_RAP_Code[1])	EP_DEV_DBG("UCP Send Err: 0x%02X\n", RCP_RAP_Code[1]);
				break;
		}

		return TRUE;
	}
	if(IntFlag_Byte & EP957_CBUS_MSC_Interrupt__INT_F) {
		BYTE RCHANGE_INT_value;

		RCHANGE_INT_value = MHL_MSC_Reg_Read(MSC_RCHANGE_INT);

		// Detect Device Capability Change Interrupt (during the connected state)
		if(RCHANGE_INT_value & DCAP_CHG) {
			EP_DEV_DBG("INT Received: DCAP_CHG\n"); 
			
			is_DACP_Change = 1;

			if(is_Connected) {
				// Check POW bit from Sink
				is_CBUS_OK = MHL_MSC_Cmd_READ_DEVICE_CAP(MSC_DEV_CAT, Temp_Byte);
				if(Temp_Byte[0] & POW) {
//					is_VBus_Power = VBUS_Power(0); // VBus Off
				}
			}
		}

		// Scratchpad Transmit Handling
		if(RCHANGE_INT_value & GRT_WRT) {
/*
			////////////////////////////////////////////////////////////////////////
			// Customer should implement their own code here
			//
			static unsigned int WriteBurstCount = 0;

			BYTE BurstData[16] = {
				0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F
			};
			BurstData[0] = Device_Capability_Default[3]; // ADOPTER_ID_H
			BurstData[1] = Device_Capability_Default[4]; // ADOPTER_ID_L
			BurstData[2] = WriteBurstCount >> 8;
			BurstData[3] = WriteBurstCount;

			EP_DEV_DBG("INT Received: GRT_WRT\n"); 
			MHL_MSC_Cmd_WRITE_BURST(MSC_SCRATCHPAD, BurstData, 16);
			WriteBurstCount++;
			//
			////////////////////////////////////////////////////////////////////////
*/
			MHL_MSC_Cmd_WRITE_STATE(MSC_RCHANGE_INT, DSCR_CHG);
		}
		
		// Scratchpad Receive Handling
		if(RCHANGE_INT_value & DSCR_CHG) { //  1st priority
			EP_DEV_DBG("INT Received: DSCR_CHG\n");     

			////////////////////////////////////////////////////////////////////////
			// Customer should implement their own code here
			//
			for(i=0; i< Device_Capability_Default[SCRATCHPAD_SIZE]; i++) {
				if(i==0) {
					EP_DEV_DBG("Burst_ID[0] = 0x%x\n", MHL_MSC_Reg_Read(MSC_SCRATCHPAD+i) );
				}
				else if(i==1) {
					EP_DEV_DBG("Burst_ID[1] = 0x%x\n", MHL_MSC_Reg_Read(MSC_SCRATCHPAD+i) );
				}
				else {
					EP_DEV_DBG("0x%x\n", MHL_MSC_Reg_Read(MSC_SCRATCHPAD+i) );
				}
			}
			//
			////////////////////////////////////////////////////////////////////////
		}
		if(RCHANGE_INT_value & REQ_WRT) { // Lowest prioirty to Grant to Write
			EP_DEV_DBG("INT Received: REQ_WRT\n");
			MHL_MSC_Cmd_WRITE_STATE(MSC_RCHANGE_INT, GRT_WRT);
		}

	}
	return FALSE;
}

void EP957Control_ContentOn(void)
{
	is_Source_Ready = 1;

	if(ConnectionState == CBUS_LINK_STATE__Connected) {
		MHL_MSC_Cmd_MSC_MSG(MSC_RAP, RAP_CONTENT_ON);
	}
}

void EP957Control_ContentOff(void)
{
	is_Source_Ready = 0;

	if(ConnectionState == CBUS_LINK_STATE__Connected) {
		MHL_MSC_Cmd_MSC_MSG(MSC_RAP, RAP_CONTENT_OFF);
	}
}

void EP957Control_PackedPixelModeOn(void)
{
	is_PackedPixelMode = 1;

	if(ConnectionState == CBUS_LINK_STATE__Connected) {
		
		if(is_PATH_EN)
			MHL_MSC_Cmd_WRITE_STATE(MSC_STATUS_LINK_MODE, CLK_MODE__PacketPixel | PATH_EN);
		else
			MHL_MSC_Cmd_WRITE_STATE(MSC_STATUS_LINK_MODE, CLK_MODE__PacketPixel);

	}
	MHL_Clock_Mode(1);
}

void EP957Control_PackedPixelModeOff(void)
{
	is_PackedPixelMode = 0;

	if(ConnectionState == CBUS_LINK_STATE__Connected) {
	
		if(is_PATH_EN)
			MHL_MSC_Cmd_WRITE_STATE(MSC_STATUS_LINK_MODE, CLK_MODE__Normal | PATH_EN);
		else 
			MHL_MSC_Cmd_WRITE_STATE(MSC_STATUS_LINK_MODE, CLK_MODE__Normal);

	}
	MHL_Clock_Mode(0);
}

void EP957Control_Reset(void)
{
	EP_DEV_DBG("Hardware Reset Start\n");
	
	Set_Gpio_Init();
	
	Set_Gpio_Reset(EP957B_GPIO_LOW);
	
	Wait_Reset_Timer(DELAY_1_M_SEC);
	
	Set_Gpio_Reset(EP957B_GPIO_HIGH);
}

//--------------------------------------------------------------------------------------------------
// Private Functions
//--------------------------------------------------------------------------------------------------
void MhlCharge(BOOL chargemode)
{
	if(chargemode) {
		EP_DEV_DBG("1500mA \n");
		MHL_set_Charge_Mode(1);
		//pm8921_set_usb_power_supply_type(POWER_SUPPLY_TYPE_MHL);
		//pm8921_charger_vbus_draw(1500);
	}
	else {
		EP_DEV_DBG("0mA \n");
		MHL_set_Charge_Mode(0);
		//pm8921_set_usb_power_supply_type(POWER_SUPPLY_TYPE_MHL);
		//pm8921_charger_vbus_draw(0);
	}
}

CBUS_LINK_STATE EP957_Read_LINK_STATE(void)
{
	return ConnectionState;
}

void Exchange_DCAP(void)
{
	is_DACP_Change = 0;

	{
		BYTE i;
		for(i=0; i<16; ++i) {
			is_CBUS_OK = MHL_MSC_Cmd_READ_DEVICE_CAP(i, &Temp_Byte[i]);
			if( !is_CBUS_OK ) return;
		}
		for(i=0; i<16; ++i) {
			EP_DEV_DBG("0x%X ", Temp_Byte[i]);
		}
	}

	EP_DEV_DBG("DEV_CAT = 0x%X \n",Temp_Byte[MSC_DEV_CAT]);
	if( ((Temp_Byte[MSC_DEV_CAT] & EP957B_DEV_CAT_DONGLE_SINK) == EP957B_DEV_CAT_DONGLE_SINK) && (ConnectionState > CBUS_LINK_STATE__USB_Mode) ) {
		EP_DEV_DBG("###Exchange_DCAP 1 \n");
		MhlCharge(TRUE);
	}
	else if( ((Temp_Byte[MSC_DEV_CAT] & EP957B_DEV_CAT_SELF_POWERED_DONGLE) == EP957B_DEV_CAT_SELF_POWERED_DONGLE) && (ConnectionState > CBUS_LINK_STATE__USB_Mode) ) {
		EP_DEV_DBG("###Exchange_DCAP 2 \n");
		MhlCharge(TRUE);
	}
	else {
		EP_DEV_DBG("###Exchange_DCAP 3 \n");
		if(ConnectionState > CBUS_LINK_STATE__USB_Mode) {
		MhlCharge(FALSE);
		}
	}
	// Check POW bit from Sink	
	if(Temp_Byte[MSC_DEV_CAT] & POW) {
//		is_VBus_Power = VBUS_Power(0); // VBus Off
	}

	// Check Scratchpad/RAP/RCP support
	if(Temp_Byte[FEATURE_FLAG] & RCP_SUPPORT) {
		EP_DEV_DBG("RCP_SUPPORT = 1\n");
	}
	if(Temp_Byte[FEATURE_FLAG] & RAP_SUPPORT) {
		EP_DEV_DBG("RAP_SUPPORT = 1\n");

		if(is_Source_Ready) {
			MHL_MSC_Cmd_MSC_MSG(MSC_RAP, RAP_CONTENT_ON);
		}	
	}
	if(Temp_Byte[FEATURE_FLAG] & SP_SUPPORT) {
		EP_DEV_DBG("SP_SUPPORT = 1\n"); 

		// Remember the ADOPTER_ID and DEVICE_ID
		//RSP_Adopter_ID[0] = Temp_Byte[ADOPTER_ID_H];
		//RSP_Adopter_ID[1] = Temp_Byte[ADOPTER_ID_L];
	}

	//
	// Add the customer code here ...
	//
	
	
}	
	
void Set_DCAP_Ready(void)
{	
	//
	// At last
	//
	
	// Reset the PATH_EN State
	is_PATH_EN = 0;
	
	// Set DCAP_RDY bit to the other side Status Register
	is_CBUS_OK = MHL_MSC_Cmd_WRITE_STATE(MSC_STATUS_CONNECTED_RDY, DCAP_RDY);
	if( !is_CBUS_OK ) return;
	
	// Set DCAP_CHG bit to the other side Interrupt Register
	is_CBUS_OK = MHL_MSC_Cmd_WRITE_STATE(MSC_RCHANGE_INT, DCAP_CHG);
	if( !is_CBUS_OK ) return;
	
}

void EP957Control_HDCP_Proc1(void)
{
	SMBUS_STATUS status = 0; // modify 130408
	// Copy HDCP registers to Sink
	// AINFO, AN, AKSV
	
	// RQ_RI_EN
	HDMI_RQ_RI_Enable(FALSE);

	// Left Hand Read -> Right Hand Write
	//HDMI_Rx_read_AINFO(Temp_Byte);            // modify 130205
	//Downstream_Rx_write_AINFO(Temp_Byte[0]); // modify 130205

	HDMI_Rx_read_AN(Temp_Byte);
	status = Downstream_Rx_write_AN(Temp_Byte); // modify 130408 (need get communication result)

	HDMI_Rx_read_AKSV(Temp_Byte);
	status = Downstream_Rx_write_AKSV(Temp_Byte); // modify 130408 (need get communication result)

	if(!status) // modify 130408 (for CBUS test, if DDC communication have error, don't enable Ri read) 
	{
		mdelay(110); // delay 100ms for ATC HDCP test  // modify 130205
	    // need modify SOC - HDMI HDCP Read R0 delay time to 200ms
	
		// RQ_RI_EN
		HDMI_RQ_RI_Enable(TRUE);
	
		EP_DEV_DBG("HDCP Propagation Part 1 Done\n");
	}  // modify 130408 
}

void PPGS_RollBack_Wait_Upstream(void)
{
	EP_DEV_DBG("State Rollback: [PPGS_Search_EDID]\n");
	
	// Disable Hot-Plug to the Source
	HDMI_Rx_HotPlug_Clear();

	// Power Down Rx
	HDMI_Rx_Power_Down();

	ReadEDID_TimeCount = 0;
	
	PPG_State = PPGS_Search_EDID;
}

void PPGS_RollBack_Stream(void)
{
	EP_DEV_DBG("State Rollback: [PPGS_Wait_Upstream]\n");

	// Disable RI Auto Update
	HDMI_RQ_RI_Enable(FALSE); //modify 130205

	// Power Down
	MHL_Tx_Power_Down();

	PPG_State = PPGS_Wait_Upstream;
}

void Set_Gpio_Reset(int state)
{
	EP_DEV_DBG("Set state to GPIO  state = %d\n", state);
	
	gpio_set_value(EP957B_RESET_GPIO, state);
}

void Wait_Reset_Timer(int wait_time_m_sec)
{
	unsigned long	wait_time_u_sec = wait_time_m_sec * 1000;
	
	EP_DEV_DBG("Set timer for Reset\n");
	
	usleep_range(wait_time_u_sec, wait_time_u_sec);
}

void Set_Gpio_Init(void)
{
	int status;
	
	EP_DEV_DBG("Init to GPIO\n");

	status = gpio_request(EP957B_RESET_GPIO, "W_RST#");
	if (status < 0)
	{
    	EP_DEV_DBG("Init to GPIO error1 status = %d\n", status);
		return;
	}

	status = gpio_direction_output(EP957B_RESET_GPIO, 1);
	if (status < 0)
	{
    	EP_DEV_DBG("Init to GPIO error2 status = %d\n", status);
		gpio_free(EP957B_RESET_GPIO);
		return;
	}
}
//----------------------------------------------------------------------------------------------------------------------
MODULE_DESCRIPTION("EP957 MHL driver");
MODULE_AUTHOR("Explore Semiconductor <http://www.epmi.com.tw/>");
MODULE_LICENSE("GPL");
