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

  File        :  EP957Controller.h

  Description :  Head file of EP957Controller.

\******************************************************************************/

#ifndef EP957CONTROLLER_H
#define EP957CONTROLLER_H

#include "EP957_If.h"

#define EP957C_VERSION_MAJOR      1  // 
#define EP957C_VERSION_MINOR      30 // v1.30

#define LINK_TASK_TIME_BASE	      55 // 80 -> 55 ms to detect CBUS disconnec faster to catch up the test // modify 130218 

#define EP957B_RESET_GPIO	      57
#define EP957B_GPIO_HIGH	      1
#define EP957B_GPIO_LOW	          0
#define DELAY_1_M_SEC	          1

#ifdef CONFIG_EP957B_MHL
#define EP957B_DEV_CAT_DONGLE_SINK		0x11
#define EP957B_DEV_CAT_SELF_POWERED_DONGLE	0x13
#endif

typedef enum {
	CBUS_LINK_STATE__USB_Mode = 0, 
	CBUS_LINK_STATE__1KOHM_Detected,
	CBUS_LINK_STATE__Start_Connect,
	CBUS_LINK_STATE__Check_DCAP,
	CBUS_LINK_STATE__Connected	
} CBUS_LINK_STATE;

typedef enum {
	PPGS_Search_EDID,
	PPGS_Wait_Upstream,
	PPGS_Wait_Authentication,
	PPGS_HDCP_Done
} PPG_STATE;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------

// The initial function that should be called once.
extern void            EP957Control_Initial(void);

// A slow looping task that check the CBUS connection for every LINK_TASK_TIME_BASE(55) ms.
extern CBUS_LINK_STATE EP957Control_Link_Task(void);

// Tis function require a fast looping to handle the EDID and HDCP propagation jobs.
extern PPG_STATE       EP957Control_Propagation_Task(void);

// Get RCP data when Interrupt is detected.
// Polling this function every 55ms is also avaliable.
extern BOOL            EP957Control_RCP_RAP_Read(PBYTE RCP_RAP_Code) ;

// Control Functions
extern void            EP957Control_ContentOn(void);
extern void            EP957Control_ContentOff(void);

extern void            EP957Control_PackedPixelModeOn(void);
extern void            EP957Control_PackedPixelModeOff(void);

extern void            EP957Control_Reset(void);

#ifdef CONFIG_EP957B_MHL
extern void            MhlCharge(BOOL chargemode);
#endif/*CONFIG_EP957B_MHL*/

extern CBUS_LINK_STATE EP957_Read_LINK_STATE(void);

// -----------------------------------------------------------------------------
#endif

