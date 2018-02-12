/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 * (C) 2013 KYOCERA Corporation
 */
/* 
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
#ifndef _KC_BOARD_H_
#define _KC_BOARD_H_

typedef enum {
    OEM_BOARD0_TYPE     = 0,
    OEM_BOARD1_TYPE     = 1,
    OEM_BOARD2_TYPE     = 2,
    OEM_BOARD3_TYPE     = 3,
    OEM_BOARD4_TYPE     = 4,
    OEM_BOARD5_TYPE     = 5,
    OEM_BOARD6_TYPE     = 6,
    OEM_BOARD7_TYPE     = 7,
} oem_board_type;

void OEM_board_judgement(void);

oem_board_type OEM_get_board(void);

#endif /* _KC_BOARD_H_ */
