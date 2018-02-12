/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2013 KYOCERA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __OEM_CHARGER_STAND_H
#define __OEM_CHARGER_STAND_H

/**
 * oem_chg_stand_init - charge stand initialize
 *
 */
void oem_chg_stand_init(void *chip);

/**
 * oem_chg_get_oem_stand_detect - return charge stand detect state
 *
 */
int oem_chg_get_oem_stand_detect(void);

/**
 * oem_chg_get_boost_chg_state - return charge stand boost charge state
 *
 */
int oem_chg_get_boost_chg_state(void);

/**
 * oem_chg_stand_timer_work_init - init oem_chg_stand_timer_worker
 *
 */
void oem_chg_stand_timer_work_init(void);
#endif
