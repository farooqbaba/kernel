/* 
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
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

#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/dnand_cdev_driver.h>
#include <mach/msm_smem.h>
#include <oem-charger_parm.h>
#include <mach/oem_fact.h>

oem_chg_param_charger oem_param_charger = {
	.chg_stop_volt = 0x1162,
	.reserve1_1 = 0xFFFF,
	.reserve1_2 = 0xFFFF,
	.reserve1_3 = 0xFFFF,

	.time_chg  = 60,
	.time_cool_chg = 60,
	.time_chg_pm = 0x0200,
	.time_trkl_pm = 0x0040,
	.reserve2_1 = 0xFFFF,
	.reserve2_2 = 0xFFFF,

	.chg_cool_tmp = 0x0A,
	.chg_warm_tmp = 0x2D,
	.wait_chg_on = 0x34,
	.wait_chg_off = 0x2F,
	.chg_adp_tmp1 = 0x0A,
	.chg_adp_tmp2 = 0x32,
	.chg_adp_tmp3 = 0x28,
	.reserve3_2 = 0xFF,

	.normal_chg = 0x10CC,
	.cool_chg = 0x1068,
	.warm_chg = 0x1068,
	.uim_undete_chg = 0x1068,
	.reserve4_1 = 0xFFFF,
	.reserve4_2 = 0xFFFF,

	.rechg_delta_volt = 0x012C,
	.initial_delta_volt = 0x000A,

	.maint_chg_vin = 0x10CC,
	.reserve6_1 = 0xFFFF,
	.reserve6_2 = 0xFFFF,
	.reserve6_3 = 0xFFFF,

	.maint_wait_chg_on_volt = 0x0F3C,
	.maint_wait_chg_off_volt = 0x0DAC,
	.reserve7_1 = 0xFFFF,
	.reserve7_2 = 0xFFFF,

	.i_chg_norm = 0x07D0,
	.i_chg_cool = 0x04E2,
	.i_chg_warm = 0x04FB,
	.i_chg_finish = 0x0064,
	.reserve8_1 = 0xFFFF,
	.reserve8_2 = 0xFFFF,

	.vdd_max_inc_mv = 0x0050,
	.rconn_mohm = 0x0001,
	.reserve9_3 = 0xFFFF,
	.reserve9_4 = 0xFFFF,

	.maint_wait_chg_on_time = 0x0028,
	.maint_wait_chg_off_time = 0x0028,

	.i_chg_adp_chk = 0x03E8,
	.reserve11_1 = 0xFFFF,

	.z_chg_adp_chk = 0x0320,
	.reserve12_1 = 0xFFFF,

	.chg_cam_tmp_off = 0x37,
	.chg_cam_tmp_on = 0x34,
	.chg_inte_cam_on = 0x33,
	.chg_inte_cam_off = 0x30,
	.chg_phone_tmp_off = 0x3C,
	.chg_phone_tmp_on = 0x39,
	.chg_inte_phone_on = 0x37,
	.chg_inte_phone_off = 0x35,
	.chg_adp_tmp_delta = 0x00,
	.reserve13_1 = 0xFF,
	.reserve13_2 = 0xFF,
	.reserve13_3 = 0xFF,

	.reserve14_1 = 0xFFFF,
	.reserve14_2 = 0xFFFF,

	.chg_disable_test = 0xFF,
	.reserve15_1 = 0xFF,
	.reserve15_2 = 0xFF,
	.reserve15_3 = 0xFF
};

oem_chg_param_hkadc oem_param_hkadc = {
	.cal_vbat1 = 0x0010C8E0,
	.cal_vbat2 = 0x00155CC0,

	.reserve2_1 = 0xFFFF,
	.reserve2_2 = 0xFFFF,

	.reserve3_1 = 0xFF,
	.reserve3_2 = 0xFF,
	.reserve3_3 = 0xFF,
	.reserve3_4 = 0xFF
};

oem_chg_param_bms oem_param_bms = {
	.temp_normal_thresh_pow_off = 0x0D16,
	.temp_normal_thresh_low_batt = 0x0D7A,
	.temp_low_thresh_pow_off = 0x0DDE,
	.temp_low_thresh_low_batt = 0x0E42,
	.bat_alarm5 = 0x0000,
	.bat_alarm6 = 0x0000,

	.soc_rate_i_0 = 0x64,
	.soc_rate_i_1 = 0x5A,
	.soc_rate_i_2 = 0x55,
	.reserve2_1 = 0xFF,

	.reserve3_1 = 0xFFFF,
	.reserve3_2 = 0xFFFF,

	.bms_dummy_soc_test = 0xFF,
	.bms_dummy_soc = 0xFF,
	.reserve4_1 = 0xFF,
	.reserve4_2 = 0xFF
};

oem_chg_param_share oem_param_share = {
	.factory_mode_1 = 0x00,
	.reserve1_2 = 0xFF,
	.reserve1_3 = 0xFF,
	.reserve1_4 = 0xFF,
	.reserve1_5 = 0xFF,
	.reserve1_6 = 0xFF,
	.reserve1_7 = 0xFF,
	.reserve1_8 = 0xFF
};

oem_chg_param_cycles oem_param_cycles = {
	.last_chargecycles = 0x0000,
	.last_charge_increase = 0x00,
	.batt_deteriorationstatus = 0x00000000
};

typedef struct {
	oem_chg_param_charger	oem_chg_param_charger;
	oem_chg_param_hkadc	oem_chg_param_hkadc;
	oem_chg_param_bms	oem_chg_param_bms;
	oem_chg_param_share	oem_chg_param_share;
	oem_chg_param_cycles	oem_chg_param_cycles;
}oem_chg_param;

uint8_t oem_cmp_zero_flag = 0;
static void oem_param_charger_init(oem_chg_param *ptr)
{

	if (ptr == NULL){
		pr_err("chg param charger read error.\n");
		return;
	}

	SET_CHG_PARAM_MINMAX(0xFFFF, 0, 65534, charger, chg_stop_volt);

	SET_CHG_PARAM(0xFFFF, charger, time_chg);
	SET_CHG_PARAM(0xFFFF, charger, time_cool_chg);
	SET_CHG_PARAM_MINMAX(0xFFFF, 4, 512, charger, time_chg_pm);
	SET_CHG_PARAM_MINMAX(0xFFFF, 1, 64, charger, time_trkl_pm);

	SET_CHG_PARAM(0xFF, charger, chg_cool_tmp);
	SET_CHG_PARAM(0xFF, charger, chg_warm_tmp);
	SET_CHG_PARAM(0xFF, charger, wait_chg_on);
	SET_CHG_PARAM(0xFF, charger, wait_chg_off);
	SET_CHG_PARAM_MINMAX(0xFF, 0, 254, charger, chg_adp_tmp1);
	SET_CHG_PARAM_MINMAX(0xFF, 0, 254, charger, chg_adp_tmp2);
	SET_CHG_PARAM_MINMAX(0xFF, 0, 254, charger, chg_adp_tmp3);

	SET_CHG_PARAM_MINMAX(0xFFFF, 3400, 4500, charger, normal_chg);
	SET_CHG_PARAM_MINMAX(0xFFFF, 3400, 4500, charger, cool_chg);
	SET_CHG_PARAM_MINMAX(0xFFFF, 3400, 4500, charger, warm_chg);
	SET_CHG_PARAM_MINMAX(0xFFFF, 3400, 4500, charger, uim_undete_chg);

	SET_CHG_PARAM(0xFFFF, charger, rechg_delta_volt);
	SET_CHG_PARAM(0xFFFF, charger, initial_delta_volt);

	SET_CHG_PARAM_MINMAX(0xFFFF, 4300, 6500, charger, maint_chg_vin);

	SET_CHG_PARAM_MINMAX(0xFFFF, 0, 65534, charger, maint_wait_chg_on_volt);
	SET_CHG_PARAM_MINMAX(0xFFFF, 0, 65534, charger, maint_wait_chg_off_volt);

	SET_CHG_PARAM_MINMAX(0xFFFF, 325, 2000, charger, i_chg_norm);
	SET_CHG_PARAM_MINMAX(0xFFFF, 325, 2000, charger, i_chg_cool);
	SET_CHG_PARAM_MINMAX(0xFFFF, 325, 2000, charger, i_chg_warm);
	SET_CHG_PARAM_MINMAX(0xFFFF, 50, 200, charger, i_chg_finish);

	SET_CHG_PARAM_MINMAX(0xFFFF, 0, 400, charger, vdd_max_inc_mv);
	SET_CHG_PARAM_MINMAX(0xFFFF, 0, 100, charger, rconn_mohm);

	SET_CHG_PARAM_MINMAX(0xFFFF, 0, 600, charger, maint_wait_chg_on_time);
	SET_CHG_PARAM_MINMAX(0xFFFF, 0, 600, charger, maint_wait_chg_off_time);

	SET_CHG_PARAM_MINMAX(0xFFFF, 0, 65534, charger, i_chg_adp_chk);

	SET_CHG_PARAM(0xFFFF, charger, z_chg_adp_chk);

	SET_CHG_PARAM_INIT(oem_cmp_zero_flag, charger, chg_cam_tmp_off);
	SET_CHG_PARAM_INIT(oem_cmp_zero_flag, charger, chg_cam_tmp_on);
	SET_CHG_PARAM_INIT(oem_cmp_zero_flag, charger, chg_inte_cam_on);
	SET_CHG_PARAM_INIT(oem_cmp_zero_flag, charger, chg_inte_cam_off);
	SET_CHG_PARAM_INIT(oem_cmp_zero_flag, charger, chg_phone_tmp_off);
	SET_CHG_PARAM_INIT(oem_cmp_zero_flag, charger, chg_phone_tmp_on);
	SET_CHG_PARAM_INIT(oem_cmp_zero_flag, charger, chg_inte_phone_on);
	SET_CHG_PARAM_INIT(oem_cmp_zero_flag, charger, chg_inte_phone_off);
	SET_CHG_PARAM_INIT(oem_cmp_zero_flag, charger, chg_adp_tmp_delta);

	SET_CHG_PARAM_TEST_FLG(0xFF, charger, chg_disable_test);
}

static void oem_param_charger_cal_init(oem_chg_param *ptr)
{

	if (ptr == NULL){
		pr_err("chg param charger read error.\n");
		return;
	}

	SET_CHG_PARAM_MINMAX(0xFFFF, 3400, 4500, charger, normal_chg);
	SET_CHG_PARAM_MINMAX(0xFFFF, 3400, 4500, charger, cool_chg);
	SET_CHG_PARAM_MINMAX(0xFFFF, 3400, 4500, charger, warm_chg);
	SET_CHG_PARAM_MINMAX(0xFFFF, 3400, 4500, charger, uim_undete_chg);
}

static void oem_param_hkadc_init(oem_chg_param *ptr)
{
	if (ptr == NULL){
		pr_err("chg param hkadc read error.\n");
		return;
	}

	SET_CHG_PARAM(0xFFFFFFFF, hkadc, cal_vbat1);
	SET_CHG_PARAM(0xFFFFFFFF, hkadc, cal_vbat2);
}

static void oem_param_hkadc_cal_init(oem_chg_param *ptr)
{
	if (ptr == NULL){
		pr_err("chg param hkadc read error.\n");
		return;
	}

	SET_CHG_PARAM(0xFFFFFFFF, hkadc, cal_vbat1);
	SET_CHG_PARAM(0xFFFFFFFF, hkadc, cal_vbat2);
}

static void oem_param_bms_init(oem_chg_param *ptr)
{
	if (ptr == NULL){
		pr_err("chg param bms read error.\n");
		return;
	}

#ifdef OEM_BMS_USE_NV
	SET_CHG_PARAM(0xFFFF, 2500, 5675, bms, temp_normal_thresh_pow_off);
	SET_CHG_PARAM(0xFFFF, 2500, 5675, bms, temp_normal_thresh_low_batt);
	SET_CHG_PARAM(0xFFFF, 2500, 5675, bms, temp_low_thresh_pow_off);
	SET_CHG_PARAM(0xFFFF, 2500, 5675, bms, temp_low_thresh_low_batt);

	SET_CHG_PARAM(0xFF, 0, 100, bms, soc_rate_i_0);
	SET_CHG_PARAM(0xFF, 0, 100, bms, soc_rate_i_1);
	SET_CHG_PARAM(0xFF, 0, 100, bms, soc_rate_i_2);
#endif

	SET_CHG_PARAM_TEST_FLG(0xFF, bms, bms_dummy_soc_test);
	SET_CHG_PARAM_TEST_MINMAX(0xFF, 1, 100, bms, bms_dummy_soc);
}

static void oem_param_share_init(oem_chg_param *ptr)
{
	if (ptr == NULL){
		pr_err("chg param share read error.\n");
		return;
	}

	SET_CHG_PARAM(0xFF, share, factory_mode_1);
}

static void oem_param_cycles_init(oem_chg_param *ptr)
{
	if (ptr == NULL){
		pr_err("chg param cycles read error.\n");
		return;
	}

	memcpy(&oem_param_cycles, &ptr->oem_chg_param_cycles,
				sizeof(oem_chg_param_cycles));
}

void oem_param_cycles_backup(uint16_t charge_cycles, u8 charge_increase, int batt_deterioration_status)
{
	int ret;

	oem_param_cycles.last_chargecycles = charge_cycles;
	oem_param_cycles.last_charge_increase = charge_increase;
	oem_param_cycles.batt_deteriorationstatus = batt_deterioration_status;

	pr_debug("last_chargecycles[%d], last_charge_increase[%d], batt_deteriorationstatus[%d]\n",
				oem_param_cycles.last_chargecycles,
				oem_param_cycles.last_charge_increase,
				oem_param_cycles.batt_deteriorationstatus);

	ret = kdnand_id_write(23, 0, (uint8_t *)&oem_param_cycles, sizeof(oem_param_cycles));

	if(ret)
		pr_err("kdnand_id_write error. ret = %d\n", ret);

	return;
}

static int init_flag = 0;
void oem_chg_param_init(void)
{
	uint32_t *smem_ptr = NULL;
	uint32_t *cmp_ptr  = NULL;

	if (init_flag) {
		return;
	}

	smem_ptr = (uint32_t *)kc_smem_alloc(SMEM_CHG_PARAM, (CHG_PARAM_SIZE + SMEM_CHG_PARAM_CYCLE));
	if (smem_ptr == NULL) {
		pr_err("chg param read error.\n");
		init_flag = 1;
		return;
	}

	cmp_ptr = kmalloc((CHG_PARAM_SIZE - SMEM_CHG_PARAM_SHARE), GFP_KERNEL);
	if (cmp_ptr == NULL) {
		pr_err("kmalloc error.\n");
		init_flag = 1;
		return;
	}

	memset(cmp_ptr, 0x00, (CHG_PARAM_SIZE - SMEM_CHG_PARAM_SHARE));
	if (0 == memcmp(smem_ptr, cmp_ptr, (CHG_PARAM_SIZE - SMEM_CHG_PARAM_SHARE))) {
		pr_err("smem data all '0'\n");
		memset(smem_ptr, 0xFF, (CHG_PARAM_SIZE - SMEM_CHG_PARAM_SHARE));
		oem_cmp_zero_flag = 1;
	}

	if(oem_fact_get_option_bit(OEM_FACT_OPTION_ITEM_01, 5)) {
		oem_param_charger_init((oem_chg_param*)smem_ptr);
		oem_param_hkadc_init((oem_chg_param*)smem_ptr);
		oem_param_bms_init((oem_chg_param*)smem_ptr);
		oem_param_share_init((oem_chg_param*)smem_ptr);
		oem_param_cycles_init((oem_chg_param*)smem_ptr);
	}
	
	else {
		oem_param_charger_cal_init((oem_chg_param*)smem_ptr);
		oem_param_hkadc_cal_init((oem_chg_param*)smem_ptr);
		oem_param_bms_init((oem_chg_param*)smem_ptr);
		oem_param_cycles_init((oem_chg_param*)smem_ptr);
	}

	kfree(cmp_ptr);
	init_flag = 1;

}
