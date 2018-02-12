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
#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/module.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/power_supply.h>
#include <oem-charger.h>
#include <oem-charger_stand.h>
#include <oem-charger_parm.h>
#include <oem-hkadc.h>
#include <mach/oem_fact.h>

DEFINE_SPINLOCK(oem_charger_master_lock);
#define MASTERDATA_SPINLOCK_INIT()	spin_lock_init(&oem_charger_master_lock);

#define	MASTERDATA_LOCK()		\
	{				\
	unsigned long oem_charger_irq_flag;	\
	spin_lock_irqsave(&oem_charger_master_lock, oem_charger_irq_flag);

#define	MASTERDATA_UNLOCK()						\
	spin_unlock_irqrestore(&oem_charger_master_lock, oem_charger_irq_flag);	\
	}

//#define FEATURE_CHG_CHG_DEBUG
#ifdef FEATURE_CHG_CHG_DEBUG
#define CHG_CHG_ERR       pr_err
#define CHG_CHG_DEBUG     pr_err
#else
#define CHG_CHG_ERR       pr_err
#define CHG_CHG_DEBUG     pr_debug
#endif

static int oem_charger_connect_state = POWER_SUPPLY_OEM_CONNECT_NONE;
static atomic_t is_chargermonit_initialized=ATOMIC_INIT(0);

enum oem_charge_vbatt_ov_state_ {
	OEM_CHARGE_VBATT_STATE_OFF= 0,
	OEM_CHARGE_VBATT_STATE_ON,
	OEM_CHARGE_VBATT_STATE_OVOFF
};

static void *the_chip;

static int oem_charge_vbatt_ov_state = OEM_CHARGE_VBATT_STATE_OFF;
static int oem_charge_vbatt_ov_state_now = OEM_CHARGE_VBATT_STATE_OFF;
static struct power_supply *oem_charge_usb_psy = NULL;

static int oem_charger_get_usb_property(enum power_supply_property psp)
{
	union power_supply_propval ret = {0,};

	if(oem_charge_usb_psy)
		oem_charge_usb_psy->get_property(oem_charge_usb_psy, psp, &ret);

	CHG_CHG_DEBUG("Get USB Property psp = %d val = %d\n", psp, ret.intval);

	return ret.intval;
}

static void oem_charger_master_data_write(enum power_supply_property psp, int val)
{
	int* masterdata;

	switch(psp){
	case POWER_SUPPLY_PROP_OEM_CHARGE_CONNECT_STATE:
		masterdata
		 = &oem_charger_connect_state;
		break;

	default:
		masterdata = NULL;
		break;
	}

	if (masterdata != NULL){
		MASTERDATA_LOCK();
		*masterdata = val;
		MASTERDATA_UNLOCK();
	}

}

static int oem_charger_master_data_read(enum power_supply_property psp, int *intval)
{
	int ret = 0;
	int initialized;
	int* masterdata;

	initialized = atomic_read(&is_chargermonit_initialized);

	if (!initialized) {
		CHG_CHG_ERR("called before init\n");
		*intval = 0;
		return -EAGAIN;
	}

	switch(psp){
	case POWER_SUPPLY_PROP_OEM_CHARGE_CONNECT_STATE:
		masterdata = &oem_charger_connect_state;
		break;

	default:
		masterdata = NULL;
		break;
	}

	if (masterdata != NULL){
		MASTERDATA_LOCK();
		*intval = *masterdata;
		MASTERDATA_UNLOCK();
	}else{
		CHG_CHG_DEBUG("Out of range psp %d \n", psp);
		ret = -EINVAL;
	}

	return ret;

}

static int oem_charger_determine_connect_state(void)
{
	if (oem_chg_get_oem_stand_detect()){
		CHG_CHG_DEBUG("charger connect %d \n", oem_chg_get_oem_stand_detect());
		if (oem_chg_get_boost_chg_state()){
			return POWER_SUPPLY_OEM_CONNECT_CHARGE_STAND_1;
		}else{
			return POWER_SUPPLY_OEM_CONNECT_CHARGE_STAND_2;
		}
		
	}else{
		CHG_CHG_DEBUG("charger disconnect %d \n", oem_charger_get_usb_property(POWER_SUPPLY_PROP_PRESENT));
		if (oem_charger_get_usb_property(POWER_SUPPLY_PROP_PRESENT)){
			return POWER_SUPPLY_OEM_CONNECT_USB;
		}else{
			return POWER_SUPPLY_OEM_CONNECT_NONE;
		}
	}
}

static int oem_charger_init_connect_state(void)
{
	int state;

	state = oem_charger_determine_connect_state();
	CHG_CHG_DEBUG("init charger connect state = %d \n", state);

	if (-1 < state) {
		oem_charger_connect_state = state;
	} else {
		return state;
	}

	return 0;
}

int oem_charger_get_connect_state(void)
{
	int state;

	state = oem_charger_determine_connect_state();
	CHG_CHG_DEBUG("charger connect state = %d \n", state);

	if (-1 < state) {
		oem_charger_master_data_write(POWER_SUPPLY_PROP_OEM_CHARGE_CONNECT_STATE, state);
	} else {
		return state;
	}

	return 0;
}
EXPORT_SYMBOL(oem_charger_get_connect_state);

int oem_charger_pm_batt_power_get_property(enum power_supply_property psp, int *intval)
{
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_OEM_CHARGE_CONNECT_STATE:
		ret = oem_charger_master_data_read(psp, intval);
		CHG_CHG_DEBUG("charger connect state = %d\n", *intval);
		break;

	default:
		ret = -EINVAL;
	}
	return ret;
}
EXPORT_SYMBOL(oem_charger_pm_batt_power_get_property);

void oem_chargermonit_init(void *chip)
{
	oem_charger_init_connect_state();
	atomic_set(&is_chargermonit_initialized, 1);
	the_chip = chip;
	oem_charge_usb_psy = power_supply_get_by_name("usb");
	CHG_CHG_DEBUG("oem_chargermonit_init()\n");

}
EXPORT_SYMBOL(oem_chargermonit_init);

void oem_chargermonit_exit(void)
{
	atomic_set(&is_chargermonit_initialized, 0);
}
EXPORT_SYMBOL(oem_chargermonit_exit);


static struct work_struct	chg_vbatt_ov_work;
void oem_chg_vbatt_ov_work_init(void)
{
	INIT_WORK(&chg_vbatt_ov_work, chg_vbatt_ov_worker);
}
EXPORT_SYMBOL(oem_chg_vbatt_ov_work_init);


void chg_vbatt_ov_worker(struct work_struct *work)
{
	bool check = true;
	int vbatt_uv = 0;
	int ret;


	CHG_CHG_DEBUG("oem_charge_vbatt_ov_state-1 %d, oem_charge_vbatt_ov_state_now %d\n"
			, oem_charge_vbatt_ov_state, oem_charge_vbatt_ov_state_now);

	switch(oem_charge_vbatt_ov_state){
	case OEM_CHARGE_VBATT_STATE_OFF:

		if (oem_charge_vbatt_ov_state_now == OEM_CHARGE_VBATT_STATE_OVOFF){
			CHG_CHG_DEBUG("charger removed\n");
			
			ret = oem_chg_buck_ctrl(the_chip, false);
			if (ret) {
				CHG_CHG_ERR("error buck converter setting value %d\n", ret);
			}
		}
		oem_charge_vbatt_ov_state_now = OEM_CHARGE_VBATT_STATE_OFF;

		break;

	case OEM_CHARGE_VBATT_STATE_ON:
		do {
			if (POWER_SUPPLY_OEM_CONNECT_NONE == oem_charger_determine_connect_state()){
				check = false;
				CHG_CHG_DEBUG("battery removed.\n");
				break;
			}
			
			vbatt_uv = oem_get_vbatt_value();
			
			if ((oem_param_charger.chg_stop_volt * 1000) > vbatt_uv){
				check = false;
				CHG_CHG_DEBUG("vbatt normal %d.\n", vbatt_uv);
				break;
			}
		} while(0);
		
		if (check) {
			CHG_CHG_ERR("vbatt ov detected\n");
			if (oem_charge_vbatt_ov_state_now != OEM_CHARGE_VBATT_STATE_OVOFF){
				CHG_CHG_ERR("chg off\n");
				
				oem_charge_vbatt_ov_state = OEM_CHARGE_VBATT_STATE_OVOFF;
				oem_charge_vbatt_ov_state_now = OEM_CHARGE_VBATT_STATE_OVOFF;
				ret = oem_chg_buck_ctrl(the_chip, true);
				if (ret) {
					CHG_CHG_ERR("error buck converter setting value %d\n", ret);
				}
			}
		}else{
			oem_charge_vbatt_ov_state_now = OEM_CHARGE_VBATT_STATE_ON;
		}

		break;

	case OEM_CHARGE_VBATT_STATE_OVOFF:

		if (POWER_SUPPLY_OEM_CONNECT_NONE == oem_charger_determine_connect_state()){
			CHG_CHG_DEBUG("battery removed.\n");
			
			oem_charge_vbatt_ov_state = OEM_CHARGE_VBATT_STATE_ON;
			oem_charge_vbatt_ov_state_now = OEM_CHARGE_VBATT_STATE_ON;
			ret = oem_chg_buck_ctrl(the_chip, false);
			if (ret) {
				CHG_CHG_ERR("error buck converter setting value %d\n", ret);
			}
			break;
		}

		break;

	default:
		CHG_CHG_DEBUG("oem_charge_vbatt_ov_state Illegal state. %d\n", oem_charge_vbatt_ov_state);
		break;

	}

	CHG_CHG_DEBUG("oem_charge_vbatt_ov_state-2 %d, oem_charge_vbatt_ov_state_now %d\n", oem_charge_vbatt_ov_state, oem_charge_vbatt_ov_state_now);
}
EXPORT_SYMBOL(chg_vbatt_ov_worker);

void oem_chg_vbatt_ov_check_charger(void)
{
    if (POWER_SUPPLY_OEM_CONNECT_NONE == oem_charger_determine_connect_state()) {
		oem_charge_vbatt_ov_state = OEM_CHARGE_VBATT_STATE_OFF;
	}else{
		oem_charge_vbatt_ov_state = OEM_CHARGE_VBATT_STATE_ON;
	}
	schedule_work(&chg_vbatt_ov_work);
}
EXPORT_SYMBOL(oem_chg_vbatt_ov_check_charger);

void oem_chg_check_vbatt_ov(void)
{
	int vbatt_uv = 0;

    if (POWER_SUPPLY_OEM_CONNECT_NONE != oem_charger_determine_connect_state()) {

		vbatt_uv = oem_get_vbatt_value();
		CHG_CHG_DEBUG("oem_chg_check_vbatt_ov. vbatt_uv = %d\n", vbatt_uv);
		
		if ((oem_param_charger.chg_stop_volt * 1000) <= vbatt_uv){
			CHG_CHG_DEBUG("vbat is over the threshold\n");
			schedule_work(&chg_vbatt_ov_work);
		}
	}
}
EXPORT_SYMBOL(oem_chg_check_vbatt_ov);

bool oem_is_chg_vbatt_ov(void)
{
	if (POWER_SUPPLY_OEM_CONNECT_NONE != oem_charger_determine_connect_state()) {
		if (oem_charge_vbatt_ov_state_now == OEM_CHARGE_VBATT_STATE_OVOFF) {
			return true;
		}
	}
	return false;
}
EXPORT_SYMBOL(oem_is_chg_vbatt_ov);


static uint32_t *uim_status_smem_ptr = NULL;
void oem_uim_smem_init(void)
{
	uim_status_smem_ptr = (uint32_t *)kc_smem_alloc(SMEM_UICC_INFO, 1);
	if (NULL == uim_status_smem_ptr) {
		CHG_CHG_ERR("smem uicc read error.\n");
		return;
	}
}
EXPORT_SYMBOL(oem_uim_smem_init);

static bool is_oem_uim_status(void)
{
	if (NULL == uim_status_smem_ptr)
	{
		CHG_CHG_ERR("uim is detatched\n");
		return false;
	}

	if ((0 == *uim_status_smem_ptr) || (1 == *uim_status_smem_ptr))
	{
		return true;
	}
	return false;
}

bool oem_is_uim_dete(void)
{
	if (oem_fact_get_option_bit(OEM_FACT_OPTION_ITEM_01, 4)) {
		return true;
	}
	
	return is_oem_uim_status();
}
EXPORT_SYMBOL(oem_is_uim_dete);

