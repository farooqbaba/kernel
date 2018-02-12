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
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <oem-charger_parm.h>
#include <oem-charger_stand.h>
#include <oem-charger.h>

#define CHG_STAND_MONITOR_MS         1000
#define THRESHOLD_VBATT_UV_LO        3800000
#define THRESHOLD_VBATT_UV_HI        4200000
#define INTERVAL_STAND_CHECK_FIRST   (5*60/(CHG_STAND_MONITOR_MS/1000))
#define INTERVAL_STAND_CHG_START     (30/(CHG_STAND_MONITOR_MS/1000))
#define IBATMAX_500                  500
#define IBATMAX_1000                 1000
#define GPIO_BAT_OVP_RST_N           9
#define GPIO_CHG_TBATT2_ON           101
#define GPIO_CHG_TBATT2              80
#define GPIO_CHG_DET_N               64
#define GPIO_BAT_OVP_N               79
#define GPIO_INPUT_LO                0
#define GPIO_INPUT_HI                1
#define NOT_DETECT                   0
#define DETECT_LO_PRE_FIX            1
#define DETECT_LO_FIX                2
#define DETECT_STAND                 1
#define THRESHOLD_IBAT_STAND_CHARGE_STOP_LO (-1500000)
#define THRESHOLD_IBAT_STAND_CHARGE_STOP_HI (-3000000)
#define THRESHOLD_DCIN               6000000
#define THRESHOLD_DCIN_HI (4400 * 1000)
#define THRESHOLD_DCIN_LO (1000 * 1000)
#define THERMAL_CHG_STOP_LVL         3

static void *the_chip;

static int oem_stand_detect  = 0;
static uint32_t oem_charge_stand_flag = 0;
static int oem_stand_detect_time_counter = 0;
static int oem_chg_stand_charge_timer = 0;

static int oem_tbatt2_detect = 0;
static int oem_det_n_detect  = 0;
struct delayed_work oem_chg_tbatt2_work;
struct delayed_work oem_chg_det_n_work;
struct delayed_work oem_bat_ovp_n_work;
struct delayed_work oem_chg_stand_monitor_work;
struct wake_lock charging_stand_wake_lock;

irqreturn_t oem_chg_tbatt2_isr(int irq, void *ptr);
irqreturn_t oem_chg_det_n_isr(int irq, void *ptr);
static void oem_stand_check(void);
static void oem_chg_stand_check_control(void);

static bool from_oem_chg_tbatt2_isr = false;
static bool oem_repeat_tbatt2 = false;

static bool from_oem_chg_det_n_isr = false;
static bool oem_repeat_det_n = false;

static bool oem_chg_ovp_n_detect = false;

static struct power_supply *oem_chg_stand_bms_psy = NULL;
static struct power_supply *oem_chg_stand_battery_psy = NULL;

extern int oem_chg_ibatmax_set(void *_chip, int chg_current);
extern void oem_chg_set_appropriate_battery_current(void *_chip);
extern int oem_chg_buck_ctrl(void *_chip, int disable);
extern int oem_chg_is_dc_chg_check(void *_chip);

static bool is_oem_chg_stand_timer_counting = false;
static void oem_chg_stand_timer_satrt(void);
static void oem_chg_stand_timer_stop(void);
static struct delayed_work	oem_chg_stand_timer_done_work;
static void oem_chg_stand_timer_done_worker(struct work_struct *work);

//#define FEATURE_CHG_STAND_DEBUG
#ifdef FEATURE_CHG_STAND_DEBUG
#define CHG_STAND_ERR       pr_err
#define CHG_STAND_DEBUG     pr_err
#else
#define CHG_STAND_ERR       pr_err
#define CHG_STAND_DEBUG     pr_debug
#endif

static void oem_chg_stand_monitor(struct work_struct *work)
{
	CHG_STAND_DEBUG("***** Run Charge Stand Monitor *****\n");
	oem_stand_check();
	oem_chg_stand_check_control();

		schedule_delayed_work(&oem_chg_stand_monitor_work,
					round_jiffies_relative(msecs_to_jiffies(CHG_STAND_MONITOR_MS)));
	}

static void oem_chg_start_stand_monitor(void)
{
		CHG_STAND_DEBUG("##### Start Charge Stand Monitor #####\n");
		schedule_delayed_work(&oem_chg_stand_monitor_work,
					round_jiffies_relative(msecs_to_jiffies(CHG_STAND_MONITOR_MS)));
	}

static void oem_stand_check(void)
{
	struct qpnp_vadc_result result;
	int ret;

	if (((DETECT_LO_FIX == oem_tbatt2_detect) && (1 != oem_charge_stand_flag)) &&
		(1 == oem_chg_is_dc_chg_check(the_chip)) && 
		(0 == oem_stand_detect_time_counter)) {
		ret = qpnp_vadc_read(DCIN, &result);
		if (ret) {
			CHG_STAND_ERR("Failed to reading dcin, rc = %d\n", ret);
			return;
		}

		if (((THRESHOLD_DCIN_HI) >= result.physical) &&
		    ((THRESHOLD_DCIN_LO) <= result.physical)) {
			//oem_pm8921_disable_source_current(true);
			oem_chg_buck_ctrl(the_chip, 1);
			oem_stand_detect_time_counter = 1;
		}
	}

	if (1 <= oem_stand_detect_time_counter) {
		oem_stand_detect_time_counter ++;
		if (4 == oem_stand_detect_time_counter) {
			//oem_pm8921_disable_source_current(false);
			oem_chg_buck_ctrl(the_chip, 0);
			oem_stand_detect_time_counter = 0;
		}
	}
}

int oem_chg_stand_on(void)
{
	CHG_STAND_DEBUG("Enter CHG Stand Boost Charge!!\n");

	gpio_tlmm_config(GPIO_CFG(GPIO_CHG_TBATT2_ON, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

	gpio_set_value(GPIO_BAT_OVP_RST_N, 1);

	mdelay(100);

	gpio_set_value(GPIO_BAT_OVP_RST_N, 0);

	gpio_set_value(GPIO_CHG_TBATT2_ON, 1);

	mdelay(100);

	oem_charge_stand_flag = 1;

	enable_irq(gpio_to_irq(GPIO_BAT_OVP_N));
	oem_chg_ovp_n_detect = false;

	oem_chg_stand_timer_satrt();

	return 0;
}

int oem_chg_stand_off(void)
{
	mdelay(100);

	gpio_set_value(GPIO_CHG_TBATT2_ON, 0);

	oem_charge_stand_flag = 0;

	oem_chg_stand_charge_timer = INTERVAL_STAND_CHECK_FIRST;

	oem_chg_stand_timer_stop();

	CHG_STAND_DEBUG("Finish CHG Stand Boost Charge!!\n");
	return 0;
}

static int oem_chg_get_bms_property(enum power_supply_property psp)
{
	union power_supply_propval ret = {0,};

	if(!oem_chg_stand_bms_psy)
		oem_chg_stand_bms_psy = power_supply_get_by_name("bms");

	if(oem_chg_stand_bms_psy)
		oem_chg_stand_bms_psy->get_property(oem_chg_stand_bms_psy, psp, &ret);

	CHG_STAND_DEBUG("Get BMS Property psp = %d val = %d\n", psp, ret.intval);

	return ret.intval;
}

static int oem_chg_get_battery_property(enum power_supply_property psp)
{
	union power_supply_propval ret = {0,};

	if(!oem_chg_stand_battery_psy)
		oem_chg_stand_battery_psy = power_supply_get_by_name("battery");

	if(oem_chg_stand_battery_psy)
		oem_chg_stand_battery_psy->get_property(oem_chg_stand_battery_psy, psp, &ret);

	CHG_STAND_DEBUG("Get BATTERY Property psp = %d val = %d\n", psp, ret.intval);

	return ret.intval;
}


static bool oem_chg_ibatt_check(void)
{
	bool ibatt = true;
	int get_ibatt = 0;

	get_ibatt = oem_chg_get_bms_property(POWER_SUPPLY_PROP_CURRENT_NOW);
	if ((oem_param_charger.i_chg_adp_chk * -1000) < get_ibatt) {
		CHG_STAND_ERR("ibatt error ibatt = %duA oem_param_charger.i_chg_adp_chk = %d \n", get_ibatt, oem_param_charger.i_chg_adp_chk);
		ibatt = false;
	}

	CHG_STAND_DEBUG("ret = %d ibatt = %d\n", ibatt, get_ibatt);
	return ibatt;
}

static bool oem_chg_dcin_check(void)
{
	struct qpnp_vadc_result result;
	int rc = 0;
	bool dcin = true;

	rc = qpnp_vadc_read(DCIN, &result);
	if (rc) {
		CHG_STAND_ERR("Failed to reading dcin, rc = %d\n", rc);
	}
	CHG_STAND_DEBUG("dcin(6x) : %lld\n", result.physical);
	if (THRESHOLD_DCIN < result.physical) {
		dcin = false;
	}

	return dcin;
}

static bool oem_chg_dcr_check(void)
{
	int rc = 0;
	int vbatt_1 = 0, vbatt_2 = 0;
	int ibatt_1 = 0, ibatt_2 = 0;
	int calc_dcr = 0;
	bool dcr = true;

	rc = oem_chg_ibatmax_set(the_chip, IBATMAX_500);
	if (rc) {
		CHG_STAND_ERR("Failed to set max current to 400 rc=%d [500mA]\n", rc);
	}
	mdelay(100);

	vbatt_1 = oem_chg_get_battery_property(POWER_SUPPLY_PROP_VOLTAGE_NOW);
	ibatt_1 = oem_chg_get_bms_property(POWER_SUPPLY_PROP_CURRENT_NOW) * (-1);

	rc = oem_chg_ibatmax_set(the_chip, IBATMAX_1000);
	if (rc) {
		CHG_STAND_ERR("Failed to set max current to 400 rc=%d [1000mA]\n", rc);
	}
	mdelay(100);

	vbatt_2 = oem_chg_get_battery_property(POWER_SUPPLY_PROP_VOLTAGE_NOW);
	ibatt_2 = oem_chg_get_bms_property(POWER_SUPPLY_PROP_CURRENT_NOW) * (-1);

	if (ibatt_1 != ibatt_2) {
		calc_dcr = ((vbatt_2 - vbatt_1) * 1000) / (ibatt_2 - ibatt_1);
	} else {
		calc_dcr = 0;
	}

	if (oem_param_charger.z_chg_adp_chk < calc_dcr) {
		dcr = false;
	}

	oem_chg_set_appropriate_battery_current(the_chip);

	return dcr;
}

static void oem_chg_stand_check_control(void)
{
	bool check = true;
	int get_ibatt = 0;
//	int fsm_state;
	int batt_temp = 0;
	int vbatt_uv = 0;
	int batt_present = 0;
	int system_temp_level = 0;

#if 0
	fsm_state = pm_chg_get_fsm_state(the_chip);
	if((the_chip->dc_present || oem_stand_detect) &&
		 (fsm_state == FSM_STATE_FAST_CHG_7) &&
		 (pm_chg_get_rt_status(the_chip, FASTCHG_IRQ)) &&
		 !delayed_work_pending(&the_chip->eoc_work)) {
		wake_lock(&the_chip->eoc_wake_lock);
		schedule_delayed_work(&the_chip->eoc_work,
			round_jiffies_relative(msecs_to_jiffies(EOC_CHECK_PERIOD_MS)));
	}
#endif

	if (oem_stand_detect != DETECT_STAND){
		CHG_STAND_DEBUG("oem_stand_detect not DETECT_STAND:%d\n", oem_stand_detect);
		return;
	} else {
		if (GPIO_INPUT_HI == gpio_get_value(GPIO_CHG_DET_N)) {
			oem_det_n_detect = NOT_DETECT;
			schedule_delayed_work(&oem_chg_det_n_work,
					round_jiffies_relative(msecs_to_jiffies(100)));
		}
		if (GPIO_INPUT_HI == gpio_get_value(GPIO_CHG_TBATT2)) {
			oem_tbatt2_detect = NOT_DETECT;
			schedule_delayed_work(&oem_chg_tbatt2_work,
				round_jiffies_relative(msecs_to_jiffies(100)));
		}
	}

	if (oem_chg_stand_charge_timer != 0){
		oem_chg_stand_charge_timer--;
		CHG_STAND_DEBUG("oem_chg_stand_charge_timer count. remain:%d\n", oem_chg_stand_charge_timer);
		return;
	}

	batt_temp = oem_chg_get_battery_property(POWER_SUPPLY_PROP_TEMP);
	vbatt_uv = oem_chg_get_battery_property(POWER_SUPPLY_PROP_VOLTAGE_NOW);
	batt_present = oem_chg_get_battery_property(POWER_SUPPLY_PROP_PRESENT);
	system_temp_level = oem_chg_get_battery_property(POWER_SUPPLY_PROP_SYSTEM_TEMP_LEVEL);

	if (0 == oem_charge_stand_flag) {
		do {
			if ((POWER_SUPPLY_CHARGE_TYPE_FAST != oem_chg_get_battery_property(POWER_SUPPLY_PROP_CHARGE_TYPE))/* &&
			    (!oem_pm8921_bms_is_cyclecorrection_chargeoffstate())*/) {
				CHG_STAND_DEBUG("Not Fast Charge\n");
				check = false;
				break;
			}
			if (((oem_param_charger.chg_adp_tmp1 * 10) > batt_temp) ||
				((oem_param_charger.chg_adp_tmp3 * 10) < batt_temp)) {
				CHG_STAND_DEBUG("Batt Temp NG [%d]\n", batt_temp);
				check = false;
				break;
			}
			if ((THRESHOLD_VBATT_UV_LO > vbatt_uv) ||
				(THRESHOLD_VBATT_UV_HI < vbatt_uv)) {
				CHG_STAND_DEBUG("VBATT NG [%d]\n", vbatt_uv);
				check = false;
				break;
			}
			if (!oem_chg_ibatt_check()) {
				CHG_STAND_DEBUG("IBATT NG\n");
				check = false;
				break;
			}
			if (!oem_chg_dcin_check() || oem_chg_is_dc_ov_check(the_chip)){
				CHG_STAND_DEBUG("Stand Charge do not start -dcin voltage NG detected.\n");
				check = false;
				break;
			}
			if (!oem_chg_dcr_check()) {
				CHG_STAND_DEBUG("DCR NG\n");
				check = false;
				break;
			}
		} while(0);

		if (check) {
			oem_chg_stand_on();
			CHG_STAND_DEBUG("stand charge start. \n");
		}

	} else {
		do {
			if (((oem_param_charger.chg_adp_tmp1 * 10) > batt_temp) ||
				((oem_param_charger.chg_adp_tmp2 * 10) < batt_temp)) {
				CHG_STAND_DEBUG("Stand Charge Stop batt_temp=%d\n", batt_temp);
				check = false;
				break;
			}
			if (!batt_present){
				CHG_STAND_DEBUG("Stand Charge Stop -battery removed.\n");
				check = false;
				break;
			}
			if (oem_chg_is_dc_ov_check(the_chip)){
				CHG_STAND_DEBUG("Stand Charge Stop -adapter over voltage detected.\n");
				check = false;
				break;
			}
			if ((oem_param_charger.chg_stop_volt * 1000) <= vbatt_uv){
				CHG_STAND_DEBUG("Stand Charge Stop -battery voltage NG detected.\n");
				check = false;
				break;
			}
			if (!oem_chg_dcin_check()){
				CHG_STAND_DEBUG("Stand Charge Stop -dcin voltage NG detected.\n");
				check = false;
				break;
			}
			if (THERMAL_CHG_STOP_LVL == system_temp_level) {
				CHG_STAND_DEBUG("Stand Charge Stop -Thermal limitation system_temp_level=%d.\n",
				        system_temp_level);
				check = false;
				break;
			}
			
			get_ibatt = oem_chg_get_bms_property(POWER_SUPPLY_PROP_CURRENT_NOW);
			if ((THRESHOLD_IBAT_STAND_CHARGE_STOP_LO <= (get_ibatt))||
			    (THRESHOLD_IBAT_STAND_CHARGE_STOP_HI > (get_ibatt))){
				CHG_STAND_DEBUG("Stand Charge Stop -ibat NG detected.%d\n", get_ibatt);
				check = false;
				break;
			}
			
			if (oem_chg_ovp_n_detect == true) {
				CHG_STAND_DEBUG("Stand Charge Stop -BAT OVP detected.\n");
				oem_chg_ovp_n_detect = false;
				check = false;
				break;
			}

		} while(0);

		if (!check) {
			oem_chg_stand_off();
			CHG_STAND_DEBUG("stand charge end. \n");
		}
	}
}

static void oem_chg_tbatt2_control(struct work_struct *work)
{
	int rc = 0;
	int gpio_trigger = 0;

	if (GPIO_INPUT_LO == gpio_get_value(GPIO_CHG_TBATT2)) {
		CHG_STAND_DEBUG("GPIO_CHG_TBATT2 LOW!! oem_tbatt2_detect = %d  \n", oem_tbatt2_detect);
		if (NOT_DETECT == oem_tbatt2_detect) {
			oem_tbatt2_detect = DETECT_LO_PRE_FIX;
			oem_repeat_tbatt2 = true;
		} else if (DETECT_LO_PRE_FIX == oem_tbatt2_detect) {
			oem_tbatt2_detect = DETECT_LO_FIX;
			gpio_trigger = IRQF_TRIGGER_HIGH;
		}
		
		if ((DETECT_LO_FIX == oem_tbatt2_detect) &&
		    (DETECT_LO_FIX == oem_det_n_detect)) {
			oem_stand_detect = DETECT_STAND;
			oem_chg_stand_charge_timer = INTERVAL_STAND_CHG_START;
			//oem_chg_start_stand_monitor();
			CHG_STAND_DEBUG("Charge Stand Detected!! \n");
		}
	} else {
		CHG_STAND_DEBUG("GPIO_CHG_TBATT2 HIGH!! oem_tbatt2_detect = %d  \n", oem_tbatt2_detect);
		if (NOT_DETECT != oem_tbatt2_detect) {
			oem_tbatt2_detect = NOT_DETECT;
			oem_repeat_tbatt2 = true;
		} else {
			if (NOT_DETECT != oem_stand_detect) {
				if (1 == oem_charge_stand_flag) {
					oem_chg_stand_off();
				}
				oem_stand_detect = NOT_DETECT;
				//cancel_delayed_work_sync(&fast_chg_limit_work);
				//is_oem_fast_chg_counting=false;
				CHG_STAND_DEBUG("oem_chg_tbatt2_control stand removed.\n");
			}
			wake_unlock(&charging_stand_wake_lock);
			gpio_trigger = IRQF_TRIGGER_LOW;
		}
	}
	
	if (oem_repeat_tbatt2) {
		oem_repeat_tbatt2 = false;
		schedule_delayed_work(&oem_chg_tbatt2_work,
				round_jiffies_relative(msecs_to_jiffies(100)));
		return;
	}

	if (from_oem_chg_tbatt2_isr) {
		from_oem_chg_tbatt2_isr = false;
		free_irq(gpio_to_irq(GPIO_CHG_TBATT2), 0);
		rc = request_irq(gpio_to_irq(GPIO_CHG_TBATT2), oem_chg_tbatt2_isr, gpio_trigger, "CHG_TBATT2", 0);
		if (rc) {
			CHG_STAND_ERR("couldn't register interrupts rc=%d\n", rc);
		}
		return;
	}

}

static void oem_chg_det_n_control(struct work_struct *work)
{
	int rc = 0;
	int gpio_trigger = 0;

	if (GPIO_INPUT_LO == gpio_get_value(GPIO_CHG_DET_N)) {
		CHG_STAND_DEBUG("GPIO_CHG_DET_N LOW!! oem_det_n_detect = %d  \n", oem_det_n_detect);
		if (NOT_DETECT == oem_det_n_detect) {
			oem_det_n_detect = DETECT_LO_PRE_FIX;
			oem_repeat_det_n = true;
		} else if (DETECT_LO_PRE_FIX == oem_det_n_detect) {
			oem_det_n_detect = DETECT_LO_FIX;
			gpio_trigger = IRQF_TRIGGER_HIGH;
		}
		
		if ((DETECT_LO_FIX == oem_det_n_detect) &&
		    (DETECT_LO_FIX == oem_tbatt2_detect)) {
			oem_stand_detect = DETECT_STAND;
			oem_chg_stand_charge_timer = INTERVAL_STAND_CHG_START;
			//oem_chg_start_stand_monitor();
			CHG_STAND_DEBUG("Charge Stand Detected!! \n");
		}
	} else {
		CHG_STAND_DEBUG("GPIO_CHG_DET_N HIGH!! oem_det_n_detect = %d  \n", oem_det_n_detect);
		if (NOT_DETECT != oem_det_n_detect) {
			oem_det_n_detect = NOT_DETECT;
			oem_repeat_det_n = true;
		} else {
			if (NOT_DETECT != oem_stand_detect) {
				if (1 == oem_charge_stand_flag) {
					oem_chg_stand_off();
				}
				oem_stand_detect = NOT_DETECT;
				//cancel_delayed_work_sync(&fast_chg_limit_work);
				//is_oem_fast_chg_counting=false;
				CHG_STAND_DEBUG("oem_chg_det_n_control stand removed.\n");
			}
			wake_unlock(&charging_stand_wake_lock);
			gpio_trigger = IRQF_TRIGGER_LOW;
		}
	}

	if (oem_repeat_det_n) {
		oem_repeat_det_n = false;
		schedule_delayed_work(&oem_chg_det_n_work,
				round_jiffies_relative(msecs_to_jiffies(100)));
		return;
	}

	if (from_oem_chg_det_n_isr) {
		from_oem_chg_det_n_isr = false;
		free_irq(gpio_to_irq(GPIO_CHG_DET_N), 0);
		rc = request_irq(gpio_to_irq(GPIO_CHG_DET_N), oem_chg_det_n_isr, gpio_trigger, "CHG_DET_N", 0);
		if (rc) {
			CHG_STAND_ERR("couldn't register interrupts rc=%d\n", rc);
		}
		return;
	}

}

static void oem_bat_ovp_n_control(struct work_struct *work)
{
	CHG_STAND_DEBUG("BAT OVP irq detected!! \n");
	//oem_chg_stand_off();
	oem_chg_ovp_n_detect = true;
}

irqreturn_t oem_chg_tbatt2_isr(int irq, void *ptr)
{
	disable_irq_nosync(gpio_to_irq(GPIO_CHG_TBATT2));

	CHG_STAND_DEBUG("TBATT2 irq detected!! GPIO = %d \n", gpio_get_value(GPIO_CHG_TBATT2));

	if (GPIO_INPUT_LO == gpio_get_value(GPIO_CHG_TBATT2)) {
		oem_tbatt2_detect = DETECT_LO_PRE_FIX;
		wake_lock(&charging_stand_wake_lock);
	} else {
		oem_tbatt2_detect = NOT_DETECT;
	}

	from_oem_chg_tbatt2_isr = true;

	schedule_delayed_work(&oem_chg_tbatt2_work,
			round_jiffies_relative(msecs_to_jiffies(100)));

	return IRQ_HANDLED;
}

irqreturn_t oem_chg_det_n_isr(int irq, void *ptr)
{
	disable_irq_nosync(gpio_to_irq(GPIO_CHG_DET_N));

	CHG_STAND_DEBUG("CHG_DET_N irq detected!! GPIO = %d \n", gpio_get_value(GPIO_CHG_DET_N));

	if (GPIO_INPUT_LO == gpio_get_value(GPIO_CHG_DET_N)) {
		oem_det_n_detect = DETECT_LO_PRE_FIX;
		wake_lock(&charging_stand_wake_lock);
	} else {
		oem_det_n_detect = NOT_DETECT;
	}

	from_oem_chg_det_n_isr = true;

	schedule_delayed_work(&oem_chg_det_n_work,
			round_jiffies_relative(msecs_to_jiffies(100)));

	return IRQ_HANDLED;
}

irqreturn_t oem_bat_ovp_n_isr(int irq, void *ptr)
{
	disable_irq_nosync(gpio_to_irq(GPIO_BAT_OVP_N));

	CHG_STAND_DEBUG("BAT_OVP_N irq detected!! GPIO = %d \n", gpio_get_value(GPIO_BAT_OVP_N));

	if ((GPIO_INPUT_LO == gpio_get_value(GPIO_BAT_OVP_N)) &&
		(1 == oem_charge_stand_flag)) {
		schedule_delayed_work(&oem_bat_ovp_n_work,
				round_jiffies_relative(msecs_to_jiffies(0)));
	} 

	return IRQ_HANDLED;
}

int oem_chg_get_oem_stand_detect(void)
{
	return oem_stand_detect;
}
EXPORT_SYMBOL(oem_chg_get_oem_stand_detect);

int oem_chg_get_boost_chg_state(void)
{
	return oem_charge_stand_flag;
}
EXPORT_SYMBOL(oem_chg_get_boost_chg_state);

void oem_chg_stand_init(void *chip)
{
	int gpio_trigger_tbatt2 = 0;
	int gpio_trigger_det_n = 0;
	bool boot_stand_flag = true;
	int rc = 0;

	the_chip = chip;
	oem_chg_stand_bms_psy = power_supply_get_by_name("bms");
	oem_chg_stand_battery_psy = power_supply_get_by_name("battery");

	wake_lock_init(&charging_stand_wake_lock, WAKE_LOCK_SUSPEND, "charging_stand");
	INIT_DELAYED_WORK(&oem_chg_tbatt2_work, oem_chg_tbatt2_control);
	INIT_DELAYED_WORK(&oem_chg_det_n_work, oem_chg_det_n_control);
	INIT_DELAYED_WORK(&oem_bat_ovp_n_work, oem_bat_ovp_n_control);
	INIT_DELAYED_WORK(&oem_chg_stand_monitor_work, oem_chg_stand_monitor);

	gpio_tlmm_config(GPIO_CFG(GPIO_CHG_TBATT2, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(GPIO_CHG_DET_N, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_tlmm_config(GPIO_CFG(GPIO_BAT_OVP_N, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

	gpio_request( GPIO_BAT_OVP_RST_N, "GPIO_BAT_OVP_RST_N PORT" );
	gpio_tlmm_config(GPIO_CFG(GPIO_BAT_OVP_RST_N, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

	gpio_request( GPIO_CHG_TBATT2_ON, "GPIO_CHG_TBATT2_ON PORT" );
	gpio_tlmm_config(GPIO_CFG(GPIO_CHG_TBATT2_ON, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);

	if (GPIO_INPUT_LO == gpio_get_value(GPIO_CHG_TBATT2)) {
		gpio_trigger_tbatt2 = IRQF_TRIGGER_HIGH;
		oem_tbatt2_detect = DETECT_LO_FIX;
	} else {
		gpio_trigger_tbatt2 = IRQF_TRIGGER_LOW;
		boot_stand_flag = false;
	}
	if (GPIO_INPUT_LO == gpio_get_value(GPIO_CHG_DET_N)) {
		gpio_trigger_det_n = IRQF_TRIGGER_HIGH;
		oem_det_n_detect = DETECT_LO_FIX;
	} else {
		gpio_trigger_det_n = IRQF_TRIGGER_LOW;
		boot_stand_flag = false;
	}

	if (boot_stand_flag) {
		oem_stand_detect = DETECT_STAND;
		wake_lock(&charging_stand_wake_lock);
		oem_chg_stand_charge_timer = INTERVAL_STAND_CHG_START;
	}

	rc = request_irq(gpio_to_irq(GPIO_CHG_TBATT2), oem_chg_tbatt2_isr, gpio_trigger_tbatt2, "CHG_TBATT2", 0);
	if (rc) {
		CHG_STAND_ERR("couldn't register interrupts rc=%d\n", rc);
		return;
	}
	rc = request_irq(gpio_to_irq(GPIO_CHG_DET_N), oem_chg_det_n_isr, gpio_trigger_det_n, "CHG_DET_N", 0);
	if (rc) {
		CHG_STAND_ERR("couldn't register interrupts rc=%d\n", rc);
		return;
	}
	rc = request_irq(gpio_to_irq(GPIO_BAT_OVP_N), oem_bat_ovp_n_isr, IRQF_TRIGGER_LOW, "BAT_OVP_N", 0);
	if (rc) {
		CHG_STAND_ERR("couldn't register interrupts rc=%d\n", rc);
		return;
	}
	disable_irq_nosync(gpio_to_irq(GPIO_BAT_OVP_N));
//	enable_irq_wake(gpio_to_irq(GPIO_CHG_TBATT2));
	enable_irq_wake(gpio_to_irq(GPIO_CHG_DET_N));

	oem_chg_start_stand_monitor();

	CHG_STAND_DEBUG("Charge Stand Init Finished!! \n");
}
EXPORT_SYMBOL(oem_chg_stand_init);


void oem_chg_stand_timer_work_init(void)
{
	INIT_DELAYED_WORK(&oem_chg_stand_timer_done_work, oem_chg_stand_timer_done_worker);
}
EXPORT_SYMBOL(oem_chg_stand_timer_work_init);

static void oem_chg_stand_timer_satrt(void)
{
	CHG_STAND_DEBUG("Stand Charge timer start called \n");
	
	if (is_oem_chg_stand_timer_counting == false) {
		schedule_delayed_work(&oem_chg_stand_timer_done_work,
				round_jiffies_relative(msecs_to_jiffies
					(oem_param_charger.time_chg * 60 * 1000)));
		CHG_STAND_DEBUG("oem_param_charger.time_chg = %d\n", oem_param_charger.time_chg);
		is_oem_chg_stand_timer_counting = true;
	}
}

static void oem_chg_stand_timer_stop(void)
{
	CHG_STAND_DEBUG("oem_chg_stand_timer_stop() Called\n");
	
	if (is_oem_chg_stand_timer_counting == true) {
		cancel_delayed_work_sync(&oem_chg_stand_timer_done_work);
		is_oem_chg_stand_timer_counting = false;
	}
}

static void oem_chg_stand_timer_done_worker(struct work_struct *work)
{
	CHG_STAND_DEBUG("Charge timer expired \n");
	
	is_oem_chg_stand_timer_counting = false;
	if (1 == oem_charge_stand_flag) {
		oem_chg_stand_off();
	}
}
