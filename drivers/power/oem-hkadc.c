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
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/qpnp/pin.h>
#include <mach/kc_board.h>
#include <oem-hkadc.h>
#include <oem-bms.h>
#include <oem-charger.h>
#include <oem-charger_parm.h>


//#define FEATURE_CHG_HKADC_DEBUG
#ifdef FEATURE_CHG_HKADC_DEBUG
#define CHG_HKADC_ERR       pr_err
#define CHG_HKADC_DEBUG     pr_err
#else
#define CHG_HKADC_ERR       pr_err
#define CHG_HKADC_DEBUG     pr_debug
#endif

DEFINE_SPINLOCK(oem_hkadc_master_lock);
#define MASTERDATA_SPINLOCK_INIT()	spin_lock_init(&oem_hkadc_master_lock);

#define	MASTERDATA_LOCK()		\
	{				\
	unsigned long oem_hkadc_irq_flag;	\
	spin_lock_irqsave(&oem_hkadc_master_lock, oem_hkadc_irq_flag);

#define	MASTERDATA_UNLOCK()						\
	spin_unlock_irqrestore(&oem_hkadc_master_lock, oem_hkadc_irq_flag);	\
	}

#define DRV_NAME "oem_hkadc-driver"

struct oem_hkadc_device {
	int gpio;
	int irq;
	struct work_struct irq_work;
};

static struct oem_hkadc_device *oem_hkadc_dev;

static void *the_chip;

static struct delayed_work	oem_hkadc_work;
static int oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_NUM];

static int vbat_work[HKADC_VBAT_VOLTS_ELEMENT_COUNT]={0};
static int bat_temp_work[HKADC_BAT_TEMP_ELEMENT_COUNT]={0};
static int pa_therm_work[HKADC_PA_THERM_ELEMENT_COUNT]={0};
static int camera_therm_work[HKADC_CAMERA_TEMP_ELEMENT_COUNT]={0};
static int substrate_therm_work[HKADC_SUBSTRATE_THERM_ELEMENT_COUNT]={0};
static int usb_therm_work[HKADC_USB_THERM_ELEMENT_COUNT]={0};

//static struct alarm androidalarm;
//static struct timespec androidalarm_interval_timespec;
//struct early_suspend hkadc_early_suspend;
struct wake_lock oem_hkadc_wake_lock;

static int pa_therm_monit_freq = 0;
static int camera_temp_monit_freq = 0;
static int substrate_therm_monit_freq = 0;
static int usb_therm_monit_freq = 0;
static int uevent_notify_freq = 0;

static atomic_t is_hkadc_initialized=ATOMIC_INIT(0);

struct power_supply *hkadc_batt_psy = NULL;

static int oem_vbat_value = 0;

static union power_supply_propval hold_status = {POWER_SUPPLY_STATUS_DISCHARGING, };

static void oem_hkadc_master_data_write(enum power_supply_property psp, int val)
{
	int* masterdata;

	switch(psp){
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		masterdata
		 = &oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_BAT_VOLTAGE];
		break;

	case POWER_SUPPLY_PROP_TEMP:
		masterdata
		 = &oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_BAT_THERM];
		break;

	case POWER_SUPPLY_PROP_OEM_PA_THERM:
		masterdata
		 = &oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_PA_THERM];
		break;

	case POWER_SUPPLY_PROP_OEM_CAMERA_THERM:
		masterdata
		 = &oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_CAMERA_THERM];
		break;

	case POWER_SUPPLY_PROP_OEM_SUBSTRATE_THERM:
		masterdata
		 = &oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_SUBSTRATE_THERM];
		break;

	case POWER_SUPPLY_PROP_OEM_USB_THERM:
		masterdata
		 = &oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_USB_THERM];
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

static int oem_hkadc_master_data_read(enum power_supply_property psp, int *intval)
{
	int ret = 0;
	int initialized;
	int* masterdata;

	initialized = atomic_read(&is_hkadc_initialized);

	if (!initialized) {
		CHG_HKADC_ERR("called before init\n");
		MASTERDATA_LOCK();
		*intval = 0;
		MASTERDATA_UNLOCK();
		return -EAGAIN;
	}

	switch(psp){
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		masterdata =
		 &oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_BAT_VOLTAGE];
		break;

	case POWER_SUPPLY_PROP_TEMP:
		masterdata =
		 &oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_BAT_THERM];
		break;

	case POWER_SUPPLY_PROP_OEM_PA_THERM:
		masterdata =
		 &oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_PA_THERM];
		break;

	case POWER_SUPPLY_PROP_OEM_CAMERA_THERM:
		masterdata =
		 &oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_CAMERA_THERM];
		break;

	case POWER_SUPPLY_PROP_OEM_SUBSTRATE_THERM:
		masterdata =
		 &oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_SUBSTRATE_THERM];
		break;

	case POWER_SUPPLY_PROP_OEM_USB_THERM:
		masterdata =
		 &oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_USB_THERM];
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
		CHG_HKADC_DEBUG("Out of range psp %d \n", psp);
		ret = -EINVAL;
	}

	return ret;

}

#define HKADC_BATT_DEFAULT_VOLTAGE	3600000
static int oem_hkadc_init_battery_uvolts(void)
{
	int rc;
	int work=0;
	int cnt;
	struct qpnp_vadc_result result;

	if (!the_chip) {
		CHG_HKADC_ERR("called before init\n");
		return -EINVAL;
	}
	rc = qpnp_vadc_read(VBAT_SNS, &result);
	if (rc) {
		CHG_HKADC_ERR("error reading adc channel = %d, rc = %d\n",
					VBAT_SNS, rc);
		work = HKADC_BATT_DEFAULT_VOLTAGE;
	}else{
		work = (int)result.physical;
	}
	for (cnt=0;cnt<HKADC_VBAT_VOLTS_ELEMENT_COUNT;cnt++) {
		vbat_work[cnt] = work;
	}

	oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_BAT_VOLTAGE] = work;

	CHG_HKADC_DEBUG("init hkadc uvolts = %d \n",
		oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_BAT_VOLTAGE]);

	return 0;
}

static int oem_hkadc_get_battery_uvolts(int *vbat)
{
	int rc;
	int work=0;
	int max=0, min=0;
	int total=0;
	int cnt;
	int average;
	struct qpnp_vadc_result result;

	if (!the_chip) {
		CHG_HKADC_ERR("called before init\n");
		return -EINVAL;
	}
	rc = qpnp_vadc_read(VBAT_SNS, &result);
	if (rc) {
		CHG_HKADC_ERR("error reading adc channel = %d, rc = %d\n",
					VBAT_SNS, rc);
		rc = oem_hkadc_master_data_read(POWER_SUPPLY_PROP_VOLTAGE_NOW, vbat);
		CHG_HKADC_DEBUG("hkadc get volt vbat do not updated. %d %d\n", rc, *vbat);
		return rc;
	}
	work = (int)result.physical;
	for (cnt=0;cnt<HKADC_VBAT_VOLTS_ELEMENT_COUNT-1;cnt++) {
		vbat_work[cnt] = vbat_work[cnt+1];
		total = total + vbat_work[cnt];
	}
	vbat_work[cnt] = work;
	total = total + vbat_work[cnt];

	max = vbat_work[0];
	min = vbat_work[0];
	for (cnt=0;cnt<HKADC_VBAT_VOLTS_ELEMENT_COUNT;cnt++) {
		if (max < vbat_work[cnt]) {
			max = vbat_work[cnt];
		} else if (min > vbat_work[cnt]) {
			min = vbat_work[cnt];
		}
	}

	average = (total - max - min) / (HKADC_VBAT_VOLTS_ELEMENT_COUNT-2);
	oem_hkadc_master_data_write(POWER_SUPPLY_PROP_VOLTAGE_NOW, average);
	*vbat = average;
	CHG_HKADC_DEBUG("hkadc volt  total = %d max = %d min = %d \n", total, max, min);
	CHG_HKADC_DEBUG("hkadc uvolts = %d   %d %d %d %d %d \n", 
		average,
		vbat_work[0],
		vbat_work[1],
		vbat_work[2],
		vbat_work[3],
		vbat_work[4]);

	return 0;
}

static int oem_hkadc_get_wakeup_battery_uvolts(int *vbat)
{
	int rc;
	int cnt;
	int ngcnt;
	int vbat_new=0;
	int max=0, min=0;
	int total=0;
	int average;
	int vbat_work_tmp[HKADC_VBAT_VOLTS_ELEMENT_COUNT];
	struct qpnp_vadc_result result;

	if (!the_chip) {
		CHG_HKADC_ERR("called before init\n");
		return -EINVAL;
	}

	ngcnt = 0;
	for (cnt=0;cnt<HKADC_VBAT_VOLTS_ELEMENT_COUNT;cnt++) {
		rc = qpnp_vadc_read(VBAT_SNS, &result);
		if (rc) {
			CHG_HKADC_ERR("error reading adc channel = %d, rc = %d\n",
						VBAT_SNS, rc);
			ngcnt++;
		}else{
			vbat_new = (int)result.physical;
			vbat_work_tmp[cnt] = (int)result.physical;
		}
	}
	if (ngcnt == 0){
		for (cnt=0;cnt<HKADC_VBAT_VOLTS_ELEMENT_COUNT;cnt++) {
			vbat_work[cnt]=vbat_work_tmp[cnt];
			total = total + vbat_work_tmp[cnt];
		}
		max = vbat_work[0];
		min = vbat_work[0];
		for (cnt=0;cnt<HKADC_VBAT_VOLTS_ELEMENT_COUNT;cnt++) {
			if (max < vbat_work[cnt]) {
				max = vbat_work[cnt];
			} else if (min > vbat_work[cnt]) {
				min = vbat_work[cnt];
			}
		}

		average = (total - max - min) / (HKADC_VBAT_VOLTS_ELEMENT_COUNT-2);
		oem_hkadc_master_data_write(POWER_SUPPLY_PROP_VOLTAGE_NOW, average);
		*vbat = average;
		CHG_HKADC_DEBUG("hkadc wakeup volt  total = %d max = %d min = %d \n", total, max, min);
		CHG_HKADC_DEBUG("hkadc wakeup uvolts = %d   %d %d %d %d %d \n", 
			average,
			vbat_work[0],
			vbat_work[1],
			vbat_work[2],
			vbat_work[3],
			vbat_work[4]);
	}else if (ngcnt < HKADC_VBAT_VOLTS_ELEMENT_COUNT){
		for (cnt=0;cnt<HKADC_VBAT_VOLTS_ELEMENT_COUNT;cnt++) {
			vbat_work[cnt] = vbat_new;
		}
		oem_hkadc_master_data_write(POWER_SUPPLY_PROP_VOLTAGE_NOW, vbat_new);
		*vbat = vbat_new;
		CHG_HKADC_DEBUG("hkadc wakeup volt vbat_new = %d\n", vbat_new);
	}else{
		rc = oem_hkadc_master_data_read(POWER_SUPPLY_PROP_VOLTAGE_NOW, vbat);
		CHG_HKADC_DEBUG("hkadc wakeup volt vbat do not updated. %d %d\n", rc, *vbat);
	}

	return 0;
}

#define HKADC_BATT_DEFAULT_TEMP		250
static int oem_hkadc_init_battery_temp(void)
{
	int rc;
	int work=0;
	int cnt;
	struct qpnp_vadc_result result;

	if (!the_chip) {
		CHG_HKADC_ERR("called before init\n");
		return -EINVAL;
	}
	rc = qpnp_vadc_read(LR_MUX1_BATT_THERM, &result);
	if (rc) {
		CHG_HKADC_ERR("error reading adc channel = %d, rc = %d\n",
					LR_MUX1_BATT_THERM, rc);
		work = HKADC_BATT_DEFAULT_TEMP;
	}else{
		work = (int)result.physical;
	}
	for (cnt=0;cnt<HKADC_BAT_TEMP_ELEMENT_COUNT;cnt++) {
		bat_temp_work[cnt] = work;
	}

	oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_BAT_THERM] = work;

	CHG_HKADC_DEBUG("init hkadc battery temp = %d \n",
		oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_BAT_THERM]);

//	oem_charger_init_batterydetectionstate(work);

	return 0;
}

static int oem_hkadc_get_battery_temp(int *temp)
{
	int rc;
	int work=0;
	int max=0, min=0;
	int total=0;
	int cnt;
	int average;
	struct qpnp_vadc_result result;

	if (!the_chip) {
		CHG_HKADC_ERR("called before init\n");
		return -EINVAL;
	}
	rc = qpnp_vadc_read(LR_MUX1_BATT_THERM, &result);
	if (rc) {
		CHG_HKADC_ERR("error reading adc channel = %d, rc = %d\n",
					LR_MUX1_BATT_THERM, rc);
		rc = oem_hkadc_master_data_read(POWER_SUPPLY_PROP_TEMP, temp);
		CHG_HKADC_DEBUG("hkadc get battery temp do not updated. %d %d\n", rc, *temp);
		return rc;
	}
	work = (int)result.physical;
	for (cnt=0;cnt<HKADC_BAT_TEMP_ELEMENT_COUNT-1;cnt++) {
		bat_temp_work[cnt] = bat_temp_work[cnt+1];
		total = total + bat_temp_work[cnt];
	}
	bat_temp_work[cnt] = work;
	total = total + bat_temp_work[cnt];

	max = bat_temp_work[0];
	min = bat_temp_work[0];
	for (cnt=0;cnt<HKADC_BAT_TEMP_ELEMENT_COUNT;cnt++) {
		if (max < bat_temp_work[cnt]) {
			max = bat_temp_work[cnt];
		} else if (min > bat_temp_work[cnt]) {
			min = bat_temp_work[cnt];
		}
	}

	average = (total - max - min) / (HKADC_BAT_TEMP_ELEMENT_COUNT-2);
	oem_hkadc_master_data_write(POWER_SUPPLY_PROP_TEMP, average);
	*temp = average;
	CHG_HKADC_DEBUG("hkadc battery temp total = %d max = %d min = %d \n", total, max, min);
	CHG_HKADC_DEBUG("hkadc battery temps = %d   %d %d %d %d %d \n", 
		average,
		bat_temp_work[0],
		bat_temp_work[1],
		bat_temp_work[2],
		bat_temp_work[3],
		bat_temp_work[4]);

	return 0;
}

static int oem_hkadc_get_wakeup_battery_temp(int *temp)
{
	int rc;
	int cnt;
	int ngcnt;
	int battemp_new=0;
	int max=0, min=0;
	int total=0;
	int average;
	int bat_temp_work_tmp[HKADC_BAT_TEMP_ELEMENT_COUNT];
	struct qpnp_vadc_result result;

	if (!the_chip) {
		CHG_HKADC_ERR("called before init\n");
		return -EINVAL;
	}

	ngcnt = 0;
	for (cnt=0;cnt<HKADC_BAT_TEMP_ELEMENT_COUNT;cnt++) {
		rc = qpnp_vadc_read(LR_MUX1_BATT_THERM, &result);
		if (rc) {
			CHG_HKADC_ERR("error reading adc channel = %d, rc = %d\n",
						LR_MUX1_BATT_THERM, rc);
			ngcnt++;
		}else{
			battemp_new = (int)result.physical;
			bat_temp_work_tmp[cnt] = (int)result.physical;
		}
	}
	if (ngcnt == 0){
		for (cnt=0;cnt<HKADC_BAT_TEMP_ELEMENT_COUNT;cnt++) {
			bat_temp_work[cnt]=bat_temp_work_tmp[cnt];
			total = total + bat_temp_work_tmp[cnt];
		}
		max = bat_temp_work[0];
		min = bat_temp_work[0];
		for (cnt=0;cnt<HKADC_BAT_TEMP_ELEMENT_COUNT;cnt++) {
			if (max < bat_temp_work[cnt]) {
				max = bat_temp_work[cnt];
			} else if (min > bat_temp_work[cnt]) {
				min = bat_temp_work[cnt];
			}
		}

		average = (total - max - min) / (HKADC_BAT_TEMP_ELEMENT_COUNT-2);
		oem_hkadc_master_data_write(POWER_SUPPLY_PROP_TEMP, average);
		*temp = average;
		CHG_HKADC_DEBUG("hkadc wakeup battery temp total = %d max = %d min = %d \n", total, max, min);
		CHG_HKADC_DEBUG("hkadc wakeup battery temp = %d   %d %d %d %d %d \n", 
			average,
			bat_temp_work[0],
			bat_temp_work[1],
			bat_temp_work[2],
			bat_temp_work[3],
			bat_temp_work[4]);
	}else if (ngcnt < HKADC_BAT_TEMP_ELEMENT_COUNT){
		for (cnt=0;cnt<HKADC_BAT_TEMP_ELEMENT_COUNT;cnt++) {
			bat_temp_work[cnt] = battemp_new;
		}
		oem_hkadc_master_data_write(POWER_SUPPLY_PROP_TEMP, battemp_new);
		*temp = battemp_new;
		CHG_HKADC_DEBUG("hkadc wakeup battery temp battemp_new = %d\n", battemp_new);
	}else{
		rc = oem_hkadc_master_data_read(POWER_SUPPLY_PROP_TEMP, temp);
		CHG_HKADC_DEBUG("hkadc wakeup battery temp do not updated. %d %d\n", rc, *temp);
	}

	return 0;
}

static int oem_hkadc_init_pa_therm(void)
{
	int rc;
	int work=0;
	int cnt;

	struct qpnp_vadc_result result;

	rc = qpnp_vadc_read(LR_MUX6_PU2_AMUX_THM3, &result);
	if (rc) {
		CHG_HKADC_DEBUG("error reading adc channel = %d, rc = %d\n",
				LR_MUX6_PU2_AMUX_THM3, rc);
		return rc;
	}
	work = (int)result.physical;

	for (cnt=0;cnt<HKADC_PA_THERM_ELEMENT_COUNT;cnt++) {
		pa_therm_work[cnt] = work;
	}

	oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_PA_THERM] = work;

	CHG_HKADC_DEBUG("init hkadc pa therm = %d \n",
		oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_PA_THERM]);

	return 0;
}

static int oem_hkadc_get_pa_therm(void)
{
	int rc;
	int work=0;
	int max=0, min=0;
	int total=0;
	int cnt;
	int average;

	struct qpnp_vadc_result result;

	rc = qpnp_vadc_read(LR_MUX6_PU2_AMUX_THM3, &result);
	if (rc) {
		CHG_HKADC_DEBUG("error reading adc channel = %d, rc = %d\n",
				LR_MUX6_PU2_AMUX_THM3, rc);
		return rc;
	}
	work = (int)result.physical;
	for (cnt=0;cnt<HKADC_PA_THERM_ELEMENT_COUNT-1;cnt++) {
		pa_therm_work[cnt] = pa_therm_work[cnt+1];
		total = total + pa_therm_work[cnt];
	}
	pa_therm_work[cnt] = (int)result.physical;
	total = total + pa_therm_work[cnt];

	max = pa_therm_work[0];
	min = pa_therm_work[0];
	for (cnt=0;cnt<HKADC_PA_THERM_ELEMENT_COUNT;cnt++) {
		if (max < pa_therm_work[cnt]) {
			max = pa_therm_work[cnt];
		} else if (min > pa_therm_work[cnt]) {
			min = pa_therm_work[cnt];
		}
	}

	average = (total - max - min) / (HKADC_PA_THERM_ELEMENT_COUNT-2);
	oem_hkadc_master_data_write(POWER_SUPPLY_PROP_OEM_PA_THERM, average);

	CHG_HKADC_DEBUG("hkadc pa_therm total = %d max = %d min = %d \n", total, max, min);
	CHG_HKADC_DEBUG("hkadc pa_therm = %d   %d %d %d %d %d \n", 
		average,
		pa_therm_work[0],
		pa_therm_work[1],
		pa_therm_work[2],
		pa_therm_work[3],
		pa_therm_work[4]);

	return 0;
}

static int oem_hkadc_get_wakeup_pa_therm(void)
{
	int rc;
	int cnt;
	int ngcnt;
	int pa_therm_new=0;
	int max=0, min=0;
	int total=0;
	int average;
	int pa_therm_work_tmp[HKADC_PA_THERM_ELEMENT_COUNT];
	struct qpnp_vadc_result result;

	ngcnt = 0;
	for (cnt=0;cnt<HKADC_PA_THERM_ELEMENT_COUNT;cnt++) {
		rc = qpnp_vadc_read(LR_MUX6_PU2_AMUX_THM3, &result);
		if (rc) {
			CHG_HKADC_ERR("error reading adc channel = %d, rc = %d\n",
					LR_MUX6_PU2_AMUX_THM3, rc);
			ngcnt++;
		}else{
			pa_therm_new = (int)result.physical;
			pa_therm_work_tmp[cnt] = (int)result.physical;
		}
	}
	if (ngcnt == 0){
		for (cnt=0;cnt<HKADC_PA_THERM_ELEMENT_COUNT;cnt++) {
			pa_therm_work[cnt]=pa_therm_work_tmp[cnt];
			total = total + pa_therm_work_tmp[cnt];
		}
		max = pa_therm_work[0];
		min = pa_therm_work[0];
		for (cnt=0;cnt<HKADC_PA_THERM_ELEMENT_COUNT;cnt++) {
			if (max < pa_therm_work[cnt]) {
				max = pa_therm_work[cnt];
			} else if (min > pa_therm_work[cnt]) {
				min = pa_therm_work[cnt];
			}
		}

		average = (total - max - min) / (HKADC_PA_THERM_ELEMENT_COUNT-2);
		oem_hkadc_master_data_write(POWER_SUPPLY_PROP_OEM_PA_THERM, average);
		CHG_HKADC_DEBUG("hkadc wakeup pa_therm total = %d max = %d min = %d \n", total, max, min);
		CHG_HKADC_DEBUG("hkadc wakeup pa_therm = %d   %d %d %d %d %d \n", 
			average,
			pa_therm_work[0],
			pa_therm_work[1],
			pa_therm_work[2],
			pa_therm_work[3],
			pa_therm_work[4]);
	}else if (ngcnt < HKADC_PA_THERM_ELEMENT_COUNT){
		for (cnt=0;cnt<HKADC_PA_THERM_ELEMENT_COUNT;cnt++) {
			pa_therm_work[cnt] = pa_therm_new;
		}
		oem_hkadc_master_data_write(POWER_SUPPLY_PROP_OEM_PA_THERM, pa_therm_new);
		CHG_HKADC_DEBUG("hkadc wakeup pa_therm_new = %d\n", pa_therm_new);
	}else{
		CHG_HKADC_DEBUG("hkadc wakeup pa_therm do not updated.\n");
	}

	return 0;
}

static int oem_hkadc_init_camera_therm(void)
{
	int rc;
	int work=0;
	int cnt;

	struct qpnp_vadc_result result;

	rc = qpnp_vadc_read(LR_MUX4_AMUX_THM1, &result);
	if (rc) {
		CHG_HKADC_DEBUG("error reading adc channel = %d, rc = %d\n",
				LR_MUX4_AMUX_THM1, rc);
		return rc;
	}
	work = (int)result.physical;

	for (cnt=0;cnt<HKADC_CAMERA_TEMP_ELEMENT_COUNT;cnt++) {
		camera_therm_work[cnt] = work;
	}

	oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_CAMERA_THERM] = work;

	CHG_HKADC_DEBUG("init hkadc camera therm = %d \n",
		oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_CAMERA_THERM]);

	return 0;
}

static int oem_hkadc_get_camera_therm(int *therm)
{
	int rc;
	int work=0;
	int max=0, min=0;
	int total=0;
	int cnt;
	int average;

	struct qpnp_vadc_result result;

	rc = qpnp_vadc_read(LR_MUX4_AMUX_THM1, &result);
	if (rc) {
		CHG_HKADC_DEBUG("error reading adc channel = %d, rc = %d\n",
				LR_MUX4_AMUX_THM1, rc);
		rc = oem_hkadc_master_data_read(POWER_SUPPLY_PROP_OEM_CAMERA_THERM, therm);
		CHG_HKADC_DEBUG("hkadc get camera_therm do not updated. %d %d\n", rc, *therm);
		return rc;
	}
	work = (int)result.physical;
	for (cnt=0;cnt<HKADC_CAMERA_TEMP_ELEMENT_COUNT-1;cnt++) {
		camera_therm_work[cnt] = camera_therm_work[cnt+1];
		total = total + camera_therm_work[cnt];
	}
	camera_therm_work[cnt] = work;
	total = total + camera_therm_work[cnt];

	max = camera_therm_work[0];
	min = camera_therm_work[0];
	for (cnt=0;cnt<HKADC_CAMERA_TEMP_ELEMENT_COUNT;cnt++) {
		if (max < camera_therm_work[cnt]) {
			max = camera_therm_work[cnt];
		} else if (min > camera_therm_work[cnt]) {
			min = camera_therm_work[cnt];
		}
	}

	average = (total - max - min) / (HKADC_CAMERA_TEMP_ELEMENT_COUNT-2);
	oem_hkadc_master_data_write(POWER_SUPPLY_PROP_OEM_CAMERA_THERM, average);
	*therm = average;
	CHG_HKADC_DEBUG("hkadc camera therm total = %d max = %d min = %d \n", total, max, min);
	CHG_HKADC_DEBUG("hkadc camera therm = %d   %d %d %d %d %d \n", 
		average, camera_therm_work[0],
		camera_therm_work[1],
		camera_therm_work[2],
		camera_therm_work[3],
		camera_therm_work[4]);

	return 0;
}

static int oem_hkadc_get_wakeup_camera_therm(int *therm)
{
	int rc;
	int cnt;
	int ngcnt;
	int camera_therm_new=0;
	int max=0, min=0;
	int total=0;
	int average;
	int camera_therm_work_tmp[HKADC_CAMERA_TEMP_ELEMENT_COUNT];
	struct qpnp_vadc_result result;

	ngcnt = 0;
	for (cnt=0;cnt<HKADC_CAMERA_TEMP_ELEMENT_COUNT;cnt++) {
		rc = qpnp_vadc_read(LR_MUX4_AMUX_THM1, &result);
		if (rc) {
			CHG_HKADC_ERR("error reading adc channel = %d, rc = %d\n",
					LR_MUX4_AMUX_THM1, rc);
			ngcnt++;
		}else{
			camera_therm_new = (int)result.physical;
			camera_therm_work_tmp[cnt] = (int)result.physical;
		}
	}
	if (ngcnt == 0){
		for (cnt=0;cnt<HKADC_CAMERA_TEMP_ELEMENT_COUNT;cnt++) {
			camera_therm_work[cnt]=camera_therm_work_tmp[cnt];
			total = total + camera_therm_work_tmp[cnt];
		}
		max = camera_therm_work[0];
		min = camera_therm_work[0];
		for (cnt=0;cnt<HKADC_CAMERA_TEMP_ELEMENT_COUNT;cnt++) {
			if (max < camera_therm_work[cnt]) {
				max = camera_therm_work[cnt];
			} else if (min > camera_therm_work[cnt]) {
				min = camera_therm_work[cnt];
			}
		}

		average = (total - max - min) / (HKADC_CAMERA_TEMP_ELEMENT_COUNT-2);
		oem_hkadc_master_data_write(POWER_SUPPLY_PROP_OEM_CAMERA_THERM, average);
		*therm = average;
		CHG_HKADC_DEBUG("hkadc wakeup camera_therm total = %d max = %d min = %d \n", total, max, min);
		CHG_HKADC_DEBUG("hkadc wakeup camera_therm = %d   %d %d %d %d %d \n", 
			average,
			camera_therm_work[0],
			camera_therm_work[1],
			camera_therm_work[2],
			camera_therm_work[3],
			camera_therm_work[4]);
	}else if (ngcnt < HKADC_CAMERA_TEMP_ELEMENT_COUNT){
		for (cnt=0;cnt<HKADC_CAMERA_TEMP_ELEMENT_COUNT;cnt++) {
			camera_therm_work[cnt] = camera_therm_new;
		}
		oem_hkadc_master_data_write(POWER_SUPPLY_PROP_OEM_CAMERA_THERM, camera_therm_new);
		*therm = camera_therm_new;
		CHG_HKADC_DEBUG("hkadc wakeup camera_therm_new = %d\n", camera_therm_new);
	}else{
		rc = oem_hkadc_master_data_read(POWER_SUPPLY_PROP_OEM_CAMERA_THERM, therm);
		CHG_HKADC_DEBUG("hkadc wakeup camera_therm do not updated. %d %d\n", rc, *therm);
	}

	return 0;
}

static int oem_hkadc_init_substrate_therm(void)
{
	int rc;
	int work=0;
	int cnt;

	struct qpnp_vadc_result result;

	rc = qpnp_vadc_read(LR_MUX5_AMUX_THM2, &result);
	if (rc) {
		CHG_HKADC_DEBUG("error reading adc channel = %d, rc = %d\n",
				LR_MUX5_AMUX_THM2, rc);
		return rc;
	}
	work = (int)result.physical;

	for (cnt=0;cnt<HKADC_SUBSTRATE_THERM_ELEMENT_COUNT;cnt++) {
		substrate_therm_work[cnt] = work;
	}

	oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_SUBSTRATE_THERM] = work;

	CHG_HKADC_DEBUG("init hkadc substrate therm = %d \n",
		oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_SUBSTRATE_THERM]);

	return 0;
}

static int oem_hkadc_get_substrate_therm(int *therm)
{
	int rc;
	int work=0;
	int max=0, min=0;
	int total=0;
	int cnt;
	int average;

	struct qpnp_vadc_result result;

	rc = qpnp_vadc_read(LR_MUX5_AMUX_THM2, &result);
	if (rc) {
		CHG_HKADC_DEBUG("error reading adc channel = %d, rc = %d\n",
				LR_MUX5_AMUX_THM2, rc);
		rc = oem_hkadc_master_data_read(POWER_SUPPLY_PROP_OEM_SUBSTRATE_THERM, therm);
		CHG_HKADC_DEBUG("hkadc get substrate_therm do not updated. %d %d\n", rc, *therm);
		return rc;
	}
	work = (int)result.physical;

	for (cnt=0;cnt<HKADC_SUBSTRATE_THERM_ELEMENT_COUNT-1;cnt++) {
		substrate_therm_work[cnt] = substrate_therm_work[cnt+1];
		total = total + substrate_therm_work[cnt];
	}
	substrate_therm_work[cnt] = work;
	total = total + substrate_therm_work[cnt];

	max = substrate_therm_work[0];
	min = substrate_therm_work[0];
	for (cnt=0;cnt<HKADC_SUBSTRATE_THERM_ELEMENT_COUNT;cnt++) {
		if (max < substrate_therm_work[cnt]) {
			max = substrate_therm_work[cnt];
		} else if (min > substrate_therm_work[cnt]) {
			min = substrate_therm_work[cnt];
		}
	}

	average = (total - max - min) / (HKADC_SUBSTRATE_THERM_ELEMENT_COUNT-2);
	oem_hkadc_master_data_write(POWER_SUPPLY_PROP_OEM_SUBSTRATE_THERM, average);
	*therm = average;
	CHG_HKADC_DEBUG("hkadc substrate therm total = %d max = %d min = %d \n", total, max, min);
	CHG_HKADC_DEBUG("hkadc substrate therm = %d   %d %d %d %d %d \n", 
		average,
		substrate_therm_work[0],
		substrate_therm_work[1],
		substrate_therm_work[2],
		substrate_therm_work[3],
		substrate_therm_work[4]);

	return 0;
}

static int oem_hkadc_get_wakeup_substrate_therm(int *therm)
{
	int rc;
	int cnt;
	int ngcnt;
	int substrate_therm_new=0;
	int max=0, min=0;
	int total=0;
	int average;
	int substrate_therm_work_tmp[HKADC_SUBSTRATE_THERM_ELEMENT_COUNT];
	struct qpnp_vadc_result result;

	ngcnt = 0;
	for (cnt=0;cnt<HKADC_SUBSTRATE_THERM_ELEMENT_COUNT;cnt++) {
		rc = qpnp_vadc_read(LR_MUX5_AMUX_THM2, &result);
		if (rc) {
			CHG_HKADC_ERR("error reading adc channel = %d, rc = %d\n",
					LR_MUX5_AMUX_THM2, rc);
			ngcnt++;
		}else{
			substrate_therm_new = (int)result.physical;
			substrate_therm_work_tmp[cnt] = (int)result.physical;
		}
	}
	if (ngcnt == 0){
		for (cnt=0;cnt<HKADC_SUBSTRATE_THERM_ELEMENT_COUNT;cnt++) {
			substrate_therm_work[cnt]=substrate_therm_work_tmp[cnt];
			total = total + substrate_therm_work_tmp[cnt];
		}
		max = substrate_therm_work[0];
		min = substrate_therm_work[0];
		for (cnt=0;cnt<HKADC_SUBSTRATE_THERM_ELEMENT_COUNT;cnt++) {
			if (max < substrate_therm_work[cnt]) {
				max = substrate_therm_work[cnt];
			} else if (min > substrate_therm_work[cnt]) {
				min = substrate_therm_work[cnt];
			}
		}

		average = (total - max - min) / (HKADC_SUBSTRATE_THERM_ELEMENT_COUNT-2);
		oem_hkadc_master_data_write(POWER_SUPPLY_PROP_OEM_SUBSTRATE_THERM, average);
		*therm = average;
		CHG_HKADC_DEBUG("hkadc wakeup substrate_therm total = %d max = %d min = %d \n", total, max, min);
		CHG_HKADC_DEBUG("hkadc wakeup substrate_therm = %d   %d %d %d %d %d \n", 
			average,
			substrate_therm_work[0],
			substrate_therm_work[1],
			substrate_therm_work[2],
			substrate_therm_work[3],
			substrate_therm_work[4]);
	}else if (ngcnt < HKADC_SUBSTRATE_THERM_ELEMENT_COUNT){
		for (cnt=0;cnt<HKADC_SUBSTRATE_THERM_ELEMENT_COUNT;cnt++) {
			substrate_therm_work[cnt] = substrate_therm_new;
		}
		oem_hkadc_master_data_write(POWER_SUPPLY_PROP_OEM_SUBSTRATE_THERM, substrate_therm_new);
		*therm = substrate_therm_new;
		CHG_HKADC_DEBUG("hkadc wakeup substrate_therm_new = %d\n", substrate_therm_new);
	}else{
		rc = oem_hkadc_master_data_read(POWER_SUPPLY_PROP_OEM_SUBSTRATE_THERM, therm);
		CHG_HKADC_DEBUG("hkadc wakeup substrate_therm do not updated. %d %d\n", rc, *therm);
	}

	return 0;
}

static int oem_hkadc_init_usb_therm(void)
{
	int rc;
	int work=0;
	int cnt;

	struct qpnp_vadc_result result;

	rc = qpnp_vadc_read(LR_MUX9_AMUX_THM5, &result);
	if (rc) {
		CHG_HKADC_DEBUG("error reading adc channel = %d, rc = %d\n",
				LR_MUX9_AMUX_THM5, rc);
		return rc;
	}
	work = (int)result.physical;

	for (cnt=0;cnt<HKADC_USB_THERM_ELEMENT_COUNT;cnt++) {
		usb_therm_work[cnt] = work;
	}

	oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_USB_THERM] = work;

	CHG_HKADC_DEBUG("init hkadc usb therm = %d \n",
		oem_hkadc_master_data[OEM_POWER_SUPPLY_PROP_IDX_USB_THERM]);

	return 0;
}

static int oem_hkadc_get_usb_therm(int *therm)
{
	int rc;
	int work=0;
	int max=0, min=0;
	int total=0;
	int cnt;
	int average;

	struct qpnp_vadc_result result;

	rc = qpnp_vadc_read(LR_MUX9_AMUX_THM5, &result);
	if (rc) {
		CHG_HKADC_DEBUG("error reading adc channel = %d, rc = %d\n",
				LR_MUX9_AMUX_THM5, rc);
		rc = oem_hkadc_master_data_read(POWER_SUPPLY_PROP_OEM_USB_THERM, therm);
		CHG_HKADC_DEBUG("hkadc get usb_therm do not updated. %d %d\n", rc, *therm);
		return rc;
	}
	work = (int)result.physical;

	for (cnt=0;cnt<HKADC_USB_THERM_ELEMENT_COUNT-1;cnt++) {
		usb_therm_work[cnt] = usb_therm_work[cnt+1];
		total = total + usb_therm_work[cnt];
	}
	usb_therm_work[cnt] = work;
	total = total + usb_therm_work[cnt];

	max = usb_therm_work[0];
	min = usb_therm_work[0];
	for (cnt=0;cnt<HKADC_USB_THERM_ELEMENT_COUNT;cnt++) {
		if (max < usb_therm_work[cnt]) {
			max = usb_therm_work[cnt];
		} else if (min > usb_therm_work[cnt]) {
			min = usb_therm_work[cnt];
		}
	}

	average = (total - max - min) / (HKADC_USB_THERM_ELEMENT_COUNT-2);
	oem_hkadc_master_data_write(POWER_SUPPLY_PROP_OEM_USB_THERM, average);
	*therm = average;
	CHG_HKADC_DEBUG("hkadc usb therm total = %d max = %d min = %d \n", total, max, min);
	CHG_HKADC_DEBUG("hkadc usb therm = %d   %d %d %d %d %d \n", 
		average,
		usb_therm_work[0],
		usb_therm_work[1],
		usb_therm_work[2],
		usb_therm_work[3],
		usb_therm_work[4]);

	return 0;
}

static int oem_hkadc_get_wakeup_usb_therm(int *therm)
{
	int rc;
	int cnt;
	int ngcnt;
	int usb_therm_new=0;
	int max=0, min=0;
	int total=0;
	int average;
	int usb_therm_work_tmp[HKADC_USB_THERM_ELEMENT_COUNT];
	struct qpnp_vadc_result result;

	ngcnt = 0;
	for (cnt=0;cnt<HKADC_USB_THERM_ELEMENT_COUNT;cnt++) {
		rc = qpnp_vadc_read(LR_MUX9_AMUX_THM5, &result);
		if (rc) {
			CHG_HKADC_ERR("error reading adc channel = %d, rc = %d\n",
					LR_MUX9_AMUX_THM5, rc);
			ngcnt++;
		}else{
			usb_therm_new = (int)result.physical;
			usb_therm_work_tmp[cnt] = (int)result.physical;
		}
	}
	if (ngcnt == 0){
		for (cnt=0;cnt<HKADC_USB_THERM_ELEMENT_COUNT;cnt++) {
			usb_therm_work[cnt]=usb_therm_work_tmp[cnt];
			total = total + usb_therm_work_tmp[cnt];
		}
		max = usb_therm_work[0];
		min = usb_therm_work[0];
		for (cnt=0;cnt<HKADC_USB_THERM_ELEMENT_COUNT;cnt++) {
			if (max < usb_therm_work[cnt]) {
				max = usb_therm_work[cnt];
			} else if (min > usb_therm_work[cnt]) {
				min = usb_therm_work[cnt];
			}
		}

		average = (total - max - min) / (HKADC_USB_THERM_ELEMENT_COUNT-2);
		oem_hkadc_master_data_write(POWER_SUPPLY_PROP_OEM_USB_THERM, average);
		*therm = average;
		CHG_HKADC_DEBUG("hkadc wakeup usb_therm total = %d max = %d min = %d \n", total, max, min);
		CHG_HKADC_DEBUG("hkadc wakeup usb_therm = %d   %d %d %d %d %d \n", 
			average,
			usb_therm_work[0],
			usb_therm_work[1],
			usb_therm_work[2],
			usb_therm_work[3],
			usb_therm_work[4]);
	}else if (ngcnt < HKADC_USB_THERM_ELEMENT_COUNT){
		for (cnt=0;cnt<HKADC_USB_THERM_ELEMENT_COUNT;cnt++) {
			usb_therm_work[cnt] = usb_therm_new;
		}
		oem_hkadc_master_data_write(POWER_SUPPLY_PROP_OEM_USB_THERM, usb_therm_new);
		*therm = usb_therm_new;
		CHG_HKADC_DEBUG("hkadc wakeup usb_therm_new = %d\n", usb_therm_new);
	}else{
		rc = oem_hkadc_master_data_read(POWER_SUPPLY_PROP_OEM_USB_THERM, therm);
		CHG_HKADC_DEBUG("hkadc wakeup usb_therm do not updated. %d %d\n", rc, *therm);
	}

	return 0;
}

int oem_hkadc_pm_batt_power_get_property(enum power_supply_property psp, int *intval)
{
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_OEM_PA_THERM:
	case POWER_SUPPLY_PROP_OEM_CAMERA_THERM:
	case POWER_SUPPLY_PROP_OEM_SUBSTRATE_THERM:
	case POWER_SUPPLY_PROP_OEM_USB_THERM:
		ret = oem_hkadc_master_data_read(psp, intval);
		break;

	case POWER_SUPPLY_PROP_TEMP:
		if ((POWER_SUPPLY_PROP_TEMP == psp) &&
		    (oem_param_share.factory_mode_1)) {
			*intval = 250;
			ret = 0;
		}else {
			ret = oem_hkadc_master_data_read(psp, intval);
		}
		break;

	default:
		ret = -EINVAL;
	}
	return ret;
}
EXPORT_SYMBOL(oem_hkadc_pm_batt_power_get_property);

static bool is_hkadc_suspend_monit = 0;

static void oem_hkadc_isr_work(struct work_struct *work)
{
	pr_info("%s : called\n", __func__);
	wake_lock(&oem_hkadc_wake_lock);
	is_hkadc_suspend_monit = 1;
}

static irqreturn_t oem_hkadc_isr(int irq, void *dev)
{
	struct oem_hkadc_device *oem_hkadc_dev = dev;
	pr_info("%s : interrupt !!\n", __func__);
	schedule_work(&oem_hkadc_dev->irq_work);

	return IRQ_HANDLED;
}

static __devinit int oem_hkadc_probe(struct platform_device *pdev)
{
	int rc = 0;

	if (!pdev->dev.of_node) {
		CHG_HKADC_ERR("No platform supplied from device tree.\n");
		rc = -EINVAL;
		goto err_arg;
	}

	oem_hkadc_dev = kzalloc(sizeof(struct oem_hkadc_device),
								GFP_KERNEL);
	if (!oem_hkadc_dev) {
		CHG_HKADC_ERR("kzalloc fail\n");
		rc = -ENOMEM;
		goto err_alloc;
	}

	oem_hkadc_dev->gpio = of_get_named_gpio(pdev->dev.of_node,
							"oem_hkadc-gpio", 0);
	if (oem_hkadc_dev->gpio < 0) {
		CHG_HKADC_ERR("%s: of_get_named_gpio failed.\n", __func__);
		rc = -EINVAL;
		goto err_gpio;
	}

	rc = gpio_request(oem_hkadc_dev->gpio, "oem_hkadc-gpio");
	if (rc) {
		CHG_HKADC_ERR("%s: gpio_request failed.\n", __func__);
		goto err_gpio;
	}

	if(OEM_get_board() == OEM_BOARD6_TYPE) {
		struct qpnp_pin_cfg oem_hkadc_pin_cfg = {
			.mode			= QPNP_PIN_MODE_DIG_IN,
			.output_type	= QPNP_PIN_OUT_BUF_CMOS,
			.invert			= QPNP_PIN_INVERT_DISABLE,
			.pull			= QPNP_PIN_GPIO_PULL_DN,
			.vin_sel		= QPNP_PIN_VIN2,
			.out_strength	= QPNP_PIN_OUT_STRENGTH_LOW,
			.src_sel		= QPNP_PIN_SEL_FUNC_CONSTANT,
			.master_en		= QPNP_PIN_MASTER_ENABLE,
		};
		qpnp_pin_config(oem_hkadc_dev->gpio, &oem_hkadc_pin_cfg);
		CHG_HKADC_ERR("%s: OEM_BOARD6_TYPE\n", __func__);
	}

	oem_hkadc_dev->irq = gpio_to_irq(oem_hkadc_dev->gpio);

	rc = request_irq(oem_hkadc_dev->irq, oem_hkadc_isr,
			IRQF_TRIGGER_RISING, "oem_hkadc-irq", oem_hkadc_dev);
	if (rc) {
		CHG_HKADC_ERR("%s: failed request_irq.\n", __func__);
		goto err_irq;
	}

	enable_irq_wake(oem_hkadc_dev->irq);

	INIT_WORK(&oem_hkadc_dev->irq_work, oem_hkadc_isr_work);

	return 0;

err_irq:
	gpio_free(oem_hkadc_dev->gpio);
err_gpio:
	kfree(oem_hkadc_dev);
err_alloc:
err_arg:
	CHG_HKADC_ERR("%s: failed.\n", __func__);

	return rc;
}

static int __devexit oem_hkadc_remove(struct platform_device *pdev)
{
	free_irq(oem_hkadc_dev->irq, oem_hkadc_dev);
	gpio_free(oem_hkadc_dev->gpio);
	kfree(oem_hkadc_dev);

	return 0;
}

static const struct of_device_id oem_hkadc_of_match[] = {
	{ .compatible = "kc,oem_hkadc-driver", },
};

static struct platform_driver oem_hkadc_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = oem_hkadc_of_match,
	},
	.probe = oem_hkadc_probe,
	.remove = __devexit_p(oem_hkadc_remove),
};

static void oem_hkadc_monitor(struct work_struct *work)
{
	int temp  = 0;
	int camera_therm  = 0;
	int substrate_therm = 0;
	int usb_therm = 0;
	int rc = 0;
	union power_supply_propval status;

	if (is_hkadc_suspend_monit){
		oem_hkadc_get_wakeup_battery_uvolts(&oem_vbat_value);
		oem_hkadc_get_wakeup_battery_temp(&temp);
		oem_hkadc_get_wakeup_pa_therm();
		oem_hkadc_get_wakeup_camera_therm(&camera_therm);
		oem_hkadc_get_wakeup_substrate_therm(&substrate_therm);
		oem_hkadc_get_wakeup_usb_therm(&usb_therm);

//		oem_battery_volts_set_status(oem_vbat_value);
//		oem_battery_temp_set_status(temp);
//		oem_camera_therm_set_status(camera_therm);
//		oem_substrate_therm_set_status(substrate_therm);
//		oem_usb_therm_set_status(usb_therm);

//		oem_bms_low_vol_detect_standby(temp);

		pa_therm_monit_freq = 0;
		camera_temp_monit_freq = 0;
		substrate_therm_monit_freq = 0;
		usb_therm_monit_freq = 0;
		uevent_notify_freq = HKADC_UEVENT_NOTIFY_COUNT;
	}else{
		rc = oem_hkadc_get_battery_uvolts(&oem_vbat_value);
		rc |= oem_hkadc_get_battery_temp(&temp);
//		oem_battery_volts_set_status(oem_vbat_value);
//		oem_battery_temp_set_status(temp);

		pa_therm_monit_freq++;
		camera_temp_monit_freq++;
		substrate_therm_monit_freq++;
		usb_therm_monit_freq++;
		uevent_notify_freq++;

		if (pa_therm_monit_freq >= HKADC_PA_THERM_MONIT_FREQ){
			pa_therm_monit_freq=0;
			oem_hkadc_get_pa_therm();
		}
		if (camera_temp_monit_freq >= HKADC_CAMERA_TEMP_MONIT_FREQ){
			camera_temp_monit_freq=0;
			oem_hkadc_get_camera_therm(&camera_therm);
//			oem_camera_therm_set_status(camera_therm);
		}
		if (substrate_therm_monit_freq >= HKADC_SUBSTRATE_THERM_MONIT_FREQ){
			substrate_therm_monit_freq=0;
			oem_hkadc_get_substrate_therm(&substrate_therm);
//			oem_substrate_therm_set_status(substrate_therm);
		}
		if (usb_therm_monit_freq >= HKADC_USB_THERM_MONIT_FREQ){
			usb_therm_monit_freq=0;
			oem_hkadc_get_usb_therm(&usb_therm);
//			oem_usb_therm_set_status(usb_therm);
		}

		oem_bms_low_vol_detect_active(oem_vbat_value, temp, rc);
	}

//	if (the_chip->dc_present || the_chip->usb_present || oem_stand_detect) {
//		oem_charger_check_battery_detection_state();
//	}

//	oem_batt_health = oem_hkadc_get_battery_property(POWER_SUPPLY_PROP_HEALTH);

//	oem_charger_monitor();

	oem_charger_get_connect_state();

//	oem_chg_check_charger_removal_after_timer_expired();

//	oem_chg_recharge_check_after_timer_expired();

//	oem_chg_check_adapter_overvoltage_after_timer_expired();

//	oem_chg_vbat_ov_check_after_timer_expired();

//	oem_chg_check_battery_detection_after_timer_expired();

//	oem_judge_charge_state();

//	oem_bms_calculate_cyclecorrection();

	oem_chg_check_vbatt_ov();

	if(uevent_notify_freq >= HKADC_UEVENT_NOTIFY_COUNT) {
		uevent_notify_freq=0;
		if(!hkadc_batt_psy)
			hkadc_batt_psy = power_supply_get_by_name("battery");

		if(hkadc_batt_psy)
			power_supply_changed(hkadc_batt_psy);
	}
	else {
		if(!hkadc_batt_psy)
			hkadc_batt_psy = power_supply_get_by_name("battery");

		if(hkadc_batt_psy)
			if(!(hkadc_batt_psy->get_property(hkadc_batt_psy, POWER_SUPPLY_PROP_STATUS, &status))) {
				if(status.intval != hold_status.intval) {
					power_supply_changed(hkadc_batt_psy);
					if(status.intval == POWER_SUPPLY_STATUS_CHARGING) {
						oem_chg_set_vbat_det_low(the_chip);
					}
				}
				hold_status.intval = status.intval;
			}
	}

	if (is_hkadc_suspend_monit){
		wake_unlock(&oem_hkadc_wake_lock);
		is_hkadc_suspend_monit = 0;
	}
//	power_supply_changed(&the_chip->usb_psy);
//	power_supply_changed(&the_chip->dc_psy);
//	power_supply_changed(&the_chip->bms_psy);

//	alarm_cancel(&androidalarm);
//	androidalarm_interval_timespec = ktime_to_timespec(alarm_get_elapsed_realtime());
//	androidalarm_interval_timespec.tv_sec += HKADC_SUSPEND_MONIT_FREQ;
//	androidalarm_interval_timespec.tv_nsec = 0;
//	alarm_start_range(&androidalarm, timespec_to_ktime(androidalarm_interval_timespec),
//		timespec_to_ktime(androidalarm_interval_timespec));

	oem_chg_set_appropriate_chg_param(the_chip);
	
	schedule_delayed_work(&oem_hkadc_work,
			round_jiffies_relative(msecs_to_jiffies(OEM_HKADC_MONITOR_TIME_1S)));
}

void oem_hkadc_init(void *chip)
{
	the_chip = chip;
	
	memset(&oem_hkadc_master_data, 0x0, sizeof(oem_hkadc_master_data));

	platform_driver_register(&oem_hkadc_driver);

	oem_hkadc_init_battery_uvolts();
	oem_hkadc_init_battery_temp();
	oem_hkadc_init_camera_therm();
	oem_hkadc_init_substrate_therm();
	oem_hkadc_init_pa_therm();
	oem_hkadc_init_usb_therm();

	atomic_set(&is_hkadc_initialized, 1);

	camera_temp_monit_freq = 0;
	substrate_therm_monit_freq = 0;

//	hkadc_early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
//	hkadc_early_suspend.suspend = oem_hkadc_early_suspend;
//	hkadc_early_suspend.resume = oem_hkadc_early_resume;
//	register_early_suspend(&hkadc_early_suspend);

	MASTERDATA_SPINLOCK_INIT();
	wake_lock_init(&oem_hkadc_wake_lock, WAKE_LOCK_SUSPEND, "oem_hkadc");

	INIT_DELAYED_WORK(&oem_hkadc_work, oem_hkadc_monitor);
	schedule_delayed_work(&oem_hkadc_work,
			round_jiffies_relative(msecs_to_jiffies(0)));

//	alarm_init(&androidalarm,
//		ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP,
//		oem_hkadc_alarm_handler);
}
EXPORT_SYMBOL(oem_hkadc_init);

void oem_hkadc_exit(void)
{
//	int rc;

//	rc = alarm_cancel(&androidalarm);
//	CHG_HKADC_DEBUG("alarm_cancel result=%d\n", rc);
	platform_driver_unregister(&oem_hkadc_driver);

	atomic_set(&is_hkadc_initialized, 0);
}
EXPORT_SYMBOL(oem_hkadc_exit);

int oem_get_vbatt_value(void)
{
 	return oem_vbat_value;
}
EXPORT_SYMBOL(oem_get_vbatt_value);

