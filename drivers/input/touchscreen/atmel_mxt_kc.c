/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2012 KYOCERA Corporation
 * (C) 2013 KYOCERA Corporation
 * (C) 2016 KYOCERA Corporation
 *
 * drivers/input/touchscreen/atmel_mxt_kc.c
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 * Copyright (c) 2011-2013, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
*/

#undef MXT_DUMP_OBJECT

#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/i2c/atmel_mxt_kc.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/gpio.h>

#include <linux/fs.h>
#include <linux/namei.h>
#include <asm/uaccess.h>
#include <linux/cdev.h>
#include <linux/regulator/consumer.h>

#include "board-8974-touch.h"

/* Family ID */
#define MXT224_ID		0x80
#define MXT224E_ID		0x81
#define MXT1386_ID		0xA0
#define MXT224S_FAMILY_ID	0x82

/* Variant ID */
#define MXT224S_VARIANT_ID	0x1a
#define MXT336S_VARIANT_ID	0x19

/* Version */
#define MXT_FW_V1_0		0x10
#define MXT_FW_V1_9		0x19
#define MXT_FW_V3_0		0x30

/* Build */
#define MXT224S_BUILD		0xAA

/* Slave addresses */
#define MXT_APP_LOW		0x4a
#define MXT_APP_HIGH		0x4b
#define MXT_BOOT_LOW		0x24
#define MXT_BOOT_HIGH		0x25

/* Firmware */
#define MXT_FW_NAME		"maxtouch.fw"

/* Registers */
#define MXT_FAMILY_ID		0x00
#define MXT_VARIANT_ID		0x01
#define MXT_VERSION		0x02
#define MXT_BUILD		0x03
#define MXT_MATRIX_X_SIZE	0x04
#define MXT_MATRIX_Y_SIZE	0x05
#define MXT_OBJECT_NUM		0x06
#define MXT_OBJECT_START	0x07

#define MXT_OBJECT_SIZE		6

/* Object types */
#define MXT_DEBUG_DIAGNOSTIC_T37	37
#define MXT_GEN_MESSAGE_T5		5
#define MXT_GEN_COMMAND_T6		6
#define MXT_SPT_USERDATA_T38		38
#define SPT_DYNAMICCONFIGURATIONCONTAINER_T71	71
#define MXT_GEN_POWER_T7		7
#define MXT_GEN_ACQUIRE_T8		8
#define MXT_TOUCH_MULTI_T9		9
#define MXT_SPT_COMMSCONFIG_T18		18
#define MXT_SPT_SELFTEST_T25		25
#define MXT_PROCI_TOUCH_T42		42
#define MXT_SPT_CTECONFIG_T46		46
#define MXT_PROCI_STYLUS_T47		47
#define MXT_PROCI_SHIELDLESS_T56	56
#define MXT_PROCI_EXTRA_T57		57
#define MXT_SPT_TIMER_T61		61
#define MXT_PROCI_LENSBENDING_T65	65
#define MXT_SPT_GOLDENREFERENCES_T66	66
#define SPT_DYNAMICCONFIGURATIONCONTROLLER_T70	70
#define MXT_NOISESUPPRESIION_T72	72
#define MXT_RETRANSMISSION_T80	80

/* MXT_GEN_COMMAND_T6 field */
#define MXT_COMMAND_RESET		0
#define MXT_COMMAND_BACKUPNV		1
#define MXT_COMMAND_CALIBRATE		2
#define MXT_COMMAND_REPORTALL		3
#define MXT_COMMAND_DIAGNOSTIC		5

/* MXT_GEN_POWER_T7 field */
#define MXT_POWER_IDLEACQINT		0
#define MXT_POWER_ACTVACQINT		1
#define MXT_POWER_ACTV2IDLETO		2
#define MXT_POWER_CFG			3

/* MXT_T7 CMD field */
#define MXT_FREE_RUN_MODE		0xFF

/* MXT_GEN_ACQUIRE_T8 field */
#define MXT_ACQUIRE_CHRGTIME		0
#define MXT_ACQUIRE_TCHDRIFT		2
#define MXT_ACQUIRE_DRIFTST		3
#define MXT_ACQUIRE_TCHAUTOCAL		4
#define MXT_ACQUIRE_SYNC		5
#define MXT_ACQUIRE_ATCHCALST		6
#define MXT_ACQUIRE_ATCHCALSTHR		7
#define MXT_ACQUIRE_ATCHFRCCALTHR	8
#define MXT_ACQUIRE_ATCHFRCCALRATIO	9

/* MXT_TOUCH_MULT_T9 field */
#define MXT_TOUCH_CTRL			0
#define MXT_TOUCH_XORIGIN		1
#define MXT_TOUCH_YORIGIN		2
#define MXT_TOUCH_XSIZE			3
#define MXT_TOUCH_YSIZE			4
#define MXT_TOUCH_AKSCFG		5
#define MXT_TOUCH_BLEN			6
#define MXT_TOUCH_TCHTHR		7
#define MXT_TOUCH_TCHDI			8
#define MXT_TOUCH_ORIENT		9
#define MXT_TOUCH_MRGTIMEOUT		10
#define MXT_TOUCH_MOVHYSTI		11
#define MXT_TOUCH_MOVHYSTN		12
#define MXT_TOUCH_MOVFILTER		13
#define MXT_TOUCH_NUMTOUCH		14
#define MXT_TOUCH_MRGHYST		15
#define MXT_TOUCH_MRGTHR		16
#define MXT_TOUCH_AMPHYST		17
#define MXT_TOUCH_XRANGE_LSB		18
#define MXT_TOUCH_XRANGE_MSB		19
#define MXT_TOUCH_YRANGE_LSB		20
#define MXT_TOUCH_YRANGE_MSB		21
#define MXT_TOUCH_XLOCLIP		22
#define MXT_TOUCH_XHICLIP		23
#define MXT_TOUCH_YLOCLIP		24
#define MXT_TOUCH_YHICLIP		25
#define MXT_TOUCH_XEDGECTRL		26
#define MXT_TOUCH_XEDGEDIST		27
#define MXT_TOUCH_YEDGECTRL		28
#define MXT_TOUCH_YEDGEDIST		29
#define MXT_TOUCH_JUMPLIMIT		30
#define MXT_TOUCH_TCHHYST		31
#define MXT_TOUCH_XPITCH		32
#define MXT_TOUCH_YPITCH		33
#define MXT_TOUCH_NEXTTCHDI		34
#define MXT_TOUCH_CFG			35

/* MXT_T37 field */
#define MXT_DELTA					0x10
#define MXT_CURRENT_REFERENCE		0x11

/* MXT_PROCI_TOUCH_T42 field */
#define MXT_SUPPRESSION_CTRL		0
#define MXT_SUPPRESSION_APPRTHR		1
#define MXT_SUPPRESSION_MAXAPPRAREA	2
#define MXT_SUPPRESSION_MAXTCHAREA	3
#define MXT_SUPPRESSION_SUPSTRENGTH	4
#define MXT_SUPPRESSION_SUPEXTTO	5
#define MXT_SUPPRESSION_MAXNUMTCHS	6
#define MXT_SUPPRESSION_SHAPESTRENGTH	7
#define MXT_SUPPRESSION_SUPDIST		8
#define MXT_SUPPRESSION_DISTHYST	9

/* MXT_SPT_CTECONFIG_T46 */
#define MXT_SPT_IDLESYNCSPERX		2
#define MXT_ACTVSYNCSPERX			3

/* MXT_PROCI_STYLUS_T47 field */
#define MXT_STYLUS_CTRL			0
#define MXT_STYLUS_CONTMIN		1
#define MXT_STYLUS_CONTMAX		2
#define MXT_STYLUS_STABILITY		3
#define MXT_STYLUS_MAXCHAREA		4
#define MXT_STYLUS_AMPLTHR		5
#define MXT_STYLUS_STYSHAPE		6
#define MXT_STYLUS_HOVERSUP		7
#define MXT_STYLUS_CONFTHR		8
#define MXT_STYLUS_SYNCSPERX		9
#define MXT_STYLUS_XPOSADJ		10
#define MXT_STYLUS_YPOSADJ		11
#define MXT_STYLUS_CFG			12

/* MXT_PROCI_SHIELDLESS_T56 field */
#define MXT_SHIELDLESS_CTRL		0
#define MXT_SHIELDLESS_ORIENT		2
#define MXT_SHIELDLESS_INTTIME		3
#define MXT_SHIELDLESS_INTDELAY_0	4
#define MXT_SHIELDLESS_MULTICUTGC	5
#define MXT_SHIELDLESS_GCLIMIT		6
#define MXT_SHIELDLESS_NCNCL		7
#define MXT_SHIELDLESS_TOUCHBIAS	8
#define MXT_SHIELDLESS_BASESCALE	9
#define MXT_SHIELDLESS_SHIFTLIMIT	10
#define MXT_SHIELDLESS_YLONOISEMUL_LSB	11
#define MXT_SHIELDLESS_YLONOISEMUL_MSB	12
#define MXT_SHIELDLESS_YLONOISEDIV_LSB	13
#define MXT_SHIELDLESS_YLONOISEDIV_MSB	14
#define MXT_SHIELDLESS_YHINOISEMUL_LSB	15
#define MXT_SHIELDLESS_YHINOISEMUL_MSB	16
#define MXT_SHIELDLESS_YHINOISEDIV_LSB	17
#define MXT_SHIELDLESS_YHINOISEDIV_MSB	18

/* MXT_PROCI_EXTRA_T57 field */
#define MXT_EXTRA_CTRL			0
#define MXT_EXTRA_AREATHR		1
#define MXT_EXTRA_AREAHYST		2

/* MXT_SPT_COMMSCONFIG_T18 */
#define MXT_COMMS_CTRL		0
#define MXT_COMMS_CMD		1

/* MXT_SPT_GOLDENREFERENCES_T66 */
#define MXT_SPT_CTRL		0
#define MXT_SPT_FCALFAILTHR	1
#define MXT_SPT_FCALDRIFTCNT	2
#define T66_CMD_SIZE			2
#define T66_RSP_SIZE			4

/* MXT_T66_MESSAGE field */
#define MXT_CMD_PRIME			0x04
#define MXT_MESSAGE_PRIME		0x02
#define MXT_CMD_GENERATE		0x08
#define MXT_MESSAGE_GENERATE	0x04
#define MXT_CMD_CONFIRM			0x0c
#define MXT_FCALSEQDONE		0x20
#define MXT_FCALCMD_STATE_MASK	0x0c
#define MXT_FCALSEQTO			0x10
#define MXT_FCALPASS			0x40
#define MXT_FCALFAIL			0x80
#define MXT_FCALSEQERR			0x08

/* MXT_NOISESUPPRESIION_T72 field */
#define MXT_MINNLTHR			0x0a

/* Define for MXT_GEN_COMMAND_T6 */
#define MXT_RESET_ORDER		0x01
#define MXT_CALIBRATE_ORDER	0x01
#define MXT_BOOT_VALUE		0xa5
#define MXT_BACKUP_VALUE	0x55
#define MXT_BACKUP_TIME		25	/* msec */
#define MXT224_RESET_TIME	65	/* msec */
#define MXT224E_RESET_TIME	22	/* msec */
#define MXT1386_RESET_TIME	250	/* msec */
#define MXT224S_RESET_TIME	250	/* msec */
#define MXT_RESET_TIME		250	/* msec */
#define MXT_MAX_RESET_TIME	160	/* msec */
#define MXT_FWRESET_TIME	175	/* msec */
#define MXT_WAKE_TIME		25

/* Command to unlock bootloader */
#define MXT_UNLOCK_CMD_MSB	0xaa
#define MXT_UNLOCK_CMD_LSB	0xdc

/* Bootloader mode status */
#define MXT_WAITING_BOOTLOAD_CMD	0xc0	/* valid 7 6 bit only */
#define MXT_WAITING_FRAME_DATA		0x80	/* valid 7 6 bit only */
#define MXT_FRAME_CRC_CHECK		0x02
#define MXT_FRAME_CRC_FAIL		0x03
#define MXT_FRAME_CRC_PASS		0x04
#define MXT_APP_CRC_FAIL		0x40	/* valid 7 8 bit only */
#define MXT_BOOT_STATUS_MASK		0x3f
#define MXT_BOOT_EXTENDED_ID	(1 << 5)
#define MXT_BOOT_ID_MASK	0x1f
#define MXT_CTRL		0
#define MXT_RPTEN		(1 << 1)

/* Current status */
#define MXT_COMSERR		(1 << 2)
#define MXT_CFGERR		(1 << 3)
#define MXT_CAL			(1 << 4)
#define MXT_SIGERR		(1 << 5)
#define MXT_OFL			(1 << 6)
#define MXT_RESET		(1 << 7)

/* Touch status */
#define MXT_SUPPRESS		(1 << 1)
#define MXT_AMP			(1 << 2)
#define MXT_VECTOR		(1 << 3)
#define MXT_MOVE		(1 << 4)
#define MXT_RELEASE		(1 << 5)
#define MXT_PRESS		(1 << 6)
#define MXT_DETECT		(1 << 7)

/* Touch orient bits */
#define MXT_XY_SWITCH		(1 << 0)
#define MXT_X_INVERT		(1 << 1)
#define MXT_Y_INVERT		(1 << 2)

/* Touchscreen absolute values */
#define MXT_MAX_RW_TRIES	3
#define MXT_BLOCK_SIZE		256
#define MXT_MAX_RST_TRIES	2
#define MXT_MAX_ERR_CNT		255
#define MXT_MAX_CHK_TRIES	2

/* Firmware frame size including frame data and CRC */
#define MXT_SINGLE_FW_MAX_FRAME_SIZE	278

static struct regulator *reg_touch;

struct mxt_message {
	u8 reportid;
	u8 message[7];
};

struct mxt_address_pair {
	int bootloader;
	int application;
};

static const struct mxt_address_pair mxt_slave_addresses[] = {
	{ 0x24, 0x4a },
	{ 0 },
};

/* t37 data pointer for debug mode */
u8 *t37_data = NULL;

bool file_header = false;
int mxt_recovery_cnt = 0;

#ifdef FEATURE_TOUCH_TEST
static int mxt_log_object(struct kc_ts_data *data, struct file *fp);
#endif
static int mxt_enable(struct kc_ts_data *data);
static void mxt_disable(struct kc_ts_data *data);
static int mxt_calibrate(struct kc_ts_data *data);

#ifdef FEATURE_TOUCH_TEST
static void mxt_write_log(struct kc_ts_data *data, enum kc_ts_log_type type,
						void *arg)
{
	struct file *fp;
	mm_segment_t old_fs;
	char *filename = "/data/local/tmp/ts_log";
	char buf[100];
	int len;
	int error;
	struct path path;
	struct mxt_message *message;
	struct ts_log_data *log;
	unsigned long long t;
	unsigned long nanosec_rem;
	u8 *val;
	int *id;

	switch (ts_log_file_enable) {
	case 0:
		return;
	case 1:
		switch (type) {
		case KC_TS_LOG_REPORT:
		case KC_TS_LOG_INTERRUPT_START:
		case KC_TS_LOG_INTERRUPT_END:
		case KC_TS_LOG_READ_MESSAGE:
			return;
		default:
			break;
		}
		break;
	case 2:
		switch (type) {
		case KC_TS_LOG_INTERRUPT_START:
		case KC_TS_LOG_INTERRUPT_END:
		case KC_TS_LOG_READ_MESSAGE:
			return;
		default:
			break;
		}
		break;
	case 3:
		switch (type) {
		case KC_TS_LOG_MSG:
		case KC_TS_LOG_INTERRUPT_START:
		case KC_TS_LOG_INTERRUPT_END:
		case KC_TS_LOG_READ_MESSAGE:
			return;
		default:
			break;
		}
		break;
	default:
		break;
	}

	mutex_lock(&file_lock);
	/* change to KERNEL_DS address limit */
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	error = kern_path(filename, LOOKUP_FOLLOW, &path);
	if(error) {
		fp = filp_open(filename, O_CREAT, S_IRUGO|S_IWUSR);
		KC_TS_DEV_DBG("created /data/local/tmp/ts_log.\n");
		file_header = false;
	} else
		fp = filp_open(filename, O_WRONLY | O_APPEND, 0);

	if (!file_header) {
		len = sprintf(buf, "Time,Core,Report ID,Data01,Data02,"
					"Data03,Data04,Data05,Data06,Data07\n");
		fp->f_op->write(fp, buf, len, &(fp->f_pos));
		memset(buf, '\0', sizeof(buf));
		file_header = true;
	}

	t = cpu_clock(smp_processor_id());
	nanosec_rem = do_div(t, 1000000000);
	len = sprintf(buf, "[%5lu.%06lu],Core[%d],", (unsigned long) t,
					nanosec_rem / 1000, smp_processor_id());
	fp->f_op->write(fp, buf, len, &(fp->f_pos));
	memset(buf, '\0', sizeof(buf));

	switch (type) {
	case KC_TS_LOG_MSG:
		message = arg;
		len = sprintf(buf, "%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X\n",
							message->reportid,
							message->message[0],
							message->message[1],
							message->message[2],
							message->message[3],
							message->message[4],
							message->message[5],
							message->message[6]);
		break;
	case KC_TS_LOG_CONF_STAT:
		switch (data->config_status) {
		case TS_CHARGE_CABLE:
			len = sprintf(buf, "Change config_status to "
							"TS_CHARGE_CABLE.\n");
			break;
		case TS_CHARGE_A_S1:
			len = sprintf(buf, "Change config_status to "
							"TS_CHARGE_A_S1.\n");
			break;
		case TS_CHARGE_A_S2:
			len = sprintf(buf, "Change config_status to "
							"TS_CHARGE_A_S2.\n");
			break;
		case TS_DISCHARGE:
			len = sprintf(buf, "Change config_status to "
							"TS_DISCHARGE.\n");
			break;
		default:
			break;
		}
		mxt_log_object(data, fp);
		break;
	case KC_TS_LOG_RESTART:
		len = sprintf(buf, "Controller IC Restart!\n");
		break;
	case KC_TS_LOG_RESET_STATUS:
		len = sprintf(buf, "Reset status in Touch Screen Driver!\n");
		break;
	case KC_TS_LOG_ESD:
		len = sprintf(buf, "Can't Read Register!\n");
		break;
	case KC_TS_LOG_DAEMON:
		log = arg;
		len = sprintf(buf, "ts_daemon log[%X][%X]\n", log->flag, log->data);
		break;
	case KC_TS_LOG_SUSPEND:
		len = sprintf(buf, "Suspend.\n");
		break;
	case KC_TS_LOG_RESUME:
		len = sprintf(buf, "Resume.\n");
		break;
	case KC_TS_LOG_CFGERR:
		len = sprintf(buf, "Received CFGERR!\n");
		break;
	case KC_TS_LOG_RPTEN:
		val = arg;
		len = sprintf(buf, "Received T[%d] message even "
					"though RPTEN is not set!\n", *val);
		break;
	case KC_TS_LOG_IC_RESET:
		len = sprintf(buf, "Sensor IC reset!\n");
		break;
	case KC_TS_LOG_REPORT:
		id = arg;
		len = sprintf(buf, "Sync Report,slot[%d],status[%X],x[%d]"
				   ",y[%d],pressure[%d],area[%d]\n",
				   *id,
				   data->finger[*id].status,
				   data->finger[*id].x,
				   data->finger[*id].y,
				   data->finger[*id].pressure,
				   data->finger[*id].area);
		break;
	case KC_TS_LOG_INTERRUPT_START:
		len = sprintf(buf, "mxt_interrupt() is called.\n");
		break;
	case KC_TS_LOG_INTERRUPT_END:
		len = sprintf(buf, "mxt_interrupt() is completed.\n");
		break;
	case KC_TS_LOG_READ_MESSAGE:
		len = sprintf(buf, "mxt_read_message() is completed.\n");
		break;
	default:
		len = sprintf(buf, "[Error] default case is called.\n");
		break;
	}

	fp->f_op->write(fp, buf, len, &(fp->f_pos));

	/* close file before return */
	if (fp)
		filp_close(fp, current->files);
	/* restore previous address limit */
	set_fs(old_fs);
	mutex_unlock(&file_lock);
}

static bool mxt_object_readable(struct kc_ts_data *data, unsigned int type)
{
		return true;
}
#endif

static bool mxt_object_writable(struct kc_ts_data *data, unsigned int type)
{
	switch (type) {
	case MXT_GEN_POWER_T7:
	case MXT_GEN_ACQUIRE_T8:
	case MXT_TOUCH_MULTI_T9:
	case MXT_SPT_COMMSCONFIG_T18:
	case MXT_SPT_SELFTEST_T25:
	case MXT_PROCI_TOUCH_T42:
	case MXT_SPT_CTECONFIG_T46:
	case MXT_PROCI_STYLUS_T47:
	case MXT_PROCI_SHIELDLESS_T56:
	case MXT_PROCI_EXTRA_T57:
	case MXT_SPT_TIMER_T61:
	case MXT_PROCI_LENSBENDING_T65:
	case MXT_SPT_GOLDENREFERENCES_T66:
	case SPT_DYNAMICCONFIGURATIONCONTROLLER_T70:
	case SPT_DYNAMICCONFIGURATIONCONTAINER_T71:
	case MXT_NOISESUPPRESIION_T72:
	case MXT_RETRANSMISSION_T80:
		return true;
	default:
		return false;
	}
}

static bool mxt_object_switchable(struct kc_ts_data *data, unsigned int type)
{
	return true;
}

static bool mxt_object_exist_rpten(struct kc_ts_data *data, unsigned int type)
{
	switch (type) {
	case MXT_TOUCH_MULTI_T9:
	case MXT_PROCI_TOUCH_T42:
	case MXT_PROCI_SHIELDLESS_T56:
	case MXT_PROCI_EXTRA_T57:
	case MXT_SPT_SELFTEST_T25:
	case MXT_SPT_TIMER_T61:
	case MXT_PROCI_LENSBENDING_T65:
	case MXT_SPT_GOLDENREFERENCES_T66:
		return true;
	default:
		return false;
	}
}

static void mxt_dump_message(struct device *dev,
				  struct mxt_message *message)
{
	if (message->reportid != 0xFF) {
		KC_TS_DEV_INFO("%s:reportid: 0x%02x, message: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n"
					, __func__, message->reportid, message->message[0]
					, message->message[1], message->message[2]
					, message->message[3], message->message[4]
					, message->message[5], message->message[6]);
	}
}


static int __mxt_read_reg(struct i2c_client *client,
			       u16 reg, u16 len, void *val)
{
	struct i2c_msg xfer[2];
	u8 buf[2];
	int i = 0;
	u16 j = 0;

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;

	/* Write register */
	xfer[0].addr = client->addr;
	xfer[0].flags = 0;
	xfer[0].len = 2;
	xfer[0].buf = buf;

	/* Read data */
	xfer[1].addr = client->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = len;
	xfer[1].buf = val;

	do {
		if (i2c_transfer(client->adapter, xfer, 2) == 2){
			if(ts_log_level & 0x08){
				for(j=0; j<len; j++){
					printk("%s: Add: %04x, Val: %02x\n",__func__, reg + j, *xfer[1].buf);
					xfer[1].buf++;
				}
			}
			return 0;
		}
		msleep(MXT_WAKE_TIME);
	} while (++i < MXT_MAX_RW_TRIES);

	dev_err(&client->dev, "%s: i2c transfer failed\n", __func__);
	return -EIO;
}

static int mxt_read_reg(struct i2c_client *client, u16 reg, u8 *val)
{
	return __mxt_read_reg(client, reg, 1, val);
}

static int __mxt_write_reg(struct i2c_client *client,
		    u16 addr, u16 length, u8 *value)
{
	u8 buf[MXT_BLOCK_SIZE + 2];
	int i, tries = 0;
	u16 j = 0;

	if (length > MXT_BLOCK_SIZE)
		return -EINVAL;

	buf[0] = addr & 0xff;
	buf[1] = (addr >> 8) & 0xff;
	for (i = 0; i < length; i++)
		buf[i + 2] = *value++;

	do {
		if (i2c_master_send(client, buf, length + 2) == (length + 2)){
			if(ts_log_level & 0x08){
				for(j=0; j<length; j++){
					printk("%s: Add: %04x, Val: %02x\n",__func__, addr+j, buf[j+2]);
				}
			}
			return 0;
		}
		msleep(MXT_WAKE_TIME);
	} while (++tries < MXT_MAX_RW_TRIES);

	client->addr = addr;

	dev_err(&client->dev, "%s: i2c send failed\n", __func__);
	return -EIO;
}

static int mxt_write_reg(struct i2c_client *client, u16 reg, u8 val)
{
	return __mxt_write_reg(client, reg, 1, &val);
}

static int mxt_bundle_write_reg(struct kc_ts_data *data, u16 size, u16 addr, u8 *dp)
{
	int ret;
	while (size > MXT_BLOCK_SIZE) {
		ret = __mxt_write_reg(data->vdata->client, addr, MXT_BLOCK_SIZE, dp);
		if (ret)
			return ret;
		size -= MXT_BLOCK_SIZE;
		addr += MXT_BLOCK_SIZE;
		dp += MXT_BLOCK_SIZE;
	}
	ret = __mxt_write_reg(data->vdata->client, addr, size, dp);
	return ret;
}

static int mxt_read_object_table(struct i2c_client *client,
				      u16 reg, u8 *object_buf)
{
	return __mxt_read_reg(client, reg, MXT_OBJECT_SIZE,
				   object_buf);
}

static struct mxt_object *mxt_get_object(struct kc_ts_data *data, u8 type)
{
	struct mxt_object *object;
	int i;

	for (i = 0; i < data->vdata->info.object_num; i++) {
		object = data->vdata->object_table + i;
		if (object->type == type)
			return object;
	}

	dev_err(data->dev, "Invalid object type\n");
	return NULL;
}

static int mxt_read_message(struct kc_ts_data *data,
				 struct mxt_message *message)
{
	struct mxt_object *object;
	u16 reg;

	object = mxt_get_object(data, MXT_GEN_MESSAGE_T5);
	if (!object)
		return -EINVAL;

	reg = object->start_address;
	return __mxt_read_reg(data->vdata->client, reg,
			sizeof(struct mxt_message), message);
}

static int mxt_bundle_read_object(struct kc_ts_data *data, u8 type, u8 *val)
{
	struct mxt_object *object;
	u16 reg;
	int num;
	int ret;

	object = mxt_get_object(data, type);
	if (!object)
		return -EINVAL;

	reg = object->start_address;
	num = (object->size + 1) * (object->instances + 1);

	while (num > MXT_BLOCK_SIZE) {
		ret = __mxt_read_reg(data->vdata->client, reg, MXT_BLOCK_SIZE, val);
		if (ret)
			return ret;
		num -= MXT_BLOCK_SIZE;
		reg += MXT_BLOCK_SIZE;
		val += MXT_BLOCK_SIZE;
	}
	ret = __mxt_read_reg(data->vdata->client, reg, num, val);

	return ret;
}

static int mxt_bundle_write_object(struct kc_ts_data *data, u8 type, u8 *dp)
{
	struct mxt_object *object;
	u16 addr;
	int size;
	int ret;

	object = mxt_get_object(data, type);
	if (!object)
		return -EINVAL;

	addr = object->start_address;
	size = (object->size + 1) * (object->instances + 1);

	ret = mxt_bundle_write_reg(data, size, addr, dp);

	return ret;
}

static int mxt_read_object(struct kc_ts_data *data,
				u8 type, u8 offset, u8 *val)
{
	struct mxt_object *object;
	u16 reg;

	object = mxt_get_object(data, type);
	if (!object)
		return -EINVAL;

	reg = object->start_address;
	return __mxt_read_reg(data->vdata->client, reg + offset, 1, val);
}

static int mxt_write_object(struct kc_ts_data *data,
				 u8 type, u8 offset, u8 val)
{
	struct mxt_object *object;
	u16 reg;

	object = mxt_get_object(data, type);
	if (!object)
		return -EINVAL;

	reg = object->start_address;
	return mxt_write_reg(data->vdata->client, reg + offset, val);
}

static void mxt_check_rpten(struct kc_ts_data *data,
				struct mxt_object *object, u8 *config)
{

	if (mxt_object_exist_rpten(data, object->type)) {
		object->report_enable = *config & MXT_RPTEN;
		KC_TS_DEV_DBG("%s: t[%d] RPTEN is [%d]\n"
				, __func__, object->type, object->report_enable);
	}
}

static void mxt_doubletap_report(struct kc_ts_data *data, int id)
{
	struct kc_ts_finger *finger = data->finger;

	if(!finger[id].doubletap){
		finger[id].doubletap = true;
		if(finger[id].x > 250 && finger[id].x < 830 &&
		   finger[id].y > 400 && finger[id].y < 1650){
			KC_TS_DEV_DBG("%s: id= %d,	%04d, %04d\n",__func__,id,finger[id].x,finger[id].y);
			data->coordinate.coor_x = finger[id].x;
			data->coordinate.coor_y = finger[id].y;
			data->coordinate.coor_num++;
		}else{
			KC_TS_DEV_DBG("%s: id= %d, Out of range!\n",__func__,id);
		}
	}
	return;
}

static void mxt_input_report(struct kc_ts_data *data)
{
	struct kc_ts_finger *finger = data->finger;
	struct input_dev *input_dev = data->input_dev;
	int finger_num = 0;
	int id;

	if (ts_event_control)
		return;

	for (id = 0; id < KC_TS_MAX_FINGER; id++) {
		if (!finger[id].status)
			continue;

		input_mt_slot(input_dev, id);
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, finger[id].status != MXT_RELEASE);

		if (finger[id].status != MXT_RELEASE) {
			finger_num++;
			if (finger[id].area <= KC_TS_AREA_MAX)
				input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, finger[id].area);
			else
				input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, KC_TS_AREA_MAX);
			input_report_abs(input_dev, ABS_MT_POSITION_X, finger[id].x);
			input_report_abs(input_dev, ABS_MT_POSITION_Y, finger[id].y);
			input_report_abs(input_dev, ABS_MT_PRESSURE, finger[id].pressure);
			KC_TS_DEV_TOUCH("%s: touch_major[%d], pos_x[%d], pos_y[%d], pressure[%d]\n",
						__func__, finger[id].area, finger[id].x, finger[id].y, finger[id].pressure);
		} else {
			finger[id].status = 0;
		}
	}
	input_report_key(input_dev, BTN_TOUCH, finger_num > 0);
	input_sync(input_dev);
}

static void mxt_input_report_clear(struct kc_ts_data *data)
{
	struct kc_ts_finger *finger = data->finger;
	int id;

	for (id = 0; id < KC_TS_MAX_FINGER; id++) {
		if (!finger[id].status)
			continue;
		finger[id].status = MXT_RELEASE;
		KC_TS_DEV_DBG("%s:[%d] released\n", __func__, id);
		mxt_input_report(data);
	}
}

static void mxt_input_touchevent(struct kc_ts_data *data,
				      struct mxt_message *message, int id)
{
	struct kc_ts_finger *finger = data->finger;
	u8 status = message->message[0], b8 = 0;
	int x;
	int y;

	/* Check the touch is present on the screen */
	if (!(status & MXT_DETECT)) {
		if (status & MXT_RELEASE || status & MXT_SUPPRESS) {
			KC_TS_DEV_DBG("%s :[%d] released\n", __func__, id);
			finger[id].status = MXT_RELEASE;
			if (data->ts_status == TS_STATUS_PRE_ACTIVE)
				mxt_doubletap_report(data, id);
			else {
				mxt_input_report(data);
				if(!data->monitoring_flag){
					b8 = data->last_touch_finger;
					b8 = (unsigned char)( ((b8 & 0xAA) >> 1) + (b8 & 0x55) );
					b8 = (unsigned char)( ((b8 & 0xCC) >> 2) + (b8 & 0x33) );
					b8 = (unsigned char)( ((b8 & 0xF0) >> 4) + (b8 & 0x0F) );

					data->last_touch_finger &= ~(1 << id);
					if(b8 != 2){
						KC_TS_DEV_DBG("%s touch release(not pinch)\n",__func__);
						data->release_check = 1;
					}
					if(data->force_calibration == 3)
						data->force_calibration = 4;
				}
			}
#ifdef FEATURE_TOUCH_TEST
			mxt_write_log(data, KC_TS_LOG_REPORT, &id);
#endif
		}
		return;
	}

	/* Check only AMP detection */
	if (!(status & (MXT_PRESS | MXT_MOVE))){
		if (data->ts_status == TS_STATUS_PRE_ACTIVE){
			KC_TS_DEV_TOUCH("%s:[%d] None\n", __func__, id);
			mxt_doubletap_report(data, id);
		} else {
			if(!data->monitoring_flag && data->force_calibration < 2){
				if(message->message[4] < 0x02){
					finger[id].ts_calibrate_cnt++;
					if(finger[id].ts_calibrate_cnt >= ts_rain_check){
						KC_TS_DEV_DBG("%s rain -> cal\n",__func__);
						data->force_calibration = 2;
					}
				}
				KC_TS_DEV_DBG("%s rain. [%d] = %d\n"
						,__func__, id, finger[id].ts_calibrate_cnt);

				finger[id].stop_finger_cnt++;
				if(finger[id].stop_finger_cnt >= ts_stop_touch){
					KC_TS_DEV_DBG("%s Touch is stopped -> cal\n", __func__);
					data->force_calibration = 2;
				}
				KC_TS_DEV_DBG("%s: STOP. [%d] = %d\n"
							,__func__, id, finger[id].stop_finger_cnt);

				finger[id].little_move_cnt++;
				if(finger[id].little_move_cnt == ts_little_move){
					KC_TS_DEV_DBG("%s: little move(stop) -> cal\n", __func__);
					if(data->force_calibration < 2)
						data->force_calibration = 3;
				}
				KC_TS_DEV_DBG("%s: little move(stop)! [%d] = %d\n"
							,__func__, id, finger[id].little_move_cnt);
			}
		}
		return;
	}

	x = (message->message[1] << 4) | ((message->message[3] >> 4) & 0xf);
	y = (message->message[2] << 4) | ((message->message[3] & 0xf));
/*  If the resolution is less than 1024, to enable the following comment statement */
/*
 *	if (data->max_x < 1024)
 *		x = x >> 2;
 *	if (data->max_y < 1024)
 *		y = y >> 2;
 */
	KC_TS_DEV_TOUCH("%s:[%d] %s \n", __func__, id, status & MXT_MOVE ? "moved" : "pressed");

	finger[id].status = status & MXT_MOVE ?
				MXT_MOVE : MXT_PRESS;
	finger[id].x = x;
	finger[id].y = y;
	finger[id].area = message->message[4];
	finger[id].pressure = message->message[5];

	if (data->ts_status == TS_STATUS_PRE_ACTIVE)
		mxt_doubletap_report(data, id);
	else {
		if((data->true_touch < ts_true_touch) && (data->force_calibration == 0))
			data->true_touch++;
		if(!data->monitoring_flag){
			finger[id].ts_calibrate_cnt = 0;
			finger[id].stop_finger_cnt = 0;
			if(finger[id].status == MXT_PRESS){
				KC_TS_DEV_DBG("%s: PRESS\n",__func__);
				finger[id].x_gr = x;
				finger[id].y_gr = y;
				finger[id].little_move_cnt = 0;
			} else {
				if(data->force_calibration < 2){
					if(finger[id].x_gr + 20 > x && finger[id].x_gr - 20 < x &&
					   finger[id].y_gr + 20 > y && finger[id].y_gr - 20 < y){
						KC_TS_DEV_DBG("%s: little Move. [%d] = %d\n"
									,__func__, id, finger[id].little_move_cnt);
						finger[id].little_move_cnt++;
						if(finger[id].little_move_cnt == ts_little_move){
							pr_info("%s: little move -> cal. [%d] = %d\n"
									,__func__, id, finger[id].little_move_cnt);
							data->force_calibration = 3;
						}
					} else {
						KC_TS_DEV_DBG("%s: little move Reset. [%d] = %d\n"
									,__func__, id, finger[id].little_move_cnt);
						finger[id].x_gr = x;
						finger[id].y_gr = y;
						finger[id].little_move_cnt = 0;
					}
				}
			}
			data->last_touch_finger |= (1 << id);
			KC_TS_DEV_DBG("%s: last_touch_finger = %02x\n",__func__,data->last_touch_finger);
			KC_TS_DEV_DBG("%s: true touch = %d\n",__func__, data->true_touch);
		}
		mxt_input_report(data);
	}
#ifdef FEATURE_TOUCH_TEST
	mxt_write_log(data, KC_TS_LOG_REPORT, &id);
#endif
}

static int mxt_check_reset_report(struct kc_ts_data *data)
{
	struct mxt_message message;
	struct mxt_object *object;
	struct device *dev = data->dev;
	int ret = 0;
	u8 t6_reportid;

	object = mxt_get_object(data, MXT_GEN_COMMAND_T6);
	if (!object) {
		pr_err("Failed to get T6 object!\n");
		ret = -EIO;
		goto error;
	}
	t6_reportid = object->max_reportid;

	do {
		ret = mxt_read_message(data, &message);
		if (ret) {
			pr_err("Failed to read message!\n");
			goto error;
		}
		if ((t6_reportid == message.reportid) &&
		    (message.message[0] & MXT_RESET)) {
			KC_TS_DEV_DBG("%s: Recieved T6 Reset Report.\n",__func__);
			return 0;
		} else {
			mxt_dump_message(dev, &message);
			ret = -EIO;
		}
	} while (message.reportid != 0xff);

error:
	pr_err("%s:Failed to get T6 Reset Report!\n", __func__);
	return ret;
};

static int mxt_wait_interrupt(struct kc_ts_data *data)
{
	const struct kc_ts_platform_data *pdata = data->pdata;
	int i;
	int val;

	for (i = 0; i < (MXT_MAX_RESET_TIME / 10); i++) {
		val = gpio_get_value(pdata->irq_gpio);
		if (val == 0) {
			KC_TS_DEV_DBG("%s: %d0 ms wait.\n", __func__, i);
			return 0;
		}
		usleep_range(10000, 10000);
	}

	return -ETIME;
}

static int mxt_reset_and_delay(struct kc_ts_data *data)
{
	const struct kc_ts_platform_data *pdata = data->pdata;
	int i;
	int error = 0;

	for (i = 0; i < MXT_MAX_RST_TRIES; i++) {
		if (pdata->reset_hw)
			error = pdata->reset_hw();
		if (error) {
			pr_err("%s: Failed to reset hardware!\n", __func__);
			goto done;
		}
		error = mxt_wait_interrupt(data);
		if (!error) {
			KC_TS_DEV_DBG("%s: Received reset interrupt.\n", __func__);
			goto done;
		}
		pr_err("%s: Reset Retry %d times.\n", __func__, i + 1);
	}
	pr_err("%s: Interrupt wait time out!\n", __func__);
done:
	return error;
}

static long mxt_switch_config_exec(struct kc_ts_data *data)
{
	struct ts_config_nv *config_nv;
	struct ts_config_nv *config_nv_last;
	struct mxt_object *object;
	long ret = 0;
	int i, j, offset, num;
	int index = 0;

	if (!ts_config_switching) {
		KC_TS_DEV_DBG("%s: Skip. Disabled config switching.\n",
								__func__);
		data->config_status = data->config_status_last;
		return 0;
	}
	if (data->config_status_last == data->config_status) {
		KC_TS_DEV_DBG("%s: Skip. same status.\n", __func__);
		return 0;
	}

	config_nv = &data->config_nv[data->config_status];
	config_nv_last = &data->config_nv[data->config_status_last];

	if (!config_nv->data || !config_nv_last->data) {
		pr_info("%s: No nv data. Skip switch config.\n",
								__func__);
		data->config_status = data->config_status_last;
		return 0;
	}

	for (i = 0; i < data->vdata->info.object_num; i++) {
		object = data->vdata->object_table + i;
		if (!mxt_object_writable(data, object->type))
			continue;
		if (!mxt_object_switchable(data, object->type)) {
			continue;
		}

		num = (object->size + 1) * (object->instances + 1);
		do {
			if (config_nv->data[index] == object->type) {
				KC_TS_DEV_DBG("%s: match. data[%d](%d)\n",
					__func__, index, config_nv->data[index]);
				break;
			} else {
				index += config_nv->data[index + 1] + 2;
				pr_info("%s: mismatch."
					" index is set to %d\n", __func__, index);
			}
		} while (((index + num + 2) <= config_nv->size) &&
			 ((index + num + 2) <= config_nv_last->size));

		if (((index + num + 2) > config_nv->size) ||
		    ((index + num + 2) > config_nv_last->size)) {
			pr_err("%s: Not enough config data!\n", __func__);
			pr_err("%s: Canceled after t[%d]!\n",
						__func__, object->type);
			return 0;
		}

		index += 2;
		for (j = 0; j < num; j++) {
			offset = index + j;
			ret = memcmp(&config_nv->data[offset],
				     &config_nv_last->data[offset], 1);
			if (ret) {
				ret = mxt_write_object(data, object->type, j,
						 config_nv->data[offset]);
				if(ret){
					pr_err("%s: Error Switch Config Exec\n", __func__);
					data->config_status = data->config_status_last;
					return ret;
				}
				KC_TS_DEV_DBG("%s: write_object[%d]=%02X\n",
						__func__, object->type,
						config_nv->data[offset]);
			}
		}
		mxt_check_rpten(data, object, &config_nv->data[index]);
		index += config_nv->data[index - 1];
	}
	KC_TS_DEV_DBG("%s: Switched to config_status [%d]\n", __func__,
							data->config_status);
#ifdef FEATURE_TOUCH_TEST
	mxt_write_log(data, KC_TS_LOG_CONF_STAT, NULL);
#endif
	return ret;
}

static long mxt_switch_config(struct kc_ts_data *data, unsigned long arg)
{
	long ret = 0;
	enum ts_config_type status;

	ret = copy_from_user(&status,
			     (void __user *)arg,
			     sizeof(status));
	if (ret) {
		pr_err("%s: copy_from_user error\n", __func__);
		return -EFAULT;
	}

	data->config_status_last = data->config_status;
	data->config_status = status;

	ret = mxt_switch_config_exec(data);

	return ret;
}

static int mxt_check_reg_exec(struct kc_ts_data *data, u8 *config, size_t length)
{
	struct mxt_object *object;
	size_t size;
	int index = 0;
	int i;
	u8 *o_data;
	int ret = 0;

	for (i = 0; i < data->vdata->info.object_num; i++) {
		object = data->vdata->object_table + i;

		if (!mxt_object_writable(data, object->type))
			continue;

		size = (object->size + 1) * (object->instances + 1);
		if ((index + size) > length) {
			pr_err("%s: Not enough config data!\n", __func__);
			pr_err("%s: Canceled after t[%d]!\n", __func__, object->type);
			return 0;
		}

		if (config[index] == object->type) {
			index += 2;

			o_data = kcalloc(size, sizeof(u8), GFP_KERNEL);
			if (!o_data) {
				pr_err("Failed to allocate memory\n");
				return -ENOMEM;
			}
			mxt_bundle_read_object(data, object->type, o_data);
			if (memcmp(o_data, &config[index], size)) {
				printk("%s: t[%d] is mismatched\n", __func__, object->type);
				ret = mxt_bundle_write_object(data, object->type, &config[index]);
				if (ret) {
					pr_err("%s: Register write error!\n", __func__);
					kfree(o_data);
					o_data = NULL;
					return ret;
				}
				if (object->type == MXT_GEN_POWER_T7)
					memcpy(&data->vdata->t7_data,
					       &config[index], T7_BACKUP_SIZE);
				data->is_set = true;
			} else {
				KC_TS_DEV_DBG("%s: t[%d] is matched\n", __func__, object->type);
			}
			mxt_check_rpten(data, object, &config[index]);
			kfree(o_data);
			o_data = NULL;
		} else {
			i--;
			size = config[index + 1];
			KC_TS_DEV_DBG("%s: skip config data, type[%d], size =[%X](%d)\n",
							__func__, object->type, size, size);
			index += 2;
		}
		index += size;
	}
	return ret;
}

static int mxt_check_reg_nv(struct kc_ts_data *data)
{
	struct ts_config_nv *nv = &data->config_nv[TS_CHARGE_CABLE];
	int ret;

	if (!nv->data) {
		pr_info("%s: No nv data, skipping.\n", __func__);
		return 0;
	}

	KC_TS_DEV_DBG("%s: mxt_check_reg_exec(size = %d).\n",
						__func__, nv->size);
	ret = mxt_check_reg_exec(data, nv->data, nv->size);

	return ret;
}

static int mxt_check_reg_config(struct kc_ts_data *data)
{
	const struct kc_ts_platform_data *pdata = data->pdata;
	int ret;

	if (!pdata->config) {
		KC_TS_DEV_DBG("%s:No cfg data defined, skipping.\n", __func__);
		return 0;
	}
	ret = mxt_check_reg_exec(data, (u8 *)pdata->config,
						pdata->config_length);

	return ret;
}

static int mxt_backup_nv(struct kc_ts_data *data)
{
	int error;
	u8 command_register;
	int timeout_counter = 0;

	/* Backup to memory */
	mxt_write_object(data, MXT_GEN_COMMAND_T6,
					MXT_COMMAND_BACKUPNV, MXT_BACKUP_VALUE);
	msleep(MXT_BACKUP_TIME);

	do {
		error = mxt_read_object(data, MXT_GEN_COMMAND_T6,
					MXT_COMMAND_BACKUPNV, &command_register);
		if (error)
			return error;

		usleep_range(1000, 2000);

	} while ((command_register != 0) && (++timeout_counter <= 100));

	if (timeout_counter > 100) {
		pr_err("%s: No response after backup!\n", __func__);
		return -EIO;
	}
	pr_info("%s() is completed.\n", __func__);
	return 0;
}

static int mxt_reset_status(struct kc_ts_data *data)
{
	int ret = 0;

	KC_TS_DEV_DBG("%s() is called.\n", __func__);

	/* Send release event as needed */
	mxt_input_report_clear(data);

	/* Check registers after reset */
	if (data->config_nv[TS_CHARGE_CABLE].data)
		ret = mxt_check_reg_nv(data);
	else
		ret = mxt_check_reg_config(data);
	if (!ret) {
		if (data->is_set) {
			pr_info("%s: NVM set.\n", __func__);
			ret = mxt_backup_nv(data);
			if (ret)
				pr_err("%s: Fail to backup!\n", __func__);
			data->is_set = false;
		}
	} else {
		pr_err("%s: Fail to check registers after reset!\n",__func__);
		return ret;
	}

	/* Check and set lateset config */
	data->config_status_last = TS_CHARGE_CABLE;
	ret = (int)mxt_switch_config_exec(data);
	ret |= mxt_calibrate(data);

	if(ret)
		pr_err("%s: switch or cal is error\n",__func__);

	KC_TS_DEV_DBG("%s() is completed.\n", __func__);
#ifdef FEATURE_TOUCH_TEST
	mxt_write_log(data, KC_TS_LOG_RESET_STATUS, NULL);
#endif
	return ret;
}

static int mxt_restart(struct kc_ts_data *data)
{
	int ret = 0;

	pr_err("%s is called.\n", __func__);
	/* Reset, then wait interrupt */
	ret = mxt_reset_and_delay(data);
	if (ret)
		return ret;

	/* Check T6 reset report  */
	ret = mxt_check_reset_report(data);
	if (ret)
		return ret;

	KC_TS_DEV_DBG("%s() is completed.\n", __func__);
#ifdef FEATURE_TOUCH_TEST
	mxt_write_log(data, KC_TS_LOG_RESTART, NULL);
#endif

	return ret;
}

static int mxt_start(struct kc_ts_data *data)
{
	int error;

	KC_TS_DEV_DBG("%s is called\n",__func__);
	/* restore the old power state values and reenable touch */
	error = __mxt_write_reg(data->vdata->client, data->vdata->t7_start_addr,
				T7_BACKUP_SIZE, data->vdata->t7_data);
	if (error < 0) {
		dev_err(data->dev,
			"failed to restore old power state\n");
		return error;
	}

	return error;
}

static int mxt_stop(struct kc_ts_data *data)
{
	int error;
	u8 t7_data[T7_BACKUP_SIZE] = {0};

	KC_TS_DEV_DBG("%s is called\n",__func__);
	error = __mxt_write_reg(data->vdata->client, data->vdata->t7_start_addr,
				T7_BACKUP_SIZE, t7_data);
	if (error < 0) {
		dev_err(data->dev,
			"failed to configure deep sleep mode\n");
		return error;
	}

	return error;
}

static int mxt_error_check_process(struct kc_ts_data *data, struct mxt_message *message)
{

	u8 reportid;
	u8 max_reportid;
	u8 min_reportid;
	struct mxt_object *object;
	int error = 0;
	u8 status = message->message[0];
	int i;

	reportid = message->reportid;
	/* Check if received T6 error message */
	object = mxt_get_object(data, MXT_GEN_COMMAND_T6);
	if (!object) {
		pr_err("%s: Failed to get T6 object!\n", __func__);
		error = -EIO;
		goto done;
	}
	max_reportid = object->max_reportid;
	min_reportid = max_reportid - object->num_report_ids + 1;
	if ((reportid >= min_reportid) && (reportid <= max_reportid)) {
		if (status & MXT_CFGERR) {
			pr_err("%s:Received GEN_COMMANDPROCESSOR_T6"
						"[%02X]!\n", __func__, status);
#ifdef FEATURE_TOUCH_TEST
			mxt_write_log(data, KC_TS_LOG_CFGERR, NULL);
#endif
			if (data->vdata->err_cnt[MXT_ERR_CFGERR] <= MXT_MAX_ERR_CNT)
				data->vdata->err_cnt[MXT_ERR_CFGERR]++;
			goto restart;
		}
		if (status & MXT_RESET) {
			pr_err("%s:Received GEN_COMMANDPROCESSOR_T6"
						"[%02X]!\n", __func__, status);
#ifdef FEATURE_TOUCH_TEST
			mxt_write_log(data, KC_TS_LOG_IC_RESET, NULL);
#endif
			if (data->vdata->err_cnt[MXT_ERR_RESET] <= MXT_MAX_ERR_CNT)
				data->vdata->err_cnt[MXT_ERR_RESET]++;
			goto reset;
		}
		goto done;
	}

	/* Check if received invalid messages */
	for (i = 0; i < data->vdata->info.object_num; i++) {
		object = data->vdata->object_table + i;

		if (!mxt_object_exist_rpten(data, object->type))
			continue;

		max_reportid = object->max_reportid;
		min_reportid = max_reportid - object->num_report_ids + 1;
		if ((reportid >= min_reportid) && (reportid <= max_reportid) &&
			(!object->report_enable)) {
			pr_err("%s:Received T[%d] message even though RPTEN is not set!\n"
						, __func__, object->type);
#ifdef FEATURE_TOUCH_TEST
			mxt_write_log(data, KC_TS_LOG_RPTEN, &object->type);
#endif
			if (data->vdata->err_cnt[MXT_ERR_MISBEHAVING] <= MXT_MAX_ERR_CNT)
				data->vdata->err_cnt[MXT_ERR_MISBEHAVING]++;
			mxt_recovery_cnt++;
			goto restart;
		} else if ((reportid >= min_reportid) && (reportid <= max_reportid) &&
					(object->report_enable)) {
				KC_TS_DEV_DBG("%s:Received T[%d] message.\n"
							, __func__, object->type);
			goto done;
		}
	}
	pr_info("%s: Received unexpected report id[%02X]\n",
					__func__, message->reportid);
	goto done;
restart:
	/* Maybe abnormal state, then restart */
	if(mxt_recovery_cnt < 5)
		error = mxt_restart(data);
	if (error)
		pr_err("%s: Failed to restart!\n", __func__);
reset:
	/* Reset status in Touch Screen Driver */
	if(mxt_recovery_cnt < 5)
		error = mxt_reset_status(data);
	if (error)
		pr_err("%s: Failed to reset status!\n", __func__);
done:
	mxt_dump_message(data->dev, message);
	return error;
}

static int mxt_interrupt(struct kc_ts_data *data)
{
	struct mxt_message message;
	struct mxt_object *object;
	int id;
	u8 reportid, anti_id = 0;
	u8 max_reportid;
	u8 min_reportid;
	int err = 0;

	KC_TS_DEV_TOUCH("%s is called.\n", __func__);
#ifdef FEATURE_TOUCH_TEST
	mxt_write_log(data, KC_TS_LOG_INTERRUPT_START, NULL);
#endif

	switch(data->ts_status){
	case TS_STATUS_ACTIVE:
	case TS_STATUS_PRE_ACTIVE:
		if(!data->monitoring_flag){
			object = mxt_get_object(data, MXT_PROCI_EXTRA_T57);
			if (!object) {
				pr_err("%s: Failed to get object.\n", __func__);
				err = -EINVAL;
				goto end;
			}
			anti_id = object->max_reportid;
		}
		do {
			err = mxt_read_message(data, &message);
			if (err) {
				pr_err("%s: Failed to read message\n", __func__);
				goto end;
			}
#ifdef FEATURE_TOUCH_TEST
			mxt_write_log(data, KC_TS_LOG_READ_MESSAGE, NULL);
#endif

			reportid = message.reportid;

			/* whether reportid is thing of MXT_TOUCH_MULTI_T9 */
			object = mxt_get_object(data, MXT_TOUCH_MULTI_T9);
			if (!object) {
				pr_err("%s: Failed to get object.\n", __func__);
				err = -EINVAL;
				goto end;
			}
			max_reportid = object->max_reportid;
			min_reportid = max_reportid - object->num_report_ids + 1;
			id = reportid - min_reportid;

			if ((reportid >= min_reportid) &&
			    (reportid <= max_reportid) &&
			    (object->report_enable)){
				mxt_input_touchevent(data, &message, id);
			} else if (reportid == anti_id){
				if(!data->monitoring_flag){
					if((message.message[4]) || (message.message[5])){
						KC_TS_DEV_DBG("%s: Anti = %02x%02x, calib = %d, finger = %02x\n",
								__func__,message.message[4],message.message[5],
								data->force_calibration, data->last_touch_finger);
						if(data->force_calibration == 0)
							data->force_calibration = 1;
					}
				}
			} else if (reportid != 0xff)
				mxt_error_check_process(data, &message);
#ifdef FEATURE_TOUCH_TEST
			mxt_write_log(data, KC_TS_LOG_MSG, &message);
#endif
		} while (reportid != 0xff);
		break;
	case TS_STATUS_SLEEP:
		/* Maybe reset occur.
		 * Read dummy message to make high CHG pin,
		 * then configure deep sleep mode.
		 */
		pr_err("%s: Device should be suspended."
				"Configure deep sleep mode.\n", __func__);
		do {
			err = mxt_read_message(data, &message);
			if (err) {
				pr_err("%s: Failed to read message\n", __func__);
				goto end;
			}
			reportid = message.reportid;
			mxt_error_check_process(data, &message);
#ifdef FEATURE_TOUCH_TEST
			mxt_write_log(data, KC_TS_LOG_MSG, &message);
#endif
		} while (reportid != 0xff);
		err = mxt_stop(data);
		if (err)
			pr_err("%s: mxt_stop failed.\n", __func__);
		break;
	case TS_STATUS_DEEP_SLEEP:
		pr_err("%s: This route interrupt that should not occur.", __func__);
		break;
	default:
		pr_err("%s: [Error] default case is called.\n", __func__);
		break;
	}

end:
	KC_TS_DEV_TOUCH("%s is completed.\n",__func__);
#ifdef FEATURE_TOUCH_TEST
	mxt_write_log(data, KC_TS_LOG_INTERRUPT_END, NULL);
#endif
	return err;
}

static int mxt_get_info(struct kc_ts_data *data)
{
	struct i2c_client *client = data->vdata->client;
	struct mxt_info *info = &data->vdata->info;
	int error;
	u8 val;

	error = mxt_read_reg(client, MXT_FAMILY_ID, &val);
	if (error)
		return error;
	info->family_id = val;

	error = mxt_read_reg(client, MXT_VARIANT_ID, &val);
	if (error)
		return error;
	info->variant_id = val;

	error = mxt_read_reg(client, MXT_VERSION, &val);
	if (error)
		return error;
	info->version = val;

	error = mxt_read_reg(client, MXT_BUILD, &val);
	if (error)
		return error;
	info->build = val;

	/* Update matrix size at info struct */
	error = mxt_read_reg(client, MXT_MATRIX_X_SIZE, &val);
	if (error)
		return error;
	info->matrix_xsize = val;

	error = mxt_read_reg(client, MXT_MATRIX_Y_SIZE, &val);
	if (error)
		return error;
	info->matrix_ysize = val;

	error = mxt_read_reg(client, MXT_OBJECT_NUM, &val);
	if (error)
		return error;
	info->object_num = val;

	return 0;
}

static int mxt_get_object_table(struct kc_ts_data *data)
{
	int error;
	int i;
	u16 reg;
	u8 reportid = 0;
	u8 buf[MXT_OBJECT_SIZE];
	u16 size;

	for (i = 0; i < data->vdata->info.object_num; i++) {
		struct mxt_object *object = data->vdata->object_table + i;

		reg = MXT_OBJECT_START + MXT_OBJECT_SIZE * i;
		error = mxt_read_object_table(data->vdata->client, reg, buf);
		if (error)
			return error;

		object->type = buf[0];
		object->start_address = (buf[2] << 8) | buf[1];
		object->size = buf[3];
		object->instances = buf[4];
		object->num_report_ids = buf[5];

		if (object->num_report_ids) {
			reportid += object->num_report_ids *
					(object->instances + 1);
			object->max_reportid = reportid;
		}
		size = (object->size + 1) * (object->instances + 1);
		if (data->vdata->max_o_size < size)
			data->vdata->max_o_size = size;
	}

	return 0;
}

#ifdef FEATURE_TOUCH_TEST
static int mxt_log_object(struct kc_ts_data *data, struct file *fp)
{
	int error = 0;
	struct mxt_object *object;
	int i, j;
	u8 val;
	char buf[700];
	u16 size;
	int len;

	for (i = 0; i < data->vdata->info.object_num; i++) {
		memset(buf, '\0', sizeof(buf));
		object = data->vdata->object_table + i;

		len = sprintf(buf, "\n,Object table Element[%d] (Type : %d)",
						i + 1, object->type);
		fp->f_op->write(fp, buf, len, &(fp->f_pos));
		memset(buf, '\0', sizeof(buf));
		if (!mxt_object_readable(data, object->type)) {
			len = sprintf(buf, "\n");
			fp->f_op->write(fp, buf, len, &(fp->f_pos));
			memset(buf, '\0', sizeof(buf));
			continue;
		}

		size = (object->size + 1) * (object->instances + 1);
		for (j = 0; j < size; j++) {
			if (!(j % 10)) {
				len = sprintf(buf, "\n,");
				fp->f_op->write(fp, buf, len, &(fp->f_pos));
				memset(buf, '\0', sizeof(buf));
			}
			error = mxt_read_object(data,
						object->type, j, &val);
			if (error) {
				KC_TS_DEV_DBG("%s: t%d[%02d] read error!\n", __func__, object->type, j);
				goto done;
			}
			len = sprintf(buf, "0x%02x ", val);
			fp->f_op->write(fp, buf, len, &(fp->f_pos));
			memset(buf, '\0', sizeof(buf));
		}
	}
done:
	len = sprintf(buf, "\n\n");
	fp->f_op->write(fp, buf, len, &(fp->f_pos));
	return error;
};
#endif

static int mxt_initialize(struct kc_ts_data *data)
{
	struct i2c_client *client = data->vdata->client;
	struct mxt_info *info = &data->vdata->info;
	int error;
	struct mxt_object *t7_object;

	error = mxt_get_info(data);
	if (error)
		return error;

	data->vdata->state = APPMODE;

	data->vdata->object_table = kcalloc(info->object_num,
				     sizeof(struct mxt_object),
				     GFP_KERNEL);
	if (!data->vdata->object_table) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	/* Get object table information */
	error = mxt_get_object_table(data);
	if (error)
		goto free_object_table;

	/* Check T6 reset report */
	error = mxt_check_reset_report(data);
	if (error)
		goto free_object_table;

	/* Check register init values */
	data->is_set = false;
	error = mxt_check_reg_config(data);
	if (error)
		goto free_object_table;

	if (data->is_set){
		pr_info("%s: NVM set.\n", __func__);
		error = mxt_backup_nv(data);
		if (error)
			pr_err("%s: Fail to backup!\n", __func__);
		data->is_set = false;
	}

	data->config_status = TS_CHARGE_CABLE;

	/* Store T7 and T9 locally, used in suspend/resume operations */
	t7_object = mxt_get_object(data, MXT_GEN_POWER_T7);
	if (!t7_object) {
		pr_err("%s: Failed to get T7 object\n", __func__);
		error = -EINVAL;
		goto free_object_table;
	}

	data->vdata->t7_start_addr = t7_object->start_address;
	error = __mxt_read_reg(client, data->vdata->t7_start_addr,
				T7_BACKUP_SIZE, data->vdata->t7_data);
	if (error < 0) {
		pr_err("%s: Failed to save current power state\n", __func__);
		goto free_object_table;
	}

	KC_TS_DEV_DBG("%s: Family ID: 0x%X Variant ID: 0x%X Version: 0x%X Build: 0x%X\n",
					__func__, info->family_id, info->variant_id,
					info->version, info->build);

	KC_TS_DEV_DBG("%s: Matrix X Size: %d Matrix Y Size: %d Object Num: %d\n", __func__,
					info->matrix_xsize, info->matrix_ysize,	info->object_num);

	return 0;

free_object_table:
	kfree(data->vdata->object_table);
	return error;
}

static void mxt_calc_resolution(struct kc_ts_data *data)
{
	unsigned int max_x = data->pdata->x_size - 1;
	unsigned int max_y = data->pdata->y_size - 1;

	if (data->pdata->orient & MXT_XY_SWITCH) {
		data->max_x = max_y;
		data->max_y = max_x;
	} else {
		data->max_x = max_x;
		data->max_y = max_y;
	}
}

static int mxt_bundle_read_reg(struct kc_ts_data *data, u16 size, u16 addr, u8 *dp)
{
	int ret;
	while (size > MXT_BLOCK_SIZE) {
		ret = __mxt_read_reg(data->vdata->client, addr, MXT_BLOCK_SIZE, dp);
		if (ret)
			return ret;
		size -= MXT_BLOCK_SIZE;
		addr += MXT_BLOCK_SIZE;
		dp += MXT_BLOCK_SIZE;
	}
	ret = __mxt_read_reg(data->vdata->client, addr, size, dp);
	return ret;
}

static int mxt_enable(struct kc_ts_data *data)
{
	KC_TS_DEV_DBG("%s is called.\n", __func__);
	if (data->is_enable)
		goto done;

	data->is_enable = true;
	enable_irq(data->vdata->client->irq);
done:
	KC_TS_DEV_DBG("%s is completed.\n", __func__);
	return 0;
}

static void mxt_disable(struct kc_ts_data *data)
{
	KC_TS_DEV_DBG("%s is called.\n", __func__);
	if (!data->is_enable)
		goto done;

	disable_irq_nosync(data->vdata->client->irq);
	data->is_enable = false;
done:
	KC_TS_DEV_DBG("%s is completed.\n", __func__);
}

static int mxt_wait_message_of_golden(struct kc_ts_data *data, char *t66_fcal,
									struct mxt_object *object, u8 t66_message)
{
	struct mxt_message message;
	u16 i;

	for(i=0; i<510; i++){
		msleep(100);
		do {
			mxt_read_message(data, &message);
			if( object->max_reportid == message.reportid){
				if(t66_message & MXT_MESSAGE_GENERATE){
					t66_fcal[0] = message.message[0];
					t66_fcal[1] = message.message[1];
					t66_fcal[2] = message.message[2];
					t66_fcal[3] = message.message[3];
				}
				KC_TS_DEV_DBG("%s: T66 message is %02x %02x %02x %02x\n",
							__func__,message.message[0],message.message[1],
									 message.message[2],message.message[3]);
				if( t66_message == (message.message[0] & t66_message)){
					KC_TS_DEV_DBG("%s: T66 message OK\n",__func__);
					return 0;
				} else if( (MXT_FCALSEQTO | MXT_FCALSEQERR | MXT_FCALFAIL)
							& message.message[0] ){
					pr_err("%s: T66 message Error! Status=%02x\n"
							,__func__, message.message[0]);
					return -1;
				} else {
					pr_err("%s: T66 unexpected message! Status=%02x\n"
							,__func__, message.message[0]);
					return -1;
				}
			}
		} while (message.reportid != 0xff);
		KC_TS_DEV_DBG("%s: T66 Wait Message %d\n", __func__, i);
	}
	return -1;
}

static struct mxt_object *mxt_data_write_to_object(struct kc_ts_data *data,
									u8 obj_type, u8 obj_field, u8 w_status, u8 cmd_mask)
{
	struct mxt_object *object;
	int err = 0;
	u8 *read_obj = NULL;
	u8 w_data;

	object = mxt_get_object(data, obj_type);
	if (!object){
		pr_err("%s: T%d_Get_Object_ERROR\n", __func__, obj_type);
		return NULL;
	}

	read_obj = kcalloc(object->size, sizeof(u8), GFP_KERNEL);
	if (!read_obj) {
		pr_err("%s: Failed to allocate memory\n", __func__);
		return NULL;
	}

	err = mxt_bundle_read_object(data, object->type, read_obj);
	if(err){
		pr_err("%s: T%d_Read_Object_ERROR\n", __func__, obj_type);
		kfree(read_obj);
		return NULL;
	}

	read_obj[obj_field] &= ~cmd_mask;
	w_data = w_status | read_obj[obj_field];
	err = mxt_write_object(data, obj_type, obj_field , w_data );

	if(err){
		pr_err("%s: Write Object ERROR!\n", __func__);
		object = NULL;
	}

	KC_TS_DEV_DBG("%s: T%d w_data=%02x, read_obj=%02x\n", __func__,
				 obj_type, w_data, read_obj[obj_field]);
	kfree(read_obj);

	return object;
}

static int mxt_get_golden_reference(struct kc_ts_data *data, char *t66_fcal)
{
	struct mxt_object *object;
	int err = 0;

	KC_TS_DEV_DBG("%s: Start\n", __func__);

	err |= mxt_write_object(data, MXT_SPT_GOLDENREFERENCES_T66,
							 MXT_SPT_FCALFAILTHR, t66_fcal[0] );
	err |= mxt_write_object(data, MXT_SPT_GOLDENREFERENCES_T66,
							 MXT_SPT_FCALDRIFTCNT, t66_fcal[1] );

	if(err){
		pr_err("%s: T66_write_ERROR\n", __func__);
		return err;
	}

	err |= mxt_write_object(data, MXT_SPT_CTECONFIG_T46,
							 MXT_SPT_IDLESYNCSPERX , 0xFF );
	err |= mxt_write_object(data, MXT_SPT_CTECONFIG_T46,
							 MXT_ACTVSYNCSPERX , 0xFF );

	if(err){
		pr_err("%s: T46_write_ERROR\n", __func__);
		return err;
	}

	err = mxt_write_object( data, MXT_NOISESUPPRESIION_T72,
							MXT_MINNLTHR, 0xFE );
	if(err){
		pr_err("%s: T72_write_ERROR\n", __func__);
		return err;
	}

	object = mxt_data_write_to_object(data, MXT_SPT_GOLDENREFERENCES_T66,
									MXT_SPT_CTRL, MXT_CMD_PRIME, MXT_FCALCMD_STATE_MASK);
	if(!object){
		pr_err("%s: T66_write_ERROR PRIME\n", __func__);
		return -ENOMEM;
	}
	err = mxt_wait_message_of_golden(data, t66_fcal, object, MXT_MESSAGE_PRIME);
	if(err){
		pr_err("%s: T66_message_TimeOut PRIME\n", __func__);
		return err;
	}

	object = mxt_data_write_to_object(data, MXT_SPT_GOLDENREFERENCES_T66,
									MXT_SPT_CTRL, MXT_CMD_GENERATE, MXT_FCALCMD_STATE_MASK);
	if(!object){
		pr_err("%s: T66_write_ERROR GENERATE\n", __func__);
		return -ENOMEM;
	}
	err = mxt_wait_message_of_golden(data, t66_fcal, object, (MXT_MESSAGE_GENERATE | MXT_FCALPASS));
	if(err){
		pr_err("%s: T66_message_TimeOut GENERATE\n", __func__);
		return err;
	}

	object = mxt_data_write_to_object(data, MXT_SPT_GOLDENREFERENCES_T66,
									MXT_SPT_CTRL, MXT_CMD_CONFIRM, MXT_FCALCMD_STATE_MASK);
	if(!object){
		pr_err("%s: T66_write_ERROR CONFIRM\n", __func__);
		return -ENOMEM;
	}
	err = mxt_wait_message_of_golden(data, t66_fcal, object, MXT_FCALSEQDONE);
	if(err){
		pr_err("%s: T66_message_TimeOut CONFIRM\n", __func__);
		return err;
	}

	err = mxt_write_object( data, MXT_NOISESUPPRESIION_T72,
							MXT_MINNLTHR, 0x14 );
	if(err){
		pr_err("%s: T72_write_ERROR\n", __func__);
		return err;
	}

	err = mxt_backup_nv(data);
	if(err){
		pr_err("%s: Failed mxt_backup_nv\n", __func__);
		return err;
	}

	return err;
}

static int mxt_get_t37_data(struct kc_ts_data *data, char cmd)
{
	struct mxt_object *object;
	int i, j;
	int err = 0;
	u8 t6_diagnostics;

	KC_TS_DEV_DBG("%s: start\n", __func__);
	if (!t37_data)
		t37_data = kcalloc(MXT_DIAG_DATA_SIZE * MXT_DIAG_NUM_PAGE,
							sizeof(u8), GFP_KERNEL);
	else
		KC_TS_DEV_DBG("%s: t37 has been allocated.\n",__func__);

	if (!t37_data)  {
		pr_err("%s Failed to allocate memory!\n", __func__);
		return -ENOMEM;
	}
	KC_TS_DEV_DBG("%s: MXT_COMMAND_DIAGNOSTIC set to [0x%02X]\n"
					,__func__, cmd);
	mxt_write_object(data, MXT_GEN_COMMAND_T6, MXT_COMMAND_DIAGNOSTIC,
						(u8)cmd);

	object = mxt_get_object(data, MXT_DEBUG_DIAGNOSTIC_T37);
	if (!object) {
		pr_err("Failed object error!\n");
		kfree(t37_data);
		t37_data = NULL;
		return -EINVAL;
	}

	for (i = 0; i < MXT_DIAG_NUM_PAGE; i++) {
		KC_TS_DEV_DBG("%s: T37 page [%d] read.\n", __func__, i);
		msleep(50);
		for (j = 0; j < 51; j++) {
			mxt_read_object(data, MXT_GEN_COMMAND_T6,
					MXT_COMMAND_DIAGNOSTIC, &t6_diagnostics);
			if (t6_diagnostics == 0)
				break;
			if (j == 50){
				pr_err("%s: T6 DIAGNOSTICS Error.\n",__func__);
				return -1;
			}
			msleep(10);
			KC_TS_DEV_DBG("%s: T6 Read [%d].\n", __func__, j);
		}

		mxt_bundle_read_reg(data,
				    MXT_DIAG_DATA_SIZE,
				    object->start_address,
				    &t37_data[i * MXT_DIAG_DATA_SIZE]);
		if (!(t37_data[i * MXT_DIAG_DATA_SIZE + 1] == i)) {
			pr_err("%s: T37 page [%d] read Error! error=%d\n",
						__func__, i, err);
			i--;
			err++;
		} else
			err = 0;

		if (err > 5) {
			pr_err("%s: Fail to read T37!\n", __func__);
			kfree(t37_data);
			t37_data = NULL;
			mxt_write_object(data, MXT_GEN_COMMAND_T6,
							 MXT_COMMAND_DIAGNOSTIC, 0x00);
			return -EIO;
		}
		if ((err == 0) && (i < 5)){
			msleep(10);
			mxt_write_object(data, MXT_GEN_COMMAND_T6,
					 MXT_COMMAND_DIAGNOSTIC, 0x01);
		}
	}
	mxt_write_object(data, MXT_GEN_COMMAND_T6,
				 MXT_COMMAND_DIAGNOSTIC, 0x00);

	return err;
}

static int mxt_pass_t37_data(u8 *reference)
{
	int err = 0;

	if(!t37_data){
		pr_err("%s: t37_data is NULL\n", __func__);
		err = 1;
		goto end;
	}
	memcpy(reference, t37_data, MXT_DIAG_DATA_SIZE * MXT_DIAG_NUM_PAGE);
	kfree(t37_data);
	t37_data = NULL;

end:
	return err;
}

static long mxt_ts_ioctl(struct kc_ts_data *data, unsigned int cmd,
						unsigned long arg)
{
	long err = 0;
	char *t66_fcal = NULL;

	KC_TS_DEV_DBG("%s() is called.\n", __func__);
	switch (cmd) {
	case IOCTL_SET_CONF_STAT:
		KC_TS_DEV_DBG("%s: IOCTL_SET_CONF_STAT\n", __func__);
		err = mxt_switch_config(data, arg);
		break;
	case IOCTL_GET_GOLDEN_REFERENCE:
		KC_TS_DEV_DBG("%s: IOCTL_GET_GOLDEN_REFERENCE\n", __func__);
		if (data->vdata->info.version != MXT_FW_V3_0) {
			pr_err("%s: This FW is not Ver 3.0\n", __func__);
			err = -1;
			goto done;
		}
		t66_fcal = kcalloc(T66_RSP_SIZE, sizeof(char), GFP_KERNEL);
		if (!t66_fcal) {
			pr_err("%s: Failed to allocate memory\n", __func__);
			return -ENOMEM;
		}
		err = copy_from_user(t66_fcal, (void __user *)arg, T66_CMD_SIZE);
		if (err){
			pr_err("%s: copy_from_user error\n", __func__);
			kfree(t66_fcal);
			return -EFAULT;
		}

		err |= mxt_write_object( data, MXT_GEN_POWER_T7,
								 MXT_POWER_IDLEACQINT, MXT_FREE_RUN_MODE);
		err |= mxt_write_object( data, MXT_GEN_POWER_T7,
								 MXT_POWER_ACTVACQINT, MXT_FREE_RUN_MODE);
		if(err){
			pr_err("%s: Failed Write T7 Object\n", __func__);
			goto done;
		}

		err = mxt_get_golden_reference(data, t66_fcal);
		if(err)
			pr_err("%s: mxt_get_golden_reference ERROR!\n", __func__);

		if (copy_to_user((void __user *)arg, t66_fcal, T66_RSP_SIZE)) {
			err = -EFAULT;
			pr_err("%s: copy_to_user error\n", __func__);
		}

		kfree(t66_fcal);
		break;
	case IOCTL_DIAG_GET_C_REFERENCE:
	case IOCTL_DIAG_GET_DELTA:
		KC_TS_DEV_DBG("%s: IOCTL_DIAG_T37_DATA\n", __func__);
		err = mxt_start(data);
		if(err){
			pr_err("%s: Failed mxt_start\n", __func__);
			goto done;
		}

		if(cmd == IOCTL_DIAG_GET_C_REFERENCE)
			err = mxt_get_t37_data(data, MXT_CURRENT_REFERENCE);
		else if(cmd == IOCTL_DIAG_GET_DELTA)
			err = mxt_get_t37_data(data, MXT_DELTA);
		if(data->ts_status <= TS_STATUS_SLEEP) {
			if(mxt_stop(data))
				pr_err("%s: Failed mxt_stop", __func__);
		}
		if(err){
			pr_err("%s: Failed get t37_data\n", __func__);
			goto done;
		}

		err = copy_to_user((void __user *)arg, t37_data,
							 MXT_DIAG_DATA_SIZE*MXT_DIAG_NUM_PAGE);
		kfree(t37_data);
		t37_data = NULL;

		if (err) {
			pr_err("%s: copy_to_user error\n", __func__);
			goto done;
		}
		break;
	default:
		pr_err("%s: cmd error[%X]\n", __func__, cmd);
		return -EINVAL;
		break;
	}
done:
	return err;
}

static int mxt_input_open(struct kc_ts_data *data)
{
	int err;

	KC_TS_DEV_DBG("%s is called.\n", __func__);

	err = mxt_enable(data);
	if (err)
		dev_err(data->dev, "mxt_enable failed in open\n");

	KC_TS_DEV_DBG("%s is completed.\n", __func__);
	return err;
}

static void mxt_input_close(struct kc_ts_data *data)
{

	KC_TS_DEV_DBG("%s is called.\n", __func__);
	mutex_lock(&data->lock);
	mxt_disable(data);
	mutex_unlock(&data->lock);
	KC_TS_DEV_DBG("%s is completed.\n", __func__);
}

#ifdef CONFIG_PM
static int mxt_suspend(struct kc_ts_data *data)
{
	int err = 0;
	int rc = 0;

	KC_TS_DEV_DBG("%s start\n", __func__);

	mxt_disable(data);

	err = mxt_stop(data);
	if (err)
		pr_err("mxt_stop failed in suspend\n");

	msleep(30);

	if(reg_touch)
		rc = regulator_set_optimum_mode(reg_touch, 100);
	if(rc < 0)
		pr_err("%s regulator set error\n",__func__);
	else
		KC_TS_DEV_DBG("%s regulator set 0x%02x\n",__func__,rc);

	KC_TS_DEV_DBG("%s done.\n", __func__);
#ifdef FEATURE_TOUCH_TEST
	mxt_write_log(data, KC_TS_LOG_SUSPEND, NULL);
#endif
	return err;
}

static int mxt_calibrate(struct kc_ts_data *data)
{
	int ret = 0;

	KC_TS_DEV_DBG("%s: Calibration\n", __func__);
	ret |= mxt_write_object(data, MXT_PROCI_EXTRA_T57,
				MXT_EXTRA_CTRL, 0x63);
	ret |= mxt_write_object(data, MXT_PROCI_TOUCH_T42,
				MXT_SUPPRESSION_CTRL, 0x00);

	ret |= mxt_write_reg(data->vdata->client, 0x02E3, 0x00);
	ret |= mxt_write_reg(data->vdata->client, 0x019D, 0x12);
	ret |= mxt_write_reg(data->vdata->client, 0x019E, 0x02);

	ret |= mxt_write_object(data, MXT_GEN_COMMAND_T6,
				MXT_COMMAND_CALIBRATE, MXT_CALIBRATE_ORDER);

	if (ret)
		pr_err("%s: Calibration error\n", __func__);

	data->true_touch = 0;
	data->force_calibration = 0;
	data->monitoring_flag = 0;
	data->release_check = 0;
	return ret;
}

static int mxt_resume(struct kc_ts_data *data)
{
	int err = 0;
	int rc = 0;

	KC_TS_DEV_DBG("%s start\n", __func__);

	if(reg_touch)
		rc = regulator_set_optimum_mode(reg_touch, 30000);
	if(rc < 0)
		pr_err("%s regulator set error\n",__func__);
	else
		KC_TS_DEV_DBG("%s regulator set 0x%02x\n",__func__,rc);

	usleep_range(1000, 1000);

	err = mxt_start(data);
	if (err) {
		pr_err("mxt_start failed in resume\n");
		goto done;
	}

	err = mxt_enable(data);
	if (err < 0)
		pr_err("%s: Failed enable\n", __func__);
done:
	KC_TS_DEV_DBG("%s done.\n", __func__);
	return err;
}
#endif

static int mxt_deep_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kc_ts_data *data = i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;
	int err = 0;

	KC_TS_DEV_DBG("%s: start\n", __func__);
	mutex_lock(&data->lock);
	if (!input_dev->users) {
		pr_err("%s input_dev->users disable\n", __func__);
		goto done;
	}

	if(data->ts_double_tap_mode){
		KC_TS_DEV_DBG("%s: Double Tap Mode ON\n", __func__);
		if(data->ts_status == TS_STATUS_PRE_ACTIVE){
			err = mxt_suspend(data);
			if (err) {
				pr_err("failed in suspend process.\n");
				goto done;
			}
			data->ts_status = TS_STATUS_DEEP_SLEEP;
		} else
			pr_err("%s Mode ON: abnormal Status [%d]\n"
					, __func__, data->ts_status);
	} else {
		KC_TS_DEV_DBG("%s: Double Tap Mode OFF\n", __func__);
		if(data->ts_status == TS_STATUS_SLEEP)
			data->ts_status = TS_STATUS_DEEP_SLEEP;
		else
			pr_err("%s Mode OFF: abnormal Status [%d]\n"
					, __func__, data->ts_status);
	}
	KC_TS_DEV_DBG("%s: done\n", __func__);

done:
	mutex_unlock(&data->lock);
	return err;
}

static int mxt_deep_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kc_ts_data *data = i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;
	int err = 0;

	KC_TS_DEV_DBG("%s: start\n", __func__);
	mutex_lock(&data->lock);
	if (!input_dev->users) {
		pr_err("%s input_dev->users disable\n", __func__);
		goto done;
	}

	if(data->ts_double_tap_mode){
		KC_TS_DEV_DBG("%s: Double Tap Mode ON\n", __func__);
		if(data->ts_status == TS_STATUS_DEEP_SLEEP){
			err = mxt_resume(data);
			if (err) {
				pr_err("failed in resume process.\n");
				goto done;
			}
			data->ts_status = TS_STATUS_PRE_ACTIVE;
		} else
			pr_err("%s Mode ON: abnormal Status [%d]\n"
					, __func__, data->ts_status);
	} else {
		KC_TS_DEV_DBG("%s: Double Tap Mode OFF\n", __func__);
		if(data->ts_status == TS_STATUS_DEEP_SLEEP)
			data->ts_status = TS_STATUS_SLEEP;
		else
			pr_err("%s Mode OFF: abnormal Status [%d]\n"
					, __func__, data->ts_status);
	}
done:
	mutex_unlock(&data->lock);
	return err;
}

static int mxt_esd_process(struct kc_ts_data *data)
{
	struct mxt_info *info = &data->vdata->info;
	struct i2c_client *client = data->vdata->client;
	u8 val;
	int err = 0;

	mxt_read_reg(client, MXT_FAMILY_ID, &val);
	if (val != info->family_id) {
		/*
		 * Can't read family id correctly.
		 * Recovery process start.
		 */
#ifdef FEATURE_TOUCH_TEST
		mxt_write_log(data, KC_TS_LOG_ESD, NULL);
#endif
		pr_err("%s: Recovery start! Family ID = %02X\n", __func__, val);
		if (data->is_enable) {
			disable_irq_nosync(data->irq);
			data->is_enable = false;
		}
		err = mxt_restart(data);
		if (err){
			pr_err("%s: Failed to restart!\n", __func__);
			goto ic_none;
		}
		if (mxt_reset_status(data))
			pr_err("%s: Failed to reset status!\n", __func__);
		if (!data->is_enable) {
			enable_irq(data->irq);
			data->is_enable = true;
		}
		/* Read dummy to avoid inconsistent */
	} else
		KC_TS_DEV_DBG("%s: Correct Family ID = %02X\n", __func__, val);

ic_none:
	return err;

}

static const struct kc_ts_operations mxt_operations = {
	.bustype	= BUS_I2C,
//	.read		= pixart_r_reg_byte,
	.multi_read	= mxt_bundle_read_reg,
	.write		= mxt_write_reg,
//	.multi_write	= pixart_w_reg_nbytes,
#ifdef FEATURE_TOUCH_TEST
	.write_log	= mxt_write_log,
#endif
	.interrupt	= mxt_interrupt,
	.power_off	= mxt_suspend,
	.power_on	= mxt_resume,
	.clear		= mxt_input_report_clear,
	.calibrate	= mxt_calibrate,
	.input_open	= mxt_input_open,
	.input_close	= mxt_input_close,
	.esd_proc	= mxt_esd_process,
	.ioctl		= mxt_ts_ioctl,
	.reference	= mxt_get_t37_data,
	.pass_reference = mxt_pass_t37_data,
	.reset		= mxt_reset_and_delay,
	.initialize	= mxt_initialize,
	.resolution	= mxt_calc_resolution,
	.check_reg_nv	= mxt_check_reg_nv,
	.backup_nv	= mxt_backup_nv,
	.enable		= mxt_enable,
	.disable	= mxt_disable,
};

static struct kc_ts_platform_data mxt_platform_data = {
	.config			= mxt_config_data,
	.config_length		= ARRAY_SIZE(mxt_config_data),
	.x_size			= MXT_PANEL_WIDTH,
	.y_size			= MXT_PANEL_HEIGHT,
	.irq_gpio		= MXT_TS_GPIO_IRQ,
	.reset_gpio		= MXT_TS_RESET_GPIO,
	.orient			= MXT_DIAGONAL,
	.irqflags		= IRQF_TRIGGER_LOW | IRQF_ONESHOT,
	.init_hw		= mxt_init_hw,
	.reset_hw		= mxt_reset_hw,
	.shutdown		= mxt_shutdown_hw,
};

static int __devinit mxt_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	const struct kc_ts_platform_data *pdata;
	struct kc_ts_data *data;
	struct kc_ts_vendor_data *vdata;
	int err;
	int rc = 0;

	KC_TS_DEV_DBG("%s is called.\n", __func__);
	usleep_range(1000, 1000);

	pr_info("%s Regulator get\n", __func__);
	reg_touch = regulator_get(NULL, "8941_l22");
	if(IS_ERR(reg_touch)){
		pr_err("%s regulator is not get\n",__func__);
	}

	if(reg_touch)
		rc = regulator_set_optimum_mode(reg_touch, 30000);
	if(rc < 0)
		pr_err("%s regulator set error\n",__func__);
	else
		KC_TS_DEV_DBG("%s regulator set 0x%02x\n",__func__,rc);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s: need I2C_FUNC_I2C\n",__func__);
		return -ENODEV;
		/*	goto err_check_functionality_failed;	*/
	}

	data = kzalloc(sizeof(struct kc_ts_data), GFP_KERNEL);
	if (!data) {
		pr_err("%s: Failed to allocate memory for ts.\n",__func__);
		return -ENOMEM;
	}

	vdata = kzalloc(sizeof (struct kc_ts_vendor_data), GFP_KERNEL);
	if (!vdata) {
		pr_err("%s: Failed to allocate memory for vdata.\n",__func__);
		err = -ENOMEM;
		goto err_free_mem;
	}

	memset(data, 0, sizeof(struct kc_ts_data));
	memset(vdata, 0, sizeof(struct kc_ts_vendor_data));

#ifdef CONFIG_OF
	client->dev.platform_data = &mxt_platform_data;
#endif
	pdata = client->dev.platform_data;
	if (!pdata)
		return -EINVAL;

	data->tops = &mxt_operations;
	data->pdata = pdata;
	data->vdata = vdata;
	data->dev = &client->dev;
	data->irq = client->irq;
	data->vdata->client = client;
	data->vdata->state = INIT;

	data->ts_sequence = TS_SEQ_PROBE_START;
	err = kc_ts_probe(data);
	if (err)
		goto done;

	KC_TS_DEV_DBG("%s is completed.\n", __func__);
	return 0;

err_free_mem:
	kfree(data);
done:
	return err;

}

static void mxt_shutdown(struct i2c_client *client)
{
	struct kc_ts_data *data = i2c_get_clientdata(client);

	KC_TS_DEV_DBG("%s is called.\n", __func__);
	kc_ts_shutdown(data);
	KC_TS_DEV_DBG("%s is completed.\n", __func__);
}

static int __devexit mxt_remove(struct i2c_client *client)
{
	struct kc_ts_data *data = i2c_get_clientdata(client);

	KC_TS_DEV_DBG("%s is called.\n", __func__);

	kfree(data->vdata->object_table);
	kc_ts_remove(data);

	KC_TS_DEV_DBG("%s is completed.\n", __func__);
	return 0;
}

static const struct i2c_device_id mxt_id[] = {
	{ "mXT224S", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mxt_id);

static const struct dev_pm_ops mxt_pm_ops = {
	.suspend = mxt_deep_suspend,
	.resume = mxt_deep_resume,
};

#ifdef CONFIG_OF
static struct of_device_id mxt_match_table[] = {
	{ .compatible = "atmel,mxt_kc",},
	{ },
};
#else
#define mxt_match_table NULL
#endif

static struct i2c_driver mxt_driver = {
	.driver = {
		.name	= "atmel_mxt_kc",
		.owner	= THIS_MODULE,
		.pm		= &mxt_pm_ops,
		.of_match_table = mxt_match_table,
	},
	.probe		= mxt_probe,
	.remove		= __devexit_p(mxt_remove),
	.shutdown	= mxt_shutdown,
	.id_table	= mxt_id,
};

#ifdef CONFIG_AFTER_KERNEL_3_3
module_i2c_driver(mxt_driver);
#else /* CONFIG_AFTER_KERNEL_3_3 */
static int __init mxt_init(void)
{

	return i2c_add_driver(&mxt_driver);
}

static void __exit mxt_exit(void)
{
	i2c_del_driver(&mxt_driver);
}

module_init(mxt_init);
module_exit(mxt_exit);
#endif /* CONFIG_AFTER_KERNEL_3_3 */

/* Module information */
MODULE_AUTHOR("KYOCERA Corporation");
MODULE_DESCRIPTION("Atmel maXTouch Touchscreen driver");
MODULE_LICENSE("GPL");
