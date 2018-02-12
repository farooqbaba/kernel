/*****************************************************************************
*
* Filename:      irda_eg1.c
* Version:       0.1
* Description:   IrDA driver
* Status:        Experimental
* Author:        KYOCERA Corporation
*
* This software is contributed or developed by KYOCERA Corporation.
* (C) 2013 KYOCERA Corporation
*
*	This program is free software; you can redistribute it and/or
*   modify it under the terms of the GNU General Public License
*   as published by the Free Software Foundation; only version 2.
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program; if not, write to the Free Software
*   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*
*****************************************************************************/
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <asm/gpio.h>
#include <linux/major.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>
#include "irda_eg.h"
#include <linux/of_gpio.h>

#include <mach/msm_serial_hs.h>

static struct irda_eg_info *irdaeg_info;
static struct platform_device *pDev;
static 	int gpio = -1;

static int irdaeg_fop_open(struct inode *inode, struct file *file) ;
static int irdaeg_fop_release(struct inode *inode, struct file *file) ;

int irdaeg_gpio_shutdown_port( void );

/* ------------------------------------------------------------------------------------
 *		irdaeg file operation
 * ------------------------------------------------------------------------------------ */
static int irdaeg_fop_open(struct inode *inode, struct file *file)
{


	printk( "%s eg1 en:%d\n", __func__, __LINE__ );


	file->private_data = irdaeg_info ;

	gpio_set_value_cansleep(gpio, 0);

	printk( "%s eg1 lv:%d\n", __func__, __LINE__ );
	return 0;
}

static int irdaeg_fop_release(struct inode *inode, struct file *file)
{
	printk( "%s eg1:%d\n", __func__, __LINE__ );
	
	return 0;
}


int irdaeg_gpio_shutdown_port()
{
	return gpio;
}
EXPORT_SYMBOL(irdaeg_gpio_shutdown_port);

static struct class *irdaeg_class;

struct file_operations irdaeg1_fops =
{
	.owner		= THIS_MODULE,
	.open		= irdaeg_fop_open,
	.release	= irdaeg_fop_release,
};

static char *irdaeg_devnode(struct device *dev, mode_t *mode)
{
	printk("irdaeg_devnode\n");

	if (mode)
		*mode = 0666;
	return kasprintf(GFP_KERNEL,"%s", dev_name(dev));
}

//static __devinit int irdaeg_probe(struct platform_device *pdev)
static int irdaeg_probe(struct platform_device *pdev)
{
	int rc;

	/* PM_GPIO */
	if (!pdev->dev.of_node) {
		pr_err("No platform supplied from device tree.\n");
		return -EINVAL;
	}

	pDev = pdev;

/* Set IrDA SD port */
	gpio = of_get_named_gpio(pDev->dev.of_node, "kc,irda-sd", 0);
	if (gpio < 0) {
		pr_err("%s eg1: of_get_named_gpio failed.\n", __func__);
		return -EINVAL;
	}

	rc = gpio_request(gpio, "irda-sd");
	if (rc) {
		pr_err("%s eg1: gpio_request failed.\n", __func__);
		return -EINVAL;
	}

	return 0;
}

//static int __devexit irdaeg_remove(struct platform_device *pdev)
static int irdaeg_remove(struct platform_device *pdev)
{

	gpio_free(gpio);
	gpio = -1;

	return 0;
}

static const struct of_device_id irdaeg_of_match[] = {
	{ .compatible = "kc,irdaeg1", },
	{},
};

static struct platform_driver irdaeg1_pd = {
	.driver = {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = irdaeg_of_match,
	},
	.probe = irdaeg_probe,
//	.remove = __devexit_p(irdaeg_remove),
	.remove = irdaeg_remove,
};


static int init_irdaeg( void )
{
	int ret = 0;

	printk( KERN_NOTICE"IRDAEG module is beeing initialized.\n" ) ;
	platform_driver_register(&irdaeg1_pd);

	irdaeg_info = kzalloc(sizeof(*irdaeg_info), GFP_KERNEL);
	if (irdaeg_info == NULL) {
		pr_err(MODULE_NAME ":kzalloc err.\n");
		return -ENOMEM;
	}
	irdaeg_class = class_create(THIS_MODULE, MODULE_NAME);

	ret = alloc_chrdev_region(&irdaeg_info->dev_num, 0, 1, MODULE_NAME);
	if (ret) {
		printk(MODULE_NAME "alloc_chrdev_region err.\n");
		return -ENODEV;
	}
	irdaeg_class->devnode = irdaeg_devnode;
	irdaeg_info->dev = device_create(irdaeg_class, NULL, irdaeg_info->dev_num,
				      irdaeg_info, MODULE_NAME);
	if (IS_ERR(irdaeg_info->dev)) {
		printk(MODULE_NAME ":device_create err.\n");
		return -ENODEV;
	}

	irdaeg_info->cdev = cdev_alloc();
	if (irdaeg_info->cdev == NULL) {
		printk(MODULE_NAME ":cdev_alloc err.\n");
		return -ENODEV;
	}
	cdev_init(irdaeg_info->cdev, &irdaeg1_fops);
	irdaeg_info->cdev->owner = THIS_MODULE;

	ret = cdev_add(irdaeg_info->cdev, irdaeg_info->dev_num, 1);
	if (ret)
		printk(MODULE_NAME ":cdev_add err=%d\n", -ret);
	else
		printk(MODULE_NAME ":irdaeg init OK..\n");

	printk( " %s driver installed.\n", MODULE_NAME );

	return ret;

}

static void exit_irdaeg( void )
{
	cdev_del(irdaeg_info->cdev);
	device_destroy(irdaeg_class, irdaeg_info->dev_num);
	unregister_chrdev_region(irdaeg_info->dev_num, 1);

	kfree(irdaeg_info);
	printk( "IRDAEG module is removed.\n" ) ;
}

module_init( init_irdaeg ) ;
module_exit( exit_irdaeg ) ;

