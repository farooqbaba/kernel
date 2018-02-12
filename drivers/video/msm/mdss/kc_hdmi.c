/*
 * This software is contributed or developed by KYOCERA Corporation.
 * (C) 2013 KYOCERA Corporation
 *
 * drivers/video/msm/mdss/kc_hdmi.c
 *
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
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
*/

#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include "mdss_hdmi_hdcp.h"
#include "mdss_panel.h"
#include "mdss_hdmi_tx.h"

static int device_major;
static struct class *device_class;

/* Global Variables */

static long kc_hdmi_hdcp_ioctl(struct file *file, unsigned int cmd,
						unsigned long arg)
{
	long ret;
	struct kc_hdmi_aksv aksv;

	switch (cmd) {
	case IOCTL_GET_AKSV:
		pr_debug("%s: IOCTL_GET_AKSV\n", __func__);
		if (!access_ok(VERIFY_WRITE, (void __user *)arg,
						_IOC_SIZE(cmd))) {
			ret = -EFAULT;
			pr_err("%s: invalid access\n", __func__);
			goto done;
		}
		memset(&aksv, 0, sizeof(struct kc_hdmi_aksv));
		ret = kc_hdmi_get_aksv(file, &aksv);
		if (copy_to_user((void __user *)arg,
				 &aksv,
				 sizeof(struct kc_hdmi_aksv))) {
			ret = -EFAULT;
			pr_err("%s: copy_to_user error\n", __func__);
		}
		break;
	default:
		pr_debug("%s: cmd error[%X]\n", __func__, cmd);
		return -EINVAL;
		break;
	}
done:
	return ret;
}

static long kc_hdmi_tx_ioctl(struct file *file, unsigned int cmd,
						unsigned long arg)
{
	long ret;
	unsigned long val = 0;

	switch (cmd) {
	case IOCTL_GET_HDCP:
		pr_debug("%s: IOCTL_GET_HDCP\n", __func__);
		if (!access_ok(VERIFY_WRITE, (void __user *)arg,
						_IOC_SIZE(cmd))) {
			ret = -EFAULT;
			pr_err("%s: invalid access\n", __func__);
			goto done;
		}
		ret = kc_hdmi_get_hdcp(file, &val);
		if (copy_to_user((void __user *)arg,
				 &val,
				 sizeof(unsigned long))) {
			ret = -EFAULT;
			pr_err("%s: copy_to_user error\n", __func__);
		}
		break;
	default:
		pr_debug("%s: cmd error[%X]\n", __func__, cmd);
		return -EINVAL;
		break;
	}
done:
	return ret;
}

const struct file_operations kc_hdmi_hdcp_fops = {
	.owner = THIS_MODULE,
	.open = kc_hdmi_hdcp_open,
	.unlocked_ioctl = kc_hdmi_hdcp_ioctl,
	.release = kc_hdmi_hdcp_release,
};

const struct file_operations kc_hdmi_tx_fops = {
	.owner = THIS_MODULE,
	.open = kc_hdmi_tx_open,
	.unlocked_ioctl = kc_hdmi_tx_ioctl,
	.release = kc_hdmi_tx_release,
};

int kc_hdmi_hdcp_init(struct cdev *device_cdev)
{

	dev_t device_t = MKDEV(0, 0);
	struct device *class_dev_t = NULL;
	int ret;

	ret = alloc_chrdev_region(&device_t, 0, 1, "kc_hdmi_hdcp");
	if (ret)
		goto error;

	device_major = MAJOR(device_t);

	cdev_init(device_cdev, &kc_hdmi_hdcp_fops);
	device_cdev->owner = THIS_MODULE;
	device_cdev->ops = &kc_hdmi_hdcp_fops;
	ret = cdev_add(device_cdev, MKDEV(device_major, 0), 1);
	if (ret)
		goto err_unregister_chrdev;

	device_class = class_create(THIS_MODULE, "kc_hdmi_hdcp");
	if (IS_ERR(device_class)) {
		ret = -1;
		goto err_cleanup_cdev;
	};

	class_dev_t = device_create(device_class, NULL,
		MKDEV(device_major, 0), NULL, "kc_hdmi_hdcp");
	if (IS_ERR(class_dev_t)) {
		ret = -1;
		goto err_destroy_class;
	}

	return 0;

err_destroy_class:
	class_destroy(device_class);
err_cleanup_cdev:
	cdev_del(device_cdev);
err_unregister_chrdev:
	unregister_chrdev_region(device_t, 1);
error:
	return ret;
}
EXPORT_SYMBOL_GPL(kc_hdmi_hdcp_init);

int kc_hdmi_hdcp_exit(struct cdev *device_cdev)
{
	dev_t device_t = MKDEV(device_major, 0);

	if (device_class) {
		device_destroy(device_class, MKDEV(device_major, 0));
		class_destroy(device_class);
	}
	if (device_cdev) {
		cdev_del(device_cdev);
		unregister_chrdev_region(device_t, 1);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(kc_hdmi_hdcp_exit);

int kc_hdmi_tx_init(struct cdev *device_cdev)
{

	dev_t device_t = MKDEV(0, 0);
	struct device *class_dev_t = NULL;
	int ret;

	ret = alloc_chrdev_region(&device_t, 0, 1, "kc_hdmi_tx");
	if (ret)
		goto error;

	device_major = MAJOR(device_t);

	cdev_init(device_cdev, &kc_hdmi_tx_fops);
	device_cdev->owner = THIS_MODULE;
	device_cdev->ops = &kc_hdmi_tx_fops;
	ret = cdev_add(device_cdev, MKDEV(device_major, 0), 1);
	if (ret)
		goto err_unregister_chrdev;

	device_class = class_create(THIS_MODULE, "kc_hdmi_tx");
	if (IS_ERR(device_class)) {
		ret = -1;
		goto err_cleanup_cdev;
	};

	class_dev_t = device_create(device_class, NULL,
		MKDEV(device_major, 0), NULL, "kc_hdmi_tx");
	if (IS_ERR(class_dev_t)) {
		ret = -1;
		goto err_destroy_class;
	}

	return 0;

err_destroy_class:
	class_destroy(device_class);
err_cleanup_cdev:
	cdev_del(device_cdev);
err_unregister_chrdev:
	unregister_chrdev_region(device_t, 1);
error:
	return ret;
}
EXPORT_SYMBOL_GPL(kc_hdmi_tx_init);

int kc_hdmi_tx_exit(struct cdev *device_cdev)
{
	dev_t device_t = MKDEV(device_major, 0);

	if (device_class) {
		device_destroy(device_class, MKDEV(device_major, 0));
		class_destroy(device_class);
	}
	if (device_cdev) {
		cdev_del(device_cdev);
		unregister_chrdev_region(device_t, 1);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(kc_hdmi_tx_exit);


MODULE_AUTHOR("KYOCERA Corporation");
MODULE_DESCRIPTION("kc hdmi driver");
MODULE_LICENSE("GPL");

