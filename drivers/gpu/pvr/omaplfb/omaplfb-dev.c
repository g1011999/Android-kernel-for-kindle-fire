/*
 * omaplfb-dev.c
 *
 * Copyright (C) 2011 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 * Author: Gustavo Diaz (gusdp@ti.com)
 *
 */

#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/io.h>

#if defined(LDM_PLATFORM)
#include <linux/platform_device.h>
#if defined(SGX_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif
#endif

#include "img_defs.h"
#include "servicesext.h"
#include "kerneldisplay.h"
#include "omaplfb.h"
#include <video/omaplfb-dev.h>
#include <video/dsscomp.h>

static int gomaplfb_dev_open_cnt;

static int omaplfb_open(struct inode *inode, struct file *filp)
{
	/* HWC in HC is the only one which opens this device, try using the
	 * DSSComp for every flip
	 */
	gomaplfb_dev_open_cnt++;
	if (gomaplfb_dev_open_cnt > 0) {
		set_use_dsscomp(1);
		DEBUG_PRINTK("DSSComp will be used for flipping");
	}
	return 0;
}

static int omaplfb_release(struct inode *inode, struct file *filp)
{
	gomaplfb_dev_open_cnt--;
	if (gomaplfb_dev_open_cnt <= 0) {
		set_use_dsscomp(0);
		omaplfb_disable_cloning_alldisp();
	}
	return 0;
}

static long omaplfb_ioctl(struct file *filp, unsigned int cmd,
	unsigned long arg)
{
	int r = 0;
	void __user *ptr = (void __user *)arg;

	switch (cmd) {
	case OMAPLFB_CLONING_ENABLE:
	{
		struct omaplfb_clone_cmd clone_cmd;
		r = copy_from_user(&clone_cmd, ptr, sizeof(clone_cmd));
		if (r)
			break;
		r = omaplfb_enable_cloning(clone_cmd.mgr_id_src,
			clone_cmd.mgr_id_dst, OMAPLFB_CLONING_BUFFER_NUM);
		break;
	}
	case OMAPLFB_CLONING_DISABLE:
	{
		struct omaplfb_clone_cmd clone_cmd;
		r = copy_from_user(&clone_cmd, ptr, sizeof(clone_cmd));
		if (r)
			break;
		r = omaplfb_disable_cloning(clone_cmd.mgr_id_src);
		break;
	}
	case OMAPLFB_DSSCOMP_SETUP:
	{
		struct omaplfb_dsscomp_info info;
		r = copy_from_user(&info, ptr, sizeof(info));
		if (r)
			break;
		r = omaplfb_dsscomp_setup(&info);
		break;
	}

	default:
		r = -EINVAL;
	}

	return r;
}

static const struct file_operations omaplfb_fops = {
	.owner		= THIS_MODULE,
	.open		= omaplfb_open,
	.release	= omaplfb_release,
	.unlocked_ioctl = omaplfb_ioctl,
};

static int create_dev_node(struct platform_device *pdev,
	struct omaplfb_device *odev)
{
	int ret;

	odev->node.minor = MISC_DYNAMIC_MINOR;
	odev->node.name = OMAPLFB_DEV_NAME;
	odev->node.mode = 0666;
	odev->node.fops = &omaplfb_fops;

	ret = misc_register(&odev->node);
	if (ret) {
		WARNING_PRINTK("failed to register misc device");
		return ret;
	}

	return 0;
}

static int omaplfb_probe(struct platform_device *pdev)
{
	struct omaplfb_device *odev;

	odev = kzalloc(sizeof(*odev), GFP_KERNEL);

	if (!odev)
		return -ENOMEM;

	if (OMAPLFBInit(odev) != OMAP_OK) {
		dev_err(&pdev->dev, "failed to setup omaplfb\n");
		kfree(odev);
		return -ENODEV;
	}

	odev->dev = &pdev->dev;
	platform_set_drvdata(pdev, odev);
	omaplfb_create_sysfs(odev);
	set_use_dsscomp(0);

	return create_dev_node(pdev, odev);
}

static int omaplfb_remove(struct platform_device *pdev)
{
	struct omaplfb_device *odev;

	odev = platform_get_drvdata(pdev);

	omaplfb_disable_cloning_alldisp();
	omaplfb_remove_sysfs(odev);
	misc_deregister(&odev->node);

	if (OMAPLFBDeinit() != OMAP_OK)
		WARNING_PRINTK("Driver cleanup failed");

	kfree(odev);

	return 0;
}

#if defined(LDM_PLATFORM)

static OMAP_BOOL bDeviceSuspended;

static void OMAPLFBCommonSuspend(void)
{
	if (bDeviceSuspended) {
		DEBUG_PRINTK("Driver is already suspended");
		return;
	}

	OMAPLFBDriverSuspend();
	bDeviceSuspended = OMAP_TRUE;
}

#if defined(SGX_EARLYSUSPEND) && defined(CONFIG_HAS_EARLYSUSPEND)

/*
 * Android specific, driver is requested to be suspended
 * in: ea_event
 */
static void OMAPLFBDriverSuspend_Entry(struct early_suspend *ea_event)
{
	DEBUG_PRINTK("Requested driver suspend");
	OMAPLFBCommonSuspend();
}

/*
 * Android specific, driver is requested to be suspended
 * in: ea_event
 */
static void OMAPLFBDriverResume_Entry(struct early_suspend *ea_event)
{
	DEBUG_PRINTK("Requested driver resume");
	OMAPLFBDriverResume();
	bDeviceSuspended = OMAP_FALSE;
}

static struct platform_driver omaplfb_driver = {
	.driver = {
		.name = DRVNAME,
		.owner  = THIS_MODULE,
	},
	.probe = omaplfb_probe,
	.remove = omaplfb_remove,
};

static struct early_suspend omaplfb_early_suspend = {
	.suspend = OMAPLFBDriverSuspend_Entry,
	.resume = OMAPLFBDriverResume_Entry,
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
};

#else /* defined(SGX_EARLYSUSPEND) && defined(CONFIG_HAS_EARLYSUSPEND) */

/*
 * Function called when the driver is requested to be suspended
 * in: pDevice, state
 */
static int OMAPLFBDriverSuspend_Entry(struct platform_device *pDevice,
	pm_message_t state)
{
	DEBUG_PRINTK("Requested driver suspend");
	OMAPLFBCommonSuspend();
	return 0;
}

/*
 * Function called when the driver is requested to resume
 * in: pDevice
 */
static int OMAPLFBDriverResume_Entry(struct platform_device *pDevice)
{
	DEBUG_PRINTK("Requested driver resume");
	OMAPLFBDriverResume();
	bDeviceSuspended = OMAP_FALSE;
	return 0;
}

/*
 * Function called when the driver is requested to shutdown
 * in: pDevice
 */
static IMG_VOID OMAPLFBDriverShutdown_Entry(
	struct platform_device *pDevice)
{
	DEBUG_PRINTK("Requested driver shutdown");
	OMAPLFBCommonSuspend();
}

static struct platform_driver omaplfb_driver = {
	.driver = {
		.name = DRVNAME,
		.owner  = THIS_MODULE,
	},
	.probe = omaplfb_probe,
	.remove = omaplfb_remove,
	.suspend = OMAPLFBDriverSuspend_Entry,
	.resume	= OMAPLFBDriverResume_Entry,
	.shutdown = OMAPLFBDriverShutdown_Entry,
};

#endif /* defined(SGX_EARLYSUSPEND) && defined(CONFIG_HAS_EARLYSUSPEND) */

#endif /* defined(LDM_PLATFORM) */

/*
 * Driver init function
 */
static int __init OMAPLFB_Init(void)
{
#if defined(LDM_PLATFORM)
	DEBUG_PRINTK("Registering platform driver");
	if (platform_driver_register(&omaplfb_driver))
		return -ENODEV;
#if defined(SGX_EARLYSUSPEND) && defined(CONFIG_HAS_EARLYSUSPEND)
	register_early_suspend(&omaplfb_early_suspend);
	DEBUG_PRINTK("Registered early suspend support");
#endif
#endif
	omaplfb_dsscomp_init();
	return 0;
}

/*
 * Driver exit function
 */
static IMG_VOID __exit OMAPLFB_Cleanup(IMG_VOID)
{
#if defined(LDM_PLATFORM)
	DEBUG_PRINTK("Removing platform driver");
	platform_driver_unregister(&omaplfb_driver);
#if defined(SGX_EARLYSUSPEND) && defined(CONFIG_HAS_EARLYSUSPEND)
	DEBUG_PRINTK("Removed early suspend support");
	unregister_early_suspend(&omaplfb_early_suspend);
#endif
#endif
}

late_initcall(OMAPLFB_Init);
module_exit(OMAPLFB_Cleanup);
