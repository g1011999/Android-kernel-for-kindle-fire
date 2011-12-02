/*
 * linux/drivers/video/omap2/dsscomp/device.c
 *
 * DSS Composition file device and ioctl support
 *
 * Copyright (C) 2011 Texas Instruments, Inc
 * Author: Lajos Molnar <molnar@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define DEBUG

#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/file.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/anon_inodes.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/seq_file.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/sched.h>

#define MODULE_NAME	"dsscomp"

#include <plat/display.h>
#include <video/dsscomp.h>
#include <plat/dsscomp.h>
#include "dsscomp.h"
#include <mach/tiler.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

static int opencnt;
static bool blanked;
static struct dsscomp_dev *cdev;

static u32 hwc_virt_to_phys(u32 arg)
{
	pmd_t *pmd;
	pte_t *ptep;

	pgd_t *pgd = pgd_offset(current->mm, arg);
	if (pgd_none(*pgd) || pgd_bad(*pgd))
		return 0;

	pmd = pmd_offset(pgd, arg);
	if (pmd_none(*pmd) || pmd_bad(*pmd))
		return 0;

	ptep = pte_offset_map(pmd, arg);
	if (ptep && pte_present(*ptep))
		return (PAGE_MASK & *ptep) | (~PAGE_MASK & arg);

	return 0;
}

static long setup_mgr(struct dsscomp_dev *cdev,
	struct dsscomp_setup_mgr_data *d, bool blankpost)
{
	int i;
	struct omap_dss_device *dev;
	struct omap_overlay_manager *mgr;

	dump_comp_info(cdev, d, "queue");
	for (i = 0; i < d->num_ovls; i++)
		dump_ovl_info(cdev, d->ovls + i);

	/* verify display is valid and connected */
	if (d->mgr.ix >= cdev->num_displays)
		return -EINVAL;
	dev = cdev->displays[d->mgr.ix];
	if (!dev)
		return -EINVAL;

	mgr = dev->manager;
	if (!mgr)
		return -ENODEV;

	/* ignore frames while we are suspended */
	if ((blanked) && (!blankpost)) {
		if (debug & DEBUG_PHASES)
			dev_info(DEV(cdev), "[%08x] ignored\n", d->sync_id);
		return 0;
	}
	dsscomp_prepdata(d);
	return dsscomp_createcomp(mgr, d, blankpost) ? 0 : -EINVAL;
}

void dsscomp_prepdata(struct dsscomp_setup_mgr_data *d)
{
	int i;

	for (i = 0; i < d->num_ovls; i++) {
		struct dss2_ovl_info *oi = d->ovls + i;
		u32 addr = (u32) oi->address;
		if (oi->cfg.enabled) {
			/* convert addresses to user space */
			oi->ba = hwc_virt_to_phys(addr);
			if (oi->cfg.color_mode == OMAP_DSS_COLOR_NV12) {
				oi->uv = hwc_virt_to_phys(addr +
					oi->cfg.height * oi->cfg.stride);
				tiler_set_buf_state(oi->ba, TILBUF_BUSY);
			}
		}
	}
}
EXPORT_SYMBOL(dsscomp_prepdata);

void dsscomp_prepdata_drop(struct dsscomp_setup_mgr_data *d)
{
	int i;
	for (i = 0; i < d->num_ovls; i++) {
		struct dss2_ovl_info *oi = d->ovls + i;
		if (oi->cfg.enabled)
			if (oi->cfg.color_mode == OMAP_DSS_COLOR_NV12)
				tiler_set_buf_state(oi->ba, TILBUF_FREE);
	}
}
EXPORT_SYMBOL(dsscomp_prepdata_drop);

dsscomp_t dsscomp_createcomp(struct omap_overlay_manager *mgr,
	struct dsscomp_setup_mgr_data *d, bool blankpost)
{
	int i, r;
	dsscomp_t comp;

	comp = dsscomp_new_sync_id(mgr, d->sync_id, blankpost);
	if (IS_ERR(comp))
		goto cleanup;

	r = dsscomp_set_mgr(comp, &d->mgr);

	for (i = 0; i < d->num_ovls; i++) {
		struct dss2_ovl_info *oi = d->ovls + i;
		r = r ? : dsscomp_set_ovl(comp, oi);
	}

	r = r ? : dsscomp_setup(comp, d->mode, d->win);
	if (r)
		dsscomp_drop(comp);
	else if (d->mode & DSSCOMP_SETUP_APPLY)
		r = dsscomp_apply(comp);

	if (r)
		goto cleanup;

	return comp;
cleanup:
	for (i = 0; i < d->num_ovls; i++) {
		struct dss2_ovl_info *oi = d->ovls + i;
		if (oi->cfg.enabled &&
		    oi->cfg.color_mode == OMAP_DSS_COLOR_NV12)
			tiler_set_buf_state(oi->ba, TILBUF_FREE);
	}
	return NULL;
}
EXPORT_SYMBOL(dsscomp_createcomp);

static long query_display(struct dsscomp_dev *cdev,
					struct dsscomp_display_info *dis)
{
	struct omap_dss_device *dev;
	struct omap_overlay_manager *mgr;
	int i;

	/* get display */
	if (dis->ix >= cdev->num_displays)
		return -EINVAL;
	dev = cdev->displays[dis->ix];
	if (!dev)
		return -EINVAL;
	mgr = dev->manager;

	/* fill out display information */
	dis->channel = dev->channel;

	/* use smart_disable if present */
	if (dev->driver->smart_is_enabled)
		dis->enabled = dev->driver->smart_is_enabled(dev);
	/* show resume info for suspended displays */
	else if (dev->state == OMAP_DSS_DISPLAY_SUSPENDED)
		dis->enabled = OMAP_DSS_DISPLAY_DISABLED;
	else
		dis->enabled  = dev->state != OMAP_DSS_DISPLAY_DISABLED;

	dis->overlays_available = 0;
	dis->overlays_owned = 0;
	dis->s3d_info = dev->panel.s3d_info;
	dis->state = dev->state;
	dis->timings = dev->panel.timings;
	if (!strcmp(dev->name, "hdmi"))
		get_hdmi_mode_code(&dis->timings, &dis->mode, &dis->code);

	/* find all overlays available for/owned by this display */
	for (i = 0; i < cdev->num_ovls && dis->enabled; i++) {
		if (cdev->ovls[i]->manager == mgr)
			dis->overlays_owned |= 1 << i;
		else if (!cdev->ovls[i]->info.enabled)
			dis->overlays_available |= 1 << i;
	}
	dis->overlays_available |= dis->overlays_owned;

	/* fill out manager information */
	if (mgr) {
		dis->mgr.alpha_blending = mgr->info.alpha_enabled;
		dis->mgr.default_color = mgr->info.default_color;
		dis->mgr.interlaced = !strcmp(dev->name, "hdmi") &&
							is_hdmi_interlaced();
		dis->mgr.trans_enabled = mgr->info.trans_enabled;
		dis->mgr.trans_key = mgr->info.trans_key;
		dis->mgr.trans_key_type = mgr->info.trans_key_type;
		dis->mgr.ix = mgr->id;
	} else {
		/* display is disabled if it has no manager */
		memset(&dis->mgr, 0, sizeof(dis->mgr));
	}

	return 0;
}

static long check_ovl(struct dsscomp_dev *cdev,
					struct dsscomp_check_ovl_data *chk)
{
	u16 x_decim, y_decim;
	bool three_tap;
	struct omap_dss_device *dev;
	struct omap_overlay_manager *mgr;
	int i;
	long allowed = 0;
	bool checked_vid = false, scale_ok = false;
	struct dss2_ovl_cfg *c = &chk->ovl.cfg;

	/*Substitute width with height when screen is in portrait mode*/
	if (c->rotation % 2)
		swap(c->win.w, c->win.h);

	/* get display */
	if (chk->mgr.ix >= cdev->num_displays)
		return -EINVAL;
	dev = cdev->displays[chk->mgr.ix];
	if (!dev)
		return -EINVAL;

	/* HACK: prevent executing IOCTL when driver in suspend mode to avoid
		kernel crash */
	if (dev->state != OMAP_DSS_DISPLAY_ACTIVE)
		return -EINVAL;

	mgr = dev->manager;

	/* check manager setup - OMAP4 only for now */
	if (!chk->mgr.alpha_blending)
		return -EINVAL;

	/* normalize decimation */
	if (!c->decim.min_x)
		c->decim.min_x = 1;
	if (!c->decim.min_y)
		c->decim.min_y = 1;
	if (!c->decim.max_x)
		c->decim.max_x = 255;
	if (!c->decim.max_y)
		c->decim.max_y = 255;

	/* check scaling support */
	for (i = 0; i < cdev->num_ovls; i++) {
		/* verify color format support */
		if (c->color_mode & ~cdev->ovls[i]->supported_modes)
			continue;

		/* verify scaling on GFX and VID pipes */
		if (!i || !checked_vid) {
			scale_ok = !dispc_scaling_decision(c->crop.w, c->crop.h,
				c->win.w, c->win.h, i, c->color_mode, mgr->id,
				c->rotation, c->decim.min_x, c->decim.max_x,
				c->decim.min_y, c->decim.max_y,
				&x_decim, &y_decim, &three_tap);

			/* update minimum decimation needs to support ovl */
			if (scale_ok) {
				if (x_decim > c->decim.min_x)
					c->decim.min_x = x_decim;
				if (y_decim > c->decim.min_y)
					c->decim.min_y = y_decim;
			}
		}
		checked_vid = i;
		if (scale_ok)
			allowed |= 1 << i;
	}

	return allowed;
}

static long wait(struct dsscomp_dev *cdev, struct dsscomp_wait_data *wd)
{
	struct omap_overlay_manager *mgr;
	dsscomp_t comp;

	/* get manager */
	if (wd->ix >= cdev->num_displays || !cdev->displays[wd->ix])
		return -EINVAL;
	mgr = cdev->displays[wd->ix]->manager;
	if (!mgr)
		return -ENODEV;

	/* get composition */
	comp = dsscomp_find(mgr, wd->sync_id);
	if ((!comp) || (IS_ERR(comp)))
		return 0;

	return dsscomp_wait(comp, wd->phase, usecs_to_jiffies(wd->timeout_us));
}

static void fill_cache(struct dsscomp_dev *cdev)
{
	unsigned long i;
	struct omap_dss_device *dssdev = NULL;

	cdev->num_ovls = min(omap_dss_get_num_overlays(), MAX_OVERLAYS);
	for (i = 0; i < cdev->num_ovls; i++)
		cdev->ovls[i] = omap_dss_get_overlay(i);

	cdev->num_mgrs = min(omap_dss_get_num_overlay_managers(), MAX_MANAGERS);
	for (i = 0; i < cdev->num_mgrs; i++)
		cdev->mgrs[i] = omap_dss_get_overlay_manager(i);

	for_each_dss_dev(dssdev) {
		const char *name = dev_name(&dssdev->dev);
		if (strncmp(name, "display", 7) ||
		    strict_strtoul(name + 7, 10, &i) ||
		    i >= MAX_DISPLAYS)
			continue;

		if (cdev->num_displays <= i)
			cdev->num_displays = i + 1;

		cdev->displays[i] = dssdev;
		dev_dbg(DEV(cdev), "display%lu=%s\n", i, dssdev->driver_name);
	}
	dev_info(DEV(cdev), "found %d displays and %d overlays\n",
				cdev->num_displays, cdev->num_ovls);
}

static long comp_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int r = 0;
	struct miscdevice *dev = filp->private_data;
	struct dsscomp_dev *cdev = container_of(dev, struct dsscomp_dev, dev);
	void __user *ptr = (void __user *)arg;

	switch (cmd) {
	case DSSCOMP_SETUP_MGR:
	{
		struct {
			struct dsscomp_setup_mgr_data set;
			struct dss2_ovl_info ovl[MAX_OVERLAYS];
		} p;
		r = copy_from_user(&p.set, ptr, sizeof(p.set)) ? :
		    p.set.num_ovls >= ARRAY_SIZE(p.ovl) ? -EINVAL :
		    copy_from_user(&p.ovl, (void __user *)arg + sizeof(p.set),
					sizeof(*p.ovl) * p.set.num_ovls) ? :
		    setup_mgr(cdev, &p.set, false);
		break;
	}
	case DSSCOMP_QUERY_DISPLAY:
	{
		struct dsscomp_display_info dis;
		r = copy_from_user(&dis, ptr, sizeof(dis)) ? :
		    query_display(cdev, &dis) ? :
		    copy_to_user(ptr, &dis, sizeof(dis));
		break;
	}
	case DSSCOMP_CHECK_OVL:
	{
		struct dsscomp_check_ovl_data chk;
		r = copy_from_user(&chk, ptr, sizeof(chk)) ? :
		    check_ovl(cdev, &chk) ? :
		    copy_to_user(ptr, &chk, sizeof(chk));
		break;
	}
	case DSSCOMP_WAIT:
	{
		struct dsscomp_wait_data wd;
		r = copy_from_user(&wd, ptr, sizeof(wd)) ? :
		    wait(cdev, &wd);
		break;
	}
	default:
		r = -EINVAL;
	}
	return r;
}

#ifdef CONFIG_EARLYSUSPEND
static void dsscomp_early_suspend(struct early_suspend *h)
{
	int d, o;
	bool isDisplayEn;
	struct omap_dss_device *dev;
	struct {
		struct dsscomp_setup_mgr_data set;
		struct dss2_ovl_info ovl[MAX_OVERLAYS];
	} p = {
		.set.mgr.alpha_blending = 1
		};
	blanked = true;

	for (d = 0; d < cdev->num_displays; d++) {
		dev = cdev->displays[d];

		/* use smart_disable if present */
		if (dev->driver->smart_is_enabled)
			isDisplayEn = dev->driver->smart_is_enabled(dev);
		/* show resume info for suspended displays */
		else if (dev->state == OMAP_DSS_DISPLAY_SUSPENDED)
			isDisplayEn = OMAP_DSS_DISPLAY_DISABLED;
		else
			isDisplayEn  = dev->state != OMAP_DSS_DISPLAY_DISABLED;

		if (!isDisplayEn)
			continue;

		p.set.num_ovls = 0;
		/* find all overlays owned by this display,
		* and disable them except overlay0
		*/
		for (o = 0; o < cdev->num_ovls; o++) {
			if (cdev->ovls[o]->manager == dev->manager) {
				if ((d == 0) && (o == 0))
					continue;
				p.ovl[p.set.num_ovls].cfg.ix = o;
				p.ovl[p.set.num_ovls].cfg.enabled = false;
				p.set.num_ovls++;
			}
		}

		p.set.mgr.ix = d;
		p.set.mode = DSSCOMP_SETUP_DISPLAY;
		setup_mgr(cdev, &p.set, true);
	}
}

static void dsscomp_late_resume(struct early_suspend *h)
{
	dsscomp_release_active_comps();
	blanked = false;
}

static struct early_suspend early_suspend_info = {
	.suspend = dsscomp_early_suspend,
	.resume = dsscomp_late_resume,
	.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING,
};
#endif


/* must implement open for filp->private_data to be filled */
static int comp_open(struct inode *inode, struct file *filp)
{
	opencnt++;
#ifdef CONFIG_HAS_EARLYSUSPEND
	if (opencnt == 1)
		register_early_suspend(&early_suspend_info);
#endif
	return 0;
}

static int comp_release(struct inode *inode, struct file *filp)
{
	opencnt--;
#ifdef CONFIG_HAS_EARLYSUSPEND
	if (opencnt == 0)
		unregister_early_suspend(&early_suspend_info);
#endif
	return 0;
}

static const struct file_operations comp_fops = {
	.owner		= THIS_MODULE,
	.open		= comp_open,
	.release	= comp_release,
	.unlocked_ioctl = comp_ioctl,
};

static int dsscomp_probe(struct platform_device *pdev)
{
	int ret;
	cdev = kzalloc(sizeof(*cdev), GFP_KERNEL);
	if (!cdev) {
		pr_err("dsscomp: failed to allocate device.\n");
		return -ENOMEM;
	}
	cdev->dev.minor = MISC_DYNAMIC_MINOR;
	cdev->dev.name = "dsscomp";
	cdev->dev.mode = 0666;
	cdev->dev.fops = &comp_fops;

	ret = misc_register(&cdev->dev);
	if (ret) {
		pr_err("dsscomp: failed to register misc device.\n");
		return ret;
	}
	cdev->dbgfs = debugfs_create_dir("dsscomp", NULL);
	if (IS_ERR_OR_NULL(cdev->dbgfs))
		dev_warn(DEV(cdev), "failed to create debug files.\n");

	platform_set_drvdata(pdev, cdev);

	fill_cache(cdev);

	/* initialize queues */
	dsscomp_queue_init(cdev);

	return 0;
}

static int dsscomp_remove(struct platform_device *pdev)
{
	struct dsscomp_dev *cdev = platform_get_drvdata(pdev);
	misc_deregister(&cdev->dev);
	debugfs_remove_recursive(cdev->dbgfs);

	dsscomp_queue_exit();
	kfree(cdev);

	return 0;
}

static struct platform_driver dsscomp_pdriver = {
	.probe = dsscomp_probe,
	.remove = dsscomp_remove,
	.driver = { .name = MODULE_NAME, .owner = THIS_MODULE }
};

static struct platform_device dsscomp_pdev = {
	.name = MODULE_NAME,
	.id = -1
};

static int __init dsscomp_init(void)
{
	int err = platform_driver_register(&dsscomp_pdriver);
	if (err)
		return err;

	err = platform_device_register(&dsscomp_pdev);
	if (err)
		platform_driver_unregister(&dsscomp_pdriver);
	return err;
}

static void __exit dsscomp_exit(void)
{
	platform_device_unregister(&dsscomp_pdev);
	platform_driver_unregister(&dsscomp_pdriver);
}

MODULE_LICENSE("GPL v2");
module_init(dsscomp_init);
module_exit(dsscomp_exit);
