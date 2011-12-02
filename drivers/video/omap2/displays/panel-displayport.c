/*
 * panel-displayport.c
 * DisplayPort driver
 *
 * Copyright (C) 2011 Texas Instruments
 * Author: Salomon Chavez <schavezv@ti.com>
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

/* TODO:
- Detect monitor capabilities together with supported
	video timings choose the best resolution.
- Use PLLs to get the proper pixel clocks, compare if it is a better option.
- Implement a polling strategic for hot-plug monitor detection.
- Add routines to attend DP501 ISRs (synchronization, training, etc).
- Verify power management.
*/

#include <linux/input.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <plat/display.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/firmware.h>

#include "panel-displayport.h"
#include <linux/slab.h>

/* uncomment this line if you want debug prints: */
/* #define DP501_DEBUG */

#define DRIVER_DESC       "DP501 DisplayPort Driver"
#define DRIVER_NAME       "displayport_panel"


/********************************/
/*****    I2C Operations    *****/
/********************************/

int dp501_12c_device_write(struct displayport *sdp, u8 reg, u8 val, char *msg)
{
	/* write a byte (val) through i2c smbus to (reg) */
	int ret = i2c_smbus_write_byte_data(sdp->client, reg, val);
	if (ret < 0)
		dev_err(&sdp->client->dev,
			"i2c_smbus_write_byte_data failed (%s)\n", msg);
	return ret;
}

int dp501_12c_device_read(struct displayport *sdp, u8 reg, char *msg)
{
	/* read a byte (ret) through i2c smbus from (reg) */
	int ret = i2c_smbus_read_byte_data(sdp->client, reg);
	if (ret < 0)
		dev_err(&sdp->client->dev,
			"i2c_smbus_read_byte_data failed (%s)\n", msg);
	return ret;
}

/****************************************************/
/*****    DP501 Parade Configuration Section    *****/
/****************************************************/

/*******************************************************************
 * Routine:dp501_init
 * Description: Initialize DP501 display port transmitter
 ******************************************************************/
static void dp501_init(struct displayport *sdp)
{
	unsigned char data, Val1, Val2;
#ifdef DP501_DEBUG
	printk(KERN_INFO "%s:%s\\n", __FILE__, __func__);
#endif
	printk(KERN_ALERT "DisplaytPort: Initializing DP501\n");

	/* Configuring on Page 2 */
	sdp->client->addr = 0x0A;
	data = dp501_12c_device_read(sdp, CHIP_VER_L, "CHIP_VER_L");
#ifdef DP501_DEBUG
	printk(KERN_ALERT "DP: CHIP_VER_L P2.82h = 0x%08X\n", data);
#endif
	/* single ended input, high active */
	dp501_12c_device_write(sdp, SEL_PIO1, 0x02, "SEL_PIO1");
	dp501_12c_device_write(sdp, SEL_PIO2, 0x04, "SEL_PIO2");
	dp501_12c_device_write(sdp, SEL_PIO3, 0x10, "SEL_PIO3");


	/* Configuring on Page 0 */
	sdp->client->addr = 0x08;

	/* auto detect DVO timing*/
	dp501_12c_device_write(sdp, VCAPCTRL3, 0x30, "VCAPCTRL3");
	/* reset tpfifo at v blank */
	dp501_12c_device_write(sdp, LINK_CTRL0, 0x82, "LINK_CTRL0");
	/* RGB24, DVO mapping,crtc mode */
	dp501_12c_device_write(sdp, VCAPCTRL0, 0x00, "VCAPCTRL0");
	/* crtc follow mode */
	dp501_12c_device_write(sdp, VCAPCTRL4, 0x00, "VCAPCTRL4");
	/*set color depth 8bit (0x00: 6bit; 0x20: 8bit ; 0x40:10bit) */
	dp501_12c_device_write(sdp, MISC0, 0x20, "MISC0");
	/* DPCD readable */
	dp501_12c_device_write(sdp, HPDCTL0, 0xa9, "HPDCTL0");
	/* Scramble on */
	dp501_12c_device_write(sdp, QUALTEST_CTL, 0x00, "QUALTEST_CTL");
	/* SP link rate (0x0a : 2.7G; 0x06 : 1.62G) */
	dp501_12c_device_write(sdp, LINK_BW, 0x06, "LINK_BW");
	/*Set lane count to 1 (0x81: 1lane; 0x82: 2 lane; 0x84: 4 lane) */
	dp501_12c_device_write(sdp, LANE_CNT, 0x84, "LANE_CNT");
#ifdef DP501_DEBUG
	printk(KERN_ALERT "DP: set DP501 drive levels\n");
#endif
	/* Software trainig enabled */
	dp501_12c_device_write(sdp, SW_TRAIN_CTRL, 0x04, "SW_TRAIN_CTRL");
	/* Software set DRV level */
	dp501_12c_device_write(sdp, SW_DRV_SET, 0x01, "SW_DRV_SET");
	/* software set PRE level */
	dp501_12c_device_write(sdp, SW_PRE_SET, 0x01, "SW_PRE_SET");
#ifdef DP501_DEBUG
	printk(KERN_ALERT "DP: start DP501 link training\n");
#endif
	dp501_12c_device_write(sdp, TRAINING_CTL, 0x0d, "TRAINING_CTL");
	mdelay(100);
	Val1 = dp501_12c_device_read(sdp, LANE01_STATUS, "LANE01_STATUS");
	Val2 = dp501_12c_device_read(sdp, LANE23_STATUS, "LANE23_STATUS");
	if ((Val1 == 0x07) & (Val2 == 0x00))
		printk(KERN_ALERT "Training success, 1 Lane\n");
	else if ((Val1 == 0x77) & (Val2 == 0x00))
		printk(KERN_ALERT "Training success, 2 lanes\n");
	else if ((Val1 == 0x77) & (Val2 == 0x77))
		printk(KERN_ALERT "Training success, 4 lanes\n");
	else
		printk(KERN_ALERT "Training failed\n");

#ifdef DP501_DEBUG
	printk(KERN_ALERT "Val1 = 0x%08X\n", Val1);
	printk(KERN_ALERT "Val2 = 0x%08X\n", Val2);
#endif
	/* Force video on (0xc0: force video on, 0x80: pattern1 video off) */
	dp501_12c_device_write(sdp, LINK_STATE_CTRL, 0xc0, "LINK_STATE_CTRL");

#ifdef DP501_DEBUG
	data = dp501_12c_device_read(sdp, HTOTAL_L, "HTOTAL_L");
	printk(KERN_ALERT "DP: P0.10h HTOTAL_L = 0x%08X\n", data);
	data = dp501_12c_device_read(sdp, HTOTAL_H, "HTOTAL_H");
	printk(KERN_ALERT "DP: P0.11h HTOTAL_H = 0x%08X\n", data);
	data = dp501_12c_device_read(sdp, HSTART_L, "HSTART_L");
	printk(KERN_ALERT "DP: P0.12h HSTART_L = 0x%08X\n", data);
	data = dp501_12c_device_read(sdp, HSTART_H, "HSTART_H");
	printk(KERN_ALERT "DP: P0.13h HSTART_H = 0x%08X\n", data);
	data = dp501_12c_device_read(sdp, HWIDTH_L, "HWIDTH_L");
	printk(KERN_ALERT "DP: P0.14h HWIDTH_L = 0x%08X\n", data);
	data = dp501_12c_device_read(sdp, HWIDTH_H, "HWIDTH_H");
	printk(KERN_ALERT "DP: P0.15h HWIDTH_H = 0x%08X\n", data);
	data = dp501_12c_device_read(sdp, VTOTAL_L, "VTOTAL_L");
	printk(KERN_ALERT "DP: P0.16h VTOTAL_L = 0x%08X\n", data);
	data = dp501_12c_device_read(sdp, VTOTAL_H, "VTOTAL_H");
	printk(KERN_ALERT "DP: P0.17h VTOTAL_H = 0x%08X\n", data);
	data = dp501_12c_device_read(sdp, VSTART_L, "VSTART_L");
	printk(KERN_ALERT "DP: P0.18h VSTART_L = 0x%08X\n", data);
	data = dp501_12c_device_read(sdp, VSTART_H, "VSTART_H");
	printk(KERN_ALERT "DP: P0.19h VSTART_H = 0x%08X\n", data);
	data = dp501_12c_device_read(sdp, VHEIGHT_L, "VHEIGHT_L");
	printk(KERN_ALERT "DP: P0.1ah VHEIGHT_L = 0x%08X\n", data);
	data = dp501_12c_device_read(sdp, VHEIGHT_H, "VHEIGHT_H");
	printk(KERN_ALERT "DP: P0.1bh VHEIGHT_H = 0x%08X\n", data);
	data = dp501_12c_device_read(sdp, HSPHSW_L, "HSPHSW_L");
	printk(KERN_ALERT "DP: P0.1ch HSPHSW_L = 0x%08X\n", data);
	data = dp501_12c_device_read(sdp, HSPHSW_H, "HSPHSW_H");
	printk(KERN_ALERT "DP: P0.1dh HSPHSW_H = 0x%08X\n", data);
	data = dp501_12c_device_read(sdp, VSPVSW_L, "VSPVSW_L");
	printk(KERN_ALERT "DP: P0.1eh VSPVSW_L = 0x%08X\n", data);
	data = dp501_12c_device_read(sdp, VSPVSW_H, "VSPVSW_H");
	printk(KERN_ALERT "DP: P0.1fh VSPVSW_H = 0x%08X\n", data);
#endif
	printk(KERN_ALERT "DisplayPort: done initializing DP501\n");

	return;
}

static int DP501_PowerDownEncoder(struct displayport *sdp)
{
	s32 ret;
#ifdef DP501_DEBUG
	printk(KERN_INFO "%s:%s\\n", __FILE__, __func__);
#endif
	ret = i2c_smbus_read_byte_data(sdp->client, TOPCFG4);
	ret = dp501_12c_device_read(sdp, TOPCFG4, "TOPCFG4");
	/* power down encoder, put DP501 in standby mode */
	ret = dp501_12c_device_write(sdp, TOPCFG4, (ret|0x30), "TOPCFG4");

	return ret;
}

static int displayport_power_off(struct displayport *sdp)
{
#ifdef DP501_DEBUG
	printk(KERN_INFO "%s:%s\\n", __FILE__, __func__);
#endif
	if (DP501_PowerDownEncoder(sdp) < 0)
		return -1;

	return 0;
}

/*****************************************/
/*****      I2C driver section       *****/
/*****************************************/

static int __devinit displayport_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
#ifdef DP501_DEBUG
	printk(KERN_INFO "%s:%s\\n", __FILE__, __func__);
#endif
	sdp = kzalloc(sizeof(struct displayport), GFP_KERNEL);
	if (sdp == NULL)
		return -ENOMEM;
	i2c_set_clientdata(client, sdp);
	sdp->client = client;
	return 0;
}

static int __devexit displayport_i2c_remove(struct i2c_client *client)
{
	struct displayport *sdp = i2c_get_clientdata(client);
#ifdef DP501_DEBUG
	printk(KERN_INFO "%s:%s\\n", __FILE__, __func__);
#endif
	kfree(sdp);
	i2c_set_clientdata(client, NULL);

	return 0;
}

#ifdef CONFIG_PM
static int displayport_i2c_suspend(struct i2c_client *client, pm_message_t mesg)
{
#ifdef DP501_DEBUG
	printk(KERN_INFO "%s:%s\\n", __FILE__, __func__);
#endif
	return 0;
}

static int displayport_i2c_resume(struct i2c_client *client)
{
#ifdef DP501_DEBUG
	printk(KERN_INFO "%s:%s\\n", __FILE__, __func__);
#endif
	return 0;
}
#endif

static const struct i2c_device_id displayport_i2c_id[] = {
	{ "DP501_i2c_driver", 0 },
	{ },
};

/*********************************************/
/*****      DSS/DPI driver section       *****/
/*********************************************/

static int DP501_panel_start(struct omap_dss_device *dssdev)
{
	int r = 0;
#ifdef DP501_DEBUG
	printk(KERN_INFO "%s:%s\\n", __FILE__, __func__);
#endif
	if (dssdev->platform_enable) {
		r = dssdev->platform_enable(dssdev);
		if (r) {
			dev_err(&dssdev->dev,
				"failed to enable DSS platform\n");
			return r;
		}
	}
	r = omapdss_dpi_display_enable(dssdev);
	if (r) {
		dev_err(&dssdev->dev, "failed to enable DPI\n");
		return r;
	}
	dp501_init(sdp);
	return 0;
}

static int DP501_panel_enable(struct omap_dss_device *dssdev)
{
#ifdef DP501_DEBUG
	printk(KERN_INFO "%s:%s\\n", __FILE__, __func__);
#endif
	if (dssdev->state != OMAP_DSS_DISPLAY_DISABLED)
		return -EINVAL;

	return DP501_panel_start(dssdev);
}

static void DP501_get_resolution(struct omap_dss_device *dssdev,
		u16 *xres, u16 *yres)
{
#ifdef DP501_DEBUG
	printk(KERN_INFO "%s:%s\\n", __FILE__, __func__);
#endif
	*xres = dssdev->panel.timings.x_res;
	*yres = dssdev->panel.timings.y_res;
}

static int DP501_panel_probe(struct omap_dss_device *dssdev)
{
#ifdef DP501_DEBUG
	printk(KERN_INFO "%s:%s\\n", __FILE__, __func__);
#endif
	dssdev->panel.config &= ~((OMAP_DSS_LCD_IPC) | (OMAP_DSS_LCD_IEO));
	dssdev->panel.config =  (OMAP_DSS_LCD_TFT) | (OMAP_DSS_LCD_ONOFF) |
						(OMAP_DSS_LCD_IHS)  |
						(OMAP_DSS_LCD_IVS) ;
	dssdev->panel.acb = 0x0;
	dssdev->panel.timings = displayport_ls_timings[27];

	return 0;
}

static void DP501_panel_remove(struct omap_dss_device *dssdev)
{
#ifdef DP501_DEBUG
	printk(KERN_INFO "%s:%s\\n", __FILE__, __func__);
#endif
}

static void DP501_panel_stop(struct omap_dss_device *dssdev)
{
#ifdef DP501_DEBUG
	printk(KERN_INFO "%s:%s\\n", __FILE__, __func__);
#endif
	omapdss_dpi_display_disable(dssdev);

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);
}

static void DP501_panel_disable(struct omap_dss_device *dssdev)
{
#ifdef DP501_DEBUG
	printk(KERN_INFO "%s:%s\\n", __FILE__, __func__);
#endif
	displayport_power_off(sdp);
	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
		DP501_panel_stop(dssdev);
}

#ifdef CONFIG_PM
static int DP501_panel_suspend(struct omap_dss_device *dssdev)
{
#ifdef DP501_DEBUG
	printk(KERN_INFO "%s:%s\\n", __FILE__, __func__);
#endif
	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
		return -EINVAL;

	DP501_panel_stop(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;

	return 0;
}

static int DP501_panel_resume(struct omap_dss_device *dssdev)
{
#ifdef DP501_DEBUG
	printk(KERN_INFO "%s:%s\\n", __FILE__, __func__);
#endif
	if (dssdev->state != OMAP_DSS_DISPLAY_SUSPENDED)
		return -EINVAL;

	return DP501_panel_start(dssdev);
}
#endif

static void DP501_set_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
#ifdef DP501_DEBUG
	printk(KERN_INFO "%s:%s\\n", __FILE__, __func__);
#endif
	dpi_set_timings(dssdev, timings);
}

static void DP501_get_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
#ifdef DP501_DEBUG
	printk(KERN_INFO "%s:%s\\n", __FILE__, __func__);
#endif
	*timings = dssdev->panel.timings;
}

static int DP501_check_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
#ifdef DP501_DEBUG
	printk(KERN_INFO "%s:%s\\n", __FILE__, __func__);
#endif
	return dpi_check_timings(dssdev, timings);
}

static int DP501_get_recommended_bpp(struct omap_dss_device *dssdev)
{
#ifdef DP501_DEBUG
	printk(KERN_INFO "%s:%s\\n", __FILE__, __func__);
#endif
	return 24;
}

/******************************************/
/***  Driver configuration I2C and DSS  ***/
/******************************************/

static struct omap_dss_driver DP501_driver = {
	.probe		= DP501_panel_probe,
	.remove		= DP501_panel_remove,
	.enable		= DP501_panel_enable,
	.disable	= DP501_panel_disable,
	.get_resolution	= DP501_get_resolution,
#ifdef CONFIG_PM
	.suspend	= DP501_panel_suspend,
	.resume		= DP501_panel_resume,
#endif
	.get_timings	= DP501_get_timings,
	.set_timings	= DP501_set_timings,
	.check_timings	= DP501_check_timings,
	.get_recommended_bpp = DP501_get_recommended_bpp,

	.driver         = {
		.name   = DRIVER_NAME,
		.owner  = THIS_MODULE,
	},
};

static struct i2c_driver displayport_i2c_driver = {
	.probe          = displayport_i2c_probe,
	.remove         = displayport_i2c_remove,
#ifdef CONFIG_PM
	.suspend        = displayport_i2c_suspend,
	.resume         = displayport_i2c_resume,
#endif
	.id_table       = displayport_i2c_id,
	.driver = {
		.name   = DRIVER_NAME,
		.owner  = THIS_MODULE,
	},
};

static int __init displayport_i2c_init(void)
{
	int r;
#ifdef DP501_DEBUG
	printk(KERN_INFO "%s:%s\\n", __FILE__, __func__);
#endif
	r = i2c_add_driver(&displayport_i2c_driver);
	if (r < 0) {
		printk(KERN_WARNING DRIVER_NAME
		" DisplayPort I2C driver registration failed (i2c1)\n");
		return r;
	}
	r = omap_dss_register_driver(&DP501_driver);
	if (r < 0) {
		printk(KERN_WARNING DRIVER_NAME
		" DisplayPort DSS driver registration failed (DP501)\n");
		return r;
	}
	return 0;
}


static void __exit displayport_i2c_exit(void)
{
#ifdef DP501_DEBUG
	printk(KERN_INFO "%s:%s\\n", __FILE__, __func__);
#endif
	i2c_del_driver(&displayport_i2c_driver);
	omap_dss_unregister_driver(&DP501_driver);
}

module_init(displayport_i2c_init);
module_exit(displayport_i2c_exit);

MODULE_AUTHOR("Salomon Chavez");
MODULE_DESCRIPTION("DisplayPort driver");
MODULE_LICENSE("GPL");
