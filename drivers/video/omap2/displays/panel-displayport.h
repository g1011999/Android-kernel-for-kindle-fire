/*
 * panel-displayport.h
 * DisplayPort driver, definitions
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

#ifndef PANEL_DISPLAYPORT_H
#define PANEL_DISPLAYPORT_H


/********   DP501 Registers   ********/
/*   Page 0   */
/* TOP */
#define TOPCFG0                   0x00
#define ROMI2C_PRESCALE           0x01
#define HDCPI2C_PRESCALE          0x02
#define GPIO                      0x03
#define GPIO_OUT_ENB              0x04
#define TESTI2C_CTL               0x05
#define I2CMTIMEOUT               0x06
#define TOPCFG1                   0x07
#define TOPCFG2                   0x08
#define TOPCFG3                   0x09
#define TOPCFG4                   0x0A
#define CLKSWRST                  0x0B
#define CADETB_CTL                0x0C

/* Video Attribute */
#define HTOTAL_L                  0x10
#define HTOTAL_H                  0x11
#define HSTART_L                  0x12
#define HSTART_H                  0x13
#define HWIDTH_L                  0x14
#define HWIDTH_H                  0x15
#define VTOTAL_L                  0x16
#define VTOTAL_H                  0x17
#define VSTART_L                  0x18
#define VSTART_H                  0x19
#define VHEIGHT_L                 0x1A
#define VHEIGHT_H                 0x1B
#define HSPHSW_L                  0x1C
#define HSPHSW_H                  0x1D
#define VSPVSW_L                  0x1E
#define VSPVSW_H                  0x1F
#define MISC0                     0x20
#define MISC1                     0x21

/* Video Capture */
#define VCAPCTRL0                 0x24
#define VCAPCTRL1                 0x25
#define VCAPCTRL2                 0x26
#define VCAPCTRL3                 0x27
#define VCAPCTRL4                 0x28
#define VCAP_MEASURE              0x29

/* Main Link Control */
#define NVID_L                    0x2C
#define NVID_M                    0x2D
#define NVID_H                    0x2E
#define LINK_CTRL0                0x2F
#define LINK_CTRL1                0x30
#define LINK_DEBUG                0x31
#define ERR_POS                   0x32
#define ERR_PAT                   0x33
#define LINK_DEB_SEL              0x34
#define IDLE_PATTERN              0x35
#define TU_SIZE                   0x36
#define CRC_CTRL                  0x37
#define CRC_OUT                   0x38

/* AVI-2 InfoFrame */
#define SD_CTRL0                  0x3A
#define SD_CTRL1                  0x3B
#define SD_HB0                    0x3C
#define SD_HB1                    0x3D
#define SD_HB2                    0x3E
#define SD_HB3                    0x3F
#define SD_DB0                    0x40
#define SD_DB1                    0x41
#define SD_DB2                    0x42
#define SD_DB3                    0x43
#define SD_DB4                    0x44
#define SD_DB5                    0x45
#define SD_DB6                    0x46
#define SD_DB7                    0x47
#define SD_DB8                    0x48
#define SD_DB9                    0x49
#define SD_DB10                   0x4A
#define SD_DB11                   0x4B
#define SD_DB12                   0x4C
#define SD_DB13                   0x4D
#define SD_DB14                   0x4E
#define SD_DB15                   0x4F

/* Aux Channel and PCS */
#define DPCD_REV                  0X50
#define MAX_LINK_RATE             0x51
#define MAX_LANE_COUNT            0x52
#define MAX_DOWNSPREAD            0x53
#define NORP                      0x54
#define DOWNSTRMPORT_PRE          0x55
#define MLINK_CH_CODING           0x56
#define RCV_P0_CAP0               0x58
#define RCV_P0_CAP1               0x59
#define RCV_P1_CAP0               0x5A
#define RCV_P1_CAP1               0x5B
#define DOWNSPREAD_CTL            0x5C
#define LINK_BW                   0x5D
#define LANE_CNT                  0x5E
#define TRAINING_CTL              0x5F
#define QUALTEST_CTL              0x60
#define SINK_COUNT                0x61
#define DEV_SERVICE_IRQ           0x62
#define LANE01_STATUS             0x63
#define LANE23_STATUS             0x64
#define LANE_STATUS_UPDATE        0x65
#define SINK_STATUS               0x66
#define AUX_NOISE                 0x67
#define TEST_MODE                 0x69
#define TEST_PATTERN0             0x6A
#define TEST_PATTERN1             0x6B
#define TEST_PATTERN2             0x6C
#define SIGNATURE                 0x6D
#define PCSCFG                    0x6E
#define AUXCTRL0                  0x6f
#define AUXCTRL2                  0x70
#define AUXCTRL1                  0x71
#define HPDCTL0                   0x72
#define HPDCTL1                   0x73
#define LINK_STATE_CTRL           0x74
#define SWRST                     0x75
#define LINK_IRQ                  0x76
#define AUXIRQ_CTRL               0x77
#define HPD2_IRQ_CTRL             0x78
#define SW_TRAIN_CTRL             0x79
#define SW_DRV_SET                0x7A
#define SW_PRE_SET                0x7B
#define DPCD_ADDR_L               0x7D
#define DPCD_ADDR_M               0x7E
#define DPCD_ADDR_H               0x7F
#define DPCD_LENGTH               0x80
#define DPCD_WDATA                0x81
#define DPCD_RDATA                0x82
#define DPCD_CTL                  0x83
#define DPCD_STATUS               0x84
#define AUX_STATUS                0x85
#define I2CTOAUX_RELENGTH         0x86
#define AUX_RETRY_CTRL            0x87
#define TIMEOUT_CTRL              0x88
#define I2CCMD_OPT1               0x89
#define AUXCMD_ERR_IRQ            0x8A
#define AUXCMD_OPT2               0x8B
#define HDCP_Reserved             0x8C

/* Audio InfoFrame */
#define TX_MVID0                  0x90
#define TX_MVID1                  0x91
#define TX_MVID2                  0x92
#define TX_MVID_OFF               0x93
#define TX_MAUD0                  0x94
#define TX_MAUD1                  0x95
#define TX_MAUD2                  0x96
#define TX_MAUD_OFF               0x97
#define MN_CTRL                   0x98
#define MOUT0                     0x99
#define MOUT1                     0x9A
#define MOUT2                     0x9B

/* Audio Control */
#define NAUD_L                    0x9F
#define NAUD_M                    0xA0
#define NAUD_H                    0xA1
#define AUD_CTRL0                 0xA2
#define AUD_CTRL1                 0xA3
#define LANE_POL                  0xAA
#define LANE_EN                   0xAB
#define LANE_MAP                  0xAC
#define SCR_POLY0                 0xAD
#define SCR_POLY1                 0xAE
#define PRBS7_POLY                0xAF

/* Video Pre-process */
#define MISC_SHDOW                0xB0
#define VCAPCPCTL0                0xB1
#define VCAPCPCTL1                0xB2
#define VCAPCPCTL2                0xB3
#define CSCPAR                    0xB4
#define I2CTODPCDSTATUS2          0xBA
#define AUXCTL__REG               0xBB

/*   Page 2   */

#define SEL_PIO1                  0x24
#define SEL_PIO2                  0x25
#define SEL_PIO3                  0x26
#define CHIP_VER_L                0x82

/********   END DP501 Registers   ********/

/* General defines */
#define DP                        0
#define DVI                       1

#define ACK                       000b
#define NACK                      001b
#define DEFER                     010b
#define INVALID_REPLAY            100b
#define TIMEOUT                   111b

#ifndef TRUE
#define TRUE (1 == 1)
#endif
#ifndef FALSE
#define FALSE (!TRUE)
#endif

#define SINK_CONNECTED           1
#define SINK_NOT_CONNECTED       0

#define DSI_DIV2            0x40C
#define DSI_DIV_LCD         16
#define DSI_DIV_PCD         0
#define DSI_CONTROL2        0x238
#define DSS_CTRL            0x40

#define CR_FAILT           -1
#define EQ_Fail            -1

#define REQUIRED_LINK_RATE    0x0A /* 2.7Gbps per lane */
#define REQUIRED_LANE_COUNT   4   /* it only must be 1,2 or 4 */


struct omap_video_timings displayport_ls_timings[] = {
	/* x_res; y_res; pixel_clock; hsw; hfp; hbp; vsw; vfp; vbp; */
	{640, 480, 25200, 96, 16, 48, 2, 10, 33},// 0
	{1280, 720, 74250, 40, 440, 220, 5, 5, 20},// 1
	{1280, 720, 74250, 40, 110, 220, 5, 5, 20},// 2
	{720, 480, 27027, 62, 16, 60, 6, 9, 30},// 3
	{2880, 576, 108000, 256, 48, 272, 5, 5, 39},// 4
	{1440, 240, 27027, 124, 38, 114, 3, 4, 15},// 5
	{1440, 288, 27000, 126, 24, 138, 3, 2, 19},// 6
	{1920, 540, 74250, 44, 528, 148, 5, 2, 15},// 7
	{1920, 540, 74250, 44, 88, 148, 5, 2, 15},// 8
	{1920, 1080, 148500, 44, 88, 148, 5, 4, 36},// 9
	{720, 576, 27000, 64, 12, 68, 5, 5, 39},// 10
	{1440, 576, 54000, 128, 24, 136, 5, 5, 39},// 11
	{1920, 1080, 148500, 44, 528, 148, 5, 4, 36},// 12
	{2880, 480, 108108, 248, 64, 240, 6, 9, 30},// 13
	{1920, 1080, 74250, 44, 638, 148, 5, 4, 36},// 14
	/* Vesa frome here */
	{640, 480, 25175, 96, 16, 48, 2 , 11, 31},// 15
	{800, 600, 40000, 128, 40, 88, 4 , 1, 23},// 16
	{848, 480, 33750, 112, 16, 112, 8 , 6, 23},// 17
	{1280, 768, 79500, 128, 64, 192, 7 , 3, 20},// 18
	{1280, 800, 83500, 128, 72, 200, 6 , 3, 22},// 19
	{1360, 768, 85500, 112, 64, 256, 6 , 3, 18},// 20
	{1280, 960, 108000, 112, 96, 312, 3 , 1, 36},// 21
	{1280, 1024, 108000, 112, 48, 248, 3 , 1, 38},// 22
	{1024, 768, 65000, 136, 24, 160, 6, 3, 29},// 23
	{1400, 1050, 121750, 144, 88, 232, 4, 3, 32},// 24
	{1440, 900, 106500, 152, 80, 232, 6, 3, 25},// 25
	{1680, 1050, 146250, 176 , 104, 280, 6, 3, 30},// 26
	{1366, 768, 85500, 143, 70, 213, 3, 3, 24},// 27
	{1920, 1080, 148500, 44, 88, 80, 5, 4, 36},// 28
	{1280, 768, 68250, 32, 48, 80, 7, 3, 12},// 29
	{1400, 1050, 101000, 32, 48, 80, 4, 3, 23},// 30
	{1680, 1050, 119000, 32, 48, 80, 6, 3, 21},// 31
	{1280, 800, 79500, 32, 48, 80, 6, 3, 14},// 32
	{1280, 720, 74250, 40, 110, 220, 5, 5, 20},// 33
	/* supported 3d timings UNDEROVER full frame */
	{1280, 1470, 148350, 40, 110, 220, 5, 5, 20},// 34
	{1280, 1470, 148500, 40, 110, 220, 5, 5, 20},// 35
	{1280, 1470, 148500, 40, 440, 220, 5, 5, 20}// 36
};

struct displayport {
	struct i2c_client    *client;
}*sdp;

#endif
