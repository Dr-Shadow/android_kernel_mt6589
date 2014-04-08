/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/

#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
	#include <platform/mt_pmic.h>
#define UBOOT_LK
#define DMS_LCD(a,arg...) printf("[lk LCD] "a,##arg)	
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
    #include <asm/arch/mtk_pmic_6320.h>
#define UBOOT_LK
#define DMS_LCD(a,arg...) printf("[uboot LCD] "a,##arg)
#else
#define DMS_LCD(a,arg...) printk("[kernel LCD] "a,##arg)
	#include <mach/mt_gpio.h>
	#include <mach/pmic_mt6320_sw.h>
#endif

#ifndef BUILD_LK
#include <linux/oppo_devices_list.h>
#include <mach/mt_boot.h>
extern LCD_DEV lcd_dev;
//int lcd_dev;
//int LCD_JDI = 0;
#endif

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (720)
#define FRAME_HEIGHT (1280)

//HW_WIDTH= sqrt((Inc*25.4*Incs*25.4)/(1+(FRAME_HEIGHT/FRAME_WIDTH)*(FRAME_HEIGHT/FRAME_WIDTH)))
//Incs is value of LCD size of inches.For example Incs = 4.0 for 12021 12061 12039
#define HW_WIDTH 58
//HW_HEIGHT = (FRAME_HEIGHT/FRAME_WIDTH)*HW_WIDTH
#define HW_HEIGHT 103




#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0xdd   // END OF REGISTERS MARKER
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)   


#define   LCM_DSI_CMD_MODE							0


struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};
static struct LCM_setting_table lcm_jdi_initialization_setting[] = {
	
	/*
	Note :

	Data ID will depends on the following rule.
	
		count of parameters > 1	=> Data ID = 0x39
		count of parameters = 1	=> Data ID = 0x15
		count of parameters = 0	=> Data ID = 0x05

	Structure Format :

	{DCS command, count of parameters, {parameter list}}
	{REGFLAG_DELAY, milliseconds of time, {}},

	...

	Setting ending by predefined flag
	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
	*/

	{0xB9,	3,	{ 0xFF, 0x83, 0x94}},    //TE ON 
{0x35, 1, {0x00}},//TE ON 
{0xBA, 1, {0x13}}, 
{0xB1, 15, {0x01,0x00,0x04,0x87,0x01,0x11,0x11,0x35,0x3D,0x29,0x29,0x47,0x02,0x01,0xE6}},//0x29, 0x29,
{0xB4, 22, {0x80,0x08,0x32,0x10,0x00,0x32,0x15,0x08,0x32,0x10,0x08,0x33,0x04,0x4A,0x04,0x37,0x04,0x4F,0x06,0x61,0x61,0x06}}, 
{0xB2, 1, {0x00}},
{0xD5, 54, {0x00,0x00,0x00,0x00,0x0A,0x00,0x01,0x00,0xCC,0x00,0x00,0x00,0x88,0x88,
0x88,0x88,0x99,0x88,0x88,0x88,0xAA,0xBB,0x23,0x01,0x67,0x45,0x01,0x23,0x88,0x88,0x88,0x88,0x88,0x88,0x88,
0x88,0x88,0x99,0x88,0x88,0x88,0x88,0x54,0x76,0x10,0x32,0x32,0x10,0x88,0x88,0x88,0x88,0x3C,0x01}}, 
{0xE0, 34, {0x01,0x05,0x07,0x25,0x35,0x3F,0x0B,0x32,0x04,0x09,0x0E,0x10,0x13,0x10,0x14,0x16,0x1B,0x01,0x05,0x07,0x25,0x35,0x3F,0x0B,0x32,0x04,0x09,0x0E,0x10,0x13,0x10,0x14,0x16,0x1B}},
{0xC6, 4, {0x04,0x00,0x80}},
{0xC7, 4, {0x00, 0x10, 0x00, 0x10}},
 {0xCC, 1, {0x09}}, 
{0xBC, 1, {0x07}},   
{0xBF, 4, {0x06,0x00,0x10,0x04}},
{0xB6, 1, {0x00}},
{0x11, 1, {0x00}},
{REGFLAG_DELAY, 150, {}},
{0x29, 1, {0x00}},
{REGFLAG_DELAY, 50, {}},
{REGFLAG_END_OF_TABLE, 0x00, {}}
};
#if 0
{
	{0xB9,	3,	{ 0xFF, 0x83, 0x94}},    //TE ON 
	//{0x44,	2,	{0x02,0x12}},
	/*{0xff,	3,	{0x12,0x82,0x01}},
	{0x00,	1,	{0x80}},
	{0xff,	2,	{0x12,0x82}},
	{0x00,	1,	{0x80}},
	//{0xc4,	4,	{0x0c,0x08,0x02,0x02}},
	{0xc4,	4,	{0x0c,0x09,0x02,0x02}},*/
	{0xC7,	4,	{0x00, 0x10, 0x00, 0x10}},
	{0xBC,	1,	{0x07}},    //SLEEP OUT
	{0xBA,	1,	{0x13}}, 
	{0xB1,	15,	{0x01, 0x00, 0x07, 0x81, 0x01, 0x0E, 0x0E, 0x32, 0x38, 0x22, 0x22, 0x50, 0x02, 0x00, 0x00}},//0x29, 0x29,
	{0xB2,	6,	{0x00, 0xC8, 0x09, 0x05, 0x00, 0x71}},
	{0xCC,	1,	{0x09}},
	{0x00,	0,	{0x00}},
	{0xD5,	52,	{0x00,0x00,0x00,0x00,0x0A,0x00,0x01,0x00,0x00,0x00,0x33,0x00,0x23,0x45,0x67,
				0x01,0x01,0x23,0x88,0x88,0x88,0x88,0x88,0x88,0x88,
				0x99,0x99,0x99,0x88,0x88,0x99,0x88,0x54,0x32,0x10,0x76,0x32,0x10,
				0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x99,0x99,0x99,0x88,0x88,0x88,0x99}},
	{0xB4,	22,	{ 0x80, 0x08, 0x32, 0x10, 0x00, 0x32, 0x15, 0x08, 0x32, 0x12, 0x20, 
					0x33, 0x05, 0x4C, 0x05, 0x37, 0x05, 0x3F, 0x1E, 0x5F, 0x5F, 0x06}},
	{0xB6,	1,	{0x00}},
	{0xE0,	34,	{0x06, 0x0a, 0x0c, 0x25, 0x35, 0x3F, 0x1f, 0x46, 0x04, 0x09, 0x0E, 0x10, 0x13, 0x10, 0x14, 0x16, 0x1B, 0x06, 0x0a, 
				0x0c, 0x25, 0x35, 0x3F, 0x1f, 0x46, 0x04, 0x09, 0x0E, 0x10, 0x13, 0x10, 0x14, 0x16, 0x1B}},
	//{0xE0,	34,	{0x01, 0x05, 0x07, 0x25, 0x35, 0x3F, 0x0B, 0x32, 0x04, 0x09, 0x0E, 0x10, 0x13, 0x10, 0x14, 0x16, 0x1B, 0x01, 0x05, 
				//0x07, 0x25, 0x35, 0x3F, 0x0B, 0x32, 0x04, 0x09, 0x0E, 0x10, 0x13, 0x10, 0x14, 0x16, 0x1B}},
				
				
	/*{0xC1,127,{0x01, 
				0x00,0x1C ,0x2A ,0x35 ,0x40 ,0x4A ,0x53 ,0x5B ,0x60 ,0x68 ,0x70 ,0x76 ,0x7C ,0x80 ,0x86 ,0x8C ,0x92 ,0x97 ,0x9D ,0xA2 ,
				0xA8 ,0xB0 ,0xB7 ,0xBE ,0xC1 ,0xC8 ,0xD0 ,0xD6 ,0xDA ,0xE0 ,0xE6 ,0xEC ,0x00 ,0x39 ,0x37 ,0xE0 ,0xC7 ,0x67 ,0xC8 ,0xC1 ,0x58 ,0xC0,
				0x00 ,0x1C ,0x2A ,0x35 ,0x40 ,0x4A ,0x53 ,0x5B ,0x60 ,0x68 ,0x70 ,0x76 ,0x7C ,0x80 ,0x86 ,0x8C ,0x92 ,0x97 ,0x9D ,0xA2 ,
				0xA8 ,0xB0 ,0xB7 ,0xBE ,0xC1 ,0xC8 ,0xD0 ,0xD6 ,0xDA ,0xE0 ,0xE6 ,0xEC ,0x00 ,0x39 ,0x37 ,0xE0 ,0xC7 ,0x67 ,0xC8 ,0xC1 ,0x58 ,0xC0, 
				0x00 ,0x1C ,0x2A ,0x35 ,0x40 ,0x4A ,0x53 ,0x5B ,0x60 ,0x68 ,0x70 ,0x76 ,0x7C ,0x80 ,0x86 ,0x8C ,0x92 ,0x97 ,0x9D ,0xA2 ,
				0xA8 ,0xB0 ,0xB7 ,0xBE ,0xC1 ,0xC8 ,0xD0 ,0xD6 ,0xDA ,0xE0 ,0xE6 ,0xEC ,0x00 ,0x39 ,0x37 ,0xE0 ,0xC7 ,0x67 ,0xC8 ,0xC1 ,0x58 ,0xC0}},	*/	
				
	{0xBF,	3,	{0x06,0x00,0x10}},
	{0xE6,1,{0x01}},
    {0xE4,1,{0x03}},//CE ON
	{0x11,	1,	{0x00}},
	{REGFLAG_DELAY, 200, {}},
	{0x29,	1,	{0x00}},
	{REGFLAG_DELAY, 50, {}},
	// Note
	// Strongly recommend not to set Sleep out / Display On here. That will cause messed frame to be shown as later the backlight is on.


	// Setting ending by predefined flag
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
#endif

static struct LCM_setting_table lcm_jdi2_initialization_setting[] = {
	{0x00,	1,	{0x00}},
	{0xff,	3,	{0x12,0x82,0x01}},
	{0x00,	1,	{0x80}},
	{0xff,	2,	{0x12,0x82}},
	{0x00,	1,	{0x80}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


 
void lcd_power_on(unsigned char on  )
{
    if(on)
    {
       pmic_config_interface(0x045A, 0x5, 0x7, 5);//SET VGP6 = 3.3V
       pmic_config_interface(0x0434,1,0x1,15);
       MDELAY(5);
       pmic_config_interface(0x0456, 0x3, 0x7, 5);//SET VGP4 = 1.8V
       pmic_config_interface(0x0430,1,0x1,15);
       MDELAY(1);
	   mt_set_gpio_mode(140, 0);
       mt_set_gpio_dir(140, 1);
	   mt_set_gpio_out(140, 1);  
       mt_set_gpio_out(29, 1);
       MDELAY(20);
       SET_RESET_PIN(1);
       MDELAY(10);
    }
    else
    {
	   SET_RESET_PIN(0);
	   MDELAY(10);
	   mt_set_gpio_out(29, 0);
	   MDELAY(1);
	   pmic_config_interface(0x0430,0,0x1,15);
	   MDELAY(1);
	   pmic_config_interface(0x0434,0,0x1,15);
    }

}


static struct LCM_setting_table lcm_vdo_sleep_mode_in_setting[] = {
	// Display off sequence
    // Sleep Mode On
	{0x10, 0, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
	
};


void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {
		
        unsigned cmd;
        cmd = table[i].cmd;
		
        switch (cmd) {
			
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
				
            case REGFLAG_END_OF_TABLE :
                break;
				
            default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
       	}
    }
	
}

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{

		memset(params, 0, sizeof(LCM_PARAMS));
	
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;
	   // params->hw_width = HW_WIDTH;
	   // params->hw_height= HW_HEIGHT;

		// enable tearing-free
		//params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
		//params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

        #if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
        #else
		params->dsi.mode   = SYNC_PULSE_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE; 
        #endif
	
		// DSI
		/* Command mode setting */
		//1 Three lane or Four lane
		params->dsi.LANE_NUM				= LCM_FOUR_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Highly depends on LCD driver capability.
		// Highly depends on LCD driver capability.
		// Not support in MT6573
		params->dsi.packet_size=256;
		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		params->dsi.vertical_active_line				= FRAME_HEIGHT;
		params->dsi.intermediat_buffer_num = 2;

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		params->dsi.word_count=720*3;
		params->dsi.vertical_sync_active				= 2;
		params->dsi.vertical_backporch					= 10;
		params->dsi.vertical_frontporch					= 7;
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 
		params->dsi.horizontal_sync_active				= 60;
		params->dsi.horizontal_backporch				= 59;
		params->dsi.horizontal_frontporch				= 79;	
		params->dsi.horizontal_active_pixel			= FRAME_WIDTH;
		
		// Video mode setting		
		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
//		params->dsi.pll_select=1;	//0: MIPI_PLL; 1: LVDS_PLL
		params->dsi.PLL_CLOCK = LCM_DSI_6589_PLL_CLOCK_208;//this value must be in MTK suggested table
											//if not config this para, must config other 7 or 3 paras to gen. PLL

}

#ifdef VENDOR_EDIT
volatile int oppo_lcd_status = 1;
#endif
extern int gbtecnt;
int istesuspend=0;
static void lcm_init(void)
{
	#ifdef UBOOT_LK	
     mt_set_gpio_mode(29, 0);
     mt_set_gpio_dir(29, 1);
     mt_set_gpio_out(29, 0) ;
     SET_RESET_PIN(0);
     MDELAY(10);

     lcd_power_on(1);
	 
	 push_table(lcm_jdi_initialization_setting, sizeof(lcm_jdi_initialization_setting) / sizeof(struct LCM_setting_table), 1);
	 #endif
    #ifdef VENDOR_EDIT
	oppo_lcd_status = 1;
	#endif
}



static void lcm_suspend(void)
{
   //#ifdef UBOOT_LK
	//yongjun.wu@Pro.drv modify for clean vedio mode afterimage 
	 push_table(lcm_vdo_sleep_mode_in_setting, sizeof(lcm_vdo_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
	    SET_RESET_PIN(0);
     MDELAY(10);
	 mt_set_gpio_out(29, 0);//power off
	    MDELAY(10);
	//#endif
	#ifndef UBOOT_LK
	DMS_LCD("===================>lcm_suspend\n");
	//lcd_power_on(0);
   if(!istesuspend)
    {
          istesuspend=1;
    }
	#if 0 //rendong.shi@Prodrv,add for factory mode camera isp current bug
	 if(get_boot_mode() ==  FACTORY_BOOT )
	 {
	    mt_set_gpio_mode(GPIOEXT22, 0);
        mt_set_gpio_dir(GPIOEXT22,1);
        mt_set_gpio_out(GPIOEXT22,0) ;
        mt_set_gpio_mode(134, 0);
        mt_set_gpio_dir(134,1);
        mt_set_gpio_out(134,0) ;
		mt_set_gpio_mode(79, 0);
        mt_set_gpio_dir(79,1);
        mt_set_gpio_out(79,0) ;
	 }
	#endif
	#endif
}


static void lcm_resume(void)
{
   DMS_LCD("lcm_resume start: \n"); 
	#ifndef UBOOT_LK	
	lcd_dev = LCD_JDI;
	#endif
     mt_set_gpio_out(29, 1);
    MDELAY(20);
    SET_RESET_PIN(0);
    MDELAY(5);
    SET_RESET_PIN(1);
    MDELAY(10);
	
	push_table(lcm_jdi_initialization_setting, sizeof(lcm_jdi_initialization_setting) / sizeof(struct LCM_setting_table), 1);
    DMS_LCD("lcm_resume end: \n");    	
}
         
#if (LCM_DSI_CMD_MODE)
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	
	data_array[0]= 0x00053902;
	data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[2]= (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);

}
#endif



static unsigned int lcm_esd_check(void)
{
   #ifndef BUILD_LK
   static unsigned char gbEsdCheckEn=0;
   static unsigned char bTeNumCnt=0;
   DMS_LCD("=============>lcm_esd_check,gbtecnt=%d,bTeNumCnt=%d\n",gbtecnt,bTeNumCnt);
   DMS_LCD("gbEsdCheckEn=%d",gbEsdCheckEn);
   if(bTeNumCnt!=5)
   {
   
        if(gbtecnt>100)
        {
             gbtecnt=0;
             bTeNumCnt++;
             if(bTeNumCnt==5)
             {
                 DMS_LCD("=====>bTeNumCnt==5");
                 gbtecnt=60;
                 gbEsdCheckEn=1;
             }
             else
             {
                 return 0;
             }
        }
        else
        {
            gbtecnt=0;
    DMS_LCD("=============>lcm_esd_check,gbtecnt2=%d\n",gbtecnt);
            return 0;
        }
   }
   
   if(!gbEsdCheckEn)
   {
       return 0;
   }
   if(istesuspend)
   {
        return 0;
   }
   if(gbtecnt<50)
   {
        printk(KERN_ERR"=====================>lcm_esd_recovery_gbtecnt=%d\n",gbtecnt);
        gbtecnt=0;
	   return 1;

   }
   printk(KERN_ERR"=====================>lcm_esd_check_gbtecnt=%d\n",gbtecnt);
   gbtecnt=0;
   #endif
    return 0;
}


static int lcm_esd_recover(void)
{
   	#ifndef UBOOT_LK	
	SET_RESET_PIN(0);
	MDELAY(1); // 1ms
	//mt_set_gpio_out(76, 1);
	mt_set_gpio_mode(29, 0);
    mt_set_gpio_dir(29, 1);
    mt_set_gpio_out(29, 1);
	MDELAY(5);
	SET_RESET_PIN(1);
	MDELAY(10); 
	//lcd_power_on(1);      
  	DMS_LCD("===================>lcm_esd recover\n");
 	push_table(lcm_jdi_initialization_setting, sizeof(lcm_jdi_initialization_setting) / sizeof(struct LCM_setting_table), 1);
    #endif
   return 0;
}




#if 0
static unsigned int lcm_esd_check(void)
{
  #ifndef BUILD_LK
	char  buffer[3];
	int   array[4];

	if(lcm_esd_test)
	{
		lcm_esd_test = FALSE;
		return TRUE;
	}

	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0x36, buffer, 1);
	if(buffer[0]==0x90)
	{
		return FALSE;
	}
	else
	{			 
		return TRUE;
	}
 #endif

}

static unsigned int lcm_esd_recover(void)
{
	lcm_init();
	lcm_resume();

	return TRUE;
}
#endif


LCM_DRIVER qhd_hx8394_13003_lcm_drv = 
{
    .name			= "qhd_hx8394_13003_lcm_drv",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
//	.compare_id     = lcm_compare_id,
//	.esd_check = lcm_esd_check,
//	.esd_recover = lcm_esd_recover,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
    };
