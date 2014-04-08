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
static int lcm_paragrams =1;
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


#define   LCM_DSI_CMD_MODE							1


static struct LCM_setting_table {
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

	{0x35,	1,	{0x00}},    //TE ON 
	{0x44,	2,	{0x02,0x80}},
	{0x11,	1,	{0x00}},    //SLEEP OUT
	{REGFLAG_DELAY, 120, {}},
	{0x29,	1,	{0x00}},
	{REGFLAG_DELAY, 120, {}},
	// Note
	// Strongly recommend not to set Sleep out / Display On here. That will cause messed frame to be shown as later the backlight is on.


	// Setting ending by predefined flag
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_jdi_rgb_standard_initialization_setting[] = {
	{0xff,	3,	{0x12,0x82,0x01}},
	{0x00,	1,	{0x80}},
	{0xff,	2,	{0x12,0x82}},
	{0x00,	1,	{0x00}},
	{0xE1,	24,	{0x46,0x49,0x4b,0x4f,0x53,0x56,0x5d,0x68,0x63,
				 0x7a,0x83,0x8a,0x70,0x6c,0x67,0x5b,0x48,0x34,
				 0x27,0x1f,0x17,0x0d,0x08,0x03}},
	{0x00,	1,	{0x00}},
	{0xE2,	24,	{0x46,0x49,0x4b,0x4f,0x53,0x56,0x5d,0x68,0x6e,
				 0x7a,0x83,0x8a,0x70,0x6c,0x67,0x5b,0x48,0x34,
				 0x27,0x1f,0x17,0x0d,0x08,0x03}},
	{0x00,	1,	{0x00}},
	{0xE3,	24,	{0x4b,0x4d,0x4f,0x53,0x57,0x5a,0x60,0x6b,0x71,
				 0x7c,0x85,0x8b,0x70,0x6c,0x67,0x5c,0x4b,0x37,
				 0x2a,0x21,0x18,0x0e,0x09,0x03}},
	{0x00,	1,	{0x00}},
	{0xE4,	24,	{0x4b,0x4d,0x4f,0x53,0x57,0x5a,0x60,0x6b,0x71,
				 0x7c,0x85,0x8b,0x70,0x6c,0x67,0x5c,0x4b,0x37,
				 0x2a,0x21,0x18,0x0e,0x09,0x03}},
	{0x00,	1,	{0x00}},			 
	{0xE5,	24,	{0x00,0x09,0x11,0x1f,0x29,0x32,0x3e,0x50,0x5d,
				 0x6e,0x7b,0x84,0x75,0x70,0x6b,0x5f,0x4e,0x3a,
				 0x2c,0x23,0x1a,0x0f,0x09,0x03}},			 
	{0x00,	1,	{0x00}},			 
	{0xE6,	24,	{0x00,0x09,0x11,0x1f,0x29,0x32,0x3e,0x50,0x5d,
				 0x6e,0x7b,0x84,0x75,0x70,0x6b,0x5f,0x4e,0x3a,
				 0x2c,0x23,0x1a,0x0f,0x09,0x03}},			 
	{0x00,	1,	{0x80}},	
	{0xff,	2,	{0x00,0x00}},
	{0xff,	3,	{0x00,0x00,0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_jdi_rgb_cold_initialization_setting[] = {
	{0xff,	3,	{0x12,0x82,0x01}},
	{0x00,	1,	{0x80}},
	{0xff,	2,	{0x12,0x82}},
	{0x00,	1,	{0x00}},
	{0xE1,	24,	{0x53,0x55,0x57,0x5a,0x5d,0x60,0x66,0x70,0x74,
				 0x7f,0x86,0x8c,0x6f,0x6b,0x66,0x5a,0x47,0x33,
				 0x26,0x1e,0x16,0x0d,0x08,0x03}},
	{0x00,	1,	{0x00}},
	{0xE2,	24,	{0x53,0x55,0x57,0x5a,0x5d,0x60,0x66,0x70,0x74,
				 0x7f,0x86,0x8c,0x6f,0x6b,0x66,0x5a,0x47,0x33,
				 0x26,0x1e,0x16,0x0d,0x08,0x03}},
	{0x00,	1,	{0x00}},
	{0xE3,	24,	{0x57,0x59,0x5a,0x5d,0x60,0x63,0x69,0x72,0x76,
				 0x81,0x88,0x8d,0x6e,0x6b,0x66,0x5b,0x49,0x35,
				 0x28,0x20,0x17,0x0e,0x08,0x03}},
	{0x00,	1,	{0x00}},
	{0xE4,	24,	{0x57,0x59,0x5a,0x5d,0x60,0x63,0x69,0x72,0x76,
				 0x81,0x88,0x8d,0x6e,0x6b,0x66,0x5b,0x49,0x35,
				 0x28,0x20,0x17,0x0e,0x08,0x03}},
	{0x00,	1,	{0x00}},			 
	{0xE5,	24,	{0x00,0x09,0x11,0x1f,0x29,0x32,0x3e,0x50,0x5d,
				 0x6e,0x7b,0x84,0x75,0x70,0x6b,0x5f,0x4e,0x3a,
				 0x2c,0x23,0x1a,0x0f,0x09,0x03}},			 
	{0x00,	1,	{0x00}},			 
	{0xE6,	24,	{0x00,0x09,0x11,0x1f,0x29,0x32,0x3e,0x50,0x5d,
				 0x6e,0x7b,0x84,0x75,0x70,0x6b,0x5f,0x4e,0x3a,
				 0x2c,0x23,0x1a,0x0f,0x09,0x03}},			 
	{0x00,	1,	{0x80}},	
	{0xff,	2,	{0x00,0x00}},
	{0xff,	3,	{0x00,0x00,0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_jdi_rgb3_initialization_setting[] = {
	{0xff,	3,	{0x12,0x82,0x01}},
	{0x00,	1,	{0x80}},
	{0xff,	2,	{0x12,0x82}},
	{0x00,	1,	{0x00}},
	{0xE1,	24,	{0x35,0x38,0x3b,0x40,0x45,0x4a,0x52,0x5f,0x67,
				 0x76,0x80,0x87,0x72,0x6e,0x68,0x5c,0x48,0x33,
				 0x26,0x1e,0x16,0x0d,0x08,0x03}},
	{0x00,	1,	{0x00}},
	{0xE2,	24,	{0x35,0x38,0x3b,0x40,0x45,0x4a,0x52,0x5f,0x67,
				 0x76,0x80,0x87,0x72,0x6e,0x68,0x5c,0x48,0x33,
				 0x26,0x1e,0x16,0x0d,0x08,0x03}},
	{0x00,	1,	{0x00}},
	{0xE3,	24,	{0x3b,0x3e,0x40,0x46,0x4b,0x4f,0x57,0x63,0x6b,
				 0x79,0x82,0x89,0x71,0x6d,0x68,0x5d,0x4b,0x37,
				 0x29,0x21,0x18,0x0e,0x09,0x03}},
	{0x00,	1,	{0x00}},
	{0xE4,	24,	{0x3b,0x3e,0x40,0x46,0x4b,0x4f,0x57,0x63,0x6b,
				 0x79,0x82,0x89,0x71,0x6d,0x68,0x5d,0x4b,0x37,
				 0x29,0x21,0x18,0x0e,0x09,0x03}},
	{0x00,	1,	{0x00}},			 
	{0xE5,	24,	{0x2d,0x30,0x34,0x3a,0x3f,0x44,0x4d,0x5c,0x64,
				 0x74,0x7e,0x86,0x73,0x6f,0x6a,0x5f,0x4e,0x3b,
				 0x2c,0x24,0x1a,0x10,0x0a,0x03}},			 			 
	{0x00,	1,	{0x00}},			 
	{0xE6,	24,	{0x2d,0x30,0x34,0x3a,0x3f,0x44,0x4d,0x5c,0x64,
				 0x74,0x7e,0x86,0x73,0x6f,0x6a,0x5f,0x4e,0x3b,
				 0x2c,0x24,0x1a,0x10,0x0a,0x03}},			 
	{0x00,	1,	{0x80}},	
	{0xff,	2,	{0x00,0x00}},
	{0xff,	3,	{0x00,0x00,0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_jdi_rgb_warm_initialization_setting[] = {
	{0xff,	3,	{0x12,0x82,0x01}},
	{0x00,	1,	{0x80}},
	{0xff,	2,	{0x12,0x82}},
	{0x00,	1,	{0x00}},
	{0xE1,	24,	{0x35,0x38,0x3b,0x40,0x45,0x4a,0x52,0x5f,0x67,
				 0x76,0x80,0x87,0x72,0x6e,0x68,0x5c,0x48,0x33,
				 0x26,0x1e,0x16,0x0d,0x08,0x03}},
	{0x00,	1,	{0x00}},
	{0xE2,	24,	{0x35,0x38,0x3b,0x40,0x45,0x4a,0x52,0x5f,0x67,
				 0x76,0x80,0x87,0x72,0x6e,0x68,0x5c,0x48,0x33,
				 0x26,0x1e,0x16,0x0d,0x08,0x03}},
	{0x00,	1,	{0x00}},
	{0xE3,	24,	{0x3b,0x3e,0x40,0x46,0x4b,0x4f,0x57,0x63,0x6b,
				 0x79,0x82,0x89,0x71,0x6d,0x68,0x5d,0x4b,0x37,
				 0x29,0x21,0x18,0x0e,0x09,0x03}},
	{0x00,	1,	{0x00}},
	{0xE4,	24,	{0x3b,0x3e,0x40,0x46,0x4b,0x4f,0x57,0x63,0x6b,
				 0x79,0x82,0x89,0x71,0x6d,0x68,0x5d,0x4b,0x37,
				 0x29,0x21,0x18,0x0e,0x09,0x03}},
	{0x00,	1,	{0x00}},			 
	{0xE5,	24,	{0x42,0x45,0x47,0x4c,0x50,0x53,0x5a,0x65,0x6c,
				 0x7a,0x82,0x89,0x71,0x6d,0x68,0x5d,0x4d,0x39,
				 0x2b,0x23,0x19,0x0f,0x09,0x03}},			 			 
	{0x00,	1,	{0x00}},			 
	{0xE6,	24,	{0x42,0x45,0x47,0x4c,0x50,0x53,0x5a,0x65,0x6c,
				 0x7a,0x82,0x89,0x71,0x6d,0x68,0x5d,0x4d,0x39,
				 0x2b,0x23,0x19,0x0f,0x09,0x03}},			 
	{0x00,	1,	{0x80}},	
	{0xff,	2,	{0x00,0x00}},
	{0xff,	3,	{0x00,0x00,0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};









static struct LCM_setting_table lcm_jdi2_initialization_setting[] = {
	{0x00,	1,	{0x00}},
	{0xff,	3,	{0x12,0x82,0x01}},
	{0x00,	1,	{0x80}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


 
static void lcd_power_on(unsigned char on  )
{
    if(on)
    {
       pmic_config_interface(0x045A, 0x7, 0x7, 5);//SET VGP6 = 3.3V
       pmic_config_interface(0x0434,1,0x1,15);
       MDELAY(1);

       pmic_config_interface(0x0456, 0x3, 0x7, 5);//SET VGP4 = 1.8V
       pmic_config_interface(0x0430,1,0x1,15);
        MDELAY(3);
	   SET_RESET_PIN(1);
	   MDELAY(10);
	   mt_set_gpio_out(29, 1); 
       MDELAY(2);
       SET_RESET_PIN(0);
	   MDELAY(1);
	   SET_RESET_PIN(1);
	   MDELAY(6);
	   SET_RESET_PIN(0);
	   MDELAY(1);
	   SET_RESET_PIN(1);
       MDELAY(6);
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


static struct LCM_setting_table lcm_jdi_deep_sleep_mode_in_setting[] = {
	// Display off sequence
	{0x28, 0, {0x00}},
    {REGFLAG_DELAY, 20, {}},
    // Sleep Mode On
	{0x10, 0, {0x00}},
    {REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
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
	    params->hw_width = HW_WIDTH;
	    params->hw_height= HW_HEIGHT;

		// enable tearing-free
		//params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
		//params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

        #if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
        #else
		params->dsi.mode   = BURST_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE; 
        #endif
	
		// DSI
		/* Command mode setting */
		//1 Three lane or Four lane
		params->dsi.LANE_NUM				= LCM_FOUR_LANE;
		//The following defined the fomat for data coming from LCD engine.
	//	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	//	params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	//	params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Highly depends on LCD driver capability.

		// Video mode setting		
		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
	//	params->dsi.vertical_active_line				= FRAME_HEIGHT;

	//	params->dsi.intermediat_buffer_num = 0;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage

		//params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4
		//params->dsi.pll_div2=1;		// div2=0,1,2,3;div1_real=1,2,4,4
		//params->dsi.fbk_div =19;		// fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)

		//params->dsi.pll_select=1;	//0: MIPI_PLL; 1: LVDS_PLL		
		params->dsi.PLL_CLOCK = LCM_DSI_6589_PLL_CLOCK_260;
}

#ifdef OPPO_R819
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
    #ifdef OPPO_R819
	oppo_lcd_status = 1;
	#endif
}

#ifndef UBOOT_LK
void lcm_change_paragrams(u8 i)

{

   unsigned char buffer_1[12] = {0}; 
   unsigned int array[16];
   unsigned int j;
   printk("lcm_change_paragrams i = %d \n",i);
   if(i==1)
   {
	//	push_table(lcm_jdi_rgb_standrd_initialization_setting, sizeof(lcm_jdi_rgb_standrd_initialization_setting) / sizeof(struct LCM_setting_table), 1);	
   }
   else if(i==2)
   {
		//push_table(lcm_jdi_rgb_cold_initialization_setting, sizeof(lcm_jdi_rgb_cold_initialization_setting) / sizeof(struct LCM_setting_table), 1);	
   }
   else if(i==3)
   {
		//push_table(lcm_jdi_rgb_warm_initialization_setting, sizeof(lcm_jdi_rgb_warm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
   }
   else
   {
        printk("do nothing!!!\n");
   }
   
     printk("read buffer\n");
    array[0] = 0x00033700; 
    //dsi_set_cmdq(array, 1, 1);
   // push_table(lcm_jdi2_initialization_setting, sizeof(lcm_jdi2_initialization_setting) / sizeof(struct LCM_setting_table), 1);
   // read_reg_v2(0xe1, buffer_1, 7); 
	 printk("read buffer2\n");
     for(j = 0; j < 7; j++)                 
      printk( "buffer_1[%d]:0x%x \n",j,buffer_1[j]); 
   
   
}
#endif





static void lcm_suspend(void)
{
	#ifndef UBOOT_LK
	DMS_LCD("===================>lcm_suspend\n");
	push_table(lcm_jdi_deep_sleep_mode_in_setting, sizeof(lcm_jdi_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
	SET_RESET_PIN(0);
	 MDELAY(10);
	 //mt_set_gpio_out(76, 0);
	 mt_set_gpio_mode(29, 0); //LCD 5v pin
     mt_set_gpio_dir(29, 1);
     mt_set_gpio_out(29, 0) ;
	 MDELAY(5);
   if(!istesuspend)
    {
          istesuspend=1;
    }
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
	#ifdef OPPO_R819
	oppo_lcd_status = 0;
	#endif
}


static void lcm_resume(void)
{
     unsigned char buffer_1[5];
	 unsigned char i;
	 unsigned int array[2]; 
	#ifndef UBOOT_LK
	lcd_dev = LCD_JDI;
	  mt_set_gpio_mode(29, 0);
     mt_set_gpio_dir(29, 1);
     mt_set_gpio_out(29, 0) ;
	 
	SET_RESET_PIN(0);
     MDELAY(10);
     lcd_power_on(1); 
	//lcd_power_on(1);      
  	DMS_LCD("===================>lcm_resume,lcm_paragrams =%d\n",lcm_paragrams);
 	push_table(lcm_jdi_initialization_setting, sizeof(lcm_jdi_initialization_setting) / sizeof(struct LCM_setting_table), 1);
	if(lcm_paragrams!=1)
	{
		//lcm_change_paragrams(lcm_paragrams);
	}
    if(istesuspend)
    {
          istesuspend=0;
          gbtecnt=60;
    }
    #endif
	#ifdef OPPO_R819
	oppo_lcd_status = 1;
	#endif
  
    
    	
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


LCM_DRIVER jdi_hd_lcm_drv_12093 = 
{
    .name			= "jdi_hd_lcm_drv_12093",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
//	.compare_id     = lcm_compare_id,
	.esd_check = lcm_esd_check,
	.esd_recover = lcm_esd_recover,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
    };