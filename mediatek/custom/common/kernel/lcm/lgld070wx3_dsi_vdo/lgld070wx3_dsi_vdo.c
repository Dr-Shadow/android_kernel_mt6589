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
	#include <platform/mt_pm_ldo.h>
	#include <platform/mt_pmic.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
	#include <asm/arch/mt_pm_ldo.h>
	#include <asm/arch/mt_pmic.h>
#include <linux/module.h>  /* Needed by all modules */
#include <linux/kernel.h>  /* Needed for KERN_ALERT */
#else
#include <linux/module.h>  /* Needed by all modules */
#include <linux/kernel.h>  /* Needed for KERN_ALERT */
	#include <mach/mt_gpio.h>
	#include <mach/mt_pm_ldo.h>
	#include <mach/mt_pmic.h>
  #include <mach/upmu_common.h>	
#endif

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (800)
#define FRAME_HEIGHT (1280)

#define LCM_ID_NT35590 (0x90)
// TODO. This LCM ID is NT51012 not 35590.


// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

#define GPIO_LCD_MIPI_PANEL_EN   GPIOEXT18
#define GPIO_LCD_BIST            GPIOEXT19
#define GPIO_LCD_PANEL_BKLTEN    GPIO177

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

static void lcd_power_en(unsigned char enabled)
{
  if(enabled)
  { 
  #if 0//def BUILD_LK
    //VGP4 1.8V
    pmic_config_interface(DIGLDO_CON10, 0x1, PMIC_RG_VGP4_EN_MASK, PMIC_RG_VGP4_EN_SHIFT);
    pmic_config_interface(DIGLDO_CON31, 0x03, PMIC_RG_VGP4_VOSEL_MASK, PMIC_RG_VGP4_VOSEL_SHIFT);    
	  MDELAY(1);
    //VGP6 3.3V
    pmic_config_interface(DIGLDO_CON12, 0x1, PMIC_RG_VGP6_EN_MASK, PMIC_RG_VGP6_EN_SHIFT);
    pmic_config_interface(DIGLDO_CON33, 0x07, PMIC_RG_VGP6_VOSEL_MASK, PMIC_RG_VGP6_VOSEL_SHIFT);
  #else
    //VGP4 1.8V
    upmu_set_rg_vgp4_en(0x1);
    upmu_set_rg_vgp4_vosel(0x3);     
    MDELAY(1);
    //VGP6 3.3V    
    upmu_set_rg_vgp6_en(0x1);
    upmu_set_rg_vgp6_vosel(0x7);    
  #endif   

/*
    mt_set_gpio_mode(GPIO_LCD_BIST, GPIO_MODE_00); //BIST PIN, always keep low	       
    mt_set_gpio_dir(GPIO_LCD_BIST, GPIO_DIR_OUT);        
    mt_set_gpio_out(GPIO_LCD_BIST, GPIO_OUT_ZERO);

    mt_set_gpio_mode(GPIO_LCD_MIPI_PANEL_EN, GPIO_MODE_00);	//MIPI_PANEL_EN PIN, always keep low	  
    mt_set_gpio_dir(GPIO_LCD_MIPI_PANEL_EN, GPIO_DIR_OUT);		 
    mt_set_gpio_out(GPIO_LCD_MIPI_PANEL_EN, GPIO_OUT_ZERO);
*/
  }
  else
  {
/*
    mt_set_gpio_mode(GPIO_LCD_BIST, GPIO_MODE_00);        //BIST PIN  
    mt_set_gpio_dir(GPIO_LCD_BIST, GPIO_DIR_OUT);        
    mt_set_gpio_out(GPIO_LCD_BIST, GPIO_OUT_ZERO); 

    mt_set_gpio_mode(GPIO_LCD_MIPI_PANEL_EN, GPIO_MODE_00);     //MIPI_PANEL_EN PIN, always keep low   
    mt_set_gpio_dir(GPIO_LCD_MIPI_PANEL_EN, GPIO_DIR_OUT);        
    mt_set_gpio_out(GPIO_LCD_MIPI_PANEL_EN, GPIO_OUT_ZERO);
*/
    #if 0 //def BUILD_LK
    //VGP4 1.8V
    pmic_config_interface(DIGLDO_CON10, 0x0, PMIC_RG_VGP4_EN_MASK, PMIC_RG_VGP4_EN_SHIFT);
    pmic_config_interface(DIGLDO_CON31, 0x0, PMIC_RG_VGP4_VOSEL_MASK, PMIC_RG_VGP4_VOSEL_SHIFT);    

    MDELAY(55);  //delay 55ms   

    //VGP6 3.3V
    pmic_config_interface(DIGLDO_CON12, 0x0, PMIC_RG_VGP6_EN_MASK, PMIC_RG_VGP6_EN_SHIFT);
    pmic_config_interface(DIGLDO_CON33, 0x0, PMIC_RG_VGP6_VOSEL_MASK, PMIC_RG_VGP6_VOSEL_SHIFT);

	#else
    //VGP6 3.3V    
    upmu_set_rg_vgp6_en(0x0);
    upmu_set_rg_vgp6_vosel(0x0);
    MDELAY(55);  //delay 55ms
    //VGP4 1.8V
    upmu_set_rg_vgp4_en(0x0);
    upmu_set_rg_vgp4_vosel(0x0);     
	#endif
  }
}
static void lcd_backlight_en(unsigned char enabled)
{
    if(enabled)
    {
    	  mt_set_gpio_mode(GPIO_LCD_PANEL_BKLTEN, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_LCD_PANEL_BKLTEN, GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_LCD_PANEL_BKLTEN, GPIO_OUT_ONE);
    }
    else
    {	
        mt_set_gpio_mode(GPIO_LCD_PANEL_BKLTEN, GPIO_MODE_00);
        mt_set_gpio_dir(GPIO_LCD_PANEL_BKLTEN, GPIO_DIR_OUT);
        mt_set_gpio_out(GPIO_LCD_PANEL_BKLTEN, GPIO_OUT_ZERO);    	
    }
}

static void init_lcm_registers(void)
{
	unsigned int data_array[16];
		
#if 1

	//data_array[0] = 0x00011500;  //software reset					 
	//dsi_set_cmdq(data_array, 1, 1);
    
    data_array[0] = 0x0BAE1500;                                         
    dsi_set_cmdq(data_array, 1, 1);
    
    data_array[0] = 0xEAEE1500;                                          
    dsi_set_cmdq(data_array, 1, 1);
    
    data_array[0] = 0x5FEF1500;                                          
    dsi_set_cmdq(data_array, 1, 1);
    
    data_array[0] = 0x68F21500;                                          
    dsi_set_cmdq(data_array, 1, 1);
    
    data_array[0] = 0x03A71500;                                          
    dsi_set_cmdq(data_array, 1, 1);

    //data_array[0] = 0xFFB21500;                                          
    //dsi_set_cmdq(data_array, 1, 1);

    //data_array[0] = 0x10CC1500;                                          
    //dsi_set_cmdq(data_array, 1, 1);

	MDELAY(20);

#if 0

	data_array[0] = 0x7DB21500;  					 
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x0BAE1500;  					 
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 0x18B61500;  					 
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 0xEAEE1500;  					 
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 0x5FEF1500;  					 
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 0x68F21500;  					 
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 0x00EE1500;  					 
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00EF1500;  					 
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0] = 0x64D21500;  					 
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00101500;  //sleep out                        
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(20);
#endif

	//data_array[0] = 0xEFB11500;                       
	//dsi_set_cmdq(data_array, 1, 1);
	//MDELAY(1);

	//data_array[0] = 0x00290500;  //display on                        
	//dsi_set_cmdq(data_array, 1, 1);
#endif

#if 0
	data_array[0] = 0x00010500;  //software reset					 
	dsi_set_cmdq(data_array, 1, 1);
	
	MDELAY(20);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x00000BAE; 					 
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);
	
	data_array[0] = 0x00023902; 
	data_array[1] = 0x0000EAEE; 					 
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);
	
	data_array[0] = 0x00023902; 
	data_array[1] = 0x00005FEF; 					 
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);
	
	data_array[0] = 0x00023902; 
	data_array[1] = 0x000068F2; 					 
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);
	
	data_array[0] = 0x00023902; 
	data_array[1] = 0x000000EE; 					 
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);
	
	data_array[0] = 0x00023902; 
	data_array[1] = 0x000000EF; 					 
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);
	
	data_array[0] = 0x00100500;  //sleep out                        
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(20);

	data_array[0] = 0x00290500;  //display on                        
	dsi_set_cmdq(data_array, 1, 1);
#endif
	//MDELAY(5);
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

#if 0
		// enable tearing-free
		params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
		params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;
#endif

        #if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
        #else
		params->dsi.mode   = BURST_VDO_MODE;
        #endif
	
		// DSI
		/* Command mode setting */
		//1 Three lane or Four lane
		params->dsi.LANE_NUM				= LCM_FOUR_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Highly depends on LCD driver capability.
		// Not support in MT6573
		params->dsi.packet_size=256;

		// Video mode setting		
		params->dsi.intermediat_buffer_num = 0;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		params->dsi.word_count=720*3;	

		
		params->dsi.vertical_sync_active				= 1;
		params->dsi.vertical_backporch					= 3;
		params->dsi.vertical_frontporch					= 3;
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 1;
		params->dsi.horizontal_backporch				= 57;
		params->dsi.horizontal_frontporch				= 32;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

		// Bit rate calculation
		//1 Every lane speed
		//params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
		//params->dsi.pll_div2=1;		// div2=0,1,2,3;div1_real=1,2,4,4	
		//params->dsi.fbk_div =31;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	

        params->dsi.PLL_CLOCK = LCM_DSI_6589_PLL_CLOCK_208;
		params->dsi.CLK_ZERO = 47;
		params->dsi.HS_ZERO = 36;

		params->dsi.compatibility_for_nvk = 2;		// this parameter would be set to 2 if DriverIC is NTK's (parellel arch) and when force match DSI clock for NTK's

}

static void lcm_init(void)
{
#ifndef BUILD_LK
    return; //do nothing in LK
#else
  lcd_power_en(1);
	MDELAY(50); 
	
	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(1);
	SET_RESET_PIN(1);

	init_lcm_registers();
	
	MDELAY(270);
	lcd_backlight_en(1);
#endif
}

#if 0
static void lcm_suspend(void)
{
	unsigned int data_array[16];

	data_array[0]=0x00280500; // Display Off
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(20);
	
	data_array[0] = 0x00111500; // Sleep In
	dsi_set_cmdq(data_array, 1, 1);

	lcd_backlight_en(0);
	MDELAY(200);
	lcd_power_en(0);
	
	MDELAY(1000);
}
static void lcm_suspend_power(void)
{
}
#else
static void lcm_suspend_power(void)
{
#ifndef BUILD_LK
    printk("%s (L:%d)\n", __FUNCTION__, __LINE__);
#endif
	lcd_power_en(0);
	MDELAY(1005);
}

static void lcm_suspend(void)
{
#ifndef BUILD_LK
    printk("%s (L:%d)\n", __FUNCTION__, __LINE__);
#endif
	unsigned int data_array[16];

	lcd_backlight_en(0);
	MDELAY(205);

	data_array[0]=0x00280500; // Display Off
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(20);
	
	data_array[0] = 0x00111500; // Sleep In
	dsi_set_cmdq(data_array, 1, 1);

}
#endif

#if 0
static void lcm_resume(void)
{
	unsigned int data_array[16];

  lcd_power_en(1);
	init_lcm_registers();

	data_array[0] = 0x00101500; // Sleep Out
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(20);

	data_array[0] = 0x00290500; // Display On
	dsi_set_cmdq(data_array, 1, 1);
	
        data_array[0] = 0x0BAE1500;                                         
        dsi_set_cmdq(data_array, 1, 1);
        
        data_array[0] = 0xEAEE1500;                                          
        dsi_set_cmdq(data_array, 1, 1);
        
        data_array[0] = 0x5FEF1500;                                          
        dsi_set_cmdq(data_array, 1, 1);
           
        #if 1
        data_array[0] = 0x68F21500;                                          
        dsi_set_cmdq(data_array, 1, 1);
        data_array[0] = 0xFFD21500;                                          
        dsi_set_cmdq(data_array, 1, 1);
        #else
        data_array[0] = 0xEFB11500;                                          
        dsi_set_cmdq(data_array, 1, 1);
        #endif
        
	MDELAY(300);
	lcd_backlight_en(1);
}
static void lcm_resume_power(void)
{
}
#else
static void lcm_resume_power(void)
{
#ifndef BUILD_LK
    printk("%s (L:%d)\n", __FUNCTION__, __LINE__);
#endif
    lcd_power_en(1);
	MDELAY(115);
    //lcd_backlight_en(1);

}
    
static void lcm_resume(void)
{
#ifndef BUILD_LK
    printk("%s (L:%d)\n", __FUNCTION__, __LINE__);
#endif
    unsigned int data_array[16];

	init_lcm_registers();

	data_array[0] = 0x00101500; // Sleep Out
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(20);

	data_array[0] = 0x00290500; // Display On
	dsi_set_cmdq(data_array, 1, 1);
	
	MDELAY(205);
    
	lcd_backlight_en(1);
}
#endif


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

	data_array[0]= 0x00290508; //HW bug, so need send one HS packet
	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);

}
#endif
#if 0
static unsigned int lcm_compare_id(void)
{
	unsigned int id=0;
	unsigned char buffer[2];
	unsigned int array[16];  

	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(1);
	
	SET_RESET_PIN(1);
	MDELAY(20); 

	array[0] = 0x00023700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);
	
	read_reg_v2(0xF4, buffer, 2);
	id = buffer[0]; //we only need ID
    #ifdef BUILD_LK
		printf("%s, LK nt35590 debug: nt35590 id = 0x%08x\n", __func__, id);
    #else
		printk("%s, kernel nt35590 horse debug: nt35590 id = 0x%08x\n", __func__, id);
    #endif

    if(id == LCM_ID_NT35590)
    	return 1;
    else
        return 0;


}
#endif

LCM_DRIVER lgld070wx3_dsi_vdo_lcm_drv = 
{
    .name			= "lgld070wx3_dsi_vdo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.suspend_power  = lcm_suspend_power,
	.resume_power   = lcm_resume_power,
	//.compare_id     = lcm_compare_id,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
    };
