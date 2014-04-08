/************************************************************************************
** File: - \mediatek\custom\common\kernel\lcm\ht_wvga\lcm_drv.c
** OPPO_R819
** Copyright (C), 2008-2012, OPPO Mobile Comm Corp., Ltd
** 
** Description: 
**      4.0  wvga driver
** 
** Version: 1.2
** Date created: 16:01:40,27/02/2012
** Author: Yixue.Ge@BasicDrv.LCD
** 
** --------------------------- Revision History: --------------------------------
** 	<author>	<data>			<desc>
** Yixue.Ge@BasicDrv.LCD 12/03/2012 modify some timing for transfer err when suspend
** Yixue.Ge@BasicDrv.LCD 28/03/2012 modify clock to 247Mhz and add te signal
** Yixue.Ge@BasicDrv.LCD 24/04/2012 modify for compatible new LCD
** Yixue.Ge@BasicDrv.LCD 25/04/2012 resolve lcd sleep out fail when resume
** Yixue.Ge@BasicDrv.LCD 26/04/2012 add 5# LCD driver and modify clock to 195Mhz and 4# two buf
************************************************************************************/
/*
//#include <linux/string.h>
#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#define DMS_LCD(a,arg...) printf("[uboot LCD] "a,##arg)
#else
#include <mach/mt6577_gpio.h>
#include <linux/kernel.h>
//#include <linux/oppo_devices_list.h>
#define DMS_LCD(a,arg...) printk("[kernel LCD] "a,##arg)
#include <mach/eint.h>
#endif
#include "lcm_drv.h"
*/
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/oppo_devices_list.h>
#include <mach/mt_boot.h>
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
#endif


#define FRAME_WIDTH  										(480)
#define FRAME_HEIGHT 										(800)

#define HW_WIDTH 56
#define HW_HEIGHT 93
#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0xFF   // END OF REGISTERS MARKER

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#ifndef BUILD_LK
extern LCD_DEV lcd_dev;
#endif

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))


#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)   		    lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    


#define TRULY      0
#define BYD    1

struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};


#ifdef OPPO_R819
volatile int oppo_lcd_status = 1;
extern int g_lcd_id_value3;

int Istruly = 0;

extern int gbtecnt;


#endif

static struct LCM_setting_table lcm_cmd_initialization_setting[] = {
	
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
	{0x2A,	4,	{0x00, 0x00, 0x01,0xDF}},
	{0x2B,	4,	{0x00, 0x00, 0x03,0x1F}},
	{0x36,	1,	{0x00}},
	{0x3A,	1,	{0x77}},
	{0xB0,	1,	{0x04}},
	{0xD6,	1,	{0x28}},
	{0xFD,	13,	{0x00,0x00,0x70,0x00,0x32,0x31,0x34,0x30,
	             0x32,0x31,0x04,0x00,0x00}},
	{0xFE,	4,	{0x00,0x00,0x00,0x20}},
	{0xB0,	1,	{0x03}},
	{0x11,	1,	{0x00}},
	{REGFLAG_DELAY, 125, {}},
	{0x35,	1,	{0x00}},
	{0x44,  2,  {0x03,0x26}},
	{0x29,	1,	{0x00}},
	// Note
	// Strongly recommend not to set Sleep out / Display On here. That will cause messed frame to be shown as later the backlight is on.


	// Setting ending by predefined flag
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_vdo_initialization_setting[] = {
	
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
	{0x36,	1,	{0x00}},
	{0x3A,	1,	{0x70}},
	{0xB0,	1,	{0x04}},
	{0x11,	0,	{0x00}},
	{REGFLAG_DELAY, 125, {}},
	{0x29,	0,	{0x00}},
	{REGFLAG_DELAY, 10, {}},
	// Note
	// Strongly recommend not to set Sleep out / Display On here. That will cause messed frame to be shown as later the backlight is on.


	// Setting ending by predefined flag
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_vdo_byd_initialization_setting[] = {
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

	//Set EXTC
	{0xb9,	3,	{0xFF, 0x83, 0x69}},

        //Set internal oscillator
	//{REGFLAG_DELAY, 10, {}},
        
        //Set Power      
	{0xb1,	19,	{0x01,0x00,0x34,0x07,0x00,0x0E,0x0E,0x1A,0x22,0x3F,
	                     0x3F,0x07,0x0b,0x01,0xE6,0xE6,0xE6,0xE6,0xE6}},
	{REGFLAG_DELAY, 1, {}},
	
        //Set Display //Gionee wangc modified for TP slipped slowly
	{0xb2,	15,	{0x00,0x23,0x0A,0x0A,0x70,0x00,0xFF,0x00,0x00,0x00, 
	                     0x00,0x03,0x03,0x00,0x01}},
    {REGFLAG_DELAY, 1, {}},
        
        //Set Dipaly CYC
	{0xb4,	5,	{0x00,0x0C,0x84,0x0C,0x01}},
        
        //Set Vcom
    {0xb6,	2,	{0x35,0x35}},

        //Set GIP
	{0xd5,	26,	{0x00,0x05,0x03,0x00,0x01,0x09,0x10,0x70,0x37,0x27,
	                   0x20,0x31,0x46,0x8A,0x57,0x9B,0x20,0x31,0x46,0x8A,
	                     0x57,0x9B,0x07,0x0F, 0x02,0x00}},
	{REGFLAG_DELAY, 1, {}},

    {0xe0,	34,	{0x00,0x02,0x05,0x31,0x39,0x3F,0x0E,0x31,0x06,0x0C,
	                  0x0D,0x10,0x12,0x10,0x13,0x10,0x18,0x00,0x02,0x05,
	                  0x31,0x39,0x3F,0x0E,0x31,0x06,0x0C,0x0D,0x10,0x12,
	                  0x10,0x13,0x10,0x18}},
	
    {REGFLAG_DELAY, 1, {}},

	{0xba,	13,    {0x00,0xA0,0xC6,0x00,0x0A,0x00,0x10,0x30,0x6C,0x02,
                      0x11,0x18,0x40}},

	{0xC9, 5, {0x3e,0x00,0x00,0x01,0x4e}},
    {0x51, 1, {0x19}},
    {0x53, 1, {0x24}},
    {0x55, 1, {0x01}},
    {REGFLAG_DELAY, 1, {}},
	{0x3A,	1,	{0x77}},

	{0xCC,	1,	{0x02}},
        {0x11,	1,	{0x00}},
	{REGFLAG_DELAY, 150, {}},
        {0x29,	1,	{0x00}},
	// Note
	// Strongly recommend not to set Sleep out / Display On here. That will cause messed frame to be shown as later the backlight is on.

	{REGFLAG_END_OF_TABLE, 0x00, {}}  
};

static struct LCM_setting_table lcm_cmd_truly_initialization_setting[] = {	
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

	//Set EXTC
	{0xB9,	3,	{0xFF, 0x83, 0x69}},
	{REGFLAG_DELAY, 10, {}},

        //Set internal oscillator
	//{REGFLAG_DELAY, 10, {}},
        
        //Set Power      
	{0xB1,	19,	{0x01,0x00,0x34,0x07,0x00,0x0E,0x0E,0x1A,0x22,0x3F,0x3F,0x07,0x13,0x01,0xE6,0xE6,0xE6,0xE6,0xE6}},
	
	{REGFLAG_DELAY, 10, {}},
	
        //Set Display //Gionee wangc modified for TP slipped slowly
	{0xB2,	15,	{0x00,0x23,0x0A,0x0A,0x70,0x00,0xFF,0x00,0x00,0x00,0x00,0x03,0x03,0x00,0x01}},
    {REGFLAG_DELAY, 10, {}},
        
        //Set Dipaly CYC
	{0xB4, 	5,	{0x00,0x0C,0x84,0x0C,0x01}},
	{REGFLAG_DELAY, 10, {}},
        
        //Set Vcom
	{0xB6,	2,	{0x27, 0x27}},//apply it as ESD indication //2D,2D Gionee:wangc modified for log flash 3 times at 2012-7-15
	{REGFLAG_DELAY, 10, {}},

        //Set GIP
	{0xD5,	26, {0x00,0x05,0x03,0x00,0x01,0x09,0x10,0x70,0x37,0x37,
	                   0x20,0x31,0x46,0x8A,0x57,0x9B,0x20,0x31,0x46,0x8A,
	                   0x57,0x9B,0x07,0x0F,0x07,0x00}},
	{REGFLAG_DELAY, 10, {}},

	{0xE0,	34, {0x00,0x02,0x05,0x31,0x39,0x3F,0x0E,0x31,0x06,0x0C,
	                  0x0D,0x10,0x12,0x10,0x13,0x10,0x18,0x00,0x02,0x05,
	                  0x31,0x39,0x3F,0x0E,0x31,0x06,0x0C,0x0D,0x10,0x12,
	                  0x10,0x13,0x10,0x18}},
	
	{REGFLAG_DELAY, 10, {}},

    {0xBA,	13, {0x00,0xA0,0xC6,0x00,0x0A,0x00,0x10,0x30,0x6C,0x02,
                      0x11,0x18,0x40}},
        {REGFLAG_DELAY, 10, {}}, 

        {0x2A,	4,	{0x00,0x00,0x01,0xDF}},

        {0x2B,	4,	{0x00,0x00,0x03,0x1F}},

        {0x11,	1,	{0x00}},
        {REGFLAG_DELAY, 200, {}},
        {0x29,	1,	{0x00}},
        {REGFLAG_DELAY, 30, {}},
        {0x2C,	1,	{0x30}},
        {REGFLAG_DELAY, 30, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}  
};

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	// Display off sequence
	{0x28, 0, {0x00}},
    {REGFLAG_DELAY, 20, {}},
    // Sleep Mode On
	{0x10, 0, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_deep_vdo_sleep_mode_in_setting[] = {
	// Display off sequence
	{0x28, 0, {0x00}},
    {REGFLAG_DELAY, 20, {}},
    // Sleep Mode On
	{0x10, 0, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	{0xB0, 1, {0x04}},
	{0xB1, 1, {0x01}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_truly_sleep_mode_in_setting[] = {
	// Display off sequence
	//{0x28, 0, {0x00}},
    //{REGFLAG_DELAY, 20, {}},
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
#ifdef  BUILD_LK
static void lcm_get_id_pin()
{
#if 0	
    int tmp0 = 0,tmp1 = 0;  
    mt_set_gpio_mode(85, 0); 
    mt_set_gpio_dir(85, 0);
    mt_set_gpio_pull_select(85,0);
    mt_set_gpio_pull_enable(85,1);
    
    mt_set_gpio_mode(86, 0); 
    mt_set_gpio_dir(86, 0);
    mt_set_gpio_pull_select(86,0);
    mt_set_gpio_pull_enable(86,1);
    tmp0 = mt_get_gpio_in(85);
    tmp1 = mt_get_gpio_in(86);
	tmp0=1;
		printk("LCD get id: tmp0 = %d,tmp1 = %d\n",tmp0,tmp1);  
    if(tmp0)
        g_lcd_id_value3 = TRULY;
    else
        g_lcd_id_value3 = TRULY;

    if(g_lcd_id_value3 == TRULY)
    {
        mt_set_gpio_mode(36, 0); 
        mt_set_gpio_dir(36,1);
        mt_set_gpio_out(36,0);
    }
    #if 0
    if(g_lcd_id_value3 == HITACHI)
    {
        mt_set_gpio_mode(36, 0); 
        mt_set_gpio_dir(36, 1);
        mt_set_gpio_out(36,1);
    }
    #endif
    DMS_LCD("************************ %d *****************\n",g_lcd_id_value3);
#else
	int ret;
	ret = mt_set_gpio_mode(148, 0); 
	DMS_LCD("LCD get id: gpio148_mode = %d\n",ret);  
	ret = mt_set_gpio_dir(148, 1);
	DMS_LCD("LCD get id: gpio148_dir = %d\n",ret);  
	ret = mt_set_gpio_out(148,1);

	DMS_LCD("LCD get id: gpio148_lvl = %d\n",ret);  
        g_lcd_id_value3 = TRULY;
#endif
}
#endif
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
        DMS_LCD("lcm_get_params start\n");	
		
       // Istruly = TRULY;
		memset(params, 0, sizeof(LCM_PARAMS));
	
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

                params->hw_width = HW_WIDTH;
	        params->hw_height= HW_HEIGHT;
		// enable tearing-free
		params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
		params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;
		
	    params->dsi.mode   = SYNC_PULSE_VDO_MODE;
       
		// DSI
		/* Command mode setting */
		params->dsi.LANE_NUM				= LCM_TWO_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Highly depends on LCD driver capability.
		// Not support in MT6573
		params->dsi.packet_size=256;
		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		params->dsi.vertical_active_line				= FRAME_HEIGHT;
		params->dsi.intermediat_buffer_num = 2;

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		params->dsi.word_count=480*3;
	#if 0
		params->dsi.vertical_sync_active				= 8;
		params->dsi.vertical_backporch					= 8;
		params->dsi.vertical_frontporch					= 8;
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 
		params->dsi.horizontal_sync_active				= 36;
		params->dsi.horizontal_backporch				= 36;
		params->dsi.horizontal_frontporch				= 36;	
		params->dsi.horizontal_active_pixel			= FRAME_WIDTH;
	#endif
	#if 1
		params->dsi.vertical_sync_active				= 8;
		params->dsi.vertical_backporch					= 16;
		params->dsi.vertical_frontporch					= 8;
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 
		params->dsi.horizontal_sync_active				= 5;
		params->dsi.horizontal_backporch				= 5;
		params->dsi.horizontal_frontporch				= 5;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
	#endif
		// Bit rate calculation
		params->dsi.pll_div1=1; 	// div1=0,1,2,3;div1_real=1,2,4,4
		params->dsi.pll_div2=1;			// div2=0~15: fout=fvo/(2*div2)
		params->dsi.fbk_div =11;		// fref=26MHz, fvco=fref*(fbk_div+1)*fbk_real/(div1_real*div2_real)
		params->dsi.fbk_sel= 1; 		//fbk_sel = 0,1,2,3;fbk_real = 1,2,4,4
		params->dsi.rg_bir = 6;
		params->dsi.rg_bic = 2;
		params->dsi.rg_bp = 0xc;



}

//Fanhong.Kong@ProDrv.CHG, 2012/11/02 for hig temp compensation
volatile int OPPO_LED_ON = 1;
static void lcm_init(void)
{  
    //DMS_LCD("gyx lcm_init is called  g_lcd_id_value3 %d\n", g_lcd_id_value3);
#ifdef UBOOT_LK 
    DMS_LCD("===============>lcm_init start,g_lcd_id_value3=%d\n",g_lcd_id_value3);
	lcm_get_id_pin();
    SET_RESET_PIN(0);
    MDELAY(5);
    SET_RESET_PIN(1);
    MDELAY(10);
    if(g_lcd_id_value3 == BYD)
        push_table(lcm_vdo_byd_initialization_setting, sizeof(lcm_vdo_byd_initialization_setting) / sizeof(struct LCM_setting_table), 1);
	else if(g_lcd_id_value3 == TRULY)
	    push_table(lcm_cmd_truly_initialization_setting, sizeof(lcm_cmd_truly_initialization_setting) / sizeof(struct LCM_setting_table), 1);
    //DMS_LCD("lcm_init---------------------------------led_on = %d\r\n", OPPO_LED_ON);	
    DMS_LCD("===============>lcm_init end,g_lcd_id_value3=%d\n",g_lcd_id_value3);
#endif
    
#ifdef OPPO_R819
	oppo_lcd_status = 1;

#endif

}

//extern int gbtecnt;
int istesuspend=0;
static void lcm_suspend(void)
{
    //DMS_LCD("gyx lcm_suspend is called g_lcd_id_value3 %d\n", g_lcd_id_value3);
    DMS_LCD("===============>lcm_suspend,g_lcd_id_value3=%d\n",g_lcd_id_value3);
   #ifndef BUILD_LK
    if(!istesuspend)
    {
          istesuspend=1;
    }
	#endif
    
	    push_table(lcm_truly_sleep_mode_in_setting, sizeof(lcm_truly_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
	    SET_RESET_PIN(0);
        MDELAY(5);
	    SET_RESET_PIN(1);
	    MDELAY(10);
    //DMS_LCD("lcm_suspend---------------------------------led_on = %d\r\n", OPPO_LED_ON);
	
#ifdef OPPO_R819
	oppo_lcd_status = 0;
#endif
}


static void lcm_resume(void)
{
    DMS_LCD("gyx lcm_resume is called g_lcd_id_value3 %d\n", g_lcd_id_value3);
    SET_RESET_PIN(0);
    MDELAY(5);
    SET_RESET_PIN(1);
    MDELAY(10);
    if(g_lcd_id_value3 == BYD)
	{
		#ifndef BUILD_LK
        lcd_dev = LCD_BYD;
		DMS_LCD("lcm_resume : This is BYD lcd,lcd_dev = %d\n", lcd_dev);
        #endif
        push_table(lcm_vdo_byd_initialization_setting, sizeof(lcm_vdo_byd_initialization_setting) / sizeof(struct LCM_setting_table), 1);
	}
	else if(g_lcd_id_value3 == TRULY)
	{
	    #ifndef BUILD_LK
        lcd_dev = LCD_TURLY;
		DMS_LCD("lcm_resume : This is TRULY lcd,lcd_dev = %d\n", lcd_dev);
        #endif
	    push_table(lcm_cmd_truly_initialization_setting, sizeof(lcm_cmd_truly_initialization_setting) / sizeof(struct LCM_setting_table), 1);
    }
	/*
   #ifndef BUILD_LK
    if(istesuspend)
    {
          istesuspend=0;
          gbtecnt=60;
    }
    
    #endif
	*/

   //DMS_LCD("lcm_resume---------------------------------led_on = %d\r\n", OPPO_LED_ON);
   
#ifdef OPPO_R819
   oppo_lcd_status = 1;
#endif
	
}
#ifndef BUILD_LK
unsigned char TP_Detect_LCM_ID(void)
{
    char i;
	unsigned char buffer[2];
	unsigned int array[16];
	for(i=0;i<2;i++)
	{
	   buffer[i]=0;
	}
	//#error
	//array[0] = 0x04B02300;//set 0xb0 04
    //dsi_set_cmdq(array, 1, 1);
    array[0] = 0x00023700;
    dsi_set_cmdq(array, 1, 1);
    MDELAY(2);
    read_reg_v2(0xDB,buffer,2);	
	//if((buffer[2]==0x14)&&(buffer[3]==0x08))
	{
	   DMS_LCD("wjw_tp_lcm:buffer[0]=%x,buffer[1]=%x \n",buffer[0],buffer[1]);
	   return 1;
	}
	
	return 0;

}
#endif

#if 1
static unsigned char buffer_d1[7] ={0x18,0x0C,0x23,0x03,0x75,0x02,0x50};
static unsigned char buffer_truly_d0[10] ={0x99,0x03,0xCE,0xA6,0x0C,0x43,0x20,0x10,0x01,0x00};
static unsigned char buffer_hitachi_d0[10] ={0xa9,0x03,0xbd,0xa5,0x0c,0x72,0x20,0x10,0x01,0x0};

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
    DMS_LCD("gyx lcm_esd_recover is called\n");
    SET_RESET_PIN(0);
    MDELAY(5);
    SET_RESET_PIN(1);
    MDELAY(10);
    if(g_lcd_id_value3 == BYD)
        push_table(lcm_vdo_byd_initialization_setting, sizeof(lcm_vdo_byd_initialization_setting) / sizeof(struct LCM_setting_table), 1);
	else if(g_lcd_id_value3 == TRULY)
	    push_table(lcm_cmd_truly_initialization_setting, sizeof(lcm_cmd_truly_initialization_setting) / sizeof(struct LCM_setting_table), 1);

    return 0;
}

#endif

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
    //DMS_LCD("===============>lcm_update,g_lcd_id_value3=%d\n",g_lcd_id_value3);

}

// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------


LCM_DRIVER ht_wvga_lcm_drv =
{
	.name 			= "ht_wvga",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    //.update         = lcm_update,
   // .esd_check      = lcm_esd_check,
   // .esd_recover    = lcm_esd_recover,
};




