#include <cust_leds.h>
#include <mach/mt_pwm.h>
#include <mach/mt_gpio.h>

#include <linux/kernel.h>
#include <linux/delay.h>
#include <asm/delay.h>
#include <mach/pmic_mt6329_hw_bank1.h> 
#include <mach/pmic_mt6329_sw_bank1.h> 
#include <mach/pmic_mt6329_hw.h>
#include <mach/pmic_mt6329_sw.h>
#include <mach/upmu_common_sw.h>
#include <mach/upmu_hw.h>

//extern int mtkfb_set_backlight_level(unsigned int level);
//extern int mtkfb_set_backlight_pwm(int div);
extern int disp_bls_set_backlight(unsigned int level);
/*
#define ERROR_BL_LEVEL 0xFFFFFFFF

unsigned int brightness_mapping(unsigned int level)
{  
	return ERROR_BL_LEVEL;
}
*/
void one_wire_control(unsigned int count)
{
	mt_set_gpio_mode(129, GPIO_MODE_GPIO);
	mt_set_gpio_dir(129, GPIO_DIR_OUT);
	
	count = 17-count;

	mt_set_gpio_out(129, 0);
	mdelay(3);
	printk("one_wire_control shot down LED driver IC to clear previous setting\n");
	
	while(count--)	//count = 1~16
	{
		
		mt_set_gpio_out(129, 1);
		udelay(100);
		mt_set_gpio_out(129, 0);
		udelay(100);
		//mt_set_gpio_out(gpio_num, 1);
	}
	mt_set_gpio_out(129, 1);
}

int Cust_GPIO_SetBacklight(unsigned int level)
{
	unsigned int mapped_level;

	if(0 != level)
	{
	    mapped_level = level/16 + 1; //1-wire control in S5 phone only has 16 step 
		printk("Cust_GPIO_SetBacklight:current_level=%d\n", mapped_level);
		one_wire_control(mapped_level);
	}
	else
	{
		mt_set_gpio_out(129, 0);
	}
	return 0;
}
unsigned int brightness_mapping(unsigned int level)
{
    unsigned int mapped_level;
    
    mapped_level = level;
       
	return mapped_level;
}

unsigned int Cust_SetBacklight(int level, int div)
{
    //mtkfb_set_backlight_pwm(div);
    //mtkfb_set_backlight_level(brightness_mapping(level));
    disp_bls_set_backlight(brightness_mapping(level));
    return 0;
}


static struct cust_mt65xx_led cust_led_list[MT65XX_LED_TYPE_TOTAL] = {
	{"red",               MT65XX_LED_MODE_NONE, -1,{0}},
	{"green",             MT65XX_LED_MODE_NONE, -1,{0}},
	{"blue",              MT65XX_LED_MODE_NONE, -1,{0}},
	{"jogball-backlight", MT65XX_LED_MODE_NONE, -1,{0}},
	{"keyboard-backlight",MT65XX_LED_MODE_NONE, -1,{0}},
	#ifndef VENDOR_EDIT
	//rendong.shi@Basic.drv, 2012/12/12, Add for 12083 keypadlight
	{"button-backlight",  MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_NLED_ISINK01,{0}},
	#else
	{"button-backlight",  MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_BUTTON,{0}},
	#endif
	{"lcd-backlight",     MT65XX_LED_MODE_CUST_BLS_PWM, (int)disp_bls_set_backlight,{0}},
};

struct cust_mt65xx_led *get_cust_led_list(void)
{
	return cust_led_list;
}

