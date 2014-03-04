#include <cust_leds.h>
#include <linux/delay.h>
#include <mach/mt_pwm.h>

#include <mach/mt_gpio.h>
#include <linux/kernel.h>
#include <mach/pmic_mt6329_hw_bank1.h> 
#include <mach/pmic_mt6329_sw_bank1.h> 
#include <mach/pmic_mt6329_hw.h>
#include <mach/pmic_mt6329_sw.h>
#include <mach/upmu_common_sw.h>
#include <mach/upmu_hw.h>
#include <linux/spinlock.h>

//extern int mtkfb_set_backlight_level(unsigned int level);
//extern int mtkfb_set_backlight_pwm(int div);
extern int disp_bls_set_backlight(unsigned int level);


extern BOOL DISP_GetBacklightEnable();

/*
#define ERROR_BL_LEVEL 0xFFFFFFFF

unsigned int brightness_mapping(unsigned int level)
{  
	return ERROR_BL_LEVEL;
}
*/
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

#if 1 //defined(BL_IC_KTD259)

static atomic_t lock=ATOMIC_INIT(0);;
/*******************************************************************************************
*
*      KTD259
*
********************************************************************************************/
#define KTD259_BRIGHTNESS_TOTAL_LEVEL   (32)
#define KTD259_BRIGHTNESS_STEP_LEVEL    (8)
int  ktd259_pre_gpio_dev_level = -1;
DEFINE_SPINLOCK(g_handle_bk_lock);
unsigned long g_handle_bk_flag;


static int  Cust_KTD259_SetBacklight(int level, int div)
{
	struct pwm_spec_config pwm_setting;
	static int clock_enable = 0;
	unsigned int con;
	int phylevel;
	int pulse_count;
	int i;
	BOOL s_enabled = FALSE;
/*************************************************************************************/	

/*************************************************************************************/	

	s_enabled = DISP_GetBacklightEnable();
//printk("\r>>>>>>>>>>>>>>>>>>>  HONGZHE:  Cust_KTD259_SetBacklight enable = %d, level = %d\n", s_enabled, level);
		
	if( s_enabled == FALSE )
	{
	spin_lock_irqsave(&g_handle_bk_lock, g_handle_bk_flag);
		    mt_set_gpio_mode(GPIO_MAINLCD_EN_PIN, GPIO_MAINLCD_EN_PIN_M_GPIO);
		    mt_set_gpio_dir(GPIO_MAINLCD_EN_PIN, GPIO_DIR_OUT);
			mt_set_gpio_out(GPIO_MAINLCD_EN_PIN, GPIO_OUT_ZERO);
			ktd259_pre_gpio_dev_level = 0;
			mdelay(5);
	 spin_unlock_irqrestore(&g_handle_bk_lock, g_handle_bk_flag);		
	 printk("\r >>>>>>>>>>>>>>>>>>>  HONGZHE:  return by backlight disable \n");
		return 0;
	}


	phylevel = level/KTD259_BRIGHTNESS_STEP_LEVEL;
    if( level>0 && phylevel == 0)
    {
        phylevel = 1;
    }

	//dmd_brightness_config(level);

	//printk(">>>>>>>>>>>>>>>>>>>  HONGZHE:  brightness_set_ktd259 : phylevel = %d <level=%d, pre-level=%d , div=%d\n", phylevel, level, ktd259_pre_gpio_dev_level, div);
	
	if( ktd259_pre_gpio_dev_level != -1 && ktd259_pre_gpio_dev_level == phylevel )
	{
		printk("\r >>>>>>>>>>>>>>>>>>>  HONGZHE:  return by backlight level same \n");
		return 0;
	}
	else if(ktd259_pre_gpio_dev_level == -1)
	{/* first op when boot */
		ktd259_pre_gpio_dev_level = KTD259_BRIGHTNESS_TOTAL_LEVEL-1;
	}

	if( phylevel <= ktd259_pre_gpio_dev_level )
	{
		pulse_count = ktd259_pre_gpio_dev_level - phylevel;
	}
	else
	{
		pulse_count = KTD259_BRIGHTNESS_TOTAL_LEVEL - (phylevel - ktd259_pre_gpio_dev_level);
	}
    printk(">>>>>>>>>>>>>>>>>>>  HONGZHE:  brightness_set_ktd259 : pulse_count = %d \n", pulse_count);

#ifdef GPIO_MAINLCD_EN_PIN
    spin_lock_irqsave(&g_handle_bk_lock, g_handle_bk_flag);
    mt_set_gpio_mode(GPIO_MAINLCD_EN_PIN, GPIO_MAINLCD_EN_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_MAINLCD_EN_PIN, GPIO_DIR_OUT);
    if( level > 0 )
    {

        if( ktd259_pre_gpio_dev_level == 0 )
        {
              mdelay(25);
        }
		    	mt_set_gpio_out(GPIO_MAINLCD_EN_PIN, GPIO_OUT_ONE);  
				mdelay(5);
	   	for( i=0; i<pulse_count; i++)
	   	{
	   		udelay(1);
	   		mt_set_gpio_out(GPIO_MAINLCD_EN_PIN, GPIO_OUT_ZERO);
	   		udelay(1);
	   		mt_set_gpio_out(GPIO_MAINLCD_EN_PIN, GPIO_OUT_ONE);
	   	}
    }
    else
    {
        mt_set_gpio_out(GPIO_MAINLCD_EN_PIN, GPIO_OUT_ZERO);
    }
	udelay(1);
    spin_unlock_irqrestore(&g_handle_bk_lock, g_handle_bk_flag);
#endif
    ktd259_pre_gpio_dev_level = phylevel;
   //atomic_set( &lock, 0 );
    return 0;
/*************************************************************************************/	


}
#endif

static struct cust_mt65xx_led cust_led_list[MT65XX_LED_TYPE_TOTAL] = {
	{"red",               MT65XX_LED_MODE_NONE, -1,{0}},
	{"green",             MT65XX_LED_MODE_NONE, -1,{0}},
	{"blue",              MT65XX_LED_MODE_NONE, -1,{0}},
	{"jogball-backlight", MT65XX_LED_MODE_NONE, -1,{0}},
	{"keyboard-backlight",MT65XX_LED_MODE_NONE, -1,{0}},
	{"button-backlight",  MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_BUTTON,{0}},
	//{"lcd-backlight",     MT65XX_LED_MODE_CUST_BLS_PWM, (int)disp_bls_set_backlight,{0}},
	{"lcd-backlight",     MT65XX_LED_MODE_CUST_LCM, (int)Cust_KTD259_SetBacklight,{0}},
};

struct cust_mt65xx_led *get_cust_led_list(void)
{
	return cust_led_list;
}

