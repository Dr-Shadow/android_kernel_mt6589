#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_pmic.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include <mach/mt_gpio.h>
#include <linux/xlog.h>
#include <mach/mt_pm_ldo.h>
#endif


//GPIO_LVDS_LCD_POWERON_EN
//GPIO_LCM_RST_PIN
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (800)
#define FRAME_HEIGHT (1280)


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

static __inline void send_ctrl_cmd(unsigned int cmd)
{
    unsigned char temp1 = (unsigned char)((cmd >> 8) & 0xFF);
    unsigned char temp2 = (unsigned char)(cmd & 0xFF);

    lcm_util.send_data(0x2000 | temp1);
    lcm_util.send_data(0x0000 | temp2);
}

static __inline void send_data_cmd(unsigned int data)
{
    lcm_util.send_data(0x0004 | data);
}

static __inline void set_lcm_register(unsigned int regIndex,
                                      unsigned int regData)
{
    send_ctrl_cmd(regIndex);
    send_data_cmd(regData);
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

    params->type   = LCM_TYPE_DPI;
    //params->type   = LCM_TYPE_DSI;
    params->ctrl   = LCM_CTRL_SERIAL_DBI;
    //params->ctrl   = LCM_CTRL_GPIO;
    params->width  = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;
    params->io_select_mode = 0;	

    /* RGB interface configurations */
    
    params->dpi.mipi_pll_clk_ref  = 0;      //the most important parameters: set pll clk to 66Mhz and dpi clk to 33Mhz
    params->dpi.mipi_pll_clk_div1 =41;//19; // 37 
    params->dpi.mipi_pll_clk_div2 = 4;//2; // 4
    params->dpi.dpi_clk_div       = 2;
    params->dpi.dpi_clk_duty      = 1;

    params->dpi.clk_pol           = LCM_POLARITY_FALLING;
    params->dpi.de_pol            = LCM_POLARITY_RISING;
    params->dpi.vsync_pol         = LCM_POLARITY_FALLING;
    params->dpi.hsync_pol         = LCM_POLARITY_FALLING;

    params->dpi.hsync_pulse_width =10;//4;//128;// 160;//128;
    params->dpi.hsync_back_porch  =30;//30;//152;//300;//310;//152;
    params->dpi.hsync_front_porch = 20;//30;//40;//80;//40;
    params->dpi.vsync_pulse_width = 1;//2;//3;//10;//3;
    params->dpi.vsync_back_porch  = 3;//3;//12;//30;//12;
    params->dpi.vsync_front_porch = 2;//3;//10;//20;//10;

    params->dpi.hsync_pulse_width = 0;
    params->dpi.hsync_back_porch  = 0;
    params->dpi.hsync_front_porch = 0;
    params->dpi.vsync_pulse_width = 0;
    params->dpi.vsync_back_porch  = 0;
    params->dpi.vsync_front_porch = 0;
    
    params->dpi.format            = LCM_DPI_FORMAT_RGB666;   // format is 24 bit
    params->dpi.rgb_order         = LCM_COLOR_ORDER_RGB;
    params->dpi.is_serial_output  = 0;

    params->dpi.intermediat_buffer_num = 2;

    params->dpi.io_driving_current = LCM_DRIVING_CURRENT_2MA;
}


static void lcm_init(void)
{
#ifdef BUILD_LK
printf("shaokai:lcm_init");
#endif
    lcm_util.set_gpio_mode(GPIO_LCM_AVDD_EN_PIN, GPIO_MODE_00);    
    lcm_util.set_gpio_dir(GPIO_LCM_AVDD_EN_PIN, GPIO_DIR_OUT);
    lcm_util.set_gpio_out(GPIO_LCM_AVDD_EN_PIN, GPIO_OUT_ONE); // LCM_AVDD

    MDELAY(50);
    lcm_util.set_gpio_mode(GPIO_LVDS_LCD_SHUTDOWN_EN, GPIO_MODE_00);    
    lcm_util.set_gpio_dir(GPIO_LVDS_LCD_SHUTDOWN_EN, GPIO_DIR_OUT);
    lcm_util.set_gpio_out(GPIO_LVDS_LCD_SHUTDOWN_EN, GPIO_OUT_ONE); // LVDS_SHUTDOWN_N   
    MDELAY(120);
}

#if ( !defined(BUILD_LK)) && (!defined(BUILD_UBOOT))
DEFINE_SEMAPHORE(sem_lcm_suspend);
DEFINE_SEMAPHORE(sem_lcm_resume);
#endif
static void lcm_suspend(void)
{
#if ( !defined(BUILD_LK)) && (!defined(BUILD_UBOOT))
	if (down_interruptible(&sem_lcm_suspend)) {
        //printk("[fb driver] can't get semaphore:%d\n", __LINE__);
        return;
    }
#endif

    lcm_util.set_gpio_mode(GPIO_LVDS_LCD_SHUTDOWN_EN, GPIO_MODE_00);    
    lcm_util.set_gpio_dir(GPIO_LVDS_LCD_SHUTDOWN_EN, GPIO_DIR_OUT);
    lcm_util.set_gpio_out(GPIO_LVDS_LCD_SHUTDOWN_EN, GPIO_OUT_ZERO); // LVDS_SHUTDOWN_N      
    MDELAY(120);
    lcm_util.set_gpio_mode(GPIO_LCM_AVDD_EN_PIN, GPIO_MODE_00);    
    lcm_util.set_gpio_dir(GPIO_LCM_AVDD_EN_PIN, GPIO_DIR_OUT);
    lcm_util.set_gpio_out(GPIO_LCM_AVDD_EN_PIN, GPIO_OUT_ZERO); // LCM_AVDD     
    MDELAY(70); // avoid LCD resume transint
#if ( !defined(BUILD_LK)) && (!defined(BUILD_UBOOT))
	up(&sem_lcm_suspend);
#endif
}


static void lcm_resume(void)
{
#if ( !defined(BUILD_LK)) && (!defined(BUILD_UBOOT))

	if (down_interruptible(&sem_lcm_resume)) {
        //printk("[fb driver] can't get semaphore:%d\n", __LINE__);
        return;
    }
#endif
    lcm_util.set_gpio_mode(GPIO_LCM_AVDD_EN_PIN, GPIO_MODE_00);    
    lcm_util.set_gpio_dir(GPIO_LCM_AVDD_EN_PIN, GPIO_DIR_OUT);
    lcm_util.set_gpio_out(GPIO_LCM_AVDD_EN_PIN, GPIO_OUT_ONE); // LCM_AVDD
    MDELAY(50);
    lcm_util.set_gpio_mode(GPIO_LVDS_LCD_SHUTDOWN_EN, GPIO_MODE_00);    
    lcm_util.set_gpio_dir(GPIO_LVDS_LCD_SHUTDOWN_EN, GPIO_DIR_OUT);
    lcm_util.set_gpio_out(GPIO_LVDS_LCD_SHUTDOWN_EN, GPIO_OUT_ONE); // LVDS_SHUTDOWN_N   
    MDELAY(120);
#if ( !defined(BUILD_LK)) && (!defined(BUILD_UBOOT))
	up(&sem_lcm_resume);
#endif
}

LCM_DRIVER lvds_wxga_lcm_drv = 
{
    	.name			= "lvds_wxga",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
};

