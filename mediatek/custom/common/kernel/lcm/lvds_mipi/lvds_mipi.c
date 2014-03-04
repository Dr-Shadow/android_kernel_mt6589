/*****************************************************************************
*add for mipi to lvds  
*2013-2-22
*****************************************************************************/


#ifndef BUILD_LK
#include <linux/string.h>
#endif
#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>

#elif (defined BUILD_UBOOT)
#include <asm/arch/mt6577_gpio.h>
#else
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#endif
#include "lcm_drv.h"



// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (800)
#define FRAME_HEIGHT (1280) 

#define GPIO_LCM_PWDN         GPIO74
#define GPIO_LCM_CHIP_EN         GPIOEXT26


//#define SN65DSI_DEBUG  //for check system(bb dsi and ti chip) statu
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (mt_set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

typedef unsigned char    kal_uint8;

static struct sn65dsi8x_setting_table {
    unsigned char cmd;
    unsigned char data;
};
// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
extern void DSI_Continuous_clock(void);
extern void DSI_clk_HS_mode(bool enter);


/******************************************************************************
*IIC drvier,:protocol type 2 add by chenguangjian start
******************************************************************************/
#if 0
#define TI_WRITE_ID	0x5a
#define TI_READ_ID	0x5b
#define TI_I2C_DELAY 	100
#define gpio_ti_clk		GPIO119
#define gpio_ti_sda		GPIO118

#define SET_SCCB_CLK_OUTPUT_TI		{mt_set_gpio_mode(gpio_ti_clk, 0);mt_set_gpio_dir(gpio_ti_clk,GPIO_DIR_OUT);}
#define SET_SCCB_DATA_OUTPUT_TI		{mt_set_gpio_mode(gpio_ti_sda, 0);mt_set_gpio_dir(gpio_ti_sda,GPIO_DIR_OUT);}
#define SET_SCCB_DATA_INPUT_TI		{mt_set_gpio_mode(gpio_ti_sda, 0);mt_set_gpio_dir( gpio_ti_sda,GPIO_DIR_IN);}
#define SET_SCCB_CLK_HIGH_TI			mt_set_gpio_out(gpio_ti_clk,1)	
#define SET_SCCB_CLK_LOW_TI			mt_set_gpio_out(gpio_ti_clk,0)	
#define SET_SCCB_DATA_HIGH_TI		mt_set_gpio_out(gpio_ti_sda,1)	
#define SET_SCCB_DATA_LOW_TI		mt_set_gpio_out(gpio_ti_sda,0)	
#define GET_SCCB_DATA_BIT_TI			mt_get_gpio_out(gpio_ti_sda)

#define I2C_START_TRANSMISSION_TI \
{ \
	volatile kal_uint8 j; \
	SET_SCCB_CLK_OUTPUT_TI; \
	SET_SCCB_DATA_OUTPUT_TI; \
	SET_SCCB_CLK_HIGH_TI; \
	SET_SCCB_DATA_HIGH_TI; \
	UDELAY(TI_I2C_DELAY); \
	SET_SCCB_DATA_LOW_TI; \
	UDELAY(TI_I2C_DELAY); \
	SET_SCCB_CLK_LOW_TI; \
}

#define I2C_STOP_TRANSMISSION_TI \
{ \
	volatile kal_uint8 j; \
	SET_SCCB_CLK_OUTPUT_TI; \
	SET_SCCB_DATA_OUTPUT_TI; \
	SET_SCCB_CLK_LOW_TI; \
	SET_SCCB_DATA_LOW_TI; \
	UDELAY(TI_I2C_DELAY); \
	SET_SCCB_CLK_HIGH_TI; \
	UDELAY(TI_I2C_DELAY); \
	SET_SCCB_DATA_HIGH_TI; \
}
static void IIC_send_byte(kal_uint8 send_byte)
{
	volatile signed char i;
	volatile kal_uint8 j;

	for (i = 7; i >= 0; i--) 
	{ /* data bit 7~0 :data -start to change*/
		if (send_byte & (1 << i)) 
		{
			SET_SCCB_DATA_HIGH_TI;
		}
		else 
		{
			SET_SCCB_DATA_LOW_TI;
		}
		//data change -end
		UDELAY(TI_I2C_DELAY);
		SET_SCCB_CLK_HIGH_TI;//set cllk high :keep data avild
		UDELAY(TI_I2C_DELAY);
		SET_SCCB_CLK_LOW_TI;
		UDELAY(TI_I2C_DELAY);
	}
	/* 9th bit ignore ACK*/
	SET_SCCB_DATA_LOW_TI;
	SET_SCCB_DATA_INPUT_TI;
	SET_SCCB_CLK_HIGH_TI;
	UDELAY(TI_I2C_DELAY);
	SET_SCCB_CLK_LOW_TI;
	SET_SCCB_DATA_OUTPUT_TI;
}  

static kal_uint8 IIC_get_byte(void)
{
	volatile signed char i;
	volatile kal_uint8 j;
	kal_uint8 get_byte = 0;

	SET_SCCB_DATA_INPUT_TI;

	for (i = 7; i >= 0; i--) 
	{ 	/* data bit 7~0 */
		SET_SCCB_CLK_HIGH_TI;
		UDELAY(TI_I2C_DELAY);
		if (GET_SCCB_DATA_BIT_TI)
			get_byte |= (1 << i);
		UDELAY(TI_I2C_DELAY);
		SET_SCCB_CLK_LOW_TI;
		UDELAY(TI_I2C_DELAY);
	}
	/* 9th bit ignore ACK */
	SET_SCCB_DATA_HIGH_TI;
	SET_SCCB_DATA_OUTPUT_TI;
	UDELAY(TI_I2C_DELAY);
	SET_SCCB_CLK_HIGH_TI;
	UDELAY(TI_I2C_DELAY);
	SET_SCCB_CLK_LOW_TI;

	return get_byte;
}   

 void TI_Sensor_Write(kal_uint8 addr, kal_uint8 data)
{
#ifdef BUILD_LK
	printf("TI_Sensor_Write cmd=0x%x  data=0x%x \n",addr,data);
#else
	printk("TI_Sensor_Write cmd=0x%x  data=0x%x \n",addr,data);
#endif
	I2C_START_TRANSMISSION_TI;
	UDELAY(TI_I2C_DELAY);
	IIC_send_byte(TI_WRITE_ID);
	
	UDELAY(TI_I2C_DELAY);
	IIC_send_byte(addr);
	
	UDELAY(TI_I2C_DELAY);
	IIC_send_byte(data);
	
	UDELAY(TI_I2C_DELAY);
	I2C_STOP_TRANSMISSION_TI;
}
 kal_uint8  TI_Sensor_Read(kal_uint8 addr)
{
	kal_uint8 get_byte = 0;
	
	I2C_START_TRANSMISSION_TI;
	UDELAY(TI_I2C_DELAY);
	IIC_send_byte(TI_WRITE_ID);
	UDELAY(TI_I2C_DELAY);
	IIC_send_byte(addr);
	UDELAY(TI_I2C_DELAY);
	
	I2C_STOP_TRANSMISSION_TI;
	UDELAY(TI_I2C_DELAY);
	I2C_START_TRANSMISSION_TI;
	UDELAY(TI_I2C_DELAY);
	IIC_send_byte(TI_READ_ID);
	UDELAY(TI_I2C_DELAY);
	get_byte = IIC_get_byte();
	UDELAY(TI_I2C_DELAY);
	I2C_STOP_TRANSMISSION_TI;

	return get_byte;
}
#endif
 //i2c hardware type -start
#ifdef BUILD_LK
#define I2C_CH                3
#define sn65dsi8x_I2C_ADDR       0x58  //0x2d
 //extern unsigned long mt_i2c_channel_init(unsigned char channel);

 U32 sn65dsi8x_reg_i2c_read (U8 addr, U8 *dataBuffer)
 {
	 U32 ret_code = I2C_OK;
	 U8 write_data = addr;
 
	 /* set register command */
	 ret_code = mt_i2c_write(I2C_CH, sn65dsi8x_I2C_ADDR, &write_data, 1, 0); // 0:I2C_PATH_NORMAL
 
	 if (ret_code != I2C_OK)
		 return ret_code;
 
	 ret_code = mt_i2c_read(I2C_CH, sn65dsi8x_I2C_ADDR, dataBuffer, 1,0); // 0:I2C_PATH_NORMAL
 
	 return ret_code;
 }
 
 U32 sn65dsi8x_reg_i2c_write(U8 addr, U8 value)
 {
	 U32 ret_code = I2C_OK;
	 U8 write_data[2];
 
	 write_data[0]= addr;
	 write_data[1] = value;
 	
	 ret_code = mt_i2c_write(I2C_CH, sn65dsi8x_I2C_ADDR, write_data, 2,0); // 0:I2C_PATH_NORMAL
	 printf("sn65dsi8x_reg_i2c_write cmd=0x%x  data=0x%x ret_code=0x%x\n",addr,value,ret_code);
	 
	 return ret_code;
 }

 //end
 #else
extern int  sn65dsi83_write_reg(u8 reg, u8 val);
//extern signed int  sn65dsi83_read_reg(u8 reg);
 U32 sn65dsi8x_reg_i2c_read (U8 addr)
 {
	 U32 ret_code = 0;
	 U8 write_data = addr;
 
	 /* set register command */
	// ret_code = sn65dsi83_write_reg(&write_data, 1, 0); // 0:I2C_PATH_NORMAL
 
	 //if (ret_code != I2C_OK)
		// return ret_code;
 
	 //ret_code = sn65dsi83_read_reg(addr); // 0:I2C_PATH_NORMAL
 
	 return ret_code;
 }
U32 sn65dsi8x_reg_i2c_write(U8 addr, U8 value)
 {
	 U32 ret_code = 0;
	 U8 write_data[2];
 
	// write_data[0]= addr;
	 //write_data[1] = value;
 	
	 ret_code = sn65dsi83_write_reg(addr,value); // 0:I2C_PATH_NORMAL
	 printk("sn65dsi8x_reg_i2c_write cmd=0x%x  data=0x%x ret_code=0x%x\n",addr,value,ret_code);
	 
	 return ret_code;
 }
 #endif
 /******************************************************************************
 *IIC drvier,:protocol type 2 add by chenguangjian end
 ******************************************************************************/
//sn65dis83 chip init table
static struct sn65dsi8x_setting_table sn65dis83_init_table[]=
{
#if 1 ///////////////////////////
{0x09,              0x00},
{0x0A,              0x05},
{0x0B,              0x10},
{0x0D,              0x00},
{0x10,              0x26},
{0x11,              0x00},
{0x12,              0x28},//0x28
{0x13,              0x00},
{0x18,              0x70},
{0x19,              0x00},
{0x1A,              0x03},
{0x1B,              0x00},
{0x20,              0x20},
{0x21,              0x03},
{0x22,              0x00},
{0x23,              0x00},
{0x24,              0x00},
{0x25,              0x00},
{0x26,              0x00},
{0x27,              0x00},
{0x28,              0x22},
{0x29,              0x00},
{0x2A,              0x00},
{0x2B,              0x00},
{0x2C,              0x16},
{0x2D,              0x00},
{0x2E,              0x00},
{0x2F,              0x00},
{0x30,              0x02},
{0x31,              0x00},
{0x32,              0x00},
{0x33,              0x00},
{0x34,              0x16},
{0x35,              0x00},
{0x36,              0x00},
{0x37,              0x00},
{0x38,              0x00},
{0x39,              0x00},
{0x3A,              0x00},
{0x3B,              0x00},
{0x3C,              0x00},
{0x3D,              0x00},
{0x3E,              0x00},
{0x0D , 		  0x01},
{0xFF,0x00},//ended flag
#endif
};
static void push_table(struct sn65dsi8x_setting_table *table, unsigned int count)
{
	unsigned int i;

    for(i = 0; i < count; i++) {
		
        unsigned cmd;
        cmd = table[i].cmd;
        switch (cmd) {	
            //case REGFLAG_DELAY :
            //MDELAY(table[i].count);
            //    break;		
            case 0xFF:
                break;
				
            default:
		sn65dsi8x_reg_i2c_write(cmd, table[i].data);//TI_Sensor_Write(cmd, table[i].data);
       	}
    }
	
}
static void dump_reg_table(struct sn65dsi8x_setting_table *table, unsigned int count)
{
	unsigned int i;
	unsigned char data;
	
    for(i = 0; i < count; i++) {
		
        unsigned cmd;
        cmd = table[i].cmd;
        switch (cmd) {	
            //case REGFLAG_DELAY :
            //MDELAY(table[i].count);
            //    break;		
            case 0xFF:
                break;
				
            default:
		#ifdef BUILD_LK
		sn65dsi8x_reg_i2c_read(cmd, &data);	 //data=TI_Sensor_Read(cmd);//	
		printf("dump cmd=0x%x  data=0x%x \n",cmd,data);
		#else
		data=sn65dsi8x_reg_i2c_read(cmd);
		//printk("dump cmd=0x%x  data=0x%x \n",cmd,data);
		#endif
		
       	}
    }
	
}

void init_sn65dsi8x(void)
{
	unsigned char data;
    push_table(sn65dis83_init_table, sizeof(sn65dis83_init_table)/sizeof(struct sn65dsi8x_setting_table));
	MDELAY(5);
	sn65dsi8x_reg_i2c_write(0x09,1);//soft reset
	MDELAY(5);
	
	#if 1  //SN65DSI_DEBUG//add for debug
	sn65dsi8x_reg_i2c_write(0xe0,1);//
	sn65dsi8x_reg_i2c_write(0xe1,0xff);//
	MDELAY(5);
	#ifdef BUILD_LK
	sn65dsi8x_reg_i2c_read(0xe5, &data);
	printf("dump cmd=0xe5  data=0x%x \n",data);
	#else
	data=sn65dsi8x_reg_i2c_read(0xe5);
	printk("dump cmd=0xe5  data=0x%x \n",data);
	#endif
	
	#endif//debug end
	dump_reg_table(sn65dis83_init_table, sizeof(sn65dis83_init_table)/sizeof(struct sn65dsi8x_setting_table)); //for debug
	
}

	

#ifdef BUILD_LK
/*
extern	void upmu_set_rg_vgp2_vosel(U32 val);
extern  void upmu_set_rg_vgp6_vosel(U32 val);
extern	void upmu_set_rg_vgp2_en(U32 val);
extern	void upmu_set_rg_vgp6_en(U32 val);
*/
void lvds_power_init(void)
{
   mt_set_gpio_mode(GPIO_LCM_PWDN, 0);
    mt_set_gpio_dir(GPIO_LCM_PWDN, 1);
    mt_set_gpio_out(GPIO_LCM_PWDN, 1);
    MDELAY(200);
}
#else //for kernel
extern bool hwPowerOn(MT65XX_POWER powerId, MT65XX_POWER_VOLTAGE powerVolt, char *mode_name);
void lvds_kernel_power_init(void)
{
	mt_set_gpio_mode(GPIO_LCM_PWDN, 0);
    mt_set_gpio_dir(GPIO_LCM_PWDN, 1);
    mt_set_gpio_out(GPIO_LCM_PWDN, 1);
    MDELAY(200);
}
void lvds_kernel_power_deinit(void)
{
	mt_set_gpio_mode(GPIO_LCM_PWDN, 0);
    mt_set_gpio_dir(GPIO_LCM_PWDN, 1);
    mt_set_gpio_out(GPIO_LCM_PWDN, 0);
    MDELAY(200);
}

#endif
//
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

		params->dsi.mode   =BURST_VDO_MODE;// BURST_VDO_MODE;
	
		// DSI
		/* Command mode setting */
		params->dsi.LANE_NUM				= LCM_FOUR_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_BGR;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		params->dsi.packet_size=256;

	    params->dsi.word_count=800*3;	

		params->dsi.vertical_sync_active= 2;
		params->dsi.vertical_backporch= 3;
		params->dsi.vertical_frontporch= 3;
		params->dsi.vertical_active_line= FRAME_HEIGHT;//hight

		params->dsi.horizontal_sync_active				= 22;  //
		params->dsi.horizontal_backporch				= 22; //
		params->dsi.horizontal_frontporch				= 20; //
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;//=wight

		params->dsi.PS=LCM_PACKED_PS_18BIT_RGB666;
		params->dsi.pll_select=0;	//0: MIPI_PLL; 1: LVDS_PLL
		//params->dsi.PLL_CLOCK = LCM_DSI_6589_PLL_CLOCK_312;
		params->dsi.pll_div1=1;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
		params->dsi.pll_div2=2;		// div2=0,1,2,3;div1_real=1,2,4,4	
		params->dsi.fbk_div =40;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)
}


static void lcm_init(void)
{
#ifdef BUILD_LK

   printf("tM070ddh06--BUILD_LK--lcm_init \n");	
	//step1: sn65dsi8x enbable and init
    mt_set_gpio_mode(GPIO_LCM_CHIP_EN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_LCM_CHIP_EN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_LCM_CHIP_EN, GPIO_OUT_ONE);
    MDELAY(5);
    mt_set_gpio_out(GPIO_LCM_CHIP_EN, GPIO_OUT_ZERO);
    MDELAY(50);
    mt_set_gpio_out(GPIO_LCM_CHIP_EN, GPIO_OUT_ONE);
    MDELAY(50);
	
    mt_i2c_channel_init(3);
    MDELAY(50);
	//step2 :set  dsi :continuous and HS mode  
	DSI_clk_HS_mode(1);
	DSI_Continuous_clock();
	
   	init_sn65dsi8x();

//step 3 :lvds lcd init
    lvds_power_init();
#elif (defined BUILD_UBOOT)
	
#else
#if 1
	 printk("--kernel--lcm_init \n");
	  DSI_clk_HS_mode(1);
	  DSI_Continuous_clock();
	  MDELAY(50);

    	lvds_kernel_power_init();
	   

#endif
#endif    
}


static void lcm_suspend(void)
{
#ifdef BUILD_LK
mt_set_gpio_mode(GPIO_LCM_PWDN, GPIO_MODE_00);
mt_set_gpio_dir(GPIO_LCM_PWDN, GPIO_DIR_OUT);
mt_set_gpio_out(GPIO_LCM_PWDN, GPIO_OUT_ZERO);

#else
    mt_set_gpio_mode(GPIO_LCM_PWDN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_LCM_PWDN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_LCM_PWDN, GPIO_OUT_ZERO);

    MDELAY(60); // avoid LCD resume transint
    mt_set_gpio_mode(GPIO_LCM_CHIP_EN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_LCM_CHIP_EN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_LCM_CHIP_EN, GPIO_OUT_ZERO);
    MDELAY(50);
#endif
}


static void lcm_resume(void)
{    

#ifdef BUILD_LK
	printf("lcm_resume cmd= lvds_lk_power_init \n");
	lvds_power_init();
#else
    mt_set_gpio_mode(GPIO_LCM_CHIP_EN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_LCM_CHIP_EN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_LCM_CHIP_EN, GPIO_OUT_ONE);
    MDELAY(5);
    mt_set_gpio_out(GPIO_LCM_CHIP_EN, GPIO_OUT_ZERO);
    MDELAY(50);
    mt_set_gpio_out(GPIO_LCM_CHIP_EN, GPIO_OUT_ONE);
    MDELAY(50);

	printk("lcm_resume cmd lvds_kernel_power_init \n");
	//DSI_clk_HS_mode(1);
	//DSI_Continuous_clock();

	init_sn65dsi8x();
	//MDELAY(100);
	lvds_kernel_power_init();
	//MDELAY(300);
#endif



    MDELAY(120); 
}
static unsigned int lcm_compare_id(void)
{
#if defined(BUILD_LK)
		printf("TM070DDH06_MIPI2LVDS  lcm_compare_id \n");
#endif

    return 1;
}

LCM_DRIVER sn65dsi83_lcm_drv = 
{
    	.name		    = "sn65dsi83",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id    = lcm_compare_id,
};
