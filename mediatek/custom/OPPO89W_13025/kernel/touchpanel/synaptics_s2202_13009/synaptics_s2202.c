/************************************************************************************
** File: - mediatek\custom\common\kernel\touchpanel\synaptics\synaptics_s3200.c
** VENDOR_EDIT
** Copyright (C), 2008-2012, OPPO Mobile Comm Corp., Ltd
** 
** Description: 
**      touch panel driver for synaptics
**      can change MAX_POINT_NUM value to support multipoint
** Version: 1.0
** Date created: 10:49:46,14/05/2012
** Author: Yixue.Ge@BasicDrv.TP
** 
** --------------------------- Revision History: --------------------------------
** 	<author>	<data>			<desc>
** Yongjun.Wu@BasicDrv.TP 18/05/2012 add some 12021 code
** Yongjun.Wu@BasicDrv.TP 19/05/2012 add Virtual key 
** Yongjun.Wu@BasicDrv.TP 21/05/2012 modify for Calibration
** Yongjun.Wu@BasicDrv.TP 08/06/2012 add firmware update
** Yongjun.Wu@BasicDrv.TP 12/06/2012 add Auto test
** Yongjun.WU@BasicDrv.TP 14/06/2012 modify for Auto test
************************************************************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <asm/uaccess.h>


#include <mach/mt_pm_ldo.h>
#include <mach/mt_gpio.h>
//#include <mach/mt_devs.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>

#include <mach/eint.h>
#include <mach/mt_boot.h>
#include <cust_eint.h>
#include <linux/kthread.h>


#include <mach/eint.h>
#include <cust_eint.h>

#include <linux/rtpm_prio.h>
#include <linux/sysfs.h>
#include <linux/oppo_devices_list.h>

#include "Synaptics_update_truly.h"
#include "Synaptics_update_ofilm.h"

#ifndef TPD_NO_GPIO 
#include "cust_gpio_usage.h"
#endif

#define TPD_DEVICE "mtk-tpd"
static unsigned int tp_multi_finger=0; 
static unsigned int tp_debug = 0; 
#ifdef VENDOR_EDIT
int Disable_key_during_touch =0;
#endif
#define TPD_DEBUG(a,arg...)\
	do{\
		if(tp_debug)\
			printk(TPD_DEVICE ": " a,##arg);\
	}while(0)

unsigned char bTP_Id_Detect_probe=0;
unsigned char bTP_Id_Detect_On=0;
static unsigned short SynaF34DataBase;
static unsigned short SynaF34QueryBase;
static unsigned short SynaF01DataBase;
static unsigned short SynaF01CommandBase;

static unsigned short SynaF34Reflash_BlockNum;
static unsigned short SynaF34Reflash_BlockData;
static unsigned short SynaF34ReflashQuery_BootID;
static unsigned short SynaF34ReflashQuery_FlashPropertyQuery;
static unsigned short SynaF34ReflashQuery_FirmwareBlockSize;
static unsigned short SynaF34ReflashQuery_FirmwareBlockCount;
static unsigned short SynaF34ReflashQuery_ConfigBlockSize;
static unsigned short SynaF34ReflashQuery_ConfigBlockCount;

static unsigned short SynaFirmwareBlockSize;
static unsigned short SynaFirmwareBlockCount;
static unsigned long SynaImageSize;

static unsigned short SynaConfigBlockSize;
static unsigned short SynaConfigBlockCount;
static unsigned long SynaConfigImageSize;

static unsigned short SynaBootloadID;

static unsigned short SynaF34_FlashControl;

static unsigned char *SynafirmwareImgData;
static unsigned char *SynaconfigImgData;
static unsigned char *SynalockImgData;
static unsigned int SynafirmwareImgVersion;	
static struct workqueue_struct *synaptics_wq = NULL;
static unsigned int is_suspend = 0;
#define TPDTM_DMESG(a,arg...) printk(TPD_DEVICE "(tm1347): " a,##arg)
#define SYNAPTICS_NAME       "synaptics-tm1347"

static int err_report_num = 0;
static int reset_staus = 0;
static unsigned int boot_mode = 0;
static atomic_t is_touch = ATOMIC_INIT(0);
static DEFINE_SEMAPHORE(work_sem);
extern TP_DEV tp_dev;
extern int TP_FW;
static unsigned int all_finger_up = 0;
static struct proc_dir_entry *prEntry = NULL; 



#ifdef CONFIG_HAS_EARLYSUSPEND
static void synaptics_ts_early_suspend(struct early_suspend *h);
static void synaptics_ts_late_resume(struct early_suspend *h);
#endif
//extern void update_lcd();
#define VKNUMBER 3
#ifdef BTN_STATE
static int tpd_keys[VKNUMBER] = {KEY_BACK,KEY_HOME,KEY_MENU};
#else
#if 0
static int tpd_keys[VKNUMBER][5] = {
	{KEY_MENU,136,1805,200,100},
	{KEY_HOME,494,1805,200,100},
	{KEY_BACK,855,1805,200,100},
	};
#endif

static int tpd_keys[VKNUMBER][5] = {
	{KEY_MENU,114, 2010,180,250},
	{KEY_HOME,520,2010,250,400},
	{KEY_BACK,1078,2010,180,250},
	};
#endif
#define MAX_POINT_NUM      5//must <=10
static struct point_info {
    int x;
    int raw_x;
    int y;
    int raw_y;
    int z;
};



//typedef struct point_info points[MAX_POINT_NUM];

static struct synaptics_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct input_dev *kpd;
	int use_irq;
	struct hrtimer timer;
	struct hrtimer report_timer;
	struct work_struct  work;
    uint32_t pre_finger_state;
	uint32_t flags;
	uint32_t btn_state;
	uint16_t max_x;
	uint16_t max_y;
	uint32_t no_erroponit_exist;
	uint32_t finger_num;
	struct early_suspend early_suspend;
};

/* ========================================= */
/* implementation of  virtual key attribute */
/* ========================================= */
static ssize_t cap_vk_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf){
      /* LEFT: search: CENTER: menu ,home:search 412, RIGHT: BACK */
      return sprintf(buf,
        	__stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":75:875:90:100"
         ":" __stringify(EV_KEY) ":" __stringify(KEY_HOME)   ":240:875:90:100"
         ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":420:875:100:100"		 
         "\n");


}

static int synaptics_probe(struct platform_device *pdev);
static int synaptics_remove(struct platform_device *pdev);


static struct platform_driver synaptics_driver = {
    .remove     = synaptics_remove,
    .probe      = synaptics_probe,
    .driver     = {
    	.owner = THIS_MODULE,
        .name = "synaptics-platform",
    },
};

#define TPD_USE_EINT

#ifdef TPD_USE_EINT
#define TOUCHPANEL_INT_PIN GPIO124
#endif

#define TP_ID_DETECT GPIO54 //0:Truly;1:Ofilm
#define TP_TRULY_ID 0
#define TP_OFILM_ID 1

//#ifdef CONFIG_LOCAL_WDT
#if 1
//#error
static enum wk_wdt_type {
	WK_WDT_LOC_TYPE,
	WK_WDT_EXT_TYPE
};
extern void mtk_wdt_restart(enum wk_wdt_type type);
#else

//#error
extern void mtk_wdt_restart(void);
#endif
extern void mtk_wdt_disable(void);
//extern TP_DEV tp_dev;
//#define TP_TRULY 0
//int tp_dev = TP_TRULY ;
static int kick_wdt()
{
	mtk_wdt_restart(WK_WDT_LOC_TYPE);
	mtk_wdt_restart(WK_WDT_EXT_TYPE);

}

static int kick_wdt_thread(void *data)
{
	mtk_wdt_restart(WK_WDT_LOC_TYPE);
	mtk_wdt_restart(WK_WDT_EXT_TYPE);

}
static void kick_Wdt_cpu0()
{

   struct task_struct *kick_wdt;
   kick_wdt = kthread_create(kick_wdt_thread,NULL,"kick_wdt_thread");
   
   if(IS_ERR(kick_wdt))
   {
	   printk("kick_wdt_thread creat erro\n");
   }
   else{
	   kthread_bind(kick_wdt,0);
	   wake_up_process (kick_wdt);
   }
}


static void kick_Wdt_cpu1()
{

   struct task_struct *kick_wdt;
   kick_wdt = kthread_create(kick_wdt_thread,NULL,"kick_wdt_thread");
   
   if(IS_ERR(kick_wdt))
   {
	   printk("kick_wdt_thread creat erro\n");
   }
   else{
	   kthread_bind(kick_wdt,1);
	   wake_up_process (kick_wdt);
   }
}



static void tpd_down(struct synaptics_ts_data *ts,
    int raw_x, int raw_y, int x, int y, int p) {
    if(ts && ts->input_dev) {
        input_report_key(ts->input_dev, BTN_TOUCH, 1);
        input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, p);
        input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, (raw_x+raw_y)/2);
        input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
        input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
        input_mt_sync(ts->input_dev);
	    all_finger_up = 1;
        TPD_DEBUG("Down[%4d %4d %4d]\n", x, y, p);
    }  
}

static void tpd_up(struct synaptics_ts_data *ts,
    int raw_x, int raw_y, int x, int y, int p) {
	 if(ts && ts->input_dev) {
        //input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, p);
        //input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, (raw_x+raw_y)/2);
        //input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
        //input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
         input_report_key(ts->input_dev, BTN_TOUCH, 0);
        input_mt_sync(ts->input_dev);
	    all_finger_up = 0;
        TPD_DEBUG("Up[%4d %4d %4d]\n", x, y, 0);
    }  
}

static void tpd_button(struct synaptics_ts_data *ts,
    unsigned int x, unsigned int y, unsigned int down) {
    int i;
    if(down) {
        for(i=0;i<VKNUMBER;i++) {
            if(x>=tpd_keys[i][1]&&
               x<=tpd_keys[i][1]+tpd_keys[i][3] &&
               y>=tpd_keys[i][2]&&
               y<=tpd_keys[i][2]+tpd_keys[i][4]&&
               !(ts->btn_state&(1<<i))) {
                input_report_key(ts->input_dev, tpd_keys[i][0], 1);
				TPD_DEBUG("tpd_button:press down the key i = %d,btn_state = 0x%x\n",i,ts->btn_state);
                ts->btn_state|=(1<<i);
            }
        }
    } else {
        for(i=0;i<4;i++) {
            if(ts->btn_state&(1<<i)) {
                input_report_key(ts->input_dev, tpd_keys[i][0], 0);
				TPD_DEBUG("tpd_button:press up the key i = %d,btn_state = 0x%x\n",i,ts->btn_state);
            }
        }
        ts->btn_state=0;
    }
}
static void tpd_hw_pwron(void)
{
	hwPowerOn(MT65XX_POWER_LDO_VGP5,VOL_2800,"CTP");
	msleep(1);
    hwPowerOn(MT65XX_POWER_LDO_VGP4,VOL_1800,"CTP");
	msleep(250);//hsy@oppo.com, add 2011/9/9 to speed up screen resume
}
static void tpd_hw_pwroff(void)
{
	hwPowerDown(MT65XX_POWER_LDO_VGP4,"CTP");
	msleep(1);
    hwPowerDown(MT65XX_POWER_LDO_VGP5,"CTP");
}
static void tpd_power(unsigned int on)
{
	//static unsigned int power_on = 0;
	//if(power_on != on)
	//{
		if(on)
		{
			tpd_hw_pwron();
		}
		else
		{
			tpd_hw_pwroff();
		}
		//power_on = on;
	//}
}
static int F11_2D_QUERY_BASE;
static int F11_2D_CMD_BASE;
static int F11_2D_CTRL_BASE;
static int F11_2D_DATA_BASE;

static int F01_RMI_QUERY_BASE;
static int F01_RMI_CMD_BASE;
static int F01_RMI_CTRL_BASE;
static int F01_RMI_DATA_BASE;

static int F34_FLASH_QUERY_BASE;
static int F34_FLASH_CMD_BASE;
static int F34_FLASH_CTRL_BASE;
static int F34_FLASH_DATA_BASE;

static int F01_RMI_QUERY11;
static int F01_RMI_CTRL00;
static int F11_2D_CTRL00;
static int F01_RMI_CTRL01;
static int F11_2D_CTRL06;
static int F11_2D_CTRL08;
static int F11_2D_DATA01;
static int F11_2D_CMD00;
static int F01_RMI_CMD00;
static int F34_FLASH_CTRL00;


static int synaptics_read_register_map(struct synaptics_ts_data *ts)
{
   uint8_t buf[4];   
   int ret;

	memset(buf, 0, sizeof(buf));
   	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0); 
	if (ret < 0) {
		TPD_DEBUG("synaptics_read_register_map: failed for page select\n");
		return -1;
		}
	ret = i2c_smbus_read_i2c_block_data(ts->client, 0xDD, 4, &(buf[0x0]));
    F11_2D_QUERY_BASE = buf[0];
	F11_2D_CMD_BASE = buf[1];
	F11_2D_CTRL_BASE = buf[2]; 
	F11_2D_DATA_BASE = buf[3];
	
    TPD_DEBUG("F11_2D_QUERY_BASE = %x \n \
		F11_2D_CMD_BASE  = %x \n\
		   F11_2D_CTRL_BASE	= %x \n\
		   F11_2D_DATA_BASE	= %x \n\
		   ",F11_2D_QUERY_BASE,F11_2D_CMD_BASE,F11_2D_CTRL_BASE,F11_2D_DATA_BASE);

	
	ret = i2c_smbus_read_i2c_block_data(ts->client, 0xE3, 4, &(buf[0x0]));    
	F01_RMI_QUERY_BASE = buf[0];
	F01_RMI_CMD_BASE = buf[1];
	F01_RMI_CTRL_BASE = buf[2]; 
	F01_RMI_DATA_BASE = buf[3];
    TPD_DEBUG("F01_RMI_QUERY_BASE = %x \n\
          F01_RMI_CMD_BASE  = %x \n\
		    F01_RMI_CTRL_BASE	= %x \n\
		    F01_RMI_DATA_BASE	= %x \n\
		   ",F01_RMI_QUERY_BASE,F01_RMI_CMD_BASE,F01_RMI_CTRL_BASE,F01_RMI_DATA_BASE);


	ret = i2c_smbus_read_i2c_block_data(ts->client, 0xE9, 4, &(buf[0x0]));	  
		F34_FLASH_QUERY_BASE = buf[0];
		F34_FLASH_CMD_BASE = buf[1];
		F34_FLASH_CTRL_BASE = buf[2]; 
		F34_FLASH_DATA_BASE = buf[3];
		TPD_DEBUG("F34_FLASH_QUERY_BASE = %x \n\
			  F34_FLASH_CMD_BASE	= %x \n\
				F34_FLASH_CTRL_BASE	= %x \n\
				F34_FLASH_DATA_BASE	= %x \n\
			   ",F34_FLASH_QUERY_BASE,F34_FLASH_CMD_BASE,F34_FLASH_CTRL_BASE,F34_FLASH_DATA_BASE);

	
		F01_RMI_QUERY11 = F11_2D_QUERY_BASE+11;
		F01_RMI_CTRL00 = F01_RMI_CTRL_BASE;
		F11_2D_CTRL00 = F11_2D_CTRL_BASE;
		F01_RMI_CTRL01 = F01_RMI_CTRL_BASE + 1;
		F11_2D_CTRL06 = F11_2D_CTRL_BASE + 6;
		F11_2D_CTRL08 = F11_2D_CTRL_BASE + 8;
		F11_2D_DATA01 = F11_2D_DATA_BASE + 2;
		F11_2D_CMD00 = F11_2D_CMD_BASE;
		F01_RMI_CMD00 = F01_RMI_CMD_BASE;
		F34_FLASH_CTRL00 = F34_FLASH_CTRL_BASE;
}
static int synaptics_read_product_id(struct synaptics_ts_data *ts)
{
	uint8_t buf[11];
	int ret ;
	
	memset(buf, 0 , sizeof(buf));
    /* page select = 0x0 */
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0); 
	if (ret < 0) {
		TPD_DEBUG("synaptics_read_product_id: failed for page select\n");
		return -1;
	}
	ret = i2c_smbus_read_i2c_block_data(ts->client, F01_RMI_QUERY11, 10, &(buf[0x0]));
	
	if (ret < 0) {
		TPD_DEBUG("synaptics_read_product_id: failed to read product info\n");
		return -1;
	}
	TPD_DEBUG("synaptics product id: %s \n",buf);
	return 0;
}

static int synaptics_init_panel(struct synaptics_ts_data *ts)
{
	int ret;
	/* page select = 0x0 */
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0); 
	if (ret < 0) {
		TPD_DEBUG("i2c_smbus_write_byte_data failed for page select\n");
		return -1;
	}
	/*device control: normal operation, configur=1*/
	ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CTRL00, 0x80); 
	if (ret < 0) {
		TPD_DEBUG("i2c_smbus_write_byte_data failed for mode select\n");
		return -1;
	}
    //enable absolutePosFilter
	//ret = i2c_smbus_write_byte_data(ts->client, F11_2D_CTRL00, 0x08); /*Enable or disable enhance reporting mode  */
	if (ret < 0) {
		TPD_DEBUG("i2c_smbus_write_byte_data failed for report mode select\n");
		return -1;
	}
	return ret;
}

static int synaptics_enable_interrupt(struct synaptics_ts_data *ts,
    int enable)
{

	int ret;
	uint8_t abs_status_int;
    /* page select = 0x0 */
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0); 
	if(ret < 0) {
		
		TPD_DEBUG("synaptics_enable_interrupt: select page failed ret = %d\n",
		    ret);
		return -1;
	}

	if(enable) {
		abs_status_int = 0x3f;

		/*clear interrupt bits for previous touch*/
		ret = i2c_smbus_read_byte_data(ts->client, 0x14);
		if(ret < 0) {
			TPD_DEBUG("synaptics_enable_interrupt :clear interrupt bits failed\n");
			return -1;
		}
	} else {
		abs_status_int = 0x0;
	}	
	
	ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CTRL01, abs_status_int);
	if(ret < 0) {
		TPD_DEBUG("synaptics_enable_interrupt: enable or disable abs \
		    interrupt failed,abs_int =%d\n",abs_status_int);
		return -1;
	}
	
	return 0;	
}

static void synaptics_ts_irq_handler(void);

static void synaptics_software_rezero(struct synaptics_ts_data *ts)
{

	printk("synaptics do software rezero\n");

	i2c_smbus_write_byte_data(ts->client, F11_2D_CMD_BASE, 0x01);
	msleep(50);
	synaptics_init_panel(ts);
	synaptics_enable_interrupt( ts, 1);

	//adp1650_torch_control_tp(30);
	//msleep(100);
	//adp1650_torch_control_tp(0);	

}

static void delay_qt_ms(unsigned long  w_ms)
{
    unsigned long i;
    unsigned long j;

    for (i = 0; i < w_ms; i++)
    {
        for (j = 0; j < 1000; j++)
        {
            udelay(1);
        }
    }
}
static void get_id(void)
{
    int tmp0 = 0,tmp1 = 0; 
	mt_set_gpio_mode(182, 0); 
	mt_set_gpio_dir(182,0);
	mt_set_gpio_pull_enable(182, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(182, GPIO_PULL_UP);
	mt_set_gpio_mode(138, 0); 
	mt_set_gpio_dir(138,0);
	mt_set_gpio_pull_enable(138, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(138, GPIO_PULL_UP);
	tmp0 = mt_get_gpio_in(182);
    tmp1 = mt_get_gpio_in(138);
     printk("tmp0=%d,tmp1=%d\n",tmp0 ,tmp1);
	if(tmp0 == 0 && tmp1 == 0 )
	{
        printk("tmp00\n");
	    tp_dev = TP_TRULY;  //2
	}
	else if (tmp0 == 1 && tmp1 == 1 )
	{
	    tp_dev = TP_OFILM;  //6
	}
	else
	{
	  tp_dev = TP_NONE;
	}
	printk(" tp_dev = %d\n",tp_dev);
}
static void int_state(struct synaptics_ts_data *ts)
{
    int ret = -1;
	ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CMD00,0x01);
	if(ret) {
		TPD_DEBUG("synaptics_ts_work_func:cannot  reset touch panel \n");
		return;
	}
	printk("ESD detected!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! init state now!\n");
	delay_qt_ms(150);
	mt_set_gpio_dir(TOUCHPANEL_INT_PIN,GPIO_DIR_IN);
	mt_set_gpio_pull_enable(TOUCHPANEL_INT_PIN, GPIO_PULL_ENABLE);

	mt_set_gpio_pull_select(TOUCHPANEL_INT_PIN, GPIO_PULL_UP);
	mt65xx_eint_set_polarity(CUST_EINT_TOUCH_PANEL_NUM,CUST_EINT_POLARITY_LOW);
	mt65xx_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM,
		CUST_EINT_LEVEL_SENSITIVE);//level low pull up
	mt65xx_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, 
		(CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN >> 4));
	mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, 
				CUST_EINT_DEBOUNCE_DISABLE, 
				CUST_EINT_POLARITY_LOW, 
				synaptics_ts_irq_handler, 
				0);
	ret = synaptics_enable_interrupt(ts, 1);
	if(ret) {
		TPD_DEBUG("synaptics_ts_work_func:cannot  enable interrupt \n");
		return;
	}
	ret = synaptics_init_panel(ts);
	if (ret < 0) {
		TPD_DEBUG("synaptics_ts_work_func: control tm1400 to sleep failed\n");
		return;
	}

}
static void int_btn(struct synaptics_ts_data *ts)
{
    int ret = -1,i = 0;
    uint8_t button_status = 0;
    ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x02); 
	if (ret < 0) {
		TPD_DEBUG("i2c_smbus_write_byte_data failed for page select\n");
		return;
	}
	ret =i2c_smbus_read_byte_data(ts->client, 0x00);
	if(ret < 0)
	{
        TPD_DEBUG("i2c_smbus_read_byte_data failed for button state\n");
		return;
	}
	button_status = ret&0x0f;
	for(i = 0;i < VKNUMBER;i++)
	{   
        if(button_status&(1<<i))
        {
            if(!(ts->btn_state&(1<<i)))
            {
                input_report_key(ts->input_dev, tpd_keys[i], 1);
            }
        }else
        {
            if(ts->btn_state&(1<<i))
            {
                input_report_key(ts->input_dev, tpd_keys[i], 0);     
            }
        }
	}
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0); 
	if (ret < 0) {
		TPD_DEBUG("i2c_smbus_write_byte_data failed for page select\n");
		return;
	}
	input_sync(ts->input_dev);
	ts->btn_state = button_status;
}

static void int_touch(struct synaptics_ts_data *ts)
{
    int ret= -1,i=0, j = 0;
    uint8_t buf[5];
    uint32_t finger_state = 0;
	uint8_t finger_num = 0;
    struct point_info points;
    memset(buf,0,sizeof(buf));
    ret = i2c_smbus_read_i2c_block_data(ts->client, 0x15, 2, &(buf[0]));
	if (ret < 0) {
		TPD_DEBUG("synaptics_ts_work_func: i2c_transfer failed\n");
        return;
	}
    finger_state = (buf[1]<<8)|buf[0];	
	TPD_DEBUG("finger_state = 0x%x\n",finger_state);
	for(j = 0;j < MAX_POINT_NUM;j++){
	    if(finger_state&(0x03<<j*2))
	    finger_num = finger_num+1;
		ts->finger_num=finger_num;
	}

	TPD_DEBUG("finger_num = %d\n",finger_num);
	if(finger_num > 0)	{
	
	    if(finger_num>=2)
		{
	        tp_multi_finger=1;		
		}

		
	for(i = 0;i < MAX_POINT_NUM;i++)
		{
			ret = i2c_smbus_read_i2c_block_data(ts->client, F11_2D_DATA01+ i*5,
			    5, &(buf[0]));
			if (ret < 0) {
				TPD_DEBUG("synaptics_ts_work_func: i2c_transfer failed\n");
	        	return;
			}
			points.x = (buf[0]<<4) | (buf[2] & 0x0f);
			points.raw_x = buf[3]&0x0f;
			points.y = (buf[1]<<4) | ((buf[2] & 0xf0)>>4);
			points.raw_y = (buf[3]&0xf0)>>4;
		    points.z = buf[4];
			
			if((boot_mode == FACTORY_BOOT)||(boot_mode == RECOVERY_BOOT))
			{
				if(points.z > 0){
					if(points.y > 2010){
			tpd_button(ts,points.x,points.y,1);
					}
					else{
							tpd_down(ts,points.raw_x, points.raw_y, points.x, points.y,
				    points.z);
						}
			 
				 }
			}
			else{
			if(points.z > 0){
			tpd_down(ts,points.raw_x, points.raw_y, points.x, points.y,
			    points.z);
				if((points.x >= 1000)&&(points.y >= 2000))
			{
            if(err_report_num == 0)
			{
				hrtimer_start(&ts->report_timer, ktime_set(1, 0), HRTIMER_MODE_REL);
			}
		
		    err_report_num ++;
			}
			TPD_DEBUG("err_report_num = %d \n",err_report_num);
			}
		}
			
		}
	}
	else{

			if((boot_mode == FACTORY_BOOT)||(boot_mode == RECOVERY_BOOT))
			tpd_button(ts,points.x,points.y,0);
			tpd_up(ts,points.raw_x, points.raw_y, points.x, points.y,
			    points.z);
	}
			
	input_sync(ts->input_dev);
	ts->pre_finger_state = finger_state;  
}

static void synaptics_ts_work_func(struct work_struct *work)
	{
		int ret = -1;
		uint8_t status = 0;
		uint8_t inte = 0;
		struct synaptics_ts_data *ts = container_of(work,
			struct synaptics_ts_data, work);
	
		if(is_suspend)
			return;
		
	    down(&work_sem);
	    atomic_set(&is_touch,1);

		
		ret = i2c_smbus_read_word_data(ts->client, 0x13);
		
		if (ret < 0) {
			TPD_DEBUG("synaptics_ts_work_func: i2c_transfer failed\n");
			goto ERR_I2C2;
		}
		status = ret&0xff;
		inte = (ret&0x7f00)>>8;
		TPD_DEBUG("synaptics_ts_work_func,inte = %x, status = %x\n",inte,status);
		//if(inte&0x02)
		if(status)
			int_state(ts);
		if(reset_staus){
			synaptics_software_rezero(ts);
			reset_staus = 0;
		}
		if(inte&0x20)
			int_btn(ts); 
	
		if(inte&0x04)
			int_touch(ts);
		ERR_I2C2:
	    atomic_set(&is_touch,0);
	    up(&work_sem);

#ifdef TPD_USE_EINT
		if (ts->use_irq)
			mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
#endif
	}

#ifndef TPD_USE_EINT
static enum hrtimer_restart synaptics_ts_timer_func(struct hrtimer *timer)
{
	struct synaptics_ts_data *ts = container_of(timer,
	    struct synaptics_ts_data, timer);
	/* TPD_DEBUG("synaptics_ts_timer_func\n"); */

	queue_work(synaptics_wq, &ts->work);

	hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}
#else
static struct synaptics_ts_data *ts_g;
static void synaptics_ts_irq_handler(void)
{
	TPD_DEBUG("synaptics_ts_irq_handler\n");
	mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	queue_work(synaptics_wq, &ts_g->work);
}
#endif
static enum hrtimer_restart synaptics_ts_report_timer_func(struct hrtimer *report_timer)
{
	if(err_report_num >= 60)	
	{
	  reset_staus = 1;
	}

	err_report_num = 0;		
	
	return HRTIMER_NORESTART;
}

static ssize_t tp_show(struct device_driver *ddri, char *buf)
{
    uint8_t ret = 0;
	//uint8_t ret0 = 0;
	
	/*mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	mt_set_gpio_mode(TOUCHPANEL_INT_PIN, GPIO_MODE_01);
	mt_set_gpio_dir(TOUCHPANEL_INT_PIN,GPIO_DIR_IN);
	mt_set_gpio_pull_enable(TOUCHPANEL_INT_PIN, GPIO_PULL_ENABLE);

	mt_set_gpio_pull_select(TOUCHPANEL_INT_PIN, GPIO_PULL_UP);
	mt65xx_eint_set_polarity(CUST_EINT_TOUCH_PANEL_NUM,CUST_EINT_POLARITY_LOW);
	mt65xx_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM,
		CUST_EINT_LEVEL_SENSITIVE);//level low pull up
	mt65xx_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, 
			(CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN >> 4));
	mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, 
					CUST_EINT_DEBOUNCE_DISABLE, 
					CUST_EINT_POLARITY_LOW, 
					synaptics_ts_irq_handler, 
					0);*/
	
	mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);

	
	ret = i2c_smbus_read_byte_data(ts_g->client, 0x13);
	if(ret < 0)
		printk("tp_show read i2c err\n");
	
	//ret0 = i2c_smbus_read_byte_data(ts_g->client, 0x14);
	//if(ret0 < 0)
		//printk("tp_show read i2c err\n");
	//update_lcd();
	return sprintf(buf, "0x13=0x%x\n", ret);
}

static ssize_t store_tp(struct device_driver *ddri, char *buf, size_t count)
{
	u8 tmp = 0;
	if (1 == sscanf(buf, "%d", &tmp))
	{
		tp_debug = tmp;
	}
	else
	{
		TPD_DEBUG("invalid content: '%s', length = %d\n", buf, count);
	}
	
	return count;
}

static DRIVER_ATTR(oppo_tp_debug, 0664, tp_show, store_tp);

#define TX_NUM 11
#define RX_NUM 20
#define MIN_ROW 1800
#define MAX_ROW 4800
static int DiagonalUpperLimit = 1100;
static int DiagonalLowerLimit = 900;
#define OTHERSUPPERLIMIT 200


static int F54_ANALOG_QUERY_BASE;//0x73
static int F54_ANALOG_COMMAND_BASE;//0x72
static int F54_ANALOG_CONTROL_BASE;//0x0d
static int F54_ANALOG_DATA_BASE;//0x00


static void synaptics_read_register_map_page1(struct synaptics_ts_data *ts)
{

unsigned char buf[4];
int ret;
printk("synaptics_read_register_map_page1 start\n");
ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x1); 
if (ret < 0) {
		TPD_DEBUG("i2c_smbus_write_byte_data failed for page select\n");
		return -1;
	}
ret = i2c_smbus_read_i2c_block_data(ts->client, 0xE9, 4, &(buf[0x0]));
F54_ANALOG_QUERY_BASE = buf[0];
printk("F54_ANALOG_QUERY_BASE = 0x%x\n",F54_ANALOG_QUERY_BASE);
F54_ANALOG_COMMAND_BASE = buf[1];
printk("F54_ANALOG_COMMAND_BASE = 0x%x\n",F54_ANALOG_COMMAND_BASE);
F54_ANALOG_CONTROL_BASE = buf[2];
printk("F54_ANALOG_CONTROL_BASE = 0x%x\n",F54_ANALOG_CONTROL_BASE);
F54_ANALOG_DATA_BASE = buf[3];
printk("F54_ANALOG_DATA_BASE = 0x%x\n",F54_ANALOG_DATA_BASE);


}


#define F54_ANALOG_DATA00 0X00
#define F54_ANALOG_DATA01 0X01
#define F54_ANALOG_DATA02 0X02
#define F54_ANALOG_DATA03 0X03
#define F54_ANALOG_CMD00  0X72
#define F54_ANALOG_QUERY00 0X73
#define F54_ANALOG_QUERY01 0X74
#define F54_ANALOG_CTRL41 0X5e
#define F54_ANALOG_CTRL57 0X63

static int16_t Truly_array_limit[] = {
	1176,	1960,	1168,	1946,	1173,	1954,	1172,	1954,	1170,	1950,	1172,	1954,	1178,	1964,	1182,	1970,	1187,	1978,	1214,	2023,	1077,	1794,	1079,	1799,	1092,	1820,	1139,	1898,	1090,	1816,	1078,	1797,	1074,	1791,	1075,	1792,	1078,	1797,	1085,	1809,
	1103,	1838,	1129,	1882,	1116,	1860,	1121,	1869,	1109,	1849,	1113,	1855,	1112,	1854,	1114,	1857,	1110,	1850,	1111,	1852,	1097,	1829,	1099,	1832,	1120,	1867,	1112,	1854,	1141,	1901,	1097,	1829,	1093,	1822,	1095,	1825,	1098,	1830,	1095,	1824,
	1089,	1815,	1106,	1842,	1109,	1849,	1109,	1848,	1104,	1841,	1104,	1840,	1108,	1847,	1107,	1846,	1105,	1842,	1104,	1840,	1098,	1830,	1101,	1834,	1139,	1898,	1145,	1908,	1146,	1910,	1101,	1834,	1095,	1825,	1097,	1829,	1100,	1833,	1100,	1834,
	1079,	1798,	1110,	1849,	1106,	1844,	1112,	1853,	1104,	1840,	1107,	1844,	1107,	1845,	1109,	1848,	1105,	1842,	1104,	1840,	1100,	1834,	1103,	1838,	1102,	1836,	1110,	1849,	1136,	1893,	1117,	1862,	1098,	1829,	1098,	1830,	1102,	1837,	1102,	1837,
	1079,	1798,	1100,	1833,	1105,	1841,	1106,	1843,	1102,	1837,	1102,	1837,	1104,	1840,	1104,	1840,	1102,	1837,	1101,	1835,	1101,	1834,	1103,	1838,	1102,	1837,	1104,	1840,	1082,	1804,	1178,	1963,	1114,	1856,	1101,	1836,	1104,	1840,	1106,	1844,
	1076,	1793,	1104,	1840,	1103,	1839,	1108,	1847,	1102,	1837,	1104,	1841,	1105,	1841,	1106,	1843,	1103,	1838,	1101,	1835,	1107,	1845,	1109,	1848,	1106,	1844,	1108,	1846,	1082,	1804,	1138,	1897,	1155,	1926,	1140,	1900,	1133,	1888,	1103,	1839,
	1074,	1790,	1097,	1829,	1102,	1836,	1103,	1838,	1100,	1833,	1100,	1833,	1102,	1836,	1102,	1837,	1099,	1832,	1098,	1830,	1108,	1847,	1110,	1850,	1107,	1846,	1109,	1848,	1082,	1804,	1134,	1890,	1148,	1914,	1129,	1881,	1121,	1868,	1162,	1936,
	1073,	1788,	1100,	1833,	1100,	1833,	1103,	1839,	1099,	1831,	1100,	1834,	1102,	1836,	1101,	1835,	1099,	1831,	1097,	1829,	1113,	1856,	1114,	1857,	1110,	1850,	1112,	1853,	1086,	1810,	1110,	1849,	1105,	1842,	1104,	1841,	1110,	1849,	1116,	1860,
	1072,	1787,	1095,	1825,	1097,	1829,	1098,	1831,	1097,	1828,	1098,	1830,	1100,	1834,	1098,	1830,	1097,	1828,	1095,	1825,	1118,	1864,	1116,	1859,	1113,	1855,	1114,	1857,	1087,	1812,	1112,	1853,	1106,	1844,	1108,	1846,	1112,	1853,	1125,	1875,
	1069,	1782,	1095,	1826,	1096,	1826,	1099,	1831,	1096,	1826,	1098,	1830,	1100,	1833,	1098,	1829,	1096,	1826,	1095,	1825,	1129,	1882,	1126,	1876,	1119,	1866,	1121,	1868,	1093,	1821,	1116,	1860,	1112,	1854,	1111,	1852,	1118,	1863,	1136,	1894,
	1035,	1725,	1057,	1761,	1059,	1765,	1060,	1766,	1058,	1763,	1059,	1765,	1061,	1769,	1060,	1767,	1058,	1763,	1057,	1761,	1420,	2366,	1312,	2187,	1284,	2140,	1281,	2135,	1260,	2099,	1256,	2093,	1248,	2080,	1249,	2082,	1258,	2096,	1336,	2227
};

static int16_t Ofilm_array_limit[] = {
	1367,	2279,	1394,	2323,	1395,	2324,	1394,	2324,	1396,	2327,	1443,	2404,	1450,	2417,	1415,	2358,	1401,	2335,	1400,	2333,	1764,	2940,	1623,	2706,	1586,	2643,	1560,	2600,	1551,	2585,	1548,	2580,	1546,	2576,	1542,	2570,	1568,	2613,	1694,	2823,
	1398,	2330,	1429,	2382,	1430,	2384,	1430,	2383,	1433,	2389,	1478,	2463,	1453,	2421,	1439,	2399,	1438,	2396,	1436,	2393,	1488,	2480,	1491,	2485,	1481,	2468,	1499,	2498,	1614,	2690,	1454,	2423,	1457,	2428,	1457,	2429,	1465,	2442,	1472,	2453,
	1400,	2333,	1432,	2387,	1431,	2385,	1433,	2388,	1435,	2392,	1500,	2499,	1451,	2419,	1462,	2436,	1440,	2400,	1439,	2399,	1453,	2421,	1450,	2416,	1452,	2420,	1452,	2420,	1452,	2421,	1456,	2427,	1459,	2432,	1455,	2425,	1468,	2447,	1490,	2484,
	1399,	2331,	1430,	2383,	1431,	2385,	1432,	2386,	1434,	2389,	1442,	2403,	1441,	2401,	1440,	2400,	1442,	2404,	1441,	2402,	1452,	2419,	1455,	2424,	1455,	2424,	1460,	2432,	1479,	2465,	1446,	2410,	1448,	2413,	1446,	2410,	1453,	2421,	1455,	2425,
	1402,	2336,	1432,	2387,	1433,	2388,	1434,	2391,	1436,	2394,	1503,	2505,	1487,	2478,	1492,	2487,	1443,	2405,	1444,	2406,	1450,	2416,	1444,	2407,	1448,	2413,	1447,	2412,	1448,	2414,	1451,	2418,	1454,	2423,	1447,	2412,	1457,	2428,	1463,	2438,
	1404,	2340,	1436,	2392,	1435,	2392,	1437,	2395,	1440,	2400,	1447,	2412,	1445,	2409,	1447,	2412,	1448,	2413,	1447,	2411,	1449,	2415,	1450,	2417,	1451,	2418,	1458,	2430,	1460,	2433,	1445,	2408,	1449,	2415,	1444,	2407,	1449,	2415,	1445,	2408,
	1408,	2347,	1440,	2399,	1438,	2397,	1439,	2399,	1442,	2404,	1507,	2512,	1462,	2437,	1455,	2424,	1455,	2424,	1453,	2422,	1469,	2448,	1445,	2408,	1448,	2413,	1449,	2414,	1448,	2414,	1449,	2416,	1454,	2424,	1448,	2413,	1455,	2426,	1455,	2426,
	1408,	2346,	1440,	2401,	1442,	2403,	1441,	2402,	1444,	2406,	1451,	2419,	1450,	2417,	1453,	2421,	1455,	2426,	1457,	2429,	1449,	2416,	1451,	2418,	1450,	2418,	1460,	2434,	1457,	2429,	1445,	2408,	1445,	2408,	1443,	2406,	1448,	2413,	1445,	2409,
	1423,	2371,	1448,	2413,	1446,	2410,	1445,	2408,	1446,	2411,	1457,	2428,	1454,	2424,	1455,	2424,	1455,	2425,	1458,	2430,	1516,	2526,	1463,	2438,	1452,	2420,	1450,	2416,	1447,	2412,	1448,	2414,	1453,	2421,	1447,	2411,	1455,	2426,	1448,	2413,
	1417,	2362,	1445,	2408,	1443,	2405,	1446,	2409,	1448,	2414,	1454,	2424,	1453,	2422,	1456,	2427,	1458,	2430,	1464,	2440,	1472,	2454,	1502,	2503,	1479,	2466,	1488,	2480,	1498,	2496,	1444,	2407,	1445,	2408,	1441,	2402,	1446,	2410,	1440,	2399,
	1519,	2531,	1492,	2486,	1498,	2496,	1494,	2490,	1502,	2503,	1508,	2513,	1505,	2508,	1511,	2518,	1518,	2529,	1536,	2559,	1440,	2401,	1467,	2446,	1449,	2416,	1434,	2389,	1407,	2345,	1409,	2349,	1415,	2358,	1404,	2341,	1412,	2353,	1405,	2342
};

static void checkCMD()
{
  int ret;
 do {

        delay_qt_ms(10); //wait 1ms

        ret =  i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE);
        
    } while (ret > 0x00); 
}

static ssize_t tp_test_show(struct device_driver *ddri, char *buf)
{
    int ret = 0;
	int x,y;
	int tx_data = 0;
	int tx_datah;
	int tx_datal;
	int16_t baseline_data = 0;
	uint8_t tmp = 0;
	uint8_t tmp_old = 0,tmp_new = 0;
	uint8_t tmp_l = 0,tmp_h = 0; 
	ssize_t num_read_chars = 0;
	int16_t Rxdata[RX_NUM][RX_NUM] = {0};
	uint16_t count = 0;
    down(&work_sem);

    mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);

	synaptics_read_register_map_page1(ts_g);
	
	printk("step 1:select report type 0x03\n");
	 //step 1:check raw capacitance.
	ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_DATA_BASE,0x03);//select report type 0x03
	 if (ret < 0) {
		 TPD_DEBUG("read_baseline: i2c_smbus_write_byte_data failed \n");
		 //return sprintf(buf, "i2c err!");
	}
	ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE+8);
	tmp_old = ret&0xff;
	tmp_new = tmp_old & 0xef;
	printk("ret = %x ,tmp_old =%x ,tmp_new = %x\n",ret,tmp_old,tmp_new);
	ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE+8,tmp_new);
	ret = i2c_smbus_write_word_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0x04);
	checkCMD();
	printk("forbid CBC oK\n");
	ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE + 81,0X01);  //Forbid NoiseMitigation F54_ANALOG_CTRL41
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0X04);//force F54_ANALOG_CMD00
    checkCMD();
	printk("forbid Forbid NoiseMitigation oK\n");
	ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0X02);//Force Cal, F54_ANALOG_CMD00
    checkCMD();
	printk("Force Cal oK\n");
	ret = i2c_smbus_write_word_data(ts_g->client,F54_ANALOG_DATA_BASE+1,0x00);//set fifo 00
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0X01);//get report
	checkCMD();
	count = 0;
	for(x = 0;x < TX_NUM; x++)
   	{
      for(y = 0; y < RX_NUM; y++)
   	  {
	    ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_DATA_BASE+3);
	    tmp_l = ret&0xff;
	    ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_DATA_BASE+3);
	    tmp_h = ret&0xff;
	    baseline_data = (tmp_h<<8)|tmp_l;
		#if 1
	    printk("%d,",baseline_data);
	if(tp_dev  == TP_TRULY){
			   if((baseline_data < Truly_array_limit[count*2]) || (baseline_data > Truly_array_limit[count*2+1])){
			   printk("===>wjw_tpbaseline_fail;count[%d][%d] =%d ;Truly_array_limit[count*2]=%d,Truly_array_limit[count*2+1]=%d\n ",count*2,(count*2+1),baseline_data,Truly_array_limit[count*2],Truly_array_limit[count*2+1]);
			        num_read_chars += sprintf(&(buf[num_read_chars]), "0 raw data erro baseline_data[%d][%d]=%d[%d,%d]\n",x,y,baseline_data,Truly_array_limit[count*2],Truly_array_limit[count*2+1]);
			        goto END;
			   }
	      }
	 else{
			  if((baseline_data < Ofilm_array_limit[count*2]) || (baseline_data > Ofilm_array_limit[count*2+1])){
			  printk("===>wjw_tpbaseline_fail;count[%d][%d] =%d ;Truly_array_limit[count*2]=%d,Truly_array_limit[count*2+1]=%d\n ",count*2,(count*2+1),baseline_data,Ofilm_array_limit[count*2],Ofilm_array_limit[count*2+1]);
			        num_read_chars += sprintf(&(buf[num_read_chars]), "0 raw data erro baseline_data[%d][%d]=%d[%d,%d]\n",x,y,baseline_data,Ofilm_array_limit[count*2],Ofilm_array_limit[count*2+1]);
			        goto END;
			   }
	      }
		  #endif
	    count++;
	  }
	  printk("\n");
   	}
	ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE+8,tmp_old);
	ret = i2c_smbus_write_word_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0x04);
	checkCMD();
	ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE+8);
	printk("[s2202]tem_new end = %x",ret&0xff);
	printk("step 2:check tx-to-tx and tx-to-vdd\n" );

    //step 2 :check tx-to-tx and tx-to-vdd
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_DATA_BASE,0x05);//select report type 0x05	 
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0X01);//get report
    checkCMD();
    ret = i2c_smbus_write_word_data(ts_g->client,F54_ANALOG_DATA_BASE+1,0);	
    tx_datal = i2c_smbus_read_word_data(ts_g->client,F54_ANALOG_DATA_BASE+3);
    tx_datah = i2c_smbus_read_word_data(ts_g->client,F54_ANALOG_DATA_BASE+3);
	tx_data = tx_datal | tx_datah<<16;
       if( tx_data!= 0)
	{
		printk("Step 2 error.\n");
	   num_read_chars += sprintf(buf, "0 tx-tx-short or tx-vdd-short");
	   goto END;
	}

  printk("step 3 :check rx-to-rx\n" );
 //step 3 :check rx-to-rx
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_DATA_BASE,7);//select report type 0x07
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE + 81,0X01); //forbid NoisMitigation 
	  //Forbid NoiseMitigation F54_ANALOG_CTRL41
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0X04);//force F54_ANALOG_CMD00
    checkCMD();                               //
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0X02);//Force Cal,F54_ANALOG_CMD00
    checkCMD();
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0X01);//get report
    checkCMD();
    ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE);//read report
	printk("F54_ANALOG_CMD00[2]=%d \n",ret);

	ret = i2c_smbus_write_word_data(ts_g->client,F54_ANALOG_DATA_BASE+1,0);
	for(x = 0;x < TX_NUM; x++)
   	{
      for(y = 0; y < RX_NUM; y++)
   	    {
	   
	         ret= i2c_smbus_read_word_data(ts_g->client,F54_ANALOG_DATA_BASE+3);
			 Rxdata[x][y] = ret&0xffff;
		        
		}
	}
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_DATA_BASE,17);//select report type 0x17 
    ret = i2c_smbus_write_word_data(ts_g->client,F54_ANALOG_DATA_BASE+1,0);
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0X01);//get report
    checkCMD();
	for(x = 0;x < RX_NUM - TX_NUM; x++)
   	{
      for(y = 0; y < RX_NUM; y++)
   	    {
	   
	         ret= i2c_smbus_read_word_data(ts_g->client,F54_ANALOG_DATA_BASE+3);
			 Rxdata[x + TX_NUM][y] = ret&0xffff;	 
		    // printk("\nRxdata[%d][%d] = %d",x+TX_NUM,y,Rxdata[x + TX_NUM][y]);   
		}
	}
	printk("\nstep 4:check rx-rx short\n");
//step 4:check rx-rx short
	if (tp_dev = TP_TRULY){
		for(x = 0;x < RX_NUM; x++)
	   	{
	      for(y = 0; y < RX_NUM; y++)
	   	  {
			  if ((x ==y)&&(x!=19))
			{
								 printk("check Rx-to-Rx and Rx-to-vdd_error,Rxdata[%d]=%d \n",x,Rxdata[x][y]);
	                if((Rxdata[x][y] < DiagonalLowerLimit)|| (Rxdata[x][y] >DiagonalUpperLimit))
	                   {
	                     num_read_chars += sprintf(buf, "0 rx-to-rx short or rx-to-vdd short Rxdata[%d][%d] = %d",x,y,Rxdata[x][y]);
						 printk("wjw_check Rx-to-Rx and Rx-to-vdd_error,Rxdata[%d]=%d \n",x,Rxdata[x][y]);
						 goto END;
					   }	
			 }
		   }
		 }
	}
	else{

	for(x = 0;x < RX_NUM; x++)
   	{
      for(y = 0; y < RX_NUM; y++)
   	  {
	    if ((x ==y)&&(x!=19))
		{
							 printk("check Rx-to-Rx and Rx-to-vdd_error,Rxdata[%d]=%d \n",x,Rxdata[x][y]);
              
                if((Rxdata[x][y] < DiagonalLowerLimit)|| (Rxdata[x][y] >DiagonalUpperLimit))
                   {
                     num_read_chars += sprintf(buf, "0 rx-to-rx short or rx-to-vdd short Rxdata[%d][%d] = %d",x,y,Rxdata[x][y]);
					 printk("wjw_check Rx-to-Rx and Rx-to-vdd_error,Rxdata[%d]=%d \n",x,Rxdata[x][y]);
					 goto END;
				   }	
				
		 }
	   }
	 }
	}
	num_read_chars += sprintf(buf, "1");
END:
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE+8,tmp_old);
	ret = i2c_smbus_write_word_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0x04);
	checkCMD();
	ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_CONTROL_BASE+8);
	printk("[s2202]tem_new end = %x",ret&0xff);
	printk("4 read F54_ANALOG_CTRL07 is: 0x%x\n",ret);
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0X02);
    delay_qt_ms(60);
    ret = i2c_smbus_write_byte_data(ts_g->client, 0xff, 0x00);
    ret = i2c_smbus_write_byte_data(ts_g->client, F01_RMI_CMD00,0x01);
    msleep(150);
    synaptics_init_panel(ts_g);
    synaptics_enable_interrupt(ts_g,1);
	mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	printk("\n\nstep5 reset and open irq complete\n");
    up(&work_sem);
	return num_read_chars;
	
}
static ssize_t tp_test_store(struct device_driver *ddri,
       char *buf, size_t count)
{
   printk("tp_test_store is not support\n");
   return count;
}
 static void re_scan_PDT(struct i2c_client *client)
 {
	 uint8_t buf[8];
	 i2c_smbus_read_i2c_block_data(client, 0xE9, 6,  buf);
	 SynaF34DataBase = buf[3];
	 SynaF34QueryBase = buf[0];
	 i2c_smbus_read_i2c_block_data(client, 0xE3, 6,  buf);
	 SynaF01DataBase = buf[3];
	 SynaF01CommandBase = buf[1];
	 i2c_smbus_read_i2c_block_data(client, 0xDD, 6,  buf);
	 SynaF34Reflash_BlockNum = SynaF34DataBase;
	 SynaF34Reflash_BlockData = SynaF34DataBase + 2;
	 SynaF34ReflashQuery_BootID = SynaF34QueryBase;
	 SynaF34ReflashQuery_FlashPropertyQuery = SynaF34QueryBase + 2;
	 SynaF34ReflashQuery_FirmwareBlockSize = SynaF34QueryBase + 3;
	 SynaF34ReflashQuery_FirmwareBlockCount = SynaF34QueryBase +5;
	 SynaF34ReflashQuery_ConfigBlockSize = SynaF34QueryBase + 3;
	 SynaF34ReflashQuery_ConfigBlockCount = SynaF34QueryBase + 7;
	 i2c_smbus_read_i2c_block_data(client, SynaF34ReflashQuery_FirmwareBlockSize,
		 2, buf);
	 SynaFirmwareBlockSize = buf[0] | (buf[1] << 8);
	 SynaF34_FlashControl = SynaF34DataBase + SynaFirmwareBlockSize + 2;
 
 }


 static int synatpitcs_ts_update(struct i2c_client *client)
{
    uint8_t data_low8,data_high8;
    uint16_t block,firmware,configuration;
	unsigned char uData;
	unsigned char uStatus;
    uint8_t buf[8];
    int ret,i,j;
	
    i2c_smbus_write_byte_data(client, 0xff, 0x0);	
	re_scan_PDT(client);
    block = 16;
    printk("block is %d \n",block);
    firmware = 2816;
    printk("firmware is %d \n",firmware);
    configuration = 32;
    printk("configuration is %d \n",configuration);

    //Enter Flash
    //step1 read bootloader Id
    data_low8 = i2c_smbus_read_byte_data(client, SynaF34ReflashQuery_BootID);
    data_high8 = i2c_smbus_read_byte_data(client,SynaF34ReflashQuery_BootID+1);
    printk("bootloader id is %x \n",(data_high8 << 8)|data_low8);
    //step 2 writer bootloader id
    i2c_smbus_write_byte_data(client,SynaF34Reflash_BlockData,data_low8);
    i2c_smbus_write_byte_data(client,SynaF34Reflash_BlockData+1,data_high8);
	//SynaWriteBootloadID();	
    printk("Write bootloader id\n");
    //step 3 Issue programe enable
    i2c_smbus_write_byte_data(client,SynaF34_FlashControl,0x0F);
    //step 4 wait attn
    //delay_qt_ms(20);
    msleep(10);
    //SynaWaitATTN();
    printk("attn step 4\n");
    //step 2 check status
    ret = i2c_smbus_read_byte_data(client,SynaF34_FlashControl);
    printk("The status(enter flash) is %x\n",ret);
    //step 6 Re-scan PDT
	re_scan_PDT(client);
    //program
    //step1 read bootloader id
    i2c_smbus_read_i2c_block_data(client,SynaF34ReflashQuery_BootID,2,buf);
     //step2 write bootloader id
    i2c_smbus_write_i2c_block_data(client,SynaF34Reflash_BlockData,2,buf);
    //step 3 erase
    i2c_smbus_write_byte_data(client,SynaF34_FlashControl,0x03);
    //step 4 wait attn
    //delay_qt_ms(500);
    msleep(500);
	kick_wdt();

    //step 5 check status, the value should be 0x80
    ret = i2c_smbus_read_byte_data(client,SynaF34_FlashControl);
    printk("The status(erase) is %x\n",ret);

    //step 6 image area
    for(j=0;j<firmware;j++)
    {
        //a) write block number
        buf[0]=j&0x00ff;
        buf[1]=(j&0xff00)>>8;
        i2c_smbus_write_i2c_block_data(client,SynaF34Reflash_BlockNum,2,buf);
        //b) write data
       // if(tp_dev == TP_TRULY)
        //for(i=0;i<block;i++)
        //{
	
               for(i=0;i<block;i++)	
               {			   
                  if(tp_dev == TP_TRULY)			   
                   i2c_smbus_write_byte_data(client,SynaF34Reflash_BlockData+i,
                   //TRULY_Firmware_Data[j][i]);
			       TRULY_Firmware_Data[j][i]);
				  if(tp_dev == TP_OFILM)			   
			       i2c_smbus_write_byte_data(client,SynaF34Reflash_BlockData+i,
                //TRULY_Firmware_Data[j][i]);
			       OFILM_Firmware_Data[j][i]);
			   }
	
        //}
    	//c) issue write
        i2c_smbus_write_byte_data(client,SynaF34_FlashControl,0x02);
        //d) wait attn
        //delay_qt_ms(20);
        msleep(5);
        //SynaWaitATTN();
        ret = i2c_smbus_read_byte_data(client,SynaF34_FlashControl);
        printk("The status(Image) of flash data3 is %x,time =%d\n",
            ret,j);
		

	   //if(j % 500 == 0){
		   kick_wdt();

	 //  	}
    }

		kick_wdt();

    //step 7 configure data
    for(j=0;j<configuration;j++)
    {
        //a)
        buf[0]=j&0x00ff;
        buf[1]=(j&0xff00)>>8;
        i2c_smbus_write_i2c_block_data(client,SynaF34Reflash_BlockNum,2,buf);
        //b) write data

		   
			    for(i=0;i<block;i++)
                {
				    if(tp_dev == TP_TRULY)
			       i2c_smbus_write_byte_data(client,SynaF34Reflash_BlockData+i,
                   TRULY_Config_Data[j][i]);
				    
					 if(tp_dev == TP_OFILM)
						i2c_smbus_write_byte_data(client,SynaF34Reflash_BlockData+i,OFILM_Config_Data[j][i]);
				   
                }				   
	
        //}
		
        //c) issue write
        i2c_smbus_write_byte_data(client,SynaF34_FlashControl,0x06);
        //d) wait attn
        //delay_qt_ms(20);
        msleep(5);
        ret = i2c_smbus_read_byte_data(client,SynaF34_FlashControl);
        printk("The status(Image) of flash data3 is %x,time =%d\n",
            ret,j);
			

    }
	kick_wdt();


    //disable
    //step 1 issue reset
    i2c_smbus_write_byte_data(client,SynaF01CommandBase,0X01);
    //step2 wait ATTN
    //delay_qt_ms(1000);
    msleep(500);
    //step 3 check status
    ret = i2c_smbus_read_byte_data(client,SynaF34_FlashControl);
    printk("The status(disable)of flash data3 is %0x\n",ret);
    //step 4 re-scan PDT
    //i2c_smbus_read_i2c_block_data(client, F34_FLASH_QUERY_BASE, 6,  buf);
    //i2c_smbus_read_i2c_block_data(client, F01_RMI_QUERY_BASE, 6,  buf);
    //i2c_smbus_read_i2c_block_data(client, F11_2D_QUERY_BASE, 6,  buf);

}


static ssize_t tp_update_show(struct device_driver *ddri, char *buf)
{
      int CURRENT_FIRMWARE_ID = 0;
	  uint8_t sbuf[4];
	  int bootloader_mode;
	  
	  synaptics_read_register_map(ts_g);	  
	  bootloader_mode = i2c_smbus_read_byte_data(ts_g->client,F01_RMI_DATA_BASE);
	  bootloader_mode = bootloader_mode&0xff;
	  printk("0x13 bootloader_mode = 0x%x\n",bootloader_mode);
	  i2c_smbus_read_i2c_block_data(ts_g->client, F34_FLASH_CTRL00, 4, sbuf); 
	  CURRENT_FIRMWARE_ID = (sbuf[0]<<24)|(sbuf[1]<<16)|(sbuf[2]<<8)|sbuf[3];
	  printk("read firmware ID: CURRENT_FIRMWARE_ID = %x\n",CURRENT_FIRMWARE_ID);
#if 0
      if(tp_dev  == TP_TRULY){  
			if((CURRENT_FIRMWARE_ID < FIRMWARE_ID_TRULY)||(bootloader_mode&0x40 == 1))
			{
				synatpitcs_ts_update(ts_g->client);  
				synaptics_read_register_map(ts_g);
				i2c_smbus_write_byte_data(ts_g->client, F01_RMI_CMD00,0x01);
				msleep(150);
			    bootloader_mode = i2c_smbus_read_byte_data(ts_g->client,F01_RMI_DATA_BASE);
	            bootloader_mode = bootloader_mode&0xff;
				i2c_smbus_read_i2c_block_data(ts_g->client, F34_FLASH_CTRL00, 4, sbuf); 
				CURRENT_FIRMWARE_ID = (sbuf[0]<<24)|(sbuf[1]<<16)|(sbuf[2]<<8)|sbuf[3];	
				TP_FW = CURRENT_FIRMWARE_ID;
				printk("read firmware ID: CURRENT_FIRMWARE_ID2 = %x\n",CURRENT_FIRMWARE_ID);
				if ((!(CURRENT_FIRMWARE_ID == FIRMWARE_ID_TRULY))||(bootloader_mode&0x40 == 1))
				sprintf(buf,"1 firmware update fail, Firmware ID = %x\n",CURRENT_FIRMWARE_ID);
				else
				sprintf(buf,"0 firmware update complete, Firmware ID = %x\n",CURRENT_FIRMWARE_ID);
		
			}
			else{
            i2c_smbus_read_i2c_block_data(ts_g->client, F34_FLASH_CTRL00, 4, sbuf); 
			CURRENT_FIRMWARE_ID = (sbuf[0]<<24)|(sbuf[1]<<16)|(sbuf[2]<<8)|sbuf[3];		
			printk("read firmware ID: CURRENT_FIRMWARE_ID2 = %x\n",CURRENT_FIRMWARE_ID);
			sprintf(buf,"2 The firmware is latest version,It's not necessary to update, Firmware ID = %x\n",CURRENT_FIRMWARE_ID);
			}
      	}
	    else{  
			if ((CURRENT_FIRMWARE_ID >= YF_UPDATE_FROME&&CURRENT_FIRMWARE_ID < YOUNGFAST_FIRMWARE_ID)||(bootloader_mode&0x40 == 1))
			{
				synatpitcs_ts_update(ts_g->client);  
				synaptics_read_register_map(ts_g);
				i2c_smbus_write_byte_data(ts_g->client, F01_RMI_CMD00,0x01);
				msleep(150);
			    bootloader_mode = i2c_smbus_read_byte_data(ts_g->client,F01_RMI_DATA_BASE);
	            bootloader_mode = bootloader_mode&0xff;
				i2c_smbus_read_i2c_block_data(ts_g->client, F34_FLASH_CTRL00, 4, sbuf); 
				CURRENT_FIRMWARE_ID = (sbuf[0]<<24)|(sbuf[1]<<16)|(sbuf[2]<<8)|sbuf[3];				
				TP_FW = CURRENT_FIRMWARE_ID;
				printk("read firmware ID: CURRENT_FIRMWARE_ID2 = %x\n",CURRENT_FIRMWARE_ID);
				if ((!(CURRENT_FIRMWARE_ID == YOUNGFAST_FIRMWARE_ID))||(bootloader_mode&0x40 == 1))
				sprintf(buf,"1 firmware update fail, Firmware ID = %x\n",CURRENT_FIRMWARE_ID);
				else
				sprintf(buf,"0 firmware update complete, Firmware ID = %x\n",CURRENT_FIRMWARE_ID);
		
			}
			else{
            i2c_smbus_read_i2c_block_data(ts_g->client, F34_FLASH_CTRL00, 4, sbuf); 
			CURRENT_FIRMWARE_ID = (sbuf[0]<<24)|(sbuf[1]<<16)|(sbuf[2]<<8)|sbuf[3];		
			printk("read firmware ID: CURRENT_FIRMWARE_ID2 = %x\n",CURRENT_FIRMWARE_ID);
			sprintf(buf,"2 The firmware is latest version,It's not necessary to update, Firmware ID = %x\n",CURRENT_FIRMWARE_ID);
			}
      	}
#endif


}

static ssize_t tp_update_store(struct device_driver *ddri,
       char *buf, size_t count)
{
	  printk("tp_test_store is not support\n");
	  return count;
}

static ssize_t tp_delta_show(struct device_driver *ddri, char *buf)
{

    int ret = 0;
	int x,y;
	ssize_t num_read_chars = 0;
	uint8_t tmp = 0;
	uint8_t tmp_old = 0,tmp_new = 0;
	uint8_t tmp_l = 0,tmp_h = 0; 
	uint16_t count = 0;
	int16_t delta_image[TX_NUM][RX_NUM] = {0};	
	int16_t baseline_data[TX_NUM][RX_NUM] = {0};

    mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);

	synaptics_read_register_map_page1(ts_g);
	
	printk("\nstep 1:select report type 0x03 baseline\n");
	 //step 1:check raw capacitance.
	ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_DATA_BASE,0x03);//select report type 0x03
	 if (ret < 0) {
		 TPD_DEBUG("read_baseline: i2c_smbus_write_byte_data failed \n");
		 //return sprintf(buf, "i2c err!");
	}
	ret = i2c_smbus_write_word_data(ts_g->client,F54_ANALOG_DATA_BASE+1,0x00);//set fifo 00
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0X01);//get report
	checkCMD();
	count = 0;
	for(x = 0;x < TX_NUM; x++)
   	{
   	
	  printk("\n[%d]",x);
      for(y = 0; y < RX_NUM; y++)
   	  {
	    ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_DATA_BASE+3);
	    tmp_l = ret&0xff;
	    ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_DATA_BASE+3);
	    tmp_h = ret&0xff;
	    baseline_data[x][y] = (tmp_h<<8)|tmp_l;
	    printk("%d,",baseline_data[x][y]);
	
	  }
   	}
	printk("\nstep 2:report type2 delta image\n");
	
	ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_DATA_BASE,0x02);//select report type 0x02
	ret = i2c_smbus_write_word_data(ts_g->client,F54_ANALOG_DATA_BASE+1,0x00);//set fifo 00
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0X01);//get report
	checkCMD();
	count = 0;
	for(x = 0;x < TX_NUM; x++)
	{
		printk("\n[%d]",x);
		 for(y = 0; y < RX_NUM; y++)
		 {
			ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_DATA_BASE+3);
			tmp_l = ret&0xff;
			ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_DATA_BASE+3);
			tmp_h = ret&0xff;
			delta_image[x][y]= (tmp_h<<8)|tmp_l;       		
			printk("%3d,",delta_image[x][y]);
		 }	 
	}

	END:

    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0X02);
    delay_qt_ms(60);
    synaptics_enable_interrupt(ts_g,1);
	mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);

}

static ssize_t tp_delta_store(struct device_driver *ddri,
       char *buf, size_t count)
{
	  printk("tp_test_store is not support\n");
	  return count;
}

static DRIVER_ATTR(oppo_tp_update,0664,tp_update_show,tp_update_store);

static DRIVER_ATTR(oppo_tp_test, 0664, tp_test_show, tp_test_store);

static DRIVER_ATTR(oppo_tp_delta_image, 0664, tp_delta_show, tp_delta_store);


static struct kobj_attribute mtk_virtual_keys_attr = {
    .attr = {
        .name = "virtualkeys.mtk-tpd",
        .mode = S_IRUGO,
    },
    .show = &cap_vk_show,
};

static struct attribute *mtk_properties_attrs[] = {
    &mtk_virtual_keys_attr.attr,
    NULL
};


static struct attribute_group mtk_properties_attr_group = {
    .attrs = mtk_properties_attrs,
};

static struct kobject *syna_properties_kobj;

static int synaptics_tpd_button_init(struct synaptics_ts_data *ts)
{
	int ret = 0;
    int i = 0;
	
	ts->kpd = input_allocate_device();
    	if (ts->kpd == NULL) 
	{
		ret = -ENOMEM;
		printk(KERN_ERR "synaptics_tpd_button_init: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	ts->kpd->name = TPD_DEVICE "-kpd";
    set_bit(EV_KEY, ts->kpd->evbit);
	__set_bit(KEY_MENU, ts->kpd->keybit);
	__set_bit(KEY_HOME, ts->kpd->keybit);
	__set_bit(KEY_BACK, ts->kpd->keybit);
	ts->kpd->id.bustype = BUS_HOST;
    ts->kpd->id.vendor  = 0x0001;
    ts->kpd->id.product = 0x0001;
    ts->kpd->id.version = 0x0100;
	
	if(input_register_device(ts->kpd))
        TPD_DEBUG("input_register_device failed.(kpd)\n");
    set_bit(EV_KEY, ts->kpd->evbit);
	__set_bit(KEY_MENU, ts->kpd->keybit);
	__set_bit(KEY_HOME, ts->kpd->keybit);
	__set_bit(KEY_BACK, ts->kpd->keybit);


	
    syna_properties_kobj = kobject_create_and_add("board_properties", NULL);
    if(syna_properties_kobj)
        ret = sysfs_create_group(syna_properties_kobj,&mtk_properties_attr_group);
    if(!syna_properties_kobj || ret)
    printk("failed to create board_properties\n");	
	
	err_input_dev_alloc_failed:		
    return ret;

}
static unsigned char TP_Detect_ID(void)
{	
    unsigned char bTP_Id_Detect=0;
    mt_set_gpio_mode(TP_ID_DETECT, 0); 
    mt_set_gpio_dir(TP_ID_DETECT, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(TP_ID_DETECT,GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(TP_ID_DETECT,1);
    bTP_Id_Detect = mt_get_gpio_in(TP_ID_DETECT);
	printk("wjw_TP_ID_DETECT:bTP_Id_Detect=%x \n",bTP_Id_Detect);
	return bTP_Id_Detect;

}
static int tp_write_func (struct file *file,const char *buffer, unsigned long count,void *data)
{
	struct synaptics_ts_data *ts; 
	int ret = 0;
	
	//return count;
	down(&work_sem);
	if(ts_g){
		ts = ts_g;
		TPD_DEBUG("tp_write_func: is be called! erroreport_handle = %d\n",ts->no_erroponit_exist);
	if(is_suspend){
		ts->no_erroponit_exist = 0;
		TPD_DEBUG("tp_write_func:tp is suspend\n");		
     	up(&work_sem);
		return count;
		
		}
    printk("tp_write_func:finger_num=%x \n",ts->finger_num);
	//if(ts->finger_num>=2)
	if(tp_multi_finger)
	{
        printk("tp_write_func:Handle erro point start!!!!!!!!!!! \n");
        tp_multi_finger=0;
	#ifndef VENDOR_EDIT
		//Fuchun.Liao@wxkfDrv,2013/05/14,delete for tp forcecal baseline in calling
		if(ts->no_erroponit_exist == 0)
	#endif /*VENDOR_EDIT*/
		{
			TPD_DEBUG("tp_write_func:find ERRO point!!!!!!!!!!\n  is_touch = %d, finger_num = %d\n",atomic_read(&is_touch),ts->finger_num);
#ifdef VENDOR_EDIT
//Fuchun.Liao@wxkfdDrv,2013/05/14,modify for tp reset
//forcecal baseline when report serval points
			ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x01); 
			ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_CMD00, 0x02); 
			ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00); 			
			if(ret) {
					printk("tp_write_func:tp_multi_finger,forcecal to take new baseline fail\n");
					goto err;
				}
					//ret = i2c_smbus_write_byte_data(ts->client, F11_2D_CMD00, 0x01); 
					//msleep(50);
#endif /*VENDOR_EDIT*/
				TPD_DEBUG("tp_write_func:tp_multi_finger,rezero successed!\n");

		}
	}
	if(!atomic_read(&is_touch)&&(!all_finger_up))
		{
			TPD_DEBUG("tp_write_func:when no touch rezero again!\n is_touch = %d, finger_num = %d\n",atomic_read(&is_touch),ts->finger_num);
			ts->no_erroponit_exist = 1;
#ifndef VENDOR_EDIT
//Fuchun.Liao@wxkfdDrv,2013/05/14,modify for tp reset
//reset pannel when no touch
			ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x01); 
			ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_CMD00, 0x02); 
			ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00);			
			if(ret) {
					printk("tp_write_func:tp_multi_finger,forcecal to take new baseline fail\n");
					goto err;
					}
					//ret = i2c_smbus_write_byte_data(ts->client, F11_2D_CMD00, 0x01); 
					//msleep(50);
#else
			ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CMD_BASE, 0x01);
			if(ret){
				     printk("tp_write_func: no touch,rezero error!\n");
					 goto err;
				}
			delay_qt_ms(70);
			ret = synaptics_init_panel(ts);		
			if (ret < 0){
				          printk("tp_write_func: no touch,synaptics_init_panel failed\n");				
			              goto err;		
						}		
			ret = synaptics_enable_interrupt(ts, 1);		
			if(ret){			
				     printk("tp_write_func:no touch,cannot enable interrupt \n");	        
			         goto err;		
					}
#endif /*VENDOR_EDIT*/			

			TPD_DEBUG("tp_write_func: no touch,rezero successed!\n");
		  }
		}
	
	TPD_DEBUG("tp_write_func:no erro point exist!\n is_suspend = %d, is_touch = %d, finger_num = %d\n",is_suspend,atomic_read(&is_touch),ts->finger_num);
	err:
	up(&work_sem);
	return count;
}

static int init_synaptics_proc(void)
{
	int ret = 0;
	
	prEntry = create_proc_entry( "synaptics_reset", 0666, NULL );
	if(prEntry == NULL)
	{
		ret = -ENOMEM;
	  	printk(KERN_INFO"init_synaptics_proc: Couldn't create proc entry\n");
	}
	else
	{
		prEntry->write_proc = tp_write_func;
	}
	
	return ret;
}
#ifdef VENDOR_EDIT
//Fuchun.Liao@wxkfDrv,2013/04/22,add for modem version detect by GPIO
char *modem_ver_buffer[1]={0};

struct proc_dir_entry *modem_ver_dir, *modem_ver_file;
 
int proc_read_modem_ver(char *page, char **start, off_t off, int count, int *eof,void *data) 
{
        int len;
        len = sprintf(page, modem_ver_buffer[0]); 
        printk("lfc proc_read_modem_ver,modem_ver_buffer[0]:%s,page:%s\n",modem_ver_buffer[0],page);
        return len;
}

int proc_wirte_modem_ver(struct file *file, const char *buffer, unsigned long count,void *data) 
{
      printk("lfc not support proc_write_modem_ver\n");
        return count;
}

static int __init proc_modem_ver_init(void) {
        modem_ver_dir = proc_mkdir("modem_ver", NULL);
        modem_ver_file = create_proc_entry("modem_ver_detect", S_IRUGO, modem_ver_dir);
       //strcpy(modem_ver_buffer, "INIT");
        modem_ver_file->read_proc = proc_read_modem_ver;
        modem_ver_file->write_proc = proc_wirte_modem_ver;
        return 0;
}
#endif /*VENDOR_EDIT*/
static int synaptics_ts_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	struct synaptics_ts_data *ts;
	int ret = 0;
	uint32_t FIRMWARE_ID_TRULY;
	uint32_t FIRMWARE_ID_OFILM;
    uint32_t CURRENT_FIRMWARE_ID = 0;
	uint8_t buf[4];
	uint8_t bootloader_mode = 0;
	uint8_t fw_cnt = 0;
	//bTP_Id_Detect_probe=TP_Detect_ID();
	#ifdef VENDOR_EDIT
	//Fuchun.Liao@wxkfDrv,2013/04/21,add for modem version detect by GPIO
	int tmp0,tmp1;
	mt_set_gpio_mode(GPIO132,0); 
	mt_set_gpio_dir(GPIO132,0);
	mt_set_gpio_pull_enable(GPIO132, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO132, GPIO_PULL_UP);
	mt_set_gpio_mode(GPIO133, 0); 
	mt_set_gpio_dir(GPIO133,0);
	mt_set_gpio_pull_enable(GPIO133, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO133, GPIO_PULL_UP);
	
	mdelay(5);
    tmp0=mt_get_gpio_in(GPIO132);
    tmp1=mt_get_gpio_in(GPIO133);
	if(tmp0==1 && tmp1==1)
	modem_ver_buffer[0]="THAI";
	else if(tmp0==1 && tmp1==0)
	modem_ver_buffer[0]="AS";
	else
	modem_ver_buffer[0]="UNKNOWN";
	printk("lfc modem ver detect GPIO132=%d,GPIO133=%d,modem_ver=%s\n",tmp0,tmp1,modem_ver_buffer[0]);
    proc_modem_ver_init();
	#endif /*VENDOR_EDIT*/
	TPD_DEBUG("synaptics_ts_probe: enter !!!!!!!!!!!!!!\n");
	tpd_power(1);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		TPD_DEBUG("synaptics_ts_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}
    printk("wjw_yf_ts_probe_synaptics_ts_probe \n");
	
    ret = i2c_smbus_read_byte_data(client,0x13);
	if(ret < 0)
	{
		ret = i2c_smbus_read_byte_data(client,0x13);
		if(ret < 0)
		{
			tpd_power(0);
			printk("synaptics is no exist!\n");
			return 0;
		}
	}
	
	init_synaptics_proc();
	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}
	memset(ts,0,sizeof(*ts));
	INIT_WORK(&ts->work, synaptics_ts_work_func);
	ts->client = client;
	i2c_set_clientdata(client, ts);
	/*disable interrupt*/
	get_id();
	synaptics_read_register_map(ts);
	bootloader_mode = i2c_smbus_read_byte_data(ts->client,F01_RMI_DATA_BASE);
	bootloader_mode = bootloader_mode&0xff;
	bootloader_mode = bootloader_mode&0x40; 
	printk("synaptics:before fw update,bootloader_mode = 0x%x\n",bootloader_mode);
	i2c_smbus_read_i2c_block_data(ts->client, F34_FLASH_CTRL00, 4, buf); 
	CURRENT_FIRMWARE_ID = (buf[0]<<24)|(buf[1]<<16)|(buf[2]<<8)|buf[3];
	TP_FW=CURRENT_FIRMWARE_ID;	
	if(tp_dev == TP_TRULY){
		printk("TP_DEV = TP_TRULY\n");
	buf[0]=TRULY_Config_Data[0][0];
	buf[1]= TRULY_Config_Data[0][1];
	buf[2]= TRULY_Config_Data[0][2];
	buf[3]=TRULY_Config_Data[0][3];
	FIRMWARE_ID_TRULY = (buf[0]<<24)|(buf[1]<<16)|(buf[2]<<8)|buf[3];
	
	printk("read firmware ID: CURRENT_FIRMWARE_ID = %x,LOCAL_FIRMWARE_ID = %x\n",CURRENT_FIRMWARE_ID,FIRMWARE_ID_TRULY);
	}else if(tp_dev == TP_OFILM)
	{
		printk("TP_DEV = TP_OFILM\n");
		buf[0]=OFILM_Config_Data[0][0];
		buf[1]= OFILM_Config_Data[0][1];
		buf[2]= OFILM_Config_Data[0][2];
		buf[3]=OFILM_Config_Data[0][3];
		FIRMWARE_ID_OFILM= (buf[0]<<24)|(buf[1]<<16)|(buf[2]<<8)|buf[3];
		printk("read firmware ID: CURRENT_FIRMWARE_ID = %x,LOCAL_FIRMWARE_ID = %x\n",CURRENT_FIRMWARE_ID,FIRMWARE_ID_OFILM);
	}else
	{
		printk("check TP type erro!!!!!!!!!\n");
		goto synaptics_ts_probe_end;
	}
	printk("synaptics_ts_probe: boot_mode = %d\n",boot_mode);
	if(boot_mode == NORMAL_BOOT)

	{
	    printk("synaptics_ts_probe:FW update start... \n");
		if(tp_dev == TP_TRULY)
		{
			if(CURRENT_FIRMWARE_ID == FIRMWARE_ID_TRULY)
				
		    goto after_fw_update;
						
				
				}
		else
		{
		   if(CURRENT_FIRMWARE_ID == FIRMWARE_ID_OFILM)
		    goto after_fw_update;
		}
fw_update:
					synatpitcs_ts_update(ts->client);  			
					synaptics_read_register_map(ts);
					i2c_smbus_read_i2c_block_data(ts->client, F34_FLASH_CTRL00, 4, buf); 
					CURRENT_FIRMWARE_ID = (buf[0]<<24)|(buf[1]<<16)|(buf[2]<<8)|buf[3];
		fw_cnt++;
		printk("after FW upate times fw_cnt = %d\n\
			, CURRENT_FIRMWARE_ID2 = %x,\n",fw_cnt,CURRENT_FIRMWARE_ID);
			        TP_FW=CURRENT_FIRMWARE_ID;
				}
after_fw_update:
	ret = synaptics_enable_interrupt(ts, 0);
	if(ret < 0) {
		TPD_DEBUG(" synaptics_ts_probe: disable interrupt failed\n");
	}
	
	/*read product id */
	ret = synaptics_read_product_id(ts);
	if(ret) {
		TPD_DEBUG("failed to read product info \n");
	}

	/*read max_x ,max_y*/
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0);
	if (ret < 0) {
		TPD_DEBUG("i2c_smbus_write_byte_data failed for page select\n");
	}

	ret = i2c_smbus_read_word_data(ts->client,F11_2D_CTRL06
	);
	if(ret < 0)
	{
		TPD_DEBUG("i2c_smbus_read_word_data failed for max_x\n");
	}
	
	ts->max_x = ret&0xfff;
	
    ret = i2c_smbus_read_word_data(ts->client,F11_2D_CTRL08);
    if(ret < 0)
    {
		TPD_DEBUG("i2c_smbus_read_word_data failed for max_y\n");
	}
	ts->max_y = ret&0xfff;
	
	bootloader_mode = i2c_smbus_read_byte_data(ts->client,F01_RMI_DATA_BASE);
	bootloader_mode = bootloader_mode&0xff;
	bootloader_mode = bootloader_mode&0x40; 
	printk("synaptics:afte fw update,bootloader_mode = 0x%x\n",bootloader_mode);
	printk("synaptics max_x = %d\n max_y = %d\n",ts->max_x,ts->max_y);
	if((ts->max_x == 0)||(ts->max_y == 0)||(bootloader_mode == 0x40)){
		if(fw_cnt < 2)
		goto fw_update;
	}
	
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) 
	{
		ret = -ENOMEM;
		printk(KERN_ERR "synaptics_ts_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	ts->input_dev->name = TPD_DEVICE;
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(ABS_MT_TOUCH_MAJOR, ts->input_dev->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR,ts->input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, ts->input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, ts->input_dev->absbit);
	set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);
	
	/* For multi touch */
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR,
			     0, 255, 0, 0);
   
      input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X,
		       0, ts->max_x, 0, 0);

    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y,
		        0, 1880, 0, 0);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(KEY_SEARCH, ts->input_dev->keybit);
	set_bit(KEY_MENU, ts->input_dev->keybit);
	set_bit(KEY_HOME, ts->input_dev->keybit);
	set_bit(KEY_BACK, ts->input_dev->keybit);
	input_set_drvdata(ts->input_dev, ts);
	
	if(input_register_device(ts->input_dev))
	{
		printk(KERN_ERR "synaptics_ts_probe: Failed to register input device\n");
		goto err_input_dev_register;
	}
    synaptics_tpd_button_init(ts);

#if 0
	set_bit(EV_SYN, tpd->dev->evbit);
	set_bit(KEY_SEARCH, tpd->dev->keybit);
	set_bit(KEY_MENU, tpd->dev->keybit);
	set_bit(KEY_HOME, tpd->dev->keybit);
	set_bit(KEY_BACK, tpd->dev->keybit);
	
	TPD_RES_X = SYNAPTICS_MAX_X;
	TPD_RES_Y = SYNAPTICS_MAX_Y-129; /*129 bottom room reserved for virtual key*/
#endif

#ifdef TPD_USE_EINT
	ts_g = ts;
	mt_set_gpio_mode(TOUCHPANEL_INT_PIN, GPIO_MODE_01);
	mt_set_gpio_dir(TOUCHPANEL_INT_PIN,GPIO_DIR_IN);
	mt_set_gpio_pull_enable(TOUCHPANEL_INT_PIN, GPIO_PULL_ENABLE);

	mt_set_gpio_pull_select(TOUCHPANEL_INT_PIN, GPIO_PULL_UP);
	mt65xx_eint_set_polarity(CUST_EINT_TOUCH_PANEL_NUM,CUST_EINT_POLARITY_LOW);
	mt65xx_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM,CUST_EINT_LEVEL_SENSITIVE);//level low pull up
	mt65xx_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, 
			(CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN >> 4));
	mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, 
					CUST_EINT_DEBOUNCE_DISABLE, 
					CUST_EINT_POLARITY_LOW, 
					synaptics_ts_irq_handler, 
					0);
	
	
	mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);

	ret = synaptics_enable_interrupt(ts, 1);
	
	if(ret) {
		TPD_DEBUG("synaptics_ts_probe: failed to enable synaptics  interrupt \n");
		free_irq(client->irq, ts);
		goto err_power_failed;
	}
	
	/*config tm1429: set report rate, sleep mode */
	ret = synaptics_init_panel(ts); /* will also switch back to page 0x04 */
	if (ret < 0) {
		TPD_DEBUG("synaptics_init_panel failed\n");	
	}

	ts->use_irq = 1;
#else
	hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ts->timer.function = synaptics_ts_timer_func;
	hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
#endif
   	hrtimer_init(&ts->report_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ts->report_timer.function = synaptics_ts_report_timer_func;
	//hrtimer_start(&ts->report_timer, ktime_set(1, 0), HRTIMER_MODE_REL);

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = synaptics_ts_early_suspend;
	ts->early_suspend.resume = synaptics_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif
	
	if (driver_create_file(&synaptics_driver.driver, &driver_attr_oppo_tp_debug))
	{            
		TPD_DEBUG("driver_create_file failt\n");
		goto exit_init_failed;
	}

	if (driver_create_file(&synaptics_driver.driver, &driver_attr_oppo_tp_test))
	{           
		TPD_DEBUG("driver_create_file failt\n");
		
		goto exit_init_failed;
	
	}	
	if (driver_create_file(&synaptics_driver.driver, &driver_attr_oppo_tp_update))
	{           
		TPD_DEBUG("driver_create_file failt\n");
		
		goto exit_init_failed;
	
	}
	if (driver_create_file(&synaptics_driver.driver, &driver_attr_oppo_tp_delta_image))
	{           
		TPD_DEBUG("driver_create_file failt\n");
		
		goto exit_init_failed;
	
	}	
	return 0;

exit_init_failed:
err_detect_failed:
err_power_failed:
	input_unregister_device(ts->input_dev);
err_input_dev_register:
	input_free_device(ts->input_dev);
err_input_dev_alloc_failed:
	kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:
synaptics_ts_probe_end:
#ifdef VENDOR_EDIT//hsy@oppo.com, add 2011/12/14 for tpd power off
	tpd_power(0);
#endif/*VENDOR_EDIT*/
	return ret;
}

static int __devexit synaptics_ts_remove(struct i2c_client *client)
{
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif
	
#ifdef TPD_USE_EINT
		mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
#else
		hrtimer_cancel(&ts->timer);
#endif	
	input_unregister_device(ts->input_dev);
	input_free_device(ts->input_dev);
	kfree(ts);
    tpd_hw_pwroff();
	return 0;
}

static int synaptics_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
	printk("synaptics_ts_suspend is called\n");
    is_suspend = 1;
	ts->no_erroponit_exist = 0;
	down(&work_sem);
#ifdef TPD_USE_EINT
	mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
#else
	hrtimer_cancel(&ts->timer);
#endif
	ret = synaptics_enable_interrupt(ts, 0);
	if(ret) {
		TPD_DEBUG("synaptics_ts_suspend: cannot disable interrupt\n");
		goto err;
	}
	ret = cancel_work_sync(&ts->work);
	if(ret) {
		TPD_DEBUG("synaptics_ts_suspend: cannot disable work\n");
	}

	ret = synaptics_enable_interrupt(ts, 0);
	if(ret) {
		TPD_DEBUG("synaptics_ts_suspend: can not disable \
		    synaptics_enable_interrupt\n");
	}
	
	ret = i2c_smbus_write_byte_data(client, F01_RMI_CTRL00, 0x01); 
	if (ret < 0) {
		TPD_DEBUG("synaptics_ts_suspend: control tm1400 to sleep failed\n");
		goto err;
	}
   	up(&work_sem);
	return 0;
   	err:
   	up(&work_sem);
	return -1;

}

static int synaptics_ts_resume(struct i2c_client *client)
{
	int ret;
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
	printk("synaptics_ts_resume is called\n");
    down(&work_sem);
	tpd_up(ts,0, 0, 0, 0, 0);
	input_sync(ts->input_dev);
	all_finger_up = 0;

	if (!ts->use_irq) {
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	} else {

        synaptics_init_panel(ts);
		if (ret < 0) {
			TPD_DEBUG("synaptics_ts_resume: control tm1400 to sleep failed\n");
			goto err;
		}
		ret = synaptics_enable_interrupt(ts, 1);
		if(ret) {
			TPD_DEBUG("synaptics_ts_resume:cannot  enable interrupt \n");
			goto err;
		}
    
        is_suspend = 0;
		#ifdef TPD_USE_EINT
		mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);    
        #endif
	}
		up(&work_sem);
	return 0;
		err:
		up(&work_sem);
	return -1;

}

/*----------------------------------------------------------------------------*/
#ifdef CONFIG_HAS_EARLYSUSPEND
static void synaptics_ts_early_suspend(struct early_suspend *h)
{
	printk("synaptics_ts_early_suspend is called\n");
	struct synaptics_ts_data *ts;
	ts = container_of(h, struct synaptics_ts_data, early_suspend);
	synaptics_ts_suspend(ts->client, PMSG_SUSPEND);
	tpd_power(0);
	if(boot_mode == FACTORY_BOOT)
	{
	    printk("shirendong codec close!!\n");
		mt_set_gpio_mode(GPIOEXT22, 0);
		mt_set_gpio_dir(GPIOEXT22, 1);
		mt_set_gpio_out(GPIOEXT22, 0);
	}
	mt_set_gpio_mode(138, 0); 
	mt_set_gpio_dir(138,0);
	mt_set_gpio_pull_enable(138, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(138, GPIO_PULL_DOWN);
}

static void synaptics_ts_late_resume(struct early_suspend *h)
{
	printk("synaptics_ts_late_resume is called\n");
	struct synaptics_ts_data *ts;
	ts = container_of(h, struct synaptics_ts_data, early_suspend);
	tpd_power(1);
	
	#ifdef VENDOR_EDIT
	//Fuchun.Liao@wxkfDrv,2013/05/07 add for tp i2c error after resume
	msleep(20);
	#endif /*VENDOR_EDIT*/
	
	//yongjun.wu@Prodrv,add for unlock screen two times
	input_report_key(ts->input_dev, BTN_TOUCH, 0);
	input_mt_sync(ts->input_dev);	
	input_sync(ts->input_dev);
	printk("synaptics_ts_late_resume:tpd_power ok\n");
		mt_set_gpio_mode(138, 0); 
	mt_set_gpio_dir(138,0);
	mt_set_gpio_pull_enable(138, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(138, GPIO_PULL_UP);
	synaptics_ts_resume(ts->client);
}
#endif

/*----------------------------------------------------------------------------*/
static const struct i2c_device_id synaptics_ts_id[] = {
	{ TPD_DEVICE, 0 },
	{ }
};

/*static unsigned short synaptics_force[] = {0x00, 0x40, 
    I2C_CLIENT_END, I2C_CLIENT_END};
static const unsigned short *const synaptics_forces[] = { synaptics_force,
    NULL };
static struct i2c_client_address_data synaptics_addr_data = {
    .forces = synaptics_forces,
    };
*/ 
static struct i2c_board_info __initdata synaptics_i2c_info ={ I2C_BOARD_INFO(TPD_DEVICE, (0x40>>1))};
/*----------------------------------------------------------------------------*/
static int synaptics_ts_detect(struct i2c_client *client,
    int kind, struct i2c_board_info *info)
{         
    strcpy(info->type, TPD_DEVICE);                                                         

	TPD_DEBUG("synaptics-rmi-ts driver_detect !!\n ");
	
    return 0;                                                                                       
}

/*----------------------------------------------------------------------------*/
static struct i2c_driver tpd_i2c_driver = {
	.probe		= synaptics_ts_probe,
	.remove		= __devexit_p(synaptics_ts_remove),
	//.detect         = synaptics_ts_detect,                           
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= synaptics_ts_suspend,
	.resume		= synaptics_ts_resume,
#endif
	.id_table	= synaptics_ts_id,
	//.address_data = &synaptics_addr_data,
	.driver = {
		//.owner    = THIS_MODULE,
		.name	= TPD_DEVICE,
	},
};

static int synaptics_probe(struct platform_device *pdev) 
{

   	boot_mode = get_boot_mode();
	printk("boot_mode = %d ............\n",boot_mode);
	if(boot_mode == META_BOOT)
		return 0;
	// Software reset mode will be treated as normal boot
	if(boot_mode==3) boot_mode = NORMAL_BOOT;
	if(boot_mode==7) boot_mode = NORMAL_BOOT;
    synaptics_wq = create_singlethread_workqueue("synaptics_wq");
    if (!synaptics_wq) {
        return -ENOMEM;
    }   
    
    if(i2c_add_driver(&tpd_i2c_driver)!=0) {
        TPD_DEBUG("unable to add i2c driver.\n");
        return -1;
    }	
    return 0;
}
static int __devexit synaptics_remove(struct platform_device *pdev)
{
	i2c_del_driver(&tpd_i2c_driver);
	if (synaptics_wq)
		destroy_workqueue(synaptics_wq);
}



/* called when loaded into kernel */
static int __init tpd_driver_init(void) {
	i2c_register_board_info(0, &synaptics_i2c_info, 1);
	if(platform_driver_register(&synaptics_driver)!=0) {
		printk("unable to register touch panel driver.\n");
		return -1;
	}	
	return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void) {
  platform_driver_unregister(&synaptics_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);

MODULE_DESCRIPTION("Synaptics Touchscreen Driver");
MODULE_LICENSE("GPL");

