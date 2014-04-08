/************************************************************************************
** File: - mediatek\custom\common\kernel\touchpanel\synaptics\synaptics_tm1347_i2c_rmi.c
** OPPO_R819
** Copyright (C), 2008-2012, OPPO Mobile Comm Corp., Ltd
** 
** Description: 
**      touch panel driver for synaptics
**      can change MAX_POINT_NUM value to support multipoint
** Version: 1.0
** Date created: 10:49:46,18/01/2012
** Author: Yixue.Ge@BasicDrv.TP
** 
** --------------------------- Revision History: --------------------------------
** 	<author>	<data>			<desc>
** Yixue.Ge@BasicDrv.TP 14/02/2012 add some 11091 code
** Yongjun.Wu@BasicDrv.TP 04/06/2012 modify for tp power reset pin
** Yongjun.Wu@BasicDrv.TP 30/06/2012 modify driver adapt deferent firmware 
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

#include "SynaFirmwareImage_3202_black.h"

#include <linux/oppo_devices_list.h>


#ifndef TPD_NO_GPIO 
#include "cust_gpio_usage.h"
#endif

#include <linux/proc_fs.h>
#define TPD_DEVICE "mtk-tpd3202-black"
#define TOUCHPANEL_INT_PIN 124
#define  CUST_EINT_TOUCH_PANEL_NUM 5
#ifdef OPPO_R819//huqiao modify by black touchpanel buttonled
#include <mach/pmic_mt6329_hw_bank1.h> 
#include <mach/pmic_mt6329_sw_bank1.h> 
#include <mach/pmic_mt6329_hw.h>
#include <mach/pmic_mt6329_sw.h>
#include <mach/upmu_common_sw.h>
#include <mach/upmu_hw.h>
#endif
static unsigned int tp_debug = 0;

#define TPD_DEBUG(a,arg...)\
	do{\
		if(tp_debug)\
			printk(TPD_DEVICE ": " a,##arg);\
	}while(0)
	
static struct workqueue_struct *synaptics_wq = NULL;
static struct proc_dir_entry *prEntry = NULL; 
static struct proc_dir_entry *prEntry_double = NULL; 

//int Disable_key_during_touch =0;
static unsigned int boot_mode = 0;
static unsigned int is_suspend = 0;
static DEFINE_SEMAPHORE(work_sem);
static unsigned int tp_multi_finger=0; 
static atomic_t is_touch = ATOMIC_INIT(0);
static unsigned int all_finger_up = 0;
static int err_report_num = 0;
static int reset_staus = 0;

extern TP_DEV tp_dev;
extern int TP_FW;
//static int tp_dev;
//static int TP_FW;
#ifndef OPPO_R819
//Fuchun.Liao@wxkfDrv,modify for engineermode display
#define TP_TRULY 1
#define TP_YOUNGFAST 0
#endif /*OPPO_R819*/



#define TP_TEST_PASS 1
#define TP_TEST_FAIL 0
#define TPDTM_DMESG(a,arg...) printk(TPD_DEVICE "(tm1347): " a,##arg)
#define SYNAPTICS_NAME       "synaptics-tm1347"
#define TRULY_FIRMWARE_ID  0x30303032
#define YF_FW_LIMIT        0x30303532
#define YF_FIRMWARE_ID    0x30303535

#define SUPPORT_DOUBLE_TAP

#ifdef SUPPORT_DOUBLE_TAP
	static atomic_t double_tap_number;
	static atomic_t double_tap_enable;
	#define KEY_POWER 116
	extern int whether_in_approach_state();
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void synaptics_ts_early_suspend(struct early_suspend *h);
static void synaptics_ts_late_resume(struct early_suspend *h);
#endif



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
static int F11_2D_CTRL32;
static int F11_2D_DATA01;
static int F11_2D_CMD00;
static int F01_RMI_CMD00;
static int F34_FLASH_CTRL00;

static int F54_ANALOG_QUERY_BASE;//0x73
static int F54_ANALOG_COMMAND_BASE;//0x72
static int F54_ANALOG_CONTROL_BASE;//0x0d
static int F54_ANALOG_DATA_BASE;//0x00

//extern void update_lcd();
#define VKNUMBER 3
static int max_x = 0;
static int max_y = 0;

static int tpd_keys[VKNUMBER][5] = {
	{KEY_MENU,90, 2050,180,100},
	{KEY_HOME,500,2050,180,100},
	{KEY_BACK,855,2050,180,100},
	};
static int tpd_keys1[VKNUMBER][5] ={
	{KEY_MENU,120,1830,92,60},
	{KEY_HOME,507,1830,90,60},
	{KEY_BACK,885,1830,90,60},
	};


#define MAX_POINT_NUM      10//must <=10
struct point_info {
    int x;
    int raw_x;
    int y;
    int raw_y;
    int z;
};

//typedef struct point_info points[MAX_POINT_NUM];

struct synaptics_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;	
	struct input_dev *kpd;
	int use_irq;
	struct hrtimer timer;
	struct hrtimer report_timer;
	struct work_struct  work;
    uint32_t pre_finger_state;
	uint32_t max_x;
    uint32_t max_y;
	uint32_t flags;
	uint32_t btn_state;
	uint32_t no_erroponit_exist;	
	uint32_t finger_num;
	struct early_suspend early_suspend;
};

static void synaptics_software_rezero(struct synaptics_ts_data *ts);
//extern struct TP_AUTOTEST *tp_autotest;

static ssize_t cap_vk_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf){
      /* LEFT: search: CENTER: menu ,home:search 412, RIGHT: BACK */
      return sprintf(buf,
        	__stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":130:1424:140:226"
         ":" __stringify(EV_KEY) ":" __stringify(KEY_HOME)   ":360:1424:140:226"
         ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":600:1424:140:226"
         "\n");

}
static struct kobj_attribute mtk_virtual_keys_attr = {
    .attr = {
        .name = "virtualkeys.mtk-tpd3202-black",
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
	ts->kpd->name = "mtk-tpd3202-kpd";
    set_bit(EV_KEY, ts->kpd->evbit);
	__set_bit(KEY_MENU, ts->kpd->keybit);
	__set_bit(KEY_HOME, ts->kpd->keybit);
	__set_bit(KEY_BACK, ts->kpd->keybit);
	ts->kpd->id.bustype = BUS_HOST;
    ts->kpd->id.vendor  = 0x0001;
    ts->kpd->id.product = 0x0001;
    ts->kpd->id.version = 0x0100;
	
	if(input_register_device(ts->kpd))
        TPDTM_DMESG("input_register_device failed.(kpd)\n");
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



static int synaptics_probe(struct platform_device *pdev);
static int synaptics_remove(struct platform_device *pdev);


static struct platform_driver synaptics_driver = {
    .remove     = synaptics_remove,
    //.shutdown   = NULL,
    .probe      = synaptics_probe,
    .driver     = {
    //	.owner = THIS_MODULE,
        .name = "synaptics-platform-black",
    },
};

#define TPD_USE_EINT

#ifdef TPD_USE_EINT
/*
#ifndef CUST_EINT_TOUCH_PANEL_NUM
#define TOUCHPANEL_INT_PIN GPIO1
#define CUST_EINT_TOUCH_PANEL_NUM              2
#define CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN      0x0
#define CUST_EINT_TOUCH_PANEL_POLARITY         CUST_EINT_POLARITY_HIGH
#define CUST_EINT_TOUCH_PANEL_SENSITIVE        CUST_EINT_EDGE_SENSITIVE
#define CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN   CUST_EINT_DEBOUNCE_DISABLE
#else
#define TOUCHPANEL_INT_PIN GPIO_CTP_EINT_PIN
#endif
*/

#define TOUCHPANEL_INT_PIN GPIO124
#endif

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
        TPD_DEBUG("D[%4d %4d %4d]\n", x, y, p);
    }  
}

static void tpd_up(struct synaptics_ts_data *ts,
    int raw_x, int raw_y, int x, int y, int p) {
	 if(ts && ts->input_dev) {
       // input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, p);
       // input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, (raw_x+raw_y)/2);
       // input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
        //input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
         input_report_key(ts->input_dev, BTN_TOUCH, 0);
        input_mt_sync(ts->input_dev);
	    all_finger_up = 0;
        TPD_DEBUG("U[%4d %4d %4d]\n", x, y, 0);
    }  
}

static void tpd_button(struct synaptics_ts_data *ts,
    unsigned int x, unsigned int y, unsigned int down) {
    int i;
	if(max_x == 1145)
	{
	    if(down) {
	        for(i=0;i<VKNUMBER;i++) {
	            if(x>=tpd_keys[i][1]&&
	               x<=tpd_keys[i][1]+tpd_keys[i][3] &&
	               y>=tpd_keys[i][2]&&
	               y<=tpd_keys[i][2]+tpd_keys[i][4]&&
	               !(ts->btn_state&(1<<i))) {
	                input_report_key(ts->input_dev, tpd_keys[i][0], 1);
	                ts->btn_state|=(1<<i);
	            }
	        }
	    } else {
	        for(i=0;i<4;i++) {
	            if(ts->btn_state&(1<<i)) {
	                input_report_key(ts->input_dev, tpd_keys[i][0], 0);
	            }
	        }
	        ts->btn_state=0;
	    }
	}else{
	    if(down) {
	        for(i=0;i<VKNUMBER;i++) {
	            if(x>=tpd_keys1[i][1]&&
	               x<=tpd_keys1[i][1]+tpd_keys1[i][3] &&
	               y>=tpd_keys1[i][2]&&
	               y<=tpd_keys1[i][2]+tpd_keys1[i][4]&&
	               !(ts->btn_state&(1<<i))) {
	                input_report_key(ts->input_dev, tpd_keys1[i][0], 1);
	                ts->btn_state|=(1<<i);
	            }
	        }
	    } else {
	        for(i=0;i<4;i++) {
	            if(ts->btn_state&(1<<i)) {
	                input_report_key(ts->input_dev, tpd_keys1[i][0], 0);
	            }
	        }
	        ts->btn_state=0;
    }
	}
}

static void tpd_hw_pwron(void)
{
    hwPowerOn(MT65XX_POWER_LDO_VGP6,VOL_2800,"CTP");
	msleep(1);
    hwPowerOn(MT65XX_POWER_LDO_VGP4,VOL_1800,"CTP");
	msleep(250);//hsy@oppo.com, add 2011/9/9 to speed up screen resume
}
static void tpd_hw_pwroff(void)
{
	hwPowerDown(MT65XX_POWER_LDO_VGP4,"TP");
	msleep(1);
    hwPowerDown(MT65XX_POWER_LDO_VGP6,"TP");
}

static void tpd_power(unsigned int on)
{
		
		if(on)
			{
				tpd_hw_pwron();
			}
			else
			{
				tpd_hw_pwroff();
			}		
}




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
		F11_2D_CTRL32 = F11_2D_CTRL_BASE + 15;
		F11_2D_DATA01 = F11_2D_DATA_BASE + 2;
		F11_2D_CMD00 = F11_2D_CMD_BASE;
		F01_RMI_CMD00 = F01_RMI_CMD_BASE;
		F34_FLASH_CTRL00 = F34_FLASH_CTRL_BASE;
}

#ifdef SUPPORT_DOUBLE_TAP
static int synaptics_enable_interrupt_for_double_tap(struct synaptics_ts_data *ts,
    int enable)
{

	int ret;
	uint8_t status_int;
	uint8_t abs_status_int;
    /* page select = 0x0 */
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0); 
	if(ret < 0) {
		
		TPD_DEBUG("synaptics_enable_interrupt_for_double_tap: select page failed ret = %d\n",
		    ret);
		return -1;
	}

	ret = i2c_smbus_read_byte_data(ts->client, F11_2D_CTRL00);
	if(ret < 0) {
		TPD_DEBUG("read reg F11_2D_CTRL00 failed\n");
		return -1;
	}

	if(enable) {
		status_int = (ret & 0xF8) | 0x04;
		
		/*clear interrupt bits for previous touch*/
		ret = i2c_smbus_read_byte_data(ts->client, 0x14);
		if(ret < 0) {
			TPD_DEBUG("synaptics_enable_interrupt :clear interrupt bits failed\n");
			return -1;
		}		
	} else {
		status_int = ret & 0xF8;
	}
	TPD_DEBUG("status_int = 0x%x\n", status_int);
	
	ret = i2c_smbus_write_byte_data(ts->client, F11_2D_CTRL00, status_int);
	if(ret < 0) {
		TPD_DEBUG("synaptics_enable_interrupt_for_double_tap: enable or disable\
		    interrupt failed,abs_int =%d\n", status_int);
		return -1;
	}
	TPD_DEBUG("synaptics_enable_interrupt_for_double_tap enable = %d\n", enable);

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
#endif

static int synaptics_read_product_id(struct synaptics_ts_data *ts)
{
	uint8_t buf1[11];
	int ret ;
	uint8_t q;
	
	memset(buf1, 0 , sizeof(buf1));
    /* page select = 0x0 */
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0); 
	if (ret < 0) {
		TPDTM_DMESG("synaptics_read_product_id: failed for page select\n");
		return -1;
	}

	ret = i2c_smbus_read_i2c_block_data(ts->client, F01_RMI_QUERY_BASE+11, 8, &(buf1[0x0]));
	ret = i2c_smbus_read_i2c_block_data(ts->client, F01_RMI_QUERY_BASE+19, 2, &(buf1[0x8]));
	
	if (ret < 0) {
		TPDTM_DMESG("synaptics_read_product_id: failed to read product info\n");
		return -1;
	}
	TPDTM_DMESG("synaptics product id: %s \n",buf1);
	return 0;
}

static int synaptics_init_panel(struct synaptics_ts_data *ts)
{
	int ret;
	/* page select = 0x0 */
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0); 
	if (ret < 0) {
		TPDTM_DMESG("i2c_smbus_write_byte_data failed for page select\n");
		return -1;
	}
	
	
	/*device control: normal operation, configur=1*/
		ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CMD00, 0x01); 
		msleep(150);
	ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CTRL00, 0x80); 
	if (ret < 0) {
		TPDTM_DMESG("i2c_smbus_write_byte_data failed for mode select\n");
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
		
		TPDTM_DMESG("synaptics_enable_interrupt: select page failed ret = %d\n",
		    ret);
		return -1;
	}
    

	if(enable) {
		abs_status_int = 0x7f;

		/*clear interrupt bits for previous touch*/
		ret = i2c_smbus_read_byte_data(ts->client, F01_RMI_DATA_BASE+1);
		if(ret < 0) {
			TPDTM_DMESG("synaptics_enable_interrupt :clear interrupt bits failed\n");
			return -1;
		}
	} else {
		abs_status_int = 0x0;
	}	

	ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CTRL00+1, abs_status_int);
	if(ret < 0) {
		TPDTM_DMESG("synaptics_enable_interrupt: enable or disable abs \
		    interrupt failed,abs_int =%d\n",abs_status_int);
		return -1;
	}

	ret = i2c_smbus_read_byte_data(ts->client, F01_RMI_CTRL00+1);
	printk("S3202-----------0x5E=%x---------\n",ret);
	return 0;	
}

static void synaptics_ts_irq_handler(void);


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
#if 0
static void tp_get_id(void)
{
    int tmp0 = 0,tmp1 = 0; 
	
     printk("tmp0=%d,tmp1=%d\n",tmp0 ,tmp1);

}
#endif
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
		synaptics_init_panel(ts);
	if (ret < 0) {
		TPD_DEBUG("synaptics_ts_work_func: control tm1400 to sleep failed\n");
		return;
	}
}
#ifdef OPPO_R819
//Fuchun.Liao add for select tp
static void TP_get_id(void)
{
   static unsigned int tmp0,tmp1;
   mt_set_gpio_mode(GPIOEXT32,0);
   mt_set_gpio_dir(GPIOEXT32,0);
   mt_set_gpio_pull_enable(GPIOEXT32,1);
   mt_set_gpio_pull_select(GPIOEXT32,1);
   mt_set_gpio_mode(GPIOEXT33,0);
   mt_set_gpio_dir(GPIOEXT33,0);
   mt_set_gpio_pull_enable(GPIOEXT33,1);
   mt_set_gpio_pull_select(GPIOEXT33,1);
   mdelay(2);
   tmp0 = mt_get_gpio_in(GPIOEXT32);
   tmp1 = mt_get_gpio_in(GPIOEXT33);
   if (tmp0 == 0 && tmp1 == 0)
   {
       tp_dev=TP_TRULY_BLACK;
   }
   //compatiable with no tp_id pin
   else if(tmp0 == 1 && tmp1 ==1)
   {
       tp_dev=TP_TRULY_BLACK;
   }
   else
   {
       tp_dev=TP_YOUNGFAST;
   }
   printk("TP_get_id:tp_dev:%d,GPIOEXT32:%d,GPIOEXT33:%d\n",tp_dev,tmp0,tmp1);
}
#endif /*OPPO_R819*/
static void int_touch(struct synaptics_ts_data *ts)
{
    int ret= -1,i=0, j = 0;
    uint8_t buf[5];
    uint32_t finger_state = 0;
	uint8_t finger_num = 0;
    struct point_info points;
    memset(buf,0,sizeof(buf));
    ret = i2c_smbus_read_i2c_block_data(ts->client, F11_2D_DATA_BASE, 3, &(buf[0]));
	if (ret < 0) {
		TPDTM_DMESG("synaptics_ts_work_func: i2c_transfer failed\n");
        return;
	}
    finger_state = ((buf[2]&0x0f)<<16)|(buf[1]<<8)|buf[0];	
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
		    ret = i2c_smbus_read_i2c_block_data(ts->client, F11_2D_DATA_BASE + 3 + i*5,
			        5, &(buf[0]));

			if (ret < 0) {
				TPDTM_DMESG("synaptics_ts_work_func: i2c_transfer failed\n");
	        	return;
			}
			points.x = (buf[0]<<4) | (buf[2] & 0x0f);
			points.raw_x = buf[3]&0x0f;
			points.y = (buf[1]<<4) | ((buf[2] & 0xf0)>>4);
			points.raw_y = (buf[3]&0xf0)>>4;
		    points.z = buf[4];
			
			if((boot_mode == FACTORY_BOOT)||(boot_mode == RECOVERY_BOOT))
			tpd_button(ts,points.x,points.y,1);
			
			if(points.z > 0)
			{
			tpd_down(ts,points.raw_x, points.raw_y, points.x, points.y,
			    points.z);
				if((points.x >= 995)&&(points.y >= 1795))
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
	else{
		    if((boot_mode == FACTORY_BOOT)||(boot_mode == RECOVERY_BOOT))
			tpd_button(ts,points.x,points.y,0);
			tpd_up(ts,points.raw_x, points.raw_y, points.x, points.y,
			    points.z);
	}
			
	input_sync(ts->input_dev);
	ts->pre_finger_state = finger_state;  
#ifdef SUPPORT_DOUBLE_TAP
	if (finger_num == 0)
	{
		ret = i2c_smbus_read_word_data(ts->client, 0x4B);
		printk("0x4b is %x\n",ret);
		if (ret & 0x01)
		{
			TPD_DEBUG("wjw simulate power key pressed\n");
			input_report_key(ts->input_dev, KEY_POWER, 1);
			input_sync(ts->input_dev);	
			input_report_key(ts->input_dev, KEY_POWER, 0);
			input_sync(ts->input_dev);
			atomic_inc(&double_tap_number);
		}		
	}
#endif	
}

static void synaptics_ts_work_func(struct work_struct *work)
{
	int ret;
	uint8_t start_reg;
	uint8_t buf[5]; 				/*17 edit by sun  for muiti Touchpanel */
	uint8_t finger_num = 0;
    uint32_t finger_state = 0;
    uint8_t status = 0;
	struct point_info points;
	uint8_t inte = 0;
	int i;

	struct synaptics_ts_data *ts = container_of(work,
	    struct synaptics_ts_data, work);

	memset(buf,0,sizeof(buf));
	if(is_suspend)
	    return;

   
    down(&work_sem);
    atomic_set(&is_touch,1);
    ret = i2c_smbus_read_byte_data(ts->client, F01_RMI_CTRL00+1);
	//printk("S3202-----------0x5e=%x---------\n",ret);
    ret  = i2c_smbus_read_byte_data(ts->client,F01_RMI_DATA_BASE+1); 
   // printk("0x14=======%x\n",ret);
	ret = i2c_smbus_read_word_data(ts->client, F01_RMI_DATA_BASE);
	
	
	if (ret < 0) {
		TPDTM_DMESG("synaptics_ts_work_func: i2c_transfer failed\n");
        goto ERR_I2C2;
	}
	status = ret&0xff;

	inte = (ret&0x7f00)>>8;
	TPDTM_DMESG("synaptics_ts_work_func,inte = %x, status = %x\n",inte,status);
	
	if(status)
		
	int_state(ts);
	if(reset_staus)//for bottom right reset
	{
		synaptics_software_rezero(ts);
		reset_staus = 0;
	}
	
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
	/* TPDTM_DMESG("synaptics_ts_timer_func\n"); */

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
static void synaptics_software_rezero(struct synaptics_ts_data *ts)
{
	printk("synaptics do software rezero\n");
	i2c_smbus_write_byte_data(ts->client, F11_2D_CMD_BASE, 0x01);
	msleep(50);
	i2c_smbus_write_byte_data(ts->client, F01_RMI_CTRL00, 0x80); 
	synaptics_enable_interrupt( ts, 1);
}
static enum hrtimer_restart synaptics_ts_report_timer_func(struct hrtimer *report_timer)
{
	if(err_report_num >= 60)	
	{
	  reset_staus = 1;
	}
	err_report_num = 0;		
	return HRTIMER_NORESTART;
}

#ifdef SUPPORT_DOUBLE_TAP
static ssize_t tp_double_tap_num_show(struct device_driver *ddri, char *buf)
{

    int ret = 0;
	
	printk("double tap number is: %d\n", atomic_read(&double_tap_number));
	ret = sprintf(buf, "%d\n", atomic_read(&double_tap_number));
	return ret;
}

static ssize_t tp_double_tap_num_store(struct device_driver *ddri,
       char *buf, size_t count)
{
	int number;

	if(1 == sscanf(buf, "%d", &number))
	{
		atomic_set(&double_tap_number, number);
	}
	else 
	{
		printk("invalid content: '%s', length = %d\n", buf, count);
	} 	

	printk("tp_double_tap_num_store number=%d\n", atomic_read(&double_tap_number));
	
	return count;
}

static ssize_t tp_double_tap_enable_show(struct device_driver *ddri, char *buf)
{

    int ret = 0;

	printk("double tap enable is: %d\n", atomic_read(&double_tap_enable));
	ret = sprintf(buf, "%d\n", atomic_read(&double_tap_enable));
	
	return ret;
}

static ssize_t tp_double_tap_enable_store(struct device_driver *ddri,
       char *buf, size_t count)
{
    int enable;
    printk("wjw_tp_double_tap_num_store is called \n");
	if(1 == sscanf(buf, "%d", &enable))
	{
		atomic_set(&double_tap_enable, enable);
	}
	else 
	{
		printk("wjw_tp_invalid content: '%s', length = %d\n", buf, count);
	}
	
	printk("wjw_tp_double_tap_enable_store enable=%d\n", atomic_read(&double_tap_enable));
	
	return count;
}

#endif


#ifdef SUPPORT_DOUBLE_TAP
static DRIVER_ATTR(oppo_tp_double_tap_num, 0664, tp_double_tap_num_show, tp_double_tap_num_store);
static DRIVER_ATTR(oppo_tp_double_tap_enable, 0777, tp_double_tap_enable_show, tp_double_tap_enable_store);
#endif


static ssize_t tp_show(struct device_driver *ddri, char *buf)
{
    uint8_t ret = 0;
	uint8_t d = 0;
	
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
   //Fuchun.Liao@wxkfDrv,2013/06/09,modify for kernel null pointer
	ret = i2c_smbus_read_word_data(ts_g->client, F01_RMI_DATA_BASE);
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
		TPDTM_DMESG("invalid content: '%s', length = %d\n", buf, count);
	}
	
	return count;
}


#define TX_NUM 12  //tx rx channel in the firmware
#define RX_NUM 23

static int DiagonalUpperLimit = 1100;
static int DiagonalLowerLimit = 900;


static int16_t Truly_array_limit[] = {
1618,	2877,	1619,	2878,	1623,	2885,	1629,	2896,	1606,	2855,	1603,	2849,	1603,	2849,	1614,	2870,	1613,	2867,	1611,	2863,	1617,	2875,	1624,	2887,	1617,	2874,	1454,	2584,	1455,	2587,	1443,	2565,	1444,	2567,	1438,	2557,	1449,	2576,	1464,	2603,	1468,	2610,	1441,	2561,	1424,	2532,
1492,	2652,	1514,	2692,	1524,	2709,	1534,	2726,	1498,	2663,	1484,	2638,	1486,	2641,	1488,	2645,	1486,	2641,	1487,	2644,	1494,	2656,	1513,	2689,	1503,	2672,	1447,	2573,	1444,	2567,	1438,	2556,	1447,	2572,	1457,	2591,	1445,	2569,	1464,	2603,	1457,	2591,	1445,	2568,	1436,	2553,
1498,	2663,	1526,	2712,	1510,	2685,	1525,	2711,	1497,	2660,	1480,	2632,	1474,	2621,	1474,	2621,	1471,	2616,	1473,	2618,	1479,	2629,	1492,	2653,	1487,	2643,	1447,	2573,	1443,	2566,	1433,	2547,	1438,	2556,	1433,	2547,	1441,	2561,	1463,	2601,	1465,	2604,	1440,	2560,	1437,	2555,
1519,	2700,	1556,	2766,	1512,	2687,	1513,	2690,	1490,	2648,	1478,	2627,	1470,	2614,	1466,	2606,	1459,	2595,	1465,	2604,	1468,	2610,	1481,	2632,	1481,	2633,	1452,	2581,	1461,	2596,	1445,	2569,	1472,	2617,	1462,	2599,	1486,	2642,	1447,	2572,	1442,	2563,	1441,	2562,	1437,	2554,
1543,	2743,	1554,	2763,	1501,	2668,	1494,	2656,	1485,	2640,	1474,	2620,	1469,	2611,	1456,	2589,	1455,	2586,	1456,	2589,	1474,	2621,	1477,	2626,	1476,	2625,	1463,	2601,	1467,	2608,	1457,	2591,	1470,	2614,	1459,	2594,	1463,	2602,	1445,	2569,	1439,	2557,	1443,	2565,	1439,	2558,
1486,	2642,	1508,	2680,	1481,	2633,	1477,	2627,	1476,	2624,	1473,	2620,	1462,	2599,	1458,	2591,	1447,	2573,	1454,	2584,	1470,	2613,	1487,	2644,	1478,	2628,	1489,	2647,	1490,	2648,	1483,	2636,	1449,	2577,	1439,	2558,	1439,	2558,	1438,	2557,	1439,	2558,	1444,	2567,	1441,	2562,
1454,	2585,	1478,	2628,	1469,	2611,	1462,	2598,	1460,	2595,	1461,	2597,	1451,	2580,	1454,	2585,	1448,	2575,	1455,	2587,	1477,	2625,	1491,	2650,	1485,	2640,	1471,	2615,	1488,	2645,	1447,	2573,	1445,	2569,	1438,	2557,	1440,	2559,	1442,	2563,	1446,	2570,	1448,	2574,	1444,	2568,
1445,	2570,	1467,	2608,	1460,	2595,	1451,	2580,	1451,	2579,	1452,	2581,	1449,	2577,	1451,	2580,	1444,	2567,	1456,	2589,	1490,	2649,	1497,	2661,	1466,	2606,	1476,	2624,	1476,	2624,	1454,	2584,	1447,	2573,	1441,	2561,	1446,	2571,	1446,	2571,	1447,	2573,	1451,	2580,	1450,	2578,
1447,	2572,	1469,	2612,	1463,	2601,	1454,	2585,	1457,	2590,	1450,	2578,	1447,	2572,	1454,	2584,	1447,	2572,	1456,	2588,	1519,	2701,	1490,	2649,	1471,	2615,	1488,	2645,	1494,	2655,	1463,	2601,	1457,	2590,	1447,	2573,	1453,	2583,	1454,	2584,	1458,	2591,	1465,	2605,	1456,	2589,
1456,	2589,	1486,	2642,	1474,	2621,	1459,	2595,	1457,	2591,	1450,	2579,	1446,	2570,	1458,	2593,	1451,	2580,	1452,	2581,	1516,	2695,	1466,	2606,	1466,	2606,	1504,	2673,	1488,	2646,	1465,	2604,	1459,	2594,	1453,	2583,	1461,	2598,	1459,	2594,	1465,	2605,	1473,	2619,	1466,	2607,
1460,	2595,	1491,	2652,	1474,	2621,	1471,	2616,	1461,	2598,	1455,	2587,	1453,	2584,	1459,	2594,	1452,	2581,	1452,	2581,	1464,	2603,	1461,	2597,	1451,	2580,	1517,	2697,	1498,	2664,	1474,	2621,	1471,	2615,	1467,	2607,	1474,	2621,	1470,	2614,	1476,	2624,	1483,	2636,	1479,	2630,
1460,	2595,	1485,	2640,	1478,	2628,	1476,	2624,	1472,	2617,	1468,	2610,	1469,	2612,	1466,	2605,	1459,	2595,	1461,	2598,	1474,	2620,	1464,	2602,	1453,	2583,	1637,	2910,	1610,	2863,	1597,	2839,	1591,	2828,	1581,	2811,	1588,	2823,	1582,	2812,	1582,	2813,	1586,	2819,	1602,	2849,
};


static int16_t Ofilm_array_limit[] = {
	1451,	2176,	1494,	2242,	1506,	2259,	1543,	2314,	1503,	2254,	1505,	2257,	1514,	2270,	1518,	2277,	1515,	2272,	1529,	2293,	1549,	2323,	1342,	2014,	1343,	2015,	1344,	2016,	1349,	2023,	1343,	2014,	1342,	2013,	1343,	2014,	1340,	2009,	1344,	2015,	1341,	2012,	1305,	1957,
	1398,	2097,	1425,	2137,	1422,	2133,	1424,	2137,	1419,	2129,	1418,	2127,	1423,	2134,	1421,	2131,	1419,	2129,	1418,	2127,	1423,	2134,	1386,	2078,	1382,	2073,	1382,	2074,	1386,	2079,	1381,	2072,	1380,	2069,	1380,	2070,	1377,	2066,	1380,	2071,	1380,	2070,	1338,	2007,
	1448,	2172,	1416,	2125,	1411,	2116,	1413,	2119,	1406,	2110,	1406,	2108,	1408,	2111,	1408,	2112,	1406,	2108,	1405,	2107,	1408,	2112,	1394,	2090,	1395,	2093,	1395,	2093,	1400,	2100,	1394,	2092,	1393,	2089,	1395,	2092,	1391,	2087,	1393,	2090,	1392,	2088,	1350,	2025,
	1393,	2090,	1414,	2121,	1412,	2118,	1414,	2121,	1409,	2114,	1407,	2110,	1411,	2116,	1410,	2115,	1409,	2113,	1406,	2110,	1410,	2115,	1397,	2095,	1394,	2091,	1393,	2090,	1398,	2096,	1392,	2088,	1392,	2088,	1393,	2089,	1390,	2085,	1393,	2089,	1393,	2089,	1354,	2030,
	1430,	2144,	1429,	2143,	1416,	2124,	1416,	2124,	1411,	2116,	1409,	2113,	1413,	2119,	1413,	2119,	1411,	2116,	1409,	2114,	1414,	2120,	1398,	2097,	1395,	2093,	1393,	2089,	1396,	2094,	1390,	2085,	1391,	2087,	1391,	2087,	1388,	2082,	1391,	2087,	1392,	2088,	1354,	2032,
	1397,	2096,	1420,	2130,	1419,	2128,	1418,	2127,	1415,	2122,	1411,	2116,	1415,	2123,	1414,	2121,	1412,	2118,	1409,	2114,	1411,	2117,	1402,	2102,	1398,	2097,	1396,	2094,	1399,	2098,	1394,	2091,	1395,	2093,	1396,	2094,	1392,	2088,	1394,	2091,	1394,	2091,	1352,	2029,
	1389,	2083,	1406,	2109,	1404,	2106,	1409,	2113,	1423,	2134,	1446,	2169,	1418,	2128,	1400,	2100,	1399,	2099,	1396,	2094,	1398,	2097,	1410,	2115,	1410,	2114,	1408,	2112,	1412,	2118,	1408,	2111,	1409,	2114,	1408,	2112,	1404,	2107,	1407,	2110,	1407,	2111,	1365,	2048,
	1412,	2118,	1435,	2152,	1408,	2112,	1405,	2108,	1403,	2105,	1400,	2100,	1405,	2107,	1404,	2106,	1403,	2104,	1399,	2099,	1401,	2102,	1411,	2116,	1410,	2115,	1407,	2111,	1413,	2119,	1407,	2111,	1408,	2112,	1407,	2111,	1404,	2106,	1407,	2111,	1407,	2111,	1370,	2056,
	1402,	2103,	1428,	2142,	1424,	2136,	1438,	2158,	1403,	2104,	1399,	2098,	1403,	2104,	1403,	2105,	1403,	2104,	1399,	2099,	1400,	2100,	1412,	2118,	1410,	2115,	1408,	2111,	1413,	2119,	1408,	2112,	1407,	2110,	1405,	2108,	1404,	2106,	1406,	2109,	1408,	2112,	1372,	2058,
	1385,	2078,	1409,	2114,	1410,	2115,	1416,	2124,	1426,	2139,	1428,	2142,	1428,	2142,	1417,	2126,	1411,	2116,	1414,	2122,	1404,	2106,	1427,	2141,	1422,	2134,	1418,	2128,	1423,	2134,	1417,	2126,	1416,	2124,	1415,	2123,	1413,	2119,	1416,	2123,	1418,	2127,	1378,	2067,
	1369,	2054,	1394,	2091,	1395,	2092,	1397,	2095,	1393,	2089,	1390,	2086,	1399,	2098,	1408,	2112,	1416,	2125,	1406,	2109,	1390,	2085,	1445,	2168,	1453,	2180,	1444,	2166,	1444,	2165,	1436,	2154,	1433,	2150,	1431,	2147,	1429,	2143,	1432,	2148,	1431,	2147,	1393,	2090,
	1344,	2016,	1370,	2054,	1369,	2053,	1370,	2054,	1382,	2073,	1367,	2051,	1402,	2103,	1383,	2074,	1386,	2079,	1379,	2068,	1365,	2047,	1830,	2745,	1669,	2504,	1618,	2427,	1599,	2398,	1580,	2370,	1567,	2351,	1567,	2350,	1564,	2346,	1560,	2340,	1578,	2367,	1566,	2348,
};

static void checkCMD()
{
  int ret;
 do {

        delay_qt_ms(10); //wait 1ms

        ret =  i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE);
        
    } while (ret > 0x00); 
}



static void synaptics_read_register_map_page1(struct synaptics_ts_data *ts)
{

unsigned char buf[4];
int ret;
printk("synaptics_read_register_map_page1 start\n");
ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x1); 
if (ret < 0) {
		TPDTM_DMESG("i2c_smbus_write_byte_data failed for page select\n");
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
		 TPDTM_DMESG("read_baseline: i2c_smbus_write_byte_data failed \n");
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
	if(tp_dev  == TP_TRULY_BLACK){
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
	printk("[s3202]tem_new end = %x",ret&0xff);
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

	ret = i2c_smbus_write_word_data(ts_g->client,F54_ANALOG_DATA_BASE+1,0); //set fifo 00
	
	for(x = 0;x < TX_NUM; x++) 
   	{
      for(y = 0; y < RX_NUM; y++)
   	    {
	   
	         ret= i2c_smbus_read_word_data(ts_g->client,F54_ANALOG_DATA_BASE+3);
			 Rxdata[x][y] = ret&0xffff;
			// printk("lfc Rxdata[%d][%d]=%d,",x,y,Rxdata[x][y]);
			// printk("\n");
		        
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
			// printk("lfc Rxdata[%d][%d]=%d,",x+TX_NUM,y,Rxdata[x+TX_NUM][y]);
			// printk("\n");
		    // printk("\nRxdata[%d][%d] = %d",x+TX_NUM,y,Rxdata[x + TX_NUM][y]);   
		}
	}
	printk("\nstep 4:check rx-rx short\n");
//step 4:check rx-rx short
	if (tp_dev = TP_TRULY_BLACK){
		for(x = 0;x < RX_NUM; x++)
	   	{
	      for(y = 0; y < RX_NUM; y++)
	   	  {
			  if (x ==y)
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

static DRIVER_ATTR(oppo_tp_delta_image, 0664, tp_delta_show, tp_delta_store);

static DRIVER_ATTR(oppo_tp_test, 0664, tp_test_show, tp_test_store);

static DRIVER_ATTR(oppo_tp_debug, 0664, tp_show, store_tp);




static int tp_write_func (struct file *file,const char *buffer,
    unsigned long count,void *data);

#ifdef SUPPORT_DOUBLE_TAP
static int tp_double_write_func (struct file *file,const char *buffer,
    unsigned long count,void *data);
static int tp_double_read_func (char *page,const char **stat,
    off_t off,int count,int *eof,void *data);
#endif

static int init_synaptics_proc()
{
	int ret = 0;
	prEntry = create_proc_entry( "synaptics_reset", 0666, NULL );
	if(prEntry == NULL)
	{
		ret = -ENOMEM;
	  	printk(KERN_INFO"init_synaptics_proc: Couldn't create proc entry\n");
	}
	else{
		prEntry->write_proc = tp_write_func;
	}

#ifdef SUPPORT_DOUBLE_TAP
	prEntry_double = create_proc_entry( "driver_attr_oppo_tp_double_tap_enable", 0777, NULL );
	if(prEntry_double == NULL)
	{
		ret = -ENOMEM;
	  	printk(KERN_INFO"init_synaptics_proc: Couldn't create proc entry\n");
	}
	else{
		prEntry_double->write_proc = tp_double_write_func;
		prEntry_double->read_proc = tp_double_read_func;
	}
#endif

	return ret;
}
#define F54_ANALOG_CMD00 0x6F

static int tp_write_func (struct file *file,const char *buffer,
    unsigned long count,void *data)
{
	struct synaptics_ts_data *ts; 
	int ret = 0;
#ifdef OPPO_R819 //modify bu huqiao for black TP buttonled
	ssize_t num_read_chars = 0;
	if(tp_dev == TP_TRULY_BLACK)
	{
		num_read_chars += sprintf(buffer, "2");
		//printk("TP_DEV = TP BLACK\n");
		}
	else 
	{
		num_read_chars += sprintf(buffer, "3");
		//printk("TP_DEV = TP WHITE\n");
		}
#endif
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
			if(ts->no_erroponit_exist == 0)
			{
				TPD_DEBUG("tp_write_func:find ERRO point!!!!!!!!!!\n  is_touch = %d, finger_num = %d\n",atomic_read(&is_touch),ts->finger_num);
				ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x01); 
				ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_CMD00, 0x02); 
				ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00); 
				if(ret) {
						TPD_DEBUG("tp_write_func:cannot  enable interrupt \n");
						goto err;
					}
						//ret = i2c_smbus_write_byte_data(ts->client, F11_2D_CMD00, 0x01); 
						//msleep(50);
					TPD_DEBUG("tp_write_func: rezero successed!\n");
	
			}
		}
	
		if(!atomic_read(&is_touch)&&(!all_finger_up))
			{
				TPD_DEBUG("tp_write_func:when no touch rezero again!\n is_touch = %d, finger_num = %d\n",atomic_read(&is_touch),ts->finger_num);
				ts->no_erroponit_exist = 1;
				ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x01); 
				ret = i2c_smbus_write_byte_data(ts->client, F54_ANALOG_CMD00, 0x02); 
				ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x00); 
				if(ret) {
					TPD_DEBUG("tp_write_func:cannot enable interrupt \n");
					goto err;
				}
				//ret = i2c_smbus_write_byte_data(ts->client, F11_2D_CMD00, 0x01); 
				//msleep(50);
				TPD_DEBUG("tp_write_func: rezero successed!\n");
			  }
			}
		
		TPD_DEBUG("tp_write_func:no erro point exist!\n is_suspend = %d, is_touch = %d, finger_num = %d\n",is_suspend,atomic_read(&is_touch),ts->finger_num);
		err:
		up(&work_sem);
	return count;
}

#ifdef SUPPORT_DOUBLE_TAP
static int tp_double_write_func (struct file *file,const char *buffer,
    unsigned long count,void *data)
	{
		int ret = 0;
		TPDTM_DMESG("tp_double_write_func is called!\n");
		sscanf(buffer,"%d",&ret);
		if((ret == 0 )||(ret == 1))
			atomic_set(&double_tap_enable,ret);
		switch(ret)
		{	case 0:
				TPDTM_DMESG("double-tap will be disable\n");
				break;
			case 1:
				TPDTM_DMESG("double-tap will be enable\n");
				break;
			default:TPDTM_DMESG("Please enter 0 or 1 to open or close the double-tap function\n");
		}
		return count;
	}
static int tp_double_read_func (char *page,const char **stat,
    off_t off,int count,int *eof,void *data)
	{
		TPDTM_DMESG("tp_double_read_func is called\n ");
		int ret=atomic_read(&double_tap_enable);
		if (ret == 0)
			TPDTM_DMESG("double-tap function is disable\n");
		if (ret == 1)
			TPDTM_DMESG("double-tap function is enable\n");
		return 0;
	}
#endif

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


 static enum wk_wdt_type {
	 WK_WDT_LOC_TYPE,
	 WK_WDT_EXT_TYPE
 };
 extern void mtk_wdt_restart(enum wk_wdt_type type);

 static int kick_wdt()
 {
	 mtk_wdt_restart(WK_WDT_LOC_TYPE);
	 mtk_wdt_restart(WK_WDT_EXT_TYPE);
 
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
    msleep(200);
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
       // if(tp_dev == TP_TRULY_BLACK)
        //for(i=0;i<block;i++)
        //{
	
               for(i=0;i<block;i++)	
               {			   
                  if(tp_dev == TP_TRULY_BLACK)			   
                   i2c_smbus_write_byte_data(client,SynaF34Reflash_BlockData+i,
                   //TRULY_Firmware_Data[j][i]);
			       TRULY_Firmware_Data[j][i]);
				 
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
		

	   if(j % 500 == 0){
		   kick_wdt();

	   	}
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
				    if(tp_dev == TP_TRULY_BLACK)
			       i2c_smbus_write_byte_data(client,SynaF34Reflash_BlockData+i,
                   TRULY_Config_Data[j][i]);
				    
					
				   
                }				   
	
		
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
    msleep(200);
    //step 3 check status
    ret = i2c_smbus_read_byte_data(client,SynaF34_FlashControl);
    printk("The status(disable)of flash data3 is %0x\n",ret);
 

}


static int synaptics_ts_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	struct synaptics_ts_data *ts;
	int ret = 0;
	uint8_t buf[4];
	uint32_t CURRENT_FIRMWARE_ID = 0;
	uint32_t bootloader_mode;
    uint32_t FIRMWARE_ID_TRULY = 0;
	uint8_t fw_cnt = 0;
	
	TPDTM_DMESG("synaptics_ts_probe_blacks: enter !!!!!!!!!!!!!!\n");	
#ifdef OPPO_R819
//Fuchun.Liao@wxkfDrv,2013/06/04,add for tp can work if alsps error
	tpd_power(1);
#endif 	
	
#ifdef SUPPORT_DOUBLE_TAP
	atomic_set(&double_tap_number,0);
	atomic_set(&double_tap_enable,1);
#endif

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		TPDTM_DMESG("synaptics_ts_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}
	
    ret = i2c_smbus_read_byte_data(client,0x13);
	if(ret < 0)
	{
		ret = i2c_smbus_read_byte_data(client,0x13);
		if(ret < 0)
		{
			//tpd_power(0);
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

	//tp_autotest = kzalloc(sizeof(*tp_autotest), GFP_KERNEL);
	//memset(tp_autotest,0,sizeof(*tp_autotest));

#ifndef OPPO_R819
//Fuchun.Liao@wxkfDrv,2013/05/21 modify
	/*check tp factory*/
	ret = mt_get_gpio_in(31);
//	if(ret){
    if(1) {
		tp_dev = TP_TRULY_BLACK;
		TPDTM_DMESG("this is Truly TP, GPIO84 = %d\n",ret);
	}
	else{		
		tp_dev = TP_YOUNGFAST;
		TPDTM_DMESG("this is YongFast TP,GPIO84 = %d\n",ret);
	}
#else
   TP_get_id();
#endif /*OPPO_R819*/
	INIT_WORK(&ts->work, synaptics_ts_work_func);
	ts->client = client;
	i2c_set_clientdata(client, ts);


	synaptics_read_register_map(ts);
	bootloader_mode = i2c_smbus_read_byte_data(ts->client,F01_RMI_DATA_BASE);
	bootloader_mode = bootloader_mode&0xff;
	bootloader_mode = bootloader_mode&0x40; 
	printk("synaptics:before fw update,bootloader_mode = 0x%x\n",bootloader_mode);
	
	i2c_smbus_read_i2c_block_data(ts->client, F34_FLASH_CTRL00, 4, buf); 
	CURRENT_FIRMWARE_ID = (buf[0]<<24)|(buf[1]<<16)|(buf[2]<<8)|buf[3];
	printk("CURRENT_FIRMWARE_ID = 0x%x\n",CURRENT_FIRMWARE_ID);	
	TP_FW = CURRENT_FIRMWARE_ID;

	if(tp_dev == TP_TRULY_BLACK){
		printk("TP_DEV = TP_TRULY_BLACK\n");
		#ifdef OPPO_R819//huqiao modify by black touchpanel buttonled
				upmu_set_kpled_dim_duty(0x1F);	 
				upmu_set_kpled_dim_div(0x0); 
				upmu_set_kpled_sel(0x5);			// Imax = 56.25mA  
		#endif
	buf[0]=TRULY_Config_Data[0][0];
	buf[1]= TRULY_Config_Data[0][1];
	buf[2]= TRULY_Config_Data[0][2];
	buf[3]=TRULY_Config_Data[0][3];
	FIRMWARE_ID_TRULY = (buf[0]<<24)|(buf[1]<<16)|(buf[2]<<8)|buf[3];
	
	printk("read firmware ID: CURRENT_FIRMWARE_ID = %x,LOCAL_FIRMWARE_ID = %x\n",CURRENT_FIRMWARE_ID,FIRMWARE_ID_TRULY);
	}
	else
	{
		printk("check TP type erro!!!!!!!!!\n");
		
		goto synaptics_ts_probe_end;
	}
	
	if(boot_mode == NORMAL_BOOT)
	{
	   
	    printk("synaptics_ts_probe:FW update start... \n");
		if(tp_dev == TP_TRULY_BLACK)
		{
	    	if(CURRENT_FIRMWARE_ID == FIRMWARE_ID_TRULY)
			{
				printk("CURRENT_FIRMWARE_ID = 0x%x,FIRMWARE_ID_TRULY = 0x%x\n",CURRENT_FIRMWARE_ID,FIRMWARE_ID_TRULY);
		   		goto after_fw_update;
			}
						
				
		}
		else
		{
		   	
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
	/*disable interrupt*/
	ret = synaptics_enable_interrupt(ts, 0);
	if(ret < 0) {
		TPDTM_DMESG(" synaptics_ts_probe: disable interrupt failed\n");
	}
	
	/*read product id */
	ret = synaptics_read_product_id(ts);
	if(ret) {
		TPDTM_DMESG("failed to read product info \n");
	}
   
	/*read max_x ,max_y*/
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0);
	if (ret < 0) {
		TPDTM_DMESG("i2c_smbus_write_byte_data failed for page select\n");
	}
	
	ret = i2c_smbus_read_word_data(ts->client, F11_2D_CTRL06);
    if(ret > 0)
	    ts->max_x = ret&0xffff;
       
    ret = i2c_smbus_read_word_data(ts->client, F11_2D_CTRL08);
    if(ret > 0)
	    ts->max_y = ret&0xffff;
	    
	TPDTM_DMESG("max_x = %d,max_y = %d\n",ts->max_x,ts->max_y);

	bootloader_mode = i2c_smbus_read_byte_data(ts->client,F01_RMI_DATA_BASE);
	bootloader_mode = bootloader_mode&0xff;
	bootloader_mode = bootloader_mode&0x40; 
	
	TPDTM_DMESG("synaptics:afte fw update,bootloader_mode = 0x%x\n",bootloader_mode);


	
	TPDTM_DMESG("synaptics max_x = %d\n max_y = %d\n",ts->max_x,ts->max_y);
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
#ifdef SUPPORT_DOUBLE_TAP
	set_bit(KEY_POWER, ts->input_dev->keybit);
#endif
	
	/* For multi touch */
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR,
			     0, 255, 0, 0);	

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X,
					 0, ts->max_x, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y,
					 0,ts->max_y - 140, 0, 0);
		

	
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


#ifdef TPD_USE_EINT
	ts_g = ts;
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
					0);
	
	
	mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);

	ret = synaptics_enable_interrupt(ts, 1);
	
	if(ret) {
		TPDTM_DMESG("synaptics_ts_probe: failed to enable synaptics  interrupt \n");
		free_irq(client->irq, ts);
		goto err_power_failed;
	}
	
	/*config tm1429: set report rate, sleep mode */
	ret = synaptics_init_panel(ts); /* will also switch back to page 0x04 */
	if (ret < 0) {
		TPDTM_DMESG("synaptics_init_panel failed\n");	
	}

	ts->use_irq = 1;
#else
	hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ts->timer.function = synaptics_ts_timer_func;
	hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
#endif

	hrtimer_init(&ts->report_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ts->report_timer.function = synaptics_ts_report_timer_func;


#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = synaptics_ts_early_suspend;
	ts->early_suspend.resume = synaptics_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif
	
	if (driver_create_file(&synaptics_driver.driver, &driver_attr_oppo_tp_debug))
	{            
		TPDTM_DMESG("driver_create_file failt\n");
		goto exit_init_failed;
	}
  	
	if (driver_create_file(&synaptics_driver.driver, &driver_attr_oppo_tp_test))
	{           
		TPDTM_DMESG("driver_create_file failt\n");
		
		goto exit_init_failed;
	
	}
	if (driver_create_file(&synaptics_driver.driver, &driver_attr_oppo_tp_delta_image))
	{            
		TPDTM_DMESG("driver_create_file failt\n");
		goto exit_init_failed;
	}

#ifdef SUPPORT_DOUBLE_TAP
	if (driver_create_file(&synaptics_driver.driver, &driver_attr_oppo_tp_double_tap_num))
	{
		TPDTM_DMESG("driver_create_file fail\n");
		goto exit_init_failed;
	}
	
	if (driver_create_file(&synaptics_driver.driver, &driver_attr_oppo_tp_double_tap_enable))
	{
		TPDTM_DMESG("driver_creat_file fail\n");
		goto exit_init_failed;
	}
#endif

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

#ifdef OPPO_R819//hsy@oppo.com, add 2011/12/14 for tpd power off
	
	#ifndef SUPPORT_DOUBLE_TAP
		tpd_power(0);
	#else
		if (1 == atomic_read(&double_tap_enable))
		{
	   //null
		}
		else
		{
		//   tpd_power(0);
		}
	#endif
#endif/*OPPO_R819*/
	return ret;
}

static int synaptics_ts_remove(struct i2c_client *client)
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
	uint8_t d;
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
		TPDTM_DMESG("synaptics_ts_suspend: cannot disable interrupt\n");
		goto err;
	}
	ret = cancel_work_sync(&ts->work);
	if(ret) {
		TPDTM_DMESG("synaptics_ts_suspend: cannot disable work\n");
	}

	ret = synaptics_enable_interrupt(ts, 0);
	if(ret) {
		TPDTM_DMESG("synaptics_ts_suspend: can not disable \
		    synaptics_enable_interrupt\n");
	}
	
	ret = i2c_smbus_write_byte_data(client, F01_RMI_CTRL00, 0x01); 
	if (ret < 0) {
		TPDTM_DMESG("synaptics_ts_suspend: control tm1400 to sleep failed\n");
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
			TPDTM_DMESG("synaptics_ts_resume: control tm1400 to sleep failed\n");
			goto err;
		}
		
        is_suspend = 0;
		
#ifdef TPD_USE_EINT
		mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);    
#endif
		ret = synaptics_enable_interrupt(ts, 1);
		if(ret) {
			TPDTM_DMESG("synaptics_ts_resume:cannot  enable interrupt \n");
			goto err;
		}
    
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
	int ret = -1;
#ifdef SUPPORT_DOUBLE_TAP
    ret = i2c_smbus_write_byte_data(ts->client, F11_2D_CTRL32, 0x9a);
	if(ret < 0) {
		TPD_DEBUG("write reg F11_2D_CTRL32 failed\n");
		return -1;
	}
	
	ret = i2c_smbus_read_byte_data(ts->client, F11_2D_CTRL32);
	TPD_DEBUG("read reg F11_2D_CTRL32=%x \n",ret);
	if(ret < 0) {
		TPD_DEBUG("read reg F11_2D_CTRL32 failed\n");
		return -1;
	}
	
    if (1 == atomic_read(&double_tap_enable)) {
	    synaptics_enable_interrupt_for_double_tap(ts, 1);
	    return;	
    }
#endif	
	
	synaptics_ts_suspend(ts->client, PMSG_SUSPEND);
#ifdef OPPO_R819//hsy@oppo.com, add 2011/12/14 for tp power off
     //tpd_power(0);
	#ifndef SUPPORT_DOUBLE_TAP
	tpd_power(0);
	#else
	if (1 == atomic_read(&double_tap_enable))
	{
		TPDTM_DMESG("synaptic do not call tpd_power(0)");
	    //null
	}
	else
	{
	   tpd_power(0);
	}
	#endif	
	 
#endif/*OPPO_R819*/
}

static void synaptics_ts_late_resume(struct early_suspend *h)
{
	printk("synaptics_ts_late_resume is called\n");
	struct synaptics_ts_data *ts;
	ts = container_of(h, struct synaptics_ts_data, early_suspend);	
	int ret = -1;
#ifdef SUPPORT_DOUBLE_TAP
	ret = i2c_smbus_write_byte_data(ts->client, F11_2D_CTRL32, 0x2b);
	if(ret < 0) {
		printk("wjw tp write reg F11_2D_CTRL32 failed\n");
		return -1;
	}
	
	ret = i2c_smbus_read_byte_data(ts->client, F11_2D_CTRL32);
	printk("read reg F11_2D_CTRL32=%x \n",ret);
	if(ret < 0) {
		printk("read reg F11_2D_CTRL32 failed\n");
		return -1;
	}
	
    if (1 == atomic_read(&double_tap_enable)) {
		synaptics_enable_interrupt_for_double_tap(ts, 0); 
    }
#endif	
	
#ifdef OPPO_R819//hsy@oppo.com, add 2011/12/14 for tpd power off
	#ifndef SUPPORT_DOUBLE_TAP
	tpd_power(1);
    //yongjun.wu@Prodrv,add for unlock screen two times
	input_report_key(ts->input_dev, BTN_TOUCH, 0);
	input_mt_sync(ts->input_dev);	
	input_sync(ts->input_dev);
	#else
	if (1 == atomic_read(&double_tap_enable))
	{
		TPDTM_DMESG("synaptic do not call tpd_power(1)");
	   //null
	}
	else
	{
	  tpd_power(1);	
	}
	#endif
#endif/*OPPO_R819*/
	synaptics_ts_resume(ts->client);
}
#endif

/*----------------------------------------------------------------------------*/
static const struct i2c_device_id synaptics_ts_id[] = {
	{ TPD_DEVICE, 0 },
	{ }
};

static struct i2c_board_info __initdata i2c_tpd={ I2C_BOARD_INFO(TPD_DEVICE, (0x42>>1))};


/*
static unsigned short synaptics_force[] = {0x00, 0xE0, 
    I2C_CLIENT_END, I2C_CLIENT_END};
static const unsigned short *const synaptics_forces[] = { synaptics_force,
    NULL };
static struct i2c_client_address_data synaptics_addr_data = { 
    .forces = synaptics_forces,
    };
*/
/*----------------------------------------------------------------------------*/
static int synaptics_ts_detect(struct i2c_client *client,
    int kind, struct i2c_board_info *info)
{         
    strcpy(info->type, TPD_DEVICE);                                                         

	TPDTM_DMESG("synaptics-rmi-ts driver_detect !!\n ");
	
    return 0;                                                                                       
}

/*----------------------------------------------------------------------------*/
static struct i2c_driver tpd_i2c_driver = {
	.probe		= synaptics_ts_probe,
	.remove		= synaptics_ts_remove,
//	.detect         = synaptics_ts_detect,                           
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= synaptics_ts_suspend,
	.resume		= synaptics_ts_resume,
#endif
	.id_table	= synaptics_ts_id,
//	.address_data = &synaptics_addr_data,
	.driver = {
//		.owner    = THIS_MODULE,
		.name	= TPD_DEVICE,
	},
};

static int synaptics_probe(struct platform_device *pdev) 
{
	boot_mode = get_boot_mode();
	printk("boot_mode = %d ............\n",boot_mode);
	// Software reset mode will be treated as normal boot
	if(boot_mode==3) boot_mode = NORMAL_BOOT;

    synaptics_wq = create_singlethread_workqueue("synaptics_wq");
    if (!synaptics_wq) {
        return -ENOMEM;
    }   

    if(i2c_add_driver(&tpd_i2c_driver)!=0) {
        TPDTM_DMESG("unable to add i2c driver.\n");
        return -1;
    }	
    return 0;
}
static int synaptics_remove(struct platform_device *pdev)
{
	i2c_del_driver(&tpd_i2c_driver);
	if (synaptics_wq)
		destroy_workqueue(synaptics_wq);
}



/* called when loaded into kernel */
static int __init tpd_driver_init(void) {


    i2c_register_board_info(0, &i2c_tpd, 1);

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
