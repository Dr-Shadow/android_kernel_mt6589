/************************************************************************************
** File: - mediatek\custom\common\kernel\touchpanel\synaptics\synaptics_tm1347_i2c_rmi.c
** VENDOR_EDIT
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
** 
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
#include <linux/kthread.h>

#include <mach/mt_pm_ldo.h>
#include <mach/mt_gpio.h>
//#include <mach/mt_devs.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>
#include <mach/eint.h>
#include <mach/pmic_mt6320_sw.h>

#include <cust_eint.h>

#include <linux/rtpm_prio.h>
#include <linux/sysfs.h>
#include "synaptics_update_12083.h"
#include <linux/oppo_devices_list.h>
#ifndef TPD_NO_GPIO 
#include "cust_gpio_usage.h"
#endif

#include "Rawdata.h"
#include <linux/proc_fs.h>
extern void lcm_change_paragrams(u8 i);
#include <linux/hrtimer.h>

#define TPD_DEBUG(a,arg...)\
	do{\
		if(tp_debug)\
			printk(TPD_DEVICE ": " a,##arg);\
	}while(0)
	
static struct workqueue_struct *synaptics_wq = NULL;
static struct proc_dir_entry *prEntry = NULL; 

#define TPDTM_DMESG(a,arg...) printk(TPD_DEVICE "(tm13472202): " a,##arg)

#define TPD_DEVICE "mtk-tpd2202"

static unsigned int tp_debug = 0; 
static unsigned int TP = 1; 		// tp manufacturer , tp = 1 : TRULY
int Disable_key_during_touch =0;
static unsigned int  lcd_display_change = 0;
unsigned int is_tp_init = 0;
extern unsigned int tp_is_in_charging;



static unsigned int boot_mode = 0;
static unsigned int is_suspend = 0;
static atomic_t is_touch = ATOMIC_INIT(0);
static void delay_qt_ms(unsigned long  w_ms);
//static DEFINE_SEMAPHORE(work_sem);
static DEFINE_SEMAPHORE(work_sem);

 int tp_charge_mode_set(int on);


#ifdef CONFIG_HAS_EARLYSUSPEND
static void synaptics_ts_early_suspend(struct early_suspend *h);
static void synaptics_ts_late_resume(struct early_suspend *h);
#endif
extern TP_DEV tp_dev;
extern int TP_FW;



#define NITTO_FIRMWARE_ID 	0x3039
#define OIKE_FIRMWARE_ID 	0x3041
#define OFILM_FIRMWARE_ID 	0x3051

#define TPD_USE_EINT

//extern void update_lcd();
#define VKNUMBER 3
static int tpd_keys[VKNUMBER][5] = 
{
	{KEY_MENU,80,1600,240,150},
	{KEY_HOME,380,1600,240,150},
	{KEY_BACK,680,1600,240,150},
};

#define MAX_POINT_NUM      10//must <=5
struct point_info 
{
    int x;
    int raw_x;
    int y;
    int raw_y;
    int z;
};

struct synaptics_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	int use_irq;
	struct hrtimer timer;
	struct work_struct  work;
	uint32_t flags;
	uint32_t btn_state;
	struct early_suspend early_suspend;
	struct 		hrtimer 	*syna_release_timer;
};

#define RELEASE_POINTER_MS 200

/* ========================================= */
/* implementation of  virtual key attribute */
/* ========================================= */
static ssize_t cap_vk_show(struct kobject *kobj, struct kobj_attribute *attr,
			char *buf){
//static ssize_t cap_vk_show(char *buf){
      /* LEFT: search: CENTER: menu ,home:search 412, RIGHT: BACK */
      return sprintf(buf,
        	__stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":65:545:70:70"
         ":" __stringify(EV_KEY) ":" __stringify(KEY_HOME)   ":165:545:70:70"
         ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":265:545:70:70"
         "\n");

}

static struct kobj_attribute mtk_virtual_keys_attr = 
{
    .attr = 
	{
		.name = "virtualkeys."TPD_DEVICE,
        .mode = S_IRUGO,
    },
    .show = &cap_vk_show,
};

static struct attribute *mtk_properties_attrs[] = 
{
    &mtk_virtual_keys_attr.attr,
    NULL
};

static struct attribute_group mtk_properties_attr_group = 
{
    .attrs = mtk_properties_attrs,
};

struct kobject *synaptics_properties_kobj;

static int synaptics_virtual_keys_init(struct synaptics_ts_data *ts)
{
	int ret;

	synaptics_properties_kobj = kobject_create_and_add("board_properties", NULL);
	
    if(synaptics_properties_kobj)
        ret = sysfs_create_group(synaptics_properties_kobj, &mtk_properties_attr_group);
	
    if(!synaptics_properties_kobj || ret)
	    printk("failed to create board_properties\n");		

	return ret;
}


static int synaptics_probe(struct platform_device *pdev);
static int synaptics_remove(struct platform_device *pdev);

static struct platform_driver synaptics_driver = {
    .remove     = synaptics_remove,
    //.shutdown   = NULL,
    .probe      = synaptics_probe,
	//#ifndef CONFIG_HAS_EARLYSUSPEND
	//.suspend    = NULL,
	//.resume     = NULL,
	//#endif
    .driver     = 
   	{
    //	.owner = THIS_MODULE,
        .name = "synaptics-platform",
    },
};


#ifdef TPD_USE_EINT
#define TOUCHPANEL_INT_PIN GPIO124
#endif


enum wk_wdt_type {
	WK_WDT_LOC_TYPE,
	WK_WDT_EXT_TYPE
};
extern void mtk_wdt_restart(enum wk_wdt_type type);
static int kick_wdt()
{
	mtk_wdt_restart(WK_WDT_LOC_TYPE);
	mtk_wdt_restart(WK_WDT_EXT_TYPE);

}





//synaptics base address
static uint8_t F11_2D_QUERY_BASE;
static uint8_t F11_2D_CMD_BASE;
static uint8_t F11_2D_CTRL_BASE;
static uint8_t F11_2D_DATA_BASE;

static uint8_t F01_RMI_QUERY_BASE;
static uint8_t F01_RMI_CMD_BASE;
static uint8_t F01_RMI_CTRL_BASE;
static uint8_t F01_RMI_DATA_BASE;

static uint8_t F34_FLASH_QUERY_BASE;
static uint8_t F34_FLASH_CMD_BASE;
static uint8_t F34_FLASH_CTRL_BASE;
static uint8_t F34_FLASH_DATA_BASE;

static uint8_t F54_ANALOG_QUERY_BASE;
static uint8_t F54_ANALOG_COMMAND_BASE;
static uint8_t F54_ANALOG_CONTROL_BASE;
static uint8_t F54_ANALOG_DATA_BASE;

static uint8_t F01_Product_ID_Query_0;
static uint8_t F01_Product_ID_Query_8;
static uint8_t F01_RMI_CTRL00;
static uint8_t F01_Interrupt_Status;

static uint8_t F11_2D_CTRL00;
static uint8_t F01_Interrupt_Enable;
static uint8_t F11_Max_X_L;
static uint8_t F11_Max_Y_L;
static uint8_t F11_2D_CTRL08;
static uint8_t F11_2D_DATA01;
static uint8_t F11_2D_CMD00;
static uint8_t F01_RMI_CMD00;
static uint8_t F34_FLASH_CTRL00;
static uint8_t F34_FLASH_QUERY00;
static uint8_t F34_FLASH_QUERY01;
static uint8_t F34_FLASH_QUERY03;
static uint8_t F34_FLASH_QUERY04;
static uint8_t F34_FLASH_QUERY05;
static uint8_t F34_FLASH_QUERY06;
static uint8_t F34_FLASH_QUERY07;
static uint8_t F34_FLASH_QUERY08;
static uint8_t F34_FLASH_DATA2_0;
static uint8_t F34_FLASH_DATA2_1;
static uint8_t F34_FLASH_DATA03;
static uint8_t F54_ANALOG_DATA00;
static uint8_t F54_ANALOG_DATA01;
static uint8_t F54_ANALOG_DATA02;
static uint8_t F54_ANALOG_DATA03;
static uint8_t F54_ANALOG_CMD00;
static uint8_t F54_ANALOG_QUERY00;
static uint8_t F54_ANALOG_QUERY01;
static uint8_t F54_ANALOG_CTRL41;
static uint8_t F54_ANALOG_CTRL57;

static int synaptics_read_register_map(struct i2c_client *client)
{
    uint8_t buf[4];   
    int ret;

    memset(buf, 0, sizeof(buf));
    ret = i2c_smbus_write_byte_data(client, 0xff, 0x0); 
    if (ret < 0) 
	{
	    TPDTM_DMESG("synaptics_read_register_map: failed for page select\n");
	    return -1;
    }
    ret = i2c_smbus_read_i2c_block_data(client, 0xDD, 4, &(buf[0x0]));
    F11_2D_QUERY_BASE = buf[0];
    F11_2D_CMD_BASE = buf[1];
    F11_2D_CTRL_BASE = buf[2]; 
    F11_2D_DATA_BASE = buf[3];

	printk("----------------------------------\n");
    printk("F11_2D_QUERY_BASE = 0x%x\n",F11_2D_QUERY_BASE);
    printk("F11_2D_CMD_BASE = 0x%x\n",F11_2D_CMD_BASE);
    printk("F11_2D_CTRL_BASE = 0x%x\n",F11_2D_CTRL_BASE);
    printk("F11_2D_DATA_BASE = 0x%x\n",F11_2D_DATA_BASE);

    ret = i2c_smbus_read_i2c_block_data(client, 0xE3, 4, &(buf[0x0]));    
    F01_RMI_QUERY_BASE = buf[0];
    F01_RMI_CMD_BASE = buf[1];
    F01_RMI_CTRL_BASE = buf[2]; 
    F01_RMI_DATA_BASE = buf[3];
	
	printk("----------------------------------\n");
    printk("F01_RMI_QUERY_BASE = 0x%x\n",F01_RMI_QUERY_BASE);
    printk("F01_RMI_CMD_BASE = 0x%x\n",F01_RMI_CMD_BASE);
    printk("F01_RMI_CTRL_BASE = 0x%x\n",F01_RMI_CTRL_BASE);
    printk("F01_RMI_DATA_BASE = 0x%x\n",F01_RMI_DATA_BASE);

    ret = i2c_smbus_read_i2c_block_data(client, 0xE9, 4, &(buf[0x0]));	  
    F34_FLASH_QUERY_BASE = buf[0];
    F34_FLASH_CMD_BASE = buf[1];
    F34_FLASH_CTRL_BASE = buf[2]; 
    F34_FLASH_DATA_BASE = buf[3];
	
	printk("----------------------------------\n");
    printk("F34_FLASH_QUERY_BASE = 0x%x\n",F34_FLASH_QUERY_BASE);
    printk("F34_FLASH_CMD_BASE = 0x%x\n",F34_FLASH_CMD_BASE);
    printk("F34_FLASH_CTRL_BASE = 0x%x\n",F34_FLASH_CTRL_BASE);
    printk("F34_FLASH_DATA_BASE = 0x%x\n",F34_FLASH_DATA_BASE);

    ret = i2c_smbus_write_byte_data(client, 0xff, 0x01); 
    if (ret < 0) 
	{
		TPDTM_DMESG("i2c_smbus_write_byte_data failed for page select\n");
		return -1;
    }
    ret = i2c_smbus_read_i2c_block_data(client, 0xE9, 4, &(buf[0x0]));
    F54_ANALOG_QUERY_BASE = buf[0];
    F54_ANALOG_COMMAND_BASE = buf[1];
    F54_ANALOG_CONTROL_BASE = buf[2];
    F54_ANALOG_DATA_BASE = buf[3];

	printk("----------------------------------\n");
    printk("F54_ANALOG_QUERY_BASE = 0x%x\n",F54_ANALOG_QUERY_BASE);
    printk("F54_ANALOG_COMMAND_BASE = 0x%x\n",F54_ANALOG_COMMAND_BASE);
    printk("F54_ANALOG_CONTROL_BASE = 0x%x\n",F54_ANALOG_CONTROL_BASE);
    printk("F54_ANALOG_DATA_BASE = 0x%x\n",F54_ANALOG_DATA_BASE);
	
    ret = i2c_smbus_write_byte_data(client, 0xff, 0x00); 
    
	F01_Product_ID_Query_0 = F01_RMI_QUERY_BASE+11;
	F01_Product_ID_Query_8 = F01_RMI_QUERY_BASE+19;	
	F01_RMI_CTRL00 = F01_RMI_CTRL_BASE;
	F01_RMI_CMD00 = F01_RMI_CMD_BASE;
	F01_Interrupt_Enable = F01_RMI_CTRL_BASE + 1;
	F01_Interrupt_Status = F01_RMI_DATA_BASE + 1;
	F11_2D_CTRL00 = F11_2D_CTRL_BASE;
	F11_Max_X_L = F11_2D_CTRL_BASE + 6;
	F11_Max_Y_L = F11_2D_CTRL_BASE + 8;
	F11_2D_DATA01 = F11_2D_DATA_BASE + 3;
	 F11_2D_CMD00 = F11_2D_CMD_BASE;
	F34_FLASH_CTRL00 = F34_FLASH_CTRL_BASE;
	F34_FLASH_QUERY00 = F34_FLASH_QUERY_BASE;
	F34_FLASH_QUERY01 = F34_FLASH_QUERY_BASE + 1;
	F34_FLASH_QUERY03 = F34_FLASH_QUERY_BASE + 3;
	F34_FLASH_QUERY04 = F34_FLASH_QUERY_BASE + 4;
	F34_FLASH_QUERY05 = F34_FLASH_QUERY_BASE + 5;
	F34_FLASH_QUERY06 = F34_FLASH_QUERY_BASE + 6;
	F34_FLASH_QUERY07 = F34_FLASH_QUERY_BASE + 7;
	F34_FLASH_QUERY08 = F34_FLASH_QUERY_BASE + 8;
	F34_FLASH_DATA2_0 = F34_FLASH_DATA_BASE + 2;
	F34_FLASH_DATA2_1 = F34_FLASH_DATA_BASE + 3;
	F34_FLASH_DATA03 = F34_FLASH_DATA_BASE + 18;
	F54_ANALOG_DATA00 = F54_ANALOG_DATA_BASE;
	F54_ANALOG_DATA01 = F54_ANALOG_DATA_BASE + 1;
	F54_ANALOG_DATA02 = F54_ANALOG_DATA_BASE + 2;
	F54_ANALOG_DATA03 = F54_ANALOG_DATA_BASE + 3;
	F54_ANALOG_CMD00  = F54_ANALOG_COMMAND_BASE;
	F54_ANALOG_QUERY00 = F54_ANALOG_QUERY_BASE;
	F54_ANALOG_QUERY01 = F54_ANALOG_QUERY_BASE + 1;
	F54_ANALOG_CTRL41 = F54_ANALOG_CONTROL_BASE + 0x8B;
	F54_ANALOG_CTRL57 = F54_ANALOG_CONTROL_BASE + 0x90;

}

uint8_t SynaF34DataBase;
uint8_t SynaF34QueryBase;
uint8_t SynaF34_FlashControl;
uint8_t SynaF34ReflashQuery_BootID;
uint8_t SynaF34DataBase;
uint8_t SynaF01DataBase;
uint8_t SynaF01CommandBase;
uint8_t SynaFirmwareBlockSize;
uint8_t SynaF34Reflash_BlockNum;
uint8_t SynaF34Reflash_BlockData;
uint8_t SynaF34ReflashQuery_FlashPropertyQuery;
uint8_t SynaF34ReflashQuery_FirmwareBlockSize;
uint8_t SynaF34ReflashQuery_FirmwareBlockCount;
uint8_t SynaF34ReflashQuery_ConfigBlockSize;
uint8_t SynaF34ReflashQuery_ConfigBlockCount;

void re_scan_PDT(struct i2c_client *client)
{
    uint8_t buf[8];

    printk("F34_FLASH_QUERY_BASE = 0x%x ,F01_RMI_QUERY_BASE = 0x%x,F11_2D_QUERY_BASE= 0x%x\n",F34_FLASH_QUERY_BASE,F01_RMI_QUERY_BASE,F11_2D_QUERY_BASE);
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
    i2c_smbus_read_i2c_block_data(client, SynaF34ReflashQuery_FirmwareBlockSize, 2, buf);
    SynaFirmwareBlockSize = buf[0] | (buf[1] << 8);
    SynaF34_FlashControl = SynaF34DataBase + SynaFirmwareBlockSize + 2;

}

static int synaptics_ts_update(struct i2c_client *client)
{
    uint8_t data_low8,data_high8;
    uint16_t block,firmware,configuration;
    uint8_t buf[8];
    int ret,i,j;
	
    i2c_smbus_write_byte_data(client, 0xff, 0x0);
    data_low8 = i2c_smbus_read_byte_data(client,F34_FLASH_QUERY03);
    data_high8 = i2c_smbus_read_byte_data(client,F34_FLASH_QUERY04);
    block = (data_high8 << 8)|data_low8;
    printk("block is %d \n",block);
    block = 16;

    data_low8 = i2c_smbus_read_byte_data(client,F34_FLASH_QUERY05);
    data_high8 = i2c_smbus_read_byte_data(client,F34_FLASH_QUERY06);
    firmware = (data_high8 << 8)|data_low8;
    printk("firmware is %d \n",firmware);
    firmware = 2816;

    data_low8 = i2c_smbus_read_byte_data(client,F34_FLASH_QUERY07);
    data_high8 = i2c_smbus_read_byte_data(client,F34_FLASH_QUERY08);
    configuration = (data_high8 << 8)|data_low8;
    printk("configuration is %d \n",configuration);
    configuration = 32;

    //Enter Flash
    //step1 read bootloader Id
    data_low8 = i2c_smbus_read_byte_data(client, F34_FLASH_QUERY00);
    data_high8 = i2c_smbus_read_byte_data(client,F34_FLASH_QUERY01);
    printk("bootloader id is %x \n",(data_high8 << 8)|data_low8);
    //step 2 writer bootloader id
    i2c_smbus_write_byte_data(client,F34_FLASH_DATA2_0,data_low8);
    i2c_smbus_write_byte_data(client,F34_FLASH_DATA2_1,data_high8);
    //step 3 Issue programe enable
    printk("F34_FLASH_DATA03 = 0x%x \n",F34_FLASH_DATA03);
    i2c_smbus_write_byte_data(client,F34_FLASH_DATA03,0x0F);
    //step 4 wait attn
    //delay_qt_ms(20);
    msleep(10);
    //SynaWaitATTN();
    printk("attn step 4\n");
    //step 2 check status
    ret = i2c_smbus_read_byte_data(client,F34_FLASH_DATA03);
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
   // mtk_wdt_restart();
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
        for(i=0;i<block;i++)
        {
           
             i2c_smbus_write_byte_data(client,SynaF34Reflash_BlockData+i, Firmware_Data[j][i]);
    
        }
        //c) issue write
        i2c_smbus_write_byte_data(client,SynaF34_FlashControl,0x02);
        //d) wait attn
        //delay_qt_ms(20);
        msleep(10);
        //SynaWaitATTN();
        ret = i2c_smbus_read_byte_data(client,SynaF34_FlashControl);
        printk("The status(Image) of flash data3 is %x,time =%d\n",ret,j);
        //mtk_wdt_restart();
		 kick_wdt();
    }
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
           
            i2c_smbus_write_byte_data(client,SynaF34Reflash_BlockData+i, Config_Data[j][i]);
           
        }
        //c) issue write
        i2c_smbus_write_byte_data(client,SynaF34_FlashControl,0x06);
        //d) wait attn
        //delay_qt_ms(20);
        msleep(10);
        ret = i2c_smbus_read_byte_data(client,SynaF34_FlashControl);
        printk("The status(Image) of flash data3 is %x,time =%d\n", ret,j);
        //mtk_wdt_restart();
		 kick_wdt();
    }

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
    
    synaptics_read_register_map(client);
    i2c_smbus_write_byte_data(client, F01_RMI_CMD00,0x01);
    msleep(150);

	printk("**************************************\n");
	printk("The TP FW Update Over\n");
	printk("**************************************\n");

}

static void tpd_down(struct synaptics_ts_data *ts, int finger, int raw_x, int raw_y, int x, int y, int p) 
{
    if(ts && ts->input_dev) 
	{
		if(p>0)
			Disable_key_during_touch = 1;
			
		else
			{
				if (hrtimer_active(ts->syna_release_timer))
						hrtimer_cancel(ts->syna_release_timer);
					hrtimer_start(ts->syna_release_timer,
						ktime_set((RELEASE_POINTER_MS /1000),
						(RELEASE_POINTER_MS % 1000) * 1000000), HRTIMER_MODE_REL);
			}
		input_report_key(ts->input_dev, BTN_TOUCH, 1);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, p);
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, (raw_x+raw_y)/2);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
		input_mt_sync(ts->input_dev);
		
//		printk("finger(%d): x = %4d; y = %4d; z = %4d\n", finger, x, y, p);
		TPD_DEBUG("finger(%d): x = %4d; y = %4d; z = %4d\n", finger, x, y, p);
    }  
}

static void tpd_up(struct synaptics_ts_data *ts, int finger, int raw_x, int raw_y, int x, int y, int p) 
{
	if(ts && ts->input_dev) 
	{
		input_report_key(ts->input_dev, BTN_TOUCH, 0);
		input_mt_sync(ts->input_dev);
		atomic_set(&is_touch,0);
		//printk(" --------All fingers UP----------\n");
		TPD_DEBUG(" --------All fingers UP----------\n");
	} 
	if (hrtimer_active(ts->syna_release_timer))
							hrtimer_cancel(ts->syna_release_timer);
						hrtimer_start(ts->syna_release_timer,
							ktime_set((RELEASE_POINTER_MS /1000),
							(RELEASE_POINTER_MS % 1000) * 1000000), HRTIMER_MODE_REL);

		if(tp_down > 1)
		{
		   //tp_reset(ts);
		   tpd_count++;
		   if(tpd_count > 100)
		   {
		       tpd_count = 0;
		   }
		   tp_down = 0;
		   printk("twe fingers up at the same time tpd_count  reset= %d!!!\n",tpd_count);
		  // tp_reset(ts);
		}
}

static void tpd_button(struct synaptics_ts_data *ts, 
						unsigned int x, unsigned int y, unsigned int z_state) 
{
    int i;
    if(z_state) 
	{
		for(i = 0;i < VKNUMBER;i++) 
		{
			if(x >= tpd_keys[i][1]&&
				x <= tpd_keys[i][1]+tpd_keys[i][3] &&
				y >= tpd_keys[i][2]&&
				y <= tpd_keys[i][2]+tpd_keys[i][4]&&
				!(ts->btn_state & (1<<i))) 
			{
				input_report_key(ts->input_dev, tpd_keys[i][0], 1);
				ts->btn_state |= (1<<i);
			}
		}
	} 
	else 
	{
		for(i = 0;i < VKNUMBER;i++) 
		{
			if(ts->btn_state & (1<<i)) 
			{
				input_report_key(ts->input_dev, tpd_keys[i][0], 0);
			}
		}
		ts->btn_state=0;
	}
}

static void tpd_hw_pwron(void)
{
	hwPowerOn(MT65XX_POWER_LDO_VGP6,VOL_3300,"TP"); 
	udelay(10);
    hwPowerOn(MT65XX_POWER_LDO_VGP4,VOL_1800,"TP");
	msleep(150);//hsy@oppo.com, add 2011/9/9 to speed up screen resume
    if(tp_first_power_on)
	{
		hwPowerOn(MT65XX_POWER_LDO_VGP6,VOL_3300,"CTP_3300"); 
		udelay(100);
		hwPowerOn(MT65XX_POWER_LDO_VGP4,VOL_1800,"CTP_1800");
	}
	else
	{
	    hwPowerDown(MT65XX_POWER_LDO_VGP6,"CTP_3300");
		msleep(100);
		hwPowerOn(MT65XX_POWER_LDO_VGP6,VOL_3300,"CTP_3300"); 
		udelay(100);
	}
	tp_first_power_on = 0;
	#if 0
	if(!tp_first_power_on)
	{
		mt_set_gpio_mode(124, 1);
		mt_set_gpio_dir(124,0);
		mt_set_gpio_pull_enable(124,1);
		mt_set_gpio_pull_select(124, 1);
		
			
		mt_set_gpio_mode(140, 0);
		mt_set_gpio_dir(140, 1);
		mt_set_gpio_out(140, 1); 
	}
		tp_first_power_on = 0;	
	#endif
	//udelay(1000);//hsy@oppo.com, add 2011/9/9 to speed up screen resume
}
static void tpd_hw_pwroff(void)
{
	hwPowerDown(MT65XX_POWER_LDO_VGP4,"TP");
	msleep(1);
    hwPowerDown(MT65XX_POWER_LDO_VGP6,"TP");
	msleep(1);
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

static int synaptics_read_product_id(struct synaptics_ts_data *ts)
{
	uint8_t buf1[15];
	int ret ;
	
	memset(buf1, 0 , sizeof(buf1));

    /* page select = 0x0 */
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0); 
	if (ret < 0) 
	{
		TPDTM_DMESG("synaptics_read_product_id: failed for page select\n");
		return -1;
	}
	
	ret = i2c_smbus_read_i2c_block_data(ts->client, F01_Product_ID_Query_0, 8, &(buf1[0x0]));
	ret = i2c_smbus_read_i2c_block_data(ts->client, F01_Product_ID_Query_8, 2, &(buf1[0x8]));
	
	if (ret < 0) 
	{
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

	if(tp_is_in_charging)
	/*device control: normal operation, no sleep, configure=1*/
	  ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CTRL_BASE, 0xa4); 
	else
		ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CTRL_BASE, 0x84); 
	if (ret < 0) {
		TPDTM_DMESG("i2c_smbus_write_byte_data failed for mode select\n");
		return -1;
	}
#if 0	
    //enable absolutePosFilter
	ret = i2c_smbus_write_byte_data(ts->client, F11_2D_CTRL_BASE, 0x08); /*Enable or disable enhance reporting mode  */
	if (ret < 0) {
		TPDTM_DMESG("i2c_smbus_write_byte_data failed for report mode select\n");
		return -1;
	}
#endif

	return ret;

}

static int synaptics_enable_interrupt(struct synaptics_ts_data *ts, int enable)
{

	int ret;
	uint8_t abs_status_int;
    /* page select = 0x0 */
	ret = i2c_smbus_write_byte_data(ts->client, 0xff, 0x0); 
	if(ret < 0) 
	{
		TPDTM_DMESG("synaptics_enable_interrupt: select page failed ret = %d\n",ret);
		return -1;
	}

	if(enable) 
	{
		abs_status_int = 0x04;

		/*clear interrupt bits for previous touch*/
		ret = i2c_smbus_read_byte_data(ts->client, F01_Interrupt_Status);
		if(ret < 0) 
		{
			TPDTM_DMESG("synaptics_enable_interrupt :clear interrupt bits failed\n");
			return -1;
		}
	} 
	else 
	{
		abs_status_int = 0x0;
	}	
	
	ret = i2c_smbus_write_byte_data(ts->client, F01_Interrupt_Enable, abs_status_int);
	if(ret < 0) 
	{
		TPDTM_DMESG("synaptics_enable_interrupt: enable or disable abs nterrupt failed, abs_int =%d\n",abs_status_int);
		return -1;
	}
	
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
static void get_id(void)
{
    int tmp0 = 0,tmp1 = 0; 
	mt_set_gpio_mode(86, 0); 
	mt_set_gpio_dir(86,0);
	mt_set_gpio_pull_enable(86, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(86, GPIO_PULL_UP);
	
	mt_set_gpio_mode(106, 0); 
	mt_set_gpio_dir(106,0);
	mt_set_gpio_pull_enable(106, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(106, GPIO_PULL_UP);
	
	tmp0 = mt_get_gpio_in(86);
    tmp1 = mt_get_gpio_in(106);
     printk("tmp0=%d,tmp1=%d\n",tmp0 ,tmp1);
	if(tmp0 == 0)
	{
        printk("tmp00\n");
	    tp_dev = TP_OIKE;
	}
	else
	{
           printk("tmp01\n");
	   if(tmp1 == 1)
	    tp_dev = TP_OIKE;
	   else
	    tp_dev = TP_OFILM; 
	}
	printk(" tp_dev = %d\n",tp_dev);
}
#endif



static void synaptics_ts_work_func(struct work_struct *work)
{
	int ret;
	uint8_t start_reg;
	uint8_t buf[5];
	uint8_t finger_num = 0;
    uint32_t finger_state = 0;
    uint8_t status = 0;
	struct point_info points;
	int i;
	
	struct synaptics_ts_data *ts = container_of(work, struct synaptics_ts_data, work);
	memset(buf,0,sizeof(buf));
	
	if(is_suspend)
	   {
	       printk("synaptics_ts_work_func is_suspend =%x\n",is_suspend);
	      return;
       }	
	//printk("shirendong work!\n");   
	down(&work_sem);
	atomic_set(&is_touch,1);
	ret = i2c_smbus_read_word_data(ts->client, F01_RMI_DATA_BASE);
	if (ret < 0) 
	{
		TPDTM_DMESG("synaptics_ts_work_func: i2c_transfer failed\n");
        goto ERR_I2C2;
	}
	status = ret&0xff;

	if(status)
	{
		//update_lcd();
		TPDTM_DMESG("synaptics_ts_work_func: status =%x\n",status);
		ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CMD_BASE,0x01);
		if(ret) 
		{
			TPDTM_DMESG("synaptics_ts_work_func:cannot  reset touch panel \n");
			goto ERR_I2C2;
		}
		//hwPowerOn(MT65XX_POWER_LDO_VIBR,VOL_2800,"CTP");
		delay_qt_ms(150);
		mt_set_gpio_dir(TOUCHPANEL_INT_PIN,GPIO_DIR_IN);
		mt_set_gpio_pull_enable(TOUCHPANEL_INT_PIN, GPIO_PULL_ENABLE);
		mt_set_gpio_pull_select(TOUCHPANEL_INT_PIN, GPIO_PULL_UP);

		mt65xx_eint_set_polarity(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_POLARITY_LOW);
		mt65xx_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM,	CUST_EINT_LEVEL_SENSITIVE);
		mt65xx_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN >> 4);
		mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, 
					CUST_EINT_DEBOUNCE_DISABLE, 
					CUST_EINT_POLARITY_LOW, 
					synaptics_ts_irq_handler, 
					0);

		ret = synaptics_enable_interrupt(ts, 1);
		TPDTM_DMESG("synaptics_ts_work_func222222\n");
		if(ret) 
		{
			TPDTM_DMESG("synaptics_ts_work_func:cannot  enable interrupt \n");
			goto ERR_I2C2 ;
		}
		synaptics_init_panel(ts);
		if (ret < 0) 
		{
			TPDTM_DMESG("synaptics_ts_work_func: control tm1400 to sleep failed\n");
			goto ERR_I2C2 ;
		}
		//hwPowerDown(MT65XX_POWER_LDO_VIBR,"CTP");
		mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
		goto ERR_I2C2 ;
	}

	ret = i2c_smbus_read_i2c_block_data(ts->client, F11_2D_DATA_BASE,3, &(buf[0]));
	if (ret < 0) 
	{
		TPDTM_DMESG("synaptics_ts_work_func: i2c_transfer failed\n");
        goto ERR_I2C2;
	}
    finger_state = ((buf[2]&0x0f)<<16)|(buf[1]<<8)|buf[0];	
	

	if(finger_state > 0)
	{
		for(i = 0;i < MAX_POINT_NUM;i++)
		{
			ret = i2c_smbus_read_i2c_block_data(ts->client, F11_2D_DATA_BASE + 3  + i*5, 5, &(buf[0]));
			if (ret < 0) 
			{
				TPDTM_DMESG("synaptics_ts_work_func: i2c_transfer failed\n");
	        	goto ERR_I2C2;
			}
			points.x = (buf[0]<<4) | (buf[2] & 0x0f);
			points.raw_x = buf[3]&0x0f;
			points.y = (buf[1]<<4) | ((buf[2] & 0xf0)>>4);
			points.raw_y = (buf[3]&0xf0)>>4;
		    points.z = buf[4];

			if(points.z)
			{
			    if( TP_FW  == 0x3030)	
				     tpd_down(ts, i, points.raw_x, points.raw_y, points.x,  1279 -  points.y, points.z);
				if(TP_FW  == 0x3031)	
				     tpd_down(ts, i, points.raw_x, points.raw_y, 719 - points.x,   points.y, points.z);
				else
				     tpd_down(ts, i, points.raw_x, points.raw_y,   points.x,   points.y, points.z);
			}

			if(boot_mode == FACTORY_BOOT)
			{
				tpd_button(ts,points.x,points.y,points.z);
			}
		}
	}
	else
	{
		tpd_up(ts, i, points.raw_x, points.raw_y, points.x, points.y, 0);

		if(boot_mode == FACTORY_BOOT)
		{
			tpd_button(ts,points.x,points.y,0);
		}

	}
	
	input_sync(ts->input_dev);
    up(&work_sem);
	mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	return;
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
	struct synaptics_ts_data *ts = container_of(timer, struct synaptics_ts_data, timer);

	queue_work(synaptics_wq, &ts->work);

	hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}
#else
static struct synaptics_ts_data *ts_g;
static void synaptics_ts_irq_handler(void)
{
	mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	queue_work(synaptics_wq, &ts_g->work);
}
#endif


 int tp_charge_mode_set(int on)
{
	int ret = 0;
    printk("tp_charge_mode_set on = %x",on);
    if(!is_suspend)	
	{
	     down(&work_sem); 
			printk("tp_set = %x",on);
	
			ret = synaptics_init_panel(ts_g);
		
			if (ret < 0) 
			{
				TPDTM_DMESG("tp_charge_mode_set failed\n");
				return -1;
			}
		up(&work_sem);
	}  
	return ret;
}





static ssize_t tp_debug_show(struct device_driver *ddri, char *buf)
{
    uint8_t ret = 0, ret1 = 0;
	int TPFW_ID = 0;
	ret = i2c_smbus_read_byte_data(ts_g->client, F34_FLASH_CTRL_BASE+2);
	ret1 = i2c_smbus_read_byte_data(ts_g->client, F34_FLASH_CTRL_BASE+3);
	TPFW_ID = (ret << 8) | ret1;
	
	ret = i2c_smbus_read_byte_data(ts_g->client, F01_RMI_DATA_BASE);
	if(ret < 0)
		printk("tp_show read i2c err\n");


	ret1= i2c_smbus_read_byte_data(ts_g->client, F01_RMI_CTRL_BASE);
	if(ret1< 0)
		printk("tp_show read i2c err\n");
	return sprintf(buf, "Device Status = 0x%x,TPFW_ID = 0x%x, DEVICE_CONTROL = 0x%x\n", ret,TPFW_ID,ret1);
}

static ssize_t tp_debug_store(struct device_driver *ddri, char *buf, size_t count)
{
	u8 tmp = 0;
	int ret;
	printk("tpd reset touch panel start \n");
	if (1 == sscanf(buf, "%d", &tmp))
	{
		tp_debug = tmp;
	}
	else
	{
		TPDTM_DMESG("invalid content: '%s', length = %d\n", buf, count);
	}
	
    if (tmp ==2)
	{
	    if(!is_suspend) 
		{
		    mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
		    printk("tpd reset touch panel \n");
		    ret = i2c_smbus_write_byte_data(ts_g->client, F01_RMI_CMD_BASE,0x01);
			if(ret) 
			{
				TPDTM_DMESG("tpd_re_check_kthread_func:cannot  reset touch panel \n");
			}
			//hwPowerOn(MT65XX_POWER_LDO_VIBR,VOL_2800,"CTP");
			delay_qt_ms(150);
			 //printk("tpd reset touch panel2 \n");
			ret = synaptics_enable_interrupt(ts_g, 1);
			if(ret) 
			{
				TPDTM_DMESG("tpd_re_check_kthread_func:cannot  enable interrupt \n");
			}
			//printk("tpd reset touch panel3 \n");
			synaptics_init_panel(ts_g);
			if (ret < 0) 
			{
				TPDTM_DMESG("tpd_re_check_kthread_func: control tm1400 to sleep failed\n");
			}
			printk("tpd reset touch panel end \n");
			mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
		}
	}
	return count;
}


static ssize_t  lcd_display_show(struct device_driver *ddri, char *buf)
{	
	return sprintf(buf, "%d\n", lcd_display_change );
}

static ssize_t  lcd_display_store(struct device_driver *ddri, char *buf, size_t count)
{
	u8 tmp = 0;
	
    printk("shirendong  lcd change paragarms \n");
	
	if (1 == sscanf(buf, "%d", &tmp))
	{
	    lcd_display_change  = tmp;
	    printk("temp = %d",tmp);
		lcm_change_paragrams(tmp);
	}
	else
	{
	   return sprintf(buf, "input number error\n");
	}

	return count;
}
static DRIVER_ATTR(oppo_tp_debug, 0664, tp_debug_show, tp_debug_store);
static DRIVER_ATTR(oppo_lcd_display, 0777, lcd_display_show, lcd_display_store);
#define TX_NUM 23
#define RX_NUM 13
int DiagonalUpperLimit = 1100;
int DiagonalLowerLimit = 900;

void checkCMD()
{
  int ret;
 do {

        delay_qt_ms(1); //wait 1ms

        ret =  i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE);
		
		printk("ret = %x\n",ret);

    } while (ret != 0x00); 
}

static ssize_t tp_test_show(struct device_driver *ddri, char *buf)
{   
    int ret = 0;
	int x,y;
	int16_t baseline_data;
	int16_t Rxdata[RX_NUM][RX_NUM] = {0};
	ssize_t num_read_chars = 0;
	uint16_t i = 0;
	uint8_t tmp_old = 0,tmp_new = 0;
	int error_piont = 0;
	//down(&work_sem);
	#if 0
    mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	ret = i2c_smbus_write_byte_data(ts_g->client, 0xff, 0x00); 
	ret = i2c_smbus_write_byte_data(ts_g->client, F01_RMI_CTRL00+1, 0x10); 
	ret = i2c_smbus_write_byte_data(ts_g->client, 0xff, 0x01); 
//step 1: forbid CBC & NoiseMitigation&Force Cal
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
	
	//step 1:check raw capacitance
	ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_DATA_BASE,0x03);
	ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0X01);//get report
	checkCMD();
    ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE);
	printk("Step 1 : F54_ANALOG_CMD00[2]=%d \n",ret);
	ret = i2c_smbus_write_word_data(ts_g->client,F54_ANALOG_DATA_BASE+1,0);
	for(x = 0;x < TX_NUM; x++)
	{
		i = 0;
		for(y = 0; y < RX_NUM; y++)
		{
			ret = i2c_smbus_read_word_data(ts_g->client,F54_ANALOG_DATA_BASE+3);
			baseline_data = ret & 0xffff;
			
			//printk("baseline_data[%d][%d] = %d\n", x, y, baseline_data);
			if(y == 16)
			{
				printk("------------------------------\n");
			}
			
			//num_read_chars += sprintf(&(buf[num_read_chars]), "%d ",baseline_data);
			if(tp_dev == TP_NITTO)
			{
			    printk("TP_NITTO\n");
				if((baseline_data >= NITTO_Rawdata[x][i])&& (baseline_data <= NITTO_Rawdata[x][i+1]))
					i += 2;
				else 
				{
				    i += 2;
					printk("The NITTO baseline_data[%d][%d] is error, the value is [%d], but the range is [%d] ~ [%d]! \n\n", 
							x ,y, baseline_data,NITTO_Rawdata[x][i], NITTO_Rawdata[x][i+1]);

					num_read_chars += sprintf(&(buf[num_read_chars]), "0 raw data erro");

					error_piont ++;
					if(error_piont == 2)
					{
						error_piont = 0;
						goto END;
					}
				}
			}
			if(tp_dev == TP_OIKE)
			{
			     printk("TP_OIKE\n");
				if((baseline_data >= OIKE_Rawdata[x][i])&& (baseline_data <= OIKE_Rawdata[x][i+1]))
					i += 2;
				else 
				{
					i += 2;
					printk("The OIKE baseline_data[%d][%d] is error, the value is [%d], but the range is [%d] ~ [%d]! \n\n", 
							x ,y, baseline_data, OIKE_Rawdata[x][i], OIKE_Rawdata[x][i+1]);

					num_read_chars += sprintf(&(buf[num_read_chars]), "0 raw data erro");

					error_piont ++;
					if(error_piont == 2)
					{
						error_piont = 0;
						goto END;
					}
				}
			}
			if(tp_dev == TP_OFILM)
			{
			  printk("TP_OFILM\n");
				if((baseline_data >= OFILM_Rawdata[x][i])&& (baseline_data <= OFILM_Rawdata[x][i+1]))
					i += 2;
				else 
				{
					i += 2;
					
					printk("The OFLIM baseline_data[%d][%d] is error, the value is [%d], but the range is [%d] ~ [%d]! \n\n", 
							x ,y, baseline_data, OFILM_Rawdata[x][i], OFILM_Rawdata[x][i+1]);

					num_read_chars += sprintf(&(buf[num_read_chars]), "0 raw data erro ");

					error_piont ++;
					if(error_piont == 2)
					{
						error_piont = 0;
						goto END;
					}
				}
			}
			
		}
	}

//step 2 :check tx-to-tx and tx-to-vdd
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_DATA_BASE,0x05);//select report type 0x05	 
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0X01);//get report
    checkCMD();
    ret = i2c_smbus_write_word_data(ts_g->client,F54_ANALOG_DATA_BASE+1,0);	
    ret = i2c_smbus_read_word_data(ts_g->client,F54_ANALOG_DATA_BASE+3);
    if( (ret&0x7ff) != 0)
	{
		printk("Step 2 error.\n");
		num_read_chars += sprintf(buf, "0 tx-tx-short or tx-vdd-short");
		goto END;
	}

//step 3 :check rx-to-rx
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_DATA_BASE,7);//select report type 0x07
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0X01);//get report
    checkCMD();
    ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE);//read report
	printk("Step 3 : F54_ANALOG_CMD00[2]=%d \n",ret);
	ret = i2c_smbus_write_word_data(ts_g->client,F54_ANALOG_DATA_BASE+1,0);
	for(x = 0;x < TX_NUM; x++)
	{
		for(y = 0; y < RX_NUM; y++)
		{
			Rxdata[x][y] = i2c_smbus_read_word_data(ts_g->client,F54_ANALOG_DATA_BASE+3);
			//num_read_chars += sprintf(&(buf[num_read_chars]), "%d ", Rxdata[x][y]);          
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
			Rxdata[ x + TX_NUM][y] = i2c_smbus_read_word_data(ts_g->client,F54_ANALOG_DATA_BASE+3);
		}
	}
	
//step 4:check rx-rx short



	for(x = 0;x < RX_NUM; x++)
	{
		for(y = 0; y < RX_NUM; y++)
		{
			if (x == y)
			{
                if((Rxdata[x][y] < DiagonalLowerLimit)|| (Rxdata[x][y] >DiagonalUpperLimit))
				{
					printk("Step 4 error, x = y = %d, the Rxdata[%d][%d] = %d, but the range is [%d] ~ [%d]\n", 
						x, x, y, Rxdata[x][y], DiagonalLowerLimit, DiagonalUpperLimit);

					num_read_chars += sprintf(buf, "0 rx-to-rx short or rx-to-vdd short Rxdata[%d][%d] = %d",x,y,Rxdata[x][y]);
					goto END;
				}
			}
		}
	}
 
	printk("Tp test success!\n");
	
	num_read_chars += sprintf(buf, "1"); 
	 
//step 5:reset touchpanel and reconfig the device
  END:	
   	ret = i2c_smbus_write_byte_data(ts_g->client, 0xff, 0x00); 
	ret = i2c_smbus_write_byte_data(ts_g->client,F01_RMI_CMD_BASE,0X01);
	delay_qt_ms(150);
	synaptics_init_panel(ts_g);
    synaptics_enable_interrupt(ts_g, 1);
	mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	
	//up(&work_sem);
	#endif
	num_read_chars += sprintf(buf, "0"); 
	return num_read_chars;
}

static ssize_t tp_test_store(struct device_driver *ddri, char *buf, size_t count)
{
   printk("tp_test_store is not support\n");
   return count;
}

static DRIVER_ATTR(oppo_tp_test, 0664, tp_test_show, tp_test_store);




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
    ssize_t num_read_chars_two = 0;
	
	down(&work_sem);
    mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);

	ret = i2c_smbus_write_byte_data(ts_g->client, 0xff, 0x01);
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
	  num_read_chars_two += sprintf(&(buf[num_read_chars_two]), "\n[%d]",x);
      for(y = 0; y < RX_NUM; y++)
   	  {
	    ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_DATA_BASE+3);
	    tmp_l = ret&0xff;
	    ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_DATA_BASE+3);
	    tmp_h = ret&0xff;
	    baseline_data[x][y] = (tmp_h<<8)|tmp_l;
		num_read_chars_two += sprintf(&(buf[num_read_chars_two]), "%d ",baseline_data[x][y]);
	    printk("%d,",baseline_data[x][y]);
	
	  }
   	}
	num_read_chars_two += sprintf(&(buf[num_read_chars_two]), "\nstep 2:report type2 delta image\n");
	printk("\nstep 2:report type2 delta image\n");
	
	ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_DATA_BASE,0x02);//select report type 0x02
	ret = i2c_smbus_write_word_data(ts_g->client,F54_ANALOG_DATA_BASE+1,0x00);//set fifo 00
    ret = i2c_smbus_write_byte_data(ts_g->client,F54_ANALOG_COMMAND_BASE,0X01);//get report
	checkCMD();
	count = 0;
	for(x = 0;x < TX_NUM; x++)
	{
		printk("\n[%d]",x);
		 num_read_chars_two += sprintf(&(buf[num_read_chars_two]), "\n[%d]",x);
		 for(y = 0; y < RX_NUM; y++)
		 {
			ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_DATA_BASE+3);
			tmp_l = ret&0xff;
			ret = i2c_smbus_read_byte_data(ts_g->client,F54_ANALOG_DATA_BASE+3);
			tmp_h = ret&0xff;
			delta_image[x][y]= (tmp_h<<8)|tmp_l;       		
			printk("%3d,",delta_image[x][y]);
			num_read_chars_two += sprintf(&(buf[num_read_chars_two]), "%d ",delta_image[x][y]);
		 }	 
	}

	END:
	ret = i2c_smbus_write_byte_data(ts_g->client, 0xff, 0x00);
	up(&work_sem);
	mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	return num_read_chars_two;
}

static ssize_t tp_delta_store(struct device_driver *ddri,
       char *buf, size_t count)
{
	  printk("tp_test_store is not support\n");
	  return count;
}

static DRIVER_ATTR(oppo_tp_delta_image, 0664, tp_delta_show, tp_delta_store);





















static int tp_write_func (struct file *file,const char *buffer, unsigned long count,void *data);

static int init_synaptics_proc()
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

static int tp_write_func (struct file *file,const char *buffer, unsigned long count,void *data)
{
  
	struct synaptics_ts_data *ts; 
	static char proc_syna_data[32];  	
	int ret = 0;
	TPDTM_DMESG("tp_write_func: is be called!\n");
	if(is_suspend||atomic_read(&is_touch))
		return count;
	down(&work_sem);

	if(ts_g)
	{
		ts = ts_g;
		ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CMD_BASE,0x01);
		if(ret) 
		{
			TPDTM_DMESG("synaptics_ts_resume: reset touch panel  \n");
			goto err;
		}
		//hwPowerOn(MT65XX_POWER_LDO_VIBR,VOL_2800,"CTP");
		TPDTM_DMESG("tp_write_func: delay_qt_ms is be called1111!\n");
		delay_qt_ms(50);
		TPDTM_DMESG("tp_write_func: msleeep is be called!\n");
        ret = synaptics_init_panel(ts);
		if (ret < 0) 
		{
			TPDTM_DMESG("synaptics_ts_resume: control tm1400 to sleep failed\n");
				goto err;
		}
		ret = synaptics_enable_interrupt(ts, 1);
		if(ret) 
		{
			TPDTM_DMESG("synaptics_ts_resume:cannot  enable interrupt \n");
	        goto err;
		}
		TPDTM_DMESG("tp_write_func: reset successed!\n");
	}
	err:
	up(&work_sem);
	return count;

}


static int lcd_write_func (struct file *file,const char *buffer, unsigned long count,void *data);
static int init_lcd_proc()
{
	int ret = 0;
	
	prEntry = create_proc_entry( "lcd_change", 0666, NULL );
	if(prEntry == NULL)
	{
		ret = -ENOMEM;
	  	printk(KERN_INFO"init_lcd_proc: Couldn't create proc entry\n");
	}
	else
	{
		prEntry->write_proc = lcd_write_func;
	}
	
	return ret;
}


static int lcd_write_func (struct file *file,const char *buffer, unsigned long count,void *data)
{
  	
	int ret = 0;
	printk("lcd_write_func: is be called!\n");
	u8 tmp = 0;
	
    printk("shirendong  lcd change paragarms \n");
	
	if (1 == sscanf(buffer, "%d", &tmp))
	{
	    lcd_display_change  = tmp;
	    printk("temp = %d",tmp);
		lcm_change_paragrams(tmp);
	}
	else
	{
	   return sprintf(buffer, "input number error\n");
	}

	return count;

}












/*zhangqiang add for TP FW don't UPdate when Lcd is not connected.*/
#ifdef VENDOR_EDIT 
//extern void tpfw_update_logo(int fw_num);
//extern int is_lcd_on(void);
#endif
static enum hrtimer_restart Release_pointer_handle(struct hrtimer *timer)
{
	#if 0
	if(Read_Gpio_TS_irq() != 0)
		release_all_pointers();
	#endif

	if(Disable_key_during_touch == 1)
		Disable_key_during_touch = 0;

	return HRTIMER_NORESTART;
}

unsigned int Touchpanel_Detected(void)
{
	printk("is_touch for key_touch is :%d\n",is_touch);
	int touch_detected= atomic_read(&is_touch);
	return touch_detected;
}

static int synaptics_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct synaptics_ts_data *ts;
	int ret = 0, ret1 =0;
	uint8_t buf[4]; 
	int max_x;
	int max_y;
	int FW_ID;
	//int tp_fw_update_when_LCD_on;
	
	TPDTM_DMESG("synaptics_ts_probe: enter !\n");
	tp_dev = TP_JDI;
	//get_id();
		#ifdef VENDOR_EDIT
	//Fuchun.Liao add
	mt_set_gpio_mode(140, 0);
    mt_set_gpio_dir(140, 1);
    mt_set_gpio_out(140, 1);
	mdelay(15);
	#endif /*VENDOR_EDIT*/
	
	struct 	hrtimer 	*release_timer;
	release_timer= kzalloc(sizeof(*release_timer), GFP_KERNEL);
	if (release_timer== NULL) {
		printk("synaptics i2c_probe Unable to allocate memory\n");
		
		return;
	}
	else{
		hrtimer_init(release_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		release_timer->function =Release_pointer_handle;
	}
	
	tpd_power(1);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		TPDTM_DMESG("synaptics_ts_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}
	
	init_synaptics_proc();
    init_lcd_proc();
	ret = i2c_smbus_write_byte_data(client, 0xff, 0x0);
	if (ret < 0) {
		TPDTM_DMESG("i2c_smbus_write_byte_data failed for page select\n");
	}

    ret = i2c_smbus_read_byte_data(client,F01_RMI_DATA_BASE);
	if(ret < 0)
	{
		printk("Synaptics tp DATE_BASE(Device Status) register is wrong, read it again ! \n");
		
		ret = i2c_smbus_read_byte_data(client,F01_RMI_DATA_BASE);
		if(ret < 0)
		{
			printk("Synaptics tp DATE_BASE(Device Status) register is still wrong ! \n");
			printk("Synaptics tp is no exist, power down the tp module!\n");
			
			tpd_power(0);
			
			return 0;
		}
	}
	
	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	memset(ts,0,sizeof(*ts));
	INIT_WORK(&ts->work, synaptics_ts_work_func);
	ts->client = client;
	i2c_set_clientdata(client, ts);

	synaptics_read_register_map(client);

	ret = i2c_smbus_read_byte_data(client, F34_FLASH_CTRL_BASE+2);
	ret1 = i2c_smbus_read_byte_data(client, F34_FLASH_CTRL_BASE+3);
	FW_ID = (ret << 8) | ret1;

	printk("The TP FW_ID = 0x%x\n", FW_ID);

/*zhangqiang add for TP FW don't UPdate when Lcd is not connected.*/
#ifdef VENDOR_EDIT 
//	tp_fw_update_when_LCD_on = is_lcd_on();

	//if(tp_fw_update_when_LCD_on == 1)


       

			//if(FW_ID != OIKE_FIRMWARE_ID)
		     // {	

//refresh2:	
if(boot_mode == NORMAL_BOOT)
{
	if(FW_ID < 0x3034)
   {
      refresh:  
           		    printk("**************************************\n");
        			printk("UPDATE The TP FW, Please wait\n");
        			printk("**************************************\n");
                    
            	//	tpfw_update_logo(1);

            		synaptics_ts_update(client);

			//		tpfw_update_logo(2);
				//}
   }
		
		
}
#endif

    ret = i2c_smbus_read_byte_data(client, F34_FLASH_CTRL_BASE+2);
	ret1 = i2c_smbus_read_byte_data(client, F34_FLASH_CTRL_BASE+3);
	FW_ID = (ret << 8) | ret1;
    TP_FW = FW_ID;
	printk("The TP FW_ID = 0x%x\n", FW_ID);
	
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

	ret = i2c_smbus_read_i2c_block_data(ts->client, F11_Max_X_L, 4, &(buf[0]));
	if (ret < 0) {
		TPDTM_DMESG("i2c_smbus_write_byte_data failed for reading max_x and max_y\n");
	}
	max_x = buf[0] | ((buf[1] & 0x0f) << 8);
	max_y = buf[2] | ((buf[3] & 0x0f) << 8);
	
	printk("TP max_x = %d, max_y = %d\n",max_x,max_y);
	
if(boot_mode == NORMAL_BOOT)
{	
	if(max_x != 719 || max_y != 1279)
	{
		printk("Synatics fw configuration failed!\n");
	
	    goto refresh;
	
	}
}
#if 0
/*zhangqiang add for TP FW don't UPdate when Lcd is not connected.*/
#ifdef VENDOR_EDIT 
		if(tp_fw_update_when_LCD_on == 1)
#endif			
		{
			printk("Synatics fw configuration failed and the same time ,Lcd is here, so reflesh the tp fw!\n");
		   if(tp_dev == TP_OIKE)
			     goto refresh2;
		   if(tp_dev == TP_NITTO)
			     goto refresh1;
		   if(tp_dev == TP_OFILM)
			     goto refresh3;
		}
#endif
	

	ts->input_dev = input_allocate_device();
	
	if (ts->input_dev == NULL) 
	{
		ret = -ENOMEM;
		printk(KERN_ERR "synaptics_ts_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	ts->input_dev->name = TPD_DEVICE;;
	ts->syna_release_timer = release_timer;
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(ABS_MT_TOUCH_MAJOR, ts->input_dev->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR,ts->input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, ts->input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, ts->input_dev->absbit);
	set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);


	
	/* For multi touch */
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, max_x, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, max_y, 0, 0);
	//input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, 1489, 0, 0);

#ifdef VENDOR_EDIT
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(KEY_SEARCH, ts->input_dev->keybit);
	set_bit(KEY_MENU, ts->input_dev->keybit);
	set_bit(KEY_HOME, ts->input_dev->keybit);
	set_bit(KEY_BACK, ts->input_dev->keybit);
#endif /* VENDOR_EDIT */

	input_set_drvdata(ts->input_dev, ts);
	
	if(input_register_device(ts->input_dev))
	{
		printk(KERN_ERR "synaptics_ts_probe: Failed to register input device\n");
		goto err_input_dev_register;
	}

#ifdef TPD_USE_EINT
	ts_g = ts;

	mt_set_gpio_mode(TOUCHPANEL_INT_PIN, GPIO_MODE_01);
	mt_set_gpio_dir(TOUCHPANEL_INT_PIN,GPIO_DIR_IN);
	mt_set_gpio_pull_enable(TOUCHPANEL_INT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(TOUCHPANEL_INT_PIN, GPIO_PULL_UP);
	
	mt65xx_eint_set_polarity(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_POLARITY_LOW);
	mt65xx_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM,	CUST_EINT_LEVEL_SENSITIVE);//level low pull up
	mt65xx_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM,(CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN >> 4));
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
	is_tp_init = 1;
	
	
	ts->use_irq = 1;
	
#else
	hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ts->timer.function = synaptics_ts_timer_func;
	hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
#endif

/*zhangqiang add for TP virtual key.*/
#ifdef VENDOR_EDIT 
	//synaptics_virtual_keys_init(ts);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
#ifndef VENDOR_EDIT
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
#else
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1;
#endif

	ts->early_suspend.suspend = synaptics_ts_early_suspend;
	ts->early_suspend.resume = synaptics_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	if (driver_create_file(&synaptics_driver.driver, &driver_attr_oppo_tp_debug))
	{            
		TPDTM_DMESG("driver_create_file failt\n");
		goto exit_init_failed;
	}
	
	/*if (driver_create_file(&synaptics_driver.driver, &driver_attr_oppo_lcd_display))
	{            
		TPDTM_DMESG("driver_create_file failt\n");
		goto exit_init_failed;
	}*/
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
	
#ifdef VENDOR_EDIT
	tpd_power(0);
#endif

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
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
	
	printk("============================ TP suspend is called\n");
	
    is_suspend = 1;
    down(&work_sem);
#ifdef TPD_USE_EINT
	mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
#else
	hrtimer_cancel(&ts->timer);
#endif

	ret = synaptics_enable_interrupt(ts, 0);
	if(ret) 
	{
		TPDTM_DMESG("synaptics_ts_suspend: cannot disable interrupt\n");
		goto err;
	}
	ret = cancel_work_sync(&ts->work);
	if(ret) 
	{
		TPDTM_DMESG("synaptics_ts_suspend: cannot disable work\n");
		goto err;
	}
    

		ret = i2c_smbus_write_byte_data(client, F01_RMI_CTRL_BASE, 0x10);

	if (ret < 0) 
	{
		TPDTM_DMESG("synaptics_ts_suspend: control tm1400 to sleep failed\n");
		goto err;
	}
	 #ifdef VENDOR_EDIT 
	 //Fuchun.Liao@wxkfDrv,2013/03/07,set TP_RESET low to reduce sleep current
	 mt_set_gpio_mode(140, 0);
     mt_set_gpio_dir(140, 1);
	 mt_set_gpio_out(140, 0);
	 mdelay(2);
    #endif /*VENDOR_EDIT*/
	tpd_power(0);
	up(&work_sem);
	return 0;
	
	err:
   	up(&work_sem);
	tpd_power(0);

	return 0;
}

static int synaptics_ts_resume(struct i2c_client *client)
{
	int ret;
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);

	printk("============================ TP resume is called\n");
  #ifdef VENDOR_EDIT
	//Fuchun.Liao add
	mt_set_gpio_mode(140, 0);
    mt_set_gpio_dir(140, 1);
    mt_set_gpio_out(140, 1);
	mdelay(10);
	#endif /*VENDOR_EDIT*/	
	tpd_power(1);
	
	Disable_key_during_touch = 0;
    down(&work_sem);
	if (!ts->use_irq) 
	{
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	} 
	else 
	{
		TPDTM_DMESG("synaptics_ts_resume: reset touch panel \n");
		ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CMD_BASE,0x01);
		if(ret) 
		{
			TPDTM_DMESG("synaptics_ts_resume: reset touch panel  \n");
			goto err;
		}
		//hwPowerOn(MT65XX_POWER_LDO_VIBR,VOL_2800,"CTP");
		msleep(50);
        ret = synaptics_init_panel(ts);
		if (ret < 0) 
		{
			TPDTM_DMESG("synaptics_ts_resume: control tm1400 to sleep failed\n");
				goto err;
		}
		
		ret = synaptics_enable_interrupt(ts, 1);
		if(ret) 
		{
			TPDTM_DMESG("synaptics_ts_resume:cannot  enable interrupt \n");
				goto err;
		}
    
       is_suspend = 0;
	   up(&work_sem);
    
		#ifdef TPD_USE_EINT
		mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);    
        #endif
		
		TPDTM_DMESG("synaptics_ts_resume:ok\n");
	}
	
	return 0;
	err:
	up(&work_sem);
	mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	return ret;
}

/*----------------------------------------------------------------------------*/
#ifdef CONFIG_HAS_EARLYSUSPEND
static void synaptics_ts_early_suspend(struct early_suspend *h)
{
	struct synaptics_ts_data *ts;

	printk("synaptics_ts_early_suspend is called\n");

	ts = container_of(h, struct synaptics_ts_data, early_suspend);
	synaptics_ts_suspend(ts->client, PMSG_SUSPEND);

}

static void synaptics_ts_late_resume(struct early_suspend *h)
{
	struct synaptics_ts_data *ts;

	printk("synaptics_ts_late_resume is called\n");

	ts = container_of(h, struct synaptics_ts_data, early_suspend);
	synaptics_ts_resume(ts->client);
}
#endif

/*----------------------------------------------------------------------------*/
static const struct i2c_device_id synaptics_ts_id[] = {
	{ TPD_DEVICE, 0 },
	{ }
};

static struct i2c_board_info __initdata i2c_tpd={ I2C_BOARD_INFO(TPD_DEVICE, (0xE0>>1))};
//static struct i2c_board_info __initdata i2c_tpd={ I2C_BOARD_INFO(TPD_DEVICE, (0xE0>>1))};

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
	
	printk("TP_______ boot_mode = %d\n",boot_mode);

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


