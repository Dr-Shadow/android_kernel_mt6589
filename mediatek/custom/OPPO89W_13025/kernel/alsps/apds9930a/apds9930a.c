/* drivers/hwmon/mt6516/amit/apds9930.c - APDS9930 ALS/PS driver
 * 
 * Author: MingHsien Hsieh <minghsien.hsieh@mediatek.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
//#include <mach/mt_gpio.h>
#ifdef MT6516
#include <mach/mt6516_devs.h>
#include <mach/mt6516_typedefs.h>
#include <mach/mt6516_gpio.h>
#include <mach/mt6516_pll.h>
#endif

#ifdef MT6573
#include <mach/mt6573_devs.h>
#include <mach/mt6573_typedefs.h>
#include <mach/mt6573_gpio.h>
#include <mach/mt6573_pll.h>
#endif

#ifdef MT6575
#include <mach/mt6575_devs.h>
#include <mach/mt6575_typedefs.h>
#include <mach/mt6575_gpio.h>
#include <mach/mt6575_pm_ldo.h>
#endif
#ifdef MT6577
#include <mach/mt6577_devs.h>
#include <mach/mt6577_typedefs.h>
#include <mach/mt6577_gpio.h>
#include <mach/mt6577_pm_ldo.h>
#endif

#ifdef MT6589
#include "mach/eint.h"
#include <mach/devs.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#endif
#ifdef MT6516
#define POWER_NONE_MACRO MT6516_POWER_NONE
#endif

#ifdef MT6573
#define POWER_NONE_MACRO MT65XX_POWER_NONE
#endif

#ifdef MT6575
#define POWER_NONE_MACRO MT65XX_POWER_NONE
#endif

#ifdef MT6577
#define POWER_NONE_MACRO MT65XX_POWER_NONE
#endif
#ifdef MT6589
#define POWER_NONE_MACRO MT65XX_POWER_NONE
#endif
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <asm/io.h>
#include <cust_eint.h>
#include <cust_alsps.h>
#include <linux/power_supply.h>
#include <linux/wakelock.h>
#include "apds9930a.h"
#include <linux/oppo_devices_list.h>
extern volatile ALSPS_DEV alsps_dev;
/******************************************************************************
 * configuration
 *******************************************************************************/
/*----------------------------------------------------------------------------*/

#define APDS9930_DEV_NAME     "APDS9930"
/*----------------------------------------------------------------------------*/
#define APS_TAG                  "[ALS/PS] "
#define APS_FUN(f)               printk(KERN_INFO APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    printk(KERN_ERR  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#ifndef VENDOR_EDIT
#define APS_LOG(fmt, args...)    printk(KERN_INFO APS_TAG fmt, ##args)
#else/*VENDOR_EDIT*/
#define APS_LOG(fmt, args...)    printk(KERN_ALERT APS_TAG fmt, ##args)
#endif/*VENDOR_EDIT*/
#define APS_DBG(fmt, args...)    printk(KERN_INFO APS_TAG fmt, ##args)                 
#define APS_PRINTK(fmt,args...)  if(debugflag) printk(APS_TAG fmt,##args)
#ifdef VENDOR_EDIT//mingqiang.guo@Prd.BasicDrv.Sensor, add 2012/10/13 for mult project sensor
#include "alsps_private_apds9930a.h"
//#include <linux/oppo_devices_list.h>
//extern ALSPS_DEV alsps_dev;
#endif /* VENDOR_EDIT*/
/******************************************************************************
 * extern functions
 *******************************************************************************/
/*for interrup work mode support --add by liaoxl.lenovo 12.08.2011*/
#ifdef MT6577
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern void mt65xx_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern unsigned int mt65xx_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt65xx_eint_registration(unsigned int eint_num, unsigned int is_deb_en, unsigned int pol, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
#endif
#ifdef MT6575
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_polarity(kal_uint8 eintno, kal_bool ACT_Polarity);
extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
        kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
        kal_bool auto_umask);

#endif
#ifdef MT6516
extern void MT6516_EINTIRQUnmask(unsigned int line);
extern void MT6516_EINTIRQMask(unsigned int line);
extern void MT6516_EINT_Set_Polarity(kal_uint8 eintno, kal_bool ACT_Polarity);
extern void MT6516_EINT_Set_HW_Debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 MT6516_EINT_Set_Sensitivity(kal_uint8 eintno, kal_bool sens);
extern void MT6516_EINT_Registration(kal_uint8 eintno, kal_bool Dbounce_En,
        kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
        kal_bool auto_umask);
#endif
/*----------------------------------------------------------------------------*/
struct i2c_client *apds9930_i2c_client = NULL;
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id apds9930_i2c_id[] = {{APDS9930_DEV_NAME,0},{}};
static struct i2c_board_info __initdata i2c_APDS9930={ I2C_BOARD_INFO("APDS9930", (0X72>>1))};
/*the adapter id & i2c address will be available in customization*/
//static unsigned short apds9930_force[] = {0x02, 0X72, I2C_CLIENT_END, I2C_CLIENT_END};
//static const unsigned short *const apds9930_forces[] = { apds9930_force, NULL };
//static struct i2c_client_address_data apds9930_addr_data = { .forces = apds9930_forces,};
/*----------------------------------------------------------------------------*/
static int apds9930_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int apds9930_i2c_remove(struct i2c_client *client);
static int apds9930_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
/*----------------------------------------------------------------------------*/
static int apds9930_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int apds9930_i2c_resume(struct i2c_client *client);


static struct apds9930_priv *g_apds9930_ptr = NULL;

struct PS_CALI_DATA_STRUCT
{
    int close;
    int far_away;
    int valid;
} ;

static struct PS_CALI_DATA_STRUCT ps_cali={0,0,0};
static int intr_flag_value = 0;
/*----------------------------------------------------------------------------*/
typedef enum {
    CMC_BIT_ALS    = 1,
    CMC_BIT_PS     = 2,
} CMC_BIT;
/*----------------------------------------------------------------------------*/
struct apds9930_i2c_addr {    /*define a series of i2c slave address*/
    u8  write_addr;  
    u8  ps_thd;     /*PS INT threshold*/
};
/*----------------------------------------------------------------------------*/
struct apds9930_priv {
    struct alsps_hw  *hw;
    struct i2c_client *client;
    struct work_struct  eint_work;

    /*i2c address group*/
    struct apds9930_i2c_addr  addr;

    /*misc*/
    u16		    als_modulus;
    atomic_t    i2c_retry;
    atomic_t    als_suspend;
    atomic_t    als_debounce;   /*debounce time after enabling als*/
    atomic_t    als_deb_on;     /*indicates if the debounce is on*/
    atomic_t    als_deb_end;    /*the jiffies representing the end of debounce*/
    atomic_t    ps_mask;        /*mask ps: always return far away*/
    atomic_t    ps_debounce;    /*debounce time after enabling ps*/
    atomic_t    ps_deb_on;      /*indicates if the debounce is on*/
    atomic_t    ps_deb_end;     /*the jiffies representing the end of debounce*/
    atomic_t    ps_suspend;


    /*data*/
    u16         als;
    u16          ps;
    u8          _align;
    u16         als_level_num;
    u16         als_value_num;
    u32         als_level[C_CUST_ALS_LEVEL-1];
    u32         als_value[C_CUST_ALS_LEVEL];

    atomic_t    als_cmd_val;    /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_cmd_val;     /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_thd_val_high;     /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_thd_val_low;     /*the cmd value can't be read, stored in ram*/
    ulong       enable;         /*enable mask*/
    ulong       pending_intr;   /*pending interrupt*/
    struct fasync_struct *fasync;

    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif     
};
/*----------------------------------------------------------------------------*/
static struct i2c_driver apds9930_i2c_driver = {	
    .probe      = apds9930_i2c_probe,
    .remove     = apds9930_i2c_remove,
    .detect     = apds9930_i2c_detect,
    /*OPPO zhye delete because use early_suspend*/
    //.suspend    = apds9930_i2c_suspend,
    //.resume     = apds9930_i2c_resume,
    .id_table   = apds9930_i2c_id,
    //	.address_data = &apds9930_addr_data,
    .driver = {
        //		.owner          = THIS_MODULE,
        .name           = APDS9930_DEV_NAME,
    },
};

static struct apds9930_priv *apds9930_obj = NULL;
static struct platform_driver apds9930_alsps_driver;

bool Get_PsState(void)
{
    bool status;
    if( NULL == apds9930_obj ) {
        status =  false;
    } else {
        if( test_bit(CMC_BIT_PS, &apds9930_obj->enable) ) {

            status = true;
        } else {
            status = false;
        }        
    }

    return status;
}
EXPORT_SYMBOL(Get_PsState);

#ifdef VENDOR_EDIT
//mingqiang.guo@prd.BasicDrv.Sensor 

#define PS_HIGH       0xffff
#define PS_LOW        0x0

static u16 g_ps_low_threshold = PS_LOW;
static u16 g_ps_high_threshold = PS_HIGH;
static u16 g_is_far = 1; 
static int g_DEVICE_ID = CUSTOM_DEVICE_ID;//0x29;//0x39;

static int g_ALS_GAIN = 0;
static unsigned int g_PPCOUNT_VALUE = 0;
static int g_CONTROL_VALUE = 0;

//LiuPing@Prd.BasicDrv.Sensor, add 2012/12/05 add for ps switch 
static int g_ps_switch_off = 0;  // 0: ps switch on  1: ps switch off

//#define ALSPS_CALI_DATA_RELOAD
//LiuPing@Prd.BasicDrv.Sensor, add 2013/03/06 , solve that cali data don't reload after alsps repower. 
#ifdef ALSPS_CALI_DATA_RELOAD
static char g_ppcount_val_bak = 0;
static char g_offset_val_bak = 0;
#endif

struct wake_lock g_prox_wake_lock; 
static DEFINE_MUTEX(set_ps_mutex);
#define TMD_TRANS_WORD(x) (0xa0 | x)

#define APDS9930_READ_WORD(client,addr)    \
    i2c_smbus_read_word_data(client, TMD_TRANS_WORD(addr))

#endif/*VENDOR_EDIT*/	

#ifdef VENDOR_EDIT//Shaoyu.Huang@BadicDrv.Sensor, add 2012/6/4 for sensor debuging
/*----------------------------------------------------------------------------*/
static struct alsps_hw* get_cust_alsps_hw_apds(void)
{
#if 0
    if (g_DEVICE_ID == 0x39){
        return get_cust_alsps_hw_apds9930a();
    } else {
        return get_cust_alsps_hw_apds9930();
    }
#else
        return get_cust_alsps_hw_apds9930a();
#endif
}

int apds9930_write_reg(char reg, char val)
{
    char databuf[2];
    int ret;

    databuf[0] = reg;
    databuf[1] = val;

    ret = i2c_master_send(apds9930_i2c_client, databuf, 0x2);

    if(ret <= 0)
    {
        APS_ERR("%s failed reg 0x%x val 0x%x \n", __func__, reg, val);
        return APDS9930_ERR_I2C;
    }
    else
    {
        return APDS9930_SUCCESS;
    }
}
int apds9930_read_reg(char reg, char *val)
{
    int ret;
#if 0
    ret = i2c_master_send(apds9930_i2c_client, &reg, 0x1);
    if(ret <= 0)
    {
        APS_ERR("%s failed reg 0x%x \n", __func__, reg);
        return APDS9930_ERR_I2C;
    }

    ret = i2c_master_recv(apds9930_i2c_client, val, 0x1);
    if(ret <= 0)
    {
        APS_ERR("%s failed reg 0x%x \n", __func__, reg);
        return APDS9930_ERR_I2C;
    }
#else
    *val = i2c_smbus_read_byte_data(apds9930_i2c_client, reg);
    if (*val < 0)
    {
        APS_ERR("%s failed reg 0x%x \n", __func__, reg);
        return APDS9930_ERR_I2C;
    }
#endif

        return APDS9930_SUCCESS;

}

static ssize_t set_reg_write(struct file *file, char __user *buf, size_t count, loff_t *pos)
{
    char tmp[128] = {0};	
    int ret;		
    int reg, val;
    char databuf[2];
    struct i2c_client *client = apds9930_i2c_client;

    ret = copy_from_user(tmp, buf, 16);	
    APS_LOG("%s : %s\n", __func__, tmp);

    sscanf(tmp, "%x:%x", &reg, &val);
    APS_LOG("reg_set_write %x %x\n", reg, val);

    apds9930_write_reg(reg, val);

    //LiuPing@Prd.BasicDrv.Sensor, add 2013/03/06 , solve that cali data don't reload after alsps repower.  so need to save the cali datas.
#ifdef ALSPS_CALI_DATA_RELOAD
    if (reg == APDS9930_CMM_PS_OFFSET)
    {
        g_offset_val_bak = val;
    }
    if (reg == APDS9930_CMM_PPCOUNT)
    {
        g_ppcount_val_bak = val;
    }
#endif

    return count;	
}

static ssize_t read_all_reg(struct file *file, char __user *buf, size_t count, loff_t *pos)
{
    char page[1024]; 	
    char *p = page;	
    int len = 0; 	
    struct i2c_client *client = apds9930_i2c_client;
    u8 buffer[1], val[1];
    int i = 0;

    for (i = 0; i <= 0x1f; i++) {
        buffer[0] = i | 0x80;
        //i2c_master_send(client, buffer, 0x1);
        //i2c_master_recv(client, val, 0x1);
        val[0] = i2c_smbus_read_byte_data(client, buffer[0]);
        p += sprintf(p, "reg 0x%x:0x%x\n", i, val[0]);	
    }

    len = p - page;	
    if (len > *pos)		
        len -= *pos;	
    else		
        len = 0;	

    if (copy_to_user(buf, page, len < count ? len  : count))		
        return -EFAULT;	

    *pos = *pos + (len < count ? len  : count);	

    return len < count ? len  : count;
}

static ssize_t read_one_reg(struct file *file, char __user *buf, size_t count, loff_t *pos)
{
    char tmp[10] = {0};	
    int ret;		
    int reg;
    struct i2c_client *client = apds9930_i2c_client;
    u8 buffer[1], val[1];

    if (count > 5) {
        APS_ERR("%s : %d", __func__, count);
        return -EINVAL;
    }
    ret = copy_from_user(tmp, buf, 4);	

    sscanf(tmp, "%x", &reg);

    buffer[0] = reg;
    //i2c_master_send(client, buffer, 0x1);
    //i2c_master_recv(client, val, 0x1);
    val[0] = i2c_smbus_read_byte_data(client, buffer[0]);

    APS_LOG("**********APDS9930 reg read %x :%x***********\n", reg, val[0]);

    return count;	
}

static ssize_t ps_thd_write(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    char tmp[128] = {0};	
    int ret;		
    int reg, val;
    struct alsps_hw *hw = get_cust_alsps_hw_apds();

    ret = copy_from_user(tmp, buf, 16);	

    APS_LOG("%s : %s", __func__, tmp);
    printk("printk %s : %s", __func__, tmp);

    sscanf(tmp, "%d:%d:%d:%d:%d:%d:%d", 
            &hw->p_ps_adjust_para->ps_up, 
            &hw->p_ps_adjust_para->ps_thd_low_notrend, 
            &hw->p_ps_adjust_para->ps_thd_high_notrend,
            &hw->p_ps_adjust_para->ps_thd_low_trend, 
            &hw->p_ps_adjust_para->ps_thd_high_trend,
            &hw->p_ps_adjust_para->ps_thd_low_highlight, 
            &hw->p_ps_adjust_para->ps_thd_high_highlight);

    return count;
}


//LiuPing@Prd.BasicDrv.Sensor, add 2012/12/05 add for ps switch 
static ssize_t show_ps_switch(struct file *file, char __user *buf, size_t count, loff_t *pos)
{
    char page[1024]; 	
    char *p = page;	
    int len = 0; 

    printk("show_ps_switch ,  g_ps_switch_off:%d \n", g_ps_switch_off);
    p += sprintf(p, "%d\n", g_ps_switch_off);	


    len = p - page;	
    if (len > *pos)		
        len -= *pos;	
    else		
        len = 0;	

    if (copy_to_user(buf, page, len < count ? len  : count))		
        return -EFAULT;	

    *pos = *pos + (len < count ? len  : count);	

    return len < count ? len  : count;
}
static ssize_t set_ps_switch(struct file *file, char __user *buf, size_t count, loff_t *pos)
{
    char tmp[128] = {0};	
    int ret;		
    int reg, val;

    ret = copy_from_user(tmp, buf, 16);	
    APS_LOG("%s : %s\n", __func__, tmp);

    sscanf(tmp, "%d", &val);
    APS_LOG("set_ps_switch %d\n", val);

    if (val == 0 || val == 1)
    {
        g_ps_switch_off = val;
    }
    else
    {
        g_ps_switch_off = 0; // default ps switch on    	
    }

    return count;	
}

static long apds9930_enable_ps(struct i2c_client *client, int enable);
long apds9930_read_ps(struct i2c_client *client, u16 *data);
static ssize_t read_pulse_count(struct file *file, char __user *buf, size_t count, loff_t *pos)
{
    char page[4024]; 	
    char *p = page;	
    int len = 0; 	
    u16 ps = 0;
    u8  last_ps_enable_status ;
    u8 offset_value=0;

    last_ps_enable_status = test_bit(CMC_BIT_PS, &apds9930_obj->enable) ;
    if(last_ps_enable_status == 0 ) // if ps is disable ,need to enable ps 
        apds9930_enable_ps(apds9930_i2c_client,1); 
    msleep(100);

    //p= page+30;
#ifndef OPPO_CMCC_TEST
    g_PPCOUNT_VALUE = APDS9930A_PPCOUNT; 
    apds9930_write_reg(APDS9930_CMM_PPCOUNT,g_PPCOUNT_VALUE ) ;

    for(offset_value=0x7f; offset_value<0x80;offset_value -= 0x10)
    {
        APDS9930_SUCCESS != apds9930_write_reg(APDS9930_CMM_PS_OFFSET,offset_value) ;
        msleep(100);
        apds9930_read_ps(apds9930_i2c_client,&ps);
        p += sprintf(p, "%d:%x:%d,in cali 1\n", g_PPCOUNT_VALUE,offset_value,ps);	

        //if(ps>100 && ps<600)
        if(ps>200 && ps<500)
        {
            //LiuPing@Prd.BasicDrv.Sensor, add 2013/03/06 , solve that cali data don't reload after alsps repower.  so need to save the cali datas.
#ifdef ALSPS_CALI_DATA_RELOAD
            g_offset_val_bak = offset_value;
            g_ppcount_val_bak = g_PPCOUNT_VALUE;
#endif            
            p += sprintf(page, "%d:%d:%d ,cali ok tao\n", g_PPCOUNT_VALUE,offset_value,ps);	
            break;
        }
    }
#if 1    
    if(ps<200 || ps>500)
    {
        if(ps>500)
        {
            offset_value = 0x7f;
            apds9930_write_reg(APDS9930_CMM_PS_OFFSET,offset_value);
            msleep(100);

            //for(g_PPCOUNT_VALUE=15;g_PPCOUNT_VALUE<16;g_PPCOUNT_VALUE -=5) //15 10 5    
            for(g_PPCOUNT_VALUE=18;g_PPCOUNT_VALUE<19;g_PPCOUNT_VALUE -=2)
            {
                if(g_PPCOUNT_VALUE==0)
                {
                    g_PPCOUNT_VALUE =8; 
                    //default setting
#ifdef ALSPS_CALI_DATA_RELOAD
                    g_offset_val_bak = 0x7f;
                    g_ppcount_val_bak = 8;
#endif 

                    p += sprintf(page, "%d:%d:%d ,cali err tao\n", g_PPCOUNT_VALUE,offset_value,ps);	
                    break;
                }
                apds9930_write_reg(APDS9930_CMM_PPCOUNT,g_PPCOUNT_VALUE );
                msleep(100);
                apds9930_read_ps(apds9930_i2c_client,&ps);
                p += sprintf(p, "%d:%x:%d ,in cali 2\n", g_PPCOUNT_VALUE,offset_value,ps);	
                if(ps>200 && ps<500)
                {
#ifdef ALSPS_CALI_DATA_RELOAD
                    g_offset_val_bak = offset_value;
                    g_ppcount_val_bak = g_PPCOUNT_VALUE;
#endif 
                    p += sprintf(page, "%d:%d:%d ,cali ok tao\n", g_PPCOUNT_VALUE,offset_value,ps);	
                    break;
                }
            }
        }else if( ps< 200 )
        {
            offset_value = 0x0;
            apds9930_write_reg(APDS9930_CMM_PS_OFFSET,offset_value);
            msleep(100);

            //for(g_PPCOUNT_VALUE=15;g_PPCOUNT_VALUE<16;g_PPCOUNT_VALUE -=5) //15 10 5    
            for(g_PPCOUNT_VALUE=20;g_PPCOUNT_VALUE<30;g_PPCOUNT_VALUE +=2)
            {               
                apds9930_write_reg(APDS9930_CMM_PPCOUNT,g_PPCOUNT_VALUE );
                msleep(100);
                apds9930_read_ps(apds9930_i2c_client,&ps);
                p += sprintf(p, "%d:%x:%d ,in cali 2\n", g_PPCOUNT_VALUE,offset_value,ps);	
                if(ps>200 && ps<500)
                {
#ifdef ALSPS_CALI_DATA_RELOAD
                    g_offset_val_bak = offset_value;
                    g_ppcount_val_bak = g_PPCOUNT_VALUE;
#endif 
                    p += sprintf(page, "%d:%d:%d ,cali ok tao\n", g_PPCOUNT_VALUE,offset_value,ps);	
                    break;
                }
            }
            //default setting
#ifdef ALSPS_CALI_DATA_RELOAD
            g_offset_val_bak = 0;
            g_ppcount_val_bak = 30;
#endif 
        }


    }
#endif    
#else
    offset_value = 0x7f;
    apds9930_write_reg(APDS9930_CMM_PS_OFFSET,offset_value);
    msleep(100);

    for(g_PPCOUNT_VALUE=25;g_PPCOUNT_VALUE<26;g_PPCOUNT_VALUE -=5) //25 20 15 10 5    
    {
        if(g_PPCOUNT_VALUE==0)
        {
            g_PPCOUNT_VALUE =5;    
            p += sprintf(page, "%d:%d:%d ,cali err notao\n", g_PPCOUNT_VALUE,offset_value,ps);	
            break;
        }
        apds9930_write_reg(APDS9930_CMM_PPCOUNT,g_PPCOUNT_VALUE );
        msleep(100);
        apds9930_read_ps(apds9930_i2c_client,&ps);
        p += sprintf(p, "%d:%x:%d ,in cali\n", g_PPCOUNT_VALUE,offset_value,ps);	
        if(ps>100 && ps<600)
        {
            p += sprintf(page, "%d:%d:%d ,cali ok notao\n", g_PPCOUNT_VALUE,offset_value,ps);	
            break;
        }
    }
#endif

    if(last_ps_enable_status == 0)// need to disable ps after read ps 
        apds9930_enable_ps(apds9930_i2c_client,0); 

    len = p - page;	
    if (len > *pos)		
        len -= *pos;	
    else		
        len = 0;	

    if (copy_to_user(buf, page, len < count ? len  : count))		
        return -EFAULT;	

    *pos = *pos + (len < count ? len  : count);	//update file inode pos parameter 

    return len < count ? len  : count;
}

static struct file_operations set_reg_if = {
    .write = set_reg_write,
};
static struct file_operations read_reg_if = {
    .write = read_one_reg,
    .read = read_all_reg,
};
static struct file_operations ps_thd_if = {
    .write = ps_thd_write,
};

static struct file_operations cali_pulse_count_fops = {
    .write =set_reg_write, 
    .read = read_pulse_count,
};

//LiuPing@Prd.BasicDrv.Sensor, add 2012/12/05 add for ps switch 
static struct file_operations ps_switch_fops = {
    .write =set_ps_switch,   
    .read = show_ps_switch,
};


#ifdef VENDOR_EDIT//mingqiang.guo@prd.BasicDrv.Sensor, 2012/11/12 add for read_ic_model
static ssize_t read_ic_model(struct file *file, char __user *buf, size_t count, loff_t *pos)
{	

    char page[1024]; 	
    char *p = page;	
    int len = 0; 	
    //if(g_DEVICE_ID == 0x39 )
        p += sprintf(p, "apds9930a\n");	
   // else
   //     p += sprintf(p, "apds9930\n");	
    len = p - page;	
    if (len > *pos)		
        len -= *pos;	
    else		
        len = 0;	

    if (copy_to_user(buf,page,len < count ? len  : count))		
        return -EFAULT;	
    *pos = *pos + (len < count ? len  : count);	

    return len < count ? len  : count;

}

static struct file_operations ic_model_fops = {
    .read = read_ic_model,
};
#endif/*VENDOR_EDIT*/

static int debugflag = 0;
static ssize_t debugflag_read(struct file *file, char __user *buf, size_t count, loff_t *pos)
{	
    char page[8]; 	
    char *p = page;	
    int len = 0; 	
    p += sprintf(p, "%d\n", debugflag);	
    len = p - page;	
    if (len > *pos)		
        len -= *pos;	
    else		
        len = 0;	

    if (copy_to_user(buf,page,len < count ? len  : count))		
        return -EFAULT;	
    *pos = *pos + (len < count ? len  : count);	

    return len < count ? len  : count;
}

static ssize_t debugflag_write(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{	
    char tmp[2] = {0, 0};	
    int ret;		
    if (count > 2)		
        return -EINVAL;		
    ret = copy_from_user(tmp, buf, 2);	
    if ('1' == tmp[0])		
        debugflag = 1;	
    else if ('0' == tmp[0])		
        debugflag = 0;	
    else		
        return -EINVAL;		
    return count;	
}
static struct file_operations debug_fops = {
    .read = debugflag_read,
    .write = debugflag_write,
};


static int als_c0_data = 0;
static int als_c1_data = 0;
static int als_data0 = 0;
static ssize_t als_data_read(struct file *file, char __user *buf, size_t count, loff_t *pos)
{	
    char page[80]; 	
    char *p = page;	
    int len = 0; 	
    p += sprintf(p, "c0=%d ,c1=%d,als=%d\n", als_c0_data,als_c1_data,als_data0);	
    len = p - page;	
    if (len > *pos)		
        len -= *pos;	
    else		
        len = 0;	

    if (copy_to_user(buf,page,len < count ? len  : count))		
        return -EFAULT;	
    *pos = *pos + (len < count ? len  : count);	

    return len < count ? len  : count;
}

static struct file_operations als_read_fops = {
    .read = als_data_read,
};

#endif/*VENDOR_EDIT*/

/*----------------------------------------------------------------------------*/

int apds9930_get_addr(struct alsps_hw *hw, struct apds9930_i2c_addr *addr)
{
    if(!hw || !addr)
    {
        return -EFAULT;
    }
    addr->write_addr= hw->i2c_addr[0];
    return 0;
}
/*----------------------------------------------------------------------------*/
static void apds9930_power(struct alsps_hw *hw, unsigned int on) 
{
    static unsigned int power_on = 0;
    APS_LOG(" %s()\n", __FUNCTION__);
    //APS_LOG("power %s\n", on ? "on" : "off");
#ifndef VENDOR_EDIT//Shaoyu.Huang@BasicDrv.Sensor, modify 2012/5/24 for alsps power
    if(hw->power_id != POWER_NONE_MACRO)
    {
        if(power_on == on)
        {
            APS_LOG("ignore power control: %d\n", on);
        }
        else if(on)
        {
            if(!hwPowerOn(hw->power_id, hw->power_vol, "APDS9930")) 
            {
                APS_ERR("power on fails!!\n");
            }
        }
        else
        {
            if(!hwPowerDown(hw->power_id, "APDS9930")) 
            {
                APS_ERR("power off fail!!\n");   
            }
        }
    }
    power_on = on;
#else/*VENDOR_EDIT*/
    hw->power(hw, on, NULL);
#endif/*VENDOR_EDIT*/
}
/*----------------------------------------------------------------------------*/
static long apds9930_enable_als(struct i2c_client *client, int enable)
{
    struct apds9930_priv *obj = i2c_get_clientdata(client);
    uint32_t testbit_PS;

    if (atomic_read(&obj->als_suspend)) {
        return 0;
    }

    APS_LOG(" %s(), enable = %d\n", __FUNCTION__,enable);
    if(client == NULL)
    {
        APS_LOG("CLIENT CANN'T EQUL NULL\n");
        return -1;
    }

    /*yucong MTK enable_als function modified for fixing reading register error problem 2012.2.16*/
    testbit_PS = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
    APS_LOG(" %s(),  enable = %d, testbit_PS = %d\n",__FUNCTION__,enable,testbit_PS);
    if(enable)
    {
        if(testbit_PS){	
            if (APDS9930_SUCCESS != apds9930_write_reg(APDS9930_CMM_ENABLE, 0x2F))
            {
                goto EXIT_ERR;
            }
        }
        else{
            if (APDS9930_SUCCESS != apds9930_write_reg(APDS9930_CMM_ENABLE, 0x2B))
            {
                goto EXIT_ERR;
            }
        }
        atomic_set(&obj->als_deb_on, 1);
        atomic_set(&obj->als_deb_end, jiffies+atomic_read(&obj->als_debounce)/(1000/HZ));
        APS_LOG("apds9930 power on\n");
    }
    else
    {	
        if(testbit_PS){
            if (APDS9930_SUCCESS != apds9930_write_reg(APDS9930_CMM_ENABLE, 0x2F))
            {
                goto EXIT_ERR;
            }
        }
        else{
            if (APDS9930_SUCCESS != apds9930_write_reg(APDS9930_CMM_ENABLE, 0x00))
            {
                goto EXIT_ERR;
            }
        }

        /*Lenovo-sw chenlj2 add 2011-06-03,modify ps_deb_on to als_deb_on */
        atomic_set(&obj->als_deb_on, 0);
        APS_LOG("apds9930 power off\n");
    }

    return 0;

EXIT_ERR:
    APS_ERR("apds9930_enable_als fail\n");
    return -1;
}
static int apds9930_init_client(struct i2c_client *client);

#ifdef VENDOR_EDIT			// zhanqiang add for G-sensor motion sensing

int whether_in_call_state()
{
    int call_state = 0;

    call_state = test_bit(CMC_BIT_PS, &apds9930_obj->enable) ;

    printk("%s, call_state = %d\n", __func__, call_state);

    return call_state;
}

int whether_in_approach_state()
{
    int ret = 0;
    int ps_value = 0;
	int final_ps = 0;

	printk("%s, power on the apds9930!\n", __func__);

	// first power on 
	apds9930_power(apds9930_obj->hw, 1);

	msleep(5);

	printk("%s, write register to enable apds9930!\n", __func__);
	
    // and then Enable proximity and power on apds9930
    ret = apds9930_write_reg(APDS9930_CMM_ENABLE, 0x05);
	if(ret < 0)
	{
		printk("%s, apds9930 I2C write error!\n", __func__);
		return 0;
	}

    // thirdly read proximity data
    for(ret = 0; ret < 10; ret ++)
    {
        ps_value = APDS9930_READ_WORD(apds9930_i2c_client, APDS9930_CMM_PDATA_L); 
        if(ps_value < 0)
        {
            printk("%s, apds9930 I2C Read error!\n", __func__);
			return 0;
        }

		final_ps += ps_value;
			
        msleep(5);
    }

	final_ps /= 10;

	printk("%s, final_ps = %d\n", __func__, ps_value);
	
    if(final_ps > 850)
    {
        printk("%s, there is something above the phone , so do not light up the screen!\n", __func__);
        ret = 1;
    }
    else
    {
        printk("%s, nothing above the phone , light up the screen!\n", __func__);
        ret = 0;
    }

    // fourthly disable apds9930
    apds9930_write_reg(APDS9930_CMM_ENABLE, 0x00);

    // last power down 
	apds9930_power(apds9930_obj->hw, 0);

    return ret;
}

#endif 

/*----------------------------------------------------------------------------*/
static long apds9930_enable_ps(struct i2c_client *client, int enable)
{
    struct apds9930_priv *obj = i2c_get_clientdata(client);
    uint32_t testbit_ALS;
    hwm_sensor_data sensor_data;
    int err;

    g_is_far = 1;	
    g_ps_low_threshold = PS_LOW;
    g_ps_high_threshold = PS_HIGH;
    if (atomic_read(&obj->ps_suspend)) {
        APS_LOG("apds9930_enable_ps before late_resume\n");

        apds9930_power(obj->hw, 1);

        if(err = apds9930_init_client(client)) {
            APS_LOG("apds9930_init_client %d \n", err);
        }

        if(test_bit(CMC_BIT_ALS, &obj->enable))
        {
            if((err = apds9930_enable_als(obj->client, 1)))
            {
                APS_ERR("enable als fail: %d\n", err);        
            }
        }

        atomic_set(&obj->als_suspend, 0);
        atomic_set(&obj->ps_suspend, 0);
    }

    if(client == NULL)
    {
        APS_LOG("CLIENT CANN'T EQUL NULL\n");
        return -1;
    }

#ifdef VENDOR_EDIT//LiuPing@Prd.BasicDrv.Sensor, add 2012/12/05 add for ps switch 
    if (g_ps_switch_off == 1)
    {
        printk(" apds9930_enable_ps ,  ps switch off \n");
        goto EXIT_ERR;
    }
#endif

    testbit_ALS = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
    APS_LOG(" %s(),  enable = %d, testbit_ALS = %d\n",__FUNCTION__,enable,testbit_ALS);

    if(enable)
    {
        if (APDS9930_SUCCESS != apds9930_write_reg(APDS9930_CMM_ENABLE, 0x0D)){
            goto EXIT_ERR;
        }

        atomic_set(&obj->ps_deb_on, 1);
        atomic_set(&obj->ps_deb_end, jiffies+atomic_read(&obj->ps_debounce)/(1000/HZ));
        APS_LOG("apds9930 power on\n");

        /*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
        if(0 == obj->hw->polling_mode_ps) //0:interrupt mode
        {

            if (APDS9930_SUCCESS != (
                        apds9930_write_reg(APDS9930_CMM_INT_LOW_THD_LOW, 0x0) | 
                        apds9930_write_reg(APDS9930_CMM_INT_LOW_THD_HIGH, 0x00) |
                        apds9930_write_reg(APDS9930_CMM_INT_HIGH_THD_LOW, (char)(0xFF & 1023)) |
                        apds9930_write_reg(APDS9930_CMM_INT_HIGH_THD_HIGH, (char)((0xFF00 & 1023) >> 8)) |
                        apds9930_write_reg(APDS9930_CMM_Persistence, 0x20) |
                        apds9930_write_reg(APDS9930_CMM_ENABLE, 0x2F)
                        ))
            {
                goto EXIT_ERR;
            }

            mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
        }
    }
    else
    {
        if(testbit_ALS){
            if (apds9930_write_reg(APDS9930_CMM_ENABLE, 0x2B) != APDS9930_SUCCESS)
            {
                goto EXIT_ERR;
            }
        }else{
            if (apds9930_write_reg(APDS9930_CMM_ENABLE, 0x00) != APDS9930_SUCCESS)
            {
                goto EXIT_ERR;
            }
        }

        atomic_set(&obj->ps_deb_on, 0);
        APS_LOG("apds9930 power off\n");

        /*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
        if(0 == obj->hw->polling_mode_ps)
        {
            cancel_work_sync(&obj->eint_work);
            mt65xx_eint_mask(CUST_EINT_ALS_NUM);
        }
    }


    sensor_data.values[0] = g_is_far;
    sensor_data.value_divide = 1;
    sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;			

    //let up layer to know
    if(hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data))
    {
        APS_ERR("call hwmsen_get_interrupt_data fail\n");
    }


    return 0;

EXIT_ERR:
    APS_ERR("apds9930_enable_ps fail\n");
    return -1;
}

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
static int apds9930_check_and_clear_intr(struct i2c_client *client) 
{
    int res,intp,intl;
    u8 buffer[2];
    char val;

    if(apds9930_read_reg(APDS9930_CMM_STATUS, &val) != APDS9930_SUCCESS)
    {
        goto EXIT_ERR;
    }

    res = 1;
    intp = 0;
    intl = 0;
    if(0 != (val & 0x20))
    {
        res = 0;
        intp = 1;
    }
    if(0 != (val & 0x10))
    {
        res = 0;
        intl = 1;		
    }

    if(0 == res)
    {
        if((1 == intp) && (0 == intl))
        {
            buffer[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x05);
        }
        else if((0 == intp) && (1 == intl))
        {
            buffer[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x06);
        }
        else
        {
            buffer[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x07);
        }
        res = i2c_master_send(client, buffer, 0x1);
        if(res <= 0)
        {
            goto EXIT_ERR;
        }
        else
        {
            res = 0;
        }
    }

    return res;

EXIT_ERR:
    APS_ERR("apds9930_check_and_clear_intr fail\n");
    return 1;
}
/*----------------------------------------------------------------------------*/

/*yucong add for interrupt mode support MTK inc 2012.3.7*/
static int apds9930_check_intr(struct i2c_client *client) 
{
    //	struct apds9930_priv *obj = i2c_get_clientdata(client);
    int res,intp,intl;
    char val;

    if (apds9930_read_reg(APDS9930_CMM_STATUS, &val) != APDS9930_SUCCESS)
    {
        goto EXIT_ERR;
    }

    res = 1;
    intp = 0;
    intl = 0;
    if(0 != (val & 0x20))
    {
        res = 0;
        intp = 1;
    }
    if(0 != (val & 0x10))
    {
        res = 0;
        intl = 1;		
    }

    return res;

EXIT_ERR:
    APS_ERR("apds9930_check_intr fail\n");
    return 1;
}

static int apds9930_clear_intr(struct i2c_client *client) 
{
    int res;
    u8 buffer[2];

    buffer[0] = (TAOS_TRITON_CMD_REG|TAOS_TRITON_CMD_SPL_FN|0x07);
    res = i2c_master_send(client, buffer, 0x1);
    if(res <= 0)
    {
        goto EXIT_ERR;
    }
    else
    {
        res = 0;
    }

    return res;

EXIT_ERR:
    APS_ERR("apds9930_check_and_clear_intr fail\n");
    return 1;
}


/*-----------------------------------------------------------------------------*/
void apds9930_eint_func(void)
{

    struct apds9930_priv *obj = g_apds9930_ptr;
    if(!obj)
    {
        return;
    }

    schedule_work(&obj->eint_work);
}

/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
int apds9930_setup_eint(struct i2c_client *client)
{
    struct apds9930_priv *obj = i2c_get_clientdata(client);        
    APS_LOG("%s()\n", __FUNCTION__);
    g_apds9930_ptr = obj;
    printk("apds9930 gpio=%d,eint=%d\n",GPIO_ALS_EINT_PIN,CUST_EINT_ALS_NUM);
    mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
    mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
    mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, TRUE);
    mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);

    mt65xx_eint_set_sens(CUST_EINT_ALS_NUM, CUST_EINT_ALS_SENSITIVE);
    mt65xx_eint_set_polarity(CUST_EINT_ALS_NUM, CUST_EINT_ALS_POLARITY);
    mt65xx_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
    mt65xx_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_EN, CUST_EINT_ALS_POLARITY, apds9930_eint_func, 0);

    //mt65xx_eint_unmask(CUST_EINT_ALS_NUM);  
    mt65xx_eint_mask(CUST_EINT_ALS_NUM);  
    return 0;
}

/*----------------------------------------------------------------------------*/

static int apds9930_init_client(struct i2c_client *client)
{
    struct apds9930_priv *obj = i2c_get_clientdata(client);
    int res = 0;

#ifdef VENDOR_EDIT//mingqiang.guo@Prd.BasicDrv.Sensor, add 2012/10/24 ¨ª¡§1y?¨¢¨¨?1a?DID¡ê?????2?¨ª?¦Ì?2?¨ºy 
    char device_id;
    if(APDS9930_SUCCESS != apds9930_read_reg(APDS9930_DEVICE_ID, &device_id) )
    {
        goto EXIT_ERR;
    }
    printk("------ als/ps DEVICE ID is 0x%x\n", device_id);
    g_DEVICE_ID = device_id;

    //if(g_DEVICE_ID == 0x39 ) // apds9930a3, 2?¨ª?¦Ì?D???¨¦¨¨??2?¨ª?¦Ì?¡¤¡ì?¦Ì
    {
        //alsps_dev = ALSPS_APDS9930A;
        g_ALS_GAIN = ALS_GAIN_9930A;	
        g_PPCOUNT_VALUE = APDS9930A_PPCOUNT;
        g_CONTROL_VALUE = APDS9930A_CONTROL;	
    }	
#if 0
    else 
    {
        //alsps_dev = ALSPS_APDS9930;
        g_ALS_GAIN = ALS_GAIN_9930;		
        g_PPCOUNT_VALUE = APDS9930_PPCOUNT;
        g_CONTROL_VALUE = APDS9930_CONTROL;	
    }
#endif

#endif /*VENDOR_EDIT*/
    if (APDS9930_SUCCESS != (
                apds9930_write_reg(APDS9930_CMM_ENABLE, 0x00) | 
                apds9930_write_reg(APDS9930_CMM_ATIME, 0xEE) |
                apds9930_write_reg(APDS9930_CMM_PTIME, 0xFF) |
                apds9930_write_reg(APDS9930_CMM_WTIME, 0xFA)
                ))
    {
        goto EXIT_ERR;
    }

#ifdef VENDOR_EDIT /* mingqiang.guo@BasicDrv.Sensor  ,  2012-9-28, add  for apds9930a */
    //if(g_DEVICE_ID==0x39)
    { 	
        printk("------ als/ps DEVICE ID is 0x%x\n", g_DEVICE_ID);
        if( APDS9930_SUCCESS != apds9930_write_reg(APDS9930_CMM_PS_OFFSET, APDS9930A_PSOFFSET) )   //0x7f
        {
            goto EXIT_ERR;
        }
    }
#endif /*VENDOR_EDIT*/	

    if(0 == obj->hw->polling_mode_ps)
    {
        APS_LOG(" init_client()->ps_cali.valid = %d, ps_cali.far_away = %d, ps_cali.close = %d\n"\
                ,ps_cali.valid,ps_cali.far_away,ps_cali.close);

        if (APDS9930_SUCCESS != (
                    apds9930_write_reg(APDS9930_CMM_INT_LOW_THD_LOW, 0x0) | 
                    apds9930_write_reg(APDS9930_CMM_INT_LOW_THD_HIGH, 0x00) |
                    apds9930_write_reg(APDS9930_CMM_INT_HIGH_THD_LOW, (char)(0xFF & 1023)) |
                    apds9930_write_reg(APDS9930_CMM_INT_HIGH_THD_HIGH, (char)((0xFF00 & 1023) >> 8)) |
                    apds9930_write_reg(APDS9930_CMM_Persistence, 0x20) |
                    apds9930_write_reg(APDS9930_CMM_ENABLE, 0x20)
                    ))
        {
            goto EXIT_ERR;
        }
    }

    if (APDS9930_SUCCESS != (
                apds9930_write_reg(APDS9930_CMM_CONFIG, 0x00) | 
                apds9930_write_reg(APDS9930_CMM_PPCOUNT, g_PPCOUNT_VALUE) |
                apds9930_write_reg(APDS9930_CMM_CONTROL, g_CONTROL_VALUE)
                ))
    {
        goto EXIT_ERR;
    }

    //LiuPing@Prd.BasicDrv.Sensor, add 2013/03/06 , solve that cali data don't reload after alsps repower. 
#ifdef ALSPS_CALI_DATA_RELOAD
    printk(" %s  g_offset_val_bak %d  g_ppcount_val_bak %d  \n", __FUNCTION__, g_offset_val_bak, g_ppcount_val_bak);
    if (g_ppcount_val_bak > 0)
    {
        if (APDS9930_SUCCESS !=  apds9930_write_reg(APDS9930_CMM_PPCOUNT, g_ppcount_val_bak))
        {
            printk("%s  recovery cali regs fail \n", __FUNCTION__);
            goto EXIT_ERR;
        }
    }
    if (g_offset_val_bak > 0)
    {
        if (APDS9930_SUCCESS != apds9930_write_reg(APDS9930_CMM_PS_OFFSET, g_offset_val_bak))
        {
            printk("%s  recovery cali regs fail \n", __FUNCTION__);
            goto EXIT_ERR;
        }
    }	
#endif	

    if((res = apds9930_setup_eint(client)) != 0)
    {
        APS_ERR("setup eint: %d\n", res);
        return res;
    }

    if((res = apds9930_check_and_clear_intr(client)))
    {
        APS_ERR("check/clear intr: %d\n", res);
        //    return res;
    }

    return APDS9930_SUCCESS;

EXIT_ERR:
    APS_ERR("init dev: %d\n", res);
    return -1;
}

/****************************************************************************** 
 * Function Configuration
 ******************************************************************************/
int apds9930_read_als(struct i2c_client *client, u16 *data)
{
    struct apds9930_priv *obj = i2c_get_clientdata(client);	 
    u16 c0_value, c1_value;	 
    u32 c0_nf, c1_nf;
    u8 als_value_low[1], als_value_high[1];
    u8 buffer[1];
    u16 atio;
    int res = 0;
#ifdef VENDOR_EDIT /* Lipeng.Bai@Prd.BasicDrv.Sensor  ,  2012-7-5,  Add  for learn from 12009*/
    int irf = 0, tmp;
#endif/*VENDOR_EDIT*/

    if(client == NULL)
    {
        APS_LOG("CLIENT CANN'T EQUL NULL\n");
        return -1;
    }

#ifdef VENDOR_EDIT /* Lipeng.Bai@Prd.BasicDrv.Sensor  ,  2012-8-6,  Add for  OPPO  PS*/
    c0_value = APDS9930_READ_WORD(client, APDS9930_CMM_C0DATA_L);	
    c0_nf = obj->als_modulus*c0_value;
    c1_value = APDS9930_READ_WORD(client, APDS9930_CMM_C1DATA_L);	
    c1_nf = obj->als_modulus*c1_value;	

    if (16000 < c0_value)
    {
        *data = 8500;
        APS_LOG("c0_value = %d, c1_value = %d, als = %d\n", c0_value, c1_value, *data);
        return 0;
    }
    if (0 == c0_value)
        atio = 100;
    else
        atio = (c1_value * 100) / c0_value;

    if ((atio > 0) && (atio <= 30))
        irf = (100000 - 1846 * atio) / 100;
    else if ((atio > 30) && (atio <= 38))
        irf = (126800 - 2740 * atio) / 100;
    else if ((atio > 38) && (atio <= 45))
        irf = (74900 - 1374 * atio) / 100;
    else if ((atio > 45) && (atio <= 54))  
        irf = (47700 - 769 * atio) / 100;
    else if (atio > 54)
        irf = 0;

    tmp = AtimeToMS( A_TIME ) * AGAIN *  TSL_GA;
#ifdef VENDOR_EDIT /* mingqiang.guo@BasicDrv.Sensor  ,  2012-10-17,  modify for apds9930a*/
   // if(g_DEVICE_ID == 0X39) {
        *data = 1 * 52 * irf * c0_value /tmp * ALS_GAIN_9930A;
    //} 
    //else {
    //    *data = 1 * 52 * irf * c0_value /tmp * ALS_GAIN_9930;
    //}   
    //*data = 1 * 52 * irf * c0_value /tmp * g_ALS_GAIN;
#endif

#endif/*VENDOR_EDIT*/

#ifndef VENDOR_EDIT//Shaoyu.Huang@BasicDrv.Sensor, add modify 2012/6/4 for sensro debuging
    APS_PRINTK("%s   als = %d\n", __func__,*data);
#else/*VENDOR_EDIT*/
    /*c0->CDATA (visible+infrared), c1->IRDATA (infrared)*/
    als_c0_data =c0_value ;
    als_c1_data = c1_value;
    als_data0 = *data;
    if (debugflag){
        printk(KERN_ALERT"[ALS]:c0(visible+infrared)=%d, c1(infrared)=%d, als= %d", 
                c0_value, c1_value, *data);
    }
#endif/*VENDOR_EDIT*/
    return 0;	 

EXIT_ERR:
    APS_ERR("apds9930_read_ps fail\n");
    return res;
}
int apds9930_read_als_ch0(struct i2c_client *client, u16 *data)
{
    //	struct apds9930_priv *obj = i2c_get_clientdata(client);	 
    u16 c0_value;	 
    u8 als_value_low[1], als_value_high[1];
    u8 buffer[1];
    int res = 0;

    if(client == NULL)
    {
        APS_LOG("CLIENT CANN'T EQUL NULL\n");
        return -1;
    }

    //get adc channel 0 value
    if(APDS9930_SUCCESS != (
                apds9930_read_reg(APDS9930_CMM_C0DATA_L, als_value_low) |
                apds9930_read_reg(APDS9930_CMM_C0DATA_H, als_value_high)
                ))
    {
        goto EXIT_ERR;
    }

    c0_value = als_value_low[0] | (als_value_high[0]<<8);
    *data = c0_value;
    return 0;	 

EXIT_ERR:
    APS_ERR("apds9930_read_ps fail\n");
    return res;
}

/*----------------------------------------------------------------------------*/
static int apds9930_get_als_value(struct apds9930_priv *obj, u16 als)
{
    int idx;
    int invalid = 0;
    //APS_LOG(" %s()\n", __FUNCTION__);

    for(idx = 0; idx < obj->als_level_num; idx++)
    {
        if(als < obj->hw->als_level[idx])
        {
            break;
        }
    }

    if(idx >= obj->als_value_num)
    {
        APS_ERR("exceed range\n"); 
        idx = obj->als_value_num - 1;
    }

    if(1 == atomic_read(&obj->als_deb_on))
    {
        unsigned long endt = atomic_read(&obj->als_deb_end);
        if(time_after(jiffies, endt))
        {
            atomic_set(&obj->als_deb_on, 0);
        }

        if(1 == atomic_read(&obj->als_deb_on))
        {
            invalid = 1;
        }
    }

    if(!invalid)
    {
        //APS_LOG("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);	
        return obj->hw->als_value[idx];
    }
    else
    {
        //APS_ERR("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);    
        return -1;
    }
}

/*----------------------------------------------------------------------------*/
int ps_data; //mingqiang.guo add for 12035
long apds9930_read_ps(struct i2c_client *client, u16 *data)
{
    u16 data_read; 
    static int ps_is_zero_num = 0; 

    if(client == NULL)
    {
        APS_LOG("CLIENT CANN'T EQUL NULL\n");
        return -1;
    }

    data_read = APDS9930_READ_WORD(client, APDS9930_CMM_PDATA_L);		

    *data = data_read;
    ps_data = data_read;

    //guomq resolve ps is zero all the time
    //data_read = 0; //test
    //APS_PRINTK("ps data =%d , ps_is_zero_num =%d \n",data_read,ps_is_zero_num);
    if(data_read==0)
        ps_is_zero_num++;
    else 
        ps_is_zero_num=0;
    if(ps_is_zero_num == 50)
    {
        APS_LOG("ps is zero , appear 100 time\n");
        apds9930_enable_ps(client,0);
        msleep(10);
#ifdef VENDOR_EDIT
        //LiPeng.Bai@wxkf.wxBaseDrv.Sensor  ,  2013/02/18,  Add for  OPPO  sensor
        //If drop the phone on the ground ,  PS may be in wrong state, for example, 0x0F register
        //may be 0x0, so add the follow code to resolve it
        if(0==apds9930_init_client(client))
        {
            APS_LOG("apds9930_init_client() reset failed!\n");
        }
        else
        {
            APS_LOG("apds9930_init_client() reset OK!\n");
        }
#endif /*VENDOR_EDIT*/
        apds9930_enable_ps(client,1);
        data_read = APDS9930_READ_WORD(client, APDS9930_CMM_PDATA_L);		
        ps_is_zero_num=0;
        *data = data_read;
        ps_data = data_read;
    }
    //end 
    if( debugflag )
        APS_PRINTK("apds9930_read_ps():  ps  = %d !\n",*data);

    return 0;    
}

/*----------------------------------------------------------------------------*/
static int apds9930_get_ps_value(struct apds9930_priv *obj, u16 ps)
{
    int invalid = 0;
    unsigned long endt = atomic_read(&obj->ps_deb_end);

    if(1 == atomic_read(&obj->ps_deb_on))
    {
        if(time_after(jiffies, endt))
        {
            atomic_set(&obj->ps_deb_on, 0);
        }

        if (1 == atomic_read(&obj->ps_deb_on))
        {
            invalid = 1;
        }
    }

    if(!invalid)
    {
        return g_is_far;
    }	
    else
    {
        return -1;
    }	
}

static long apds9930_set_ps_threshold(struct i2c_client *client, int low_threshold, int high_threshold)
{
    long res = 0;
    mutex_lock(&set_ps_mutex);	
    APS_LOG(" low_threshold= %d high_threshold= %d\n", low_threshold, high_threshold);

    g_ps_low_threshold = low_threshold;
    g_ps_high_threshold = high_threshold;
    if (g_is_far) {
        low_threshold = PS_LOW;
    } else {
        high_threshold = PS_HIGH;
    }

    if (APDS9930_SUCCESS != (
                apds9930_write_reg(APDS9930_CMM_INT_LOW_THD_LOW, (char)(low_threshold & 0x00ff)) | 
                apds9930_write_reg(APDS9930_CMM_INT_LOW_THD_HIGH, (char)((low_threshold & 0xff00) >> 8)) |
                apds9930_write_reg(APDS9930_CMM_INT_HIGH_THD_LOW, (char)(high_threshold & 0x00FF)) |
                apds9930_write_reg(APDS9930_CMM_INT_HIGH_THD_HIGH, (char)((high_threshold & 0xFF00) >> 8))
                ))
    {
        goto EXIT_ERR;
    }
    mutex_unlock(&set_ps_mutex);
    return 0;

EXIT_ERR:
    mutex_unlock(&set_ps_mutex);	
    APS_ERR("apds9930_enable_ps fail\n");
    return res;
}

/*----------------------------------------------------------------------------*/
/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
static void apds9930_eint_work(struct work_struct *work)
{
    struct apds9930_priv *obj = (struct apds9930_priv *)container_of(work, struct apds9930_priv, eint_work);
    int err;
    hwm_sensor_data sensor_data;
    int res = 0;
    printk("%s enter\n",__func__);

    if (!test_bit(CMC_BIT_PS, &obj->enable)) {
        goto exit;
    }

    if((err = apds9930_check_intr(obj->client)))//read the interrupt flag bit , check if there is a interrupt happend!
    {
        APS_ERR("apds9930_eint_work check intrs: %d\n", err);
    }
    else
    {
        apds9930_read_ps(obj->client, &obj->ps);

        if (g_is_far == 1){
            g_is_far = 0;	
            APS_LOG("apds9930_eint_work() from far to near! ps = %d,low_thd=%d,up_thd=%d\n", obj->ps,g_ps_low_threshold,g_ps_high_threshold);

            wake_unlock(&g_prox_wake_lock);
            if (obj->ps > 1000){
                APS_LOG("To avoid a dirty problem");
                g_ps_low_threshold = g_ps_low_threshold + 50;
                g_ps_high_threshold = g_ps_high_threshold + 50;
            }
        } else if (g_is_far == 0) { 
            g_is_far = 1;
            APS_LOG("apds9930_eint_work() from near to far! ps = %d,low_thd=%d,up_thd=%d\n", obj->ps,g_ps_low_threshold,g_ps_high_threshold);
            wake_lock_timeout(&g_prox_wake_lock, 1000);
        }
        apds9930_set_ps_threshold(obj->client, g_ps_low_threshold, g_ps_high_threshold);

        sensor_data.values[0] = apds9930_get_ps_value(obj, obj->ps);
        sensor_data.value_divide = 1;
        sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;			

        if (sensor_data.values[0] != -1) {
            //let up layer to know
            printk("%s hwmsen_get_interrupt_data\n",__func__);
            if((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
            {
                APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);
            }
        }
    }

exit:

    apds9930_clear_intr(obj->client);//clear the interrupt flag bit
    mt65xx_eint_unmask(CUST_EINT_ALS_NUM); 
    printk("%s leave\n",__func__);
}


/****************************************************************************** 
 * Function Configuration
 ******************************************************************************/
static int apds9930_open(struct inode *inode, struct file *file)
{
    file->private_data = apds9930_i2c_client;
    APS_LOG(" %s()\n", __FUNCTION__);

    if (!file->private_data)
    {
        APS_ERR("null pointer!!\n");
        return -EINVAL;
    }

    return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int apds9930_release(struct inode *inode, struct file *file)
{
    file->private_data = NULL;
    APS_LOG(" %s()\n", __FUNCTION__);
    return 0;
}

/*----------------------------------------------------------------------------*/
static long apds9930_unlocked_ioctl(struct file *file, unsigned int cmd,
        unsigned long arg)
{
    struct i2c_client *client = (struct i2c_client*)file->private_data;
    struct apds9930_priv *obj = i2c_get_clientdata(client);  
    long err = 0;
    void __user *ptr = (void __user*) arg;
    int dat;
    uint32_t enable;
    struct alsps_hw *hw = NULL;
    struct set_ps_thd_para set_ps_thd_para;

    switch (cmd)
    {
        case ALSPS_SET_PS_MODE:	//	0x01
            if(copy_from_user(&enable, ptr, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }
            if(enable)
            {
                if((err = apds9930_enable_ps(obj->client, 1)))
                {
                    APS_ERR("enable ps fail: %ld\n", err); 
                    goto err_out;
                }
                set_bit(CMC_BIT_PS, &obj->enable);

            }
            else
            {
                if((err = apds9930_enable_ps(obj->client, 0)))
                {
                    APS_ERR("disable ps fail: %ld\n", err); 
                    goto err_out;
                }
                clear_bit(CMC_BIT_PS, &obj->enable);

            }

            if (g_apds9930_ptr->fasync != NULL)
            {
                APS_LOG("kill a SIGIO when %s \n", __func__);
                kill_fasync(&g_apds9930_ptr->fasync, SIGIO, POLL_IN);
            }

            break;

        case ALSPS_GET_PS_MODE://0x02           
            enable = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
            //APS_LOG(" %s()->case ALSPS_GET_PS_MODE enable=%d\n", __FUNCTION__,enable);
            if(copy_to_user(ptr, &enable, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

        case ALSPS_GET_PS_DATA://0x03
            //APS_LOG(" %s()->case ALSPS_GET_PS_DATA \n", __FUNCTION__);
            dat = apds9930_get_ps_value(obj, obj->ps);
            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }  
            break;

        case ALSPS_GET_PS_RAW_DATA://0x04   
            //APS_LOG(" %s()->case ALSPS_GET_PS_RAW_DATA \n", __FUNCTION__);
            if((err = apds9930_read_ps(obj->client, &obj->ps)))
            {
                goto err_out;
            }
            dat = obj->ps;
            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }  
            break;              

        case ALSPS_SET_ALS_MODE://0x05
            //APS_LOG(" %s()->case ALSPS_SET_ALS_MODE \n", __FUNCTION__);
            if(copy_from_user(&enable, ptr, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }
            if(enable)
            {
                if((err = apds9930_enable_als(obj->client, 1)))
                {
                    APS_ERR("enable als fail: %ld\n", err); 
                    goto err_out;
                }
                set_bit(CMC_BIT_ALS, &obj->enable);
            }
            else
            {
                if((err = apds9930_enable_als(obj->client, 0)))
                {
                    APS_ERR("disable als fail: %ld\n", err); 
                    goto err_out;
                }
                clear_bit(CMC_BIT_ALS, &obj->enable);
            }
            break;

        case ALSPS_GET_ALS_MODE://0x06
            //APS_LOG(" %s()->case ALSPS_GET_ALS_MODE \n", __FUNCTION__);
            enable = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
            if(copy_to_user(ptr, &enable, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

        case ALSPS_GET_ALS_DATA: //0x07
            //APS_LOG(" %s()->case ALSPS_GET_ALS_DATA \n", __FUNCTION__);
            if((err = apds9930_read_als(obj->client, &obj->als)))
            {
                goto err_out;
            }

            dat = apds9930_get_als_value(obj, obj->als);
            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }              
            break;

        case ALSPS_GET_ALS_RAW_DATA://0x08    
            //APS_LOG(" %s()->case ALSPS_GET_ALS_RAW_DATA \n", __FUNCTION__);
            if((err = apds9930_read_als(obj->client, &obj->als)))
            {
                goto err_out;
            }
            dat = obj->als;
            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
            //zhanhua.li just for test
            if(0)
            {
                hwm_sensor_data sensor_data;
                static int report_rate=0;
                sensor_data.values[0] = apds9930_get_ps_value(obj, obj->ps);
                sensor_data.value_divide = 1;
                sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;			

                if (sensor_data.values[0] != -1) {
                    //let up layer to know
                    report_rate++;

                    if((report_rate%10==0)&&(err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
                    {
                        APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);
                    }

                }

            }
            break;

        case ALSPS_SET_PS_THRESHOLD: //0x21 lycan add for 
            //APS_LOG(" %s()->case ALSPS_SET_PS_THRESHOLD\n", __FUNCTION__);
            if(copy_from_user(&set_ps_thd_para, ptr, sizeof(set_ps_thd_para)))
            {
                err = -EFAULT;
                goto err_out;
            }

            switch (set_ps_thd_para.algo_state) {
                case PS_ADJUST_TREND_STATE : 
                    APS_LOG("set_ps_threshold ps average:%d state: TREND\n", set_ps_thd_para.ps_average);
                    break;
                case PS_ADJUST_NOTREND_STATE :
                    APS_LOG("set_ps_threshold ps average:%d state: NOTREND\n", set_ps_thd_para.ps_average);
                    break;
                case PS_ADJUST_HIGHLIGHT_STATE :
                    APS_LOG("set_ps_threshold ps average:%d state: HIGHLIGHT\n", set_ps_thd_para.ps_average);
                    break;
                case PS_ADJUST_AVOID_DIRTY_STATE : 
                    APS_LOG("set_ps_threshold ps average:%d state: AVOID_DIRTY\n", set_ps_thd_para.ps_average);
                    break;
                default:
                    APS_LOG("set_ps_threshold ps average:%d state: impossible\n", set_ps_thd_para.ps_average);
                    break;
            }

            apds9930_set_ps_threshold(obj->client,
                    set_ps_thd_para.low_threshold, 
                    set_ps_thd_para.high_threshold);
            break;

        case ALSPS_GET_CUST_PS_ADJUST_PARA: //0x22 lycan add for
            APS_LOG(" %s()->case ALSPS_GET_CUST_PS_ADJUST_PARA \n", __FUNCTION__);
            hw = get_cust_alsps_hw_apds();
            if(copy_to_user(ptr, hw->p_ps_adjust_para, sizeof( struct ps_adjust_para)))
            {
                err = -EFAULT;
                goto err_out;
            }              
            break;

        default:
            APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
            err = -ENOIOCTLCMD;
            break;
    }

err_out:
    return err;    
}

static int apds9930_fasync(int fd, struct file * file, int on)
{
    APS_LOG("%s on : %d\n", __func__, on);
    return fasync_helper(fd, file, on, &g_apds9930_ptr->fasync);
}

/*----------------------------------------------------------------------------*/
static struct file_operations apds9930_fops = {
    .owner = THIS_MODULE,
    .open = apds9930_open,
    .release = apds9930_release,
    .unlocked_ioctl = apds9930_unlocked_ioctl,
    .fasync = apds9930_fasync,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice apds9930_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "als_ps",
    .fops = &apds9930_fops,
};
/*----------------------------------------------------------------------------*/
static int apds9930_i2c_suspend(struct i2c_client *client, pm_message_t msg) 
{
    APS_FUN();    
    APS_LOG(" %s()\n", __FUNCTION__);
    return 0;
}
/*----------------------------------------------------------------------------*/
static int apds9930_i2c_resume(struct i2c_client *client)
{
    APS_FUN();
    APS_LOG(" %s()\n", __FUNCTION__);
    return 0;
}
/*----------------------------------------------------------------------------*/
static void apds9930_early_suspend(struct early_suspend *h) 
{
    struct apds9930_priv *obj = container_of(h, struct apds9930_priv, early_drv);   
    int err;
    APS_FUN();    
    APS_LOG(" %s()\n", __FUNCTION__);
    if(!obj)
    {
        APS_ERR("null pointer!!\n");
        return;
    }

    if (test_bit(CMC_BIT_PS, &obj->enable)) 
    {
        APS_LOG("in call state , apds9930_early_suspend()->not power off  apds9930 !\n");
    }
    else//not in call state
    {
        atomic_set(&obj->als_suspend, 1);
        atomic_set(&obj->ps_suspend, 1);
        if(test_bit(CMC_BIT_ALS, &obj->enable))
        {
            if((err = apds9930_enable_als(obj->client, 0)))
            {
                APS_ERR("disable als fail: %d\n", err); 
            }
        }
        mt65xx_eint_mask(CUST_EINT_ALS_NUM); 	//unreg eint ,otherwise lots of eint result in reboot kernel.
        apds9930_power(obj->hw, 0);
        APS_LOG(" not in call state , apds9930_early_suspend()->power off  apds9930 !\n");
    }
}

/*----------------------------------------------------------------------------*/
static void apds9930_late_resume(struct early_suspend *h)
{   /*early_suspend is only applied for ALS*/
    struct apds9930_priv *obj = container_of(h, struct apds9930_priv, early_drv);         
    int err;
    APS_LOG(" %s()\n", __FUNCTION__);
    APS_FUN();

    if(!obj)
    {
        APS_ERR("null pointer!!\n");
        return;
    }

    if (test_bit(CMC_BIT_PS, &obj->enable)) 
    {
        APS_LOG(" in call state!\n");
    }
    else//not in call state
    {
        APS_LOG("not in call state , apds9930_late_resume()->power on apds9930 !\n");
        apds9930_power(obj->hw, 1);

        if(err = apds9930_init_client(obj->client)) {
            APS_LOG("apds9930_init_client %d \n", err);
        }

        atomic_set(&obj->als_suspend, 0);
        atomic_set(&obj->ps_suspend, 0);

        if(test_bit(CMC_BIT_ALS, &obj->enable))
        {
            if((err = apds9930_enable_als(obj->client, 1)))
            {
                APS_ERR("enable als fail: %d\n", err);        
            }
        }


    }

}

int apds9930_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
        void* buff_out, int size_out, int* actualout)
{
    int err = 0;
    int value;
    hwm_sensor_data* sensor_data;
    struct apds9930_priv *obj = (struct apds9930_priv *)self;
    //APS_LOG(" %s()->case %d\n", __FUNCTION__,command);
    //APS_FUN(f);
    switch (command)
    {
        case SENSOR_DELAY://0x01
            if((buff_in == NULL) || (size_in < sizeof(int)))
            {
                APS_ERR("Set delay parameter error!\n");
                err = -EINVAL;
            }
            // Do nothing
            break;

        case SENSOR_ENABLE://0x02
            if((buff_in == NULL) || (size_in < sizeof(int)))
            {
                APS_ERR("Enable sensor parameter error!\n");
                err = -EINVAL;
            }
            else
            {				
                value = *(int *)buff_in;

                if((err = apds9930_enable_ps(obj->client, value)))
                {
                    APS_ERR("enable ps fail: %d\n", err); 
                    return -1;
                }

                if(value)
                {
                    set_bit(CMC_BIT_PS, &obj->enable);
                }
                else
                {
                    clear_bit(CMC_BIT_PS, &obj->enable);
                }

                if (g_apds9930_ptr->fasync != NULL)
                {
                    APS_LOG("kill a SIGIO when %s \n", __func__);
                    kill_fasync(&g_apds9930_ptr->fasync, SIGIO, POLL_IN);
                }
            }
            break;

        case SENSOR_GET_DATA://0x04
            if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
            {
                APS_ERR("get sensor data parameter error!\n");
                err = -EINVAL;
            }
            else
            {
                sensor_data = (hwm_sensor_data *)buff_out;	
                apds9930_read_ps(obj->client, &obj->ps);

                //mdelay(160);
                apds9930_read_als_ch0(obj->client, &obj->als);
                APS_ERR("apds9930_ps_operate als data=%d!\n",obj->als);
                sensor_data->values[0] = apds9930_get_ps_value(obj, obj->ps);
                sensor_data->value_divide = 1;
                sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;			
            }
            break;
        default:
            APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
            err = -1;
            break;
    }

    return err;
}

int apds9930_als_operate(void* self, uint32_t command, void* buff_in, int size_in,
        void* buff_out, int size_out, int* actualout)
{
    int err = 0;
    int value;
    hwm_sensor_data* sensor_data;
    struct apds9930_priv *obj = (struct apds9930_priv *)self;
    //APS_LOG(" %s()->case %d\n", __FUNCTION__,command);
    switch (command)
    {
        case SENSOR_DELAY:	//	0x01
            if((buff_in == NULL) || (size_in < sizeof(int)))
            {
                APS_ERR("Set delay parameter error!\n");
                err = -EINVAL;
            }
            // Do nothing
            break;

        case SENSOR_ENABLE://0x02
            if((buff_in == NULL) || (size_in < sizeof(int)))
            {
                APS_ERR("Enable sensor parameter error!\n");
                err = -EINVAL;
            }
            else
            {
                value = *(int *)buff_in;				
                if(value)
                {
                    if((err = apds9930_enable_als(obj->client, 1)))
                    {
                        APS_ERR("enable als fail: %d\n", err); 
                        return -1;
                    }
                    set_bit(CMC_BIT_ALS, &obj->enable);
                }
                else
                {
                    if((err = apds9930_enable_als(obj->client, 0)))
                    {
                        APS_ERR("disable als fail: %d\n", err); 
                        return -1;
                    }
                    clear_bit(CMC_BIT_ALS, &obj->enable);
                }

            }
            break;

        case SENSOR_GET_DATA://0x04
            if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
            {
                APS_ERR("get sensor data parameter error!\n");
                err = -EINVAL;
            }
            else
            {
                sensor_data = (hwm_sensor_data *)buff_out;

                /*yucong MTK add for fixing know issue*/
#if 1
                apds9930_read_als(obj->client, &obj->als);
                if(obj->als == 0)
                {
                    //sensor_data->values[0] = -1;				
                    sensor_data->values[0] = 0;//mingqing.guo modify for when als always zero , key light can not open 				
                }else{
                    u16 b[2];
                    int i;
                    for(i = 0;i < 2;i++){
                        apds9930_read_als(obj->client, &obj->als);
                        b[i] = obj->als;
                    }
                    (b[1] > b[0])?(obj->als = b[0]):(obj->als = b[1]);
                    sensor_data->values[0] = apds9930_get_als_value(obj, obj->als);
                }
#endif

                sensor_data->value_divide = 1;
                sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
            }
            break;
        default:
            APS_ERR("light sensor operate function no this parameter %d!\n", command);
            err = -1;
            break;
    }

    return err;
}


/*----------------------------------------------------------------------------*/
static int apds9930_i2c_detect(struct i2c_client *client, struct i2c_board_info *info) 
{    
    strcpy(info->type, APDS9930_DEV_NAME);
    return 0;
}

/*----------------------------------------------------------------------------*/
static int apds9930_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct apds9930_priv *obj;
    struct hwmsen_object obj_ps, obj_als;
    int err = 0;
    APS_LOG(" %s()\n", __FUNCTION__);
    if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
    {
        err = -ENOMEM;
        goto exit;
    }
    memset(obj, 0, sizeof(*obj));
    apds9930_obj = obj;

    obj->hw = get_cust_alsps_hw_apds();
    apds9930_get_addr(obj->hw, &obj->addr);

    /*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
    INIT_WORK(&obj->eint_work, apds9930_eint_work);
    obj->client = client;
    i2c_set_clientdata(client, obj);	
    atomic_set(&obj->als_debounce, 50);
    atomic_set(&obj->als_deb_on, 0);
    atomic_set(&obj->als_deb_end, 0);
    atomic_set(&obj->ps_debounce, 10);
    atomic_set(&obj->ps_deb_on, 0);
    atomic_set(&obj->ps_deb_end, 0);
    atomic_set(&obj->ps_mask, 0);
    atomic_set(&obj->als_suspend, 0);
    atomic_set(&obj->ps_suspend, 0);
    atomic_set(&obj->als_cmd_val, 0xDF);
    atomic_set(&obj->ps_cmd_val,  0xC1);
    atomic_set(&obj->ps_thd_val_high,  obj->hw->ps_threshold_high);
    atomic_set(&obj->ps_thd_val_low,  obj->hw->ps_threshold_low);
    obj->enable = 0;
    obj->pending_intr = 0;
    obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
    obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);  
    /*Lenovo-sw chenlj2 add 2011-06-03,modified gain 16 to 1/5 accoring to actual thing */
#ifndef VENDOR_EDIT /* Lipeng.Bai@Prd.BasicDrv.Sensor  ,  2012-7-5,  Modify  for learn from 12009*/
    obj->als_modulus = (400*100*ZOOM_TIME)/(1*150);//(1/Gain)*(400/Tine), this value is fix after init ATIME and CONTROL register value
    //(400)/16*2.72 here is amplify *100 //16
#else
    obj->als_modulus = (400*100)/(16*150);//(1/Gain)*(400/Tine), this value is fix after init ATIME and CONTROL register value
    //(400)/16*2.72 here is amplify *100
#endif
    BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
    memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
    BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
    memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
    atomic_set(&obj->i2c_retry, 3);
    //set_bit(CMC_BIT_ALS, &obj->enable);
    //set_bit(CMC_BIT_PS, &obj->enable);


    apds9930_i2c_client = client;


    if((err = apds9930_init_client(client)))
    {
        goto exit_init_failed;
    }
    APS_LOG("apds9930_init_client() OK!\n");

    if((err = misc_register(&apds9930_device)))
    {
        APS_ERR("apds9930_device register failed\n");
        goto exit_misc_device_register_failed;
    }
    /*
       if(err = apds9930_create_attr(&apds9930_alsps_driver.driver))
       {
       APS_ERR("create attribute err = %d\n", err);
       goto exit_create_attr_failed;
       }
     */
    obj_ps.self = apds9930_obj;
    /*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
    if(1 == obj->hw->polling_mode_ps)
    {
        obj_ps.polling = 1;
    }
    else
    {
        obj_ps.polling = 0;
    }

    obj_ps.sensor_operate = apds9930_ps_operate;
    if((err = hwmsen_attach(ID_PROXIMITY, &obj_ps)))
    {
        APS_ERR("attach fail = %d\n", err);
        goto exit_create_attr_failed;
    }

    obj_als.self = apds9930_obj;
    obj_als.polling = 1;
    obj_als.sensor_operate = apds9930_als_operate;
    if((err = hwmsen_attach(ID_LIGHT, &obj_als)))
    {
        APS_ERR("attach fail = %d\n", err);
        goto exit_create_attr_failed;
    }


#if defined(CONFIG_HAS_EARLYSUSPEND)
    obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
        obj->early_drv.suspend  = apds9930_early_suspend,
        obj->early_drv.resume   = apds9930_late_resume,    
        register_early_suspend(&obj->early_drv);
#endif
#ifdef VENDOR_EDIT//Shaoyu.Huang@BasicDrv.Sensor, add 2012/6/4 for sensor debuging
    hwmsen_make_debug_flag(&debug_fops, "open_alsps_log");
    hwmsen_make_debug_flag(&set_reg_if, "set_apds_reg");
    hwmsen_make_debug_flag(&read_reg_if, "read_apds_reg");
    hwmsen_make_debug_flag(&ps_thd_if, "set_apds_thd");
#ifdef ALSPS_CALI_DATA_RELOAD    
    //hwmsen_make_debug_flag(&cali_pulse_count_fops,"cali_pulse_count");//oppo mingqiang.guo add for cali_pulse_count
#endif    
    hwmsen_make_debug_flag(&ps_switch_fops,"ps_switch");//LiuPing@Prd.BasicDrv.Sensor, add 2012/12/05 add for ps switch 
    hwmsen_make_debug_flag(&als_read_fops,"read_als_data");
    wake_lock_init(&g_prox_wake_lock, WAKE_LOCK_SUSPEND, "Prox wakelock");
    hwmsen_make_debug_flag(&ic_model_fops, "read_alsps_model");//mingqiang.guo@prd.BasicDrv.Sensor, 2012/11/12 add for read_ic_model 
#endif/*VENDOR_EDIT*/
    alsps_dev = ALSPS_TAOS_277X;
    APS_LOG("%s: OK\n", __func__);
    return 0;

exit_create_attr_failed:
    misc_deregister(&apds9930_device);
exit_misc_device_register_failed:
exit_init_failed:
    //i2c_detach_client(client);
    //exit_kfree:
#ifdef VENDOR_EDIT//Shaoyu.Huang@Prd.BasicDrv.Sensor, add 2012/6/14 for sensor power off
    apds9930_power(obj->hw, 0);
#endif/*VENDOR_EDIT*/
    kfree(obj);
exit:
    apds9930_i2c_client = NULL;           
    //	MT6516_EINTIRQMask(CUST_EINT_ALS_NUM);  /*mask interrupt if fail*/
    APS_ERR("%s: err = %d\n", __func__, err);
    return err;
}
/*----------------------------------------------------------------------------*/
static int apds9930_i2c_remove(struct i2c_client *client)
{
    int err;	
    /*	
        if(err = apds9930_delete_attr(&apds9930_i2c_driver.driver))
        {
        APS_ERR("apds9930_delete_attr fail: %d\n", err);
        } 
     */
    APS_LOG(" %s()\n", __FUNCTION__);
    if((err = misc_deregister(&apds9930_device)))
    {
        APS_ERR("misc_deregister fail: %d\n", err);    
    }

    apds9930_i2c_client = NULL;
    i2c_unregister_device(client);
    kfree(i2c_get_clientdata(client));

    return 0;
}
/*----------------------------------------------------------------------------*/
static int apds9930_probe(struct platform_device *pdev) 
{
    struct alsps_hw *hw = get_cust_alsps_hw_apds();

    APS_LOG("%s()\n", __FUNCTION__);

    apds9930_power(hw, 1);    
    //apds9930_force[0] = hw->i2c_num;
    //apds9930_force[1] = hw->i2c_addr[0];
    //APS_LOG("I2C = %d, addr =0x%x\n",apds9930_force[0],apds9930_force[1]);
    if(i2c_add_driver(&apds9930_i2c_driver))
    {
        APS_ERR("add driver error\n");
        return -1;
    } 
    return 0;
}
/*----------------------------------------------------------------------------*/
static int apds9930_remove(struct platform_device *pdev)
{
    struct alsps_hw *hw = get_cust_alsps_hw_apds();
    APS_LOG(" %s()\n", __FUNCTION__);
    APS_FUN();    
    apds9930_power(hw, 0);    
    i2c_del_driver(&apds9930_i2c_driver);
    return 0;
}
/*----------------------------------------------------------------------------*/
static struct platform_driver apds9930_alsps_driver = {
    .probe      = apds9930_probe,
    .remove     = apds9930_remove,    
    .driver     = {
        .name  = "als_ps_apds9930",
        //		.owner = THIS_MODULE,
    }
};
/*----------------------------------------------------------------------------*/
static int __init apds9930_init(void)
{
    APS_FUN();
    i2c_register_board_info(0, &i2c_APDS9930, 1);
    if(platform_driver_register(&apds9930_alsps_driver))
    {
        APS_ERR("failed to register driver");
        return -ENODEV;
    }
    return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit apds9930_exit(void)
{
    APS_FUN();
    platform_driver_unregister(&apds9930_alsps_driver);
}
/*----------------------------------------------------------------------------*/
module_init(apds9930_init);
module_exit(apds9930_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("Dexiang Liu");
MODULE_DESCRIPTION("apds9930 driver");
MODULE_LICENSE("GPL");

