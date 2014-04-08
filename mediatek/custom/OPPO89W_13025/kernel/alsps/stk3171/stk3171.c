/* drivers/hwmon/mt6516/amit/stk3171.c - stk3171 ALS/PS driver
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

#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <linux/hwmsen_helper.h>
#include <asm/io.h>
#include <cust_eint.h>
#include <cust_alsps.h>
#include "stk3171.h"

//#include <mach/mt_devs.h>/*OPPO 2013-01-21 zhye add for devices init*/
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <linux/semaphore.h>/*OPPO 2013-01-21 zhye add for sync suspend and resume*/

#include <linux/ctype.h>
#include <linux/kernel.h>
#include <linux/oppo_devices_list.h>

extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern void mt65xx_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern unsigned int mt65xx_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt65xx_eint_registration(unsigned int eint_num, unsigned int is_deb_en, unsigned int pol, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);

/*-------------------------MT6516&MT6573 define-------------------------------*/

#define POWER_NONE_MACRO MT65XX_POWER_NONE
#if 0
/* TEMPLATE */
#define GPIO_ALS_EINT_PIN         GPIO190
#define GPIO_ALS_EINT_PIN_M_GPIO  GPIO_MODE_00
#define GPIO_ALS_EINT_PIN_M_EINT  GPIO_MODE_01
#define GPIO_ALS_EINT_PIN_M_PWM  GPIO_MODE_04
#define CUST_EINT_ALS_NUM              3
#define CUST_EINT_ALS_DEBOUNCE_CN      0
#define CUST_EINT_ALS_POLARITY         CUST_EINT_POLARITY_LOW
#define CUST_EINT_ALS_SENSITIVE        CUST_EINT_LEVEL_SENSITIVE
#define CUST_EINT_ALS_DEBOUNCE_EN      CUST_EINT_DEBOUNCE_DISABLE
#endif
#define GPIO_ALS_EINT_PIN         GPIO38
#define CUST_EINT_ALS_NUM              4

/******************************************************************************
 * configuration
*******************************************************************************/

/*----------------------------------------------------------------------------*/
#define STK31XX_DEV_NAME     "stk3171"
/*----------------------------------------------------------------------------*/
#define APS_TAG                  "[ALS/PS]"
#define APS_FUN(f)               printk(KERN_INFO APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    printk(KERN_ERR  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    printk(KERN_INFO APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    printk(KERN_INFO fmt, ##args)                 
/******************************************************************************
 * extern functions
*******************************************************************************/
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
#define mt6516_I2C_DATA_PORT        ((base) + 0x0000)
#define mt6516_I2C_SLAVE_ADDR       ((base) + 0x0004)
#define mt6516_I2C_INTR_MASK        ((base) + 0x0008)
#define mt6516_I2C_INTR_STAT        ((base) + 0x000c)
#define mt6516_I2C_CONTROL          ((base) + 0x0010)
#define mt6516_I2C_TRANSFER_LEN     ((base) + 0x0014)
#define mt6516_I2C_TRANSAC_LEN      ((base) + 0x0018)
#define mt6516_I2C_DELAY_LEN        ((base) + 0x001c)
#define mt6516_I2C_TIMING           ((base) + 0x0020)
#define mt6516_I2C_START            ((base) + 0x0024)
#define mt6516_I2C_FIFO_STAT        ((base) + 0x0030)
#define mt6516_I2C_FIFO_THRESH      ((base) + 0x0034)
#define mt6516_I2C_FIFO_ADDR_CLR    ((base) + 0x0038)
#define mt6516_I2C_IO_CONFIG        ((base) + 0x0040)
#define mt6516_I2C_DEBUG            ((base) + 0x0044)
#define mt6516_I2C_HS               ((base) + 0x0048)
#define mt6516_I2C_DEBUGSTAT        ((base) + 0x0064)
#define mt6516_I2C_DEBUGCTRL        ((base) + 0x0068)
/*----------------------------------------------------------------------------*/
static struct i2c_client *stk3171_i2c_client = NULL;
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id stk3171_i2c_id[] = {{STK31XX_DEV_NAME,0},{}};
/*the adapter id & i2c address will be available in customization*/
static struct i2c_board_info __initdata i2c_stk3171={ I2C_BOARD_INFO(STK31XX_DEV_NAME, (0x90>>1))};

//static unsigned short stk3171_force[] = {0x00, 0x00, I2C_CLIENT_END, I2C_CLIENT_END};
//static const unsigned short *const stk3171_forces[] = { stk3171_force, NULL };
//static struct i2c_client_address_data stk3171_addr_data = { .forces = stk3171_forces,};
/*----------------------------------------------------------------------------*/
static int stk3171_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int stk3171_i2c_remove(struct i2c_client *client);
static int stk3171_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
/*----------------------------------------------------------------------------*/
static int stk3171_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int stk3171_i2c_resume(struct i2c_client *client);
static int stk3171_set_als_int_thd(struct i2c_client *client, u16 als_data_reg);
static int stk3171_init_client(struct i2c_client *client);

static struct stk3171_priv *g_stk3171_ptr = NULL;
static unsigned int gain_setting;

static u8 reg8[3];
static unsigned int init_count = 0;

/*----------------------------------------------------------------------------*/
typedef enum {
    STK_TRC_ALS_DATA= 0x0001,
    STK_TRC_PS_DATA = 0x0002,
    STK_TRC_EINT    = 0x0004,
    STK_TRC_IOCTL   = 0x0008,
    STK_TRC_I2C     = 0x0010,
    STK_TRC_CVT_ALS = 0x0020,
    STK_TRC_CVT_PS  = 0x0040,
    STK_TRC_DEBUG   = 0x8000,
} STK_TRC;
/*----------------------------------------------------------------------------*/
typedef enum {
    STK_BIT_ALS    = 1,
    STK_BIT_PS     = 2,
} STK_BIT;
/*----------------------------------------------------------------------------*/
struct stk3171_i2c_addr {    /*define a series of i2c slave address*/
    u8  status;     /*Alert Response Address*/
    u8  init;       /*device initialization */
    u8  als_cmd;    /*ALS command*/
    u8  als_dat1;   /*ALS MSB*/
    u8  als_dat2;   /*ALS LSB*/
    u8  ps_cmd;     /*PS command*/
    u8  ps_dat;     /*PS data*/
	u8	ps_gain;	/*PS gain*/
    u8  ps_high_thd;     /*PS INT threshold*/
	u8  ps_low_thd;     /*PS INT threshold*/
	u8  als_high_thd1;	/*ALS INT threshold high*/
	u8  als_high_thd2;	/*ALS INT threshold high*/
	u8  als_low_thd1;	/*ALS INT threshold low*/
	u8  als_low_thd2;	/*ALS INT threshold low*/
};
/*----------------------------------------------------------------------------*/
struct stk3171_priv {
    struct alsps_hw  *hw;
    struct i2c_client *client;
    struct work_struct  eint_work;

    /*i2c address group*/
    struct stk3171_i2c_addr  addr;
    
    /*misc*/
    atomic_t    trace;
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
    u8          ps;
    u8          _align;
    u16         als_level_num;
    u16         als_value_num;
    u32         als_level[C_CUST_ALS_LEVEL-1];
    u32         als_value[C_CUST_ALS_LEVEL];

    atomic_t    als_cmd_val;    /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_cmd_val;     /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_high_thd_val;     /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_low_thd_val;     /*the cmd value can't be read, stored in ram*/
    ulong       enable;         /*enable mask*/
    ulong       pending_intr;   /*pending interrupt*/
	atomic_t	recv_reg;
	struct fasync_struct *fasync;
    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif     
};

/********************zhye add for debug*****************************/
static struct alsps_hw *get_cust_alsps_hw(void) 
{
    return get_cust_alsps_hw_stk3171();
}

static unsigned int g_is_far = 1; /*init this flag with 1, means FAR*/

static unsigned int 	g_ps_low_threshold = 0;
static unsigned int 	g_ps_high_threshold = 255;
static unsigned int g_ps_switch_off = 0;

//static DEFINE_SEMAPHORE(ps_thd_sem);
static DEFINE_SEMAPHORE(ps_suspend_sem);
static DEFINE_SEMAPHORE(ps_set_mode);
//extern struct wait_queue_head_t ps_wait_queue_head;
#ifdef THREAD_NEED_WAIT
static DECLARE_WAIT_QUEUE_HEAD(ps_adjust_head);
#endif

#ifdef NEED_TIMER
static struct hrtimer report_timer;
static unsigned int timer_count = 0;
#endif
#ifdef VENDOR_EDIT
/*relate to stk3171 als*/
static unsigned int level_count = 0;
static unsigned int intensity_level = 0;
static unsigned int temp_level = 5500;
static unsigned int prev_level = 5500;
/********end********/

/*relate ot stk3171 ps */

static unsigned int ps_work_status = 0;
static unsigned int suspend_status = 0;
static unsigned int ps_eint_mode = 0;
static unsigned int distance = 1;
static unsigned int prev_dist = 1;
static unsigned int ps_invalid_flags = 0;

#define STK31XX_CMM_POLL_LOW_THD		PS_THDL
#define STK31XX_CMM_POLL_HIGH_THD		PS_THDH
#define STK31XX_ERR_I2C -1

/********end********/


#endif/*VENDOR_EDIT*/
/*************************************************/
static s32 atoi(char *psz_buf)
{
	char *pch = psz_buf;
	s32 base = 0;

	while (isspace(*pch))
		pch++;

	if (*pch == '-' || *pch == '+') {
		base = 10;
		pch++;
	} else if (*pch && tolower(pch[strlen(pch) - 1]) == 'h') {
		base = 16;
	}

	return simple_strtoul(pch, NULL, base);
}
/*----------------------------------------------*/
static u8 set_reg = 0;
static ssize_t set_prox_reg_read(struct file *file, char __user *buf, size_t count, loff_t *pos)
{	
	char page[8]; 	
	char *p = page;	
	int len = 0; 	
	p += sprintf(p, "0x%x\n", set_reg);	
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
static ssize_t set_prox_reg_write(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{	
	char tmp[16] = {0};	
	char hex[8] = {0};
	u8 reg, val;
	int ret;
	int i = 0;
	if (count > 16)		
		return -EINVAL;		
	ret = copy_from_user(tmp, buf, 16);
	ret = 1;
	while(tmp[i] != '\0'){
		if (tmp[i] == ':'){
			i++;
			ret = i + 1;
			reg = atoi(hex);
			continue;
		}
		if (i / ret == 0)
			hex[i] = tmp[i];
		else hex[i % ret] = tmp[i];
	}	
	val = atoi(hex);
	set_reg = reg;
	ret = i2c_smbus_write_byte_data(stk3171_i2c_client, reg, val);
	if (ret < 0)
		printk("[ALS/PS] : %s  set 0x%x to 0x%x fail \n",__func__, reg, val);
	
	return count;	
}
static struct file_operations set_prox_reg = {
	.read = set_prox_reg_read,
	.write = set_prox_reg_write,
};
/*----------------------------------------------*/
static ssize_t read_prox_reg_read(struct file *file, char __user *buf, size_t count, loff_t *pos)
{
	char page[1024]; 	
	char *p = page;	
	int len = 0; 	
	int ret;
	u8 i, val;
	for (i = 0x01; i <= 0x0c; i += 0x01){
		i2c_master_send(stk3171_i2c_client, &i, 1);
		i2c_master_recv(stk3171_i2c_client, &val, 1);
		p += sprintf(p, "Reg 0x%x:0x%x\n", i, val);	
	}
	i = 0x82;
	i2c_master_send(stk3171_i2c_client, &i, 1);
	i2c_master_recv(stk3171_i2c_client, &val, 1);
	p += sprintf(p, "Reg 0x%x:0x%x\n", i, val);	
	
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
static ssize_t read_prox_reg_write(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	char tmp[8] = {0};
	int ret;
	u8 reg,val;
	ret = copy_from_user(tmp, buf, 8);
	reg = atoi(tmp);
	val = i2c_smbus_read_byte_data(stk3171_i2c_client, reg);
	if (val < 0)
		printk("ALS/PS : %s  0x%x  fail\n",__func__,reg);
	else printk("ALS/PS : %s  reg 0x%x = 0x%x\n",__func__, reg, val);

	return count;
}
static struct file_operations read_prox_reg = {
	.read = read_prox_reg_read,
	.write = read_prox_reg_write,
};
/*---------------------------------------------------------------------------*/
static ssize_t read_ps_switch_status(struct file *file, char __user *buf, size_t count, loff_t *pos)
{
    char page[1024]; 	
    char *p = page;	
    int len = 0; 

    printk("stk3171  show_ps_switch ,  g_ps_switch_off:%d \n", g_ps_switch_off);
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
    APS_LOG("stk3171  %s : %s\n", __func__, tmp);

    sscanf(tmp, "%d", &val);
    APS_LOG("stk3171 set_ps_switch %d\n", val);

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
static struct file_operations ps_switch_fops = {
	.read = read_ps_switch_status,
	.write = set_ps_switch,
};
/*----------------------------------------------*/
#ifdef VENDOR_EDIT//Shaoyu.Huang@BadicDrv.Sensor, add 2012/6/4 for sensor debuging
/*----------------------------------------------------------------------------*/
static int alsps_dbg_flag = 1;
static ssize_t debugflag_read(struct file *file, char __user *buf, size_t count, loff_t *pos)
{	
	char page[8]; 	
	char *p = page;	
	int len = 0; 	
	p += sprintf(p, "%d\n", alsps_dbg_flag);	
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
		alsps_dbg_flag = 1;	
	else if ('0' == tmp[0])		
		alsps_dbg_flag = 0;	
	else		
		return -EINVAL;		
	return count;	
}
static struct file_operations open_alsps_log = {
	.read = debugflag_read,
	.write = debugflag_write,
};

#endif /*VENDOR_EDIT*/

/*----------------------------------------------------------------------------*/
static struct i2c_driver stk3171_i2c_driver = {	
	.probe      = stk3171_i2c_probe,
	.remove     = stk3171_i2c_remove,
//	.detect     = stk3171_i2c_detect,

/*OPPO zhye delete code because use early_suspend*/
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend    = stk3171_i2c_suspend,
	.resume     = stk3171_i2c_resume,
#endif
/*OPPO zhye delete end*/
	
	.id_table   = stk3171_i2c_id,
//	.address_data = &stk3171_addr_data,
	.driver = {
//		.owner          = THIS_MODULE,
		.name           = STK31XX_DEV_NAME,
	},
};

static struct stk3171_priv *stk3171_obj = NULL;
static struct platform_driver stk3171_alsps_driver;
static int stk3171_get_ps_value(struct stk3171_priv *obj, u16 ps);
static int stk3171_get_als_value(struct stk3171_priv *obj, u16 als);
static int stk3171_read_als(struct i2c_client *client, u16 *data);
static int stk3171_read_ps(struct i2c_client *client, u8 *data);
/*----------------------------------------------------------------------------*/
int stk3171_get_addr(struct alsps_hw *hw, struct stk3171_i2c_addr *addr)
{
	if(!hw || !addr)
	{
		return -EFAULT;
	}
	addr->status    = ALSPS_STATUS; 
	addr->als_cmd   = ALS_CMD;         
	addr->als_dat1  = ALS_DT1;
	addr->als_dat2  = ALS_DT2;
	addr->ps_cmd    = PS_CMD;
	addr->ps_high_thd    = PS_THDH;
	addr->ps_low_thd    = PS_THDL;
	addr->ps_dat    = PS_DT;
	addr->ps_gain = PS_GAIN;
	addr->als_high_thd1 = ALS_THDH1;
	addr->als_high_thd2 = ALS_THDH2;
	addr->als_low_thd1 = ALS_THDL1;
	addr->als_low_thd2 = ALS_THDL2;	
	
	return 0;
}
/*----------------------------------------------------------------------------*/
int stk3171_get_timing(void)
{
return 200;
/*
	u32 base = I2C2_BASE; 
	return (__raw_readw(mt6516_I2C_HS) << 16) | (__raw_readw(mt6516_I2C_TIMING));
*/
}
/*----------------------------------------------------------------------------*/
/*
int stk3171_config_timing(int sample_div, int step_div)
{
	u32 base = I2C2_BASE; 
	unsigned long tmp;

	tmp  = __raw_readw(mt6516_I2C_TIMING) & ~((0x7 << 8) | (0x1f << 0));
	tmp  = (sample_div & 0x7) << 8 | (step_div & 0x1f) << 0 | tmp;

	return (__raw_readw(mt6516_I2C_HS) << 16) | (tmp);
}
*/
/*----------------------------------------------------------------------------*/
int stk3171_master_recv(struct i2c_client *client, u16 addr, u8 *buf ,int count)
{
	struct stk3171_priv *obj = i2c_get_clientdata(client);        
	struct i2c_adapter *adap = client->adapter;
	struct i2c_msg msg;
	int ret = 0, retry = 0;
	int trc = atomic_read(&obj->trace);
	int max_try = atomic_read(&obj->i2c_retry);

	while(retry++ < max_try)
	{
		ret = hwmsen_read_block(client, addr, buf, count);
		if(ret == 0)
            break;
		udelay(100);
	}

	if(unlikely(trc))
	{
		if(trc & STK_TRC_I2C)
		{
			printk("(recv) %x %d %d %p [%02X]\n", msg.addr, msg.flags, msg.len, msg.buf, msg.buf[0]);    
		}

		if((retry != 1) && (trc & STK_TRC_DEBUG))
		{
			printk("(recv) %d/%d\n", retry-1, max_try); 

		}
	}

	/* If everything went ok (i.e. 1 msg transmitted), return #bytes
	transmitted, else error code. */
	return (ret == 0) ? count : ret;
}
/*----------------------------------------------------------------------------*/
int stk3171_master_send(struct i2c_client *client, u16 addr, u8 *buf ,int count)
{
	int ret = 0, retry = 0;
	struct stk3171_priv *obj = i2c_get_clientdata(client);        
	struct i2c_adapter *adap=client->adapter;
	struct i2c_msg msg;
	int trc = atomic_read(&obj->trace);
	int max_try = atomic_read(&obj->i2c_retry);


	while(retry++ < max_try)
	{
		ret = hwmsen_write_block(client, addr, buf, count);
		if (ret == 0)
		    break;
		udelay(100);
	}

	if(unlikely(trc))
	{
		if(trc & STK_TRC_I2C)
		{
			printk("(send) %x %d %d %p [%02X]\n", msg.addr, msg.flags, msg.len, msg.buf, msg.buf[0]);    
		}

		if((retry != 1) && (trc & STK_TRC_DEBUG))
		{
			printk("(send) %d/%d\n", retry-1, max_try);
		}
	}
	/* If everything went ok (i.e. 1 msg transmitted), return #bytes
	transmitted, else error code. */
	return (ret == 0) ? count : ret;
}
/*----------------------------------------------------------------------------*/
int stk3171_read_als(struct i2c_client *client, u16 *data)
{
	struct stk3171_priv *obj = i2c_get_clientdata(client);    
	int ret = 0;
	u8 buf[2];
/*	ALS_DT1  :  MSB
	ALS_DT2  :  LSB	*/
down(&ps_set_mode);
#if 0
	if(1 != (ret = stk3171_master_recv(client, obj->addr.als_dat1, (char*)&buf[1], 1)))
	{
		APS_DBG("reads als data1 = %d\n", ret);
		return -EFAULT;
	}
	else if(1 != (ret = stk3171_master_recv(client, obj->addr.als_dat2, (char*)&buf[0], 1)))
	{
		APS_DBG("reads als data2 = %d\n", ret);
		return -EFAULT;
	}
#else
	buf[1] = i2c_smbus_read_byte_data(client, ALS_DT1);
	buf[0] = i2c_smbus_read_byte_data(client, ALS_DT2);
#endif
	*data = (buf[1] << 8) | (buf[0]);
	if(atomic_read(&obj->trace) & STK_TRC_ALS_DATA)
	{
		APS_DBG("ALS: 0x%04X\n", (u32)(*data));
	}

	*data = (*data)*3; // add for als ratio
	
#ifdef VENDOR_EDIT
	if (alsps_dbg_flag){
		printk(APS_TAG"als = %d\n",*data);
	}
#endif
up(&ps_set_mode);
	return 0;    
}
/*----------------------------------------------------------------------------*/
int stk3171_write_als(struct i2c_client *client, u8 data)
{
	struct stk3171_priv *obj = i2c_get_clientdata(client);
	int ret = 0;
    
    ret = stk3171_master_send(client, obj->addr.als_cmd, &data, 1);
	if(ret < 0)
	{
		APS_ERR("write als = %d\n", ret);
		return -EFAULT;
	}
	
	return 0;    
}
/*----------------------------------------------------------------------------*/
int stk3171_read_ps(struct i2c_client *client, u8 *data)
{
	struct stk3171_priv *obj = i2c_get_clientdata(client);    
	int ret = 0;

#if 0
	if(sizeof(*data) != (ret = stk3171_master_recv(client, obj->addr.ps_dat, (char*)data, sizeof(*data))))
	{
		APS_ERR("reads ps data = %d\n", ret);
		return -EFAULT;
	} 
#else
	ret = i2c_smbus_read_byte_data(client, PS_DT);
	if (ret < 0)
	{
		printk(KERN_ERR"%s  read ps value fail !!!\n",__func__);
	}
	*data = ret;
#endif

	if(atomic_read(&obj->trace) & STK_TRC_PS_DATA)
	{
		APS_DBG("PS:  0x%04X\n", (u32)(*data));
	}
	return 0;    
}
/*----------------------------------------------------------------------------*/
int stk3171_write_ps(struct i2c_client *client, u8 data)
{
	struct stk3171_priv *obj = i2c_get_clientdata(client);        
	int ret = 0;

    ret = stk3171_master_send(client, obj->addr.ps_cmd, &data, 1);
	if (ret < 0)
	{
		APS_ERR("write ps = %d\n", ret);
		return -EFAULT;
	} 
	return 0;    
}
/*----------------------------------------------------------------------------*/
int stk3171_read_ps_gain(struct i2c_client *client, u8 *data)
{
	struct stk3171_priv *obj = i2c_get_clientdata(client);    
	int ret = 0;

	if(sizeof(*data) != (ret = stk3171_master_recv(client, obj->addr.ps_gain, (char*)data, sizeof(*data))))
	{
		APS_ERR("reads ps gain = %d\n", ret);
		return -EFAULT;
	} 

	APS_DBG("PS gain:  0x%04X\n", (u32)(*data));

	return 0;    
}
/*----------------------------------------------------------------------------*/
int stk3171_write_ps_gain(struct i2c_client *client, u8 data)
{
	struct stk3171_priv *obj = i2c_get_clientdata(client);        
	int ret = 0;

    ret = stk3171_master_send(client, obj->addr.ps_gain, &data, 1);
	if (ret < 0)
	{
		APS_ERR("write ps gain = %d\n", ret);
		return -EFAULT;
	} 
	return 0;    
}
/*----------------------------------------------------------------------------*/
int stk3171_write_ps_high_thd(struct i2c_client *client, u8 thd)
{
	struct stk3171_priv *obj = i2c_get_clientdata(client);        
	u8 buf = thd;
	int ret = 0;

	if(sizeof(buf) != (ret = stk3171_master_send(client, obj->addr.ps_high_thd, (char*)&buf, sizeof(buf))))
	{
		APS_ERR("write thd = %d\n", ret);
		return -EFAULT;
	} 
	return 0;    
}
/*----------------------------------------------------------------------------*/
int stk3171_write_ps_low_thd(struct i2c_client *client, u8 thd)
{
	struct stk3171_priv *obj = i2c_get_clientdata(client);        
	u8 buf = thd;
	int ret = 0;

	if(sizeof(buf) != (ret = stk3171_master_send(client, obj->addr.ps_low_thd, (char*)&buf, sizeof(buf))))
	{
		APS_ERR("write thd = %d\n", ret);
		return -EFAULT;
	} 
	return 0;    
}
/*----------------------------------------------------------------------------*/
static unsigned int power_status = 0;
static void stk3171_power(struct alsps_hw *hw, unsigned int on) 
{
	printk("%s  power %s\n",__func__ ,on ? "on" : "off");
#if 0
	if(hw->power_id != POWER_NONE_MACRO)
	{
		if(power_on == on)
		{
			printk("ignore power control: %d\n", on);
		}
		else if(on)
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "stk3171")) 
			{
				APS_ERR("power on fails!!\n");
			}
		}
		else
		{
			if(!hwPowerDown(hw->power_id, "stk3171")) 
			{
				APS_ERR("power off fail!!\n");   
			}
		}
	}
#else
	hw->power(hw, on, NULL);
#endif
	power_status = on;
}

/*----------------------------------------------------------------------------*/
static int stk3171_enable_als(struct i2c_client *client, int enable)
{
	struct stk3171_priv *obj = i2c_get_clientdata(client);
	int err, cur = 0, old = atomic_read(&obj->als_cmd_val);
	int trc = atomic_read(&obj->trace);

	printk("%s: enable=%d\n", __func__, enable);
	if(enable)
	{
		cur = old & (~SD_ALS);   
	}
	else
	{
		cur = old | (SD_ALS); 
	}
	
	if(trc & STK_TRC_DEBUG)
	{
		printk("%s: 0x%08X, 0x%08X, %d\n", __func__, cur, old, enable);
	}
#if 0
	if(0 == (cur ^ old))
	{
		return 0;
	}
	
	if(0 == (err = stk3171_write_als(client, cur))) 
	{
		atomic_set(&obj->als_cmd_val, cur);
	}
#else
	err = stk3171_write_als(client, cur);
#endif
	
	if(enable)
	{
		intensity_level = 0;
		temp_level = 5500;
		atomic_set(&obj->als_deb_on, 1);
		atomic_set(&obj->als_deb_end, jiffies+atomic_read(&obj->als_debounce)/(1000/HZ));
		set_bit(STK_BIT_ALS,  &obj->pending_intr);
		set_bit(STK_BIT_ALS, &obj->enable);
		//schedule_delayed_work(&obj->eint_work,260); //after enable the value is not accurate
	} else clear_bit(STK_BIT_ALS, &obj->enable);

	if(trc & STK_TRC_DEBUG)
	{
		printk("enable als (%d)\n", enable);
	}

	return err;
}
/*----------------------------------------------------------------------------*/
static void stk3171_set_ps_threshold(struct i2c_client *client, int low_threshold, int high_threshold);
static int stk3171_clear_intr(struct i2c_client *client , u8 status, u8 disable_flag);
static int stk3171_enable_ps(struct i2c_client *client, int enable)
{
	struct stk3171_priv *obj = i2c_get_clientdata(client);
	int err, cur = 0, old = atomic_read(&obj->ps_cmd_val);
	int trc = atomic_read(&obj->trace);
	hwm_sensor_data sensor_data;
	printk("%s: enable=%d\n", __func__, enable);
#ifdef VENDOR_EDIT
        g_is_far = 1; //initial this parama with FAR
        g_ps_low_threshold = 0;
        g_ps_high_threshold = 255;
#endif
	if (g_ps_switch_off == 1)
	{
		printk("%s  ps is unuseable\n",__func__);
		return 0;
	}

	if(enable)
	{
		cur = (old & (~INT_PS)) & (~SD_PS);   //open ps with polling mode
        //wake_lock(&ps_lock);
		if(!test_bit(STK_BIT_ALS, &obj->enable)){
			i2c_smbus_write_byte_data(obj->client, obj->addr.als_cmd, 0x48);
			printk("%s  open ALS\n",__func__);
		}

	}
	else
	{
		cur = (old & (~INT_PS)) | (SD_PS);  //close ps with polling mode
        //wake_unlock(&ps_lock);
		if(!test_bit(STK_BIT_ALS, &obj->enable)){
			i2c_smbus_write_byte_data(obj->client, obj->addr.als_cmd, 0x49);
			printk("%s  shut down ALS\n",__func__);
		}
	}
	
	if(trc & STK_TRC_DEBUG)
	{
		printk("%s: %08X, %08X, %d\n", __func__, cur, old, enable);
	}
#if 0
	if(0 == (cur ^ old))
	{
		return 0;
	}
#endif
	if(0 == (err = stk3171_write_ps(client, cur))) 
	{
		//atomic_set(&obj->ps_cmd_val, cur);
	} else {
		printk("%s  write ps fail\n",__func__);
		return err;
	}
	
	if(enable)
	{
#ifdef VENDOR_EDIT /* 2012-6-20 zhye Add for  PS*/
		ps_work_status = 1;
		distance = 1;
		prev_dist = 1;
		ps_invalid_flags = 0;
		ps_eint_mode = 0;
		printk("%s enable  PS   ps_work_status = %d\n",__func__,ps_work_status);	
#endif
		atomic_set(&obj->ps_deb_on, 1);
		atomic_set(&obj->ps_deb_end, jiffies+atomic_read(&obj->ps_debounce)/(1000/HZ));
		set_bit(STK_BIT_PS,  &obj->pending_intr);
		stk3171_set_ps_threshold(obj->client,g_ps_low_threshold,g_ps_high_threshold);

		{
			sensor_data.values[0] = 1;
			sensor_data.value_divide = 1;
			sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
			printk("%s  First start ,send a far event to up layer\n",__func__);
			//let up layer to know
			hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data);
			stk3171_clear_intr(client, ALSPS_STATUS, 0x0); // clear int when enable
			mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
		}
	}else {
		mt65xx_eint_mask(CUST_EINT_ALS_NUM);
		ps_work_status = 0;

		atomic_set(&obj->ps_deb_on, 0);
		printk("%s  disable  PS   ps_work_status = %d\n",__func__,ps_work_status);
	}

	if(trc & STK_TRC_DEBUG)
	{
		printk("enable ps  (%d)\n", enable);
	}
	
	if (g_stk3171_ptr->fasync != NULL)
	{
		APS_LOG("kill a SIGIO when %s", __func__);
		kill_fasync(&g_stk3171_ptr->fasync, SIGIO, POLL_IN);
	}
	
	return err;
}
/*----------------------------------------------------------------------------*/
static int stk3171_check_intr(struct i2c_client *client, u8 *status) 
{
	struct stk3171_priv *obj = i2c_get_clientdata(client);
	int err;

	//if (mt_get_gpio_in(GPIO_ALS_EINT_PIN) == 1) /*skip if no interrupt*/  
	//    return 0;

    err = stk3171_master_recv(client, obj->addr.status, status, 1);
	if (err < 0)
	{
		APS_ERR("WARNING: read status error: %d\n", err);
		return -EFAULT;
	}
	printk("stk3171_check_intr: read status reg: 0x%x\n", *status);
    
	if(*status & 0x10)
	{
		set_bit(STK_BIT_ALS, &obj->pending_intr);
	}
	else
	{
	   clear_bit(STK_BIT_ALS, &obj->pending_intr);
	}
	
	if(*status & 0x20)
	{
		set_bit(STK_BIT_PS,  &obj->pending_intr);
	}
	else
	{
	    clear_bit(STK_BIT_PS, &obj->pending_intr);
	}
	
	if(atomic_read(&obj->trace) & STK_TRC_DEBUG)
	{
		printk("check intr: 0x%02X => 0x%08lX\n", *status, obj->pending_intr);
	}

	return 0;
}

static int stk3171_clear_intr(struct i2c_client *client , u8 status, u8 disable_flag) 
{
#if 1
    struct stk3171_priv *obj = i2c_get_clientdata(client);
    int err;

	status &= 0x30;
    status = status & (~disable_flag);
	printk("stk3171_clear_intr: set status reg = 0x%x to value = 0x%x\n",obj->addr.status, status);
    err = stk3171_master_send(client, obj->addr.status, &status, 1);    
    if (err < 0)
    {
		APS_ERR("ERROR: stk3171_clear_intr clear intrrupt fail: %d\n", err);
		return -EFAULT;
	}
#else
	int ret;
	ret = i2c_smbus_write_byte_data(client, ALSPS_STATUS, 0x0);
	if (ret < 0)
	{
		printk(KERN_ERR"stk3171 clear interrupt fail !!!\n");
		return -EFAULT;
	}
#endif
    return 0;
}

/*----------------------------------------------------------------------------*/
/*
static u16 stk3171_lux2alscode(u16 input_lux)
{
	u32 ratio;
	u32 output_code;

	if(!stk3171_obj)
	{
		APS_ERR("stk3171_obj is null!!\n");
		return 0;
	}
	
	ratio = (stk3171_obj->hw->als_value[10] << 10 ) / stk3171_obj->hw->als_level[10];
	output_code = (input_lux<<10) / ratio;
	
	if (unlikely(output_code>=(1<<16)))
        output_code = (1<<16) -1;
	return (u16)output_code;
}
*/
/*----------------------------------------------------------------------------*/
static int stk3171_set_als_int_thd(struct i2c_client *client, u16 als_data_reg) 
{
	struct stk3171_priv *obj = i2c_get_clientdata(client);
	int err;
	u32 als_thd_h, als_thd_l;
	u8 buf[2] = {0,0};		
		
    als_thd_h = als_data_reg + CONFIG_STK_ALS_CHANGE_THRESHOLD;
    als_thd_l = als_data_reg - CONFIG_STK_ALS_CHANGE_THRESHOLD;
    if (als_thd_h >= (1<<16))
        als_thd_h = (1<<16) -1;
    if (als_thd_l <0)
        als_thd_l = 0;
	printk("stk3171_set_als_int_thd:als_thd_h:%d,als_thd_l:%d\n", als_thd_h, als_thd_l);	
		
    buf[0] = (u8) ((0xFF00 & als_thd_h) >> 8);
    buf[1] = (u8) (0x00FF & als_thd_h);
	//printk("%s:als_thd_h, buf[0]=0x%x, buf[1]=0x%x\n", __func__, buf[0], buf[1]);
    err = stk3171_master_send(client, obj->addr.als_high_thd1, &buf, 1);
	if (err < 0)
	{
		APS_ERR("WARNING: %s: %d\n", __func__, err);
		return 0;
	}

    err = stk3171_master_send(client, obj->addr.als_high_thd2, &(buf[1]), 1);
	if (err < 0)
	{
		APS_ERR("WARNING: %s: %d\n", __func__, err);
		return 0;
	}
	
    buf[0] = (u8) ((0xFF00 & als_thd_l) >> 8);
    buf[1] = (u8) (0x00FF & als_thd_l);
	//printk("%s:als_thd_l, buf[0]=0x%x, buf[1]=0x%x\n", __func__, buf[0], buf[1]);	
    err = stk3171_master_send(client, obj->addr.als_low_thd1, &buf, 1);
	if (err < 0)
	{
		APS_ERR("WARNING: %s: %d\n", __func__, err);
		return 0;
	}	

    err = stk3171_master_send(client, obj->addr.als_low_thd2, &(buf[1]), 1);
	if (err < 0)
	{
		APS_ERR("WARNING: %s: %d\n", __func__, err);
		return 0;
	}	

	return 0;
}
/*----------------------------------------------------------------------------*/
static int stk3171_set_ps_mode(struct stk3171_priv *obj, int ps_eint)
{
	int ret;
	u8 val;
	u8 tmp = atomic_read(&obj->ps_cmd_val);
	printk("%s  set ps mode from %s to %s\n",__func__, ps_eint==0?"eint":"polling",ps_eint==0?"polling":"eint");
down(&ps_set_mode);
	if (ps_eint){
		val = tmp | SD_PS; // shut down ps
		ret = stk3171_write_ps(obj->client, val);
		msleep(10);
		val = (tmp | INT_PS) & ~SD_PS; // open ps with eint mode
		ret |= stk3171_write_ps(obj->client, val);
	} else {
		val = tmp | SD_PS; // shut down ps
		ret |= stk3171_write_ps(obj->client, val);
		msleep(10);
		val = (tmp & ~INT_PS) & ~SD_PS; // open ps with polling mode
		ret |= stk3171_write_ps(obj->client, val);
	}
	ps_eint_mode = ps_eint;
	msleep(10);
up(&ps_set_mode);
	return ret;
}
/*----------------------------------------------------------------------------*/
void stk3171_eint_func(void)
{
#if 0
	printk("%s  Interrupt coming here !!!\n",__func__);
	return; // add because interrupt come abnormal
#else
	struct stk3171_priv *obj = g_stk3171_ptr;

	printk(APS_TAG"%s  Interrupt func\n",__func__);

	if (ps_work_status == 0)
	{
		return;
	}
	
	if(!obj)
	{
		return;
	}
	schedule_work(&obj->eint_work);
	//schedule_delayed_work(&obj->eint_work,0);
	if(atomic_read(&obj->trace) & STK_TRC_EINT)
	{
		printk("eint: als/ps intrs\n");
	}
	//printk(APS_TAG"%s  leave \n",__func__);
#endif
}
/*----------------------------------------------------------------------------*/
static void stk3171_eint_work(struct work_struct *work)
{
	struct stk3171_priv *obj = g_stk3171_ptr;
	int err;
	hwm_sensor_data sensor_data;
	u8 int_status,disable_flag = 0;

	memset(&sensor_data, 0, sizeof(sensor_data));

	printk("%s  Enter !!!\n",__func__);
	
	if((err = stk3171_check_intr(obj->client, &int_status)))
	{
		printk(APS_TAG"check intrs fail: %d\n", err);
		msleep(30);
		mt65xx_eint_unmask(CUST_EINT_ALS_NUM);      		
		return;
	}


#if 0
	printk("%s  &obj->pending_intr =%lx\n",__func__,obj->pending_intr);
	if((1<<STK_BIT_ALS) & obj->pending_intr)
	{
	  //get raw data
	  printk("%s  stk als change\n",__func__);
	  disable_flag = 0x10;
	  if((err = stk3171_read_als(obj->client, &obj->als)))
	  {
		 APS_ERR("stk3171 read als data: %d\n", err);
	  }
	  stk3171_set_als_int_thd(obj->client, obj->als);
	  //map and store data to hwm_sensor_data
 	  sensor_data.values[0] = stk3171_get_als_value(obj, obj->als);
	  sensor_data.value_divide = 1;
	  sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
	  printk("%s:als raw 0x%x -> value 0x%x \n", __func__, obj->als,sensor_data.values[0]);
	  //let up layer to know
	  if((err = hwmsen_get_interrupt_data(ID_LIGHT, &sensor_data)))
	  {
		APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);
	  }
	  
	}
#endif
	//else//if((1<<STK_BIT_PS) &  obj->pending_intr)
	{
		//get raw data
		//printk("%s  stk ps change\n",__func__);
		disable_flag |= 0x20;
	#if 1
		if((err = stk3171_read_ps(obj->client, &obj->ps)))
		{
			APS_ERR("stk3171 read ps data: %d\n", err);
		}
	#endif
		sensor_data.values[0] &= 0; //to avoid unexpected data
		
		//map and store data to hwm_sensor_data
		sensor_data.values[0] = stk3171_get_ps_value(obj, obj->ps);
		while(-1 == sensor_data.values[0])
		{
			if (ps_invalid_flags == 1){
				printk("%s  This state is coming too soon, not invalid !!!\n",__func__);
				sensor_data.values[0] = 1;
				ps_invalid_flags = 0;
			} else 
				sensor_data.values[0] = distance;
			printk(APS_TAG"%s  Wrong state, assign a RIGHT ONE  %d\n",__func__, sensor_data.values[0]);
		}
		sensor_data.value_divide = 1;
		sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
		stk3171_set_ps_threshold(obj->client, g_ps_low_threshold, g_ps_high_threshold);
		printk("%s  set ps threshold  low=%d  high=%d\n",__func__,g_ps_low_threshold, g_ps_high_threshold);
		printk("%s : PS=%d -> state=%d \n",__func__, obj->ps,sensor_data.values[0]);
		//let up layer to know
		if (prev_dist != sensor_data.values[0]){
			if((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
			{
				APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);
			}
			prev_dist = sensor_data.values[0];
		} else 
		{	
			printk("%s  The same as last status, NOT reprot !!!\n",__func__);
		}
	}
#if 0
	if(err = stk3171_clear_intr(obj->client , int_status, disable_flag))
	{
		APS_ERR("stk3171_clear_intr fail: %d\n", err);
		msleep(30);
	}	
#else /*clear interrupt*/
	g_is_far = distance;
	err = i2c_smbus_write_byte_data(obj->client, ALSPS_STATUS, 0x0);
	if (err < 0)
	{
		printk(KERN_ERR"stk3171 clear interrupt fail !!!\n");
		return;
	}
#endif
	msleep(1);

	if ((distance == 1) && (ps_eint_mode == 1)){
		stk3171_set_ps_mode(obj, 0);
	}

	mt65xx_eint_unmask(CUST_EINT_ALS_NUM);

	printk("%s  Leave !!!\n",__func__);
}
/*----------------------------------------------------------------------------*/
int stk3171_setup_eint(struct i2c_client *client)
{
	struct stk3171_priv *obj = i2c_get_clientdata(client);        

	g_stk3171_ptr = obj;
	/*configure to GPIO function, external interrupt*/

	printk("ALS/PS  %s  interrupt pin = %d\n",__func__, GPIO_ALS_EINT_PIN);
	
	mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);
	
	//mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
	//mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
	//mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, GPIO_PULL_ENABLE);
	//mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);

	mt65xx_eint_set_sens(CUST_EINT_ALS_NUM, CUST_EINT_ALS_SENSITIVE);
	mt65xx_eint_set_polarity(CUST_EINT_ALS_NUM, CUST_EINT_ALS_POLARITY);
	mt65xx_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
	mt65xx_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_EN, CUST_EINT_ALS_POLARITY, stk3171_eint_func, 0);
/*zhye modify for disable interrupt when init device*/
	#if 0
		mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
	#else
		mt65xx_eint_mask(CUST_EINT_ALS_NUM);
	#endif
/*zhye modify  end*/

    return 0;
	
}
/*----------------------------------------------------------------------------*/
static int stk3171_dump_all_reg(struct i2c_client *client)
{
	u8 val = 0;
	//u8 reg = 0x01;
	u8 i;
	int ret;
	for (i = 0x01; i <= 0x0c; i += 0x01)
	{
		i2c_master_send(client, &i, 1);
		i2c_master_recv(client, &val, 1);
		printk("%s  Reg 0x%x = 0x%x\n",__func__,i, val);
	}
	
	for (i = 0x81; i < 0x84; i += 0x01){
		i2c_master_send(client, &i, 1);
		i2c_master_recv(client, &val, 1);
		printk("%s  Reg 0x%x = 0x%x\n",__func__,i, val);
	}
	
	return 0;
}
/*----------------------------------------------------------------------------*/
static int stk3171_init_client(struct i2c_client *client)
{
	struct stk3171_priv *obj = i2c_get_clientdata(client);
	int err;
	u8 gain;
	u8 int_status;
	int i = 0;
	u8 reg = 0x81,reg_val;
	
	//if (obj->hw->polling_mode_ps == 0)
	{
		if((err = stk3171_check_intr(client, &int_status)))
		{
			APS_ERR("check intr: %d\n", err);
			//    return err;
		}
	
		if(err = stk3171_clear_intr(client, int_status, 0x30))
		{
			APS_ERR("clear intr: %d\n", err);	
		}
	}
	
	if((err = stk3171_write_als(client, atomic_read(&obj->als_cmd_val))))
	{
		APS_ERR("write als: %d\n", err);
		return err;
	}
	
	if((err = stk3171_write_ps(client, atomic_read(&obj->ps_cmd_val))))
	{
		APS_ERR("write ps: %d\n", err);
		return err;        
	}
#if 0
	if((err = stk3171_write_ps_high_thd(client, atomic_read(&obj->ps_high_thd_val))))
	{
		APS_ERR("write thd: %d\n", err);
		return err;        
	}
	
	if((err = stk3171_write_ps_high_thd(client, atomic_read(&obj->ps_high_thd_val))))
	{
		APS_ERR("write thd: %d\n", err);
		return err;        
	}
	
	if((err = stk3171_write_ps_low_thd(client, atomic_read(&obj->ps_low_thd_val))))
	{
		APS_ERR("write thd: %d\n", err);
		return err;        
	}
#endif
	/*Read PS Gain Register*/
	if((err = stk3171_read_ps_gain(client, &gain)))
	{
		APS_ERR("read ps gain: %d\n", err);
		return err;
	}
	
	gain = (gain&0xF0)|gain_setting; //Setting PS Gain
	printk("PS GAIN SETTING = 0x%x\n",gain_setting);
	printk("%s ps gain = 0x%x\n",__func__,gain);
	printk("%s ps int_status = 0x%x\n",__func__,int_status);
	printk("%s als_cmd_val = 0x%x\n",__func__,atomic_read(&obj->als_cmd_val));
	printk("%s ps_cmd_val = 0x%x\n",__func__,atomic_read(&obj->ps_cmd_val));
	/*Write PS Gain*/
	if((err = stk3171_write_ps_gain(client, gain)))
	{
		APS_ERR("write ps gain: %d\n", err);
		return err;
	}
	/*Set PS Gain END*/
	printk("%s  setup initerrupt !!!\n",__func__);
	if((err = stk3171_setup_eint(client)))
	{
		APS_ERR("setup eint: %d\n", err);
		return err;
	}

#ifdef VENDOR_EDIT
	if (init_count < 1){
		for (i = 0; i < 3; i++){
			reg8[i] = i2c_smbus_read_byte_data(client, reg);
			if (reg8[i] < 0){
				printk("%s  read reg 0x%x fail\n",__func__,reg8[i]);
				return reg8[i];
			}
			printk("%s read 0x%x = 0x%x\n",__func__,reg, reg8[i]);
			reg += 0x01;
		}
	}
#endif
	
	if(err = stk3171_dump_all_reg(client))
	{
		APS_ERR("dump all reg fail: %d\n", err);
		return err;
	}
	init_count++;
	return 0;
}
/******************************************************************************
 * Sysfs attributes
*******************************************************************************/
static ssize_t stk3171_show_config(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	
	if(!stk3171_obj)
	{
		APS_ERR("stk3171_obj is null!!\n");
		return 0;
	}
	
	res = snprintf(buf, PAGE_SIZE, "(%d %d %d %d %d %d)\n", 
		atomic_read(&stk3171_obj->i2c_retry), atomic_read(&stk3171_obj->als_debounce), 
		atomic_read(&stk3171_obj->ps_mask), atomic_read(&stk3171_obj->ps_high_thd_val),atomic_read(&stk3171_obj->ps_low_thd_val), atomic_read(&stk3171_obj->ps_debounce));     
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3171_store_config(struct device_driver *ddri, const char *buf, size_t count)
{
	int retry, als_deb, ps_deb, mask, hthres, lthres;
	if(!stk3171_obj)
	{
		APS_ERR("stk3171_obj is null!!\n");
		return 0;
	}
	
	if(6 == sscanf(buf, "%d %d %d %d %d %d", &retry, &als_deb, &mask, &hthres, &lthres, &ps_deb))
	{ 
		atomic_set(&stk3171_obj->i2c_retry, retry);
		atomic_set(&stk3171_obj->als_debounce, als_deb);
		atomic_set(&stk3171_obj->ps_mask, mask);
		atomic_set(&stk3171_obj->ps_high_thd_val, hthres);    
		atomic_set(&stk3171_obj->ps_low_thd_val, lthres);        
		atomic_set(&stk3171_obj->ps_debounce, ps_deb);
	}
	else
	{
		APS_ERR("invalid content: '%s', length = %d\n", buf, count);
	}
	return count;    
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3171_show_trace(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	if(!stk3171_obj)
	{
		APS_ERR("stk3171_obj is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&stk3171_obj->trace));     
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3171_store_trace(struct device_driver *ddri, const char *buf, size_t count)
{
    int trace;
    if(!stk3171_obj)
	{
		APS_ERR("stk3171_obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&stk3171_obj->trace, trace);
	}
	else 
	{
		APS_ERR("invalid content: '%s', length = %d\n", buf, count);
	}
	return count;    
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3171_show_als(struct device_driver *ddri, char *buf)
{
	int res;
	
	if(!stk3171_obj)
	{
		APS_ERR("stk3171_obj is null!!\n");
		return 0;
	}
	if((res = stk3171_read_als(stk3171_obj->client, &stk3171_obj->als)))
	{
		return snprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
	}
	else
	{
		return snprintf(buf, PAGE_SIZE, "0x%04X\n", stk3171_obj->als);     
	}
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3171_show_ps(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	if(!stk3171_obj)
	{
		APS_ERR("stk3171_obj is null!!\n");
		return 0;
	}
	
	if((res = stk3171_read_ps(stk3171_obj->client, &stk3171_obj->ps)))
	{
		return snprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
	}
	else
	{
		return snprintf(buf, PAGE_SIZE, "0x%04X\n", stk3171_obj->ps);     
	}
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3171_show_reg(struct device_driver *ddri, char *buf)
{
	u8 int_status;
	if(!stk3171_obj)
	{
		APS_ERR("stk3171_obj is null!!\n");
		return 0;
	}
	
	/*read*/
	stk3171_check_intr(stk3171_obj->client, &int_status);
	stk3171_clear_intr(stk3171_obj->client, int_status, 0x0);
	stk3171_read_ps(stk3171_obj->client, &stk3171_obj->ps);
	stk3171_read_als(stk3171_obj->client, &stk3171_obj->als);
	/*write*/
	stk3171_write_als(stk3171_obj->client, atomic_read(&stk3171_obj->als_cmd_val));
	stk3171_write_ps(stk3171_obj->client, atomic_read(&stk3171_obj->ps_cmd_val)); 
	stk3171_write_ps_high_thd(stk3171_obj->client, atomic_read(&stk3171_obj->ps_high_thd_val));
	stk3171_write_ps_low_thd(stk3171_obj->client, atomic_read(&stk3171_obj->ps_low_thd_val));
	return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3171_show_send(struct device_driver *ddri, char *buf)
{
    return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3171_store_send(struct device_driver *ddri, const char *buf, size_t count)
{
	int addr, cmd;
	u8 dat;

	if(!stk3171_obj)
	{
		APS_ERR("stk3171_obj is null!!\n");
		return 0;
	}
	else if(2 != sscanf(buf, "%x %x", &addr, &cmd))
	{
		APS_ERR("invalid format: '%s'\n", buf);
		return 0;
	}

	dat = (u8)cmd;
	printk("send(%02X, %02X) = %d\n", addr, cmd, 
	stk3171_master_send(stk3171_obj->client, (u16)addr, &dat, sizeof(dat)));
	
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3171_show_recv(struct device_driver *ddri, char *buf)
{
	if(!stk3171_obj)
	{
		APS_ERR("stk3171_obj is null!!\n");
		return 0;
	}
	return snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&stk3171_obj->recv_reg));     	
  //  return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3171_store_recv(struct device_driver *ddri, const char *buf, size_t count)
{
	int addr;
	u8 dat;
	if(!stk3171_obj)
	{
		APS_ERR("stk3171_obj is null!!\n");
		return 0;
	}
	else if(1 != sscanf(buf, "%x", &addr))
	{
		APS_ERR("invalid format: '%s'\n", buf);
		return 0;
	}

	printk("recv(%02X) = %d, 0x%02X\n", addr, 
	stk3171_master_recv(stk3171_obj->client, (u16)addr, (char*)&dat, sizeof(dat)), dat);
	atomic_set(&stk3171_obj->recv_reg, dat);	
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3171_show_status(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	
	if(!stk3171_obj)
	{
		APS_ERR("stk3171_obj is null!!\n");
		return 0;
	}
	
	if(stk3171_obj->hw)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d, (%d %d) (%02X %02X) (%02X %02X %02X) (%02X %02X %02X)\n", 
			stk3171_obj->hw->i2c_num, stk3171_obj->hw->power_id, stk3171_obj->hw->power_vol, stk3171_obj->addr.init, 
			stk3171_obj->addr.status,stk3171_obj->addr.als_cmd, stk3171_obj->addr.als_dat2, stk3171_obj->addr.als_dat1,
			stk3171_obj->addr.ps_cmd, stk3171_obj->addr.ps_dat, stk3171_obj->addr.ps_high_thd);
	}
	else
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}
	
	len += snprintf(buf+len, PAGE_SIZE-len, "REGS: %02X %02X %02X %02X %02lX %02lX\n", 
				atomic_read(&stk3171_obj->als_cmd_val), atomic_read(&stk3171_obj->ps_cmd_val), 
				atomic_read(&stk3171_obj->ps_high_thd_val), atomic_read(&stk3171_obj->ps_low_thd_val),stk3171_obj->enable, stk3171_obj->pending_intr);
	#ifdef MT6516
	len += snprintf(buf+len, PAGE_SIZE-len, "EINT: %d (%d %d %d %d)\n", mt_get_gpio_in(GPIO_ALS_EINT_PIN),
				CUST_EINT_ALS_NUM, CUST_EINT_ALS_POLARITY, CUST_EINT_ALS_DEBOUNCE_EN, CUST_EINT_ALS_DEBOUNCE_CN);

	len += snprintf(buf+len, PAGE_SIZE-len, "GPIO: %d (%d %d %d %d)\n",	GPIO_ALS_EINT_PIN, 
				mt_get_gpio_dir(GPIO_ALS_EINT_PIN), mt_get_gpio_mode(GPIO_ALS_EINT_PIN), 
				mt_get_gpio_pull_enable(GPIO_ALS_EINT_PIN), mt_get_gpio_pull_select(GPIO_ALS_EINT_PIN));
	#endif

	len += snprintf(buf+len, PAGE_SIZE-len, "MISC: %d %d\n", atomic_read(&stk3171_obj->als_suspend), atomic_read(&stk3171_obj->ps_suspend));

	return len;
}
/*----------------------------------------------------------------------------*/
/*
static ssize_t stk3171_show_i2c(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	u32 base = I2C2_BASE;

	if(!stk3171_obj)
	{
		APS_ERR("stk3171_obj is null!!\n");
		return 0;
	}
	
	len += snprintf(buf+len, PAGE_SIZE-len, "DATA_PORT      = 0x%08X\n", __raw_readl(mt6516_I2C_DATA_PORT    ));
	len += snprintf(buf+len, PAGE_SIZE-len, "SLAVE_ADDR     = 0x%08X\n", __raw_readl(mt6516_I2C_SLAVE_ADDR));
	len += snprintf(buf+len, PAGE_SIZE-len, "INTR_MASK      = 0x%08X\n", __raw_readl(mt6516_I2C_INTR_MASK));
	len += snprintf(buf+len, PAGE_SIZE-len, "INTR_STAT      = 0x%08X\n", __raw_readl(mt6516_I2C_INTR_STAT));
	len += snprintf(buf+len, PAGE_SIZE-len, "CONTROL        = 0x%08X\n", __raw_readl(mt6516_I2C_CONTROL));
	len += snprintf(buf+len, PAGE_SIZE-len, "TRANSFER_LEN   = 0x%08X\n", __raw_readl(mt6516_I2C_TRANSFER_LEN));
	len += snprintf(buf+len, PAGE_SIZE-len, "TRANSAC_LEN    = 0x%08X\n", __raw_readl(mt6516_I2C_TRANSAC_LEN));
	len += snprintf(buf+len, PAGE_SIZE-len, "DELAY_LEN      = 0x%08X\n", __raw_readl(mt6516_I2C_DELAY_LEN));
	len += snprintf(buf+len, PAGE_SIZE-len, "TIMING         = 0x%08X\n", __raw_readl(mt6516_I2C_TIMING));
	len += snprintf(buf+len, PAGE_SIZE-len, "START          = 0x%08X\n", __raw_readl(mt6516_I2C_START));
	len += snprintf(buf+len, PAGE_SIZE-len, "FIFO_STAT      = 0x%08X\n", __raw_readl(mt6516_I2C_FIFO_STAT));
	len += snprintf(buf+len, PAGE_SIZE-len, "FIFO_THRESH    = 0x%08X\n", __raw_readl(mt6516_I2C_FIFO_THRESH));
	len += snprintf(buf+len, PAGE_SIZE-len, "FIFO_ADDR_CLR  = 0x%08X\n", __raw_readl(mt6516_I2C_FIFO_ADDR_CLR));
	len += snprintf(buf+len, PAGE_SIZE-len, "IO_CONFIG      = 0x%08X\n", __raw_readl(mt6516_I2C_IO_CONFIG));
	len += snprintf(buf+len, PAGE_SIZE-len, "DEBUG          = 0x%08X\n", __raw_readl(mt6516_I2C_DEBUG));
	len += snprintf(buf+len, PAGE_SIZE-len, "HS             = 0x%08X\n", __raw_readl(mt6516_I2C_HS));
	len += snprintf(buf+len, PAGE_SIZE-len, "DEBUGSTAT      = 0x%08X\n", __raw_readl(mt6516_I2C_DEBUGSTAT));
	len += snprintf(buf+len, PAGE_SIZE-len, "DEBUGCTRL      = 0x%08X\n", __raw_readl(mt6516_I2C_DEBUGCTRL));    

	return len;
}
*/
/*----------------------------------------------------------------------------*/
/*
static ssize_t stk3171_store_i2c(struct device_driver *ddri, const char *buf, size_t count)
{
	int sample_div, step_div;
	unsigned long tmp;
	u32 base = I2C2_BASE;    

	if(!stk3171_obj)
	{
		APS_ERR("stk3171_obj is null!!\n");
		return 0;
	}
	else if(2 != sscanf(buf, "%d %d", &sample_div, &step_div))
	{
		APS_ERR("invalid format: '%s'\n", buf);
		return 0;
	}
	tmp  = __raw_readw(mt6516_I2C_TIMING) & ~((0x7 << 8) | (0x1f << 0));
	tmp  = (sample_div & 0x7) << 8 | (step_div & 0x1f) << 0 | tmp;
	__raw_writew(tmp, mt6516_I2C_TIMING);        

	return count;
}
*/
/*----------------------------------------------------------------------------*/
#define IS_SPACE(CH) (((CH) == ' ') || ((CH) == '\n'))
/*----------------------------------------------------------------------------*/
static int read_int_from_buf(struct stk3171_priv *obj, const char* buf, size_t count,
                             u32 data[], int len)
{
	int idx = 0;
	char *cur = (char*)buf, *end = (char*)(buf+count);

	while(idx < len)
	{
		while((cur < end) && IS_SPACE(*cur))
		{
			cur++;        
		}

		if(1 != sscanf(cur, "%d", &data[idx]))
		{
			break;
		}

		idx++; 
		while((cur < end) && !IS_SPACE(*cur))
		{
			cur++;
		}
	}
	return idx;
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3171_show_alslv(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	int idx;
	if(!stk3171_obj)
	{
		APS_ERR("stk3171_obj is null!!\n");
		return 0;
	}
	
	for(idx = 0; idx < stk3171_obj->als_level_num; idx++)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "%d ", stk3171_obj->hw->als_level[idx]);
	}
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	return len;    
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3171_store_alslv(struct device_driver *ddri, const char *buf, size_t count)
{
//	struct stk3171_priv *obj;
	if(!stk3171_obj)
	{
		APS_ERR("stk3171_obj is null!!\n");
		return 0;
	}
	else if(!strcmp(buf, "def"))
	{
		memcpy(stk3171_obj->als_level, stk3171_obj->hw->als_level, sizeof(stk3171_obj->als_level));
	}
	else if(stk3171_obj->als_level_num != read_int_from_buf(stk3171_obj, buf, count, 
			stk3171_obj->hw->als_level, stk3171_obj->als_level_num))
	{
		APS_ERR("invalid format: '%s'\n", buf);
	}    
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3171_show_alsval(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	int idx;
	if(!stk3171_obj)
	{
		APS_ERR("stk3171_obj is null!!\n");
		return 0;
	}
	
	for(idx = 0; idx < stk3171_obj->als_value_num; idx++)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "%d ", stk3171_obj->hw->als_value[idx]);
	}
	len += snprintf(buf+len, PAGE_SIZE-len, "\n");
	return len;    
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3171_store_alsval(struct device_driver *ddri, const char *buf, size_t count)
{
	if(!stk3171_obj)
	{
		APS_ERR("stk3171_obj is null!!\n");
		return 0;
	}
	else if(!strcmp(buf, "def"))
	{
		memcpy(stk3171_obj->als_value, stk3171_obj->hw->als_value, sizeof(stk3171_obj->als_value));
	}
	else if(stk3171_obj->als_value_num != read_int_from_buf(stk3171_obj, buf, count, 
			stk3171_obj->hw->als_value, stk3171_obj->als_value_num))
	{
		APS_ERR("invalid format: '%s'\n", buf);
	}    
	return count;
}
/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(als,     S_IWUSR | S_IRUGO, stk3171_show_als,   NULL);
static DRIVER_ATTR(ps,      S_IWUSR | S_IRUGO, stk3171_show_ps,    NULL);
static DRIVER_ATTR(config,  S_IWUSR | S_IRUGO, stk3171_show_config,stk3171_store_config);
static DRIVER_ATTR(alslv,   S_IWUSR | S_IRUGO, stk3171_show_alslv, stk3171_store_alslv);
static DRIVER_ATTR(alsval,  S_IWUSR | S_IRUGO, stk3171_show_alsval,stk3171_store_alsval);
static DRIVER_ATTR(trace,   S_IWUSR | S_IRUGO, stk3171_show_trace, stk3171_store_trace);
static DRIVER_ATTR(status,  S_IWUSR | S_IRUGO, stk3171_show_status,  NULL);
static DRIVER_ATTR(send,    S_IWUSR | S_IRUGO, stk3171_show_send,  stk3171_store_send);
static DRIVER_ATTR(recv,    S_IWUSR | S_IRUGO, stk3171_show_recv,  stk3171_store_recv);
static DRIVER_ATTR(reg,     S_IWUSR | S_IRUGO, stk3171_show_reg,   NULL);
//static DRIVER_ATTR(i2c,     S_IWUSR | S_IRUGO, stk3171_show_i2c,   stk3171_store_i2c);
/*----------------------------------------------------------------------------*/
static struct driver_attribute *stk3171_attr_list[] = {
    &driver_attr_als,
    &driver_attr_ps,    
    &driver_attr_trace,        /*trace log*/
    &driver_attr_config,
    &driver_attr_alslv,
    &driver_attr_alsval,
    &driver_attr_status,
    &driver_attr_send,
    &driver_attr_recv,
    //&driver_attr_i2c,
    &driver_attr_reg,
};

/*----------------------------------------------------------------------------*/
static int stk3171_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(stk3171_attr_list)/sizeof(stk3171_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, stk3171_attr_list[idx])))
		{            
			APS_ERR("driver_create_file (%s) = %d\n", stk3171_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
	static int stk3171_delete_attr(struct device_driver *driver)
	{
	int idx ,err = 0;
	int num = (int)(sizeof(stk3171_attr_list)/sizeof(stk3171_attr_list[0]));

	if (!driver)
	return -EINVAL;

	for (idx = 0; idx < num; idx++) 
	{
		driver_remove_file(driver, stk3171_attr_list[idx]);
	}
	
	return err;
}
/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static int stk3171_get_als_value(struct stk3171_priv *obj, u16 als)
{
	int idx;
	int invalid = 0;
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
		if (atomic_read(&obj->trace) & STK_TRC_CVT_ALS)
		{
			APS_DBG("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);
		}
		
		return obj->hw->als_value[idx];
	}
	else
	{
		if(atomic_read(&obj->trace) & STK_TRC_CVT_ALS)
		{
			APS_DBG("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);    
		}
		return -1;
	}
}
/*----------------------------------------------------------------------------*/
static int stk3171_get_ps_value(struct stk3171_priv *obj, u16 ps)
{
	int val, mask = atomic_read(&obj->ps_mask);
	int invalid = 0;
	u8 buf[2];
	u16 light_intensity;
	u8 current_ps_data;
	int ret;

	if (obj->ps >= g_ps_high_threshold){
		val = 0;
		distance = 0;
	}
	else if (obj->ps <= g_ps_low_threshold){
		val = 1;
		distance = 1;
	} else val = distance;

	if ((g_ps_low_threshold == 0)&&(g_ps_high_threshold == 255)){
		val = 1;
		distance = 1; //far
	}
	
/***************detect environment light intensity***************/
	if (val == 0) //when ps value is near, to detect environment light intensity,to ensure indeed getting close
	{
		/*	ALS_DT1  :  MSB
		ALS_DT2  :  LSB	*/
		buf[1] = i2c_smbus_read_byte_data(stk3171_i2c_client, ALS_DT1);
		buf[0] = i2c_smbus_read_byte_data(stk3171_i2c_client, ALS_DT2);
		light_intensity = ((buf[1]<<8) | buf[0]);
		if (6*light_intensity > 20000){
			val = 1; //far
			distance = 1;
			printk("%s light_intensity=%d actually now state is far !!!\njust in the sun\n",__func__,6*light_intensity);
		}
	}
	
	if(alsps_dbg_flag){
		printk("%s  val = %d   %s\n",__func__,val, val ? "far":"near");
		printk("%s  ps=%d  low_thd=%d   high_thd=%d\n",__func__,ps,g_ps_low_threshold,g_ps_high_threshold);
	}
	if(atomic_read(&obj->ps_suspend))
	{
		invalid = 1;
	}
	else if(1 == atomic_read(&obj->ps_deb_on))
	{
		unsigned long endt = atomic_read(&obj->ps_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->ps_deb_on, 0);
		}
		
		if (1 == atomic_read(&obj->ps_deb_on))
		{
			ps_invalid_flags = 1;
			invalid = 1;
		}
	}

	if(!invalid)
	{
		if(unlikely(atomic_read(&obj->trace) & STK_TRC_CVT_PS))
		{
			if(mask)
			{
				APS_DBG("PS:  %05d => %05d [M] \n", ps, val);
			}
			else
			{
				APS_DBG("PS:  %05d => %05d\n", ps, val);
			}
		}
		return val;
		
	}	
	else
	{
		if(unlikely(atomic_read(&obj->trace) & STK_TRC_CVT_PS))
		{
			APS_DBG("PS:  %05d => %05d (-1)\n", ps, val);    
		}
		return -1;
	}	
}
/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static int stk3171_open(struct inode *inode, struct file *file)
{
	file->private_data = stk3171_i2c_client;

	if (!file->private_data)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}
	
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int stk3171_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}
/*----------------------------------------------------------------------------*/
static long stk3171_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct stk3171_priv *obj = i2c_get_clientdata(client);  
	long err = 0;
	void __user *ptr = (void __user*) arg;
	int dat;
	uint32_t enable;
	hwm_sensor_data sensor_data;

	struct alsps_hw *hw = NULL;
	struct set_ps_thd_para set_ps_thd_para;

	switch (cmd)
	{
		case ALSPS_SET_PS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
            down(&ps_suspend_sem);
            if(enable)
            {
                if((err = stk3171_enable_ps(obj->client, 1)))
                {
                    APS_ERR("enable ps fail: %d\n", err); 
                    up(&ps_suspend_sem);
                    goto err_out;
                }

                set_bit(STK_BIT_PS, &obj->enable);
            }
            else
            {
                if((err = stk3171_enable_ps(obj->client, 0)))
                {
                    APS_ERR("disable ps fail: %d\n", err); 
                    up(&ps_suspend_sem);
                    goto err_out;
                }

                clear_bit(STK_BIT_PS, &obj->enable);
            }
            up(&ps_suspend_sem);
			printk("%s  %s  PS  done !\n",__func__,enable == 0?"Disable":"Enable");
			break;

		case ALSPS_GET_PS_MODE:
			enable = test_bit(STK_BIT_PS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_DATA:
#ifndef VENDOR_EDIT
			if((err = stk3171_read_ps(obj->client, &obj->ps)))
			{
				goto err_out;
			}
#endif
			if ((ps_eint_mode == 0) && (distance == 1))
				dat = stk3171_get_ps_value(obj, obj->ps);
			else
				dat = g_is_far;
			
			if ((distance == 0) && (ps_eint_mode == 0)){
				stk3171_set_ps_mode(obj, 1); //change ps mode to eint
			}
#ifdef STK_PS_POLLING_LOG	
			printk("%s:ps value %d -> status %d \n",__func__, obj->ps, dat);
#endif			
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;

		case ALSPS_GET_PS_RAW_DATA:
#ifdef VENDOR_EDIT
		down(&ps_set_mode);
			if((err = stk3171_read_ps(obj->client, &obj->ps)))
			{
				goto err_out;
			}
		up(&ps_set_mode);
#endif
			dat = obj->ps;

			if (dat == 0)
				err = stk3171_dump_all_reg(obj->client);
			
			if(alsps_dbg_flag)
				printk("%s:ALSPS_GET_PS_RAW_DATA ps value %d\n",__func__, obj->ps);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;            

		case ALSPS_SET_ALS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			down(&ps_suspend_sem);
			if(enable)
			{
				if (suspend_status == 1){
					printk("%s open ALS suspend now, need resume\n",__func__);
					stk3171_power(stk3171_obj->hw, 1);
					if((err = stk3171_init_client(client)))
					{
						APS_ERR("initialize client fail!!\n");
						up(&ps_suspend_sem);
						return err;
					}

					atomic_set(&stk3171_obj->ps_suspend, 0);
					suspend_status = 0;
					atomic_set(&stk3171_obj->als_suspend, 0);
				}
				if((err = stk3171_enable_als(obj->client, 1)))
				{
					APS_ERR("enable als fail: %d\n", err); 
					up(&ps_suspend_sem);
					goto err_out;
				}
				set_bit(STK_BIT_ALS, &obj->enable);
			}
			else
			{
				if((err = stk3171_enable_als(obj->client, 0)))
				{
					APS_ERR("disable als fail: %d\n", err); 
					up(&ps_suspend_sem);
					goto err_out;
				}
				clear_bit(STK_BIT_ALS, &obj->enable);
			}
			up(&ps_suspend_sem);
			break;

		case ALSPS_GET_ALS_MODE:
			enable = test_bit(STK_BIT_ALS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_ALS_DATA: 
			if((err = stk3171_read_als(obj->client, &obj->als)))
			{
				goto err_out;
			}
		#if 0
			dat = stk3171_get_als_value(obj, obj->als);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}
			printk(APS_TAG" %s light intensity = %d\n",__func__,dat);
			break;
		#else
			temp_level = stk3171_get_als_value(obj, obj->als);
			if (temp_level != intensity_level)
			{
				intensity_level = temp_level;
				level_count = 0;
			} else level_count++;

			if (level_count >= 20)
			{
				level_count = 0;
				if(copy_to_user(ptr, &intensity_level, sizeof(intensity_level)))
				{
					err = -EFAULT;
					printk("Send data to uplayer fault !!!\n");
					goto err_out;
				}
				printk(APS_TAG" %s light intensity_level = %d\n",__func__,intensity_level);
			}else {
				if(copy_to_user(ptr, &prev_level, sizeof(prev_level)))
				{
					err = -EFAULT;
					printk("Send data to uplayer fault !!!\n");
					goto err_out;
				}
				printk(APS_TAG" %s light prev_level = %d\n",__func__,prev_level);
			}
			break;
		#endif
		case ALSPS_GET_ALS_RAW_DATA:
			if((err = stk3171_read_als(obj->client, &obj->als)))
			{
				goto err_out;
			}

			dat = obj->als;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;
/*******************************************************************/
#ifdef VENDOR_EDIT
	case ALSPS_SET_PS_THRESHOLD: //0x21 lycan add for 
		
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
			if(alsps_dbg_flag)
				printk("%s  psensord server set threshold, low_threshold=%d   high_threshold=%d\n",__func__,
					set_ps_thd_para.low_threshold, 
             			       set_ps_thd_para.high_threshold);
		
            stk3171_set_ps_threshold(obj->client,
                    set_ps_thd_para.low_threshold, 
                    set_ps_thd_para.high_threshold);
            break;

        case ALSPS_GET_CUST_PS_ADJUST_PARA: //0x22 lycan add for
            hw = get_cust_alsps_hw();
            if(copy_to_user(ptr, hw->p_ps_adjust_para, sizeof( struct ps_adjust_para)))
            {
                err = -EFAULT;
                goto err_out;
            }              
            break;
#endif
/*******************************************************************/
		default:
			APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
			err = -ENOIOCTLCMD;
			break;
	}

	err_out:
	return err;    
}
static int stk3171_fasync(int fd, struct file * file, int on)
{
    APS_LOG("%s on : %d\n", __func__, on);
    return fasync_helper(fd, file, on, &g_stk3171_ptr->fasync);
}
/*----------------------------------------------------------------------------*/
static struct file_operations stk3171_fops = {
	.owner = THIS_MODULE,
	.open = stk3171_open,
	.release = stk3171_release,
	.unlocked_ioctl = stk3171_unlocked_ioctl,
	.fasync = stk3171_fasync,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice stk3171_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &stk3171_fops,
};
/*----------------------------------------------------------------------------*/
static int stk3171_i2c_suspend(struct i2c_client *client, pm_message_t msg) 
{
	down(&ps_suspend_sem);
	int err;
	printk(APS_TAG"%s  Enter !!!\n",__func__);
	//if (ps_work_status == 1)
		//return 0;
	if(msg.event == PM_EVENT_SUSPEND)
	{   
		if(!stk3171_obj)
		{
			APS_ERR("null pointer!!\n");
			up(&ps_suspend_sem);
			return -EINVAL;
		}
		
		atomic_set(&stk3171_obj->als_suspend, 1);

		if (ps_work_status == 1)
		{
			up(&ps_suspend_sem);
			printk(APS_TAG"%s  PS is working  Exit !!!\n",__func__);
			return 0;
		}

		if(test_bit(STK_BIT_PS,  &stk3171_obj->enable)){
			if((err = stk3171_enable_als(client, 0)))
			{
				APS_ERR("disable als: %d\n", err);
				up(&ps_suspend_sem);
				return err;
			}
		}
		
		suspend_status = 1;
		atomic_set(&stk3171_obj->ps_suspend, 1);

		if((err = stk3171_enable_ps(client, 0)))
		{
			APS_ERR("disable ps:  %d\n", err);
			up(&ps_suspend_sem);
			return err;
		}
		
		stk3171_power(stk3171_obj->hw, 0);
	}
	up(&ps_suspend_sem);
	printk(APS_TAG"%s  Exit !!!\n",__func__);

	return 0;
}
/*----------------------------------------------------------------------------*/
static int stk3171_i2c_resume(struct i2c_client *client)
{
	down(&ps_suspend_sem);
	int err;
	printk("%s  Enter !!!\n",__func__);
	//if (ps_work_status == 1)
		//return 0;
	if(!stk3171_obj)
	{
		APS_ERR("null pointer!!\n");
		up(&ps_suspend_sem);
		return -EINVAL;
	}
	
	atomic_set(&stk3171_obj->als_suspend, 0);
		
	if ((ps_work_status == 1) || (suspend_status == 0)){
		printk("%s  PS is working ! not init\n",__func__);
		up(&ps_suspend_sem);
		return 0;
	}

		stk3171_power(stk3171_obj->hw, 1);
		if((err = stk3171_init_client(client)))
		{
			APS_ERR("initialize client fail!!\n");
			up(&ps_suspend_sem);
			return err;
		}

		/*clear interrupt when resume*/
		err = i2c_smbus_write_byte_data(client, ALSPS_STATUS, 0x0);
		if (err < 0)
		{
			printk(KERN_ERR"stk3171 resume clear interrupt fail !!!\n");
			up(&ps_suspend_sem);
			return err;
		}

		atomic_set(&stk3171_obj->ps_suspend, 0);
		if(test_bit(STK_BIT_PS,  &stk3171_obj->enable))
		{
			if((err = stk3171_enable_ps(client, 1)))
			{
				APS_ERR("enable ps fail: %d\n", err);
			}
		}

		suspend_status = 0;
	
	if(test_bit(STK_BIT_ALS, &stk3171_obj->enable))
	{
		if((err = stk3171_enable_als(client, 1)))
		{
			APS_ERR("enable als fail: %d\n", err);        
		}
	}

	up(&ps_suspend_sem);
	printk("%s  Leave !!!\n",__func__);
	return 0;
}
/*----------------------------------------------------------------------------*/
static void stk3171_early_suspend(struct early_suspend *h) 
{   /*early_suspend is only applied for ALS*/
	struct stk3171_priv *obj = container_of(h, struct stk3171_priv, early_drv);   
	int err;
#if 0
	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}
	
	atomic_set(&obj->als_suspend, 1);    
	if((err = stk3171_enable_als(obj->client, 0)))
	{
		APS_ERR("disable als fail: %d\n", err); 
	}
#else
	pm_message_t msg;
	msg.event = PM_EVENT_SUSPEND;
	err = stk3171_i2c_suspend(stk3171_obj->client, msg);
	//up(&ps_suspend_sem);
#endif
}
/*----------------------------------------------------------------------------*/
static void stk3171_late_resume(struct early_suspend *h)
{   /*early_suspend is only applied for ALS*/
	struct stk3171_priv *obj = container_of(h, struct stk3171_priv, early_drv);         
	int err;
#if 0
	hwm_sensor_data sensor_data;
	
	memset(&sensor_data, 0, sizeof(sensor_data));
	APS_FUN();

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}

	atomic_set(&obj->als_suspend, 0);
	if(test_bit(STK_BIT_ALS, &obj->enable))
	{
		if((err = stk3171_enable_als(obj->client, 1)))
		{
			APS_ERR("enable als fail: %d\n", err);        

		}
	}
#else
	err = stk3171_i2c_resume(stk3171_obj->client);
	//up(&ps_suspend_sem);
#endif
}

int stk3171_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* sensor_data;
	struct stk3171_priv *obj = (struct stk3171_priv *)self;
	//printk("%s  command=0x%x\n",__func__,command);
	//APS_FUN(f);
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
                down(&ps_suspend_sem);
                if(value)
                {
                        /*maybe sometimes need enable ps before resume, such as pull out headset*/
					if (suspend_status != 0)
					{
						printk("%s  ALSPS is suspend now, RESUME it !!!\n",__func__);
						stk3171_power(stk3171_obj->hw, 1);
						if((err = stk3171_init_client(obj->client)))
							APS_ERR("initialize client fail!!\n");

						/*clear interrupt when resume*/
						err = i2c_smbus_write_byte_data(obj->client, ALSPS_STATUS, 0x0);
						if (err < 0)
							printk(KERN_ERR"stk3171 resume clear interrupt fail !!!\n");

						if (err)
							printk("Need enable ps before resume, but now resume fail\n");
						else {
							suspend_status = 0;
							atomic_set(&stk3171_obj->ps_suspend, 0);
							atomic_set(&stk3171_obj->als_suspend, 0);
						}
					}
                    if((err = stk3171_enable_ps(obj->client, 1)))
                    {
                        APS_ERR("enable ps fail: %d\n", err); 
                        up(&ps_suspend_sem);
                        return -1;
                    }
                    set_bit(STK_BIT_PS, &obj->enable);
                }
                else
                {
                    if((err = stk3171_enable_ps(obj->client, 0)))
                    {
                        APS_ERR("disable ps fail: %d\n", err); 
                        up(&ps_suspend_sem);
                        return -1;
                    }
                    clear_bit(STK_BIT_PS, &obj->enable);
                }
                up(&ps_suspend_sem);
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;
				
				if((err = stk3171_read_ps(obj->client, &obj->ps)))
				{
					err = -1;
				}
				else
				{
					printk(APS_TAG"%s  getting ps data : obj->ps = %d\n",__func__, obj->ps);
					sensor_data->values[0] = stk3171_get_ps_value(obj, obj->ps);
					if (-1 == sensor_data->values[0])
						sensor_data->values[0] = distance;
					sensor_data->value_divide = 1;
					sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
#ifdef STK_PS_POLLING_LOG
					printk(APS_TAG"%s  FTM : ps value=%d -> state=%d \n",__func__, obj->ps, sensor_data->values[0]);					
#endif
				}				
			}
			break;
		default:
			APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}

int stk3171_als_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* sensor_data;
	struct stk3171_priv *obj = (struct stk3171_priv *)self;
	
	//APS_FUN(f);
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
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
					if((err = stk3171_enable_als(obj->client, 1)))
					{
						APS_ERR("enable als fail: %d\n", err); 
						return -1;
					}
					set_bit(STK_BIT_ALS, &obj->enable);
				}
				else
				{
					if((err = stk3171_enable_als(obj->client, 0)))
					{
						APS_ERR("disable als fail: %d\n", err); 
						return -1;
					}
					clear_bit(STK_BIT_ALS, &obj->enable);
				}
				
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;

				if((err = stk3171_read_als(obj->client, &obj->als)))
				{
					err = -1;
				}
				else
				{
				#if 0
					sensor_data->values[0] = stk3171_get_als_value(obj, obj->als);
					sensor_data->value_divide = 1;
					sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
				#else
					temp_level = stk3171_get_als_value(obj, obj->als);
					if (temp_level != intensity_level)
					{
						intensity_level = temp_level;
						level_count = 0;
					} else level_count++;

					if (level_count >= 20)
					{
						level_count = 0;
						printk(APS_TAG"  Light intensity = %d\n",intensity_level);
						prev_level = intensity_level;
						sensor_data->values[0] = intensity_level;
						sensor_data->value_divide = 1;
						sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
					} else {
						sensor_data->values[0] = prev_level;
						sensor_data->value_divide = 1;
						sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
					}
				#endif
				}
			}
			break;
		default:
			APS_ERR("light sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}

/**zhye**add**for**ps**algorithm**/
#ifdef VENDOR_EDIT

static void stk3171_set_ps_threshold(struct i2c_client *client, int low_threshold, int high_threshold)
{
	struct stk3171_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];
	long res = 0;
	u8 reg, chk[3] = {0};
	int i = 0;
	
	if (distance == 0){
		low_threshold = high_threshold - (0x01);
		if (obj->ps < high_threshold){
			printk("%s  Not set\n",__func__);
			return;
		}
	}
	
	g_ps_low_threshold = low_threshold;
	g_ps_high_threshold = high_threshold;

	databuf[0] = (u8)(low_threshold & 0x00FF);		//far_away
	databuf[1] = (u8)(high_threshold & 0x00FF);			//close
#ifndef VENDOR_EDIT
	res = stk3171_write_ps_high_thd(client, databuf[1]);
	res = stk3171_write_ps_low_thd(client, databuf[0]);
#else
	res = i2c_smbus_write_byte_data(client, STK31XX_CMM_POLL_LOW_THD, databuf[0]);
	res |= i2c_smbus_write_byte_data(client, STK31XX_CMM_POLL_HIGH_THD, databuf[1]);
#endif
	if (res < 0)
	{
		APS_ERR("i2c_write_thd  wrong!\n");
		goto EXIT_ERR;
	}

/*check reg 0x81 0x82 0x83 val are changed or not*/
#ifdef VENDOR_EDIT
	reg = 0x81;
	for (i = 0; i < 3; i++){
		chk[i] = i2c_smbus_read_byte_data(client, reg);
		printk("%s  read reg 0x%x=0x%x\n",__func__,reg, chk[i]);
		reg += 0x01;
	}

	if (strncmp(reg8,chk,3)){
		printk("%s  recv reg are changed, correct them\n",__func__);
		reg = 0x81;
		for (i = 0; i < 3; i++){
			res = i2c_smbus_write_byte_data(client, reg, reg8[i]);
			printk("%s  read reg 0x%x=0x%x\n",__func__,reg, chk[i]);
			reg += 0x01;
		}
	} else printk("%s  recv reg are right\n",__func__);
#endif/*VENDOR_EDIT*/
	
	return;
	
EXIT_ERR:
	APS_ERR("stk3171_enable_ps fail\n");
	return;
}
/**zhye**add**end**/
#endif/*VENDOR_EDIT*/
/*----------------------------------------------------------------------------*/
extern volatile ALSPS_DEV alsps_dev;
static int stk3171_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct stk3171_priv *obj;
	struct hwmsen_object obj_ps, obj_als;
	int err = 0;
	uint8_t als_reg,ps_reg;
	struct alsps_hw *hw_probe;
	hw_probe = get_cust_alsps_hw();
	printk("%s  Enter  !!!\n",__func__);
	
    //wake_lock_init(&ps_lock,WAKE_LOCK_SUSPEND,"ps wakelock");

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	printk("%s   kzalloc  OK !!!\n",__func__);
	//memset(obj, 0, sizeof(*obj));
	stk3171_obj = obj;
	obj->hw = get_cust_alsps_hw();
	stk3171_get_addr(obj->hw, &obj->addr);
	
#ifndef VENDOR_EDIT
	INIT_DELAYED_WORK(&stk3171_ps_work, stk3171_ps_adjust_delayed_work);
#endif
	INIT_WORK(&obj->eint_work, stk3171_eint_work);
	obj->client = client;
	i2c_set_clientdata(client, obj);	
	atomic_set(&obj->als_debounce, 1000);
	atomic_set(&obj->als_deb_on, 0);
	atomic_set(&obj->als_deb_end, 0);
	atomic_set(&obj->ps_debounce, 100);
	atomic_set(&obj->ps_deb_on, 0);
	atomic_set(&obj->ps_deb_end, 0);
	atomic_set(&obj->ps_mask, 0);
	atomic_set(&obj->trace, 0x00);
	atomic_set(&obj->als_suspend, 0);
#if 0
	atomic_set(&obj->als_cmd_val, obj->hw->als_cmd_val);
	atomic_set(&obj->ps_cmd_val, obj->hw->ps_cmd_val);
	gain_setting = obj->hw->ps_gain_setting;
	
	atomic_set(&obj->ps_high_thd_val, obj->hw->ps_high_thd_val ); 
	atomic_set(&obj->ps_low_thd_val, obj->hw->ps_low_thd_val ); 
#else
	atomic_set(&obj->als_cmd_val, 0x49);
	atomic_set(&obj->ps_cmd_val, 0x61);
	gain_setting = 0x01;
	
	atomic_set(&obj->ps_high_thd_val, 255 ); 
	atomic_set(&obj->ps_low_thd_val, 0 ); 
#endif
	atomic_set(&obj->recv_reg, 0);  
#if 0
	if(obj->hw->polling_mode_ps == 0)
	{
	  //atomic_set(&obj->ps_cmd_val,  (obj->hw->ps_cmd_val) | 0x02);
	  atomic_set(&obj->ps_cmd_val,  0x61 | INT_PS);
	  printk("enable PS interrupt\n");
	}
#endif
	if(obj->hw->polling_mode_als == 0)
	{
	  //atomic_set(&obj->als_cmd_val,  (obj->hw->als_cmd_val) | 0x02);
	  atomic_set(&obj->als_cmd_val,  0x49 | INT_ALS);
	  printk("enable ALS interrupt\n");
	}	
	printk("ps_cmd_val=0x%x\n", obj->ps_cmd_val);
	printk("als_cmd_val=0x%x\n", obj->als_cmd_val);
	
	obj->enable = 0;
	obj->pending_intr = 0;
	obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
	obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);   
	BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
	memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
	BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
	memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
	atomic_set(&obj->i2c_retry, 1);
	if(!(atomic_read(&obj->als_cmd_val) & SD_ALS))
	{
		set_bit(STK_BIT_ALS, &obj->enable);
	}
	
	if(!(atomic_read(&obj->ps_cmd_val) & SD_PS))
	{
		set_bit(STK_BIT_PS, &obj->enable);
	}
	
	stk3171_i2c_client = client;

	
	if((err = stk3171_init_client(client)))
	{
		goto exit_init_failed;
	}
	
	if((err = misc_register(&stk3171_device)))
	{
		APS_ERR("stk3171_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	if((err = stk3171_create_attr(&stk3171_alsps_driver.driver)))
	{
		APS_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
	obj_ps.self = stk3171_obj;
	if(1 == obj->hw->polling_mode_ps)
	{
		printk("%s  polling  mode \n",__func__);
	  obj_ps.polling = 1;
	}
	else
	{
	  obj_ps.polling = 0;//interrupt mode
	}
	obj_ps.sensor_operate = stk3171_ps_operate;
	if((err = hwmsen_attach(ID_PROXIMITY, &obj_ps)))
	{
		APS_ERR("attach fail = %d\n", err);
		goto exit_create_attr_failed;
	}
	
	obj_als.self = stk3171_obj;
	if(1 == obj->hw->polling_mode_als)
	{
	  obj_als.polling = 1;
	}
	else
	{
	  obj_als.polling = 0;//interrupt mode
	}
	obj_als.sensor_operate = stk3171_als_operate;
	if((err = hwmsen_attach(ID_LIGHT, &obj_als)))
	{
		APS_ERR("attach fail = %d\n", err);
		goto exit_create_attr_failed;
	}

	als_reg = i2c_smbus_read_byte_data(client, 0x01);
	ps_reg = i2c_smbus_read_byte_data(client, 0x09);
	printk("%s  als_reg = 0x%x   ps_reg = 0x%x \n",__func__, als_reg,ps_reg);

#if 1
#if defined(CONFIG_HAS_EARLYSUSPEND)
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1;
	obj->early_drv.suspend  = stk3171_early_suspend;
	obj->early_drv.resume   = stk3171_late_resume;
	register_early_suspend(&obj->early_drv);
#endif
#endif

#ifdef VENDOR_EDIT /*zhye add a interface to control register*/
	//wake_lock_init(&ps_lock, WAKE_LOCK_SUSPEND, "ps_wakelock");
	hwmsen_make_debug_flag(&open_alsps_log, "open_alsps_log");
	hwmsen_make_debug_flag(&set_prox_reg, "set_prox_reg");
	hwmsen_make_debug_flag(&read_prox_reg, "read_prox_reg");
	hwmsen_make_debug_flag(&ps_switch_fops,"ps_switch");
	//hwmsen_make_debug_flag(&set_prox_thd, "set_prox_thd");
#endif/*VENDOR_EDIT*/
	alsps_dev = ALSPS_STK31XX;
	printk("%s: OK\n", __func__);
	return 0;

	exit_create_attr_failed:
	misc_deregister(&stk3171_device);
	exit_misc_device_register_failed:
	exit_init_failed:
	//i2c_detach_client(client);
//	exit_kfree:
	kfree(obj);
	exit:
	stk3171_i2c_client = NULL;
	APS_ERR("%s: err = %d\n", __func__, err);
	stk3171_power(hw_probe, 0);
	return err;
}
/*----------------------------------------------------------------------------*/
static int stk3171_i2c_remove(struct i2c_client *client)
{
	int err;	
	
	if((err = stk3171_delete_attr(&stk3171_i2c_driver.driver)))
	{
		APS_ERR("stk3171_delete_attr fail: %d\n", err);
	} 

	if((err = misc_deregister(&stk3171_device)))
	{
		APS_ERR("misc_deregister fail: %d\n", err);    
	}
	
	stk3171_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	return 0;
}
/*----------------------------------------------------------------------------*/
static int stk3171_probe(struct platform_device *pdev) 
{
	struct alsps_hw *hw = get_cust_alsps_hw();
	struct stk3171_i2c_addr addr;

	stk3171_power(hw, 1);    
	stk3171_get_addr(hw, &addr);
	//stk3171_force[0] = hw->i2c_num;
	//stk3171_force[1] = hw->i2c_addr[0];
	if(i2c_add_driver(&stk3171_i2c_driver))
	{
		APS_ERR("add driver error\n");
		return -1;
	} 
	return 0;
}
/*----------------------------------------------------------------------------*/
static int stk3171_remove(struct platform_device *pdev)
{
	struct alsps_hw *hw = get_cust_alsps_hw();
	APS_FUN();    
	stk3171_power(hw, 0);    
	i2c_del_driver(&stk3171_i2c_driver);
	return 0;
}
/*----------------------------------------------------------------------------*/
static struct platform_driver stk3171_alsps_driver = {
	.probe      = stk3171_probe,
	.remove     = stk3171_remove,    
	.driver     = {
		.name  = "als_ps_stk3171",
//		.owner = THIS_MODULE,
	}
};
/*----------------------------------------------------------------------------*/
static int __init stk3171_init(void)
{
	APS_FUN();
	i2c_register_board_info(4, &i2c_stk3171, 1);
	if(platform_driver_register(&stk3171_alsps_driver))
	{
		APS_ERR("failed to register driver");
		return -ENODEV;
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit stk3171_exit(void)
{
	APS_FUN();
	platform_driver_unregister(&stk3171_alsps_driver);
}
/*----------------------------------------------------------------------------*/
module_init(stk3171_init);
module_exit(stk3171_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("MingHsien Hsieh");
MODULE_DESCRIPTION("stk3171 proximity and light sensor driver");
MODULE_LICENSE("GPL");
