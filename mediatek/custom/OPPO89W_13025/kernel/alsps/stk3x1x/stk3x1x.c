/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/* drivers/hwmon/mt6516/amit/stk3x1x.c - stk3x1x ALS/PS driver
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
#include "stk3x1x.h"
#define DRIVER_VERSION          "3.1.2"
//#define STK_PS_POLLING_LOG
#define STK_TUNE0
//#define STK_FIR
//#define STK_IRS

#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <linux/semaphore.h>/*OPPO 2013-01-21 zhye add for sync suspend and resume*/
#include <linux/wakelock.h>

#include <linux/ctype.h>
#include <linux/kernel.h>

#if ((defined MT6573) || (defined MT6575) || (defined MT6577) || (defined MT6589)  || (defined MT6572))	
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_polarity(kal_uint8 eintno, kal_bool ACT_Polarity);
extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
                                     kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
                                     kal_bool auto_umask);

#endif

/*-------------------------MT6516&MT6573 define-------------------------------*/

#if ((defined MT6573) || (defined MT6575) || (defined MT6577) || (defined MT6589) || (defined MT6572))	
#define POWER_NONE_MACRO MT65XX_POWER_NONE
#endif
#define GPIO_ALS_EINT_PIN         GPIO38
#define CUST_EINT_ALS_NUM              4

/******************************************************************************
 * configuration
*******************************************************************************/

/*----------------------------------------------------------------------------*/
#define STK3X1X_DEV_NAME     "stk3x1x"
/*----------------------------------------------------------------------------*/
#define APS_TAG                  "[ALS/PS] "
#define APS_FUN(f)               printk(KERN_INFO APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    printk(KERN_ERR  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    printk(KERN_INFO APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    printk(KERN_INFO fmt, ##args)                 
/******************************************************************************
 * extern functions
*******************************************************************************/
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
#define STK2213_PID			0x23
#define STK2213I_PID			0x22
#define STK3010_PID			0x33
#define STK3210_STK3310_PID	0x13
#define STK3211_STK3311_PID	0x12

#ifdef STK_TUNE0
	#define STK_MAX_MIN_DIFF	200
	#define STK_LT_N_CT	100
	#define STK_HT_N_CT	150
#endif /* #ifdef STK_TUNE0 */

#define STK_IRC_MAX_ALS_CODE		20000
#define STK_IRC_MIN_ALS_CODE		25
#define STK_IRC_MIN_IR_CODE		50
#define STK_IRC_ALS_DENOMI		2		
#define STK_IRC_ALS_NUMERA		5
#define STK_IRC_ALS_CORREC		748

/*----------------------------------------------------------------------------*/
static struct i2c_client *stk3x1x_i2c_client = NULL;
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id stk3x1x_i2c_id[] = {{STK3X1X_DEV_NAME,0},{}};

static struct i2c_board_info __initdata i2c_stk3x1x={ I2C_BOARD_INFO(STK3X1X_DEV_NAME, (0x90>>1))};

/*----------------------------------------------------------------------------*/
static int stk3x1x_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int stk3x1x_i2c_remove(struct i2c_client *client);

/*----------------------------------------------------------------------------*/
static int stk3x1x_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int stk3x1x_i2c_resume(struct i2c_client *client);
static struct stk3x1x_priv *g_stk3x1x_ptr = NULL;
#ifdef VENDOR_EDIT
//Fuchun.Liao@wxkfDrv,2013/05/31,add for solve compile error
int whether_in_call_state=0;
int whether_in_approach_state=0;
int L3G4200D_OpenGyro_preview(void)
{
    return 0;
}
int L3G4200D_CloseGyro_preview(void)
{
   return 0;
}
int L3G4200D_ReadGyroData_preview(char *buf)
{
   return 0;
}
#endif 

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
struct stk3x1x_i2c_addr {    
/*define a series of i2c slave address*/
    u8  state;      	/* enable/disable state */
    u8  psctrl;     	/* PS control */
    u8  alsctrl;    	/* ALS control */
    u8  ledctrl;   		/* LED control */
    u8  intmode;    	/* INT mode */
    u8  wait;     		/* wait time */
    u8  thdh1_ps;   	/* PS INT threshold high 1 */
	u8	thdh2_ps;		/* PS INT threshold high 2 */
    u8  thdl1_ps;   	/* PS INT threshold low 1 */
	u8  thdl2_ps;   	/* PS INT threshold low 2 */
    u8  thdh1_als;   	/* ALS INT threshold high 1 */
	u8	thdh2_als;		/* ALS INT threshold high 2 */
    u8  thdl1_als;   	/* ALS INT threshold low 1 */
	u8  thdl2_als;   	/* ALS INT threshold low 2 */	
	u8  flag;			/* int flag */
	u8  data1_ps;		/* ps data1 */
	u8  data2_ps;		/* ps data2 */
	u8  data1_als;		/* als data1 */
	u8  data2_als;		/* als data2 */
	u8  data1_offset;	/* offset data1 */
	u8  data2_offset;	/* offset data2 */
	u8  data1_ir;		/* ir data1 */
	u8  data2_ir;		/* ir data2 */
	u8  soft_reset;		/* software reset */
};
/*----------------------------------------------------------------------------*/
#ifdef STK_FIR
struct data_filter {
    s16 raw[8];
    int sum;
    int num;
    int idx;
};
#endif

struct stk3x1x_priv {
    struct alsps_hw  *hw;
    struct i2c_client *client;
    struct delayed_work  eint_work;

    /*i2c address group*/
    struct stk3x1x_i2c_addr  addr;

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
    u16         ps;
    u8          _align;
    u16         als_level_num;
    u16         als_value_num;
    u32         als_level[C_CUST_ALS_LEVEL-1];
    u32         als_value[C_CUST_ALS_LEVEL];

	atomic_t	state_val;
	atomic_t 	psctrl_val;
	atomic_t 	alsctrl_val;
	u8 			wait_val;
	u8		 	ledctrl_val;
	u8		 	int_val;
	
    atomic_t    ps_high_thd_val;     /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_low_thd_val;     /*the cmd value can't be read, stored in ram*/
    ulong       enable;         /*enable mask*/
    ulong       pending_intr;   /*pending interrupt*/
	atomic_t	recv_reg;
    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend    early_drv;
#endif     
	bool first_boot;
#ifdef STK_TUNE0
	uint16_t psa;
	uint16_t psi;	
	uint16_t psi_set;	
	struct hrtimer ps_tune0_timer;	
	struct workqueue_struct *stk_ps_tune0_wq;
	struct work_struct stk_ps_tune0_work;
	ktime_t ps_tune0_delay;	
	bool tune_zero_init_proc;
	uint32_t ps_stat_data[3];
	int data_count;	
#endif	
#ifdef STK_FIR
	struct data_filter      fir;
#endif
	uint16_t ir_code;
	uint16_t als_correct_factor;	

	struct fasync_struct *fasync;
};
/*----------------------------------------------------------------------------*/
static struct i2c_driver stk3x1x_i2c_driver = {	
	.probe      = stk3x1x_i2c_probe,
	.remove     = stk3x1x_i2c_remove,
#if !defined(CONFIG_HAS_EARLYSUSPEND)
	.suspend    = stk3x1x_i2c_suspend,
	.resume     = stk3x1x_i2c_resume,
#endif
	.id_table   = stk3x1x_i2c_id,
	.driver = {
		.name           = STK3X1X_DEV_NAME,
	},
};
static struct alsps_hw *get_cust_alsps_hw(void) 
{
    return get_cust_alsps_hw_stk3x1x();
}

static unsigned int g_is_far = 1; /*init this flag with 1, means FAR*/
static unsigned int g_in_interrupt = 0; // if in interrupt , psensord can not read ps 
static unsigned int 	g_ps_low_threshold = 0;
static unsigned int 	g_ps_high_threshold = 255;
static unsigned int g_ps_switch_off = 0;
static unsigned int suspend_status = 0;
static unsigned int g_first_thd = 0;
static DEFINE_SEMAPHORE(ps_suspend_sem);

static struct stk3x1x_priv *stk3x1x_obj = NULL;
static struct platform_driver stk3x1x_alsps_driver;
static int stk3x1x_get_ps_value(struct stk3x1x_priv *obj, u16 ps);
static int stk3x1x_get_ps_value_only(struct stk3x1x_priv *obj, u16 ps);
static int stk3x1x_get_als_value(struct stk3x1x_priv *obj, u16 als);
static int stk3x1x_read_als(struct i2c_client *client, u16 *data);
static int stk3x1x_read_ps(struct i2c_client *client, u16 *data);
static int stk3x1x_set_als_int_thd(struct i2c_client *client, u16 als_data_reg);
static int32_t stk3x1x_get_ir_value(struct stk3x1x_priv *obj);
#ifdef STK_TUNE0
static int stk_ps_tune_zero_func_fae(struct stk3x1x_priv *obj);
#endif
struct wake_lock ps_lock;
/*------------------------------------------------------------------------------*/
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
    ret = i2c_smbus_write_byte_data(stk3x1x_i2c_client, reg, val);
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
    for (i = 0x00; i <= 0x18; i += 0x01){
        //i2c_master_send(stk3x1x_i2c_client, &i, 1);
        //i2c_master_recv(stk3x1x_i2c_client, &val, 1);
        val = i2c_smbus_read_byte_data(stk3x1x_i2c_client, i);
        p += sprintf(p, "Reg 0x%x:0x%x\n", i, val);	
    }

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
    val = i2c_smbus_read_byte_data(stk3x1x_i2c_client, reg);
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

    printk("stk3x1x  show_ps_switch ,  g_ps_switch_off:%d \n", g_ps_switch_off);
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
    APS_LOG("stk3x1x  %s : %s\n", __func__, tmp);

    sscanf(tmp, "%d", &val);
    APS_LOG("stk3x1x set_ps_switch %d\n", val);

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
/**zhye**add**for**ps**algorithm**/
#ifdef VENDOR_EDIT

static void stk3x1x_set_ps_threshold(struct i2c_client *client, int low_threshold, int high_threshold)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);
	long ret = 0;
	u16 byte_low,byte_high;
	u16 h_thd, L_thd;
	g_ps_low_threshold = low_threshold;
	g_ps_high_threshold = high_threshold;
	
	byte_high = (high_threshold & 0xFF00)>>8;
	byte_low = (high_threshold & 0x00FF)<<8;
	h_thd = byte_low | byte_high;
	printk("%s  high_threshold  byte_low=0x%4x  byte_high = 0x%4x\n",__func__, byte_low, byte_high);
	ret = i2c_smbus_write_word_data(client, STK_THDH1_PS_REG, h_thd);

	byte_high = (low_threshold & 0xFF00)>>8;
	byte_low = (low_threshold & 0x00FF)<<8;
	L_thd = byte_low | byte_high;
	printk("%s  low_threshold  byte_low=0x%4x  byte_high = 0x%4x\n",__func__, byte_low, byte_high);
	ret |= i2c_smbus_write_word_data(client, STK_THDL1_PS_REG, L_thd);
	
	if (ret < 0)
	{
		APS_ERR("i2c_write_thd  wrong!\n");
		goto EXIT_ERR;
	}

	return;
	
EXIT_ERR:
	APS_ERR("stk3x1x_enable_ps fail\n");
	return;
}
/**zhye**add**end**/
#endif/*VENDOR_EDIT*/
/*----------------------------------------------------------------------------*/
int stk3x1x_get_addr(struct alsps_hw *hw, struct stk3x1x_i2c_addr *addr)
{
	if(!hw || !addr)
	{
		return -EFAULT;
	}
	addr->state   = STK_STATE_REG; 
	addr->psctrl   = STK_PSCTRL_REG;         
	addr->alsctrl  = STK_ALSCTRL_REG;
	addr->ledctrl  = STK_LEDCTRL_REG;
	addr->intmode    = STK_INT_REG;
	addr->wait    = STK_WAIT_REG;
	addr->thdh1_ps    = STK_THDH1_PS_REG;
	addr->thdh2_ps    = STK_THDH2_PS_REG;
	addr->thdl1_ps = STK_THDL1_PS_REG;
	addr->thdl2_ps = STK_THDL2_PS_REG;
	addr->thdh1_als    = STK_THDH1_ALS_REG;
	addr->thdh2_als    = STK_THDH2_ALS_REG;
	addr->thdl1_als = STK_THDL1_ALS_REG ;
	addr->thdl2_als = STK_THDL2_ALS_REG;
	addr->flag = STK_FLAG_REG;	
	addr->data1_ps = STK_DATA1_PS_REG;
	addr->data2_ps = STK_DATA2_PS_REG;
	addr->data1_als = STK_DATA1_ALS_REG;	
	addr->data2_als = STK_DATA2_ALS_REG;	
	addr->data1_offset = STK_DATA1_OFFSET_REG;
	addr->data2_offset = STK_DATA2_OFFSET_REG;
	addr->data1_ir = STK_DATA1_IR_REG;	
	addr->data2_ir = STK_DATA2_IR_REG;		
	addr->soft_reset = STK_SW_RESET_REG;	
	
	return 0;
}
/*----------------------------------------------------------------------------*/
int stk3x1x_hwmsen_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	u8 beg = addr; 
	struct i2c_msg msgs[2] = 
	{
		{
			.addr = client->addr,	 
			.flags = 0,
			.len = 1,				 
			.buf= &beg
		},
		{
			.addr = client->addr,	 
			.flags = I2C_M_RD,
			.len = len, 			 
			.buf = data,
		}
	};
	int err;

	if (!client)
		return -EINVAL;
	else if (len > C_I2C_FIFO_SIZE) 
	{		 
		APS_LOG(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}

	err = i2c_transfer(client->adapter, msgs, sizeof(msgs)/sizeof(msgs[0]));
	if (err != 2) 
	{
		APS_LOG("i2c_transfer error: (%d %p %d) %d\n", addr, data, len, err);
		err = -EIO;
	}
	else 
	{
		err = 0;/*no error*/
	}
	return err;
}
/*----------------------------------------------------------------------------*/
int stk3x1x_get_timing(void)
{
	return 200;
/*
	u32 base = I2C2_BASE; 
	return (__raw_readw(mt6516_I2C_HS) << 16) | (__raw_readw(mt6516_I2C_TIMING));
*/
}

/*----------------------------------------------------------------------------*/
int stk3x1x_master_recv(struct i2c_client *client, u16 addr, u8 *buf ,int count)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);        
	int ret = 0, retry = 0;
	int trc = atomic_read(&obj->trace);
	int max_try = atomic_read(&obj->i2c_retry);

	while(retry++ < max_try)
	{
		ret = stk3x1x_hwmsen_read_block(client, addr, buf, count);
		if(ret == 0)
            break;
		udelay(100);
	}

	if(unlikely(trc))
	{
		if((retry != 1) && (trc & STK_TRC_DEBUG))
		{
			APS_LOG("(recv) %d/%d\n", retry-1, max_try); 

		}
	}

	/* If everything went ok (i.e. 1 msg transmitted), return #bytes
	transmitted, else error code. */
	return (ret == 0) ? count : ret;
}
/*----------------------------------------------------------------------------*/
int stk3x1x_master_send(struct i2c_client *client, u16 addr, u8 *buf ,int count)
{
	int ret = 0, retry = 0;
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);        
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
		if((retry != 1) && (trc & STK_TRC_DEBUG))
		{
			APS_LOG("(send) %d/%d\n", retry-1, max_try);
		}
	}
	/* If everything went ok (i.e. 1 msg transmitted), return #bytes
	transmitted, else error code. */
	return (ret == 0) ? count : ret;
}
/*----------------------------------------------------------------------------*/
int stk3x1x_write_led(struct i2c_client *client, u8 data)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);
	int ret = 0;
    
	ret = stk3x1x_master_send(client, obj->addr.ledctrl, &data, 1);
	if(ret < 0)
	{
		APS_ERR("write led = %d\n", ret);
		return -EFAULT;
	}
	
	return 0;    
}
/*----------------------------------------------------------------------------*/
int stk3x1x_read_als(struct i2c_client *client, u16 *data)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);    
	int ret = 0;
	u8 buf[2];
	int32_t als_comperator;	
	u16 als_data;
#ifdef STK_FIR
	int idx;   
#endif
	if(NULL == client)
	{
		return -EINVAL;
	}	
	ret = stk3x1x_master_recv(client, obj->addr.data1_als, buf, 0x02);
	if(ret < 0)
	{
		APS_DBG("error: %d\n", ret);
		return -EFAULT;
	}
	else
	{
		als_data = (buf[0] << 8) | (buf[1]);
#ifdef STK_FIR
		if(obj->fir.num < 8)
		{                
			obj->fir.raw[obj->fir.num] = als_data;
			obj->fir.sum += als_data;
			obj->fir.num++;
			obj->fir.idx++;
		}
		else
		{
			idx = obj->fir.idx % 8;
			obj->fir.sum -= obj->fir.raw[idx];
			obj->fir.raw[idx] = als_data;
			obj->fir.sum += als_data;
			obj->fir.idx++;
			als_data = obj->fir.sum/8;
		}	
#endif
	}
	
	if(obj->ir_code)
	{
		obj->als_correct_factor = 1000;
		if(als_data < STK_IRC_MAX_ALS_CODE && als_data > STK_IRC_MIN_ALS_CODE && 
			obj->ir_code > STK_IRC_MIN_IR_CODE)
		{
			als_comperator = als_data * STK_IRC_ALS_NUMERA / STK_IRC_ALS_DENOMI;
			if(obj->ir_code > als_comperator)
				obj->als_correct_factor = STK_IRC_ALS_CORREC;
		}
		APS_LOG("%s: als=%d, ir=%d, als_correct_factor=%d", __func__, als_data, obj->ir_code, obj->als_correct_factor);
		obj->ir_code = 0;
	}	
	*data = als_data * obj->als_correct_factor / 100;
	
	if(atomic_read(&obj->trace) & STK_TRC_ALS_DATA)
	{
		APS_DBG("ALS: 0x%04X\n", (u32)(*data));
	}
	
	return 0;    
}
/*----------------------------------------------------------------------------*/
int stk3x1x_write_als(struct i2c_client *client, u8 data)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);
	int ret = 0;
    
	ret = stk3x1x_master_send(client, obj->addr.alsctrl, &data, 1);
	if(ret < 0)
	{
		APS_ERR("write als = %d\n", ret);
		return -EFAULT;
	}
	
	return 0;    
}
/*----------------------------------------------------------------------------*/
int stk3x1x_read_flag(struct i2c_client *client, u8 *data)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);    
	int ret = 0;
	u8 buf;
	
	if(NULL == client)
	{
		return -EINVAL;
	}	
	ret = stk3x1x_master_recv(client, obj->addr.flag, &buf, 0x01);
	if(ret < 0)
	{
		APS_DBG("error: %d\n", ret);
		return -EFAULT;
	}
	else
	{
		*data = buf;
	}
	
	if(atomic_read(&obj->trace) & STK_TRC_ALS_DATA)
	{
		APS_DBG("PS NF flag: 0x%04X\n", (u32)(*data));
	}
	
	return 0;    
}
/*----------------------------------------------------------------------------*/
int stk3x1x_read_id(struct i2c_client *client)
{
	int ret = 0;
	u8 buf[2];
	
	if(NULL == client)
	{
		return -EINVAL;
	}	
	ret = stk3x1x_master_recv(client, STK_PDT_ID_REG, buf, 0x02);
	if(ret < 0)
	{
		APS_DBG("error: %d\n", ret);
		return -EFAULT;
	}
	APS_LOG("%s: PID=0x%d, VID=0x%x\n", __func__, buf[0], buf[1]);
	
	if(buf[1] == 0xC0)
		APS_LOG( "%s: RID=0xC0!!!!!!!!!!!!!\n", __func__);		
	
	switch(buf[0])
	{
		case STK2213_PID:
		case STK2213I_PID:
		case STK3010_PID:
		case STK3210_STK3310_PID:
		case STK3211_STK3311_PID:
			return 0;
		case 0x0:
			APS_ERR("PID=0x0, please make sure the chip is stk3x1x!\n");
			return -2;
		default:
			APS_ERR( "%s: invalid PID(%#x)\n", __func__, buf[0]);	
			return -1;
	}	
	return 0;    
}
/*----------------------------------------------------------------------------*/
int stk3x1x_read_ps(struct i2c_client *client, u16 *data)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);    
	int ret = 0;
	u8 buf[2];
	
	if(NULL == client)
	{
		APS_ERR("i2c client is NULL\n");
		return -EINVAL;
	}	
	ret = stk3x1x_master_recv(client, obj->addr.data1_ps, buf, 0x02);
	if(ret < 0)
	{
		APS_DBG("error: %d\n", ret);
		return -EFAULT;
	}
	else
	{
		*data = (buf[0] << 8) | (buf[1]);
	}
	printk("%s  ps = %d\n",__func__, *data);
	if(atomic_read(&obj->trace) & STK_TRC_ALS_DATA)
	{
		APS_DBG("PS: 0x%04X\n", (u32)(*data));
	}
	
	return 0;     
}
/*----------------------------------------------------------------------------*/
int stk3x1x_write_ps(struct i2c_client *client, u8 data)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);        
	int ret = 0;

	ret = stk3x1x_master_send(client, obj->addr.psctrl, &data, 1);
	if (ret < 0)
	{
		APS_ERR("write ps = %d\n", ret);
		return -EFAULT;
	} 
	return 0;    
}

/*----------------------------------------------------------------------------*/
int stk3x1x_write_wait(struct i2c_client *client, u8 data)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);        
	int ret = 0;

	ret = stk3x1x_master_send(client, obj->addr.wait, &data, 1);
	if (ret < 0)
	{
		APS_ERR("write wait = %d\n", ret);
		return -EFAULT;
	} 
	return 0;    
}

/*----------------------------------------------------------------------------*/
int stk3x1x_write_int(struct i2c_client *client, u8 data)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);        
	int ret = 0;

	ret = stk3x1x_master_send(client, obj->addr.intmode, &data, 1);
	if (ret < 0)
	{
		APS_ERR("write intmode = %d\n", ret);
		return -EFAULT;
	} 
	return 0;    
}

/*----------------------------------------------------------------------------*/
int stk3x1x_write_state(struct i2c_client *client, u8 data)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);        
	int ret = 0;

	ret = stk3x1x_master_send(client, obj->addr.state, &data, 1);
	if (ret < 0)
	{
		APS_ERR("write state = %d\n", ret);
		return -EFAULT;
	} 
	return 0;    
}
/*----------------------------------------------------------------------------*/
int stk3x1x_write_flag(struct i2c_client *client, u8 data)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);        
	int ret = 0;

	ret = stk3x1x_master_send(client, obj->addr.flag, &data, 1);
	if (ret < 0)
	{
		APS_ERR("write ps = %d\n", ret);
		return -EFAULT;
	} 
	return 0;    
}
/*----------------------------------------------------------------------------*/
int stk3x1x_write_sw_reset(struct i2c_client *client)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);        
	u8 buf = 0, r_buf = 0;	
	int ret = 0;

	buf = 0x7F;
	ret = stk3x1x_master_send(client, obj->addr.wait, (char*)&buf, sizeof(buf));
	if (ret < 0)
	{
		APS_ERR("i2c write test error = %d\n", ret);
		return -EFAULT;
	} 	
	
	ret = stk3x1x_master_recv(client, obj->addr.wait, &r_buf, 1);
	if (ret < 0)
	{
		APS_ERR("i2c read test error = %d\n", ret);
		return -EFAULT;
	}	
	
	if(buf != r_buf)
	{
        APS_ERR("i2c r/w test error, read-back value is not the same, write=0x%x, read=0x%x\n", buf, r_buf);		
		return -EIO;
	}
	
	buf = 0;
	ret = stk3x1x_master_send(client, obj->addr.soft_reset, (char*)&buf, sizeof(buf));
	if (ret < 0)
	{
		APS_ERR("write software reset error = %d\n", ret);
		return -EFAULT;
	} 
	msleep(1);
	return 0;    
}

/*----------------------------------------------------------------------------*/
int stk3x1x_write_ps_high_thd(struct i2c_client *client, u16 thd)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);        
	u8 buf[2];
	int ret = 0;
	
	buf[0] = (u8) ((0xFF00 & thd) >> 8);
	buf[1] = (u8) (0x00FF & thd);	
	ret = stk3x1x_master_send(client, obj->addr.thdh1_ps, &buf[0], 1);
	if (ret < 0)
	{
		APS_ERR("WARNING: %d\n",  ret);
		return -EFAULT;
	}
	
	ret = stk3x1x_master_send(client, obj->addr.thdh2_ps, &(buf[1]), 1);
	if (ret < 0)
	{
		APS_ERR("WARNING: %d\n", ret);
		return -EFAULT;
	}	
	
	return 0;    
}
/*----------------------------------------------------------------------------*/
int stk3x1x_write_ps_low_thd(struct i2c_client *client, u16 thd)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);        
	u8 buf[2];
	int ret = 0;
	
	buf[0] = (u8) ((0xFF00 & thd) >> 8);
	buf[1] = (u8) (0x00FF & thd);	
	ret = stk3x1x_master_send(client, obj->addr.thdl1_ps, &buf[0], 1);
	if (ret < 0)
	{
		APS_ERR("WARNING: %s: %d\n", __func__, ret);
		return -EFAULT;
	}
	
	ret = stk3x1x_master_send(client, obj->addr.thdl2_ps, &(buf[1]), 1);
	if (ret < 0)
	{
		APS_ERR("WARNING: %s: %d\n", __func__, ret);
		return -EFAULT;
	}	
	
	return 0;    
}
/*----------------------------------------------------------------------------*/
int stk3x1x_write_als_high_thd(struct i2c_client *client, u16 thd)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);        
	u8 buf[2];
	int ret = 0;
	
	buf[0] = (u8) ((0xFF00 & thd) >> 8);
	buf[1] = (u8) (0x00FF & thd);	
	ret = stk3x1x_master_send(client, obj->addr.thdh1_als, &buf[0], 1);
	if (ret < 0)
	{
		APS_ERR("WARNING: %s: %d\n", __func__, ret);
		return -EFAULT;
	}
	
	ret = stk3x1x_master_send(client, obj->addr.thdh2_als, &(buf[1]), 1);
	if (ret < 0)
	{
		APS_ERR("WARNING: %s: %d\n", __func__, ret);
		return -EFAULT;
	}	
	
	return 0;    
}
/*----------------------------------------------------------------------------*/
int stk3x1x_write_als_low_thd(struct i2c_client *client, u16 thd)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);        
	u8 buf[2];
	int ret = 0;
	
	buf[0] = (u8) ((0xFF00 & thd) >> 8);
	buf[1] = (u8) (0x00FF & thd);	
	ret = stk3x1x_master_send(client, obj->addr.thdl1_als, &buf[0], 1);
	if (ret < 0)
	{
		APS_ERR("WARNING: %s: %d\n", __func__, ret);
		return -EFAULT;
	}
	
	ret = stk3x1x_master_send(client, obj->addr.thdl2_als, &(buf[1]), 1);
	if (ret < 0)
	{
		APS_ERR("WARNING: %s: %d\n", __func__, ret);
		return -EFAULT;
	}	
	
	return 0;    
}
/*----------------------------------------------------------------------------*/
#if 0
int stk3x1x_write_foffset(struct i2c_client *client, u16 ofset)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);        
	u8 buf[2];
	int ret = 0;
	
	buf[0] = (u8) ((0xFF00 & ofset) >> 8);
	buf[1] = (u8) (0x00FF & ofset);	
	ret = stk3x1x_master_send(client, obj->addr.data1_offset, &buf[0], 1);
	if (ret < 0)
	{
		APS_ERR("WARNING: %s: %d\n", __func__, ret);
		return -EFAULT;
	}
	
	ret = stk3x1x_master_send(client, obj->addr.data2_offset, &(buf[1]), 1);
	if (ret < 0)
	{
		APS_ERR("WARNING: %s: %d\n", __func__, ret);
		return -EFAULT;
	}	
	
	return 0;   	
}

/*----------------------------------------------------------------------------*/

int stk3x1x_write_aoffset(struct i2c_client *client,  u16 ofset)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);        
	u8 buf[2];
	int ret = 0;
	u8 s_buf = 0, re_en;
	ret = stk3x1x_master_recv(client, obj->addr.state, &s_buf, 1);
	if (ret < 0)
	{
		APS_ERR("i2c read state error = %d\n", ret);
		return -EFAULT;
	}		
	re_en = (s_buf & STK_STATE_EN_AK_MASK) ? 1: 0;
	if(re_en)
	{
		s_buf &= (~STK_STATE_EN_AK_MASK); 		
		ret = stk3x1x_master_send(client, obj->addr.state, &s_buf, 1);
		if (ret < 0)
		{
			APS_ERR("write state = %d\n", ret);
			return -EFAULT;
		} 			
		msleep(3);		
	}	

	buf[0] = (u8) ((0xFF00 & ofset) >> 8);
	buf[1] = (u8) (0x00FF & ofset);	
	ret = stk3x1x_master_send(client, 0x0E, &buf[0], 1);
	if (ret < 0)
	{
		APS_ERR("WARNING: %s: %d\n", __func__, ret);
		return -EFAULT;
	}
	
	ret = stk3x1x_master_send(client, 0x0F, &(buf[1]), 1);
	if (ret < 0)
	{
		APS_ERR("WARNING: %s: %d\n", __func__, ret);
		return -EFAULT;
	}	
	if(!re_en)
		return 0;
	s_buf |= STK_STATE_EN_AK_MASK; 		
	ret = stk3x1x_master_send(client, obj->addr.state, &s_buf, 1);
	if (ret < 0)
	{
		APS_ERR("write state = %d\n", ret);
		return -EFAULT;
	} 			
	return 0;  	
}
#endif
/*----------------------------------------------------------------------------*/
static void stk3x1x_power(struct alsps_hw *hw, unsigned int on) 
{
	static unsigned int power_on = 0;

	//APS_LOG("power %s\n", on ? "on" : "off");
#ifndef VENDOR_EDIT
	if(hw->power_id != POWER_NONE_MACRO)
	{
		if(power_on == on)
		{
			APS_LOG("ignore power control: %d\n", on);
		}
		else if(on)
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "stk3x1x")) 
			{
				APS_ERR("power on fails!!\n");
			}
		}
		else
		{
			if(!hwPowerDown(hw->power_id, "stk3x1x")) 
			{
				APS_ERR("power off fail!!\n");   
			}
		}
	}
#else
	hw->power(hw, on, NULL);
#endif
	power_on = on;
}

/*----------------------------------------------------------------------------*/
static int stk3x1x_enable_als(struct i2c_client *client, int enable)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);
	int err, cur = 0, old = atomic_read(&obj->state_val);
	int trc = atomic_read(&obj->trace);

	printk("%s: enable=%d\n", __func__, enable);
	cur = old & (~(STK_STATE_EN_ALS_MASK | STK_STATE_EN_WAIT_MASK)); 
	if(enable)
	{
		cur |= STK_STATE_EN_ALS_MASK;
	}
	else if (old & STK_STATE_EN_PS_MASK)
	{
		cur |= STK_STATE_EN_WAIT_MASK;   
	}
	if(trc & STK_TRC_DEBUG)
	{
		APS_LOG("%s: %08X, %08X, %d\n", __func__, cur, old, enable);
	}
	
	if(0 == (cur ^ old))
	{
		return 0;
	}
		
#ifdef STK_IRS		
	if(enable && !(old & STK_STATE_EN_PS_MASK))
	{		
		err =  stk3x1x_get_ir_value(obj);
		if(err > 0)
			obj->ir_code = err;
	}			
#endif
		
	if(enable && obj->hw->polling_mode_als == 0)
	{
		stk3x1x_write_als_high_thd(client, 0x0);
		stk3x1x_write_als_low_thd(client, 0xFFFF);
	}
	err = stk3x1x_write_state(client, cur);
	if(err < 0)
		return err;
	else
		atomic_set(&obj->state_val, cur);
	
	if(enable)
	{
		if(obj->hw->polling_mode_als)
		{
			atomic_set(&obj->als_deb_on, 1);
			atomic_set(&obj->als_deb_end, jiffies+atomic_read(&obj->als_debounce)*HZ/1000);
		}
		else
		{
			//set_bit(STK_BIT_ALS,  &obj->pending_intr);
			schedule_delayed_work(&obj->eint_work,220*HZ/1000); 
		}
	}

	if(trc & STK_TRC_DEBUG)
	{
		printk("enable als (%d)\n", enable);
	}

	return err;
}
/*----------------------------------------------------------------------------*/
static int stk3x1x_clear_intr(struct i2c_client *client, u8 status, u8 disable_flag) ;
static int stk3x1x_check_intr(struct i2c_client *client, u8 *status) ;
static int stk3x1x_enable_ps(struct i2c_client *client, int enable)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);
	int err, cur = 0, old = atomic_read(&obj->state_val);
	int trc = atomic_read(&obj->trace);
	hwm_sensor_data sensor_data;
	u8 int_val = 0;

	u8 i, val;

#ifdef VENDOR_EDIT//LiuPing@Prd.BasicDrv.Sensor, add 2013/06/20 add for ps switch 
    if (g_ps_switch_off == 1)
    {
        printk(" stk3x1x_enable_ps fail ,  ps switch off \n");
        return -1;
    }
#endif


#ifdef VENDOR_EDIT
	g_is_far = 1; /*init this flag with 1, means FAR*/
	g_ps_low_threshold = 0;
	g_ps_high_threshold = 0xFFFF;
	suspend_status = 0;
	g_first_thd = 0;
	stk3x1x_set_ps_threshold(client, g_ps_low_threshold, g_ps_high_threshold);
/*	if (!enable){
		val = i2c_smbus_read_byte_data(client, 0x10);

		if (!(val & 0x01)){
			stk3x1x_set_ps_threshold(client, 0xFFFF, 0xFFFF);
			msleep(10);
			if((err = stk3x1x_check_intr(client, &val)))
			{
				APS_ERR("check intr error: %d\n", err);
				return err;
			}
	
			if((err = stk3x1x_clear_intr(client, val, STK_FLG_PSINT_MASK | STK_FLG_ALSINT_MASK)))
			{
				APS_ERR("clear intr error: %d\n", err);	
				return err;
			}
		}	
	}*/
#endif
	
	cur = old;	
#ifdef STK_TUNE0		
	if (!(obj->psi_set) && !enable)
	{
		hrtimer_cancel(&obj->ps_tune0_timer);					
		cancel_work_sync(&obj->stk_ps_tune0_work);
	}	

	if(obj->first_boot == true)
	{		
		obj->first_boot = false;
	}
#else
	if(obj->first_boot == true)
	{			
		obj->first_boot = false;	
	}			
#endif

	printk("%s: enable=%d\n", __FUNCTION__, enable);	
	cur &= (~(0x45)); 
	if(enable)
	{
		cur |= (STK_STATE_EN_PS_MASK);
		if(!(old & STK_STATE_EN_ALS_MASK))
			cur |= STK_STATE_EN_WAIT_MASK;
		if(1 == obj->hw->polling_mode_ps)
			wake_lock(&ps_lock);
	}
	else
	{
		if(1 == obj->hw->polling_mode_ps)		
			wake_unlock(&ps_lock);
	}
	
	if(trc & STK_TRC_DEBUG)
	{
		APS_LOG("%s: %08X, %08X, %d\n", __func__, cur, old, enable);
	}
	
	if(0 == (cur ^ old))
	{
		return 0;
	}
	
	err = stk3x1x_write_state(client, cur);
	if(err < 0)
		return err;
	else
		atomic_set(&obj->state_val, cur);
	
	if(enable)
	{
#ifdef STK_TUNE0		
		if (!(obj->psi_set))
			hrtimer_start(&obj->ps_tune0_timer, obj->ps_tune0_delay, HRTIMER_MODE_REL);			
#endif		
		if(obj->hw->polling_mode_ps)
		{
			atomic_set(&obj->ps_deb_on, 1);
			atomic_set(&obj->ps_deb_end, jiffies+atomic_read(&obj->ps_debounce)*HZ/1000);
		}
		else
		{
		#if 0
			msleep(10);
			if((err = stk3x1x_read_ps(obj->client, &obj->ps)))
			{
				APS_ERR("stk3x1x read ps data: %d\n", err);
				return err;
			}
			
			err = stk3x1x_get_ps_value_only(obj, obj->ps);
			if(err < 0)
			{
				APS_ERR("stk3x1x get ps value: %d\n", err);
				return err;
			}
			else
		#endif
			if(stk3x1x_obj->hw->polling_mode_ps == 0)
			{	
				sensor_data.values[0] = g_is_far;
				sensor_data.value_divide = 1;
				sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
				printk("%s:ps raw 0x%x -> value 0x%x \n",__FUNCTION__, obj->ps,sensor_data.values[0]);
				if((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
				{	
					APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);
				}			
			}			

		}
		
		mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
	}else { 
		i2c_smbus_write_byte_data(client, obj->addr.intmode, int_val);
		mt65xx_eint_mask(CUST_EINT_ALS_NUM);
	}

	if(trc & STK_TRC_DEBUG)
	{
		printk("enable ps  (%d)\n", enable);
	}
	
	for (i = 0x00; i <= 0x18; i += 0x01){
		val = i2c_smbus_read_byte_data(stk3x1x_i2c_client, i);
		printk("%s  Reg 0x%x:0x%x\n", __func__, i, val);	
	}
	return err;
}
/*----------------------------------------------------------------------------*/

static int stk3x1x_check_intr(struct i2c_client *client, u8 *status) 
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);
	int err;

	//if (mt_get_gpio_in(GPIO_ALS_EINT_PIN) == 1) /*skip if no interrupt*/  
	//    return 0;

	err = stk3x1x_read_flag(client, status);	
	if (err < 0)
	{
		APS_ERR("WARNING: read flag reg error: %d\n", err);
		return -EFAULT;
	}
	APS_LOG("%s: read status reg: 0x%x\n", __func__, *status);
    
	if(*status & STK_FLG_ALSINT_MASK)
	{
		set_bit(STK_BIT_ALS, &obj->pending_intr);
	}
	else
	{
	   clear_bit(STK_BIT_ALS, &obj->pending_intr);
	}
	
	if(*status & STK_FLG_PSINT_MASK)
	{
		set_bit(STK_BIT_PS,  &obj->pending_intr);
	}
	else
	{
	    clear_bit(STK_BIT_PS, &obj->pending_intr);
	}
	
	if(atomic_read(&obj->trace) & STK_TRC_DEBUG)
	{
		APS_LOG("check intr: 0x%02X => 0x%08lX\n", *status, obj->pending_intr);
	}

	return 0;
}


static int stk3x1x_clear_intr(struct i2c_client *client, u8 status, u8 disable_flag) 
{
	int err = 0;

	status = status | (STK_FLG_ALSINT_MASK | STK_FLG_PSINT_MASK | STK_FLG_OUI_MASK | STK_FLG_IR_RDY_MASK);
	status &= (~disable_flag);
	printk("%s set flag reg: 0x%x\n",__func__, status);
	if((err = stk3x1x_write_flag(client, status)))
		APS_ERR("stk3x1x_write_flag failed, err=%d\n", err);
	return err;
}

/*----------------------------------------------------------------------------*/
static int stk3x1x_set_als_int_thd(struct i2c_client *client, u16 als_data_reg) 
{
	s32 als_thd_h, als_thd_l;	
		
	als_thd_h = als_data_reg + STK_ALS_CODE_CHANGE_THD;
	als_thd_l = als_data_reg - STK_ALS_CODE_CHANGE_THD;
	if (als_thd_h >= (1<<16))
		als_thd_h = (1<<16) -1;
	if (als_thd_l <0)
		als_thd_l = 0;
	APS_LOG("stk3x1x_set_als_int_thd:als_thd_h:%d,als_thd_l:%d\n", als_thd_h, als_thd_l);	
		
	stk3x1x_write_als_high_thd(client, als_thd_h);
	stk3x1x_write_als_low_thd(client, als_thd_l);

	return 0;
}

#ifdef STK_TUNE0	
static int stk3x1x_ps_tune_zero_val(void)
{
	int mode;
	int32_t word_data, lii;
	u8 buf[2];
	int ret;
	
	ret = stk3x1x_master_recv(stk3x1x_obj->client, 0x20, buf, 2);
	if(ret < 0)	
	{
		APS_ERR("%s fail, err=0x%x", __FUNCTION__, ret);
		return ret;	   
	}
	word_data = (buf[0] << 8) | buf[1];
	
	ret = stk3x1x_master_recv(stk3x1x_obj->client, 0x22, buf, 2);
	if(ret < 0)		
	{
		APS_ERR("%s fail, err=0x%x", __FUNCTION__, ret);
		return ret;	   
	}	
	word_data += (buf[0] << 8) | buf[1];	
	
	mode = atomic_read(&stk3x1x_obj->psctrl_val) & 0x3F;
	if(mode == 0x30)	
	lii = 100;	
	else if (mode == 0x31)
	lii = 200;		
	else if (mode == 0x32)
	lii = 400;				
	else if (mode == 0x33)
	lii = 800;			
	else
	{
		APS_ERR("%s: unsupported PS_IT(0x%x)\n", __FUNCTION__, mode);
		return -1;
	}
	
	if(word_data > lii)	
	{
		APS_LOG( "%s: word_data=%d, lii=%d\n", __FUNCTION__, word_data, lii);		
		return 0xFFFF;	
	}
	return 0;
}

static int stk_ps_tune_zero_final(struct stk3x1x_priv *obj)
{
	int err;
	
	obj->tune_zero_init_proc = false;
	if((err = stk3x1x_write_int(obj->client, obj->int_val)))
	{
		APS_ERR("write int mode error: %d\n", err);
		return err;        
	}	
	
	if((err = stk3x1x_write_state(obj->client, atomic_read(&obj->state_val))))
	{
		APS_ERR("write stete error: %d\n", err);
		return err;        
	}		
	
	if(obj->data_count == -1)
	{
		APS_LOG("%s: exceed limit\n", __func__);
		hrtimer_cancel(&obj->ps_tune0_timer);	
		return 0;
	}	
	
	obj->psa = obj->ps_stat_data[0];
	obj->psi = obj->ps_stat_data[2];							
	atomic_set(&obj->ps_high_thd_val, obj->ps_stat_data[1] + STK_HT_N_CT); 
	atomic_set(&obj->ps_low_thd_val, obj->ps_stat_data[1] + STK_LT_N_CT); 		
	if((err = stk3x1x_write_ps_high_thd(obj->client, atomic_read(&obj->ps_high_thd_val))))
	{
		APS_ERR("write high thd error: %d\n", err);
		return err;        
	}	
	if((err = stk3x1x_write_ps_low_thd(obj->client, atomic_read(&obj->ps_low_thd_val))))
	{
		APS_ERR("write low thd error: %d\n", err);
		return err;        
	}
	
	APS_LOG("%s: set HT=%d,LT=%d\n", __func__, atomic_read(&obj->ps_high_thd_val),  atomic_read(&obj->ps_low_thd_val));		
	hrtimer_cancel(&obj->ps_tune0_timer);					
	return 0;
}

static int32_t stk_tune_zero_get_ps_data(struct stk3x1x_priv *obj)
{
	int err;
	
	err = stk3x1x_ps_tune_zero_val();	
	if(err == 0xFFFF)
	{	
		obj->data_count = -1;
		stk_ps_tune_zero_final(obj);
		return 0;	
	}
	
	if((err = stk3x1x_read_ps(obj->client, &obj->ps)))
	{
		APS_ERR("stk3x1x read ps data: %d\n", err);
		return err;
	}	
	APS_LOG("%s: ps #%d=%d\n", __func__, obj->data_count, obj->ps);
	
	obj->ps_stat_data[1]  +=  obj->ps;			
	if(obj->ps > obj->ps_stat_data[0])
		obj->ps_stat_data[0] = obj->ps;
	if(obj->ps < obj->ps_stat_data[2])
		obj->ps_stat_data[2] = obj->ps;						
	obj->data_count++;	
	
	if(obj->data_count == 5)
	{
		obj->ps_stat_data[1]  /= obj->data_count;			
		stk_ps_tune_zero_final(obj);
	}		
	
	return 0;
}

static int stk_ps_tune_zero_init(struct stk3x1x_priv *obj)
{
	u8 w_state_reg;	
	int err;
	
	obj->psa = 0;
	obj->psi = 0xFFFF;	
	obj->psi_set = 0;	
	obj->tune_zero_init_proc = true;		
	obj->ps_stat_data[0] = 0;
	obj->ps_stat_data[2] = 9999;
	obj->ps_stat_data[1] = 0;
	obj->data_count = 0;
	
	if((err = stk3x1x_write_int(obj->client, 0)))
	{
		APS_ERR("write int mode error: %d\n", err);
		return err;        
	}	
	
	w_state_reg = (STK_STATE_EN_PS_MASK | STK_STATE_EN_WAIT_MASK);			
	if((err = stk3x1x_write_state(obj->client, w_state_reg)))
	{
		APS_ERR("write stete error: %d\n", err);
		return err;        
	}			
	hrtimer_start(&obj->ps_tune0_timer, obj->ps_tune0_delay, HRTIMER_MODE_REL);		
	return 0;	
}


static int stk_ps_tune_zero_func_fae(struct stk3x1x_priv *obj)
{
	int32_t word_data;
	u8 flag;
	bool ps_enabled = false;
	u8 buf[2];
	int ret, diff;
	
	ps_enabled = (atomic_read(&obj->state_val) & STK_STATE_EN_PS_MASK) ? true : false;	
	if(obj->psi_set || !(ps_enabled))
	{
		//APS_LOG("%s: FAE tune0 close\n", __func__);
		return 0;
	}	
	//APS_LOG("%s: FAE tune0 entry\n", __func__);
	
	ret = stk3x1x_read_flag(obj->client, &flag);
	if(ret < 0)
	{
		printk(KERN_ERR "%s: get flag failed, err=0x%x\n", __func__, ret);
		return ret;
	}
	if(!(flag&STK_FLG_PSDR_MASK))
	{
		return 0;
	}
	
	ret = stk3x1x_ps_tune_zero_val();	
	if(ret == 0)
	{
		ret = stk3x1x_master_recv(obj->client, 0x11, buf, 2);
		if(ret < 0)
		{
			printk(KERN_ERR "%s fail, err=0x%x", __func__, ret);
			return ret;	   
		}
		word_data = (buf[0] << 8) | buf[1];
		//APS_LOG("%s: word_data=%d\n", __func__, word_data);
		
		if(word_data == 0)
		{
			printk(KERN_ERR "%s: incorrect word data (0)\n", __func__);
			return 0xFFFF;
		}
		
		if(word_data > obj->psa)
		{
			obj->psa = word_data;
			APS_LOG("%s: update psa: psa=%d,psi=%d\n", __func__, obj->psa, obj->psi);
		}
		if(word_data < obj->psi)
		{
			obj->psi = word_data;	
			APS_LOG("%s: update psi: psa=%d,psi=%d\n", __func__, obj->psa, obj->psi);	
		}	
	}	
	
	diff = obj->psa - obj->psi;
	if(diff > STK_MAX_MIN_DIFF)
	{
		obj->psi_set = obj->psi;
		atomic_set(&obj->ps_high_thd_val, obj->psi + STK_HT_N_CT); 
		atomic_set(&obj->ps_low_thd_val, obj->psi + STK_LT_N_CT); 
		
		if((ret = stk3x1x_write_ps_high_thd(obj->client, atomic_read(&obj->ps_high_thd_val))))
		{
			APS_ERR("write high thd error: %d\n", ret);
			return ret;        
		}		
		if((ret = stk3x1x_write_ps_low_thd(obj->client, atomic_read(&obj->ps_low_thd_val))))
		{
			APS_ERR("write low thd error: %d\n", ret);
			return ret;        
		}	
#ifdef STK_DEBUG_PRINTF				
		APS_LOG("%s: FAE tune0 psa-psi(%d) > DIFF found\n", __func__, diff);
#endif					
		hrtimer_cancel(&obj->ps_tune0_timer);
	}
	
	//APS_LOG("%s: FAE tune0 exit\n", __func__);
	return 0;
}

static void stk_ps_tune0_work_func(struct work_struct *work)
{
	struct stk3x1x_priv *obj = container_of(work, struct stk3x1x_priv, stk_ps_tune0_work);		
	if(obj->tune_zero_init_proc)
		stk_tune_zero_get_ps_data(obj);
	else
		stk_ps_tune_zero_func_fae(obj);
	return;
}	


static enum hrtimer_restart stk_ps_tune0_timer_func(struct hrtimer *timer)
{
	struct stk3x1x_priv *obj = container_of(timer, struct stk3x1x_priv, ps_tune0_timer);
	queue_work(obj->stk_ps_tune0_wq, &obj->stk_ps_tune0_work);	
	hrtimer_forward_now(&obj->ps_tune0_timer, obj->ps_tune0_delay);
	return HRTIMER_RESTART;	
}
	
#endif	/*#ifdef STK_TUNE0	*/

/*----------------------------------------------------------------------------*/
void stk3x1x_eint_func(void)
{
	struct stk3x1x_priv *obj = g_stk3x1x_ptr;
	APS_LOG(" interrupt fuc\n");
	if(!obj)
	{
		return;
	}
	//schedule_work(&obj->eint_work);
	if(obj->hw->polling_mode_ps == 0 || obj->hw->polling_mode_als == 0)
		schedule_delayed_work(&obj->eint_work,0);
	if(atomic_read(&obj->trace) & STK_TRC_EINT)
	{
		APS_LOG("eint: als/ps intrs\n");
	}
}
/*----------------------------------------------------------------------------*/
static void stk3x1x_eint_work(struct work_struct *work)
{
	struct stk3x1x_priv *obj = g_stk3x1x_ptr;
	int err;
	hwm_sensor_data sensor_data;
	u8 flag_reg, disable_flag = 0;
	
	memset(&sensor_data, 0, sizeof(sensor_data));

	printk("%s  eint work\n",__func__);
	
	if((err = stk3x1x_check_intr(obj->client, &flag_reg)))
	{
		APS_ERR("stk3x1x_check_intr fail: %d\n", err);
		goto err_i2c_rw;
	}

	printk(" &obj->pending_intr =%lx\n",obj->pending_intr);
	
	if(((1<<STK_BIT_PS) &  obj->pending_intr) && (obj->hw->polling_mode_ps == 0))
	{
		printk("stk ps change\n");
		disable_flag |= STK_FLG_PSINT_MASK;
		
		if((err = stk3x1x_read_ps(obj->client, &obj->ps)))
		{
			APS_ERR("stk3x1x read ps data: %d\n", err);
			goto err_i2c_rw;
		}
	#ifdef VENDOR_EDIT
		sensor_data.values[0] = (flag_reg & STK_FLG_NF_MASK)? 1 : 0;
		g_is_far = sensor_data.values[0];
	#else
		g_is_far = !g_is_far;
		sensor_data.values[0] = g_is_far;
	#endif
		sensor_data.value_divide = 1;
		sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
		printk("%s:ps raw %d -> value %d \n",__FUNCTION__, obj->ps,sensor_data.values[0]);
		printk("%s:ps low_thd=%d---high_thd=%d \n",__FUNCTION__, g_ps_low_threshold,g_ps_high_threshold);
		//let up layer to know
		if((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
		{
			APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);
		}
		
	}
	
	if((err = stk3x1x_clear_intr(obj->client, flag_reg, disable_flag)))
	{
		APS_ERR("fail: %d\n", err);
		goto err_i2c_rw;
	}		

	msleep(1);
   
#if ((defined MT6573) || (defined MT6575) || (defined MT6577) || (defined MT6589) || (defined MT6572))
	mt65xx_eint_unmask(CUST_EINT_ALS_NUM);    
#endif	

	return;
	
err_i2c_rw:	
	msleep(30);
   
#if ((defined MT6573) || (defined MT6575) || (defined MT6577) || (defined MT6589) || (defined MT6572))
	mt65xx_eint_unmask(CUST_EINT_ALS_NUM);    
#endif	
	return;
}
/*----------------------------------------------------------------------------*/
int stk3x1x_setup_eint(struct i2c_client *client)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);        

	g_stk3x1x_ptr = obj;
	/*configure to GPIO function, external interrupt*/

    printk("ALS/PS  %s  interrupt pin = %d\n",__func__, GPIO_ALS_EINT_PIN);

    mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
    mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);

    mt65xx_eint_set_sens(CUST_EINT_ALS_NUM, CUST_EINT_ALS_SENSITIVE);
    mt65xx_eint_set_polarity(CUST_EINT_ALS_NUM, CUST_EINT_ALS_POLARITY);
    mt65xx_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
    mt65xx_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_EN, CUST_EINT_ALS_POLARITY, stk3x1x_eint_func, 0);
    /*zhye modify for disable interrupt when init device*/
    mt65xx_eint_mask(CUST_EINT_ALS_NUM);
    return 0;
}
/*----------------------------------------------------------------------------*/
static int stk3x1x_init_client(struct i2c_client *client)
{
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);
	int err;
	int ps_ctrl;
	//u8 int_status;
	
	if((err = stk3x1x_write_sw_reset(client)))
	{
		APS_ERR("software reset error, err=%d", err);
		return err;
	}

	if((err = stk3x1x_read_id(client)))
	{
		APS_ERR("stk3x1x_read_id error, err=%d", err);
		return err;
	}		
	
	if((err = stk3x1x_write_state(client, atomic_read(&obj->state_val))))
	{
		APS_ERR("write stete error: %d\n", err);
		return err;        
	}	
	
	/*
	if((err = stk3x1x_check_intr(client, &int_status)))
	{
		APS_ERR("check intr error: %d\n", err);
		//    return err;
	}
	
	if((err = stk3x1x_clear_intr(client, int_status, STK_FLG_PSINT_MASK | STK_FLG_ALSINT_MASK)))
	{
		APS_ERR("clear intr error: %d\n", err);	
		return err;
	}
	*/
	ps_ctrl = atomic_read(&obj->psctrl_val);
	if(obj->hw->polling_mode_ps == 1)
		ps_ctrl &= 0x3F;
	
	if((err = stk3x1x_write_ps(client, ps_ctrl)))
	{
		APS_ERR("write ps error: %d\n", err);
		return err;        
	}
	
	if((err = stk3x1x_write_als(client, atomic_read(&obj->alsctrl_val))))
	{
		APS_ERR("write als error: %d\n", err);
		return err;
	}	
	
	if((err = stk3x1x_write_led(client, obj->ledctrl_val)))
	{
		APS_ERR("write led error: %d\n", err);
		return err;
	}	
	
	if((err = stk3x1x_write_wait(client, obj->wait_val)))
	{
		APS_ERR("write wait error: %d\n", err);
		return err;
	}	
#ifndef STK_TUNE0	
	if((err = stk3x1x_write_ps_high_thd(client, atomic_read(&obj->ps_high_thd_val))))
	{
		APS_ERR("write high thd error: %d\n", err);
		return err;        
	}
	
	if((err = stk3x1x_write_ps_low_thd(client, atomic_read(&obj->ps_low_thd_val))))
	{
		APS_ERR("write low thd error: %d\n", err);
		return err;        
	}
#endif	
	if((err = stk3x1x_write_int(client, obj->int_val)))
	{
		APS_ERR("write int mode error: %d\n", err);
		return err;        
	}	
#ifdef STK_FIR
	memset(&obj->fir, 0x00, sizeof(obj->fir));  
#endif
#ifdef STK_TUNE0
	stk_ps_tune_zero_init(obj);
#endif	
	if(obj->hw->polling_mode_ps == 0 || obj->hw->polling_mode_als == 0)
	{
		if((err = stk3x1x_setup_eint(client)))
		{
			APS_ERR("setup eint error: %d\n", err);
			return err;
		}
	}
	return 0;
}

/******************************************************************************
 * Sysfs attributes
*******************************************************************************/
static ssize_t stk3x1x_show_config(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	
	res = scnprintf(buf, PAGE_SIZE, "(%d %d %d %d %d %d)\n", 
		atomic_read(&stk3x1x_obj->i2c_retry), atomic_read(&stk3x1x_obj->als_debounce), 
		atomic_read(&stk3x1x_obj->ps_mask), atomic_read(&stk3x1x_obj->ps_high_thd_val),atomic_read(&stk3x1x_obj->ps_low_thd_val), atomic_read(&stk3x1x_obj->ps_debounce));     
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_store_config(struct device_driver *ddri, const char *buf, size_t count)
{
	int retry, als_deb, ps_deb, mask, hthres, lthres, err;
	struct i2c_client *client;
	client = stk3x1x_i2c_client;
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	
	if(6 == sscanf(buf, "%d %d %d %d %d %d", &retry, &als_deb, &mask, &hthres, &lthres, &ps_deb))
	{ 
		atomic_set(&stk3x1x_obj->i2c_retry, retry);
		atomic_set(&stk3x1x_obj->als_debounce, als_deb);
		atomic_set(&stk3x1x_obj->ps_mask, mask);
		atomic_set(&stk3x1x_obj->ps_high_thd_val, hthres);    
		atomic_set(&stk3x1x_obj->ps_low_thd_val, lthres);        
		atomic_set(&stk3x1x_obj->ps_debounce, ps_deb);

		if((err = stk3x1x_write_ps_high_thd(client, atomic_read(&stk3x1x_obj->ps_high_thd_val))))
		{
			APS_ERR("write high thd error: %d\n", err);
			return err;        
		}
		
		if((err = stk3x1x_write_ps_low_thd(client, atomic_read(&stk3x1x_obj->ps_low_thd_val))))
		{
			APS_ERR("write low thd error: %d\n", err);
			return err;        
		}
	}
	else
	{
		APS_ERR("invalid content: '%s', length = %d\n", buf, count);
	}
	return count;    
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_show_trace(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}

	res = scnprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&stk3x1x_obj->trace));     
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_store_trace(struct device_driver *ddri, const char *buf, size_t count)
{
	int trace;
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&stk3x1x_obj->trace, trace);
	}
	else 
	{
		APS_ERR("invalid content: '%s', length = %d\n", buf, (int)count);
	}
	return count;    
}

/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_show_ir(struct device_driver *ddri, char *buf)
{
    int32_t reading;
	
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	reading = stk3x1x_get_ir_value(stk3x1x_obj);
	if(reading < 0)
		return scnprintf(buf, PAGE_SIZE, "ERROR: %d\n", reading);

	stk3x1x_obj->ir_code = reading;
	return scnprintf(buf, PAGE_SIZE, "0x%04X\n", stk3x1x_obj->ir_code);     
}

/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_show_als(struct device_driver *ddri, char *buf)
{
	int res;
	
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	if((res = stk3x1x_read_als(stk3x1x_obj->client, &stk3x1x_obj->als)))
	{
		return scnprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
	}
	else
	{
		return scnprintf(buf, PAGE_SIZE, "0x%04X\n", stk3x1x_obj->als);     
	}
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_show_ps(struct device_driver *ddri, char *buf)
{
	int res;
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	
	if((res = stk3x1x_read_ps(stk3x1x_obj->client, &stk3x1x_obj->ps)))
	{
		return scnprintf(buf, PAGE_SIZE, "ERROR: %d\n", res);
	}
	else
	{
		return scnprintf(buf, PAGE_SIZE, "0x%04X\n", stk3x1x_obj->ps);     
	}
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_show_reg(struct device_driver *ddri, char *buf)
{
	u8 int_status;
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	
	/*read*/
	stk3x1x_check_intr(stk3x1x_obj->client, &int_status);
	//stk3x1x_clear_intr(stk3x1x_obj->client, int_status, 0x0);
	stk3x1x_read_ps(stk3x1x_obj->client, &stk3x1x_obj->ps);
	stk3x1x_read_als(stk3x1x_obj->client, &stk3x1x_obj->als);
	/*write*/
	stk3x1x_write_als(stk3x1x_obj->client, atomic_read(&stk3x1x_obj->alsctrl_val));
	stk3x1x_write_ps(stk3x1x_obj->client, atomic_read(&stk3x1x_obj->psctrl_val)); 
	stk3x1x_write_ps_high_thd(stk3x1x_obj->client, atomic_read(&stk3x1x_obj->ps_high_thd_val));
	stk3x1x_write_ps_low_thd(stk3x1x_obj->client, atomic_read(&stk3x1x_obj->ps_low_thd_val));
	return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_show_send(struct device_driver *ddri, char *buf)
{
    return 0;
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_store_send(struct device_driver *ddri, const char *buf, size_t count)
{
	int addr, cmd;
	u8 dat;

	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	else if(2 != sscanf(buf, "%x %x", &addr, &cmd))
	{
		APS_ERR("invalid format: '%s'\n", buf);
		return 0;
	}

	dat = (u8)cmd;
	APS_LOG("send(%02X, %02X) = %d\n", addr, cmd, 
	stk3x1x_master_send(stk3x1x_obj->client, (u16)addr, &dat, sizeof(dat)));
	
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_show_recv(struct device_driver *ddri, char *buf)
{
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	return scnprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&stk3x1x_obj->recv_reg));     	
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_store_recv(struct device_driver *ddri, const char *buf, size_t count)
{
	int addr;
	u8 dat;
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	else if(1 != sscanf(buf, "%x", &addr))
	{
		APS_ERR("invalid format: '%s'\n", buf);
		return 0;
	}

	APS_LOG("recv(%02X) = %d, 0x%02X\n", addr, 
	stk3x1x_master_recv(stk3x1x_obj->client, (u16)addr, (char*)&dat, sizeof(dat)), dat);
	atomic_set(&stk3x1x_obj->recv_reg, dat);	
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_show_allreg(struct device_driver *ddri, char *buf)
{
	int ret = 0;
	u8 rbuf[27];
	int cnt;	
	
	memset(rbuf, 0, sizeof(rbuf));
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	ret = stk3x1x_master_recv(stk3x1x_obj->client, 0, &rbuf[0], 7);
	if(ret < 0)
	{
		APS_DBG("error: %d\n", ret);
		return -EFAULT;
	}	
	ret = stk3x1x_master_recv(stk3x1x_obj->client, 7, &rbuf[7], 7);
	if(ret < 0)
	{
		APS_DBG("error: %d\n", ret);
		return -EFAULT;
	}	
	ret = stk3x1x_master_recv(stk3x1x_obj->client, 14, &rbuf[14], 7);
	if(ret < 0)
	{
		APS_DBG("error: %d\n", ret);
		return -EFAULT;
	}	
	ret = stk3x1x_master_recv(stk3x1x_obj->client, 21, &rbuf[21], 4);
	if(ret < 0)
	{
		APS_DBG("error: %d\n", ret);
		return -EFAULT;
	}
	ret = stk3x1x_master_recv(stk3x1x_obj->client, STK_PDT_ID_REG, &rbuf[25], 2);
	if(ret < 0)
	{
		APS_DBG("error: %d\n", ret);
		return -EFAULT;
	}	
	
	for(cnt=0;cnt<25;cnt++)
	{
		APS_LOG("reg[0x%x]=0x%x\n", cnt, rbuf[cnt]);
	}	
	APS_LOG("reg[0x3E]=0x%x\n", rbuf[cnt]);
	APS_LOG("reg[0x3F]=0x%x\n", rbuf[cnt++]);
	return scnprintf(buf, PAGE_SIZE, "[0]%2X [1]%2X [2]%2X [3]%2X [4]%2X [5]%2X [6/7 HTHD]%2X,%2X [8/9 LTHD]%2X, %2X [A]%2X [B]%2X [C]%2X [D]%2X [E/F Aoff]%2X,%2X,[10]%2X [11/12 PS]%2X,%2X [13]%2X [14]%2X [15/16 Foff]%2X,%2X [17]%2X [18]%2X [3E]%2X [3F]%2X\n", 	
		rbuf[0], rbuf[1], rbuf[2], rbuf[3], rbuf[4], rbuf[5], rbuf[6], rbuf[7], rbuf[8], 
		rbuf[9], rbuf[10], rbuf[11], rbuf[12], rbuf[13], rbuf[14], rbuf[15], rbuf[16], rbuf[17], 
		rbuf[18], rbuf[19], rbuf[20], rbuf[21], rbuf[22], rbuf[23], rbuf[24], rbuf[25], rbuf[26]);	
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_show_status(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	u8 rbuf[25];
	int ret = 0;
	
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	
	if(stk3x1x_obj->hw)
	{
		len += scnprintf(buf+len, PAGE_SIZE-len, "CUST: %d, (%d %d) (%02X) (%02X %02X %02X) (%02X %02X %02X %02X)\n", 
			stk3x1x_obj->hw->i2c_num, stk3x1x_obj->hw->power_id, stk3x1x_obj->hw->power_vol, stk3x1x_obj->addr.flag, 
			stk3x1x_obj->addr.alsctrl, stk3x1x_obj->addr.data1_als, stk3x1x_obj->addr.data2_als, stk3x1x_obj->addr.psctrl, 
			stk3x1x_obj->addr.data1_ps, stk3x1x_obj->addr.data2_ps, stk3x1x_obj->addr.thdh1_ps);
	}
	else
	{
		len += scnprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}
	
	len += scnprintf(buf+len, PAGE_SIZE-len, "REGS: %02X %02X %02X %02X %02X %02X %02X %02X %02lX %02lX\n", 
				atomic_read(&stk3x1x_obj->state_val), atomic_read(&stk3x1x_obj->psctrl_val), atomic_read(&stk3x1x_obj->alsctrl_val), 
				stk3x1x_obj->ledctrl_val, stk3x1x_obj->int_val, stk3x1x_obj->wait_val, 
				atomic_read(&stk3x1x_obj->ps_high_thd_val), atomic_read(&stk3x1x_obj->ps_low_thd_val),stk3x1x_obj->enable, stk3x1x_obj->pending_intr);
#ifdef MT6516
	len += scnprintf(buf+len, PAGE_SIZE-len, "EINT: %d (%d %d %d %d)\n", mt_get_gpio_in(GPIO_ALS_EINT_PIN),
				CUST_EINT_ALS_NUM, CUST_EINT_ALS_POLARITY, CUST_EINT_ALS_DEBOUNCE_EN, CUST_EINT_ALS_DEBOUNCE_CN);

	len += scnprintf(buf+len, PAGE_SIZE-len, "GPIO: %d (%d %d %d %d)\n",	GPIO_ALS_EINT_PIN, 
				mt_get_gpio_dir(GPIO_ALS_EINT_PIN), mt_get_gpio_mode(GPIO_ALS_EINT_PIN), 
				mt_get_gpio_pull_enable(GPIO_ALS_EINT_PIN), mt_get_gpio_pull_select(GPIO_ALS_EINT_PIN));
#endif

	len += scnprintf(buf+len, PAGE_SIZE-len, "MISC: %d %d\n", atomic_read(&stk3x1x_obj->als_suspend), atomic_read(&stk3x1x_obj->ps_suspend));	
	len += scnprintf(buf+len, PAGE_SIZE-len, "VER.: %s\n", DRIVER_VERSION);
	
	memset(rbuf, 0, sizeof(rbuf));	
	ret = stk3x1x_master_recv(stk3x1x_obj->client, 0, &rbuf[0], 7);
	if(ret < 0)
	{
		APS_DBG("error: %d\n", ret);
		return -EFAULT;
	}	
	ret = stk3x1x_master_recv(stk3x1x_obj->client, 7, &rbuf[7], 7);
	if(ret < 0)
	{
		APS_DBG("error: %d\n", ret);
		return -EFAULT;
	}	
	ret = stk3x1x_master_recv(stk3x1x_obj->client, 14, &rbuf[14], 7);
	if(ret < 0)
	{
		APS_DBG("error: %d\n", ret);
		return -EFAULT;
	}		
	/*
	ret = stk3x1x_master_recv(stk3x1x_obj->client, 21, &rbuf[21], 4);
	if(ret < 0)
	{
		APS_DBG("error: %d\n", ret);
		return -EFAULT;
	}
	*/	
	len += scnprintf(buf+len, PAGE_SIZE-len, "[PS=%2X] [ALS=%2X] [WAIT=%4Xms] [EN_ASO=%2X] [EN_AK=%2X] [NEAR/FAR=%2X] [FLAG_OUI=%2X] [FLAG_PSINT=%2X] [FLAG_ALSINT=%2X]\n", 
		rbuf[0]&0x01,(rbuf[0]&0x02)>>1,((rbuf[0]&0x04)>>2)*rbuf[5]*6,(rbuf[0]&0x20)>>5,
		(rbuf[0]&0x40)>>6,rbuf[16]&0x01,(rbuf[16]&0x04)>>2,(rbuf[16]&0x10)>>4,(rbuf[16]&0x20)>>5);		
	
	return len;
}
/*----------------------------------------------------------------------------*/
#define IS_SPACE(CH) (((CH) == ' ') || ((CH) == '\n'))
/*----------------------------------------------------------------------------*/
static int read_int_from_buf(struct stk3x1x_priv *obj, const char* buf, size_t count,
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
static ssize_t stk3x1x_show_alslv(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	int idx;
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	
	for(idx = 0; idx < stk3x1x_obj->als_level_num; idx++)
	{
		len += scnprintf(buf+len, PAGE_SIZE-len, "%d ", stk3x1x_obj->hw->als_level[idx]);
	}
	len += scnprintf(buf+len, PAGE_SIZE-len, "\n");
	return len;    
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_store_alslv(struct device_driver *ddri, const char *buf, size_t count)
{
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	else if(!strcmp(buf, "def"))
	{
		memcpy(stk3x1x_obj->als_level, stk3x1x_obj->hw->als_level, sizeof(stk3x1x_obj->als_level));
	}
	else if(stk3x1x_obj->als_level_num != read_int_from_buf(stk3x1x_obj, buf, count, 
			stk3x1x_obj->hw->als_level, stk3x1x_obj->als_level_num))
	{
		APS_ERR("invalid format: '%s'\n", buf);
	}    
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_show_alsval(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	int idx;
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	
	for(idx = 0; idx < stk3x1x_obj->als_value_num; idx++)
	{
		len += scnprintf(buf+len, PAGE_SIZE-len, "%d ", stk3x1x_obj->hw->als_value[idx]);
	}
	len += scnprintf(buf+len, PAGE_SIZE-len, "\n");
	return len;    
}
/*----------------------------------------------------------------------------*/
static ssize_t stk3x1x_store_alsval(struct device_driver *ddri, const char *buf, size_t count)
{
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}
	else if(!strcmp(buf, "def"))
	{
		memcpy(stk3x1x_obj->als_value, stk3x1x_obj->hw->als_value, sizeof(stk3x1x_obj->als_value));
	}
	else if(stk3x1x_obj->als_value_num != read_int_from_buf(stk3x1x_obj, buf, count, 
			stk3x1x_obj->hw->als_value, stk3x1x_obj->als_value_num))
	{
		APS_ERR("invalid format: '%s'\n", buf);
	}    
	return count;
}

#ifdef STK_TUNE0
static ssize_t stk3x1x_show_cali(struct device_driver *ddri, char *buf)
{
	int32_t word_data;
	u8 r_buf[2];
	int ret;
	
	if(!stk3x1x_obj)
	{
		APS_ERR("stk3x1x_obj is null!!\n");
		return 0;
	}

	ret = stk3x1x_master_recv(stk3x1x_obj->client, 0x20, r_buf, 2);
	if(ret < 0)	
	{
		APS_ERR("%s fail, err=0x%x", __FUNCTION__, ret);
		return ret;	   
	}
	word_data = (r_buf[0] << 8) | r_buf[1];

	ret = stk3x1x_master_recv(stk3x1x_obj->client, 0x22, r_buf, 2);
	if(ret < 0)		
	{
		APS_ERR("%s fail, err=0x%x", __FUNCTION__, ret);
		return ret;	   
	}	
	word_data += (r_buf[0] << 8) | r_buf[1];	

	APS_LOG("%s: psi_set=%d, psa=%d,psi=%d, word_data=%d\n", __FUNCTION__, 
		stk3x1x_obj->psi_set, stk3x1x_obj->psa, stk3x1x_obj->psi, word_data);	
	
	return 0;
}

#endif
/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(als,     S_IWUSR | S_IRUGO, stk3x1x_show_als,   NULL);
static DRIVER_ATTR(ps,      S_IWUSR | S_IRUGO, stk3x1x_show_ps,    NULL);
static DRIVER_ATTR(ir,      S_IWUSR | S_IRUGO, stk3x1x_show_ir,    NULL);
static DRIVER_ATTR(config,  S_IWUSR | S_IRUGO, stk3x1x_show_config,stk3x1x_store_config);
static DRIVER_ATTR(alslv,   S_IWUSR | S_IRUGO, stk3x1x_show_alslv, stk3x1x_store_alslv);
static DRIVER_ATTR(alsval,  S_IWUSR | S_IRUGO, stk3x1x_show_alsval,stk3x1x_store_alsval);
static DRIVER_ATTR(trace,   S_IWUSR | S_IRUGO, stk3x1x_show_trace, stk3x1x_store_trace);
static DRIVER_ATTR(status,  S_IWUSR | S_IRUGO, stk3x1x_show_status,  NULL);
static DRIVER_ATTR(send,    S_IWUSR | S_IRUGO, stk3x1x_show_send,  stk3x1x_store_send);
static DRIVER_ATTR(recv,    S_IWUSR | S_IRUGO, stk3x1x_show_recv,  stk3x1x_store_recv);
static DRIVER_ATTR(reg,     S_IWUSR | S_IRUGO, stk3x1x_show_reg,   NULL);
static DRIVER_ATTR(allreg,  S_IWUSR | S_IRUGO, stk3x1x_show_allreg,   NULL);
#ifdef STK_TUNE0
static DRIVER_ATTR(cali,    S_IWUSR | S_IRUGO, stk3x1x_show_cali,  NULL);
#endif
/*----------------------------------------------------------------------------*/
static struct driver_attribute *stk3x1x_attr_list[] = {
    &driver_attr_als,
    &driver_attr_ps,    
    &driver_attr_ir,    
    &driver_attr_trace,        /*trace log*/
    &driver_attr_config,
    &driver_attr_alslv,
    &driver_attr_alsval,
    &driver_attr_status,
    &driver_attr_send,
    &driver_attr_recv,
    &driver_attr_allreg,
//    &driver_attr_i2c,
    &driver_attr_reg,
#ifdef STK_TUNE0
    &driver_attr_cali,
#endif	
};

/*----------------------------------------------------------------------------*/
static int stk3x1x_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(stk3x1x_attr_list)/sizeof(stk3x1x_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, stk3x1x_attr_list[idx])))
		{            
			APS_ERR("driver_create_file (%s) = %d\n", stk3x1x_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
static int stk3x1x_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(stk3x1x_attr_list)/sizeof(stk3x1x_attr_list[0]));

	if (!driver)
	return -EINVAL;

	for (idx = 0; idx < num; idx++) 
	{
		driver_remove_file(driver, stk3x1x_attr_list[idx]);
	}
	
	return err;
}
/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static int stk3x1x_get_als_value(struct stk3x1x_priv *obj, u16 als)
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
static int stk3x1x_get_ps_value_only(struct stk3x1x_priv *obj, u16 ps)
{
	int mask = atomic_read(&obj->ps_mask);
	int invalid = 0, val;
	int err;
	u8 flag;

	err = stk3x1x_read_flag(obj->client, &flag);
	if(err)
		return err;
	val = (flag & STK_FLG_NF_MASK)? 1 : 0;	
	
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
		APS_ERR(" ps value is invalid, PS:  %05d => %05d\n", ps, val);
		if(unlikely(atomic_read(&obj->trace) & STK_TRC_CVT_PS))
		{
			APS_DBG("PS:  %05d => %05d (-1)\n", ps, val);    
		}
		return -1;
	}	
}

/*----------------------------------------------------------------------------*/
static int stk3x1x_get_ps_value(struct stk3x1x_priv *obj, u16 ps)
{
	int mask = atomic_read(&obj->ps_mask);
	int invalid = 0, val;
	int err;
	u8 flag;

	err = stk3x1x_read_flag(obj->client, &flag);
	if(err)
		return err;
	
	val = (flag & STK_FLG_NF_MASK)? 1 : 0;	
	if((err = stk3x1x_clear_intr(obj->client, flag, STK_FLG_OUI_MASK)))
	{
		APS_ERR("fail: %d\n", err);
		return err;
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
		APS_ERR(" ps value is invalid, PS:  %05d => %05d\n", ps, val);
		if(unlikely(atomic_read(&obj->trace) & STK_TRC_CVT_PS))
		{
			APS_DBG("PS:  %05d => %05d (-1)\n", ps, val);    
		}
		return -1;
	}	
}

/*----------------------------------------------------------------------------*/

static int32_t stk3x1x_set_irs_it_slp(struct stk3x1x_priv *obj, uint16_t *slp_time)
{
	uint8_t irs_alsctrl;
	int32_t ret;
		
	irs_alsctrl = (atomic_read(&obj->alsctrl_val) & 0x0F) - 2;		
	switch(irs_alsctrl)
	{
		case 6:
			*slp_time = 12;
			break;
		case 7:
			*slp_time = 24;			
			break;
		case 8:
			*slp_time = 48;			
			break;
		case 9:
			*slp_time = 96;			
			break;				
		default:
			printk(KERN_ERR "%s: unknown ALS IT=0x%x\n", __func__, irs_alsctrl);
			ret = -EINVAL;	
			return ret;
	}
	irs_alsctrl |= (atomic_read(&obj->alsctrl_val) & 0xF0);
	ret = i2c_smbus_write_byte_data(obj->client, STK_ALSCTRL_REG, irs_alsctrl);
	if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		return ret;		
	}		
	return 0;
}

static int32_t stk3x1x_get_ir_value(struct stk3x1x_priv *obj)
{
    int32_t word_data, ret;
	uint8_t w_reg, retry = 0;	
	uint16_t irs_slp_time = 100;
	bool re_enable_ps = false;
	u8 flag;
	u8 buf[2];
	
	re_enable_ps = (atomic_read(&obj->state_val) & STK_STATE_EN_PS_MASK) ? true : false;	
	if(re_enable_ps)
	{
#ifdef STK_TUNE0		
		if (!(obj->psi_set))
		{
			hrtimer_cancel(&obj->ps_tune0_timer);					
			cancel_work_sync(&obj->stk_ps_tune0_work);
		}		
#endif		
		stk3x1x_enable_ps(obj->client, 0);
	}
	
	ret = stk3x1x_set_irs_it_slp(obj, &irs_slp_time);
	if(ret < 0)
		goto irs_err_i2c_rw;
		
	w_reg = atomic_read(&obj->state_val) | STK_STATE_EN_IRS_MASK;		
	ret = i2c_smbus_write_byte_data(obj->client, STK_STATE_REG, w_reg);
	if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		goto irs_err_i2c_rw;
	}	
	msleep(irs_slp_time);	
	
	do
	{
		msleep(3);		
		ret = stk3x1x_read_flag(obj->client, &flag);	
		if (ret < 0)
		{
			APS_ERR("WARNING: read flag reg error: %d\n", ret);
			goto irs_err_i2c_rw;
		}	
		retry++;
	}while(retry < 10 && ((flag&STK_FLG_IR_RDY_MASK) == 0));
	
	if(retry == 10)
	{
		printk(KERN_ERR "%s: ir data is not ready for 300ms\n", __func__);
		ret = -EINVAL;
		goto irs_err_i2c_rw;
	}

	ret = stk3x1x_clear_intr(obj->client, flag, STK_FLG_IR_RDY_MASK);	
	if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		goto irs_err_i2c_rw;
	}		
	
	ret = stk3x1x_master_recv(obj->client, STK_DATA1_IR_REG, buf, 2);
	if(ret < 0)	
	{
		printk(KERN_ERR "%s fail, ret=0x%x", __func__, ret); 
		goto irs_err_i2c_rw;		
	}
	word_data =  (buf[0] << 8) | buf[1];

	ret = i2c_smbus_write_byte_data(obj->client, STK_ALSCTRL_REG, atomic_read(&obj->alsctrl_val));
	if (ret < 0)
	{
		printk(KERN_ERR "%s: write i2c error\n", __func__);
		goto irs_err_i2c_rw;
	}
	if(re_enable_ps)
		stk3x1x_enable_ps(obj->client, 1);		
	return word_data;

irs_err_i2c_rw:	
	if(re_enable_ps)
		stk3x1x_enable_ps(obj->client, 1);	
	return ret;
}

/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static int stk3x1x_open(struct inode *inode, struct file *file)
{
	file->private_data = stk3x1x_i2c_client;

	if (!file->private_data)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}
	
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int stk3x1x_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}
/*----------------------------------------------------------------------------*/
	
static long stk3x1x_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);  
	u8 int_val = 0;
	long err = 0;
	void __user *ptr = (void __user*) arg;
	int dat;
	uint32_t enable;

	struct set_ps_thd_para set_ps_thd_para;
	struct alsps_hw* hw = get_cust_alsps_hw();
	
	switch (cmd)
	{
		case ALSPS_SET_PS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
				if((err = stk3x1x_enable_ps(obj->client, 1)))
				{
					APS_ERR("enable ps fail: %ld\n", err); 

					goto err_out;
				}
				
				set_bit(STK_BIT_PS, &obj->enable);
			}
			else
			{
				if((err = stk3x1x_enable_ps(obj->client, 0)))
				{
					APS_ERR("disable ps fail: %ld\n", err); 
	
					goto err_out;
				}
				
				clear_bit(STK_BIT_PS, &obj->enable);
			}
            if (g_stk3x1x_ptr->fasync != NULL)
            {
                APS_LOG("kill a SIGIO when %s", __func__);
                kill_fasync(&g_stk3x1x_ptr->fasync, SIGIO, POLL_IN);
            }
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
			if((err = stk3x1x_read_ps(obj->client, &obj->ps)))
			{
				goto err_out;
			}
		
			dat = stk3x1x_get_ps_value(obj, obj->ps);
			if(dat < 0)
			{
				err = dat;
				goto err_out;
			}
		#else
			dat = g_is_far;
		#endif
#ifdef STK_PS_POLLING_LOG	
			APS_LOG("%s:ps raw 0x%x -> value 0x%x \n",__FUNCTION__, obj->ps, dat);			
#endif			
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;

		case ALSPS_GET_PS_RAW_DATA:    
			if((err = stk3x1x_read_ps(obj->client, &obj->ps)))
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

		case ALSPS_SET_ALS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
				if((err = stk3x1x_enable_als(obj->client, 1)))
				{
					APS_ERR("enable als fail: %ld\n", err); 

					goto err_out;
				}
				set_bit(STK_BIT_ALS, &obj->enable);
			}
			else
			{
				if((err = stk3x1x_enable_als(obj->client, 0)))
				{

					APS_ERR("disable als fail: %ld\n", err); 

					goto err_out;
				}
				clear_bit(STK_BIT_ALS, &obj->enable);
			}
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
			if((err = stk3x1x_read_als(obj->client, &obj->als)))
			{
				goto err_out;
			}

			dat = stk3x1x_get_als_value(obj, obj->als);
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;

		case ALSPS_GET_ALS_RAW_DATA:    
			if((err = stk3x1x_read_als(obj->client, &obj->als)))
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
            stk3x1x_set_ps_threshold(obj->client,
                    set_ps_thd_para.low_threshold, 
                    set_ps_thd_para.high_threshold);

		if (!g_first_thd){
			i2c_smbus_write_byte_data(client, obj->addr.intmode, int_val | STK_INT_PS_MODE1);
			g_first_thd = 1;
		}
			
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
		default:
			APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
			err = -ENOIOCTLCMD;
			break;
	}

	err_out:
	return err;    
}
/*----------------------------------------------------------------------------*/
static int stk3x1x_fasync(int fd, struct file * file, int on)
{
    APS_LOG("%s on : %d\n", __func__, on);
    return fasync_helper(fd, file, on, &g_stk3x1x_ptr->fasync);
}
/*----------------------------------------------------------------------------*/
static struct file_operations stk3x1x_fops = {
	.open = stk3x1x_open,
	.release = stk3x1x_release,
	.unlocked_ioctl = stk3x1x_unlocked_ioctl,
	.fasync = stk3x1x_fasync,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice stk3x1x_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &stk3x1x_fops,
};
/*----------------------------------------------------------------------------*/
static int stk3x1x_i2c_suspend(struct i2c_client *client, pm_message_t msg) 
{
	down(&ps_suspend_sem);
	int err;
	struct stk31xx_priv *obj = i2c_get_clientdata(client);
	printk(APS_TAG"%s  Enter !!!\n",__func__);

	if(msg.event == PM_EVENT_SUSPEND)
	{   
		if(!stk3x1x_obj)
		{
			APS_ERR("null pointer!!\n");
			up(&ps_suspend_sem);
			return -EINVAL;
		}

		atomic_set(&stk3x1x_obj->als_suspend, 1);

		if (test_bit(STK_BIT_PS,  &stk3x1x_obj->enable))
		{
			printk(APS_TAG"%s  PS is working  Exit !!!\n",__func__);
			up(&ps_suspend_sem);
			return 0;
		}

		if(test_bit(STK_BIT_ALS,  &stk3x1x_obj->enable)){
			if((err = stk3x1x_enable_als(client, 0)))
			{
				APS_ERR("disable als: %d\n", err);
				up(&ps_suspend_sem);
				return err;
			}
		}

		suspend_status = 1;
		atomic_set(&stk3x1x_obj->ps_suspend, 1);

		if((err = stk3x1x_enable_ps(client, 0)))
		{
			APS_ERR("disable ps:  %d\n", err);
			up(&ps_suspend_sem);
			return err;
		}

		stk3x1x_power(stk3x1x_obj->hw, 0);
	}
	up(&ps_suspend_sem);
	printk(APS_TAG"%s  Exit !!!\n",__func__);

	return 0;
}
/*----------------------------------------------------------------------------*/
static int stk3x1x_i2c_resume(struct i2c_client *client)
{
	down(&ps_suspend_sem);
	int err;
	struct stk31xx_priv *obj = i2c_get_clientdata(client);
	printk("%s  Enter !!!\n",__func__);

	if(!stk3x1x_obj)
	{
		APS_ERR("null pointer!!\n");
		up(&ps_suspend_sem);
		return -EINVAL;
	}

	atomic_set(&stk3x1x_obj->als_suspend, 0);

	if ((test_bit(STK_BIT_PS,  &stk3x1x_obj->enable)) || (suspend_status == 0)){
		printk("%s  PS is working ! not init\n",__func__);
		up(&ps_suspend_sem);
		return 0;
	}

	stk3x1x_power(stk3x1x_obj->hw, 1);
	if((err = stk3x1x_init_client(client)))
	{
		APS_ERR("initialize client fail!!\n");
		up(&ps_suspend_sem);
		return err;
	}

	atomic_set(&stk3x1x_obj->ps_suspend, 0);

	suspend_status = 0;

	if(test_bit(STK_BIT_ALS, &stk3x1x_obj->enable))
	{
		if((err = stk3x1x_enable_als(client, 1)))
		{
			APS_ERR("enable als fail: %d\n", err);        
		}
	}

	up(&ps_suspend_sem);
	printk("%s  Leave !!!\n",__func__);
	return 0;
}
/*----------------------------------------------------------------------------*/
static void stk3x1x_early_suspend(struct early_suspend *h) 
{   /*early_suspend is only applied for ALS*/
	int err;
#if 0
	struct stk3x1x_priv *obj = container_of(h, struct stk3x1x_priv, early_drv);   	
	int old = atomic_read(&obj->state_val);
	APS_FUN();    

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}
	
	if(old & STK_STATE_EN_ALS_MASK)
	{
		atomic_set(&obj->als_suspend, 1);    
		if((err = stk3x1x_enable_als(obj->client, 0)))
		{
			APS_ERR("disable als fail: %d\n", err); 
		}
	}
	    struct stk31xx_priv *obj = container_of(h, struct stk31xx_priv, early_drv);   
    int err;
#else
    pm_message_t msg;
    msg.event = PM_EVENT_SUSPEND;
    err = stk3x1x_i2c_suspend(stk3x1x_obj->client, msg);
#endif
}
/*----------------------------------------------------------------------------*/
static void stk3x1x_late_resume(struct early_suspend *h)
{   /*early_suspend is only applied for ALS*/
	int err;
#if 0
	hwm_sensor_data sensor_data;
	struct stk3x1x_priv *obj = container_of(h, struct stk3x1x_priv, early_drv);         
	
	memset(&sensor_data, 0, sizeof(sensor_data));
	APS_FUN();

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}
	if(atomic_read(&obj->als_suspend))
	{
		atomic_set(&obj->als_suspend, 0);
		if(test_bit(STK_BIT_ALS, &obj->enable))
		{
			if((err = stk3x1x_enable_als(obj->client, 1)))
			{
				APS_ERR("enable als fail: %d\n", err);        

			}
		}
	}
#else
    err = stk3x1x_i2c_resume(stk3x1x_obj->client);
#endif
}

int stk3x1x_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* sensor_data;
	struct stk3x1x_priv *obj = (struct stk3x1x_priv *)self;
	
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
					if((err = stk3x1x_enable_ps(obj->client, 1)))
					{
						APS_ERR("enable ps fail: %d\n", err); 
						return -1;
					}
					set_bit(STK_BIT_PS, &obj->enable);
				}
				else
				{
					if((err = stk3x1x_enable_ps(obj->client, 0)))
					{
						APS_ERR("disable ps fail: %d\n", err); 
						return -1;
					}
					clear_bit(STK_BIT_PS, &obj->enable);
				}
                if (g_stk3x1x_ptr->fasync != NULL)
                {
                    APS_LOG("kill a SIGIO when %s", __func__);
                    kill_fasync(&g_stk3x1x_ptr->fasync, SIGIO, POLL_IN);
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
				
				if((err = stk3x1x_read_ps(obj->client, &obj->ps)))
				{
					err = -1;
				}
				else
				{
					value = stk3x1x_get_ps_value(obj, obj->ps);
					if(value < 0)
					{
						err = -1;
					}
					else
					{
						sensor_data->values[0] = value;
						sensor_data->value_divide = 1;
						sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
#ifdef STK_PS_POLLING_LOG						
						APS_LOG("%s:ps raw 0x%x -> value 0x%x \n",__FUNCTION__, obj->ps, sensor_data->values[0]);					
#endif				
					}	
				}				
			}
			break;
		default:
			APS_ERR("proximity sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}

int stk3x1x_als_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* sensor_data;
	struct stk3x1x_priv *obj = (struct stk3x1x_priv *)self;
	u8 flag;
				
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
					if((err = stk3x1x_enable_als(obj->client, 1)))
					{
						APS_ERR("enable als fail: %d\n", err); 
						return -1;
					}
					set_bit(STK_BIT_ALS, &obj->enable);
				}
				else
				{
					if((err = stk3x1x_enable_als(obj->client, 0)))
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
				err = stk3x1x_read_flag(obj->client, &flag);
				if(err)
					return err;
				
				if(!(flag & STK_FLG_ALSDR_MASK))
					return -1;				
				
				sensor_data = (hwm_sensor_data *)buff_out;								
				if((err = stk3x1x_read_als(obj->client, &obj->als)))
				{
					err = -1;
				}
				else
				{
					sensor_data->values[0] = stk3x1x_get_als_value(obj, obj->als);
					sensor_data->value_divide = 1;
					sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
					printk("%s  als = %d\n",__func__,obj->als);
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

/*----------------------------------------------------------------------------*/
static int stk3x1x_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct stk3x1x_priv *obj;
	struct hwmsen_object obj_ps, obj_als;
	int err = 0;
	struct alsps_hw *hw_probe;
	hw_probe = get_cust_alsps_hw();
	printk("%s  Enter  !!!\n",__func__);

	printk("%s: driver version: %s\n", __FUNCTION__, DRIVER_VERSION);

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(obj, 0, sizeof(*obj));

	stk3x1x_obj = obj;
	
	obj->hw = get_cust_alsps_hw();
	stk3x1x_get_addr(obj->hw, &obj->addr);

	INIT_DELAYED_WORK(&obj->eint_work, stk3x1x_eint_work);
	obj->client = client;
	i2c_set_clientdata(client, obj);	
	atomic_set(&obj->als_debounce, 200);
	atomic_set(&obj->als_deb_on, 0);
	atomic_set(&obj->als_deb_end, 0);
	atomic_set(&obj->ps_debounce, 100);
	atomic_set(&obj->ps_deb_on, 0);
	atomic_set(&obj->ps_deb_end, 0);
	atomic_set(&obj->ps_mask, 0);
	atomic_set(&obj->trace, 0x00);
	atomic_set(&obj->als_suspend, 0);
#if 0
	atomic_set(&obj->state_val, obj->hw->state_val);
	atomic_set(&obj->psctrl_val, obj->hw->psctrl_val);
	atomic_set(&obj->alsctrl_val, obj->hw->alsctrl_val);
	obj->ledctrl_val = obj->hw->ledctrl_val;
	obj->wait_val = obj->hw->wait_val;
#else
	atomic_set(&obj->state_val, 0x20);
	atomic_set(&obj->psctrl_val, 0x31);
	atomic_set(&obj->alsctrl_val, 0x19);
	obj->ledctrl_val = 0xFF;
	obj->wait_val = 0x7;
#endif
	obj->int_val = 0;
	obj->first_boot = true;			 
	obj->als_correct_factor = 1000;
	
	atomic_set(&obj->recv_reg, 0);  
	
	if(obj->hw->polling_mode_ps == 0)
	{
		printk("%s:  PS mode is interrupt\n", __FUNCTION__);
	}
	//obj->int_val |= STK_INT_PS_MODE1;
	
	if(obj->hw->polling_mode_als == 0)
	{
		obj->int_val |= STK_INT_ALS;		
		printk("%s:  ALS mode is interrupt\n", __FUNCTION__);
	}	

	printk("%s: state_val=0x%x, psctrl_val=0x%x, alsctrl_val=0x%x, ledctrl_val=0x%x, wait_val=0x%x, int_val=0x%x\n", 
		__FUNCTION__, atomic_read(&obj->state_val), atomic_read(&obj->psctrl_val), atomic_read(&obj->alsctrl_val), 
		obj->ledctrl_val, obj->wait_val, obj->int_val);
	
	obj->enable = 0;
	obj->pending_intr = 0;
	obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
	obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);   
	BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
	memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
	BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
	memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
	atomic_set(&obj->i2c_retry, 1);
	if(atomic_read(&obj->state_val) & STK_STATE_EN_ALS_MASK)
	{
		set_bit(STK_BIT_ALS, &obj->enable);
	}
	
	if(atomic_read(&obj->state_val) & STK_STATE_EN_PS_MASK)
	{
		set_bit(STK_BIT_PS, &obj->enable);
	}
	
	stk3x1x_i2c_client = client;

#ifdef STK_TUNE0
	obj->stk_ps_tune0_wq = create_singlethread_workqueue("stk_ps_tune0_wq");
	INIT_WORK(&obj->stk_ps_tune0_work, stk_ps_tune0_work_func);
	hrtimer_init(&obj->ps_tune0_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	obj->ps_tune0_delay = ns_to_ktime(60 * NSEC_PER_MSEC);
	obj->ps_tune0_timer.function = stk_ps_tune0_timer_func;
#endif		
	if((err = stk3x1x_init_client(client)))
	{
		goto exit_init_failed;
	}
	
	if((err = misc_register(&stk3x1x_device)))
	{
		APS_ERR("stk3x1x_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	if((err = stk3x1x_create_attr(&stk3x1x_alsps_driver.driver)))
	{
		APS_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
	obj_ps.self = stk3x1x_obj;
	if(1 == obj->hw->polling_mode_ps)
	{
		obj_ps.polling = 1;
		wake_lock_init(&ps_lock,WAKE_LOCK_SUSPEND,"ps wakelock");
	}
	else
	{
		obj_ps.polling = 0;//PS interrupt mode
	}
	
	obj_ps.sensor_operate = stk3x1x_ps_operate;
	if((err = hwmsen_attach(ID_PROXIMITY, &obj_ps)))
	{
		printk("attach fail = %d\n", err);
		goto exit_create_attr_failed;
	}
	
	obj_als.self = stk3x1x_obj;
	if(1 == obj->hw->polling_mode_als)
	{
	  obj_als.polling = 1;
	}
	else
	{
	  obj_als.polling = 0;//ALS interrupt mode
	}
	obj_als.sensor_operate = stk3x1x_als_operate;
	if((err = hwmsen_attach(ID_LIGHT, &obj_als)))
	{
		printk("attach fail = %d\n", err);
		goto exit_create_attr_failed;
	}


#if defined(CONFIG_HAS_EARLYSUSPEND)
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend  = stk3x1x_early_suspend,
	obj->early_drv.resume   = stk3x1x_late_resume,    
	register_early_suspend(&obj->early_drv);
#endif
#ifdef VENDOR_EDIT /*zhye add a interface to control register*/
    //wake_lock_init(&ps_lock, WAKE_LOCK_SUSPEND, "ps_wakelock");
	hwmsen_make_debug_flag(&open_alsps_log, "open_alsps_log");
	hwmsen_make_debug_flag(&set_prox_reg, "set_prox_reg");
	hwmsen_make_debug_flag(&read_prox_reg, "read_prox_reg");
	hwmsen_make_debug_flag(&ps_switch_fops,"ps_switch");
    //hwmsen_make_debug_flag(&set_prox_thd, "set_prox_thd");
#endif/*VENDOR_EDIT*/
	printk("%s: OK\n", __FUNCTION__);

	return 0;

exit_create_attr_failed:
	misc_deregister(&stk3x1x_device);
exit_misc_device_register_failed:
exit_init_failed:
	//i2c_detach_client(client);
//	exit_kfree:
	//kfree(obj);
exit:
	stk3x1x_i2c_client = NULL;           

	APS_ERR("%s: err = %d\n", __FUNCTION__, err);
	stk3x1x_power(hw_probe, 0);
	kfree(obj);
	return err;
}
/*----------------------------------------------------------------------------*/
static int stk3x1x_i2c_remove(struct i2c_client *client)
{
	int err;	
#ifdef STK_TUNE0
	struct stk3x1x_priv *obj = i2c_get_clientdata(client);		
	destroy_workqueue(obj->stk_ps_tune0_wq);	
#endif		
	
	if((err = stk3x1x_delete_attr(&stk3x1x_i2c_driver.driver)))
	{
		APS_ERR("stk3x1x_delete_attr fail: %d\n", err);
	} 

	if((err = misc_deregister(&stk3x1x_device)))
	{
		APS_ERR("misc_deregister fail: %d\n", err);    
	}
	
	stk3x1x_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	return 0;
}
/*----------------------------------------------------------------------------*/
static int stk3x1x_probe(struct platform_device *pdev) 
{
	struct alsps_hw *hw = get_cust_alsps_hw();
	struct stk3x1x_i2c_addr addr;

	stk3x1x_power(hw, 1);    
	stk3x1x_get_addr(hw, &addr);

	if(i2c_add_driver(&stk3x1x_i2c_driver))
	{
		APS_ERR("add driver error\n");
		return -1;
	} 

	return 0;
}
/*----------------------------------------------------------------------------*/
static int stk3x1x_remove(struct platform_device *pdev)
{
	struct alsps_hw *hw = get_cust_alsps_hw();
	APS_FUN();    
	stk3x1x_power(hw, 0);    
	i2c_del_driver(&stk3x1x_i2c_driver);
	return 0;
}
/*----------------------------------------------------------------------------*/
static struct platform_driver stk3x1x_alsps_driver = {
	.probe      = stk3x1x_probe,
	.remove     = stk3x1x_remove,    
	.driver     = {
		.name  = "als_ps_stk3x1x",
	}
};
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static int __init stk3x1x_init(void)
{
	struct alsps_hw *hw = get_cust_alsps_hw();
	APS_LOG("%s: i2c_number=%d\n", __func__,hw->i2c_num); 
	i2c_register_board_info(0, &i2c_stk3x1x, 1);	

	APS_FUN();
	if(platform_driver_register(&stk3x1x_alsps_driver))
	{
		APS_ERR("failed to register driver\n");
		return -ENODEV;
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit stk3x1x_exit(void)
{
	APS_FUN();
	platform_driver_unregister(&stk3x1x_alsps_driver);
}
/*----------------------------------------------------------------------------*/
module_init(stk3x1x_init);
module_exit(stk3x1x_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("MingHsien Hsieh");
MODULE_DESCRIPTION("SensorTek stk3x1x proximity and light sensor driver");
MODULE_LICENSE("GPL");
