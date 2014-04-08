/*
 * Copyright (C) 2010 MEMSIC, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */ 
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/mm.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/sysctl.h>
#include <linux/input-polldev.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/earlysuspend.h>
//#include <linux/i2c/mmc3416x.h>
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <cust_mag.h>
#include "mmc3416x.h"
#include <linux/hwmsen_helper.h>
#include <linux/time.h>
#include <linux/hrtimer.h>
#define DEBUG			1
#define MMC3416x_DEBUG		1
//#define MMC3416x_DEBUG_MSG	1
//#define MMC3416x_DEBUG_FUNC	1
#define MMC3416x_DEBUG_DATA	1
#define MAX_FAILURE_COUNT	3
#define READMD			0

#define MMC3416X_DELAY_TM	10	/* ms */
#define MMC3416X_DELAY_SET	50	/* ms */
#define MMC3416X_DELAY_RESET     50     /* ms */
#define MMC3416X_DEFAULT_DELAY   100
#define MMC3416X_RETRY_COUNT	3
#define MMC3416X_SET_INTV	250
#define	MMC3416x_BUFSIZE  0x20
#define MMC3416X_RESET_INTV 10
#define MMC3416X_DEV_NAME	"mmc3416x"

#if MMC3416x_DEBUG_MSG
#define MMCDBG(format, ...)	printk( "mmc3416x " format "\n", ## __VA_ARGS__)
#else
#define MMCDBG(format, ...)
#endif

#if MMC3416x_DEBUG_FUNC
#define MMCFUNC(func) printk( "mmc3416x " func " is called\n")
#else
#define MMCFUNC(func)
#endif

    #if  0
		#define MSEN_DBG(x...) printk(x)
	#else
		#define MSEN_DBG(x...) 
	#endif 

#ifdef OPPO_R819//mingqiang.guo@Prd.BasicDrv.Sensor, add 2012/10/15 for mult project sensor
#include "mmc3416x_private.h"
#endif /* OPPO_R819*/

struct mmc3416x_i2c_data {
    struct i2c_client *client;
    struct mag_hw *hw; 
    atomic_t layout;   
    atomic_t trace;
	struct hwmsen_convert   cvt;
#if defined(CONFIG_HAS_EARLYSUSPEND)    
    struct early_suspend    early_drv;
#endif 
#ifdef OPPO_R819//mingqiang.guo@BasicDrv.Sensor, add 2012/6/6 for msensor power off
    atomic_t suspend;
    atomic_t enable_before_resume; 
#endif /*OPPO_R819*/
};

static u32 read_idx = 0;
struct class *mag_class;
static struct i2c_client *this_client = NULL;

static struct input_polled_dev *ipdev;
static struct mutex lock;
static const struct i2c_device_id mmc3416x_i2c_id[] = {{MMC3416X_DEV_NAME,0},{}};
static struct i2c_board_info __initdata i2c_mmc3416x={ I2C_BOARD_INFO("mmc3416x", (0x60>>1))};
// calibration msensor and orientation data
#define CALIBRATION_DATA_SIZE 12
static int sensor_data[CALIBRATION_DATA_SIZE];
static struct mutex sensor_data_mutex;
static struct mutex read_i2c_xyz;
static DECLARE_WAIT_QUEUE_HEAD(data_ready_wq);
static DECLARE_WAIT_QUEUE_HEAD(open_wq);

static int I2C_RxData(char *rxData, int length);
static int I2C_TxData(char *txData, int length);

static int mmcd_delay = MMC3416X_DEFAULT_DELAY;

static atomic_t open_flag = ATOMIC_INIT(0);
static atomic_t m_flag = ATOMIC_INIT(0);
static atomic_t o_flag = ATOMIC_INIT(0);
static short ecompass_delay = 0;


typedef enum {
    MMC_FUN_DEBUG  = 0x01,
	MMC_DATA_DEBUG = 0X02,
	MMC_HWM_DEBUG  = 0X04,
	MMC_CTR_DEBUG  = 0X08,
	MMC_I2C_DEBUG  = 0x10,
} MMC_TRC;

static int platform_mmc3416x_probe(struct platform_device *pdev) ;
static int platform_mmc3416x_remove(struct platform_device *pdev) ;
static struct platform_driver mmc3416x_platform_driver = {
   .probe  = platform_mmc3416x_probe,
   .remove = platform_mmc3416x_remove,
   .driver = {
        .name = "msensor_mmc3416x",
   }

};
static DEFINE_MUTEX(ecompass_lock);

#ifdef OPPO_R819//Shaoyu.Huang@BadicDrv.Sensor, add 2012/6/4 for sensor debuging
/*----------------------------------------------------------------------------*/

static debugflag = 0;
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

static int I2C_RxData(char *rxData, int length)
{
	uint8_t loop_i;
	int i;
	struct i2c_client *client = this_client;  
	struct mmc3416x_i2c_data *data = i2c_get_clientdata(client);
	char addr = rxData[0];
	/* Caller should check parameter validity.*/
	if((rxData == NULL) || (length < 1)){
		return -EINVAL;
	}

	for(loop_i = 0; loop_i < MMC3416X_RETRY_COUNT; loop_i++){
		this_client->addr = this_client->addr & I2C_MASK_FLAG | I2C_WR_FLAG;
		if(i2c_master_send(this_client, (const char*)rxData, ((length<<0X08) | 0X01))){
			break;
		}
		printk("I2C_RxData delay!\n");
		mdelay(10);
	}
	
	if(loop_i >= MMC3416X_RETRY_COUNT){
		printk(KERN_ERR "%s retry over %d\n", __func__, MMC3416X_RETRY_COUNT);
		return -EIO;
	}
	if(atomic_read(&data->trace) & MMC_I2C_DEBUG){
		printk(KERN_INFO "RxData: len=%02x, addr=%02x\n  data=", length, addr);
		for(i = 0; i < length; i++){
			printk(KERN_INFO " %02x", rxData[i]);
		}
	    printk(KERN_INFO "\n");
	}
	return 0;
}

static int I2C_TxData(char *txData, int length)
{
	uint8_t loop_i;	
	int i;
	struct i2c_client *client = this_client;  
	struct mmc3416x_i2c_data *data = i2c_get_clientdata(client);
	/* Caller should check parameter validity.*/
	if ((txData == NULL) || (length < 2)){
		return -EINVAL;
	}

	this_client->addr = this_client->addr & I2C_MASK_FLAG;
	for(loop_i = 0; loop_i < MMC3416X_RETRY_COUNT; loop_i++){
		if(i2c_master_send(this_client, (const char*)txData, length) > 0){
			break;
		}
		printk("I2C_TxData delay!\n");
		mdelay(10);
	}
	
	if(loop_i >= MMC3416X_RETRY_COUNT) {
		printk(KERN_ERR "%s retry over %d\n", __func__, MMC3416X_RETRY_COUNT);
		return -EIO;
	}
	if(atomic_read(&data->trace) & MMC_I2C_DEBUG){
		printk(KERN_INFO "TxData: len=%02x, addr=%02x\n  data=", length, txData[0]);
		for(i = 0; i < (length-1); i++){
			printk(KERN_INFO " %02x", txData[i + 1]);
		}
		printk(KERN_INFO "\n");
	}
	return 0;
}

#endif/*OPPO_R819*/
static int mmc3xxx_i2c_rx_data(char *buf, int len)
{

	return I2C_RxData(buf,len);

#if 0	
	uint8_t i;
	struct i2c_msg msgs[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= buf,
		},
		{
			.addr	= this_client->addr,
			.flags	= I2C_M_RD,
			.len	= len,
			.buf	= buf,
		}
	};

	for (i = 0; i < MMC3416X_RETRY_COUNT; i++) {
		if (i2c_transfer(this_client->adapter, msgs, 2) >= 0) {
			break;
		}
		mdelay(10);
	}

	if (i >= MMC3416X_RETRY_COUNT) {
		pr_err("%s: retry over %d\n", __FUNCTION__, MMC3416X_RETRY_COUNT);
		return -EIO;
	}

	return 0;
#endif	
}

static int mmc3xxx_i2c_tx_data(char *buf, int len)
{

	return I2C_TxData(buf,len);
#if 0
	uint8_t i;
	struct i2c_msg msg[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= len,
			.buf	= buf,
		}
	};
	
	for (i = 0; i < MMC3416X_RETRY_COUNT; i++) {
		if (i2c_transfer(this_client->adapter, msg, 1) >= 0) {
			break;
		}
		mdelay(10);
	}

	if (i >= MMC3416X_RETRY_COUNT) {
		pr_err("%s: retry over %d\n", __FUNCTION__, MMC3416X_RETRY_COUNT);
		return -EIO;
	}
	return 0;
#endif
}

static int mmc3416x_dev_init(struct i2c_client *client)
{
	u8 data[2];
	
	
//	MMC_FUN();
	data[0] = MMC3416X_REG_CTRL;
	data[1] = MMC3416X_CTRL_REFILL;
	if (mmc3xxx_i2c_tx_data( data, 2) < 0) {
		return MMC3416X_I2C_ERR;
	}
	msleep(MMC3416X_DELAY_SET);

	data[0] = MMC3416X_REG_CTRL;
	data[1] = MMC3416X_CTRL_SET;
	if (mmc3xxx_i2c_tx_data(  data, 2) < 0) {
		return MMC3416X_I2C_ERR;
	}
	msleep(MMC3416X_I2C_DELAY);
	
	data[0] = MMC3416X_REG_CTRL;
	data[1] = 0;
	if (mmc3xxx_i2c_tx_data( data, 2) < 0) {
		return MMC3416X_I2C_ERR;
	}
	msleep(MMC3416X_I2C_DELAY);

	data[0] = MMC3416X_REG_CTRL;
	data[1] = MMC3416X_CTRL_REFILL;
	if (mmc3xxx_i2c_tx_data(  data, 2) < 0) {
		return MMC3416X_I2C_ERR;
	}
	msleep(MMC3416X_DELAY_RESET);
	data[0] = MMC3416X_REG_CTRL;
	data[1] = MMC3416X_CTRL_RESET;
	if (mmc3xxx_i2c_tx_data(  data, 2) < 0) {
		return MMC3416X_I2C_ERR;
	}
	msleep(MMC3416X_I2C_DELAY);
	data[0] = MMC3416X_REG_CTRL;
	data[1] = 0;
	if (mmc3xxx_i2c_tx_data( data, 2) < 0) {
		return MMC3416X_I2C_ERR;
	}
	msleep(MMC3416X_I2C_DELAY);

	data[0] = MMC3416X_REG_BITS;
	data[1] = MMC3416X_BITS_SLOW_16;
	if (mmc3xxx_i2c_tx_data(  data, 2) < 0) {
		return MMC3416X_I2C_ERR;
	}
	msleep(MMC3416X_I2C_DELAY);

	data[0] = MMC3416X_REG_CTRL;
	data[1] = MMC3416X_CTRL_TM;
	if (mmc3xxx_i2c_tx_data( data, 2) < 0) {
		return MMC3416X_I2C_ERR;
	}
	msleep(MMC3416X_DELAY_TM);

	return MMC3416X_SUCCESS;
}



// Daemon application save the data
static int ECS_SaveData(int buf[12])
{
#if DEBUG	
	struct i2c_client *client = this_client;  
	struct mmc3416x_i2c_data *data = i2c_get_clientdata(client);
#endif

	mutex_lock(&sensor_data_mutex);
	memcpy(sensor_data, buf, sizeof(sensor_data));	
	mutex_unlock(&sensor_data_mutex);
	
#if DEBUG
	if(atomic_read(&data->trace) & MMC_HWM_DEBUG)
	{
		MMCDBG("Get daemon data: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d!\n",
			sensor_data[0],sensor_data[1],sensor_data[2],sensor_data[3],
			sensor_data[4],sensor_data[5],sensor_data[6],sensor_data[7],
			sensor_data[8],sensor_data[9],sensor_data[10],sensor_data[11]);
	}	
#endif

	return 0;

}

static int ECS_ReadXYZData(int *vec, int size)
{
	unsigned char data[6] = {0,0,0,0,0,0};
	ktime_t expires;
	int wait_n=0;
	int MD_times = 0;
	static int last_data[3];
	struct timespec time1, time2, time3,time4,delay,aa;	
	struct i2c_client *client = this_client;  
	struct mmc3416x_i2c_data *clientdata = i2c_get_clientdata(client);

//set_current_state(TASK_INTERRUPTIBLE);

    time1 = current_kernel_time();    
	if (vec == NULL || size < 3) {
		return -1;
	}
	mutex_lock(&read_i2c_xyz);
    time2 = current_kernel_time();

	if (!(read_idx % MMC3416X_SET_INTV)) {
			data[0] = MMC3416X_REG_CTRL;
		    data[1] = MMC3416X_CTRL_REFILL;
		    mmc3xxx_i2c_tx_data(data, 2);
		    msleep(MMC3416X_DELAY_SET);
		    data[0] = MMC3416X_REG_CTRL;
		    data[1] = MMC3416X_CTRL_SET;
		    mmc3xxx_i2c_tx_data(data, 2);
		    msleep(1);
		    data[0] = MMC3416X_REG_CTRL;
		    data[1] = 0;
		    mmc3xxx_i2c_tx_data(data, 2);
		    msleep(1);

            data[0] = MMC3416X_REG_CTRL;
            data[1] = MMC3416X_CTRL_REFILL;
            mmc3xxx_i2c_tx_data(data, 2);
            msleep(MMC3416X_DELAY_RESET);
            data[0] = MMC3416X_REG_CTRL;
            data[1] = MMC3416X_CTRL_RESET;
            mmc3xxx_i2c_tx_data(data, 2);
            msleep(1);
            data[0] = MMC3416X_REG_CTRL;
            data[1] = 0;
            mmc3xxx_i2c_tx_data(data, 2);
            msleep(1); 
	
	}
	time3 = current_kernel_time();
	/* send TM cmd before read */
	data[0] = MMC3416X_REG_CTRL;
	data[1] = MMC3416X_CTRL_TM;
	/* not check return value here, assume it always OK */
	if (I2C_TxData(data, 2) ){
        msleep(MMC3416X_I2C_DELAY);		
		data[0] = MMC3416X_REG_CTRL;
		data[1] = MMC3416X_CTRL_TM;		
		if ( I2C_TxData( data, 2)) {
			printk("write MMC3416X_CTRL_TM failed!\n");
			mutex_unlock(&read_i2c_xyz);
			return MMC3416X_I2C_ERR;
		}

    
	}
	msleep(MMC3416X_DELAY_TM);
	
	/* Read MD */
		data[0] = MMC3416X_REG_DS;
		I2C_RxData(data, 1);
		while (!(data[0] & 0x01)) {
			msleep(1);
			/* Read MD again*/
			data[0] = MMC3416X_REG_DS;
			I2C_RxData(data, 1);
			if (data[0] & 0x01) break;
			MD_times++;
			if (MD_times > 3) {	
				printk("TM not work!!");
				mutex_unlock(&read_i2c_xyz);
				return -EFAULT;
			}
		}
	read_idx++;
	data[0] = MMC3416X_REG_DATA;
	if(I2C_RxData(data, 6) < 0){
		mutex_unlock(&read_i2c_xyz);
		return -EFAULT;
	}
	vec[0] = data[1] << 8 | data[0];
	vec[1] = data[3] << 8 | data[2];
	vec[2] = data[5] << 8 | data[4];
	//vec[2] = 65536 - vec[2];

	if(atomic_read(&clientdata->trace) & MMC_DATA_DEBUG){
		//MMCDBG("[X - %04x] [Y - %04x] [Z - %04x]\n", vec[0], vec[1], vec[2]);
		MMCDBG("[X - %d] [Y - %d] [Z - %d]\n", vec[0], vec[1], vec[2]);
	}			
	mutex_unlock(&read_i2c_xyz);
	last_data[0] = vec[0];
	last_data[1] = vec[1];
	last_data[2] = vec[2];
	return 0;
}

static int ECS_GetRawData(int data[3])
{
	int err = 0;
	err = ECS_ReadXYZData(data, 3);
	if(err !=0 )
	{
		printk(KERN_ERR "MMC328x_IOC_TM failed\n");
		return -1;
	}

	// sensitivity 512 count = 1 Guass = 100uT
	#if 1
	data[0] = (data[0] - MMC3416X_OFFSET_X) * 100 / MMC3416X_SENSITIVITY_X;
	data[1] = (data[1] - MMC3416X_OFFSET_X) * 100 / MMC3416X_SENSITIVITY_X;
	data[2] = (data[2] - MMC3416X_OFFSET_X) * 100 / MMC3416X_SENSITIVITY_X;
    #endif
	return err;
}

static int ECS_GetOpenStatus(void)
{
	wait_event_interruptible(open_wq, (atomic_read(&open_flag) != 0));
	return atomic_read(&open_flag);
}

void mmc3416_power(struct mag_hw *hw, unsigned int on) 
{
   // static unsigned int power_on = 0;  
    hw->power(hw,on, NULL);
}



/*********************************************/
static ssize_t mmc3416x_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	sprintf(buf, "MMC3416X");
	ret = strlen(buf) + 1;
	return ret;
}

static DEVICE_ATTR(mmc3416x, S_IRUGO, mmc3416x_show, NULL);

/*----------------------------------------------------------------------------*/
static int mmc3416x_ReadChipInfo(char *buf, int bufsize)
{
	if((!buf)||(bufsize <= MMC3416x_BUFSIZE -1)){
		return -1;
	}
	if(!this_client){
		*buf = 0;
		return -2;
	}
	sprintf(buf, "mmc3416x Chip");
	return 0;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	char strbuf[MMC3416x_BUFSIZE];
	memset(strbuf,0,MMC3416x_BUFSIZE);
	mmc3416x_ReadChipInfo(strbuf, MMC3416x_BUFSIZE);
	return sprintf(buf, "%s\n", strbuf);        
}
/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	int sensordata[3];
	char strbuf[MMC3416x_BUFSIZE];
	
	ECS_GetRawData(sensordata);	
	sprintf(strbuf, "%d %d %d\n", sensordata[0],sensordata[1],sensordata[2]);
	return sprintf(buf, "%s\n", strbuf);
}
/*----------------------------------------------------------------------------*/
static ssize_t show_posturedata_value(struct device_driver *ddri, char *buf)
{
	int tmp[3];
	char strbuf[MMC3416x_BUFSIZE];
	tmp[0] = sensor_data[0] * CONVERT_O / CONVERT_O_DIV;				
	tmp[1] = sensor_data[1] * CONVERT_O / CONVERT_O_DIV;
	tmp[2] = sensor_data[2] * CONVERT_O / CONVERT_O_DIV;
	sprintf(strbuf, "%d, %d, %d\n", tmp[0],tmp[1], tmp[2]);		
	return sprintf(buf, "%s\n", strbuf);;           
}

/*----------------------------------------------------------------------------*/
static ssize_t show_direction_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = this_client;  
	struct mmc3416x_i2c_data *data = i2c_get_clientdata(client);
	return sprintf(buf, "(%d, %d)\n[%+2d %+2d %+2d]\n[%+2d %+2d %+2d]\n",
		data->hw->direction,atomic_read(&data->layout),	data->cvt.sign[0], data->cvt.sign[1],
		data->cvt.sign[2],data->cvt.map[0], data->cvt.map[1], data->cvt.map[2]);            
}
/*----------------------------------------------------------------------------*/
static ssize_t store_direction_value(struct device_driver *ddri, char *buf, size_t count)
{
	struct i2c_client *client = this_client;  
	struct mmc3416x_i2c_data *data = i2c_get_clientdata(client);
	int layout = 0;

	if(1 == sscanf(buf, "%d", &layout)){
		atomic_set(&data->layout, layout);
		if(!hwmsen_get_convert(layout, &data->cvt)){
			printk(KERN_ERR "HWMSEN_GET_CONVERT function error!\r\n");
		} else if(!hwmsen_get_convert(data->hw->direction, &data->cvt)){
			printk(KERN_ERR "invalid layout: %d, restore to %d\n", layout, data->hw->direction);
		} else {
			printk(KERN_ERR "invalid layout: (%d, %d)\n", layout, data->hw->direction);
			hwmsen_get_convert(0, &data->cvt);
		}
	} else {
		printk(KERN_ERR "invalid format = '%s'\n", buf);
	}
	
	return count;            
}
/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = this_client;  
	struct mmc3416x_i2c_data *data = i2c_get_clientdata(client);
	ssize_t len = 0;

	if(data->hw){
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d %d (%d %d)\n", 
			data->hw->i2c_num, data->hw->direction, data->hw->power_id, data->hw->power_vol);
	} else {
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}
	
	//len += snprintf(buf+len, PAGE_SIZE-len, "OPEN: %d\n", atomic_read(&dev_open_count));
	return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct mmc3416x_i2c_data *obj = i2c_get_clientdata(this_client);
	if(NULL == obj){
		printk(KERN_ERR "mmc328x_i2c_data is null!!\n");
		return 0;
	}		
	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));     
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri, char *buf, size_t count)
{
	struct mmc3416x_i2c_data *obj = i2c_get_clientdata(this_client);
	int trace;
	if(NULL == obj){
		printk(KERN_ERR "mmc328x_i2c_data is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "0x%x", &trace)) {
		atomic_set(&obj->trace, trace);
	} else {
		printk(KERN_ERR "invalid content: '%s', length = %d\n", buf, count);
	}
	
	return count;    
}
static ssize_t show_daemon_name(struct device_driver *ddri, char *buf)
{
	char strbuf[MMC3416x_BUFSIZE];
	sprintf(strbuf, "memsicd");
	return sprintf(buf, "%s", strbuf);		
}
#ifdef OPPO_R819//mingqiang.guo@BasicDrv.Sensor, add 2012/6/6 for msesensor auto test
#define abs(a) (((a) < 0) ? -(a) : (a))
int data_initial[3] = {0, 0, 0};
int data_magnet_close[3] = {0, 0, 0};
int data_magnet_leave[3] = {0, 0, 0};
static ssize_t show_autotest_testID(struct device_driver *ddri, char *buf)
{
	unsigned char id[2];
	unsigned char data[2];
	/*check device ID*/

#if 0
	/*SET*/
	data[0] = MMC3416X_REG_CTRL;
	data[1] = MMC3416X_CTRL_RM;
	if (I2C_TxData(data, 2) < 0)
		goto RESET_DEV_FAIL;
	msleep(MMC3416X_DELAY_SET);

	/*Enable*/
	data[0] = MMC3416X_CTRL_TM;
	data[1] = MMC3416X_REG_CTRL;
	if (I2C_TxData(data, 2) < 0)
		goto RESET_DEV_FAIL;
	msleep(MMC3416X_DELAY_SET);
	/*Reset*/
	data[0] = MMC3416X_REG_CTRL;
	data[1] = MMC3416X_CTRL_RESET;
	if (I2C_TxData(data, 2) < 0)
		goto RESET_DEV_FAIL;
	msleep(MMC3416X_DELAY_SET);
	/*Enable again*/
	data[0] = MMC3416X_CTRL_TM;
	data[1] = MMC3416X_REG_CTRL;
	if (I2C_TxData(data, 2) < 0)
		goto RESET_DEV_FAIL;
	msleep(MMC3416X_DELAY_SET);
#endif
    //mmc3416x_dev_init(NULL);
	/*Read initial data*/
	ECS_ReadXYZData(data_initial, 3);

	/*Data availble??*/
	if ((65535 == data_initial[0])
		|| (65535 == data_initial[1])
		|| (65535 == data_initial[2]))
		goto READ_DATA_FAIL;

    copy_to_user(buf,data_initial,sizeof(data_initial) );
	printk(KERN_ERR"%d, %d, %d\n", data_initial[0], data_initial[1], data_initial[2]);
	return 0;


READ_ID_FAIL:
	printk(KERN_ERR"Auto Test: read id error!!");
	printk(KERN_ERR"%d, %d", id[0], id[1]);	
	return -1;
RESET_DEV_FAIL:
	printk(KERN_ERR"Auto Test: reset device error!!");
	return -1;
READ_DATA_FAIL:
	printk(KERN_ERR"Auto Test: read data error!!");

}
static ssize_t show_autotest_magnetclose(struct device_driver *ddri, char *buf)
{
	int i ;
	/*50 loops which take about 5 seconds*/
	for (i = 0; i < 50; i ++)
	{
		if (ECS_ReadXYZData(data_magnet_close, 3))
		{
			MMCDBG(KERN_ERR"Auto Test:close read data fail");
			break;
		}

		/*Data availble??*/
		if ((65535 == data_magnet_close[0])
				|| (65535 == data_magnet_close[1])
				|| (65535 == data_magnet_close[2]))
		{
			MMCDBG(KERN_ERR"Auto Test:close read data fail");
			break;
		}
		MMCDBG(KERN_ERR"%d, %d, %d\n", data_magnet_close[0], data_magnet_close[1], data_magnet_close[2]);

		if ((abs(data_magnet_close[0] - data_initial[0]) >= 50)
			&& (abs(data_magnet_close[1] - data_initial[1]) >= 50)
			&& (abs(data_magnet_close[2] - data_initial[2]) >= 50))
			return 0;
		msleep(100);

	}
	return -1;

}
static ssize_t show_autotest_magnetleave(struct device_driver *ddri, char *buf)
{
	int i ;
	unsigned char data[2];
	return 0;
}

static ssize_t show_autotest_get_ic_mode(struct device_driver *ddri, char *buf)	
{
	//buf="mmc3416x";
	printk("msensor is mmc416x\n");
	return 0;
}
#endif/*OPPO_R819*/
/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(daemon,      S_IRUGO, show_daemon_name, NULL);
static DRIVER_ATTR(chipinfo,    S_IRUGO, show_chipinfo_value, NULL);
static DRIVER_ATTR(sensordata,  S_IRUGO, show_sensordata_value, NULL);
static DRIVER_ATTR(posturedata, S_IRUGO, show_posturedata_value, NULL);
static DRIVER_ATTR(direction,      S_IRUGO | S_IWUSR, show_direction_value, store_direction_value );
static DRIVER_ATTR(status,      S_IRUGO, show_status_value, NULL);
static DRIVER_ATTR(trace,       S_IRUGO | S_IWUSR, show_trace_value, store_trace_value);
#ifdef OPPO_R819//mingqiang.guo@BasicDrv.Sensor, add 2012/6/6 for msensor auto test
static DRIVER_ATTR(test_id,       S_IRUGO, show_autotest_testID, NULL);
static DRIVER_ATTR(magnet_close,  S_IRUGO, show_autotest_magnetclose, NULL);
static DRIVER_ATTR(magnet_leave,  S_IRUGO, show_autotest_magnetleave, NULL);
static DRIVER_ATTR(get_ic_modle,  S_IRUGO, show_autotest_get_ic_mode, NULL);
#endif/*OPPO_R819*/
/*----------------------------------------------------------------------------*/
static struct driver_attribute *mmc3416x_attr_list[] = {
    &driver_attr_daemon,
	&driver_attr_chipinfo,
	&driver_attr_sensordata,
	&driver_attr_posturedata,
	&driver_attr_direction,
	&driver_attr_status,
	&driver_attr_trace,
#ifdef OPPO_R819//mingqiang.guo@BasicDrv.Sensor, add 2012/6/6 for msensor auto test
	&driver_attr_test_id,
	&driver_attr_magnet_close,
	&driver_attr_magnet_leave,
	&driver_attr_get_ic_modle,	
#endif/*OPPO_R819*/
};
/*----------------------------------------------------------------------------*/
static int mmc3416x_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(mmc3416x_attr_list)/sizeof(mmc3416x_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if(err = driver_create_file(driver, mmc3416x_attr_list[idx]))
		{            
			printk(KERN_ERR "driver_create_file (%s) = %d\n", mmc3416x_attr_list[idx]->attr.name, err);
			break;
		}
	}
	
	return err;
}
/*----------------------------------------------------------------------------*/
static int mmc3416x_delete_attr(struct device_driver *driver)
{
	int idx;
	int num = (int)(sizeof(mmc3416x_attr_list)/sizeof(mmc3416x_attr_list[0]));

	if(driver == NULL)
	{
		return -EINVAL;
	}
	

	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, mmc3416x_attr_list[idx]);
	}
	

	return 0;
}

/******************************************/

static int mmc3416x_open(struct inode *inode, struct file *file)
{
    printk("mmc3416x_open !!\n");
	return nonseekable_open(inode, file);
}

static int mmc3416x_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long mmc3416x_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *pa = (void __user *)arg;
	int __user *pa_i = (void __user *)arg;
	unsigned char data[16] = {0};
	char buff[MMC3416x_BUFSIZE];
	int vec[3] = {0};
	int reg;
	int value[12];			/* for SET_YPR */
	int status;
	short sensor_status;		/* for Orientation and Msensor status */
	short flag;
	short delay;
	hwm_sensor_data* osensor_data;
	uint32_t enable;
	struct i2c_client *client = this_client;  
	struct mmc3416x_i2c_data *clientdata = i2c_get_clientdata(client);
	
	
    //MMCDBG("mmc3416x_ioctl enter! cmd=%x\n",cmd);
	//mutex_lock(&ecompass_lock);
	//MMCDBG("mmc3416x_ioctl_enter lock \n");
	switch (cmd) {
	case MMC31XX_IOC_TM:
	    MMCDBG("MMC3416X_IOC_TM \n");
		data[0] = MMC3416X_REG_CTRL;
		data[1] = MMC3416X_CTRL_TM;
		if (mmc3xxx_i2c_tx_data(data, 2) < 0) {
	        //mutex_unlock(&ecompass_lock);
			return -EFAULT;
		}
		/* wait TM done for coming data read */
		msleep(MMC3416X_DELAY_TM);
		break;
	case MMC31XX_IOC_SET:
	    MMCDBG("MMC3416X_IOC_SET \n");
		data[0] = MMC3416X_REG_CTRL;
		data[1] = MMC3416X_CTRL_REFILL;
		if (mmc3xxx_i2c_tx_data(data, 2) < 0) {
	        //mutex_unlock(&ecompass_lock);
			return -EFAULT;
		}
		msleep(MMC3416X_DELAY_SET);
		data[0] = MMC3416X_REG_CTRL;
		data[1] = MMC3416X_CTRL_SET;
		if (mmc3xxx_i2c_tx_data(data, 2) < 0) {
	        //mutex_unlock(&ecompass_lock);
			return -EFAULT;
		}
		msleep(1);
		data[0] = MMC3416X_REG_CTRL;
		data[1] = 0;
		if (mmc3xxx_i2c_tx_data(data, 2) < 0) {
	       // mutex_unlock(&ecompass_lock);
			return -EFAULT;
		}
			msleep(MMC3416X_I2C_DELAY);
		break;
	case MMC31XX_IOC_RESET:
	    MMCDBG("MMC3416X_IOC_RESET \n");
		data[0] = MMC3416X_REG_CTRL;
		data[1] = MMC3416X_CTRL_REFILL;
		if (mmc3xxx_i2c_tx_data(data, 2) < 0) {
	       // mutex_unlock(&ecompass_lock);
			return -EFAULT;
		}
		msleep(MMC3416X_DELAY_RESET);
		data[0] = MMC3416X_REG_CTRL;
		data[1] = MMC3416X_CTRL_RESET;
		if (mmc3xxx_i2c_tx_data(data, 2) < 0) {
	       // mutex_unlock(&ecompass_lock);
			return -EFAULT;
		}
			msleep(MMC3416X_I2C_DELAY);
		data[0] = MMC3416X_REG_CTRL;
		data[1] = 0;
		if (mmc3xxx_i2c_tx_data(data, 2) < 0) {
	        //mutex_unlock(&ecompass_lock);
			return -EFAULT;
		}
			msleep(MMC3416X_I2C_DELAY);
		break;
	case MMC31XX_IOC_READ:
	    MMCDBG("MMC3416X_IOC_READ \n");
		data[0] = MMC3416X_REG_DATA;
		if (mmc3xxx_i2c_rx_data(data, 6) < 0) {
	       // mutex_unlock(&ecompass_lock);
			return -EFAULT;
		}
		vec[0] = data[1] << 8 | data[0];
		vec[1] = data[3] << 8 | data[2];
		vec[2] = data[5] << 8 | data[4];
//		vec[2] = 65536 - vec[2];
	#if 0
		MMCDBG("[X - %04x] [Y - %04x] [Z - %04x]\n", 
			vec[0], vec[1], vec[2]);
	#endif
		if (copy_to_user(pa, vec, sizeof(vec))) {
	        //mutex_unlock(&ecompass_lock);
			return -EFAULT;
		}
		break;
	case MMC31XX_IOC_READXYZ:
	    MMCDBG("MMC3416X_IOC_READXYZ \n");
        //mmc3416x_dev_init( client);
	    if (ECS_ReadXYZData(vec, 3) )
	    {
            //mutex_unlock(&ecompass_lock);
			return -EFAULT;
	    }
		
	#if 0
		MMCDBG("[X - %04x] [Y - %04x] [Z - %04x]\n", 
			vec[0], vec[1], vec[2]);
	#endif
		if (copy_to_user(pa, vec, sizeof(vec))) {
	       // mutex_unlock(&ecompass_lock);
			return -EFAULT;
		}

		break;
#ifdef OPPO_R819
        case ECOMPASS_IOC_GET_DELAY:			
			delay = mmcd_delay;
			if(copy_to_user(pa, &delay, sizeof(delay)))
			{
			   // mutex_unlock(&ecompass_lock);
				printk(KERN_ERR "copy_to_user failed.");
				return -EFAULT;
			}
			break;		
			
		case ECOMPASS_IOC_SET_YPR:			
			if(pa == NULL)
			{
			    //mutex_unlock(&ecompass_lock);
				MMCDBG("invalid argument.");
				return -EINVAL;
			}
			if(copy_from_user(value, pa, sizeof(value)))
			{
			   // mutex_unlock(&ecompass_lock);
				MMCDBG("copy_from_user failed.");
				return -EFAULT;
			}
			ECS_SaveData(value);
			break;

		case ECOMPASS_IOC_GET_OPEN_STATUS:
			status = ECS_GetOpenStatus();			
			if(copy_to_user(pa, &status, sizeof(status)))
			{
			   // mutex_unlock(&ecompass_lock);
				MMCDBG("copy_to_user failed.");
				return -EFAULT;
			}
			break;
			
		case ECOMPASS_IOC_GET_MFLAG:
			sensor_status = atomic_read(&m_flag);
			if(copy_to_user(pa, &sensor_status, sizeof(sensor_status)))
			{
			    //mutex_unlock(&ecompass_lock);
				MMCDBG("copy_to_user failed.");
				return -EFAULT;
			}
			break;
			
		case ECOMPASS_IOC_GET_OFLAG:
		    MMCDBG("ECOMPASS_IOC_GET_OFLAG=%d\n",ECOMPASS_IOC_GET_OFLAG);
			sensor_status = atomic_read(&o_flag);
			MMCDBG("ECOMPASS_IOC_GET_OFLAG step1\n");
			if(copy_to_user(pa, &sensor_status, sizeof(sensor_status)))
			{
			    //mutex_unlock(&ecompass_lock);
				MMCDBG("copy_to_user failed.");
				return -EFAULT;
			}
			MMCDBG("ECOMPASS_IOC_GET_OFLAG step2\n");
			break;			
		                

		case MSENSOR_IOCTL_READ_CHIPINFO:
			if(pa == NULL)
			{
			    //mutex_unlock(&ecompass_lock);
				printk(KERN_ERR "IO parameter pointer is NULL!\r\n");
				break;
			}
			
			mmc3416x_ReadChipInfo(buff, MMC3416x_BUFSIZE);
			if(copy_to_user(pa, buff, strlen(buff)+1))
			{
			    //mutex_unlock(&ecompass_lock);
				return -EFAULT;
			}                
			break;

		case MSENSOR_IOCTL_READ_SENSORDATA:	
			if(pa == NULL)
			{
			    //mutex_unlock(&ecompass_lock);
				printk(KERN_ERR "IO parameter pointer is NULL!\r\n");
				break;    
			}
			ECS_GetRawData(vec);			
			sprintf(buff, "%x %x %x", vec[0], vec[1], vec[2]);
			if(copy_to_user(pa, buff, strlen(buff)+1))
			{
			    //mutex_unlock(&ecompass_lock);
				return -EFAULT;
			}                
			break;

		case ECOMPASS_IOC_GET_LAYOUT:
			status = atomic_read(&clientdata->layout);
			if(copy_to_user(pa, &status, sizeof(status)))
			{
				MMCDBG("copy_to_user failed.");
				//mutex_unlock(&ecompass_lock);
				return -EFAULT;
			}
			break;

		case MSENSOR_IOCTL_SENSOR_ENABLE:
			
			if(pa == NULL)
			{
				printk(KERN_ERR "IO parameter pointer is NULL!\r\n");
				break;
			}
			if(copy_from_user(&enable, pa, sizeof(enable)))
			{
				MMCDBG("copy_from_user failed.");
				//mutex_unlock(&ecompass_lock);
				return -EFAULT;
			}
			else
			{
			    printk( "MSENSOR_IOCTL_SENSOR_ENABLE enable=%d!\r\n",enable);
				if(1 == enable)
				{
					atomic_set(&o_flag, 1);
					atomic_set(&open_flag, 1);
				}
				else
				{
					atomic_set(&o_flag, 0);
					if(atomic_read(&m_flag) == 0)
					{
						atomic_set(&open_flag, 0);
					}			
				}
				wake_up(&open_wq);
				
			}
			
			break;
			
		case MSENSOR_IOCTL_READ_FACTORY_SENSORDATA:			
			if(pa == NULL)
			{
				printk(KERN_ERR "IO parameter pointer is NULL!\r\n");
				break;    
			}
			
			//AKECS_GetRawData(buff, AKM8975_BUFSIZE);
			osensor_data = (hwm_sensor_data *)buff;
		    mutex_lock(&sensor_data_mutex);
				
			osensor_data->values[0] = sensor_data[8] * CONVERT_O;
			osensor_data->values[1] = sensor_data[9] * CONVERT_O;
			osensor_data->values[2] = sensor_data[10] * CONVERT_O;
			osensor_data->status = sensor_data[11];
			osensor_data->value_divide = CONVERT_O_DIV;
					
			mutex_unlock(&sensor_data_mutex);

            sprintf(buff, "%x %x %x %x %x", osensor_data->values[0], osensor_data->values[1],
				osensor_data->values[2],osensor_data->status,osensor_data->value_divide);
			if(copy_to_user(pa, buff, strlen(buff)+1))
			{
			   //mutex_unlock(&ecompass_lock);
				return -EFAULT;
			} 
			
			break;

#endif /*OPPO_R819*/
	default:
	    MMCDBG("lizhanhua no command failed.\n");
		break;
	}
	//mutex_unlock(&ecompass_lock);
	return 0;
}


static struct file_operations mmc3416x_fops = {
	.owner		= THIS_MODULE,
	.open		= mmc3416x_open,
	.release	= mmc3416x_release,
	.unlocked_ioctl = mmc3416x_ioctl,
};

static struct miscdevice mmc3416x_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "msensor",
	.fops = &mmc3416x_fops,
};

/**********************************************************/

static void mmc3416x_poll(struct input_polled_dev *ipdev)
{
	unsigned char data[16] = {0, 0, 0, 0, 0, 0, 0, 0,
				  0, 0, 0, 0, 0, 0, 0, 0};
	int vec[3] = {0, 0, 0};
	static int first = 1;
	mutex_lock(&lock);
	if (!first) {
		/* read xyz raw data */
		read_idx++;
		data[0] = MMC3416X_REG_DATA;
		if (mmc3xxx_i2c_rx_data(data, 6) < 0) {
			mutex_unlock(&lock);
			return;
		}
		vec[0] = data[1] << 8 | data[0];
		vec[1] = data[3] << 8 | data[2];
		vec[2] = data[5] << 8 | data[4];
//		vec[2] = 65536 - vec[2];
#if 0
		printk("[X - %04x] [Y - %04x] [Z - %04x]\n", 
			vec[0], vec[1], vec[2]);
#endif
		input_report_abs(ipdev->input, ABS_X, vec[0]);
		input_report_abs(ipdev->input, ABS_Y, vec[1]);
		input_report_abs(ipdev->input, ABS_Z, vec[2]);

		input_sync(ipdev->input);
	} else {
		first = 0;
	}

	/* send TM cmd before read */
	data[0] = MMC3416X_REG_CTRL;
	data[1] = MMC3416X_CTRL_TM;
	/* not check return value here, assume it always OK */
	mmc3xxx_i2c_tx_data(data, 2);
	msleep(MMC3416X_DELAY_TM);
	mutex_unlock(&lock);
}
/****************************************************/

/*----------------------------------------------------------------------------*/
#ifndef	CONFIG_HAS_EARLYSUSPEND
/*----------------------------------------------------------------------------*/
static int mmc3416x_suspend(struct i2c_client *client, pm_message_t msg) 
{
	struct mmc3416x_i2c_data *obj = i2c_get_clientdata(client)
	if(msg.event == PM_EVENT_SUSPEND)
	{
		mmc3416_power(obj->hw, 0);
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static int mmc3416x_resume(struct i2c_client *client)
{
	struct mmc3416x_i2c_data *obj = i2c_get_clientdata(client)
	mmc3416_power(obj->hw, 1);	
	return 0;
}
/*----------------------------------------------------------------------------*/
#else /*CONFIG_HAS_EARLY_SUSPEND is defined*/
/*----------------------------------------------------------------------------*/
static void mmc3416x_early_suspend(struct early_suspend *h) 
{
	struct mmc3416x_i2c_data *obj = container_of(h, 
	                        struct mmc3416x_i2c_data, early_drv);   

	if(NULL == obj) {
		printk(KERN_ERR "null pointer!!\n");
		return;
	}

	
#ifdef OPPO_R819//mingqiang.guo@BasicDrv.Sensor, add 2012/6/6 for sensor power off
	atomic_set(&obj->suspend, 1);	
	mmc3416_power(obj->hw, 0);
#endif/*OPPO_R819*/
	       
}
/*----------------------------------------------------------------------------*/
static void mmc3416x_late_resume(struct early_suspend *h)
{
	struct mmc3416x_i2c_data *obj = container_of(h, 
	                    struct mmc3416x_i2c_data, early_drv);         
 
	if(NULL == obj) {
		printk(KERN_ERR "null pointer!!\n");
		return;
	}
	mmc3416_power(obj->hw, 1);
#ifdef OPPO_R819//mingqiang.guo@BasicDrv.Sensor, add 2012/6/6 for msensor power off
	if (atomic_read(&obj->enable_before_resume)) {
		wake_up(&open_wq);	
		atomic_set(&obj->enable_before_resume, 0);
	}
	atomic_set(&obj->suspend, 0);
#endif/*OPPO_R819*/	
}
/*----------------------------------------------------------------------------*/
#endif /*CONFIG_HAS_EARLYSUSPEND*/

/*----------------------------------------------------------------------------*/
int mmc3416x_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* msensor_data;	
	struct i2c_client *client = this_client;  
	struct mmc3416x_i2c_data *data = i2c_get_clientdata(client);
 
	if(atomic_read(&data->trace) & MMC_FUN_DEBUG) {
		MMCFUNC("mmc3416x_operate\n");
	}	
 
#ifdef OPPO_R819//mingqiang.guo@BasicDrv.Sensor, add 2012/6/6 for msensor power off
	if (atomic_read(&data->suspend)) {
		if (SENSOR_ENABLE == command) {
			value = *(int *)buff_in;
			if(value == 1){
				atomic_set(&m_flag, 1);
				atomic_set(&open_flag, 1);
			} else {
				atomic_set(&m_flag, 0);
				if(atomic_read(&o_flag) == 0){
					atomic_set(&open_flag, 0);					
				}
			}
			atomic_set(&data->enable_before_resume, 1);
		}
		return 0;
	}
#endif/*OPPO_R819*/
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int))){
				printk(KERN_ERR "Set delay parameter error!\n");
				err = -EINVAL;
			} else {
				value = *(int *)buff_in;
				mmcd_delay = value;
				if(value <= 20){
					mmcd_delay = 20;
				}				
			}	
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int))){
				printk(KERN_ERR "Enable sensor parameter error!\n");
				err = -EINVAL;
			} else {				
				value = *(int *)buff_in;
				if(value == 1){
					atomic_set(&m_flag, 1);
					atomic_set(&open_flag, 1);
				} else {
					atomic_set(&m_flag, 0);
					if(atomic_read(&o_flag) == 0){
						atomic_set(&open_flag, 0);
					}
				}
				wake_up(&open_wq);				
				// TODO: turn device into standby or normal mode
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data))){
				printk(KERN_ERR "get sensor data parameter error!\n");
				err = -EINVAL;
			} else {
				msensor_data = (hwm_sensor_data *)buff_out;
				mutex_lock(&sensor_data_mutex);
				
				msensor_data->values[0] = sensor_data[4] * CONVERT_M;
				msensor_data->values[1] = sensor_data[5] * CONVERT_M;
				msensor_data->values[2] = sensor_data[6] * CONVERT_M;
#ifndef OPPO_SENSOR_HAL_ST_SUPPORT
// zhangqiang delete for use ST 9 axis arithmetic , this orientation sensor will be add in hal, 
// so delete it in kernel , but we need the orientation accuracy that calculated from kernel , 
// sensor_data[7] is the magnetic accuracy and we don't need it, we need sensor_data[11].
				msensor_data->status = sensor_data[7];
#else
				msensor_data->status = sensor_data[11];
#endif 
				msensor_data->value_divide = CONVERT_M_DIV;					
				mutex_unlock(&sensor_data_mutex);
				if(atomic_read(&data->trace) & MMC_HWM_DEBUG){
					MMCDBG("Hwm get m-sensor data: %d, %d, %d. divide %d, status %d!\n",
						msensor_data->values[0],msensor_data->values[1],msensor_data->values[2],
						msensor_data->value_divide,msensor_data->status);
				}	
			}
			break;
		default:
			printk(KERN_ERR "msensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}

/*----------------------------------------------------------------------------*/
int mmc3416x_orientation_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* osensor_data;		
	struct i2c_client *client = this_client;  
	struct mmc3416x_i2c_data *data = i2c_get_clientdata(client);

	
	if(atomic_read(&data->trace) & MMC_FUN_DEBUG){
		MMCFUNC("mmc3416x_orientation_operate");
	}	
#ifdef OPPO_R819//mingqiang.guo@BasicDrv.Sensor, add 2012/6/6 for msensor power off
	if (atomic_read(&data->suspend)){
		if (SENSOR_ENABLE == command){
			value = *(int *)buff_in;
			if(value == 1){
				atomic_set(&o_flag, 1);
				atomic_set(&open_flag, 1);
			} else {
				atomic_set(&o_flag, 0);
				if(atomic_read(&m_flag) == 0){
					atomic_set(&open_flag, 0);
				}									
			}	
			atomic_set(&data->enable_before_resume, 1);
		}
		return 0;
	}
#endif/*OPPO_R819*/

	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int))){
				printk(KERN_ERR "Set delay parameter error!\n");
				err = -EINVAL;
			} else {
				value = *(int *)buff_in;
				mmcd_delay = value;
				if(value <= 20){
					mmcd_delay = 20;
				}				
			}	
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int))) {
				printk(KERN_ERR "Enable sensor parameter error!\n");
				err = -EINVAL;
			} else {				
				value = *(int *)buff_in;
				if(value == 1){
					atomic_set(&o_flag, 1);
					atomic_set(&open_flag, 1);
				} else {
					atomic_set(&o_flag, 0);
					if(atomic_read(&m_flag) == 0){
						atomic_set(&open_flag, 0);
					}									
				}	
				wake_up(&open_wq);
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data))){
				printk(KERN_ERR "get sensor data parameter error!\n");
				err = -EINVAL;
			} else {
				osensor_data = (hwm_sensor_data *)buff_out;
				mutex_lock(&sensor_data_mutex);
				
				osensor_data->values[0] = sensor_data[8] * CONVERT_O;
				osensor_data->values[1] = sensor_data[9] * CONVERT_O;
				osensor_data->values[2] = sensor_data[10] * CONVERT_O;
				osensor_data->status = sensor_data[11];					
				osensor_data->value_divide = CONVERT_O_DIV;
				mutex_unlock(&sensor_data_mutex);
			    if(atomic_read(&data->trace) & MMC_HWM_DEBUG){
				    MMCDBG("Hwm get o-sensor data: %d, %d, %d. divide %d, status %d!\n",
					osensor_data->values[0],osensor_data->values[1],osensor_data->values[2],
					osensor_data->value_divide,osensor_data->status);
			    }	
			}
			break;
		default:
			printk(KERN_ERR "gsensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}


static int mmc3416x_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int res = 0;
    struct i2c_client *new_client;
	struct mmc3416x_i2c_data *data;
	char tmp[2];
	int err = 0;
	struct hwmsen_object sobj_m, sobj_o;

	MMCDBG("%s: ++++\n", __func__);
    if(!(data = kmalloc(sizeof(struct mmc3416x_i2c_data), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(data, 0, sizeof(struct mmc3416x_i2c_data));
	data->hw = get_cust_mag_hw_mmc3461();	 
	atomic_set(&data->layout, MMC3416XMA_DIRECTION);
	atomic_set(&data->trace, 0);
	atomic_set(&data->suspend, 0);
	atomic_set(&data->enable_before_resume, 0);
	

	mutex_init(&sensor_data_mutex);
	mutex_init(&read_i2c_xyz);	
	init_waitqueue_head(&data_ready_wq);
	init_waitqueue_head(&open_wq);

	data->client = client;
	new_client = data->client;
	i2c_set_clientdata(new_client, data);	
	this_client = new_client;	
	
		// init mmc3416x
	if ((err = mmc3416x_dev_init(client)) < 0) {
		MMCDBG("init device error!\n");
		goto exit_init_failed;
	}
	#if 0
	/* send ST cmd to mag sensor first of all */
	tmp[0] = MMC3416X_REG_CTRL;
	tmp[1] = MMC3416X_CTRL_REFILL;
	if (mmc3xxx_i2c_tx_data(tmp, 2) < 0) {
	}
	msleep(MMC3416X_DELAY_SET);

	tmp[0] = MMC3416X_REG_CTRL;
	tmp[1] = MMC3416X_CTRL_SET;
	if (mmc3xxx_i2c_tx_data(tmp, 2) < 0) {
	}
	msleep(1);
	tmp[0] = MMC3416X_REG_CTRL;
	tmp[1] = 0;
	if (mmc3xxx_i2c_tx_data(tmp, 2) < 0) {
	}
	msleep(MMC3416X_DELAY_SET);

	tmp[0] = MMC3416X_REG_CTRL;
	tmp[1] = MMC3416X_CTRL_REFILL;
	if (mmc3xxx_i2c_tx_data(tmp, 2) < 0) {
	}
	msleep(MMC3416X_DELAY_RESET);
	tmp[0] = MMC3416X_REG_CTRL;
	tmp[1] = MMC3416X_CTRL_RESET;
	if (mmc3xxx_i2c_tx_data(tmp, 2) < 0) {
	}
	msleep(1);
	tmp[0] = MMC3416X_REG_CTRL;
	tmp[1] = 0;
	if (mmc3xxx_i2c_tx_data(tmp, 2) < 0) {
	}
	msleep(1);

	tmp[0] = MMC3416X_REG_BITS;
	tmp[1] = MMC3416X_BITS_SLOW_16;
	if (mmc3xxx_i2c_tx_data(tmp, 2) < 0) {
	}
	msleep(MMC3416X_DELAY_TM);

	tmp[0] = MMC3416X_REG_CTRL;
	tmp[1] = MMC3416X_CTRL_TM;
	if (mmc3xxx_i2c_tx_data(tmp, 2) < 0) {
	}
	msleep(MMC3416X_DELAY_TM);
	#endif

	/* Register sysfs attribute */
	if(err = mmc3416x_create_attr(&mmc3416x_platform_driver.driver))
	{
		printk(KERN_ERR "create attribute err = %d\n", err);
		goto exit_sysfs_create_group_failed;
	}

	
	if(err = misc_register(&mmc3416x_device)){
		printk(KERN_ERR "mmc3416x_device register failed\n");
		goto exit_misc_device_register_failed;	
	}    

	sobj_m.self = data;
    sobj_m.polling = 1;
    sobj_m.sensor_operate = mmc3416x_operate;
	if(err = hwmsen_attach(ID_MAGNETIC, &sobj_m)){
		printk(KERN_ERR "attach fail = %d\n", err);
		goto exit_kfree;
	}
	
	sobj_o.self = data;
    sobj_o.polling = 1;
    sobj_o.sensor_operate = mmc3416x_orientation_operate;
	
#ifndef OPPO_SENSOR_HAL_ST_SUPPORT
// zhangqiang delete for use ST 9 axis arithmetic , this orientation sensor will be add in hal, so delete it here
	if(err = hwmsen_attach(ID_ORIENTATION, &sobj_o))
	{
		printk(KERN_ERR "attach fail = %d\n", err);
		goto exit_kfree;
	}
#endif
	
#if CONFIG_HAS_EARLYSUSPEND
	data->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	data->early_drv.suspend  = mmc3416x_early_suspend,
	data->early_drv.resume   = mmc3416x_late_resume,    
	register_early_suspend(&data->early_drv);
#endif
#ifdef OPPO_R819//Shaoyu.Huang@BasicDrv.Sensor, add 2012/6/4 for sensor debuging
	hwmsen_make_debug_flag(&debug_fops, "mmc3416xma_msensor");
#endif/*OPPO_R819*/

	MMCDBG("%s: OK\n", __func__);
	return 0;
    exit_kfree:
	    misc_deregister(&mmc3416x_device);
	exit_sysfs_create_group_failed:	
	exit_misc_device_register_failed:
    exit_init_failed:
    this_client = NULL;
	exit:
#ifdef OPPO_R819//mingqiang.guo@BasicDrv.Sensor, add 2012/6/6 for msensor power off
	mmc3416_power(data->hw, 0);
#endif/*OPPO_R819*/

	kfree(data);

	printk(KERN_ERR "%s: err = %d\n", __func__, err);
	return err;
}

static int mmc3416x_remove(struct i2c_client *client)
{
    int err;
    
	if( (err= mmc3416x_delete_attr( &mmc3416x_platform_driver.driver))< 0){
        printk(KERN_ERR "mmc3416x_delete_attr fail: %d\n", err);
	}
	this_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	misc_deregister(&mmc3416x_device);
	//if (ipdev) input_unregister_device(ipdev);

	return 0;
}

static const struct i2c_device_id mmc3416x_id[] = {
	{ MMC3416X_I2C_NAME, 0 },
	{ }
};

static struct i2c_driver mmc3416x_i2c_driver = {
	.probe 		= mmc3416x_probe,
	.remove 	= mmc3416x_remove,
	.id_table	= mmc3416x_id,
	.driver 	= {
		.owner	= THIS_MODULE,
		.name	= MMC3416X_I2C_NAME,
	},
};

//platform 
/*----------------------------------------------------------------------------*/
static int platform_mmc3416x_probe(struct platform_device *pdev) 
{
	struct mag_hw *hw = get_cust_mag_hw_mmc3461();

	mmc3416_power(hw, 1);
	
	//atomic_set(&dev_open_count, 0);
	//akm8975_force[0] = hw->i2c_num;
    
	if(i2c_add_driver(&mmc3416x_i2c_driver))
	{
		printk(KERN_ERR "add driver error\n");
		return -1;
	} 
	return 0;
}
/*----------------------------------------------------------------------------*/
static int platform_mmc3416x_remove(struct platform_device *pdev)
{
	struct mag_hw *hw = get_cust_mag_hw_mmc3461();
 
	mmc3416_power(hw, 0);    
	//atomic_set(&dev_open_count, 0);  
	i2c_del_driver(&mmc3416x_i2c_driver);
	return 0;
}


static int __init mmc3416x_init(void)
{
    //i2c-3
   //eturn 0;
    i2c_register_board_info(2, &i2c_mmc3416x, 1);
	if(platform_driver_register(&mmc3416x_platform_driver))
	{
		printk(KERN_ERR "failed to register driver");
		return -ENODEV;
	}
	return 0;    
}

static void __exit mmc3416x_exit(void)
{
        
      platform_driver_unregister(&mmc3416x_platform_driver);
}


module_init(mmc3416x_init);
module_exit(mmc3416x_exit);

MODULE_DESCRIPTION("MEMSIC MMC3416X Magnetic Sensor Driver");
MODULE_LICENSE("GPL");

