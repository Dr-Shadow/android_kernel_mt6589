/* drivers/i2c/chips/lis3dh.c - LIS3DH motion sensor driver
 *
 *
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

#include <cust_eint.h>
#include <mach/eint.h>

#include <linux/proc_fs.h>

#include <linux/gpio.h>

#include <cust_acc.h>
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include "lis3dh.h"
#include <linux/hwmsen_helper.h>

#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>

#define GSEN_DBG(x...) 

//#define OPPO_MOTION_SENSITIVE_GSENSOR

#define DEBUG 1
#define CONFIG_LIS3DH_LOWPASS   /*apply low pass filter on output*/       

#define LIS3DH_AXIS_X          0
#define LIS3DH_AXIS_Y          1
#define LIS3DH_AXIS_Z          2
#define LIS3DH_AXES_NUM        3
#define LIS3DH_DATA_LEN        6
#define LIS3DH_DEV_NAME        "LIS3DH"

/*----------------------------------------------------------------------------*/
static const struct i2c_device_id lis3dh_i2c_id[] = {{LIS3DH_DEV_NAME,0},{}};

static struct i2c_board_info __initdata i2c_LIS3DH={ I2C_BOARD_INFO("LIS3DH", (0x32>>1))};

static struct workqueue_struct *gsensor_wq = NULL;

extern unsigned int sensor_suspend;

extern int whether_in_call_state();
extern int whether_in_approach_state();

/*----------------------------------------------------------------------------*/
static int calculate_gsensor_cali_data(struct i2c_client *client, s16 data[3]);

static int LIS3DH_SetIntEnable(struct i2c_client *client, u8 intenable);

static void gsensor_motion_enable(struct i2c_client *client, u8 enable);

/*----------------------------------------------------------------------------*/
static int lis3dh_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int lis3dh_i2c_remove(struct i2c_client *client);
static int lis3dh_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);

#ifdef OPPO_MOTION_SENSITIVE_GSENSOR

static void gsensor_motion_detect_init(struct i2c_client *client);
#endif
/*----------------------------------------------------------------------------*/
typedef enum 
{
    ADX_TRC_FILTER  = 0x01,
    ADX_TRC_RAWDATA = 0x02,
    ADX_TRC_IOCTL   = 0x04,
    ADX_TRC_CALI	= 0X08,
    ADX_TRC_INFO	= 0X10,
} ADX_TRC;

/*----------------------------------------------------------------------------*/
struct scale_factor
{
    u8  whole;
    u8  fraction;
};

/*----------------------------------------------------------------------------*/
struct data_resolution 
{
    struct scale_factor scalefactor;
    int                 sensitivity;
};

/*----------------------------------------------------------------------------*/
#define C_MAX_FIR_LENGTH (32)

/*----------------------------------------------------------------------------*/
struct data_filter 
{
    s16 raw[C_MAX_FIR_LENGTH][LIS3DH_AXES_NUM];
    int sum[LIS3DH_AXES_NUM];
    int num;
    int idx;
};

/*----------------------------------------------------------------------------*/
struct lis3dh_i2c_data 
{
    struct i2c_client *client;
    struct acc_hw *hw;
    struct hwmsen_convert   cvt;
	struct work_struct  work;
	struct input_dev *input_dev;
	struct proc_dir_entry* entry;
	
    /*misc*/
    struct data_resolution *reso;
    atomic_t                trace;
    atomic_t                suspend;
    atomic_t                selftest;
	atomic_t				filter;
	atomic_t				interrupt_enable;
	atomic_t				interrupt_true;
	atomic_t				senosr_interrupt;
	atomic_t				sensor_suspend_flag;
	atomic_t				in_call_state;
	
    s16                     cali_sw[LIS3DH_AXES_NUM+1];

    /*data*/
    s8                      offset[LIS3DH_AXES_NUM+1];  /*+1: for 4-byte alignment*/
    s16                     data[LIS3DH_AXES_NUM+1];

#if defined(CONFIG_LIS3DH_LOWPASS)
    atomic_t                firlen;
    atomic_t                fir_en;
    struct data_filter      fir;
#endif 
    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif     
};

/*----------------------------------------------------------------------------*/
static struct i2c_driver lis3dh_i2c_driver = 
{
    .driver = 
	{
//		.owner          = THIS_MODULE,
		.name           = LIS3DH_DEV_NAME,
    },
	.probe      		= lis3dh_i2c_probe,
	.remove    			= lis3dh_i2c_remove,
	.detect				= lis3dh_i2c_detect,
#if !defined(CONFIG_HAS_EARLYSUSPEND)    
    .suspend            = lis3dh_suspend,
    .resume             = lis3dh_resume,
#endif
	.id_table = lis3dh_i2c_id,
};

/*----------------------------------------------------------------------------*/
static struct i2c_client *lis3dh_i2c_client = NULL;
static struct platform_driver lis3dh_gsensor_driver;
static struct lis3dh_i2c_data *lis3dh_data = NULL;
static bool sensor_power = false;
static GSENSOR_VECTOR3D gsensor_gain, gsensor_offset;

/*----------------------------------------------------------------------------*/
#define GSE_TAG                  "[Gsensor] "
#define GSE_FUN(f)               printk( GSE_TAG"%s\n", __FUNCTION__)
#define GSE_ERR(fmt, args...)    printk(KERN_ERR GSE_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define GSE_LOG(fmt, args...)    printk( GSE_TAG fmt, ##args)

/*----------------------------------------------------------------------------*/
static struct data_resolution lis3dh_data_resolution[] = 
{
     /* combination by {FULL_RES,RANGE}*/
    {{ 1, 0}, 1024},	// dataformat +/-2g  in 12-bit resolution;  { 1, 0} = 1.0 = (2*2*1000)/(2^12);  1024 = (2^12)/(2*2) 
    {{ 1, 9}, 512},		// dataformat +/-4g  in 12-bit resolution;  { 1, 9} = 1.9 = (2*4*1000)/(2^12);  512 = (2^12)/(2*4) 
    {{ 3, 9}, 256},		// dataformat +/-8g  in 12-bit resolution;  { 1, 0} = 1.0 = (2*8*1000)/(2^12);  1024 = (2^12)/(2*8) 
};

/*----------------------------------------------------------------------------*/
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

static struct file_operations debug_fops = 
{
	.read = debugflag_read,
	.write = debugflag_write,
};

/*----------------------------------------------------------------------------*/
static void dumpReg()
{
	u8 regdata = 0;

	//dump Reg
	regdata = i2c_smbus_read_byte_data(lis3dh_i2c_client, LIS3DH_REG_CTL_REG1);
	HWM_LOG("LIS3DH_DUMP CTL_REG1 : addr = 0x20 , right reg data is 0x57 , now data is 0x%x \n", regdata);

	regdata = i2c_smbus_read_byte_data(lis3dh_i2c_client, LIS3DH_REG_CTL_REG2);
	HWM_LOG("LIS3DH_DUMP CTL_REG2 : addr = 0x21 , right reg data is 0x00 , now data is 0x%x \n", regdata);

	regdata = i2c_smbus_read_byte_data(lis3dh_i2c_client, LIS3DH_REG_CTL_REG3);
	HWM_LOG("LIS3DH_DUMP CTL_REG3 : addr = 0x22 , right reg data is 0x40 , now data is 0x%x \n", regdata);

	regdata = i2c_smbus_read_byte_data(lis3dh_i2c_client, LIS3DH_REG_CTL_REG4);
	HWM_LOG("LIS3DH_DUMP CTL_REG4 : addr = 0x23 , right reg data is 0x88 , now data is 0x%x \n", regdata);

	regdata = i2c_smbus_read_byte_data(lis3dh_i2c_client, LIS3DH_REG_CTL_REG5);
	HWM_LOG("LIS3DH_DUMP CTL_REG5 : addr = 0x24 , right reg data is 0x%x , now data is 0x%x \n", GSENSOR_LIR_INT1, regdata);

	regdata = i2c_smbus_read_byte_data(lis3dh_i2c_client, LIS3DH_REG_CTL_REG6);
	HWM_LOG("LIS3DH_DUMP CTL_REG6 : addr = 0x25 , right reg data is 0x%x , now data is 0x%x \n", GSENSOR_H_LACTIVE, regdata);

	regdata = i2c_smbus_read_byte_data(lis3dh_i2c_client, LIS3DH_REFERENCE);
	HWM_LOG("LIS3DH_DUMP REFERENCE : addr = 0x26 , right reg data is 0x00 , now data is 0x%x \n", regdata);

	regdata = i2c_smbus_read_byte_data(lis3dh_i2c_client, LIS3DH_STATUS_REG);
	HWM_LOG("LIS3DH_DUMP STATUS_REG : addr = 0x27 , right reg data is 0x00 , now data is 0x%x \n", regdata);

	regdata = i2c_smbus_read_byte_data(lis3dh_i2c_client, LIS3DH_INT1_CFG);
	HWM_LOG("LIS3DH_DUMP INT1_CFG : addr = 0x30 , right reg data is 0x%x , now data is 0x%x \n", GSENSOR_INT1_DESCRIPTION, regdata);

	regdata = i2c_smbus_read_byte_data(lis3dh_i2c_client, LIS3DH_INT1_SRC);
	HWM_LOG("LIS3DH_DUMP INT1_SRC : addr = 0x31 , right reg data is 0x-- , now data is 0x%x \n", regdata);

	regdata = i2c_smbus_read_byte_data(lis3dh_i2c_client, LIS3DH_INT1_THS);
	HWM_LOG("LIS3DH_DUMP INT1_THS : addr = 0x32 , right reg data is 0x%x , now data is 0x%x \n", GSENSOR_INT1_THS_VALUE, regdata);

	regdata = i2c_smbus_read_byte_data(lis3dh_i2c_client, LIS3DH_INT1_DURATION);
	HWM_LOG("LIS3DH_DUMP INT1_DURATION : addr = 0x33 , right reg data is 0x%x , now data is 0x%x \n", GSENSOR_INT1_DURATION_VALUE, regdata);
}

/*--------------------ADXL power control function----------------------------------*/
static void LIS3DH_power(struct acc_hw *hw, unsigned int on) 
{
	printk(GSE_TAG"%s  power=%s\n",__func__,on==1?"ON":"OFF");
	hw->power(hw, on, NULL);
}

/*----------------------------------------------------------------------------*/
static int LIS3DH_SetDataResolution(struct lis3dh_i2c_data *obj)
{
	int err = 0;
	u8 databuf = 0, reso = 0;

	databuf = i2c_smbus_read_byte_data(obj->client, LIS3DH_REG_CTL_REG4);
	if(databuf < 0)
	{
		GSE_ERR("write data format fail!!\n");
		return -EINVAL;
	}

	/*the data_reso is combined by 3 bits: {FULL_RES, DATA_RANGE}*/
	reso  = (databuf & 0x30)<<4;
	if(reso >= 0x3)
		reso = 0x2;

	printk("%s, reso = 0x%x, databuf[0] = 0x%x\n", __func__, reso, databuf);

	if(reso < sizeof(lis3dh_data_resolution)/sizeof(lis3dh_data_resolution[0]))
	{        
		obj->reso = &lis3dh_data_resolution[reso];
		return 0;
	}
	else
	{
		return -EINVAL;
	}
}

/*----------------------------------------------------------------------------*/
static int LIS3DH_ReadData(struct i2c_client *client, s16 data[LIS3DH_AXES_NUM])
{
	struct lis3dh_i2c_data *priv = i2c_get_clientdata(client);        
	u8 buf[LIS3DH_DATA_LEN] = {0};
	int err = 0;

	if(NULL == client)
	{
		err = -EINVAL;
	}
	else
	{

#ifndef VENDOR_EDIT  //modify for read data at one time

		if(hwmsen_read_block(client, LIS3DH_REG_OUT_X, buf, 0x01))
		{
			GSE_ERR("read  G sensor data register err!\n");
			return -1;
		}
		if(hwmsen_read_block(client, LIS3DH_REG_OUT_X+1, &buf[1], 0x01))
		{
			GSE_ERR("read  G sensor data register err!\n");
			return -1;
		}
		data[LIS3DH_AXIS_X] = (s16)((buf[0]+(buf[1]<<8))>>4);
		
		if(hwmsen_read_block(client, LIS3DH_REG_OUT_Y, &buf[2], 0x01))
		{
			GSE_ERR("read  G sensor data register err!\n");
			return -1;
		}
		if(hwmsen_read_block(client, LIS3DH_REG_OUT_Y+1, &buf[3], 0x01))
		{
			GSE_ERR("read  G sensor data register err!\n");
			return -1;
		}
		data[LIS3DH_AXIS_Y] =  (s16)((s16)(buf[2] +( buf[3]<<8))>>4);

		if(hwmsen_read_block(client, LIS3DH_REG_OUT_Z, &buf[4], 0x01))
		{
			GSE_ERR("read  G sensor data register err!\n");
			return -1;
		}
		if(hwmsen_read_block(client, LIS3DH_REG_OUT_Z+1, &buf[5], 0x01))
		{
			GSE_ERR("read  G sensor data register err!\n");
			return -1;
		}
		data[LIS3DH_AXIS_Z] =(s16)((buf[4]+(buf[5]<<8))>>4);

		//GSE_LOG("[%08X %08X %08X %08x %08x %08x]\n",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5]);
#else 

		if(hwmsen_read_block(client, LIS3DH_REG_OUT_X | 0x80, buf, 0x06)) 
		{
			GSE_ERR("read  G sensor data register err!\n");
			return -1;
		}
		
		data[LIS3DH_AXIS_X] =( (s16)(buf[0]|(buf[1]<<8)) ) >>4;  
		data[LIS3DH_AXIS_Y] =( (s16)(buf[2]|(buf[3]<<8)) ) >>4;
		data[LIS3DH_AXIS_Z] =( (s16)(buf[4]|(buf[5]<<8)) ) >>4;	
		
		GSEN_DBG(" read form i2c data : %d,%d,%d \n",data[0],data[1],data[2]);
	
#endif 
	
		data[LIS3DH_AXIS_X] &= 0xfff;
		data[LIS3DH_AXIS_Y] &= 0xfff;
		data[LIS3DH_AXIS_Z] &= 0xfff;

		if(atomic_read(&priv->trace) & ADX_TRC_RAWDATA)
		{
			GSE_LOG("[%08X %08X %08X] => [%5d %5d %5d]\n", data[LIS3DH_AXIS_X], data[LIS3DH_AXIS_Y], 
					data[LIS3DH_AXIS_Z], data[LIS3DH_AXIS_X], data[LIS3DH_AXIS_Y], data[LIS3DH_AXIS_Z]);
		}

		if(data[LIS3DH_AXIS_X]&0x800)
		{
			data[LIS3DH_AXIS_X] = ~data[LIS3DH_AXIS_X];
			data[LIS3DH_AXIS_X] &= 0xfff;
			data[LIS3DH_AXIS_X] +=1;
			data[LIS3DH_AXIS_X] = -data[LIS3DH_AXIS_X];
		}
		if(data[LIS3DH_AXIS_Y]&0x800)
		{
			data[LIS3DH_AXIS_Y] = ~data[LIS3DH_AXIS_Y];
			data[LIS3DH_AXIS_Y] &= 0xfff;
			data[LIS3DH_AXIS_Y] +=1;
			data[LIS3DH_AXIS_Y] = -data[LIS3DH_AXIS_Y];
		}
		if(data[LIS3DH_AXIS_Z]&0x800)
		{
			data[LIS3DH_AXIS_Z] = ~data[LIS3DH_AXIS_Z];
			data[LIS3DH_AXIS_Z] &= 0xfff;
			data[LIS3DH_AXIS_Z] +=1;
			data[LIS3DH_AXIS_Z] = -data[LIS3DH_AXIS_Z];
		}

		if(atomic_read(&priv->trace) & ADX_TRC_RAWDATA)
		{
			GSE_LOG("[%08X %08X %08X] => [%5d %5d %5d] after\n", data[LIS3DH_AXIS_X], data[LIS3DH_AXIS_Y], 
					data[LIS3DH_AXIS_Z], data[LIS3DH_AXIS_X], data[LIS3DH_AXIS_Y], data[LIS3DH_AXIS_Z]);
		}
		
#ifdef CONFIG_LIS3DH_LOWPASS
		if(atomic_read(&priv->filter))
		{
			if(atomic_read(&priv->fir_en) && !atomic_read(&priv->suspend))
			{
				int idx, firlen = atomic_read(&priv->firlen);   
				if(priv->fir.num < firlen)
				{                
					priv->fir.raw[priv->fir.num][LIS3DH_AXIS_X] = data[LIS3DH_AXIS_X];
					priv->fir.raw[priv->fir.num][LIS3DH_AXIS_Y] = data[LIS3DH_AXIS_Y];
					priv->fir.raw[priv->fir.num][LIS3DH_AXIS_Z] = data[LIS3DH_AXIS_Z];
					priv->fir.sum[LIS3DH_AXIS_X] += data[LIS3DH_AXIS_X];
					priv->fir.sum[LIS3DH_AXIS_Y] += data[LIS3DH_AXIS_Y];
					priv->fir.sum[LIS3DH_AXIS_Z] += data[LIS3DH_AXIS_Z];
					if(atomic_read(&priv->trace) & ADX_TRC_FILTER)
					{
						GSE_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d]\n", priv->fir.num,
							priv->fir.raw[priv->fir.num][LIS3DH_AXIS_X], priv->fir.raw[priv->fir.num][LIS3DH_AXIS_Y], priv->fir.raw[priv->fir.num][LIS3DH_AXIS_Z],
							priv->fir.sum[LIS3DH_AXIS_X], priv->fir.sum[LIS3DH_AXIS_Y], priv->fir.sum[LIS3DH_AXIS_Z]);
					}
					priv->fir.num++;
					priv->fir.idx++;
				}
				else
				{
					idx = priv->fir.idx % firlen;
					priv->fir.sum[LIS3DH_AXIS_X] -= priv->fir.raw[idx][LIS3DH_AXIS_X];
					priv->fir.sum[LIS3DH_AXIS_Y] -= priv->fir.raw[idx][LIS3DH_AXIS_Y];
					priv->fir.sum[LIS3DH_AXIS_Z] -= priv->fir.raw[idx][LIS3DH_AXIS_Z];
					priv->fir.raw[idx][LIS3DH_AXIS_X] = data[LIS3DH_AXIS_X];
					priv->fir.raw[idx][LIS3DH_AXIS_Y] = data[LIS3DH_AXIS_Y];
					priv->fir.raw[idx][LIS3DH_AXIS_Z] = data[LIS3DH_AXIS_Z];
					priv->fir.sum[LIS3DH_AXIS_X] += data[LIS3DH_AXIS_X];
					priv->fir.sum[LIS3DH_AXIS_Y] += data[LIS3DH_AXIS_Y];
					priv->fir.sum[LIS3DH_AXIS_Z] += data[LIS3DH_AXIS_Z];
					priv->fir.idx++;
					data[LIS3DH_AXIS_X] = priv->fir.sum[LIS3DH_AXIS_X]/firlen;
					data[LIS3DH_AXIS_Y] = priv->fir.sum[LIS3DH_AXIS_Y]/firlen;
					data[LIS3DH_AXIS_Z] = priv->fir.sum[LIS3DH_AXIS_Z]/firlen;
					if(atomic_read(&priv->trace) & ADX_TRC_FILTER)
					{
						GSE_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d] : [%5d %5d %5d]\n", idx,
						priv->fir.raw[idx][LIS3DH_AXIS_X], priv->fir.raw[idx][LIS3DH_AXIS_Y], priv->fir.raw[idx][LIS3DH_AXIS_Z],
						priv->fir.sum[LIS3DH_AXIS_X], priv->fir.sum[LIS3DH_AXIS_Y], priv->fir.sum[LIS3DH_AXIS_Z],
						data[LIS3DH_AXIS_X], data[LIS3DH_AXIS_Y], data[LIS3DH_AXIS_Z]);
					}
				}
			}
		}	
#endif         
	}
	
	return err;
}

/*----------------------------------------------------------------------------*/
static int LIS3DH_ResetCalibration(struct i2c_client *client)
{
	struct lis3dh_i2c_data *obj = i2c_get_clientdata(client);	

	memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
	return 0;     
}

/*----------------------------------------------------------------------------*/
static int LIS3DH_ReadCalibration(struct i2c_client *client, int dat[LIS3DH_AXES_NUM])
{
    struct lis3dh_i2c_data *obj = i2c_get_clientdata(client);

    dat[obj->cvt.map[LIS3DH_AXIS_X]] = obj->cvt.sign[LIS3DH_AXIS_X]*obj->cali_sw[LIS3DH_AXIS_X];
    dat[obj->cvt.map[LIS3DH_AXIS_Y]] = obj->cvt.sign[LIS3DH_AXIS_Y]*obj->cali_sw[LIS3DH_AXIS_Y];
    dat[obj->cvt.map[LIS3DH_AXIS_Z]] = obj->cvt.sign[LIS3DH_AXIS_Z]*obj->cali_sw[LIS3DH_AXIS_Z];                        
                                       
    return 0;
}

/*----------------------------------------------------------------------------*/
static int LIS3DH_WriteCalibration(struct i2c_client *client, int dat[LIS3DH_AXES_NUM])
{
	struct lis3dh_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;

	GSE_FUN();

	if(!obj || ! dat)
	{
		GSE_ERR("null ptr!!\n");
		return -EINVAL;
	}
	else
	{        
		obj->cali_sw[LIS3DH_AXIS_X] = dat[0];
		obj->cali_sw[LIS3DH_AXIS_Y] = dat[1];
		obj->cali_sw[LIS3DH_AXIS_Z] = dat[2];
	} 

	printk("%s: x_cal = %d, y_cal = %d, z_cal = %d\n",__func__, dat[0], dat[1], dat[2]);
	
	return err;
}

/*----------------------------------------------------------------------------*/
static int LIS3DH_SetPowerMode(struct i2c_client *client, bool enable)
{
	u8 databuf[2];    
	int res = 0;
	struct lis3dh_i2c_data *obj = i2c_get_clientdata(client);
	
	if(enable == sensor_power)
	{
		printk("Sensor power status is newest!\n");
		return LIS3DH_SUCCESS;
	}

	databuf[0] = 0x07;
	
	if(enable == TRUE)
	{
		databuf[0] &=  ~LIS3DH_MEASURE_MODE;
	}
	else
	{
		databuf[0] |= LIS3DH_MEASURE_MODE;
	}

	printk("%s, databuf[0] = 0x%x\n", __func__, databuf[0]);
	
	databuf[1] = databuf[0];
	databuf[0] = LIS3DH_REG_CTL_REG1;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GSE_LOG("set power mode failed!\n");
		return LIS3DH_ERR_I2C;
	}
	else if(atomic_read(&obj->trace) & ADX_TRC_INFO)
	{
		GSE_LOG("set power mode ok %d!\n", databuf[1]);
	}

	sensor_power = enable;
	
	return LIS3DH_SUCCESS;    
}

/*----------------------------------------------------------------------------*/
static int LIS3DH_SetBWRate(struct i2c_client *client, u8 bwrate)
{
	u8 databuf[2];
	int res = 0;

#ifndef VENDOR_EDIT 	
// changed by zhangqiang for some wrong data that read from register when many APK of G-sensor open close continually

	if(hwmsen_read_byte(client, addr, &databuf[0]))
	{
		GSE_ERR("read reg_ctl_reg1 register err!\n");
		return LIS3DH_ERR_I2C;
	}

	databuf[0] |= bwrate;

#else 

	databuf[0] = 0x07;
	databuf[0] |= bwrate;

#endif

	printk("%s, bwrate = 0x%x, databuf[0] = 0x%x\n", __func__, bwrate, databuf[0]);

	databuf[1] = databuf[0];
	databuf[0] = LIS3DH_REG_CTL_REG1;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		return LIS3DH_ERR_I2C;
	}

	return LIS3DH_SUCCESS;    
}

/*----------------------------------------------------------------------------*/
static int LIS3DH_SetDataFormat(struct i2c_client *client, u8 dataformat)
{
	struct lis3dh_i2c_data *obj = i2c_get_clientdata(client);
	u8 databuf[2];
	int res = 0;

	databuf[0] = 0x88;		
	// output registers not update until MSB and LSB reading, High resolution Enable

	databuf[0] |= dataformat;

	printk("%s, databuf[0] = 0x%x\n", __func__, databuf[0]);

	databuf[1] = databuf[0];
	databuf[0] = LIS3DH_REG_CTL_REG4;
	res = i2c_master_send(client, databuf, 0x2);

	if(res <= 0)
	{
		return LIS3DH_ERR_I2C;
	}
	
	return LIS3DH_SetDataResolution(obj);    
}

/*----------------------------------------------------------------------------*/
static int LIS3DH_Init_client(struct i2c_client *client, int reset_cali)
{
	struct lis3dh_i2c_data *obj = i2c_get_clientdata(client);
	int res = 0;

    // first clear reg1
    res = hwmsen_write_byte(client,LIS3DH_REG_CTL_REG1,0x07);
	if(res != LIS3DH_SUCCESS)
	{
		printk(GSE_TAG"%s  LIS3DH_REG_CTL_REG1 write Fail !!!\n",__func__);
		return res;
	}

	res = LIS3DH_SetPowerMode(client, true);
	if(res != LIS3DH_SUCCESS)
	{
		printk(GSE_TAG"%s  SetPowerMode Fail !!!\n",__func__);
		return res;
	}

	res = LIS3DH_SetBWRate(client, LIS3DH_BW_100HZ);//400 or 100 no other choice
	if(res != LIS3DH_SUCCESS )
	{
		printk(GSE_TAG"%s  SetBWRate Fail !!!\n",__func__);
		return res;
	}

	res = LIS3DH_SetDataFormat(client, LIS3DH_RANGE_2G);//8g or 2G no oher choise
	if(res != LIS3DH_SUCCESS) 
	{
		printk(GSE_TAG"%s  SetDataFormat Fail !!!\n",__func__);
		return res;
	}
	
	gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = obj->reso->sensitivity;

	if(NULL != reset_cali)
	{ 
		//reset calibration only in power on
		printk("Lis3dh reset calibrate! \n");
		res = LIS3DH_ResetCalibration(client);
		if(res != LIS3DH_SUCCESS)
		{
			return res;
		}
	}

#ifdef CONFIG_LIS3DH_LOWPASS
	memset(&obj->fir, 0x00, sizeof(obj->fir));  
#endif
   #ifdef OPPO_MOTION_SENSITIVE_GSENSOR
    gsensor_motion_detect_init(client );
   #endif
	return LIS3DH_SUCCESS;
}

/*----------------------------------------------------------------------------*/
static int LIS3DH_ReadChipInfo(struct i2c_client *client, char *buf, int bufsize)
{
	u8 databuf[10];    

	memset(databuf, 0, sizeof(u8)*10);

	if((NULL == buf)||(bufsize<=30))
	{
		return -1;
	}
	
	if(NULL == client)
	{
		*buf = 0;
		return -2;
	}

	sprintf(buf, "LIS3DH Chip");
	return 0;
}

/*----------------------------------------------------------------------------*/
static int LIS3DH_ReadSensorData(struct i2c_client *client, char *buf, int bufsize)
{
	struct lis3dh_i2c_data *obj = (struct lis3dh_i2c_data*)i2c_get_clientdata(client);
	u8 databuf[20];
	int acc[LIS3DH_AXES_NUM];
	int res = 0;
	memset(databuf, 0, sizeof(u8)*10);

	if(NULL == buf)
	{
		return -1;
	}
	if(NULL == client)
	{
		*buf = 0;
		return -2;
	}

	if(sensor_power == FALSE)
	{
		res = LIS3DH_SetPowerMode(client, true);
		if(res)
		{
			GSE_ERR("Power on lis3dh error %d!\n", res);
		}
		msleep(20);
	}

	if(res = LIS3DH_ReadData(client, obj->data))
	{        
		GSE_ERR("I2C error: ret value=%d", res);
		return -3;
	}
	else
	{
		/*remap coordinate*/
		acc[obj->cvt.map[LIS3DH_AXIS_X]] = obj->cvt.sign[LIS3DH_AXIS_X]*obj->data[LIS3DH_AXIS_X];
		acc[obj->cvt.map[LIS3DH_AXIS_Y]] = obj->cvt.sign[LIS3DH_AXIS_Y]*obj->data[LIS3DH_AXIS_Y];
		acc[obj->cvt.map[LIS3DH_AXIS_Z]] = obj->cvt.sign[LIS3DH_AXIS_Z]*obj->data[LIS3DH_AXIS_Z];
	
		//Out put the mg 
		acc[LIS3DH_AXIS_X] = acc[LIS3DH_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		acc[LIS3DH_AXIS_Y] = acc[LIS3DH_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		acc[LIS3DH_AXIS_Z] = acc[LIS3DH_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;		

#ifndef VENDOR_EDIT

		acc[LIS3DH_AXIS_X] += obj->cali_sw[LIS3DH_AXIS_X];
		acc[LIS3DH_AXIS_Y] += obj->cali_sw[LIS3DH_AXIS_Y];
		acc[LIS3DH_AXIS_Z] += obj->cali_sw[LIS3DH_AXIS_Z];

#else 	

		//when user calibrat gsensor, need original data from lis3dh ,can not add cali_sw data
		if( strcmp(buf,"calibration gsensor") ) 
		{
			acc[LIS3DH_AXIS_X] += obj->cali_sw[LIS3DH_AXIS_X];
			acc[LIS3DH_AXIS_Y] += obj->cali_sw[LIS3DH_AXIS_Y];
			acc[LIS3DH_AXIS_Z] += obj->cali_sw[LIS3DH_AXIS_Z];
		}
		
#endif
		
		sprintf(buf, "%04x %04x %04x", acc[LIS3DH_AXIS_X], acc[LIS3DH_AXIS_Y], acc[LIS3DH_AXIS_Z]);
	}
	
	return 0;
}

/*----------------------------------------------------------------------------*/
static int LIS3DH_ReadRawData(struct i2c_client *client, char *buf)
{
	struct lis3dh_i2c_data *obj = (struct lis3dh_i2c_data*)i2c_get_clientdata(client);
	int res = 0;

	if (!buf || !client)
	{
		return EINVAL;
	}
	
	if(res = LIS3DH_ReadData(client, obj->data))
	{        
		GSE_ERR("I2C error: ret value=%d", res);
		return EIO;
	}
	else
	{
		sprintf(buf, "%04x %04x %04x", obj->data[LIS3DH_AXIS_X], obj->data[LIS3DH_AXIS_Y], obj->data[LIS3DH_AXIS_Z]);
	}
	
	printk(GSE_TAG"%s  \nLIS3DH_AXIS_X=%d\nLIS3DH_AXIS_Y=%d\nLIS3DH_AXIS_Z=%d\n",__func__,
			obj->data[LIS3DH_AXIS_X], obj->data[LIS3DH_AXIS_Y], obj->data[LIS3DH_AXIS_Z]);
	
	return 0;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = lis3dh_i2c_client;
	char strbuf[LIS3DH_BUFSIZE];
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	
	LIS3DH_ReadChipInfo(client, strbuf, LIS3DH_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);        
}

/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = lis3dh_i2c_client;
	char strbuf[LIS3DH_BUFSIZE];
	
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	LIS3DH_ReadSensorData(client, strbuf, LIS3DH_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);            
}

/*----------------------------------------------------------------------------*/
static ssize_t show_cali_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = lis3dh_i2c_client;
	struct lis3dh_i2c_data *obj;
	s16 data[3];
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	obj = i2c_get_clientdata(client);

	int err = 0, len = 0;
	int tmp[LIS3DH_AXES_NUM];
	
	if(err = LIS3DH_ReadCalibration(client, tmp))
	{
		return -EINVAL;
	}
	else
	{    
		calculate_gsensor_cali_data(client,&(obj->cali_sw));
	    printk("calibrate data: (%d %d %d) \n", obj->cali_sw[LIS3DH_AXIS_X], obj->cali_sw[LIS3DH_AXIS_Y], obj->cali_sw[LIS3DH_AXIS_Z]);
	
		return len;
    }
}

/*----------------------------------------------------------------------------*/
static ssize_t store_cali_value(struct device_driver *ddri, char *buf, size_t count)
{
	struct i2c_client *client = lis3dh_i2c_client;  
	int err, x, y, z;
	int dat[LIS3DH_AXES_NUM];

	if(!strncmp(buf, "rst", 3))
	{
		if(err = LIS3DH_ResetCalibration(client))
		{
			GSE_ERR("reset offset err = %d\n", err);
		}	
	}
	else if(3 == sscanf(buf, "0x%02X 0x%02X 0x%02X", &x, &y, &z))
	{
		dat[LIS3DH_AXIS_X] = x;
		dat[LIS3DH_AXIS_Y] = y;
		dat[LIS3DH_AXIS_Z] = z;
		if(err = LIS3DH_WriteCalibration(client, dat))
		{
			GSE_ERR("write calibration err = %d\n", err);
		}		
	}
	else
	{
		GSE_ERR("invalid format\n");
	}
	
	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_power_status(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = lis3dh_i2c_client;
	struct lis3dh_i2c_data *obj;
	u8 data;


	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	obj = i2c_get_clientdata(client);

	data = i2c_smbus_read_byte_data(client, LIS3DH_REG_CTL_REG1);
	
	data &= 0x08;
	data = data>>3;
    return snprintf(buf, PAGE_SIZE, "%x\n", data);
}

/*----------------------------------------------------------------------------*/
static ssize_t show_firlen_value(struct device_driver *ddri, char *buf)
{
#ifdef CONFIG_LIS3DH_LOWPASS
	struct i2c_client *client = lis3dh_i2c_client;
	struct lis3dh_i2c_data *obj = i2c_get_clientdata(client);
	if(atomic_read(&obj->firlen))
	{
		int idx, len = atomic_read(&obj->firlen);
		GSE_LOG("len = %2d, idx = %2d\n", obj->fir.num, obj->fir.idx);

		for(idx = 0; idx < len; idx++)
		{
			GSE_LOG("[%5d %5d %5d]\n", obj->fir.raw[idx][LIS3DH_AXIS_X], obj->fir.raw[idx][LIS3DH_AXIS_Y], obj->fir.raw[idx][LIS3DH_AXIS_Z]);
		}
		
		GSE_LOG("sum = [%5d %5d %5d]\n", obj->fir.sum[LIS3DH_AXIS_X], obj->fir.sum[LIS3DH_AXIS_Y], obj->fir.sum[LIS3DH_AXIS_Z]);
		GSE_LOG("avg = [%5d %5d %5d]\n", obj->fir.sum[LIS3DH_AXIS_X]/len, obj->fir.sum[LIS3DH_AXIS_Y]/len, obj->fir.sum[LIS3DH_AXIS_Z]/len);
	}
	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&obj->firlen));
#else
	return snprintf(buf, PAGE_SIZE, "not support\n");
#endif
}

/*----------------------------------------------------------------------------*/
static ssize_t store_firlen_value(struct device_driver *ddri, char *buf, size_t count)
{
#ifdef CONFIG_LIS3DH_LOWPASS
	struct i2c_client *client = lis3dh_i2c_client;  
	struct lis3dh_i2c_data *obj = i2c_get_clientdata(client);
	int firlen;

	if(1 != sscanf(buf, "%d", &firlen))
	{
		GSE_ERR("invallid format\n");
	}
	else if(firlen > C_MAX_FIR_LENGTH)
	{
		GSE_ERR("exceeds maximum filter length\n");
	}
	else
	{ 
		atomic_set(&obj->firlen, firlen);
		if(NULL == firlen)
		{
			atomic_set(&obj->fir_en, 0);
		}
		else
		{
			memset(&obj->fir, 0x00, sizeof(obj->fir));
			atomic_set(&obj->fir_en, 1);
		}
	}
#endif    
	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct lis3dh_i2c_data *obj = lis3dh_data;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	
	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));     
	return res;    
}

/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri, char *buf, size_t count)
{
	struct lis3dh_i2c_data *obj = lis3dh_data;
	int trace;

	
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&obj->trace, trace);
	}	
	else
	{
		GSE_ERR("invalid content: '%s', length = %d\n", buf, count);
	}

	return count;    
}

/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;    
	struct lis3dh_i2c_data *obj = lis3dh_data;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}	
	
	if(obj->hw)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d %d (%d %d)\n", 
	            obj->hw->i2c_num, obj->hw->direction, obj->hw->power_id, obj->hw->power_vol);   
	}
	else
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}
	return len;    
}

/*----------------------------------------------------------------------------*/
static ssize_t store_dump_register(struct device_driver *ddri, char *buf, size_t count)
{
	struct lis3dh_i2c_data *obj = lis3dh_data;
	int trace;

	printk("%s\n", __func__);
	
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	sscanf(buf, "%d", &trace);
		
	if(1 == trace)
	{
		dumpReg();
	}	
	else
	{
		GSE_ERR("invalid content: '%s', length = %d\n", buf, count);
	}

	return count; 	
}
#ifdef VENDOR_EDIT//mingqiang.guo@Prd.BasicDrv.Sensor,  2012/9/18 add for debug gsensor direction
/*----------------------------------------------------------------------------*/
static ssize_t store_direction_value(struct device_driver *ddri, char *buf, size_t count)
{
	struct lis3dh_i2c_data *obj = lis3dh_data;
	int trace;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "%d", &trace))
	{
		atomic_set(&obj->trace, trace);
	}	
	else
	{
		GSE_ERR("invalid content: '%s', length = %d\n", buf, count);
	}
	 if( hwmsen_get_convert(trace, &obj->cvt))
         {
                GSE_ERR("invalid direction: %d\n", trace);
         }
        printk("set direction %d\n",trace);
return count;    
}
#endif /*VENDOR_EDIT*/


/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(chipinfo,             S_IRUGO, show_chipinfo_value,      NULL);
static DRIVER_ATTR(sensordata,           S_IRUGO, show_sensordata_value,    NULL);
static DRIVER_ATTR(cali,       S_IWUSR | S_IRUGO, show_cali_value,          store_cali_value);
static DRIVER_ATTR(power,                S_IRUGO, show_power_status,          NULL);
static DRIVER_ATTR(firlen,     S_IWUSR | S_IRUGO, show_firlen_value,        store_firlen_value);
static DRIVER_ATTR(trace,      S_IWUSR | S_IRUGO, show_trace_value,         store_trace_value);
static DRIVER_ATTR(status,               S_IRUGO, show_status_value,        NULL);
static DRIVER_ATTR(dump_register,        S_IWUSR, NULL,    					store_dump_register);
#ifdef VENDOR_EDIT//mingqiang.guo@Prd.BasicDrv.Sensor,  2012/9/18 add for debug gsensor direction
static DRIVER_ATTR(direction,      S_IWUSR | S_IRUGO, NULL,  store_direction_value);
#endif /*VENDOR_EDIT*/

/*----------------------------------------------------------------------------*/
static struct driver_attribute *lis3dh_attr_list[] = {
	&driver_attr_chipinfo,     /*chip information*/
	&driver_attr_sensordata,   /*dump sensor data*/
	&driver_attr_cali,         /*show calibration data*/
	&driver_attr_power,        /*show power reg*/
	&driver_attr_firlen,       /*filter length: 0: disable, others: enable*/
	&driver_attr_trace,        /*trace log*/
	&driver_attr_status,        
	&driver_attr_dump_register,
#ifdef VENDOR_EDIT//mingqiang.guo@Prd.BasicDrv.Sensor,  2012/9/18 add for debug gsensor direction
        &driver_attr_direction,
#endif /*VENDOR_EDIT*/
};

/*----------------------------------------------------------------------------*/
static int lis3dh_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(lis3dh_attr_list)/sizeof(lis3dh_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if(err = driver_create_file(driver, lis3dh_attr_list[idx]))
		{            
			GSE_ERR("driver_create_file (%s) = %d\n", lis3dh_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}

/*----------------------------------------------------------------------------*/
static int lis3dh_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(lis3dh_attr_list)/sizeof(lis3dh_attr_list[0]));

	if(driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, lis3dh_attr_list[idx]);
	}

	return err;
}

/*----------------------------------------------------------------------------*/
int gsensor_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value, sample_delay;	
	struct lis3dh_i2c_data *priv = (struct lis3dh_i2c_data*)self;
	hwm_sensor_data* gsensor_data;
	char buff[LIS3DH_BUFSIZE];
	
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				GSE_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;

				printk("%s, SENSOR_DELAY = %d\n", __func__, value);
				
				if(value <= 5)
				{
					sample_delay = LIS3DH_BW_200HZ;
				}
				else if(value <= 10)
				{
					sample_delay = LIS3DH_BW_100HZ;
				}
				else
				{
					sample_delay = LIS3DH_BW_50HZ;
				}
				
				err = LIS3DH_SetBWRate(priv->client, sample_delay);
				if(err != LIS3DH_SUCCESS ) 
				{
					GSE_ERR("Set delay parameter error!\n");
				}

				if(value >= 50)
				{
					atomic_set(&priv->filter, 0);
				}
				else
				{					
					priv->fir.num = 0;
					priv->fir.idx = 0;
					priv->fir.sum[LIS3DH_AXIS_X] = 0;
					priv->fir.sum[LIS3DH_AXIS_Y] = 0;
					priv->fir.sum[LIS3DH_AXIS_Z] = 0;
					atomic_set(&priv->filter, 1);
				}
			}
			break;

		case SENSOR_ENABLE:

			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				GSE_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;

				printk("%s, SENSOR_ENABLE = %d\n", __func__, value);
				
				GSE_LOG("enable value=%d, sensor_power =%d\n",value,sensor_power);
				if(((value == 0) && (sensor_power == false)) ||((value == 1) && (sensor_power == true)))
				{
					GSE_LOG("Gsensor device have updated!\n");
				}
				else
				{
//					err = LIS3DH_SetPowerMode( priv->client, !sensor_power);

#ifdef OPPO_MOTION_SENSITIVE_GSENSOR

					LIS3DH_SetPowerMode(priv->client, true);

#endif					
				}
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				GSE_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				gsensor_data = (hwm_sensor_data *)buff_out;
				
				LIS3DH_ReadSensorData(priv->client, buff, LIS3DH_BUFSIZE);
				
				sscanf(buff, "%x %x %x", &gsensor_data->values[0], 
					&gsensor_data->values[1], &gsensor_data->values[2]);	
				
				gsensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;				
				gsensor_data->value_divide = 1000;
				
//				printk("[Gsensor]:x= %6d, y = %6d, z = %6d\n", gsensor_data->values[0],gsensor_data->values[1],gsensor_data->values[2]);
			}
			break;
		default:
			GSE_ERR("gsensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}

static int calculate_gsensor_cali_data(struct i2c_client *client, s16 data[3])
{
	u8 i = 0;
	s32 average_offset[3] = {0};
	s32 read_buff[3] = {0};
	
	int res = -1;
	char buff[LIS3DH_BUFSIZE];
	struct lis3dh_i2c_data *obj = (struct lis3dh_i2c_data*)i2c_get_clientdata(client);	

	if (NULL == client)
	{	
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}

	printk("/*****************************************/\n");
	
	for (i = 0; i < 20; i ++)
	{		
		strcpy(buff,"calibration gsensor");
		if(res = LIS3DH_ReadSensorData(client, buff,LIS3DH_BUFSIZE))
		{        
			GSE_ERR("I2C error: ret value=%d", res);
			return -EIO;
		}
		
		sscanf(buff, "%x %x %x", &read_buff[0],&read_buff[1],&read_buff[2] );
		
		printk("calculate_gsensor: (%5d %5d %5d)\n",read_buff[0],read_buff[1],read_buff[2]);
		
		average_offset[0] += read_buff[0];
		average_offset[1] += read_buff[1];	
		average_offset[2] += read_buff[2];	
		msleep(20);
	}
	
	average_offset[0] /= 20;
	average_offset[1] /= 20;
	average_offset[2] /= 20;
	
	obj->cali_sw[0]=0;
	obj->cali_sw[1]=0;
	obj->cali_sw[2]=0;	

	obj->cali_sw[0] = 0-average_offset[0];
	obj->cali_sw[1] = 0-average_offset[1] ;
	obj->cali_sw[2] = 9800-average_offset[2] ;

	data[0] = obj->cali_sw[0];
	data[1] = obj->cali_sw[1];
	data[2] = obj->cali_sw[2];

	printk("\ngsensor offset: (%5d, %5d, %5d)\n\n", obj->cali_sw[0], obj->cali_sw[1], obj->cali_sw[2]);	

	printk("/*****************************************/\n");

	return 0;	
}

/*----------------------------------------------------------------------------*/
static int lis3dh_open(struct inode *inode, struct file *file)
{
	file->private_data = lis3dh_i2c_client;

	if(file->private_data == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}

/*----------------------------------------------------------------------------*/
static int lis3dh_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

/*----------------------------------------------------------------------------*/
static long lis3dh_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct lis3dh_i2c_data *obj = (struct lis3dh_i2c_data*)i2c_get_clientdata(client);	
	char strbuf[LIS3DH_BUFSIZE];
	void __user *data;
	SENSOR_DATA sensor_data;
	long err = 0;
	int cali[3];

	//GSE_FUN(f);
	if(_IOC_DIR(cmd) & _IOC_READ)
	{
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	}
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
	{
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	}

	if(err)
	{
		GSE_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch(cmd)
	{
		case GSENSOR_IOCTL_INIT:
			LIS3DH_Init_client(client, 0);			
			break;

		case GSENSOR_IOCTL_READ_CHIPINFO:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			
			LIS3DH_ReadChipInfo(client, strbuf, LIS3DH_BUFSIZE);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;
			}				 
			break;	  

		case GSENSOR_IOCTL_READ_SENSORDATA:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
				LIS3DH_ReadSensorData(client, strbuf, LIS3DH_BUFSIZE);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;	  
			}				 
			break;

		case GSENSOR_IOCTL_READ_GAIN:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			
			
			if(copy_to_user(data, &gsensor_gain, sizeof(GSENSOR_VECTOR3D)))
			{
				err = -EFAULT;
				break;
			}				 
			break;

		case GSENSOR_IOCTL_READ_OFFSET:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			
			if(copy_to_user(data, &gsensor_offset, sizeof(GSENSOR_VECTOR3D)))
			{
				err = -EFAULT;
				break;
			}				 
			break;

		case GSENSOR_IOCTL_READ_RAW_DATA:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			LIS3DH_ReadRawData(client, &strbuf);
			if(copy_to_user(data, &strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;	  
			}
			break;	  

		case GSENSOR_IOCTL_SET_CALI:
			data = (void __user*)arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			if(copy_from_user(&sensor_data, data, sizeof(sensor_data)))
			{
				err = -EFAULT;
				break;	  
			}
			if(atomic_read(&obj->suspend))
			{
				GSE_ERR("Perform calibration in suspend state!!\n");
				err = -EINVAL;
			}
			else
			{
				printk("%s: GSENSOR_IOCTL_SET_CALI\n",__func__);
				
				cali[LIS3DH_AXIS_X] = sensor_data.x ;
				cali[LIS3DH_AXIS_Y] = sensor_data.y ;
				cali[LIS3DH_AXIS_Z] = sensor_data.z ;	
				
				err = LIS3DH_WriteCalibration(client, cali);			 
			}
			break;

		case GSENSOR_IOCTL_CLR_CALI:
			err = LIS3DH_ResetCalibration(client);
			break;

		case GSENSOR_IOCTL_GET_CALI:
			data = (void __user*)arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			if(err = LIS3DH_ReadCalibration(client, cali))
			{
				break;
			}

			cali[LIS3DH_AXIS_X] = sensor_data.x ;
			cali[LIS3DH_AXIS_Y] = sensor_data.y ;
			cali[LIS3DH_AXIS_Z] = sensor_data.z ;	

			if(copy_to_user(data, &sensor_data, sizeof(sensor_data)))
			{
				err = -EFAULT;
				break;
			}		
			break;
		
	#ifdef VENDOR_EDIT	// for user calibrate gsensor in engineering mode
		case GSENSOR_IOCTL_USER_CALI:
			{
				int buf[3];
                s16 cali_data[3];
				data = (void __user *) arg;
                err = calculate_gsensor_cali_data(client, &cali_data);
				buf[0] = (int)(cali_data[0]);
				buf[1] = (int)(cali_data[1]);
				buf[2] = (int)(cali_data[2]);
				copy_to_user(data, &buf, 3 * sizeof(int));
			}
		break;
	#endif 

		default:
			GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
			err = -ENOIOCTLCMD;
			break;
			
	}

	return err;
}

/*----------------------------------------------------------------------------*/
static struct file_operations lis3dh_fops = 
{
	.owner = THIS_MODULE,
	.open = lis3dh_open,
	.release = lis3dh_release,
	//.ioctl = lis3dh_ioctl,
	.unlocked_ioctl = lis3dh_unlocked_ioctl,
};

/*----------------------------------------------------------------------------*/
static struct miscdevice lis3dh_device = 
{
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gsensor",
	.fops = &lis3dh_fops,
};

/*----------------------------------------------------------------------------*/
#ifndef CONFIG_HAS_EARLYSUSPEND
/*----------------------------------------------------------------------------*/
static int lis3dh_suspend(struct i2c_client *client, pm_message_t msg) 
{
	struct lis3dh_i2c_data *obj = i2c_get_clientdata(client);    
	int err = 0;
	u8 dat;
	GSE_FUN();    

	if(msg.event == PM_EVENT_SUSPEND)
	{   
		if(obj == NULL)
		{
			GSE_ERR("null pointer!!\n");
			return -EINVAL;
		}
		
		//read old data
		if ((err = hwmsen_read_byte(client, LIS3DH_REG_CTL_REG1, &dat))) 
		{
           GSE_ERR("write data format fail!!\n");
           return err;
        }
		dat = dat&0b10111111;
		atomic_set(&obj->suspend, 1);
		if(err = hwmsen_write_byte(client, LIS3DH_REG_CTL_REG1, dat))
		{
			GSE_ERR("write power control fail!!\n");
			return err;
		}        
		LIS3DH_power(obj->hw, 0);
	}
	return err;
}
/*----------------------------------------------------------------------------*/
static int lis3dh_resume(struct i2c_client *client)
{
	struct lis3dh_i2c_data *obj = i2c_get_clientdata(client);        
	int err;
	GSE_FUN();

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}

	LIS3DH_power(obj->hw, 1);
	if(err = LIS3DH_Init_client(client, 0))
	{
		GSE_ERR("initialize client fail!!\n");
		return err;        
	}
	atomic_set(&obj->suspend, 0);

	return 0;
}

#else /*CONFIG_HAS_EARLY_SUSPEND is defined*/

/*----------------------------------------------------------------------------*/
static void lis3dh_early_suspend(struct early_suspend *h) 
{
	struct lis3dh_i2c_data *obj = container_of(h, struct lis3dh_i2c_data, early_drv);   
	int err = 0;
	
	GSE_FUN();    
	printk("%s  Enter !!!\n",__func__);

	u8 databuf[2];
	u8 addr = LIS3DH_INT1_SRC;	// 0x31

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return;
	}
	
	atomic_set(&obj->suspend, 1); 

#ifdef OPPO_MOTION_SENSITIVE_GSENSOR

	// sensors not suspend( motion sensing ), G sensor works normally 
	if( sensor_suspend == 0 )	
	{
		// this flag is used to make sure : first calling state and then suspend.
		atomic_set(&obj->sensor_suspend_flag, 1);
	
		printk("%s: Need motion sensing , G-sensor not suspend and works normally !\n", __func__);

		if(whether_in_call_state())
		{
			printk("%s: Now, in a call !\n", __func__);

			atomic_set(&obj->in_call_state, 1);
			
			return ;
		}
		else if(atomic_read(&obj->interrupt_enable))	// there is un-answered call or un-read note
		{
			printk("%s : There is un-answered call or un-read note , enable G-sensor interrupt in suspend ! \n", __func__);

			// enable gsensor interrupt
            gsensor_motion_detect_init(obj->client);
			gsensor_motion_enable(obj->client, true);

			// this flag is used to make sure : first un-handle message and then suspend.
			// filter out this situation : first suspend and then message comes
			atomic_set(&obj->senosr_interrupt, 1); 	

			return ;
		}

		// not in a call state and do not have un-answered call or un-read note
		return ;
	}
	else
	{
		printk("%s, motion sensing switch closed , G-sensor will suspend !\n", __func__);
	}
	
#endif

	if(err = LIS3DH_SetPowerMode(obj->client, false))
	{
		GSE_ERR("write power control fail!!\n");
		return;
	}

	sensor_power = false;

	// now , we must mask G-sensor interrupt flag , or power down the gsensor ,  
	// the interrupt gpio will get to 0 from 1, and will trigger the interrupt forever.

	mt65xx_eint_mask(CUST_EINT_GSE_1_NUM);

	err = cancel_work_sync(&lis3dh_data->work);
	if(err) 
	{
		printk("%s, cannot disable work\n", __func__);
	}
	
	LIS3DH_power(obj->hw, 0);

	printk("%s	Leave !!!\n",__func__);	
}

/*----------------------------------------------------------------------------*/
static void lis3dh_late_resume(struct early_suspend *h)
{
	struct lis3dh_i2c_data *obj = container_of(h, struct lis3dh_i2c_data, early_drv);         
	int err;
	int databuf;
	
	GSE_FUN();
	printk("%s  Enter !!!\n",__func__);
	
	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return;
	}

	atomic_set(&obj->suspend, 0); 

#ifdef OPPO_MOTION_SENSITIVE_GSENSOR

	// sensors not suspend( motion sensing ), G sensor works normally 
	if(atomic_read(&obj->sensor_suspend_flag))	
	{
		atomic_set(&obj->sensor_suspend_flag, 0);
		
		printk("%s: Need motion sensing , G-sensor not suspend and works normally !\n", __func__);

		if(atomic_read(&obj->in_call_state))
		{
			printk("%s: Now, in a call !\n", __func__);

			atomic_set(&obj->in_call_state, 0);
			
			return ;
		}
		else if(atomic_read(&obj->senosr_interrupt))
		{
			printk("%s : There is un-answered call or un-read note , G-sensor works normally and disable G-sensor interrupt in resume ! \n", __func__);

			atomic_set(&obj->senosr_interrupt, 0); 

			// close gsensor interrupt when resume , it means that the interrupt only be enabled when suspend
			gsensor_motion_enable(lis3dh_i2c_client, false);

			// read interrupt source to clear interrupt latch to enable the next interrupt
			databuf = i2c_smbus_read_byte_data(lis3dh_i2c_client, LIS3DH_INT1_SRC);
			printk("%s : Read interrupt source, and the value of LIS3DH_INT1_SRC = 0x%x !\n", __func__, databuf); 

			return ;
		}

		// not in a call state and do not have un-answered call or un-read note
		return ;	
	}
	else
	{
		printk("%s, motion sensing switch closed , G-sensor will resume !\n", __func__);
	}
	
#endif

	LIS3DH_power(obj->hw, 1);

	if(err = LIS3DH_Init_client(obj->client, 0))
	{
		GSE_ERR("initialize client fail!!\n");
		return;        
	}

	mt65xx_eint_unmask(CUST_EINT_GSE_1_NUM);
	
	printk("%s  Leave !!!\n",__func__);
}
/*----------------------------------------------------------------------------*/
#endif /*CONFIG_HAS_EARLYSUSPEND*/

/*----------------------------------------------------------------------------*/
static int lis3dh_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) 
{    
	strcpy(info->type, LIS3DH_DEV_NAME);
	return 0;
}

#ifdef OPPO_MOTION_SENSITIVE_GSENSOR

/*----------------------------------------------------------------------------*/
static void gsensor_irq_handler(struct work_struct *work)
{

	mt65xx_eint_mask(CUST_EINT_GSE_1_NUM);

	queue_work(gsensor_wq, &lis3dh_data->work);	
}

/*----------------------------------------------------------------------------*/
static void gsensor_irq_handle_work_fun(struct work_struct *work)
{
	u8 data;

	// suspend state
	// there is un-answered call or un-read note	
	// when in a calling state , do not wake up 
	if(atomic_read(&lis3dh_data->suspend) 
		&& atomic_read(&lis3dh_data->interrupt_enable) 
		&& (whether_in_call_state() == 0))
	{
		if(whether_in_approach_state())	// something above the phone
		{
			// read interrupt source to clear interrupt latch to enable the next interrupt
			i2c_smbus_read_byte_data(lis3dh_i2c_client, LIS3DH_INT1_SRC);
			printk("Read interrupt source to clear interrupt for next interrupt!\n");
		}
		else	// nothing above the phone
		{
			atomic_set(&lis3dh_data->interrupt_true, 1);	

			msleep(2);
			printk(" AAAAAAAAAAAA===BBBBBBBBBBB===CCCCCCCCCCC \n");
			printk("/************* Simulate power key pressed for motion sensing! *************/ \n");		

			input_report_key(lis3dh_data->input_dev, KEY_POWER, 1);
			input_sync(lis3dh_data->input_dev);	
			input_report_key(lis3dh_data->input_dev, KEY_POWER, 0);
			input_sync(lis3dh_data->input_dev);
		}
	}
	
	mt65xx_eint_unmask(CUST_EINT_GSE_1_NUM);	
}


/*----------------------------------------------------------------------------*/
static void gsensor_interrupt_gpio_config(void)
{
    mt_set_gpio_mode(GPIO_GSENSOR_EINT_PIN, GPIO_MODE_01);
    mt_set_gpio_dir(GPIO_GSENSOR_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_GSENSOR_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_GSENSOR_EINT_PIN, GPIO_PULL_UP);

	mt65xx_eint_set_polarity(CUST_EINT_GSE_1_NUM, CUST_EINT_GSE_1_POLARITY);
	mt65xx_eint_set_sens(CUST_EINT_GSE_1_NUM, CUST_EINT_GSE_1_SENSITIVE);
	mt65xx_eint_set_hw_debounce(CUST_EINT_GSE_1_NUM, CUST_EINT_GSE_1_DEBOUNCE_EN);
	mt65xx_eint_registration(CUST_EINT_GSE_1_NUM, CUST_EINT_GSE_1_DEBOUNCE_EN, 
							CUST_EINT_GSE_1_POLARITY, gsensor_irq_handler, 0);
	
	mt65xx_eint_unmask(CUST_EINT_GSE_1_NUM);	

	enable_irq_wake(CUST_EINT_GSE_1_NUM);

	printk("Gsensor INT gpio config over. \n");
}

/*----------------------------------------------------------------------------*/
//enable data ready interrupt
static int LIS3DH_SetIntEnable(struct i2c_client *client, u8 intenable)
{
	u8 databuf[2];
	u8 addr = LIS3DH_REG_CTL_REG3;
	int res = 0;

	if(intenable == false)	
	{
		databuf[1] = 0x00;
		databuf[0] = LIS3DH_REG_CTL_REG3;		// 0x22
		res = i2c_master_send(client, databuf, 0x2);

		if(res <= 0)
		{
			GSE_ERR("REG[0X%x] write 0x%x error\n", addr, databuf[1]);
			return LIS3DH_ERR_I2C;
		}
	}
	else
	{
		databuf[1] = 0x40;
		databuf[0] = LIS3DH_REG_CTL_REG3;		// 0x22
		res = i2c_master_send(client, databuf, 0x2);
	
		if(res <= 0)
		{
			GSE_ERR("REG[0X%x] write 0x%x error\n", addr, databuf[1]);
			return LIS3DH_ERR_I2C;
		}

	}
	
	return LIS3DH_SUCCESS;    
}

/*----------------------------------------------------------------------------*/

static void gsensor_motion_detect_init(struct i2c_client *client)
{
	u8 databuf[2] = {0}; 
	int res = 0;

/*

	databuf[0] = LIS3DH_REG_CTL_REG2;		// 0x21
	databuf[1] = GSENSOR_HIGHPASS_MODE; 	// 0x01
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GSE_ERR("Write MPU6050C_INT_ENABLE register err!\n");
		return LIS3DH_ERR_I2C;
	}
	
	hwmsen_read_byte(client, LIS3DH_REFERENCE, &databuf[0]);
	printk("The value of LIS3DH_REFERENCE = 0x%x !\n", databuf[0]);
*/

	databuf[0] = LIS3DH_REG_CTL_REG5;	 	// 0x24
	databuf[1] = GSENSOR_LIR_INT1;			// 0x08
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GSE_ERR("Write MPU6050C_INT_ENABLE register err!\n");
		return LIS3DH_ERR_I2C;
	}

	databuf[0] = LIS3DH_REG_CTL_REG6;	 	// 0x25
	databuf[1] = GSENSOR_H_LACTIVE;			// 0x02
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GSE_ERR("Write MPU6050C_INT_ENABLE register err!\n");
		return LIS3DH_ERR_I2C;
	}

	databuf[0] = LIS3DH_INT1_THS;			// 0x32
	databuf[1] = GSENSOR_INT1_THS_VALUE;	// 0x06
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GSE_ERR("Write MPU6050C_ACCEL_CONFIG register err!\n");
		return LIS3DH_ERR_I2C;
	}
	
	databuf[0] = LIS3DH_INT1_DURATION;			// 0x33
	databuf[1] = GSENSOR_INT1_DURATION_VALUE;	// 0x00
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GSE_ERR("Write MPU6050C_ACCEL_CONFIG register err!\n");
		return LIS3DH_ERR_I2C;
	}	

	databuf[1] = 0x40;
	databuf[0] = LIS3DH_REG_CTL_REG3;			// 0x22
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GSE_ERR("REG[0X%x] write 0x%x error\n", LIS3DH_REG_CTL_REG3, databuf[1]);
		return LIS3DH_ERR_I2C;
	}	
}

/*----------------------------------------------------------------------------*/
static void gsensor_motion_enable(struct i2c_client *client, u8 enable)
{
	u8 databuf[2] = {0}; 
	int res = 0;

	if(enable)
	{
		databuf[0] = LIS3DH_INT1_CFG;			// 0x30
		databuf[1] = GSENSOR_INT1_DESCRIPTION;	// 0x0a
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			GSE_ERR("Write MPU6050C_INT_ENABLE register err!\n");
			return LIS3DH_ERR_I2C;
		}
	}
	else
	{
		databuf[0] = LIS3DH_INT1_CFG;			// 0x30
		databuf[1] = 00;						// 0x0a
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			GSE_ERR("Write MPU6050C_INT_ENABLE register err!\n");
			return LIS3DH_ERR_I2C;
		}		
	}
}

/*----------------------------------------------------------------------------*/
static ssize_t gesture_enable_read(struct file *file, char __user *buf, size_t count, loff_t *pos)
{
    char page[1024]; 	
    char *p = page;	
    int len = 0; 

	struct lis3dh_i2c_data *obj = lis3dh_data;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}

    p += sprintf(p, "%d\n", atomic_read(&obj->interrupt_enable));	
    
    len = p - page;	
    if (len > *pos)		
        len -= *pos;	
    else		
        len = 0;	

    if (copy_to_user(buf, page, len < count ? len  : count))		
        return -EFAULT;	

    *pos = *pos + (len < count ? len  : count);	

	printk(" %s , len == %d, count = %d\n", __func__, len, count);

    return len < count ? len  : count;	
}

static ssize_t gesture_enable_write(struct file *file, char __user *buf, size_t count, loff_t *pos)
{
    char tmp[128] = {0};	
    int ret;		
    int reg, val;
	int interrupt_enable = 0;
	
	struct lis3dh_i2c_data *obj = lis3dh_data;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}

    ret = copy_from_user(tmp, buf, 16);	

	if(1 == sscanf(tmp, "%d", &interrupt_enable))
	{
		printk("%s, write %d to gesture_enable file. \n", __func__, interrupt_enable);

		atomic_set(&obj->interrupt_enable, interrupt_enable);
	}	
	else
	{
		GSE_ERR("invalid content: '%s', length = %d\n", tmp, count);
	}

    return count;	
}

static ssize_t gesture_true_read(struct file *file, char __user *buf, size_t count, loff_t *pos)
{
    char page[1024]; 	
    char *p = page;	
    int len = 0; 

	struct lis3dh_i2c_data *obj = lis3dh_data;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}

    p += sprintf(p, "%d\n", atomic_read(&obj->interrupt_true));	
    
    len = p - page;	
    if (len > *pos)		
        len -= *pos;	
    else		
        len = 0;	

    if (copy_to_user(buf, page, len < count ? len  : count))		
        return -EFAULT;	

    *pos = *pos + (len < count ? len  : count);	

	printk(" %s , len == %d, count = %d\n", __func__, len, count);

    return len < count ? len  : count;	
}

static ssize_t gesture_true_write(struct file *file, char __user *buf, size_t count, loff_t *pos)
{
    char tmp[128] = {0};	
    int ret;		
    int reg, val;
	int interrupt_true = 0;
	
	struct lis3dh_i2c_data *obj = lis3dh_data;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}

    ret = copy_from_user(tmp, buf, 16);	

	if(1 == sscanf(tmp, "%d", &interrupt_true))
	{
		printk("%s, write %d to gesture_true file. \n", __func__, interrupt_true);
	
		atomic_set(&obj->interrupt_true, interrupt_true);
	}	
	else
	{
		GSE_ERR("invalid content: '%s', length = %d\n", tmp, count);
	}

    return count;	
}

static struct file_operations gesture_enable_fops = 
{
    .write = gesture_enable_write,   
    .read = gesture_enable_read,
};

static struct file_operations gesture_true_fops = 
{
    .write = gesture_true_write,   
    .read = gesture_true_read,
};

static void proc_file_create()
{
	lis3dh_data->entry = proc_mkdir_mode("gyro_gsensor", 0777, NULL);
	// The mode must be 777, or the proc file may not be opened.

	proc_create("gesture_enable", 0666, lis3dh_data->entry, &gesture_enable_fops);
	proc_create("gesture_true", 0666, lis3dh_data->entry, &gesture_true_fops);
}

#endif	/*end of OPPO_MOTION_SENSITIVE_GSENSOR*/

/*----------------------------------------------------------------------------*/
static int lis3dh_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_client *new_client;
	struct lis3dh_i2c_data *obj;
	struct hwmsen_object sobj;
	int err = 0;
	GSE_FUN();

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	
	memset(obj, 0, sizeof(struct lis3dh_i2c_data));

	printk("Lis3dh addr is 0x%x!\n", client->addr);

	obj->hw = get_cust_acc_hw_lis3dh();
	
	if(err = hwmsen_get_convert(obj->hw->direction, &obj->cvt))
	{
		GSE_ERR("invalid direction: %d\n", obj->hw->direction);
		goto exit;
	}

	LIS3DH_power(obj->hw, 1);

	lis3dh_data = obj;
	obj->client = client;
	new_client = obj->client;
	i2c_set_clientdata(new_client,obj);
	
	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);
	atomic_set(&obj->interrupt_enable, 0);
	atomic_set(&obj->senosr_interrupt, 0);
	atomic_set(&obj->sensor_suspend_flag, 0);
	atomic_set(&obj->in_call_state, 0);


#ifdef CONFIG_LIS3DH_LOWPASS
	if(obj->hw->firlen > C_MAX_FIR_LENGTH)
	{
		atomic_set(&obj->firlen, C_MAX_FIR_LENGTH);
	}	
	else
	{
		atomic_set(&obj->firlen, obj->hw->firlen);
	}
	
	if(atomic_read(&obj->firlen) > 0)
	{
		atomic_set(&obj->fir_en, 1);
	}
	
#endif

	lis3dh_i2c_client = new_client;	

#ifndef VENDOR_EDIT	// modify for gsensor init error
	if(err = LIS3DH_Init_client(new_client, 1))
	{
		printk("LIS3DH_Init_client failed, not lis3dh or bad chip!\n");	
		goto exit_init_failed;
	}
#else
	{ 
		int i = 3;
		while(err = LIS3DH_Init_client(new_client, 1))
		{
			GSE_LOG("LIS3DH_Init_client err=%d NO=%d \n",err,i);
			msleep(100);
			
			i--;
			if(i == 0)
			{
				printk("LIS3DH_Init_client failed, not lis3dh or bad chip!\n");	
				goto exit_init_failed;
			}
		}
	}
#endif

	if(err = misc_register(&lis3dh_device))
	{
		GSE_ERR("lis3dh_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	if(err = lis3dh_create_attr(&lis3dh_gsensor_driver.driver))
	{
		GSE_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}

	sobj.self = obj;
    sobj.polling = 1;
    sobj.sensor_operate = gsensor_operate;
	if(err = hwmsen_attach(ID_ACCELEROMETER, &sobj))
	{
		GSE_ERR("attach fail = %d\n", err);
		goto exit_kfree;
	}

#ifdef OPPO_MOTION_SENSITIVE_GSENSOR

    gsensor_wq = create_singlethread_workqueue("gsensor_wq");

	obj->input_dev = input_allocate_device();
	obj->input_dev->name = "sensor-powerkey";
	set_bit(EV_KEY, obj->input_dev->evbit);
	set_bit(KEY_POWER, obj->input_dev->keybit);
	set_bit(EV_SYN, obj->input_dev->evbit);
	
	if(input_register_device(obj->input_dev))
	{
		printk("Gyro or G-sensor failed to register input device\n");
	}

	gsensor_interrupt_gpio_config();

	gsensor_motion_detect_init(new_client);

	gsensor_motion_enable(new_client, false);
	
	INIT_WORK(&obj->work, gsensor_irq_handle_work_fun);

	proc_file_create();
	
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1,
	obj->early_drv.suspend  = lis3dh_early_suspend,
	obj->early_drv.resume   = lis3dh_late_resume,    
	register_early_suspend(&obj->early_drv);
#endif 

	hwmsen_make_debug_flag(&debug_fops, "lis3dh_gsensor");

	printk("%s: OK\n", __func__);

	return 0;

	exit_create_attr_failed:
	misc_deregister(&lis3dh_device);
	exit_misc_device_register_failed:
	exit_init_failed:
	printk("Lis3dh probe failed , power down!\n");		
	LIS3DH_power(obj->hw, 0);
	exit_kfree:
	kfree(obj);
	exit:
	GSE_ERR("%s: err = %d\n", __func__, err);        
	return err;
}

/*----------------------------------------------------------------------------*/
static int lis3dh_i2c_remove(struct i2c_client *client)
{
	int err = 0;	
	
	if(err = lis3dh_delete_attr(&lis3dh_gsensor_driver.driver))
	{
		GSE_ERR("lis3dh_delete_attr fail: %d\n", err);
	}
	
	if(err = misc_deregister(&lis3dh_device))
	{
		GSE_ERR("misc_deregister fail: %d\n", err);
	}

	if(err = hwmsen_detach(ID_ACCELEROMETER))
	    

	lis3dh_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	return 0;
}
/*----------------------------------------------------------------------------*/
static int lis3dh_probe(struct platform_device *pdev) 
{
	GSE_FUN();
	printk("%s\n", __func__);

	if(i2c_add_driver(&lis3dh_i2c_driver))
	{
		GSE_ERR("add driver error\n");
		return -1;
	}
	return 0;
}

/*----------------------------------------------------------------------------*/
static int lis3dh_remove(struct platform_device *pdev)
{
    struct acc_hw *hw = get_cust_acc_hw_lis3dh();

    GSE_FUN();    
    LIS3DH_power(hw, 0);    
    i2c_del_driver(&lis3dh_i2c_driver);
    return 0;
}

/*----------------------------------------------------------------------------*/
static struct platform_driver lis3dh_gsensor_driver = 
{
	.probe      = lis3dh_probe,
	.remove     = lis3dh_remove,    
	.driver     = 
	{
		#ifndef VENDOR_EDIT	//add for muti gsensor
		.name  = "gsensor",
		#else
		.name  = "gsensor_lis3dh",
		#endif
		.owner = THIS_MODULE,
	}
};

/*----------------------------------------------------------------------------*/
static int __init lis3dh_init(void)
{
	GSE_FUN();
	i2c_register_board_info(2, &i2c_LIS3DH, 1);
	if(platform_driver_register(&lis3dh_gsensor_driver))
	{
		GSE_ERR("failed to register driver");
		return -ENODEV;
	}
	return 0;    
}

/*----------------------------------------------------------------------------*/
static void __exit lis3dh_exit(void)
{
	GSE_FUN();
	platform_driver_unregister(&lis3dh_gsensor_driver);
}

/*----------------------------------------------------------------------------*/
module_init(lis3dh_init);
module_exit(lis3dh_exit);

/*----------------------------------------------------------------------------*/
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("LIS3DH I2C driver");
MODULE_AUTHOR("Chunlei.Wang@mediatek.com");

