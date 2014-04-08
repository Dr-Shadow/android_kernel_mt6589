/* MPU6050C motion sensor driver
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

#include <cust_gyro.h>
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include "mpu6050c.h"
#include <linux/hwmsen_helper.h>
#include <linux/kernel.h>


#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>

#include <mach/mt_boot.h>
#include <cust_eint.h>
#include <mach/eint.h>

#include <linux/proc_fs.h>

#include <linux/oppo_devices_list.h>

extern volatile GYRO_DEV gyro_dev;

//#define MOTION_SENSITIVE_GSENSOR 

static struct proc_dir_entry *proc_entry_calibration = NULL; 

/*----------------------------------------------------------------------------*/
#define CONFIG_MPU6050C_LOWPASS   /*apply low pass filter on output*/       

/*----------------------------------------------------------------------------*/
#define MPU6050C_AXIS_X          0
#define MPU6050C_AXIS_Y          1
#define MPU6050C_AXIS_Z          2
#define MPU6050C_AXES_NUM        3
#define MPU6050C_DATA_LEN        6   
#define MPU6050C_DEV_NAME        "MPU6050C"
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id mpu6050c_i2c_id[] = {{MPU6050C_DEV_NAME,0},{}};
static struct i2c_board_info __initdata i2c_mpu6050c={ I2C_BOARD_INFO("MPU6050C", (0xD0>>1))};

static int packet_thresh = 75; // 600 ms / 8ms/sample

static struct workqueue_struct *gyro_wq = NULL;

/*----------------------------------------------------------------------------*/
static int mpu6050c_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int mpu6050c_i2c_remove(struct i2c_client *client);
static int MPU6050C_SetPowerMode(struct i2c_client *client, bool enable);
static void MPU6050C_gsensor_z_axis_active();
static void MPU6050C_gsensor_standby();
static void MPU6050C_gyro_standby();
static void MPU6050C_active_all_axis();

/*----------------------------------------------------------------------------*/
typedef enum {
    GYRO_TRC_FILTER  = 0x01,
    GYRO_TRC_RAWDATA = 0x02,
    GYRO_TRC_IOCTL   = 0x04,
    GYRO_TRC_CALI	= 0X08,
    GYRO_TRC_INFO	= 0X10,
    GYRO_TRC_DATA	= 0X20,
} GYRO_TRC;

/*----------------------------------------------------------------------------*/
struct scale_factor{
    u8  whole;
    u8  fraction;
};

/*----------------------------------------------------------------------------*/
struct data_resolution {
    struct scale_factor scalefactor;
    int                 sensitivity;
};

/*----------------------------------------------------------------------------*/
#define C_MAX_FIR_LENGTH (32)

/*----------------------------------------------------------------------------*/
struct data_filter {
    s16 raw[C_MAX_FIR_LENGTH][MPU6050C_AXES_NUM];
    int sum[MPU6050C_AXES_NUM];
    int num;
    int idx;
};

/*----------------------------------------------------------------------------*/
struct mpu6050c_i2c_data {
    struct i2c_client *client;
    struct gyro_hw *hw;
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
    s16                     cali_gyro[MPU6050C_AXES_NUM+1];
	s16						cali_gsensor[MPU6050C_AXES_NUM+1];

    /*data*/
    s8                      offset[MPU6050C_AXES_NUM+1];  /*+1: for 4-byte alignment*/
    s16                     data[MPU6050C_AXES_NUM+1];

#if defined(CONFIG_MPU6050C_LOWPASS)
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
static struct i2c_driver mpu6050c_i2c_driver = 
{
    .driver = 
	{
//		.owner          = THIS_MODULE,//modified
		.name           = MPU6050C_DEV_NAME,
    },
	.probe      		= mpu6050c_i2c_probe,
	.remove    			= mpu6050c_i2c_remove,
	
#if !defined(CONFIG_HAS_EARLYSUSPEND)    
    .suspend            = mpu6050c_suspend,
    .resume             = mpu6050c_resume,
#endif

	.id_table = mpu6050c_i2c_id,

};

/*----------------------------------------------------------------------------*/
static struct i2c_client *mpu6050c_i2c_client = NULL;
static struct platform_driver mpu6050c_gyro_driver;
static struct mpu6050c_i2c_data *mpu6050c_data = NULL;
static bool sensor_power = false;

/*----------------------------------------------------------------------------*/
#define GYRO_TAG                  "[Gyroscope] "
#define GYRO_FUN(f)               printk(KERN_INFO GYRO_TAG"%s\n", __FUNCTION__)
#define GYRO_ERR(fmt, args...)    printk(KERN_ERR GYRO_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define GYRO_LOG(fmt, args...)    printk(KERN_INFO GYRO_TAG fmt, ##args)

/*--------------------gyroscopy power control function------------------------*/
static void MPU6050C_power(struct gyro_hw *hw, unsigned int on) 
{
	hw->power(hw, on, NULL);
}

/*----------------------------------------------------------------------------*/
static int MPU6050C_Gsensor_ResetCalibration(struct i2c_client *client)
{
	struct mpu6050c_i2c_data *obj = i2c_get_clientdata(client);	

	memset(obj->cali_gsensor, 0x00, sizeof(obj->cali_gsensor));
	return 0;    
}

/*----------------------------------------------------------------------------*/
static int MPU6050C_Gyro_ResetCalibration(struct i2c_client *client)
{
	struct mpu6050c_i2c_data *obj = i2c_get_clientdata(client);	

	memset(obj->cali_gyro, 0x00, sizeof(obj->cali_gyro));
	return 0;    
}

/*----------------------------------------------------------------------------*/
static int MPU6050C_ReadCalibration(struct i2c_client *client, int dat[MPU6050C_AXES_NUM])
{
                                   
    return 0;
}

/*----------------------------------------------------------------------------*/
static int MPU6050C_Write_Gsensor_Calibration_data(struct i2c_client *client, int dat[MPU6050C_AXES_NUM])
{
	struct mpu6050c_i2c_data *obj = i2c_get_clientdata(client);

	obj->cali_gsensor[MPU6050C_AXIS_X] = dat[0];
	obj->cali_gsensor[MPU6050C_AXIS_Y] = dat[1];
	obj->cali_gsensor[MPU6050C_AXIS_Z] = dat[2];

	return 0;
}


/*----------------------------------------------------------------------------*/
static int MPU6050C_WriteCalibration(struct i2c_client *client, int dat[MPU6050C_AXES_NUM])
{
	struct mpu6050c_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;
	int cali[MPU6050C_AXES_NUM];


	GYRO_FUN();
	if(!obj || ! dat)
	{
		GYRO_ERR("null ptr!!\n");
		return -EINVAL;
	}
	else
	{        		


	} 

	return err;
}

/*----------------------------------------------------------------------------*/
static int MPU6050C_ReadStart(struct i2c_client *client, bool enable)
{
	u8 databuf[2] = {0};    
	int res = 0;
	GYRO_FUN();    
	  
	databuf[0] = MPU6050C_REG_FIFO_EN;  
	
	if(enable)
	{
		//enable xyz gyro in FIFO
		databuf[1] = (MPU6050C_FIFO_GYROX_EN|MPU6050C_FIFO_GYROY_EN|MPU6050C_FIFO_GYROZ_EN);
	}
	else
	{
		//disable xyz gyro in FIFO
		databuf[1] = 0;
	}
	
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GYRO_ERR(" enable xyz gyro in FIFO error,enable: 0x%x!\n", databuf[1]);
		return MPU6050C_ERR_I2C;
	}
	GYRO_LOG("MPU6050C_ReadStart: enable xyz gyro in FIFO: 0x%x\n", databuf[1]);
	return MPU6050C_SUCCESS;
}

/*----------------------------------------------------------------------------*/
static int MPU6050C_FIFOConfig(struct i2c_client *client, u8 clk)
{
	u8 databuf[2] = {0};    
	int res = 0;
	GYRO_FUN();    

	//use gyro X, Y or Z for clocking
	databuf[0] = MPU6050C_PWR_MGMT_1;
	databuf[1] = clk;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GYRO_ERR("write Power CTRL register err!\n");
		return MPU6050C_ERR_I2C;
	}
	GYRO_LOG("MPU6050C use gyro X for clocking OK!\n");

	mdelay(50);
	
	//enable xyz gyro in FIFO
	databuf[0] = MPU6050C_REG_FIFO_EN;    
	databuf[1] = (MPU6050C_FIFO_GYROX_EN|MPU6050C_FIFO_GYROY_EN|MPU6050C_FIFO_GYROZ_EN);
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GYRO_ERR("write Power CTRL register err!\n");
		return MPU6050C_ERR_I2C;
	}
	GYRO_LOG("MPU6050C enable xyz gyro in FIFO OK!\n");

	//disable AUX_VDDIO
	databuf[0] = MPU6050C_REG_AUX_VDD;    
	databuf[1] = MPU6050C_AUX_VDDIO_DIS;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GYRO_ERR("write AUX_VDD register err!\n");
		return MPU6050C_ERR_I2C;
	}
	GYRO_LOG("MPU6050C disable AUX_VDDIO OK!\n");
	
	//enable FIFO and reset FIFO
	databuf[0] = MPU6050C_REG_FIFO_CTL;    
	databuf[1] = (MPU6050C_FIFO_EN | MPU6050C_FIFO_RST);
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GYRO_ERR("write FIFO CTRL register err!\n");
		return MPU6050C_ERR_I2C;
	}
	
	GYRO_LOG("MPU6050C_FIFOConfig OK!\n");
	return MPU6050C_SUCCESS;    
}

/*----------------------------------------------------------------------------*/
static int MPU6050C_ReadFifoData(struct i2c_client *client, s16 *data, int* datalen)
{
	struct mpu6050c_i2c_data *obj = i2c_get_clientdata(client);        
	u8 buf[MPU6050C_DATA_LEN] = {0};
	s16 tmp1[MPU6050C_AXES_NUM] = {0}; 
	s16 tmp2[MPU6050C_AXES_NUM] = {0}; 
	int err = 0;
	u8 tmp = 0;
	int packet_cnt = 0;
	int i;
	GYRO_FUN();
	
	if(NULL == client)
	{
		return -EINVAL;
	}

	//stop putting data in FIFO
	MPU6050C_ReadStart(client, FALSE);

	//read data number of bytes in FIFO
	err = hwmsen_read_byte(client, MPU6050C_REG_FIFO_CNTH, &tmp);
	if(err)
	{
		GYRO_ERR("read data high number of bytes error: %d\n", err);
		return -1;
	}
	packet_cnt = tmp<< 8;

	err = hwmsen_read_byte(client, MPU6050C_REG_FIFO_CNTL, &tmp);
	if(err)
	{
		GYRO_ERR("read data low number of bytes error: %d\n", err);
		return -1;
	}
	packet_cnt = (packet_cnt + tmp) /MPU6050C_DATA_LEN;
	
	printk("MPU6050C Read Data packet number OK: %d\n", packet_cnt);

	*datalen = packet_cnt;
	
	//Within +-3% range: timing_tolerance * packet_thresh=0.03*75
	if(packet_cnt && (abs(packet_thresh -packet_cnt) < 4))
	{
		//read data in FIFO
		for(i = 0; i < packet_cnt; i++)
		{
			if(hwmsen_read_block(client, MPU6050C_REG_FIFO_DATA, buf, MPU6050C_DATA_LEN))
			{
				GYRO_ERR("MPU6050C read data from FIFO error: %d\n", err);
				return -2;
			}
			else
			{
				GYRO_LOG("MPU6050C read Data of diff address from FIFO OK !\n");
			}
				
			tmp1[MPU6050C_AXIS_X] = (s16)((buf[MPU6050C_AXIS_X*2+1]) | (buf[MPU6050C_AXIS_X*2] << 8));
			tmp1[MPU6050C_AXIS_Y] = (s16)((buf[MPU6050C_AXIS_Y*2+1]) | (buf[MPU6050C_AXIS_Y*2] << 8));
			tmp1[MPU6050C_AXIS_Z] = (s16)((buf[MPU6050C_AXIS_Z*2+1]) | (buf[MPU6050C_AXIS_Z*2] << 8));
				
			//remap coordinate//
			tmp2[obj->cvt.map[MPU6050C_AXIS_X]] = obj->cvt.sign[MPU6050C_AXIS_X]*tmp1[MPU6050C_AXIS_X];
			tmp2[obj->cvt.map[MPU6050C_AXIS_Y]] = obj->cvt.sign[MPU6050C_AXIS_Y]*tmp1[MPU6050C_AXIS_Y];
			tmp2[obj->cvt.map[MPU6050C_AXIS_Z]] = obj->cvt.sign[MPU6050C_AXIS_Z]*tmp1[MPU6050C_AXIS_Z];

			data[3* i +MPU6050C_AXIS_X] = tmp2[MPU6050C_AXIS_X];
			data[3* i +MPU6050C_AXIS_Y] = tmp2[MPU6050C_AXIS_Y];
			data[3* i +MPU6050C_AXIS_Z] = tmp2[MPU6050C_AXIS_Z];
			
			GYRO_LOG("gyro FIFO packet[%d]:[%04X %04X %04X] => [%5d %5d %5d]\n", i, 
			data[3*i +MPU6050C_AXIS_X], data[3*i +MPU6050C_AXIS_Y], data[3*i +MPU6050C_AXIS_Z], 
			data[3*i +MPU6050C_AXIS_X], data[3*i +MPU6050C_AXIS_Y], data[3*i +MPU6050C_AXIS_Z]);
		}

	}
	else
	{
		GYRO_ERR("MPU6050C Incorrect packet count: %d\n", packet_cnt);
		return -3;
	}

	return 0;
}

/*----------------------------------------------------------------------------*/
static int MPU6050C_ReadGsensorRawData(struct i2c_client *client, s16 buf[MPU6050C_AXES_NUM])
{
	char databuf[MPU6050C_DATA_LEN];	
	
	if(sensor_power == false)
	{
		MPU6050C_SetPowerMode(client, true);
		msleep(50);
	}

	if(hwmsen_read_block(client, MPU6050C_REG_GSENSOR_XH, databuf, 6))
	{
		GYRO_ERR("MPU6050C read gsensor data  error\n");
		return -2;
	}
	else
	{
		buf[MPU6050C_AXIS_X] = ((s16)((databuf[MPU6050C_AXIS_X*2+1]) | (databuf[MPU6050C_AXIS_X*2] << 8)));
		buf[MPU6050C_AXIS_Y] = ((s16)((databuf[MPU6050C_AXIS_Y*2+1]) | (databuf[MPU6050C_AXIS_Y*2] << 8)));
		buf[MPU6050C_AXIS_Z] = ((s16)((databuf[MPU6050C_AXIS_Z*2+1]) | (databuf[MPU6050C_AXIS_Z*2] << 8)));
	}

//	printk("Gsensor Raw data :[x = %d, y = %d, z = %d]\n", buf[MPU6050C_AXIS_X], buf[MPU6050C_AXIS_Y], buf[MPU6050C_AXIS_Z]);
	
	return 0;
}


/*----------------------------------------------------------------------------*/
static int MPU6050C_ReadGsensorData(struct i2c_client *client, char *buf, int bufsize)
{
	char databuf[6];	
	int data[3];
	struct mpu6050c_i2c_data *obj = i2c_get_clientdata(client);  
	
	if(sensor_power == false)
	{
		MPU6050C_SetPowerMode(client, true);
		msleep(50);
	}

	if(hwmsen_read_block(client, MPU6050C_REG_GSENSOR_XH, databuf, 6))
	{
		GYRO_ERR("MPU6050C read gsensor data  error\n");
		return -2;
	}
	else
	{
		obj->data[MPU6050C_AXIS_X] = ((s16)((databuf[MPU6050C_AXIS_X*2+1]) | (databuf[MPU6050C_AXIS_X*2] << 8)));
		obj->data[MPU6050C_AXIS_Y] = ((s16)((databuf[MPU6050C_AXIS_Y*2+1]) | (databuf[MPU6050C_AXIS_Y*2] << 8)));
		obj->data[MPU6050C_AXIS_Z] = ((s16)((databuf[MPU6050C_AXIS_Z*2+1]) | (databuf[MPU6050C_AXIS_Z*2] << 8)));
	
		/*remap coordinate*/
		data[obj->cvt.map[MPU6050C_AXIS_X]] = obj->cvt.sign[MPU6050C_AXIS_X]*obj->data[MPU6050C_AXIS_X];
		data[obj->cvt.map[MPU6050C_AXIS_Y]] = obj->cvt.sign[MPU6050C_AXIS_Y]*obj->data[MPU6050C_AXIS_Y];
		data[obj->cvt.map[MPU6050C_AXIS_Z]] = obj->cvt.sign[MPU6050C_AXIS_Z]*obj->data[MPU6050C_AXIS_Z];

		//Out put the x,y,z, the UNITS is m/s2
		data[MPU6050C_AXIS_X] = data[MPU6050C_AXIS_X] * GRAVITY_EARTH_1000 / GSNSOR_SENSITIVITY_SCALE_FACTOR_AFS_SEL_1;
		data[MPU6050C_AXIS_Y] = data[MPU6050C_AXIS_Y] * GRAVITY_EARTH_1000 / GSNSOR_SENSITIVITY_SCALE_FACTOR_AFS_SEL_1;
		data[MPU6050C_AXIS_Z] = data[MPU6050C_AXIS_Z] * GRAVITY_EARTH_1000 / GSNSOR_SENSITIVITY_SCALE_FACTOR_AFS_SEL_1;
	}

	data[MPU6050C_AXIS_X] += obj->cali_gsensor[MPU6050C_AXIS_X];
	data[MPU6050C_AXIS_Y] += obj->cali_gsensor[MPU6050C_AXIS_Y];
	data[MPU6050C_AXIS_Z] += obj->cali_gsensor[MPU6050C_AXIS_Z];

	sprintf(buf, "%04x %04x %04x", data[MPU6050C_AXIS_X],data[MPU6050C_AXIS_Y],data[MPU6050C_AXIS_Z]);

//	printk("Gsensor data :[x = %d, y = %d, z = %d]\n", data[0], data[1], data[2]);

	return 0;
	
}

/*----------------------------------------------------------------------------*/
static int MPU6050C_ReadGyroRawData(struct i2c_client *client, s16 buf[MPU6050C_AXES_NUM])
{
	char databuf[MPU6050C_DATA_LEN];	
	
	if(sensor_power == false)
	{
		MPU6050C_SetPowerMode(client, true);
		msleep(50);
	}

	if(hwmsen_read_block(client, MPU6050C_REG_GYRO_XH, databuf, 6))
	{
		GYRO_ERR("MPU6050C read gsensor data  error\n");
		return -2;
	}
	else
	{
		buf[MPU6050C_AXIS_X] = ((s16)((databuf[MPU6050C_AXIS_X*2+1]) | (databuf[MPU6050C_AXIS_X*2] << 8)));
		buf[MPU6050C_AXIS_Y] = ((s16)((databuf[MPU6050C_AXIS_Y*2+1]) | (databuf[MPU6050C_AXIS_Y*2] << 8)));
		buf[MPU6050C_AXIS_Z] = ((s16)((databuf[MPU6050C_AXIS_Z*2+1]) | (databuf[MPU6050C_AXIS_Z*2] << 8)));
	}

//	printk("Gsensor Raw data :[x = %d, y = %d, z = %d]\n", buf[MPU6050C_AXIS_X], buf[MPU6050C_AXIS_Y], buf[MPU6050C_AXIS_Z]);
	
	return 0;
}


/*----------------------------------------------------------------------------*/
static int MPU6050C_ReadGyroData(struct i2c_client *client, char *buf, int bufsize)
{
	char databuf[6];	
	int data[3];
	struct mpu6050c_i2c_data *obj = i2c_get_clientdata(client);  
	
	if(sensor_power == false)
	{
		MPU6050C_SetPowerMode(client, true);
		msleep(50);
	}

	if(hwmsen_read_block(client, MPU6050C_REG_GYRO_XH, databuf, 6))
	{
		GYRO_ERR("MPU6050C read gyroscope data  error\n");
		return -2;
	}
	else
	{
		obj->data[MPU6050C_AXIS_X] = ((s16)((databuf[MPU6050C_AXIS_X*2+1]) | (databuf[MPU6050C_AXIS_X*2] << 8)));
		obj->data[MPU6050C_AXIS_Y] = ((s16)((databuf[MPU6050C_AXIS_Y*2+1]) | (databuf[MPU6050C_AXIS_Y*2] << 8)));
		obj->data[MPU6050C_AXIS_Z] = ((s16)((databuf[MPU6050C_AXIS_Z*2+1]) | (databuf[MPU6050C_AXIS_Z*2] << 8)));

		/*remap coordinate*/
		data[obj->cvt.map[MPU6050C_AXIS_X]] = obj->cvt.sign[MPU6050C_AXIS_X]*obj->data[MPU6050C_AXIS_X];
		data[obj->cvt.map[MPU6050C_AXIS_Y]] = obj->cvt.sign[MPU6050C_AXIS_Y]*obj->data[MPU6050C_AXIS_Y];
		data[obj->cvt.map[MPU6050C_AXIS_Z]] = obj->cvt.sign[MPU6050C_AXIS_Z]*obj->data[MPU6050C_AXIS_Z];

		//Out put the degree/second(o/s)
		data[MPU6050C_AXIS_X] = data[MPU6050C_AXIS_X] * PRECISION_ADVANCE / MPU6050C_FS_250_LSB;
		data[MPU6050C_AXIS_Y] = data[MPU6050C_AXIS_Y] * PRECISION_ADVANCE / MPU6050C_FS_250_LSB;
		data[MPU6050C_AXIS_Z] = data[MPU6050C_AXIS_Z] * PRECISION_ADVANCE / MPU6050C_FS_250_LSB;
	}
	
	data[MPU6050C_AXIS_X] += obj->cali_gyro[MPU6050C_AXIS_X];
	data[MPU6050C_AXIS_Y] += obj->cali_gyro[MPU6050C_AXIS_Y];
	data[MPU6050C_AXIS_Z] += obj->cali_gyro[MPU6050C_AXIS_Z];
	
	sprintf(buf, "%04x %04x %04x", data[MPU6050C_AXIS_X],data[MPU6050C_AXIS_Y],data[MPU6050C_AXIS_Z]);

//	printk("Gyro data :[x = %6d, y = %6d, z = %6d]\n", data[0], data[1], data[2]);

	return 0;
}

//for factory mode
static int MPU6050C_PROCESS_SMT_DATA(struct i2c_client *client, short *data)
{
    int total_num = 0;
	int retval =0;
    long xSum = 0;
    long ySum = 0;
    long zSum = 0;
    long xAvg, yAvg, zAvg;
    long xRMS, yRMS, zRMS;
	int i=0;

	int bias_thresh = 5242; // 40 dps * 131.072 LSB/dps
    //float RMS_thresh = 687.19f; // (.2 dps * 131.072) ^ 2
    long RMS_thresh = 68719; // (.2 dps * 131.072) ^ 2

	total_num = data[0];
	retval = data[1];
	GYRO_LOG("MPU6050C read gyro data OK, total number: %d \n", total_num); 
	
	for(i = 0; i < total_num; i++)
	{
		xSum =xSum + data[MPU6050C_AXES_NUM*i + MPU6050C_AXIS_X +2]; 
		ySum =ySum + data[MPU6050C_AXES_NUM*i + MPU6050C_AXIS_Y +2]; 
		zSum =zSum + data[MPU6050C_AXES_NUM*i + MPU6050C_AXIS_Z +2]; 
		
		/*
		FLPLOGD("read gyro data OK: packet_num:%d, [X:%5d, Y:%5d, Z:%5d]\n", i, data[MPU6050C_AXES_NUM*i + MPU6050C_AXIS_X +2], 
			data[MPU6050C_AXES_NUM*i + MPU6050C_AXIS_Y +2], data[MPU6050C_AXES_NUM*i + MPU6050C_AXIS_Z +2]);
		FLPLOGD("MPU6050C xSum: %5d,  ySum: %5d, zSum: %5d \n", xSum, ySum, zSum); 
		*/
	}
	GYRO_LOG("MPU6050C xSum: %5ld,  ySum: %5ld, zSum: %5ld \n", xSum, ySum, zSum); 
	
	if (total_num != 0)
	{
		xAvg = (xSum / total_num); 
		yAvg = (ySum / total_num); 
		zAvg = (zSum / total_num);
	}
	else
	{
		xAvg = xSum;
		yAvg = ySum;
		zAvg = zSum;
	}
	
	GYRO_LOG("MPU6050C xAvg: %ld,  yAvg: %ld,  zAvg: %ld \n", xAvg, yAvg, zAvg); 

	if ( abs(xAvg) >bias_thresh)
	{
		GYRO_LOG("X-Gyro bias exceeded threshold \n");
		retval |= 1 << 3;
	}
	if ( abs(yAvg) >  bias_thresh)
	{
		GYRO_LOG("Y-Gyro bias exceeded threshold \n");
		retval |= 1 << 4;
	}
	if ( abs(zAvg ) > bias_thresh)
	{
		GYRO_LOG("Z-Gyro bias exceeded threshold \n");
		retval |= 1 << 5;
	}

	xRMS = 0; 
	yRMS = 0; 
	zRMS = 0;

	//Finally, check RMS
	for ( i = 0; i < total_num ; i++)
	{
		xRMS += (data[MPU6050C_AXES_NUM*i + MPU6050C_AXIS_X+2]-xAvg)*(data[MPU6050C_AXES_NUM*i + MPU6050C_AXIS_X+2]-xAvg);
		yRMS += (data[MPU6050C_AXES_NUM*i + MPU6050C_AXIS_Y+2]-yAvg)*(data[MPU6050C_AXES_NUM*i + MPU6050C_AXIS_Y+2]-yAvg);
		zRMS += (data[MPU6050C_AXES_NUM*i + MPU6050C_AXIS_Z+2]-zAvg)*(data[MPU6050C_AXES_NUM*i + MPU6050C_AXIS_Z+2]-zAvg);        
	}

	GYRO_LOG("MPU6050C xRMS: %ld,  yRMS: %ld,  zRMS: %ld \n", xRMS, yRMS, zRMS); 
	xRMS = 100*xRMS;
	yRMS = 100*yRMS;
	zRMS = 100*zRMS;
	
	if (FACTORY_BOOT == get_boot_mode())
  		return retval;
	if ( xRMS > RMS_thresh * total_num)
	{
		GYRO_LOG("X-Gyro RMS exceeded threshold, RMS_thresh: %ld \n", RMS_thresh * total_num);
		retval |= 1 << 6;
	}
	if ( yRMS > RMS_thresh * total_num )
	{	
		GYRO_LOG("Y-Gyro RMS exceeded threshold, RMS_thresh: %ld \n", RMS_thresh * total_num);
		retval |= 1 << 7;
	}
	if ( zRMS > RMS_thresh * total_num )
	{
		GYRO_LOG("Z-Gyro RMS exceeded threshold, RMS_thresh: %ld \n", RMS_thresh * total_num);
		retval |= 1 << 8;
	}
	if ( xRMS == 0 || yRMS == 0 || zRMS == 0)
		//If any of the RMS noise value returns zero, then we might have dead gyro or FIFO/register failure
		retval |= 1 << 9;

	return retval;
	
}

/*----------------------------------------------------------------------------*/
static int MPU6050C_SMTReadSensorData(struct i2c_client *client, s16 *buf, int bufsize)
{
	//S16 gyro[MPU6050C_AXES_NUM*MPU6050C_FIFOSIZE];
	int res = 0;
	int i;
	int datalen, total_num= 0;

	GYRO_FUN();

	if(sensor_power == false)
	{
		MPU6050C_SetPowerMode(client, true);
	}

	if(NULL == buf)
	{
		return -1;
	}
	if(NULL == client)
	{
		*buf = 0;
		return -2;
	}
	
	for(i = 0; i < MPU6050C_AXES_NUM; i++)
	{
		res = MPU6050C_FIFOConfig(client, (i+1));
		if(res)
		{
			GYRO_ERR("MPU6050C_FIFOConfig error:%d!\n", res);
			return -3;
		}
		
		//putting data in FIFO during the delayed 600ms
		mdelay(600);

		res = MPU6050C_ReadFifoData(client, &(buf[total_num+2]), &datalen);
		if(res)
		{  
			if(res == (-3))
			{
				buf[1] = (1<< i);
			}
			else
			{
				GYRO_ERR("MPU6050C_ReadData error:%d!\n", res);
				return -3;
			}
		}
		else
		{
			buf[0] = datalen;
			total_num+=datalen*MPU6050C_AXES_NUM;
		}
	}

	GYRO_LOG("gyroscope read data OK, total packet: %d", buf[0] ); 
	
	return 0;
}

/*----------------------------------------------------------------------------*/
static int MPU6050C_ReadChipInfo(struct i2c_client *client, char *buf, int bufsize)
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

	sprintf(buf, "MPU6050C Chip");
	return 0;
}


/*----------------------------------------------------------------------------*/
static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = mpu6050c_i2c_client;
	char strbuf[MPU6050C_BUFSIZE];
	if(NULL == client)
	{
		GYRO_ERR("i2c client is null!!\n");
		return 0;
	}
	
	MPU6050C_ReadChipInfo(client, strbuf, MPU6050C_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);        
}

/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = mpu6050c_i2c_client;
	char strbuf[MPU6050C_BUFSIZE];
	
	if(NULL == client)
	{
		GYRO_ERR("i2c client is null!!\n");
		return 0;
	}
	
	MPU6050C_ReadGyroData(client, strbuf, MPU6050C_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);;            
}

/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct mpu6050c_i2c_data *obj = mpu6050c_data;
	if (obj == NULL)
	{
		GYRO_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	
	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));     
	return res;    
}

/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct mpu6050c_i2c_data *obj = mpu6050c_data;
	int trace = 0;
	if (obj == NULL)
	{
		GYRO_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "%d", &trace))
	{
		atomic_set(&obj->trace, trace);
	}	
	else
	{
		GYRO_ERR("invalid content: '%s', length = %d\n", buf, count);
	}
	
	return count;    
}

/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;    
	struct mpu6050c_i2c_data *obj = mpu6050c_data;
	if (obj == NULL)
	{
		GYRO_ERR("i2c_data obj is null!!\n");
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
static ssize_t show_interrupt_true_value(struct device_driver * ddri,char * buf)
{
	ssize_t res;
	struct mpu6050c_i2c_data *obj = mpu6050c_data;
	if (obj == NULL)
	{
		GYRO_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	printk("Gyro %s\n", __func__);

	res = snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&obj->interrupt_true));     
	return res;    
}

/*----------------------------------------------------------------------------*/
static ssize_t store_interrupt_true_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct mpu6050c_i2c_data *obj = mpu6050c_data;
	int interrupt_true = 0;

	printk("Gyro %s\n", __func__);
	
	if (obj == NULL)
	{
		GYRO_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "%d", &interrupt_true))
	{
		atomic_set(&obj->interrupt_true, interrupt_true);
	}	
	else
	{
		GYRO_ERR("invalid content: '%s', length = %d\n", buf, count);
	}
	
	return count;    
}

/*----------------------------------------------------------------------------*/
static ssize_t show_interrupt_enable_value(struct device_driver * ddri,char * buf)
{
	ssize_t res;
	struct mpu6050c_i2c_data *obj = mpu6050c_data;
	if (obj == NULL)
	{
		GYRO_ERR("i2c_data obj is null!!\n");
		return 0;
	}

	printk("Gyro %s\n", __func__);

	res = snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&obj->interrupt_enable));     
	return res;    
}

/*----------------------------------------------------------------------------*/
static ssize_t store_interrupt_enable_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct mpu6050c_i2c_data *obj = mpu6050c_data;
	int interrupt_enable = 0;

	printk("Gyro %s\n", __func__);
	
	if (obj == NULL)
	{
		GYRO_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "%d", &interrupt_enable))
	{
		atomic_set(&obj->interrupt_enable, interrupt_enable);
	}	
	else
	{
		GYRO_ERR("invalid content: '%s', length = %d\n", buf, count);
	}
	
	return count;    
}

/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(chipinfo,             S_IRUGO, show_chipinfo_value,      NULL);
static DRIVER_ATTR(sensordata,           S_IRUGO, show_sensordata_value,    NULL);
static DRIVER_ATTR(trace,		S_IWUSR | S_IRUGO, show_trace_value,		store_trace_value);
static DRIVER_ATTR(status,               S_IRUGO, show_status_value,        NULL);
static DRIVER_ATTR(interrupt_true,	S_IRUGO | S_IWUSR,	show_interrupt_true_value,	store_interrupt_true_value);
static DRIVER_ATTR(interrupt_enable,	S_IRUGO | S_IWUSR , show_interrupt_enable_value,	store_interrupt_enable_value);

/*----------------------------------------------------------------------------*/
static struct driver_attribute *MPU6050C_attr_list[] = {
	&driver_attr_chipinfo,     /*chip information*/
	&driver_attr_sensordata,   /*dump sensor data*/	
	&driver_attr_trace,        /*trace log*/
	&driver_attr_status,  
	&driver_attr_interrupt_enable, 
	&driver_attr_interrupt_true, 
};

/*----------------------------------------------------------------------------*/
static int mpu6050c_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(MPU6050C_attr_list)/sizeof(MPU6050C_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if(0 != (err = driver_create_file(driver, MPU6050C_attr_list[idx])))
		{            
			GYRO_ERR("driver_create_file (%s) = %d\n", MPU6050C_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}

/*----------------------------------------------------------------------------*/
static int mpu6050c_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(MPU6050C_attr_list)/sizeof(MPU6050C_attr_list[0]));

	if(driver == NULL)
	{
		return -EINVAL;
	}
	
	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, MPU6050C_attr_list[idx]);
	}
	
	return err;
}

/*----------------------------------------------------------------------------*/
static int MPU6050C_Device_Reset(struct i2c_client *client)	//reset all the internal registers to default values
{
	u8 databuf[2] = {0};    
	int res = 0;

	databuf[1] = 0x80;
	databuf[0] = MPU6050C_PWR_MGMT_1;    
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		printk("MPU6050c Device Reset failed!\n");
		return MPU6050C_ERR_I2C;
	}	

	return MPU6050C_SUCCESS;
}

/*----------------------------------------------------------------------------*/
static int MPU6050C_SetPowerMode(struct i2c_client *client, bool enable)
{
	u8 databuf[2] = {0};    
	int res = 0;

	if(enable == sensor_power)
	{
		printk("Sensor power status is newest but set again!\n");
		
#ifndef OPPO_R819		// Delete by zhangqiang

		return MPU6050C_SUCCESS;

#endif 

	}

	if(enable == FALSE)
	{
		databuf[0] |= MPU6050C_SLEEP;
	}
	else
	{
		databuf[0] &= ~MPU6050C_SLEEP;
	}

	printk("%s, databuf[0] = 0x%x\n", __func__, databuf[0]);
	
	databuf[1] = databuf[0];
	databuf[0] = MPU6050C_PWR_MGMT_1;    
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GYRO_LOG("set power mode failed!\n");
		return MPU6050C_ERR_I2C;
	}	
	else
	{
		printk("Gyro or G-sensor set power mode to %d!\n", enable);
	}

	sensor_power = enable;
	
	return MPU6050C_SUCCESS;    
}

/*----------------------------------------------------------------------------*/
static int MPU6050C_SetDataFormat(struct i2c_client *client, u8 dataformat)
{
	u8 databuf[2] = {0};    
	int res = 0;
	GYRO_FUN();     

	printk("%s, dataformat = 0x%x\n", __func__, dataformat);

	databuf[0] = MPU6050C_REG_DATA_FMT;    
	databuf[1] = dataformat;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		return MPU6050C_ERR_I2C;
	}

	databuf[0] = MPU6050C_GYRO_CONFIG;    // Set MPU5050c Gyro Full Scale Range
	databuf[1] = 0;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		return MPU6050C_ERR_I2C;
	}
	else
	{
		printk("Set Gyro Full Scale Range +-250(o/s)\n");
	}
	
	return MPU6050C_SUCCESS;    
}

// set the sample rate
static int MPU6050C_SetSampleRate(struct i2c_client *client, int sample_rate)
{
	u8 databuf[2] = {0}; 
	int rate_div = 0;
	int res = 0;
	GYRO_FUN();    

	if(hwmsen_read_byte(client, MPU6050C_REG_DATA_FMT, databuf))
	{
		GYRO_ERR("read gyro data format register err!\n");
		return MPU6050C_ERR_I2C;
	}
	else
	{
		printk("Read  gyro data format register: 0x%x\n", databuf[0]);
	}

	if((databuf[0] & 0x07) == 0)	//Analog sample rate is 8KHz
	{
		rate_div = 8 * 1024 / sample_rate - 1;
	}
	else	// Analog sample rate is 1kHz( execute!)
	{
		rate_div = 1024 / sample_rate - 1;
	}

	if(rate_div > 255)	// rate_div: 0 to 255;
	{
		rate_div = 255;
	}
	else if(rate_div < 0)
	{
		rate_div = 0;
	}

	printk("Rate_div is : 0x%x\n", rate_div);
	
	databuf[0] = MPU6050C_REG_SAMRT_DIV;    
	databuf[1] = rate_div;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GYRO_ERR("write sample rate register err!\n");
		return MPU6050C_ERR_I2C;
	}

	//read sample div after written for test	
	udelay(500);
	if(hwmsen_read_byte(client, MPU6050C_REG_SAMRT_DIV, databuf))
	{
		GYRO_ERR("read gyro sample rate register err!\n");
		return MPU6050C_ERR_I2C;
	}
	else
	{
		printk("Read  gyro sample rate is %d Hz!\n", sample_rate);
	}
	
	return MPU6050C_SUCCESS;    
}

#ifdef MOTION_SENSITIVE_GSENSOR

/*----------------------------------------------------------------------------*/
static void gyro_irq_handler(struct work_struct *work)
{
	mt65xx_eint_mask(CUST_EINT_GYRO_NUM);
	
	queue_work(gyro_wq, &mpu6050c_data->work);	
}

static void gyro_irq_handle_fun(struct work_struct *work)
{
	printk("Gyro interrupt , interrupt_enable  = %d\n", mpu6050c_data->interrupt_enable);

	printk("XXXXXXXXXXXXXX AAAAAAAAAABBBBBBBBCCCCCCCC XXXXXXXXXXXXXX\n");

	if(atomic_read(&mpu6050c_data->suspend) && atomic_read(&mpu6050c_data->interrupt_enable))
	{
		printk("Simulate power key pressed !\n");

		atomic_set(&mpu6050c_data->interrupt_true, 1);		
		
		input_report_key(mpu6050c_data->input_dev, KEY_POWER, 1);
		input_sync(mpu6050c_data->input_dev);	
		input_report_key(mpu6050c_data->input_dev, KEY_POWER, 0);
		input_sync(mpu6050c_data->input_dev);
	}
	
 	mt65xx_eint_unmask(CUST_EINT_GYRO_NUM);	
}

static void MPU6050C_Interrupt_enable(struct i2c_client *client, int mode)
{
	u8 databuf[2] = {0}; 
	int res = 0;

	if(mode == false)
	{
		databuf[0] = MPU6050C_INT_ENABLE;	 
		databuf[1] = 0;	// disable the interrupt when not suspend 
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			GYRO_ERR("Write MPU6050C_INT_ENABLE register err!\n");
			return MPU6050C_ERR_I2C;
		}
	}
	else
	{
		databuf[0] = MPU6050C_INT_ENABLE;	 
		databuf[1] = 0x40;	// enable the interrupt when suspend 
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			GYRO_ERR("Write MPU6050C_INT_ENABLE register err!\n");
			return MPU6050C_ERR_I2C;
		}

	}
}

static void MPU6050C_MotionDetect(struct i2c_client *client)
{
	struct mpu6050c_i2c_data *obj = i2c_get_clientdata(client);
	
	u8 databuf[2] = {0}; 
	int res = 0;

	// Motion interrupt enable only when suspend
	MPU6050C_Interrupt_enable(client, false);

	databuf[0] = MPU6050C_INT_PIN_CFG;    
//	databuf[1] = 0x30;	// interrupt active high, push-pull, int latch until clear, clear if any register read
	databuf[1] = 0x00;	// interrupt active high, push-pull, int emits a 50us long pulse
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GYRO_ERR("Write MPU6050C_INT_PIN_CFG register err!\n");
		return MPU6050C_ERR_I2C;
	}

	databuf[0] = MPU6050C_MOT_DETECT_CTRL;    
	databuf[1] = 0;	// counter decrement : rezero
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GYRO_ERR("Write MPU6050C_MOT_DETECT_CTRL register err!\n");
		return MPU6050C_ERR_I2C;
	}

	databuf[0] = MPU6050C_MOT_THR;    
	databuf[1] = 2;	// motion threshold = n * 2mg
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GYRO_ERR("Write MPU6050C_MOT_THR register err!\n");
		return MPU6050C_ERR_I2C;
	}

	databuf[0] = MPU6050C_MOT_DUR;	 
	databuf[1] = 100;	// motion duration = n * 1ms
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GYRO_ERR("Write MPU6050C_MOT_DUR register err!\n");
		return MPU6050C_ERR_I2C;
	}
}

/*----------------------------------------------------------------------------*/
static int mpu6050c_gpio_config(void)
{

    mt_set_gpio_mode(GPIO_GYRO_EINT_PIN, GPIO_MODE_01);
    mt_set_gpio_dir(GPIO_GYRO_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_GYRO_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_GYRO_EINT_PIN, GPIO_PULL_UP);

	mt65xx_eint_set_polarity(CUST_EINT_GYRO_NUM, CUST_EINT_GYRO_POLARITY);
    #ifndef OPPO_R819 //xiaohua.tian@wxkf.wxBasicDriver.Sensor, 2013/03/12, modify interrupt mode
	mt65xx_eint_set_sens(CUST_EINT_GYRO_NUM, CUST_EINT_GYRO_SENSITIVE);
    #else /*OPPO_R819*/
	mt65xx_eint_set_sens(CUST_EINT_GYRO_NUM, CUST_EINT_EDGE_SENSITIVE);
    #endif /*OPPO_R819*/
	mt65xx_eint_set_hw_debounce(CUST_EINT_GYRO_NUM, CUST_EINT_GYRO_DEBOUNCE_CN);
	mt65xx_eint_registration(CUST_EINT_GYRO_NUM, CUST_EINT_GYRO_DEBOUNCE_EN, 
							CUST_EINT_GYRO_POLARITY, gyro_irq_handler, 0);
	
	mt65xx_eint_unmask(CUST_EINT_GYRO_NUM);	

	enable_irq_wake(CUST_EINT_GYRO_NUM);

	printk("MPU INT gpio config over. \n");
	
	return 0;
}

#endif

/*----------------------------------------------------------------------------*/
static int mpu6050c_init_client(struct i2c_client *client, bool enable)
{
	struct mpu6050c_i2c_data *obj = i2c_get_clientdata(client);
	u8 databuf[2] = {0};    
	int res = 0;
	GYRO_FUN();	

	MPU6050C_Device_Reset(client);

	msleep(50);
	
	res = MPU6050C_SetPowerMode(client, enable);
	if(res != MPU6050C_SUCCESS)
	{
		return res;
	}
	
	res = MPU6050C_SetDataFormat(client, (MPU6050C_SYNC_GYROX << MPU6050C_EXT_SYNC)|
										(MPU6050C_FS_1000 << MPU6050C_FS_RANGE)|
										MPU6050C_RATE_1K_LPFB_188HZ);
	if(res != MPU6050C_SUCCESS) 
	{
		return res;
	}

	// Set 200HZ sample rate
	res = MPU6050C_SetSampleRate(client, 200);
	if(res != MPU6050C_SUCCESS ) 
	{
		return res;
	}

	databuf[0] = MPU6050C_ACCEL_CONFIG;		// Set MPU5050c G-sensor Full Scale Range
	databuf[1] = 0x09;						// ACCEL : +-4G, HPF = 0x01
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		GYRO_ERR("Write MPU6050C_ACCEL_CONFIG register err!\n");
		return MPU6050C_ERR_I2C;
	}

	MPU6050C_active_all_axis();

	printk("mpu6050c_init_client OK!\n");

#ifdef CONFIG_MPU6050C_LOWPASS
	memset(&obj->fir, 0x00, sizeof(obj->fir));  
#endif

	return MPU6050C_SUCCESS;
}

/*----------------------------------------------------------------------------*/
static void MPU6050C_gyro_standby()
{
	u8 databuf[2] = {0}; 
	int res = 0;

	databuf[0] = MPU6050C_PWR_MGMT_2;    
	databuf[1] = 0x07;	
	res = i2c_master_send(mpu6050c_data->client, databuf, 0x2);
	if(res <= 0)
	{
		GYRO_ERR("Write MPU6050C_PWR_MGMT_2 register err!\n");
		return MPU6050C_ERR_I2C;
	}	
}

static void MPU6050C_gsensor_standby()
{
	u8 databuf[2] = {0}; 
	int res = 0;

	databuf[0] = MPU6050C_PWR_MGMT_2;    
	databuf[1] = 0x38;	
	res = i2c_master_send(mpu6050c_data->client, databuf, 0x2);
	if(res <= 0)
	{
		GYRO_ERR("Write MPU6050C_PWR_MGMT_2 register err!\n");
		return MPU6050C_ERR_I2C;
	}	
}

static void MPU6050C_gsensor_z_axis_active()
{
	u8 databuf[2] = {0}; 
	int res = 0;

	databuf[0] = MPU6050C_PWR_MGMT_2;    
	databuf[1] = 0x37;	
	res = i2c_master_send(mpu6050c_data->client, databuf, 0x2);
	if(res <= 0)
	{
		GYRO_ERR("Write MPU6050C_PWR_MGMT_2 register err!\n");
		return MPU6050C_ERR_I2C;
	}	
}

static void MPU6050C_active_all_axis()
{
	u8 databuf[2] = {0}; 
	int res = 0;

	databuf[0] = MPU6050C_PWR_MGMT_2;    
	databuf[1] = 0x0;	
	res = i2c_master_send(mpu6050c_data->client, databuf, 0x2);
	if(res <= 0)
	{
		GYRO_ERR("Write MPU6050C_PWR_MGMT_2 register err!\n");
		return MPU6050C_ERR_I2C;
	}	
}

static int calculate_gsensor(struct i2c_client *client, s32 data[3])
{
	int ret = 0, i = 0;
	s32 read_buff[3] = {0};
	s32 average_offset[3] = {0};
	char buff[256];

	// when calibrat gsensor, need original data
	MPU6050C_Gsensor_ResetCalibration(client);
	
	printk("\n\nWrite Gsensor calibration Data!\n\n");

	for (i = 0; i < 10; i ++)
	{
		if(ret = MPU6050C_ReadGsensorData(client, buff, MPU6050C_BUFSIZE))
		{
			printk("I2C error: ret value=%d", ret);
			return -EIO;
		}
		
		sscanf(buff, "%x %x %x", &read_buff[0],&read_buff[1],&read_buff[2]);
		
		printk("Gsensor Data:(%d %d %d)\n",read_buff[0],read_buff[1],read_buff[2]);
		
		average_offset[MPU6050C_AXIS_X] += read_buff[0];
		average_offset[MPU6050C_AXIS_Y] += read_buff[1];
		average_offset[MPU6050C_AXIS_Z] += read_buff[2];	
		
		msleep(40);
	}

	average_offset[MPU6050C_AXIS_X] /= 10;
	average_offset[MPU6050C_AXIS_Y] /= 10;
	average_offset[MPU6050C_AXIS_Z] /= 10;

	printk("Gsensor average offset Data : average_offset[X]= %d, average_offset[Y]= %d, average_offset[Z]= %d\n",
		average_offset[MPU6050C_AXIS_X],average_offset[MPU6050C_AXIS_Y],average_offset[MPU6050C_AXIS_Z]);

	data[MPU6050C_AXIS_X] = 0-average_offset[MPU6050C_AXIS_X];
	data[MPU6050C_AXIS_Y] = 0-average_offset[MPU6050C_AXIS_Y];
	data[MPU6050C_AXIS_Z] = 9807-average_offset[MPU6050C_AXIS_Z];

	mpu6050c_data->cali_gsensor[MPU6050C_AXIS_X] = data[MPU6050C_AXIS_X];
	mpu6050c_data->cali_gsensor[MPU6050C_AXIS_Y] = data[MPU6050C_AXIS_Y];
	mpu6050c_data->cali_gsensor[MPU6050C_AXIS_Z] = data[MPU6050C_AXIS_Z];

	printk("Gsensor calibrate datas : cali_gsensor[X]= %4d, cali_gsensor[Y]= %4d, cali_gsensor[Z]= %4d\n",
		mpu6050c_data->cali_gsensor[MPU6050C_AXIS_X],mpu6050c_data->cali_gsensor[MPU6050C_AXIS_Y],mpu6050c_data->cali_gsensor[MPU6050C_AXIS_Z]);

	return 0;
}


/*----------------------------------------------------------------------------*/
int mpu6050c_operate_gsensor(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;	
	struct mpu6050c_i2c_data *priv = (struct mpu6050c_i2c_data*)self;
	hwm_sensor_data* gsensor_data;
	char buff[MPU6050C_BUFSIZE];	

	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				GYRO_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			else
			{
			
			}
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				GYRO_ERR("Enable gsensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				if(((value == 0) && (sensor_power == false)) ||((value == 1) && (sensor_power == true)))
				{
					printk("G sensor device have updated!\n");
				}
				else
				{
					printk("%s, G_SENSOR_ENABLE = %d\n", __func__, !sensor_power);	
					err = MPU6050C_SetPowerMode(priv->client, !sensor_power);

#ifdef MOTION_SENSITIVE_GSENSOR
					
					MPU6050C_SetPowerMode(priv->client, true);

#endif
				}
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				GYRO_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				gsensor_data = (hwm_sensor_data *)buff_out;
				MPU6050C_ReadGsensorData(priv->client, buff, MPU6050C_BUFSIZE);
				sscanf(buff, "%x %x %x", &gsensor_data->values[0], 
									&gsensor_data->values[1], &gsensor_data->values[2]);				
				gsensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;				
				gsensor_data->value_divide = GSENSOR_GRAVITY_DIVIDE;
			}
			break;
		default:
			GYRO_ERR("gyroscope operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}

	return err;
}

/*----------------------------------------------------------------------------*/
int mpu6050c_operate_gyro(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;	
	struct mpu6050c_i2c_data *priv = (struct mpu6050c_i2c_data*)self;
	hwm_sensor_data* gyro_data;
	char buff[MPU6050C_BUFSIZE];	

	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				GYRO_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			else
			{
			
			}
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				GYRO_ERR("Enable gyroscope parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				if(((value == 0) && (sensor_power == false)) ||((value == 1) && (sensor_power == true)))
				{
					printk("Gyroscope device have updated!\n");
				}
				else
				{
					printk("%s, Gyro_SENSOR_ENABLE = %d\n", __func__, !sensor_power);				
					err = MPU6050C_SetPowerMode(priv->client, !sensor_power);

#ifdef MOTION_SENSITIVE_GSENSOR
					MPU6050C_SetPowerMode(priv->client, true);
#endif			

				}
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				GYRO_ERR("get gyroscope data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				gyro_data = (hwm_sensor_data *)buff_out;
				MPU6050C_ReadGyroData(priv->client, buff, MPU6050C_BUFSIZE);
				sscanf(buff, "%x %x %x", &gyro_data->values[0], 
									&gyro_data->values[1], &gyro_data->values[2]);				
				gyro_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;				
				gyro_data->value_divide = DEGREE_TO_RAD;
			}
			break;
		default:
			GYRO_ERR("gyroscope operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}

	return err;
}

/****************************************************************************** 
 * G-sensor Function Configuration
******************************************************************************/
static int mpu6050c_gsensor_open(struct inode *inode, struct file *file)
{
	file->private_data = mpu6050c_i2c_client;

	if(file->private_data == NULL)
	{
		GYRO_ERR("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}

/*----------------------------------------------------------------------------*/
static int mpu6050c_gsensor_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static long mpu6050c_gsensor_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	char strbuf[MPU6050C_BUFSIZE] = {0};
	void __user *data;
	long err = 0;
	int copy_cnt = 0;
	SENSOR_DATA sensor_data;
	int cali[3];
	int smtRes=0;

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
		GYRO_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch(cmd)
	{
		case GSENSOR_IOCTL_INIT:
			printk("GSENSOR_IOCTL_INIT 0x01\n");
			
			mpu6050c_init_client(client, false);			
			break;

		case GSENSOR_IOCTL_READ_CHIPINFO:
			printk("GSENSOR_IOCTL_READ_CHIPINFO 0x02\n");
			
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			
			MPU6050C_ReadChipInfo(client, strbuf, MPU6050C_BUFSIZE);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;
			}

			break;

		case GSENSOR_IOCTL_READ_SENSORDATA:
//			printk("GSENSOR_IOCTL_READ_SENSORDATA 0x03\n");
			
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			MPU6050C_ReadGsensorData(client, strbuf, MPU6050C_BUFSIZE);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;	  
			}	
		 
			break;

		case GSENSOR_IOCTL_READ_OFFSET:
			printk("GSENSOR_IOCTL_READ_OFFSET 0x04\n");

			break;

		case GSENSOR_IOCTL_READ_GAIN:
			printk("GSENSOR_IOCTL_READ_GAIN 0x05\n");

			break;

		case GSENSOR_IOCTL_READ_RAW_DATA:
			printk("GSENSOR_IOCTL_READ_RAW_DATA 0x06\n");
			
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			MPU6050C_ReadGsensorRawData(client, &strbuf);
			if(copy_to_user(data, &strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;	  
			}
	
			break;

		case GSENSOR_IOCTL_SET_CALI:
			printk("GSENSOR_IOCTL_SET_CALI 0x06\n");
			
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
			
			cali[MPU6050C_AXIS_X] = sensor_data.x ;
			cali[MPU6050C_AXIS_Y] = sensor_data.y ;
			cali[MPU6050C_AXIS_Z] = sensor_data.z ;	

			MPU6050C_Write_Gsensor_Calibration_data(client, cali);
			
			break;
			
		case GSENSOR_IOCTL_GET_CALI:
			printk("GSENSOR_IOCTL_GET_CALI 0x07\n");

			data = (void __user*)arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			
			break;

		case GSENSOR_IOCTL_CLR_CALI:
			printk("GSENSOR_IOCTL_CLR_CALI 0x08\n");
			
			err = MPU6050C_Gsensor_ResetCalibration(client);
			
			break;	

#ifdef OPPO_R819	// add for user calibrate gsensor in engineering mode
		case GSENSOR_IOCTL_USER_CALI:
			{
				int buf[3];
                s32 cali_data[3];
				data = (void __user *) arg;
				
                err = calculate_gsensor(client, &cali_data);
				
				buf[0] = (int)(cali_data[0]);
				buf[1] = (int)(cali_data[1]);
				buf[2] = (int)(cali_data[2]);
				
				copy_to_user(data, &buf, 3 * sizeof(int));
			}
		break;
#endif
	
		default:
			GYRO_ERR("unknown IOCTL: 0x%08x\n", cmd);
			err = -ENOIOCTLCMD;
			break;			
	}
	return err;
}


/*----------------------------------------------------------------------------*/
static struct file_operations mpu6050c_gsensor_fops = {
//	.owner = THIS_MODULE,//modified
	.open = mpu6050c_gsensor_open,
	.release = mpu6050c_gsensor_release,
	.unlocked_ioctl = mpu6050c_gsensor_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice mpu6050c_gsensor_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gsensor",
	.fops = &mpu6050c_gsensor_fops,
};

/****************************************************************************** 
 * Gyro Function Configuration
******************************************************************************/
static int mpu6050c_gyro_open(struct inode *inode, struct file *file)
{
	file->private_data = mpu6050c_i2c_client;

	if(file->private_data == NULL)
	{
		GYRO_ERR("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}

/*----------------------------------------------------------------------------*/
static int mpu6050c_gyro_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static long mpu6050c_gyro_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	char strbuf[MPU6050C_BUFSIZE] = {0};
	s16 *SMTdata;
	void __user *data;
	long err = 0;
	int copy_cnt = 0;
	SENSOR_DATA sensor_data;
	int cali[3];
	int smtRes=0;

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
		GYRO_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch(cmd)
	{
		case GYROSCOPE_IOCTL_INIT:
			mpu6050c_init_client(client, false);			
			break;

		case GYROSCOPE_IOCTL_SMT_DATA:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}

			SMTdata = kzalloc(sizeof(*SMTdata) * 800, GFP_KERNEL);
			if(SMTdata == NULL)
			{
				err = -ENOMEM;
				break;
			}
			memset(SMTdata, 0, sizeof(*SMTdata) * 800);
			
			MPU6050C_SMTReadSensorData(client, SMTdata, 800);
			//GYRO_LOG("gyroscope read data from kernel OK: sizeof:%d, strlen:%d, packet:%d!\n", sizeof(SMTdata), strlen(SMTdata), SMTdata[0]);
				
			GYRO_LOG("gyroscope read data from kernel OK: SMTdata[0]:%d, copied packet:%d!\n", SMTdata[0], ((SMTdata[0]*MPU6050C_AXES_NUM+2)*sizeof(s16)+1));
			
			smtRes = MPU6050C_PROCESS_SMT_DATA(client,SMTdata);
			
			copy_cnt = copy_to_user(data, &smtRes,  sizeof(smtRes));
			
			kfree(SMTdata);
			if(copy_cnt)
			{
				err = -EFAULT;
				GYRO_ERR("copy gyro data to user failed!\n");
			}	
			GYRO_LOG("copy gyro data to user OK: %d!\n", copy_cnt);
			break;

		case GYROSCOPE_IOCTL_READ_SENSORDATA:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			
			MPU6050C_ReadGyroData(client, strbuf, MPU6050C_BUFSIZE);
			if(copy_to_user(data, strbuf, sizeof(strbuf)))
			{
				err = -EFAULT;
				break;	  
			}				 
			break;

		case GYROSCOPE_IOCTL_SET_CALI:
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
			else
			{
				err = MPU6050C_WriteCalibration(client, cali);
			}
			break;

		case GYROSCOPE_IOCTL_CLR_CALI:
			err = MPU6050C_Gyro_ResetCalibration(client);
			break;

		case GYROSCOPE_IOCTL_GET_CALI:
			data = (void __user*)arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			err = MPU6050C_ReadCalibration(client, cali);
			if(err)
			{
				break;
			}
			
			if(copy_to_user(data, &sensor_data, sizeof(sensor_data)))
			{
				err = -EFAULT;
				break;
			}		
			break;

		default:
			GYRO_ERR("unknown IOCTL: 0x%08x\n", cmd);
			err = -ENOIOCTLCMD;
			break;			
	}
	return err;
}

/*----------------------------------------------------------------------------*/
static struct file_operations mpu6050c_gyro_fops = {
//	.owner = THIS_MODULE,//modified
	.open = mpu6050c_gyro_open,
	.release = mpu6050c_gyro_release,
	.unlocked_ioctl = mpu6050c_gyro_unlocked_ioctl,
};

/*----------------------------------------------------------------------------*/
static struct miscdevice mpu6050c_gyro_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gyroscope",
	.fops = &mpu6050c_gyro_fops,
};

#ifdef MOTION_SENSITIVE_GSENSOR

/*----------------------------------------------------------------------------*/
static int gyro_gsensor_calibration_write(struct file *file,const char *buffer, unsigned long count,void *data)
{
	struct mpu6050c_i2c_data *mpu = mpu6050c_data;
	static char proc_data[32];
	int ret = 0, i = 0;

	s32 read_buff[3] = {0};
	s32 average_offset[3] = {0};
	char buff[256];
	
	if (copy_from_user( &proc_data, buffer, count ))
	{
		printk(KERN_INFO"Gyro_Gsensor_handle data error.\n");
		return -EFAULT;
	}

	if(proc_data[0] == '0')
	{
		printk("Clear Gsensor calibration Data!\n");

		MPU6050C_Gsensor_ResetCalibration(mpu->client);
	}
	else if(proc_data[0] == '1')
	{
		// when calibrat gsensor, need original data
		MPU6050C_Gsensor_ResetCalibration(mpu->client);
		
		printk("Write Gsensor calibration Data!\n");

		for (i = 0; i < 10; i ++)
		{
			if(ret = MPU6050C_ReadGsensorData(mpu->client, buff, MPU6050C_BUFSIZE))
			{
				printk("I2C error: ret value=%d", ret);
				return -EIO;
			}
			
			sscanf(buff, "%x %x %x", &read_buff[0],&read_buff[1],&read_buff[2]);
			
			printk("Gsensor Data:(%d %d %d)\n",read_buff[0],read_buff[1],read_buff[2]);
			
			average_offset[MPU6050C_AXIS_X] += read_buff[0];
			average_offset[MPU6050C_AXIS_Y] += read_buff[1];
			average_offset[MPU6050C_AXIS_Z] += read_buff[2];	
			
			msleep(40);
		}

		average_offset[MPU6050C_AXIS_X] /= 10;
		average_offset[MPU6050C_AXIS_Y] /= 10;
		average_offset[MPU6050C_AXIS_Z] /= 10;

		printk("Gsensor average offset Data : average_offset[X]= %d, average_offset[Y]= %d, average_offset[Z]= %d\n",
			average_offset[MPU6050C_AXIS_X],average_offset[MPU6050C_AXIS_Y],average_offset[MPU6050C_AXIS_Z]);

		mpu->cali_gsensor[MPU6050C_AXIS_X] = 0-average_offset[MPU6050C_AXIS_X];
		mpu->cali_gsensor[MPU6050C_AXIS_Y] = 0-average_offset[MPU6050C_AXIS_Y] ;
		mpu->cali_gsensor[MPU6050C_AXIS_Z] = 9807-average_offset[MPU6050C_AXIS_Z] ;

		printk("Gsensor calibrate datas : cali_gsensor[X]= %4d, cali_gsensor[Y]= %4d, cali_gsensor[Z]= %4d\n",
			mpu->cali_gsensor[MPU6050C_AXIS_X],mpu->cali_gsensor[MPU6050C_AXIS_Y],mpu->cali_gsensor[MPU6050C_AXIS_Z]);
	}
	else if(proc_data[0] == 'a')
	{
		printk("Clear Gyro calibration Data!\n");

		MPU6050C_Gyro_ResetCalibration(mpu->client);			
	}
	else if(proc_data[0] == 'b')
	{
		// when calibrat Gyro, need original data
		MPU6050C_Gyro_ResetCalibration(mpu->client);
		
		printk("Write Gyro calibration Data!\n");

		for (i = 0; i < 10; i ++)
		{
			if(ret = MPU6050C_ReadGyroData(mpu->client, buff, MPU6050C_BUFSIZE))
			{
				printk("I2C error: ret value=%d", ret);
				return -EIO;
			}
			
			sscanf(buff, "%x %x %x", &read_buff[0],&read_buff[1],&read_buff[2]);
			
			printk("Gyro Data:(%d %d %d)\n",read_buff[0],read_buff[1],read_buff[2]);
			
			average_offset[MPU6050C_AXIS_X] += read_buff[0];
			average_offset[MPU6050C_AXIS_Y] += read_buff[1];
			average_offset[MPU6050C_AXIS_Z] += read_buff[2];	
			
			msleep(40);
		}

		average_offset[MPU6050C_AXIS_X] /= 10;
		average_offset[MPU6050C_AXIS_Y] /= 10;
		average_offset[MPU6050C_AXIS_Z] /= 10;

		printk("Gyro average offset Data : average_offset[X]= %d, average_offset[Y]= %d, average_offset[Z]= %d\n",
			average_offset[MPU6050C_AXIS_X],average_offset[MPU6050C_AXIS_Y],average_offset[MPU6050C_AXIS_Z]);

		mpu->cali_gyro[MPU6050C_AXIS_X] = 0-average_offset[MPU6050C_AXIS_X];
		mpu->cali_gyro[MPU6050C_AXIS_Y] = 0-average_offset[MPU6050C_AXIS_Y];
		mpu->cali_gyro[MPU6050C_AXIS_Z] = 0-average_offset[MPU6050C_AXIS_Z];

		printk("Gyro calibrate datas : cali_gsensor[X]= %4d, cali_gsensor[Y]= %4d, cali_gsensor[Z]= %4d\n",
			mpu->cali_gyro[MPU6050C_AXIS_X],mpu->cali_gyro[MPU6050C_AXIS_Y],mpu->cali_gyro[MPU6050C_AXIS_Z]);

	}

	return count;
}


static ssize_t gesture_enable_read(struct file *file, char __user *buf, size_t count, loff_t *pos)
{
    char page[1024]; 	
    char *p = page;	
    int len = 0; 

	struct mpu6050c_i2c_data *obj = mpu6050c_data;
	if (obj == NULL)
	{
		GYRO_ERR("i2c_data obj is null!!\n");
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
	
	struct mpu6050c_i2c_data *obj = mpu6050c_data;
	if (obj == NULL)
	{
		GYRO_ERR("i2c_data obj is null!!\n");
		return 0;
	}

    ret = copy_from_user(tmp, buf, 16);	

	if(1 == sscanf(tmp, "%d", &interrupt_enable))
	{
		atomic_set(&obj->interrupt_enable, interrupt_enable);
	}	
	else
	{
		GYRO_ERR("invalid content: '%s', length = %d\n", tmp, count);
	}

    return count;	
}

static ssize_t gesture_true_read(struct file *file, char __user *buf, size_t count, loff_t *pos)
{
    char page[1024]; 	
    char *p = page;	
    int len = 0; 

	struct mpu6050c_i2c_data *obj = mpu6050c_data;
	if (obj == NULL)
	{
		GYRO_ERR("i2c_data obj is null!!\n");
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
	
	struct mpu6050c_i2c_data *obj = mpu6050c_data;
	if (obj == NULL)
	{
		GYRO_ERR("i2c_data obj is null!!\n");
		return 0;
	}

    ret = copy_from_user(tmp, buf, 16);	

	if(1 == sscanf(tmp, "%d", &interrupt_true))
	{
		atomic_set(&obj->interrupt_true, interrupt_true);
	}	
	else
	{
		GYRO_ERR("invalid content: '%s', length = %d\n", tmp, count);
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
	mpu6050c_data->entry = proc_mkdir_mode("gyro_gsensor", 0777, NULL);
	// The mode would be better 777, or the proc file may not be opened.

	proc_entry_calibration = create_proc_entry( "gyro_gsensor_calibration", 0666, mpu6050c_data->entry);
	if(proc_entry_calibration == NULL)
	{
	  	printk(KERN_INFO"Couldn't create proc entry\n");
	}
	else
	{
		proc_entry_calibration->write_proc = gyro_gsensor_calibration_write;
	}

	proc_create("gesture_enable", 0666, mpu6050c_data->entry, &gesture_enable_fops);
	proc_create("gesture_true", 0666, mpu6050c_data->entry, &gesture_true_fops);
}

#endif	/* define MOTION_SENSITIVE_GSENSOR*/

/*----------------------------------------------------------------------------*/
#ifndef CONFIG_HAS_EARLYSUSPEND
/*----------------------------------------------------------------------------*/
static int mpu6050c_suspend(struct i2c_client *client, pm_message_t msg) 
{
	struct mpu6050c_i2c_data *obj = i2c_get_clientdata(client);    
	int err; 
	GYRO_FUN();   

	if(msg.event == PM_EVENT_SUSPEND)
	{   
		if(obj == NULL)
		{
			GYRO_ERR("null pointer!!\n");
			return -EINVAL;
		}
		atomic_set(&obj->suspend, 1);		
		
		err = MPU6050C_SetPowerMode(client, false);
		if(err <= 0)
		{
			return err;
		}
	}
	return 0;//modified
}
/*----------------------------------------------------------------------------*/
static int mpu6050c_resume(struct i2c_client *client)
{
	struct mpu6050c_i2c_data *obj = i2c_get_clientdata(client);        
	int err;
	GYRO_FUN();

	if(obj == NULL)
	{
		GYRO_ERR("null pointer!!\n");
		return -EINVAL;
	}

	MPU6050C_power(obj->hw, 1);
	err = mpu6050c_init_client(client, false);
	if(err)
	{
		GYRO_ERR("initialize client fail!!\n");
		return err;        
	}
	atomic_set(&obj->suspend, 0);

	return 0;
}
#else /*CONFIG_HAS_EARLY_SUSPEND is defined*/
/*----------------------------------------------------------------------------*/
static void mpu6050c_early_suspend(struct early_suspend *h) 
{
	struct mpu6050c_i2c_data *obj = container_of(h, struct mpu6050c_i2c_data, early_drv);   
	int err;
	u8 databuf[2];
	GYRO_FUN();    

	printk("%s\n", __func__);

	if(obj == NULL)
	{
		GYRO_ERR("null pointer!!\n");
		return;
	}

	atomic_set(&obj->suspend, 1);

#ifdef MOTION_SENSITIVE_GSENSOR

	// only enable motion enable when suspend 
	MPU6050C_Interrupt_enable(obj->client, true);

	MPU6050C_SetPowerMode(obj->client, true);
	
/*
	// only enable z axis to generate interrupt signal
	MPU6050C_gsensor_z_axis_active();
*/
	// enable all axis to generate interrupt signal
	MPU6050C_gyro_standby();
	
	return ;

#endif

	err = MPU6050C_SetPowerMode(obj->client, false);
	if(err)
	{
		GYRO_ERR("write power control fail!!\n");
		return;
	}

	sensor_power = false;
	
	MPU6050C_power(obj->hw, 0);
}

/*----------------------------------------------------------------------------*/
static void mpu6050c_late_resume(struct early_suspend *h)
{
	struct mpu6050c_i2c_data *obj = container_of(h, struct mpu6050c_i2c_data, early_drv);         
	int err;
	GYRO_FUN();

	printk("%s\n", __func__);

	if(obj == NULL)
	{
		GYRO_ERR("null pointer!!\n");
		return;
	}

#ifdef MOTION_SENSITIVE_GSENSOR

	// disable interrupt when resume
	MPU6050C_Interrupt_enable(obj->client, false);

	MPU6050C_active_all_axis();

	return ;
	
#endif
	
	MPU6050C_power(obj->hw, 1);

	err = mpu6050c_init_client(obj->client, true);
	if(err)
	{
		GYRO_ERR("initialize client fail! err code %d!\n", err);
		return;        
	}
	atomic_set(&obj->suspend, 0);    
}
/*----------------------------------------------------------------------------*/
#endif /*CONFIG_HAS_EARLYSUSPEND*/

/*----------------------------------------------------------------------------*/
static int mpu6050c_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_client *new_client;
	struct mpu6050c_i2c_data *obj;
	struct hwmsen_object sobj;
	
	int err = 0;
	GYRO_FUN();

	printk("%s start! \n", __func__);

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}

	memset(obj, 0, sizeof(struct mpu6050c_i2c_data));

	printk("MPU6050C addr is 0x%x!\n", client->addr);

	obj->hw = get_cust_gyro_hw_mpu6050c();
	err = hwmsen_get_convert(obj->hw->direction, &obj->cvt);
	if(err)
	{
		GYRO_ERR("invalid direction: %d\n", obj->hw->direction);
		goto exit;
	}

	MPU6050C_power(obj->hw, 1);

	mpu6050c_data = obj;
	obj->client = client;
	new_client = obj->client;
	i2c_set_clientdata(new_client,obj);
	
	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);
	atomic_set(&obj->interrupt_enable, 0);
	atomic_set(&obj->interrupt_true, 0);
	
	mpu6050c_i2c_client = new_client;	
	err = mpu6050c_init_client(new_client, true);
	if(err)
	{
		printk("mpu6050c_init_client failed, not mpu6050c or bad chip!\n");
		goto exit_init_failed;
	}

	printk("gyro_default_i2c_addr: 0x%x\n", client->addr);

	err = misc_register(&mpu6050c_gsensor_device);
	if(err)
	{
		GYRO_ERR("mpu6050c_gsensor_device misc register failed!\n");
		goto exit_misc_device_register_failed;
	}
	
	err = misc_register(&mpu6050c_gyro_device);
	if(err)
	{
		GYRO_ERR("mpu6050c_gyro_device misc register failed!\n");
		goto exit_misc_device_register_failed;
	}

	err = mpu6050c_create_attr(&mpu6050c_gyro_driver.driver);
	if(err)
	{
		GYRO_ERR("mpu6050c create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}

#ifdef MOTION_SENSITIVE_GSENSOR
		
	obj->input_dev = input_allocate_device();
	obj->input_dev->name = "sensor-key";
	set_bit(EV_KEY, obj->input_dev->evbit);
	set_bit(KEY_POWER, obj->input_dev->keybit);
	set_bit(EV_SYN, obj->input_dev->evbit);
	
	if(input_register_device(obj->input_dev))
	{
		printk("Gyro or G-sensor failed to register input device\n");
	}

	mpu6050c_gpio_config();

	INIT_WORK(&obj->work, gyro_irq_handle_fun);

	MPU6050C_MotionDetect(new_client);

	proc_file_create();
		
#endif

	sobj.self = obj;
    sobj.polling = 1;
    sobj.sensor_operate = mpu6050c_operate_gsensor;
	err = hwmsen_attach(ID_ACCELEROMETER, &sobj);
	if(err)
	{
		GYRO_ERR("hwmsen_attach fail = %d\n", err);
		goto exit_kfree;
	}

	sobj.self = obj;
    sobj.polling = 1;
    sobj.sensor_operate = mpu6050c_operate_gyro;
	err = hwmsen_attach(ID_GYROSCOPE, &sobj);
	if(err)
	{
		GYRO_ERR("hwmsen_attach fail = %d\n", err);
		goto exit_kfree;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 2,
	obj->early_drv.suspend  = mpu6050c_early_suspend,
	obj->early_drv.resume   = mpu6050c_late_resume,    
	register_early_suspend(&obj->early_drv);
#endif 

	printk("%s: OK\n", __func__);    

	gyro_dev = GYRO_MPU6050C;

	return 0;

	exit_create_attr_failed:
	misc_deregister(&mpu6050c_gsensor_device);
	misc_deregister(&mpu6050c_gyro_device);
	exit_misc_device_register_failed:
	exit_init_failed:
	printk("MPU6050C probe failed , power down!\n");		
	MPU6050C_power(obj->hw, 0);
	exit_kfree:
	kfree(obj);
	exit:
	GYRO_ERR("%s: err = %d\n", __func__, err);        
	return err;
}

/*----------------------------------------------------------------------------*/
static int mpu6050c_i2c_remove(struct i2c_client *client)
{
	int err = 0;	
	
	err = mpu6050c_delete_attr(&mpu6050c_gyro_driver.driver);
	if(err)
	{
		GYRO_ERR("mpu6050c_delete_attr fail: %d\n", err);
	}
	
	err = misc_deregister(&mpu6050c_gsensor_device);
	if(err)
	{
		GYRO_ERR("misc_deregister fail: %d\n", err);
	}

	err = misc_deregister(&mpu6050c_gyro_device);
	if(err)
	{
		GYRO_ERR("misc_deregister fail: %d\n", err);
	}

	err = hwmsen_detach(ID_ACCELEROMETER);
	if(err)
	{
		GYRO_ERR("hwmsen_detach fail: %d\n", err);
	}

	mpu6050c_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	return 0;
}

/*----------------------------------------------------------------------------*/
static int mpu6050c_probe(struct platform_device *pdev) 
{
	GYRO_FUN();
	printk("%s\n", __func__);
	
    gyro_wq = create_singlethread_workqueue("gyro_wq");

	if(i2c_add_driver(&mpu6050c_i2c_driver))
	{
		GYRO_ERR("add driver error\n");
		return -1;
	}
	return 0;
}

/*----------------------------------------------------------------------------*/
static int mpu6050c_remove(struct platform_device *pdev)
{
    struct gyro_hw *hw = get_cust_gyro_hw_mpu6050c();

    GYRO_FUN();    
	printk("%s\n", __func__);

    MPU6050C_power(hw, 0);    
    i2c_del_driver(&mpu6050c_i2c_driver);
    return 0;
}

/*----------------------------------------------------------------------------*/
static struct platform_driver mpu6050c_gyro_driver = 
{
	.probe      = mpu6050c_probe,
	.remove     = mpu6050c_remove,    
	.driver     = 
	{
		.name  = "gyroscope_mpu6050c",
//		.owner = THIS_MODULE,//modified
	}
};

/*----------------------------------------------------------------------------*/
static int __init mpu6050c_init(void)
{
	struct gyro_hw *hw = get_cust_gyro_hw_mpu6050c();
	GYRO_FUN();	
	printk("%s: i2c_number=%d\n", __func__,hw->i2c_num);
	
	i2c_register_board_info(hw->i2c_num, &i2c_mpu6050c, 1);	
	if(platform_driver_register(&mpu6050c_gyro_driver))
	{
		GYRO_ERR("failed to register driver");
		return -ENODEV;
	}
	return 0;    
}

/*----------------------------------------------------------------------------*/
static void __exit mpu6050c_exit(void)
{
	GYRO_FUN();
	platform_driver_unregister(&mpu6050c_gyro_driver);
}

/*----------------------------------------------------------------------------*/
module_init(mpu6050c_init);
module_exit(mpu6050c_exit);

/*----------------------------------------------------------------------------*/
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MPU6050C gyroscope driver");
MODULE_AUTHOR("Chunlei.Wang@mediatek.com");
