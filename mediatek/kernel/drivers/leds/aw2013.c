#ifdef MOZ_AW2013_SUPPORT
#include <cust_eint.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>

#include <asm/irq.h>
#include <asm/mach/irq.h>
#include <linux/slab.h>
#include <linux/earlysuspend.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include "cust_gpio_usage.h"
#include <linux/miscdevice.h>

//for dym create device
#include <linux/cdev.h>
#include <linux/device.h>
#include <asm/device.h>
#include <asm/uaccess.h>

//kthread
#include <linux/fcntl.h>
#include <linux/syscalls.h>
#include <linux/platform_device.h>

#include <linux/slab.h>
#include <linux/semaphore.h>
#include <linux/completion.h>
#include <linux/spinlock.h>
#include "aw2013.h"

#define MIN(A,B)	((A)<(B)?(A):(B))
#define MAX(A,B)	((A)>(B)?(A):(B))

#define AW2013_NAME     	"aw2013"
#define AW2013_I2C_ADDR  	(0x8a)

#define RISE_TIME   (2)
#define HOLD_TIME   (3)
#define FALL_TIME 	(2)
#define OFF_TIME    (3)
#define DELAY_TIME  (0)
#define REPEAT_TIMES (0)

#define BRIGHTNESS_LEVEL (0xff)

//========================================================================================
// ========================AW2013 寄存器定义======================================
//------------------------------常用寄存器定义 ---------------------------------------   
//    0x01：LED使能寄存器，正常工作为0x01，关断为0x00
//    0x02：中断使能寄存器，可不用设置
//    0x30：LED使能寄存器，低三位分别对应LED0 LED1 LED2，设为1使能，为0关断

//    0x31 0x32 0x33：依次对应LED0 LED1 LED2，可配置LED最大电流0,5,10,15mA，淡进淡出效果
//    0x34 0x35 0x36：依次对应LED0 LED1 LED2，可配LED最大电流，淡进淡出效果

//    0x37 LED0 上升时间，常亮时间(前6级可选)设定，8级：0.13 0.26 0.52 1.04 2.08 4.16 8.32 16.64s
//    0x38 LED0 下降时间设定，常灭时间设定8级可选：0.13 0.26 0.52 1.04 2.08 4.16 8.32 16.64s
//    0x39 LED0 呼吸延迟时间设定，9级选择0s,0.13s,0.26s,0.52s,1.04s,2.08s,4.16s,8.32s,16.64s；呼吸次数设定，可无穷次，可1到15次

//    0x3A LED1 上升时间，常亮时间(前6级可选)设定，8级：0.13 0.26 0.52 1.04 2.08 4.16 8.32 16.64s
//    0x3B LED1 下降时间设定，常灭时间设定8级可选：0.13 0.26 0.52 1.04 2.08 4.16 8.32 16.64s
//    0x3C LED1 呼吸延迟时间设定，9级选择0s,0.13s,0.26s,0.52s,1.04s,2.08s,4.16s,8.32s,16.64s；呼吸次数设定，可无穷次，可1到15次

//    0x3D LED2 上升时间，常亮时间(前6级可选)设定，8级：0.13 0.26 0.52 1.04 2.08 4.16 8.32 16.64s
//    0x3E LED2 下降时间设定，常灭时间设定8级可选：0.13 0.26 0.52 1.04 2.08 4.16 8.32 16.64s
//    0x3F LED2 呼吸延迟时间设定，9级选择0s,0.13s,0.26s,0.52s,1.04s,2.08s,4.16s,8.32s,16.64s；呼吸次数设定，可无穷次，可1到15次
//========================================================================================


static struct i2c_client *g_i2c_client = NULL;
static struct platform_driver aw2013_leds_driver;
static struct i2c_driver aw2013_i2c_driver;

static int aw2013_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int aw2013_i2c_remove(struct i2c_client *client);

static const struct i2c_device_id leds_id[] = {{AW2013_NAME,0},{}};
static struct i2c_board_info __initdata i2c_leds={ I2C_BOARD_INFO(AW2013_NAME, (AW2013_I2C_ADDR>>1))};

#if defined(CONFIG_HAS_EARLYSUSPEND)
static struct early_suspend   early_drv;
#endif

static int early_suspend_mode = 0 ;
static int led_power 	  	  = 1 ;

//DECLARE_MUTEX(semaphore_led_mutex); // linux kernel forsake the api.
static struct semaphore semaphore_led_mutex;
static spinlock_t led_lock;
static int aw2013_is_present = 1;
//DECLARE_COMPLETION(op_read_completion);
//DECLARE_COMPLETION(op_erase_completion);
//DECLARE_COMPLETION(op_write_completion);
//static struct completion op_read_completion;
//static struct completion op_erase_completion;
//static struct completion op_write_completion;

#define AW2013_MASK_LED_RED         0x01
#define AW2013_MASK_LED_GREEN    0x02
#define AW2013_MASK_LED_BLUE       0x04
#define AW2013_MASK_LED_WHITE    0x80


char keyhome_standby_breathled_enabled = 0;

/******************************************************************
** apis declear {
******************************************************************/
static int aw2013_i2c_master_send(struct i2c_client *client, u8 addr, char *buf ,int count);
static int aw2013_i2c_master_recv(struct i2c_client *client, u8 addr, char *buf ,int count);

static int aw2013_config_power(u8 power_on);
static int aw2013_config_enable(u8 led_index,u8 enable);
static int aw2013_config_enable_all(u8 rgb);
static int aw2013_config_breath_mode(u8 led_index,u8 is_breath_mode);
static int aw2013_config_current_max(u8 led_index,u8 current_level);
static int aw2013_config_fade(u8 led_index,u8 fade_off_enable,u8 fade_on_enable);
static int aw2013_config_flash_mode_brightness(u8 led_index,u8 brightness);
static int aw2013_config_time(u8 led_index,u8 rise_time,u8 hold_time,u8 fall_time,u8 off_time,u8 delay_time,u8 repeat_time);

static int led_power_on(u8 type,u8 on);
static int led_process_effects();
static int aw2013_make_effect(u8 type);

static void led_test();
/******************************************************************
** apis declear }
******************************************************************/

#define DATA_LEN (32)
#define I2C_TWO_MSG_ONCE
//#define I2C_SEND_AND_RECV_TWO_MSG
//#define I2C_SEND_WRITE_THEN_READ_ONE_MSG
/****************************************************************************************************
** discript:read i2c
** @client :[I]: client 
** @addr   :[I]: command 
** @buf    :[O]: recv data buffer pointer
** @count  :[I]: data size for read
*****************************************************************************************************/
static int aw2013_i2c_master_recv(struct i2c_client *client, u8 addr, char *buf ,int count)
{
	 int ret = 0, retry = 0;
	 struct i2c_adapter *adap = client->adapter;
	 struct i2c_msg msg[2]; // write then read 

	 int max_try = 3;

	 char data[DATA_LEN] = { 0 } ;
	 char data_buff[DATA_LEN] = { 0 } ;
	 int  data_len = 0 ;

	 if( aw2013_is_present == 0 ) return 0;

	//return i2c_master_recv(client,buf,count);
#if defined(I2C_TWO_MSG_ONCE)
	 // success
	 memset(data,0,sizeof(data));
	 data[0] 		  = addr ;
	 data_len = 1;

	 // send addr
	 //msg.addr = 0x8a >> 1;
	 msg[0].addr = (AW2013_I2C_ADDR) >> 1;
	 msg[0].flags = 0;
	 msg[0].timing = 200;
	 msg[0].len = data_len;
	 msg[0].buf = (char *)data;
	 msg[0].ext_flag = client->ext_flag;

	 // read addr data
	 msg[1].addr = (AW2013_I2C_ADDR) >> 1;
	 msg[1].flags = I2C_M_RD;
	 msg[1].timing = 200;
	 msg[1].len = data_len;
	 msg[1].buf = (char *)&data_buff[0];
	 msg[1].ext_flag = client->ext_flag;

	 retry = 0;
	 while(retry++ < max_try)
	 {
			ret = i2c_transfer(adap, msg, 2);
			//printk("<1> shaokai debug i2c_transfer 2 msgs ret recv = %d",ret);
			if (ret == 2)
					break;
			udelay(100);
	 }
	 memcpy(buf,&data_buff[0],data_len); // move data

	 //printk("<1> recv data = %x %x %x \n",data_buff[0],data_buff[1],data_buff[2]);
 
	 /* If everything went ok (i.e. 1 msg transmitted), return #bytes
	 transmitted, else error code. */
	 return (ret == 2) ? count : ret;
#elif defined(I2C_SEND_AND_RECV_TWO_MSG)
	 // send addr 
	 memset(data,0,sizeof(data));
	 data[0] 		  = addr ;
	 data_len = 1;

	 //msg.addr = 0x8a >> 1;
	 msg[0].addr = (AW2013_I2C_ADDR) >> 1;
	 msg[0].flags = 0;
	 msg[0].timing = 200;
	 msg[0].len = data_len;
	 msg[0].buf = (char *)data;
	 msg[0].ext_flag = client->ext_flag;

	 retry = 0;
	 while(retry++ < max_try)
	 {
			ret = i2c_transfer(adap, &msg[0], 1);
			//printk("<1> shaokai debug i2c_transfer ret recv 1= %d",ret);
			if (ret == 1)
					break;
			udelay(100);
	 }
	 
	 //udelay(100);
	 msleep(10);

	 //recv data
	 memset(data_buff,0,sizeof(data_buff));
	 data_len = (count > (DATA_LEN)) ? (DATA_LEN) : count ;

	 //msg.addr = 0x8a >> 1;
	 msg[1].addr = (AW2013_I2C_ADDR) >> 1;
	 msg[1].flags = I2C_M_RD;
	 msg[1].timing = 200;
	 msg[1].len = data_len;
	 msg[1].buf = (char *)&data_buff[0];
	 msg[1].ext_flag = client->ext_flag;
	 
	 retry = 0;
	 while(retry++ < max_try)
	 {
		 ret = i2c_transfer(adap, &msg[1], 1);
		 //printk("<1> shaokai debug i2c_transfer ret recv 2= %d\n",ret);
		 if (ret == 1)
				 break;
		 udelay(100);
	 }
	 memcpy(buf,&data_buff[0],data_len); // move data

	 //printk("<1> recv data = %x %x %x \n",data_buff[0],data_buff[1],data_buff[2]);
 
	 /* If everything went ok (i.e. 1 msg transmitted), return #bytes
	 transmitted, else error code. */
	 return (ret == 1) ? count : ret;
#elif defined(I2C_SEND_WRITE_THEN_READ_ONE_MSG)
	 // write then read 
	 memset(data_buff,0,sizeof(data_buff));
	 data_buff[0]	  = addr ;
	 data_len = 1 ;//count > (DATA_LEN-1-1) ? (DATA_LEN-1-1) : count ;

	 //msg.addr = 0x8a >> 1;
	 msg[0].addr = (((AW2013_I2C_ADDR) >> 1) & I2C_MASK_FLAG) | I2C_WR_FLAG | I2C_RS_FLAG;
	 msg[0].flags = 0;
	 msg[0].timing = 200;
	 msg[0].len = (1<<8) | 1 ; // ((data_len << 8) | data_len)
	 msg[0].buf = (char *)data_buff;
	 msg[0].ext_flag = client->ext_flag;

	 retry = 0;
	 while(retry++ < max_try)
	 {
			ret = i2c_transfer(adap, &msg[0], 1);
			//printk("<1> shaokai debug i2c_transfer recv ret = %d\n",ret);
			if (ret == 1)
					break;
			udelay(100);
	 }
	 memcpy(buf,data_buff,data_len); // move data .

	 //printk("<1> recv data = %x %x %x \n",data_buff[0],data_buff[1],data_buff[2]);
 
	 /* If everything went ok (i.e. 1 msg transmitted), return #bytes
	 transmitted, else error code. */
	 return (ret == 1) ? count : ret;
#else
	return i2c_smbus_read_i2c_block_data(client,addr,count,buf);
#endif

}

/****************************************************************************************************
** discript:write i2c
** @client :[I]: client 
** @addr   :[I]: command 
** @buf    :[I]: send data buffer pointer
** @count  :[I]: data size for write
*****************************************************************************************************/
static int aw2013_i2c_master_send(struct i2c_client *client, u8 addr, char *buf ,int count)
{
#if 1
		//single addr and multi byte data.

		int ret = 0, retry = 0;
		struct i2c_adapter *adap = client->adapter;
		struct i2c_msg msg;

		char data_buff[DATA_LEN] = { 0 } ;
		int  data_len = 0 ;

		int max_try = 3;

		if( aw2013_is_present == 0 ) return 0;

		data_len = count > (DATA_LEN-1) ? (DATA_LEN-1) : count ;

		memset(data_buff,0,sizeof(data_buff));
		data_buff[0]	  = addr ;
		memcpy(&data_buff[1],buf,data_len);


		//msg.addr = 0x8a >> 1;
	 	msg.addr = (AW2013_I2C_ADDR) >> 1;
		msg.flags = 0;
		msg.timing = 200;
		msg.len = data_len+1;
		msg.buf = (char *)data_buff;
		msg.ext_flag = client->ext_flag;

		while(retry++ < max_try)
		{
				ret = i2c_transfer(adap, &msg, 1);
				//printk("<1> shaokai debug i2c_transfer send ret = %d",ret);
				if (ret == 1)
						break;
				udelay(100);
		}

		/* If everything went ok (i.e. 1 msg transmitted), return #bytes
		transmitted, else error code. */
		return (ret == 1) ? count : ret;
#else		
		//return i2c_master_send(client,data_buff,(data_len+1));
		// multi addr and multi data
		return i2c_smbus_write_i2c_block_data(client,addr,count,buf);
#endif
}


/*********************************************************************************
** led control {
*********************************************************************************/
// order by priority 
typedef enum __LED_TYPE__	
{
	TYPE_MIN = 0,
	TYPE_CHARGE = TYPE_MIN,
	TYPE_CALL,
	TYPE_MSG,
	TYPE_NOTIFY,
	TYPE_STANDBY,
	TYPE_TOP= 9,
	TYPE_MAX 
}LED_TYPE;

typedef struct __LED_CFG__
{
	u8   enable; /* on-off */
	u8   breath; /* long time on or breath */
	u8   r;
	u8   r_ratio; /* current_level */
	u8   g;
	u8   g_ratio;
	u8   b;
	u8   b_ratio;
	
	u8   rise_time;
	u8   hold_time;
	u8   fall_time;
	u8   off_time;
	u8   delay_time;
	u8   period_num; /* breath times */
}LED_CFG;

static LED_CFG led_state[TYPE_MAX] = { 0 };

/*********************************************************************************
** led control }
*********************************************************************************/

static u8 aw2013_is_poweron = 0;  // init state : power off
/*********************************************************************************
** led function {
*********************************************************************************/
static int aw2013_config_power(u8 power_on)
{
	// ic power: 0x01
	//  R7 R6    /    R5      /   R4  R3   / R2   R1  R0
	//  X       /    HZ       /  X         /  current level
	int ret = -1;
	u8 data_value = 0x00;

	if(aw2013_is_poweron == power_on)
	{
		//already power on/off
		return -1;
	}

#if 1
	// first reset 
	if(power_on > 0)
	{
		if(aw2013_is_poweron == 0)
		{
			data_value = 0x55;
			aw2013_i2c_master_send(g_i2c_client, AW2013_RESET_REG, &data_value,1);
		}
	}
#endif

	if(power_on > 0)
	{
			data_value = 1;
			aw2013_is_poweron = 1 ;
	}
	else
	{
			data_value = 0;
			aw2013_is_poweron = 0 ;
	}

	//printk("<1> aw2013_config_power() write %d\n",data_value);
	aw2013_i2c_master_send(g_i2c_client, 0x01, &data_value,1);

	// test read back data.
	//aw2013_i2c_master_recv(g_i2c_client, 0x01, &data_value,1);
	//printk("<1> aw2013_config_power() read %d\n",data_value);

	return 0 ;
}

static int aw2013_config_enable(u8 led_index ,u8 enable)
{
	// led enable: 0x30
	//  R7 R6    /    R5      /   R4  R3   / R2   R1  R0
	//  X       /    HZ       /  X         /  leds enable

	int ret = -1;
	u8 data_value = 0x00;

	// led r or g or b enable
	switch(led_index)
	{
		case 0:
			ret=aw2013_i2c_master_recv(g_i2c_client, AW2013_LED_CHANNEL_ENABLE_REG, &data_value,1);
			if(enable > 0)
			{
				data_value |= (0x01);
			}
			else
			{
				data_value &= ~(0x01);
			}
			if(ret >= 0)
			{
				aw2013_i2c_master_send(g_i2c_client, AW2013_LED_CHANNEL_ENABLE_REG, &data_value,1);
			}
			break;
		case 1:
			ret=aw2013_i2c_master_recv(g_i2c_client, AW2013_LED_CHANNEL_ENABLE_REG, &data_value,1);
			if(enable > 0)
			{
				data_value |= (0x01<<1);
			}
			else
			{
				data_value &= ~(0x01<<1);
			}
			if(ret >= 0)
			{
				aw2013_i2c_master_send(g_i2c_client, AW2013_LED_CHANNEL_ENABLE_REG, &data_value,1);
			}
			break;
		case 2:
			ret=aw2013_i2c_master_recv(g_i2c_client, AW2013_LED_CHANNEL_ENABLE_REG, &data_value,1);
			if(enable > 0)
			{
				data_value |= (0x01<<2);
			}
			else
			{
				data_value &= ~(0x01<<2);
			}
			if(ret >= 0)
			{
				aw2013_i2c_master_send(g_i2c_client, AW2013_LED_CHANNEL_ENABLE_REG, &data_value,1);
			}
			break;
		default:
			break;
	}

	//data_value = (led_sets & 0x07);
	//aw2013_i2c_master_send(g_i2c_client, 0x30, &data_value,1);
}

static int aw2013_config_enable_all(u8 rgb)
{
	// led enable: 0x30
	//  R7 R6    /    R5      /   R4  R3   / R2   R1  R0
	//  X       /    HZ       /  X         /  leds enable

	int ret = -1;
	u8 data_value = 0x00;

	data_value = ( rgb & 0x07 );
	//data_value = 0x07;//open all led
	//data_value = 0x04;//b
	//data_value = 0x02;//g
	//data_value = 0x01;//r
	aw2013_i2c_master_send(g_i2c_client, 0x30, &data_value,1);

}

static int aw2013_config_breath_mode(u8 led_index,u8 is_breath_mode)
{
	// mode and current: 0x31 ~ 0x33
	//  R7      /  	R6       /    R5      /   R4        /  R3  R2 /  R1  R0
	//  X       / fade off  /  fade on   /   breath    /  XX     /  current level

	int ret = -1;
	u8 data_value = 0x00;

	switch (led_index)
	{
		case 0:
			// led0
			ret=aw2013_i2c_master_recv(g_i2c_client, AW2013_LED0_CFG_REG, &data_value,1);
			if(is_breath_mode == 1)
			{
				data_value |= (1 << 4);
			}
			else
			{
				data_value &= ~(1 << 4);
			}
			if(ret >= 0)
			{
				aw2013_i2c_master_send(g_i2c_client, AW2013_LED0_CFG_REG, &data_value,1);
			}
			break;
		case 1:
			// led1
			ret = aw2013_i2c_master_recv(g_i2c_client, AW2013_LED1_CFG_REG, &data_value,1);
			if(is_breath_mode == 1)
			{
				data_value |= (1 << 4);
			}
			else
			{
				data_value &= ~(1 << 4);
			}
			if(ret >= 0)
			{
					aw2013_i2c_master_send(g_i2c_client, AW2013_LED1_CFG_REG, &data_value,1);
			}
			break;
		case 2:
			// led2
			ret = aw2013_i2c_master_recv(g_i2c_client, AW2013_LED2_CFG_REG, &data_value,1);
			if(is_breath_mode == 1)
			{
				data_value |= (1 << 4);
			}
			else
			{
				data_value &= ~(1 << 4);
			}
			if(ret >= 0)
			{
					aw2013_i2c_master_send(g_i2c_client, AW2013_LED2_CFG_REG, &data_value,1);
			}
			break;
		default:
			break;
	}
}

static int aw2013_config_current_max(u8 led_index,u8 current_level)
{
	// mode and current: 0x31 ~ 0x33
	//  R7      /  	R6       /    R5      /   R4        /  R3  R2 /  R1  R0
	//  X       / fade off  /  fade on   /   breath    /  XX     /  current level

	int ret = -1;
	u8 data_value = 0x00;

	switch (led_index)
	{
		case 0:
			// led0
			ret=aw2013_i2c_master_recv(g_i2c_client, AW2013_LED0_CFG_REG, &data_value,1);
			data_value &= ~0x03;
			data_value |= (current_level & 0x03);
			if(ret >= 0)
			{
				aw2013_i2c_master_send(g_i2c_client, AW2013_LED0_CFG_REG, &data_value,1);
			}
			break;
		case 1:
			// led1
			ret = aw2013_i2c_master_recv(g_i2c_client, AW2013_LED1_CFG_REG, &data_value,1);
			data_value &= ~0x03;
			data_value |= (current_level & 0x03);
			if(ret >= 0)
			{
					aw2013_i2c_master_send(g_i2c_client, AW2013_LED1_CFG_REG, &data_value,1);
			}
			break;
		case 2:
			// led2
			ret = aw2013_i2c_master_recv(g_i2c_client, AW2013_LED2_CFG_REG, &data_value,1);
			data_value &= ~0x03;
			data_value |= (current_level & 0x03);
			if(ret >= 0)
			{
					aw2013_i2c_master_send(g_i2c_client, AW2013_LED2_CFG_REG, &data_value,1);
			}
			break;
		default:
			break;
	}
}

static int aw2013_config_fade(u8 led_index,u8 fade_off_enable,u8 fade_on_enable)
{
	// mode and current: 0x31 ~ 0x33
	//  R7      /  	R6       /    R5      /   R4        /  R3  R2 /  R1  R0
	//  X       / fade off  /  fade on   /   breath    /  XX     /  current level

	int ret = -1;
	u8 data_value = 0x00;

	switch (led_index)
	{
		case 0:
			// led0
			ret=aw2013_i2c_master_recv(g_i2c_client, AW2013_LED0_CFG_REG, &data_value,1);
			if(fade_off_enable)
			{
				data_value |= (1<<6);
			}
			else
			{
				data_value &= ~(1<<6);
			}
			if(fade_on_enable)
			{
				data_value |= (1<<5);
			}
			else
			{
				data_value &= ~(1<<5);
			}
			if(ret >= 0)
			{
				aw2013_i2c_master_send(g_i2c_client, AW2013_LED0_CFG_REG, &data_value,1);
			}
			break;
		case 1:
			// led1
			ret = aw2013_i2c_master_recv(g_i2c_client, AW2013_LED1_CFG_REG, &data_value,1);
			if(fade_off_enable)
			{
				data_value |= (1<<6);
			}
			else
			{
				data_value &= ~(1<<6);
			}
			if(fade_on_enable)
			{
				data_value |= (1<<5);
			}
			else
			{
				data_value &= ~(1<<5);
			}
			if(ret >= 0)
			{
					aw2013_i2c_master_send(g_i2c_client, AW2013_LED1_CFG_REG, &data_value,1);
			}
			break;
		case 2:
			// led2
			ret = aw2013_i2c_master_recv(g_i2c_client, AW2013_LED2_CFG_REG, &data_value,1);
			if(fade_off_enable)
			{
				data_value |= (1<<6);
			}
			else
			{
				data_value &= ~(1<<6);
			}
			if(fade_on_enable)
			{
				data_value |= (1<<5);
			}
			else
			{
				data_value &= ~(1<<5);
			}
			if(ret >= 0)
			{
					aw2013_i2c_master_send(g_i2c_client, AW2013_LED2_CFG_REG, &data_value,1);
			}
			break;
		default:
			break;
	}
}

static int aw2013_config_flash_mode_brightness(u8 led_index,u8 brightness)
{
	// brightness: 0x34 ~ 0x36
	// R7 R6 R5 R4 R3 R2 R1 R0
	//  0 ~ 255

	int ret = -1;
	u8 data_value = 0x00;

	data_value = (brightness & 0xff);

	switch (led_index)
	{
		case 0:
			// led0
			aw2013_i2c_master_send(g_i2c_client, AW2013_PWM0_REG, &data_value,1);
			break;
		case 1:
			// led1
			aw2013_i2c_master_send(g_i2c_client, AW2013_PWM1_REG, &data_value,1);
			break;
		case 2:
			// led2
			aw2013_i2c_master_send(g_i2c_client, AW2013_PWM2_REG, &data_value,1);
			break;
		default:
			break;
	}
}

static int aw2013_config_time(u8 led_index,u8 rise_time,u8 hold_time,u8 fall_time,u8 off_time,u8 delay_time,u8 repeat_time)
{
	// time contrl: 0x37 ~ 0x3F
	// R7 /  R6 R5 R4    / R3  / R2 R1 R0
	// X  /  rise time   / X   / hold_time
	// R7 /  R6 R5 R4    / R3  / R2 R1 R0
	// X  /  fall time   / X   / off time
	// R7 R6 R5 R4 / R3 R2 R1 R0
	// delay time  / repeat times (0 -> loop)

	int ret = -1;
	u8 data_value = 0x00;
	u8 hold_time_value = ((hold_time & 0x07)>5) ? 5 : (hold_time & 0x07);  	  // level 6
	u8 delay_time_value = ((delay_time & 0x0f)>8) ? 8 : (delay_time & 0x0f);  // level 9

	switch (led_index)
	{
		case 0:
			// led0
			data_value = ((rise_time & 0x07)<<4) | hold_time_value;
			aw2013_i2c_master_send(g_i2c_client, AW2013_LED0_TIME_REG0, &data_value,1);
			data_value = ((fall_time & 0x07)<<4) | (off_time & 0x07);
			aw2013_i2c_master_send(g_i2c_client, AW2013_LED0_TIME_REG1, &data_value,1);
			data_value = ((delay_time_value)<<4) | (repeat_time & 0x0f);
			aw2013_i2c_master_send(g_i2c_client, AW2013_LED0_TIME_REG2, &data_value,1);
			break;
		case 1:
			// led1
			data_value = ((rise_time & 0x07)<<4) | hold_time_value;
			aw2013_i2c_master_send(g_i2c_client, AW2013_LED1_TIME_REG0, &data_value,1);
			data_value = ((fall_time & 0x07)<<4) | (off_time & 0x07);
			aw2013_i2c_master_send(g_i2c_client, AW2013_LED1_TIME_REG1, &data_value,1);
			data_value = ((delay_time_value)<<4) | (repeat_time & 0x0f);
			aw2013_i2c_master_send(g_i2c_client, AW2013_LED1_TIME_REG2, &data_value,1);
			break;
		case 2:
			// led2
			data_value = ((rise_time & 0x07)<<4) | hold_time_value;
			aw2013_i2c_master_send(g_i2c_client, AW2013_LED2_TIME_REG0, &data_value,1);
			data_value = ((fall_time & 0x07)<<4) | (off_time & 0x07);	
			aw2013_i2c_master_send(g_i2c_client, AW2013_LED2_TIME_REG1, &data_value,1);
			data_value = ((delay_time_value)<<4) | (repeat_time & 0x0f);
			aw2013_i2c_master_send(g_i2c_client, AW2013_LED2_TIME_REG2, &data_value,1);
			break;
		default:
			break;
	}
}
/*********************************************************************************
** led function }
*********************************************************************************/

/*******************************************************************************
** ic control {
*******************************************************************************/
static int aw2013_make_effect(u8 type)
{

	if(led_state[type].enable > 0)
	{
		printk("<1> aw2013_make_effect begin \n");

		if(led_state[type].breath > 0)
		{
			aw2013_config_breath_mode(0,1); 	//  led0 breath mode
			aw2013_config_breath_mode(1,1); 	//  led1 breath mode
			aw2013_config_breath_mode(2,1); 	//  led2 breath mode
		}
		else
		{
			aw2013_config_breath_mode(0,0); 	//  led0 normal mode
			aw2013_config_breath_mode(1,0); 	//  led1 normal mode
			aw2013_config_breath_mode(2,0); 	//  led2 normal mode
		}

		aw2013_config_fade(0,1,1); 			// fade off -> 1 , fade_on -> 1
		aw2013_config_fade(1,1,1); 			// fade off -> 1 , fade_on -> 1
		aw2013_config_fade(2,1,1); 			// fade off -> 1 , fade_on -> 1

		// led0
		//if(led_state[type].r_ratio > 0)
		{
			aw2013_config_current_max(0,led_state[type].r_ratio); 		//  0ma, 5ma,10ma,15ma
			aw2013_config_flash_mode_brightness(0,BRIGHTNESS_LEVEL);    //  full 
			aw2013_config_time(0,led_state[type].rise_time,led_state[type].hold_time,led_state[type].fall_time,led_state[type].off_time,led_state[type].delay_time,led_state[type].period_num);  // /-\_   led1
		}

		// led1
		//if(led_state[type].g_ratio > 0)
		{
			aw2013_config_current_max(1,led_state[type].g_ratio); 		//  0ma, 5ma,10ma,15ma
			aw2013_config_flash_mode_brightness(1,BRIGHTNESS_LEVEL);    //  full 
			aw2013_config_time(1,led_state[type].rise_time,led_state[type].hold_time,led_state[type].fall_time,led_state[type].off_time,led_state[type].delay_time,led_state[type].period_num);  // /-\_   led1
		}

		// led2
		//if(led_state[type].b_ratio > 0)
		{
			aw2013_config_current_max(2,led_state[type].b_ratio); 		//  0ma, 5ma,10ma,15ma
			aw2013_config_flash_mode_brightness(2,BRIGHTNESS_LEVEL);    //  full 
			aw2013_config_time(2,led_state[type].rise_time,led_state[type].hold_time,led_state[type].fall_time,led_state[type].off_time,led_state[type].delay_time,led_state[type].period_num);  // /-\_   led1
		}

#if 0
		// led0
		if(led_state[type].r > 0)		
		{
			aw2013_config_enable(0,1); 			//  enable
		}
		else
		{
			aw2013_config_enable(0,0); 			//  disable
		}

		// led1
		if(led_state[type].g > 0)		
		{
			aw2013_config_enable(1,1); 			//  enable
		}
		else
		{
			aw2013_config_enable(1,0); 			//  disable
		}

		// led2
		if(led_state[type].b > 0 )	
		{
			aw2013_config_enable(2,1); 			//  enable
		}
		else
		{
			aw2013_config_enable(2,0); 			//  disable
		}
#else
		u8 rgb = (((led_state[type].b)<<2) | ((led_state[type].g)<<1) | (led_state[type].r));
		aw2013_config_enable_all(rgb); 			//  enable
#endif

		printk("<1> aw2013_make_effect end \n");
	}
	return 0 ; 
}


/*******************************************************************************
** ic control }
*******************************************************************************/

/*********************************************************************************************************
** led type 
*********************************************************************************************************/

static int led_power_on(u8 type,u8 on)
{
	if( (type < TYPE_MIN) || (type >= TYPE_MAX))
	{
		printk("<1> led power on off error\n");
		return -1;
	}

	printk("<1> led power %s success.\n",(on > 0 )?"on":"off");

	spin_lock(&led_lock);
	//down(&semaphore_led_mutex);

	led_state[type].enable = on;

	//up(&semaphore_led_mutex);
	spin_unlock(&led_lock);

#if 0 //hongzhe-dbg
	// if os is working mode , only charge work
	if((early_suspend_mode == 0) && (type != TYPE_CHARGE))
	{
		return 0;
	}
#endif

	led_process_effects();

	return 0;
}

int aw2013_power_on_standby(int enable, int press_time)
{
	if( keyhome_standby_breathled_enabled == 0 )enable = 0;


	
	led_power_on(TYPE_STANDBY, enable);
}

static int led_process_effects()
{
	int index = TYPE_MIN ;
	int first_power_on = -1;
	while(index < TYPE_MAX)
	{
		if(led_state[index].enable > 0)
		{
			first_power_on = index;	
			break;
		}
		index ++ ;
	}

	if(led_power == 0)  //if logic power off already ,do nothing ......
		return -1;

	printk("shaokai:<1> led_process_effects first_power_on = %d\n",first_power_on);

	if(index == TYPE_MAX)
	{
#if 0
		aw2013_config_enable(0,0); // power off led0
		aw2013_config_enable(1,0); // power off led1
		aw2013_config_enable(2,0); // power off led2
#else
		aw2013_config_enable_all(0x00); //power off led0,led1,led2 at same time.
#endif
		aw2013_config_power(0);    // power off ic
	}
	else
	{
#if 1
		aw2013_config_power(0);    // power off ic first
		msleep(1);
#endif
		aw2013_config_power(1); // power on ic

		aw2013_make_effect(first_power_on);
	}
}

static void led_test()
{
	char data = 0;
    char data_arr =  0x00 ;

	printk("<1> led_test() start \n");

	data_arr = 0x55;
	aw2013_i2c_master_send(g_i2c_client, 0x00, &data_arr,1);

	//ic module off control : 00 -> off  , 01 -> on
	data_arr = 0x01;
	aw2013_i2c_master_send(g_i2c_client, 0x01, &data_arr,1);

	// led rgb enable
	data_arr = 0x07;
	aw2013_i2c_master_send(g_i2c_client, 0x30, &data_arr,1);

	// current: 0x31 ~ 0x33, &03-> current . 4 -> breath, 
	data_arr = 0x73;
	//data_arr = 0x03; 
	aw2013_i2c_master_send(g_i2c_client, 0x31, &data_arr,1);

	data_arr = 0x73;
	//data_arr = 0x03;
	aw2013_i2c_master_send(g_i2c_client, 0x32, &data_arr,1);

	data_arr = 0x73;
	//data_arr = 0x03;
	aw2013_i2c_master_send(g_i2c_client, 0x33, &data_arr,1);


	// brightness : 0x34 35 36  
	data_arr = 0xff; // 0 ~ 255
	aw2013_i2c_master_send(g_i2c_client, 0x34, &data_arr,1);

	data_arr = 0xff;
	aw2013_i2c_master_send(g_i2c_client, 0x35, &data_arr,1);

	data_arr = 0xff;
	aw2013_i2c_master_send(g_i2c_client, 0x36, &data_arr,1);

#if 1
	// 37 ~ 39 -> led0
	// /-
	data_arr = 0x53;
	aw2013_i2c_master_send(g_i2c_client, 0x37, &data_arr,1);

	// \_
	data_arr = 0x55;
	aw2013_i2c_master_send(g_i2c_client, 0x38, &data_arr,1);

	// &0xf0 start delay time  ; &0x0f -> times, 0 means loop
	data_arr = 0x00;
	aw2013_i2c_master_send(g_i2c_client, 0x39, &data_arr,1);
#endif
#if 1
	// 3a ~ 3c -> led1
	data_arr = 0x43;
	aw2013_i2c_master_send(g_i2c_client, 0x3a, &data_arr,1);

	data_arr = 0x55;
	aw2013_i2c_master_send(g_i2c_client, 0x3b, &data_arr,1);

	data_arr = 0x00;
	aw2013_i2c_master_send(g_i2c_client, 0x3c, &data_arr,1);
#endif
#if 1
	// 3d ~ 3f -> led2
	data_arr = 0x33;
	aw2013_i2c_master_send(g_i2c_client, 0x3d, &data_arr,1);

	data_arr = 0x55;
	aw2013_i2c_master_send(g_i2c_client, 0x3e, &data_arr,1);

	data_arr = 0x00;
	aw2013_i2c_master_send(g_i2c_client, 0x3f, &data_arr,1);
#endif
	printk("<1> led_test() end \n");

}


#if defined(CONFIG_HAS_EARLYSUSPEND)
static void aw2013_early_suspend(struct early_suspend *h)
{
	early_suspend_mode = 1;

	// process led effects
	led_process_effects();	

}

static void aw2013_late_resume(struct early_suspend *h)
{
	early_suspend_mode = 0;
#if 0 //hongzhe-dbg
	spin_lock(&led_lock);


	// skip charging, the other close. default skip led_state[0] (charging state);
	memset(&led_state[1],0,sizeof(LED_CFG)*(TYPE_MAX-TYPE_MIN-1));

	spin_unlock(&led_lock);
#endif
	led_process_effects();
}
#endif
/*----------------------------------------------------------------------------------*/
/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static void aw2013_red_enable(u8 type,u8 enable,u8 breath,u8 ratio)
{
	if((type >= TYPE_MIN) && (type < TYPE_MAX))
	{
		spin_lock(&led_lock);
		led_state[type].breath = (breath>0)?1:0;
		led_state[type].r = 1;
		led_state[type].r_ratio = MAX(MIN(ratio,3),0);
		led_state[type].g = 0;
		led_state[type].g_ratio = 0;
		led_state[type].b = 0;
		led_state[type].b_ratio = 0;

		led_state[type].rise_time = AW2013_LED_TIME_0_52S; 
		led_state[type].hold_time = AW2013_LED_TIME_1_04S;
		led_state[type].fall_time = AW2013_LED_TIME_0_52S;
		led_state[type].off_time = AW2013_LED_TIME_1_04S;
		led_state[type].delay_time = AW2013_LED_DELAY_TIME_0S;
		led_state[type].period_num = AW2013_LED_REPEAT_ALWAYS; /* always */
		spin_unlock(&led_lock);
		
		enable = (enable > 0)? 1:0;
		led_power_on(type,enable); // power on / off led .

	}
}
static void aw2013_green_enable(u8 type,u8 enable,u8 breath,u8 ratio)
{
	if((type >= TYPE_MIN) && (type < TYPE_MAX))
	{
		spin_lock(&led_lock);
		led_state[type].breath = (breath>0)?1:0;
		led_state[type].g = 1;
		led_state[type].g_ratio = MAX(MIN(ratio,3),0);
		led_state[type].r = 0;
		led_state[type].r_ratio = 0;
		led_state[type].b = 0;
		led_state[type].b_ratio = 0;

		led_state[type].rise_time = AW2013_LED_TIME_0_52S; 
		led_state[type].hold_time = AW2013_LED_TIME_1_04S;
		led_state[type].fall_time = AW2013_LED_TIME_0_52S;
		led_state[type].off_time = AW2013_LED_TIME_1_04S;
		led_state[type].delay_time = AW2013_LED_DELAY_TIME_0S;
		led_state[type].period_num = AW2013_LED_REPEAT_ALWAYS; /* always */
		spin_unlock(&led_lock);
		
		enable = (enable > 0)? 1:0;
		led_power_on(type,enable); // power on / off led .

	}
}
static void aw2013_blue_enable(u8 type,u8 enable,u8 breath,u8 ratio)
{
	if((type >= TYPE_MIN) && (type < TYPE_MAX))
	{
		spin_lock(&led_lock);

		led_state[type].breath = (breath>0)?1:0;
		led_state[type].b = 1;
		led_state[type].b_ratio = MAX(MIN(ratio,3),0);
		led_state[type].g = 0;
		led_state[type].g_ratio = 0;
		led_state[type].r = 0;
		led_state[type].r_ratio = 0;

		led_state[type].rise_time = AW2013_LED_TIME_0_52S; 
		led_state[type].hold_time = AW2013_LED_TIME_1_04S;
		led_state[type].fall_time = AW2013_LED_TIME_0_52S;
		led_state[type].off_time = AW2013_LED_TIME_1_04S;
		led_state[type].delay_time = AW2013_LED_DELAY_TIME_0S;
		led_state[type].period_num = AW2013_LED_REPEAT_ALWAYS; /* always */

		spin_unlock(&led_lock);
		
		enable = (enable > 0)? 1:0;
		led_power_on(type,enable); // power on / off led .

	}
}
static void aw2013_white_enable(u8 type,u8 enable,u8 breath,u8 ratio)
{
	if((type >= TYPE_MIN) && (type < TYPE_MAX))
	{
		spin_lock(&led_lock);

		led_state[type].breath = (breath>0)?1:0;
		led_state[type].r = 1;
		led_state[type].r_ratio = MAX(MIN(ratio,3),0);
		led_state[type].g = 1;
		led_state[type].g_ratio = MAX(MIN(ratio,3),0);
		led_state[type].b = 1;
		led_state[type].b_ratio = MAX(MIN(ratio,3),0);

		led_state[type].rise_time = AW2013_LED_TIME_0_52S; 
		led_state[type].hold_time = AW2013_LED_TIME_1_04S;
		led_state[type].fall_time = AW2013_LED_TIME_0_52S;
		led_state[type].off_time = AW2013_LED_TIME_1_04S;
		led_state[type].delay_time = AW2013_LED_DELAY_TIME_0S;
		led_state[type].period_num = AW2013_LED_REPEAT_ALWAYS; /* always */

		spin_unlock(&led_lock);
		
		enable = (enable > 0)? 1:0;
		led_power_on(type,enable); // power on / off led .

	}
}

static void aw2013_standby_enable(u8 type,u8 enable,u8 breath,u8 ratio)
{
	spin_lock(&led_lock);
	led_state[TYPE_STANDBY].breath = 1;//(breath>0)?1:0;
	led_state[TYPE_STANDBY].r = 0;
	led_state[TYPE_STANDBY].r_ratio = MAX(MIN(ratio,3),0);
	led_state[TYPE_STANDBY].g = 1;
	led_state[TYPE_STANDBY].g_ratio = MAX(MIN(ratio,3),0);
	led_state[TYPE_STANDBY].b = 0;
	led_state[TYPE_STANDBY].b_ratio = MAX(MIN(ratio,3),0);

	led_state[TYPE_STANDBY].rise_time = AW2013_LED_TIME_0_52S; 
	led_state[TYPE_STANDBY].hold_time = AW2013_LED_TIME_1_04S;
	led_state[TYPE_STANDBY].fall_time = AW2013_LED_TIME_0_52S;
	led_state[TYPE_STANDBY].off_time = AW2013_LED_TIME_8_32S;
	led_state[TYPE_STANDBY].delay_time = AW2013_LED_DELAY_TIME_0_52S;
	led_state[TYPE_STANDBY].period_num = AW2013_LED_REPEAT_ALWAYS; 

	spin_unlock(&led_lock);
	enable = (enable > 0)? 1:0;
	keyhome_standby_breathled_enabled  = enable;
}

static int aw2013_open(struct inode *inode, struct file *file)
{
	return 0;
}
/*----------------------------------------------------------------------------*/
static int aw2013_release(struct inode *inode, struct file *file)
{
	return 0;
}
/*----------------------------------------------------------------------------*/

static long aw2013_unlocked_ioctl(struct file *file, unsigned int cmd,unsigned long arg) 
{
	long ret;
	void __user *data;
	LED_CFG_AW led_cfg;
	switch(cmd){
		case MZ_LED_RED_DEV:
		if(copy_from_user(&led_cfg, (void __user *)arg, sizeof(led_cfg)))
		{
		    printk("copy_from_user failed! line\n");
		    ret = -1;
		    break;
		}else{
		    aw2013_red_enable(led_cfg.type,led_cfg.enable,led_cfg.breath,led_cfg.ratio);
		    ret = 0;
		    break;
		}
		case MZ_LED_GREEN_DEV:
		if(copy_from_user(&led_cfg, (void __user *)arg, sizeof(led_cfg)))
		{
		    printk("copy_from_user failed! line\n");
		    ret = -1;
		    break;
		}else{
		    aw2013_green_enable(led_cfg.type,led_cfg.enable,led_cfg.breath,led_cfg.ratio);
		    ret = 0;
		    break;
		}		
		case MZ_LED_BLUE_DEV:
		if(copy_from_user(&led_cfg, (void __user *)arg, sizeof(led_cfg)))
		{
		    printk("copy_from_user failed! line\n");
		    ret = -1;
		    break;
		}else{
		    aw2013_blue_enable(led_cfg.type,led_cfg.enable,led_cfg.breath,led_cfg.ratio);
		    ret = 0;
		    break;
		}		
		case MZ_LED_WHITE_DEV:
		if(copy_from_user(&led_cfg, (void __user *)arg, sizeof(led_cfg)))
		{
		    printk("copy_from_user failed! line\n");
		    ret = -1;
		    break;
		}else{
		    aw2013_white_enable(led_cfg.type,led_cfg.enable,led_cfg.breath,led_cfg.ratio);
		    ret = 0;
		    break;
		}
		case MZ_LED_STANDBY_DEV:
		if(copy_from_user(&led_cfg, (void __user *)arg, sizeof(led_cfg)))
		{
		    printk("copy_from_user failed! line\n");
		    ret = -1;
		    break;
		}else{
		    aw2013_standby_enable(led_cfg.type,led_cfg.enable,led_cfg.breath,led_cfg.ratio);
		    ret = 0;
		    break;
		}		
	}
	return ret;

}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static struct file_operations aw2013_fops = {
	//.owner = THIS_MODULE,
	.open = aw2013_open,
	.release = aw2013_release,
	.unlocked_ioctl = aw2013_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice aw2013_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "aw2013",
	.fops = &aw2013_fops,
};
/*----------------------------------------------------------------------------*/

/******************************************************************************
 * Sysfs attributes
*******************************************************************************/
static ssize_t led_enable_show(struct device_driver *ddri, char *buf)
{
	   int len = 0 ;
	   int index = TYPE_MIN ;
       ssize_t res = 0 ;
	   u8 data_value = 0;
	   index = 0 ;
	   while(index < 16)
	   {
			data_value =  0 ;
			aw2013_i2c_master_recv(g_i2c_client,0x30+index,&data_value,1);

			len = snprintf(buf+res, PAGE_SIZE, "R[0x%x]=0x%x\n",0x30+index,data_value);
			res += len ;
			index ++ ;
	   }
       return res;
}

/*----------------------------------------------------------------------------*/

static ssize_t led_enable_store(struct device_driver *ddri, const char *buf, size_t count)
{
	   u8 type = 0; 
	   u8 enable = 0; 

       if(2 == sscanf(buf, "%d %d", &type,&enable))
       {
			if((type >= TYPE_MIN) && (type < TYPE_MAX))
			{
				enable = (enable > 0)? 1:0;

				led_power_on(type,enable); // power on / off led .
			}
       }

       return count;
}
static DRIVER_ATTR(enable,  S_IWUGO | S_IRUGO, led_enable_show,led_enable_store);

/******************************************************************************
 * Sysfs attributes
*******************************************************************************/
static ssize_t led_config_show(struct device_driver *ddri, char *buf)
{
	   int len = 0 ;
	   int index = TYPE_MIN ;
       ssize_t res = 0 ;
		
	   while(index < TYPE_MAX)
	   {
			len = snprintf(buf+res, PAGE_SIZE, "================================{\n");
			res += len ;
			len = snprintf(buf+res, PAGE_SIZE, "led_state[%02d].enable=%d\n",index,led_state[index].enable);
			res += len ;
			len = snprintf(buf+res, PAGE_SIZE, "led_state[%02d].breath=%d\n",index,led_state[index].breath);
			res += len ;
			len = snprintf(buf+res, PAGE_SIZE, "led_state[%02d].r=%d\n",index,led_state[index].r);
			res += len ;
			len = snprintf(buf+res, PAGE_SIZE, "led_state[%02d].r_ratio=%d\n",index,led_state[index].r_ratio);
			res += len ;
			len = snprintf(buf+res, PAGE_SIZE, "led_state[%02d].g=%d\n",index,led_state[index].g);
			res += len ;
			len = snprintf(buf+res, PAGE_SIZE, "led_state[%02d].g_ratio=%d\n",index,led_state[index].g_ratio);
			res += len ;
			len = snprintf(buf+res, PAGE_SIZE, "led_state[%02d].b=%d\n",index,led_state[index].b);
			res += len ;
			len = snprintf(buf+res, PAGE_SIZE, "led_state[%02d].b_ratio=%d\n",index,led_state[index].b_ratio);
			res += len ;
			len = snprintf(buf+res, PAGE_SIZE, "================================}\n");
			res += len ;
			index ++;
	   }

       return res;
}

/*----------------------------------------------------------------------------*/

static ssize_t led_config_store(struct device_driver *ddri, const char *buf, size_t count)
{
	   int type = 0; 
	   int breath= 0; 

	   int r = 0; 
	   int r_ratio = 0; 

	   int g = 0; 
	   int g_ratio = 0; 

	   int b = 0; 
	   int b_ratio = 0; 

       if(8 == sscanf(buf, "%d %d %d %d %d %d %d %d", &type,&breath,&r,&r_ratio,&g,&g_ratio,&b,&b_ratio))
       {
			if((type >= TYPE_MIN ) && (type < TYPE_MAX))
			{
				spin_lock(&led_lock);

				led_state[type].breath = (breath>0)?1:0;
				led_state[type].r = (r>0)?1:0 ;
				led_state[type].r_ratio = MAX(MIN(r_ratio,3),0);
				led_state[type].g = (g>0)?1:0;
				led_state[type].g_ratio = MAX(MIN(g_ratio,3),0);
				led_state[type].b = (b>0)?1:0 ;
				led_state[type].b_ratio = MAX(MIN(b_ratio,3),0);

				spin_unlock(&led_lock);
			}
       }

       return count;
}
static DRIVER_ATTR(config,  S_IWUGO | S_IRUGO, led_config_show,led_config_store);

/******************************************************************************
 * Sysfs attributes
*******************************************************************************/
static ssize_t led_power_show(struct device_driver *ddri, char *buf)
{
	   int len = 0 ;
       ssize_t res = 0 ;
	   len = snprintf(buf+res, PAGE_SIZE, "%d",led_power);
	   res += len ;
       return res;
}

/*----------------------------------------------------------------------------*/
static ssize_t led_power_store(struct device_driver *ddri, const char *buf, size_t count)
{
	   int value = 0; 

       if(1 == sscanf(buf, "%d", &value))
       {
			spin_lock(&led_lock);
			if(value == 0)	
			{
				led_power = 0 ;

				aw2013_config_enable_all(0x00); //power off led0,led1,led2 at same time.
				aw2013_config_power(0);

			}
			else
			{
				led_power = 1 ;
			}
			spin_unlock(&led_lock);
       }

       return count;
}
static DRIVER_ATTR(power,  S_IWUGO | S_IRUGO, led_power_show,led_power_store);

static struct i2c_driver aw2013_i2c_driver = 
{  
		.probe 	 	= aw2013_i2c_probe,
		.remove	 	= aw2013_i2c_remove,
		.suspend	= NULL,//aw2013_i2c_suspend,
		.resume	 	= NULL,//aw2013_i2c_resume,
		.id_table	= leds_id,
		.driver 	= 
		{
			//.owner 		 = THIS_MODULE,
			.name			 = AW2013_NAME,
		},
};

static int aw2013_init_client(struct i2c_client * client)
{

	int ret = -1;
	u8 data_value = 0x00;

			ret=aw2013_i2c_master_recv(client, 0x30, &data_value,1);
	if( ret == 1) return 0;
	else return -1;
}

static int aw2013_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
		int err = -1 ;

		printk("aw2013_i2c_probe start");

		g_i2c_client = client;

		if(err = aw2013_init_client(client))
		{
			goto exit_i2c_probe_faile;
		}

		if((err = driver_create_file(&aw2013_i2c_driver.driver, &driver_attr_enable)))
		{
			printk(KERN_INFO "dirver_create_file enable error !\n");
		}

		if((err = driver_create_file(&aw2013_i2c_driver.driver, &driver_attr_config)))
		{
			printk(KERN_INFO "dirver_create_file config error !\n");
		}

		if((err = driver_create_file(&aw2013_i2c_driver.driver, &driver_attr_power)))
		{
			printk(KERN_INFO "dirver_create_file power error !\n");
		}

		if(err = misc_register(&aw2013_device))
		{
			printk("aw2013_device register failed\n");
			misc_deregister(&aw2013_device);
		}
#ifdef CONFIG_HAS_EARLYSUSPEND
		early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1;
		early_drv.suspend  = aw2013_early_suspend;
		early_drv.resume   = aw2013_late_resume;
	    register_early_suspend(&early_drv);
#endif

		printk("aw2013_i2c_probe end");
		aw2013_is_present = 1;
		return 0;

exit_i2c_probe_faile:
		aw2013_is_present = 0;
		return -1;
}

static int aw2013_i2c_remove(struct i2c_client *client)
{ 
	int err =0;
	if(err = misc_deregister(&aw2013_device))
	{
		printk("misc_deregister fail: %d\n", err);
	}
	return 0;
}

static int aw2013_leds_probe(struct platform_device *pdev) 
{
		//printk("shaokai:aw2013_leds_probe");
		if(i2c_add_driver(&aw2013_i2c_driver))
		{
			printk(KERN_INFO "add driver error\n");
			return -1;
		}
		return 0;
}

static void aw2013_leds_shutdown(struct platform_device *dev)
{
    //GPIOLOG("shutdown\n");
	//printk("<1> shaokai debug aw2013_leds_shutdown() \n");

	aw2013_config_enable_all(0x00); //power off led0,led1,led2 at same time.
	aw2013_config_power(0);

	return ;
}

static int aw2013_leds_remove(struct platform_device *pdev)
{
		i2c_del_driver(&aw2013_i2c_driver);
		return 0;
}

struct platform_device aw2013_leds_device = 
{
		.name	 = AW2013_NAME,
		//.id	 = -1,
};

static struct platform_driver aw2013_leds_driver =
{
		.driver	 = 
		{
			.name	 = AW2013_NAME,
			//.owner	 = THIS_MODULE,
		},
		.probe 	 = aw2013_leds_probe,
		.remove	 = aw2013_leds_remove,
		.shutdown = aw2013_leds_shutdown
};

/* called when loaded into kernel */
static int __init aw2013_leds_init(void)
{
		int retval = -1;

		//printk(KERN_INFO "aw2013 driver init start\n");

		spin_lock_init(&led_lock);

		sema_init(&semaphore_led_mutex, 1);

		i2c_register_board_info(3, &i2c_leds, 1); 

		retval = platform_device_register(&aw2013_leds_device);
		if(retval < 0)
		{
			return -2;
		}

		if(platform_driver_register(&aw2013_leds_driver) < 0)
		{
			 //printk("platform register aw2013 driver failed\n");
			 return -1;
		}

		//printk(KERN_INFO "aw2013 driver init end.\n");

		return 0;
}
 
/* should never be called */
static void __exit aw2013_leds_exit(void)
{
		platform_driver_unregister(&aw2013_leds_driver);
		platform_device_unregister(&aw2013_leds_device);
}
 
module_init(aw2013_leds_init);
module_exit(aw2013_leds_exit);

/*----------------------------------------------------------------------------*/

MODULE_AUTHOR("shaokai@moztek.net");
MODULE_DESCRIPTION("motek aw2013 respiration lamp ic driver");
MODULE_LICENSE("GPL");
#endif

