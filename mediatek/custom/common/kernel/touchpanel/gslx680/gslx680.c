#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/byteorder/generic.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/rtpm_prio.h>
#include "tpd.h"
#include <cust_eint.h>

#include "tpd_custom_gslx680.h"

#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>

#include "cust_gpio_usage.h"
#include "gslx680.h"

//#define GSL_DEBUG
#define GSL_TIMER

#define GSLX680_NAME	"gslX680"
#define GSLX680_ADDR	0x40
#define MAX_FINGERS	  	5
#define MAX_CONTACTS	10
#define DMA_TRANS_LEN	0x20
#define SMBUS_TRANS_LEN	0x08
#define GSL_PAGE_REG		0xf0
#define ADD_I2C_DEVICE_ANDROID_4_0
//#define HIGH_SPEED_I2C
//#define FILTER_POINT
#ifdef FILTER_POINT
#define FILTER_MAX	6
#endif

static int tpd_flag = 0;
static int tpd_halt=0;
static char eint_flag = 0;
extern struct tpd_device *tpd;
static struct i2c_client *i2c_client = NULL;
static struct task_struct *thread = NULL;
#ifdef GSL_TIMER
//#undef TPD_PROC_DEBUG
#define GSL_TIMER_CHECK_CIRCLE2        200
#define GSL_TIMER_CHECK_CIRCLE        1000
static struct delayed_work gsl_timer_check_work;
static struct workqueue_struct *gsl_timer_workqueue = NULL;
static u32 gsl_timer_data = 0;
static volatile int gsl_timer_flag = 0;  // 0:first test  1:second test  2:doing gsl_load_fw
static volatile int gsl_halt_flag = 0;

#endif


static kal_uint32 id_sign[MAX_CONTACTS+1] = {0};
static kal_uint8 id_state_flag[MAX_CONTACTS+1] = {0};
static kal_uint8 id_state_old_flag[MAX_CONTACTS+1] = {0};
static kal_uint16 x_old[MAX_CONTACTS+1] = {0};
static kal_uint16 y_old[MAX_CONTACTS+1] = {0};
static kal_uint16 x_new = 0;
static kal_uint16 y_new = 0;
static int gsl_compatible_flag = 0X82;
static void gsl_compatible_id(struct i2c_client *client);
static int gslx680x_suspend = 0;

#ifdef MOZ_AW2013_SUPPORT
extern char keyhome_standby_breathled_enabled;
extern int aw2013_power_on_standby(int enable, int press_time);

#endif

static DECLARE_WAIT_QUEUE_HEAD(waiter);
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
		kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
		kal_bool auto_umask);

#ifdef GSL_DEBUG
#define print_info(fmt, args...)   \
        do{                              \
                printk(fmt, ##args);     \
        }while(0)
#else
#define print_info(fmt, args...)
#endif

#ifdef TPD_HAVE_BUTTON
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
static int tpd_wb_start_local[TPD_WARP_CNT] = TPD_WARP_START;
static int tpd_wb_end_local[TPD_WARP_CNT]   = TPD_WARP_END;
#endif
#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
static int tpd_calmat_local[8]     = TPD_CALIBRATION_MATRIX;
static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX;
#endif


#ifdef TPD_PROC_DEBUG
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
static struct proc_dir_entry *gsl_config_proc = NULL;
#define GSL_CONFIG_PROC_FILE "gsl_config"
#define CONFIG_LEN 31
static char gsl_read[CONFIG_LEN];
static u8 gsl_data_proc[8] = {0};
static u8 gsl_proc_flag = 0;
#endif


#define I2C_TRANS_SPEED 200
static u32 gsl_write_interfacexw(struct i2c_client *client,
        const u8 reg, u8 *buf, u32 num)
{
	struct i2c_msg xfer_msg[1] = {0};
	u8 tmp_buf[num + 1];

	tmp_buf[0] = reg;
	memcpy(tmp_buf + 1, buf, num);

	xfer_msg[0].addr = client->addr;
	xfer_msg[0].len = num + 1;
	xfer_msg[0].flags = client->flags & I2C_M_TEN;
	xfer_msg[0].buf = tmp_buf;
	xfer_msg[0].timing = I2C_TRANS_SPEED;

	return i2c_transfer(client->adapter, xfer_msg, 1);
}

static u32 gsl_read_interface(struct i2c_client *client,
        u8 reg, u8 *buf, u32 num)
{
#if 1
	struct i2c_msg xfer_msg[2] = {0};
	u32 ret = 0;
	u8 i;

	xfer_msg[0].addr = client->addr;
	xfer_msg[0].flags = 0;
	xfer_msg[0].len = 1;
	xfer_msg[0].buf = (char *)&reg;
	xfer_msg[0].timing = I2C_TRANS_SPEED;

	xfer_msg[1].addr = client->addr;
	xfer_msg[1].flags = I2C_M_RD;
	xfer_msg[1].len = num;
	xfer_msg[1].buf = buf;
	xfer_msg[1].timing = I2C_TRANS_SPEED;
	if(num > 8) {
		for(i = 0; i < num; i = i + 8)
		{
			if(num - i >= 8) {
				xfer_msg[1].len = 8;
			}
			else {
				xfer_msg[1].len = num - i;
			}
			xfer_msg[1].buf = buf + i;
			xfer_msg[0].buf = (char *)&reg;
			ret = i2c_transfer(client->adapter, xfer_msg, 2);
			reg += 8;
		}
		return ret; 
	}
	if(reg < 0x80)
		i2c_transfer(client->adapter, xfer_msg, 2);
	return i2c_transfer(client->adapter, xfer_msg, 2);
#else 
	u8 tmp_addr = 0
#endif
}



static void gsl_clear_chip(struct i2c_client *client)
{
	u8 buf[4]={0};
	//clear reg
	buf[0]=0x88;
	gsl_write_interfacexw(client,0xe0,buf,4);
//	i2c_smbus_write_i2c_block_data(client,0xe0,4,buf);
	
	msleep(20);
	buf[0]=0x1;
	gsl_write_interfacexw(client,0x80,buf,4);
//	i2c_smbus_write_i2c_block_data(client,0x80,4,buf);
	
	msleep(5);
	buf[0]=0x4;
	gsl_write_interfacexw(client,0xe4,buf,4);	
//	i2c_smbus_write_i2c_block_data(client,0xe4,4,buf);
	
	msleep(5);
	buf[0]=0x0;
	gsl_write_interfacexw(client,0xe0,buf,4);	
//	i2c_smbus_write_i2c_block_data(client,0xe0,4,buf);
	msleep(20);
	//clear reg
}
static void startup_chip(struct i2c_client *client)
{
	char write_buf = 0x00;
		int ret;

	u8 buf[4] = {0};
	buf[0]=0;
	buf[1]=0x10;
	buf[2]=0xfe;
	buf[3]=0x1;
	gsl_write_interfacexw(client,0xf0,buf,4);
//	i2c_smbus_write_i2c_block_data(client,0xf0,4,buf);
	buf[0]=0xf;
	buf[1]=0;
	buf[2]=0;
	buf[3]=0;
	gsl_write_interfacexw(client,0x4,buf,4);
//	i2c_smbus_write_i2c_block_data(client,0x4,4,buf);
	msleep(20);
	buf[0]=0;
	buf[1]=0;
	buf[2]=0;
	buf[3]=0;
	gsl_write_interfacexw(client,0xe0,buf,4);	
	//ret = i2c_smbus_write_i2c_block_data(client, 0xe0, 1, &write_buf);
	mdelay(10);
}

static int reset_chip(struct i2c_client *client)
{
	char write_buf[4]	= {0};
	int ret = 0;
printk("############################################################################# CTP RESET CHIP %d\n",__LINE__);
	write_buf[0] = 0x88;
	gsl_write_interfacexw(client,0xe0,write_buf,4);
//	i2c_smbus_write_i2c_block_data(client, 0xe0, 1, &write_buf[0]);
	msleep(5);
printk("############################################################################# CTP RESET CHIP %d\n",__LINE__);
	write_buf[0] = 0x04;

	gsl_write_interfacexw(client,0xe4,write_buf,4);
	msleep(5);
printk("############################################################################# CTP RESET CHIP %d\n",__LINE__);
	write_buf[0] = 0x00;
	write_buf[1] = 0x00;
	write_buf[2] = 0x00;
	write_buf[3] = 0x00;
	gsl_write_interfacexw(client,0xbc,write_buf,4);
//	ret = i2c_smbus_write_i2c_block_data(client, 0xbc, 4, write_buf);
	mdelay(10);
	return 0;
printk("############################################################################# CTP RESET CHIP %d\n",__LINE__);
}


#ifdef TPD_PROC_DEBUG
static int char_to_int(char ch)
{
	if(ch>='0' && ch<='9')
		return (ch-'0');
	else
		return (ch-'a'+10);
}

static int gsl_config_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	char *ptr = page;
	//char temp_data[4] = {0};
	char temp_data[5] = {0};
	//int i;
	unsigned int tmp=0;
	if('v'==gsl_read[0]&&'s'==gsl_read[1])
	{
#ifdef GSL_ALG_ID
		tmp=gsl_version_id();
#else
		tmp=0x20121215;
#endif
		ptr += sprintf(ptr,"version:%x\n",tmp);
	}
	else if('r'==gsl_read[0]&&'e'==gsl_read[1])
	{
		if('i'==gsl_read[3])
		{
#ifdef GSL_ALG_ID
			tmp=(gsl_data_proc[5]<<8) | gsl_data_proc[4];
			ptr +=sprintf(ptr,"gsl_config_data_id[%d] = ",tmp);
			if(tmp>=0&&tmp<256)
				ptr +=sprintf(ptr,"%d\n",gsl_config_data_id[tmp]);
#endif
		}
		else
		{
            //i2c_smbus_read_i2c_block_data(i2c_client, 0x80,8,tmp_buf);
	    		gsl_write_interfacexw(i2c_client,0xf0,&gsl_data_proc[4],4);
			gsl_read_interface(i2c_client,gsl_data_proc[0],temp_data,4);
		//	i2c_smbus_read_i2c_block_data(i2c_client,gsl_data_proc[0],4,temp_data);
			ptr +=sprintf(ptr,"offset : {0x%02x,0x",gsl_data_proc[0]);
			ptr +=sprintf(ptr,"%02x",temp_data[3]);
			ptr +=sprintf(ptr,"%02x",temp_data[2]);
			ptr +=sprintf(ptr,"%02x",temp_data[1]);
			ptr +=sprintf(ptr,"%02x};\n",temp_data[0]);
		}
	}
	*eof = 1;
	return (ptr - page);
}
static int gsl_config_write_proc(struct file *file, const char *buffer, unsigned long count, void *data)
{
	u8 buf[8] = {0};
	//u8 addr = 0;
	int tmp = 0;
	int tmp1 = 0;
	print_info("[tp-gsl][%s] \n",__func__);

	if(count > CONFIG_LEN)
	{
		print_info("size not match [%d:%ld]\n", CONFIG_LEN, count);
        	return -EFAULT;
	}

	if(copy_from_user(gsl_read, buffer, (count<CONFIG_LEN?count:CONFIG_LEN)))
	{
		print_info("copy from user fail\n");
        	return -EFAULT;
	}
	print_info("[tp-gsl][%s][%s]\n",__func__,gsl_read);

	buf[3]=char_to_int(gsl_read[14])<<4 | char_to_int(gsl_read[15]);
	buf[2]=char_to_int(gsl_read[16])<<4 | char_to_int(gsl_read[17]);
	buf[1]=char_to_int(gsl_read[18])<<4 | char_to_int(gsl_read[19]);
	buf[0]=char_to_int(gsl_read[20])<<4 | char_to_int(gsl_read[21]);

	buf[7]=char_to_int(gsl_read[5])<<4 | char_to_int(gsl_read[6]);
	buf[6]=char_to_int(gsl_read[7])<<4 | char_to_int(gsl_read[8]);
	buf[5]=char_to_int(gsl_read[9])<<4 | char_to_int(gsl_read[10]);
	buf[4]=char_to_int(gsl_read[11])<<4 | char_to_int(gsl_read[12]);
	if('v'==gsl_read[0]&& 's'==gsl_read[1])//version //vs
	{
		printk("gsl version\n");
	}
	else if('s'==gsl_read[0]&& 't'==gsl_read[1])//start //st
	{
		gsl_proc_flag = 1;
		reset_chip(i2c_client);
		/*msleep(20);
		mt_set_gpio_out(GPIO_CTP_RST_PIN, 0);
		msleep(20);
		mt_set_gpio_out(GPIO_CTP_RST_PIN, 1);
		msleep(20);
		*/
		//gsl_start_core(ddata->client);
	}
	else if('e'==gsl_read[0]&&'n'==gsl_read[1])//end //en
	{
		msleep(20);
		reset_chip(i2c_client);
		startup_chip(i2c_client);
#ifdef GSL_ALG_ID
		gsl_DataInit(gsl_config_data_id);
#endif
		gsl_proc_flag = 0;
	}
	else if('r'==gsl_read[0]&&'e'==gsl_read[1])//read buf //
	{
		memcpy(gsl_data_proc,buf,8);
	}
	else if('w'==gsl_read[0]&&'r'==gsl_read[1])//write buf
	{
		i2c_smbus_write_i2c_block_data(i2c_client,buf[4],4,buf);
	}

#ifdef GSL_ALG_ID
	else if('i'==gsl_read[0]&&'d'==gsl_read[1])//write id config //
	{
		tmp1=(buf[7]<<24)|(buf[6]<<16)|(buf[5]<<8)|buf[4];
		tmp=(buf[3]<<24)|(buf[2]<<16)|(buf[1]<<8)|buf[0];
		if(tmp1>=0 && tmp1<256)
		{
			gsl_config_data_id[tmp1] = tmp;
		}
	}
#endif
	return count;
}
#endif


#if 1 //hongzhe-dbg
#ifdef HIGH_SPEED_I2C
static u32 gsl_read_interface(struct i2c_client *client, u8 reg, u8 *buf, u32 num)
{
	struct i2c_msg xfer_msg[2];

	xfer_msg[0].addr = client->addr;
	xfer_msg[0].len = 1;
	xfer_msg[0].flags = client->flags & I2C_M_TEN;
	xfer_msg[0].buf = &reg;
	xfer_msg[0].timing = 400;

	xfer_msg[1].addr = client->addr;
	xfer_msg[1].len = num;
	xfer_msg[1].flags |= I2C_M_RD;
	xfer_msg[1].buf = buf;
	xfer_msg[1].timing = 400;

	if (reg < 0x80) {
		i2c_transfer(client->adapter, xfer_msg, ARRAY_SIZE(xfer_msg));
		mdelay(5);
	}

	return i2c_transfer(client->adapter, xfer_msg, ARRAY_SIZE(xfer_msg)) == ARRAY_SIZE(xfer_msg) ? 0 : -EFAULT;
}

static u32 gsl_write_interface(struct i2c_client *client, const u8 reg, u8 *buf, u32 num)
{
	struct i2c_msg xfer_msg[1];

	buf[0] = reg;

	xfer_msg[0].addr = client->addr;
	xfer_msg[0].len = num + 1;
	xfer_msg[0].flags = client->flags & I2C_M_TEN;
	xfer_msg[0].buf = buf;
	xfer_msg[0].timing = 400;

	return i2c_transfer(client->adapter, xfer_msg, 1) == 1 ? 0 : -EFAULT;
}

static __inline__ void fw2buf(u8 *buf, const u32 *fw)
{
	u32 *u32_buf = (int *)buf;
	*u32_buf = *fw;
}

static void gsl_load_fw(struct i2c_client *client)
{
	u8 buf[DMA_TRANS_LEN*4 + 1] = {0};
	u8 send_flag = 1;
	u8 *cur = buf + 1;
	u32 source_line = 0;
	u32 source_len = 0;
	struct fw_data *GSLX680_FW = NULL;

	if (0X88 == gsl_compatible_id(client))
	{
		GSLX680_FW = GSLX680A_FW;
		source_len = ARRAY_SIZE(GSLX680A_FW);
	}
	else
	{
		GSLX680_FW = GSLX680B_FW;
		source_len = ARRAY_SIZE(GSLX680B_FW);
	}



	printk("=============gsl_load_fw start==============\n");

	for (source_line = 0; source_line < source_len; source_line++)
	{
		/* init page trans, set the page val */
		if (GSL_PAGE_REG == GSLX680_FW[source_line].offset)
		{
			fw2buf(cur, &GSLX680_FW[source_line].val);
			gsl_write_interface(client, GSL_PAGE_REG, buf, 4);
			send_flag = 1;
		}
		else
		{
			if (1 == send_flag % (DMA_TRANS_LEN < 0x20 ? DMA_TRANS_LEN : 0x20))
	    			buf[0] = (u8)GSLX680_FW[source_line].offset;

			fw2buf(cur, &GSLX680_FW[source_line].val);
			cur += 4;

			if (0 == send_flag % (DMA_TRANS_LEN < 0x20 ? DMA_TRANS_LEN : 0x20))
			{
	    			gsl_write_interface(client, buf[0], buf, cur - buf - 1);
	    			cur = buf + 1;
			}

			send_flag++;
		}
	}

	printk("=============gsl_load_fw end==============\n");

}
#else
static void gsl_load_fw(struct i2c_client *client)
{
	char buf[SMBUS_TRANS_LEN*4] = {0};
	char reg = 0, send_flag = 1, cur = 0;
	int ret;
	unsigned int source_line = 0;
	u32 source_len = 0;
	struct fw_data *GSLX680_FW = NULL;
	gsl_compatible_id(client);
	if (0X88 == gsl_compatible_flag)
	{
		GSLX680_FW = GSLX680A_FW;
		source_len = ARRAY_SIZE(GSLX680A_FW);
	}
	else
	{
		GSLX680_FW = GSLX680B_FW;
		source_len = ARRAY_SIZE(GSLX680B_FW);
	}

	printk("=============gsl_load_fw start==============\n");

	for (source_line = 0; source_line < source_len; source_line++)
	{
		/* init page trans, set the page val */
		if (GSL_PAGE_REG == GSLX680_FW[source_line].offset)
		{
			buf[0] = (char)(GSLX680_FW[source_line].val & 0x000000ff);
			ret = i2c_smbus_write_i2c_block_data(client, GSL_PAGE_REG, 1, &buf[0]);
			if  (ret  < 0)
			{
				printk("[hongzhe-gslx680] I2C transfer error!!!!!!!!!!!!!!!!!111\n");
				break;
			}

			send_flag = 1;
		}
		else
		{
			//if (1 == send_flag % (SMBUS_TRANS_LEN < 0x08 ? SMBUS_TRANS_LEN : 0x08))
				reg = GSLX680_FW[source_line].offset;

			buf[cur + 0] = (char)(GSLX680_FW[source_line].val & 0x000000ff);
			buf[cur + 1] = (char)((GSLX680_FW[source_line].val & 0x0000ff00) >> 8);
			buf[cur + 2] = (char)((GSLX680_FW[source_line].val & 0x00ff0000) >> 16);
			buf[cur + 3] = (char)((GSLX680_FW[source_line].val & 0xff000000) >> 24);
			//cur += 4;

			//if (0 == send_flag % (SMBUS_TRANS_LEN < 0x08 ? SMBUS_TRANS_LEN : 0x08))
			{
				ret = i2c_smbus_write_i2c_block_data(client, reg, /*SMBUS_TRANS_LEN**/4, buf);
				if  (ret  < 0)
				{
					printk("[hongzhe-gslx680] I2C transfer error!!!!!!!!!!!!!!!!!!!!!!!!!!222\n");
					break;
				}
				cur = 0;
			}

			//send_flag++;

		}
	}

	printk("=============gsl_load_fw end==============\n");

}
#endif
#endif //hongzhe-dbg
static void test_i2c(struct i2c_client *client)
{
	char read_buf = 0;
	char write_buf = 0x12;
	int ret;
	ret = i2c_smbus_read_i2c_block_data( client, 0xf0, 1, &read_buf );
	if  (ret  < 0)
	{
		printk("I2C transfer error!\n");
	}
	else
	{
		printk("I read reg 0xf0 is %x\n", read_buf);
	}
	mdelay(10);

	ret = i2c_smbus_write_i2c_block_data( client, 0xf0, 1, &write_buf );
	if  (ret  < 0)
	{
		printk("I2C transfer error!\n");
	}
	else
	{
		printk("I write reg 0xf0 0x12\n");
	}
	mdelay(10);

	ret = i2c_smbus_read_i2c_block_data( client, 0xf0, 1, &read_buf );
	if  (ret  <  0 )
	{
		printk("I2C transfer error!\n");
	}
	else
	{
		printk("I read reg 0xf0 is 0x%x\n", read_buf);
	}
	mdelay(10);

}
static void init_chip(struct i2c_client *client)
{
	int ret;
	static volatile int gsl_sw_flag=0;
	if(1==gsl_sw_flag)
		return;
	gsl_sw_flag=1;
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
	mdelay(20);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	mdelay(30);
	//test_i2c(client);
	//test_i2c(client);
	gsl_clear_chip(client);
	ret = reset_chip(client);
	if( ret<0 ) return;
	gsl_load_fw(client);
	startup_chip(client);
	gsl_sw_flag=0;
	
}

static void check_mem_data(struct i2c_client *client)
{
	char write_buf;
	char read_buf[4]  = {0};

	mdelay(30);
	i2c_smbus_read_i2c_block_data(client, 0xb0, 4, read_buf);
	print_info("!!!!!!!!!!!page: %x offset: %x val: %x %x %x %x\n",0x0, 0x0, read_buf[3], read_buf[2], read_buf[1], read_buf[0]);

	if (read_buf[3] != 0x5a || read_buf[2] != 0x5a || read_buf[1] != 0x5a || read_buf[0] != 0x5a)
	{
		printk("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX CTP MEM ERROR!!!\n");
		init_chip(client);
	}
}

#ifdef FILTER_POINT
static void filter_point(u16 x, u16 y , u8 id)
{
	u16 x_err =0;
	u16 y_err =0;
	u16 filter_step_x = 0, filter_step_y = 0;

	id_sign[id] = id_sign[id] + 1;
	if(id_sign[id] == 1)
	{
		x_old[id] = x;
		y_old[id] = y;
	}

	x_err = x > x_old[id] ? (x -x_old[id]) : (x_old[id] - x);
	y_err = y > y_old[id] ? (y -y_old[id]) : (y_old[id] - y);

	if( (x_err > FILTER_MAX && y_err > FILTER_MAX/3) || (x_err > FILTER_MAX/3 && y_err > FILTER_MAX) )
	{
		filter_step_x = x_err;
		filter_step_y = y_err;
	}
	else
	{
		if(x_err > FILTER_MAX)
			filter_step_x = x_err;
		if(y_err> FILTER_MAX)
			filter_step_y = y_err;
	}

	if(x_err <= 2*FILTER_MAX && y_err <= 2*FILTER_MAX)
	{
		filter_step_x >>= 2;
		filter_step_y >>= 2;
	}
	else if(x_err <= 3*FILTER_MAX && y_err <= 3*FILTER_MAX)
	{
		filter_step_x >>= 1;
		filter_step_y >>= 1;
	}

	x_new = x > x_old[id] ? (x_old[id] + filter_step_x) : (x_old[id] - filter_step_x);
	y_new = y > y_old[id] ? (y_old[id] + filter_step_y) : (y_old[id] - filter_step_y);

	x_old[id] = x_new;
	y_old[id] = y_new;
}
#else

static void record_point(u16 x, u16 y , u8 id)
{
	u16 x_err =0;
	u16 y_err =0;

	id_sign[id]=id_sign[id]+1;

	if(id_sign[id]==1){
		x_old[id]=x;
		y_old[id]=y;
	}

	x = (x_old[id] + x)/2;
	y = (y_old[id] + y)/2;

	if(x>x_old[id]){
		x_err=x -x_old[id];
	}
	else{
		x_err=x_old[id]-x;
	}

	if(y>y_old[id]){
		y_err=y -y_old[id];
	}
	else{
		y_err=y_old[id]-y;
	}

	if( (x_err > 3 && y_err > 1) || (x_err > 1 && y_err > 3) ){
		x_new = x;     x_old[id] = x;
		y_new = y;     y_old[id] = y;
	}
	else{
		if(x_err > 3){
			x_new = x;     x_old[id] = x;
		}
		else
			x_new = x_old[id];
		if(y_err> 3){
			y_new = y;     y_old[id] = y;
		}
		else
			y_new = y_old[id];
	}

	if(id_sign[id]==1){
		x_new= x_old[id];
		y_new= y_old[id];
	}

}
#endif

void tpd_down( int id, int x, int y, int p)
{
	print_info("------tpd_down id: %d, x:%d, y:%d------ \n", id, x, y);

	input_report_key(tpd->dev, BTN_TOUCH, 1);
	input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 1);
	input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
	input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, id);
	input_mt_sync(tpd->dev);

	if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
	{
	#ifdef TPD_HAVE_BUTTON
		tpd_button(x, y, 1);
	#endif
	}
	if(y > TPD_RES_Y) //virtual key debounce to avoid android ANR issue
	{
		 mdelay(50);
		 print_info("D virtual key \n");
	}
}

void tpd_up(void)
{
	print_info("------tpd_up------ \n");

	input_report_key(tpd->dev, BTN_TOUCH, 0);
	input_mt_sync(tpd->dev);

	if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
	{
	#ifdef TPD_HAVE_BUTTON
	    tpd_button(0, 0, 0);
	#endif
	}
}


#ifdef GSL_TIMER
static void gsl_timer_check_func(struct work_struct *work)
{	
	u8 buf[4] = {0};
	u32 tmp;
	int i;
	int err;
	static int timer_count;
	if(gsl_halt_flag == 1){ 
		return;
	}
	//buf[0] = 0x9f;
	//gsl_write_interface(ddata->client, GSL_PAGE_REG, buf, 4);
	
//	i2c_smbus_write_i2c_block_data(i2c_client,0xb4,4,buf);
	gsl_read_interface(i2c_client, 0xb4, buf, 4);
	tmp = (buf[3]<<24)|(buf[2]<<16)|(buf[1]<<8)|(buf[0]);
	
	printk("[tp-gsl][pre] 0xb4 = %x \n",gsl_timer_data);
	printk("[tp-gsl][cur] 0xb4 = %x \n",tmp);
	printk("[tp-gsl]gsl_timer_flag=%d\n",gsl_timer_flag);
	printk("[tp-gsl] timer_count = %d \n",timer_count);
	if(0 == gsl_timer_flag)
	{
		if(tmp==gsl_timer_data)
		{
			gsl_timer_flag = 1;
			if(0==gsl_halt_flag)
			{
				queue_delayed_work(gsl_timer_workqueue, &gsl_timer_check_work, 25);
			}
		}
		else
		{
			for(i=0;i<3;i++){
				gsl_read_interface(i2c_client,0xbc,buf,4);
				if(buf[0]==0&&buf[1]==0&&buf[2]==0&&buf[3]==0){
					err = 1;
					break;
				}else{
					err =0;
				}
			}
			if(err == 0){
				reset_chip(i2c_client);
				startup_chip(i2c_client);
			}
			gsl_timer_flag = 0;
			timer_count = 0;
			if(0 == gsl_halt_flag)
			{
				queue_delayed_work(gsl_timer_workqueue, 
					&gsl_timer_check_work, GSL_TIMER_CHECK_CIRCLE2);
			}
		}
	}
	else if(1==gsl_timer_flag){
		if(tmp==gsl_timer_data)
		{
			if(0==gsl_halt_flag)
			{
				timer_count++;
				gsl_timer_flag = 2;
				//gsl_sw_init(ddata->client);
				init_chip(i2c_client);
				gsl_timer_flag = 1;
			}
			if(0 == gsl_halt_flag && timer_count < 20)
			{
				queue_delayed_work(gsl_timer_workqueue, 
					&gsl_timer_check_work, GSL_TIMER_CHECK_CIRCLE2);
			}
		}
		else{
			timer_count = 0;
			if(0 == gsl_halt_flag && timer_count < 20)
			{
				queue_delayed_work(gsl_timer_workqueue, 
					&gsl_timer_check_work, GSL_TIMER_CHECK_CIRCLE2);
			}
		}
		gsl_timer_flag = 0;
	}
	gsl_timer_data = tmp;
}
#endif

static void gsl_report_point(struct gsl_touch_info *ti)
{
    int tmp = 0;
	static int gsl_up_flag = 0; //prevent more up event
    print_info("gsl_report_point %d \n", ti->finger_num);

#if 0 //def TPD_KEYHOME_STANDBY_BREATH_LED_SUPPORT
	if(gslx680x_suspend == 1 && 1== keyhome_standby_breathled_enabled )
	{
		if( unlikely(ti->finger_num == 1)
				&& ti->x[0]>=tpd_keys_dim_local[TPD_KEY_HOME_INDEX][0]-(tpd_keys_dim_local[TPD_KEY_HOME_INDEX][2]/2) 
		              &&  ti->x[0]<=tpd_keys_dim_local[TPD_KEY_HOME_INDEX][0]+(tpd_keys_dim_local[TPD_KEY_HOME_INDEX][2]/2)
		              &&  ti->y[0]>=tpd_keys_dim_local[TPD_KEY_HOME_INDEX][1]-(tpd_keys_dim_local[TPD_KEY_HOME_INDEX][3]/2)
		              &&  ti->y[0]<=tpd_keys_dim_local[TPD_KEY_HOME_INDEX][1]+(tpd_keys_dim_local[TPD_KEY_HOME_INDEX][3]/2) 
				)
		{
			gsl_up_flag = 1;

		}
		else if(unlikely(ti->finger_num == 0)){
		    	if(gsl_up_flag == 0)
					return;
		    	gsl_up_flag = 0;		
#ifdef MOZ_AW2013_SUPPORT				
			aw2013_power_on_standby(1, 0);
#endif
		}

		return;
	}
#endif


    if (unlikely(ti->finger_num == 0))
    {
    	if(gsl_up_flag == 0)
			return;
    	gsl_up_flag = 0;
        input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 0);
        input_report_key(tpd->dev, BTN_TOUCH, 0);
        input_mt_sync(tpd->dev);
	    if (FACTORY_BOOT == get_boot_mode()||
			RECOVERY_BOOT == get_boot_mode())
		{

			tpd_button(ti->x[tmp], ti->y[tmp], 0);

		}
	}
	else
	{
		gsl_up_flag = 1;
		for (tmp = 0; ti->finger_num > tmp; tmp++)
		{
			print_info("[gsl1680](x[%d],y[%d]) = (%d,%d);\n",
				ti->id[tmp], ti->id[tmp], ti->x[tmp], ti->y[tmp]);

			input_report_key(tpd->dev, BTN_TOUCH, 1);
			input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 1);

			if(0!=NORMAL_BOOT && (MTK_LCM_PHYSICAL_ROTATION == 270
				|| MTK_LCM_PHYSICAL_ROTATION == 90) )
			{
				int temp;
				temp = ti->y[tmp];
				ti->y[tmp] = ti->x[tmp];
				ti->x[tmp] = TPD_RES_X-temp;
			}

			if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
			{
				tpd_button(ti->x[tmp], ti->y[tmp], 1);
			}
			input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, ti->id[tmp] - 1);
			input_report_abs(tpd->dev, ABS_MT_POSITION_X, ti->x[tmp]);
			input_report_abs(tpd->dev, ABS_MT_POSITION_Y, ti->y[tmp]);

			input_mt_sync(tpd->dev);
		}
	}
	input_sync(tpd->dev);
}

static void gsl_report_work(void)
{

	u8 buf[4] = {0};
	u8 i = 0;
	u16 ret = 0;
	u16 tmp = 0;
	struct gsl_touch_info cinfo={0};
	u8 tmp_buf[44] ={0};
	//print_info("enter gsl_report_work\n");


#ifdef GSL_TIMER 
	if(2==gsl_timer_flag){
		goto gsl_report_work_out;
	}
#endif


#ifdef TPD_PROC_DEBUG
	if(gsl_proc_flag == 1){
		return;
	}
#endif

 	i2c_smbus_read_i2c_block_data(i2c_client, 0x80,8,tmp_buf);
	//if(tmp_buf[0]>=2&&tmp_buf[0]<=4)
	if(tmp_buf[0]>=2)
		i2c_smbus_read_i2c_block_data(i2c_client, 0x88,8,&tmp_buf[8]);
	if(tmp_buf[0]>=4)
		i2c_smbus_read_i2c_block_data(i2c_client, 0x90,8,&tmp_buf[16]);
	if(tmp_buf[0]>=6)
		i2c_smbus_read_i2c_block_data(i2c_client, 0x98,8,&tmp_buf[24]);

	if(tmp_buf[0]>=8)
		i2c_smbus_read_i2c_block_data(i2c_client, 0xa0,8,&tmp_buf[32]);
	if(10==tmp_buf[0])
		i2c_smbus_read_i2c_block_data(i2c_client, 0xa8,4,&tmp_buf[40]);
	cinfo.finger_num = tmp_buf[0] & 0x0f;
	//print_info("tp-gsl  finger_num = %d\n",cinfo.finger_num);
	for(tmp=0;tmp<(cinfo.finger_num>10?10:cinfo.finger_num);tmp++)
	{
		cinfo.id[tmp] = tmp_buf[tmp*4+7] >> 4;
		cinfo.y[tmp] = (tmp_buf[tmp*4+4] | ((tmp_buf[tmp*4+5])<<8));
		cinfo.x[tmp] = (tmp_buf[tmp*4+6] | ((tmp_buf[tmp*4+7] & 0x0f)<<8));
		//print_info("tp-gsl  x = %d y = %d \n",cinfo.x[tmp],cinfo.y[tmp]);
	}
#ifdef GSL_ALG_ID
	int tmp1 = 0;
	//print_info("cinfo.finger1111=%d\n",cinfo.finger_num);
	cinfo.finger_num = (tmp_buf[3]<<24)|(tmp_buf[2]<<16)|(tmp_buf[1]<<8)|(tmp_buf[0]);
	//print_info("cinfo.finger2222=%d\n",cinfo.finger_num);
	gsl_alg_id_main(&cinfo);
	//print_info("cinfo.finger3333=%d\n",cinfo.finger_num);
	

	tmp1=gsl_mask_tiaoping();
	//print_info("[tp-gsl] tmp1=%x\n",tmp1);
	if(tmp1>0&&tmp1<0xffffffff)
	{
		buf[0]=0xa;
		buf[1]=0;
		buf[2]=0;
		buf[3]=0;
		i2c_smbus_write_i2c_block_data(i2c_client,0xf0,4,buf);
		buf[0]=(u8)(tmp1 & 0xff);
		buf[1]=(u8)((tmp1>>8) & 0xff);
		buf[2]=(u8)((tmp1>>16) & 0xff);
		buf[3]=(u8)((tmp1>>24) & 0xff);
		//printk("tmp1=%08x,buf[0]=%02x,buf[1]=%02x,buf[2]=%02x,buf[3]=%02x\n",tmp1,buf[0],buf[1],buf[2],buf[3]);
		i2c_smbus_write_i2c_block_data(i2c_client,0x8,4,buf);
	}
#endif
	if(MTK_LCM_PHYSICAL_ROTATION == 270 || MTK_LCM_PHYSICAL_ROTATION == 90)
	{
		print_info("MTK_LCM_PHYSICAL_ROTATION = %d\n",MTK_LCM_PHYSICAL_ROTATION);
		for(i = 0;i < 5 ;i++)
		{
			ret = cinfo.x[i];
			cinfo.x[i] = cinfo.y[i];
			cinfo.y[i] = ret;
		}
	}

	gsl_report_point(&cinfo);

gsl_report_work_out:
	mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);

}

static void report_data_handle(void)
{
	char touch_data[MAX_FINGERS * 4 + 4] = {0};
	char buf[4] = {0};
	char id, point_num = 0;
	unsigned int x, y, temp_a, temp_b, i;
	#ifdef GSL_ALG_ID
	int tmp1 = 0;
	static struct gsl_touch_info cinfo={0};
	#endif


	
#ifdef GSL_TIMER 
		if(2==gsl_timer_flag){
			return;
		}
#endif


	i2c_smbus_read_i2c_block_data(i2c_client, 0xbc, 4, buf);
	if(buf[3] == 0 && buf[2] == 0 && buf[1] == 0 && buf[0] == 0 )
	{
		i2c_smbus_read_i2c_block_data(i2c_client, 0x80, 8, &touch_data[0]);
		point_num = touch_data[0];
		if(point_num > 1)
		{
			i2c_smbus_read_i2c_block_data(i2c_client, 0x88, 8, &touch_data[8]);
		}
		if(point_num > 3)
		{
			i2c_smbus_read_i2c_block_data(i2c_client, 0x90, 8, &touch_data[16]);
		}
		for(i=1;i<=MAX_CONTACTS;i++)
		{
			if(point_num == 0)
				id_sign[i] = 0;
			id_state_flag[i] = 0;
		}
		for(i = 0; i < (point_num < MAX_FINGERS ? point_num : MAX_FINGERS); i ++)
		{
			id = touch_data[(i + 1) * 4 + 3] >> 4;
			temp_a = touch_data[(i + 1) * 4 + 3] & 0x0f;
			temp_b = touch_data[(i + 1) * 4 + 2];
			x = temp_a << 8 |temp_b;
			temp_a = touch_data[(i + 1) * 4 + 1];
			temp_b = touch_data[(i + 1) * 4 + 0];
			y = temp_a << 8 |temp_b;


		}
		for(i = 0; i < (point_num < MAX_FINGERS ? point_num : MAX_FINGERS); i ++)
			{

				x=cinfo.x[i];
				y=cinfo.y[i];
				id=cinfo.id[i];

				if(1 <= cinfo.id[i] && cinfo.id[i] <= MAX_CONTACTS)
				{
				#ifdef FILTER_POINT
					filter_point(x, y ,id);
				#else
					record_point(x, y , id);
				#endif
					tpd_down(id, x_new, y_new, 10);
					id_state_flag[id] = 1;
				}


			}
		for(i=1;i<=MAX_CONTACTS;i++)
		{
			if( (0 == point_num) || ((0 != id_state_old_flag[i]) && (0 == id_state_flag[i])) )
			{
				id_sign[i]=0;
			}
			id_state_old_flag[i] = id_state_flag[i];
		}
		if(0 == point_num)
		{
			tpd_up();
		}
		input_sync(tpd->dev);
	}
	else
	{
		printk("=======bc = %x-%x-%x-%x======= \n", buf[3], buf[2], buf[1], buf[0]);
		tpd_up();
		input_sync(tpd->dev);
		reset_chip(i2c_client);
		startup_chip(i2c_client);
	}
}

static int touch_event_handler(void *unused)
{
	struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
	sched_setscheduler(current, SCHED_RR, &param);

	do
	{
		mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
		set_current_state(TASK_INTERRUPTIBLE);
		wait_event_interruptible(waiter, tpd_flag != 0);
		tpd_flag = 0;
		TPD_DEBUG_SET_TIME;
		set_current_state(TASK_RUNNING);
		//print_info("===touch_event_handler, task running===\n");

		{
			eint_flag = 0;

#ifdef GSL_ALG_ID
           gsl_report_work();
#else
		   report_data_handle();
#endif
		}

	} while (!kthread_should_stop());

	return 0;
}

void tpd_eint_interrupt_handler(void)
{
	//print_info("===tpd irq interrupt===\n");

	eint_flag = 1;
	tpd_flag=1;
	wake_up_interruptible(&waiter);
}

static int tpd_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) {
	strcpy(info->type, TPD_DEVICE);
	return 0;
}

static int tpd_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id) {
	int err = 0;
	char buffer[5];
	int status=0;
	char check_reg[4];

	printk("==tpd_i2c_probe==\n");

	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
	mdelay(100);


            hwPowerOn(MT65XX_POWER_LDO_VGP4, VOL_2800, "TP");
            hwPowerOn(MT65XX_POWER_LDO_VGP5, VOL_1800, "TP");

	mdelay(100);

	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
#if 1 //hongz-dbg
	mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
#else
    mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
    mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_DISABLE);
#endif
	mdelay(50);


//hongzhe-dbg
gsl_read_interface(client, 0xbc, check_reg, 4);
if( check_reg[0]==0 && check_reg[1]==0 && check_reg[2]==0 && check_reg[3]==0x80 )
{

}
else
{
	printk(" Hongzhe : NO TP  \n");
	return -1;
}
//hongzhe-dbg end


	i2c_client = client;
	init_chip(i2c_client);
	msleep(20);
	check_mem_data(i2c_client);

	mt65xx_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
	mt65xx_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
	mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_TOUCH_PANEL_POLARITY, tpd_eint_interrupt_handler, 1);

	tpd_load_status = 1;
	thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	if (IS_ERR(thread)) {
		err = PTR_ERR(thread);
		TPD_DMESG(TPD_DEVICE " failed to create kernel thread: %d\n", err);
	}

#ifdef GSL_TIMER
	INIT_DELAYED_WORK(&gsl_timer_check_work, gsl_timer_check_func);
	gsl_timer_workqueue = create_workqueue("gsl_esd_check");
	queue_delayed_work(gsl_timer_workqueue, &gsl_timer_check_work, GSL_TIMER_CHECK_CIRCLE2);
#endif

#ifdef GSL_ALG_ID
	gsl_DataInit(gsl_config_data_id);
#endif
#ifdef TPD_PROC_DEBUG
        gsl_config_proc = create_proc_entry(GSL_CONFIG_PROC_FILE, 0666, NULL);
        if (gsl_config_proc == NULL)
        {
            print_info("create_proc_entry %s failed\n", GSL_CONFIG_PROC_FILE);
        }
        else
        {
            gsl_config_proc->read_proc = gsl_config_read_proc;
            gsl_config_proc->write_proc = gsl_config_write_proc;
        }
        gsl_proc_flag = 0;
#endif
	mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);

	printk("==tpd_i2c_probe end==\n");

	return 0;
}

static int tpd_i2c_remove(struct i2c_client *client)
{
	printk("==tpd_i2c_remove==\n");

	return 0;
}


static const struct i2c_device_id tpd_i2c_id[] = {{TPD_DEVICE,0},{}};
#ifdef ADD_I2C_DEVICE_ANDROID_4_0
static struct i2c_board_info __initdata gslX680_i2c_tpd={ I2C_BOARD_INFO(TPD_DEVICE, (GSLX680_ADDR))};
#else
static unsigned short force[] = {0, (GSLX680_ADDR << 1), I2C_CLIENT_END,I2C_CLIENT_END};
static const unsigned short * const forces[] = { force, NULL };
static struct i2c_client_address_data addr_data = { .forces = forces,};
#endif

struct i2c_driver tpd_i2c_driver = {
	.driver = {
		.name = TPD_DEVICE,
	#ifndef ADD_I2C_DEVICE_ANDROID_4_0
		.owner = THIS_MODULE,
	#endif
	},
	.probe = tpd_i2c_probe,
	.remove = tpd_i2c_remove,
	.id_table = tpd_i2c_id,
	.detect = tpd_i2c_detect,
	#ifndef ADD_I2C_DEVICE_ANDROID_4_0
	.address_data = &addr_data,
	#endif
};

int tpd_local_init(void)
{
	printk("==tpd_local_init==\n");

	if(i2c_add_driver(&tpd_i2c_driver)!=0) {
		TPD_DMESG("unable to add i2c driver.\n");
		return -1;
	}
	/*
	if(tpd_load_status == 0)
	{
		TPD_DMESG("add error touch panel driver.\n");
		i2c_del_driver(&tpd_i2c_driver);
		return -1;
	}
	*/
#ifdef TPD_HAVE_BUTTON
	tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
#endif

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
	TPD_DO_WARP = 1;
	memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT*4);
	memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT*4);
#endif

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
	memcpy(tpd_calmat, tpd_calmat_local, 8*4);
	memcpy(tpd_def_calmat, tpd_def_calmat_local, 8*4);
#endif
	tpd_type_cap = 1;

	printk("==tpd_local_init end==\n");
	return 0;
}

/* Function to manage low power suspend */
void tpd_suspend(struct early_suspend *h)
{
	printk("==tpd_suspend==\n");

#if 0//def TPD_KEYHOME_STANDBY_BREATH_LED_SUPPORT	
	if(0== keyhome_standby_breathled_enabled)
	{
#endif
        #ifdef TPD_PROC_DEBUG
            if(gsl_proc_flag == 1){
                return;
            }
        #endif
            
            tpd_halt = 1;
        #ifdef GSL_TIMER	
            gsl_halt_flag = 1;
        #endif
            mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
        #ifdef GSL_TIMER	
            cancel_delayed_work_sync(&gsl_timer_check_work);
            if(2==gsl_timer_flag){
                return;
            }
        #endif
            
            mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
            mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
            mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
#if 0//def TPD_KEYHOME_STANDBY_BREATH_LED_SUPPORT
	}
	else
	{
        #ifdef GSL_TIMER	
            cancel_delayed_work_sync(&gsl_timer_check_work);
            if(2==gsl_timer_flag){
                return;
            }
        #endif
	}
#endif	
	gslx680x_suspend = 1;
#ifdef TPD_KEYHOME_STANDBY_BREATH_LED_SUPPORT
#ifdef MOZ_AW2013_SUPPORT	
	aw2013_power_on_standby(1, 0);
#endif
#endif
}

/* Function to manage power-on resume */
void tpd_resume(struct early_suspend *h)
{

	printk("==tpd_resume==\n");
#if 0 //def TPD_KEYHOME_STANDBY_BREATH_LED_SUPPORT
	if(1== keyhome_standby_breathled_enabled)
	{
#ifdef MOZ_AW2013_SUPPORT		
		aw2013_power_on_standby(0, 0);
#endif

        #ifdef TPD_PROC_DEBUG
            if(gsl_proc_flag == 1){
                return;
            }
        #endif
            
            tpd_halt = 1;
        #ifdef GSL_TIMER	
            gsl_halt_flag = 1;
        #endif
            mt65xx_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
            
            mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
            mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
            mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
	     mdelay(10);
	}
#endif


#ifdef TPD_KEYHOME_STANDBY_BREATH_LED_SUPPORT
#ifdef MOZ_AW2013_SUPPORT	
	aw2013_power_on_standby(0, 0);
#endif
#endif






//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<










#ifdef TPD_PROC_DEBUG
        if(gsl_proc_flag == 1){
            return;
        }
#endif
#ifdef GSL_TIMER
	if(2==gsl_timer_flag)
	{
		gsl_halt_flag=0;
		mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
		return;
	}
#endif


	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	mdelay(30);
	reset_chip(i2c_client);
	startup_chip(i2c_client);
	msleep(20);
	check_mem_data(i2c_client);
	mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
#ifdef GSL_TIMER
	queue_delayed_work(gsl_timer_workqueue, &gsl_timer_check_work, GSL_TIMER_CHECK_CIRCLE);
	gsl_timer_flag = 0;
	gsl_halt_flag=0;	
#endif

	tpd_halt = 0;


	gslx680x_suspend = 0;
}

static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name = GSLX680_NAME,
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume,
#ifdef TPD_HAVE_BUTTON
	.tpd_have_button = 1,
#else
	.tpd_have_button = 0,
#endif
};

/* called when loaded into kernel */
static int __init tpd_driver_init(void) {
	printk("Sileadinc gslX680 touch panel driver init\n");

#ifdef ADD_I2C_DEVICE_ANDROID_4_0
	i2c_register_board_info(0, &gslX680_i2c_tpd, 1);
#endif
	if(tpd_driver_add(&tpd_device_driver) < 0)
		printk("add gslX680 driver failed\n");
	return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void) {
	printk("Sileadinc gslX680 touch panel driver exit\n");
	//input_unregister_device(tpd->dev);
	tpd_driver_remove(&tpd_device_driver);
}


#if 1 //def GSL_COMPATIBLE_CHIP
static void gsl_compatible_id(struct i2c_client *client)
{
	u8 buf[4]={0};
	#if HIGH_SPEED_I2C
	gsl_read_interface(client,0xfc,buf,4);
	#else
	i2c_smbus_write_i2c_block_data(client,0xfc,4,buf);
	#endif
	if(buf[3] == 0xa0 && buf[2] == 0x82)
	{
		gsl_compatible_flag = 0x82;
	}
	else if(buf[3] == 0xa0 && buf[2] == 0x88)
	{
		gsl_compatible_flag = 0x88;
	}
	else
	{
		msleep(20);
		#if HIGH_SPEED_I2C
		gsl_read_interface(client,0xfc,buf,4);
		#else
		i2c_smbus_write_i2c_block_data(client,0xfc,4,buf);
		#endif
		if(buf[3] == 0xa0 && buf[2] == 0x82)
	{
		gsl_compatible_flag = 0x82;
	}
	else if(buf[3] == 0xa0 && buf[2] == 0x88)
	{
		gsl_compatible_flag = 0x88;
	}
	else
	{
		msleep(20);
		#if HIGH_SPEED_I2C
		gsl_read_interface(client,0xfc,buf,4);
		#else
		i2c_smbus_write_i2c_block_data(client,0xfc,4,buf);
		#endif	
		if(buf[3] == 0xa0 && buf[2] == 0x88)
		{
			gsl_compatible_flag = 0x88;
		}
		else
		{
			gsl_compatible_flag = 0x82;
		}
	}
}

}
#endif

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);



