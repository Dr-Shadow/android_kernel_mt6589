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

#include <cust_acc.h>
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <linux/hwmsen_helper.h>

#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>

#include "bq24156a.h"

#include <oppo_chg_def.h>
#ifdef OPPO_USE_II_BQ24156A

#ifdef OPPO_HIGH_BATTERY_VOLTAGE
extern kal_uint8 oppo_high_battery_status;
extern kal_uint8 oppo_DC_charger_init_sign;
#endif
#if 0
/**********************************************************
  *
  *   [I2C Slave Setting] 
  *
  *********************************************************/
#define BQ24156A_SLAVE_ADDR_WRITE   0xD4
#define BQ24156A_SLAVE_ADDR_Read    0xD5

static struct i2c_client *new_client = NULL;
static const struct i2c_device_id bq24156a_i2c_id[] = {{"bq24156a",0},{}};   
unsigned int g_bq24156a_rdy_flag = 0;
static int bq24156a_driver_probe(struct i2c_client *client, const struct i2c_device_id *id);
extern int Enable_BATDRV_LOG;

static struct i2c_driver bq24156a_driver = {
    .driver = {
        .name    = "bq24156a",
    },
    .probe       = bq24156a_driver_probe,
    .id_table    = bq24156a_i2c_id,
};
#else
unsigned int g_bq24156a_rdy_flag = 0;
extern int Enable_BATDRV_LOG;
#endif
/**********************************************************
  *
  *   [Global Variable] 
  *
  *********************************************************/
#define BQ24156A_REG_NUM 7  
kal_uint8 bq24156a_reg[BQ24156A_REG_NUM] = {0};
#if 0
static DEFINE_MUTEX(bq24156a_i2c_access);
/**********************************************************
  *
  *   [I2C Function For Read/Write bq24156a] 
  *
  *********************************************************/
int bq24156a_read_byte(kal_uint8 cmd, kal_uint8 *returnData)
{
    char     cmd_buf[1]={0x00};
    char     readData = 0;
    int      ret=0;

    mutex_lock(&bq24156a_i2c_access);
    
    //new_client->addr = ((new_client->addr) & I2C_MASK_FLAG) | I2C_WR_FLAG;    
    new_client->ext_flag=((new_client->ext_flag ) & I2C_MASK_FLAG ) | I2C_WR_FLAG | I2C_DIRECTION_FLAG;

    cmd_buf[0] = cmd;
    ret = i2c_master_send(new_client, &cmd_buf[0], (1<<8 | 1));
    if (ret < 0) 
    {    
        //new_client->addr = new_client->addr & I2C_MASK_FLAG;
        new_client->ext_flag=0;

        mutex_unlock(&bq24156a_i2c_access);
        return 0;
    }
    
    readData = cmd_buf[0];
    *returnData = readData;

    // new_client->addr = new_client->addr & I2C_MASK_FLAG;
    new_client->ext_flag=0;
    
    mutex_unlock(&bq24156a_i2c_access);    
    return 1;
}

int bq24156a_write_byte(kal_uint8 cmd, kal_uint8 writeData)
{
    char    write_data[2] = {0};
    int     ret=0;
    
    mutex_lock(&bq24156a_i2c_access);
    
    write_data[0] = cmd;
    write_data[1] = writeData;
    
    new_client->ext_flag=((new_client->ext_flag ) & I2C_MASK_FLAG ) | I2C_DIRECTION_FLAG;
    
    ret = i2c_master_send(new_client, write_data, 2);
    if (ret < 0) 
    {
       
        new_client->ext_flag=0;
        mutex_unlock(&bq24156a_i2c_access);
        return 0;
    }
    
    new_client->ext_flag=0;
    mutex_unlock(&bq24156a_i2c_access);
    return 1;
}
#endif
/**********************************************************
  *
  *   [Read / Write Function] 
  *
  *********************************************************/
kal_uint32 bq24156a_read_interface (kal_uint8 RegNum, kal_uint8 *val, kal_uint8 MASK, kal_uint8 SHIFT)
{
    kal_uint8 bq24156a_reg = 0;
    int ret = 0;

    printk("--------------------------------------------------\n");

    ret = bq24156a_read_byte(RegNum, &bq24156a_reg);
	if (Enable_BATDRV_LOG == 1) {
    printk("[bq24156a_read_interface] Reg[%x]=0x%x\n", RegNum, bq24156a_reg);
	}
    bq24156a_reg &= (MASK << SHIFT);
    *val = (bq24156a_reg >> SHIFT);    
	if (Enable_BATDRV_LOG == 1) {
    printk("[bq24156a_read_interface] val=0x%x\n", *val);
	}
    return ret;
}

kal_uint32 bq24156a_config_interface (kal_uint8 RegNum, kal_uint8 val, kal_uint8 MASK, kal_uint8 SHIFT)
{
    kal_uint8 bq24156a_reg = 0;
    int ret = 0;

    printk("--------------------------------------------------\n");

    ret = bq24156a_read_byte(RegNum, &bq24156a_reg);
    printk("[bq24156a_config_interface] Reg[%x]=0x%x\n", RegNum, bq24156a_reg);
    
    bq24156a_reg &= ~(MASK << SHIFT);
    bq24156a_reg |= (val << SHIFT);

    ret = bq24156a_write_byte(RegNum, bq24156a_reg);
    printk("[bq24156a_config_interface] write Reg[%x]=0x%x\n", RegNum, bq24156a_reg);

    // Check
    //bq24156a_read_byte(RegNum, &bq24156a_reg);
    //printk("[bq24156a_config_interface] Check Reg[%x]=0x%x\n", RegNum, bq24156a_reg);

    return ret;
}

/**********************************************************
  *
  *   [Internal Function] 
  *
  *********************************************************/
//CON0
void bq24156a_set_tmr_rst(kal_uint32 val)
{
	kal_uint32 ret=0;	

	ret=bq24156a_config_interface(	(kal_uint8)(BQ24156A_CON0), 
									(kal_uint8)(val),
									(kal_uint8)(CON0_TMR_RST_MASK),
									(kal_uint8)(CON0_TMR_RST_SHIFT)
									);
}

kal_uint32 bq24156a_get_slrst_status(void)
{
	kal_uint32 ret=0;
	kal_uint32 val=0;

	ret=bq24156a_read_interface( 	(kal_uint8)(BQ24156A_CON0), 
									(&val),
									(kal_uint8)(CON0_SLRST_MASK),
									(kal_uint8)(CON0_SLRST_SHIFT)
									);
	return val;
}

void bq24156a_set_en_stat(kal_uint32 val)
{
	kal_uint32 ret=0;	

	ret=bq24156a_config_interface(	(kal_uint8)(BQ24156A_CON0), 
									(kal_uint8)(val),
									(kal_uint8)(CON0_EN_STAT_MASK),
									(kal_uint8)(CON0_EN_STAT_SHIFT)
									);
}

kal_uint32 bq24156a_get_chip_status(void)
{
	kal_uint32 ret=0;
	kal_uint32 val=0;

	ret=bq24156a_read_interface( 	(kal_uint8)(BQ24156A_CON0), 
									(&val),
									(kal_uint8)(CON0_STAT_MASK),
									(kal_uint8)(CON0_STAT_SHIFT)
									);
	return val;
}

kal_uint32 bq24156a_get_fault_reason(void)
{
	kal_uint32 ret=0;
	kal_uint32 val=0;

	ret=bq24156a_read_interface( 	(kal_uint8)(BQ24156A_CON0), 
									(&val),
									(kal_uint8)(CON0_FAULT_MASK),
									(kal_uint8)(CON0_FAULT_SHIFT)
									);
	return val;
}

//CON1
void bq24156a_set_lin_limit(kal_uint32 val)
{
	kal_uint32 ret=0;	

	ret=bq24156a_config_interface(	(kal_uint8)(BQ24156A_CON1), 
									(kal_uint8)(val),
									(kal_uint8)(CON1_LIN_LIMIT_MASK),
									(kal_uint8)(CON1_LIN_LIMIT_SHIFT)
									);
}

void bq24156a_set_lowv_2(kal_uint32 val)
{
	kal_uint32 ret=0;	

	ret=bq24156a_config_interface(	(kal_uint8)(BQ24156A_CON1), 
									(kal_uint8)(val),
									(kal_uint8)(CON1_LOW_V_2_MASK),
									(kal_uint8)(CON1_LOW_V_2_SHIFT)
									);
}

void bq24156a_set_lowv_1(kal_uint32 val)
{
	kal_uint32 ret=0;	

	ret=bq24156a_config_interface(	(kal_uint8)(BQ24156A_CON1), 
									(kal_uint8)(val),
									(kal_uint8)(CON1_LOW_V_1_MASK),
									(kal_uint8)(CON1_LOW_V_1_SHIFT)
									);
}

void bq24156a_set_te(kal_uint32 val)
{
	kal_uint32 ret=0;	

	ret=bq24156a_config_interface(	(kal_uint8)(BQ24156A_CON1), 
									(kal_uint8)(val),
									(kal_uint8)(CON1_TE_MASK),
									(kal_uint8)(CON1_TE_SHIFT)
									);
}

void bq24156a_set_ce(kal_uint32 val)
{
	kal_uint32 ret=0;	

	#ifdef OPPO_USE_II_BQ24157
	val = 0;//Jiada.Li@BasicDrv.CHG, 2013/04/15, debug for BQ24157'bug
	return;
	#endif

	ret=bq24156a_config_interface(	(kal_uint8)(BQ24156A_CON1), 
									(kal_uint8)(val),
									(kal_uint8)(CON1_CE_MASK),
									(kal_uint8)(CON1_CE_SHIFT)
									);
}

void bq24156a_set_hz_mode(kal_uint32 val)
{
	kal_uint32 ret=0;	

	ret=bq24156a_config_interface(	(kal_uint8)(BQ24156A_CON1), 
									(kal_uint8)(val),
									(kal_uint8)(CON1_HZ_MODE_MASK),
									(kal_uint8)(CON1_HZ_MODE_SHIFT)
									);
}

void bq24156a_set_opa_mode(kal_uint32 val)
{
	kal_uint32 ret=0;	

	ret=bq24156a_config_interface(	(kal_uint8)(BQ24156A_CON1), 
									(kal_uint8)(val),
									(kal_uint8)(CON1_OPA_MODE_MASK),
									(kal_uint8)(CON1_OPA_MODE_SHIFT)
									);
}

//CON2
void bq24156a_set_cv_vth(kal_uint32 val)
{
	kal_uint32 ret=0;	

	ret=bq24156a_config_interface(	(kal_uint8)(BQ24156A_CON2), 
									(kal_uint8)(val),
									(kal_uint8)(CON2_CV_VTH_MASK),
									(kal_uint8)(CON2_CV_VTH_SHIFT)
									);
}

//CON3
kal_uint32 bq24156a_get_vender_code(void)
{
	kal_uint32 ret=0;
	kal_uint32 val=0;

	ret=bq24156a_read_interface( 	(kal_uint8)(BQ24156A_CON3), 
									(&val),
									(kal_uint8)(CON3_VENDER_CODE_MASK),
									(kal_uint8)(CON3_VENDER_CODE_SHIFT)
									);
	return val;
}

kal_uint32 bq24156a_get_pin(void)
{
	kal_uint32 ret=0;
	kal_uint32 val=0;

	ret=bq24156a_read_interface( 	(kal_uint8)(BQ24156A_CON3), 
									(&val),
									(kal_uint8)(CON3_PIN_MASK),
									(kal_uint8)(CON3_PIN_SHIFT)
									);
	return val;
}

kal_uint32 bq24156a_get_revision(void)
{
	kal_uint32 ret=0;
	kal_uint32 val=0;

	ret=bq24156a_read_interface( 	(kal_uint8)(BQ24156A_CON3), 
									(&val),
									(kal_uint8)(CON3_REVISION_MASK),
									(kal_uint8)(CON3_REVISION_SHIFT)
									);
	return val;
}

//CON4
void bq24156a_set_reset(kal_uint32 val)
{
	kal_uint32 ret=0;	

	ret=bq24156a_config_interface(	(kal_uint8)(BQ24156A_CON4), 
									(kal_uint8)(val),
									(kal_uint8)(CON4_RESET_MASK),
									(kal_uint8)(CON4_RESET_SHIFT)
									);
}

void bq24156a_set_ac_charging_current(kal_uint32 val)
{
	kal_uint32 ret=0;	

	ret=bq24156a_config_interface(	(kal_uint8)(BQ24156A_CON4), 
									(kal_uint8)(val),
									(kal_uint8)(CON4_I_CHR_MASK),
									(kal_uint8)(CON4_I_CHR_SHIFT)
									);
}

void bq24156a_set_termination_current(kal_uint32 val)
{
	kal_uint32 ret=0;	

	ret=bq24156a_config_interface(	(kal_uint8)(BQ24156A_CON4), 
									(kal_uint8)(val),
									(kal_uint8)(CON4_I_TERM_MASK),
									(kal_uint8)(CON4_I_TERM_SHIFT)
									);
}

//CON5
void bq24156a_set_low_chg(kal_uint32 val)
{
	kal_uint32 ret=0;	

	ret=bq24156a_config_interface(	(kal_uint8)(BQ24156A_CON5), 
									(kal_uint8)(val),
									(kal_uint8)(CON5_LOW_CHG_MASK),
									(kal_uint8)(CON5_LOW_CHG_SHIFT)
									);
}

kal_uint32 bq24156a_get_dpm_status(void)
{
	kal_uint32 ret=0;
	kal_uint32 val=0;

	ret=bq24156a_read_interface( 	(kal_uint8)(BQ24156A_CON5), 
									(&val),
									(kal_uint8)(CON5_DPM_STATUS_MASK),
									(kal_uint8)(CON5_DPM_STATUS_SHIFT)
									);
	return val;
}

kal_uint32 bq24156a_get_cd_status(void)
{
	kal_uint32 ret=0;
	kal_uint32 val=0;

	ret=bq24156a_read_interface( 	(kal_uint8)(BQ24156A_CON5), 
									(&val),
									(kal_uint8)(CON5_CD_STATUS_MASK),
									(kal_uint8)(CON5_CD_STATUS_SHIFT)
									);
	return val;
}

void bq24156a_set_vsreg(kal_uint32 val)
{
	kal_uint32 ret=0;	

	ret=bq24156a_config_interface(	(kal_uint8)(BQ24156A_CON5), 
									(kal_uint8)(val),
									(kal_uint8)(CON5_VSREG_MASK),
									(kal_uint8)(CON5_VSREG_SHIFT)
									);
}

//CON6
void bq24156a_set_mchrg(kal_uint32 val)
{
	kal_uint32 ret=0;	

	ret=bq24156a_config_interface(	(kal_uint8)(BQ24156A_CON6), 
									(kal_uint8)(val),
									(kal_uint8)(CON6_MCHRG_MASK),
									(kal_uint8)(CON6_MCHRG_SHIFT)
									);
}

void bq24156a_set_mreg(kal_uint32 val)
{
	kal_uint32 ret=0;	

	ret=bq24156a_config_interface(	(kal_uint8)(BQ24156A_CON6), 
									(kal_uint8)(val),
									(kal_uint8)(CON6_MREG_MASK),
									(kal_uint8)(CON6_MREG_SHIFT)
									);
}

/**********************************************************
  *
  *   [Internal Function] 
  *
  *********************************************************/
void bq24156a_dump_register(void)
{
    int i=0;
    printk("[bq24156a] ");
    for (i=0;i<BQ24156A_REG_NUM;i++)
    {
        bq24156a_read_byte(i, &bq24156a_reg[i]);
        printk("[0x%x]=0x%x ", i, bq24156a_reg[i]);        
    }
    printk("\n");
}

#ifndef OPPO_R819
extern int g_enable_high_vbat_spec;
#endif
extern int g_pmic_cid;
void bq24156a_hw_init(void)
{    
    #ifdef OPPO_R819
	oppo_DC_charger_init_sign=0;
    #endif
	
    #if 0//def OPPO_R819
    mt_set_gpio_mode(GPIO_BQ24156A_STAT_NUM,GPIO_BQ24156A_STAT_MODE);
    mt_set_gpio_dir(GPIO_BQ24156A_STAT_NUM,GPIO_BQ24156A_STAT_DIR);
    mt_set_gpio_out(GPIO_BQ24156A_STAT_NUM,GPIO_BQ24156A_STAT_ON_OUT);

    mt_set_gpio_mode(GPIO_BQ24156A_DISABLE_NUM,GPIO_BQ24156A_DISABLE_MODE);
    mt_set_gpio_dir(GPIO_BQ24156A_DISABLE_NUM,GPIO_BQ24156A_DISABLE_DIR);
    mt_set_gpio_out(GPIO_BQ24156A_DISABLE_NUM,GPIO_BQ24156A_DISABLE_OFF_OUT);

	#ifdef OPPO_USE_CHARGER_RESET  // Nanwei.Deng@ProDrv.chager, modify 2013/05/18 for charger use Hardware Reset
    mt_set_gpio_mode(GPIO_BQ24156A_RST_NUM,GPIO_BQ24156A_RST_MODE);
    mt_set_gpio_dir(GPIO_BQ24156A_RST_NUM,GPIO_BQ24156A_RST_DIR);
    mt_set_gpio_out(GPIO_BQ24156A_RST_NUM,GPIO_BQ24156A_RST_LOW_OUT);
    msleep(5);
    mt_set_gpio_out(GPIO_BQ24156A_RST_NUM,GPIO_BQ24156A_RST_HIGH_OUT);
    #endif
    #endif
    
    bq24156a_set_mchrg(0xa); // max range = 1.55A
    #ifdef OPPO_R819
    #ifdef OPPO_HIGH_BATTERY_VOLTAGE
    if(oppo_high_battery_status == 1) 
		bq24156a_set_mreg(0x07);// max volt = 4.34V
    else
    #endif
		bq24156a_set_mreg(0x01);// max volt = 4.22V
    #endif
	
	#ifdef OPPO_R819
	oppo_DC_charger_init_sign=1;
    #endif
}
#if 0
static int bq24156a_driver_probe(struct i2c_client *client, const struct i2c_device_id *id) 
{             
    int err=0; 

    printk("[bq24156a_driver_probe] \n");

    if (!(new_client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL))) {
        err = -ENOMEM;
        goto exit;
    }    
    memset(new_client, 0, sizeof(struct i2c_client));

    new_client = client;    

    //---------------------
    #ifdef OPPO_R819
    //do not delete,or regist 0x06 will can not write???(need to check)
    bq24156a_dump_register();
    #endif
    bq24156a_hw_init();
    bq24156a_dump_register();
	g_bq24156a_rdy_flag = 1;

    return 0;                                                                                       

exit:
    return err;

}

/**********************************************************
  *
  *   [platform_driver API] 
  *
  *********************************************************/
kal_uint8 g_reg_value_bq24156a=0;
static ssize_t show_bq24156a_access(struct device *dev,struct device_attribute *attr, char *buf)
{
    printk("[show_bq24156a_access] 0x%x\n", g_reg_value_bq24156a);
    return sprintf(buf, "%u\n", g_reg_value_bq24156a);
}
static ssize_t store_bq24156a_access(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    int ret=0;
    char *pvalue = NULL;
    unsigned int reg_value = 0;
    unsigned int reg_address = 0;
    
    printk("[store_bq24156a_access] \n");
    
    if(buf != NULL && size != 0)
    {
        printk("[store_bq24156a_access] buf is %s and size is %d \n",buf,size);
        reg_address = simple_strtoul(buf,&pvalue,16);
        
        if(size > 3)
        {        
            reg_value = simple_strtoul((pvalue+1),NULL,16);        
            printk("[store_bq24156a_access] write bq24156a reg 0x%x with value 0x%x !\n",reg_address,reg_value);
            ret=bq24156a_config_interface(reg_address, reg_value, 0xFF, 0x0);
        }
        else
        {    
            ret=bq24156a_read_interface(reg_address, &g_reg_value_bq24156a, 0xFF, 0x0);
            printk("[store_bq24156a_access] read bq24156a reg 0x%x with value 0x%x !\n",reg_address,g_reg_value_bq24156a);
            printk("[store_bq24156a_access] Please use \"cat bq24156a_access\" to get value\r\n");
        }        
    }    
    return size;
}
static DEVICE_ATTR(bq24156a_access, 0664, show_bq24156a_access, store_bq24156a_access); //664

static int bq24156a_user_space_probe(struct platform_device *dev)    
{    
    int ret_device_file = 0;

    printk("******** bq24156a_user_space_probe!! ********\n" );
    
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_bq24156a_access);
    
    return 0;
}

struct platform_device bq24156a_user_space_device = {
    .name   = "bq24156a-user",
    .id     = -1,
};

static struct platform_driver bq24156a_user_space_driver = {
    .probe      = bq24156a_user_space_probe,
    .driver     = {
        .name = "bq24156a-user",
    },
};

#define bq24156a_BUSNUM 6
static struct i2c_board_info __initdata i2c_bq24156a = { I2C_BOARD_INFO("bq24156a", (0xd4>>1))};

static int __init bq24156a_init(void)
{    
    int ret=0;
    
    printk("[bq24156a_init] init start\n");
    
    i2c_register_board_info(bq24156a_BUSNUM, &i2c_bq24156a, 1);

    if(i2c_add_driver(&bq24156a_driver)!=0)
    {
        printk("[bq24156a_init] failed to register bq24156a i2c driver.\n");
    }
    else
    {
        printk("[bq24156a_init] Success to register bq24156a i2c driver.\n");
    }

    // bq24156a user space access interface
    ret = platform_device_register(&bq24156a_user_space_device);
    if (ret) {
        printk("****[bq24156a_init] Unable to device register(%d)\n", ret);
        return ret;
    }    
    ret = platform_driver_register(&bq24156a_user_space_driver);
    if (ret) {
        printk("****[bq24156a_init] Unable to register driver (%d)\n", ret);
        return ret;
    }
    
    return 0;        
}

static void __exit bq24156a_exit(void)
{
    i2c_del_driver(&bq24156a_driver);
}

module_init(bq24156a_init);
module_exit(bq24156a_exit);
   
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C BQ24156A Driver");
MODULE_AUTHOR("James Lo<james.lo@mediatek.com>");
#endif
#endif
