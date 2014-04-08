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

#include "excharger.h"

#include <oppo_chg_def.h>
#ifdef OPPO_USE_EXCHARGER
extern kal_uint8 oppo_excharger_id;
/**********************************************************
  *
  *   [I2C Slave Setting] 
  *
  *********************************************************/
#define EXCHARGER_SLAVE_ADDR_WRITE   0xD4
#define EXCHARGER_SLAVE_ADDR_Read    0xD5

static struct i2c_client *new_client = NULL;
static const struct i2c_device_id excharger_i2c_id[] = {{"excharger",0},{}};   
unsigned int g_excharger_rdy_flag = 0;
static int excharger_driver_probe(struct i2c_client *client, const struct i2c_device_id *id);
extern int Enable_BATDRV_LOG;

static struct i2c_driver excharger_driver = {
    .driver = {
        .name    = "excharger",
    },
    .probe       = excharger_driver_probe,
    .id_table    = excharger_i2c_id,
};

/**********************************************************
  *
  *   [Global Variable] 
  *
  *********************************************************/
#define EXCHARGER_REG_NUM 7  
kal_uint8 excharger_reg[EXCHARGER_REG_NUM] = {0};

static DEFINE_MUTEX(excharger_i2c_access);
/**********************************************************
  *
  *   [I2C Function For Read/Write excharger] 
  *
  *********************************************************/
int excharger_read_byte(kal_uint8 cmd, kal_uint8 *returnData)
{
    char     cmd_buf[1]={0x00};
    char     readData = 0;
    int      ret=0;

    mutex_lock(&excharger_i2c_access);
    
    //new_client->addr = ((new_client->addr) & I2C_MASK_FLAG) | I2C_WR_FLAG;    
    new_client->ext_flag=((new_client->ext_flag ) & I2C_MASK_FLAG ) | I2C_WR_FLAG | I2C_DIRECTION_FLAG;

    cmd_buf[0] = cmd;
    ret = i2c_master_send(new_client, &cmd_buf[0], (1<<8 | 1));
    if (ret < 0) 
    {    
        //new_client->addr = new_client->addr & I2C_MASK_FLAG;
        new_client->ext_flag=0;

        mutex_unlock(&excharger_i2c_access);
        return 0;
    }
    
    readData = cmd_buf[0];
    *returnData = readData;

    // new_client->addr = new_client->addr & I2C_MASK_FLAG;
    new_client->ext_flag=0;
    
    mutex_unlock(&excharger_i2c_access);    
    return 1;
}

int excharger_write_byte(kal_uint8 cmd, kal_uint8 writeData)
{
    char    write_data[2] = {0};
    int     ret=0;
    
    mutex_lock(&excharger_i2c_access);
    
    write_data[0] = cmd;
    write_data[1] = writeData;
    
    new_client->ext_flag=((new_client->ext_flag ) & I2C_MASK_FLAG ) | I2C_DIRECTION_FLAG;
    
    ret = i2c_master_send(new_client, write_data, 2);
    if (ret < 0) 
    {
       
        new_client->ext_flag=0;
        mutex_unlock(&excharger_i2c_access);
        return 0;
    }
    
    new_client->ext_flag=0;
    mutex_unlock(&excharger_i2c_access);
    return 1;
}

/**********************************************************
  *
  *   [Read / Write Function] 
  *
  *********************************************************/
kal_uint32 excharger_read_interface (kal_uint8 RegNum, kal_uint8 *val, kal_uint8 MASK, kal_uint8 SHIFT)
{
    kal_uint8 excharger_reg = 0;
    int ret = 0;

    printk("--------------------------------------------------\n");

    ret = excharger_read_byte(RegNum, &excharger_reg);
	if (Enable_BATDRV_LOG == 1) {
    printk("[excharger_read_interface] Reg[%x]=0x%x\n", RegNum, excharger_reg);
	}
    excharger_reg &= (MASK << SHIFT);
    *val = (excharger_reg >> SHIFT);    
	if (Enable_BATDRV_LOG == 1) {
    printk("[excharger_read_interface] val=0x%x\n", *val);
	}
    return ret;
}

kal_uint32 excharger_config_interface (kal_uint8 RegNum, kal_uint8 val, kal_uint8 MASK, kal_uint8 SHIFT)
{
    kal_uint8 excharger_reg = 0;
    int ret = 0;

    printk("--------------------------------------------------\n");

    ret = excharger_read_byte(RegNum, &excharger_reg);
    printk("[excharger_config_interface] Reg[%x]=0x%x\n", RegNum, excharger_reg);
    
    excharger_reg &= ~(MASK << SHIFT);
    excharger_reg |= (val << SHIFT);

    ret = excharger_write_byte(RegNum, excharger_reg);
    printk("[excharger_config_interface] write Reg[%x]=0x%x\n", RegNum, excharger_reg);

    // Check
    //excharger_read_byte(RegNum, &excharger_reg);
    //printk("[excharger_config_interface] Check Reg[%x]=0x%x\n", RegNum, excharger_reg);

    return ret;
}

//write one register directly
kal_uint32 excharger_config_interface_liao (kal_uint8 RegNum, kal_uint8 val)
{   
    int ret = 0;
    
    ret = excharger_write_byte(RegNum, val);

    return ret;
}
#if 0
/**********************************************************
  *
  *   [Internal Function] 
  *
  *********************************************************/
//CON0----------------------------------------------------

void excharger_set_tmr_rst(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=excharger_config_interface(   (kal_uint8)(EXCHARGER_CON0), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON0_TMR_RST_MASK),
                                    (kal_uint8)(CON0_TMR_RST_SHIFT)
                                    );
}

kal_uint32 excharger_get_otg_status(void)
{
    kal_uint32 ret=0;
    kal_uint8 val=0;

    ret=excharger_read_interface(     (kal_uint8)(EXCHARGER_CON0), 
                                    (&val),
                                    (kal_uint8)(CON0_OTG_MASK),
                                    (kal_uint8)(CON0_OTG_SHIFT)
                                    );
    return val;
}

void excharger_set_en_stat(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=excharger_config_interface(   (kal_uint8)(EXCHARGER_CON0), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON0_EN_STAT_MASK),
                                    (kal_uint8)(CON0_EN_STAT_SHIFT)
                                    );
}

kal_uint32 excharger_get_chip_status(void)
{
    kal_uint32 ret=0;
    kal_uint8 val=0;

    ret=excharger_read_interface(     (kal_uint8)(EXCHARGER_CON0), 
                                    (&val),
                                    (kal_uint8)(CON0_STAT_MASK),
                                    (kal_uint8)(CON0_STAT_SHIFT)
                                    );
    return val;
}

kal_uint32 excharger_get_boost_status(void)
{
    kal_uint32 ret=0;
    kal_uint8 val=0;

    ret=excharger_read_interface(     (kal_uint8)(EXCHARGER_CON0), 
                                    (&val),
                                    (kal_uint8)(CON0_BOOST_MASK),
                                    (kal_uint8)(CON0_BOOST_SHIFT)
                                    );
    return val;
}

kal_uint32 excharger_get_fault_status(void)
{
    kal_uint32 ret=0;
    kal_uint8 val=0;

    ret=excharger_read_interface(     (kal_uint8)(EXCHARGER_CON0), 
                                    (&val),
                                    (kal_uint8)(CON0_FAULT_MASK),
                                    (kal_uint8)(CON0_FAULT_SHIFT)
                                    );
    return val;
}

//CON1----------------------------------------------------

void excharger_set_input_charging_current(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=excharger_config_interface(   (kal_uint8)(EXCHARGER_CON1), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON1_LIN_LIMIT_MASK),
                                    (kal_uint8)(CON1_LIN_LIMIT_SHIFT)
                                    );
}

void excharger_set_v_low(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=excharger_config_interface(   (kal_uint8)(EXCHARGER_CON1), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON1_LOW_V_MASK),
                                    (kal_uint8)(CON1_LOW_V_SHIFT)
                                    );
}

void excharger_set_te(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=excharger_config_interface(   (kal_uint8)(EXCHARGER_CON1), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON1_TE_MASK),
                                    (kal_uint8)(CON1_TE_SHIFT)
                                    );
}

void excharger_set_ce(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=excharger_config_interface(   (kal_uint8)(EXCHARGER_CON1), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON1_CE_MASK),
                                    (kal_uint8)(CON1_CE_SHIFT)
                                    );
}

void excharger_set_hz_mode(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=excharger_config_interface(   (kal_uint8)(EXCHARGER_CON1), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON1_HZ_MODE_MASK),
                                    (kal_uint8)(CON1_HZ_MODE_SHIFT)
                                    );
}

void excharger_set_opa_mode(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=excharger_config_interface(   (kal_uint8)(EXCHARGER_CON1), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON1_OPA_MODE_MASK),
                                    (kal_uint8)(CON1_OPA_MODE_SHIFT)
                                    );
}

//CON2----------------------------------------------------

void excharger_set_oreg(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=excharger_config_interface(   (kal_uint8)(EXCHARGER_CON2), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON2_OREG_MASK),
                                    (kal_uint8)(CON2_OREG_SHIFT)
                                    );
}

void excharger_set_otg_pl(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=excharger_config_interface(   (kal_uint8)(EXCHARGER_CON2), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON2_OTG_PL_MASK),
                                    (kal_uint8)(CON2_OTG_PL_SHIFT)
                                    );
}

void excharger_set_otg_en(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=excharger_config_interface(   (kal_uint8)(EXCHARGER_CON2), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON2_OTG_EN_MASK),
                                    (kal_uint8)(CON2_OTG_EN_SHIFT)
                                    );
}

//CON3----------------------------------------------------

kal_uint32 excharger_get_vender_code(void)
{
    kal_uint32 ret=0;
    kal_uint8 val=0;

    ret=excharger_read_interface(     (kal_uint8)(EXCHARGER_CON3), 
                                    (&val),
                                    (kal_uint8)(CON3_VENDER_CODE_MASK),
                                    (kal_uint8)(CON3_VENDER_CODE_SHIFT)
                                    );
    return val;
}

kal_uint32 excharger_get_pn(void)
{
    kal_uint32 ret=0;
    kal_uint8 val=0;

    ret=excharger_read_interface(     (kal_uint8)(EXCHARGER_CON3), 
                                    (&val),
                                    (kal_uint8)(CON3_PIN_MASK),
                                    (kal_uint8)(CON3_PIN_SHIFT)
                                    );
    return val;
}

kal_uint32 excharger_get_revision(void)
{
    kal_uint32 ret=0;
    kal_uint8 val=0;

    ret=excharger_read_interface(     (kal_uint8)(EXCHARGER_CON3), 
                                    (&val),
                                    (kal_uint8)(CON3_REVISION_MASK),
                                    (kal_uint8)(CON3_REVISION_SHIFT)
                                    );
    return val;
}

//CON4----------------------------------------------------

void excharger_set_reset(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=excharger_config_interface(   (kal_uint8)(EXCHARGER_CON4), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON4_RESET_MASK),
                                    (kal_uint8)(CON4_RESET_SHIFT)
                                    );
}

void excharger_set_iocharge(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=excharger_config_interface(   (kal_uint8)(EXCHARGER_CON4), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON4_I_CHR_MASK),
                                    (kal_uint8)(CON4_I_CHR_SHIFT)
                                    );
}

void excharger_set_iterm(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=excharger_config_interface(   (kal_uint8)(EXCHARGER_CON4), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON4_I_TERM_MASK),
                                    (kal_uint8)(CON4_I_TERM_SHIFT)
                                    );
}

//CON5----------------------------------------------------

void excharger_set_dis_vreg(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=excharger_config_interface(   (kal_uint8)(EXCHARGER_CON5), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON5_DIS_VREG_MASK),
                                    (kal_uint8)(CON5_DIS_VREG_SHIFT)
                                    );
}

void excharger_set_io_level(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=excharger_config_interface(   (kal_uint8)(EXCHARGER_CON5), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON5_IO_LEVEL_MASK),
                                    (kal_uint8)(CON5_IO_LEVEL_SHIFT)
                                    );
}

kal_uint32 excharger_get_sp_status(void)
{
    kal_uint32 ret=0;
    kal_uint8 val=0;

    ret=excharger_read_interface(     (kal_uint8)(EXCHARGER_CON5), 
                                    (&val),
                                    (kal_uint8)(CON5_SP_STATUS_MASK),
                                    (kal_uint8)(CON5_SP_STATUS_SHIFT)
                                    );
    return val;
}

kal_uint32 excharger_get_en_level(void)
{
    kal_uint32 ret=0;
    kal_uint8 val=0;

    ret=excharger_read_interface(     (kal_uint8)(EXCHARGER_CON5), 
                                    (&val),
                                    (kal_uint8)(CON5_EN_LEVEL_MASK),
                                    (kal_uint8)(CON5_EN_LEVEL_SHIFT)
                                    );
    return val;
}

void excharger_set_vsp(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=excharger_config_interface(   (kal_uint8)(EXCHARGER_CON5), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON5_VSP_MASK),
                                    (kal_uint8)(CON5_VSP_SHIFT)
                                    );
}

//CON6----------------------------------------------------

void excharger_set_i_safe(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=excharger_config_interface(   (kal_uint8)(EXCHARGER_CON6), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON6_ISAFE_MASK),
                                    (kal_uint8)(CON6_ISAFE_SHIFT)
                                    );
}

void excharger_set_v_safe(kal_uint32 val)
{
    kal_uint32 ret=0;    

    ret=excharger_config_interface(   (kal_uint8)(EXCHARGER_CON6), 
                                    (kal_uint8)(val),
                                    (kal_uint8)(CON6_VSAFE_MASK),
                                    (kal_uint8)(CON6_VSAFE_SHIFT)
                                    );
}
#endif
/**********************************************************
  *
  *   [Internal Function] 
  *
  *********************************************************/
void excharger_dump_register(void)
{
    int i=0;
    printk("[excharger] ");
    for (i=0;i<EXCHARGER_REG_NUM;i++)
    {
        excharger_read_byte(i, &excharger_reg[i]);
        printk("[0x%x]=0x%x ", i, excharger_reg[i]);        
    }
    printk("\n");
}
#if 0
#ifndef VENDOR_EDIT
extern int g_enable_high_vbat_spec;
#endif
extern int g_pmic_cid;
#ifdef VENDOR_EDIT
#define GPIO_EXCHARGER_STAT_NUM           GPIOEXT26
#define GPIO_EXCHARGER_STAT_MODE          GPIO_MODE_GPIO
#define GPIO_EXCHARGER_STAT_DIR           GPIO_DIR_OUT
#define GPIO_EXCHARGER_STAT_OFF_OUT       GPIO_OUT_ONE
#define GPIO_EXCHARGER_STAT_ON_OUT        GPIO_OUT_ZERO

#define GPIO_EXCHARGER_DISABLE_NUM        GPIOEXT20
#define GPIO_EXCHARGER_DISABLE_MODE       GPIO_MODE_GPIO
#define GPIO_EXCHARGER_DISABLE_DIR        GPIO_DIR_OUT
#define GPIO_EXCHARGER_DISABLE_OFF_OUT    GPIO_OUT_ONE
#define GPIO_EXCHARGER_DISABLE_ON_OUT     GPIO_OUT_ZERO
#endif
void excharger_hw_init(void)
{    
    #ifdef VENDOR_EDIT
    mt_set_gpio_mode(GPIO_EXCHARGER_STAT_NUM,GPIO_EXCHARGER_STAT_MODE);
    mt_set_gpio_dir(GPIO_EXCHARGER_STAT_NUM,GPIO_EXCHARGER_STAT_DIR);
    mt_set_gpio_out(GPIO_EXCHARGER_STAT_NUM,GPIO_EXCHARGER_STAT_ON_OUT);

    mt_set_gpio_mode(GPIO_EXCHARGER_DISABLE_NUM,GPIO_EXCHARGER_DISABLE_MODE);
    mt_set_gpio_dir(GPIO_EXCHARGER_DISABLE_NUM,GPIO_EXCHARGER_DISABLE_DIR);
    mt_set_gpio_out(GPIO_EXCHARGER_DISABLE_NUM,GPIO_EXCHARGER_DISABLE_ON_OUT);
    #endif
    
    #ifndef VENDOR_EDIT
    if(g_enable_high_vbat_spec == 1)
    {
        if(g_pmic_cid == 0x1020)
        {
            printk("[excharger_hw_init] (0x06,0x70) because 0x1020\n");
            excharger_config_interface_liao(0x06,0x70); // set ISAFE
        }
        else
        {
            printk("[excharger_hw_init] (0x06,0x77)\n");
            excharger_config_interface_liao(0x06,0x77); // set ISAFE and HW CV point (4.34)
        }
    }
    else
    {
        printk("[excharger_hw_init] (0x06,0x70) \n");
        excharger_config_interface_liao(0x06,0x70); // set ISAFE
    }
    #else
    printk("[excharger_hw_init] (0x06,0x71) \n");
    excharger_config_interface_liao(0x06,0x71); // set ISAFE
    #endif
}
#endif
extern void OPPO_CHG_Para_Init(void);
static int excharger_driver_probe(struct i2c_client *client, const struct i2c_device_id *id) 
{             
    int err=0; 
    kal_uint8 excharger_id;

    printk("[excharger_driver_probe] \n");

    if (!(new_client = kmalloc(sizeof(struct i2c_client), GFP_KERNEL))) {
        err = -ENOMEM;
        goto exit;
    }    
    memset(new_client, 0, sizeof(struct i2c_client));

    new_client = client;    

    //---------------------
    #ifdef VENDOR_EDIT
    //do not delete,or regist 0x06 will can not write???(need to check)
    excharger_dump_register();
    #endif
     
    excharger_id = excharger_reg[VENDER_CODE_PN_CON];
    excharger_id &= (VENDER_CODE_PN_MASK << VENDER_CODE_PN_SHIFT);
    excharger_id = (excharger_id >> VENDER_CODE_PN_SHIFT); 
	printk("dengnanwei_oppo_charging_DEVICE22= %d\r\n", excharger_id);
	
    if(FAN5405_VENDER_CODE == excharger_id)
    {
        oppo_excharger_id = 0;//FAN5405_ID;
        fan5405_hw_init();
        fan5405_dump_register();
    }
    else if(BQ24156A_VENDER_CODE == excharger_id)
    {
        oppo_excharger_id = 1;//BQ24156A_ID;
        bq24156a_hw_init();
        bq24156a_dump_register();
    }
    else
    {
        oppo_excharger_id = 2;//BQ24157_ID;
        bq24156a_hw_init();
        bq24156a_dump_register();
    }
    OPPO_CHG_Para_Init();
    
	g_excharger_rdy_flag = 1;

    return 0;                                                                                       

exit:
    return err;

}

/**********************************************************
  *
  *   [platform_driver API] 
  *
  *********************************************************/
kal_uint8 g_reg_value_excharger=0;
static ssize_t show_excharger_access(struct device *dev,struct device_attribute *attr, char *buf)
{
    printk("[show_excharger_access] 0x%x\n", g_reg_value_excharger);
    return sprintf(buf, "%u\n", g_reg_value_excharger);
}
static ssize_t store_excharger_access(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    int ret=0;
    char *pvalue = NULL;
    unsigned int reg_value = 0;
    unsigned int reg_address = 0;
    
    printk("[store_excharger_access] \n");
    
    if(buf != NULL && size != 0)
    {
        printk("[store_excharger_access] buf is %s and size is %d \n",buf,size);
        reg_address = simple_strtoul(buf,&pvalue,16);
        
        if(size > 3)
        {        
            reg_value = simple_strtoul((pvalue+1),NULL,16);        
            printk("[store_excharger_access] write excharger reg 0x%x with value 0x%x !\n",reg_address,reg_value);
            ret=excharger_config_interface(reg_address, reg_value, 0xFF, 0x0);
        }
        else
        {    
            ret=excharger_read_interface(reg_address, &g_reg_value_excharger, 0xFF, 0x0);
            printk("[store_excharger_access] read excharger reg 0x%x with value 0x%x !\n",reg_address,g_reg_value_excharger);
            printk("[store_excharger_access] Please use \"cat excharger_access\" to get value\r\n");
        }        
    }    
    return size;
}
static DEVICE_ATTR(excharger_access, 0664, show_excharger_access, store_excharger_access); //664

static int excharger_user_space_probe(struct platform_device *dev)    
{    
    int ret_device_file = 0;

    printk("******** excharger_user_space_probe!! ********\n" );
    
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_excharger_access);
    
    return 0;
}

struct platform_device excharger_user_space_device = {
    .name   = "excharger-user",
    .id     = -1,
};

static struct platform_driver excharger_user_space_driver = {
    .probe      = excharger_user_space_probe,
    .driver     = {
        .name = "excharger-user",
    },
};

#define EXCHARGER_BUSNUM 6
static struct i2c_board_info __initdata i2c_excharger = { I2C_BOARD_INFO("excharger", (0xd4>>1))};

static int __init excharger_init(void)
{    
    int ret=0;
    
    printk("[excharger_init] init start\n");
    
    i2c_register_board_info(EXCHARGER_BUSNUM, &i2c_excharger, 1);

    if(i2c_add_driver(&excharger_driver)!=0)
    {
        printk("[excharger_init] failed to register excharger i2c driver.\n");
    }
    else
    {
        printk("[excharger_init] Success to register excharger i2c driver.\n");
    }

    // excharger user space access interface
    ret = platform_device_register(&excharger_user_space_device);
    if (ret) {
        printk("****[excharger_init] Unable to device register(%d)\n", ret);
        return ret;
    }    
    ret = platform_driver_register(&excharger_user_space_driver);
    if (ret) {
        printk("****[excharger_init] Unable to register driver (%d)\n", ret);
        return ret;
    }
    
    return 0;        
}

static void __exit excharger_exit(void)
{
    i2c_del_driver(&excharger_driver);
}

module_init(excharger_init);
module_exit(excharger_exit);
   
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C excharger Driver");
MODULE_AUTHOR("Jida Li<ljd@oppo.com>");
#endif
