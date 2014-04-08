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

#include "bq24157.h"

#include <oppo_chg_def.h>
#ifdef OPPO_USE_II_BQ24157


kal_uint32 bq24157_config_interface (kal_uint8 RegNum, kal_uint8 val, kal_uint8 MASK, kal_uint8 SHIFT)
{
    kal_uint8 bq24157_reg = 0;
    int ret = 0;

    printk("--------------------------------------------------\n");

    ret = bq24157_read_byte(RegNum, &bq24157_reg);
    printk("[bq24157_config_interface] Reg[%x]=0x%x\n", RegNum, bq24157_reg);
    
    bq24157_reg &= ~(MASK << SHIFT);
    bq24157_reg |= (val << SHIFT);

    ret = bq24157_write_byte(RegNum, bq24157_reg);
    printk("[bq24157_config_interface] write Reg[%x]=0x%x\n", RegNum, bq24157_reg);

    // Check
    //bq24157_read_byte(RegNum, &bq24157_reg);
    //printk("[bq24157_config_interface] Check Reg[%x]=0x%x\n", RegNum, bq24157_reg);

    return ret;
}

void bq24157_set_ac_charging_current(kal_uint32 val)
{
	kal_uint32 ret=0;	

	ret=bq24157_config_interface(	(kal_uint8)(BQ24157_CON4), 
									(kal_uint8)(val),
									(kal_uint8)(CON4_I_CHR_MASK),
									(kal_uint8)(CON4_I_CHR_SHIFT)
									);
}
#endif
