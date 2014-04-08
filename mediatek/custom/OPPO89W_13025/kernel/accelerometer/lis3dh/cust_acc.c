
#include <linux/types.h>
#include <cust_acc.h>
#include <mach/mt_pm_ldo.h>
#include <linux/delay.h>

static int power(struct gyro_hw *hw, unsigned int on, char *devname)
{
	static unsigned int status = 0;
	if (on == status)
	{
		return 0;
	}
	if (on)
	{
		hwPowerOn(MT65XX_POWER_LDO_VMCH1,VOL_3000,"gsensor_3000");
		msleep(1);
		hwPowerOn(MT65XX_POWER_LDO_VGP5,VOL_1800,"gsensor_1800");
		msleep(1);
	}
	else
	{
		hwPowerDown(MT65XX_POWER_LDO_VGP5,"gsensor_1800");
		msleep(1);
		hwPowerDown(MT65XX_POWER_LDO_VMCH1,"gsensor_3000");
	}

	status = on;
	return 0;
}

static struct acc_hw cust_acc_hw = {
    .i2c_num = 2,
    .direction =7,
    .power_id = MT65XX_POWER_NONE,  /*!< LDO is not used */
    .power_vol= VOL_DEFAULT,        /*!< LDO is not used */
    .firlen = 0, //old value 16                  /*!< don't enable low pass fileter */

	.power = power,
};
/*---------------------------------------------------------------------------*/
struct acc_hw* get_cust_acc_hw_lis3dh(void) 
{
    return &cust_acc_hw;
}
