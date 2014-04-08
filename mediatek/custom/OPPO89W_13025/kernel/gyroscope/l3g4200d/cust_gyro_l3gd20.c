
#include <linux/types.h>
#include <cust_gyro.h>
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
		hwPowerOn(MT65XX_POWER_LDO_VMCH1,VOL_3000,"gyro_3000");
		msleep(1);
		hwPowerOn(MT65XX_POWER_LDO_VGP5,VOL_1800,"gyro_1800");
		msleep(1);
	}
	else
	{
		hwPowerDown(MT65XX_POWER_LDO_VGP5,"gyro_1800");
		msleep(1);
		hwPowerDown(MT65XX_POWER_LDO_VMCH1,"gyro_3000");
	}

	status = on;
	return 0;
}

/*---------------------------------------------------------------------------*/
static struct gyro_hw cust_gyro_hw = 
{
    .i2c_num = 2,
    .direction = 3,
    .power_id = MT65XX_POWER_NONE,  /*!< LDO is not used */
    .power_vol= VOL_DEFAULT,        /*!< LDO is not used */
    .firlen = 0, 					/*!< don't enable low pass fileter */
	.power = power,
};

/*---------------------------------------------------------------------------*/
struct gyro_hw* get_cust_gyro_hw_l3gd20(void) 
{
    return &cust_gyro_hw;
}

