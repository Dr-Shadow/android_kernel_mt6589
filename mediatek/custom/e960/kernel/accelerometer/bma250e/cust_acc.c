#include <linux/types.h>
#include <cust_acc.h>
#include <mach/mt_pm_ldo.h>
/***************************************************************************
ABOUT Direction: x, y, z -> 
    0 :    x, y, z        
    1 :    -y, x, z
    2 :    -x, -y, z
    3 :    y, -x, z
    4 :    -x, y, -z
    5 :    y, x, -z
    6 :    x, -y, -z
    7 :    -y, -x, -z
****************************************************************************/

/*---------------------------------------------------------------------------*/
static struct acc_hw cust_acc_hw = {
    .i2c_num = 3,
    .direction = 5,
    .power_id = MT65XX_POWER_NONE,  /*!< LDO is not used */
    .power_vol= VOL_DEFAULT,        /*!< LDO is not used */
    .firlen = 0, //old value 16                /*!< don't enable low pass fileter */
};
/*---------------------------------------------------------------------------*/
struct acc_hw* get_cust_acc_hw(void) 
{
    return &cust_acc_hw;
}


