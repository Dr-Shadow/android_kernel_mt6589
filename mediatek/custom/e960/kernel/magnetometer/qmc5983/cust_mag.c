#include <linux/types.h>
#include <mach/mt_pm_ldo.h>
#include <cust_mag.h>

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

static struct mag_hw cust_mag_hw = {
    .i2c_num = 3,
#ifdef PROJECT_E960_PCB_V2		
    .direction =4,
#else
    .direction =7, // PROJECT_E960_PCB_V3
#endif
    .power_id = MT65XX_POWER_NONE,  /*!< LDO is not used */
    .power_vol= VOL_DEFAULT,        /*!< LDO is not used */
};
struct mag_hw* get_cust_mag_hw(void) 
{
    return &cust_mag_hw;
}
