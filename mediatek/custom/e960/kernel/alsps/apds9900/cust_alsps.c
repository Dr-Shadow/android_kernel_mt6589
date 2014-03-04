#include <linux/types.h>
#include <mach/mt_pm_ldo.h>
#include <cust_alsps.h>
//#include <mach/mt6577_pm_ldo.h>

static struct alsps_hw cust_alsps_hw = {
    .i2c_num    = 3,
	.polling_mode_ps =1, /* 0 - hongzhe */
	.polling_mode_als =1,
    .power_id   = MT65XX_POWER_NONE,    /*LDO is not used*/
    .power_vol  = VOL_DEFAULT,          /*LDO is not used*/
    .i2c_addr   = {0x72, 0x48, 0x78, 0x00},
    /*Lenovo-sw chenlj2 add 2011-06-03,modify parameter below two lines*/
    //.als_level  = { 4, 40,  80,   120,   160, 250,  400, 800, 1200,  1600, 2000, 3000, 5000, 10000, 65535},
    //.als_value  = {10, 20,20,  120, 120, 280,  280,  280, 1600,  1600,  1600,  6000,  6000, 9000,  10240, 10240},
   // .als_level  = { 5, 10,  25,   50,  100, 150,  200, 400, 1000,  1500, 2000, 3000, 5000, 8000, 10000},
   // .als_value  = {10, 50,  100,  150, 200, 250,  280,  280, 1600,  1600,  1600,  6000,  6000, 9000,  10240, 10240},
    /* MTK: modified to support AAL */
    //.als_level  = { 6, 9, 17, 38, 56, 74, 116, 342, 778, 1082,  1386, 1914, 3000, 5000, 8000 },
    //.als_value  = {136, 218, 312, 730, 1065, 1400, 2250, 4286, 5745, 7390, 9034, 11000, 12250, 12250, 12250, 12250},


  // .als_level  = { 0,  1,  1,   2,  3,  3,  20, 200, 400,  600,  1200, 2000, 2800, 3800, 4096}, /* orig value */
  //  .als_value  = {40, 40, 90,  90, 160, 160,  225,  320,  640,  1280,  1280,  2600,  2600, 2600,  10240, 10240}, /* convert value */

    .als_level  = { 2, 20,  50,   120,  200, 400,  800, 1000, 1200,  1500, 2000, 3000, 5000, 8000, 10000},
   .als_value  = {10, 80,  150,  150, 400, 400,  800,  800, 1600,  1600,  1600,  6000,  6000, 9000,  10240, 10240},
#if 1
    .ps_threshold_high = 450, /*200*/
    .ps_threshold_low = 250, /* 100 */
#else
    .ps_threshold_high = 130,
    .ps_threshold_low = 110, /* 100 */
#endif    
    .ps_threshold = 900,
};
struct alsps_hw *get_cust_alsps_hw(void) {
    return &cust_alsps_hw;
}
int APDS9900_CMM_PPCOUNT_VALUE =0x02;// 0x02;//0x08;
int APDS9900_CMM_CONTROL_VALUE = 0x24;//0xE4;//0x24; //0x22; //0xE4;
int ZOOM_TIME = 4;
