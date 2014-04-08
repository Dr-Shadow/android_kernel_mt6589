#ifndef _OPPO_BATTERY_H_
#define _OPPO_BATTERY_H_


#ifdef OPPO_NEW_CHARGING
#define OPPO_KERNEL_CHARGING
//#define OPPO_UBOOT_CHARGING

#define OPPO_CHARGE_CURRENT_ADAPTIVE
#define OPPO_NEW_CHARGE_POSTFULL
#define OPPO_CHARGE_CURRENT_BY_LCD
#define OPPO_NEW_BAT_VOL_ALG
//#define OPPO_NEW_FULL_STOP
#ifdef OPPO_KERNEL_CHARGING
#define OPPO_CALL_MODE_CHARGING_PAUSE
#define OPPO_FACTORY_TEST
#endif

#ifdef OPPO_KERNEL_CHARGING
#define oppo_chg_lcd_is_on oppo_lcd_status
#define oppo_chg_log printk
#define oppo_chg_sleep_ms(a) msleep(a)
#elif defined OPPO_UBOOT_CHARGING
#define oppo_chg_lcd_is_on g_bl_on
#define oppo_chg_log printf
#define oppo_chg_sleep_ms(a)    if (g_bl_on == KAL_TRUE)        \
                                {                               \
                                mtk_sleep(a, KAL_FALSE);     \
                                }                               \
                                else                            \
                                {                               \
                                mtk_sleep(a, KAL_TRUE);      \
                                }  
#endif

typedef struct{	
	unsigned int BattVolt;
	unsigned int BattPercent;
}VBAT_TO_PERCENT;

/* Battery Voltage and Percentage Mapping Table */
VBAT_TO_PERCENT Batt_VoltToPercent_Table[] = {
	/*BattVolt,BattPercent*/
	{3400,0},
	{3500,1},
	{3628,5},
	{3650,9},
	{3675,13},
	{3695,17},
	{3711,21},
	{3723,25},
	{3737,29},
	{3750,33},
	{3762,37},
	{3777,41},
	{3791,45},
	{3806,49},
	{3823,53},
	{3842,57},
	{3864,61},
	{3889,65},
	{3921,69},
	{3954,73},
	{3992,77},
	{4033,81},
	{4071,85},
	{4111,89},
	{4154,93},
	{4196,97},
	{4208,98},
	{4218,99},
	{4230,100},
};

#define OPPO_BAT_VOLT_PERCENT100        4230

#ifdef OPPO_USE_EXCHARGER
#define R_CURRENT_SENSE 1				// 0.1 Ohm
#else
#define R_CURRENT_SENSE 2				// 0.2 Ohm
#endif
/*****************************************************************************
 * Charging Notify
 ****************************************************************************/
#define NOTIFY_OVER_TIME                1
#define NOTIFY_LOW_VCHARGER             2
#define NOTIFY_OVER_VCHARGER            3
#define NOTIFY_OVER_VBAT                4
#define NOTIFY_BAD_BATTEMP              5
#define NOTIFY_LOW_BATTEMP              6
#define NOTIFY_OVER_BATTEMP             7
#define NOTIFY_BAT_POSTFULL             8
#define NOTIFY_BAT_FULL                 9
#define NOTIFY_CHARGER_EXIST            10
#define NOTIFY_NORMAL_BAT_FULL          11
#define NOTIFY_LOWTEMP_BAT_FULL         12
#define NOTIFY_HIGHTEMP_BAT_FULL        13
#define NOTIFY_THIRD_BAT_FULL           14


#ifdef OPPO_USE_EXCHARGER
typedef enum
{
FAN5405_ID,
BQ24156A_ID,
BQ24157_ID
}oppo_excharger_id_enum;
#endif

#ifdef OPPO_USE_II_BQ24156A
typedef enum
{
BQ24156A_CURRENT_0MA,
BQ24156A_CURRENT_221MA,
BQ24156A_CURRENT_374MA,
BQ24156A_CURRENT_442MA,
BQ24156A_CURRENT_510MA,
BQ24156A_CURRENT_578MA,
BQ24156A_CURRENT_646MA,
BQ24156A_CURRENT_714MA,
BQ24156A_CURRENT_782MA,
BQ24156A_CURRENT_850MA,
BQ24156A_CURRENT_918MA,
BQ24156A_CURRENT_986MA,
BQ24156A_CURRENT_1054MA
}bq24156a_charging_current_enum;
#endif

#ifdef OPPO_USE_II_BQ24157
#ifdef OPPO_BQ24157_R68
typedef enum
{
BQ24157_CURRENT_0MA,
BQ24157_CURRENT_221MA,
BQ24157_CURRENT_325MA,
BQ24157_CURRENT_550MA,
BQ24157_CURRENT_650MA,
BQ24157_CURRENT_750MA,
BQ24157_CURRENT_850MA,
BQ24157_CURRENT_950MA,
BQ24157_CURRENT_1050MA,
BQ24157_CURRENT_1150MA,
BQ24157_CURRENT_1250MA
}bq24157_charging_current_enum;
#else
typedef enum
{
BQ24157_CURRENT_0MA,
BQ24157_CURRENT_221MA,
BQ24157_CURRENT_374MA,
BQ24157_CURRENT_442MA,
BQ24157_CURRENT_510MA,
BQ24157_CURRENT_578MA,
BQ24157_CURRENT_646MA,
BQ24157_CURRENT_714MA,
BQ24157_CURRENT_782MA,
BQ24157_CURRENT_850MA
}bq24157_charging_current_enum;
#endif
#endif

#ifdef OPPO_USE_FAIRCHILD_FAN5405
typedef enum
{
FAN5405_CURRENT_0MA,
FAN5405_CURRENT_221MA,
FAN5405_CURRENT_374MA,
FAN5405_CURRENT_442MA,
FAN5405_CURRENT_510MA,
FAN5405_CURRENT_578MA,
FAN5405_CURRENT_646MA,
FAN5405_CURRENT_714MA,
FAN5405_CURRENT_782MA,
FAN5405_CURRENT_850MA
}fan5405_charging_current_enum;
#endif

#ifdef OPPO_USE_EXCHARGER
#ifdef OPPO_USE_II_BQ24156A
#define BQ24156A_USB_CHARGER_CURRENT                BQ24156A_CURRENT_1054MA      
#define BQ24156A_AC_CHARGER_CURRENT                 BQ24156A_CURRENT_1054MA
#define BQ24156A_ZERO_CHARGER_CURRENT               BQ24156A_CURRENT_0MA
#define BQ24156A_INIT_CHARGER_CURRENT               BQ24156A_CURRENT_442MA
#define BQ24156A_SUPER_LOW_TEMP_CHARGER_CURRENT     BQ24156A_CURRENT_221MA
#define BQ24156A_LOW_TEMP_CHARGER_CURRENT           BQ24156A_CURRENT_578MA
#define BQ24156A_LOW_TEMP_POSTFULL_CHARGER_CURRENT  BQ24156A_CURRENT_374MA
#define BQ24156A_HIGH_TEMP_LCD_OFF_CHARGER_CURRENT  BQ24156A_CURRENT_578MA
#define BQ24156A_HIGH_TEMP_LCD_ON_CHARGER_CURRENT   BQ24156A_CURRENT_782MA
#define BQ24156A_AT_CHARGER_CURRENT                 BQ24156A_CURRENT_510MA

#define BQ24156A_STATUS_READY                       0x00
#define BQ24156A_STATUS_IN_PROGESS                  0x01
#define BQ24156A_STATUS_DONE                        0x02
#define BQ24156A_STATUS_FAULT                       0x03
#endif
#ifdef OPPO_USE_II_BQ24157
#ifdef OPPO_BQ24157_R68
#define BQ24157_USB_CHARGER_CURRENT				    BQ24157_CURRENT_1050MA      
#define BQ24157_AC_CHARGER_CURRENT				    BQ24157_CURRENT_1050MA
#define BQ24157_ZERO_CHARGER_CURRENT                BQ24157_CURRENT_0MA
#define BQ24157_INIT_CHARGER_CURRENT                BQ24157_CURRENT_550MA
#define BQ24157_SUPER_LOW_TEMP_CHARGER_CURRENT      BQ24157_CURRENT_221MA
#define BQ24157_LOW_TEMP_CHARGER_CURRENT            BQ24157_CURRENT_550MA
#define BQ24157_LOW_TEMP_POSTFULL_CHARGER_CURRENT   BQ24157_CURRENT_325MA
#define BQ24157_HIGH_TEMP_LCD_OFF_CHARGER_CURRENT   BQ24157_CURRENT_550MA
#define BQ24157_HIGH_TEMP_LCD_ON_CHARGER_CURRENT    BQ24157_CURRENT_750MA
#define BQ24157_AT_CHARGER_CURRENT				    BQ24157_CURRENT_550MA
#else
#define BQ24157_USB_CHARGER_CURRENT				    BQ24157_CURRENT_850MA      
#define BQ24157_AC_CHARGER_CURRENT				    BQ24157_CURRENT_850MA
#define BQ24157_ZERO_CHARGER_CURRENT                BQ24157_CURRENT_0MA
#define BQ24157_INIT_CHARGER_CURRENT                BQ24157_CURRENT_442MA
#define BQ24157_SUPER_LOW_TEMP_CHARGER_CURRENT      BQ24157_CURRENT_221MA
#define BQ24157_LOW_TEMP_CHARGER_CURRENT            BQ24157_CURRENT_578MA
#define BQ24157_LOW_TEMP_POSTFULL_CHARGER_CURRENT   BQ24157_CURRENT_374MA
#define BQ24157_HIGH_TEMP_LCD_OFF_CHARGER_CURRENT   BQ24157_CURRENT_578MA
#define BQ24157_HIGH_TEMP_LCD_ON_CHARGER_CURRENT    BQ24157_CURRENT_782MA
#define BQ24157_AT_CHARGER_CURRENT				    BQ24157_CURRENT_510MA
#endif

//#define BQ24156A_STATUS_READY                       0x00
//#define BQ24156A_STATUS_IN_PROGESS                  0x01
//#define BQ24156A_STATUS_DONE                        0x02
//#define BQ24156A_STATUS_FAULT                       0x03
#endif
#ifdef OPPO_USE_FAIRCHILD_FAN5405
#define FAN5405_USB_CHARGER_CURRENT				    FAN5405_CURRENT_850MA      
#define FAN5405_AC_CHARGER_CURRENT				    FAN5405_CURRENT_850MA
#define FAN5405_ZERO_CHARGER_CURRENT                FAN5405_CURRENT_0MA
#define FAN5405_INIT_CHARGER_CURRENT                FAN5405_CURRENT_442MA
#define FAN5405_SUPER_LOW_TEMP_CHARGER_CURRENT      FAN5405_CURRENT_221MA
#define FAN5405_LOW_TEMP_CHARGER_CURRENT            FAN5405_CURRENT_578MA
#define FAN5405_LOW_TEMP_POSTFULL_CHARGER_CURRENT   FAN5405_CURRENT_374MA
#define FAN5405_HIGH_TEMP_LCD_OFF_CHARGER_CURRENT   FAN5405_CURRENT_578MA
#define FAN5405_HIGH_TEMP_LCD_ON_CHARGER_CURRENT    FAN5405_CURRENT_782MA
#define FAN5405_AT_CHARGER_CURRENT				    FAN5405_CURRENT_510MA

#define FAN5405_STATUS_READY                        0x00
#define FAN5405_STATUS_IN_PROGESS                   0x01
#define FAN5405_STATUS_DONE                         0x02
#define FAN5405_STATUS_FAULT                        0x03
#endif
#else /*OPPO_USE_EXCHARGER*/
#define USB_CHARGER_CURRENT                         Cust_CC_450MA      
#define AC_CHARGER_CURRENT                          Cust_CC_550MA
#define ZERO_CHARGER_CURRENT                        Cust_CC_0MA
#define INIT_CHARGER_CURRENT                        Cust_CC_70MA
#define SUPER_LOW_TEMP_CHARGER_CURRENT              Cust_CC_200MA
#define LOW_TEMP_CHARGER_CURRENT                    Cust_CC_400MA
#define LOW_TEMP_POSTFULL_CHARGER_CURRENT           Cust_CC_400MA
#define HIGH_TEMP_LCD_OFF_CHARGER_CURRENT           Cust_CC_200MA
#define HIGH_TEMP_LCD_ON_CHARGER_CURRENT            Cust_CC_550MA
#define AT_CHARGER_CURRENT                          Cust_CC_450MA
#endif /*OPPO_USE_EXCHARGER*/

#define D_BAT_THREAD_CB_TIME			(1000)

#define MAX_VBAT_LI     4500

#define PRE_COLD_RBAT_PULL_UP_VOLT          1150
#define PRE_HOT_RBAT_PULL_UP_VOLT           1100 
    
/* Battery Temperature Protection */
#define OPPO_MAX_CHARGE_TEMPERATURE          55
#define OPPO_HIGH_CHARGE_TEMPERATURE         45
#define OPPO_LOW_CHARGE_TEMPERATURE          10
#define OPPO_SUPER_LOW_CHARGE_TEMPERATURE    0
#define OPPO_MIN_CHARGE_TEMPERATURE          -10
#define OPPO_BAD_CHARGE_TEMPERATURE          -20//-40
//#define ERR_CHARGE_TEMPERATURE          0xFF

#ifdef OPPO_KERNEL_CHARGING
#define OPPO_MAX_PRE_CHARGING_TIME           30*60 	    // 30min
#define OPPO_MAX_CV_CHARGING_TIME            2*60*60    // 2hr
#define OPPO_MAX_CHARGING_TIME               6*60*60 	// 6hr
#define OPPO_MAX_USBCHARGING_TIME            8*60*60 	// 8hr
#else/*OPPO_UBOOT_CHARGING*/
#define OPPO_MAX_PRE_CHARGING_TIME           30*55//30*60 	    // 30min
#define OPPO_MAX_CV_CHARGING_TIME            30*60//2*60*55//2*60*60    // 2hr
#define OPPO_MAX_CHARGING_TIME               6*60*55//6*60*60 	// 6hr
#define OPPO_MAX_USBCHARGING_TIME            8*60*55//10*60*60 	// 8hr
#endif


#define INIT_CHARGE_TEMP_STATUS         0
#define NORMAL_CHARGE_TEMP_STATUS       1
#define SUPER_LOW_CHARGE_TEMP_STATUS    2
#define LOW_CHARGE_TEMP_STATUS          3
#define HIGH_CHARGE_TEMP_STATUS         4

#define LOW_CHARGER_ERROR_RESTORING     3
#define OVER_CHARGER_ERROR_RESTORING    3
#define LOW_CHARGER_PROTECT_FG          0x01
#define OVER_CHARGER_PROTECT_FG         0x02

#define LOW_VCHARGER_CHECK_COUNT    5
#define OVER_VCHARGER_CHECK_COUNT   5
#define MAX_VBAT_CHECK_COUNT        5
#define BATTEMP_STATUS_CHECK_COUNT  5
#define BAD_BATTEMP_CHECK_COUNT     5
#define LOW_BATTEMP_CHECK_COUNT     5
#define OVER_BATTEMP_CHECK_COUNT    5
#define OPPO_VBAT_TOPOFF_COUNT      10
#define OPPO_VBAT_POSTFULL_COUNT    10
#define OPPO_VBAT_FULL_COUNT        10

#define OPPO_CHARGING_POSTFULL_CURRENT          130
#define OPPO_CHARGING_FULL_CURRENT              130

#define SUPER_LOW_CHARGE_TOPPOFF_THRESHOLD              4000
#define HI_BAT_SUPER_LOW_CHARGE_TOPPOFF_THRESHOLD       4000
#define LOW_CHARGE_TOPPOFF_THRESHOLD                    4050
#define HI_BAT_LOW_CHARGE_TOPPOFF_THRESHOLD             4150
#define HIGH_CHARGE_TOPPOFF_THRESHOLD                   4050
#define HI_BAT_HIGH_CHARGE_TOPPOFF_THRESHOLD            4050
#define NORMAL_CHARGE_TOPPOFF_THRESHOLD                 4050
#define HI_BAT_NORMAL_CHARGE_TOPPOFF_THRESHOLD          4150

#define SUPER_LOW_CHARGE_POSTFULL_THRESHOLD             4000
#define HI_BAT_SUPER_LOW_CHARGE_POSTFULL_THRESHOLD      4000 //HI_BAT
#define LOW_CHARGE_POSTFULL_THRESHOLD                   4100
#define HI_BAT_LOW_CHARGE_POSTFULL_THRESHOLD            4200 //HI_BAT
#define HIGH_CHARGE_POSTFULL_THRESHOLD                  4100
#define HI_BAT_HIGH_CHARGE_POSTFULL_THRESHOLD           4100 //HI_BAT
#define NORMAL_CHARGE_POSTFULL_THRESHOLD                4100
#define HI_BAT_NORMAL_CHARGE_POSTFULL_THRESHOLD         4200 //HI_BAT

#define SUPER_LOW_CHARGE_FULL_THRESHOLD                 4000
#define HI_BAT_SUPER_LOW_CHARGE_FULL_THRESHOLD          4000 //HI_BAT
#define LOW_CHARGE_FULL_THRESHOLD                       4200
#define HI_BAT_LOW_CHARGE_FULL_THRESHOLD                4340 //4300 //HI_BAT
#define HIGH_CHARGE_FULL_THRESHOLD                      4100
#define HI_BAT_HIGH_CHARGE_FULL_THRESHOLD               4100 //HI_BAT
#define NORMAL_CHARGE_FULL_THRESHOLD                    4200
#define HI_BAT_NORMAL_CHARGE_FULL_THRESHOLD             4340 //4300 //HI_BAT
#ifdef OPPO_USE_II_BQ24156A
#define BQ24156A_SUPER_LOW_DCDC_OREG                    0x23 // 4.20    0x1e // 4.10    0x19 // 4.00
#define BQ24156A_HI_BAT_SUPER_LOW_DCDC_OREG             0x23 // 4.20    0x1e // 4.10    0x19 // 4.00
#define BQ24156A_LOW_DCDC_OREG                          0x23 // 4.20
#define BQ24156A_HI_BAT_LOW_DCDC_OREG                   0x28 // 4.30
#define BQ24156A_HIGH_DCDC_OREG                         0x23 // 4.20    0x1e // 4.10
#define BQ24156A_HI_BAT_HIGH_DCDC_OREG                  0x23 // 4.20    0x1e // 4.10
#define BQ24156A_NORMAL_DCDC_OREG                       0x23 // 4.20
#define BQ24156A_HI_BAT_NORMAL_DCDC_OREG                0x2a // 4.34    0x28 // 4.30
#endif
#ifdef OPPO_USE_FAIRCHILD_FAN5405
#define FAN5405_SUPER_LOW_DCDC_OREG                     0x8c // 4.20    0x78 // 4.10    0x64 // 4.00
#define FAN5405_HI_BAT_SUPER_LOW_DCDC_OREG              0x8c // 4.20    0x78 // 4.10    0x64 // 4.00
#define FAN5405_LOW_DCDC_OREG                           0x8c // 4.20
#define FAN5405_HI_BAT_LOW_DCDC_OREG                    0xa0 // 4.30
#define FAN5405_HIGH_DCDC_OREG                          0x8c // 4.20    0x78 // 4.10
#define FAN5405_HI_BAT_HIGH_DCDC_OREG                   0x8c // 4.20    0x78 // 4.10
#define FAN5405_NORMAL_DCDC_OREG                        0x8c // 4.20
#define FAN5405_HI_BAT_NORMAL_DCDC_OREG                 0xa8 // 4.34    0xa0 // 4.30
#endif

#define SUPER_LOW_RECHARGING_THRESHOLD                  3800
#define HI_BAT_SUPER_LOW_RECHARGING_THRESHOLD           3800 //HI_BAT
#define LOW_RECHARGING_THRESHOLD                        4100
#define HI_BAT_LOW_RECHARGING_THRESHOLD                 4200 //HI_BAT
#define HIGH_RECHARGING_THRESHOLD                       3900
#define HI_BAT_HIGH_RECHARGING_THRESHOLD                3900 //HI_BAT
#define NORMAL_RECHARGING_THRESHOLD                     4100
#define HI_BAT_NORMAL_RECHARGING_THRESHOLD              4200 //HI_BAT

#define PRE_TO_FULL_CHARGNIG_THRESHOLD                  4200
#define HI_BAT_PRE_TO_FULL_CHARGNIG_THRESHOLD           4340 //4300 //HI_BAT

#define OPPO_V_CHARGER_MAX                      5800
#define OPPO_V_CHARGER_MIN                      4400
#define OPPO_IPO_BOOT_VOL                	3450    

#ifdef OPPO_CALL_MODE_CHARGING_PAUSE
#define IN_CALL_HIGH_VOL                4050
#define IN_CALL_LOW_VOL                 3800
#endif

#ifdef OPPO_CHARGE_CURRENT_ADAPTIVE
#define CHARGER_VOLTAGE_THRESHOLD   4300            //4.3V
#endif

#ifdef OPPO_KERNEL_CHARGING
#define OPPO_BAT_VOL_DISCARD_TIME               8     //(8*10)S
#define OPPO_BAT_VOL_FIRST_OFFSET               67    //67mV
#define OPPO_BAT_VOL_CHG_PWRON_FIRST_OFFSET     5     //5mV
#define OPPO_BAT_VOL_CHG_IN_OFFSET              23    //23mV
#define OPPO_BAT_VOL_LCD_OFF_OFFSET             40    //40mV
#define OPPO_BAT_VOL_LCD_ON_OFFSET              40    //40mV
#define OPPO_SOC_UP_SYNC_TIME                   5     // 1min
#define OPPO_SOC_DOWN_SYNC_TIME                 5     // 1min

#else/*OPPO_UBOOT_CHARGING*/
#define OPPO_UBOOT_FIRST_BAT_OFFSET             -30    //-30mV
#define OPPO_UBOOT_FIRST_BAT_NO_CHARGER_OFFSET  11     //11mV
#define OPPO_UBOOT_BAT_VOL_OFFSET               11     //11mV
#define OPPO_LK_FIRST_BAT_OFFSET                18     //18mV
#define OPPO_LK_FIRST_USB_ON_BAT_OFFSET         50     //50mV
#define OPPO_LK_FIRST_USB_OFF_BAT_OFFSET        50     //50mV
#endif

#ifdef OPPO_USE_II_BQ24156A
static void check_bq24156a_status(void);
static void upmu_chr_vbat_cs_vth_by_bq24156a(kal_uint8 val);
#endif
#ifdef OPPO_USE_FAIRCHILD_FAN5405
static void check_fan5405_status(void);
static void upmu_chr_vbat_cs_vth_by_fan5405(kal_uint8 val);
#endif
#ifdef OPPO_UBOOT_CHARGING
static BOOL oppo_check_battery_exist(void);
#endif
#ifdef OPPO_KERNEL_CHARGING
static kal_int32 oppo_get_first_bat_vol(void);
#endif
static void oppo_charger_in_reset_variable(void);
static void oppo_charging_status_by_temp(void);
#ifdef OPPO_FACTORY_TEST
static void set_at_charger_current(void);
#endif
static void check_charger_current_ability(void);
static kal_uint8 if_check_charger_ability(void);
static void set_charge_adaptive_current(void);
static void close_charge_adaptive_current(void);
static void OPPO_BAT_PhyCheck(void);
static void OPPO_BAT_Charge(void);
#endif /* OPPO_NEW_CHARGING */ 
#endif /* _OPPO_BATTERY_H_ */ 
