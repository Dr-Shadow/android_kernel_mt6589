/*****************************************************************************
 *
 * Filename:
 * ---------
 *    mt6320_battery.c
 *
 * Project:
 * --------
 *   Android_Software
 *
 * Description:
 * ------------
 *   This Module defines functions of mt6320 Battery charging algorithm 
 *   and the Anroid Battery service for updating the battery status
 *
 * Author:
 * -------
 * James Lo
 *
 ****************************************************************************/
#include <linux/init.h>        /* For init/exit macros */
#include <linux/module.h>      /* For MODULE_ marcros  */
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/power_supply.h>
#include <linux/wakelock.h>
#include <linux/time.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/xlog.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <mach/hardware.h>
#include <mach/system.h>

#include <mach/mt_typedefs.h>
#include <mach/mt_gpt.h>
#include <mach/mt_boot.h>
#include <oppo_chg_def.h>
#include <oppo_battery.h>
#ifdef OPPO_USE_II_BQ24156A
#include <bq24156a.h>
#endif
#ifdef OPPO_USE_II_BQ24157
#include <bq24157.h>
#endif
#ifdef OPPO_USE_FAIRCHILD_FAN5405
#include <fan5405.h>
#endif
#include <cust_battery.h>  
#include "mt6320_battery.h"

#include <mach/pmic_mt6320_sw.h>
#include <mach/upmu_common.h>
#include <mach/upmu_hw.h>

#include <mach/mt_sleep.h>
#include <mach/mt_gpio.h>

#ifdef VENDOR_EDIT//Nanwei.Deng@BasicDrv.CHG, 2012/03/08, Add for Battery encryption
#include "bq2202a.h"
#endif/*VENDOR_EDIT*/

#ifdef OPPO_NEW_CHARGING
#undef MTK_JEITA_STANDARD_SUPPORT
#endif

#ifndef OPPO_NEW_CHARGING
int Enable_BATDRV_LOG = 2;
#else
int Enable_BATDRV_LOG = 1;
#endif

///////////////////////////////////////////////////////////////////////////////////////////
//// Thermal related flags
///////////////////////////////////////////////////////////////////////////////////////////
int g_battery_thermal_throttling_flag=3; // 0:nothing, 1:enable batTT&chrTimer, 2:disable batTT&chrTimer, 3:enable batTT, disable chrTimer
int battery_cmd_thermal_test_mode=0;
int battery_cmd_thermal_test_mode_value=0;
int g_battery_tt_check_flag=0; // 0:default enable check batteryTT, 1:default disable check batteryTT

///////////////////////////////////////////////////////////////////////////////////////////
//// JEITA
///////////////////////////////////////////////////////////////////////////////////////////
#if defined(MTK_JEITA_STANDARD_SUPPORT)
int g_jeita_recharging_voltage=4110;
int gFGsyncTimer_jeita = 0;
int g_default_sync_time_out_jeita= CUST_SOC_JEITA_SYNC_TIME;
int g_temp_status=TEMP_POS_10_TO_POS_45;
kal_bool temp_error_recovery_chr_flag =KAL_TRUE;
int mtk_jeita_support_flag=1;
#else
int mtk_jeita_support_flag=0;
#endif

///////////////////////////////////////////////////////////////////////////////////////////
//// PMIC AUXADC Related APIs
///////////////////////////////////////////////////////////////////////////////////////////
#define AUXADC_BATTERY_VOLTAGE_CHANNEL  0
#define AUXADC_REF_CURRENT_CHANNEL         1
#define AUXADC_CHARGER_VOLTAGE_CHANNEL  2
#define AUXADC_TEMPERATURE_CHANNEL         3
int g_R_BAT_SENSE = R_BAT_SENSE;
int g_R_I_SENSE = R_I_SENSE;
int g_R_CHARGER_1 = R_CHARGER_1;
int g_R_CHARGER_2 = R_CHARGER_2;

#ifdef OPPO_NEW_CHARGING
static kal_uint8 oppo_protect_to_hold_status;
static kal_uint8 low_charger_count;
static kal_uint8 low_charger_error_count;
static kal_uint8 over_charger_count;
static kal_uint8 over_charger_error_count;
static kal_uint8 bad_temper_count;
static kal_uint8 max_vbat_count;
static kal_uint8 low_temper_count;
static kal_uint8 over_temper_count;

static kal_uint8 high_temp_status_count = 0;
static kal_uint8 super_low_temp_status_count = 0;
static kal_uint8 low_temp_status_count = 0;
static kal_uint8 normal_temp_status_count = 0;
static kal_uint8 oppo_charge_temp_status = INIT_CHARGE_TEMP_STATUS;
static kal_uint32 oppo_charge_to_topoff_threshold;
static kal_uint32 oppo_charge_to_postfull_threshold;
static kal_uint32 oppo_charge_to_full_threshold;
static kal_uint32 oppo_charge_to_recharging_threshold;
static kal_uint32 oppo_check_charger_off_vol = 5000;//5.0V
static kal_uint32 oppo_check_vbat_off_vol;
#ifdef OPPO_HIGH_BATTERY_VOLTAGE
kal_uint8 oppo_high_battery_status = 1;
kal_uint8 oppo_DC_charger_init_sign= 0;
kal_uint8 oppo_check_ID_status = 0;
kal_uint8 oppo_high_battery_status_count = 0;  
#endif

#ifdef OPPO_CALL_MODE_CHARGING_PAUSE
static kal_uint8 oppo_call_pause_mode=KAL_FALSE;
extern volatile int call_status;
#endif
#ifdef OPPO_CHARGE_CURRENT_ADAPTIVE
static kal_uint8 check_charger_ability_fg;// = ZERO_CHARGER_CURRENT;
static kal_uint8 charger_current_abilty;// = INIT_CHARGER_CURRENT;
static kal_uint8 oppo_actual_cc_volue;// = ZERO_CHARGER_CURRENT;
#endif
#ifdef OPPO_CHARGE_CURRENT_BY_LCD
extern volatile int oppo_lcd_status;
#endif
#ifdef OPPO_NEW_BAT_VOL_ALG
static kal_uint8 oppo_soc_sync_time = 0;
#endif
#ifdef OPPO_FACTORY_TEST
static kal_uint8 oppo_charging_stop_fg = KAL_FALSE;
#endif
#if defined(OPPO_USE_II_BQ24156A) || defined(OPPO_USE_II_BQ24157)
static kal_uint8 oppo_bq24156a_status = BQ24156A_STATUS_READY;
#endif
#ifdef OPPO_USE_FAIRCHILD_FAN5405
static kal_uint8 oppo_fan5405_status = FAN5405_STATUS_READY;
#endif
static kal_uint8 USB_CHARGER_CURRENT;
static kal_uint8 AC_CHARGER_CURRENT;
static kal_uint8 ZERO_CHARGER_CURRENT;
static kal_uint8 INIT_CHARGER_CURRENT;
static kal_uint8 SUPER_LOW_TEMP_CHARGER_CURRENT;
static kal_uint8 LOW_TEMP_CHARGER_CURRENT;
static kal_uint8 LOW_TEMP_POSTFULL_CHARGER_CURRENT;
static kal_uint8 HIGH_TEMP_LCD_OFF_CHARGER_CURRENT;
static kal_uint8 HIGH_TEMP_LCD_ON_CHARGER_CURRENT;
static kal_uint8 AT_CHARGER_CURRENT;
#ifdef OPPO_USE_EXCHARGER
kal_uint8 oppo_excharger_id;
#endif
static kal_uint8 oppo_ipod_status = 0;
#endif/* OPPO_NEW_CHARGING */ 

#ifdef VENDOR_EDIT//Fanhong.Kong@ProDrv.CHG, add 2012/7/6 for low bat power on
#define BAT_LOW_CLOSED_POWERON

#ifdef BAT_LOW_CLOSED_POWERON
static kal_int8 bat_low_closed_status = KAL_FALSE;
extern void kpd_pwkey_longpass(void);
kal_int8 g_BAT_Low_Close = KAL_FALSE;
kal_int8 g_IPO_status = KAL_FALSE;
#endif
#endif //#ifdef VENDOR_EDIT
#if defined(CONFIG_POWER_VERIFY)
kal_bool upmu_is_chr_det(void)
{
    return 1;
}
EXPORT_SYMBOL(upmu_is_chr_det);

int get_charger_detect_status(void)
{
    return upmu_get_rgs_chrdet();
}
EXPORT_SYMBOL(get_charger_detect_status);

#else
///////////////////////////////////////////////////////////////////////////////////////////
//// PMIC FGADC Related APIs
///////////////////////////////////////////////////////////////////////////////////////////
extern void mt_power_off(void);
//void mt_power_off(void)
//{
//    printk("mt_power_off is empty\n");
//}
extern kal_int32 FGADC_Get_BatteryCapacity_CoulombMothod(void);
extern kal_int32 FGADC_Get_BatteryCapacity_VoltageMothod(void);
extern void FGADC_Reset_SW_Parameter(void);
extern void FGADC_thread_kthread(void);
extern void fgauge_initialization(void);
extern kal_int32 fgauge_read_capacity_by_v(void);
extern kal_int32 fgauge_read_current(void);
extern int get_r_fg_value(void);
extern kal_bool get_gFG_Is_Charging(void);

extern bool mt_usb_is_device(void);
extern void mt_usb_connect(void);
extern void mt_usb_disconnect(void);

extern CHARGER_TYPE mt_charger_type_detection(void);
#if 0//def VENDOR_EDIT //Jiada.Li@BasicDrv.Charger, 2013/06/18, for  Charger type judgment 
extern CHARGER_TYPE oppo_mt_charger_type_detection(void);
#endif
extern int PMIC_IMM_GetOneChannelValue(int dwChannel, int deCount, int trimd);
extern kal_uint32 upmu_get_reg_value(kal_uint32 reg);
extern void upmu_set_reg_value(kal_uint32 reg, kal_uint32 reg_val);

extern kal_int32 gFG_current;
extern kal_int32 gFG_voltage;
extern kal_int32 gFG_DOD0;
extern kal_int32 gFG_DOD1;
extern kal_int32 gFG_columb;
extern kal_bool gFG_Is_Charging;
extern int gFG_15_vlot;

extern BOOTMODE g_boot_mode;

#ifndef VENDOR_EDIT
//rendong.shi@BasicDrv.PRD add for avoiding  noise in charging for TP 
unsigned int tp_is_in_charging = 0;
unsigned int tp_set = 0;
extern unsigned int is_tp_init;
extern int tp_charge_mode_set(int on);
#endif
///////////////////////////////////////////////////////////////////////////////////////////
//// PMIC HW Related APIs
///////////////////////////////////////////////////////////////////////////////////////////
void charger_hv_init(void)
{
    upmu_set_rg_vcdt_hv_vth(0xB);    //VCDT_HV_VTH, 7V
}

U32 get_charger_hv_status(void)
{
    return upmu_get_rgs_vcdt_hv_det();
}

void kick_charger_wdt(void)
{
    upmu_set_rg_chrwdt_td(0x0);           // CHRWDT_TD, 4s
    upmu_set_rg_chrwdt_int_en(1);         // CHRWDT_INT_EN
    upmu_set_rg_chrwdt_en(1);             // CHRWDT_EN
    upmu_set_rg_chrwdt_wr(1);             // CHRWDT_WR
}

int get_bat_sense_volt(int times)
{
    return PMIC_IMM_GetOneChannelValue(0,times,1);
}

int get_i_sense_volt(int times)
{
    return PMIC_IMM_GetOneChannelValue(1,times,1);
}

int get_charger_volt(int times)
{
    return PMIC_IMM_GetOneChannelValue(2,times,1);
}

extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);
extern int IMM_IsAdcInitReady(void);

int get_tbat_volt(int times)
{
#if defined(MTK_PCB_TBAT_FEATURE)
    
        int ret = 0, data[4], i, ret_value = 0, ret_temp = 0;
        int Channel=1;
        
        if( IMM_IsAdcInitReady() == 0 )
        {
            if (Enable_BATDRV_LOG == 1) {
                xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[get_tbat_volt] AUXADC is not ready");
            }
            return 0;
        }
    
        i = times;
        while (i--)
        {
            ret_value = IMM_GetOneChannelValue(Channel, data, &ret_temp);
            ret += ret_temp;
            if (Enable_BATDRV_LOG == 1) {
                xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[get_tbat_volt] ret_temp=%d\n",ret_temp);
            }
        }
        
        ret = ret*1500/4096 ;
        ret = ret/times;
        if (Enable_BATDRV_LOG == 1) {
            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[get_tbat_volt] Battery output mV = %d\n",ret);
        }
    
        return ret;
        
#else

    if(upmu_get_cid() == 0x1020)
    {
        return PMIC_IMM_GetOneChannelValue(4,times,1);
    }
    else    
    {
        #if defined(ENABLE_TBAT_TREF_SUPPORT)
            if (Enable_BATDRV_LOG == 1) {
                xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[get_tbat_volt] return PMIC_IMM_GetOneChannelValue(3,times,1);\n");
            }
            return PMIC_IMM_GetOneChannelValue(3,times,1);
        #else
            if (Enable_BATDRV_LOG == 1) {
                xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[get_tbat_volt] return PMIC_IMM_GetOneChannelValue(4,times,1);\n");
            }
            return PMIC_IMM_GetOneChannelValue(4,times,1);
        #endif    
    }

#endif    
}

int get_charger_detect_status(void)
{
    return upmu_get_rgs_chrdet();
}

int PMIC_IMM_GetOneChannelValueSleep(int dwChannel, int deCount)
{
    kal_int32 adc_result = 0;

    adc_result = PMIC_IMM_GetOneChannelValue(dwChannel,deCount,1);

    return adc_result;
    
}

///////////////////////////////////////////////////////////////////////////////////////////
//// PMIC PCHR Related APIs
///////////////////////////////////////////////////////////////////////////////////////////
kal_bool upmu_is_chr_det(void)
{
    kal_uint32 tmp32;
    tmp32=get_charger_detect_status();
    if(tmp32 == 0)
    {
        //xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[upmu_is_chr_det] No charger\n");
        return KAL_FALSE;
    }
    else
    {
        //xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[upmu_is_chr_det] Charger exist\n");
        if( mt_usb_is_device() )
        {
            if (Enable_BATDRV_LOG == 1) {
                xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[upmu_is_chr_det] Charger exist and USB is not host\n");
            }
            return KAL_TRUE;
        }
        else
        {
            if (Enable_BATDRV_LOG == 1) {
                xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[upmu_is_chr_det] Charger exist but USB is host\n");
            }
            return KAL_FALSE;
        }
        
        //return KAL_TRUE;
    }
}
EXPORT_SYMBOL(upmu_is_chr_det);
#endif // CONFIG_POWER_VERIFY

///////////////////////////////////////////////////////////////////////////////////////////
//// Smart Battery Structure
///////////////////////////////////////////////////////////////////////////////////////////
#define UINT32 unsigned long
#define UINT16 unsigned short
#define UINT8 unsigned char 

typedef struct 
{
    kal_bool       bat_exist;
    kal_bool       bat_full;  
    kal_bool       bat_low;  
    INT32          bat_charging_state;
    INT32          bat_vol;            
    kal_bool     charger_exist;   
    INT32          pre_charging_current;
    INT32          charging_current;
    INT32          charger_vol;        
    INT32       charger_protect_status; 
    INT32          ISENSE;                
    INT32          ICharging;
    INT32       temperature;
    UINT32      total_charging_time;
    UINT32      PRE_charging_time;
    UINT32      CC_charging_time;
    UINT32      TOPOFF_charging_time;
    UINT32      POSTFULL_charging_time;
    INT32       charger_type;
    INT32       PWR_SRC;
    INT32       SOC;
    INT32       ADC_BAT_SENSE;
    INT32       ADC_I_SENSE;
} PMU_ChargerStruct;

typedef enum 
{
    PMU_STATUS_OK = 0,
    PMU_STATUS_FAIL = 1,
}PMU_STATUS;

///////////////////////////////////////////////////////////////////////////////////////////
//// Global Variable
///////////////////////////////////////////////////////////////////////////////////////////
static CHARGER_TYPE CHR_Type_num = CHARGER_UNKNOWN;
static unsigned short batteryVoltageBuffer[BATTERY_AVERAGE_SIZE];
static unsigned short batteryCurrentBuffer[BATTERY_AVERAGE_SIZE];
static unsigned short batterySOCBuffer[BATTERY_AVERAGE_SIZE];
static int batteryTempBuffer[BATTERY_AVERAGE_SIZE];
static int batteryIndex = 0;
static int batteryVoltageSum = 0;
static int batteryCurrentSum = 0;
static int batterySOCSum = 0;
static int batteryTempSum = 0;
PMU_ChargerStruct BMT_status;
kal_bool g_bat_full_user_view = KAL_FALSE;
kal_bool g_Battery_Fail = KAL_FALSE;
kal_bool batteryBufferFirst = KAL_FALSE;

struct wake_lock battery_suspend_lock; 

int V_PRE2CC_THRES = 3400;
int V_CC2TOPOFF_THRES = 4050;

int g_HW_Charging_Done = 0;
int g_Charging_Over_Time = 0;

int g_HW_stop_charging = 0;

int CHARGING_FULL_CURRENT=220;    // mA 

#ifndef OPPO_NO_USE_FGADC
int gForceADCsolution=0;
#else
int gForceADCsolution=1;
#endif

int gSyncPercentage=0;

unsigned int g_BatteryNotifyCode=0x0000;
unsigned int g_BN_TestMode=0x0000;

kal_uint32 gFGsyncTimer=0;
kal_uint32 DEFAULT_SYNC_TIME_OUT=60; //1mins

int g_Calibration_FG=0;

int g_XGPT_restart_flag=0;

#define CHR_OUT_CURRENT    50

int gSW_CV_prepare_flag=0;

int getVoltFlag = 0;
int g_bat_temperature_pre=0;

int gADC_BAT_SENSE_temp=0;
int gADC_I_SENSE_temp=0;
int gADC_I_SENSE_offset=0;

int g_battery_flag_resume=0;

int gBAT_counter_15=1;

int gFG_can_reset_flag = 1;

//cut off to full
#define POST_CHARGING_TIME 30 * 60 // 30mins
int post_charging_time=0;

extern kal_int32 gFG_booting_counter_I_FLAG;
int g_fg_ready_flag = 0;

int read_tbat_value(void)
{
    return BMT_status.temperature;
}

////////////////////////////////////////////////////////////////////////////////
// EM
////////////////////////////////////////////////////////////////////////////////
int g_BAT_TemperatureR = 0;
int g_TempBattVoltage = 0;
int g_InstatVolt = 0;
int g_BatteryAverageCurrent = 0;
int g_BAT_BatterySenseVoltage = 0;
int g_BAT_ISenseVoltage = 0;
int g_BAT_ChargerVoltage = 0;

////////////////////////////////////////////////////////////////////////////////
// Definition For GPT
////////////////////////////////////////////////////////////////////////////////
static int bat_thread_timeout = 0;

static DEFINE_MUTEX(bat_mutex);
static DECLARE_WAIT_QUEUE_HEAD(bat_thread_wq);

////////////////////////////////////////////////////////////////////////////////
//Logging System
////////////////////////////////////////////////////////////////////////////////
int g_chr_event = 0;
int bat_volt_cp_flag = 0;
int bat_volt_check_point = 0;
int g_wake_up_bat=0;

void wake_up_bat (void)
{
    if (Enable_BATDRV_LOG == 1) {
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] wake_up_bat. \r\n");
    }

    g_wake_up_bat=1;    

    bat_thread_timeout = 1;
    wake_up(&bat_thread_wq);
}
EXPORT_SYMBOL(wake_up_bat);

////////////////////////////////////////////////////////////////////////////////
// USB-IF
////////////////////////////////////////////////////////////////////////////////
int g_usb_state = USB_UNCONFIGURED;

#ifndef OPPO_NEW_CHARGING
int g_temp_CC_value = Cust_CC_0MA;
#else
int g_temp_CC_value;// = ZERO_CHARGER_CURRENT;
#endif
int g_soft_start_delay = 1;

#if (CONFIG_USB_IF == 0)
int g_Support_USBIF = 0;
#else
int g_Support_USBIF = 1;
#endif

////////////////////////////////////////////////////////////////////////////////
// Integrate with NVRAM 
////////////////////////////////////////////////////////////////////////////////
#define ADC_CALI_DEVNAME "MT_pmic_adc_cali"

#define TEST_ADC_CALI_PRINT _IO('k', 0)
#define SET_ADC_CALI_Slop _IOW('k', 1, int)
#define SET_ADC_CALI_Offset _IOW('k', 2, int)
#define SET_ADC_CALI_Cal _IOW('k', 3, int)
#define ADC_CHANNEL_READ _IOW('k', 4, int)
#define BAT_STATUS_READ _IOW('k', 5, int)
#define Set_Charger_Current _IOW('k', 6, int)

#ifdef OPPO_NEW_CHARGING
#define Get_FakeOff_Param _IOW('k', 7, int)
#define Get_Notify_Param _IOW('k', 8, int)
#define Turn_Off_Charging _IOW('k', 9, int)

//add for auto test
#define K_AT_CHG_CHGR_IN   _IOW('k', 10, int)
#define K_AT_CHG_CHGR_OFF  _IOW('k', 11, int)
#define K_AT_CHG_ON      _IOW('k', 12, int)
#define K_AT_CHG_OFF      _IOW('k', 13, int)
#define K_AT_CHG_INFO         _IOW('k', 14, int)
static int auto_out_data[5]={0,0,0,0,0};
static int at_test_chg_on=1;// default is on;
static int at_enter_chging_state = 0;

int fakeoff_out_data[3] = {0,0,0};
int notify_out_data[1] = {0};

int g_NotifyFlag = 0;


/*#define     Notify_Charger_Over_Vol                   1 
#define     Notify_Charger_Low_Vol                    2 
#define     Notify_Bat_Over_Temp                      3
#define     Notify_Bat_Low_Temp                       4
#define     Notify_Bat_Not_Connect                    5
#define     Notify_Bat_Over_Vol                       6
#define     Notify_Bat_Full                           7
#define     Notify_Chging_Current                     8*/
#endif

static struct class *adc_cali_class = NULL;
static int adc_cali_major = 0;
static dev_t adc_cali_devno;
static struct cdev *adc_cali_cdev;

int adc_cali_slop[14] = {1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000};
int adc_cali_offset[14] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int adc_cali_cal[1] = {0};

int adc_in_data[2] = {1,1};
int adc_out_data[2] = {1,1};

int battery_in_data[1] = {0};
int battery_out_data[1] = {0};    

int charging_level_data[1] = {0};
int g_bat_init_flag=0;

kal_bool g_ADC_Cali = KAL_FALSE;
kal_bool g_ftm_battery_flag = KAL_FALSE;

////////////////////////////////////////////////////////////////////////////////
// Battery Logging Entry
////////////////////////////////////////////////////////////////////////////////
static struct proc_dir_entry *proc_entry;
static char proc_bat_data[32];  

ssize_t bat_log_write( struct file *filp, const char __user *buff,
                        unsigned long len, void *data )
{
    if (copy_from_user( &proc_bat_data, buff, len )) {
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "bat_log_write error.\n");
        return -EFAULT;
    }

    if (proc_bat_data[0] == '1') {
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "enable battery driver log system\n");
        Enable_BATDRV_LOG = 1;
    } else if (proc_bat_data[0] == '2') {
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "enable battery driver log system:2\n");
        Enable_BATDRV_LOG = 2;    
    } else {
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "Disable battery driver log system\n");
        Enable_BATDRV_LOG = 0;
    }
    
    return len;
}

int init_proc_log(void)
{
    int ret=0;
    proc_entry = create_proc_entry( "batdrv_log", 0644, NULL );
    
    if (proc_entry == NULL) {
        ret = -ENOMEM;
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "init_proc_log: Couldn't create proc entry\n");
    } else {
        proc_entry->write_proc = bat_log_write;       
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "init_proc_log loaded.\n");
    }
  
    return ret;
}

////////////////////////////////////////////////////////////////////////////////
// FOR ANDROID BATTERY SERVICE
////////////////////////////////////////////////////////////////////////////////
struct mt6320_ac_data {
    struct power_supply psy;
    int AC_ONLINE;    
};

struct mt6320_usb_data {
    struct power_supply psy;
    int USB_ONLINE;    
};

struct mt6320_battery_data {
    struct power_supply psy;
    int BAT_STATUS;
    int BAT_HEALTH;
    int BAT_PRESENT;
    int BAT_TECHNOLOGY;
    int BAT_CAPACITY;
    /* Add for Battery Service*/
    int BAT_batt_vol;
    int BAT_batt_temp;
    /* Add for EM */
    int BAT_TemperatureR;
    int BAT_TempBattVoltage;
    int BAT_InstatVolt;
    int BAT_BatteryAverageCurrent;
    int BAT_BatterySenseVoltage;
    int BAT_ISenseVoltage;
    int BAT_ChargerVoltage;
    #ifdef OPPO_NEW_CHARGING
    int battery_request_poweroff;
    #endif
};

static enum power_supply_property mt6320_ac_props[] = {
    POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property mt6320_usb_props[] = {
    POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property mt6320_battery_props[] = {
    POWER_SUPPLY_PROP_STATUS,
    POWER_SUPPLY_PROP_HEALTH,
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_TECHNOLOGY,
    POWER_SUPPLY_PROP_CAPACITY,
    /* Add for Battery Service */
    POWER_SUPPLY_PROP_batt_vol,
    POWER_SUPPLY_PROP_batt_temp,    
    /* Add for EM */
    POWER_SUPPLY_PROP_TemperatureR,
    POWER_SUPPLY_PROP_TempBattVoltage,
    POWER_SUPPLY_PROP_InstatVolt,
    POWER_SUPPLY_PROP_BatteryAverageCurrent,
    #ifdef OPPO_NEW_CHARGING
    POWER_SUPPLY_PROP_BatteryRequestPoweroff,
    #endif
    POWER_SUPPLY_PROP_BatterySenseVoltage,
    POWER_SUPPLY_PROP_ISenseVoltage,
    POWER_SUPPLY_PROP_ChargerVoltage,
};

static int mt6320_ac_get_property(struct power_supply *psy,
    enum power_supply_property psp,
    union power_supply_propval *val)
{
    int ret = 0;
    struct mt6320_ac_data *data = container_of(psy, struct mt6320_ac_data, psy);    

    switch (psp) {
    case POWER_SUPPLY_PROP_ONLINE:                           
        val->intval = data->AC_ONLINE;
        break;
    default:
        ret = -EINVAL;
        break;
    }
    return ret;
}

static int mt6320_usb_get_property(struct power_supply *psy,
    enum power_supply_property psp,
    union power_supply_propval *val)
{
    int ret = 0;
    struct mt6320_usb_data *data = container_of(psy, struct mt6320_usb_data, psy);    

    switch (psp) {
    case POWER_SUPPLY_PROP_ONLINE:     
        #if defined(CONFIG_POWER_EXT)
        //#if 0
        data->USB_ONLINE = 1;
        val->intval = data->USB_ONLINE;
        #else
        val->intval = data->USB_ONLINE;
        #endif        
        break;
    default:
        ret = -EINVAL;
        break;
    }
    return ret;
}

static int mt6320_battery_get_property(struct power_supply *psy,
    enum power_supply_property psp,
    union power_supply_propval *val)
{
    int ret = 0;     
    struct mt6320_battery_data *data = container_of(psy, struct mt6320_battery_data, psy);

    switch (psp) {
    case POWER_SUPPLY_PROP_STATUS:
        val->intval = data->BAT_STATUS;
        break;
    case POWER_SUPPLY_PROP_HEALTH:
        val->intval = data->BAT_HEALTH;
        break;
    case POWER_SUPPLY_PROP_PRESENT:
        val->intval = data->BAT_PRESENT;
        break;
    case POWER_SUPPLY_PROP_TECHNOLOGY:
        val->intval = data->BAT_TECHNOLOGY;
        break;
    case POWER_SUPPLY_PROP_CAPACITY:
        val->intval = data->BAT_CAPACITY;
        break;        
    case POWER_SUPPLY_PROP_batt_vol:
        val->intval = data->BAT_batt_vol;
        break;
    case POWER_SUPPLY_PROP_batt_temp:
        val->intval = data->BAT_batt_temp;
        break;
    case POWER_SUPPLY_PROP_TemperatureR:
        val->intval = data->BAT_TemperatureR;
        break;    
    case POWER_SUPPLY_PROP_TempBattVoltage:        
        val->intval = data->BAT_TempBattVoltage;
        break;    
    case POWER_SUPPLY_PROP_InstatVolt:
        val->intval = data->BAT_InstatVolt;
        break;    
    case POWER_SUPPLY_PROP_BatteryAverageCurrent:
        val->intval = data->BAT_BatteryAverageCurrent;
        break;    
	#ifdef OPPO_NEW_CHARGING
    case POWER_SUPPLY_PROP_BatteryRequestPoweroff:
	    val->intval = data->battery_request_poweroff;
	    break;
	#endif
    case POWER_SUPPLY_PROP_BatterySenseVoltage:
        val->intval = data->BAT_BatterySenseVoltage;
        break;    
    case POWER_SUPPLY_PROP_ISenseVoltage:
        val->intval = data->BAT_ISenseVoltage;
        break;    
    case POWER_SUPPLY_PROP_ChargerVoltage:
        val->intval = data->BAT_ChargerVoltage;
        break;

    default:
        ret = -EINVAL;
        break;
    }

    return ret;
}

/* mt6320_ac_data initialization */
static struct mt6320_ac_data mt6320_ac_main = {
    .psy = {
    .name = "ac",
    .type = POWER_SUPPLY_TYPE_MAINS,
    .properties = mt6320_ac_props,
    .num_properties = ARRAY_SIZE(mt6320_ac_props),
    .get_property = mt6320_ac_get_property,                
    },
    .AC_ONLINE = 0,
};

/* mt6320_usb_data initialization */
static struct mt6320_usb_data mt6320_usb_main = {
    .psy = {
    .name = "usb",
    .type = POWER_SUPPLY_TYPE_USB,
    .properties = mt6320_usb_props,
    .num_properties = ARRAY_SIZE(mt6320_usb_props),
    .get_property = mt6320_usb_get_property,                
    },
    .USB_ONLINE = 0,
};

/* mt6320_battery_data initialization */
static struct mt6320_battery_data mt6320_battery_main = {
    .psy = {
    .name = "battery",
    .type = POWER_SUPPLY_TYPE_BATTERY,
    .properties = mt6320_battery_props,
    .num_properties = ARRAY_SIZE(mt6320_battery_props),
    .get_property = mt6320_battery_get_property,                
    },
/* CC: modify to have a full power supply status */
#if defined(CONFIG_POWER_EXT)
//#if 0
    .BAT_STATUS = POWER_SUPPLY_STATUS_FULL,    
    .BAT_HEALTH = POWER_SUPPLY_HEALTH_GOOD,
    .BAT_PRESENT = 1,
    .BAT_TECHNOLOGY = POWER_SUPPLY_TECHNOLOGY_LION,
    .BAT_CAPACITY = 100,
    .BAT_batt_vol = 4200,
    .BAT_batt_temp = 22,
#else
    .BAT_STATUS = POWER_SUPPLY_STATUS_NOT_CHARGING,    
    .BAT_HEALTH = POWER_SUPPLY_HEALTH_GOOD,
    .BAT_PRESENT = 1,
    .BAT_TECHNOLOGY = POWER_SUPPLY_TECHNOLOGY_LION,
    .BAT_CAPACITY = 50,
    .BAT_batt_vol = 0,
    .BAT_batt_temp = 0,
    #ifdef OPPO_NEW_CHARGING
    .battery_request_poweroff=0,
    #endif
#endif
};

#if defined(CONFIG_POWER_EXT)
#else
static void mt6320_ac_update(struct mt6320_ac_data *ac_data)
{
    struct power_supply *ac_psy = &ac_data->psy;

    if( upmu_is_chr_det() == KAL_TRUE )
    {         
        if ( (BMT_status.charger_type == NONSTANDARD_CHARGER) || 
             (BMT_status.charger_type == STANDARD_CHARGER)        )
        {
            ac_data->AC_ONLINE = 1;        
            ac_psy->type = POWER_SUPPLY_TYPE_MAINS;
        }
    }
    else
    {
        ac_data->AC_ONLINE = 0;        
    }

    power_supply_changed(ac_psy);    
}

static void mt6320_usb_update(struct mt6320_usb_data *usb_data)
{
    struct power_supply *usb_psy = &usb_data->psy;

    if( upmu_is_chr_det() == KAL_TRUE )        
    {
    	#ifdef VENDOR_EDIT //Nanwei.Deng@BasicDrv.CHG, 2013/03/08, Add for usb fault detection
        printk("dengnanwei--charger_type_old = %d \n", BMT_status.charger_type);
        if(BMT_status.charger_type == CHARGING_HOST)//for standard charger before poweron
        {
            CHR_Type_num = mt_charger_type_detection();//oppo_mt_charger_type_detection();
            printk("--charger_type_old = %d, charger_type_new = %d\r\n", BMT_status.charger_type, CHR_Type_num);
            BMT_status.charger_type = CHR_Type_num;
            if((CHR_Type_num != STANDARD_HOST) && (CHR_Type_num != CHARGING_HOST) )
            {
                mt_usb_disconnect();
            }
        }
		#endif /*VENDOR_EDIT*/
		
        if ( (BMT_status.charger_type == STANDARD_HOST) ||
             (BMT_status.charger_type == CHARGING_HOST)        )
        {
            usb_data->USB_ONLINE = 1;            
            usb_psy->type = POWER_SUPPLY_TYPE_USB;            
        }
    }
    else
    {
        usb_data->USB_ONLINE = 0;
    }   

    power_supply_changed(usb_psy); 
}

extern int set_rtc_spare_fg_value(int val);
extern int g_tracking_point;

static void mt6320_battery_update(struct mt6320_battery_data *bat_data)
{
    struct power_supply *bat_psy = &bat_data->psy;
    int i;

    #ifndef VENDOR_EDIT//Nanwei.Deng@BasicDrv.CHG, 2012/03/08, Add for Battery encryption
    static int CheckIDSign=5;
	if(CheckIDSign>0)
    {
        CheckIDCompare();
        CheckIDSign--;
		printk( "<dengnanwei_check_ID = %d: oppo_high_battery_status =%d :%s: LINE= %d : \n", oppo_check_ID_status,oppo_high_battery_status,__FUNCTION__,__LINE__);
        if(oppo_check_ID_status == 6)
        {
            oppo_high_battery_status = 1;
            oppo_check_ID_status=0;
            CheckIDSign =0;    
        }
        else if((CheckIDSign==0)&& (oppo_check_ID_status != 6))
        {
            oppo_high_battery_status = 0;
            oppo_check_ID_status=0;
        }
        
		printk( "<dengnanwei_dayin_ID = %d: oppo_high_battery_status =%d :%s: LINE= %d : \n", oppo_check_ID_status,oppo_high_battery_status,__FUNCTION__,__LINE__);
    }
    #endif/*VENDOR_EDIT*/

    bat_data->BAT_TECHNOLOGY = POWER_SUPPLY_TECHNOLOGY_LION;
    bat_data->BAT_HEALTH = POWER_SUPPLY_HEALTH_GOOD;
    bat_data->BAT_batt_vol = BMT_status.bat_vol;
    bat_data->BAT_batt_temp= BMT_status.temperature * 10;

    if (BMT_status.bat_exist)
        bat_data->BAT_PRESENT = 1;
    else
        bat_data->BAT_PRESENT = 0;

    /* Charger and Battery Exist */
    //if( (upmu_is_chr_det(CHR)==KAL_TRUE) && (!g_Battery_Fail) )
    if( (upmu_is_chr_det()==KAL_TRUE) && (!g_Battery_Fail) && (g_Charging_Over_Time==0))
    {     
        if ( BMT_status.bat_exist )                
        {
			printk("pmic6320_battery  BMT_status.bat_full= %d %s: LINE= %d :\n", BMT_status.bat_full, __FUNCTION__,__LINE__);

            /* Battery Full */
#if defined(MTK_JEITA_STANDARD_SUPPORT)
            if ( (BMT_status.bat_vol >= g_jeita_recharging_voltage) && (BMT_status.bat_full == KAL_TRUE) )
#else
            #ifndef OPPO_NEW_BAT_VOL_ALG
            if ( (BMT_status.bat_vol >= RECHARGING_VOLTAGE) && (BMT_status.bat_full == KAL_TRUE) )
            #else
            if ( (BMT_status.bat_vol >= OPPO_BAT_VOLT_PERCENT100) && (BMT_status.bat_full == KAL_TRUE) )
            #endif
#endif            	
            {
                /*Use no gas gauge*/
                if( gForceADCsolution == 1 )
                {
                    bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_FULL;
                    bat_data->BAT_CAPACITY = Battery_Percent_100;
                    
                    /* For user view */
                    for (i=0; i<BATTERY_AVERAGE_SIZE; i++) {
                        batterySOCBuffer[i] = 100; 
                        batterySOCSum = 100 * BATTERY_AVERAGE_SIZE; /* for user view */
                    }
                    bat_volt_check_point = 100;
                }
                /*Use gas gauge*/
                else
                {                    
                    gSyncPercentage=1;
                    
#if defined(MTK_JEITA_STANDARD_SUPPORT)
                    //increase after xxs
                    if(gFGsyncTimer_jeita >= g_default_sync_time_out_jeita)
                    {
                        gFGsyncTimer_jeita=0;
                        bat_volt_check_point++;					   
                    }
                    else
                    {
                        gFGsyncTimer_jeita+=10;
                        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[Battery] In JEITA (%d on %d)\r\n", 
                            bat_volt_check_point, gFGsyncTimer_jeita);
                    }
#else                    
                    bat_volt_check_point++;
#endif                    
                    if(bat_volt_check_point>=100)
                    {
                        bat_volt_check_point=100;
                        bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_FULL;                        
                    }
                    bat_data->BAT_CAPACITY = bat_volt_check_point;
                    
                    if (Enable_BATDRV_LOG == 1) {
                        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[Battery] In FULL Range (%d)\r\n", bat_volt_check_point);
                    }
                    
                        gSyncPercentage=1;

                        if (Enable_BATDRV_LOG == 1) {
                            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[Battery_SyncRecharge] In recharging state, do not sync FG\r\n");
                        }
                }
            }
            /* battery charging */
            else 
            {
                #if 1//ndef OPPO_NEW_BAT_VOL_ALG
                /* Do re-charging for keep battery soc */
                if (g_bat_full_user_view) 
                {
                    bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_FULL;
                    #ifdef OPPO_NEW_BAT_VOL_ALG
                    if(BMT_status.bat_vol >= (HI_BAT_NORMAL_RECHARGING_THRESHOLD-50))
                    #endif
                    {
                    bat_data->BAT_CAPACITY = Battery_Percent_100;

                    /* For user view */
                    for (i=0; i<BATTERY_AVERAGE_SIZE; i++) {
                        batterySOCBuffer[i] = 100; 
                        batterySOCSum = 100 * BATTERY_AVERAGE_SIZE; /* for user view */
                    }
                    bat_volt_check_point = 100;
                    }
                    #ifdef OPPO_NEW_BAT_VOL_ALG
                    else
                    {
                        bat_data->BAT_CAPACITY = bat_volt_check_point;
                    }
                    #endif

                    gSyncPercentage=1;
                    if (Enable_BATDRV_LOG == 1) {
                        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[Battery_Recharging] Keep UI as 100. bat_volt_check_point=%d, BMT_status.SOC=%ld\r\n", 
                        bat_volt_check_point, BMT_status.SOC);
                    }
                }
                else
                #endif
                {
                    bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_CHARGING;                    

                    /*Use no gas gauge*/
                    if( gForceADCsolution == 1 )
                    {
                        /* SOC only UP when charging */
                        if ( BMT_status.SOC > bat_volt_check_point ) {                        
	                        #ifndef OPPO_NEW_BAT_VOL_ALG
                            bat_volt_check_point = BMT_status.SOC;
							#else
							if(oppo_soc_sync_time >= OPPO_SOC_UP_SYNC_TIME)
                            {
                                oppo_soc_sync_time = 0;
                                bat_volt_check_point++;                            	
                            }  
                            else
                            {
                                oppo_soc_sync_time += 1;
                            }
							#endif
                        } 
                        bat_data->BAT_CAPACITY = bat_volt_check_point;
                    }
                    /*Use gas gauge*/
                    else
                    {                        
                        if(bat_volt_check_point >= 100 )
                        {                    
                            #ifndef OPPO_NEW_BAT_VOL_ALG
                            bat_volt_check_point=99;
                            #endif
                            //BMT_status.SOC=99;
                            gSyncPercentage=1;

                            //if (Enable_BATDRV_LOG == 1) {
                                xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[Battery] Use gas gauge : gas gague get 100 first (%d)\r\n", bat_volt_check_point);
                            //}
                        }
                        else
                        {
                            #ifndef OPPO_NEW_BAT_VOL_ALG
                            if(bat_volt_check_point == BMT_status.SOC)
                            #else//Jiada.Li@BasicDrv.CHG, 2012/04/09,debug for power on in 0%,and charger in, percent cannot update 
                            if(bat_volt_check_point <= BMT_status.SOC)
                            #endif
                            {
                                gSyncPercentage=0;

                                if (Enable_BATDRV_LOG == 1) {
                                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[Battery] Can sync due to bat_volt_check_point=%d, BMT_status.SOC=%ld\r\n", 
                                    bat_volt_check_point, BMT_status.SOC);
                                }
                            }
                            else
                            {
                                if (Enable_BATDRV_LOG == 1) {
                                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[Battery] Keep UI due to bat_volt_check_point=%d, BMT_status.SOC=%ld\r\n", 
                                    bat_volt_check_point, BMT_status.SOC);
                                }
                            }
                        }
                        bat_data->BAT_CAPACITY = bat_volt_check_point;                                                
                    }
                }
            }
        }
        /* No Battery, Only Charger */
        else
        {
            bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_UNKNOWN;
            bat_data->BAT_CAPACITY = 0;
        }
        
    }
    /* Only Battery */
    else
    {
        bat_data->BAT_STATUS = POWER_SUPPLY_STATUS_NOT_CHARGING;

        /* If VBAT < CLV, then shutdown */
        if (BMT_status.bat_vol <= SYSTEM_OFF_VOLTAGE)
        {   
            /*Use no gas gauge*/
            if( gForceADCsolution == 1 )
            {
                xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BAT BATTERY] VBAT < %d mV : Android will Power Off System !!\r\n", SYSTEM_OFF_VOLTAGE);                              
	            #ifndef OPPO_NEW_BAT_VOL_ALG
                bat_data->BAT_CAPACITY = 0;
	            #else
	            if(bat_volt_check_point > 0)
	            {
    	            if(oppo_soc_sync_time > OPPO_SOC_DOWN_SYNC_TIME)
        	        {
        	            oppo_soc_sync_time=0;
        	            bat_volt_check_point--;
        	        }
        	        else
        	        {
        	            oppo_soc_sync_time += 1;
        	        }
    	        }
    	        else
    	        {
    	            bat_volt_check_point = 0;
    	        }

    	        bat_data->BAT_CAPACITY = bat_volt_check_point;
	            #endif
            }
            /*Use gas gauge*/
            else
            {                
                gSyncPercentage=1;                
                bat_volt_check_point--;
                if(bat_volt_check_point <= 0)
                {
                    bat_volt_check_point=0;
                }
                g_Calibration_FG = 0;
                FGADC_Reset_SW_Parameter();
                gFG_DOD0=100-bat_volt_check_point;
                gFG_DOD1=gFG_DOD0;
                BMT_status.SOC=bat_volt_check_point;
                bat_data->BAT_CAPACITY = bat_volt_check_point;
                xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[Battery] VBAT < %d mV (%d, gFG_DOD0=%d)\r\n", SYSTEM_OFF_VOLTAGE, bat_volt_check_point,gFG_DOD0);                
            }
        }
        #ifndef OPPO_NO_USE_FGADC
        /* If FG_VBAT <= gFG_15_vlot, then run to 15% */
        //else if ( (gFG_voltage <= gFG_15_vlot)&&(gForceADCsolution==0)&&(bat_volt_check_point>=15) )
        else if ( (gFG_voltage <= gFG_15_vlot)&&(gForceADCsolution==0)&&(bat_volt_check_point>=g_tracking_point) )
        {
            /*Use gas gauge*/
            gSyncPercentage=1;            
            if(gBAT_counter_15==0)
            {
                bat_volt_check_point--;
                gBAT_counter_15=1;
            }        
            else
            {
                gBAT_counter_15=0;
            }
            g_Calibration_FG = 0;
            FGADC_Reset_SW_Parameter();
            gFG_DOD0=100-bat_volt_check_point;
            gFG_DOD1=gFG_DOD0;
            BMT_status.SOC=bat_volt_check_point;
            bat_data->BAT_CAPACITY = bat_volt_check_point;
            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[Battery] FG_VBAT <= %d, then SOC run to %d. (SOC=%ld,Point=%d,D1=%d,D0=%d)\r\n", 
                gFG_15_vlot, g_tracking_point, BMT_status.SOC, bat_volt_check_point, gFG_DOD1, gFG_DOD0);
        }
        /* If "FG_VBAT > gFG_15_vlot" and "FG_report=15%" , then keep 15% till FG_VBAT <= gFG_15_vlot */
        //else if ( (gFG_voltage > gFG_15_vlot)&&(gForceADCsolution==0)&&(bat_volt_check_point==15) )
        else if ( (gFG_voltage > gFG_15_vlot)&&(gForceADCsolution==0)&&(bat_volt_check_point==g_tracking_point) )
        {
            /*Use gas gauge*/
            gSyncPercentage=1;
            gBAT_counter_15=1;
            g_Calibration_FG = 0;
            FGADC_Reset_SW_Parameter();
            gFG_DOD0=100-bat_volt_check_point;
            gFG_DOD1=gFG_DOD0;
            BMT_status.SOC=bat_volt_check_point;
            bat_data->BAT_CAPACITY = bat_volt_check_point;
            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[Battery] FG_VBAT(%d) > gFG_15_vlot(%d) and FG_report=%d, then UI(%d) keep %d. (D1=%d,D0=%d)\r\n", 
                gFG_voltage, gFG_15_vlot, g_tracking_point, bat_volt_check_point, g_tracking_point, gFG_DOD1, gFG_DOD0);
        }        
		#endif
        else
        {          
            gBAT_counter_15=1;
            /*Use no gas gauge*/
            if( gForceADCsolution == 1 )
            {
                /* SOC only Done when dis-charging */
                if ( BMT_status.SOC < bat_volt_check_point ) {
	                #ifndef OPPO_NEW_BAT_VOL_ALG
                    bat_volt_check_point = BMT_status.SOC;
					#else
					if(oppo_soc_sync_time > OPPO_SOC_DOWN_SYNC_TIME)
        	        {
        	            oppo_soc_sync_time=0;
        	            bat_volt_check_point--;
        	        }
        	        else
        	        {
        	            oppo_soc_sync_time += 1;
        	        }
					#endif
                }
                bat_data->BAT_CAPACITY = bat_volt_check_point;            
            }
            /*Use gas gauge : gas gague get 0% fist*/
            else
            {                    
                if (Enable_BATDRV_LOG == 1) {
                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[Battery_OnlyBattery!] bat_volt_check_point=%d,BMT_status.SOC=%ld\r\n", 
                    bat_volt_check_point, BMT_status.SOC);
                }
                
                //if(bat_volt_check_point != BMT_status.SOC)
                //if(bat_volt_check_point > BMT_status.SOC)
                if( (bat_volt_check_point>BMT_status.SOC) && ((bat_volt_check_point!=1)) )
                {        
                    if (Enable_BATDRV_LOG == 1) {
                        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[Battery_OnlyBattery] bat_volt_check_point=%d,BMT_status.SOC=%ld,gFGsyncTimer=%d(on %d)\r\n", 
                        bat_volt_check_point, BMT_status.SOC, gFGsyncTimer, DEFAULT_SYNC_TIME_OUT);
                    }
                    
                    //reduce after xxs
                    if(gFGsyncTimer >= DEFAULT_SYNC_TIME_OUT)
                    {
                        gFGsyncTimer=0;
                        bat_volt_check_point--;
                        bat_data->BAT_CAPACITY = bat_volt_check_point;
                    }
                    else
                    {
                        gFGsyncTimer+=10;
                    }
                }
                else
                {                
                    if(bat_volt_check_point <= 0 )
                    {                    
                        bat_volt_check_point=1;
                        //BMT_status.SOC=1;
                        gSyncPercentage=1;

                        //if (Enable_BATDRV_LOG == 1) {
                            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[Battery] Use gas gauge : gas gague get 0 first (%d)\r\n", bat_volt_check_point);
                        //}
                    }
                    else
                    {
                        gSyncPercentage=0;
                    }

                    if(bat_volt_check_point > 100)
                    {
                        bat_volt_check_point=100;
                    }
                    
                    bat_data->BAT_CAPACITY = bat_volt_check_point;
                }

                if(bat_volt_check_point == 100) {
                    g_bat_full_user_view = KAL_TRUE;
                    if (Enable_BATDRV_LOG == 1) {
                        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[Battery_Only] Set g_bat_full_user_view=KAL_TRUE\r\n");
                    }
                }
            }
        }
    }    

	#ifndef OPPO_NO_USE_FGADC
    if (Enable_BATDRV_LOG >= 1) {
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[FG] %d,%ld,%d,%d,%ld,ADC_Solution=%d\r\n", 
        bat_volt_check_point, BMT_status.SOC, FGADC_Get_BatteryCapacity_VoltageMothod(), 
        BATTERY_AVERAGE_SIZE, BMT_status.bat_vol, gForceADCsolution);    
    }
	#endif

	#ifdef OPPO_NEW_CHARGING	
	#define CRITICAL_LOW_POWER  3400
	
    extern wake_reason_t slp_wake_reason;
    UINT32 instant_bat_vol=0;
    
	if(slp_wake_reason == WR_WAKE_SRC)//WR_SLP_TMR
	{
	    instant_bat_vol = BAT_Get_Battery_Voltage(0);
	    if(instant_bat_vol < CRITICAL_LOW_POWER)
	    {
	        printk("WR_LOW_BAT,instant_bat_vol=%d <CRITICAL_LOW_POWER, request power off\n",instant_bat_vol);
	        bat_data->battery_request_poweroff=1;
	    }
	    else
	    {
	        printk("WR_LOW_BAT,instant_bat_vol=%d >CRITICAL_LOW_POWER, NOT REUQST power off now\n",instant_bat_vol);
	    } 
	}
	else
	{   
	    bat_data->battery_request_poweroff=0;
	}
    slp_wake_reason=WR_NONE;  
    #endif

	
    /* Update for EM */
    bat_data->BAT_TemperatureR=g_BAT_TemperatureR;
    bat_data->BAT_TempBattVoltage=g_TempBattVoltage;
    bat_data->BAT_InstatVolt=g_InstatVolt;
    bat_data->BAT_BatteryAverageCurrent=g_BatteryAverageCurrent;
    bat_data->BAT_BatterySenseVoltage=g_BAT_BatterySenseVoltage;
    bat_data->BAT_ISenseVoltage=g_BAT_ISenseVoltage;
    bat_data->BAT_ChargerVoltage=g_BAT_ChargerVoltage;

	#ifndef OPPO_NO_USE_FGADC
	if (gFG_booting_counter_I_FLAG == 2) {
		if (bat_volt_check_point == 1) {
			set_rtc_spare_fg_value(0);
		}
		else {
    set_rtc_spare_fg_value(bat_volt_check_point);
		}
	}
	#else
	printk("deng_charger_Battery-bat_volt_check_point= %d,BMT_status.SOC= %ld,%s: LINE= %d :\n",bat_volt_check_point, BMT_status.SOC,__FUNCTION__,__LINE__);
	set_rtc_spare_fg_value(bat_volt_check_point);
	#endif
        
    power_supply_changed(bat_psy);    
}

static void mt6320_battery_update_power_down(struct mt6320_battery_data *bat_data)
{
    struct power_supply *bat_psy = &bat_data->psy;

    bat_data->BAT_CAPACITY = 0;

    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[Battery] mt6320_battery_update_power_down\r\n");
        
    power_supply_changed(bat_psy);    
}
#endif

void BAT_UpdateChargerStatus(void)
{
#if !defined(CONFIG_POWER_EXT)	
	if(g_bat_init_flag == 1) {
		mt6320_ac_update(&mt6320_ac_main);
		mt6320_usb_update(&mt6320_usb_main);
	}
#endif	
}

#if defined(CONFIG_POWER_VERIFY)

void BATTERY_SetUSBState(int usb_state_value)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY_SetUSBState] in FPGA/EVB, no service\r\n");
}
EXPORT_SYMBOL(BATTERY_SetUSBState);

static int mt6320_battery_probe(struct platform_device *dev)    
{
    struct class_device *class_dev = NULL;
    int ret=0;
    int i=0;

    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "******** MT6320 battery driver probe!! ********\n" );
        
    /* Integrate with Android Battery Service */
    ret = power_supply_register(&(dev->dev), &mt6320_ac_main.psy);
    if (ret)
    {            
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[MT6320 BAT_probe] power_supply_register AC Fail !!\n");                    
        return ret;
    }             
    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[MT6320 BAT_probe] power_supply_register AC Success !!\n");

    ret = power_supply_register(&(dev->dev), &mt6320_usb_main.psy);
    if (ret)
    {            
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[MT6320 BAT_probe] power_supply_register USB Fail !!\n");                    
        return ret;
    }             
    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[MT6320 BAT_probe] power_supply_register USB Success !!\n");

    ret = power_supply_register(&(dev->dev), &mt6320_battery_main.psy);
    if (ret)
    {
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[MT6320 BAT_probe] power_supply_register Battery Fail !!\n");
        return ret;
    }
    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[MT6320 BAT_probe] power_supply_register Battery Success !!\n");

    /* battery kernel thread for 10s check and charger in/out event */
    //kthread_run(bat_thread_kthread, NULL, "bat_thread_kthread"); //move to pmic driver

    CHR_Type_num = CHARGER_UNKNOWN;

    for (i=0; i<BATTERY_AVERAGE_SIZE; i++) {
        batteryCurrentBuffer[i] = 0;
        batteryVoltageBuffer[i] = 0; 
        batterySOCBuffer[i] = 0;
        batteryTempBuffer[i] = 0;
    }

    batteryIndex = 0;
    batteryVoltageSum = 0;
    batteryCurrentSum = 0;
    batterySOCSum = 0;
    batteryTempSum = 0;

    /* Integrate with NVRAM */
    ret = alloc_chrdev_region(&adc_cali_devno, 0, 1, ADC_CALI_DEVNAME);
    if (ret) 
       xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "Error: Can't Get Major number for adc_cali \n");
    adc_cali_cdev = cdev_alloc();
    adc_cali_cdev->owner = THIS_MODULE;
    //adc_cali_cdev->ops = &adc_cali_fops;
    ret = cdev_add(adc_cali_cdev, adc_cali_devno, 1);
    if(ret)
       xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "adc_cali Error: cdev_add\n");
    adc_cali_major = MAJOR(adc_cali_devno);
    adc_cali_class = class_create(THIS_MODULE, ADC_CALI_DEVNAME);
    class_dev = (struct class_device *)device_create(adc_cali_class, 
                                                   NULL, 
                                                   adc_cali_devno, 
                                                   NULL, 
                                                   ADC_CALI_DEVNAME);
    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[MT6320 BAT_probe] adc_cali prepare : done !!\n ");    

    g_bat_init_flag=1;

    return 0;
}

unsigned long BAT_Get_Battery_Voltage(int polling_mode)
{
    return 4000;
}

#else
///////////////////////////////////////////////////////////////////////////////////////////
//// Battery Temprature Parameters and functions
///////////////////////////////////////////////////////////////////////////////////////////
typedef struct{
    INT32 BatteryTemp;
    INT32 TemperatureR;
}BATT_TEMPERATURE;

/* convert register to temperature  */
INT16 BattThermistorConverTemp(INT32 Res)
{
    int i=0;
    INT32 RES1=0,RES2=0;
    INT32 TBatt_Value=-200,TMP1=0,TMP2=0;

#if defined(BAT_NTC_TSM_1)
BATT_TEMPERATURE Batt_Temperature_Table[] = {
{-20,70603},    
{-15,55183},
{-10,43499},
{ -5,34569},
{  0,27680},
{  5,22316},
{ 10,18104},
{ 15,14773},
{ 20,12122},
{ 25,10000},
{ 30,8294},
{ 35,6915},
{ 40,5795},
{ 45,4882},
{ 50,4133},
{ 55,3516},
{ 60,3004}
};
#endif

#if defined(BAT_NTC_10_SEN_1)        
BATT_TEMPERATURE Batt_Temperature_Table[] = {
 {-20,74354},
 {-15,57626},
 {-10,45068},
 { -5,35548},
 {  0,28267},
 {  5,22650},
 { 10,18280},
 { 15,14855},
 { 20,12151},
 { 25,10000},
 { 30,8279},
 { 35,6892},
 { 40,5768},
 { 45,4852},
 { 50,4101},
 { 55,3483},
 { 60,2970}
};
#endif

#if (BAT_NTC_10 == 1)
#if 1//ndef OPPO_NEW_CHARGING
    BATT_TEMPERATURE Batt_Temperature_Table[] = {
        {-20,68237},
        {-15,53650},
        {-10,42506},
        { -5,33892},
        {  0,27219},
        {  5,22021},
        { 10,17926},
        { 15,14674},
        { 20,12081},
        { 25,10000},
        { 30,8315},
        { 35,6948},
        { 40,5834},
        { 45,4917},
        { 50,4161},
        { 55,3535},
        { 60,3014}
    };
#else
    BATT_TEMPERATURE Batt_Temperature_Table[] = {
		{-40,195652},
		{-35,148171},
		{-30,113347},
		{-25,87559},
        {-20,68237},
        {-15,53650},
        {-10,42506},
        { -5,33892},
        {  0,27219},
        {  5,22021},
        { 10,17926},
        { 15,14674},
        { 20,12081},
        { 25,10000},
        { 30,8315},
        { 35,6948},
        { 40,5834},
        { 45,4917},
        { 50,4161},
		{ 55,3651},
        { 60,3111}
    };
#endif
#endif

#if (BAT_NTC_47 == 1)
    BATT_TEMPERATURE Batt_Temperature_Table[] = {
        {-20,483954},
        {-15,360850},
        {-10,271697},
        { -5,206463},
        {  0,158214},
        {  5,122259},
        { 10,95227},
        { 15,74730},
        { 20,59065},
        { 25,47000},
        { 30,37643},
        { 35,30334},
        { 40,24591},
        { 45,20048},
        { 50,16433},
        { 55,13539},
        { 60,11210}        
    };
#endif

    if(Res>=Batt_Temperature_Table[0].TemperatureR)
    {
        #if 0
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "Res>=%d\n", Batt_Temperature_Table[0].TemperatureR);
        #endif
        #if 1//ndef OPPO_NEW_CHARGING
        TBatt_Value = -20;
        #else
        TBatt_Value = -40;
        #endif
    }
    #if 1//ndef OPPO_NEW_CHARGING
    else if(Res<=Batt_Temperature_Table[16].TemperatureR)
    #else
    else if(Res<=Batt_Temperature_Table[20].TemperatureR)
    #endif
    {
        #if 0
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "Res<=%d\n", Batt_Temperature_Table[16].TemperatureR);
        #endif
        TBatt_Value = 60;
    }
    else
    {
        RES1=Batt_Temperature_Table[0].TemperatureR;
        TMP1=Batt_Temperature_Table[0].BatteryTemp;

        #if 1//ndef OPPO_NEW_CHARGING
        for(i=0;i<=16;i++)
        #else
        for(i=0;i<=20;i++)
        #endif
        {
            if(Res>=Batt_Temperature_Table[i].TemperatureR)
            {
                RES2=Batt_Temperature_Table[i].TemperatureR;
                TMP2=Batt_Temperature_Table[i].BatteryTemp;
                break;
            }
            else
            {
                RES1=Batt_Temperature_Table[i].TemperatureR;
                TMP1=Batt_Temperature_Table[i].BatteryTemp;
            }
        }
        
        TBatt_Value = (((Res-RES2)*TMP1)+((RES1-Res)*TMP2))/(RES1-RES2);
    }

    #if 0
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "BattThermistorConverTemp() : TBatt_Value = %d\n",TBatt_Value);
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "BattThermistorConverTemp() : Res = %d\n",Res);
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "BattThermistorConverTemp() : RES1 = %d\n",RES1);
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "BattThermistorConverTemp() : RES2 = %d\n",RES2);
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "BattThermistorConverTemp() : TMP1 = %d\n",TMP1);
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "BattThermistorConverTemp() : TMP2 = %d\n",TMP2);
    #endif

    return TBatt_Value;    
}

/* convert ADC_bat_temp_volt to register */
INT16 BattVoltToTemp(UINT32 dwVolt)
{
    INT32 TRes;
    INT32 dwVCriBat = 0; 
    INT32 sBaTTMP = -100;

#if 0
    //Temp workaround
    if(upmu_get_cid() == 0x1020)
    {
        sBaTTMP=21;
        return sBaTTMP;
    }
#endif 
    #if 0//def OPPO_NEW_CHARGING
    #if defined(OPPO_USE_FAIRCHILD_FAN5405)
    if( upmu_is_chr_det() == KAL_TRUE)
    {
        dwVolt = dwVolt - oppo_actual_cc_volue * 6;
    }
    else
    {
        //
    }
    #elif defined(OPPO_USE_II_BQ24156A)
    if( upmu_is_chr_det() == KAL_TRUE)
    {
        dwVolt = dwVolt - oppo_actual_cc_volue * 6;
    }
    else
    {
        //
    }
    #else
    #endif
    #endif

    //SW workaround-----------------------------------------------------
    //dwVCriBat = (TBAT_OVER_CRITICAL_LOW * 1800) / (TBAT_OVER_CRITICAL_LOW + 39000);
    dwVCriBat = (TBAT_OVER_CRITICAL_LOW * RBAT_PULL_UP_VOLT) / (TBAT_OVER_CRITICAL_LOW + RBAT_PULL_UP_R);
    
    #if 1//ndef OPPO_NEW_CHARGING
    if(dwVolt > dwVCriBat)
    {
        TRes = TBAT_OVER_CRITICAL_LOW;
    }
    else
    {
        //TRes = (39000*dwVolt) / (1800-dwVolt);
        TRes = (RBAT_PULL_UP_R*dwVolt) / (RBAT_PULL_UP_VOLT-dwVolt);        
    }
    #else
    if(dwVolt > dwVCriBat)
    {
        TRes = TBAT_OVER_CRITICAL_LOW;
    }
    else if(dwVolt > 573)
    {
        TRes = (RBAT_PULL_UP_R*dwVolt)/(PRE_COLD_RBAT_PULL_UP_VOLT-dwVolt);
    }
    else if(dwVolt > 204)
    {
        TRes = (RBAT_PULL_UP_R*dwVolt)/(RBAT_PULL_UP_VOLT-dwVolt);
    }
    else
    {
		TRes = (RBAT_PULL_UP_R*dwVolt)/(PRE_HOT_RBAT_PULL_UP_VOLT-dwVolt);
    }
    #endif
    //------------------------------------------------------------------

    #if 0
    #ifdef OPPO_NEW_CHARGING
    #define OPPO_CHG_TRES_OFFSET    104
    
    if(upmu_is_chr_det() == KAL_TRUE)
    {
        g_BAT_TemperatureR = (TRes * 100) / OPPO_CHG_TRES_OFFSET;
    }
    else
    #endif
    #else
    #ifdef OPPO_NEW_CHARGING
    #define OPPO_CHG_TRES_OFFSET    110
    if(TRes < 4161)//Batt_Temperature_Table[14].TemperatureR===>   > 50
    {
       TRes = (TRes * OPPO_CHG_TRES_OFFSET) / 100;
    }
    #endif
    #endif
    g_BAT_TemperatureR = TRes;

    /* convert register to temperature */
    sBaTTMP = BattThermistorConverTemp(TRes);

    #if 0
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "BattVoltToTemp() : TBAT_OVER_CRITICAL_LOW = %d\n", TBAT_OVER_CRITICAL_LOW);    
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "BattVoltToTemp() : RBAT_PULL_UP_VOLT = %d\n", RBAT_PULL_UP_VOLT);
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "BattVoltToTemp() : dwVolt = %d\n", dwVolt);
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "BattVoltToTemp() : TRes = %d\n", TRes);
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "BattVoltToTemp() : sBaTTMP = %d\n", sBaTTMP);
    #endif
       
    return sBaTTMP;
}

//void BAT_SetUSBState(int usb_state_value)
void BATTERY_SetUSBState(int usb_state_value)
{
    if ( (usb_state_value < USB_SUSPEND) || ((usb_state_value > USB_CONFIGURED))){
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] BAT_SetUSBState Fail! Restore to default value\r\n");    
        usb_state_value = USB_UNCONFIGURED;
    } else {
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] BAT_SetUSBState Success! Set %d\r\n", usb_state_value);    
        g_usb_state = usb_state_value;    
    }
}
//EXPORT_SYMBOL(BAT_SetUSBState);
EXPORT_SYMBOL(BATTERY_SetUSBState);

kal_bool pmic_chrdet_status(void)
{
    if( upmu_is_chr_det() == KAL_TRUE )    
    {
        return KAL_TRUE;
    }
    else
    {
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[pmic_chrdet_status] No charger\r\n");
        return KAL_FALSE;
    }
}

///////////////////////////////////////////////////////////////////////////////////////////
//// Pulse Charging Algorithm 
///////////////////////////////////////////////////////////////////////////////////////////

int boot_check_once=1;

void select_charging_curret(void)
{
    if (g_ftm_battery_flag) 
    {
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] FTM charging : %d\r\n", charging_level_data[0]);    
        g_temp_CC_value = charging_level_data[0];                
    }    
    else 
    {    
        if ( BMT_status.charger_type == STANDARD_HOST ) 
        {

            if (g_Support_USBIF == 1)
            {
                if (g_usb_state == USB_SUSPEND)
                {
                    g_temp_CC_value = USB_CHARGER_CURRENT_SUSPEND;
                }
                else if (g_usb_state == USB_UNCONFIGURED)
                {
                    g_temp_CC_value = USB_CHARGER_CURRENT_UNCONFIGURED;
                }
                else if (g_usb_state == USB_CONFIGURED)
                {
                    g_temp_CC_value = USB_CHARGER_CURRENT_CONFIGURED;
                }
                else
                {
                    g_temp_CC_value = USB_CHARGER_CURRENT_UNCONFIGURED;
                }
                
                if (Enable_BATDRV_LOG == 1) {
                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] Support BC1.1\r\n");
                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] STANDARD_HOST CC mode charging : %d on %d state\r\n", g_temp_CC_value, g_usb_state);
                }
            }
            else
            {    
            	
#if defined(MTK_JEITA_STANDARD_SUPPORT)
                if(g_temp_status == TEMP_NEG_10_TO_POS_0)
                {
                    g_temp_CC_value = Cust_CC_200MA;   //for low temp	
                }
                else 
                {
                    g_temp_CC_value = USB_CHARGER_CURRENT;            
                }                	
#else            	 
                g_temp_CC_value = USB_CHARGER_CURRENT;            
#endif        
                
                if (Enable_BATDRV_LOG == 1) {
                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] Not Support BC1.1\r\n");
                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] STANDARD_HOST CC mode charging : %d\r\n", USB_CHARGER_CURRENT);
                }
            }
        } 
        else if (BMT_status.charger_type == NONSTANDARD_CHARGER) 
        {   

#if defined(MTK_JEITA_STANDARD_SUPPORT)
            if(g_temp_status == TEMP_NEG_10_TO_POS_0)
            {
                g_temp_CC_value = Cust_CC_200MA;   //for low temp 
            }
            else 
            {
            g_temp_CC_value = USB_CHARGER_CURRENT;
            }                
#else        	
            g_temp_CC_value = USB_CHARGER_CURRENT;
#endif
            
            if (Enable_BATDRV_LOG == 1) {
                xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] NONSTANDARD_CHARGER CC mode charging : %d\r\n", USB_CHARGER_CURRENT); // USB HW limitation
            }
        } 
        else if (BMT_status.charger_type == STANDARD_CHARGER) 
        {

#if defined(MTK_JEITA_STANDARD_SUPPORT)
            if(g_temp_status == TEMP_NEG_10_TO_POS_0)
            {
                g_temp_CC_value = Cust_CC_200MA;   //for low temp	
            }
            else 
            {
            g_temp_CC_value = AC_CHARGER_CURRENT;
            }            	
#else            
            g_temp_CC_value = AC_CHARGER_CURRENT;
#endif
            
            if (Enable_BATDRV_LOG == 1) {
                xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] STANDARD_CHARGER CC mode charging : %d\r\n", AC_CHARGER_CURRENT);
            }           
        }
        else if (BMT_status.charger_type == CHARGING_HOST) 
        {
            
#if defined(MTK_JEITA_STANDARD_SUPPORT)
            if(g_temp_status == TEMP_NEG_10_TO_POS_0)
            {
                g_temp_CC_value = Cust_CC_200MA;   //for low temp	
            }
            else 
            {
                g_temp_CC_value = AC_CHARGER_CURRENT;  
            }            	
#else
            g_temp_CC_value = AC_CHARGER_CURRENT;
#endif
            
            if (Enable_BATDRV_LOG == 1) {
                xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] CHARGING_HOST CC mode charging : %d\r\n", AC_CHARGER_CURRENT);
            }           
        }
        else 
        {
        	#ifndef OPPO_NEW_CHARGING
            g_temp_CC_value = Cust_CC_70MA;
            #else
            g_temp_CC_value = INIT_CHARGER_CURRENT;
            #endif
			
            if (Enable_BATDRV_LOG == 1) {
                xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] Default CC mode charging : %d\r\n", Cust_CC_70MA);
            }            
        }
        
    }

    #ifdef OPPO_NEW_CHARGING
    oppo_charging_status_by_temp();
    #endif
}

void ChargerHwInit(void)
{
    if (Enable_BATDRV_LOG == 1) {
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[MT6320 BAT_probe] ChargerHwInit\n" );
    }

	#ifdef OPPO_USE_EXCHARGER
	//none
	//none
	#else
    kick_charger_wdt();

    upmu_set_rg_vcdt_mode(0);       //VCDT_MODE
    upmu_set_rg_vcdt_hv_en(1);      //VCDT_HV_EN    

    upmu_set_rg_bc11_bb_ctrl(1);    //BC11_BB_CTRL
    upmu_set_rg_bc11_rst(1);        //BC11_RST
    
    upmu_set_rg_csdac_mode(1);      //CSDAC_MODE
    upmu_set_rg_vbat_ov_en(1);      //VBAT_OV_EN
    upmu_set_rg_vbat_ov_vth(0x1);   //VBAT_OV_VTH, 4.3V, 0x1 by ICE
    upmu_set_rg_baton_en(1);        //BATON_EN

    //Tim, for TBAT
    //upmu_set_rg_buf_pwd_b(1);       //RG_BUF_PWD_B
    upmu_set_rg_baton_ht_en(0);     //BATON_HT_EN
    
    upmu_set_rg_ulc_det_en(1);      // RG_ULC_DET_EN=1
    upmu_set_rg_low_ich_db(1);      // RG_LOW_ICH_DB=000001'b
	#endif
}

void pchr_turn_off_charging (void)
{
    #if 1//def OPPO_USE_II_SN111008
    if(oppo_actual_cc_volue == ZERO_CHARGER_CURRENT) return;
    #endif
    
    
    #ifdef OPPO_FACTORY_TEST
    if(at_enter_chging_state == 1) return;
    #endif
    
    if (Enable_BATDRV_LOG == 1) {
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] pchr_turn_off_charging !\r\n");
    }

    #ifdef OPPO_NEW_CHARGING
    close_charge_adaptive_current();
    #endif
    #ifdef OPPO_USE_EXCHARGER
    if(FAN5405_ID == oppo_excharger_id)
    {
        mt_set_gpio_mode(GPIO_FAN5405_DISABLE_NUM,GPIO_FAN5405_DISABLE_MODE);
        mt_set_gpio_dir(GPIO_FAN5405_DISABLE_NUM,GPIO_FAN5405_DISABLE_DIR);
        mt_set_gpio_out(GPIO_FAN5405_DISABLE_NUM,GPIO_FAN5405_DISABLE_OFF_OUT);
    }
    else if(BQ24156A_ID == oppo_excharger_id)
    {
        mt_set_gpio_mode(GPIO_BQ24156A_DISABLE_NUM,GPIO_BQ24156A_DISABLE_MODE);  
        mt_set_gpio_dir(GPIO_BQ24156A_DISABLE_NUM,GPIO_BQ24156A_DISABLE_DIR);
        mt_set_gpio_out(GPIO_BQ24156A_DISABLE_NUM,GPIO_BQ24156A_DISABLE_OFF_OUT);
    }
    else//BQ24157
    {
        mt_set_gpio_mode(GPIO_BQ24156A_DISABLE_NUM,GPIO_BQ24156A_DISABLE_MODE);  
        mt_set_gpio_dir(GPIO_BQ24156A_DISABLE_NUM,GPIO_BQ24156A_DISABLE_DIR);
        mt_set_gpio_out(GPIO_BQ24156A_DISABLE_NUM,GPIO_BQ24156A_DISABLE_OFF_OUT);
    }
    #else
    upmu_set_rg_chrwdt_int_en(0);    // CHRWDT_INT_EN
    upmu_set_rg_chrwdt_en(0);        // CHRWDT_EN
    upmu_set_rg_chrwdt_flag_wr(0);   // CHRWDT_FLAG
    
    upmu_set_rg_csdac_en(0);         // CSDAC_EN
    upmu_set_rg_chr_en(0);           // CHR_EN
    upmu_set_rg_hwcv_en(0);          // RG_HWCV_EN
	#endif

    if (Enable_BATDRV_LOG == 1) {
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] pchr_turn_off_charging : can do charger type detection\r\n");
    }
}


int g_bcct_flag=0;

void select_charging_curret_bcct(void)
{
    // done on set_bat_charging_current_limit
}

int get_bat_charging_current_level(void)
{
    kal_uint32 ret_val=0;
    
    //Get current level
    ret_val=upmu_get_reg_value(0x8);
    
    //Parsing
    if(ret_val==0x00)         return 1600;
    else if(ret_val==0x01)    return 1500;
    else if(ret_val==0x02)    return 1400;
    else if(ret_val==0x03)    return 1300;
    else if(ret_val==0x04)    return 1200;
    else if(ret_val==0x05)    return 1100;
    else if(ret_val==0x06)    return 1000;
    else if(ret_val==0x07)    return  900;
    else if(ret_val==0x08)    return  800;
    else if(ret_val==0x09)    return  700;
    else if(ret_val==0x0A)    return  650;
    else if(ret_val==0x0B)    return  550;
    else if(ret_val==0x0C)    return  450;
    else if(ret_val==0x0D)    return  400;
    else if(ret_val==0x0E)    return  200;
    else if(ret_val==0x0F)    return   70;
    else                      return   -1;
}

int set_bat_charging_current_limit(int current_limit)
{
    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] set_bat_charging_current_limit (%d)\r\n", current_limit);

    if(current_limit != -1)
    {
        g_bcct_flag=1;
        
        if(current_limit < 70)         g_temp_CC_value=0xDD;
        else if(current_limit < 200)   g_temp_CC_value=0xF;
        else if(current_limit < 400)   g_temp_CC_value=0xE;
        else if(current_limit < 450)   g_temp_CC_value=0xD;
        else if(current_limit < 550)   g_temp_CC_value=0xC;
        else if(current_limit < 650)   g_temp_CC_value=0xB;
        else if(current_limit < 700)   g_temp_CC_value=0xA;
        else if(current_limit < 800)   g_temp_CC_value=0x9;
        else if(current_limit < 900)   g_temp_CC_value=0x8;
        else if(current_limit < 1000)  g_temp_CC_value=0x7;
        else if(current_limit < 1100)  g_temp_CC_value=0x6;
        else if(current_limit < 1200)  g_temp_CC_value=0x5;
        else if(current_limit < 1300)  g_temp_CC_value=0x4;
        else if(current_limit < 1400)  g_temp_CC_value=0x3;
        else if(current_limit < 1500)  g_temp_CC_value=0x2;
        else if(current_limit < 1600)  g_temp_CC_value=0x1;
        else if(current_limit == 1600) g_temp_CC_value=0x0;
        else                           g_temp_CC_value=0xC;
    }
    else
    {
        //change to default current setting
        g_bcct_flag=0;
    }
    
    wake_up_bat();

    return g_bcct_flag;
}    

void pchr_turn_on_charging (void)
{
    if ( BMT_status.bat_charging_state == CHR_ERROR ) 
    {
        if (Enable_BATDRV_LOG == 1) {   
            printk("[BATTERY] Charger Error, turn OFF charging !\n");
        }
        pchr_turn_off_charging();
    }
    else if( (get_boot_mode()==META_BOOT) || (get_boot_mode()==ADVMETA_BOOT) )
    {   
        printk("[BATTERY] In meta or advanced meta mode, disable charging.\n");    
        pchr_turn_off_charging();
    }
	#ifdef OPPO_CALL_MODE_CHARGING_PAUSE
	else if((oppo_call_pause_mode == KAL_TRUE) || (oppo_protect_to_hold_status != 0) || (oppo_charging_stop_fg == KAL_TRUE))
	{
	   
    	if (Enable_BATDRV_LOG == 1) {
    		xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] OPPO_CALL_MODE_CHARGING_PAUSE, turn OFF charging !\r\n");
    	}
    	pchr_turn_off_charging();
	}
	#endif
	#ifdef OPPO_FACTORY_TEST
	else if( (get_boot_mode()==META_BOOT) || (get_boot_mode()==ADVMETA_BOOT)/* ||(get_boot_mode()==FACTORY_BOOT) */)
	{
	    if (Enable_BATDRV_LOG == 1) {
			printk("[BATTERY] META_BOOT turn off.\r\n");
		}
		pchr_turn_off_charging();
	}
	else if(at_test_chg_on==0)
	{
	    if (Enable_BATDRV_LOG == 1) {
			printk("[BATTERY] at_test_chg_on==0, disable charging.\r\n");
		}
		pchr_turn_off_charging();
	}
	#endif
    else
    {
        /*HW initialization*/
        ChargerHwInit();
    
        if (Enable_BATDRV_LOG == 1) {
            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] pchr_turn_on_charging !\r\n");
        }

        #ifndef OPPO_NEW_CHARGING
        if (g_bcct_flag == 1)
        {
            select_charging_curret_bcct();
            if (Enable_BATDRV_LOG == 1) {
                printk("[BATTERY] select_charging_curret_bcct !\n");
            }
        }
        else
        {
            select_charging_curret();
            if (Enable_BATDRV_LOG == 1) {
                printk("[BATTERY] select_charging_curret !\n");
            }
        }
        #else
		select_charging_curret();
		#endif
		#ifndef OPPO_NEW_CHARGING
        if( g_temp_CC_value == Cust_CC_0MA)
        #else
        if( g_temp_CC_value == ZERO_CHARGER_CURRENT)
        #endif
        {
            pchr_turn_off_charging();
            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] charging current is set 0mA !\r\n");
        }
        else
        {    
            #ifdef OPPO_USE_EXCHARGER
            if(FAN5405_ID == oppo_excharger_id)
            {
                mt_set_gpio_mode(GPIO_FAN5405_DISABLE_NUM,GPIO_FAN5405_DISABLE_MODE);
                mt_set_gpio_dir(GPIO_FAN5405_DISABLE_NUM,GPIO_FAN5405_DISABLE_DIR);
                mt_set_gpio_out(GPIO_FAN5405_DISABLE_NUM,GPIO_FAN5405_DISABLE_ON_OUT);
            }
            else if(BQ24156A_ID == oppo_excharger_id)
            {
            mt_set_gpio_mode(GPIO_BQ24156A_DISABLE_NUM,GPIO_BQ24156A_DISABLE_MODE);
            mt_set_gpio_dir(GPIO_BQ24156A_DISABLE_NUM,GPIO_BQ24156A_DISABLE_DIR);
            mt_set_gpio_out(GPIO_BQ24156A_DISABLE_NUM,GPIO_BQ24156A_DISABLE_ON_OUT);
            }
            else//BQ24157
            {
                mt_set_gpio_mode(GPIO_BQ24156A_DISABLE_NUM,GPIO_BQ24156A_DISABLE_MODE);  
                mt_set_gpio_dir(GPIO_BQ24156A_DISABLE_NUM,GPIO_BQ24156A_DISABLE_DIR);
                mt_set_gpio_out(GPIO_BQ24156A_DISABLE_NUM,GPIO_BQ24156A_DISABLE_ON_OUT);
            }
            #endif
            
		    #ifndef OPPO_NEW_CHARGING
            //upmu_set_baton_tdet_en(0);              // check me
			#else
//lijiada need check			upmu_chr_baton_tdet_en(1);				// check me
			#endif
		
			#ifndef OPPO_CHARGE_CURRENT_ADAPTIVE
            upmu_set_rg_cs_vth(g_temp_CC_value);    // CS_VTH
			#else
			#ifdef OPPO_FACTORY_TEST
			if(at_enter_chging_state == 1)
			{
			    set_at_charger_current();
			}
			else
			#endif/*OPPO_FACTORY_TEST*/
			{
			    #ifdef OPPO_USE_II_BQ24156A
			    //none
				#elif defined(OPPO_USE_II_BQ24157)
				//none
			    #elif defined(OPPO_USE_FAIRCHILD_FAN5405)
			    //none
			    #else
			    upmu_set_rg_cs_vth(Cust_CC_70MA);	    // CS_VTH
			    #endif
			}
			#endif/*OPPO_CHARGE_CURRENT_ADAPTIVE*/

			#ifdef OPPO_USE_II_BQ24156A
			//none
			#elif defined(OPPO_USE_FAIRCHILD_FAN5405)
			//none
			#else
            upmu_set_rg_csdac_dly(0x4);             // CSDAC_DLY
            upmu_set_rg_csdac_stp(0x1);             // CSDAC_STP
            upmu_set_rg_csdac_stp_inc(0x1);         // CSDAC_STP_INC
            upmu_set_rg_csdac_stp_dec(0x2);         // CSDAC_STP_DEC
            upmu_set_rg_cs_en(1);                   // CS_EN, check me

            upmu_set_rg_hwcv_en(1);
            
            upmu_set_rg_vbat_cv_en(1);              // CV_EN


#if defined(MTK_JEITA_STANDARD_SUPPORT)
            if(g_temp_status == TEMP_POS_10_TO_POS_45)
            {
                upmu_set_rg_vbat_cv_vth(0x0); // VBAT_CV_VTH,4.2V
            }
            else if((g_temp_status == TEMP_NEG_10_TO_POS_0)||(g_temp_status == TEMP_BELOW_NEG_10))
            {
                upmu_set_rg_vbat_cv_vth(0x16); // VBAT_CV_VTH,3.9V low temp
            }
            else if ((g_temp_status == TEMP_POS_0_TO_POS_10)||(g_temp_status == TEMP_POS_45_TO_POS_60)||(g_temp_status == TEMP_ABOVE_POS_60))
            {
                upmu_set_rg_vbat_cv_vth(0x11); // VBAT_CV_VTH.4.1v
            }
            else
            {
                upmu_set_rg_vbat_cv_vth(0x0); // VBAT_CV_VTH,default 4.2v
            }            
#else            
            upmu_set_rg_vbat_cv_vth(0x0);           // VBAT_CV_VTH
#endif            
            
            upmu_set_rg_csdac_en(1);                // CSDAC_EN
            //upmu_set_rg_chr_en(1);                  // CHR_EN
			#endif
#ifdef MTK_KERNEL_POWER_OFF_CHARGING
    #ifndef OPPO_NEW_CHARGING
		upmu_set_rg_chr_en(1); 				// CHR_EN
    #else
		#ifdef OPPO_FACTORY_TEST
		if(at_enter_chging_state == 1)
		{
		    return;
		}
		#endif

		#ifdef OPPO_CHARGE_CURRENT_ADAPTIVE
		if(KAL_TRUE == if_check_charger_ability())
		{
		    check_charger_ability_fg = g_temp_CC_value;
		    oppo_chg_sleep_ms(5);
		    check_charger_current_ability();
		}
		else
		{
		    oppo_chg_sleep_ms(5);
		    set_charge_adaptive_current();
		}
		#endif

    #endif
#else
            // do charging after FG ready            
			#ifndef OPPO_NEW_CHARGING
            if(gFG_booting_counter_I_FLAG == 2)
			#endif
            {
			    #ifdef OPPO_USE_II_BQ24156A
			    //none
				#elif defined(OPPO_USE_II_BQ24157)
			    //none
			    #elif defined(OPPO_USE_FAIRCHILD_FAN5405)
			    //none
			    #else
                upmu_set_rg_chr_en(1);             // CHR_EN
				#endif

				#ifdef OPPO_FACTORY_TEST
				if(at_enter_chging_state == 1)
				{
				    return;
				}
				#endif

				#ifdef OPPO_CHARGE_CURRENT_ADAPTIVE
    			if(KAL_TRUE == if_check_charger_ability())
    			{
    			    check_charger_ability_fg = g_temp_CC_value;
    			    oppo_chg_sleep_ms(5);
    			    check_charger_current_ability();
    			}
    			else
    			{
    			    oppo_chg_sleep_ms(5);
    			    set_charge_adaptive_current();
    			}
    			#endif
				#ifndef OPPO_NO_USE_FGADC
                if(g_fg_ready_flag == 0)
                {
                    FGADC_Reset_SW_Parameter();
                    g_fg_ready_flag = 1;
                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] wait g_fg_ready_flag (%d)\r\n", g_fg_ready_flag);
                }
				#endif
            }
			#ifndef OPPO_NO_USE_FGADC
            else
            {
                pchr_turn_off_charging();
                xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] wait gFG_booting_counter_I_FLAG==2 (%d)\r\n", gFG_booting_counter_I_FLAG);
            }
			#endif
#endif
                        
        }
    }

    if (Enable_BATDRV_LOG == 1) {
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] Reg[0x%x]=0x%x, Reg[0x%x]=0x%x(VBAT_CV_VTH), Reg[0x%x]=0x%x, Reg[0x%x]=0x%x\r\n", 			
            CHR_CON0, upmu_get_reg_value(CHR_CON0), CHR_CON3, upmu_get_reg_value(CHR_CON3), 
            CHR_CON4, upmu_get_reg_value(CHR_CON4), CHR_CON13, upmu_get_reg_value(CHR_CON13) );
    }

}

int BAT_CheckPMUStatusReg(void)
{ 
    if( upmu_is_chr_det() == KAL_TRUE )
    {
        BMT_status.charger_exist = TRUE;
    }
    else
    {   
        BMT_status.charger_exist = FALSE;
        
        BMT_status.total_charging_time = 0;
        BMT_status.PRE_charging_time = 0;
        BMT_status.CC_charging_time = 0;
        BMT_status.TOPOFF_charging_time = 0;
        BMT_status.POSTFULL_charging_time = 0;

        BMT_status.bat_charging_state = CHR_PRE;        
        
        return PMU_STATUS_FAIL;
    }  

    return PMU_STATUS_OK;
}

extern int get_hw_ocv(void);

unsigned long BAT_Get_Battery_Voltage(int polling_mode)
{
    unsigned long ret_val = 0;

#if 1
    if(polling_mode == 1)
        ret_val=get_bat_sense_volt(1);
    else
        ret_val=get_bat_sense_volt(1);
#else
    ret_val=get_hw_ocv();
#endif    

    return ret_val;
}

int g_Get_I_Charging(void)
{
    kal_int32 ADC_BAT_SENSE_tmp[20]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    kal_int32 ADC_BAT_SENSE_sum=0;
    kal_int32 ADC_BAT_SENSE=0;
    kal_int32 ADC_I_SENSE_tmp[20]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    kal_int32 ADC_I_SENSE_sum=0;
    kal_int32 ADC_I_SENSE=0;    
    int repeat=20;
    int i=0;
    int j=0;
    kal_int32 temp=0;
    int ICharging=0;    

    for(i=0 ; i<repeat ; i++)
    {
        ADC_BAT_SENSE_tmp[i] = get_bat_sense_volt(1);
        ADC_I_SENSE_tmp[i] = get_i_sense_volt(1);
    
        ADC_BAT_SENSE_sum += ADC_BAT_SENSE_tmp[i];
        ADC_I_SENSE_sum += ADC_I_SENSE_tmp[i];    
    }

    //sorting    BAT_SENSE 
    for(i=0 ; i<repeat ; i++)
    {
        for(j=i; j<repeat ; j++)
        {
            if( ADC_BAT_SENSE_tmp[j] < ADC_BAT_SENSE_tmp[i] )
            {
                temp = ADC_BAT_SENSE_tmp[j];
                ADC_BAT_SENSE_tmp[j] = ADC_BAT_SENSE_tmp[i];
                ADC_BAT_SENSE_tmp[i] = temp;
            }
        }
    }
    if (Enable_BATDRV_LOG == 1) {
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[g_Get_I_Charging:BAT_SENSE]\r\n");    
        for(i=0 ; i<repeat ; i++ )
        {
            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "%d,", ADC_BAT_SENSE_tmp[i]);
        }
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "\r\n");
    }

    //sorting    I_SENSE 
    for(i=0 ; i<repeat ; i++)
    {
        for(j=i ; j<repeat ; j++)
        {
            if( ADC_I_SENSE_tmp[j] < ADC_I_SENSE_tmp[i] )
            {
                temp = ADC_I_SENSE_tmp[j];
                ADC_I_SENSE_tmp[j] = ADC_I_SENSE_tmp[i];
                ADC_I_SENSE_tmp[i] = temp;
            }
        }
    }
    if (Enable_BATDRV_LOG == 1) {
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[g_Get_I_Charging:I_SENSE]\r\n");    
        for(i=0 ; i<repeat ; i++ )
        {
            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "%d,", ADC_I_SENSE_tmp[i]);
        }
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "\r\n");
    }
        
    ADC_BAT_SENSE_sum -= ADC_BAT_SENSE_tmp[0];
    ADC_BAT_SENSE_sum -= ADC_BAT_SENSE_tmp[1];
    ADC_BAT_SENSE_sum -= ADC_BAT_SENSE_tmp[18];
    ADC_BAT_SENSE_sum -= ADC_BAT_SENSE_tmp[19];        
    ADC_BAT_SENSE = ADC_BAT_SENSE_sum / (repeat-4);

    if (Enable_BATDRV_LOG == 1) {
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[g_Get_I_Charging] ADC_BAT_SENSE=%d\r\n", ADC_BAT_SENSE);
    }

    ADC_I_SENSE_sum -= ADC_I_SENSE_tmp[0];
    ADC_I_SENSE_sum -= ADC_I_SENSE_tmp[1];
    ADC_I_SENSE_sum -= ADC_I_SENSE_tmp[18];
    ADC_I_SENSE_sum -= ADC_I_SENSE_tmp[19];
    ADC_I_SENSE = ADC_I_SENSE_sum / (repeat-4);

    if (Enable_BATDRV_LOG == 1) {
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[g_Get_I_Charging] ADC_I_SENSE(Before)=%d\r\n", ADC_I_SENSE);
    }
    
    ADC_I_SENSE += gADC_I_SENSE_offset;

    if (Enable_BATDRV_LOG == 1) {
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[g_Get_I_Charging] ADC_I_SENSE(After)=%d\r\n", ADC_I_SENSE);
    }

    BMT_status.ADC_BAT_SENSE = ADC_BAT_SENSE;
    BMT_status.ADC_I_SENSE = ADC_I_SENSE;
    
    if(ADC_I_SENSE > ADC_BAT_SENSE)
    {
        #ifdef OPPO_BQ24157_R68
        ICharging = ((ADC_I_SENSE - ADC_BAT_SENSE)*1000)/68;
        #else
        ICharging = (ADC_I_SENSE - ADC_BAT_SENSE)*10/R_CURRENT_SENSE;
        #endif
    }
    else
    {
        ICharging = 0;
    }

    return ICharging;
}

void BAT_GetVoltage(void)
{ 
    int bat_temperature_volt=0;

    int fg_r_value=0;
    kal_int32 fg_current_temp=0;
    kal_bool fg_current_state;
    int bat_temperature_volt_temp=0;

    /* Get V_BAT_SENSE */
    if (g_chr_event == 0) 
    {        
        BMT_status.ADC_BAT_SENSE = get_bat_sense_volt(1);        
    } 
    else 
    {
        /* Just charger in/out event, same as I_sense */
        g_chr_event = 0;        
        BMT_status.ADC_BAT_SENSE = get_i_sense_volt(1);
    }
	#ifndef OPPO_NEW_BAT_VOL_ALG
    BMT_status.bat_vol = BMT_status.ADC_BAT_SENSE;
	#endif

    /* Get V_Charger */
    BMT_status.charger_vol = get_charger_volt(5);
    BMT_status.charger_vol = BMT_status.charger_vol / 100;

    #ifndef OPPO_NEW_CHARGING
    /* Get V_BAT_Temperature */
    bat_temperature_volt = get_tbat_volt(5);
    if(bat_temperature_volt == 0)
    {
        #if 0    
        if(upmu_get_cid() == 0x1020)
            g_bat_temperature_pre=21;
        #endif 
    
        BMT_status.temperature = g_bat_temperature_pre;
        if (Enable_BATDRV_LOG == 1) {
            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] Warning !! bat_temperature_volt == 0, restore temperature value\n\r");
        }
    }
    else
    {
    #if defined(CONFIG_POWER_EXT)
    //by pass
    #else
    //-----------------------------------------------------------------------------
        if( gForceADCsolution == 1 )
        {
            /*Use no gas gauge*/
        }
        else
        {
            fg_r_value = get_r_fg_value();
            fg_current_temp = fgauge_read_current();
            fg_current_temp = fg_current_temp/10;
            fg_current_state = get_gFG_Is_Charging();
            if(fg_current_state==KAL_TRUE)
            {
                bat_temperature_volt_temp = bat_temperature_volt;
                bat_temperature_volt = bat_temperature_volt - ((fg_current_temp*fg_r_value)/1000);
            }
            else
            {
                bat_temperature_volt_temp = bat_temperature_volt;
                bat_temperature_volt = bat_temperature_volt + ((fg_current_temp*fg_r_value)/1000);
            }
            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[tbat_workaround] %d,%d,%d,%d,%d\n", 
                bat_temperature_volt_temp, bat_temperature_volt, fg_current_state, fg_current_temp, fg_r_value);        
        }    
    //-----------------------------------------------------------------------------
    #endif

        BMT_status.temperature = BattVoltToTemp(bat_temperature_volt);
        g_bat_temperature_pre = BMT_status.temperature;     
    }
    if( (g_battery_tt_check_flag==0) && (BMT_status.temperature<60) && (BMT_status.temperature>(-20)) )
    {
        g_battery_thermal_throttling_flag=3;
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] Enable battery TT (%d)\n\r", BMT_status.temperature);
        g_battery_tt_check_flag=1;
    }    
    #endif

    /* Calculate the charging current */
    BMT_status.ICharging = g_Get_I_Charging();

    if (Enable_BATDRV_LOG == 1) {
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY:ADC] VCHR:%d BAT_SENSE:%d I_SENSE:%d Current:%d CAL:%d BatTT:%d\n", BMT_status.charger_vol,
            BMT_status.ADC_BAT_SENSE, BMT_status.ADC_I_SENSE, BMT_status.ICharging, g_ADC_Cali, g_battery_thermal_throttling_flag );
    }

    g_InstatVolt = get_bat_sense_volt(1);
    g_BatteryAverageCurrent = BMT_status.ICharging;
    g_BAT_BatterySenseVoltage = BMT_status.ADC_BAT_SENSE;
    g_BAT_ISenseVoltage = BMT_status.ADC_I_SENSE;
    g_BAT_ChargerVoltage = BMT_status.charger_vol;   
}

UINT32 BattVoltToPercent(UINT16 dwVoltage)
{
    UINT32 m=0;
    UINT32 VBAT1=0,VBAT2=0;
    UINT32 bPercntResult=0,bPercnt1=0,bPercnt2=0;

    if (Enable_BATDRV_LOG == 1) {
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "###### 100 <-> voltage : %d ######\r\n", Batt_VoltToPercent_Table[10].BattVolt);
    }
    
    if(dwVoltage<=Batt_VoltToPercent_Table[0].BattVolt)
    {
        bPercntResult = Batt_VoltToPercent_Table[0].BattPercent;
        return bPercntResult;
    }
    #ifndef OPPO_NEW_CHARGING
    else if (dwVoltage>=Batt_VoltToPercent_Table[10].BattVolt)
    {
        bPercntResult = Batt_VoltToPercent_Table[10].BattPercent;
        return bPercntResult;
    }
    #else
    else if (dwVoltage>=Batt_VoltToPercent_Table[28].BattVolt)
    {
		 bPercntResult = Batt_VoltToPercent_Table[28].BattPercent;	
         return bPercntResult;
    }
    else if (dwVoltage>=Batt_VoltToPercent_Table[27].BattVolt)
    {
		 bPercntResult = Batt_VoltToPercent_Table[27].BattPercent;	
         return bPercntResult;
    }
    else if (dwVoltage>=Batt_VoltToPercent_Table[26].BattVolt)
    {
		 bPercntResult = Batt_VoltToPercent_Table[26].BattPercent;	
         return bPercntResult;
    }
    else if (dwVoltage>=Batt_VoltToPercent_Table[25].BattVolt)
    {
		 bPercntResult = Batt_VoltToPercent_Table[25].BattPercent;	
         return bPercntResult;
    }
    #endif
    else
    {        
        VBAT1 = Batt_VoltToPercent_Table[0].BattVolt;
        bPercnt1 = Batt_VoltToPercent_Table[0].BattPercent;
        #ifndef OPPO_NEW_CHARGING
        for(m=1;m<=10;m++)
        #else
        for(m=1;m<=25;m++)
        #endif
        {
            if(dwVoltage<=Batt_VoltToPercent_Table[m].BattVolt)
            {
                VBAT2 = Batt_VoltToPercent_Table[m].BattVolt;
                bPercnt2 = Batt_VoltToPercent_Table[m].BattPercent;
                break;
            }
            else
            {
                VBAT1 = Batt_VoltToPercent_Table[m].BattVolt;
                bPercnt1 = Batt_VoltToPercent_Table[m].BattPercent;    
            }
        }
    }
    
    bPercntResult = ( ((dwVoltage-VBAT1)*bPercnt2)+((VBAT2-dwVoltage)*bPercnt1) ) / (VBAT2-VBAT1);    

    return bPercntResult;
    
}

#if defined(MTK_JEITA_STANDARD_SUPPORT)
int do_jeita_state_machine(void)
{
    //JEITA battery temp Standard 
    if (BMT_status.temperature >= TEMP_POS_60_THRESHOLD) 
    {
        xlog_printk(ANDROID_LOG_WARN, "Power/Battery", "[BATTERY] Battery Over high Temperature(%d) !!\n\r", TEMP_POS_60_THRESHOLD);  
        g_temp_status = TEMP_ABOVE_POS_60;
        return PMU_STATUS_FAIL; 
    }
    else if(BMT_status.temperature > TEMP_POS_45_THRESHOLD)  
    {             
        if((g_temp_status == TEMP_ABOVE_POS_60) && (BMT_status.temperature >= TEMP_POS_60_THRES_MINUS_X_DEGREE))
        {
            xlog_printk(ANDROID_LOG_WARN, "Power/Battery", "[BATTERY] Battery Temperature between %d and %d,not allow charging yet!!\n\r",
                TEMP_POS_60_THRES_MINUS_X_DEGREE,TEMP_POS_60_THRESHOLD); 
            return PMU_STATUS_FAIL; 
        }
        else
        {
            xlog_printk(ANDROID_LOG_WARN, "Power/Battery", "[BATTERY] Battery Temperature between %d and %d !!\n\r",
                TEMP_POS_45_THRESHOLD,TEMP_POS_60_THRESHOLD); 
            g_temp_status = TEMP_POS_45_TO_POS_60;
            g_jeita_recharging_voltage = 4000;
            V_CC2TOPOFF_THRES = 4050;  
        }
    }
    else if(BMT_status.temperature >= TEMP_POS_10_THRESHOLD)  
    {
        if( ((g_temp_status == TEMP_POS_45_TO_POS_60) && (BMT_status.temperature >= TEMP_POS_45_THRES_MINUS_X_DEGREE)) ||
            ((g_temp_status == TEMP_POS_0_TO_POS_10 ) && (BMT_status.temperature <= TEMP_POS_10_THRES_PLUS_X_DEGREE ))    ) 
        {
            xlog_printk(ANDROID_LOG_WARN, "Power/Battery", "[BATTERY] Battery Temperature not recovery to normal temperature charging mode yet!!\n\r");     
        }
        else
        {
            if(Enable_BATDRV_LOG >=1)
            {
                xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] Battery Normal Temperature between %d and %d !!\n\r",
                    TEMP_POS_10_THRESHOLD, TEMP_POS_45_THRESHOLD); 
            }                
            g_temp_status = TEMP_POS_10_TO_POS_45;
            g_jeita_recharging_voltage = 4100;
            V_CC2TOPOFF_THRES = 4050;  
        }
    }
    else if(BMT_status.temperature >= TEMP_POS_0_THRESHOLD)  
    {
        if((g_temp_status == TEMP_NEG_10_TO_POS_0 || g_temp_status == TEMP_BELOW_NEG_10) && (BMT_status.temperature <= TEMP_POS_0_THRES_PLUS_X_DEGREE))
        {
			if (g_temp_status == TEMP_NEG_10_TO_POS_0) {
            xlog_printk(ANDROID_LOG_WARN, "Power/Battery", "[BATTERY] Battery Temperature between %d and %d !!\n\r",
                TEMP_POS_0_THRES_PLUS_X_DEGREE, TEMP_POS_10_THRESHOLD); 
			}
			if (g_temp_status == TEMP_BELOW_NEG_10) {
				xlog_printk(ANDROID_LOG_WARN, "Power/Battery", "[BATTERY] Battery Temperature between %d and %d,not allow charging yet!!\n\r",
					TEMP_POS_0_THRESHOLD,TEMP_POS_0_THRES_PLUS_X_DEGREE); 
				return PMU_STATUS_FAIL; 
			}
        }
        else {
            xlog_printk(ANDROID_LOG_WARN, "Power/Battery", "[BATTERY] Battery Temperature between %d and %d !!\n\r",
                TEMP_POS_0_THRESHOLD,TEMP_POS_10_THRESHOLD); 
            g_temp_status = TEMP_POS_0_TO_POS_10;
            g_jeita_recharging_voltage = 4000;
            V_CC2TOPOFF_THRES = 4050; 
        }
    }
    else if(BMT_status.temperature >= TEMP_NEG_10_THRESHOLD)
    {
        if((g_temp_status == TEMP_BELOW_NEG_10) && (BMT_status.temperature <= TEMP_NEG_10_THRES_PLUS_X_DEGREE))
        {
            xlog_printk(ANDROID_LOG_WARN, "Power/Battery", "[BATTERY] Battery Temperature between %d and %d,not allow charging yet!!\n\r",
                TEMP_NEG_10_THRESHOLD, TEMP_NEG_10_THRES_PLUS_X_DEGREE); 
            return PMU_STATUS_FAIL; 
        }else{
            xlog_printk(ANDROID_LOG_WARN, "Power/Battery", "[BATTERY] Battery Temperature between %d and %d !!\n\r",
                TEMP_NEG_10_THRESHOLD,TEMP_POS_0_THRESHOLD); 
            g_temp_status = TEMP_NEG_10_TO_POS_0;
            g_jeita_recharging_voltage = 3800;
            V_CC2TOPOFF_THRES = 3850;  
        }
    }
    else
    {
        xlog_printk(ANDROID_LOG_WARN, "Power/Battery", "[BATTERY] Battery below low Temperature(%d) !!\n\r", TEMP_NEG_10_THRESHOLD);  
        g_temp_status = TEMP_BELOW_NEG_10;
        return PMU_STATUS_FAIL; 
    }

    return 0;
}
#endif


#ifdef VENDOR_EDIT//Nanwei.Deng@BasicDrv.CHG, 2012/03/08, Add for Battery encryption
extern signed int g_true_vol;
extern signed int g_charger_type;
#endif

#ifdef OPPO_NEW_CHARGING
void oppo_get_tbat_at_turnoff(void)
{
    int bat_temperature_volt=0;

    int fg_r_value=0;
    kal_int32 fg_current_temp=0;
    kal_bool fg_current_state;
    int bat_temperature_volt_temp=0;
    
    /* Get V_BAT_Temperature */
    bat_temperature_volt = get_tbat_volt(5);
    oppo_chg_log("==========>bat_temperature_volt=%d \r\n",bat_temperature_volt);
    if(bat_temperature_volt == 0)
    {
        BMT_status.temperature = g_bat_temperature_pre;
        if (Enable_BATDRV_LOG == 1) {
            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] Warning !! bat_temperature_volt == 0, restore temperature value\n\r");
        }
    }
    else
    {
        BMT_status.temperature = BattVoltToTemp(bat_temperature_volt);
        g_bat_temperature_pre = BMT_status.temperature;     
    }
    if( (g_battery_tt_check_flag==0) && (BMT_status.temperature<60) && (BMT_status.temperature>(-20)) )
    {
        g_battery_thermal_throttling_flag=3;
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] Enable battery TT (%d)\n\r", BMT_status.temperature);
        g_battery_tt_check_flag=1;
    }    

}
#endif

int BAT_CheckBatteryStatus(void)
{
    int BAT_status = PMU_STATUS_OK;
    int i = 0;

    /* Get Battery Information */
    BAT_GetVoltage();

    #if 0//def OPPO_USE_EXCHARGER
    if( upmu_is_chr_det() == KAL_TRUE)
    {
        if(BQ24156A_ID == oppo_excharger_id)
        {
            check_bq24156a_status();
        }
        else if(BQ24157_ID == oppo_excharger_id)
        {
            check_bq24156a_status();
        }
        else if(FAN5405_ID == oppo_excharger_id)
        {
            check_fan5405_status();
        }
    }
    #endif

    /*Charging 9s and discharging 1s : start*/    
    if( (upmu_is_chr_det() == KAL_TRUE) && 
        (g_HW_Charging_Done == 0) &&
		#ifndef OPPO_NEW_CHARGING
        (BMT_status.bat_charging_state != CHR_ERROR) &&
        (BMT_status.bat_charging_state != CHR_TOP_OFF)) 
		#else
		(BMT_status.bat_charging_state != CHR_ERROR)) 
		#endif
    {
        g_HW_stop_charging = 1;
    
        if (Enable_BATDRV_LOG == 1) {
            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] Dis Charging 1s\n\r");
        }
        
        pchr_turn_off_charging();
        getVoltFlag = 1;        

        #ifdef OPPO_NEW_CHARGING
        //debug for be at a standstill in ipod chargering display 
        if((1 ==oppo_ipod_status) && (1 == oppo_chg_lcd_is_on)) msleep(50);
        else
        #endif
        msleep(1000);        
    }

	#ifndef OPPO_NEW_BAT_VOL_ALG
    BMT_status.ADC_BAT_SENSE = get_bat_sense_volt(1);
    BMT_status.bat_vol = BMT_status.ADC_BAT_SENSE;    
	#else
	static kal_uint8 bat_vol_first_sample_fg = OPPO_BAT_VOL_DISCARD_TIME;

	static kal_int32 OPPO_FIRST_VOL;

	if(bat_vol_first_sample_fg == OPPO_BAT_VOL_DISCARD_TIME)
	{
		bat_vol_first_sample_fg--;

		pchr_turn_off_charging();
	
		if(g_true_vol==0)
		{
	        if(upmu_is_chr_det()==KAL_TRUE)
	        {
	            BMT_status.bat_vol = oppo_get_first_bat_vol() + OPPO_BAT_VOL_CHG_PWRON_FIRST_OFFSET;
	        }
	        else
	        {  
	            BMT_status.bat_vol = oppo_get_first_bat_vol() + OPPO_BAT_VOL_FIRST_OFFSET;
	        }
		}
		else
		{
			BMT_status.bat_vol =g_true_vol;
			g_true_vol=0;
		}
	}
	else if(bat_vol_first_sample_fg > 0)
	{
	    bat_vol_first_sample_fg--;
	    //BMT_status.ADC_BAT_SENSE = PMIC_IMM_GetOneChannelValue(AUXADC_BATTERY_VOLTAGE_CHANNEL,1);
	    //BMT_status.bat_vol = BMT_status.ADC_BAT_SENSE + OPPO_BAT_VOL_OPEN_OFFSET;

	    //Jiada.Li@BasicDrv.CHG, 2013/06/05, debug for QE'test
	    over_temper_count = 0;
	    low_temper_count = 0;
	    
	}
	else
	{
    	BMT_status.ADC_BAT_SENSE = PMIC_IMM_GetOneChannelValue(AUXADC_BATTERY_VOLTAGE_CHANNEL,1,1);
    	//BMT_status.ADC_BAT_SENSE += V_compensate_EVB;	
    	if(upmu_is_chr_det() == KAL_TRUE)
    	{
    	    BMT_status.bat_vol = BMT_status.ADC_BAT_SENSE + OPPO_BAT_VOL_CHG_IN_OFFSET;
    	}
    	else
    	{
        	if(1 == oppo_chg_lcd_is_on)
        	{
        	    BMT_status.bat_vol = BMT_status.ADC_BAT_SENSE + OPPO_BAT_VOL_LCD_ON_OFFSET;
        	}
        	else
        	{
        	    BMT_status.bat_vol = BMT_status.ADC_BAT_SENSE + OPPO_BAT_VOL_LCD_OFF_OFFSET;
        	}    	
    	}
	}
	printk("true_bat_vol=%d \r\n", BMT_status.bat_vol);
	#endif
	#ifdef OPPO_NEW_CHARGING
	oppo_check_vbat_off_vol = BMT_status.bat_vol;
	oppo_check_charger_off_vol = PMIC_IMM_GetOneChannelValue(AUXADC_CHARGER_VOLTAGE_CHANNEL,5,1);
	oppo_check_charger_off_vol = oppo_check_charger_off_vol / 100;
	#endif
	#ifndef OPPO_NEW_CHARGING
    if ( BMT_status.bat_charging_state != CHR_TOP_OFF)
	#endif
    {
        gADC_BAT_SENSE_temp = (int)BMT_status.bat_vol;
        gADC_I_SENSE_temp = get_i_sense_volt(1);

        //workaround
        gADC_BAT_SENSE_temp = get_bat_sense_volt(1);
        gADC_I_SENSE_temp   = get_i_sense_volt(1);
        gADC_BAT_SENSE_temp = get_bat_sense_volt(1);
        gADC_I_SENSE_temp   = get_i_sense_volt(1);
        gADC_BAT_SENSE_temp = get_bat_sense_volt(1);
        gADC_I_SENSE_temp   = get_i_sense_volt(1);
        gADC_BAT_SENSE_temp = get_bat_sense_volt(1);
        gADC_I_SENSE_temp   = get_i_sense_volt(1);
        
        if (Enable_BATDRV_LOG == 1) {            
            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] gADC_BAT_SENSE_temp=%d, gADC_I_SENSE_temp=%d\n\r", gADC_BAT_SENSE_temp, gADC_I_SENSE_temp);
        }
        gADC_I_SENSE_offset = gADC_BAT_SENSE_temp - gADC_I_SENSE_temp;
    }
        
    g_TempBattVoltage = BMT_status.bat_vol;
    if ( getVoltFlag == 1 )
    {
        if (Enable_BATDRV_LOG == 1) {            
            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] Charging 9s\n\r");
        }        
        getVoltFlag = 0;
    }    
    #ifdef OPPO_NEW_CHARGING
    oppo_get_tbat_at_turnoff();
    #endif
    /*Charging 9s and discharging 1s : end*/

    /*Use no gas gauge*/
    if( gForceADCsolution == 1 )
    {
        /* Re-calculate Battery Percentage (SOC) */    
        BMT_status.SOC = BattVoltToPercent(BMT_status.bat_vol);
                
        /* User smooth View when discharging : start */
        if( upmu_is_chr_det() == KAL_FALSE )
        {                
#if defined(MTK_JEITA_STANDARD_SUPPORT)
            if (BMT_status.bat_vol >= g_jeita_recharging_voltage)
#else        	
			#ifndef OPPO_NEW_BAT_VOL_ALG
            if (BMT_status.bat_vol >= RECHARGING_VOLTAGE) 
			#else
			if (BMT_status.bat_vol >= OPPO_BAT_VOLT_PERCENT100)
			#endif
#endif    
            {               
                BMT_status.SOC = 100;    
                BMT_status.bat_full = KAL_TRUE;
            }        
        }        
        if (bat_volt_cp_flag == 0) 
        {
            bat_volt_cp_flag = 1;        
            bat_volt_check_point = BMT_status.SOC;
        }
        /* User smooth View when discharging : end */

        /**************** Averaging : START ****************/        
        if (!batteryBufferFirst)
        {
            if(BMT_status.bat_vol != 0)
            {
                batteryBufferFirst = KAL_TRUE;
                
                for (i=0; i<BATTERY_AVERAGE_SIZE; i++) {
                    batteryVoltageBuffer[i] = BMT_status.bat_vol;            
                    batteryCurrentBuffer[i] = BMT_status.ICharging;            
                    batterySOCBuffer[i] = BMT_status.SOC;
                }

                batteryVoltageSum = BMT_status.bat_vol * BATTERY_AVERAGE_SIZE;
                batteryCurrentSum = BMT_status.ICharging * BATTERY_AVERAGE_SIZE;        
                batterySOCSum = BMT_status.SOC * BATTERY_AVERAGE_SIZE;
            }
            else
            {
                if (Enable_BATDRV_LOG == 1) {            
                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] BMT_status.bat_vol == 0, init avg array again.\n\r");
                }
            }
        }

        batteryVoltageSum -= batteryVoltageBuffer[batteryIndex];
        batteryVoltageSum += BMT_status.bat_vol;
        batteryVoltageBuffer[batteryIndex] = BMT_status.bat_vol;

        batteryCurrentSum -= batteryCurrentBuffer[batteryIndex];
        batteryCurrentSum += BMT_status.ICharging;
        batteryCurrentBuffer[batteryIndex] = BMT_status.ICharging;
        
	    #if !defined(OPPO_NEW_CHARGING) || !defined(OPPO_NEW_CHARGE_POSTFULL)
        if (BMT_status.bat_full)
            BMT_status.SOC = 100;
        if (g_bat_full_user_view)
            BMT_status.SOC = 100;
	    #endif
        batterySOCSum -= batterySOCBuffer[batteryIndex];
        batterySOCSum += BMT_status.SOC;
        batterySOCBuffer[batteryIndex] = BMT_status.SOC;
        
        BMT_status.bat_vol = batteryVoltageSum / BATTERY_AVERAGE_SIZE;
        BMT_status.ICharging = batteryCurrentSum / BATTERY_AVERAGE_SIZE;    
        BMT_status.SOC = batterySOCSum / BATTERY_AVERAGE_SIZE;

        batteryIndex++;
        if (batteryIndex >= BATTERY_AVERAGE_SIZE)
            batteryIndex = 0;
        /**************** Averaging : END ****************/
        
        if( BMT_status.SOC == 100 ) {
            BMT_status.bat_full = KAL_TRUE;   
        }
    }
    /*Use gas gauge*/
    else
    {
        /* Re-calculate Battery Percentage (SOC) */    
        BMT_status.SOC = FGADC_Get_BatteryCapacity_CoulombMothod();

        /* Sync FG's percentage */
        if(gSyncPercentage==0)
        {            
            if( (upmu_is_chr_det()==KAL_TRUE) && (!g_Battery_Fail) && (g_Charging_Over_Time==0))
            {
                /* SOC only UP when charging */
                if ( BMT_status.SOC > bat_volt_check_point ) {                        
                    bat_volt_check_point = BMT_status.SOC;
                }
            }
            else
            {
                /* SOC only Done when dis-charging */
                if ( BMT_status.SOC < bat_volt_check_point ) {
                    bat_volt_check_point = BMT_status.SOC;
                }
            }                    
        }

        /**************** Averaging : START ****************/
        if (!batteryBufferFirst)
        {
            batteryBufferFirst = KAL_TRUE;
            
            for (i=0; i<BATTERY_AVERAGE_SIZE; i++) {
                batteryVoltageBuffer[i] = BMT_status.bat_vol;
                batteryCurrentBuffer[i] = BMT_status.ICharging;
                batteryTempBuffer[i] = BMT_status.temperature;
            }

            batteryVoltageSum = BMT_status.bat_vol * BATTERY_AVERAGE_SIZE;
            batteryCurrentSum = BMT_status.ICharging * BATTERY_AVERAGE_SIZE;
            batteryTempSum = BMT_status.temperature * BATTERY_AVERAGE_SIZE;
        }

        if( (batteryCurrentSum==0) && (BMT_status.ICharging!=0) )
        {
            for (i=0; i<BATTERY_AVERAGE_SIZE; i++) {            
                batteryCurrentBuffer[i] = BMT_status.ICharging;
            }
            batteryCurrentSum = BMT_status.ICharging * BATTERY_AVERAGE_SIZE;
        }

        batteryVoltageSum -= batteryVoltageBuffer[batteryIndex];
        batteryVoltageSum += BMT_status.bat_vol;
        batteryVoltageBuffer[batteryIndex] = BMT_status.bat_vol;

        batteryCurrentSum -= batteryCurrentBuffer[batteryIndex];
        batteryCurrentSum += BMT_status.ICharging;
        batteryCurrentBuffer[batteryIndex] = BMT_status.ICharging;

        batteryTempSum -= batteryTempBuffer[batteryIndex];
        batteryTempSum += BMT_status.temperature;
        batteryTempBuffer[batteryIndex] = BMT_status.temperature;
    
        BMT_status.bat_vol = batteryVoltageSum / BATTERY_AVERAGE_SIZE;
        BMT_status.ICharging = batteryCurrentSum / BATTERY_AVERAGE_SIZE;
        #ifndef OPPO_NEW_CHARGING
        BMT_status.temperature = batteryTempSum / BATTERY_AVERAGE_SIZE;
        #endif

        batteryIndex++;
        if (batteryIndex >= BATTERY_AVERAGE_SIZE)
            batteryIndex = 0;
        /**************** Averaging : END ****************/
    }

    //if (Enable_BATDRV_LOG >= 1) {
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY:AVG] BatTemp:%d Vbat:%d VBatSen:%d SOC:%d ChrDet:%d Vchrin:%d Icharging:%d ChrType:%d USBstate:%d gADC_I_SENSE_offset:%d gForceADCsolution=%d\r\n", 
           BMT_status.temperature ,BMT_status.bat_vol, BMT_status.ADC_BAT_SENSE, BMT_status.SOC, 
           upmu_is_chr_det(), BMT_status.charger_vol, BMT_status.ICharging, CHR_Type_num, g_usb_state, gADC_I_SENSE_offset, gForceADCsolution ); 
    //}           

    #ifndef OPPO_NO_USE_FGADC
    if (Enable_BATDRV_LOG == 1) {
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY:FG] %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n", 
           BMT_status.temperature ,BMT_status.bat_vol, BMT_status.ADC_BAT_SENSE, BMT_status.SOC, 
           upmu_is_chr_det(), BMT_status.charger_vol, BMT_status.ICharging, CHR_Type_num,
           FGADC_Get_BatteryCapacity_CoulombMothod(), FGADC_Get_BatteryCapacity_VoltageMothod(), BATTERY_AVERAGE_SIZE );

        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[PMIC_ADC] Bank0[0x31]=0x%x, Bank0[0x38]=0x%x\r\n", 
        upmu_get_reg_value(0x31), upmu_get_reg_value(0x38));
    }
    #endif

    /* Protection Check : start*/
    BAT_status = BAT_CheckPMUStatusReg();
    #ifndef OPPO_NEW_CHARGING
    if(BAT_status != PMU_STATUS_OK)
        return PMU_STATUS_FAIL;                  

    if(battery_cmd_thermal_test_mode == 1){
        BMT_status.temperature = battery_cmd_thermal_test_mode_value;
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] In thermal_test_mode 1, Tbat=%d\n", BMT_status.temperature);
    }

#if defined(MTK_JEITA_STANDARD_SUPPORT)

    if (Enable_BATDRV_LOG == 1) {
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] support JEITA, Tbat=%d\n", BMT_status.temperature);            
    }

    if( do_jeita_state_machine() == PMU_STATUS_FAIL)
    {
        printk(  "[BATTERY] JEITA : fail\n");
        BMT_status.bat_charging_state = CHR_ERROR;
        return PMU_STATUS_FAIL;
    }

#else

    #if (BAT_TEMP_PROTECT_ENABLE == 1)
    if ((BMT_status.temperature < MIN_CHARGE_TEMPERATURE) || 
        (BMT_status.temperature == ERR_CHARGE_TEMPERATURE))
    {
        xlog_printk(ANDROID_LOG_WARN, "Power/Battery", "[BATTERY] Battery Under Temperature or NTC fail !!\n\r");                
        BMT_status.bat_charging_state = CHR_ERROR;
        return PMU_STATUS_FAIL;       
    }
    #endif                
    if (BMT_status.temperature >= MAX_CHARGE_TEMPERATURE)
    {
        xlog_printk(ANDROID_LOG_WARN, "Power/Battery", "[BATTERY] Battery Over Temperature !!\n\r");                
        BMT_status.bat_charging_state = CHR_ERROR;
        return PMU_STATUS_FAIL;       
    }    

#endif

    if( upmu_is_chr_det() == KAL_TRUE)
    {
        #if (V_CHARGER_ENABLE == 1)
        if (BMT_status.charger_vol <= V_CHARGER_MIN )
        {
            xlog_printk(ANDROID_LOG_WARN, "Power/Battery", "[BATTERY]Charger under voltage!!\r\n");                    
            BMT_status.bat_charging_state = CHR_ERROR;
            return PMU_STATUS_FAIL;        
        }
        #endif        
        if ( BMT_status.charger_vol >= V_CHARGER_MAX )
        {
            xlog_printk(ANDROID_LOG_WARN, "Power/Battery", "[BATTERY]Charger over voltage !!\r\n");                    
            BMT_status.charger_protect_status = charger_OVER_VOL;
            BMT_status.bat_charging_state = CHR_ERROR;
            return PMU_STATUS_FAIL;        
        }            
    }
    /* Protection Check : end*/

    if( upmu_is_chr_det() == KAL_TRUE)
    {        
    	       
#if defined(MTK_JEITA_STANDARD_SUPPORT)
        if((BMT_status.bat_vol < g_jeita_recharging_voltage) && (BMT_status.bat_full) && (g_HW_Charging_Done == 1) && (!g_Battery_Fail) )    
#else         
        if((BMT_status.bat_vol < RECHARGING_VOLTAGE) && (BMT_status.bat_full) && (g_HW_Charging_Done == 1) && (!g_Battery_Fail) )    
#endif        	
        {
            if (Enable_BATDRV_LOG == 1) {
                xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] Battery Re-charging !!\n\r");                
            }
            BMT_status.bat_full = KAL_FALSE;    
            g_bat_full_user_view = KAL_TRUE;
            BMT_status.bat_charging_state = CHR_CC;

            g_HW_Charging_Done = 0;
            g_Calibration_FG = 0;            

            if (Enable_BATDRV_LOG == 1) {
                xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] Battery Re-charging. Call FGADC_Reset_SW_Parameter.\n\r");    
            }
            FGADC_Reset_SW_Parameter();
        }        
    }
    #else/* OPPO_NEW_CHARGING */ 
    OPPO_BAT_PhyCheck();
    #endif/* OPPO_NEW_CHARGING */ 
        
    return PMU_STATUS_OK;
}

PMU_STATUS BAT_BatteryStatusFailAction(void)
{
    if (Enable_BATDRV_LOG == 1) {
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] BAD Battery status... Charging Stop !!\n\r");            
    }

#if defined(MTK_JEITA_STANDARD_SUPPORT)
    if((g_temp_status == TEMP_ABOVE_POS_60) ||(g_temp_status == TEMP_BELOW_NEG_10))
    {
        temp_error_recovery_chr_flag=KAL_FALSE;
    }	
    if((temp_error_recovery_chr_flag==KAL_FALSE) && (g_temp_status != TEMP_ABOVE_POS_60) && (g_temp_status != TEMP_BELOW_NEG_10))
    {
        temp_error_recovery_chr_flag=KAL_TRUE;
        BMT_status.bat_charging_state=CHR_PRE;
    }
#endif

    BMT_status.total_charging_time = 0;
    BMT_status.PRE_charging_time = 0;
    BMT_status.CC_charging_time = 0;
    BMT_status.TOPOFF_charging_time = 0;
    BMT_status.POSTFULL_charging_time = 0;
    post_charging_time=0;

    /*  Disable charger */
    pchr_turn_off_charging();

    return PMU_STATUS_OK;
}

PMU_STATUS BAT_ChargingOTAction(void)
{    
    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] Charging over %d hr stop !!\n\r", MAX_CHARGING_TIME);            
 
    //BMT_status.bat_full = KAL_TRUE;
    BMT_status.total_charging_time = 0;
    BMT_status.PRE_charging_time = 0;
    BMT_status.CC_charging_time = 0;
    BMT_status.TOPOFF_charging_time = 0;
    BMT_status.POSTFULL_charging_time = 0;
    
    g_HW_Charging_Done = 1;    
    g_Charging_Over_Time = 1;

    /*  Disable charger*/
    pchr_turn_off_charging();

    return PMU_STATUS_OK;
}

extern void fg_qmax_update_for_aging(void);

PMU_STATUS BAT_BatteryFullAction(void)
{
    if (Enable_BATDRV_LOG == 1) {    
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] Battery full !!\n\r");            
    }
    
    BMT_status.bat_full = KAL_TRUE;
    BMT_status.total_charging_time = 0;
    BMT_status.PRE_charging_time = 0;
    BMT_status.CC_charging_time = 0;
    BMT_status.TOPOFF_charging_time = 0;
    BMT_status.POSTFULL_charging_time = 0;
    
    g_HW_Charging_Done = 1;
	#ifndef OPPO_NO_USE_FGADC
    fg_qmax_update_for_aging();
    g_Calibration_FG = 1;
    if(gFG_can_reset_flag == 1)
    {
        gFG_can_reset_flag = 0;

        if (Enable_BATDRV_LOG == 1) {
            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] Battery real full. Call FGADC_Reset_SW_Parameter.\n");
        }
        FGADC_Reset_SW_Parameter();
    }
    g_Calibration_FG = 0;
	#endif

    /*  Disable charger */
    pchr_turn_off_charging();

    #ifdef OPPO_NEW_CHARGING
    g_BatteryNotifyCode |= 1 << NOTIFY_BAT_FULL;

    if(SUPER_LOW_CHARGE_TEMP_STATUS == oppo_charge_temp_status)
    {
        g_BatteryNotifyCode |= 1 << NOTIFY_LOWTEMP_BAT_FULL;
    }
    else if(HIGH_CHARGE_TEMP_STATUS == oppo_charge_temp_status)
    {
        g_BatteryNotifyCode |= 1 << NOTIFY_HIGHTEMP_BAT_FULL;
    }
    #ifdef OPPO_HIGH_BATTERY_VOLTAGE
    else if(oppo_high_battery_status == 0)
    {
        g_BatteryNotifyCode |= 1 << NOTIFY_THIRD_BAT_FULL;
    }
    #endif
    else
    {
        g_BatteryNotifyCode |= 1 << NOTIFY_NORMAL_BAT_FULL;
    }
    
    //if((BMT_status.bat_vol < RECHARGING_VOLTAGE) && (BMT_status.bat_full) && (g_HW_Charging_Done == 1) && (!g_Battery_Fail) )	
    if(oppo_check_vbat_off_vol < oppo_charge_to_recharging_threshold)//(BMT_status.bat_vol < oppo_charge_to_recharging_threshold)
    {
        if (Enable_BATDRV_LOG == 1) {
            #ifdef OPPO_KERNEL_CHARGING
        	xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] Battery Re-charging !!\n\r");  
        	#elif defined(OPPO_UBOOT_CHARGING)
        	printf("[BATTERY] Battery Re-charging !!\n\r"); 
        	#endif
        }
        BMT_status.bat_full = KAL_FALSE;    
        g_bat_full_user_view = KAL_TRUE;
        g_BatteryNotifyCode |= 1 << NOTIFY_BAT_FULL;

        if(SUPER_LOW_CHARGE_TEMP_STATUS == oppo_charge_temp_status)
        {
            g_BatteryNotifyCode |= 1 << NOTIFY_LOWTEMP_BAT_FULL;
        }
        else if(HIGH_CHARGE_TEMP_STATUS == oppo_charge_temp_status)
        {
            g_BatteryNotifyCode |= 1 << NOTIFY_HIGHTEMP_BAT_FULL;
        }
        #ifdef OPPO_HIGH_BATTERY_VOLTAGE
        else if(oppo_high_battery_status == 0)
        {
            g_BatteryNotifyCode |= 1 << NOTIFY_THIRD_BAT_FULL;
        }
        #endif
        else
        {
            g_BatteryNotifyCode |= 1 << NOTIFY_NORMAL_BAT_FULL;
        }
        
        //g_NotifyFlag = NOTIFY_BAT_FULL;
		BMT_status.bat_charging_state = CHR_CC;

		g_HW_Charging_Done = 0;
    }
    #endif/* OPPO_NEW_CHARGING */ 
    gSyncPercentage=1;
    
    return PMU_STATUS_OK;
}

PMU_STATUS BAT_PreChargeModeAction(void)
{
    if (Enable_BATDRV_LOG == 1) {
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] Pre-CC mode charge, timer=%ld on %ld !!\n\r",
        BMT_status.PRE_charging_time, BMT_status.total_charging_time);    
    }

    BMT_status.PRE_charging_time += BAT_TASK_PERIOD;
    BMT_status.CC_charging_time = 0;
    BMT_status.TOPOFF_charging_time = 0;
    BMT_status.total_charging_time += BAT_TASK_PERIOD;
    post_charging_time=0;                    

    /*  Enable charger */
    pchr_turn_on_charging();            

	#ifndef OPPO_NEW_CHARGING
    if ( BMT_status.bat_vol > V_PRE2CC_THRES )
	#else
	#ifdef OPPO_HIGH_BATTERY_VOLTAGE
    if ( BMT_status.bat_vol > HI_BAT_PRE_TO_FULL_CHARGNIG_THRESHOLD )
	{
	    BMT_status.bat_charging_state = CHR_BATFULL;
	}
    else if(( BMT_status.bat_vol > PRE_TO_FULL_CHARGNIG_THRESHOLD ) && ( oppo_high_battery_status == 0))
    #else
	if ( BMT_status.bat_vol > PRE_TO_FULL_CHARGNIG_THRESHOLD )
    #endif
	{
	    BMT_status.bat_charging_state = CHR_BATFULL;
	}
	else if ( BMT_status.bat_vol > V_PRE2CC_THRES )
	#endif
    {
        BMT_status.bat_charging_state = CHR_CC;
    }

    return PMU_STATUS_OK;        
} 

PMU_STATUS BAT_ConstantCurrentModeAction(void)
{
    int i=0;

    if (Enable_BATDRV_LOG == 1) {
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] CC mode charge, timer=%ld on %ld !!\n\r",
        BMT_status.CC_charging_time, BMT_status.total_charging_time);    
    }

    BMT_status.PRE_charging_time = 0;
    BMT_status.CC_charging_time += BAT_TASK_PERIOD;
    BMT_status.TOPOFF_charging_time = 0;
    BMT_status.total_charging_time += BAT_TASK_PERIOD;
    post_charging_time=0;                    

    /*  Enable charger */
    pchr_turn_on_charging();     

	#ifndef OPPO_NEW_CHARGING
    if ( BMT_status.bat_vol > V_CC2TOPOFF_THRES )
	#else
	if(BMT_status.bat_vol > oppo_charge_to_topoff_threshold)
	#endif
    {
        BMT_status.bat_charging_state = CHR_TOP_OFF;
    
        if(g_temp_CC_value == AC_CHARGER_CURRENT)
        {
            for (i=0; i<BATTERY_AVERAGE_SIZE; i++) {                
                batteryCurrentBuffer[i] = 650;
            }
            batteryCurrentSum = 650 * BATTERY_AVERAGE_SIZE;
        }
        else
        {
            for (i=0; i<BATTERY_AVERAGE_SIZE; i++) {                
                batteryCurrentBuffer[i] = 450;
            }
            batteryCurrentSum = 450 * BATTERY_AVERAGE_SIZE;
        }                
    }

    return PMU_STATUS_OK;        
}    


PMU_STATUS BAT_TopOffModeAction(void)
{
    if (Enable_BATDRV_LOG == 1) {
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] Top Off mode charge, timer=%ld on %ld !!\n\r",
        BMT_status.TOPOFF_charging_time, BMT_status.total_charging_time);    
    }

    BMT_status.PRE_charging_time = 0;
    BMT_status.CC_charging_time = 0;
	#ifndef OPPO_NEW_BAT_VOL_ALG
    BMT_status.TOPOFF_charging_time += BAT_TASK_PERIOD;
	#else
	BMT_status.TOPOFF_charging_time = 0;
	#endif
    BMT_status.total_charging_time += BAT_TASK_PERIOD;                    

    /*  Enable charger */
    pchr_turn_on_charging();

	#if defined(OPPO_NEW_CHARGING) && defined(OPPO_NEW_CHARGE_POSTFULL)
	static kal_uint8 check_postfull_cnt=0;
	
    if(/*(BMT_status.TOPOFF_charging_time > 60) && */(BMT_status.ICharging <= OPPO_CHARGING_POSTFULL_CURRENT))
    {
        if(Enable_BATDRV_LOG == 1)
        {
            #ifdef OPPO_KERNEL_CHARGING
            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] Battery postfull on %ld mA \n", BMT_status.ICharging);
            #elif defined(OPPO_UBOOT_CHARGING)
            printf("[BATTERY] Battery postfull on %ld mA \n", BMT_status.ICharging);
            #endif
        }
        BMT_status.bat_charging_state = CHR_POST_FULL;
        return;
    }

    /*if(BMT_status.TOPOFF_charging_time >= OPPO_MAX_CV_CHARGING_TIME)
    {
        if(Enable_BATDRV_LOG == 1)
        {
            #ifdef OPPO_KERNEL_CHARGING
            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "BMT_status.TOPOFF_charging_time >= %d \r\n", OPPO_MAX_CV_CHARGING_TIME);
            #elif defined(OPPO_UBOOT_CHARGING)
            printf("BMT_status.TOPOFF_charging_time >= %d \r\n", OPPO_MAX_CV_CHARGING_TIME);
            #endif
        }
        
        BMT_status.bat_charging_state = CHR_POST_FULL;									
        return;
    }*/

    if(BMT_status.bat_vol >= oppo_charge_to_postfull_threshold)
    {
        check_postfull_cnt++;
    }
    else
    {
        check_postfull_cnt = 0;
    }
    if(OPPO_VBAT_POSTFULL_COUNT < check_postfull_cnt)
    {
        if(Enable_BATDRV_LOG == 1)
        {
            #ifdef OPPO_KERNEL_CHARGING
            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "BMT_status.OPPO_VBAT_POSTFULL_COUNT >= %d \r\n", OPPO_VBAT_POSTFULL_COUNT);
            #elif defined(OPPO_UBOOT_CHARGING)
            printf("BMT_status.OPPO_VBAT_POSTFULL_COUNT >= %d \r\n", OPPO_VBAT_POSTFULL_COUNT);
            #endif
        }
        
        BMT_status.bat_charging_state = CHR_POST_FULL;										
        return;    
    }
	#endif/* OPPO_NEW_CHARGING */ 

	//g_sw_cv_enable=1;

//lijiada need check	upmu_chr_hw_cv_en(1); // RG_HWCV_EN
	
    return PMU_STATUS_OK;        
} 

#ifdef OPPO_NEW_CHARGING
int POSTFULL_safety_timer=0;

PMU_STATUS BAT_PostFullModeAction(void)
{
	#if defined(OPPO_NEW_CHARGING) && defined(OPPO_NEW_CHARGE_POSTFULL)
    if (Enable_BATDRV_LOG == 1) {
    	xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] Top Off mode charge, timer=%ld on %ld !!\n\r",
    	BMT_status.TOPOFF_charging_time, BMT_status.total_charging_time);    
    }

    BMT_status.PRE_charging_time = 0;
	BMT_status.CC_charging_time = 0;
	BMT_status.TOPOFF_charging_time += BAT_TASK_PERIOD;
    BMT_status.total_charging_time += BAT_TASK_PERIOD;                    

	/*  Enable charger */
	pchr_turn_on_charging();


    if(BMT_status.bat_vol >= OPPO_BAT_VOLT_PERCENT100)
    {
    	g_bat_full_user_view = KAL_TRUE;
    	g_BatteryNotifyCode |= 1 << NOTIFY_BAT_POSTFULL;
    	//g_NotifyFlag = NOTIFY_BAT_POSTFULL;
	}
	
	static kal_uint8 check_full_cnt=0;
	
    if(BMT_status.ICharging <= OPPO_CHARGING_FULL_CURRENT)
    {
        if(Enable_BATDRV_LOG == 1)
        {
            #ifdef OPPO_KERNEL_CHARGING
            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] Battery real full and disable charging on %ld mA \n", BMT_status.ICharging);
            #elif defined(OPPO_UBOOT_CHARGING)
            printf("[BATTERY] Battery real full and disable charging on %ld mA \n", BMT_status.ICharging);
            #endif
        }
        BMT_status.bat_charging_state = CHR_BATFULL;
        //BAT_BatteryFullAction();
        return;
    }

    if(BMT_status.TOPOFF_charging_time >= OPPO_MAX_CV_CHARGING_TIME)
    {
        if(Enable_BATDRV_LOG == 1)
        {
            #ifdef OPPO_KERNEL_CHARGING
            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "BMT_status.TOPOFF_charging_time >= %d \r\n", OPPO_MAX_CV_CHARGING_TIME);
            #elif defined(OPPO_UBOOT_CHARGING)
            printf("BMT_status.TOPOFF_charging_time >= %d \r\n", OPPO_MAX_CV_CHARGING_TIME);
            #endif
        }
        
        BMT_status.bat_charging_state = CHR_BATFULL;
        //BAT_BatteryFullAction();										
        return;
    }

    if(BMT_status.bat_vol >= oppo_charge_to_full_threshold)
    {
        check_full_cnt++;
    }
    else
    {
        check_full_cnt = 0;
    }
    if(OPPO_VBAT_FULL_COUNT < check_full_cnt)
    {
        if(Enable_BATDRV_LOG == 1)
        {
            #ifdef OPPO_KERNEL_CHARGING
            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "BMT_status.OPPO_VBAT_FULL_COUNT >= %d \r\n", OPPO_VBAT_FULL_COUNT);
            #elif defined(OPPO_UBOOT_CHARGING)
            printf("BMT_status.OPPO_VBAT_FULL_COUNT >= %d \r\n", OPPO_VBAT_FULL_COUNT);
            #endif
        }
        
        BMT_status.bat_charging_state = CHR_BATFULL;
        //BAT_BatteryFullAction();										
        return;    
    }

//lijiada need check	upmu_chr_hw_cv_en(1); // RG_HWCV_EN

	#endif/* OPPO_NEW_CHARGING */

	//g_sw_cv_enable=0;

    return PMU_STATUS_OK;        
} 
#endif

#ifndef OPPO_NEW_CHARGING
void mt_battery_notify_check(void)
{
    g_BatteryNotifyCode = 0x0000;
    
    if(g_BN_TestMode == 0x0000)
    {
        if (Enable_BATDRV_LOG == 1) {
            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] mt_battery_notify_check\n");
        }

#if defined(BATTERY_NOTIFY_CASE_0000)    
        if (Enable_BATDRV_LOG == 1) {
            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] BATTERY_NOTIFY_CASE_0000\n");
        }
#endif

#if defined(BATTERY_NOTIFY_CASE_0001)
        if(BMT_status.charger_vol > V_CHARGER_MAX)
        //if(BMT_status.charger_vol > 3000) //test
        {
            g_BatteryNotifyCode |= 0x0001;
            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] BMT_status.charger_vol(%ld) > %d mV\n", 
                BMT_status.charger_vol, V_CHARGER_MAX);
        }
        else
        {
            g_BatteryNotifyCode &= ~(0x0001);
        }
        if (Enable_BATDRV_LOG == 1) {
            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] BATTERY_NOTIFY_CASE_0001 (%x)\n", 
                g_BatteryNotifyCode);
        }    
#endif

#if defined(BATTERY_NOTIFY_CASE_0002)
        if( (BMT_status.temperature >= MAX_CHARGE_TEMPERATURE) || 
            (BMT_status.temperature < MIN_CHARGE_TEMPERATURE)
            )
        {
            g_BatteryNotifyCode |= 0x0002;
            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] bat_temp(%d) out of range\n", BMT_status.temperature);
        }
        else
        {
            g_BatteryNotifyCode &= ~(0x0002);
        }
        if (Enable_BATDRV_LOG == 1) {
            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] BATTERY_NOTIFY_CASE_0002 (%x)\n", 
                g_BatteryNotifyCode);
        }    
#endif

#if defined(BATTERY_NOTIFY_CASE_0003)
        if( (BMT_status.ICharging > 1000) &&
            (BMT_status.total_charging_time > 300)
            )
        //if(BMT_status.ICharging > 200) //test
        {
            g_BatteryNotifyCode |= 0x0004;
            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] I_charging(%ld) > 1000mA\n", BMT_status.ICharging);
        }
        else
        {
            g_BatteryNotifyCode &= ~(0x0004);
        }
        if (Enable_BATDRV_LOG == 1) {
            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] BATTERY_NOTIFY_CASE_0003 (%x)\n", 
                g_BatteryNotifyCode);
        }    
#endif

#if defined(BATTERY_NOTIFY_CASE_0004)
        if(BMT_status.bat_vol > 4350)
        //if(BMT_status.bat_vol > 3800) //test
        {
            g_BatteryNotifyCode |= 0x0008;
            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] bat_vlot(%ld) > 4350mV\n", BMT_status.bat_vol);
        }
        else
        {
            g_BatteryNotifyCode &= ~(0x0008);
        }
        if (Enable_BATDRV_LOG == 1) {
            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] BATTERY_NOTIFY_CASE_0004 (%x)\n", 
                g_BatteryNotifyCode);
        }
#endif

#if defined(BATTERY_NOTIFY_CASE_0005)
        if( (g_battery_thermal_throttling_flag==2) || (g_battery_thermal_throttling_flag==3) )
        {
            printk("[TestMode] Disable Safty Timer : no UI display\n");
        }
        else
        {
            if(BMT_status.total_charging_time >= MAX_CHARGING_TIME)
            //if(BMT_status.total_charging_time >= 60) //test
            {
                g_BatteryNotifyCode |= 0x0010;
                xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] Charging Over Time\n");
            }
            else
            {
                g_BatteryNotifyCode &= ~(0x0010);
            }
        }
        
        if (Enable_BATDRV_LOG == 1) {
            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] BATTERY_NOTIFY_CASE_0005 (%x)\n", 
                g_BatteryNotifyCode);
        }
#endif

    }
    else if(g_BN_TestMode == 0x0001)
    {
        g_BatteryNotifyCode = 0x0001;
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY_TestMode] BATTERY_NOTIFY_CASE_0001\n");
    }
    else if(g_BN_TestMode == 0x0002)
    {
        g_BatteryNotifyCode = 0x0002;
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY_TestMode] BATTERY_NOTIFY_CASE_0002\n");
    }
    else if(g_BN_TestMode == 0x0003)
    {
        g_BatteryNotifyCode = 0x0004;
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY_TestMode] BATTERY_NOTIFY_CASE_0003\n");
    }
    else if(g_BN_TestMode == 0x0004)
    {
        g_BatteryNotifyCode = 0x0008;
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY_TestMode] BATTERY_NOTIFY_CASE_0004\n");
    }
    else if(g_BN_TestMode == 0x0005)
    {
        g_BatteryNotifyCode = 0x0010;
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY_TestMode] BATTERY_NOTIFY_CASE_0005\n");
    }
    else
    {
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] Unknown BN_TestMode Code : %x\n", g_BN_TestMode);
    }
}
#endif/* OPPO_NEW_CHARGING */ 

#ifdef OPPO_NEW_CHARGING
int get_pmic_flag=0;
#endif
void BAT_thread(void)
{
    int i=0;
    int BAT_status = 0;
	UINT32 charger_vol;
    //kal_uint32 tmp32;

	#ifndef VENDOR_EDIT//Nanwei.Deng@BasicDrv.CHG, 2012/03/08, Add for Battery encryption
	if(upmu_is_chr_det()==KAL_TRUE)
    {
		if(oppo_high_battery_status == 0)
	    {
	    	if(oppo_high_battery_status_count < 10)
			{
		        CheckIDCompare();
		        if(oppo_check_ID_status == 6)
		        {
		            oppo_high_battery_status = 1;
		            oppo_check_ID_status=0;
		        }   
				oppo_high_battery_status_count++; 
				printk( "<dengnanwei_dayin_ID = %d: oppo_high_battery_status =%d oppo_high_battery_status_count =%d:%s: LINE= %d : \n", oppo_check_ID_status,oppo_high_battery_status,oppo_high_battery_status_count,__FUNCTION__,__LINE__);
	    	}
		}
		else 
		{
			oppo_high_battery_status_count=0;
		}
	}
    #endif/*VENDOR_EDIT*/    
	    
    if (Enable_BATDRV_LOG == 1) {
#if defined(MTK_JEITA_STANDARD_SUPPORT)
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY_TOP] LOG. %d,%d,%d,%d,%d----------------------------\n", 
            BATTERY_AVERAGE_SIZE, CHARGING_FULL_CURRENT, g_jeita_recharging_voltage, gFG_15_vlot, mtk_jeita_support_flag);
#else    	
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY_TOP] LOG. %d,%d,%d,%d,%d----------------------------\n", 
            BATTERY_AVERAGE_SIZE, CHARGING_FULL_CURRENT, RECHARGING_VOLTAGE, gFG_15_vlot, mtk_jeita_support_flag);
#endif            
    }
    
    #ifdef OPPO_NEW_CHARGING
	if(get_pmic_flag == 0)
	{
		#ifdef OPPO_FACTORY_TEST
		if((get_boot_mode()==FACTORY_BOOT))
    	{
    	    at_test_chg_on=0; //first disable charger, need ioctl to turn on
    	    
    	}
    	else
    	{
    	    at_test_chg_on=1; //first enable charger
    	}
    	#endif

		get_pmic_flag = 1;
	}
	#endif
	
    if( (g_battery_thermal_throttling_flag==1) || (g_battery_thermal_throttling_flag==3) )
    {
        if(battery_cmd_thermal_test_mode == 1){
            BMT_status.temperature = battery_cmd_thermal_test_mode_value;
            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[Battery] In thermal_test_mode 2, Tbat=%d\n", BMT_status.temperature);
        }
    
#if defined(MTK_JEITA_STANDARD_SUPPORT)
        //ignore default rule
#else    
        #ifndef OPPO_NEW_CHARGING
        if(BMT_status.temperature >= 60)
        {
            #if defined(CONFIG_POWER_EXT)
            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] CONFIG_POWER_EXT, no update mt6320_battery_update_power_down.\n");
            #else
            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[Battery] Tbat(%d)>=60, system need power down.\n", BMT_status.temperature);

            mt6320_battery_update_power_down(&mt6320_battery_main);

            if( upmu_is_chr_det() == KAL_TRUE )
            {
                // can not power down due to charger exist, so need reset system
                arch_reset(0,NULL);
            }
            //avoid SW no feedback
            mt_power_off();
            #endif
        }
        #endif
#endif
        
    }

#ifdef VENDOR_EDIT//rendong.shi@BasicDrv.PRD add for avoiding  noise in charging for TP
	 charger_vol = PMIC_IMM_GetOneChannelValue(AUXADC_CHARGER_VOLTAGE_CHANNEL,5,1);
	 charger_vol = charger_vol / 100;
	 printk("shirendong charger_vol = %d   1!!!1\n",charger_vol);
	if((charger_vol > 4000)&& (charger_vol< 6000 ))
	{
		#ifndef VENDOR_EDIT//rendong.shi@BasicDrv.PRD add for avoiding  noise in charging for TP
			tp_is_in_charging = 1;
			if(is_tp_init == 1 && tp_set != 1)
			{
				printk("shirendong tp_charge_mode_set  1!!!1\n");
				tp_charge_mode_set(1);
				tp_set = 1;
			}
	    #endif
	}
	else
	{
		#ifndef VENDOR_EDIT//rendong.shi@BasicDrv.PRD add for avoiding  noise in charging for TP
		tp_is_in_charging = 0;
	      if(is_tp_init == 1 && tp_set != 0)
		{
		    printk("shirendong tp_charge_mode_set  off!!!0\n");
			tp_charge_mode_set(0);
			tp_set = 0;	
		}
		#endif
	}
#endif /*VENDOR_EDIT*/	
	
	
    /* If charger exist, than get the charger type */    
    if( upmu_is_chr_det() == KAL_TRUE )
    {
	
	
        wake_lock(&battery_suspend_lock);        
		    
		#ifdef VENDOR_EDIT//Nanwei.Deng@BasicDrv.charger, add for  Charger type judgment
        if((g_charger_type > CHARGER_UNKNOWN)&&(g_charger_type <= STANDARD_CHARGER))
        {
            BMT_status.charger_type = g_charger_type;
            g_charger_type= CHARGER_UNKNOWN;
	
			#ifndef VENDOR_EDIT//rendong.shi@BasicDrv.PRD add for avoiding  noise in charging for TP
			tp_is_in_charging = 1;
			if(is_tp_init == 1)
			{
				printk("shirendong tp_charge_mode_set  1!!!1\n");
				tp_charge_mode_set(1);
				tp_set = 1;
			}
			#endif
        }
        else
        {
            g_charger_type= CHARGER_UNKNOWN;
        }
		#endif /*VENDOR_EDIT*/
		
        if(BMT_status.charger_type == CHARGER_UNKNOWN)
        //if((BMT_status.charger_type == CHARGER_UNKNOWN) && mt_usb_is_device())        
        {
			#ifndef VENDOR_EDIT//rendong.shi@BasicDrv.PRD add for avoiding  noise in charging for TP
			tp_is_in_charging = 1;
			if(is_tp_init == 1)
			{
				printk("shirendong tp_charge_mode_set  1!!!1\n");
				tp_charge_mode_set(1);
				tp_set = 1;
			}
			#endif

			#if 0//def VENDOR_EDIT  //Jiada.Li@BasicDrv.Charger, 2013/06/18, for  Charger type judgment 
			CHR_Type_num = oppo_mt_charger_type_detection();
			#else
            CHR_Type_num = mt_charger_type_detection();                  
            #endif
            BMT_status.charger_type = CHR_Type_num;

            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BAT_thread] CHR_Type_num=%d\r\n", CHR_Type_num);
            
            if( (CHR_Type_num==STANDARD_HOST) || (CHR_Type_num==CHARGING_HOST) )
            {
                mt_usb_connect();
            }
			
			#ifdef OPPO_NEW_CHARGING
            oppo_charger_in_reset_variable();
	        #endif
        }        
    }
    else 
    {

        post_charging_time=0;
   
        wake_unlock(&battery_suspend_lock);

        #ifdef OPPO_USE_II_BQ24157
        //Jiada.Li@BasicDrv.CHG, 2013/04/15, debug for BQ24157'bug
        if(BMT_status.charger_type != CHARGER_UNKNOWN)
        {
            if(BQ24157_ID == oppo_excharger_id)
            {
                pchr_turn_off_charging();

                #ifdef OPPO_BQ24157_R68
                bq24156a_set_low_chg(1);
                #else
                bq24156a_set_low_chg(0);// not 221mA
                #endif
            }
        }
        #endif
    
        BMT_status.charger_type = CHARGER_UNKNOWN;
        BMT_status.bat_full = KAL_FALSE;
        
        /*Use no gas gauge*/
        if( gForceADCsolution == 1 )
        {
            g_bat_full_user_view = KAL_FALSE;
        }
        /*Use gas gauge*/
        else
        {
            if(bat_volt_check_point != 100) {
                g_bat_full_user_view = KAL_FALSE;
                
                if (Enable_BATDRV_LOG == 1) {
                    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[Battery_Only] Set g_bat_full_user_view=KAL_FALSE\r\n");
                }
            }
        }

        g_usb_state = USB_UNCONFIGURED;

        g_HW_Charging_Done = 0;
        g_Charging_Over_Time = 0;
        g_Calibration_FG = 0;

        mt_usb_disconnect();          
        
        for (i=0; i<BATTERY_AVERAGE_SIZE; i++) {
            batteryCurrentBuffer[i] = 0;
        }
        batteryCurrentSum = 0;
    }

    /* Check Battery Status */
    BAT_status = BAT_CheckBatteryStatus();
    #ifndef OPPO_NEW_CHARGING
    if( BMT_status.bat_charging_state == CHR_ERROR || BAT_status == PMU_STATUS_FAIL)
        g_Battery_Fail = KAL_TRUE;
    else
        g_Battery_Fail = KAL_FALSE;

    if(battery_cmd_thermal_test_mode == 1){
        BMT_status.temperature = battery_cmd_thermal_test_mode_value;
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] In thermal_test_mode 3, Tbat=%d\n", BMT_status.temperature);
    }

    /* Battery Notify Check */    
    mt_battery_notify_check();
	#endif/* OPPO_NEW_CHARGING */ 
#if defined(CONFIG_POWER_EXT)
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] CONFIG_POWER_EXT, no update Android.\n");
#elif defined(MTK_KERNEL_POWER_OFF_CHARGING) 
    #ifndef OPPO_NO_USE_FGADC
	/*Only in kpoc mode*/
	if(gFG_booting_counter_I_FLAG == 1)
	{
		/*Use no gas gauge*/
		if( gForceADCsolution == 1 )
		{
			//do nothing
		}
		else
		{
			if(g_boot_mode == KERNEL_POWER_OFF_CHARGING_BOOT || g_boot_mode == LOW_POWER_OFF_CHARGING_BOOT)
			{			
				mt6320_ac_update(&mt6320_ac_main);
				mt6320_usb_update(&mt6320_usb_main);
				mt6320_battery_update(&mt6320_battery_main);  
			}
			xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] gFG_booting_counter_I_FLAG is 1, soc=%d\n", fgauge_read_capacity_by_v());
		}
	}
	else if(gFG_booting_counter_I_FLAG == 2)
	#else
	if(1)
	#endif
	{
	    #ifndef OPPO_NO_USE_FGADC
		if(boot_check_once==1)
		{
		    extern int get_rtc_spare_fg_value(void);
		    
			if( upmu_is_chr_det() == KAL_TRUE && BMT_status.SOC == 100 && get_rtc_spare_fg_value() == 100)
			{
				g_bat_full_user_view = KAL_TRUE;                
				printk("[BATTERY] g_bat_full_user_view=%d\n", g_bat_full_user_view);
			}
			boot_check_once=0;
		}
		#endif
		mt6320_ac_update(&mt6320_ac_main);
		mt6320_usb_update(&mt6320_usb_main);
		mt6320_battery_update(&mt6320_battery_main);  	
	}
	else
	{
		xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] gFG_booting_counter_I_FLAG!=2 (%d)\r\n", gFG_booting_counter_I_FLAG);
	}
#else
	#ifndef OPPO_NO_USE_FGADC
    if(gFG_booting_counter_I_FLAG == 2)
	#else
	if(1)
	#endif
    {
        /* AC/USB/Battery information update for Android */
        mt6320_ac_update(&mt6320_ac_main);
        mt6320_usb_update(&mt6320_usb_main);
        mt6320_battery_update(&mt6320_battery_main);   
    }
    else if(gFG_booting_counter_I_FLAG == 1)
    {
        /*Use no gas gauge*/
        if( gForceADCsolution == 1 )
        {
            //do nothing
        }
        else
        {
            mt6320_battery_main.BAT_CAPACITY = 50;
            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] gFG_booting_counter_I_FLAG is 1, soc=50.\n");
        }
    }
    else
    {
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] gFG_booting_counter_I_FLAG!=2 (%d)\r\n", gFG_booting_counter_I_FLAG);
    }
#endif

    #ifndef OPPO_NEW_CHARGING
    /* No Charger */
    if(BAT_status == PMU_STATUS_FAIL || g_Battery_Fail)    
    {
        gFG_can_reset_flag = 1;
        
        BAT_BatteryStatusFailAction();        
    }
    
    /* Battery Full */    /* HW charging done, real stop charging */
    else if (g_HW_Charging_Done == 1)
    {   
        if (Enable_BATDRV_LOG == 1) {
            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] Battery real full. \n");
        }
        BAT_BatteryFullAction();                
    }

    /* Charging Overtime, can not charging */
    else if (g_Charging_Over_Time == 1)
    {
        if (Enable_BATDRV_LOG == 1) {
            xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] Charging Over Time. \n");
        }
        pchr_turn_off_charging();

        if(gFG_can_reset_flag == 1)
        {
            gFG_can_reset_flag = 0;            
        }
    }
    
    /* Battery Not Full and Charger exist : Do Charging */
    else
    {
        gFG_can_reset_flag = 1;
            
        if( (g_battery_thermal_throttling_flag==2) || (g_battery_thermal_throttling_flag==3) )
        {
			if (Enable_BATDRV_LOG == 1) {
            printk("[TestMode] Disable Safty Timer. bat_tt_enable=%d, bat_thr_test_mode=%d, bat_thr_test_value=%d\n", 
            g_battery_thermal_throttling_flag, battery_cmd_thermal_test_mode, battery_cmd_thermal_test_mode_value);
        }
        }
        else
        {    
            /* Charging OT */
            if(BMT_status.total_charging_time >= MAX_CHARGING_TIME)
            {            
                BAT_ChargingOTAction();
                return;
            }
        }    

        if ( BMT_status.TOPOFF_charging_time >= MAX_CV_CHARGING_TIME )
        {
            //if (Enable_BATDRV_LOG == 1) {
                xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "BMT_status.TOPOFF_charging_time >= %d \r\n", MAX_CV_CHARGING_TIME);
            //}
            BMT_status.bat_charging_state = CHR_BATFULL;
            BAT_BatteryFullAction();                                        
            return;
        }

        if(0)
        {
            if ( (BMT_status.bat_charging_state == CHR_TOP_OFF) &&
                 (BMT_status.SOC == 100) && 
                 (BMT_status.bat_vol >= Batt_VoltToPercent_Table[10].BattVolt) )
            {
                if (Enable_BATDRV_LOG == 1) {
                    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] Battery real full(%ld,%d) and disable charging !\n", 
                            BMT_status.SOC, Batt_VoltToPercent_Table[10].BattVolt); 
                }
                BMT_status.bat_charging_state = CHR_BATFULL;
                BAT_BatteryFullAction();
                return;
            }    
        }
        else
        {
#if defined(DISABLE_POST_CHARGE)
            /* charging full condition when charging current < CHARGING_FULL_CURRENT mA on CHR_TOP_OFF mode*/
            if ( (BMT_status.bat_charging_state == CHR_TOP_OFF ) 
                 && (BMT_status.TOPOFF_charging_time > 60)
                 && (BMT_status.ICharging <= CHARGING_FULL_CURRENT)                 
                 )
            {
                BMT_status.bat_charging_state = CHR_BATFULL;
                BAT_BatteryFullAction();                                
                xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] Battery real full and disable charging on %ld mA \n", BMT_status.ICharging); 
                return;
            }
#else
            #if defined(MTK_JEITA_STANDARD_SUPPORT)
            if(g_temp_status == TEMP_NEG_10_TO_POS_0)
            {
                CHARGING_FULL_CURRENT=120;
            }
            else
            {
                CHARGING_FULL_CURRENT=220;
            }
            #endif

            if (post_charging_time >= POST_CHARGING_TIME)
            {
                BMT_status.bat_charging_state = CHR_BATFULL;
                BAT_BatteryFullAction();                
                xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] Battery real full and disable charging on %ld mA \n", BMT_status.ICharging); 
                
                post_charging_time=0;
                return;
            }
            else if (post_charging_time > 0)
            {
                post_charging_time+=BAT_TASK_PERIOD;
                xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] post_charging_time=%d,POST_CHARGING_TIME=%d\n", post_charging_time, POST_CHARGING_TIME); 
            }
            else if ((BMT_status.bat_charging_state == CHR_TOP_OFF)
                && (BMT_status.TOPOFF_charging_time > 60)
                && (BMT_status.ICharging <= CHARGING_FULL_CURRENT))
            {
                post_charging_time=BAT_TASK_PERIOD;
                xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] Enter Post charge\n"); 
                
                BMT_status.bat_full=KAL_TRUE;
                xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[BATTERY] UI show full first\n"); 
            }
            else
            {
                post_charging_time=0;
            }
#endif
        }
        
        /* Charging flow begin */
        switch(BMT_status.bat_charging_state)
        {            
            case CHR_PRE :
                BAT_PreChargeModeAction();
                break;    
                
            case CHR_CC :
                BAT_ConstantCurrentModeAction();
                break;    
                
            case CHR_TOP_OFF :
                BAT_TopOffModeAction();
                break;              
				
            #ifdef OPPO_NEW_CHARGING
            case CHR_POST_FULL :
                BAT_PostFullModeAction();
                break;				
            #endif

            case CHR_BATFULL:
                BAT_BatteryFullAction();
                break;
                
            case CHR_ERROR:
                BAT_BatteryStatusFailAction();
                break;                
        }    
    }
    #else/* OPPO_NEW_CHARGING */ 
    OPPO_BAT_Charge();
    #endif/* OPPO_NEW_CHARGING */ 

    g_HW_stop_charging = 0;

}

///////////////////////////////////////////////////////////////////////////////////////////
//// Internal API
///////////////////////////////////////////////////////////////////////////////////////////
int g_FG_init = 0;
int bat_thread_kthread(void *x)
{
    /* Run on a process content */  
    while (1) {               
        
        if(g_battery_flag_resume==0)
        {
            mutex_lock(&bat_mutex);
            
#if defined(CONFIG_POWER_EXT)
            BAT_thread();
#else
		    #ifndef OPPO_NO_USE_FGADC
            if(g_FG_init == 1)
            {
                g_FG_init=2;
                FGADC_thread_kthread();
                //sync FG timer
                FGADC_thread_kthread();
            }
            else
            {            
                // if plug-in/out USB, bypass once
                if(g_chr_event==0)
                {            
                    FGADC_thread_kthread();
                }
                
                BAT_thread();                      
            }
		    #else
		    BAT_thread();
		    #endif
#endif

            mutex_unlock(&bat_mutex);
        }
        else
        {
            g_battery_flag_resume=0;
            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[bat_thread_kthread] g_battery_flag_resume=%d\r\n", g_battery_flag_resume);
        }
        
        if (Enable_BATDRV_LOG == 1) {
            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "******** MT6320 battery : bat_thread_kthread : 1 ********\n" );
        }

        wait_event(bat_thread_wq, bat_thread_timeout);

        if (Enable_BATDRV_LOG == 1) {
            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "******** MT6320 battery : bat_thread_kthread : 2 ********\n" );
        }
        
        bat_thread_timeout=0;

		#ifndef OPPO_NO_USE_FGADC
        if( g_wake_up_bat==1 )
        {
            g_wake_up_bat=0;
            g_Calibration_FG = 0;
            FGADC_Reset_SW_Parameter();
            
            if (Enable_BATDRV_LOG == 1) {
                xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[BATTERY] Call FGADC_Reset_SW_Parameter.\r\n");
            }
        }
		#endif
    }

    return 0;
}

UINT32 bat_thread_timeout_sum=0;

void bat_thread_wakeup(void)
{
    if (Enable_BATDRV_LOG == 1) {
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "******** MT6320 battery : bat_thread_wakeup : 1 ********\n" );
    }
    
    bat_thread_timeout = 1;
    wake_up(&bat_thread_wq);    
    
    if (Enable_BATDRV_LOG == 1) {
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "******** MT6320 battery : bat_thread_wakeup : 2 ********\n" );
    }    
}
#ifdef OPPO_NEW_CHARGING

#ifdef OPPO_UBOOT_CHARGING
#define NO_BATTERY_TEMP     1150  //NO BATTERY VAL IS 1196,OPPO'S BATTERY MAX VAL IS 1068 
static BOOL oppo_check_battery_exist(void)
{
    kal_uint8 i=0;
    kal_uint8 cnt=0;
    
    for(i=0;i<5;i++)
    {
        UINT32 temp = PMIC_IMM_GetOneChannelValue(AUXADC_TEMPERATURE_CHANNEL,5,1);
        
        BMT_status.temperature = BattVoltToTemp(temp);
        oppo_chg_log("temp=%d,BMT_status.temperature =%d\n",temp ,BMT_status.temperature );
        
        if(temp > NO_BATTERY_TEMP)
        {
            cnt++;
            if(cnt >= 4) 
            {
                oppo_chg_log("uboot,no battery!\n");
                return FALSE;
            }
        }
     } 
     return TRUE;
}
#endif

#ifdef OPPO_KERNEL_CHARGING
UINT32 oppo_auto_test_reset_g_notify_flag(void)
{
    UINT32 data=0;
    
    if(g_BatteryNotifyCode & (1 <<NOTIFY_OVER_VCHARGER))
    {
        data=0x0010;
    }
    else if(g_BatteryNotifyCode & (1 <<NOTIFY_LOW_VCHARGER))
    {
        data=0x0020;
    }
    else if(g_BatteryNotifyCode & (1 <<NOTIFY_OVER_BATTEMP))
    {
        data=0x0040;
    }
    else if(g_BatteryNotifyCode & (1 <<NOTIFY_LOW_BATTEMP))
    {
        data=0x0080;
    }
    else if(g_BatteryNotifyCode & (1 <<NOTIFY_BAD_BATTEMP))
    {
        data=0x0400;
    }
    else if(g_BatteryNotifyCode & (1 <<NOTIFY_OVER_VBAT))
    {
        data=0x0200;
    }
    else if(g_BatteryNotifyCode & (1 <<NOTIFY_BAT_FULL))
    {
        data=0x0004;
    }
    else
    {
        data=0x0002;
    }

    return data;
}

#define FIRST_ADC_REPEAT    20
static kal_int32 oppo_get_first_bat_vol(void)
{
    kal_uint8 i=0,j=0;
    kal_int32 temp=0;
    
    kal_int32 ADC_BAT_SENSE_tmp[20]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    kal_int32 ADC_BAT_SENSE_sum=0;
    kal_int32 ADC_BAT_SENSE=0;	

    for(i=0;i<FIRST_ADC_REPEAT;i++)
    {
        ADC_BAT_SENSE_tmp[i] = PMIC_IMM_GetOneChannelValue(AUXADC_BATTERY_VOLTAGE_CHANNEL,1,1);		
        ADC_BAT_SENSE_sum += ADC_BAT_SENSE_tmp[i];
    }

    //sorting	BAT_SENSE , min to max
    for(i=0;i<FIRST_ADC_REPEAT;i++)
    {
        for(j=i;j<FIRST_ADC_REPEAT;j++)
        {
            if( ADC_BAT_SENSE_tmp[j] < ADC_BAT_SENSE_tmp[i] )
            {
            	temp = ADC_BAT_SENSE_tmp[j];
            	ADC_BAT_SENSE_tmp[j] = ADC_BAT_SENSE_tmp[i];
            	ADC_BAT_SENSE_tmp[i] = temp;
            }
        }
    }

    oppo_chg_log("Power/Battery", "[oppo_get_first_bat_vol:]\r\n");	
    for(i=0;i<FIRST_ADC_REPEAT;i++ )
    {
        oppo_chg_log("Power/Battery", "%d,", ADC_BAT_SENSE_tmp[i]);
    }
    oppo_chg_log("Power/Battery", "\r\n");

    ADC_BAT_SENSE_sum -= ADC_BAT_SENSE_tmp[0];
    ADC_BAT_SENSE_sum -= ADC_BAT_SENSE_tmp[1];
    ADC_BAT_SENSE_sum -= ADC_BAT_SENSE_tmp[18];
    ADC_BAT_SENSE_sum -= ADC_BAT_SENSE_tmp[19]; 	
    ADC_BAT_SENSE = ADC_BAT_SENSE_sum / (FIRST_ADC_REPEAT-4); //kick off the max and min 2,

    oppo_chg_log("Power/Battery ADC_BAT_SENSE=", "%d,", ADC_BAT_SENSE);

    return ADC_BAT_SENSE;
}
#endif

static void oppo_charger_in_reset_variable(void)
{
    g_BatteryNotifyCode = 0x0000;
    g_NotifyFlag = 0;
    
    #ifdef OPPO_FACTORY_TEST
    oppo_charging_stop_fg = KAL_FALSE;
    at_enter_chging_state = 0;
    #endif
    oppo_protect_to_hold_status = 0;
    low_charger_count = 0;
    low_charger_error_count = 0;
    over_charger_count = 0;
    over_charger_error_count = 0;
    bad_temper_count = 0;
    max_vbat_count = 0;
    low_temper_count = 0;
    over_temper_count = 0;

    high_temp_status_count = 0;
    super_low_temp_status_count = 0;
    low_temp_status_count = 0;
    normal_temp_status_count = 0;
    oppo_charge_temp_status = INIT_CHARGE_TEMP_STATUS;
    #ifdef OPPO_HIGH_BATTERY_VOLTAGE
    if(oppo_high_battery_status == 1)
    {
        oppo_charge_to_topoff_threshold = HI_BAT_NORMAL_CHARGE_TOPPOFF_THRESHOLD;
        oppo_charge_to_postfull_threshold = HI_BAT_NORMAL_CHARGE_POSTFULL_THRESHOLD;
        oppo_charge_to_full_threshold = HI_BAT_NORMAL_CHARGE_FULL_THRESHOLD;
        oppo_charge_to_recharging_threshold = HI_BAT_NORMAL_RECHARGING_THRESHOLD;
    }
    else
    #endif
    {
        oppo_charge_to_topoff_threshold = NORMAL_CHARGE_TOPPOFF_THRESHOLD;
        oppo_charge_to_postfull_threshold = NORMAL_CHARGE_POSTFULL_THRESHOLD;
        oppo_charge_to_full_threshold = NORMAL_CHARGE_FULL_THRESHOLD;
        oppo_charge_to_recharging_threshold = NORMAL_RECHARGING_THRESHOLD;
    }

    #ifdef OPPO_CALL_MODE_CHARGING_PAUSE
    oppo_call_pause_mode=KAL_FALSE;
    #endif
    #ifdef OPPO_CHARGE_CURRENT_ADAPTIVE
    check_charger_ability_fg = ZERO_CHARGER_CURRENT;
    charger_current_abilty = INIT_CHARGER_CURRENT;
    oppo_actual_cc_volue = ZERO_CHARGER_CURRENT;
    #endif
}

#ifdef OPPO_CALL_MODE_CHARGING_PAUSE
static void oppo_check_call_pause_mode(void)
{
    #if 1
    oppo_call_pause_mode = KAL_FALSE;
    #else
    if(call_status!=0)
    {
        if(KAL_FALSE == oppo_call_pause_mode)
        {
            if(IN_CALL_HIGH_VOL < BMT_status.bat_vol)
            {
                oppo_call_pause_mode = KAL_TRUE;
                if(Enable_BATDRV_LOG)
                {
                    oppo_chg_log("Power/Battery", "in call state&bat_vol(%d)>4v, call pause mode\n",BMT_status.bat_vol);
                }
            }
            
        }
        else
        {
            if(IN_CALL_LOW_VOL > BMT_status.bat_vol)
            {
                oppo_call_pause_mode = KAL_FALSE;
                if(Enable_BATDRV_LOG)
                {
                    oppo_chg_log("Power/Battery", "in call state&bat_vol(%d)<3800, out of call pause\n",BMT_status.bat_vol);
                }
            }
        }
    }
    else
    {
        oppo_call_pause_mode = KAL_FALSE;
    }
    #endif
}
#endif

static void oppo_charging_status_by_temp(void)
{
    #ifdef OPPO_USE_II_BQ24156A
    if(g_temp_CC_value > INIT_CHARGER_CURRENT)
    #elif defined(OPPO_USE_FAIRCHILD_FAN5405)
    if(g_temp_CC_value > INIT_CHARGER_CURRENT)
    #else
    if(g_temp_CC_value < INIT_CHARGER_CURRENT)
    #endif
    {
        switch(oppo_charge_temp_status)
        {
            case INIT_CHARGE_TEMP_STATUS:
                g_temp_CC_value = INIT_CHARGER_CURRENT;
                break;
                
            case NORMAL_CHARGE_TEMP_STATUS:
                
                break;
                
            case SUPER_LOW_CHARGE_TEMP_STATUS:
                g_temp_CC_value = SUPER_LOW_TEMP_CHARGER_CURRENT;
                break;
                
            case LOW_CHARGE_TEMP_STATUS:
                if((CHR_BATFULL == BMT_status.bat_charging_state) || (CHR_POST_FULL == BMT_status.bat_charging_state))
                {
                    g_temp_CC_value = LOW_TEMP_POSTFULL_CHARGER_CURRENT;
                }
                else
                {
                    g_temp_CC_value = LOW_TEMP_CHARGER_CURRENT;
                }
                break;

            case HIGH_CHARGE_TEMP_STATUS:
                #ifdef OPPO_CHARGE_CURRENT_BY_LCD
                #ifdef OPPO_USE_II_BQ24156A
                if((HIGH_TEMP_LCD_ON_CHARGER_CURRENT <= g_temp_CC_value) && (1 == oppo_chg_lcd_is_on))
                #elif defined(OPPO_USE_FAIRCHILD_FAN5405)
                if((HIGH_TEMP_LCD_ON_CHARGER_CURRENT <= g_temp_CC_value) && (1 == oppo_chg_lcd_is_on))
				#elif defined(OPPO_USE_II_BQ24157)
				if((HIGH_TEMP_LCD_ON_CHARGER_CURRENT <= g_temp_CC_value) && (1 == oppo_chg_lcd_is_on))
                #else
                if((AC_CHARGER_CURRENT == g_temp_CC_value) && (1 == oppo_chg_lcd_is_on))
                #endif
                {
                    g_temp_CC_value = HIGH_TEMP_LCD_ON_CHARGER_CURRENT;
                }
                else
                #endif
                {
                    g_temp_CC_value = HIGH_TEMP_LCD_OFF_CHARGER_CURRENT;
                }
                break;

            default:
                g_temp_CC_value = INIT_CHARGER_CURRENT;
                break;
        }
    }
    oppo_chg_log("oppo_charging_status_current_by_temp= %d\r\n", g_temp_CC_value);

    switch(oppo_charge_temp_status)
    {
        case INIT_CHARGE_TEMP_STATUS:
            #ifdef OPPO_HIGH_BATTERY_VOLTAGE
            if(oppo_high_battery_status == 1)
            {
                oppo_charge_to_topoff_threshold = HI_BAT_NORMAL_CHARGE_TOPPOFF_THRESHOLD;
                oppo_charge_to_postfull_threshold = HI_BAT_NORMAL_CHARGE_POSTFULL_THRESHOLD;
                oppo_charge_to_full_threshold = HI_BAT_NORMAL_CHARGE_FULL_THRESHOLD;
                oppo_charge_to_recharging_threshold = HI_BAT_NORMAL_RECHARGING_THRESHOLD;
            }
            else
            #endif
            {
                oppo_charge_to_topoff_threshold = NORMAL_CHARGE_TOPPOFF_THRESHOLD;
                oppo_charge_to_postfull_threshold = NORMAL_CHARGE_POSTFULL_THRESHOLD;
                oppo_charge_to_full_threshold = NORMAL_CHARGE_FULL_THRESHOLD;
                oppo_charge_to_recharging_threshold = NORMAL_RECHARGING_THRESHOLD;
            }
            break;
            
        case NORMAL_CHARGE_TEMP_STATUS:
            #ifdef OPPO_HIGH_BATTERY_VOLTAGE
            if(oppo_high_battery_status == 1)
            {
                oppo_charge_to_topoff_threshold = HI_BAT_NORMAL_CHARGE_TOPPOFF_THRESHOLD;
                oppo_charge_to_postfull_threshold = HI_BAT_NORMAL_CHARGE_POSTFULL_THRESHOLD;
                oppo_charge_to_full_threshold = HI_BAT_NORMAL_CHARGE_FULL_THRESHOLD;
                oppo_charge_to_recharging_threshold = HI_BAT_NORMAL_RECHARGING_THRESHOLD;
            }
            else
            #endif
            {
                oppo_charge_to_topoff_threshold = NORMAL_CHARGE_TOPPOFF_THRESHOLD;
                oppo_charge_to_postfull_threshold = NORMAL_CHARGE_POSTFULL_THRESHOLD;
                oppo_charge_to_full_threshold = NORMAL_CHARGE_FULL_THRESHOLD;
                oppo_charge_to_recharging_threshold = NORMAL_RECHARGING_THRESHOLD;
            }
            break;
            
        case SUPER_LOW_CHARGE_TEMP_STATUS:
            #ifdef OPPO_HIGH_BATTERY_VOLTAGE
            if(oppo_high_battery_status == 1)
            {
                oppo_charge_to_topoff_threshold = HI_BAT_SUPER_LOW_CHARGE_TOPPOFF_THRESHOLD;
                oppo_charge_to_postfull_threshold = HI_BAT_SUPER_LOW_CHARGE_POSTFULL_THRESHOLD;
                oppo_charge_to_full_threshold = HI_BAT_SUPER_LOW_CHARGE_FULL_THRESHOLD;
                oppo_charge_to_recharging_threshold = HI_BAT_SUPER_LOW_RECHARGING_THRESHOLD;
            }
            #endif
            else
            {
                oppo_charge_to_topoff_threshold = SUPER_LOW_CHARGE_TOPPOFF_THRESHOLD;
                oppo_charge_to_postfull_threshold = SUPER_LOW_CHARGE_POSTFULL_THRESHOLD;
                oppo_charge_to_full_threshold = SUPER_LOW_CHARGE_FULL_THRESHOLD;
                oppo_charge_to_recharging_threshold = SUPER_LOW_RECHARGING_THRESHOLD;
            }
            break;
            
        case LOW_CHARGE_TEMP_STATUS:
            #ifdef OPPO_HIGH_BATTERY_VOLTAGE
            if(oppo_high_battery_status == 1)
            {
                oppo_charge_to_topoff_threshold = HI_BAT_LOW_CHARGE_TOPPOFF_THRESHOLD;
                oppo_charge_to_postfull_threshold = HI_BAT_LOW_CHARGE_POSTFULL_THRESHOLD;
                oppo_charge_to_full_threshold = HI_BAT_LOW_CHARGE_FULL_THRESHOLD;
                oppo_charge_to_recharging_threshold = HI_BAT_LOW_RECHARGING_THRESHOLD;
            }
            #endif
            else
            {
                oppo_charge_to_topoff_threshold = LOW_CHARGE_TOPPOFF_THRESHOLD;
                oppo_charge_to_postfull_threshold = LOW_CHARGE_POSTFULL_THRESHOLD;
                oppo_charge_to_full_threshold = LOW_CHARGE_FULL_THRESHOLD;
                oppo_charge_to_recharging_threshold = LOW_RECHARGING_THRESHOLD;
            }
            break;

        case HIGH_CHARGE_TEMP_STATUS:
            #ifdef OPPO_HIGH_BATTERY_VOLTAGE
            if(oppo_high_battery_status == 1)
            {
                oppo_charge_to_topoff_threshold = HI_BAT_HIGH_CHARGE_TOPPOFF_THRESHOLD;
                oppo_charge_to_postfull_threshold = HI_BAT_HIGH_CHARGE_POSTFULL_THRESHOLD;
                oppo_charge_to_full_threshold = HI_BAT_HIGH_CHARGE_FULL_THRESHOLD;
                oppo_charge_to_recharging_threshold = HI_BAT_HIGH_RECHARGING_THRESHOLD;
            }
            #endif
            else
            {
                oppo_charge_to_topoff_threshold = HIGH_CHARGE_TOPPOFF_THRESHOLD;
                oppo_charge_to_postfull_threshold = HIGH_CHARGE_POSTFULL_THRESHOLD;
                oppo_charge_to_full_threshold = HIGH_CHARGE_FULL_THRESHOLD;
                oppo_charge_to_recharging_threshold = HIGH_RECHARGING_THRESHOLD;
            }
            break;

        default:
            #ifdef OPPO_HIGH_BATTERY_VOLTAGE
            if(oppo_high_battery_status == 1)
            {
                oppo_charge_to_topoff_threshold = HI_BAT_SUPER_LOW_CHARGE_TOPPOFF_THRESHOLD;
                oppo_charge_to_postfull_threshold = HI_BAT_SUPER_LOW_CHARGE_POSTFULL_THRESHOLD;
                oppo_charge_to_full_threshold = HI_BAT_SUPER_LOW_CHARGE_FULL_THRESHOLD;
                oppo_charge_to_recharging_threshold = HI_BAT_SUPER_LOW_RECHARGING_THRESHOLD;
            }
            #endif
            else
            {
                oppo_charge_to_topoff_threshold = SUPER_LOW_CHARGE_TOPPOFF_THRESHOLD;
                oppo_charge_to_postfull_threshold = SUPER_LOW_CHARGE_POSTFULL_THRESHOLD;
                oppo_charge_to_full_threshold = SUPER_LOW_CHARGE_FULL_THRESHOLD;
                oppo_charge_to_recharging_threshold = SUPER_LOW_RECHARGING_THRESHOLD;
            }
            break;
    }
    oppo_chg_log("oppo_charging_status_topoff_by_temp= %d\r\n", oppo_charge_to_topoff_threshold);
    oppo_chg_log("oppo_charging_status_full_by_temp= %d\r\n", oppo_charge_to_full_threshold);
}

#ifdef BAT_LOW_CLOSED_POWERON
void Bat_Low_Closed_Status_Check(void)
{     
    printk("dengnanwei_Bat_Low_Closed_Status_Check--charge = %d, g_BAT_Low_Close = %d, g_IPO_status = %d\r\n", BMT_status.charger_exist, g_BAT_Low_Close, g_IPO_status);
    if( BMT_status.charger_exist && (g_BAT_Low_Close == KAL_TRUE) && (g_IPO_status == KAL_TRUE)&& (BMT_status.bat_vol > OPPO_IPO_BOOT_VOL ))
    {
        if (Enable_BATDRV_LOG == 1) 
        {
    //        printk("kpd_pwkey_longpass-----bat_vol = %d, soc = %d,AC_ONLINE = %d, USB_ONLINE = %d, BAT_CAPACITY = %d\r\n", BMT_status.bat_vol, BMT_status.SOC, mt6320_ac_data.AC_ONLINE, mt6320_usb_data.USB_ONLINE, mt6320_battery_data.BAT_CAPACITY);
        }       
        kpd_pwkey_longpass();
    }
}
#endif
#ifdef OPPO_CHARGE_CURRENT_ADAPTIVE
#ifdef OPPO_USE_EXCHARGER
#ifdef OPPO_USE_FAIRCHILD_FAN5405

//FAIRCHILD FAN5405 REGISTERS
//
//Register 00: (7) (6) (5) (4)  (3) (2) (1) (0)       Default:x1xx 0xxxx
//              |   |   |   |    |   |   |   |
//              |   |   |   |    |   |   |   |-------|FAULT:
//              |   |   |   |    |   |   |-----------|
//              |   |   |   |    |   |---------------|
//              |   |   |   |    |--------------------BOOST
//              |   |   |   |
//              |   |   |   |------------------------|STAT:00 Ready  01 Charge in process  10 Charge done 11 Fault
//              |   |   |----------------------------|
//              |   |---------------------------------EN_STAT
//              |-------------------------------------TMR_RST:1
//
//Register 01: (7) (6) (5) (4)  (3) (2) (1) (0)       Default:0011 0000
//              |   |   |   |    |   |   |   |
//              |   |   |   |    |   |   |   |--------OPA_MODE:0 Charge mode  1 Boost mode
//              |   |   |   |    |   |   |------------HZ_MODE:0 Not high-impedance mode  1 high-impedance mode
//              |   |   |   |    |   |----------------CE:0  Charge enable  1 Charge disable
//              |   |   |   |    |--------------------TE:0  Disable charge current termination  1 Enable
//              |   |   |   |
//              |   |   |   |------------------------|Weak battery low threshold:00 3.4V  01 3.5V  10 3.6V  11 3.7V
//              |   |   |----------------------------|
//              |   |--------------------------------|Input current limit:00 100mA  01 500mA  10 800mA  11 no limit
//              |------------------------------------|
//
//Register 02: (7) (6) (5) (4)  (3) (2) (1) (0)       Default:0000 1010
//              |   |   |   |    |   |   |   |
//              |   |   |   |    |   |   |   |--------OTG_EN:0 Disable OTG pin  1 Enable OTG pin
//              |   |   |   |    |   |   |------------OTG_PL:0 OTG pin active LOW  1 OTG pin active HIGH
//              |   |   |   |    |   |---------------|OREG:0x00 3.50V  0x01 3.52V  0x02 3.54V... ...0x24 4.22V... ...
//              |   |   |   |    |-------------------|
//              |   |   |   |
//              |   |   |   |------------------------|
//              |   |   |----------------------------|
//              |   |--------------------------------|
//              |------------------------------------|
//
//Register 03: (7) (6) (5) (4)  (3) (2) (1) (0)       Default:100x xxxx
//              |   |   |   |    |   |   |   |
//              |   |   |   |    |   |   |   |--------REV:
//              |   |   |   |    |   |   |------------PN:
//              |   |   |   |    |   |---------------|Vendor code:
//              |   |   |   |    |-------------------|
//              |   |   |   |
//              |   |   |   |------------------------|
//              |   |   |----------------------------|
//              |   |--------------------------------|
//              |------------------------------------|
//
//Register 04: (7) (6) (5) (4)  (3) (2) (1) (0)       Default:1000 1001
//              |   |   |   |    |   |   |   |
//              |   |   |   |    |   |   |   |-------|ITERM:Sets the current used for charging termination
//              |   |   |   |    |   |   |-----------|         (100mO)000 33mA   001 66mA   010 99mA   011 132mA
//              |   |   |   |    |   |---------------|                100 165mA  101 198mA  110 231mA  111 264mA
//              |   |   |   |    |--------------------Reserved:unused
//              |   |   |   |
//              |   |   |   |------------------------|IOCHARGE:Max charge current
//              |   |   |----------------------------|         (100mO)000 374mA  001 442mA  010 510mA  011 578mA
//              |   |--------------------------------|                100 646mA  101 714mA  110 782mA  111 850mA
//              |-------------------------------------RESET:Writing a 1 resets charge parameters ,except the Safety register(Reg6)
//
//Register 05: (7) (6) (5) (4)  (3) (2) (1) (0)       Default:001x x100
//              |   |   |   |    |   |   |   |
//              |   |   |   |    |   |   |   |-------|VSP:
//              |   |   |   |    |   |   |-----------|
//              |   |   |   |    |   |---------------|
//              |   |   |   |    |--------------------EN_LEVEL:0 DISABLE pin is LOW  1 DISABLE pin is HIGH
//              |   |   |   |
//              |   |   |   |-------------------------SP:
//              |   |   |-----------------------------IO_LEVEL:0 Output current is controlled by IOCHARGE bits  1 221mA(100mO)
//              |   |---------------------------------DIS_VREG:0 1.8V regulator in ON  1 1.8V regulator in OFF
//              |-------------------------------------Reserved:unused
//
//Register 06: (7) (6) (5) (4)  (3) (2) (1) (0)       Default:0100 0000
//              |   |   |   |    |   |   |   |
//              |   |   |   |    |   |   |   |-------|VSAFE:Sets the max OREG:0x00 3.50V  0x01 3.52V  0x02 3.54V... ...0x24 4.22V... ...
//              |   |   |   |    |   |   |-----------|
//              |   |   |   |    |   |---------------|
//              |   |   |   |    |-------------------|
//              |   |   |   |
//              |   |   |   |------------------------|ISAFE:Sets the max IOCHARGE
//              |   |   |----------------------------|         (100mO)000 374mA  001 442mA  010 510mA  011 578mA
//              |   |--------------------------------|                100 646mA  101 714mA  110 782mA  111 850mA
//              |-------------------------------------Reserved:Bit disabled and always returns 0 when read back
//
//Register 10H:(7) (6) (5) (4)  (3) (2) (1) (0)       Default:this bits only read
//              |   |   |   |    |   |   |   |
//              |   |   |   |    |   |   |   |--------CV
//              |   |   |   |    |   |   |------------VBUS_VALID
//              |   |   |   |    |   |----------------IBUS
//              |   |   |   |    |--------------------ICHG
//              |   |   |   |
//              |   |   |   |-------------------------T_120
//              |   |   |-----------------------------LINCHG
//              |   |---------------------------------VBAT_CMP
//              |-------------------------------------ITERM_CMP


void ChargerHwInit_fan5405(void)
{
	if (Enable_BATDRV_LOG == 1)
	{
		oppo_chg_log("[BAT_probe] ChargerHwInit\n" );
	}
	
	fan5405_config_interface_liao(0x00,0x80);//reset safty timer
}

static void check_fan5405_status(void)
{
    UINT8 reg_val=0;
    UINT8 fault_reasion=0;
 
    reg_val=fan5405_get_chip_status();
    
    oppo_chg_log("%s,reg_val=0x%x,bat_vol=%d\n",__func__,reg_val,BMT_status.bat_vol);
    
    switch(reg_val&FAN5405_STATUS_FAULT)
    {
        
        case FAN5405_STATUS_IN_PROGESS:
            oppo_fan5405_status = FAN5405_STATUS_IN_PROGESS;
            break;
        case FAN5405_STATUS_DONE:
            oppo_fan5405_status = FAN5405_STATUS_DONE;
            BMT_status.bat_charging_state = CHR_BATFULL;
            break;
        case FAN5405_STATUS_READY:
            oppo_fan5405_status = FAN5405_STATUS_READY;
            break;
        case FAN5405_STATUS_FAULT:
            oppo_fan5405_status = FAN5405_STATUS_FAULT;
            //BMT_status.bat_charging_state=CHR_ERROR;
            fault_reasion=fan5405_get_fault_status();
            //oppo_chg_log("dc charger fault,fault reasion:%d,so turn off charging\n",fault_reasion);
            break;
    }
    #ifdef BAT_LOW_CLOSED_POWERON //Fanhong.Kong@ProDrv.CHG, add 2012/7/6 for low bat power on
    Bat_Low_Closed_Status_Check();
    #endif
    
}

#define FAN5405_I_CHR_SHIFT 4
static void upmu_chr_vbat_cs_vth_by_fan5405(kal_uint8 val)
{
    if(FAN5405_CURRENT_221MA < val)
    {
        fan5405_config_interface_liao(0x05,0x04);
        fan5405_config_interface_liao(0x04,((val - FAN5405_CURRENT_374MA)<< FAN5405_I_CHR_SHIFT)|0x0C);//current used for charging termination:165mA
    }
    else
    {
        fan5405_config_interface_liao(0x05,0x24);
    }
}

#ifdef OPPO_FACTORY_TEST
static void set_at_charger_current_fan5405(void)
{
    fan5405_config_interface_liao(0x00,0x80);
    oppo_chg_sleep_ms(5);//wait for reset complete

    fan5405_config_interface_liao(0x01,0x48);
    #ifdef OPPO_HIGH_BATTERY_VOLTAGE
	if(oppo_high_battery_status == 1) fan5405_config_interface_liao(0x02,FAN5405_HI_BAT_NORMAL_DCDC_OREG);
	else
	#endif
	fan5405_config_interface_liao(0x02,FAN5405_NORMAL_DCDC_OREG);
    
    upmu_chr_vbat_cs_vth_by_fan5405(AT_CHARGER_CURRENT);//CS_VTH
    
	ChargerHwInit_fan5405();
}
#endif

static void check_charger_current_ability_fan5405(void)
{
    UINT8 i,temp_step;
    UINT32 charger_vol;
    UINT8 dpmStatus;

    temp_step = check_charger_ability_fg;

    fan5405_config_interface_liao(0x00,0x80);
    oppo_chg_sleep_ms(5);//wait for reset complete

    if(( BMT_status.charger_type == CHARGING_HOST ) || ( BMT_status.charger_type == STANDARD_CHARGER ))//(USB_CHARGER_CURRENT < temp_step)
    {
        fan5405_config_interface_liao(0x01,0xc8);
    }
    else
    {
        fan5405_config_interface_liao(0x01,0x48);
    }
    #ifdef OPPO_HIGH_BATTERY_VOLTAGE
	if(oppo_high_battery_status == 1) fan5405_config_interface_liao(0x02,FAN5405_HI_BAT_NORMAL_DCDC_OREG);
	else
	#endif
    fan5405_config_interface_liao(0x02,FAN5405_NORMAL_DCDC_OREG);

    for(i=FAN5405_CURRENT_221MA;i<=temp_step;i++)
    {
        upmu_chr_vbat_cs_vth_by_fan5405(i);//CS_VTH
        oppo_chg_sleep_ms(5);
#if 0        
        dpmStatus = fan5405_get_dpm_status();
        
        oppo_chg_log("Power/Battery", "dpmStatus=%d \r\n", dpmStatus);

		charger_current_abilty = i; 
		if(0x01 == dpmStatus)//B4 DPM_STATUE
		{
		    charger_current_abilty = i-1;
		    break;
		}
#else
        /* Get V_Charger */
	    charger_vol = PMIC_IMM_GetOneChannelValue(AUXADC_CHARGER_VOLTAGE_CHANNEL,5,1);
	    charger_vol = charger_vol / 100;

		//oppo_chg_log("check_charger_current_ability", "charger_vol=%d \r\n", charger_vol);
		//oppo_chg_log("current_i", "current_i=%d \r\n", i);

        charger_current_abilty = i;        
        oppo_actual_cc_volue = i;
		if(charger_vol < CHARGER_VOLTAGE_THRESHOLD)
		{
		    charger_current_abilty = i-1;
            break;
		}
#endif		
		oppo_chg_sleep_ms(1);
	}
	ChargerHwInit_fan5405();
}

/*static kal_uint8 if_check_charger_ability_fan5405(void)
{
    if(g_temp_CC_value > check_charger_ability_fg)
    {
        return KAL_TRUE;
    }
    return KAL_FALSE;
}*/

static void set_charge_adaptive_current_fan5405(void)
{
    UINT8 i;
    UINT32 charger_vol;
    UINT8 dpmStatus;

    if(g_temp_CC_value > charger_current_abilty)
    {
        oppo_actual_cc_volue = charger_current_abilty;
    }
    else
    {
        oppo_actual_cc_volue = g_temp_CC_value;
    }

    fan5405_config_interface_liao(0x00,0x80);
    oppo_chg_sleep_ms(5);//wait for reset complete

    if(( BMT_status.charger_type == CHARGING_HOST ) || ( BMT_status.charger_type == STANDARD_CHARGER ))//(USB_CHARGER_CURRENT < oppo_actual_cc_volue)
    {
        fan5405_config_interface_liao(0x01,0xc8);
    }
    else
    {
        fan5405_config_interface_liao(0x01,0x48);
    }
    if(SUPER_LOW_CHARGE_TEMP_STATUS == oppo_charge_temp_status)
	{
	    #ifdef OPPO_HIGH_BATTERY_VOLTAGE
    	if(oppo_high_battery_status == 1) fan5405_config_interface_liao(0x02,FAN5405_HI_BAT_SUPER_LOW_DCDC_OREG);
    	else
    	#endif
    	fan5405_config_interface_liao(0x02,FAN5405_SUPER_LOW_DCDC_OREG);
	}
	else if(LOW_CHARGE_TEMP_STATUS == oppo_charge_temp_status)
	{
	    #ifdef OPPO_HIGH_BATTERY_VOLTAGE
    	if(oppo_high_battery_status == 1) fan5405_config_interface_liao(0x02,FAN5405_HI_BAT_LOW_DCDC_OREG);
    	else
    	#endif
	    fan5405_config_interface_liao(0x02,FAN5405_LOW_DCDC_OREG);
	}
	else if(HIGH_CHARGE_TEMP_STATUS == oppo_charge_temp_status)
	{
	    #ifdef OPPO_HIGH_BATTERY_VOLTAGE
    	if(oppo_high_battery_status == 1) fan5405_config_interface_liao(0x02,FAN5405_HI_BAT_HIGH_DCDC_OREG);
    	else
    	#endif
	    fan5405_config_interface_liao(0x02,FAN5405_HIGH_DCDC_OREG);
	}
	else
	{
	    #ifdef OPPO_HIGH_BATTERY_VOLTAGE
    	if(oppo_high_battery_status == 1) fan5405_config_interface_liao(0x02,FAN5405_HI_BAT_NORMAL_DCDC_OREG);
    	else
    	#endif
	    fan5405_config_interface_liao(0x02,FAN5405_NORMAL_DCDC_OREG);
	}

    for(i=FAN5405_CURRENT_221MA;i<=oppo_actual_cc_volue;i++)
    {
        upmu_chr_vbat_cs_vth_by_fan5405(i);//CS_VTH
		oppo_chg_sleep_ms(5);
#if 0
		dpmStatus = fan5405_get_dpm_status();
		
		//oppo_chg_log("set_charge_adaptive_current", "current_i=%d \r\n", i);

		if(0x01 == dpmStatus)//B4 DPM_STATUE
		{
		    charger_current_abilty = i-1;
		    oppo_actual_cc_volue = i;
            break;
		}
#else
		/* Get V_Charger */
	    charger_vol = PMIC_IMM_GetOneChannelValue(AUXADC_CHARGER_VOLTAGE_CHANNEL,5,1);
	    charger_vol = charger_vol / 100;
		
		//oppo_chg_log("set_charge_adaptive_current", "current_i=%d \r\n", i);

		if(charger_vol < CHARGER_VOLTAGE_THRESHOLD)
		{
		    charger_current_abilty = i-1;
		    oppo_actual_cc_volue = i;
            break;
		}
#endif
	}
	ChargerHwInit_fan5405();
}

static void close_charge_adaptive_current_fan5405(void)
{
    while(1)
    {
        if(oppo_actual_cc_volue <= FAN5405_CURRENT_221MA)
        {
            oppo_actual_cc_volue = ZERO_CHARGER_CURRENT;
            upmu_chr_vbat_cs_vth_by_fan5405(oppo_actual_cc_volue);//CS_VTH

            //oppo_chg_log("close_charge_adaptive_current", "oppo_actual_cc_volue=%d \r\n", oppo_actual_cc_volue);
            break;
        }
        else
        {
            oppo_actual_cc_volue--;
            upmu_chr_vbat_cs_vth_by_fan5405(oppo_actual_cc_volue);//CS_VTH
		    oppo_chg_sleep_ms(5);

		    //oppo_chg_log("close_charge_adaptive_current", "oppo_actual_cc_volue=%d \r\n", oppo_actual_cc_volue);
        }
	}
}
#endif /*OPPO_USE_FAIRCHILD_FAN5405*/

#ifdef OPPO_USE_II_BQ24156A

//TI SN111008(BQ24156A) REGISTERS
//
//Register 00: (7) (6) (5) (4)  (3) (2) (1) (0)       Default:x1xx 0xxxx
//              |   |   |   |    |   |   |   |
//              |   |   |   |    |   |   |   |-------|FAULT:
//              |   |   |   |    |   |   |-----------|
//              |   |   |   |    |   |---------------|
//              |   |   |   |    |--------------------NA
//              |   |   |   |
//              |   |   |   |------------------------|STAT:00 Ready  01 Charge in process  10 Charge done 11 Fault
//              |   |   |----------------------------|
//              |   |---------------------------------EN_STAT
//              |-------------------------------------TMR_RST:1
//
//Register 01: (7) (6) (5) (4)  (3) (2) (1) (0)       Default:0011 0000
//              |   |   |   |    |   |   |   |
//              |   |   |   |    |   |   |   |--------NA
//              |   |   |   |    |   |   |------------HZ_MODE:0 Not high-impedance mode  1 high-impedance mode
//              |   |   |   |    |   |----------------CE:0  Charge enable  1 Charge disable
//              |   |   |   |    |--------------------TE:0  Disable charge current termination  1 Enable
//              |   |   |   |
//              |   |   |   |------------------------|Weak battery low threshold:00 3.4V  01 3.5V  10 3.6V  11 3.7V
//              |   |   |----------------------------|
//              |   |--------------------------------|Input current limit:00 100mA  01 500mA  10 800mA  11 no limit
//              |------------------------------------|
//
//Register 02: (7) (6) (5) (4)  (3) (2) (1) (0)       Default:0000 1010
//              |   |   |   |    |   |   |   |
//              |   |   |   |    |   |   |   |--------NA
//              |   |   |   |    |   |   |------------NA
//              |   |   |   |    |   |---------------|OREG:3.50V+(offset)offset step = 20mV
//              |   |   |   |    |-------------------|
//              |   |   |   |
//              |   |   |   |------------------------|
//              |   |   |----------------------------|
//              |   |--------------------------------|
//              |------------------------------------|
//
//Register 03: (7) (6) (5) (4)  (3) (2) (1) (0)       Default:0101 000x
//              |   |   |   |    |   |   |   |
//              |   |   |   |    |   |   |   |-------|Revision:
//              |   |   |   |    |   |   |-----------|
//              |   |   |   |    |   |---------------|
//              |   |   |   |    |-------------------|I2C address:PN1
//              |   |   |   |
//              |   |   |   |------------------------|            PN0
//              |   |   |----------------------------|Vendor code:
//              |   |--------------------------------|
//              |------------------------------------|
//
//Register 04: (7) (6) (5) (4)  (3) (2) (1) (0)       Default:0000 0001
//              |   |   |   |    |   |   |   |
//              |   |   |   |    |   |   |   |-------|ITERM:Termination current:34mA+(offset)offset step = 34mA
//              |   |   |   |    |   |   |-----------|         (100mO)000 33mA   001 66mA   010 99mA   011 132mA
//              |   |   |   |    |   |---------------|                100 165mA  101 198mA  110 231mA  111 264mA
//              |   |   |   |    |-------------------|IOCHARGE:Charge current:374mA+(offset)offset step = 68mA
//              |   |   |   |
//              |   |   |   |------------------------|
//              |   |   |----------------------------|
//              |   |--------------------------------|
//              |-------------------------------------RESET:Writing a 1 resets charge parameters ,except the Safety register(Reg6)
//
//Register 05: (7) (6) (5) (4)  (3) (2) (1) (0)       Default:001x x100
//              |   |   |   |    |   |   |   |
//              |   |   |   |    |   |   |   |-------|VSREG:Special charge voltage:4.2V+(offset)offset step = 80mV
//              |   |   |   |    |   |   |-----------|
//              |   |   |   |    |   |---------------|
//              |   |   |   |    |--------------------CD STATUS:0 CD pin is LOW  1 CD pin is HIGH
//              |   |   |   |
//              |   |   |   |-------------------------DPM_STATUS:0 DPM mode is not active  1 DPM mode is active
//              |   |   |-----------------------------LOW_CHG:0 Output current is controlled by IOCHARGE bits  1 221mA(100mO)
//              |   |---------------------------------NA
//              |-------------------------------------NA
//
//Register 06: (7) (6) (5) (4)  (3) (2) (1) (0)       Default:0100 0000
//              |   |   |   |    |   |   |   |
//              |   |   |   |    |   |   |   |-------|VMREG:Max battery regulation voltage:4.2V+(offset)offset step = 20mV
//              |   |   |   |    |   |   |-----------|
//              |   |   |   |    |   |---------------|
//              |   |   |   |    |-------------------|
//              |   |   |   |
//              |   |   |   |------------------------|VMCHRG:Max charge current:374mA+(offset)offset step = 68mA
//              |   |   |----------------------------|
//              |   |--------------------------------|
//              |------------------------------------|


void ChargerHwInit_bq24156a(void)
{
	if (Enable_BATDRV_LOG == 1)
	{
		oppo_chg_log("[BAT_probe] ChargerHwInit\n" );
	}
	bq24156a_set_tmr_rst(1);//reset safty timer
	
	bq24156a_set_lowv_2(0);// 3.4V
	bq24156a_set_lowv_1(0);
	bq24156a_set_te(1); 

	bq24156a_set_hz_mode(0);//not in hz mode
	#if 0//Jiada.Li@BasicDrv.CHG, 2012/06/20, delete for use default 4.52
	bq24156a_set_vsreg(0x04);  //4.52
	#endif
}

static void check_bq24156a_status(void)
{
    UINT8 reg_val=0;
    UINT8 fault_reasion=0;
 
    reg_val=bq24156a_get_chip_status();
    
    oppo_chg_log("%s,reg_val=0x%x,bat_vol=%d\n",__func__,reg_val,BMT_status.bat_vol);
    
    switch(reg_val&BQ24156A_STATUS_FAULT)
    {
        
        case BQ24156A_STATUS_IN_PROGESS:
            oppo_bq24156a_status = BQ24156A_STATUS_IN_PROGESS;
            break;
        case BQ24156A_STATUS_DONE:
            oppo_bq24156a_status = BQ24156A_STATUS_DONE;
            BMT_status.bat_charging_state = CHR_BATFULL;
            break;
        case BQ24156A_STATUS_READY:
            oppo_bq24156a_status = BQ24156A_STATUS_READY;
            break;
        case BQ24156A_STATUS_FAULT:
            oppo_bq24156a_status = BQ24156A_STATUS_FAULT;
            //BMT_status.bat_charging_state=CHR_ERROR;
            fault_reasion=bq24156a_get_fault_reason();
            //oppo_chg_log("dc charger fault,fault reasion:%d,so turn off charging\n",fault_reasion);
            break;
    }
    #ifdef BAT_LOW_CLOSED_POWERON //Fanhong.Kong@ProDrv.CHG, add 2012/7/6 for low bat power on
    Bat_Low_Closed_Status_Check();
    #endif
}

static void upmu_chr_vbat_cs_vth_by_bq24156a(kal_uint8 val)
{
    #ifdef OPPO_BQ24157_R68
    if(BQ24156A_CURRENT_221MA < val)
    {
        bq24156a_set_low_chg(0);// not 221mA
        if(BQ24156A_ID == oppo_excharger_id)
        {
            bq24156a_set_ac_charging_current(val - BQ24156A_CURRENT_374MA);
        }
        else//BQ24157
        {
            if(BQ24157_CURRENT_325MA < val)
            {
                bq24157_set_ac_charging_current(val - BQ24157_CURRENT_550MA);
            }
            else
            {
                bq24156a_set_low_chg(1);// 325mA
            }
        }
    }
    else
    {
        bq24156a_set_low_chg(1);// 221mA
    }
    #else
    if(BQ24156A_CURRENT_221MA < val)
    {
        bq24156a_set_low_chg(0);// not 221mA
        if(BQ24156A_ID == oppo_excharger_id)
        {
            bq24156a_set_ac_charging_current(val - BQ24156A_CURRENT_374MA);
        }
        else//BQ24157
        {
            bq24157_set_ac_charging_current(val - BQ24157_CURRENT_374MA);
        }
    }
    else
    {
        bq24156a_set_low_chg(1);// 221mA
    }
    #endif
}

#ifdef OPPO_FACTORY_TEST
static void set_at_charger_current_bq24156a(void)
{
    bq24156a_set_reset(1);
    oppo_chg_sleep_ms(5);//wait for reset complete

    bq24156a_set_lin_limit(0x01);//500ma
	bq24156a_set_termination_current(0x04);// 34+136=170

	#ifdef OPPO_HIGH_BATTERY_VOLTAGE
	if(oppo_high_battery_status == 1) bq24156a_set_cv_vth(BQ24156A_HI_BAT_NORMAL_DCDC_OREG);
	else
	#endif
	bq24156a_set_cv_vth(BQ24156A_NORMAL_DCDC_OREG);

    bq24156a_set_ce(0);//charger enable
    
    upmu_chr_vbat_cs_vth_by_bq24156a(AT_CHARGER_CURRENT);//CS_VTH
    
	ChargerHwInit_bq24156a();
}
#endif

static void check_charger_current_ability_bq24156a(void)
{
    UINT8 i,temp_step;
    UINT32 charger_vol;
    UINT8 dpmStatus;

    temp_step = check_charger_ability_fg;

    bq24156a_set_reset(1);
    oppo_chg_sleep_ms(5);//wait for reset complete

    #ifdef OPPO_BQ24157_R68
    if(( BMT_status.charger_type == CHARGING_HOST ) || ( BMT_status.charger_type == STANDARD_CHARGER ))//(USB_CHARGER_CURRENT < temp_step)
    {
        bq24156a_set_lin_limit(0x03);//no limit
    }
    else
    {
        bq24156a_set_lin_limit(0x01);//500ma
    }
    #else
    if(( BMT_status.charger_type == CHARGING_HOST ) || ( BMT_status.charger_type == STANDARD_CHARGER ))//(USB_CHARGER_CURRENT < temp_step)
    {
        bq24156a_set_lin_limit(0x02);//800ma
    }
    else
    {
        bq24156a_set_lin_limit(0x01);//500ma
    }
    #endif

    bq24156a_set_termination_current(0x04);// 34+136=170
    #ifdef OPPO_HIGH_BATTERY_VOLTAGE
    if(oppo_high_battery_status == 1) bq24156a_set_cv_vth(BQ24156A_HI_BAT_NORMAL_DCDC_OREG);
    else
    #endif
	bq24156a_set_cv_vth(BQ24156A_NORMAL_DCDC_OREG);

    bq24156a_set_ce(0);//charger enable
    for(i=BQ24156A_CURRENT_221MA;i<=temp_step;i++)
    {
        upmu_chr_vbat_cs_vth_by_bq24156a(i);//CS_VTH
        oppo_chg_sleep_ms(5);
        
#if 0
        dpmStatus = bq24156a_get_dpm_status();
        
        //oppo_chg_log("Power/Battery", "dpmStatus=%d \r\n", dpmStatus);

		charger_current_abilty = i; 
		oppo_actual_cc_volue = i;
		if(0x01 == dpmStatus)//B4 DPM_STATUE
		{
		    charger_current_abilty = i-1;
		    break;
		}
#else
        /* Get V_Charger */
	    charger_vol = PMIC_IMM_GetOneChannelValue(AUXADC_CHARGER_VOLTAGE_CHANNEL,5,1);
	    charger_vol = charger_vol / 100;

		//oppo_chg_log("check_charger_current_ability", "charger_vol=%d \r\n", charger_vol);
		//oppo_chg_log("current_i", "current_i=%d \r\n", i);

        charger_current_abilty = i;        
        oppo_actual_cc_volue = i;
		if(charger_vol < CHARGER_VOLTAGE_THRESHOLD)
		{
		    charger_current_abilty = i-1;
            break;
		}
#endif	
		oppo_chg_sleep_ms(1);
	}
	ChargerHwInit_bq24156a();
}

/*static kal_uint8 if_check_charger_ability_bq24156a(void)
{
    if(g_temp_CC_value > check_charger_ability_fg)
    {
        return KAL_TRUE;
    }
    return KAL_FALSE;
}*/

static void set_charge_adaptive_current_bq24156a(void)
{
    UINT8 i;
    UINT32 charger_vol;
    UINT8 dpmStatus;

    if(g_temp_CC_value > charger_current_abilty)
    {
        oppo_actual_cc_volue = charger_current_abilty;
    }
    else
    {
        oppo_actual_cc_volue = g_temp_CC_value;
    }

    bq24156a_set_reset(1);
    oppo_chg_sleep_ms(5);//wait for reset complete

    #ifdef OPPO_BQ24157_R68
    if(( BMT_status.charger_type == CHARGING_HOST ) || ( BMT_status.charger_type == STANDARD_CHARGER ))//(USB_CHARGER_CURRENT < oppo_actual_cc_volue)
    {
        bq24156a_set_lin_limit(0x03);//no limit
		
    }
    else
    {
        bq24156a_set_lin_limit(0x01);//500ma
    }
    #else
    if(( BMT_status.charger_type == CHARGING_HOST ) || ( BMT_status.charger_type == STANDARD_CHARGER ))//(USB_CHARGER_CURRENT < oppo_actual_cc_volue)
    {
        bq24156a_set_lin_limit(0x02);//800ma
		
    }
    else
    {
        bq24156a_set_lin_limit(0x01);//500ma
    }
    #endif
    
    bq24156a_set_termination_current(0x04);// 34+136=170
	if(SUPER_LOW_CHARGE_TEMP_STATUS == oppo_charge_temp_status)
	{
	    #ifdef OPPO_HIGH_BATTERY_VOLTAGE
	    if(oppo_high_battery_status == 1) bq24156a_set_cv_vth(BQ24156A_HI_BAT_SUPER_LOW_DCDC_OREG);
	    else
	    #endif
	    bq24156a_set_cv_vth(BQ24156A_SUPER_LOW_DCDC_OREG);
	}
	else if(LOW_CHARGE_TEMP_STATUS == oppo_charge_temp_status)
	{
	    #ifdef OPPO_HIGH_BATTERY_VOLTAGE
	    if(oppo_high_battery_status == 1) bq24156a_set_cv_vth(BQ24156A_HI_BAT_LOW_DCDC_OREG);
	    else
	    #endif
	    bq24156a_set_cv_vth(BQ24156A_LOW_DCDC_OREG);
	}
	else if(HIGH_CHARGE_TEMP_STATUS == oppo_charge_temp_status)
	{
	    #ifdef OPPO_HIGH_BATTERY_VOLTAGE
	    if(oppo_high_battery_status == 1) bq24156a_set_cv_vth(BQ24156A_HI_BAT_HIGH_DCDC_OREG);
	    else
	    #endif
	    bq24156a_set_cv_vth(BQ24156A_HIGH_DCDC_OREG);
	}
	else
	{
	    #ifdef OPPO_HIGH_BATTERY_VOLTAGE
	    if(oppo_high_battery_status == 1) bq24156a_set_cv_vth(BQ24156A_HI_BAT_NORMAL_DCDC_OREG);
	    else
	    #endif
	    bq24156a_set_cv_vth(BQ24156A_NORMAL_DCDC_OREG);
	}

    bq24156a_set_ce(0);//charger enable
    for(i=BQ24156A_CURRENT_221MA;i<=oppo_actual_cc_volue;i++)
    {
        upmu_chr_vbat_cs_vth_by_bq24156a(i);//CS_VTH
		oppo_chg_sleep_ms(5);

#if 0
		dpmStatus = bq24156a_get_dpm_status();
		
		//oppo_chg_log("set_charge_adaptive_current", "current_i=%d \r\n", i);

		if(0x01 == dpmStatus)//B4 DPM_STATUE
		{
		    charger_current_abilty = i-1;
		    oppo_actual_cc_volue = i;
            break;
		}
#else
		/* Get V_Charger */
	    charger_vol = PMIC_IMM_GetOneChannelValue(AUXADC_CHARGER_VOLTAGE_CHANNEL,5,1);
	    charger_vol = charger_vol / 100;
		
		//oppo_chg_log("set_charge_adaptive_current", "current_i=%d \r\n", i);

		if(charger_vol < CHARGER_VOLTAGE_THRESHOLD)
		{
		    charger_current_abilty = i-1;
		    oppo_actual_cc_volue = i;
            break;
		}
#endif
	}
	ChargerHwInit_bq24156a();
}

static void close_charge_adaptive_current_bq24156a(void)
{
    while(1)
    {
        if(oppo_actual_cc_volue <= BQ24156A_CURRENT_221MA)
        {
            oppo_actual_cc_volue = ZERO_CHARGER_CURRENT;
            upmu_chr_vbat_cs_vth_by_bq24156a(oppo_actual_cc_volue);//CS_VTH

            //oppo_chg_log("close_charge_adaptive_current", "oppo_actual_cc_volue=%d \r\n", oppo_actual_cc_volue);
            break;
        }
        else
        {
            oppo_actual_cc_volue--;
            upmu_chr_vbat_cs_vth_by_bq24156a(oppo_actual_cc_volue);//CS_VTH
		    oppo_chg_sleep_ms(5);

		    //oppo_chg_log("close_charge_adaptive_current", "oppo_actual_cc_volue=%d \r\n", oppo_actual_cc_volue);
        }
	}
}
#endif /*OPPO_USE_II_BQ24156A*/

#ifdef OPPO_FACTORY_TEST
static void set_at_charger_current(void)
{
    if(FAN5405_ID == oppo_excharger_id)
    {
        set_at_charger_current_fan5405();
    }
    else if(BQ24156A_ID == oppo_excharger_id)
    {
        set_at_charger_current_bq24156a();
    }
    else//BQ24157
    {
        set_at_charger_current_bq24156a();
    }
}
#endif

static void check_charger_current_ability(void)
{
    if(FAN5405_ID == oppo_excharger_id)
    {
        check_charger_current_ability_fan5405();
    }
    else if(BQ24156A_ID == oppo_excharger_id)
    {
        check_charger_current_ability_bq24156a();
    }
    else//BQ24157
    {
        check_charger_current_ability_bq24156a();
    }
}

static kal_uint8 if_check_charger_ability(void)
{
    if(g_temp_CC_value > check_charger_ability_fg)
    {
        return KAL_TRUE;
    }
    return KAL_FALSE;
}

static void set_charge_adaptive_current(void)
{
    if(FAN5405_ID == oppo_excharger_id)
    {
        set_charge_adaptive_current_fan5405();
    }
    else if(BQ24156A_ID == oppo_excharger_id)
    {
        set_charge_adaptive_current_bq24156a();
    }
    else//BQ24157
    {
        set_charge_adaptive_current_bq24156a();
    }
}

static void close_charge_adaptive_current(void)
{
    if(FAN5405_ID == oppo_excharger_id)
    {
        close_charge_adaptive_current_fan5405();
    }
    else if(BQ24156A_ID == oppo_excharger_id)
    {
        close_charge_adaptive_current_bq24156a();
    }
    else//BQ24157
    {
        close_charge_adaptive_current_bq24156a();
    }
}

#else /*OPPO_USE_EXCHARGER*/
#ifdef OPPO_FACTORY_TEST
static void set_at_charger_current(void)
{
    upmu_chr_vbat_cs_vth(AT_CHARGER_CURRENT);//CS_VTH
}
#endif

static void check_charger_current_ability(void)
{
    UINT8 i,temp_step;
    UINT32 charger_vol;

    temp_step = check_charger_ability_fg;
    
    for(i=Cust_CC_200MA;i>temp_step-1;i--)
    {
        upmu_chr_vbat_cs_vth(i);//CS_VTH
		oppo_chg_sleep_ms(5);
		
		/* Get V_Charger */
	    charger_vol = PMIC_IMM_GetOneChannelValue(AUXADC_CHARGER_VOLTAGE_CHANNEL,5,1);
	    charger_vol = charger_vol / 100;
		
		//oppo_chg_log("check_charger_current_ability", "charger_vol=%d \r\n", charger_vol);
		//oppo_chg_log("current_i", "current_i=%d \r\n", i);

        charger_current_abilty = i;        
        oppo_actual_cc_volue = i;
		if(charger_vol < CHARGER_VOLTAGE_THRESHOLD)
		{
		    charger_current_abilty = i+1;
            break;
		}
	}
}

static kal_uint8 if_check_charger_ability(void)
{
    if(g_temp_CC_value < check_charger_ability_fg)
    {
        return KAL_TRUE;
    }
    return KAL_FALSE;
}

static void set_charge_adaptive_current(void)
{
    UINT8 i;
    UINT32 charger_vol;

    if(g_temp_CC_value < charger_current_abilty)
    {
        oppo_actual_cc_volue = charger_current_abilty;
    }
    else
    {
        oppo_actual_cc_volue = g_temp_CC_value;
    }
    for(i=Cust_CC_70MA;i>oppo_actual_cc_volue-1;i--)
    {
		upmu_chr_vbat_cs_vth(i);//CS_VTH
		oppo_chg_sleep_ms(5);

		/* Get V_Charger */
	    charger_vol = PMIC_IMM_GetOneChannelValue(AUXADC_CHARGER_VOLTAGE_CHANNEL,5,1);
	    charger_vol = charger_vol / 100;
		
		//oppo_chg_log("set_charge_adaptive_current", "current_i=%d \r\n", i);

		if(charger_vol < CHARGER_VOLTAGE_THRESHOLD)
		{
		    if(i == Cust_CC_70MA)
		    {
		        charger_current_abilty = Cust_CC_0MA;
		    }
		    else
		    {
		        charger_current_abilty = i+1;
		    }
		    oppo_actual_cc_volue = i;
            break;
		}
	}
}

static void close_charge_adaptive_current(void)
{
    while(1)
    {
        if(oppo_actual_cc_volue >= Cust_CC_70MA)//<70mA
        {
            oppo_actual_cc_volue = Cust_CC_0MA;
            upmu_chr_vbat_cs_vth(oppo_actual_cc_volue);//CS_VTH

            //oppo_chg_log("close_charge_adaptive_current", "oppo_actual_cc_volue=%d \r\n", oppo_actual_cc_volue);
            break;
        }
        else
        {
            oppo_actual_cc_volue++;
            upmu_chr_vbat_cs_vth(oppo_actual_cc_volue);//CS_VTH
		    oppo_chg_sleep_ms(5);

		    //oppo_chg_log("close_charge_adaptive_current", "oppo_actual_cc_volue=%d \r\n", oppo_actual_cc_volue);
        }
	}
}
#endif /*OPPO_USE_EXCHARGER*/
#endif /*OPPO_CHARGE_CURRENT_ADAPTIVE*/
#ifdef OPPO_UBOOT_CHARGING
static kal_bool OPPO_BAT_PhyCheck_PreOverTime(void)
{
	if(OPPO_MAX_PRE_CHARGING_TIME <= BMT_status.PRE_charging_time)
	{
		return KAL_FALSE;
	}
	else
	{
		return KAL_TRUE;
	}
}
#endif

#ifndef OPPO_CMCC_TEST
static kal_bool OPPO_BAT_PhyCheck_OverTime(void)
{
    UINT32 charing_time;
    
    if((BMT_status.charger_type == STANDARD_CHARGER) || (BMT_status.charger_type == CHARGING_HOST))
    {
        charing_time = OPPO_MAX_CHARGING_TIME;
    }
    else
    {
        charing_time = OPPO_MAX_USBCHARGING_TIME;
    }
    
	if(charing_time <= BMT_status.total_charging_time)
	{
		return KAL_FALSE;
	}
	else
	{
		return KAL_TRUE;
	}
}
#endif

#if 0
static kal_bool OPPO_BAT_PhyCheck_LowCurrentCharger(void)
{
	if(Cust_CC_0MA == charger_current_abilty)
	{
		return KAL_FALSE;
	}
	else
	{
		return KAL_TRUE;
	}
}
#endif

static kal_bool OPPO_BAT_PhyCheck_LowCharger(void)
{
    if(oppo_protect_to_hold_status & LOW_CHARGER_PROTECT_FG)
    {
        if(oppo_check_charger_off_vol > OPPO_V_CHARGER_MIN + 200)
        {
            low_charger_count = 0;
            return KAL_TRUE;
        }
        return KAL_FALSE;
    }
    
	if(oppo_check_charger_off_vol <= OPPO_V_CHARGER_MIN )
	{
	    if(low_charger_count <= LOW_VCHARGER_CHECK_COUNT)
	    {
		    low_charger_count++;
		}
	}
	else
	{			
		low_charger_count = 0;
	}
	
	if(low_charger_count > LOW_VCHARGER_CHECK_COUNT)
	{
		return KAL_FALSE;
	}
	
	return KAL_TRUE;
}

static kal_bool OPPO_BAT_PhyCheck_OverCharger(void)
{
    if(oppo_protect_to_hold_status & OVER_CHARGER_PROTECT_FG)
    {
        if(oppo_check_charger_off_vol < OPPO_V_CHARGER_MAX - 200)
        {
            over_charger_count = 0;
            return KAL_TRUE;
        }
        return KAL_FALSE;
    }
    
	if(oppo_check_charger_off_vol >= OPPO_V_CHARGER_MAX)
	{
	    if(over_charger_count <= OVER_VCHARGER_CHECK_COUNT)
	    {
		    over_charger_count++;
		}
	}
	else
	{			
		over_charger_count = 0;
	}
	
	if(over_charger_count > OVER_VCHARGER_CHECK_COUNT)
	{
		return KAL_FALSE;
	}
	
	return KAL_TRUE;
}

static kal_bool OPPO_BAT_PhyCheck_VBat(void)
{
    if(oppo_check_vbat_off_vol > MAX_VBAT_LI)
	{
		max_vbat_count++;
	}
	else
	{
		max_vbat_count = 0;
	}
	
	if (max_vbat_count > MAX_VBAT_CHECK_COUNT)
	{
		return KAL_FALSE;
	}
	
	return KAL_TRUE;
}


static kal_bool OPPO_BAT_PhyCheck_BadBattContact(void)
{
    if(BMT_status.temperature <= OPPO_BAD_CHARGE_TEMPERATURE)
	{
		bad_temper_count++;
	}
	else
	{
		bad_temper_count = 0;
	}
	
	if (bad_temper_count > BAD_BATTEMP_CHECK_COUNT)
	{
		return KAL_FALSE;
	}
	
	return KAL_TRUE;
}

static kal_bool OPPO_BAT_PhyCheck_LowBatTemp(void)
{
    if((BMT_status.temperature > OPPO_BAD_CHARGE_TEMPERATURE) && (BMT_status.temperature <= OPPO_MIN_CHARGE_TEMPERATURE))
	{
		low_temper_count++;
	}
	else
	{
		low_temper_count = 0;
	}
	
	if (low_temper_count > LOW_BATTEMP_CHECK_COUNT)
	{
		return KAL_FALSE;
	}
	
	return KAL_TRUE;
}

static kal_bool OPPO_BAT_PhyCheck_OverBatTemp(void)
{
	if(BMT_status.temperature >= OPPO_MAX_CHARGE_TEMPERATURE)
	{
		over_temper_count++;
	}
	else
	{
		over_temper_count = 0;
	}
	
	if (over_temper_count > OVER_BATTEMP_CHECK_COUNT)
	{
		over_temper_count = 0;
		return KAL_FALSE;
	}
	
	return KAL_TRUE;
}

static void OPPO_BAT_PhyCheck_BatTempStatus(void)
{
    INT32 nBatTemp = BMT_status.temperature;

    oppo_chg_log("Power/Battery", "OPPO_BAT_PhyCheck_BatTempStatus= %d\r\n", nBatTemp);
    
    if(OPPO_HIGH_CHARGE_TEMPERATURE <= nBatTemp)
	{
	    if(high_temp_status_count < BATTEMP_STATUS_CHECK_COUNT)
	    {
	        high_temp_status_count++;
	    }
	    super_low_temp_status_count = 0;
	    low_temp_status_count = 0;
	    normal_temp_status_count = 0;
	}
	else if(/*(OPPO_MIN_CHARGE_TEMPERATURE <= nBatTemp)&&*/(OPPO_SUPER_LOW_CHARGE_TEMPERATURE > nBatTemp))
	{
	    high_temp_status_count = 0;
	    if(super_low_temp_status_count < BATTEMP_STATUS_CHECK_COUNT)
	    {
	        super_low_temp_status_count++;
	    }
	    low_temp_status_count = 0;
	    normal_temp_status_count = 0;
	}
	else if((OPPO_SUPER_LOW_CHARGE_TEMPERATURE <= nBatTemp)&&(OPPO_LOW_CHARGE_TEMPERATURE > nBatTemp))
	{
	    high_temp_status_count = 0;
	    super_low_temp_status_count = 0;
	    if(low_temp_status_count < BATTEMP_STATUS_CHECK_COUNT)
	    {
	        low_temp_status_count++;
	    }
	    normal_temp_status_count = 0;
	}
	else
	{
	    high_temp_status_count = 0;
	    super_low_temp_status_count = 0;
	    low_temp_status_count = 0;
	    if(normal_temp_status_count < BATTEMP_STATUS_CHECK_COUNT)
	    {
	        normal_temp_status_count++;
	    }
	}

    if(BATTEMP_STATUS_CHECK_COUNT <= high_temp_status_count)
    {
        oppo_charge_temp_status = HIGH_CHARGE_TEMP_STATUS;
    }
    else if(BATTEMP_STATUS_CHECK_COUNT <= super_low_temp_status_count)
    {
        oppo_charge_temp_status = SUPER_LOW_CHARGE_TEMP_STATUS;
    }	
    else if(BATTEMP_STATUS_CHECK_COUNT <= low_temp_status_count)
    {
        oppo_charge_temp_status = LOW_CHARGE_TEMP_STATUS;
    }
    else if(BATTEMP_STATUS_CHECK_COUNT <= normal_temp_status_count)
    {
        oppo_charge_temp_status = NORMAL_CHARGE_TEMP_STATUS;
    }
}

static void OPPO_BAT_PhyCheck(void)
{
    if( upmu_is_chr_det() == KAL_TRUE)
    {
        #ifdef OPPO_USE_EXCHARGER
        if(BQ24156A_ID == oppo_excharger_id)
        {
            check_bq24156a_status();
        }
        else if(BQ24157_ID == oppo_excharger_id)
        {
            check_bq24156a_status();
        }
        else if(FAN5405_ID == oppo_excharger_id)
        {
            check_fan5405_status();
        }
        #endif
        
		#ifdef OPPO_UBOOT_CHARGING
        if(KAL_FALSE == OPPO_BAT_PhyCheck_PreOverTime())
        {
            oppo_chg_log("Power/Battery", "[BATTERY] Battery PreOver Time !!\n\r");
            
            g_Charging_Over_Time = 1;            
    		BMT_status.bat_charging_state = CHR_ERROR;
            return;
        }
        #endif
        
        #ifndef OPPO_CMCC_TEST
        if(KAL_FALSE == OPPO_BAT_PhyCheck_OverTime())
        {
            oppo_chg_log("Power/Battery", "[BATTERY] Battery Over Time !!\n\r");
            
            g_Charging_Over_Time = 1;
            #ifdef OPPO_UBOOT_CHARGING
            if(CHR_ERROR != BMT_status.bat_charging_state)
            {
                bl_switch = KAL_FALSE;
            }
            #endif
            g_BatteryNotifyCode |= 1 << NOTIFY_OVER_TIME;
    		BMT_status.bat_charging_state = CHR_ERROR;
            return;
        }
        #endif

        #if 0
        if(KAL_FALSE == OPPO_BAT_PhyCheck_LowCurrentCharger())
        {
            oppo_chg_log("Power/Battery", "[BATTERY] Battery Low Current Charger !!\n\r");
            
            #ifdef OPPO_UBOOT_CHARGING
            if(CHR_ERROR != BMT_status.bat_charging_state)
            {
                bl_switch = KAL_FALSE;
            }
            #endif
            g_BatteryNotifyCode |= 1 << Notify_Charger_Low_Vol;          
            BMT_status.bat_charging_state = CHR_ERROR;
            return;
        }
        #endif

        if((low_charger_error_count & 0x7F) >= LOW_CHARGER_ERROR_RESTORING)
        {
            oppo_chg_log("Power/Battery", "[BATTERY] Battery Low Charger !!\n\r");
            
            #ifdef OPPO_UBOOT_CHARGING
            if(CHR_ERROR != BMT_status.bat_charging_state)
            {
                bl_switch = KAL_FALSE;
            }
            #endif
            oppo_protect_to_hold_status = oppo_protect_to_hold_status & (~LOW_CHARGER_PROTECT_FG);
            g_BatteryNotifyCode |= 1 << NOTIFY_LOW_VCHARGER;
            BMT_status.bat_charging_state = CHR_ERROR;
            return;
        }
        else
        {
            if(KAL_FALSE == OPPO_BAT_PhyCheck_LowCharger())
            {
                oppo_chg_log("Power/Battery", "[BATTERY] Battery Low Charger !!\n\r");
                
                #ifdef OPPO_UBOOT_CHARGING
                if(KAL_FALSE == (oppo_protect_to_hold_status & OVER_CHARGER_PROTECT_FG))
                {
                    bl_switch = KAL_FALSE;
                }
                #endif
                if((low_charger_error_count & 0x80) == 0)
                {
                    low_charger_error_count++;
                }
                low_charger_error_count = low_charger_error_count | 0x80;
                
                g_BatteryNotifyCode |= 1 << NOTIFY_LOW_VCHARGER;
                oppo_protect_to_hold_status = oppo_protect_to_hold_status | LOW_CHARGER_PROTECT_FG;
                return;
            }
            else
            {
                oppo_protect_to_hold_status = oppo_protect_to_hold_status & (~LOW_CHARGER_PROTECT_FG);
                low_charger_error_count = low_charger_error_count & 0x7F;

                g_BatteryNotifyCode &= ~ (1 << NOTIFY_LOW_VCHARGER);
            }
        }
        
        if((over_charger_error_count & 0x7F) >= OVER_CHARGER_ERROR_RESTORING)
        {
            oppo_chg_log("Power/Battery", "[BATTERY] Battery Over Charger !!\n\r");
            
            #ifdef OPPO_UBOOT_CHARGING
            if(CHR_ERROR != BMT_status.bat_charging_state)
            {
                bl_switch = KAL_FALSE;
            }
            #endif
            oppo_protect_to_hold_status = oppo_protect_to_hold_status & (~OVER_CHARGER_PROTECT_FG);
            g_BatteryNotifyCode |= 1 << NOTIFY_OVER_VCHARGER;            
            BMT_status.bat_charging_state = CHR_ERROR;
            return;
        }
        else
        {
            if(KAL_FALSE == OPPO_BAT_PhyCheck_OverCharger())
            {
                oppo_chg_log("Power/Battery", "[BATTERY] Battery Over Charger !!\n\r");
                
                #ifdef OPPO_UBOOT_CHARGING
                if(KAL_FALSE == (oppo_protect_to_hold_status & OVER_CHARGER_PROTECT_FG))
                {
                    bl_switch = KAL_FALSE;
                }
                #endif
                if((over_charger_error_count & 0x80) == 0)
                {
                    over_charger_error_count++;
                }
                over_charger_error_count = over_charger_error_count | 0x80;
                
                g_BatteryNotifyCode |= 1 << NOTIFY_OVER_VCHARGER;                
                oppo_protect_to_hold_status = oppo_protect_to_hold_status | OVER_CHARGER_PROTECT_FG;
                return;
            }
            else
            {
                oppo_protect_to_hold_status = oppo_protect_to_hold_status & (~OVER_CHARGER_PROTECT_FG);
                over_charger_error_count = over_charger_error_count & 0x7F;

                g_BatteryNotifyCode &= ~(1 << NOTIFY_OVER_VCHARGER);
            }
        }

        if(KAL_FALSE == OPPO_BAT_PhyCheck_VBat())
        {
            oppo_chg_log("Power/Battery", "[BATTERY] Battery VBat Over !!\n\r");
            
            #ifdef OPPO_UBOOT_CHARGING
            if(CHR_ERROR != BMT_status.bat_charging_state)
            {
                bl_switch = KAL_FALSE;
            }
            #endif
            g_BatteryNotifyCode |= 1 << NOTIFY_OVER_VBAT;
    		BMT_status.bat_charging_state = CHR_ERROR;
            return;
        }

        if(KAL_FALSE == OPPO_BAT_PhyCheck_BadBattContact())
        {
            oppo_chg_log("Power/Battery", "[BATTERY] Battery Bad Contact !!\n\r");
            
            #ifdef OPPO_UBOOT_CHARGING
            if(CHR_ERROR != BMT_status.bat_charging_state)
            {
                bl_switch = KAL_FALSE;
            }
            #endif
            g_BatteryNotifyCode |= 1 << NOTIFY_BAD_BATTEMP;
    		BMT_status.bat_charging_state = CHR_ERROR;
            return;
        }
        
        if(KAL_FALSE == OPPO_BAT_PhyCheck_LowBatTemp())
        {
            oppo_chg_log("Power/Battery", "[BATTERY] Battery Under Temperature or NTC fail !!\n\r");
            
            #ifdef OPPO_UBOOT_CHARGING
            if(CHR_ERROR != BMT_status.bat_charging_state)
            {
                bl_switch = KAL_FALSE;
            }
            #endif
            g_BatteryNotifyCode |= 1 << NOTIFY_LOW_BATTEMP;
    		BMT_status.bat_charging_state = CHR_ERROR;
            return;
        }
        
    	if(KAL_FALSE == OPPO_BAT_PhyCheck_OverBatTemp())
        {
            oppo_chg_log("Power/Battery", "[BATTERY] Battery Over Temperature !!\n\r");
            
            #ifdef OPPO_UBOOT_CHARGING
            if(CHR_ERROR != BMT_status.bat_charging_state)
            {
                bl_switch = KAL_FALSE;
            }
            #endif
            g_BatteryNotifyCode |= 1 << NOTIFY_OVER_BATTEMP;
    		BMT_status.bat_charging_state = CHR_ERROR;
            return;
        }

        #ifdef OPPO_CALL_MODE_CHARGING_PAUSE
        oppo_check_call_pause_mode();
        #endif

        OPPO_BAT_PhyCheck_BatTempStatus();

        g_BatteryNotifyCode |=  1 << NOTIFY_CHARGER_EXIST;
	}
	else
	{
	    g_BatteryNotifyCode = 0x0000;
	    g_NotifyFlag = 0;
	}
}

static void OPPO_BAT_Charge(void)
{
#if 1
	if(oppo_DC_charger_init_sign)
	{
    	if(FAN5405_ID == oppo_excharger_id)				//#ifdef OPPO_USE_FAIRCHILD_FAN5405
    		fan5405_config_interface_liao(0x00,0x80);//reset safty timer
    	else if(BQ24156A_ID == oppo_excharger_id)		//#elif defined(OPPO_USE_II_BQ24156A)
    		bq24156a_set_tmr_rst(1);//reset safty timer
    	else if(BQ24157_ID == oppo_excharger_id)		//#elif defined(OPPO_USE_II_BQ24157)
    		bq24156a_set_tmr_rst(1);//reset safty timer
    													//#else    
														//endif
	}
#endif
    
    if( upmu_is_chr_det() == KAL_FALSE)
    {
        pchr_turn_off_charging();
        return;
    }
    
    switch(BMT_status.bat_charging_state)
    {            
        case CHR_PRE :
			BAT_PreChargeModeAction();
            break;
            
        case CHR_CC :
            BAT_ConstantCurrentModeAction();
            break;
            
        case CHR_TOP_OFF :
            BAT_TopOffModeAction();
            break;
			
        case CHR_POST_FULL :
            BAT_PostFullModeAction();
            break;

        case CHR_BATFULL:
			BAT_BatteryFullAction();
            break;
			
        case CHR_ERROR:
			BAT_BatteryStatusFailAction();
            break;
    }
}

#ifdef OPPO_KERNEL_CHARGING
void OPPO_CHG_Para_Init(void)
{
    #ifdef OPPO_USE_EXCHARGER
    g_temp_CC_value = ZERO_CHARGER_CURRENT;
    if(FAN5405_ID == oppo_excharger_id)
    {
        USB_CHARGER_CURRENT = FAN5405_USB_CHARGER_CURRENT;
        AC_CHARGER_CURRENT = FAN5405_AC_CHARGER_CURRENT;
        ZERO_CHARGER_CURRENT = FAN5405_ZERO_CHARGER_CURRENT;
        INIT_CHARGER_CURRENT = FAN5405_INIT_CHARGER_CURRENT;
        SUPER_LOW_TEMP_CHARGER_CURRENT = FAN5405_SUPER_LOW_TEMP_CHARGER_CURRENT;
        LOW_TEMP_CHARGER_CURRENT = FAN5405_LOW_TEMP_CHARGER_CURRENT;
        LOW_TEMP_POSTFULL_CHARGER_CURRENT = FAN5405_LOW_TEMP_POSTFULL_CHARGER_CURRENT;
        HIGH_TEMP_LCD_OFF_CHARGER_CURRENT = FAN5405_HIGH_TEMP_LCD_OFF_CHARGER_CURRENT;
        HIGH_TEMP_LCD_ON_CHARGER_CURRENT = FAN5405_HIGH_TEMP_LCD_ON_CHARGER_CURRENT;
        AT_CHARGER_CURRENT = FAN5405_AT_CHARGER_CURRENT;
    }
    else if(BQ24156A_ID == oppo_excharger_id)
    {
        USB_CHARGER_CURRENT = BQ24156A_USB_CHARGER_CURRENT;
        AC_CHARGER_CURRENT = BQ24156A_AC_CHARGER_CURRENT;
        ZERO_CHARGER_CURRENT = BQ24156A_ZERO_CHARGER_CURRENT;
        INIT_CHARGER_CURRENT = BQ24156A_INIT_CHARGER_CURRENT;
        SUPER_LOW_TEMP_CHARGER_CURRENT = BQ24156A_SUPER_LOW_TEMP_CHARGER_CURRENT;
        LOW_TEMP_CHARGER_CURRENT = BQ24156A_LOW_TEMP_CHARGER_CURRENT;
        LOW_TEMP_POSTFULL_CHARGER_CURRENT = BQ24156A_LOW_TEMP_POSTFULL_CHARGER_CURRENT;
        HIGH_TEMP_LCD_OFF_CHARGER_CURRENT = BQ24156A_HIGH_TEMP_LCD_OFF_CHARGER_CURRENT;
        HIGH_TEMP_LCD_ON_CHARGER_CURRENT = BQ24156A_HIGH_TEMP_LCD_ON_CHARGER_CURRENT;
        AT_CHARGER_CURRENT = BQ24156A_AT_CHARGER_CURRENT;
    }
    else
    {
        USB_CHARGER_CURRENT = BQ24157_USB_CHARGER_CURRENT;
        AC_CHARGER_CURRENT = BQ24157_AC_CHARGER_CURRENT;
        ZERO_CHARGER_CURRENT = BQ24157_ZERO_CHARGER_CURRENT;
        INIT_CHARGER_CURRENT = BQ24157_INIT_CHARGER_CURRENT;
        SUPER_LOW_TEMP_CHARGER_CURRENT = BQ24157_SUPER_LOW_TEMP_CHARGER_CURRENT;
        LOW_TEMP_CHARGER_CURRENT = BQ24157_LOW_TEMP_CHARGER_CURRENT;
        LOW_TEMP_POSTFULL_CHARGER_CURRENT = BQ24157_LOW_TEMP_POSTFULL_CHARGER_CURRENT;
        HIGH_TEMP_LCD_OFF_CHARGER_CURRENT = BQ24157_HIGH_TEMP_LCD_OFF_CHARGER_CURRENT;
        HIGH_TEMP_LCD_ON_CHARGER_CURRENT = BQ24157_HIGH_TEMP_LCD_ON_CHARGER_CURRENT;
        AT_CHARGER_CURRENT = BQ24157_AT_CHARGER_CURRENT;
    }
    #endif
}
#endif
#endif/* OPPO_NEW_CHARGING */ 


///////////////////////////////////////////////////////////////////////////////////////////
//// fop API 
///////////////////////////////////////////////////////////////////////////////////////////
static long adc_cali_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int *user_data_addr;
    int *naram_data_addr;
    int i = 0;
    int ret = 0;

    #ifdef OPPO_FACTORY_TEST
    static int at_i_sense=0,at_bat_sense=0,at_current_offset=0;
    #endif

    mutex_lock(&bat_mutex);

    switch(cmd)
    {
        case TEST_ADC_CALI_PRINT :
            g_ADC_Cali = KAL_FALSE;
            break;
        
        case SET_ADC_CALI_Slop:            
            naram_data_addr = (int *)arg;
            ret = copy_from_user(adc_cali_slop, naram_data_addr, 36);
            g_ADC_Cali = KAL_FALSE; /* enable calibration after setting ADC_CALI_Cal */            
            /* Protection */
            for (i=0;i<14;i++) 
            { 
                if ( (*(adc_cali_slop+i) == 0) || (*(adc_cali_slop+i) == 1) ) {
                    *(adc_cali_slop+i) = 1000;
                }
            }
            for (i=0;i<14;i++) xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "adc_cali_slop[%d] = %d\n",i , *(adc_cali_slop+i));
            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "**** unlocked_ioctl : SET_ADC_CALI_Slop Done!\n");            
            break;    
            
        case SET_ADC_CALI_Offset:            
            naram_data_addr = (int *)arg;
            ret = copy_from_user(adc_cali_offset, naram_data_addr, 36);
            g_ADC_Cali = KAL_FALSE; /* enable calibration after setting ADC_CALI_Cal */
            for (i=0;i<14;i++) xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "adc_cali_offset[%d] = %d\n",i , *(adc_cali_offset+i));
            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "**** unlocked_ioctl : SET_ADC_CALI_Offset Done!\n");            
            break;
            
        case SET_ADC_CALI_Cal :            
            naram_data_addr = (int *)arg;
            ret = copy_from_user(adc_cali_cal, naram_data_addr, 4);
            g_ADC_Cali = KAL_TRUE;
            if ( adc_cali_cal[0] == 1 ) {
                g_ADC_Cali = KAL_TRUE;
            } else {
                g_ADC_Cali = KAL_FALSE;
            }            
            for (i=0;i<1;i++) xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "adc_cali_cal[%d] = %d\n",i , *(adc_cali_cal+i));
            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "**** unlocked_ioctl : SET_ADC_CALI_Cal Done!\n");            
            break;    

        case ADC_CHANNEL_READ:            
            //g_ADC_Cali = KAL_FALSE; /* 20100508 Infinity */
            user_data_addr = (int *)arg;
            ret = copy_from_user(adc_in_data, user_data_addr, 8); /* 2*int = 2*4 */
          
            if( adc_in_data[0] == 0 ) // I_SENSE
            {
                adc_out_data[0] = get_i_sense_volt(adc_in_data[1]) * adc_in_data[1];
            	#ifdef OPPO_FACTORY_TEST
            	adc_out_data[0] += at_current_offset;
            	#endif
            }
            else if( adc_in_data[0] == 1 ) // BAT_SENSE
            {
                adc_out_data[0] = get_bat_sense_volt(adc_in_data[1]) * adc_in_data[1];
            }
            #ifdef VENDOR_EDIT //Jiada.LI@ProDrv.chager, modify 2013/05/18 for AT adc test
            else if( adc_in_data[0] == 99 ) // CHARGE_OFF_ADC_BAT_SEN
            {
                #ifdef OPPO_USE_EXCHARGER
                if(FAN5405_ID == oppo_excharger_id)
                {
                    mt_set_gpio_mode(GPIO_FAN5405_DISABLE_NUM,GPIO_FAN5405_DISABLE_MODE);
                    mt_set_gpio_dir(GPIO_FAN5405_DISABLE_NUM,GPIO_FAN5405_DISABLE_DIR);
                    mt_set_gpio_out(GPIO_FAN5405_DISABLE_NUM,GPIO_FAN5405_DISABLE_OFF_OUT);
                }
                else if(BQ24156A_ID == oppo_excharger_id)
                {
                    mt_set_gpio_mode(GPIO_BQ24156A_DISABLE_NUM,GPIO_BQ24156A_DISABLE_MODE);  
                    mt_set_gpio_dir(GPIO_BQ24156A_DISABLE_NUM,GPIO_BQ24156A_DISABLE_DIR);
                    mt_set_gpio_out(GPIO_BQ24156A_DISABLE_NUM,GPIO_BQ24156A_DISABLE_OFF_OUT);
                }
                else//BQ24157
                {
                    mt_set_gpio_mode(GPIO_BQ24156A_DISABLE_NUM,GPIO_BQ24156A_DISABLE_MODE);  
                    mt_set_gpio_dir(GPIO_BQ24156A_DISABLE_NUM,GPIO_BQ24156A_DISABLE_DIR);
                    mt_set_gpio_out(GPIO_BQ24156A_DISABLE_NUM,GPIO_BQ24156A_DISABLE_OFF_OUT);
                }
    	        #else
				pchr_turn_off_charging();
				#endif
                oppo_chg_sleep_ms(100);
                adc_out_data[0] = get_bat_sense_volt(adc_in_data[1]) * adc_in_data[1];
                pchr_turn_on_charging();
            }
            #endif
            else if( adc_in_data[0] == 3 ) // V_Charger
            {
                adc_out_data[0] = get_charger_volt(adc_in_data[1]) * adc_in_data[1];
                adc_out_data[0] = adc_out_data[0] / 100;
            }    
            else if( adc_in_data[0] == 30 ) // V_Bat_temp magic number
            {                
                adc_out_data[0] = BMT_status.temperature;                
            }
            else if( adc_in_data[0] == 66 ) 
            {
                adc_out_data[0] = (gFG_current)/10;
                
                if (gFG_Is_Charging == KAL_TRUE) 
                {                    
                    adc_out_data[0] = 0 - adc_out_data[0]; //charging
                }                                
            }
            else
            {
                adc_out_data[0] = PMIC_IMM_GetOneChannelValue(adc_in_data[0],adc_in_data[1],1) * adc_in_data[1];
            }
            
            if (adc_out_data[0]<0)
                adc_out_data[1]=1; /* failed */
            else
                adc_out_data[1]=0; /* success */

            if( adc_in_data[0] == 30 )
                adc_out_data[1]=0; /* success */

            if( adc_in_data[0] == 66 )
                adc_out_data[1]=0; /* success */
                
            ret = copy_to_user(user_data_addr, adc_out_data, 8);
            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "**** unlocked_ioctl : Channel %d * %d times = %d\n", adc_in_data[0], adc_in_data[1], adc_out_data[0]);            
            break;

        case BAT_STATUS_READ:            
            user_data_addr = (int *)arg;
            ret = copy_from_user(battery_in_data, user_data_addr, 4); 
            /* [0] is_CAL */
            if (g_ADC_Cali) {
                battery_out_data[0] = 1;
            } else {
                battery_out_data[0] = 0;
            }
            ret = copy_to_user(user_data_addr, battery_out_data, 4); 
            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "**** unlocked_ioctl : CAL:%d\n", battery_out_data[0]);                        
            break;        

        case Set_Charger_Current: /* For Factory Mode*/
            user_data_addr = (int *)arg;
            ret = copy_from_user(charging_level_data, user_data_addr, 4);
            g_ftm_battery_flag = KAL_TRUE;            
            if( charging_level_data[0] == 0 ) {                charging_level_data[0] = Cust_CC_70MA;
            } else if ( charging_level_data[0] == 1  ) {    charging_level_data[0] = Cust_CC_200MA;
            } else if ( charging_level_data[0] == 2  ) {    charging_level_data[0] = Cust_CC_400MA;
            } else if ( charging_level_data[0] == 3  ) {    charging_level_data[0] = Cust_CC_450MA;
            } else if ( charging_level_data[0] == 4  ) {    charging_level_data[0] = Cust_CC_550MA;
            } else if ( charging_level_data[0] == 5  ) {    charging_level_data[0] = Cust_CC_650MA;
            } else if ( charging_level_data[0] == 6  ) {    charging_level_data[0] = Cust_CC_700MA;
            } else if ( charging_level_data[0] == 7  ) {    charging_level_data[0] = Cust_CC_800MA;
            } else if ( charging_level_data[0] == 8  ) {    charging_level_data[0] = Cust_CC_900MA;
            } else if ( charging_level_data[0] == 9  ) {    charging_level_data[0] = Cust_CC_1000MA;
            } else if ( charging_level_data[0] == 10 ) {    charging_level_data[0] = Cust_CC_1100MA;
            } else if ( charging_level_data[0] == 11 ) {    charging_level_data[0] = Cust_CC_1200MA;
            } else if ( charging_level_data[0] == 12 ) {    charging_level_data[0] = Cust_CC_1300MA;
            } else if ( charging_level_data[0] == 13 ) {    charging_level_data[0] = Cust_CC_1400MA;
            } else if ( charging_level_data[0] == 14 ) {    charging_level_data[0] = Cust_CC_1500MA;
            } else if ( charging_level_data[0] == 15 ) {    charging_level_data[0] = Cust_CC_1600MA;
            } else { 
                charging_level_data[0] = Cust_CC_450MA;
            }
            wake_up_bat();
            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "**** unlocked_ioctl : set_Charger_Current:%d\n", charging_level_data[0]);
            break;

        #ifdef OPPO_NEW_CHARGING
        case Get_FakeOff_Param: /* For Factory Mode*/
    		#ifdef OPPO_NEW_CHARGING 
            oppo_ipod_status = 1;
    		#endif
    
			user_data_addr = (int *)arg;
			fakeoff_out_data[0] = bat_volt_check_point;

			if(g_BatteryNotifyCode & (1 <<NOTIFY_OVER_TIME))
        	{
        	    g_NotifyFlag = NOTIFY_OVER_TIME;
        	}
        	else if(g_BatteryNotifyCode & (1 <<NOTIFY_LOW_VCHARGER))
        	{
        	    g_NotifyFlag = NOTIFY_LOW_VCHARGER;
        	}
        	else if(g_BatteryNotifyCode & (1 <<NOTIFY_OVER_VCHARGER))
        	{
        	    g_NotifyFlag = NOTIFY_OVER_VCHARGER;
        	}
        	else if(g_BatteryNotifyCode & (1 <<NOTIFY_OVER_VBAT))
        	{
        	    g_NotifyFlag = NOTIFY_OVER_VBAT;
        	}
        	else if(g_BatteryNotifyCode & (1 <<NOTIFY_BAD_BATTEMP))
        	{
        	    g_NotifyFlag = NOTIFY_BAD_BATTEMP;
        	}
        	else if(g_BatteryNotifyCode & (1 <<NOTIFY_LOW_BATTEMP))
        	{
        	    g_NotifyFlag = NOTIFY_LOW_BATTEMP;
        	}
        	else if(g_BatteryNotifyCode & (1 <<NOTIFY_OVER_BATTEMP))
        	{
        	    g_NotifyFlag = NOTIFY_OVER_BATTEMP;
        	}
        	else if(g_BatteryNotifyCode & (1 <<NOTIFY_BAT_FULL))
        	{
        	    //g_NotifyFlag = NOTIFY_BAT_FULL;
        	    
        	    if(g_BatteryNotifyCode & (1 <<NOTIFY_LOWTEMP_BAT_FULL))
                {
                    g_NotifyFlag = NOTIFY_LOWTEMP_BAT_FULL;
                }
                else if(g_BatteryNotifyCode & (1 <<NOTIFY_HIGHTEMP_BAT_FULL))
                {
                    g_NotifyFlag = NOTIFY_HIGHTEMP_BAT_FULL;
                }
                #ifdef OPPO_HIGH_BATTERY_VOLTAGE
                else if(g_BatteryNotifyCode & (1 <<NOTIFY_THIRD_BAT_FULL))
                {
                    g_NotifyFlag = NOTIFY_THIRD_BAT_FULL;
                }
                #endif
                else
                {
                    g_NotifyFlag = NOTIFY_NORMAL_BAT_FULL;
                }
        	}
        	else if(g_BatteryNotifyCode & (1 <<NOTIFY_BAT_POSTFULL))
        	{
        	    g_NotifyFlag = NOTIFY_BAT_POSTFULL;
        	}
        	else
        	{
        	    g_NotifyFlag = 0;
        	}
			fakeoff_out_data[1] = g_NotifyFlag;
			
            if(upmu_is_chr_det() == KAL_TRUE)
            {
				fakeoff_out_data[2] = 1;
			}
			else
			{
				fakeoff_out_data[2] = 0;
			}
			ret = copy_to_user(user_data_addr, fakeoff_out_data, 12);
            printk("ioctl : Get_FakeOff_Param:bat_volt_check_point:%d, g_BatteryNotifyCode:%d,chr_det:%d\n",fakeoff_out_data[0],fakeoff_out_data[1],fakeoff_out_data[2]);
            break; 
			
		case Get_Notify_Param: /* For Fakeoff Mode*/
			user_data_addr = (int *)arg;

			if(g_BatteryNotifyCode & (1 <<NOTIFY_OVER_TIME))
        	{
        	    g_NotifyFlag = NOTIFY_OVER_TIME;
        	}
        	else if(g_BatteryNotifyCode & (1 <<NOTIFY_LOW_VCHARGER))
        	{
        	    g_NotifyFlag = NOTIFY_LOW_VCHARGER;
        	}
        	else if(g_BatteryNotifyCode & (1 <<NOTIFY_OVER_VCHARGER))
        	{
        	    g_NotifyFlag = NOTIFY_OVER_VCHARGER;
        	}
        	else if(g_BatteryNotifyCode & (1 <<NOTIFY_OVER_VBAT))
        	{
        	    g_NotifyFlag = NOTIFY_OVER_VBAT;
        	}
        	else if(g_BatteryNotifyCode & (1 <<NOTIFY_BAD_BATTEMP))
        	{
        	    g_NotifyFlag = NOTIFY_BAD_BATTEMP;
        	}
        	else if(g_BatteryNotifyCode & (1 <<NOTIFY_LOW_BATTEMP))
        	{
        	    g_NotifyFlag = NOTIFY_LOW_BATTEMP;
        	}
        	else if(g_BatteryNotifyCode & (1 <<NOTIFY_OVER_BATTEMP))
        	{
        	    g_NotifyFlag = NOTIFY_OVER_BATTEMP;
        	}
        	else if(g_BatteryNotifyCode & (1 <<NOTIFY_BAT_FULL))
        	{
        	    g_NotifyFlag = NOTIFY_BAT_FULL;
        	}
        	else if(g_BatteryNotifyCode & (1 <<NOTIFY_BAT_POSTFULL))
        	{
        	    g_NotifyFlag = NOTIFY_BAT_POSTFULL;
        	}
        	else
        	{
        	    g_NotifyFlag = 0;
        	}
			notify_out_data[0] = g_NotifyFlag;
			
			ret = copy_to_user(user_data_addr, notify_out_data, 4);
            printk("ioctl : Get_Notify_Param: g_BatteryNotifyCode:%d\n",g_BatteryNotifyCode); 
            break; 	
		case Turn_Off_Charging: /* For Turnoffcharging Mode*/
		    #ifdef OPPO_USE_EXCHARGER
            if(FAN5405_ID == oppo_excharger_id)
            {
                mt_set_gpio_mode(GPIO_FAN5405_DISABLE_NUM,GPIO_FAN5405_DISABLE_MODE);
                mt_set_gpio_dir(GPIO_FAN5405_DISABLE_NUM,GPIO_FAN5405_DISABLE_DIR);
                mt_set_gpio_out(GPIO_FAN5405_DISABLE_NUM,GPIO_FAN5405_DISABLE_OFF_OUT);
            }
            else if(BQ24156A_ID == oppo_excharger_id)
            {
                mt_set_gpio_mode(GPIO_BQ24156A_DISABLE_NUM,GPIO_BQ24156A_DISABLE_MODE);  
                mt_set_gpio_dir(GPIO_BQ24156A_DISABLE_NUM,GPIO_BQ24156A_DISABLE_DIR);
                mt_set_gpio_out(GPIO_BQ24156A_DISABLE_NUM,GPIO_BQ24156A_DISABLE_OFF_OUT);
            }
            else//BQ24157
            {
                mt_set_gpio_mode(GPIO_BQ24156A_DISABLE_NUM,GPIO_BQ24156A_DISABLE_MODE);  
                mt_set_gpio_dir(GPIO_BQ24156A_DISABLE_NUM,GPIO_BQ24156A_DISABLE_DIR);
                mt_set_gpio_out(GPIO_BQ24156A_DISABLE_NUM,GPIO_BQ24156A_DISABLE_OFF_OUT);
            }
	        #else
            pchr_turn_off_charging();
            #endif
            //struExternalBMT.bChgingOn = KAL_FALSE;
            #ifdef OPPO_FACTORY_TEST
            oppo_charging_stop_fg = KAL_TRUE;
            #endif
			break;

		case K_AT_CHG_CHGR_IN:
		    user_data_addr = (int *)arg;
		    if(BMT_status.charger_exist)
		    {
		        auto_out_data[0]=1;
                if(BMT_status.charger_type==STANDARD_CHARGER)
                {
                    auto_out_data[1]=0x01;
                }
                else if(BMT_status.charger_type==NONSTANDARD_CHARGER)
                {
                    auto_out_data[1]=0x02;
                }
                else
                {
                    auto_out_data[1]=0;//usb
                }
		    }
		    else
		    {
		        auto_out_data[0]=0;
		        auto_out_data[1]=0;
		    }
		    ret = copy_to_user(user_data_addr, auto_out_data, 8);
		    if(ret<0)
		    {
		        printk("AT_CHG_CHGR_IN,copy_to_user return val error\n");
		    }
		    

		    printk("AT_CHG_CHGR_ON\N");
		    for(i=0;i<3;i++)
		    {
		         printk("auto_out_data[%d]=%d\n",i,auto_out_data[i]);
		    }
		    break;
	    case K_AT_CHG_CHGR_OFF:
	        user_data_addr = (int *)arg;
	        if(BMT_status.charger_exist)
		    {
		        auto_out_data[0]=1;
		    }
		    else
		    {
		        auto_out_data[0]=0;
		    }
		    ret = copy_to_user(user_data_addr, auto_out_data, 4);
		    if(ret<0)
		    {
		        printk("AT_CHG_CHGR_OFF,copy_to_user return val error\n");
		    }
		    
		    printk("AT_CHG_CHGR_OFF\N");
		    for(i=0;i<5;i++)
		    {
		         printk("auto_out_data[%d]=%d\n",i,auto_out_data[i]);
		    }
	        break;
	    case K_AT_CHG_ON:
	        at_test_chg_on=1;
	        at_enter_chging_state = 1;
	        oppo_charge_temp_status = NORMAL_CHARGE_TEMP_STATUS;

	        #ifdef OPPO_USE_EXCHARGER
            if(FAN5405_ID == oppo_excharger_id)
            {
                mt_set_gpio_mode(GPIO_FAN5405_DISABLE_NUM,GPIO_FAN5405_DISABLE_MODE);
                mt_set_gpio_dir(GPIO_FAN5405_DISABLE_NUM,GPIO_FAN5405_DISABLE_DIR);
                mt_set_gpio_out(GPIO_FAN5405_DISABLE_NUM,GPIO_FAN5405_DISABLE_OFF_OUT);
            }
            else if(BQ24156A_ID == oppo_excharger_id)
            {
                mt_set_gpio_mode(GPIO_BQ24156A_DISABLE_NUM,GPIO_BQ24156A_DISABLE_MODE);  
                mt_set_gpio_dir(GPIO_BQ24156A_DISABLE_NUM,GPIO_BQ24156A_DISABLE_DIR);
                mt_set_gpio_out(GPIO_BQ24156A_DISABLE_NUM,GPIO_BQ24156A_DISABLE_OFF_OUT);
            }
            else//BQ24157
            {
                mt_set_gpio_mode(GPIO_BQ24156A_DISABLE_NUM,GPIO_BQ24156A_DISABLE_MODE);  
                mt_set_gpio_dir(GPIO_BQ24156A_DISABLE_NUM,GPIO_BQ24156A_DISABLE_DIR);
                mt_set_gpio_out(GPIO_BQ24156A_DISABLE_NUM,GPIO_BQ24156A_DISABLE_OFF_OUT);
            }
	        #else
	        pchr_turn_off_charging();
	        #endif
	        msleep(5);

	        #define ADC_COUNT 5
	        
	        at_bat_sense = get_bat_sense_volt(ADC_COUNT) * ADC_COUNT;
            at_i_sense = get_i_sense_volt(ADC_COUNT) * ADC_COUNT;
            at_current_offset = at_bat_sense - at_i_sense;
            
	        pchr_turn_on_charging();
	        printk("AT_CHG_ON\N");
	        break;
	    case K_AT_CHG_OFF:
	        at_test_chg_on=0;
	        at_enter_chging_state = 0;
            #ifdef OPPO_USE_EXCHARGER
            if(FAN5405_ID == oppo_excharger_id)
            {
                mt_set_gpio_mode(GPIO_FAN5405_DISABLE_NUM,GPIO_FAN5405_DISABLE_MODE);
                mt_set_gpio_dir(GPIO_FAN5405_DISABLE_NUM,GPIO_FAN5405_DISABLE_DIR);
                mt_set_gpio_out(GPIO_FAN5405_DISABLE_NUM,GPIO_FAN5405_DISABLE_OFF_OUT);
            }
            else if(BQ24156A_ID == oppo_excharger_id)
            {
                mt_set_gpio_mode(GPIO_BQ24156A_DISABLE_NUM,GPIO_BQ24156A_DISABLE_MODE);  
                mt_set_gpio_dir(GPIO_BQ24156A_DISABLE_NUM,GPIO_BQ24156A_DISABLE_DIR);
                mt_set_gpio_out(GPIO_BQ24156A_DISABLE_NUM,GPIO_BQ24156A_DISABLE_OFF_OUT);
            }
            else//BQ24157
            {
                mt_set_gpio_mode(GPIO_BQ24156A_DISABLE_NUM,GPIO_BQ24156A_DISABLE_MODE);  
                mt_set_gpio_dir(GPIO_BQ24156A_DISABLE_NUM,GPIO_BQ24156A_DISABLE_DIR);
                mt_set_gpio_out(GPIO_BQ24156A_DISABLE_NUM,GPIO_BQ24156A_DISABLE_OFF_OUT);
            }
	        #else
	        pchr_turn_off_charging();
	        #endif
	        printk("AT_CHG_OFF\N");
	        break;
	    case K_AT_CHG_INFO:
	        user_data_addr = (int *)arg;	
            
			auto_out_data[0]=oppo_auto_test_reset_g_notify_flag();
			auto_out_data[1]=g_BatteryAverageCurrent;
			auto_out_data[2]=BMT_status.temperature;
			auto_out_data[3]=BMT_status.bat_vol;
			auto_out_data[4]=BMT_status.charger_vol;
			ret = copy_to_user(user_data_addr, auto_out_data, 20);
            printk("K_AT_CHG_INFO\N");
		    for(i=0;i<5;i++)
		    {
		         printk("auto_out_data[%d]=%d\n",i,auto_out_data[i]);
		    }
            break; 
        #endif
          
        default:
            g_ADC_Cali = KAL_FALSE;
            break;
    }

    mutex_unlock(&bat_mutex);
    
    return 0;
}

static int adc_cali_open(struct inode *inode, struct file *file)
{ 
   return 0;
}

static int adc_cali_release(struct inode *inode, struct file *file)
{
    return 0;
}

static struct file_operations adc_cali_fops = {
    .owner        = THIS_MODULE,
    .unlocked_ioctl    = adc_cali_ioctl,
    .open        = adc_cali_open,
    .release    = adc_cali_release,    
};

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Charger_Voltage
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Charger_Voltage(struct device *dev,struct device_attribute *attr, char *buf)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] show_ADC_Charger_Voltage : %d\n", BMT_status.charger_vol);
    return sprintf(buf, "%d\n", BMT_status.charger_vol);
}
static ssize_t store_ADC_Charger_Voltage(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Charger_Voltage, 0664, show_ADC_Charger_Voltage, store_ADC_Charger_Voltage);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_0_Slope
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_0_Slope(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_slop+0));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_0_Slope : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_0_Slope(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_0_Slope, 0664, show_ADC_Channel_0_Slope, store_ADC_Channel_0_Slope);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_1_Slope
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_1_Slope(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_slop+1));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_1_Slope : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_1_Slope(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_1_Slope, 0664, show_ADC_Channel_1_Slope, store_ADC_Channel_1_Slope);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_2_Slope
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_2_Slope(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_slop+2));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_2_Slope : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_2_Slope(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_2_Slope, 0664, show_ADC_Channel_2_Slope, store_ADC_Channel_2_Slope);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_3_Slope
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_3_Slope(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_slop+3));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_3_Slope : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_3_Slope(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_3_Slope, 0664, show_ADC_Channel_3_Slope, store_ADC_Channel_3_Slope);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_4_Slope
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_4_Slope(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_slop+4));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_4_Slope : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_4_Slope(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_4_Slope, 0664, show_ADC_Channel_4_Slope, store_ADC_Channel_4_Slope);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_5_Slope
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_5_Slope(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_slop+5));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_5_Slope : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_5_Slope(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_5_Slope, 0664, show_ADC_Channel_5_Slope, store_ADC_Channel_5_Slope);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_6_Slope
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_6_Slope(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_slop+6));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_6_Slope : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_6_Slope(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_6_Slope, 0664, show_ADC_Channel_6_Slope, store_ADC_Channel_6_Slope);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_7_Slope
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_7_Slope(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_slop+7));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_7_Slope : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_7_Slope(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_7_Slope, 0664, show_ADC_Channel_7_Slope, store_ADC_Channel_7_Slope);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_8_Slope
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_8_Slope(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_slop+8));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_8_Slope : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_8_Slope(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_8_Slope, 0664, show_ADC_Channel_8_Slope, store_ADC_Channel_8_Slope);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_9_Slope
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_9_Slope(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_slop+9));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_9_Slope : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_9_Slope(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_9_Slope, 0664, show_ADC_Channel_9_Slope, store_ADC_Channel_9_Slope);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_10_Slope
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_10_Slope(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_slop+10));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_10_Slope : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_10_Slope(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_10_Slope, 0664, show_ADC_Channel_10_Slope, store_ADC_Channel_10_Slope);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_11_Slope
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_11_Slope(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_slop+11));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_11_Slope : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_11_Slope(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_11_Slope, 0664, show_ADC_Channel_11_Slope, store_ADC_Channel_11_Slope);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_12_Slope
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_12_Slope(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_slop+12));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_12_Slope : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_12_Slope(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_12_Slope, 0664, show_ADC_Channel_12_Slope, store_ADC_Channel_12_Slope);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_13_Slope
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_13_Slope(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_slop+13));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_13_Slope : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_13_Slope(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_13_Slope, 0664, show_ADC_Channel_13_Slope, store_ADC_Channel_13_Slope);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_0_Offset
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_0_Offset(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_offset+0));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_0_Offset : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_0_Offset(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_0_Offset, 0664, show_ADC_Channel_0_Offset, store_ADC_Channel_0_Offset);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_1_Offset
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_1_Offset(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_offset+1));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_1_Offset : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_1_Offset(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_1_Offset, 0664, show_ADC_Channel_1_Offset, store_ADC_Channel_1_Offset);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_2_Offset
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_2_Offset(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_offset+2));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_2_Offset : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_2_Offset(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_2_Offset, 0664, show_ADC_Channel_2_Offset, store_ADC_Channel_2_Offset);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_3_Offset
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_3_Offset(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_offset+3));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_3_Offset : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_3_Offset(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_3_Offset, 0664, show_ADC_Channel_3_Offset, store_ADC_Channel_3_Offset);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_4_Offset
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_4_Offset(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_offset+4));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_4_Offset : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_4_Offset(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_4_Offset, 0664, show_ADC_Channel_4_Offset, store_ADC_Channel_4_Offset);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_5_Offset
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_5_Offset(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_offset+5));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_5_Offset : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_5_Offset(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_5_Offset, 0664, show_ADC_Channel_5_Offset, store_ADC_Channel_5_Offset);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_6_Offset
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_6_Offset(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_offset+6));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_6_Offset : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_6_Offset(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_6_Offset, 0664, show_ADC_Channel_6_Offset, store_ADC_Channel_6_Offset);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_7_Offset
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_7_Offset(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_offset+7));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_7_Offset : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_7_Offset(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_7_Offset, 0664, show_ADC_Channel_7_Offset, store_ADC_Channel_7_Offset);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_8_Offset
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_8_Offset(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_offset+8));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_8_Offset : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_8_Offset(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_8_Offset, 0664, show_ADC_Channel_8_Offset, store_ADC_Channel_8_Offset);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_9_Offset
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_9_Offset(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_offset+9));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_9_Offset : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_9_Offset(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_9_Offset, 0664, show_ADC_Channel_9_Offset, store_ADC_Channel_9_Offset);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_10_Offset
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_10_Offset(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_offset+10));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_10_Offset : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_10_Offset(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_10_Offset, 0664, show_ADC_Channel_10_Offset, store_ADC_Channel_10_Offset);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_11_Offset
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_11_Offset(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_offset+11));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_11_Offset : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_11_Offset(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_11_Offset, 0664, show_ADC_Channel_11_Offset, store_ADC_Channel_11_Offset);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_12_Offset
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_12_Offset(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_offset+12));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_12_Offset : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_12_Offset(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_12_Offset, 0664, show_ADC_Channel_12_Offset, store_ADC_Channel_12_Offset);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_13_Offset
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_13_Offset(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = (*(adc_cali_offset+13));
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_13_Offset : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_13_Offset(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_13_Offset, 0664, show_ADC_Channel_13_Offset, store_ADC_Channel_13_Offset);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : ADC_Channel_Is_Calibration
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_ADC_Channel_Is_Calibration(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=2;
    ret_value = g_ADC_Cali;
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] ADC_Channel_Is_Calibration : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_ADC_Channel_Is_Calibration(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(ADC_Channel_Is_Calibration, 0664, show_ADC_Channel_Is_Calibration, store_ADC_Channel_Is_Calibration);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : Power_On_Voltage
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_Power_On_Voltage(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = Batt_VoltToPercent_Table[0].BattVolt;
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Power_On_Voltage : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_Power_On_Voltage(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(Power_On_Voltage, 0664, show_Power_On_Voltage, store_Power_On_Voltage);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : Power_Off_Voltage
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_Power_Off_Voltage(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = Batt_VoltToPercent_Table[0].BattVolt;
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Power_Off_Voltage : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_Power_Off_Voltage(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(Power_Off_Voltage, 0664, show_Power_Off_Voltage, store_Power_Off_Voltage);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : Charger_TopOff_Value
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_Charger_TopOff_Value(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=1;
    ret_value = Batt_VoltToPercent_Table[10].BattVolt;
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Charger_TopOff_Value : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_Charger_TopOff_Value(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(Charger_TopOff_Value, 0664, show_Charger_TopOff_Value, store_Charger_TopOff_Value);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : FG_Battery_CurrentConsumption
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_FG_Battery_CurrentConsumption(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value=8888;
    ret_value = gFG_current;    
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] FG_Battery_CurrentConsumption : %d/10 mA\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_FG_Battery_CurrentConsumption(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(FG_Battery_CurrentConsumption, 0664, show_FG_Battery_CurrentConsumption, store_FG_Battery_CurrentConsumption);

///////////////////////////////////////////////////////////////////////////////////////////
//// Create File For EM : FG_SW_CoulombCounter
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_FG_SW_CoulombCounter(struct device *dev,struct device_attribute *attr, char *buf)
{
    kal_int32 ret_value=7777;
    ret_value = gFG_columb;
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] FG_SW_CoulombCounter : %d\n", ret_value);
    return sprintf(buf, "%u\n", ret_value);
}
static ssize_t store_FG_SW_CoulombCounter(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[EM] Not Support Write Function\n");    
    return size;
}
static DEVICE_ATTR(FG_SW_CoulombCounter, 0664, show_FG_SW_CoulombCounter, store_FG_SW_CoulombCounter);

///////////////////////////////////////////////////////////////////////////////////////////
//// oppo zhanglong add: Create File For average current
///////////////////////////////////////////////////////////////////////////////////////////
#define SECONDS_PER_HOUR (60 * 60)

static kal_int32    coulomb;
static unsigned long second_rec;
static int          bStarted;       //if the start time and coulomb is recorded
static ssize_t show_FG_averageCurrByCoulomb(struct device *dev, struct device_attribute *attr, char *buf)
{
    kal_int32     coulomb_end;
    int           seconds;
	int 		  *iBuf = (int *)buf;

    if(!bStarted)
    {
        printk("[AVERAGE CURRENT] %s: not started, can't calculate\n", __func__);
		*iBuf = -1;
        return -2;
    }
    bStarted = 0;

    
    seconds = get_seconds() - second_rec;
    coulomb_end = gFG_columb;

    printk("[AVERAGE CURRENT] columb0:%d, columb1:%d, seconds:%d\n", coulomb, coulomb_end, seconds);
    if(coulomb_end < 3 && coulomb_end > -3)
    {
        printk("gFG_columb is probably reseted!!\n");               
        *iBuf = -1;
        return -1;
    }
    *iBuf = (coulomb-coulomb_end)*SECONDS_PER_HOUR/seconds;
	return sizeof(int);
}

static ssize_t store_FG_averageCurrByCoulomb(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    if(bStarted)
    {
        printk("[AVERAGE CURRENT] %s: already started, ignore this\n", __func__);
        return size;
    }

    if(buf[0]=='b')
    {
        bStarted = 1;

        coulomb = gFG_columb;
        second_rec = get_seconds();    
    }
    return size;
}
static DEVICE_ATTR(FG_averageCurrByCoulomb, 0666, show_FG_averageCurrByCoulomb, store_FG_averageCurrByCoulomb);

///////////////////////////////////////////////////////////////////////////////////////////
//// platform_driver API 
///////////////////////////////////////////////////////////////////////////////////////////
#define BAT_MS_TO_NS(x) (x * 1000 * 1000)
static struct hrtimer charger_hv_detect_timer;
static struct task_struct *charger_hv_detect_thread = NULL;
static int charger_hv_detect_flag = 0;
static DECLARE_WAIT_QUEUE_HEAD(charger_hv_detect_waiter);

int charger_hv_detect_sw_thread_handler(void *unused)
{
    ktime_t ktime;

    do
    {
        ktime = ktime_set(0, BAT_MS_TO_NS(500));       
    
        //xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[charger_hv_detect_sw_thread_handler] \n");

        charger_hv_init();
            
        wait_event_interruptible(charger_hv_detect_waiter, charger_hv_detect_flag != 0);
    
        charger_hv_detect_flag = 0;
        
        if( get_charger_hv_status() == 1)
        {
            xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[charger_hv_detect_sw_thread_handler] charger hv\n");    
            
            pchr_turn_off_charging();
        }
        else
        {
            //xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[charger_hv_detect_sw_thread_handler] upmu_chr_get_vcdt_hv_det() != 1\n");    
        }

        kick_charger_wdt(); 
       
        hrtimer_start(&charger_hv_detect_timer, ktime, HRTIMER_MODE_REL);    
        
    } while (!kthread_should_stop());
    
    return 0;
}

enum hrtimer_restart charger_hv_detect_sw_workaround(struct hrtimer *timer)
{
    charger_hv_detect_flag = 1; 
    wake_up_interruptible(&charger_hv_detect_waiter);

    //xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[charger_hv_detect_sw_workaround] \n");
    
    return HRTIMER_NORESTART;
}

void charger_hv_detect_sw_workaround_init(void)
{
    ktime_t ktime;

    ktime = ktime_set(0, BAT_MS_TO_NS(500));
    hrtimer_init(&charger_hv_detect_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    charger_hv_detect_timer.function = charger_hv_detect_sw_workaround;    
    hrtimer_start(&charger_hv_detect_timer, ktime, HRTIMER_MODE_REL);

    charger_hv_detect_thread = kthread_run(charger_hv_detect_sw_thread_handler, 0, "mtk charger_hv_detect_sw_workaround");
    if (IS_ERR(charger_hv_detect_thread))
    {
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[%s]: failed to create charger_hv_detect_sw_workaround thread\n", __FUNCTION__);
    }

    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "charger_hv_detect_sw_workaround_init : done\n" );
}

///////////////////////////////////////////////////////////////////////////////////////////
//// platform_driver API 
///////////////////////////////////////////////////////////////////////////////////////////
static struct hrtimer battery_kthread_timer;
static struct task_struct *battery_kthread_hrtimer_task = NULL;
static int battery_kthread_flag = 0;
static DECLARE_WAIT_QUEUE_HEAD(battery_kthread_waiter);
#ifdef VENDOR_EDIT//Jiada.LI@ProDrv.chager, modify 2013/05/18 for ioctrol
static u8 g_bat_thread_count = 0;
#endif /*VENDOR_EDIT*/
int battery_kthread_handler(void *unused)
{
    ktime_t ktime;

    do
    {
		#ifdef VENDOR_EDIT//Jiada.LI@ProDrv.chager, modify 2013/05/18 for ioctrol
		if(g_bat_thread_count < 3) {
			xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "g_bat_thread_count : done\n", g_bat_thread_count);
			g_bat_thread_count += 1;
			ktime = ktime_set(5, 0);	// 5s, 5* 1000 ms
		}else {
			ktime = ktime_set(10, 0);	// 10s, 10* 1000 ms
		}
    	#endif /*VENDOR_EDIT*/
    
        wait_event_interruptible(battery_kthread_waiter, battery_kthread_flag != 0);
    
        battery_kthread_flag = 0;
        bat_thread_wakeup();
        hrtimer_start(&battery_kthread_timer, ktime, HRTIMER_MODE_REL);    
        
    } while (!kthread_should_stop());
    
    return 0;
}

enum hrtimer_restart battery_kthread_hrtimer_func(struct hrtimer *timer)
{
    battery_kthread_flag = 1; 
    wake_up_interruptible(&battery_kthread_waiter);

    return HRTIMER_NORESTART;
}

void battery_kthread_hrtimer_init(void)
{
    ktime_t ktime;

	ktime = ktime_set(5, 0);	// 5s, 5* 1000 ms
	
    hrtimer_init(&battery_kthread_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    battery_kthread_timer.function = battery_kthread_hrtimer_func;    
    hrtimer_start(&battery_kthread_timer, ktime, HRTIMER_MODE_REL);

    battery_kthread_hrtimer_task = kthread_run(battery_kthread_handler, NULL, "mtk battery_kthread_handler"); 
    if (IS_ERR(battery_kthread_hrtimer_task))
    {
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[%s]: failed to create battery_kthread_hrtimer_task thread\n", __FUNCTION__);
    }

    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "battery_kthread_hrtimer_init : done\n" );
}

static int mt6320_battery_probe(struct platform_device *dev)    
{
    struct class_device *class_dev = NULL;
    int ret=0;
    int i=0;
    int ret_device_file=0;

    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "******** MT6320 battery driver probe!! ********\n" );

    /* Integrate with NVRAM */
    ret = alloc_chrdev_region(&adc_cali_devno, 0, 1, ADC_CALI_DEVNAME);
    if (ret) 
       xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "Error: Can't Get Major number for adc_cali \n");
    adc_cali_cdev = cdev_alloc();
    adc_cali_cdev->owner = THIS_MODULE;
    adc_cali_cdev->ops = &adc_cali_fops;
    ret = cdev_add(adc_cali_cdev, adc_cali_devno, 1);
    if(ret)
       xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "adc_cali Error: cdev_add\n");
    adc_cali_major = MAJOR(adc_cali_devno);
    adc_cali_class = class_create(THIS_MODULE, ADC_CALI_DEVNAME);
    class_dev = (struct class_device *)device_create(adc_cali_class, 
                                                   NULL, 
                                                   adc_cali_devno, 
                                                   NULL, 
                                                   ADC_CALI_DEVNAME);
    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[MT6320 BAT_probe] adc_cali prepare : done !!\n ");

    /* Integrate with Android Battery Service */
    ret = power_supply_register(&(dev->dev), &mt6320_ac_main.psy);
    if (ret)
    {            
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[MT6320 BAT_probe] power_supply_register AC Fail !!\n");                    
        return ret;
    }             
    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[MT6320 BAT_probe] power_supply_register AC Success !!\n");

    ret = power_supply_register(&(dev->dev), &mt6320_usb_main.psy);
    if (ret)
    {            
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[MT6320 BAT_probe] power_supply_register USB Fail !!\n");                    
        return ret;
    }             
    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[MT6320 BAT_probe] power_supply_register USB Success !!\n");

    ret = power_supply_register(&(dev->dev), &mt6320_battery_main.psy);
    if (ret)
    {
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[MT6320 BAT_probe] power_supply_register Battery Fail !!\n");
        return ret;
    }
    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[MT6320 BAT_probe] power_supply_register Battery Success !!\n");

    /* For EM */
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Charger_Voltage);
    
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_0_Slope);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_1_Slope);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_2_Slope);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_3_Slope);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_4_Slope);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_5_Slope);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_6_Slope);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_7_Slope);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_8_Slope);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_9_Slope);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_10_Slope);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_11_Slope);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_12_Slope);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_13_Slope);

    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_0_Offset);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_1_Offset);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_2_Offset);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_3_Offset);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_4_Offset);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_5_Offset);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_6_Offset);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_7_Offset);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_8_Offset);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_9_Offset);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_10_Offset);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_11_Offset);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_12_Offset);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_13_Offset);

    ret_device_file = device_create_file(&(dev->dev), &dev_attr_ADC_Channel_Is_Calibration);

    ret_device_file = device_create_file(&(dev->dev), &dev_attr_Power_On_Voltage);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_Power_Off_Voltage);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_Charger_TopOff_Value);
    
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_FG_Battery_CurrentConsumption);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_FG_SW_CoulombCounter);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_FG_averageCurrByCoulomb);

    /* Initialization BMT Struct */
    for (i=0; i<BATTERY_AVERAGE_SIZE; i++) {
        batteryCurrentBuffer[i] = 0;
        batteryVoltageBuffer[i] = 0; 
        batterySOCBuffer[i] = 0;
        batteryTempBuffer[i] = 0;
    }
    batteryVoltageSum = 0;
    batteryCurrentSum = 0;
    batterySOCSum = 0;
    batteryTempSum = 0;

    BMT_status.bat_exist = 1;       /* phone must have battery */
    BMT_status.charger_exist = 0;     /* for default, no charger */
    BMT_status.bat_vol = 0;
    BMT_status.ICharging = 0;
    BMT_status.temperature = 0;
    BMT_status.charger_vol = 0;
    BMT_status.total_charging_time = 0;
    BMT_status.PRE_charging_time = 0;
    BMT_status.CC_charging_time = 0;
    BMT_status.TOPOFF_charging_time = 0;
    BMT_status.POSTFULL_charging_time = 0;

    BMT_status.bat_charging_state = CHR_PRE;

    #ifdef OPPO_NEW_CHARGING
    oppo_ipod_status = 0;
    #endif

    #ifndef OPPO_NO_USE_FGADC
	if(g_FG_init == 0)
	{
		g_FG_init=1;
		fgauge_initialization();
	}
	#endif
    //baton initial setting
    //ret=pmic_config_interface(CHR_CON7, 0x01, PMIC_BATON_TDET_EN_MASK, PMIC_BATON_TDET_EN_SHIFT); //BATON_TDET_EN=1
    //ret=pmic_config_interface(AUXADC_CON0, 0x01, PMIC_RG_BUF_PWD_B_MASK, PMIC_RG_BUF_PWD_B_SHIFT); //RG_BUF_PWD_B=1
    //xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[mt6320_battery_probe] BATON_TDET_EN=1 & RG_BUF_PWD_B=1\n");

    //battery kernel thread for 10s check and charger in/out event
    /* Replace GPT timer by hrtime */
    battery_kthread_hrtimer_init();
	
    kthread_run(bat_thread_kthread, NULL, "bat_thread_kthread"); 
    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "[mt6320_battery_probe] bat_thread_kthread Done\n");    
    
    charger_hv_detect_sw_workaround_init();

    /*LOG System Set*/
    init_proc_log();

    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "DCT-PMIC-ADC:AUXADC_BATTERY_VOLTAGE_CHANNEL=%d\r\n",AUXADC_BATTERY_VOLTAGE_CHANNEL);
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "DCT-PMIC-ADC:AUXADC_REF_CURRENT_CHANNEL=%d\r\n",AUXADC_REF_CURRENT_CHANNEL);
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "DCT-PMIC-ADC:AUXADC_CHARGER_VOLTAGE_CHANNEL=%d\r\n",AUXADC_CHARGER_VOLTAGE_CHANNEL);
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "DCT-PMIC-ADC:AUXADC_TEMPERATURE_CHANNEL=%d\r\n",AUXADC_TEMPERATURE_CHANNEL);

    g_bat_init_flag=1;

    return 0;
}
#endif

static int mt6320_battery_remove(struct platform_device *dev)    
{
    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "******** MT6320 battery driver remove!! ********\n" );

    return 0;
}

static void mt6320_battery_shutdown(struct platform_device *dev)    
{
    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "******** MT6320 battery driver shutdown!! ********\n" );

}

static int mt6320_battery_suspend(struct platform_device *dev, pm_message_t state)    
{
    //xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "******** MT6320 battery driver suspend!! ********\n" );

    return 0;
}

int force_get_tbat(void)
{
#if defined(CONFIG_POWER_EXT)
    return 19;
#else
    int bat_temperature_volt=0;
    int bat_temperature_val=0;
    int fg_r_value=0;
    kal_int32 fg_current_temp=0;
    kal_bool fg_current_state=KAL_FALSE;
    int bat_temperature_volt_temp=0;
    
    /* Get V_BAT_Temperature */
    bat_temperature_volt = get_tbat_volt(2); 
    if(bat_temperature_volt != 0)
    {   
        if( gForceADCsolution == 1 )
        {
            /*Use no gas gauge*/
        }
        else
        {
            fg_r_value = get_r_fg_value();
            fg_current_temp = fgauge_read_current();
            fg_current_temp = fg_current_temp/10;
            fg_current_state = get_gFG_Is_Charging();
            if(fg_current_state==KAL_TRUE)
            {
                bat_temperature_volt_temp = bat_temperature_volt;
                bat_temperature_volt = bat_temperature_volt - ((fg_current_temp*fg_r_value)/1000);
            }
            else
            {
                bat_temperature_volt_temp = bat_temperature_volt;
                bat_temperature_volt = bat_temperature_volt + ((fg_current_temp*fg_r_value)/1000);
            }
        }
        
        bat_temperature_val = BattVoltToTemp(bat_temperature_volt);        
    }
    
    printk(KERN_CRIT "[tbat] %d,%d,%d,%d,%d,%d\n", 
        bat_temperature_volt_temp, bat_temperature_volt, fg_current_state, fg_current_temp, fg_r_value, bat_temperature_val);
    
    return bat_temperature_val;    
#endif    
}
EXPORT_SYMBOL(force_get_tbat);

static int mt6320_battery_resume(struct platform_device *dev)
{
    //xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "******** MT6320 battery driver resume!! ********\n" );

#if defined(CONFIG_POWER_EXT)
#else        
        #ifdef VENDOR_EDIT //oppo zhanglong add for average current
        mutex_lock(&bat_mutex);
        FGADC_thread_kthread();
        mutex_unlock(&bat_mutex);
        #endif
        
        if(slp_get_wake_reason() == WR_PCM_TIMER)
        {
            printk(KERN_CRIT "[bat resume] by pcm timer\n");
            
            mutex_lock(&bat_mutex);
            #ifndef VENDOR_EDIT  //oppo zhanglong move for average current
            FGADC_thread_kthread();
            #endif
            BAT_thread();
            mutex_unlock(&bat_mutex);
        }    
#endif

    return 0;
}

struct platform_device MT6320_battery_device = {
    .name   = "mt6320-battery",
    .id        = -1,
};

static struct platform_driver mt6320_battery_driver = {
    .probe         = mt6320_battery_probe,
    .remove        = mt6320_battery_remove,
    .shutdown      = mt6320_battery_shutdown,
    //#ifdef CONFIG_PM
    .suspend       = mt6320_battery_suspend,
    .resume        = mt6320_battery_resume,
    //#endif
    .driver        = {
        .name = "mt6320-battery",
    },
};

///////////////////////////////////////////////////////////////////////////////////////////
//// Battery Notify API 
///////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_BatteryNotify(struct device *dev,struct device_attribute *attr, char *buf)
{
    if (Enable_BATDRV_LOG == 1) {
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[Battery] show_BatteryNotify : %x\n", g_BatteryNotifyCode);
    }
    return sprintf(buf, "%u\n", g_BatteryNotifyCode);
}
static ssize_t store_BatteryNotify(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    char *pvalue = NULL;
    unsigned int reg_BatteryNotifyCode = 0;
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[Battery] store_BatteryNotify\n");
    if(buf != NULL && size != 0)
    {
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[Battery] buf is %s and size is %d \n",buf,size);
        reg_BatteryNotifyCode = simple_strtoul(buf,&pvalue,16);
        g_BatteryNotifyCode = reg_BatteryNotifyCode;
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[Battery] store code : %x \n",g_BatteryNotifyCode);        
    }        
    return size;
}
static DEVICE_ATTR(BatteryNotify, 0664, show_BatteryNotify, store_BatteryNotify);

static ssize_t show_BN_TestMode(struct device *dev,struct device_attribute *attr, char *buf)
{
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[Battery] show_BN_TestMode : %x\n", g_BN_TestMode);
    return sprintf(buf, "%u\n", g_BN_TestMode);
}
static ssize_t store_BN_TestMode(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    char *pvalue = NULL;
    unsigned int reg_BN_TestMode = 0;
    xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[Battery] store_BN_TestMode\n");
    if(buf != NULL && size != 0)
    {
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[Battery] buf is %s and size is %d \n",buf,size);
        reg_BN_TestMode = simple_strtoul(buf,&pvalue,16);
        g_BN_TestMode = reg_BN_TestMode;
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[Battery] store g_BN_TestMode : %x \n",g_BN_TestMode);        
    }        
    return size;
}
static DEVICE_ATTR(BN_TestMode, 0664, show_BN_TestMode, store_BN_TestMode);


#ifdef VENDOR_EDIT//Fanhong.Kong@ProDrv.CHG, add 2012/7/6 for low bat power on
static ssize_t show_Powerkey_Close(struct device *dev,struct device_attribute *attr, char *buf)
{
	if (Enable_BATDRV_LOG == 1) 
    {
	xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[Battery] show_Powerkey_Close : %x\n", g_BAT_Low_Close);
	}
    
    printk("[Battery] show_Powerkey_Close : %x\n", g_BAT_Low_Close);
	return sprintf(buf, "%u\n", g_BAT_Low_Close);
}
static ssize_t store_Powerkey_Close(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
	char *pvalue = NULL;
	int reg_BAT_Low_Close = 0;
    
	xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[Battery] store_Powerkey_Close\n");
	if(buf != NULL && size != 0)
	{
		xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[Battery] buf is %s and size is %d \n",buf,size);
		reg_BAT_Low_Close = simple_strtoul(buf,&pvalue,16);
		g_BAT_Low_Close = reg_BAT_Low_Close;
		xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[Battery] store code : %x \n",g_BAT_Low_Close);		
	}		
	return size;
}
static DEVICE_ATTR(BAT_Low_Close_Status, 0664, show_Powerkey_Close, store_Powerkey_Close);

static ssize_t show_IPO_status(struct device *dev,struct device_attribute *attr, char *buf)
{
	if (Enable_BATDRV_LOG == 1) 
    {
	xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[Battery] show_IPO_status : %x\n", g_IPO_status);
	}
    
    printk("[Battery] show_IPO_status : %x\n", g_IPO_status);
	return sprintf(buf, "%u\n", g_IPO_status);
}
static ssize_t store_IPO_status(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
	char *pvalue = NULL;
	int reg_IPO_status = 0;
	xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[Battery] store_IPO_status\n");
	if(buf != NULL && size != 0)
	{
		xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[Battery] buf is %s and size is %d \n",buf,size);
		reg_IPO_status = simple_strtoul(buf,&pvalue,16);
		g_IPO_status = reg_IPO_status;
		xlog_printk(ANDROID_LOG_DEBUG, "Power/Battery", "[Battery] store code : %x \n",g_IPO_status);		
	}		
	return size;
}
static DEVICE_ATTR(IPO_Status, 0664, show_IPO_status, store_IPO_status);

static ssize_t show_ID_status(struct device *dev,struct device_attribute *attr, char *buf)
{
	if (Enable_BATDRV_LOG == 1) 
    {
	    printk("dengnanwei_[Battery] oppo_high_battery_status= : %x\n", oppo_high_battery_status);
	}
    
    printk("dengnanwei_[Battery] oppo_high_battery_status= : %x\n", oppo_high_battery_status);
	return sprintf(buf, "%u\n", oppo_high_battery_status);
}
static ssize_t store_ID_status(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
	char *pvalue = NULL;
	int reg_ID_status = 0;
    
	printk("dengnanwei_[Battery] store_ID_status\n");
	if(buf != NULL && size != 0)
	{
		printk("dengnanwei_[Battery] buf is %s and size is %d \n",buf,size);
		reg_ID_status = simple_strtoul(buf,&pvalue,16);
		oppo_high_battery_status = reg_ID_status;
		printk("dengnanwei_[Battery] oppo_high_battery_status= : %x \n",oppo_high_battery_status);		
	}		
	return size;
}
static DEVICE_ATTR(ID_status, 0664, show_ID_status, store_ID_status);

#endif

///////////////////////////////////////////////////////////////////////////////////////////
//// platform_driver API 
///////////////////////////////////////////////////////////////////////////////////////////
static int battery_cmd_read(char *buf, char **start, off_t off, int count, int *eof, void *data)
{
    int len = 0;
    char *p = buf;
    
    p += sprintf(p, "g_battery_thermal_throttling_flag=%d,\nbattery_cmd_thermal_test_mode=%d,\nbattery_cmd_thermal_test_mode_value=%d\n", 
        g_battery_thermal_throttling_flag, battery_cmd_thermal_test_mode, battery_cmd_thermal_test_mode_value);
    
    *start = buf + off;
    
    len = p - buf;
    if (len > off)
        len -= off;
    else
        len = 0;
    
    return len < count ? len  : count;
}

static ssize_t battery_cmd_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
    int len = 0, bat_tt_enable=0, bat_thr_test_mode=0, bat_thr_test_value=0;
    char desc[32];
    
    len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
    if (copy_from_user(desc, buffer, len))
    {
        return 0;
    }
    desc[len] = '\0';
    
    if (sscanf(desc, "%d %d %d", &bat_tt_enable, &bat_thr_test_mode, &bat_thr_test_value) == 3)
    {
        g_battery_thermal_throttling_flag = bat_tt_enable;
        battery_cmd_thermal_test_mode = bat_thr_test_mode;
        battery_cmd_thermal_test_mode_value = bat_thr_test_value;
        
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Thermal", "bat_tt_enable=%d, bat_thr_test_mode=%d, bat_thr_test_value=%d\n", 
            g_battery_thermal_throttling_flag, battery_cmd_thermal_test_mode, battery_cmd_thermal_test_mode_value);
        
        return count;
    }
    else
    {
        xlog_printk(ANDROID_LOG_DEBUG, "Power/Thermal", "  bad argument, echo [bat_tt_enable] [bat_thr_test_mode] [bat_thr_test_value] > battery_cmd\n");
    }
    
    return -EINVAL;
}

static int mt_batteryNotify_probe(struct platform_device *dev)    
{    
    int ret_device_file = 0;
    struct proc_dir_entry *entry = NULL;
    struct proc_dir_entry *battery_dir = NULL;

    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "******** mt_batteryNotify_probe!! ********\n" );
    
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_BatteryNotify);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_BN_TestMode);
	#ifdef VENDOR_EDIT //Fanhong.Kong@ProDrv.CHG, add 2012/7/6 for low bat power on
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_BAT_Low_Close_Status);
    ret_device_file = device_create_file(&(dev->dev), &dev_attr_IPO_Status);
	ret_device_file = device_create_file(&(dev->dev), &dev_attr_ID_status);
	#endif
    
    battery_dir = proc_mkdir("mtk_battery_cmd", NULL);
    if (!battery_dir)
    {
        pr_err("[%s]: mkdir /proc/mtk_battery_cmd failed\n", __FUNCTION__);
    }
    else
    {
        entry = create_proc_entry("battery_cmd", S_IRUGO | S_IWUSR, battery_dir);
        if (entry)
        {
            entry->read_proc = battery_cmd_read;
            entry->write_proc = battery_cmd_write;
        }
    }

    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "******** mtk_battery_cmd!! ********\n" );    
    
    return 0;
}

struct platform_device MT_batteryNotify_device = {
    .name   = "mt-battery",
    .id        = -1,
};

static struct platform_driver mt_batteryNotify_driver = {
    .probe        = mt_batteryNotify_probe,
    .driver     = {
        .name = "mt-battery",
    },
};

static int __init mt6320_battery_init(void)
{
    int ret;

    ret = platform_device_register(&MT6320_battery_device);
    if (ret) {
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "****[mt6320_battery_driver] Unable to device register(%d)\n", ret);
    return ret;
    }
    
    ret = platform_driver_register(&mt6320_battery_driver);
    if (ret) {
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "****[mt6320_battery_driver] Unable to register driver (%d)\n", ret);
    return ret;
    }

    // battery notofy UI
    ret = platform_device_register(&MT_batteryNotify_device);
    if (ret) {
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "****[mt_batteryNotify] Unable to device register(%d)\n", ret);
        return ret;
    }    
    ret = platform_driver_register(&mt_batteryNotify_driver);
    if (ret) {
        xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "****[mt_batteryNotify] Unable to register driver (%d)\n", ret);
        return ret;
    }

    xlog_printk(ANDROID_LOG_INFO, "Power/Battery", "****[mt6320_battery_driver] Initialization : DONE !!\n");

    return 0;
}

static void __exit mt6320_battery_exit (void)
{
}

module_init(mt6320_battery_init);
module_exit(mt6320_battery_exit);

MODULE_AUTHOR("James Lo");
MODULE_DESCRIPTION("MT6320 Battery Device Driver");
MODULE_LICENSE("GPL");

