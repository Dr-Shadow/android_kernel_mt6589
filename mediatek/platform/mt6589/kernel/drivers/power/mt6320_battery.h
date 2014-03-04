/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 *
 * MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

#ifndef MT6320_BATTERY_H
#define MT6320_BATTERY_H

#include <linux/ioctl.h>

/*****************************************************************************
 *  BATTERY VOLTAGE
 ****************************************************************************/
#define PRE_CHARGE_VOLTAGE                  3200
#define SYSTEM_OFF_VOLTAGE                  3400  
#define CONSTANT_CURRENT_CHARGE_VOLTAGE     4100  
#define CONSTANT_VOLTAGE_CHARGE_VOLTAGE     4200  
#define CV_DROPDOWN_VOLTAGE                 4000
#define CHARGER_THRESH_HOLD                 4300
#define BATTERY_UVLO_VOLTAGE                2700

/*****************************************************************************
 *  BATTERY TIMER
 ****************************************************************************/
//#define MAX_CHARGING_TIME                   8*60*60 	// 8hr
//#define MAX_CHARGING_TIME                   12*60*60 	// 12hr
#define MAX_CHARGING_TIME                   24*60*60 	// 24hr
#define MAX_POSTFULL_SAFETY_TIME       		1*30*60 	// 30mins
#define MAX_PreCC_CHARGING_TIME         	1*30*60  	// 0.5hr
#define MAX_CV_CHARGING_TIME              	3*60*60 	// 3hr
//#define MAX_CV_CHARGING_TIME              	24*60*60 	// 24hr
#define MUTEX_TIMEOUT                       5000
#define BAT_TASK_PERIOD                     10 			// 10sec
#define g_free_bat_temp 					1000 		// 1 s

/*****************************************************************************
 *  BATTERY Protection
 ****************************************************************************/
#define Battery_Percent_100    100
#define charger_OVER_VOL	    1
#define BATTERY_UNDER_VOL		2
#define BATTERY_OVER_TEMP		3
#define ADC_SAMPLE_TIMES        5

/*****************************************************************************
 *  Pulse Charging State
 ****************************************************************************/
#define  CHR_PRE                        0x1000
#define  CHR_CC                         0x1001 
#define  CHR_TOP_OFF                    0x1002 
#define  CHR_POST_FULL                  0x1003
#define  CHR_BATFULL                    0x1004 
#define  CHR_ERROR                      0x1005

/*****************************************************************************
 *  Type define
 ****************************************************************************/
typedef unsigned int       WORD;
typedef enum {
    CHARGER_UNKNOWN = 0,
    STANDARD_HOST,          // USB : 450mA
    CHARGING_HOST,
    NONSTANDARD_CHARGER,    // AC : 450mA~1A 
    STANDARD_CHARGER,       // AC : ~1A
} CHARGER_TYPE;

typedef enum
{
	USB_SUSPEND = 0,
	USB_UNCONFIGURED,
	USB_CONFIGURED
}usb_state_enum;

/*****************************************************************************
*   JEITA battery temperature standard 
    charging info ,like temperatue, charging current, re-charging voltage, CV threshold would be reconfigurated.
    Temperature hysteresis default 6C.  
    Reference table:
    degree    AC Current    USB current    CV threshold    Recharge Vol    hysteresis condition 
    > 60       no charging current,             X                    X                     <54(Down) 
    45~60     600mA         450mA             4.1V               4V                   <39(Down) >60(Up) 
    10~45     600mA         450mA             4.2V               4.1V                <10(Down) >45(Up) 
    0~10       600mA         450mA             4.1V               4V                   <0(Down)  >16(Up) 
    -10~0     200mA         200mA             4V                  3.9V                <-10(Down) >6(Up) 
    <-10      no charging current,              X                    X                    >-10(Up)  
****************************************************************************/
typedef enum
{
        TEMP_BELOW_NEG_10 = 0,
        TEMP_NEG_10_TO_POS_0,
        TEMP_POS_0_TO_POS_10,
        TEMP_POS_10_TO_POS_45,
        TEMP_POS_45_TO_POS_60,
        TEMP_ABOVE_POS_60
}temp_state_enum;
    
#define TEMP_POS_60_THRESHOLD  50
#define TEMP_POS_60_THRES_MINUS_X_DEGREE 47  

#define TEMP_POS_45_THRESHOLD  45
#define TEMP_POS_45_THRES_MINUS_X_DEGREE 39

#define TEMP_POS_10_THRESHOLD  10
#define TEMP_POS_10_THRES_PLUS_X_DEGREE 16

#define TEMP_POS_0_THRESHOLD  0
#define TEMP_POS_0_THRES_PLUS_X_DEGREE 6

#ifdef MTK_FAN5405_SUPPORT
#define TEMP_NEG_10_THRESHOLD  0
#define TEMP_NEG_10_THRES_PLUS_X_DEGREE  0
#elif defined( MTK_BQ24158_SUPPORT)
#define TEMP_NEG_10_THRESHOLD  0
#define TEMP_NEG_10_THRES_PLUS_X_DEGREE  0
#else
#define TEMP_NEG_10_THRESHOLD  0
#define TEMP_NEG_10_THRES_PLUS_X_DEGREE  0
#endif

/*****************************************************************************
 *  Extern Function
 ****************************************************************************/
#ifdef CONFIG_MTK_SMART_BATTERY

extern void wake_up_bat(void);
extern unsigned long BAT_Get_Battery_Voltage(int polling_mode);

#else

#define wake_up_bat()			do {} while (0)
#define BAT_Get_Battery_Voltage(polling_mode)	({ 0; })

#endif

#endif

