/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
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
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/xlog.h>

#include "kd_camera_hw.h"

#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_camera_feature.h"

/******************************************************************************
 * Debug configuration
******************************************************************************/
#define PFX "[kd_camera_hw]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    xlog_printk(ANDROID_LOG_INFO, PFX , fmt, ##arg)

#define DEBUG_CAMERA_HW_K
#ifdef DEBUG_CAMERA_HW_K
#define PK_DBG PK_DBG_FUNC
#define PK_ERR(fmt, arg...)         xlog_printk(ANDROID_LOG_ERR, PFX , fmt, ##arg)
#define PK_XLOG_INFO(fmt, args...) \
                do {    \
                   xlog_printk(ANDROID_LOG_INFO, PFX , fmt, ##arg); \
                } while(0)
#else
#define PK_DBG(a,...)
#define PK_ERR(a,...)
#define PK_XLOG_INFO(fmt, args...)
#endif


#define CAMERA_POWER_DOVDD  CAMERA_POWER_VCAM_D2
#define CAMERA_POWER_DVDD   CAMERA_POWER_VCAM_D
#define CAMERA_POWER_AVDD   CAMERA_POWER_VCAM_A
#define CAMERA_POWER_AF     CAMERA_POWER_VCAM_A2

#if 0


#define DOVDD_DELEY_COUNT          (20)
#define AVDD_DELAY_COUNT           (20)
#define DVDD_DELAY_COUNT           (20)
#define PWRDN_OFF_DELAY_COUNT      (30)
#define RST_DELAY_TO_PWRON_COUNT   (30)
#define PWRDN_ON_DELAY_COUNT       (30)
#define RST_DELAY_COUNT            (20)
#define PWR_STABLE_DELAY_COUNT     (30)

//static int MZ_GPIO_OUT_ZERO = 0;
//static int MZ_GPIO_OUT_ONE = 1;

int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, BOOL On, char* mode_name)
{
	u32 pinSetIdx = 0;
	int MZ_GPIO_OUT_ZERO = 0;
	int MZ_GPIO_OUT_ONE = 1;
	MT65XX_POWER_VOLTAGE MZ_VOL_DOVDD,MZ_VOL_AVDD,MZ_VOL_DVDD,MZ_VOL_AF;
	

	{
		MZ_GPIO_OUT_ZERO = 0;
		MZ_GPIO_OUT_ONE = 1;
	}
	
	//printk("<<<<CAMERA POWER>>>>currSensorName = %s MZ_GPIO_OUT_ZERO = %d  MZ_GPIO_OUT_ONE = %d\n",currSensorName, MZ_GPIO_OUT_ZERO,MZ_GPIO_OUT_ONE);
	if (DUAL_CAMERA_MAIN_SENSOR == SensorIdx){
		pinSetIdx = 0;
	}
    	else if (DUAL_CAMERA_SUB_SENSOR == SensorIdx) {
        	pinSetIdx = 1;
    	}
    	else if (DUAL_CAMERA_MAIN_2_SENSOR == SensorIdx) {
        	pinSetIdx = 2;
/*
		mt_set_gpio_out(GPIO_CAMERA_CMPDN_PIN ,GPIO_OUT_ZERO);
		mt_set_gpio_out(GPIO_CAMERA_CMRST_PIN ,GPIO_OUT_ZERO);
		mt_set_gpio_out(GPIO_CAMERA_CMPDN1_PIN ,GPIO_OUT_ZERO);
		mt_set_gpio_out(GPIO_CAMERA_CMRST1_PIN ,GPIO_OUT_ZERO);

		hwPowerDown(CAMERA_POWER_DOVDD, mode_name);
		mdelay(1);
		hwPowerDown(CAMERA_POWER_AVDD, mode_name);
		mdelay(1);
		hwPowerDown(CAMERA_POWER_DVDD, mode_name);
		mdelay(1);
		hwPowerDown(CAMERA_POWER_AF, mode_name);		
			
		return;
		*/
    	}		


	mt_set_gpio_mode(GPIO_CAMERA_CMPDN_PIN, GPIO_CAMERA_CMPDN_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CAMERA_CMPDN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_mode(GPIO_CAMERA_CMRST_PIN,GPIO_CAMERA_CMRST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CAMERA_CMRST_PIN,GPIO_DIR_OUT);		
	mt_set_gpio_mode(GPIO_CAMERA_CMRST1_PIN,GPIO_CAMERA_CMRST1_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CAMERA_CMRST1_PIN,GPIO_DIR_OUT);					
	mt_set_gpio_mode(GPIO_CAMERA_CMPDN1_PIN,GPIO_CAMERA_CMPDN1_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CAMERA_CMPDN1_PIN,GPIO_DIR_OUT);

	mt_set_gpio_out(GPIO_CAMERA_CMPDN_PIN ,GPIO_OUT_ZERO);
	mt_set_gpio_out(GPIO_CAMERA_CMRST_PIN ,GPIO_OUT_ZERO);
	mt_set_gpio_out(GPIO_CAMERA_CMPDN1_PIN ,GPIO_OUT_ZERO);
	mt_set_gpio_out(GPIO_CAMERA_CMRST1_PIN ,GPIO_OUT_ZERO);

	printk("<<<<CAMERA POWER>>>>currSensorName = %s SensorIdx = %d MZ_GPIO_OUT_ZERO = %d  MZ_GPIO_OUT_ONE = %d\n",currSensorName, pinSetIdx, MZ_GPIO_OUT_ZERO,MZ_GPIO_OUT_ONE);

	if( On )
	{
		if (currSensorName &&(0 == strcmp(SENSOR_DRVNAME_OV12830_MIPI_RAW, currSensorName)) && (pinSetIdx==0 || pinSetIdx==2))
		{
			printk("<<<<<<<<HONGZHE>>>>>>>>>> CAMERA OV12830 VOLTAGE SET");
			MZ_VOL_DOVDD = VOL_1800;
			MZ_VOL_AVDD =VOL_2800;
			MZ_VOL_DVDD=VOL_1200;
			MZ_VOL_AF=VOL_2800;	
			/*PWRDN  = 1 work*/

			hwPowerOn(CAMERA_POWER_AVDD, MZ_VOL_AVDD,mode_name);
			mdelay(10);
			hwPowerOn(CAMERA_POWER_DOVDD, MZ_VOL_DOVDD,mode_name);
			hwPowerOn(CAMERA_POWER_DVDD, MZ_VOL_DVDD,mode_name);
			mdelay(10);
			hwPowerOn(CAMERA_POWER_AF, MZ_VOL_AF,mode_name);
			mdelay(10);

			mt_set_gpio_out(GPIO_CAMERA_CMRST_PIN,GPIO_OUT_ZERO); //low == reset sensor
			mdelay(10);
			mt_set_gpio_out(GPIO_CAMERA_CMRST_PIN,GPIO_OUT_ONE); //HIGH == reset sensor
			mdelay(10);

			mt_set_gpio_out(GPIO_CAMERA_CMPDN_PIN, GPIO_OUT_ONE);
			mdelay(20);
		}
		else if (currSensorName &&(0 == strcmp(SENSOR_DRVNAME_S5K4E1GA_RAW, currSensorName)) && pinSetIdx==1)
		{
		#if 0
			printk("<<<<<<<<HONGZHE>>>>>>>>>> CAMERA S5K4E1GA VOLTAGE SET");
			MZ_VOL_DVDD=VOL_1500;
			MZ_VOL_AVDD =VOL_2800;
			MZ_VOL_DOVDD = VOL_1800;
			MZ_VOL_AF=VOL_2800;	
			/*PWRDN  = 1 work*/

			hwPowerOn(CAMERA_POWER_DOVDD, MZ_VOL_DOVDD,mode_name);
			mdelay(10);
			hwPowerOn(CAMERA_POWER_AVDD, MZ_VOL_AVDD,mode_name);
			mdelay(10);
			hwPowerOn(CAMERA_POWER_DVDD, MZ_VOL_DVDD,mode_name);
			mdelay(10);


			//hwPowerOn(CAMERA_POWER_AF, MZ_VOL_AF,mode_name);
			//mdelay(10);
			mt_set_gpio_out(GPIO_CAMERA_CMRST1_PIN,GPIO_OUT_ZERO); //HIGH == reset sensor
			mdelay(10);
			mt_set_gpio_out(GPIO_CAMERA_CMRST1_PIN,GPIO_OUT_ONE); //low == reset sensor
			mdelay(10);
			mt_set_gpio_out(GPIO_CAMERA_CMRST1_PIN,GPIO_OUT_ZERO); //HIGH == reset sensor
			mdelay(10);

			mt_set_gpio_out(GPIO_CAMERA_CMPDN1_PIN, GPIO_OUT_ONE);
			mdelay(20);
			#endif
		}
		else
		{
			return -EIO;
		}
	}
	else
	{
		mt_set_gpio_out(GPIO_CAMERA_CMPDN_PIN ,GPIO_OUT_ZERO);
		mt_set_gpio_out(GPIO_CAMERA_CMRST_PIN ,GPIO_OUT_ZERO);
		mt_set_gpio_out(GPIO_CAMERA_CMPDN1_PIN ,GPIO_OUT_ZERO);
		mt_set_gpio_out(GPIO_CAMERA_CMRST1_PIN ,GPIO_OUT_ZERO);

		hwPowerDown(CAMERA_POWER_DOVDD, mode_name);
		mdelay(1);
		hwPowerDown(CAMERA_POWER_AVDD, mode_name);
		mdelay(1);
		hwPowerDown(CAMERA_POWER_DVDD, mode_name);
		mdelay(1);
		hwPowerDown(CAMERA_POWER_AF, mode_name);		
	}




	return 0;

}

#else
/*
int kdCameraHwAFPowerOn(BOOL on, char * mode_name)
{
	if( on)
		hwPowerOn(CAMERA_POWER_AF, VOL_2800,mode_name);
	else
		hwPowerDown(CAMERA_POWER_AF,mode_name);
}
EXPORT_SYMBOL(kdCameraHwAFPowerOn);
*/
int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, BOOL On, char* mode_name)
{
u32 pinSetIdx = 0;//default main sensor
int rst_pin_state_sensor_work = GPIO_OUT_ONE;
int pwdn_pin_state_sensor_work = GPIO_OUT_ONE;
int rst_pin_state_sensor_off = GPIO_OUT_ZERO;
int pwdn_pin_state_sensor_off = GPIO_OUT_ZERO;

MT65XX_POWER_VOLTAGE dovdd_voltage;
MT65XX_POWER_VOLTAGE dvdd_voltage;
MT65XX_POWER_VOLTAGE avdd_voltage;
MT65XX_POWER_VOLTAGE afvdd_voltage;

#define IDX_PS_CMRST 0
#define IDX_PS_CMPDN 4

#define IDX_PS_MODE 1
#define IDX_PS_ON   2
#define IDX_PS_OFF  3


if (currSensorName &&(0 == strcmp(SENSOR_DRVNAME_OV12830_MIPI_RAW, currSensorName)) )
{
	rst_pin_state_sensor_work = GPIO_OUT_ONE;
	rst_pin_state_sensor_off = GPIO_OUT_ZERO;
	pwdn_pin_state_sensor_work = GPIO_OUT_ONE;
	pwdn_pin_state_sensor_off = GPIO_OUT_ZERO;

	dovdd_voltage = VOL_1800;
	dvdd_voltage = VOL_1200;
	avdd_voltage = VOL_2800;
	afvdd_voltage = VOL_2800;	
}
else if (currSensorName &&(0 == strcmp(SENSOR_DRVNAME_OV5648_MIPI_RAW, currSensorName)) )
{
	rst_pin_state_sensor_work = GPIO_OUT_ONE;
	rst_pin_state_sensor_off = GPIO_OUT_ZERO;
	pwdn_pin_state_sensor_work = GPIO_OUT_ONE;
	pwdn_pin_state_sensor_off = GPIO_OUT_ZERO;

	dovdd_voltage = VOL_1800;
	dvdd_voltage = VOL_1500;
	avdd_voltage = VOL_2800;
	afvdd_voltage = VOL_2800;	
}
else
{// s5k4
	rst_pin_state_sensor_work = GPIO_OUT_ZERO;
	rst_pin_state_sensor_off = GPIO_OUT_ONE;
#if 0	
	pwdn_pin_state_sensor_work = GPIO_OUT_ZERO;//GPIO_OUT_ONE;
	pwdn_pin_state_sensor_off =GPIO_OUT_ONE;// GPIO_OUT_ZERO;
#else
	pwdn_pin_state_sensor_work = GPIO_OUT_ONE;
	pwdn_pin_state_sensor_off =GPIO_OUT_ZERO;
#endif
	dovdd_voltage = VOL_1800;
	dvdd_voltage = VOL_1500;
	avdd_voltage = VOL_2800;
	afvdd_voltage = VOL_2800;		
}

u32 pinSet[3][8] = {
                    //for main sensor
                    {GPIO_CAMERA_CMRST_PIN,
                        GPIO_CAMERA_CMRST_PIN_M_GPIO,   /* mode */
                        rst_pin_state_sensor_work,//GPIO_OUT_ONE,                   /* ON state */
                        rst_pin_state_sensor_off,//GPIO_OUT_ZERO,                  /* OFF state */
                     GPIO_CAMERA_CMPDN_PIN,
                        GPIO_CAMERA_CMPDN_PIN_M_GPIO,
                        pwdn_pin_state_sensor_work,//GPIO_OUT_ONE,
                        pwdn_pin_state_sensor_off,//GPIO_OUT_ZERO,
                    },
                    //for sub sensor
                    {GPIO_CAMERA_CMRST1_PIN,
                     GPIO_CAMERA_CMRST1_PIN_M_GPIO,
                        rst_pin_state_sensor_work,//GPIO_OUT_ONE,                   /* ON state */
                        rst_pin_state_sensor_off,//GPIO_OUT_ZERO,                  /* OFF state */
                     GPIO_CAMERA_CMPDN1_PIN,
                        GPIO_CAMERA_CMPDN1_PIN_M_GPIO,
                        pwdn_pin_state_sensor_work,//GPIO_OUT_ONE,
                        pwdn_pin_state_sensor_off,//GPIO_OUT_ZERO,
                    },
                    //for main_2 sensor
                    {GPIO_CAMERA_2_CMRST_PIN,
                        GPIO_CAMERA_2_CMRST_PIN_M_GPIO,   /* mode */
                        rst_pin_state_sensor_work,//GPIO_OUT_ONE,                   /* ON state */
                        rst_pin_state_sensor_off,//GPIO_OUT_ZERO,                  /* OFF state */
                     GPIO_CAMERA_2_CMPDN_PIN,
                        GPIO_CAMERA_2_CMPDN_PIN_M_GPIO,
                        pwdn_pin_state_sensor_work,//GPIO_OUT_ONE,
                        pwdn_pin_state_sensor_off,//GPIO_OUT_ZERO,
                    }
                   };




    if (DUAL_CAMERA_MAIN_SENSOR == SensorIdx){
        pinSetIdx = 0;
    }
    else if (DUAL_CAMERA_SUB_SENSOR == SensorIdx) {
        pinSetIdx = 1;
    }
    else if (DUAL_CAMERA_MAIN_2_SENSOR == SensorIdx) {
        pinSetIdx = 2;
    }

    //power ON
    if (On) {


#if 0  //TODO: depends on HW layout. Should be notified by SA.

        PK_DBG("Set CAMERA_POWER_PULL_PIN for power \n");
        if (mt_set_gpio_pull_enable(GPIO_CAMERA_LDO_EN_PIN, GPIO_PULL_DISABLE)) {PK_DBG("[[CAMERA SENSOR] Set CAMERA_POWER_PULL_PIN DISABLE ! \n"); }
        if(mt_set_gpio_mode(GPIO_CAMERA_LDO_EN_PIN, GPIO_CAMERA_LDO_EN_PIN_M_GPIO)){PK_DBG("[[CAMERA SENSOR] set CAMERA_POWER_PULL_PIN mode failed!! \n");}
        if(mt_set_gpio_dir(GPIO_CAMERA_LDO_EN_PIN,GPIO_DIR_OUT)){PK_DBG("[[CAMERA SENSOR] set CAMERA_POWER_PULL_PIN dir failed!! \n");}
        if(mt_set_gpio_out(GPIO_CAMERA_LDO_EN_PIN,GPIO_OUT_ONE)){PK_DBG("[[CAMERA SENSOR] set CAMERA_POWER_PULL_PIN failed!! \n");}
#endif


        //enable active sensor
        //RST pin
        if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
            if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
            if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
            if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
            mdelay(10);
            if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
            mdelay(1);

            //PDN pin
            if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
            if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
            if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
        }



        //DOVDD
         hwPowerOn(CAMERA_POWER_DOVDD, dovdd_voltage,mode_name);
        mdelay(10);


        //AVDD
        hwPowerOn(CAMERA_POWER_AVDD, avdd_voltage,mode_name);
        mdelay(10);
		
        //DVDD
       hwPowerOn(CAMERA_POWER_DVDD, dvdd_voltage,mode_name);
        mdelay(10);
        //AF_VCC
        hwPowerOn(CAMERA_POWER_AF, afvdd_voltage,mode_name);


#if 1
        //enable active sensor
        if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
            if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
            if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
            if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
            mdelay(10);
            if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
            mdelay(1);

            //PDN pin
            if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
            if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
            if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
        }
#endif

        //disable inactive sensor
        if(pinSetIdx == 0 || pinSetIdx == 2) {//disable sub
	        if (GPIO_CAMERA_INVALID != pinSet[1][IDX_PS_CMRST]) {
	            if(mt_set_gpio_mode(pinSet[1][IDX_PS_CMRST],pinSet[1][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
	            if(mt_set_gpio_mode(pinSet[1][IDX_PS_CMPDN],pinSet[1][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
	            if(mt_set_gpio_dir(pinSet[1][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
	            if(mt_set_gpio_dir(pinSet[1][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
	            if(mt_set_gpio_out(pinSet[1][IDX_PS_CMRST],pinSet[1][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
	            if(mt_set_gpio_out(pinSet[1][IDX_PS_CMPDN],pinSet[1][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
	        }
        }
		else {
	        if (GPIO_CAMERA_INVALID != pinSet[0][IDX_PS_CMRST]) {
	            if(mt_set_gpio_mode(pinSet[0][IDX_PS_CMRST],pinSet[0][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
	            if(mt_set_gpio_mode(pinSet[0][IDX_PS_CMPDN],pinSet[0][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
	            if(mt_set_gpio_dir(pinSet[0][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
	            if(mt_set_gpio_dir(pinSet[0][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
	            if(mt_set_gpio_out(pinSet[0][IDX_PS_CMRST],pinSet[0][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
	            if(mt_set_gpio_out(pinSet[0][IDX_PS_CMPDN],pinSet[0][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
	        }
	        /*if (GPIO_CAMERA_INVALID != pinSet[2][IDX_PS_CMRST]) {
	            if(mt_set_gpio_mode(pinSet[2][IDX_PS_CMRST],pinSet[2][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
	            if(mt_set_gpio_mode(pinSet[2][IDX_PS_CMPDN],pinSet[2][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
	            if(mt_set_gpio_dir(pinSet[2][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
	            if(mt_set_gpio_dir(pinSet[2][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
	            if(mt_set_gpio_out(pinSet[2][IDX_PS_CMRST],pinSet[2][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
	            if(mt_set_gpio_out(pinSet[2][IDX_PS_CMPDN],pinSet[2][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
	        }*/
		}


    }
    else {//power OFF

#if 0 //TODO: depends on HW layout. Should be notified by SA.
        PK_DBG("Set GPIO 94 for power OFF\n");
        if (mt_set_gpio_pull_enable(GPIO_CAMERA_LDO_EN_PIN, GPIO_PULL_DISABLE)) {PK_DBG("[CAMERA SENSOR] Set GPIO94 PULL DISABLE ! \n"); }
        if(mt_set_gpio_mode(GPIO_CAMERA_LDO_EN_PIN, GPIO_CAMERA_LDO_EN_PIN_M_GPIO)){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
        if(mt_set_gpio_dir(GPIO_CAMERA_LDO_EN_PIN,GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
        if(mt_set_gpio_out(GPIO_CAMERA_LDO_EN_PIN,GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
#endif

        //PK_DBG("[OFF]sensorIdx:%d \n",SensorIdx);
        if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) {
            if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
            if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
            if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
            if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
            if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
    	    if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
        }

    	if(TRUE != hwPowerDown(CAMERA_POWER_AVDD,mode_name)) {
            PK_DBG("[CAMERA SENSOR] Fail to OFF analog power\n");
            //return -EIO;
            goto _kdCISModulePowerOn_exit_;
        }
        if(TRUE != hwPowerDown(CAMERA_POWER_AF,mode_name))
        {
            PK_DBG("[CAMERA SENSOR] Fail to enable analog power\n");
            //return -EIO;
            goto _kdCISModulePowerOn_exit_;
        }
        if(TRUE != hwPowerDown(CAMERA_POWER_DVDD, mode_name)) {
            PK_DBG("[CAMERA SENSOR] Fail to OFF digital power\n");
            //return -EIO;
            goto _kdCISModulePowerOn_exit_;
        }
        if(TRUE != hwPowerDown(CAMERA_POWER_DOVDD,mode_name))
        {
            PK_DBG("[CAMERA SENSOR] Fail to enable digital power\n");
            //return -EIO;
            goto _kdCISModulePowerOn_exit_;
        }
    }//

	return 0;

_kdCISModulePowerOn_exit_:
    return -EIO;
}
#endif
EXPORT_SYMBOL(kdCISModulePowerOn);


//!--
//




