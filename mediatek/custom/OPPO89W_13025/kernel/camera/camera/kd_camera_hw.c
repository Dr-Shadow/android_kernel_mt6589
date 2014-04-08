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
#include <linux/kernel.h>//for printk


#include "kd_camera_hw.h"

#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_camera_feature.h"

/******************************************************************************
 * Debug configuration
******************************************************************************/
#define PFX "[kd_camera_hw]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    printk(KERN_INFO PFX "%s: " fmt, __FUNCTION__ ,##arg)

#define DEBUG_CAMERA_HW_K
#ifdef DEBUG_CAMERA_HW_K
#define PK_DBG PK_DBG_FUNC
#define PK_ERR(fmt, arg...)         printk(KERN_ERR PFX "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_XLOG_INFO(fmt, args...) \
                do {    \
                    xlog_printk(ANDROID_LOG_INFO, "kd_camera_hw", fmt, ##args); \
                } while(0)
#else
#define PK_DBG(a,...)
#define PK_ERR(a,...)
#define PK_XLOG_INFO(fmt, args...)
#endif



int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, BOOL On, char* mode_name)
{
u32 pinSetIdx = 0;//default main sensor

#define IDX_PS_CMRST 0
#define IDX_PS_CMPDN 4

#define IDX_PS_MODE 1
#define IDX_PS_ON   2
#define IDX_PS_OFF  3   
u32 pinSet[3][8] = {
                    //for main sensor
                    {GPIO_CAMERA_CMRST_PIN,
                        GPIO_CAMERA_CMRST_PIN_M_GPIO,   /* mode */
                        GPIO_OUT_ONE,                   /* ON state */
                        GPIO_OUT_ZERO,                  /* OFF state */
                     GPIO_CAMERA_INVALID,
                        0,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                    },
                    //for sub sensor
                    {GPIO_CAMERA_CMRST1_PIN,
                     GPIO_CAMERA_CMRST1_PIN_M_GPIO,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                     GPIO_CAMERA_CMPDN1_PIN,
                        GPIO_CAMERA_CMPDN1_PIN_M_GPIO,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                    },
                    //for main_2 sensor
                    {GPIO_CAMERA_2_CMRST_PIN,
                        GPIO_CAMERA_2_CMRST_PIN_M_GPIO,   /* mode */
                        GPIO_OUT_ONE,                   /* ON state */
                        GPIO_OUT_ZERO,                  /* OFF state */
                     GPIO_CAMERA_2_CMPDN_PIN,
                        GPIO_CAMERA_2_CMPDN_PIN_M_GPIO,
                        GPIO_OUT_ONE,
                        GPIO_OUT_ZERO,
                    }
                   };




    if (DUAL_CAMERA_MAIN_SENSOR == SensorIdx){
        pinSetIdx = 0;
    }
    else if (DUAL_CAMERA_SUB_SENSOR == SensorIdx) {
        pinSetIdx = 1;
    }
    else if (DUAL_CAMERA_MAIN_2_SENSOR == SensorIdx) {
       // pinSetIdx = 2;
    }

    //power ON
    if (On) 
	{
		PK_DBG("[POWER ON]------pinSetIdx [%d]----->Start \r\n",pinSetIdx);
		
		if (0 == pinSetIdx)
		{
			#ifdef OPPO_R819
			//LiuBin@MtkCamera, 2012/11/08, Add for pull down SPI_CS
			if(mt_set_gpio_mode(GPIO134, GPIO_MODE_00)){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
			if(mt_set_gpio_dir(GPIO134,GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
			if(mt_set_gpio_out(GPIO134,GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
			#endif /* OPPO_R819 */

#ifndef OPPO_R819
//Jinshui.Liu@wxkf.wxBaseDrv, 2013/02/25, Modify for different hareware
			//LiuBin@MtkCamera, 2012/09/05, Add for pull up ISP_1V2_EN 	
			if(mt_set_gpio_mode(GPIO32, GPIO_MODE_00)){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
			if(mt_set_gpio_dir(GPIO32,GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
			if(mt_set_gpio_out(GPIO32,GPIO_OUT_ONE)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
			mdelay(5);
#else /*OPPO_R819*/
            if(TRUE != hwPowerOn(MT65XX_POWER_LDO_VAST, VOL_1200,mode_name))
            {
                PK_DBG("[CAMERA SENSOR] Fail to enable vast power\n");
                goto _kdCISModulePowerOn_exit_;
            }
#endif /*OPPO_R819*/

			if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
	        {
	            PK_DBG("[CAMERA SENSOR] Fail to enable Analog power\n");
	            goto _kdCISModulePowerOn_exit_;
	        }

			if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_IO, VOL_1800,mode_name))
	        {
	            PK_DBG("[CAMERA SENSOR] Fail to enable IO power\n");
	            goto _kdCISModulePowerOn_exit_;
	        }

			if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1200,mode_name))
			{
				 PK_DBG("[CAMERA SENSOR] Fail to enable Digital power\n");
				 goto _kdCISModulePowerOn_exit_;
			}

			if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_AF, VOL_3300,mode_name))
	        {
	            PK_DBG("[CAMERA SENSOR] Fail to enable AF power\n");
	            goto _kdCISModulePowerOn_exit_;
	        }
			
			if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) 
			{
				//PDN pin
				if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
				if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
				mdelay(1);
				if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				mdelay(1);
				if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
				mdelay(1);
				if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
				if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
				if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
				mdelay(5);
	        }	
		}
		else if (1 == pinSetIdx)
		{
			if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
			if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
			if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_ON])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
			mdelay(1);
			if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
			if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
			if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
			mdelay(1);
			if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_IO, VOL_1800,mode_name))  
			{
				PK_DBG("[CAMERA SENSOR] Fail to enable Digital power\n");
			}
			mdelay(1);
			if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D, VOL_1800,mode_name))  
			{
				PK_DBG("[CAMERA SENSOR] Fail to enable Digital power\n");
			}
			mdelay(1);
			if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_A, VOL_2800,mode_name))
			{
				PK_DBG("[CAMERA SENSOR] Fail to enable Analog power\n");
			}
			mdelay(10);
			if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");}
			//if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_ON])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}

			mdelay(1);
			
		}

		
		#if 1
        //disable inactive sensor
        if(pinSetIdx == 0 || pinSetIdx == 2) 
		{	
			//disable sub
	        if (GPIO_CAMERA_INVALID != pinSet[1][IDX_PS_CMRST])
			{
	            if(mt_set_gpio_mode(pinSet[1][IDX_PS_CMRST],pinSet[1][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
	            if(mt_set_gpio_mode(pinSet[1][IDX_PS_CMPDN],pinSet[1][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
	            if(mt_set_gpio_dir(pinSet[1][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
	            if(mt_set_gpio_dir(pinSet[1][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
	            if(mt_set_gpio_out(pinSet[1][IDX_PS_CMRST],pinSet[1][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
	            if(mt_set_gpio_out(pinSet[1][IDX_PS_CMPDN],pinSet[1][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
	        }
        }
		else 
		{
	        if (GPIO_CAMERA_INVALID != pinSet[0][IDX_PS_CMRST]) 
			{
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
		#endif
		
		PK_DBG("[POWER ON]------ pinSetIdx [%d]----->End \r\n", pinSetIdx);
    }
    else 
	{	
		//POWER OFF
		PK_DBG("[POWER OFF]------ pinSetIdx [%d]-----> Start \r\n", pinSetIdx);
		if (GPIO_CAMERA_INVALID != pinSet[pinSetIdx][IDX_PS_CMRST]) 
		{
            if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_MODE])){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
            if(mt_set_gpio_mode(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_MODE])){PK_DBG("[CAMERA LENS] set gpio mode failed!! \n");}
            if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMRST],GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
            if(mt_set_gpio_dir(pinSet[pinSetIdx][IDX_PS_CMPDN],GPIO_DIR_OUT)){PK_DBG("[CAMERA LENS] set gpio dir failed!! \n");}
			if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMPDN],pinSet[pinSetIdx][IDX_PS_CMPDN+IDX_PS_OFF])){PK_DBG("[CAMERA LENS] set gpio failed!! \n");} //high == power down lens module
            if(mt_set_gpio_out(pinSet[pinSetIdx][IDX_PS_CMRST],pinSet[pinSetIdx][IDX_PS_CMRST+IDX_PS_OFF])){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");} //low == reset sensor
        }

		if (0 == pinSetIdx)
		{
			if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_AF,mode_name)) 
			{
				PK_DBG("[CAMERA SENSOR] Fail to OFF AF power\n");
				goto _kdCISModulePowerOn_exit_;
			}
			if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D, mode_name)) 
			{
				PK_DBG("[CAMERA SENSOR] Fail to OFF Digital power\n");
				goto _kdCISModulePowerOn_exit_;
			}
			if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name)) 
		    {
			    PK_DBG("[CAMERA SENSOR] Fail to OFF Analog power\n");
			    goto _kdCISModulePowerOn_exit_;
		    }

			if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_IO, mode_name)) 
			{
				PK_DBG("[CAMERA SENSOR] Fail to OFF IO power\n");
				goto _kdCISModulePowerOn_exit_;
			}
			
#ifndef OPPO_R819
//Jinshui.Liu@wxkf.wxBaseDrv, 2013/02/25, Modify for different hareware
			#ifdef OPPO_R819
			//LiuBin@MtkCamera, 2012/09/05, Add for pull down ISP_1V2_EN 
			if(mt_set_gpio_mode(GPIO32, GPIO_MODE_00)){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
			if(mt_set_gpio_dir(GPIO32, GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
			if(mt_set_gpio_out(GPIO32, GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
			#endif /* OPPO_R819 */
#else /*OPPO_R819*/
            if(TRUE != hwPowerDown(MT65XX_POWER_LDO_VAST, mode_name)) 
            {
                PK_DBG("[CAMERA SENSOR] Fail to OFF vast power\n");
                goto _kdCISModulePowerOn_exit_;
            }
#endif /*OPPO_R819*/
			
		}
		else if (1 == pinSetIdx)
		{
			if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_IO, mode_name)) 
			{
		        PK_DBG("[CAMERA SENSOR] Fail to OFF digital power\n");
		        goto _kdCISModulePowerOn_exit_;
		    }
			if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D, mode_name)) 
			{
		        PK_DBG("[CAMERA SENSOR] Fail to OFF digital power\n");
		        goto _kdCISModulePowerOn_exit_;
		    }
			
    		if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_A,mode_name)) 
			{
	            PK_DBG("[CAMERA SENSOR] Fail to OFF analog power\n");
	            goto _kdCISModulePowerOn_exit_;
        	}
			
			#if 0 //TODO: depends on HW layout. Should be notified by SA.
			PK_DBG("Set GPIO 94 for power OFF\n");
			if (mt_set_gpio_pull_enable(GPIO_CAMERA_LDO_EN_PIN, GPIO_PULL_DISABLE)) {PK_DBG("[CAMERA SENSOR] Set GPIO94 PULL DISABLE ! \n"); }
			if(mt_set_gpio_mode(GPIO_CAMERA_LDO_EN_PIN, GPIO_CAMERA_LDO_EN_PIN_M_GPIO)){PK_DBG("[CAMERA SENSOR] set gpio mode failed!! \n");}
			if(mt_set_gpio_dir(GPIO_CAMERA_LDO_EN_PIN,GPIO_DIR_OUT)){PK_DBG("[CAMERA SENSOR] set gpio dir failed!! \n");}
			if(mt_set_gpio_out(GPIO_CAMERA_LDO_EN_PIN,GPIO_OUT_ZERO)){PK_DBG("[CAMERA SENSOR] set gpio failed!! \n");}
			#endif
		}

		PK_DBG("[POWER OFF]------ pinSetIdx [%d]-----> End \r\n", pinSetIdx);
    }//

	return 0;

_kdCISModulePowerOn_exit_:
    return -EIO;
}

EXPORT_SYMBOL(kdCISModulePowerOn);


//!--
//




