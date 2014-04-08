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
#include <linux/types.h>
#include <mach/mt_pm_ldo.h>
#include <cust_mag.h>

#ifdef OPPO_R819//Shaoyu.Huang@Prd.BasicDrv.Sensor,add 2012/5/28 for gsensor power
#include <linux/delay.h>
#include <mach/mt_gpio.h>
static int power(struct mag_hw *hw, unsigned int on, char *devname)
{
	static unsigned int status = 0;
	if (on == status){
		return 0;
	}
	if (on){
		//mt_set_gpio_mode(49, 0);
		//mt_set_gpio_dir(49,1);
		//mt_set_gpio_out(49, 1);

		hwPowerOn(MT65XX_POWER_LDO_VMCH1,VOL_3000,"msensor");
		msleep(1);
		hwPowerOn(MT65XX_POWER_LDO_VGP5,VOL_1800,"msensor");
		msleep(1);
	}
	else{
		hwPowerDown(MT65XX_POWER_LDO_VGP5,"msensor");
		msleep(1);
		hwPowerDown(MT65XX_POWER_LDO_VMCH1,"msensor");
	}

	status = on;
	return 0;
}
#endif/*OPPO_R819*/

static struct mag_hw cust_mag_hw = {
    .i2c_num = 2,//i2c-3
    .direction = 7,
    .power_id = MT65XX_POWER_NONE,  /*!< LDO is not used */
    .power_vol= VOL_DEFAULT,        /*!< LDO is not used */
#ifdef OPPO_R819//mingqiang.guo@Prd.BasicDrv.Sensor, add 2012/8/13 for msensor power
	.power = power,
#endif/*OPPO_R819*/
};
struct mag_hw* get_cust_mag_hw_mmc3461(void) 
{
    return &cust_mag_hw;
}
