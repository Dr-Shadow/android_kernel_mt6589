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
#include <cust_alsps.h>
#include <mach/mt_pm_ldo.h>

#ifdef VENDOR_EDIT//Shaoyu.Huang@Prd.BasicDrv.Sensor,add 2012/5/28 for alsps power
#include <linux/delay.h>
#include <mach/mt_gpio.h>

static int power(struct alsps_hw *hw, unsigned int on, char *devname)
{
	static unsigned int status = 0;
	if (on == status){
		return 0;
	}
	if (on){
    //Fuchun.Liao modify VGP6 from 3.3V to 2.8v
	hwPowerOn(MT65XX_POWER_LDO_VGP6,VOL_2800,"alsps_2800"); 
	udelay(100);
    hwPowerOn(MT65XX_POWER_LDO_VGP4,VOL_1800,"alsps_1800");
	   
	
	}
	else{
	hwPowerDown(MT65XX_POWER_LDO_VGP4,"alsps_1800");
	udelay(10);
    hwPowerDown(MT65XX_POWER_LDO_VGP6,"alsps_2800");
	}

	status = on;
	return 0;
}
#endif/*VENDOR_EDIT*/
static struct ps_adjust_para cust_ps_adjust_para_tmd2772 = {
    .ps_up = 60,
    .ps_thd_low_notrend = 40,
    .ps_thd_high_notrend = 60,
    .ps_thd_low_trend = 40,
    .ps_thd_high_trend = 50,
    .ps_thd_low_highlight = 600,
    .ps_thd_high_highlight = 650,
    .ps_adjust_min = 0,
    .ps_adjust_max = 850,
    .highlight_limit = 8000,
    .sampling_time = 20, //Unit:ms
    .sampling_count = 6,
    .dirty_adjust_limit = 900,
    .dirty_adjust_low_thd = 290,
    .dirty_adjust_high_thd = 300,
};
static struct ps_adjust_para cust_ps_adjust_para_tmd2771 = {
    .ps_up = 60,
    .ps_thd_low_notrend = 110,
    .ps_thd_high_notrend = 120,
    .ps_thd_low_trend = 50,
    .ps_thd_high_trend = 60,
    .ps_thd_low_highlight = 600,
    .ps_thd_high_highlight = 650,
    .ps_adjust_min = 0,
    .ps_adjust_max = 850,
    .highlight_limit = 8000,
    .sampling_time = 20, //Unit:ms
    .sampling_count = 6,
    .dirty_adjust_limit = 900,
    .dirty_adjust_low_thd = 290,
    .dirty_adjust_high_thd = 300,
};


static struct alsps_hw cust_alsps_hw = {
    .i2c_num    = 0,//i2c-3
	.polling_mode_ps =0, //0 for interrupt mode
	.polling_mode_als =1,
    .power_id   = MT65XX_POWER_NONE,    /*LDO is not used*/
    .power_vol  = VOL_DEFAULT,          /*LDO is not used*/
    .i2c_addr   = {0x72, 0x48, 0x78, 0x00},
    .als_level  = { 30, 800, 2500, 5500, 8500, 65535},
    .als_value  = {29, 799, 2499, 5499, 8499, 65534},
    .ps_threshold_high = 0, //no use 
    .ps_threshold_low = 0, //no use
    .ps_threshold = 0,    //no use
    .p_ps_adjust_para = &cust_ps_adjust_para_tmd2772,
	.power = power,
};

struct alsps_hw *get_cust_alsps_hw_tmd2772(void) {
    cust_alsps_hw.p_ps_adjust_para = &cust_ps_adjust_para_tmd2772;
    return &cust_alsps_hw;
}
struct alsps_hw *get_cust_alsps_hw_tmd2771(void) {
    cust_alsps_hw.p_ps_adjust_para = &cust_ps_adjust_para_tmd2771;
    return &cust_alsps_hw;
}


