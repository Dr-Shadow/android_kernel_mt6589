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
 

 
 
// use accdet + EINT solution
#ifndef VENDOR_EDIT
/* xiang.fei@MutimediaDrv.audio, 2012/06/12, Remove for headset */

#define ACCDET_EINT

#ifdef ACCDET_EINT
#define ACCDET_DELAY_ENABLE_TIME 500 //delay 500ms to enable accdet after EINT 
//#define ACCDET_EINT_HIGH_ACTIVE //defaule low active if not define ACCDET_EINT_HIGH_ACTIVE
#endif
#endif /* VENDOR_EDIT */
// support multi_key feature
//#define ACCDET_MULTI_KEY_FEATURE
// after 5s disable accdet
//#define ACCDET_LOW_POWER

//#define ACCDET_PIN_RECOGNIZATION

struct headset_mode_settings{
    int pwm_width;	//pwm frequence
    int pwm_thresh;	//pwm duty 
    int fall_delay;	//falling stable time
    int rise_delay;	//rising stable time
    int debounce0;	//hook switch or double check debounce
    int debounce1;	//mic bias debounce
    int debounce3;	//plug out debounce
};

//key press customization: long press time
int long_press_time = 2000;

#ifndef VENDOR_EDIT
//li.zhanhua@Prd.BasicDrv.audio,2013/01/08, remove for headset
#ifdef  ACCDET_MULTI_KEY_FEATURE
struct headset_mode_settings cust_headset_settings = {
	0x900, 0x200, 1, 0x1f0, 0x800, 0x800, 0x20
};
#else
//headset mode register settings(for MT6575)
struct headset_mode_settings cust_headset_settings = {
	//0x1900, 0x140, 1, 0x12c, 0x3000, 0x3000, 0x400

	0x900, 0x200, 1, 0x1f0, 0x800, 0x800, 0x20
};
struct headset_mode_settings sleep_headset_settings = {
	//0x1900, 0x140, 1, 0x12c, 0x3000, 0x3000, 0x400

	0x900, 0x200, 1, 0x1f0, 0x800, 0x800, 0x20
};

#endif

#else /*VENDOR_DEDIT*/
struct headset_mode_settings cust_headset_settings = {
	0x450, 0x227, 1, 0x60, 0x300, 0x2, 0x400
};
struct headset_mode_settings sleep_headset_settings = {
	0x1900, 0x140, 1, 0x12c, 0x3000, 0x3000, 0x400
};
#endif
