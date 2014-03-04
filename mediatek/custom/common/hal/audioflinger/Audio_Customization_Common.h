/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein is
 * confidential and proprietary to MediaTek Inc. and/or its licensors. Without
 * the prior written permission of MediaTek inc. and/or its licensors, any
 * reproduction, modification, use or disclosure of MediaTek Software, and
 * information contained herein, in whole or in part, shall be strictly
 * prohibited.
 *
 * MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER
 * ON AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL
 * WARRANTIES, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR
 * NONINFRINGEMENT. NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH
 * RESPECT TO THE SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY,
 * INCORPORATED IN, OR SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES
 * TO LOOK ONLY TO SUCH THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO.
 * RECEIVER EXPRESSLY ACKNOWLEDGES THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO
 * OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES CONTAINED IN MEDIATEK
 * SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE
 * RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S
 * ENTIRE AND CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE
 * RELEASED HEREUNDER WILL BE, AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE
 * MEDIATEK SOFTWARE AT ISSUE, OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE
 * CHARGE PAID BY RECEIVER TO MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek
 * Software") have been modified by MediaTek Inc. All revisions are subject to
 * any receiver's applicable license agreements with MediaTek Inc.
 */

#ifndef AUDIO_CUSTOMIZATION_COMMON_H
#define AUDIO_CUSTOMIZATION_COMMON_H

/****************************************************
* Define Volume Range of  sound & Voice.
*****************************************************/
#define DEVICE_MAX_VOLUME           (12)
#define DEVICE_VOICE_MAX_VOLUME     (12)
#define DEVICE_MIN_VOLUME           (-4)
#define DEVICE_VOICE_MIN_VOLUME     (-4)
#define DEVICE_VOLUME_RANGE     (64)
#define DEVICE_VOLUME_STEP (256)

/***************************************************
*adjust boot animation volume. the volume range is from 0 to 1.
*****************************************************/
#define BOOT_ANIMATION_VOLUME       (0.25)

/***************************************************
*(1)->Use Ref Mic as main mic; (0)->Use original main mic.
*****************************************************/
#define USE_REFMIC_IN_LOUDSPK       (0)

/****************************************************
* Define this will enable audio compensation filter for loudspeaker
*Please see ACF Document for detail.
*****************************************************/
#define ENABLE_AUDIO_COMPENSATION_FILTER

/***************************************************
* Define this will enable DRC for loudspeaker.
*****************************************************/
#define ENABLE_AUDIO_DRC_SPEAKER


/***************************************************
* Define this will enable headphone compensation filter.
*Please see HCF Document for detail.
*****************************************************/
#define ENABLE_HEADPHONE_COMPENSATION_FILTER
#define HEADPHONE_COMPENSATION_FLT_MODE (4)


/***************************************************
*Define this will enable SW stereo to mono on LCH & RCH
*If not define this, HW stereo to mono (only LCH) will be applied.
*****************************************************/
#define ENABLE_AUDIO_SW_STEREO_TO_MONO


/***************************************************
*Define this will enable high samplerate record.
*****************************************************/
#define ENABLE_HIGH_SAMPLERATE_RECORD


/****************************************************
* WARNING: this macro is now obsolete, please change
* the property value ro.camera.sound.forced=1 to take effect.
*the property is defined in alps\mediatek\config\YOUR_PROJECT
*\system.prop.
*****************************************************/
//#define FORCE_CAMERA_SHUTTER_SOUND_AUDIBLE


/****************************************************
* Define this  , speaker output will not do stero to mono,
*keep in stereo format,because stereo output can apply
*on more than 1 speaker.
*****************************************************/
//#define ENABLE_STEREO_SPEAKER


/****************************************************
* Define this will enable Voice  to use  VoiceBuffer
*when using speaker and headphone in incall mode.
*****************************************************/
//#define ALL_USING_VOICEBUFFER_INCALL


/****************************************************
*Define this, audioflinger will use first active stream samplerate
*as hardware setting. it is only used for verifying hardware
*****************************************************/
//#define AUDIO_HQA_SUPPORT

#define AUDIO_DROP_FRAME_COUNT_NORMAL 5
#define AUDIO_DROP_FRAME_COUNT_RECORD 5
#define AUDIO_DROP_FRAME_COUNT_CTS 5


/****************************************************
*Define this, HD Rec will use this for default param if no mode is set
*****************************************************/
#define DEFAULT_HDRecordEnhanceParas \
	0, 479, 16388, 36892, 37124, 8192,  768, 0,  4048, 2245, 611, 0, 0, 0, 0, 8192

#define DEFAULT_HDRecordCompenFilter \
    32767,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0, \
        0,     0,     0,     0,     0,     0,     0,     0,     0,     0
#endif