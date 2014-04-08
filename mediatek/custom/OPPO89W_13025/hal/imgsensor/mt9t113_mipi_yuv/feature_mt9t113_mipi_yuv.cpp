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

/********************************************************************************************
 *     LEGAL DISCLAIMER
 *
 *     (Header of MediaTek Software/Firmware Release or Documentation)
 *
 *     BY OPENING OR USING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 *     THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE") RECEIVED
 *     FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON AN "AS-IS" BASIS
 *     ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES, EXPRESS OR IMPLIED,
 *     INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR
 *     A PARTICULAR PURPOSE OR NONINFRINGEMENT. NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY
 *     WHATSOEVER WITH RESPECT TO THE SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY,
 *     INCORPORATED IN, OR SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK
 *     ONLY TO SUCH THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
 *     NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S SPECIFICATION
 *     OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
 *
 *     BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE LIABILITY WITH
 *     RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE, AT MEDIATEK'S OPTION,
TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE, OR REFUND ANY SOFTWARE LICENSE
 *     FEES OR SERVICE CHARGE PAID BY BUYER TO MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 *     THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE WITH THE LAWS
 *     OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF LAWS PRINCIPLES.
 ************************************************************************************************/


/*******************************************************************************
* Configuration Info
* (it's needed for sensor authors to specify the following info)
*******************************************************************************/
//  header file name where sensor features are specified;
//  it is included in this source file below.
#define CFG_FTBL_FILENAME   "cfg_ftbl_mt9t113_mipi_yuv.h"

//  sensor id; the same as specified in SensorList[] in sensorlist.cpp.
#define SENSOR_ID           MT9T113MIPI_SENSOR_ID

//  sensor name; just for debug log now.
#define SENSOR_NAME         "[mt9t113mipi_yuv]"


/*******************************************************************************
* 
*******************************************************************************/
#define LOG_TAG "feature_YUV"
//
#include <utils/Errors.h>
#include <cutils/log.h>
//
#define USE_CAMERA_FEATURE_MACRO 1  //define before "camera_feature.h"
#include "camera_feature.h"


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  Local Define
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#define TOTAL_TABLE_SCENE_NUM   static_cast<MUINT32>(NSFeature::ENumOfScene)


/*******************************************************************************
* MACRO Define: Scene Independent
*******************************************************************************/
#define GETFINFO_SCENE_INDEP()          _GETFINFO_SCENE_INDEP(SENSOR_NAME)
#define END_GETFINFO_SCENE_INDEP()  _END_GETFINFO_SCENE_INDEP(SENSOR_NAME)


/*******************************************************************************
* MACRO Define: Scene Dependent
*******************************************************************************/
#define GETFINFO_SCENE_DEP()            _GETFINFO_SCENE_DEP(SENSOR_NAME)
#define END_GETFINFO_SCENE_DEP()    _END_GETFINFO_SCENE_DEP(SENSOR_NAME)


/*******************************************************************************
* MACRO Define: Config Scene
*******************************************************************************/
#define CONFIG_SCENE(_sid)              _CONFIG_SCENE(_sid, SENSOR_NAME)
#define END_CONFIG_SCENE()          _END_CONFIG_SCENE(SENSOR_NAME)


/*******************************************************************************
* MACRO Define: Config Feature
*******************************************************************************/
#define CHECK_FID_SI    CHECK_FID_YUV_SI
#define CHECK_FID_SD    CHECK_FID_YUV_SD


/*******************************************************************************
* Implementation of Feature Tables
*******************************************************************************/
namespace
{
using namespace NSFeature;
#include CFG_FTBL_FILENAME
}


static
inline
NSFeature::PF_GETFINFO_SCENE_INDEP_T
GetFInfo_YUV_SI()
{
    return  NSYUV::NSSceneIndep::GetFInfo;
}


static
inline
NSFeature::PF_GETFINFO_SCENE_DEP_T
GetFInfo_YUV_SD()
{
    return  NSYUV::NSSceneDep::GetFInfo<TOTAL_TABLE_SCENE_NUM>;
}


/*******************************************************************************
* Implementation of class SensorInfo
*******************************************************************************/
#include "camera_custom_sensor.h"
#include "kd_imgsensor.h"


namespace NSFeature
{


typedef YUVSensorInfo<SENSOR_ID> SensorInfoSingleton_T;


template <>
SensorInfoBase*
SensorInfoSingleton_T::
GetInstance()
{
    static SensorInfoSingleton_T singleton;
    return &singleton;
}


template <>
MBOOL
SensorInfoSingleton_T::
GetFeatureProvider(FeatureInfoProvider_T& rFInfoProvider)
{
    rFInfoProvider.pfGetFInfo_SceneIndep = GetFInfo_YUV_SI();
    rFInfoProvider.pfGetFInfo_SceneDep   = GetFInfo_YUV_SD();

    return  MTRUE;
}


};  //  NSFeature

