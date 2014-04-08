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
#ifndef _CAMERA_FEATURE_ID_H_
#define _CAMERA_FEATURE_ID_H_


namespace NSFeature
{


#include "kd_camera_feature_id.h"
typedef FEATURE_ID  FEATURE_ID_T;


enum
{
    FID_BEGIN               = FID_PRE_BEGIN + 1, 
    FID_NUM                 = FID_OVER_LAST - FID_BEGIN, 

  //////////////////////////////////////////////////////////////////////////////
  // Scene-Independent (SI) feature id.
    FID_BEGIN_SI            = FID_PRE_BEGIN_SI + 1, 
    FID_END_SI              = FID_OVER_LAST_SI, 

    //Misc. feature id.
    FID_BEGIN_MISC_SI       = FID_PRE_BEGIN_MISC_SI + 1, 
    FID_END_MISC_SI         = FID_OVER_LAST_MISC_SI, 
    FID_NUM_MISC_SI         = FID_END_MISC_SI - FID_BEGIN_MISC_SI, 

    //RAW-only feature id.
    FID_BEGIN_RAW_ONLY_SI   = FID_PRE_BEGIN_RAW_ONLY_SI + 1, 
    FID_END_RAW_ONLY_SI     = FID_OVER_LAST_RAW_ONLY_SI, 
    FID_NUM_RAW_ONLY_SI     = FID_END_RAW_ONLY_SI - FID_BEGIN_RAW_ONLY_SI, 
    //RAW-YUV-shared feature id.
    FID_BEGIN_RAW_YUV_SI    = FID_PRE_BEGIN_RAW_YUV_SI + 1, 
    FID_END_RAW_YUV_SI      = FID_OVER_LAST_RAW_YUV_SI, 
    FID_NUM_RAW_YUV_SI      = FID_END_RAW_YUV_SI - FID_BEGIN_RAW_YUV_SI, 
    //YUV-only feature id.
    FID_BEGIN_YUV_ONLY_SI   = FID_PRE_BEGIN_YUV_ONLY_SI + 1, 
    FID_END_YUV_ONLY_SI     = FID_OVER_LAST_YUV_ONLY_SI, 
    FID_NUM_YUV_ONLY_SI     = FID_END_YUV_ONLY_SI - FID_BEGIN_YUV_ONLY_SI, 

    //RAW feature id.
    FID_BEGIN_RAW_SI        = FID_BEGIN_RAW_ONLY_SI, 
    FID_END_RAW_SI          = FID_OVER_LAST_RAW_YUV_SI, 
    FID_NUM_RAW_SI          = FID_END_RAW_SI - FID_BEGIN_RAW_SI, 
    //YUV feature id.
    FID_BEGIN_YUV_SI        = FID_BEGIN_RAW_YUV_SI, 
    FID_END_YUV_SI          = FID_END_YUV_ONLY_SI, 
    FID_NUM_YUV_SI          = FID_END_YUV_SI - FID_BEGIN_YUV_SI, 

  //////////////////////////////////////////////////////////////////////////////
  // Scene-Dependent (SD) feature id.
    FID_BEGIN_SD            = FID_PRE_BEGIN_SD + 1, 
    FID_END_SD              = FID_OVER_LAST_SD, 

    //Misc. feature id.
    FID_BEGIN_MISC_SD       = FID_PRE_BEGIN_MISC_SD + 1, 
    FID_END_MISC_SD         = FID_OVER_LAST_MISC_SD, 
    FID_NUM_MISC_SD         = FID_END_MISC_SD - FID_BEGIN_MISC_SD, 

    //RAW-only feature id.
    FID_BEGIN_RAW_ONLY_SD   = FID_PRE_BEGIN_RAW_ONLY_SD + 1, 
    FID_END_RAW_ONLY_SD     = FID_OVER_LAST_RAW_ONLY_SD, 
    FID_NUM_RAW_ONLY_SD     = FID_END_RAW_ONLY_SD - FID_BEGIN_RAW_ONLY_SD, 
    //RAW-YUV-shared feature id.
    FID_BEGIN_RAW_YUV_SD    = FID_PRE_BEGIN_RAW_YUV_SD + 1, 
    FID_END_RAW_YUV_SD      = FID_OVER_LAST_RAW_YUV_SD, 
    FID_NUM_RAW_YUV_SD      = FID_END_RAW_YUV_SD - FID_BEGIN_RAW_YUV_SD, 
    //YUV-only feature id.
    FID_BEGIN_YUV_ONLY_SD   = FID_PRE_BEGIN_YUV_ONLY_SD + 1, 
    FID_END_YUV_ONLY_SD     = FID_OVER_LAST_YUV_ONLY_SD, 
    FID_NUM_YUV_ONLY_SD     = FID_END_YUV_ONLY_SD - FID_BEGIN_YUV_ONLY_SD, 

    //RAW feature id.
    FID_BEGIN_RAW_SD        = FID_BEGIN_RAW_ONLY_SD, 
    FID_END_RAW_SD          = FID_OVER_LAST_RAW_YUV_SD, 
    FID_NUM_RAW_SD          = FID_END_RAW_SD - FID_BEGIN_RAW_SD, 
    //YUV feature id.
    FID_BEGIN_YUV_SD        = FID_BEGIN_RAW_YUV_SD, 
    FID_END_YUV_SD          = FID_END_YUV_ONLY_SD, 
    FID_NUM_YUV_SD          = FID_END_YUV_SD - FID_BEGIN_YUV_SD, 

};


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//  Generic Feature ID Info
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
template <MUINT32 _fid>
struct FidInfo
{
    template <MUINT32 _val, MUINT32 _beg, MUINT32 _end>
    struct IsWithin
    {
        enum { Result = (_beg <= _val && _val < _end) };
    };

    enum
    {
      isSceneIndep      =
          IsWithin<_fid,FID_BEGIN_SI,FID_END_SI>::Result, 
      isMiscSceneIndep  =
          IsWithin<_fid,FID_BEGIN_MISC_SI,FID_END_MISC_SI>::Result, 
      isRAWSceneIndep   =
          IsWithin<_fid,FID_BEGIN_RAW_SI,FID_END_RAW_SI>::Result, 
      isYUVSceneIndep   =
          IsWithin<_fid,FID_BEGIN_YUV_SI,FID_END_YUV_SI>::Result, 

      isSceneDep        =
          IsWithin<_fid,FID_BEGIN_SD,FID_END_SD>::Result, 
      isMiscSceneDep    =
          IsWithin<_fid,FID_BEGIN_MISC_SD,FID_END_MISC_SD>::Result, 
      isRAWSceneDep     =
          IsWithin<_fid,FID_BEGIN_RAW_SD,FID_END_RAW_SD>::Result, 
      isYUVSceneDep     =
          IsWithin<_fid,FID_BEGIN_YUV_SD,FID_END_YUV_SD>::Result, 

      isRAW             = (isRAWSceneIndep || isRAWSceneDep), 
      isYUV             = (isYUVSceneIndep || isYUVSceneDep), 
    };
};


};  //  namespace NSFeature


#endif  //  _CAMERA_FEATURE_ID_H_

