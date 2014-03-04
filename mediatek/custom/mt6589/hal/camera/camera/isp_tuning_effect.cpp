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

//
#define LOG_TAG "IspTuningCustom_effect"
#ifndef ENABLE_MY_LOG
    #define ENABLE_MY_LOG       (0)
#endif
//
#include <utils/Errors.h>
#include <cutils/log.h>
//
#define USE_CUSTOM_ISP_TUNING
#include "isp_tuning.h"
//
//using namespace NS_MT6575ISP_EFFECT;
//
namespace NSIspTuning
{

#if 0
/*******************************************************************************
* Effect: MONO
*******************************************************************************/
template <>
MVOID
IspTuningCustom::
prepare_effect<MEFFECT_MONO>(ISP_EFFECT_T& rEffect)
{
    MY_LOG("[+prepare_effect] MEFFECT_MONO");
    //--------------------------------------------------------------------------
    //  Reference.
    ISP_EFFECT_YCCGO_T& rYCCGO = rEffect.yccgo;
    ISP_EFFECT_EDGE_T&  rEdge  = rEffect.edge;
    ISP_EFFECT_CCM_T&   rCCM   = rEffect.ccm;
    //--------------------------------------------------------------------------
    //  TODO:
    //  Begin your setting.
    //
    //  YCCGO
    rYCCGO.ctrl.val         = 0;
    rYCCGO.ctrl.bits.ENC2   = 1;
    rYCCGO.cfg1.bits.H12    = 0;
    rYCCGO.cfg1.bits.H11    = 0;

}


/*******************************************************************************
* Effect: SEPIA
*******************************************************************************/
template <>
MVOID
IspTuningCustom::
prepare_effect<MEFFECT_SEPIA>(ISP_EFFECT_T& rEffect)
{
    MY_LOG("[+prepare_effect] MEFFECT_SEPIA");
    //--------------------------------------------------------------------------
    //  Reference.
    ISP_EFFECT_YCCGO_T& rYCCGO = rEffect.yccgo;
    ISP_EFFECT_EDGE_T&  rEdge  = rEffect.edge;
    ISP_EFFECT_CCM_T&   rCCM   = rEffect.ccm;
    //--------------------------------------------------------------------------
    //  TODO:
    //  Begin your setting.
    //
    //  YCCGO
    rYCCGO.ctrl.val         = 0;
    rYCCGO.ctrl.bits.ENC1   = 1;
    rYCCGO.cfg1.bits.MU     = 98;
    rYCCGO.cfg1.bits.MV     = 158;

}


/*******************************************************************************
* Effect: AQUA
*******************************************************************************/
template <>
MVOID
IspTuningCustom::
prepare_effect<MEFFECT_AQUA>(ISP_EFFECT_T& rEffect)
{
    MY_LOG("[+prepare_effect] MEFFECT_AQUA");
    //--------------------------------------------------------------------------
    //  Reference.
    ISP_EFFECT_YCCGO_T& rYCCGO = rEffect.yccgo;
    ISP_EFFECT_EDGE_T&  rEdge  = rEffect.edge;
    ISP_EFFECT_CCM_T&   rCCM   = rEffect.ccm;
    //--------------------------------------------------------------------------
    //  TODO:
    //  Begin your setting.
    //
    //  YCCGO
    rYCCGO.ctrl.val         = 0;
    rYCCGO.ctrl.bits.ENC1   = 1;
    rYCCGO.cfg1.bits.MU     = 216;
    rYCCGO.cfg1.bits.MV     = 98;

}


/*******************************************************************************
* Effect: NEGATIVE
*******************************************************************************/
template <>
MVOID
IspTuningCustom::
prepare_effect<MEFFECT_NEGATIVE>(ISP_EFFECT_T& rEffect)
{
    MY_LOG("[+prepare_effect] MEFFECT_NEGATIVE");
    //--------------------------------------------------------------------------
    //  Reference.
    ISP_EFFECT_YCCGO_T& rYCCGO = rEffect.yccgo;
    ISP_EFFECT_EDGE_T&  rEdge  = rEffect.edge;
    ISP_EFFECT_CCM_T&   rCCM   = rEffect.ccm;
    //--------------------------------------------------------------------------
    //  TODO:
    //  Begin your setting.
    //
    //  EDGE
    rEdge.cpscon2.bits.OPRGM_IVT = 1;

}


/*******************************************************************************
* Effect: SOLARIZE
*******************************************************************************/
template <>
MVOID
IspTuningCustom::
prepare_effect<MEFFECT_SOLARIZE>(ISP_EFFECT_T& rEffect)
{
    MY_LOG("[+prepare_effect] MEFFECT_SOLARIZE");
    //--------------------------------------------------------------------------
    //  Reference.
    ISP_EFFECT_YCCGO_T& rYCCGO = rEffect.yccgo;
    ISP_EFFECT_EDGE_T&  rEdge  = rEffect.edge;
    ISP_EFFECT_CCM_T&   rCCM   = rEffect.ccm;
    //--------------------------------------------------------------------------
    //  TODO:
    //  Begin your setting.
    //
    //  YCCGO
    rYCCGO.ctrl.val     = 0x0000002b;
    rYCCGO.cfg1.val     = 0x80807f01;
    rYCCGO.cfg2.val     = 0x1020e0f0;
    rYCCGO.cfg3.val     = 0x20464846;
    rYCCGO.cfg4.val     = 0x00e00000;
    rYCCGO.cfg5.val     = 0x90000058;
    rYCCGO.cfg6.val     = 0xff00ff00;
    //
    //  EDGE
    rEdge.edgcore.val   = 0x001F1814;
    rEdge.edggain1.val  = 0x00000080;
    rEdge.edgvcon.val   = 0x231F0232;
    rEdge.ee_ctrl.val   = 0x005f1f1f;

}


/*******************************************************************************
* Effect: POSTERIZE
*******************************************************************************/
template <>
MVOID
IspTuningCustom::
prepare_effect<MEFFECT_POSTERIZE>(ISP_EFFECT_T& rEffect)
{
    MY_LOG("[+prepare_effect] MEFFECT_POSTERIZE");
    //--------------------------------------------------------------------------
    //  Reference.
    ISP_EFFECT_YCCGO_T& rYCCGO = rEffect.yccgo;
    ISP_EFFECT_EDGE_T&  rEdge  = rEffect.edge;
    ISP_EFFECT_CCM_T&   rCCM   = rEffect.ccm;
    //--------------------------------------------------------------------------
    //  TODO:
    //  Begin your setting.
    //
    //  TODO:
    //  Begin your setting.
    //
    //  YCCGO
    //  Set YCCGO Saturation Gain: (100,100,100,100,100)
    rYCCGO.ctrl.val         = 0;
    rYCCGO.ctrl.bits.ENC3   = 1;
    rYCCGO.cfg3.bits.G1     = 100;
    rYCCGO.cfg3.bits.G2     = 100;
    rYCCGO.cfg3.bits.G3     = 100;
    rYCCGO.cfg3.bits.G4     = 100;
    //
    //  EDGE
    rEdge.edgcore.bits.SPECIAL_EN   = 1;
    rEdge.edgcore.bits.EMBOSS2_EN   = 1;
    rEdge.edgcore.bits.EMBOSS1_EN   = 0;
    rEdge.edgcore.bits.COREH        = 0;

    rEdge.edggain1.bits.EGAINLINE   = 15;
    rEdge.edggain1.bits.KNEESEL     = 3;
    rEdge.edggain1.bits.OILEN       = 1;
    rEdge.edggain1.bits.EGAIN_VB    = 31;
    rEdge.edggain1.bits.EGAIN_H2    = 31;
    rEdge.edggain1.bits.EGAIN_H     = 15;
    rEdge.edggain1.bits.SPECIPONLY  = 0;
    rEdge.edggain1.bits.SPECIGAIN   = 1;

    rEdge.edgvcon.bits.E_TH1_V      = 35;

    rEdge.ee_ctrl.bits.RGBEDGE_EN   = 1;

}


/*******************************************************************************
* Effect: BLACKBOARD
*******************************************************************************/
template <>
MVOID
IspTuningCustom::
prepare_effect<MEFFECT_BLACKBOARD>(ISP_EFFECT_T& rEffect)
{
    MY_LOG("[+prepare_effect] MEFFECT_BLACKBOARD");
    //--------------------------------------------------------------------------
    //  Reference.
    ISP_EFFECT_YCCGO_T& rYCCGO = rEffect.yccgo;
    ISP_EFFECT_EDGE_T&  rEdge  = rEffect.edge;
    ISP_EFFECT_CCM_T&   rCCM   = rEffect.ccm;
    //--------------------------------------------------------------------------
    //  TODO:
    //  Begin your setting.
    //
    //  TODO:
    //  Begin your setting.
    //
    //  CCM
    rCCM.ccm1.val = 0x00000000;
    rCCM.ccm2.val = 0x00000000;
    rCCM.ccm3.val = 0x00000000;
    rCCM.ccm4.val = 0x00000000;
    rCCM.ccm5.val = 0x00000000;

    //
    //  YCCGO
    rYCCGO.ctrl.val         = 0;
    rYCCGO.ctrl.bits.ENC2   = 1;
    rYCCGO.cfg1.bits.H12    = 0;
    rYCCGO.cfg1.bits.H11    = 0;
    //
    //  EDGE
    rEdge.ed_ctrl.val               = 0x00000122;
    rEdge.ed_inter1.val             = 0x08000810;
    rEdge.ed_inter2.val             = 0x00000414;
    rEdge.ed_inter2.bits.THRE_LEDGE = 127;

    rEdge.edgcore.bits.SPECIAL_EN   = 1;
    rEdge.edgcore.bits.EMBOSS2_EN   = 1;
    rEdge.edgcore.bits.EMBOSS1_EN   = 1;    //
    rEdge.edgcore.bits.COREH        = 0;

    rEdge.edggain1.bits.EGAINLINE   = 0;    //
    rEdge.edggain1.bits.KNEESEL     = 3;
    rEdge.edggain1.bits.OILEN       = 0;    //
    rEdge.edggain1.bits.EGAIN_VB    = 31;
    rEdge.edggain1.bits.EGAIN_H2    = 31;
    rEdge.edggain1.bits.EGAIN_H     = 0;    //
    rEdge.edggain1.bits.SPECIPONLY  = 1;    //
    rEdge.edggain1.bits.SPECIGAIN   = 0;    //

    rEdge.edggain2.bits.SPECIINV    = 0;    //
    rEdge.edggain2.bits.SPECIABS    = 0;    //

    rEdge.edgvcon.bits.E_TH1_V      = 4;    //

    rEdge.cpscon2.bits.Y_EGAIN      = 15;   //
    rEdge.cpscon2.bits.OPRGM_IVT    = 0;    //  //

    rEdge.ee_ctrl.bits.YEDGE_EN     = 1;    //
    rEdge.ee_ctrl.bits.RGBEDGE_EN   = 1;

}


/*******************************************************************************
* Effect: WHITEBOARD
*******************************************************************************/
template <>
MVOID
IspTuningCustom::
prepare_effect<MEFFECT_WHITEBOARD>(ISP_EFFECT_T& rEffect)
{
    MY_LOG("[+prepare_effect] MEFFECT_WHITEBOARD");
    //--------------------------------------------------------------------------
    //  Reference.
    ISP_EFFECT_YCCGO_T& rYCCGO = rEffect.yccgo;
    ISP_EFFECT_EDGE_T&  rEdge  = rEffect.edge;
    ISP_EFFECT_CCM_T&   rCCM   = rEffect.ccm;
    //--------------------------------------------------------------------------
    //  TODO:
    //  Begin your setting.
    //
    //  TODO:
    //  Begin your setting.
    //
    //  CCM
    rCCM.ccm1.val = 0x00000000;
    rCCM.ccm2.val = 0x00000000;
    rCCM.ccm3.val = 0x00000000;
    rCCM.ccm4.val = 0x00000000;
    rCCM.ccm5.val = 0x00000000;

    //
    //  YCCGO
    rYCCGO.ctrl.val         = 0;
    rYCCGO.ctrl.bits.ENC2   = 1;
    rYCCGO.cfg1.bits.H12    = 0;
    rYCCGO.cfg1.bits.H11    = 0;
    //
    //  EDGE
    rEdge.ed_ctrl.val               = 0x00000122;
    rEdge.ed_inter1.val             = 0x08000810;
    rEdge.ed_inter2.val             = 0x00000414;
    rEdge.ed_inter2.bits.THRE_LEDGE = 127;

    rEdge.edgcore.bits.SPECIAL_EN   = 1;
    rEdge.edgcore.bits.EMBOSS2_EN   = 1;
    rEdge.edgcore.bits.EMBOSS1_EN   = 1;    //
    rEdge.edgcore.bits.COREH        = 0;

    rEdge.edggain1.bits.EGAINLINE   = 0;    //
    rEdge.edggain1.bits.KNEESEL     = 3;
    rEdge.edggain1.bits.OILEN       = 0;    //
    rEdge.edggain1.bits.EGAIN_VB    = 31;
    rEdge.edggain1.bits.EGAIN_H2    = 31;
    rEdge.edggain1.bits.EGAIN_H     = 0;    //
    rEdge.edggain1.bits.SPECIPONLY  = 1;    //
    rEdge.edggain1.bits.SPECIGAIN   = 0;    //

    rEdge.edggain2.bits.SPECIINV    = 0;    //
    rEdge.edggain2.bits.SPECIABS    = 0;    //

    rEdge.edgvcon.bits.E_TH1_V      = 4;    //

    rEdge.cpscon2.bits.Y_EGAIN      = 15;   //
    rEdge.cpscon2.bits.OPRGM_IVT    = 1;    //  //

    rEdge.ee_ctrl.bits.YEDGE_EN     = 1;    //
    rEdge.ee_ctrl.bits.RGBEDGE_EN   = 1;

}
#endif

/*******************************************************************************
*
*******************************************************************************/
};  //NSIspTuning


