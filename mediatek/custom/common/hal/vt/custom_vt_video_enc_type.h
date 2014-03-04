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

#ifndef CUSTOM_VT_VIDEO_ENC_TYPE_H
#define CUSTOM_VT_VIDEO_ENC_TYPE_H

#ifdef __cplusplus
extern "C"
    {
#endif


/*****************************************************************************
*
*****************************************************************************/
typedef enum
{
	CUSTOM_VT_SENSOR_AUTO_MODE = 0,
	CUSTOM_VT_SENSOR_NIGHT_MODE,
	CUSTOM_VT_SENSOR_UNKNOWN_MODE = 0x1FFFFFFF
}CUSTOM_VT_SENSOR_MODE_T;



/*****************************************************************************
*
*****************************************************************************/
typedef enum
{ 
	CUSTOM_VENC_RESOLUTION_176x144 = 0,             // QCIF
    CUSTOM_VENC_RESOLUTION_128x96,          // SQCIF
    CUSTOM_VENC_RESOLUTION_UNKNOWN = 0x1FFFFFFF // Unknown
} CUSTOM_VT_VENC_RESOLUTION_T;


/*****************************************************************************
*
*****************************************************************************/
typedef enum
{
    CUSTOM_VENC_QUALITY_GOOD = 0,
	CUSTOM_VENC_QUALITY_NORMAL,
    CUSTOM_VENC_QUALITY_LOW,   //fluency

    CUSTOM_VENC_QUALITY_FINE,
    CUSTOM_VENC_QUALITY_ALL,
    CUSTOM_VENC_QUALITY_UNKNOWN = 0x1FFFFFFF        // Keep it at end.
} CUSTOM_VT_VENC_QUALITY_T;



/*****************************************************************************
*
*****************************************************************************/
typedef enum
{
   	CUSTOM_VENC_CODEC_H263 = 0,
   	CUSTOM_VENC_CODEC_MPEG4,
    CUSTOM_VENC_CODEC_H264,
     CUSTOM_VENC_CODEC_ALL,
    CUSTOM_VENC_CODEC_UNKNOWN = 0x1FFFFFFF          // Keep it at end.
} CUSTOM_VT_VENC_CODEC_T;


/******************************************************************************
* This data structure is used by MMI.
******************************************************************************/
	


typedef struct
{
    CUSTOM_VT_VENC_RESOLUTION_T eResolution;
	CUSTOM_VT_SENSOR_MODE_T eSensorMode;
    CUSTOM_VT_VENC_QUALITY_T eQuality;
    CUSTOM_VT_VENC_CODEC_T eCodec;
} CUSTOM_VT_VENC_SCENARIO_TABLE_T;


/*****************************************************************************
*
*****************************************************************************/
	
typedef struct
{
	int iFrameRate;
	int iInitQp;
	int iMinQp;
	int iMaxQp;
	int iIntraVopRate;
	int iAlgorithmSelect;
	int iBitRate;
	int iDynamicRangeReduction;
	int iIsCustomerSetTable;
	signed short  *pDynamicRangeTable;

}CUSTOM_VT_VENC_PARAM_TABLE_T;


/*****************************************************************************
*
*****************************************************************************/
typedef struct
{
    CUSTOM_VT_VENC_SCENARIO_TABLE_T tVT_Scenario_T;
	CUSTOM_VT_VENC_PARAM_TABLE_T tVT_VEnc_Param_T;
  
} CUSTOM_VT_VENC_QUALITY_CTRL_TABLE_T;


/*****************************************************************************
*
*****************************************************************************/
typedef struct
{	
	float iGoodQualityFrameRate; //default 7.5fps
	float iNormalQualityFrameRate; // default 10fps
	float iLowQualityFrameRate;  // default 12fps
}CUSTOM_VT_VENC_QUALITY_LEVEL;



/******************************************************************************
* The functions to get the customer setting 
******************************************************************************/

bool _Custom_VT_GetFrameRates(int iCodec,int iSensorMode,void* pFrameRates);

bool _Custom_VT_GetCodecParam(int iCodec,int iSensorMode,int iEncLevel,CUSTOM_VT_VENC_PARAM_TABLE_T* pCodecParam);

signed short* _Custom_VT_GetDynamicRangeTable();

#ifdef __cplusplus
    }
#endif

#endif  //CUSTOM_VT_VIDEO_ENC_TYPE_H

