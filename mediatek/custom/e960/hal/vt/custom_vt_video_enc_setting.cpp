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

#include "custom_vt_video_enc_type.h"


/******************************************************************************
* The variables to store the customer setting 
******************************************************************************/

static CUSTOM_VT_VENC_QUALITY_CTRL_TABLE_T _rVtVEncQualityCtrlTable[] =
{

	/*********************For sensor is in Auto Mode************************/
	{
        /* Upper table begins. */
        {CUSTOM_VENC_RESOLUTION_176x144, CUSTOM_VT_SENSOR_AUTO_MODE,
        CUSTOM_VENC_QUALITY_LOW,CUSTOM_VENC_CODEC_MPEG4},
        /* Lower table begins. */
        {120, 18, 1, 31, 40, 3, 48000, 1, 0,0}
    },


	{
        /* Upper table begins. */
        {CUSTOM_VENC_RESOLUTION_176x144, CUSTOM_VT_SENSOR_AUTO_MODE,
        CUSTOM_VENC_QUALITY_NORMAL,CUSTOM_VENC_CODEC_MPEG4},
        /* Lower table begins. */
        {100, 18, 1, 31, 40, 3, 48000, 1, 0,0}
    },


	{
        /* Upper table begins. */
        {CUSTOM_VENC_RESOLUTION_176x144, CUSTOM_VT_SENSOR_AUTO_MODE,
        CUSTOM_VENC_QUALITY_GOOD,CUSTOM_VENC_CODEC_MPEG4},
        /* Lower table begins. */
        {75, 18, 1, 31, 40, 3, 48000, 1, 0,0}
    },

	
    {
        /* Upper table begins. */
        {CUSTOM_VENC_RESOLUTION_176x144, CUSTOM_VT_SENSOR_AUTO_MODE,
        CUSTOM_VENC_QUALITY_LOW,CUSTOM_VENC_CODEC_H263},
        /* Lower table begins. */
        {120, 18, 1, 31, 40, 3, 48000, 1, 0,0}
    },


	{
        /* Upper table begins. */
        {CUSTOM_VENC_RESOLUTION_176x144, CUSTOM_VT_SENSOR_AUTO_MODE,
        CUSTOM_VENC_QUALITY_NORMAL,CUSTOM_VENC_CODEC_H263},
        /* Lower table begins. */
        {100, 18, 1, 31, 40, 3, 48000, 1, 0,0}
    },


	{
        /* Upper table begins. */
        {CUSTOM_VENC_RESOLUTION_176x144, CUSTOM_VT_SENSOR_AUTO_MODE,
        CUSTOM_VENC_QUALITY_GOOD,CUSTOM_VENC_CODEC_H263},
        /* Lower table begins. */
        {75, 18, 1, 31, 40, 3, 48000, 1, 0,0}
    },
    
	
	/*********************For sensor is in Night Mode************************/

	{
        /* Upper table begins. */
        {CUSTOM_VENC_RESOLUTION_176x144, CUSTOM_VT_SENSOR_NIGHT_MODE,
        CUSTOM_VENC_QUALITY_LOW,CUSTOM_VENC_CODEC_MPEG4},
        /* Lower table begins. */
        {120, 18, 1, 31, 40, 3, 48000, 1, 0,0}
    },


	{
        /* Upper table begins. */
        {CUSTOM_VENC_RESOLUTION_176x144, CUSTOM_VT_SENSOR_NIGHT_MODE,
        CUSTOM_VENC_QUALITY_NORMAL,CUSTOM_VENC_CODEC_MPEG4},
        /* Lower table begins. */
        {100, 18, 1, 31, 40, 3, 48000, 1, 0,0}
    },


	{
        /* Upper table begins. */
        {CUSTOM_VENC_RESOLUTION_176x144, CUSTOM_VT_SENSOR_NIGHT_MODE,
        CUSTOM_VENC_QUALITY_GOOD,CUSTOM_VENC_CODEC_MPEG4},
        /* Lower table begins. */
        {75, 18, 1, 31, 40, 3, 48000, 1, 0,0}
    },

	
    {
        /* Upper table begins. */
        {CUSTOM_VENC_RESOLUTION_176x144, CUSTOM_VT_SENSOR_NIGHT_MODE,
        CUSTOM_VENC_QUALITY_LOW,CUSTOM_VENC_CODEC_H263},
        /* Lower table begins. */
        {120, 18, 1, 31, 40, 3, 48000, 1, 0,0}
    },


	{
        /* Upper table begins. */
        {CUSTOM_VENC_RESOLUTION_176x144, CUSTOM_VT_SENSOR_NIGHT_MODE,
        CUSTOM_VENC_QUALITY_NORMAL,CUSTOM_VENC_CODEC_H263},
        /* Lower table begins. */
        {100, 18, 1, 31, 40, 3, 48000, 1, 0,0}
    },


	{
        /* Upper table begins. */
        {CUSTOM_VENC_RESOLUTION_176x144, CUSTOM_VT_SENSOR_NIGHT_MODE,
        CUSTOM_VENC_QUALITY_GOOD,CUSTOM_VENC_CODEC_H263},
        /* Lower table begins. */
        {75, 18, 1, 31, 40, 3, 48000, 1, 0,0}
    },
    
};

/******************************************************************************************
* If want to use special dynamic range table, set the table here, a array with 511 elements
*******************************************************************************************/
static signed short variation_vt[]={0};  


/******************************************************************************
* The functions to get the customer setting 
******************************************************************************/

bool _Custom_VT_GetFrameRates(int iCodec,int iSensorMode,void* pFrameRates)
{
	CUSTOM_VT_VENC_QUALITY_LEVEL *  pTempFrameR = (CUSTOM_VT_VENC_QUALITY_LEVEL * )pFrameRates;
	//...
	if(!pTempFrameR)
	{
		return false;
	}
	int iEntry = sizeof(_rVtVEncQualityCtrlTable)/sizeof(_rVtVEncQualityCtrlTable[0]);
	//LOGI("[VTMAL][CustomSetting]iEntry = %d",iEntry);
	for(int i = 0; i < iEntry; i++)
	{
		if((_rVtVEncQualityCtrlTable[i].tVT_Scenario_T.eCodec == iCodec) && (_rVtVEncQualityCtrlTable[i].tVT_Scenario_T.eSensorMode == iSensorMode))
		{
			if(_rVtVEncQualityCtrlTable[i].tVT_Scenario_T.eQuality == CUSTOM_VENC_QUALITY_GOOD)
			{
				pTempFrameR->iGoodQualityFrameRate = (_rVtVEncQualityCtrlTable[i].tVT_VEnc_Param_T.iFrameRate)/10.0;
			}
			else if(_rVtVEncQualityCtrlTable[i].tVT_Scenario_T.eQuality == CUSTOM_VENC_QUALITY_NORMAL)
			{
				pTempFrameR->iNormalQualityFrameRate = (_rVtVEncQualityCtrlTable[i].tVT_VEnc_Param_T.iFrameRate)/10.0;
			}
			else if(_rVtVEncQualityCtrlTable[i].tVT_Scenario_T.eQuality == CUSTOM_VENC_QUALITY_LOW)
			{
				pTempFrameR->iLowQualityFrameRate = (_rVtVEncQualityCtrlTable[i].tVT_VEnc_Param_T.iFrameRate)/10.0;
			}
		}
	}
	
	return true;
}

signed short * _Custom_VT_GetDynamicRangeTable()
{
	return variation_vt;
}

bool _Custom_VT_GetCodecParam(int iCodec,int iSensorMode,int iEncLevel,CUSTOM_VT_VENC_PARAM_TABLE_T* pCodecParam)
{
	
	if(!pCodecParam)
	{
		return false;
	}
	int iEntry = sizeof(_rVtVEncQualityCtrlTable)/sizeof(_rVtVEncQualityCtrlTable[0]);
	for(int i = 0; i < iEntry; i++)
	{
		if((_rVtVEncQualityCtrlTable[i].tVT_Scenario_T.eCodec == iCodec) && (_rVtVEncQualityCtrlTable[i].tVT_Scenario_T.eSensorMode == iSensorMode) && (_rVtVEncQualityCtrlTable[i].tVT_Scenario_T.eQuality == iEncLevel))
		{
			

			if(_rVtVEncQualityCtrlTable[i].tVT_VEnc_Param_T.iIsCustomerSetTable == 1)
			{
				_rVtVEncQualityCtrlTable[i].tVT_VEnc_Param_T.pDynamicRangeTable = variation_vt;//_Custom_VT_GetDynamicRangeTable();
			}
			else
			{
				_rVtVEncQualityCtrlTable[i].tVT_VEnc_Param_T.pDynamicRangeTable = 0;
			}

			*pCodecParam = _rVtVEncQualityCtrlTable[i].tVT_VEnc_Param_T;
			break;
		}
	}

	return true;

}




