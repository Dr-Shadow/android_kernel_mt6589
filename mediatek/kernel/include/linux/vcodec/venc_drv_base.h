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

#include "venc_drv_if.h"
#include "vcodec_if.h"

#include <sys/time.h>

#ifndef _VENC_DRV_BASE_
#define _VENC_DRV_BASE_

#define DO_VCODEC_RESET(cmd, index)                                                             \
{                                                                                               \
}

typedef enum __VDDRV_MRESULT_T
{
    VDDRV_MRESULT_SUCCESS = VAL_TRUE,  ///< Represent success
    VDDRV_MRESULT_FAIL = VAL_FALSE     ///< Represent failure
} VDDRV_MRESULT_T;

typedef struct __VENC_DRV_BASE_T
{
    VAL_UINT32_T    (*Init)(VAL_HANDLE_T *handle, VAL_HANDLE_T halhandle, VAL_HANDLE_T valhandle);                   ///< Function to do driver Initialization
    VAL_UINT32_T    (*Encode)(VAL_HANDLE_T handle, VENC_DRV_START_OPT_T eOpt, P_VENC_DRV_PARAM_FRM_BUF_T pFrameBuf, P_VENC_DRV_PARAM_BS_BUF_T pBitstreamBuf, VENC_DRV_DONE_RESULT_T *pResult);
    VAL_UINT32_T    (*GetParam)(VAL_HANDLE_T handle, VENC_DRV_GET_TYPE_T a_eType, VAL_VOID_T *a_pvInParam, VAL_VOID_T *a_pvOutParam);  ///< get codec's required memory size. 
    VAL_UINT32_T    (*SetParam)(VAL_HANDLE_T handle, VENC_DRV_SET_TYPE_T a_eType, VAL_VOID_T *a_pvInParam, VAL_VOID_T *a_pvOutParam);  ///< set codec's required memory size. 
    VAL_UINT32_T    (*DeInit)(VAL_HANDLE_T handle);                                                                                 ///< Function to do driver de-initialization
} VENC_DRV_BASE_T;

/**
 * @par Structure
 *   mhalVdoDrv_t
 * @par Description
 *   This is a structure which store common video enc driver information 
 */
typedef struct mhalVdoDrv_s 
{
    VAL_VOID_T                      *prCodecHandle;
    VAL_UINT32_T                    u4EncodedFrameCount;
    VCODEC_ENC_CALLBACK_T           rCodecCb;
    VIDEO_ENC_API_T                 *prCodecAPI;
    VENC_BS_T                       pBSBUF;

    VCODEC_ENC_BUFFER_INFO_T        EncoderInputParamNC;
    VENC_DRV_PARAM_BS_BUF_T         BSout;  
    VENC_HYBRID_ENCSETTING          rVencSetting;
    VAL_UINT8_T                     *ptr;	
} mhalVdoDrv_t;


typedef struct __VENC_HYBRID_HANDLE_T 
{
    mhalVdoDrv_t                    rMhalVdoDrv;                
    VAL_MEMORY_T                    rBSDrvWorkingMem;
    VAL_UINT32_T                    nOmxTids;
    VIDEO_ENC_WRAP_HANDLE_T         hWrapper;
    VAL_VOID_T                      *pDrvModule;    ///< used for dlopen and dlclose
} VENC_HYBRID_HANDLE_T;


typedef struct __VENC_HANDLE_T
{
    VENC_DRV_VIDEO_FORMAT_T CodecFormat;
    VENC_DRV_BASE_T         rFuncPtr;      ///< Point to driver's proprietary function.
    VAL_HANDLE_T            hDrvHandle;    ///< Handle of each format driver
    VAL_HANDLE_T            hHalHandle;    ///< HAL handle
    VAL_HANDLE_T            hValHandle;    ///< VAL handle    
    VAL_MEMORY_T            rHandleMem;    ///< Memory for venc handle
    VAL_VOID_T              *prExtraData;  ///< Driver private data pointer.
    VAL_MEMORY_T            rExtraDataMem; ///< Save extra data memory information to be used in release.
    VENC_HYBRID_HANDLE_T    rHybridHandle; ///< Hybrid handle
    FILE                    *pfDump;       ///< Dump file
    VAL_UINT32_T            u4ShowInfo;    ///< Flag for show FPS and BitRate
    VAL_UINT32_T            u4FPS;         ///< FPS
    VAL_UINT32_T            u4Bitrate;     ///< Bitrate
    struct timeval          tStart;        ///< Start time counting FPS and bitrate
} VENC_HANDLE_T;

VENC_DRV_MRESULT_T ParseConfig(const char* cfgFileName, const char* ParameterItem, VAL_UINT32_T *val);


#endif
