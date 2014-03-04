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

#ifndef _HAL_API_H_
#define _HAL_API_H_
#include "val_types.h"

#define DumpReg__
#ifdef DumpReg__
#include <stdio.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define ADD_QUEUE(queue, index, q_type, q_address, q_offset, q_value, q_mask)       \
{                                                                                   \
  queue[index].type     = q_type;                                                   \
  queue[index].address  = q_address;                                                \
  queue[index].offset   = q_offset;                                                 \
  queue[index].value    = q_value;                                                  \
  queue[index].mask     = q_mask;                                                   \
  index = index + 1;                                                                \
}

typedef enum __HAL_CODEC_TYPE_T
{
    HAL_CODEC_TYPE_VDEC,
    HAL_CODEC_TYPE_VENC,
    HAL_CODEC_TYPE_MAX = 0xFFFFFFFF
} HAL_CODEC_TYPE_T;

typedef enum
{
    HAL_CMD_SET_CMD_QUEUE,
    HAL_CMD_SET_POWER,
    HAL_CMD_SET_ISR,
    HAL_CMD_GET_CACHE_CTRL_ADDR,
    HAL_CMD_MAX = 0xFFFFFFFF
} HAL_CMD_T;

typedef enum
{
    VDEC_SYS,
    VDEC_MISC,
    VDEC_VLD,
    VDEC_VLD_TOP,
    VDEC_MC,
    VDEC_AVC_VLD,
    VDEC_AVC_MV,
    VDEC_PP,
//    VDEC_SQT,
    VDEC_VP8_VLD,
    VDEC_VP6_VLD,
    VDEC_VP8_VLD2,
    VENC_HW_BASE,
    VENC_MP4_HW_BASE,
    VCODEC_MAX
} REGISTER_GROUP_T;

typedef enum
{
    ENABLE_HW_CMD,
    DISABLE_HW_CMD,
    WRITE_REG_CMD,
    READ_REG_CMD,
    WRITE_SYSRAM_CMD,
    READ_SYSRAM_CMD,
    MASTER_WRITE_CMD,
    WRITE_SYSRAM_RANGE_CMD,
    READ_SYSRAM_RANGE_CMD,
    SETUP_ISR_CMD,
    WAIT_ISR_CMD,
    TIMEOUT_CMD,
    MB_CMD,
    POLL_REG_STATUS_CMD,
    END_CMD
} VCODEC_DRV_CMD_TYPE;

typedef struct __VCODEC_DRV_CMD_T *P_VCODEC_DRV_CMD_T;
typedef struct __VCODEC_DRV_CMD_T
{
    VAL_UINT32_T type;
    VAL_UINT32_T address;
    VAL_UINT32_T offset;
    VAL_UINT32_T value;
    VAL_UINT32_T mask;
} VCODEC_DRV_CMD_T;

typedef struct _HAL_HANDLE_T_
{
    VAL_INT32_T     fd_vdec;
    VAL_INT32_T     fd_venc;
    VAL_MEMORY_T    rHandleMem;
    VAL_UINT32_T    mmap[VCODEC_MAX];
    VAL_DRIVER_TYPE_T    driverType;
    VAL_UINT32_T    u4TimeOut;
    VAL_UINT32_T    u4FrameCount;
#ifdef DumpReg__
    FILE *pf_out;
#endif    
} HAL_HANDLE_T;


VAL_RESULT_T eHalInit(VAL_HANDLE_T *a_phHalHandle, HAL_CODEC_TYPE_T a_eHalCodecType);
VAL_RESULT_T eHalDeInit(VAL_HANDLE_T *a_phHalHandle);
VAL_UINT32_T eHalGetMMAP(VAL_HANDLE_T *a_hHalHandle, VAL_UINT32_T RegAddr);
VAL_RESULT_T eHalCmdProc(VAL_HANDLE_T *a_hHalHandle, HAL_CMD_T a_eHalCmd, VAL_VOID_T *a_pvInParam, VAL_VOID_T *a_pvOutParam);



#ifdef __cplusplus
}
#endif

#endif // #ifndef _HAL_API_H_
