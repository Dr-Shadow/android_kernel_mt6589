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


#ifndef _BITS_API_H_
#define _BITS_API_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "val_types.h"
#include "hal_api.h"

typedef VAL_UINT32_T (*fgPrepare32FN)(VAL_HANDLE_T *a_phBitsHandle);
typedef struct __VBITS_HANDLE_T
{
    VAL_HANDLE_T    hHALHandle;
    VAL_HANDLE_T    hVALHandle;
    VAL_MEM_ADDR_T  BitsStart;
    VAL_MEMORY_T    rHandleMem;
    VAL_UINT32_T    nReadingMode; // 0 for software, 1 for mmap, 2 for hardware
    VAL_UINT32_T    StartAddr; // used for software mode fast access
    VAL_UINT32_T    nSize;
    VAL_UINT32_T    nBitCnt;
    VAL_UINT32_T    nZeroCnt;
    VAL_UINT32_T    Cur32Bits;
    VAL_UINT32_T    CurBitCnt;
    VAL_UINT32_T    n03RemoveCount,n03CountBit;
    VAL_INT32_T     n03FirstIndex, n03SecondIndex;
    VAL_UINT32_T    n03RemoveIgnore;
    VAL_BOOL_T      bFirstCheck, bEverRemove, bIgnoreByBS;
    VAL_BOOL_T      bEOF;
    fgPrepare32FN   Prepare32Bits;
    VAL_DRIVER_TYPE_T vFormat;
    VAL_UINT32_T    value;
} VBITS_HANDLE_T;

typedef enum VBITS_READTYPE_T
{
    VBITS_SOFTWARE = 0,
    VBITS_MMAP,
    VBITS_HARDWARE,
    VBITS_MAX
} VBITS_READTYPE_T;
/*=============================================================================
 *                             Function Declaration
 *===========================================================================*/
VAL_UINT32_T eBufEnable(VAL_HANDLE_T *a_phBitsHandle,VAL_HANDLE_T hHALHandle,VAL_UINT32_T nMode, VAL_DRIVER_TYPE_T vFormat);
VAL_UINT32_T eBufDisable(VAL_HANDLE_T *a_phBitsHandle,VAL_HANDLE_T hHALHandle,VAL_UINT32_T nMode, VAL_DRIVER_TYPE_T vFormat);
VAL_RESULT_T eBufInit(VAL_HANDLE_T *a_phBitsHandle, VAL_HANDLE_T hVALHandle, VAL_HANDLE_T hHALHandle, VAL_MEM_ADDR_T rBufAddrStart, VAL_UINT32_T nMode, VAL_DRIVER_TYPE_T vFormat);
VAL_RESULT_T eBufDeinit(VAL_HANDLE_T *a_phBitsHandle);
VAL_UINT32_T eBufGetBitCnt(VAL_HANDLE_T *a_phBitsHandle);
VAL_UINT32_T eBufGetBits(VAL_HANDLE_T *a_phBitsHandle, VAL_UINT32_T numBits);
VAL_UINT32_T eBufNextBits(VAL_HANDLE_T *a_phBitsHandle, VAL_UINT32_T numBits);
VAL_UINT32_T eBufGetUEGolomb(VAL_HANDLE_T *a_phBitsHandle);
VAL_INT32_T  eBufGetSEGolomb(VAL_HANDLE_T *a_phBitsHandle);
VAL_BOOL_T   eBufCheckEOF(VAL_HANDLE_T *a_phBitsHandle);
VAL_UINT32_T eBufGetBufSize(VAL_HANDLE_T *a_phBitsHandle);
void NextBytesAlignment(VAL_HANDLE_T *a_phBitsHandle, VAL_UINT32_T nBytesAlignment);
VAL_BOOL_T   eBufInitBS(VAL_HANDLE_T *a_phBitsHandle, P_VCODEC_DRV_CMD_T cmd_queue, VAL_UINT32_T *pIndex);
VAL_UINT32_T eBufGetPAddr(VAL_HANDLE_T *a_phBitsHandle);
VAL_BOOL_T eBufReInite(VAL_HANDLE_T *a_phBitsHandle, VAL_UINT32_T nBytes, VAL_UINT32_T nBits);

#ifdef __cplusplus
}
#endif

#endif // #ifndef _VAL_API_H_
