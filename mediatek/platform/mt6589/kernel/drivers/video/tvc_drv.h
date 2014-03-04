/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/

#ifndef __TVC_DRV_H__
#define __TVC_DRV_H__

#include "disp_drv.h"
#include "tv_def.h"


#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------------

#define TVC_CHECK_RET(expr)             \
    do {                                \
        TVC_STATUS ret = (expr);        \
        ASSERT(TVC_STATUS_OK == ret);   \
    } while (0)

// ---------------------------------------------------------------------------

typedef enum
{
   TVC_STATUS_OK = 0,

   TVC_STATUS_ERROR,
} TVC_STATUS;


typedef enum
{
    TVC_RGB565     = 0,
    TVC_YUV420_PLANAR = 1,
    TVC_UYVY422 = 2,
    TVC_YUV420_BLK = 3,
} TVC_SRC_FORMAT;


typedef enum
{
    TVC_NTSC  = 0, // 525 lines
    TVC_PAL_M = 1, // 525 lines
    TVC_PAL_C = 2, // 625 lines
    TVC_PAL   = 3, // 625 lines
} TVC_TV_TYPE;


// ---------------------------------------------------------------------------

TVC_STATUS TVC_Init(void);
TVC_STATUS TVC_Deinit(void);

TVC_STATUS TVC_PowerOn(void);
TVC_STATUS TVC_PowerOff(void);

TVC_STATUS TVC_Enable(void);
TVC_STATUS TVC_Disable(void);

TVC_STATUS TVC_SetTvType(TVC_TV_TYPE type);

TVC_STATUS TVC_SetSrcFormat(TVC_SRC_FORMAT format);
TVC_STATUS TVC_SetSrcRGBAddr(UINT32 address);
TVC_STATUS TVC_SetSrcYUVAddr(UINT32 Y, UINT32 U, UINT32 V);
TVC_STATUS TVC_SetSrcSize(UINT32 width, UINT32 height);
TVC_STATUS TVC_SetTarSize(UINT32 width, UINT32 height);
TVC_STATUS TVC_SetTarSizeForHQA(BOOL enable);

TVC_STATUS TVC_CommitChanges(BOOL blocking);
UINT32     TVC_GetWorkingAddr(void);
TVC_STATUS TVC_AllocMva(unsigned int va, unsigned int size, unsigned int* mva);
TVC_STATUS TVC_DeallocMva(unsigned int va, unsigned int size, unsigned int mva);
TVC_STATUS TVC_CheckFormat(TVC_SRC_FORMAT format);

// Debug
TVC_STATUS TVC_DumpRegisters(void);

void TVC_ConfigSize(unsigned int src_width, unsigned int src_height, unsigned int tar_width, unsigned int tar_height);
void TVC_SetCheckLineOffset(UINT32 offset);


// ---------------------------------------------------------------------------

#ifdef __cplusplus
}
#endif

#endif // __TVC_DRV_H__
