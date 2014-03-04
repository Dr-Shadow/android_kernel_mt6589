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

#ifndef __TVROT_DRV_H__
#define __TVROT_DRV_H__

//#include "disp_drv.h"
#include "tv_def.h"

//#define MTK_TVROT_LDVT



#ifdef __cplusplus
extern "C" {
#endif



// ---------------------------------------------------------------------------

#define TVR_CHECK_RET(expr)             \
    do {                                \
        TVR_STATUS ret = (expr);        \
        ASSERT(TVR_STATUS_OK == ret);   \
    } while (0)

// ---------------------------------------------------------------------------

typedef enum
{
   TVR_STATUS_OK = 0,

   TVR_STATUS_ERROR,
   TVR_STATUS_INSUFFICIENT_SRAM,
} TVR_STATUS;


typedef enum
{
    TVR_RGB565  = 2,
    TVR_YUYV422 = 4,
} TVR_FORMAT;


typedef enum
{
    TVR_ROT_0   = 0,
    TVR_ROT_90  = 1,
    TVR_ROT_180 = 2,
    TVR_ROT_270 = 3,
} TVR_ROT;

typedef struct
{
    unsigned int x;
    unsigned int y;
    unsigned int w;
    unsigned int h;
} TVR_SRC_ROI;


// ---------------------------------------------------------------------------


#if defined CONFIG_MTK_LDVT
#define TVR_BUFFERS (16)

typedef struct
{
    unsigned int srcWidth;
    unsigned int srcHeight;
    TVR_SRC_ROI  srcRoi;

    unsigned int dstWidth;
    TVR_FORMAT outputFormat;
    unsigned int dstBufNum;
    unsigned int dstBufAddr[TVR_BUFFERS];

    TVR_ROT rotation;
    int    bAuto;
    int    flip;

} TVR_PARAM;


#else
#define TVR_BUFFERS (2)

typedef struct
{
    UINT32      srcWidth;
    UINT32      srcHeight;

    TVR_FORMAT  outputFormat;
    UINT32      dstBufAddr[TVR_BUFFERS];
    TVR_ROT     rotation;
    BOOL        flip;
#if defined TV_BUFFER_PIPE
    UINT32 dstBufOffset;
#endif

} TVR_PARAM;

#endif

TVR_STATUS TVR_Init(void);
TVR_STATUS TVR_Deinit(void);

TVR_STATUS TVR_PowerOn(void);
TVR_STATUS TVR_PowerOff(void);

TVR_STATUS TVR_Config(const TVR_PARAM *param);
TVR_STATUS TVR_Start(void);
TVR_STATUS TVR_Stop(void);
TVR_STATUS TVR_Wait_Done(void);
TVR_STATUS TVR_AllocMva(unsigned int va, unsigned int size, unsigned int* mva);
TVR_STATUS TVR_DeallocMva(unsigned int va, unsigned int size, unsigned int mva);
unsigned int TVR_GetWorkingAddr(void);
bool       TVR_EnqueueBuffer(unsigned int addr, unsigned int* pAddrPrev);


// Debug
TVR_STATUS TVR_DumpRegisters(void);




// ---------------------------------------------------------------------------

#ifdef __cplusplus
}
#endif

#endif // __TVROT_DRV_H__
