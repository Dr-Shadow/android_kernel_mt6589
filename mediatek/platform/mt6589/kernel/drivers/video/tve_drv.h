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

#ifndef __TVE_DRV_H__
#define __TVE_DRV_H__

#include "disp_drv.h"
#include "tv_def.h"


#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------------

#define TVE_CHECK_RET(expr)             \
    do {                                \
        TVE_STATUS ret = (expr);        \
        ASSERT(TVE_STATUS_OK == ret);   \
    } while (0)

// ---------------------------------------------------------------------------

typedef enum
{
   TVE_STATUS_OK = 0,

   TVE_STATUS_ERROR,
} TVE_STATUS;


typedef enum
{
    TVE_NTSC  = 0, // 525 lines
    TVE_PAL_M = 1, // 525 lines
    TVE_PAL_C = 2, // 625 lines
    TVE_PAL   = 3, // 625 lines
} TVE_TV_TYPE;


// ---------------------------------------------------------------------------

TVE_STATUS TVE_Init(void);
TVE_STATUS TVE_Deinit(void);

TVE_STATUS TVE_PowerOn(void);
TVE_STATUS TVE_PowerOff(void);

TVE_STATUS TVE_Enable(void);
TVE_STATUS TVE_Disable(void);

TVE_STATUS TVE_SetTvType(TVE_TV_TYPE type);
TVE_STATUS TVE_EnableColorBar(BOOL enable);

TVE_STATUS TVE_ResetDefaultSettings(void);


// Debug
TVE_STATUS TVE_DumpRegisters(void);

// ---------------------------------------------------------------------------

#ifdef __cplusplus
}
#endif

#endif // __TVE_DRV_H__
