/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2005
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

/*****************************************************************************
 *
 * Filename:
 * ---------
 *   video_custom_sp.h
 *
 * Project:
 * --------
 *  ALPS
 *
 * Description:
 * ------------
 *   This file declared the customerized interface for smart phone
 *
 * Author:
 * -------
 *   Steve Su (mtk01898)
 *
 *============================================================================
 *             HISTORY
 * Below this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * $Log$
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#ifndef VIDEO_CUSTOM_SP_H
#define VIDEO_CUSTOM_SP_H

//#include "video_codec_if_sp.h"
#include "vcodec_if.h"

#define ASSERT(expr)                                            \
    do {                                                        \
        if (!(expr))                                            \
            AssertionFailed(__FUNCTION__,__FILE__, __LINE__);   \
    } while (0)

/******************************************************************************
 *
 *
 * decode
 *
 *
******************************************************************************/

typedef enum
{
    CODEC_DEC_NONE      = 0,
    CODEC_DEC_H263      = (0x1<<0),
    CODEC_DEC_MPEG4     = (0x1<<1),
    CODEC_DEC_H264      = (0x1<<2),
    CODEC_DEC_RV        = (0x1<<3),
    CODEC_DEC_VC1       = (0x1<<4),
    CODEC_DEC_VP8       = (0x1<<5),
    CODEC_DEC_MAX       = (0x1<<6)
} VIDEO_DECODER_T;

//VIDEO_DEC_API_T  *GetDecoderAPI(VIDEO_DECODER_T, HANDLE); // HANDLE : wrapper's handle
//VCODEC_DEC_API_T *GetMPEG4DecoderAPI(void);
//VCODEC_DEC_API_T *GetH264DecoderAPI(void);
//VCODEC_DEC_API_T *GetRVDecoderAPI(void);
//VCODEC_DEC_API_T *GetVP8DecoderAPI(void);
//VCODEC_DEC_API_T *GetVC1DecoderAPI(void);


/******************************************************************************
*
*
* encode
*
*
******************************************************************************/

typedef enum
{
    CODEC_ENC_NONE      = 0,
    CODEC_ENC_H263      = (0x1<<0),
    CODEC_ENC_MPEG4     = (0x1<<1),
    CODEC_ENC_H264      = (0x1<<2),
    CODEC_ENC_VP8       = (0x1<<5),
    CODEC_ENC_MAX       = (0x1<<6)
} VIDEO_ENCODER_T;

VIDEO_ENC_API_T *GetEncoderAPI(VIDEO_ENCODER_T eEncType, HANDLE hWrapper, void **ppDrvModule, unsigned int bUseMultiCoreCodec);
VCODEC_ENC_API_T *GetMPEG4EncoderAPI(void);
//VCODEC_ENC_API_T* GetH264EncoderAPI(void);
//VIDEO_ENCODER_API_T *GetVP8EncoderAPI(void);


#endif /* VIDEO_CUSTOM_SP_H */
