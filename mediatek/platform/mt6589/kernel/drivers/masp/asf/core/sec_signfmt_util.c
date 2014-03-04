/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 *
 * MediaTek Inc. (C) 2011. All rights reserved.
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


//#include "sec_osal_light.h"
#include "sec_signfmt_util.h"

/**************************************************************************
  *  GLOBAL VARIABLES
  **************************************************************************/ 
unsigned int sec_crypto_hash_size[] =
{
    CRYPTO_SIZE_UNKNOWN,
    SEC_SIZE_HASH_MD5,
    SEC_SIZE_HASH_SHA1,
    SEC_SIZE_HASH_SHA256,
    SEC_SIZE_HASH_SHA512
};
 
unsigned int sec_crypto_sig_size[] =
{
    CRYPTO_SIZE_UNKNOWN,
    SEC_SIZE_SIG_RSA512,
    SEC_SIZE_SIG_RSA1024,
    SEC_SIZE_SIG_RSA2048
};

/**************************************************************************
 *  UTILITY FUNCTIONS
 **************************************************************************/
unsigned int get_hash_size(SEC_CRYPTO_HASH_TYPE hash)
{
    return sec_crypto_hash_size[hash];
}
 
unsigned int get_signature_size(SEC_CRYPTO_SIGNATURE_TYPE sig)
{
    return sec_crypto_sig_size[sig];
}

unsigned char is_signfmt_v1(SEC_IMG_HEADER *hdr)
{
    if( 0 == hdr->signature_length )
    {        
        return true;
    }
    
    return false;
}

unsigned char is_signfmt_v2(SEC_IMG_HEADER *hdr)
{
    if( 0 == hdr->signature_length )
    {        
        return false;
    }
    else if( SEC_EXTENSION_MAGIC == hdr->sign_offset )
    {
        return false;
    }

    return true;
}

unsigned char is_signfmt_v3(SEC_IMG_HEADER *hdr)
{

    if( SEC_EXTENSION_MAGIC == hdr->sign_offset )
    {
        return true;
    }
    
    return false;
}

