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

#include "sec_osal_light.h"

/**************************************************************************
 *  TYPEDEF
 **************************************************************************/
typedef unsigned int uint32;
typedef unsigned char uchar;


/**************************************************************************
 *  AES FUNCTION
 **************************************************************************/
#include "sec_aes.h"
/* legacy function used for W1128/32 MP */
#include "aes_legacy.h"
/* standard operation aes function used for W1150 MP */
#include "aes_so.h"

/**************************************************************************
 *  DEFINITIONS
 **************************************************************************/
#define MOD                             "AES_EXPORT"

/**************************************************************************
 *  MACRO
 **************************************************************************/
#define SMSG printk

/**************************************************************************
 *  GLOBAL VARIABLE
 **************************************************************************/
/* using W1128/32 mP solution by default */
AES_VER g_ver = AES_VER_LEGACY; 

/**************************************************************************
 *  LIBRARY EXPORT FUNCTION - ENCRYPTION
 **************************************************************************/
int lib_aes_enc(uchar* input_buf,  uint32 input_len, uchar* output_buf, uint32 output_len)
{
    
    switch (g_ver)
    {
        case AES_VER_LEGACY:
            if(0 != aes_legacy_enc(input_buf,input_len,output_buf,output_len))
            {
                goto _err;
            }
            break;
            
        case AES_VER_SO:
            if(0 != aes_so_enc(input_buf,input_len,output_buf,output_len))
            {
                goto _err;
            }
            break;
            
        default:
            SMSG("[%s] Invalid Ver\n",MOD);
            goto _err;
    }    
    
    return 0;

_err:

    return -1;    

}

/**************************************************************************
 *  LIBRARY EXPORT FUNCTION - DECRYPTION
 **************************************************************************/
int lib_aes_dec(uchar* input_buf,  uint32 input_len, uchar* output_buf, uint32 output_len)
{
    switch (g_ver)
    {
        case AES_VER_LEGACY:
            if(0 != aes_legacy_dec(input_buf,input_len,output_buf,output_len))
            {
                goto _err;
            }
            break;
            
        case AES_VER_SO:
            if(0 != aes_so_dec(input_buf,input_len,output_buf,output_len))
            {
                goto _err;
            }
            break;
            
        default:
            SMSG("[%s] Invalid Ver\n",MOD);
            goto _err;
    }    
    
    return 0;

_err:

    return -1;    

}

/**************************************************************************
 *  LIBRARY EXPORT FUNCTION - KEY INITIALIZATION
 **************************************************************************/
int lib_aes_init_key(uchar* key_buf,  uint32 key_len, AES_VER ver)
{
    switch (ver)
    {
        case AES_VER_LEGACY:
            g_ver = AES_VER_LEGACY;
            SMSG("\n[%s] Legacy\n",MOD);
            if(0 != aes_legacy_init_key(key_buf,key_len))
            {
                goto _err;
            }
            break;
            
        case AES_VER_SO:
            g_ver = AES_VER_SO;
            SMSG("\n[%s] SO\n",MOD);
            if(0 != aes_so_init_key(key_buf,key_len))
            {
                goto _err;            
            }
            break;
            
        default:
            SMSG("\n[%s] Invalid Ver\n",MOD);
            goto _err;
    }
    
    return 0;

_err:

    return -1;    
}

int lib_aes_init_vector(AES_VER ver)
{
    switch (ver)
    {
        case AES_VER_LEGACY:
            g_ver = AES_VER_LEGACY;
            SMSG("[%s] Legacy(V)\n",MOD);
            if(0 != aes_legacy_init_vector())
            {
                goto _err;
            }
            break;
            
        case AES_VER_SO:
            g_ver = AES_VER_SO;
            SMSG("[%s] SO(V)\n",MOD);
            if(0 != aes_so_init_vector())
            {
                goto _err;            
            }
            break;
            
        default:
            SMSG("[%s] Invalid Ver(V)\n",MOD);
            goto _err;
    }
    
    return 0;

_err:
    return -1;
}

