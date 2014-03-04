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

#ifndef _RSA_H
#define _RSA_H

#include "bgn_export.h"
#include "sec_cust_struct.h"

/**************************************************************************
 *  INCLUDING
 **************************************************************************/
#include "alg_sha1.h"

/**************************************************************************
 *  MODE SELECTION
 **************************************************************************/
#define SIG_RSA_RAW     0
#define RSA_SIGN        1

/**************************************************************************
 *  CORE DATA STRUCTURE
 **************************************************************************/
typedef struct
{
    int len; 

    int pad;  
    int h_id;  
    int (*f_rng)(void *); 
    void *p_rng;    

    /* keys { */
    bgn N;
    bgn E;
    bgn D;

    bgn RN;
    bgn RP;
    bgn RQ;          
    /* keys } */    
}
rsa_ctx;

/**************************************************************************
 *  EXPORT FUNCTIONS
 **************************************************************************/    
int rsa_sign( rsa_ctx *ctx, int h_len, const unsigned char *hash, unsigned char *sig );
int rsa_verify( rsa_ctx *ctx, int h_len, const unsigned char *hash, unsigned char *sig );    

/**************************************************************************
 *  EXPORT VARIABLES
 **************************************************************************/        
extern rsa_ctx rsa;


/**************************************************************************
 *  ERROR CODE
 **************************************************************************/
#define E_RSA_BAD_INPUT_DATA                    0x0001
#define E_RSA_INVALID_PADDING                   0x0002
#define E_RSA_KEY_GEN_FAILED                    0x0003
#define E_RSA_KEY_CHECK_FAILED                  0x0004
#define E_RSA_PUBLIC_FAILED                     0x0005
#define E_RSA_PRIVATE_FAILED                    0x0006
#define E_RSA_VERIFY_FAILED                     0x0007
#define E_RSA_OUTPUT_TOO_LARGE                  0x0008

#endif
