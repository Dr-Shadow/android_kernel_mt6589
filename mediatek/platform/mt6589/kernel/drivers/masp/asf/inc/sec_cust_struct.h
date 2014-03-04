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

#ifndef _CUST_SEC_H
#define _CUST_SEC_H

/**************************************************************************
 * RSA PKCS01 CONTROL
 **************************************************************************/
#define RSA_KEY_LEN_2048                    (0)
#define RSA_KEY_LEN_1024                    (1)

/**************************************************************************
 * RSA PKCS01 CHECKING
 **************************************************************************/
#if RSA_KEY_LEN_2048
#if RSA_KEY_LEN_1024
#error "RSA_KEY_LEN_1024 should be disabled"
#endif
#endif

#if RSA_KEY_LEN_1024
#if RSA_KEY_LEN_2048
#error "RSA_KEY_LEN_2048 should be disabled"
#endif
#endif

/**************************************************************************
 * RSA PKCS01 CONFIGURATION
 **************************************************************************/
#define RSA_KEY_MAX_LEN                     (128) // bytes
#define RSA_E_KEY_LEN                       (5) // bytes

#if RSA_KEY_LEN_2048
#define RSA_KEY_LEN                         (256) // bytes
#define HASH_LEN                            (32)  // bytes

#elif RSA_KEY_LEN_1024
#define RSA_KEY_LEN                         (128) // bytes
#define HASH_LEN                            (20)  // bytes

#else
#error "RSA_KEY_LEN is not defined"

#endif

/**************************************************************************
 *  CUSTOMER INTERFACE
 **************************************************************************/
typedef struct _CUST_SEC_INTER
{
    /* key to sign image patch */
    unsigned char                   key_rsa_n[RSA_KEY_MAX_LEN*2]; // string. number base = 16
    unsigned char                   key_rsa_d[RSA_KEY_MAX_LEN*2]; // string. number base = 16
    unsigned char                   key_rsa_e[RSA_E_KEY_LEN]; // string. number base = 16
} CUST_SEC_INTER;

#endif   /*_CUST_SEC_H*/
