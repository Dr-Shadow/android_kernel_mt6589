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

#ifndef AC_REGION_H
#define AC_REGION_H

#include "sec_osal_light.h"

/**************************************************************************
 * [FILE SYSTEM SECROIMG PATH]
 **************************************************************************/
/* it is only used for non-security platform */
#define FS_SECRO_PATH                   "/system/secro/AC_REGION"


/**************************************************************************
 * [AC REGION ID]
 **************************************************************************/
#define ROM_SEC_AC_REGION_ID            "AND_AC_REGION"
#define ROM_SEC_AC_REGION_ID_LEN        (13)

#define RES_FOR_HEADER                  (0x400) // 1KB
#define ROM_SEC_AC_SEARCH_LEN           0x100000 // 1MB


/**************************************************************************
 * [AC-REGION HEADER FORNAT]
 **************************************************************************/
#define AC_H_MAGIC                      (0x48484848)
 
typedef struct {

    unsigned char                       m_id[16];
    unsigned int                        magic_number;

    unsigned int                        region_length;  /* include andro and sv5*/
    unsigned int                        region_offset; 

    unsigned int                        hash_length;  
    unsigned int                        hash_offset;
    
    unsigned int                        andro_length;   
    unsigned int                        andro_offset;   
    
    unsigned int                        md_length;   
    unsigned int                        md_offset;
    
    unsigned int                        md2_length;
    unsigned int                        md2_offset;     
    
    unsigned char                       reserve[4];
    
} AND_AC_HEADER_T;

/**************************************************************************
 * [AC-REGION ANDRO FORNAT]
 **************************************************************************/
#define AC_ANDRO_MAGIC                  (0x41414141)
#define AP_SECRO_MAX_LEN                (2939)

/* control flag */
#define FACTORY_EN_CODE                 (0x45)      

typedef struct {

    unsigned int                        magic_number;       
    unsigned char                       sml_aes_key[32]; /* sml aes key */ 
    unsigned char                       factory_en;    
    unsigned char                       reserve2[AP_SECRO_MAX_LEN];
    
} AND_AC_ANDRO_T;

/**************************************************************************
 * [AC-REGION MD FORNAT]
 **************************************************************************/
#define AC_MD_MAGIC                     (0x35353535)
#define MD_SECRO_MAX_LEN                (4092)

typedef struct {

    unsigned int                        magic_number;
    unsigned char                       reserve[MD_SECRO_MAX_LEN];
    
} AND_AC_MD_T;

#define AC_MD2_MAGIC                (0x36363636)
#define MD2_SECRO_MAX_LEN               (4092)

typedef struct {

    unsigned int                        magic_number;
    unsigned char                       reserve[MD2_SECRO_MAX_LEN];
    
} AND_AC_MD2_T;


/**************************************************************************
 * [AC-REGION FORNAT]
 **************************************************************************/
typedef struct {
    
    AND_AC_HEADER_T                     m_header;   /* 64 */
    AND_AC_ANDRO_T                      m_andro;    /* 0xBA0  : 2976 */
    AND_AC_MD_T                         m_md;       /* 0x1000 : 4096 */ 
    AND_AC_MD2_T                        m_md2;      /* 0x1000 : 4096 */      
    unsigned char                       hash[32];   /* it can be extended to SHA256 */
    
} AND_SECROIMG_T;

typedef enum{
    SECRO_MD1 = 0,
    SECRO_MD2,    
} SECRO_USER;

/**************************************************************************
 * [EXPORT FUNCTION]
 **************************************************************************/
extern unsigned int sec_secro_check (void);
extern bool sec_secro_en (void);
extern bool sec_secro_ac (void);
extern unsigned int sec_secro_md_len (SECRO_USER user);
extern unsigned int sec_secro_md_get_data (SECRO_USER user, unsigned char* buf, unsigned int offset, unsigned int len);

#endif /* AC_REGION_H */
