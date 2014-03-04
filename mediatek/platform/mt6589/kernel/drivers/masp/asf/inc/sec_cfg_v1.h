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
 
#ifndef SEC_CFG_V1_H
#define SEC_CFG_V1_H

/**************************************************************************
 *  INCLUDE SEC CFG COMMON
 **************************************************************************/
#include "sec_cfg_common.h"

/**************************************************************************
 *  INCLUDE SIGN HEADER FORMAT
 **************************************************************************/
#include "sec_sign_header.h"

/**************************************************************************
 *  SECURE DOWNLOAD LOCK TABLE
 **************************************************************************/
#define MAX_IMG_LOCK_COUNT      (20)
#define IMAGE_LOCK_MAGIC        (0x4C4C4C4C) /* LLLL */

typedef struct
{
    unsigned int                magic_number; 
    unsigned char               name[32];   /* partition name */
    unsigned char               unlocked;

} IMAGE_DL_LOCK_INFO;

typedef struct
{
    unsigned int                magic_number;    
    unsigned char               lock_not_all;    
    IMAGE_DL_LOCK_INFO          lock_info[MAX_IMG_LOCK_COUNT];
   
} SECURE_DL_LOCK_TABLE;

/**************************************************************************
 *  SECURE IMAGE HEADER
 **************************************************************************/

typedef struct
{   
    unsigned int                magic_number;     
    unsigned char               name[16];     /* index for identification */    
    unsigned int                real_offset;  /* download agent will update the real offset */        
    ROM_TYPE                    image_type;   /* yaffs2 format or raw binary */
    SEC_IMG_ATTR                attr;         /* image attributes */
    SEC_IMG_HEADER_U            header;
    unsigned char               signature_hash [HASH_SIG_LEN];
    
} SECURE_IMG_INFO_V1;

/**************************************************************************
 *  SECURE CFG FORMAT
 **************************************************************************/
#define SECURE_IMAGE_COUNT      (12)
#define SEC_CFG_RESERVED        (4)

/* ================================= */
/* SECCFG FORMAT                     */
/* ================================= */
typedef struct
{   
    unsigned char               id[16];   
    unsigned int                magic_number;
    unsigned int                lib_ver;     

    unsigned int                sec_cfg_size; 

    unsigned char               sw_sec_lock_try;    
    unsigned char               sw_sec_lock_done;
    
    unsigned short              page_size; 
    unsigned int                page_count;

    /* ================== */
    /* encrypted region { */
    /* ================== */  
    SECURE_IMG_INFO_V1          image_info [SECURE_IMAGE_COUNT];
    SIU_STATUS                  siu_status; 
    unsigned char               reserve [SEC_CFG_RESERVED]; 
    SECCFG_STATUS               status;
    SECCFG_ATTR                 attr;
    /* ================== */
    /* encrypted region } */
    /* ================== */  
    
    SECURE_DL_LOCK_TABLE        lock_table;  
    unsigned int                end_pattern;

} SECURE_CFG_V1;

#endif // SEC_CFG_V1_H