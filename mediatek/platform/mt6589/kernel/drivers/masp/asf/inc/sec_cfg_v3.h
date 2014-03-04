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

#ifndef SEC_CFG_V3_H
#define SEC_CFG_V3_H

/**************************************************************************
 *  INCLUDE SEC CFG COMMON
 **************************************************************************/
#include "sec_cfg_common.h"

/**************************************************************************
 *  INCLUDE SIGN HEADER FORMAT
 **************************************************************************/
#include "sec_sign_header.h"

/**************************************************************************
 *  SECCFG EXTENSION REGION
 **************************************************************************/
#define EXT_REGION_BUF_SIZE     (4096)        /* used to store extension headers */
 
typedef struct
{    
    unsigned int                content_len;  /* total used data length */    
    unsigned char               buf [EXT_REGION_BUF_SIZE];
    
} SECCFG_EXT_REGION;

/**************************************************************************
 *  SECURE IMAGE HEADER
 **************************************************************************/

typedef struct
{   
    unsigned int                magic_number;     
    unsigned char               name [16];    /* index for identification */    
    unsigned int                real_offset;  /* download agent will update the real offset */        
    ROM_TYPE                    image_type;   /* yaffs2 format or raw binary */
    SEC_IMG_ATTR                image_attr;   /* image attributes */
    SEC_IMG_HEADER_U            header;       /* image sign header */
    unsigned int                ext_offset;   /* offset of extension header */    
    unsigned int                ext_len;      /* length of extension header */        
    
} SECURE_IMG_INFO_V3;

/**************************************************************************
 *  SECURE CFG FORMAT
 **************************************************************************/
#define SECURE_IMAGE_COUNT_V3   (20)

/* ================================= */
/* SECCFG FORMAT                     */
/* ================================= */
typedef struct
{   
    unsigned char               id[16];   
    unsigned int                magic_number;
    
    unsigned int                seccfg_ver;     
    unsigned int                seccfg_size; 

    unsigned int                seccfg_enc_offset; 
    unsigned int                seccfg_enc_len; 
    
    unsigned char               sw_sec_lock_try;
    unsigned char               sw_sec_lock_done;
    
    unsigned short              page_size; 
    unsigned int                page_count;

    /* ================== */
    /* encrypted region { */
    /* ================== */    
    SECURE_IMG_INFO_V3          image_info [SECURE_IMAGE_COUNT_V3];
    SIU_STATUS                  siu_status; 
    SECCFG_STATUS               seccfg_status;
    SECCFG_ATTR                 seccfg_attr;     
    SECCFG_EXT_REGION           seccfg_ext;
    /* ================== */    
    /* encrypted region } */
    /* ================== */    
    
    unsigned int                end_pattern;

} SECURE_CFG_V3;

#endif // SEC_CFG_V3_H
