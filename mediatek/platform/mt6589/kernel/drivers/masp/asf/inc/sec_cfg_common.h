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


#ifndef SEC_CFG_COMMON_H
#define SEC_CFG_COMMON_H

/* ========================================================================= */
/*  ROM TYPE                                                                 */
/* ========================================================================= */
typedef enum {
    NORMAL_ROM                  = 0x01,
    YAFFS_IMG                   = 0x08,
} ROM_TYPE;

/* ========================================================================= */
/*  SECURE IMAGE HEADER                                                      */
/* ========================================================================= */
/* one image comes with one secure image information. */
#define SEC_IMG_MAGIC_NUM       (0x49494949)    /* IIII */

typedef enum
{
    ATTR_SEC_IMG_UPDATE         = 0x10,         /* only used in FlashTool */
    ATTR_SEC_IMG_COMPLETE       = 0x43434343,   /* CCCC */ 
    ATTR_SEC_IMG_INCOMPLETE     = 0x49494949,   /* IIII */
    ATTR_SEC_IMG_FORCE_UPDATE   = 0x46464646    /* FFFF */
    
} SEC_IMG_ATTR;

/* ========================================================================= */
/*  SECURE CFG STORAGE CONFIG                                                */
/* ========================================================================= */
/* buffer allocated in DA */
#define SEC_BUF_LEN             (0x3000)

/* ========================================================================= */
/*  SECURE CFG VERSION                                                       */
/* ========================================================================= */
#define SECCFG_SUPPORT_VERSION  (0x1)

/* ========================================================================= */
/*  SECURE CFG FORMAT                                                        */
/* ========================================================================= */
#define SEC_CFG_MAGIC_NUM       (0x4D4D4D4D)    /* MMMM */
#define SEC_CFG_BEGIN           "AND_SECCFG_v"
#define SEC_CFG_BEGIN_LEN       (12)

/* in order to avoid power loss potential issue */
/* before sec cfg start to update, status will be set as in-complete. */
typedef enum
{
    SEC_CFG_COMPLETE_NUM        = 0x43434343,   /* CCCC */ 
    SEC_CFG_INCOMPLETE_NUM      = 0x49494949    /* IIII */
     
} SECCFG_STATUS;

/* attributes which can disable secure boot (internal use only) */
typedef enum
{
    ATTR_DEFAULT                = 0x33333333,   /* 3333 */ 
    ATTR_DISABLE_IMG_CHECK      = 0x44444444    /* DDDD */
     
} SECCFG_ATTR;

/* specify image was upgraded by SIU or flash tool */
typedef enum
{
    UBOOT_UPDATED_BY_SIU        = 0x0001,
    BOOT_UPDATED_BY_SIU         = 0x0010,    
    RECOVERY_UPDATED_BY_SIU     = 0x0100,        
    SYSTEM_UPDATED_BY_SIU       = 0x1000            
     
} SIU_STATUS;

/* end pattern for debugging */
#define SEC_CFG_END_PATTERN     (0x45454545)    /* EEEE */

#endif // SEC_CFG_COMMON_H
