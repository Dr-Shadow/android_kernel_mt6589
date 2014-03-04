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

#ifndef SEC_BOOT_H
#define SEC_BOOT_H

/**************************************************************************
 * [S-BOOT]
 **************************************************************************/
 
/* S-BOOT Attribute */
#define ATTR_SBOOT_DISABLE                  0x00
#define ATTR_SBOOT_ENABLE                   0x11
#define ATTR_SBOOT_ONLY_ENABLE_ON_SCHIP     0x22

/**************************************************************************
 * [SECURE BOOT CHECK] 
 **************************************************************************/
 
/* Note : this structure record all the partitions
          which should be verified by secure boot check */
#define AND_SEC_BOOT_CHECK_PART_SIZE        (90)
typedef struct 
{
    unsigned char                           name[9][10];
    
} AND_SECBOOT_CHECK_PART_T;

/* Note : partition name between preloader/DA and kernel mtd table may be different
          in order to reduce maintainence effort, secure boot update will apply
          following transfation table to correct mtd partition name */
#define MTD_SECCFG                     "seccnfg"
#define MTD_UBOOT                      "uboot"
#define MTD_LOGO                       "logo"
#define MTD_BOOTIMG                    "boot"
#define MTD_USER                       "userdata"
#define MTD_ANDSYSIMG                  "system"
#define MTD_RECOVERY                   "recovery"
#define MTD_SECRO                      "secstatic"

#define USIF_SECCFG                    "seccfg"
#define USIF_UBOOT                     "uboot"
#define USIF_LOGO                      "logo"
#define USIF_BOOTIMG                   "bootimg"
#define USIF_USER                      "userdata"
#define USIF_ANDSYSIMG                 "android"
#define USIF_RECOVERY                  "recovery"
#define USIF_SECRO                     "sec_ro"

#define PL_SECCFG                      "SECCFG"
#define PL_UBOOT                       "UBOOT"
#define PL_LOGO                        "LOGO"
#define PL_BOOTIMG                     "BOOTIMG"
#define PL_USER                        "USRDATA"
#define PL_ANDSYSIMG                   "ANDROID"
#define PL_RECOVERY                    "RECOVERY"
#define PL_SECRO                       "SEC_RO"

/**************************************************************************
 * EXPORT FUNCTION
 **************************************************************************/
extern int sec_boot_init (void);
extern int sec_boot_enabled (void);
extern int sec_modem_auth_enabled (void);
extern int sec_schip_enabled (void);

#endif /* SEC_BOOT_H */

