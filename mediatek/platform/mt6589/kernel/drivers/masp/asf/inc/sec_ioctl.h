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

#ifndef SEC_IOCTL_H
#define SEC_IOCTL_H

/* use 's' as magic number */
#define SEC_IOC_MAGIC       's'

/* random id */
#define SEC_GET_RANDOM_ID               _IOR(SEC_IOC_MAGIC,  1, unsigned int)

/* secure boot init */
#define SEC_BOOT_INIT                   _IOR(SEC_IOC_MAGIC,  2, unsigned int)
#define SEC_BOOT_IS_ENABLED             _IOR(SEC_IOC_MAGIC,  3, unsigned int)

/* secure seccfg process */
#define SEC_SECCFG_DECRYPT              _IOR(SEC_IOC_MAGIC,  4, unsigned int)
#define SEC_SECCFG_ENCRYPT              _IOR(SEC_IOC_MAGIC,  5, unsigned int)

/* secure usbdl */
#define SEC_USBDL_IS_ENABLED            _IOR(SEC_IOC_MAGIC,  6, unsigned int)

/* HACC HW */
#define SEC_HACC_CONFIG                  _IOR(SEC_IOC_MAGIC,  7, unsigned int)
#define SEC_HACC_LOCK                    _IOR(SEC_IOC_MAGIC,  8, unsigned int)
#define SEC_HACC_UNLOCK                  _IOR(SEC_IOC_MAGIC,  9, unsigned int)
#define SEC_HACC_ENABLE_CLK              _IOR(SEC_IOC_MAGIC, 10, unsigned int)

/* secure boot check */
#define SEC_BOOT_PART_CHECK_ENABLE      _IOR(SEC_IOC_MAGIC, 11, unsigned int)
#define SEC_BOOT_NOTIFY_MARK_STATUS     _IOR(SEC_IOC_MAGIC, 12, unsigned int)
#define SEC_BOOT_NOTIFY_PASS            _IOR(SEC_IOC_MAGIC, 13, unsigned int)
#define SEC_BOOT_NOTIFY_FAIL            _IOR(SEC_IOC_MAGIC, 14, unsigned int)
#define SEC_BOOT_NOTIFY_RMSDUP_DONE     _IOR(SEC_IOC_MAGIC, 15, unsigned int)

/* rom info */
#define SEC_READ_ROM_INFO               _IOR(SEC_IOC_MAGIC, 16, unsigned int)

/* META */
#define SEC_NVRAM_HW_ENCRYPT            _IOR(SEC_IOC_MAGIC, 17, unsigned int)
#define SEC_NVRAM_HW_DECRYPT            _IOR(SEC_IOC_MAGIC, 18, unsigned int)

#define SEC_IOC_MAXNR       (19)

#define SEC_DEV             "/dev/sec"

#endif /* end of SEC_IOCTL_H */
