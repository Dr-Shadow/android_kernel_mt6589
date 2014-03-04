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


#ifndef SEC_HDR_H
#define SEC_HDR_H

/******************************************************************************
 *  INCLUDE LIBRARY
 ******************************************************************************/
#include "sec_boot_lib.h"

/**************************************************************************
 * EXPORT FUNCTIONS
 **************************************************************************/
extern unsigned int shdr_magic (SEC_IMG_HEADER_U* sec_hdr);
extern unsigned char* shdr_cust_name (SEC_IMG_HEADER_U* sec_hdr);
extern unsigned int shdr_cust_name_len (SEC_IMG_HEADER_U* sec_hdr);
extern unsigned int shdr_img_ver (SEC_IMG_HEADER_U* sec_hdr);
extern unsigned int shdr_img_len (SEC_IMG_HEADER_U* sec_hdr);
extern unsigned int shdr_img_offset (SEC_IMG_HEADER_U* sec_hdr);
extern unsigned int shdr_sign_len (SEC_IMG_HEADER_U* sec_hdr);
extern unsigned int shdr_sign_offset (SEC_IMG_HEADER_U* sec_hdr);
extern unsigned int shdr_sig_len (SEC_IMG_HEADER_U* sec_hdr);
extern unsigned int shdr_sig_offset (SEC_IMG_HEADER_U* sec_hdr);
extern void set_shdr_magic (SEC_IMG_HEADER_U* sec_hdr, unsigned int val);
extern void set_shdr_img_ver (SEC_IMG_HEADER_U* sec_hdr, unsigned int ver);
extern void set_shdr_cust_name (SEC_IMG_HEADER_U* sec_hdr, unsigned char* name, unsigned int len);
extern void set_shdr_sign_len (SEC_IMG_HEADER_U* sec_hdr, unsigned int val);
extern void set_shdr_sign_offset (SEC_IMG_HEADER_U* sec_hdr, unsigned int val);
extern void set_shdr_ver (SEC_IMG_HEADER_VER ver);
extern SEC_IMG_HEADER_VER get_shdr_ver (void);

#endif /* SEC_HDR_H */

