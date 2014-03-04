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


#ifndef SECIMGHEADER_H
#define SECIMGHEADER_H

/**************************************************************************
 *  SIGN CFG PARSING
 **************************************************************************/
#define CUSTOM_NAME                  "CUSTOM_NAME"
#define IMAGE_VERSION                "IMAGE_VERSION"

/* in order to speedup verification, you can customize the image size 
   which should be signed and checked at boot time */
#define VERIFY_OFFSET                "VERIFY_OFFSET"
#define VERIFY_LENGTH                "VERIFY_LENGTH"

/**************************************************************************
 *  SEC IMAGE HEADER FORMAT
 **************************************************************************/
#define SEC_IMG_MAGIC                (0x53535353)
#define SEC_IMG_MAGIC_LEN            (4)

#define SEC_IMG_HEADER_SIZE          (64)

#define HASH_SIZE                    (20)
#define RSA_KEY_SIZE                 (128)
#define SIG_LEN                      (RSA_KEY_SIZE)
#define HASH_SIG_LEN                 (HASH_SIZE+SIG_LEN) 
                                     /* HASH + SIGNATURE */
/* Legacy (before W1151) */
typedef struct _SEC_IMG_HEADER_V1
{
    unsigned int magic_number;
    
    unsigned char cust_name [16];
    unsigned int image_version;
    unsigned int image_length;  
    unsigned int image_offset;
    
    unsigned int sign_offset;
    unsigned int sign_length;
    
    unsigned int signature_offset;
    unsigned int signature_length;

    unsigned char dummy[16];

} SEC_IMG_HEADER_V1;

/* New (After W1151) */ 
typedef struct _SEC_IMG_HEADER_V2
{
    unsigned int magic_number;

    /* After WK1151, the size of customer name will be changed from 16 
       bytes to 32 bytes due to customer's request. To distinguish between the 
       old version and new version, the simplest way is to check the value of 
       signature_length. 
    
       Flash tool downloads images by using new format 
       => Tool can find the image is old because signature_length is all 0x00.
          Therefore, Flash tool will automatically apply old image format */
              
    unsigned char cust_name [32];
    unsigned int image_version;
    unsigned int image_length;  
    unsigned int image_offset;
    
    unsigned int sign_offset;
    unsigned int sign_length;
    
    unsigned int signature_offset;
    unsigned int signature_length;
    
} SEC_IMG_HEADER_V2, SEC_IMG_HEADER_V3, SEC_IMG_HEADER;

typedef union
{
    SEC_IMG_HEADER_V1 v1;
    SEC_IMG_HEADER_V2 v2;
    SEC_IMG_HEADER_V3 v3;
    /*this implies all*/
    SEC_IMG_HEADER va;
} SEC_IMG_HEADER_U;

typedef enum
{
    SEC_HDR_V1 = 1,
    SEC_HDR_V2 = 2,
    SEC_HDR_V3 = 3,
    UNSET
    
} SEC_IMG_HEADER_VER;

/**************************************************************************
 *  IMAGE HEADER CHECK OPERATION
 **************************************************************************/

#define SEC_USBDL_WRITE_IMAGE_HEADER 0x11
#define SEC_USBDL_WRITE_IMAGE_SIGNATURE_HASH 0x22
#define SEC_USBDL_IMAGE_INFO_CHECK_RESULT 0x33
#define SEC_USBDL_WRITE_IMAGE_NAME 0x44
#define SEC_USBDL_WRITE_IMAGE_OFFSET 0x55
#define SEC_USBDL_WRITE_TYPE 0x66
#define SEC_USBDL_WRITE_IMAGE_EXTENSION 0x77


#define SEC_USBDL_IMAGE_NAME_LEN 16    
#define SEC_USBDL_IMAGE_INFO_RESULT_PASS 0x5555
#define SEC_USBDL_IMAGE_INFO_RESULT_FAIL 0x4444  


#endif   /* SECIMGHEADER_H */
