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


#include "sec_boot_lib.h"
#include "sec_ccci.h"

/**************************************************************************
 *  MODULE NAME
 **************************************************************************/
#define MOD                         "SEC_CCCI"

int sec_ccci_signfmt_verify_file(char *file_path, unsigned int *data_offset, unsigned int *data_sec_len)
{
    unsigned int ret = SEC_OK; 
    SEC_IMG_HEADER img_hdr;

    *data_offset = 0;
    *data_sec_len = 0;

    ret = sec_signfmt_verify_file(file_path, &img_hdr, data_offset, data_sec_len);

    /* image is not signed */
    if( ret == ERR_SIGN_FORMAT_MAGIC_WRONG )
    {
        if((sec_modem_auth_enabled() == 0) && (sec_schip_enabled() == 0)) 
        {
            SMSG(true,"[%s] image has no sec header\n",MOD);
			ret = SEC_OK;
			goto _out;
		}
		else 
        {
            SMSG(true,"[%s] (img not signed) sec_modem_auth_enabled() = %d\n",MOD,sec_modem_auth_enabled());
            SMSG(true,"[%s] (img not signed) sec_schip_enabled() = %d\n",MOD,sec_schip_enabled());
			ret = ERR_SIGN_FORMAT_MAGIC_WRONG;
			goto _out;
		}
    }

    if( ret != SEC_OK )
    {
        SMSG(true,"[%s] file '%s' verify failed\n",MOD,file_path);
        goto _out;
    }

    SMSG(true,"[%s] data_offset is %d\n",MOD,*data_offset);
    SMSG(true,"[%s] data_sec_len is %d\n",MOD,*data_sec_len);

_out:
    
    return ret;
}

int sec_ccci_version_info(void)
{
    return CCCI_VERSION;
}

int sec_ccci_file_open(char *file_path)
{
    int fp_id;

    fp_id = osal_filp_open_read_only(file_path);

    if(fp_id != OSAL_FILE_NULL)
    {
        return fp_id;
    }

    return -1;
}

int sec_ccci_file_close(int fp_id)
{
    return osal_filp_close(fp_id);
}


int sec_ccci_is_cipherfmt(int fp_id, unsigned int start_off, unsigned int *img_len)
{
    if( SEC_OK != sec_cipherfmt_check_cipher(fp_id, start_off, img_len) )
    {
        *img_len = 0;
        return 0;
    }

    return 1;
}

int sec_ccci_decrypt_cipherfmt(int fp_id, unsigned int start_off, char *buf, unsigned int buf_len, unsigned int *data_offset)
{
    return sec_cipherfmt_decrypted(fp_id, start_off, buf, buf_len, data_offset);
}


