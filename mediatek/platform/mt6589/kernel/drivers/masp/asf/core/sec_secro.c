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
     
#include <mach/sec_osal.h> 
#include "sec_typedef.h"
#include "sec_rom_info.h"
#include "sec_usbdl.h"
#include "sec_secroimg.h"
#include "hacc_export.h"
#include "sec_error.h"
#include "sec_lib.h"
#include "sec_log.h"
#include "alg_sha1.h"

/******************************************************************************
 *  DEFINITIONS
 ******************************************************************************/
#define MOD                         "ASF"


/******************************************************************************
 *  GLOBAL VARIABLES
 ******************************************************************************/
AND_SECROIMG_T                      secroimg;
bool                                bSecroExist = FALSE;
bool                                bSecroIntergiy = FALSE;
              
/******************************************************************************
 *  EXTERNAL VARIABLES
 ******************************************************************************/
extern AND_ROMINFO_T                rom_info;
extern uchar                        sha1sum[];


/******************************************************************************
 * VALIDATE SECRO
 ******************************************************************************/
uint32 sec_secro_check (void)
{
    uint32 ret = SEC_OK;

    /* ------------------------ */       
    /* check header             */
    /* ------------------------ */                
    if(AC_ANDRO_MAGIC != secroimg.m_andro.magic_number)
    {
        ret = ERR_SECROIMG_HACC_AP_DECRYPT_FAIL;
        goto _end;
    }

    if(AC_MD_MAGIC != secroimg.m_md.magic_number)
    {
        ret = ERR_SECROIMG_HACC_MD_DECRYPT_FAIL;                    
        goto _end;
    }

    if(AC_MD2_MAGIC != secroimg.m_md2.magic_number)
    {
        ret = ERR_SECROIMG_HACC_MD_DECRYPT_FAIL;                    
        goto _end;
    }

    /* ------------------------ */       
    /* check integrity          */
    /* ------------------------ */
    if(FALSE == bSecroIntergiy)
    {
        sha1((uchar*)&secroimg, sizeof(AND_SECROIMG_T) - sizeof(secroimg.hash), sha1sum);
        SMSG(TRUE,"[%s] hash value :\n",MOD);                        
        dump_buf(sha1sum,secroimg.m_header.hash_length);
        SMSG(TRUE,"[%s] correct :\n",MOD);                        
        dump_buf(secroimg.hash,secroimg.m_header.hash_length);                             
                    
        if(0 != mcmp(secroimg.hash, sha1sum, secroimg.m_header.hash_length))
        {
            SMSG(TRUE,"[%s] SECRO hash check fail\n",MOD);
            ret = ERR_SECROIMG_HASH_CHECK_FAIL;                    
            goto _end;                    
        }                

        bSecroIntergiy = TRUE;
        SMSG(TRUE,"[%s] SECRO hash check pass\n",MOD);
    }

_end:

    return ret;
}

/******************************************************************************
 * CHECK IF SECROIMG IS USED
 ******************************************************************************/
bool sec_secro_en (void)
{
    /* return ProjectConfig's setting */
    if(TRUE == rom_info.m_SEC_CTRL.m_secro_ac_en)
    {    
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

/******************************************************************************
 * CHECK IF SECROIMG AC IS ENABLED
 ******************************************************************************/
bool sec_secro_ac (void)
{
    /* PLEASE NOTE THAT !!!!!!!!!!!!!!!!!!
       SECRO AC is only effected when SUSBDL is on */
    if(TRUE == sec_usbdl_enabled())
    {    
        return TRUE;
    }
    /* If security chip, secroimage must be encrypted */
    else if(TRUE == es_sbc_enabled())
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

/******************************************************************************
 * RETURN SECROIMG BLK SIZE
 ******************************************************************************/
uint32 sec_secro_blk_sz (void)
{
    return sp_hacc_blk_sz();
}


/******************************************************************************
 * RETURN SECROIMG MD LENGTH
 ******************************************************************************/
uint32 sec_secro_md_len (SECRO_USER user)
{
    if(TRUE == bSecroExist)
    {
        switch(user)
        {
            case SECRO_MD1:
                return secroimg.m_header.md_length;
            case SECRO_MD2:
                return secroimg.m_header.md2_length;
            default:
                return 0;
        }
    }
    else
    {
        return 0;
    }
}

/******************************************************************************
 * RETURN SECROIMG MD PLAINTEXT DATA
 ******************************************************************************/
uint32 sec_secro_md_get_data (SECRO_USER user, uchar* buf, uint32 offset, uint32 len)
{
    uint32 ret = SEC_OK;
    const uint32 cipher_len = sizeof(AND_AC_ANDRO_T) + sizeof(AND_AC_MD_T) + sizeof(AND_AC_MD2_T);   

    osal_secro_lock();

    /* ----------------- */
    /* check             */
    /* ----------------- */

    if(FALSE == bSecroExist)
    {
        ret = ERR_SECROIMG_IS_EMPTY;
        goto _exit;
    }

    if(len == 0)
    {
        ret = ERR_SECROIMG_INVALID_BUF_LEN;
        goto _exit;
    }

    if (0 != (len % sp_hacc_blk_sz())) 
    {   
        ret = ERR_HACC_DATA_UNALIGNED;
        goto _exit;
    }    

    /* ------------------------ */       
    /* decrypt secroimg         */
    /* ------------------------ */
    if(TRUE == sec_secro_ac())
    {
        sp_hacc_dec((uchar*)&secroimg.m_andro, cipher_len, TRUE,HACC_USER1,TRUE);
    }

    /* ------------------------ */       
    /* check header             */
    /* ------------------------ */                
    if(AC_ANDRO_MAGIC != secroimg.m_andro.magic_number)
    {
        ret = ERR_SECROIMG_HACC_AP_DECRYPT_FAIL;
        goto _exit;
    }

    if(AC_MD_MAGIC != secroimg.m_md.magic_number)
    {
        ret = ERR_SECROIMG_HACC_MD_DECRYPT_FAIL;                    
        goto _exit;
    }
  
    if(AC_MD2_MAGIC != secroimg.m_md2.magic_number)
    {
        ret = ERR_SECROIMG_HACC_MD_DECRYPT_FAIL;                    
        goto _exit;
    }
    
    /* ------------------------ */       
    /* fill buffer              */
    /* ------------------------ */    
    /* only copy the data with user specified length */
    switch(user)
    {
        case SECRO_MD1:
            mcpy(buf,secroimg.m_md.reserve+offset,len);
            break;
        case SECRO_MD2:
            mcpy(buf,secroimg.m_md2.reserve+offset,len);
            break;
        default:
            SMSG(TRUE,"[%s] MD user not supported!\n",MOD);            
            break;
    }
    
    /* ------------------------ */       
    /* encrypt secro image      */
    /* ------------------------ */ 
    if(TRUE == sec_secro_ac())
    {    
        sp_hacc_enc((uchar*)&secroimg.m_andro, cipher_len, TRUE,HACC_USER1,TRUE);
    }

_exit:

    osal_secro_unlock();

    return ret;
}
