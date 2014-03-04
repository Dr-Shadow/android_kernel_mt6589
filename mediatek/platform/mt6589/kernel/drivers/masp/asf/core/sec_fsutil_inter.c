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


/******************************************************************************
 *  INCLUDE LIBRARY
 ******************************************************************************/
#include "sec_boot_lib.h"

/**************************************************************************
 *  MODULE NAME
 **************************************************************************/
#define MOD                         "ASF.FS"

/**************************************************************************
 *  EXTERNAL VARIABLES
 **************************************************************************/
extern bool                         bSecroExist;
extern bool                         bSecroIntergiy;

/**************************************************************************
 *  READ SECRO
 **************************************************************************/
uint32 sec_fs_read_secroimg (char* path, char* buf)
{
    uint32 ret  = SEC_OK;    
    const uint32 size = sizeof(AND_SECROIMG_T);
    uint32 temp = 0;    
    ASF_FILE fd;

    /* ------------------------ */    
    /* check parameter          */
    /* ------------------------ */
    SMSG(TRUE,"[%s] open '%s'\n",MOD,path);
    if(0 == size)
    {
        ret = ERR_FS_SECRO_READ_SIZE_CANNOT_BE_ZERO;
        goto _end;
    }

    /* ------------------------ */    
    /* open secro               */
    /* ------------------------ */    
    fd = ASF_OPEN(path);
    
    if (ASF_IS_ERR(fd)) 
    {
        ret = ERR_FS_SECRO_OPEN_FAIL;
        goto _open_fail;
    }

    /* ------------------------ */
    /* read secro               */
    /* ------------------------ */    
    /* configure file system type */
    osal_set_kernel_fs();
    
    /* adjust read off */
    ASF_SEEK_SET(fd,0);     
    
    /* read secro content */   
    if(0 >= (temp = ASF_READ(fd,buf,size)))
    {
        ret = ERR_FS_SECRO_READ_FAIL;
        goto _end;
    }

    if(size != temp)
    {
        SMSG(TRUE,"[%s] size '0x%x', read '0x%x'\n",MOD,size,temp);
        ret = ERR_FS_SECRO_READ_WRONG_SIZE;
        goto _end;
    }

    /* ------------------------ */       
    /* check integrity          */
    /* ------------------------ */
    if(SEC_OK != (ret = sec_secro_check()))
    {        
        goto _end;
    }

    /* ------------------------ */       
    /* SECROIMG is valid        */
    /* ------------------------ */
    bSecroExist = TRUE;    

_end:    
    ASF_CLOSE(fd);
    osal_restore_fs();
    
_open_fail:
    return ret;
}


