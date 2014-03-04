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

#include "sec_osal_light.h"

#include "sec_boot_lib.h"
#include "sec_boot.h"
#include "sec_error.h"
#include "sec_typedef.h"
#include "sec_log.h"

/**************************************************************************
 *  MACRO
 **************************************************************************/
#define MOD                                "ASF.USIF"

/**************************************************************************
 *  EXTERNAL VARIABLES
 *************************************************************************/
extern SECURE_INFO                         sec_info;


/**************************************************************************
 *  FIND DEVICE PARTITION
 **************************************************************************/
int sec_usif_check(void)
{
    int ret = SEC_OK;
    ASF_FILE fd;
    const uint32 buf_len = 2048;
    char *buf = ASF_MALLOC(buf_len);
    char *pmtdbufp;

    uint32 rn = 0;    
    ssize_t pm_sz;
    int cnt;

    ASF_GET_DS   

    /* -------------------------- */
    /* open proc device           */
    /* -------------------------- */
    SMSG(TRUE,"[%s] open /proc/dumchar_info\n",MOD);
    fd = ASF_OPEN("/proc/dumchar_info");   

    if (ASF_FILE_ERROR(fd))
    {
        SMSG(TRUE,"[%s] open /proc/dumchar_info fail\n",MOD);
        goto _usif_dis;
    }
    
    buf[buf_len - 1] = '\0';
    pm_sz = ASF_READ(fd, buf, buf_len - 1);
    pmtdbufp = buf;

    /* -------------------------- */
    /* parsing proc device        */
    /* -------------------------- */
    while (pm_sz > 0) 
    {
        int m_num, m_sz, mtd_e_sz;
        char m_name[16];
        m_name[0] = '\0';
        m_num = -1;

        m_num ++;

        /* -------------------------- */
        /* parsing proc/dumchar_info  */
        /* -------------------------- */        
        cnt = sscanf(pmtdbufp, "%15s %x %x %x",m_name, &m_sz, &mtd_e_sz, &rn);

        if ((4 == cnt) && (2 == rn))
        {
            SMSG(TRUE,"[%s] RN = 2\n",MOD);        
            goto _usif_en;
        }
        else if ((4 == cnt) && (1 == rn))
        {
            SMSG(TRUE,"[%s] RN = 1\n",MOD);        
            goto _usif_dis;            
        }

        while (pm_sz > 0 && *pmtdbufp != '\n') 
        {
            pmtdbufp++;
            pm_sz--;
        }
        
        if (pm_sz > 0) 
        {
            pmtdbufp++;
            pm_sz--;
        }
    }
    
    ret = ERR_USIF_PROC_RN_NOT_FOUND;
    ASF_CLOSE(fd);
    goto _exit;

_usif_en:
    SMSG(TRUE,"[%s] usif enabled\n",MOD);
    sec_info.bUsifEn = TRUE;
    ASF_CLOSE(fd);
    goto _exit;    

_usif_dis:
    SMSG(TRUE,"[%s] usif disabled\n",MOD);
    sec_info.bUsifEn = FALSE;    

_exit:
    ASF_FREE(buf);
    ASF_PUT_DS
        
    return ret;
}

