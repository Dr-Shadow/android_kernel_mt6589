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


/**************************************************************************
 *  INCLUDE LIBRARY
 **************************************************************************/
#include "sec_boot_lib.h"

#define PART_PATH_PREFIX                   "/dev/"

/**************************************************************************
 *  EXTERNAL VARIABLES
 *************************************************************************/
extern SECURE_INFO                         sec_info;

/**************************************************************************
 *  CHECK USIF ENABLED OR NOT
 **************************************************************************/
bool sec_usif_enabled(void)
{
    return sec_info.bUsifEn;
}

/**************************************************************************
 *  RETURN PART NAME
 **************************************************************************/
void sec_usif_part_name (uint32 part_num, char* part_name)
{
    mcpy(part_name,pl2usif(mtd_part_map[part_num].name),strlen(pl2usif(mtd_part_map[part_num].name)));
}

/**************************************************************************
 *  PART NUM TO PART PATH
 **************************************************************************/
void sec_usif_part_path(uint32 part_num, char* part_path, uint32 part_path_len)
{    
    memset(part_path,0x0,part_path_len);
    mcpy(part_path,PART_PATH_PREFIX,strlen(PART_PATH_PREFIX));
    sec_usif_part_name(part_num,part_path+strlen(PART_PATH_PREFIX));
    SMSG(TRUE,"usif part path %s\n",part_path);
    
}

/**************************************************************************
 *  PART NAME QUERY
 **************************************************************************/
char* usif2pl (char* part_name)
{
    /* ----------------- */
    /* seccfg            */
    /* ----------------- */    
    if(0 == mcmp(part_name,USIF_SECCFG,strlen(USIF_SECCFG)))
    {   
        return (char*) PL_SECCFG;
    }
    /* ----------------- */
    /* uboot             */
    /* ----------------- */    
    else if(0 == mcmp(part_name,USIF_UBOOT,strlen(USIF_UBOOT)))
    {   
        return (char*) PL_UBOOT;
    }
    /* ----------------- */    
    /* logo              */
    /* ----------------- */    
    else if(0 == mcmp(part_name,USIF_LOGO,strlen(USIF_LOGO)))
    {
        return (char*) PL_LOGO;
    }
    /* ----------------- */
    /* boot image        */
    /* ----------------- */    
    else if(0 == mcmp(part_name,USIF_BOOTIMG,strlen(USIF_BOOTIMG)))
    {
        return (char*) PL_BOOTIMG;
    }
    /* ----------------- */    
    /* user data         */
    /* ----------------- */    
    else if(0 == mcmp(part_name,USIF_USER,strlen(USIF_USER)))
    {
        return (char*) PL_USER;               
    }   
    /* ----------------- */    
    /* system image      */
    /* ----------------- */    
    else if(0 == mcmp(part_name,USIF_ANDSYSIMG,strlen(USIF_ANDSYSIMG)))
    {
        return (char*) PL_ANDSYSIMG;
    }   
    /* ----------------- */    
    /* recovery          */
    /* ----------------- */    
    else if(0 == mcmp(part_name,USIF_RECOVERY,strlen(USIF_RECOVERY)))
    {
        return (char*) PL_RECOVERY;
    }       
    /* ----------------- */    
    /* sec ro            */
    /* ----------------- */    
    else if(0 == mcmp(part_name,USIF_SECRO,strlen(USIF_SECRO)))
    {
        return (char*) PL_SECRO;
    }
    /* ----------------- */    
    /* not found         */
    /* ----------------- */    
    else
    {
        return part_name;
    }    
}

char* pl2usif (char* part_name)
{
    /* ----------------- */
    /* seccfg            */
    /* ----------------- */    
    if(0 == mcmp(part_name,PL_SECCFG,strlen(PL_SECCFG)))
    {   
        return (char*) USIF_SECCFG;
    }
    /* ----------------- */    
    /* uboot             */
    /* ----------------- */    
    else if(0 == mcmp(part_name,PL_UBOOT,strlen(PL_UBOOT)))
    {   
        return (char*) USIF_UBOOT;
    }
    /* ----------------- */    
    /* logo              */
    /* ----------------- */    
    else if(0 == mcmp(part_name,PL_LOGO,strlen(PL_LOGO)))
    {
        return (char*) USIF_LOGO;
    }
    /* ----------------- */    
    /* boot image        */
    /* ----------------- */    
    else if(0 == mcmp(part_name,PL_BOOTIMG,strlen(PL_BOOTIMG)))
    {
        return (char*) USIF_BOOTIMG;
    }
    /* ----------------- */    
    /* user data         */
    /* ----------------- */    
    else if(0 == mcmp(part_name,PL_USER,strlen(PL_USER)))
    {
        return (char*) USIF_USER;               
    }   
    /* ----------------- */    
    /* system image      */
    /* ----------------- */    
    else if(0 == mcmp(part_name,PL_ANDSYSIMG,strlen(PL_ANDSYSIMG)))
    {
        return (char*) USIF_ANDSYSIMG;
    }   
    /* ----------------- */    
    /* recovery          */
    /* ----------------- */    
    else if(0 == mcmp(part_name,PL_RECOVERY,strlen(PL_RECOVERY)))
    {
        return (char*) USIF_RECOVERY;
    }       
    /* ----------------- */    
    /* sec ro            */
    /* ----------------- */    
    else if(0 == mcmp(part_name,PL_SECRO,strlen(PL_SECRO)))
    {
        return (char*) USIF_SECRO;
    }
    /* ----------------- */    
    /* not found         */
    /* ----------------- */    
    else
    {
        return part_name;
    }    
}
