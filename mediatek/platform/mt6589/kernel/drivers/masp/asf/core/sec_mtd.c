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
#include "sec_osal_light.h"
#include "sec_boot_lib.h"
#include "sec_rom_info.h"
#include "sec_secroimg.h"
#include "hacc_export.h"
#include "sec_boot.h"
#include "sec_error.h"
#include "alg_sha1.h"
#include "sec_mtd.h"
#include "sec_typedef.h"
#include "sec_log.h"

/**************************************************************************
 *  MACRO
 **************************************************************************/
#define MOD                         "ASF"

/**************************************************************************
 *  EXTERNAL VARIABLES
 *************************************************************************/
extern AND_ROMINFO_T                rom_info;
extern uchar                        sha1sum[];
extern AND_SECROIMG_T               secroimg;
extern bool                         bSecroExist;
extern bool                         bSecroIntergiy;
extern uint32                       secro_img_off;
extern uint32                       secro_img_mtd_num;

/**************************************************************************
 *  GET MTD PARTITION OFFSET
 **************************************************************************/
uint32 sec_mtd_get_off(char* part_name)
{
    uint32 i = 0;
    
    for(i = 0; i < MAX_MTD_PARTITIONS; i++) 
    {
        if(0 == mcmp(mtd_part_map[i].name,part_name,strlen(mtd_part_map[i].name)))
        {
            return mtd_part_map[i].off;
        }
    }

    SEC_ASSERT(0);
    return 0;
}

/**************************************************************************
 *  READ IMAGE
 **************************************************************************/
uint32 sec_mtd_read_image(char* part_name, char* buf, uint32 off, uint32 size)
{
    ASF_FILE fp;
    uint32 ret = SEC_OK;
    uint32 i = 0;    
    char mtd_name[32];    
    uint32 part_index = 0;        
    
    /* find which partition should be updated in mtd */
    for(i=0; i<MAX_MTD_PARTITIONS; i++) 
    {
        if(0 == mcmp(mtd_part_map[i].name,part_name,strlen(part_name)))
        {   
            part_index = i;
            break;
        }
    }

    if(MAX_MTD_PARTITIONS == i)
    {
        ret = ERR_SBOOT_UPDATE_IMG_NOT_FOUND_IN_MTD;
        goto _end;        
    }


    /* indicate which partition */
    sprintf(mtd_name, "/dev/mtd/mtd%d", part_index);
    
    fp = ASF_OPEN(mtd_name);
    if (ASF_IS_ERR(fp)) 
    {
        SMSG(true,"[%s] open fail\n",MOD);     
        ret = ERR_SBOOT_UPDATE_IMG_OPEN_FAIL;
        goto _open_fail;
    }

    /* configure file system type */
    osal_set_kernel_fs();

    /* adjust read off */
    ASF_SEEK_SET(fp,off); 

    /* read image to input buf */
    if(0 >= ASF_READ(fp,buf,size))
    {        
        ret = ERR_SBOOT_UPDATE_IMG_READ_FAIL;
        goto _read_fail;
    }

_read_fail:
    ASF_CLOSE(fp);
    osal_restore_fs();
_open_fail:
_end:
    return ret;
}
