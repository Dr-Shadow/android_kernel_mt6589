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

/******************************************************************************
 * CONSTANT DEFINITIONS                                                       
 ******************************************************************************/
#define MOD                         "ASF"

/******************************************************************************
 *  EXTERNAL VARIABLES
 ******************************************************************************/
extern uchar                        sha1sum[];

/******************************************************************************
 *  EXTERNAL FUNCTION
 ******************************************************************************/

/**************************************************************************
 *  INTERNAL UTILITES
 **************************************************************************/
void * mcpy(void *dest, const void *src, int  cnt)
{
    char *tmp = dest;
    const char *s = src;

    while (cnt--)
        *tmp++ = *s++;
    return dest;
}

int mcmp (const void *cs, const void *ct, int cnt)
{
    const uchar *su1, *su2;
    int res = 0;

    for (su1 = cs, su2 = ct; 0 < cnt; ++su1, ++su2, cnt--)
        if ((res = *su1 - *su2) != 0)
            break;
    return res;
}

void dump_buf(uchar* buf, uint32 len)
{
    uint32 i = 1;

    for (i =1; i <len+1; i++)
    {   
        if(0 != buf[i-1])
        {
            SMSG(true,"0x%x,",buf[i-1]);
            
            if(0 == i%8)        
            {
                SMSG(true,"\n");
            }
        }
    }
    
    SMSG(true,"\n");    
}

/******************************************************************************
 *  IMAGE HASH DUMP
 ******************************************************************************/
void img_hash_dump (uchar *buf, uint32 size)
{
    uint32 i = 0;

    for (i = 0 ; i < size ; i++)
    {
        if(i % 4 ==0)
        {   
            SMSG(true,"\n");
        }

        if(buf[i] < 0x10)
        {
            SMSG(true,"0x0%x, ",buf[i]);
        }
        else
        {   
            SMSG(true,"0x%x, ",buf[i]);
        }
    }
}

/******************************************************************************
 *  IMAGE HASH CALCULATION
 ******************************************************************************/
void img_hash_compute (uchar *buf, uint32 size)
{
    SEC_ASSERT(0);
}

/******************************************************************************
 *  IMAGE HASH UPDATE
 ******************************************************************************/
uint32 img_hash_update (char* part_name)
{
    uint32 ret = SEC_OK;
    
    SEC_ASSERT(0);

    return ret;
}


/******************************************************************************
 *  IMAGE HASH CHECK
 ******************************************************************************/
uint32 img_hash_check (char* part_name)
{
    uint32 ret = SEC_OK;
    
    SEC_ASSERT(0);
    
    return ret;
}


/******************************************************************************
 *  GET BUILD INFORMATION
 ******************************************************************************/
char* asf_get_build_info(void)
{
    return BUILD_TIME""BUILD_BRANCH;
}

