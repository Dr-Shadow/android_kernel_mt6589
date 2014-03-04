/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
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

/*
**
** Copyright 2008, The Android Open Source Project
**
** Licensed under the Apache License, Version 2.0 (the "License");
** you may not use this file except in compliance with the License.
** You may obtain a copy of the License at
**
**     http://www.apache.org/licenses/LICENSE-2.0
**
** Unless required by applicable law or agreed to in writing, software
** distributed under the License is distributed on an "AS IS" BASIS,
** WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
** See the License for the specific language governing permissions and
** limitations under the License.
*/

#ifndef _DBG_EIS_PARAM_H
#define _DBG_EIS_PARAM_H

typedef struct
{
    MUINT32 u4FieldID;
    MUINT32 u4FieldValue;
} DEBUG_EIS_T;


// dbgEIS debug info
#define EISTAG(module_id, tag, line_keep)   \
( (MINT32)                                  \
  ((MUINT32)(0x00000000) |                  \
   (MUINT32)((module_id & 0xff) << 24) |    \
   (MUINT32)((line_keep & 0x01) << 23) |    \
   (MUINT32)(tag & 0xffff))                 \
)

#define MODULE_NUM(total_module, tag_module)      \
((MINT32)                                         \
 ((MUINT32)(0x00000000) |                         \
  (MUINT32)((total_module & 0xff) << 16) |        \
  (MUINT32)(tag_module & 0xff))                   \
)
//
#define DEBUG_EIS_MID               0x4001
//
#define DEBUF_EIS_TOT_MODULE_NUM    1
#define DEBUF_EIS_TAG_MODULE_NUM    1
//
#define DEBUG_EIS_KEYID 0xF1F3F5F7
//

// EIS Parameter Structure
typedef enum
{
    EIS_TAG_VERSION = 0,

    /* TBD */

}DEBUG_EIS_TAG_T;

//
typedef struct DEBUG_EIS_INFO_S
{
    struct Header
    {
        MUINT32  u4KeyID;
        MUINT32  u4ModuleCount;
        MUINT32  u4DbgEISInfoOffset;
    } hdr;

    DEBUG_EIS_TAG_T    rDbgEISInfo;

} DEBUG_EIS_INFO_T;


#endif  //_DBG_EIS_PARAM_H

