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


#ifndef SEC_PAL_H
#define SEC_PAL_H

#include "sec_osal_light.h"
#include <mach/sec_osal.h>  

#ifndef FALSE
#define FALSE 0
#endif
#ifndef TRUE
#define TRUE 1
#endif

typedef unsigned int uint32;
typedef unsigned char uchar;

/******************************************************************************
 *  DEBUG
 ******************************************************************************/
/* Debug message event */
#define DBG_EVT_NONE        (0)       /* No event */
#define DBG_EVT_CMD         (1 << 0)  /* SEC CMD related event */
#define DBG_EVT_FUNC        (1 << 1)  /* SEC function event */
#define DBG_EVT_INFO        (1 << 2)  /* SEC information event */
#define DBG_EVT_WRN         (1 << 30) /* Warning event */
#define DBG_EVT_ERR         (1 << 31) /* Error event */
#define DBG_EVT_ALL         (0xffffffff)

#define DBG_EVT_MASK        (DBG_EVT_ALL)

#ifdef SEC_DEBUG
#define MSG(evt, fmt, args...) \
do {    \
    if ((DBG_EVT_##evt) & DBG_EVT_MASK) { \
        secpal_print(fmt, ##args); \
    }   \
} while(0)
#else
#define MSG(evt, fmt, args...)  do{}while(0)
#endif

#define MSG_FUNC(mod) MSG(FUNC, "[%s] %s\n", mod, __FUNCTION__)

/******************************************************************************
 *  EXPORT FUNCTION
 ******************************************************************************/
extern void *secpal_memcpy(void *dst, void *src, unsigned int size);
extern void *secpal_memset(void *s, int c, unsigned int count);
extern int secpal_print(const char *s, ...);
extern int secpal_get_uuid(unsigned int *uuid);
extern void *mcpy(void *dest, const void *src, int  count);
extern int mcmp (const void *cs, const void *ct, int count);
extern void dump_buf(unsigned char* buf, unsigned int len);

#endif /* end of SEC_LIB_H */
