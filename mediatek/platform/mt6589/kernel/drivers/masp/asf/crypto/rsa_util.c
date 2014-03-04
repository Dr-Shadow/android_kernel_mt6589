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
#include "sec_cust_struct.h"
#include "bgn_internal.h"
#include "bgn_asm.h"

#define MOD "BGN"

/**************************************************************************
 *  TYPEDEF
 **************************************************************************/
typedef unsigned int uint32;
typedef unsigned char uchar;


/**************************************************************************
 *  FUNCTIONS
 **************************************************************************/
void montg_init( ulong *mm, const bgn *P_N )
{
    ulong x, m0 = P_N->p[0];

    x  = m0;
    x += ( ( m0 + 2 ) & 4 ) << 1;
    x *= ( 2 - ( m0 * x ) );

    if( biL >= 16 ) 
    {   
        x *= ( 2 - ( m0 * x ) );
    }
    
    if( biL >= 32 )
    {
        x *= ( 2 - ( m0 * x ) );
    }
    
    if( biL >= 64 ) 
    {
        x *= ( 2 - ( m0 * x ) );
    }

    *mm = ~x + 1;
}

void montg_mul( bgn *P_A, const bgn *P_B, const bgn *P_N, ulong mm, const bgn *P_T )
{
    int i, n, m;
    ulong u0, u1, *d;

    memset( P_T->p, 0, P_T->n * ciL );

    d = P_T->p;
    n = P_N->n;
    m = ( P_B->n < n ) ? P_B->n : n;

    for( i = 0; i < n; i++ )
    {
        u0 = P_A->p[i];
        u1 = ( d[0] + u0 * P_B->p[0] ) * mm;

        bgn_mul_hlp( m, P_B->p, d, u0 );
        bgn_mul_hlp( n, P_N->p, d, u1 );

        *d++ = u0; d[n + 1] = 0;
    }

    memcpy( P_A->p, d, (n + 1) * ciL );

    if( bgn_cmp_abs( P_A, P_N ) >= 0 )
    {
        bgn_sub_hlp( n, P_N->p, P_A->p );
    }
    else
    {
        bgn_sub_hlp( n, P_A->p, P_T->p );
    }
}

void montg_red( bgn *P_A, const bgn *P_N, ulong mm, const bgn *P_T )
{
    ulong z = 1;
    bgn U;

    U.n = U.s = z;
    U.p = &z;

    montg_mul( P_A, &U, P_N, mm, P_T );
}
