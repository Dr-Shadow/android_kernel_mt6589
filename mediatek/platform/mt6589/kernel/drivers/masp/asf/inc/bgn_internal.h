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

#ifndef _BIGNUM_INTERNAL_H
#define _BIGNUM_INTERNAL_H

#include "bgn_export.h"

/**************************************************************************
 *  INTERNAL DEFINIITION
 **************************************************************************/
#define ciL    ((int) sizeof(unsigned long))
#define biL    (ciL << 3)               
#define biH    (ciL << 2) 

#define B_T_L(i)  (((i) + biL - 1) / biL)
#define C_T_L(i) (((i) + ciL - 1) / ciL)

/**************************************************************************
 *  INTERNAL FUNCTIONS
 **************************************************************************/
void bgn_init( bgn *P_X );
void bgn_free( bgn *P_X );
int bgn_grow( bgn *P_X, int nbl );
int bgn_copy( bgn *P_X, const bgn *P_Y );
void bgn_swap( bgn *P_X, bgn *P_Y );
int bgn_lset( bgn *P_X, int z );
int bgn_lsb( const bgn *P_X );
int bgn_msb( const bgn *P_X );
int bgn_size( const bgn *P_X );
int bgn_shift_l( bgn *P_X, int count );
int bgn_shift_r( bgn *P_X, int count );
int bgn_cmp_abs( const bgn *P_X, const bgn *P_Y );
int bgn_cmp_num( const bgn *P_X, const bgn *P_Y );
int bgn_cmp_int( const bgn *P_X, int z );
int bgn_add_abs( bgn *P_X, const bgn *P_A, const bgn *P_B );
int bgn_sub_abs( bgn *P_X, const bgn *P_A, const bgn *P_B );
int bgn_add_bgn( bgn *P_X, const bgn *P_A, const bgn *P_B );
int bgn_sub_bgn( bgn *P_X, const bgn *P_A, const bgn *P_B );
int bgn_add_int( bgn *P_X, const bgn *P_A, int b );
int bgn_sub_int( bgn *P_X, const bgn *P_A, int b );
int bgn_mul_bgn( bgn *P_X, const bgn *P_A, const bgn *B );
int bgn_mul_int( bgn *P_X, const bgn *P_A, unsigned long b );
int bgn_div_bgn( bgn *P_Q, bgn *P_R, const bgn *P_A, const bgn *P_B );
int bgn_div_int( bgn *P_Q, bgn *P_R, const bgn *P_A, int b );
int bgn_mod_bgn( bgn *P_R, const bgn *P_A, const bgn *P_B );
int bgn_mod_int( unsigned long *r, const bgn *P_A, int b );
int bgn_exp_mod( bgn *P_X, const bgn *P_E, const bgn *P_N, bgn *_RR );
int bgn_inv_mod( bgn *P_X, const bgn *P_A, const bgn *P_N );
int bgn_is_prime( bgn *P_X, int (*f_rng)(void *), void *p_rng );

void montg_init( unsigned long *mm, const bgn *P_N );
void montg_mul( bgn *P_A, const bgn *P_B, const bgn *P_N, unsigned long mm, const bgn *P_T );
void montg_red( bgn *P_A, const bgn *P_N, unsigned long mm, const bgn *P_T );
void bgn_sub_hlp( int n, unsigned long *s, unsigned long *d );
void bgn_mul_hlp( int i, unsigned long *s, unsigned long *d, unsigned long b );

#endif
