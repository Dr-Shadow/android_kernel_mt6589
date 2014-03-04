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

/********************************************************************************************
 *     LEGAL DISCLAIMER
 *
 *     (Header of MediaTek Software/Firmware Release or Documentation)
 *
 *     BY OPENING OR USING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 *     THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE") RECEIVED
 *     FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON AN "AS-IS" BASIS
 *     ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES, EXPRESS OR IMPLIED,
 *     INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR
 *     A PARTICULAR PURPOSE OR NONINFRINGEMENT. NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY
 *     WHATSOEVER WITH RESPECT TO THE SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY,
 *     INCORPORATED IN, OR SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK
 *     ONLY TO SUCH THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
 *     NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S SPECIFICATION
 *     OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
 *
 *     BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE LIABILITY WITH
 *     RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE, AT MEDIATEK'S OPTION,
TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE, OR REFUND ANY SOFTWARE LICENSE
 *     FEES OR SERVICE CHARGE PAID BY BUYER TO MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 *     THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE WITH THE LAWS
 *     OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF LAWS PRINCIPLES.
 ************************************************************************************************/
#ifndef _DBG_AF_PARAM_H_
#define _DBG_AF_PARAM_H_

// --- AF debug info ---
#define AF_DEBUG_TAG_SIZE 1030

typedef struct
{
    AAA_DEBUG_TAG_T Tag[AF_DEBUG_TAG_SIZE];

} AF_DEBUG_INFO_T;

typedef enum
{
    IDX = 0,	// search idx
    POS,		// lens position
    VLU,		// focus value
    MINL,		// idx for min FV in inf side
    MAX,		// idx for max FV
    MINR,		// idx for min FV in macro side

    FIN_3P, 	// finish: Peak found from 3 points
    FIN_ICL,	// finish: Incline case
    FIN_BND,	// finish: Full search case

    BEST_POS,   // lens target position from curve fitting
    FOCUS_POS,  // focused lens position

    LV,         // light value
    FAIL,       // can not find peak
    FAIL_BND,   // can not find peak     (boundary)
    MAIN_THRES, // main threshold percent
    SUB_THRES,  // sub threshold percent
    THRES_OFFSET,          // spot threshold offset
    MATRIX_THRES_OFFSET,   // matrix threshold offset

    THRES_VLU_MAIN,        // main threshold value
    STATE,      // AF state
    AFMODE,     // 1: AFS, 2: AFC, 3: Macro, 4: Inf, 5: MF, 6: Cal, 7: Fullscan
    METER_MENU,    // 1: Spot, 2: Matrix
    METER_REAL,    // real meter
    WIN_X,    // AF window location  - left
    WIN_Y,    // AF window location  - top
    WIN_W,    // AF window location  - width
    WIN_H,    // AF window location  - height

    AFTIME,     // AF process time

    FD_STATUS,  // 0: no face, 1: face detected

    SCAN_START, // fullscan start position
    SCAN_STEP,  // fullscan step interval
    SCAN_NUM,

    STEP_L,     // lnfinity boundary in AF table
    STEP_H,     // macro boundary in AF table

    AF_AREA_PERCENT_W,  // AF window width vs image width percetage 
    AF_AREA_PERCENT_H,  // AF window height vs image height percetage

	FOCUSED_IDX_NEAREST,   // nearest focused window idx
    FOCUSED_IDX,           // focused window idx
    FOCUSED_IDX_FARTHEST,  // farthest focused window idx
    DOF,                   // DOF

    VERSION,               // version

    ZOOM_W,                // image width after zoom
	ZOOM_H,                // image higght after zoom
    ZOOM_X,                // image left,top position after zoom
    ZOOM_Y,                // image left,top position after zoom

    FIRST_FV,              // first focus value for scene change compare
    CHANGE_FV,             // scene change focus value
    HW_TH,                 // af hardware threshold
    FV_DC,                 // DC focus value
    MIN_TH,                // minimum threshold
    
    ZSD,                   // is ZSD mode
    ZSD_FIN_BND,           // peak found in ZSD mode
    ZSD_FAIL,              // peak not found in ZSD mode
    ZSD_MONO_VLU,          // monotous focus value in ZSD mode
    AE_STABLE,             // is AE stable
    ISO,                   // ISO value
    GSUM,                  // G sum value
    XCURR,                 // 2D interpolation Xcurr
    ZCURR,                 // 2D interpolation Zcurr
    X0,                    // 2D interpolation X0                 
    X1,                    // 2D interpolation X1
    Z0,                    // 2D interpolation Z0
    Z1,                    // 2D interpolation Z1
    Y00,                   // 2D interpolation Y00
    Y01,                   // 2D interpolation Y01
    Y10,                   // 2D interpolation Y10
    Y11,                   // 2D interpolation Y11
    FIRST_GS,              // first gsum value for scene change compare
    CHANGE_GS,             // scene change gsum value

    OVER_PATH_LENGTH

} AF_DEBUG_TAG_T;

#endif // _DBG_AF_PARAM_H_

