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

/*******************************************************************************
*   ISP_NVRAM_PCA_STRUCT
********************************************************************************/
Slider:{
    value:{// low middle high
               0,    0,   0
    }
},
Config:{
    set:{//00
        0x00000000, 0x00000000
    }
}, 
PCA_LUTS:{
    lut_lo:{    // low
        //y_gain sat_gain hue_shift reserved
        {   0,      0,      0,      0   }, //000
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //010
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //020
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //030
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //040
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //050
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //060
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //070
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //080
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //090
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //100
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //110
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //120
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //130
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //140
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //150
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //160
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //170
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
    }, 
    lut_md:{    //  middle
        //y_gain sat_gain hue_shift reserved
        {   0,      0,      0,      0   }, //000
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //010
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //020
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //030
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //040
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //050
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //060
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //070
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //080
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //090
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //100
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //110
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //120
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //130
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //140
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //150
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //160
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //170
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
    }, 
    lut_hi:{    //  high
        //y_gain sat_gain hue_shift reserved
        {   0,      0,      0,      0   }, //000
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //010
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //020
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //030
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //040
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //050
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //060
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //070
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //080
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //090
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //100
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //110
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //120
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //130
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //140
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //150
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //160
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //170
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
        {   0,      0,      0,      0   }, //
    }
}

