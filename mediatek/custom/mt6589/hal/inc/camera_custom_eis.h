/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 *
 * MediaTek Inc. (C) 2010. All rights reserved.
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
#ifndef _EIS_CONFIG_H_
#define _EIS_CONFIG_H_

typedef unsigned int MUINT32;

typedef enum
{
    CUSTOMER_EIS_SENSI_LEVEL_HIGH = 0,
    CUSTOMER_EIS_SENSI_LEVEL_NORMAL = 1,
    CUSTOMER_EIS_SENSI_LEVEL_ADVTUNE = 2
}Customize_EIS_SENSI;

typedef enum
{
    ABSOLUTE_HIST,
    SMOOTH_HIST    
} Customize_EIS_VOTE_METHOD_ENUM;


typedef struct
{
    Customize_EIS_SENSI sensitivity;
    MUINT32 filter_small_motion;
    MUINT32 new_tru_th; // 0~100
    MUINT32 vot_th;      // 1~16
    MUINT32 votb_enlarge_size;  // 0~1280
    MUINT32 min_s_th; // 10~100
    MUINT32 vec_th;   // 0~11   should be even
    MUINT32 spr_offset; //0 ~ MarginX/2
    MUINT32 spr_gain1; // 0~127
    MUINT32 spr_gain2; // 0~127
    MUINT32 gmv_pan_array[4];           //0~5
    MUINT32 gmv_sm_array[4];            //0~5
    MUINT32 cmv_pan_array[4];           //0~5
    MUINT32 cmv_sm_array[4];            //0~5
    
    Customize_EIS_VOTE_METHOD_ENUM vot_his_method; //0 or 1
    MUINT32 smooth_his_step; // 2~6
    MUINT32 eis_debug;
}EIS_Customize_Para_t;

void get_EIS_CustomizeData(EIS_Customize_Para_t *a_pDataOut);
	
#endif /* _EIS_CONFIG_H */

