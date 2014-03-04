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

#ifndef _DBG_CAM_SENSOR_PARAM_H
#define _DBG_CAM_SENSOR_PARAM_H

// Sensor debug info
#define SENSOR_DEBUG_TAG_SIZE     20
#define SENSOR_DEBUG_TAG_VERSION  0


typedef struct DEBUG_SENSOR_INFO_S
{
    DEBUG_CAM_TAG_T Tag[SENSOR_DEBUG_TAG_SIZE];
} DEBUG_SENSOR_INFO_T;


//Common Parameter Structure
typedef enum
{
    SENSOR_TAG_VERSION = 0,
    SENSOR1_TAG_COLORORDER, //0:B , 1:Gb, 2:Gr, 3:R, 4:UYVY, 5:VYUY, 6:YUYV, 7:YVYU
    SENSOR1_TAG_DATATYPE, //0:RAW, 1:YUV, 2:YCBCR, 3:RGB565, 4:RGB888, 5:JPEG
    SENSOR1_TAG_HARDWARE_INTERFACE,//0: parallel, 1:MIPI
    SENSOR1_TAG_GRAB_START_X,
    SENSOR1_TAG_GRAB_START_Y,
    SENSOR1_TAG_GRAB_WIDTH,
    SENSOR1_TAG_GRAB_HEIGHT,
    SENSOR2_TAG_COLORORDER, //0:B , 1:Gb, 2:Gr, 3:R, 4:UYVY, 5:VYUY, 6:YUYV, 7:YVYU
    SENSOR2_TAG_DATATYPE, //0:RAW, 1:YUV, 2:YCBCR, 3:RGB565, 4:RGB888, 5:JPEG
    SENSOR2_TAG_HARDWARE_INTERFACE,//0: parallel, 1:MIPI
    SENSOR2_TAG_GRAB_START_X,
    SENSOR2_TAG_GRAB_START_Y,
    SENSOR2_TAG_GRAB_WIDTH,
    SENSOR2_TAG_GRAB_HEIGHT,
    
    /* TBD */

}DEBUG_SENSOR_TAG_T;

#endif //_DBG_CAM_SENSOR_PARAM_H