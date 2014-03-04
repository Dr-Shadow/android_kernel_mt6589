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
//-----------------------------------------------------------------------------
#ifndef CAMERA_PIPE_MGR_H
#define CAMERA_PIPE_MGR_H
//-----------------------------------------------------------------------------
#define CAM_PIPE_MGR_DEV_NAME       "camera-pipemgr"
#define CAM_PIPE_MGR_MAGIC_NO       'p'
//-----------------------------------------------------------------------------
#define CAM_PIPE_MGR_PIPE_MASK_CAM_IO       ((unsigned long)1 << 0)
#define CAM_PIPE_MGR_PIPE_MASK_POST_PROC    ((unsigned long)1 << 1)
#define CAM_PIPE_MGR_PIPE_MASK_CDP_CAM      ((unsigned long)1 << 2)
#define CAM_PIPE_MGR_PIPE_MASK_CDP_CONCUR   ((unsigned long)1 << 3)
#define CAM_PIPE_MGR_PIPE_MASK_CDP_LINK     ((unsigned long)1 << 4)
//-----------------------------------------------------------------------------
typedef enum
{
    CAM_PIPE_MGR_SCEN_SW_NONE,
    CAM_PIPE_MGR_SCEN_SW_CAM_IDLE,
    CAM_PIPE_MGR_SCEN_SW_CAM_PRV,
    CAM_PIPE_MGR_SCEN_SW_CAM_CAP,
    CAM_PIPE_MGR_SCEN_SW_VIDEO_PRV,
    CAM_PIPE_MGR_SCEN_SW_VIDEO_REC,
    CAM_PIPE_MGR_SCEN_SW_VIDEO_VSS,
    CAM_PIPE_MGR_SCEN_SW_ZSD,
    CAM_PIPE_MGR_SCEN_SW_N3D,
}CAM_PIPE_MGR_SCEN_SW_ENUM;
//
typedef enum
{
    CAM_PIPE_MGR_SCEN_HW_NONE,
    CAM_PIPE_MGR_SCEN_HW_IC,
    CAM_PIPE_MGR_SCEN_HW_VR,
    CAM_PIPE_MGR_SCEN_HW_ZSD,
    CAM_PIPE_MGR_SCEN_HW_IP,
    CAM_PIPE_MGR_SCEN_HW_N3D,
    CAM_PIPE_MGR_SCEN_HW_VSS
}CAM_PIPE_MGR_SCEN_HW_ENUM;
//
typedef enum
{
    CAM_PIPE_MGR_DEV_CAM,
    CAM_PIPE_MGR_DEV_ATV,
    CAM_PIPE_MGR_DEV_VT
}CAM_PIPE_MGR_DEV_ENUM;
//
typedef struct
{
    unsigned long   PipeMask;
    unsigned long   Timeout;
}CAM_PIPE_MGR_LOCK_STRUCT;
//
typedef struct
{
    unsigned long   PipeMask;
}CAM_PIPE_MGR_UNLOCK_STRUCT;
//
typedef struct
{
    CAM_PIPE_MGR_SCEN_SW_ENUM   ScenSw;
    CAM_PIPE_MGR_SCEN_HW_ENUM   ScenHw;
    CAM_PIPE_MGR_DEV_ENUM       Dev;
}CAM_PIPE_MGR_MODE_STRUCT;
//
typedef struct
{
    unsigned long   PipeMask;
}CAM_PIPE_MGR_ENABLE_STRUCT;
//
typedef struct
{
    unsigned long   PipeMask;
}CAM_PIPE_MGR_DISABLE_STRUCT;
//-----------------------------------------------------------------------------
typedef enum
{
    CAM_PIPE_MGR_CMD_LOCK,
    CAM_PIPE_MGR_CMD_UNLOCK,
    CAM_PIPE_MGR_CMD_DUMP,
    CAM_PIPE_MGR_CMD_SET_MODE,
    CAM_PIPE_MGR_CMD_GET_MODE,
    CAM_PIPE_MGR_CMD_ENABLE_PIPE,
    CAM_PIPE_MGR_CMD_DISABLE_PIPE
}CAM_PIPE_MGR_CMD_ENUM;
//-----------------------------------------------------------------------------
#define CAM_PIPE_MGR_LOCK           _IOW(   CAM_PIPE_MGR_MAGIC_NO,  CAM_PIPE_MGR_CMD_LOCK,          CAM_PIPE_MGR_LOCK_STRUCT)
#define CAM_PIPE_MGR_UNLOCK         _IOW(   CAM_PIPE_MGR_MAGIC_NO,  CAM_PIPE_MGR_CMD_UNLOCK,        CAM_PIPE_MGR_UNLOCK_STRUCT)
#define CAM_PIPE_MGR_DUMP           _IO(    CAM_PIPE_MGR_MAGIC_NO,  CAM_PIPE_MGR_CMD_DUMP)
#define CAM_PIPE_MGR_SET_MODE       _IOW(   CAM_PIPE_MGR_MAGIC_NO,  CAM_PIPE_MGR_CMD_SET_MODE,      CAM_PIPE_MGR_MODE_STRUCT)
#define CAM_PIPE_MGR_GET_MODE       _IOW(   CAM_PIPE_MGR_MAGIC_NO,  CAM_PIPE_MGR_CMD_GET_MODE,      CAM_PIPE_MGR_MODE_STRUCT)
#define CAM_PIPE_MGR_ENABLE_PIPE    _IOW(   CAM_PIPE_MGR_MAGIC_NO,  CAM_PIPE_MGR_CMD_ENABLE_PIPE,   CAM_PIPE_MGR_ENABLE_STRUCT)
#define CAM_PIPE_MGR_DISABLE_PIPE   _IOW(   CAM_PIPE_MGR_MAGIC_NO,  CAM_PIPE_MGR_CMD_DISABLE_PIPE,  CAM_PIPE_MGR_DISABLE_STRUCT)
//-----------------------------------------------------------------------------
#endif
//-----------------------------------------------------------------------------

