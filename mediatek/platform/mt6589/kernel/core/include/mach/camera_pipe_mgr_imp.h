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
#ifndef CAMERA_PIPE_MGR_IMP_H
#define CAMERA_PIPE_MGR_IMP_H
//-----------------------------------------------------------------------------
typedef unsigned long long  MUINT64;
typedef long long           MINT64;
typedef unsigned long       MUINT32;
typedef long                MINT32;
typedef unsigned char       MUINT8;
typedef char                MINT8;
typedef bool                MBOOL;
#define MTRUE               true
#define MFALSE              false
//-----------------------------------------------------------------------------
#define LOG_TAG "CamPipeMgr"
#define LOG_MSG(fmt, arg...)    xlog_printk(ANDROID_LOG_VERBOSE, LOG_TAG, "[%s]"          fmt "\r\n", __FUNCTION__,           ##arg)
#define LOG_WRN(fmt, arg...)    xlog_printk(ANDROID_LOG_VERBOSE, LOG_TAG, "[%s]WRN(%5d):" fmt "\r\n", __FUNCTION__, __LINE__, ##arg)
#define LOG_ERR(fmt, arg...)    xlog_printk(ANDROID_LOG_ERROR,   LOG_TAG, "[%s]ERR(%5d):" fmt "\r\n", __FUNCTION__, __LINE__, ##arg)
#define LOG_DMP(fmt, arg...)    xlog_printk(ANDROID_LOG_ERROR,   LOG_TAG, ""              fmt,                                ##arg)
//-----------------------------------------------------------------------------
#define CAM_PIPE_MGR_DEV_NUM        (1)
#define CAM_PIPE_MGR_DEV_MINOR_NUM  (1)
#define CAM_PIPE_MGR_DEV_NO_MINOR   (0)
#define CAM_PIPE_MGR_JIFFIES_MAX    (0xFFFFFFFF)
#define CAM_PIPE_MGR_PROC_NAME      "Default"
#define CAM_PIPE_MGR_SCEN_HW_AMOUNT (7)
//-----------------------------------------------------------------------------
#define CAM_PIPE_MGR_PIPE_NAME_LEN          (10)
#define CAM_PIPE_MGR_PIPE_NAME_CAM_IO       "CamIO"
#define CAM_PIPE_MGR_PIPE_NAME_POST_PROC    "PostProc"
#define CAM_PIPE_MGR_PIPE_NAME_CDP_CAM      "CamCDP"
#define CAM_PIPE_MGR_PIPE_NAME_CDP_CONCUR   "ConcurCDP"
#define CAM_PIPE_MGR_PIPE_NAME_CDP_LINK     "LinkCDP"
//-----------------------------------------------------------------------------
typedef enum
{
    CAM_PIPE_MGR_PIPE_CAM_IO,
    CAM_PIPE_MGR_PIPE_POST_PROC,
    CAM_PIPE_MGR_PIPE_CDP_CAM,
    CAM_PIPE_MGR_PIPE_CDP_CONCUR,
    CAM_PIPE_MGR_PIPE_CDP_LINK,
    CAM_PIPE_MGR_PIPE_AMOUNT
}CAM_PIPE_MGR_PIPE_ENUM;
//
typedef enum
{
    CAM_PIPE_MGR_STATUS_OK,
    CAM_PIPE_MGR_STATUS_FAIL,
    CAM_PIPE_MGR_STATUS_TIMEOUT,
    CAM_PIPE_MGR_STATUS_UNKNOW
}CAM_PIPE_MGR_STATUS_ENUM;
//
#define CAM_PIPE_MGR_LOCK_TABLE_NONE    (   (1<<CAM_PIPE_MGR_PIPE_CDP_LINK))
#define CAM_PIPE_MGR_LOCK_TABLE_IC      (   (1<<CAM_PIPE_MGR_PIPE_CAM_IO)| \
                                            (1<<CAM_PIPE_MGR_PIPE_POST_PROC))
#define CAM_PIPE_MGR_LOCK_TABLE_VR      (   (1<<CAM_PIPE_MGR_PIPE_CAM_IO)| \
                                            (1<<CAM_PIPE_MGR_PIPE_CDP_CAM))
#define CAM_PIPE_MGR_LOCK_TABLE_ZSD     (   (1<<CAM_PIPE_MGR_PIPE_CAM_IO)| \
                                            (1<<CAM_PIPE_MGR_PIPE_CDP_CAM))
#define CAM_PIPE_MGR_LOCK_TABLE_IP      (   (1<<CAM_PIPE_MGR_PIPE_POST_PROC))
#define CAM_PIPE_MGR_LOCK_TABLE_N3D     (   (1<<CAM_PIPE_MGR_PIPE_CAM_IO)| \
                                            (1<<CAM_PIPE_MGR_PIPE_POST_PROC))
#define CAM_PIPE_MGR_LOCK_TABLE_VSS     (   (1<<CAM_PIPE_MGR_PIPE_CAM_IO)| \
                                            (1<<CAM_PIPE_MGR_PIPE_POST_PROC))
//-----------------------------------------------------------------------------
typedef struct
{
    pid_t   Pid;
    pid_t   Tgid;
    char    ProcName[TASK_COMM_LEN];
    MUINT32 PipeMask;
    MUINT32 TimeS;
    MUINT32 TimeUS;
}CAM_PIPE_MGR_PROC_STRUCT;
//
typedef struct
{
    pid_t   Pid;
    pid_t   Tgid;
    char    ProcName[TASK_COMM_LEN];
    MUINT32 TimeS;
    MUINT32 TimeUS;
}CAM_PIPE_MGR_PIPE_STRUCT;
//
typedef struct
{
    MUINT32                     PipeMask;
    spinlock_t                  SpinLock;
    dev_t                       DevNo;
    struct cdev*                pCharDrv;
    struct class*               pClass;
    wait_queue_head_t           WaitQueueHead;
    CAM_PIPE_MGR_MODE_STRUCT    Mode;
    CAM_PIPE_MGR_PIPE_STRUCT    PipeInfo[CAM_PIPE_MGR_PIPE_AMOUNT];
    char                        PipeName[CAM_PIPE_MGR_PIPE_AMOUNT][CAM_PIPE_MGR_PIPE_NAME_LEN];
    MUINT32                     PipeLockTable[CAM_PIPE_MGR_SCEN_HW_AMOUNT];
    MUINT32                     LogMask;
}CAM_PIPE_MGR_STRUCT;
//-----------------------------------------------------------------------------
#endif
//-----------------------------------------------------------------------------

