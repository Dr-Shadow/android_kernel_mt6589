/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/
/*****************************************************************************
 *
 * Filename:
 * ---------
 *   sensor.h
 *
 * Project:
 * --------
 *   DUMA
 *
 * Description:
 * ------------
 *   Header file of Sensor driver
 *
 *
 * Author:
 * -------
 *   PC Huang (MTK02204)
 *
 *============================================================================
 *             HISTORY
 * Below this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * $Revision:$
 * $Modtime:$
 * $Log:$
 * 
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
/* SENSOR FULL SIZE */
#ifndef __SENSOR_H
#define __SENSOR_H

#define MT9V114_WRITE_ID                      (0x7A)

/* SENSOR REGISTER DEFINE */
#define MT9V114_ID_REG                          (0x0000)
 
/* sensor size */
#define MT9V114_IMAGE_SENSOR_VGA_WIDTH          (640)
#define MT9V114_IMAGE_SENSOR_VGA_HEIGHT         (480)

 /* SENSOR PIXEL/LINE NUMBERS IN ONE PERIOD */
#define MT9V114_PERIOD_PIXEL_NUMS               (652 + 147 + 1)/* Active + HST + 1 */
#define MT9V114_PERIOD_LINE_NUMS                (492 + 9)      /* Active + 9 */

#define MT9V114_BLANK_REGISTER_LIMITATION       0x3FF

/*50Hz,60Hz*/
#define MT9V114_NUM_50HZ                        (50 * 2)
#define MT9V114_NUM_60HZ                        (60 * 2)

/* FRAME RATE UNIT */
#define MT9V114_FRAME_RATE_UNIT                 (10)

/* MAX CAMERA FRAME RATE */
#define MT9V114_MAX_CAMERA_FPS                  (MT9V114_FRAME_RATE_UNIT * 30)


//export functions
UINT32 MT9V113Open(void);
UINT32 MT9V113GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution);
UINT32 MT9V113GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_INFO_STRUCT *pSensorInfo, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 MT9V113Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 MT9V113FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId, UINT8 *pFeaturePara,UINT32 *pFeatureParaLen);
UINT32 MT9V113Close(void);


#endif /* __SENSOR_H */
