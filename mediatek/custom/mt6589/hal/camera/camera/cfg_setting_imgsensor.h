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
#ifndef _CFG_SETTING_IMGSENSOR_H_
#define _CFG_SETTING_IMGSENSOR_H_


/*******************************************************************************
* Image Sensor Orientation
*******************************************************************************/
SensorOrientation_T const&
getSensorOrientation()
{
    static SensorOrientation_T const inst = {
#if 0
	u4Degree_0  : 90,   //  main sensor in degree (0, 90, 180, 270)
	u4Degree_1  : 0,   //  main sensor in degree (0, 90, 180, 270)
	u4Degree_2  : 90,   //  main sensor in degree (0, 90, 180, 270)
#else
		
#if defined(MAIN_SENSOR_ROTATE_DEGREE_0)
        u4Degree_0  : 0,   //  main sensor in degree (0, 90, 180, 270)
#elif defined(MAIN_SENSOR_ROTATE_DEGREE_90)
        u4Degree_0  : 90,   //  main sensor in degree (0, 90, 180, 270)
#elif defined(MAIN_SENSOR_ROTATE_DEGREE_180)
        u4Degree_0  : 180,   //  main sensor in degree (0, 90, 180, 270)
#elif defined(MAIN_SENSOR_ROTATE_DEGREE_270)
        u4Degree_0  : 270,   //  main sensor in degree (0, 90, 180, 270)
#else
        u4Degree_0  : 90,   //  main sensor in degree (0, 90, 180, 270)
#endif

#if defined(SUB_SENSOR_ROTATE_DEGREE_0)
        u4Degree_1  : 0,   //  main sensor in degree (0, 90, 180, 270)
#elif defined(SUB_SENSOR_ROTATE_DEGREE_90)
        u4Degree_1  : 90,   //  main sensor in degree (0, 90, 180, 270)
#elif defined(SUB_SENSOR_ROTATE_DEGREE_180)
        u4Degree_1  : 180,   //  main sensor in degree (0, 90, 180, 270)
#elif defined(SUB_SENSOR_ROTATE_DEGREE_270)
        u4Degree_1  : 270,   //  main sensor in degree (0, 90, 180, 270)
#else
        u4Degree_1  : 90,   //  main sensor in degree (0, 90, 180, 270)
#endif

#if defined(MAIN2_SENSOR_ROTATE_DEGREE_0)
        u4Degree_2  : 0,   //  main sensor in degree (0, 90, 180, 270)
#elif defined(MAIN2_SENSOR_ROTATE_DEGREE_90)
        u4Degree_2  : 90,   //  main sensor in degree (0, 90, 180, 270)
#elif defined(MAIN2_SENSOR_ROTATE_DEGREE_180)
        u4Degree_2  : 180,   //  main sensor in degree (0, 90, 180, 270)
#elif defined(MAIN2_SENSOR_ROTATE_DEGREE_270)
        u4Degree_2  : 270,   //  main sensor in degree (0, 90, 180, 270)
#else
        u4Degree_2  : 90,   //  main sensor in degree (0, 90, 180, 270)
#endif

#endif	

    };
    return inst;

}


/*******************************************************************************
* Return fake orientation for front sensor or not
*       MTRUE: return 90 for front sensor in degree 0, 
*              return 270 for front sensor in degree 180.
*       MFALSE: not return fake orientation.
*******************************************************************************/
MBOOL isRetFakeSubOrientation()  
{
	return MFALSE;
}


/*******************************************************************************
* Sensor Input Data Bit Order
*   Return:
*       0   : raw data input [9:2]
*       1   : raw data input [7:0]
*       -1  : error
*******************************************************************************/
MINT32
getSensorInputDataBitOrder(EDevId const eDevId)
{
    switch  (eDevId)
    {
    case eDevId_ImgSensor0:
        return  0;
    case eDevId_ImgSensor1:
        return  0;
    case eDevId_ImgSensor2:
        return  0;        
    default:
        break;
    }
    return  -1;
}


/*******************************************************************************
* Sensor Pixel Clock Inverse in PAD side.
*   Return:
*       0   : no inverse
*       1   : inverse
*       -1  : error
*******************************************************************************/
MINT32
getSensorPadPclkInv(EDevId const eDevId)
{
    switch  (eDevId)
    {
    case eDevId_ImgSensor0:
        return  0;
    case eDevId_ImgSensor1:
        return  0;
    case eDevId_ImgSensor2:
        return  0;        
    default:
        break;
    }
    return  -1;
}

/*******************************************************************************
* Sensor Placement Facing Direction
*   Return:
*       0   : Back side  
*       1   : Front side (LCD side)
*       -1  : error
*******************************************************************************/
MINT32  
getSensorFacingDirection(EDevId const eDevId)
{
    switch  (eDevId)
    {
    case eDevId_ImgSensor0:
        return  0;
    case eDevId_ImgSensor1:
        return  1;
    case eDevId_ImgSensor2:
        return  0;
    default:
        break;
    }
    return  -1;
}

#endif //  _CFG_SETTING_IMGSENSOR_H_

