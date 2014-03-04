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
 * 03 02 2012 chengxue.shen
 * [ALPS00246092] MT9T113 MIPI sensor driver check in
 * MT9T113 MIPI Sensor Driver Check in Only.
 *
 * 09 10 2010 jackie.su
 * [ALPS00002279] [Need Patch] [Volunteer Patch] ALPS.Wxx.xx Volunteer patch for
 * .10y dual sensor
 *
 * 09 02 2010 jackie.su
 * [ALPS00002279] [Need Patch] [Volunteer Patch] ALPS.Wxx.xx Volunteer patch for
 * .roll back dual sensor
 *
 * 05 25 2010 sean.cheng
 * [ALPS00001357][Meta]CameraTool 
 * .
 * Add MT9T113 YUV sensor driver support
 *
 * Mar 4 2010 mtk70508
 * [DUMA00154792] Sensor driver
 * 
 *
 * Feb 24 2010 mtk01118
 * [DUMA00025869] [Camera][YUV I/F & Query feature] check in camera code
 * 
 *
 * Aug 5 2009 mtk01051
 * [DUMA00009217] [Camera Driver] CCAP First Check In
 * 
 *
 * Apr 7 2009 mtk02204
 * [DUMA00004012] [Camera] Restructure and rename camera related custom folders and folder name of came
 * 
 *
 * Mar 26 2009 mtk02204
 * [DUMA00003515] [PC_Lint] Remove PC_Lint check warnings of camera related drivers.
 * 
 *
 * Mar 2 2009 mtk02204
 * [DUMA00001084] First Check in of MT6516 multimedia drivers
 * 
 *
 * Feb 24 2009 mtk02204
 * [DUMA00001084] First Check in of MT6516 multimedia drivers
 * 
 *
 * Dec 27 2008 MTK01813
 * DUMA_MBJ CheckIn Files
 * created by clearfsimport
 *
 * Dec 10 2008 mtk02204
 * [DUMA00001084] First Check in of MT6516 multimedia drivers
 * 
 *
 * Oct 27 2008 mtk01051
 * [DUMA00000851] Camera related drivers check in
 * Modify Copyright Header
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
/* SENSOR FULL SIZE */
#ifndef __SENSOR_H
#define __SENSOR_H


/* DUMMY NEEDS TO BE INSERTED */
/* SETUP TIME NEED TO BE INSERTED */


/* SENSOR READ/WRITE ID */
#define MT9T113_WRITE_ID_0			(0x78)    //FROM 0x78
#define MT9T113_READ_ID_0			(0x79)    //FROM 0x79

#define MT9T113_WRITE_ID_1			(0x7A)
#define MT9T113_READ_ID_1			(0x7B)



#define MT9T113_PV_PERIOD_PIXEL_NUMS			(3336)			/* Default preview line length */
#define MT9T113_PV_PERIOD_LINE_NUMS 			(857)		/* Default preview frame length */
#define MT9T113_FULL_PERIOD_PIXEL_NUMS			(6809)		/* Default full size line length */
#define MT9T113_FULL_PERIOD_LINE_NUMS			(1629)		/* Default full size frame length */


/* Sensor Exposure Line Limitation */
#define MT9T113_PV_EXPOSURE_LIMITATION      	(618)
#define MT9T113_FULL_EXPOSURE_LIMITATION    	(1236)

/* Sensor Preview Size (3M: 1024x768 or 640x480, 2M: 800x600, 1,3M: 640x512, VGA: 640x480, CIF: 352x288) */
#define MT9T113_IMAGE_SENSOR_PV_WIDTH   		(1024) //(640) //(1024)
#define MT9T113_IMAGE_SENSOR_PV_HEIGHT  		(768) //(480) //(768)
/* Sensor Capture Size (3M: 2048x1536, 2M: 1600x1200, 1.3M: 1280x1024, VGA: 640x480, CIF: 352x288) */
#define MT9T113_IMAGE_SENSOR_FULL_WIDTH     	(2048)
#define MT9T113_IMAGE_SENSOR_FULL_HEIGHT    	(1536)

/* Config the ISP grab start x & start y, Config the ISP grab width & height */
#define MT9T113_PV_GRAB_START_X 				(4)  //8
#define MT9T113_PV_GRAB_START_Y  				(2)
#define MT9T113_PV_GRAB_WIDTH					(MT9T113_IMAGE_SENSOR_PV_WIDTH - 16)  //(MT9T113_IMAGE_SENSOR_PV_WIDTH - 16)
#define MT9T113_PV_GRAB_HEIGHT					(MT9T113_IMAGE_SENSOR_PV_HEIGHT - 12) //(MT9T113_IMAGE_SENSOR_PV_HEIGHT - 12)

#define MT9T113_FULL_GRAB_START_X   			(4)
#define MT9T113_FULL_GRAB_START_Y	  			(4)
#define MT9T113_FULL_GRAB_WIDTH					(MT9T113_IMAGE_SENSOR_FULL_WIDTH - 32)
#define MT9T113_FULL_GRAB_HEIGHT				(MT9T113_IMAGE_SENSOR_FULL_HEIGHT - 24)

static kal_uint8 MT9T113_sccb_addr[] = 
{
	MT9T113_WRITE_ID_0,			/* Slave address0, Write ID */
	MT9T113_WRITE_ID_1,			/* Slave address1, Write ID */
};

typedef struct
	{
		kal_uint16	video_target_width;
		kal_uint16	video_target_height;

		kal_bool	MJPEG_encode_mode;			/* Motion JPEG */
		kal_bool	MPEG4_encode_mode;			/* MJPEG4 JPEG */
		kal_bool	FULLVIDEO_encode_mode;		/* 3G Video Call */

		kal_bool	sensor_cap_state;			/* Preview or Capture mode */
		kal_bool	is_PV_mode; 				/* PV size or Full size */
		kal_bool	is_panorama_capturing;		/* 3G Video Call */

		kal_uint32	curr_banding;				/* 50Hz/60Hz */
		kal_bool	night_mode;
	} MT9T113_OPERATION_STATE_ST;
	
	typedef struct
	{
		kal_uint8	sccb_write_id;
		kal_uint8	sccb_read_id;

		kal_uint32	pv_shutter;
		kal_uint32	pv_extra_shutter;
		kal_uint32	pv_sensor_gain;

		kal_uint32	pv_dummy_pixels;
		kal_uint32	pv_dummy_lines;
		kal_uint32	cap_dummy_pixels;
		kal_uint32	cap_dummy_lines;

		/* Preview & Capture Pixel Clock, 360 means 36.0MHz. Unit Multiple 10. */
		kal_uint32	preview_pclk;
		kal_uint32	capture_pclk;

		/* Video frame rate 300 means 30.0fps. Unit Multiple 10. */
		kal_uint32	video_frame_rate;	
	}MT9T113_SENSOR_INFO_ST;



	/* SENSOR CHIP VERSION */
//	#define MT9T113_SENSOR_ID    (0x364C)    // rev.2C




//s_add for porting
//s_add for porting
//s_add for porting

//export functions
UINT32 MT9T113Open(void);
UINT32 MT9T113GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution);
UINT32 MT9T113GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_INFO_STRUCT *pSensorInfo, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 MT9T113Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 MT9T113FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId, UINT8 *pFeaturePara,UINT32 *pFeatureParaLen);
UINT32 MT9T113Close(void);


//e_add for porting
//e_add for porting
//e_add for porting


#endif /* __SENSOR_H */
