/************************************************************************************
** File: - ms2ryuv_Sensor.h
** OPPO_R819
** Copyright (C), 2008-2012, OPPO Mobile Comm Corp., Ltd
** 
** Description: 
**      The header file of MS2R
** 
** Version: 1.0
** Date created: 14:01:46,04/09/2012
** Author: LiuBin@MtkCamera
** 
** --------------------------- Revision History: --------------------------------
** 	<author>	<data>			<desc>
** 
************************************************************************************/

#ifndef __MS2RYUVSENSOR_H
#define __MS2RYUVSENSOR_H

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <asm/atomic.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"

#include "ms2r_bin.h"

typedef enum
{
	INT_STATUS_MODE,
	INT_STATUS_AF,
	INT_STATUS_ZOOM,
	INT_STATUS_CAPTURE,
	INT_STATUS_FRAMESYNC,
	INT_STATUS_FD,
	INT_STATUS_INIT,
	INT_STATUS_SOUND,
	INT_STATUS_MAX
} MS2R_INTERRUPT_TYPE;

typedef enum
{
	SYS_MODE_POWEROFF,
	SYS_MODE_PRE_INITIALIZATION,
	SYS_MODE_INITIALIZATION,
	SYS_MODE_PARAMETER_SETTING,
	SYS_MODE_MONITOR,
	SYS_MODE_VIDEO_MONITOR,
	SYS_MODE_PRE_CAPTURE,
	SYS_MODE_SINGLE_CAPTURE,
	SYS_MODE_AUTO_FOCUS,
	SYS_MODE_FACE_DETECTION,
	SYS_MODE_MAX
}MS2R_SYS_MODE;

typedef enum
{
	FOCUS_MODE_NORMAL,
	FOCUS_MODE_MACRO,
	FOCUS_MODE_FULL,
	FOCUS_MODE_MAX
}MS2R_FOCUS_MODE;

typedef enum
{
	MS2R_AE_MODE_AUTO,
	MS2R_AE_MODE_TOUCH,
	MS2R_AE_MODE_FACE,
	MS2R_AE_MODE_MAX
}MS2R_AE_MODE;

#define MS2R_PV_WIDTH					(2560)
#define MS2R_PV_HEIGHT					(1920)

#define MS2R_FULL_WIDTH					(3264)
#define MS2R_FULL_HEIGHT				(2448)

#define MS2R_VIDEO_WIDTH				(1920)
#define MS2R_VIDEO_HEIGHT				(1080)

#define MS2R_REAL_PV_WIDTH				(MS2R_PV_WIDTH-16)
#define MS2R_REAL_PV_HEIGHT				(MS2R_PV_HEIGHT-12)

#define MS2R_REAL_VIDEO_WIDTH			(MS2R_VIDEO_WIDTH-16)
#define MS2R_REAL_VIDEO_HEIGHT			(MS2R_VIDEO_HEIGHT-12)

#define MS2R_REAL_CAP_WIDTH				(MS2R_FULL_WIDTH - 16)
#define MS2R_REAL_CAP_HEIGHT			(MS2R_FULL_HEIGHT - 12)

#define MS2R_PREVIEW_START_X			(7)
#define MS2R_PREVIEW_START_Y			(4)  // The value must bigger or equal than 1.

#define MS2R_VIDEO_START_X				(7)
#define MS2R_VIDEO_START_Y				(4)

#define MS2R_CAPTURE_START_X			(7)
#define MS2R_CAPTURE_START_Y			(4)

/*--------- capture raw data config start ----------*/
#define MS2R_CAP_RAW_PREVIEW_START_X	(8)
#define MS2R_CAP_RAW_PREVIEW_START_Y	(4)  // The value must bigger or equal than 1.

#define MS2R_CAP_RAW_VIDEO_START_X		(8)
#define MS2R_CAP_RAW_VIDEO_START_Y		(4)

#define MS2R_CAP_RAW_DATA_START_X		(0)
#define MS2R_CAP_RAW_DATA_START_Y		(0)

#define MS2R_CAP_RAW_DATA_WIDTH			(4160/2)
#define MS2R_CAP_RAW_DATA_HEIGHT		(2466)
/*--------- capture raw data config end ----------*/

#define MS2R_WRITE_ID 					(0x3E)
#define MS2R_READ_ID					(0x3F)

#define MS2R_SENSOR_ID					(0x0F16)

/*Command ID*/
#define COMMAND_ID_READ_CATEGORY_PARAMETER		(0x01)
#define COMMAND_ID_WRITE_CATEGORY_PARAMETER		(0x02)
#define COMMAND_ID_READ_MEMORY_8BIT_ACCESS		(0x03)
#define COMMAND_ID_WRITE_MEMORY_8BIT_ACCESS		(0x04)

/*MAX data size send by I2C*/
#define MAX_MEMORY_DATA_SIZE					(255-8) //255-8

/*SIO loader program start address*/
#define SIO_LOADER_PROGRAM_START_ADDR			(0x01000100)

/*Camera FirmWare top address*/
#define CAMERA_FIRMWARE_TOP_ADDR				(0x01100000)

/*Max data size send by SPI*/
#define MAX_SPI_TRANSFER_SIZE					RS_MS2_BIN_SIZE//(512*32)

/* SPI GPIO pin and mode, refer to the dct tool set*/
/*SPI_CS*/
#ifndef GPIO_SPI_CS_PIN
#define GPIO_SPI_CS_PIN							GPIO134
#endif
#ifndef GPIO_SPI_CS_PIN_M_GPIO
#define GPIO_SPI_CS_PIN_M_GPIO					GPIO_MODE_00
#endif
/*SPI_MO*/
#ifndef GPIO_SPI_MOSI_PIN						
#define GPIO_SPI_MOSI_PIN						GPIO135
#endif
#ifndef	GPIO_SPI_MOSI_PIN_M_SPI1_MO
#define GPIO_SPI_MOSI_PIN_M_SPI1_MO				GPIO_MODE_07
#endif
/*SPI CLK*/
#ifndef GPIO_SPI_SCK_PIN
#define GPIO_SPI_SCK_PIN						GPIO137
#endif
#ifndef GPIO_SPI_SCK_PIN_M_CLK
#define GPIO_SPI_SCK_PIN_M_CLK					GPIO_MODE_07
#endif

/*ms2r pin address*/
#define MS2R_ADDRESS_OF_PIN1					(0x90001200)
#define MS2R_ADDRESS_OF_PIN2					(0x90001000)
#define MS2R_ADDRESS_OF_PIN3					(0x90001100)

/* set MS2R pin by using "MS2R memory write" command */
#define MS2R_PIN_DATA_SIZE						(64)
char data_pin1[MS2R_PIN_DATA_SIZE] = 
{
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07,
	0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0xFF,
};

char data_pin2[MS2R_PIN_DATA_SIZE] = 
{
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

char data_pin3[MS2R_PIN_DATA_SIZE] = 
{
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C,
	0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x10,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

#ifdef OPPO_R819
//feng.hu@mtk camera team modified for ms2r continuous af
static MUINT32 mPreFv;
static MUINT32 uPreWB;
static MUINT32 uPreBright;
bool mCAFflag;
bool mmIsCAFin;
INT32 mdelayDetectCnt;
INT32 mdelayAFCnt;
INT32 mdelayFrameCnt;
INT32 m3AFrameCnt;
INT32 m3ASingleFrameCnt;
bool m3ASingleAFLock;

enum{
    THR_CASE = 0,
    STABLE_CASE
};
#define WDV_CAF_LOW_THR 2000 
#define ANTI_WDV_SHAKE_DELAY 10 //hufeng for test 60
#define DO_AF_DELAY 2
#endif
//export functions
UINT32 MS2ROpen(void);
UINT32 MS2RGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution);
UINT32 MS2RGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_INFO_STRUCT *pSensorInfo, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 MS2RControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
UINT32 MS2RFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId, UINT8 *pFeaturePara,UINT32 *pFeatureParaLen);
UINT32 MS2RClose(void);

//e_add for porting
//e_add for porting
//e_add for porting

#endif /* __MS2RYUVSENSOR_H */

