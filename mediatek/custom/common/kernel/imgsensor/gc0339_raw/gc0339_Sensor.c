/*****************************************************************************
 *
 * Filename:
 * ---------
 *   sensor.c
 *
 * Project:
 * --------
 *   RAW
 *
 * Description:
 * ------------
 *   Source code of Sensor driver
 *
 *
 * Author:
 * -------
 *   Leo Lee
 *
 *============================================================================
 *             HISTORY
 *------------------------------------------------------------------------------
 * $Revision:$
 * $Modtime:$
 * $Log:$
 *
 * 04 10  2013
 * First release MT6589 GC0339 driver Version 1.0
 *
 *------------------------------------------------------------------------------
 *============================================================================
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "gc0339_Sensor.h"
#include "gc0339_Camera_Sensor_para.h"
#include "gc0339_CameraCustomized.h"

#ifdef GC0339_DEBUG
#define SENSORDB printk
#else
#define SENSORDB(x,...)
#endif

#if defined(MT6577)||defined(MT6589)
static DEFINE_SPINLOCK(gc0339_drv_lock);
#endif

extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);


static GC0339_sensor_struct GC0339_sensor =
{
	.eng_info =
	{
		.SensorId = 128,
		.SensorType = CMOS_SENSOR,
		.SensorOutputDataFormat = GC0339_COLOR_FORMAT,
	},
	.Mirror = GC0339_IMAGE_H_MIRROR,
	.shutter = 0x20,  
	.gain = 0x20,
	.pclk = GC0339_PREVIEW_CLK,
	.frame_height = GC0339_PV_PERIOD_LINE_NUMS,
	.line_length = GC0339_PV_PERIOD_PIXEL_NUMS,
};


/*************************************************************************
* FUNCTION
*    GC0339_write_cmos_sensor
*
* DESCRIPTION
*    This function wirte data to CMOS sensor through I2C
*
* PARAMETERS
*    addr: the 16bit address of register
*    para: the 8bit value of register
*
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
static void GC0339_write_cmos_sensor(kal_uint8 addr, kal_uint8 para)
{
kal_uint8 out_buff[2];

    out_buff[0] = addr;
    out_buff[1] = para;

    iWriteRegI2C((u8*)out_buff , (u16)sizeof(out_buff), GC0339_WRITE_ID); 
}

/*************************************************************************
* FUNCTION
*    GC2035_read_cmos_sensor
*
* DESCRIPTION
*    This function read data from CMOS sensor through I2C.
*
* PARAMETERS
*    addr: the 16bit address of register
*
* RETURNS
*    8bit data read through I2C
*
* LOCAL AFFECTED
*
*************************************************************************/
static kal_uint8 GC0339_read_cmos_sensor(kal_uint8 addr)
{
  kal_uint8 in_buff[1] = {0xFF};
  kal_uint8 out_buff[1];
  
  out_buff[0] = addr;

    if (0 != iReadRegI2C((u8*)out_buff , (u16) sizeof(out_buff), (u8*)in_buff, (u16) sizeof(in_buff), GC0339_WRITE_ID)) {
        SENSORDB("ERROR: GC0339_read_cmos_sensor \n");
    }
  return in_buff[0];
}

/*************************************************************************
* FUNCTION
*	GC0339_SetShutter
*
* DESCRIPTION
*	This function set e-shutter of GC0339 to change exposure time.
*
* PARAMETERS
*   iShutter : exposured lines
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void GC0339_set_shutter(kal_uint16 iShutter)
{
#if defined(MT6577)||defined(MT6589)
	spin_lock(&gc0339_drv_lock);
#endif
	GC0339_sensor.shutter = iShutter;
#if defined(MT6577)||defined(MT6589)
	spin_unlock(&gc0339_drv_lock);
#endif
	//leo add for video
	if(GC0339_sensor.video_mode)
		iShutter = iShutter*3;   //Alan
	if (!iShutter) iShutter = 1; /* avoid 0 */
	
#ifdef GC0339_DRIVER_TRACE
	SENSORDB("GC0339_set_shutter iShutter = %d \n",iShutter);
#endif
	if(iShutter < 1) iShutter = 1;
	if(iShutter > 4095) iShutter = 4095;// 2^13
	//Update Shutter
	GC0339_write_cmos_sensor(0x04, (iShutter) & 0xFF);
	GC0339_write_cmos_sensor(0x03, (iShutter >> 8) & 0x0F);	
}   /*  Set_GC0339_Shutter */

/*************************************************************************
* FUNCTION
*	GC0339_SetGain
*
* DESCRIPTION
*	This function is to set global gain to sensor.
*
* PARAMETERS
*   iGain : sensor global gain(base: 0x40)
*
* RETURNS
*	the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
kal_uint16 GC0339_SetGain(kal_uint16 iGain)
{
	kal_uint16 iReg,temp;
#ifdef GC0339_DRIVER_TRACE
	SENSORDB("GC0339_SetGain iGain = %d \n",iGain);
#endif
	iReg = iGain;
	if(iReg>255)
		iReg = 255;
	//digital gain
	GC0339_write_cmos_sensor(0x51, iReg);
	return iGain;
}
/*************************************************************************
* FUNCTION
*	GC0339_NightMode
*
* DESCRIPTION
*	This function night mode of GC0339.
*
* PARAMETERS
*	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void GC0339_night_mode(kal_bool enable)
{
/*No Need to implement this function*/
#if 0 
	const kal_uint16 dummy_pixel = GC0339_sensor.line_length - GC0339_PV_PERIOD_PIXEL_NUMS;
	const kal_uint16 pv_min_fps =  enable ? GC0339_sensor.night_fps : GC0339_sensor.normal_fps;
	kal_uint16 dummy_line = GC0339_sensor.frame_height - GC0339_PV_PERIOD_LINE_NUMS;
	kal_uint16 max_exposure_lines;
	
	printk("[GC0339_night_mode]enable=%d",enable);
	if (!GC0339_sensor.video_mode) return;
	max_exposure_lines = GC0339_sensor.pclk * GC0339_FPS(1) / (pv_min_fps * GC0339_sensor.line_length);
	if (max_exposure_lines > GC0339_sensor.frame_height) /* fix max frame rate, AE table will fix min frame rate */
//	{
//	  dummy_line = max_exposure_lines - GC0339_PV_PERIOD_LINE_NUMS;
//	}
#endif
}   /*  GC0339_NightMode    */


/* write camera_para to sensor register */
static void GC0339_camera_para_to_sensor(void)
{
  kal_uint32 i;
#ifdef GC0339_DRIVER_TRACE
	 SENSORDB("GC0339_camera_para_to_sensor\n");
#endif
  for (i = 0; 0xFFFFFFFF != GC0339_sensor.eng.reg[i].Addr; i++)
  {
    GC0339_write_cmos_sensor(GC0339_sensor.eng.reg[i].Addr, GC0339_sensor.eng.reg[i].Para);
  }
  for (i = GC0339_FACTORY_START_ADDR; 0xFFFFFFFF != GC0339_sensor.eng.reg[i].Addr; i++)
  {
    GC0339_write_cmos_sensor(GC0339_sensor.eng.reg[i].Addr, GC0339_sensor.eng.reg[i].Para);
  }
  GC0339_SetGain(GC0339_sensor.gain); /* update gain */
}

/* update camera_para from sensor register */
static void GC0339_sensor_to_camera_para(void)
{
  kal_uint32 i,temp_data;
#ifdef GC0339_DRIVER_TRACE
   SENSORDB("GC0339_sensor_to_camera_para\n");
#endif
  for (i = 0; 0xFFFFFFFF != GC0339_sensor.eng.reg[i].Addr; i++)
  {
  	temp_data = GC0339_read_cmos_sensor(GC0339_sensor.eng.reg[i].Addr);
#if defined(MT6577)||defined(MT6589)
	spin_lock(&gc0339_drv_lock);
#endif
    GC0339_sensor.eng.reg[i].Para = temp_data;
#if defined(MT6577)||defined(MT6589)
	spin_unlock(&gc0339_drv_lock);
#endif
  }
  for (i = GC0339_FACTORY_START_ADDR; 0xFFFFFFFF != GC0339_sensor.eng.reg[i].Addr; i++)
  {
  	temp_data = GC0339_read_cmos_sensor(GC0339_sensor.eng.reg[i].Addr);
#if defined(MT6577)||defined(MT6589)
	spin_lock(&gc0339_drv_lock);
#endif
    GC0339_sensor.eng.reg[i].Para = temp_data;
#if defined(MT6577)||defined(MT6589)
	spin_unlock(&gc0339_drv_lock);
#endif
  }
}

/* ------------------------ Engineer mode ------------------------ */
inline static void GC0339_get_sensor_group_count(kal_int32 *sensor_count_ptr)
{
#ifdef GC0339_DRIVER_TRACE
   SENSORDB("GC0339_get_sensor_group_count\n");
#endif
  *sensor_count_ptr = GC0339_GROUP_TOTAL_NUMS;
}

inline static void GC0339_get_sensor_group_info(MSDK_SENSOR_GROUP_INFO_STRUCT *para)
{
#ifdef GC0339_DRIVER_TRACE
   SENSORDB("GC0339_get_sensor_group_info\n");
#endif
  switch (para->GroupIdx)
  {
  case GC0339_PRE_GAIN:
    sprintf(para->GroupNamePtr, "CCT");
    para->ItemCount = 5;
    break;
  case GC0339_CMMCLK_CURRENT:
    sprintf(para->GroupNamePtr, "CMMCLK Current");
    para->ItemCount = 1;
    break;
  case GC0339_FRAME_RATE_LIMITATION:
    sprintf(para->GroupNamePtr, "Frame Rate Limitation");
    para->ItemCount = 2;
    break;
  case GC0339_REGISTER_EDITOR:
    sprintf(para->GroupNamePtr, "Register Editor");
    para->ItemCount = 2;
    break;
  default:
    ASSERT(0);
  }
}

inline static void GC0339_get_sensor_item_info(MSDK_SENSOR_ITEM_INFO_STRUCT *para)
{

  const static kal_char *cct_item_name[] = {"SENSOR_BASEGAIN", "Pregain-R", "Pregain-Gr", "Pregain-Gb", "Pregain-B"};
  const static kal_char *editer_item_name[] = {"REG addr", "REG value"};
  
#ifdef GC0339_DRIVER_TRACE
	 SENSORDB("GC0339_get_sensor_item_info\n");
#endif
  switch (para->GroupIdx)
  {
  case GC0339_PRE_GAIN:
    switch (para->ItemIdx)
    {
    case GC0339_SENSOR_BASEGAIN:
    case GC0339_PRE_GAIN_R_INDEX:
    case GC0339_PRE_GAIN_Gr_INDEX:
    case GC0339_PRE_GAIN_Gb_INDEX:
    case GC0339_PRE_GAIN_B_INDEX:
      break;
    default:
      ASSERT(0);
    }
    sprintf(para->ItemNamePtr, cct_item_name[para->ItemIdx - GC0339_SENSOR_BASEGAIN]);
    para->ItemValue = GC0339_sensor.eng.cct[para->ItemIdx].Para * 1000 / BASEGAIN;
    para->IsTrueFalse = para->IsReadOnly = para->IsNeedRestart = KAL_FALSE;
    para->Min = GC0339_MIN_ANALOG_GAIN * 1000;
    para->Max = GC0339_MAX_ANALOG_GAIN * 1000;
    break;
  case GC0339_CMMCLK_CURRENT:
    switch (para->ItemIdx)
    {
    case 0:
      sprintf(para->ItemNamePtr, "Drv Cur[2,4,6,8]mA");
      switch (GC0339_sensor.eng.reg[GC0339_CMMCLK_CURRENT_INDEX].Para)
      {
      case ISP_DRIVING_2MA:
        para->ItemValue = 2;
        break;
      case ISP_DRIVING_4MA:
        para->ItemValue = 4;
        break;
      case ISP_DRIVING_6MA:
        para->ItemValue = 6;
        break;
      case ISP_DRIVING_8MA:
        para->ItemValue = 8;
        break;
      default:
        ASSERT(0);
      }
      para->IsTrueFalse = para->IsReadOnly = KAL_FALSE;
      para->IsNeedRestart = KAL_TRUE;
      para->Min = 2;
      para->Max = 8;
      break;
    default:
      ASSERT(0);
    }
    break;
  case GC0339_FRAME_RATE_LIMITATION:
    switch (para->ItemIdx)
    {
    case 0:
      sprintf(para->ItemNamePtr, "Max Exposure Lines");
      para->ItemValue = 5998;
      break;
    case 1:
      sprintf(para->ItemNamePtr, "Min Frame Rate");
      para->ItemValue = 5;
      break;
    default:
      ASSERT(0);
    }
    para->IsTrueFalse = para->IsNeedRestart = KAL_FALSE;
    para->IsReadOnly = KAL_TRUE;
    para->Min = para->Max = 0;
    break;
  case GC0339_REGISTER_EDITOR:
    switch (para->ItemIdx)
    {
    case 0:
    case 1:
      sprintf(para->ItemNamePtr, editer_item_name[para->ItemIdx]);
      para->ItemValue = 0;
      para->IsTrueFalse = para->IsReadOnly = para->IsNeedRestart = KAL_FALSE;
      para->Min = 0;
      para->Max = (para->ItemIdx == 0 ? 0xFFFF : 0xFF);
      break;
    default:
      ASSERT(0);
    }
    break;
  default:
    ASSERT(0);
  }
}

inline static kal_bool GC0339_set_sensor_item_info(MSDK_SENSOR_ITEM_INFO_STRUCT *para)
{
  kal_uint16 temp_para;
#ifdef GC0339_DRIVER_TRACE
   SENSORDB("GC0339_set_sensor_item_info\n");
#endif
  switch (para->GroupIdx)
  {
  case GC0339_PRE_GAIN:
    switch (para->ItemIdx)
    {
    case GC0339_SENSOR_BASEGAIN:
    case GC0339_PRE_GAIN_R_INDEX:
    case GC0339_PRE_GAIN_Gr_INDEX:
    case GC0339_PRE_GAIN_Gb_INDEX:
    case GC0339_PRE_GAIN_B_INDEX:
#if defined(MT6577)||defined(MT6589)
		spin_lock(&gc0339_drv_lock);
#endif
      GC0339_sensor.eng.cct[para->ItemIdx].Para = para->ItemValue * BASEGAIN / 1000;
#if defined(MT6577)||defined(MT6589)
	  spin_unlock(&gc0339_drv_lock);
#endif
      GC0339_SetGain(GC0339_sensor.gain); /* update gain */
      break;
    default:
      ASSERT(0);
    }
    break;
  case GC0339_CMMCLK_CURRENT:
    switch (para->ItemIdx)
    {
    case 0:
      switch (para->ItemValue)
      {
      case 2:
        temp_para = ISP_DRIVING_2MA;
        break;
      case 3:
      case 4:
        temp_para = ISP_DRIVING_4MA;
        break;
      case 5:
      case 6:
        temp_para = ISP_DRIVING_6MA;
        break;
      default:
        temp_para = ISP_DRIVING_8MA;
        break;
      }
      //GC0339_set_isp_driving_current(temp_para);
#if defined(MT6577)||defined(MT6589)
      spin_lock(&gc0339_drv_lock);
#endif
      GC0339_sensor.eng.reg[GC0339_CMMCLK_CURRENT_INDEX].Para = temp_para;
#if defined(MT6577)||defined(MT6589)
	  spin_unlock(&gc0339_drv_lock);
#endif
      break;
    default:
      ASSERT(0);
    }
    break;
  case GC0339_FRAME_RATE_LIMITATION:
    ASSERT(0);
    break;
  case GC0339_REGISTER_EDITOR:
    switch (para->ItemIdx)
    {
      static kal_uint32 fac_sensor_reg;
    case 0:
      if (para->ItemValue < 0 || para->ItemValue > 0xFFFF) return KAL_FALSE;
      fac_sensor_reg = para->ItemValue;
      break;
    case 1:
      if (para->ItemValue < 0 || para->ItemValue > 0xFF) return KAL_FALSE;
      GC0339_write_cmos_sensor(fac_sensor_reg, para->ItemValue);
      break;
    default:
      ASSERT(0);
    }
    break;
  default:
    ASSERT(0);
  }
  return KAL_TRUE;
}

void GC0339_SetMirrorFlip(GC0339_IMAGE_MIRROR Mirror)
{
	switch(Mirror)
	{
		case GC0339_IMAGE_NORMAL://IMAGE_V_MIRROR:
		   GC0339_write_cmos_sensor(0x14,0x20);
		    break;
		case GC0339_IMAGE_H_MIRROR://IMAGE_NORMAL:
		   GC0339_write_cmos_sensor(0x14,0x21);
		    break;
		case GC0339_IMAGE_V_MIRROR://IMAGE_HV_MIRROR:
		   GC0339_write_cmos_sensor(0x14,0x22);
		    break;
		case GC0339_IMAGE_HV_MIRROR://IMAGE_H_MIRROR:
		   GC0339_write_cmos_sensor(0x14,0x23);
		    break;
	}
}

static void GC0339_Sensor_Init(void)
{
	GC0339_write_cmos_sensor(0xfc,0x10);
	GC0339_write_cmos_sensor(0xfe,0x00);
	GC0339_write_cmos_sensor(0xf6,0x07);
	GC0339_write_cmos_sensor(0xf7,0x01);
	GC0339_write_cmos_sensor(0xf7,0x03);
	GC0339_write_cmos_sensor(0xfc,0x16);
	GC0339_write_cmos_sensor(0x06,0x00);
	GC0339_write_cmos_sensor(0x08,0x04);
	GC0339_write_cmos_sensor(0x09,0x01);
	GC0339_write_cmos_sensor(0x0a,0xe8);
	GC0339_write_cmos_sensor(0x0b,0x02);
	GC0339_write_cmos_sensor(0x0c,0x88);
	GC0339_write_cmos_sensor(0x0f,0x02);
	GC0339_write_cmos_sensor(0x14,0x20);
	GC0339_write_cmos_sensor(0x1a,0x21);
	GC0339_write_cmos_sensor(0x1b,0x88);
	GC0339_write_cmos_sensor(0x1c,0x49);
	
	GC0339_write_cmos_sensor(0x61,0x2a);
	GC0339_write_cmos_sensor(0x62,0x8c);
	GC0339_write_cmos_sensor(0x63,0x02);
	
	GC0339_write_cmos_sensor(0x32,0x00);
	GC0339_write_cmos_sensor(0x3a,0x20);
	GC0339_write_cmos_sensor(0x3b,0x20);
	GC0339_write_cmos_sensor(0x69,0x03);
	
	GC0339_write_cmos_sensor(0x60,0x86);

	GC0339_write_cmos_sensor(0x65,0x10);
	GC0339_write_cmos_sensor(0x6c,0xaa);
	GC0339_write_cmos_sensor(0x6d,0x00);
	GC0339_write_cmos_sensor(0x67,0x10);
	GC0339_write_cmos_sensor(0x4a,0x40);
	GC0339_write_cmos_sensor(0x4b,0x40);
	GC0339_write_cmos_sensor(0x4c,0x40);
	GC0339_write_cmos_sensor(0xe8,0x04);
	GC0339_write_cmos_sensor(0xe9,0xbb);
	GC0339_write_cmos_sensor(0x42,0x28);
	GC0339_write_cmos_sensor(0x47,0x10);
	GC0339_write_cmos_sensor(0x50,0x60);// 40
	GC0339_write_cmos_sensor(0x55,0xa0);// Alan
	GC0339_write_cmos_sensor(0xd0,0x00);
	GC0339_write_cmos_sensor(0xd3,0x50);
	GC0339_write_cmos_sensor(0xf6,0x05);
	GC0339_write_cmos_sensor(0x01,0x0a);// 6a
	GC0339_write_cmos_sensor(0x02,0x0c);
	GC0339_write_cmos_sensor(0x0f,0x01);// 00
	GC0339_write_cmos_sensor(0x6a,0x11);
	GC0339_write_cmos_sensor(0x71,0x01);
	GC0339_write_cmos_sensor(0x72,0x01);
	GC0339_write_cmos_sensor(0x73,0x01);
	GC0339_write_cmos_sensor(0x79,0x01);
	GC0339_write_cmos_sensor(0x7a,0x01);
	GC0339_write_cmos_sensor(0x2e,0x30);
	GC0339_write_cmos_sensor(0x2b,0x00);
	GC0339_write_cmos_sensor(0x2c,0x03);
	GC0339_write_cmos_sensor(0xd2,0x00);
	GC0339_write_cmos_sensor(0x20,0xb0);
	
	GC0339_write_cmos_sensor(0x60,0x96);
}   /*  GC0339_Sensor_Init  */

/*****************************************************************************/
/* Windows Mobile Sensor Interface */
/*****************************************************************************/
/*************************************************************************
* FUNCTION
*	GC0339Open
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/

UINT32 GC0339Open(void)
{
	kal_uint16 sensor_id=0; 

	// check if sensor ID correct
	GC0339_write_cmos_sensor(0xfc,0x10);
	GC0339_write_cmos_sensor(0xfc,0x10);

	sensor_id=GC0339_read_cmos_sensor(0x00);
#ifdef GC0339_DRIVER_TRACE
	SENSORDB("GC0339Open, sensor_id:%x \n",sensor_id);
#endif		
	if (sensor_id != GC0339_SENSOR_ID)
		return ERROR_SENSOR_CONNECT_FAIL;
	
	/* initail sequence write in  */
	GC0339_Sensor_Init();

	//GC0339_SetMirrorFlip(GC0339_sensor.Mirror);

	return ERROR_NONE;
}   /* GC0339Open  */

/*************************************************************************
* FUNCTION
*   GC0339GetSensorID
*
* DESCRIPTION
*   This function get the sensor ID 
*
* PARAMETERS
*   *sensorID : return the sensor ID 
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 GC0339GetSensorID(UINT32 *sensorID) 
{
	// check if sensor ID correct
	GC0339_write_cmos_sensor(0xfc,0x10);
	GC0339_write_cmos_sensor(0xfc,0x10);
	
	*sensorID=GC0339_read_cmos_sensor(0x00);	
#ifdef GC0339_DRIVER_TRACE
	SENSORDB("GC0339GetSensorID:%x \n",*sensorID);
#endif		
	if (*sensorID != GC0339_SENSOR_ID) {		
		*sensorID = 0xFFFFFFFF;		
		return ERROR_SENSOR_CONNECT_FAIL;
	}
   return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*	GC0339Close
*
* DESCRIPTION
*	This function is to turn off sensor module power.
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 GC0339Close(void)
{
#ifdef GC0339_DRIVER_TRACE
   SENSORDB("GC0339Close\n");
#endif
  //kdCISModulePowerOn(SensorIdx,currSensorName,0,mode_name);
//	DRV_I2CClose(GC0339hDrvI2C);
	return ERROR_NONE;
}   /* GC0339Close */

/*************************************************************************
* FUNCTION
* GC0339Preview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 GC0339Preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	
#if defined(MT6577)||defined(MT6589)
	spin_lock(&gc0339_drv_lock);	
#endif
	GC0339_sensor.pv_mode = KAL_TRUE;
	
	//GC0339_set_mirror(sensor_config_data->SensorImageMirror);
	//switch (sensor_config_data->SensorOperationMode)
	//{
	//  case MSDK_SENSOR_OPERATION_MODE_VIDEO: 	  	
	//	GC0339_sensor.video_mode = KAL_TRUE;
	//  default: /* ISP_PREVIEW_MODE */
	//	GC0339_sensor.video_mode = KAL_FALSE;
	//}
	GC0339_sensor.line_length = GC0339_PV_PERIOD_PIXEL_NUMS;
	GC0339_sensor.frame_height = GC0339_PV_PERIOD_LINE_NUMS;
	GC0339_sensor.pclk = GC0339_PREVIEW_CLK;
#if defined(MT6577)||defined(MT6589)
	spin_unlock(&gc0339_drv_lock);
#endif
	//GC0339_Write_Shutter(GC0339_sensor.shutter);
	return ERROR_NONE;
}   /*  GC0339Preview   */

/*************************************************************************
* FUNCTION
*	GC0339Capture
*
* DESCRIPTION
*	This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 GC0339Capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	kal_uint32 shutter = (kal_uint32)GC0339_sensor.shutter;
#if defined(MT6577)||defined(MT6589)
	spin_lock(&gc0339_drv_lock);
#endif
	GC0339_sensor.video_mode = KAL_FALSE;
		GC0339_sensor.pv_mode = KAL_FALSE;
#if defined(MT6577)||defined(MT6589)
		spin_unlock(&gc0339_drv_lock);
#endif
	return ERROR_NONE;
}   /* GC0339_Capture() */

UINT32 GC0339GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
	pSensorResolution->SensorFullWidth=GC0339_IMAGE_SENSOR_FULL_WIDTH;
	pSensorResolution->SensorFullHeight=GC0339_IMAGE_SENSOR_FULL_HEIGHT;
	pSensorResolution->SensorPreviewWidth=GC0339_IMAGE_SENSOR_PV_WIDTH;
	pSensorResolution->SensorPreviewHeight=GC0339_IMAGE_SENSOR_PV_HEIGHT;
	pSensorResolution->SensorVideoWidth=GC0339_IMAGE_SENSOR_VIDEO_WIDTH;
	pSensorResolution->SensorVideoHeight=GC0339_IMAGE_SENSOR_VIDEO_HEIGHT;
	return ERROR_NONE;
}	/* GC0339GetResolution() */

UINT32 GC0339GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
					  MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	pSensorInfo->SensorPreviewResolutionX=GC0339_IMAGE_SENSOR_PV_WIDTH;
	pSensorInfo->SensorPreviewResolutionY=GC0339_IMAGE_SENSOR_PV_HEIGHT;
	pSensorInfo->SensorFullResolutionX=GC0339_IMAGE_SENSOR_FULL_WIDTH;
	pSensorInfo->SensorFullResolutionY=GC0339_IMAGE_SENSOR_FULL_HEIGHT;

	pSensorInfo->SensorCameraPreviewFrameRate=30;
	pSensorInfo->SensorVideoFrameRate=30;
	pSensorInfo->SensorStillCaptureFrameRate=10;
	pSensorInfo->SensorWebCamCaptureFrameRate=15;
	pSensorInfo->SensorResetActiveHigh=TRUE; //low active
	pSensorInfo->SensorResetDelayCount=5; 
	pSensorInfo->SensorOutputDataFormat=GC0339_COLOR_FORMAT;
	pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;	
	pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	

	pSensorInfo->CaptureDelayFrame = 2; 
	pSensorInfo->PreviewDelayFrame = 1;
	pSensorInfo->VideoDelayFrame = 1;

	pSensorInfo->SensorMasterClockSwitch = 0; 
	pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA;
	pSensorInfo->AEShutDelayFrame =0;		   /* The frame of setting shutter default 0 for TG int */
	pSensorInfo->AESensorGainDelayFrame = 0;   /* The frame of setting sensor gain */
	pSensorInfo->AEISPGainDelayFrame = 4;  

	pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_MIPI;
	pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE;

	pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 20; 
	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	pSensorInfo->SensorWidthSampling = 0;
	pSensorInfo->SensorHightSampling = 0;
	pSensorInfo->SensorPacketECCOrder = 1;
	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=3;
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;
			pSensorInfo->SensorDataLatchCount= 2;
			pSensorInfo->SensorGrabStartX = GC0339_PV_X_START; 
			pSensorInfo->SensorGrabStartY = GC0339_PV_Y_START; 

		break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=3;
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;
			pSensorInfo->SensorDataLatchCount= 2;
			pSensorInfo->SensorGrabStartX = GC0339_VIDEO_X_START; 
			pSensorInfo->SensorGrabStartY = GC0339_VIDEO_Y_START; 

		break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount= 3;
			pSensorInfo->SensorClockRisingCount=0;
			pSensorInfo->SensorClockFallingCount=2;
			pSensorInfo->SensorPixelClockCount=3;
			pSensorInfo->SensorDataLatchCount=2;
			pSensorInfo->SensorGrabStartX = GC0339_FULL_X_START; 
			pSensorInfo->SensorGrabStartY = GC0339_FULL_Y_START; 
		break;
		default:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=3;
			pSensorInfo->SensorClockRisingCount=0;
			pSensorInfo->SensorClockFallingCount=2;		
			pSensorInfo->SensorPixelClockCount=3;
			pSensorInfo->SensorDataLatchCount=2;
			pSensorInfo->SensorGrabStartX = GC0339_PV_X_START; 
			pSensorInfo->SensorGrabStartY = GC0339_PV_Y_START; 
		break;
	}
	memcpy(pSensorConfigData, &GC0339_sensor.cfg_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
  return ERROR_NONE;
}	/* GC0339GetInfo() */


UINT32 GC0339Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
					  MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{

#ifdef GC2235_DRIVER_TRACE
	SENSORDB("GC0339Control ScenarioId = %d \n",ScenarioId);
#endif
	//leo add for video
	if(ScenarioId == MSDK_SCENARIO_ID_VIDEO_PREVIEW)
		GC0339_sensor.video_mode = KAL_TRUE;
	else
		GC0339_sensor.video_mode = KAL_FALSE;
	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			GC0339Preview(pImageWindow, pSensorConfigData);
		break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			GC0339Capture(pImageWindow, pSensorConfigData);
		break;		
        default:
            return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* GC0339Control() */



UINT32 GC0339SetVideoMode(UINT16 u2FrameRate)
{};

UINT32 GC0339FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
							 UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
	UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
	UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
	UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
	//UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
	//UINT32 GC0339SensorRegNumber;
	//UINT32 i;
	//PNVRAM_SENSOR_DATA_STRUCT pSensorDefaultData=(PNVRAM_SENSOR_DATA_STRUCT) pFeaturePara;
	//MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
	MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;
	//MSDK_SENSOR_GROUP_INFO_STRUCT *pSensorGroupInfo=(MSDK_SENSOR_GROUP_INFO_STRUCT *) pFeaturePara;
	//MSDK_SENSOR_ITEM_INFO_STRUCT *pSensorItemInfo=(MSDK_SENSOR_ITEM_INFO_STRUCT *) pFeaturePara;
	//MSDK_SENSOR_ENG_INFO_STRUCT	*pSensorEngInfo=(MSDK_SENSOR_ENG_INFO_STRUCT *) pFeaturePara;

	switch (FeatureId)
	{
		case SENSOR_FEATURE_GET_RESOLUTION:
			*pFeatureReturnPara16++=GC0339_IMAGE_SENSOR_FULL_WIDTH;
			*pFeatureReturnPara16=GC0339_IMAGE_SENSOR_FULL_HEIGHT;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_GET_PERIOD:	/* 3 */
			*pFeatureReturnPara16++=GC0339_sensor.line_length;
			*pFeatureReturnPara16=GC0339_sensor.frame_height;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:  /* 3 */
			*pFeatureReturnPara32 = GC0339_sensor.pclk;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_SET_ESHUTTER:	/* 4 */
			GC0339_set_shutter(*pFeatureData16);
		break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
			GC0339_night_mode((BOOL) *pFeatureData16);
		break;
		case SENSOR_FEATURE_SET_GAIN:	/* 6 */			
			GC0339_SetGain((UINT16) *pFeatureData16);
		break;
		case SENSOR_FEATURE_SET_FLASHLIGHT:
		break;
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
		break;
		case SENSOR_FEATURE_SET_REGISTER:
			GC0339_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
		break;
		case SENSOR_FEATURE_GET_REGISTER:
			pSensorRegData->RegData = GC0339_read_cmos_sensor(pSensorRegData->RegAddr);
		break;
		case SENSOR_FEATURE_SET_CCT_REGISTER:
			memcpy(&GC0339_sensor.eng.cct, pFeaturePara, sizeof(GC0339_sensor.eng.cct));
			break;
		break;
		case SENSOR_FEATURE_GET_CCT_REGISTER:	/* 12 */
			if (*pFeatureParaLen >= sizeof(GC0339_sensor.eng.cct) + sizeof(kal_uint32))
			{
			  *((kal_uint32 *)pFeaturePara++) = sizeof(GC0339_sensor.eng.cct);
			  memcpy(pFeaturePara, &GC0339_sensor.eng.cct, sizeof(GC0339_sensor.eng.cct));
			}
			break;
		case SENSOR_FEATURE_SET_ENG_REGISTER:
			memcpy(&GC0339_sensor.eng.reg, pFeaturePara, sizeof(GC0339_sensor.eng.reg));
			break;
		case SENSOR_FEATURE_GET_ENG_REGISTER:	/* 14 */
			if (*pFeatureParaLen >= sizeof(GC0339_sensor.eng.reg) + sizeof(kal_uint32))
			{
			  *((kal_uint32 *)pFeaturePara++) = sizeof(GC0339_sensor.eng.reg);
			  memcpy(pFeaturePara, &GC0339_sensor.eng.reg, sizeof(GC0339_sensor.eng.reg));
			}
		case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
			((PNVRAM_SENSOR_DATA_STRUCT)pFeaturePara)->Version = NVRAM_CAMERA_SENSOR_FILE_VERSION;
			((PNVRAM_SENSOR_DATA_STRUCT)pFeaturePara)->SensorId = GC0339_SENSOR_ID;
			memcpy(((PNVRAM_SENSOR_DATA_STRUCT)pFeaturePara)->SensorEngReg, &GC0339_sensor.eng.reg, sizeof(GC0339_sensor.eng.reg));
			memcpy(((PNVRAM_SENSOR_DATA_STRUCT)pFeaturePara)->SensorCCTReg, &GC0339_sensor.eng.cct, sizeof(GC0339_sensor.eng.cct));
			*pFeatureParaLen = sizeof(NVRAM_SENSOR_DATA_STRUCT);
			break;
		case SENSOR_FEATURE_GET_CONFIG_PARA:
			memcpy(pFeaturePara, &GC0339_sensor.cfg_data, sizeof(GC0339_sensor.cfg_data));
			*pFeatureParaLen = sizeof(GC0339_sensor.cfg_data);
			break;
		case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
		     GC0339_camera_para_to_sensor();
			break;
		case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
			GC0339_sensor_to_camera_para();
			break;							
		case SENSOR_FEATURE_GET_GROUP_COUNT:
			GC0339_get_sensor_group_count((kal_uint32 *)pFeaturePara);
			*pFeatureParaLen = 4;
			break;
		case SENSOR_FEATURE_GET_GROUP_INFO:
			GC0339_get_sensor_group_info((MSDK_SENSOR_GROUP_INFO_STRUCT *)pFeaturePara);
			*pFeatureParaLen = sizeof(MSDK_SENSOR_GROUP_INFO_STRUCT);
			break;
		case SENSOR_FEATURE_GET_ITEM_INFO:
			GC0339_get_sensor_item_info((MSDK_SENSOR_ITEM_INFO_STRUCT *)pFeaturePara);
			*pFeatureParaLen = sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
			break;
		case SENSOR_FEATURE_SET_ITEM_INFO:
			GC0339_set_sensor_item_info((MSDK_SENSOR_ITEM_INFO_STRUCT *)pFeaturePara);
			*pFeatureParaLen = sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
			break;
		case SENSOR_FEATURE_GET_ENG_INFO:
			memcpy(pFeaturePara, &GC0339_sensor.eng_info, sizeof(GC0339_sensor.eng_info));
			*pFeatureParaLen = sizeof(GC0339_sensor.eng_info);
			break;
		case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
			// get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
			// if EEPROM does not exist in camera module.
			*pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
			*pFeatureParaLen=4;
		break;
		case SENSOR_FEATURE_SET_VIDEO_MODE:
		   //GC0339SetVideoMode(*pFeatureData16);
			break; 
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			GC0339GetSensorID(pFeatureReturnPara32); 
			break; 
		case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
			break;
		default:
			break;
	}
	return ERROR_NONE;
}	/* GC0339FeatureControl() */
SENSOR_FUNCTION_STRUCT	SensorFuncGC0339=
{
	GC0339Open,
	GC0339GetInfo,
	GC0339GetResolution,
	GC0339FeatureControl,
	GC0339Control,
	GC0339Close
};

UINT32 GC0339SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&SensorFuncGC0339;

	return ERROR_NONE;
}	/* SensorInit() */
