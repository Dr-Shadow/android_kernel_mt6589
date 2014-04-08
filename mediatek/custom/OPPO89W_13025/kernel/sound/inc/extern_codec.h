/************************************************************************************
** File: - mediatek\custom\OPPO89T_12083\kernel\sound\inc\extern_codec.h
** VENDOR_EDIT
** Copyright (C), 2008-2012, OPPO Mobile Comm Corp., Ltd
** 
** Description: 
**      extern codec
** 
** Version: 1.0
** Date created: 10:31:59,18/12/2012
** Author: xiang.fei
** 
** --------------------------- Revision History: --------------------------------
** 	<author>	<data>			<desc>
** 
************************************************************************************/


#ifndef _EXTERN_CODEC_H
#define _EXTERN_CODEC_H

/*****************************************************
*       Define
*****************************************************/
#define EXTERN_CEDOC_DRIVER_DEBUG
#ifdef EXTERN_CEDOC_DRIVER_DEBUG
#define AUDIO_DBG(format, args...) printk(format,##args)
#else
#define AUDIO_DBG 
#endif

#define EXTERN_CODEC_DRVNAME "ExternCodec"
#define EXTERN_CODEC_I2C_DRVNAME "ExternCodec_I2C"
#define EXTERN_CODEC_DEVICE_ID 0x34

#define FREQ_32K 32000
#define FREQ_44K 44100
#define FREQ_48K 48000

/*****************************************************
*       Function declare
*****************************************************/
static void Extern_Codec_Power(bool Enable);
static void Extern_Codec_Init(void);
static void Extern_Codec_FreeClk(kal_bool b_OnOff);
void Extern_Codec_SetPlayFreq(kal_uint16 uiFreq);
void Extern_Codec_DAC_Input_Path(kal_bool b_OnOff);
void Extern_Codec_IN2_Input_Path(kal_bool b_OnOff);
void Extern_Codec_Hp_Output_Path(kal_bool b_OnOff);
void Extern_Codec_Spk_Output_Path(kal_bool b_OnOff);
static void Extern_Codec_ReadReg(kal_uint8 ucAddr, kal_uint16 *pucResult);
static void Extern_Codec_WriteReg(kal_uint8 ucAddr, kal_uint16 uiData);
static int Extern_Codec_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int Extern_Codec_i2c_remove(struct i2c_client *client);
static int Extern_Codec_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
static int Extern_Codec_probe(struct platform_device *pdev);
static int Extern_Codec_remove(struct platform_device *pdev);
static int Extern_Codec_suspend(struct platform_device *pdev);
static int Extern_Codec_resume(struct platform_device *pdev);
static int __init Extern_Codec_init(void);
static void __exit Extern_Codec_exit(void);
#endif /* _EXTERN_CODEC_H */

