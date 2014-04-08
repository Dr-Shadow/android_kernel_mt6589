/************************************************************************************
** File: - Z:\work\code\mtkbase\11071\svn2871_whole\development_emmc\mediatek\custom\oppo75_emmc_11071\common\oppo_custom_partition.h
** OPPO_R819
** Copyright (C), 2008-2012, OPPO Mobile Comm Corp., Ltd
** 
** Description: 
**      Add this file for customizing OPPO_CUSTOM partition!
** 
** Version: 0.1
** Date created: 15:20:03,09/04/2012
** Author: Jinquan.Lin@BasicDrv.BL
** 
** --------------------------- Revision History: --------------------------------
** 	<author>	                    <data>			<desc>
** Jinquan.Lin@BasicDrv.BL          09/04/2012      Init
************************************************************************************/

#ifndef __OPPO_CUSTOM_PARTITION_H__
#define __OPPO_CUSTOM_PARTITION_H__


#define D_OPPO_CUST_PART_BUF_SIZE           (1024)
#define D_OPPO_CUST_PART_BLK_SIZE           (512)

#define D_OPPO_CUST_PART_CLR_FLAG_SOURCE_MAX    (8) //  64/8
#define D_OPPO_CUST_PART_MAGIC_NUM          (0x6F70706F)
#define D_OPPO_CUST_PART_CONFIG_MAGIC_NUM   (0x636F6E66)

typedef enum
{
    OPPO_CUST_PART_ITEM__CONFIG     = 0x00,
    OPPO_CUST_PART_ITEM__MAX        
}TOppoCustPartItem;

typedef struct
{
    unsigned int    nMagicNum1;
    unsigned int    nMagicNum2;
    unsigned int    nPlUsbEnumEnabled;
      unsigned int        nUsbAutoSwitch;
      unsigned long long  nClrFlag;
      unsigned char       sMcpId[16];
      unsigned int 		nGsensorCalibrateFlag;
      int					GsensorData[3];
      unsigned int        nRebootNumber;
      unsigned int        nRebootReason;
      //#ifndef OPPO_R819//mingqiang.guo@Prd.BasicDrv.Sensor, add 2012/11/15 add for ps revoer send pulse count 
      unsigned int        nAlsPsCalibratePsFlag;
      unsigned int        nAlsPsSendPulseNumber;
      unsigned int        nAlsPsOffsetValue;
      //#endif /*OPPO_R819*/
      //#ifndef OPPO_R819//LiuPing@Prd.BasicDrv.Sensor, add 2012/12/05 add for ps switch 
      unsigned int        nAlsPsSwitchPsStoreFlag;
      unsigned int        nAlsPsSwitchPsVal;  // 0:on 1:off
      //#endif /*OPPO_R819*/

}TOppoCustConfigInf;

typedef enum
{
    OPPO_CUST_CONFIG_ITEM__PL_USB_ENUM,
    OPPO_CUST_CONFIG_ITEM__CLR_FLAG,
    OPPO_CUST_CONFIG_ITEM__MAX
}TOppoCustConfigItem;

typedef enum
{
    OPPO_CUST_USB_VOTER__MANUAL     =   (1 << 0),
    OPPO_CUST_USB_VOTER__AUTO       =   (1 << 1)
}TOppoCustUsbVoter;

extern int OppoCustPartLoadPartInf(TOppoCustPartItem ePartItem, unsigned char *pDstAddr);
extern int OppoCustPartWritePartInf(TOppoCustPartItem ePartItem, unsigned char *pSrcAddr);

#endif/*__OPPO_CUSTOM_PARTITION_H__*/