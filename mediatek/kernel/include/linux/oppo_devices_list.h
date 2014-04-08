#ifndef __OPPO_DEVICES_LIST_H__
#define __OPPO_DEVICES_LIST_H__

#define D_OPPO_CUST_PART_MAGIC_NUM              (0x6F70706F)
#define D_OPPO_CUST_PART_CONFIG_MAGIC_NUM       (0x636F6E66)

typedef struct
{
    unsigned int        nMagicNum1;
    unsigned int        nMagicNum2;
    unsigned int        nPlUsbEnumEnabled;
    unsigned int        nUsbAutoSwitch;
    unsigned long long  nClrFlag;
    unsigned char       sMcpId[16];
}TOppoCustConfigInf;

typedef enum
{
    LCD_NONE = 0,
    LCD_HITACHI,
    LCD_HITACHI_VIDEO,
    LCD_TURLY,
	LCD_BYD,
	LCD_JDI,
	LCD_TRULY_OLD,
	LCD_TRULY_NEW,
	LCD_NOVTEK,
    LCD_OTHRE
} LCD_DEV;

typedef enum
{
    TP_NONE = 0,
    TP_ALPS,
    TP_TRULY,
    TP_TRULY_BLACK,
    TP_YOUNGFAST,
	TP_NITTO,
	TP_OIKE,
	TP_OFILM,
	TP_TPK,
	TP_JDI,
    TP_OFILM_WHITE,
	TP_OFILM_BLACK,
    TP_OTHER
	
} TP_DEV;

typedef enum
{
    CAMERA_BACK_NONE = 0,
    CAMERA_BACK_OV5650MIPI,
    CAMERA_BACK_OV5647,
    CAMERA_BACK_S5K4E5YA,
	//lvxj@MutimediaDrv.camsensor, 2012/08/31, add for 12021 test mode
    CAMERA_BACK_IMX105MIPI,
    CAMERA_BACK_VD6803A,
    CAMERA_BACK_MS2R,
    CAMERA_BACK_OTHER
} CAMERA_BACK_DEV;

typedef enum
{
    CAMERA_FRONT_NONE = 0,
    CAMERA_FRONT_mt9d115,
    CAMERA_FRONT_s5k5bbgx,
	//lvxj@MutimediaDrv.camsensor, 2012/08/31, add for 12021 test mode
    CAMERA_FRONT_ov7675,
    CAMERA_FRONT_hi704,
    CAMERA_FRONT_HI253,
    CAMERA_FRONT_OV5647,
     //zhangkw@CameraDrv.camera, 2013/06/08, add for 13003 test mode
    CAMERA_FRONT_OV5693,
    CAMERA_FRONT_OTHER
} CAMERA_FRONT_DEV;

typedef enum
{
    MCP_NONE = 0,
    MCP_KMSJS000KM_B308,
    MCP_H9DP32A4JJACGR_KEM,
    MCP_KMKJS000VM_B309,
    MCP_KMNJS000ZM_B205,
    MCP_H9TP32A4GDMCPR_KDM,
    MCP_KMKJS000VM_B604,
    MCP_KMK3U000VM_B410,
} MCP_DEV;

typedef enum
{
	ALSPS_NONE = 0,
	ALSPS_STK31XX,
	ALSPS_STK3X1X,
	ALSPS_TAOS_277X,
	ALSPS_LITE_558,
}ALSPS_DEV;

typedef enum
{
	GYRO_NONE = 0,
	GYRO_MISS,	
	GYRO_MPU6050C,
	GYRO_L3GD20,
}GYRO_DEV;

#endif