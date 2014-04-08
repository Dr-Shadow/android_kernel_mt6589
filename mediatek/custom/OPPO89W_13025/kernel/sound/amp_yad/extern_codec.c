/************************************************************************************
** File: - mediatek\custom\OPPO89T_12083\kernel\sound\amp_yad\extern_codec.c
** OPPO_R819
** Copyright (C), 2008-2012, OPPO Mobile Comm Corp., Ltd
** 
** Description: 
**      extern codec
** 
** Version: 1.0
** Date created: 10:34:16,18/12/2012
** Author: xiang.fei
** 
** --------------------------- Revision History: --------------------------------
** 	<author>	<data>			<desc>
** 
************************************************************************************/

#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include "extern_codec.h"

#define PROC_DEBUG
#ifdef  PROC_DEBUG
#include <linux/proc_fs.h>
static struct proc_dir_entry* Debug_entry;
struct proc_file{
    struct file_operations opera;
    char name[20];
};
#endif

extern void upmu_set_rg_gpio26m_pdn(U32 val);
static bool bFreeClkStatus = false;
static bool bCodecStatus = false;
static const struct i2c_device_id Extern_Codec_i2c_id[] = {{EXTERN_CODEC_I2C_DRVNAME,0},{}};
static struct i2c_board_info __initdata Extern_Codec_i2c_info = { I2C_BOARD_INFO(EXTERN_CODEC_I2C_DRVNAME, (EXTERN_CODEC_DEVICE_ID>>1))};
static struct i2c_client * g_pstExtern_Codec_I2Cclient = NULL;

static struct i2c_driver Extern_Codec_i2c_driver = {
    .probe = Extern_Codec_i2c_probe,                                   
    .remove = Extern_Codec_i2c_remove,                           
    .detect = Extern_Codec_i2c_detect,                           
    .driver.name = EXTERN_CODEC_I2C_DRVNAME,
    .id_table = Extern_Codec_i2c_id,                                
};

// platform structure
static struct platform_driver g_stExtern_Codec_Driver = {
    .probe = Extern_Codec_probe,
    .remove	= Extern_Codec_remove,
    .suspend = Extern_Codec_suspend,
    .resume	= Extern_Codec_resume,
    .driver = {
    .name = EXTERN_CODEC_DRVNAME,
    .owner = THIS_MODULE,
    }
};

#ifdef PROC_DEBUG
static ssize_t Extern_Codec_proc_read_status(struct file *file, char __user *buf, size_t count, loff_t *pos)
{
    kal_uint8 ucAddr = 0;
    kal_uint16 val = 0;
    int len = 0;
    char pri_buf[PAGE_SIZE];
    memset(pri_buf,0,sizeof(pri_buf));

    for(ucAddr = 0; ucAddr < 0xFF; ucAddr++)
    {
        Extern_Codec_ReadReg(ucAddr, &val);
        len += sprintf(pri_buf+len, "R:%02x V:%04x\n", ucAddr, val);
        val = 0;
    }
    copy_to_user(buf, pri_buf, len);

    if (len > *pos)
    {
        len -= *pos;
    }    
    else
    {
        len = 0;
    }
    *pos =*pos + len;
    count -= len;

    return len;
}
static ssize_t Extern_Codec_proc_set_reg(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    int reg = 0;
	int val = 0;
	int read = 0;
	char tmp[10] = {0};

	copy_from_user(tmp, buf, count);
	if (read = sscanf(tmp, "%x,%x", &reg, &val))
	{    
	    Extern_Codec_WriteReg(reg, val);
		AUDIO_DBG("%s reg = 0x%x, val = 0x%x, read = %d\n", __func__, reg, val, read);
	}    
	else 
	{    
		AUDIO_DBG("%s fail!!\n", __func__);
	}    

	return count; 
}

static ssize_t Extern_Codec_proc_get_reg(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    kal_uint8 reg = 0;
	int val = 0;
	int read = 0;
	char tmp[10] = {0};

	copy_from_user(tmp, buf, count);
	if (read = sscanf(tmp, "%x", &reg))
	{    
	    Extern_Codec_ReadReg(reg, &val);
		AUDIO_DBG("%s reg = 0x%x, val = 0x%x, read = %d\n", __func__, reg, val, read);
	}    
	else 
	{    
		AUDIO_DBG("%s fail!!\n", __func__);
	}    

	return count; 
}

static struct proc_file proc_file_st[] = {
    {.name = "status",  .opera = {.read = Extern_Codec_proc_read_status}},
    {.name = "set_reg", .opera = {.write = Extern_Codec_proc_set_reg}},
    {.name = "get_reg", .opera = {.write = Extern_Codec_proc_get_reg}},
};
static void Extern_Codec_create_proc_files(struct proc_dir_entry *entry)
{
    int count = sizeof(proc_file_st)/sizeof(proc_file_st[0]);
    int i = 0;
    for (i = 0; i < count; i++)
    {
        proc_create(proc_file_st[i].name, 0666, entry, &proc_file_st[i].opera);
    }
}
static void Extern_Codec_create_proc(void)
{
    Debug_entry = proc_mkdir_mode("Extern_Codec_debug", 0666, NULL);
    Extern_Codec_create_proc_files(Debug_entry);
}
#endif

static void Extern_Codec_Power(bool Enable)
{
    if (Enable)
	{
		hwPowerOn(MT65XX_POWER_LDO_VMCH1,VOL_3000, "codec_3000");//for g_sensor m_sensor
		msleep(1);
		hwPowerOn(MT65XX_POWER_LDO_VGP5,VOL_1800, "codec_1800");
		msleep(1);
	}
	else
	{
		hwPowerDown(MT65XX_POWER_LDO_VGP5, "codec_1800");
		msleep(1);
		hwPowerDown(MT65XX_POWER_LDO_VMCH1, "codec_3000");
	}

	return;
}

static void Extern_Codec_Init(void)
{
	Extern_Codec_WriteReg(0x04, 0x0018);
	Extern_Codec_WriteReg(0x05, 0x0047);
	mdelay(5);
	Extern_Codec_WriteReg(0x05, 0x0043);
	Extern_Codec_WriteReg(0x04, 0x0019);
	Extern_Codec_WriteReg(0x0E, 0x0003);
	Extern_Codec_WriteReg(0x0F, 0x0003);
	mdelay(1);
    Extern_Codec_FreeClk(true);
	Extern_Codec_WriteReg(0xFF, 0x0000);
	Extern_Codec_WriteReg(0x04, 0x000B);
	Extern_Codec_WriteReg(0x62, 0x0001);
	mdelay(5);
	Extern_Codec_WriteReg(0x14, 0x845E);
	Extern_Codec_WriteReg(0x19, 0x4002);

	/* Set path volume */
	Extern_Codec_WriteReg(0x39, 0x00B6); 
	Extern_Codec_WriteReg(0x3A, 0x00B6);
	Extern_Codec_WriteReg(0x68, 0x0004);
	Extern_Codec_WriteReg(0x2C, 0x0005); 
	Extern_Codec_WriteReg(0x2D, 0x0005);
	Extern_Codec_WriteReg(0x2E, 0x0045);/* IN2L cfg different line */
	Extern_Codec_WriteReg(0x2F, 0x0045);/* IN2R cfg different line */

    AUDIO_DBG("[Extern_Codec] %s Success\n", __func__);

    return;
}

static void Extern_Codec_FreeClk(kal_bool b_OnOff)
{
    if(KAL_TRUE == b_OnOff)
    {
        /* open free running fll clk */
        Extern_Codec_WriteReg(0x74, 0x0003);
    	Extern_Codec_WriteReg(0x75, 0x0707);
    	Extern_Codec_WriteReg(0xF7, 0x0001);
    	Extern_Codec_WriteReg(0xF8, 0x0019);
    	Extern_Codec_WriteReg(0x16, 0x4004);
    	bFreeClkStatus = true;
    }
    else
    {
        /* close free running fll clk */
        Extern_Codec_WriteReg(0xF7, 0x0000);
        Extern_Codec_WriteReg(0x16, 0x0000);
        Extern_Codec_WriteReg(0x74, 0x0000);
        bFreeClkStatus = false;
    }
    
	return;
}

void Extern_Codec_SetPlayFreq(kal_uint16 uiFreq)
{
    if(bFreeClkStatus)
    {
        Extern_Codec_FreeClk(false);
    }
    
   
    switch(uiFreq)
    {			
		case FREQ_32K:
		    Extern_Codec_WriteReg(0x78, 0x0006);
			Extern_Codec_WriteReg(0x75, 0x0700);	
			Extern_Codec_WriteReg(0x76, 0x8FD5);	
			Extern_Codec_WriteReg(0x77, 0x00E0);
			Extern_Codec_WriteReg(0x74, 0x0005);
			Extern_Codec_WriteReg(0x15, 0x1004);
			break;
			
		case FREQ_44K:
		case FREQ_48K:
		default:
		    Extern_Codec_WriteReg(0x78, 0x0006);
		    Extern_Codec_WriteReg(0x75, 0x0704);	
			Extern_Codec_WriteReg(0x76, 0x0000);	
			Extern_Codec_WriteReg(0x77, 0x1000);
			Extern_Codec_WriteReg(0x74, 0x0005);
			Extern_Codec_WriteReg(0x15, 0x0C05);
			break;
	}
	
	Extern_Codec_WriteReg(0x62, 0x0001);
	Extern_Codec_WriteReg(0x16, 0x4006);
	
    return;
}

void Extern_Codec_DAC_Input_Path(kal_bool b_OnOff)
{
    if(KAL_TRUE == b_OnOff)
    {
        /* Config DAC */
        Extern_Codec_WriteReg(0x12, 0x000C);
        Extern_Codec_WriteReg(0x21, 0x0040);
        Extern_Codec_WriteReg(0x3D, 0x0000);
    }
    else
    {
        Extern_Codec_WriteReg(0x21, 0x0008);/* Mute DAC */
        Extern_Codec_WriteReg(0x12, 0x0000); 
    }

    return;
}

void Extern_Codec_IN2_Input_Path(kal_bool b_OnOff)
{
	if(KAL_TRUE == b_OnOff)
	{
        if(!bFreeClkStatus)
        {
            Extern_Codec_FreeClk(true);
        }
        
	    /* Config line input */
		Extern_Codec_WriteReg(0x0C, 0x0003);/* power */
		Extern_Codec_WriteReg(0x3D, 0x000F);
	}
	else
	{
	    /* Turn off power that disable line input */
		Extern_Codec_WriteReg(0x0C, 0x0000);	
	}

	return;
}

void Extern_Codec_Hp_Output_Path(kal_bool b_OnOff)
{
    kal_uint16 uiTemp = 0;
    
	if(KAL_TRUE == b_OnOff)
	{
		Extern_Codec_WriteReg(0x5A, 0x0011);
		mdelay(1);
		Extern_Codec_WriteReg(0x5A, 0x0033);
		Extern_Codec_WriteReg(0x43, 0x0003);
		Extern_Codec_WriteReg(0x44, 0x0030);
		msleep(256);
		Extern_Codec_WriteReg(0xFF, 0x0000);
		Extern_Codec_WriteReg(0x5A, 0x0077);
		Extern_Codec_WriteReg(0x5A, 0x00FF);
		Extern_Codec_WriteReg(0xFF, 0x0000);
		Extern_Codec_WriteReg(0xFF, 0x0000);
		bCodecStatus = true;
	}
	else
	{
	    Extern_Codec_ReadReg(0x5A, &uiTemp);
        uiTemp &= ~0x0088;
		Extern_Codec_WriteReg(0x5A, uiTemp);
		uiTemp &= ~0x0011;
		Extern_Codec_WriteReg(0x5A, uiTemp);
		uiTemp &= ~0x0022;
		Extern_Codec_WriteReg(0x5A, uiTemp);		
		uiTemp &= ~0x0044;
		Extern_Codec_WriteReg(0x5A, uiTemp);
		Extern_Codec_ReadReg(0x43, &uiTemp);
		uiTemp &= ~0x0003;
		Extern_Codec_WriteReg(0x43, uiTemp);	
		bCodecStatus = false;
	}

	return;
}

void Extern_Codec_Spk_Output_Path(kal_bool b_OnOff)
{
    kal_uint16 uiTemp = 0;
    
	if(KAL_TRUE == b_OnOff)
	{
		Extern_Codec_WriteReg(0x5E, 0x0011);
		mdelay(1);
		Extern_Codec_WriteReg(0x5E, 0x0033);
		Extern_Codec_WriteReg(0x43, 0x000C);
		Extern_Codec_WriteReg(0x44, 0x00C0);
		msleep(256);
		Extern_Codec_WriteReg(0xFF, 0x0000);
		Extern_Codec_WriteReg(0x5E, 0x0077);
		Extern_Codec_WriteReg(0x5E, 0x00FF);
		Extern_Codec_WriteReg(0xFF, 0x0000);
		Extern_Codec_WriteReg(0xFF, 0x0000);
		bCodecStatus = true;
	}
	else
	{
		Extern_Codec_ReadReg(0x5E, &uiTemp);
        uiTemp &= ~0x0088;
		Extern_Codec_WriteReg(0x5E, uiTemp);
		uiTemp &= ~0x0011;
		Extern_Codec_WriteReg(0x5E, uiTemp);
		uiTemp &= ~0x0022;
		Extern_Codec_WriteReg(0x5E, uiTemp);		
		uiTemp &= ~0x0044;
		Extern_Codec_WriteReg(0x5E, uiTemp);
		Extern_Codec_ReadReg(0x43, &uiTemp);
		uiTemp &= ~0x000C;
		Extern_Codec_WriteReg(0x43, uiTemp);
		bCodecStatus = false;
	}

	return;
}

static void Extern_Codec_ReadReg(kal_uint8 ucAddr, kal_uint16 *pucResult)
{
    int iRetValue = 0;
    kal_uint8 ucBuff = 0;
    char ucData[2] = {0};

    ucBuff = ucAddr;
    iRetValue = i2c_master_send(g_pstExtern_Codec_I2Cclient, &ucBuff, 1);
    iRetValue |= i2c_master_recv(g_pstExtern_Codec_I2Cclient, ucData, 2);
    *pucResult = ((kal_uint16)ucData[0]) << 8;
    *pucResult |= (kal_uint16)ucData[1];
    if (iRetValue < 0) 
    {
        AUDIO_DBG("[Extern_Codec] %s Addr:0x%x=0x%x failed!! \n", __func__, ucAddr, *pucResult);
    }
    
    return;
}

static void Extern_Codec_WriteReg(kal_uint8 ucAddr, kal_uint16 uiData)
{
    int iRetValue = 0;
    char ucSendData[3] = {0};

    ucSendData[0] = (char)ucAddr;
    ucSendData[1] = (char)(uiData >> 8);
    ucSendData[2] = (char)(uiData & 0x00FF);
    iRetValue = i2c_master_send(g_pstExtern_Codec_I2Cclient, ucSendData, 3);
    if (iRetValue < 0) 
    {
        AUDIO_DBG("[Extern_Codec] %s Addr:0x%x=0x%x failed!! \n", __func__, ucAddr, uiData);
    }

    return;
}

static int Extern_Codec_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id) 
{             
    /* get i2c client */
    g_pstExtern_Codec_I2Cclient = client;

    Extern_Codec_Power(true);
    Extern_Codec_WriteReg(0x00, 0x0000);
    Extern_Codec_Init();
    AUDIO_DBG("[Extern_Codec] %s\n", __func__);
    
    return 0;                                                                                       
}

static int Extern_Codec_i2c_remove(struct i2c_client *client)
{
    return 0;
}

static int Extern_Codec_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) 
{         
    strcpy(info->type, EXTERN_CODEC_I2C_DRVNAME);
    return 0;
}

static int Extern_Codec_probe(struct platform_device *pdev)
{
#ifdef PROC_DEBUG
    Extern_Codec_create_proc();
#endif 

    return i2c_add_driver(&Extern_Codec_i2c_driver);
}

static int Extern_Codec_remove(struct platform_device *pdev)
{
    i2c_del_driver(&Extern_Codec_i2c_driver);
    return 0;
}

static int Extern_Codec_suspend(struct platform_device *pdev)
{
    if(!bCodecStatus)
    {
        Extern_Codec_WriteReg(0x00, 0x0000);
        Extern_Codec_Power(false);
        AUDIO_DBG("[Extern_Codec] %s\n", __func__);
    }
    
    return 0;
}

static int Extern_Codec_resume(struct platform_device *pdev)
{
    if(!bCodecStatus)
    {
        Extern_Codec_Power(true);
        Extern_Codec_Init();
        AUDIO_DBG("[Extern_Codec] %s\n", __func__);
    }
    
    return 0;
}

static int __init Extern_Codec_init(void)
{ 
    i2c_register_board_info(2, &Extern_Codec_i2c_info, 1);

    if(platform_driver_register(&g_stExtern_Codec_Driver))
    {
        AUDIO_DBG("[Extern_Codec] failed to register driver\n");
        return -ENODEV;
    }	

    return 0;
}

static void __exit Extern_Codec_exit(void)
{
	platform_driver_unregister(&g_stExtern_Codec_Driver);
}

module_init(Extern_Codec_init);
module_exit(Extern_Codec_exit);

MODULE_DESCRIPTION("Extern_Codec Driver");


