/************************************************************************************
** File: - mediatek\source\kernel\drivers\oppo_devices_list\oppo_devices_list.c
** VENDOR_EDIT
** Copyright (C), 2008-2012, OPPO Mobile Comm Corp., Ltd
** 
** Description: 
**      driver for get devices list
** 
** Version: 1.0
** Date created: 10:38:47,18/06/2012
** Author: Yixue.Ge@ProDrv.BL
** 
** --------------------------- Revision History: --------------------------------
** 	<author>	<data>			<desc>
** 
************************************************************************************/


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
//#include <mach/mt6577_gpio.h>
#include <linux/platform_device.h>
#include <linux/oppo_devices_list.h>

volatile LCD_DEV lcd_dev = LCD_NONE;
volatile TP_DEV tp_dev = TP_NONE;
volatile int TP_FW = 0;
volatile CAMERA_BACK_DEV camera_back_dev = CAMERA_BACK_NONE;
volatile CAMERA_FRONT_DEV camera_front_dev = CAMERA_FRONT_NONE;
volatile MCP_DEV mcp_dev= MCP_NONE;
volatile ALSPS_DEV alsps_dev = ALSPS_NONE;
volatile GYRO_DEV gyro_dev = GYRO_NONE;

unsigned char Mcp_Id[][9] = {
{0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff},//NONE
{0x15,0x01,0x00,0x53,0x4A,0x53,0x30,0x30,0x4D},//KMSJS000KM_B308
{0x90,0x01,0x4A,0x20,0x58,0x49,0x4E,0x59,0x48},//H9DP32A4JJACGR_KEM
{0x15,0x01,0x00,0x4B,0x4A,0x53,0x30,0x30,0x4D},//KMKJS000VM_B309
{0x15,0x01,0x00,0x4E,0x4A,0x53,0x30,0x30,0x4D},//KMNJS000ZM_B205
{0x90,0x01,0x4A,0x20,0x58,0x49,0x4E,0x59,0x48},//H9TP32A4GDMCPR_KDM
{0x15,0x01,0x00,0x4B,0x54,0x53,0x30,0x30,0x4D},//KMKJS000VM_B604
{0x15,0x01,0x00,0x4B,0x33,0x55,0x30,0x30,0x4D},//KMK3U000VM_B410
};
static int getMcpId(void)
{
    struct file* pfile = NULL;
	unsigned long magic; 
	loff_t pos = 0;
	mm_segment_t fs;
	int num = -1;
    unsigned char pCustPartBuf[1024];
    TOppoCustConfigInf *pConfigInf;
    int i;
    
	if(NULL == pfile){
		pfile = filp_open("/dev/oppo_custom", O_RDONLY, 0);
	}
	if(IS_ERR(pfile)){
		printk("error occured while opening file /dev/oppo_custom.\n");
		return 0;
	}
	//char * buf;
	fs = get_fs();
	set_fs(KERNEL_DS);
	num = vfs_read(pfile, pCustPartBuf, 1024, &pos);
	if(num != 1024)
	{
	    filp_close(pfile, NULL);
        return 0;
	}
	filp_close(pfile, NULL);
	set_fs(fs);
	pConfigInf = (TOppoCustConfigInf *)pCustPartBuf;
	if(D_OPPO_CUST_PART_MAGIC_NUM != pConfigInf->nMagicNum1)
    {  
	    printk("getMcpId OPPO_CUSTOM partition is illegal!\n");
	    return 0;
	}

	if(D_OPPO_CUST_PART_CONFIG_MAGIC_NUM != pConfigInf->nMagicNum2)
	{
	    printk("getMcpId OPPO_CUSTOM partition with error config magic number!\n");
	    return 0;
	}   
    printk("getMcpId:%x %x %x %x %x %x %x %x %x\n",pConfigInf->sMcpId[0],pConfigInf->sMcpId[1],
        pConfigInf->sMcpId[2],pConfigInf->sMcpId[3],pConfigInf->sMcpId[4],pConfigInf->sMcpId[5],
        pConfigInf->sMcpId[6],pConfigInf->sMcpId[7],pConfigInf->sMcpId[8]);
    for(i = 0; i < sizeof(Mcp_Id)/9;i++)
    {
        if(strncmp(Mcp_Id[i],pConfigInf->sMcpId,9) == 0)
        return i;
    }
	return 0;
}
void McpIdInit()
{
    if(!mcp_dev)
        mcp_dev = getMcpId();
}
static ssize_t oppo_devices_list_write(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	/*not support write now*/
	return count;
}

static ssize_t oppo_devices_list_read(struct file *file, char __user *buf, size_t count, loff_t *pos)
{
	char page[128]; 
	char *p = page;
	int len = 0; 
	unsigned char id[12]={0};
	printk("oppo_devices_list_read is called\n");

	/*************************add here for customer devices list***************/
    switch(lcd_dev)
    {
        case LCD_NONE:
            p += sprintf(p, "LCD :None\n");
            break;
        case LCD_HITACHI:
            p += sprintf(p, "LCD :Hitachi\n");
            break;
        case LCD_TURLY:
            p += sprintf(p, "LCD :Turly\n");
            break;
        case LCD_BYD:
            p += sprintf(p, "LCD :BYD\n");
            break;		
	     case LCD_JDI:
            p += sprintf(p, "LCD :JDI\n");
            break;	
        case LCD_HITACHI_VIDEO:
            p += sprintf(p, "LCD :Hitachi_video\n");
            break;
		 case LCD_TRULY_OLD:
            p += sprintf(p, "LCD :TRULY_OLD\n");
            break;	
	    case LCD_TRULY_NEW:
            p += sprintf(p, "LCD :TRULY_NEW\n");
			break;
		case LCD_NOVTEK:
            p += sprintf(p, "LCD :LCD_NOVTEK\n");		
            break;
		default:
            p += sprintf(p, "LCD :unknown\n");
            break;
    }
    
    switch(tp_dev)
    {
        case TP_NONE:
            p += sprintf(p, "TP  :None\n");
            break;
        case TP_ALPS:
            p += sprintf(p, "TP  :ALPS FW:0x%x\n",TP_FW);
            break;
        case TP_TRULY:
            p += sprintf(p, "TP  :TRULY 0x%x\n",TP_FW);
            break;
		case TP_TRULY_BLACK:
            p += sprintf(p, "TP  :TRULY_BLACK 0x%x\n",TP_FW);
            break;
		case TP_YOUNGFAST:		
            p += sprintf(p, "TP  :YOUNGFAST 0x%x\n",TP_FW);
			break;
		case TP_OFILM:		
            p += sprintf(p, "TP  :OFILM 0x%x\n",TP_FW);		
			break;
		case TP_TPK:		
            p += sprintf(p, "TP  :TPK 0x%x\n",TP_FW);	
            break;
		case TP_NITTO:
            p += sprintf(p, "TP  :NITTO 0x%x\n",TP_FW);
            break;
        case TP_OIKE:
            p += sprintf(p, "TP  :OIKE 0x%x\n",TP_FW);
            break;	
		case TP_JDI:
            p += sprintf(p, "TP  :JDI 0x%x\n",TP_FW);
            break;		
		case TP_OFILM_WHITE:
            p += sprintf(p, "TP  :OFILM_WHITE 0x%x\n",TP_FW);
			break;
		case TP_OFILM_BLACK:
            p += sprintf(p, "TP  :OFILM_BLACK 0x%x\n",TP_FW);
			break;
        default:
            p += sprintf(p, "TP  :unknown\n");
            break;
    }
    
    switch(camera_back_dev)
    {
        case CAMERA_BACK_NONE:
            p += sprintf(p, "Cam_b:None\n");
            break;
        case CAMERA_BACK_OV5650MIPI:
            p += sprintf(p, "Cam_b:OV5650MIPI\n");
            break;
        case CAMERA_BACK_OV5647:
            p += sprintf(p, "Cam_b:OV5647\n");
            break;
        case CAMERA_BACK_S5K4E5YA:
            p += sprintf(p, "Cam_b:S5K4E5YA\n");
            break;
        case CAMERA_BACK_IMX105MIPI:
            p += sprintf(p, "Cam_b:IMX105MIPI\n");
            break;
	  case CAMERA_BACK_MS2R:
            p += sprintf(p, "Cam_b:MS2RMIPI\n");
            break;
        case CAMERA_BACK_VD6803A:
	    p += sprintf(p,"Cam_b:VD6803A\n");
	    break;   
    
		
        default:
            p += sprintf(p, "Cam_b:unknown\n");
            break;
    }
    
    switch(camera_front_dev)
    {
        case CAMERA_FRONT_NONE:
            p += sprintf(p, "Cam_f:None\n");
            break;
        case CAMERA_FRONT_mt9d115:
            p += sprintf(p, "Cam_f:mt9d115\n");
            break;
        case CAMERA_FRONT_s5k5bbgx:
            p += sprintf(p, "Cam_f:s5k5bbgx\n");
            break;
		//lvxj@MutimediaDrv.camsensor, 2012/08/31, add for 12021 test mode	
        case CAMERA_FRONT_ov7675:
            p += sprintf(p, "Cam_f:ov7675\n");
            break;
        case CAMERA_FRONT_hi704:
            p += sprintf(p, "Cam_f:hi704\n");
            break;
        case CAMERA_FRONT_HI253:
            p += sprintf(p, "Cam_f:HI253\n");
            break;			
        case CAMERA_FRONT_OV5647:
            p += sprintf(p, "Cam_f:OV5647\n");
            break;
	//zhangkw@CameraDrv, 2013/06/08, add for test mode 13003		
	 case CAMERA_FRONT_OV5693:
            p += sprintf(p, "Cam_f:OV5693\n");
            break;
        default:
            p += sprintf(p, "Cam_f:unknown\n");
            break;
    }
    //McpIdInit();
    /*switch(mcp_dev)
    {
        case MCP_NONE:
            p += sprintf(p, "mcp :None\n");
            break;
        case MCP_KMSJS000KM_B308:
            p += sprintf(p, "mcp :EMMC_KMSJS000KM_B308\n");
            break;
        case MCP_H9DP32A4JJACGR_KEM:
            p += sprintf(p, "mcp :EMMC_H9DP32A4JJACGR_KEM\n");
            break;
        case MCP_KMNJS000ZM_B205:
            p += sprintf(p, "mcp :EMMC_KMNJS000ZM_B205\n");
            break;
        case MCP_H9TP32A4GDMCPR_KDM:
            p += sprintf(p, "mcp :EMMC_H9TP32A4GDMCPR_KDM\n");
            break;
        default:
            p += sprintf(p, "mcp :unknown\n");
            break;
    }*/
/*for als ps dev*/
	switch(alsps_dev)
	{
		case ALSPS_NONE:
			p += sprintf(p, "ALS_PS:none\n");
			break;
		case ALSPS_STK31XX:
			p += sprintf(p, "ALS_PS:stk31xx\n");
			break;
		case ALSPS_STK3X1X:
			p += sprintf(p, "ALS_PS:stk3x1x\n");
			break;
		case ALSPS_TAOS_277X:
			p += sprintf(p, "ALS_PS:tmd277x\n");
			break;
		case ALSPS_LITE_558:
			p += sprintf(p, "ALS_PS:ltr558\n");
			break;
		default :
			p += sprintf(p, "ALS_PS:unknown\n");

	}

	switch(gyro_dev)
	{
		case GYRO_NONE:
			p += sprintf(p, "GYRO:None\n");
			break;
		case GYRO_MISS:
			p += sprintf(p, "GYRO:Miss\n");
			break;			
		case GYRO_MPU6050C:
			p += sprintf(p, "GYRO:MPU6050C\n");
			break;
		case GYRO_L3GD20:
			p += sprintf(p, "GYRO:L3GD20\n");
			break;
		default :
			p += sprintf(p, "GYRO:Unknown\n");

	}


	/*****************************add end**************************************/
	
	len = p - page;
	if (len > *pos)
		len -= *pos;
	else
		len = 0;
	if (copy_to_user(buf,page,len < count ? len  : count))
		return -EFAULT;
	*pos = *pos + (len < count ? len  : count);
	return len < count ? len  : count;

}

static const struct file_operations oppo_devices_list = {
	.write		= oppo_devices_list_write,
	.read		= oppo_devices_list_read,
};
static int oppo_dev_platform_probe(struct platform_device *pdev)
{
    proc_create("oppo_devices_list", 0666, NULL, &oppo_devices_list);
   
    return 0;
}

static struct platform_driver oppo_dev_platform_driver = {
    //.remove     = NULL,
    //.shutdown   = NULL,
    .probe      = oppo_dev_platform_probe,
    //#ifndef CONFIG_HAS_EARLYSUSPEND
    //.suspend    = NULL,
    .resume     = NULL,
   // #endif
    .driver     = {
    	.owner = THIS_MODULE,
        .name = "oppo_dev_platform_driver",
    },
};
static struct platform_device oppo_dev_platform_device = {
	.name = "oppo_dev_platform_driver",
	.id = -1
};

static int __init oppo_devices_list_init(void)
{
    int ret;
	ret = platform_device_register(&oppo_dev_platform_device);
	if (ret)
		printk("oppo_dev_platform_device:dev:E%d\n", ret);

	ret = platform_driver_register(&oppo_dev_platform_driver);

	if (ret)
	{
		printk("oppo_dev_platform_driver:drv:E%d\n", ret);
		platform_device_unregister(&oppo_dev_platform_device);
		return ret;
	}
	return 0;
}
static void __exit oppo_devices_list_exit(void)
{
	platform_driver_unregister(&oppo_dev_platform_driver);
	platform_device_unregister(&oppo_dev_platform_device);
}
module_init(oppo_devices_list_init);

