/************************************************************************************
** File: - ms2ryuv_Sensor.c
** VENDOR_EDIT
** Copyright (C), 2008-2012, OPPO Mobile Comm Corp., Ltd
** 
** Description: 
**      The driver of ms2r_yuv sensor
** 
** Version: 1.0
** Date created: 16:45:14,04/09/2012
** Author: LiuBin@MtkCamera
** 
** --------------------------- Revision History: --------------------------------
** 	<author>	<data>			<desc>
** 
************************************************************************************/

#include "ms2ryuv_Sensor.h"
#define USE_WQ_accelerate
#define USE_IRQ
#define USE_KTHREAD
//#undef USE_KTHREAD
#ifdef USE_KTHREAD
#include <linux/semaphore.h>
#include <linux/kthread.h>
struct semaphore ms2r_sem;
struct semaphore close_sem;
static volatile bool ms_work_mode = false;
static struct task_struct *ms2r_init_task = NULL;
static kal_bool ms2r_kthread = KAL_FALSE;
static int ms2r_init_kthread(void *data);
#endif
static volatile bool force_stop_worker = false;

#ifdef USE_KTHREAD
#undef USE_WQ_accelerate
#endif

#ifdef USE_WQ_accelerate
#include <linux/workqueue.h>
#include <linux/wait.h>
static struct work_struct ms2r_work;
static struct workqueue_struct * ms2r_workqueue = NULL;
static kal_bool ms2r_queue = KAL_FALSE;
#endif

//ISP debug flag
static volatile bool isp_tuning_enable = false;
//ISP capture raw data flag
static volatile bool isp_capture_raw_data = false;
//ISP debug flag
static volatile bool isp_debug_enable = false;

#define SENSORDB(fmt, arg...)				\
{											\
	if (isp_debug_enable)					\
	{										\
		printk( "[MS2R_YUV] "  fmt, ##arg); \
	}										\
}											\

#define Sleep(ms) mdelay(ms)

#ifdef USE_IRQ
#include <linux/interrupt.h>
#include <mach/irqs.h>
#define EINT_NUM		(159)
static kal_bool ms2r_int = KAL_FALSE;
static struct work_struct ms2r_status_work;
static struct workqueue_struct * ms2r_status_workqueue = NULL;
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_polarity(kal_uint8 eintno, kal_bool ACT_Polarity);
extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
                                     kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
                                     kal_bool auto_umask);

static irqreturn_t handle_ms2r_state(int irq,void *dev_id);
static void MS2R_AF_constant_focus(void);
void ms2r_status_work_callback(struct work_struct *work);
static irqreturn_t handle_ms2r_state(int irq,void *dev_id)
{
	SENSORDB("handle_ms2r_state---------> IN \r\n");
	queue_work(ms2r_status_workqueue, &ms2r_status_work); //schedule a work for the first detection
	return IRQ_HANDLED;
}
static void ms2r_install_IRQ()
{
	/* xiang.fei@MutimediaDrv.audio, 2012/08/16, Modify for headset eint */
	SENSORDB("ms2r_install_IRQ 0x%x\r\n");
	mt65xx_eint_set_sens(EINT_NUM, 1);
	mt65xx_eint_set_hw_debounce(EINT_NUM, 0);
	mt65xx_eint_registration(EINT_NUM, false, 1, handle_ms2r_state, false);
	mt65xx_eint_mask(EINT_NUM);
}
#endif

//#define CHECK_CAMERA_FIRMWARE
//#define SEND_CAMERA_FW_BY_I2C_MODE

#if defined (SEND_CAMERA_FW_BY_I2C_MODE) || defined(CHECK_CAMERA_FIRMWARE)
#undef  USE_KTHREAD
#undef USE_WQ_accelerate
#endif

static MSDK_SENSOR_CONFIG_STRUCT MS2RSensorConfigData;
static volatile MS2R_SYS_MODE g_eSysMode = SYS_MODE_POWEROFF;
static volatile u8 FW_transfer_mode = 0;
static u8 FW_transfer_retrycnt = 0;
static UINT8 g_uZone[4] = {0x00};

//HDR
static kal_bool g_bHDREnable = KAL_FALSE;

#if defined(USE_WQ_accelerate)||defined(USE_KTHREAD)
static kal_bool ms2r_ctl_init = KAL_FALSE;

static struct{  
	UINT16 Effect;  
	UINT16 Exposure;  
	UINT16 Wb;  
	UINT16 Stobe;  
	UINT16 Scene_mode;  
	UINT16 Hue;  
	UINT16 Contrast;  
	UINT16 Edge;  
	UINT16 Brightness;  
	UINT16 Sat;
	UINT16 AE_HDR;
	UINT16 AE_ISO;
	UINT16 AE_mode;
	UINT16 AF_mode;
	UINT16 update;  
    } ms2r_ctl;
static void MS2R_update_parm(void);

#endif

//exif info
static SENSOR_EXIF_INFO_STRUCT g_stExifInfo;

static struct
{
	UINT16 m_uEffect;
	UINT16 m_uExposure;
	UINT16 m_uWb;
	UINT16 m_uStobe;  
	UINT16 m_uSceneMode;  
	UINT16 m_uContrast;
	UINT16 m_uEdge;
	UINT16 m_uBrightness;
	UINT16 m_uSat;
	UINT16 m_uAEISO;
}MS2RFeatureInfo;

//capture flag
static kal_bool bISPCapture = KAL_FALSE;

//focus position
static UINT16 pre_AFposition = 0; 

//AE mode
static MS2R_AE_MODE g_eAEMode = MS2R_AE_MODE_AUTO;

//Face AE
static MUINT32 g_uFaceAECnt = 0;
static MUINT32 g_uFaceAECntPre = 0;
static DEFINE_SPINLOCK(FaceAE_lock);
static bool g_bHasFaces = false;

//WaveDetect value if valid, when before trigger focus, is not valid
static kal_bool bWaveDetectValid = KAL_FALSE;
static kal_bool nDelayFrameCnt = 0;
static kal_bool bNeedDelay = KAL_FALSE;

static bool bCaptureStart = false;

/*----------get gyro data for CAF--------------*/
static int g_uGyroData[3] = {0};
static bool g_bGyroOpened = false;

extern int L3G4200D_OpenGyro_preview(void);
extern int L3G4200D_CloseGyro_preview(void);
extern int L3G4200D_ReadGyroData_preview(char *buf);
/*----------get gyro data for CAF--------------*/

static void MS2R_update_feature_param(void);
static DEFINE_SPINLOCK(ms2r_drv_lock);
static void MS2R_capture_sequence(void);
static void MS2R_monitor_sequence(void);
static void MS2R_video_sequence(void);
static void MS2R_get_brightness(UINT16 *puBrightness);
static void MS2R_get_wavedetectvalue(UINT32 *wdv);
static void MS2R_get_whitebalance(UINT16 *puWhiteBalance);
void MS2R_update_exif_info(void);

extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern int camera_spi_write(char *pBuf, unsigned int uSize, u8 mode);
extern int camera_spi_read(char *pBuf, unsigned int uSize);

/*----Read camera firmware from system file, this method only use for debug-----*/
#if 1
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/stat.h>
#include <linux/fs.h>
#include <asm/unistd.h>
#include <asm/uaccess.h>
#include <linux/types.h>
#include <linux/ioctl.h>

static char *g_pMS2RBinBuf = NULL;
#define CAMERA_FIRMWARE_FILE		"/mnt/sdcard/wallpaper/RS_MS2_PF_IU111.bin"

static int MS2R_allocate_camera_firmware_buf(void)
{
	g_pMS2RBinBuf = kmalloc(RS_MS2_BIN_SIZE, GFP_KERNEL);
	if (NULL == g_pMS2RBinBuf)
	{
		SENSORDB("Allocate camera firmware buffer FAIL \r\n");
		return -1;
	}

	SENSORDB("Allocate camera firmware buffer SUCCESS \r\n");

	return 0;
	
}

static void MS2R_free_camera_firmware_buf(void)
{
	if (NULL != g_pMS2RBinBuf)
	{
		kfree(g_pMS2RBinBuf);
	}
}
static int MS2R_read_camera_firmware_from_file(void)
{
	struct file *filep = NULL;
	mm_segment_t old_fs;
	
	ssize_t result;

	filep = filp_open(CAMERA_FIRMWARE_FILE, O_RDONLY, 0600);
	if (IS_ERR(filep))
	{
		printk("Open file [%s] fail!  errno[%d] \r\n", CAMERA_FIRMWARE_FILE, IS_ERR(filep));
		return -EIO;
	}

	old_fs=get_fs();
 	set_fs(KERNEL_DS);
	
	result = vfs_read(filep, g_pMS2RBinBuf, RS_MS2_BIN_SIZE, &filep->f_pos);
	if (result >= 0)
	{
		printk("---get the camera firmware form file succeed !! \r\n");
	}
	else
	{
		printk("---read the file failed!! \r\n");
	}
	
	set_fs(old_fs);
	
	filp_close(filep, NULL);

	return 0;
	
}

#define SAVE_CAMERA_FIRMWARE_DIR "/mnt/sdcard/ms2r_camera_firmware"
static void MS2R_save_camera_firmware_to_system_file(void)
{
	struct file *filep = NULL;
	mm_segment_t old_fs;

	static int fileIndex = 0;
	char pFileName[256] ;
	sprintf(pFileName, "%s_%04d.bin", SAVE_CAMERA_FIRMWARE_DIR, fileIndex++);
	
	SENSORDB("save the sensor driver index to system file \r\n");

	filep = filp_open(pFileName, O_CREAT|O_RDWR, 0600);
	if (IS_ERR(filep))
	{
		printk("Open file [%s] fail!  errno[%d] \r\n", pFileName, IS_ERR(filep));
		return;
	}

	old_fs=get_fs();
 	set_fs(KERNEL_DS);
	
	vfs_write(filep, g_pMS2RBinBuf, RS_MS2_BIN_SIZE, &filep->f_pos);

	set_fs(old_fs);
 
	filp_close(filep, NULL);
	
}
#endif


/*--------------------- read / write memory ---------------------*/
static void MS2R_read_memory(u32 addr, u8 *pReadData, u32 uDataSize)
{
	int nCount = 1;
	u8 uSendCmd[8] = {0};

	u8 * pTempBuff = (u8 *)kmalloc((MAX_MEMORY_DATA_SIZE+3), GFP_KERNEL);
	if (NULL == pTempBuff)
	{
		SENSORDB("[MS2R_read_memory]----kmalloc  fail \r\n");
		return;
	}

	if ((0 == addr) || (NULL == pReadData) || (0 == uDataSize))
	{
		SENSORDB("[MS2R_read_memory] Parameter Invalid \r\n");
		return;
	}

	while (uDataSize > 0)
	{
		*uSendCmd = 0x00;
		*(uSendCmd+1) = COMMAND_ID_READ_MEMORY_8BIT_ACCESS;
		*(uSendCmd+2) = (addr >> 24) & 0xFF;
		*(uSendCmd+3) = (addr >> 16) & 0xFF;
		*(uSendCmd+4) = (addr >> 8) & 0xFF;
		*(uSendCmd+5) = addr & 0xFF;

		if (uDataSize >= MAX_MEMORY_DATA_SIZE)
		{
			SENSORDB("read to memory ----[%d]---\r\n", nCount);
			nCount++;
			*(uSendCmd+6) = MAX_MEMORY_DATA_SIZE  >> 8;
			*(uSendCmd+7) = MAX_MEMORY_DATA_SIZE & 0xFF;

			iReadRegI2C(uSendCmd, 8, pTempBuff, MAX_MEMORY_DATA_SIZE+3, MS2R_WRITE_ID);
			memcpy(pReadData, pTempBuff+3, MAX_MEMORY_DATA_SIZE);

			addr += MAX_MEMORY_DATA_SIZE;
			pReadData += MAX_MEMORY_DATA_SIZE;
			uDataSize -= MAX_MEMORY_DATA_SIZE;
		}
		else
		{
			SENSORDB("the last bytes read start \r\n");
			*(uSendCmd+6) = (u8)(uDataSize >> 8);
			*(uSendCmd+7) = (u8)(uDataSize & 0xFF);

			iReadRegI2C(uSendCmd, 8, pTempBuff, uDataSize+3, MS2R_WRITE_ID);
			memcpy(pReadData, pTempBuff+3, uDataSize);
			SENSORDB("the last bytes read finish \r\n");
			uDataSize = 0;
		}
		
	}

	if (NULL != pTempBuff)
	{
		kfree(pTempBuff);
	}
	
}

static void MS2R_write_memory(u32 addr, u8 *pSendData, u32 uDataSize)
{	
	int nCount = 1;

	if ((0 == addr) || (NULL == pSendData) || (0 == uDataSize))
	{
		printk("[MS2R_write_memory] Parameter Invalid \r\n");
		return;
	}

	#ifndef SEND_CAMERA_FW_BY_I2C_MODE
	if(uDataSize > 256)
	{
		printk("[MS2R_write_memory]----start \r\n");
		iWriteRegI2C(pSendData, uDataSize, MS2R_WRITE_ID);
		printk("[MS2R_write_memory]----finish \r\n");
		return;
	}
	#endif
	
	u8 *pSendCmd = (u8 *)kmalloc(MAX_MEMORY_DATA_SIZE+8, GFP_KERNEL);
	if (NULL == pSendCmd)
	{
		printk("[MS2R_write_memory]----kmalloc  fail \r\n");
		return;
	}

	while (uDataSize > 0)
	{
		*pSendCmd = 0x00;
		*(pSendCmd+1) = COMMAND_ID_WRITE_MEMORY_8BIT_ACCESS;
		*(pSendCmd+2) = (addr >> 24) & 0xFF;
		*(pSendCmd+3) = (addr >> 16) & 0xFF;
		*(pSendCmd+4) = (addr >> 8) & 0xFF;
		*(pSendCmd+5) = addr & 0xFF;

		if (uDataSize >= MAX_MEMORY_DATA_SIZE)
		{
			printk("write to memory ----[%d]---\r\n", nCount);
			nCount++;
			
			*(pSendCmd+6) = MAX_MEMORY_DATA_SIZE >> 8;
			*(pSendCmd+7) = MAX_MEMORY_DATA_SIZE & 0xFF;
			
			memcpy(pSendCmd+8, pSendData, MAX_MEMORY_DATA_SIZE);
			iWriteRegI2C(pSendCmd, MAX_MEMORY_DATA_SIZE+8, MS2R_WRITE_ID);

			pSendData += MAX_MEMORY_DATA_SIZE;
			addr += MAX_MEMORY_DATA_SIZE;
			uDataSize = uDataSize - MAX_MEMORY_DATA_SIZE;
		}
		else
		{
			printk("the last bytes transfer start \r\n");
			*(pSendCmd+6) = (u8)(uDataSize >> 8);
			*(pSendCmd+7) = (u8)(uDataSize & 0xFF);

			memcpy(pSendCmd+8, pSendData, uDataSize);
			iWriteRegI2C(pSendCmd, uDataSize+8, MS2R_WRITE_ID);

			printk("the last bytes transfer finish \r\n");

			uDataSize = 0;
		}
	}

	if (NULL != pSendCmd)
	{
		kfree(pSendCmd);
	}
	
}

/*-------------  read / write category parameter  ----------*/
static int MS2R_read_category_parameter_ram(u8 category, u8 byte)
{
	int ret = 0;
	
	u8 uSendCmd[5] = 
	{
		0x05, 
		COMMAND_ID_READ_CATEGORY_PARAMETER, 
		category, 
		byte, 
		1
	};
	
	u8 uGetData[2] = {0};

	ret = iReadRegI2C(uSendCmd, 5, uGetData, 2, MS2R_WRITE_ID);
	 
	if(ret == -1)
		return ret;
	
    return uGetData[1];
	 
	
}

static int MS2R_write_category_parameter_ram(u8 category, u8 byte, u8 uPara)
{
	int ret = 0;
	u8 uSendCmd[5] =
	{
		0x05, 
		COMMAND_ID_WRITE_CATEGORY_PARAMETER, 
		category, 
		byte, 
		uPara
	};

    ret = iWriteRegI2C(uSendCmd, 5, MS2R_WRITE_ID);
	return ret;
}

static u32 MS2R_read_category_parameter_4Byte(u8 category, u8 byte)
{
	u32 uReturnValue = 0;
	u8 uSendCmd[5] = 
	{
		0x05, 
		COMMAND_ID_READ_CATEGORY_PARAMETER, 
		category, 
		byte, 
		4
	};
	u8 uGetData[5] = {0};

	iReadRegI2C(uSendCmd, 5, uGetData, 5, MS2R_WRITE_ID);

	uReturnValue = ((u32)uGetData[1] << 24)
				    | ((u32)uGetData[2] << 16)
				    | ((u32)uGetData[3] << 8)
				    | (u32)uGetData[4] ;
	
	return uReturnValue;
	
}
static int MS2R_write_category_parameter_4Byte(u8 category, u8 byte, u32 uPara)
{
	int ret = 0;
	u8 uSendCmd[8] = 
	{
		8, 
		COMMAND_ID_WRITE_CATEGORY_PARAMETER, 
		category, 
		byte,
		(u8)((uPara>>24)&0xFF),
		(u8)((uPara>>16)&0xFF),
		(u8)((uPara>>8)&0xFF),
		(u8)(uPara&0xFF)
	};

	ret = iWriteRegI2C(uSendCmd, 8, MS2R_WRITE_ID);
	return ret;
	
}
static void MS2R_HDR_captrue(UINT16 EV)
{
	printk("MS2R_HDR_captrue g_bHDREnable:%d",g_bHDREnable);
	g_bHDREnable = KAL_TRUE;
	MS2R_capture_sequence();
}
static void MS2R_wait_for_interrupt(MS2R_INTERRUPT_TYPE eInterruptType)
{
	u32 getdata = 0;
	u8 int_status = 0;
	int nCount = 0;

	while(nCount<15)
	{
		Sleep(100); //100ms
		
		getdata = MS2R_read_category_parameter_ram(0x00, 0x1C);

		SENSORDB("MS2R_wait_for_interrupt------------[0x%x]----\r\n", getdata);

		if ((0x00 == getdata) || (0xF0 == getdata) || (0xF1 == getdata) || (0xF2 == getdata) 
			|| (0xF3 == getdata) || (0xF4 == getdata) || (0xFA == getdata) || (0xFF == getdata))
		{
			SENSORDB("ISP is busing, Please wait.... \r\n");
			nCount++;
			continue;
		}
		else
		{
			switch (eInterruptType)
			{
				case INT_STATUS_MODE:
					int_status = getdata & 0x01;
					break;
				case INT_STATUS_AF:
					int_status = (getdata>>1)&0x01;
					break;
				case INT_STATUS_ZOOM:
					int_status = (getdata>>2)&0x01;
					break;
				case INT_STATUS_CAPTURE:
					int_status = (getdata>>3)&0x01;
					break;
				case INT_STATUS_FRAMESYNC:
					int_status = (getdata>>4)&0x01;
					break;
				case INT_STATUS_FD:
					int_status = (getdata>>5)&0x01;
					break;
				case INT_STATUS_INIT:
					int_status = (getdata>>6)&0x01;
					break;
				case INT_STATUS_SOUND:
					int_status = (getdata>>7)&0x01;
					break;

				default:
					break;
					
			}

			if (int_status == 0x01)
			{
				SENSORDB("The interrupt [%d] generate!! \r\n", eInterruptType);
				break;
			}
			else
			{
				SENSORDB("The interrupt is not coming , Please wait.....\r\n");
				nCount++;
			}
		}
		
	}

}

#ifdef SEND_CAMERA_FW_BY_I2C_MODE
static void MS2R_send_cameraFW_by_I2C_mode(void)
{
	//set the PLL value 
	//CLKIN = 24MHz
	MS2R_write_category_parameter_4Byte(0x0F, 0x1C, 0x0247036D);
	//
	
	/*send by I2C*/
	//Active I2C write to RAM
	MS2R_write_category_parameter_ram(0x0F, 0x4A, 0x01);
	//
	SENSORDB("Send camera firmware by I2C start >>>>>>> \r\n");
	//send camera firmware by I2C
	MS2R_write_memory(CAMERA_FIRMWARE_TOP_ADDR, rs_ms2, RS_MS2_BIN_SIZE);
	//
	SENSORDB("Send camera firmware by I2C finish <<<<< \r\n");
	
}
#endif


/*------------- transfer data by spi function START --------------*/
static void MS2R_send_by_SPI(u8 *pBuf, u32 uSize)
{
	int nCount = 0;
	
	if ((pBuf == NULL) || (uSize == 0))
	{
		SENSORDB("[MS2R_send_by_SPI] Parameter Invalid \r\n");
		return;
	}
    
	while ((uSize > 0)&&(!force_stop_worker))
	{
		if (uSize > MAX_SPI_TRANSFER_SIZE)
		{
			nCount++;
			SENSORDB("[MS2R_send_by_SPI] -------[%d]------ FW_transfer_mode [%d]\r\n", nCount,FW_transfer_mode);
			
			camera_spi_write(pBuf, MAX_SPI_TRANSFER_SIZE,FW_transfer_mode);

			pBuf += MAX_SPI_TRANSFER_SIZE;
			uSize -= MAX_SPI_TRANSFER_SIZE;
		}
		else
		{
			SENSORDB("[MS2R_send_by_SPI] -------[%d]------ FW_transfer_mode [%d]\r\n", nCount,FW_transfer_mode);
			camera_spi_write(pBuf, uSize,FW_transfer_mode);
			
			uSize = 0;
		}
	}
}

static void MS2R_initialize_general_purpose_port(void)
{
	MS2R_write_memory(MS2R_ADDRESS_OF_PIN1, data_pin1, 64);
	MS2R_write_memory(MS2R_ADDRESS_OF_PIN2, data_pin2, 64);
	MS2R_write_memory(MS2R_ADDRESS_OF_PIN3, data_pin3, 64);
}

static void MS2R_spi_gpio_set(void)
{
	//SPI_CS
	if(mt_set_gpio_mode(GPIO134, GPIO_MODE_00)){SENSORDB("set gpio mode failed!! \n");}
	if(mt_set_gpio_dir(GPIO134, GPIO_DIR_OUT)){SENSORDB("set gpio dir failed!! \n");}
	if(mt_set_gpio_out(GPIO134, GPIO_OUT_ONE)){SENSORDB("set gpio failed!! \n");}
	
	//SPI_MOSI
	if(mt_set_gpio_mode(GPIO135, GPIO_MODE_07)){SENSORDB("set gpio mode failed!! \n");}
	if(mt_set_gpio_dir(GPIO135, GPIO_DIR_OUT)){SENSORDB("set gpio dir failed!! \n");}
	if(mt_set_gpio_pull_enable(GPIO135, GPIO_PULL_ENABLE)){SENSORDB("set gpio pull enable failed!! \n");}
	
	//SPI_CLK
	if(mt_set_gpio_mode(GPIO137, GPIO_MODE_07)){SENSORDB("set gpio mode failed!! \n");}
	if(mt_set_gpio_dir(GPIO137, GPIO_DIR_OUT)){SENSORDB("set gpio dir failed!! \n");}
	if(mt_set_gpio_pull_enable(GPIO137, GPIO_PULL_ENABLE)){SENSORDB("set gpio pull enable failed!! \n");}
}

static void MS2R_spi_gpio_reset(void)
{
	//set dir pull to save power
	//SPI_CS
	if(mt_set_gpio_out(GPIO134, GPIO_OUT_ZERO)){SENSORDB("set gpio failed!! \n");}

	//SPI_MOSI
	if(mt_set_gpio_mode(GPIO135, GPIO_MODE_00)){SENSORDB("set gpio mode failed!! \n");}
	if(mt_set_gpio_dir(GPIO135, GPIO_DIR_IN)){SENSORDB("set gpio dir failed!! \n");}
	if(mt_set_gpio_pull_enable(GPIO135, GPIO_PULL_DISABLE)){SENSORDB("set gpio pull enable failed!! \n");}

	//SPI_CLK
	if(mt_set_gpio_mode(GPIO137, GPIO_MODE_00)){SENSORDB("set gpio mode failed!! \n");}
	if(mt_set_gpio_dir(GPIO137, GPIO_DIR_IN)){SENSORDB("set gpio dir failed!! \n");}
	if(mt_set_gpio_pull_enable(GPIO137, GPIO_PULL_DISABLE)){SENSORDB("set gpio pull enable failed!! \n");}
	
}

static void MS2R_send_cameraFW_by_SIO_mode(void)
{
	int ret = 0;
	//Initialize general_purpose port
do_init:
	MS2R_initialize_general_purpose_port();
	//
	//send SIO Loader Program by I2C
	if(force_stop_worker)
		return;
	MS2R_write_memory(SIO_LOADER_PROGRAM_START_ADDR, ms2_swf, MS2_SWF_BIN_SIZE+(8*29));
	
	if(force_stop_worker)
		return;
	//
	Sleep(5);
	//set start addr
	MS2R_write_category_parameter_4Byte(0x0F, 0x0C, SIO_LOADER_PROGRAM_START_ADDR);
	//
	//start SIO Loader Program
	MS2R_write_category_parameter_ram(0x0F, 0x12, 0x02);
	//
	//select LSI type
	MS2R_write_category_parameter_ram(0x0F, 0x11, 0x01);
	//
	//set the PLL value 
	//CLKIN = 24MHz
	MS2R_write_category_parameter_4Byte(0x0F, 0x1C, 0x0247036D);
	//
	//set SIO setting
	MS2R_write_category_parameter_ram(0x0F, 0x4A, 0x02);
	//
	Sleep(3);


	//set transfer addr
	MS2R_write_category_parameter_4Byte(0x0F, 0x14, CAMERA_FIRMWARE_TOP_ADDR);
	//
	//set the data size to transfer
	MS2R_write_category_parameter_4Byte(0x0F, 0x18, RS_MS2_BIN_SIZE);
	//
	//Please set a pin which is connected to YSIOOUT/PB2 to high level
	MS2R_spi_gpio_set();
	Sleep(1);
	//
	//send camera firmware
	printk("----send firmware by SPI start --->>>>  \r\n");
	//isp_tuning_enable = 1;
	if (isp_tuning_enable)
	{
		printk("---read camera firmware form system file---- \r\n");
		if (0 == MS2R_allocate_camera_firmware_buf())
		{
			if (0 == MS2R_read_camera_firmware_from_file())
			{
				MS2R_send_by_SPI(g_pMS2RBinBuf, RS_MS2_BIN_SIZE);
			}
		}
		MS2R_free_camera_firmware_buf();
	}
	else
	{
		printk("--- read the camera from the Array ------------ \r\n");
		
		if(force_stop_worker)
			return;
		
		MS2R_send_by_SPI(rs_ms2, RS_MS2_BIN_SIZE);
	}
	
	printk("----send firmware by SPI finish ---<<<<  \r\n");
	//
	
	//Please set a pin which is connected to YSIOOUT/PB2 to low level
	//reset spi gpio for save power
	MS2R_spi_gpio_reset();
	//
		
	printk("set start address \r\n");
		//set the start address
	ret = MS2R_write_category_parameter_4Byte(0x0F, 0x0C, 0x01100020);
	printk("check FW ok ret:%d \r\n",ret);
	if((ret != 0)&&(FW_transfer_retrycnt++ < 3))
	{
		FW_transfer_mode = 1;	
		printk("do_spi_transfer \r\n");
#ifdef CHECK_CAMERA_FIRMWARE
		MS2R_check_camera_firmware();
#endif
        if(mt_set_gpio_out(GPIO33, GPIO_OUT_ZERO)){SENSORDB("set gpio failed!! \n");}
		Sleep(5);
		if(mt_set_gpio_out(GPIO33, GPIO_OUT_ONE)){SENSORDB("set gpio failed!! \n");}
		Sleep(10);
		goto do_init;
	}
		//goto do_spi_transfer;
		//start camera program
	ret = MS2R_write_category_parameter_ram(0x0F, 0x12, 0x02);
	//ret = MS2R_write_category_parameter_4Byte(0x0F, 0x0C, 0x01100020);
	printk("check FW ok ret:%d \r\n",ret);
	if((ret != 0)&&(FW_transfer_retrycnt++ < 3))
	{
		FW_transfer_mode = 1;	
#ifdef CHECK_CAMERA_FIRMWARE
		MS2R_check_camera_firmware();
#endif
        printk("do_spi_transfer \r\n");
        if(mt_set_gpio_out(GPIO33, GPIO_OUT_ZERO)){SENSORDB("set gpio failed!! \n");}
		Sleep(5);
		if(mt_set_gpio_out(GPIO33, GPIO_OUT_ONE)){SENSORDB("set gpio failed!! \n");}
		Sleep(10);
		goto do_init;
	}    
#if 1
	//wait for interrupt of system init
	u8 nCount = 0;
	int nValue = 0;
	while(nCount++ < 10)
	{
		Sleep(5);
		nValue = MS2R_read_category_parameter_ram(0x00, 0x1C);
		if (nValue == 0x01)
		{
			printk("start camera program-----Interrupt come \r\n");
			break;
		}
		if((nValue == -1)&&(FW_transfer_retrycnt++ < 3))
		{
			FW_transfer_mode = 1;	
		    FW_transfer_retrycnt++;
			printk("do_spi_transfer \r\n");
			if(mt_set_gpio_out(GPIO33, GPIO_OUT_ZERO)){SENSORDB("set gpio failed!! \n");}
			Sleep(5);
			if(mt_set_gpio_out(GPIO33, GPIO_OUT_ONE)){SENSORDB("set gpio failed!! \n");}
			Sleep(10);
		    goto do_init;
		}
		printk("MS2R_Sensor_Init---Count[%d] Value[%d] \r\n", nCount, nValue);
	}
#endif
	
	   //check FW version
	//Please set a pin which is connected to YSIOOUT/PB2 to low level
	//reset spi gpio for save power

}
/*------------- transfer data by spi function END --------------*/

#ifdef CHECK_CAMERA_FIRMWARE
static void MS2R_check_camera_firmware(void)
{
	SENSORDB("MS2R_check_camera_firmware--------->IN \r\n");
	if (0 == MS2R_allocate_camera_firmware_buf())
	{
		MS2R_read_memory(CAMERA_FIRMWARE_TOP_ADDR, g_pMS2RBinBuf, RS_MS2_BIN_SIZE);
		MS2R_save_camera_firmware_to_system_file();
	}

	MS2R_free_camera_firmware_buf();
	SENSORDB("MS2R_check_camera_firmware--------->OUT \r\n");
}
#endif

static void MS2R_Sensor_Init(void)
{	
	printk("[MS2R_Sensor_Init] start \r\n");
	if(g_eSysMode == SYS_MODE_POWEROFF || g_eSysMode == SYS_MODE_PRE_INITIALIZATION)
	{
		/*------------   boot up progress  -----------*/
		#ifdef SEND_CAMERA_FW_BY_I2C_MODE
		MS2R_send_cameraFW_by_I2C_mode();
		#else
		MS2R_send_cameraFW_by_SIO_mode();
		#endif
		if(force_stop_worker)
			return;
		spin_lock(&ms2r_drv_lock);
	    g_eSysMode = SYS_MODE_INITIALIZATION;
	    spin_unlock(&ms2r_drv_lock);
	}

	MS2R_monitor_sequence();

	//Open gyro help for CAF
	if (0 ==L3G4200D_OpenGyro_preview())
	{
		g_bGyroOpened = true;
	}
	else
	{
		g_bGyroOpened = false;
	}
	
    printk("[MS2R_Sensor_Init] end \r\n");
	
}   /*  MS2R_Sensor_Init  */

static void MS2R_monitor_sequence(void)
{
	printk("[MS2R_monitor_sequence]---------------------> IN \r\n");
	if((g_eSysMode == SYS_MODE_MONITOR)||(g_eSysMode == SYS_MODE_POWEROFF)||(g_eSysMode == SYS_MODE_PRE_INITIALIZATION))
		return;

	bCaptureStart = false;
	bWaveDetectValid = false;
	g_eAEMode = MS2R_AE_MODE_AUTO;
	
	if (SYS_MODE_SINGLE_CAPTURE == g_eSysMode)
	{
		//stop main image data
		MS2R_write_category_parameter_ram(0x0C, 0x09, 0x02);
		bNeedDelay = KAL_TRUE;
		mdelayFrameCnt = 20;
	}

	//MIPI 4lane
	MS2R_write_category_parameter_ram(0x01, 0x3E, 0x04);
	//
	if (SYS_MODE_VIDEO_MONITOR == g_eSysMode)
	{
		//change to parameter setting mode
		MS2R_write_category_parameter_ram(0x00, 0x0B, 0x01);
		//
		Sleep(50);
	}
	//set frame rate dynamic
	MS2R_write_category_parameter_ram(0x03, 0x0A, 0x00);
	//
	g_bHDREnable = KAL_FALSE;
	//monitor size
	MS2R_write_category_parameter_ram(0x01, 0x01, 0x2C);
	//
	//Enable interrupt signal / start YUV output
	MS2R_write_category_parameter_ram(0x00, 0x10, 0x01);
	//	
	//change to monitor mode
	MS2R_write_category_parameter_ram(0x00, 0x0B, 0x02);
	//
	
	spin_lock(&ms2r_drv_lock);
	g_eSysMode = SYS_MODE_MONITOR;
	bISPCapture = KAL_FALSE;
	spin_unlock(&ms2r_drv_lock);
	
    #if defined(USE_WQ_accelerate)||defined(USE_KTHREAD)
	MS2R_update_parm();
	#endif
	#ifdef USE_IRQ
	mt65xx_eint_unmask(EINT_NUM);
	#endif
	printk("[MS2R_monitor_sequence]---------------------> OUT \r\n");
	
}

static void MS2R_video_sequence(void)
{
	printk("[MS2R_video_sequence]---------------------> IN \r\n");
	if((g_eSysMode == SYS_MODE_VIDEO_MONITOR)||(g_eSysMode == SYS_MODE_POWEROFF)||(g_eSysMode == SYS_MODE_PRE_INITIALIZATION))
		return;

	//MIPI 4lane
	MS2R_write_category_parameter_ram(0x01, 0x3E, 0x04);
	//
	//change to parameter setting mode
	MS2R_write_category_parameter_ram(0x00, 0x0B, 0x01);
	//
	Sleep(100);
	//set frame rate 30fps
	MS2R_write_category_parameter_ram(0x03, 0x0A, 0x03);
	//
	//monitor size
	//1920x1080
	MS2R_write_category_parameter_ram(0x01, 0x01, 0x28);
	//
	//Enable interrupt signal / start YUV output
	MS2R_write_category_parameter_ram(0x00, 0x10, 0x01);
	//	
	//change to monitor mode
	MS2R_write_category_parameter_ram(0x00, 0x0B, 0x02);
	//
	
	spin_lock(&ms2r_drv_lock);
	g_eSysMode = SYS_MODE_VIDEO_MONITOR;
	bISPCapture = KAL_FALSE;
	spin_unlock(&ms2r_drv_lock);
	
    #if defined(USE_WQ_accelerate)||defined(USE_KTHREAD)
	MS2R_update_parm();
	#endif
	#ifdef USE_IRQ
	mt65xx_eint_unmask(EINT_NUM);
	#endif
	printk("[MS2R_video_sequence]---------------------> OUT \r\n");
	
}


static void MS2R_capture_sequence(void)
{
	printk("[ms2r_capture_sequence]---------------------> IN \r\n");
	
	if((g_eSysMode != SYS_MODE_MONITOR) && (g_eSysMode != SYS_MODE_VIDEO_MONITOR))
		return;

	bCaptureStart = true;

	#ifdef VENDOR_EDIT
	//LiuBin@MtkCamera, 2012/11/08, Add for avoid caputure bad picture
	//If status is focusing now, Please wait a while to capture
	int nAFstatus = 0;
	while (1)
	{
		nAFstatus = MS2R_read_category_parameter_ram(0x0A, 0x02);
		if(1 != nAFstatus)
		{
			break;
		}
		//HDR capture mode must sync with pipe
		if((KAL_TRUE == g_bHDREnable)&&(1 == nAFstatus))
		{
			Sleep(40);
			continue;
		}
		if ((1 == nAFstatus)&&(KAL_FALSE == g_bHDREnable))
		{
			g_eSysMode = SYS_MODE_PRE_CAPTURE;
			#ifdef USE_KTHREAD
			if(ms2r_init_task)
		      up(&ms2r_sem);
			#endif
			#ifdef USE_WQ_accelerate
		    if(ms2r_workqueue)
		      queue_work(ms2r_workqueue, &ms2r_work); //schedule a work for the first detection
		    #endif
			return;
		}	
	}
	#endif /* VENDOR_EDIT */
	
	//set still capture mode
	if (KAL_TRUE == g_bHDREnable)  //HDR enable 
	{
		MS2R_write_category_parameter_ram(0x0C, 0x00, 0x04);
		MS2R_write_category_parameter_ram(0x03, 0x20, 0x50);
	}
	else
	{
		MS2R_write_category_parameter_ram(0x0C, 0x00, 0x00);
	}
	//
	//Enable interrupt signal
	MS2R_write_category_parameter_ram(0x00, 0x10, 0x08);
	//
	//select main image format
	if (isp_capture_raw_data)
	{
		MS2R_write_category_parameter_ram(0x0B, 0x00, 0x05);
	}
	else
	{
		MS2R_write_category_parameter_ram(0x0B, 0x00, 0x00);
	}
	//
	
	//select main image size
	/*
	0x14---1280x960 SXGA
	0x17---1600x1200 2M
	0x1F---2560x1920 5M
	0x25---3264x2448 8M
	*/
	MS2R_write_category_parameter_ram(0x0B, 0x01, 0x25);
	//
	//start single capture
	MS2R_write_category_parameter_ram(0x00, 0x0B, 0x03);
	//
	
	#ifndef USE_IRQ
	//wait for capture interrupt
	MS2R_wait_for_interrupt(INT_STATUS_CAPTURE);
	//
	bISPCapture = KAL_TRUE;
	//get main image data
	MS2R_write_category_parameter_ram(0x0C, 0x09, 0x01);
	//
	#endif
	
	spin_lock(&ms2r_drv_lock);
	g_eSysMode = SYS_MODE_SINGLE_CAPTURE;
	spin_unlock(&ms2r_drv_lock);	
    #if defined(USE_WQ_accelerate)||defined(USE_KTHREAD)
	ms2r_ctl.update = KAL_TRUE;
	#endif
	printk("[ms2r_capture_sequence]---------------------> OUT \r\n");
	
}

#ifdef USE_IRQ
void ms2r_status_work_callback(struct work_struct *work)
{
	u32 ms2r_status = 0;
	//status check
	
	//unmask at last
	ms2r_status = MS2R_read_category_parameter_ram(0x00, 0x1C);
	if (0x08 == ms2r_status)
	{
		printk("The capture interrupt come----- [0x%x]\r\n", ms2r_status);
		bISPCapture = KAL_TRUE;
		//get main image data
		MS2R_write_category_parameter_ram(0x0C, 0x09, 0x01);
		//update exif info
		Sleep(10);
		MS2R_update_exif_info();
	}
	
	printk("ms2r_status_work_callback 0x%x\r\n",ms2r_status);
	mt65xx_eint_unmask(EINT_NUM);
}

#endif
/*****************************************************************************/
/* Windows Mobile Sensor Interface */
/*****************************************************************************/
#ifdef USE_KTHREAD
static int ms2r_init_kthread(void *data)
{
		int hpd_result = 0;
        struct sched_param param = { .sched_priority = RTPM_PRIO_SCRN_UPDATE };
		sema_init(&ms2r_sem, 0);
        sched_setscheduler(current, SCHED_RR, &param);
        for( ;; ) 
        {
			down(&ms2r_sem);
			sema_init(&close_sem, 0);
			ms_work_mode = true;
			if(g_eSysMode == SYS_MODE_PRE_INITIALIZATION)
			{
				MS2R_Sensor_Init();
			}else if(g_eSysMode == SYS_MODE_PRE_CAPTURE){
				#define POLL_CNT (100)
				printk("ms2r_work_callback! capture cmd start!\r\n");
				int poll_cnt = 0;
				//Cancel AF when focusing
				while(1 == MS2R_read_category_parameter_ram(0x0A, 0x02)&&(poll_cnt++ < POLL_CNT))
				{
					MS2R_write_category_parameter_ram(0x0A, 0x02, 0x00);
					Sleep(40);
				}
				
				//set still capture mode
				if (KAL_TRUE == g_bHDREnable)  //HDR enable 
				{
					MS2R_write_category_parameter_ram(0x0C, 0x00, 0x04);
					MS2R_write_category_parameter_ram(0x03, 0x20, 0x50);
				}
				else
				{
					MS2R_write_category_parameter_ram(0x0C, 0x00, 0x00);
				}
				//
				//Enable interrupt signal
				MS2R_write_category_parameter_ram(0x00, 0x10, 0x08);
				//
				//select main image format
				if (isp_capture_raw_data)
				{
					MS2R_write_category_parameter_ram(0x0B, 0x00, 0x05);
				}
				else
				{
					MS2R_write_category_parameter_ram(0x0B, 0x00, 0x00);
				}
				
				/*
				0x14---1280x960 SXGA
				0x17---1600x1200 2M
				0x1F---2560x1920 5M
				0x25---3264x2448 8M
				*/
				MS2R_write_category_parameter_ram(0x0B, 0x01, 0x25);
				MS2R_write_category_parameter_ram(0x00, 0x0B, 0x03);

				#ifndef USE_IRQ
				//wait for capture interrupt
				MS2R_wait_for_interrupt(INT_STATUS_CAPTURE);
				//
				bISPCapture = KAL_TRUE;
				//get main image data
				MS2R_write_category_parameter_ram(0x0C, 0x09, 0x01);
				//
				#endif
				
				printk("ms2r_work_callback! capture cmd finish!\r\n");
				spin_lock(&ms2r_drv_lock);
				g_eSysMode = SYS_MODE_SINGLE_CAPTURE;
				spin_unlock(&ms2r_drv_lock);
				ms2r_ctl.update = KAL_TRUE;
			}  
			ms_work_mode = false;
			up(&close_sem);
        }

        return 0;
}
#endif
#ifdef USE_WQ_accelerate
void ms2r_work_callback(struct work_struct *work)
{   if(g_eSysMode == SYS_MODE_PRE_INITIALIZATION)
	{
	  MS2R_Sensor_Init();
    }else if(g_eSysMode == SYS_MODE_PRE_CAPTURE){
        #define POLL_CNT (100)
        printk("ms2r_work_callback! capture cmd start!\r\n");
		int poll_cnt = 0;
		//MS2R_write_category_parameter_ram(0x0A, 0x02, 0x00);
		while(1 == MS2R_read_category_parameter_ram(0x0A, 0x02)&&(poll_cnt++ < POLL_CNT))
		{
			Sleep(33);
		}
	      //set still capture mode
		if (KAL_TRUE == g_bHDREnable)  //HDR enable 
		{
			MS2R_write_category_parameter_ram(0x0C, 0x00, 0x04);
			MS2R_write_category_parameter_ram(0x03, 0x20, 0x50);
		}
		else
		{
			MS2R_write_category_parameter_ram(0x0C, 0x00, 0x00);
		}
		//
		//Enable interrupt signal
		MS2R_write_category_parameter_ram(0x00, 0x10, 0x08);
		//
		//select main image format
		if (isp_capture_raw_data)
		{
			MS2R_write_category_parameter_ram(0x0B, 0x00, 0x05);
		}
		else
		{
			MS2R_write_category_parameter_ram(0x0B, 0x00, 0x00);
		}
		//
		
		/*
		0x14---1280x960 SXGA
		0x17---1600x1200 2M
		0x1F---2560x1920 5M
		0x25---3264x2448 8M
		*/
		MS2R_write_category_parameter_ram(0x0B, 0x01, 0x25);
		MS2R_write_category_parameter_ram(0x00, 0x0B, 0x03);
		#ifndef USE_IRQ
		//wait for capture interrupt
		MS2R_wait_for_interrupt(INT_STATUS_CAPTURE);
		//
		bISPCapture = KAL_TRUE;
		//get main image data
		MS2R_write_category_parameter_ram(0x0C, 0x09, 0x01);
		//
		#endif
		printk("ms2r_work_callback! capture cmd finish!\r\n");
		spin_lock(&ms2r_drv_lock);
		g_eSysMode = SYS_MODE_SINGLE_CAPTURE;
		spin_unlock(&ms2r_drv_lock);
		ms2r_ctl.update = KAL_TRUE;
    }
}
#endif
/*************************************************************************
* FUNCTION
*   MS2RCheckIsAlive
*
* DESCRIPTION
*   This function Check MS2R is alive
*
* PARAMETERS
* 	None
*
* RETURNS
*   Bool
*
* GLOBALS AFFECTED
*
*************************************************************************/
static BOOL MS2RCheckIsAlive(void) 
{
	BOOL bAlive = FALSE;
	
    int nRetry = 3;
	u32 uWriteData = 0x23;
	u32 uReadData = 0;

	//check if I2C communication is OK
	//before send camera firmware, only category 0x0F is available
	do 
	{
        MS2R_write_category_parameter_ram(0x0F, 0x0C, uWriteData);
		uReadData = MS2R_read_category_parameter_ram(0x0F, 0x0C);
		if (uReadData == uWriteData)
		{
			bAlive = TRUE;
			printk("MS2R is alive! \r\n");
			break;
		}
		
		nRetry--;
		
    } while (nRetry > 0);
	#ifdef USE_KTHREAD
	if(!ms2r_kthread&&bAlive)
	{
		
		ms2r_init_task = kthread_create(ms2r_init_kthread, NULL, "MS2R_kthread"); 
		wake_up_process(ms2r_init_task);
		sema_init(&close_sem,0);
		ms2r_kthread = KAL_TRUE;
	}
	#endif
	#ifdef USE_WQ_accelerate
	if(!ms2r_queue&&bAlive)
	{
		ms2r_workqueue = create_singlethread_workqueue("ms2r");
		INIT_WORK(&ms2r_work, ms2r_work_callback);
		ms2r_queue = KAL_TRUE;
	}
	#endif
    return bAlive;
	
}


/*************************************************************************
* FUNCTION
*   MS2ROpen
*
* DESCRIPTION
*   This function initialize the registers of CMOS sensor
*
* PARAMETERS
*   None
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/

UINT32 MS2ROpen(void)
{
    // first check MS2R is alive
    if (FALSE == MS2RCheckIsAlive())
    {
		printk("MS2R is not alive \r\n");
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	force_stop_worker = false;
	bCaptureStart = false;
	#ifdef USE_IRQ
	if(!ms2r_int)
	{
		ms2r_status_workqueue = create_singlethread_workqueue("ms2r_status");
		INIT_WORK(&ms2r_status_work, ms2r_status_work_callback);
		if(ms2r_status_workqueue != NULL)
			ms2r_install_IRQ();
		ms2r_int = KAL_TRUE;
	}
	#endif
	
	
    #if defined(USE_WQ_accelerate)||defined(USE_KTHREAD)	
	if(!ms2r_ctl_init)
	{
		ms2r_ctl.Effect = MEFFECT_OFF;  
		ms2r_ctl.Exposure = AE_EV_COMP_00;  
		ms2r_ctl.Wb = AWB_MODE_AUTO;  
		ms2r_ctl.Stobe = FLASHLIGHT_FORCE_OFF;  
		ms2r_ctl.Scene_mode = AE_MODE_AUTO;  
		ms2r_ctl.Hue = ISP_HUE_MIDDLE;  
		ms2r_ctl.Contrast = ISP_CONTRAST_MIDDLE;  
		ms2r_ctl.Edge = ISP_EDGE_MIDDLE;  
		ms2r_ctl.Brightness = ISP_BRIGHT_MIDDLE;  
		ms2r_ctl.Sat = ISP_SAT_MIDDLE;  
		ms2r_ctl.AE_HDR = AE_EV_HDR_00;  
		ms2r_ctl.AF_mode = AF_MODE_AFC; 
	    ms2r_ctl.update = KAL_TRUE;  
		ms2r_ctl_init = KAL_TRUE;
	}
	#ifdef USE_KTHREAD
	if(ms2r_kthread)
	{
		spin_lock(&ms2r_drv_lock);
		g_eSysMode = SYS_MODE_PRE_INITIALIZATION;
		spin_unlock(&ms2r_drv_lock);
		if(ms2r_init_task)
		   up(&ms2r_sem);
		return ERROR_NONE;
	}
	#endif
    #ifdef USE_WQ_accelerate
	if(ms2r_queue)
	{
	   
		spin_lock(&ms2r_drv_lock);
		g_eSysMode = SYS_MODE_PRE_INITIALIZATION;
		spin_unlock(&ms2r_drv_lock);
		if(ms2r_workqueue)
		   queue_work(ms2r_workqueue, &ms2r_work); //schedule a work for the first detection
		return ERROR_NONE;
	}
	#endif
	#endif
    MS2R_Sensor_Init();
	
#ifdef VENDOR_EDIT
//feng.hu@mtk camera team modified for ms2r continuous af
	mmIsCAFin = false;
	mdelayDetectCnt = ANTI_WDV_SHAKE_DELAY;
	mdelayAFCnt = DO_AF_DELAY;
	m3AFrameCnt= 0;
	m3ASingleAFLock = false;
	m3ASingleFrameCnt = 0;
#endif

	spin_lock(&FaceAE_lock);
	g_uFaceAECnt = 0;
	g_uFaceAECntPre = 0;
	spin_unlock(&FaceAE_lock);

    return ERROR_NONE;
	
}

/*************************************************************************
* FUNCTION
*   MS2RClose
*
* DESCRIPTION
*   This function is to turn off sensor module power.
*
* PARAMETERS
*   None
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 MS2RClose(void)
{
	if(g_eSysMode == SYS_MODE_POWEROFF)
		return ERROR_NONE;
	#ifdef USE_KTHREAD
	if(ms_work_mode == true)
	{
		force_stop_worker = true;
		down(&close_sem);
		force_stop_worker = false;
	}
	#endif
	FW_transfer_retrycnt = 0;
    #if defined(USE_WQ_accelerate)
	if((g_eSysMode == SYS_MODE_INITIALIZATION)||(g_eSysMode == SYS_MODE_PRE_CAPTURE)||(g_eSysMode == SYS_MODE_PRE_INITIALIZATION))
	{
		if (work_pending(&ms2r_work))
		{
			printk("Camera is reconnect, cancel the work \r\n");
			cancel_work_sync(&ms2r_work);
		}	
	}
	#endif
	
	#ifdef USE_IRQ
	mt65xx_eint_mask(EINT_NUM);
	#endif
	#ifdef VENDOR_EDIT
	//LiuBin@MtkCamera, 2012/11/09, Add for close flashlight, need to sleep 10ms for ISP send closing flashlight command
	MS2R_write_category_parameter_ram(0x0B, 0x1F, 0x00);
	MS2R_write_category_parameter_ram(0x0D, 0x28, 0x00);
	Sleep(10);
	#endif /* VENDOR_EDIT */

	if (g_bGyroOpened)
	{
		L3G4200D_CloseGyro_preview();
		g_bGyroOpened = false;
	}

	bCaptureStart = false;
	
	spin_lock(&ms2r_drv_lock);
	g_eSysMode = SYS_MODE_POWEROFF;
	spin_unlock(&ms2r_drv_lock);
	
    #if defined(USE_WQ_accelerate)||defined(USE_KTHREAD)
	ms2r_ctl.update = KAL_TRUE;
	ms2r_ctl.Scene_mode = SCENE_MODE_NORMAL;
	#endif
    return ERROR_NONE;
}	/* MS2RClose() */


/*************************************************************************
* FUNCTION
*   MS2RPreview
*
* DESCRIPTION
*   This function start the sensor preview.
*
* PARAMETERS
*   *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 MS2RPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	printk("[MS2RPreview]---------------------> IN \r\n");
    memcpy(&MS2RSensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	if (isp_capture_raw_data)
	{
		image_window->GrabStartX= MS2R_CAP_RAW_PREVIEW_START_X;
    	image_window->GrabStartY= MS2R_CAP_RAW_PREVIEW_START_Y;
	}
	else
	{
		image_window->GrabStartX= MS2R_PREVIEW_START_X;
    	image_window->GrabStartY= MS2R_PREVIEW_START_Y;
	}
    image_window->ExposureWindowWidth= MS2R_REAL_PV_WIDTH;
    image_window->ExposureWindowHeight= MS2R_REAL_PV_HEIGHT;
	MS2R_monitor_sequence();
	printk("[MS2RPreview]---------------------> OUT \r\n");
    return ERROR_NONE;
	
}	/* MS2RPreview() */

/*************************************************************************
* FUNCTION
*   MS2RVideo
*
* DESCRIPTION
*   This function start the sensor preview.
*
* PARAMETERS
*   *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 MS2RVideo(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	printk("[MS2RVideo]---------------------> IN \r\n");
    memcpy(&MS2RSensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	if (isp_capture_raw_data)
	{
		image_window->GrabStartX= MS2R_CAP_RAW_VIDEO_START_X;
    	image_window->GrabStartY= MS2R_CAP_RAW_VIDEO_START_Y;
	}
	else
	{
		image_window->GrabStartX= MS2R_VIDEO_START_X;
    	image_window->GrabStartY= MS2R_VIDEO_START_Y;
	}
    image_window->ExposureWindowWidth= MS2R_REAL_VIDEO_WIDTH;
    image_window->ExposureWindowHeight= MS2R_REAL_VIDEO_HEIGHT;
	MS2R_video_sequence();
	printk("[MS2RVideo]---------------------> OUT \r\n");
    return ERROR_NONE;
	
}	/* MS2RVideo() */


UINT32 MS2RCapture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	printk("[MS2RCapture]---------------------> IN \r\n");
    memcpy(&MS2RSensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	
	if (isp_capture_raw_data)
	{
		image_window->GrabStartX= MS2R_CAP_RAW_DATA_START_X;
    	image_window->GrabStartY= MS2R_CAP_RAW_DATA_START_Y;
		image_window->ExposureWindowWidth= MS2R_CAP_RAW_DATA_WIDTH;
    	image_window->ExposureWindowHeight= MS2R_CAP_RAW_DATA_HEIGHT;
	}
	else
	{
		image_window->GrabStartX= MS2R_CAPTURE_START_X;
    	image_window->GrabStartY= MS2R_CAPTURE_START_Y;
    	image_window->ExposureWindowWidth= MS2R_REAL_CAP_WIDTH;
    	image_window->ExposureWindowHeight= MS2R_REAL_CAP_HEIGHT;
	}
	
	MS2R_capture_sequence();
	printk("[MS2RCapture]---------------------> OUT \r\n");
    return ERROR_NONE;
	
}	/* MS2RCapture() */

UINT32 MS2RGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
	
	pSensorResolution->SensorPreviewWidth	= MS2R_REAL_PV_WIDTH;
    pSensorResolution->SensorPreviewHeight	= MS2R_REAL_PV_HEIGHT;

	if (isp_capture_raw_data)
	{
		pSensorResolution->SensorFullWidth		= MS2R_CAP_RAW_DATA_WIDTH;
    	pSensorResolution->SensorFullHeight		= MS2R_CAP_RAW_DATA_HEIGHT;
	}
	else
	{
		pSensorResolution->SensorFullWidth		= MS2R_REAL_CAP_WIDTH;
    	pSensorResolution->SensorFullHeight		= MS2R_REAL_CAP_HEIGHT;
	}

	pSensorResolution->SensorVideoWidth  	= MS2R_REAL_VIDEO_WIDTH;
	pSensorResolution->SensorVideoHeight	= MS2R_REAL_VIDEO_HEIGHT;

    return ERROR_NONE;
	
}   /* MS2RGetResolution() */

UINT32 MS2RGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
						MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
						MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	switch(ScenarioId){
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			if (isp_capture_raw_data)
			{
				pSensorInfo->SensorPreviewResolutionX = MS2R_CAP_RAW_DATA_WIDTH;
				pSensorInfo->SensorPreviewResolutionY = MS2R_CAP_RAW_DATA_HEIGHT;
				pSensorInfo->SensorFullResolutionX = MS2R_CAP_RAW_DATA_WIDTH;
				pSensorInfo->SensorFullResolutionY = MS2R_CAP_RAW_DATA_HEIGHT;
			}
			else
			{
				pSensorInfo->SensorPreviewResolutionX = MS2R_REAL_CAP_WIDTH;
				pSensorInfo->SensorPreviewResolutionY = MS2R_REAL_CAP_HEIGHT;
				pSensorInfo->SensorFullResolutionX = MS2R_REAL_CAP_WIDTH;
				pSensorInfo->SensorFullResolutionY = MS2R_REAL_CAP_HEIGHT;
			}
			pSensorInfo->SensorCameraPreviewFrameRate=15;
			break;

		default:
	        pSensorInfo->SensorPreviewResolutionX = MS2R_REAL_PV_WIDTH;
	        pSensorInfo->SensorPreviewResolutionY = MS2R_REAL_PV_HEIGHT;
			if (isp_capture_raw_data)
			{
				pSensorInfo->SensorFullResolutionX = MS2R_CAP_RAW_DATA_WIDTH;
				pSensorInfo->SensorFullResolutionY = MS2R_CAP_RAW_DATA_HEIGHT;
			}
			else
			{
				pSensorInfo->SensorFullResolutionX = MS2R_REAL_CAP_WIDTH;
				pSensorInfo->SensorFullResolutionY = MS2R_REAL_CAP_HEIGHT;
			}
			pSensorInfo->SensorCameraPreviewFrameRate = 24;
			break;
	}

    pSensorInfo->SensorVideoFrameRate=24;
    pSensorInfo->SensorStillCaptureFrameRate=8;
    pSensorInfo->SensorWebCamCaptureFrameRate=15;
    pSensorInfo->SensorResetActiveHigh=FALSE;
    pSensorInfo->SensorResetDelayCount=5;
	if (isp_capture_raw_data)
	{
		pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_UYVY;
	}
	else
	{
		pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_YUYV;
	}
	
    pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorInterruptDelayLines = 1;
    pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_MIPI;
	
    pSensorInfo->CaptureDelayFrame = 0; 
    pSensorInfo->PreviewDelayFrame = 0; 
    pSensorInfo->VideoDelayFrame = 5; 
    pSensorInfo->SensorMasterClockSwitch = 0; 
    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA;      
    pSensorInfo->AEShutDelayFrame = 0;		    /* The frame of setting shutter default 0 for TG int */
    pSensorInfo->AESensorGainDelayFrame = 0;     /* The frame of setting sensor gain */
    pSensorInfo->AEISPGainDelayFrame = 2;	
	   
    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockDividCount=	5;
            pSensorInfo->SensorClockRisingCount= 0;
            pSensorInfo->SensorClockFallingCount= 2;
            pSensorInfo->SensorPixelClockCount= 3;
            pSensorInfo->SensorDataLatchCount= 2;
			if (isp_capture_raw_data)
			{
				pSensorInfo->SensorGrabStartX = MS2R_CAP_RAW_PREVIEW_START_X; 
            	pSensorInfo->SensorGrabStartY = MS2R_CAP_RAW_PREVIEW_START_Y; 
			}
			else
			{
				pSensorInfo->SensorGrabStartX = MS2R_PREVIEW_START_X; 
            	pSensorInfo->SensorGrabStartY = MS2R_PREVIEW_START_Y; 
			}
            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_4_LANE;			
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
			pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
			pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
            pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x 
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockDividCount=	5;
            pSensorInfo->SensorClockRisingCount= 0;
            pSensorInfo->SensorClockFallingCount= 2;
            pSensorInfo->SensorPixelClockCount= 3;
            pSensorInfo->SensorDataLatchCount= 2;
			if (isp_capture_raw_data)
			{
				pSensorInfo->SensorGrabStartX = MS2R_CAP_RAW_DATA_START_X;
            	pSensorInfo->SensorGrabStartY = MS2R_CAP_RAW_DATA_START_Y; 
			}
			else
			{
				pSensorInfo->SensorGrabStartX = MS2R_CAPTURE_START_X;
				pSensorInfo->SensorGrabStartY = MS2R_CAPTURE_START_Y;
			}    			
            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_4_LANE;			
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0; 
            pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14; 
            pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0; 
            pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
            pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        default:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockDividCount=	3;
            pSensorInfo->SensorClockRisingCount= 0;
            pSensorInfo->SensorClockFallingCount= 2;
            pSensorInfo->SensorPixelClockCount= 3;
            pSensorInfo->SensorDataLatchCount= 2;
            pSensorInfo->SensorGrabStartX = 1; 
            pSensorInfo->SensorGrabStartY = 1;             
            break;
    }

    memcpy(pSensorConfigData, &MS2RSensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));

    return ERROR_NONE;
}   /* MS2RGetInfo() */


UINT32 MS2RControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
						MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			MS2RPreview(pImageWindow, pSensorConfigData);
			break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			MS2RVideo(pImageWindow, pSensorConfigData);
			break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			MS2RCapture(pImageWindow, pSensorConfigData);
			break;
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			break;
        default:
            return ERROR_INVALID_SCENARIO_ID;
    }
    return ERROR_NONE;
} /* MS2RControl() */

static void MS2R_set_AF_mode(UINT16 uPara)
{
	if (g_eSysMode != SYS_MODE_MONITOR)
	{
		
        #if defined(USE_WQ_accelerate)||defined(USE_KTHREAD)
		ms2r_ctl.AF_mode = uPara;
		#endif
		return;
	}
	switch (uPara)
	{
		case AF_MODE_AFS:
		{
			SENSORDB("MS2R_set_AF_mode ------------- [Single] \r\n");
			MS2R_write_category_parameter_ram(0x0A, 0x00, 0x01);
			break;
		}
		case AF_MODE_AFC:
		{
			SENSORDB("MS2R_set_AF_mode ------------- [Constant] \r\n");
			MS2R_write_category_parameter_ram(0x0A, 0x00, 0x01);
			break;
		}
		case AF_MODE_INFINITY:
		{
			SENSORDB("MS2R_set_AF_mode ------------- [Infinity] \r\n");
			MS2R_write_category_parameter_ram(0x0A, 0x00, 0x01);
			break;
		}

		default:
			break;
	}
	
    #if defined(USE_WQ_accelerate)||defined(USE_KTHREAD)
	ms2r_ctl.AF_mode = uPara;
	#endif
}

static void MS2R_set_flashlight_mode(UINT32 uPara)
{
	if ((SYS_MODE_MONITOR != g_eSysMode) 
		&& (SYS_MODE_SINGLE_CAPTURE != g_eSysMode))
	{
		#if defined(USE_WQ_accelerate)||defined(USE_KTHREAD)
		ms2r_ctl.Stobe = uPara;
		#endif
		return;
	}
	switch (uPara)
	{
		case FLASHLIGHT_AUTO:
		{
			SENSORDB(" MS2R_set_flashlight_mode -------[AUTO] \r\n");
			MS2R_write_category_parameter_ram(0x0D, 0x28, 0x00);
			Sleep(10);
			MS2R_write_category_parameter_ram(0x0B, 0x1F, 0x02);
			break;
		}
		case FLASHLIGHT_FORCE_ON:
		{
			SENSORDB(" MS2R_set_flashlight_mode -------[ON] \r\n");
			MS2R_write_category_parameter_ram(0x0D, 0x28, 0x00);
			Sleep(10);
			MS2R_write_category_parameter_ram(0x0B, 0x1F, 0x03);
			break;
		}
		case 556://Torch off
		case FLASHLIGHT_FORCE_OFF:
		{
			SENSORDB(" MS2R_set_flashlight_mode -------[OFF] \r\n");
			MS2R_write_category_parameter_ram(0x0D, 0x28, 0x00);
			MS2R_write_category_parameter_ram(0x0B, 0x1F, 0x00);
			break;
		}
		//When camera not in preview state, can use 555 to enable torch
		case 555:
		case FLASHLIGHT_TORCH:
		{
			SENSORDB(" MS2R_set_flashlight_mode -------[TORCH] \r\n");
			MS2R_write_category_parameter_ram(0x0B, 0x1F, 0x00);
			MS2R_write_category_parameter_ram(0x0D, 0x28, 0x01);
			break;
		}

		default:
			break;
	}
    #if defined(USE_WQ_accelerate)||defined(USE_KTHREAD)
	ms2r_ctl.Stobe = uPara;
	#endif

	MS2RFeatureInfo.m_uStobe = uPara;
	
}

static void MS2R_set_param_wb(UINT16 uPara)
{
	if (g_eSysMode != SYS_MODE_MONITOR)
	{
		#if defined(USE_WQ_accelerate)||defined(USE_KTHREAD)
		ms2r_ctl.Wb = uPara;
		#endif
		return;
	}
	switch (uPara)
	{
		case AWB_MODE_OFF:
			break;
		case AWB_MODE_AUTO:
		{
			SENSORDB("MS2R_set_param_wb------[Auto]---- \r\n");
			MS2R_write_category_parameter_ram(0x06, 0x02, 0x01);
			break;
		}
		case AWB_MODE_INCANDESCENT:
		{
			SENSORDB("MS2R_set_param_wb------[Incandescent]---- \r\n");
			MS2R_write_category_parameter_ram(0x06, 0x02, 0x02);
			MS2R_write_category_parameter_ram(0x06, 0x03, 0x01);
			break;
		}
		case AWB_MODE_FLUORESCENT:
		{
			SENSORDB("MS2R_set_param_wb------[Fluorescent]---- \r\n");
			MS2R_write_category_parameter_ram(0x06, 0x02, 0x02);
			MS2R_write_category_parameter_ram(0x06, 0x03, 0x02);
			break;
		}
		case AWB_MODE_DAYLIGHT:
		{
			SENSORDB("MS2R_set_param_wb------[Daylight]---- \r\n");
			MS2R_write_category_parameter_ram(0x06, 0x02, 0x02);
			MS2R_write_category_parameter_ram(0x06, 0x03, 0x04);
			break;
		}
		case AWB_MODE_CLOUDY_DAYLIGHT:
		{
			SENSORDB("MS2R_set_param_wb------[Cloudy]---- \r\n");
			MS2R_write_category_parameter_ram(0x06, 0x02, 0x02);
			MS2R_write_category_parameter_ram(0x06, 0x03, 0x05);
			break;
		}
		
		default:
			break;
		
	}
  	#if defined(USE_WQ_accelerate)||defined(USE_KTHREAD)
	ms2r_ctl.Wb = uPara;
	#endif

	MS2RFeatureInfo.m_uWb = uPara;
	
}

static void MS2R_set_param_ev(UINT16 uPara)
{
	#ifndef OPPO_CMCC_TEST
	return;
	#endif
	if (g_eSysMode != SYS_MODE_MONITOR)
	{
		#if defined(USE_WQ_accelerate)||defined(USE_KTHREAD)
		ms2r_ctl.Exposure = uPara;
		#endif
		return;
	}
	switch (uPara)
	{
		case AE_EV_COMP_n30:
		{
			SENSORDB("MS2R_set_param_ev------[-3.0EV]---- \r\n");
			#if 1
			MS2R_write_category_parameter_ram(0x03, 0x09, 0x0F);
			#else
			MS2R_write_category_parameter_ram(0x02, 0x1F, 0x01);
			#endif
			break;
		}
		case AE_EV_COMP_n20:
		{
			SENSORDB("MS2R_set_param_ev------[-2.0EV]---- \r\n");
			#if 1
			MS2R_write_category_parameter_ram(0x03, 0x09, 0x14);
			#else
			MS2R_write_category_parameter_ram(0x02, 0x1F, 0x02);
			#endif
			break;
		}
		case AE_EV_COMP_n10:
		{
			SENSORDB("MS2R_set_param_ev------[-1.0EV]---- \r\n");
			#if 1
			MS2R_write_category_parameter_ram(0x03, 0x09, 0x19);
			#else
			MS2R_write_category_parameter_ram(0x02, 0x1F, 0x03);
			#endif
			break;
		}
		case AE_EV_COMP_00:
		{
			SENSORDB("MS2R_set_param_ev------[0EV]---- \r\n");
			MS2R_write_category_parameter_ram(0x03, 0x09, 0x1E);
			break;
		}
		case AE_EV_COMP_10:
		{
			SENSORDB("MS2R_set_param_ev------[+1.0EV]---- \r\n");
			#if 1
			MS2R_write_category_parameter_ram(0x03, 0x09, 0x23);
			#else
			MS2R_write_category_parameter_ram(0x02, 0x0F, 0x01);
			MS2R_write_category_parameter_ram(0x02, 0x10, 0x01);
			#endif
			break;
		}
		case AE_EV_COMP_20:
		{
			SENSORDB("MS2R_set_param_ev------[+2.0EV]---- \r\n");
			#if 1
			MS2R_write_category_parameter_ram(0x03, 0x09, 0x28);
			#else
			MS2R_write_category_parameter_ram(0x02, 0x0F, 0x02);
			MS2R_write_category_parameter_ram(0x02, 0x10, 0x01);
			#endif
			break;
		}
		case AE_EV_COMP_30:
		{
			SENSORDB("MS2R_set_param_ev------[+3.0EV]---- \r\n");
			#if 1
			MS2R_write_category_parameter_ram(0x03, 0x09, 0x2D);
			#else
			MS2R_write_category_parameter_ram(0x02, 0x0F, 0x03);
			MS2R_write_category_parameter_ram(0x02, 0x10, 0x01);
			#endif
			break;
		}

		default:
			break;
			
	}
  	#if defined(USE_WQ_accelerate)||defined(USE_KTHREAD)
	ms2r_ctl.Exposure = uPara;
	#endif

	MS2RFeatureInfo.m_uExposure = uPara;
	
}

static void MS2R_set_param_effect(UINT16 uPara)
{
	if (g_eSysMode != SYS_MODE_MONITOR)
	{
		#if defined(USE_WQ_accelerate)||defined(USE_KTHREAD)
		ms2r_ctl.Effect = uPara;
		#endif
		return;
	}
	switch (uPara)
	{
		case MEFFECT_OFF:
		{
			MS2R_write_category_parameter_ram(0x02, 0x0B, 0x00);
			break;
		}
		case MEFFECT_MONO: //(Black & White)
		{
			SENSORDB("MS2R_set_param_effect------[Black & White]---- \r\n");
			MS2R_write_category_parameter_ram(0x02, 0x0B, 0x01);
			MS2R_write_category_parameter_ram(0x02, 0x09, 0x00);
			MS2R_write_category_parameter_ram(0x02, 0x0A, 0x00);
			break;
		}
		case MEFFECT_SEPIA:
		{
			SENSORDB("MS2R_set_param_effect------[MEFFECT_SEPIA]---- \r\n");
			MS2R_write_category_parameter_ram(0x02, 0x0B, 0x01);
			MS2R_write_category_parameter_ram(0x02, 0x09, 0xD8);
			MS2R_write_category_parameter_ram(0x02, 0x0A, 0x18);
			break;
		}
		case MEFFECT_NEGATIVE:
		{
			SENSORDB("MS2R_set_param_effect------[MEFFECT_NEGATIVE]---- \r\n");
			MS2R_write_category_parameter_ram(0x02, 0x0B, 0x01);
			MS2R_write_category_parameter_ram(0x02, 0x09, 0xD0);
			MS2R_write_category_parameter_ram(0x02, 0x0A, 0x30);
			break;
		}
		case MEFFECT_SEPIAGREEN:
		{
			SENSORDB("MS2R_set_param_effect------[MEFFECT_SEPIAGREEN]---- \r\n");
			MS2R_write_category_parameter_ram(0x02, 0x0B, 0x01);
			MS2R_write_category_parameter_ram(0x02, 0x09, 0xE0);
			MS2R_write_category_parameter_ram(0x02, 0x0A, 0xE0);
			break;
		}
		case MEFFECT_SEPIABLUE:
		{
			SENSORDB("MS2R_set_param_effect------[MEFFECT_SEPIABLUE]---- \r\n");
			MS2R_write_category_parameter_ram(0x02, 0x0B, 0x01);
			MS2R_write_category_parameter_ram(0x02, 0x09, 0x40);
			MS2R_write_category_parameter_ram(0x02, 0x0A, 0x00);
			break;
		}
		case MEFFECT_AQUA:
		{
			SENSORDB("MS2R_set_param_effect------[MEFFECT_AQUA]---- \r\n");
			MS2R_write_category_parameter_ram(0x02, 0x0B, 0x01);
			MS2R_write_category_parameter_ram(0x02, 0x09, 0xD0);
			MS2R_write_category_parameter_ram(0x02, 0x0A, 0x30);
			break;
		}

		default:
			break;
	}
	#if defined(USE_WQ_accelerate)||defined(USE_KTHREAD)
	ms2r_ctl.Effect = uPara;
	#endif

	MS2RFeatureInfo.m_uEffect = uPara;
	
}

static void MS2R_set_ae_mode(UINT16 uPara)
{
	return;
	if (g_eSysMode != SYS_MODE_MONITOR)
	{
		#if defined(USE_WQ_accelerate)||defined(USE_KTHREAD)
		ms2r_ctl.AE_mode = uPara;
		#endif
		return;
	}
	switch (uPara)
	{
		case AE_MODE_OFF:
		{
			SENSORDB("MS2R_set_ae_mode ------------- [OFF] \r\n");
			//MS2R_write_category_parameter_ram(0x03, 0x01, 0x00);
			break;
		}
		case AE_MODE_AUTO:
		{
			SENSORDB("MS2R_set_ae_mode ------------- [AUTO] \r\n");
			//MS2R_write_category_parameter_ram(0x03, 0x01, 0x01);
			break;
		}

		default:
			break;
	}
	#if defined(USE_WQ_accelerate)||defined(USE_KTHREAD)
	ms2r_ctl.AE_mode = uPara;
	#endif
}

static void MS2R_set_isp_contrast(UINT16 uPara)
{
	#ifndef OPPO_CMCC_TEST
	return;
	#endif
	if (g_eSysMode != SYS_MODE_MONITOR)
	{
		#if defined(USE_WQ_accelerate)||defined(USE_KTHREAD)
		ms2r_ctl.Contrast = uPara;
		#endif
		return;
	}
	switch (uPara)
	{
		case ISP_CONTRAST_LOW:
		{
			SENSORDB("MS2R_set_isp_contrast ------------- [LOW] \r\n");
			//MS2R_write_category_parameter_ram(0x02, 0x25, 0x02);
			MS2R_write_category_parameter_ram(0x02, 0x1F, 0x01);
			break;
		}
		case ISP_CONTRAST_MIDDLE:
		{
			SENSORDB("MS2R_set_isp_contrast ------------- [MIDDLE] \r\n");
			//MS2R_write_category_parameter_ram(0x02, 0x25, 0x01);
			MS2R_write_category_parameter_ram(0x02, 0x1F, 0x00);
			break;
		}
		case ISP_CONTRAST_HIGH:
		{
			SENSORDB("MS2R_set_isp_contrast ------------- [HIGH] \r\n");
			//MS2R_write_category_parameter_ram(0x02, 0x25, 0x03);
			MS2R_write_category_parameter_ram(0x02, 0x1F, 0x03);
			break;
		}

		default:
			MS2R_write_category_parameter_ram(0x02, 0x25, 0x00);
			break;
	}
	#if defined(USE_WQ_accelerate)||defined(USE_KTHREAD)
	ms2r_ctl.Contrast = uPara;
	#endif

	MS2RFeatureInfo.m_uContrast = uPara;
}

static void MS2R_set_isp_sharpness(UINT16 uPara)
{
	return;
	if (g_eSysMode != SYS_MODE_MONITOR)
	{
		#if defined(USE_WQ_accelerate)||defined(USE_KTHREAD)
		ms2r_ctl.Edge = uPara;
		#endif
		return;
	}
	switch (uPara)
	{
		case ISP_EDGE_LOW:
		{
			SENSORDB("MS2R_set_isp_sharpness ------------- [LOW] \r\n");
			MS2R_write_category_parameter_ram(0x02, 0x18, 0x01);
			MS2R_write_category_parameter_ram(0x02, 0x16, 0x01);
			break;
		}
		case ISP_EDGE_MIDDLE:
		{
			SENSORDB("MS2R_set_isp_sharpness ------------- [MIDDLE] \r\n");
			MS2R_write_category_parameter_ram(0x02, 0x18, 0x02);
			MS2R_write_category_parameter_ram(0x02, 0x16, 0x01);
			break;
		}
		case ISP_EDGE_HIGH:
		{
			SENSORDB("MS2R_set_isp_sharpness ------------- [HIGH] \r\n");
			MS2R_write_category_parameter_ram(0x02, 0x18, 0x03);
			MS2R_write_category_parameter_ram(0x02, 0x16, 0x01);
			break;
		}

		default:
			break;
	}
	#if defined(USE_WQ_accelerate)||defined(USE_KTHREAD)
	ms2r_ctl.Edge = uPara;
	#endif

	MS2RFeatureInfo.m_uEdge = uPara;
}

static void MS2R_set_isp_brightness(UINT16 uPara)
{
	return;
	if (g_eSysMode != SYS_MODE_MONITOR)
	{
		#if defined(USE_WQ_accelerate)||defined(USE_KTHREAD)
		ms2r_ctl.Brightness = uPara;
		#endif
		return;
	}
	switch (uPara)
	{
		case ISP_BRIGHT_LOW:
		{
			SENSORDB("MS2R_set_isp_brightness ------------- [LOW] \r\n");
			//MS2R_write_category_parameter_ram(0x03, 0x09, 0x00);
			break;
		}
		case ISP_BRIGHT_MIDDLE:
		{
			SENSORDB("MS2R_set_isp_brightness ------------- [MIDDLE] \r\n");
			//MS2R_write_category_parameter_ram(0x03, 0x09, 0x1E);
			break;
		}
		case ISP_BRIGHT_HIGH:
		{
			SENSORDB("MS2R_set_isp_brightness ------------- [HIGH] \r\n");
			//MS2R_write_category_parameter_ram(0x03, 0x09, 0x3C);
			break;
		}

		default:
			MS2R_write_category_parameter_ram(0x03, 0x09, 0x1E);
			break;
	}
	#if defined(USE_WQ_accelerate)||defined(USE_KTHREAD)
	ms2r_ctl.Brightness = uPara;
	#endif

	MS2RFeatureInfo.m_uBrightness = uPara;
}

static void MS2R_set_isp_saturation(UINT16 uPara)
{
	#ifndef OPPO_CMCC_TEST
	return;
	#endif
	if (g_eSysMode != SYS_MODE_MONITOR)
	{
		#if defined(USE_WQ_accelerate)||defined(USE_KTHREAD)
		ms2r_ctl.Sat = uPara;
		#endif
		return;
	}
	switch (uPara)
	{
		case ISP_SAT_LOW:
		{
			SENSORDB("MS2R_set_isp_saturation ------------- [LOW] \r\n");
			MS2R_write_category_parameter_ram(0x02, 0x0F, 0x01);
			MS2R_write_category_parameter_ram(0x02, 0x10, 0x01);
			break;
		}
		case ISP_SAT_MIDDLE:
		{
			SENSORDB("MS2R_set_isp_saturation ------------- [MIDDLE] \r\n");
			MS2R_write_category_parameter_ram(0x02, 0x0F, 0x02);
			MS2R_write_category_parameter_ram(0x02, 0x10, 0x01);
			break;
		}
		case ISP_SAT_HIGH:
		{
			SENSORDB("MS2R_set_isp_saturation ------------- [HIGH] \r\n");
			MS2R_write_category_parameter_ram(0x02, 0x0F, 0x03);
			MS2R_write_category_parameter_ram(0x02, 0x10, 0x01);
			break;
		}

		default:
			break;
	}
	#if defined(USE_WQ_accelerate)||defined(USE_KTHREAD)
	ms2r_ctl.Sat = uPara;
	#endif

	MS2RFeatureInfo.m_uSat = uPara;
}

static void MS2R_set_scene_mode(UINT16 uPara)
{
	if (g_eSysMode != SYS_MODE_MONITOR)
	{
		#if defined(USE_WQ_accelerate)||defined(USE_KTHREAD)
		ms2r_ctl.Scene_mode = uPara;
		#endif
		return;
	}

	while (1 == MS2R_read_category_parameter_ram(0x0A, 0x02))
	{
		MS2R_write_category_parameter_ram(0x0A, 0x02, 0x00);
		Sleep(40);
	}

	switch (uPara)
	{
		case SCENE_MODE_OFF:
		case SCENE_MODE_NORMAL:
		{
			SENSORDB("MS2R_set_scene_mode ------------- [AUTO / NORMAL] \r\n");
			MS2R_write_category_parameter_ram(0x02, 0x59, 0x01);
			MS2R_update_feature_param();
			break;
		}
		case SCENE_MODE_LANDSCAPE:
		{
			SENSORDB("MS2R_set_scene_mode ------------- [LANDSCAPE] \r\n");
			MS2R_write_category_parameter_ram(0x02, 0x59, 0x02);
			break;
		}
		case SCENE_MODE_SPORTS:
		{
			SENSORDB("MS2R_set_scene_mode ------------- [SPORTS] \r\n");
			MS2R_write_category_parameter_ram(0x02, 0x59, 0x03);
			break;
		}
		case SCENE_MODE_PORTRAIT:
		{
			SENSORDB("MS2R_set_scene_mode ------------- [PORTRAIT] \r\n");
			MS2R_write_category_parameter_ram(0x02, 0x59, 0x04);
			break;
		}
		case SCENE_MODE_NIGHTSCENE:
		{
			SENSORDB("MS2R_set_scene_mode ------------- [NIGHTSCENE] \r\n");
			MS2R_write_category_parameter_ram(0x02, 0x59, 0x05);
			break;
		}
		case SCENE_MODE_SUNSET:
		{
			SENSORDB("MS2R_set_scene_mode ------------- [SUNSET] \r\n");
			MS2R_write_category_parameter_ram(0x02, 0x59, 0x06);
			break;
		}
		#ifdef VENDOR_EDIT
		//ZhangZhengrong@MtkCamera, 2013/03/19, Add for scene detect
		case SCENE_MODE_INDOOR:
		{
			SENSORDB("MS2R_set_scene_mode ------------- [INDOOR] \r\n");
			MS2R_write_category_parameter_ram(0x02, 0x59, 0x08);
			break;
		}
		case SCENE_MODE_MIX_ILLUMINANT:
		{
			SENSORDB("MS2R_set_scene_mode ------------- [MIX_ILLUMINANT] \r\n");
			MS2R_write_category_parameter_ram(0x02, 0x59, 0x07);
			break;
		}
		case SCENE_MODE_MACRO:
		{
			SENSORDB("MS2R_set_scene_mode ------------- [MACRO] \r\n");
			MS2R_write_category_parameter_ram(0x02, 0x59, 0x09);
			break;
		}
		#endif /* VENDOR_EDIT */
		case SCENE_MODE_HDR:
		{
			MS2R_HDR_captrue(uPara);
			break;
		}
		default:
		{
			MS2R_write_category_parameter_ram(0x02, 0x59, 0x01);
			break;
		}	
	}

	#if defined(USE_WQ_accelerate)||defined(USE_KTHREAD)
	ms2r_ctl.Scene_mode = uPara;
	#endif

	MS2RFeatureInfo.m_uSceneMode = uPara;
	
}

static void MS2R_set_AE_ISO(UINT16 uPara)
{
	#ifndef OPPO_CMCC_TEST
	return;
	#endif
	if (g_eSysMode != SYS_MODE_MONITOR)
	{
		#if defined(USE_WQ_accelerate)||defined(USE_KTHREAD)
		ms2r_ctl.AE_ISO = uPara;
		#endif
		return;
	}

	switch (uPara)
	{
		case AE_ISO_AUTO:
		{
			SENSORDB("MS2R_set_AE_ISO ------------- [AUTO] \r\n");
			MS2R_write_category_parameter_ram(0x03, 0x05, 0x00);
			break;
		}
		case AE_ISO_100:
		{
			SENSORDB("MS2R_set_AE_ISO ------------- [ISO_100] \r\n");
			#if 0 //modify for cmcc test
			MS2R_write_category_parameter_ram(0x03, 0x05, 0x02);
			#else
			MS2R_write_category_parameter_ram(0x03, 0x05, 0x00);
			#endif
			break;
		}
		case AE_ISO_200:
		{
			SENSORDB("MS2R_set_AE_ISO ------------- [ISO_200] \r\n");
			MS2R_write_category_parameter_ram(0x03, 0x05, 0x03);
			break;
		}
		case AE_ISO_400:
		{
			SENSORDB("MS2R_set_AE_ISO ------------- [ISO_400] \r\n");
			MS2R_write_category_parameter_ram(0x03, 0x05, 0x04);
			break;
		}
		case AE_ISO_800:
		{
			SENSORDB("MS2R_set_AE_ISO ------------- [ISO_800] \r\n");
			MS2R_write_category_parameter_ram(0x03, 0x05, 0x05);
			break;
		}
		case AE_ISO_1600:
		{
			SENSORDB("MS2R_set_AE_ISO ------------- [ISO_1600] \r\n");
			MS2R_write_category_parameter_ram(0x03, 0x05, 0x06);
			break;
		}
		default:
		{
			MS2R_write_category_parameter_ram(0x03, 0x05, 0x00);
			break;
		}
	}

	#if defined(USE_WQ_accelerate)||defined(USE_KTHREAD)
	ms2r_ctl.AE_ISO = uPara;
	#endif

	MS2RFeatureInfo.m_uAEISO = uPara;
	
}

static void MS2R_update_feature_param(void)
{
	MS2R_set_param_wb(MS2RFeatureInfo.m_uWb);
	MS2R_set_AE_ISO(MS2RFeatureInfo.m_uAEISO);
	MS2R_set_isp_saturation(MS2RFeatureInfo.m_uSat);
	MS2R_set_isp_sharpness(MS2RFeatureInfo.m_uEdge);
	MS2R_set_param_ev(MS2RFeatureInfo.m_uExposure);
	MS2R_set_isp_contrast(MS2RFeatureInfo.m_uContrast);
}

#if defined(USE_WQ_accelerate)||defined(USE_KTHREAD)
static void MS2R_update_parm(void)
{
	if(ms2r_ctl.update == KAL_TRUE)
	{
		Sleep(10);
		if (ms2r_ctl.Scene_mode != SCENE_MODE_HDR)
		{
			MS2R_set_scene_mode(ms2r_ctl.Scene_mode);
		}
	    MS2R_set_AF_mode(ms2r_ctl.AF_mode);
		MS2R_set_AE_ISO(ms2r_ctl.AE_ISO);
		//MS2R_set_ae_mode(ms2r_ctl.AE_mode);
		MS2R_set_param_ev(ms2r_ctl.Exposure);
		MS2R_set_param_wb(ms2r_ctl.Wb);
		MS2R_set_isp_saturation(ms2r_ctl.Sat);
		MS2R_set_isp_contrast(ms2r_ctl.Contrast);
		MS2R_set_isp_sharpness(ms2r_ctl.Edge);
		MS2R_set_flashlight_mode(ms2r_ctl.Stobe);
	}
	ms2r_ctl.update = KAL_FALSE;
}
#endif
static void MS2R_feature_setting(FEATURE_ID eCmd, UINT16 uPara)
{
	switch (eCmd)
	{
		case FID_AE_ISO:
		{
			MS2R_set_AE_ISO(uPara);
			break;
		}
		case FID_SCENE_MODE:
		{
			MS2R_set_scene_mode(uPara);
			break;
		}
		case FID_AF_MODE:
		{
			MS2R_set_AF_mode(uPara);
			break;
		}

		case FID_AE_STROBE:
		{
			MS2R_set_flashlight_mode(uPara);
			break;
		}

		case FID_AE_EV:
		{
			MS2R_set_param_ev(uPara);
			break;
		}

		case FID_AWB_MODE:
		{
			MS2R_set_param_wb(uPara);
			break;
		}

		case FID_COLOR_EFFECT:
		{
			MS2R_set_param_effect(uPara);
			break;
		}

		case FID_AE_SCENE_MODE:
		{
			MS2R_set_ae_mode(uPara);
			break;
		}
		case FID_ISP_CONTRAST:
		{
			MS2R_set_isp_contrast(uPara);
			break;
		}
		case FID_ISP_EDGE:
		{
			MS2R_set_isp_sharpness(uPara);
			break;
		}
		case FID_ISP_BRIGHT:
		{
			MS2R_set_isp_brightness(uPara);
			break;
		}
		case FID_ISP_SAT:
		{
			MS2R_set_isp_saturation(uPara);
			break;
		}
		
		default:
			break;
			
	}
 
}

static void MS2R_set_auto_flicker(UINT32 uPara)
{
	switch (uPara)
	{
		case AE_FLICKER_MODE_50HZ:
		{
			SENSORDB("MS2R_set_auto_flicker ------------- [50HZ] \r\n");
			MS2R_write_category_parameter_ram(0x03, 0x07, 0x01);
			break;
		}
		case AE_FLICKER_MODE_60HZ:
		{
			SENSORDB("MS2R_set_auto_flicker ------------- [60HZ] \r\n");
			MS2R_write_category_parameter_ram(0x03, 0x07, 0x02);
			break;
		}
		
		default:
			break;
	}
}

static UINT32 MS2R_map_ae_iso_speed(u32 uSensorISO)
{
	UINT32 uAEISOSpeed = 0;
	
	if (uSensorISO < 100)
	{
		uAEISOSpeed = AE_ISO_100;
	}
	else if (uSensorISO < 200)
	{
		uAEISOSpeed = AE_ISO_200;
	}
	else if (uSensorISO < 400)
	{
		uAEISOSpeed = AE_ISO_400;
	}
	else if (uSensorISO < 800)
	{
		uAEISOSpeed = AE_ISO_800;
	}
	else
	{
		uAEISOSpeed = AE_ISO_1600;
	}

	return uAEISOSpeed;
	
}

void MS2R_update_exif_info(void)
{	
	u32 u32AEISOSpeed = 0;
	u32 uNumerator = 0;
	u32 uDenominator = 0;
	u32 uFlashInfo = 0;
	u32 uFlashlightTimeus = 0;
	
	//AE ISO Speed
	u32AEISOSpeed = MS2R_read_category_parameter_ram(0x07, 0x28) << 8
					| MS2R_read_category_parameter_ram(0x07, 0x29);

	SENSORDB("u32AEISOSpeed = %d \r\n", u32AEISOSpeed);

	//exposure time
	uNumerator = MS2R_read_category_parameter_4Byte(0x07, 0x00);
	uDenominator = MS2R_read_category_parameter_4Byte(0x07, 0x04);

	SENSORDB("uNumerator------[%d], uDenominator-----[%d]---\r\n", uNumerator, uDenominator);

	//flashlight
	uFlashInfo = (u32)MS2R_read_category_parameter_ram(0x07, 0x2A) << 8
				| (u32)MS2R_read_category_parameter_ram(0x07, 0x2B);
	//flash is fire
	SENSORDB("---uFlashInfo----[0x%x]----\r\n", uFlashInfo);
	if ((uFlashInfo == 0x0009) || (uFlashInfo == 0x0019) || 
		(MS2RFeatureInfo.m_uStobe == FLASHLIGHT_TORCH))
	{
		SENSORDB("flash is fire \n");
		uFlashlightTimeus = 500000;//us
	}
	else
	{
		uFlashlightTimeus = 0;
	}
	
	spin_lock(&ms2r_drv_lock);
	//The unit of CapExposureTime is us
	if (uDenominator != 0)
	{
		g_stExifInfo.CapExposureTime = (uNumerator * 1000000) / uDenominator;
	}
	g_stExifInfo.AEISOSpeed = MS2R_map_ae_iso_speed(u32AEISOSpeed);
	g_stExifInfo.FlashLightTimeus = uFlashlightTimeus;
	spin_unlock(&ms2r_drv_lock);
	
}

static void MS2R_get_exif_info(UINT32 exifAddr)
{
	SENSOR_EXIF_INFO_STRUCT *pstExifInfo = (SENSOR_EXIF_INFO_STRUCT*)exifAddr;
	
	pstExifInfo->FNumber = 20;
	pstExifInfo->AWBMode = MS2RFeatureInfo.m_uWb;
	pstExifInfo->CapExposureTime = g_stExifInfo.CapExposureTime;
	pstExifInfo->FlashLightTimeus = g_stExifInfo.FlashLightTimeus;
	pstExifInfo->AEISOSpeed = g_stExifInfo.AEISOSpeed;
	pstExifInfo->RealISOValue = AE_ISO_100;
}

static void MS2R_set_AF_window(UINT32 uZoneAddr)
{
	UINT32 *pZone = (UINT32 *)uZoneAddr;

	UINT32 uLeft = 0;
	UINT32 uTop = 0;
	UINT32 uRight = 0;
	UINT32 uBottom = 0;
	
	UINT32 uFDWidth = 0;
	UINT32 uFDHeight = 0;

	SENSORDB("MS2R_set_AF_window--------> IN \r\n");
	
	uLeft = *pZone;
	uTop = *(pZone+1);
	uRight = *(pZone+2);
	uBottom = *(pZone+3);
	
	uFDWidth = *(pZone+4);
	uFDHeight = *(pZone+5);

	SENSORDB("MS2R_set_AF_window--------uLeft[%d] uTop[%d] uRight[%d] uBottom[%d] \r\n", uLeft, uTop, uRight, uBottom);
	SENSORDB("MS2R_set_AF_window--------uFDWidth[%d] uFDHeight[%d] \r\n", uFDWidth, uFDHeight);

	spin_lock(&ms2r_drv_lock);
	
	g_uZone[0] = uLeft;
	g_uZone[1] = 55 + uTop * 144/256;
	g_uZone[2] = 50;
	g_uZone[3] = 60;
	
	if (g_uZone[0] > 205)
	{
		g_uZone[0] = 205;
	}

	if (g_uZone[1] > 139)
	{
		g_uZone[1] = 139;
	}
	
	spin_unlock(&ms2r_drv_lock);
	
	SENSORDB("MS2R_set_AF_window--------X[%d] Y[%d] Width[%d] Height[%d] \r\n", g_uZone[0], g_uZone[1], g_uZone[2], g_uZone[3]);	

	SENSORDB("MS2R_set_AF_window--------> OUT \r\n");
	
}

static void MS2R_start_touch_AE_AF(UINT8 *puZone)
{
	u8 uAFAEEnable = 0;
	
	SENSORDB("MS2R_start_touch_AE_AF--------> IN \r\n");
	
	//set touch AE strength
	MS2R_write_category_parameter_ram(0x02, 0x49, 0x0F);
	//change to touch AF mode
	MS2R_write_category_parameter_ram(0x0A, 0x21, 0x04);

	//stop Touch AE/AF
	MS2R_write_category_parameter_ram(0x02, 0x48, 0x00);

	//set location of Touch Area
	MS2R_write_category_parameter_ram(0x02, 0x40, 0x00);
	MS2R_write_category_parameter_ram(0x02, 0x41, *puZone);
	MS2R_write_category_parameter_ram(0x02, 0x42, 0x00);
	MS2R_write_category_parameter_ram(0x02, 0x43, *(puZone+1));

	//set frame size of Touch Area
	MS2R_write_category_parameter_ram(0x02, 0x44, 0x00);
	MS2R_write_category_parameter_ram(0x02, 0x45, *(puZone+2));
	MS2R_write_category_parameter_ram(0x02, 0x46, 0x00);
	MS2R_write_category_parameter_ram(0x02, 0x47, *(puZone+3));

	//start Touch AE/AF
	MS2R_write_category_parameter_ram(0x02, 0x48, 0x01);

	uAFAEEnable = MS2R_read_category_parameter_ram(0x02, 0x48);
	if (0x00 == uAFAEEnable)
	{
		SENSORDB("Touch AF/AE is NOT starting \r\n");
	}
	else if (0x01 == uAFAEEnable)
	{
		SENSORDB("Touch AF/AE is starting \r\n");
		g_eAEMode = MS2R_AE_MODE_TOUCH;
	}

	SENSORDB("MS2R_start_touch_AE_AF--------> OUT \r\n");
}

static void MS2R_AF_single_focus(void)
{
	printk("MS2R_AF_single_focus--------> IN \r\n");
	
	MS2R_start_touch_AE_AF(g_uZone);

	MS2R_write_category_parameter_ram(0x0A, 0x49, 0x02);
	MS2R_write_category_parameter_ram(0x0A, 0x02, 0x01);
	#ifdef VENDOR_EDIT
	//hufeng@mtk camera team modified 20121227 for continuous af
	m3ASingleAFLock = true;
	m3ASingleFrameCnt = 0;
	#endif
	printk("MS2R_AF_single_focus--------> OUT \r\n");
	
}


static void MS2R_start_face_AE(UINT8 *puZone)
{
	u8 uFaceAEEnable = 0;
	
	SENSORDB("MS2R_start_face_AE--------> IN \r\n");
	
	//set touch AE strength
    #ifdef VENDOR_EDIT	//lxl@camera team modify
		MS2R_write_category_parameter_ram(0x02, 0x49, 0x08);
    #else		
	    MS2R_write_category_parameter_ram(0x02, 0x49, 0x0F);
    #endif		
	//change to touch AF mode
	MS2R_write_category_parameter_ram(0x0A, 0x21, 0x04);

	//stop Touch AE/AF
	MS2R_write_category_parameter_ram(0x02, 0x48, 0x00);

	//set location of Touch Area
	MS2R_write_category_parameter_ram(0x02, 0x40, 0x00);
	MS2R_write_category_parameter_ram(0x02, 0x41, *puZone);
	MS2R_write_category_parameter_ram(0x02, 0x42, 0x00);
	MS2R_write_category_parameter_ram(0x02, 0x43, *(puZone+1));

	//set frame size of Touch Area
	MS2R_write_category_parameter_ram(0x02, 0x44, 0x00);
	MS2R_write_category_parameter_ram(0x02, 0x45, *(puZone+2));
	MS2R_write_category_parameter_ram(0x02, 0x46, 0x00);
	MS2R_write_category_parameter_ram(0x02, 0x47, *(puZone+3));

	//start Touch AE/AF
	MS2R_write_category_parameter_ram(0x02, 0x48, 0x02);

	uFaceAEEnable = MS2R_read_category_parameter_ram(0x02, 0x48);
	if (0x00 == uFaceAEEnable)
	{
		SENSORDB("Face AE is NOT starting \r\n");
	}
	else if (0x02 == uFaceAEEnable)
	{
		SENSORDB("Face AE is starting \r\n");
		g_eAEMode = MS2R_AE_MODE_FACE;
	}

	Sleep(100);
	//MS2R_write_category_parameter_ram(0x0A, 0x49, 0x00);
	MS2R_write_category_parameter_ram(0x0A, 0x02, 0x01);

	SENSORDB("MS2R_start_face_AE--------> OUT \r\n");
}

static void MS2R_set_AE_window(UINT32 uZoneAddr)
{
	UINT32 *pZone = (UINT32 *)uZoneAddr;

	UINT32 uLeft = 0;
	UINT32 uTop = 0;
	UINT32 uRight = 0;
	UINT32 uBottom = 0;
	
	UINT32 uFDWidth = 0;
	UINT32 uFDHeight = 0;

	UINT8 uAEZone[4] = {0};

	SENSORDB("MS2R_set_AE_window--------> IN \r\n");
	
	uLeft = *pZone;
	uTop = *(pZone+1);
	uRight = *(pZone+2);
	uBottom = *(pZone+3);
	
	uFDWidth = *(pZone+4);
	uFDHeight = *(pZone+5);

	SENSORDB("MS2R_set_AE_window--------uLeft[%d] uTop[%d] uRight[%d] uBottom[%d] \r\n", uLeft, uTop, uRight, uBottom);
	SENSORDB("MS2R_set_AE_window--------uFDWidth[%d] uFDHeight[%d] \r\n", uFDWidth, uFDHeight);
	
	uAEZone[0] = uLeft;
	uAEZone[1] = 55 + uTop * 144/256;
	uAEZone[2] = uRight - uLeft;
	uAEZone[3] = (uBottom - uTop)*144/256;
	
	if (uAEZone[0] > 255 - uAEZone[2])
	{
		uAEZone[0] = 255 - uAEZone[2];
	}

	if (uAEZone[1] > 199 - uAEZone[3])
	{
		uAEZone[1] = 199 - uAEZone[3];
	}

	SENSORDB("MS2R_set_AE_window--------X[%d] Y[%d] Width[%d] Height[%d] \r\n", uAEZone[0], uAEZone[1], uAEZone[2], uAEZone[3]);

	spin_lock(&FaceAE_lock);
	g_uFaceAECnt++;
	spin_unlock(&FaceAE_lock);
	
	if (0x00 == MS2R_read_category_parameter_ram(0x02, 0x48))
	{
		MS2R_start_face_AE(uAEZone);
	}

	SENSORDB("MS2R_set_AE_window--------> OUT \r\n");
	
}

static bool MS2R_check_if_has_faces(void)
{
	bool bHasFaces = false;
	static int nCheckTimes = 0;

	spin_lock(&FaceAE_lock);
	
	if (g_uFaceAECnt != g_uFaceAECntPre)
	{
		bHasFaces = true;
		nCheckTimes = 0;
		g_uFaceAECntPre = g_uFaceAECnt;
	}
	else
	{
		if (g_uFaceAECnt == 0
			|| (g_uFaceAECnt != 0 && ++nCheckTimes > 10))
		{
			nCheckTimes = 0;
			bHasFaces = false;
			g_uFaceAECnt = 0;
			g_uFaceAECntPre = 0;
			if (MS2R_AE_MODE_FACE == g_eAEMode)
			{
				//stop Touch AE/AF
				MS2R_write_category_parameter_ram(0x02, 0x48, 0x00);
				//set AE MODE auto
				MS2R_write_category_parameter_ram(0x03, 0x01, 0x02);
				g_eAEMode = MS2R_AE_MODE_AUTO;

				//set CAF to init condition after face ae
				mmIsCAFin = false;
				mdelayDetectCnt = ANTI_WDV_SHAKE_DELAY;
				mdelayAFCnt = DO_AF_DELAY;
				m3ASingleAFLock = false;
				m3ASingleFrameCnt = 0;
				nDelayFrameCnt = 5;
			}
		}
	}

	spin_unlock(&FaceAE_lock);

	return bHasFaces;
	
}

static void MS2R_get_AF_status(UINT32 *pAFResult)
{
	u8 u8AFResult = 0;

	u8AFResult = MS2R_read_category_parameter_ram(0x0A, 0x03);

	switch (u8AFResult)
	{
		case 0x00:
		{
			*pAFResult = SENSOR_AF_FOCUSING;
			break;
		}
		case 0x01:
		{
			*pAFResult = SENSOR_AF_FOCUSED;
			break;
		}
		case 0x02:
		{
			*pAFResult = SENSOR_AF_ERROR;
			break;
		}
		case 0x03:
		{
			*pAFResult = SENSOR_AF_SCENE_DETECTING;
			break;
		}

		default:
		{
			*pAFResult = SENSOR_AF_IDLE;
			break;
		}	
	}

	//SENSORDB("MS2R_get_AF_status------[%d]-- \r\n", *pAFResult);
	
}

static void MS2R_AF_constant_focus(void)
{
	SENSORDB("MS2R_AF_constant_focus--------> IN \r\n");
	
	//stop Touch AE/AF
	MS2R_write_category_parameter_ram(0x02, 0x48, 0x00);
	
	/*set window size for normal AF*/
	MS2R_write_category_parameter_ram(0x0A, 0x21, 0x00);

	//set AE MODE
	MS2R_write_category_parameter_ram(0x03, 0x01, 0x02);
	
	/*start Auto Focus*/
	MS2R_write_category_parameter_ram(0x0A, 0x49, 0x00);
	MS2R_write_category_parameter_ram(0x0A, 0x02, 0x01);
	
	g_eAEMode = MS2R_AE_MODE_AUTO;

	SENSORDB("MS2R_AF_constant_focus--------> OUT \r\n");
	
}

static void MS2R_get_brightness(UINT16 *puBrightness)
{
	u8 uHigh = 0;
	u8 uLow = 0;
	
	//SENSORDB("MS2R_get_brightness--------> IN \r\n");

	uHigh = MS2R_read_category_parameter_ram(0x03, 0x32);
	uLow = MS2R_read_category_parameter_ram(0x03, 0x33);

	SENSORDB("MS2R_get_brightness----high [0x%x] ---- low [0x%x]---\r\n", uHigh, uLow);
	//if (uHigh == 0xFF)
	if(uHigh >= 0xFC) //ZhangZhengrong 2013/03/12 modify
	{
		*puBrightness = 0;
	}
	else
	{
		*puBrightness = ((UINT16)uHigh << 8)
				| (UINT16)uLow;
	}
	
	SENSORDB("MS2R_get_brightness-----[%d]---> OUT \r\n", *puBrightness);
}
#ifdef VENDOR_EDIT
//ZhangZhengrong@MtkCamera, 2013/03/19, Add for scene detect
static void MS2R_get_AE_gain_shutter(UINT32 *pAEparameter)
{
	u32 u32AEISOSpeed = 0;
	SENSORDB("MS2R_get_AE_gain_shutter--------> IN \r\n");

	*pAEparameter = (((UINT32)MS2R_read_category_parameter_ram(0x03, 0x0E) << 24) |
                        ((UINT32)MS2R_read_category_parameter_ram(0x03, 0x0F) << 16) |
                        ((UINT32)MS2R_read_category_parameter_ram(0x03, 0x10) << 8) |
                        ((UINT32)MS2R_read_category_parameter_ram(0x03, 0x11)));
	
	SENSORDB("MS2R_get_AE_gain_shutter end: pAEparameter=%d\n", *pAEparameter);
}

static void MS2R_get_AF_position(UINT16 *pAFposition)
{	
	//SENSORDB("MS2R_get_AF_position--------> IN \r\n");
	*pAFposition = ((UINT16)MS2R_read_category_parameter_ram(0x0A, 0x10) << 8) |
				(UINT16)MS2R_read_category_parameter_ram(0x0A, 0x11);

	if(pre_AFposition != *pAFposition && (*pAFposition != 0))
		pre_AFposition = *pAFposition;
	else if(*pAFposition == 0)
		*pAFposition = pre_AFposition;
	
	SENSORDB("MS2R_get_AF_position-----[%d]---> OUT \r\n", *pAFposition);
}
#endif /* VENDOR_EDIT */

static void MS2R_get_whitebalance(UINT16 *puWhiteBalance)
{
	//SENSORDB("MS2R_get_whitebalance--------> IN \r\n");

	//start update wb
	MS2R_write_category_parameter_ram(0x06,0x4A,0x01);

	*puWhiteBalance = ((UINT16)MS2R_read_category_parameter_ram(0x06, 0x06) << 8)
				| (UINT16)MS2R_read_category_parameter_ram(0x06, 0x07);
	
	SENSORDB("MS2R_get_whitebalance-----[%d]---> OUT \r\n", *puWhiteBalance);
}
#ifdef VENDOR_EDIT
//feng.hu@mtk camera team modified for ms2r continuous af
static void MS2R_get_wavedetectvalue(UINT32 *wdv)    
{
	//SENSORDB("MS2R_get_wavedetectvalue--------> IN \r\n");

	*wdv = (((UINT32)MS2R_read_category_parameter_ram(0x0A, 0x0C) << 24) |
                        ((UINT32)MS2R_read_category_parameter_ram(0x0A, 0x0D) << 16) |
                        ((UINT32)MS2R_read_category_parameter_ram(0x0A, 0x0E) << 8) |
                        ((UINT32)MS2R_read_category_parameter_ram(0x0A, 0x0F)));
	
	SENSORDB("MS2R_get_wavedetectvalue-----[%ul]---> OUT \r\n", *wdv);
}
static MINT8 getOrder(MUINT32 num)
{
    MUINT8 order = 0;
    MUINT32 cpyWDV = 0;
 
    cpyWDV = num;
    while(cpyWDV != 0){
       cpyWDV = cpyWDV/10;
       order++;
    }
    return order; 
}

static bool doCAFDetect(MUINT32 mode)
{ 
    bool result;

    MUINT32 curFv = 0;
    MINT32 delta  = 0;  
    MINT32 doCAFthr = 0;
    MINT32 doCAFthrLow = 0;
    MINT32 uCurBrightness = 0;
    MINT32 uCurWB = 0;
    MINT32 uWhiteBalance = 0;
    MINT32 uBrightness = 0;
	MINT32 uGyroData[3] = {0};
	MINT32 uGyroX = 0;
	MINT32 uGyroY = 0;
	MINT32 uGyroZ = 0;
	
    MS2R_get_wavedetectvalue(&curFv);
	MS2R_get_brightness(&uCurBrightness);
	MS2R_get_whitebalance(&uCurWB);
	
	//detect gyro data 
	if (g_bGyroOpened)
	{
		L3G4200D_ReadGyroData_preview(uGyroData);
		uGyroX = abs(uGyroData[0] - g_uGyroData[0]);
		uGyroY = abs(uGyroData[1] - g_uGyroData[1]);
		uGyroZ = abs(uGyroData[2] - g_uGyroData[2]);
		SENSORDB("uGyroData X[%d] , Y[%d], Z[%d] \r\n", uGyroX, uGyroY, uGyroZ);
	}

	printk("[doCAFDetect] hufeng debug mPreFv: %u  order: %d\n", mPreFv, getOrder(mPreFv));
	
    if(getOrder(curFv) < 5 || getOrder(curFv) > 6)
	{
		SENSORDB("WaveDetect Value is not valid, not use it \r\n");
		return false;
    }

    delta = abs(mPreFv - curFv);
	uWhiteBalance = abs(uCurWB - uPreWB);
	uBrightness = abs(uCurBrightness - uPreBright);

    switch(mode)
	{
		case THR_CASE:
		{
			SENSORDB("THR_CASE-----start \r\n");
			if (getOrder(mPreFv) == 5)
			{
				doCAFthr = mPreFv/30;
			}
			else
			{
				doCAFthr = mPreFv/100;
			}

			SENSORDB("delta = %d , doCAFthr = %d \r\n", delta, doCAFthr);
			
			if(delta > doCAFthr)
			{
				SENSORDB("[doCAFDetect] hsd debug THR_CASE  true $$$$$$$$$$$$$$$$$$$\n");
				result = true;
			}
			else 
			{
				result = false;
			}
			
			//avoid focus continous at some scenes
			if (result == true)
			{
				SENSORDB("check the brightness and wb------thread \r\n");
				//SENSORDB("uBrightness---[%d], uCurBrightness----[%d] \r\n", uBrightness, uCurBrightness);
				//SENSORDB("uWhiteBalance---[%d], uCurWB----[%d] \r\n", uWhiteBalance, uCurWB);
				if ((uBrightness < 20 && uCurBrightness > 0)
					&& (uWhiteBalance < 60 && uCurWB > 0))
				{
					SENSORDB("the brightness vare not strongly, do not focus \r\n");
					result = false;
				}

				if (g_bGyroOpened)
				{
					//avoid do focus when hand shake
					if ( uGyroX < 100 && uGyroY < 100 && uGyroZ < 100)
					{
						result = false;
					}
				}
				
			}

			if (result == false)
			{
				//at low contrast scene, detect by gyro
				if (uBrightness < 5 && uCurBrightness > 0
					&& uWhiteBalance < 5 && uCurWB > 0 
					&& g_bGyroOpened)
				{
					SENSORDB("Detect by gyro at low contrast scene \r\n");
					if (uGyroX > 100 || uGyroY > 100 || uGyroZ > 100)
					{
						SENSORDB("moving at low contrast scene, need do focus \r\n");
						result = true;
					}
				}
			}

			//at low light scene, do detect specially
			if(uCurBrightness == 0)
			{
				SENSORDB("Now at low light scene, need do detect specialy \r\n");
				//SENSORDB("[low light]----uWhiteBalance = %d , uPreWB = %d \r\n", uWhiteBalance , uPreWB);
				if (uWhiteBalance < 50)
				{
					result = false;
				}
			}
			break;
        }
        case STABLE_CASE:
		{
			SENSORDB("STABLE_CASE-----start \r\n");
			
			if (g_bGyroOpened)
			{
				//detect by gyro data
				if (uGyroX > 100 || uGyroY > 100 || uGyroZ > 100)
				{
					result = false;
				}
				else
				{
					result = true;
				}

				//avoid do focus when moving
				if (result == true)
				{
					SENSORDB("check the brightness and wb-----stable\r\n");
					//SENSORDB("[stable] uBrightness---[%d], uCurBrightness----[%d] \r\n", uBrightness, uCurBrightness);
					//SENSORDB("[stable] uWhiteBalance---[%d], uCurWB----[%d] \r\n", uWhiteBalance, uCurWB);
					if ((uBrightness > 20 && uCurBrightness > 0) ||
						(uWhiteBalance > 50 && uCurWB > 0))
					{
						SENSORDB("cancel focus \r\n");
						result = false;
					}
				}
			}
			else
			{
				//if gyro not work, detect by WaveDetect Value
				if (getOrder(mPreFv) == 5)
				{
					doCAFthrLow = mPreFv/10;
				}
				else
				{
					doCAFthrLow = mPreFv/50;
				}

				if(WDV_CAF_LOW_THR <= mPreFv/60) 
				{
					doCAFthrLow = mPreFv/50;
				}
				else
				{
					doCAFthrLow = WDV_CAF_LOW_THR;
				}
				
				SENSORDB("delta = %d , doCAFthrLow = %d, mPreFv = %d \r\n", delta, doCAFthrLow, mPreFv);
				if(delta > doCAFthrLow)
				{
					result = false;
				} 
				else 
				{
					SENSORDB("[doCAFDetect] hsd debug STABLE_CASE true ^^^^^^^^^^^^^^^\n");
					result = true;
				}

				//avoid focus when moving sometimes
				if (result == false)
				{
					SENSORDB("check the brightness and wb-----stable\r\n");
					//SENSORDB("[stable] uBrightness---[%d], uCurBrightness----[%d] \r\n", uBrightness, uCurBrightness);
					//SENSORDB("[stable] uWhiteBalance---[%d], uCurWB----[%d] \r\n", uWhiteBalance, uCurWB);
					
					if ((uBrightness < 10 && uCurBrightness > 0) && 
						(uWhiteBalance < 10 && uCurWB > 0))
					{
						SENSORDB("the brightness and WB is stable, do focus \r\n");
						result = true;
					}
				}
				else
				{
					if ((uBrightness > 30 && uCurBrightness > 0) &&
						(uWhiteBalance > 100 && uCurWB > 0))
					{
						SENSORDB("cancel focus \r\n");
						result = false;
					}
				}
				
			}
			break;
		}
        default:
            break;
    }
 
	mPreFv = curFv;
	uPreWB = uCurWB;
	uPreBright = uCurBrightness;

	if (g_bGyroOpened)
	{
		g_uGyroData[0] = uGyroData[0];
		g_uGyroData[1] = uGyroData[1];
		g_uGyroData[2] = uGyroData[2];
	}

    return result;
}

static bool doCAFDetectOnlyByWBAndBright(void)
{
	bool bTriggered = false;
	MUINT32 u4CurBrightness = 0;
	MUINT32 u4CurWB = 0;
	MUINT32 u4DiffBrightness = 0;
	MUINT32 u4DiffWB = 0;
	static MUINT32 u4PreBrightness = 0;
	static MUINT32 u4PreWB = 0;

	MS2R_get_brightness(&u4CurBrightness);
	MS2R_get_whitebalance(&u4CurWB);

	u4DiffBrightness = abs(u4CurBrightness - u4PreBrightness);
	u4DiffWB = abs(u4CurWB - u4PreWB);
	SENSORDB("u4DiffBrightness [%d], u4PreBrightness [%d] \r\n", u4DiffBrightness,u4PreBrightness);
	SENSORDB("u4DiffWB [%d], u4PreWB [%d] \r\n", u4DiffWB, u4PreWB);
	if ((u4DiffBrightness * 100 > u4PreBrightness * 10)
		|| ((u4DiffWB * 100 > u4PreWB * 3) && (u4DiffWB > 50))
		)
	{
		SENSORDB("doCAFDetectOnlyByWBAndBright------------trigger \r\n");
		bTriggered = true;
	}
	else
	{
		SENSORDB("doCAFDetectOnlyByWBAndBright------------not trigger \r\n");
		bTriggered = false;
	}

	u4PreBrightness = u4CurBrightness;
	u4PreWB = u4CurWB;

	return bTriggered;
	
}


static void MS2R_DO_CAF(bool *pbTrigger)
{
	if (g_bHasFaces && mdelayFrameCnt > 0)
	{
		SENSORDB("Now is not in monitor, do not detect CAF \r\n");
		mdelayFrameCnt--;
		return;
	}

	if (bCaptureStart)
	{
		SENSORDB("Now start take picture, not do CAF \r\n");
		return;
	}
	
	//delay some frames not to detect after capture
	if (bNeedDelay)
	{
		SENSORDB("After capture, need delay some frames \r\n");
		
		if (mdelayFrameCnt > 0)
		{
			mdelayFrameCnt--;

			//if not stable and WD value vare out of thread, do focus
			if (doCAFDetect(THR_CASE))
			{
				MS2R_AF_constant_focus();
				bNeedDelay = KAL_FALSE;
				bWaveDetectValid = KAL_TRUE;
				mdelayFrameCnt = 0;
			}
		}
		else
		{
			bNeedDelay = KAL_FALSE;
		}
		return;
	}

	//when start monitor, must do one time focus,the WD value can valid
	if (bWaveDetectValid == KAL_FALSE)
	{
		SENSORDB("Detect by WB and Bright \r\n");
		if (doCAFDetectOnlyByWBAndBright())
		{
			*pbTrigger = true;
			SENSORDB("WaveDetect------trigger \r\n");
			MS2R_AF_constant_focus();
			bWaveDetectValid = KAL_TRUE;
			nDelayFrameCnt = 20;
		}
		else
		{
			SENSORDB("WaveDetect------not trigger \r\n");
			*pbTrigger = false;
		}
		return;	
	}

	//after check bright and WB trigger focus, can delay some frames
	if (bWaveDetectValid == KAL_TRUE && nDelayFrameCnt > 0)
	{
		MS2R_get_wavedetectvalue(&mPreFv);
		MS2R_get_brightness(&uPreBright);
		MS2R_get_whitebalance(&uPreWB);
		nDelayFrameCnt--;
		return;
	}
	
	if(!mmIsCAFin)
	{
		mCAFflag = doCAFDetect(THR_CASE);
		if(mCAFflag)
		{
			mdelayAFCnt = DO_AF_DELAY;
			mmIsCAFin = true;
		}
	}
       
	if (m3ASingleAFLock)
	{
		mCAFflag = false;
		m3ASingleFrameCnt++;
		//SENSORDB("hufeng  m3ASingleFrameCnt: %d\n", m3ASingleFrameCnt);
		if (m3ASingleFrameCnt > 20)
		{
			m3ASingleAFLock = false;
			m3ASingleFrameCnt = 0;
			mmIsCAFin = false;
		}
	}
	//SENSORDB("hufeng  m3ASingleAFLock: %d, mCAFflag: %d\n", m3ASingleAFLock, mCAFflag);
	if(!m3ASingleAFLock)
	{
		if(mCAFflag)
		{
			SENSORDB("mdelayAFCnt = %d \r\n", mdelayAFCnt);
			if((mdelayAFCnt < 1) && doCAFDetect(STABLE_CASE)) 
			{
				MS2R_AF_constant_focus();
				*pbTrigger = true;
				mCAFflag = false;
				mdelayDetectCnt = ANTI_WDV_SHAKE_DELAY;
			}
			else
			{
				*pbTrigger = false;
			}
			
			mdelayAFCnt--;
			
		}
		
	}
	
	if(mdelayDetectCnt > 0)
	{
		mdelayDetectCnt--;
	} 
	else if(mdelayDetectCnt == 0)
	{
		MS2R_get_wavedetectvalue(&mPreFv);
		MS2R_get_brightness(&uPreBright);
		MS2R_get_whitebalance(&uPreWB);
		mmIsCAFin = false;
		mdelayDetectCnt--;
	}
	
}
#endif

UINT32 MS2RFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
                                                                UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
    UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
    UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
    UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
    UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
    UINT32 SensorRegNumber;
    UINT32 i;
    PNVRAM_SENSOR_DATA_STRUCT pSensorDefaultData=(PNVRAM_SENSOR_DATA_STRUCT) pFeaturePara;
    MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
    MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_GROUP_INFO_STRUCT *pSensorGroupInfo=(MSDK_SENSOR_GROUP_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_ITEM_INFO_STRUCT *pSensorItemInfo=(MSDK_SENSOR_ITEM_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_ENG_INFO_STRUCT	*pSensorEngInfo=(MSDK_SENSOR_ENG_INFO_STRUCT *) pFeaturePara;

    switch (FeatureId)
    {
        case SENSOR_FEATURE_GET_RESOLUTION:
			if (isp_capture_raw_data)
			{
				*pFeatureReturnPara16++=MS2R_CAP_RAW_DATA_WIDTH;
            	*pFeatureReturnPara16=MS2R_CAP_RAW_DATA_HEIGHT;
			}
			else
			{
            	*pFeatureReturnPara16++=MS2R_REAL_CAP_WIDTH;
            	*pFeatureReturnPara16=MS2R_REAL_CAP_HEIGHT;
			}
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_PERIOD:
          	break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			break;
        case SENSOR_FEATURE_SET_ESHUTTER:
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            break;
        case SENSOR_FEATURE_SET_GAIN:
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
		{
			MS2R_set_flashlight_mode(*pFeatureData32);
			break;
		}
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
            break;
        case SENSOR_FEATURE_SET_REGISTER:
            break;
        case SENSOR_FEATURE_GET_REGISTER:
        case SENSOR_FEATURE_SET_CCT_REGISTER:
        case SENSOR_FEATURE_GET_CCT_REGISTER:
        case SENSOR_FEATURE_SET_ENG_REGISTER:
        case SENSOR_FEATURE_GET_ENG_REGISTER:
        case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
            break;
        case SENSOR_FEATURE_GET_CONFIG_PARA:
            memcpy(pSensorConfigData, &MS2RSensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
            *pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
            break;
        case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
            break;

        case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
            break;
        case SENSOR_FEATURE_GET_GROUP_COUNT:
            break;
        case SENSOR_FEATURE_GET_GROUP_INFO:
            break;
        case SENSOR_FEATURE_GET_ITEM_INFO:
            break;

        case SENSOR_FEATURE_SET_ITEM_INFO:
            break;

        case SENSOR_FEATURE_GET_ENG_INFO:
            pSensorEngInfo->SensorId = MS2R_SENSOR_ID;
            pSensorEngInfo->SensorType = CMOS_SENSOR;
            pSensorEngInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_YUYV;
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ENG_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
            // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
            // if EEPROM does not exist in camera module.
            *pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
            *pFeatureParaLen=4;
            break;

    	case SENSOR_FEATURE_SET_YUV_CMD:
		{
			MS2R_feature_setting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));
			break;
		}
		//Touch AF
		case SENSOR_FEATURE_CANCEL_AF:
		{
			//MS2R_write_category_parameter_ram(0x0A, 0x02, 0x00);
			break;
		}
		case SENSOR_FEATURE_SET_AF_WINDOW:
		{
			if ((SYS_MODE_MONITOR == g_eSysMode) 
				|| (SYS_MODE_VIDEO_MONITOR == g_eSysMode))
			{
				MS2R_set_AF_window(*pFeatureData32);
			}
			break;
		}
		case SENSOR_FEATURE_SINGLE_FOCUS_MODE:
		{
			if ((SYS_MODE_MONITOR == g_eSysMode) 
				|| (SYS_MODE_VIDEO_MONITOR == g_eSysMode))
			{
				MS2R_AF_single_focus();
			}
			break;
		}

        case SENSOR_FEATURE_INITIALIZE_AF:
		{
			break;
		}
        case SENSOR_FEATURE_CONSTANT_AF:
        {
			if (SYS_MODE_MONITOR == g_eSysMode)
			{
				MS2R_AF_constant_focus();
			}
			
			break;
		}
		case SENSOR_FEATURE_GET_AF_MAX_NUM_FOCUS_AREAS:
		case SENSOR_FEATURE_GET_AE_MAX_NUM_METERING_AREAS:
		{
			*pFeatureReturnPara32 = 1;
			*pFeatureParaLen=4;
			break;
		}
		case SENSOR_FEATURE_SET_AE_WINDOW:
		{
			if (SYS_MODE_MONITOR == g_eSysMode)
			{
				MS2R_set_AE_window(*pFeatureData32);
			}
			break;
		}
		case SENSOR_FEATURE_GET_AF_STATUS:
		{
			MS2R_get_AF_status(pFeatureReturnPara32);
			*pFeatureParaLen=4;
			break;
		}
		case SENSOR_FEATURE_GET_ISP_STATUS:
		{
			*pFeatureReturnPara32 = bISPCapture;
			SENSORDB("SENSOR_FEATURE_GET_ISP_STATUS-------[0x%x] \r\n", *pFeatureReturnPara32);
			*pFeatureParaLen=4;
			break;
		}
		case SENSOR_FEATURE_GET_BRIGHTNESS:
		{
			MS2R_get_brightness(pFeatureReturnPara16);
			*pFeatureParaLen=2;
			break;
		}
		#ifdef VENDOR_EDIT
		//ZhangZhengrong@MtkCamera, 2013/03/19, Add for scene detect
		case SENSOR_FEATURE_GET_WDV:
		{
			MS2R_get_wavedetectvalue(pFeatureReturnPara32);
			*pFeatureParaLen = 4;
			break;
		}
		
		case SENSOR_FEATURE_GET_AE_GAIN_SHUTTER:
		{
			MS2R_get_AE_gain_shutter(pFeatureReturnPara32);
			*pFeatureParaLen = 4;
			break;
		}
		
		case SENSOR_FEATURE_GET_AF_POSITION:
		{
			MS2R_get_AF_position(pFeatureReturnPara16);
			*pFeatureParaLen = 2;
			break;
		}
		#endif /* VENDOR_EDIT */
		case SENSOR_FEATURE_GET_WB:
		{
			MS2R_get_whitebalance(pFeatureReturnPara16);
			*pFeatureParaLen=2;
			break;
		}
		
		case SENSOR_FEATURE_GET_EXIF_INFO:
		{
			SENSORDB("SENSOR_FEATURE_GET_EXIF_INFO----EXIF addr = 0x%x\n", *pFeatureData32);
			MS2R_get_exif_info(*pFeatureData32);
			break;
		}
		
        case SENSOR_FEATURE_MOVE_FOCUS_LENS:
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
		{
            if (TRUE == MS2RCheckIsAlive())
            {
				*pFeatureData32 = MS2R_SENSOR_ID;
				*pFeatureParaLen=4;
			}
			else
			{
				*pFeatureData32 = 0xFFFFFFFF;
				*pFeatureParaLen=4;
			}
            break;
        }
		case SENSOR_FEATURE_SET_ISP_DEBUG_FLAG:
		{
			ISP_DEBUG_INFO *pISPDebugInfo = (ISP_DEBUG_INFO *)pFeatureData32;
			isp_tuning_enable = (bool)pISPDebugInfo->ISPTuningEnable;
			isp_capture_raw_data = (bool)pISPDebugInfo->ISPRawDataEnable;
			isp_debug_enable = (bool)pISPDebugInfo->ISPDebugEnable;
			printk("isp_tuning_enable[%d], isp_capture_raw_data[%d], isp_debug_enable[%d] \r\n", \
				isp_tuning_enable, isp_capture_raw_data, isp_debug_enable);
			break;
		}
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
			MS2R_set_auto_flicker(*pFeatureData32);
	        break;
        case SENSOR_FEATURE_SET_TEST_PATTERN:
            break;
	#ifdef VENDOR_EDIT
	//feng.hu@mtk camera team modified for ms2r continuous af
        case SENSOR_FEATURE_DO_YUV_CAF:
			g_bHasFaces = MS2R_check_if_has_faces();
			if (g_bHasFaces)
			{
				mdelayFrameCnt = 10;
			}
			MS2R_DO_CAF((bool *)pFeatureData32);
			break;
	#endif
	    #ifdef QUICK_CAPTURE
	    //lanhe@MTKCamera,2013/03/14 ,add for silent preview sensor 
		case SENSOR_FEATURE_SILNET_PREVIEW:
			MS2R_monitor_sequence();
			break;
		#endif /*QUICK_CAPTURE*/
		case SENSOR_FEATURE_SET_ISP_CAPTURE:
			MS2R_capture_sequence();
			break;
        default:
            break;
    }
    return ERROR_NONE;
}	/* MS2RFeatureControl() */


SENSOR_FUNCTION_STRUCT	SensorFuncMS2R=
{
    MS2ROpen,
    MS2RGetInfo,
    MS2RGetResolution,
    MS2RFeatureControl,
    MS2RControl,
    MS2RClose
};

UINT32 MS2R_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc!=NULL)
        *pfFunc=&SensorFuncMS2R;

    return ERROR_NONE;
}   /* SensorInit() */

