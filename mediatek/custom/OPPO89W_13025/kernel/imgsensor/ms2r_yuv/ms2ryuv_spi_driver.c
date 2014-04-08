/************************************************************************************
** File: - camera_spi_driver.c
** OPPO_R819
** Copyright (C), 2008-2012, OPPO Mobile Comm Corp., Ltd
** 
** Description: 
**      The Camera SPI driver 
** 
** Version: 1.0
** Date created: 20:34:20,06/09/2012
** Author: LiuBin@MtkCamera
** 
** --------------------------- Revision History: --------------------------------
** 	<author>	<data>			<desc>
** 
************************************************************************************/
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/compat.h>
	
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
typedef enum{
   SPI_DMA = 0,
   SPI_FIFO,
   SPI_MAX,
}transfer_mode;
#define SPI_MS2_BIN_SIZE		(0x78800)
#define SPI_FIFO_SIZE (32)
#define SPI_DMA_SIZE (1024*256)
#define MAX_TRANSFER_CNT (SPI_MS2_BIN_SIZE/SPI_FIFO_SIZE)
struct spi_transfer stSPITransfer[MAX_TRANSFER_CNT];
static struct spi_device *g_pstSPIDevice = NULL;
int (*spi_transfer[SPI_MAX])(char *pBuf, unsigned int uSize) = {NULL};
static int camera_spi_sync(struct spi_message *pSpiMessage)
{
	int status = -1;

	printk("camera_spi_sync------->IN \r\n");
	status = spi_sync(g_pstSPIDevice, pSpiMessage);
	printk("camera_spi_sync---[%d]---->OUT \r\n", status);
	if (0 != status)
	{
		printk("spi_sync fail \r\n");
	}

	return status;
	
}

int camera_spi_read(char *pBuf, unsigned int uSize)
{
	struct spi_message stSPIMessage;
	struct spi_transfer stSPITransfer = 
	{
		.rx_buf = pBuf, 
		.len = uSize,
	};

	spi_message_init(&stSPIMessage);
	spi_message_add_tail(&stSPITransfer, &stSPIMessage);

	return camera_spi_sync(&stSPIMessage);
	
}
static int DMA_transfer(char *pBuf, unsigned int uSize)
{
	struct spi_message stSPIMessage;
	
	int i = 0;
	int cnt = uSize/SPI_DMA_SIZE;
	int remain = uSize%SPI_DMA_SIZE;
	
	printk("DO DMA_transfer! pBuf(0x%x);uSize(0x%x);remain(0x%x)\n",pBuf,uSize,remain);
	
	if((uSize > SPI_DMA_SIZE*MAX_TRANSFER_CNT)||(!pBuf))
		return -1;
	
	spi_message_init(&stSPIMessage);
	
	for(i=0;i<cnt;i++)
	{
		stSPITransfer[i].tx_buf = pBuf + SPI_DMA_SIZE*i;
		stSPITransfer[i].len = SPI_DMA_SIZE;
		spi_message_add_tail(&stSPITransfer[i], &stSPIMessage);	
	}
	if(remain)
	{
		stSPITransfer[i].tx_buf = pBuf + SPI_DMA_SIZE*i;
		stSPITransfer[i].len = remain;
		spi_message_add_tail(&stSPITransfer[i], &stSPIMessage);	
	}
	
	return camera_spi_sync(&stSPIMessage);
}
static int FIFO_transfer(char *pBuf, unsigned int uSize)
{
	struct spi_message stSPIMessage;
	int i = 0;
	int cnt = uSize/SPI_FIFO_SIZE;
	int remain = uSize%SPI_FIFO_SIZE;
	
	printk("DO FIFO_transfer! pBuf(0x%x);uSize(0x%x);remain(0x%x)\n",pBuf,uSize,remain);
	
	if((uSize > SPI_FIFO_SIZE*MAX_TRANSFER_CNT)||(!pBuf))
		return -1;

	printk("FIFO_transfer----SPI message start \r\n");
	spi_message_init(&stSPIMessage);
	printk("spi_message_init finish \r\n");
	for(i=0;i<cnt;i++)
	{
			stSPITransfer[i].tx_buf = pBuf + SPI_FIFO_SIZE*i;
			stSPITransfer[i].len = SPI_FIFO_SIZE;
			spi_message_add_tail(&stSPITransfer[i], &stSPIMessage);	
	}
	printk("FIFO transfer size [0x%x] \r\n", i*SPI_FIFO_SIZE);
	if(remain)
	{
		stSPITransfer[i].tx_buf = pBuf + SPI_FIFO_SIZE*i;
		stSPITransfer[i].len = remain;
		spi_message_add_tail(&stSPITransfer[i], &stSPIMessage);	
	}
	printk("FIFO_transfer----SPI message end \r\n");
	return camera_spi_sync(&stSPIMessage);
}
int camera_spi_write(char *pBuf, unsigned int uSize, u8 mode)
{	
	#if 0
	struct spi_message stSPIMessage;
	struct spi_transfer stSPITransfer = 
	{
		.tx_buf = pBuf, 
		.len = uSize,
	};

	spi_message_init(&stSPIMessage);
	spi_message_add_tail(&stSPITransfer, &stSPIMessage);
	
	return camera_spi_sync(&stSPIMessage);
	#else
	if(spi_transfer[mode])
		return spi_transfer[mode](pBuf,uSize);

	return -1;
	#endif
	
}


static int camera_spi_probe(struct spi_device *spi)
{
	spi->mode = SPI_MODE_1;
	spi->bits_per_word = 8;
	g_pstSPIDevice = spi;
	spi_transfer[SPI_DMA] = DMA_transfer;
	spi_transfer[SPI_FIFO] = FIFO_transfer;
	return 0;
	
}
static int camera_spi_remove(struct spi_device *spi)
{	
	spi_transfer[SPI_DMA] = NULL;
	spi_transfer[SPI_FIFO] = NULL;
	return 0;
}

static struct spi_driver camera_spi_driver = {
	.driver = {
		.name =		"ms2r-spi",
		.bus = 		&spi_bus_type,
		.owner =	THIS_MODULE,
	},
	.probe =	camera_spi_probe,
	.remove =	camera_spi_remove,
};

static struct spi_board_info __initdata spi_board_devs[] = {
	[0] = {
		.modalias="ms2r-spi",
		.bus_num = 0,
		.chip_select=0,
		.mode = SPI_MODE_1,
	},
};

static int __init ms2r_spi_dev_init(void)
{
	printk("ms2r_spi_dev_init.\n");
	spi_register_board_info(spi_board_devs, ARRAY_SIZE(spi_board_devs));
	return spi_register_driver(&camera_spi_driver);
}

static void __exit ms2r_spi_dev_exit(void)
{
	printk("ms2r_spi_dev_exit.\n");
	spi_unregister_driver(&camera_spi_driver);
	
	return;
}

module_init(ms2r_spi_dev_init);
module_exit(ms2r_spi_dev_exit);

MODULE_DESCRIPTION ( "ms2r spi device driver" );
MODULE_AUTHOR ( "LiuBin@MtkCamera <liub@oppo.com>" );
MODULE_LICENSE("GPL");


