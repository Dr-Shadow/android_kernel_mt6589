/*
 * Copyright (C) 2010 MEMSIC, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

/*
 * Definitions for mmc3416x magnetic sensor chip.
 */
#ifndef __MMC3416X_H__
#define __MMC3416X_H__

#include <linux/ioctl.h>

#define MMC3416X_I2C_NAME		"mmc3416x"

#define MMC3416X_OFFSET_X		32768
#define MMC3416X_OFFSET_Y		32768
#define MMC3416X_OFFSET_Z		32768
#define MMC3416X_SENSITIVITY_X		2048
#define MMC3416X_SENSITIVITY_Y		2048
#define MMC3416X_SENSITIVITY_Z		2048

#define MMC3416X_I2C_ADDR		0x30
#define MMC3416X_REG_CTRL		0x07 //control 0
#define MMC3416X_REG_BITS		0x08 //control 1
#define MMC3416X_REG_DATA		0x00//data 
#define MMC3416X_REG_DS			0x06//device status
#define MMC3416X_REG_PRODUCTID_0		0x10
#define MMC3416X_REG_PRODUCTID_1		0x20

#define MMC3416X_CTRL_TM			0x01//take measurment
#define MMC3416X_CTRL_CM			0x02//continue mode 
#define MMC3416X_CTRL_50HZ		0x00// set mode 00
#define MMC3416X_CTRL_25HZ		0x04// set mode 01
#define MMC3416X_CTRL_12HZ		0x08// set mode 10
#define MMC3416X_CTRL_NOBOOST            0x10//disable charge pump
#define MMC3416X_CTRL_SET  	        0x20
#define MMC3416X_CTRL_RESET              0x40
#define MMC3416X_CTRL_REFILL             0x80 //recharge pump,do before set/reset

#define MMC3416X_BITS_SLOW_16            0x00
#define MMC3416X_BITS_FAST_16            0x01
#define MMC3416X_BITS_14                 0x02
/* Use 'm' as magic number */
#define MMC3416X_IOM			'm'

/* IOCTLs for MMC3416X device */
#define MMC3416X_IOC_TM			_IO (MMC3416X_IOM, 0x00)
#define MMC3416X_IOC_SET			_IO (MMC3416X_IOM, 0x01)
#define MMC3416X_IOC_READ		_IOR(MMC3416X_IOM, 0x02, int[3])
#define MMC3416X_IOC_READXYZ		_IOR(MMC3416X_IOM, 0x03, int[3])
#define MMC3416X_IOC_RESET               _IO (MMC3416X_IOM, 0x04)
#define MMC3416X_IOC_NOBOOST             _IO (MMC3416X_IOM, 0x05)
#define MMC3416X_IOC_ID                  _IOR(MMC3416X_IOM, 0x06, short)
#define MMC3416X_IOC_DIAG                _IOR(MMC3416X_IOM, 0x14, int[1])

//
#define CONVERT_M			25
#define CONVERT_M_DIV				8192		
#define CONVERT_O			45
#define CONVERT_O_DIV		8192
#define MMC3416X_CTRL_RM    MMC3416X_CTRL_RESET
#define MMC3416X_RETRY_COUNT		3
#define MMC3416X_SET_INTV			250
#define MMC3416X_I2C_DELAY			2		// ms
#define MMC3416X_DELAY_TM			10		// ms
#define MMC3416X_DELAY_SET			50		// ms
#define MMC3416X_DELAY_RESET     	50  	// ms
#define MMC3416X_DEFAULT_DELAY		10//100		// ms

#define SENSOR_DATA_SIZE			8
#define CALIBRATION_DATA_SIZE		12
#define MMC3416X_BUFSIZE			0x20

#define POWER_ON					1
#define POWER_OFF					0
#define MMC3416X_SUCCESS			0
#define MMC3416X_I2C_ERR			-1


#endif /* __MMC3416X_H__ */

