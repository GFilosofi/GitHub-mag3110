/******************** (C) COPYRIGHT 2012 Freescale Semiconductor, Inc. *************
 *
 * File Name		: mag_3110.h
 * Authors		: Rick Zhang(rick.zhang@freescale.com)
 			  Rick is willing to be considered the contact and update points 
 			  for the driver
 * Version		: V.1.0.0
 * Date			: 2012/Mar/15
 * Description		: MAG3110  some marco declarations and register defines
 *
 ******************************************************************************
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
 * OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
 * PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
 * AS A RESULT, FREESCALE SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH FREESCALE PARTS.

 ******************************************************************************
 * Revision 1.0.0 3/15/2012 First Release;
 ******************************************************************************
*/

#ifndef __MAG_REGS_3110_H__
#define __MAG_REGS_3110_H__

/*! Macros for chip id */
#define ID_MAG3110	(0xC4)
//! An enum for MAG3110 registers
/*! Contains offset for hw registers */
enum {
	REG3110_DR_STATUS = 0x00,
	REG3110_OUT_X_MSB ,
	REG3110_OUT_X_LSB ,
	REG3110_OUT_Y_MSB ,
	REG3110_OUT_Y_LSB ,
	REG3110_OUT_Z_MSB ,
	REG3110_OUT_Z_LSB ,
	REG3110_WHO_AM_I ,
	REG3110_SYSMOD ,
	REG3110_OFF_X_MSB ,
	REG3110_OFF_X_LSB ,
	REG3110_OFF_Y_MSB ,
	REG3110_OFF_Y_LSB ,
	REG3110_OFF_Z_MSB ,
	REG3110_OFF_Z_LSB ,
	REG3110_DIE_TEMP ,
	REG3110_CTRL_REG1 ,
	REG3110_CTRL_REG2 ,
};

//! An enum 
/*! device states */
enum 
{
	STANDBY,
	WAKE,
};

//! An enum 
/*! data output modes */
enum 
{
	SCALED,
	RAW,
};

/*! Bit definations of hardware registers*/
#define XYZ_DATA_OVF	(0x01 << 7)
#define Z_DATA_OVF	(0x01 << 6)
#define Y_DATA_OVF	(0x01 << 5)
#define X_DATA_OVF	(0x01 << 4)
#define XYZ_DATA_RDY	(0x01 << 3)
#define Z_DATA_RDY	(0x01 << 2)
#define Y_DATA_RDY	(0x01 << 1)
#define X_DATA_RDY	(0x01 << 0)

#define SYS_MODE_MASK		0x03
#define SYS_MODE_STDBY		0x00
#define SYS_MODE_ACTIVE_RAW	0x01
#define SYS_MODE_ACTIVE_SCALED	0x02

#define DATA_RATE_MASK		0xE0
#define OVERSAMP_MASK		0x18
#define FAST_READ		0x04
#define TRIG_MEASURE		0x02
#define ACTIVE_MODE		0x01
#define STDBY_MODE		0x00

#define RAW_DATA_MASK		(0x01 << 4)
#define MAG_SEN_RESET		(0x01 << 3)
#define Z_SELT_TEST		(0x01 << 2)
#define Y_SELT_TEST		(0x01 << 1)
#define X_SELT_TEST		(0x01 << 0)
#endif // __MXC_REGS_H__
