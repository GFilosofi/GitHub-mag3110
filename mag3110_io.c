/******************** (C) COPYRIGHT 2012 Freescale Semiconductor, Inc. *************
 * Copyright (C) 2012 Cosmed, Ltd. <http://www.cosmed.com/>
 * Author: Gabriele Filosofi <gabrielef@cosmed.it>
 * Original Copyrights follows:
 *
 * File Name		: mag_3110.c
 * Authors		: Rick Zhang(rick.zhang@freescale.com)
 			  Rick is willing to be considered the contact and update points 
 			  for the driver
 * Version		: V.1.0.0
 * Date			: 2012/Mar/15
 * Description		: methods and data structure related to MAG3110 magnetometer sensor.
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
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/input-polldev.h>
#include <linux/hwmon.h>
#include <linux/regulator/consumer.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <mach/hardware.h>
#include <asm/delay.h>
#ifdef CONFIG_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include "mag3110_regs.h"
#include "mag3110_io.h"
#include "mag3110_sysfs.h"

/*! Declarations */
int InitializeChip(struct ChipInfo_t *pChipInfo);
static int ReadChip( int type, void *data);
static int SetChipMode( int mode );
static int GetChipMode( void );

extern int ParseString(char *Data, int Length, char Delimiter, char *Tokens[], int TokenCnt);
/*macro define*/
#define MMA3110_I2C_ADDR	0x1d

#define LOBYTE(W)	(unsigned char)((W) & 0xFF)
#define HIBYTE(W)	(unsigned char)(((W) >> 8) & 0xFF)

const char * Samplinglist[] = {
	"normal",
	"low_noise",
	"High_resolution",
	"low_power",
	NULL,
};

static const FIXPOINT OdrList[] = { 
	FLOAT_TO_FIXP(80.0),
	FLOAT_TO_FIXP(40.0),
	FLOAT_TO_FIXP(20.0),
	FLOAT_TO_FIXP(10.0),
	FLOAT_TO_FIXP(40.0),
	FLOAT_TO_FIXP(20.0),
	FLOAT_TO_FIXP(10.0),
	FLOAT_TO_FIXP(5.0),
	FLOAT_TO_FIXP(20.0),
	FLOAT_TO_FIXP(10.0),
	FLOAT_TO_FIXP(5.0),
	FLOAT_TO_FIXP(2.5),
	FLOAT_TO_FIXP(10.0),
	FLOAT_TO_FIXP(5.0),
	FLOAT_TO_FIXP(2.5),
	FLOAT_TO_FIXP(1.25),
	FLOAT_TO_FIXP(5.0),
	FLOAT_TO_FIXP(2.5),
	FLOAT_TO_FIXP(1.25),
	FLOAT_TO_FIXP(20.0),
	FLOAT_TO_FIXP(2.5),
	FLOAT_TO_FIXP(1.25),
	FLOAT_TO_FIXP(0.63),
	FLOAT_TO_FIXP(0.31),
	FLOAT_TO_FIXP(1.25),
	FLOAT_TO_FIXP(0.63),
	FLOAT_TO_FIXP(0.31),
	FLOAT_TO_FIXP(0.16),
	FLOAT_TO_FIXP(0.63),
	FLOAT_TO_FIXP(0.31),
	FLOAT_TO_FIXP(0.16),
	FLOAT_TO_FIXP(0.08),
	-1,
}; /*!< an array containing supported data rate list */


static const int SupportedResolution[] = {16, 8, -1};
/*! Supported range is -1 to +1 */
static const FIXPOINT SupportedRange[] = { INT_TO_FIXP(1), -1};

/*definition*/
static int ChipType = 0;	//MAG3110;
static struct i2c_client * pClient = NULL;
static struct ChipInfo_t * pChip = NULL;

/*!
* This method is used to write a hardware register of MAG3110.
* regaddr  : address of hardware register.
* start    : position of the bit from which data will be written.
* bitlengh : no of bits to be written.
* value    : data to be written. 	
* return 0 	    : For successful write.  
* return -EINVAL : For write failure  
*/ 
static int WriteRegValue(int regaddr , int startpos , int bitlength , int value)
{
	char ret = 0;
	
	if(value < (0x01 << bitlength)) {
		ret = i2c_smbus_read_byte_data(pClient, regaddr);
		ret &= ~(((0x01<<bitlength)-1)<<startpos);               // to reset the required bits
		ret |= value<<startpos;
		i2c_smbus_write_byte_data(pClient, regaddr, ret);
		ret = i2c_smbus_read_byte_data(pClient, regaddr);	
		return 0;
	}
	else
       	{
		printk("\t Invalid Value \r\n");
		return -EINVAL;
	}
}

/*!
* This method is used to set the offsets for magnetic field readings. 
* val 		: array contaning value to be written to offset registers.
* return 0  		: For successful write.  
* return -EINVAL 	: For write failure.  
 */ 
static int SetCalOffset(int val[])
{
	// Set Calibration offset
#if 1 /* gf16-07-2012 added: */
	short tmp;
	tmp = (short)( val[0] << 1 ) & 0xfffe;
	i2c_smbus_write_byte_data(pClient, (REG3110_OFF_X_MSB), HIBYTE(tmp));
	i2c_smbus_write_byte_data(pClient, (REG3110_OFF_X_LSB), LOBYTE(tmp));
	tmp = (short)( val[1] << 1 ) & 0xfffe;
	i2c_smbus_write_byte_data(pClient, (REG3110_OFF_Y_MSB), HIBYTE(tmp));
	i2c_smbus_write_byte_data(pClient, (REG3110_OFF_Y_LSB), LOBYTE(tmp));
	tmp = (short)( val[2] << 1 ) & 0xfffe;
	i2c_smbus_write_byte_data(pClient, (REG3110_OFF_Z_MSB), HIBYTE(tmp));
	i2c_smbus_write_byte_data(pClient, (REG3110_OFF_Z_LSB), LOBYTE(tmp));
#else
	i2c_smbus_write_byte_data(pClient, (REG3110_OFF_X_MSB), val[0]);
	i2c_smbus_write_byte_data(pClient, (REG3110_OFF_X_LSB), val[1]);
	i2c_smbus_write_byte_data(pClient, (REG3110_OFF_Y_MSB), val[2]);
	i2c_smbus_write_byte_data(pClient, (REG3110_OFF_Y_LSB), val[3]);
	i2c_smbus_write_byte_data(pClient, (REG3110_OFF_Z_MSB), val[4]);
	i2c_smbus_write_byte_data(pClient, (REG3110_OFF_Z_LSB), val[5]);
#endif
	return 0;
}

/*!
* This method is used to set different settings of MAG3110. 
* cmd  	: contains setting to be applied to MAG3110.
* val  	: contains data required to apply settings specified in cmd.
* return 0 		: For successful write.  
* return -EINVAL 	: For write failure.  
*/
static int SetRegVal(int cmd, int val)
{
	switch(cmd)
	{
		case CMD_MODE :
			WriteRegValue((REG3110_CTRL_REG1) , 0, 1, val);		
			break;

		case CMD_ODR:
			// Put device in standby
			WriteRegValue((REG3110_CTRL_REG1), 0, 1, STDBY_MODE);
			WriteRegValue((REG3110_CTRL_REG1), 5, 3, val);

			// Put device to original state
			if((pChip->state != STANDBY))  	
				WriteRegValue((REG3110_CTRL_REG1) , 0, 1, ACTIVE_MODE);			
			break;

		case CMD_SAMPLE:
			// Put device in standby
			WriteRegValue((REG3110_CTRL_REG1), 0, 1, STDBY_MODE);
			WriteRegValue((REG3110_CTRL_REG1), 3, 2, val);

			// Put device to original state
			if((pChip->state != STANDBY))  	
				WriteRegValue((REG3110_CTRL_REG1) , 0, 1, ACTIVE_MODE);			
			break;

		case CMD_RESOLUTION:

			// Put device in standby
			WriteRegValue((REG3110_CTRL_REG1), 0, 1, STDBY_MODE);
			/*!
			  When this bit is set 8 bit resoluion is set and when cleared 16 bit resolution is set   
			  0: The full 16bit values are read;  
			  1: Fast read, 8 bit values read from the MSB registers 			  
			 */
			WriteRegValue((REG3110_CTRL_REG1), 2, 1, val);
	
			// Put device to original state
			if((pChip->state != STANDBY))  	
				WriteRegValue((REG3110_CTRL_REG1) , 0, 1, ACTIVE_MODE);			
			break;	

		case CMD_TRIGGER:
			/*!
			  Trigger immediate measurement
			  0:normal operation based on AC condition  
			  1: trigger measurement 
			  If part is in active mode, any measurement in progress will complete before triggered 
			  measurement.  
			  In Standby mode triggered measurement will occur immediately and part will return to 
			  standby mode as soon as the measurement is complete. 			
			*/
			WriteRegValue((REG3110_CTRL_REG1), 1, 1, val);
			break;	

		case CMD_DATA_MODE:
			// Put device in standby
			WriteRegValue((REG3110_CTRL_REG1), 0, 1, STDBY_MODE);
			/*!
			  Raw data output mode.  
			  0: Normal, data values modified by offset register values;  
			  1: Raw mode data is not scaled by offset register values. 
			*/
			WriteRegValue((REG3110_CTRL_REG2), 5, 1, val);

			// Put device to original state
			if((pChip->state != STANDBY))  	
				WriteRegValue((REG3110_CTRL_REG1) , 0, 1, ACTIVE_MODE);	
			break;

		case CMD_SENSOR_RST:
			// Put device in standby
			WriteRegValue((REG3110_CTRL_REG1), 0, 1, STDBY_MODE);
			/*!
			  Magnetic Sensor Reset:  Default Value = 0 
			  0: Reset cycle not active  
			  1: Reset cycle initiate or Reset cycle busy / active 
			  When asserted, initiates a magnetic sensor reset cycle that will restore correct operation 
			  after exposure to excessive magnetic field. 
			  When the cycle is finished value returns to 0 
			*/
			WriteRegValue((REG3110_CTRL_REG2), 4, 1, 1);

			// Put device to original state
			if((pChip->state != STANDBY))  	
				WriteRegValue((REG3110_CTRL_REG1) , 0, 1, ACTIVE_MODE);	
			break;	

		default:
			printk("%s: DEFAULT... unhandled case\r\n", __func__);

	}
	if(pChip->state  != STANDBY)	
	  i2c_smbus_read_byte_data(pClient, (REG3110_OUT_X_MSB));
	return 0;
}

/*!
* This method is used to set initialize MAG3110 chip with default settings
* pChipInfo  	: Chip structure of MAG3110.
* return 0 		: For successful chip initialization.  
* return -EINVAL 	: For failure.  
*/
static int _f3110InitializeChip(struct ChipInfo_t *pChipInfo)
{
	unsigned char ret = 0;
	int i = 0;
	
	if(pChipInfo == NULL)
	{
		printk("%s:: NULL chip pointer\r\n", __func__);
		return -ENOMEM;
	}

	pChip = pChipInfo;
	pClient = pChip->client;
	ChipType = pChip->ChipType;

	// Put device in standby state
	ret = i2c_smbus_read_byte_data(pClient, (REG3110_CTRL_REG1));
	i2c_smbus_write_byte_data(pClient, (REG3110_CTRL_REG1), (ret & ~0x03));	
	/*!
	  Configure sensor for:
	    - System Output Data Rate of 80Hz 
	    - Fast read disabled.
	    - Trigger immediate disabled.
	    - Standby mode
	*/
	ret = 0x00;  

	i2c_smbus_write_byte_data(pClient, (REG3110_CTRL_REG1), ret);

	/*!
	 Configure sensor data for:
	   - Disable RAW output mode
	   - No self test on X, Y, Z axis
	*/
	ret = 0x80;
	i2c_smbus_write_byte_data(pClient, (REG3110_CTRL_REG2), ret);

	// Put device in active mode(+/-8g) 
	ret = i2c_smbus_read_byte_data(pClient, (REG3110_CTRL_REG1));
	ret |= ACTIVE_MODE;
	i2c_smbus_write_byte_data(pClient, (REG3110_CTRL_REG1), ret);
	
	/* Dump all registers */
	for(i = 0; i < 0x10; i++) 
	{
		ret = i2c_smbus_read_byte_data(pClient, i);
	}
	pChip->state = WAKE; 

	return 0;
}

/*!
* This method is used to read output sample data from MAG3110
* type 	: data type(FIFO or normal. FIFO mode not supported).
* data 	: pointer to a buffer in which sample data to be read.
* return 0 		: For successful chip initialization.  
* return -EINVAL 	: For failure.  
*/
static int ReadChip( int type, void *data)
{
	int ret = 0;
	unsigned char *buff;
	int cnt;
	int i = 0;
	short *ps;
	short *pd;

	switch(type)
	{
		case MAG_DATA:
			buff = (unsigned char *)data;

			ps = (short *)buff;
			pd = (short *)buff;

			// Read status register
			//ret  = (int)i2c_smbus_read_byte_data(pClient, (REG3110_DR_STATUS));				
			if(pChip->pSupportedResolution[pChip->resolution] == 0x10) //16-bit resolution
			{
				cnt = 6;
				// Read data
				ret = i2c_smbus_read_i2c_block_data(pClient, (REG3110_OUT_X_MSB), cnt, (u8 *) buff);
				for(i = 0; i < 3; i++)
				{
					*pd = __be16_to_cpu(*ps);
					pd++;
					ps++;
				}
			}
			else // if(pChip->pSupportedResolution[pChip->resolution] == 0x08) //8-bit resolution
			{
				cnt = 3;
				// Read data
				ret = i2c_smbus_read_i2c_block_data(pClient, (REG3110_OUT_X_MSB), cnt, (u8 *) buff);
				for(i = 0; i < 3; i++)
				{
					*pd++ = *buff++;	
				}
			}

			break;
		default:
			break;
		
	}
	return ret;
}

/*!
* This method is used to set chip mode as active or standby
* mode 	: mode to be set.
* return 0 		: For successful mode initialization.   
* return -EINVAL 	: if mode value is invalid.  
* return -ENOMEM 	: if chip pointer is NULL.  
*/
static int SetChipMode(int mode)
{
	int ret = 0;
	struct i2c_client * pClient;
	if(pChip == NULL)
	{
		printk("%s:: NULL chip pointer\r\n", __func__);
		return -ENOMEM;
	}
	/*!
	  Check if mode is other than stanby or active
	*/
	if(mode > 0x01)
	{
		printk("%s:: Invalid mode\r\n", __func__);
		return -EINVAL;
	}
	pClient = pChip->client;
	
	ret = i2c_smbus_read_byte_data(pClient, (REG3110_CTRL_REG1));
	ret |= mode;
	i2c_smbus_write_byte_data(pClient, (REG3110_CTRL_REG1), ret);
	if(mode == ACTIVE_MODE)
	{
		i2c_smbus_read_byte_data(pClient, (REG3110_OUT_X_MSB));
	}
	return 0;
}

/*!
* This method is used to read operation mode
* return 0 		: If chip mode set.  
* return -ENOMEM 	: If chip pointer is NULL.  
*/
static int GetChipMode( void )
{
	struct i2c_client * pClient;

	if(pChip == NULL)
	{
		printk("%s:: NULL chip pointer\r\n", __func__);
		return -ENOMEM;
	}

	pClient = pChip->client;
	ChipType = pChip->ChipType;

	return (i2c_smbus_read_byte_data(pClient, (REG3110_CTRL_REG1)) & MODE_MASK);
}

/*!
* This method is used to read current die temperature in degree Celsius 
* temp : Pointer to a variable in which read die temperature will be copied.  
*/
void GetMag3110DieTemp(char *temp)
{
	struct i2c_client * pClient;
	pClient = pChip->client;
	
	*temp = i2c_smbus_read_byte_data(pClient, (REG3110_DIE_TEMP));	
}

/* Sysfs info */
static struct device_attribute common_attributes[] = {
	__ATTR(name, 0444, mag_name_show, NULL),
	__ATTR(vendor, 0444, mag_vendor_show, NULL),
	__ATTR(device_id, 0444, mag_devid_show, NULL),
	__ATTR(version, 0444, mag_version_show, NULL),
	__ATTR(type, 0444, mag_type_show, NULL),
	__ATTR(max_range, 0444, mag_max_range_show, NULL),
	__ATTR(operation_mode, 0666, mag_operation_mode_show, mag_operation_mode_store),
	__ATTR(odr, 0666, mag_odr_show, mag_odr_store),
	__ATTR(supported_odr, 0444, mag_supported_odr_show, NULL),
	__ATTR(oversampling, 0666, mag_oversampling_show, mag_oversampling_store),
	__ATTR(oversampling_values, 0444, mag_oversampling_values_show, NULL),
	__ATTR(resolutions, 0444, mag_resolutions_show,  NULL),
	__ATTR(resolution, 0666, mag_resolution_show, mag_resolution_store),
	__ATTR(range, 0666, mag_range_show, mag_range_store),
	__ATTR(range_low, 0444, mag_range_low_show,  NULL),
	__ATTR(range_high, 0444, mag_range_high_show,  NULL),
	__ATTR(precision, 0444, mag_precision_show,  NULL),
	__ATTR(sample_rate, 0444, mag_sample_rate_show,  NULL),
	__ATTR(value, 0444, mag_value_show,  NULL),
	__ATTR(calibration_offset, 0666, mag_calibration_offset_show, mag_calibration_offset_store),
	__ATTR(die_temperature, 0444, mag_die_temp_show, NULL),
	__ATTR(data_output_mode, 0666, mag_output_mode_show, mag_output_mode_store),
	__ATTR(poll_time, 0666, mag_poll_time_show, mag_poll_time_store),
	__ATTR(trigger, 0666, NULL, mag_trigger_store),
};

static struct SysfsInfo_t SysfsInfo[] = {
	{NULL, &common_attributes[0], (sizeof(common_attributes)/sizeof(common_attributes[0])), 1},
};

struct ChipInfo_t mag3110Chip = {
	.id = 0, 	// MAG3110,
	.ChipId = ID_MAG3110,
	.Init = _f3110InitializeChip,
	.Read = ReadChip,
	.EnableInt = NULL,
	.DisableInt = NULL,
	.name = "MAG3110",
	.devtype = 1,
	.maxrange = FLOAT_TO_FIXP(1.0),	
	.range = 0,
	.OutputMode = SCALED,
	.poll_time = 5,

	.pOdrList = (FIXPOINT *)OdrList,
	.pSupportedResolution = (int *)SupportedResolution,
	.pSupportedRange = (FIXPOINT *)SupportedRange,
	.pSamplinglist = (char **)Samplinglist,

	.pSysfsInfo = SysfsInfo,
	.SysfsInfoSize = sizeof(SysfsInfo)/sizeof(SysfsInfo[0]),

	.SetMode = SetChipMode,
	.GetMode = GetChipMode,
	.GetIntSrc = NULL,
	.SetRegVal = SetRegVal,
	.SetCalOffset = SetCalOffset,
	.GetDieTemp = GetMag3110DieTemp,
};
