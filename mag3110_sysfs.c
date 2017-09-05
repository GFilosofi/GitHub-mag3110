/******************** (C) COPYRIGHT 2012 Freescale Semiconductor, Inc. *************
 *
 * File Name		: mag_sysfs.c
 * Authors		: Rick Zhang(rick.zhang@freescale.com)
 			  Rick is willing to be considered the contact and update points 
 			  for the driver
 * Version		: V.1.0.0
 * Date			: 2012/Mar/15
 * Description		: MAG3110 methods and data structure related sysfs module
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
#include <asm/uaccess.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include "mag3110_regs.h"
#include <mach/hardware.h>
#include "mag3110_io.h"

#define LEN(x)	(sizeof(x)/sizeof(x[0]))

/*!
* This method is used to parse the string according to delimiter. 
* Data 	: Pointer to the buffer containing string to be parsed.
* Length	: length of string in bytes.
* Token 	: Token for parsing the string .
* TokenCnt 	: length of token string.
* return TokenCount 	: No of token occurrences found in input string.  
*/
static int ParseString(char *Data, int Length, char Delimiter, char *Tokens[], int TokenCnt)
{
	int TokenCount = 0;
	int Iterator;

	TokenCnt--;
	Tokens[TokenCount++] = Data;
	for(Iterator = 0; (Iterator < Length) && TokenCnt; Iterator++)
	{
		if(Data[Iterator] == Delimiter)
		{
			Data[Iterator] = '\0';
			Tokens[TokenCount] = &Data[Iterator + 1];
			TokenCount++;
			TokenCnt--;
		}
	}
	Iterator = 0;
	while(Iterator < TokenCount)
	{
		printk("TOKEN %d: [%s]\r\n", Iterator, Tokens[Iterator]);
		Iterator++;
	}

	return TokenCount;
}

/*!
* This method is used to get name of chip. 
* dev 	: Pointer to device structure.
* attr : Pointer to device attributes.
* buf 	: Pointer to buffer in which name of chip will be copied.
* return  	: Length of string.  
*/
ssize_t mag_name_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int RetVal = 0;

	struct mxc_mag_device_t *pDev = dev_get_drvdata(dev);
	struct ChipInfo_t *pChip = pDev->pChip;
	if(pDev)
		sprintf(buf, "%s\n", pChip->name);
	RetVal = strlen(buf) + 1;

	return RetVal;
}

/*!
* This method is used to get name of vendor. 
* dev 	: Pointer to device structure.
* attr : Pointer to device attributes.
* buf 	: Pointer to buffer in which name of vendor will be copied.
* return  	: Length of string.  
*/
ssize_t mag_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	sprintf(buf, "%s\n", VENDOR_NAME);
	ret = strlen(buf) + 1;

	return ret;
}

/*!
* This method is used to get ID of chip. 
* dev 	: Pointer to device structure.
* attr : Pointer to device attributes.
* buf 	: Pointer to buffer in which ID of chip will be copied.
* return  	: Length of string.  
*/
ssize_t mag_devid_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	struct mxc_mag_device_t *pDev = dev_get_drvdata(dev);
	struct ChipInfo_t *pChip = pDev->pChip;

	if(pDev)
		sprintf(buf, "0x%x\n", pChip->ChipId);
	ret = strlen(buf) + 1;

	return ret;
}

/*!
* This method is used to get version of chip driver. 
* dev 	: Pointer to device structure.
* attr : Pointer to device attributes.
* buf 	: Pointer to buffer in which version of chip will be copied.
* return  	: Length of string.  
*/
ssize_t mag_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct mxc_mag_device_t *pDev = dev_get_drvdata(dev);

	if(pDev)
		sprintf(buf, "%d", pDev->version);
	ret = strlen(buf) + 1;

	return ret;
}

/*!
* This method is used to get name of chip. 
* dev 	: Pointer to device structure.
* attr : Pointer to device attributes.
* buf 	: Pointer to buffer in which type of chip will be copied.
* return  	: Length of string.  
*/
ssize_t mag_type_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct mxc_mag_device_t *pDev = dev_get_drvdata(dev);
	struct ChipInfo_t *pChip = pDev->pChip;

	sprintf(buf, "%d", pChip->devtype);
	ret = strlen(buf) + 1;

	return ret;
}

/*!
* This method is used to get max range. 
* dev 	: Pointer to device structure.
* attr : Pointer to device attributes.
* buf 	: Pointer to buffer in which max range of chip will be copied.
* return  	: Length of string.  
*/
ssize_t mag_max_range_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	struct mxc_mag_device_t *pDev = dev_get_drvdata(dev);
	struct ChipInfo_t *pChip = pDev->pChip;

	sprintf(buf, "%d.%d\n", FIXP_INT_PART(pChip->maxrange), FIXP_DEC_PART(pChip->maxrange));
	ret = strlen(buf) + 1;

	return ret;
}

/*!
* This method is used to get current set range and supported ranges. 
* dev 	: Pointer to device structure.
* attr : Pointer to device attributes.
* buf 	: Pointer to buffer in which ranges of chip will be copied.
* return  	: Length of string.  
*/
ssize_t mag_range_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{	
	ssize_t ret = 0;
	int i = 0;
	int len = 0; 

	struct mxc_mag_device_t	*pDev = dev_get_drvdata(dev);
	struct ChipInfo_t *pChip = pDev->pChip;

	sprintf(buf, "-%d.%d:%d.%d;", FIXP_INT_PART(pChip->pSupportedRange[pChip->range]), FIXP_DEC_PART(pChip->pSupportedRange[pChip->range]),
				      FIXP_INT_PART(pChip->pSupportedRange[pChip->range]), FIXP_DEC_PART(pChip->pSupportedRange[pChip->range]));
		len = strlen(buf);

	for(i = 0; pChip->pSupportedRange[i] != -1; i++)
	{
		sprintf(&buf[len],"-%d.%d:%d.%d,", FIXP_INT_PART(pChip->pSupportedRange[i]), FIXP_DEC_PART(pChip->pSupportedRange[i]),
						   FIXP_INT_PART(pChip->pSupportedRange[i]), FIXP_DEC_PART(pChip->pSupportedRange[i]));
		
		len = strlen(buf);
	}
	
	buf[len - 1] = '\n';
	buf[len] = '\0';
	ret = strlen(buf) + 1;
	
	return ret;
}

/*!
* This method is used to get min range. 
* dev 	: Pointer to device structure.
* attr : Pointer to device attributes.
* buf 	: Pointer to buffer in which min range of chip will be copied.
* return  	: Length of string.  
*/
ssize_t mag_range_low_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	struct mxc_mag_device_t *pDev = dev_get_drvdata(dev);
	struct ChipInfo_t *pChip = pDev->pChip;

	sprintf(buf, "-%d.%d\n", FIXP_INT_PART(pChip->pSupportedRange[pChip->range]), FIXP_DEC_PART(pChip->pSupportedRange[pChip->range]));
	ret = strlen(buf) + 1;

	return ret;
}

/*!
* This method is used to get max range. 
* dev 	: Pointer to device structure.
* attr : Pointer to device attributes.
* buf 	: Pointer to buffer in which max range of chip will be copied.
* return  	: Length of string.  
*/
ssize_t mag_range_high_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	struct mxc_mag_device_t *pDev = dev_get_drvdata(dev);
	struct ChipInfo_t *pChip = pDev->pChip;

	sprintf(buf, "%d.%d\n", FIXP_INT_PART(pChip->pSupportedRange[pChip->range]), FIXP_DEC_PART(pChip->pSupportedRange[pChip->range]));
	ret = strlen(buf) + 1;

	return ret;
}

char * DeviceState[] = {
	"standby",
	"wake",
	NULL
};

/*!
* This method is used to get current operation mode. 
* dev 	: Pointer to device structure.
* attr : Pointer to device attributes.
* buf 	: Pointer to buffer in which operation mode of chip will be copied.
* return  	: Length of string.  
*/
ssize_t mag_operation_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	struct mxc_mag_device_t *pDev = dev_get_drvdata(dev);
	struct ChipInfo_t *pChip = pDev->pChip;
	
	sprintf(buf, "%s\n", DeviceState[pChip->state]);
	ret = strlen(buf) + 1;

	return ret;
}

/*!
* This method is used to set operation mode. 
* dev 		: Pointer to device structure.
* attr 	: Pointer to device attributes.
* buf 		: Pointer to buffer containing operation mode to be set.
* return  		: Length of string for valid operation mode.  
* return -EINVAL 	: For invalid operation mode.  
*/
ssize_t mag_operation_mode_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	ssize_t ret = 0;
	int i = 0;

	struct mxc_mag_device_t *pDev = dev_get_drvdata(dev);
	struct ChipInfo_t *pChip = pDev->pChip;

	for(i = 0; DeviceState[i] != NULL; i++)
	{
		if(!strncmp(DeviceState[i], buf, count-1))
			break;
	}
	
	if(DeviceState[i] != NULL)
	{
		// Set operational mode
		ret = pChip->SetRegVal(CMD_MODE, i);
		pChip->state = i;
		ret = count;
	}
	else
	{
		ret = -EINVAL;
	}

	return ret;
}

/*!
* This method is used to get current output data rate. 
* dev 	: Pointer to device structure.
* attr : Pointer to device attributes.
* buf 	: Pointer to buffer in which output data rate of chip will be copied.
* return  	: Length of string.  
*/

ssize_t mag_odr_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	struct mxc_mag_device_t *pDev = dev_get_drvdata(dev);
	struct ChipInfo_t *pChip = pDev->pChip;

	sprintf(buf,"%d.%d\n",FIXP_INT_PART(pChip->pOdrList[pChip->oversampling+(pChip->odr*4)]),
			      FIXP_DEC_PART(pChip->pOdrList[pChip->oversampling +(pChip->odr*4)]));

	ret = strlen(buf) + 1;

	return ret;
}

/*!
* This method is used to set operation mode. 
* dev 	: Pointer to device structure.
* attr : Pointer to device attributes.
* buf 	: Pointer to buffer containing output data rate to be set.
* return  	: Length of string.  
*/

ssize_t mag_odr_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int i = 0;
	size_t len = 0; 
	char tmpBuf[8] = "";

	struct mxc_mag_device_t *pDev = dev_get_drvdata(dev);
	struct ChipInfo_t *pChip = pDev->pChip;
	
	for(i=0;i<8;i++)
	{
		len =sprintf(tmpBuf,"%d.%d", FIXP_INT_PART(pChip->pOdrList[pChip->oversampling+(i*4)]),
					FIXP_DEC_PART(pChip->pOdrList[pChip->oversampling+(i*4)]));
		if(0 == strncmp(tmpBuf, buf, min(len, count)))
		{
			//  Set ODR here
			pChip->SetRegVal(CMD_ODR, i);
			pChip->odr = i;
			return count;
		}
	}
	
	return -EINVAL;
}

/*!
* This method is used to get list of supported output data rate. 
* dev 	: Pointer to device structure.
* attr : Pointer to device attributes.
* buf 	: Pointer to buffer in which supported output data rates of chip will be copied.
* return  	: Length of string.  
*/
ssize_t mag_supported_odr_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	
	ssize_t ret = 0;
	int len = 0;
	int i = 0;

	struct mxc_mag_device_t *pDev = dev_get_drvdata(dev);
	struct ChipInfo_t *pChip = pDev->pChip;

	for(i=0;i<8;i++)
	{
		sprintf(&buf[len], "%d.%d;", FIXP_INT_PART(pChip->pOdrList[pChip->oversampling +(i*4)]),
					     FIXP_DEC_PART(pChip->pOdrList[pChip->oversampling +(i*4)]));
		len = strlen(buf);

	}

	buf[len - 1] = '\0';
	ret = strlen(buf) + 1;

	return ret;
}

/*!
* This method is used to get current oversampling. 
* dev 	: Pointer to device structure.
* attr : Pointer to device attributes.
* buf 	: Pointer to buffer in which current oversampling value of chip will be copied.
* return  	: Length of string.  
*/

 ssize_t mag_oversampling_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	struct mxc_mag_device_t *pDev = dev_get_drvdata(dev);
	struct ChipInfo_t *pChip = pDev->pChip;

	sprintf(buf, "%s\n", pChip->pSamplinglist[pChip->oversampling]);
	ret = strlen(buf) + 1;

	return ret;
}


/*!
* This method is used to set oversampling. 
* dev 	: Pointer to device structure.
* attr : Pointer to device attributes.
* buf 	: Pointer to buffer containing oversampling to be set.
* return  	: Length of string.  
*/
ssize_t mag_oversampling_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int i = 0;
	int ret = 0;

	struct mxc_mag_device_t *pDev = dev_get_drvdata(dev);
	struct ChipInfo_t *pChip = pDev->pChip;

	for(i = 0; pChip->pSamplinglist[i] != NULL; i++)
	{
		if(!strncmp(pChip->pSamplinglist[i], buf, count-1))
			break;
	}
	
	if(pChip->pSamplinglist[i] != NULL)
	{
		// Set operational mode
		ret = pChip->SetRegVal(CMD_SAMPLE, i);
		pChip->oversampling = i;
		ret = count;
	}
	else
	{
		ret = -EINVAL;
	}

	return ret;
}

/*!
* This method is used to get list of oversampling values. 
* dev 	: Pointer to device structure.
* attr : Pointer to device attributes.
* buf 	: Pointer to buffer in which oversampling values of chip will be copied.
* return  	: Length of string.  
*/
ssize_t mag_oversampling_values_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0,len =0;
	int i = 0;

	struct mxc_mag_device_t *pDev = dev_get_drvdata(dev);
	struct ChipInfo_t *pChip = pDev->pChip;

	while(pChip->pSamplinglist[i] != NULL)
	{
	sprintf(&buf[len],"%s;",pChip->pSamplinglist[i]);
	len = strlen(buf);
	i++;
	}
	ret = strlen(buf) + 1;

	return ret;	
}


/*!
* This method is used to get list of supported resolutions. 
* dev 	: Pointer to device structure.
* attr : Pointer to device attributes.
* buf 	: Pointer to buffer in which supported resolutions of chip will be copied.
* return  	: Length of string.  
*/
ssize_t mag_resolutions_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	int len = 0;
	int i = 0;

	struct mxc_mag_device_t *pDev = dev_get_drvdata(dev);
	struct ChipInfo_t *pChip = pDev->pChip;

	while(pChip->pSupportedResolution[i] != -1)
	{
		sprintf(&buf[len], "%d;",pChip->pSupportedResolution[i]);
		len = strlen(buf);
		printk("pChip->pSupportedResolution[i] = %d \r\n",pChip->pSupportedResolution[i]);
		i++;

	}
	buf[len - 1] = '\0';
	ret = strlen(buf) + 1;

	return ret;
}

/*!
* This method is used to set resolution. 
* dev 		: Pointer to device structure.
* attr 	: Pointer to device attributes.
* buf 		: Pointer to buffer containing resolution to be set.
* count 	: Length of string  
* return  		: Length of string for valid resolution value.  
* return  -EINVAL 	: For invalid resolution value. 
*/
ssize_t mag_resolution_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int i = 0;
	unsigned long val = 0;

	struct mxc_mag_device_t *pDev = dev_get_drvdata(dev);
	struct ChipInfo_t *pChip = pDev->pChip;
	
	strict_strtoul(buf, 10, &val);

	for(i = 0; pChip->pSupportedResolution[i] != -1; i++)
	{
		if(val == pChip->pSupportedResolution[i])
		{
			printk("Setting resolution to %d (%d)\r\n", i, pChip->pSupportedResolution[i]);
			pChip->SetRegVal(CMD_RESOLUTION, i);
			pChip->resolution = i;
			return count;
		}
	}

	printk("Invalid Resolution, (buf: %s count: %d)\n", buf, count);
	return -EINVAL;
}

/*!
* This method is used to get current resolution. 
* dev 	: Pointer to device structure.
* attr : Pointer to device attributes.
* buf 	: Pointer to buffer in which current resolution of chip will be copied.
* return  	: Length of string.  
*/
ssize_t mag_resolution_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	struct mxc_mag_device_t *pDev = dev_get_drvdata(dev);
	struct ChipInfo_t *pChip = pDev->pChip;

	sprintf(buf, "%d\n", pChip->pSupportedResolution[pChip->resolution]);
	ret = strlen(buf) + 1;

	return ret;
}

/*!
* This method is used to set range. 
* dev 		: Pointer to device structure.
* attr 	: Pointer to device attributes.
* buf 		: Pointer to buffer containing range to be set.
* count 	: Length of string  
* return  		: Length of string for valid range value.  
* return  -EINVAL 	: For invalid range value.     
*/
ssize_t mag_range_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int i = 0;
	size_t len = 0; 
	char tmpBuf[8] = "";

	struct mxc_mag_device_t *pDev = dev_get_drvdata(dev);
	struct ChipInfo_t *pChip = pDev->pChip;

	while(pChip->pSupportedRange[i] != -1)
	{
		len = sprintf(tmpBuf, "%d.%d", FIXP_INT_PART(pChip->pSupportedRange[i]), FIXP_DEC_PART(pChip->pSupportedRange[i]));
		if(0 == strncmp(tmpBuf, buf, min(len, count)))
		{
			//  Set ODR here
			printk("Setting range to %d (%d.%d)\r\n", i,  FIXP_INT_PART(pChip->pSupportedRange[i]), FIXP_DEC_PART(pChip->pSupportedRange[i]));
			pChip->SetRegVal(CMD_RANGE, i);
			pChip->range = i;
			return count;
		}
		i++;
	}

	printk("Invalid Range, (buf: %s count: %d)\n", buf, count);
	return -EINVAL;
}

/*!
* This method is used to get current resolution. 
* dev 	: Pointer to device structure.
* attr : Pointer to device attributes.
* buf 	: Pointer to buffer in which current resolution of chip will be copied.
* return  	: Length of string.  
*/
ssize_t mag_precision_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	FIXPOINT precision;

	struct mxc_mag_device_t *pDev = dev_get_drvdata(dev);
	struct ChipInfo_t *pChip = pDev->pChip;

	precision = (pChip->pSupportedRange[pChip->range] * FLOAT_TO_FIXP(1) * 1000) / (0x1 << (pChip->pSupportedResolution[pChip->resolution] - 1));
	precision = precision * FLOAT_TO_FIXP(0.001); 
	sprintf(buf, "%d.%01d nTesla\n", FIXP_INT_PART(precision), FIXP_DEC_PART(precision));
	ret = strlen(buf) + 1;

	return ret;
}

/*!
* This method is used to get current real sample rate . 
* dev 	: Pointer to device structure.
* attr : Pointer to device attributes.
* buf 	: Pointer to buffer in which current current real sample rate of chip will be copied.
* return  	: Length of string.  
*/
ssize_t mag_sample_rate_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{

	ssize_t ret = 0;

	struct mxc_mag_device_t *pDev = dev_get_drvdata(dev);
	struct ChipInfo_t *pChip = pDev->pChip;

	sprintf(buf, "%d.%d\n", FIXP_INT_PART(pChip->pOdrList[pChip->odr]), FIXP_DEC_PART(pChip->pOdrList[pChip->odr]));
	ret = strlen(buf) + 1;

	return ret;
}

/*!
* This method is used to get raw integer data values of sensor. 
* dev 	: Pointer to device structure.
* attr : Pointer to device attributes.
* buf 	: Pointer to buffer in which current real sample rate of chip will be copied.
* return  	: Length of string.  
*/
ssize_t mag_value_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	/*! 
	  @todo : Value should Semi coma separated list of coma separated timestamp_uS:x,y,z raw value integer readings, as
	  they come from the sensor. This is circular list of the last 32 sensor readings, where the first value set is the 
	  oldest value and the last value set is the newest value. Please note that if resolution changes, the FIFO would be
	  flashed and the older values will be purged. 
	*/
#if 1 /* gf10-07-2012 added: */
	short buff[512];
	struct mxc_mag_device_t *pDev = dev_get_drvdata(dev);
	struct ChipInfo_t *pChip = pDev->pChip;

	pChip->Read(MAG_DATA, (void *)buff);

	sprintf(buf, "%d,%d,%d\n", buff[0],buff[1],buff[2]);
#endif
	ret = strlen(buf) + 1;

	return ret;
}

/*!
* This method is used to set calibration offsets. 
* dev 		: Pointer to device structure.
* attr 	: Pointer to device attributes.
* buf 		:  Pointer to buffer containing calibration offset values to be set. 
* count 	: Length of string  
* return  		: Length of string for calibration offset values.  
* return  -EINVAL 	: For invalid calibration offset values.      
*/
ssize_t mag_calibration_offset_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	int i=0;
	char *params[3];
	int val[3];
#if 1 /* gf16-07-2012 added: */
	long lval[3];
#else
	unsigned long lval[3];
#endif

	struct mxc_mag_device_t *pDev = dev_get_drvdata(dev);
	struct ChipInfo_t *pChip = pDev->pChip;

	ParseString((char *)buf, strlen(buf), ',', params, 3);
	while(params[i] != NULL && i < 3)
	{ 
		if (strict_strtol(params[i], 10, &lval[i]))
			return -EINVAL;
#if 1 /* gf16-07-2012 added: */
		if((lval[i] > 10000) || (lval[i] < -10000))
#else
		if(lval[i] > 255)
#endif
		{
			printk("Invalid calibration offset_ value(buf: %s count: %d)\n", buf, count);
			return -EINVAL;
		}
		val[i] = lval[i];
		i++;
	}
	pChip->SetCalOffset(val);
	pChip->xCalOffset = val[0];
	pChip->yCalOffset = val[1];
	pChip->zCalOffset = val[2];
	return count;
}

/*!
* This method is used to get calibration offset values. 
* dev 	: Pointer to device structure.
* attr : Pointer to device attributes.
* buf 	: Pointer to buffer in which calibration offset values will be copied.
* return  	: Length of string.  
*/
ssize_t mag_calibration_offset_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	struct mxc_mag_device_t *pDev = dev_get_drvdata(dev);
	struct ChipInfo_t *pChip = pDev->pChip;

	sprintf(buf, "%d,%d,%d\n", pChip->xCalOffset,pChip->yCalOffset,pChip->zCalOffset);
	ret = strlen(buf) + 1;

	return ret;
}

/*!
* This method is used to set chip in trigger measurement mode. 
* dev 		: Pointer to device structure.
* attr 	: Pointer to device attributes.
* buf 		: Pointer to buffer containing trigger set value.
* count 	: Length of string  
* return  		: Length of string for valid trigger value.  
* return  -EINVAL 	: For invalid trigger value.       
*remark 		: Magnetometer chip takes reading periodically. When we apply this setting 
    it forces magnetometer to take immediate readings.   
*/
ssize_t mag_trigger_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct mxc_mag_device_t *pDev = dev_get_drvdata(dev);
	struct ChipInfo_t *pChip = pDev->pChip;

	if(!strncmp(buf, "enable", count-1))
	{
		printk("Put chip in trigger measurement mode\r\n");
		pChip->SetRegVal(CMD_TRIGGER, 1);
	}
	else
	{
		printk("Invalid value. (buf: %s count: %d)\n", buf, count);
		return -EINVAL;
	}

	return -EINVAL;
}

/*!
* This method is used to set output data mode as raw or scaled. 
* dev 		: Pointer to device structure.
* attr 	: Pointer to device attributes.
* buf 		: Pointer to buffer containing output mode to be set.
* count 	: Length of string  
* return  		: Length of string for valid output mode value.  
* return  -EINVAL 	: For invalid output mode value.         
*/
ssize_t mag_output_mode_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct mxc_mag_device_t *pDev = dev_get_drvdata(dev);
	struct ChipInfo_t *pChip = pDev->pChip;
	
	if(!strncmp(buf, "raw", count-1))
	{
		printk("Put chip in raw data output mode\r\n");
		pChip->SetRegVal(CMD_DATA_MODE, 1);
		pChip->OutputMode = RAW;
	}
	else if(!strncmp(buf, "scaled", count-1))
	{
		printk("Put chip in scaled data output mode\r\n");
		pChip->SetRegVal(CMD_DATA_MODE, 0);
		pChip->OutputMode = SCALED;
	}	
	else
	{
		printk("Invalid value. (buf: %s count: %d)\n", buf, count);
		return -EINVAL;
	}

	return -EINVAL;
}

/*!
* This method is used to get data output mode. 
* dev 	: Pointer to device structure.
* attr : Pointer to device attributes.
* buf 	: Pointer to buffer in which current data output mode will be copied.
* return  	: Length of string.  
*/
ssize_t mag_output_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	struct mxc_mag_device_t *pDev = dev_get_drvdata(dev);
	struct ChipInfo_t *pChip = pDev->pChip;

	sprintf(buf, "%s\n", (pChip->OutputMode == SCALED)? "scaled" : "raw");
	ret = strlen(buf) + 1;

	return ret;
}


/*!
* This method is used to set polling time. 
* dev 		: Pointer to device structure.
* attr 	: Pointer to device attributes.
* buf 		: Pointer to buffer containing poll timer value to be set.
* count 	: Length of string  
* return  		: Length of string for valid poll timer value.  
* return  -EINVAL    : Invalid poll timer value.
*/
ssize_t mag_poll_time_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct mxc_mag_device_t *pDev = dev_get_drvdata(dev);
	struct ChipInfo_t *pChip = pDev->pChip;
	unsigned long val = 0;

	strict_strtoul(buf, 10, &val);

	if (val <= 0) 
        {
                return -EINVAL;
        }

	printk("poll time set to: %d\n", pChip->poll_time);

	pChip->poll_time = ( val / 10);


	return count;
}

/*!
* This method is used to get poll time.
* dev 	: Pointer to device structure.
* attr : Pointer to device attributes.
* buf 	: Pointer to buffer in which current data poll time value will be copied.
* return  	: Length of string.  
*/
ssize_t mag_poll_time_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	struct mxc_mag_device_t *pDev = dev_get_drvdata(dev);
	struct ChipInfo_t *pChip = pDev->pChip;

	sprintf(buf, "%d\n", pChip->poll_time * 10);
	ret = strlen(buf) + 1;

	return ret;
}


/*!
* This method is used to get current chip die temperature. 
* dev 	: Pointer to device structure.
* attr : Pointer to device attributes.
* buf 	: Pointer to buffer in which current temperature will be copied.
* return  	: Length of string.  
*/
ssize_t mag_die_temp_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	char temp = 0;

	struct mxc_mag_device_t *pDev = dev_get_drvdata(dev);
	struct ChipInfo_t *pChip = pDev->pChip;

	pChip->GetDieTemp(&temp);

	sprintf(buf, "%d\n", temp);
	ret = strlen(buf) + 1;

	return ret;
}

/*!
* This method is used to initialize sysfs. 
* client 	: Pointer to i2c client structure.
* return -EPERM 	: If device pointer is null.  
* return 0 		: If class device successfully created.  
*/

extern struct class *sensor_class_obj;

int magInitializeSysfs(struct i2c_client *client)
{
	int RetVal = 0;
	int i = 0;
	int Iterator = 0;
	int Instance = 0;
	struct mxc_mag_device_t *pDev = i2c_get_clientdata(client);
	struct ChipInfo_t *pChip = pDev->pChip;
	struct SysfsInfo_t *pSysfsInfo = pChip->pSysfsInfo;

	if(pDev == NULL)
	{
		printk("%s: pDev => NULL pointer\r\n", __func__);
		return -EPERM;
	}

	pDev->class = class_create(THIS_MODULE, "mag");
	if (IS_ERR(pDev->class)) {
			printk(KERN_ERR "Unable to create class for Mxc mag\n");
			RetVal = PTR_ERR(pDev->class);
		}

	pDev->sys_device = device_create(pDev->class, NULL, MKDEV(pDev->major, 0), pDev,
			     "mag");
  
	if (IS_ERR(pDev->sys_device)) {
		printk(KERN_ERR "Unable to create class device for Mxc Ipu\n");
		RetVal = PTR_ERR(pDev->sys_device);
		return RetVal;
	}

	dev_set_drvdata(pDev->sys_device, pDev);

	/* Create common entries */
	for(Iterator = 0; Iterator < pChip->SysfsInfoSize; Iterator++)
	{
		for(Instance = 0; Instance < pSysfsInfo[Iterator].Instance; Instance++)
		{		
			for(i=0; i < pSysfsInfo[Iterator].TotalEntries; i++)
			{
				if(sysfs_create_file(&pDev->sys_device->kobj, &pSysfsInfo[Iterator].AttrEntry[i].attr) < 0)
					printk("%s sys file creation failed.\r\n", pSysfsInfo[Iterator].AttrEntry[i].attr.name);
			}

		}
	}

	return RetVal;	
}

/*!
* This method is used to de-initialize sysfs. 
* client 	: Pointer to i2c client structure.
* return -EPERM 	: If device pointer is null.  
* return 0 		: If class device successfully destroyed.  
*/
int magDeInitializeSysfs(struct mxc_mag_device_t *pDev)
{
	if(pDev == NULL)
	{
		printk("%s: pDev => NULL pointer\r\n", __func__);
		return -EPERM;
	}
		device_destroy(pDev->class, MKDEV(pDev->major, 0));
		class_destroy(pDev->class);

	return 0;
}
