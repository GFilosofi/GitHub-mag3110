/******************** (C) COPYRIGHT 2012 Freescale Semiconductor, Inc. *************
 *
 * File Name		: mag_input.c
 * Authors		: Rick Zhang(rick.zhang@freescale.com)
 			  Rick is willing to be considered the contact and update points 
 			  for the driver
 * Version		: V.1.0.0
 * Date			: 2012/Mar/15
 * Description		: MAG3110  input interface implementation
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
#include "mag3110_regs.h"
#include <mach/hardware.h>

/*!
* This method is used to initialize input interface. 
* pDev 	: Pointer to device structure.
* return  -EPERM 	: If device pointer is NULL 
* return  -ENOMEM 	: If memory allocation to the device fails 
* return  0 		: If device is successfully registered.	
*/
int magInitializeInputInterface(struct mxc_mag_device_t *pDev)
{
	int RetVal = 0;
	
	if(pDev == NULL)
	{
		printk("%s: pDev => NULL pointer\r\n", __func__);
		return -EPERM;
	}

	pDev->inp1 = input_allocate_device();

	if (!pDev->inp1) {
		RetVal = -ENOMEM;
		printk(KERN_ERR
		       "%s: Failed to allocate input device-1\n", __func__);
		return RetVal;
	}

	set_bit(EV_ABS, pDev->inp1->evbit);	// Magnetometer readings

	/* x-axis magnetization */
	input_set_abs_params(pDev->inp1, ABS_X, -32768, 32767, 10, 0);
	/* y-axis magnetization */
	input_set_abs_params(pDev->inp1, ABS_Y, -32768, 32767, 10, 0);
	/* z-axis magnetization */
	input_set_abs_params(pDev->inp1, ABS_Z, -32768, 32767, 10, 0);

	pDev->inp1->name = "mag3110";
	RetVal = input_register_device(pDev->inp1);
	if (RetVal) {
		printk(KERN_ERR
		       "%s: Unable to register input device: %s\n",
		       __func__, pDev->inp1->name);
		return RetVal;
	}

	return RetVal;	
}

/*!
* This method is used to deinitialize input interface. 
* pDev 	: Pointer to device structure.
* return  -EPERM 	: If device pointer is NULL. 
* return  0 		: If device is successfully deinitialized.	
*/
int magDeInitializeInputInterface(struct mxc_mag_device_t *pDev)
{
	if(pDev == NULL)
	{
		printk("%s: pDev => NULL pointer\r\n", __func__);
		return -EPERM;
	}

	if(pDev->inp1)
	{
		input_unregister_device(pDev->inp1);
		input_free_device(pDev->inp1);
	}
	return 0;
}

/*!
* This method is used to report event to upper layer. 
* pDev 	: Pointer to device structure.
* type 	: Input type.
* buff 	: Pointer to buffer containing input data.
* return 0 		: If device is successfully deinitialized.	
* return -ENOMEM 	: If device pointer is NULL.	
*/
int magReportEvent(struct mxc_mag_device_t *pDev, int type, void *buff)
{
	if(pDev == NULL)
		return -ENOMEM;

	switch(type)
	{
		case MAG_DATA:
			{
				pMagData_t pData = (pMagData_t)buff;

				if(pDev->inp1 && pData)
				{
					input_report_abs(pDev->inp1, ABS_X, pData->x);
					input_report_abs(pDev->inp1, ABS_Y, pData->y);
					input_report_abs(pDev->inp1, ABS_Z, pData->z);

					input_sync(pDev->inp1);
#ifdef DEBUG				
					printk("X: [%04x] Y:[%04x] Z: [%04x]\r\n", pData->x, pData->y, pData->z);
#endif
				}
			}
			break;

		default:
			printk("%s:: Unhandled input type\r\n", __func__);
			break;
	}

	return 0;
}
