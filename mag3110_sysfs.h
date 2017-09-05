/******************** (C) COPYRIGHT 2012 Freescale Semiconductor, Inc. *************
 *
 * File Name		: mag_sysfs.h
 * Authors		: Rick Zhang(rick.zhang@freescale.com)
 			  Rick is willing to be considered the contact and update points 
 			  for the driver
 * Version		: V.1.0.0
 * Date			: 2012/Mar/15
 * Description		: MAG3110  declarations and defines required magnetometer sysfs entries
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
#include <linux/device.h>
#include <linux/i2c.h>
#include "mag3110_regs.h"
#include <mach/hardware.h>
#include "mag3110_io.h"

#define DECLARE_ATTR(_name, _mode, _show, _store)		\
{                                                               \
	.attr   = { .name = __stringify(_name), .mode = _mode,	\
		    .owner = THIS_MODULE },  			\
	.show   = mag_##_show,                                        \
	.store  = mag_##_store,                                       \
}

#define mag_NULL	NULL

extern ssize_t mag_name_show(struct device *dev,
		struct device_attribute *attr, char *buf);

extern ssize_t mag_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf);

extern ssize_t mag_devid_show(struct device *dev,
		struct device_attribute *attr, char *buf);

extern ssize_t mag_version_show(struct device *dev,
		struct device_attribute *attr, char *buf);

extern ssize_t mag_type_show(struct device *dev,
		struct device_attribute *attr, char *buf);

extern ssize_t mag_max_range_show(struct device *dev,
		struct device_attribute *attr, char *buf);

extern ssize_t mag_max_res_show(struct device *dev,
		struct device_attribute *attr, char *buf);

extern ssize_t mag_nominal_power_show(struct device *dev,
		struct device_attribute *attr, char *buf);

extern ssize_t mag_operation_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf);

extern ssize_t mag_operation_mode_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count);

extern ssize_t mag_odr_show(struct device *dev,
		struct device_attribute *attr, char *buf);

extern ssize_t mag_odr_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count);

extern ssize_t mag_supported_odr_show(struct device *dev,
		struct device_attribute *attr, char *buf);

extern ssize_t mag_oversampling_show(struct device *dev,
		struct device_attribute *attr, char *buf);

extern ssize_t mag_oversampling_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count);

extern ssize_t mag_oversampling_values_show(struct device *dev,
		struct device_attribute *attr, char *buf);

extern ssize_t mag_resolutions_show(struct device *dev,
		struct device_attribute *attr, char *buf);

extern ssize_t mag_resolution_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count);

extern ssize_t mag_resolution_show(struct device *dev,
		struct device_attribute *attr, char *buf);

extern ssize_t mag_range_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count);

extern ssize_t mag_range_show(struct device *dev,
		struct device_attribute *attr, char *buf);

extern ssize_t mag_range_low_show(struct device *dev,
		struct device_attribute *attr, char *buf);

extern ssize_t mag_range_high_show(struct device *dev,
		struct device_attribute *attr, char *buf);

extern ssize_t mag_precision_show(struct device *dev,
		struct device_attribute *attr, char *buf);

extern ssize_t mag_sample_rate_show(struct device *dev,
		struct device_attribute *attr, char *buf);

extern ssize_t mag_value_show(struct device *dev,
		struct device_attribute *attr, char *buf);

extern ssize_t mag_calibration_offset_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count);

extern ssize_t mag_calibration_offset_show(struct device *dev,
		struct device_attribute *attr, char *buf);

extern ssize_t mag_calibration_offset_show(struct device *dev,
		struct device_attribute *attr, char *buf);

ssize_t mag_trigger_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count);

ssize_t mag_output_mode_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count);

ssize_t mag_output_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf);

extern ssize_t mag_poll_time_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count);

extern ssize_t mag_poll_time_show(struct device *dev,
		struct device_attribute *attr, char *buf);

ssize_t mag_die_temp_show(struct device *dev,
		struct device_attribute *attr, char *buf);
