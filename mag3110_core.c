/******************** (C) COPYRIGHT 2012 Freescale Semiconductor, Inc. *************
 * Copyright (C) 2012 Cosmed, Ltd. <http://www.cosmed.com/>
 * Author: Gabriele Filosofi <gabrielef@cosmed.it>
 * Original Copyrights follows:
 *
 * File Name		: mag_core.c
 * Authors		: Rick Zhang(rick.zhang@freescale.com)
 			  Rick is willing to be considered the contact and update points 
 			  for the driver
 * Version		: V.1.0.0
 * Date			: 2012/Mar/15
 * Description		: MAG3110  driver module implementation for
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
#include <linux/delay.h>
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
//#include <linux/fs.h>
#ifdef CONFIG_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <cosmed/mag3110.h>

#include "mag3110_regs.h"
#include "mag3110_io.h"

#include	<linux/slab.h>

/*! macro define */
#define DEVICE_NAME		"mag"
/*! extern declarations */
extern const struct file_operations mag_fops;
extern int magInitializeInputInterface(struct mxc_mag_device_t *pDev);
extern int magDeInitializeInputInterface(struct mxc_mag_device_t *pDev);
extern int magReportEvent(struct mxc_mag_device_t *pDev, int type, void *buff);
extern int magInitializeSysfs(struct i2c_client *pClient);
extern int magDeInitializeSysfs(struct mxc_mag_device_t *pDev);

/*! forward declear */
static int mag3110_probe(struct i2c_client *client,
			 const struct i2c_device_id *id);
static int mag3110_remove(struct i2c_client *client);
static int mag3110_suspend(struct i2c_client *client, pm_message_t state);
static int mag3110_resume(struct i2c_client *client);
static int IdentifyChipset( struct i2c_client *client );

/*! definition */
static struct semaphore chip_ready;
struct task_struct 	*hIstThread;

static int ChipType = 0;
static struct mxc_mag3110_platform_data *plat_data;
static struct ChipInfo_t *gpChip = NULL;
static int done = 0; /*! variable used to check if mag3110 driver is removed or not */

/*! Variable to determine where the device is in poll mode or interrupted mode */
static int poll_mode = 0;

static const struct i2c_device_id mag3110_id[] = {
	{"mag3110", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, mag3110_id);

static struct i2c_driver i2c_mag3110_driver = {
	.driver = {
		   .name = "mag3110",
		   },
	.probe    = mag3110_probe,
	.remove   = mag3110_remove,
	.suspend  = mag3110_suspend,
	.resume   = mag3110_resume,
	.id_table = mag3110_id,
};

extern struct ChipInfo_t mag3110Chip;

struct ChipInfo_t * ChipTable[] = {
	&mag3110Chip,
};

static struct timer_list stall_timer; /*! Workaround to clear pending interrupt*/
static int IsSuspended = 0;

/*!
* Early suspend call for magnetometer. 
* h : Early suspend structure pointer.
* return None
*/
#ifdef CONFIG_EARLYSUSPEND
static void mag3110_early_suspend(struct early_suspend *h)
{
    if(gpChip->state == WAKE)
		gpChip->SetMode(0x00);
	IsSuspended = 1;
	// WORKAROUND
}
#endif
/*!
* Late resume call for magnetometer. 
* h : Early suspend structure pointer.
* return None
*/
#ifdef CONFIG_EARLYSUSPEND
static void mag3110_late_resume(struct early_suspend *h)
{
	 if(gpChip->state == WAKE)
		gpChip->SetMode(0x01);
	 IsSuspended = 0;
}
#endif
#ifdef CONFIG_EARLYSUSPEND
static struct early_suspend mag3110_early_suspend_desc;
#endif
#if 0
static struct early_suspend mag3110_early_suspend_desc = 
{
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
	.suspend = mag3110_early_suspend,
	.resume = mag3110_late_resume,
};
#endif

/*!
* Interrupt handler for Mag3110 chip. 
* irq 		: IRQ number.
* dev_id 	: Pointer to dev_id structure.
* return 1
*/
static irqreturn_t mag3110_interrupt(int irq, void *dev_id)
{
	if(plat_data->int1 == irq)
	{
		//disable_irq_nosync(irq);
		up(&chip_ready);
	}
	return IRQ_RETVAL(1);
}

/*!
* Workaround timer routine. 
* data.
* return None
*/
static void stall_timer_fn(unsigned long data)
{
	up(&chip_ready);
}

/*!
* Interrupt service thread implementation. 
* data : Pointer which is used to get device/chip data .
* return 0 	: After servicing thread.
*/
static int IntServiceThread(void *data)
{
	wait_queue_t wait;
	int ret = 0;
	char buff[1024];

	struct mxc_mag_device_t *pDev = (struct mxc_mag_device_t *)data;
	struct ChipInfo_t *pChip = pDev->pChip;
	
	init_waitqueue_entry(&wait, current);
	
	// WORKAROUND
	mod_timer(&stall_timer, jiffies + (HZ));
	while(!done)
	{

		do {
			/* down_interruptible - acquire the semaphore unless interrupted
			 * If the sleep is interrupted by a signal, this function will return -EINTR.
			 * If the semaphore is successfully acquired, this function returns 0.
			 */
			ret = down_interruptible(&chip_ready);		
		} while (ret == -EINTR);

		if(IsSuspended)
			continue;

		pChip->Read(MAG_DATA, (void *)buff);
		
		UpdateMagFiFo(buff);

		if(!IsSuspended)
			magReportEvent(pDev, MAG_DATA, buff);
		
		if (!IsSuspended) {
			if (!poll_mode)	
			{
				mod_timer(&stall_timer, jiffies + (HZ));
				//enable_irq(plat_data->int1);
			}
			else 
				mod_timer(&stall_timer, jiffies + pChip->poll_time);
		}
	}
	return 0;
}

/*!
* Identify the chip connected on bus and associate client driver for the chipset. 
* client : Pointer of i2c client driver.
* return -1 	  : If chip not identified.
* return 0 	  : If chip identified.
*/

static struct semaphore temp;	
static int IdentifyChipset( struct i2c_client *client )
{
	int retVal = 0;
	int ChipIdentified = 0;
	int i = 10;

	{
	    retVal = down_timeout(&temp, jiffies + (5 * HZ));

		i = 10;
		retVal = i2c_smbus_read_byte_data(client, REG3110_WHO_AM_I);
		switch(retVal)
		{
			case ID_MAG3110:
				{
					printk("%s:: Found MAG3110 chipset with chip ID 0x%02x\r\n", __func__, retVal);
					ChipIdentified = 1;
				}
				break;

			default:
				{
					printk("%s:: Not a valid MAG3110 chipset. Chip ID 0x%02x\r\n", __func__, retVal);
					ChipIdentified = 0;
				}
				break;
		}

		if(!ChipIdentified)
		{
			return -1;
		}
	}
	return retVal;
}

/*!
* Set default values of chip structure. 
* pChip 	: Pointer to chip structure.
* return -1 		: If chip not identified.
* return -ENOMEM 	: If chip pointer is NULL.
* return -EINVAL 	: If chip id is not matching.
* return 0 		: After setting default values.
*/
static int SetDefaultVal(struct ChipInfo_t *pChip)
{
	int ret = 0;

	if(pChip == NULL)
	{
		printk("%s: NULL pointer\r\n", __func__);
		return -ENOMEM;
	}

	switch(pChip->ChipId)
	{
		case ID_MAG3110:
			{
				pChip->odr = 0x06;
				pChip->oversampling = 0; 	//Normal
				pChip->resolution = 0; 		// 16-bit
				pChip->range = 0; 		// -1 to +1
				pChip->xCalOffset = 0;
				pChip->yCalOffset = 0;
				pChip->zCalOffset = 0;
				pChip->OutputMode = SCALED;				

			}
			break;

		default:
			{
				printk("Invalid chip \n");
				return -EINVAL;
			}
	}
	return ret;
}

/*!
* Probe function for magnetometer. 
* client 	: Pointer to i2c client.
* id 		: Pointer to i2c device id.
* return -ENODEV 	: If platform data not found.
* return -ENOMEM 	: If failed to allocate memory for device structure.
* return -EIO 	: If unable to identify chipset.
* return 0 		: After successful probing.
*/
static int mag3110_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int ret = 0, i = 0;
	struct ChipInfo_t *pChip = NULL;
	struct mxc_mag_device_t *pDev = NULL;

	plat_data =
	    (struct mxc_mag3110_platform_data *)client->dev.platform_data;
	if (plat_data == NULL) {
		dev_err(&client->dev, "lack of platform data!\n");
		return -ENODEV;
	}

	pDev = kzalloc(sizeof(struct mxc_mag_device_t), GFP_KERNEL);
	
	if (!pDev) {
		return -ENOMEM;
	}

	 printk(KERN_INFO"\r\nProbing Module: %s %s\r\n", MODULE_NAME, DRIVER_VERSION);
#if 0 /* gf30-07-2012 */
	 printk(KERN_INFO "Android Release: %s\r\n", ANDROID_RELEASE);
#endif
	 printk(KERN_INFO "Build Date: %s [%s]\r\n\r\n", __DATE__, __TIME__);

	sema_init(&temp, 0);
	i2c_set_clientdata(client, pDev);

	/*bind the right device to the driver */
	ret = IdentifyChipset( client );
	if(ret < 0)
	{
		 printk(KERN_INFO "%s:: Unable to identify device.\r\n", __func__);
		return -EIO;
	}

	/* Associate chip layer */
	for(i = 0; i < sizeof(ChipTable)/sizeof(ChipTable[0]); i++ )	
	{
		if(ChipTable[i]->ChipId == ret)
		{
			pChip = ChipTable[i];
			pChip->ChipId = ret;
			pChip->client = client;
			pChip->ChipType = ChipType;
			break;
		}
	}

	if(i >= (sizeof(ChipTable)/sizeof(ChipTable[0])))
	{
		 printk(KERN_INFO "Chipset not supported by MAG driver\r\n");
		return -ENOMEM;
	}

	gpChip = pChip;
	SetDefaultVal(pChip);

	/* Inialize default event codes */
	/*! @todo : event type*/
	pDev->data_event_type = 0x25;

	/* Initialize chipset */
	pChip->Init(pChip);

	pDev->pChip = pChip;
	pDev->version = 1;
	
	/* Register character device */
	pDev->major = register_chrdev(0,"mag",&mag_fops);
	if(ret < 0)
	{
		 printk(KERN_INFO "%s:: Unable to register device\r\n", __func__);
		goto error_disable_power;
	}

	strcpy(pDev->devname, "mag");

	/*! Initialize input layer */
	magInitializeInputInterface(pDev);	

	/*! Create sysfs entries */
	magInitializeSysfs(client);
	
	/*! Initialize chip ready semaphore */
	sema_init(&chip_ready, 0);


	//! WORKAROUND
	setup_timer(&stall_timer, stall_timer_fn, 0);

	/*! Start IST */
	hIstThread = kthread_run(IntServiceThread, pDev, "mag3110_ist");
	if (IS_ERR(hIstThread)) 
	{
		 printk(KERN_INFO "Error creating mag3110_ist.\n");
		goto error_free_irq1;
	}

	if (plat_data->int1 > 0)
	{
		/* interrupt base */
		/* when to register interrupt is to be considered later */
		 printk(KERN_INFO "Configuring IRQ ==> %d\r\n", plat_data->int1);
		irq_set_irq_type(plat_data->int1, IRQF_TRIGGER_RISING);
		/* register interrupt handle */
		ret = request_irq(plat_data->int1, mag3110_interrupt,
				  IRQF_TRIGGER_RISING, DEVICE_NAME, pDev);

		if (ret < 0) {
			dev_err(&client->dev, "request_irq(%d) returned error %d\n",
				plat_data->int1, ret);
			goto error_disable_power;
		}
	}
	else 
	{
		/* poll base */	
		poll_mode = 1;
	}
	/* Configure suspend/resume sources */
#ifdef CONFIG_EARLYSUSPEND
	mag3110_early_suspend_desc.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	mag3110_early_suspend_desc.suspend = mag3110_early_suspend;
	mag3110_early_suspend_desc.resume = mag3110_late_resume;
	// Register early_suspend and late_wakeup handlers
	register_early_suspend(&mag3110_early_suspend_desc);	
#endif

	/* Set mode to normal */
	pChip->SetMode(0x01);

	dev_info(&client->dev, "mag3110 device is probed successfully.\n");

	return 0;

	/*error handle */
error_free_irq1:
	free_irq(plat_data->int1, plat_data);
error_disable_power:
	
	return ret;
}

/*!
* Remove function for magnetometer. 
* client 	: Pointer to i2c client.
* return 0 		: After successful removal.
*/
static int mag3110_remove(struct i2c_client *client)
{
	struct mxc_mag_device_t *pDev = i2c_get_clientdata(client);
	printk(KERN_INFO "%s:: Enter\n", __func__);
	
	/* Remove sysfs entries */
	magDeInitializeSysfs(pDev);

	/* unegister character device */
	printk(KERN_INFO "%s:: Unregister char interface\n", __func__);
	unregister_chrdev(pDev->major, pDev->devname);

	/* DeInitialize input layer */
	printk(KERN_INFO "%s:: Unregister input interface....\n", __func__);
	magDeInitializeInputInterface(pDev);
	
	/* DeInit chipset */
	printk(KERN_INFO "%s:: DeInit chipset\n", __func__);
	if(plat_data->gpio_pin_put)
		plat_data->gpio_pin_put();
	
	/* DeRegister interrupt */
	printk(KERN_INFO "%s:: Freeing IRQs\n", __func__);
	free_irq(plat_data->int1, pDev);

	/* Stop IST */
	printk(KERN_INFO "%s:: Stopping thread....\n", __func__);
	done = 1;
	up(&chip_ready);
	kthread_stop(hIstThread);

	/* Release device */
	kfree(pDev);
	pDev = NULL;
	return 0;
}

/*!
* Suspend function for magnetometer. 
* client : Pointer to i2c client.
* state  : Structure variable.
* return 0     : After suspending device.
*/
static int mag3110_suspend(struct i2c_client *client, pm_message_t state)
{
	return 0;
}

/*!
* Suspend function for magnetometer. 
* client : Pointer to i2c client.
* return 0 	  : After resuming device.
*/
static int mag3110_resume(struct i2c_client *client)
{
	return 0;
}

/*!
* Init call for magnetometer module. 
* return Driver addition result.
*/
static int __init init_mag3110(void)
{
	/*register driver */
	printk(KERN_INFO "add mag i2c driver\n");
	return i2c_add_driver(&i2c_mag3110_driver);
}

/*!
* Exit call for magnetometer module. 
* return Driver deletion result.
*/
static void __exit exit_mag3110(void)
{
	printk(KERN_INFO "del mag i2c driver.\n");
#ifdef CONFIG_EARLYSUSPEND
	// Register early_suspend and late_wakeup handlers
	unregister_early_suspend(&mag3110_early_suspend_desc);	
#endif
	return i2c_del_driver(&i2c_mag3110_driver);
}

module_init(init_mag3110);
module_exit(exit_mag3110);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MAG3110 sensor driver");
MODULE_LICENSE("GPL");
