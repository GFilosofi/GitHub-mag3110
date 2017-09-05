/******************** (C) COPYRIGHT 2012 Freescale Semiconductor, Inc. *************
 *
 * File Name		: mag_char.c
 * Authors		: Rick Zhang(rick.zhang@freescale.com)
 			  Rick is willing to be considered the contact and update points 
 			  for the driver
 * Version		: V.1.0.0
 * Date			: 2012/Mar/15
 * Description		: MAG3110   char driver interface implementation 
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
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/err.h>
#include <mach/hardware.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include "mag3110_regs.h"

#define MAG_FIFO_SIZE		(32)

/*! Function prototypes */
static int mag_open(struct inode * inode, struct file * file);
static ssize_t mag_read(struct file *file, char __user *buf, size_t size, loff_t *ppos);
static int mag_release(struct inode *inode, struct file *file);

typedef struct {
	struct semaphore sem;
	int threshold;
	int read_pos;
}Context_t, *pContext_t;


/*! FiFo to store magnetometer data */
MagData_t MagDataFifo[MAG_FIFO_SIZE];
static int writepos;

static DECLARE_WAIT_QUEUE_HEAD(MagDataQ);

/*! 
 * This structure is the file operations structure, which specifies what
 * callbacks functions the kernel should call when a user mode process
 * attempts to perform these operations on the device.
 */
const struct file_operations mag_fops = {
	.owner	 = THIS_MODULE,
	.open	 = mag_open,
	.release = mag_release,
	.read	 = mag_read,
};

/*!
* This method is used to update x, y, z vlaues to magnetometer FIFO. 
* pbuff: Pointer to the buffer containing magnetometer data.
* return 0  	: After updating values to FIFO  
*/
int UpdateMagFiFo(void * pbuff)
{
	pMagData_t pData = (pMagData_t)pbuff;
	/* Copy data to Fifo */
	MagDataFifo[writepos].x = pData->x;
	MagDataFifo[writepos].y = pData->y;
	MagDataFifo[writepos].z = pData->z;

	if(++writepos >= MAG_FIFO_SIZE)
		writepos = 0;

	wake_up(&MagDataQ);
	return 0;
}

/*!
* This method is used to get the position of read pointer. 
* pos : Pointer to the buffer containing magnetometer data.
* return    : Position of read pointer 
*/
static inline int GetAvailableData(int pos)
{
	int ret = 0;
	ret = (pos <= writepos)?(writepos - pos):(MAG_FIFO_SIZE - pos + writepos);
	return ret;
}

/*!
* Open call for magnetometer char driver. 
* inode : Pointer to the node to be opened.
* file  : Pointer to file structure.
* return 0    : After successful opening.  
*/
static int mag_open(struct inode * inode, struct file * file)
{
	pContext_t p = file->private_data;
	if (!p) {
		p = kmalloc(sizeof(*p), GFP_KERNEL);
		if (!p)
			return -ENOMEM;

		p->threshold = 4;
		p->read_pos = 0;
		sema_init(&p->sem, 1);
		file->private_data = p;
	}
	return 0;	
}

/*!
* read call for magnetometer char driver. 
* file  		: Pointer to file structure.
* buf 	 		: Pointer to user buffer in which data will be read.
* size  		: Size of data be read.
* ppos  		: Pointer to Offset in the file.
* return  no of bytes read 	: For successful read \
* return  -ENOMEM 		: file private data pointer is NULL	 
*/
static ssize_t mag_read(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
	int retval = 0;
	int nr = 0;
	pContext_t dev = file->private_data;
	int bytestocopy = 0;
	unsigned long bytescopied = 0;
	char __user *buff = buf;

	unsigned short hdr = 0xffff;
	
	if (!dev) 
	{
		return -ENOMEM;
	}

	/* Acquire semaphore to manage re-entrancy */
	if (down_interruptible(&dev->sem))
		return -ERESTARTSYS;

	/* Loop till data available has crossed the watermark */
	nr = GetAvailableData(dev->read_pos);
	while (nr < dev->threshold ) 
	{ 
		/* Wait on Magnetometer queue (MagQ) till condition GetAvailableData(dev->read_pos) >= dev->threshold gets satisfied */
		if (wait_event_interruptible(MagDataQ, 
					(GetAvailableData(dev->read_pos) >= dev->threshold)))
			return -ERESTARTSYS;
		nr = GetAvailableData(dev->read_pos);
	}

	bytescopied = copy_to_user(buff, &hdr, sizeof(unsigned short));
	retval += sizeof(unsigned short);
	buff += sizeof(unsigned short);
	bytescopied = copy_to_user(buff, &nr, sizeof(unsigned short));

	retval += sizeof(unsigned short);
	buff += sizeof(unsigned short);

	/* Loop here to copy bytes to user buffer */
	while(nr)
	{
		if(dev->read_pos + nr >= MAG_FIFO_SIZE)
		{
			bytestocopy = MAG_FIFO_SIZE - dev->read_pos ;
		}
		else
		{
			bytestocopy = nr;
		}

		/* Copy the required records to user buffer */
		bytescopied = copy_to_user(buff, &MagDataFifo[dev->read_pos], bytestocopy * sizeof(MagData_t));

		retval += bytestocopy * sizeof(MagData_t);
		buff += bytestocopy * sizeof(MagData_t);

		nr -= bytestocopy;

		/* Increment the read_pos */
		dev->read_pos += bytestocopy;
		if(dev->read_pos >= MAG_FIFO_SIZE)
			dev->read_pos -= MAG_FIFO_SIZE;
	}
	/* release the lock */
	up(&dev->sem); 
	/* Return the number of bytes written to buffer */
	return retval;
}

/*!
* Release call for magnetometer char driver. 
* inode : Pointer to the node to be opened.
* file  : Pointer to file structure.
* return 0    : After successful release of resources.  
*/
static int mag_release(struct inode *inode, struct file *file)
{
	pContext_t p = file->private_data;

	if (p) {
		//mutex_destroy(&p->lock);
		p = NULL;
	}

	return 0;
}
