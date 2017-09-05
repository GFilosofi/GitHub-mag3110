/******************** (C) COPYRIGHT 2012 Freescale Semiconductor, Inc. *************
 *
 * File Name		: mag_regs.h
 * Authors		: Rick Zhang(rick.zhang@freescale.com)
 			  Rick is willing to be considered the contact and update points 
 			  for the driver
 * Version		: V.1.0.0
 * Date			: 2012/Mar/15
 * Description		: MAG3110  declarations and defines required to driver
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
#ifndef __MAG_REGS_H__
#define __MAG_REGS_H__


#define ANDROID_RELEASE	"10.2"
#define MODULE_NAME	"mmg3110"

#define DRIVER_VERSION	"Ver. 1.0"
#define VENDOR_NAME	"Freescale Semiconductors" 

/* Macros for fixed point arithematic */
#define FP_SHIFT 3     // shifts to produce a fixed-point number
#define FP_SCALE 1000  // scaling factor
#define PRECISION		1000

#define INT_TO_FIXP(n) ((FIXPOINT)((n * FP_SCALE)))
#define FLOAT_TO_FIXP(n) ((FIXPOINT)((float)n * FP_SCALE))
#define FIXP_INT_PART(n) (n / FP_SCALE)
#define FIXP_DEC_PART(n) (n % FP_SCALE)

typedef unsigned int FIXPOINT;

//#define REG(x)	(ChipType == MMA845X ? REG845X_##x : REG8450_##x)
#define REG(x)	(REG845X_##x)

/* Data types */
#define MAG_DATA	0x01
#define FIFO_DATA	0x02

/* Interrupt source macros */
#define SRC_ASLP	_BIT(7)		// Auto-SLEEP/WAKE
#define SRC_FIFO	_BIT(6)		// FIFO interrupt 
#define SRC_TRANS	_BIT(5)		// Transient interrupt
#define SRC_LNDPRT 	_BIT(4)		// Landscape/portrait (orientation)
#define SRC_PULSE 	_BIT(3)		// Pulse
#define SRC_FF_MT 	_BIT(2)		// Freefall/motion interrupt
#define SRC_FF_MT_2	_BIT(1)		// Freefall/motion 2 interrupt
#define SRC_DRDY 	_BIT(0)		// Dataready interrupt

/* Command codes for registers */
enum {
	CMD_ODR = 0x00,
	CMD_SAMPLE,
	CMD_RANGE,
	CMD_MODE,
	CMD_RESOLUTION,
	CMD_TRIGGER,
	CMD_DATA_MODE,
	CMD_SENSOR_RST,
};


/* sysfs entries table */
struct SysfsInfo_t {
	char 	* grpName;			// sysfs group name;
						// NULL is treated as ungrouped
	struct 	device_attribute *AttrEntry;	// Pointer to attribute table
	int 	TotalEntries;			// Number of attributes
	int 	Instance;			// No. of instances for group
};

struct ChipInfo_t{
	int id;
	int ChipId;
	int ChipType;
	char name[16];
	int devtype;

	/* Parameters */
	unsigned int maxres;		// Maximum resolution
	unsigned int maxrange;	
	int enablefifo;
	int fifo_threshold;
	int min_fifo_th;
	int max_fifo_th;

	FIXPOINT * pOdrList;
	char 	 ** pSamplinglist;
	int * pSupportedResolution;
	int * pSupportedRange;
	FIXPOINT * pCalibration_offsetList;
	FIXPOINT * pZ_lock_angle_thresholdList;
	int * pBack_front_trip_angle_thresholdList;
	int * pTrip_angle_thresholdList;

	/* current Values*/
	int state;
	int odr;
	int oversampling;
	int resolution;
	int range;
	int value;
	int calibration_offset;
	int xCalOffset;
	int yCalOffset;
	int zCalOffset;
	int interrupt_set;
	int OutputMode;
	int poll_time;
	
	struct i2c_client * client;
	struct SysfsInfo_t *pSysfsInfo;
	int SysfsInfoSize;

	/* Function pointers */
	int (*Init)( struct ChipInfo_t * );
	int (*Read)( int, void * );
	int (*SetMode)( int );
	int (*GetMode)( void );
	int (*EnableInt)( int );
	int (*DisableInt)( int );
	int (*GetIntSrc)( void );
	int (*SetRegVal)(int cmd, int val);
	int (*SetCalOffset)(int val[]);
	void (*GetDieTemp)(char *temp);
};

typedef struct {
	short x;
	short y;
	short z;
}MagData_t, *pMagData_t;

struct mxc_mag_device_t{
	struct ChipInfo_t *pChip;

	/* Character (FIFO) layer members */
	int major;
	char devname[32];
	int version;

	/* Control information */
	int data_event_type;

	/* Kernel object */
	struct kobject *kobj;

	/* Input layer members */
	struct input_dev *inp1;
	struct input_dev *inp2;

	struct class *class;
	struct device *sys_device;   	// Common entries

};

/* Function prototypes */
int UpdateMagFiFo(void * buff);

#endif // __MXC_REGS_H__
