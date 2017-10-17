/* 
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
/*
 * Definitions for helix als/ps sensor chip.
 */
#ifndef __helix_H__
#define __helix_H__

#include <linux/ioctl.h>

extern struct alsps_hw *helix_get_cust_alsps_hw(void);

//#define helix_CMM_ENABLE 		0X80
#define helix_CMM_ALS_CONFIG		0x25   //0X81
#define helix_CMM_PS_CONFIG 		0x15   //0X82
//#define helix_CMM_WTIME 		0X83

/*for interrup work mode support -- by liaoxl.lenovo 12.08.2011*/
#define helix_CMM_INT_LOW_THD_LOW   0x13  //0X88
#define helix_CMM_INT_LOW_THD_HIGH  0x12  //0X89
#define helix_CMM_INT_HIGH_THD_LOW  0x11  //0X8A
#define helix_CMM_INT_HIGH_THD_HIGH 0x10  //0X8B
#define helix_CMM_Persistence       0x14  //0X8C
#define helix_CMM_STATUS            0x40  //0X93
//#define TAOS_TRITON_CMD_REG           0X80
//#define TAOS_TRITON_CMD_SPL_FN        0x60

//#define helix_CMM_CONFIG 		0X8D
//#define helix_CMM_PPCOUNT 		0X8E
#define helix_CMM_CONTROL 		0x0F  // 0X8F

#define helix_CMM_PDATA_L 		0x42  //0X98
#define helix_CMM_PDATA_H 		0x41  //0X99
#define helix_CMM_C0DATA_L 	    0x44  //0X94
#define helix_CMM_C0DATA_H   	0x43  //0X95
#define helix_CMM_C1DATA_L 	    0x46  //0X96
#define helix_CMM_C1DATA_H 	    0x45  //0X97

#define helix_CMM_Reset        		0x01
#define helix_CMM_INT_Config        0x02
#define helix_CMM_LED_Frequency  	0x0D
#define helix_CMM_PS_Samle_Delay  	0x0E
#define helix_CMM_PS_Interval     	0x16
#define helix_CMM_PS_Control       	0x17
#define helix_CMM_ALS_Interval     	0x26
#define helix_CMM_ALS_Control     	0x27
#define MAX_LUX         100000  //30000


#define helix_SUCCESS						0
#define helix_ERR_I2C						-1
#define helix_ERR_STATUS					-3
#define helix_ERR_SETUP_FAILURE				-4
#define helix_ERR_GETGSENSORDATA			-5
#define helix_ERR_IDENTIFICATION			-6


#endif

