/*
 * System.h
 *
 *  Created on: Aug 12, 2020
 *      Author: martin
 */

#ifndef SYSTEM_H_
#define SYSTEM_H_

#include <stdint.h>

#define SYS_DEVICE_CLEANFIX_CUC				5

#define  SYS_DEVICE_NAME                  "CLEANFIX CUC"
#define  SYS_SW_VERSION                   "V0.00.032"
#define  SYS_LAST_CHANGE                  "2023-03-02"

#define VERSION_MAJOR   						0		// One Byte
#define VERSION_MINOR   						0		// One Byte
#define VERSION_BUILD   						30		// Two Bytes

#define  SYS_INFO_STRUCT_N_ENTRIES        3

#define  SYS_DEVICE_TYPE                  SYS_DEVICE_CLEANFIX_CUC    	//!< Device is a CleanFix CUC

typedef struct __attribute__((packed))
{
   uint8_t        nEntries;                                    //!< Number of Entries
   uint8_t        DeviceName_len;                              //!< Length of Name of the device
   char           DeviceName[sizeof(SYS_DEVICE_NAME)];         //!< Name of the device
   uint8_t        SW_Version_len;                              //!< Length of Software Version
   char           SW_Version[sizeof(SYS_SW_VERSION)];          //!< Software Version
   uint8_t        LastChange_len;                              //!< Length of Last Change
   char           LastChange[sizeof(SYS_LAST_CHANGE)];         //!< Last Change
} SysInfo_t,*pSysInfo_t;

SysInfo_t 	*GetSystemInfoStructPtr(void);
uint16_t 	GetSystemDeviceType(void);

#endif /* SYSTEM_H_ */
