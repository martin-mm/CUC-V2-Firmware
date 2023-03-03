/*
 * CmdDevice.h
 *
 *  Created on: Sep 12, 2021
 *      Author: martin
 */

#ifndef CMDDEVICE_H_
#define CMDDEVICE_H_

#include <stdint.h>

int InitDeviceCommandHandler(void);
int DeviceCommandHandler(int16_t dev_nr,uint8_t *command,int len);

#endif /* CMDDEVICE_H_ */
