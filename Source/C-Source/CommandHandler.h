/*
 * CommandHandler.h
 *
 *  Created on: Aug 12, 2020
 *      Author: martin
 */

#ifndef COMMANDHANDLER_H_
#define COMMANDHANDLER_H_

#include <stdint.h>

#include "common.h"
#include "uart.h"
#include "semphr.h"

/**
 * \defgroup CommandValues Command interpreter values
 * @{
 */

#define MUTEX_CmdHandler_WAIT       500                     //!< Wait time for Mutex Acquisition in Ticks

extern SemaphoreHandle_t   	xMutex_CmdHandler;
extern SemaphoreHandle_t   	xSemaBin_CmdPollDevices;

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

int                 	InitCommandHandler(void);
int                 	CommandHandler(void);
void                	SendCommandType(uint8_t com_type);
void                	commtask_systimer_isr_handler(void);
void                	UART_CmdIF_IRQHandler(UART_descriptor_t *UART_desc);
void 			   		UART_CmdIF_HandleUSB(uint8_t *buf,int count);
void 						SendPacketCMD(uint8_t *packet,int len);
void 						SendNAK(uint8_t command,uint8_t subcommand,int16_t error_code);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* COMMANDHANDLER_H_ */
