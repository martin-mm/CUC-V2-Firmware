/*
 * CommandMeasure.c
 *
 *  Created on: Dec 4, 2020
 *      Author: martin
 */

#include <string.h>

#include "common.h"
#include "uart.h"
#include "CommandDefs.h"
#include "CommandMeasure.h"
#include "CommandHandler.h"
#include "Board.h"
#include "System.h"
#include "Misc.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/*!
 ******************************************************************************
 *	Initializes the Measurement Command Handler
 * \return     1 if success, 0 else
 ******************************************************************************
*/
int InitMeasureCommandHandler(void)
{
   return 1;
}

/*!
 ******************************************************************************
 *	System Command: Calls the Measurement SUB-Command functions
 *	\param[in]	dev_nr      device number (index)
 *	\param[in]	command     parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int MeasureCommandHandler(int16_t dev_nr,uint8_t *command,int len)
{
   switch(*command)
   {
      default:
         return CMD_ERR_UNKNOWN_SUBCMD;    	// we should never get there!
   }
}

