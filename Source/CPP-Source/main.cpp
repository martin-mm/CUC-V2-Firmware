/*
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    MK22FN1M0Axxx12_Project.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "board-DigIO.h"
#include "board-Ana.h"
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
//#include "MK22FA12.h"
#include "virtual_com.h"
#include "CommandHandler.h"

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"

#include "base.h"
#include "UCDevice.h"
#include "IO.h"
#include "BoardMgr.h"
#include "EEPROM.h"

#include "cmsis_os2.h"

#define WATCHDOG_UPDATE_PERIOD   	1    // in uc cycles

TaskHandle_t            		TaskComm;

static const char					osSysTaskName[] = "osSYS-TASK";

uint32_t								error_flags = 0;
osThreadId_t						sysThread = NULL;

traceString							dbgChannel;

#if USE_STACK_PROTECTION != 0
void *__stack_chk_guard = (void *)0xA5432198;
#endif

/*!
 ******************************************************************************
 *	Print function encapsulation for Heap Status printf()
 * \param[in]     pvParameters   Task parameters
 ******************************************************************************
*/
int hprintf(FILE *stream,const char * format,...)
{
va_list 		args;

	va_start(args, format);
	vdbgprintf(format,args);
	return 1;
}	

/*!
 ******************************************************************************
 *	Checks the Heap Status
 ******************************************************************************
*/
void CheckHeapStatus(void)
{
	dbgprintf("****************** Begin Of Heap Status ******************\n");
	__heapstats((__heapprt)hprintf,stdout);
	dbgprintf("RTOS actual heap size: %d\n",configTOTAL_HEAP_SIZE - xPortGetFreeHeapSize()); 
	dbgprintf("RTOS max. heap size  : %d\n",configTOTAL_HEAP_SIZE - xPortGetMinimumEverFreeHeapSize()); 
	dbgprintf("******************  End Of Heap Status  ******************\n");
}	

/*!
 ******************************************************************************
 *	Gets information about the Stack and the Heap position and length
 ******************************************************************************
*/
void GetHeapAndStackInfo(void)
{
uint32_t		heap_base,heap_size,stack_base,stack_size;
	
	dbgprintf("************ Begin Of Stack and Heap Info ****************\n");
	if (BOARD_GetStackAndHeapInfo(false,&stack_base,&stack_size,&heap_base,&heap_size))
	{
		dbgprintf("NON-ZI regions:\n");
		dbgprintf("   Heap base address : 0x%08X, Heap size : %d bytes\n",heap_base,heap_size); 
		dbgprintf("   Stack base address: 0x%08X, Stack size: %d bytes\n",stack_base,stack_size); 
	}
	if (BOARD_GetStackAndHeapInfo(true,&stack_base,&stack_size,&heap_base,&heap_size))
	{
		dbgprintf("ZI regions:\n");
		dbgprintf("   Heap base address : 0x%08X, Heap size : %d bytes\n",heap_base,heap_size); 
		dbgprintf("   Stack base address: 0x%08X, Stack size: %d bytes\n",stack_base,stack_size); 
	}
	dbgprintf("************  End Of Stack and Heap Info  ****************\n");
}	

// ----------------------------------------------------------------------------
//! \brief Main OS task
__NO_RETURN void osSysInit(void * argument)
{
	BoardMgr *pMgr = new BoardMgr();
	SystemTime::Init(1000);

//   dbgprintf("Switch Relay 1 on ...");
//	if (ControlRelay1(true,2000,false))
//		dbgprintf(" succeeded\n");
//	else
//		dbgprintf(" failed\n");
//   dbgprintf("Switch Relay 2 on ...");
//	if (ControlRelay2(true,false))
//		dbgprintf(" succeeded\n");
//	else
//		dbgprintf(" failed\n");

	// Manage watchdog
	
	CheckHeapStatus();

   dbgprintf("\nEntering System Init Task ...\n");
	while (1)
	{
		pMgr->HandleWatchdog();
		pMgr->UpdateLEDs();
		//update the SystemTime 100 times faster than the HandleWatchdog
//		for(uint8_t i=0; i<100; i++)
//		{
			osDelay(WATCHDOG_UPDATE_PERIOD * 100);
			SystemTime::Update();
//		}
	}
}

/*!
 ******************************************************************************
 *	This is the Communication Task
 * \param[in]     pvParameters   Task parameters
 ******************************************************************************
*/
void CommunicationTask(void *pvParameters)
{
uint32_t            ulNotify;
static bool			  start = true;

   for( ;; )
   {
   	if (start)
   	{
   	    dbgprintf("Starting Communication Task\n");
   	    start = false;
   	}
//	   cmd_SUB_SYS_GET_CLOCKS();
      if (xTaskNotifyWait(0x00,UINT32_MAX,&ulNotify,(TickType_t)500) == pdTRUE)
      {
         if ((ulNotify & (1 << 0)) != 0)
         {
           CommandHandler();
         }
         if ((ulNotify & (1 << 1)) != 0)
         {
            CommandHandler();
         }
         if ((ulNotify & (1 << 2)) != 0)
         {
				HandleRelay1();
         	// CheckVB1_Voltage();
         }
      }
   }
}

/*
 * @brief   Application entry point.
 */
int main(void) {
osThreadAttr_t 	thread_attr;

  	/* Init board hardware. */
   BOARD_InitBootPins();
	BOARD_InitBootClocks();
   BOARD_InitBootPeripherals();
   BOARD_Init();

#if (configUSE_TRACE_FACILITY == 1)
#if TRACEALYZER != 0
   vTraceEnable(TRC_START);         // Percepio Tracealyzer
   dbgChannel = xTraceRegisterString("CUC");
#endif
#endif
	
	portENABLE_INTERRUPTS();
//	__enable_irq();

	dbgprintf("Hello, I am the new CUC ...\n\n");
   dbgprintf("Internal Clock Frequencies:\n");
   dbgprintf("   Core Clock Frequency         : %lu Hz\n",(unsigned long)CLOCK_GetFreq(kCLOCK_CoreSysClk));
   dbgprintf("   Bus Clock Frequency          : %lu Hz\n",(unsigned long)CLOCK_GetFreq(kCLOCK_BusClk));
   dbgprintf("   Flash Clock Frequency        : %lu Hz\n",(unsigned long)CLOCK_GetFreq(kCLOCK_FlashClk));
   dbgprintf("   External Reference Frequency : %lu Hz\n",(unsigned long)g_xtal0Freq);
   dbgprintf("   USB Clock Frequency          : %lu Hz\n\n",(unsigned long)CLOCK_GetFreq(kCLOCK_CoreSysClk) / 2);

	SleepBM(2000);

	BOARD_EnaADC_Timer(true);

	BOARD_FTM3_enable_isr();
	dbgprintf("Calibrating ADC channels ...\n");
	if (BOARD_calib_ADC_channel_offset(ADC_LIFT_SUCT_CUR))
		dbgprintf("   LIFT_SUCT_CUR offset: %d\n",BOARD_getOffsetCalibValue(ADC_LIFT_SUCT_CUR));
	else
		dbgprintf("   LIFT_SUCT_CUR offset: failed\n");
	if (BOARD_calib_ADC_channel_offset(ADC_LIFT_BR_CUR))
		dbgprintf("   LIFT_BRUSH_CUR offset: %d\n",BOARD_getOffsetCalibValue(ADC_LIFT_BR_CUR));
	else
		dbgprintf("   LIFT_BRUSH_CUR offset: failed\n");
	if (BOARD_calib_ADC_channel_offset(ADC_PUMP_CUR))
		dbgprintf("   ADC_PUMP_CUR offset: %d\n",BOARD_getOffsetCalibValue(ADC_PUMP_CUR));
	else
		dbgprintf("   ADC_PUMP_CURR offset: failed\n");
	if (BOARD_calib_ADC_channel_offset(ADC_BRUSH_CUR))
		dbgprintf("   ADC_BRUSH_CUR offset: %d\n",BOARD_getOffsetCalibValue(ADC_BRUSH_CUR));
	else
		dbgprintf("   ADC_BRUSH_CUR offset: failed\n");
	if (BOARD_calib_ADC_channel_offset(ADC_SUCT_CUR))
		dbgprintf("   ADC_SUCT_CUR offset: %d\n",BOARD_getOffsetCalibValue(ADC_SUCT_CUR));
	else
		dbgprintf("   ADC_SUCT_CUR offset: failed\n");
	dbgprintf("... done\n\n");
	BOARD_FTM3_disable_isr();
		

//	BOARD_EnableLED_Blink(eLED0,true,500);
//	BOARD_EnableLED_Blink(eLED1,true,750);
	BOARD_EnableLED_Blink(eLED2,true,500);
//	BOARD_EnableLED_Blink(eCHARGE_LED_G,true,1500);
//	BOARD_EnableLED_Blink(eCHARGE_LED_Y,true,2000);
//	BOARD_EnableLED_Blink(eCHARGE_LED_R,true,4000);
	
	CheckHeapStatus();

	dbgprintf("Initializing USB stack ...");
	if (usb_main_init())
		dbgprintf(" SUCCESS\n\n");
	else
		dbgprintf(" FAILED\n\n");

	dbgprintf("CMSIS2-RTOS:\n");
   dbgprintf("Creating Communication Task ... ");
   if (xTaskCreate(CommunicationTask, "COM-TASK", CommTask_STACK_SIZE, NULL, CommTask_PRIORITY, &TaskComm) != pdPASS)
   {
		dbgprintf("FAILED\n");
   }
   else
   {
		dbgprintf("SUCCESS\n");
   }
   dbgprintf("Messages will be send to the Console (UART0).\n");	
	UCDevice::SetUCFreq(96*1000*1000);
	osKernelInitialize();
	memset(&thread_attr,0,sizeof(thread_attr));
	thread_attr.name = osSysTaskName;
	thread_attr.stack_size = 2048;
   dbgprintf("Creating System Init Task ...\n");

	sysThread = osThreadNew(osSysInit,nullptr,&thread_attr);
   dbgprintf("Enabling the FAN ...\n\n");
	BOARD_SetFAN(true);
	GetHeapAndStackInfo();
	portENABLE_INTERRUPTS();
	SleepBM(1000);
	
	osKernelStart();
   for(;;) {
   }
//   return 0 ;
}

extern "C" void vApplicationMallocFailedHook(void)
{
   dbgprintf("ERROR RTOS Malloc Failed\n");	
	error_flags |= (1 << 0);
}

extern "C" void vApplicationStackOverflowHook(TaskHandle_t xTask,char *pcTaskName)
{
   dbgprintf("ERROR RTOS Stack Overflow\n");	
	error_flags |= (1 << 1);
}

#if USE_STACK_PROTECTION != 0
extern "C" void __stack_chk_fail(void)
{
   dbgprintf("ERROR Stack violation\n");	
	error_flags |= (1 << 1);
}
#endif
