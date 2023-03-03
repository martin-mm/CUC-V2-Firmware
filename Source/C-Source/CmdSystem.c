/*
 * CmdSystem.c
 *
 *  Created on: Aug 12, 2020
 *      Author: martin
 */

#include <string.h>

#include "common.h"
#include "uart.h"
#include "CommandDefs.h"
#include "CmdSystem.h"
#include "CommandHandler.h"
#include "board.h"
#include "board-DigIO.h"
#include "System.h"
#include "Misc.h"
#include "CAN.h"
#include "TMP100.h"
#include "CleaningUnitMgr.h"
#include "LiftDevice.h"
#include "BoardMgr.h"
#include "SafetyMgr.h"
#include "CANNode.h"
#include "I2C.h"
#include "EEPROM.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "Task_CMSIS2.h"
#include "cmsis_os2.h"

extern osThreadId_t						sysThread;


/*!
 ******************************************************************************
 *	Initializes the System Command Handler
 * \return     1 if success, 0 else
 ******************************************************************************
*/
int InitSystemCommandHandler(void)
{
   return 1;
}

/*!
 ******************************************************************************
 *	System Subcommand: Send Ping
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
static int cmd_SUB_SYS_REQUEST_PING(uint8_t *data,int len)
{
uint8_t     buf[10];
int         i;

   if (len < 4)
      return(CMD_ERR_INVALID_LENGTH);
   for (i=0;i<4;i++)
      buf[i+6] = data[i] ^ 0xFF;
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_REQUEST_PING,CMD_RX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Get the value of an ADC channel
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
static int cmd_SUB_SYS_GET_ADC_VALUE(uint8_t *data,int len)
{
uint8_t     buf[10];
uint16_t    value;

   if (len < 1)
      return(CMD_ERR_INVALID_LENGTH);
   if (!BOARD_get_ADC(data[0],&value))
   	return(CMD_ERR_COMMAND_FAILED);
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_GET_ADC_VALUE,CMD_RX,BOARD_GetOwnAddress());
   SetVal_16(buf+6,data[0]);
   SetVal_16(buf+8,value);
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Get the value of an ADC channel with its calibrated
 * offset
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
static int cmd_SUB_SYS_GET_ADC_VAL_OFFSET(uint8_t *data,int len)
{
uint8_t     buf[12];
uint16_t    value,offset;

   if (len < 1)
      return(CMD_ERR_INVALID_LENGTH);
	if (!BOARD_getValueAndOffset(data[0],&value,&offset))
   	return(CMD_ERR_COMMAND_FAILED);
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_GET_ADC_VAL_OFFSET,CMD_RX,BOARD_GetOwnAddress());
   SetVal_16(buf+6,data[0]);
   SetVal_16(buf+8,value);
   SetVal_16(buf+10,offset);
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Get all ADC values
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_GET_ALL_ADC_VALUES(uint8_t *data,int len)
{
uint8_t     buf[BOARD_ADC_NumberOfChannels * 2 + 8];
uint16_t    value;

   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_GET_ALL_ADC_VALUES,CMD_RX,BOARD_GetOwnAddress());
   SetVal_16(buf+6,(uint16_t)BOARD_ADC_NumberOfChannels);
   for (int i = 0;i < BOARD_ADC_NumberOfChannels;i++)
   {
      if (!BOARD_get_ADC(i,&value))
      {
      	return(CMD_ERR_COMMAND_FAILED);
      }
   	SetVal_16(buf+i*2+8,value);
   }
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Get all ADC values with their calibrated offset
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_GET_ALL_ADC_VAL_OFFSET(uint8_t *data,int len)
{
uint8_t     buf[BOARD_ADC_NumberOfChannels * 4 + 8];
uint16_t    value,offset;

   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_GET_ALL_ADC_VAL_OFFSET,CMD_RX,BOARD_GetOwnAddress());
   SetVal_16(buf+6,(uint16_t)BOARD_ADC_NumberOfChannels);
   for (int i = 0;i < BOARD_ADC_NumberOfChannels;i++)
   {
      if (!BOARD_getValueAndOffset(i,&value,&offset))
      {
      	return(CMD_ERR_COMMAND_FAILED);
      }
   	SetVal_16(buf+i*4+8,value);
   	SetVal_16(buf+i*4+10,offset);
   }
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Get the value of an ADC channel as floating point value
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
static int cmd_SUB_SYS_GET_ADC_VALUE_FLOAT(uint8_t *data,int len)
{
uint8_t     buf[12];
float    	value;

   if (len < 1)
      return(CMD_ERR_INVALID_LENGTH);
   if (!BOARD_get_ADC_float(data[0],&value))
   	return(CMD_ERR_COMMAND_FAILED);
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_GET_ADC_VALUE_FLOAT,CMD_RX,BOARD_GetOwnAddress());
   SetVal_16(buf+6,(uint16_t)(data[0]));
   SetVal_Float(buf+8,value);
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Get all ADC values as floating point values
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_GET_ALL_ADC_VAL_FLOAT(uint8_t *data,int len)
{
uint8_t     buf[BOARD_ADC_NumberOfChannels * 4 + 8];
float    	value;

   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_GET_ALL_ADC_VAL_FLOAT,CMD_RX,BOARD_GetOwnAddress());
   SetVal_16(buf+6,(uint16_t)BOARD_ADC_NumberOfChannels);
   for (int i = 0;i < BOARD_ADC_NumberOfChannels;i++)
   {
      if (!BOARD_get_ADC_float(i,&value))
      {
      	return(CMD_ERR_COMMAND_FAILED);
      }
      SetVal_Float(buf+i*4+8,value);
   }
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Get all ADC-Values and the Temperature Values (float)
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_GET_ALL_ADC_AND_TEMP_F(uint8_t *data,int len)
{
uint8_t     buf[BOARD_ADC_NumberOfChannels * 4 + 8 + 8];
#if USE_FLOAT != 0
float    	value;
float    	fvalue;
#else
int32_t    	value;
float			fvalue;
#endif

   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_GET_ALL_ADC_AND_TEMP_F,CMD_RX,BOARD_GetOwnAddress());
   SetVal_16(buf+6,(uint16_t)BOARD_ADC_NumberOfChannels + 2);
   for (int i = 0;i < BOARD_ADC_NumberOfChannels;i++)
   {
      if (!BOARD_get_ADC_float(i,&fvalue))
      {
      	return(CMD_ERR_COMMAND_FAILED);
      }
      SetVal_Float(buf+i*4+8,fvalue);
   }
	if (!TMP100_GetTemperature(0,&value))
		return CMD_ERR_COMMAND_FAILED;
	SetVal_Float(buf+BOARD_ADC_NumberOfChannels*4+8,value);
	if (!TMP100_GetTemperature(1,&value))
		return CMD_ERR_COMMAND_FAILED;
	SetVal_Float(buf+BOARD_ADC_NumberOfChannels*4+12,value);
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Get the Safety Manager Status
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_GET_SAFETYMNGR_STATUS(uint8_t *data,int len)
{
uint8_t     buf[35];
uint32_t 	SafetyState;
uint32_t 	SafetyCheckError;
uint32_t 	WatchDogErrors;
uint32_t 	ANTOkErrors;
uint32_t 	SensorErrors;
uint32_t 	RecoveryErrors;
uint32_t 	EnabledTests;
uint8_t 		TaskIsRunning;

	if (!SafetyMngrGetStatus(&SafetyState,&SafetyCheckError,&WatchDogErrors,&ANTOkErrors,&SensorErrors,
									 &RecoveryErrors,&EnabledTests,&TaskIsRunning))
		return CMD_ERR_COMMAND_FAILED;
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_GET_SAFETYMNGR_STATUS,CMD_RX,BOARD_GetOwnAddress());
   SetVal_32(buf+6,SafetyState);
   SetVal_32(buf+10,SafetyCheckError);
   SetVal_32(buf+14,WatchDogErrors);
   SetVal_32(buf+18,ANTOkErrors);
   SetVal_32(buf+22,SensorErrors);
   SetVal_32(buf+26,RecoveryErrors);
   SetVal_32(buf+30,EnabledTests);
	buf[34] = TaskIsRunning;
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Get the Safety Manager Internal Status
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_GET_SAFETYMNGR_INT_STATUS(uint8_t *data,int len)
{
uint8_t     buf[14];
uint32_t 	SafetyTaskError;
uint32_t 	SafetyTaskStatus;

	if (!SafetyMngrGetIntStatus(&SafetyTaskError,&SafetyTaskStatus))
		return CMD_ERR_COMMAND_FAILED;
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_GET_SAFETYMNGR_INT_STATUS,CMD_RX,BOARD_GetOwnAddress());
   SetVal_32(buf+6,SafetyTaskError);
   SetVal_32(buf+10,SafetyTaskStatus);
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Get the Cleaning Manager Status
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_GET_CLEANINGMNGR_STATUS(uint8_t *data,int len)
{
uint8_t     buf[23];
uint16_t 	value[6];

	if (!ClMgr_GetStatus(value,6))
		return CMD_ERR_COMMAND_FAILED;
	for (int i = 0;i < 6;i++)
		SetVal_16(buf + 2 * i + 6,value[i]);
	if (!ClMgr_GetDeviceStatus(buf + 18,5))
		return CMD_ERR_COMMAND_FAILED;
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_GET_CLEANINGMNGR_STATUS,CMD_RX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Get the Cleaning Manager Maximum Device Currents
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_GET_CLMNGR_MAX_CUR(uint8_t *data,int len)
{
uint8_t     buf[26];
uint32_t 	value[5];

	if (!ClMgr_GetMaxDeviceCurrent(value,5))
		return CMD_ERR_COMMAND_FAILED;
	for (int i = 0;i < 5;i++)
		SetVal_32(buf + 4 * i + 6,value[i]);
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_GET_CLMNGR_MAX_CUR,CMD_RX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Resets the Cleaning Manager Maximum Device Currents
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_RESET_CLMNGR_MAX_CUR(uint8_t *data,int len)
{
uint8_t     buf[6];

	if (len < 2)
      return(CMD_ERR_INVALID_LENGTH);

	if (!ClMgr_ResetMaxDeviceCurrent(GetU16_Val(data)))
		return CMD_ERR_COMMAND_FAILED;
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_RESET_CLMNGR_MAX_CUR,CMD_TX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Set or Reset the Dry Run Mode
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_SET_DRYRUN(uint8_t *data,int len)
{
uint8_t     buf[6];

	if (len < 1)
      return(CMD_ERR_INVALID_LENGTH);
	if (!ClMgr_EnableDryRun(data[0]))
		return CMD_ERR_COMMAND_FAILED;
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_SET_DRYRUN,CMD_TX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Set or Maximum Lift Current for a Lift
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_SET_LIFT_MAX_CURRENT(uint8_t *data,int len)
{
uint8_t     buf[6];

	if (len < 5)
      return(CMD_ERR_INVALID_LENGTH);
	
	if (!SetLiftMaxCurrent(data[0],GetU16_Val(data + 1)))
		return CMD_ERR_COMMAND_FAILED;
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_SET_LIFT_MAX_CURRENT,CMD_TX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Get the Safety Manager Status
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_GET_LIFT_HALL_COUNT(uint8_t *data,int len)
{
uint8_t     buf[12];
int			value;
	
   if (len < 1)
      return(CMD_ERR_INVALID_LENGTH);
   if ((value = GetHallCountPulses(data[0])) == -1)
   	return(CMD_ERR_COMMAND_FAILED);
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_GET_LIFT_HALL_COUNT,CMD_RX,BOARD_GetOwnAddress());
   SetVal_32(buf+6,value);
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Get the Lift Device Status
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_GET_LIFT_DEVICE_STATUS(uint8_t *data,int len)
{
uint8_t     buf[15];
int			value;
uint32_t		current;
	
   if (len < 1)
      return(CMD_ERR_INVALID_LENGTH);
   value = GetLiftDeviceStatus(data[0]);
	current = GetLiftDeviceOvercurrentTest(data[0]);
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_GET_LIFT_DEVICE_STATUS,CMD_RX,BOARD_GetOwnAddress());
	buf[6] = data[0];
   SetVal_32(buf+7,value);
   SetVal_32(buf+11,current);
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Switch a Relay On or Off
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SUB_SYS_SWITCH_RELAY(uint8_t *data,int len)
{
uint8_t     buf[7];
uint16_t		delay;
	
   if (len < 4)
      return(CMD_ERR_INVALID_LENGTH);
	delay = GetU16_Val(data+2);
	if (delay < 2000)
		delay = 2000;
	if (data[0] == 0)
	{
		if (!ControlRelay1(data[1],delay,false))
			return CMD_ERR_COMMAND_FAILED;
	}
	else
		if (data[0] == 1)
		{
			if (!ControlRelay2(data[1],false))
				return CMD_ERR_COMMAND_FAILED;
		}
		else
			return(CMD_ERR_INVALID_LENGTH);
	buf[6] = data[0];
	MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_SWITCH_RELAY,CMD_TX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Gets the Status of the Relays
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_GET_RELAY_STATUS(uint8_t *data,int len)
{
uint8_t     buf[11];
float			value;
	
	if (!BOARD_GetRelayStatus(buf + 6,&value))
		return CMD_ERR_COMMAND_FAILED;
	SetVal_Float(buf+7,value);
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_GET_RELAY_STATUS,CMD_RX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Enables the Lift Motor Hall Counters for Debugging
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_ENA_LIFT_HALL_CNTR(uint8_t *data,int len)
{
uint8_t     buf[6];
	
	if (len < 2)
		return(CMD_ERR_INVALID_LENGTH);
	if (!BOARD_EnaHallTestCounters(data[0],data[1]))
		return CMD_ERR_COMMAND_FAILED;
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_ENA_LIFT_HALL_CNTR,CMD_TX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Enables the Flow Meter for Debugging
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_ENA_FLOWMETER_CNTR(uint8_t *data,int len)
{
uint8_t     buf[6];
	
	if (len < 2)
		return(CMD_ERR_INVALID_LENGTH);
	if (!BOARD_EnaFlowMeterTestCounter(data[0],data[1]))
		return CMD_ERR_COMMAND_FAILED;
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_ENA_FLOWMETER_CNTR,CMD_TX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Resets the Lift Motor Hall Counters for Debugging
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_RESET_LIFT_HALL_CNTR(uint8_t *data,int len)
{
uint8_t     buf[6];
	
	if (len < 1)
		return(CMD_ERR_INVALID_LENGTH);
	if (!BOARD_ResetHallTestCounters(data[0]))
		return CMD_ERR_COMMAND_FAILED;
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_RESET_LIFT_HALL_CNTR,CMD_TX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Resets the Flow Meter Counter for Debugging
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_RESET_FLOWMETER_CNTR(uint8_t *data,int len)
{
uint8_t     buf[6];
	
	if (!BOARD_ResetFlowMeterTestCounter())
		return CMD_ERR_COMMAND_FAILED;
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_RESET_FLOWMETER_CNTR,CMD_TX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Resets the Endswitch State of the Lift Devices
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_RESET_ENDSWITCH_STATE(uint8_t *data,int len)
{
uint8_t     buf[6];
	
	ClMgr_ClearLiftEndSWstate();
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_RESET_ENDSWITCH_STATE,CMD_TX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Resets the Cleaning Manager FSM Error State
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_RESET_CLMGR_FSM_ERR(uint8_t *data,int len)
{
uint8_t     buf[6];
	
	ClMgr_ResetFSMerrorState();
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_RESET_CLMGR_FSM_ERR,CMD_TX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Sets the Cleaning Manager Water Pump Parametes 
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_SET_PUMP_CL_MGR(uint8_t *data,int len)
{
uint8_t     buf[6];
	
	if (len < 19)
		return(CMD_ERR_INVALID_LENGTH);
	if (!BOARD_ClMngr_SetParameters(GetU32_Val(data),GetU32_Val(data + 4),
		GetU32_Val(data + 8),GetU32_Val(data + 12),GetU16_Val(data + 16)))
		return CMD_ERR_COMMAND_FAILED;
	if (!BOARD_ClMngr_EnaPumpCtrl(data[18]))
		return CMD_ERR_COMMAND_FAILED;
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_SET_PUMP_CL_MGR,CMD_TX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Sends a direct command to the Cleaning Manager 
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_SEND_CL_MGR_CMD(uint8_t *data,int len)
{
uint8_t     buf[6];
uint32_t		command;
uint32_t 	params;
	
	if (len < 8)
		return(CMD_ERR_INVALID_LENGTH);
	command = GetU32_Val(data);
	params = GetU32_Val(data + 4);
	if (!ClMgr_SendDirectCommand(command,params))
		return CMD_ERR_COMMAND_FAILED;
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_SEND_CL_MGR_CMD,CMD_TX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Disables or Enables the Lift Device Adjust 
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_DISABLE_LIFT_ADJUST(uint8_t *data,int len)
{
uint8_t     buf[6];
	
	if (len < 2)
		return(CMD_ERR_INVALID_LENGTH);
	if (!DisableAdjust(data[0],data[1]))
		return CMD_ERR_COMMAND_FAILED;
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_DISABLE_LIFT_ADJUST,CMD_TX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Gets the Lift Motor Hall Counters Values for Debugging
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_GET_LIFT_HALL_CNTR(uint8_t *data,int len)
{
uint8_t     buf[15];
uint32_t		value1,value2;
	
	if (!BOARD_GetHallTestCounters(&value1,&value2))
		return CMD_ERR_COMMAND_FAILED;
	if (!BOARD_GetHallTestStatus(buf + 14))
		return CMD_ERR_COMMAND_FAILED;
	SetVal_32(buf + 6,value1);
	SetVal_32(buf + 10,value2);
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_GET_LIFT_HALL_CNTR,CMD_RX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Gets the Flow Meter Counter Values for Debugging
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_GET_FLOWMETER_CNTR(uint8_t *data,int len)
{
uint8_t     buf[11];
uint32_t		value;
	
	if (!BOARD_GetFlowMeterTestCounters(&value))
		return CMD_ERR_COMMAND_FAILED;
	if (!BOARD_GetFlowMeterTestStatus(buf + 10))
		return CMD_ERR_COMMAND_FAILED;
	SetVal_32(buf + 6,value);
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_GET_FLOWMETER_CNTR,CMD_RX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Gets Information about the System Heap and the
 * FREERTOS Heap
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_GET_HEAP_INFO(uint8_t *data,int len)
{
uint8_t     buf[26];
uint32_t		heap1,heap2,heapBase,HeapSize,StackBase,StackSize;
	
	if (len < 1)
		return(CMD_ERR_INVALID_LENGTH);
	heap1 = configTOTAL_HEAP_SIZE - xPortGetMinimumEverFreeHeapSize();
	heap2 = configTOTAL_HEAP_SIZE - xPortGetFreeHeapSize();
	if (!BOARD_GetStackAndHeapInfo(data[0] != 0,&StackBase,&StackSize,
			&heapBase,&HeapSize))
		return CMD_ERR_COMMAND_FAILED;
	SetVal_32(buf + 6,heap1);
	SetVal_32(buf + 10,heap2);
	SetVal_32(buf + 14,heapBase);
	SetVal_32(buf + 18,HeapSize);
	SetVal_32(buf + 22,StackBase);
	SetVal_32(buf + 26,StackSize);
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_GET_HEAP_INFO,CMD_RX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Gets the number of Flow Meter Pulses
 * FREERTOS Heap
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_GET_FLOWMETER_PULSES(uint8_t *data,int len)
{
uint8_t     buf[10];
uint32_t		pulses;
	
	if (!ClMgr_GetNumberOfFlowPulses(&pulses))
		return CMD_ERR_COMMAND_FAILED;
	SetVal_32(buf + 6,pulses);
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_GET_FLOWMETER_PULSES,CMD_RX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Gets the status of the CANopen Node
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_GET_CAN_STATUS(uint8_t *data,int len)
{
uint8_t     buf[18];
uint32_t		nProvider,id,nmt_state;
	
	if (!GetCANstatus(&nProvider,&id,&nmt_state))
		return CMD_ERR_COMMAND_FAILED;
	SetVal_32(buf + 6,nProvider);
	SetVal_32(buf + 10,id);
	SetVal_32(buf + 14,nmt_state);
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_GET_CAN_STATUS,CMD_RX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Gets the data of a CANopen register
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_GET_CAN_PROVIDER_DATA(uint8_t *data,int len)
{
uint8_t     buf[15];
uint32_t		id,subid,content;
bool			valid;
	
   if (len < 4)
      return(CMD_ERR_INVALID_LENGTH);
	id = GetU16_Val(data);
	subid = GetU16_Val(data+2);
	if (!GetCAN_ProviderContent(id,subid,&content,&valid))
		return CMD_ERR_COMMAND_FAILED;
	SetVal_16(buf + 6,id);
	SetVal_16(buf + 8,subid);
	SetVal_32(buf + 10,content);
	buf[14] = valid;
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_GET_CAN_PROVIDER_DATA,CMD_RX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Gets info about a CANopen Provider
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_GET_CAN_PROVIDER_INFO(uint8_t *data,int len)
{
uint8_t     buf[32];
uint32_t		id;
int			ObjID;
char			*sPtr;
	
   if (len < 2)
      return(CMD_ERR_INVALID_LENGTH);
	id = GetU16_Val(data);
	if (!GetCAN_ProviderInfo(id,&sPtr,&ObjID))
		return CMD_ERR_COMMAND_FAILED;
	SetVal_16(buf + 6,id);
	SetVal_32(buf + 8,ObjID);
	strncpy((char *)(buf + 12),sPtr,20);
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_GET_CAN_PROVIDER_INFO,CMD_RX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Set the Lift Device Status
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_SET_LIFT_STATUS(uint8_t *data,int len)
{
uint8_t     buf[6];
uint32_t		value;
	
   if (len < 5)
      return(CMD_ERR_INVALID_LENGTH);
   value = GetU32_Val(data+1);
	if (!ChangeLiftDeviceStatus(data[0],value))
		return CMD_ERR_COMMAND_FAILED;
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_SET_LIFT_STATUS,CMD_TX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Get the Frequency and Pulse Duration of a Pumpp
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_GET_PUMP_PULSE(uint8_t *data,int len)
{
uint8_t     buf[11];
uint16_t    freq,pulse;

   if (len < 1)
      return(CMD_ERR_INVALID_LENGTH);
	if (!BOARD_GetPumpFreqPulse(*data,&freq,&pulse))
      return(CMD_ERR_COMMAND_FAILED);
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_GET_PUMP_PULSE,CMD_RX,BOARD_GetOwnAddress());
	buf[6] = *data;
   SetVal_16(buf+7,freq);
   SetVal_16(buf+9,pulse);
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Get the On/Off-State of a Pump
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_GET_PUMP_ENABLE(uint8_t *data,int len)
{
uint8_t     buf[8];
bool    		onoff;

   if (len < 1)
      return(CMD_ERR_INVALID_LENGTH);
	if (!BOARD_GetPumpStatus(*data,&onoff))
      return(CMD_ERR_COMMAND_FAILED);
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_GET_PUMP_ENABLE,CMD_RX,BOARD_GetOwnAddress());
	buf[6] = *data;
	buf[7] = onoff ? 1 : 0;
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Sets the value of a GPIO
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
static int cmd_SUB_SYS_SET_GPIO(uint8_t *data,int len)
{
uint8_t     buf[6];

   if (len < 2)
      return(CMD_ERR_INVALID_LENGTH);
   if (!BOARD_WriteGPIOpin(data[0],data[1]))
   	return(CMD_ERR_COMMAND_FAILED);
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_SET_GPIO,CMD_TX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Gets the value of a GPIO
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
static int cmd_SUB_SYS_GET_GPIO(uint8_t *data,int len)
{
uint8_t     buf[8];
uint8_t		value;

   if (len < 1)
      return(CMD_ERR_INVALID_LENGTH);
   if (!BOARD_ReadGPIOpin(data[0],&value))
   	return(CMD_ERR_COMMAND_FAILED);
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_GET_GPIO,CMD_RX,BOARD_GetOwnAddress());
   buf[6] = data[0];
   buf[7] = value;
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Set the value of a digital output channel
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
static int cmd_SUB_SYS_SET_DIG_VALUE(uint8_t *data,int len)
{
uint8_t     buf[8];

   if (len < 2)
      return(CMD_ERR_INVALID_LENGTH);
   if (!BOARD_SetGPIO_Output(data[0],data[1]))
   	return(CMD_ERR_COMMAND_FAILED);
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_SET_DIG_VALUE,CMD_TX,BOARD_GetOwnAddress());
	buf[6] = data[0];
	buf[7] = data[1];
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Set the value of a security channel
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
static int cmd_SUB_SYS_SET_DIG_SECURITY_VALUE(uint8_t *data,int len)
{
uint8_t     buf[8];

   if (len < 2)
      return(CMD_ERR_INVALID_LENGTH);
   if (!BOARD_SetSecurityOutput(data[0],data[1]))
   	return(CMD_ERR_COMMAND_FAILED);
	buf[6] = data[0];
	buf[7] = data[1];
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_SET_DIG_SECURITY_VALUE,CMD_TX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Set the Test Multiplexer
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
static int cmd_SUB_SYS_SET_TEST_MUX(uint8_t *data,int len)
{
uint8_t     buf[6];

   if (len < 2)
      return(CMD_ERR_INVALID_LENGTH);
   if (!BOARD_SetTestMUX(data[0],data[1]))
   	return(CMD_ERR_COMMAND_FAILED);
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_SET_TEST_MUX,CMD_TX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Set the control values of a PWM channel
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
static int cmd_SUB_SYS_SET_PWM_CONTROL(uint8_t *data,int len)
{
uint8_t     buf[7];

   if (len < 4)
      return(CMD_ERR_INVALID_LENGTH);
	if (!BOARD_SetPWMControl(data[0],GetU16_Val(data+2),(eDirMode_t)(data[1])))
   	return(CMD_ERR_COMMAND_FAILED);
	buf[6] = data[0];
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_SET_PWM_CONTROL,CMD_TX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Set the parameters of the CAN channel
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
static int cmd_SUB_SYS_SET_CAN_PARAMETERS(uint8_t *data,int len)
{
uint8_t     buf[6];

   if (len < 13)
      return(CMD_ERR_INVALID_LENGTH);
	if (!CAN_ChangeParameters(0,GetU32_Val(data),GetU32_Val(data+4),
                         GetU32_Val(data+8),data[12]))
   	return(CMD_ERR_COMMAND_FAILED);
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_SET_CAN_PARAMETERS,CMD_TX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Set the CAN acceptance filter of the CAN channel
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
static int cmd_SUB_SYS_SET_CAN_ACCEPTANCE(uint8_t *data,int len)
{
uint8_t     buf[6];

   if (len < 5)
      return(CMD_ERR_INVALID_LENGTH);
	if (!CAN_SetAcceptanceFilter(0,GetU32_Val(data),data[4]))
   	return(CMD_ERR_COMMAND_FAILED);
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_SET_CAN_ACCEPTANCE,CMD_TX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Set the CAN ID of the CAN channel
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
static int cmd_SUB_SYS_SET_CAN_ID(uint8_t *data,int len)
{
uint8_t     buf[6];

   if (len < 5)
      return(CMD_ERR_INVALID_LENGTH);
	if (!CAN_ChangeID(0,GetU32_Val(data),data[4]))
   	return(CMD_ERR_COMMAND_FAILED);
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_SET_CAN_ID,CMD_TX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Set the CAN ID of the CAN channel
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
static int cmd_SUB_SYS_CLEAR_ERRORS(uint8_t *data,int len)
{
uint8_t     buf[6];

	if (!CAN_clearErrorFlags(0))
   	return(CMD_ERR_COMMAND_FAILED);
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_CLEAR_ERRORS,CMD_TX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}


/*!
 ******************************************************************************
 *	System Subcommand: Send a CAN message
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
static int cmd_SUB_SYS_SEND_CAN_MESSAGE(uint8_t *data,int len)
{
uint8_t     buf[6];

	/*
		data[0..3]	CAN ID
		data[4]		Payload Length
		data[5]		1 if CAN ID is an extended ID, 0 else
		data[6..12]	Payload (size depends on Payload Length (0 .. 8)
	*/
   if (len < 6)
      return(CMD_ERR_INVALID_LENGTH);
   if (len < 6 + data[4])		// Check the payload length
      return(CMD_ERR_INVALID_LENGTH);
	if (!CAN_SendMessage(0,GetU32_Val(data),data[5],data + 6,data[4]))
   	return(CMD_ERR_COMMAND_FAILED);
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_SEND_CAN_MESSAGE,CMD_TX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Set the CAN baudrate
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
static int cmd_SUB_SYS_SET_CAN_BAUDRATE(uint8_t *data,int len)
{
uint8_t     buf[6];

   if (len < 4)
      return(CMD_ERR_INVALID_LENGTH);
	if (!CAN_ChangeBaudrate(0,GetU32_Val(data)))
   	return(CMD_ERR_COMMAND_FAILED);
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_SET_CAN_BAUDRATE,CMD_TX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Set the outputs of the security chain
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
static int cmd_SUB_SYS_SET_SECURITY_GPIOS(uint8_t *data,int len)
{
uint8_t     buf[6];

   if (len < 4)
      return(CMD_ERR_INVALID_LENGTH);
	if (!BOARD_SetAllSecurityOutput(GetU32_Val(data)))
   	return(CMD_ERR_COMMAND_FAILED);
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_SET_SECURITY_GPIOS,CMD_TX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Suspend or Resume all CUC tasks
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
static int cmd_SUB_SYS_SET_ALL_CUC_TASKS_ENA(uint8_t *data,int len)
{
uint8_t     buf[6];

   if (len < 1)
      return(CMD_ERR_INVALID_LENGTH);		
	if (*data != 0)
	{
		if (sysThread != NULL)
			osThreadResume(sysThread);
		CUC_Task_ResumeAll();
		BOARD_Ena_CUC_IRQ_Callback(true);
	}
	else
	{
		if (sysThread != NULL)
			osThreadSuspend(sysThread);
		CUC_Task_SuspendAll();
		BOARD_Ena_CUC_IRQ_Callback(false);
	}
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_SET_ALL_CUC_TASKS_ENA,CMD_TX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Enable or disable the External Watchdog Test
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
static int cmd_SUB_SYS_SET_TEST_EXT_WD(uint8_t *data,int len)
{
uint8_t     buf[6];

   if (len < 1)
      return(CMD_ERR_INVALID_LENGTH);
	if (!BOARD_Set_Test_Ext_WD(*data))
   	return(CMD_ERR_COMMAND_FAILED);
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_SET_TEST_EXT_WD,CMD_TX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Set Frequency and Pulse Duration for a Pump Driver
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
static int cmd_SUB_SYS_SET_PUMP_PULSE(uint8_t *data,int len)
{
uint8_t     buf[6];

   if (len < 5)
      return(CMD_ERR_INVALID_LENGTH);
	if (!BOARD_SetPumpPWM(*data,1000))
   	return(CMD_ERR_COMMAND_FAILED);
	if (!BOARD_SetPumpFreqPulse(*data,GetU16_Val(data + 1), GetU16_Val(data + 3)))
   	return(CMD_ERR_COMMAND_FAILED);
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_SET_PUMP_PULSE,CMD_TX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Enables or Disables a Pump (On/Off)
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
static int cmd_SUB_SYS_SET_PUMP_ENABLE(uint8_t *data,int len)
{
uint8_t     buf[6];

   if (len < 2)
      return(CMD_ERR_INVALID_LENGTH);
	if (!BOARD_EnablePump(data[0],data[1] != 0))
   	return(CMD_ERR_COMMAND_FAILED);
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_SET_PUMP_ENABLE,CMD_TX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Restarts the Safety Manager Task
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
static int cmd_SUB_SYS_RESTART_SAFETYMNGR(uint8_t *data,int len)
{
uint8_t     buf[6];

	RestartSafetyManager();
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_RESTART_SAFETYMNGR,CMD_TX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Sets the Safety Manager Checks
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
static int cmd_SUB_SYS_SET_SAFETYMNGR_CHECKS(uint8_t *data,int len)
{
uint8_t     buf[6];

   if (len < 4)
      return(CMD_ERR_INVALID_LENGTH);
	SafetyMngrEnableChecks(GetU32_Val(data));
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_SET_SAFETYMNGR_CHECKS,CMD_TX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Get the setting of the Test Multiplexer
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
static int cmd_SUB_SYS_GET_TEST_MUX(uint8_t *data,int len)
{
uint8_t     buf[8];
unsigned		channel;
bool			enable;

   if (!BOARD_GetTestMUX(&channel,&enable))
   	return(CMD_ERR_COMMAND_FAILED);
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_GET_TEST_MUX,CMD_RX,BOARD_GetOwnAddress());
   buf[6] = channel;
   buf[7] = enable ? 1 : 0;
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Get the value of a digital input channel
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
static int cmd_SUB_SYS_GET_DIG_VALUE(uint8_t *data,int len)
{
uint8_t     buf[8];
uint8_t		value;

   if (len < 1)
      return(CMD_ERR_INVALID_LENGTH);
   if (!BOARD_GetGPIO_Input(data[0],&value))
   	return(CMD_ERR_COMMAND_FAILED);
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_GET_DIG_VALUE,CMD_RX,BOARD_GetOwnAddress());
   buf[6] = data[0];
   buf[7] = value;
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Get the value of a security channel
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
static int cmd_SUB_SYS_GET_DIG_SECURITY_VALUE(uint8_t *data,int len)
{
uint8_t     buf[8];
uint8_t		value;

   if (len < 1)
      return(CMD_ERR_INVALID_LENGTH);
   if (!BOARD_GetSecurityGPIO(data[0],&value))
   	return(CMD_ERR_COMMAND_FAILED);
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_GET_DIG_SECURITY_VALUE,CMD_RX,BOARD_GetOwnAddress());
   buf[6] = data[0];
   buf[7] = value;
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Get the value of all digital input channels
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
static int cmd_SUB_SYS_GET_ALL_DIG_VALUES(uint8_t *data,int len)
{
uint8_t     buf[10];
uint32_t    value;

   if (!BOARD_GetAllGPIO_Inputs(data[0],&value))
   	return(CMD_ERR_COMMAND_FAILED);
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_GET_ALL_DIG_VALUES,CMD_RX,BOARD_GetOwnAddress());
   SetVal_32(buf+6,value);
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Get the value of all security channel
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
static int cmd_SUB_SYS_GET_ALL_DIG_SEC_VALUES(uint8_t *data,int len)
{
uint8_t     buf[14];
uint32_t    value[2];

   if (!BOARD_GetAllSecurityGPIOs(value,2))
   	return(CMD_ERR_COMMAND_FAILED);
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_GET_ALL_DIG_SEC_VALUES,CMD_RX,BOARD_GetOwnAddress());
   SetVal_32(buf+6,value[0]);
   SetVal_32(buf+10,value[1]);
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Get the value of an output channel (readback)
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
static int cmd_SUB_SYS_GET_DIG_READBACK(uint8_t *data,int len)
{
uint8_t     buf[8];
uint8_t     value;

   if (len < 1)
      return(CMD_ERR_INVALID_LENGTH);
   if (!BOARD_GetGPIO_Readback(data[0],&value))
   	return(CMD_ERR_COMMAND_FAILED);
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_GET_DIG_READBACK,CMD_RX,BOARD_GetOwnAddress());
   buf[6] = data[0];
   buf[7] = value;
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Get the value of all digital output channel
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
static int cmd_SUB_SYS_GET_ALL_DIG_READBACKS(uint8_t *data,int len)
{
uint8_t     buf[10];
uint32_t    value;

   if (len < 1)
      return(CMD_ERR_INVALID_LENGTH);
   if (!BOARD_GetAllGPIO_Inputs(data[0],&value))
   	return(CMD_ERR_COMMAND_FAILED);
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_GET_ALL_DIG_READBACKS,CMD_RX,BOARD_GetOwnAddress());
   SetVal_32(buf+6,value);
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Get the value of all digital IOs
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
static int cmd_SUB_SYS_GET_ALL_DIG_IOS(uint8_t *data,int len)
{
uint8_t     buf[22];
uint32_t    value[4];

	if (!BOARD_GetAllGPIOsignals(value,4))
   	return(CMD_ERR_COMMAND_FAILED);
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_GET_ALL_DIG_IOS,CMD_RX,BOARD_GetOwnAddress());
	for (int i = 0;i < 4;i++)
		SetVal_32(buf + i * 4 + 6,value[i]);
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Get the values of a PWM control
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
static int cmd_SUB_SYS_GET_PWM_CONTROL(uint8_t *data,int len)
{
uint8_t     	buf[20];
uint32_t    	dutycycle1,dutycycle2,pwm;
eDirMode_t		DirMode;

   if (len < 1)
      return(CMD_ERR_INVALID_LENGTH);
#if defined (USE_DRV8701P) && (USE_DRV8701P != 0)
	if (!BOARD_GetPWMControl(data[0],&pwm,&DirMode,&dutycycle1))
   	return(CMD_ERR_COMMAND_FAILED);
	dutycycle2 = 0;
#else
	if (!BOARD_GetPWMControl(data[0],&pwm,&DirMode,&dutycycle1,&dutycycle2))
   	return(CMD_ERR_COMMAND_FAILED);
#endif
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_GET_PWM_CONTROL,CMD_RX,BOARD_GetOwnAddress());
	buf[6] = data[0];
	buf[7] = (uint8_t)DirMode;
   SetVal_32(buf+8,pwm);
   SetVal_32(buf+12,dutycycle1);
   SetVal_32(buf+16,dutycycle2);
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Get all parameters of the CAN channel
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_GET_CAN_PARAMETERS(uint8_t *data,int len)
{
uint8_t     buf[19];
uint32_t		baudrate,IDmask,ACCmask;
uint8_t		options;

	if (!CAN_GetParameters(0,&baudrate,&IDmask,&ACCmask,&options))
   	return(CMD_ERR_COMMAND_FAILED);
	MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_GET_CAN_PARAMETERS,CMD_RX,BOARD_GetOwnAddress());
   SetVal_32(buf+6,baudrate);
   SetVal_32(buf+10,IDmask);
   SetVal_32(buf+14,ACCmask);
   buf[18] = options;
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Get the CAN acceptance mask
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_GET_CAN_ACCEPTANCE(uint8_t *data,int len)
{
uint8_t     buf[11];
uint32_t		ACCmask;
uint8_t		ACCmaskOnOff;

	if (!CAN_GetAcceptanceFilter(0,&ACCmask,&ACCmaskOnOff))
   	return(CMD_ERR_COMMAND_FAILED);
	MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_GET_CAN_ACCEPTANCE,CMD_RX,BOARD_GetOwnAddress());
   SetVal_32(buf+6,ACCmask);
   buf[10] = ACCmaskOnOff;;
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Get the CAN ID
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_GET_CAN_ID(uint8_t *data,int len)
{
uint8_t     buf[11];
uint32_t		ID;
uint8_t		Type;

	if (!CAN_GetID(0,&ID,&Type))
   	return(CMD_ERR_COMMAND_FAILED);
	MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_GET_CAN_ID,CMD_RX,BOARD_GetOwnAddress());
   SetVal_32(buf+6,ID);
   buf[10] = Type;;
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Get the CAN Error Flags
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_GET_CAN_ERROR_FLAGS(uint8_t *data,int len)
{
uint8_t     buf[10];
uint32_t		errFlags;

   if (len < 1)
      return(CMD_ERR_INVALID_LENGTH);
	errFlags = CAN_getErrorFlags(0,data[0]);
	MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_GET_CAN_ERROR_FLAGS,CMD_RX,BOARD_GetOwnAddress());
   SetVal_32(buf+6,errFlags);
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Get the number of available (received) CAN messages 
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_GET_CAN_N_RX_MSGS(uint8_t *data,int len)
{
uint8_t     buf[10];
int			nCANmessages;

	nCANmessages = CAN_NumberOfRxMessageAvailable(0);
	MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_GET_CAN_N_RX_MSGS,CMD_RX,BOARD_GetOwnAddress());
   SetVal_32(buf+6,nCANmessages);
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Get a (received) CAN messages from the RX FIFO
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_GET_CAN_MESSAGE(uint8_t *data,int len)
{
uint8_t     buf[21];
uint32_t		ID;
bool 			IDisExtended;
int			packet_len;

	/*
		buf[6 + 0..3]	CAN ID
		buf[6 + 4]		Payload Length
		buf[6 + 5]		1 if CAN ID is an extended ID, 0 else
		buf[6 + 6..13]	Payload (size depends on Payload Length (0 .. 8)
		buf[6 + 14]		1 if there is a Packet available, 0 else
	*/
	if (!CAN_getRxMessage(0,&ID,&IDisExtended,buf+12,&packet_len))
		buf[20] = 0;
	else
	{
		SetVal_32(buf+6,ID);
		buf[10] = packet_len;
		buf[11] = IDisExtended ? 1 : 0;
		buf[20] = 1;
	}
	MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_GET_CAN_MESSAGE,CMD_RX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Get the CAN baudrate
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_GET_CAN_BAUDRATE(uint8_t *data,int len)
{
uint8_t     buf[10];
uint32_t		baudrate;

	if (!CAN_GetBaudrate(0,&baudrate))
   	return(CMD_ERR_COMMAND_FAILED);
	MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_GET_CAN_BAUDRATE,CMD_RX,BOARD_GetOwnAddress());
   SetVal_32(buf+6,baudrate);
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Gets the temperature of a temperature sensor at the
 * I2C-Bus
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_GET_TEMPERATURE_SENSOR(uint8_t *data,int len)
{
uint8_t     buf[11];
#if USE_FLOAT != 0
float			result;
#else
int32_t		result;
#endif

	if (len < 1)
		return CMD_ERR_INVALID_LENGTH;
	if (!TMP100_GetTemperature(data[0],&result))
		return CMD_ERR_COMMAND_FAILED;
	else
	{
#if USE_FLOAT != 0
		SetVal_Float(buf+6,result);
#else
		SetVal_Float(buf+6,(float)result);
#endif
		buf[10] = data[0];
	}
	MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_GET_TEMPERATURE_SENSOR,CMD_RX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Gets the System Timer (time) in ns
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_GET_SYSTEM_TIME_NS(uint8_t *data,int len)
{
uint8_t     buf[14];
uint64_t		time;

	if (!BOARD_getSystemTime(&time))
		return CMD_ERR_COMMAND_FAILED;
	else
	{
		SetVal_64(buf+6,time);
	}
	MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_GET_SYSTEM_TIME_NS,CMD_RX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Get all system clock frequencies
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_GET_CLOCKS(void)
{
uint8_t     buf[26];

	MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_GET_CLOCKS,CMD_RX,BOARD_GetOwnAddress());
   SetVal_32(buf+6,CLOCK_GetFreq(kCLOCK_CoreSysClk));
   SetVal_32(buf+10,CLOCK_GetFreq(kCLOCK_BusClk));
   SetVal_32(buf+14,CLOCK_GetFreq(kCLOCK_FlashClk));
   SetVal_32(buf+18,g_xtal0Freq);
   SetVal_32(buf+22,CLOCK_GetFreq(kCLOCK_McgIrc48MClk));
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	Gets the Device Type
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_GET_DEVICE_TYPE(void)
{
uint8_t  buf[8];

   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_GET_DEVICE_TYPE,CMD_RX,BOARD_GetOwnAddress());
   SetVal_16(buf+6,GetSystemDeviceType());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Set own address
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_SET_DEVICE_OWN_ADDR(uint8_t *data,int len)
{
uint8_t     buf[6];
uint16_t    address;

   if (len < 2)
      return(CMD_ERR_INVALID_LENGTH);
   address = GetU16_Val(data);
   BOARD_SetOwnAddress(address);
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_SET_DEVICE_OWN_ADDR,CMD_TX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Gets the State of the Lift End Switches
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_GET_ENDSWITCH_STATE(uint8_t *data,int len)
{
uint8_t     buf[10];

   SetVal_32(buf+6,ClMgr_GetLiftEndSWstate());
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_GET_ENDSWITCH_STATE,CMD_RX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Gets the State and Position of a Lift Device
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_GET_LIFT_STATE_POS(uint8_t *data,int len)
{
uint8_t     buf[32];
int 			act_pos;
int 			home_pos;
int 			rest_pos;
int 			delta_pos;
int 			work_pos;
int 			in_range;
uint8_t 		is_up;
uint8_t		adjust_state;

   if (len < 1)
      return(CMD_ERR_INVALID_LENGTH);
	if (!GetLiftStateAndPos(data[0],&act_pos,&home_pos,&rest_pos,&delta_pos,
			&work_pos,&in_range,&is_up,&adjust_state))
		return CMD_ERR_COMMAND_FAILED;
   SetVal_32(buf+6,act_pos);
   SetVal_32(buf+10,home_pos);
   SetVal_32(buf+14,rest_pos);
   SetVal_32(buf+18,delta_pos);
   SetVal_32(buf+22,work_pos);
   SetVal_32(buf+26,in_range);
	buf[30] = is_up;
	buf[31] = adjust_state;
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_GET_LIFT_STATE_POS,CMD_RX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Writes a Data Block to the EEPROM (max. 64 Bytes)
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_WRITE_EEPROM(uint8_t *data,int len)
{
uint8_t     buf[6];
uint16_t		address;
uint16_t		size;

   if (len < 3)
      return(CMD_ERR_INVALID_LENGTH);
	address = GetU16_Val(data);
	size = data[2];
	if (len < size + 3 || len > 35)
      return(CMD_ERR_INVALID_LENGTH);
	if (!EEPROM_WriteBlock(EERPROM_I2C_ADDRESS,address,data + 3,size))
		return CMD_ERR_COMMAND_FAILED;
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_WRITE_EEPROM,CMD_TX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Reads a Data Block from the EEPROM (max. 64 Bytes)
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_READ_EEPROM(uint8_t *data,int len)
{
uint8_t     buf[64+6+3];
uint16_t		address;
uint16_t		size;

   if (len < 3)
      return(CMD_ERR_INVALID_LENGTH);
	address = GetU16_Val(data);
	size = data[2];
	if (size > 32)
      return(CMD_ERR_INVALID_LENGTH);
	if (!EEPROM_ReadBlock(EERPROM_I2C_ADDRESS,address,buf + 9,size))
		return CMD_ERR_COMMAND_FAILED;
	SetVal_16(buf + 6,address);
	buf[8] = size;
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_READ_EEPROM,CMD_RX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,size + 9);
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Gets the Safety Manager Error Counters
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_GET_SAFETY_ERR_CNTR(uint8_t *data,int len)
{
uint8_t     buf[6 + 7];

	if (!SafetyMngrGetErrCounters(buf + 7,6))
		return CMD_ERR_COMMAND_FAILED;
	buf[6] = 6;
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_GET_SAFETY_ERR_CNTR,CMD_RX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Clears the Safety Manager Error Counters
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_CLR_SAFETY_ERR_CNTR(uint8_t *data,int len)
{
uint8_t     buf[6];

	if (!SafetyMngrClrErrCounters())
		return CMD_ERR_COMMAND_FAILED;
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_CLR_SAFETY_ERR_CNTR,CMD_TX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Enables or disables the CAN Self Reception
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_CAN_SET_SELF_RX(uint8_t *data,int len)
{
uint8_t     buf[6];

   if (len < 1)
      return(CMD_ERR_INVALID_LENGTH);
	if (!CAN_EnableSelfReception(0,data[0]))
		return CMD_ERR_COMMAND_FAILED;
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_CAN_SET_SELF_RX,CMD_TX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Subcommand: Injects a Lift Command Error
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_INJECT_LIFT_ERROR(uint8_t *data,int len)
{
uint8_t     buf[6];

   if (len < 3)
      return(CMD_ERR_INVALID_LENGTH);
	if (!Lift_InjectError(data[0],GetU16_Val(data + 1)))
		return CMD_ERR_COMMAND_FAILED;
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_INJECT_LIFT_ERROR,CMD_TX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	Gets the CANopen Node TX- and RX-Counter
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_GET_CANOPEN_CTRS(uint8_t *data,int len)
{
int		TX_Counter,RX_Counter;
uint8_t  buf[14];

	CAN_Node_GetCounters(&TX_Counter,&RX_Counter);
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_GET_CANOPEN_CTRS,CMD_RX,BOARD_GetOwnAddress());
   SetVal_32(buf+6,TX_Counter);
   SetVal_32(buf+10,RX_Counter);
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	Sets the Ramp Slope of the Brush or Suction Device
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_SET_RAMP_SLOPE(uint8_t *data,int len)
{
uint8_t  buf[6];

   if (len < 9)
      return(CMD_ERR_INVALID_LENGTH);
	if (!ClMgr_SetRampSlope(buf[0],GetU32_Val(buf + 1),GetU32_Val(buf + 5)))
		return CMD_ERR_COMMAND_FAILED;
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_SET_RAMP_SLOPE,CMD_TX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	Gets the status of the PWM Drivers
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_SYS_GET_PWM_STATUS(uint8_t *data,int len)
{
uint8_t  buf[8];

	buf[6] = Board_GetPWMnSleep();
	buf[7] = Board_GetPWMnFault();
   MakeCommandHeader(buf,CMD_SYSTEM,CMD_ACK,SUB_SYS_GET_PWM_STATUS,CMD_RX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	System Command: Calls the System SUB-Command functions
 *	\param[in]	dev_nr      device number (index)
 *	\param[in]	command     parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int SystemCommandHandler(int16_t dev_nr,uint8_t *command,int len)
{
   switch(*command)
   {
		case SUB_SYS_REQUEST_PING:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_REQUEST_PING(command+1,len-1);
		case SUB_SYS_SET_DEVICE_OWN_ADDR:
			SendCommandType(CMD_TX);
			return cmd_SUB_SYS_SET_DEVICE_OWN_ADDR(command+1,len-1);
		case SUB_SYS_SET_DIG_VALUE:
			SendCommandType(CMD_TX);
			return cmd_SUB_SYS_SET_DIG_VALUE(command+1,len-1);
		case SUB_SYS_SET_GPIO:
			SendCommandType(CMD_TX);
			return cmd_SUB_SYS_SET_GPIO(command+1,len-1);
		case SUB_SYS_SET_DIG_SECURITY_VALUE:
			SendCommandType(CMD_TX);
			return cmd_SUB_SYS_SET_DIG_SECURITY_VALUE(command+1,len-1);
		case SUB_SYS_SET_TEST_MUX:
			SendCommandType(CMD_TX);
			return cmd_SUB_SYS_SET_TEST_MUX(command+1,len-1);
		case SUB_SYS_SET_PWM_CONTROL:
			SendCommandType(CMD_TX);
			return cmd_SUB_SYS_SET_PWM_CONTROL(command+1,len-1);
		case SUB_SYS_SET_CAN_PARAMETERS:
			SendCommandType(CMD_TX);
			return cmd_SUB_SYS_SET_CAN_PARAMETERS(command+1,len-1);
		case SUB_SYS_SET_CAN_ACCEPTANCE:
			SendCommandType(CMD_TX);
			return cmd_SUB_SYS_SET_CAN_ACCEPTANCE(command+1,len-1);
		case SUB_SYS_SET_CAN_ID:
			SendCommandType(CMD_TX);
			return cmd_SUB_SYS_SET_CAN_ID(command+1,len-1);
		case SUB_SYS_CLEAR_ERRORS:
			SendCommandType(CMD_TX);
			return cmd_SUB_SYS_CLEAR_ERRORS(command+1,len-1);
		case SUB_SYS_SEND_CAN_MESSAGE:
			SendCommandType(CMD_TX);
			return cmd_SUB_SYS_SEND_CAN_MESSAGE(command+1,len-1);
		case SUB_SYS_SET_CAN_BAUDRATE:
			SendCommandType(CMD_TX);
			return cmd_SUB_SYS_SET_CAN_BAUDRATE(command+1,len-1);
		case SUB_SYS_SET_SECURITY_GPIOS:
			SendCommandType(CMD_TX);
			return cmd_SUB_SYS_SET_SECURITY_GPIOS(command+1,len-1);
		case SUB_SYS_SET_ALL_CUC_TASKS_ENA:
			SendCommandType(CMD_TX);
			return cmd_SUB_SYS_SET_ALL_CUC_TASKS_ENA(command+1,len-1);
		case SUB_SYS_SET_TEST_EXT_WD:
			SendCommandType(CMD_TX);
			return cmd_SUB_SYS_SET_TEST_EXT_WD(command+1,len-1);
		case SUB_SYS_SET_PUMP_PULSE:
			SendCommandType(CMD_TX);
			return cmd_SUB_SYS_SET_PUMP_PULSE(command+1,len-1);
		case SUB_SYS_SET_PUMP_ENABLE:
			SendCommandType(CMD_TX);
			return cmd_SUB_SYS_SET_PUMP_ENABLE(command+1,len-1);
		case SUB_SYS_SET_SAFETYMNGR_CHECKS:
			SendCommandType(CMD_TX);
			return cmd_SUB_SYS_SET_SAFETYMNGR_CHECKS(command+1,len-1);
		case SUB_SYS_RESTART_SAFETYMNGR:
			SendCommandType(CMD_TX);
			return cmd_SUB_SYS_RESTART_SAFETYMNGR(command+1,len-1);
		case SUB_SYS_SET_LIFT_STATUS:
			SendCommandType(CMD_TX);
			return cmd_SUB_SYS_SET_LIFT_STATUS(command+1,len-1);
		case SUB_SYS_ENA_LIFT_HALL_CNTR:
			SendCommandType(CMD_TX);
			return cmd_SUB_SYS_ENA_LIFT_HALL_CNTR(command+1,len-1);
		case SUB_SYS_RESET_LIFT_HALL_CNTR:
			SendCommandType(CMD_TX);
			return cmd_SUB_SYS_RESET_LIFT_HALL_CNTR(command+1,len-1);
		case SUB_SYS_ENA_FLOWMETER_CNTR:
			SendCommandType(CMD_TX);
			return cmd_SUB_SYS_ENA_FLOWMETER_CNTR(command+1,len-1);
		case SUB_SYS_RESET_FLOWMETER_CNTR:
			SendCommandType(CMD_TX);
			return cmd_SUB_SYS_RESET_FLOWMETER_CNTR(command+1,len-1);
		case SUB_SYS_RESET_ENDSWITCH_STATE:
			SendCommandType(CMD_TX);
			return cmd_SUB_SYS_RESET_ENDSWITCH_STATE(command+1,len-1);
		case SUB_SYS_RESET_CLMGR_FSM_ERR:
			SendCommandType(CMD_TX);
			return cmd_SUB_SYS_RESET_CLMGR_FSM_ERR(command+1,len-1);
		case SUB_SYS_SET_PUMP_CL_MGR:
			SendCommandType(CMD_TX);
			return cmd_SUB_SYS_SET_PUMP_CL_MGR(command+1,len-1);
		case SUB_SYS_SEND_CL_MGR_CMD:
			SendCommandType(CMD_TX);
			return cmd_SUB_SYS_SEND_CL_MGR_CMD(command+1,len-1);
		case SUB_SYS_DISABLE_LIFT_ADJUST:
			SendCommandType(CMD_TX);
			return cmd_SUB_SYS_DISABLE_LIFT_ADJUST(command+1,len-1);
		case SUB_SYS_RESET_CLMNGR_MAX_CUR:
			SendCommandType(CMD_TX);
			return cmd_SUB_SYS_RESET_CLMNGR_MAX_CUR(command+1,len-1);
		case SUB_SYS_SET_DRYRUN:
			SendCommandType(CMD_TX);
			return cmd_SUB_SYS_SET_DRYRUN(command+1,len-1);
		case SUB_SYS_SET_LIFT_MAX_CURRENT:
			SendCommandType(CMD_TX);
			return cmd_SUB_SYS_SET_LIFT_MAX_CURRENT(command+1,len-1);

		case SUB_SYS_GET_DEVICE_TYPE:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_GET_DEVICE_TYPE();
		case SUB_SYS_GET_CLOCKS:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_GET_CLOCKS();
		case SUB_SYS_GET_ADC_VALUE:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_GET_ADC_VALUE(command+1,len-1);
		case SUB_SYS_GET_ALL_ADC_VALUES:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_GET_ALL_ADC_VALUES(command+1,len-1);
		case SUB_SYS_GET_ADC_VALUE_FLOAT:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_GET_ADC_VALUE_FLOAT(command+1,len-1);
		case SUB_SYS_GET_ALL_ADC_VAL_FLOAT:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_GET_ALL_ADC_VAL_FLOAT(command+1,len-1);
		case SUB_SYS_GET_DIG_VALUE:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_GET_DIG_VALUE(command+1,len-1);
		case SUB_SYS_GET_DIG_SECURITY_VALUE:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_GET_DIG_SECURITY_VALUE(command+1,len-1);
		case SUB_SYS_GET_ALL_DIG_VALUES:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_GET_ALL_DIG_VALUES(command+1,len-1);
		case SUB_SYS_GET_ALL_DIG_SEC_VALUES:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_GET_ALL_DIG_SEC_VALUES(command+1,len-1);
		case SUB_SYS_GET_DIG_READBACK:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_GET_DIG_READBACK(command+1,len-1);
		case SUB_SYS_GET_ALL_DIG_READBACKS:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_GET_ALL_DIG_READBACKS(command+1,len-1);
		case SUB_SYS_GET_TEST_MUX:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_GET_TEST_MUX(command+1,len-1);
		case SUB_SYS_GET_GPIO:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_GET_GPIO(command+1,len-1);
		case SUB_SYS_GET_ALL_DIG_IOS:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_GET_ALL_DIG_IOS(command+1,len-1);
		case SUB_SYS_GET_PWM_CONTROL:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_GET_PWM_CONTROL(command+1,len-1);
		case SUB_SYS_GET_CAN_ACCEPTANCE:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_GET_CAN_ACCEPTANCE(command+1,len-1);
		case SUB_SYS_GET_CAN_BAUDRATE:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_GET_CAN_BAUDRATE(command+1,len-1);
		case SUB_SYS_GET_CAN_ERROR_FLAGS:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_GET_CAN_ERROR_FLAGS(command+1,len-1);
		case SUB_SYS_GET_CAN_ID:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_GET_CAN_ID(command+1,len-1);
		case SUB_SYS_GET_CAN_N_RX_MSGS:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_GET_CAN_N_RX_MSGS(command+1,len-1);
		case SUB_SYS_GET_CAN_PARAMETERS:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_GET_CAN_PARAMETERS(command+1,len-1);
		case SUB_SYS_GET_CAN_MESSAGE:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_GET_CAN_MESSAGE(command+1,len-1);
		case SUB_SYS_GET_TEMPERATURE_SENSOR:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_GET_TEMPERATURE_SENSOR(command+1,len-1);
		case SUB_SYS_GET_SYSTEM_TIME_NS:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_GET_SYSTEM_TIME_NS(command+1,len-1);
		case SUB_SYS_GET_ADC_VAL_OFFSET:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_GET_ADC_VAL_OFFSET(command+1,len-1);
		case SUB_SYS_GET_ALL_ADC_VAL_OFFSET:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_GET_ALL_ADC_VAL_OFFSET(command+1,len-1);
		case SUB_SYS_GET_PUMP_PULSE:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_GET_PUMP_PULSE(command+1,len-1);
		case SUB_SYS_GET_PUMP_ENABLE:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_GET_PUMP_ENABLE(command+1,len-1);
		case SUB_SYS_GET_ALL_ADC_AND_TEMP_F:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_GET_ALL_ADC_AND_TEMP_F(command+1,len-1);
		case SUB_SYS_GET_SAFETYMNGR_STATUS:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_GET_SAFETYMNGR_STATUS(command+1,len-1);
		case SUB_SYS_GET_SAFETYMNGR_INT_STATUS:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_GET_SAFETYMNGR_INT_STATUS(command+1,len-1);
		case SUB_SYS_GET_CLEANINGMNGR_STATUS:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_GET_CLEANINGMNGR_STATUS(command+1,len-1);
		case SUB_SYS_GET_CLMNGR_MAX_CUR:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_GET_CLMNGR_MAX_CUR(command+1,len-1);
		case SUB_SYS_GET_LIFT_HALL_COUNT:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_GET_LIFT_HALL_COUNT(command+1,len-1);
		case SUB_SYS_GET_LIFT_DEVICE_STATUS:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_GET_LIFT_DEVICE_STATUS(command+1,len-1);
		case SUB_SYS_SWITCH_RELAY:
			SendCommandType(CMD_TX);
			return cmd_SUB_SUB_SYS_SWITCH_RELAY(command+1,len-1);
		case SUB_SYS_GET_RELAY_STATUS:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_GET_RELAY_STATUS(command+1,len-1);
		case SUB_SYS_GET_LIFT_HALL_CNTR:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_GET_LIFT_HALL_CNTR(command+1,len-1);
		case SUB_SYS_GET_FLOWMETER_CNTR:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_GET_FLOWMETER_CNTR(command+1,len-1);
		case SUB_SYS_GET_HEAP_INFO:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_GET_HEAP_INFO(command+1,len-1);
		case SUB_SYS_GET_FLOWMETER_PULSES:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_GET_FLOWMETER_PULSES(command+1,len-1);
		case SUB_SYS_GET_CAN_STATUS:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_GET_CAN_STATUS(command+1,len-1);
		case SUB_SYS_GET_CAN_PROVIDER_DATA:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_GET_CAN_PROVIDER_DATA(command+1,len-1);
		case SUB_SYS_GET_CAN_PROVIDER_INFO:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_GET_CAN_PROVIDER_INFO(command+1,len-1);
		case SUB_SYS_GET_CANOPEN_CTRS:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_GET_CANOPEN_CTRS(command+1,len-1);
		case SUB_SYS_GET_ENDSWITCH_STATE:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_GET_ENDSWITCH_STATE(command+1,len-1);
		case SUB_SYS_GET_LIFT_STATE_POS:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_GET_LIFT_STATE_POS(command+1,len-1);
		case SUB_SYS_WRITE_EEPROM:
			SendCommandType(CMD_TX);
			return cmd_SUB_SYS_WRITE_EEPROM(command+1,len-1);
		case SUB_SYS_READ_EEPROM:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_READ_EEPROM(command+1,len-1);		
		case SUB_SYS_GET_SAFETY_ERR_CNTR:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_GET_SAFETY_ERR_CNTR(command+1,len-1);
		case SUB_SYS_CLR_SAFETY_ERR_CNTR:
			SendCommandType(CMD_TX);
			return cmd_SUB_SYS_CLR_SAFETY_ERR_CNTR(command+1,len-1);
		case SUB_SYS_CAN_SET_SELF_RX:
			SendCommandType(CMD_TX);
			return cmd_SUB_SYS_CAN_SET_SELF_RX(command+1,len-1);
		case SUB_SYS_INJECT_LIFT_ERROR:
			SendCommandType(CMD_TX);
			return cmd_SUB_SYS_INJECT_LIFT_ERROR(command+1,len-1);		
		case SUB_SYS_SET_RAMP_SLOPE:
			SendCommandType(CMD_TX);
			return cmd_SUB_SYS_SET_RAMP_SLOPE(command+1,len-1);
		case SUB_SYS_GET_PWM_STATUS:
			SendCommandType(CMD_RX);
			return cmd_SUB_SYS_GET_PWM_STATUS(command+1,len-1);		
      default:
         return CMD_ERR_UNKNOWN_SUBCMD;    	// we should never get there!
   }
}

