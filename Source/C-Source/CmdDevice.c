/*
 * CmdDevice.c
 *
 *  Created on: Sep 12, 2021
 *      Author: martin
 */

#include "Task_CMSIS2.h"
#include "cmsis_os2.h"
#include "board.h"
#include "Misc.h"
#include "EEPROMHandler.h"
#include "CommandDefs.h"
#include "CommandHandler.h"

#include "CmdDevice.h"

/*!
 ******************************************************************************
 *	Initializes the Device Command Handler
 * \return     1 if success, 0 else
 ******************************************************************************
*/
int InitDeviceCommandHandler(void)
{
   return 1;
}

/*!
 ******************************************************************************
 *	Device Subcommand: Handle a specific Task
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_DEVICE_HANDLE_TASK(uint8_t *data,int len)
{
uint8_t     buf[6];

   if (len < 2)
      return(CMD_ERR_INVALID_LENGTH);
	if (data[1] != 0)
	{
		if (!CUC_Task_ResumeByIndex(data[0]))
			return CMD_ERR_COMMAND_FAILED;
	}
	else
	{
		if (!CUC_Task_SuspendByIndex(data[0]))
			return CMD_ERR_COMMAND_FAILED;
	}
   MakeCommandHeader(buf,CMD_DEVICE,CMD_ACK,SUB_DEVICE_HANDLE_TASK,CMD_TX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	Device Subcommand: Handle a specific Task
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_DEVICE_HANDLE_ALL_CUC_TASKS(uint8_t *data,int len)
{
uint8_t     buf[6];

   if (len < 1)
      return(CMD_ERR_INVALID_LENGTH);
	if (data[0] != 0)
	{
		if (!CUC_Task_ResumeAll())
			return CMD_ERR_COMMAND_FAILED;
	}
	else
	{
		if (!CUC_Task_SuspendAll())
			return CMD_ERR_COMMAND_FAILED;
	}
   MakeCommandHeader(buf,CMD_DEVICE,CMD_ACK,SUB_DEVICE_HANDLE_ALL_CUC_TASKS,CMD_TX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	Device Subcommand: Gets the State of a specific Task
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_DEVICE_GET_TASK_STATE(uint8_t *data,int len)
{
uint8_t     buf[9];
int			ret;

   if (len < 1)
      return(CMD_ERR_INVALID_LENGTH);
	if ((ret = CUC_Task_GetStateByIndex(data[0])) == -1)
		return CMD_ERR_COMMAND_FAILED;
	buf[6] = data[0];
	buf[7] = ret & 0xFF;
	buf[8] = (ret >> 8) & 0xFF;
   MakeCommandHeader(buf,CMD_DEVICE,CMD_ACK,SUB_DEVICE_GET_TASK_STATE,CMD_RX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	Device Subcommand: Gets the Name of a specific Task
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_DEVICE_GET_TASK_NAME(uint8_t *data,int len)
{
uint8_t     buf[24];
char			* ptr;

   if (len < 1)
      return(CMD_ERR_INVALID_LENGTH);
	if ((ptr = CUC_Task_GetNameByIndex(data[0])) == NULL)
		return CMD_ERR_COMMAND_FAILED;
	buf[6] = data[0];
	strncpy((char *)(buf + 7),ptr,17);
	buf[23] = 0;
   MakeCommandHeader(buf,CMD_DEVICE,CMD_ACK,SUB_DEVICE_GET_TASK_NAME,CMD_RX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	Device Subcommand: Gets the Number of Tasks
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_DEVICE_GET_NUMBER_OF_TASKS(uint8_t *data,int len)
{
uint8_t     buf[7];

	buf[6] = CUC_Task_GetNumberOfTasks();
   MakeCommandHeader(buf,CMD_DEVICE,CMD_ACK,SUB_DEVICE_GET_NUMBER_OF_TASKS,CMD_RX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	Device Subcommand: Initialize the EEPROM for Parameter Storing
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_DEVICE_EEPROM_INIT(uint8_t *data,int len)
{
uint8_t     buf[6];

	if (!EEPROM_InitializeEEPROM())
		return CMD_ERR_COMMAND_FAILED;
   MakeCommandHeader(buf,CMD_DEVICE,CMD_ACK,SUB_DEVICE_EEPROM_INIT,CMD_TX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	Device Subcommand: Initialize the EEPROM for Parameter Storing
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_DEVICE_CHECK_MAGIC_NUMBER(uint8_t *data,int len)
{
uint8_t     buf[7];

	if (EEPROM_CheckMagicWord())
		buf[6] = 1;
	else
		buf[6] = 0;
   MakeCommandHeader(buf,CMD_DEVICE,CMD_ACK,SUB_DEVICE_CHECK_MAGIC_NUMBER,CMD_RX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	Device Subcommand: Clear all Parameters and reset Paramter Count to zero
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_DEVICE_EEPROM_CLEAR_PARAMS(uint8_t *data,int len)
{
uint8_t     buf[6];

	if (!EEPROM_ClearParamStructCount())
		return CMD_ERR_COMMAND_FAILED;
   MakeCommandHeader(buf,CMD_DEVICE,CMD_ACK,SUB_DEVICE_EEPROM_CLEAR_PARAMS,CMD_TX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	Device Subcommand: Write The Version Structure
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_DEVICE_EEPROM_WRITE_VERSION(uint8_t *data,int len)
{
uint8_t     				buf[6];
EEPROM_UVersionEntry_t	entry;

   if (len < 32)
      return(CMD_ERR_INVALID_LENGTH);
	if (!EEPROM_MakeVersionStruct(&entry,
			data[0],data[1],data[2],
			(char *)(data + 3),
			(char *)(data + 19)))
		return CMD_ERR_COMMAND_FAILED;
	if (!EEPROM_ClearParamStructCount())
		return CMD_ERR_COMMAND_FAILED;
	if (!EEPROM_WriteVersionStruct(&entry))
		return CMD_ERR_COMMAND_FAILED;
   MakeCommandHeader(buf,CMD_DEVICE,CMD_ACK,SUB_DEVICE_EEPROM_WRITE_VERSION,CMD_TX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	Device Subcommand: Read The Version Structure 
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_DEVICE_EEPROM_READ_VERSION(uint8_t *data,int len)
{
EEPROM_UVersionEntry_t	entry;
uint8_t     				buf[6 + sizeof(entry.ByteVect)];

	if (!EEPROM_ReadVersionStruct(&entry))
		return CMD_ERR_COMMAND_FAILED;
	memcpy(buf + 6,entry.ByteVect,sizeof(entry.ByteVect));
   MakeCommandHeader(buf,CMD_DEVICE,CMD_ACK,SUB_DEVICE_EEPROM_READ_VERSION,CMD_RX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	Device Subcommand: Write A Parameter Structure
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_DEVICE_EEPROM_WRITE_PARAM(uint8_t *data,int len)
{
uint8_t     				buf[6];
EEPROM_UParamEntry_t		entry;

   if (len < 33)
      return(CMD_ERR_INVALID_LENGTH);
	if (!EEPROM_MakeParameterStruct(&entry,
			GetU16_Val(data + 3),GetU32_Val(data + 5),
			GetU64_Val(data + 9),(char *)(data + 17)))
		return CMD_ERR_COMMAND_FAILED;
	if ((int8_t)(data[0]) == -1)
	{
		if (!EEPROM_WriteNextParamStruct(&entry))
			return CMD_ERR_COMMAND_FAILED;
	}
	else
	{
		if (!EEPROM_WriteParamStruct(data[0],&entry))
			return CMD_ERR_COMMAND_FAILED;
	}
   MakeCommandHeader(buf,CMD_DEVICE,CMD_ACK,SUB_DEVICE_EEPROM_WRITE_PARAM,CMD_TX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	Device Subcommand: Read A Parameter Structure 
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_DEVICE_EEPROM_READ_PARAM(uint8_t *data,int len)
{
EEPROM_UParamEntry_t		entry;
uint8_t     				buf[6 + sizeof(entry.ByteVect)];

   if (len < 1)
      return(CMD_ERR_INVALID_LENGTH);

	if (!EEPROM_ReadParamStruct(data[0],&entry))
		return CMD_ERR_COMMAND_FAILED;
	memcpy(buf + 6,entry.ByteVect,sizeof(entry.ByteVect));
   MakeCommandHeader(buf,CMD_DEVICE,CMD_ACK,SUB_DEVICE_EEPROM_READ_PARAM,CMD_RX,BOARD_GetOwnAddress());
   SendPacketCMD(buf,sizeof(buf));
   return(CMD_OK);
}

/*!
 ******************************************************************************
 *	Device Subcommand: Get the Number of Parameter Structures stored in the
 * EEPROM
 *	\param[in]	data        parameter buffer
 *	\param[in]	len         length of paramter buffer
 * \return     CMD_OK if success, Errorcode else
 ******************************************************************************
*/
int cmd_SUB_DEVICE_EEPROM_GET_PARAM_CNT(uint8_t *data,int len)
{
uint8_t     				buf[6 + sizeof(uint16_t)];
int							n;

	if ((n = EEPROM_ReadParamStructCount()) == -1)
		return CMD_ERR_COMMAND_FAILED;
	SetVal_16(buf + 6,n);
   MakeCommandHeader(buf,CMD_DEVICE,CMD_ACK,SUB_DEVICE_EEPROM_GET_PARAM_CNT,CMD_RX,BOARD_GetOwnAddress());
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
int DeviceCommandHandler(int16_t dev_nr,uint8_t *command,int len)
{
   switch(*command)
   {
		case SUB_DEVICE_HANDLE_TASK:
			SendCommandType(CMD_TX);
			return cmd_SUB_DEVICE_HANDLE_TASK(command+1,len-1);
		case SUB_DEVICE_HANDLE_ALL_CUC_TASKS:
			SendCommandType(CMD_TX);
			return cmd_SUB_DEVICE_HANDLE_ALL_CUC_TASKS(command+1,len-1);
		case SUB_DEVICE_GET_TASK_STATE:
			SendCommandType(CMD_RX);
			return cmd_SUB_DEVICE_GET_TASK_STATE(command+1,len-1);
		case SUB_DEVICE_GET_TASK_NAME:
			SendCommandType(CMD_RX);
			return cmd_SUB_DEVICE_GET_TASK_NAME(command+1,len-1);
		case SUB_DEVICE_GET_NUMBER_OF_TASKS:
			SendCommandType(CMD_RX);
			return cmd_SUB_DEVICE_GET_NUMBER_OF_TASKS(command+1,len-1);
		case SUB_DEVICE_EEPROM_INIT:
			SendCommandType(CMD_RX);
			return cmd_SUB_DEVICE_EEPROM_INIT(command+1,len-1);
		case SUB_DEVICE_CHECK_MAGIC_NUMBER:
			SendCommandType(CMD_RX);
			return cmd_SUB_DEVICE_CHECK_MAGIC_NUMBER(command+1,len-1);
		case SUB_DEVICE_EEPROM_CLEAR_PARAMS:
			SendCommandType(CMD_TX);
			return cmd_SUB_DEVICE_EEPROM_CLEAR_PARAMS(command+1,len-1);
		case SUB_DEVICE_EEPROM_WRITE_VERSION:
			SendCommandType(CMD_TX);
			return cmd_SUB_DEVICE_EEPROM_WRITE_VERSION(command+1,len-1);
		case SUB_DEVICE_EEPROM_READ_VERSION:
			SendCommandType(CMD_RX);
			return cmd_SUB_DEVICE_EEPROM_READ_VERSION(command+1,len-1);
		case SUB_DEVICE_EEPROM_WRITE_PARAM:
			SendCommandType(CMD_TX);
			return cmd_SUB_DEVICE_EEPROM_WRITE_PARAM(command+1,len-1);
		case SUB_DEVICE_EEPROM_READ_PARAM:
			SendCommandType(CMD_RX);
			return cmd_SUB_DEVICE_EEPROM_READ_PARAM(command+1,len-1);
		case SUB_DEVICE_EEPROM_GET_PARAM_CNT:
			SendCommandType(CMD_RX);
			return cmd_SUB_DEVICE_EEPROM_GET_PARAM_CNT(command+1,len-1);
      default:
         return CMD_ERR_UNKNOWN_SUBCMD;    	// we should never get there!
   }
}
