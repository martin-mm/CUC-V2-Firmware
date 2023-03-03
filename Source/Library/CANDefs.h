// ---------------------------------------------------------------------------
//! \package     ARMLibrary
//! \file        CANDefs.h
//! \brief       Common CAN definitions
//! 
//! \copyright   Copyright (C) 2011-2012 BlueBotics SA
// ----------------------------------------------------------------------------

#ifndef _CANDEFS_H_
#define _CANDEFS_H_

// ----------------------------------------------------------------------------
// Includes
extern "C"
{
//#include "rtx_can.h"
}
#include "Base.h"

// ----------------------------------------------------------------------------
//! \brief Enumeration of CAN NMT states
typedef enum
{
	ENMTState_Initialising = 0,
	ENMTState_Disconnected = 1,
	ENMTState_Connecting = 2,
	ENMTState_Preparing = 3,
	ENMTState_Stopped = 4,
	ENMTState_Operational = 5,
	ENMTState_Preoperational = 127,
	ENMTState_Unknown = 255
} ENMTState;

// ----------------------------------------------------------------------------
//! \brief Enumeration of CAN NMT commands
typedef enum
{
	ENMTCommand_Start = 0x01,
	ENMTCommand_Stop = 0x02,
	ENMTCommand_EnterPreOperational = 0x80,
	ENMTCommand_Reset =0x81,
	ENMTCommand_ResetCommunication = 0x82
} ENMTCommand;

// ----------------------------------------------------------------------------
//! \brief Enumeration of CANopen PDO message
typedef enum
{
	EPdoOrder_TxPDO1 = 0,
	EPdoOrder_RxPDO1 = 1,
	EPdoOrder_TxPDO2 = 2,
	EPdoOrder_RxPDO2 = 3,
	EPdoOrder_TxPDO3 = 4,
	EPdoOrder_RxPDO3 = 5,
	EPdoOrder_TxPDO4 = 6,
	EPdoOrder_RxPDO4 = 7
} EPdoOrder;

// ----------------------------------------------------------------------------
//! \brief Enumeration of CANopen messages base
typedef enum
{
	EMessageBase_NmtControl = 0x000,
	EMessageBase_SyncAndEmergency = 0x080,
	EMessageBase_TimeSamp = 0x100,
	EMessageBase_TxPDO1 = 0x180,
	EMessageBase_RxPDO1 = 0x200,
	EMessageBase_TxPDO2 = 0x280,
	EMessageBase_RxPDO2 = 0x300,
	EMessageBase_TxPDO3 = 0x380,
	EMessageBase_RxPDO3 = 0x400,
	EMessageBase_TxPDO4 = 0x480,
	EMessageBase_RxPDO4 = 0x500,
	EMessageBase_TxSDO = 0x580,
	EMessageBase_RxSDO = 0x600,
	EMessageBase_NmtMonitorng = 0x700
} EMessageBase;

// ----------------------------------------------------------------------------
//! \brief Enumeration of CANopen SDO command
typedef enum
{
	ESdoCommand_WriteRequest4Bytes = 0x23,
	ESdoCommand_WriteRequest2Bytes = 0x2B,
	ESdoCommand_WriteRequest1Byte = 0x2F,
	ESdoCommand_WriteResponse = 0x60,
	ESdoCommand_ReadRequest = 0x40,
	ESdoCommand_ReadResponse4Bytes = 0x43,
	ESdoCommand_ReadResponse2Bytes = 0x4B,
	ESdoCommand_ReadResponse1Byte = 0x4F,
	ESdoCommand_ErrorResponse = 0x80
} ESdoCommand;

#endif // _CANDEFS_H_
