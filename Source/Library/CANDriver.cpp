// ---------------------------------------------------------------------------
//! \package     ARMLibrary
//! \file        CANDriver.cpp
//! \brief       Defines the class managing a CAN controller
//! 
//! \copyright   Copyright (C) 2011-2012 BlueBotics SA
// ----------------------------------------------------------------------------

#include "CANDriver.h"
#include "CAN.h"
#include <stdio.h>

// ----------------------------------------------------------------------------
//! \brief Constructor
CANDriver::CANDriver(EDevice_t _eDevice, uint32_t _nBaudrate)
	: m_Timeout(10)
{
	dbgprintf("CAN Driver Constructor, Device = %d, Baudrate = %d ...\n",(int)_eDevice,_nBaudrate);	
	// Initialize and start CAN
	m_nController = (EDevice_CAN1 == _eDevice) ? CONTROLLER_CAN1 : CONTROLLER_CAN2;
	// CAN_init(m_nController, _nBaudrate);
	dbgprintf("... CAN Driver Constructor done.\n");	
}

// ----------------------------------------------------------------------------
//! \brief Start the CAN controller
bool CANDriver::Start()
{
//	return (CAN_OK == CAN_start(m_nController));
	return true;
}

// ----------------------------------------------------------------------------
//! \brief Set timeout value used by SendMessage calls
void CANDriver::SetTimeout(uint8_t timeout)
{
	m_Timeout = timeout;
}

// ----------------------------------------------------------------------------
//! \brief Set the device's own CAN ID
bool CANDriver::SetCANid(uint32_t ID,uint8_t type)
{
//	return false;
	return CAN_ChangeID(0,ID,type);
}

// ----------------------------------------------------------------------------
//! \brief Adds a CAN id to the CAN acceptance filters
bool CANDriver::AddCANid(uint32_t ID,bool isExtended,bool isRemote)
{
//	return false;
	return CAN_AddMessageBuffer(ID,isExtended,isRemote);
}

// ----------------------------------------------------------------------------
//! \brief Send a request to a remote node
bool CANDriver::SendRequest(uint32_t id)
{
	return CAN_RequestMessage(0,id,false);
}

// ----------------------------------------------------------------------------
//! \brief Send a 0 byte message
bool CANDriver::SendMessage(uint32_t id)
{
	return CAN_SendMessage(0,id,false,NULL,0);
}

// ----------------------------------------------------------------------------
//! \brief Send a 1 byte message
bool CANDriver::SendMessage(uint32_t id, uint8_t data)
{
	return CAN_SendMessage(0,id,false,&data,1);
}

// ----------------------------------------------------------------------------
//! \brief Send a 2 byte message
bool CANDriver::SendMessage(uint32_t id, uint16_t data)
{
	return CAN_SendMessage(0,id,false,(uint8_t *)(&data),2);
}

// ----------------------------------------------------------------------------
//! \brief Send a 4 byte message
bool CANDriver::SendMessage(uint32_t id, uint32_t data)
{
	return CAN_SendMessage(0,id,false,(uint8_t *)(&data),4);
}

// ----------------------------------------------------------------------------
//! \brief Send a 8 byte message
bool CANDriver::SendMessage(uint32_t id, uint32_t data1, uint32_t data2)
{
uint8_t		buffer[8];
	
	*(uint32_t *)(&buffer[0]) = data1;	
	*(uint32_t *)(&buffer[4]) = data2;	
	return CAN_SendMessage(0,id,false,buffer,8);
}

// ----------------------------------------------------------------------------
//! \brief Read a CAN message. Returns true if successful.
bool CANDriver::ReadMessage(CAN_msg &msg)
{
	return ReadMessage(msg, m_Timeout);
}

// ----------------------------------------------------------------------------
//! \brief Read a CAN message. Returns true if successful.
bool CANDriver::ReadMessage(CAN_msg &msg, uint16_t timeout)
{
bool		IDisExtended;
int		len;
	
	if (CAN_isRxMessageAvailable(0))
	{
		if (!CAN_getRxMessage(0,&(msg.id),&IDisExtended,msg.data,&len))
			return false;
		msg.len = len;
		msg.format = IDisExtended ? 1 : 0;
		return true;
	}
	return false;
}

uint32_t CANDriver::GetRxStatsLost()
{
	// TODO
   // return CAN_rx_stats_lost(m_nController);
	return 0;
}

uint32_t CANDriver::GetRxStatsMaxSize()
{
	// TODO
	// return CAN_rx_stats_max_size(m_nController);
	return 0;
}

void CANDriver::ResetRxStats()
{
	// TODO
	// CAN_rx_stats_reset(m_nController);
}

uint32_t CANDriver::GetTxStatsLost()
{
	// TODO
	// return CAN_tx_stats_lost(m_nController);
	return 0;
}

uint32_t CANDriver::GetTxStatsMaxSize()
{
	// TODO
	// return CAN_tx_stats_max_size(m_nController);
	return 0;
}

void CANDriver::ResetTxStats()
{
//	CAN_tx_stats_reset(m_nController);
}
