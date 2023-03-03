// ----------------------------------------------------------------------------
// ARM library
//
// Defines the class encapsulating a controlled remote CAN device
//	- Download and upload SDO
// 	- Dispatch received SDOs
//
// Copyright (C) 2011-2012 BlueBotics SA
// ----------------------------------------------------------------------------

#include "CANControlledDevice.h"
#include "Task_FreeRTOS.h"
#include "board.h"

#define OFFSET_8BITS    8

// ----------------------------------------------------------------------------
// CANDevice constructor
CANControlledDevice::CANControlledDevice(uint8_t _nDeviceId, bool _bNMTControl, bool _bConfigurePDOs)
	: m_pDriver(NULL),
	  m_nDeviceId(_nDeviceId),
	  m_nNMTState(ENMTState_Unknown),
	  m_nNMTCounter(0),
	  m_bNMTControl(_bNMTControl),
	  m_bConfigurePdo(_bConfigurePDOs),
	  m_bDeviceConfigured(false)
{
	dbgprintf("CAN Controller Device Constructor, ID = %d ...\n",_nDeviceId);	
	m_bHeartbeatTimeout = false;
	uint64_t now = (uint32_t)(SystemTime::GetTime());
	for (int i = 0; i < 8; i++)
	{
		m_aPdo[i].m_nLastTime = now;
		m_aPdo[i].m_nPeriod = -1;
		m_aPdo[i].m_bTimeout = false;
	}
	dbgprintf("... CAN Controller Device Constructor done, Systime = %llu.\n",now);	
}

// ----------------------------------------------------------------------------
// Called when starting the CAN controller
bool CANControlledDevice::CANInit(CANDriver &_driver)
{
	m_pDriver = &_driver;
	if (m_bNMTControl)
	{
		SentNMT(ENMTCommand_Reset);
		uint64_t timeout = SystemTime::GetTime() + 2000;  // wait until the node is in pre-operational or initialising state, or until timeout occurs
		while (SystemTime::GetTime() < timeout)
		{
			if ((m_nNMTState == ENMTState_Preoperational) || (m_nNMTState == ENMTState_Initialising))
			{
				return true;
			}
			vTaskDelay(10);
		}
		return false;
	}
	return true;
}

// ----------------------------------------------------------------------------
// Called to configure the device (PDO, heartbeat, etc)
bool CANControlledDevice::CANConfigure(int32_t _nHeartbeatPeriod, uint8_t _masterId)
{
	bool _heartbeatConfigured  = true;
	if (m_bNMTControl)
	{
		_heartbeatConfigured &= DownloadSDO(0x1017, 0x00, _nHeartbeatPeriod, 4, true);                                                   //heartbeat producer time to 300 ms
		_heartbeatConfigured &= DownloadSDO(0x1016, 0x01, (((_masterId & 0xFF) << 16) | ((2 * _nHeartbeatPeriod) & 0xFFFF)), 4, true);   //heartbeat consumer time to 600 ms
		_heartbeatConfigured &= DownloadSDO(0x6007, 0x00, 0x02, 4, true);                                                                //abort connection code set to "Disable Voltage"
		SentNMT(ENMTCommand_Start);
	}

	return _heartbeatConfigured;
}

// ----------------------------------------------------------------------------
// Called each time the CAN sync message is sent
bool CANControlledDevice::OnCANSync()
{
	if (NULL != m_pDriver)
	{
		// Send request to get NMT status for devices not supporting NMTControl (CUC)
		if(!m_bNMTControl)
		{
				m_pDriver->SendRequest(EMessageBase_NmtMonitorng + m_nDeviceId);
		}
		return true;
	}
	return false;
}

// ----------------------------------------------------------------------------
// Send RxPDO to nodes
void CANControlledDevice::SendRxPDO(uint32_t _nNow)
{
	// Default implementation: nothing to do...
}


// ----------------------------------------------------------------------------
// Handle answers to SDO commands
void CANControlledDevice::HandleSDOAnswer(CAN_msg &_msg)
{
	for (int i = 0; i < 8; i++)
	{
		m_sdoAnswer.data[i] = _msg.data[i];
	}
	m_bWaitSdo = false;
}

// ----------------------------------------------------------------------------
// Handle PDO
void CANControlledDevice::HandlePDO(CAN_msg &msg)
{
	// Default implementation: nothing to do...
}


// ----------------------------------------------------------------------------
// Send a SDO to read an object
bool CANControlledDevice::UploadSDO(uint16_t _objIndex, uint8_t _subIndex, uint32_t& _data, bool _waitForAnswer)
{
	if (NULL != m_pDriver)
	{
		uint32_t cmd = 0;
		cmd |= ESdoCommand_ReadRequest;
		cmd |= _objIndex << OFFSET_8BITS;
		cmd |= _subIndex << (3 * OFFSET_8BITS);
		m_pDriver->SendMessage(EMessageBase_RxSDO + m_nDeviceId, cmd, 0x00000000);  //send the read RxSDO

		if (!_waitForAnswer)  //return true if we do not want to wait for the answer to our SDO
		{
			return true;
		}
		else  //wait until the TxSDO to our request as been received (in the CANmaster main loop)
		{
			uint64_t timeout = SystemTime::GetTime() + 500;  //wait max 500ms
			m_bWaitSdo = true;
			while (m_bWaitSdo)
			{
				if (SystemTime::GetTime() > timeout)
				{
					return false;
				}
				vTaskDelay(10);
			}

			switch (m_sdoAnswer.data[0])  //read the data
			{
				case ESdoCommand_ReadResponse1Byte:
					_data = m_sdoAnswer.data[4];
					return true;
				case ESdoCommand_ReadResponse2Bytes:
					_data = m_sdoAnswer.data[4] + (m_sdoAnswer.data[5] << OFFSET_8BITS);
					return true;
				case ESdoCommand_ReadResponse4Bytes:
					_data = m_sdoAnswer.data[4] + (m_sdoAnswer.data[5] << OFFSET_8BITS) + (m_sdoAnswer.data[6] << (2 * OFFSET_8BITS)) + (m_sdoAnswer.data[7] << (3 * OFFSET_8BITS));
					return true;
				case ESdoCommand_ErrorResponse:
					return false;
				default:
					return false;
			}
		}
	}
	return false;
}

// ----------------------------------------------------------------------------
// Send a SDO to write an object
bool CANControlledDevice::DownloadSDO(uint16_t _objIndex, uint8_t _subIndex, uint32_t _data, uint8_t _nDataLen, bool _waitForAnswer)
{
	if (NULL != m_pDriver)
	{
		if (_nDataLen > 4)
		{
			return false;
		}
		uint32_t cmd = 0;
		cmd |= ESdoCommand_WriteRequest4Bytes + ((4-_nDataLen) << 2);
		cmd |= _objIndex << OFFSET_8BITS;
		cmd |= _subIndex << (3 * OFFSET_8BITS);
		m_pDriver->SendMessage(EMessageBase_RxSDO + m_nDeviceId, cmd, _data);
		
		if (!_waitForAnswer)  //return true if we do not want to wait for the answer to our SDO
		{
			return true;
		}
		else  //wait until the TxSDO to our request as been received (in the CANmaster main loop)
		{
			uint64_t timeout = SystemTime::GetTime() + 500;  //wait max 500ms
			m_bWaitSdo = true;
			while (m_bWaitSdo)
			{
				if (SystemTime::GetTime() > timeout)
				{
					return false;
				}
				vTaskDelay(10);
			}
			if (m_sdoAnswer.data[0] == ESdoCommand_ErrorResponse)
				return false;
			return true;
		}
	}
	return false;
}


// ----------------------------------------------------------------------------
// Send the specified NMT command
void CANControlledDevice::SentNMT(ENMTCommand _cmd)
{
	if (NULL != m_pDriver)
	{
		uint16_t cmd = _cmd + (m_nDeviceId << OFFSET_8BITS);
		m_pDriver->SendMessage(0, cmd);
	}
}

// ----------------------------------------------------------------------------
// Handle the NMT state machine
void CANControlledDevice::HandleNMTStateMachine(CAN_msg &msg)
{
	m_nNMTCounter++;
	m_nNMTState = msg.data[0] & 0x7F;
	m_nHeartbeatLastTime = SystemTime::GetTime();
}

// ----------------------------------------------------------------------------
// Return true if there is a timeout on the heartbeat message or on one of the PDOs
bool CANControlledDevice::IsTimedOut()
{
	bool _timeout = m_bHeartbeatTimeout;
	for (int i = 0; i < 8; i++)
	{
		_timeout |= m_aPdo[i].m_bTimeout;
	}
	return _timeout;
}
