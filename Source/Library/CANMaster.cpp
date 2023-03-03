// ----------------------------------------------------------------------------
// ARM library
//
// Encapsulate the functionality of a CAN master node
//
//	- Manage SYNC messages
//	- Manage NMT commands (start, stop, reset, entre preop, ...)
//	- Send NMT request and handle device timeouts
//	- Manage a collection of controlled devices
//
// Copyright (C) 2012 BlueBotics SA
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Includes
#include "CANMaster.h"
#include "CANControlledDevice.h"
#include "Task_FreeRTOS.h"
#include "board.h"

// ----------------------------------------------------------------------------
// Constants
#define CAN_SYNC_TIMER_EVENT    1
#define CAN_SYNC_PERIOD         20     // ms
#define CAN_READ_PERIOD         1000   // us
#define CAN_HEARTBEATPERIOD     300    // ms

// ----------------------------------------------------------------------------
// Enumerations 
typedef enum
{
	ECANMasterEventId_Timer = 100,
} ECANMasterEventId;

// ----------------------------------------------------------------------------
// IOBoardCANDevice constructor
CANMaster::CANMaster(CANDriver &_driver, uint8_t _nCANId, osPriority_t _nPriority, uint8_t _nTimer)
	: CUC_Task(_nPriority,"CAN-MASTER"),	  
	  m_Driver(_driver),
	  m_nCANId(_nCANId),
	  m_Timer(_nTimer)
{
	dbgprintf("CAN Master Constructor, CAN ID = %d, Priority = %d ...\n",_nCANId,(int)_nPriority);	
	uint32_t now = SystemTime::GetTime();
	m_nSyncLastTime = now;
	m_nHeartbeatLastTime = now;
	dbgprintf("... CAN Node Constructor done.\n");	
}

// ----------------------------------------------------------------------------
// Register a new device to control
bool CANMaster::AddDevice(CANControlledDevice &_device)
{
	if (m_apDevices[_device.GetDeviceId()] != NULL)
	{
		return false;
	}

	m_apDevices[_device.GetDeviceId()] = &_device;
	return true;
}

// ----------------------------------------------------------------------------
// Configure the devices
bool CANMaster::ConfigureDevices()
{
	bool devicesConfigured = true;
	for (uint8_t nodeId = 0; nodeId < CANMASTER_MAX_DEVICES; nodeId++)
	{
		if (NULL != m_apDevices[nodeId])
		{
			if (m_apDevices[nodeId]->CANInit(m_Driver))
			{
				m_apDevices[nodeId]->m_bDeviceConfigured = m_apDevices[nodeId]->CANConfigure(CAN_HEARTBEATPERIOD, m_nCANId);
			}
			else
			{
				m_apDevices[nodeId]->m_bDeviceConfigured = false;
			}
			devicesConfigured &= m_apDevices[nodeId]->m_bDeviceConfigured;
		}
	}
	return devicesConfigured;
}

// ----------------------------------------------------------------------------
// Send the sync message
bool CANMaster::SendSync(uint32_t now)
{
	if ((now - m_nSyncLastTime) >= CAN_SYNC_PERIOD)
	{
		m_Driver.SendMessage(EMessageBase_SyncAndEmergency);
		m_nSyncLastTime = now;
		return true;
	}
	return false;
}

// ----------------------------------------------------------------------------
// Send the heartbeat message
bool CANMaster::SendHeartbeat(uint32_t now)
{
	if ((now - m_nHeartbeatLastTime) >= CAN_HEARTBEATPERIOD)
	{
		m_Driver.SendMessage(EMessageBase_NmtMonitorng + m_nCANId, (uint8_t) ENMTState_Operational);
		m_nHeartbeatLastTime = now;
		return true;
	}
	return false;
}

// ----------------------------------------------------------------------------
// Task listening to the CAN bus
void CANMaster::Main()
{
	// Start the CAN controller
	m_Driver.Start();

	// Configure and start the timer
	m_Timer.Configure(CAN_READ_PERIOD);
	m_Timer.RegisterHandler(this, ECANMasterEventId_Timer);
	m_Timer.Start();

	uint32_t nbSyncSent = 0;

// *****************************************
// Send SYNC
// Send NMT requests
// Check device timeouts
// *****************************************
	while (1)
	{
		uint8_t nodeId = 0;
		CANControlledDevice *node;

		// Wait for next timer event
		WaitForEvent(CAN_SYNC_TIMER_EVENT, CAN_READ_PERIOD);

		uint32_t now = SystemTime::GetTime();
		// Send Sync message
		bool syncSent = SendSync(now);
		if (syncSent)
		{
			if(nbSyncSent >= 5)
			{
				nbSyncSent = 0;
			}
			nbSyncSent++;
		}
		// Send Heartbeat message
		bool heartbeatSent = SendHeartbeat(now);
		for (nodeId = 0; nodeId < CANMASTER_MAX_DEVICES; nodeId++)
		{
			node = m_apDevices[nodeId];
			if (NULL != node)
			{
				// Send PDO only if node is operational and configured
				if ((node->m_nNMTState == ENMTState_Operational) && node->m_bDeviceConfigured)
				{
					node->SendRxPDO(now);
				}
				// Put back in operationnal node if it is not anymore
				if ((node->m_nNMTState != ENMTState_Operational) && node->m_bNMTControl && node->m_bDeviceConfigured && heartbeatSent)
				{
					node->SentNMT(ENMTCommand_Start);
				}
				// Send SDO to CUC every 5 sync as it does not support yet the PDO
				if (syncSent && (nbSyncSent >= 5))
				{
					node->OnCANSync();
				}
			}
		}
		
		// Check for nodes activity
		for (nodeId = 0; nodeId < CANMASTER_MAX_DEVICES; nodeId++)
		{
			node = m_apDevices[nodeId];
			if (NULL != node)
			{
				// Make a timeout of 2 times the heartbeat period before triggering an error
				node->m_bHeartbeatTimeout = (node->m_bNMTControl) && (node->m_nHeartbeatLastTime < (SystemTime::GetTime() - 2 * CAN_HEARTBEATPERIOD));
				for (int pdoId = 0; pdoId < 8; pdoId++)
				{
					int16_t pdoPeriod = node->m_aPdo[pdoId].m_nPeriod;
					// Make a timeout of 2 times the PDO period before triggering an error
					node->m_aPdo[pdoId].m_bTimeout = (pdoPeriod > 0) && (node->m_aPdo[pdoId].m_nLastTime < (SystemTime::GetTime() - 2 * pdoPeriod));
				}
			}
		}
		
		// Dispatch incomming messages
		CAN_msg msg;
		while (m_Driver.ReadMessage(msg))
		{
			nodeId = msg.id % 128;
			node = m_apDevices[nodeId];
			if (NULL != node)
			{
				uint32_t messageBase = msg.id - nodeId;
				switch(messageBase)
				{
					case(EMessageBase_NmtControl):
						//nothing to do as we are the master and thus only us should send NMT control messages
						break;
					case(EMessageBase_SyncAndEmergency):
						//nothing to do
						break;
					case(EMessageBase_TimeSamp):
						//nothing to do
						break;
					case(EMessageBase_TxPDO1):
					case(EMessageBase_TxPDO2):
					case(EMessageBase_TxPDO3):
					case(EMessageBase_TxPDO4):
						node->HandlePDO(msg);
						break;
					case(EMessageBase_RxPDO1):
					case(EMessageBase_RxPDO2):
					case(EMessageBase_RxPDO3):
					case(EMessageBase_RxPDO4):
						node->HandlePDO(msg);
						break;
					case(EMessageBase_TxSDO):
						node->HandleSDOAnswer(msg);
						break;
					case(EMessageBase_RxSDO):
						//nothing to do as we are the master and thus we send the RxSDO normally
						break;
					case(EMessageBase_NmtMonitorng):
						node->HandleNMTStateMachine(msg);
						break;
					default:
						break;
				}
			}
		}
	}
}

// ----------------------------------------------------------------------------
// Handle events
void CANMaster::HandleEvent(EventSource *_pSource, uint32_t _nEventId, uint32_t _nData)
{
	switch (_nEventId)
	{
		case ECANMasterEventId_Timer:
		{
			SetEvent(CAN_SYNC_TIMER_EVENT, true);
			break;
		}
		default:
			break;
	}
}
