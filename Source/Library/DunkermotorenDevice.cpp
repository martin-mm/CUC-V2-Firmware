// ----------------------------------------------------------------------------
// ARM library
//
// Defines the class managing a Dunkermotoren device on the CAN bus
//
// Copyright (C) 2011-2012 BlueBotics SA
// ----------------------------------------------------------------------------

#include <stdio.h>
#include "DunkermotorenDevice.h"

// ----------------------------------------------------------------------------
// Constants
#define COB_COMMAND_ID						0x3000
#define COB_COMMAND_SUB						0
#define COB_COMMAND_LEN		   			4
#define COMMAND_CLEAR_ERROR	   		1
#define COB_STATUS_ENABLEDMASK 			0x01
#define COB_STATUS_ERRORMASK   			0x02
#define COB_ENABLE_ID		   			0x3004
#define COB_ENABLE_SUB		   			0x00
#define COB_ENABLE_LEN		   			4
#define COB_ENABLE_ON		   			1
#define COB_ENABLE_OFF		   			0
#define COB_SET_SPEED_ID	  				0x3300
#define COB_SET_SPEED_SUB	  				0x00
#define COB_SET_SPEED_LEN	  				4

// ----------------------------------------------------------------------------
// MotorDevice constructor
DunkermotorenDevice::DunkermotorenDevice(uint8_t _nDeviceId, bool _bNMTControl, bool _bConfigurePDOs)
	: CANControlledDevice(_nDeviceId, _bNMTControl, _bConfigurePDOs)
{
	dbgprintf("Dunkermotor Device Constructor, Device ID = %d ...\n",_nDeviceId);	
	dbgprintf("... Dunkermotor Device Constructor done.\n");	
}

// ----------------------------------------------------------------------------
// Called to configure the device (PDO, heartbeat, etc)
bool DunkermotorenDevice::CANConfigure(int32_t _nHeartbeatPeriod, uint8_t _masterId)
{
	bool configSuccess = true;
	configSuccess &= DownloadSDO(0x3A02, 0x00, 0x00000014, 4, true);                           	//Set measurement velocity period to 20 ms
	configSuccess &= DownloadSDO(0x3003, 0x00, 3, 4, true);                                    	//Set device mode to velocity mode
	configSuccess &= DownloadSDO(0x334C, 0x00, 1, 4, true);                                    	//Set ramp generator to trapezium
	configSuccess &= DownloadSDO(0x3340, 0x00, 1000, 4, true);                                 	//Set velocity accel - deltaV to 5000 rpm
	configSuccess &= DownloadSDO(0x3341, 0x00, 300, 4, true);                                  	//Set velocity accel - deltaT to 1000 ms
	configSuccess &= DownloadSDO(0x3342, 0x00, 1000, 4, true);                                 	//Set velocity decel - deltaV to 5000 rpm
	configSuccess &= DownloadSDO(0x3343, 0x00, 300, 4, true);                                  	//Set velocity decel - deltaT to 1000 ms
	configSuccess &= DownloadSDO(0x3090, 0x00, 5000, 4, true);                                 	//Set minimum voltage for op to 0 mV

	configSuccess &= DownloadSDO(COB_COMMAND_ID, COB_COMMAND_SUB, COMMAND_CLEAR_ERROR, COB_COMMAND_LEN, true);  //clear error

	if (m_bConfigurePdo)
	{
		uint32_t data;
		configSuccess &= UploadSDO(0x1400, 0x01, data, true);                                      //read RxPDO1 COB-ID
		configSuccess &= DownloadSDO(0x1400, 0x01, (data | 0x80000000), 4, true);                  //RxPDO1 COB-ID to (value read | 0x80000000)
		configSuccess &= DownloadSDO(0x1400, 0x02, 0xFF, 4, true);                                 //RxPDO transmission type to event-driven
		configSuccess &= DownloadSDO(0x1600, 0x00, 0x00, 4, true);                                 //RxPDO number mapped object 0
		configSuccess &= DownloadSDO(0x1600, 0x01, 0x33000020, 4, true);                           //RxPDO 1st mapped object: velocity - desired value
		configSuccess &= DownloadSDO(0x1600, 0x00, 0x01, 4, true);                                 //RxPDO number mapped object 1
		configSuccess &= UploadSDO(0x1400, 0x01, data, true);                                      //read RxPDO1 COB-ID
		configSuccess &= DownloadSDO(0x1400, 0x01, ((data & 0x7FFFFFFF) | (EMessageBase_RxPDO1 + m_nDeviceId)), 4, true);  //RxPDO1 COB-ID to (value read & 0x7FFFFFFF)
		m_aPdo[EPdoOrder_RxPDO1].m_nPeriod = PDO_PERIOD_BASE;

		configSuccess &= UploadSDO(0x1800, 0x01, data, true);                                      //read TxPDO1 COB-ID
		configSuccess &= DownloadSDO(0x1800, 0x01, (data | 0x80000000), 4, true);                  //TxPDO1 COB-ID to (value read | 0x80000000)
		configSuccess &= DownloadSDO(0x1800, 0x02, 0x01, 4, true);                                 //TxPDO1 transmission type to synchronous on sync
		configSuccess &= DownloadSDO(0x1A00, 0x00, 0x00, 4, true);                                 //TxPDO1 number of mapped object 0
		configSuccess &= DownloadSDO(0x1A00, 0x01, 0x30020020, 4, true);                           //TxPDO1 1st mapped object: status register
		configSuccess &= DownloadSDO(0x1A00, 0x02, 0x3A040120, 4, true);                           //TxPDO1 2nd mapped object: measured velocity in rpm
		configSuccess &= DownloadSDO(0x1A00, 0x00, 0x02, 4, true);                                 //TxPDO1 number of mapped object 2
		configSuccess &= UploadSDO(0x1800, 0x01, data, true);                                      //read TxPDO1 COB-ID
		configSuccess &= DownloadSDO(0x1800, 0x01, ((data & 0x7FFFFFFF) | (EMessageBase_TxPDO1 + m_nDeviceId)), 4, true);  //TxPDO1 COB-ID to (value read & 0x7FFFFFFF)
		m_aPdo[EPdoOrder_TxPDO1].m_nPeriod = PDO_PERIOD_BASE;

		configSuccess &= UploadSDO(0x1801, 0x01, data, true);                                      //read TxPDO2 COB-ID
		configSuccess &= DownloadSDO(0x1801, 0x01, (data | 0x80000000), 4, true);                  //TxPDO2 COB-ID to (value read | 0x80000000)
		configSuccess &= DownloadSDO(0x1801, 0x02, 0x05, 4, true);                                 //TxPDO2 transmission type to synchronous on 1/5 sync
		configSuccess &= DownloadSDO(0x1A01, 0x00, 0x00, 4, true);                                 //TxPDO2 number of mapped object 0
		configSuccess &= DownloadSDO(0x1A01, 0x01, 0x30010010, 4, true);                           //TxPDO2 1st mapped object: error register
		configSuccess &= DownloadSDO(0x1A01, 0x02, 0x32620020, 4, true);                           //TxPDO2 2nd mapped object: actual current
		configSuccess &= DownloadSDO(0x1A01, 0x00, 0x02, 4, true);                                 //TxPDO2 number of mapped object 2
		configSuccess &= UploadSDO(0x1801, 0x01, data, true);                                      //read TxPDO2 COB-ID
		configSuccess &= DownloadSDO(0x1801, 0x01, ((data & 0x7FFFFFFF) | (EMessageBase_TxPDO2 + m_nDeviceId)), 4, true);  //TxPDO2 COB-ID to (value read & 0x7FFFFFFF)
		m_aPdo[EPdoOrder_TxPDO2].m_nPeriod = 5 * PDO_PERIOD_BASE;
	}
	return (configSuccess & CANControlledDevice::CANConfigure(_nHeartbeatPeriod, _masterId));
}

// ----------------------------------------------------------------------------
// Called each time the sync message is sent
bool DunkermotorenDevice::OnCANSync(void)
{
	return CANControlledDevice::OnCANSync();
}

// ----------------------------------------------------------------------------
// Handle answers to SDO commands
void DunkermotorenDevice::HandleSDOAnswer(CAN_msg &_msg)
{
	CANControlledDevice::HandleSDOAnswer(_msg);
}

// ----------------------------------------------------------------------------
// Handle reception of TxPDO
void DunkermotorenDevice::HandlePDO(CAN_msg &_msg)
{
	if(_msg.id == EMessageBase_TxPDO1 + m_nDeviceId)
	{	
		m_aPdo[EPdoOrder_TxPDO1].m_nLastTime = (uint32_t)(SystemTime::GetTime());
		uint32_t nStatus = *(uint16_t*)&_msg.data[0];
		m_bEnabled = ((nStatus & COB_STATUS_ENABLEDMASK) != 0);
		m_bError = ((nStatus & COB_STATUS_ERRORMASK) != 0);
		if (m_bError)
		{
			ClearError();
		}
		
		m_nSpeed = *(uint32_t*)&_msg.data[4];
	}
	
	if(_msg.id == EMessageBase_TxPDO2 + m_nDeviceId)
	{	
		m_aPdo[EPdoOrder_TxPDO2].m_nLastTime = (uint32_t)(SystemTime::GetTime());
		m_nError = *(uint16_t*)&_msg.data[0];
		m_nCurrent = *(uint32_t*)&_msg.data[2];
	}
}

// ----------------------------------------------------------------------------
// Handle sending of RxPDO
void DunkermotorenDevice::SendRxPDO(uint64_t _nNow)
{
	if (NULL != m_pDriver)
	{
		if ((_nNow - m_aPdo[EPdoOrder_RxPDO1].m_nLastTime) >= m_aPdo[EPdoOrder_RxPDO1].m_nPeriod)
		{
			uint32_t data = m_nSpeedSetPoint;
			m_pDriver->SendMessage(EMessageBase_RxPDO1 + m_nDeviceId, data);
			m_aPdo[EPdoOrder_RxPDO1].m_nLastTime = _nNow;
		}	
	}
}

// ----------------------------------------------------------------------------
// Called to enable the drive
void DunkermotorenDevice::Enable(void)
{
	if (!m_bEnabled)
	{	
		DownloadSDO(COB_ENABLE_ID, COB_ENABLE_SUB, COB_ENABLE_ON, COB_ENABLE_LEN, false);			// Enable device
	}
}

// ----------------------------------------------------------------------------
// Called to disable the drive
void DunkermotorenDevice::Disable(void)
{
	DownloadSDO(COB_SET_SPEED_ID, COB_SET_SPEED_SUB, 0, COB_SET_SPEED_LEN, false);		// Set speed to 0
	DownloadSDO(COB_ENABLE_ID, COB_ENABLE_SUB, COB_ENABLE_OFF, COB_ENABLE_LEN, false);	// Disable device
}

// ----------------------------------------------------------------------------
// Clear error if possible
void DunkermotorenDevice::ClearError(void)
{
	DownloadSDO(COB_COMMAND_ID, COB_COMMAND_SUB, COMMAND_CLEAR_ERROR, COB_COMMAND_LEN, false);	// Clear error	
}

// ----------------------------------------------------------------------------
// Called to set the motor speed
void DunkermotorenDevice::SetSpeed(uint32_t _nSpeed)
{
	m_nSpeedSetPoint = _nSpeed;
}
