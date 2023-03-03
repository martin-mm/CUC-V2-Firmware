// ----------------------------------------------------------------------------
// ARM library
//
// Defines the class managing a Battery device on the CAN bus
//
// Copyright (C) 2020 BlueBotics SA
// ----------------------------------------------------------------------------

#include <stdio.h>
#include "BatteryDevice.h"

// ----------------------------------------------------------------------------
// BatteryDevice constructor
BatteryDevice::BatteryDevice(uint8_t _nDeviceId, bool _bNMTControl, bool _bConfigurePDOs)
    : CANControlledDevice(_nDeviceId, _bNMTControl, _bConfigurePDOs)
{
	 dbgprintf("Battery Device Constructor, ID = %d ...\n",_nDeviceId);	
    m_nVoltage = 0;
    m_nSoc = 0;
    m_nStatus = 0;
    m_nTemperature = 0;
	 dbgprintf("... Battery Device Constructor done.\n");	
}

// ----------------------------------------------------------------------------
// Called to configure the device (PDO, heartbeat, etc)
bool BatteryDevice::CANConfigure(int32_t _nHeartbeatPeriod, uint8_t _masterId)
{
    m_aPdo[EPdoOrder_TxPDO1].m_nPeriod = 1000;
    return CANControlledDevice::CANConfigure(_nHeartbeatPeriod, _masterId);
}

// ----------------------------------------------------------------------------
// Called each time the CAN sync message is sent
bool BatteryDevice::OnCANSync()
{
    return true;
}

// ----------------------------------------------------------------------------
// Handle PDO
void BatteryDevice::HandlePDO(CAN_msg &_msg)
{
    if(_msg.id == EMessageBase_TxPDO1 + m_nDeviceId)
    {
        m_aPdo[EPdoOrder_TxPDO1].m_nLastTime = SystemTime::GetTime();
        m_nVoltage = *(uint16_t*)&_msg.data[2];
        m_nTemperature = *(int16_t*)&_msg.data[4];
        m_nSoc = *(uint8_t*)&_msg.data[6];
        m_nStatus = *(uint8_t*)&_msg.data[7];
    }
}
