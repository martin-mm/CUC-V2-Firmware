// ----------------------------------------------------------------------------
// ARM library
//
// Encapsulate the functionality of a CAN master node
//
// Copyright (C) 2012 BlueBotics SA
// ----------------------------------------------------------------------------

#ifndef _CANMASTER_H_
#define _CANMASTER_H_

// ----------------------------------------------------------------------------
// Includes
#include "cmsis_os2.h"
#include "CANDefs.h"
#include "CANDriver.h"
#include "CANNode.h"
#include "Timer.h"
#include "base.h"

// ----------------------------------------------------------------------------
// Constants
#define CANMASTER_MAX_DEVICES	128

// ----------------------------------------------------------------------------
// Forward declarations
class CANControlledDevice;

// ----------------------------------------------------------------------------
// Class CANMaster
//
// Encapsulate the functionality of a CAN master node
//
class CANMaster : public CUC_Task,
				  public IEventHandler
{
public:
	CANMaster(CANDriver &_driver, uint8_t _nCANId, osPriority_t _nPriority, uint8_t _nTimer);
	virtual ~CANMaster() {}
	HideDefaultMethods(CANMaster);

public:
	bool AddDevice(CANControlledDevice &_device);
	bool ConfigureDevices();
	virtual void Main();
	virtual void HandleEvent(EventSource *_pSource, uint32_t _nEventId, uint32_t _nData);

private:
	bool SendSync(uint32_t now);
	bool SendHeartbeat(uint32_t now);

private:
	CANDriver &m_Driver;
	uint8_t m_nCANId;
	Timer m_Timer;
	CANControlledDevice *m_apDevices[CANMASTER_MAX_DEVICES];
	uint32_t m_nSyncLastTime;
	uint32_t m_nHeartbeatLastTime;
};

#endif // _CANMASTER_H_
