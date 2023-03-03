// ----------------------------------------------------------------------------
// ARM library
//
// Defines the class managing a Dunkermotoren device on the CAN bus
//
// Copyright (C) 2011-2012 BlueBotics SA
// ----------------------------------------------------------------------------

#ifndef _DUNKERMOTORENDEVICE_H_
#define _DUNKERMOTORENDEVICE_H_

#include "Base.h"
#include "CANControlledDevice.h"

#define USE_SPEED_AS_FEEDBACK           1     // Return measured speed instead of position
#define PDO_PERIOD_BASE                 20    // in ms

// ----------------------------------------------------------------------------
// Class MotorDevice
//
// Manage a Dunkermotoren device on the CAN bus
//
class DunkermotorenDevice : public CANControlledDevice
{
public:
	DunkermotorenDevice(uint8_t _nDeviceId, bool _bNMTControl, bool _bConfigurePDOs);
	virtual ~DunkermotorenDevice(void) {}
	HideDefaultMethods(DunkermotorenDevice);

public:
	void Enable(void);
	void Disable(void);
	void ClearError(void);
	bool IsEnabled(void) { return m_bEnabled; }
	bool IsOk(void) { return !m_bError; }
	void SetSpeed(uint32_t _nSpeed);
#ifdef USE_SPEED_AS_FEEDBACK
    int32_t GetSpeed() { return m_nSpeed; }
#else
    int32_t GetPosition() { return m_nPosition; }
#endif
	int32_t GetCurrent() { return m_nCurrent; }

private:
	virtual bool CANConfigure(int32_t _nHeartbeatPeriod, uint8_t _masterId);
	virtual bool OnCANSync(void);
	virtual void HandleSDOAnswer(CAN_msg &_msg);
	virtual void HandlePDO(CAN_msg &_msg);
	virtual void SendRxPDO(uint32_t _nNow);

private:
	int32_t m_nSpeedSetPoint;
#ifdef USE_SPEED_AS_FEEDBACK
   int32_t m_nSpeed;
#else
    int32_t m_nPosition;
#endif
	int32_t m_nCurrent;
	bool m_bEnabled;
	bool m_bError;
	int16_t m_nError;
};

#endif // _DUNKERMOTORENDEVICE_H_
