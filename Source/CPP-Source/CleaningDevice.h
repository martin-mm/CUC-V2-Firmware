// ---------------------------------------------------------------------------
//! \package     CleaningUnit
//! \file        CleaningDevice.h
//! \brief       Defines a class for all cleaning devices
//!
//! \copyright   Copyright (C) 2011-2012 BlueBotics SA
// ----------------------------------------------------------------------------

#ifndef _CLEANINGDEVICE_H_
#define _CLEANINGDEVICE_H_

// ----------------------------------------------------------------------------
// Includes
#include "Base.h"
#include "CANIds.h"

// ----------------------------------------------------------------------------
//! \brief List the different status of a CleaningDevice.
typedef enum
{
	ECleaningDeviceStatus_Disabled = 0,
	ECleaningDeviceStatus_Initializing,
	ECleaningDeviceStatus_Stopped,
	ECleaningDeviceStatus_Starting,
	ECleaningDeviceStatus_Running,
	ECleaningDeviceStatus_Stopping,
	ECleaningDeviceStatus_Error
} ECleaningDeviceStatus;

// ----------------------------------------------------------------------------
//! \class      CleaningDevice
//! \brief      Base class for all cleaning devices
class CleaningDevice
{
public:
	CleaningDevice() : m_eStatus(ECleaningDeviceStatus_Disabled), m_DryRunEnabled(false),
							 m_IsMoving(false), m_EnableMoving(false) {}
	virtual ~CleaningDevice() {}
    //! @cond 
	HideCopyAssignCompMethods(CleaningDevice);
    //! @endcond 

public:
	virtual bool Enable(void) = 0;
	virtual bool Start(void) = 0;
	virtual bool Stop(void) = 0;
	virtual bool Disable(void) = 0;
   virtual bool IsExecutingCommand(void) { return false; }
   virtual bool IsRunning() { return (ECleaningDeviceStatus_Running == m_eStatus); }
   virtual uint32_t GetMaxCurrent(void) = 0;
   virtual void ResetMaxCurrent(void) = 0;
	virtual const char * GetDeviceName(void) { return ""; };
	virtual void EnableMoving(bool Enable) { m_EnableMoving = Enable; };
	virtual bool GetMovingState(void) { return m_IsMoving; };
	virtual bool RecoverFromError(void) { return true; };
	virtual bool IsInError(void) { return false; };
//	virtual void SetPowerState(bool OnOff) { m_PowerState = OnOff; };
	virtual void SetHold(bool OnOff) { m_Hold = OnOff; };
	ECleaningDeviceStatus GetStatus() { return m_eStatus; }
   void EnableDryRun(bool enabled) {m_DryRunEnabled = enabled;}

protected:
	ECleaningDeviceStatus m_eStatus;
   bool m_DryRunEnabled;
   bool m_IsMoving;
   bool m_EnableMoving;
//	bool m_PowerState;
	bool m_Hold;												//!< Signals that the device execution should be hold
};

#endif // _CLEANINGDEVICE_H_
