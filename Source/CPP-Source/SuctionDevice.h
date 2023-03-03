// ---------------------------------------------------------------------------
//! \package     CleaningUnit
//! \file        SuctionDevice.h
//! \brief       Defines the class responsible for management of the suction device
//! 
//! \copyright   Copyright (C) 2011-2012 BlueBotics SA
// ----------------------------------------------------------------------------

#ifndef _SUCTIONDEVICE_H_
#define _SUCTIONDEVICE_H_

// ----------------------------------------------------------------------------
// Includes
#include "Base.h"
#include "CleaningDevice.h"
#include "MotionGenerator.h"
#include "EventSource.h"

#define	SUCTION_RAMP_SLOPE				3		// Suction Ramp Slope Numerator
#define	SUCTION_RAMP_SLOPE_DIV			1000	// Suction Ramp Slope Denominator

#define	SUCTION_PWM_DIV					10000	// PWM Divider (normalized PWM is 0 .. 1)

// ----------------------------------------------------------------------------
// Forward declaration
class MotorDriver;
class AnalogInput;

// ----------------------------------------------------------------------------
//! \class      SuctionDevice
//! \brief      Control the suction Device 
class SuctionDevice : public CleaningDevice, public IEventHandler
{
public:
	SuctionDevice(MotorDriver &_motor,int ID);
	virtual ~SuctionDevice() {}
    //! @cond 
	HideDefaultMethods(SuctionDevice);
    //! @endcond 

public:
	virtual void HandleEvent(EventSource *_pSource, uint32_t _nEventId, uint32_t _nData);
	virtual bool Enable();
	virtual bool Start();
	virtual bool Stop();
	virtual bool Disable();
   virtual uint32_t GetMaxCurrent();
   virtual void ResetMaxCurrent();
	virtual const char * GetDeviceName(void);
	bool SetPower(uint32_t _nPower /*percentage*/);
	bool SetRampSlope(unsigned Slope,unsigned SlopeDiv);

private:
	bool CheckCurrent();

private:
	MotorDriver &m_Motor;               //!< The motor controller
	int m_ID;									//!< ID of the Device
	RampGenerator m_RampGenerator;      //!< The ramp generator used to change speed smoothly
	uint32_t m_nTime;                     //!< An internal image of the current time
	uint32_t m_nPower;                    //!< The desired power of the suction device in percent
#if TRACEALYZER != 0 && TRC_SUCTION != 0
	ECleaningDeviceStatus m_OldCleaningState;
	uint32_t	m_old_ratio;
#endif
};

#endif // _SUCTIONDEVICE_H_
