// ---------------------------------------------------------------------------
//! \package     CleaningUnit
//! \file        BrushDevice.h
//! \brief       Defines the class responsible for management of the brush device
//! 
//! \copyright   Copyright (C) 2011-2012 BlueBotics SA
// ----------------------------------------------------------------------------

#ifndef _BRUSHDEVICE_H_
#define _BRUSHDEVICE_H_

// ----------------------------------------------------------------------------
// Includes
#include "Base.h"
#include "CleaningDevice.h"
#include "MotionGenerator.h"
#include "EventSource.h"
#include "IO.h"

#define	BRUSH_RAMP_SLOPE					1		// Brush Ramp Slope Numerator
#define	BRUSH_RAMP_SLOPE_DIV				100	// Brush Ramp Slope Denominator

#define	BRUSH_PWM_DIV						10000	// PWM Divider (normalized PWM is 0 .. 1)

// ----------------------------------------------------------------------------
// Forward declaration
class PWMOutput;
class AnalogInput;
class MotorDriver;

// ----------------------------------------------------------------------------
//! \class      BrushDevice
//! \brief      Control de brush device
class BrushDevice : public CleaningDevice, public IEventHandler
{
public:
	BrushDevice(MotorDriver &_motor,int ID);
	virtual ~BrushDevice() {}
    //! @cond 
	HideDefaultMethods(BrushDevice);
    //! @endcond 

public:
	virtual void HandleEvent(EventSource *_pSource, uint32_t _nEventId, uint32_t _nData);
	virtual bool Enable();
	virtual bool Start();
	virtual bool Stop();
	virtual bool Disable();	
   virtual uint32_t GetMaxCurrent();
	virtual void ResetMaxCurrent();
	virtual const char *GetDeviceName(void);
	bool SetRampSlope(unsigned Slope,unsigned SlopeDiv);

private:
	bool CheckCurrent();

private:
	MotorDriver &m_Motor;               //!< The motor controller
	int m_ID;									//!< ID of the Device
	RampGenerator m_RampGenerator;      //!< The ramp generator used to change speed smoothly
	uint32_t m_nTime;                   //!< An internal image of the current time
#if TRACEALYZER != 0 && TRC_BRUSH != 0
	ECleaningDeviceStatus m_OldCleaningState;
	uint32_t	m_old_ratio;
#endif
};

#endif // _BRUSHDEVICE_H_
