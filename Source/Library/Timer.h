// ---------------------------------------------------------------------------
//! \package     ARMLibrary
//! \file        Timer.h
//! \brief       Defines a class encapsulating a timer 
//! 
//! \copyright   Copyright (C) 2011-2012 BlueBotics SA
// ----------------------------------------------------------------------------

#ifndef _TIMER_H_
#define _TIMER_H_

// ----------------------------------------------------------------------------
// Includes
#if defined(__cplusplus)

#include "Base.h"
#include "UCDevice.h"
#include "EventSource.h"
#include "IO.h"

// ----------------------------------------------------------------------------
//! \class      Timer
//! \brief      Encapsulate a timer
class Timer : public UCDevice, public EventSource
{
public:
	Timer(uint32_t _nTimer, uint32_t _nFrequency = 0);
    //! \cond 
	virtual ~Timer() {}
	HideDefaultMethods(Timer);
    //! \endcond 

public:
	static void DispatchInterrupt(uint32_t _nId);	// Used by interrupt handler

public:
	bool Configure(uint32_t _nPeriod /*us*/);
	bool Start();
	bool Stop();
	bool Reset(void);
	bool SetPeriod(uint32_t period);
	uint32_t GetTimerValue(void);
	bool EnableIRQ(void);
	bool DisableIRQ(void);
	uint32_t GetId() { return m_nTimerId; }

private:
	uint32_t		tmrCount;				// used to generate a 32bit-value for timer capture mode
	FTM_Type		*GetTimerBaseAddr(void);
	bool CalculateReloadValue(uint32_t period,uint16_t &prescale,uint16_t &reloadval);

private:
	static Timer * m_apTimers[2];		// Array of instantiated timers, used to dispatch interrupts
	uint8_t m_nTimerId;

friend class CaptureInput;
};

// ----------------------------------------------------------------------------
//! \class      CaptureInput
//! \brief      Encapsulate a capture input associated with a timer
//! \details    Today, this class only provides support for handling interrupts from capture inputs.
class CaptureInput : public EventSource
{
public:
	CaptureInput(Timer &_timer, uint8_t _nId);
    //! \cond 
	virtual ~CaptureInput() {}
	HideDefaultMethods(CaptureInput);
    //! \endcond 

public:
	bool Configure(EEdge _eEdge, bool _bUseInterrupt);

public:
	static void DispatchInterrupt(uint8_t _nTimerId, uint8_t _nId);	// Used by interrupt handler

private:
	Timer &m_Timer;                             //!< The associated timer
	uint8_t m_nId;                                //!< 
	static CaptureInput* m_apCaptures[2][3];	//!< Array of instantiated captures, used to dispatch interrupts
	uint8_t m_nCaptureId;	                        //!< 
};

// ----------------------------------------------------------------------------
//! \class      SystemTime
//! \brief      Basically encapsulate the system tick of the CPU
//! \details    This class provides a way to measure time in ms from the startup.
class SystemTime
{     
public:
    static void Init(uint16_t _nTickDuration);    	// Initialization: Specify the duration in ns of a system tick
    static void Update(void);                     	// Must be called periodically (every few ms)
    static uint32_t GetTime(void);						// Return the number of ms elapsed since startup
	 static bool Wait(uint32_t wait_time);				// Wait for a given time (in ms)
private:
    static uint32_t m_nTickDuration;  					// [micro seconds]
    static uint64_t m_nTime;          					// [ns]
	 static uint64_t m_uWaitTime;							// Wait Time in ms
};

extern "C" void TIMER0_IRQHandler(uint64_t value,uint32_t flags);
extern "C" void TIMER1_IRQHandler(uint64_t value,uint32_t flags);

#else

#include <stdint.h>
#include <stdbool.h>

extern void TIMER0_IRQHandler(uint64_t value,uint32_t flags);
extern void TIMER1_IRQHandler(uint64_t value,uint32_t flags);

#endif

#endif // _TIMER_H_
