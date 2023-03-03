// ---------------------------------------------------------------------------
//! \package     ARMLibrary
//! \file        PWMDriver.h
//! \brief       Defines a small wrapper around the PWM component of the processor
//! 
//! \copyright   Copyright (C) 2011-2012 BlueBotics SA
// ----------------------------------------------------------------------------

#ifndef _PWMDRIVER_H_
#define _PWMDRIVER_H_

// ----------------------------------------------------------------------------
// Includes
#if defined(__cplusplus)

#include "Base.h"
#include "UCDevice.h"
#include "EventSource.h"
#include "EventSource.h"
#include "board.h"

// ----------------------------------------------------------------------------
// Constants
#define SMALLER_PWM_ID	1
#define LARGER_PWM_ID	(N_PWM_CONTROL_CHANNELS + 1)

typedef enum
{
	eDevTimer0 = 0,
	eDevTimer1,
} eDevTimer_t;

typedef enum
{
	eBlockDirNone = 0,
	eBlockDirLeft,
	eBlockDirRight,
	eBlockDirBoth
} eBlockDirMode_t;

// ----------------------------------------------------------------------------
// Forward declaration
class PWMOutput;
class DigitalInput;

// ----------------------------------------------------------------------------
//! \class      PWMDriver
//! \brief      Encapsulate the PWM functionality of the processor
class PWMDriver : public UCDevice,
                  public EventSource
{
public:
	PWMDriver(uint32_t _nPeriodDuration, uint32_t _nFrequency = 0); 	// Period is in us
    //! \cond 
	virtual ~PWMDriver() {}
	HideDefaultMethods(PWMDriver);
    //! \endcond 

public:
	static PWMDriver *GetInstance();
	PWMOutput* DeclareOutput(uint8_t _nId);
	PWMOutput* GetOutput(uint8_t _nId);
	void DispatchInterrupt(uint8_t channel);
	uint32_t GetCountsPerPeriod(void) { return m_nPeriodCounts; }		// ticks
//	uint32_t GetPeriodDuration(void) { return m_nPeriodDuration; }		// us
	uint32_t GetPeriodDuration(void);

private:
	static PWMDriver *m_pTheInstance;
	uint32_t m_nPeriodCounts;
	uint32_t m_nPeriodDuration;
	PWMOutput* m_apOutputs[LARGER_PWM_ID + 1];
};

// ----------------------------------------------------------------------------
//! \class      PWMOutput
//! \brief      Encapsulate a single PWM output. PWMOutput instances can be created by methods DeclareOutput or GetOutput of class PWMDriver.
class PWMOutput
{
public:
	virtual ~PWMOutput() {}
	HideDefaultMethods(PWMOutput);

private:
	PWMOutput(uint8_t _nId, uint8_t m_TimerIndex,
		DigitalInput *_pEndSWlower = nullptr,
		DigitalInput *_pEndSWupper = nullptr);
#if TRACEALYZER != 0 && TRC_PWM != 0
	traceString trcName;
#endif

public:
	bool Start();
	bool IsStarted();
	bool SetRatio(uint32_t _nNumerator, uint32_t _nDenominator, eDirMode_t DirMode);
	bool Stop();
	bool Enable();
	bool Disable();
	void Block(bool _bBlock,eBlockDirMode_t _Direction);
	bool GetStatus(void);
	unsigned GetTimerIndex(void);
	uint8_t GetBlockingState(void);
	bool IsBlocked(void);

	uint32_t GetCountsPerPeriod();
	uint32_t GetPeriodDuration();

	bool RegisterHandler(IEventHandler *_pHandler, uint32_t _nEventId);

private:
	uint8_t 				m_nId;
	uint8_t				m_PWMchannel;
	uint8_t				m_TimerIndex;
	eBlockDirMode_t 	m_BlockDirMode;
	bool					m_Block;
	char 					m_Name[16];
	eDirMode_t 			m_CurrentDir;
	DigitalInput 		*m_pEndSWlower;
	DigitalInput 		*m_pEndSWupper;

friend class PWMDriver;
};

extern "C" void PWM_IRQHandler(unsigned channel);

#else

#include <stdint.h>
#include <stdbool.h>

extern void PWM_IRQHandler(unsigned channel);

#endif

#endif // _PWMDRIVER_H_
