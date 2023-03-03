// ---------------------------------------------------------------------------
//! \package     ARMLibrary
//! \file        Timer.cpp
//! \brief       Defines a class encapsulating a timer 
//! 
//! \copyright   Copyright (C) 2011-2012 BlueBotics SA
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Includes
#include "Timer.h"
#include "base.h"
#include "board.h"
#include "fsl_ftm.h"
#include "fsl_clock.h"
#include "board.h"
// extern U16 os_time;

extern "C" void TIMER0_IRQHandler(uint64_t value,uint32_t flags);
extern "C" void TIMER1_IRQHandler(uint64_t value,uint32_t flags);

// ----------------------------------------------------------------------------
// Instantiation of static variables
Timer* Timer::m_apTimers[2];
CaptureInput* CaptureInput::m_apCaptures[2][3];
uint32_t SystemTime::m_nTickDuration;
uint64_t SystemTime::m_nTime; 
uint64_t SystemTime::m_uWaitTime; 

// ----------------------------------------------------------------------------
//! \brief Constructor
Timer::Timer(uint32_t _nTimer, uint32_t _nFrequency)
	: UCDevice(_nTimer == 0 ? EDevice_Timer0 : EDevice_Timer1, _nFrequency),
	  EventSource(4)
{
	dbgprintf("Timer Constructor, Timer ID = %d ...\n",_nTimer);	
	switch (m_eDevice)
	{
		case EDevice_Timer0: 
			m_apTimers[0] = this;
			BOARD_RegisterFTM_Callback(0,TIMER0_IRQHandler);
			break;
		case EDevice_Timer1:
			m_apTimers[1] = this;
			BOARD_RegisterFTM_Callback(1,TIMER1_IRQHandler);
			break;
		default: break;
	};
	dbgprintf("... Timer Constructor done.\n");	
}

// ----------------------------------------------------------------------------
//! \brief Returns a pointer to the correct timer register
FTM_Type		*Timer::GetTimerBaseAddr(void)
{
	switch (m_eDevice)
	{
		case EDevice_Timer0:
			return FTM1;
		case EDevice_Timer1:
			return FTM2;
		default:
			return NULL;
	}
}

// ----------------------------------------------------------------------------
//! \brief Calculate the timer reload value (period is in us)
bool Timer::CalculateReloadValue(uint32_t period,uint16_t &prescale,uint16_t &reloadval)
{
int	k = 0;
	
	uint32_t fBus = CLOCK_GetBusClkFreq();
	while ((uint64_t)fBus * period / ((1 << k) * 1000000) >= 65536)
		k++;
	if ((1 << k) > 128)
		return false;
	else
	prescale = k;
	reloadval = ((uint64_t)period * fBus) / 1000000;
	return true;
}

// ----------------------------------------------------------------------------
//! \brief Configure the timer
bool Timer::Configure(uint32_t _nPeriod /*us*/)
{
FTM_Type			*base;
ftm_config_t	config;
uint16_t			reloadval,prescale;
	
	if (UCDevice::Configure())
	{
		if ((base = GetTimerBaseAddr()) == NULL)
			return false;
		if (!CalculateReloadValue(_nPeriod,prescale,reloadval))
			return false;
		FTM_Deinit(base);
		FTM_GetDefaultConfig(&config);
		config.bdmMode = kFTM_BdmMode_3;
		config.prescale = (ftm_clock_prescale_t)prescale;
		FTM_Init(base,&config);
		FTM_SetTimerPeriod(base,reloadval);
		base->CNTIN = 0;
		if (base == FTM1)
		{
			BOARD_FTM1_enable_isr();
			BOARD_FTM1_start();
		}
		else
			if (base == FTM2)
			{
				BOARD_FTM2_enable_isr();
				BOARD_FTM2_start();
			}
		return true;
	}
	return false;
}

// ----------------------------------------------------------------------------
//! \brief Start the timer
bool Timer::Start(void)
{
FTM_Type			*base;

	if ((base = GetTimerBaseAddr()) == NULL)
		return false;
	FTM_StartTimer(base, kFTM_SystemClock);
	return true; 
}

// ----------------------------------------------------------------------------
//! \brief Stop the timer
bool Timer::Stop(void)
{
FTM_Type			*base;

	if ((base = GetTimerBaseAddr()) == NULL)
		return false;
	FTM_StopTimer(base);
	return true; 
}

// ----------------------------------------------------------------------------
//! \brief Resets the timer
bool Timer::Reset(void)
{
FTM_Type			*base;

	if ((base = GetTimerBaseAddr()) == NULL)
		return false;
	base->CNT = 0;
	tmrCount = 0;
	return true; 
}

// ----------------------------------------------------------------------------
//! \brief Sets the timer period (period is in usec)
bool Timer::SetPeriod(uint32_t period)
{
FTM_Type			*base;
ftm_config_t	config;
uint16_t			reloadval,prescale;
	
	if ((base = GetTimerBaseAddr()) == NULL)
		return false;
	if (!CalculateReloadValue(period,prescale,reloadval))
		return false;
	FTM_Deinit(base);
	FTM_GetDefaultConfig(&config);
	config.bdmMode = kFTM_BdmMode_3;
	config.prescale = (ftm_clock_prescale_t)prescale;
	FTM_Init(base,&config);
	FTM_SetTimerPeriod(base,reloadval);
	base->CNTIN = 0;
	return true;
}

// ----------------------------------------------------------------------------
//! \brief Gets the timer value
uint32_t Timer::GetTimerValue(void)
{
FTM_Type					*base;
volatile uint32_t		regval;		
uint32_t					val;

	if ((base = GetTimerBaseAddr()) == NULL)
		return 0;
	regval = base->SC;
	base->SC &= ~FTM_SC_TOIE_MASK;
	val = base->CNT;
	val += tmrCount << 16;
	if ((regval & FTM_SC_TOIE_MASK) != 0)
		base->SC |= FTM_SC_TOIE_MASK;
	return val;
}

// ----------------------------------------------------------------------------
//! \brief Enables the timer IRQ
bool Timer::EnableIRQ(void)
{
FTM_Type			*base;

	if ((base = GetTimerBaseAddr()) == NULL)
		return false;
	FTM_EnableInterrupts(base, kFTM_TimeOverflowInterruptEnable);
	return true; 
}

// ----------------------------------------------------------------------------
//! \brief Disables the timer IRQ
bool Timer::DisableIRQ(void)
{
FTM_Type			*base;

	if ((base = GetTimerBaseAddr()) == NULL)
		return false;
	FTM_DisableInterrupts(base, kFTM_TimeOverflowInterruptEnable);
	return true; 
}

// ----------------------------------------------------------------------------
//! \brief Dispatch a timer interrupt
void Timer::DispatchInterrupt(uint32_t _nId)
{
	// Make sure the timer id is within our range
	uint8_t nTimers = sizeof(m_apTimers) / sizeof(Timer *);
	if (_nId < nTimers)
	{
		// Get the Timer object associated with the interrupt
		Timer *pTimer = m_apTimers[_nId];
		if (pTimer != NULL)
		{
			pTimer->tmrCount++;	// increse the internal counter
			// Notify handlers
			pTimer->Signal();
		}
	}
}

// -- ********************************************************************** --
// -- ********************************************************************** --

// ----------------------------------------------------------------------------
//! \brief Capture constructor
CaptureInput::CaptureInput(Timer &_timer, uint8_t _nId)
	: EventSource(4),	  
	  m_Timer(_timer),
	  m_nId(_nId)
{
	dbgprintf("Capture Input Constructor, ID = %d ...\n",_nId);	
	if (m_nId > 1) return;

	switch (m_Timer.GetDevice())
	{
		case EDevice_Timer0:
			m_apCaptures[0][_nId] = this;
			break;
		case EDevice_Timer1:
			m_apCaptures[1][_nId] = this;
			break;
		default:
			break;	
	}
	dbgprintf("... Capture Input Constructor done.\n");	
}

// ----------------------------------------------------------------------------
//! \brief Configure a capture input
bool CaptureInput::Configure(EEdge _eEdge, bool _bUseInterrupt)
{
FTM_Type							*base;
ftm_input_capture_edge_t 	captureMode;

	if (m_nId > 1)
		return false;
	base = m_Timer.GetTimerBaseAddr();
	if (base != NULL)
	{
		switch (_eEdge)
		{
			case EEdge_Rising:
				captureMode = kFTM_RisingEdge;
				break;
			case EEdge_Falling:
				captureMode = kFTM_FallingEdge;
				break;
			case EEdge_Both:
				captureMode = kFTM_RiseAndFallEdge;
				break;
			default:
				return false;
		}		
		FTM_SetupInputCapture(base,(ftm_chnl_t)m_nId,captureMode,0);
		// Configure the pin
		// TODO
	}
	return false;
}

// ----------------------------------------------------------------------------
//! \brief Dispatch a capture interrupt
void CaptureInput::DispatchInterrupt(uint8_t _nTimerId, uint8_t _nId)
{
	// Make sure that timer and capture ids are within our range
	if (_nTimerId < 2 && _nId < 2)
	{
		// Get the Timer object associated with the interrupt
		CaptureInput *pCapture = m_apCaptures[_nTimerId][_nId];
		if (NULL != pCapture)
		{
			// Notify handlers
			pCapture->Signal();
		}
	}
}

// ----------------------------------------------------------------------------
//! \brief Initialization: Specify the duration in ns of a system tick.
void SystemTime::Init(uint16_t _nTickDuration)
{
	// This is fixed, so the input value is not used
   m_nTickDuration = 1000000000 / CLOCK_GetBusClkFreq();
	m_uWaitTime = 0;
}

// ----------------------------------------------------------------------------
//! \brief Update our internal counter. This method must be called periodically (every few ms)
void SystemTime::Update(void)
{
	// nothing to do here
}

// ----------------------------------------------------------------------------
//! \brief Gets the System Time in ms
uint32_t SystemTime::GetTime(void)
{
uint64_t		system_time;
	
	if (!BOARD_getSystemTime(&system_time))
		return 0;
	m_nTime = system_time;
	return system_time / 1000000;		// System Time in ms
}

// ----------------------------------------------------------------------------
//! \brief Waits for the specified wait time (in ms)
bool SystemTime::Wait(uint32_t wait_time)
{
uint64_t				uSystemTime;
	
	if (wait_time != 0)
	{
		BOARD_getSystemTime(&uSystemTime);
		m_uWaitTime = uSystemTime + (uint64_t)wait_time * 1000000;
		return true;
	}
	else
	{
		BOARD_getSystemTime(&uSystemTime);
		return (uSystemTime >= m_uWaitTime);
	}
}

// ----------------------------------------------------------------------------
//! \brief Interrupt handler for Timer 0
extern "C" void TIMER0_IRQHandler(uint64_t value,uint32_t flags)
{
	if ((flags & (1U << 31)) != 0)		// Timer Match Interrupt
	{
		Timer::DispatchInterrupt(0);
	}	
	if ((flags & (1U << 0)) != 0)		// Capture Channel 0 interrupt
	{
		CaptureInput::DispatchInterrupt(0, 0);
	}
	if ((flags & (1U << 1)) != 0)		// Capture Channel 1 interrupt
	{
		CaptureInput::DispatchInterrupt(0, 1);
	}
	if ((flags & (1U << 2)) != 0)		// Capture Channel 2 interrupt
	{
		CaptureInput::DispatchInterrupt(0, 2);
	}
}

// ----------------------------------------------------------------------------
//! \brief Interrupt handler for Timer 1
extern "C" void TIMER1_IRQHandler(uint64_t value,uint32_t flags)
{
	if ((flags & (1U << 31)) != 0)		// Timer Match Interrupt
	{
		Timer::DispatchInterrupt(1);
	}	
	if ((flags & (1U << 0)) != 0)		// Capture Channel 0 interrupt
	{
		CaptureInput::DispatchInterrupt(1, 0);
	}
	if ((flags & (1U << 1)) != 0)		// Capture Channel 1 interrupt
	{
		CaptureInput::DispatchInterrupt(1, 1);
	}
	if ((flags & (1U << 2)) != 0)		// Capture Channel 2 interrupt
	{
		CaptureInput::DispatchInterrupt(1, 2);
	}
}
