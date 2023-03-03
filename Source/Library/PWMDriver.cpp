// ---------------------------------------------------------------------------
//! \package     ARMLibrary
//! \file        PWMDriver.cpp
//! \brief       Defines a small wrapper around the PWM component of the processor
//! 
//! \copyright   Copyright (C) 2011-2012 BlueBotics SA
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Includes
#include "PWMDriver.h"
#include "board.h"

#if (TRACEALYZER != 0) && (TRC_PWM != 0)
static traceString 				trcPWM = nullptr;
#endif

extern "C" void PWM_IRQHandler(unsigned channel);		// forward declaration

// ----------------------------------------------------------------------------
// Static member variables
PWMDriver *PWMDriver::m_pTheInstance = nullptr;

// ----------------------------------------------------------------------------
//! \brief Constructor
PWMDriver::PWMDriver(uint32_t _nPeriodDuration, uint32_t _nFrequency)		// Period is in us
	: UCDevice(EDevice_PWM, _nFrequency),
	  EventSource(2 * LARGER_PWM_ID),
	  m_nPeriodCounts(0),
	  m_nPeriodDuration(0)
	  
{
uint32_t		period;
int			frequency;
	
	dbgprintf("PWM Driver Constructor, Period = %d us\n",_nPeriodDuration);	
	// Can be created only once! 
	if (m_pTheInstance == nullptr)
	{
		dbgprintf("PWM Driver, initial setup ...\n");	
		frequency = 1000000 / _nPeriodDuration;
		BOARD_Set_PWM_FTM_Frequency(0,frequency);
		BOARD_Set_PWM_FTM_Frequency(1,frequency);
		if (BOARD_GetPWM_CounterPeriod(0,&period))
		{
			m_nPeriodCounts = period;
			frequency = BOARD_Get_PWM_FTM_Frequency(0);
			m_nPeriodDuration = BOARD_Get_PWM_FTM_Period(0);
			dbgprintf("PWM Driver, Frequency = %d Hz, Period Counts = %d\n",frequency,m_nPeriodCounts);	
			dbgprintf("PWM Driver, Period Duration = %d us\n",m_nPeriodDuration);	
		}
		for (int i = 0; i <= LARGER_PWM_ID; i++)
			m_apOutputs[i] = nullptr;
		m_pTheInstance = this;
		Configure();

		// Clear all channels
		for (int i = 0;i < N_PWM_CONTROL_CHANNELS;i++)
			BOARD_SetPWMControl(i,0,eDirModeBrake);
		// Register the timer overflow interrupt handler
		BOARD_RegisterPWM_FTM_Callback(PWM_IRQHandler);
		BOARD_FTM0_enable_isr();
		BOARD_FTM3_enable_isr();
		dbgprintf("... PWM Driver, initial setup done.\n");	
	}
	dbgprintf("... PWM Driver Constructor done\n");	
}

uint32_t PWMDriver::GetPeriodDuration(void)
{
	return m_nPeriodDuration;
}

//! \brief Interrupt Dispatcher
void PWMDriver::DispatchInterrupt(uint8_t channel)
{
	Signal(channel);
}

// ----------------------------------------------------------------------------
//! \brief Return the single instance of this class
PWMDriver *PWMDriver::GetInstance()
{
	return m_pTheInstance;
}

// ----------------------------------------------------------------------------
//! \brief Declare one of the PWM channel
PWMOutput* PWMDriver::DeclareOutput(uint8_t _nId)
{
uint8_t			TimerIndex;
	
	PWMOutput *pOutput = GetOutput(_nId);
	if (pOutput == NULL &&
	    (_nId >= SMALLER_PWM_ID && _nId <= LARGER_PWM_ID))
	{
		m_apOutputs[_nId] = pOutput;
		if (!BOARD_GetPWMTimer(_nId - 1,&TimerIndex))
		{
			TimerIndex = 0;		// TODO Error Handling
		}
		pOutput = new PWMOutput(_nId,TimerIndex);
	}
	return pOutput;
}

// ----------------------------------------------------------------------------
//! \brief Return the specified PWMOutput object
PWMOutput* PWMDriver::GetOutput(uint8_t _nId)
{
	PWMOutput *pOutput = NULL;
	if (_nId >= SMALLER_PWM_ID && _nId <= LARGER_PWM_ID)
	{
		pOutput = m_apOutputs[_nId];
	}
	return pOutput;
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
//! \brief Constructor
PWMOutput::PWMOutput(uint8_t _nId,uint8_t m_TimerIndex,DigitalInput *_pEndSWlower,DigitalInput *_pEndSWupper) : 
		m_nId(_nId),
		m_PWMchannel(_nId - 1),
		m_TimerIndex(m_TimerIndex),
	   m_CurrentDir(eDirModeBrake),
		m_pEndSWlower(_pEndSWlower),
		m_pEndSWupper(_pEndSWupper)
{
const char *	str;
	
	m_Block = false;
	str = BOARD_GetPWMName(m_PWMchannel);
	if (str != nullptr)
		strcpy(m_Name,str);
	else
		m_Name[0] = '\0';
	dbgprintf("PWM Output Constructor, ID = %d, Timer Index = %d, Name = %s\n",
		_nId,m_TimerIndex,str != nullptr ? str : "");
#if TRACEALYZER != 0 && TRC_PWM != 0
	if (trcPWM != nullptr)
		trcPWM = xTraceRegisterString("PWM");
	trcName = xTraceRegisterString(m_Name);
#endif
}

// ----------------------------------------------------------------------------
//! \brief Get the Timer Index of the Timer controlling the output
unsigned PWMOutput::GetTimerIndex(void)
{
	return m_TimerIndex;
}

// ----------------------------------------------------------------------------
//! \brief Enable one of the PWM channels (enable output)
bool PWMOutput::Enable(void)
{
	return BOARD_EnablePWMchannel(m_PWMchannel);
}

// ----------------------------------------------------------------------------
//! \brief Disable one of the PWM channels (disable output)
bool PWMOutput::Disable(void)
{
	return BOARD_DisablePWMchannel(m_PWMchannel);
}

// ----------------------------------------------------------------------------
//! \brief Start one of the PWM channels (enable output)
bool PWMOutput::Start(void)
{
	return BOARD_EnablePWMchannel(m_PWMchannel);
}

// ----------------------------------------------------------------------------
//! \brief Stop one of the PWM channels (disable output)
bool PWMOutput::Stop()
{
	// Stop PWM
	return BOARD_DisablePWMchannel(m_PWMchannel);
}

// ----------------------------------------------------------------------------
//! \brief Blocks the PWM in the desired direction
void PWMOutput::Block(bool _bBlock,eBlockDirMode_t _Direction)
{
eDirMode_t		actualDir;
	
	if (_bBlock)
	{	
		if (BOARD_GetPWMdirection(m_PWMchannel,&actualDir))
		{
			if (_Direction == eBlockDirLeft && actualDir == eDirModeLeft)
			{
#if TRACEALYZER != 0 && TRC_PWM != 0
				vTracePrintF(trcPWM,"%s - Block Dir Left",trcName);
#endif
				m_Block = true;
				m_BlockDirMode = eBlockDirLeft;
				SetRatio(0,1,eDirModeLeft);
			}
			else
			{
				if (_Direction == eBlockDirRight && actualDir == eDirModeRight)
				{
#if TRACEALYZER != 0 && TRC_PWM != 0
					vTracePrintF(trcPWM,"%s - Block Dir Right",trcName);
#endif
					m_Block = true;
					m_BlockDirMode = eBlockDirRight;
					SetRatio(0,1,eDirModeRight);
				}
				else
				{
					if (_Direction == eBlockDirBoth)
					{
#if TRACEALYZER != 0 && TRC_PWM != 0
						vTracePrintF(trcPWM,"%s - Block Dir Right",trcName);
#endif
						m_Block = true;
						m_BlockDirMode = eBlockDirBoth;
						SetRatio(0,1,eDirModeRight);
					}
					else
					{
#if TRACEALYZER != 0 && TRC_PWM != 0
						vTracePrintF(trcPWM,"%s - Block Dir None");
#endif
						m_Block = false;
						m_BlockDirMode = eBlockDirNone;
					}
				}
			}
		}
	}
	else
	{
		m_Block = false;
		m_BlockDirMode = eBlockDirNone;
	}
}

// ----------------------------------------------------------------------------
//! \brief Return true if blocked
uint8_t PWMOutput::GetBlockingState(void)
{
uint8_t	blState;
	
	blState = m_Block ? 1 : 0;
	blState += (m_BlockDirMode & 7) << 1; 
	blState += (m_CurrentDir & 7) << 4; 
	blState += IsBlocked() ? (1 << 7) : 0;
	return blState;
}

// ----------------------------------------------------------------------------
//! \brief Return true if blocked
bool PWMOutput::IsBlocked(void)
{
	if (m_Block && ((m_BlockDirMode == eBlockDirLeft && m_CurrentDir == eDirModeLeft) || 
			(m_BlockDirMode == eBlockDirRight && m_CurrentDir == eDirModeRight)))
		return true;
	else
		return false;
}

// ----------------------------------------------------------------------------
//! \brief Return true if started
bool PWMOutput::IsStarted(void)
{
uint8_t	status;
	
	if (!BOARD_GetPWMchannelStatus(m_PWMchannel,&status))
		return false;
	return ((status & (1 << 0)) != 0);
}

// ----------------------------------------------------------------------------
//! \brief Set the ratio of one of the PWM channels
bool PWMOutput::SetRatio(uint32_t _nNumerator, uint32_t _nDenominator, eDirMode_t DirMode)
{
uint32_t 		pwm = (1000 * _nNumerator) / _nDenominator;		// PWM is expected in %%
	
	m_CurrentDir = DirMode;
	if (m_Block)
	{	
		if (m_BlockDirMode == eBlockDirLeft && DirMode == eDirModeLeft)
		{
			pwm = 0;
		}
		else
			if (m_BlockDirMode == eBlockDirRight && DirMode == eDirModeRight)
			{
				pwm = 0;
			}
	}
	return BOARD_SetPWMControl(m_PWMchannel,pwm,DirMode);
}

// ----------------------------------------------------------------------------
//! \brief Get the Status of the PWM channels
bool PWMOutput::GetStatus(void)
{
uint32_t		status;
	
	if (!BOARD_GetPWMStatus(m_PWMchannel,&status))
		return false;
	return ((status & 1) == 0);
}

// ----------------------------------------------------------------------------
//! \brief Return the number of uc ticks per PWM period
uint32_t PWMOutput::GetCountsPerPeriod()
{
	PWMDriver *pPWMDriver = PWMDriver::GetInstance();
	if (pPWMDriver != NULL)
		return pPWMDriver->GetCountsPerPeriod();
	else
		return 0;
}

// ----------------------------------------------------------------------------
//! \brief Return the duration in micro seconds of a PWM period
uint32_t PWMOutput::GetPeriodDuration()
{
	PWMDriver *pPWMDriver = PWMDriver::GetInstance();
	if (pPWMDriver != NULL)
		return pPWMDriver->GetPeriodDuration();
	else
		return 0;
}

// ----------------------------------------------------------------------------
//! \brief Forward to PWMDriver::RegisterHandler
bool PWMOutput::RegisterHandler(IEventHandler *_pHandler, uint32_t _nEventId)
{
	PWMDriver *pPWMDriver = PWMDriver::GetInstance();
	if (NULL != pPWMDriver)
	{
		return pPWMDriver->RegisterHandler(_pHandler, _nEventId);
	}
	return false;
}

// ----------------------------------------------------------------------------
//! \brief PWM input interrupt handler...
extern "C" void PWM_IRQHandler(unsigned channel)
{
	// TODO: Call of this from interrupt
	PWMDriver *pPWMDriver = PWMDriver::GetInstance();
	if (NULL != pPWMDriver)
	{
		pPWMDriver->DispatchInterrupt(channel);
	}
}
