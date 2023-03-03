// ---------------------------------------------------------------------------
//! \package     CleaningUnit
//! \file        MotorDriver.cpp
//! \brief       Defines classes responsible for the management of a one of the PWM motors
//! \details     All motors controlled by the cleaning unit board are driven by a VNH3SP30-E chip from STMicroelectronics.
//! \copyright   Copyright (C) 2011-2012 BlueBotics SA
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Includes
#include "PWMDriver.h"
#include "AnalogInput.h"
#include "IO.h"
#include "MotorDriver.h"
#include <stdlib.h>

#if (TRACEALYZER != 0) && (TRC_MOTOR != 0)
static traceString 				trcMotor;
#endif

#define DBGPRINTF_MOTOR
#undef DBGPRINTF_MOTOR

#define TEST_ENDSWITCH
//#undef TEST_ENDSWITCH

//#ifdef TEST_ENDSWITCH
//#define ENDSWITCH_BLOCKED_MODE	EMotorDriverMode_Clockwise
//#endif

// ----------------------------------------------------------------------------
//! \brief List event ids supported by MotorDriver.
typedef enum
{
	EMotorDriverEventId_PWM = 100,
	EMotorDriverEventId_ENDSW_L,
	EMotorDriverEventId_ENDSW_U,
} EMotorDriverEventId;

// ----------------------------------------------------------------------------
//! \brief Constructor
/*
	_nADCgain is the gain of the VoltageDivider * the gain of the ADC. It gives
		the relation between ADC-Values and current (in digits / A)
*/
CurrentProbe::CurrentProbe(AnalogInput *_pCurrentFB, int32_t _nADCgain)
	: m_pCurrentFB(_pCurrentFB),
	  m_nADCgain(_nADCgain),
	  m_nCurrent(0)				// added since this was not initialized
{
   dbgprintf("Current Probe Constructor ...\n");	
	m_nOffset = m_pCurrentFB->GetOffset();
   dbgprintf("   Current Probe Offset = %d, Gain = %d\n",m_nOffset,m_nADCgain);	
   dbgprintf("... Current Probe Constructor done.\n");	
}

// ----------------------------------------------------------------------------
//! \brief Make a new measure 
int32_t CurrentProbe::Measure()
{
	int32_t nRaw = m_pCurrentFB->Read();				// Get the raw ADC value in ADC units
	int32_t nCurrent = nRaw - m_nOffset;				// Subtract the Offset in ADC units
#if !NB_CURRENTMONITOR_SIGNED
	nCurrent = abs(nCurrent);	
#endif	// we are not interested in the current direction
	m_nCurrent = (nCurrent * 1000) / m_nADCgain;		// Calculate the Current in mA
	return m_nCurrent;
}

// ----------------------------------------------------------------------------
//! \brief Set the Offset Correction in ADC units (raw value)
void CurrentProbe::SetOffset(int32_t offset)
{
	m_nOffset = offset;
}

// ----------------------------------------------------------------------------
//! \brief Set the Gain
void CurrentProbe::SetGain(int32_t gain)
{
	m_nADCgain = gain;
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
//! \brief Constructor
CurrentAverage::CurrentAverage(AnalogInput *_pCurrentFB)
	: CurrentProbe(_pCurrentFB)
{	
   dbgprintf("Current Average Constructor ...\n");	
	Reset();
   dbgprintf("... Current Average Constructor done.\n");	
}

// ----------------------------------------------------------------------------
//! \brief Reset the measure
void CurrentAverage::Reset()
{
	m_bReset = true;
}

// ----------------------------------------------------------------------------
//! \brief Get a new measure and update the average
int32_t CurrentAverage::Measure()
{
	// Handle reset requests
	if (m_bReset)
	{
		m_nTotal = 0;
		m_nIndex = 0;
		m_nAverage = 0;
		for (int i = 0;i < NB_CURRENTMONITOR_HISTORYSLOTS;i++)
			m_anHistory[m_nIndex] = 0;
		m_bFull = false;
		m_bReset = false;
	}

	int32_t nCurrent = CurrentProbe::Measure();

	m_nTotal += nCurrent;
	m_anHistory[m_nIndex] = nCurrent;
	m_nIndex++;
	if (m_nIndex >= NB_CURRENTMONITOR_HISTORYSLOTS)
	{
		m_nIndex = 0;
		m_bFull = true;
	}

	// Buffer is full ?
	if (m_bFull) 
	{
		// Handle cyclic buffer
		m_nTotal -= m_anHistory[m_nIndex];
		m_nAverage = (m_nTotal/NB_CURRENTMONITOR_HISTORYSLOTS);
	}
	else
	{
		m_nAverage = (m_nTotal/m_nIndex);
	}
	return m_nAverage;
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------

const char * MotorDriver::strDirection[4] = {
															"None",
															"Left",
															"Right",
															"Both",
};	

// ----------------------------------------------------------------------------
//! \brief Constructor 
MotorDriver::MotorDriver(PWMOutput *_pPWM, 
				CurrentProbe *_pCurrentProbe,
				EMotorDriverMode _eEnableMode,
				EMotorDriverMode _eDisableMode,
				const char *Name,
				DigitalInput * _pEndSWlower,
				DigitalInput * _pEndSWupper,
				eBlockDirMode_t _EndSWlowerBlock,
				eBlockDirMode_t _EndSWupperBlock)
	: 	m_pCurrentProbe(_pCurrentProbe),
		m_pPWM(_pPWM),
		m_eEnableMode(_eEnableMode),
		m_eDisableMode(_eDisableMode),
		m_pEndSWlower(_pEndSWlower),
		m_pEndSWupper(_pEndSWupper),
		m_EndSWlowerBlock(_EndSWlowerBlock),
		m_EndSWupperBlock(_EndSWupperBlock)
{
	if (Name != nullptr)
	{
		strncpy(m_Name,Name,sizeof(m_Name));
		m_Name[sizeof(m_Name) - 1] = '\0';
	}
	else
		m_Name[0] = '\0';
   dbgprintf("Motor Driver Constructor %s ...\n",m_Name);
	if (_pEndSWlower != nullptr)
		dbgprintf("   Lower Endswitch, Block Direction = %s\n",strDirection[_EndSWlowerBlock]);
	if (_pEndSWupper != nullptr)
		dbgprintf("   Upper Endswitch, Block Direction = %s\n",strDirection[_EndSWupperBlock]);
	m_IsEnabled = false;	
	m_EndSwitchState = 0;
	m_nMaxCurrent = 0;
	Disable();
   RegisterHandler(this, EMotorDriverEventId_PWM);
//	if (m_pEndSWlower != nullptr)
//		m_pEndSWlower->RegisterHandler(this, EMotorDriverEventId_ENDSW_L);
//	if (m_pEndSWupper != nullptr)
//		m_pEndSWupper->RegisterHandler(this, EMotorDriverEventId_ENDSW_U);
#if (TRACEALYZER != 0) && (TRC_MOTOR != 0)
	trcMotor = xTraceRegisterString("MOTOR DRV");
	m_s_eMode = EMotorDriverMode_BrakeGND;
	m_s_Num = 0;
	m_s_Denom = 0;
#endif
   dbgprintf("... Motor Driver Constructor done.\n");	
}

// ----------------------------------------------------------------------------

//! \brief Register handlers of PWM notifications
void MotorDriver::HandleEvent(EventSource *_pSource, uint32_t _nEventId, uint32_t _nData)
{
#if (TRACEALYZER != 0) && (TRC_MOTOR != 0)
bool 		s_pEndSWlower = false;		
bool 		s_pEndSWupper = false;		
#endif
	
//	if (_nEventId == EMotorDriverEventId_ENDSW_L)
//	{
//#if (TRACEALYZER != 0) && (TRC_MOTOR != 0)
//		vTracePrint(trcMotor,"EndLower Interrupt");;
//#endif
//	}
//	if (_nEventId == EMotorDriverEventId_ENDSW_U)
//	{
//#if (TRACEALYZER != 0) && (TRC_MOTOR != 0)
//		vTracePrint(trcMotor,"EndUpper Interrupt");;
//#endif
//	}
	if (_nEventId == EMotorDriverEventId_PWM)
	{
		if (m_pPWM->GetTimerIndex() == _nData)		// must match the timer that generated the event
		{
			if (m_eEnableMode != EMotorDriverSide_None && NULL != m_pCurrentProbe)
			{
				int32_t nCurrent = m_pCurrentProbe->Measure();
				if (abs(nCurrent) > m_nMaxCurrent)
					m_nMaxCurrent = abs(nCurrent);
				if (abs(nCurrent) < 5)
				{
	//				Disable();
				}
			}
		}
		if (m_pEndSWlower != nullptr)
		{
			if (m_pEndSWlower->Read())
			{
#if (TRACEALYZER != 0) && (TRC_MOTOR != 0)
				if (!s_pEndSWlower)
				{
					s_pEndSWlower = true;
					vTracePrint(trcMotor,"EndLower active");
				}
#endif
				m_EndSwitchState |= (3 << 0);
#ifdef TEST_ENDSWITCH
#if (TRACEALYZER != 0) && (TRC_MOTOR != 0)
				vTracePrint(trcMotor,"EndLower active, Block Lower");
#endif
				m_pPWM->Block(true,m_EndSWlowerBlock);
#endif
			}
			else
			{
#if (TRACEALYZER != 0) && (TRC_MOTOR != 0)
				if (s_pEndSWlower)
				{
					s_pEndSWlower = false;
					vTracePrint(trcMotor,"EndLower inactive");
				}
#endif
#ifdef TEST_ENDSWITCH
				m_pPWM->Block(false,m_EndSWlowerBlock);
#endif
				m_EndSwitchState &= ~(1 << 0);
			}
		}
		if (m_pEndSWupper != nullptr)
		{
			if (m_pEndSWupper->Read())
			{
#if (TRACEALYZER != 0) && (TRC_MOTOR != 0)
				if (!s_pEndSWupper)
				{
					s_pEndSWupper = true;
					vTracePrint(trcMotor,"EndUpper active");
				}
#endif
				m_EndSwitchState |= (3 << 2);
#ifdef TEST_ENDSWITCH
#if (TRACEALYZER != 0) && (TRC_MOTOR != 0)
				vTracePrint(trcMotor,"EndLower active, Block Upper");
#endif
				m_pPWM->Block(true,m_EndSWupperBlock);
#endif
			}
			else
			{
#if (TRACEALYZER != 0) && (TRC_MOTOR != 0)
				if (s_pEndSWupper)
				{
					s_pEndSWupper = false;
					vTracePrint(trcMotor,"EndUpper inactive");
				}
#endif
#ifdef TEST_ENDSWITCH
				m_pPWM->Block(false,m_EndSWupperBlock);
#endif
				m_EndSwitchState &= ~(1 << 2);
			}
		}
	}	
}

// ----------------------------------------------------------------------------
//! \brief Register handlers of PWM notifications
bool MotorDriver::RegisterHandler(IEventHandler *_pHandler, uint32_t _nEventId)
{
	if (NULL != m_pPWM)
	{
		return m_pPWM->RegisterHandler(_pHandler, _nEventId);
	}
	return false;
}

// ----------------------------------------------------------------------------
//! \brief Enable driver's outputs
bool MotorDriver::Enable(void)
{    
#ifdef DBGPRINTF_MOTOR
	dbgprintf("Enabling Motor Driver %s ...\n",m_Name);
#endif
	SetRatioAndMode(0, 1, m_eEnableMode);
	if (m_pPWM != NULL)
	{
#ifdef DBGPRINTF_MOTOR
		dbgprintf("PWM device assigned\n");
#endif
		m_IsEnabled = true;
		m_pPWM->Start();
		SleepBM(5);
	}
	else
	{
#ifdef DBGPRINTF_MOTOR
		dbgprintf("Error, no PWM device assigned\n");
#endif
		return false;
	}
#ifdef DBGPRINTF_MOTOR
	if (IsStatusOk())
		dbgprintf("... success, starting - status OK\n");
	else
		dbgprintf("... success, starting - status NOK\n");
#endif
	return IsStatusOk();
}

// ----------------------------------------------------------------------------
//! \brief Disable driver's outputs
bool MotorDriver::Disable(void)
{	
	SetRatioAndMode(0, 1, m_eDisableMode);
	if (m_pPWM != NULL)
	{
		m_IsEnabled = false;
		m_pPWM->Stop();
		return true;
	}
	else
		return false;
}

// ----------------------------------------------------------------------------
//! \brief Returns the Enable Status of the Motor Driver
bool MotorDriver::IsEnabled(void)
{	
	return m_IsEnabled;
}

// ----------------------------------------------------------------------------
//! \brief Returns the End Switch State
uint8_t MotorDriver::GetEndSwitchState(void)
{	
	return m_EndSwitchState;
}

// ----------------------------------------------------------------------------
//! \brief Resets the End Switch State
void MotorDriver::ResetEndSwitchState(void)
{
	m_EndSwitchState &= ~(1 << 1);
	m_EndSwitchState &= ~(1 << 3);
}

bool MotorDriver::isBlocked(void)
{
	return m_pPWM->IsBlocked();
}

uint8_t MotorDriver::GetMotorState(void)
{
	return m_pPWM->GetBlockingState();
}

// ----------------------------------------------------------------------------
//! \brief Returns the duration of the PWM period 
uint32_t MotorDriver::GetPeriodDuration() 
{ 
	if (NULL != m_pPWM)
	{
		return m_pPWM->GetPeriodDuration(); 
	}
	return 1;
}

// ----------------------------------------------------------------------------
//! \brief Set the speed of the Enabled Channel (PWM ratio)
bool MotorDriver::SetRatio(uint32_t _nNumerator, uint32_t _nDenominator)
{
	return SetRatioAndMode(_nNumerator, _nDenominator, m_eEnableMode);
}

// ----------------------------------------------------------------------------
//! \brief Set both the speed (PWM ratio) and the mode
bool MotorDriver::SetRatioAndMode(uint32_t _nNumerator, uint32_t _nDenominator, EMotorDriverMode _eMode)
{
eDirMode_t		mode;
	
#if (TRACEALYZER != 0) && (TRC_MOTOR != 0)
	if (_eMode != m_s_eMode || _nNumerator != m_s_Num || _nDenominator != m_s_Denom)
	{
		if (m_pPWM == nullptr)
			vTracePrint(trcMotor,"PWM Device not set");
		else
			vTracePrintF(trcMotor,"%d %d %d",(int)_eMode,_nNumerator,_nDenominator);
		m_s_eMode = _eMode;
		m_s_Num = _nNumerator;
		m_s_Denom = _nDenominator;
	}
#endif
	
	// Avoid divid by zero errors...
	if (_nDenominator == 0) 
	{
		_nNumerator = 0;
		_nDenominator = 1;
	}
	switch (_eMode)
	{
		case EMotorDriverMode_BrakeVCC:
			mode = eDirModeHighZ;
			break;
		case EMotorDriverMode_Clockwise:
			mode = eDirModeLeft;		// TODO Check Direction
			break;
		case EMotorDriverMode_CounterClockwise:
			mode = eDirModeRight;		// TODO Check Direction
			break;
		case EMotorDriverMode_BrakeGND:
			mode = eDirModeBrake;
			break;
		default:
			break;
	}

	if (m_pPWM != NULL) 
	{
		m_eMode = _eMode;
		m_pPWM->SetRatio(_nNumerator, _nDenominator, mode);	// PWM is expected in %%
	}
	return IsStatusOk();
}

// ----------------------------------------------------------------------------
//! \brief Returns true if the status of the driver is ok
bool MotorDriver::IsStatusOk()
{
	if (m_pPWM != NULL) 
	{
		return m_pPWM->GetStatus();
	}
	else
		return false;
}

// ----------------------------------------------------------------------------
//! \brief Return the value of the current
int32_t MotorDriver::GetCurrent() 
{ 
	if (NULL != m_pCurrentProbe)
	{
		return m_pCurrentProbe->GetCurrent(); 
	}
	return 0;
}

// ----------------------------------------------------------------------------
//! \brief Return the measure of current
uint32_t MotorDriver::GetMaxCurrent()
{
    return m_nMaxCurrent;
}

// ----------------------------------------------------------------------------
//! \brief Reset the measure of current
void MotorDriver::ResetCurrentMeasure() 
{ 
	if (NULL != m_pCurrentProbe)
	{
		m_pCurrentProbe->Reset(); 
	}
}

// ----------------------------------------------------------------------------
//! \brief Reset the max current value
void MotorDriver::ResetMaxCurrent()
{
    m_nMaxCurrent = m_pCurrentProbe->GetCurrent();
}
