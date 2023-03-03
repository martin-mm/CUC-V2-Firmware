// ---------------------------------------------------------------------------
//! \package     CleaningUnit
//! \file        SuctionDevice.cpp
//! \brief       Defines the class responsible for management of the suction device
//! 
//! \copyright   Copyright (C) 2011-2012 BlueBotics SA
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Includes
#include <stdlib.h>
#include <stdio.h>
#include "SuctionDevice.h"
#include "MotorDriver.h"
#include "board.h"

#if TRACEALYZER != 0 && TRC_SUCTION != 0
static traceString 				trcSuction;
#endif

#define DBGPRINTF_SUCTION
#undef DBGPRINTF_SUCTION

// ----------------------------------------------------------------------------
//! \brief List event ids supported by SuctionDevice.
typedef enum
{
	ESuctionDeviceEventId_PWM = 100,
} ESuctionDeviceEventId;

// ----------------------------------------------------------------------------
//! \brief Constructor
SuctionDevice::SuctionDevice(MotorDriver &_motor,int ID)
	: m_Motor(_motor),
	  m_ID(ID),
	  m_RampGenerator(SUCTION_RAMP_SLOPE,SUCTION_RAMP_SLOPE_DIV),
	  m_nTime(0)
{
	// The Ramp Slope is SUCTION_RAMP_SLOPE / SUCTION_RAMP_SLOPE_DIV in PWM units / us
   dbgprintf("Suction Device Constructor ...\n");	
	m_Motor.RegisterHandler(this, ESuctionDeviceEventId_PWM);
	m_IsMoving = false;
   dbgprintf("... Suction Device Constructor done.\n");	
#if TRACEALYZER != 0 && TRC_SUCTION != 0
	m_old_ratio = 0xFFFFFFFF;
	m_OldCleaningState = ECleaningDeviceStatus_Disabled;
	trcSuction = xTraceRegisterString("SUCTION");
#endif
}

// ----------------------------------------------------------------------------
//! \brief Event handler
void SuctionDevice::HandleEvent(EventSource *_pSource, uint32_t _nEventId, uint32_t _nData)
{
	if (_nEventId == ESuctionDeviceEventId_PWM)
	{
		if (m_Hold)
		{
       	m_RampGenerator.Reset();
       	m_eStatus = ECleaningDeviceStatus_Stopped;
			m_Motor.SetRatio(0, SUCTION_PWM_DIV);
			m_nTime = 0;
		}
		else
		{
			m_nTime += m_Motor.GetPeriodDuration();
			m_RampGenerator.Execute(m_nTime);
			uint32_t nRatio = m_RampGenerator.GetValue();
			// Update motor command

#if TRACEALYZER != 0 && TRC_SUCTION != 0
			if (m_OldCleaningState != m_eStatus)
			{
				vTracePrintF(trcSuction,"Cleaning State changed from %d to %d = %d",m_OldCleaningState,m_eStatus);
				m_OldCleaningState = m_eStatus;
			}
#endif

			switch (m_eStatus)
			{
				case ECleaningDeviceStatus_Starting:
					m_IsMoving = m_RampGenerator.IsMoving();
					if (!m_IsMoving)
					{
#if TRACEALYZER != 0 && TRC_SUCTION != 0
						vTracePrint(trcSuction,"RampUp done");
#endif
						m_eStatus = ECleaningDeviceStatus_Running;
					}
					break;
				case ECleaningDeviceStatus_Running:
					break;
				case ECleaningDeviceStatus_Stopping:
					if (!m_RampGenerator.IsMoving())
					{
#if TRACEALYZER != 0 && TRC_SUCTION != 0
						vTracePrint(trcSuction,"RampDown done");
#endif
						m_eStatus = ECleaningDeviceStatus_Stopped;
					}
					break;
				default:
					nRatio = 0;
					break;
			}
	      if (!m_DryRunEnabled)
	      {
				m_Motor.SetRatio(nRatio, SUCTION_PWM_DIV);
	      }
			else
	      {
				m_Motor.SetRatio(0, SUCTION_PWM_DIV);
	      }
#if TRACEALYZER != 0 && TRC_SUCTION != 0
			if (m_old_ratio != nRatio)
			{
				vTracePrintF(trcSuction,"Suctionratio = %d",nRatio);
				m_old_ratio = nRatio;
			}
#endif
		}
	}
}

// ----------------------------------------------------------------------------
//! \brief Enable the suction motor
bool SuctionDevice::Enable()
{
#ifdef DBGPRINTF_SUCTION
	dbgprintf("Enable Suction Device, Timer = %d ...\n",m_nTime);
#endif
	m_nTime = 0;
	m_eStatus = ECleaningDeviceStatus_Stopped;
#if TRACEALYZER != 0 && TRC_SUCTION != 0
	vTracePrint(trcSuction,"Enable Suction");
#endif
   m_RampGenerator.SetTarget(0);
 	if (m_Motor.Enable())
	{
#ifdef DBGPRINTF_SUCTION
		dbgprintf("... Enable Suction Device - Success\n");
#endif
		return true;
	}
	m_eStatus = ECleaningDeviceStatus_Error;
#ifdef DBGPRINTF_SUCTION
	dbgprintf("... Enable Suction Device - Failed\n");
#endif
	return false;
}

// ----------------------------------------------------------------------------
//! \brief Start the device
bool SuctionDevice::Start()
{
#if TRACEALYZER != 0 && TRC_SUCTION != 0
	vTracePrintF(trcSuction,"Start Suction, ClStatus = %d",(int)m_eStatus);
#endif
	 if (m_Hold) // Device cannot start due to EMStop
	    return false;
    switch (m_eStatus)
    {
        case ECleaningDeviceStatus_Stopped:
        case ECleaningDeviceStatus_Stopping:			// added by MM
        case ECleaningDeviceStatus_Error:
#ifdef DBGPRINTF_SUCTION
				dbgprintf("Start Suction Device, Now Starting (%d) ...\n",(int)m_eStatus);
#endif
            m_eStatus = ECleaningDeviceStatus_Starting;
#ifdef DBGPRINTF_SUCTION
					 dbgprintf("Start Suction - Normal Mode\n");
//					 dbgprintf("   Set Ramp Generator to 7000\n");
#endif
//            m_RampGenerator.SetTarget(7000);
#if TRACEALYZER != 0 && TRC_SUCTION != 0
				vTracePrint(trcSuction,"Start Suction, RampTarget = 95%");
#endif
            return SetPower(95);
        case ECleaningDeviceStatus_Starting:
        case ECleaningDeviceStatus_Running:
#ifdef DBGPRINTF_SUCTION
			   dbgprintf("Cleaning Device Starting or Running\n");
#endif
            return true;
        default:
            break;
    }
    return false;
}

// ----------------------------------------------------------------------------
//! \brief Stop the device
bool SuctionDevice::Stop()
{
#if TRACEALYZER != 0 && TRC_SUCTION != 0
	vTracePrintF(trcSuction,"Stop Suction, ClStatus = %d",(int)m_eStatus);
#endif
#ifdef DBGPRINTF_SUCTION
	 dbgprintf("Stop Brush Device, Status = %d\n",(int)m_eStatus);
#endif
	 if (m_Hold) // Device is already stopped due to EMStop
	    return true;
    switch (m_eStatus)
    {
        case ECleaningDeviceStatus_Running:
        case ECleaningDeviceStatus_Starting:
#ifdef DBGPRINTF_SUCTION
				dbgprintf("Stop Suction Device, now stopping\n");
#endif
            m_eStatus = ECleaningDeviceStatus_Stopping;
#ifdef DBGPRINTF_SUCTION
				dbgprintf("Stop Suction Device, set Ramp Geberator to 0\n");
#endif
#if TRACEALYZER != 0 && TRC_SUCTION != 0
				vTracePrint(trcSuction,"Stop Suction, RampTarget = 0%");
#endif
//            m_RampGenerator.SetTarget(0);
            return SetPower(0);
        case ECleaningDeviceStatus_Stopping:
        case ECleaningDeviceStatus_Stopped:
#ifdef DBGPRINTF_SUCTION
				dbgprintf("Stop Suction Device, stopping or stoppes\n");
#endif
            return true;
        default:
            break;
    }
    return false;
}

// ----------------------------------------------------------------------------
//! \brief Disable the suction motor
bool SuctionDevice::Disable()
{	
#if TRACEALYZER != 0 && TRC_SUCTION != 0
	vTracePrint(trcSuction,"Disable Suction");
#endif
#ifdef DBGPRINTF_SUCTION
	dbgprintf("Disable Suction Device ...\n");
#endif
	SetPower(0);
	if (m_Motor.Disable())
	{
		m_eStatus = ECleaningDeviceStatus_Disabled;
#ifdef DBGPRINTF_SUCTION
		dbgprintf("... Disable Suction Device - Success\n");
#endif
		return true;
	}
#ifdef DBGPRINTF_SUCTION
	dbgprintf("... Disable Suction Device - Failed\n");
#endif
	m_eStatus = ECleaningDeviceStatus_Error;
	return false;

}

// ----------------------------------------------------------------------------
//! \brief Change the suction power
bool SuctionDevice::SetPower(uint32_t _nPower /*percentage*/)
{
	if (m_eStatus == ECleaningDeviceStatus_Starting ||
	    m_eStatus == ECleaningDeviceStatus_Stopping ||
		m_eStatus == ECleaningDeviceStatus_Running)
	{
		if (_nPower > 100)
			_nPower = 100;
		m_RampGenerator.SetTarget(_nPower * 100);
		return true;
	}
	return false;
}

// ----------------------------------------------------------------------------
//! \brief Return true if the current is not too low (> 500 mA)
bool SuctionDevice::CheckCurrent()
{
	int32_t nCurrent = m_Motor.GetCurrent();
	return (abs(nCurrent) > 500);
}

// ----------------------------------------------------------------------------
//! \brief Return the maximum measured current
uint32_t SuctionDevice::GetMaxCurrent() 
{ 
    return m_Motor.GetMaxCurrent();
}

// ----------------------------------------------------------------------------
//! \brief Reset the maximum measured current
void SuctionDevice::ResetMaxCurrent()
{
    m_Motor.ResetMaxCurrent();
}

// ----------------------------------------------------------------------------
//! \brief Returns the Suction Device Name
const char * SuctionDevice::GetDeviceName(void)
{
	return m_Motor.GetName();
}

// ----------------------------------------------------------------------------
//! \brief Sets the Ramp Slope of the Suction Device
bool SuctionDevice::SetRampSlope(unsigned Slope,unsigned SlopeDiv)
{
	m_RampGenerator.SetSlope(Slope,SlopeDiv);
	return true;
}
