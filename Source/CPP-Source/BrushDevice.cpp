// ---------------------------------------------------------------------------
//! \package     CleaningUnit
//! \file        BrushDevice.cpp
//! \brief       Defines the class responsible for management of the brush device
//! 
//! \copyright   Copyright (C) 2011-2012 BlueBotics SA
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Includes
#include <stdlib.h>
#include <stdio.h>
#include "BrushDevice.h"
#include "MotorDriver.h"
#include "AnalogInput.h"
#include "board.h"

#if TRACEALYZER != 0 && TRC_BRUSH != 0
static traceString 				trcBrush;
#endif

#define DBGPRINTF_BRUSH
#undef DBGPRINTF_BRUSH

// ----------------------------------------------------------------------------
//! \brief List event ids supported by BrushDevice.
typedef enum
{
	EBrushDeviceEventId_PWM = 100,
} EBrushDeviceEventId;

// ----------------------------------------------------------------------------
//! \brief Constructor
BrushDevice::BrushDevice(MotorDriver &_motor,int ID)
	: m_Motor(_motor),
	  m_ID(ID),
	  m_RampGenerator(BRUSH_RAMP_SLOPE,BRUSH_RAMP_SLOPE_DIV),
	  m_nTime(0)
{
   dbgprintf("Brush Device Constructor ...\n");	
	m_Motor.RegisterHandler(this, EBrushDeviceEventId_PWM);
	m_IsMoving = false;
   dbgprintf("... Brush Device Constructor done.\n");	
#if TRACEALYZER != 0 && TRC_BRUSH != 0
	trcBrush = xTraceRegisterString("BRUSH");
	m_OldCleaningState = ECleaningDeviceStatus_Disabled;
	m_old_ratio = 0xFFFFFFFF;
#endif
}

// ----------------------------------------------------------------------------
//! \brief Event handler
void BrushDevice::HandleEvent(EventSource *_pSource, uint32_t _nEventId, uint32_t _nData)
{
//	dbgprintf("Brush Device Event: %d, Status = %d\n",(int)_nEventId,(int)m_eStatus);
	if (_nEventId == EBrushDeviceEventId_PWM)
	{
		if (m_Hold)
		{
       	m_RampGenerator.Reset();
       	m_eStatus = ECleaningDeviceStatus_Stopped;
			m_Motor.SetRatio(0, BRUSH_PWM_DIV);
			m_nTime = 0;
		}
		else
		{
			m_nTime += m_Motor.GetPeriodDuration();
			m_RampGenerator.Execute(m_nTime);
			uint32_t ratio = m_RampGenerator.GetValue();
			m_Motor.SetRatio(ratio, BRUSH_PWM_DIV);
#if TRACEALYZER != 0 && TRC_BRUSH != 0
			if (m_old_ratio != ratio)
			{
				vTracePrintF(trcBrush,"Brushratio = %d",ratio);
				m_old_ratio = ratio;
			}
			if (m_OldCleaningState != m_eStatus)
			{
				vTracePrintF(trcBrush,"Cleaning State changed from %d to %d = %d",m_OldCleaningState,m_eStatus);
				m_OldCleaningState = m_eStatus;
			}
#endif
			switch (m_eStatus)
			{
				case ECleaningDeviceStatus_Starting:
					m_IsMoving = m_RampGenerator.IsMoving();
					if (!m_IsMoving)
					{
#if TRACEALYZER != 0 && TRC_BRUSH != 0
						vTracePrint(trcBrush,"RampUp done");
#endif
						m_eStatus = ECleaningDeviceStatus_Running;
					}
					break;
				case ECleaningDeviceStatus_Running:
	//				if (!CheckCurrent()) Disable();
					break;
				case ECleaningDeviceStatus_Stopping:
					if (!m_RampGenerator.IsMoving())
					{
#if TRACEALYZER != 0 && TRC_BRUSH != 0
						vTracePrint(trcBrush,"RampDown done");
#endif
						m_eStatus = ECleaningDeviceStatus_Stopped;
					}
					break;
				default:
					break;
			}
		}
	}
}

// ----------------------------------------------------------------------------
//! \brief Enable the brush motor
bool BrushDevice::Enable()
{
#ifdef DBGPRINTF_BRUSH
	dbgprintf("Enable Brush Device, Timer = %d ...\n",m_nTime);
#endif
#if TRACEALYZER != 0 && TRC_BRUSH != 0
	vTracePrint(trcBrush,"Enable Brush");
#endif
	m_RampGenerator.SetTarget(0);
	m_eStatus = ECleaningDeviceStatus_Stopped;
 	if (m_Motor.Enable())
	{
#ifdef DBGPRINTF_BRUSH
		dbgprintf("... Enable Brush Device - Success\n");
#endif
		return true;
	}
	m_eStatus = ECleaningDeviceStatus_Error;
#ifdef DBGPRINTF_BRUSH
	dbgprintf("... Enable Brush Device - Failed\n");
#endif
	return false;
}

// ----------------------------------------------------------------------------
//! \brief Start rotation of the brush
bool BrushDevice::Start()
{
#if TRACEALYZER != 0 && TRC_BRUSH != 0
	vTracePrintF(trcBrush,"Start Brush, ClStatus = %d",(int)m_eStatus);
#endif
	 if (m_Hold) // Device cannot start due to EMStop
	    return false;
    switch (m_eStatus)
    {
        case ECleaningDeviceStatus_Stopped:
        case ECleaningDeviceStatus_Stopping:			// added by MM
        case ECleaningDeviceStatus_Error:
#ifdef DBGPRINTF_BRUSH
				dbgprintf("Start Brush Device, Now Starting (%d) ...\n",(int)m_eStatus);
#endif
            m_eStatus = ECleaningDeviceStatus_Starting;

            if (!m_DryRunEnabled)
            {
#ifdef DBGPRINTF_BRUSH
					 dbgprintf("Start Brush - Normal Mode\n");
					 dbgprintf("   Set Ramp Generator to 7000\n");
#endif
#if TRACEALYZER != 0 && TRC_BRUSH != 0
					 vTracePrint(trcBrush,"Start Brush, RampTarget = 70%");
#endif
                m_RampGenerator.SetTarget(7000);
            }
            else
            {
#ifdef DBGPRINTF_BRUSH
					 dbgprintf("Start Brush - Demo Mode\n");
					 dbgprintf("   Set Ramp Generator to 0\n");
#endif
#if TRACEALYZER != 0 && TRC_BRUSH != 0
					 vTracePrint(trcBrush,"Start Brush, RampTarget = 0%");
#endif
                m_RampGenerator.SetTarget(0);
            }

            return true;
        case ECleaningDeviceStatus_Starting:
        case ECleaningDeviceStatus_Running:
#ifdef DBGPRINTF_BRUSH
			   dbgprintf("Cleaning Device Starting or Running\n");
#endif
            return true;
        default:
            break;
    }
    return false;
}

// ----------------------------------------------------------------------------
//! \brief Stop rotation of the brush
bool BrushDevice::Stop()
{	
#ifdef DBGPRINTF_BRUSH
	 dbgprintf("Stop Brush Device, Status = %d\n",(int)m_eStatus);
#endif
#if TRACEALYZER != 0 && TRC_BRUSH != 0
	vTracePrintF(trcBrush,"Stop Brush, ClStatus = %d",(int)m_eStatus);
#endif
	 if (m_Hold) // Device is already stopped due to EMStop
	    return true;
    switch (m_eStatus)
    {
        case ECleaningDeviceStatus_Running:
        case ECleaningDeviceStatus_Starting:
#ifdef DBGPRINTF_BRUSH
				dbgprintf("Stop Brush Device, now stopping\n");
#endif
            m_eStatus = ECleaningDeviceStatus_Stopping;
#ifdef DBGPRINTF_BRUSH
				dbgprintf("Stop Brush Device, set Ramp Generator to 0\n");
#endif
#if TRACEALYZER != 0 && TRC_BRUSH != 0
				vTracePrint(trcBrush,"Stop Suction, RampTarget = 0%");
#endif
            m_RampGenerator.SetTarget(0);
            return true;
        case ECleaningDeviceStatus_Stopping:
        case ECleaningDeviceStatus_Stopped:
#ifdef DBGPRINTF_BRUSH
				dbgprintf("Stop Brush Device, stopping or stoppes\n");
#endif
            return true;
        default:
            break;
    }
    return false;
}

// ----------------------------------------------------------------------------
//! \brief Disable the brush motor
bool BrushDevice::Disable()
{
#if TRACEALYZER != 0 && TRC_BRUSH != 0
	vTracePrint(trcBrush,"Disable Brush");
#endif
#ifdef DBGPRINTF_BRUSH
	dbgprintf("Disable Brush Device ...\n");
#endif
	m_RampGenerator.SetTarget(0);
	if (m_Motor.Disable())
	{
		m_eStatus = ECleaningDeviceStatus_Disabled;
#ifdef DBGPRINTF_BRUSH
		dbgprintf("... Disable Brush Device - Success\n");
#endif
		return true;
	}
#ifdef DBGPRINTF_BRUSH
	dbgprintf("... Disable Brush Device - Failed\n");
#endif
	m_eStatus = ECleaningDeviceStatus_Error;
	return false;
}

// ----------------------------------------------------------------------------
//! \brief Return true if the current is not too low (> 500 mA)
bool BrushDevice::CheckCurrent()
{
	int32_t nCurrent = m_Motor.GetCurrent();
	return (abs(nCurrent) > 500);
}

// ----------------------------------------------------------------------------
//! \brief Return the maximum measured current
uint32_t BrushDevice::GetMaxCurrent() 
{ 
    return m_Motor.GetMaxCurrent();
}

// ----------------------------------------------------------------------------
//! \brief Reset the maximum measured current
void BrushDevice::ResetMaxCurrent()
{
    m_Motor.ResetMaxCurrent();
}

// ----------------------------------------------------------------------------
//! \brief Returns the Brush Device Name
const char * BrushDevice::GetDeviceName(void)
{
	return m_Motor.GetName();
}

// ----------------------------------------------------------------------------
//! \brief Sets the Ramp Slope of the Brush Device
bool BrushDevice::SetRampSlope(unsigned Slope,unsigned SlopeDiv)
{
	m_RampGenerator.SetSlope(Slope,SlopeDiv);
	return true;
}
