// ---------------------------------------------------------------------------
//! \package     CleaningUnit
//! \file        WaterPumpDevice.cpp
//! \brief       Defines the class responsible for management of the water pump device
//! 
//! \copyright   Copyright (C) 2011-2012 BlueBotics SA
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Includes
#include <stdlib.h>
#include <stdio.h>
#include "CleaningUnit.h"
#include "WaterPumpDevice.h"
#include "MotorDriver.h"
#include "Timer.h"
#include "IO.h"
#include "board.h"

#define WATERPUMP_ALTERNATE_HW  	1

// ----------------------------------------------------------------------------
// Constants
// The FlowMeter generates 236 pulses per liter
#define NB_MICROLITER_PER_PULSE             4237 	// The flow per pulse in ul / pulse
#define WATERPUMPS_FLOW                     1400 	// The waterpump flow in ul / pulse
#define ACTIVATION_DURATION                 1700 	// Duration of pump activation in usec
#define PULSE_DURATION                      6700 	// The duration of a pump pulse in usec
#define DEACTIVATION_DURATION               1600 	// The duration of pump deactivation in usec
#define WATERPUMPS_HALFSINE_DURATION        7000 	// The waterpump halfsine duration in usec
#define WATERPUMPS_SHORTWAIT_DURATION       3000 	// The waterpump shortwait duration in usec
#define WATERPUMPS_WAIT_MAX_DURATION        (100000 - (2 * (WATERPUMPS_HALFSINE_DURATION + WATERPUMPS_SHORTWAIT_DURATION))) 
//#define WATERPUMPS_AMPLITUDE                1000	// The waterpump PWM amplitude in %%
#define WATERPUMPS_AMPLITUDE                1000	// The waterpump PWM amplitude in %%

#define DBGPRINTF_WPUMP
#undef DBGPRINTF_WPUMP

#if TRACEALYZER != 0 && TRC_WATER != 0
static traceString 				trcWater;
#endif

// ----------------------------------------------------------------------------
//! \brief List event ids supported by WaterPumpDevice.
typedef enum
{
	EWaterPumpDeviceEventId_PWM_1 = 100,
	EWaterPumpDeviceEventId_PWM_2,
	EWaterPumpDeviceEventId_FlowMeter,
} EWaterPumpDeviceEventId;

const char * WaterPumpDevice::Name = "Waterpump 1 and 2";

// ----------------------------------------------------------------------------
//! \brief Constructor
WaterPumpDevice::WaterPumpDevice(MotorDriver &_pump1, MotorDriver &_pump2, DigitalInput *_pFlowMeterCapture,
	int ID)
	: 	m_Pump1(_pump1),
		m_Pump2(_pump2),	 
		m_pFlowMeterCapture(_pFlowMeterCapture),
		m_PIDController(5, 0, 0, ID),
      m_nFlowPulses(0),
      m_nNextMeasureTime(0),
		m_nWaterDensity(0),
      m_bTankIsEmpty(false),
		m_ID(ID),
      m_nAbsFlowPulses(0)
{
   dbgprintf("Water Pump Device Constructor ...\n");	
	_pump1.RegisterHandler(this, EWaterPumpDeviceEventId_PWM_1);
	_pump2.RegisterHandler(this, EWaterPumpDeviceEventId_PWM_2);
	m_pFlowMeterCapture->RegisterHandler(this, EWaterPumpDeviceEventId_FlowMeter);

	int32_t nHalfRange = WATERPUMPS_WAIT_MAX_DURATION/2;
	m_PIDController.SetOutputSaturation(-nHalfRange, nHalfRange);

	Stop();
	Disable();
	SetWaterDensity(13800); // in ul/m, = 0.5 l/min @ 0.6 m/s
#if TRACEALYZER != 0 && TRC_WATER != 0
	trcWater = xTraceRegisterString("WATERPUMP");
	m_nTotalDuration_old = 0;  
	m_OldCleaningState = ECleaningDeviceStatus_Disabled;
	m_SignalStateOld = WaterPumpSignalState_Wait;
	m_eStatus_old = ECleaningDeviceStatus_Disabled;
#endif
	dbgprintf("... Water Pump Device Constructor done.\n");	
}


#if WATERPUMP_ALTERNATE_HW != 0

// ----------------------------------------------------------------------------
//! \brief Event handler
void WaterPumpDevice::HandleEvent(EventSource *_pSource, uint32_t _nEventId, uint32_t _nData)
{
uint32_t 							nTotalDuration = 0;
uint32_t 							nTargetFlow = 0;
	
	if (_nEventId == EWaterPumpDeviceEventId_FlowMeter)
	{
		m_nFlowPulses++;
		m_nAbsFlowPulses++;
		return;
	}

	if (_nEventId == EWaterPumpDeviceEventId_PWM_1)
	{
#if TRACEALYZER != 0 && TRC_WATER != 0
		if (m_OldCleaningState != m_eStatus)
		{
			vTracePrintF(trcWater,"Cleaning State changed from %d to %d",m_OldCleaningState,m_eStatus);
			m_OldCleaningState = m_eStatus;
		}
#endif
		if (m_Hold)
		{
       	BOARD_EnablePump(0,false);
       	BOARD_EnablePump(1,false);
       	m_pFlowMeterCapture->DisableInterrupt(EEdge_Rising);
       	m_eStatus = ECleaningDeviceStatus_Stopped;
		}
		uint32_t nDuration = 2 * (ACTIVATION_DURATION + PULSE_DURATION + DEACTIVATION_DURATION); // in usec for A and B;
		if (BOARD_isPumpClMgrEnabled())
		{
			// Calculate flow target value
			switch (m_eStatus)
			{
				case ECleaningDeviceStatus_Starting:
					if (BOARD_EnablePump(0,true) && BOARD_EnablePump(1,true))
					{
#if TRACEALYZER != 0 && TRC_WATER != 0
						vTracePrint(trcWater,"Pump 1 & 2 enabled (FSM)");
						vTracePrint(trcWater,"ClState Starting -> Running (FSM)");
#endif
						m_eStatus = ECleaningDeviceStatus_Running; 
					}
					break;
				case ECleaningDeviceStatus_Running:
					nTargetFlow = (m_nWaterDensity * m_nAbsSd) / 100;  // density in ul/m * velocity in m/s -> nTargetFlow = microliters / second
					if (nTargetFlow > 0)
					{
						nTotalDuration = (1000000 * WATERPUMPS_FLOW) / nTargetFlow; // in us
						if (nTotalDuration < nDuration)
							nTotalDuration = nDuration;
						if (m_DryRunEnabled)
							BOARD_ClMngr_SetParameters(ACTIVATION_DURATION / 1000,PULSE_DURATION / 1000,
									DEACTIVATION_DURATION / 1000,nTotalDuration / 1000,0);
						else
							BOARD_ClMngr_SetParameters(ACTIVATION_DURATION / 1000,PULSE_DURATION / 1000,
									DEACTIVATION_DURATION / 1000,nTotalDuration / 1000,WATERPUMPS_AMPLITUDE);
#if TRACEALYZER != 0 && TRC_WATER != 0
						if (nTotalDuration != m_nTotalDuration_old) 
						{
							vTracePrintF(trcWater,"ClState nTotDur = %dus, Pulse = %dus (on) (FSM)",(int)nTotalDuration,(int)PULSE_DURATION);
							m_nTotalDuration_old = nTotalDuration;
						}
#endif
					}
					else
					{
						nTotalDuration = ACTIVATION_DURATION + PULSE_DURATION + DEACTIVATION_DURATION;
						BOARD_ClMngr_SetParameters(ACTIVATION_DURATION / 1000,0,
								DEACTIVATION_DURATION / 1000,nTotalDuration / 1000,WATERPUMPS_AMPLITUDE);
#if TRACEALYZER != 0 && TRC_WATER != 0
						if (nTotalDuration != m_nTotalDuration_old) 
						{
							vTracePrintF(trcWater,"nTotDur = %d us (off) (FSM)",(int)nTotalDuration);
							vTracePrintF(trcWater,"Target flow = %d ul/s (FSM)",(int)nTargetFlow);
							vTracePrintF(trcWater,"Flow Pulses = %d, abs = %d (FSM)",(int)m_nFlowPulses,(int)m_nAbsFlowPulses);
							m_nTotalDuration_old = nTotalDuration;
						}
#endif
					}
					CheckWaterFlow(nTargetFlow);            
					break;
				case ECleaningDeviceStatus_Stopping:
					if (BOARD_EnablePump(0,false) && BOARD_EnablePump(1,false))
					{
#if TRACEALYZER != 0 && TRC_WATER != 0
						vTracePrint(trcWater,"Pump 1 & 2 disabled (FSM)");
#endif
					}
#if TRACEALYZER != 0 && TRC_WATER != 0
					vTracePrint(trcWater,"ClState Stopping -> Stopped (FSM)");
#endif
					m_eStatus = ECleaningDeviceStatus_Stopped;
//					m_bTankIsEmpty = false;
					break;
				case ECleaningDeviceStatus_Stopped:
					break;
				default:
					;
			}
		}
#if TRACEALYZER != 0 && TRC_WATER != 0
		if (m_eStatus_old != m_eStatus)
		{
			vTracePrintF(trcWater,"ClState -> %d (FSM)",(int)m_eStatus);
			m_eStatus_old = m_eStatus;
		}
#if WATERPUMP_ALTERNATE_HW == 0
		if (m_eState != m_eState_old)
		{
			vTracePrintF(trcWater,"WaState -> %d (FSM)",(int)m_eState);
			m_eState_old = m_eState;
		}
#endif
#endif
	}
}

#else

// ----------------------------------------------------------------------------
//! \brief Event handler
void WaterPumpDevice::HandleEvent(EventSource *_pSource, uint32_t _nEventId, uint32_t _nData)
{
static ECleaningDeviceStatus	m_eStatus_old = ECleaningDeviceStatus_Disabled;
static WaterPumpSignalState	m_eState_old = WaterPumpSignalState_Wait;
	
	if (_nEventId == EWaterPumpDeviceEventId_FlowMeter)
	{
		m_nFlowPulses++;
		m_nAbsFlowPulses++;
		return;
	}

	if (_nEventId == EWaterPumpDeviceEventId_PWM_1)
	{
		if (m_Hold)
		{
       	BOARD_EnablePump(0,false);
       	BOARD_EnablePump(1,false);
       	m_pFlowMeterCapture->DisableInterrupt(EEdge_Rising);
       	m_eStatus = ECleaningDeviceStatus_Stopped;
		}
		// Calculate flow target value
		uint32_t nWaitDuration = 0;
      uint32_t nTargetFlow = (m_nWaterDensity * m_nAbsSd) / 100;  // density in ul/m * velocity in m/s -> nTargetFlow = microliters / second
      if (nTargetFlow > 0) 
      {
			uint32_t nDuration = 2 * (ACTIVATION_DURATION + PULSE_DURATION + DEACTIVATION_DURATION); 	// in us for A and B;
         uint32_t nTotalDuration = (1000000 * WATERPUMPS_FLOW) / nTargetFlow; 							// in us
         if (nTotalDuration > nDuration)
				nWaitDuration = nTotalDuration - nDuration;
      }

		// Initialize values for the not running case
		uint32_t ratioA = 0;
		uint32_t ratioB = 0;
		EMotorDriverMode eModeA = EMotorDriverMode_Clockwise;
		EMotorDriverMode eModeB = EMotorDriverMode_Clockwise;
		uint32_t nDuration = 0;
		WaterPumpSignalState eNextState = WaterPumpSignalState_Wait;

#if TRACEALYZER != 0 && TRC_WATER != 0
		if (m_eStatus_old != m_eStatus)
		{
			vTracePrintF(trcWater,"ClState -> %d (FSM)",(int)m_eStatus);
			m_eStatus_old = m_eStatus;
		}
		if (m_eState != m_eState_old)
		{
			vTracePrintF(trcWater,"WaState -> %d (FSM)",(int)m_eState);
			m_eState_old = m_eState;
		}
#endif
		// Are we in the running state ?
		if (m_eStatus == ECleaningDeviceStatus_Running ||
          m_eStatus == ECleaningDeviceStatus_Stopping)
		{
			CheckWaterFlow(nTargetFlow);            
			bool bEnteringState = (m_nCounter == 0);
			m_nCounter += m_Pump1.GetPeriodDuration();   // usec
			switch (m_eState)
			{
				// Activation of Pump 1
				case WaterPumpSignalState_Activation_A: 
					if (bEnteringState)
						m_Pump1.ResetCurrentMeasure();
					eModeA = EMotorDriverMode_Clockwise; 
					ratioA = 0;
					eNextState = WaterPumpSignalState_Pulse_A;
					nDuration = ACTIVATION_DURATION; // usec
					break;
				// Init Pulse for Pump 1
				case WaterPumpSignalState_Pulse_A:
					eModeA = EMotorDriverMode_Clockwise; 
					ratioA = WATERPUMPS_AMPLITUDE;
					eNextState = WaterPumpSignalState_Deactivation_A;
					nDuration = PULSE_DURATION; // usec
					break;
				// Deactivation of Pump 1
				case WaterPumpSignalState_Deactivation_A:
					eModeA = EMotorDriverMode_Clockwise; 
					ratioA = 0;
					eNextState = WaterPumpSignalState_Activation_B;
					nDuration = DEACTIVATION_DURATION; // usec
					break;
				// Activation of Pump 2
				case WaterPumpSignalState_Activation_B:
					eModeB = EMotorDriverMode_Clockwise; 
					ratioB = 0;
					eNextState = WaterPumpSignalState_Pulse_B;
					nDuration = ACTIVATION_DURATION; // usec
					break;
				// Init Pulse for Pump 2
				case WaterPumpSignalState_Pulse_B:
					eModeB = EMotorDriverMode_Clockwise; 
					ratioB = WATERPUMPS_AMPLITUDE;
					eNextState = WaterPumpSignalState_Deactivation_B;
					nDuration = PULSE_DURATION; // usec
					break;
				// Deactivation of Pump 2
				case WaterPumpSignalState_Deactivation_B:
					eModeB = EMotorDriverMode_Clockwise; 
					ratioB = 0;
					eNextState = WaterPumpSignalState_Wait;
					nDuration = DEACTIVATION_DURATION; // usec
					break;
				case WaterPumpSignalState_Wait:
					eModeA = EMotorDriverMode_BrakeGND; 
					eModeB = EMotorDriverMode_BrakeGND; 
					ratioA = 0;
					ratioB = 0;
					if (nTargetFlow > 0)
						eNextState = WaterPumpSignalState_Activation_A;
					else
						eNextState = WaterPumpSignalState_Wait;
					nDuration = nWaitDuration;
               if (m_eStatus == ECleaningDeviceStatus_Stopping)
						m_eStatus = ECleaningDeviceStatus_Stopped;
					break;
				default:
					break;
			}
		}

		// Handle switch to next state
		if (m_nCounter > nDuration) 
		{
			if (m_eState == WaterPumpSignalState_Wait)
			{
				// Make sure a load is present on the output
//				if (!CheckCurrent(void)) Disable(void);
			}
			m_nCounter = 0;
			m_eState = eNextState;
		}

		// Set ratio and direction
      if (!m_DryRunEnabled)
      {
			m_Pump1.SetRatioAndMode(ratioA, WATERPUMPS_AMPLITUDE, eModeA);
         m_Pump2.SetRatioAndMode(ratioB, WATERPUMPS_AMPLITUDE, eModeB);
      }
	}
	if (_nEventId == EWaterPumpDeviceEventId_PWM_2)
	{
		// Nothing to do here, all done in Event Handler for EWaterPumpDeviceEventId_PWM_1
	}
}

#endif

#if WATERPUMP_ALTERNATE_HW != 0

// ----------------------------------------------------------------------------
//! \brief Enable the pump
bool WaterPumpDevice::Enable(void)
{
	BOARD_SetPumpPWM(0,0);
	BOARD_SetPumpPWM(1,0);
#if TRACEALYZER != 0 && TRC_WATER != 0
	vTracePrint(trcWater,"ClState -> Stopped (Enable)");
#endif
	m_eStatus = ECleaningDeviceStatus_Stopped;
#ifdef DBGPRINTF_WPUMP
	dbgprintf("Enabling Water Pump Device ... ");
#endif
	return true;
}

#else

// ----------------------------------------------------------------------------
//! \brief Enable the pump
bool WaterPumpDevice::Enable(void)
{
//	m_Pump1.SetRatio(0, 1);
//	m_Pump2.SetRatio(0, 1);
#if TRACEALYZER != 0 && TRC_WATER != 0
	vTracePrint(trcWater,"ClState -> Stopped (Enable)");
#endif
	m_eStatus = ECleaningDeviceStatus_Stopped;
#ifdef DBGPRINTF_WPUMP
	dbgprintf("Enabling Water Pump Device ... ");
#endif
	return true;
}

#endif


#if WATERPUMP_ALTERNATE_HW != 0

// ----------------------------------------------------------------------------
//! \brief Start the pump
bool WaterPumpDevice::Start(void)
{
#ifdef DBGPRINTF_WPUMP
	 dbgprintf("Start Water Pump Device, Status = %d\n",(int)m_eStatus);
#endif
#if TRACEALYZER != 0 && TRC_WATER != 0
	vTracePrintF(trcWater,"ClState = %d (START)",(int)m_eStatus);
#endif
	 if (m_Hold) // Device cannot start due to EMStop
	    return false;
    switch (m_eStatus)
    {
        case ECleaningDeviceStatus_Stopped:
        case ECleaningDeviceStatus_Error:
        {
#ifdef DBGPRINTF_WPUMP
				dbgprintf("Water Pump Device - Cleaning stopped or error\n");
#endif
//            m_pFlowMeterCapture->Configure(EEdge_Rising, true);	
				m_pFlowMeterCapture->EnableInterrupt(EEdge_Rising);	
            uint32_t nPWMPeriod = m_Pump1.GetPeriodDuration();
            m_nCounter = 0;
				BOARD_ClMngr_EnaPumpCtrl(1);
				BOARD_ClMngr_EnaClFuidValveCtrl(1);
				if (m_Pump1.Enable() && m_Pump2.Enable())
				{	
					m_eStatus = ECleaningDeviceStatus_Starting;
#if TRACEALYZER != 0 && TRC_WATER != 0
					vTracePrint(trcWater,"ClState -> Starting (Start)");
#endif
            }
				else
				{
					m_eStatus = ECleaningDeviceStatus_Error;
#if TRACEALYZER != 0 && TRC_WATER != 0
					vTracePrint(trcWater,"ERROR, Pump Enable Failed (Start)");
#endif
				}
            m_nMeasureIndex = 0;
            m_nTotalActual = 0;
            m_nTotalTargeted =0 ;
            for (int i = 0; i < NB_FLOW_MEASURES ; i++)
            {
                m_anActualFlows[i] = 0;
                m_anTargetFlows[i] = 0;
            }
            m_bTankIsEmpty = false;
            return true;
        }
        case ECleaningDeviceStatus_Starting:
        case ECleaningDeviceStatus_Running:
#ifdef DBGPRINTF_WPUMP
				dbgprintf("Water Pump Device - Cleaning starting or running\n");
#endif
            return true;
        default:
            break;
    }
    return false;
}

#else

// ----------------------------------------------------------------------------
//! \brief Start the pump
bool WaterPumpDevice::Start(void)
{
#ifdef DBGPRINTF_WPUMP
	 dbgprintf("Start Water Pump Device, Status = %d\n",(int)m_eStatus);
#endif
#if TRACEALYZER != 0 && TRC_WATER != 0
	vTracePrintF(trcWater,"ClState = %d (START)",(int)m_eStatus);
#endif
	 if (m_Hold) // Device cannot start due to EMStop
	    return false;
    switch (m_eStatus)
    {
        case ECleaningDeviceStatus_Stopped:
        case ECleaningDeviceStatus_Error:
        {
#ifdef DBGPRINTF_WPUMP
				dbgprintf("Water Pump Device - Cleaning stopped or error\n");
#endif
//            m_pFlowMeterCapture->Configure(EEdge_Rising, true);	
				m_pFlowMeterCapture->EnableInterrupt(EEdge_Rising);	
            uint32_t nPWMPeriod = m_Pump1.GetPeriodDuration();
            m_nCounter = 0;
            m_eState = WaterPumpSignalState_Activation_A;
            m_eStatus = ECleaningDeviceStatus_Running;
#if TRACEALYZER != 0 && TRC_WATER != 0
				vTracePrint(trcWater,"ClState -> Running (Start)");
				vTracePrint(trcWater,"WaState -> Activate A (Start)");
#endif
            
            m_nMeasureIndex = 0;
            m_nTotalActual = 0;
            m_nTotalTargeted =0 ;
            for (int i = 0; i < NB_FLOW_MEASURES ; i++)
            {
                m_anActualFlows[i] = 0;
                m_anTargetFlows[i] = 0;
            }
            m_bTankIsEmpty = false;
            return true;
        }
        case ECleaningDeviceStatus_Starting:
        case ECleaningDeviceStatus_Running:
#ifdef DBGPRINTF_WPUMP
				dbgprintf("Water Pump Device - Cleaning starting or running\n");
#endif
            return true;
        default:
            break;
    }
    return false;
}

#endif

#if WATERPUMP_ALTERNATE_HW != 0

// ----------------------------------------------------------------------------
//! \brief Stop the pump
bool WaterPumpDevice::Stop(void)
{
#ifdef DBGPRINTF_WPUMP
	 dbgprintf("Stop Water Pump Device, Status = %d\n",(int)m_eStatus);
#endif
#if TRACEALYZER != 0 && TRC_WATER != 0
	 vTracePrintF(trcWater,"ClState = %d (Stop)",(int)m_eStatus);
#endif
	 if (m_Hold) // Device is already stopped due to EMStop
	    return true;
    switch (m_eStatus)
    {
        case ECleaningDeviceStatus_Running:
        case ECleaningDeviceStatus_Starting:
            // Wait end of next cycle before setting ratio to 0 // See HandleEvent
#ifdef DBGPRINTF_WPUMP
				dbgprintf("Stop Water Pump Device, now stopping\n");
#endif
#if TRACEALYZER != 0 && TRC_WATER != 0
				vTracePrint(trcWater,"ClState -> Stopping (Stop)");
#endif
				m_pFlowMeterCapture->DisableInterrupt(EEdge_Rising);	
            m_eStatus = ECleaningDeviceStatus_Stopping;
            return true;
        case ECleaningDeviceStatus_Stopping:
        case ECleaningDeviceStatus_Stopped:
#if TRACEALYZER != 0 && TRC_WATER != 0
				vTracePrintF(trcWater,"ClState = %d (Stop)",(int)m_eStatus);
#endif
#ifdef DBGPRINTF_WPUMP
				dbgprintf("Stop Water Pump, stopping or stoppes\n");
#endif
            return true;
        default:
            break;
    }
    return false;
}

#else

// ----------------------------------------------------------------------------
//! \brief Stop the pump
bool WaterPumpDevice::Stop(void)
{
#ifdef DBGPRINTF_WPUMP
	 dbgprintf("Stop Water Pump Device, Status = %d\n",(int)m_eStatus);
#endif
#if TRACEALYZER != 0 && TRC_WATER != 0
	 vTracePrintF(trcWater,"ClState = %d (Stop)",(int)m_eStatus);
#endif
	 if (m_Hold) // Device is already stopped due to EMStop
	    return true;
    switch (m_eStatus)
    {
        case ECleaningDeviceStatus_Running:
        case ECleaningDeviceStatus_Starting:
            // Wait end of next cycle before setting ratio to 0 // See HandleEvent
#ifdef DBGPRINTF_WPUMP
				dbgprintf("Stop Water Pump Device, now stopping\n");
#endif
#if TRACEALYZER != 0 && TRC_WATER != 0
				vTracePrint(trcWater,"ClState -> Stopping (Stop)");
#endif
				m_pFlowMeterCapture->DisableInterrupt(EEdge_Rising);	
            m_eStatus = ECleaningDeviceStatus_Stopping;
            return true;
        case ECleaningDeviceStatus_Stopping:
        case ECleaningDeviceStatus_Stopped:
#if TRACEALYZER != 0 && TRC_WATER != 0
				vTracePrintF(trcWater,"ClState = %d (Stop)",(int)m_eStatus);
#endif
#ifdef DBGPRINTF_WPUMP
				dbgprintf("Stop Water Pump, stopping or stoppes\n");
#endif
            return true;
        default:
            break;
    }
    return false;
}

#endif

#if WATERPUMP_ALTERNATE_HW != 0

// ----------------------------------------------------------------------------
//! \brief Disable the pump
bool WaterPumpDevice::Disable(void)
{
#ifdef DBGPRINTF_WPUMP
	dbgprintf("Disable Water Pump Device, Status = %d\n",(int)m_eStatus);
#endif
	BOARD_SetPumpPWM(0,0);
	BOARD_SetPumpPWM(1,0);
	if (BOARD_EnablePump(0,false) && BOARD_EnablePump(1,false))
	{		
#ifdef DBGPRINTF_WPUMP
		dbgprintf("Disable Water Pump Device - Disabled\n");
#endif
#if TRACEALYZER != 0 && TRC_WATER != 0
		vTracePrint(trcWater,"Pump 1 & 2 disabled (Disable)");
		vTracePrint(trcWater,"WaState -> Disabled (Disable)");
#endif
		m_eStatus = ECleaningDeviceStatus_Disabled;
		return true;
	}
#ifdef DBGPRINTF_WPUMP
	dbgprintf("Disable Water Pump Device - Error\n");
#endif
#if TRACEALYZER != 0 && TRC_WATER != 0
		vTracePrint(trcWater,"WaState -> Error (Disable)");
#endif
	m_eStatus = ECleaningDeviceStatus_Error;
	return false;
}

#else

// ----------------------------------------------------------------------------
//! \brief Disable the pump
bool WaterPumpDevice::Disable(void)
{
#ifdef DBGPRINTF_WPUMP
	dbgprintf("Disable Water Pump Device, Status = %d\n",(int)m_eStatus);
#endif
	m_Pump1.SetRatio(0, 1);
	m_Pump2.SetRatio(0, 1);
	if (m_Pump1.Disable() && m_Pump2.Disable())
	{		
#ifdef DBGPRINTF_WPUMP
		dbgprintf("Disable Water Pump Device - Disabled\n");
#endif
#if TRACEALYZER != 0 && TRC_WATER != 0
		vTracePrint(trcWater,"WaState -> Disabled (Disable)");
#endif
		m_eStatus = ECleaningDeviceStatus_Disabled;
		return true;
	}
#ifdef DBGPRINTF_WPUMP
	dbgprintf("Disable Water Pump Device - Error\n");
#endif
#if TRACEALYZER != 0 && TRC_WATER != 0
		vTracePrint(trcWater,"WaState -> Error (Disable)");
#endif
	m_eStatus = ECleaningDeviceStatus_Error;
	return false;
}

#endif

// ----------------------------------------------------------------------------
//! \brief Set the nominal water density [ul/m]
void WaterPumpDevice::SetWaterDensity(uint32_t _nDensity)
{
	m_nWaterDensity = _nDensity;
}

// ----------------------------------------------------------------------------
//! \brief Regulate the flow based on the speed. Linear speed is in cm/sec
void WaterPumpDevice::SetLinearSpeed(uint8_t _nAbsSd)
{
#ifdef TEST_MODE
   _nAbsSd = 100;
#endif    
   m_nAbsSd = _nAbsSd;
	if (m_nAbsSd < 3)
		m_nAbsSd = 0;    // don't pump water when speed is below 30 mm/s    
}

// ----------------------------------------------------------------------------
//! \brief Return the maximum measured current
uint32_t WaterPumpDevice::GetMaxCurrent(void)
{ 
	return m_Pump1.GetMaxCurrent(); 
}

// ----------------------------------------------------------------------------
//! \brief Reset the maximum measured current
void WaterPumpDevice::ResetMaxCurrent(void)
{
	m_Pump1.ResetMaxCurrent();
}

// ----------------------------------------------------------------------------
//! \brief Return true if the current is not too low (> 500 mA)
bool WaterPumpDevice::CheckCurrent(void)
{
	int32_t nCurrent = abs(m_Pump1.GetCurrent());
	return (nCurrent > 250);
}

// ----------------------------------------------------------------------------
//! \brief Check if tank is empty
void WaterPumpDevice::CheckWaterFlow(uint32_t _nTargetFlow)
{
	// Update flow measure
   uint64_t now = SystemTime::GetTime();
   if (m_nNextMeasureTime < now)
   {
		m_nNextMeasureTime = now + 1000; // Next measure in second

      // Compute actual flow
      int32_t nActualFlow = m_nFlowPulses*NB_MICROLITER_PER_PULSE;   // => ul/s
      m_nFlowPulses = 0;

      // Update totals and history
      m_nTotalActual -= m_anActualFlows[m_nMeasureIndex];
      m_nTotalActual += nActualFlow;
      m_anActualFlows[m_nMeasureIndex] = nActualFlow;
      
      m_nTotalTargeted -= m_anTargetFlows[m_nMeasureIndex];
      m_nTotalTargeted += _nTargetFlow;
      m_anTargetFlows[m_nMeasureIndex] = _nTargetFlow;

      m_nMeasureIndex = (m_nMeasureIndex+1) % NB_FLOW_MEASURES;
    
        // Check water flow (when measurable: flow big enough => at least one pulse every two second...)
#ifdef TEST_MODE
      m_bTankIsEmpty = (nActualFlow == 0);
#else
      if ((m_nTotalTargeted/NB_MICROLITER_PER_PULSE) >= MIN_PULSES_FOR_DETECTION)
      {
#if TEST_TANK_EMPTY != 0
			if (!m_DryRunEnabled)
				m_bTankIsEmpty |= m_nTotalActual == 0;
			else
				m_bTankIsEmpty = false;
#endif
      }
#endif 
	}
}

// ----------------------------------------------------------------------------
//! \brief Returns the Water Pump Device Name
const char * WaterPumpDevice::GetDeviceName(void)
{
	return Name;
}
