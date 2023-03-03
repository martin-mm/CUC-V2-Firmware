// ---------------------------------------------------------------------------
//! \package     CleaningUnit
//! \file        LiftDevice.cpp
//! \brief       Defines the class responsible for management of a lift device
//!
//! \copyright   Copyright (C) 2011-2012 BlueBotics SA
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Includes
#include <stdlib.h>
#include <stdio.h>
#include "LiftDevice.h"
#include "MotorDriver.h"
#include "AnalogInput.h"
#include "IO.h"
#include "board.h"

// #include "AnalogOutput.h" // YJE test...

#define DBGPRINTF_LIFT
#undef  DBGPRINTF_LIFT

#define LIFT_HALL_COUNT
#undef  LIFT_HALL_COUNT

#define ERROR_OVC_CNTR_LIMIT		50

// ----------------------------------------------------------------------------
// Constants
#define LIFT_PWM_AMPLITUDE 10000
#define LIFTMOTOR_HOMING_PWMRATIO 4000

// ----------------------------------------------------------------------------

//! \brief List event ids supported by LiftDevice.
typedef enum
{
	ELiftDeviceEventId_PWM = 100,
	ELiftDeviceEventId_HallSensor,
} ELiftDeviceEventId;

static LiftDevice	*  sLiftDevices[2];
static int				sLiftDevicesN = 0;

// ----------------------------------------------------------------------------
//! \brief Constructor
LiftDevice::LiftDevice(ECleaningUnitMgrErrors _eDeviceErrorId, MotorDriver &_liftMotor,
							  MotorDriver *_pDeviceMotor, HallSensorInput *_pHallSensor,
							  int _nLiftID)
	: m_LiftMotor(_liftMotor),						// set the Lift Motor
	  m_pDeviceMotor(_pDeviceMotor),				// set the Motor of the associated device
	  m_TMPGenerator(10, 1, _nLiftID),			// call the TMP generator constructor
	  m_PIDController(10, 0, 0,_nLiftID),		// call the PID controller constructor
	  m_nHomePosition(0),							// Homing Position
	  m_nRestPosition(0),							// Position after the Device moved out of the Homing Position
	  m_nWorkingPosition(0),						// Working Position
	  m_nHomingCurrentLimit(0),					// Current Limit for the Homing
	  m_nAdjCurrentLimit(0),						// Current Limit for the Adjust States
	  m_nHomingMaxDuration(0),						// Maximum Duration of the Homing
	  m_nHomingStartCurrentLimit(0),				// Minimum Current during Homing
	  m_nHomingStartDuration(0),					// Time Limit for Start Homing
	  m_nCurrentMax(0),								// Maximum Device Current in mA
	  m_nCurrentMin(250),							// Minimum Device Current in mA
	  m_pHallSensor(_pHallSensor),				// set the Hall sensor of the Lift Motor
     m_nLiftMotorInPosRange(8),					// Max. Deviation from Position
	  m_bEnaAdjust(false),							// Disables the Device Adjust
	  m_nLiftID(_nLiftID),							// ID of the Lift Device
	  m_bLiftUpDryRun(false)						// Lift Position in Dry Run Mode
{	
   dbgprintf("Lift Device Constructor ...\n");	
	// Disable the Device
	Disable();
	// Set the output saturation of the PID controller
	m_PIDController.SetOutputSaturation(-LIFT_PWM_AMPLITUDE / 2, LIFT_PWM_AMPLITUDE / 2);
	// Delta Position initially is 0 (nothing to do)
	m_nDeltaPosition = 0;
	// Register the interrupt handler of the Lift Motor (PWM Timer)
	m_LiftMotor.RegisterHandler(this, ELiftDeviceEventId_PWM);
	// Register the interrupt handler of the Hall Sensor Count Interrupt (GPIO)
	m_pHallSensor->RegisterHandler(this, ELiftDeviceEventId_HallSensor);
	// Get the actual system time in ms
	m_nDelayPulse = SystemTime::GetTime();
	// Register the Lift Device
	if (sLiftDevicesN < 2)
		sLiftDevices[sLiftDevicesN++] = this;
#if USE_IIR_FILTER != 0
	m_nAvgCurrentFB = new IIR<int32_t>(
		3.9130215E-05F,		// b0
		7.8260411E-05F,		// b1
		3.9130215E-05F,		// b2
		-1.982229F,				// a1
		0.9823854F				// a2
		);
#endif
#if TRACEALYZER != 0 && TRC_LIFT != 0 
		sprintf(m_char,"LIFT %d",m_nLiftID);
		trcLift = xTraceRegisterString(m_char);
		vTracePrint(trcLift,"DevState -> Idle (Constructor)");
#endif
	// This is the Lift Device Initial State
	m_eLiftState = ELiftDeviceStatus_Idle;
	// Reset the Error Source
	m_eLiftStateErrSrc = ELiftDeviceStatus_Idle;
	// Reset the Overcurrent
	m_OverCurrent_Test = 0; 
	// Reset the Overcurrent Recovery Counter
	m_RecoveryTime = 0;
	m_OC_Cnt = 0;
	m_Hold = false;		// Relays are assumed to be on and EMStop is deasserted
	m_ErrorWhileMoving = false;
#ifdef USE_DWELLTIME
	bLiftTimeout = false;
#endif
#if TRACEALYZER != 0 && TRC_LIFT != 0 
	m_OldCleaningState = ECleaningDeviceStatus_Disabled;
	m_OldLiftState = ELiftDeviceStatus_Error;
#endif
#ifdef USE_DWELLTIME
	// Presets the Dwell Time Array and Ptr
	for (int i = 0;i < N_MOTOR_CYCLES_CONT;i++)
		DwellMoveTimes[i] = -1;
	DwellTime = -1;
	DwellMovTimePtr = 0;
#endif
   dbgprintf("... Lift Device Constructor done\n");	
}

// ----------------------------------------------------------------------------
//! \brief Tests for a maximum current violation of the Lift Device
bool inline LiftDevice::TestOvercurrent(void)
{
int32_t 	nCurrent;

	nCurrent = m_LiftMotor.GetCurrent();
#if TRACEALYZER != 0 && TRC_LIFT != 0 && TRC_LIFT_SHOW_MEAS_CURR != 0
	vTracePrintF(trcLift,"Lift Current = %d mA",nCurrent);
#endif
	if (m_nCurrentMax > 0 && abs(nCurrent) > m_nCurrentMax && m_eLiftState != ELiftDeviceStatus_Error)
	{
		m_OC_Cnt += (abs(nCurrent) - m_nCurrentMax) / 1000 + 1;
#if TRACEALYZER != 0 && TRC_LIFT != 0 && TRC_LIFT_SHOW_MEAS_CURR != 0
		vTracePrintF(trcLift,"Overcurrent, Cnt = %d",m_OC_Cnt);
#endif
		if (m_OC_Cnt > MAX_OC_CNT)
		{
#if TRACEALYZER != 0 && TRC_LIFT != 0 && TRC_LIFT_SHOW_MEAS_CURR != 0
			vTracePrintF(trcLift,"Overcurrent, Lift Current = %d mA",nCurrent);
#endif
			m_LiftMotor.SetRatio(0, 1);								// Stop the Lift Motor
			m_IsMoving = false;											// Lift is not moving anymore
			m_eLiftState = ELiftDeviceStatus_Error;				// Set the Lift Motor FSM state to "Error"
			m_eStatus = ECleaningDeviceStatus_Error;				// Set the Cleaning Device state to "Error"
			m_Lift_Error_Reason = eLiftDeviceErrReason_OVC;		// Error Reason for the Lift FSM
			m_eLiftStateErrSrc = m_eLiftState;						// Set the state that caused the error
			m_OverCurrent_Test = nCurrent;							// Store the current that cause the Overcurrent Error
			m_OC_Cnt = 0;													// Reset the Overcurrent Counter
			m_RecoveryTime = SystemTime::GetTime() + 
					MAX_OC_RECOVERY_TIME;								// Set the Recovery Time (ms)
			return false;
		}
	}
	else
	{
		if (m_OC_Cnt > 0)
		{
			m_OC_Cnt--;
		}
	}
	return true;
}

//// ----------------------------------------------------------------------------
////! \brief Handles the I2t-behavior of the motor
//void LiftDevice::HandleI2tBehavior(bool Reset)
//{
//uint32_t		DeltaTime;
//uint32_t		Now;

//	if (Reset)
//	{
//		m_MotorI2t = 0;
//		m_MotorStartTime = SystemTime::GetTime();
//	}
//	else
//	{
//		Now = SystemTime::GetTime() - m_MotorStartTime;
//	DeltaTime = m_MotorRunTime - Now;
//		m_MotorI2t += abs(m_LiftMotor.GetCurrent()) * DeltaTime;
//		m_MotorRunTime = Now;	
//	} 
//}

// ----------------------------------------------------------------------------
//! \brief Event handler (Called by the Interrupt Handler)
void LiftDevice::HandleEvent(EventSource *_pSource, uint32_t _nEventId, uint32_t _nData)
{
	// If the Dry Run is enabled then the range of the Lift Position Error
	// is set to a smaller value then in the normal Cleaning Mode (The
	// Lift Position is in Hall Sensor Pulses)
	if (!m_DryRunEnabled)				// Check if the Dry Run is enabled
   {
		m_nLiftMotorInPosRange = 8;	// Set the In Position Range (max. Position Error)
   }
   else
   {
		m_nLiftMotorInPosRange = 4;	// Set the In Position Range (max. Position Error)
   }
	
	// ******************************************
	// * Event from the Hall Sensor Capture     *
	// ******************************************
	if (_nEventId == ELiftDeviceEventId_HallSensor)
	{
		m_cnt_pulse++;		// Count up the Hall Sensor Pulse Counter (only used for debugging)
#ifdef LIFT_HALL_COUNT
#if TRACEALYZER != 0 && TRC_LIFT != 0
		vTracePrintF(trcLift,"Hall Sensor Count %d",m_cnt_pulse);
#endif
#endif
		// Count only when we are moving or homing
		if (m_eLiftState == ELiftDeviceStatus_StartHoming ||
		    m_eLiftState == ELiftDeviceStatus_Homing ||
		    m_eLiftState == ELiftDeviceStatus_Moving)
		{
			// if the system time is higher than the current time plus the allowed delay time
			if (SystemTime::GetTime() > m_nDelayPulse)
			{
				// Calculate the delta position according to the rotation direction of the lift motor
				// This is either 1 or -1
				m_nDeltaPosition += (m_LiftMotor.GetMode() == EMotorDriverMode_Clockwise ? -1 : 1);
				// Set the delay in ms for the next Hall Sensor Pulse to occur. If this delay
				// did not expire the delta position is zeroed and the capture interrupt for
				// the Hall Sensor is disabled
				#if LIFTMOTOR_VERSION == 1
				m_nDelayPulse = SystemTime::GetTime() + 6;
				#endif
				#if LIFTMOTOR_VERSION == 2
				m_nDelayPulse = SystemTime::GetTime() + 8;
				#endif
			}
		}
		else
		{
			// Otherwise reset the counter
			m_nDeltaPosition = 0;
			// And disable the hall sensor capture interrupt
#if BOARD_VERSION == 10
			m_pHallSensor->Configure(EEdge_Rising, false);
#endif
#if BOARD_VERSION == 11
			m_pHallSensor->DisableInterrupt(EEdge_Rising);	
#endif
		}
		return;
	}

	// ******************************************
	// * Event from the PWM Timer               *
	// ******************************************
	if (_nEventId == ELiftDeviceEventId_PWM)
	{
		// calculate the actual position from the previos value plus the delta position
		// (the actual count value from the hall sensor pulse counter, either +1 or -1)
		m_nActualPosition += m_nDeltaPosition;
		// and reset the delta position to 0
		m_nDeltaPosition = 0;
		// increase the internal time by one PWM period duration (in ms)
		m_nTime += m_LiftMotor.GetPeriodDuration() / 1000;
#if TRACEALYZER != 0 && TRC_LIFT != 0
		if (m_OldCleaningState != m_eStatus)
		{
			vTracePrintF(trcLift,"Cleaning State changed from %d to %d",m_OldCleaningState,m_eStatus);
			m_OldCleaningState = m_eStatus;
		}
		if (m_OldLiftState != m_eLiftState) 
		{
			vTracePrintF(trcLift,"Lift State changed from %d to %d",m_OldLiftState,m_eLiftState);
			m_OldLiftState = m_eLiftState;
		}
#endif

		// Compute new position by executing the Trapezoid Motion Generator
		m_TMPGenerator.Execute(m_nTime);
        
		// Update the current measurement of the external motor
		// (This is the associated device, usually either the
		// Brush Motor or the Suction Motor).
		// If not used, m_pDeviceMotor is NULL.
		// This is only done if the Lift Device is in the
		// AdjustUp, AdjustDown or AdjustIdle State, which
		// can only be reached if the associated Device is in
		// the Running Mode (consuming current)
		int32_t nExtCurrentFB = 0;
		if ((nullptr != m_pDeviceMotor) && 
			 (m_eLiftState == ELiftDeviceStatus_AdjustUp ||
			  m_eLiftState == ELiftDeviceStatus_AdjustIdle ||
			  m_eLiftState == ELiftDeviceStatus_AdjustDown)) 
		{
#if TRACEALYZER != 0 && TRC_LIFT != 0 && TRC_LIFT_SHOW_MEAS_CURR != 0
			int ExtCurrent = abs(m_pDeviceMotor->GetCurrent());
#if USE_IIR_FILTER != 0
		   nExtCurrentFB = m_nAvgCurrentFB->Update(ExtCurrent);
#else
		   nExtCurrentFB = m_nAvgCurrentFB.Update(ExtCurrent);
#endif
			if (ExtCurrent > 200 || nExtCurrentFB > 200)
				vTracePrintF(trcLift,"Ext. Device Cur. = %d mA, Mean = %d mA",ExtCurrent,nExtCurrentFB);
#else
#if USE_IIR_FILTER != 0
			nExtCurrentFB = m_nAvgCurrentFB->Update(abs(m_pDeviceMotor->GetCurrent()));
#else
			nExtCurrentFB = m_nAvgCurrentFB.Update(abs(m_pDeviceMotor->GetCurrent()));
#endif
#endif
		}
		if (m_Hold)		// Active Hold from the Cleaning Manager resets the State of the FSM
		{
			m_eLiftState = ELiftDeviceStatus_Idle;
			m_eStatus = ECleaningDeviceStatus_Stopped;
			m_IsMoving = false;
			m_LiftMotor.SetRatio(0, 1);
		}
      if (bLiftTimeout)
		{
			m_eLiftState = ELiftDeviceStatus_Error;
			bLiftTimeout = false;
		}
		// *********************************************
		// * This is the PWM Timer Event State machine *
		// *********************************************
		switch (m_eLiftState)
		{
			// *** FSM State: Idle ***
			case ELiftDeviceStatus_Idle:
				{
					// this will set the Lift Motor PWM to 0
					m_LiftMotor.SetRatio(0, 1);
					m_IsMoving = false;
					break;
				}
			// *** FSM State: Start Homing ***
			case ELiftDeviceStatus_StartHoming:
				{
					if (!m_EnableMoving)
					{
						if (m_ErrorWhileMoving)
						{
							m_ErrorWhileMoving = false;
						}
						m_IsMoving = true;
						// Set the Lift Motor PWM to the LIFT PWM AMPLITUDE and the Homing Direction
						m_LiftMotor.SetRatioAndMode(LIFTMOTOR_HOMING_PWMRATIO, LIFT_PWM_AMPLITUDE, m_eHomingDirection);
						// Measure the Lift Motor Actual Current.
						int32_t nCurrent = m_LiftMotor.GetCurrent();
						// did the time limit for the Start Homing expire?
						if (m_nHomingStartDuration == 0 || (m_nWaitDeadline < SystemTime::GetTime()) || (abs(nCurrent) > m_nHomingStartCurrentLimit))
						{
							// The time limit is set to the maximum time allowed for the homing
							m_nWaitDeadline = SystemTime::GetTime() + m_nHomingMaxDuration;
							// The FSM goes to the Homing State
#if TRACEALYZER != 0 && TRC_LIFT != 0
							vTracePrint(trcLift,"DevSt Start Homing -> Homing (FSM)");
#endif
//							HandleI2tBehavior(true);		// Reset the Motor I2t Integral
//							m_MotorRunTime = 0;	// Re
							// Store the Motor Movement Time and check if we have to wait for the Dwell Time
#ifdef USE_DWELLTIME
							if (StoreDwellMovTime())
							{
								m_eLiftState = ELiftDeviceStatus_Homing;
							}
							else
							{
#if TRACEALYZER != 0 && TRC_LIFT != 0
								vTracePrint(trcLift,"Dwell Time Error (FSM, Homing)");
#endif
								m_eLiftState = ELiftDeviceStatus_DwellTime;	// go to the Dwell Time State
							}
#else
							m_eLiftState = ELiftDeviceStatus_Homing;
#endif
						}
					}
					break;
				}
			// *** FSM State: Homing ***
			case ELiftDeviceStatus_Homing:
				{
					// Reached the highest point? Homing is based on current or timeout
					// Measure the current of the Lift Motor
//					HandleI2tBehavior();
					int32_t nCurrent = m_LiftMotor.GetCurrent();
					// if the current is above the Homing Current Limit the FSM leaves this state
					if ((abs(nCurrent) > m_nHomingCurrentLimit) || (m_nWaitDeadline < SystemTime::GetTime()))
					{
						m_LiftMotor.SetRatio(0, LIFT_PWM_AMPLITUDE);		// set the Lift Motor PWM to 0
						m_IsMoving = false;
						m_nActualPosition = m_nHomePosition;				// set the actual position to the homing position
#if TRACEALYZER != 0 && TRC_LIFT != 0
						vTracePrintF(trcLift,"Home Position: %d, Rest Position: %d",m_nHomePosition,m_nRestPosition);
#endif
						m_TMPGenerator.SetPosition(m_nHomePosition);		// set the start position of the TMP Generator to the homing position
						m_TMPGenerator.MoveTo(m_nRestPosition);			// set the target position of the TMP Generator to the rest position
						m_cnt_pulse = 0;											// reset the Hall Sensor Pulses debug counter
#if BOARD_VERSION == 11
						m_pHallSensor->EnableInterrupt(EEdge_Rising);	// Enable the Interrupt for the Lift Device Hall Sensor Pulses
#endif
						if (SystemTime::GetTime() > m_RecoveryTime && m_OC_Cnt == 0)		// Are we waiting for an Overcurrent Recovery?
						{
#if TRACEALYZER != 0 && TRC_LIFT != 0
							vTracePrint(trcLift," DevSt Homing -> Moving (FSM)");
#endif
							m_RecoveryTime = 0;
							m_MovingTimer = SystemTime::GetTime(); 
#ifdef USE_DWELLTIME
							if (StoreDwellMovTime())
							{
								m_eLiftState = ELiftDeviceStatus_Moving;		// go to the Moving State
							}
							else
							{
								m_eLiftState = ELiftDeviceStatus_DwellTime;	// go to the Dwell Time State
							}
#else
							m_eLiftState = ELiftDeviceStatus_Moving;		// go to the Moving State
#endif
						}
					}
					break;
				}
			// *** FSM State: Moving ***
			case ELiftDeviceStatus_Moving:
				{
#ifdef USE_DWELLTIME
					if (CheckDwellMovementTime(Now()))
					{
#endif
					// The time limit is compared to the maximum time allowed for the moving
#ifdef USE_DWELLTIME
						if (SystemTime::GetTime() > m_MovingTimer + MAX_MOVINT_TIME || bLiftTimeout)
#else
						if (SystemTime::GetTime() > m_MovingTimer + MAX_MOVINT_TIME)
#endif
						{
							bLiftTimeout = false;
#if TRACEALYZER != 0 && TRC_LIFT != 0
							vTracePrint(trcLift,"Timeout during Lift Moving occured");
#endif
							m_LiftMotor.SetRatio(0, 1);									// Stop the Lift Motor
							m_IsMoving = false;												// Lift is not moving anymore
							m_eLiftState = ELiftDeviceStatus_Error;					// Set the Lift Motor FSM state to "Error"
							m_eStatus = ECleaningDeviceStatus_Error;					// Set the Cleaning Device state to "Error"
							m_Lift_Error_Reason = eLiftDeviceErrReasonMovTimeout;	// Error Reason for the Lift FSM
							m_eLiftStateErrSrc = m_eLiftState;							// Set the state that caused the error
						}
						if (!m_EnableMoving)
						{	
							m_IsMoving = true;
							// if there is no overcurrent condition
							uint8_t MotorState = m_LiftMotor.GetMotorState();
							// if the TMP Generator has stopped moving and the PID controller error is below the 
							// Max Position Error threshold
							if ((!m_TMPGenerator.IsMoving() && (abs(m_PIDController.GetError()) < m_nLiftMotorInPosRange)) || m_LiftMotor.isBlocked())
							{
#if TRACEALYZER != 0 && TRC_LIFT != 0
							  vTracePrint(trcLift," ClDevSt -> End Of Move (FSM)");
							  vTracePrintF(trcLift," Counts: %d",m_cnt_pulse);
#endif
							  m_eStatus = m_eEndOfMoveStatus;				// Set the Cleaning Device Status to End Of Moving
							  m_LiftMotor.SetRatio(0, 1);						// Set the Lift Motor PWM to zero
							  m_IsMoving = false;								// We are not moving anymore
							  // If an assotiated device (Cleaning Device) is present (e.g. Brush Motor
							  // or Suction Motor) and if the Cleaning Device Status is "Running" and the 
							  // Lift Motor is not in the upper position	
							  if (NULL != m_pDeviceMotor && !IsUp() && 
									m_eStatus == ECleaningDeviceStatus_Running && m_bEnaAdjust)
							  {
#if TRACEALYZER != 0 && TRC_LIFT != 0
									vTracePrint(trcLift," DevSt Moving -> Adj Idle (FSM)");
#endif
									m_eLiftState = ELiftDeviceStatus_AdjustIdle;	// set the FSM state to "Adjust Idle"
							  }
							  else
							  {
#if TRACEALYZER != 0 && TRC_LIFT != 0
									vTracePrint(trcLift," DevSt Moving -> Idle (FSM)");
#endif
									m_eLiftState = ELiftDeviceStatus_Idle;	// All done, go to the Idle State	
							  }									  
							}
							// Get the desired position from the Trapezoid Movement Generator (TMP)
							// and execute the PID controller using the TMP Generator's position
							// (Target Position) as the Set Value of the PID controller and the
							// actual position as the Actual Value of the PID controller.
							// The output of the PID controller is used as the Set Value of
							// the PWM controller that is assigned to the Lift Device.
	//						HandleI2tBehavior();
							int32_t nCmd = m_PIDController.Execute(m_nTime, m_TMPGenerator.GetPosition(), m_nActualPosition);
							// Determine the moving direction used to control the Lift Device
							EMotorDriverMode eMode = (nCmd > 0 ? EMotorDriverMode_CounterClockwise : EMotorDriverMode_Clockwise);
							// Set the Lift Device PWM
							m_LiftMotor.SetRatioAndMode(abs(nCmd), LIFT_PWM_AMPLITUDE, eMode);
						}
						break;
					}
#ifdef USE_DWELLTIME
				}
#endif
			// *** FSM State: Adjust Up ***
			case ELiftDeviceStatus_AdjustUp:
				{
					// Set the PWM and Moving Direction of the Lift Motor
					m_LiftMotor.SetRatioAndMode(LIFTMOTOR_HOMING_PWMRATIO, LIFT_PWM_AMPLITUDE, EMotorDriverMode_Clockwise);
					
					//if (nExtCurrentFB < m_nExtCurRef) m_eLiftState = ELiftDeviceStatus_AdjustIdle;
					//if (abs(m_LiftMotor.GetCurrent()) > m_nHomingCurrentLimit) m_eLiftState = ELiftDeviceStatus_AdjustIdle;
					
					// if the current of the associated device is below the limit or the
					// Lift Motor current is above the Adjust Current Limit
					int maxAdjustCurrent = abs(m_LiftMotor.GetCurrent());
					if (nExtCurrentFB < m_nExtCurRef)
					{
#if TRACEALYZER != 0 && TRC_LIFT != 0
						vTracePrintF(trcLift,"ClErr: ExtCurrentFB < ExtCurRef %d",(int)nExtCurrentFB); 
						vTracePrint(trcLift,"ClDevSt AdjUp -> Error (FSM)");
#endif
						m_eLiftState = ELiftDeviceStatus_Idle;						// set the FSM state to the Idle State
						m_eStatus = ECleaningDeviceStatus_Error;					// set the Cleaning Device State to "Error"
						m_Lift_Error_Reason = eLiftDeviceErrReason_AdjCurL;	// Error Reason for the Lift FSM
						m_eLiftStateErrSrc = ELiftDeviceStatus_AdjustUp;		// set the state that caused the error
					}
					else
					{
						if (maxAdjustCurrent > m_nAdjCurrentLimit)
						{
							if (m_AdjErrorCntr > 50)		// must be present for at least 100ms
							{
								m_IsMoving = false;
								m_LiftMotor.SetRatio(0, 1);					// set the Lift Motor PWM to 0
#if TRACEALYZER != 0 && TRC_LIFT != 0
								vTracePrintF(trcLift,"ClErr: CurrentFB > AdjustCurLim %d, Cnt = &d",maxAdjustCurrent,m_AdjErrorCntr); 
								vTracePrint(trcLift,"DevSt AdjustUp -> Idle (FSM)");
								vTracePrintF(trcLift,"LiftCur %d",(int)(m_LiftMotor.GetCurrent()));
								vTracePrint(trcLift,"ClDevSt AdjUp -> Error (FSM)");
#endif
								m_eLiftState = ELiftDeviceStatus_Idle;						// set the FSM state to the Idle State
								m_eStatus = ECleaningDeviceStatus_Error;					// set the Cleaning Device State to "Error"
								m_Lift_Error_Reason = eLiftDeviceErrReason_AdjCurH;	// Error Reason for the Lift FSM
								m_eLiftStateErrSrc = ELiftDeviceStatus_AdjustUp;		// set the state that caused the error
							}
							else
								m_AdjErrorCntr++;
						}
						else
						{
							m_AdjErrorCntr = 0;
						}
					}
					break;
				}
			// *** FSM State: Adjust Idle ***
			case ELiftDeviceStatus_AdjustIdle:
				{
				   m_IsMoving = false;
					m_LiftMotor.SetRatio(0, 1);					// set the Lift Motor PWM to 0
					// if the current of the associated device is above the limit
					if (nExtCurrentFB > m_nExtCurRefToLift)
					{
#if TRACEALYZER != 0 && TRC_LIFT != 0
						vTracePrint(trcLift," DevSt Adj Idle -> Adj Up (FSM)");
#endif
						m_eLiftState = ELiftDeviceStatus_AdjustUp;	// go to the Adjust Up State
					}
					//if (abs(m_pDeviceMotor->GetCurrent()) > 1000 && nExtCurrentFB < m_nExtCurRefToLower) m_eLiftState = ELiftDeviceStatus_AdjustDown;
				}
				break;
			// *** FSM State: Adjust Down ***
			case ELiftDeviceStatus_AdjustDown:
				{
					// Set the Lift Motor PWM and direction
					m_LiftMotor.SetRatioAndMode(LIFTMOTOR_HOMING_PWMRATIO, LIFT_PWM_AMPLITUDE, EMotorDriverMode_CounterClockwise);
					// if the current of the associated device is above the limit
					if (nExtCurrentFB >= m_nExtCurRef)
					{
#if TRACEALYZER != 0 && TRC_LIFT != 0
						vTracePrint(trcLift," DevSt Adj Down -> Adj Idle (FSM)");
#endif
						m_eLiftState = ELiftDeviceStatus_AdjustIdle;	// go to the Adjust Idle State
					}
					// if the Lift Motor current is above the Homing Current Limit
					if (abs(m_LiftMotor.GetCurrent()) > m_nHomingCurrentLimit)
					{
#if TRACEALYZER != 0 && TRC_LIFT != 0
						vTracePrint(trcLift," DevSt Adj Down -> Adj Idle (FSM)");
#endif
						m_eLiftState = ELiftDeviceStatus_AdjustIdle;	// go to the Adjust Idle State
					}
				}
            break;
			// *** FSM State: Error ***
			case ELiftDeviceStatus_Error:
//				m_eStatus = ECleaningDeviceStatus_Error;			// set the Cleaning Device State to "Error"
				m_LiftMotor.SetRatio(0, 1);							// set the Lift Motor PWM to 0
				break;
#ifdef USE_DWELLTIME
			// *** FSM State: Dwell Time ***
			case ELiftDeviceStatus_DwellTime:
				if (WaitDwellTime())
					m_eLiftState = ELiftDeviceStatus_Idle;			// go back to the Idle State
//				m_Lift_Error_Reason = eLiftDeviceErrReasonMovTimeout;	// Error Reason for the Lift FSM
				break;
#endif
			// *** FSM State: Default State (should never get reached) ***
			default:															// Default State (should not occur!)
         {
#if TRACEALYZER != 0 && TRC_LIFT != 0
				vTracePrint(trcLift," DevSt Default -> Error (FSM)");
#endif
				m_LiftMotor.SetRatio(0, 1);					// set the Lift Motor PWM to 0
				m_eStatus = ECleaningDeviceStatus_Error;	// set the Cleaning Device State to "Error"
				break;
         }
		}	// End of Switch
		// ********************************************************
		// * This is the end of the PWM Timer Event State machine *
		// ********************************************************
		
		// Test for Overcurrent
		TestOvercurrent();
//		if (abs(nCurrent) < m_nCurrentMin)
//			Disable(); // YJE Todo: check empty load !!!
	}
}

// ----------------------------------------------------------------------------
//! \brief Gets the lift device FSM state
ELiftDeviceStatus LiftDevice::GetFSMstate(void)
{
	return m_eLiftState;
}

// ----------------------------------------------------------------------------
//! \brief Enable the device
bool LiftDevice::Enable()
{
	int32_t nCurrent = m_LiftMotor.GetCurrent();
#ifdef DBGPRINTF_LIFT
	dbgprintf("Lift Device (%s) Enable, Initial Current is %dmA\n",GetDeviceName(),nCurrent);
#endif
	m_nTime = 0;
	m_TMPGenerator.SetPosition(0);
		
	// Start homing
	Lift();
#if TRACEALYZER != 0 && TRC_LIFT != 0
	vTracePrint(trcLift," ClDevSt -> Initializing (Enable)");
	vTracePrint(trcLift," CleanDevEOMState -> Stopped (Enable)");
#endif
	m_eStatus = ECleaningDeviceStatus_Initializing;
	m_eEndOfMoveStatus = ECleaningDeviceStatus_Stopped;

 	if (m_LiftMotor.Enable())
	{
		return true;
	}
#if TRACEALYZER != 0 && TRC_LIFT != 0
	vTracePrint(trcLift," ClDevSt -> Error (Enable)");
#endif
	m_eStatus = ECleaningDeviceStatus_Error;
	return false;
}

// ----------------------------------------------------------------------------
//! \brief Start the device
bool LiftDevice::Start()
{
#ifdef DBGPRINTF_LIFT
	dbgprintf("Lift Device (%s) Start\n",GetDeviceName());
#endif
	 if (m_Hold) // Device cannot start due to EMStop
	    return false;
   switch (m_eStatus)
   {
		case ECleaningDeviceStatus_Stopped:
      case ECleaningDeviceStatus_Error:
			Lower();
#if TRACEALYZER != 0 && TRC_LIFT != 0
			vTracePrint(trcLift," ClDevSt -> Starting (Start)");
			vTracePrint(trcLift," CleanDevEOMState -> Running (Start)");
#endif
         m_eEndOfMoveStatus = ECleaningDeviceStatus_Running;
         m_eStatus = ECleaningDeviceStatus_Starting;
         return true;
      case ECleaningDeviceStatus_Starting:
      case ECleaningDeviceStatus_Running:
         return true;
      default:
         break;
   }
   return false;
}

// ----------------------------------------------------------------------------
//! \brief Stop the device
bool LiftDevice::Stop()
{
#ifdef DBGPRINTF_LIFT
	 dbgprintf("Lift Device (%s) Stop\n",GetDeviceName());
#endif
	 if (m_Hold) // Device is already stopped due to EMStop
	    return true;
	 switch (m_eStatus)
	 {
		  case ECleaningDeviceStatus_Running:
		  case ECleaningDeviceStatus_Starting:
				Lift();
#if TRACEALYZER != 0 && TRC_LIFT != 0
				vTracePrint(trcLift," ClDevSt -> Stopping (Stop)");
				vTracePrint(trcLift," CleanDevEOMState -> Stopped (Stop)");
#endif
				m_eStatus = ECleaningDeviceStatus_Stopping;
				m_eEndOfMoveStatus = ECleaningDeviceStatus_Stopped;
				return true;
		  case ECleaningDeviceStatus_Stopping:
		  case ECleaningDeviceStatus_Stopped:
				return true;
		  default:
				break;
	 }
    return false;
}

// ----------------------------------------------------------------------------
//! \brief Disable the device
bool LiftDevice::Disable()
{
#ifdef DBGPRINTF_LIFT
	dbgprintf("Lift Device (%s) Disable\n",GetDeviceName());
#endif
#if TRACEALYZER != 0 && TRC_LIFT != 0
	vTracePrint(trcLift," DevState -> Idle (Disable)");
#endif
   m_eLiftState = ELiftDeviceStatus_Idle;	
	if (m_LiftMotor.Disable())
	{
		m_IsMoving = false;
#if TRACEALYZER != 0 && TRC_LIFT != 0
		vTracePrint(trcLift," ClDevSt -> Disabled (Disable)");
#endif
		m_eStatus = ECleaningDeviceStatus_Disabled;
		return true;
	}
#if TRACEALYZER != 0 && TRC_LIFT != 0
	vTracePrint(trcLift," ClDevSt -> Error (Disable)");
#endif
	m_eStatus = ECleaningDeviceStatus_Error;
	return false;
}

// ----------------------------------------------------------------------------
//! \brief Lift up the device
bool LiftDevice::Lift()
{
#ifdef DBGPRINTF_LIFT
	dbgprintf("Lift Device (%s) Lift Up\n",GetDeviceName());
#endif
#ifdef USE_DWELLTIME
	if (!IsWaitingForDwellTime())
	{
#endif
		if (!IsExecutingCommand())
		{
#if TRACEALYZER != 0 && TRC_LIFT != 0
			vTracePrint(trcLift,"Lift: Start Homing");
#endif
			m_eLiftState = ELiftDeviceStatus_StartHoming;
			m_eEndOfMoveStatus = ECleaningDeviceStatus_Running;
			m_nWaitDeadline = SystemTime::GetTime() + m_nHomingStartDuration;
#ifdef DBGPRINTF_LIFT
			dbgprintf("Lift device start homing (lift up), timeout = %dms ...\n",
						 m_nHomingStartDuration);
#endif
		}
//		if (m_DryRunEnabled)
//		{
//			m_bLiftUpDryRun = true;
//#if TRACEALYZER != 0 && TRC_LIFT != 0
//			vTracePrint(trcLift,"DryRun: Lift");
//#endif
//		}
		return true;
#ifdef USE_DWELLTIME
	}
	else
	{
#if TRACEALYZER != 0 && TRC_LIFT != 0
		vTracePrint(trcLift,"Lift: Waiting for Dwell Time");
#endif
		return false;
	}
#endif
}

// ----------------------------------------------------------------------------
//! \brief Lower down the device
bool LiftDevice::Lower()
{
#ifdef DBGPRINTF_LIFT
	dbgprintf("Lift Device (%s) Lower Down\n",GetDeviceName());
#endif
//	if (!m_DryRunEnabled)
//	{
#ifdef USE_DWELLTIME
		if (!IsWaitingForDwellTime())
		{
#endif
			if (!IsExecutingCommand())
			{
				m_TMPGenerator.MoveTo(m_nWorkingPosition);
#if TRACEALYZER != 0 && TRC_LIFT != 0
				vTracePrintF(trcLift,"Lower: from %d to %d",m_nActualPosition,m_nWorkingPosition);
#endif
				if (SystemTime::GetTime() > m_RecoveryTime && m_OC_Cnt == 0)		// Are we waiting for an Overcurrent Recovery?
				{
					m_RecoveryTime = 0;
//					HandleI2tBehavior(true);		// Reset the Motor I2t Integral
					// Store the Motor Movement Time and check if we have to wait for the Dwell Time
#ifdef USE_DWELLTIME
					if (StoreDwellMovTime())
					{
#endif
						m_eLiftState = ELiftDeviceStatus_Moving;
						m_MovingTimer = SystemTime::GetTime(); 
						m_eEndOfMoveStatus = ECleaningDeviceStatus_Running;
#if BOARD_VERSION == 10
						m_pHallSensor->Configure(EEdge_Rising, true);
#endif
#if BOARD_VERSION == 11
						m_pHallSensor->EnableInterrupt(EEdge_Rising);	
#endif
#ifdef USE_DWELLTIME
					}
					else
					{
#if TRACEALYZER != 0 && TRC_LIFT != 0
						vTracePrint(trcLift,"Dwell Time Error (FSM, Homing)");
#endif
						m_eLiftState = ELiftDeviceStatus_DwellTime;	// go to the Dwell Time State
					}
#endif
#ifdef DBGPRINTF_LIFT
					dbgprintf("Lift device moving (lower down) ...\n");
#endif
				}
				else
				{
#if TRACEALYZER != 0 && TRC_LIFT != 0
					vTracePrint(trcLift,"Lower: Still waiting for Error Recovery");
#endif
					return false;
				}
			}	
//   	}
//		else
//		{
//			m_bLiftUpDryRun = false;
//#if TRACEALYZER != 0 && TRC_LIFT != 0
//			vTracePrint(trcLift,"DryRun: Lower");
//#endif
//		}
#ifdef USE_DWELLTIME
	}		
	else
	{
#if TRACEALYZER != 0 && TRC_LIFT != 0
		vTracePrint(trcLift,"Lower: Waiting for Dwell Time");
#endif
		return false;
	}
#endif
	return true;
}

// ----------------------------------------------------------------------------
//! \brief Recover from the Device FSM Error State
bool LiftDevice::RecoverFromError(void)
{
	if (m_eLiftState == ELiftDeviceStatus_Error)
	{
		m_eLiftState = ELiftDeviceStatus_StartHoming;
	}
	return true;
}

// ----------------------------------------------------------------------------
//! \brief Checks if the Device FSM is in the Error State
bool LiftDevice::IsInError(void)
{
	return m_eLiftState == ELiftDeviceStatus_Error;
}

// ----------------------------------------------------------------------------
//! \brief Returns true is we are currently moving the lift
bool LiftDevice::IsExecutingCommand(void) 
{ 
	return (ELiftDeviceStatus_StartHoming == m_eLiftState ||
           ELiftDeviceStatus_Homing == m_eLiftState ||
           ELiftDeviceStatus_Moving == m_eLiftState);
}

#ifdef USE_DWELLTIME
// ----------------------------------------------------------------------------
//! \brief Returns true is we are currently moving the lift
bool LiftDevice::IsWaitingForDwellTime(void) 
{ 
	return ELiftDeviceStatus_DwellTime == m_eLiftState;
}
#endif

// ----------------------------------------------------------------------------
//! \brief Return the maximum measured current
uint32_t LiftDevice::GetMaxCurrent()
{ 
	return m_LiftMotor.GetMaxCurrent();
}

// ----------------------------------------------------------------------------
//! \brief Reset the maximum measured current
void LiftDevice::ResetMaxCurrent()
{
    m_LiftMotor.ResetMaxCurrent();
}

// ----------------------------------------------------------------------------
//! \brief Returns true is the lift is in the up position 
bool LiftDevice::IsUp()
{
//	if (!m_bLiftUpDryRun)
		// In Normal Mode the Lift is up if the Position is in the correct Position Range
		return (abs(m_nActualPosition - m_nRestPosition) < (m_nLiftMotorInPosRange * 5));
//	else
//	{
		// In DryRun-Mode we do not move the Lift down but fake the isUp state
//#if TRACEALYZER != 0 && TRC_LIFT != 0
//		if (m_bLiftUpDryRun)
//			vTracePrint(trcLift,"DryRun: Lift is up");
//		else
//			vTracePrint(trcLift,"DryRun: Lift is down");
//#endif
//		return m_bLiftUpDryRun;
//	}
}

// ----------------------------------------------------------------------------
//! \brief Set PID parameters
void LiftDevice::InitPID(uint32_t _nKp, uint32_t _nKd, uint32_t _nKi)
{
	m_PIDController.Init(_nKp, _nKd, _nKi);
#ifdef DBGPRINTF_LIFT
	dbgprintf("Lift device (%s) init PID controller\n",GetDeviceName());
	dbgprintf("   P = %d, I = %d, D = %d\n",_nKp,_nKd,_nKi);
#endif
}

// ----------------------------------------------------------------------------
//! \brief Init rest and working positions
void LiftDevice::InitPositions(int32_t _nRestPosition, int32_t _nWorkingPosition)
{ 
	m_nRestPosition = _nRestPosition;
	m_nWorkingPosition = _nWorkingPosition;
#ifdef DBGPRINTF_LIFT
	dbgprintf("Lift device (%s) init positions\n",GetDeviceName());
	dbgprintf("   Rest Position = %d, Working Position = %d\n",_nRestPosition,
		_nWorkingPosition);
#endif
}

// ----------------------------------------------------------------------------
//! \brief Initialize homing
void LiftDevice::InitHoming(EMotorDriverMode _eHomingDirection, int32_t _nHomePosition, 
									 uint32_t _nCurrentLimit, uint32_t _nMaxDuration, 
									 uint32_t _nStartCurrentLimit, uint32_t _nStartDuration)
{
	m_eHomingDirection = _eHomingDirection;
	m_nHomePosition = _nHomePosition;
	m_nHomingCurrentLimit = _nCurrentLimit;
//	m_nAdjCurrentLimit = _nCurrentLimit;
	m_nHomingMaxDuration = _nMaxDuration;
	m_nHomingStartCurrentLimit = (_nCurrentLimit < _nStartCurrentLimit) ? _nStartCurrentLimit : _nCurrentLimit;
	m_nHomingStartDuration = _nStartDuration;
#ifdef DBGPRINTF_LIFT
	dbgprintf("Lift device (%s) init homing\n",GetDeviceName());
	dbgprintf("   Home Position = %d\n",_nHomePosition);
	dbgprintf("   Home Direction = %d\n",_eHomingDirection);
	dbgprintf("   Homing Start Current Limit = %dmA\n",m_nHomingStartCurrentLimit);
	dbgprintf("   Homing Stop Current Limit = %dmA\n",m_nHomingCurrentLimit);
	dbgprintf("   Homing Start Time Limit = %dms\n",m_nHomingStartDuration);
	dbgprintf("   Homing Max Time Limit = %dms\n",_nMaxDuration);
#endif
}

// ----------------------------------------------------------------------------
//! \brief Initialize how we adapt position of the lift based on external current
void LiftDevice::InitPositionRegulation(int32_t _nRef, uint32_t _nRefToLift, uint32_t _nRefToLower)
{
	m_nExtCurRef = _nRef;
   m_nExtCurRefToLift = _nRefToLift;
   m_nExtCurRefToLower = _nRefToLower;
#ifdef DBGPRINTF_LIFT
	dbgprintf("Lift device (%s) init position regulation\n",GetDeviceName());
	dbgprintf("   ExtCurRef = %d, ExtCurRefToLift = %d, RefToLower = %d\n",_nRef,
		_nRefToLift,_nRefToLower);
#endif
}

// ----------------------------------------------------------------------------
//! \brief Set maximum current. If this limit is reached, motor is disabled.
void LiftDevice::SetCurrentMax(uint32_t _nCurrent,uint32_t _nAdjCurrentLimit)
{
	m_nCurrentMax = _nCurrent;
	m_nAdjCurrentLimit = _nAdjCurrentLimit;
#ifdef DBGPRINTF_LIFT
	dbgprintf("Lift device (%s) set Max Current = %dmA\n",GetDeviceName(),_nCurrent);
#endif
}

// ----------------------------------------------------------------------------
//! \brief Return true if the current is not too low (> 500 mA)
bool LiftDevice::CheckCurrent()
{
	int32_t nCurrent = m_LiftMotor.GetCurrent();
	return (abs(nCurrent) > 20) && (abs(nCurrent) < m_nCurrentMax);
}

// ----------------------------------------------------------------------------
//! \brief Returns the Lift Device Name
const char * LiftDevice::GetDeviceName(void)
{
	return m_LiftMotor.GetName();
}

// ----------------------------------------------------------------------------
//! \brief Returns the number of counted Hall Sensor Pulses
uint32_t LiftDevice::GetHallCountPulses(void)
{
	return m_cnt_pulse;
}

// ----------------------------------------------------------------------------
//! \brief Returns the Lift Device Status
uint32_t LiftDevice::GetLiftDeviceStatus(void)
{
	return m_eLiftState;
}

// ----------------------------------------------------------------------------
//! \brief Returns the Lift Device Status that caused an error
uint32_t LiftDevice::GetLiftDeviceErrorSrc(void)
{
	return m_eLiftStateErrSrc;
}

// ----------------------------------------------------------------------------
//! \brief Returns the Lift Device Status Error Reason
uint32_t LiftDevice::GetLiftDeviceErrorReason(void)
{
	return m_Lift_Error_Reason;
}

// ----------------------------------------------------------------------------
//! \brief Returns the Cleaning Device Status
uint32_t LiftDevice::GetCleaningDeviceStatus(void)
{
	return m_eStatus;
}

// ----------------------------------------------------------------------------
//! \brief Sets the Lift Device Status
uint32_t LiftDevice::ChangeStatus(uint32_t status)
{
	if (status == ELiftDeviceStatus_Error)
		m_Lift_Error_Reason = eLiftDeviceErrReason_ExtChange;		// Error Reason for the Lift FSM
#if TRACEALYZER != 0 && TRC_LIFT != 0
	vTracePrint(trcLift," DevState -> Set");
#endif
	m_eLiftState = (ELiftDeviceStatus)status;
	return 1;
}

// ----------------------------------------------------------------------------
//! \brief Returns the Lift Device Motor Status
uint32_t LiftDevice::GetLiftDeviceMotorStatus(void)
{
	return m_LiftMotor.GetMotorState();
}

// ----------------------------------------------------------------------------
//! \brief Gets the End Switch Status
uint32_t LiftDevice::GetEndSwitchState(void)
{
	return m_LiftMotor.GetEndSwitchState();
}

// ----------------------------------------------------------------------------
//! \brief Gets the Overcurrent that caused the device to fail
uint32_t LiftDevice::GetOverCurrent_Test(void)
{
	return m_OverCurrent_Test;
}

// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
//! \brief Reets the End Switch Status
void LiftDevice::ResetEndSwitchState(void)
{
	m_LiftMotor.ResetEndSwitchState();
}

// ----------------------------------------------------------------------------
//! \brief Disables Device Adjust
void LiftDevice::DisableDeviceAdjust(bool disable)
{
	m_bEnaAdjust = !disable;
}

// ----------------------------------------------------------------------------
//! \brief Sets the Maximum Lift Current
void LiftDevice::SetLiftMaxCurrent(uint32_t current)
{
	m_nCurrentMax = current;
}

// ----------------------------------------------------------------------------
//! \brief Disables Device Adjust
bool LiftDevice::GetStateAndPos(int *act_pos,int *home_pos,int *rest_pos,
	int *delta_pos,int *work_pos,int *in_range,uint8_t *is_up,uint8_t *state)
{
	*act_pos = m_nActualPosition;
	*home_pos = m_nHomePosition;
	*rest_pos = m_nRestPosition;
	*delta_pos = m_nDeltaPosition;
	*work_pos = m_nWorkingPosition;
	*in_range = m_nLiftMotorInPosRange;
	*is_up = IsUp() ? 1 : 0;	
	*state = m_bEnaAdjust ? 1 : 0;
	return true;
}

//! \brief Injects an Error in the Lift Device FSM
bool LiftDevice::InjectError(uint16_t Error)
{
	bLiftTimeout = true;
	m_eStatus = ECleaningDeviceStatus_Error;					// set the Cleaning Device State to "Error"
	return true;
}

#ifdef USE_DWELLTIME
// ----------------------------------------------------------------------------
//! \brief Wait for the Dwell Time to expire
bool LiftDevice::WaitDwellTime(void)
{
	if (DwellTime == -1 || Now() - DwellTime > 30000)
		return true;
	else
		return false;
}
#endif

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------

extern "C" int GetHallCountPulses(int nDevice)
{
	if (nDevice < sLiftDevicesN)
		return sLiftDevices[nDevice]->GetHallCountPulses();
	else
		return 0;
}

extern "C" int GetLiftDeviceStatus(int nDevice)
{
	if (nDevice < sLiftDevicesN)
	{
		int status = sLiftDevices[nDevice]->GetLiftDeviceStatus() & 0xFF;
		status |= (sLiftDevices[nDevice]->GetLiftDeviceErrorSrc() & 0xFF) << 8;
		status |= (sLiftDevices[nDevice]->GetLiftDeviceErrorReason() & 0xFF) << 16;
		status |= (sLiftDevices[nDevice]->GetLiftDeviceMotorStatus()  & 0xFF) << 24;
		return status;
	}
	else
		return 0;
}

extern "C" uint32_t GetLiftDeviceOvercurrentTest(int nDevice)
{
	if (nDevice < sLiftDevicesN)
	{
		return sLiftDevices[nDevice]->GetOverCurrent_Test();
	}
	else
		return 0;
}

extern "C" int GetCleaningDeviceStatus(int nDevice)
{
	if (nDevice < sLiftDevicesN)
		return sLiftDevices[nDevice]->GetCleaningDeviceStatus();
	else
		return 0;
}

extern "C" int ChangeLiftDeviceStatus(int nDevice,uint32_t status)
{
	if (nDevice < sLiftDevicesN)
	{
		if (sLiftDevices[nDevice]->ChangeStatus(status) != 0)
			return 1;
		else
			return 0;
	}
	else
		return 0;
}

extern "C" bool DisableAdjust(int nDevice,uint8_t disable)
{
	if (nDevice < sLiftDevicesN)
	{
		sLiftDevices[nDevice]->DisableDeviceAdjust(disable != 0);
		return true;
	}
	else
		return false;
}

extern "C" bool GetLiftStateAndPos(int nDevice,int *act_pos,
		int *home_pos,int *rest_pos,int *delta_pos,
		int *work_pos,int *in_range,uint8_t *is_up,
		uint8_t *state)
{
	if (nDevice < sLiftDevicesN)
	{
		return sLiftDevices[nDevice]->GetStateAndPos(act_pos,home_pos,rest_pos,
				delta_pos,work_pos,in_range,is_up,state);
	}
	else
		return false;
}

extern "C" bool SetLiftMaxCurrent(int nDevice,uint32_t current)
{
	if (nDevice < sLiftDevicesN)
	{
		sLiftDevices[nDevice]->SetLiftMaxCurrent(current);
		return true;
	}
	else
		return false;
}

// ----------------------------------------------------------------------------
//! \brief Injects a Timeout Error Into the Cleaning Manager
extern "C" bool Lift_InjectError(int nDevice,uint16_t Error)
{
	if (nDevice < sLiftDevicesN)
		return sLiftDevices[nDevice]->InjectError(Error);
	else
		return false;
}
