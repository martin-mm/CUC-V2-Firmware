// ---------------------------------------------------------------------------
//! \package     ARMLibrary
//! \file        MotionGenerator.cpp
//! \brief       Defines a set of motion related classes 
//! 
//! \copyright   Copyright (C) 2011-2012 BlueBotics SA
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Includes
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "MotionGenerator.h"
#include "board.h"

#define RAMP_GENERATOR_DEBUG
#undef RAMP_GENERATOR_DEBUG
#define TMP_GENERATOR_DEBUG
#undef TMP_GENERATOR_DEBUG

#if (TRACEALYZER != 0) && (TRC_MOTION != 0)
static traceString 				trcRamp = nullptr;
static traceString 				trcPID = nullptr;
#endif

// ----------------------------------------------------------------------------
//! \brief Helper function to compute time differences
static uint32_t ComputeDeltaTime(uint32_t t1, uint32_t t2)
{	
	uint32_t dt = t2 - t1;	
	if (dt > 0x80000000) dt = dt - 0x80000000;
	return dt;	
}

// ----------------------------------------------------------------------------
//! \brief Constructor
RampGenerator::RampGenerator(uint32_t _nSlope,uint32_t _nSlopeDiv)
	: m_nTime(0),
	  m_nCurrent(0),
	  m_nTarget(0),
	  m_nSlope(_nSlope),
	  m_nSlopeDiv(_nSlopeDiv)
{	
	dbgprintf("Ramp Generator Constructor, Slope = %d, Slope_Div = %d ...\n",
			_nSlope,_nSlopeDiv);	
#if TRACEALYZER != 0 && TRC_MOTION != 0
	if (trcRamp == nullptr)
		trcRamp = xTraceRegisterString("RAMP");
#endif
	dbgprintf("... Ramp Generator Constructor done.\n");	
}

// ----------------------------------------------------------------------------
//! \brief Reset the ramp generator
void RampGenerator::Reset(void)
{
	m_nTarget = 0;
	m_nTime = 0;
}

// ----------------------------------------------------------------------------
//! \brief Set the target value for the ramp generator
void RampGenerator::SetTarget(int32_t _nTarget)
{
	m_nTarget = _nTarget;
}

// ----------------------------------------------------------------------------
//! \brief Set the slope of the ramp
void RampGenerator::SetSlope(uint32_t _nSlope,uint32_t _nSlopeDiv)
{
	m_nSlope = _nSlope;
	m_nSlopeDiv = _nSlopeDiv;
}

// ----------------------------------------------------------------------------
//! \brief Get the actual position from the Ramp Generator
int32_t RampGenerator::GetValue()
{
	return m_nCurrent;
}

// ----------------------------------------------------------------------------
//! \brief Checks if the ramp is in progress
bool RampGenerator::IsMoving()
{
	return (m_nTarget != m_nCurrent);
}

// ----------------------------------------------------------------------------
//! \brief Execute one cycle of the ramp
void RampGenerator::Execute(uint32_t _nTime)
{	

	// Compute delta time
	int32_t dt = ComputeDeltaTime(m_nTime, _nTime);
	m_nTime = _nTime;

	// Compute step 
	int32_t nDelta = m_nTarget - m_nCurrent;										// Compute delta
	int32_t nSign = ((nDelta < 0) ? -1 : 1);										// Compute direction (sign of delta)
	int32_t nStep = (int32_t)m_nSlope * nSign * dt / m_nSlopeDiv;			// Compute step

	if (abs(nStep) > abs(nDelta)) 
   {
      // Handle reaching the target
		m_nCurrent = m_nTarget;
   }
   else 
   {
		// Compute new current value
      m_nCurrent += nStep;
   }
}

// ----------------------------------------------------------------------------
//! \brief Constructor
TMPGenerator::TMPGenerator(uint32_t _nTravelSpeed, uint32_t _nAcceleration,int ID)
{
	dbgprintf("Trapezoid Motion Generator Constructor, TravelSpeed = %d, Acceleration = %d ...\n",_nTravelSpeed,_nAcceleration);	
	m_nCurPos = 0;
	m_nCurSpeed = 0;
	m_nTime = 0;
	m_eState = ETMPState_Idle;
	m_ID = ID;
	SetTravelSpeed(_nTravelSpeed);
	SetAcceleration(_nAcceleration);
#if TRACEALYZER != 0 && TRC_TMP != 0 
	sprintf(m_char,"TMP %d",m_ID);
	trcTMP = xTraceRegisterString(m_char);
	sprintf(m_char_s,"TMP_s %d",m_ID);
	trcTMP_s = xTraceRegisterString(m_char_s);
	sprintf(m_char_v,"TMP_v %d",m_ID);
	trcTMP_v = xTraceRegisterString(m_char_v);
	sprintf(m_char_a,"TMP_a %d",m_ID);
	trcTMP_a = xTraceRegisterString(m_char_a);
#endif
	dbgprintf("... Trapezoid Motion Generator Constructor done.\n");	
}

// ----------------------------------------------------------------------------
//! \brief Execute one cycle of the TMP generator
void TMPGenerator::Execute(uint32_t _nTime)
{
	// Compute delta time
	// Time Difference to the Start of the Movement
	uint32_t dtStart = ComputeDeltaTime(m_nStartTime, _nTime);	
	// Time Difference to the last Time the TMP Generator did execute
	uint32_t dt = ComputeDeltaTime(m_nTime, _nTime);	

	// Save current time and speed
	m_nTime = _nTime;
	int32_t nLastSpeed = m_nCurSpeed;	
	// Set the Target Reached Flag to false;
	bool bTargetReached = false;

	// Handle the State Machine
	switch (m_eState)
	{
		case ETMPState_Idle:							// Idle phase			
			m_nCurSpeed = 0;
			break;

		case ETMPState_Acc: 							// Acceleration phase			
			m_nCurSpeed += m_nCurAcc;					
			if (dtStart >= m_tAcc)					// Check end of acceleration
			{						
				m_eState = ETMPState_Dec;		
#if TRACEALYZER != 0 && TRC_TMP != 0
				vTracePrintF(trcTMP,"TMPGEN: End of Accel., Speed = %d",m_nCurSpeed);
#endif
				if (m_tCst != 0)						// Do we have a constant speed phase?
				{
					m_eState = ETMPState_Cst;
					m_nCurSpeed = m_nCurTravelSpeed;
#if TRACEALYZER != 0 && TRC_TMP != 0
					vTracePrintF(trcTMP,"TMPGEN: -> Const. Speed, Speed = %d",m_nCurSpeed);
#endif
				}
				else
				{
#if TRACEALYZER != 0 && TRC_TMP != 0
					vTracePrintF(trcTMP,"TMPGEN: -> Deaccel., Speed = %d",m_nCurSpeed);
#endif
				}
			}
			else
			{
#if TRACEALYZER != 0 && TRC_TMP != 0
				vTracePrintF(trcTMP,"TMPGEN: Accel., Speed = %d",m_nCurSpeed);
#endif
			}
			break;
		case ETMPState_Cst:							// Constant speed phase			
			m_nCurSpeed = m_nCurTravelSpeed;			
			if (dtStart >= (m_tAcc + m_tCst))	// Check end of acceleration
			{
#if TRACEALYZER != 0 && TRC_TMP != 0
				vTracePrintF(trcTMP,"TMPGEN: -> Deaccel., Speed = %d",m_nCurSpeed);
#endif
				m_eState = ETMPState_Dec;
			}	
			else
			{
#if TRACEALYZER != 0 && TRC_TMP != 0
				vTracePrintF(trcTMP,"TMPGEN: Const., Speed = %d",m_nCurSpeed);
#endif
			}				
			break;
		case ETMPState_Dec:							// Deceleration phase			
			m_nCurSpeed -= m_nCurAcc;
			bTargetReached = (dtStart >= (m_tAcc + m_tCst + m_tDec));
			if (bTargetReached)
			{		
#if TRACEALYZER != 0 && TRC_TMP != 0
				vTracePrintF(trcTMP,"TMPGEN: -> Idle, Speed = %d",m_nCurSpeed);
#endif
				m_eState = ETMPState_Idle;				
				m_nCurSpeed = 0;				
				m_nCurPos = m_nTargetPos;
			}
			else
			{
#if TRACEALYZER != 0 && TRC_TMP != 0
				vTracePrintF(trcTMP,"TMPGEN: Deaccel., Speed = %d",m_nCurSpeed);
#endif
			}
			break;
		default:
			break;
	}	

	// Set the current position if the target has not been reached and
	// the current delta time (time from the las execution of the
	// TMP Gen) is greater than zero
	if (!bTargetReached && dt > 0)
	{
		m_nCurPos += ((m_nCurSpeed + nLastSpeed) * (int32_t)dt) / 2;
	}
#if TRACEALYZER != 0 && TRC_TMP != 0
				vTracePrintF(trcTMP_s,"%d",m_nCurPos);
				vTracePrintF(trcTMP_v,"%d",m_nCurSpeed);
				vTracePrintF(trcTMP_a,"%d",m_nCurAcc);
#endif
}

// ----------------------------------------------------------------------------
//! \brief Change the travel speed of the generator
void TMPGenerator::SetTravelSpeed(uint32_t _nTravelSpeed)
{ 
	m_nTravelSpeed = _nTravelSpeed; 
#if TRACEALYZER != 0 && TRC_TMP != 0
	vTracePrintF(trcTMP,"TMPGEN: Change Trav. Speed: %d",m_nTravelSpeed);
#endif
}

// ----------------------------------------------------------------------------
//! \brief Change the accelerations of the generator
void TMPGenerator::SetAcceleration(uint32_t _nAcceleration)
{ 
	m_nAcceleration = _nAcceleration; 
}

// ----------------------------------------------------------------------------
//! \brief Request the generator to move to a specified position
void TMPGenerator::MoveTo(int32_t _nPosition)
{
	// Compute delta position and speed	
	int32_t deltaPos = _nPosition - m_nCurPos;								// Delta Position
	int32_t nDirection = ((_nPosition > m_nCurPos) ? 1 : -1);			// Direction of the Movement
	int32_t deltaSpeed = (nDirection * m_nTravelSpeed) - m_nCurSpeed;	// Delta Speed
	int32_t speedDirection = ((deltaSpeed > 0) ? 1 : -1);					// Speed Direction
	if (deltaPos == 0)
		return;
	
#if TRACEALYZER != 0 && TRC_TMP != 0
	vTracePrint(trcTMP,"TMPGEN: Move to Target");
#endif
	// Compute time and distance of both acceleration and deceleration phases of the trapezoidal curve	
	m_tAcc = abs(deltaSpeed) / ((int32_t)m_nAcceleration);				// Acceleration Duration
	int32_t dAcc = m_tAcc * (m_nCurSpeed + deltaSpeed / 2);				// Acceleration Distance
	m_tDec = m_nTravelSpeed / m_nAcceleration;								// Deacceleration Duration
	int32_t dDec = m_tDec * nDirection * (int32_t)m_nTravelSpeed / 2;	// Deacceleration Distance

	// Compute lenght of the constant speed phase
	int32_t dCst = nDirection * (deltaPos - (dAcc + dDec));				// Constant Speed Distance

	// Is the distance large enough to reach target speed?
	if (dCst > (int32_t)m_nTravelSpeed)
	{		
		// Yes, generate the trapezoidal curve
		m_tCst = dCst / m_nTravelSpeed;											// Constant Speed Duration
		m_nCurTravelSpeed = nDirection * m_nTravelSpeed;					// Current Travel Speed equals the desired Travel Speed
		m_nCurAcc = speedDirection * m_nAcceleration;						// Calculate the Current Acceleration from the desired Acceleration
		m_nStartTime = m_nTime;														// Set the Starting Time	
		m_nTargetPos = _nPosition;													// Set the Target Position 
		m_eState = ETMPState_Acc;													// Set the FSM to "Acceleration"
#if TRACEALYZER != 0 && TRC_TMP != 0
		vTracePrint(trcTMP,"TMPGEN: Can reach Target Speed");
#endif
		return;
	}

	// Not enough time to reach travel speed...
	// Are we moving at a slower speed than travel speed?
	if (abs(m_nCurSpeed) < m_nTravelSpeed)
	{
		// Yes, then generate a triangular curve
		m_tAcc = sqrt((double)abs(deltaPos) / m_nAcceleration);				// Calculate the Acceleration Time
		m_tCst = 0;																			// no Constant Speed Phase
		m_tDec = m_tAcc;																	// Acceleration Time equals the Deacceleration Time
		m_nCurAcc = speedDirection * abs(deltaPos) / (m_tAcc * m_tAcc);	// Calculate the Current Acceleration
		m_nStartTime = m_nTime;															// Set the Starting Time	
		m_nTargetPos = _nPosition;														// Set the Target Position 
		m_eState = ETMPState_Acc;														// Set the FSM to "Acceleration"
#if TRACEALYZER != 0 && TRC_TMP != 0
		vTracePrint(trcTMP,"TMPGEN: Cannot reach vTargrt, accel.");
#endif
		return;
	}

	// We are already running faster than travel speed: Now just break!
#if TRACEALYZER != 0 && TRC_TMP != 0
	vTracePrint(trcTMP,"TMPGEN: Brake");
#endif
	m_tAcc = 0;
	m_tCst = 0;
	m_tDec = sqrt((double)abs(deltaPos) / m_nAcceleration);
	m_nCurAcc = speedDirection * abs(deltaPos)/(m_tDec * m_tDec);
	m_nStartTime = m_nTime;
	m_nTargetPos = _nPosition;
	m_eState = ETMPState_Dec;
}

// ----------------------------------------------------------------------------
//! \brief Force the position of the generator (typically used at the end of the homing sequence
bool TMPGenerator::SetPosition(int32_t _nPosition)
{
	if (ETMPState_Idle == m_eState)
	{
#if TRACEALYZER != 0 && TRC_TMP != 0
		vTracePrintF(trcTMP,"TMPGEN: Set Position to %d",_nPosition);
#endif
		m_nCurPos = _nPosition;		
		return true;
	}
#if TRACEALYZER != 0 && TRC_TMP != 0
		vTracePrintF(trcTMP,"TMPGEN: FSM not idle, State = %d",(uint32_t)m_eState);
#endif
	return false;
}

// ----------------------------------------------------------------------------
//! \brief Constructor
PIDController::PIDController(int32_t _nKp, int32_t _nKi, int32_t _nKd, int ID,
									  int32_t _nLowSat,int32_t _nHighSat,
									  int32_t _nIntLowSat,int32_t _nIntHighSat) :
									     m_nKp(_nKp),
									     m_nKi(_nKi),
									     m_nKd(_nKd),
										  m_ID(ID),	
									     m_nLowSat(_nLowSat),
									     m_nHighSat(_nHighSat),
									     m_nIntLowSat(_nIntLowSat),
									     m_nIntHighSat(_nIntHighSat),
									     m_nIntegral(0),
									     m_nPreError(0),
										  m_nTime(0)
{
	dbgprintf("PID Controller Constructor, Kp = %d, Ki = %d, Kd = %d ...\n",_nKp,_nKi,_nKd);	
#if TRACEALYZER != 0 && TRC_PID != 0 
	sprintf(m_char_s,"PID_s %d",m_ID);
	trcPID_s = xTraceRegisterString(m_char_s);
	sprintf(m_char_a,"PID_a %d",m_ID);
	trcPID_a = xTraceRegisterString(m_char_a);
	sprintf(m_char_o,"PID_o %d",m_ID);
	trcPID_o = xTraceRegisterString(m_char_o);
#endif
	dbgprintf("... PID Controller Constructor done\n");	
}

// ----------------------------------------------------------------------------
//! \brief Set PID coefficients and reset the integral
void PIDController::Init(int32_t _nKp, int32_t _nKi, int32_t _nKd)
{
	SetKp(_nKp);
	SetKi(_nKi);
	SetKd(_nKd);
	m_nIntegral = 0;
	m_nPreError = 0;
}

// ----------------------------------------------------------------------------
//! \brief Execute one cycle of the PID algorithm
int32_t PIDController::Execute(uint32_t _nTime, int32_t _nTarget, int32_t _nActual)
{
	// Compute delta time (time to the last execution)
	uint32_t dt = ComputeDeltaTime(m_nTime, _nTime);
	m_nTime = _nTime;

	// Caculate the position error
	int32_t nError = _nTarget - _nActual;
	// Integrate the position error to form the integral part of the controller
	int64_t int_h = m_nIntegral + nError * dt;
	// prevent wind_up of the integrator by saturation
	if (int_h > m_nIntHighSat)
		int_h = m_nIntHighSat;		// Upper saturation
	if (int_h < m_nIntLowSat)
		int_h = m_nIntLowSat;		// Lower saturation
	m_nIntegral = (int32_t)int_h;	// Store the integral
	// Now calculate the derivative ot the position
	int32_t nDerivative = (dt > 0) ? (nError - m_nPreError) / dt : 0;
	// Calculate the output of the PID controller using KI, KP and KD
	int64_t nOutput = (int64_t)m_nKp * nError + 
							(int64_t)m_nKi * m_nIntegral + 
							(int64_t)m_nKd * nDerivative;
	// Saturate the output
	if (nOutput > m_nHighSat)
		nOutput = m_nHighSat;		// Upper saturation
	if (nOutput < m_nLowSat)
		nOutput = m_nLowSat;			// Lower saturation
	// Store the actual PID controller error
	m_nPreError = nError;
#if TRACEALYZER != 0 && TRC_PID != 0
		vTracePrintF(trcPID_s,"PID: Target = %d",_nTarget);
		vTracePrintF(trcPID_a,"PID: Actual = %d",_nActual);
		vTracePrintF(trcPID_o,"PID: Output = %d",nOutput);
#endif	
	return (int32_t)nOutput;
}

