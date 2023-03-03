// ---------------------------------------------------------------------------
//! \package     ARMLibrary
//! \file        MotionGenerator.h
//! \brief       Defines a set of motion related classes 
//! 
//! \copyright   Copyright (C) 2011-2012 BlueBotics SA
// ----------------------------------------------------------------------------

#ifndef _RAMPGENERATOR_H_
#define _RAMPGENERATOR_H_

// ----------------------------------------------------------------------------
// Includes
#include <limits>
#include "Base.h"
#include "board.h"

// ----------------------------------------------------------------------------
//! \brief Enumeration of Trapezoidal profile states
typedef enum {
	ETMPState_Idle = 0,
	ETMPState_Acc,
	ETMPState_Cst,
	ETMPState_Dec
} ETMPState;

// ----------------------------------------------------------------------------
//! \class      RampGenerator
//! \brief      Class responsible to generate a ramp from the actual value to a new target value.
class RampGenerator
{
public:
	RampGenerator(uint32_t _nSlope,uint32_t _nSlopeDiv);
    //! \cond 
	virtual ~RampGenerator() {}
	HideDefaultMethods(RampGenerator);
    //! \endcond 

public:
	void Execute(uint32_t _nTime);
	void Reset(void);
	void SetTarget(int32_t _nTarget);
	void SetSlope(uint32_t _nSlope,uint32_t _nSlopeDiv);
	int32_t GetValue();
	bool IsMoving();

private:
	uint32_t m_nTime;
	int32_t m_nCurrent;
	int32_t m_nTarget;
	uint32_t m_nSlope;
	uint32_t m_nSlopeDiv;
};

// ----------------------------------------------------------------------------
//! \class      TMPGenerator
//! \brief      Trapezoidal Motion Profile. 
//! \details    Generate positions based on a trapezoid of speed.
class TMPGenerator
{
public:
	TMPGenerator(uint32_t _nTravelSpeed, uint32_t _nAcceleration,int ID);
    //! \cond 
	virtual ~TMPGenerator() {}
	HideDefaultMethods(TMPGenerator);
    //! \endcond 

public:
	void SetTravelSpeed(uint32_t _nTravelSpeed);
	void SetAcceleration(uint32_t _nAcceleration);
	void MoveTo(int32_t _nPosition);
	void Execute(uint32_t _nTime);
	bool SetPosition(int32_t _nPosition);
	int32_t GetPosition()	{ return m_nCurPos; }
	int32_t GetSpeed()		{ return m_nCurSpeed; }
	ETMPState GetState()		{ return m_eState; }
	bool IsMoving()			{ return (m_eState != ETMPState_Idle); }

private:
	// Current state
	ETMPState m_eState;				// Generator state
	uint32_t m_nTime;					// Current time
	int32_t m_nCurPos;				// Current position
	int32_t m_nCurSpeed;				// Current speed

	// Parameters of the current command
	uint32_t m_nStartTime;			// Start time
	int32_t m_nTargetPos;				// Target position
	int32_t m_tAcc;					// Duration of the acceleration phase
	int32_t m_tDec;					// Duration of the deceleration phase
	int32_t m_tCst;					// Duration of the constant speed phase
	int32_t m_nCurAcc;				// Actual acceleration
	int32_t m_nCurTravelSpeed;		// Actual travel speed
	int m_ID;							// ID of the Motion Generator
#if TRACEALYZER != 0 && TRC_TMP != 0
	traceString trcTMP;
	traceString trcTMP_s;
	traceString trcTMP_v;
	traceString trcTMP_a;
	char m_char[8];											//!< Tracealyzer String
	char m_char_s[8];											//!< Tracealyzer String
	char m_char_v[8];											//!< Tracealyzer String
	char m_char_a[8];											//!< Tracealyzer String
#endif

	// General Settings
	uint32_t m_nAcceleration;			// Acceleration
	uint32_t m_nTravelSpeed;			// Travel speed
};

// ----------------------------------------------------------------------------
//! \class      PIDController
//! \brief      Simple PID implementation
class PIDController
{
public:
	PIDController(int32_t _nKp, int32_t _nKi, int32_t _nKd, int ID,
					  int32_t _nLowSat = std::numeric_limits<int>::min(),
					  int32_t _nHighSat = std::numeric_limits<int>::max(),
					  int32_t _nIntLowSat = std::numeric_limits<int>::min(),
					  int32_t _nIntHighSat = std::numeric_limits<int>::max());
    //! \cond 
	virtual ~PIDController() {}
	HideDefaultMethods(PIDController);
    //! \endcond 

public:
	void Init(int32_t _nKp, int32_t _nKi, int32_t _nKd);	// Set PID coefficients and reset the integral
	void SetKp(int32_t _nKp) { m_nKp = _nKp; }
	void SetKi(int32_t _nKi) { m_nKi = _nKi; }
	void SetKd(int32_t _nKd) { m_nKd = _nKd; }
	void SetOutputSaturation(int32_t _nMin, int32_t _nMax) { m_nLowSat = _nMin; m_nHighSat = _nMax; }
	void SetIntegratorSaturation(int32_t _nMin, int32_t _nMax) { m_nIntLowSat = _nMin; m_nIntHighSat = _nMax; }
	int32_t Execute(uint32_t _nTime, int32_t _nTarget, int32_t _nActual);
	int32_t GetError() { return m_nPreError; }

private:
	int32_t m_nKp;
	int32_t m_nKi;
	int32_t m_nKd;
	int m_ID;							// ID of the PID Controller
	int32_t m_nLowSat;
	int32_t m_nHighSat;
	int32_t m_nIntLowSat;
	int32_t m_nIntHighSat;
	int32_t m_nIntegral;
	int32_t m_nPreError;
	uint32_t m_nTime;
#if TRACEALYZER != 0 && TRC_PID != 0
	traceString trcPID_s;
	traceString trcPID_a;
	traceString trcPID_o;
	char m_char_s[8];											//!< Tracealyzer String
	char m_char_a[8];											//!< Tracealyzer String
	char m_char_o[8];											//!< Tracealyzer String
#endif
};

#endif // _RAMPGENERATOR_H_

