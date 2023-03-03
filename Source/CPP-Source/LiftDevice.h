// ---------------------------------------------------------------------------
//! \package     CleaningUnit
//! \file        LiftDevice.h
//! \brief       Defines the class responsible for management of a lift device
//!
//! \copyright   Copyright (C) 2011-2012 BlueBotics SA
// ----------------------------------------------------------------------------

#ifndef _LIFTDEVICE_H_
#define _LIFTDEVICE_H_

#define 	USE_DWELLTIME
#undef	USE_DWELLTIME

#if defined(__cplusplus)

// ----------------------------------------------------------------------------
// Includes
#include "Base.h"
#include "CleaningDevice.h"
#include "MotionGenerator.h"
#include "EventSource.h"
#include "Timer.h"
#include "MotorDriver.h"
#include "CleaningUnit.h"
#include "Average.h"
#include "board.h"
#if USE_IIR_FILTER != 0
#include "IIRFilter.h"
#endif

#define MAX_OC_CNT					100		// Number of Overcurrent Counts (Moving Overcurrent)
#define MAX_MOVINT_TIME				5000		// Maximum Time for Motor Movement (Moving Timeout)
#define MAX_OC_RECOVERY_TIME		5000		// Overcurrent Recovery Time in ms (Moving Overcurrent)

#define N_MOTOR_CYCLES_CONT		5			// Maximum Motor Cycles (End-to-End) without Dwell Time
#define MOTOR_CYCLE_BLOCK_TIME	1000		// Maximum Motor Block Time in ms
#define MOTOR_DWELL_TIME			30000		// Motor Dwell Time in ms

#define LIFT_CHECK_DWELL_TIME

// ----------------------------------------------------------------------------
// Forward declaration
class PWMOutput;
class AnalogInput;
class DigitalInput;
class MotorDriver;

// ----------------------------------------------------------------------------
//! \brief List the different status of LiftDevice
typedef enum
{
	ELiftDeviceStatus_Idle = 0,	
	ELiftDeviceStatus_StartHoming,
	ELiftDeviceStatus_Homing,
	ELiftDeviceStatus_Moving,
	ELiftDeviceStatus_AdjustUp,
	ELiftDeviceStatus_AdjustIdle,
	ELiftDeviceStatus_AdjustDown,
	ELiftDeviceStatus_Error,
	ELiftDeviceStatus_DwellTime
} ELiftDeviceStatus;

typedef enum
{
	eLiftDeviceErrReason_None = 0,
	eLiftDeviceErrReason_OVC,
	eLiftDeviceErrReason_Ch_To_Err,
	eLiftDeviceErrReason_AdjCurL,
	eLiftDeviceErrReason_AdjCurH,
	eLiftDeviceErrReason_ExtChange,
	eLiftDeviceErrReasonMovTimeout,
	eLiftDeviceErrReasonDwellTime
} eLiftDeviceErrorReason_t;

// ----------------------------------------------------------------------------
//! \class      LiftDevice
//! \brief      Control one of the lift motor
//! \details    Two instances are created. One for the brush lift and one for the suction lift
class LiftDevice : public CleaningDevice, public IEventHandler
{
public:
  LiftDevice(ECleaningUnitMgrErrors _eDeviceErrorId, 
             MotorDriver &_liftMotor,
             MotorDriver *_pDeviceMotor,
             DigitalInput *_pHallSensor,
				 int _nLiftID);
  virtual ~LiftDevice() {}
  //! @cond 
  HideDefaultMethods(LiftDevice);
  //! @endcond 

public:
	virtual void HandleEvent(EventSource *_pSource, uint32_t _nEventId,
									 uint32_t _nData);	
	virtual bool Enable();
	virtual bool Start();
	virtual bool Stop();
	virtual bool Disable();
	virtual bool RecoverFromError();
	virtual bool IsInError(void);
	virtual bool IsExecutingCommand();
#ifdef USE_DWELLTIME
	virtual bool IsWaitingForDwellTime(void); 
#endif
	virtual uint32_t GetMaxCurrent();
	virtual void ResetMaxCurrent();
	virtual bool IsUp();
	virtual const char * GetDeviceName(void);

	bool Lift();
	bool Lower();

	void InitPID(uint32_t _nKp, uint32_t _nKd, uint32_t _nKi);
	void InitPositions(int32_t _nRestPosition, int32_t _nWorkingPosition);
	void InitHoming(EMotorDriverMode _eHomingDirection, int32_t _nHomePosition,
						 uint32_t _nCurrentLimit,uint32_t _nMaxDuration,
						 uint32_t _nStartCurrentLimit = 0,
						 uint32_t _nStartDuration = 0);
	void InitPositionRegulation(int32_t _nRef, uint32_t _nRefToLift,
										 uint32_t _nRefToLower);
	void SetCurrentMax(uint32_t _nCurrent,uint32_t _nAdjCurrentLimit);
	ELiftDeviceStatus GetFSMstate(void);
	uint32_t GetHallCountPulses(void);
	uint32_t GetLiftDeviceStatus(void);
	uint32_t GetLiftDeviceErrorSrc(void);
	uint32_t GetLiftDeviceErrorReason(void);
	uint32_t GetCleaningDeviceStatus(void);
	uint32_t ChangeStatus(uint32_t status);
	uint32_t GetEndSwitchState(void);
	void ResetEndSwitchState(void);
	uint32_t GetOverCurrent_Test(void);
	uint32_t GetLiftDeviceMotorStatus(void);
	void DisableDeviceAdjust(bool disable);
	void SetLiftMaxCurrent(uint32_t current);
	bool GetStateAndPos(int *act_pos,int *home_pos,int *rest_pos,
		int *delta_pos,int *work_pos,int *in_range,uint8_t *is_up,
		uint8_t *state);
	bool InjectError(uint16_t Error);

private:
	bool CheckCurrent(void);
	bool TestOvercurrent(void);
	void HandleObstacle(void);
//	void HandleI2tBehavior(bool Reset = false);
	inline void SetErrorStatusBit(unsigned bit,unsigned position)
	{
		if (bit)
			m_ErrorStatus |= 1 << position;
		else
			m_ErrorStatus &= ~(1 << position);
	};
	bool CheckDwellMovementTime(uint32_t time);
	bool StoreDwellMovTime(void);
	bool WaitDwellTime(void);

private:
	MotorDriver &m_LiftMotor;                       //!< The motor controller
	MotorDriver *m_pDeviceMotor;                    //!< The motor controller of the associated device (basically a link between the brush lift and the brush motor. Not used in the suction case)
	TMPGenerator m_TMPGenerator;                    //!< A trapezoidal motion controller 
	PIDController m_PIDController;                  //!< The PID controller 
	EMotorDriverMode m_eHomingDirection;            //!< Indicates the homing direction
	ELiftDeviceStatus m_eLiftState;                	//!< Current internal status of the lift
	ELiftDeviceStatus m_eLiftStateErrSrc;           //!< Internal status of the lift that caused a transition to the error state
	ECleaningDeviceStatus m_eEndOfMoveStatus;       //!< Status to switch to when move is finished
	uint32_t m_nTime;                               //!< Internal image of the current time
	int32_t m_nActualPosition;                      //!< The actual position
	int32_t m_nDeltaPosition;                       //!< Distance to the target position
	int32_t m_nHomePosition;                        //!< The home position (position reached at the end of the homing sequence)
	int32_t m_nRestPosition;                        //!< The rest position (when the vehicle is not cleaning)
	int32_t m_nWorkingPosition;                     //!< The working position (when the vehicle is cleaning)
	uint32_t m_nHomingCurrentLimit;                 //!< Current used to detect the home position
	uint32_t m_nAdjCurrentLimit;							//!< Max. Current for the Adjust States
	uint32_t m_nHomingMaxDuration;                  //!< Max duration for the homing
	uint32_t m_nHomingStartCurrentLimit;            //!< Current used at the start of the homing
	uint32_t m_nHomingStartDuration;                //!< Duration for the start of the homing
	uint64_t m_nWaitDeadline;							  	//!< Used to set the deadline timing in the Lift Device FSM	
	uint32_t m_nCurrentMax;                         //!< Maximum current accepted when running
	uint32_t m_nCurrentMin;                         //!< Minimum current accepted when running (to detect missing load)
	HallSensorInput *m_pHallSensor;                 //!< The capture input used to measure the position of the motor
	int m_OC_Cnt;												//!< Overcurrent Counter
	int m_OC_ErrorCount;										//!< Overcurrent Error Counter
	int m_ErrorCntr;											//!< Internal Error Counter
	int m_AdjErrorCntr;										//!< Lift Motor Adjust Error Counter
	int m_RecoveryTime;										//!< Lift Overcurrent Recovery Time
	bool bLiftTimeout;										//!< Used For Error Injection from GUI
	int32_t DwellMoveTimes[N_MOTOR_CYCLES_CONT];		//!< Holds the last N_MOTOR_CYCLES_CONT times the Lift has moved
	unsigned	DwellMovTimePtr;								//!< Pointer (Index) into the DwellMoveTimes array (next position)	
	int32_t DwellTime;										//!< Dwell Time, no movement before Dwell Time has not expired
#if USE_IIR_FILTER != 0
	IIR<int32_t> * m_nAvgCurrentFB;			  			//!< This is the IIR Filter for the current measurement (Biquad LP)	
#else
	Average<128, int32_t> m_nAvgCurrentFB;			  	//!< This is the Averager for the current measurement (Moving Average)	
#endif
	int32_t m_nExtCurRefToLift;
	int32_t m_nExtCurRefToLower;
	uint32_t m_nExtCurRef;
	uint32_t m_nLiftMotorInPosRange;
	uint64_t m_nDelayPulse;                         //!< Allowed time between 2 consecutive pulse interrupt
	uint16_t m_ErrorStatus;								  	//!< Actual Error Status of the Lift Device	
	uint32_t m_cnt_pulse;									//!< Counts the Hall Sensor Pulses
	uint32_t m_Lift_Error_Reason;							//!< Reason for the Lift Error State
	uint32_t m_OverCurrent_Test;							//!< Overcurrent at which the motor did stop
	uint32_t m_MovingTimer;									//!< This is the Movement Timer
//	uint32_t m_MotorI2t;										//!< Motor I2t value
//	uint32_t m_MotorRunTime;								//!< Motor Run Time in ms
//	uint32_t m_MotorStartTime;								//!< Start of Motor Movement in ms
	uint32_t cnt;												//!< Counter used for testing/debugging
	bool m_bEnaAdjust;										//!< Enables the Device Adjustment
	int m_nLiftID;												//!< ID of the Lift Device
	bool m_bLiftUpDryRun;									//!< Lift Status in Dry Run Mode (Up or Down)
	bool m_ErrorWhileMoving;
#if TRACEALYZER != 0 && TRC_LIFT != 0
	traceString trcLift;										//!< Tracealyzer String Pointer
	char m_char[8];											//!< Tracealyzer String
	ECleaningDeviceStatus m_OldCleaningState;
	ELiftDeviceStatus m_OldLiftState;
#endif
};

extern "C" int GetHallCountPulses(int nDevice);
extern "C" int GetLiftDeviceStatus(int nDevice);
extern "C" int GetCleaningDeviceStatus(int nDevice);
extern "C" int ChangeLiftDeviceStatus(int nDevice,uint32_t status);
extern "C" uint32_t GetLiftDeviceOvercurrentTest(int nDevice);
extern "C" bool DisableAdjust(int nDevice,uint8_t disable);
extern "C" bool GetLiftStateAndPos(int nDevice,int *act_pos,
		int *home_pos,int *rest_pos,int *delta_pos,int *work_pos,
		int *in_range,uint8_t *is_up,uint8_t *state);
extern "C" bool SetLiftMaxCurrent(int nDevice,uint32_t current);
extern "C" bool Lift_InjectError(int nDevice,uint16_t Error);

#else

#include <stdint.h>
#include <stdbool.h>

extern int GetHallCountPulses(int nDevice);
extern int GetLiftDeviceStatus(int nDevice);
extern int GetCleaningDeviceStatus(int nDevice);
extern int ChangeLiftDeviceStatus(int nDevice,uint32_t status);
extern uint32_t GetLiftDeviceOvercurrentTest(int nDevice);
extern bool DisableAdjust(int nDevice,uint8_t disable);
extern bool GetLiftStateAndPos(int nDevice,int *act_pos,
		int *home_pos,int *rest_pos,int *delta_pos,int *work_pos,
		int *in_range,uint8_t *is_up,uint8_t *state);
extern bool SetLiftMaxCurrent(int nDevice,uint32_t current);
extern bool Lift_InjectError(int nDevice,uint16_t Error);

#endif

#endif // _LIFTDEVICE_H_
