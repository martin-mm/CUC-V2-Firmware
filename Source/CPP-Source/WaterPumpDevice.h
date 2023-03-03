// ---------------------------------------------------------------------------
//! \package     CleaningUnit
//! \file        WaterPumpDevice.h
//! \brief       Defines the class responsible for management of the water pump device
//! 
//! \copyright   Copyright (C) 2011-2012 BlueBotics SA
// ----------------------------------------------------------------------------

#ifndef _WATERPUMPDEVICE_H_
#define _WATERPUMPDEVICE_H_

// ----------------------------------------------------------------------------
// Includes
#include "Base.h"
#include "CleaningDevice.h"
#include "EventSource.h"
#include "MotionGenerator.h"

#define NB_FLOW_MEASURES 10
#define MIN_PULSES_FOR_DETECTION 10

#define TEST_TANK_EMPTY		1

// ----------------------------------------------------------------------------
// Forward declaration
class DigitalInput;
class MotorDriver;
class AnalogInput;
class DigitalOutput;
class CaptureInput;

// ----------------------------------------------------------------------------
//! \brief List the different states used to generate the pumps PWM signal
typedef enum {
	WaterPumpSignalState_Activation_A = 0,
	WaterPumpSignalState_Pulse_A,
	WaterPumpSignalState_Deactivation_A,
	WaterPumpSignalState_Activation_B,
	WaterPumpSignalState_Pulse_B,
	WaterPumpSignalState_Deactivation_B,
	WaterPumpSignalState_Wait,
} WaterPumpSignalState;

// ----------------------------------------------------------------------------
//! \class      WaterPumpDevice
//! \brief      Control both water pumps with a single PWM
class WaterPumpDevice : public CleaningDevice, public IEventHandler
{
public:
	WaterPumpDevice(MotorDriver &_pump1, MotorDriver &_pump2, DigitalInput *_pFlowMeterCapture,
		int ID);
	virtual ~WaterPumpDevice(void) {}
    //! \cond 
	HideDefaultMethods(WaterPumpDevice);
    //! \endcond 

public:
	virtual void HandleEvent(EventSource *_pSource, uint32_t _nEventId, uint32_t _nData);
	virtual bool Enable(void);
	virtual bool Start(void);
	virtual bool Stop(void);
	virtual bool Disable(void);
	void SetWaterDensity(uint32_t _nDensity); 		// microliter/m
   void SetLinearSpeed(uint8_t _nAbsSd);     		// cm/s
   virtual uint32_t GetMaxCurrent(void);
   virtual void ResetMaxCurrent(void);
	virtual const char * GetDeviceName(void);
   bool IsTankEmpty(void) { return m_bTankIsEmpty; }
   uint32_t GetAbsFlowPulses(void) { return m_nAbsFlowPulses; };

private:
	static const char *Name;
	bool CheckCurrent(void);
   void CheckWaterFlow(uint32_t _nTargetFlow);

private:
	MotorDriver &m_Pump1;                   	//!< The "Motor" (PWM channel) used to drive pump 1
	MotorDriver &m_Pump2;                    	//!< The "Motor" (PWM channel) used to drive pump 2
	DigitalInput *m_pFlowMeterCapture;      	//!< The capture input used to measure the flow of water
	PIDController m_PIDController;          	//!< The PID controller used to regulate the flow of water
	WaterPumpSignalState m_eState;          	//!< The internal state used to generate the output signal
	uint32_t m_nCounter;	                    	//!< Counter used to measure time while generating the output signal
	uint32_t m_nFlowPulses;                   //!< [pulses] Counter used to measure the water flow (count events from the hall sensor)
	uint64_t m_nNextMeasureTime;              //!< Time indicating when to update the water flow measure
	uint32_t m_nWaterDensity;                 //!< [microliter / meter] The nominal water density
   uint32_t m_nAbsSd;                        //!< [cm/s] The linear speed of the vehicle
   bool m_bTankIsEmpty;                    	//!< Becomes true when the tank is empty
	int m_ID;											//!< ID of the Waterpump
	ECleaningDeviceStatus m_eStatus_old;		//!< Old Cleaning Device Status
#if TRACEALYZER != 0 && TRC_WATER != 0
	uint32_t m_nTotalDuration_old;				//!< Old Total Duration
	ECleaningDeviceStatus m_OldCleaningState;
	WaterPumpSignalState	m_SignalStateOld;
#endif

   uint8_t m_nMeasureIndex;
   uint32_t m_nTotalActual;
   uint32_t m_nTotalTargeted;
   uint32_t m_anActualFlows[NB_FLOW_MEASURES];
   uint32_t m_anTargetFlows[NB_FLOW_MEASURES];
   uint32_t m_nAbsFlowPulses;
};

#endif // _WATERPUMPDEVICE_H_
