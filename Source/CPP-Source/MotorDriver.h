// ---------------------------------------------------------------------------
//! \package     CleaningUnit
//! \file        MotorDriver.h
//! \brief       Defines classes responsible for the management of a one of the PWM motors
//! \details     All motors controlled by the cleaning unit board are driven by a VNH3SP30-E chip from STMicroelectronics.
//! \copyright   Copyright (C) 2011-2012 BlueBotics SA
// ---------------------------------------------------------------------------

#ifndef _MOTORDRIVER_H_
#define _MOTORDRIVER_H_

// ----------------------------------------------------------------------------
// Includes
#include "Base.h"
#include "EventSource.h"
#include "PWMDriver.h"

// ----------------------------------------------------------------------------
// Constants
#define NB_CURRENTMONITOR_HISTORYSLOTS 	32
#define NB_CURRENTMONITOR_SIGNED				0			// if != 0 measure current signed

#define CURRENTMONITOR_VDIV_GAIN				333		// Voltage Divider Gain * 1000

// ----------------------------------------------------------------------------
// Forward declaration
class PWMOutput;
class DigitalInput;
class DigitalOutput;
class DigitalInputOutput;
class IEventHandler;
class AnalogInput;

// ----------------------------------------------------------------------------
//! \brief List the different modes of the bridge
typedef enum 
{
	EMotorDriverMode_BrakeVCC = 0,
	EMotorDriverMode_Clockwise,
	EMotorDriverMode_CounterClockwise,
	EMotorDriverMode_BrakeGND,
} EMotorDriverMode;

// ----------------------------------------------------------------------------
//! \brief List the different combinations of active sides of the bridge
typedef enum 
{
	EMotorDriverSide_None = 0x00,
	EMotorDriverSide_A = 	0x01,
	EMotorDriverSide_B = 	0x02,
	EMotorDriverSide_Both = 0x03,
} EMotorDriverSide;

// ----------------------------------------------------------------------------
//! \class      CurrentProbe
//! \brief      Encapsulate the measure of a current 
//! \details    CurrentProbe uses an analog input to read the raw data and convert it into a current based on conversion factors and offset passed during instanciation.
class CurrentProbe
{
public:
	CurrentProbe(AnalogInput *_pCurrentFB,  int32_t _nADCgain = 1201);
	virtual ~CurrentProbe() {}
    //! \cond 
	HideDefaultMethods(CurrentProbe);
    //! \endcond 

public:
	virtual void Reset() {} // Not need in this class (see CurrentAverage)
	virtual void SetOffset(int32_t offset);
	virtual void SetGain(int32_t offset);
	virtual int32_t Measure();
	virtual int32_t GetCurrent() { return m_nCurrent; }	

private:
	AnalogInput *m_pCurrentFB;
	int32_t m_nADCgain; 
	int32_t m_nCurrent;
	int32_t m_nOffset;
};

// ----------------------------------------------------------------------------
//! \class      CurrentAverage
//! \brief      Provide the average of the measure of a current 
//! \details    CurrentAverage basically makes an average of the current measured by its base class CurrentProbe
class CurrentAverage : public CurrentProbe
{
public:
	CurrentAverage(AnalogInput *_pCurrentFB);
	virtual ~CurrentAverage() {}
    //! @cond 
	HideDefaultMethods(CurrentAverage);
    //! @endcond 

public:
	virtual void Reset();
	virtual int32_t Measure();
	virtual int32_t GetCurrent() { return m_nAverage; }

private:
	AnalogInput *m_pCurrentFB;
	int16_t m_anHistory[NB_CURRENTMONITOR_HISTORYSLOTS];
	int32_t m_nTotal;
	int32_t m_nIndex;
	int32_t m_nAverage;	
	bool m_bFull;
	bool m_bReset;

};

// ----------------------------------------------------------------------------
//! \class      MotorDriver
//! \brief      Class responsible for management of one of the PWM motors. 
//! \details    All motors controlled by the cleaning unit board are driven by a VNH3SP30-E chip from STMicroelectronics. 
class MotorDriver: public IEventHandler
{
public:
    MotorDriver(PWMOutput *_pPWM, 
                CurrentProbe *_pCurrentProbe,
                EMotorDriverMode _eEnableMode,
                EMotorDriverMode _eDisableMode,
					 const char *Name = nullptr,
					 DigitalInput *_pEndSWlower = nullptr,
					 DigitalInput *_pEndSWupper = nullptr,
					 eBlockDirMode_t _EndSWlowerBlock = eBlockDirNone,
					 eBlockDirMode_t _EndSWupperBlock = eBlockDirNone)
;
    virtual ~MotorDriver() {}
    //! \cond 
    HideDefaultMethods(MotorDriver);
    //! \endcond 

public:
    virtual void HandleEvent(EventSource *_pSource, uint32_t _nEventId, uint32_t _nData);	
    bool RegisterHandler(IEventHandler *_pHandler, uint32_t _nEventId);
    bool Enable(void);
    bool Disable(void);
    bool SetRatio(uint32_t _nNumerator, uint32_t _nDenominator);
    bool SetRatioAndMode(uint32_t _nNumerator, uint32_t _nDenominator, EMotorDriverMode _eMode);
    int32_t GetCurrent(void);
    uint32_t GetMaxCurrent(void);
    void ResetCurrentMeasure(void);
    void ResetMaxCurrent(void);
    uint32_t GetPeriodDuration(void);
    bool IsStatusOk(void);
    EMotorDriverMode GetMode(void) { return m_eMode; }
	 char * GetName(void) { return m_Name; }
	 bool IsEnabled(void);
	 uint8_t GetEndSwitchState(void);
	 void ResetEndSwitchState(void);
	 uint8_t GetMotorState(void);
	 bool isBlocked(void);

private:
	static const char *strDirection[4];				//!< Direction Strings
	CurrentProbe 		*m_pCurrentProbe;          //!< The probe to measure the current
	PWMOutput 			*m_pPWM;                  	//!< The PWM output that will control the motor
	EMotorDriverMode 	m_eEnableMode;             //!< Indicates what mode should be used when MotorDriver is enabled
	EMotorDriverMode 	m_eDisableMode;            //!< Indicates what mode should be used when MotorDriver is disabled
	DigitalInput 		* m_pEndSWlower;				//!< Lower End Switch IO
	DigitalInput 		* m_pEndSWupper;				//!< Upper End Switch IO
	eBlockDirMode_t	m_EndSWlowerBlock;			//!< Lower End Switch Blocking Direction
	eBlockDirMode_t	m_EndSWupperBlock;			//!< Lower End Switch Blocking Direction
	EMotorDriverMode 	m_eMode;                   //!< Remember the current mode
   uint32_t				m_nMaxCurrent;             //!< The maximum measured value of the current
	char 					m_Name[20];						//!< The name of the device
	bool					m_IsEnabled;					//!< Shows the Mode of the Motor
	uint8_t				m_EndSwitchState;				//!< Set if a Endswitch has been activated
#if (TRACEALYZER != 0) && (TRC_MOTOR != 0)
	EMotorDriverMode	m_s_eMode = EMotorDriverMode_BrakeGND;
	uint32_t m_s_Num = 0,m_s_Denom = 0;
#endif
};

#endif // _MOTORDRIVER_H_
