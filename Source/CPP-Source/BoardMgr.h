// ---------------------------------------------------------------------------
//! \package     CleaningUnit
//! \file        BoardMgr.h
//! \brief       Defines the class responsible for the overall management of the cleaning unit board
//!
//! \copyright   Copyright (C) 2011-2012 BlueBotics SA
// ----------------------------------------------------------------------------

#ifndef _BOARDMGR_H_
#define _BOARDMGR_H_

// ----------------------------------------------------------------------------
// Includes
#if defined(__cplusplus)

#include "Base.h"
#include "EventSource.h"
#include "AnalogInput.h"
#include "Watchdog.h"
#include "CleaningUnitMgr.h"
#include "SafetyMgr.h"
#include "PWMDriver.h"
#include "MotorDriver.h"
#include "IO.h"
#include "AnalogInput.h"
#include "CANDriver.h"
#include "CANNode.h"

#define BOARD_N_DATAPROVIDER		2

typedef struct
{
	int						ObjID;
	const char 				Name[20];
} DataProviderInfo_t;

// ----------------------------------------------------------------------------
//! \class      BoardMgr
//! \brief      Main class of the project. Instantiate and manage the different components running on the cleaning unit board. 
class BoardMgr
{
public:
	BoardMgr(void);
	virtual ~BoardMgr(void) {}
   //! \cond 
   HideCopyAssignCompMethods(BoardMgr);
   //! \endcond 

public:
   void UpdateLEDs();
	void HandleWatchdog();
	static void RestartSafetyTask(void);
	static bool GetCANstatus(uint32_t *nProvider,uint32_t *id,
			uint32_t *nmt_state);
	static bool GetCAN_ProviderContent(uint32_t id,uint32_t subid,
			uint32_t *content,bool *valid);
	static bool GetCAN_ProviderInfo(uint32_t id,char **Name,int *ObjID);

private:
	Timer m_Timer0;
	Timer m_Timer1;
	PWMDriver m_PWMDriver;
	AnalogInputMgr m_AnalogInputMgr;
	CurrentProbe m_BrushCurrentMonitor;
	CurrentProbe m_BrushLiftCurrentMonitor;
//	CurrentAverage m_BrushLiftCurrentMonitor;
	CurrentProbe m_SuctionCurrentMonitor;
	CurrentProbe m_SuctionLiftCurrentMonitor;
//	CurrentAverage m_SuctionLiftCurrentMonitor;
	CurrentAverage m_PumpsCurrentMonitor;
	MotorDriver m_BrushMotor;
	MotorDriver m_BrushLiftMotor;
	MotorDriver m_SuctionMotor;
	MotorDriver m_SuctionLiftMotor;
	MotorDriver m_PumpMotor1;
	MotorDriver m_PumpMotor2;
	CleaningUnitMgr m_CleaningUnitMgr;
	SafetyMgr m_SafetyMgr;
	CANDriver m_CANDriver;
	CANNode m_CANMgr;
	DigitalOutput m_RedLed;
	DigitalOutput m_GreenLed;
	Watchdog m_Watchdog;
	static BoardMgr *BoardMgrInstance;
	static const DataProviderInfo_t m_DataProviderInfo[BOARD_N_DATAPROVIDER];
};

extern "C" bool GetCANstatus(uint32_t *nProvider,uint32_t *id,uint32_t *nmt_state);
extern "C" bool GetCAN_ProviderContent(uint32_t id,uint32_t subid,uint32_t *content,bool *valid);
extern "C" bool GetCAN_ProviderInfo(uint32_t id,char **Name,int *ObjID);
extern "C" void RestartSafetyManager(void);

#else

#include <stdint.h>
#include <stdbool.h>

extern bool GetCANstatus(uint32_t *nProvider,uint32_t *id,uint32_t *nmt_state);
extern bool GetCAN_ProviderContent(uint32_t id,uint32_t subid,uint32_t *content,bool *valid);
extern bool GetCAN_ProviderInfo(uint32_t id,char **Name,int *ObjID);
extern void RestartSafetyManager(void);

#endif

#endif // _BOARDMGR_H_
