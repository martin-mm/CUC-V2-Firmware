// ---------------------------------------------------------------------------
//! \package     CleaningUnit
//! \file        SafetyMgr.h
//! \brief       Defines the class responsible to handle all safety related aspects of RoboII
//!
//! \copyright   Copyright (C) 2011-2012 BlueBotics SA
// ----------------------------------------------------------------------------

#ifndef _SAFETYMGR_H_
#define _SAFETYMGR_H_

// ----------------------------------------------------------------------------
// Includes
#if defined(__cplusplus)

#include "cmsis_os2.h"
#include "Base.h"
#include "CANIds.h"
#include "CANNode.h"
#include "ProcessData.h"
#include "IO.h"
#include "AnalogInput.h"
#include "Average.h"
#include "TracealyzerSetup.h"

#define MAX_SAFETY_ERR_COUNTERS	6

typedef enum
{  
    // Error generated during startup check (used with ESafetyMgrStatus_CheckFailed)
	 eSM_Error_NoError				= 0,
    eSM_Error_Check24V           = 1,
    eSM_Error_CheckRecovery      = 2,
    eSM_Error_CheckIRBumper      = 3,
    eSM_Error_CheckARMOk         = 4,
    eSM_Error_CheckANTOk         = 5,
    eSM_Error_CheckEMStop        = 6,
    eSM_Error_Bumper1         	= 7,
    eSM_Error_Bumper2         	= 8,
    eSM_Error_Bumper3         	= 9,
    eSM_Error_Bumper4        		= 10,
    eSM_Error_Floor1             = 11,
    eSM_Error_Floor2             = 12,
    eSM_Error_Floor3             = 13,
    eSM_Error_Floor4             = 14,
    eSM_Error_Simulation         = 15,
    eSM_Error_E24VError          = 16,
    eSM_Error_EMStopError        = 17,
    eSM_Error_ANTError           = 18,
    eSM_Error_BumperTestError    = 19,
    eSM_Error_RecoveryTestError  = 20,
    eSM_Error_RecoveryTimeout    = 21,
    eSM_Error_CANTimeout         = 22,
    eSM_Error_RecoveryDeadline   = 23,
    eSM_Error_SafetyActivated    = 24,
    eSM_Error_UnknownState    	= 25
} eSM_Error;

typedef enum
{
	eCNT_WATCHDOG 						= 0,
	eCNT_24V 							= 1,
	eCNT_ANT_OK 						= 2,
	eCNT_EMC_STOP						= 3,
	eCNT_BMP_TEST 						= 4,
	eCNT_RECOVERY 						= 5
} eERRcnt;

// ----------------------------------------------------------------------------
//! \class      SafetyMgr
//! \brief      Class responsible for all safety related issues
//! \details    
class SafetyMgr : public CUC_Task,
                  public ICANNodeDataProvider
{
public:
	SafetyMgr(osPriority_t _nPriority,
					DigitalInput *_pBumper1In,
					DigitalInput *_pBumper2In,
					DigitalInput *_pBumper3In,
					DigitalInput *_pBumper4In,
					DigitalInput *_pFloor1In,
					DigitalInput *_pFloor2In,
					DigitalInput *_pFloor3In,
					DigitalInput *_pFloor4In,
					DigitalInput *_pTestRecoveryIn,
					DigitalInput *_pTest24In,
					DigitalInput *_pTestBumperIn, 
					DigitalInput *_pTestARMIn,
					DigitalInput *_pTestANTOkIn,
					DigitalInput *_pTestEMStopIn,
					DigitalInput *_pSafetyChkOutANT,
					AnalogInput *_pMain24In,
					DigitalOutput *_pSafetyCheckLogOut,
					DigitalOutput *_pRecoveryOut,
					DigitalOutput *_pARMOkOut,
					DigitalOutput *_pTestMuxEn,
					DigitalOutput *_pTestMuxA0,
					DigitalOutput *_pTestMuxA1,
					DigitalOutput *_pTestMuxA2);
	virtual ~SafetyMgr() {}
    //! @cond 
	HideCopyAssignCompMethods(SafetyMgr);
    //! @endcond 

public:
	virtual void Main();
	virtual void RegisterData(CANNode &_node, uint16_t _nObjIndex);
   bool CheckSafetyChain(uint32_t retry = 0);
   void ClearErrors(void) { m_bClearErrorRequest = true; }
   bool IsOk(void) { return (m_eSafetyState != ESafetyMgrStatus_Error && 
                         m_eSafetyState != ESafetyMgrStatus_CheckFailed); }
	uint32_t getSafetyCheckState(void) { return m_SafetyCheckError; }
	static void EnableChecks(uint32_t CheckFlags);
	static uint32_t GetTaskStatus(void);
	static bool GetStatus(uint32_t *SafetyState,uint32_t *SafetyCheckError,uint32_t *WatchDogErrors,
								 uint32_t *ANTOkErrors, uint32_t *SensorErrors,uint32_t *RecoveryErrors,
								 uint32_t *EnabledTests,uint8_t *TaskIsRunning);
	static bool GetIntStatus(uint32_t *SafetyTaskError,uint32_t *SafetyTaskStatus);
	static bool GetErrCounters(uint8_t *nErrCntr,int size);
	static bool ClearErrCounters(void);
	bool IsEMstopActive(void);
//	bool RecoverFromEMStop(bool Release);

private:
	static SafetyMgr *SafetyMgrInstance;
   uint32_t CheckSensorStates(ESafetyMgrRequests _eRequest);
   uint32_t CheckErrors(uint32_t _nSensors, bool &_bEMCStopActivated);
	Average<32, uint32_t> m_nWatchdogErrorAvg;
	uint32_t 				m_max;
	uint32_t 				m_EMCStopCount;
	uint8_t					m_ErrorCntr[MAX_SAFETY_ERR_COUNTERS];

private:
	uint32_t					m_bDoSafetyCheck;
	ESafetyMgrStatus 		m_eSafetyState;
	eSM_Error				m_SafetyCheckError;
	static const char		*m_SafetyMgrErrorString[];
	uint32_t					m_SafetyTaskError;
	uint32_t					m_SafetyTaskStatus;
	bool						m_bEMCStopActivated;

	// Safety inputs and outputs
	DigitalInput *m_pBumper1In;
	DigitalInput *m_pBumper2In;
	DigitalInput *m_pBumper3In;
	DigitalInput *m_pBumper4In;
	DigitalInput *m_pFloor1In;
	DigitalInput *m_pFloor2In;
	DigitalInput *m_pFloor3In;
	DigitalInput *m_pFloor4In;
	DigitalInput *m_pTestRecoveryIn;
	DigitalInput *m_pTest24SafetyIn;
	DigitalInput *m_pTestBumperIn;
	DigitalInput *m_pTestARMIn;
	DigitalInput *m_pTestANTOkIn;
	DigitalInput *m_pTestEMStopIn;
	DigitalInput *m_pSafetyChkOutANT;
   AnalogInput *m_pMain24In;
	DigitalOutput *m_pSafetyCheckLogOut;    // Goes to the group of AND gates where bumpers and floors signals are connected
	DigitalOutput *m_pRecoveryOut;
	DigitalOutput *m_pARMOkOut;
	DigitalOutput *m_pTestMuxEn;
	DigitalOutput *m_pTestMuxA0;
	DigitalOutput *m_pTestMuxA1;
	DigitalOutput *m_pTestMuxA2;

   bool m_bClearErrorRequest;
   uint32_t m_nWatchdogErrors;
   uint16_t m_nLastWatchdog;
   uint64_t m_nRecoveryDeadline;
   uint64_t m_nBackToOkDeadline;
	uint64_t m_nDelayErrorRecovTimeout;
   uint32_t m_nANTOkErrors;
   uint32_t m_nSensorsErrors;
   uint32_t m_nRecoveryErrors;
	bool m_bDelayErrorRecovery;

	// CAN communication data
	ProcessDataOut<uint32_t> *m_pANTOk;
	ProcessDataOut<uint32_t> *m_pRequest;
	ProcessDataOut<uint32_t> *m_pStatus;
#if TRACEALYZER != 0 && TRC_SAFETY != 0
	uint32_t m_OldSensorState;
#endif
};

extern "C" void SafetyMngrEnableChecks(uint32_t CheckFlags);
extern "C" bool SafetyMngrGetStatus(uint32_t *SafetyState,uint32_t *SafetyCheckError,uint32_t *WatchDogErrors,
												uint32_t *ANTOkErrors, uint32_t *SensorErrors,uint32_t *RecoveryErrors,
												uint32_t *EnabledTests,uint8_t *TaskIsRunning);
extern "C" bool SafetyMngrGetIntStatus(uint32_t *SafetyTaskError,uint32_t *SafetyTaskStatus);
extern "C" bool SafetyMngrGetErrCounters(uint8_t *ErrCntrs,int size);
extern "C" bool SafetyMngrClrErrCounters(void);

#else

#include <stdint.h>
#include <stdbool.h>

extern void SafetyMngrEnableChecks(uint32_t CheckFlags);
extern bool SafetyMngrGetStatus(uint32_t *SafetyState,uint32_t *SafetyCheckError,uint32_t *WatchDogErrors,
										  uint32_t *ANTOkErrors, uint32_t *SensorErrors,uint32_t *RecoveryErrors,
										  uint32_t *EnabledTests,uint8_t *TaskIsRunning);
extern bool SafetyMngrGetIntStatus(uint32_t *SafetyTaskError,uint32_t *SafetyTaskStatus);

extern bool SafetyMngrGetIntStatus(uint32_t *SafetyTaskError,uint32_t *SafetyTaskStatus);
extern bool SafetyMngrGetErrCounters(uint8_t *ErrCntrs,int size);
extern bool SafetyMngrClrErrCounters(void);

#endif

#endif // _SAFETYMGR_H_
