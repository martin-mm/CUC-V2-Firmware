// ---------------------------------------------------------------------------
//! \package     CleaningUnit
//! \file        SafetyMgr.cpp
//! \brief       Defines the class responsible to handle all safety related aspects of RoboII
//!
//! \copyright   Copyright (C) 2011-2012 BlueBotics SA
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Includes
#include "SafetyMgr.h"
#include "CleaningUnit.h"
#include "Timer.h"
#include "board.h"

//#define TEST_WITHOUT_BUMPERS          1
#define NB_MAX_MISSED_WATCHDOG          125
#define NB_MAX_ERRORS                   3
#define NB_MIN_EMCSTOP_ACTIVE           3
#define DELAY_BEFORE_GOING_BACK_TO_OK   2000
#define MAX_TIME_IN_RECOVERY_MODE       6000
#define EMSTOP_RECOVERY_TIMEOUT   		 30000

#define NO_ANT_OK
#define NO_SAFETY_CHECK
// #define TEST_WITHOUT_BUMPERS

#define DBGPRINTF_SAFETY
#undef DBGPRINTF_SAFETY

#define INC_ERR_CNT(index)		if (m_ErrorCntr[index] < 255) m_ErrorCntr[index]++

#if (TRACEALYZER != 0) && (TRC_SAFETY != 0)
static traceString 				trcSaveMgr;
#endif

static const char	osCANTaskName[] = "SAFETY-MGR";

SafetyMgr * SafetyMgr::SafetyMgrInstance = nullptr;

const char * SafetyMgr::m_SafetyMgrErrorString[] = 
{  
	 "NoError",
    "Error_Check24V",
    "Error_CheckRecovery",
    "Error_CheckBumper",
    "Error_CheckARMOk",
    "Error_CheckANTOk",
    "Error_CheckEMStop",
    "Error_Bumper1",
    "Error_Bumper2",
    "Error_Bumper3",
    "Error_Bumper4",
    "Error_Floor1",
    "Error_Floor2",
    "Error_Floor3",
    "Error_Floor4",
    "Error_Simulation",
    "Error_E24VError",
    "Error_EMStopError",
    "Error_ANTError",
    "Error_BumperTestError",
    "Error_RecoveryTestError",
    "Error_RecoveryTimeout",
    "Error_CANTimeout"
};

// ----------------------------------------------------------------------------
//! \brief Constructor
SafetyMgr::SafetyMgr(osPriority_t _nPriority,
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
							DigitalOutput *_pTestMuxA2
							) :
									CUC_Task(_nPriority,osCANTaskName),
									m_pBumper1In(_pBumper1In),
									m_pBumper2In(_pBumper2In),
									m_pBumper3In(_pBumper3In),
									m_pBumper4In(_pBumper4In),
									m_pFloor1In(_pFloor1In),
									m_pFloor2In(_pFloor2In),
									m_pFloor3In(_pFloor3In),
									m_pFloor4In(_pFloor4In),
									m_pTestRecoveryIn(_pTestRecoveryIn),
									m_pTest24SafetyIn(_pTest24In),
									m_pTestBumperIn(_pTestBumperIn),
									m_pTestARMIn(_pTestARMIn),
									m_pTestANTOkIn(_pTestANTOkIn),
									m_pTestEMStopIn(_pTestEMStopIn),
									m_pSafetyChkOutANT(_pSafetyChkOutANT),
									m_pMain24In(_pMain24In),
									m_pSafetyCheckLogOut(_pSafetyCheckLogOut),
									m_pRecoveryOut(_pRecoveryOut),
									m_pARMOkOut(_pARMOkOut),
									m_pTestMuxEn(_pTestMuxEn),
									m_pTestMuxA0(_pTestMuxA0),
									m_pTestMuxA1(_pTestMuxA1),
									m_pTestMuxA2(_pTestMuxA2),
									m_pANTOk(NULL),
									m_pRequest(NULL),
									m_pStatus(NULL)
							
{
   dbgprintf("Safety Manager Constructor ...\n");	
	m_SafetyCheckError = eSM_Error_NoError;
	m_bDoSafetyCheck = 0; /* 0x00FFFFFF; */
	SafetyMgrInstance = this;
	m_SafetyTaskError = 0;
	m_SafetyTaskStatus = 0;
	m_eSafetyState = ESafetyMgrStatus_Ok;
	m_max = 0;
	m_EMCStopCount = 0;
	m_bDelayErrorRecovery = false;
	m_bEMCStopActivated = false;
    
#if TRACEALYZER != 0 && TRC_SAFETY != 0
	trcSaveMgr = xTraceRegisterString("SAFETY MGR");
	m_OldSensorState = 0;
#endif
   dbgprintf("... Safety Manager Constructor done, Safety Check Flags = %08X.\n",m_bDoSafetyCheck);	
}

// ----------------------------------------------------------------------------
//! \brief Entry point of the safety task
void SafetyMgr::Main()
{	
	dbgprintf("Safety Manager, entering Main Loop.\n");	
#if TRACEALYZER != 0 && TRC_SAFETY != 0
	ESafetyMgrStatus		oldState = ESafetyMgrStatus(-1);
	ESafetyMgrRequests	OldRequest = (ESafetyMgrRequests)-1; 
	vTracePrintF(trcSaveMgr,"SM Entering Main Loop, FSM State: %d",m_eSafetyState);
#endif
	while (1)
   {		 
		Wait(10);		// Task Test
		 
      // Get request data
      ESafetyMgrRequests eRequest;
      SplitSafetyRequest(m_pRequest->Read(), eRequest);
      m_pRequest->Write(0);
#if TRACEALYZER != 0 && TRC_SAFETY != 0
		if (eRequest != OldRequest)
		{
			vTracePrintF(trcSaveMgr,"SM Request: %X (TASK)",(int)eRequest);
			OldRequest = eRequest;
		}
#endif
        
      // Get current Sensor States and Errors
      bool bEMCStopActivated;
      uint32_t nSensors = CheckSensorStates(eRequest);
      uint32_t nErrors = CheckErrors(nSensors, bEMCStopActivated);
		m_bEMCStopActivated = bEMCStopActivated;
     
      // Handle the clear error request coming from the cleaning unit manager or the IOBoard
      bool bClearErrorRequest = (m_bClearErrorRequest || ((eRequest & ESafetyMgrRequests_ClearErrors) != 0));
      m_bClearErrorRequest = false;
 #if TRACEALYZER != 0 && TRC_SAFETY != 0
		if (oldState != m_eSafetyState)
		{
			vTracePrintF(trcSaveMgr,"SM State: FSM [%d -> %d]",oldState,m_eSafetyState);
			oldState = m_eSafetyState;
		}
#endif
       
      // Safety State handling state machine
      switch (m_eSafetyState)
      {
			case ESafetyMgrStatus_Ok:
         {
				m_SafetyCheckError = eSM_Error_NoError;
				// Check request for entering the recovery mode
            if ((eRequest & ESafetyMgrRequests_EnterRecovery) != 0)
            {
					m_eSafetyState = ESafetyMgrStatus_Recovering;
#if TRACEALYZER != 0 && TRC_SAFETY != 0
					vTracePrintF(trcSaveMgr,"SM State: OK -> Recovering (FSM, %d)",m_eSafetyState);
#endif
               m_nRecoveryDeadline = SystemTime::GetTime() + MAX_TIME_IN_RECOVERY_MODE;
				}
            // Check EMC stop and errors
            if (bEMCStopActivated)
				{
					m_eSafetyState = ESafetyMgrStatus_EmergencyStop;
#if TRACEALYZER != 0 && TRC_SAFETY != 0
					vTracePrintF(trcSaveMgr,"SM State: OK -> EMstop (FSM, %d)",m_eSafetyState);
#endif
				}
            if (nErrors != 0)
				{
					m_eSafetyState = ESafetyMgrStatus_SafetyActivated;
#if TRACEALYZER != 0 && TRC_SAFETY != 0
					vTracePrintF(trcSaveMgr,"SM State: OK -> Safety Activated (FSM, %d)",m_eSafetyState);
#endif
				}
            break;
			}
         case ESafetyMgrStatus_CheckFailed:
         {
				// Initialization failed at startup...
            // There is nothing we can do in this case.
#if TRACEALYZER != 0 && TRC_SAFETY != 0
				vTracePrintF(trcSaveMgr,"SM State: CheckFailed -> Init. Check failed (FSM, %d)",m_eSafetyState);
#endif
            break;
         }
         case ESafetyMgrStatus_Recovering:
         {
				// Check timeout
            if (m_nRecoveryDeadline < SystemTime::GetTime())
            {
					nErrors = ESafetyMgrErrors_RecoveryTimeout;
					m_eSafetyState = ESafetyMgrStatus_Error;
					m_SafetyCheckError = eSM_Error_RecoveryDeadline;
#if TRACEALYZER != 0 && TRC_SAFETY != 0
					vTracePrintF(trcSaveMgr,"SM State: Recovering -> Error (FSM, %d)",m_eSafetyState);
					vTracePrintF(trcSaveMgr,"CheckError -> Recovery Deadline (FSM, %d)",m_SafetyCheckError);
#endif
            }
                
				// Check if we are asked to leave the recovery mode
				if (bClearErrorRequest || ((eRequest & ESafetyMgrRequests_LeaveRecovery) != 0))
				{
					m_eSafetyState = ESafetyMgrStatus_Ok;
					m_SafetyCheckError = eSM_Error_NoError;
#if TRACEALYZER != 0 && TRC_SAFETY != 0
					vTracePrintF(trcSaveMgr,"SM State: Recovering -> Ok (FSM, %d)",m_eSafetyState);
					vTracePrintF(trcSaveMgr,"CheckError -> No Error (FSM, %d)",m_SafetyCheckError);
#endif
				}

				// Check EMC stop and errors
				if (bEMCStopActivated)
				{
					m_eSafetyState = ESafetyMgrStatus_EmergencyStop;
#if TRACEALYZER != 0 && TRC_SAFETY != 0
					vTracePrintF(trcSaveMgr,"SM State: Recovering -> EMstop (FSM, %d)",m_eSafetyState);
#endif
				}
				if ((nErrors != 0) && (nErrors != ESafetyMgrErrors_RecoveryTimeout))
				{
					m_eSafetyState = ESafetyMgrStatus_SafetyActivated;
#if TRACEALYZER != 0 && TRC_SAFETY != 0
					vTracePrintF(trcSaveMgr,"SM State: Recovering -> Safety Activated (FSM, %d)",m_eSafetyState);
#endif
				}
				break;
         }
         case ESafetyMgrStatus_EmergencyStop:
         {
				if (!bEMCStopActivated) 	// Wait until EM Stop gets released
            {
					// Set the Timeout for the Error Recovery from EMStop
					m_nDelayErrorRecovTimeout = SystemTime::GetTime() + EMSTOP_RECOVERY_TIMEOUT;
//					m_bDelayErrorRecovery = true;
					m_eSafetyState = ESafetyMgrStatus_SafetyActivated;
					m_SafetyCheckError = eSM_Error_SafetyActivated;
#if TRACEALYZER != 0 && TRC_SAFETY != 0
					vTracePrintF(trcSaveMgr,"SM State: EMstop -> Safety Activated (FSM, %d)",m_eSafetyState);
					vTracePrintF(trcSaveMgr,"CheckError -> Safety Activated (FSM, %d)",m_SafetyCheckError);
#endif
				}
				break;
//				  if (!bEMCStopActivated) 	// Wait until EM Stop gets released
//            {
//               if (nErrors != 0)
//					  {
//#if TRACEALYZER != 0 && TRC_SAFETY != 0
//						  vTracePrint(trcSaveMgr,"SM State: EMstop -> Safety Activated (FSM)");
//						  vTracePrint(trcSaveMgr,"CheckError -> Safety Activated (FSM)");
//#endif
//						  m_eSafetyState = ESafetyMgrStatus_SafetyActivated;
//						  m_SafetyCheckError = eSM_Error_SafetyActivated;
//					  }
//					  else
//					  {
//#if TRACEALYZER != 0 && TRC_SAFETY != 0
//						  vTracePrint(trcSaveMgr,"SM State: EMStop -> OK (FSM)");
//#endif
//						  m_eSafetyState = ESafetyMgrStatus_Ok;
//					  }
//				  }
//            break;
			}
         case ESafetyMgrStatus_SafetyActivated:
			{
//				if (m_bDelayErrorRecovery)
//				{
//					if (m_nDelayErrorRecovTimeout < SystemTime::GetTime())
//					{
//#if TRACEALYZER != 0 && TRC_SAFETY != 0
//						vTracePrint(trcSaveMgr,"EMstop Recov. Timeout (FSM)");
//#endif
//						m_bDelayErrorRecovery = false;
//					}
//				}
//				else
//				{
					// Back to ok if no errors during 2s
					if (nErrors == 0)
					{
						if (m_nBackToOkDeadline < SystemTime::GetTime())
						{
							m_SafetyCheckError = eSM_Error_NoError;
							if (bEMCStopActivated)
							{
								m_eSafetyState = ESafetyMgrStatus_EmergencyStop;
#if TRACEALYZER != 0 && TRC_SAFETY != 0
								vTracePrintF(trcSaveMgr,"SM State: SafetyActivated -> EMStop (FSM, %d)",m_eSafetyState);
								vTracePrintF(trcSaveMgr,"CheckError -> No Error (FSM, %d)",m_SafetyCheckError);
#endif
							}
							else
							{
								m_eSafetyState = ESafetyMgrStatus_Ok;
#if TRACEALYZER != 0 && TRC_SAFETY != 0
								vTracePrintF(trcSaveMgr,"SM State: SafetyActivated -> OK (FSM, %d)",m_eSafetyState);
								vTracePrintF(trcSaveMgr,"CheckError -> No Error (FSM, %d)",m_SafetyCheckError);
#endif
							}
						}
					}
					else
					{
						m_nBackToOkDeadline = SystemTime::GetTime() + DELAY_BEFORE_GOING_BACK_TO_OK;
					}
//				}
            break;
			}
			case ESafetyMgrStatus_Error:
			{
				if (bClearErrorRequest)
				{
					m_eSafetyState = ESafetyMgrStatus_Ok;
					m_SafetyCheckError = eSM_Error_NoError;
#if TRACEALYZER != 0 && TRC_SAFETY != 0
					vTracePrintF(trcSaveMgr,"SM State: Error -> OK [Clear Err] (FSM, %d)",m_eSafetyState);
					vTracePrintF(trcSaveMgr,"CheckError -> No Error (FSM, %d)",m_SafetyCheckError);
#endif
				}
				break;
			}
			default:
			{
				m_eSafetyState = ESafetyMgrStatus_Error;
				m_SafetyCheckError = eSM_Error_UnknownState;
#if TRACEALYZER != 0 && TRC_SAFETY != 0
				vTracePrintF(trcSaveMgr,"SM State: Default -> ERROR [Clear Err] (FSM, %d)",m_eSafetyState);
				vTracePrintF(trcSaveMgr,"CheckError -> Unknown (FSM, %d)",m_SafetyCheckError);
#endif
				break;
			}
		}

      // Update outputs
      uint32_t nStatusDetails = 0;
      switch (m_eSafetyState)
      {
			case ESafetyMgrStatus_Ok:
				m_pARMOkOut->Set();
            m_pRecoveryOut->Reset();
            break;
         case ESafetyMgrStatus_Recovering:
				nStatusDetails = nSensors;
            m_pARMOkOut->Set();
            m_pRecoveryOut->Set();
            break;
         case ESafetyMgrStatus_EmergencyStop:
            break;
         case ESafetyMgrStatus_SafetyActivated:
         case ESafetyMgrStatus_Error:
         default:
            m_pARMOkOut->Reset();
            m_pRecoveryOut->Reset();
            nStatusDetails = nErrors;
            break;
		}

		// Update the status data availabe from the CAN network
		uint32_t nStatus = BuildSafetyStatus(m_eSafetyState, nStatusDetails);
		m_SafetyTaskError = nErrors;
		m_SafetyTaskStatus = nStatus;
      m_pStatus->Write(nStatus);
		// Wait for next cycle
		Wait(10);
	}
}

// ----------------------------------------------------------------------------
//! \brief Check if the Emergency Stop Button is active
bool SafetyMgr::IsEMstopActive(void)
{
	return m_bEMCStopActivated;
}

// ----------------------------------------------------------------------------
//! \brief Sets or Resets the "Recover From EMStop Flag
//bool SafetyMgr::RecoverFromEMStop(bool Release)
//{
//#if TRACEALYZER != 0 && TRC_SAFETY != 0
//	vTracePrintF(trcSaveMgr,"Error Recovery requested: %d (RecoverFromEMStop)",
//					 Release ? 1 : 0);
//#endif
//	m_bDelayErrorRecovery = Release;
//	return true;
//}

// ----------------------------------------------------------------------------
//! \brief Check individually each level of the safety chain 
bool SafetyMgr::CheckSafetyChain(uint32_t retry)
{
uint32_t 	nResult = 0;
uint32_t 	nStatus;
bool 			bFloorOk;
bool 			bTestBumper1;
bool 			bTestBumper2;
bool 			bTestBumper3;
bool 			bTestBumper4;
uint32_t		retry_ctr = 0;
    
#if TRACEALYZER != 0 && TRC_SAFETY != 0
	vTracePrintF(trcSaveMgr,"Execute CheckSafetyChain, Check Flags: %08X",m_bDoSafetyCheck);
#endif
	// Retry until all IR and bumpers are ok and ANT is ok
   bool bRetry = true;
	// Reset the Safety Check Error 
	m_SafetyCheckError = eSM_Error_NoError;
	while (bRetry)
	{
		bRetry = false;

      // Force ARM not ok
      m_pARMOkOut->Reset();
         
      // Check that 24V safety is present
		if (m_bDoSafetyCheck & (1 << 0))		// Check activated
		{
#if TRACEALYZER != 0 && TRC_SAFETY != 0
			vTracePrint(trcSaveMgr,"Check Safety: 24V ok");
#endif
			if (m_pTest24SafetyIn->Read() != 1)
			{
				nResult = ESafetyMgrErrors_Check24V; 
				m_SafetyCheckError = eSM_Error_Check24V;
				goto ErrorOut;
			}
		}

		// Check the recovery relay
		if (m_bDoSafetyCheck & (1 << 1))		// Check activated
		{
#if TRACEALYZER != 0 && TRC_SAFETY != 0
			vTracePrint(trcSaveMgr,"Check Safety: Recovery");
#endif
			m_pRecoveryOut->Reset();
			Wait(100);
			if (m_pTestRecoveryIn->Read() != 0)
			{
				nResult = ESafetyMgrError_CheckRecovery;
				m_SafetyCheckError = eSM_Error_CheckRecovery;
				goto ErrorOut;
			}
			m_pRecoveryOut->Set();
			Wait(100);
			if (m_pTestRecoveryIn->Read() == 0)
			{
				nResult = ESafetyMgrError_CheckRecovery;
				m_SafetyCheckError = eSM_Error_CheckRecovery;
				goto ErrorOut;
			}
		}

      // Open bumper relay (recovery relay already closed) to test the next signals
		m_pSafetyCheckLogOut->Reset();
      Wait(100); 

      // Check ARM OK
		if (m_bDoSafetyCheck & (1 << 2))		// Check activated
		{
#if TRACEALYZER != 0 && TRC_SAFETY != 0
			vTracePrint(trcSaveMgr,"Check Safety: ARM ok");
#endif
			if (m_pTestARMIn->Read() != 0)
			{
				nResult = ESafetyMgrError_CheckARMOk;
				m_SafetyCheckError = eSM_Error_CheckARMOk;
				goto ErrorOut;
			}
			m_pARMOkOut->Set();
			Wait(100);
			if (m_pTestARMIn->Read() == 0)
			{
				nResult = ESafetyMgrError_CheckARMOk;
				m_SafetyCheckError = eSM_Error_CheckARMOk;
				goto ErrorOut;
			}  
		}

      // Check ANT OK
		if (m_bDoSafetyCheck & (1 << 3))		// Check activated
		{
#if TRACEALYZER != 0 && TRC_SAFETY != 0
			vTracePrint(trcSaveMgr,"Check Safety: ANT ok");
#endif
			// Check ANT ok, Ask IO board to set it to 0
			nStatus = BuildSafetyStatus(ESafetyMgrStatus_CheckANTOk0, 0);
			m_pStatus->Write(nStatus);
			Wait(300);
			// This is the direct return (3.3V-Level)
			if (m_pSafetyChkOutANT->Read() != 0)
			{
				bRetry = true;
				nResult = ESafetyMgrError_CheckANTOk;
				m_SafetyCheckError = eSM_Error_CheckANTOk;
				goto ErrorOut;
			}		
			// This is the sense output (feedback) of the 3.3V->24V Level Translation
			if (m_pTestANTOkIn->Read() != 0)
			{
				bRetry = true;
				nResult = ESafetyMgrError_CheckANTOk;
				m_SafetyCheckError = eSM_Error_CheckANTOk;
				goto ErrorOut;
			}
			// Check ANT ok => Ask IO board to set it to 1
			nStatus = BuildSafetyStatus(ESafetyMgrStatus_CheckANTOk1, 0);
			m_pStatus->Write(nStatus);
			Wait(300);
			// This is the direct return (3.3V-Level)
			if (m_pSafetyChkOutANT->Read() != 1)
			{
				bRetry = true;
				nResult = ESafetyMgrError_CheckANTOk;
				m_SafetyCheckError = eSM_Error_CheckANTOk;
				goto ErrorOut;
			}		
			// This is the sense output (feedback) of the 3.3V->24V Level Translation
			if (m_pTestANTOkIn->Read() != 1)
			{
				bRetry = true; 
				nResult = ESafetyMgrError_CheckANTOk;
				m_SafetyCheckError = eSM_Error_CheckANTOk;
				goto ErrorOut;
			}
			// Free ANT ok => Ask IO board to control it normally
			nStatus = BuildSafetyStatus(ESafetyMgrStatus_Ok, 0);
			m_pStatus->Write(nStatus);
		}

      // Deactivate recovery to test bumpers and floor sensors
      m_pRecoveryOut->Reset();
      m_pSafetyCheckLogOut->Set();

      // Open ARM OK to disable the safety while testing
      m_pARMOkOut->Reset();

#ifndef TEST_WITHOUT_BUMPERS
		// Wait until bumpers and IRs are not activated and test if relay is closed
		if (m_bDoSafetyCheck & (1 << 4))		// Check activated
		{
#if TRACEALYZER != 0 && TRC_SAFETY != 0
			vTracePrint(trcSaveMgr,"Check Safety: Bumper 1");
#endif
			if (m_pBumper1In->Read() == 0)
			{
				bRetry = true; 
				nResult = ESafetyMgrError_CheckIRBumper;
				m_SafetyCheckError = eSM_Error_Bumper1;
				goto ErrorOut;
			}
		}
		if (m_bDoSafetyCheck & (1 << 5))		// Check activated
		{
#if TRACEALYZER != 0 && TRC_SAFETY != 0
			vTracePrint(trcSaveMgr,"Check Safety: Bumper 2");
#endif
			if (m_pBumper2In->Read() == 0)
			{ 
				bRetry = true; 
				nResult = ESafetyMgrError_CheckIRBumper; 
				m_SafetyCheckError = eSM_Error_Bumper2;
				goto ErrorOut;
			}
		}
		if (m_bDoSafetyCheck & (1 << 6))		// Check activated
		{
#if TRACEALYZER != 0 && TRC_SAFETY != 0
			vTracePrint(trcSaveMgr,"Check Safety: Bumper 3");
#endif
			if (m_pBumper3In->Read() == 0)
			{ 
				bRetry = true; 
				nResult = ESafetyMgrError_CheckIRBumper; 
				m_SafetyCheckError = eSM_Error_Bumper3;
				goto ErrorOut;
			}
		}
		if (m_bDoSafetyCheck & (1 << 7))		// Check activated
		{
#if TRACEALYZER != 0 && TRC_SAFETY != 0
			vTracePrint(trcSaveMgr,"Check Safety: Bumper 4");
#endif
			if (m_pBumper4In->Read() == 0)
			{ 
				bRetry = true;
				nResult = ESafetyMgrError_CheckIRBumper;
				m_SafetyCheckError = eSM_Error_Bumper4;
				goto ErrorOut;
			}
		}
		if (m_bDoSafetyCheck & (1 << 8))		// Check activated
		{
#if TRACEALYZER != 0 && TRC_SAFETY != 0
			vTracePrint(trcSaveMgr,"Check Safety: Floor 1");
#endif
			if (m_pFloor1In->Read() == 0)
			{
				bRetry = true;
				nResult = ESafetyMgrError_CheckIRBumper;
				m_SafetyCheckError = eSM_Error_Floor1;
				goto ErrorOut;
			}
		}
		if (m_bDoSafetyCheck & (1 << 9))		// Check activated
		{
#if TRACEALYZER != 0 && TRC_SAFETY != 0
			vTracePrint(trcSaveMgr,"Check Safety: Floor 2");
#endif
			if (m_pFloor2In->Read() == 0)
			{ 
				bRetry = true; 
				nResult = ESafetyMgrError_CheckIRBumper;
				m_SafetyCheckError = eSM_Error_Floor2;
				goto ErrorOut;
			}
		}
		if (m_bDoSafetyCheck & (1 << 10))		// Check activated
		{
#if TRACEALYZER != 0 && TRC_SAFETY != 0
			vTracePrint(trcSaveMgr,"Check Safety: Floor 3");
#endif
			if (m_pFloor3In->Read() == 0)
			{
				bRetry = true;
				nResult = ESafetyMgrError_CheckIRBumper;
				m_SafetyCheckError = eSM_Error_Floor3;
				goto ErrorOut;
			}
		}
		if (m_bDoSafetyCheck & (1 << 11))		// Check activated
		{
#if TRACEALYZER != 0 && TRC_SAFETY != 0
			vTracePrint(trcSaveMgr,"Check Safety: Floor 4");
#endif
			if (m_pFloor4In->Read() == 0)
			{
				bRetry = true;
				nResult = ESafetyMgrError_CheckIRBumper;
				m_SafetyCheckError = eSM_Error_Floor4;
				goto ErrorOut; 
			}
		}
		if (m_bDoSafetyCheck & (1 << 12))		// Check activated
		{
#if TRACEALYZER != 0 && TRC_SAFETY != 0
			vTracePrint(trcSaveMgr,"Check Safety: Bumpers");
#endif
			if (m_pTestBumperIn->Read() == 0)
			{
				nResult = ESafetyMgrError_CheckIRBumper;
				m_SafetyCheckError = eSM_Error_CheckIRBumper;
				goto ErrorOut; 
			}
		}
		if (m_bDoSafetyCheck & (1 << 13))		// Check activated
		{
#if TRACEALYZER != 0 && TRC_SAFETY != 0
			vTracePrint(trcSaveMgr,"Check Safety: Ask ANT for Bumpers Test");
#endif
			// Ask ANT if Bumpers need to be tested
			nStatus = BuildSafetyStatus(ESafetyMgrStatus_IsBumpTestRequired, 0);
			m_pStatus->Write(nStatus);
			Wait(300);
			ESafetyMgrRequests eRequest;
			SplitSafetyRequest(m_pRequest->Read(), eRequest);
			m_pRequest->Write(0);
			// Free ANT ok => Ask IO board to control it normally
			nStatus = BuildSafetyStatus(ESafetyMgrStatus_Ok, 0);
			m_pStatus->Write(nStatus);
			if ((eRequest & ESafetyMgrRequests_NoBumperTestRequired) == 0)
			{
#if TRACEALYZER != 0 && TRC_SAFETY != 0
			vTracePrint(trcSaveMgr,"Check Safety: Perform Bumpers Test");
#endif
				// Wait until all four bumpers are pressed
				bTestBumper1 = true;
				bTestBumper2 = true;
				bTestBumper3 = true;
				bTestBumper4 = true;
				uint32_t wait = 0;
				while (bTestBumper1 || bTestBumper2 || bTestBumper3 || bTestBumper4)
				{
					// Wait an arbitrary time before checking the bumpers
					if (wait > 20000000)
					{
						bFloorOk = ((m_pFloor1In->Read() == 1) && (m_pFloor2In->Read() == 1) 
							&& (m_pFloor3In->Read() == 1) && (m_pFloor4In->Read() == 1));
						if ((m_pBumper1In->Read() == 0) && (m_pBumper2In->Read() == 1) && 
							(m_pBumper3In->Read() == 1) && (m_pBumper4In->Read() == 1) && 
							bFloorOk && bTestBumper1)
						{
							Wait(100);
							if (m_pTestBumperIn->Read() != 0)
							{
								nResult = ESafetyMgrError_CheckIRBumper;
								m_SafetyCheckError = eSM_Error_Bumper1;
								goto ErrorOut;
							}
							bTestBumper1 = false;
						}
						if ((m_pBumper1In->Read() == 1) && (m_pBumper2In->Read() == 0) && 
							(m_pBumper3In->Read() == 1) && (m_pBumper4In->Read() == 1) && 
							bFloorOk && bTestBumper2)
						{
							Wait(100);
							if (m_pTestBumperIn->Read() != 0) 
							{
								nResult = ESafetyMgrError_CheckIRBumper;
								m_SafetyCheckError = eSM_Error_Bumper2;
								goto ErrorOut;
							}
							bTestBumper2 = false;
						}
						if ((m_pBumper1In->Read() == 1) && (m_pBumper2In->Read() == 1) &&
							(m_pBumper3In->Read() == 0) && (m_pBumper4In->Read() == 1) && 
							bFloorOk && bTestBumper3)
						{
							Wait(100);
							if (m_pTestBumperIn->Read() != 0)
							{ 
								nResult = ESafetyMgrError_CheckIRBumper;
								m_SafetyCheckError = eSM_Error_Bumper3;
								goto ErrorOut;
							}
							bTestBumper3 = false;
						}
						if ((m_pBumper1In->Read() == 1) && (m_pBumper2In->Read() == 1) && 
							(m_pBumper3In->Read() == 1) && (m_pBumper4In->Read() == 0) &&
							bFloorOk && bTestBumper4)
						{
							Wait(100);
							if (m_pTestBumperIn->Read() != 0)
							{
								nResult = ESafetyMgrError_CheckIRBumper;
								m_SafetyCheckError = eSM_Error_Bumper4;
								goto ErrorOut;
							}
							bTestBumper1 = false;
						}
					}
					else
					{
						wait++;
					}
				}
			}
		}
#endif            
		// Tests passed
      // Current status of system: Recovery = 0, SafetyCheckLogout = 1, ANT OK = 1
		// Set ARM OK
      m_pARMOkOut->Set();
		// Reset all Errors and Error Counters
      m_nWatchdogErrors = 0;
      m_nANTOkErrors = 0;
      m_nSensorsErrors = 0;
      m_nRecoveryErrors = 0;
      m_nLastWatchdog = 0;
		// Reset the Clear Error Request
      m_bClearErrorRequest = false;
		// All done, status OK (no errors)
#if TRACEALYZER != 0 && TRC_SAFETY != 0
		vTracePrint(trcSaveMgr,"CheckError -> NONE (CheckSafetyChain)");
#endif
      return true;

ErrorOut:		// We get here in case of an error 
		// Write the Error Status to the Object Dictionary 
      nStatus = BuildSafetyStatus(ESafetyMgrStatus_CheckFailed, nResult);
      m_pStatus->Write(nStatus);
#if TRACEALYZER != 0 && TRC_SAFETY != 0
		vTracePrintF(trcSaveMgr,"CheckError -> %X (CheckSafetyChain)",nStatus);
#endif
		// is the Retry Flag set?
      if (!bRetry)
      {
         // Errors detected, Set control signal to a defined state before exiting
			// Reset ARM OK
         m_pARMOkOut->Reset();
			// Set SafetyCheckLogOut
         m_pSafetyCheckLogOut->Set();
			// Reset RecoveryOut
         m_pRecoveryOut->Reset();
#ifdef DBGPRINTF_SAFETY
			dbgprintf("   Error %d in Safety Chain: <%s>, Exiting ...\n",(int)m_SafetyCheckError,
				m_SafetyMgrErrorString[(int)m_SafetyCheckError]);
#endif
#if TRACEALYZER != 0 && TRC_SAFETY != 0
			vTracePrint(trcSaveMgr,"No Retry (CheckSafetyChain)");
#endif
			return false;
		}
		else
		{
			// Is the Retry Counter activated (retry > 0)?
			if (retry > 0)
			{
#ifdef DBGPRINTF_SAFETY
				dbgprintf("   Error %d in Safety Chain: <%s>, Retry %d of %d\n",
					(int)m_SafetyCheckError,m_SafetyMgrErrorString[(int)m_SafetyCheckError],
					retry_ctr + 1,retry);
#endif
				retry_ctr++;
#if TRACEALYZER != 0 && TRC_SAFETY != 0
				vTracePrintF(trcSaveMgr,"Retry, Cntr = %d (CheckSafetyChain)",retry_ctr);
#endif
				// Is the number of maximum retries reached?
				if (retry_ctr == retry)
				{
#if TRACEALYZER != 0 && TRC_SAFETY != 0
					vTracePrint(trcSaveMgr,"Max number of Retries reached, abort");
#endif
					// Maximum number of retries reached
					// Reset ARM OK
					m_pARMOkOut->Reset();
					// Set SafetyCheckLogOut
					m_pSafetyCheckLogOut->Set();
					// Reset RecoveryOut
					m_pRecoveryOut->Reset();
					return false;
				}
			}
#ifdef DBGPRINTF_SAFETY
			else
			{
				// we will retry
				dbgprintf("   Error in Safety Chain, Retrying ...\n");
			}
#endif
		}
      Wait(2500);
	}
   return true;
}

// ----------------------------------------------------------------------------
//! \brief Called by CANNode to give us a chance to register our CAN data
void SafetyMgr::RegisterData(CANNode &_node, uint16_t _nObjIndex)
{
	// CUC inputs
	dbgprintf("Safety Manager, Registering Data, Object Index = %d ....\n",(int)_nObjIndex);
	dbgprintf("   Register SubID %d\n",SAFETY_ANTOK_ID);
	_node.DeclareData(_nObjIndex, SAFETY_ANTOK_ID, m_pANTOk);
	dbgprintf("   Register SubID %d\n",SAFETY_REQUEST_ID);
   _node.DeclareData(_nObjIndex, SAFETY_REQUEST_ID, m_pRequest);
    
    // CUC outputs
	dbgprintf("   Register SubID %d\n",SAFETY_STATUS_ID);
	_node.DeclareData(_nObjIndex, SAFETY_STATUS_ID, m_pStatus);
	dbgprintf("... Safety Manager, Registering Data done\n");
}

// ----------------------------------------------------------------------------
//! \brief Return the states of bumper and floor sensors
uint32_t SafetyMgr::CheckSensorStates(ESafetyMgrRequests _eRequest)
{
	uint32_t nSensors = 0;
    
   // Handle the error simulation request and output
   if ((_eRequest & ESafetyMgrRequests_SimulateObstacleDetection) != 0)
   {
		m_pSafetyCheckLogOut->Reset();
      nSensors |= ESafetyMgrErrors_Simulation;
//#if TRACEALYZER != 0 && TRC_SAFETY != 0
//		if (nSensors != m_OldSensorState)
//		{
//			vTracePrintF(trcSaveMgr,"Simulation, Sensors = %X (CheckSensorStates)",nSensors);
//			m_OldSensorState = nSensors;
//		}
//#endif
   }
   else
   {
//#if TRACEALYZER != 0 && TRC_SAFETY != 0
//		if (nSensors != m_OldSensorState)
//		{
//			vTracePrint(trcSaveMgr,"Set SafetyCheckLogOut (CheckSensorStates)");
//			m_OldSensorState = nSensors;
//		}
//#endif
		m_pSafetyCheckLogOut->Set();
   }

#ifndef TEST_WITHOUT_BUMPERS
	// Check bumper and floor inputs
   if ((m_bDoSafetyCheck & (1 << 4)) && 0 == m_pBumper1In->Read())
		nSensors |= ESafetyMgrErrors_BumperLeft;
   if ((m_bDoSafetyCheck & (1 << 5)) && 0 == m_pBumper2In->Read())
		nSensors |= ESafetyMgrErrors_BumperRight;
   if ((m_bDoSafetyCheck & (1 << 6)) && 0 == m_pBumper3In->Read())
		nSensors |= ESafetyMgrErrors_BumperLeft;		// TODO: Check error flag
   if ((m_bDoSafetyCheck & (1 << 7)) && 0 == m_pBumper4In->Read())
		nSensors |= ESafetyMgrErrors_BumperRight;
   if ((m_bDoSafetyCheck & (1 << 8)) && 0 == m_pFloor1In->Read())
		nSensors |= ESafetyMgrErrors_Floor1;
   if ((m_bDoSafetyCheck & (1 << 9)) && 0 == m_pFloor2In->Read())
		nSensors |= ESafetyMgrErrors_Floor2;
   if ((m_bDoSafetyCheck & (1 << 10)) && 0 == m_pFloor3In->Read())
		nSensors |= ESafetyMgrErrors_Floor3;
   if ((m_bDoSafetyCheck & (1 << 11)) && 0 == m_pFloor4In->Read())
		nSensors |= ESafetyMgrErrors_Floor4;
#if TRACEALYZER != 0 && TRC_SAFETY != 0
	if (nSensors != m_OldSensorState)
	{
		vTracePrintF(trcSaveMgr,"CheckSensorStates: %X (CheckSensorStates)",nSensors);
		m_OldSensorState = nSensors;
	}
#endif
#else
    m_pRecoveryOut->Set();
#endif 
    return nSensors;
}

// ----------------------------------------------------------------------------
//! \brief Return the list of current errors
uint32_t SafetyMgr::CheckErrors(uint32_t _nSensors, bool &_bEMCStopActivated)
{
uint32_t 							nErrors = 0;
bool 									bANTOk;
uint16_t 							nWatchdog;

   // Handle watchdog
   SplitSafetyANTOk(m_pANTOk->Read(), bANTOk, nWatchdog);
   if ((m_bDoSafetyCheck & (1 << 16)) && nWatchdog == m_nLastWatchdog) 
   {   
		m_nWatchdogErrors++;
      if (m_nWatchdogErrors > NB_MAX_MISSED_WATCHDOG)
		{
			nErrors |= ESafetyMgrErrors_CANTimeout;
			INC_ERR_CNT(eCNT_WATCHDOG);
		}
   }
   else
   {
		if (m_nWatchdogErrors > m_max)
			m_max = m_nWatchdogErrors;								// DEBUG: Store max value
		m_nWatchdogErrorAvg.Update(m_nWatchdogErrors);			// DEBUG: Compute average value
      m_nLastWatchdog = nWatchdog;
      m_nWatchdogErrors = 0;
		m_nLastWatchdog = nWatchdog;
	}
    
   // 24V 
   if ((m_bDoSafetyCheck & (1 << 0)) && 0 == m_pTest24SafetyIn->Read()) 
   {
		nErrors |= ESafetyMgrErrors_E24VError;
		INC_ERR_CNT(eCNT_24V);
   }

   // ANT ok (from CAN)
   if ((m_bDoSafetyCheck & (1 << 14)) && 0 == bANTOk) 
   {
		if (++m_nANTOkErrors > NB_MAX_ERRORS)
		{
			nErrors |= ESafetyMgrErrors_ANTError;
			INC_ERR_CNT(eCNT_ANT_OK);
		}
   }
   else
   {
		m_nANTOkErrors = 0;
   }
        
   // EMC Stop => Set only if the rest of the chain is ok
   if ((m_bDoSafetyCheck & (1 << 15)) && (0 != m_pTestANTOkIn->Read()) && (0 == m_pTestEMStopIn->Read()))
   {			
		_bEMCStopActivated = (m_EMCStopCount > NB_MIN_EMCSTOP_ACTIVE);
		if (!_bEMCStopActivated)
			m_EMCStopCount++;
		else
			INC_ERR_CNT(eCNT_EMC_STOP);
   }
   else
	{
      m_EMCStopCount = 0;
		_bEMCStopActivated = false;
	}			

   // Check test inputs:
   // - TestBumper should be down if sensors have detected something
   // - TestRecovery input should not be down if recovery is active
   // - TestRecovery input should be down if recovery is not active
	if (m_bDoSafetyCheck & (1 << 16))
	{		
		bool bRecovering = m_pRecoveryOut->Read();
		if (_nSensors == 0 && 0 == m_pTestBumperIn->Read())
		{
			if (++m_nSensorsErrors > NB_MAX_ERRORS)
			{
				nErrors |= ESafetyMgrErrors_BumperTestError;
				INC_ERR_CNT(eCNT_BMP_TEST);
			}
		}
		else
		{
			m_nSensorsErrors = 0;
		}	
		if ((bRecovering && 0 == m_pTestRecoveryIn->Read()) || (!bRecovering && 0 != m_pTestRecoveryIn->Read()))
		{
			if (++m_nRecoveryErrors > NB_MAX_ERRORS)
			{
				nErrors |= ESafetyMgrErrors_RecoveryTestError;   
				INC_ERR_CNT(eCNT_RECOVERY);
			}
		}
		else
		{
			m_nRecoveryErrors = 0;
		}
	}
	else
	{
		m_nSensorsErrors = 0;
		m_nRecoveryErrors = 0;
	}
#if TRACEALYZER != 0 && TRC_SAFETY != 0
	if (m_nSensorsErrors != 0 || m_nRecoveryErrors != 0)
		vTracePrintF(trcSaveMgr,"Errors: Sens. = %X, Recov. = %X (CheckErrors)",m_nSensorsErrors,m_nRecoveryErrors);
#endif
	return nErrors;
}

void SafetyMgr::EnableChecks(uint32_t CheckFlags)
{
	if (SafetyMgrInstance != nullptr)
	{
#ifdef DBGPRINTF_SAFETY
		dbgprintf("Setting Safety Manager Checks to %08X ...\n",CheckFlags);
#endif
		SafetyMgrInstance->m_bDoSafetyCheck = CheckFlags;
#ifdef DBGPRINTF_SAFETY
		dbgprintf("... done\n");
#endif
#if TRACEALYZER != 0 && TRC_SAFETY != 0
		vTracePrintF(trcSaveMgr,"Checks = %X (EnableChecks)",CheckFlags);
#endif
	}
}

uint32_t SafetyMgr::GetTaskStatus(void)
{
	if (SafetyMgrInstance != nullptr)
	{
		uint32_t		ret_val = 0;
		ret_val = SafetyMgrInstance->m_bDoSafetyCheck;
		if (SafetyMgrInstance->IsTaskRunning())
			ret_val |= (1U << 31);
		return ret_val;
	}
	else
		return 0;
}

bool SafetyMgr::GetStatus(uint32_t *SafetyState,uint32_t *SafetyCheckError,uint32_t *WatchDogErrors,
								  uint32_t *ANTOkErrors, uint32_t *SensorErrors,uint32_t *RecoveryErrors,
								  uint32_t *EnabledTests,uint8_t *TaskIsRunning)
{
	if (SafetyMgrInstance != nullptr)
	{
		*SafetyState = SafetyMgrInstance->m_eSafetyState;
		*SafetyCheckError = SafetyMgrInstance->m_SafetyCheckError;
		*WatchDogErrors = SafetyMgrInstance->m_nWatchdogErrors;
		*ANTOkErrors = SafetyMgrInstance->m_nANTOkErrors;
		*EnabledTests = SafetyMgrInstance->m_bDoSafetyCheck;
		*SensorErrors = SafetyMgrInstance->m_nSensorsErrors;
		*RecoveryErrors = SafetyMgrInstance->m_nRecoveryErrors;
		if (SafetyMgrInstance->IsTaskRunning())
			*TaskIsRunning = 1;
		else
			*TaskIsRunning = 0;
		return true;
	}
	else
		return false;
}

bool SafetyMgr::GetIntStatus(uint32_t *SafetyTaskError,uint32_t *SafetyTaskStatus)
{
	if (SafetyMgrInstance != nullptr)
	{
		*SafetyTaskError = SafetyMgrInstance->m_SafetyTaskError;
		*SafetyTaskStatus = SafetyMgrInstance->m_SafetyTaskStatus;
		return true;
	}
	else
		return false;
}

bool SafetyMgr::GetErrCounters(uint8_t *nErrCntr,int size)
{
	if (SafetyMgrInstance != nullptr)
	{
		for (int i = 0;i < MAX_SAFETY_ERR_COUNTERS;i++)
		{
			if (i >= size)
				return false;
			else
				nErrCntr[i] = SafetyMgrInstance->m_ErrorCntr[i];
		}
		return true;
	}
	else
		return false;
}

bool SafetyMgr::ClearErrCounters(void)
{
	if (SafetyMgrInstance != nullptr)
	{
		for (int i = 0;i < MAX_SAFETY_ERR_COUNTERS;i++)
			SafetyMgrInstance->m_ErrorCntr[i] = 0;		
		return true;
	}
	else
		return false;
}

extern "C" void SafetyMngrEnableChecks(uint32_t CheckFlags)
{
	SafetyMgr::EnableChecks(CheckFlags);
}

extern "C" bool SafetyMngrGetStatus(uint32_t *SafetyState,uint32_t *SafetyCheckError,uint32_t *WatchDogErrors,
												uint32_t *ANTOkErrors, uint32_t *SensorErrors,uint32_t *RecoveryErrors,
												uint32_t *EnabledTests,uint8_t *TaskIsRunning)
{
	return SafetyMgr::GetStatus(SafetyState,SafetyCheckError,WatchDogErrors,ANTOkErrors,SensorErrors,
										 RecoveryErrors,EnabledTests,TaskIsRunning);
}

extern "C" bool SafetyMngrGetIntStatus(uint32_t *SafetyTaskError,uint32_t *SafetyTaskStatus)
{
	return SafetyMgr::GetIntStatus(SafetyTaskError,SafetyTaskStatus);
}

extern "C" bool SafetyMngrGetErrCounters(uint8_t *ErrCntrs,int size)
{
	return SafetyMgr::GetErrCounters(ErrCntrs,size);
}

extern "C" bool SafetyMngrClrErrCounters(void)
{
	return SafetyMgr::ClearErrCounters();
}
