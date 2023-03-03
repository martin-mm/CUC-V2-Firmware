// ---------------------------------------------------------------------------
//! \package     CleaningUnit
//! \file        CleaningUnitMgr.cpp
//! \brief       Defines the class responsible for management of all cleaning components
//! \details     <UL><LI>Brush rotation and lift</LI>
//!                  <LI>Suction and lift</LI>
//!                  <LI>Water pumps</LI></UL>
//!
//! \copyright   Copyright (C) 2011-2012 BlueBotics SA
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Includes
#include "CleaningUnit.h"
#include "CleaningUnitMgr.h"
#include "MotorDriver.h"
#include "IO.h"
#include "Version.h"
#include "SvnRevision.h"
#include "board.h"
#include "Debug.h"

#define LIFT_TEST
#undef LIFT_TEST

#define USE_DELAYED_START
#undef USE_DELAYED_START

Trace 		g_Trace;

#if (TRACEALYZER != 0) && (TRC_CLEAN != 0)
static traceString 				trcClMgr;
#endif

static CleaningUnitMgr	* ClMngr = nullptr;

static const char	osCANTaskName[] = "CLEANING-MGR";

// ----------------------------------------------------------------------------
//! \brief Constructor
CleaningUnitMgr::CleaningUnitMgr(SafetyMgr &_safetyMgr, 
                                 MotorDriver &_brushMotor,
                                 MotorDriver &_suctionMotor,
                                 MotorDriver &_brushLift,
											HallSensorInput *_pBrushLiftHallSensor,
                                 MotorDriver &_suctionLift,
											HallSensorInput *_pSuctionLiftHallSensor,
                                 MotorDriver &_pump1,
                                 MotorDriver &_pump2,
											DigitalInput *_pFlowMeterCapture,
                                 DigitalInput *_pMotorPowerSupplyOn,
											DigitalOutput * _pDosingValve)
    : CUC_Task(osPriorityLow2,osCANTaskName),
      m_SafetyMgr(_safetyMgr),
      m_BrushLift(ECleaningUnitMgrErrors_BrushLift, _brushLift, &_brushMotor, _pBrushLiftHallSensor,1),
      m_SuctionLift(ECleaningUnitMgrErrors_SuctionLift, _suctionLift, NULL, _pSuctionLiftHallSensor,2),
      m_Brush(_brushMotor,3),
      m_Suction(_suctionMotor,4),
      m_WaterPump(_pump1, _pump2, _pFlowMeterCapture,5),
      m_pMotorPowerSupplyOn(_pMotorPowerSupplyOn),
		m_pDosingValve(_pDosingValve),
      m_pStatus(NULL),
      m_pRequest(NULL),
      m_bPrevDryRun(false)
{
	dbgprintf("Cleaning Manager Constructor ...\n");
	// Init the EndSwitch 1 Input
	// Init array of devices
#if TRACEALYZER != 0 && TRC_CLEAN != 0
	vTracePrintF(trcClMgr,"Constructor: Device Brush Lift = 1, Error ID = %d",ECleaningUnitMgrErrors_BrushLift);
	vTracePrintF(trcClMgr,"Constructor: Device Suction Lift = 2, Error ID = %d",ECleaningUnitMgrErrors_SuctionLift);
	vTracePrint(trcClMgr,"Constructor: Device Brush = 3");
	vTracePrint(trcClMgr,"Constructor: Device Suction = 4");
	vTracePrint(trcClMgr,"Constructor: Device Waterpump = 5");
	m_PreviousStatus = 0xFF;
	m_OldOkState = false;
	m_OldNOkState = false;
	m_OldGetState = EClMgrFSMState_Error;
	m_PreviousClMgrState = EClMgrFSMState_Error;
	m_oldCANstate = 0xFFFFFFFF;
	m_OldErrors = 0xFF;
#endif

	int n = sizeof(m_apDevices)/sizeof(m_apDevices[0]);
	for (int i=0; i<n; i++)
	{
		m_apDevices[i] = NULL;
		m_apDevicesStatusOld[i] = -1;
	}

	AddDevice(m_SuctionLift,"SuctionLift");
	AddDevice(m_WaterPump,"Waterpump");
	AddDevice(m_BrushLift,"BrushLift");
	AddDevice(m_Brush,"Brush");
	AddDevice(m_Suction,"Suction");
	
	nErrors_Old = 0;
	m_ErrorReason = eCLErrReason_NoError;

	// Init lift devices
	m_BrushLift.InitPID(250, 0 ,0);
//	m_BrushLift.InitPositions(3*CNTS_PER_MM, 33*CNTS_PER_MM);
	m_BrushLift.InitPositions(5*CNTS_PER_MM, 33*CNTS_PER_MM);
#if LIFTMOTOR_VERSION == 1
	m_BrushLift.InitHoming(EMotorDriverMode_Clockwise, 0, 3700, 7000, 4200, 200);
#endif
#if LIFTMOTOR_VERSION == 2
#ifdef LIFT_TEST
/* Parameters: 
		Homing Direction,
		Homing Postition,
		Current Limit,
		Max Duration,
		Start Current Limit,
		Start Duration 
*/
	m_BrushLift.InitHoming(EMotorDriverMode_Clockwise, 0, 350, 7000, 450, 200);
#else	
//	m_BrushLift.InitHoming(EMotorDriverMode_Clockwise, 0, 3300, 7000, 4500, 200);
	m_BrushLift.InitHoming(EMotorDriverMode_Clockwise, 0, 6000, 7000, 4500, 200);
#endif
#endif
	m_BrushLift.InitPositionRegulation(7500, 10100, 6500);
#ifdef LIFT_TEST
	m_BrushLift.SetCurrentMax(600,600);
#else
	m_BrushLift.SetCurrentMax(7000,7000);
#endif
	m_SuctionLift.InitPID(10000, 0 ,0);
#if LIFTMOTOR_VERSION == 1
	m_SuctionLift.InitPositions(-6*CNTS_PER_MM, -40*CNTS_PER_MM);  //maximal motion of the motor is 41 mm
	m_SuctionLift.InitHoming(EMotorDriverMode_CounterClockwise, 0, 3900, 7000, 4400, 200);
#endif
#if LIFTMOTOR_VERSION == 2
	m_SuctionLift.InitPositions(-6*CNTS_PER_MM, -34*CNTS_PER_MM);  //maximal motion of the motor is 35 mm
//	m_SuctionLift.InitPositions(-6*CNTS_PER_MM, -340*CNTS_PER_MM);  //maximal motion of the motor is 350 mm	// Only for Testing the Overcurrent Condition
#ifdef LIFT_TEST
/* Parameters: 
		Homing Direction,
		Homing Postition,
		Current Limit,
		Max Duration,
		Start Current Limit,
		Start Duration 
*/
	m_SuctionLift.InitHoming(EMotorDriverMode_CounterClockwise, 0, 3300, 7000, 470, 200);
#else
	m_SuctionLift.InitHoming(EMotorDriverMode_CounterClockwise, 0, 6000, 7000, 4700, 200);
#endif
#endif
#ifdef LIFT_TEST
	m_SuctionLift.SetCurrentMax(600,600);
#else
	m_SuctionLift.SetCurrentMax(7000,7000);
#endif
	m_State = EClMgrFSMState_NotInitialized;
	ClMngr = this;
#if TRACEALYZER != 0 && TRC_CLEAN != 0
	trcClMgr = xTraceRegisterString("CLEANING MGR");
#endif
#if TRACEALYZER != 0 && TRC_CLEAN != 0
	m_old_status = -1;
#endif
#ifdef USE_DELAYED_START
	m_delayed_state = 0;
#endif
	m_PowerState = false;
	m_EMstopActive = false;
	m_AuxFlags = 0;
	m_ClMgrStatus = ECleaningUnitMgrStatus_NotInitialized;
	mFatalErrorTimeout = 0;
	dbgprintf("... Cleaning Unit Constructor done\n");	
}

// ----------------------------------------------------------------------------
//! \brief Updates the CAN Data
void inline CleaningUnitMgr::UpdateCANdata(ECleaningUnitMgrStatus eState,uint8_t 
														 nErrors,uint8_t nDeviceStatus)
{
uint32_t nStatus;
	
	m_pBrushMaxCurrent->Write(m_Brush.GetMaxCurrent());
	m_pBrushLiftMaxCurrent->Write(m_BrushLift.GetMaxCurrent());
	m_pSuctionMaxCurrent->Write(m_Suction.GetMaxCurrent());
	m_pSuctionLiftMaxCurrent->Write(m_SuctionLift.GetMaxCurrent());
	m_pWaterPumpMaxCurrent->Write(m_WaterPump.GetMaxCurrent());
	nStatus = BuildCUCStatus(eState, nErrors, nDeviceStatus, m_AuxFlags);
	m_pStatus->Write(nStatus);
//	m_pDryRun->Write(0);
	m_pDebugInfo->Write(m_FlowPulses = m_WaterPump.GetAbsFlowPulses());
	m_Errors = nErrors;
	m_DeviceStatus = nDeviceStatus;
#if TRACEALYZER != 0 && TRC_CLEAN != 0
	if (nStatus != m_oldCANstate)
	{
		vTracePrintF(trcClMgr,"Status   = %X (UpdateCANdata)",nStatus);
		vTracePrintF(trcClMgr,"Errors   = %X (UpdateCANdata)",nStatus,m_Errors,m_DeviceStatus);
		vTracePrintF(trcClMgr,"DevState = %X (UpdateCANdata)",nStatus,m_Errors,m_DeviceStatus);
		m_oldCANstate = nStatus;
	}
#endif
}

// ----------------------------------------------------------------------------
//! \brief Enables or Disables the Dry Run
void inline CleaningUnitMgr::EnableDryRun(void)
{
	bool bDryRun = m_pDryRun->Read();
	if (m_bPrevDryRun != bDryRun)
	{
#if TRACEALYZER != 0 && TRC_CLEAN != 0
		vTracePrintF(trcClMgr,"DryRun from %d to %d",m_pDryRun ? 1 : 0,bDryRun ? 1 : 0);
#endif
		if (bDryRun)
		{
			m_BrushLift.InitPositions(3*CNTS_PER_MM, 6*CNTS_PER_MM);
			m_SuctionLift.InitPositions(-6*CNTS_PER_MM, -10*CNTS_PER_MM);
		}
		else
		{
			m_BrushLift.InitPositions(3*CNTS_PER_MM, 33*CNTS_PER_MM);
#if LIFTMOTOR_VERSION == 1
			m_SuctionLift.InitPositions(-6*CNTS_PER_MM, -40*CNTS_PER_MM);  //maximal motion of the motor is 41 mm
#endif
#if LIFTMOTOR_VERSION == 2
			m_SuctionLift.InitPositions(-6*CNTS_PER_MM, -34*CNTS_PER_MM);  //maximal motion of the motor is 35 mm
#endif
		}
		uint32_t nSlots = sizeof(m_apDevices) / sizeof(CleaningDevice*);
		for (uint32_t i=0; i<nSlots; i++)
		{
			if (m_apDevices[i] != NULL)
			{
				if (bDryRun)  	//Add to do a "if ... else ..." because it was not working with "m_apDevices[i]->EnableDryRun(bDryRun);"
				{              //(CUC was always in error during boot-up without any reason...)
					m_apDevices[i]->EnableDryRun(true);
				}
				else
				{
					m_apDevices[i]->EnableDryRun(false);
				}
			}
		}
		m_bPrevDryRun = bDryRun;
	}
}

// ----------------------------------------------------------------------------
//! \brief Prepares the Device Status
uint8_t inline CleaningUnitMgr::PrepareDeviceStatus(void)
{
	uint8_t nDeviceStatus = 0;	
	if (m_Brush.IsRunning())
		nDeviceStatus |= ECleaningUnitDeviceStatus_BrushRunning;
	if (m_Suction.IsRunning())
		nDeviceStatus |= ECleaningUnitDeviceStatus_SuctionRunning;
	if (m_WaterPump.IsRunning())
		nDeviceStatus |= ECleaningUnitDeviceStatus_WaterPumpRunning;
	if (m_BrushLift.IsExecutingCommand())
		nDeviceStatus |= ECleaningUnitDeviceStatus_BrushLiftExecuteCmd;
	if (m_SuctionLift.IsExecutingCommand())
		nDeviceStatus |= ECleaningUnitDeviceStatus_SuctionLiftExecuteCmd;
	if (m_BrushLift.IsUp())
		nDeviceStatus |= ECleaningUnitDeviceStatus_BrushLiftIsUp;
	if (m_SuctionLift.IsUp())
		nDeviceStatus |= ECleaningUnitDeviceStatus_SuctionLiftIsUp;
#if TRACEALYZER != 0 && TRC_CLEAN != 0
	if (m_PreviousStatus != nDeviceStatus)
	{
		vTracePrintF(trcClMgr,"Device Status changed from 0x%X to 0x%X (PEF)",m_PreviousStatus,nDeviceStatus);
		m_PreviousStatus = nDeviceStatus;
	}
#endif
	return nDeviceStatus;
}

// ----------------------------------------------------------------------------
//! \brief Prepares the Error Feedback
uint8_t inline CleaningUnitMgr::PrepareErrorFeedback(void)
{
	uint8_t nErrors = 0;	
	if (m_Brush.GetStatus() == ECleaningDeviceStatus_Error)
		nErrors |= ECleaningUnitMgrErrors_Brush;
	if (m_Suction.GetStatus() == ECleaningDeviceStatus_Error)
		nErrors |= ECleaningUnitMgrErrors_Suction;
	if (m_WaterPump.GetStatus() == ECleaningDeviceStatus_Error)
		nErrors |= ECleaningUnitMgrErrors_WaterPump;
	if (m_BrushLift.GetStatus() == ECleaningDeviceStatus_Error)
		nErrors |= ECleaningUnitMgrErrors_BrushLift;
	if (m_SuctionLift.GetStatus() == ECleaningDeviceStatus_Error)
		nErrors |= ECleaningUnitMgrErrors_SuctionLift;
	if (m_WaterPump.IsTankEmpty())
		nErrors |= ECleaningUnitMgrErrors_TankEmpty;
#if TRACEALYZER != 0 && TRC_CLEAN != 0
	if (m_OldErrors != nErrors)
	{
		vTracePrintF(trcClMgr,"Error Feedback changed from 0x%X to 0x%X (PEF)",m_OldErrors,nErrors);
		m_OldErrors = nErrors;
	}
#endif
	return nErrors;
}

// ----------------------------------------------------------------------------
//! \brief Check and Handle the Power State and EM Stop
bool CleaningUnitMgr::CheckAndHandlePowerStateAndEMstop(void)
{
	// Get the Actual Power State
	bool ActualPowerState = GetPowerState();
	bool ActualEMState = m_SafetyMgr.IsEMstopActive();
	
	if (ActualPowerState != m_PowerState || ActualEMState != m_EMstopActive)
	{
#if TRACEALYZER != 0 && TRC_CLEAN != 0
		vTracePrintF(trcClMgr,"Power State changed to %d",ActualPowerState ? 1 : 0);
		vTracePrintF(trcClMgr,"EMStop State changed to %d",ActualEMState ? 1 : 0);
#endif
		m_PowerState = ActualPowerState;
		m_EMstopActive = ActualEMState;
		if (!ActualPowerState || ActualEMState)
			HoldDevices(true);
		else
			HoldDevices(false);
	}
	return ActualPowerState && !ActualEMState;
}

// ----------------------------------------------------------------------------
//! \brief Handles the EMStop Recovery
eEMRCV_t CleaningUnitMgr::CleaningRFS_FSM(bool Reset)
{
	if (Reset)
	{
#if TRACEALYZER != 0 && TRC_CLEAN != 0
		vTracePrint(trcClMgr,"ClMgrRFSstate Reset to Start (FSM)");
#endif
		m_eRFSstate = ECleaningRFSstate_Start;
	}
	switch (m_eRFSstate)
	{
		case ECleaningRFSstate_Start:
			// Check if EMStop is active of Power is absent
			if (!CheckAndHandlePowerStateAndEMstop())
			{
#if TRACEALYZER != 0 && TRC_CLEAN != 0
				vTracePrint(trcClMgr,"ClMgrRFSstate Start -> Active (FSM)");
#endif
				m_eRFSstate = ECleaningRFSstate_Active;
				return eEMRCV_Active;
			}
			else
				return eEMRCV_Waiting;
		case ECleaningRFSstate_Active:
			// Check if EMStop is inactive and Power is present
			if (CheckAndHandlePowerStateAndEMstop())
			{ 
				if (!EnableDevices())
				{
#if TRACEALYZER != 0 && TRC_CLEAN != 0
					vTracePrint(trcClMgr,"Cannot enable Devices (RFS-FSM)");
					vTracePrint(trcClMgr,"ClMgrRFSstate Start -> Error");
#endif
					return eEMRCV_Error;
				}
				else
				{
#if TRACEALYZER != 0 && TRC_CLEAN != 0
					vTracePrint(trcClMgr,"Devices enabled (RFS-FSM)");
					vTracePrint(trcClMgr,"ClMgrRFSstate Active -> Initializing");
#endif
					m_eRFSstate = ECleaningRFSstate_Initializing;
					return eEMRCV_Initializing;
				}
			}
			else
				return eEMRCV_Active;
		case ECleaningRFSstate_Initializing:
			// Wait until all devices are in the "Stopped" State
			if (CheckStatus(ECleaningDeviceStatus_Stopped))
			{
				// Clear the Safety Manager Errors
				m_SafetyMgr.ClearErrors();
				// Safety Recovery Flag inactive
//				m_SafetyMgr.RecoverFromEMStop(false);
#if TRACEALYZER != 0 && TRC_CLEAN != 0
				vTracePrint(trcClMgr,"ClMgrRFSstate Initializing -> Start (FSM)");
#endif
				m_eRFSstate = ECleaningRFSstate_Start;
				return eEMRCV_Waiting;
			}
#if TRACEALYZER != 0 && TRC_CLEAN != 0
			else
			{
				if (m_old_status != m_DeviceStatusBits)
				{
					vTracePrintF(trcClMgr,"ClMgrRFSstate Initializing: DeviceState = 0x%X",m_DeviceStatusBits);
					Wait(250);			// TODO: Check if this delay is possible
					m_old_status = m_DeviceStatusBits;
				}
				else
				{
					Wait(100);
				}
				return eEMRCV_Initializing;
			}
#endif
		case ECleaningRFSstate_Error:
			return eEMRCV_Error;
		default:		// We should not get here
#if TRACEALYZER != 0 && TRC_CLEAN != 0
					vTracePrint(trcClMgr,"ClMgrRFSstate Default -> Start (FSM)");
#endif
			m_eRFSstate = ECleaningRFSstate_Start;
			return eEMRCV_Waiting;
	}
}

// ----------------------------------------------------------------------------
//! \brief Check for Device Errors
bool CleaningUnitMgr::CheckForDeviceErrors(void)
{    
		uint8_t nDeviceStatus = PrepareDeviceStatus();
		uint8_t nErrors = PrepareErrorFeedback();
		// If there are any Device Errors set the state of the FSM to 
		// EClMgrFSMState_Error
		// Update CAN data
		UpdateCANdata(m_ClMgrStatus,nErrors,nDeviceStatus);
	if (nErrors != 0) 
	{
		// Stop the Devices
		StopCleaning();
		m_ErrorReason = eCLErrReason_DeviceError;
#if TRACEALYZER != 0 && TRC_CLEAN != 0
//		if (nErrors != nErrors_Old)
//		{
			vTracePrintF(trcClMgr,"State -> Error (Errors %X %X %X) (Task)",
				nDeviceStatus,nErrors,m_ErrorReason);
			nErrors_Old = nErrors;
//		}
#endif
		return false;
	}				 
	else
		return true;
}

// ----------------------------------------------------------------------------
//! \brief Handles the Fatal Error Counter
void CleaningUnitMgr::HandleFatalErrorCnt(bool CountUp,bool ResetCounter)
{
uint32_t			 timerval;
	
	if (ResetCounter)
	{
		mFatalErrorTimeout = SystemTime::GetTime();
		m_FatalErrorCntr = 0;
	}
	else
	{
		if (CountUp)
		{
			if (m_FatalErrorCntr < MAX_FATAL_ERROR_CNT)
			{
				m_FatalErrorCntr++;
#if TRACEALYZER != 0 && TRC_CLEAN != 0
			vTracePrintF(trcClMgr,"Fatal Error Counter incremented: %d",m_FatalErrorCntr);
#endif
			}
		}
		else
		{
			timerval = SystemTime::GetTime();
			if (timerval > mFatalErrorTimeout + FATAL_ERROR_TIMESTEP) 
			{
				mFatalErrorTimeout = timerval;
				if (m_FatalErrorCntr > 0)
				{
					m_FatalErrorCntr--;
#if TRACEALYZER != 0 && TRC_CLEAN != 0
				vTracePrintF(trcClMgr,"Fatal Error Counter decremented: %d",m_FatalErrorCntr);
#endif
				}
			}
		}
	}
	if (m_FatalErrorCntr < FATAL_ERROR_THRESHOLD)
		m_AuxFlags &= ~(1 << AUX_FLAGS_FATAL_CNT_MET);
	else
		m_AuxFlags |= 1 << AUX_FLAGS_FATAL_CNT_MET;
}
// ----------------------------------------------------------------------------
//! \brief The cleaning unit task
void CleaningUnitMgr::Main(void)
{    
ECleaningUnitMgrFSMstate 	eState;
#if TRACEALYZER != 0 && TRC_CLEAN != 0
bool 								PreviousSafetyMgrStatus = true;
#endif
eEMRCV_t 						emrc_state;
	
	// * ************************************
	// * Initializing the Task              *
	// * ************************************
	dbgprintf("Cleaning Manager Task has started.\n");
#if TRACEALYZER != 0 && TRC_CLEAN != 0
	vTracePrint(trcClMgr,"Cleaning Manager Task started");
#endif

	// Wait until all CAN data are initialized
   while (NULL == m_pStatus || NULL == m_pRequest || NULL == m_pWaterSettings ||
          NULL == m_pBrushMaxCurrent || NULL == m_pSuctionMaxCurrent ||
          NULL == m_pBrushLiftMaxCurrent || NULL == m_pSuctionLiftMaxCurrent ||
          NULL == m_pWaterPumpMaxCurrent || NULL == m_pVersionNumber ||
          NULL == m_pDryRun)
	{
		Wait(50);
   }
    
   // Prepare version number
   // m_pVersionNumber->Write(SVN_REVISION & 0xFFFFFFFF);

	// Initialize the Actual Device State
	for (int i = 0;i < N_CLEANING_DEVICES;i++)
		m_ActualState[i] = 0;	
	

   // Initialize the Cleaning Manager State
   eState = m_State = EClMgrFSMState_NotInitialized;		// m_state used for debugging
   
   // Reset the EMStop Handling FSM
   CleaningRFS_FSM(true);
	
#if TRACEALYZER != 0 && TRC_CLEAN != 0
	vTracePrint(trcClMgr,"Initial State = Not Initialized (FSM)");
#endif
	dbgprintf("Cleaning Manager, CAN data initialized.\n");
	dbgprintf("Cleaning Manager, entering Main Loop.\n");
	
	// * ************************************
	// * This is the Main Task Loop         *
	// * ************************************
   while (1)	// As long as the Task is not deleted
   {
		ECleaningUnitMgrRequest 	eCommand;
      uint16_t 						nParams;

		// Fetch any Command
      SplitCUCRequest(m_pRequest->Read(), eCommand, nParams);
		if (eCommand != 0)
		{
			if (eCommand != 0)
			{
#if TRACEALYZER != 0 && TRC_CLEAN != 0
				vTracePrintF(trcClMgr,"ClMgr: CAN-Msg, Cmd = %d, Params = %d",eCommand,nParams);
#endif
			}
			eCommand = eCommand;
		}
		// Clear the Request variable
      m_pRequest->Write(0);
		
		// Reset the Error State if this has been requested
		if (eState == EClMgrFSMState_Error && m_ResetCleaningMangerFSM)
		{
			eState = m_State = EClMgrFSMState_Stopped;
			m_ErrorReason = eCLErrReason_ResetFSMOccured;
			m_ResetCleaningMangerFSM = false;
#if TRACEALYZER != 0 && TRC_CLEAN != 0
			vTracePrint(trcClMgr,"ClMgr Error Reset requested (TASK)");
#endif
		}

		// Check the Power State and EMStop
		emrc_state = CleaningRFS_FSM(false);
		// EMStop is active and FSM is not in State Stopped
		if (emrc_state == eEMRCV_Active && eState != EClMgrFSMState_Stopped)
		{
			// So send the Cleaning manager FSM into the Stopped State
			eState = EClMgrFSMState_Stopped;
		}
		else
			// EMStop Handler is in Error State and FSM is not in State Error
			if (emrc_state == eEMRCV_Error && eState != EClMgrFSMState_Error)	// Error in EMStop Handler State Machine
			{
				// This causes also an Error of the Cleaning Unit Manager
				CleaningRFS_FSM(true);		// Reset the EMStop Handling FSM
				m_ErrorReason = eCLErrReason_EMStopError;
				eState = EClMgrFSMState_Error;	
				HandleFatalErrorCnt(true,false);
			}
		if (eState != EClMgrFSMState_NotInitialized && eState != EClMgrFSMState_Initializing && eState != EClMgrFSMState_Error)
			HandleFatalErrorCnt(false,false);
#if TRACEALYZER != 0 && TRC_CLEAN != 0
		if (m_PreviousClMgrState != eState)
		{
			vTracePrintF(trcClMgr,"ClMgr State: %d",eState);
			m_PreviousClMgrState = eState;
		}
#endif
		// * ******************************************
		// * This is the State Machine of the Handler *
		// * ******************************************
      switch (eState)
      {
			// * ************************************
			// * State: Not Initialized             *
			// * ************************************
			case EClMgrFSMState_NotInitialized:
				CheckForDeviceErrors();
				// Start initialization only once power supply is present
				// and the EM-Stop Signal from the Flexisoft is not active
            if ((NULL != m_pMotorPowerSupplyOn) && 
                (m_pMotorPowerSupplyOn->Read() != 0) &&
					 m_PowerState &&
					 !m_EMstopActive)
            { 
					// Enable the Cleaning Devices
               if (!EnableDevices())
               {
#if TRACEALYZER != 0 && TRC_CLEAN != 0
						vTracePrint(trcClMgr,"Cannot enable Devices (FSM)");
						vTracePrint(trcClMgr,"ClMgrState NotInit -> Error (FSM)");
#endif
						m_ErrorReason = eCLErrReason_EnaDevFailed;
						// Go to the Error State if Enabling of the devices failed
						eState = EClMgrFSMState_Error;
						HandleFatalErrorCnt(true,false);
               }
					else
					{
#if TRACEALYZER != 0 && TRC_CLEAN != 0
						vTracePrint(trcClMgr,"Devices enabled (FSM)");
						vTracePrint(trcClMgr,"ClMgrState NotInit -> Initialize (FSM)");
#endif
						// Cleaning Devices are enabled now so start Initializing
						eState = EClMgrFSMState_Initializing;
					}
            }
            break;

			// * ************************************
			// * State: Initializing                *
			// * ************************************
			case EClMgrFSMState_Initializing:
				// Wait until all devices are in the "Stopped" State
				CheckForDeviceErrors();
            if (CheckStatus(ECleaningDeviceStatus_Stopped))
            {
					// Clear the Safety Manager Errors
					m_SafetyMgr.ClearErrors();
					// Safety Recovery Flag inactive
//					m_SafetyMgr.RecoverFromEMStop(false);
#if TRACEALYZER != 0 && TRC_CLEAN != 0
					vTracePrint(trcClMgr,"ClMgrState Initialize -> Stopped (FSM)");
#endif
					mFatalErrorTimeout = SystemTime::GetTime();
               eState = EClMgrFSMState_Stopped;
            }
#if TRACEALYZER != 0 && TRC_CLEAN != 0
				else
				{
					if (m_old_status != m_DeviceStatusBits)
					{
#if TRACEALYZER != 0 && TRC_CLEAN != 0
						vTracePrintF(trcClMgr,"ClMgrState Initialize: DeviceState = 0x%X (FSM)",m_DeviceStatusBits);
#endif
						Wait(250);			// TODO: Check if this delay is possible
						m_old_status = m_DeviceStatusBits;
					}
					else
					{
						Wait(100);
					}
				}
#endif
            break;

			// * ************************************
			// * State: Stopped                     *
			// * ************************************
			case EClMgrFSMState_Stopped:
			{
				// Stopped: Wait for the "start" request
			
				bool CheckDeviceResult = CheckForDeviceErrors();
				// Check the Power State and EMStop
				if (emrc_state != eEMRCV_Waiting)
				{
					
				}
				else
				{
					if (!CheckDeviceResult)
					{
						eState = EClMgrFSMState_Error;
						HandleFatalErrorCnt(true,false);
					}
					else if (eCommand != 0)
					{
						m_pRequest->Write(0);
						if ((eCommand & ECleaningUnitMgrRequest_Start) != 0)
						{
							// Clear safety errors
							m_SafetyMgr.ClearErrors();
							Wait(250);
							// Start
	#if TRACEALYZER != 0 && TRC_CLEAN != 0
							vTracePrint(trcClMgr,"ClMgrState Start requested (FSM)");
							vTracePrint(trcClMgr,"ClMgrState Stopped -> Starting (FSM)");
	#endif
							eState = EClMgrFSMState_Running;
						}
						else
						{
							HandleDeviceCommands(eCommand, (ECleaningUnitMgrDevices)nParams);
						}
					}
				}
				break;
			}
			// * ************************************
			// * State: Running                     *
			// * ************************************
			case EClMgrFSMState_Running:
				// Running: Wait for the incoming requests
				if (!CheckForDeviceErrors())
				{
					eState = EClMgrFSMState_Error;
					HandleFatalErrorCnt(true,false);
				}
				else if (eCommand != 0)
				{
					if ((eCommand & ECleaningUnitMgrRequest_Stop) != 0)
					{
#if TRACEALYZER != 0 && TRC_CLEAN != 0
						vTracePrint(trcClMgr,"ClMgrState Stop requested");
						vTracePrint(trcClMgr,"ClMgrState Running -> Stopping (FSM)");
#endif
						eState = EClMgrFSMState_Stopping;
					}
					else
					{
						HandleDeviceCommands(eCommand, (ECleaningUnitMgrDevices)nParams);
					}
				}				 
				if (!m_SafetyMgr.IsOk()) 
				{
					StopCleaning();
#if TRACEALYZER != 0 && TRC_CLEAN != 0
					vTracePrint(trcClMgr,"ClMgrState: Safety failed");
					vTracePrint(trcClMgr,"ClMgrState Running -> Error (FSM)");
#endif
					m_ErrorReason = eCLErrReason_SafetyFailed;
					eState = EClMgrFSMState_Error;
					HandleFatalErrorCnt(true,false);
				}
				break;

			// * ************************************
			// * State: Stopping                    *
			// * ************************************
			case EClMgrFSMState_Stopping:
				CheckForDeviceErrors();
				if (StopCleaning() && CheckStatus(ECleaningDeviceStatus_Stopped)) 
				{
#if TRACEALYZER != 0 && TRC_CLEAN != 0
					vTracePrint(trcClMgr,"ClMgrState Stop succeeded");
					vTracePrint(trcClMgr,"ClMgrState Stopping -> Stopped (FSM)");
#endif
					eState = EClMgrFSMState_Stopped;
				}
				else
				{
#if TRACEALYZER != 0 && TRC_CLEAN != 0
					vTracePrint(trcClMgr,"ClMgrState Stop failed");
					vTracePrint(trcClMgr,"ClMgrState -> Error (FSM)");
#endif
					m_ErrorReason = eCLErrReason_StopCleaningFailed;
					eState = EClMgrFSMState_Error;
					HandleFatalErrorCnt(true,false);
				}
				break;

			// * ************************************
			// * State: Error and Default           *
			// * ************************************
			case EClMgrFSMState_Error:
			default:
				if (m_FatalErrorCntr < FATAL_ERROR_THRESHOLD)
				{
					CheckForDeviceErrors();
					// Do we get here due to an EMStopp Error and can we recover or did ANT clear the error
					if (m_ErrorReason == eCLErrReason_EMStopError && !m_SafetyMgr.IsEMstopActive())
					{
						// Reset the EMStop Handler
						CleaningRFS_FSM(true);
#if TRACEALYZER != 0 && TRC_CLEAN != 0
						if (PreviousSafetyMgrStatus && !m_SafetyMgr.IsEMstopActive())
							vTracePrint(trcClMgr,"State Error: Cleared by EMstop deasserted");
						else
							vTracePrint(trcClMgr,"State Error: Cleared by Request");
						vTracePrint(trcClMgr,"State Error -> Not Initialized (FSM Clear Err)");
#endif
						eState = EClMgrFSMState_NotInitialized;
						m_ErrorReason = eCLErrReason_ClearErrorOccured;
					}
					else
					{
						// Make sure all devices are stopped
						if (m_Brush.IsRunning())
							m_Brush.Stop();
						if (m_Suction.IsRunning())
							m_Suction.Stop();
						if (m_WaterPump.IsRunning())
						{
							m_WaterPump.Stop();
							m_pDosingValve->Reset();
						}
						eState = EClMgrFSMState_NotInitialized;
						m_ErrorReason = eCLErrReason_AutoClearError;
					}
				}
				else
				{
					if (eCommand == ECleaningUnitMgrRequest_ClearError)
					{
						HandleFatalErrorCnt(false,true);
						eState = EClMgrFSMState_NotInitialized;
						m_ErrorReason = eCLErrReason_ClearErrorOccured;
					}						
				}
#if TRACEALYZER != 0 && TRC_CLEAN != 0
				uint32_t SfState = m_SafetyMgr.IsOk();
				if (PreviousSafetyMgrStatus != SfState)
					vTracePrintF(trcClMgr,"State Error: Safety State change %d to %d",PreviousSafetyMgrStatus,SfState);
				PreviousSafetyMgrStatus = SfState;
#endif
				break;
		}	// End of Switch
		// * ******************************************
		// * This is the end of State Machine         *
		// * ******************************************
		m_State = eState;		// Remember the actual state (used for debugging)
		// * ******************************************
		// * The following has to be checked			 * 
		// * permanently         						    *
		// * ******************************************

		// 1. Update water density and allow water flow to change in function of the current speed of the platform
		uint8_t nDensity;
		uint8_t nSpeed;
		SplitCUCWaterSettings(m_pWaterSettings->Read(), nDensity, nSpeed);

		// 2. Enable or Disable Dry Run Mode if needed (Dry Run State requested and changed)
		EnableDryRun();

		// 3. Set the Waterpump Parameters
		m_WaterPump.SetLinearSpeed(nSpeed);
		m_WaterPump.SetWaterDensity(nDensity * 1000);
		
		m_ClMgrStatus = (ECleaningUnitMgrStatus)eState;		// has to be removed when the new Cleaning Manager Status has been defined

		Wait(10);
	}	// End of While
}

// ----------------------------------------------------------------------------
//! \brief Called by CANNode to give us a chance to register our CAN data
void CleaningUnitMgr::RegisterData(CANNode &_node, uint16_t _nObjIndex)
{
#if TRACEALYZER != 0 && TRC_CLEAN != 0
	vTracePrint(trcClMgr,"ClMgrState: Register Data [RegisterData]");
#endif
	dbgprintf("Cleaning Manager, Registering Data, Object Index = %d ....\n",(int)_nObjIndex);
	m_nObjectIndex = _nObjIndex;
	dbgprintf("   Register SubID %d\n",CLEANING_STATUS_ID);
	_node.DeclareData(_nObjIndex, CLEANING_STATUS_ID, m_pStatus);
	dbgprintf("   Register SubID %d\n",CLEANING_REQUEST_ID);
	_node.DeclareData(_nObjIndex, CLEANING_REQUEST_ID, m_pRequest);
	dbgprintf("   Register SubID %d\n",CLEANING_WATERSETTINGS_ID);
	_node.DeclareData(_nObjIndex, CLEANING_WATERSETTINGS_ID, m_pWaterSettings);
	dbgprintf("   Register SubID %d\n",CLEANING_BRUSH_MAXCURRENT_ID);
	_node.DeclareData(_nObjIndex, CLEANING_BRUSH_MAXCURRENT_ID, m_pBrushMaxCurrent);
	dbgprintf("   Register SubID %d\n",CLEANING_BRUSHLIFT_MAXCURRENT_ID);
	_node.DeclareData(_nObjIndex, CLEANING_BRUSHLIFT_MAXCURRENT_ID, m_pBrushLiftMaxCurrent);
	dbgprintf("   Register SubID %d\n",CLEANING_SUCTION_MAXCURRENT_ID);
	_node.DeclareData(_nObjIndex, CLEANING_SUCTION_MAXCURRENT_ID, m_pSuctionMaxCurrent);
	dbgprintf("   Register SubID %d\n",CLEANING_SUCTIONLIFT_MAXCURRENT_ID);
	_node.DeclareData(_nObjIndex, CLEANING_SUCTIONLIFT_MAXCURRENT_ID, m_pSuctionLiftMaxCurrent);
	dbgprintf("   Register SubID %d\n",CLEANING_WATERPUMP_MAXCURRENT_ID);
	_node.DeclareData(_nObjIndex, CLEANING_WATERPUMP_MAXCURRENT_ID, m_pWaterPumpMaxCurrent);
	dbgprintf("   Register SubID %d\n",CLEANING_VERSION_ID);
	_node.DeclareData(_nObjIndex, CLEANING_VERSION_ID, m_pVersionNumber);
   m_pVersionNumber->Write(BuildCUCVersion(VERSION_MAJOR, VERSION_MINOR, VERSION_BUILD));
	dbgprintf("   Register SubID %d\n",CLEANING_DRYRUN_ID);
	_node.DeclareData(_nObjIndex, CLEANING_DRYRUN_ID, m_pDryRun);
	dbgprintf("   Register SubID %d\n",CLEANING_DEBUG_INFO_ID);
	_node.DeclareData(_nObjIndex, CLEANING_DEBUG_INFO_ID, m_pDebugInfo);
	m_pStatus->Write(BuildCUCStatus(ECleaningUnitMgrStatus_NotInitialized, 0, 0, 0));
	m_pRequest->Write(0);
	m_pDebugInfo->Write(0);
	dbgprintf("... Cleaning Unit, Registering Data done\n");
}

// ----------------------------------------------------------------------------
//! \brief Add a device to our list of devices
bool CleaningUnitMgr::AddDevice(CleaningDevice &_device,const char *name)
{
	uint32_t nSlots = sizeof(m_apDevices) / sizeof(CleaningDevice *);
	for (int i=0;i < nSlots;i++)
	{
		if (m_apDevices[i] == NULL)
		{
			m_apDevices[i] = &_device;
			// Tell the Device the actual Power State
			m_apDevices[i]->SetHold(m_PowerState);
#if TRACEALYZER != 0 && TRC_CLEAN != 0
			vTracePrintF(trcClMgr,"ClMgrState: Add Device %d (%s) [AddDevice]",i,xTraceRegisterString(name));
#endif
//#ifdef DBGPRINTF_CLEANING						
			dbgprintf("Cleaning Mngr: Add Device (%d) %s ...\n",i+1,name);
//#endif
			return true;
		}
		else
		{
		}
	}
	return false;
}

// ----------------------------------------------------------------------------
//! \brief Sets the devices into the Hold Mode due to no power or EM Stop
bool CleaningUnitMgr::HoldDevices(bool Hold)
{
	uint32_t nSlots = sizeof(m_apDevices) / sizeof(CleaningDevice*);
	for (int i = 0; i < nSlots; i++)
	{
		if (m_apDevices[i] != NULL)
		{
			m_apDevices[i]->SetHold(Hold);
#if TRACEALYZER != 0 && TRC_CLEAN != 0
			vTracePrintF(trcClMgr,"Send Hold State %d to Device %d",Hold ? 1 : 0,i);
#endif
		}
	}
	return true;
}

// ----------------------------------------------------------------------------
//! \brief Returns true if all registered devices are in the specified state
bool CleaningUnitMgr::CheckStatus(ECleaningDeviceStatus _status)
{	
	uint32_t nSlots = sizeof(m_apDevices) / sizeof(CleaningDevice*);
	bool isOk = true;

	m_DeviceStatusBits = 0;
	for (int i = 0; i < nSlots; i++)
	{
		if (m_apDevices[i] != NULL)
		{
			if ((m_ActualState[i] = m_apDevices[i]->GetStatus()) != _status)
			{
				isOk = false;
			}
			else
			{
				m_DeviceStatusBits |= (1 << i);
			}
			if (m_ActualState[i] != m_apDevicesStatusOld[i])
			{
				m_apDevicesStatusOld[i] = m_ActualState[i];
			}
		}
	}
#if TRACEALYZER != 0 && TRC_CLEAN != 0
	if (m_OldOkState != isOk)
	{
		vTracePrintF(trcClMgr,"DevStatus: %X [OK = %d] (CheckStatus)",m_DeviceStatusBits,isOk ? 1 : 0);
		m_OldOkState = isOk;
	}
#endif
	return isOk;
}

// ----------------------------------------------------------------------------
//! \brief Returns true if all registered devices are not the specified state
bool CleaningUnitMgr::CheckNotStatus(ECleaningDeviceStatus _status)
{	
	uint32_t nSlots = sizeof(m_apDevices)/sizeof(CleaningDevice*);
	bool isOk = true;
	
	for (uint32_t i=0; i<nSlots; i++)
	{
		if (m_apDevices[i] != NULL)
		{
			if ((m_ActualState[i] = m_apDevices[i]->GetStatus()) == _status)
			{
				m_DeviceStatusBits |= (1 << i);
				isOk = false;
			}
			if (m_ActualState[i] != m_apDevicesStatusOld[i])
			{
				m_apDevicesStatusOld[i] = m_ActualState[i];
			}
		}
	}
#if TRACEALYZER != 0 && TRC_CLEAN != 0
	if (m_OldNOkState != isOk)
	{
		vTracePrintF(trcClMgr,"Errors: %X [OK = %d] (CheckNotStatus)",isOk ? 1 : 0,m_DeviceStatusBits);
		m_OldNOkState = isOk;
	}
#endif
	return isOk;
}

// ----------------------------------------------------------------------------
//! \brief Enable all cleaning devices
bool CleaningUnitMgr::EnableDevices(void)
{    
	bool bResult = true;
	bool bResultAll = true;
	uint32_t nSlots = sizeof(m_apDevices)/sizeof(CleaningDevice*);
	BOARD_SetPumpPWM(0,0);
	BOARD_SetPumpPWM(1,0);
	for (uint32_t i=0; i<nSlots; i++)
	{
		if (m_apDevices[i] != NULL)
		{
			bResult = m_apDevices[i]->Enable();
         Wait(100);
			if (bResult)
			{
				uint32_t nCount = 0;
				ECleaningDeviceStatus eStatus = m_apDevices[i]->GetStatus();
				while (eStatus != ECleaningDeviceStatus_Stopped && nCount++ < 1000)
				{
					eStatus = m_apDevices[i]->GetStatus();
					Wait(10);
				}
				bResult = (eStatus == ECleaningDeviceStatus_Stopped);
            Wait(500);
#if TRACEALYZER != 0 && TRC_CLEAN != 0
				if (nCount >= 1000)
					vTracePrint(trcClMgr,"EnableStatus = OK, Timeout Device (EnableDevices)");
				else
					vTracePrint(trcClMgr,"EnableStatus = OK, Device Stopped (EnableDevices)");
#endif
			}
			else
			{
#if TRACEALYZER != 0 && TRC_CLEAN != 0
				vTracePrint(trcClMgr,"EnableStatus = NOK (EnableDevices)");
#endif
				bResultAll = false;
			}
		}
		else
		{
#if TRACEALYZER != 0 && TRC_CLEAN != 0
			vTracePrint(trcClMgr,"EnableStatus = NOK, DevPtr = NULL (EnableDevices)");
#endif
			bResultAll = false;
		}
	}
	if (!bResultAll)
	{
		DisableDevices();
	}
	return bResultAll;
}

// ----------------------------------------------------------------------------
//! \brief Start cleaning
bool CleaningUnitMgr::StartCleaning(void)
{
bool 			bResult = true;

#if TRACEALYZER != 0 && TRC_CLEAN != 0
	uint32_t nSlots = sizeof(m_apDevices)/sizeof(CleaningDevice*);	
	vTracePrintF(trcClMgr,"N Slots = %d (StartCleaning)",nSlots);
#endif
	return bResult;
}

// ----------------------------------------------------------------------------
//! \brief Stop cleaning
bool CleaningUnitMgr::StopCleaning(void)
{
	bool bResult = true;
	uint32_t nSlots = sizeof(m_apDevices)/sizeof(CleaningDevice*);

#if TRACEALYZER != 0 && TRC_CLEAN != 0
	vTracePrintF(trcClMgr,"N Slots = %d (StopCleaning)",nSlots);
#endif
	for (int32_t i=nSlots-1; i>=0; i--)
	{
		if (m_apDevices[i] != NULL)
		{
			bResult &= m_apDevices[i]->Stop();
			if (bResult)
			{
				ECleaningDeviceStatus eStatus = m_apDevices[i]->GetStatus();
				uint64_t m_nWaitForDevice = SystemTime::GetTime() + MAX_TIME_DEVICE_READY;
#if TRACEALYZER != 0 && TRC_CLEAN != 0
				bool timeout = false;
#endif
				while (eStatus != ECleaningDeviceStatus_Stopped)
				{
					if (SystemTime::GetTime() >= m_nWaitForDevice)
					{
#if TRACEALYZER != 0 && TRC_CLEAN != 0
						timeout = true;
#endif
						break;
					}
					eStatus = m_apDevices[i]->GetStatus();
					Wait(10);
				}
#if TRACEALYZER != 0 && TRC_CLEAN != 0
				if (timeout)
					vTracePrint(trcClMgr,"Timeout waiting for Device ready (StopCleaning)");
				vTracePrintF(trcClMgr,"Dev.State(%d) = %d, OK (StopCleaning)",i,(int)eStatus);
#endif
				bResult = (eStatus == ECleaningDeviceStatus_Stopped);
			}
			else
			{
#if TRACEALYZER != 0 && TRC_CLEAN != 0
				vTracePrintF(trcClMgr,"Device %d, NOK (StopCleaning)",i);
#endif
			}
		}
	}
	return bResult;
}

// ----------------------------------------------------------------------------
//! \brief Disable all cleaning devices
bool CleaningUnitMgr::DisableDevices(void)
{
#if TRACEALYZER != 0 && TRC_CLEAN != 0
	bool		eResult;
#endif
	bool bResult = true;
	uint32_t nSlots = sizeof(m_apDevices) / sizeof(CleaningDevice*);

#if TRACEALYZER != 0 && TRC_CLEAN != 0
	vTracePrintF(trcClMgr,"N Slots = %d (DisableDevices)",nSlots);
#endif
	for (uint32_t i=0; i<nSlots; i++)
	{
		if (m_apDevices[i] != NULL)
		{
#if TRACEALYZER != 0 && TRC_CLEAN != 0
			bResult &= eResult = m_apDevices[i]->Disable();
#else
			bResult &= m_apDevices[i]->Disable();
#endif
		}
#if TRACEALYZER != 0 && TRC_CLEAN != 0
		vTracePrintF(trcClMgr,"Dev.(%d) Stopped = %d OK (DisableDevices)",i,eResult ? 1 : 0);
#endif
	}
	return bResult;
}

// ----------------------------------------------------------------------------
//! \brief Handle individual device commands
bool CleaningUnitMgr::HandleDeviceCommands(ECleaningUnitMgrRequest _eRequest, ECleaningUnitMgrDevices _eDevices)
{
#if TRACEALYZER != 0 && TRC_CLEAN != 0
		vTracePrintF(trcClMgr,"Dev. = %d, Req = %d (HandleDeviceCommands)",(int)_eDevices,(int)_eRequest);
#endif
	bool bResult = true;
	if ((_eRequest & ECleaningUnitMgrRequest_MoveDeviceUp) != 0)
	{
		if ((_eDevices & ECleaningUnitMgrDevices_Brush) != 0)
			bResult &= m_BrushLift.Lift();
		if ((_eDevices & ECleaningUnitMgrDevices_Suction) != 0)
			bResult &= m_SuctionLift.Lift();
	}
	else if ((_eRequest & ECleaningUnitMgrRequest_MoveDeviceDown) != 0)
	{
		if ((_eDevices & ECleaningUnitMgrDevices_Brush) != 0)
			bResult &= m_BrushLift.Lower();
		if ((_eDevices & ECleaningUnitMgrDevices_Suction) != 0)
			bResult &= m_SuctionLift.Lower();
	}
	else if ((_eRequest & ECleaningUnitMgrRequest_StartDevice) != 0)
	{
		if ((_eDevices & ECleaningUnitMgrDevices_Brush) != 0)
			bResult &= m_Brush.Start();
		if ((_eDevices & ECleaningUnitMgrDevices_Suction) != 0)
			bResult &= m_Suction.Start();
		if ((_eDevices & ECleaningUnitMgrDevices_WaterPump) != 0)
		{
			bResult &= m_WaterPump.Start();
			bResult &= m_pDosingValve->Set();
		}
	}
	else if ((_eRequest & ECleaningUnitMgrRequest_StopDevice) != 0)
	{
		if ((_eDevices & ECleaningUnitMgrDevices_Brush) != 0)
			m_Brush.Stop();
		if ((_eDevices & ECleaningUnitMgrDevices_Suction) != 0)
			m_Suction.Stop();
		if ((_eDevices & ECleaningUnitMgrDevices_WaterPump) != 0)
		{
			m_WaterPump.Stop();
			m_pDosingValve->Reset();
		}
	}
	else if ((_eRequest & ECleaningUnitMgrRequest_ResetMaxCurrent) != 0)
	{
		m_Brush.ResetMaxCurrent();
		m_BrushLift.ResetMaxCurrent();
		m_Suction.ResetMaxCurrent();
		m_SuctionLift.ResetMaxCurrent();
		m_WaterPump.ResetMaxCurrent();
	}
	return bResult;
}

// ----------------------------------------------------------------------------
//! \brief Return the absolute number of Flow Sensor pulses
bool CleaningUnitMgr::GetNumberOfFlowPulses(uint32_t *result)
{	
	*result = m_WaterPump.GetAbsFlowPulses();
	return true;
}

// ----------------------------------------------------------------------------
//! \brief Return the current state
ECleaningUnitMgrFSMstate CleaningUnitMgr::GetState(void)
{	
	if (NULL == m_pStatus)
	{
#if TRACEALYZER != 0 && TRC_CLEAN != 0
		if (m_OldGetState != ECleaningUnitMgrStatus_NotInitialized)
			vTracePrintF(trcClMgr,"Status = %X (GetState)",ECleaningUnitMgrStatus_NotInitialized);
		m_OldGetState = EClMgrFSMState_NotInitialized;
#endif
		return EClMgrFSMState_NotInitialized;
	}
#if TRACEALYZER != 0 && TRC_CLEAN != 0
	ECleaningUnitMgrFSMstate ActualState = m_State;
	if (ActualState != m_OldGetState)
	{
		vTracePrintF(trcClMgr,"Status = %X (GetState)",ActualState);
		m_OldGetState = ActualState;
	}
	return ActualState;
#else
	return m_State;
#endif
}

// ----------------------------------------------------------------------------
//! \brief Gets the State of the End Switches
uint32_t CleaningUnitMgr::GetEndSwitchState(void)
{	
uint32_t		state = 0;
	
	state |= (m_BrushLift.GetEndSwitchState() & 0x0F) << 0;
	state |= (m_SuctionLift.GetEndSwitchState() & 0x0F) << 4;
	return state;
}

// ----------------------------------------------------------------------------
//! \brief Resets the State of the End Switches
void CleaningUnitMgr::ResetEndSwitchState(void)
{	
	m_BrushLift.ResetEndSwitchState();
}

// ----------------------------------------------------------------------------
//! \brief Resets the Cleaning Manager Error State
void CleaningUnitMgr::ResetCleaningMngrErrorState(void)
{
	m_ResetCleaningMangerFSM = true;
}

//! \brief Request the current state and errors
bool CleaningUnitMgr::RequestState(uint16_t *state,int size)
{
	if (size < 6)
		return false;
	state[0] = m_State;
	state[1] = m_Errors;
	state[2] = m_DeviceStatus;
	state[3] = m_FlowPulses;
	state[4] = m_ErrorReason;
	state[5] = m_pDryRun->Read();
	m_ErrorReason = eCLErrReason_NoError;
	return true;
}

// ----------------------------------------------------------------------------
//! \brief Request the current state of a Device
bool CleaningUnitMgr::RequestDeviceState(uint8_t *status,int size)
{	
	if (size < N_CLEANING_DEVICES)
		return false;
	
	status[0] = m_Brush.GetStatus();
	status[1] = m_Suction.GetStatus();
	status[2] = m_WaterPump.GetStatus();
	status[3] = m_BrushLift.GetStatus();
	status[4] = m_SuctionLift.GetStatus();
	return true;
}

// ----------------------------------------------------------------------------
//! \brief Request the Maximum Device Currents
bool CleaningUnitMgr::RequestMaxDeviceCurrent(uint32_t *current,int size)
{	
	if (size < N_CLEANING_DEVICES)
		return false;
	
	current[0] = m_Brush.GetMaxCurrent();
	current[1] = m_Suction.GetMaxCurrent();
	current[2] = m_WaterPump.GetMaxCurrent();
	current[3] = m_BrushLift.GetMaxCurrent();
	current[4] = m_SuctionLift.GetMaxCurrent();
	return true;
}

// ----------------------------------------------------------------------------
//! \brief Resets the Maximum Device Currents
bool CleaningUnitMgr::ResetMaxDeviceCurrent(uint16_t reset)
{	
	for (int i = 0;i < N_CLEANING_DEVICES;i++)
	{
		switch (reset & (1 << i))
		{
			case 0x01:
				m_Brush.ResetMaxCurrent();
				break;
			case 0x02:
				m_Suction.ResetMaxCurrent();
				break;
			case 0x04:
				m_WaterPump.ResetMaxCurrent();
				break;
			case 0x08:
				m_BrushLift.ResetMaxCurrent();
				break;
			case 0x10:
				m_SuctionLift.ResetMaxCurrent();
				break;
		}
	}
	return true;
}

// ----------------------------------------------------------------------------
//! \brief Gets the CANopen Objects content
bool CleaningUnitMgr::getCANopenContent(int index,uint32_t *Content)
{	
	switch (index)
	{
		case 0:
			*Content = m_pStatus->Read();
			break;
		case 1:
			*Content = m_pRequest->Read();
			break;
		case 2:
			*Content = m_pWaterSettings->Read();
			break;
		case 3:
			*Content = m_pBrushMaxCurrent->Read();
			break;
		case 4:
			*Content = m_pBrushLiftMaxCurrent->Read();
			break;
		case 5:
			*Content = m_pSuctionMaxCurrent->Read();
			break;
		case 6:
			*Content = m_pSuctionLiftMaxCurrent->Read();
			break;
		case 7:
			*Content = m_pWaterPumpMaxCurrent->Read();
			break;
		case 8:
			*Content = m_pVersionNumber->Read();
			break;
		case 9:
			*Content = m_pDryRun->Read();
			break;
		case 10:
			*Content = m_pDebugInfo->Read();
			break;
		default:
			return false;
	}
	return true;
}

// ----------------------------------------------------------------------------
//! \brief Simulate sending a request
void CleaningUnitMgr::SendRequest(ECleaningUnitMgrRequest eRequest)
{
	if (NULL != m_pRequest)
		m_pRequest->Write(eRequest);
}

// ----------------------------------------------------------------------------
//! \brief Handles a direct command for testing purposes
bool CleaningUnitMgr::HandleDirectCommand(uint32_t command,uint32_t params)
{
#if TRACEALYZER != 0 && TRC_CLEAN != 0
	vTracePrintF(trcClMgr,"Direct Command = %X, Params = %X (HdlDirCmd)",command,params);
#endif
	switch (command)
	{
		case DIR_CMD_PUMPS_ON:
			m_WaterPump.Start();
			return true;
		case DIR_CMD_PUMPS_OFF:
			m_WaterPump.Stop();
			return true;
		case DIR_CMD_BRUSH_ON:
			m_Brush.Start();
			return true;
		case DIR_CMD_BRUSH_OFF:
			m_Brush.Stop();
			return true;
		case DIR_CMD_SUCTION_ON:
			m_Suction.Start();
			return true;
		case DIR_CMD_SUCTION_OFF:
			m_Suction.Stop();
			return true;
		case DIR_CMD_BRUSH_LIFT_ENABLE:
			m_BrushLift.Enable();
			return true;
		case DIR_CMD_BRUSH_LIFT_DISABLE:
			m_BrushLift.Disable();
			return true;
		case DIR_CMD_BRUSH_LIFT_RESET:
			return true;
		case DIR_CMD_BRUSH_LIFT_UP:
			m_BrushLift.Lift();
			return true;
		case DIR_CMD_BRUSH_LIFT_DOWN:
			m_BrushLift.Lower();
			return true;
		case DIR_CMD_SUCTION_LIFT_ENABLE:
			m_SuctionLift.Enable();
			return true;
		case DIR_CMD_SUCTION_LIFT_DISABLE:
			m_SuctionLift.Disable();
			return true;
		case DIR_CMD_SUCTION_LIFT_RESET:
			return true;
		case DIR_CMD_SUCTION_LIFT_UP:
			m_SuctionLift.Lift();
			return true;
		case DIR_CMD_SUCTION_LIFT_DOWN:
			m_SuctionLift.Lower();
			return true;
	}
	return false;
}

// ----------------------------------------------------------------------------
//! \brief Handles a direct command for testing purposes
bool CleaningUnitMgr::SetDryRun(uint16_t enable)
{
#if TRACEALYZER != 0 && TRC_CLEAN != 0
	vTracePrintF(trcClMgr,"Set Dry Run to 0x%X (SetDryRun)",enable);
#endif
	m_pDryRun->Write(enable);
	return true;
}
	
// ----------------------------------------------------------------------------
//! \brief Checks the Status of the Main Relays
bool CleaningUnitMgr::GetPowerState(void)
{
#if TRACEALYZER != 0 && TRC_LIFT != 0
static int 	old_state = -1;
int			state;	

	state = BOARD_TestRelayStatus() ? 1 : 0;
	if (old_state == -1 || state != old_state)
	{
		vTracePrintF(trcClMgr,"Relay State:  %d",state ? 1 : 0);
		old_state = state;
	}
	if (state != 0)
		return true;
	else
		return false;
#else
	return BOARD_TestRelayStatus() != 0;
#endif
}

// ----------------------------------------------------------------------------
//! \brief Sets the Ramp Slope of the Brush or Suction Device
bool CleaningUnitMgr::SetBrushSuctionRampSlope(uint8_t Device,unsigned Slope,unsigned SlopeDiv)
{
	switch(Device)
	{
		case 0:
			return m_Brush.SetRampSlope(Slope,SlopeDiv);
		case 1:
			return m_Suction.SetRampSlope(Slope,SlopeDiv);
		default:
			return false;
	}
}

// ----------------------------------------------------------------------------
// End Of Object
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
//! \brief Get the Status of the Cleaning Manager
extern "C" bool ClMgr_GetStatus(uint16_t *state,int size)
{
	if (ClMngr == nullptr)
		return false;
	return ClMngr->RequestState(state,size);
}

// ----------------------------------------------------------------------------
//! \brief Get the Status of a Cleaning Manager Device
extern "C" bool ClMgr_GetDeviceStatus(uint8_t *status,int size)
{
	if (ClMngr == nullptr)
		return false;
	return ClMngr->RequestDeviceState(status,size);
}

// ----------------------------------------------------------------------------
//! \brief Get the Maximum Device Current
extern "C" bool ClMgr_GetMaxDeviceCurrent(uint32_t *current,int size)
{
	if (ClMngr == nullptr)
		return false;
	return ClMngr->RequestMaxDeviceCurrent(current,size);
}

// ----------------------------------------------------------------------------
//! \brief Resets the Maximum Device Current
extern "C" bool ClMgr_ResetMaxDeviceCurrent(uint16_t reset)
{
	if (ClMngr == nullptr)
		return false;
	return ClMngr->ResetMaxDeviceCurrent(reset);
}

// ----------------------------------------------------------------------------
//! \brief Get the Number of Flow Pulses
extern "C" bool ClMgr_GetNumberOfFlowPulses(uint32_t *pulses)
{
	if (ClMngr == nullptr)
		return false;
	if (!ClMngr->GetNumberOfFlowPulses(pulses))
		return false;
	return true;
}

// ----------------------------------------------------------------------------
//! \brief Gets the state of the End Switches
extern "C" uint32_t ClMgr_GetLiftEndSWstate(void)
{
	if (ClMngr == nullptr)
		return 0;
	return ClMngr->GetEndSwitchState();
}

// ----------------------------------------------------------------------------
//! \brief Clears the state of the End Switches
extern "C" void ClMgr_ClearLiftEndSWstate(void)
{
	if (ClMngr != nullptr)
		ClMngr->ResetEndSwitchState();
}

// ----------------------------------------------------------------------------
//! \brief Clears the Error State of the Cleaning Manager FSM
extern "C" void ClMgr_ResetFSMerrorState(void)
{
	if (ClMngr != nullptr)
		ClMngr->ResetCleaningMngrErrorState();
}

// ----------------------------------------------------------------------------
//! \brief Sends a direct command to the Cleaning Manager
extern "C" bool ClMgr_SendDirectCommand(uint32_t command,uint32_t params)
{
	if (ClMngr != nullptr)
		return ClMngr->HandleDirectCommand(command,params);
	else
		return false;
}

// ----------------------------------------------------------------------------
//! \brief Sends a direct command to the Cleaning Manager
extern "C" bool ClMgr_EnableDryRun(uint8_t enable)
{
	if (ClMngr != nullptr)
		return ClMngr->SetDryRun(enable);
	else
		return false;
}

// ----------------------------------------------------------------------------
//! \brief Sets the Ramp Slope of a Suction or Brush Device
extern "C" bool ClMgr_SetRampSlope(uint8_t Device,unsigned Slope,unsigned SlopeDiv)
{
	return ClMngr->SetBrushSuctionRampSlope(Device,Slope,SlopeDiv);
}
