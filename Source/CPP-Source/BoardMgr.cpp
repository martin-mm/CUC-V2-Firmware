// ---------------------------------------------------------------------------
//! \package     CleaningUnit
//! \file        BoardMgr.cpp
//! \brief       Defines the class responsible for the overall management of the cleaning unit board
//!
//! \copyright   Copyright (C) 2011-2012 BlueBotics SA
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Includes
#include "BoardMgr.h"
#include "ProcessData.h"
//#include "AnalogOutput.h"

#define DBGPRINTF_BOARDMGR
#undef DBGPRINTF_BOARDMGR

#define TEST_SAFETY_MANAGER_TASK
#undef TEST_SAFETY_MANAGER_TASK

const DataProviderInfo_t BoardMgr::m_DataProviderInfo[BOARD_N_DATAPROVIDER] = 
{
	{CLEANING_OBJID,"Cleaning Manager"},
	{SAFETY_OBJID,"Safety Manager"}
};

BoardMgr *BoardMgr::BoardMgrInstance = nullptr;

// ----------------------------------------------------------------------------
// Constants
#define BOARDMGR_TASK_PERIOD 	100		// in uc cycles

// ----------------------------------------------------------------------------
//! \brief Constructor
BoardMgr::BoardMgr(void) :
	m_Timer0(0),
	m_Timer1(1),
	m_PWMDriver(2000),		// Period in us
	m_AnalogInputMgr(0),
	m_BrushCurrentMonitor(m_AnalogInputMgr.DeclareInput(ADC_BRUSH_CUR)),
	m_BrushLiftCurrentMonitor(m_AnalogInputMgr.DeclareInput(ADC_LIFT_BR_CUR)),
	m_SuctionCurrentMonitor(m_AnalogInputMgr.DeclareInput(ADC_SUCT_CUR)),
	m_SuctionLiftCurrentMonitor(m_AnalogInputMgr.DeclareInput(ADC_LIFT_SUCT_CUR)),
	m_PumpsCurrentMonitor(m_AnalogInputMgr.DeclareInput(ADC_PUMP_CUR)),
	m_BrushMotor(m_PWMDriver.DeclareOutput(PWM_CONTROL_BRUSH + 1), 
					&m_BrushCurrentMonitor,
					EMotorDriverMode_CounterClockwise,
					EMotorDriverMode_BrakeGND,
					"Brush Motor"),
	m_BrushLiftMotor(m_PWMDriver.DeclareOutput(PWM_CONTROL_LIFT_BR + 1), 
					&m_BrushLiftCurrentMonitor, 
					EMotorDriverMode_Clockwise,
					EMotorDriverMode_BrakeGND,
					"Brush Lift Motor",
					new DigitalInput(GP_END_SW1),
					nullptr,
					eBlockDirRight,
					eBlockDirNone),
	m_SuctionMotor(m_PWMDriver.DeclareOutput(PWM_CONTROL_SUCT + 1), 
				   &m_SuctionCurrentMonitor,
				   EMotorDriverMode_CounterClockwise,
					EMotorDriverMode_BrakeGND,
					"Suction Motor"),
	m_SuctionLiftMotor(m_PWMDriver.DeclareOutput(PWM_CONTROL_LIFT_SUC + 1), 
					&m_SuctionLiftCurrentMonitor,
					EMotorDriverMode_Clockwise,
					EMotorDriverMode_BrakeGND,
					"Suct Lift Motor"),
	m_PumpMotor1(m_PWMDriver.DeclareOutput(PWM_CONTROL_PUMP1 + 1), 
					&m_PumpsCurrentMonitor,
					EMotorDriverMode_BrakeGND,
					EMotorDriverMode_BrakeGND,
					"Pump 1 Motor"),
	m_PumpMotor2(m_PWMDriver.DeclareOutput(PWM_CONTROL_PUMP2 + 1), 
					&m_PumpsCurrentMonitor,
					EMotorDriverMode_BrakeGND,
					EMotorDriverMode_BrakeGND,
					"Pump 2 Motor"),
	m_CleaningUnitMgr(m_SafetyMgr,
	                  m_BrushMotor,
	                  m_SuctionMotor,
#if BOARD_VERSION == 10
							m_BrushLiftMotor, new HallSensorInput(m_Timer0, 0),
							m_SuctionLiftMotor, new HallSensorInput(m_Timer0, 1),
#endif
#if BOARD_VERSION == 11
							m_BrushLiftMotor, new HallSensorInput(GP_LIFT_BRUSH_HALL_IN),
							m_SuctionLiftMotor, new HallSensorInput(GP_LIFT_SUCT_HALL_IN),
#endif
	                  m_PumpMotor1,
	                  m_PumpMotor2,
							new FlowMeterInput(GP_FLOW_METER),
							new DigitalInput(GP_T_24V_Safety),		// TODO: Check the Digital Input, should be VB present
							new DigitalOutput(GP_VALVE_DOSING_PUMP)),
	m_SafetyMgr(osPriorityNormal2,
					new DigitalInput(GP_BUMPER0),
					new DigitalInput(GP_BUMPER1),
					new DigitalInput(GP_BUMPER2),
					new DigitalInput(GP_BUMPER3),
					new DigitalInput(GP_FLOOR0),
					new DigitalInput(GP_FLOOR1),
					new DigitalInput(GP_FLOOR2),
					new DigitalInput(GP_FLOOR3),
					new DigitalInput(GP_Test_Rec_Safety),
					new DigitalInput(GP_T_24V_Safety),
					new DigitalInput(GP_Test_Bumpers), 
					new DigitalInput(GP_Test_ARM),
					new DigitalInput(GP_Test_ANT_OK),
					new DigitalInput(GP_Test_EM_Stop),
					new DigitalInput(GP_Safety_Chk_Out_Ant),
					new AnalogInput(ADC_VB_SENSE),
					new DigitalOutput(GP_SAFETY_ChkInLog,false),
					new DigitalOutput(GP_Recover_Safety), 
					new DigitalOutput(GP_ARM_OK),
					new DigitalOutput(GP_TEST_MUX_EN),
					new DigitalOutput(GP_TEST_MUX_A0),
					new DigitalOutput(GP_TEST_MUX_A1),
					new DigitalOutput(GP_TEST_MUX_A2)),
	m_CANDriver(EDevice_CAN1, 125000),
	m_CANMgr(m_CANDriver, CAN_DEVID_CLEANINGUNIT, osPriorityBelowNormal5),
	m_RedLed(GP_LED0),
	m_GreenLed(GP_LED1)
{
   dbgprintf("Board Manager Constructor ...\n");	
	m_Timer0.Configure(10000);
	m_Timer1.Configure(10000);

	// Enable analog output for debug
//	AnalogOutput::Enable();
//	AnalogOutput::Write(0);

	// Register CAN data providers
	m_CANMgr.RegisterDataProvider(&m_CleaningUnitMgr, CLEANING_OBJID);
	m_CANMgr.RegisterDataProvider(&m_SafetyMgr, SAFETY_OBJID);

   // Start all managers
   m_CANMgr.Start(2048);
	// wait 12 seconds to let time for the flexisoft to start up
	dbgprintf("Waiting 12s to allow Flexisoft to start up: ");	
//	for (int i = 0;i < 12;i++)
//	{
//		CUC_Task::Wait(1000);  
//		dbgprintf(" %d",i+1);
//	}
//	dbgprintf(" done.\n");
	// wait 12 seconds to let time for the Flexisoft to start up
   CUC_Task::Wait(12000);
//   CUC_Task::Wait(1000);  // wait 12 seconds to let time for the flexisoft to start up
   dbgprintf("Switch Relay 1 on ...");
	if (ControlRelay1(true,2000,false))
		dbgprintf(" succeeded\n");
	else
		dbgprintf(" failed\n");
   dbgprintf("Switch Relay 2 on ...");
	if (ControlRelay2(true,false))
		dbgprintf(" succeeded\n");
	else
		dbgprintf(" failed\n");
	// wait 1 second to allow the Relays to settle
	dbgprintf("Waiting to let the Relays settle ...\n");
   CUC_Task::Wait(10000);
	dbgprintf("... done.\n");

	dbgprintf("Safety Manager: Enable Checks ...\n");
	m_SafetyMgr.EnableChecks(0xFFFFFFFF);
	dbgprintf("done ...\n");
	dbgprintf("Safety Manager: Initial Check of Safety Chain ...\n");
#ifdef TEST_SAFETY_MANAGER_TASK
	m_SafetyMgr.Start(2048);
	dbgprintf("... SUCCESS, Safety Manager Task started\n");
#else
#ifdef SAFETY_NO_INITIAL_CHECK
   if (true)
#else
   if (m_SafetyMgr.CheckSafetyChain(10))
#endif
   {
		m_SafetyMgr.Start(2048);
		dbgprintf("... SUCCESS, Safety Manager Task started\n");
   }
	else
	{
		dbgprintf("... FAILED due to error in Safety Chain, Safety State = %d\n",m_SafetyMgr.getSafetyCheckState());
	}
#endif
	dbgprintf("Enabling GPIO-Interrupts of PORTA\n");
	BOARD_Enable_Port_IRQ(PORTA);
	dbgprintf("Enabling GPIO-Interrupts of PORTC\n");
	BOARD_Enable_Port_IRQ(PORTC);
	dbgprintf("Now starting the Cleaning Unit Manager ...\n");
	m_CleaningUnitMgr.Start(2048);
    
	// Start the watchdog 
   // Note that this must be done after the check of the safety chain because it takes too much time...
	dbgprintf("Now starting the Watchdog ...\n");
	m_Watchdog.Start(10);			// TODO Check Watchdog Implementation
	
	BoardMgrInstance = this;
	
	dbgprintf("RTOS Free Heap: %d\n",xPortGetFreeHeapSize());
   dbgprintf("... Board Manager Constructor done.\n");	
}

// ----------------------------------------------------------------------------
//! \brief Restarts the Safety Task
void BoardMgr::RestartSafetyTask(void)
{
	if (BoardMgrInstance != nullptr)
	{
#ifdef DBGPRINTF_BOARDMGR
		dbgprintf("Restarting the Safety Manager Task ...\n");
#endif
		if (BoardMgrInstance->m_SafetyMgr.IsTaskRunning()) 
		{
#ifdef DBGPRINTF_BOARDMGR
			dbgprintf("Safety Manager: Initial Check of Safety Chain ...\n");
#endif
			if (BoardMgrInstance->m_SafetyMgr.CheckSafetyChain(10))
			{
				BoardMgrInstance->m_SafetyMgr.Start(2048);
#ifdef DBGPRINTF_BOARDMGR
				dbgprintf("... SUCCESS, Safety Manager Task started\n");
#endif
			}
#ifdef DBGPRINTF_BOARDMGR
			else
			{
				dbgprintf("... FAILED due to error in Safety Chain, Safety State = %d\n",BoardMgrInstance->m_SafetyMgr.getSafetyCheckState());
			}
#endif
		}
#ifdef DBGPRINTF_BOARDMGR
		else
		{
			dbgprintf("... FAILED, Safety Manager Task is not running\n");
		}
#endif
	}
}

// ----------------------------------------------------------------------------
//! \brief Gets the CANopen Status
bool BoardMgr::GetCANstatus(uint32_t *nProvider,uint32_t *id,uint32_t *nmt_state)
{
	if (BoardMgrInstance != nullptr)
	{
		*id = BoardMgrInstance->m_CANMgr.getCAN_ID();
		*nmt_state = BoardMgrInstance->m_CANMgr.getNMT_State();
		*nProvider = BoardMgrInstance->m_CANMgr.getNumberOfProvider();
		return true;
	}
	return false;
}

// ----------------------------------------------------------------------------
//! \brief Gets the content of a provider
bool BoardMgr::GetCAN_ProviderContent(uint32_t id,uint32_t subid,uint32_t *content,bool *valid)
{
	if (BoardMgrInstance != nullptr)
	{
		return BoardMgrInstance->m_CANMgr.getProviderContent(id,subid,*content,*valid);
	}
	return false;
}

// ----------------------------------------------------------------------------
//! \brief Gets the content of a provider
bool BoardMgr::GetCAN_ProviderInfo(uint32_t id,char **Name,int *ObjID)
{
	if (BoardMgrInstance != nullptr)
	{
		if (id > BOARD_N_DATAPROVIDER)
			return false;
		*Name = (char *)m_DataProviderInfo[id].Name;
		*ObjID = m_DataProviderInfo[id].ObjID;
		return true;
	}
	return false;
}

// ----------------------------------------------------------------------------
//! \brief Update LEDs
void BoardMgr::UpdateLEDs()
{
    ECleaningUnitMgrFSMstate eStatus = m_CleaningUnitMgr.GetState();
    (eStatus == ECleaningUnitMgrStatus_Running) ? m_GreenLed.Toggle() : m_GreenLed.Set();
    //(eStatus == ECleaningUnitMgrStatus_Error) ? m_RedLed.Set() : m_RedLed.Reset();
}

// ----------------------------------------------------------------------------
//! \brief Handle the CPU watchdog and refresh the ANTok output signal
void BoardMgr::HandleWatchdog()
{
	// Refresh the CPU's watchdog
	m_Watchdog.Refresh();
}

extern "C" void RestartSafetyManager(void)
{
#ifdef DBGPRINTF_BOARDMGR
	dbgprintf("Restarting the Safety Manager ...\n");
#endif
	BoardMgr::RestartSafetyTask();
#ifdef DBGPRINTF_BOARDMGR
	dbgprintf("... done.\n");
#endif
}

extern "C" bool GetCANstatus(uint32_t *nProvider,uint32_t *id,uint32_t *nmt_state)
{
#ifdef DBGPRINTF_BOARDMGR
	dbgprintf("Requesting CANopen Node Status ...\n");
#endif
	return BoardMgr::GetCANstatus(nProvider,id,nmt_state);
#ifdef DBGPRINTF_BOARDMGR
	dbgprintf("... done.\n");
#endif
}

extern "C" bool GetCAN_ProviderContent(uint32_t id,uint32_t subid,uint32_t *content,bool *valid)
{
#ifdef DBGPRINTF_BOARDMGR
	dbgprintf("Requesting CANopen Node Status ...\n");
#endif
	return BoardMgr::GetCAN_ProviderContent(id,subid,content,valid);
#ifdef DBGPRINTF_BOARDMGR
	dbgprintf("... done.\n");
#endif
}

extern "C" bool GetCAN_ProviderInfo(uint32_t id,char **Name,int *ObjID)
{
#ifdef DBGPRINTF_BOARDMGR
	dbgprintf("Requesting CANopen Node Status ...\n");
#endif
	return BoardMgr::GetCAN_ProviderInfo(id,Name,ObjID);
#ifdef DBGPRINTF_BOARDMGR
	dbgprintf("... done.\n");
#endif
}
