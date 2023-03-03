// ---------------------------------------------------------------------------
//! \package     CleaningUnit
//! \file        CleaningUnitMgr.h
//! \brief       Defines the class responsible for management of all cleaning components
//! \details     <UL><LI>Brush rotation and lift</LI>
//!                  <LI>Suction and lift</LI>
//!                  <LI>Water pumps</LI></UL>
//!
//! \copyright   Copyright (C) 2011-2012 BlueBotics SA
// ----------------------------------------------------------------------------

#ifndef _CLEANINGUNITMGR_H_
#define _CLEANINGUNITMGR_H_

// ----------------------------------------------------------------------------
// Includes
#if defined(__cplusplus)

#include "cmsis_os2.h"
#include "Base.h"
#include "Task_CMSIS2.h"
#include "SafetyMgr.h"
#include "CleaningDevice.h"
#include "BrushDevice.h"
#include "SuctionDevice.h"
#include "LiftDevice.h"
#include "WaterPumpDevice.h"
#include "CANIds.h"
#include "CANNode.h"
#include "ProcessData.h"

#define MAN_START_DEVICE_MODE				1

#define N_CLEANING_DEVICES					5

#define CLM_DEVICE_LIFT_BRUSH				0
#define CLM_DEVICE_LIFT_SUCTION			1
#define CLM_DEVICE_BRUSH					2
#define CLM_DEVICE_SUCTION					3
#define CLM_DEVICE_WATER_PUMP				4

#define CLM_LIFT_UP							1
#define CLM_LIFT_DOWN						2
#define CLM_START								3
#define CLM_STOP								4

#define DIR_CMD_PUMPS_ON					0
#define DIR_CMD_PUMPS_OFF					1
#define DIR_CMD_BRUSH_ON					2
#define DIR_CMD_BRUSH_OFF					3
#define DIR_CMD_SUCTION_ON					4
#define DIR_CMD_SUCTION_OFF				5
#define DIR_CMD_BRUSH_LIFT_ENABLE		6
#define DIR_CMD_BRUSH_LIFT_DISABLE		7
#define DIR_CMD_BRUSH_LIFT_RESET			8
#define DIR_CMD_BRUSH_LIFT_UP				9
#define DIR_CMD_BRUSH_LIFT_DOWN			10
#define DIR_CMD_SUCTION_LIFT_ENABLE		11
#define DIR_CMD_SUCTION_LIFT_DISABLE	12
#define DIR_CMD_SUCTION_LIFT_RESET		13
#define DIR_CMD_SUCTION_LIFT_UP			14
#define DIR_CMD_SUCTION_LIFT_DOWN		15

#define MAX_TIME_DEVICE_READY				30000		//!< Maximum Time for a Device to get ready (in ms)

#define MAX_FATAL_ERROR_CNT				32000
#define FATAL_ERROR_THRESHOLD				3
#define FATAL_ERROR_TIMESTEP				5000

#define AUX_FLAGS_FATAL_CNT_MET			0						

typedef enum
{
	ECleaningRFSstate_Start = 0,
	ECleaningRFSstate_Active,
	ECleaningRFSstate_Initializing,
	ECleaningRFSstate_Error
} eRFSstate_t;

// ----------------------------------------------------------------------------
//! \brief List the different status of CleaningUnitMgr
typedef enum
{
	EClMgrFSMState_NotInitialized 	= 0x01,
	EClMgrFSMState_Initializing   	= 0x02,
	EClMgrFSMState_Error          	= 0x03,
	EClMgrFSMState_Stopped        	= 0x04,
	EClMgrFSMState_Starting       	= 0x05,
	EClMgrFSMState_Running        	= 0x06,
	EClMgrFSMState_Stopping       	= 0x07
} ECleaningUnitMgrFSMstate;

typedef enum
{
	eEMRCV_Waiting = 0,
	eEMRCV_Initializing,
	eEMRCV_Active,
	eEMRCV_Error
} eEMRCV_t;

typedef enum
{
	eCLErrReason_NoError = 0,
	eCLErrReason_EnaDevFailed = 1,
	eCLErrReason_StartCleaningFailed = 2,
	eCLErrReason_StopCleaningFailed = 3,
	eCLErrReason_StopStateNotReached = 4,
	eCLErrReason_ResetFSMOccured = 5,
	eCLErrReason_ClearErrorOccured = 6,
	eCLErrReason_EMStopError = 7,
	eCLErrReason_DeviceError = 8,
	eCLErrReason_SafetyFailed = 9,
	eCLErrReason_AutoClearError = 10
} eCLErrorReason_t;

// ----------------------------------------------------------------------------
//! \class      CleaningUnitMgr
//! \brief      Class responsible for managing cleaning devices and running the cleaning cycle.
class CleaningUnitMgr : public CUC_Task, public ICANNodeDataProvider
{
public:
    CleaningUnitMgr(SafetyMgr &_safetyMgr,
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
							DigitalOutput *_pDosingValve);
    virtual ~CleaningUnitMgr() {}
    //! \cond
    HideDefaultMethods(CleaningUnitMgr);
    //! \endcond 

public: // Task
    virtual void Main(void);

public: // ICANNodeDataProvider
    virtual void RegisterData(CANNode &_node, uint16_t _nObjIndex);

protected:
    bool AddDevice(CleaningDevice &_device,const char *name);
    bool CheckStatus(ECleaningDeviceStatus _status);        // Check that all devices are in the specified status
    bool CheckNotStatus(ECleaningDeviceStatus _status);     // Check that no device is in the specified status
    bool EnableDevices(void);
    bool StartCleaning(void);
    bool StopCleaning(void);
    bool DisableDevices(void);
    bool HandleDeviceCommands(ECleaningUnitMgrRequest _eRequest, ECleaningUnitMgrDevices _eDevices);
	 bool StartDeviceDelayed(unsigned Device,unsigned Command);
	 bool HandleDeviceDelayed(void);

public: // For testing purpose only...
    ECleaningUnitMgrFSMstate GetState();
    void SendRequest(ECleaningUnitMgrRequest eRequest);
	 bool RequestState(uint16_t *state,int size);
	 bool RequestDeviceState(uint8_t *status,int size);
	 bool RequestMaxDeviceCurrent(uint32_t *current,int size);
	 bool ResetMaxDeviceCurrent(uint16_t reset);
	 bool GetNumberOfFlowPulses(uint32_t *result);
	 bool getCANopenContent(int index,uint32_t *Content);
	 uint32_t GetEndSwitchState(void);
	 void ResetEndSwitchState(void);
	 void ResetCleaningMngrErrorState(void);
	 bool HandleDirectCommand(uint32_t command,uint32_t params);
	 bool SetDryRun(uint16_t enable);
	 bool IsInEMstop(void);
	 bool SetBrushSuctionRampSlope(uint8_t Device,unsigned Slope,unsigned SlopeDiv);

private:
    SafetyMgr &						m_SafetyMgr;

    LiftDevice 						m_BrushLift;
    LiftDevice 						m_SuctionLift;
    BrushDevice 						m_Brush;
    SuctionDevice 					m_Suction;
    WaterPumpDevice 					m_WaterPump;
    DigitalInput *					m_pMotorPowerSupplyOn;
    DigitalOutput *					m_pDosingValve;

    CleaningDevice *					m_apDevices[N_CLEANING_DEVICES];
    int 				  					m_apDevicesStatusOld[N_CLEANING_DEVICES];

    uint16_t m_nObjectIndex;
    ProcessDataOut<uint32_t> *	m_pStatus;
    ProcessDataOut<uint32_t> *	m_pRequest;
    ProcessDataOut<uint16_t> *	m_pWaterSettings;
    ProcessDataOut<uint32_t> *	m_pBrushMaxCurrent;
    ProcessDataOut<uint32_t> *	m_pBrushLiftMaxCurrent;
    ProcessDataOut<uint32_t> *	m_pSuctionMaxCurrent;
    ProcessDataOut<uint32_t> *	m_pSuctionLiftMaxCurrent;
    ProcessDataOut<uint32_t> *	m_pWaterPumpMaxCurrent;
    ProcessDataOut<uint32_t> *	m_pVersionNumber;
    ProcessDataOut<uint16_t> *	m_pDryRun;
    ProcessDataOut<uint32_t> *	m_pDebugInfo;
    
    bool 								m_bPrevDryRun;
	 unsigned							nErrors_Old;	
	 eCLErrorReason_t					m_ErrorReason;	 
	 uint32_t							m_DeviceStatusBits;
	 uint8_t								m_ActualState[N_CLEANING_DEVICES];
	 
	 ECleaningUnitMgrFSMstate		m_State;
	 ECleaningUnitMgrStatus			m_ClMgrStatus;
	 bool									m_ResetCleaningMangerFSM;
	 uint8_t 							m_Errors;
	 uint8_t								m_DeviceStatus;
	 uint32_t							m_FlowPulses; 
	 eRFSstate_t						m_eRFSstate;
	 bool									m_PowerState;
	 bool									m_EMstopActive;
	 uint8_t								m_AuxFlags;
	 int									m_FatalErrorCntr;
	 uint32_t							mFatalErrorTimeout;
#if TRACEALYZER != 0 && TRC_CLEAN != 0
	 int									m_old_status;
	 bool									m_OldOkState;
	 bool									m_OldNOkState;
	 ECleaningUnitMgrFSMstate 		m_PreviousClMgrState;
	 ECleaningUnitMgrFSMstate 		m_OldGetState;
	 uint32_t							m_oldCANstate;
	 uint8_t								m_OldErrors;
	 uint8_t								m_PreviousStatus;
#endif

private:
	void UpdateCANdata(ECleaningUnitMgrStatus eState,uint8_t nErrors,
			uint8_t nDeviceStatus);
	void EnableDryRun(void);
	uint8_t PrepareDeviceStatus(void);
	uint8_t PrepareErrorFeedback(void);
	bool GetPowerState(void);
	bool HoldDevices(bool Hold);
	bool CheckAndHandlePowerStateAndEMstop(void);
	eEMRCV_t CleaningRFS_FSM(bool Reset);
	bool CheckForDeviceErrors(void);
	void HandleFatalErrorCnt(bool CountUp,bool ResetCounter);
};

extern "C" bool ClMgr_GetStatus(uint16_t *state,int size);
extern "C" bool ClMgr_GetDeviceStatus(uint8_t *status,int size);
extern "C" bool ClMgr_GetMaxDeviceCurrent(uint32_t *current,int size);
extern "C" bool ClMgr_ResetMaxDeviceCurrent(uint16_t reset);
extern "C" bool ClMgr_GetNumberOfFlowPulses(uint32_t *pulses);
extern "C" uint32_t ClMgr_GetLiftEndSWstate(void);
extern "C" void ClMgr_ClearLiftEndSWstate(void);
extern "C" void ClMgr_ResetFSMerrorState(void);
extern "C" bool ClMgr_SendDirectCommand(uint32_t command,uint32_t params);
extern "C" bool ClMgr_EnableDryRun(uint8_t enable);
extern "C" bool ClMgr_SetRampSlope(uint8_t Device,unsigned Slope,unsigned SlopeDiv);

#else

#include <stdint.h>
#include <stdbool.h>

extern bool ClMgr_GetStatus(uint16_t *state,int size);
extern bool ClMgr_GetDeviceStatus(uint8_t *status,int size);
extern bool ClMgr_GetMaxDeviceCurrent(uint32_t *current,int size);
extern bool ClMgr_ResetMaxDeviceCurrent(uint16_t reset);
extern bool ClMgr_GetNumberOfFlowPulses(uint32_t *pulses);
extern uint32_t ClMgr_GetLiftEndSWstate(void);
extern void ClMgr_ClearLiftEndSWstate(void);
extern void ClMgr_ResetFSMerrorState(void);
extern bool ClMgr_SendDirectCommand(uint32_t command,uint32_t params);
extern bool ClMgr_EnableDryRun(uint8_t enable);
extern bool ClMgr_SetRampSlope(uint8_t Device,unsigned Slope,unsigned SlopeDiv);

#endif

#endif // _CLEANINGUNITMGR_H_
