// ---------------------------------------------------------------------------
//! \package     CleaningUnit
//! \file        CANIds.h
//! \brief       Define CAN ids (object indexes and subindexes)
//!
//! \copyright   Copyright (C) 2011-2012 BlueBotics SA
// ----------------------------------------------------------------------------

#ifndef _CUC_CANIDS_H_
#define _CUC_CANIDS_H_

#include <stdint.h>

// ----------------------------------------------------------------------------
// Can device ids
#define CAN_DEVID_CLEANINGUNIT          		4

// ----------------------------------------------------------------------------
// Cleaning module
#define CLEANING_OBJID					    		1		// Object index
#define CLEANING_STATUS_ID 				    	0		// CUC State sub-index
#define CLEANING_REQUEST_ID 			    		1		// Request sub-index
#define CLEANING_WATERSETTINGS_ID 		    	2		// Water settings sub-index (water density and max speed ratio)
#define CLEANING_BRUSH_MAXCURRENT_ID        	3     // Brush max current sub-index
#define CLEANING_BRUSHLIFT_MAXCURRENT_ID    	4     // Brush lift max current sub-index
#define CLEANING_SUCTION_MAXCURRENT_ID      	5     // Suction max current sub-index
#define CLEANING_SUCTIONLIFT_MAXCURRENT_ID  	6     // Suction lift max current sub-index
#define CLEANING_WATERPUMP_MAXCURRENT_ID    	7     // Water pump max current sub-index
#define CLEANING_VERSION_ID                 	8     // Version numbers sub-index
#define CLEANING_DRYRUN_ID    		    		9		// Dry Run mode sub-index
#define CLEANING_DEBUG_INFO_ID              10     // CUC debug infos sub-index

// ----------------------------------------------------------------------------
// Safety module
#define SAFETY_OBJID									2		// Object index
#define SAFETY_STATUS_ID 							0		// Status sub-index
#define SAFETY_ANTOK_ID 							1		// ANT ok sub-index
#define SAFETY_REQUEST_ID               		2		// Request sub-index

// ----------------------------------------------------------------------------
//! \brief List all devices supported by CleaningUnitMgr
typedef enum
{
    ECleaningUnitMgrDevices_Brush = 0x01,
    ECleaningUnitMgrDevices_Suction = 0x02,
    ECleaningUnitMgrDevices_WaterPump = 0x04,
} ECleaningUnitMgrDevices;

// ----------------------------------------------------------------------------
//! \brief List all requests accepted by CleaningUnitMgr
typedef enum
{
    ECleaningUnitMgrRequest_Start           = 0x01,
    ECleaningUnitMgrRequest_Stop            = 0x02,
    ECleaningUnitMgrRequest_ClearError      = 0x04,
    ECleaningUnitMgrRequest_StartDevice     = 0x08,
    ECleaningUnitMgrRequest_StopDevice      = 0x10,
    ECleaningUnitMgrRequest_MoveDeviceUp    = 0x20,
    ECleaningUnitMgrRequest_MoveDeviceDown  = 0x40,
    ECleaningUnitMgrRequest_ResetMaxCurrent = 0x80
} ECleaningUnitMgrRequest;

// ----------------------------------------------------------------------------
//! \brief List the different status of CleaningUnitMgr
typedef enum
{
	ECleaningUnitMgrStatus_NotInitialized = 0x01,
	ECleaningUnitMgrStatus_Initializing   = 0x02,
	ECleaningUnitMgrStatus_Error          = 0x03,
	ECleaningUnitMgrStatus_Stopped        = 0x04,
	ECleaningUnitMgrStatus_Starting       = 0x05,
	ECleaningUnitMgrStatus_Running        = 0x06,
	ECleaningUnitMgrStatus_Stopping       = 0x07
} ECleaningUnitMgrStatus;

// ----------------------------------------------------------------------------
//! \brief List the different errors of CleaningUnitMgr
typedef enum 
{    
    ECleaningUnitMgrErrors_Brush             = ECleaningUnitMgrDevices_Brush,
    ECleaningUnitMgrErrors_Suction           = ECleaningUnitMgrDevices_Suction,
    ECleaningUnitMgrErrors_WaterPump         = ECleaningUnitMgrDevices_WaterPump,
    ECleaningUnitMgrErrors_BrushLift         = ECleaningUnitMgrDevices_Brush << 4,
    ECleaningUnitMgrErrors_SuctionLift       = ECleaningUnitMgrDevices_Suction << 4,    
    ECleaningUnitMgrErrors_TankEmpty         = 0x80,
} ECleaningUnitMgrErrors;

// ----------------------------------------------------------------------------
//! \brief List bits of device status
typedef enum
{
    ECleaningUnitDeviceStatus_BrushRunning              = ECleaningUnitMgrDevices_Brush,
    ECleaningUnitDeviceStatus_SuctionRunning            = ECleaningUnitMgrDevices_Suction,
    ECleaningUnitDeviceStatus_WaterPumpRunning          = ECleaningUnitMgrDevices_WaterPump,
    ECleaningUnitDeviceStatus_BrushLiftExecuteCmd       = ECleaningUnitMgrDevices_Brush << 4,
    ECleaningUnitDeviceStatus_SuctionLiftExecuteCmd     = ECleaningUnitMgrDevices_Suction << 4,    
    ECleaningUnitDeviceStatus_BrushLiftIsUp             = ECleaningUnitMgrDevices_Brush << 6,
    ECleaningUnitDeviceStatus_SuctionLiftIsUp           = ECleaningUnitMgrDevices_Suction << 6,    
} ECleaningUnitDeviceStatus;

// ----------------------------------------------------------------------------
//! \brief List the different status of SafetyMgr
typedef enum
{
    ESafetyMgrStatus_Ok                 = 0x00,
    ESafetyMgrStatus_CheckFailed        = 0x01,
    ESafetyMgrStatus_Recovering         = 0x02,
    ESafetyMgrStatus_SafetyActivated    = 0x03,
    ESafetyMgrStatus_EmergencyStop      = 0x04,
    ESafetyMgrStatus_Error 	          = 0x05,
    ESafetyMgrStatus_CheckANTOk0        = 0x06,
    ESafetyMgrStatus_CheckANTOk1        = 0x07,
    ESafetyMgrStatus_IsBumpTestRequired = 0x08,
} ESafetyMgrStatus;

// ----------------------------------------------------------------------------
//! \brief List the different errors of SafetyMgr
typedef enum
{  
    // Error generated during startup check (used with ESafetyMgrStatus_CheckFailed)
    ESafetyMgrErrors_Check24V           = 0x0001,
    ESafetyMgrError_CheckRecovery       = 0x0002,
    ESafetyMgrError_CheckIRBumper       = 0x0003,
    ESafetyMgrError_CheckARMOk          = 0x0004,
    ESafetyMgrError_CheckANTOk          = 0x0005,
    ESafetyMgrError_CheckEMStop         = 0x0006,
    
    // Source of obstacle detection (used with ESafetyMgrStatus_Recovering)
    ESafetyMgrErrors_BumperLeft         = 0x0001,
    ESafetyMgrErrors_BumperRight        = 0x0002,
    ESafetyMgrErrors_Floor1             = 0x0004,
    ESafetyMgrErrors_Floor2             = 0x0008,
    ESafetyMgrErrors_Floor3             = 0x0010,
    ESafetyMgrErrors_Floor4             = 0x0020,
    ESafetyMgrErrors_Simulation         = 0x0040,  // Via the LOG output
    
    // Errors during execution (used with ESafetyMgrStatus_Error)
    ESafetyMgrErrors_E24VError          = 0x0001,
    ESafetyMgrErrors_EMStopError        = 0x0002,
    ESafetyMgrErrors_ANTError           = 0x0004,
    ESafetyMgrErrors_BumperTestError    = 0x0008,
    ESafetyMgrErrors_RecoveryTestError  = 0x0010,
    ESafetyMgrErrors_RecoveryTimeout    = 0x0020,
    ESafetyMgrErrors_CANTimeout         = 0x0040,
} ESafetyMgrErrors;

// ----------------------------------------------------------------------------
//! \brief List possible requests for SafetyMgr
typedef enum
{
    ESafetyMgrRequests_None                         = 0x00,
    ESafetyMgrRequests_EnterRecovery                = 0x01,
    ESafetyMgrRequests_LeaveRecovery                = 0x02,
    ESafetyMgrRequests_ClearErrors                  = 0x04,
    ESafetyMgrRequests_SimulateObstacleDetection    = 0x08,
    ESafetyMgrRequests_NoBumperTestRequired         = 0x10,
} ESafetyMgrRequests;


// ==============================================================
// Helper methods to build/split CUC requests
inline uint32_t BuildCUCRequest(ECleaningUnitMgrRequest _eCmd, uint16_t _nParams) 
{ 
    return (((_eCmd & 0xFFFF) << 16) | (_nParams & 0xFFFF));
}
inline void SplitCUCRequest(uint32_t _nRequest, ECleaningUnitMgrRequest &_eCmd, uint16_t &_nParams)
{
    _eCmd = (ECleaningUnitMgrRequest)((_nRequest>>16) & 0xFFFF);
    _nParams = (_nRequest & 0xFFFF);
}

// ==============================================================
// Helper methods to build/split CUC status
inline uint32_t BuildCUCStatus(ECleaningUnitMgrStatus _eStatus, uint8_t _nErrors, uint8_t _nDeviceStatus,uint8_t _AuxFlags) 
{ 
    return ((_eStatus << 24) | 
				(_AuxFlags << 16) |
            (_nErrors << 8) | 
            (_nDeviceStatus & 0xFF));
}
inline void SplitCUCStatus(uint32_t _nStatus, ECleaningUnitMgrStatus &_eStatus, ECleaningUnitMgrErrors &_eErrors, ECleaningUnitDeviceStatus &_eDeviceStatus)
{
    _eStatus = (ECleaningUnitMgrStatus)((_nStatus >> 24) & 0xFF);
    _eErrors = (ECleaningUnitMgrErrors)((_nStatus >> 8) & 0xFF);
    _eDeviceStatus = (ECleaningUnitDeviceStatus)(_nStatus & 0xFF);
}

// ==============================================================
// Helper methods to build/split safety status
inline uint32_t BuildSafetyRequest(ESafetyMgrRequests _eRequest) 
{ 
    return (_eRequest & 0xFFFF);
}
inline void SplitSafetyRequest(uint32_t _nRequest, ESafetyMgrRequests & _eRequest)
{
    _eRequest = (ESafetyMgrRequests)(_nRequest & 0xFFFF);
}

// ==============================================================
// Helper methods to build/split safety status
inline uint32_t BuildSafetyANTOk(bool _bANTOk, uint16_t _nWatchdog) 
{ 
    return (((_bANTOk ? 1 : 0) << 31) | (_nWatchdog & 0xFFFF));
}
inline void SplitSafetyANTOk(uint32_t _nValue, bool &_bANTOk, uint16_t &_nWatchdog)
{
    _bANTOk = ((_nValue >> 31) & 0x01);
    _nWatchdog= (uint16_t)(_nValue & 0xFFFF);
}

// ==============================================================
// Helper methods to build/split safety status
inline uint32_t BuildSafetyStatus(ESafetyMgrStatus _eStatus, uint16_t _nErrors) 
{ 
    return (((_eStatus & 0xFFFF) << 16) | (_nErrors & 0xFFFF));
}
inline void SplitSafetyStatus(uint32_t _nStatus, ESafetyMgrStatus &_eStatus, ESafetyMgrErrors &_eErrors)
{
    _eStatus = (ESafetyMgrStatus)((_nStatus >> 16) & 0xFFFF);
    _eErrors = (ESafetyMgrErrors)(_nStatus & 0xFFFF);
}

// ==============================================================
// Helper methods to build/split CUC version number
inline uint32_t BuildCUCVersion(uint8_t _nMajor, uint8_t _nMinor, uint16_t _nBuild) 
{ 
    return (((_nMajor&0xFF)<<24) | ((_nMinor&0xFF)<<16) | ((_nBuild&0xFFFF)));
}
inline void SplitCUCVersion(uint32_t _nVersion, uint8_t &_nMajor, uint8_t &_nMinor, uint16_t &_nBuild)
{
    _nMajor = ((_nVersion>>24) & 0xFF);
    _nMinor = ((_nVersion>>16) & 0xFF);
    _nBuild = (_nVersion & 0xFFFF);
}

// ==============================================================
// Helper methods to build/split CUC water settings
inline uint16_t BuildCUCWaterSettings(uint8_t _nDensity, uint8_t _nSpeed) 
{ 
    return (((_nDensity & 0xFF) << 8) | ((_nSpeed & 0xFF)));
}
inline void SplitCUCWaterSettings(uint16_t _nSettings, uint8_t &_nDensity, uint8_t &_nSpeed)
{
    _nDensity = ((_nSettings >> 8) & 0xFF);
    _nSpeed = (_nSettings & 0xFF);
}


#endif // _CUC_CANIDS_H_
