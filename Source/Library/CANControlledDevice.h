// ----------------------------------------------------------------------------
// ARM Library
//
// Defines the class encapsulating a controlled remote CAN device
//
// Copyright (C) 2011-2012 BlueBotics SA
// ----------------------------------------------------------------------------

#ifndef _CANCONTROLLEDDEVICE_H_
#define _CANCONTROLLEDDEVICE_H_

// ----------------------------------------------------------------------------
// Includes
#include "Base.h"
#include "CANDefs.h"
#include "CANDriver.h"
#include "Timer.h"


// CAN PDO structure
typedef struct {
  int16_t m_nPeriod;
  uint32_t m_nLastTime;
  bool m_bTimeout;
} CAN_PDO;

// ----------------------------------------------------------------------------
// Class CANControlledDevice
//
// Encapsulating a controlled remote CAN device
//
class CANControlledDevice
{
public:
	CANControlledDevice(uint8_t _nDeviceId, bool _bNMTControl, bool _bConfigurePDOs);
	virtual ~CANControlledDevice() {}
	HideDefaultMethods(CANControlledDevice);

public:
	uint8_t GetDeviceId() { return m_nDeviceId; }
	uint8_t GetNMTCounter() { return m_nNMTCounter; }
	bool IsTimedOut();

protected:
	bool UploadSDO(uint16_t _objIndex, uint8_t _subIndex, uint32_t& _data, bool _waitForAnswer);
	bool DownloadSDO(uint16_t _objIndex, uint8_t _subIndex, uint32_t _data, uint8_t _nDataLen, bool _waitForAnswer);
	void SentNMT(ENMTCommand _cmd);

	virtual bool CANInit(CANDriver &_driver);
	virtual bool CANConfigure(int32_t _nHeartbeatPeriod, uint8_t _masterId);
	virtual bool OnCANSync();
	virtual void HandleSDOAnswer(CAN_msg &_msg);
	virtual void HandlePDO(CAN_msg &_msg);
	virtual void SendRxPDO(uint32_t _nNow);

private:
	void HandleNMTStateMachine(CAN_msg &_msg);

protected:
	CANDriver 				*m_pDriver;
	const uint8_t 			m_nDeviceId;
	uint8_t 					m_nNMTState;
	uint8_t 					m_nNMTCounter;
	const bool 				m_bNMTControl;
	const bool 				m_bConfigurePdo;
	bool 						m_bDeviceConfigured;
	bool 						m_bHeartbeatTimeout;
	uint64_t 				m_nHeartbeatLastTime;
	CAN_PDO 					m_aPdo[8];

private:
	CAN_msg 					m_sdoAnswer;
	bool 						m_bWaitSdo;

friend class CANMaster;
};

#endif // _CANCONTROLLEDDEVICE_H_
