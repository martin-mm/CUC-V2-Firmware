// ----------------------------------------------------------------------------
// ARM library
//
// Defines the class managing a Battery device on the CAN bus
//
// Copyright (C) 2020 BlueBotics SA
// ----------------------------------------------------------------------------

#ifndef _BATTERYDEVICE_H_
#define _BATTERYDEVICE_H_

#include "Base.h"
#include "CANControlledDevice.h"

// ----------------------------------------------------------------------------
// Class BatteryDevice
//
// Manage a Battery device on the CAN bus
//
class BatteryDevice : public CANControlledDevice
{
public:
  BatteryDevice(uint8_t _nDeviceId, bool _bNMTControl, bool _bConfigurePDOs);
  virtual ~BatteryDevice() {}
  HideDefaultMethods(BatteryDevice);

public:
  uint16_t GetVoltage() { return (10 * m_nVoltage); }  // return value in mV
  uint8_t GetSoc() { return m_nSoc; }
  uint8_t GetStatus() {return m_nStatus; }
  int16_t GetTemperature() { return (m_nTemperature / 10); }  //return value in °C

private:
  virtual bool CANConfigure(int32_t _nHeartbeatPeriod, uint8_t _masterId);
  virtual bool OnCANSync();
  virtual void HandlePDO(CAN_msg &_msg);

private:
  uint16_t m_nVoltage;
  uint8_t m_nSoc;
  uint8_t m_nStatus;
  int16_t m_nTemperature;
};

#endif // _BATTERYDEVICE_H_
