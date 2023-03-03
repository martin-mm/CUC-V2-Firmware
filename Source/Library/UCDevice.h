// ---------------------------------------------------------------------------
//! \package     ARMLibrary
//! \file        UCDevice.h
//! \brief       Defines a class encapsulating a device of the micro controller 
//! 
//! \copyright   Copyright (C) 2011-2012 BlueBotics SA
// ----------------------------------------------------------------------------

#ifndef _UCDEVICE_H_
#define _UCDEVICE_H_

// ----------------------------------------------------------------------------
// Includes
#include "Base.h"

// ----------------------------------------------------------------------------
//! \brief Enumeration of clock divisors
// typedef enum {
// EClkDiv_4 = 0,
// EClkDiv_1 = 1,
// EClkDiv_2 = 2,
// EClkDiv_8 = 3
// } EClkDiv;

// ----------------------------------------------------------------------------
//! \brief Enumeration of available devices
typedef enum
{
	EDevice_Timer0 = 1,
	EDevice_Timer1 = 2,
	EDevice_CAN1 = 13,
	EDevice_UART0 = 3,
	EDevice_UART1 = 4,
	EDevice_UART2 = 24,
	EDevice_UART3 = 25,
	EDevice_PWM = 6,
	EDevice_DAC = 11,
	EDevice_ADC = 30,
	EDevice_I2C0 = 7,
	EDevice_I2C1 = 19,
	EDevice_I2C2 = 26,
} EDevice_t;

// ----------------------------------------------------------------------------
//! \class      UCDevice
//! \brief      Base class for all devices
//! \details    Implement generic functionality of a processor device: configuration of power and clock.
class UCDevice
{
public:
	UCDevice(EDevice_t _eDevice, uint32_t _nFrequency = 0);
    //! \cond 
	virtual ~UCDevice() {}
	HideDefaultMethods(UCDevice);
    //! \endcond 

public:
	static void SetUCFreq(uint32_t _nUCFreq);
	virtual uint32_t GetDeviceFreq(void);	// [Hz]
	uint32_t GetPeriodsFromDuration(uint32_t _nDuration /*us*/);

public: 
	bool Configure(void);
	EDevice_t GetDevice(void) { return m_eDevice; }

protected:
	EDevice_t m_eDevice;
	uint32_t m_DeviceFreqHz;

private:
	static uint32_t m_nUCFreq;
	static const char *	DeviceName[30];
};

#endif // _UCDEVICE_H_
