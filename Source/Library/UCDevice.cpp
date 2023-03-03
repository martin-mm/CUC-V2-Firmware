// ---------------------------------------------------------------------------
//! \package     ARMLibrary
//! \file        UCDevice.cpp
//! \brief       Defines a class encapsulating a device of the micro controller 
//! 
//! \copyright   Copyright (C) 2011-2012 BlueBotics SA
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Includes
#include <stdio.h>
#include "UCDevice.h"
#include "fsl_clock.h"
#include "board.h"

// ----------------------------------------------------------------------------
// Static member variables
uint32_t 		UCDevice::m_nUCFreq = CLOCK_GetCoreSysClkFreq();
const char *	UCDevice::DeviceName[30] =
{
	"None",				// Device 1
	"None",				// Device 2
	"None",				// Device 3
	"None",				// Device 4
	"None",				// Device 5
	"None",				// Device 6
	"None",				// Device 7
	"None",				// Device 8
	"None",				// Device 9
	"None",				// Device 10
	"None",				// Device 11
	"None",				// Device 12
	"None",				// Device 13
	"None",				// Device 14
	"None",				// Device 15
	"None",				// Device 16
	"None",				// Device 17
	"None",				// Device 18
	"None",				// Device 19
	"None",				// Device 20
	"None",				// Device 21
	"None",				// Device 22
	"None",				// Device 23
	"None",				// Device 24
	"None",				// Device 25
	"None",				// Device 26
	"None",				// Device 27
	"None",				// Device 28
	"None",				// Device 29
	"None",				// Device 30
};

// ----------------------------------------------------------------------------
//! \brief Constructor
UCDevice::UCDevice(EDevice_t _eDevice, uint32_t ClockFrequency)
{
	m_eDevice = _eDevice;
	if (ClockFrequency == 0)
		m_DeviceFreqHz = CLOCK_GetFreq(kCLOCK_BusClk);
	else
		m_DeviceFreqHz = ClockFrequency;
	dbgprintf("UC Device Constructor, Device = %d, Clock Frequency = %d Hz ...\n",(int)_eDevice,m_DeviceFreqHz);	
	dbgprintf("... UC Device Constructor done.\n");	
}

// ----------------------------------------------------------------------------
//! \brief Define the frequency of the processor
void UCDevice::SetUCFreq(uint32_t _nUCFreq)
{
// CPU frequency is fixed
//	m_nUCFreq = _nUCFreq;
}

// ----------------------------------------------------------------------------
//! \brief Return the device frequency
uint32_t UCDevice::GetDeviceFreq(void)
{
	return m_DeviceFreqHz;
}

// ----------------------------------------------------------------------------
//! \brief Return the number of device periods (number of cycles of the associated clock) for a given duration
uint32_t UCDevice::GetPeriodsFromDuration(uint32_t _nDuration)
{	
	return (_nDuration * m_DeviceFreqHz) / (1000 * 1000);
}

// ----------------------------------------------------------------------------
//! \brief Configure the device (Power up the device and set its clock divisor)
bool UCDevice::Configure(void)
{
	uint32_t nDeviceId = (uint32_t)m_eDevice;
	// do we need PowerUp here
	return true;
}
