// ---------------------------------------------------------------------------
//! \package     ARMLibrary
//! \file        AnalogInput.cpp
//! \brief       Defines a class encapsulating analog inputs
//! 
//! \copyright   Copyright (C) 2011-2012 BlueBotics SA
// ----------------------------------------------------------------------------

#include <stdio.h>
#include "AnalogInput.h"
#include "board.h"


// ----------------------------------------------------------------------------
//! \brief Constructor
AnalogInputMgr::AnalogInputMgr(int channel)
	: UCDevice(EDevice_ADC,0)
{
	dbgprintf("Analog Input Manager Constructor channel %d ...\n",channel);	
	Configure();
	SetUCFreq(BOARD_get_ADC_clock(ADC0));
	for (int i = 0;i < NB_MAX_ANALOG_INPUTS;i++)
		m_apInputs[i] = nullptr;
   dbgprintf("... Analog Input Manager Constructor done.\n");	
}

	const char * GetName(void);

// ----------------------------------------------------------------------------
//! \brief Create the specified analog input object
AnalogInput* AnalogInputMgr::DeclareInput(uint8_t _nId)
{
	dbgprintf("   Analog Input Manager - Declare Input %d\n",_nId);	
	AnalogInput *pInput = GetInput(_nId);
	if (pInput == NULL && _nId < NB_MAX_ANALOG_INPUTS)
	{		
		pInput = new AnalogInput(_nId);
		m_apInputs[_nId] = pInput;
	}
	return pInput;
}

// ----------------------------------------------------------------------------
//! \brief Return the specified analog input object
AnalogInput* AnalogInputMgr::GetInput(uint8_t _nId)
{
	AnalogInput *pInput = NULL;
	if (_nId < NB_MAX_ANALOG_INPUTS)
	{		
		pInput = m_apInputs[_nId];
	}
	return pInput;
}

// ----------------------------------------------------------------------------
//! \brief Return the result of the last A/D conversion of the specified analog input
uint16_t AnalogInputMgr::Read(uint8_t _nId)
{
	AnalogInput *pInput = GetInput(_nId);
	if (pInput != NULL)
	{
		return pInput->Read();
	}
	return 0;
}


// ----------------------------------------------------------------------------
//! \brief Analog input constructor
AnalogInput::AnalogInput(uint8_t _nId)
	: m_nId(0)
{
	dbgprintf("Analog Input Constructor, ID = %d ...\n",_nId);	
	if (_nId < NB_MAX_ANALOG_INPUTS)
	{
		m_nId = _nId;
		const char *strPtr = GetName();
		if (strPtr != nullptr)
			dbgprintf("   Analog Input Name: %s\n",strPtr);	
	}
	dbgprintf("... Analog Input Constructor done\n");	
}

// ----------------------------------------------------------------------------
//! \brief Gets the name of the analog input
const char * AnalogInput::GetName(void)
{ 
	return BOARD_getADCchannelName(m_nId);
}

// ----------------------------------------------------------------------------
//! \brief Read the (calibrated) offset value of the A/D channel
uint16_t AnalogInput::GetOffset(void)
{
	return BOARD_getOffsetCalibValue(m_nId);
}

// ----------------------------------------------------------------------------
//! \brief Read the value of the last A/D conversion
uint16_t AnalogInput::Read(void)
{
uint16_t 	value;
	
	if (BOARD_get_ADC(m_nId,&value))
		return value;
	else
		return 0;
}
