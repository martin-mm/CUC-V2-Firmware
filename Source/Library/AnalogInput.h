// ---------------------------------------------------------------------------
//! \package     ARMLibrary
//! \file        AnalogInput.h
//! \brief       Defines a class encapsulating analog inputs
//! 
//! \copyright   Copyright (C) 2011-2012 BlueBotics SA
// ----------------------------------------------------------------------------

#ifndef _ANALOGINPUT_H_
#define _ANALOGINPUT_H_

#include "Base.h"
#include "fsl_adc16.h"
#include "UCDevice.h"
#include "board-Ana.h"

using namespace std;

// ----------------------------------------------------------------------------
// Constants
#define NB_MAX_ANALOG_INPUTS	BOARD_ADC_NumberOfChannels

// ----------------------------------------------------------------------------
// Forward declarations
class AnalogInput;

// ----------------------------------------------------------------------------
//! \class      AnalogInputMgr
//! \brief      Encapsulate the analog to digital converter 
class AnalogInputMgr : UCDevice
{
public:
	AnalogInputMgr(int channel);
    //! \cond HideDefaultMethods
   virtual ~AnalogInputMgr() {}
	HideDefaultMethods(AnalogInputMgr);
    //! \endcond 

public:
	AnalogInput* DeclareInput(uint8_t _nId);
	AnalogInput* GetInput(uint8_t _nId);
	uint16_t Read(uint8_t _nId);

private:
	AnalogInput* m_apInputs[NB_MAX_ANALOG_INPUTS];
};

// ----------------------------------------------------------------------------
//! \class      AnalogInput
//! \brief      Encapsulate a single analog input
class AnalogInput
{
public:
    //! \cond 
	virtual ~AnalogInput() {}
	HideDefaultMethods(AnalogInput);
    //! \endcond 

private:

public:
	AnalogInput(uint8_t _nId);
	const char * GetName(void);
	uint16_t GetOffset(void);
	uint16_t Read(void);

private:
	uint8_t m_nId;

friend class AnalogInputMgr;
};

#endif // _ANALOGINPUT_H_
