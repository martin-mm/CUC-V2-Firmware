// ---------------------------------------------------------------------------
//! \package     ARMLibrary
//! \file        IO.cpp
//! \brief       Defines a set of classes encapsulating Input and Output of the LPC1768 micro controller
//! 
//! \copyright   Copyright (C) 2011-2012 BlueBotics SA
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Includes
#include <stdio.h>
#include "IO.h"
#include "board.h"

// ----------------------------------------------------------------------------
// Static variables
DigitalInput* DigitalInput::m_Inputs[NB_MAX_INPUTS];


// ----------------------------------------------------------------------------
//! \brief Constructor 1
DigitalIOBase::DigitalIOBase(unsigned _nPinId)
{
const strGPIOattrib_t	*ptr;
	
	dbgprintf("Digital Base Constructor, PinID = %d ...\n",_nPinId);	
	if (_nPinId < BOARD_N_GPIO)
	{
		if ((ptr = BOARD_GetPinStructPtr(_nPinId)) != NULL)
		{
			m_PinID = _nPinId;
			m_GPIO_ptr = ptr;
			m_Valid = true;
			dbgprintf("   IO Pin Name: %s\n",GetName());
		}
		else
		{
			m_PinID = 0;
			m_Valid = false;
		}
	}
	else
	{
		m_PinID = 0;
		m_Valid = false;
	}
	dbgprintf("... Digital Base Constructor done.\n");	
}
	 
// ----------------------------------------------------------------------------
//! \brief Gets the name of the IO
const char * DigitalIOBase::GetName(void)
{ 
	return m_GPIO_ptr->Name;
}

// ----------------------------------------------------------------------------
//! \brief Configure the IO pin
bool DigitalIOBase::ConfigPin(EPinMode _eMode, EPinOption _eOption)
{
uint32_t		value;
int			pos;
	
	if (!m_Valid)
		return false;
	value = m_GPIO_ptr->Base->PCR[pos = m_GPIO_ptr->Offset];
	value &= ~(PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_SRE_MASK | PORT_PCR_ODE_MASK | PORT_PCR_DSE_MASK);
	switch (_eMode)
	{
		case EPinMode_NoPull:
			break;
		case EPinMode_PullUp:
			value |= (PORT_PCR_PE_MASK | PORT_PCR_PS_MASK);
			break;
		case EPinMode_PullDown:
			value |= (PORT_PCR_PE_MASK);
			break;
		default:
			return false;
	}
	if ((_eOption & EPinOption_ODE) != 0)
		value |= PORT_PCR_ODE_MASK;
	if ((_eOption & EPinOption_DSE) != 0)
		value |= PORT_PCR_DSE_MASK;
	if ((_eOption & EPinOption_SLEW) != 0)
		value |= PORT_PCR_SRE_MASK;
	m_GPIO_ptr->Base->PCR[pos] = value;
	return true;
}


// ----------------------------------------------------------------------------
//! \brief Constructor
DigitalInput::DigitalInput(uint8_t _nPinId, bool isInverted, EPinMode _eMode,EPinOption _eOption)
	: DigitalIOBase(_nPinId),
	  EventSource(4),
	  m_isInverted(isInverted)
{
	dbgprintf("Digital Input Constructor, PinID = %d ...\n",_nPinId);	
	m_InterruptEdge = EEdge_Rising;
	ConfigPin(_eMode,_eOption);
	RegisterInput(this);
	dbgprintf("... Digital Input Constructor done.\n");
}

// ----------------------------------------------------------------------------
//! \brief Read the input
bool DigitalInput::Read()
{
	if (!m_Valid)
		return false;
	uint8_t value = (m_GPIO_ptr->GPIO->PDIR >> m_GPIO_ptr->Offset) & 0x01U;
	if (m_isInverted)
		value ^= 0x01;
	return (value != 0);
}

// ----------------------------------------------------------------------------
//! \brief Enable the associated interrupt
bool DigitalInput::EnableInterrupt(EEdge _eEdge)
{
	PORT_Type * Base = m_GPIO_ptr->Base;
	unsigned		Offset = m_GPIO_ptr->Offset;

	if (!m_Valid)
		return false;
	if (m_GPIO_ptr->Direction != kGPIO_DigitalInput)
		return false;
	if (Base == nullptr)
		return false;
	switch (_eEdge)
	{
		case EEdge_Rising:
			m_mode = kPORT_InterruptRisingEdge;
			break;
		case EEdge_Falling:
			m_InterruptEdge = EEdge_Falling;
			m_mode = kPORT_InterruptFallingEdge;
			break;
		case EEdge_Both:
			m_InterruptEdge = EEdge_Rising;
			m_mode = kPORT_InterruptEitherEdge;
			break;
		default:
			return false;
	}
   Base->PCR[Offset] = (Base->PCR[Offset] & ~PORT_PCR_IRQC_MASK) | PORT_PCR_IRQC(m_mode);
	m_InterruptEdge = _eEdge;
	return true;
}

// ----------------------------------------------------------------------------
//! \brief Disable the associated interrupt
bool DigitalInput::DisableInterrupt(EEdge _eEdge)
{
	PORT_Type * Base = m_GPIO_ptr->Base;
	unsigned		Offset = m_GPIO_ptr->Offset;

	if (!m_Valid)
		return false;

   Base->PCR[Offset] = (Base->PCR[Offset] & ~PORT_PCR_IRQC_MASK) | PORT_PCR_IRQC(kPORT_InterruptOrDMADisabled);
	return true;
}

// ----------------------------------------------------------------------------
//! \brief Clear the associated interrupt
bool DigitalInput::ClearInterrupt()
{
	m_GPIO_ptr->Base->ISFR = 1 << m_GPIO_ptr->Offset;
	return true;
}

// ----------------------------------------------------------------------------
//! \brief Handle the associated interrupt
void DigitalInput::HandleInterrupt(EEdge _eEdge)
{
	Signal();
	ClearInterrupt();
}

// ----------------------------------------------------------------------------
//! \brief Dispatch input interrupts
bool DigitalInput::DispatchInterrupt(PORT_Type *PortBase,uint8_t PinOffset)
{
	// Loop through registered inputs
	for (int i = 0; i < NB_MAX_INPUTS; i++)
	{
		DigitalInput *pInput = m_Inputs[i];
		if (pInput == NULL)
			return false; 	// Not found... (first slot empty means end of the array)
		// Does this input correspond to the port and pin ids?
		if (pInput->m_GPIO_ptr->Base == PortBase && pInput->m_GPIO_ptr->Offset == PinOffset)
		{
			pInput->HandleInterrupt(pInput->m_InterruptEdge);
			return true;	
		}		
	}
	return false; // Not found...
}

// ----------------------------------------------------------------------------
//! \brief Register an input
bool DigitalInput::RegisterInput(DigitalInput *_pInput)
{
	if (_pInput->m_GPIO_ptr->Direction != kGPIO_DigitalInput)
		return false;
	for (int i = 0; i < BOARD_N_GPIO_IN; i++)
	{
		if (m_Inputs[i] == NULL)
		{
			m_Inputs[i] = _pInput;
			return true;
		}
	}
	return false;
}


// ----------------------------------------------------------------------------
//! \brief Constructor
DigitalOutput::DigitalOutput(uint8_t _nPinId, bool isInverted, EPinOption _PinOption)
	:  DigitalIOBase(_nPinId),
		m_isInverted(isInverted)
{
	dbgprintf("Digital Output Constructor, PinID = %d ...\n",_nPinId);	
	ConfigPin(EPinMode_NoPull,_PinOption);
	dbgprintf("... Digital Output Constructor done.\n");	
}

// ----------------------------------------------------------------------------
//! \brief Set the output
bool DigitalOutput::Set()
{
	if (m_isInverted)
		m_GPIO_ptr->GPIO->PCOR = 1U << m_GPIO_ptr->Offset;
	else
		m_GPIO_ptr->GPIO->PSOR = 1U << m_GPIO_ptr->Offset;
	return true;
}

// ----------------------------------------------------------------------------
//! \brief Reset the output
bool DigitalOutput::Reset()
{
	if (m_isInverted)
		m_GPIO_ptr->GPIO->PSOR = 1U << m_GPIO_ptr->Offset;
	else
		m_GPIO_ptr->GPIO->PCOR = 1U << m_GPIO_ptr->Offset;
	return true;
}

// ----------------------------------------------------------------------------
//! \brief Read the current value of the output
bool DigitalOutput::Read()
{
	if (!m_Valid)
		return false;
	uint8_t value = (m_GPIO_ptr->GPIO->PDIR >> m_GPIO_ptr->Offset) & 0x01U;
	if (m_isInverted)
		value ^= 0x01;
	return (value != 0);
}

// ----------------------------------------------------------------------------
//! \brief Toggle the output
bool DigitalOutput::Toggle(void)
{
	m_GPIO_ptr->GPIO->PTOR = 1U << m_GPIO_ptr->Offset;
	return true;
}

// ----------------------------------------------------------------------------
//! \brief The interrupt handler for IOs
extern "C" void PORT_IRQHandler(PORT_Type *port,unsigned IRQmask)
{
	// Check what input is the source of the interrupt
	for (int nPinId = 0; nPinId < 32; nPinId++)
	{
		if ((IRQmask & (1 << nPinId)) != 0)
			DigitalInput::DispatchInterrupt(port,nPinId);
	}
}
