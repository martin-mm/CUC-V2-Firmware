// ---------------------------------------------------------------------------
//! \package     ARMLibrary
//! \file        Watchdog.h
//! \brief       Defines a small wrapper around the CPU watchdog
//! 
//! \copyright   Copyright (C) 2011-2012 BlueBotics SA
// ----------------------------------------------------------------------------

#ifndef _WATCHDOG_H_
#define _WATCHDOG_H_

// ----------------------------------------------------------------------------
// Includes
#if defined(__cplusplus)

#include "Base.h"
#include "UCDevice.h"

// ----------------------------------------------------------------------------
//! \class      Watchdog
//! \brief      Encapsulate the watch dog functionality of the processor
class Watchdog
{
public:
	Watchdog(void);
    //! \cond 
	virtual ~Watchdog(void) {}
	HideCopyAssignCompMethods(Watchdog);
    //! \endcond 

public:
	void Start(uint32_t _nTimeout, bool _bDebug = false);
	void Refresh();

private:
	uint32_t		m_Frequency;
	int			m_Cnt;
};

extern "C" void WDT_IRQHandler();

#else

#include <stdint..h>
#include <stdbool.h>

extern "C" void WDT_IRQHandler();

#endif

#endif // _WATCHDOG_H_
