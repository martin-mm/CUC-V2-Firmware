// ---------------------------------------------------------------------------
//! \package     ARMLibrary
//! \file        EventSource.cpp
//! \brief       Defines a base class for classes that are producing events
//! 
//! \copyright   Copyright (C) 2011-2012 BlueBotics SA
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Includes
#include <stdio.h>
#include "EventSource.h"
#include "board.h"

// ----------------------------------------------------------------------------
//! \brief Constructor
EventSource::EventSource(uint8_t _nMaxHandlers)
{	
	dbgprintf("Event Source Constructor, Max. Handlers = %d ...\n",_nMaxHandlers);	
	m_nMaxHandlers = _nMaxHandlers;

	// Create the array of handlers and make sure it is initialized
	m_apHandlers = new EventHandlerInfo_t[_nMaxHandlers];
	for (int i=0; i<m_nMaxHandlers; i++)
	{
		m_apHandlers[i].pHandler = nullptr;		
	}
	dbgprintf("... Event Source Constructor done.\n");	
}

// ----------------------------------------------------------------------------
//! \brief Register an event handler
bool EventSource::RegisterHandler(IEventHandler *_pHandler, uint32_t _nEventId)
{
	if (_pHandler == nullptr) 
	{
		return false;
	}

	// Store the handler
	for (int i=0; i<m_nMaxHandlers; i++)
	{
		if (m_apHandlers[i].pHandler == nullptr)
		{			
			m_apHandlers[i].pHandler = _pHandler;
			m_apHandlers[i].nEventId = _nEventId;
			dbgprintf("+Event Source - Register Event, ID = %d ...\n",_nEventId);	
			return true;
		}
	}
	return false;	// No free slots!
}

// ----------------------------------------------------------------------------
//! \brief Signal all handlers that the event occured
void EventSource::Signal(uint32_t _nData)
{
	// Store the handler
	for (int i=0; i<m_nMaxHandlers; i++)
	{
		if (nullptr != m_apHandlers[i].pHandler)
		{
			m_apHandlers[i].pHandler->HandleEvent(this, m_apHandlers[i].nEventId, _nData);
		}
	}
}
