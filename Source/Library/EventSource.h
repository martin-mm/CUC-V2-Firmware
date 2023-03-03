// ---------------------------------------------------------------------------
//! \package     ARMLibrary
//! \file        EventSource.h
//! \brief       Defines a base class for classes that are producing events
//! 
//! \copyright   Copyright (C) 2011-2012 BlueBotics SA
// ----------------------------------------------------------------------------

#ifndef _EVENTSOURCE_H_
#define _EVENTSOURCE_H_

#include "Base.h"

// Forward declarations
class EventSource;

// ----------------------------------------------------------------------------
//! \class      IEventHandler
//! \brief      Interface that all classes that need to handle events must implement.
//! \details    Implementing this interface allows a class to receive notifications 
//!             other classes deriving from EventSource.
class IEventHandler
{
public:
    //! \cond
	virtual ~IEventHandler() {}
    //! \endcond
	virtual void HandleEvent(EventSource *_pSource, uint32_t _nEventId, uint32_t _nData) = 0;
};

// ----------------------------------------------------------------------------
//! \struct     TEventHandlerInfo
//! \brief      Structure used internally by EventSource
typedef struct 
{
	IEventHandler *pHandler;	//!< A pointer on the handler
	uint32_t nEventId;			//!< An id defined and understood by the handler itself
} EventHandlerInfo_t;

// ----------------------------------------------------------------------------
//! \struct     EventSource
//! \brief      Base class for all classes producing events
//! \details    Class that need to handle event must implement the IEventHandler interface.
class EventSource
{
public:
	EventSource(uint8_t _nMaxHandlers);
    //! \cond 
	virtual ~EventSource(void) {}
	HideDefaultMethods(EventSource);
    //! \endcond 

public:
	bool RegisterHandler(IEventHandler *_pHandler, uint32_t _nEventId);

protected:
	void Signal(uint32_t _nData = 0);

private:
	uint8_t 					m_nMaxHandlers;               //!< Maximum number of supported handlers
	EventHandlerInfo_t 	*m_apHandlers;   //!< The array of registered handlers
};

#endif // _EVENTSOURCE_H_
