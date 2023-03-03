// ---------------------------------------------------------------------------
//! \package     ARMLibrary
//! \file        Task.h
//! \brief       Defines the class encapsulating an RTX task
//! 
//! \copyright   Copyright (C) 2011-2012 BlueBotics SA
// ----------------------------------------------------------------------------

#ifndef _TASK_H_
#define _TASK_H_

// ----------------------------------------------------------------------------
// Includes
#include "base.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

// ----------------------------------------------------------------------------
//! \class      Task
//! \brief      Encapsulate a task
//! \details    This class is a wrapper around tasks provided by the RTX operating system.
class Task
{
public:
	Task(UBaseType_t nPriority);
    //! \cond 
	virtual ~Task() {}
	HideDefaultMethods(Task);
    //! \endcond 

public:
	virtual void Main() = 0;            //!< The entry point of the new task. To be implemented by derive class!
	void Start();
	static void Wait(uint32_t _nTicks);

	void SetEvent(uint16_t _nEventId, bool _bFromISR = false);
	void ResetEvent(uint16_t _nEventId);
	bool WaitForEvent(uint16_t _nEventId, uint16_t _nTimeout);

private:
	static void TaskStarter(void *argument);

private:
	BaseType_t  			m_nTaskId;         //!< The id of the task
	TaskHandle_t 			pxCreatedTask;
	UBaseType_t  			m_nPriority;	    //!< The priority
//	osEventFlagsId_t 		m_Event_Id;			 //!< The Event Object used by the task for signaling
};

#endif // _TASK_H_
