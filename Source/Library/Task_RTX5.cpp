// ---------------------------------------------------------------------------
//! \package     ARMLibrary
//! \file        Task.cpp
//! \brief       Defines the class encapsulating an RTX task
//! 
//! \copyright   Copyright (C) 2011-2012 BlueBotics SA
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Includes
#include "Task.h"
// ----------------------------------------------------------------------------
//! \brief Constructor
Task::Task(osPriority_t nPriority)
{
	m_nTaskId = 0;
	m_nPriority = nPriority;
	m_Event_Id = nullptr;
}

// ----------------------------------------------------------------------------
//! \brief Create and start the task
void Task::Start()
{
//	m_nTaskId = os_tsk_create(Task::TaskStarter, m_nPriority, (void*)this);
	osThreadAttr_t 	attrib;

	m_Event_Id = osEventFlagsNew(NULL);
	// TODO Handle Event Creation Error
	attrib.name = NULL;
	attrib.attr_bits = osThreadDetached;
	attrib.cb_mem = NULL;
	attrib.cb_size = 0;
	attrib.stack_mem = NULL;
	attrib.stack_size = 0;
	attrib.priority = m_nPriority;
	attrib.tz_module = 0;
	attrib.reserved = 0;
	m_nTaskId = osThreadNew(Task::TaskStarter, (void *)this, &attrib);
}

// ----------------------------------------------------------------------------
//! \brief Wait the specified amount of system ticks
void Task::Wait(uint32 _nTicks)
{
	osDelay(_nTicks);
}

// ----------------------------------------------------------------------------
//! \brief Set the specified event
void Task::SetEvent(uint16_t nEventId, bool _bFromISR)
{
	if (_bFromISR)
	{
//		isr_evt_set(_nEventId, m_nTaskId);
		osEventFlagsSet(m_Event_Id, nEventId);
	}
	else
	{
//		os_evt_set(_nEventId, m_nTaskId);
		osEventFlagsSet(m_Event_Id, nEventId);
	}
}

// ----------------------------------------------------------------------------
//! \brief Reset the specified event
void Task::ResetEvent(uint16 _nEventId)
{
//	os_evt_clr(_nEventId, m_nTaskId);
	osEventFlagsClear(m_Event_Id,_nEventId);
}

// ----------------------------------------------------------------------------
//! \brief Wait for the specified event to occur osFlagsErrorUnknown
bool Task::WaitForEvent(uint16 _nEventId, uint16 _nTimeout)
{
//	OS_RESULT result = os_evt_wait_and(_nEventId, _nTimeout);
	uint32_t result = osEventFlagsWait(m_Event_Id, _nEventId, osFlagsWaitAll, _nTimeout);
	return ((result & osFlagsError) == 0);
}

// ----------------------------------------------------------------------------
//! \brief "C" basic code used to start a new task
void Task::TaskStarter(void *argument)
{
	Task	*ptr = (Task *)argument;
	if (ptr != nullptr)
		ptr->Main();
}
