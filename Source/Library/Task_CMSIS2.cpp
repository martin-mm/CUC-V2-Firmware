// ---------------------------------------------------------------------------
//! \package     ARMLibrary
//! \file        Task.cpp
//! \brief       Defines the class encapsulating an RTX task
//! 
//! \copyright   Copyright (C) 2011-2012 BlueBotics SA
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Includes
#include <stdlib.h>
#include "Task_CMSIS2.h"
#include "board.h"

bool 				CUC_Task::init = true;
CUCtask_reg_t 	CUC_Task::m_TaskIdRegister[OS_MAX_NUMBER_OF_TASKS];
int				CUC_Task::m_nTasks;

// ----------------------------------------------------------------------------
//! \brief Constructor
CUC_Task::CUC_Task(osPriority_t nPriority,const char *name)
{
	dbgprintf("<CUC-Task Constructor, Priority = %d ...>\n",(int)nPriority);	
	if (init)		// do this only once
	{
		for (int i = 0;i < OS_MAX_NUMBER_OF_TASKS;i++)
		{
			m_TaskIdRegister[i].TaskPtr = nullptr;
			m_TaskIdRegister[i].started = false;
			m_nTasks = 0;
		}
		init = false;
	}
	m_TaskId = 0;
	m_nPriority = nPriority;
	m_Event_Id = nullptr;
	m_TaskRunning = false;
	if (name != nullptr)
	{
		strncpy(TaskName,name,TASK_NAME_SIZE-1);
		TaskName[TASK_NAME_SIZE-1] = '\0';
		dbgprintf("   CUC-Task Name = %s\n",TaskName);
	}
	else
		TaskName[0] = '\0';
	dbgprintf("<... CUC-Task Constructor done.>\n");	
}

// ----------------------------------------------------------------------------
//! \brief Create and start the task
void CUC_Task::Start(unsigned StackSize)
{
	osThreadAttr_t 	attrib;

	m_Event_Id = osEventFlagsNew(NULL);
	// TODO Handle Event Creation Error
	if (TaskName[0] != '\0')
		attrib.name = TaskName;
	else
		attrib.name = nullptr;
	attrib.attr_bits = osThreadDetached;
	attrib.cb_mem = NULL;
	attrib.cb_size = 0;
	attrib.stack_mem = NULL;
	attrib.stack_size = StackSize;
	attrib.priority = m_nPriority;
	attrib.tz_module = 0;
	attrib.reserved = 0;
	// we first have to register the task's handler since osThreadNew 
	// calls it directly after creation
	if (TaskRegister(this))
	{
		dbgprintf("<Task %s registered>\n",TaskName);
		m_TaskId = osThreadNew(CUC_Task::TaskStarter, (void *)this, &attrib);
		dbgprintf("<Task %s started>\n",TaskName);
	}
//	m_TaskId = osThreadNew(CUC_Task::TaskStarter, (void *)this, nullptr);
}

// ----------------------------------------------------------------------------
//! \brief Checks if the task is running or is suspended
bool CUC_Task::IsTaskRunning(void)
{
	return m_TaskRunning;
}

// ----------------------------------------------------------------------------
//! \brief Sets the task state to is running or is suspended
void CUC_Task::SetTaskState(bool isRunning)
{
	m_TaskRunning = isRunning;
}

// ----------------------------------------------------------------------------
//! \brief Wait the specified amount of system ticks
void CUC_Task::Wait(uint32_t _nTicks)
{
	osDelay(_nTicks);
}

// ----------------------------------------------------------------------------
//! \brief Set the specified event
void CUC_Task::SetEvent(uint16_t nEventId, bool _bFromISR)
{
	if (_bFromISR)
	{
//		isr_evt_set(_nEventId, m_TaskId);
		osEventFlagsSet(m_Event_Id, nEventId);
	}
	else
	{
//		os_evt_set(_nEventId, m_TaskId);
		osEventFlagsSet(m_Event_Id, nEventId);
	}
}

// ----------------------------------------------------------------------------
//! \brief Reset the specified event
void CUC_Task::ResetEvent(uint16_t _nEventId)
{
//	os_evt_clr(_nEventId, m_TaskId);
	osEventFlagsClear(m_Event_Id,_nEventId);
}

// ----------------------------------------------------------------------------
//! \brief Wait for the specified event to occur osFlagsErrorUnknown
bool CUC_Task::WaitForEvent(uint16_t _nEventId, uint16_t _nTimeout)
{
//	OS_RESULT result = os_evt_wait_and(_nEventId, _nTimeout);
	uint32_t result = osEventFlagsWait(m_Event_Id, _nEventId, osFlagsWaitAll, _nTimeout);
	return ((result & osFlagsError) == 0);
}

// ----------------------------------------------------------------------------
//! \brief "C" basic code used to start a new task
void CUC_Task::TaskStarter(void *argument)
{
	CUC_Task	*ptr = (CUC_Task *)argument;
	if (ptr != nullptr)
	{
		dbgprintf("<Task %s start Task>\n",ptr->TaskName);
		for (int i = 0;i < OS_MAX_NUMBER_OF_TASKS;i++)
			if (m_TaskIdRegister[i].TaskPtr == ptr)
			{
				ptr->SetTaskState(true);
				ptr->Main();
			}
	}
}

// ----------------------------------------------------------------------------
//! \brief "C" basic code used to suspend an existing task
bool CUC_Task::TaskSuspend(void *argument)
{
CUC_Task	*ptr = (CUC_Task *)argument;
	
	if (ptr != nullptr)
	{
		dbgprintf("<Task %s - stop Task>\n",ptr->TaskName);
		for (int i = 0;i < OS_MAX_NUMBER_OF_TASKS;i++)
		{
			if (m_TaskIdRegister[i].TaskPtr == ptr)
			{
				if (ptr->IsTaskRunning())
				{
					osStatus_t ret = osThreadSuspend(ptr->m_TaskId);
					if (ret == osOK)
					{
						ptr->SetTaskState(false);
					}
					else
						return false;
				}
				return true;
			}
		}
	}
	return false;
}

// ----------------------------------------------------------------------------
//! \brief "C" basic code used to resume an existing task
bool CUC_Task::TaskResume(void *argument)
{
CUC_Task	*ptr = (CUC_Task *)argument;
	
	if (ptr != nullptr)
	{
		dbgprintf("<Task %s - stop Task>\n",ptr->TaskName);
		for (int i = 0;i < OS_MAX_NUMBER_OF_TASKS;i++)
		{
			if (m_TaskIdRegister[i].TaskPtr == ptr)
			{
				if (!ptr->IsTaskRunning())
				{
					osStatus_t ret = osThreadResume(ptr->m_TaskId);
					if (ret == osOK)
					{
						ptr->SetTaskState(true);
					}
					else
						return false;
				}
				return true;
			}
		}
	}
	return false;
}

// ----------------------------------------------------------------------------
//! \brief "C" basic code used to stop all existing task
bool CUC_Task::TaskSuspendAll(void)
{
CUC_Task	*ptr;
int		err = 0;
	
	for (int i = 0;i < OS_MAX_NUMBER_OF_TASKS;i++)
	{
		if ((ptr = m_TaskIdRegister[i].TaskPtr) == nullptr)
			return true;
		if (ptr->IsTaskRunning())
		{
			osStatus_t ret = osThreadSuspend(ptr->m_TaskId);
			if (ret == osOK)
				ptr->SetTaskState(false);
			else
				err++;
		}
	}
	return err == 0;
}

// ----------------------------------------------------------------------------
//! \brief "C" basic code used to stop all existing task
bool CUC_Task::TaskResumeAll(void)
{
CUC_Task	*ptr;
int		err = 0;

	for (int i = 0;i < OS_MAX_NUMBER_OF_TASKS;i++)
	{
		if ((ptr = m_TaskIdRegister[i].TaskPtr) == nullptr)
			return true;
		if (!ptr->IsTaskRunning())
		{
			osStatus_t ret = osThreadResume(m_TaskIdRegister[i].TaskPtr->m_TaskId);
			if (ret == osOK)
				ptr->SetTaskState(true);
			else
				err++;
		}
	}
	return err == 0;
}

// ----------------------------------------------------------------------------
//! \brief "C" basic code used to start a new task
bool CUC_Task::TaskRegister(CUC_Task *ptrTask)
{
	for (int i = 0;i < OS_MAX_NUMBER_OF_TASKS;i++)
	{
		if (m_TaskIdRegister[i].TaskPtr == nullptr)
		{
			m_TaskIdRegister[i].TaskPtr = ptrTask;
			m_TaskIdRegister[i].started = false;
			m_nTasks = i + 1;
			return true;		// task successcully registers
		}
		else
			if (m_TaskIdRegister[i].TaskPtr == ptrTask)
				return true;	// already registered
	}
	return false;				// no more space available
}

// ----------------------------------------------------------------------------
//! \brief "C" basic code used to get a Point to a Task Entry
CUCtask_reg_t * CUC_Task::GetTaskPtrByID(int TaskID)
{
	if (TaskID >= OS_MAX_NUMBER_OF_TASKS)
		return nullptr;
	return &(m_TaskIdRegister[TaskID]);
}

// ----------------------------------------------------------------------------
//! \brief "C" basic code used to get the Task Name
char * CUC_Task::GetTaskName(void)
{
	return TaskName;
}

// ----------------------------------------------------------------------------
//! \brief "C" basic code used to get the Task Name
int CUC_Task::GetNumberOfTasks(void)
{
	return m_nTasks;
}

// ----------------------------------------------------------------------------
//! \brief "C" basic code used to get the Task Name
int CUC_Task::GetTaskState(void)
{
osThreadState_t	state;
int					ret;
	
	state = osThreadGetState(osThreadGetId());
	switch (state)
	{
		case osThreadInactive:
			ret = 0;
			break;
		case osThreadReady :
			ret = 1;
			break;
		case osThreadRunning :
			ret = 2;
			break;
		case osThreadBlocked :
			ret = 3;
			break;
		case osThreadTerminated :
			ret = 4;
			break;
		case osThreadError :
			ret = 5;
			break;
		case osThreadReserved :
			ret = 6;
			break;
		default:
			return -1;
	}
	if (IsTaskRunning())
		ret |= (1 << 8);
	return ret;
}

// ----------------------------------------------------------------------------
// ----------------------------------------------------------------------------

extern "C" bool CUC_Task_Suspend(void *ptr)
{
	CUC_Task	*ptrTask = (CUC_Task *)ptr;
	return CUC_Task::TaskSuspend(ptrTask);
}

extern "C" bool CUC_Task_Resume(void *ptr)
{
	CUC_Task	*ptrTask = (CUC_Task *)ptr;
	return CUC_Task::TaskResume(ptrTask);
}

extern "C" bool CUC_Task_SuspendByIndex(int TaskID)
{
	CUCtask_reg_t * ptr = CUC_Task::GetTaskPtrByID(TaskID);
	if (ptr == nullptr)
		return false;
	if (ptr->TaskPtr == nullptr)
		return false;
	return CUC_Task::TaskSuspend(ptr->TaskPtr);
}

extern "C" bool CUC_Task_ResumeByIndex(int TaskID)
{
	CUCtask_reg_t * ptr = CUC_Task::GetTaskPtrByID(TaskID);
	if (ptr == nullptr)
		return false;
	if (ptr->TaskPtr == nullptr)
		return false;
	return CUC_Task::TaskResume(ptr->TaskPtr);
}

extern "C" char * CUC_Task_GetNameByIndex(int TaskID)
{
	CUCtask_reg_t * ptr = CUC_Task::GetTaskPtrByID(TaskID);
	if (ptr == nullptr)
		return nullptr;
	if (ptr->TaskPtr == nullptr)
		return nullptr;
	return ptr->TaskPtr->GetTaskName();
}

extern "C" int CUC_Task_GetStateByIndex(int TaskID)
{
	CUCtask_reg_t * ptr = CUC_Task::GetTaskPtrByID(TaskID);
	if (ptr == nullptr)
		return -1;
	if (ptr->TaskPtr == nullptr)
		return -1;
	return ptr->TaskPtr->GetTaskState();
}

extern "C" int CUC_Task_GetNumberOfTasks(void)
{
	return CUC_Task::GetNumberOfTasks();
}

extern "C" bool CUC_Task_SuspendAll(void)
{
	return CUC_Task::TaskSuspendAll();
}

extern "C" bool CUC_Task_ResumeAll(void)
{
	return CUC_Task::TaskResumeAll();
}
