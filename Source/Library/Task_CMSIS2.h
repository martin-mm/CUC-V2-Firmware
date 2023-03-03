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
#if defined(__cplusplus)

#include "cmsis_os2.h"
#include "base.h"
#endif
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#define OS_MAX_NUMBER_OF_TASKS	10
#define OS_DEFAULT_STACK_SIZE		1024

#define TASK_NAME_SIZE				17

#if defined(__cplusplus)

using namespace std;

class	CUC_Task;

typedef struct
{
	CUC_Task		*TaskPtr;
	bool			started;
} CUCtask_reg_t;

// ----------------------------------------------------------------------------
//! \class      Task
//! \brief      Encapsulate a task
//! \details    This class is a wrapper around tasks provided by the RTX operating system.
class CUC_Task
{
public:
	CUC_Task(osPriority_t nPriority,const char *name = nullptr);
    //! \cond 
	virtual ~CUC_Task() {}
	HideDefaultMethods(CUC_Task);
    //! \endcond 

public:
	virtual void Main() = 0;            //!< The entry point of the new task. To be implemented by derive class!
	void Start(unsigned StackSize = OS_DEFAULT_STACK_SIZE);
	static void Wait(uint32_t _nTicks);
	static bool TaskSuspend(void *argument);
	static bool TaskResume(void *argument);
	static bool TaskSuspendAll(void);
	static bool TaskResumeAll(void);
	static CUCtask_reg_t * GetTaskPtrByID(int TaskID);
	static int GetNumberOfTasks(void);

	char * GetTaskName(void);
	int GetTaskState(void);
	void SetEvent(uint16_t _nEventId, bool _bFromISR = false);
	void ResetEvent(uint16_t _nEventId);
	bool WaitForEvent(uint16_t _nEventId, uint16_t _nTimeout);
   bool IsTaskRunning(void);
   void SetTaskState(bool isRunning);

private:
	char							TaskName[TASK_NAME_SIZE];
	static CUCtask_reg_t 	m_TaskIdRegister[OS_MAX_NUMBER_OF_TASKS];
	static int					m_nTasks;
	static bool					init;
	static void TaskStarter(void *argument);
	static bool TaskRegister(CUC_Task *ptrTask);

private:
	osThreadId_t 			m_TaskId;           //!< The id of the task
	osPriority_t 			m_nPriority;	     //!< The priority
	osEventFlagsId_t 		m_Event_Id;			 //!< The Event Object used by the task for signaling
	bool						m_TaskRunning;
};

extern "C" bool CUC_Task_Suspend(void *ptr);
extern "C" bool CUC_Task_Resume(void *ptr);
extern "C" bool CUC_Task_SuspendAll(void);
extern "C" bool CUC_Task_ResumeAll(void);
extern "C" bool CUC_Task_SuspendByIndex(int TaskID);
extern "C" bool CUC_Task_ResumeByIndex(int TaskID);
extern "C" int CUC_Task_GetStateByIndex(int TaskID);
extern "C" int CUC_Task_GetNumberOfTasks(void);
extern "C" char * CUC_Task_GetNameByIndex(int TaskID);

#else  /* __cplusplus */

#include <stdint.h>
#include <stdbool.h>

extern bool CUC_Task_Suspend(void *ptr);
extern bool CUC_Task_Resume(void *ptr);
extern bool CUC_Task_SuspendAll(void);
extern bool CUC_Task_ResumeAll(void);
extern bool CUC_Task_SuspendByIndex(int TaskID);
extern bool CUC_Task_ResumeByIndex(int TaskID);
extern int CUC_Task_GetStateByIndex(int TaskID);
extern int CUC_Task_GetNumberOfTasks(void);
extern char * CUC_Task_GetNameByIndex(int TaskID);

#endif /* __cplusplus */

#endif // _TASK_H_
