// ---------------------------------------------------------------------------
//! \package     ARMLibrary
//! \file        ProcessData.h
//! \brief       Defines small classes encapsulating individual process data
//! 
//! \copyright   Copyright (C) 2011-2012 BlueBotics SA
// ----------------------------------------------------------------------------

#ifndef _PROCESSDATA_H_
#define _PROCESSDATA_H_

// ----------------------------------------------------------------------------
// Includes
#include "Base.h"
#include "FreeRTOS.h"
#include "semphr.h"

// ----------------------------------------------------------------------------
//! \class      ProcessDataIn
//! \brief      Smart pointer for input process data
template<class T>
class ProcessDataIn
{
public:
	ProcessDataIn(uint8_t* _pData, uint8_t _nBytes, SemaphoreHandle_t _mutex): m_nBytes(_nBytes), m_pData((T*)_pData), m_Mutex(_mutex) {}
    //! \cond 
	virtual ~ProcessDataIn() {}
	HideDefaultMethods(ProcessDataIn);
    //! \endcond 

public:
 	T Read(bool _bFromISR = false);

private:
	uint8_t 					m_nBytes;
	T* 						m_pData;
	SemaphoreHandle_t 	m_Mutex;
};

// ----------------------------------------------------------------------------
//! \class      ProcessDataOut
//! \brief      Smart pointer for output process data
template<class T>
class ProcessDataOut
{
public:
	ProcessDataOut(uint8_t* _pData, uint8_t _nBytes, SemaphoreHandle_t _mutex): m_nBytes(_nBytes), m_pData((T*)_pData), m_Mutex(_mutex) {}
   //! \cond 
	virtual ~ProcessDataOut() {}
	HideDefaultMethods(ProcessDataOut);
   //! \endcond 

public:
 	T Read(bool _bFromISR = false);
	void Write(T _value, bool _bFromISR = false);

private:
	uint8_t 					m_nBytes;
	T *						m_pData;
	SemaphoreHandle_t 	m_Mutex;
};


#endif // _PROCESSDATA_H_
