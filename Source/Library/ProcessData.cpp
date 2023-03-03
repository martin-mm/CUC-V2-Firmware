// ---------------------------------------------------------------------------
//! \package     ARMLibrary
//! \file        ProcessData.cpp
//! \brief       Defines small classes encapsulating individual process data
//! 
//! \copyright   Copyright (C) 2011-2012 BlueBotics SA
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Includes
#include "ProcessData.h"
#include "cmsis_os2.h"

// ----------------------------------------------------------------------------
//! \brief Read data
template <class T> T ProcessDataIn<T>::Read(bool _bFromISR)          		
{ 
	if (!_bFromISR)
		osMutexAcquire(m_Mutex, 0xffff);
	T value = *m_pData;
	if (!_bFromISR)
		osMutexRelease(m_Mutex);
	return value; 
}

// ----------------------------------------------------------------------------
//! \brief Read data
template <class T> T ProcessDataOut<T>::Read(bool _bFromISR)          		
{ 
	if (!_bFromISR)
		osMutexAcquire(m_Mutex, 0xffff);
	T value = *m_pData;
	if (!_bFromISR)
		osMutexRelease(m_Mutex);
	return value; 
}

// ----------------------------------------------------------------------------
//! \brief Write data
template <class T> void ProcessDataOut<T>::Write(T _value, bool _bFromISR)
{ 
	if (!_bFromISR)
		osMutexAcquire(m_Mutex, 0xffff);
	*m_pData = _value; 
	if (!_bFromISR)
		osMutexRelease(m_Mutex);
}	

// ----------------------------------------------------------------------------
//! \brief Specialization of the Read method for unsigned 32 bits data
template <> uint32_t ProcessDataIn<uint32_t>::Read(bool _bFromISR)
{	
	uint32_t nMask = (1 << (m_nBytes*8)) -1;

	if (!_bFromISR)
		osMutexAcquire(m_Mutex, 0xffff);
	uint32_t value = *m_pData & nMask;
	if (!_bFromISR)
		osMutexRelease(m_Mutex);

	return value;
}

// ----------------------------------------------------------------------------
//! \brief Specialization of the Read method for signed 32 bits data
template <> int32_t ProcessDataIn<int32_t>::Read(bool _bFromISR)
{	
	uint32_t nMask = (1 << (m_nBytes*8)) -1;

	if (!_bFromISR)
		osMutexAcquire(m_Mutex, 0xffff);
	uint32_t value = *m_pData & nMask;
	if (!_bFromISR)
		osMutexRelease(m_Mutex);

	// Handle the sign bit
	uint32_t nMaskNeg = (1 << ((m_nBytes*8)-1));
	if (value & nMaskNeg)
		value |= ~nMask;

	return value;
}

// ----------------------------------------------------------------------------
//! \brief Specialization of the Read method for unsigned 32 bits data
template <> uint32_t ProcessDataOut<uint32_t>::Read(bool _bFromISR)
{	
	uint32_t nMask = (1 << (m_nBytes*8)) -1;

	if (!_bFromISR)
		osMutexAcquire(m_Mutex, 0xffff);
	uint32_t value = *m_pData & nMask;
	if (!_bFromISR)
		osMutexRelease(m_Mutex);

	return value;
}

// ----------------------------------------------------------------------------
//! \brief Specialization of the Write method for unsigned 32 bits data
template <> void ProcessDataOut<uint32_t>::Write(uint32_t _value, bool _bFromISR)
{	
	uint32_t nMask = (1 << (m_nBytes*8)) -1;
	uint32_t value = _value & nMask;

	if (!_bFromISR)
		osMutexAcquire(m_Mutex, 0xffff);
	uint32_t nMSB = *m_pData & ~nMask;
	value |= nMSB;
	*m_pData = value;
	if (!_bFromISR)
		osMutexRelease(m_Mutex);
}

// ----------------------------------------------------------------------------
//! \brief Specialization of the Read method for signed 32 bits data
template <> int32_t ProcessDataOut<int32_t>::Read(bool _bFromISR)
{	
	int32_t nMask = (1 << (m_nBytes*8)) -1;

	if (!_bFromISR)
		osMutexAcquire(m_Mutex, 0xffff);
	int32_t value = *m_pData & nMask;
	if (!_bFromISR)
		osMutexRelease(m_Mutex);

	return value;
}

// ----------------------------------------------------------------------------
//! \brief Specialization of the Write method for signed 32 bits data
template <> void ProcessDataOut<int32_t>::Write(int32_t _value, bool _bFromISR)
{	
	int32_t nMask = (1 << (m_nBytes*8)) -1;
	int32_t value = _value & nMask;

	if (!_bFromISR)
		osMutexAcquire(m_Mutex, 0xffff);
	int32_t nMSB = (*m_pData) & ~nMask;
	value |= nMSB;
	*m_pData = value;
	if (!_bFromISR)
		osMutexRelease(m_Mutex);
}

// ----------------------------------------------------------------------------
// Instantiate all the templates we will be using...
template class ProcessDataIn<uint8_t>;
template class ProcessDataIn<uint16_t>;
template class ProcessDataIn<uint32_t>;
template class ProcessDataIn<int16_t>;
template class ProcessDataIn<int32_t>;
template class ProcessDataOut<uint8_t>;
template class ProcessDataOut<uint16_t>;
template class ProcessDataOut<uint32_t>;
template class ProcessDataOut<int16_t>;
template class ProcessDataOut<int32_t>;
