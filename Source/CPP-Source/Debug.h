// ----------------------------------------------------------------------------
// IOBoard
//
// Defines debugging classes and functions
//
// Copyright (C) 2011-2013 BlueBotics SA
// ----------------------------------------------------------------------------

#ifndef _DEBUG_H_
#define _DEBUG_H_

#include "Base.h"

// ----------------------------------------------------------------------------
// Constants
#define NB_TRACES 128

// ----------------------------------------------------------------------------
// TraceData
class TraceData
{
public:
    uint16_t nMagic;
    uint32_t nData1;
    uint32_t nData2;
    uint32_t nData3;
};

// ----------------------------------------------------------------------------
// Trace
class Trace
{
public:    
    Trace() { m_nPos = 0; }
    static void Log(uint32_t _nData1, uint32_t _nData2, uint32_t _nData3);
    
public:
    uint8_t m_nPos;
    TraceData m_Data[NB_TRACES];
};

extern Trace g_Trace;
void Trace::Log(uint32_t _nData1, uint32_t _nData2, uint32_t _nData3)
{
    TraceData *pLastData = &g_Trace.m_Data[g_Trace.m_nPos];
    if (_nData1 != pLastData->nData1 ||
        _nData2 != pLastData->nData2 ||
        _nData3 != pLastData->nData3)
    {
        g_Trace.m_nPos = (++g_Trace.m_nPos) % NB_TRACES;
        TraceData *pData = &g_Trace.m_Data[g_Trace.m_nPos];
        pData->nMagic = 0x1234;
        pData->nData1 = _nData1;
        pData->nData2 = _nData2;
        pData->nData3 = _nData3;
    }        
}

#endif // _DEBUG_H_
