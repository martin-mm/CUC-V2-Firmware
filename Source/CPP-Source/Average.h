// ----------------------------------------------------------------------------
// ARM library
//
// Defines a class computing the average of a serie of values
//
// Copyright (C) 2011-2013 BlueBotics SA
// ----------------------------------------------------------------------------

#ifndef _AVERAGE_H_
#define _AVERAGE_H_

#include "Base.h"

template<int N, class T>
class Average
{
public:
    Average() 
    { 
        //m_nMagic = 0x55443322;
        Reset();
    }
    T Update(T nMeasure) 
    {
        m_nTotal -= m_aMeasures[m_nPos];
        m_nTotal += nMeasure;
        m_aMeasures[m_nPos] = nMeasure;
        m_nPos = (++m_nPos) % N;
        return Get();
    }
    T Get() { return (m_nTotal/N); }
    void Reset()
    {
        m_nPos=0;
        m_nTotal=0;
        for (uint32_t i=0; i<N; i++)
				m_aMeasures[i] = 0;
    }

private:
    //uint32_t m_nMagic;
    uint32_t m_nPos;
    int32_t m_nTotal;
    T m_aMeasures[N];
};

#endif // _AVERAGE_H_
