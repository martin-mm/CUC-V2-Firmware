#ifndef _IIRFILTER_H_
#define _IIRFILTER_H_

#include <stdint.h>

#include "Base.h"

template<class T>

class IIR_64
{
public:
	IIR_64(int64_t b[3],int64_t a[2],int32_t div) :
		m_div(div)
   {
		for (int i = 0;i < 3;i++)
			m_b[i] = b[i];
		for (int i = 0;i < 2;i++)
			m_a[i] = a[i];
		Reset();
   }

   T Update(T nMeasure) 
   {
		int64_t sum = m_a[0] * (int64_t)nMeasure;
		for (int i = 1;i < 2;i++)
		{
			int k = i - 1;
			sum += x_old[k] * m_a[k];
			sum -= y_old[k] * m_b[k];
		}
		sum /= m_div;
		x_old[1] = x_old[0];
		x_old[0] = nMeasure;
		y_old[1] = y_old[0];
		y_old[0] = sum;
		result = (T)sum;
		return Get();
   }

   T Get() { return result; }

   void Reset()
   {
		for (int i = 0;i < 2;i++)
		{
			x_old[i] = 0;
			x_old[i] = 0;
		}
   }

private:
   int32_t	x_old[2];
	int32_t	y_old[2];
	int32_t	m_a[2];
	int32_t	m_b[2];
	int32_t	m_div;
	T			result;
};


template<class T>

class IIR
{
	private:
		float	m_b0;
		float	m_b1;
		float	m_b2;
		float	m_a1;
		float	m_a2;
		float	x1;
		float	x2;
		float	y1;
		float	y2;
	public:
		IIR(float b0,float b1,float b2,float a1,float a2) :
			m_b0(b0),
			m_b1(b1),
			m_b2(b2),
			m_a1(a1),
			m_a2(a2)
		{
			Reset();
		}
		T Update(T value)
		{
			float		result;
			float		x0 = (float)value;
			
			result = m_b0 * x0 + m_b1 * x1 + m_b2 * x2 - m_a1 * y1 - m_a2 *y2;
			x2 = x1;
			x1 = x0;
			y2 = y1;
			y1 = result;
			return (T)result;
		}
		void Reset(void)
		{
			x1 = 0.0F;
			x2 = 0.0F;
			y1 = 0.0F;
			y2 = 0.0F;
		}
};

#endif /* _IIRFILTER_H_ */
