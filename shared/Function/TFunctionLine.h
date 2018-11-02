#ifndef TFUNCTIONLINE_H
#define TFUNCTIONLINE_H

/**
  Represent linear function

**/


#include "TFunction.h"

template <class T>
class TFunctionLine : public TFunction<T>
{
public:

    TFunctionLine() {m_k = 0; m_c = 0;}
    TFunctionLine(T k, T c) {m_k = k; m_c = c;}
    TFunctionLine(T x1, T y1,T x2, T y2) {
        T dx = x2 - x1;
        m_k = dx != 0? (y2 - y1) / dx : 0;
        m_c = y1 - (x1 * m_k);
    }

    inline T operator () (const T& param) const {
        return m_k*param+m_c;
    }

private:
    T m_k;
    T m_c;
};

#endif // TFUNCTIONLINE_H
