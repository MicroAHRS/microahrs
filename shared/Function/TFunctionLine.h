#ifndef TFUNCTIONLINE_H
#define TFUNCTIONLINE_H

/**
  Represent linear function

**/


#include "TFunction.h"

template <class TVal, class TArg>
class TFunctionLine : public TFunction<TVal, TArg>
{
public:

    TFunctionLine() {m_k = (TVal)1; m_c =(TVal) 0;}
    TFunctionLine(TVal k, TVal c) {m_k = k; m_c = c;}
    TFunctionLine(TArg x1, TVal y1, TArg x2, TVal y2) {
        TArg dx = x2 - x1;
        m_k = (dx != 0) ? (y2 - y1) / dx : (TVal)0;
        m_c = y1 - (m_k * x1    );
    }

    TVal operator () (const TArg& param) const {
        return m_k*param + m_c;
    }

private:
    TVal m_k;
    TVal m_c;
};

#endif // TFUNCTIONLINE_H
