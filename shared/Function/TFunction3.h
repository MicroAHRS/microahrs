#ifndef TFUNCTIONLINE3_H
#define TFUNCTIONLINE3_H


#include "TFunction.h"
#include "../Geometry/TPoint3.h"

/**
 * T - class function 1d  based form TFunction
 **/
template <class TFunc, class T>
class TFunction3
{
public:
    TFunction3() {}
    TFunction3(const TFunc& fx, const TFunc& fy,const TFunc& fz) {
        m_fx = fx;
        m_fy = fy;
        m_fz = fz;
    }

    inline TPoint3<T> operator () (const T& param) const {
        return TPoint3<T>(m_fx(param),m_fy(param), m_fz(param));
    }

public:
    TFunc m_fx;
    TFunc m_fy;
    TFunc m_fz;
};


#endif // TFUNCTIONLINE3_H
