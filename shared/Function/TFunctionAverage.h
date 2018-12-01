#ifndef TFUNCTIONAVERAGE_H
#define TFUNCTIONAVERAGE_H

#include "TFunction.h"

/**
 * The avarage vlue
 * for example avarage speed
 *     input values - speed and delta time
 *
 * result - avarage speed by all time
 *
 */
template <class TVal, class TArg>
class TFunctionAverage : public TFunction<TVal, TArg>
{
public:
    TFunctionAverage(): m_length(0), m_time_total(0) {}
    TFunctionAverage(const TVal& length, const TArg& time): m_length(length), m_time_total(time) {}

    static TFunctionAverage<TVal, TArg> FromValue(const TVal& speed, const TArg& time) {
        return TFunctionAverage<TVal, TArg>(speed*time, time );
    }


    TVal getAvarage() const  { return ((m_time_total > 0) ? (m_length / m_time_total) : TVal(0) ); }
    TVal operator ()() const  { return getAvarage(); }

    TFunctionAverage<TVal, TArg>& put(const TVal& speed, const TArg& time) {
        return (*this) += TFunctionAverage<TVal, TArg>::FromValue(speed, time);
    }

    TFunctionAverage<TVal, TArg>& operator += (const TFunctionAverage<TVal, TArg>& p) {
        m_length += p.m_length;
        m_time_total += p.m_time_total;
        return *this;
    }

    TFunctionAverage<TVal, TArg>& operator -= (const TFunctionAverage<TVal, TArg>& p) {
        m_length -= p.m_length;
        m_time_total -= p.m_time_total;
        return *this;
    }

    TFunctionAverage<TVal, TArg> operator + (const TFunctionAverage<TVal, TArg>& p) const {
        TFunctionAverage<TVal, TArg> result(*this);
        return result += p;
    }

    const TArg& getTimeTotal() const {return m_time_total;}

protected:
    TVal m_length;       //integral
    TArg m_time_total;
};

#endif // TFUNCTIONAVERAGE_H






