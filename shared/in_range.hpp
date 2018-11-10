#ifndef __GAIJIN_CHIBILIB_IN_RANGE_HEADER_
#define __GAIJIN_CHIBILIB_IN_RANGE_HEADER_
#ifdef _MSC_VER
#  pragma once
#endif

#include <assert.h>

///x - в промежутке [a, b) исключая b. 
template <class T, class T1, class T2>
inline bool InRange(const T &x, const T1 &a, const T2 &b)
{
	return x >= a  && x < b;
}

///x - в промежутке [a, b] _ВКЛЮЧИТЕЛЬНО_. Так что осторожней с границами для массивов и итераторов, там мы привыкли к другой логике.
template <class T, class T1, class T2>
inline bool InRangeInc(const T &x, const T1 &a, const T2 &b)
{
	return x >= a  && x <= b;
}


///Обрезает x по отрезку [a, b].
///Если x меньше a, возращает a, если x больше b возвращает b, иначе возвращает сам x.
///a должно быть меньше либо равно b
template <class T, class T2, class T3>
inline T Clamp(const T &x, const T2 &a, const T3 &b)
{
    assert(a <= b);
    return (x < a) ? a : (x > b ? b : x);
}

/// озвращает число от 0 до 1.0f, 0 если progress не достиг from
///  инейно от 0 до 1 если progress  в пределах от from до to
/// 1 если progress больше to
inline float GetProgressSection(float progress, float from , float to)
{
    float len = to - from;
    assert(len > 0);
    return Clamp( (progress - from) / len, 0.0f, 1.0f);
}

inline float Lerp(float progress, float from, float to )
{
    return from * (1.0 - progress) + to * progress;
}

#endif //__GAIJIN_CHIBILIB_IN_RANGE_HEADER_
