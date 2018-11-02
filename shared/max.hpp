#ifndef __GAIJIN_CHIBILIB_MAX_HEADER_
#define __GAIJIN_CHIBILIB_MAX_HEADER_
#ifdef _MSC_VER
#  pragma once
#endif

//ради того, что бы не включать мега-хидер <algorithm> ради std::max

template <class T> const T &Max(const T &a, const T &b)
{
    return (a < b) ? b : a;
}

template <class T> const T &Min(const T &a, const T &b)
{
    return (a < b) ? a : b;
}

#endif //__GAIJIN_CHIBILIB_MAX_HEADER_
