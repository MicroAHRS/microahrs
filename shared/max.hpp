#ifndef ___MAX_HEADER_
#define ___MAX_HEADER_
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

#endif //___MAX_HEADER_
