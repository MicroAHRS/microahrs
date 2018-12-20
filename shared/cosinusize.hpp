#ifndef __COSINUSIZE_H_INCLUDED__
#define __COSINUSIZE_H_INCLUDED__

#include <math.h>

inline float cosinusize(float v, float period=1)
{
    return 1-((cosf(v*period*M_PI)+1)*0.5f);
}

#endif