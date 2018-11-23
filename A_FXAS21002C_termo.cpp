#include "A_FXAS21002C_termo.h"

A_FXAS21002C_termo::A_FXAS21002C_termo() : A_FXAS21002C()
{
    m_enable_conpensation = true;
}

bool A_FXAS21002C_termo::getGyro( TPoint3F& gyro, float& temp )
{        
    if(!A_FXAS21002C::getGyro(gyro, temp))
        return false;

    if(!m_enable_conpensation)
        return true;

    gyro -= m_gyro_zero_avg(temp) * SENSORS_DPS_TO_RADS;
    return true;
}
