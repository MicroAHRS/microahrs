#include "A_FXAS21002C_termo.h"
#include "shared/Geometry/TPoint3F.h"

A_FXAS21002C_termo::A_FXAS21002C_termo(int32_t sensorID)
    :A_FXAS21002C(sensorID)
{
    m_enable_conpensation = true;

}

bool A_FXAS21002C_termo::getEvent( sensors_event_t* gyro_event, sensors_event_t *temp_event )
{    
    if(!temp_event)
        return false;

    if(!A_FXAS21002C::getEvent(gyro_event, temp_event))
        return false;

    if(!m_enable_conpensation)
        return true;

    TPoint3F gyro_avg = m_gyro_zero_avg(temp_event->temperature) * SENSORS_DPS_TO_RADS;
    gyro_event->gyro.x -= gyro_avg.x;
    gyro_event->gyro.y -= gyro_avg.y;
    gyro_event->gyro.z -= gyro_avg.z;
    return true;
}
