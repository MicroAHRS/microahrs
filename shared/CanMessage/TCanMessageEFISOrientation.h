#ifndef TCANMESSAGEFUEL_H
#define TCANMESSAGEFUEL_H


#include "TCanMessageEFIS.h"
#include "math.h"
#include "../in_range.hpp"

#pragma pack(push,1)
class TCanMessageEFISOrientation : public TCanMessageEFIS
{
public:
    uint16_t m_roll;    //x
    uint16_t m_pitch;   //y
    uint16_t m_yaw;     //z
    uint8_t  m_g_angle;
    uint8_t  m_g;

    const uint16_t UINT16_MAX_VAL = 0xFF00;
    const uint8_t  UINT8_MAX_VAL = 250;

public:
    TCanMessageEFISOrientation(): TCanMessageEFIS() {
        m_roll      = 0;
        m_pitch     = 0;
        m_yaw       = 0;
        m_g_angle   = 0;
        m_g         = 0;
    }

    uint8_t decode() {
        uint8_t i = 0;
        decodeField(m_roll , i);
        decodeField(m_pitch , i);
        decodeField(m_yaw , i);
        decodeField(m_g_angle , i);
        decodeField(m_g, i);
        return i;
    }

    uint8_t encode() {
        setIdentifier(E_PRIORITY_LOW, E_MESSAGE_CODE_ORIENTATION);

        uint8_t i = 0;
        encodeField(m_roll , i);
        encodeField(m_pitch , i);
        encodeField(m_yaw , i);
        encodeField(m_g_angle , i);
        encodeField(m_g , i);
        m_len = i;
        return i;
    }

    inline void setRoll(float value) {m_roll = setAngle(value);}
    inline void setPitch(float value) {m_pitch = setAngle(value);}
    inline void setYaw(float value) {m_yaw = setAngle(value);}
    inline void setG(float value) {
        value = GetProgressSection(value, -2, 4);
        value *= UINT8_MAX_VAL;
        m_g = value;
    }

    inline void setGAngle(float value) {
        value = GetProgressSection(value, -0.174533, 0.174533);
        value *= UINT8_MAX_VAL;
        m_g_angle = value;
    }

private:
    uint16_t setAngle(float angle) {
        const float MAX_RANGE =  M_PI * 2;

        while (angle >= MAX_RANGE)
            angle -= MAX_RANGE;

        while (angle < 0)
            angle += MAX_RANGE;

        return GetProgressSection(angle, 0, MAX_RANGE) * UINT16_MAX_VAL;
    }
};

#pragma pack(pop)



#endif // TCANMESSAGEFUEL_H
