#ifndef TCANMESSAGEBASE_H
#define TCANMESSAGEBASE_H

#include <stdint.h>

#include "TCanMessageHeaderV1.h"

enum EPriority
{
    E_PRIORITY_EXTRA  = 0,
    E_PRIORITY_HIGH   = 1,
    E_PRIORITY_NORMAL = 2,
    E_PRIORITY_LOW    = 3
};

enum ESystem
{
    E_SYSTEM_EFIS     = 1,
    E_SYSTEM_EMS      = 2,
    E_SYSTEM_NAV      = 4,
    E_SYSTEM_OTHER    = 8,
};

#define CAN_MESSAGE_MAX_SIZE 8

class TCanMessageBase
{
public:
    TCanMessageBase() {
        m_len = 0;
    }

    virtual uint8_t decode() = 0;
    virtual uint8_t encode() = 0;

protected:

    template<class T>
    void encodeField(const T field, uint8_t& i) {
        const uint8_t l = sizeof(field);
        uint32_t f = field;
        if(uint8_t(i+l) > sizeof(m_data))
            return;
        m_data[i] = f;
        if(l>=2)
            m_data[i+1] = f >> 8;
        if(l>=3)
            m_data[i+2] = f >> 16;
        if(l>=4)
            m_data[i+3] = f >> 24;

        i+=l;
    }

    template<class T>
    void decodeField(T field, uint8_t& i) {
        const uint8_t l = sizeof(field);
        if(uint8_t(i+l) > m_len)
            return;
        uint32_t f = 0;
        f = m_data[i];
        if(l>=2)
            f |=  uint32_t(m_data[i+1]) << 8;
        if(l>=3)
            f |=  uint32_t(m_data[i+2]) << 16;
        if(l>=4)
            f |=  uint32_t(m_data[i+3]) << 24;
        field = f;
        i+=l;
    }


public:
    uint8_t  m_data[CAN_MESSAGE_MAX_SIZE];
    uint8_t  m_len;
    uint16_t m_id;

};

#endif // TCANMESSAGEBASE_H
