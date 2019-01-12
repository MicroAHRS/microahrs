#ifndef TCANMESSAGEFUEL_H
#define TCANMESSAGEFUEL_H


#include "TCanMessageEMS.h"

#pragma pack(push,1)
class TCanMessageEMSFuel : public TCanMessageEMS
{
public:
    enum EStatus {
        E_STATUS_EMPTY    = 0,
        E_STATUS_OK       = 1,
        E_STATUS_BREAK    = 2,
    };

public:
    uint16_t m_level;
    uint8_t  m_status;
    uint8_t  m_key_tank_id;

public:

    TCanMessageEMSFuel(): TCanMessageEMS() {
        m_key_tank_id = 0;
        m_level = 0;
        m_status = 0;
    }

    uint8_t decode() {
        uint8_t i = 0;
        decodeField<uint16_t>(m_level , i);
        decodeField<uint8_t>(m_status , i);
        return i;
    }

    uint8_t encode() {
        setIdentifier(E_PRIORITY_NORMAL, E_MESSAGE_CODE_TANK_1 + m_key_tank_id);        
        uint8_t i = 0;
        encodeField<uint16_t>(m_level , i);
        encodeField<uint8_t>(m_status , i);
        m_len = i;
        return i;
    }
};

#pragma pack(pop)



#endif // TCANMESSAGEFUEL_H
