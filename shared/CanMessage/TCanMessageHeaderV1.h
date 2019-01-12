#ifndef CANMESSAGEHEADER_H
#define CANMESSAGEHEADER_H

#include <stdint.h>

#pragma pack(push,1)

union TCanMessageHeader_1
{
    uint16_t data;
    struct {
        uint8_t m_message: 4;
        uint8_t m_system : 4;
        uint8_t m_priority: 2;
        uint8_t m_reserv: 1;
        uint8_t m_unused: 6;
    } bits;


    static uint16_t Build(uint8_t priority, uint8_t system_id, uint8_t message_id )
    {
        TCanMessageHeader_1 header;
        header.data = 0;
        header.bits.m_priority = priority;
        header.bits.m_system  = system_id;
        header.bits.m_message = message_id;
        return header.data;
    }

};

#endif
