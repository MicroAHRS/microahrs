#ifndef TCanMessageEMS_H
#define TCanMessageEMS_H


#include "TCanMessageBase.h"
class TCanMessageEMS : public TCanMessageBase
{
public:
    enum EMessageCode {
        E_MESSAGE_CODE_TANK_1  = 0,
        E_MESSAGE_CODE_TANK_2  = 1,
        E_MESSAGE_CODE_TANK_3  = 2,
        E_MESSAGE_CODE_TANK_4  = 3,
        E_MESSAGE_CODE_TANK_5  = 4,
        E_MESSAGE_CODE_TANK_6  = 5,
        E_MESSAGE_CODE_TANK_7  = 6,
        E_MESSAGE_CODE_TANK_8  = 7,

        E_MESSAGE_CODE_EGTCHT12  = 8,
        E_MESSAGE_CODE_EGTCHT34  = 9,
        E_MESSAGE_CODE_EGTCHT56  = 10,
        E_MESSAGE_CODE_RESERV_0  = 11,

        E_MESSAGE_CODE_RPM       = 12,
        E_MESSAGE_CODE_ELECTRO   = 13,
        E_MESSAGE_CODE_OIL_FUEL  = 14,
        E_MESSAGE_CODE_RESERV_1  = 15,
    };

protected:
    inline void setIdentifier(uint8_t priority, uint8_t message_code) {
         m_id = TCanMessageHeader_1::Build(priority, E_SYSTEM_EMS, message_code);
    }

};

#endif // TCANMESSAGEBASE_H
