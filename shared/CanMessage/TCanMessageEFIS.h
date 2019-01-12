#ifndef TCANMESSAGEEFIS_H
#define TCANMESSAGEEFIS_H


#include "TCanMessageBase.h"
class TCanMessageEFIS : public TCanMessageBase
{
public:
    enum EMessageCode {
        E_MESSAGE_CODE_ORIENTATION  = 0,
        E_MESSAGE_CODE_ACCEL        = 1,
        E_MESSAGE_CODE_BARO         = 2,
        E_MESSAGE_CODE_BARO_RAW     = 3,
        E_MESSAGE_CODE_QNH   = 4,
        E_MESSAGE_CODE_OUTT  = 5,
    };

protected:
    inline void setIdentifier(uint8_t priority, uint8_t message_code) {
         m_id = TCanMessageHeader_1::Build(priority, E_SYSTEM_EFIS, message_code);
    }

};

#endif // TCANMESSAGEBASE_H
