/*!
 *
 *
 * @section author Author
 *
 * Written by Evgeny Pronin titan.the.proger@gmail.com
 *
 * @section license License
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
#include "TStorage.h"

#include "config.h"

TStorage::TStorage()
{
}

#ifdef PLATFORM_ARDUINO
#include <EEPROM.h>
void TStorage::getData( int address, uint8_t *ptr, unsigned int len)
{
    //code form EEPROM.h
    EEPtr e = address;
    for( ; len ; --len, ++e )
        *ptr++ = *e;
}

void TStorage::putData( int address, const uint8_t *ptr, unsigned int len)
{
    //code form EEPROM.h
    EEPtr e = address;
    for( ; len ; --len, ++e )
        (*e).update( *ptr++ );
}
#endif

#ifdef PLATFORM_RASPBERRY


#endif
