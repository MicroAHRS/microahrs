#ifndef TSTORAGE_H
#define TSTORAGE_H

#include <stdint.h>
class TStorage
{
public:
    TStorage();

    void getData(int address, uint8_t *ptr, unsigned int len);
    void putData( int address, const uint8_t *ptr, unsigned int len);

    //Functionality to 'get' and 'put' objects to and from EEPROM.
    template< typename T > T& get( int idx, T& t ) {getData(idx, (uint8_t*) &t, sizeof(T)); return t; }
    template< typename T > const T& put( int idx, const T& t ){ putData(idx, (const uint8_t*) &t, sizeof(T)); return t; }
};

#endif // TSTORAGE_H
