/***************************************************
  This is a library for the FXAS21002C Gyroscope

  Designed specifically to work with the Adafruit FXAS21002C Breakout
  ----> https://www.adafruit.com/products

  These sensors use I2C to communicate, 2 pins (I2C)
  are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Kevin "KTOWN" Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/
#ifndef __FXAS21002C_H__
#define __FXAS21002C_H__

#include "Adafruit_Sensor.h"

class Adafruit_FXAS21002C : public Adafruit_Sensor
{
public:
    enum EGyroRange {
        GYRO_RANGE_250DPS  = 250,
        GYRO_RANGE_500DPS  = 500,
        GYRO_RANGE_1000DPS = 1000,
        GYRO_RANGE_2000DPS = 2000
    };

    struct TGyroRawData {
        int16_t x;
        int16_t y;
        int16_t z;
    };
public:
    Adafruit_FXAS21002C(int32_t sensorID = -1);

    bool begin           ( EGyroRange rng = GYRO_RANGE_250DPS );
    bool getEvent        ( sensors_event_t* event);
    bool getEvent        ( sensors_event_t* , sensors_event_t *temp_event );
    void getSensor       ( sensor_t* );

    TGyroRawData    m_raw_data; /* Raw values from last sensor read */
    int8_t          m_raw_temperature;

protected:
    void        write8  ( byte reg, byte value );
    byte        read8   ( byte reg );
    byte        getFullScaleCode(const EGyroRange& rng );
    float       getSensitivity(const EGyroRange& rng );

    EGyroRange  _range;
    int32_t     _sensorID;    
};

#endif
