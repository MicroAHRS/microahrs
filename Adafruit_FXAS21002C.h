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
    // see https://cdn-learn.adafruit.com/assets/assets/000/040/671/original/FXAS21002.pdf?1491475056
    enum EGyroRange {
        GYRO_RANGE_250DPS  = 3,
        GYRO_RANGE_500DPS  = 2,
        GYRO_RANGE_1000DPS = 1,
        GYRO_RANGE_2000DPS = 0,
    };
#define GYRO_RANGE_COUNT 4

    struct TGyroRawData {
        int16_t x;
        int16_t y;
        int16_t z;
    };
public:
    Adafruit_FXAS21002C(int32_t sensorID = -1);

    bool begin           ( uint8_t rng = GYRO_RANGE_250DPS );
    bool getEvent        ( sensors_event_t* event);
    bool getEvent        ( sensors_event_t* , sensors_event_t *temp_event );    
    void getSensor       ( sensor_t* );
    uint16_t        getRangeDegrees();

    TGyroRawData    m_raw_data; /* Raw values from last sensor read */
    int8_t          m_raw_temperature;


protected:
    void        write8  ( uint8_t reg, uint8_t value );
    uint8_t     read8   ( uint8_t reg );
    //inline uint8_t     getFullScaleCode(const EGyroRange& rng );
    inline float       getSensitivity(const EGyroRange& rng );


    EGyroRange  m_range_code;
    int32_t     m_sensor_id;
};

#endif
