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

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Adafruit_Sensor.h>
#include <Wire.h>

/*=========================================================================
    I2C ADDRESS/BITS AND SETTINGS
    -----------------------------------------------------------------------*/
    #define FXAS21002C_ADDRESS       (0x21)       // 0100001
    #define FXAS21002C_ID            (0xD7)       // 1101 0111
    #define GYRO_SENSITIVITY_250DPS  (0.0078125F) // Table 35 of datasheet
    #define GYRO_SENSITIVITY_500DPS  (0.015625F)  // ..
    #define GYRO_SENSITIVITY_1000DPS (0.03125F)   // ..
    #define GYRO_SENSITIVITY_2000DPS (0.0625F)    // ..
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    typedef enum
    {                                             // DEFAULT    TYPE
      GYRO_REGISTER_STATUS              = 0x00,
      GYRO_REGISTER_OUT_X_MSB           = 0x01,
      GYRO_REGISTER_OUT_X_LSB           = 0x02,
      GYRO_REGISTER_OUT_Y_MSB           = 0x03,
      GYRO_REGISTER_OUT_Y_LSB           = 0x04,
      GYRO_REGISTER_OUT_Z_MSB           = 0x05,
      GYRO_REGISTER_OUT_Z_LSB           = 0x06,
      GYRO_REGISTER_WHO_AM_I            = 0x0C,   // 11010111   r
      GYRO_REGISTER_CTRL_REG0           = 0x0D,   // 00000000   r/w
      GYRO_REGISTER_CTRL_REG1           = 0x13,   // 00000000   r/w
      GYRO_REGISTER_CTRL_REG2           = 0x14,   // 00000000   r/w    

      GYRO_REGISTER_CTRL_REG3           = 0x15,   // double range  r/w
      GYRO_REGISTER_TEMP                = 0x12,   // temperature value   r
    } gyroRegisters_t;
/*=========================================================================*/

/*=========================================================================
    OPTIONAL SPEED SETTINGS
    -----------------------------------------------------------------------*/
    typedef enum
    {
      GYRO_RANGE_250DPS  = 250,
      GYRO_RANGE_500DPS  = 500,
      GYRO_RANGE_1000DPS = 1000,
      GYRO_RANGE_2000DPS = 2000
    } gyroRange_t;
/*=========================================================================*/

/*=========================================================================
    RAW GYROSCOPE DATA TYPE
    -----------------------------------------------------------------------*/
    typedef struct gyroRawData_s
    {
      int16_t x;
      int16_t y;
      int16_t z;
    } gyroRawData_t;
/*=========================================================================*/
template<class T>
class Vector3
{
public:
    T x;
    T y;
    T z;
    uint8_t status;

    Vector3() { x = y = z = 0; status=1; }
    Vector3(T _x, T _y, T _z ) { x = _x;y = _y; z = _z; status=1; }

    inline Vector3<T> operator + (const Vector3<T>& p) {
        Vector3<T> result;
        result.x = this->x + p.x;
        result.y = this->y + p.y;
        result.z = this->z + p.z;
        return result;
    }

    inline bool hasNan() {return x!=x || y!=y || z!=z;}
    inline bool isInited() {return status;}
};

typedef Vector3<float> Vector3f;
typedef Vector3<int>   Vector3i;

class Adafruit_FXAS21002C : public Adafruit_Sensor
{
  public:  
    Adafruit_FXAS21002C(int32_t sensorID = -1);

    bool begin           ( gyroRange_t rng = GYRO_RANGE_250DPS );
    bool getEvent        (sensors_event_t* event);
    bool getEvent        (sensors_event_t* , sensors_event_t *temp_event );
    void getSensor       ( sensor_t* );

    gyroRawData_t   raw; /* Raw values from last sensor read */
    int8_t          raw_temperature;
    bool            enable_calibration;

    bool calibratePointSet(const Vector3f &point, int temp);
    bool        calibratePointExist(int8_t temp);
    void        calibratePrintStatus();

  private:
    void        write8  ( byte reg, byte value );
    byte        read8   ( byte reg );
    byte        getFullScaleCode(const gyroRange_t& rng );
    float       getSensitivity(const gyroRange_t& rng );

    int         calibratePointAddressGet(int8_t temp);
    Vector3f&   calibratePointGet(int8_t temp);

    gyroRange_t _range;
    int32_t     _sensorID;
    Vector3f    _calibration_point;
    int         _calibration_address;
};

#endif
