/***************************************************
  This is a library for the FXOS8700 Accel/Mag

  Designed specifically to work with the Adafruit FXOS8700 Breakout
  ----> https://www.adafruit.com/products

  These sensors use I2C to communicate, 2 pins (I2C)
  are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Kevin "KTOWN" Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/
#ifndef __FXOS8700_H__
#define __FXOS8700_H__

#include "shared/Geometry/TPoint3F.h"
#include "Arduino.h"
#include <stdint.h>

//#include "A_Sensor.h"
//#include <Wire.h>

#define FXOS8700_ADDRESS           (0x1F)     // 0011111
#define FXOS8700_ID                (0xC7)     // 1100 0111

enum EFXOS8700Register
{                                             // DEFAULT    TYPE
    FXOS8700_REGISTER_STATUS          = 0x00,
    FXOS8700_REGISTER_OUT_X_MSB       = 0x01,
    FXOS8700_REGISTER_OUT_X_LSB       = 0x02,
    FXOS8700_REGISTER_OUT_Y_MSB       = 0x03,
    FXOS8700_REGISTER_OUT_Y_LSB       = 0x04,
    FXOS8700_REGISTER_OUT_Z_MSB       = 0x05,
    FXOS8700_REGISTER_OUT_Z_LSB       = 0x06,
    FXOS8700_REGISTER_WHO_AM_I        = 0x0D,   // 11000111   r
    FXOS8700_REGISTER_XYZ_DATA_CFG    = 0x0E,
    FXOS8700_REGISTER_CTRL_REG1       = 0x2A,   // 00000000   r/w
    FXOS8700_REGISTER_CTRL_REG2       = 0x2B,   // 00000000   r/w
    FXOS8700_REGISTER_CTRL_REG3       = 0x2C,   // 00000000   r/w
    FXOS8700_REGISTER_CTRL_REG4       = 0x2D,   // 00000000   r/w
    FXOS8700_REGISTER_CTRL_REG5       = 0x2E,   // 00000000   r/w
    FXOS8700_REGISTER_MSTATUS         = 0x32,
    FXOS8700_REGISTER_MOUT_X_MSB      = 0x33,
    FXOS8700_REGISTER_MOUT_X_LSB      = 0x34,
    FXOS8700_REGISTER_MOUT_Y_MSB      = 0x35,
    FXOS8700_REGISTER_MOUT_Y_LSB      = 0x36,
    FXOS8700_REGISTER_MOUT_Z_MSB      = 0x37,
    FXOS8700_REGISTER_MOUT_Z_LSB      = 0x38,
    FXOS8700_REGISTER_MCTRL_REG1      = 0x5B,   // 00000000   r/w
    FXOS8700_REGISTER_MCTRL_REG2      = 0x5C,   // 00000000   r/w
    FXOS8700_REGISTER_MCTRL_REG3      = 0x5D,   // 00000000   r/w
};

#define ACCEL_RANGE_COUNT 3
class A_FXOS8700
{
public:
    enum ERange
    {
      ACCEL_RANGE_2G                    = 0x00,
      ACCEL_RANGE_4G                    = 0x01,
      ACCEL_RANGE_8G                    = 0x02
    };

    struct TRawData
    {
      int16_t x;
      int16_t y;
      int16_t z;
    };
  public:
    A_FXOS8700();

    bool begin           (uint8_t rng = ACCEL_RANGE_2G );
    bool getAccMag       (TPoint3F& acc,TPoint3F& mag);

    TRawData accel_raw; /* Raw values from last sensor read */
    TRawData mag_raw;   /* Raw values from last sensor read */

  private:
    void        write8  ( uint8_t reg, uint8_t value );
    uint8_t     read8   ( uint8_t reg );

    float       getSensivity() const;

    uint8_t              m_range;
};

#endif
