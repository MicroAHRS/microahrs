// see https://www.nxp.com/docs/en/data-sheet/FXAS21002.pdf

//#include <limits.h>

#include "A_FXAS21002C.h"

#include "config.h"


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

#pragma pack(push,1)

union TReg_CTRL_REG_0
{
    uint8_t data;
    struct {
        uint8_t  fullscale_range:2;
        uint8_t  hipass_filter_enable:1;
        uint8_t  hipass_filter_selection:2;
        uint8_t  spi_mode:1;   // 0 - 4 wire mode,  1 - 3 wire mode
        uint8_t  bandwidth_low_pass_filter:2;
    } bits;
};

union TReg_CTRL_REG_1
{
    enum DataRate {
        DATA_RATE_800  = 0,
        DATA_RATE_400  = 1,
        DATA_RATE_200  = 2,
        DATA_RATE_100  = 3,
        DATA_RATE_50   = 4,
        DATA_RATE_25   = 5,
        DATA_RATE_12_5 = 6,
        //DATA_RATE_12_5 = 7,
    };

    uint8_t data;
    struct {
        uint8_t  ready:1;
        uint8_t  active:1;
        uint8_t  data_rate:3;
        uint8_t  self_test:1;
        uint8_t  reset:1;
        uint8_t  unused:1;
    } bits;
};

union TReg_CTRL_REG_3
{
    uint8_t data;
    struct {
        uint8_t  full_scale_double:1;
        uint8_t  unused:1;
        uint8_t  extctrlen:1;
        uint8_t  wraptoone:1;
        uint8_t  unused_2:4;
    } bits;
};

union TReg_D_STATUS
{
    uint8_t data;
    struct {
        uint8_t  x_data_ready:1;
        uint8_t  y_data_ready:1;
        uint8_t  z_data_ready:1;
        uint8_t  zyx_data_ready:1;
        uint8_t  x_overwrite:1;
        uint8_t  y_overwrite:1;
        uint8_t  z_overwrite:1;
        uint8_t  zyx_overwrite:1;
    } bits;
};


#pragma pack(pop)








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

#ifdef PLATFORM_ARDUINO
#include "Arduino.h"
#include <Wire.h>

void A_FXAS21002C::write8(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(FXAS21002C_ADDRESS);
    Wire.write((uint8_t)reg);
    Wire.write((uint8_t)value);
    Wire.endTransmission();
}

uint8_t A_FXAS21002C::read8(uint8_t reg)
{  
    Wire.beginTransmission((uint8_t)FXAS21002C_ADDRESS);
    Wire.write((uint8_t)reg | 0x80);
    if (Wire.endTransmission(false) != 0)
        return 0;
    Wire.requestFrom((uint8_t)FXAS21002C_ADDRESS, (uint8_t)1);
    return Wire.read();
}
#endif

float  A_FXAS21002C::getSensitivity(const A_FXAS21002C::EGyroRange &range ) {
    switch(range)
    {
      case GYRO_RANGE_250DPS:
        return GYRO_SENSITIVITY_250DPS;
      case GYRO_RANGE_500DPS:
        return GYRO_SENSITIVITY_500DPS;
      case GYRO_RANGE_1000DPS:
        return GYRO_SENSITIVITY_1000DPS;
      case GYRO_RANGE_2000DPS:
        return GYRO_SENSITIVITY_2000DPS;
    }
    return GYRO_SENSITIVITY_2000DPS;
}


uint16_t A_FXAS21002C::getRangeDegrees() {
    switch (m_range_code) {
    case  GYRO_RANGE_250DPS:
        return 250;
    case GYRO_RANGE_500DPS:
        return 500;
    case GYRO_RANGE_1000DPS:
        return 1000;
    case GYRO_RANGE_2000DPS:
        return 2000;
    }
    return 0x0;
}


A_FXAS21002C::A_FXAS21002C()
{
    m_raw_temperature = 0;
    m_too_slow = 0;
}



bool A_FXAS21002C::begin(uint8_t rng)
{
    /* Enable I2C */
    Wire.begin();

    /* Set the range the an appropriate value */
    m_range_code = (EGyroRange)rng;

    /* Clear the raw sensor data */
    m_raw_data.x = 0;
    m_raw_data.y = 0;
    m_raw_data.z = 0;

    uint8_t id = read8(GYRO_REGISTER_WHO_AM_I);
    // Serial.print("WHO AM I? 0x"); Serial.println(id, HEX);
    if (id != FXAS21002C_ID)
        return false;

    TReg_CTRL_REG_0 reg0;
    TReg_CTRL_REG_1 reg1;
    TReg_CTRL_REG_3 reg3;
    reg0.data = reg1.data = reg3.data =  0;
    write8(GYRO_REGISTER_CTRL_REG1, reg1.data);
    reg1.bits.reset = 1;
    write8(GYRO_REGISTER_CTRL_REG1, reg1.data);
    reg0.bits.fullscale_range = m_range_code;
    //reg0.bits.bandwidth
    write8(GYRO_REGISTER_CTRL_REG0, reg0.data);

    reg3.bits.full_scale_double = 0;
    write8(GYRO_REGISTER_CTRL_REG3, reg3.data);

    reg1.bits.reset     = 0;
    reg1.bits.data_rate = TReg_CTRL_REG_1::DATA_RATE_200;
    reg1.bits.active    = 1;
    write8(GYRO_REGISTER_CTRL_REG1, reg1.data);

    delay(100); // 60 ms + 1/ODR
    /* ------------------------------------------------------------------ */

    return true;
}

bool A_FXAS21002C::getGyro( TPoint3F& gyro)
{
    /* Read 7 bytes from the sensor */
    Wire.beginTransmission((uint8_t)FXAS21002C_ADDRESS);
    Wire.write(GYRO_REGISTER_STATUS | 0x80);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)FXAS21002C_ADDRESS, (uint8_t)7);

    TReg_D_STATUS reg_status;
    reg_status.data = Wire.read();
    if(reg_status.bits.zyx_overwrite)
        m_too_slow++;

    if(!reg_status.bits.zyx_data_ready)
        return false; // data is not ready
        //Serial.println("override");

    uint8_t xhi = Wire.read();
    uint8_t xlo = Wire.read();
    uint8_t yhi = Wire.read();
    uint8_t ylo = Wire.read();
    uint8_t zhi = Wire.read();
    uint8_t zlo = Wire.read();

    /* Shift values to create properly formed integer */
    gyro.x = (m_raw_data.x = (int16_t)((xhi << 8) | xlo));
    gyro.y = (m_raw_data.y = (int16_t)((yhi << 8) | ylo));
    gyro.z = (m_raw_data.z = (int16_t)((zhi << 8) | zlo));
    gyro *= getSensitivity(m_range_code) * SENSORS_DPS_TO_RADS;

    return true;
}

void A_FXAS21002C::getTemp( float& temp )
{
    m_raw_temperature = read8(GYRO_REGISTER_TEMP);
    temp = m_raw_temperature;
}













