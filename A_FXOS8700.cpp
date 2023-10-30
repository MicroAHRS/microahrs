/*!
 * @file Adafruit_FXOS8700.cpp
 *
 * @mainpage Adafruit FXOS8700 accel/mag sensor driver
 *
 * @section intro_sec Introduction
 *
 * This is the documentation for Adafruit's FXOS8700 driver for the
 * Arduino platform.  It is designed specifically to work with the
 * Adafruit FXOS8700 breakout: https://www.adafruit.com/products/3463
 *
 * These sensors use I2C to communicate, 2 pins (SCL+SDA) are required
 * to interface with the breakout.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section dependencies Dependencies
 *
 * This library depends on <a
 * href="https://github.com/adafruit/Adafruit_Sensor"> Adafruit_Sensor</a> being
 * present on your system. Please make sure you have installed the latest
 * version before using this library.
 *
 * @section author Author
 *
 * Written by Kevin "KTOWN" Townsend for Adafruit Industries.
 *
 * @section license License
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

//#include "Arduino.h"

//#include <limits.h>
#include "A_FXOS8700.h"
#include "config.h"

#define FXOS8700_ADDRESS           (0x1F)     // 0011111
#define FXOS8700_ID                (0xC7)     // 1100 0111

#define ACCEL_MG_LSB_2G (0.000244F)
#define ACCEL_MG_LSB_4G (0.000488F)
#define ACCEL_MG_LSB_8G (0.000976F)
#define MAG_UT_LSB      (0.1F)



// see https://www.nxp.com/docs/en/data-sheet/FXOS8700CQ.pdf
/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/
#ifdef PLATFORM_ARDUINO
#include <Wire.h>
void A_FXOS8700::write8(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(FXOS8700_ADDRESS);
    Wire.write((uint8_t)reg);
    Wire.write((uint8_t)value);
    Wire.endTransmission();
}
uint8_t A_FXOS8700::read8(uint8_t reg)
{
    Wire.beginTransmission((uint8_t)FXOS8700_ADDRESS);

    Wire.write((uint8_t)reg);

    if (Wire.endTransmission(false) != 0) return 0;
    Wire.requestFrom((uint8_t)FXOS8700_ADDRESS, (uint8_t)1);

    return Wire.read();
}
#endif

union FXOS8700Reg_CTRL_REG_1
{
    enum DataRateMono {
        DATA_RATE_MONO_800  = 0,
        DATA_RATE_MONO_400  = 1,
        DATA_RATE_MONO_200  = 2,
        DATA_RATE_MONO_100  = 3,
        DATA_RATE_MONO_50   = 4,
        DATA_RATE_MONO_12_5 = 5,
        DATA_RATE_MONO_6_25 = 6,
        DATA_RATE_MONO_1_5625 = 7,
    };

    enum DataRateHybrid {
        DATA_RATE_HYBRID_400  = 0,
        DATA_RATE_HYBRID_200  = 1,
        DATA_RATE_HYBRID_100  = 2,
        DATA_RATE_HYBRID_50  = 3,
        DATA_RATE_HYBRID_25   = 4,
        DATA_RATE_HYBRID_6_25 = 5,
        DATA_RATE_HYBRID_3_125 = 6,
        DATA_RATE_HYBRID_0_7813 = 7,
    };

    uint8_t data;
    struct {
        uint8_t  active:1;
        uint8_t  f_read:1;
        uint8_t  lnoise:1;
        uint8_t  data_rate:3;
        uint8_t  aslp_rate:3;
    } bits;
};

union FXOS8700Reg_MCTRL_REG_1
{
    uint8_t data;
    struct {
        uint8_t  hybrid_mode_select:2;
        uint8_t  oversample_ratio:3;
        uint8_t  one_shoot_triger:1;
        uint8_t  one_shoot_reset:1;
        uint8_t  mag_auto_calibration:1;
    } bits;
};

A_FXOS8700::A_FXOS8700()
{
}

bool A_FXOS8700::begin(uint8_t rng)
{
    /* Enable I2C */
    Wire.begin();

    /* Set the range the an appropriate value */
    m_range = rng;

    /* Clear the raw sensor data */
    accel_raw.x = accel_raw.y = accel_raw.z = 0;
    mag_raw.x = mag_raw.y = mag_raw.z = 0;

    /* Make sure we have the correct chip ID since this checks
     for correct address and that the IC is properly connected */
    uint8_t id = read8(FXOS8700_REGISTER_WHO_AM_I);
    if (id != FXOS8700_ID)
        return false;

    FXOS8700Reg_CTRL_REG_1 reg1;
    reg1.data = 0;
    reg1.bits.active = 0;

    /* Set to standby mode (required to make changes to this register) */
    write8(FXOS8700_REGISTER_CTRL_REG1, reg1.data);

    write8(FXOS8700_REGISTER_XYZ_DATA_CFG, (uint8_t)m_range);
    /* Configure the accelerometer */

    /* High resolution */
    //write8(FXOS8700_REGISTER_CTRL_REG2, 0x02);
    write8(FXOS8700_REGISTER_CTRL_REG2, 0x00); // ProninE we need avarage

    /* Active, Normal Mode, Low Noise, 50Hz in Hybrid Mode */

    reg1.data = 0;
    reg1.bits.active = 1;
    reg1.bits.lnoise = 1;
    //no fast read
    reg1.bits.data_rate = FXOS8700Reg_CTRL_REG_1::DATA_RATE_HYBRID_25;

    //write8(FXOS8700_REGISTER_CTRL_REG1, 0x15); // 00010101
    write8(FXOS8700_REGISTER_CTRL_REG1, reg1.data); // 00010101

    /* Configure the magnetometer */
    /* Hybrid Mode, Over Sampling Rate = 16 */
    FXOS8700Reg_MCTRL_REG_1 mreg1;
    mreg1.data = 0;
    mreg1.bits.oversample_ratio = 0x6;
    mreg1.bits.hybrid_mode_select = 0x3;
    write8(FXOS8700_REGISTER_MCTRL_REG1, mreg1.data); // was 0x1F
    /* Jump to reg 0x33 after reading 0x06 */
    write8(FXOS8700_REGISTER_MCTRL_REG2, 0x20);
    return true;
}

bool A_FXOS8700::getAccMag(TPoint3F& acc,TPoint3F& mag)
{
    /* Clear the raw data placeholder */
//    accel_raw.x = accel_raw.y = accel_raw.z = 0;
//     mag_raw.x = mag_raw.y = mag_raw.z = 0;

    /* Read 13 uint8_ts from the sensor */
    Wire.beginTransmission((uint8_t)FXOS8700_ADDRESS);
    Wire.write(FXOS8700_REGISTER_STATUS | 0x80);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)FXOS8700_ADDRESS, (uint8_t)13);

    /* ToDo: Check status first! */
    /*uint8_t status =*/ Wire.read();
    uint8_t axhi = Wire.read();
    uint8_t axlo = Wire.read();
    uint8_t ayhi = Wire.read();
    uint8_t aylo = Wire.read();
    uint8_t azhi = Wire.read();
    uint8_t azlo = Wire.read();

    /* Shift values to create properly formed integers */
    /* Note, accel data is 14-bit and left-aligned, so we shift two bit right */
    acc.x = accel_raw.x = (int16_t)((axhi << 8) | axlo) >> 2;
    acc.y = accel_raw.y = (int16_t)((ayhi << 8) | aylo) >> 2;
    acc.z = accel_raw.z = (int16_t)((azhi << 8) | azlo) >> 2;

    uint8_t mxhi = Wire.read();
    uint8_t mxlo = Wire.read();
    uint8_t myhi = Wire.read();
    uint8_t mylo = Wire.read();
    uint8_t mzhi = Wire.read();
    uint8_t mzlo = Wire.read();

    mag.x = mag_raw.x = (int16_t)((mxhi << 8) | mxlo);
    mag.y = mag_raw.y = (int16_t)((myhi << 8) | mylo);
    mag.z = mag_raw.z = (int16_t)((mzhi << 8) | mzlo);

    /* Convert accel values to m/s^2 */
    acc *= getSensivity() * SENSORS_GRAVITY_STANDARD;

    /* Convert mag values to uTesla */
    mag *= MAG_UT_LSB;
    return true;
}


float A_FXOS8700::getSensivity() const {
    switch (m_range) {
    case ACCEL_RANGE_2G:
        return ACCEL_MG_LSB_2G;
    case ACCEL_RANGE_4G:
        return ACCEL_MG_LSB_4G;
    case ACCEL_RANGE_8G:
        return ACCEL_MG_LSB_8G;
    }
    return 0;
}



