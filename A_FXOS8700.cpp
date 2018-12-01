/***************************************************
  This is a library for the FXOS8700 Accel/Mag

  Designed specifically to work with the Adafruit FXOS8700 Breakout
  ----> https://www.adafruit.com/products/

  These sensors use I2C to communicate, 2 pins (I2C)
  are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Kevin "KTOWN" Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include "Arduino.h"
#include <Wire.h>
//#include <limits.h>
#include "A_FXOS8700.h"

#define ACCEL_MG_LSB_2G (0.000244F)
#define ACCEL_MG_LSB_4G (0.000488F)
#define ACCEL_MG_LSB_8G (0.000976F)
#define MAG_UT_LSB      (0.1F)

#define SENSORS_GRAVITY_EARTH             (9.80665F)              /**< Earth's gravity in m/s^2 */
#define SENSORS_GRAVITY_STANDARD          (SENSORS_GRAVITY_EARTH)
/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/

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

    /* Set to standby mode (required to make changes to this register) */
    write8(FXOS8700_REGISTER_CTRL_REG1, 0);

    write8(FXOS8700_REGISTER_XYZ_DATA_CFG, (uint8_t)m_range);
    /* Configure the accelerometer */

    /* High resolution */
    write8(FXOS8700_REGISTER_CTRL_REG2, 0x02);
    /* Active, Normal Mode, Low Noise, 100Hz in Hybrid Mode */
    write8(FXOS8700_REGISTER_CTRL_REG1, 0x15);

    /* Configure the magnetometer */
    /* Hybrid Mode, Over Sampling Rate = 16 */
    write8(FXOS8700_REGISTER_MCTRL_REG1, 0x1F);
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



