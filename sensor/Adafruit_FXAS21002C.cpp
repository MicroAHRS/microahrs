/***************************************************
  This is a library for the FXAS21002C GYROSCOPE

  Designed specifically to work with the Adafruit FXAS21002C Breakout
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

#include "Adafruit_FXAS21002C.h"


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
    I2C ADDRESS/BITS AND SETTINGS
    -----------------------------------------------------------------------*/
    #define FXAS21002C_ADDRESS       (0x21)       // 0100001
    #define FXAS21002C_ID            (0xD7)       // 1101 0111
    #define GYRO_SENSITIVITY_250DPS  (0.0078125F) // Table 35 of datasheet
    #define GYRO_SENSITIVITY_500DPS  (0.015625F)  // ..
    #define GYRO_SENSITIVITY_1000DPS (0.03125F)   // ..
    #define GYRO_SENSITIVITY_2000DPS (0.0625F)    // ..
/*=========================================================================*/

/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/
void Adafruit_FXAS21002C::write8(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(FXAS21002C_ADDRESS);
  #if ARDUINO >= 100
    Wire.write((uint8_t)reg);
    Wire.write((uint8_t)value);
  #else
    Wire.send(reg);
    Wire.send(value);
  #endif
  Wire.endTransmission();
}

/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/
uint8_t Adafruit_FXAS21002C::read8(uint8_t reg)
{
  uint8_t value;

  Wire.beginTransmission((uint8_t)FXAS21002C_ADDRESS);
  #if ARDUINO >= 100
    Wire.write((uint8_t)reg | 0x80);
  #else
    Wire.send(reg | 0x80);
  #endif
  if (Wire.endTransmission(false) != 0) return 0;
  Wire.requestFrom((uint8_t)FXAS21002C_ADDRESS, (uint8_t)1);
  #if ARDUINO >= 100
    value = Wire.read();
  #else
    value = Wire.receive();
  #endif

  return value;
}

float  Adafruit_FXAS21002C::getSensitivity(const Adafruit_FXAS21002C::EGyroRange &range ) {
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

//inline uint8_t Adafruit_FXAS21002C::getFullScaleCode(const Adafruit_FXAS21002C::EGyroRange& rng )
//{
//    /*
//    * 1:0 FullScale
//    *     00  = 2000
//    *     01  = 1000
//    *     10  = 500
//    *     11  = 250
//    *
//    * see https://cdn-learn.adafruit.com/assets/assets/000/040/671/original/FXAS21002.pdf?1491475056
//    *  Selectable Full Scale Ranges
//    */
//    switch (rng) {
//    case  GYRO_RANGE_250DPS:
//        return 0x3;
//    case GYRO_RANGE_500DPS:
//        return 0x2;
//    case GYRO_RANGE_1000DPS:
//        return 0x1;
//    case GYRO_RANGE_2000DPS:
//        return 0x0;
//    }
//    return 0x0;
//}

uint16_t Adafruit_FXAS21002C::getRangeDegrees() {
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

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Instantiates a new Adafruit_FXAS21002C class
*/
/**************************************************************************/
Adafruit_FXAS21002C::Adafruit_FXAS21002C(int32_t sensorID) {
  m_sensor_id = sensorID;
  m_raw_temperature = 0;
}


bool Adafruit_FXAS21002C::begin(uint8_t rng)
{
  /* Enable I2C */
  Wire.begin();

  /* Set the range the an appropriate value */
  m_range_code = (EGyroRange)rng;

  /* Clear the raw sensor data */
  m_raw_data.x = 0;
  m_raw_data.y = 0;
  m_raw_data.z = 0;

  /* Make sure we have the correct chip ID since this checks
     for correct address and that the IC is properly connected */
  uint8_t id = read8(GYRO_REGISTER_WHO_AM_I);
  // Serial.print("WHO AM I? 0x"); Serial.println(id, HEX);
  if (id != FXAS21002C_ID)  
    return false;

  /* Set CTRL_REG1 (0x13)
   ====================================================================
   BIT  Symbol    Description                                   Default
   ---  ------    --------------------------------------------- -------
     6  RESET     Reset device on 1                                   0
     5  ST        Self test enabled on 1                              0
   4:2  DR        Output data rate                                  000
                  000 = 800 Hz
                  001 = 400 Hz
                  010 = 200 Hz
                  011 = 100 Hz
                  100 = 50 Hz
                  101 = 25 Hz
                  110 = 12.5 Hz
                  111 = 12.5 Hz
     1  ACTIVE    Standby(0)/Active(1)                                0
     0  READY     Standby(0)/Ready(1)                                 0
     */

  /* Reset then switch to active mode with 100Hz output */
  write8(GYRO_REGISTER_CTRL_REG1, 0x00);
  write8(GYRO_REGISTER_CTRL_REG1, (1<<6));


  /* TODO control range scale by CTRL_REG0
   * Warning there is double range!! at CTRL_REG3
   * Set CTRL_REG3 (0x15)
   * Bit  7 6     5    4 3         2        1 0
   * * Read BW[1:0] SPIW SEL[1:0] HPF_EN FS[1:0]
   *
   * 1:0 FullScale
   *     00  = 2000
   *     01  = 1000
   *     10  = 500
   *     11  = 250
   *
   * see https://cdn-learn.adafruit.com/assets/assets/000/040/671/original/FXAS21002.pdf?1491475056
   */
  write8(GYRO_REGISTER_CTRL_REG0, m_range_code);// TODO try hi-pass filter

  write8(GYRO_REGISTER_CTRL_REG1, 0x0E); // 000-011-1-0     = 100 hz - active -

  delay(100); // 60 ms + 1/ODR
  /* ------------------------------------------------------------------ */

  return true;
}


/**************************************************************************/
/*!
    @brief  Gets the most recent sensor event
*/
/**************************************************************************/
bool Adafruit_FXAS21002C::getEvent(sensors_event_t* event, sensors_event_t* temp_event)
{
    //bool readingValid = false;

    /* Clear the event */
    memset(event, 0, sizeof(sensors_event_t));

    /* Clear the raw data placeholder */
    m_raw_data.x = 0;
    m_raw_data.y = 0;
    m_raw_data.z = 0;

    event->version   = sizeof(sensors_event_t);
    event->sensor_id = m_sensor_id;
    event->type      = SENSOR_TYPE_GYROSCOPE;
    event->timestamp = millis();

    /* Read 7 bytes from the sensor */
    Wire.beginTransmission((uint8_t)FXAS21002C_ADDRESS);
    #if ARDUINO >= 100
    Wire.write(GYRO_REGISTER_STATUS | 0x80);
    #else
    Wire.send(GYRO_REGISTER_STATUS | 0x80);
    #endif
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)FXAS21002C_ADDRESS, (uint8_t)7);

    /*uint8_t status =*/ Wire.read();
    uint8_t xhi = Wire.read();
    uint8_t xlo = Wire.read();
    uint8_t yhi = Wire.read();
    uint8_t ylo = Wire.read();
    uint8_t zhi = Wire.read();
    uint8_t zlo = Wire.read();  

    /* Shift values to create properly formed integer */
    event->gyro.x = (m_raw_data.x = (int16_t)((xhi << 8) | xlo));
    event->gyro.y = (m_raw_data.y = (int16_t)((yhi << 8) | ylo));
    event->gyro.z = (m_raw_data.z = (int16_t)((zhi << 8) | zlo));


    float sensitivity = getSensitivity(m_range_code);
    sensitivity *= SENSORS_DPS_TO_RADS; /* Convert values to rad/s */

    /* Compensate values depending on the resolution */
    event->gyro.x *= sensitivity;
    event->gyro.y *= sensitivity;
    event->gyro.z *= sensitivity;

    if(temp_event) {
        m_raw_temperature = read8(GYRO_REGISTER_TEMP);
        temp_event->temperature = m_raw_temperature;
        temp_event->version   = sizeof(sensors_event_t);
        temp_event->sensor_id = m_sensor_id;
        temp_event->type      = SENSOR_TYPE_AMBIENT_TEMPERATURE;
    }

  return true;
}

bool Adafruit_FXAS21002C::getEvent(sensors_event_t* event)
{
    return getEvent(event , NULL);
}


#ifndef DISABLE_SENSOR_INFO
/**************************************************************************/
/*!
    @brief  Gets the sensor_t data
*/
/**************************************************************************/
void  Adafruit_FXAS21002C::getSensor(sensor_t* sensor)
{
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy (sensor->name, "FXAS21002C", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name)- 1] = 0;
  sensor->version     = 1;
  sensor->sensor_id   = _sensorID;
  sensor->type        = SENSOR_TYPE_GYROSCOPE;
  sensor->min_delay   = 0;
  sensor->max_value   = (float)getRangeDegrees(_range) * SENSORS_DPS_TO_RADS;
  sensor->min_value   = (getRangeDegrees(_range) * -1.0) * SENSORS_DPS_TO_RADS;
  sensor->resolution  = 0.0F; // TBD
}

#endif













