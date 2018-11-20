#ifndef TAPPSETTINGS_H
#define TAPPSETTINGS_H

#include "shared/Geometry/TMatrix3F.h"
#include "shared/Geometry/TPoint3F.h"
#include "shared/Geometry/TQuaternionF.h"
#include "shared/Function/TFunction3.h"
#include "shared/Function/TFunctionLineF.h"
#include <EEPROM.h>

#include "Adafruit_FXOS8700.h"
#include "Adafruit_FXAS21002C.h"

#define SETTINGS_ADDRESS 10

#define PRINTOUT_TIME_MS 100

//#define DEVICE_SET_ALL
//#define DEVICE_SET_1
#define DEVICE_SET_2

class TAppSettings
{
public:
    TAppSettings() { initDefault();}
    void initDefault() {

        acc_zero_offset = TPoint3F( 0,0,0);
        acc_scale = TPoint3F( 1, 1, 1 );
        gyro_temperature = TFunction3< TFunctionLineF , float>(
                    TFunctionLineF(), // kx kc
                    TFunctionLineF(),
                    TFunctionLineF()
                );

        acc_mode = Adafruit_FXOS8700::ACCEL_RANGE_4G;
        gyro_mode = Adafruit_FXAS21002C::GYRO_RANGE_250DPS;

        mag_matrix = TMatrix3F(
                TPoint3F(  1, 0, 0 ),
                TPoint3F(  0, 1, 0 ),
                TPoint3F(  0, 0, 1 )
        );
        mag_offset = TPoint3F( 0, 0, 0);
        gyro_zero_offset = TPoint3F(0, 0, 0 );

        beta       = 0.04;
        zeta       = 0.004;
        disable_gyro = false;
        disable_acc  = false;
        disable_mag  = false;
        print_mag    = true;
        pitch_max = 250;
        roll_max = 150;

#ifdef DEVICE_SET_1
        acc_zero_offset = TPoint3F( 0.3253305, -0.3593825, 0.6124513);
        acc_scale = TPoint3F( 1.01591586, 0.9863349377, 1.017566665 );
        gyro_temperature = TFunction3< TFunctionLineF , float>(
                TFunctionLineF(0.02923636286, -2.442451397), // kx kc
                TFunctionLineF(-0.00153622977, -0.512149624),
                TFunctionLineF(-0.006581737428, 0.4034881183)
                );
        mag_offset = TPoint3F( 19.07, 16.78 - 0.25, 63.38 - 0.25);
#endif
#ifdef DEVICE_SET_2
        acc_zero_offset = TPoint3F( 0.2368895, -0.3631025, 0.3326025);
        acc_scale = TPoint3F( 1.008551146, 1.005089637, 1.00459703 );
        gyro_mode = Adafruit_FXAS21002C::GYRO_RANGE_500DPS;
        gyro_temperature = TFunction3< TFunctionLineF , float>(
                    TFunctionLineF(32, 1.26462    ,69 ,  1.80386 ), // kx kc
                    TFunctionLineF(32,-0.13643    ,69 ,  0.0376  ),
                    TFunctionLineF(32,-0.14466001 ,69 , -0.18477 )
                );
        mag_matrix = TMatrix3F(
                TPoint3F(  1.001, 0.026, 0.003 ),
                TPoint3F(  0.026, 0.9999, 0.001 ),
                TPoint3F(  0.003, 0.001, 1.001 )
        );
        mag_offset = TPoint3F( -72.49,-87.58, 66.34);
        beta       = 0.016;
        zeta       = 0.004;
        disable_mag  = true;
#endif

        float gravity_delta    = 9.8 * 0.2;
        acc_max_length_sq      = 9.8 + gravity_delta;
        acc_min_length_sq      = 9.8 - gravity_delta;
        acc_max_length_sq     *= acc_max_length_sq;
        acc_min_length_sq     *= acc_min_length_sq; // square


        sensor_to_frame_orientation = TQuaternionF(1,0,0,0);

        m_save_count = 0;
        m_crc = 0;
    }

    static uint32_t updateCRC32(uint32_t crc, uint8_t b) {
        const unsigned long crc_table[16] = {
          0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
          0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
          0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
          0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
        };
        crc = crc_table[(crc ^ b) & 0x0f] ^ (crc >> 4);
        crc = crc_table[(crc ^ (b >> 4)) & 0x0f] ^ (crc >> 4);
        return  ~crc;
    }
    static uint32_t calcCRC32(const void *data, int size)
    {
        const uint8_t* p = (const uint8_t *)data;
        uint32_t crc = ~0L;
        for(int i=0;i<size;i++) {
            crc = updateCRC32(crc, p[i]);
        }
        return crc;
    }

    bool load() {
        EEPROM.get(SETTINGS_ADDRESS, *this);
        uint32_t crc_origin = m_crc;
        m_crc = 0;
        m_crc = calcCRC32(this, sizeof(*this));
        return m_crc == crc_origin;
    }
    bool save() {
        m_save_count++;
        m_crc = 0;
        m_crc = calcCRC32(this, sizeof(*this));
        EEPROM.put(SETTINGS_ADDRESS, *this);
        return true;
    }

public:
    float beta;    
    float zeta;        

    bool disable_gyro;
    bool disable_acc;
    bool disable_mag;

    bool print_mag;

    float acc_max_length_sq;
    float acc_min_length_sq;

    float pitch_max;
    float roll_max;

    // mag calibration
    TPoint3F  mag_offset;
    TMatrix3F mag_matrix;
    // gyro calibration
    uint8_t   gyro_mode;
    TPoint3F  gyro_zero_offset;
    TFunction3< TFunctionLineF , float> gyro_temperature;
    // acc colibration
    TPoint3F  acc_zero_offset;
    TPoint3F  acc_scale;
    uint8_t   acc_mode;


    // default orientation
    TQuaternionF sensor_to_frame_orientation;

    uint32_t m_crc;
    uint16_t m_save_count;
};

#endif // TAPPSETTINGS_H
