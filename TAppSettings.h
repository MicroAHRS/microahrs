#ifndef TAPPSETTINGS_H
#define TAPPSETTINGS_H

#include "shared/Geometry/TMatrix3F.h"
#include "shared/Geometry/TPoint3F.h"
#include "shared/Geometry/TQuaternionF.h"
#include "shared/Function/TFunctionLinePoint3F.h"

#include "A_FXOS8700.h"
#include "A_FXAS21002C.h"

#define SETTINGS_ADDRESS 10
#define PRINTOUT_TIME_MS 100

class TAppSettings
{
public:
    TAppSettings() { initDefault();}
    void initDefault();
    bool load();
    bool save();

    uint8_t getVersion() const {return 1;}
public:
    uint8_t version;
    float beta;    
    float zeta;        
    float neta;

    bool disable_gyro;
    bool disable_acc;
    bool disable_mag;

    bool print_mag;

    float acc_max_length_sq;
    float acc_min_length_sq;

    float roll_max;

    // mag calibration
    TPoint3F  mag_offset;
    TMatrix3F mag_matrix;
    // gyro calibration
    uint8_t   gyro_mode;
    TPoint3F  gyro_zero_offset;
    TFunctionLinePoint3F gyro_temperature;
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
