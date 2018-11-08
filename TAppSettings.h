#ifndef TAPPSETTINGS_H
#define TAPPSETTINGS_H

#include "shared/Geometry/TMatrix3F.h"
#include "shared/Geometry/TPoint3F.h"
#include "shared/Function/TFunction3.h"
#include "shared/Function/TFunctionLineF.h"

class TAppSettings
{
public:
    TAppSettings() :
        mag_offset(
            19.07,
            16.78 - 0.25,
            63.38 - 0.25
        )
        ,mag_matrix (
              TPoint3F(  0.959, 0.002, 0.003 ),
              TPoint3F(  0.002, 0.964, 0.008  ),
              TPoint3F(  0.003, 0.008, 1.082  )
        )
        ,gyro_zero_offset(
           0,
           0,
           0
        )
      ,gyro_temperature(
           TFunctionLineF(0.02923636286, -2.442451397), // kx kc
           TFunctionLineF(-0.00153622977, -0.512149624),
           TFunctionLineF(-0.006581737428, 0.4034881183)
      )
      ,acc_zero_offset(
           0.3253305,
           -0.3593825,
           0.6124513)
      ,acc_scale(
           1.01591586,
           0.9863349377,
           1.017566665
       )
    {

        beta_start = 20;
        beta       = 0.1;

        zeta_start = 0;
        zeta       = 0.01;


        print_out_time_ms = 1000 / 20;
        disable_gyro = false;
        disable_acc  = false;
        disable_mag  = false;

        if(disable_gyro) {
            zeta = 0;
            zeta_start = 0;
        }

        float gravity_delta    = 9.8 * 0.2;
        acc_max_length_sq      = 9.8 + gravity_delta;
        acc_min_length_sq      = 9.8 - gravity_delta;
        acc_max_length_sq     *= acc_max_length_sq;
        acc_min_length_sq     *= acc_min_length_sq; // square

        pitch_max = 250;
        roll_max = 150;

        gravity_vector  = TPoint3F(0,0,1);
        magnetic_vector = TPoint3F(1,0,0);
    }


    float beta;
    float beta_start;
    float zeta;
    float zeta_start;
    unsigned long print_out_time_ms;   //miliseconds

    bool disable_gyro;
    bool disable_acc;
    bool disable_mag;

    float acc_max_length_sq;
    float acc_min_length_sq;

    float pitch_max;
    float roll_max;

    // mag calibration
    TPoint3F  mag_offset;
    TMatrix3F mag_matrix;
    // gyro calibration
    TPoint3F                            gyro_zero_offset;
    TFunction3< TFunctionLineF , float> gyro_temperature;
    // acc colibration
    TPoint3F  acc_zero_offset;
    TPoint3F  acc_scale;

    // default orientation
    TPoint3F  gravity_vector;
    TPoint3F  magnetic_vector;
};

#endif // TAPPSETTINGS_H
