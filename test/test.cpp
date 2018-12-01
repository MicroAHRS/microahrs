// install it2c https://learn.sparkfun.com/tutorials/raspberry-pi-spi-and-i2c-tutorial/all

#include "../shared/Geometry/TQuaternionF.h"
#include "../shared/Geometry/TPoint3F.h"

#include "../shared/Function/TFunctionLineF.h"

#include <math.h>


//typedef TFunction3< TFunctionLineF , float> TFunctionCalibrate;


TQuaternionF InitWithAngles ( float yaw, float pitch, float roll )
{
    yaw   *= 0.5f;
    pitch *= 0.5f;
    roll  *= 0.5f;

    float	cx = (float)cos ( yaw );
    float	cy = (float)cos ( pitch );
    float	cz = (float)cos ( roll );
    float	sx = (float)sin ( yaw );
    float	sy = (float)sin ( pitch );
    float	sz = (float)sin ( roll );

    float	cc = cx * cx;
    float	cs = cx * sz;
    float	sc = sx * cz;
    float	ss = sx * sz;

    return TQuaternionF(cy * cc + sy * ss, cy * sc - sy * cs , cy * ss + sy * cc , cy * cs - sy * sc);
}


static inline void rotateAndScaleVector2(
    float q0, float q1, float q2, float q3,
    float _2dx, float _2dy, float _2dz,
    float& rx, float& ry, float& rz) {

    // result is half as long as input
    rx = _2dx * (0.5f - q2 * q2 - q3 * q3)
       + _2dy * (q0 * q3 + q1 * q2)
       + _2dz * (q1 * q3 - q0 * q2);
    ry = _2dx * (q1 * q2 - q0 * q3)
       + _2dy * (0.5f - q1 * q1 - q3 * q3)
       + _2dz * (q0 * q1 + q2 * q3);
    rz = _2dx * (q0 * q2 + q1 * q3)
       + _2dy * (q2 * q3 - q0 * q1)
       + _2dz * (0.5f - q1 * q1 - q2 * q2);
}

float DegToRad(float v) {
    return v / 180.0 * M_PI;
}

#include "stdio.h"
#include "shared/Geometry/TPoint3F.h"
#include "shared/Function/TFunctionAveragePoint3F.h"
#include "shared/Function/TFunctionAverageF.h"
#include "shared/Function/TFunctionLimitedAverage.h"

int main() {
    TFunctionLimitedAverage<TPoint3F,float,4> speed_avg(1);
    float time;
    for(time =0; time < 4;) {
        float dt = 0.01;
        time += dt;
        float speed = 5;
        if(time > 3)
            speed = 10;
        speed_avg.put(TPoint3F(speed),dt );

        printf("t = %f , avg speed = %f\n", time, speed_avg().x);
    }


//    TQuaternionF q = InitWithAngles( DegToRad(45), 0,  0);


//    TPoint3F v(1,0,0);
//    TPoint3F v2 = q.rotateVector(v) * 0.5;

//    TPoint3F v3;
//    rotateAndScaleVector2(q.w,q.x,q.y,q.z, v.x, v.y, v.z, v3.x, v3.y, v3.z );

//    int k = 0;


//     TFunctionCalibrate fun(
//                TFunctionLineF(0.02923636286, -2.442451397), // kx kc
//                TFunctionLineF(-0.00153622977, -0.512149624),
//                TFunctionLineF(-0.006581737428, 0.4034881183)
//                                            );
//     for(int t=-29;t<=74;t++) {
//         TPoint3F p = fun(t);
//         printf("t=%d %f %f %f \n", t , p.x, p.y,p.z );
//     }


}
