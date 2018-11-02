

#include "../shared/TQuaternion.h"
#include <math.h>

TQuaternionf InitWithAngles ( float yaw, float pitch, float roll )
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

    return TQuaternionf(cy * cc + sy * ss, cy * sc - sy * cs , cy * ss + sy * cc , cy * cs - sy * sc);
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

int main() {


    TQuaternionf q = InitWithAngles( DegToRad(45), 0,  0);


    TVector3f v(1,0,0);
    TVector3f v2 = q.rotateVector(v) * 0.5;

    TVector3f v3;
    rotateAndScaleVector2(q.w,q.x,q.y,q.z, v.x, v.y, v.z, v3.x, v3.y, v3.z );

    int k = 0;




}
