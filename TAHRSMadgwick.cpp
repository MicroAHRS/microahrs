//=============================================================================================
// Madgwick.c
//=============================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
// see @link = https://github.com/ccny-ros-pkg/imu_tools/tree/indigo/imu_filter_madgwick
//-------------------------------------------------------------------------------------------
// Header files

#include "TAHRSMadgwick.h"
#include <math.h>

#include "shared/max.hpp"
#include "shared/in_range.hpp"

//#if ARDUINO >= 100
// #include "Arduino.h"
//#else
// #include "WProgram.h"
//#endif


#define gyroMeasError 10   // gyroscope measurement error in rad/s (shown as 5 deg/s)
#define gyroMeasDrift 0.5f // gyroscope measurement error in rad/s/s (shown as 0.2f deg/s/s)
#define GRAVITY_VECTOR TPoint3F(0,0,1)
#define MAGNITUDE_VECTOR TPoint3F(1,0,0)
//============================================================================================
// Functions

//-------------------------------------------------------------------------------------------
// AHRS algorithm update

TAHRSMadgwick::TAHRSMadgwick()
{
    setGyroMeas(gyroMeasError, gyroMeasDrift);
    setOrientation(TQuaternionF( 1.0f, 0, 0, 0));    
//    m_gravity = TPoint3F(0,0,1);
//    m_magnitude = TPoint3F(1,0,0);
    setZetaMaxAngle(90);
}


void TAHRSMadgwick::setZetaMaxAngle(float value) {
    m_zeta_max_angle = cos(value * CONVERT_DPS_TO_RAD);
}

void TAHRSMadgwick::setGyroMeas(float error, float drift)
{
    beta = sqrt(3.0f / 4.0f) * error * CONVERT_DPS_TO_RAD;
    zeta = sqrt(3.0f / 4.0f) * drift * CONVERT_DPS_TO_RAD;

//    beta  = Clamp(beta , 0.0f, 10.0f);
//    zeta  = Clamp(zeta , 0.0f, 10.0f);
}

static inline TPoint3F compensateMagneticDistortion(
        const TQuaternionF& q,
        const TPoint3F& mag)
{
    // Reference direction of Earth's magnetic field (See EQ 46)
    TPoint3F result = q.getConjugate().rotateVector(mag);
    // now we have vector in Earth system
    float zz = result.z * result.z;
    result.y = 0;
    result.x = (zz >= 1.0) ? 0 : sqrt(1.0 - zz);
    return result;
}


static inline TPoint3F removeMagneticVertical(
        const TQuaternionF& q,
        const TPoint3F& mag)
{
    // now we have vector in Earth ref system
    TPoint3F result = q.getConjugate().rotateVector(mag);
    // remove vertical component
    result.z = 0;
    // return vector to Device ref system
    return q.rotateVector(result);
}


static inline TQuaternionF orientationChangeFromGyro(
    const TQuaternionF& q,
    const TPoint3F& gyro)
{
    // Rate of change of quaternion from gyroscope
    // See EQ 12    
    // Qdot = 0.5 * q * gyro
    return  q * 0.5f * TQuaternionF(gyro);
}

static TQuaternionF addGradientDescentStep(
    const TQuaternionF& q,     // текущая ориентация
    const TPoint3F& dest,     // вектор в системе Земли целевой - длинна 1
    const TPoint3F& source,    // вектор в локальной систее - длинна 1
        TPoint3F& dest_local  // dest vector at local system - for reuse
        )
{    
    TQuaternionF result;

    // Gradient decent algorithm corrective step
    // EQ 15, 21    
    dest_local = q.rotateVector(dest);
    TPoint3F delta = dest_local - source;
    TPoint3F d2 = dest*2;

    // EQ 22, 34
    // Jt * f
    result.w =
        (                            d2.y * q.z -        d2.z * q.y) * delta.x
      + (-       d2.x * q.z                            + d2.z * q.x) * delta.y
      + (        d2.x * q.y -        d2.y * q.x                    ) * delta.z;

    result.x =
        (                            d2.y * q.y +        d2.z * q.z) * delta.x
      + (        d2.x * q.y - 2.0f * d2.y * q.x +        d2.z * q.w) * delta.y
      + (        d2.x * q.z -        d2.y * q.w - 2.0f * d2.z * q.x) * delta.z;

    result.y =
        (-2.0f * d2.x * q.y +        d2.y * q.x -        d2.z * q.w) * delta.x
      + (        d2.x * q.x                     +        d2.z * q.z) * delta.y
      + (        d2.x * q.w +        d2.y * q.z - 2.0f * d2.z * q.y) * delta.z;

    result.z =
        (-2.0f * d2.x * q.z +        d2.y * q.w +        d2.z * q.x) * delta.x
      + (-       d2.x * q.w - 2.0f * d2.y * q.z +        d2.z * q.y) * delta.y
      + (        d2.x * q.x +        d2.y * q.y                    ) * delta.z;

    return result;
}


static inline bool SameSign(const float& x, const float& y) {
    return (x >= 0) ^ (y < 0);
}

inline  void TAHRSMadgwick::compensateGyroDrift(
        const TQuaternionF& s,     // поправка кватрениона c отрицательным знаком
        const TPoint3F& gyro,
        float dt)
{   
    TQuaternionF& q = m_q; //just reference

    // w_err = 2 q x s
    if(zeta == 0)
        return;

    //TPoint3F err = (q * s).toVector3() * 2.0f;  не работает - возможно имел ввиду векторное произведение
    // w_err = 2 q x s
    TPoint3F err;
    err.x = q.w * s.x - q.x * s.w -  q.y * s.z + q.z * s.y;
    err.y = q.w * s.y + q.x * s.z -  q.y * s.w - q.z * s.x;
    err.z = q.w * s.z - q.x * s.y +  q.y * s.x - q.z * s.w;
    err *= -2.0f;
    float coef = dt * zeta;

    //if(SameSign(err.x, gyro.x) && m_angle_dest.x > m_zeta_max_angle)
    if(SameSign(err.x, gyro.x))
        m_gyro_error.x += err.x * coef;

    //if(SameSign(err.y, gyro.y)  && m_angle_dest.y > m_zeta_max_angle)
    if(SameSign(err.y, gyro.y))
        m_gyro_error.y += err.y * coef;

    //if(SameSign(err.z, gyro.z)  && m_angle_dest.z > m_zeta_max_angle)
    if(SameSign(err.z, gyro.z))
        m_gyro_error.z += err.z * coef;
}

#define AXIS_X 0
#define AXIS_Y 1
#define AXIS_Z 2

float ComputeVectorAngle(TPoint3F p1, TPoint3F p2, const short a)
{
    switch (a) {
    case AXIS_X:
        p1.x= p2.x= 0;
        break;
    case AXIS_Y:
        p1.y= p2.y= 0;
        break;
    case AXIS_Z:
        p1.z= p2.z= 0;
        break;
    }
    p1.normalize();
    p2.normalize();
    return p1.dot_product(p2);
}

TQuaternionF TAHRSMadgwick::correctiveStepAccel(TPoint3F& acc)
{
    TQuaternionF result;
    if(acc.isZero())
        return result;

    acc.normalize();
    TPoint3F dest_local;
    return addGradientDescentStep(m_q, GRAVITY_VECTOR, acc, dest_local);
    //m_angle_dest.x = ComputeVectorAngle(dest_local, acc, AXIS_X);
    //m_angle_dest.y = ComputeVectorAngle(dest_local, acc, AXIS_Y);
    //return result;
}

TQuaternionF TAHRSMadgwick::correctiveStepMag(TPoint3F& mag, bool yaw_only)
{
    TQuaternionF result;
    if(mag.isZero())
        return result;
    mag.normalize();

    if(yaw_only)
        mag = removeMagneticVertical(m_q, mag);

    if(mag.isZero())
        return result;
    mag.normalize();

//    TPoint3F mag_dest = (yaw_only) ? bearing : compensateMagneticDistortion(q, mag);
//    return addGradientDescentStep(q, mag_dest, mag);
    //TPoint3F dest_local;
    //m_angle_dest.z = ComputeVectorAngle(dest_local, mag, AXIS_Z);

    return addGradientDescentStep(m_q, MAGNITUDE_VECTOR, mag, m_north_local);
}


//////////////////////////////////////////////////////////////////

void TAHRSMadgwick::update(TPoint3F gyro, TPoint3F acc, TPoint3F mag, float dt)
{
    gyro -= m_gyro_error;

    TQuaternionF q_dot(0,0,0,0);
    q_dot -= correctiveStepAccel(acc);
    q_dot -= correctiveStepMag(mag, true);

    if(!q_dot.isZero()) {
        q_dot.normalize();
        compensateGyroDrift(q_dot, gyro, dt);
        q_dot *= beta;
    }

    q_dot += orientationChangeFromGyro(m_q, gyro);
    // Integrate rate of change of quaternion to yield quaternion
    changeOrientation(q_dot * dt);       
}

inline void TAHRSMadgwick::changeOrientation(const TQuaternionF& delta)
{
    m_q += delta;
    m_q.normalize();    
}

void TAHRSMadgwick::setOrientation(const TQuaternionF& value) {
    m_q = value;
    m_q.normalize();
}
