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


#define gyroMeasErrorMag 1
#define gyroMeasError    1   // gyroscope measurement error in rad/s (shown as 5 deg/s)
#define gyroMeasDrift 0.05f // gyroscope measurement error in rad/s/s (shown as 0.2f deg/s/s)
//#define gyroMeasCoef  0.8660254038


#define GRAVITY_VECTOR TPoint3F(0,0,1)
#define MAGNITUDE_VECTOR TPoint3F(1,0,0)

#define CORRECTION_MAX_TIME_STABLE  0.5
#define GYRO_COPNESATION_MIN_TIME   2
#define GYRO_COPNESATION_MAX_TIME   30
//============================================================================================
// Functions

//-------------------------------------------------------------------------------------------
// AHRS algorithm update

TAHRSMadgwick::TAHRSMadgwick()
    : m_q()
#ifdef GYRO_COPNESATION_AVARAGE
    ,m_gyro_avg_f(10, 1)
#endif
{
    setGyroMeas(gyroMeasError, gyroMeasDrift, gyroMeasErrorMag);
    setOrientation(TQuaternionF( 1.0f, 0, 0, 0));
    m_accel_correction_size = 1;
    m_calibration_mode = false;
//    m_gravity = TPoint3F(0,0,1);
//    m_magnitude = TPoint3F(1,0,0);
//    setZetaMaxAngle(90);
}

//void TAHRSMadgwick::setZetaMaxAngle(float value) {
//    m_zeta_max_angle = cos(value * CONVERT_DPS_TO_RAD);
//}

void TAHRSMadgwick::setGyroMeas(float error, float drift,float error_mag)
{
    //0.8660254038 = sqrt(3.0f / 4.0f)
    //float coef = sqrt(3.0f / 4.0f) * CONVERT_DPS_TO_RAD;

    beta = BETTA_COEF * error;
    zeta = BETTA_COEF * drift;
    neta = BETTA_COEF * error_mag;
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

static float CompensateGyroChannel(const float& gyro, const float& err_delta, float& err_delta_first, float& err_time  , float & unstable_time, const float& dt)
{
    if(gyro == 0)
        return 0;


    err_time += dt;
    unstable_time += dt;

    // time of swicth sign
    if(!SameSign(err_delta, err_delta_first)) {
        if(err_time < CORRECTION_MAX_TIME_STABLE)
            unstable_time = 0;
        err_time = 0;
        err_delta_first = err_delta;
    }

#ifdef GYRO_COPNESATION_TIME
    if(InRange(unstable_time, GYRO_COPNESATION_MIN_TIME, GYRO_COPNESATION_MAX_TIME))
        return 0;
#endif

#ifdef GYRO_COPNESATION_SLOWDOWN
    if(!SameSign(err_delta, gyro) && unstable_time > 1.0f)
        return 0;
#endif

    return err_delta;
}

void TAHRSMadgwick::compensateGyroDrift(
        const TQuaternionF& s,     // поправка кватрениона
        const TPoint3F& gyro,      // показания гиросокпа , 0 если канал отключен
        const float& dt)
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
    err *= 2.0f * dt * zeta;
    m_gyro_error.x += CompensateGyroChannel(gyro.x , err.x , m_correction_direction.x , m_correction_switch_time.x , m_unstable_time.x, dt);
    m_gyro_error.y += CompensateGyroChannel(gyro.y , err.y , m_correction_direction.y , m_correction_switch_time.y , m_unstable_time.y, dt);
    m_gyro_error.z += CompensateGyroChannel(gyro.z , err.z , m_correction_direction.z , m_correction_switch_time.z , m_unstable_time.z, dt);


    if(gyro.x != 0 && gyro.y != 0 ) {
        float max_time = Max(m_correction_switch_time.x, m_correction_switch_time.y);
        if(max_time <= dt * 2.5)
            m_accel_correction_size = Max(0.01f , m_accel_correction_size*0.9f);
        else if(max_time >= dt * 4)
            m_accel_correction_size = Min(1.0f , m_accel_correction_size*1.5f);
    }

    if(gyro.z != 0 ) {
        if(m_correction_switch_time.z <= dt * 2.5)
            m_mag_correction_size = Max(0.01f , m_mag_correction_size*0.9f);
        else if(m_correction_switch_time.z >= dt * 4)
            m_mag_correction_size = Min(1.0f , m_mag_correction_size*1.5f);
    }

}

//#define AXIS_X 0
//#define AXIS_Y 1
//#define AXIS_Z 2

//float ComputeVectorAngle(TPoint3F p1, TPoint3F p2, const short a)
//{
//    switch (a) {
//    case AXIS_X:
//        p1.x= p2.x= 0;
//        break;
//    case AXIS_Y:
//        p1.y= p2.y= 0;
//        break;
//    case AXIS_Z:
//        p1.z= p2.z= 0;
//        break;
//    }
//    p1.normalize();
//    p2.normalize();
//    return p1.dot_product(p2);
//}

TQuaternionF TAHRSMadgwick::correctiveStepAccel(TPoint3F& acc, const float& dt)
{
    TQuaternionF result;
    if(acc.isZero() || beta == 0.0f)
        return result;

    acc.normalize();
    TPoint3F dest_local;
    result = addGradientDescentStep(m_q, GRAVITY_VECTOR, acc, dest_local);

    if(!result.isZero())  {
        result.normalize();

        if(m_accel_correction_size < 1.0f)
            result *= m_accel_correction_size;

        compensateGyroDrift(result, TPoint3F(m_gyro_avg.x, m_gyro_avg.y, 0), dt);
        result *= beta;
    }

    return result;
}


inline TPoint3F TAHRSMadgwick::removeMagneticVertical(const TPoint3F& mag)
{
    // now we have vector in Earth ref system
    TPoint3F result = m_q.getConjugate().rotateVector(mag);
    // remove vertical component
    result.z = 0;
    m_mag_horisontal = result;
    // return vector to Device ref system
    return m_q.rotateVector(result);
}

TQuaternionF TAHRSMadgwick::correctiveStepMag(TPoint3F& mag, const float &dt)
{            
    TQuaternionF result;
    if(mag.isZero() || neta == 0.0f)
        return result;
    mag.normalize();

    //if(yaw_only)
    mag = removeMagneticVertical(mag);

    if(mag.isZero())
        return result;
    mag.normalize();

    TPoint3F dest_local;
    result = addGradientDescentStep(m_q, MAGNITUDE_VECTOR, mag, dest_local);
    if(!result.isZero())  {
        result.normalize();
        if(m_mag_correction_size < 1.0f)
            result *= m_mag_correction_size;
        compensateGyroDrift(result, TPoint3F(0, 0, m_gyro_avg.z), dt);
        result *= neta;
    }
    return result;
}

void TAHRSMadgwick::updateGyroAverage(TPoint3F& gyro, float dt)
{
#ifdef GYRO_COPNESATION_AVARAGE
    m_gyro_avg_f.put(gyro, dt);
    m_gyro_avg = m_gyro_avg_f() - m_gyro_error;
#endif
    gyro -= m_gyro_error;

#ifndef GYRO_COPNESATION_AVARAGE
    m_gyro_avg = gyro;
#else
    if(m_calibration_mode)
        m_gyro_avg = gyro;
#endif
}
//////////////////////////////////////////////////////////////////

void TAHRSMadgwick::update(TPoint3F gyro, TPoint3F acc, TPoint3F mag, float dt)
{    
    updateGyroAverage(gyro, dt);
    TQuaternionF q_dot;
    q_dot -= correctiveStepAccel(acc, dt);
    q_dot -= correctiveStepMag(mag, dt);
    q_dot += orientationChangeFromGyro(m_q, gyro);    
    changeOrientation(q_dot * dt);       
}

void TAHRSMadgwick::updateGyro(TPoint3F gyro, float dt) {
    updateGyroAverage(gyro, dt);
    TQuaternionF q_dot = orientationChangeFromGyro(m_q, gyro);
    changeOrientation(q_dot * dt);
}

void TAHRSMadgwick::updateAccMag(TPoint3F acc, TPoint3F mag, float dt) {
    TQuaternionF q_dot;
    q_dot -= correctiveStepAccel(acc, dt);
    q_dot -= correctiveStepMag(mag, dt);
    changeOrientation(q_dot * dt);
}


inline void TAHRSMadgwick::changeOrientation(const TQuaternionF& delta)
{
// TODO chek for NAN
    m_q += delta;
    m_q.normalize();    
}

void TAHRSMadgwick::setOrientation(const TQuaternionF& value) {
    m_q = value;
    m_q.normalize();
}
