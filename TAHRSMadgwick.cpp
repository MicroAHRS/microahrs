//=============================================================================================
// Madgwick.c
//=============================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// From the x-io website "Open-source resources available on this website are
// provided under the GNU General Public Licence unless an alternative licence
// is provided in source."
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 19/02/2012	SOH Madgwick	Magnetometer measurement is normalised
//
//=============================================================================================


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

//============================================================================================
// Functions

//-------------------------------------------------------------------------------------------
// AHRS algorithm update

TAHRSMadgwick::TAHRSMadgwick() {
    setGyroMeas(gyroMeasError, gyroMeasDrift);
    m_q = TQuaternionF( 1.0f, 0, 0, 0);
    m_angles_computed = false;
}

void TAHRSMadgwick::setGyroMeas(float error, float drift)
{
    beta = sqrt(3.0f / 4.0f) * (M_PI * (error / 180.0f));
    zeta = sqrt(3.0f / 4.0f) * (M_PI * (drift / 180.0f));

    beta  = Clamp(beta , 0.0f, 1.0f);
    zeta  = Clamp(zeta , 0.0f, 1.0f);
}

static inline TPoint3F compensateMagneticDistortion(
        const TQuaternionF& q,
        const TPoint3F& mag)
{
    // Reference direction of Earth's magnetic field (See EQ 46)
    TPoint3F result = q.getInvert().rotateVector(mag);
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
    TPoint3F result = q.getInvert().rotateVector(mag);
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
    const TPoint3F& dest,     // вектор в системе Земли целевой - длинна 2
    const TPoint3F& source    // вектор в локальной систее - длинна 1
        )
{    
    TQuaternionF result;

    // Gradient decent algorithm corrective step
    // EQ 15, 21    
    TPoint3F dest_local = q.rotateVector(dest);
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


static inline void compensateGyroDrift(
        const TQuaternionF& q,     // текущая ориентация
        const TQuaternionF& s,        // поправка кватрениона
        float dt,
        float zeta,
        TPoint3F& gyro_err)
{   
    // w_err = 2 q x s
    if(zeta == 0)
        return;
    TPoint3F err = (q * s).toVector3() * 2.0f;
    gyro_err +=  err * dt * zeta;
}


inline TQuaternionF CorrectiveStepAccel(const TQuaternionF& q,TPoint3F acc)
{
    if(acc.isZero())
        return TQuaternionF();

    acc.normalize();
    const TPoint3F earth_gravity(0.0f, 0.0f, 1.0f);
    return addGradientDescentStep(q, earth_gravity, acc);
}


inline TQuaternionF CorrectiveStepMag(const TQuaternionF& q,TPoint3F mag, bool yaw_only)
{
    if(mag.isZero())
        return TQuaternionF();
    mag.normalize();

    if(yaw_only)
        mag = removeMagneticVertical(q, mag);

    if(mag.isZero())
        return TQuaternionF();
    mag.normalize();

    TPoint3F mag_dest = (yaw_only) ? TPoint3F(1.0f, 0.0f, 0.0f) : compensateMagneticDistortion(q, mag);
    return addGradientDescentStep(q, mag_dest, mag);
}


//////////////////////////////////////////////////////////////////

void TAHRSMadgwick::update(TPoint3F gyro, TPoint3F acc, TPoint3F mag, float dt)
{
    TQuaternionF q_dot;

    gyro -= m_gyro_avarage;
    q_dot = orientationChangeFromGyro(m_q, gyro);

    TQuaternionF s;
    s += CorrectiveStepAccel(m_q, acc);
    s += CorrectiveStepMag(m_q, mag, true);

    if(!s.isZero()) {
        s.normalize();
        q_dot -= s * beta;
        compensateGyroDrift(m_q, s, dt, zeta, m_gyro_avarage);
    }

    // Integrate rate of change of quaternion to yield quaternion
    m_q += q_dot * dt;
    m_q.normalize();
    m_angles_computed = false;
}

//-------------------------------------------------------------------------------------------

void TAHRSMadgwick::computeAngles()
{
    float roll  = atan2f(m_q.w*m_q.x + m_q.y*m_q.z, 0.5f - m_q.x*m_q.x - m_q.y*m_q.y); // roll
    float pitch = asinf(-2.0f * (m_q.x*m_q.z - m_q.w*m_q.y));
    float yaw   = atan2f(m_q.x*m_q.y + m_q.w*m_q.z, 0.5f - m_q.y*m_q.y - m_q.z*m_q.z);
    m_angles = TPoint3F(roll, pitch, yaw);
    m_angles_computed = true;
}
