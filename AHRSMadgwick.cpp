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

#include "AHRSMadgwick.h"
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

AHRSMadgwick::AHRSMadgwick() {
    setGyroMeas(gyroMeasError, gyroMeasDrift);
    m_q = TQuaternionf( 1.0f, 0, 0, 0);
    m_angles_computed = false;
}

void AHRSMadgwick::setGyroMeas(float error, float drift)
{
    beta = sqrt(3.0f / 4.0f) * (M_PI * (error / 180.0f));
    zeta = sqrt(3.0f / 4.0f) * (M_PI * (drift / 180.0f));

    beta  = Clamp(beta , 0.0f, 1.0f);
    zeta  = Clamp(zeta , 0.0f, 1.0f);
}

//-------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root


static float invSqrt(float x)
{
    return 1.0 / sqrt(x);
}

template<typename T>
static inline void normalizeVector(T& vx, T& vy, T& vz) {
    T recipNorm = invSqrt (vx * vx + vy * vy + vz * vz);
    vx *= recipNorm;
    vy *= recipNorm;
    vz *= recipNorm;
}

template<typename T>
static inline T VectorCrossProduct(T& v1x, T& v1y, T& v1z, T& v2x, T& v2y, T& v2z) {
    return v1x*v2x +  v1y*v2y + v1z*v2z;
}

template<typename T>
static inline bool isZeroVector(const T& vx, const T& vy,const T& vz) {
    return vx == 0.0f && vy == 0.0f && vz == 0.0f;
}

template<typename T>
static inline bool isZeroQuaternion(T& q0, T& q1, T& q2, T& q3) {
    return q0 == 0.0f && q1 == 0.0f && q2 == 0.0f && q3 == 0.0f;
}

template<typename T>
static inline void normalizeQuaternion(T& q0, T& q1, T& q2, T& q3) {
    T recipNorm = invSqrt (q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

static inline void rotateAndScaleVector(
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

/**
 *
 *
 *
*/

static inline TVector3f compensateMagneticDistortion(
        const TQuaternionf& q,
        const TVector3f& mag)
{
    // Reference direction of Earth's magnetic field (See EQ 46)
    TVector3f result = q.getInvert().rotateVector(mag);
    // получили вектор в системе Земли
    result.y = 0;
    result.x = (result.z == 1.0) ? 0 : sqrt(1.0 - (result.z * result.z));
    return result;
}


static inline TVector3f removeMagneticVertical(
        const TQuaternionf& q,
        const TVector3f& mag)
{
    // получили вектор в системе Земли
    TVector3f result = q.getInvert().rotateVector(mag);
    // убираем веритикаль
    result.z = 0;
    // обратно возвращаем в систему датчика
    return q.rotateVector(result);
}


static inline TQuaternionf orientationChangeFromGyro(
    const TQuaternionf& q,
    const TVector3f& gyro)
{
    // Rate of change of quaternion from gyroscope
    // See EQ 12    
    // Qdot = 0.5 * q * gyro
    return  q * 0.5f * TQuaternionf(gyro);
}



static TQuaternionf addGradientDescentStep(
    const TQuaternionf& q,     // текущая ориентация
    const TVector3f& dest,     // вектор в системе Земли целевой - длинна 2
    const TVector3f& source    // вектор в локальной систее - длинна 1
        )
{    
    TQuaternionf result;

    // Gradient decent algorithm corrective step
    // EQ 15, 21    
    TVector3f dest_local = q.rotateVector(dest);
    TVector3f delta = dest_local - source;
    TVector3f d2 = dest*2;

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
        const TQuaternionf& q,     // текущая ориентация
        const TQuaternionf& s,        // поправка кватрениона
        float dt,
        float zeta,
        TVector3f& gyro_err)
{   
    // w_err = 2 q x s
    if(zeta == 0)
        return;
    TVector3f err = (q * s).toVector3() * 2.0f;
    gyro_err +=  err * dt * zeta;
}


inline TQuaternionf CorrectiveStepAccel(const TQuaternionf& q,TVector3f acc)
{
    if(acc.isZero())
        return TQuaternionf();

    acc.normalize();
    const TVector3f earth_gravity(0.0f, 0.0f, 1.0f);
    return addGradientDescentStep(q, earth_gravity, acc);
}


inline TQuaternionf CorrectiveStepMag(const TQuaternionf& q,TVector3f mag, bool yaw_only)
{
    if(mag.isZero())
        return TQuaternionf();
    mag.normalize();

    if(yaw_only)
        mag = removeMagneticVertical(q, mag);

    if(mag.isZero())
        return TQuaternionf();
    mag.normalize();

    TVector3f mag_dest = (yaw_only) ? TVector3f(1.0f, 0.0f, 0.0f) : compensateMagneticDistortion(q, mag);
    return addGradientDescentStep(q, mag_dest, mag);
}


//////////////////////////////////////////////////////////////////

void AHRSMadgwick::update(const TVector3f& gyro, TVector3f acc, TVector3f mag, float dt)
{
    TQuaternionf q_dot;

    gyro -= m_gyro_error;
    q_dot = orientationChangeFromGyro(m_q, gyro);

    TQuaternionf s;
    s += CorrectiveStepAccel(m_q, acc);
    s += CorrectiveStepMag(m_q, mag, true);

    if(!s.isZero()) {
        s.normalize();
        q_dot -= s * beta;
        compensateGyroDrift(m_q, s, dt, zeta, m_gyro_error);
    }

    // Integrate rate of change of quaternion to yield quaternion
    m_q += q_dot * dt;

    m_q.normalize();
    m_angles_computed = 0;
}

//-------------------------------------------------------------------------------------------

void AHRSMadgwick::computeAngles()
{
    float roll = atan2f(m_q.w*m_q.x + m_q.y*m_q.z, 0.5f - m_q.x*m_q.x - m_q.y*m_q.y); // roll
    float pitch = asinf(-2.0f * (m_q.x*m_q.z - m_q.w*m_q.y));
    float yaw = atan2f(m_q.x*m_q.y + m_q.w*m_q.z, 0.5f - m_q.y*m_q.y - m_q.z*m_q.z);
    m_angles = TVector3f(roll, pitch, yaw);
    m_angles_computed = true;
}
