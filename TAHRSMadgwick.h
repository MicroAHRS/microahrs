//=============================================================================================
// Madgwick.h
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
//
//=============================================================================================
#ifndef __Madgwick2_h__
#define __Madgwick2_h__

#include "shared/Geometry/TPoint3F.h"
#include "shared/Geometry/TQuaternionF.h"

#define CONVERT_RAD_TO_DPS 57.29578f
#define CONVERT_DPS_TO_RAD 0.017453293f
//--------------------------------------------------------------------------------------------
// Variable declaration
class TAHRSMadgwick{
public:
    float beta;
    float zeta;

    TQuaternionF m_q;
    TPoint3F     m_gyro_error;
    TPoint3F     m_angles;      //(roll,pitch,yaw)
//    TPoint3F     m_gravity;     // direction of gravity force destination
//    TPoint3F     m_magnitude;   // direction of magnit destination

    bool         m_angles_computed;
public:
    TAHRSMadgwick();

    void setGyroMeas(float error, float drift);
    void update(TPoint3F gyro, TPoint3F acc, TPoint3F mag, float dt);

    const TPoint3F& getAngles() { if (!m_angles_computed) {computeAngles();} return m_angles; }
    inline float getRoll()  { return getAngles().x * CONVERT_RAD_TO_DPS;}
    inline float getPitch() { return getAngles().y * CONVERT_RAD_TO_DPS;}
    inline float getYaw()   { return getAngles().z * CONVERT_RAD_TO_DPS;}

    inline void setGyroError(const TPoint3F& vec) { m_gyro_error = vec;}
//    inline void setGravity(const TPoint3F& vec) { m_gravity = vec; m_gravity.normalize();}
//    inline void setMagnitude(const TPoint3F& vec) { m_magnitude = vec; m_magnitude.normalize();}
    void setOrientation(const TQuaternionF& value);

protected:
    void computeAngles();
    void changeOrientation(const TQuaternionF& delta);

};
#endif
