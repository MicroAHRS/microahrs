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

//--------------------------------------------------------------------------------------------
// Variable declaration
class TAHRSMadgwick{
public:
    float beta;
    float zeta;

    TQuaternionF m_q;
    TPoint3F     m_gyro_avarage;
    TPoint3F     m_angles; //(roll,pitch,yaw)
    bool         m_angles_computed;
public:
    TAHRSMadgwick();

    void setGyroMeas(float error, float drift);
    void update(TPoint3F gyro, TPoint3F acc, TPoint3F mag, float dt);

    const TPoint3F& getAngles() { if (!m_angles_computed) {computeAngles();} return m_angles; }
    inline float getRoll()  { return getAngles().x * 57.29578f;}
    inline float getPitch() { return getAngles().y * 57.29578f;}
    inline float getYaw()   { return getAngles().z * 57.29578f;}

    void resetPitchRoll();
    void setRollPitchByAccelerometer(const TPoint3F& acc);
    void setYawByMagnetometer(const TPoint3F& mag);

protected:
    void computeAngles();
};
#endif
