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
#include <math.h>
#include "shared/TVector3.h"
#include "shared/TQuaternion.h"

//--------------------------------------------------------------------------------------------
// Variable declaration
class AHRSMadgwick{
public:
    float beta;
    float zeta;

    TQuaternionf m_q;
    TVector3f    m_gyro_error;

    TVector3f    m_angles; //(roll,pitch,yaw)
    bool         m_angles_computed;
public:
    AHRSMadgwick(void);


    void setGyroMeas(float error, float drift);
    void update(const TVector3f& gyro, TVector3f acc, TVector3f mag, float dt);

    const TVector3f& getAngles() { if (!m_angles_computed) {computeAngles();} return m_angles; }
    inline float getRoll()  { return getAngles().x * 57.29578f;}
    inline float getPitch() { return getAngles().y * 57.29578f;}
    inline float getYaw()   { return getAngles().z * 57.29578f  + 180;}
protected:
    void computeAngles();

};
#endif
