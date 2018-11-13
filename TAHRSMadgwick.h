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
    TPoint3F     m_angle_dest;
    float        m_zeta_max_angle;
public:
    TAHRSMadgwick();

    void setZetaMaxAngle(float value);
    void setGyroMeas(float error, float drift);
    void update(TPoint3F gyro, TPoint3F acc, TPoint3F mag, float dt);
    inline TPoint3F getAngles() const { return m_q.getAngles();}
    inline void setGyroError(const TPoint3F& vec) { m_gyro_error = vec;}
    void setOrientation(const TQuaternionF& value);

protected:        
    inline void changeOrientation(const TQuaternionF& delta);
    inline void compensateGyroDrift( const TQuaternionF& s, const TPoint3F& gyro, float dt);
    inline TQuaternionF correctiveStepAccel(TPoint3F& acc);
    inline TQuaternionF correctiveStepMag(TPoint3F& mag, bool yaw_only);

};
#endif
