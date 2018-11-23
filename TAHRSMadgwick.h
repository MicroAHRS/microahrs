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

    float beta;     //for accelerometer and mag
    float zeta;     //for gyro compensation

    //float gama;

    TQuaternionF m_q;
    TPoint3F     m_gyro_error;            
    TPoint3F     m_north_local;
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
