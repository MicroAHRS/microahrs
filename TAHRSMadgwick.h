#ifndef __Madgwick2_h__
#define __Madgwick2_h__

#include "shared/Geometry/TPoint3F.h"
#include "shared/Geometry/TQuaternionF.h"
#include "shared/Function/TFunctionLimitedAverage.h"

#define CONVERT_RAD_TO_DPS 57.29578f
#define CONVERT_DPS_TO_RAD 0.017453293f
//--------------------------------------------------------------------------------------------
// Variable declaration

// Some GYRO COMPESATION TECHNICES

//#define GYRO_COPNESATION_AVARAGE
#define GYRO_COPNESATION_TIME
//#define GYRO_COPNESATION_SLOWDOWN

#define BETTA_COEF  0.0058134597      // sqrt(3/4) / 2.6 * CONVERT_DPS_TO_RAD , where 2.6 experimental coef

class TAHRSMadgwick{
public:

    float neta;     //for magnitometer
    float beta;     //for accelerometer
    float zeta;     //for gyro compensation

    //float gama;

    TQuaternionF m_q;
    TPoint3F     m_gyro_error;            
    TPoint3F     m_mag_horisontal;
    TPoint3F     m_gyro_avg;

#ifdef GYRO_COPNESATION_AVARAGE
    TFunctionLimitedAverage<TPoint3F, float , 4>  m_gyro_avg_f;
#endif


    TPoint3F    m_gyro_err_switch_time;
    TPoint3F    m_gyro_err_direction;

public:
    TAHRSMadgwick();

    void setZetaMaxAngle(float value);
    void setGyroMeas(float error, float drift, float error_mag);
    void update(TPoint3F gyro, TPoint3F acc, TPoint3F mag, float dt);

    inline TPoint3F getAngles() const { return m_q.getAngles();}
    inline void setGyroError(const TPoint3F& vec) { m_gyro_error = vec;}
    void setOrientation(const TQuaternionF& value);

protected:        
    inline void changeOrientation(const TQuaternionF& delta);
    void compensateGyroDrift( const TQuaternionF& s, const TPoint3F& gyro, const float& dt);
    TQuaternionF correctiveStepAccel(TPoint3F& acc, const float &dt);
    TQuaternionF correctiveStepMag(TPoint3F& mag, const float &dt);
    inline TPoint3F removeMagneticVertical(const TPoint3F& mag);
    inline void updateGyroAverage(TPoint3F &gyro, float dt);

};
#endif
