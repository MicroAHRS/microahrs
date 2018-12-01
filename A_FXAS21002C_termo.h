#ifndef ADAFRUIT_FXAS21002C_TERMO_H
#define ADAFRUIT_FXAS21002C_TERMO_H

#include "A_FXAS21002C.h"
//#include "shared/Function/TFunctionLineF.h"
#include "shared/Function/TFunctionLinePoint3F.h"

class A_FXAS21002C_termo: public A_FXAS21002C
{
public:
    typedef TFunctionLinePoint3F TFunctionCalibrate;
    A_FXAS21002C_termo();

    inline void setCalibrateFunction(const TFunctionCalibrate& f)  { m_gyro_zero_avg = f; }

    bool getGyro         ( TPoint3F& gyro, float& temp );

    /*
     * Set linear function represent gyro zero output ( temperature)
     *  degrees and celsius
     */
    inline void setEnableCompensation(bool val) {m_enable_conpensation = val;}
private:

   TFunctionCalibrate m_gyro_zero_avg;
   bool               m_enable_conpensation;
};

#endif // ADAFRUIT_FXAS21002C_TERMO_H
