#ifndef ADAFRUIT_FXAS21002C_TERMO_H
#define ADAFRUIT_FXAS21002C_TERMO_H

#include "Adafruit_FXAS21002C.h"
#include "shared/Function/TFunctionLineF.h"
#include "shared/Function/TFunction3.h"

class Adafruit_FXAS21002C_termo: public Adafruit_FXAS21002C
{
public:
    typedef TFunction3< TFunctionLineF , float> TFunctionCalibrate;
    Adafruit_FXAS21002C_termo(int32_t sensorID = -1);

    inline void setCalibrateFunction(const TFunctionCalibrate& f)  { m_gyro_zero_avg = f; }

    bool getEvent( sensors_event_t* , sensors_event_t *temp_event );

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
