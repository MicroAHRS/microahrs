#ifndef TAPPLICATIONARDUINO_H
#define TAPPLICATIONARDUINO_H

//#include <Adafruit_FXAS21002C_termo.h>

#include "shared/Geometry/TPoint3F.h"

#include "A_Sensor.h"



// 1.01
// 1.02 поменялись коды комманды
// 1.03 Reset Pitch Roll с учетом ориентации сенсора
// 1.04 отправка сырых даных магнитометра
// 1.05 ускорение фильтра за одну итерацию - при помощи рассчета углов

#define AHRS_VERSION "1.05"

class TAppSettings;
class A_FXAS21002C_termo;
class A_FXAS21002C;
class A_FXOS8700;
class TAHRSMadgwick;

class TApplication
{
public:
    TApplication();
    void setup();
    void loop();

protected:
    bool init();
    void update(float dt);
    void updateDevices() ;
    void updateAHRS(float dt);    
    void updateDriftCoefByAngles();
    //void turnLight(bool enabled);
    void printOut();

    TPoint3F getAcc();
    TPoint3F getMagn();
    TPoint3F getGyro() {return m_gyro_value;}
    float getTemperature() {return m_temperature;}

    inline void onCommandResetPitchRoll();
    inline void onCommandSetYawByMag();
    inline void onCommandSetPitchRollByAcc();
    void        onCommandBoostFilter();
    inline void onCommandCalibrateGyro();
    inline void onCommandSetGravityVector();
    inline void onCommandSetMagnitudeVector();
    void        onCommandLoad();
    inline void onCommandSave();
    inline void onCommandSetMagnitudeMatrix();
    inline void onCommandDebugAction();
    inline void onChangeGyroRange();
    inline void onChangeAccRange();

    void receiveCmd();

    void CalibrateGyroCycle(float beta_start, float beta_end, float max_time, bool verbose);
    void CalibrateGyroStep1(float max_time);
    void onSettingsChanged();


private:
    unsigned long   m_fps;
    unsigned long   m_tick_count;
    unsigned long   m_last_update_time;
    unsigned long   m_seconds_count;
    //bool            m_light_enabled;
    unsigned int    m_print_out_timer;

    float           m_temperature;
    TPoint3F        m_gyro_value;
    TPoint3F        m_acc_value;
    TPoint3F        m_mag_value;

    TAHRSMadgwick*                m_ahrs;
    //Adafruit_FXAS21002C_termo    m_device_gyro;
    A_FXAS21002C_termo*    m_device_gyro;
    A_FXOS8700*            m_device_accelmag;
    TAppSettings*                 m_settings;
};

#endif // TAPPLICATIONARDUINO_H
