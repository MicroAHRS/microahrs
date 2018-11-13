#ifndef TAPPLICATIONARDUINO_H
#define TAPPLICATIONARDUINO_H

#include <Adafruit_FXAS21002C_termo.h>
#include <Adafruit_FXOS8700.h>
#include "shared/Geometry/TPoint3F.h"
#include "TAHRSMadgwick.h"
#include "TAppSettings.h"
#include "ECommandCode.h"

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

    inline TPoint3F getMagn();
    inline TPoint3F getGyro();
    inline TPoint3F getAcc();
    inline float getTemperature();

    inline void onCommandResetPitchRoll();
    inline void onCommandSetYawByMag();
    inline void onCommandSetPitchRollByAcc();
    void onCommandBoostFilter();
    inline void onCommandCalibrateGyro();
    inline void onCommandSetGravityVector();
    inline void onCommandSetMagnitudeVector();
    void onCommandLoad();
    inline void onCommandSave();
    inline void onCommandSetMagnitudeMatrix();
    inline void onCommandDebugAction();

    void receiveCmd();

    void CalibrateGyroCycle(float beta_start, float beta_end, float max_time);
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
    sensors_event_t gyro_event;
    sensors_event_t gyro_t_event;
    sensors_event_t accel_event;
    sensors_event_t mag_event;

    TAHRSMadgwick                m_ahrs;
    Adafruit_FXAS21002C_termo    m_device_gyro;
    Adafruit_FXOS8700            m_device_accelmag;
    TAppSettings                 m_settings;
};

#endif // TAPPLICATIONARDUINO_H
