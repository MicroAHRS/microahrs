#ifndef TAPPLICATIONARDUINO_H
#define TAPPLICATIONARDUINO_H

#include <Adafruit_FXAS21002C_termo.h>
#include <Adafruit_FXOS8700.h>
#include "shared/Geometry/TPoint3F.h"
#include "TAHRSMadgwick.h"
#include "TAppSettings.h"

class TApplication
{
public:
    TApplication();
    void setup();
    void loop();

protected:
    bool init();
    void update(float dt);
    void updateAHRS(float dt);
    void updateDriftCoefByTime(float dt);
    void turnLight(bool enabled);
    void printOut();

    inline TPoint3F getMagn();
    inline TPoint3F getGyro();
    inline TPoint3F getAcc();

    void onCommandResetPitchRoll();
    void onCommandResetYawByMag();
    void receiveCmd();

private:
    unsigned long   m_tick_count;
    unsigned long   m_last_update_time;
    bool            m_is_started;
    bool            m_light_enabled;
    unsigned int    m_print_out_timer;

    bool            m_enable_accel_by_angle;

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
