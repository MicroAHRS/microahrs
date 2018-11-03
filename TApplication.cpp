#include "TApplication.h"


#include <Adafruit_FXAS21002C_termo.h>
#include <Adafruit_FXOS8700.h>

#include "TAHRSMadgwick.h"
//#include "shared/in_range.hpp"

#include <Wire.h>



TApplication::TApplication() :
   m_device_gyro( Adafruit_FXAS21002C_termo(0x0021002C) ),
   m_device_accelmag( Adafruit_FXOS8700(0x8700A, 0x8700B))

{
    m_temperature = 0;
    m_print_out_timer = 0;
    m_tick_count = 0;
    m_last_update_time  = 0;
    m_is_started = false;
    m_light_enabled = false;
    m_enable_accel_by_angle= true;

    m_device_gyro.setCalibrateFunction( m_settings.gyro_temperature );
}

inline TPoint3F VectorFromEvent(sensors_vec_t vec )
{
    return TPoint3F(vec.x, vec.y ,vec.z);
}

void PrintVector(const TPoint3F& vec) {
    Serial.print(vec.x);
    Serial.print(" ");
    Serial.print(vec.y);
    Serial.print(" ");
    Serial.print(vec.z);
    Serial.print(" ");
}


bool TApplication::init()
{
    Serial.begin(115200);

    // Initialize the sensors.
    if(!m_device_gyro.begin()) {
        Serial.println("Ooops, no gyro detected ... Check your wiring!");
        return false;
    }


    if(!m_device_accelmag.begin(ACCEL_RANGE_4G)) {
        Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");
        return false;
    }

    pinMode(LED_BUILTIN, OUTPUT);

    return true;
}



inline TPoint3F TApplication::getMagn() {
    TPoint3F mag_vec = VectorFromEvent(mag_event.magnetic) - m_settings.mag_offset;
    return m_settings.mag_matrix * mag_vec;
}
inline TPoint3F TApplication::getGyro() {
    return VectorFromEvent(gyro_event.gyro) - m_settings.gyro_zero_offset;
}

inline TPoint3F TApplication::getAcc() {
    return (VectorFromEvent(accel_event.acceleration) - m_settings.acc_zero_offset).scale(m_settings.acc_scale);
}

void TApplication::update(float dt)
{
    updateDriftCoefByTime(dt);
    updateAHRS(dt);

    const float PRINT_INTERVAL = m_settings.print_out_time_ms;

    m_print_out_timer += dt * 1000;
    while(m_print_out_timer >= PRINT_INTERVAL ) {
        printOut();
        receiveCmd();
        m_print_out_timer -= PRINT_INTERVAL;
    }
}
#include "shared/in_range.hpp"

void TApplication::updateAHRS(float dt) {
    m_device_gyro.getEvent(&gyro_event,&gyro_t_event);
    m_device_accelmag.getEvent(&accel_event, &mag_event);
    m_temperature = gyro_t_event.temperature;

    TPoint3F mag = getMagn();
    TPoint3F acc = getAcc();
    TPoint3F gyr = getGyro();

    if(!m_enable_accel_by_angle)
        acc = TPoint3F(0,0,0);

    if(!acc.isZero()) {
        float len = acc.length();
        if(!InRangeInc(len , m_settings.acc_min_length , m_settings.acc_max_length))
            acc = TPoint3F(0,0,0);
    }

    if(m_settings.disable_mag)
        mag = TPoint3F(0,0,0);
    if(m_settings.disable_acc)
        acc = TPoint3F(0,0,0);
    if(m_settings.disable_gyro)
        gyr = TPoint3F(0,0,0);

    m_ahrs.update(gyr,acc,mag,dt);
}

//#include "shared/in_range.hpp"
void TApplication::updateDriftCoefByTime(float /*dt*/)
{
    unsigned long ts = millis();

    if(ts >= m_settings.total_init_time_ms)
        return;

    if(ts < m_settings.beta_init_time_ms) {
        float progress = GetProgressSection(ts, 0, m_settings.beta_init_time_ms);
        progress = 0;
        float b = m_settings.beta_start * (1- progress) + m_settings.beta * progress;
        m_ahrs.setGyroMeas(b, 0);
    } else if(ts < m_settings.zeta_init_time_ms) {
        float progress = GetProgressSection(ts, m_settings.beta_init_time_ms , m_settings.zeta_init_time_ms);
        progress = 0;
        float z = m_settings.zeta_start * (1- progress)  + m_settings.zeta * progress;
        m_ahrs.setGyroMeas(m_settings.beta, z);
    } else {
        m_ahrs.setGyroMeas(m_settings.beta, m_settings.zeta);
    }
}

void TApplication::turnLight(bool enabled) {
    if(m_light_enabled == enabled)
        return;
    digitalWrite(LED_BUILTIN, !enabled?LOW:HIGH);
    m_light_enabled = enabled;
}


void TApplication::setup()
{
    if(!init())
        while(1);
}

void TApplication::loop()
{
    unsigned long ts = millis();
    float dt = ((ts - m_last_update_time) / 1000.0);
    if(dt < 0.01) {
      delay(1);
      return;
    }
    unsigned long seconds_count = int(ts / 1000);

    m_last_update_time = ts;
    update(dt);
    m_tick_count++;
    turnLight(seconds_count % 2 == 0 || !m_is_started);
}


void TApplication::printOut() {
    float roll    = m_ahrs.getRoll();
    float pitch   = m_ahrs.getPitch();
    float heading = m_ahrs.getYaw();
    Serial.print("Orientation: ");
    Serial.print(heading);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.print(roll);
    //Serial.print(" ");
    //Serial.print(tick_count / ((float)millis() / 1000) );
//    Serial.print(" ");
//    Serial.print(gyro_t_event.temperature);
    TPoint3F acc = getAcc();
    float len =0;
    if(!acc.isZero())
        len = acc.length();

    Serial.print(" ");
    Serial.print(len);

    Serial.println();

    m_enable_accel_by_angle = fabs(pitch) < m_settings.pitch_max && fabs(roll) < m_settings.roll_max;
}


enum ECOmmandCode {
    E_CMD_CODE_NONE = 0,
    E_CMD_CODE_RESET_PITCH_ROLL = 1,
    E_CMD_CODE_RESET_YAW_BY_MAG = 2,
};

void TApplication::onCommandResetPitchRoll() {
    m_ahrs.resetPitchRoll();
}

void TApplication::onCommandResetYawByMag()
{
    m_ahrs.setYawByMagnetometer(getMagn());
    //m_ahrs.setGyroAvarage(getGyro());
}

void TApplication::receiveCmd() {
    if (!Serial.available())
        return;

    byte b = Serial.read();

    if(b==E_CMD_CODE_RESET_PITCH_ROLL)
        onCommandResetPitchRoll();

    if(b==E_CMD_CODE_RESET_YAW_BY_MAG)
        onCommandResetYawByMag();

}



































