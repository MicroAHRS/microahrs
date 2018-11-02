#include "TApplicationArduino.h"

#include <Wire.h>
#include <Adafruit_FXAS21002C_termo.h>
#include <Adafruit_FXOS8700.h>

#include "TAHRSMadgwick.h"
#include "shared/in_range.hpp"


TPoint3F mag_offset
(
    19.07,
    16.78 - 0.25,
    63.38 - 0.25
);

// Soft iron error compensation matrix
float mag_softiron_matrix[3][3] = { {  0.959, 0.002, 0.003 },
                                    {  0.002, 0.964, 0.008 },
                                    {  0.003, 0.008, 1.082 } };

TPoint3F gyro_zero_offset(
    -22612.24 * 0,
    -8122.33  * 0,
    3508.38 * 0
);

TPoint3F acc_zero_offset(
    0.3253305,
    -0.3593825,
    0.6121345
  );

TPoint3F acc_scale(
    1.01591586,
    0.9863349377,
    1.017566665
);


TApplicationArduino::TApplicationArduino() :
   m_device_gyro( Adafruit_FXAS21002C_termo(0x0021002C) ),
   m_device_accelmag( Adafruit_FXOS8700(0x8700A, 0x8700B))

{
    m_temperature = 0;
    m_print_out_timer = 0;
    m_tick_count = 0;
    m_last_update_time  = 0;
    m_is_started = false;
    m_light_enabled = false;

    m_device_gyro.setCalibrateFunction( Adafruit_FXAS21002C_termo::TFunctionCalibrate (
                TFunctionLineF(0.02923636286, -2.442451397), // kx kc
                TFunctionLineF(-0.00153622977, -0.512149624),
                TFunctionLineF(-0.006581737428, 0.4034881183)
                                            ));
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


bool TApplicationArduino::init()
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



inline TPoint3F TApplicationArduino::getMagn() {
    TPoint3F mag = VectorFromEvent(mag_event.magnetic) - mag_offset;
    //TODO use Matrix3
    float mx = mag.x * mag_softiron_matrix[0][0] + mag.y * mag_softiron_matrix[0][1] + mag.z * mag_softiron_matrix[0][2];
    float my = mag.x * mag_softiron_matrix[1][0] + mag.y * mag_softiron_matrix[1][1] + mag.z * mag_softiron_matrix[1][2];
    float mz = mag.x * mag_softiron_matrix[2][0] + mag.y * mag_softiron_matrix[2][1] + mag.z * mag_softiron_matrix[2][2];
    mag.x = mx;
    mag.y = my;
    mag.z = mz;
    return mag;
}
inline TPoint3F TApplicationArduino::getGyro() {
    return VectorFromEvent(gyro_event.gyro) - gyro_zero_offset;
}

inline TPoint3F TApplicationArduino::getAcc() {
    return (VectorFromEvent(accel_event.acceleration) - acc_zero_offset).scale(acc_scale);
}

void TApplicationArduino::update(float dt)
{
    updateDriftCoef(dt);
    updateAHRS(dt);

    const float PRINT_INTERVAL = 0.05;

    m_print_out_timer += dt;
    while(m_print_out_timer >= PRINT_INTERVAL ) {
        printOut();
        m_print_out_timer -= PRINT_INTERVAL;
    }
}

void TApplicationArduino::updateAHRS(float dt) {
    m_device_gyro.getEvent(&gyro_event,&gyro_t_event);
    m_device_accelmag.getEvent(&accel_event, &mag_event);
    m_temperature = gyro_t_event.temperature;

    TPoint3F mag = getMagn();
    TPoint3F acc = getAcc();
    TPoint3F gyr = getGyro();

    // mag_angle = atan2(mag.y ,mag.x ) * 180 / M_PI;

    m_ahrs.update(gyr,acc,mag,dt);
}

void TApplicationArduino::updateDriftCoef(float /*dt*/)
{
    m_ahrs.setGyroMeas(0.4, 0.0);
    //  if( ts + 10000 < 5000 ) {
    //    filter.setGyroMeas(40, 0);
    //  } else if( ts + 10000 < 6000 ) {
    //    float progress = GetProgressSection( ts , 0 , 5000);
    //    float beta = 40 * (1-progress) + 0.4 * progress;
    //    float zeta = 0 * (1-progress) + 0.1 * progress;
    //    filter.setGyroMeas(beta, zeta);
    //  }
}

void TApplicationArduino::turnLight(bool enabled) {
    if(m_light_enabled == enabled)
        return;
    digitalWrite(LED_BUILTIN, !enabled?LOW:HIGH);
    m_light_enabled = enabled;
}


void TApplicationArduino::setup()
{
    if(!init())
        while(1);
}

void TApplicationArduino::loop()
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


void TApplicationArduino::printOut() {
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
    Serial.print(" ");
    Serial.print(gyro_t_event.temperature);
    Serial.println();
}
