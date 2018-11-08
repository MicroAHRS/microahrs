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
    m_ahrs.setGyroError(m_settings.gyro_zero_offset);
    m_ahrs.setGyroMeas(m_settings.beta, m_settings.zeta);
    m_device_gyro.setCalibrateFunction( m_settings.gyro_temperature );
}

inline TPoint3F VectorFromEvent(sensors_vec_t vec ) {
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

void PrintQuaternion(const TQuaternionF& q) {
    Serial.print(q.w);
    Serial.print(" ");
    Serial.print(q.x);
    Serial.print(" ");
    Serial.print(q.y);
    Serial.print(" ");
    Serial.print(q.z);
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

    //onCommandBoostFilter();

    return true;
}



inline TPoint3F TApplication::getMagn() {
    return TPoint3F(1,0.0001,0.0001);
    TPoint3F mag_vec = VectorFromEvent(mag_event.magnetic) - m_settings.mag_offset;
    return m_settings.mag_matrix * mag_vec;
}
inline TPoint3F TApplication::getGyro() {    
    return VectorFromEvent(gyro_event.gyro);
}
inline float TApplication::getTemperature() {
    return gyro_t_event.temperature;
}

inline TPoint3F TApplication::getAcc() {
    return TPoint3F(-0.001,-0.001,1.01);
    return (VectorFromEvent(accel_event.acceleration) - m_settings.acc_zero_offset).scale(m_settings.acc_scale);
}

void TApplication::update(float dt)
{
    updateDevices();    
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
void TApplication::updateDevices() {
    m_device_gyro.getEvent(&gyro_event,&gyro_t_event);
    m_device_accelmag.getEvent(&accel_event, &mag_event);
    m_temperature = gyro_t_event.temperature;
}


void TApplication::updateAHRS(float dt) {
    TPoint3F mag = getMagn();
    TPoint3F acc = getAcc();
    TPoint3F gyr = getGyro();

//    if(!m_enable_accel_by_angle)
//        acc = TPoint3F(0,0,0);

    if(m_settings.disable_mag)
        mag = TPoint3F(0,0,0);
    if(m_settings.disable_acc)
        acc = TPoint3F(0,0,0);
    if(m_settings.disable_gyro)
        gyr = TPoint3F(0,0,0);


    m_ahrs.update(gyr,acc,mag,dt);
}

void TApplication::updateDriftCoefByAngles(const TPoint3F &angles, float acc_len_square)
{
    // if pitch or roll too big
    // set beta to smoler value

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

    if(dt > 1)
       return;

    update(dt);
    m_tick_count++;
    turnLight(seconds_count % 2 == 0 || !m_is_started);
}


void TApplication::printOut() {
    TPoint3F angles = m_ahrs.getAngles();
    Serial.print("Orient: ");
    PrintVector(angles * CONVERT_RAD_TO_DPS);

    Serial.print("acc: ");
    TPoint3F acc = getAcc();
    PrintVector(acc);

    float temp = getTemperature();
    Serial.print("t: ");
    Serial.print(int(temp));

    Serial.print(" gerr: ");
    PrintVector(m_ahrs.m_gyro_error * CONVERT_RAD_TO_DPS * 1000);
    Serial.println();

    //Serial.print("q_dor: ");
    //PrintQuaternion(m_ahrs.m_q_dot * 1000);
//    Serial.print("zeta: ");
//    Serial.print((m_ahrs.zeta * CONVERT_RAD_TO_DPS) / sqrt(3.0f / 4.0f) );



    updateDriftCoefByAngles(angles, acc.lengthSq());
}



float GetDeltaTime(unsigned long& last_time, float min_dt)
{           
    unsigned long ts = millis();
    if(last_time == 0) {
        last_time = ts;
        return 0;
    }

    float dt = ((ts - last_time) / 1000.0);
    if(dt < min_dt)
        return 0;
    last_time = ts;
    return dt;
}

void TApplication::onCommandResetPitchRoll() {
    Serial.println("Reset pitch and roll");
    TPoint3F angles = m_ahrs.getAngles();
    m_ahrs.setOrientation(TQuaternionF::CreateFormAngles(0,0,angles.z));
}

void TApplication::onCommandSetYawByMag() {       
    Serial.println("Set yaw by mag");
    TPoint3F angles = m_ahrs.getAngles();
    TPoint3F mag =getMagn();
    float yaw = atan2(-mag.y, mag.x);
    m_ahrs.setOrientation(TQuaternionF::CreateFormAngles(angles.x,angles.y,yaw));
}

void TApplication::onCommandSetPitchRollByAcc() {
    //TODO
}

void TApplication::onCommandSetGravityVector() {
    Serial.println("Set gravity vector");
    m_ahrs.setGravity(getAcc());
    onCommandBoostFilter();
}

void TApplication::onCommandSetMagnitudeVector() {
    Serial.println("Set magnitude vector");
    TPoint3F magn = getMagn();
    magn.z = 0;
    m_ahrs.setMagnitude(magn);
    onCommandBoostFilter();
}

void TApplication::onCommandCalibrateGyro()
{
    Serial.println("Start calibrate gyro. Dont move 30 seconds!");
    CalibrateGyroStep1(60);
    Serial.print("Done ");
    PrintVector(m_settings.gyro_zero_offset * CONVERT_RAD_TO_DPS);
    Serial.println();

    Serial.println("Start descover drifting. Dont move 30 seconds!");
    CalibrateGyroStep2(30);
    Serial.print("Done. beta x 1000 = ");
    Serial.print(m_settings.beta * 1000);
    Serial.print("zeta x 1000 = ");
    Serial.print(m_settings.zeta * 1000);
    Serial.println();

    m_ahrs.setGyroError(m_settings.gyro_zero_offset);
    m_ahrs.setGyroMeas(m_settings.beta, m_settings.zeta);
}

void TApplication::CalibrateGyroStep1(float max_time)
{        
    float time = 0;    
    float MIN_DT = 0.01;
    unsigned long last_time = 0;
    TPoint3F gravity (0,0,1);
    TPoint3F mag (1,0,0);
    m_ahrs.setOrientation(TQuaternionF(1.0f, 0.0, 0.0, 0.0));
    m_ahrs.setGyroMeas(0.2, 0.1);

    while(time < max_time) {
        delay(1);
        float dt = GetDeltaTime(last_time, MIN_DT);
        if(dt == 0)
            continue;

        time += dt;
        updateDevices();        
        TPoint3F gyr = getGyro();
        m_ahrs.update(gyr, gravity, mag, dt);
    }

    m_settings.gyro_zero_offset  = m_ahrs.m_gyro_error;
}

#include "shared/max.hpp"

void TApplication::CalibrateGyroStep2(float max_time)
{
//    m_settings.beta =  0;
//    m_settings.zeta =  0;
}


void TApplication::onCommandBoostFilter() {
    Serial.println("Start boost filter");
    //ускорить фильтр - чтобы выровнить ориентацию
    m_ahrs.setGyroMeas(m_settings.beta_start, m_settings.zeta_start);
    TPoint3F gyr(0,0,0);
    TPoint3F acc = getAcc();
    TPoint3F mag = getMagn();
    float dt = 0.01;
    for(unsigned long i = 0;i<5000;i++)
        m_ahrs.update(gyr, acc,mag, dt);    

    m_ahrs.setGyroMeas(m_settings.beta,m_settings.zeta);
    Serial.println("Done");
}



void TApplication::onCommandDebugAction()
{
    Serial.println("Debug action");
    static int roll = 0;

    roll += 45;
    if(roll >= 90)
        roll = -45;
    //m_ahrs.setGyroError(TPoint3F(0,0,0));
    m_ahrs.setOrientation(TQuaternionF::CreateFormAngles(roll * CONVERT_DPS_TO_RAD, 0, 0));
}


void ToggleFlag(bool& flag, const char* name)
{
    flag = !flag;
    Serial.print(name);
    Serial.println(!flag ? " enabled": " disabled");
}

void TApplication::receiveCmd() {
    if (!Serial.available())
        return;

    byte b = Serial.read();
    switch (b) {
    case E_CMD_CODE_RESET_PITCH_ROLL:
        return onCommandResetPitchRoll();

    case E_CMD_CODE_SET_YAW_BY_MAG:
        return onCommandSetYawByMag();

    case E_CMD_CODE_SET_PITCH_ROLL_BY_ACC:
        return onCommandSetPitchRollByAcc();

    case E_CMD_CODE_BOOST_FILTER:
        return onCommandBoostFilter();

    case E_CMD_CODE_CALIBRATE_GYRO:
        return onCommandCalibrateGyro();

    case E_CMD_CODE_SET_GRAVITY_VECTOR:
        return onCommandSetGravityVector();

    case E_CMD_CODE_SET_YAW_NORTH:
        return onCommandSetMagnitudeVector();

    case E_CMD_CODE_DEBUG_ACTION:
        return onCommandDebugAction();

    case E_CMD_CODE_TOGGLE_GYRO:
        ToggleFlag(m_settings.disable_gyro, "Gyro");
        m_ahrs.setGyroMeas(m_settings.beta, m_settings.disable_gyro? 0 : m_settings.zeta);
        m_ahrs.setGyroError( m_settings.disable_gyro ? TPoint3F(0,0,0) : m_settings.gyro_zero_offset) ;
        break;
    case E_CMD_CODE_TOGGLE_ACC:
        ToggleFlag(m_settings.disable_acc, "Accel");
        break;
    case E_CMD_CODE_TOGGLE_MAG:
        ToggleFlag(m_settings.disable_mag, "Magnit");
        break;
    }
}



































