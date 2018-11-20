#include "TApplication.h"

#include "ECommandCode.h"


#include "TAHRSMadgwick.h"
#include "TAppSettings.h"


#include "Adafruit_FXOS8700.h"
#include "Adafruit_FXAS21002C.h"
#include "Adafruit_FXAS21002C_termo.h"


#include "Arduino.h"

#define FLOAT_ACCURENCY 1000000.f

const char* MSG_MAGNIT = "Mag";
const char* MSG_ACCEL = "Acc";
const char* MSG_GYRO = "Gyro";
const char* MSG_DONE = "Done";
const char* MSG_SUCCESS = "success";
const char* MSG_FAIL   = "FAIL";
const float FLOAT_FACKTOR = 1000;
TApplication::TApplication()   
{
    m_temperature = 0;
    m_print_out_timer = 0;
    m_tick_count = 0;
    m_last_update_time  = 0;    
    //m_light_enabled = false;
    m_seconds_count = 0;
    m_fps = 0;
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

//void PrintVectorI(const TPoint3F& vec) {
//    Serial.print(int(vec.x));
//    Serial.print(" ");
//    Serial.print(int(vec.y));
//    Serial.print(" ");
//    Serial.print(int(vec.z));
//    Serial.print(" ");
//}
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
    Serial.println();

    m_ahrs = new TAHRSMadgwick();
    m_settings = new TAppSettings();
    m_device_gyro = new Adafruit_FXAS21002C_termo(0x0021002C);
    m_device_accelmag = new Adafruit_FXOS8700(0x8700A, 0x8700B);

    onCommandLoad();

    // Initialize the sensors.
    if(!m_device_gyro->begin(m_settings->gyro_mode)) {
        Serial.println("Ooops, no gyro detected ... Check your wiring!");
        return false;
    }

    if(!m_device_accelmag->begin(m_settings->acc_mode)) {
    //if(!m_device_accelmag->begin(Adafruit_FXOS8700::ACCEL_RANGE_4G)) {
        Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");
        return false;
    }

    //pinMode(LED_BUILTIN, OUTPUT);

    if(m_settings->m_save_count == 0)
        return true;

    delay(100);
    //updateDevices();
    //onCommandBoostFilter();

    Serial.print("Welcome! AHRS Ver ");
    Serial.println(AHRS_VERSION);

    return true;
}



inline TPoint3F TApplication::getMagn() {
    TPoint3F mag_vec = VectorFromEvent(mag_event.magnetic) - m_settings->mag_offset;
    return m_settings->mag_matrix * mag_vec;
}
inline TPoint3F TApplication::getGyro() {    
    return VectorFromEvent(gyro_event.gyro);
}
inline float TApplication::getTemperature() {
    return gyro_t_event.temperature;
}

inline TPoint3F TApplication::getAcc() {    
    return (VectorFromEvent(accel_event.acceleration) - m_settings->acc_zero_offset).scale(m_settings->acc_scale);
}

void TApplication::update(float dt)
{   
    updateDevices();    
    updateAHRS(dt);

    m_print_out_timer += dt * 1000;
    while(m_print_out_timer >= PRINTOUT_TIME_MS ) {
        printOut();
        updateDriftCoefByAngles();
        receiveCmd();
        m_print_out_timer -= PRINTOUT_TIME_MS;
    }
}
#include "shared/in_range.hpp"
void TApplication::updateDevices() {
    m_device_gyro->getEvent(&gyro_event,&gyro_t_event);
    m_device_accelmag->getEvent(&accel_event, &mag_event);
    m_temperature = gyro_t_event.temperature;
}


void TApplication::updateAHRS(float dt) {
    TPoint3F mag = getMagn();
    TPoint3F acc = getAcc();
    TPoint3F gyr = getGyro();

//    if(!m_enable_accel_by_angle)
//        acc = TPoint3F(0,0,0);

    if(m_settings->disable_mag)
        mag = TPoint3F(0,0,0);
    if(m_settings->disable_acc)
        acc = TPoint3F(0,0,0);
    if(m_settings->disable_gyro)
        gyr = TPoint3F(0,0,0);


    m_ahrs->update(gyr,acc,mag,dt);
}

void TApplication::updateDriftCoefByAngles()
{
    float acc_len_square = getAcc().lengthSq();
    // if pitch or roll too big
    // set beta to smoler value    
    if(!InRangeInc(acc_len_square, m_settings->acc_min_length_sq, m_settings->acc_max_length_sq )) {
        m_ahrs->setGyroMeas(0,0);
        return;
    }

    float beta = m_settings->beta;
    float zeta = m_settings->zeta;

    // TPoint3F angles = m_ahrs->getAngles(); // USE modidfed angles
    // TODO change beta zeta by angle

    m_ahrs->setGyroMeas(beta, zeta);

}

//void TApplication::turnLight(bool enabled) {
//    if(m_light_enabled == enabled)
//        return;
//    digitalWrite(LED_BUILTIN, !enabled?LOW:HIGH);
//    m_light_enabled = enabled;
//}


void TApplication::setup()
{
    delay(1000);

    if(!init())
        while(1);

    //m_start_time = millis();
}

void TApplication::loop()
{
    unsigned long ts = millis();
    float dt = ((ts - m_last_update_time) / 1000.0);
    if(dt < 0.01) {
      delay(1);
      return;
    }

    m_last_update_time = ts;

    if(dt > 1)
       return;

    unsigned long seconds_count = int(ts / 1000);
    if(m_seconds_count != seconds_count) {
        m_fps = m_tick_count / 1.0;
        m_tick_count = 0;
        m_seconds_count = seconds_count;
    }
    update(dt);
    m_tick_count++;
    //turnLight(seconds_count % 2 == 0);
}



void TApplication::printOut() {
    TQuaternionF q = m_ahrs->m_q * m_settings->sensor_to_frame_orientation;
    TPoint3F angles = q.getAngles();
    //TPoint3F angles = m_ahrs->getAngles();
    Serial.print("Orient: ");
    PrintVector(angles * CONVERT_RAD_TO_DPS);

    Serial.print("acc: ");
    TPoint3F acc = getAcc();
    PrintVector(acc);

//    Serial.println(" ");
//    Serial.println(acc.x * FLOAT_ACCURENCY);
//    Serial.println(acc.y * FLOAT_ACCURENCY);
//    Serial.println(acc.z * FLOAT_ACCURENCY);

    float temp = getTemperature();    
    Serial.print("t: ");
    Serial.print(int(temp));

    if(m_settings->print_mag) {
        TPoint3F mag_raw = VectorFromEvent(mag_event.magnetic);
        Serial.print(" gerr: ");
        PrintVector(m_ahrs->m_gyro_error * CONVERT_RAD_TO_DPS * FLOAT_FACKTOR);

        Serial.print(m_ahrs->beta * FLOAT_FACKTOR * CONVERT_RAD_TO_DPS / 0.866025404);
        Serial.print(" ");
        Serial.print(m_ahrs->zeta * FLOAT_FACKTOR * CONVERT_RAD_TO_DPS / 0.866025404);
        Serial.print(" ");
        Serial.print("mag: ");
        TPoint3F mag = getMagn();
        PrintVector(mag);
        Serial.print("fps: ");
        Serial.print(m_fps);

        Serial.print(" mag_raw: ");
        PrintVector(mag_raw * FLOAT_FACKTOR );
    }
    Serial.println();

    //Serial.print("q_dor: ");
    //PrintQuaternion(m_ahrs->m_q_dot * 1000);
//    Serial.print("zeta: ");
//    Serial.print((m_ahrs->zeta * CONVERT_RAD_TO_DPS) / sqrt(3.0f / 4.0f) );
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
    //Serial.println("Reset pitch and roll");
    TPoint3F angles = m_ahrs->getAngles();
    TPoint3F angles_def = m_settings->sensor_to_frame_orientation.getAngles();
    m_ahrs->setOrientation(TQuaternionF::CreateFormAngles(-angles_def.x,-angles_def.y,angles.z));
}

void TApplication::onCommandSetYawByMag() {       
    //Serial.println("Set yaw by mag");
    TPoint3F angles = m_ahrs->getAngles();
    TPoint3F mag = getMagn();
    float yaw = atan2(-mag.y, mag.x);
    m_ahrs->setOrientation(TQuaternionF::CreateFormAngles(angles.x,angles.y,yaw));
}

void TApplication::onCommandSetPitchRollByAcc() {
    //TODO
}

void TApplication::onCommandSetGravityVector() {

    TPoint3F angles = (m_ahrs->m_q * m_settings->sensor_to_frame_orientation).getAngles();
    m_settings->sensor_to_frame_orientation *= TQuaternionF::CreateFormAngles( -angles.x , -angles.y , 0) ;
}

void TApplication::onCommandSetMagnitudeVector() {

    TPoint3F angles = (m_ahrs->m_q * m_settings->sensor_to_frame_orientation).getAngles();
    m_settings->sensor_to_frame_orientation *= TQuaternionF::CreateFormAngles( 0,0,-angles.z) ;
}

void TApplication::onCommandCalibrateGyro()
{
    TQuaternionF q = m_ahrs->m_q;
    CalibrateGyroStep1(15);

    m_ahrs->setOrientation(q);
    m_ahrs->setGyroError(m_settings->gyro_zero_offset);
    m_ahrs->setGyroMeas(m_settings->beta, m_settings->zeta);
}

void TApplication::CalibrateGyroCycle(float beta_start, float beta_end, float max_time)
{
    TPoint3F gravity (0.0000001f,0.0000001f,9.8f);
    TPoint3F mag     (1.0f,0.0000001f,0.0000001f);
    float time = 0;
    float MIN_DT = 0.01;
    float dt = 0;
    unsigned long last_time = 0;
    float last_print_time = 0;

    while(time < max_time) {
        delay(1);

        dt = GetDeltaTime(last_time, MIN_DT);
        if(dt == 0)
            continue;
        time += dt;
        if(time - last_print_time > 1) {
            Serial.print("Calibrate ");
            Serial.println(int(max_time - time));
            last_print_time = time;
        }
        printOut();

        float beta = Lerp(GetProgressSection(time,0,max_time), beta_start, beta_end);
        float zeta = beta * 0.1;
        m_ahrs->setGyroMeas(beta,zeta);
        updateDevices();
        TPoint3F gyr = getGyro();
        m_ahrs->update(gyr, gravity, mag, dt);
    }
}

void TApplication::CalibrateGyroStep1(float max_time)
{                        
    m_ahrs->setOrientation(TQuaternionF::Identity());

    CalibrateGyroCycle(1, 0, max_time);

    m_settings->gyro_zero_offset = m_ahrs->m_gyro_error;

    Serial.println(MSG_DONE);
}


void ReadVector(TPoint3F& vec, const char* msg) {
    vec.x = Serial.parseInt() / FLOAT_ACCURENCY;
    vec.y = Serial.parseInt() / FLOAT_ACCURENCY;
    vec.z = Serial.parseInt() / FLOAT_ACCURENCY;
    Serial.print(msg);
    PrintVector(vec);
    Serial.println();
}

void TApplication::onCommandSetMagnitudeMatrix() {
    float* mtx = &m_settings->mag_matrix.x[0][0];

    for(int i=0;i<9 ; i++)
        mtx[i] = Serial.parseInt() / FLOAT_ACCURENCY;

    Serial.print("Matrix ");
    for(int i=0;i<9 ; i++) {
        if(i%3 == 0)
            Serial.println();
        Serial.print(mtx[i]);
        Serial.print(" ");
    }
    Serial.println();
}
#define BETA_START 100
#define BETA_END 1
void TApplication::onCommandBoostFilter() {
    Serial.println("Boost");
    //ускорить фильтр - чтобы выровнить ориентацию    

    float dt = 0.01;
    unsigned long MAX_I = 500;
    for(unsigned long i = 0;i < MAX_I;i++) {
        if(i%10 == 0) {
            float progress = GetProgressSection(i , 0, MAX_I);
            float beta = Lerp(progress, BETA_START, BETA_END);       
            m_ahrs->setGyroMeas(beta,0);
        }

        updateDevices();
        TPoint3F gyr(0,0,0);
        TPoint3F acc = getAcc();
        TPoint3F mag = m_settings->disable_mag ? TPoint3F() : getMagn();
        m_ahrs->update(gyr,acc,mag, dt);
        if(i%20 == 0)
            printOut();
    }

    m_ahrs->setGyroMeas(m_settings->beta,m_settings->zeta);
    Serial.println(MSG_DONE);
}

void TApplication::onSettingsChanged() {

    m_ahrs->setGyroError(m_settings->gyro_zero_offset);
    m_ahrs->setGyroMeas(m_settings->beta, m_settings->zeta);
    m_device_gyro->setCalibrateFunction( m_settings->gyro_temperature );
}

void TApplication::onCommandLoad() {
    Serial.print("Load ");
    if(!m_settings->load()) {
        Serial.println(MSG_FAIL);
        m_settings->initDefault();
    } else {
        Serial.println(MSG_SUCCESS);
    }
    onSettingsChanged();
}

void TApplication::onCommandSave() {

    Serial.print("Save ");
    m_settings->save();
    //Serial.println(m_settings->m_save_count);
    if(!m_settings->load())
        Serial.println(MSG_FAIL);
    else
        Serial.println(MSG_SUCCESS);
}


void TApplication::onCommandDebugAction()
{
    Serial.println("Debug");
    static int variant = -1;

    variant++;
    float roll = 0;
    float pitch = 0;
    float yaw = 0;


    switch (variant % 3) {
    case 1:
        roll = -45;
        break;
    case 2:
        roll = 45;
        break;
    }

    switch (variant / 3 ) {
    case 1:
        pitch = 45;
        break;
    case 2:
        pitch = -45;
        break;

    case 3:
        yaw = -145;
        break;

    case 4:
        yaw = -145;
        pitch = 45;
        break;

    default:
        break;
    }
    if(variant >= 16)
        variant = 0;

    //m_ahrs->setGyroError(TPoint3F(0,0,0));
    m_ahrs->setOrientation(TQuaternionF::CreateFormAngles(roll * CONVERT_DPS_TO_RAD, pitch* CONVERT_DPS_TO_RAD, yaw* CONVERT_DPS_TO_RAD));
    m_ahrs->setGyroError(m_settings->gyro_zero_offset);
}

inline void ChangeRangeFlags(uint8_t& range, const uint8_t& range_count, bool& disable) {
    if(disable) {
        disable = false;
        range = 0;
        return;
    }
    range++;
    if(range >= range_count) {
        disable = true;
        range = 0;
    }
}

inline void TApplication::onChangeGyroRange() {
    ChangeRangeFlags(m_settings->gyro_mode, GYRO_RANGE_COUNT, m_settings->disable_gyro);
    m_device_gyro->begin(m_settings->gyro_mode);
    Serial.print(MSG_GYRO);
    if(m_settings->disable_gyro) {
        Serial.println(" off");
    } else {
        Serial.print(" range ");
        Serial.println(m_device_gyro->getRangeDegrees());
    }
}

inline void TApplication::onChangeAccRange() {
//    ChangeRangeFlags(m_settings->acc_mode, ACCEL_RANGE_COUNT, m_settings->disable_acc);
//    //m_device_accelmag->begin(m_settings->acc_mode);
//    if(m_settings->disable_acc) {
//        Serial.println("Acc disabled");
//    } else {
//        Serial.print("Acc range ");
//        Serial.println(m_device_accelmag->getRangeG());
//    }
}


#include "shared/max.hpp"
#define EARTH_ROTATION_SPEED 0.004166667f
void ToggleFlag(bool& flag, const char* name, bool inverse)
{
    flag = !flag;
    Serial.print(name);
    Serial.println(flag ^ inverse ? " on": " off");
}

void ChagneCoef(float& coef)
{
    coef *= 2;
    if(coef >= 100)
        coef = 0;
    else
    if(coef == 0)
        coef = 0.0005f;

}




void TApplication::receiveCmd() {
    if (!Serial.available())
        return;

    uint8_t b = Serial.read();
    switch (b) {
    case E_CMD_CODE_NONE:
    case E_CMD_CODE_CALIBRATION_STOP:
        break;
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

    case E_CMD_CODE_SET_MAGNITUDE_OFFSET:
        ReadVector(m_settings->mag_offset, "m o");
        break;
    case E_CMD_CODE_SET_MAGNITUDE_MATRIX:
        return onCommandSetMagnitudeMatrix();

    case E_CMD_CODE_SET_ACC_OFFSET:
        ReadVector(m_settings->acc_zero_offset, "a o");
        break;
    case E_CMD_CODE_SET_ACC_SCALE:
        ReadVector(m_settings->acc_scale, "a s");
        break;

    case E_CMD_CODE_SET_GRAVITY_VECTOR:        
        return onCommandSetGravityVector();
    case E_CMD_CODE_SET_YAW_NORTH:        
        return onCommandSetMagnitudeVector();
    case E_CMD_CODE_DEFAULT_ORIENTATION:        
        m_settings->sensor_to_frame_orientation = TQuaternionF(1,0,0,0);
        break;
    case E_CMD_CODE_DEBUG_ACTION:
        return onCommandDebugAction();

    case E_CMD_CODE_TOGGLE_PRINT_MODE:
        ToggleFlag(m_settings->print_mag, "prnt mode", false);
        break;

    case E_CMD_CODE_TOGGLE_GYRO:
        //ToggleFlag(m_settings->disable_gyro, "Gyro", true);
        onChangeGyroRange();
        m_ahrs->setGyroMeas(m_settings->beta, m_settings->disable_gyro? 0 : m_settings->zeta);
        m_ahrs->setGyroError( m_settings->disable_gyro ? TPoint3F() : m_settings->gyro_zero_offset) ;
        break;
    case E_CMD_CODE_TOGGLE_ACC:
        //onChangeAccRange();
        ToggleFlag(m_settings->disable_acc, MSG_ACCEL, true);
        break;
    case E_CMD_CODE_TOGGLE_MAG:
        ToggleFlag(m_settings->disable_mag, MSG_MAGNIT, true);
        break;

    case E_CMD_CODE_CHANGE_BETA:
        ChagneCoef(m_settings->beta);
        m_ahrs->setGyroMeas(m_settings->beta,m_settings->zeta);
        break;

    case E_CMD_CODE_CHANGE_ZETA:
        ChagneCoef(m_settings->zeta);
        m_ahrs->setGyroMeas(m_settings->beta,m_settings->zeta);
        break;
    case E_CMD_CODE_LOAD:
        return onCommandLoad();
    case E_CMD_CODE_SAVE:
        return onCommandSave();
    case E_CMD_CODE_LOAD_DEFAULT: {
        Serial.println(MSG_DONE);
        uint16_t save_cnt = m_settings->m_save_count;
        m_settings->initDefault();
        m_settings->m_save_count = save_cnt;
        onSettingsChanged();
        break;
    }
    default:
        Serial.println(MSG_FAIL);
    }
}



































