#ifndef TAPPLICATIONARDUINO_H
#define TAPPLICATIONARDUINO_H

//#include <Adafruit_FXAS21002C_termo.h>

#include "shared/Geometry/TPoint3F.h"
#include "shared/Function/TFunctionAverageF.h"
#include "shared/Function/TFunctionLimitedAverage.h"

// 1.01
// 1.02 поменялись коды комманды
// 1.03 Reset Pitch Roll с учетом ориентации сенсора
// 1.04 отправка сырых даных магнитометра
// 1.05 ускорение фильтра за одну итерацию - при помощи рассчета углов
// 1.06 разделены поправки акселерометра и магнетометра, поправка магенитометр вынесена в параметр neta
// 1.07 yaw компенсирует только магнитометр а roll picth только акселерометр
// 1.08 усредненные показания перегрузки + автокалибровка
// 1.09 перегрузка установленна как 5% , алгоритм компенсации дрейфа гироскопа работает только в если проходим через ноль за поределенное время
//      более точные коэфициенты beta zeta neta - в 2.6 раза надо их увеличить относительно прежних чтобы осталось тоже самое значение

// 1.10 смена частоты опроса - гироскоп 400 герц, акселерометр и магнитометр 25 герц.
//      ускорение частоты цикла с 140 до 380
// 1.11 настроил способы отключения компенсации нуля гироскопа  200 герц частота гирика
// 1.12 включил компенсацию по времени
// 1.13 добавленн вывод в через CAN

#define AHRS_VERSION "1.13"

class TAppSettings;
class A_FXAS21002C_termo;
class A_FXAS21002C;
class A_FXOS8700;
class TAHRSMadgwick;
class TAirCan;

class TApplication
{
public:
    TApplication();
    void setup();
    void loop();

protected:
    bool init();
    void update(float dt);
    void updateDevices(float dt) ;
    void updateCalibration(float dt);
    void updateAHRS(float dt);    
    void updateDriftCoefByAngles();

    void updateCalibrationAcc(float dt);
    void onCalibrationFinished();
    //void turnLight(bool enabled);
    void printOut();
    void sendCan(const TPoint3F &angles, float g, float g_angle);

    TPoint3F getAcc() const;
    TPoint3F getMagn() const;
    TPoint3F getGyro() const {return m_gyro_value;}
    float getTemperature() const {return m_temperature;}

    inline void onCommandResetPitchRoll();
    inline void onCommandSetYawByMag();
    inline void onCommandSetPitchRollByAcc();
    void        onCommandBoostFilter(bool boost_mag);
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

    bool isCalibrationMode() const {return m_calib_mode_time_cur > 0;}


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
    float           m_gyro_dt;
    float           m_gyro_dt_filter;
    float           m_acc_dt;
    float           m_acc_dt_filter;

    float           m_calib_mode_time_cur; // if > 0 we in caliblrate mode

    TAHRSMadgwick*       m_ahrs;
    A_FXAS21002C_termo*  m_device_gyro;
    A_FXOS8700*          m_device_accelmag;
    TAppSettings*        m_settings;
    TAirCan*             m_air_can;

    TFunctionLimitedAverage<TPoint3F, float, 4> m_avg_acc;
    TFunctionAverageF    m_avg_acc_length;
    TFunctionAverageF    m_avg_acc_dispersion;
};

#endif // TAPPLICATIONARDUINO_H
