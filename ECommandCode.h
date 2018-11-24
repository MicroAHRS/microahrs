#ifndef ECOMMANDCODE_H
#define ECOMMANDCODE_H

enum ECommandCode {
    E_CMD_CODE_NONE = 0,
    E_CMD_CODE_RESET_PITCH_ROLL      = 1,  // сбросить крен тангаж
    E_CMD_CODE_SET_YAW_BY_MAG        = 2,  // установить углы по магнетометру
    E_CMD_CODE_SET_PITCH_ROLL_BY_ACC = 3,  // установить углы по акселерометру
    E_CMD_CODE_BOOST_FILTER          = 4,  // установить углы по акселерометру

    E_CMD_CODE_CHANGE_BETA           = 5,
    E_CMD_CODE_CHANGE_ZETA           = 6,
    E_CMD_CODE_CHANGE_NETA           = 7,

    E_CMD_CODE_SET_GRAVITY_VECTOR  = 10,  // текущее направление силы тяжести принять за 0 (roll pitch)
    E_CMD_CODE_SET_YAW_NORTH       = 11,  // текущее направление на север принять за 0 (yaw)
    E_CMD_CODE_DEFAULT_ORIENTATION = 12,  // сбросить модификатор ориентации

    E_CMD_CODE_CALIBRATE_GYRO       = 20,
    E_CMD_CODE_SET_MAGNITUDE_OFFSET = 21,
    E_CMD_CODE_SET_MAGNITUDE_MATRIX = 22,

    E_CMD_CODE_SET_ACC_OFFSET       = 23,
    E_CMD_CODE_SET_ACC_SCALE        = 24,

    E_CMD_CODE_DEBUG_ACTION         = 30,
    E_CMD_CODE_TOGGLE_GYRO          = 31,
    E_CMD_CODE_CALIBRATION_STOP     = 32,  // code = space - useful when send calibration
    E_CMD_CODE_TOGGLE_MAG           = 33,
    E_CMD_CODE_TOGGLE_ACC           = 34,


    E_CMD_CODE_SAVE                = 40,
    E_CMD_CODE_LOAD                = 41,
    E_CMD_CODE_LOAD_DEFAULT        = 42,
    E_CMD_CODE_TOGGLE_PRINT_MODE   = 43,
};

#endif // ECOMMANDCODE_H
