HEADERS += \                    
    shared/in_range.hpp \
    shared/max.hpp \    
    shared/Function/TFunction.h \
    shared/Function/TFunctionLine.h \
    shared/Function/TFunction3.h \
    shared/Geometry/TPoint3.h \
    shared/Geometry/TQuaternion.h \
    shared/Function/TFunctionLineF.h \
    shared/Geometry/TQuaternionF.h \
    shared/Geometry/TPoint3F.h \
    shared/Geometry/TPoint3D.h \
    shared/Geometry/TPoint3I.h \
    TAppSettings.h \
    TAHRSMadgwick.h \
    shared/Geometry/TMatrix3.h \
    shared/Geometry/TMatrix3F.h \
    TApplication.h \
    ECommandCode.h \
    EPrintMode.h \
    A_FXOS8700.h \
    A_FXAS21002C.h \
    A_FXAS21002C_termo.h \
    A_Sensor.h


SOURCES += \        
    test/test.cpp \
     TAHRSMadgwick.cpp \
    TApplication.cpp \
    A_FXOS8700.cpp \
    A_FXAS21002C.cpp \
    A_FXAS21002C_termo.cpp \
    TAppSettings.cpp

OTHER_FILES += \
    microahrs.ino


INCLUDEPATH += sensor
DEFINES *= ARDUINO=200
