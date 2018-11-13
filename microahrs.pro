HEADERS += \                    
    shared/in_range.hpp \
    shared/max.hpp \    
    sensor/Adafruit_FXAS21002C.h \
    sensor/Adafruit_Sensor.h \
    sensor/Adafruit_FXOS8700.h \            
    sensor/Adafruit_FXAS21002C_termo.h \    
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
    EPrintModeh


SOURCES += \        
    test/test.cpp \
     sensor/Adafruit_FXAS21002C.cpp \
     sensor/Adafruit_FXOS8700.cpp \
     sensor/Adafruit_FXAS21002C_termo.cpp \
     TAHRSMadgwick.cpp \
    TApplication.cpp

OTHER_FILES += \
    microahrs.ino


INCLUDEPATH += sensor
DEFINES *= ARDUINO=200
