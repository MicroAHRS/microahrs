#include <Wire.h>
#include <Adafruit_Sensor.h>

#include "AHRSMadgwick.h"
#include "shared/in_range.hpp"

#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>


Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);


TVector3f mag_offset
( 
  19.07,
  16.78 - 0.25,
  63.38 - 0.25
);

// Soft iron error compensation matrix
float mag_softiron_matrix[3][3] = { {  0.959, 0.002, 0.003 },
                                    {  0.002, 0.964, 0.008 },
                                    {  0.003, 0.008, 1.082 } };
                                   
TVector3f gyro_zero_offset( 
  22612.24 / 1000000,
  8122.33 / 1000000,
  -3508.38 / 1000000  
);
TVector3f acc_zero_offset( 
  0.3253305,  
  -0.3593825,
  0.6121345
  );

TVector3f acc_scale(
  1.01591586,
  0.9863349377,
  1.017566665 
);


AHRSMadgwick filter;

void setup()
{
  Serial.begin(115200);

  // Wait for the Serial Monitor to open (comment out to run without Serial Monitor)
  // while(!Serial);

  Serial.println(F("Adafruit AHRS Fusion Example")); Serial.println("");

  // Initialize the sensors.
  if(!gyro.begin())
  {
    /* There was a problem detecting the gyro ... check your connections */
    Serial.println("Ooops, no gyro detected ... Check your wiring!");
    while(1);
  }


  if(!accelmag.begin(ACCEL_RANGE_4G))
  {
    Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");
    while(1);
  }

  pinMode(LED_BUILTIN, OUTPUT);      
}

static unsigned long tick_count = 0;

float mag_angle = 0;

void PrintVector(const TVector3f& vec) {
    Serial.print(vec.x);      
    Serial.print(" ");
    Serial.print(vec.y);
    Serial.print(" ");
    Serial.print(vec.z);
    Serial.print(" ");    
}

void PrintOut(sensors_event_t& gyro_event , sensors_event_t& accel_event, sensors_event_t& mag_event,sensors_event_t& gyro_t_event) {
  // Print the orientation filter output
  // Note: To avoid gimbal lock you should read quaternions not Euler
  // angles, but Euler angles are used here since they are easier to
  // understand looking at the raw values. See the ble fusion sketch for
  // and example of working with quaternion data.
  
    float roll = filter.getRoll();
    float pitch = filter.getPitch();
    float heading = filter.getYaw();
    //int gyro_temperature = (int)gyro_event.gyro.reserved[0];
    
    //Serial.print(millis());
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


    //Serial.print(" Error deg ");
    //PrintVector(filter.m_gyro_error * (180 / M_PI));
    
    Serial.println();  
}


void TurnLight(bool enabled) {
  static bool g_light_enabled = false;
  if(g_light_enabled == enabled)  
    return;
  digitalWrite(LED_BUILTIN, !enabled?LOW:HIGH);  
  g_light_enabled = enabled;
}


inline TVector3f VectorFromEvent(sensors_vec_t vec )  { return TVector3f(vec.x, vec.y ,vec.z); }

float print_timer = 0.0;
void loop(void)
{  
  static unsigned long last_update_time  = 0;
  static bool started = false;

  unsigned long ts = millis();
  float dt = ((ts - last_update_time) / 1000.0);
  if(dt < 0.01) {
    delay(1);
    return;
  }
  unsigned long seconds_count = int(ts / 1000);
     
  last_update_time = ts;
  
   
  sensors_event_t gyro_event;
  sensors_event_t gyro_t_event;
  sensors_event_t accel_event;
  sensors_event_t mag_event;

//  Get new data samples
  gyro.getEvent(&gyro_event,&gyro_t_event);
  accelmag.getEvent(&accel_event, &mag_event);

  // Apply mag offset compensation (base values in uTesla)
  TVector3f mag = VectorFromEvent(mag_event.magnetic) - mag_offset;  
  // Apply mag soft iron error compensation
  float mx = mag.x * mag_softiron_matrix[0][0] + mag.y * mag_softiron_matrix[0][1] + mag.z * mag_softiron_matrix[0][2];
  float my = mag.x * mag_softiron_matrix[1][0] + mag.y * mag_softiron_matrix[1][1] + mag.z * mag_softiron_matrix[1][2];
  float mz = mag.x * mag_softiron_matrix[2][0] + mag.y * mag_softiron_matrix[2][1] + mag.z * mag_softiron_matrix[2][2];
  mag.x = mx; mag.y = my; mag.z = mz;

  TVector3f gyr = VectorFromEvent(gyro_event.gyro) + gyro_zero_offset;   
  TVector3f acc = (VectorFromEvent(accel_event.acceleration) - acc_zero_offset).scale(acc_scale); 

  TVector3f zero(0,0,0);

  //gyr = zero;
  //gyr.z = -1.5/180 * M_PI ; //rotate
  //acc = TVector3f(0.0001,0.0001,1);
  //mag = zero;

  //mag_angle = atan2(mag.y ,mag.x ) * 180 / M_PI;
  
//  Serial.print(" ");
//  Serial.print(mag.z);
     
  // Update the filter
  filter.update( gyr,acc,mag,dt );

  tick_count++;  

  filter.setGyroMeas(0.4, 0.0); 

//  if( ts + 10000 < 5000 ) {    
//    filter.setGyroMeas(40, 0);     
//  } else if( ts + 10000 < 6000 ) {
//    float progress = GetProgressSection( ts , 0 , 5000);
//    float beta = 40 * (1-progress) + 0.4 * progress;
//    float zeta = 0 * (1-progress) + 0.1 * progress;    
//    filter.setGyroMeas(beta, zeta);         
//  } 

  TurnLight(seconds_count % 2 == 0 || !started);

  print_timer += dt;
  while(print_timer >= 0.05 ) {
      PrintOut(gyro_event, accel_event , mag_event, gyro_t_event);                      
      print_timer -= 0.05;      
  }
}
