# microahrs

AHRS algorithm for fusion of data from Gyroscope/ Acceleromenter/ Magnetometer / Termometer into orienation (roll / pith / yaw)

Video example
[![video_example](https://img.youtube.com/vi/z7SdzQfINPQ/0.jpg)](https://www.youtube.com/watch?v=z7SdzQfINPQ)

For Visualization programm see https://github.com/titanproger/ahrs_web_visualisator

Based on Madgwick algorithm 
See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
 * see @link = https://github.com/ccny-ros-pkg/imu_tools/tree/indigo/imu_filter_madgwick

![alt text](https://github.com/titanproger/ahrs_web_visualisator/blob/master/readme/demo_screen_1.jpg)

![alt text](https://github.com/titanproger/ahrs_web_visualisator/blob/master/readme/mag_calibration.jpg)

#Features:
  - Temperature compensation
  - Gyro drift compensation
  - Magnetormeter calibration
  - Can Bus
  - Serial Bus
  - Compass (Yaw)
  - Roll
  - Pith
    
# Used Hardware: 
Microcontroller: Arduino | STM32f103C8T6 (Blue pill)
Acc/magnetometer/termometer: FXAS21002C
Gyro:FXOS8700
CanBus Controller: mcp2515


# Used software:
  Linux OS: Ubuntu


Please clone common https://github.com/MicroAHRS/common to folder above this

  
