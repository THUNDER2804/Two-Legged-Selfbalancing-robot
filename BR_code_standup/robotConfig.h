#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#define SERIAL_BAUD 921600
char ESP32_MACADDRESS[19] = "C8:F0:9E:9E:3C:DC";  // ESP32 Mac Address for PS4 connection "C8:F0:9E:9E:F6:40" KIT0

// Servo Motor Parameters
#define SERVO_DRIVER_OSC_FREQ 23800000  //Servo Motor Driver setOscillatorFrequency 

#define SERVO_FREQ 100   // Servo Motor PWM frequency

// Servo motor 1
#define SERVO1_CH 14  
#define SERVO1_PULSE_POS1 480
#define SERVO1_PULSE_POS2 750
#define SERVO1_ANGLE_POS1 -90
#define SERVO1_ANGLE_POS2 0
#define SERVO1_ANGLE_MIN -110
#define SERVO1_ANGLE_MAX 60
#define SERVO1_INIT_ANG -50

// Servo motor 2
#define SERVO2_CH 15  // motor 2
#define SERVO2_PULSE_POS1 720
#define SERVO2_PULSE_POS2 990
#define SERVO2_ANGLE_POS1 180
#define SERVO2_ANGLE_POS2 270
#define SERVO2_ANGLE_MIN 120  
#define SERVO2_ANGLE_MAX 290
#define SERVO2_INIT_ANG 230

// Servo motor 3
#define SERVO3_CH 1  // motor 3
#define SERVO3_PULSE_POS1 730
#define SERVO3_PULSE_POS2 1000
#define SERVO3_ANGLE_POS1 0
#define SERVO3_ANGLE_POS2 90
#define SERVO3_ANGLE_MIN -60
#define SERVO3_ANGLE_MAX 120
#define SERVO3_INIT_ANG 50

// Servo motor 4
#define SERVO4_CH 0   // motor 4
#define SERVO4_PULSE_POS1 480
#define SERVO4_PULSE_POS2 750
#define SERVO4_ANGLE_POS1 90
#define SERVO4_ANGLE_POS2 180
#define SERVO4_ANGLE_MIN 70
#define SERVO4_ANGLE_MAX 240
#define SERVO4_INIT_ANG 130


// DC motor parameters
#define RIGHT_DC_ENA_PIN 4
#define RIGHT_DC_PWM_CHANNEL 0
#define RIGHT_DC_ENC_A 16
#define RIGHT_DC_ENC_B 17
#define RIGHT_DC_PIN_1 27
#define RIGHT_DC_PIN_2 26

#define LEFT_DC_ENA_PIN 14
#define LEFT_DC_PWM_CHANNEL 1
#define LEFT_DC_ENC_A 32
#define LEFT_DC_ENC_B 35
#define LEFT_DC_PIN_1 25
#define LEFT_DC_PIN_2 33

// BNO080 IMU
#define BNO08X_RESET -1

//LEDs
#define IMU_CAL_LED 19


#define LEG_RANGE_Y 40

#define WHEEL_MAX_RPM 150 

#define WHEEL_FB_MAX_SPEED 60 
#define MAX_TURN_SPEED 30

#define MAX_CONTROL_ANGLE 5

#endif //ROBOT_CONFIG_H