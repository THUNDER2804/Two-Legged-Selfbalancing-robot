#include "parallelLeg.h"
#include <PS4Controller.h>
#include "robotConfig.h"
#include <Adafruit_BNO08x.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver servoPWM = Adafruit_PWMServoDriver();

ParallelLeg rightLeg;
ParallelLeg leftLeg;

float targetXr;
float targetYr;
float targetXl;
float targetYl;

// IMU, converted Euler angle value
volatile float eulerX;
volatile float eulerY;
volatile float eulerZ;

void quaternionToEuler(float qr, float qi, float qj, float qk, bool degrees = false) 
 {
  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);
  eulerZ = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  eulerY = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  eulerX = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));
  if (degrees) 
   {
    eulerZ *= RAD_TO_DEG;
    eulerY *= RAD_TO_DEG;
    eulerX *= RAD_TO_DEG;
   }
 }



// IMU Sensor define
Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
sh2_SensorId_t reportType = SH2_ROTATION_VECTOR;
long reportIntervalUs = 5000;  // 5ms -> 
float offset_roll, offset_pitch, offset_yaw;
float mod_roll, mod_pitch, mod_yaw;
volatile float rawAccX, rawAccY, rawAccZ;
volatile float rawGyroX, rawGyroY, rawGyroZ;
volatile float rawMagX, rawMagY, rawMagZ;

// PS4 Controller input
int PS4_LStick_x = 0;
int PS4_LStick_y = 0;
int PS4_RStick_x = 0;
int PS4_RStick_y = 0;

// PID Control Parameter for balancing
float Kp = 20.0;
float Ki = 8.0;
float Kd = 0.0001; // Inverted Pendulum PID control gains
float current_theta = 0;
float target_theta  = 0;
float prev_theta = 0;
float err_theta = 0.0;    // current error
float p_err_theta = 0.0;  // previous error
float int_err_theta = 0.0;
float d_err_theta = 0.0;
unsigned long ct; // current time
unsigned long pt; // previous time
float dt; // time inverval 
float whl_ctrl_input; // computed control input to DC motor





// misc
float rightWheelRPM_target = 0.0;
float leftWheelRPM_target  = 0.0;
float Motor_LR = 0.0,Target_Motor_LR = 0.0,Motor_FB = 0.0;
float Left_Motor_Wheel = 0.0,Right_Motor_Wheel = 0.0;
float set_point_angle=0.0;
int circle_trigger=0,stand_trigger = 0;
int temp_speed=0;

void IMUtickerCallback(){
  if (bno08x.wasReset()) {
    setReports();
  }
  
  if(bno08x.getSensorEvent(&sensorValue)){
    switch (sensorValue.sensorId) {
      case SH2_ROTATION_VECTOR:
        quaternionToEuler(sensorValue.un.rotationVector.real, sensorValue.un.rotationVector.i, sensorValue.un.rotationVector.j, sensorValue.un.rotationVector.k, true);        
        break;
      // We don't use raw IMU data, but for the case you want to use
      case SH2_RAW_ACCELEROMETER:
        rawAccX = sensorValue.un.rawAccelerometer.x;
        rawAccY = sensorValue.un.rawAccelerometer.y;
        rawAccZ = sensorValue.un.rawAccelerometer.z;        
        break;
      case SH2_RAW_GYROSCOPE:
        rawGyroX = sensorValue.un.rawGyroscope.x;
        rawGyroY = sensorValue.un.rawGyroscope.y;
        rawGyroZ = sensorValue.un.rawGyroscope.z;        
        break;
      case SH2_RAW_MAGNETOMETER:
        rawMagX = sensorValue.un.rawMagnetometer.x;
        rawMagY = sensorValue.un.rawMagnetometer.y;
        rawMagZ = sensorValue.un.rawMagnetometer.z;        
        break;
    }

    // When the IMU is calibrated LED lits on
    if(sensorValue.status == 3){
      digitalWrite(IMU_CAL_LED,HIGH) ;
    }else{
      digitalWrite(IMU_CAL_LED,LOW) ;
    }
  }  
}

void setReports(void) {
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR,reportIntervalUs)) {
    Serial.println("Could not enable rotation vector");
  }
  // We don't use raw IMU data, but for the case you want to use, you need to uncomment the below. 
  // If you want to read raw data of IMU, then uncomment these
  // if (!bno08x.enableReport(SH2_RAW_ACCELEROMETER,reportIntervalUs)) {
  //   Serial.println("Could not enable raw accelerometer");
  // }
  // if (!bno08x.enableReport(SH2_RAW_GYROSCOPE,reportIntervalUs)) {
  //   Serial.println("Could not enable raw gyroscope");
  // }
  // if (!bno08x.enableReport(SH2_RAW_MAGNETOMETER,reportIntervalUs)) {
  //   Serial.println("Could not enable raw magnetometer");
  // }
}

void legServoInit()
 {
  servoPWM.begin();
  servoPWM.setOscillatorFrequency(SERVO_DRIVER_OSC_FREQ); // Driver Oscillator Frequency
  servoPWM.setPWMFreq(SERVO_FREQ);  

  rightLeg.setParameters(servoPWM, SERVO1_INIT_ANG, SERVO2_INIT_ANG, 1);
  rightLeg.setFrontServo(SERVO1_CH, SERVO1_PULSE_POS1, SERVO1_PULSE_POS2, SERVO1_ANGLE_POS1, SERVO1_ANGLE_POS2, SERVO1_ANGLE_MIN, SERVO1_ANGLE_MAX);
  rightLeg.setRearServo(SERVO2_CH, SERVO2_PULSE_POS1,  SERVO2_PULSE_POS2, SERVO2_ANGLE_POS1, SERVO2_ANGLE_POS2, SERVO2_ANGLE_MIN, SERVO2_ANGLE_MAX);
  //rightLeg.goHome();  // move to SERVOx_INIT_ANG position
  // Get the current position as the target position
  targetXr = rightLeg.endPoint[0];
  targetYr = rightLeg.endPoint[1];

  leftLeg.setParameters(servoPWM, SERVO3_INIT_ANG, SERVO4_INIT_ANG, 2);
  leftLeg.setFrontServo(SERVO3_CH, SERVO3_PULSE_POS1, SERVO3_PULSE_POS2, SERVO3_ANGLE_POS1, SERVO3_ANGLE_POS2, SERVO3_ANGLE_MIN, SERVO3_ANGLE_MAX);
  leftLeg.setRearServo(SERVO4_CH, SERVO4_PULSE_POS1,  SERVO4_PULSE_POS2, SERVO4_ANGLE_POS1, SERVO4_ANGLE_POS2, SERVO4_ANGLE_MIN, SERVO4_ANGLE_MAX);
  leftLeg.goHome();  // move to SERVOx_INIT_ANG position
  //targetXl = leftLeg.endPoint[0];
  //targetYl = leftLeg.endPoint[1];


 }

void legWheelInit(){
  //DC motor setup
  rightLeg.wheelMotor.setupParameters(RIGHT_DC_ENA_PIN,RIGHT_DC_PIN_1,RIGHT_DC_PIN_2, RIGHT_DC_PWM_CHANNEL,RIGHT_DC_ENC_A,RIGHT_DC_ENC_B);
  leftLeg.wheelMotor.setupParameters(LEFT_DC_ENA_PIN,LEFT_DC_PIN_1,LEFT_DC_PIN_2, LEFT_DC_PWM_CHANNEL,LEFT_DC_ENC_A,LEFT_DC_ENC_B);  
  rightWheelRPM_target = leftWheelRPM_target = 0.0;
}

// Initialize bno085 IMU sensor
void IMUinit()
 {
  // Try to initialize!
  if (!bno08x.begin_I2C()) 
   {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10);}
   }
  
  setReports();
  offset_roll = offset_pitch = offset_yaw = 0.0;
  mod_roll = mod_pitch = mod_yaw = 0.0;
  delay(100);

  // For IMU calibration status monitoring LED
  pinMode(IMU_CAL_LED, OUTPUT); 
  digitalWrite(IMU_CAL_LED,LOW);

}



//PID control
void control_angle()
 {
  err_theta = target_theta - current_theta;    //deg
  int_err_theta = int_err_theta + err_theta*dt;
  d_err_theta = (err_theta - p_err_theta)/dt;
  p_err_theta = current_theta;
  whl_ctrl_input = Kp*err_theta + Ki*int_err_theta + Kd*d_err_theta;
  whl_ctrl_input = constrain(whl_ctrl_input,-WHEEL_MAX_RPM,WHEEL_MAX_RPM);// DC motor control input Saturation
 }



void setup() {
  // If your Arduino IDE doen't have option for baudrate 921600, then change SERIAL_BAUD parameter in 'robotConfig.h'
  Serial.begin(SERIAL_BAUD);  

  // I2C Bus initialization
  Wire.begin();
  Wire.setClock(400000);

  // initialize units
  legServoInit();
  legWheelInit();
  IMUinit();  

  // You need to change ESP32_MACADDRESS in 'robotConfig.h' according to your ESP32 Mac Address. 
  PS4.begin(ESP32_MACADDRESS);
  while(!PS4.isConnected());

  // Time variables initialized to the current time
  dt = pt = ct = micros();
}

void loop() 
 {
  Tune_PID();
  Robot_move();
  // IMU data update
  IMUtickerCallback();
  // IMU Euler angle
  mod_roll  = eulerX - offset_roll;
  mod_pitch = eulerY - offset_pitch;
  mod_yaw   = eulerZ - offset_yaw; 
  current_theta = -mod_roll;  // deg

  // PID controller part
  ct = micros(); // current time
  dt = (float)(ct - pt)/1e6;   // time interval in sec
  pt = ct;


  //control_speed();

  control_angle();
  //if(whl_ctrl_input < 2 && whl_ctrl_input > -2)whl_ctrl_input = 0;

  Left_Motor_Wheel = whl_ctrl_input-Target_Motor_LR;
  Right_Motor_Wheel = whl_ctrl_input+Target_Motor_LR;

  


  stand_auto();

  if(stand_trigger == 1)
   {
    rightLeg.wheelMotor.setTargetRPM(-Right_Motor_Wheel);
    leftLeg.wheelMotor.setTargetRPM(Left_Motor_Wheel);
   }
  if(circle_trigger == 0)
   {
    rightLeg.wheelMotor.setTargetRPM(0);
    leftLeg.wheelMotor.setTargetRPM(0);
   }


  // Leg Inverse Kinematics
  targetYr = -90;
  targetYl = 90;
  rightLeg.solveLegIK(targetXr, targetYr);
  leftLeg.solveLegIK(targetXl, targetYl);  



  Serial.print(Kp); Serial.print("\t"); 
  Serial.print(Ki); Serial.print("\t"); 
  Serial.print(Kd); Serial.print("\t"); 
  Serial.print(whl_ctrl_input); Serial.print("\t\t"); 
  Serial.print(current_theta); Serial.print("\t");
  Serial.print(targetYr); Serial.print("\n");
 }


void Robot_move()
 {
  if(PS4.isConnected())
   {
    //Robot Position control
   if(PS4.LStickX()) 
     {
      if(abs(PS4.LStickX())>10)
       Target_Motor_LR = map(PS4.LStickX(),-125,125,-MAX_TURN_SPEED, MAX_TURN_SPEED);  
      else 
       Target_Motor_LR = 0;  
      Target_Motor_LR = constrain(Target_Motor_LR,-MAX_TURN_SPEED,MAX_TURN_SPEED); 
     }
    
    if(PS4.LStickY()) 
     {
      if(abs(PS4.LStickY())>10)
       Motor_FB = map(PS4.LStickY(),-125,125,-WHEEL_FB_MAX_SPEED, WHEEL_FB_MAX_SPEED);
      else 
       Motor_FB=0;
      Motor_FB = constrain(Motor_FB,-WHEEL_FB_MAX_SPEED,WHEEL_FB_MAX_SPEED);
     }

    if(PS4.LStickY()) 
     {
      if(abs(PS4.LStickY())>10)
       target_theta = map(PS4.LStickY(),-125,125,-5, 5);
      else 
       target_theta=0;
      target_theta = constrain(target_theta,-5,5);
     }
  
    //Robot Leg control
    if (PS4.RStickY()) // Move the wheel axis in y-axis with repect to Frame R and L
     {
      PS4_RStick_y = map(PS4.RStickY(),-125,125,-LEG_RANGE_Y/2,LEG_RANGE_Y/2);
      if(abs(PS4_RStick_y) < LEG_RANGE_Y/10) PS4_RStick_y=0;
      targetYr = rightLeg.endPoint[1] + (float)PS4_RStick_y/10;
      targetYl = -targetYr;   // Because the y-axis of Frame L is downward
     }

    if(PS4.Triangle())
     {
      offset_roll  += mod_roll;
      offset_pitch += mod_pitch;
      offset_yaw   += mod_yaw;
      int_err_theta  = 0.0;  // Angle integration error reset
     }
    if (PS4.Circle())
    {
      circle_trigger=1;
    }
    if (PS4.Cross()) 
    {
      circle_trigger =0;
      stand_trigger =0;
    }
   }
  rightLeg.moveLeg(rightLeg.LegIK_res[0], rightLeg.LegIK_res[2]);
  leftLeg.moveLeg(leftLeg.LegIK_res[0], leftLeg.LegIK_res[2]);
 }


  
void stand_auto()
 {
   temp_speed = 150;
  if(circle_trigger ==1 && stand_trigger ==0) 
   { 
    if(abs(current_theta)<0.5 && stand_trigger == 0)
     {
      int_err_theta  = 0.0;
      stand_trigger =1;
     }
    if(stand_trigger !=1 )
     {
      if(current_theta>0 )
       {
        rightLeg.wheelMotor.setTargetRPM(temp_speed);
        leftLeg.wheelMotor.setTargetRPM(-temp_speed);
       }
      if(current_theta<0)
       {
        rightLeg.wheelMotor.setTargetRPM(-temp_speed);
        leftLeg.wheelMotor.setTargetRPM(temp_speed);
       }
      if(abs(current_theta)<10)
      {
       rightLeg.wheelMotor.setTargetRPM(0);
        leftLeg.wheelMotor.setTargetRPM(0);
      }      
     }        
   }

 }

void Tune_PID()
 {
  if(Serial.available())
   {
    Kp = Serial.parseFloat();
    Ki = Serial.parseFloat();
    Kd = Serial.parseFloat();
    Serial.read(); // read the carriage return    
   }
 }

