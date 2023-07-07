
#include"servoMotor.h"

#define ANGLE_SCALE 10
#define SERVOMIN  200 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  1000 // This is the 'maximum' pulse length count (out of 4096)  3840

void ServoMotor::setupParameters(uint8_t _pwmChannel, Adafruit_PWMServoDriver &_pwm, uint16_t plPos1, uint16_t plPos2, float angPos1, float angPos2){
  pwmChannel = _pwmChannel;
  pwm = _pwm;
  currentPulseLength = 0;
  pulseLength_Min = SERVOMIN;
  pulseLength_Max = SERVOMAX;
  angleLimit_min = 0;
  angleLimit_max = 90;

  // tuning variables
  pulseLength_Pos1 = plPos1;   
  pulseLength_Pos2 = plPos2;   
  angle_Pos1 = angPos1;
  angle_Pos2 = angPos2;
}

void ServoMotor::setPWMpulse(uint16_t pl){  
  // Set the PWM with the given pulse length value pl
  if(pl < pulseLength_Min){
    pl = pulseLength_Min;
  }else if( pl > pulseLength_Max){
    pl = pulseLength_Max;
  }
  pwm.setPWM(pwmChannel,0,pl);
}

void ServoMotor::setAngleDegree(float degree){
  // Set the PWM with the given orientation value in degree

  if(degree < angleLimit_min){
    degree = angleLimit_min;
  }else if( degree > angleLimit_max){
    degree = angleLimit_max;
  }
  currentPulseLength = map(degree*ANGLE_SCALE,angle_Pos1*ANGLE_SCALE, angle_Pos2*ANGLE_SCALE,pulseLength_Pos1,pulseLength_Pos2);  
  setPWMpulse(currentPulseLength);
}

void ServoMotor::setAngleLimits(float ang_min, float ang_max){
  // Set the mechanical angle limits
  angleLimit_min = ang_min;
  angleLimit_max = ang_max;

  pulseLength_Min = map(angleLimit_min*ANGLE_SCALE,angle_Pos1*ANGLE_SCALE, angle_Pos2*ANGLE_SCALE,pulseLength_Pos1,pulseLength_Pos2);  
  pulseLength_Max = map(angleLimit_max*ANGLE_SCALE,angle_Pos1*ANGLE_SCALE, angle_Pos2*ANGLE_SCALE,pulseLength_Pos1,pulseLength_Pos2);  
}

