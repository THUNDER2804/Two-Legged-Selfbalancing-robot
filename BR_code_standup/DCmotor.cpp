#include <Arduino.h>
#include "DCmotor.h"

void IRAM_ATTR tickerCallback(DCmotor *arg){
  arg->onTickerCallback();
}

void IRAM_ATTR controlTickerCallback(DCmotor *arg){
  arg->onControlTickerCallback();
}

void DCmotor::setupParameters(int pwmPin, int motorIN1, int motorIN2, int pwmChannel, int encA, int encB){
  _pwmOutputPin = pwmPin;
  _motorIN1 = motorIN1;
  _motorIN2 = motorIN2;
  _pwmChannel = pwmChannel;
  
  // PWM pin settup
  pinMode(_pwmOutputPin, OUTPUT);
  pinMode(_motorIN1, OUTPUT);
  pinMode(_motorIN2, OUTPUT);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(_pwmOutputPin, _pwmChannel);

  // configure LED PWM functionalitites
  currentPWMfreq = ledcSetup(_pwmChannel, _pwmFreq, _pwmResol);  
  encoder.attachFullQuad(encA, encB);  
  init();
}


void DCmotor::init(){
  // Write codes for initialization  
  setMotorDir(0); // stop motor initially

  // encdoer init
  prev_count = 0;
  encoder.clearCount(); 
  // encoder.setFilter(1023);
  delta_time = 0;    
  currentTime = prevTime = micros();  
  rpm = prevRPM = filteredRPM = prev_fRPM = target_fRPM = 0.0;
  prev_rpmErr = 0.0;
  accRPM = 0;

  Kp = 0.9;//0.9
  Kd = 7.0;//7
  Ki = 0.12;

  // Encoder data reading frequency 500 Hz
  rpmTicker.attach_ms(2,tickerCallback,this);  

  // DC motor control frequency 100Hz
  controlTicker.attach_ms(10,controlTickerCallback,this);  
}

void DCmotor::onTickerCallback(){    
  currentTime = micros();
  current_count = encoder.getCount();  
  delta_time = currentTime - prevTime;
  diff_cnt = current_count - prev_count;  
  // rotation per min = 60*1000*1000*diff_cnt/delta_time/(64*30)
  rpm = 6e7*diff_cnt/delta_time/4480;

  // LOW PASS FILTER 25Hz cutoff    
  filteredRPM = filteredRPM2.step(rpm);
  prevTime = currentTime;
  prev_count  = current_count;
  prevRPM = rpm;
}

void DCmotor::onControlTickerCallback(){  
  // 10ms
  float dt = 10.0; // this is an approximated sampling time
  rpmErr = target_fRPM - filteredRPM;
  rpmdErr = rpmErr - prev_rpmErr;
  rpmErrSum = rpmErrSum +  rpmErr;
  ctrlPRM = Kp*rpmErr + Kd*rpmdErr/dt + Ki*rpmErrSum;

  driveMotor(ctrlPRM);

  prev_fRPM = filteredRPM;
  prev_rpmErr = rpmErr;
}

void DCmotor::setDutyCycle(int dutyCycle){
  _dutyCycle =  min(MOTOR_MAX_DUTY,dutyCycle);
}

void DCmotor::setPWM(int dutyCycle){
  // Serial.println(dutyCycle);
  _dutyCycle = min(MOTOR_MAX_DUTY,dutyCycle);
  ledcWrite(_pwmChannel, _dutyCycle);  
}

void DCmotor::setTargetRPM(float target){
  target_fRPM = target;  
}


float DCmotor::getRPM(){
  return rpm;
}

void DCmotor::setMotorDir(int dir){
  // dir = 1; CW
  // dir = -1; CCW
  // dir = 0; stop
  switch(dir){
    case 0:
      digitalWrite(_motorIN1,LOW);
      digitalWrite(_motorIN2,LOW);
      break;
    case 1:
      digitalWrite(_motorIN1,HIGH);
      digitalWrite(_motorIN2,LOW);
      break;
    case -1:
      digitalWrite(_motorIN1,LOW);
      digitalWrite(_motorIN2,HIGH);
      break;
  }
}

void DCmotor::driveMotor(float driveVal){
  int pulseWidth = map(driveVal*10,-1500,1500,-1*MOTOR_MAX_DUTY,MOTOR_MAX_DUTY);
  if(pulseWidth > 0){
    setMotorDir(1);
    setPWM(abs(pulseWidth));
  }else if(pulseWidth < 0){
    setMotorDir(-1);
    setPWM(abs(pulseWidth));
  }else{
    setMotorDir(0);
  }
}
