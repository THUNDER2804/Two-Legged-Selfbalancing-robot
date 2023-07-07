#include <Adafruit_PWMServoDriver.h>

class ServoMotor{
  public:
    // called this way, it uses the default address 0x40
    Adafruit_PWMServoDriver pwm;
    uint8_t pwmChannel;  
    uint16_t currentPulseLength;
    
    ServoMotor(){};    
    void setupParameters(uint8_t _pwmChannel, Adafruit_PWMServoDriver &_pwm, uint16_t plPos1, uint16_t plPos2, float angPos1, float angPos2);
    void setPWMpulse(uint16_t pl);
    void setAngleDegree(float degree);    
    void setAngleLimits(float ang_min, float ang_max);
    
    float angleLimit_min;
    float angleLimit_max;

  protected:
    uint16_t pulseLength_Pos1;   
    uint16_t pulseLength_Pos2;   
    uint16_t pulseLength_Min;   
    uint16_t pulseLength_Max;  
    float angle_Pos1;
    float angle_Pos2;
    
};