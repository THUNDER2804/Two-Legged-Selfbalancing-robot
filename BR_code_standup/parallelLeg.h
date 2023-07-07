#include "servoMotor.h"
#include <ArduinoEigen.h>
#include "DCmotor.h"
#include <Ticker.h>


class ParallelLeg{
  public:
    ParallelLeg(){};
    void setParameters(Adafruit_PWMServoDriver &_pwm, float initAngleFront, float initAngleRear, int legId);
    void setFrontServo(uint8_t _pwmChannel,  uint16_t plPos1, uint16_t plPos2, float angPos1, float angPos2, float angMin, float angMax);
    void setRearServo(uint8_t _pwmChannel,  uint16_t plPos1, uint16_t plPos2, float angPos1, float angPos2, float angMin, float angMax);
    ServoMotor frontLeg;
    ServoMotor rearLeg;    

    Adafruit_PWMServoDriver pwm;
    float _initAngleFront;
    float _initAngleRear;

    float _curTheta_front;  
    float _curPsi_front;
    float _curTheta_rear;  
    float _curPsi_rear;
    float endPoint[2];

    int _legId; // 1: right leg, 2: left leg

    void forwardKinematic(double frontAngle, double rearAngle);    
    void moveLeg(double frontAngle, double rearAngle);

    void goHome();

    // Inverse Kinematics
    /* Define member functions and variables for inverse kinematics */
    void inverseKinematic(float xp, float yp);        
    void solveLegIK(float x, float y);   
    float targetEndPoint[2];    // coord of the reference point of the leg end effector. input for IK
    float LegIK_res[4];         // IK result   
    Eigen::Matrix4d  sol;       // Inverse kinematics solutions
    void angleCheck();
    uint8_t isFeasible;         // Target Point Feasibility Check variable
    
    // DC motor object
    DCmotor wheelMotor;

  private:
    float L1 = 103.0;   // upper leg length [mm]
    float L2 = 170.0;   // lower leg length [mm]
    float Dw = 60.0;    // [mm]
};