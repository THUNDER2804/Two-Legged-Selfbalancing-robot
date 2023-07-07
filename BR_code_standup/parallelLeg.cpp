#include <algorithm>
#include "parallelLeg.h"


void ParallelLeg::setParameters(Adafruit_PWMServoDriver &_pwm, float initAngleFront, float initAngleRear, int legId) {
  pwm = _pwm;
  _initAngleFront = initAngleFront;
  _initAngleRear = initAngleRear;
  _legId = legId;

  _curTheta_front = 0;
  _curPsi_front = 0;
  _curTheta_rear = 0;
  _curPsi_rear = 0;
}

void ParallelLeg::setFrontServo(uint8_t _pwmChannel, uint16_t plPos1, uint16_t plPos2, float angPos1, float angPos2, float angMin, float angMax) {
  frontLeg.setupParameters(_pwmChannel, pwm, plPos1, plPos2, angPos1, angPos2);
  frontLeg.setAngleLimits(angMin, angMax);  // you need to find proper limits
}

void ParallelLeg::setRearServo(uint8_t _pwmChannel, uint16_t plPos1, uint16_t plPos2, float angPos1, float angPos2, float angMin, float angMax) {
  rearLeg.setupParameters(_pwmChannel, pwm, plPos1, plPos2, angPos1, angPos2);
  rearLeg.setAngleLimits(angMin, angMax);  // you need to find proper limits
}

void ParallelLeg::moveLeg(double frontAngle, double rearAngle) {
  double the1, the2;
  the1 = frontAngle;
  the2 = rearAngle;
  // Mechanical angle limit consideration
  the1 = min(max((double)frontLeg.angleLimit_min,the1), (double)frontLeg.angleLimit_max);
  the2 = min(max((double)rearLeg.angleLimit_min,the2), (double)rearLeg.angleLimit_max);

  frontLeg.setAngleDegree(the1);
  rearLeg.setAngleDegree(the2);

  forwardKinematic(the1, the2);
}



void ParallelLeg::forwardKinematic(double frontAngle, double rearAngle) {
  // angle inputs are in degree
  float A, B, C, theta1, theta2, temp_psi1, temp_psi2;
  theta1 = frontAngle * PI / 180;
  theta2 = rearAngle * PI / 180;
  A = 2 * L1 * L2 * (cos(theta2) - cos(theta1)) - 4 * L2 * Dw;
  B = 2 * L1 * L2 * (sin(theta2) - sin(theta1));
  C = pow(L1 * (cos(theta2) - cos(theta1)) - 2 * Dw, 2.0) + pow(L1 * (sin(theta2) - sin(theta1)), 2.0);

  if (_legId == 1) {  // Right leg: downward
    temp_psi1 = 2 * atan2(-B - sqrt(B*B - C*C + A*A), -C - A) - theta1;
    temp_psi2 = 2 * atan2(-B - sqrt(B*B - C*C + A*A), C - A) - theta2;
  } else if (_legId == 2) {  // Left leg: upward
    temp_psi1 = 2 * atan2(-B + sqrt(B*B - C*C + A*A), -C - A) - theta1;
    temp_psi2 = 2 * atan2(-B + sqrt(B*B - C*C + A*A), C - A) - theta2;
  } else {
    return;
  }

  // update joint angle configurations
  _curTheta_front = frontAngle;
  _curPsi_front   = temp_psi1*180/PI;

  _curTheta_rear = rearAngle;
  _curPsi_rear   =  temp_psi2*180/PI;

  // update end-effector position
  endPoint[0] = Dw + L1*cos(theta1) + L2*cos(temp_psi1+theta1);
  endPoint[1] = L1*sin(theta1) + L2*sin(temp_psi1+theta1);  
}


void ParallelLeg::goHome() {
  moveLeg(_initAngleFront, _initAngleRear);
}

/* Define your new functions here */
void ParallelLeg::inverseKinematic(float xp, float yp){
    // front leg
  float A = L1*L1 - L2*L2 + xp*xp + yp*yp - 2*Dw*xp + Dw*Dw;
  float B = -2*yp*L1;
  float C = 2*L1*(Dw- xp);
  float D = B*B - A*A + C*C;

  if(D < 0) { // solution with imaginary values
    isFeasible = 0;
    return;
  }

  float theta1_1 = 2 * atan2(-B + sqrt(D), A-C);
  float psi1_1   = atan2(yp - L1*sin(theta1_1), xp - L1*cos(theta1_1) - Dw);
  float theta2_1 = psi1_1 - theta1_1;

  float theta1_2 = 2 * atan2(-B - sqrt(D), A-C);
  float psi1_2   = atan2(yp - L1*sin(theta1_2), xp - L1*cos(theta1_2) - Dw);
  float theta2_2 = psi1_2 - theta1_2;

  A = L1*L1 - L2*L2 + xp*xp + yp*yp + 2*Dw*xp + Dw*Dw;
  B = -2*yp*L1;
  C = 2*L1*(-Dw - xp);
  D = B*B - A*A + C*C;
  if(D < 0) { // solution with imaginary values
    isFeasible = 0;
    return;
  }

  float theta3_1 = 2 * atan2(-B + sqrt(D), A-C);
  float psi2_1   = atan2(yp - L1*sin(theta3_1), xp - L1*cos(theta3_1) + Dw);
  float theta4_1 = psi2_1 - theta3_1;

  float theta3_2 = 2 * atan2(-B - sqrt(D), A-C);
  float psi2_2   = atan2(yp - L1*sin(theta3_2), xp - L1*cos(theta3_2) + Dw);
  float theta4_2 = psi2_2 - theta3_2;

  sol(0,0) = theta1_1; sol(0,1) = theta2_1; sol(0,2) = theta3_1; sol(0,3) = theta4_1;
  sol(1,0) = theta1_1; sol(1,1) = theta2_1; sol(1,2) = theta3_2; sol(1,3) = theta4_2;  // <-- for right leg
  sol(2,0) = theta1_2; sol(2,1) = theta2_2; sol(2,2) = theta3_1; sol(2,3) = theta4_1;  // <-- for left leg
  sol(3,0) = theta1_2; sol(3,1) = theta2_2; sol(3,2) = theta3_2; sol(3,3) = theta4_2;
  isFeasible = 1;
}

void ParallelLeg::solveLegIK(float x, float y){
  // x, y for right leg end effector --> left leg will be synchronized 
  targetEndPoint[0] = x;
  targetEndPoint[1] = y;

  isFeasible = 0;
  inverseKinematic(x, y);
  if(isFeasible == 0) return;  // if the solution of IK is not feasible, do nothing

  if(_legId == 1){// right leg
    LegIK_res[0] = sol(1,0)*180/M_PI;
    LegIK_res[1] = sol(1,1)*180/M_PI;
    LegIK_res[2] = sol(1,2)*180/M_PI;
    LegIK_res[3] = sol(1,3)*180/M_PI;
  }else if(_legId == 2){  // left leg
    LegIK_res[0] = sol(2,0)*180/M_PI;
    LegIK_res[1] = sol(2,1)*180/M_PI;
    LegIK_res[2] = sol(2,2)*180/M_PI;
    LegIK_res[3] = sol(2,3)*180/M_PI;
  }
  angleCheck();
}

void ParallelLeg::angleCheck(){  // recalculate the inverse kinematic result compatible to the servo motor range
  if(frontLeg.angleLimit_min > LegIK_res[0] || frontLeg.angleLimit_max < LegIK_res[0]){
    if(frontLeg.angleLimit_min <= LegIK_res[0] + 360 && frontLeg.angleLimit_max >= LegIK_res[0] + 360){
      LegIK_res[0] = LegIK_res[0] + 360;
    }else if(frontLeg.angleLimit_min <= LegIK_res[0] - 360 && frontLeg.angleLimit_max >= LegIK_res[0] - 360){
      LegIK_res[0] = LegIK_res[0] - 360;
    }else{
      LegIK_res[0] = _curTheta_front;
      LegIK_res[1] = _curPsi_front;
      LegIK_res[2] = _curTheta_rear;
      LegIK_res[3] = _curPsi_rear;
      return;
    }
  }
 
  if(rearLeg.angleLimit_min > LegIK_res[2] || rearLeg.angleLimit_max < LegIK_res[2]){
    if(rearLeg.angleLimit_min <= LegIK_res[2] + 360 && rearLeg.angleLimit_max>= LegIK_res[2] + 360){
      LegIK_res[2] = LegIK_res[2] + 360;
    }else if(rearLeg.angleLimit_min <= LegIK_res[2] - 360 && rearLeg.angleLimit_max  >= LegIK_res[2] - 360){
      LegIK_res[2] = LegIK_res[2] - 360;
    }else{
      LegIK_res[0] = _curTheta_front;
      LegIK_res[1] = _curPsi_front;
      LegIK_res[2] = _curTheta_rear;
      LegIK_res[3] = _curPsi_rear;
      return;
    }
  }
}



