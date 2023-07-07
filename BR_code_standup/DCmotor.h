#include <ESP32Encoder.h>
#include <Ticker.h>

#define MOTOR_MAX_DUTY 255
#define MAX_ACC 1500

class  FilterBuLp2
{
	public:
		FilterBuLp2()
		{
			v[0]=0.0;
			v[1]=0.0;
			v[2]=0.0;
		}
	private:
		float v[3];
	public:
		float step(float x) //class II 
		{
			v[0] = v[1];
			v[1] = v[2];
			v[2] = (5.542717210280685182e-3 * x)
				 + (-0.80080264666570755150 * v[0])
				 + (1.77863177782458481424 * v[1]);
			return 
				 (v[0] + v[2])
				+2 * v[1];
		}
};


class DCmotor{
  public:   
    DCmotor(){};
    void setupParameters(int pwmPin, int motorIN1, int motorIN2, int pwmChannel, int encA, int encB);    
    void init();
    void setDutyCycle(int dutyCycle);
    void setPWM(int dutyCycle);    
    void setMotorDir(int dir);
    void driveMotor(float driveVal);  
    float getRPM();
    void onTickerCallback();
    void onControlTickerCallback();
    void setTargetRPM(float target);
    volatile unsigned long delta_time;    

    Ticker rpmTicker;
    Ticker controlTicker;
    FilterBuLp2 filteredRPM2;

  // private:
    // Parameters for PWM signal    
    int _pwmFreq    = 5000;
    int _pwmChannel = 0;
    int _pwmResol   = 8;
    float currentPWMfreq;
    int _dutyCycle;

    // pins
    int _pwmOutputPin;
    int _motorIN1;
    int _motorIN2;

    // Encoder object
    ESP32Encoder encoder;    
    
    unsigned long prevTime;
    unsigned long currentTime;
    
    int64_t prev_count;
    int64_t current_count;
    int64_t diff_cnt;
    float rpm;    
    float filteredRPM;       
    float prevRPM;   

    float prev_fRPM;   
    float target_fRPM;   
    float Kp;
    float Kd;
    float Ki;
    float rpmErr;
    float rpmdErr;
    float prev_rpmErr;
    float rpmErrSum;
    float ctrlPRM;
    float accRPM;
};
