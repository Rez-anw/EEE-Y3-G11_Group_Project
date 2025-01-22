// Included Files
#include "driverlib.h"
#include "device.h"
#include "F28x_Project.h"

// PID Controller Structure
typedef struct {
    float kp;
    float ki;
    float kd;
    float prevError;
    float integral;
    float outPut;
    //float outPutMin;
    //float outPutMax;
} PIDController;

// PIDController for X and Y
PIDController pidX, pidY;

// Variables
float currentX, currentY; // Ball's current position
float targetX, targetY; // Target position of the ball
float motorOutputX, motorOutputY;


// Main
void main(void)
{
    // Initialise system, GPIOS,....
    InitSysCtrl();
    InitGpio();
    DINT;
    InitPieCtrl();

    // Initialise PID InitPID(pid, kp, ki, kd, output)
    InitPID(&pidX, 2.0, 0.1, 0.5, 255);
    InitPID(&pidY, 2.0, 0.1, 0.5, 255);

    while(1) {
        // Update PID controller
        motorOutputX = PID(&pidX, targetX, currentX);
        motorOutputY = PID(&pidY, targetX, currentY);
    }

}

// Update PID
float PID(PIDController *pid, float setpoint, float actualValue) {
    float error = setpoint - actualValue;
    pid->integral = pid->integral + error;
    float derivative = error - pid->prevError;

    // PID Output
    float output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);

    pid->prevError = error;

    return output;
}

// Initial PID
void InitPID(PIDController *pid, float kp, float ki, float kd, float outPut)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->prevError = 0.0;
    pid->integral = 0.0;
    pid->outPut = outPut;
}

// Motor control
void ControlMotor()
{

}

// PWM output
void SetPWM()
{

}
