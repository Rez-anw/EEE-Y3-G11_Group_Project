#include <stdio.h>
#include <stdlib.h>

// WiringPi is outdated, but it can still be installed using:
// sudo apt-get install wiringpi
#include <WiringPi.h>

// This library allows you to generate PWM (Pulse Width Modulation) signals
// on any GPIO pin of a Raspberry Pi, even if the pin does not support hardware PWM.
// To install: sudo apt-get install wiringpi
#include <softPwm.h>

// provides access to the POSIX operating system API
// Delays (usleep(), nanosleep()) for precise motor timing.
// Process control (fork(), getpid()) if you run multiple processes (e.g., camera processing + motor control).
// File operations (access(), read(), write()) for logging or reading configurations.
#include <unistd.h>

// GPIO pins for X-axis and Y-axis motors
#define MOTOR_UP 0
#define MOTOR_DOWN 1
#define MOTOR_LEFT 2
#define MOTOR_RIGHT 3

// Minimum and maximum value for PWM. 
#define MOTOR_ANGLE_MIN 0
//#define MOTOR_ANGLE_MID 12.5
#define MOTOR_ANGLE_MAX 45
#define PWM_RANGE 2000

// PID parameters
float Kp = 1.0;
float Ki = 0.1;
float Kp = 0.05;
float prevError[2] = {0, 0};
float integral[2] = {0, 0};
int x_axis = 0, y_axis = 1;

// Motor control function
void moveMotor(int motorPin, float angle)
{
	// Ensure output stays within motor range (0 to 40 degree)
	if (angle < MOTOR_ANGLE_MIN) angle = MOTOR_ANGLE_MIN;
	if (angle > MOTOR_ANGLE_MAX) angle = MOTOR_ANGLE_MAX;

	// Convert angle to pulse width
	int pulseWidth = 1500 + (angle * 500 / 45);
	pwmWrite(motorPin, pulseWidth);
}

// PID Function to calculate motor position adjustment
float PIDContol(float currentPosition, float desiredPosition, int axis) 
{
	float error = desiredPosition - currentPosition;
	integral[axis] += error;
	float derivative = error - prevError[axis];
	float output =  (Kp * error) + (Ki * integral[axis]) + (Kd * derivative);
	prevError[axis] = error;

	return output;
}

void motorControl(float currentX, float currentY, float desiredX, float desiredY)
{
	// Compute PID output for x and y axis
	float tiltX = PIDContol(currentX, desiredX, x_axis);
	float tiltY = PIDContol(currentY, desiredY, y_axis)

	// Control x-axis motors (left/right)
	if (tiltX > 0){
		moveMotor(MOTOR_LEFT, tiltX);
		moveMotor(MOTOR_RIGHT, MOTOR_ANGLE_MIN);
	} else {
		moveMotor(MOTOR_RIGHT, -tiltX);
		moveMotor(MOTOR_LEFT, MOTOR_ANGLE_MIN);
	}

	// Control y-axis motors (up/down)
	if (tiltX > 0){
		moveMotor(MOTOR_UP, tiltY);
		moveMotor(MOTOR_DOWN, MOTOR_ANGLE_MIN);
	} else {
		moveMotor(MOTOR_DOWN, -tiltY);
		moveMotor(MOTOR_UP, MOTOR_ANGLE_MIN);
	}
}

// Read from a txt file
void readFromFile(int)
{

}

int main()
{
	wiringPiSetup();

	pinMode(MOTOR_UP, PWM_OUTPUT);
	pinMode(MOTOR_DOWN, PWM_OUTPUT);
	pinMode(MOTOR_LEFT, PWM_OUTPUT);
	pinMode(MOTOR_RIGHT, PWM_OUTPUT);

	pwmSetRange(PWM_RANGE);
	pwmSetClock(192);

	// Read CurrentX, CurrentY, DesiredX and DesiredY from txt file
	float currentX = 10;
	float currentY = 10;
	float desiredX = 50;
	float desiredY = 50;

	while(1)
	{
		motorControl(currentX, currentY, desiredX, desiredY);
		usleep(20000);
	}

	return 0;
}

