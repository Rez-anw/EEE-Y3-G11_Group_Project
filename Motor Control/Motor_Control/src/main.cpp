#include <Arduino.h>
#include <Servo.h>

// Servo objects for X-axis and Y-axis motors
Servo motorX, motorY;

// Minimum and maximum motor angles
#define BASE_ANGLE 45
#define MOTOR_ANGLE_MIN 20
#define MOTOR_ANGLE_MAX 70


// PID parameters
float Kp = 5.0;
float Ki = 0.1;
float Kd = 0.5;
float prevError[2] = {0, 0};
float integral[2] = {0, 0};
float integralLimit = 10.0;

int x_axis = 0, y_axis = 1;

float data[5]; 
float prevData[2] = {0, 0};


// PID function to compute motor adjustments
float PIDControl(float error, int axis) {
    //float error = desiredPosition - currentPosition;

    // Reset integral if the corresponding value has changed
    if (data[axis] <= 0.5){
        integral[axis] = 0;
        // prevData[axis] = data[axis];
    }

    integral[axis] += error;
    integral[axis] = constrain(integral[axis], -integralLimit, integralLimit);

    float derivative = error - prevError[axis];
    float output = (Kp * error) + (Ki * integral[axis]) + (Kd * derivative);
    prevError[axis] = error;

    return output;
}

// Function to move motorX to a specific angle
void moveMotor(Servo &motor, float angle) {
    // Adjust angle relative to the base angle
    float adjustedAngle = BASE_ANGLE + angle;

    // Constrain the angle within min/max bounds
    adjustedAngle = constrain(adjustedAngle, MOTOR_ANGLE_MIN, MOTOR_ANGLE_MAX);

    motor.write(adjustedAngle);
}

// Function to control motors based on PID output
void motorControl(float errorX, float errorY) {
    // Compute PID output for X and Y axes
    float tiltX = PIDControl(errorX, x_axis);
    float tiltY = PIDControl(errorY, y_axis);
    
    // Without PID control  (Use this)
    //float tiltX = errorX;
    //float tiltY = errorY;

    // Control Motors
    moveMotor(motorX, tiltX);
    moveMotor(motorY, tiltY);

}

void setup() {
    // Attach servos to pins that can output PWM signals
    // Two motors
    motorX.attach(3);
    motorY.attach(5);

    moveMotor(motorX, 45);
    moveMotor(motorY, 45);
}

void loop() {
    
    motorX.write(45);
    motorY.write(45);
    delay(5000);

    float dataX[6] = {10, 7, 0, 8, -8, -10};
    float dataY[6] = {10, 7, 0, 8, -8, -10};
    for (int i = 0; i < 6; i++){
        motorControl(dataX[i], dataY[i]);
        delay(2000);
    }


}
