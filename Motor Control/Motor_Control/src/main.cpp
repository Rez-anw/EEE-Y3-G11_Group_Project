#include <Arduino.h>
#include <Servo.h>

// Servo objects for X-axis and Y-axis motors
Servo motorX, motorY;

// Minimum and maximum motor angles
#define MOTOR_ANGLE_MIN 0
#define MOTOR_ANGLE_MAX 90
#define BASE_ANGLE 45 

// PID parameters
float Kp = 1.0;
float Ki = 0.1;
float Kd = 0.5;
float prevError[2] = {0, 0};
float integral[2] = {0, 0};
float integralLimit = 10.0;

int x_axis = 0, y_axis = 1;

float data[5]; 
float prevData[2] = {0, 0};


// PID function to compute motor adjustments
float PIDControl(float currentPosition, float desiredPosition, int axis) {
    float error = desiredPosition - currentPosition;

    // Reset integral if the corresponding value has changed
    if (data[axis + 2] != prevData[axis]){
        integral[axis] = 0;
        prevData[axis] = data[axis];
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
    // Ensure output stays within motor range 
    // if (angle < MOTOR_ANGLE_MIN) angle = MOTOR_ANGLE_MIN;
    // if (angle > MOTOR_ANGLE_MAX) angle = MOTOR_ANGLE_MAX;

    // Convert angle to servo value 
    int servoAngle = map(adjustedAngle, -45, 45, MOTOR_ANGLE_MIN, MOTOR_ANGLE_MAX);  // Adjust based on servo range
    motor.write(servoAngle);
}

// Function to control motors based on PID output
void motorControl(float currentX, float currentY, float desiredX, float desiredY) {
    // Compute PID output for X and Y axes
    //float tiltX = PIDControl(currentX, desiredX, x_axis);
    //float tiltY = PIDControl(currentY, desiredY, y_axis);
    
    // Without PID control  (Use this)
    float tiltX = desiredX - currentX;
    float tiltY = desiredY - currentY;

    // Control Motors
    moveMotor(motorX, tiltX);
    moveMotor(motorY, tiltY);


    // if (tiltX == 0) {
    //     moveMotor(motorX, 45);
    // } else {
    //     moveMotor(motorX, 45 + tiltX);
    // }
    // if (tiltY == 0) {
    //     moveMotor(motorY, 45);
    // } else {
    //     moveMotor(motorY, 45 + tiltY);
    // }
}

void setup() {
    // Attach servos to pins that can output PWM signals
    // Two motors
    motorX.attach(3);
    motorY.attach(5);

    moveMotor(motorX, BASE_ANGLE + 45);
    moveMotor(motorY, BASE_ANGLE + 45);
}

void loop() {

    // Example:
    // data[5] = {currentX, currentY, desiredX, desiredY, completion_statuse};
    float data[5] = {50, 50, 50, 0, 0};
    
    motorX.write(90);
    motorY.write(90);
    delay(5000);

    while (data[4] == 0){
        motorControl(data[0], data[1], data[2], data[3]);
        delay(20);
    }

    // For test only
    float currentX[5] = {100, 100, 100, 100, 100};
    float currentY[5] = {100, 100, 100, 100, 100};
    float desiredX[5] = {100, 150, 50, 100, 150};
    float desiredY[5] = {100, 100, 100, 100, 100};

    // for (int i = 0; i < 5; i++){
    //     motorControl(currentX[i], currentY[i], desiredX[i], desiredY[i]);
    //     delay(2000);
    // }

    // while (data[4] == 0){
    //     //motorX.write(0);
    //     //motorY.write(0);
    //     //delay(5000);
    //     for (int i = 0; i < 5; i++){
    //         motorControl(currentX[i], currentY[i], desiredX[i], desiredY[i]);
    //         delay(2000);
    //     }
    // }

}
