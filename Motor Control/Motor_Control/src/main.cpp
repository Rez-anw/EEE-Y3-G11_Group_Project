#include <Arduino.h>
#include <Servo.h>

// Servo objects for X-axis and Y-axis motors
Servo motorUp, motorDown, motorLeft, motorRight;


// Minimum and maximum motor angles
#define MOTOR_ANGLE_MIN 0
#define MOTOR_ANGLE_MAX 30

// PID parameters
float Kp = 1.0;
float Ki = 0.1;
float Kd = 0.05;
float prevError[2] = {0, 0};
float integral[2] = {0, 0};
int x_axis = 0, y_axis = 1;

// Function to move motorX to a specific angle
void moveMotor(Servo &motor, float angle) {
    // Constrain the angle within min/max bounds
    angle = constrain(angle, MOTOR_ANGLE_MIN, MOTOR_ANGLE_MAX);

    // Convert angle to servo value (0 to 45 degree)
    int servoAngle = map(angle, 0, 45, MOTOR_ANGLE_MIN, MOTOR_ANGLE_MAX);  // Adjust based on servo range
    motor.write(servoAngle);
}

// PID function to compute motor adjustments
float PIDControl(float currentPosition, float desiredPosition, int axis) {
    float error = desiredPosition - currentPosition;
    integral[axis] += error;
    float derivative = error - prevError[axis];
    float output = (Kp * error) + (Ki * integral[axis]) + (Kd * derivative);
    prevError[axis] = error;

    return output;
}

// Function to control motors based on PID output
void motorControl(float currentX, float currentY, float desiredX, float desiredY) {
    // Compute PID output for X and Y axes
    float tiltX = PIDControl(currentX, desiredX, x_axis);
    float tiltY = PIDControl(currentY, desiredY, y_axis);

    // Without PID control  (Use this)
    // float tiltX = desiredX - currentX;
    // float tiltY = desiredY - currentY;


    // Control X-axis motors (left/right)
    if (tiltX > 0) {
        moveMotor(motorLeft, tiltX);
        moveMotor(motorRight, MOTOR_ANGLE_MIN);
        printf("tiltX %.2f\n", (double)tiltX);
    } else {
        moveMotor(motorRight, -tiltX);
        moveMotor(motorLeft, MOTOR_ANGLE_MIN);
        printf("tiltX %.2f\n", (double)tiltX);
    }

    // Control Y-axis motors (up/down)
    if (tiltY > 0) {
        moveMotor(motorUp, tiltY);
        moveMotor(motorDown, MOTOR_ANGLE_MIN);
        printf("tiltY %.2f\n", (double)tiltY);
    } else {
        moveMotor(motorDown, -tiltY);
        moveMotor(motorUp, MOTOR_ANGLE_MIN);
        printf("tiltY %.2f\n", (double)tiltY);
    }
}

void setup() {
    // Attach servos to pins that can output PWM signals
    motorUp.attach(3);     // Connect motor UP to pin 3
    motorDown.attach(5);  // Connect motor DOWN to pin 5
    motorLeft.attach(9);  // Connect motor LEFT to pin 9
    motorRight.attach(11);  // Connect motor RIGHT to pin 11
}

void loop() {
    // Example values (Replace with actual sensor readings)
    float currentX = 200, desiredX = 100;
    float currentY = 150, desiredY = 100;

    // Control motors based on the desired and current positions
    motorControl(currentX, currentY, desiredX, desiredY);

    delay(20); // Delay for 20 ms
}
