#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>

// Servo objects for X-axis and Y-axis motors
Servo motorX, motorY;

////////////////////////////////////////////
// I2C Communication with raspberry pi
//  Raspberry Pi → Arduino
//  SDA (GPIO2) → A4
//  SCL (GPIO3) → A5
//  GND → GND

#define SLAVE_ADDRESS 0x08
#define DATA_SIZE 4  // 5 floats (4 bytes each)
#define BYTE_SIZE (DATA_SIZE * 4)

byte receivedBytes[BYTE_SIZE];
float receivedData[DATA_SIZE];

void receiveEvent(int numBytes) {
    int index = 0;
    while (Wire.available() && index < BYTE_SIZE) {
        receivedBytes[index++] = Wire.read();  // Read byte from I2C
    }

    // Convert bytes back to floats
    memcpy(receivedData, receivedBytes, BYTE_SIZE);

    // Print received floats
    Serial.print("Received: ");
    for (int i = 0; i < DATA_SIZE; i++) {
        Serial.print(receivedData[i], 2);
        Serial.print(" ");
    }
    Serial.println();
}

////////////////////////////////////////////

// Minimum and maximum motor angles
#define BASE_ANGLE 45
#define MOTOR_ANGLE_MIN (BASE_ANGLE - 20)
#define MOTOR_ANGLE_MAX (BASE_ANGLE + 20)


// PID parameters
float Kp = 5.0;
float Ki = 0.1;
float Kd = 0.5;
float prevError[2] = {0, 0};
float integral[2] = {0, 0};
float integralLimit = 10.0;

int x_axis = 0, y_axis = 1;

// float data[]; 
float prevData[2] = {0, 0};
int completionStatus = 0;


// PID function to compute motor adjustments
float PIDControl(float currentPosition, float desiredPosition, int axis) {
    float error = desiredPosition - currentPosition;

    // Reset integral if the corresponding value has changed
    if (receivedData[axis + 2] != prevData[axis]) {
        integral[axis] = 0;
        prevData[axis] = receivedData[axis+2];
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
void motorControl(float currentX, float currentY, float desiredX, float desiredY) {
    // Compute PID output for X and Y axes
    float tiltX = PIDControl(currentX, desiredX, x_axis);
    float tiltY = PIDControl(currentY, desiredY, y_axis);
    
    // Without PID control  (Use this)
    //float tiltX = desiredX - currentX;
    //float tiltY = desiredY - currentY;

    // Control Motors
    moveMotor(motorX, tiltX);
    moveMotor(motorY, tiltY);

}

void setup() {
    // Attach servos to pins that can output PWM signals
    // Two motors
    motorX.attach(3);
    motorY.attach(5);

    moveMotor(motorX, BASE_ANGLE);
    moveMotor(motorY, BASE_ANGLE);

    ////////////////////////////////////
    // I2C
        Wire.begin(SLAVE_ADDRESS);
    Wire.onReceive(receiveEvent);
    Serial.begin(9600);

    /////////////////////////////////////////
}

void loop() {
    
    // For running actual
    // while (completionStatus == 0){
    //     motorControl(receivedData[0], receivedData[1], receivedData[2], receivedData[3]);
    //     delay(20);
    // }


    // For test only 
    delay(10000);
    
    float currentX[5] = {100, 100, 100, 100, 100};
    float currentY[5] = {100, 100, 100, 100, 100};
    float desiredX[5] = {100, 150, 50, 100, 150};
    float desiredY[5] = {100, 150, 50, 100, 150};

    for (int i = 0; i < 5; i++){
      motorControl(currentX[i], currentY[i], desiredX[i], desiredY[i]);
      delay(2000);
    }
    
}
