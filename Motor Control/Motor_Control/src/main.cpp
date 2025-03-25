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
#define BASE_ANGLE_X 40
#define MIN_ANGLE_X (BASE_ANGLE_X - 10)
#define MAX_ANGLE_X (BASE_ANGLE_X + 10)

#define BASE_ANGLE_Y 50
#define MIN_ANGLE_Y (BASE_ANGLE_Y - 10)
#define MAX_ANGLE_Y (BASE_ANGLE_Y + 10)


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
void moveMotorX(Servo &motor, float angle) {
    // Adjust angle relative to the base angle
    float adjustedAngle = BASE_ANGLE_X + angle;

    // Constrain the angle within min/max bounds
    adjustedAngle = constrain(adjustedAngle, MIN_ANGLE_X, MAX_ANGLE_X);

    motor.write(adjustedAngle);
}

void moveMotorY(Servo &motor, float angle) {
    // Adjust angle relative to the base angle
    float adjustedAngle = BASE_ANGLE_Y + angle;

    // Constrain the angle within min/max bounds
    adjustedAngle = constrain(adjustedAngle, MIN_ANGLE_Y, MAX_ANGLE_Y);

    motor.write(adjustedAngle);
}

// Function to control motors based on PID output
void motorControl(float currentX, float currentY, float desiredX, float desiredY) {
    // Compute PID output for X and Y axes
    // float tiltX = PIDControl(currentX, desiredX, x_axis);
    // float tiltY = PIDControl(currentY, desiredY, y_axis);
    
    // Without PID control  (Use this)
    float tiltX = desiredX - currentX;
    float tiltY = desiredY - currentY;

    // Control Motors
    moveMotorX(motorX, tiltX);
    moveMotorY(motorY, tiltY);

}

void setup() {
    // Attach servos to pins that can output PWM signals
    // Two motors
    motorX.attach(3);
    motorY.attach(5);

    moveMotorX(motorX, BASE_ANGLE_X);
    moveMotorY(motorY, BASE_ANGLE_Y);

    ////////////////////////////////////
    // I2C
    Wire.begin(SLAVE_ADDRESS);
    // Or use external pull up resistor: 4.7 kilo_ohms (GPIO2 > 3.3V and GPIO3 > 3.3V)
    //pinMode(A4, INPUT_PULLUP);  // Enable internal pull-up on SDA
    //pinMode(A5, INPUT_PULLUP);  // Enable internal pull-up on SCL
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
    delay(5000);
    
    float currentX[5] = {100, 100, 100, 100, 100};
    float currentY[5] = {100, 100, 100, 100, 100};
    float desiredX[5] = {100, 150, 50, 100, 150};
    float desiredY[5] = {100, 150, 50, 100, 150};

    for (int i = 0; i < 5; i++){
      motorControl(currentX[i], currentY[i], desiredX[i], desiredY[i]);
      delay(2000);
    }
    
}
