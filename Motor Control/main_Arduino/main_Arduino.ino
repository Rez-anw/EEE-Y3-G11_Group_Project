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
#define I2C_ADDR 0x08
#define DATA_SIZE 5



////////////////////////////////////////////

// Minimum and maximum motor angles
#define BASE_ANGLE_X 40
#define MIN_ANGLE_X (BASE_ANGLE_X - 5)
#define MAX_ANGLE_X (BASE_ANGLE_X + 5)

#define BASE_ANGLE_Y 50
#define MIN_ANGLE_Y (BASE_ANGLE_Y - 5)
#define MAX_ANGLE_Y (BASE_ANGLE_Y + 5)


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


////////////////////////////////////////////////////////////////////////

// I2C 
int16_t currentX, currentY, desiredX, desiredY;
int16_t receivedData[DATA_SIZE];
int16_t data[4];

void receiveData(int byteCount) {
  if (byteCount >= 9) {
    uint8_t buffer[8];
    Wire.read();
    // Read 8 bytes from the I2C buffer
    for (int i = 0; i < 8 && Wire.available(); i++) {
      buffer[i] = Wire.read();
    }

    // Combine high and low bytes into 16-bit signed integers
    currentX = (int16_t)((buffer[0] << 8) | buffer[1]) * 230 / 1920 ; // current_x
    currentY = (int16_t)((buffer[2] << 8) | buffer[3]) * 230 / 1920 ; // current_y
    desiredX = (int16_t)((buffer[4] << 8) | buffer[5]) * 230 / 1920 ; // desired_x
    desiredY = (int16_t)((buffer[6] << 8) | buffer[7]) * 230 / 1920 ; // desired_y

    receivedData[0] = map(currentX, 0, 1080, 0, 230);
    receivedData[1] = map(currentY, 0, 1080, 0, 230);
    receivedData[2] = map(desiredX, 0, 1080, 0, 230);
    receivedData[3] = map(desiredY, 0, 1080, 0, 230);

  }
}




/////////////////////////////////////////////////////////////////////////////

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
    //Serial.println(tiltX);
    //Serial.println(tiltY);

}

void setup() {
    // Attach servos to pins that can output PWM signals
    // Two motors
    motorX.attach(3);
    motorY.attach(5);

    moveMotorX(motorX, 0);
    moveMotorY(motorY, 0);

    //////////////////////////////////////////////////////////////////
    // I2C
    Wire.begin(0x08);  // Initialize I2C as a slave with address 0x08
    Wire.onReceive(receiveData);  // Register receive callback
    Serial.begin(115200);  // Initialize serial monitor for debugging

    /////////////////////////////////////////////////////////////////////
}

void loop() {
    
    //For running actual
    while (completionStatus == 0){
        motorControl(receivedData[0], receivedData[1], receivedData[2], receivedData[3]);
        //Serial.println(receivedData);
        // Print the received positions to Serial Monitor if you want
        // Print receivedData as CSV for MATLAB
        Serial.print(receivedData[0]); Serial.print(",");
        Serial.print(receivedData[1]); Serial.print(",");
        Serial.print(receivedData[2]); Serial.print(",");
        Serial.println(receivedData[3]); // Last value with newline
        delay(20);
    }


    // // For test only 
    // moveMotorX(motorX, 0);
    // moveMotorY(motorY, 0);
    // delay(10000);
    
    // float currentX[5] = {100, 100, 100, 100, 100};
    // float currentY[5] = {100, 100, 100, 100, 100};
    // float desiredX[5] = {100, 150, 50, 100, 150};
    // float desiredY[5] = {100, 150, 50, 100, 150};

    // for (int i = 0; i < 5; i++){
    //   motorControl(currentX[i], currentY[i], desiredX[i], desiredY[i]);
    //   delay(200);
    // }
    
}
