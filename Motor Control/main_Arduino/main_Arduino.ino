#include <Arduino.h>
#include <Servo.h>
#include <Wire.h> 
#include <stdio.h>
#include <stdlib.h>

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
#define diffX 8

#define BASE_ANGLE_X 45
#define MIN_ANGLE_X (BASE_ANGLE_X - diffX)
#define MAX_ANGLE_X (BASE_ANGLE_X + diffX)

#define diffY 8
#define BASE_ANGLE_Y 40
#define MIN_ANGLE_Y (BASE_ANGLE_Y - diffY ) 
#define MAX_ANGLE_Y (BASE_ANGLE_Y + diffY)


// PID parameters
float Kp[2] = {1, 1};
float Ki[2] = {0.1, 0.1};
float Kd[2] = {0.5,0.5};
float Kv[2] = {0, 0};
float prevError[2] = {0, 0};
float integral[2] = {0, 0};
float integralLimit = 5.0;

int x_axis = 0, y_axis = 1;

// float data[]; 
float prevData[2] = {0, 0};
int completionStatus = 0;  

const char *filename = "C:\EEE\GP-G11\EEE-Y3-G11_Group_Project\Motor Control\main_Arduino\data.csv";  // Specify the CSV file name


float tiltX; 
float tiltY;


//  Variables for EMA
float alpha = 0.7; // Smoothing factor for EMA (adjustable)
float emaCurrentX = 0.0, emaCurrentY = 0.0;
float emaDesiredX = 0.0, emaDesiredY = 0.0;

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
    currentX = (int16_t)((buffer[0] << 8) | buffer[1]); // current_x
    currentY = (int16_t)((buffer[2] << 8) | buffer[3]); // current_y
    desiredX = (int16_t)((buffer[4] << 8) | buffer[5]); // desired_x
    desiredY = (int16_t)((buffer[6] << 8) | buffer[7]); // desired_y

    receivedData[0] = map(currentX, 0, 1200, 0, 180);
    receivedData[1] = map(currentY, 0, 1200, 0, 180);
    receivedData[2] = map(desiredX, 0, 1200, 0, 180);
    receivedData[3] = map(desiredY, 0, 1200, 0, 180); 

    //   receivedData[0] = currentX;
    //   receivedData[1] = currentY;
    //   receivedData[2] = desiredX;
    //   receivedData[3] = desiredY;  

      smoothData(currentX, currentY, desiredX, desiredY);

  }
}

// Function for EMA smoothing
void smoothData(int currentX, int currentY, int desiredX, int desiredY) {
  // Update EMA values for current positions
  emaCurrentX = alpha * currentX + (1 - alpha) * emaCurrentX;
  emaCurrentY = alpha * currentY + (1 - alpha) * emaCurrentY;

  // Update EMA values for desired positions
  emaDesiredX = alpha * desiredX + (1 - alpha) * emaDesiredX;
  emaDesiredY = alpha * desiredY + (1 - alpha) * emaDesiredY;
}


/////////////////////////////////////////////////////////////////////////////

// Add velocity gain parameter

float PIDControl(int currentPosition, int desiredPosition, int currentVelocity, int axis) {
    float error = desiredPosition - currentPosition;

    // Reset integral if the target position has changed
    if (receivedData[axis + 2] != prevData[axis]) {
        integral[axis] = 0;
        prevData[axis] = receivedData[axis + 2];
    }

    // Accumulate integral if the error is small
    // if (abs(error) < 5) {
    //     integral[axis] += error;
    // } else {
    //     integral[axis] = 0;
    // }

    integral[axis] += error;
    // Constrain integral to prevent windup
    integral[axis] = constrain(integral[axis], -integralLimit, integralLimit);

    // Calculate derivative (change in error)
    float derivative = error - prevError[axis];

    // Compute the velocity feedforward term
    float velocityCompensation = Kv[axis] * currentVelocity;

    // Compute final output including velocity
    float output = (Kp[axis] * error) + (Ki[axis] * integral[axis]) + (Kd[axis] * derivative) + velocityCompensation;
    
    prevError[axis] = error;  // Store current error for next loop

    return output;
}


// Function to move motorX to a specific angle
void moveMotorX(Servo &motor, int angle) {
    //int adjustedAngle = constrain(angle, -15, 15);
    // Adjust angle relative to the base angle
    int adjustedAngle = BASE_ANGLE_X + angle;

    // Apply jitter: small random oscillation to prevent stiction
    //int jitter = random(-1, 2);

    // Constrain the angle within min/max bounds
    adjustedAngle = constrain(adjustedAngle, MIN_ANGLE_X, MAX_ANGLE_X);
    //adjustedAngle = constrain(adjustedAngle + jitter, MIN_ANGLE_X, MAX_ANGLE_X);

    motor.write(adjustedAngle);
    //Serial.print(adjustedAngle);
}

// Function to move motorX to a specific angle
void moveMotorY(Servo &motor, int angle) {

    //int adjustedAngle = constrain(angle, -15, 15);
    // Adjust angle relative to the base angle
    int adjustedAngle = BASE_ANGLE_Y - angle;

    // Apply jitter: small random oscillation to prevent stiction
    //int jitter = random(-1, 2);

    // Constrain the angle within min/max bounds
    adjustedAngle = constrain(adjustedAngle, MIN_ANGLE_Y, MAX_ANGLE_Y);
    //adjustedAngle = constrain(adjustedAngle + jitter, MIN_ANGLE_Y, MAX_ANGLE_Y);

    motor.write(adjustedAngle);
    //Serial.print(adjustedAngle);
}

// Function to control motors based on PID output
void motorControl(int currentX, int currentY, int desiredX, int desiredY) {
    // Compute PID output for X and Y axes
    tiltX = PIDControl(emaCurrentX, emaDesiredX, (emaCurrentX - prevError[x_axis]), x_axis);
    tiltY = PIDControl(emaCurrentY, emaDesiredY, (emaCurrentY - prevError[y_axis]), y_axis);

    moveMotorX(motorX, tiltX); 
    moveMotorY(motorY, tiltY);

    
    // Without PID control  (Use this)
    // tiltX = (desiredX - currentX) ;
    // tiltY = (desiredY - currentY);
    // moveMotorX(motorX, tiltX);
    // moveMotorY(motorY, tiltY);

} 

// Function to write X1 and Y1 to a CSV file
void write_to_csv(int X1, int Y1,int X2,int Y2,const char *filename) {
    FILE *file = fopen(filename, "a+");  // Open the file in append mode
    if (file == NULL) {
        perror("Error opening file");
        return;
    }

    // Write the data to the file in CSV format
    fprintf(file, "%d,%d,%d,%d\n", X1,Y1,X2,Y2);

    // Close the file
    fclose(file);
}


void setup() {
    // Attach servos to pins that can output PWM signals
    // Two motors
    motorX.attach(6);
    motorY.attach(3);

    moveMotorX(motorX, 0);
    moveMotorY(motorY, 0);

    //////////////////////////////////////////////////////////////////
    // I2C
    Wire.begin(0x08);  // Initialize I2C as a slave with address 0x08
    Wire.onReceive(receiveData);  // Register receive callback
    Serial.begin(2000000);  // Initialize serial monitor for debugging

    /////////////////////////////////////////////////////////////////////
}

void loop() {
    
    //For running actual
    while (completionStatus == 0){
        motorControl(receivedData[0], receivedData[1], receivedData[2], receivedData[3]); 
        //motorControl(emaCurrentX, emaCurrentY, emaDesiredX, emaDesiredY);
        // write_to_csv( emaCurrentX, emaCurrentY, emaDesiredX, emaDesiredY, filename);
        delay(100);
        moveMotorX(motorX, 0);
        //delay(1);
        moveMotorY(motorY, 0);
        //motorControl(receivedData[0]/5, receivedData[1]/5, receivedData[2]/5, receivedData[3]/5);


        // Print the received positions to Serial Monitor if you want 
        Serial.print("X1: ");
        Serial.print(currentX); Serial.print(",   "); 
        Serial.print("Y1: ");
        Serial.print(currentY); Serial.print(",   "); 
        Serial.print("X2: ");
        Serial.print(desiredX); Serial.print(",   "); 
        Serial.print("Y2: ");
        Serial.print(desiredY), Serial.print(",   "); // Last value with newline
        Serial.print("diff_X: ");
        Serial.print(tiltX), Serial.print(",   "); 
        Serial.print("diff_Y: ");
        Serial.print(tiltY), Serial.println(""); 
      
        delay(10);
    }


    // For test only 
    // moveMotorX(motorX, 0);
    // moveMotorY(motorY, 0);
    // delay(3000);
    
    // float currentX[5] = {100, 100, 100, 100, 100};
    // float currentY[5] = {100, 100, 100, 100, 100};
    // float desiredX[5] = {100, 150, 50, 100, 150};
    // float desiredY[5] = {100, 150, 50, 100, 150};

    // for (int i = 0; i < 5; i++){
    //   motorControl(currentX[i], currentY[i], desiredX[i], desiredY[i]);
    //   delay(4000);
    // }
    
}