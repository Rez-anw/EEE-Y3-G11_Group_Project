Arduino Code (I2C Slave + Continuous Control)

#include <Wire.h>
#include <Servo.h>

Servo servoX; // X-axis tilt control
Servo servoY; // Y-axis tilt control

int servoX_Pin = 9;  // Connect to Arduino PWM pin
int servoY_Pin = 10; // Connect to Arduino PWM pin
int angleX = 90;  // Initial position of servo X
int angleY = 90;  // Initial position of servo Y

bool commandReceived = false; // Flag for new command

void setup() {
    Wire.begin(8);  // Set Arduino as I2C Slave with address 8
    Wire.onReceive(receiveData);
    Wire.onRequest(sendData);
    Serial.begin(9600);

    // Attach servos to pins
    servoX.attach(servoX_Pin);
    servoY.attach(servoY_Pin);

    // Set initial positions
    servoX.write(angleX);
    servoY.write(angleY);
}

void receiveData(int byteCount) {
    while (Wire.available()) {
        char command = Wire.read();  // Read command from Raspberry Pi
        Serial.print("Received Command: ");
        Serial.println(command);
        commandReceived = true;

        // Move servos based on received command
        if (command == 'L') {
            Serial.println("Tilting Left");
            angleX = constrain(angleX - 5, 0, 180);
        } 
        else if (command == 'R') {
            Serial.println("Tilting Right");
            angleX = constrain(angleX + 5, 0, 180);
        } 
        else if (command == 'U') {
            Serial.println("Tilting Up");
            angleY = constrain(angleY - 5, 0, 180);
        } 
        else if (command == 'D') {
            Serial.println("Tilting Down");
            angleY = constrain(angleY + 5, 0, 180);
        } 
        else {
            Serial.println("Unknown Command");
        }
    }
}

// Function to send real-time servo position to Raspberry Pi
void sendData() {
    Wire.write(angleX); // Send X-angle
    Wire.write(angleY); // Send Y-angle
}

void loop() {
    if (commandReceived) {
        // Apply servo movements only when a new command is received
        servoX.write(angleX);
        servoY.write(angleY);
        commandReceived = false; // Reset flag
    }

    delay(50); // Short delay to ensure smooth movement
}
