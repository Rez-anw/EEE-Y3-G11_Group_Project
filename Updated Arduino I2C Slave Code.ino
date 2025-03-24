#include <Wire.h>
#include <Servo.h>

#define SLAVE_ADDRESS 0x08
#define DATA_SIZE 5        // Number of float values expected
#define BYTE_SIZE (DATA_SIZE * 4)

Servo servoX;
Servo servoY;

int servoX_Pin = 9;
int servoY_Pin = 10;
int angleX = 90;
int angleY = 90;

byte receivedBytes[BYTE_SIZE];
float receivedFloats[DATA_SIZE];

bool commandReceived = false;

void setup() {
    Wire.begin(SLAVE_ADDRESS);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(sendData);
    Serial.begin(9600);

    servoX.attach(servoX_Pin);
    servoY.attach(servoY_Pin);
    servoX.write(angleX);
    servoY.write(angleY);
}

void receiveEvent(int numBytes) {
    int index = 0;
    while (Wire.available() && index < BYTE_SIZE) {
        receivedBytes[index++] = Wire.read();
    }

    // Convert bytes to floats
    memcpy(receivedFloats, receivedBytes, BYTE_SIZE);

    // Debug print
    Serial.print("Received floats: ");
    for (int i = 0; i < DATA_SIZE; i++) {
        Serial.print(receivedFloats[i], 2);
        Serial.print(" ");
    }
    Serial.println();

    // Use the first two values to control X and Y angles
    float xVal = receivedFloats[0];  // Expecting range -1.0 to 1.0
    float yVal = receivedFloats[1];  // Expecting range -1.0 to 1.0

    angleX = constrain(map(xVal * 100, -100, 100, 0, 180), 0, 180);
    angleY = constrain(map(yVal * 100, -100, 100, 0, 180), 0, 180);

    commandReceived = true;
}

void sendData() {
    // Send current angles as bytes (just for feedback, optional)
    Wire.write((byte)angleX);
    Wire.write((byte)angleY);
}

void loop() {
    if (commandReceived) {
        servoX.write(angleX);
        servoY.write(angleY);
        commandReceived = false;
    }

    delay(50);
}
