#include <Servo.h> 
#include <Wire.h> 

#define LengthBoard 26.0 
#define WidthBoard 18.8
#define G 9.81
#define PI 3.141592653589793

const uint8_t address = 0x08; 
float X_pos = 0; 
float Y_pos = 0;  
float L = 12.0; 
float r = 0.02;   

Servo servo1; 
Servo servo2;

unsigned long prev_time = 0;
float delta_t = 0.01;  // Default time interval to avoid division by zero

void Recieve_data(int bytes) { 
    if (Wire.available() >= 2) {  // Ensure at least 2 bytes are available
        X_pos = (float)Wire.read();  
        Y_pos = (float)Wire.read();
        Serial.print("X: "); Serial.print(X_pos);
        Serial.print(" Y: "); Serial.println(Y_pos);
    }
}

void setup() {
    Serial.begin(9600);
    Wire.begin(address); 
    Wire.onReceive(Recieve_data); // Set up I2C receive handler
    servo1.attach(3); 
    servo2.attach(5);
    prev_time = millis();
}

void loop() { 
    unsigned long current_time = millis();
    delta_t = (current_time - prev_time) / 1000.0; // Convert to seconds
    prev_time = current_time;

    if (delta_t > 0) {
        float acceleration_x = X_pos / (delta_t * delta_t);
        float acceleration_y = Y_pos / (delta_t * delta_t);

        // Taylor Series Approximation for asin(x) ≈ x + x^3/6
        float arm1 = (acceleration_y / G) * LengthBoard + pow(acceleration_y / G, 3) / 6 * LengthBoard;
        float arm2 = (acceleration_x / G) * WidthBoard + pow(acceleration_x / G, 3) / 6 * WidthBoard;

        // Compute approximation for acos(x) using the Taylor series: acos(x) ≈ π/2 - (x + x^3/6)
        float angle1_rad = (PI / 2) - ((arm1 - L) / r + pow((arm1 - L) / r, 3) / 6);
        float angle2_rad = (PI / 2) - ((arm2 - L) / r + pow((arm2 - L) / r, 3) / 6);

        // Convert radians to degrees
        float servo_angle1 = angle1_rad * (180 / PI); 
        float servo_angle2 = angle2_rad * (180 / PI);

        servo1.write(servo_angle1); 
        servo2.write(servo_angle2);

        Serial.print("Servo1 Angle: "); Serial.print(servo_angle1);
        Serial.print(" Servo2 Angle: "); Serial.println(servo_angle2);
    }
    delay(50);
}