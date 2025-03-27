#include <Wire.h>

// Variables to store the received coordinates
int16_t current_x = 0, current_y = 0;
int16_t desired_x = 0, desired_y = 0;

void setup() {
  Wire.begin(0x08);  // Initialize I2C as a slave with address 0x08 that we set the address in R.P.
  Wire.onReceive(receiveData);  // Register a callback function receiveData, which is automatically triggered when the master device(R.P.) sends data.
  Serial.begin(115200);  // Initialize serial monitor for debugging
}

void loop() {
  // Print the received positions to Serial Monitor if you want
  Serial.print("Current: (");
  Serial.print(current_x);
  Serial.print(", ");
  Serial.print(current_y);
  Serial.print(") → Desired: (");
  Serial.print(desired_x);
  Serial.print(", ");
  Serial.print(desired_y);
  Serial.println(")");

  

  // : Add your control logic here 



  delay(100);  // a delay 
}

// I2C receive callback function
void receiveData(int byteCount) {
  if (byteCount >= 9) {
    uint8_t buffer[8];
    Wire.read();
    // Read 8 bytes from the I2C buffer
    for (int i = 0; i < 8 && Wire.available(); i++) {
      buffer[i] = Wire.read();
    }

    current_x = (int16_t)((buffer[0] << 8) | buffer[1]) /100; 
    current_y = (int16_t)((buffer[2] << 8) | buffer[3]) / 100;
    desired_x = (int16_t)((buffer[4] << 8) | buffer[5]);
    desired_y = (int16_t)((buffer[6] << 8) | buffer[7]);
  }
}
