#include <Servo.h>

Servo servoX, servoY;  // create servo object to control a servo
// twelve servo objects can be created on most boards

float pos ;    // variable to store the servo position

void setup() {
  servoX.attach(3);  // attaches the servo on pin 9 to the servo object
  servoY.attach(5); 
  //servoY.write(51); // Base starting angles 
  //servoX.write(38); 
  Serial.begin(9600); 
}

void loop() { 
  pos=analogRead(1);
 //for (pos = 40.0; pos <= 45.0; pos -= 1.0) { // goes from 0 degrees to 180 degrees
     //in steps of 1 degree
  servoX.write(40);  
  
  servoY.write(50); 
  Serial.println(pos);              // tell servo to go to position in variable 'pos'
   // Serial.println(pos); 
   // delay(3000);                      // waits 15ms for the servo to reach the position
  //}
 
}