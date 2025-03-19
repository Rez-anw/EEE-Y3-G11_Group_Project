#include <Servo.h>

Servo myservo, servo2;  // create servo object to control a servo
// twelve servo objects can be created on most boards

float pos ;    // variable to store the servo position

void setup() {
  myservo.attach(3);  // attaches the servo on pin 9 to the servo object
  servo2.attach(5); 
  //servo2.write(45);
}

void loop() {
  //for (pos = 0.0; pos <= 45.0; pos += 0.5) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    //myservo.write(pos);
    myservo.write(45);              // tell servo to go to position in variable 'pos'
    servo2.write(45);
    //delay(15);                       // waits 15ms for the servo to reach the position
  //}
 
}