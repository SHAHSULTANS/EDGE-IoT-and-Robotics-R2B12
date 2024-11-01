#include <Servo.h>

Servo myservo;  // create Servo object to control a servo
int pos = 90;   // variable to store the servo position, starting at 90 degrees (center position)

void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the Servo object
  myservo.write(pos); // set initial position to 90 degrees
}

void loop() {
  pos += 4;                // move clockwise by 4 degrees
  myservo.write(pos);      // set servo to new position
  delay(15);               // wait 15 ms for the servo to reach the position

  pos -= 3;                // move anti-clockwise by 3 degrees
  myservo.write(pos);      // set servo to new position
  delay(15);               // wait 15 ms for the servo to reach the position
}