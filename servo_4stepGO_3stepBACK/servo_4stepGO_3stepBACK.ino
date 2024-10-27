#include <Servo.h>
int fg = 0;
Servo myservo;  // create Servo object to control a servo

int pos = 0;    // variable to store the servo position

void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the Servo object
}

void loop() {
  // // Move clockwise in steps of 20 degrees
  // for (pos = fg; pos <= 40+fg; pos += 10) { // goes from current fg position up to 80 degrees
  //   myservo.write(pos);              // tell servo to go to position in variable 'pos'
  //   delay(1000);                     // waits 1 second for the servo to reach the position
  // }

  // // Set fg to the last position reached (80) for the next cycle
  // fg = 40+fg;

  // // Move anti-clockwise in steps of 20 degrees
  // for (pos = fg; pos >= 10; pos -= 10) { // goes from 80 degrees down to 20 degrees
  //   myservo.write(pos);              // tell servo to go to position in variable 'pos'
  //   delay(1000);                     // waits 1 second for the servo to reach the position
  // }

  // // Set fg to the last position reached (20) for the next cycle
  // fg -= 30;

  // for(int i=10;i<=180;i+=10){
  //    myservo.write(i);  
  //    delay(500);
  // }

  for(int i=20;i<=80;i+=20){
    myservo.write(i);  
    delay(1000);
  }

  for(int i=80;i>=1;i-=26){
    myservo.write(i);  
    delay(1000);
  }
}