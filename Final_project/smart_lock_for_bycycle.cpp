#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;




#define SOLENOID_PIN 26                 // GPIO pin connected to the Base of the transistor
const long ownerDeviceNumber = 123456;  // Replace with the owner's device number (as a number)
#define BUZZER_PIN 12
long inputNumber = 0;     // Variable to store the incoming number
bool solenoidOn = false;  // Track the state of the solenoid

void setup() {
  Serial.begin(9600);
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  delay(100);
  pinMode(SOLENOID_PIN, OUTPUT);   
  pinMode(BUZZER_PIN, OUTPUT); // Set GPIO as output
  digitalWrite(SOLENOID_PIN, LOW);  // Ensure the solenoid is off initially

  

  Serial.println("Waiting for device number...");
}

void loop() {
  // Read from the Serial Monitor and build the number
  while (Serial.available()) {
    char c = Serial.read();  // Read the incoming character
    delay(200);
    if (c == '\n') {         // Check for the newline character (end of input)
      Serial.print("Device Number Received: ");
      Serial.println(inputNumber);

      // Check if the incoming number matches the owner's device number
      if (inputNumber == ownerDeviceNumber) {
        Serial.println("Owner's device connected!");
        solenoidOn = true;                 // Set solenoid state to ON
        digitalWrite(SOLENOID_PIN, HIGH);  // Turn the solenoid on
      } else {
        Serial.println("Unknown device. Ignoring...");
      }

      inputNumber = 0;                             // Clear the number buffer
    } else if (c >= '0' && c <= '9') {             // Check if the character is a digit
      inputNumber = inputNumber * 10 + (c - '0');  // Convert char to number and build the number
    }

    // If solenoid is ON and we receive a "0", turn it off
    if (solenoidOn && c == '0') {


      solenoidOn = false;
      digitalWrite(SOLENOID_PIN, LOW);  // Turn the solenoid off
      Serial.println("Solenoid turned off.");
    }
  }

  if (!solenoidOn) {
    delay(200);
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    Serial.print("Accel X: "); Serial.print(a.acceleration.x); Serial.print(" m/s^2, ");
    Serial.print("Y: "); Serial.print(a.acceleration.y); Serial.print(" m/s^2, ");
    Serial.print("Z: "); Serial.print(a.acceleration.z); Serial.println(" m/s^2");
    Serial.println("");


    if (abs(a.acceleration.x) > 2 || abs(a.acceleration.y) > 2 || abs(a.acceleration.z) > 11) {
      Serial.println("Theft Detected");
      digitalWrite(BUZZER_PIN, HIGH);  // Activate buzzer
      delay(1000);   
      digitalWrite(BUZZER_PIN, LOW);  
      delay(2000);                    // Buzzer on for 1 second
         // Deactivate buzzer                 // Buzzer off for 1 second
    }
  }
}