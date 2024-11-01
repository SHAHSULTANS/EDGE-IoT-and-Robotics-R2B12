/* Arduino example code for MaxBotix MB1240 XL-MaxSonar-EZ4 ultrasonic distance sensor: analog voltage output. 
   More info: www.makerguides.com */

#define sensorPin A0  // Define the sensor pin
#define ledPin 7      // Define the LED pin

float distance = 0;

void setup() {
  pinMode(sensorPin, INPUT);  // Set sensor pin as input
  pinMode(ledPin, OUTPUT);    // Set LED pin as output
  Serial.begin(9600);         // Start the serial communication
}

void read_sensor() {
  distance = analogRead(sensorPin) * 1;  // Read analog value and convert to distance
}

void print_data() {
  Serial.print("Distance = ");
  Serial.print(distance);
  Serial.println(" cm");
}

void control_led() {
  if (distance >= 43.18 && distance <= 48.26 ) {  // If distance is between 23 inches(43.18 cm) and 25 inches(48.26 cm) 
    digitalWrite(ledPin, HIGH);  // Turn the LED on
  } else {
    digitalWrite(ledPin, LOW);   // Turn the LED off
  }
}

void loop() {
  read_sensor();    // Read the distance from the sensor
  print_data();     // Print the distance to the serial monitor
  control_led();    // Control the LED based on the exact distance
  delay(1000);      // Wait for 1 second before next measurement
}
