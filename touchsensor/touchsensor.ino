// Define the pin numbers
const int touchSensorPin = 2; // Pin connected to the touch sensor output
const int ledPin = 13;        // Pin connected to the LED

void setup() {
  Serial.begin(9600);
  pinMode(touchSensorPin, INPUT); // Set touch sensor pin as input
  pinMode(ledPin, OUTPUT);        // Set LED pin as output
}

void loop() {
  // Read the state of the touch sensor
  int sensorValue = digitalRead(touchSensorPin);
  Serial.println(sensorValue);
  // If the sensor is touched, turn on the LED
  if (sensorValue == HIGH) {
    digitalWrite(ledPin, HIGH); // Turn on LED
  }
  // If the sensor is not touched, turn off the LED
  else {
    digitalWrite(ledPin, LOW);  // Turn off LED
  }
}