// Define the pin numbers
const int pirSensorPin = 2;   // Pin connected to the PIR sensor output
const int ledPin = 13;        // Pin connected to the LED

void setup() {
  pinMode(pirSensorPin, INPUT); // Set PIR sensor pin as input
  pinMode(ledPin, OUTPUT);      // Set LED pin as output

  Serial.begin(9600);           // Start serial communication for debugging
}

void loop() {
  // Read the state of the PIR sensor
  int pirState = digitalRead(pirSensorPin);

  // If motion is detected, turn on the LED
  if (pirState == HIGH) {
    digitalWrite(ledPin, HIGH); // Turn on LED
    Serial.println("Motion detected!");
  }
  // If no motion is detected, turn off the LED
  else {
    digitalWrite(ledPin, LOW);  // Turn off LED
    Serial.println("No motion");
  }

  delay(1000); // Delay to prevent rapid switching
}