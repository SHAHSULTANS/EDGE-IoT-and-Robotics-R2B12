// Define the pin where the relay is connected
#define RELAY_PIN 7

// Setup function to initialize the relay pin
void setup() {
  // Set the relay pin as an OUTPUT
  pinMode(RELAY_PIN, OUTPUT);
}

// Loop function to control the relay
void loop() {
  // Turn ON the relay (and the light)
  digitalWrite(RELAY_PIN, HIGH);  // Sending HIGH to the relay activates it
  delay(5000);  // Light stays ON for 5 seconds

  // Turn OFF the relay (and the light)
  digitalWrite(RELAY_PIN, LOW);  // Sending LOW to the relay deactivates it
  delay(5000);  // Light stays OFF for 5 seconds
}
