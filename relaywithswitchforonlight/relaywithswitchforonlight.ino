// Define the pin where the relay and switch are connected
#define RELAY_PIN 7   // Relay control pin
#define SWITCH_PIN 2  // Switch input pin

void setup() {
  // Set the relay pin as an OUTPUT
  pinMode(RELAY_PIN, OUTPUT);

  // Set the switch pin as an INPUT with internal pull-up resistor
  pinMode(SWITCH_PIN, INPUT_PULLUP);
}

void loop() {
  // Check if the switch is pressed (LOW because we use INPUT_PULLUP)
  if (digitalRead(SWITCH_PIN) == LOW) {
    // Turn ON the relay (and the light)
    digitalWrite(RELAY_PIN, HIGH);  // Sending HIGH to the relay activates it
  } else {
    // Turn OFF the relay (and the light)
    digitalWrite(RELAY_PIN, LOW);   // Sending LOW to the relay deactivates it
  }
}