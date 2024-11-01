// Define the pins for each relay
int relay1 = 3;  // Pin for Relay 1
int relay2 = 4;  // Pin for Relay 2
int relay3 = 5;  // Pin for Relay 3
int relay4 = 6;  // Pin for Relay 4

void setup() {
  // Set all relay pins as output
  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);
  pinMode(relay3, OUTPUT);
  pinMode(relay4, OUTPUT);

  // Initialize all relays to HIGH (off)
  digitalWrite(relay1, HIGH);
  digitalWrite(relay2, HIGH);
  digitalWrite(relay3, HIGH);
  digitalWrite(relay4, HIGH);
}

void loop() {
  // Relay 4 ON, others OFF
  digitalWrite(relay4, LOW);
  delay(2000);
  digitalWrite(relay4, HIGH);

  // Relay 3 ON, others OFF
  digitalWrite(relay3, LOW);
  delay(2000);
  digitalWrite(relay3, HIGH);

  // Relay 2 ON, others OFF
  digitalWrite(relay2, LOW);
  delay(2000);
  digitalWrite(relay2, HIGH);

  // Relay 1 ON, others OFF
  digitalWrite(relay1, LOW);
  delay(10000);
  digitalWrite(relay1, HIGH);
}