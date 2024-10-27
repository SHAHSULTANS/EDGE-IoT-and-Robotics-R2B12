int ldrPin = A0;        // Analog pin connected to the LDR
int ldrValue = 0;       // Variable to store the LDR value

void setup() {
  Serial.begin(9600); 
  pinMode(A0,INPUT);
  pinMode(4,OUTPUT);  // Start the serial communication
}

void loop() {
  ldrValue = analogRead(ldrPin);  // Read the value from the LDR
  Serial.print("LDR Value: ");    // Print label for clarity
  Serial.println(ldrValue);       // Print the value to the Serial Monitor
  if(ldrValue<=5){
    digitalWrite(4,HIGH);
  }
  else{
    digitalWrite(4,LOW);
  }
  delay(500);  // Wait half a second before the next reading
}