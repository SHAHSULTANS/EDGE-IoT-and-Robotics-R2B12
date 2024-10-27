const int sensorPin = A3;  // Analog pin where the sensor is connected

void setup(){
  Serial.begin(9600);
  Serial.println("Distance Measurement (MD1030): ");
}

void loop(){
  // Read analog value from sensor
  int sensorValue = analogRead(sensorPin);
  
  // Convert the analog value to a voltage (assuming 5V reference)
  float voltage = sensorValue * (5.0 / 1024.0);
  
  // Convert the voltage to distance (MD1030 sensor-specific formula)
  // Sensor outputs 9.8mV per cm -> distance (cm) = voltage / 0.0098
  float distance = voltage / 512;  

  // Print the distance in CM
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  
  // Print the voltage for debugging
  Serial.print("Voltage: ");
  Serial.print(voltage);
  Serial.println(" V");
  Serial.println();

  // Wait 500ms before next reading
  delay(500);
}