/* 
  Using the DHT-22 sensor with Arduino Uno
  Temperature and humidity sensor
*/

// Libraries
#include <DHT.h>

// Constants
#define DHTPIN 7        // Pin where the DHT sensor is connected
#define DHTTYPE DHT22   // DHT 22 (AM2302)

// Initialize DHT sensor
DHT dht(DHTPIN, DHTTYPE); 

// Variables
float hum;  // Stores humidity value
float temp; // Stores temperature value

void setup() {
  Serial.begin(9600);  // Start the serial communication
  dht.begin();         // Initialize the DHT sensor
}

void loop() {
  delay(2000);  // Wait a few seconds between measurements
  
  // Read humidity and temperature values
  hum = dht.readHumidity();
  temp = dht.readTemperature();
  
  // Check if any readings failed and exit early (to try again)
  if (isnan(hum) || isnan(temp)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  
  // Print temperature and humidity values to the serial monitor
  Serial.print("Humidity: ");
  Serial.print(hum);
  Serial.print(" %, Temp: ");
  Serial.print(temp);
  Serial.println(" °C");
  
  delay(10000);  // Delay 10 seconds before the next reading
}