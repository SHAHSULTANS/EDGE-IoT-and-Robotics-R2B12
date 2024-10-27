#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
#define OLED_RESET    -1  // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define MQ135_PIN A0  // Analog pin connected to MQ-135 sensor

void setup() {
  // Start serial communication for debugging purposes
  Serial.begin(9600);

  // Initialize the OLED display
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {  // 0x3C is the I2C address of the OLED
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);  // Don't proceed, loop forever
  }

  // Clear the buffer
  display.clearDisplay();

  // Print the initialization message
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.println(F("MQ-135 Sensor"));
  display.display();           // Show the message on the display
  delay(500);                 // Wait for 2 seconds
}

void loop() {
  // Read analog data from MQ-135
  int mq135_value = analogRead(MQ135_PIN);

  // Convert the raw analog value to a readable form (depends on calibration)
  float voltage = mq135_value * (5.0 / 1023.0); // Convert the ADC value to voltage (0-5V)
  
  // Display data on OLED
  display.clearDisplay();   // Clear the display
  
  // Show sensor value in raw format
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);  // Start at top-left corner
  display.println("MQ-135 Sensor");
  display.setCursor(0, 10);
  display.print("Raw Value: ");
  display.println(mq135_value);
  
  // Show converted voltage
  display.setCursor(0, 20);
  display.print("Voltage: ");
  display.print(voltage, 2);  // Display voltage with 2 decimal places
  display.println("V");

  // Show a message based on the sensor value (optional)
  display.setCursor(0, 40);
  if (mq135_value > 600) {
    display.println("High Pollution");
  } else if (mq135_value > 400) {
    display.println("Moderate Pollution");
  } else {
    display.println("Clean Air");
  }

  display.display();   // Show the updated display content

  // Debug output to the serial monitor
  Serial.print("MQ-135 Raw Value: ");
  Serial.println(mq135_value);
  Serial.print("Voltage: ");
  Serial.print(voltage);
  Serial.println(" V");
  
  delay(500);  // Update every second
}