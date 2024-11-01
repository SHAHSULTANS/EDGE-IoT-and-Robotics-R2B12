#define SOIL_WET 500    // Define the max value we consider soil 'wet'
#define SOIL_DRY 750    // Define the min value we consider soil 'dry'

// Sensor and LED pins
#define SENSOR_POWER 7
#define SENSOR_PIN A0
#define WHITE_LED 2     // Pin for the white LED
#define RED_LED 3       // Pin for the red LED

void setup() {
    // Initialize sensor and LED pins as outputs
    pinMode(SENSOR_POWER, OUTPUT);
    pinMode(WHITE_LED, OUTPUT);
    pinMode(RED_LED, OUTPUT);

    // Turn the sensor OFF initially
    digitalWrite(SENSOR_POWER, LOW);

    // Start the serial communication
    Serial.begin(9600);
}

void loop() {
    // Get the soil moisture reading
    int moisture = readSensor();
    Serial.print("Analog Output: ");
    Serial.println(moisture);

    // Determine the soil moisture status
    if (moisture >= 700) {
        Serial.println("Status: Soil is too dry");
        digitalWrite(WHITE_LED, HIGH);
        digitalWrite(RED_LED, HIGH);
    } else if (moisture >= 500 && moisture < 700) {
        Serial.println("Status: Soil moisture is perfect");
        digitalWrite(WHITE_LED, HIGH);
        digitalWrite(RED_LED, LOW);
    } else if (moisture >= 300 && moisture < 500) {
        Serial.println("Status: Soil moisture is low");
        digitalWrite(WHITE_LED, LOW);
        digitalWrite(RED_LED, HIGH);
    } else {
        Serial.println("Status: Soil is too weight");
        digitalWrite(WHITE_LED, LOW);
        digitalWrite(RED_LED, HIGH);
    }

    delay(1000);  // Take a reading every second for testing
    Serial.println();
}

// Function to read the soil moisture sensor value
int readSensor() {
    digitalWrite(SENSOR_POWER, HIGH);  // Turn the sensor ON
    delay(1000);                         // Allow power to settle
    int val = analogRead(SENSOR_PIN);  // Read the analog value from the sensor
    digitalWrite(SENSOR_POWER, LOW);   // Turn the sensor OFF
    return val;                        // Return the analog moisture value
}