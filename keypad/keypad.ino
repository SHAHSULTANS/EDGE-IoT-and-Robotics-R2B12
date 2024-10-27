#include <Keypad.h>

// Define the size of the keypad
const byte ROWS = 4; // Four rows
const byte COLS = 4; // Four columns

// Define the keymap for the 4x4 keypad
char keys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

// Define the connections to the keypad rows and columns
byte rowPins[ROWS] = {9, 8, 7, 6}; // Connect to the row pinouts of the keypad
byte colPins[COLS] = {5, 4, 3, 2}; // Connect to the column pinouts of the keypad

// Create the Keypad object
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

void setup() {
  Serial.begin(9600); // Initialize the serial communication
}

void loop() {
  char key = keypad.getKey(); // Get the key pressed

  if (key == 'D') { // If a key is pressed
    Serial.println();
    // Serial.println(key); // Print the key to the serial monitor
  }
  else{
        Serial.print(key); // Print the key to the serial monitor

  }
}