#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Define the LCD I2C address (usually 0x27 or 0x3F)
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Change 0x27 if necessary

const char* scrollingText = "Abdullah, Santo, Debashis, Abid "; // Add spaces to clear the display
const int textLength = 16; // Length of the LCD display

void setup() {
  // Initialize the LCD
  lcd.begin(16, 2);
  lcd.backlight();  // Turn on the backlight (optional)

  // Print Group-06 on the first line
  lcd.setCursor(0, 0);  // Set cursor to column 0, row 0
  lcd.print("Group-06");
}

void loop() {
  // Loop through the scrolling text to create a scrolling effect
  for (int position = 0; position < strlen(scrollingText) + textLength; position++) {
    lcd.setCursor(0, 1); // Set cursor to the second row (row 1)
    
    // Display the part of the scrolling text that fits the display
    for (int i = 0; i < textLength; i++) {
      // Check if the character is within the bounds of the scrolling text
      if (i + position < strlen(scrollingText)) {
        lcd.print(scrollingText[i + position]);  // Print the current character
      } else {
        lcd.print(' ');  // Print space if outside the bounds of the scrolling text
      }
    }

    delay(300);  // Adjust delay for speed of scrolling
  }
}