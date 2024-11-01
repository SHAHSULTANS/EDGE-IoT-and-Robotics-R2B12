const int ledPin = 9;         // PWM pin connected to the LED
const int maxDutyCycle = 191; // Max PWM value for 8-bit resolution (0-255)
int dutyCycle = 0;            // Starting duty cycle
int increment = 5;            // Angle increment for smooth transitions
int delayTime = 500;           // Delay for smooth fading

void setup() {
  pinMode(ledPin, OUTPUT);
}

void loop() {
  // Increase brightness in a smooth wave
  for (int angle = 0; angle <= 180; angle += increment) {
    // Calculate duty cycle using a sine wave pattern for smooth transition
    dutyCycle = (int)(maxDutyCycle * (sin(radians(angle))));
    
    // Apply PWM to the LED
    analogWrite(ledPin, dutyCycle);
    
    // Delay for smooth transition effect
    delay(delayTime);
  }

  // Decrease brightness in a smooth wave
  for (int angle = 180; angle >= 0; angle -= increment) {
    dutyCycle = (int)(maxDutyCycle * (sin(radians(angle))));
    analogWrite(ledPin, dutyCycle);
    delay(delayTime);
  }
}