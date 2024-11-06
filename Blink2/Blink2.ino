const int led1 = 13;  // LED 1 connected to digital pin 9
const int led2 = 10; // LED 2 connected to digital pin 10

// Blink intervals in microseconds (e.g., 500ms = 500000µs)
const unsigned long led1Interval = 500000;  // 500000µs = 500ms = 2Hz
const unsigned long led2Interval = 350000;  // 250000µs = 250ms = 4Hz

// Timers for each LED in microseconds
unsigned long previousMicros1 = 0;
unsigned long previousMicros2 = 0;

void setup() {
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
}

void loop() {
  unsigned long currentMicros = micros();

  // Check if it's time to toggle LED1
  if ((currentMicros - previousMicros1) >= led1Interval) {
    digitalWrite(led1, !digitalRead(led1)); // Toggle LED1
    previousMicros1 += led1Interval;        // Update time for LED1
  }

  // Check if it's time to toggle LED2
  if ((currentMicros - previousMicros2) >= led2Interval) {
    digitalWrite(led2, !digitalRead(led2)); // Toggle LED2
    previousMicros2 += led2Interval;        // Update time for LED2
  }
}
