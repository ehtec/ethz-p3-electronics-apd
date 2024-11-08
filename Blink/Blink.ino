const int LED_EXTERNAL = 13;

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_EXTERNAL, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  
  digitalWrite(LED_EXTERNAL, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_EXTERNAL, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
}
