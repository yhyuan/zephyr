#include <Arduino.h>

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  Serial.begin(115200);
}

unsigned int count = 0;

// the loop function runs over and over again forever
void loop() {
  Serial.println(count++);
  delay(1000);
}
