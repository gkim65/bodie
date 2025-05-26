#include <Arduino.h>

// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
  // Initialize LED pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);
}
 
void loop()
{
  // Set the LED HIGH
  digitalWrite(LED_BUILTIN, HIGH);
 
  // Wait for a second
  delay(1000);
 
  // Set the LED LOW
  digitalWrite(LED_BUILTIN, LOW);
 
   // Wait for a second
  delay(1000);

}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}


