// #include <Wire.h>

// void setup() {
//   Wire.begin();
//   Serial.begin(9600);
//   while (!Serial);
//   Serial.println("\nI2C Scanner");
//   for (byte address = 1; address < 127; ++address) {
//     Wire.beginTransmission(address);
//     if (Wire.endTransmission() == 0) {
//       Serial.print("Found device at 0x");
//       Serial.println(address, HEX);
//     }
//   }
// }

// void loop() {}