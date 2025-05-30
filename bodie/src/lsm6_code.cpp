// /*
// The sensor outputs provided by the library are the raw
// 16-bit values obtained by concatenating the 8-bit high and
// low accelerometer and gyro data registers. They can be
// converted to units of g and dps (degrees per second) using
// the conversion factors specified in the datasheet for your
// particular device and full scale setting (gain).

// Example: An LSM6DS33 gives an accelerometer Z axis reading
// of 16276 with its default full scale setting of +/- 2 g. The
// LA_So specification in the LSM6DS33 datasheet (page 15)
// states a conversion factor of 0.061 mg/LSB (least
// significant bit) at this FS setting, so the raw reading of
// 16276 corresponds to 16276 * 0.061 = 992.8 mg = 0.9928 g.
// */

// #include <Wire.h>
// #include <LSM6.h>
// LSM6 imu;

// char report[100];

// void setup()
// {
//   Serial.begin(9600);
//   Wire.begin();

  
//   while (!imu.init())
//   {
//     Serial.println("Failed to detect and initialize IMU!");
//     delay(1000);
//   }
//   imu.enableDefault();
// }

// void loop()
// {
//   imu.read();
//   // Serial.println(imu.g.z);

//   // snprintf(report, sizeof(report), "A: %8.2f %8.2f %8.2f    G: %8.2f %8.2f %8.2f",
//   //   imu.a.x* 0.061, imu.a.y* 0.061, imu.a.z* 0.061,
//   //   imu.g.x, imu.g.y, imu.g.z);
//   Serial.print("A: ");
//   Serial.print((double) imu.a.x* 0.061);
//   Serial.print("     ");
//   Serial.print((double) imu.a.y* 0.061);
//   Serial.print("     ");
//   Serial.print((double) imu.a.z* 0.061);
//   Serial.print("     G: ");
//   Serial.print((double) imu.g.x);
//   Serial.print("     ");
//   Serial.print((double) imu.g.y);
//   Serial.print("     ");
//   Serial.println((double) imu.g.z);
//   delay(100);
// }