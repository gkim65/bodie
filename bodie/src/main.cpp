/*
The sensor outputs provided by the library are the raw
16-bit values obtained by concatenating the 8-bit high and
low accelerometer and gyro data registers. They can be
converted to units of g and dps (degrees per second) using
the conversion factors specified in the datasheet for your
particular device and full scale setting (gain).

Example: An LSM6DS33 gives an accelerometer Z axis reading
of 16276 with its default full scale setting of +/- 2 g. The
LA_So specification in the LSM6DS33 datasheet (page 15)
states a conversion factor of 0.061 mg/LSB (least
significant bit) at this FS setting, so the raw reading of
16276 corresponds to 16276 * 0.061 = 992.8 mg = 0.9928 g.
*/

#include <Wire.h>
#include <LSM6.h>
#include "motor_driver.h"
#include <Arduino.h>
LSM6 imu;

char report[100];

void setup()
{
  Serial.begin(9600);
  Wire.begin();

  
  while (!imu.init())
  {
    Serial.println("Failed to detect and initialize IMU!");
    delay(1000);
  }
  imu.enableDefault();

  // Setup the motor driver
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);

  // Motor driver speed control
  pinMode(9,   OUTPUT); 
  pinMode(10, OUTPUT);
}

void loop()
{
  imu.read();
  // Serial.println(imu.g.z);

  // snprintf(report, sizeof(report), "A: %8.2f %8.2f %8.2f    G: %8.2f %8.2f %8.2f",
  //   imu.a.x* 0.061, imu.a.y* 0.061, imu.a.z* 0.061,
  //   imu.g.x, imu.g.y, imu.g.z);
  // Serial.print("A: ");
  // Serial.print((double) imu.a.x* 0.061);
  // Serial.print("     ");
  // Serial.print((double) imu.a.y* 0.061);
  // Serial.print("     ");
  // Serial.print((double) imu.a.z* 0.061);
  // Serial.print("     G: ");
  // Serial.print((double) imu.g.x);
  // Serial.print("     ");
  // Serial.print((double) imu.g.y);
  // Serial.print("     ");
  // Serial.println((double) imu.g.z);
  // delay(100);

  // Calculate the angular rate and angular position of the body



  // Calculate control input
  double inputVoltage1 = ;
  double inputVoltage2 = imu.g.z;

  int clippedMotorInput1 = saturateTo255(inputVoltage1);
  int clippedMotorInput2 = saturateTo255(inputVoltage2);

  analogWrite(9, abs(clippedMotorInput1));
  analogWrite(9, abs(clippedMotorInput1));
  // analogWrite(10, 200); //ENB pin
  //(Optional)
  
  if (clipped_gz > 0) {
    digitalWrite(motor1pin1, HIGH);
    digitalWrite(motor1pin2, LOW);
  } else if (clipped_gz < 0) {
    digitalWrite(motor1pin1, LOW);
    digitalWrite(motor1pin2, HIGH);
  } else {
    digitalWrite(motor1pin1, LOW);
    digitalWrite(motor1pin2, LOW);
  }
  // digitalWrite(motor1pin1,   HIGH);
  // digitalWrite(motor1pin2, LOW);

  // digitalWrite(motor2pin1, HIGH);
  //  digitalWrite(motor2pin2, LOW);
  // delay(3000);

  // digitalWrite(motor1pin1,   LOW);
  // digitalWrite(motor1pin2, HIGH);

  // digitalWrite(motor2pin1, LOW);
  // digitalWrite(motor2pin2, HIGH);
  // delay(3000);

  // digitalWrite(motor1pin1,   LOW);
  // digitalWrite(motor1pin2, HIGH);
  // delay(3000);
}


// Ok here's what I need to do:
// Overall: Two control inputs, as a function of angle and angular rate of the robot
// Big pieces of code:
// Filtering the IMU inputs (and getting real values)
// Converting the input values into motor inputs.

// input_voltage1, input_voltage2 = function(state)
// staturated at +/- 255
// based on input voltage sign, do either one of the pins
int saturateTo255(double value) {
  if (value > 255.0) return 255;
  if (value < -255.0) return -255;
  return static_cast<int>(value);
}

void writeToMotor(int clippedMotorInput) {
  if (clipped_gz > 0) {
    digitalWrite(motor1pin1, HIGH);
    digitalWrite(motor1pin2, LOW);
  } else if (clipped_gz < 0) {
    digitalWrite(motor1pin1, LOW);
    digitalWrite(motor1pin2, HIGH);
  } else {
    digitalWrite(motor1pin1, LOW);
    digitalWrite(motor1pin2, LOW);
  } 
}