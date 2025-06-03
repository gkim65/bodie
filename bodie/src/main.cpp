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
#include "constants.h"
#include "math.h"
#include <Arduino.h>
LSM6 imu;

char report[100];

// Initialize omega history
double omegaHistory[4] = {0.0, 0.0, 0.0, 0.0};
static double theta = 0.0;
unsigned long prev_time = 0;

// TUNABLE PARAMETERS
double blend_factor = 0.5;
double Kp = 15.0;
double Kd = 0.5;

int saturateTo255(double value) {
  if (value > 255.0) return 255;
  if (value < -255.0) return -255;
  return static_cast<int>(value);
}


typedef struct {
  double pitch;
  double pitchRate;
} state;

state estimateState(LSM6 imu) {

  // TUNABLE PARAMETERS: BLEND_FACTOR, OMEGA FILTERING
  
  state RobotState;
  for (int i = 3; i > 0; --i) {
    omegaHistory[i] = omegaHistory[i - 1];
  }

  // Get pitch rate
  double omega = (double)(imu.g.y*GYRO_FACTOR);
  Serial.print("Omega raw: ");
  Serial.println(omega);
  omegaHistory[0] = omega;

  // Basic-ass low pass filter on the omega
  double sum = 0.0;
  for (int i = 0; i < 4; ++i) {
    sum += omegaHistory[i];
  }
  double omegaFiltered = sum / 4.0;

  // Get dt
  unsigned long current_time = micros();
  double dt = (current_time - prev_time) / 1e6;  // dt in seconds

  Serial.print("Current Time: ");
  Serial.println(current_time);
  prev_time = current_time;
  // Integrate omega for theta
  double thetaFromGyro = theta + omegaFiltered*dt;
  Serial.print("Theta from gyro: ");
  Serial.println(thetaFromGyro);
  
  // Get theta from accelerometer
  double accelX = (double) imu.a.x*ACCEL_FACTOR;
  double accelZ = (double) imu.a.z*ACCEL_FACTOR;
  double thetaFromAccel = atan2(accelX,accelZ)*57.29;
  Serial.print("Theta from accel: ");
  Serial.println(thetaFromAccel);
  // Blend theta values
  theta = blend_factor*thetaFromGyro + (1 - blend_factor)*thetaFromAccel;

  RobotState.pitch = theta;
  RobotState.pitchRate = omega;

  return RobotState;
}

void setup()
{
  // Begin serial for debugging
  Serial.begin(9600);
  Wire.begin();

  // Initialize IMU
  while (!imu.init())
  {
    Serial.println("Failed to detect and initialize IMU!");
    delay(1000);
  }
  imu.enableDefault();

  // Setup the motor driver
  pinMode(MOTOR1PIN1, OUTPUT);
  pinMode(MOTOR1PIN2, OUTPUT);
  pinMode(MOTOR2PIN1, OUTPUT);
  pinMode(MOTOR2PIN2, OUTPUT);

  // Motor driver speed control
  pinMode(MOTOR1SPEEDPIN, OUTPUT); 
  pinMode(MOTOR2SPEEDPIN, OUTPUT);
  
  // Previous time for integration
  prev_time = micros();
}

void loop()
{
  // Get IMU data
  imu.read();

  // Calculate the angular rate and angular position of the body
  state RobotState = estimateState(imu);

  
  Serial.print("pitch angle: ");
  Serial.println(RobotState.pitch);
  Serial.print("pitch rate: ");
  Serial.println(RobotState.pitchRate);

  // Calculate control input
  double inputVoltage1 = Kp*RobotState.pitch + Kd*RobotState.pitchRate;
  double inputVoltage2 = Kp*RobotState.pitch + Kd*RobotState.pitchRate;

  Serial.print("Input");
  Serial.println(inputVoltage1);

  int clippedMotorInput1 = saturateTo255(inputVoltage1);
  int clippedMotorInput2 = saturateTo255(inputVoltage2);

  Serial.print("clipped input");
  Serial.println(clippedMotorInput1);

  // delay(1000);
  // Write speed signal
  analogWrite(MOTOR1SPEEDPIN, abs(clippedMotorInput1));
  analogWrite(MOTOR2SPEEDPIN, abs(clippedMotorInput1));

  // Write motor directions
  if (clippedMotorInput1 > 0) {
    digitalWrite(MOTOR1PIN1, HIGH);
    digitalWrite(MOTOR1PIN2, LOW);
  } else if (clippedMotorInput1 < 0) {
    digitalWrite(MOTOR1PIN1, LOW);
    digitalWrite(MOTOR1PIN2, HIGH);
  } else {
    digitalWrite(MOTOR1PIN1, LOW);
    digitalWrite(MOTOR1PIN2, LOW);
  } 
  if (clippedMotorInput2 > 0) {
    digitalWrite(MOTOR2PIN1, HIGH);
    digitalWrite(MOTOR2PIN2, LOW);
  } else if (clippedMotorInput2 < 0) {
    digitalWrite(MOTOR2PIN1, LOW);
    digitalWrite(MOTOR2PIN2, HIGH);
  } else {
    digitalWrite(MOTOR2PIN1, LOW);
    digitalWrite(MOTOR2PIN2, LOW);
  } 

}

