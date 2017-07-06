#ifndef _GYRO_H
#define _GYRO_H

#include <Wire.h>

// number of calibration iterations to perform
const int GYRO_CALIBRATIONS = 2000;

// I2C slave address of the gyroscope
const int GYRO_ADDR = 0b1101000;

// Gyro values
struct Gyroscope {
  struct acc { long x, y, z; } acc;
  int address;
  int temp;
  double roll, pitch, yaw;
  double roll_cal, pitch_cal, yaw_cal;
  bool calibrated;
};

inline void enableGyro(struct Gyroscope* gyro) {
  Wire.beginTransmission(gyro->address);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  
  // set 500 dps scale
  Wire.beginTransmission(gyro->address);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  // set acceleration sensitivity (+/- 8g)
  Wire.beginTransmission(gyro->address);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  // set low pass filter
  Wire.beginTransmission(gyro->address);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();
}

inline void readGyroValues(struct Gyroscope* gyro) {
  // read and print gyro values
  Wire.beginTransmission(gyro->address);
  Wire.write(0x3B); // communicate with register 0x28 and increment so we read all bytes
  Wire.endTransmission();

  Wire.requestFrom(gyro->address, 14); // request 14 bytes from gyro
  while (Wire.available() < 14);  // wait while data is loading

  // read acceleration data
  gyro->acc.x = -((Wire.read() << 8) | Wire.read()); // negate due to gyro mounted incorrectly
  gyro->acc.y = -((Wire.read() << 8) | Wire.read()); // negate due to gyro mounted incorrectly
  gyro->acc.z = (Wire.read() << 8) | Wire.read();

  gyro->temp = (Wire.read() << 8) | Wire.read();

  // read gyro data
  // read pitch first due to gyro mounted incorrectly
  gyro->pitch = ((Wire.read() << 8) | Wire.read());
  gyro->roll = ((Wire.read() << 8) | Wire.read());
  gyro->yaw = (Wire.read() << 8) | Wire.read();

  if (gyro->calibrated) {
    gyro->roll  -= gyro->roll_cal;
    gyro->pitch -= gyro->pitch_cal;
    gyro->yaw   -= gyro->yaw_cal;
  }
}

inline void calibrateGyro(struct Gyroscope* gyro) {
  gyro->calibrated = false;
  Serial.println("Calibrating...");
  for (int i = 0; i < GYRO_CALIBRATIONS; i++) {
    if (i % 100 == 0)
      Serial.print(".");
    readGyroValues(gyro);
    gyro->roll_cal  += gyro->roll;
    gyro->pitch_cal += gyro->pitch;
    gyro->yaw_cal   += gyro->yaw;
    delay(4);
  }
  Serial.println("Calibrated:");
  Serial.print(gyro->roll_cal); Serial.print("   ");
  Serial.print(gyro->pitch_cal); Serial.print("   ");
  Serial.print(gyro->yaw_cal); Serial.print("   ");

  gyro->roll_cal  /= GYRO_CALIBRATIONS;
  gyro->pitch_cal /= GYRO_CALIBRATIONS;
  gyro->yaw_cal   /= GYRO_CALIBRATIONS;
  gyro->calibrated = true;
}

#endif
