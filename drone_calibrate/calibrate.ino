#include <Wire.h>
#include <EEPROM.h>

#include "util.h"
#include "gyro.h"

struct Gyroscope gyro;

void setup() {
  while (!Serial);
  Serial.begin(115200);
  while (!Serial.availableForWrite());
  
  // zero data structures
  memset(&gyro, 0, sizeof(gyro));

  // set pin modes
  pinMode(LED_PIN, OUTPUT);

  // status indication: setup
  digitalWrite(LED_PIN, HIGH);

  // start the gyroscope
  gyro.address = GYRO_ADDR;
  enableGyro(&gyro);

  Serial.println("Calibration will start in 3 seconds. Do not move the quadcopter");
  delay(3000);

  calibrateGyro(&gyro);
  writeGryoToEEPROM(gyro, GYRO_STRUCT_LOC);
  Serial.println("Calibrated: ");
  Serial.println(gyro.roll_cal);
  Serial.println(gyro.pitch_cal);
  Serial.println(gyro.yaw_cal);
  Serial.println();

  struct Gyroscope g2;
  memset(&g2, 0, sizeof(g2));

  readGyroFromEEPROM(g2, GYRO_STRUCT_LOC);
  Serial.println("Verify EEPROM Values: ");
  Serial.println(g2.roll_cal);
  Serial.println(g2.pitch_cal);
  Serial.println(g2.yaw_cal);
}

void loop() { 
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(1);
}
