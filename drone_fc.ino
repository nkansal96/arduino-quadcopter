#include "util.h"
#include "pid.h"
#include "gyro.h"
#include "receiver.h"

// program state
enum State { STOPPED, STARTING, STARTED };
State state = STOPPED;

int battery_voltage = 0;            // current battery voltage
unsigned long loop_timer = 0;       // loop timer
unsigned long last_interrupt = 0;   // time of last interrupt for receiver input

struct Gyroscope gyro;              // gyroscope
struct Receiver rcvr;               // receiver
struct PID pid;                     // PID settings

int esc_fr, esc_rr, esc_rl, esc_fl; // esc outputs (0 => RF, 1 => RR, 2 => LR, 3 => RF)
double pitch_angle, roll_angle;
double pitch_acc = 0, roll_acc = 0;

void setup() {
  // zero data structures
  memset(&gyro, 0, sizeof(gyro));
  memset(&rcvr, 0, sizeof(rcvr));
  memset(&pid, 0, sizeof(pid));

  // set pin modes
  pinMode(LED_PIN, OUTPUT);
  pinMode(ESC_RF_PIN, OUTPUT);
  pinMode(ESC_RR_PIN, OUTPUT);
  pinMode(ESC_LR_PIN, OUTPUT);
  pinMode(ESC_LF_PIN, OUTPUT);

  // status indication: setup
  digitalWrite(LED_PIN, HIGH);

  // set interrupt service vector pins
  PCICR  |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT0);
  PCMSK0 |= (1 << PCINT1);
  PCMSK0 |= (1 << PCINT2);
  PCMSK0 |= (1 << PCINT3);

  // set the I2C clock speed to 400kHz.
  TWBR = 12;

  // start the gyroscope
  gyro.address = GYRO_ADDR;
  enableGyro(&gyro);

  // read calibration values from gyroscope
  while (!readGyroFromEEPROM(gyro, GYRO_STRUCT_LOC)) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delayMicroseconds(250);
  }

  // setup PID settings
  setupPID(&pid);

  int count = 0;
  // wait for start signal
  while (rcvr.ch3.pulse < 990 || rcvr.ch3.pulse > 1020 || rcvr.ch4.pulse < 1400) {
    PORTD |= 0b11110000; // turn on all ESC
    delayMicroseconds(1000); // 1000 microsecond pulse
    PORTD &= 0b00001111; // turn off the ESC

    if (count++ == 125) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      count = 0;
    }

    delay(3); // delay 3 ms to keep ESC refresh rate at 250Hz.
  }

  // Read the battery voltage
  battery_voltage = (analogRead(BATTERY_PIN) + 65) * 1.2317;

  loop_timer = micros();
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  // regardless of state, calculate current orientation
  // calculate current orientation according to the gyro

  // use a 70-30 complementary filter
  // 65.5 is from the MPU-6050 spec
  pid.yaw.gyro = (pid.yaw.gyro * 0.7) + ((gyro.yaw / 65.5) * 0.3);
  pid.roll.gyro = (pid.roll.gyro * 0.7) + ((gyro.roll / 65.5) * 0.3);
  pid.pitch.gyro = (pid.pitch.gyro * 0.7) + ((gyro.pitch / 65.5) * 0.3);

  // calculate the angles
  // 0.0000611 = 1 / 250Hz / 65.5
  pitch_angle += gyro.pitch * 0.0000611;
  roll_angle += gyro.roll * 0.0000611;

  // shift the roll/pitch axes if the drone has yawed
  pitch_angle -= roll_angle * sin(gyro.yaw * 0.000001066);
  roll_angle += pitch_angle * sin(gyro.yaw * 0.000001066);

  // calculate the total acceleration
  double total_acc = sqrt(pow2(gyro.acc.x) + pow2(gyro.acc.y) + pow2(gyro.acc.z));
  if (abs(gyro.acc.y) < total_acc)
    pitch_acc = asin((float)gyro.acc.y/total_acc) * 57.296;
  if (abs(gyro.acc.x) < total_acc)
    roll_acc = -asin((float)gyro.acc.x/total_acc) * 57.296;

  // use acceleration to correct the drift from the gyro
  roll_angle = (roll_angle * 0.9996) + (roll_acc * 0.0004);
  pitch_angle = (pitch_angle * 0.9996) + (pitch_acc * 0.0004);

  double roll_adjust = roll_angle * 15.0;
  double pitch_adjust = pitch_angle * 15.0;

  // based on program state, do different things
  switch (state) {
    case STOPPED: {
      if (rcvr.ch3.pulse < 1050 && rcvr.ch4.pulse < 1050)
        state = STARTING;
      resetESCPulses();
      break;
    }

    case STARTING: {
      if (rcvr.ch3.pulse < 1050 && rcvr.ch4.pulse < 1550 && rcvr.ch4.pulse > 1450) {
        state = STARTED;

        // do pre-flight setup
        pid.yaw.i.total = 0;
        pid.roll.i.total = 0;
        pid.pitch.i.total = 0;

        pid.yaw.d.prev = 0;
        pid.roll.d.prev = 0;
        pid.pitch.d.prev = 0;

        roll_angle = roll_acc;
        pitch_angle = pitch_acc;
      }
      resetESCPulses();
      break;
    }

    case STARTED: {
      if (rcvr.ch3.pulse < 1050 && rcvr.ch4.pulse > 1950)
        state = STOPPED;

      // set PID target for roll (16 micros deadband)
      pid.roll.target = 0;
      if (rcvr.ch1.pulse > 1508) pid.roll.target = rcvr.ch1.pulse - 1508;
      if (rcvr.ch1.pulse < 1492) pid.roll.target = rcvr.ch1.pulse - 1492;
      pid.roll.target -= roll_adjust;
      pid.roll.target /= 3; // convert to degrees

      // set PID target for pitch (16 micros deadband)
      pid.pitch.target = 0;
      if (rcvr.ch2.pulse > 1508) pid.pitch.target = rcvr.ch2.pulse - 1508;
      if (rcvr.ch2.pulse < 1492) pid.pitch.target = rcvr.ch2.pulse - 1492;
      pid.pitch.target -= pitch_adjust;
      pid.pitch.target /= 3; // convert to degrees

      // do not yaw when turning off the motors
      pid.yaw.target = 0;
      if (rcvr.ch3.pulse > 1100) {
        // set PID target for yaw (16 micros deadband)
        if (rcvr.ch4.pulse > 1508) pid.yaw.target = rcvr.ch4.pulse - 1508;
        if (rcvr.ch4.pulse < 1492) pid.yaw.target = rcvr.ch4.pulse - 1492;
        pid.yaw.target /= 3; // convert to degrees
      }

      calculatePID(&pid);

      // calculate ESC pulses necessary to level
      int throttle = min(1800, rcvr.ch3.pulse);
      esc_fr = throttle - pid.pitch.output + pid.roll.output - pid.yaw.output;
      esc_rr = throttle + pid.pitch.output + pid.roll.output + pid.yaw.output;
      esc_rl = throttle + pid.pitch.output - pid.roll.output - pid.yaw.output;
      esc_fl = throttle - pid.pitch.output - pid.roll.output + pid.yaw.output;

      // compensate for battery voltage
      if (battery_voltage < 1240 && battery_voltage > 800) {
        float compensation = 1 + ((1240.0 - battery_voltage)/3500.0);
        esc_fr *= compensation;
        esc_rr *= compensation;
        esc_rl *= compensation;
        esc_fl *= compensation;
      }

      // limit the ESC outputs - keep them on
      esc_fr = min(2000, max(1100, esc_fr));
      esc_rr = min(2000, max(1100, esc_rr));
      esc_rl = min(2000, max(1100, esc_rl));
      esc_fl = min(2000, max(1100, esc_fl));

      // check receiver inputs to make sure they didn't get disconnected
      if (loop_timer > last_interrupt + 16000) {
        state = STOPPED;
        rcvr.ch1.pulse = 1500;
        rcvr.ch2.pulse = 1500;
        rcvr.ch3.pulse = 1000;
        rcvr.ch4.pulse = 1500;
        return;
      }
    }
  }

    // keep loop at 4000 microseconds (250 Hz) so ESC can function properly
  while (micros() - loop_timer < 4000);
  loop_timer = micros();

  PORTD |= 0b11110000; // turn on all the ESC
  unsigned long timer_1 = esc_fr + loop_timer;
  unsigned long timer_2 = esc_rr + loop_timer;
  unsigned long timer_3 = esc_rl + loop_timer;
  unsigned long timer_4 = esc_fl + loop_timer;

  // there is always at least 1ms of free time, do something useful
  readGyroValues(&gyro);

  // update the voltage
  battery_voltage = battery_voltage * 0.92 + ((analogRead(BATTERY_PIN) + 65) * 0.09853);
  if (battery_voltage < LOW_BATTERY_V)
    digitalWrite(LED_PIN, HIGH);

  while (PORTD & 0b11110000) {
    register unsigned long t = micros();
    if (timer_1 <= t) PORTD &= 0b11101111;
    if (timer_2 <= t) PORTD &= 0b11011111;
    if (timer_3 <= t) PORTD &= 0b10111111;
    if (timer_4 <= t) PORTD &= 0b01111111;
  }
}

inline void resetESCPulses() {
  esc_fr = 1000;
  esc_rr = 1000;
  esc_rl = 1000;
  esc_fl = 1000;
}
