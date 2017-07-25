#ifndef _PID_H
#define _PID_H

#include "util.h"

// PID attribute struct 
struct PIDAttribute {
  int max;    // maximum output for this attribute
  double gyro;   // gyro value for this attribute
  double target; // target value for this attribute
  double output; // PID output for this attribute
  struct proportion {
    double gain; // p-gain
  } p;
  struct integral {
    double gain; // i-gain
    double total; // total so far
  } i;
  struct derivative {
    double gain; // d-gain
    double prev; // previous error
  } d;
};

// PID controller struct
struct PID {
  struct PIDAttribute roll;
  struct PIDAttribute pitch;
  struct PIDAttribute yaw;
};

inline void setupPID(struct PID* pid) {
  pid->yaw.max = 0;
  pid->yaw.p.gain = 3;
  pid->yaw.i.gain = 0.02;
  pid->yaw.d.gain = 0.0;

  pid->roll.max = 0;
  pid->roll.p.gain = 4.8;
  pid->roll.i.gain = 0.02;
  pid->roll.d.gain = 15.0;

  pid->pitch.max = 400;//pid->roll.max;
  pid->pitch.p.gain = pid->roll.p.gain;
  pid->pitch.i.gain = pid->roll.i.gain;
  pid->pitch.d.gain = pid->roll.d.gain;
}

inline void calculatePID(struct PID* pid) {
  double error, output, pid_i;
  double hysterisis = 0;
  // for each of (roll, pitch, yaw):
  //   error = gyro - receiver
  //   output = error * p_gain + (i_total + error) * i_gain + (error - prev_error) * d_gain
  //   constaint abs(output) to max

  // calculate PID for roll
  error = pid->roll.gyro - pid->roll.target;
  pid_i = pid->roll.i.total + (error * pid->roll.i.gain);
  pid->roll.i.total = limit(-pid->roll.max, pid_i, pid->roll.max);
  // pid->roll.i.total = pid_i;

  output = (error * pid->roll.p.gain) + pid->roll.i.total + pid->roll.d.gain * (error - pid->roll.d.prev);
  output = limit(-pid->roll.max, output, pid->roll.max);

  // Serial.print("ROLL P: ");
  Serial.print(error * pid->roll.p.gain);
  Serial.print(", ");
  Serial.print(pid->roll.i.total);
  Serial.print(", ");
  Serial.print(pid->roll.d.gain * (error - pid->roll.d.prev));

  pid->roll.d.prev = error;
  pid->roll.output = output;

  // calculate PID for pitch
  error = pid->pitch.gyro - pid->pitch.target;
  pid_i = pid->pitch.i.total + (error * pid->pitch.i.gain);
  pid->pitch.i.total = limit(-pid->pitch.max, pid_i, pid->pitch.max);
  // pid->pitch.i.total = pid_i;
  // todo - avoid total's overflow

  output = (error * pid->pitch.p.gain) + pid->pitch.i.total + pid->pitch.d.gain * (error - pid->pitch.d.prev);
  output = limit(-pid->pitch.max, output, pid->pitch.max);
  pid->pitch.d.prev = error;
  pid->pitch.output = output;

  // calculate PID for yaw
  error = pid->yaw.gyro - pid->yaw.target;
  pid_i = pid->yaw.i.total + (error * pid->yaw.i.gain);
  pid->yaw.i.total = limit(-pid->yaw.max, pid_i, pid->yaw.max);
  // pid->yaw.i.total = pid_i;

  output = (error * pid->yaw.p.gain) + pid->yaw.i.total + pid->yaw.d.gain * (error - pid->yaw.d.prev);
  output = limit(-pid->yaw.max, output, pid->yaw.max);
  pid->yaw.d.prev = error;
  pid->yaw.output = output;
}

#endif
