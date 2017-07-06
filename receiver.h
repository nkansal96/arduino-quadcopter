#ifndef _RECEIVER_H
#define _RECEIVER_H

// Receiver channel
struct ReceiverChannel {
  unsigned long time;
  int pulse;
  bool state;
};

// Receiver values
struct Receiver {
  struct ReceiverChannel ch1;
  struct ReceiverChannel ch2;
  struct ReceiverChannel ch3;
  struct ReceiverChannel ch4;
};

extern Receiver rcvr;

ISR(PCINT0_vect) {
  register unsigned long curr_time = micros();
  // Channel 1 ==================================
  if (!rcvr.ch1.state && (PINB & 0b0001)) {
    rcvr.ch1.state = true;
    rcvr.ch1.time = curr_time;
  } else if (rcvr.ch1.state && !(PINB & 1)) {
    rcvr.ch1.state = false;
    rcvr.ch1.pulse = curr_time - rcvr.ch1.time;
  }
  // Channel 2 ==================================
  if (!rcvr.ch2.state && (PINB & 0b0010)) {
    rcvr.ch2.state = true;
    rcvr.ch2.time = curr_time;
  } else if (rcvr.ch2.state && !(PINB & 0b0010)) {
    rcvr.ch2.state = false;
    rcvr.ch2.pulse = curr_time - rcvr.ch2.time;
  }
  // Channel 3 ==================================
  if (!rcvr.ch3.state && (PINB & 0b0100)) {
    rcvr.ch3.state = true;
    rcvr.ch3.time = curr_time;
  } else if (rcvr.ch3.state && !(PINB & 0b0100)) {
    rcvr.ch3.state = false;
    rcvr.ch3.pulse = curr_time - rcvr.ch3.time;
  }
  // Channel 4 ==================================
  if (!rcvr.ch4.state && (PINB & 0b1000)) {
    rcvr.ch4.state = true;
    rcvr.ch4.time = curr_time;
  } else if (rcvr.ch4.state && !(PINB & 0b1000)) {
    rcvr.ch4.state = false;
    rcvr.ch4.pulse = curr_time - rcvr.ch4.time;
  }
}


#endif
