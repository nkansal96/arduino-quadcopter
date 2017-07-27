#ifndef _RECEIVER_H
#define _RECEIVER_H

// Receiver channel
struct ReceiverChannel {
  volatile unsigned long time;
  volatile int pulse;
  volatile bool state;
};

// Receiver values
struct Receiver {
  volatile struct ReceiverChannel ch1;
  volatile struct ReceiverChannel ch2;
  volatile struct ReceiverChannel ch3;
  volatile struct ReceiverChannel ch4;
};

extern Receiver rcvr;
extern unsigned long last_interrupt;

ISR(PCINT0_vect) {
  last_interrupt = micros();
  // Channel 1 ==================================
  if (!rcvr.ch1.state && (PINB & 0b0001)) {
    rcvr.ch1.state = true;
    rcvr.ch1.time = last_interrupt;
  } else if (rcvr.ch1.state && !(PINB & 0b0001)) {
    rcvr.ch1.state = false;
    rcvr.ch1.pulse = last_interrupt - rcvr.ch1.time;
  }
  // Channel 2 ==================================
  if (!rcvr.ch2.state && (PINB & 0b0010)) {
    rcvr.ch2.state = true;
    rcvr.ch2.time = last_interrupt;
  } else if (rcvr.ch2.state && !(PINB & 0b0010)) {
    rcvr.ch2.state = false;
    rcvr.ch2.pulse = last_interrupt - rcvr.ch2.time;
  }
  // Channel 3 ==================================
  if (!rcvr.ch3.state && (PINB & 0b0100)) {
    rcvr.ch3.state = true;
    rcvr.ch3.time = last_interrupt;
  } else if (rcvr.ch3.state && !(PINB & 0b0100)) {
    rcvr.ch3.state = false;
    rcvr.ch3.pulse = last_interrupt - rcvr.ch3.time;
  }
  // Channel 4 ==================================
  if (!rcvr.ch4.state && (PINB & 0b1000)) {
    rcvr.ch4.state = true;
    rcvr.ch4.time = last_interrupt;
  } else if (rcvr.ch4.state && !(PINB & 0b1000)) {
    rcvr.ch4.state = false;
    rcvr.ch4.pulse = last_interrupt - rcvr.ch4.time;
  }
}


#endif
