#ifndef ENCODER_READER_H
#define ENCODER_READER_H

#include <Arduino.h>

class EncoderReader {
  private:
    uint8_t pinA, pinB;
    volatile long count;

  public:
    EncoderReader(uint8_t _pinA, uint8_t _pinB)
      : pinA(_pinA), pinB(_pinB), count(0) {}

    void begin(void (*isr)()) {
      pinMode(pinA, INPUT_PULLUP);
      pinMode(pinB, INPUT_PULLUP);
      attachInterrupt(digitalPinToInterrupt(pinA), isr, RISING);
    }

    void update() {
      int b = digitalRead(pinB);
      count += (b == HIGH) ? 1 : -1;  // xác định chiều quay
    }

    long read() {
      noInterrupts();
      long val = count;
      interrupts();
      return val;
    }

    void reset() {
      noInterrupts();
      count = 0;
      interrupts();
    }
};

#endif
