#ifndef ENCODER_READER_H
#define ENCODER_READER_H

#include <Arduino.h>

class EncoderReader {
public:
  EncoderReader(uint8_t pinA, uint8_t pinB)
    : pinA(pinA), pinB(pinB), position(0), lastReadTime(0) {}

  void begin(void (*isrCallback)()) {
    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(pinA), isrCallback, RISING);
  }

  void update() {
    int b = digitalRead(pinB);
    if (b > 0) {
        position++;
    } else {
        position--;
    }
  }

  int32_t read() const {
    return position;
  }

  void reset() {
    position = 0;
  }

  int32_t readAndReset() {
    noInterrupts();
    int32_t val = position;
    position = 0;
    interrupts();
    return val;
  }

private:
  uint8_t pinA;
  uint8_t pinB;
  volatile int32_t position;
  unsigned long lastReadTime;
};

#endif // ENCODER_READER_H
