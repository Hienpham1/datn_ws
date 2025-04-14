#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

class MotorController {
  private:
    uint8_t in_r, in_l, en_r, en_l;

  public:
    MotorController(uint8_t _in_r, uint8_t _in_l, uint8_t _en_r, uint8_t _en_l) 
      : in_r(_in_r), in_l(_in_l), en_r(_en_r), en_l(_en_l) {}

    void setup() {
      pinMode(in_r, OUTPUT);
      pinMode(in_l, OUTPUT);
      pinMode(en_r, OUTPUT);
      pinMode(en_l, OUTPUT);
      stop();
    }

    void drive(uint8_t speed, bool direction) {
      digitalWrite(en_r, HIGH);
      digitalWrite(en_l, HIGH);
      if (direction) {
        analogWrite(in_r, speed);
        analogWrite(in_l, 0);
      } else {
        analogWrite(in_r, 0);
        analogWrite(in_l, speed);
      }
    }

    void stop() {
      digitalWrite(en_r, LOW);
      digitalWrite(en_l, LOW);
      analogWrite(in_r, 0);
      analogWrite(in_l, 0);
    }
};

#endif
