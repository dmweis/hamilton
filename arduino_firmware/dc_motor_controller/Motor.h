#include "Arduino.h"

class Motor
{
  public:
    Motor(int forward_pin, int back_pin, int pwm_pin) : _forward_pin(forward_pin), _back_pin(back_pin), _pwm_pin(pwm_pin)
    {
      pinMode(_forward_pin, OUTPUT);
      pinMode(_back_pin, OUTPUT);
      pinMode(_pwm_pin, OUTPUT);
      digitalWrite(_forward_pin, LOW);
      digitalWrite(_back_pin, LOW);
      digitalWrite(_pwm_pin, LOW);
    }

    void set_speed(int speed)
    {
      if (speed == 0)
      {
        digitalWrite(_forward_pin, LOW);
        digitalWrite(_back_pin, LOW);
        analogWrite(_pwm_pin, 0);
      }
      else if (speed > 0)
      {
        digitalWrite(_forward_pin, HIGH);
        analogWrite(_pwm_pin, speed);
        digitalWrite(_back_pin, LOW);
      }
      else if (speed < 0)
      {
        digitalWrite(_forward_pin, LOW);
        analogWrite(_pwm_pin, -speed);
        digitalWrite(_back_pin, HIGH);
      }

    }

  private:
    int _forward_pin;
    int _back_pin;
    int _pwm_pin;
};
