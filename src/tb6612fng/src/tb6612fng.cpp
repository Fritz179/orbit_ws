#include "tb6612fng/tb6612fng.h"

TB6612FNGMotor::TB6612FNGMotor(uint8_t pwm_pin,
                               uint8_t in1_pin,
                               uint8_t in2_pin)
  : pwm_pin_(pwm_pin),
    in1_pin_(in1_pin),
    in2_pin_(in2_pin)
{
  if (gpioInitialise() < 0) {
    throw std::runtime_error("pigpio init failed");
  }

  // Configure pins as outputs
  gpioSetMode(pwm_pin_, PI_OUTPUT);
  gpioSetMode(in1_pin_, PI_OUTPUT);
  gpioSetMode(in2_pin_, PI_OUTPUT);

  // Start stopped/braked
  brake();
}

void TB6612FNGMotor::setSpeed(int16_t speed)
{
  // TODO: clamp to –255..+255
  // speed = std::clamp(speed, int16_t(-255), int16_t(255));
  if (speed < -255) {
    speed = -255;
  } else if (speed > 255) {
    speed = 255;
  }

  uint8_t duty = static_cast<uint8_t>(std::abs(speed));

  if (speed > 0) {
    // forward
    gpioWrite(in1_pin_, 1);
    gpioWrite(in2_pin_, 0);
  }
  else if (speed < 0) {
    // reverse
    gpioWrite(in1_pin_, 0);
    gpioWrite(in2_pin_, 1);
  }
  else {
    // zero → brake
    gpioWrite(in1_pin_, 1);
    gpioWrite(in2_pin_, 1);
  }

  gpioPWM(pwm_pin_, duty);
}

void TB6612FNGMotor::brake()
{
  gpioWrite(in1_pin_, 1);
  gpioWrite(in2_pin_, 1);
  gpioPWM(pwm_pin_, 0);
}

// ----------------------------------------------------------------------------

TB6612FNG::TB6612FNG(uint8_t stby_pin,
                     uint8_t pwma, uint8_t ain1, uint8_t ain2,
                     uint8_t pwmb, uint8_t bin1, uint8_t bin2)
  : stby_pin_(stby_pin),
    motorA_(pwma, ain1, ain2),
    motorB_(pwmb, bin1, bin2)
{
  // Configure standby pin
  gpioSetMode(stby_pin_, PI_OUTPUT);
  // Start in standby (motors braked in their ctors)
  gpioWrite(stby_pin_, 0);
}

TB6612FNG::~TB6612FNG()
{
  // ensure safe shutdown
  standby();
  gpioTerminate();
}

void TB6612FNG::activate()
{
  gpioWrite(stby_pin_, 1);
}

void TB6612FNG::standby()
{
  // brake both channels...
  motorA_.brake();
  motorB_.brake();
  // ...then disable outputs
  gpioWrite(stby_pin_, 0);
}