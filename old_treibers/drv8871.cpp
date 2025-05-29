#include "drv8871.h"

DRV8871::DRV8871(uint8_t in1_pin, uint8_t in2_pin)
  : m_in1_pin(in1_pin),
    m_in2_pin(in2_pin)
{

  // Configure pins as outputs
  gpioSetMode(m_in1_pin, PI_OUTPUT);
  gpioSetMode(m_in2_pin, PI_OUTPUT);

  // Start stopped/braked
  brake();
}

void DRV8871::setSpeed(int16_t speed)
{
  // TODO: clamp to –255..+255
  // speed = std::clamp(speed, int16_t(-255), int16_t(255));
  if (speed < -255) {
    speed = -255;
  } else if (speed > 255) {
    speed = 255;
  }

  // 0 = full speed, 255 = brake
  uint8_t duty = 255 - static_cast<uint8_t>(std::abs(speed));

  if (speed > 0) {
    // forward
    gpioWrite(m_in1_pin, 1);
    gpioPWM(m_in2_pin, duty);

  }
  else if (speed < 0) {
    // reverse
    gpioPWM(m_in1_pin, duty);
    gpioWrite(m_in2_pin, 1);
  }
  else {
    // zero → brake
    brake();
  }
}

void DRV8871::brake()
{
  gpioWrite(m_in1_pin, 1);
  gpioWrite(m_in2_pin, 1);
}

void DRV8871::coast()
{
  gpioWrite(m_in1_pin, 0);
  gpioWrite(m_in2_pin, 0);
}


