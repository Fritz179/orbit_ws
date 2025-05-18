#include "l298n.h"
#include "ros/ros.h"
#include <pigpiod_if2.h>

L298N::L298N(int pi, uint8_t in1_pin, uint8_t in2_pin)
  : m_PI(pi),
    m_in1_pin(in1_pin),
    m_in2_pin(in2_pin)
{

  // Configure pins as outputs
  set_mode(m_PI, m_in1_pin, PI_OUTPUT);
  set_mode(m_PI, m_in2_pin, PI_OUTPUT);

  set_PWM_frequency(m_PI, m_in1_pin, 1000);
  set_PWM_frequency(m_PI, m_in2_pin, 1000);

  // Start stopped/braked
  brake();
}

void L298N::setSpeed(int16_t speed)
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

  ROS_INFO("Speed: %d, duty: %d", speed, duty);

  if (speed > 0) {
    // forward
    gpio_write(m_PI, m_in1_pin, 1);

    if (speed == 255) {
      gpio_write(m_PI, m_in2_pin, 0);
    } else {
      set_PWM_dutycycle(m_PI, m_in2_pin, duty);
    }
  } else if (speed < 0) {
    // reverse
    if (speed == -255) {
      gpio_write(m_PI, m_in1_pin, 0);
    } else {
      set_PWM_dutycycle(m_PI, m_in1_pin, duty);
    }

    gpio_write(m_PI, m_in2_pin, 1);
  } else {
    // zero → brake
    brake();
  }
}

void L298N::brake()
{
  gpio_write(m_PI, m_in1_pin, 1);
  gpio_write(m_PI, m_in2_pin, 1);
}

void L298N::coast()
{
  gpio_write(m_PI, m_in1_pin, 0);
  gpio_write(m_PI, m_in2_pin, 0);
}


