#include "atd5833.h"
#include "ros/ros.h"

#include <unistd.h>  // for delay
#include <vector>

#include <pigpiod_if2.h>

ATD5833::ATD5833(int pi,   uint8_t step_pin,
                           uint8_t dir_pin,
                           uint8_t ms1_pin,
                           uint8_t ms2_pin)
  : m_PI(pi),
    step_pin_(step_pin),
    dir_pin_(dir_pin),
    ms1_pin_(ms1_pin),
    ms2_pin_(ms2_pin)
{
  // Configure pins as outputs
  set_mode(m_PI, step_pin_,   PI_OUTPUT);
  set_mode(m_PI, dir_pin_,    PI_OUTPUT);
  set_mode(m_PI, ms1_pin_,    PI_OUTPUT);
  set_mode(m_PI, ms2_pin_,    PI_OUTPUT);

  // Safe defaults:
  gpio_write(m_PI, dir_pin_,    1);  // forward
  gpio_write(m_PI, step_pin_,   0);
  
  // default to full-step
  setMicrostepMode(MicrostepMode::FULL);
}

ATD5833::~ATD5833(){}

void ATD5833::setDirection(bool dir)
{
  gpio_write(m_PI, dir_pin_, dir ? 1 : 0);
}

void ATD5833::setMicrostepMode(MicrostepMode mode)
{
  switch (mode) {
    case FULL:
      set_mode(m_PI, ms1_pin_,    PI_OUTPUT);
      set_mode(m_PI, ms2_pin_,    PI_OUTPUT);
      gpio_write(m_PI, ms1_pin_, 0);
      gpio_write(m_PI, ms2_pin_, 0);
      m_microstep_size = 16;
      break;
    case HALF:
      set_mode(m_PI, ms1_pin_,    PI_OUTPUT);
      set_mode(m_PI, ms2_pin_,    PI_OUTPUT);
      gpio_write(m_PI, ms1_pin_, 1);
      gpio_write(m_PI, ms2_pin_, 0);
      m_microstep_size = 8;
      break;
    case QUARTER:
      set_mode(m_PI, ms1_pin_,    PI_OUTPUT);
      set_mode(m_PI, ms2_pin_,    PI_INPUT);
      gpio_write(m_PI, ms1_pin_, 1);
      set_pull_up_down(m_PI, ms2_pin_, PI_PUD_OFF);
      m_microstep_size = 4;
      break;
    case SIXTEENTH:
      set_mode(m_PI, ms1_pin_,    PI_INPUT);
      set_mode(m_PI, ms2_pin_,    PI_INPUT);
      set_pull_up_down(m_PI, ms1_pin_, PI_PUD_OFF);
      set_pull_up_down(m_PI, ms2_pin_, PI_PUD_OFF);
      m_microstep_size = 1;
      break;
  }
}

int ATD5833::getMicrostepSize() {
  return m_microstep_size;
}


void ATD5833::step_sync(uint32_t steps, uint32_t period_us)
{
  for (uint32_t i = 0; i < steps; ++i) {
    gpio_write(m_PI, step_pin_, 1);
    time_sleep(((double)period_us) / 2.0 / 1000000.0);

    gpio_write(m_PI, step_pin_, 0);
    time_sleep(((double)period_us) / 2.0 / 1000000.0);
  }
}

void ATD5833::step_async(uint32_t steps, uint32_t period_us)
{
  ROS_INFO("Creating vawe for steps: %d, period_us: %d", steps, period_us);

  wave_clear(m_PI);

  std::vector<gpioPulse_t> pulses;
  pulses.reserve(steps * 2);
  
  uint32_t mask = 1u << step_pin_;
  for (uint32_t i = 0; i < steps; ++i) {
    pulses.push_back({mask, 0, period_us / 2});
    pulses.push_back({0,   mask, period_us / 2});
  }

  wave_add_generic(m_PI, pulses.size(), pulses.data());
  int wid = wave_create(m_PI);
  wave_send_once(m_PI, wid);   // returns straight away

  ROS_INFO("Wave sent! id: %d", wid);
}

bool ATD5833::is_stepping() {
  return wave_tx_busy(m_PI);
}