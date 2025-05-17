#include "atd5833.h"
#include <pigpio.h>
#include <unistd.h>  // for gpioDelay
#include <vector>

ATD5833::ATD5833(uint8_t step_pin,
                           uint8_t dir_pin,
                           uint8_t enable_pin,
                           uint8_t ms1_pin,
                           uint8_t ms2_pin)
  : step_pin_(step_pin),
    dir_pin_(dir_pin),
    enable_pin_(enable_pin),
    ms1_pin_(ms1_pin),
    ms2_pin_(ms2_pin)
{
  // Configure pins as outputs
  gpioSetMode(step_pin_,   PI_OUTPUT);
  gpioSetMode(dir_pin_,    PI_OUTPUT);
  gpioSetMode(enable_pin_, PI_OUTPUT);
  gpioSetMode(ms1_pin_,    PI_OUTPUT);
  gpioSetMode(ms2_pin_,    PI_OUTPUT);

  // Safe defaults:
  disable();
  gpioWrite(dir_pin_,    1);  // forward
  gpioWrite(step_pin_,   0);
  
  // default to full-step
  setMicrostepMode(MicrostepMode::FULL);
}

ATD5833::~ATD5833()
{
  disable();
}

void ATD5833::enable()
{
  // ATD5833 ENABLE is active-low
  gpioWrite(enable_pin_, 0);
}

void ATD5833::disable()
{
  gpioWrite(enable_pin_, 1);
}

void ATD5833::setDirection(bool dir)
{
  gpioWrite(dir_pin_, dir ? 1 : 0);
}

void ATD5833::setMicrostepMode(MicrostepMode mode)
{
  switch (mode) {
    case FULL:
      gpioSetMode(ms1_pin_,    PI_OUTPUT);
      gpioSetMode(ms2_pin_,    PI_OUTPUT);
      gpioWrite(ms1_pin_, 0);
      gpioWrite(ms2_pin_, 0);
      m_microstep_size = 16;
      break;
    case HALF:
      gpioSetMode(ms1_pin_,    PI_OUTPUT);
      gpioSetMode(ms2_pin_,    PI_OUTPUT);
      gpioWrite(ms1_pin_, 1);
      gpioWrite(ms2_pin_, 0);
      m_microstep_size = 8;
      break;
    case QUARTER:
      gpioSetMode(ms1_pin_,    PI_OUTPUT);
      gpioSetMode(ms2_pin_,    PI_INPUT);
      gpioWrite(ms1_pin_, 1);
      gpioSetPullUpDown(ms2_pin_, PI_PUD_OFF);
      m_microstep_size = 4;
      break;
    case SIXTEENTH:
      gpioSetMode(ms1_pin_,    PI_INPUT);
      gpioSetMode(ms2_pin_,    PI_INPUT);
      gpioSetPullUpDown(ms1_pin_, PI_PUD_OFF);
      gpioSetPullUpDown(ms2_pin_, PI_PUD_OFF);
      m_microstep_size = 1;
      break;
  }
}

int ATD5833::getMicrostepSize() {
  return m_microstep_size;
}


void ATD5833::step_sync(uint32_t steps, uint32_t period_us)
{
  // Ensure period is at least as long as pulse width * 2
  if (period_us < kPulseWidthUs * 2) {
    period_us = kPulseWidthUs * 2;
  }

  for (uint32_t i = 0; i < steps; ++i) {
    gpioWrite(step_pin_, 1);
    gpioDelay(period_us / 2);

    gpioWrite(step_pin_, 0);
    gpioDelay(period_us / 2);
  }
}

void ATD5833::step_async(uint32_t steps, uint32_t period_us)
{
  gpioWaveClear();

  std::vector<gpioPulse_t> pulses;
  pulses.reserve(steps * 2);
  
  uint32_t mask = 1u << step_pin_;
  for (uint32_t i = 0; i < steps; ++i) {
    pulses.push_back({mask, 0, kPulseWidthUs});
    pulses.push_back({0,   mask, period_us - kPulseWidthUs});
  }

  gpioWaveAddGeneric(pulses.size(), pulses.data());
  int wid = gpioWaveCreate();
  gpioWaveTxSend(wid, PI_WAVE_MODE_ONE_SHOT);   // returns straight away
}

bool ATD5833::is_stepping() {
  return gpioWaveTxBusy();
}
