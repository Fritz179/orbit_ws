#include "a4988.h"
#include <pigpiod_if2.h>
#include <unistd.h>  // for gpioDelay
#include <vector>

A4988::A4988(uint8_t step_pin,
                           uint8_t dir_pin,
                           uint8_t enable_pin,
                           uint8_t ms1_pin,
                           uint8_t ms2_pin,
                           uint8_t ms3_pin)
  : step_pin_(step_pin),
    dir_pin_(dir_pin),
    enable_pin_(enable_pin),
    ms1_pin_(ms1_pin),
    ms2_pin_(ms2_pin),
    ms3_pin_(ms3_pin)
{
  // Configure pins as outputs
  gpioSetMode(step_pin_,   PI_OUTPUT);
  gpioSetMode(dir_pin_,    PI_OUTPUT);
  gpioSetMode(enable_pin_, PI_OUTPUT);
  gpioSetMode(ms1_pin_,    PI_OUTPUT);
  gpioSetMode(ms2_pin_,    PI_OUTPUT);
  gpioSetMode(ms3_pin_,    PI_OUTPUT);

  // Safe defaults:
  gpioWrite(enable_pin_, 1);  // disabled (active-low)
  gpioWrite(dir_pin_,    1);  // forward
  gpioWrite(step_pin_,   0);
  
  // default to full-step
  gpioWrite(ms1_pin_, 0);
  gpioWrite(ms2_pin_, 0);
  gpioWrite(ms3_pin_, 0);
}

A4988::~A4988()
{
  disable();
}

void A4988::enable()
{
  // A4988 ENABLE is active-low
  gpioWrite(enable_pin_, 0);
}

void A4988::disable()
{
  gpioWrite(enable_pin_, 1);
}

void A4988::setDirection(bool dir)
{
  gpioWrite(dir_pin_, dir ? 1 : 0);
}

void A4988::setMicrostepMode(MicrostepMode mode)
{
  switch (mode) {
    case FULL:
      gpioWrite(ms1_pin_, 0);
      gpioWrite(ms2_pin_, 0);
      gpioWrite(ms3_pin_, 0);
      break;
    case HALF:
      gpioWrite(ms1_pin_, 1);
      gpioWrite(ms2_pin_, 0);
      gpioWrite(ms3_pin_, 0);
      break;
    case QUARTER:
      gpioWrite(ms1_pin_, 0);
      gpioWrite(ms2_pin_, 1);
      gpioWrite(ms3_pin_, 0);
      break;
    case EIGHTH:
      gpioWrite(ms1_pin_, 1);
      gpioWrite(ms2_pin_, 1);
      gpioWrite(ms3_pin_, 0);
      break;
    case SIXTEENTH:
      gpioWrite(ms1_pin_, 1);
      gpioWrite(ms2_pin_, 1);
      gpioWrite(ms3_pin_, 1);
      break;
  }
}

void A4988::step_sync(uint32_t steps, uint32_t period_us)
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

void A4988::step_async(uint32_t steps, uint32_t period_us)
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

bool A4988::is_stepping() {
  return gpioWaveTxBusy();
}
