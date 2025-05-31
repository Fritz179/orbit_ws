#ifndef A4988_H
#define A4988_H

#include <cstdint>
#include "NC.h"

class A4988 {
public:
  enum MicrostepMode {
    FULL,       // MS1=0, MS2=0, MS3=0
    HALF,       // MS1=1, MS2=0, MS3=0
    QUARTER,    // MS1=0, MS2=1, MS3=0
    EIGHTH,     // MS1=1, MS2=1, MS3=0
    SIXTEENTH   // MS1=1, MS2=1, MS3=1
  };

  /**
   * @param step_pin    GPIO pin connected to A4988 STEP
   * @param dir_pin     GPIO pin connected to A4988 DIR
   * @param enable_pin  GPIO pin connected to A4988 ENABLE (active low)
   * @param ms1_pin     GPIO pin connected to A4988 MS1
   * @param ms2_pin     GPIO pin connected to A4988 MS2
   * @param ms3_pin     GPIO pin connected to A4988 MS3
   */
  A4988(uint8_t step_pin,
               uint8_t dir_pin,
               uint8_t enable_pin,
               uint8_t ms1_pin,
               uint8_t ms2_pin,
               uint8_t ms3_pin);

  ~A4988();

  /** Enable the driver (allows stepping). */
  void enable();

  /** Disable the driver (forces high-impedance, no stepping). */
  void disable();

  /**
   * Set direction.
   * @param dir  true = forward, false = reverse
   */
  void setDirection(bool dir);

  /**
   * Select microstep resolution.
   */
  void setMicrostepMode(MicrostepMode mode);

  /**
   * Step the motor.
   * Blocks until all steps are sent.
   * @param steps      Number of step pulses to generate.
   * @param period_us  Total pulse period in microseconds (must be ≥ pulse width).
   */
  void step_sync(uint32_t steps, uint32_t period_us);

  /**
   * Step the motor async.
   * Doesn't block.
   * @param steps      Number of step pulses to generate.
   * @param period_us  Total pulse period in microseconds (must be ≥ pulse width).
   */
  void step_async(uint32_t steps, uint32_t period_us);

  /**
   * Check if the motor is stepping.
   * @return Is still stepping.
   */
  bool is_stepping();


private:
  uint8_t step_pin_;
  uint8_t dir_pin_;
  uint8_t enable_pin_;
  uint8_t ms1_pin_;
  uint8_t ms2_pin_;
  uint8_t ms3_pin_;

  static constexpr uint32_t kPulseWidthUs = 2;  // STEP pin high time
};

#endif  // A4988_H