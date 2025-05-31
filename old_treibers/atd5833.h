#ifndef ATD5833_H
#define ATD5833_H

#include <cstdint>

class ATD5833 {
public:
  enum MicrostepMode {
    FULL,       // MS1=0,  MS2=0
    HALF,       // MS1=1,  MS2=0
    QUARTER,    // MS1=1,  MS2=NC
    SIXTEENTH   // MS1=NC, MS2=NC
  };

  /**
   * @param step_pin    GPIO pin connected to ATD5833 STEP
   * @param dir_pin     GPIO pin connected to ATD5833 DIR
   * @param enable_pin  GPIO pin connected to ATD5833 ENABLE (active low)
   * @param ms1_pin     GPIO pin connected to ATD5833 MS1
   * @param ms2_pin     GPIO pin connected to ATD5833 MS2
   */
  ATD5833(int pi, uint8_t step_pin,
               uint8_t dir_pin,
               uint8_t ms1_pin,
               uint8_t ms2_pin);

  ~ATD5833();

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
   * Get microstep size.
   */
  int getMicrostepSize();

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
  int m_PI;

  uint8_t step_pin_;
  uint8_t dir_pin_;
  uint8_t enable_pin_;
  uint8_t ms1_pin_;
  uint8_t ms2_pin_;

  int m_microstep_size;
};

#endif  // ATD5833_H