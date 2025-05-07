#ifndef TB6612FNG_H
#define TB6612FNG_H

#include <pigpio.h>
#include <cstdint>
#include <stdexcept>
#include <algorithm>

#include "NC.h"

/**
 * @brief Driver for one channel of a TB6612FNG motor driver
 */
class TB6612FNGMotor {
public:
  /**
   * @param pwm_pin   GPIO pin for PWM (0–255 duty)
   * @param in1_pin   GPIO pin for direction bit 1
   * @param in2_pin   GPIO pin for direction bit 2
   */
  TB6612FNGMotor(uint8_t pwm_pin, uint8_t in1_pin, uint8_t in2_pin);

  /**
   * @brief Set motor speed
   * @param speed  –255..+255 (sign = dir, magnitude = duty)
   */
  void setSpeed(int16_t speed);

  /**
   * @brief Brake the motor (both inputs high, PWM=0)
   */
  void brake();

private:
  uint8_t pwm_pin_, in1_pin_, in2_pin_;

  // disable copy
  TB6612FNGMotor(const TB6612FNGMotor&) = delete;
  TB6612FNGMotor& operator=(const TB6612FNGMotor&) = delete;
};


/**
 * @brief Driver for the full TB6612FNG (2 channels + standby)
 */
class TB6612FNG {
public:
  /**
   * @param pwma, ain1, ain2  pins for Motor A
   * @param pwmb, bin1, bin2  pins for Motor B
   */
  TB6612FNG(uint8_t pwma, uint8_t ain1, uint8_t ain2,
            uint8_t pwmb, uint8_t bin1, uint8_t bin2);

  ~TB6612FNG();

  /** Access Motor A */
  TB6612FNGMotor& A() { return motorA_; }

  /** Access Motor B */
  TB6612FNGMotor& B() { return motorB_; }

private:
  TB6612FNGMotor motorA_, motorB_;
  
  // disable copy
  TB6612FNG(const TB6612FNG&) = delete;
  TB6612FNG& operator=(const TB6612FNG&) = delete;
};

#endif // TB6612FNG_H