#ifndef DRV8871_H
#define DRV8871_H

#include <pigpiod_if2.h>
#include <cstdint>
#include <algorithm>

/**
 * @brief Driver for one channel of a DRV8871 motor driver
 */
class DRV8871 {
public:
  /**
   * @param in1_pin   GPIO pin for direction bit 1
   * @param in2_pin   GPIO pin for direction bit 2
   */
  DRV8871(uint8_t in1_pin, uint8_t in2_pin);

  /**
   * @brief Set motor speed
   * @param speed  –255..+255 (sign = dir, magnitude = duty)
   */
  void setSpeed(int16_t speed);

  /**
   * @brief Brake the motor (both inputs high)
   */
  void brake();

    /**
   * @brief Coast the motor (both inputs low)
   */
  void coast();

private:
  uint8_t m_in1_pin, m_in2_pin;

  // disable copy
  DRV8871(const DRV8871&) = delete;
  DRV8871& operator=(const DRV8871&) = delete;
};

#endif // DRV8871_H