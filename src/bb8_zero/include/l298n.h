#ifndef L298N_H
#define L298N_H

#include <cstdint>
#include <algorithm>

/**
 * @brief Driver for one channel of a L298N motor driver
 */
class L298N {
public:
  /**
   * @param in1_pin   GPIO pin for direction bit 1
   * @param in2_pin   GPIO pin for direction bit 2
   */
  L298N(int pi, uint8_t in1_pin, uint8_t in2_pin);

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
  int m_PI;
  uint8_t m_in1_pin, m_in2_pin;

  // disable copy
  L298N(const L298N&) = delete;
  L298N& operator=(const L298N&) = delete;
};

#endif // L298N_H