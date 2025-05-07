#ifndef LIMIT_SWITCH_H
#define LIMIT_SWITCH_H

#include <pigpio.h>
#include <cstdint>
#include "NC.h"

class LimitSwitch {
public:
  /**
   * @param vcc_pin     GPIO pin connected to LimitSwitch VCC
   * @param gnd_pin     GPIO pin connected to LimitSwitch GND
   * @param signal_pin  GPIO pin connected to LimitSwitch SIGNAL
   */
  LimitSwitch(uint8_t vcc_pin,
        uint8_t gnd_pin,
        uint8_t signal_pin)
    :   m_vcc_pin(vcc_pin),
        m_gnd_pin(gnd_pin),
        m_signal_pin(signal_pin)
    {
        if (m_vcc_pin != NC) {
            gpioSetMode(m_vcc_pin,    PI_OUTPUT);
            gpioWrite(m_vcc_pin, 1);
        }

        if (m_gnd_pin != NC) {
            gpioSetMode(m_gnd_pin,    PI_OUTPUT);
            gpioWrite(m_gnd_pin, 0);
        }

        gpioSetMode(m_signal_pin,    PI_INPUT);
    }

  ~LimitSwitch();

  /**
   * Read the limit switch.
   * @return Sate of the switch.
   */
  int read() {
    return gpioRead(m_signal_pin);
  }

  /**
   * Get GPIO pin number
   * @return GPIO pin.
   */
  uint8_t get_gpio() {
    return m_signal_pin;
  }

private:
  uint8_t m_vcc_pin;
  uint8_t m_gnd_pin;
  uint8_t m_signal_pin;
};

#endif  // LIMIT_SWITCH_H