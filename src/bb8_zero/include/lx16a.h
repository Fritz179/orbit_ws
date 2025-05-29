#ifndef LX16A_H
#define LX16A_H

#include <cstdint>

class LX16A {
public:
  LX16A(int pi, int handle, int ID);
  ~LX16A();

  void set_speed(int speed);
  void set_ID(int ID);

  double get_voltage();
  int get_position();
  int get_ID();

private:
  int m_PI;
  int m_handle;
  int m_ID;

  std::vector<uint8_t> read_bytes(int count);
  std::vector<uint8_t> read_command(char command);
};

#endif  // LX16A_H