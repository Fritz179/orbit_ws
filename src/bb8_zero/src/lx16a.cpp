#include "ros/ros.h"

#include <lx16a.h>
#include <pigpiod_if2.h>

#define GET_LOW_BYTE(A) (uint8_t)((A))
#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8)
#define BYTES_TO_INT(A, B) ((int16_t)((((uint16_t)(A)) << 8) | (uint8_t)(B)))

#define LOBOT_SERVO_FRAME_HEADER         0x55
#define LOBOT_SERVO_MOVE_TIME_WRITE      1
#define LOBOT_SERVO_MOVE_TIME_READ       2
#define LOBOT_SERVO_MOVE_TIME_WAIT_WRITE 7
#define LOBOT_SERVO_MOVE_TIME_WAIT_READ  8
#define LOBOT_SERVO_MOVE_START           11
#define LOBOT_SERVO_MOVE_STOP            12
#define LOBOT_SERVO_ID_WRITE             13
#define LOBOT_SERVO_ID_READ              14
#define LOBOT_SERVO_ANGLE_OFFSET_ADJUST  17
#define LOBOT_SERVO_ANGLE_OFFSET_WRITE   18
#define LOBOT_SERVO_ANGLE_OFFSET_READ    19
#define LOBOT_SERVO_ANGLE_LIMIT_WRITE    20
#define LOBOT_SERVO_ANGLE_LIMIT_READ     21
#define LOBOT_SERVO_VIN_LIMIT_WRITE      22
#define LOBOT_SERVO_VIN_LIMIT_READ       23
#define LOBOT_SERVO_TEMP_MAX_LIMIT_WRITE 24
#define LOBOT_SERVO_TEMP_MAX_LIMIT_READ  25
#define LOBOT_SERVO_TEMP_READ            26
#define LOBOT_SERVO_VIN_READ             27
#define LOBOT_SERVO_POS_READ             28
#define LOBOT_SERVO_OR_MOTOR_MODE_WRITE  29
#define LOBOT_SERVO_OR_MOTOR_MODE_READ   30
#define LOBOT_SERVO_LOAD_OR_UNLOAD_WRITE 31
#define LOBOT_SERVO_LOAD_OR_UNLOAD_READ  32
#define LOBOT_SERVO_LED_CTRL_WRITE       33
#define LOBOT_SERVO_LED_CTRL_READ        34
#define LOBOT_SERVO_LED_ERROR_WRITE      35
#define LOBOT_SERVO_LED_ERROR_READ       36

char lobot_checksum(char buf[]) {
  char i;
  uint16_t temp = 0;
  for (i = 2; i < buf[3] + 2; i++) {
    temp += buf[i];
  }
  temp = ~temp;
  i = (char)temp;
  return i;
}


LX16A::LX16A(int pi, int handle, int ID): m_PI(pi), m_handle(handle), m_ID(ID) {

}

// TODO: Stop
LX16A::~LX16A() {

}

std::vector<uint8_t> LX16A::read_bytes(int count) {
  for (int i = 0; i < 1000; i++) {
    int available = serial_data_available(m_PI, m_handle);

    if (available >= count) break;
  }

  std::vector<uint8_t> data(count);
  int read = serial_read(m_PI, m_handle, reinterpret_cast<char*>(data.data()), count);

  if (read < count) {
    ROS_ERROR("Tried to read: %d, but got: %d", count, read);
    return {};
  }

  return data;
}

std::vector<uint8_t> LX16A::read_command(char command) {
    char buf[10];

    buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
    buf[2] = m_ID;
    buf[3] = 3;
    buf[4] = command;
    buf[5] = lobot_checksum(buf);
  
    serial_write(m_PI, m_handle, buf, 6);

  
    // Receiving 2xHeader, Id, Length, Command
    std::vector<uint8_t> data = read_bytes(5);

    if (data.at(3) < 1 || data.at(3) > 10) {
        ROS_ERROR("Invalid data length: %d", data.at(3));
        return data;
    }

    return read_bytes(data.at(3) - 2);
}

void LX16A::set_speed(int speed) {
    char buf[10];

    buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
    buf[2] = m_ID;
    buf[3] = 7;
    buf[4] = LOBOT_SERVO_OR_MOTOR_MODE_WRITE;
    buf[5] = 1;
    buf[6] = 0;
    buf[7] = GET_LOW_BYTE((uint16_t)speed);
    buf[8] = GET_HIGH_BYTE((uint16_t)speed);
    buf[9] = lobot_checksum(buf);

    serial_write(m_PI, m_handle, buf, 10);
}

void LX16A::set_ID(int ID) {
  char buf[7];

  buf[0] = buf[1] = LOBOT_SERVO_FRAME_HEADER;
  buf[2] = m_ID;
  buf[3] = 4;
  buf[4] = LOBOT_SERVO_ID_WRITE;
  buf[5] = ID;
  buf[6] = lobot_checksum(buf);

  serial_write(m_PI, m_handle, buf, 7);
}

double LX16A::get_voltage() {
    std::vector<uint8_t> data = read_command(LOBOT_SERVO_VIN_READ);

    int voltage = BYTES_TO_INT(data.at(1), data.at(0));
    return ((double)voltage) / 1000.0;
}

int LX16A::get_position() {
  std::vector<uint8_t> data = read_command(LOBOT_SERVO_POS_READ);

  return BYTES_TO_INT(data.at(1), data.at(0));
}

int LX16A::get_ID() {
    std::vector<uint8_t> data = read_command(LOBOT_SERVO_ID_READ);

    return data.at(0);
}

void LobotSerialServoSetID(int pi, int handle, uint8_t oldID, uint8_t newID)
{

}