// CRC for unitree_hg::msg::LowCmd, adapted from unitree_ros2 (example/src/src/common/motor_crc_hg.cpp).
// The raw structs below must keep Unitree's exact (unpacked) layout, since the CRC runs over their memory.

#ifndef UNITREE_G1_DRIVER__MOTOR_CRC_HG_HPP_
#define UNITREE_G1_DRIVER__MOTOR_CRC_HG_HPP_

#include <array>
#include <cstdint>
#include <cstring>

#include "unitree_hg/msg/low_cmd.hpp"

namespace unitree_g1_driver
{
namespace crc_detail
{
struct RawMotorCmd
{
  uint8_t mode;
  float q;
  float dq;
  float tau;
  float kp;
  float kd;
  uint32_t reserve = 0;
};

struct RawLowCmd
{
  uint8_t mode_pr;
  uint8_t mode_machine;
  std::array<RawMotorCmd, 35> motor_cmd;
  std::array<uint32_t, 4> reserve;
  uint32_t crc;
};

inline uint32_t crc32_core(const uint32_t * ptr, uint32_t len)
{
  uint32_t crc = 0xFFFFFFFF;
  const uint32_t polynomial = 0x04c11db7;
  for (uint32_t i = 0; i < len; i++)
  {
    uint32_t xbit = 1u << 31;
    const uint32_t data = ptr[i];
    for (uint32_t bits = 0; bits < 32; bits++)
    {
      if (crc & 0x80000000)
      {
        crc <<= 1;
        crc ^= polynomial;
      }
      else
      {
        crc <<= 1;
      }
      if (data & xbit)
      {
        crc ^= polynomial;
      }
      xbit >>= 1;
    }
  }
  return crc;
}
}  // namespace crc_detail

inline void fill_crc(unitree_hg::msg::LowCmd & msg)
{
  crc_detail::RawLowCmd raw{};
  raw.mode_pr = msg.mode_pr;
  raw.mode_machine = msg.mode_machine;
  for (size_t i = 0; i < raw.motor_cmd.size(); ++i)
  {
    raw.motor_cmd[i].mode = msg.motor_cmd[i].mode;
    raw.motor_cmd[i].q = msg.motor_cmd[i].q;
    raw.motor_cmd[i].dq = msg.motor_cmd[i].dq;
    raw.motor_cmd[i].tau = msg.motor_cmd[i].tau;
    raw.motor_cmd[i].kp = msg.motor_cmd[i].kp;
    raw.motor_cmd[i].kd = msg.motor_cmd[i].kd;
    raw.motor_cmd[i].reserve = msg.motor_cmd[i].reserve;
  }
  std::memcpy(raw.reserve.data(), msg.reserve.data(), sizeof(raw.reserve));
  msg.crc = crc_detail::crc32_core(
    reinterpret_cast<const uint32_t *>(&raw), (sizeof(crc_detail::RawLowCmd) >> 2) - 1);
}

}  // namespace unitree_g1_driver

#endif  // UNITREE_G1_DRIVER__MOTOR_CRC_HG_HPP_
