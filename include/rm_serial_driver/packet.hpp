// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace rm_serial_driver
{
struct ReceivePacket
{
  uint8_t header[2];
  uint8_t detect_color : 1;  // 0-red 1-blue
  bool reset_tracker : 1;
  uint8_t reserved : 6;
  float roll;
  float pitch;
  float yaw;
  float aim_x;
  float aim_y;
  float aim_z;
  uint16_t checksum = 0;
} __attribute__((packed));

struct SendPacket
{
  uint8_t header = 0xA5;
  bool tracking : 1;
  uint8_t id : 3;          // 0-outpost 6-guard 7-base
  uint8_t armors_num : 3;  // 2-balance 3-outpost 4-normal
  uint8_t reserved : 1;
  float x;
  float y;
  float z;
  float yaw;
  float vx;
  float vy;
  float vz;
  float v_yaw;
  float r1;
  float r2;
  float dz;
  uint16_t checksum = 0;
} __attribute__((packed));


struct color_data
{
  uint8_t my_color;
} __attribute__((packed));

struct gimbal_data
{
  float rel_yaw;
  float rel_pitch;
  // Search Target is 0. Move Yoke is 1.
  uint8_t mode;
  uint8_t debug_int;
} __attribute__((packed));

struct chassis_data
{
  float vx;
  float vy;
  float vw;
} __attribute__((packed));

/* 
HEADER	2 ASCII char	2 bytes
MY_COLOR	uint8_t ENUM	1 byte
CRC_CHECKSUM	uint8_t checksum	1 bytes
PACK_END	2 ASCII char	2 bytes
*/

struct old_packet{
  uint8_t header[2];
  uint8_t my_color;
  uint8_t crc_checksum;
  uint8_t pack_end[2];
} __attribute__((packed));

typedef color_data color_data_t;
typedef gimbal_data gimbal_data_t;
typedef chassis_data chassis_data_t;
typedef old_packet old_packet_t;

inline ReceivePacket fromVector(const std::vector<uint8_t> & data)
{
  ReceivePacket packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

inline std::vector<uint8_t> toVector(const SendPacket & data)
{
  std::vector<uint8_t> packet(sizeof(SendPacket));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
  return packet;
}

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_
