// Copyright 2021 Australian Robotics Supplies & Technology
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef UBLOX_DGNSS_NODE__UBX__UTILS_HPP_
#define UBLOX_DGNSS_NODE__UBX__UTILS_HPP_
#include <string>
#include <sstream>
#include <vector>
#include "ublox_dgnss_node/ubx/ubx_types.hpp"

namespace ubx
{
// inline u1_t* u2_to_u1_array(u2_t u2) {
//   u1_t *u1_array = new u1_t[2];
//   u1_array[0] = u2 & 0xff;
//   u1_array[1] = u2 >> 8;
//   return u1_array;
// }

// inline i1_t* i2_to_i1_array(i2_t i2) {
//   i1_t *i1_array = new i1_t[2];
//   i1_array[0] = i2 & 0xff;
//   i1_array[1] = i2 >> 8;
//   return i1_array;
// }

// inline x1_t* x2_to_x1_array(x2_t x2) {
//   x1_t *x1_array = new x1_t[2];
//   x1_array[0] = x2 & 0xff;
//   x1_array[1] = x2 >> 8;
//   return x1_array;
// }

// inline u1_t* u4_to_u1_array(u4_t u4) {
//   u1_t *u1_array = new u1_t[4];
//   u1_array[0] = u4 & 0xff;
//   u1_array[1] = u4 >> 8;
//   u1_array[2] = u4 >> 16;
//   u1_array[3] = u4 >> 24;
//   return u1_array;
// }

// inline i1_t* i4_to_i1_array(i4_t i4) {
//   i1_t *i1_array = new i1_t[4];
//   i1_array[0] = i4 & 0xff;
//   i1_array[1] = i4 >> 8;
//   i1_array[2] = i4 >> 16;
//   i1_array[3] = i4 >> 24;
//   return i1_array;
// }

// inline x1_t* x4_to_x1_array(x4_t x4) {
//   x1_t *x1_array = new x1_t[4];
//   x1_array[0] = x4 & 0xff;
//   x1_array[1] = x4 >> 8;
//   x1_array[2] = x4 >> 16;
//   x1_array[3] = x4 >> 24;
//   return x1_array;
// }

inline void buf_append_u2(std::vector<u1_t> * buf, u2_t u2)
{
  buf->push_back(u2 & 0xff);
  buf->push_back(u2 >> 8);
}

inline void buf_append_i2(std::vector<u1_t> * buf, i2_t i2)
{
  buf->push_back(i2 & 0xff);
  buf->push_back(i2 >> 8);
}

inline void buf_append_x2(std::vector<u1_t> * buf, x2_t x2)
{
  buf->push_back(x2 & 0xff);
  buf->push_back(x2 >> 8);
}

inline void buf_append_u4(std::vector<u1_t> * buf, u4_t u4)
{
  buf->push_back(u4 & 0xff);
  buf->push_back(u4 >> 8);
  buf->push_back(u4 >> 16);
  buf->push_back(u4 >> 24);
}

inline void buf_append_i4(std::vector<u1_t> * buf, i4_t i4)
{
  buf->push_back(i4 & 0xff);
  buf->push_back(i4 >> 8);
  buf->push_back(i4 >> 16);
  buf->push_back(i4 >> 24);
}

inline void buf_append_x4(std::vector<u1_t> * buf, x4_t x4)
{
  buf->push_back(x4 & 0xff);
  buf->push_back(x4 >> 8);
  buf->push_back(x4 >> 16);
  buf->push_back(x4 >> 24);
}

// todo check this
inline void buf_append_r4(std::vector<u1_t> * buf, r4_t r4)
{
  r4_bin_t r4_bin = {r4};
  buf->push_back(r4_bin.bytes[0]);
  buf->push_back(r4_bin.bytes[1]);
  buf->push_back(r4_bin.bytes[2]);
  buf->push_back(r4_bin.bytes[3]);
}

inline void buf_append_r4_bin(std::vector<u1_t> * buf, r4_bin_t r4_bin)
{
  buf->push_back(r4_bin.bytes[0]);
  buf->push_back(r4_bin.bytes[1]);
  buf->push_back(r4_bin.bytes[2]);
  buf->push_back(r4_bin.bytes[3]);
}

// todo check this
inline void buf_append_r8(std::vector<u1_t> * buf, r8_t r8)
{
  r8_bin_t r8_bin = {r8};
  buf->push_back(r8_bin.bytes[0]);
  buf->push_back(r8_bin.bytes[1]);
  buf->push_back(r8_bin.bytes[2]);
  buf->push_back(r8_bin.bytes[3]);
  buf->push_back(r8_bin.bytes[4]);
  buf->push_back(r8_bin.bytes[5]);
  buf->push_back(r8_bin.bytes[6]);
  buf->push_back(r8_bin.bytes[7]);
}

inline void buf_append_r8_bin(std::vector<u1_t> * buf, r8_bin_t r8_bin)
{
  buf->push_back(r8_bin.bytes[0]);
  buf->push_back(r8_bin.bytes[1]);
  buf->push_back(r8_bin.bytes[2]);
  buf->push_back(r8_bin.bytes[3]);
  buf->push_back(r8_bin.bytes[4]);
  buf->push_back(r8_bin.bytes[5]);
  buf->push_back(r8_bin.bytes[6]);
  buf->push_back(r8_bin.bytes[7]);
}

// inline u2_t buf_offset_u2(std::vector<u1_t> *buf, uint16_t offset) {
//   auto value = (u2_t)buf->data()+offset;
//   return swap_endian(value);
// }

// inline i2_t buf_offset_i2(std::vector<u1_t> *buf, uint16_t offset) {
//   auto value = (i2_t)buf->data()+offset;
//   return swap_endian(value);
// }

// inline u4_t buf_offset_u4(std::vector<u1_t> *buf, uint16_t offset) {
//   auto value = (u2_t)buf->data()+offset;
//   return swap_endian(value);
// }

// inline i4_t buf_offset_i4(std::vector<u1_t> *buf, uint16_t offset) {
//   auto ptr = buf->data()+offset;
//   i4_t value = (ptr[0] << 0 | ptr[0] << 8 | ptr[0] << 16 | ptr[0] << 24);
//   return value;
// }

template<typename T>
inline T swap_endian(T & value)
{
  union U {
    T value;
    std::array<ubx::u1_t, sizeof(T)> bytes;
  } src, dst;

  src.value = value;
  std::reverse_copy(src.bytes.begin(), src.bytes.end(), dst.bytes.begin());
  return dst.value;
}

template<typename T>
inline T buf_offset(std::vector<u1_t> * buf, u2_t offset)
{
  T value;
  memcpy(&value, buf->data() + offset, sizeof(T));
  return value;
}

template<typename T>
std::string to_hex(T & value)
{
  std::ostringstream oss;
  size_t width = sizeof(value) / 4;
  oss << std::setfill('0') << std::setw(width) << std::right << std::hex << +value;
  return oss.str();
}
}  // namespace ubx
#endif  // UBLOX_DGNSS_NODE__UBX__UTILS_HPP_
