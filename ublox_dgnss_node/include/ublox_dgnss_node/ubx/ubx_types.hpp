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

#ifndef UBLOX_DGNSS_NODE__UBX__UBX_TYPES_HPP_
#define UBLOX_DGNSS_NODE__UBX__UBX_TYPES_HPP_

#include <cstdint>
#include <cmath>

namespace ubx
{

using u1_t = uint8_t;
using i1_t = int8_t;
using x1_t = uint8_t;
using u2_t = uint16_t;
using i2_t = int16_t;
using x2_t = uint16_t;
using u4_t = uint32_t;
using i4_t = int32_t;
using x4_t = uint32_t;
using u8_t = uint64_t;
using i8_t = int64_t;
using x8_t = uint64_t;
using r4_t = float_t;
using r8_t = double_t;
using ch_t = unsigned char;
using l_t = u1_t;   // single bit boolean (true =1, false =0) stored as u1_t

using msg_class_t = uint8_t;
using msg_id_t = uint8_t;

union float_u1_bytes_t {
  float_t f;
  u1_t bytes[sizeof(float_t)];
};
using r4_bin_t = float_u1_bytes_t;

union double_u1_bytes_t {
  double_t f;
  u1_t bytes[sizeof(double_t)];
};
using r8_bin_t = double_u1_bytes_t;

// allowed Ubx Types for for configuration values
enum ubx_type_t {L, U1, I1, E1, X1,
  U2, I2, E2, X2,
  U4, I4, E4, X4, R4,
  U8, I8, X8, R8};

// preamble sync characters
const u1_t UBX_SYNC_CHAR_1 = 0xB5;
const u1_t UBX_SYNC_CHAR_2 = 0x62;

union value_t {
  u1_t bytes[8];
  l_t l : 1;                // single bit boolen
  u1_t u1 : 8;
  i1_t i1 : 8;
  x1_t x1 : 8;
  u2_t u2 : 16;
  i2_t i2 : 16;
  x2_t x2 : 16;
  u4_t u4 : 32;
  i4_t i4 : 32;
  x4_t x4 : 32;
  r4_t r4;
  u8_t u8 : 64;
  i8_t i8 : 64;
  x8_t x8 : 64;
  r8_t r8;
};
}  // namespace ubx
#endif  // UBLOX_DGNSS_NODE__UBX__UBX_TYPES_HPP_
