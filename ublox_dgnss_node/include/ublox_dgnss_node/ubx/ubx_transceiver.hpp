// Copyright 2025 GreenForge Labs
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

/**
 * @file ubx_transceiver.hpp
 * @brief Definition of the UbxTransceiver interface for UBX message communication
 */

#ifndef UBLOX_DGNSS_NODE__UBX__UBX_TRANSCEIVER_HPP_
#define UBLOX_DGNSS_NODE__UBX__UBX_TRANSCEIVER_HPP_

#include <memory>
#include <vector>
#include "ublox_dgnss_node/ubx/ubx.hpp"

namespace ubx
{

/**
 * @brief Status of a write operation, indicating ACK/NAK or communication failure.
 */
enum class AckNack
{
  ACK,   //< Message acknowledged
  NACK,  //< Message not acknowledged
  NONE   //< No response / Timeout / Communication error
};

/**
 * @brief Result of a write operation.
 */
struct WriteResult
{
  AckNack status;
};

/**
 * @brief Status of a read operation.
 */
enum class ReadStatus
{
  SUCCESS,  //< Message successfully read
  TIMEOUT,  //< Read operation timed out
  NO_DATA,  //< No data available to read (e.g. for a non-blocking poll-like read)
  ERROR     //< Other communication error
};

/**
 * @brief Result of a read operation.
 */
struct ReadResult
{
  ReadStatus status;
};

/**
 * @brief Interface for UBX message communication
 *
 * This interface abstracts the communication with a UBX device,
 * allowing for different transport mechanisms (USB, serial, etc.)
 */
class UbxTransceiver
{
public:
  /**
   * @brief Virtual destructor
   */
  virtual ~UbxTransceiver() = default;

  /**
   * @brief Write a UBX message to the device and get ACK/NAK status.
   * @param frame Shared pointer to the constant Frame object to be written.
   * @return WriteResult indicating status of the write and acknowledgement.
   */
  virtual WriteResult write(std::shared_ptr<const ubx::Frame> frame) = 0;

  /**
   * @brief Read a UBX message from the device.
   * @param frame Shared pointer to a Frame object to be populated with the read message.
   *              The caller should ensure this points to a valid object or handle nullptr if read creates it.
   * @param timeout_ms Timeout in milliseconds for the read operation.
   * @return ReadResult indicating status of the read operation.
   */
  virtual ReadResult read(std::shared_ptr<ubx::Frame> & frame, int timeout_ms = 1000) = 0;

  /**
   * @brief Check if the transceiver is open/connected.
   * @return True if open/connected, false otherwise.
   */
  virtual bool is_open() = 0;

  /**
   * @brief Open/connect to the device.
   * @return True if opening/connection was successful, false otherwise.
   */
  virtual bool open() = 0;

  /**
   * @brief Close/disconnect from the device.
   */
  virtual void close() = 0;
};

}  // namespace ubx

#endif  // UBLOX_DGNSS_NODE__UBX__UBX_TRANSCEIVER_HPP_
