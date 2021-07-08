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

#ifndef UBLOX_DGNSS_NODE_UBX_EXCEPTIONS_HPP
#define UBLOX_DGNSS_NODE_UBX_EXCEPTIONS_HPP
#include <stdexcept>
#include <string>
namespace ubx {
  class UbxAckNackException : public std::runtime_error {
    public:
      UbxAckNackException(std::string msg):std::runtime_error(msg){} 
  };

  class UbxValueException : public std::runtime_error {
    public:
      UbxValueException(std::string msg):std::runtime_error(msg){}
  };
  
  class UbxPayloadException : public std::runtime_error {
    public:
      UbxPayloadException(std::string msg):std::runtime_error(msg){}
  };

}

#endif