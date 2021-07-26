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

#ifndef UBLOX_DGNSS_NODE__CALLBACK_HPP_
#define UBLOX_DGNSS_NODE__CALLBACK_HPP_
#include <stdio.h>
#include <functional>

template<typename T>
struct callback_in_t;

template<typename Ret, typename ... Params>
struct callback_in_t<Ret(Params...)>
{
  template<typename ... Args>
  static Ret callback(Args... args)
  {
    return func(args ...);
  }
  static std::function<Ret(Params...)> func;
};

template<typename Ret, typename ... Params>
std::function<Ret(Params...)> callback_in_t<Ret(Params...)>::func;

template<typename T>
struct callback_out_t;

template<typename Ret, typename ... Params>
struct callback_out_t<Ret(Params...)>
{
  template<typename ... Args>
  static Ret callback(Args... args)
  {
    return func(args ...);
  }
  static std::function<Ret(Params...)> func;
};

template<typename Ret, typename ... Params>
std::function<Ret(Params...)> callback_out_t<Ret(Params...)>::func;

template<typename T>
struct hotplug_detach_callback_t;

template<typename Ret, typename ... Params>
struct hotplug_detach_callback_t<Ret(Params...)>
{
  template<typename ... Args>
  static Ret callback(Args... args)
  {
    return func(args ...);
  }
  static std::function<Ret(Params...)> func;
};

template<typename Ret, typename ... Params>
std::function<Ret(Params...)> hotplug_detach_callback_t<Ret(Params...)>::func;

template<typename T>
struct hotplug_attach_callback_t;

template<typename Ret, typename ... Params>
struct hotplug_attach_callback_t<Ret(Params...)>
{
  template<typename ... Args>
  static Ret callback(Args... args)
  {
    return func(args ...);
  }
  static std::function<Ret(Params...)> func;
};

template<typename Ret, typename ... Params>
std::function<Ret(Params...)> hotplug_attach_callback_t<Ret(Params...)>::func;

#endif  // UBLOX_DGNSS_NODE__CALLBACK_HPP_
