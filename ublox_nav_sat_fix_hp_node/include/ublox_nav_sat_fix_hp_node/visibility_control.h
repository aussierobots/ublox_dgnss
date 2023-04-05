// Copyright 2023 CMP Engineers Pty Ltd
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

#ifndef UBLOX_NAV_SAT_FIX_HP_NODE__VISIBILITY_CONTROL_H_
#define UBLOX_NAV_SAT_FIX_HP_NODE__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C" {
#endif

// This logic was borrowed (then namespaced) from the existing instance under
// ublox_dgnss_node/include

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define UBLOX_NAV_SAT_FIX_HP_NODE_EXPORT __attribute__((dllexport))
#define UBLOX_NAV_SAT_FIX_HP_NODE_IMPORT __attribute__((dllimport))
#else
#define UBLOX_NAV_SAT_FIX_HP_NODE_EXPORT __declspec(dllexport)
#define UBLOX_NAV_SAT_FIX_HP_NODE_IMPORT __declspec(dllimport)
#endif
#ifdef UBLOX_NAV_SAT_FIX_HP_NODE_BUILDING_DLL
#define UBLOX_NAV_SAT_FIX_HP_NODE_PUBLIC UBLOX_NAV_SAT_FIX_HP_NODE_EXPORT
#else
#define UBLOX_NAV_SAT_FIX_HP_NODE_PUBLIC UBLOX_NAV_SAT_FIX_HP_NODE_IMPORT
#endif
#define UBLOX_NAV_SAT_FIX_HP_NODE_PUBLIC_TYPE UBLOX_NAV_SAT_FIX_HP_NODE_PUBLIC
#define UBLOX_NAV_SAT_FIX_HP_NODE_LOCAL
#else
#define UBLOX_NAV_SAT_FIX_HP_NODE_EXPORT __attribute__((visibility("default")))
#define UBLOX_NAV_SAT_FIX_HP_NODE_IMPORT
#if __GNUC__ >= 4
#define UBLOX_NAV_SAT_FIX_HP_NODE_PUBLIC __attribute__((visibility("default")))
#define UBLOX_NAV_SAT_FIX_HP_NODE_LOCAL __attribute__((visibility("hidden")))
#else
#define UBLOX_NAV_SAT_FIX_HP_NODE_PUBLIC
#define UBLOX_NAV_SAT_FIX_HP_NODE_LOCAL
#endif
#define UBLOX_NAV_SAT_FIX_HP_NODE_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // UBLOX_NAV_SAT_FIX_HP_NODE__VISIBILITY_CONTROL_H_
