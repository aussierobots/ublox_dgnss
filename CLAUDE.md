# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a ROS2 package for high-precision GNSS positioning using u-blox receivers: Generation 9 (ZED-F9P/F9R) and the ZED-X20P (all-band). The driver supports DGNSS rover configurations, moving base station setups, and RTCM correction data integration for centimeter-level positioning accuracy. Device family is selected via the `DEVICE_FAMILY` parameter (F9P/F9R/X20P).

## Build System & Common Commands

### Building
```bash
# Build entire workspace
colcon build

# ROS2 distro: prefer the latest installed released distro (Lyrical Luth, May 2026 LTS)
# over jazzy. Source ONE distro consistently - do not mix:
#   bash -c 'source /opt/ros/lyrical/setup.bash 2>/dev/null || source /opt/ros/jazzy/setup.bash; colcon build'
# Stale build cache: if CMake errors reference a DIFFERENT distro (e.g. /opt/ros/rolling),
# that package's build/ dir is stale. Fix: rm -rf build/<pkg> install/<pkg>, then rebuild.

# Build specific package
colcon build --packages-select ublox_dgnss_node

# Build with specific package override
colcon build --packages-select ublox_dgnss_node --allow-overriding ublox_dgnss_node
```

### Code Formatting
```bash
# Format code using uncrustify (config in uncrustify.cfg)
ament_uncrustify --reformat ublox_dgnss_node/src/

# Check formatting without modifying (no --check flag, just omit --reformat)
ament_uncrustify ublox_dgnss_node/src/ ublox_dgnss_node/include/
```

### Testing
```bash
# Full lint suite (cpplint, uncrustify, copyright, lint_cmake, xmllint). cppcheck shows
# as "skipped" when the tool isn't installed - that is not a failure.
colcon test --packages-select ublox_dgnss_node ublox_ubx_msgs
colcon test-result --all --verbose
```

### Running
```bash
# Basic launch with parameter override
ros2 run ublox_dgnss_node ublox_dgnss_node --ros-args -p CFG_USBOUTPROT_NMEA:=False

# Launch configurations
ros2 launch ublox_dgnss ublox_rover_hpposllh.launch.py
ros2 launch ublox_dgnss ublox_rover_hpposecef.launch.py
ros2 launch ublox_dgnss ublox_rover_hpposllh_satellite.launch.py

# NTRIP client for RTCM corrections
ros2 launch ublox_dgnss ntrip_client.launch.py host:=ntrip.data.gnss.ga.gov.au port:=443 mountpoint:=MBCH00AUS0
```

## Architecture Overview

### Component Structure
- **ublox_dgnss_node**: Core driver with USB communication, parameter management, and UBX protocol handling
- **ublox_nav_sat_fix_hp_node**: Converts UBX messages to standard ROS NavSatFix messages
- **ntrip_client_node**: Fetches RTCM correction data from NTRIP casters
- **ublox_ubx_msgs**: ROS2 message definitions for UBX protocol
- **ublox_ubx_interfaces**: ROS2 service definitions for device control

### Key Architectural Patterns

#### Parameter Management System (ENHANCED)
The system uses a sophisticated parameter state machine with thread-safe operations and unified ParameterManager architecture:

**Parameter States:**
- `PARAM_INITIAL`: Default/unknown values
- `PARAM_USER`: User-set values (launch file, runtime)  
- `PARAM_LOADED`: Retrieved from GPS device
- `PARAM_VALSET`: Sent to GPS device
- `PARAM_VALGET`: Reading from GPS device

**Parameter Value Sources:**
- `UNKNOWN`: Parameter exists but value unknown (std::nullopt)
- `DEVICE_ACTUAL`: Real value from u-blox device via CFG-VALGET
- `START_ARG`: From launch file/yaml/command args during initialization
- `RUNTIME_USER`: From ros2 param set during operation

**State Transitions:**
```
PARAM_INITIAL → PARAM_VALGET → PARAM_LOADED        (device fetch)
PARAM_LOADED → PARAM_VALGET → PARAM_LOADED         (hot plug re-fetch)
PARAM_USER → PARAM_VALSET → PARAM_LOADED           (user override)
PARAM_LOADED → PARAM_VALSET → PARAM_LOADED         (modify device value)
```

**Key Classes:**
- `ParameterManager` (`parameters.hpp/cpp`): Unified parameter handling with ParamState class and Builder pattern
- `ParamState` class: Modern parameter state structure with validation and optional values
- Uses `set_parameter_cache()` for initialization and `update_parameter_cache()` for state transitions
- **Critical Fix**: Parameter initialization timing issue resolved - constructor now properly retrieves parameter values

**3-Phase Parameter Initialization:**
1. **Phase 1**: `ublox_send_user_params_async()` - Send ALL user parameters to device FIRST (highest priority)
2. **Phase 2**: `ublox_declare_missing_params()` - Declare missing parameters as PARAM_INITIAL  
3. **Phase 3**: `ublox_fetch_device_params_async()` - Fetch device parameter values

#### USB Hot-Plug Architecture (ENHANCED)
**USB Driver States:** `DISCONNECTED` → `CONNECTING` → `CONNECTED` → `ERROR` (4 states total)

**Device Readiness States:** `UNREADY` → `READY` (parameter-aware state management)

**Enhanced Integration:** 
- ParameterManager coordinates with DeviceReadinessState for parameter synchronization
- Device readiness transitions to `READY` only after parameters are synchronized  
- Reconnection optimization: Only user parameters restored (not full re-initialization)
- Automatic parameter restoration via `restore_user_parameters_to_device()` on hotplug

#### ROS2 Component Architecture
Uses composition-based design with:
- Separate callback groups for USB processing and parameter management
- Mutex-protected thread-safe operations
- Timer-based parameter synchronization (100ms cycle)

### UBX Protocol Implementation

#### Configuration Management
- UBX config items defined in `ubx_cfg_item_map.hpp` 
- Real-time parameter changes via CFG-VALSET/CFG-VALGET commands
- 150+ supported UBX configuration parameters

#### Per-Family Config Generation (@only / @exclude)

- Device configs `ublox_dgnss/config/{f9p,f9r,x20p}_ubx_config.toml` are GENERATED from
  `ubx_cfg_item_map.hpp` - do not hand-edit. Regenerate after changing the map:
  `python3 ublox_dgnss/scripts/generate_toml_from_existing.py`
- Family scoping via comments before const defs: `// @only: F9P,F9R` (allowlist) or
  `// @exclude: X20P` (denylist). Prefer @only (allowlist) for new restrictions.
- Scope: the annotation applies to the following consts until a blank line OR a non-`@`
  comment resets it. GOTCHA: a comment that contains `@` does NOT reset the rule.
- Commented-out (`//`) consts are skipped by the generator - that is how to disable an
  item/port while preserving it. A message's port variants appear in a toml only if their
  const is defined (uncommented) in the map.

#### Message Processing
- Organized by UBX message class: NAV, RXM, ESF, MON, SEC
- Each message type has dedicated header files in `ubx/` subdirectories
- Direct USB communication using libusb 1.0 API

#### Adding a New UBX Message (e.g., RXM-SFRBX)
Files to create/modify:
1. `ubx/rxm/ubx_rxm_<name>.hpp` - Payload parser (namespace `ubx::rxm::<name>`, extends `UBXPayload`)
2. `ublox_ubx_msgs/msg/UBXRxm<Name>.msg` - ROS2 message definition
3. `ubx/ubx_msg.hpp` - Add `UBX_RXM_<NAME>` message ID constant
4. `ubx/ubx_rxm.hpp` - Add include, typedef, member, constructor init, accessor
5. `ubx_cfg_item_map.hpp` - Add `CFG_MSGOUT_UBX_RXM_<NAME>_USB` constant + map entry
6. `ublox_ubx_msgs/CMakeLists.txt` - Add .msg file to `rosidl_generate_interfaces`
7. `ublox_dgnss/config/{f9p,f9r,x20p}_ubx_config.toml` - Add MSGOUT entry
8. `ublox_dgnss_node.cpp` - Add include, publisher member, publisher creation, out_frame case, in_frame case, pub function

**Output vs Input direction:**

- Output (device→ROS, e.g. NAV, RXM-COR/RAWX/SFRBX): the 8-file list above (publisher +
  in_frame case + pub function + `CFG_MSGOUT_*` key).
- Input (ROS→device, e.g. RXM-PMP/QZSSL6/SPARTN-KEY, ESF-MEAS): subscriber on a
  `*_to_device` topic; callback does `comms->payload_poll()->load_from_msg(msg)` then
  `comms->send_async()`. NO publisher / `CFG_MSGOUT`. Gate creation by device family +
  an enable param via `should_create_input_sub()`.
- WARNING (input): do NOT use `payload()->poll_async()`. `poll_async()` caches a frame
  built from `payload_poll_`, so loading `payload()` would send an empty frame. Use
  `payload_poll()->load_from_msg()` + `send_async()`.

## Code Conventions & Important Notes

### Critical Code Change Protocol
**NEVER remove existing code.** Always follow this exact pattern when making changes:

#### 1. Comment Out Original Code
```cpp
/* TODO: Review - Original implementation commented out
void original_function() {
    // original implementation here
    return original_value;
}
*/
```

#### 2. Add New Implementation with TODO Marker
```cpp
// TODO: Review - New implementation with [brief description of change]
void original_function() {
    // new implementation here
    return new_value;
}
```

#### 3. For Parameter System Changes
```cpp
// TODO: Review - Changed to use set_parameter_cache() instead of update_parameter_cache()
parameter_manager_->set_parameter_cache(param_name, value, PARAM_USER);

/* TODO: Review - Original parameter cache update
parameter_manager_->update_parameter_cache(param_name, value, status);
*/
```

#### 4. For Large Function Rewrites
```cpp
void complex_function() {
    // TODO: Review - New 3-phase implementation
    phase_1();
    phase_2(); 
    phase_3();
    
    /* TODO: Review - Original code preserved below
    original_complex_logic();
    more_original_code();
    */
}
```

**Why This Matters:**
- Allows easy rollback if changes cause issues
- Enables comparison between old and new approaches
- Preserves institutional knowledge about why code was written certain ways
- Required for all changes in this codebase

### Parameter System Rules
- Use `set_parameter_cache()` for parameter initialization (PARAM_INITIAL or PARAM_USER only)
- Use `update_parameter_cache()` for state transitions with validation
- All parameter operations must follow the state machine rules
- Parameter processing happens in separate thread with `MutuallyExclusive` callback group

### USB Communication
- All USB operations use libusb 1.0 API directly
- Hot-plug detection and automatic reconnection
- Transfer queues for async USB operations
- Error handling with automatic retry mechanisms

### Multi-Device Support
- Support for multiple simultaneous u-blox devices
- Device identification via serial strings (`DEVICE_SERIAL_STRING` parameter)
- Frame ID customization via `FRAME_ID` parameter
- `DEVICE_FAMILY` param (F9P/F9R/X20P, default F9P) sets `device_family_` in the
  constructor. F9P and F9R share USB product ID 0x01a9, so family is NOT auto-detected
  from hardware - family-gated features (subscriptions, publishers, services, config
  items) key off this declared param. Use allowlist checks (`== F9P || == F9R`).

### Commit Messages

- NO Claude / AI attribution - do not add "Generated with Claude Code", "Co-Authored-By:
  Claude", or any similar trailer.
- Keep messages short and concise.
- Group related changes into a single logical commit rather than many small ones.

## Launch File Parameters

### Common UBX Configuration Parameters
Key parameters for device configuration (see README.md for complete list):
- `CFG_RATE_MEAS`: Measurement rate in Hz
- `CFG_MSGOUT_UBX_NAV_HPPOSLLH_USB`: Enable high-precision position messages
- `CFG_USBOUTPROT_NMEA`: Enable/disable NMEA output
- Message output rates: `CFG_MSGOUT_UBX_*_USB` parameters

### Device Identification
- `DEVICE_SERIAL_STRING`: Target specific device by serial number
- `FRAME_ID`: Custom frame ID for ROS messages

## Additional Documentation

### Detailed Architecture Documentation
The repository contains two comprehensive documentation files that provide deep technical details:

#### `usb_device_state.md`
- **Purpose**: Comprehensive USB hotplug and device state management documentation
- **Content**: Complete USB state machine, connection lifecycle, timer operations, and integration details
- **Coverage**: USB driver states, hotplug callbacks, error handling, thread safety, and performance considerations
- **Use Cases**: Understanding USB hotplug behavior, debugging connection issues, implementing USB features

#### `parameter_lifecycle.md` 
- **Purpose**: Deep dive into parameter management system architecture  
- **Content**: Complete parameter state machine, threading model, device synchronization, and integration patterns
- **Coverage**: Parameter states, state transitions, UBX config management, hotplug support, and system flow
- **Use Cases**: Understanding parameter flow, debugging parameter issues, implementing parameter features

**When to Reference These Files:**
- **USB Development**: `usb_device_state.md` for hotplug implementation, state management, and device communication
- **Parameter Development**: `parameter_lifecycle.md` for parameter system architecture, state machines, and integration
- **System Architecture**: Both files provide comprehensive view of the complete system design
- **Debugging**: Detailed troubleshooting information for both USB and parameter subsystems

**Key Architecture Concepts Covered:**
- USB state machine with 7 distinct states and validated transitions
- Parameter state machine with 6 states and enforced transition validation  
- Thread-safe operations with mutex protection and callback groups
- Integration between USB hotplug events and parameter restoration
- Direct UBX config storage with `set_parameter_cache()` vs `update_parameter_cache()` separation

## Debugging & Diagnostics

### Parameter Verification
```bash
# Check current parameter values
ros2 param get /ublox_dgnss CFG_RATE_NAV

# Monitor parameter changes
ros2 param set /ublox_dgnss CFG_RATE_NAV 2
```

### RTCM Data Verification
```bash
# Verify RTCM reception and usage
ros2 topic echo /ubx_rxm_rtcm  # msg_used: 2 indicates successful usage
```

### Precision Monitoring
```bash
# Monitor high-precision positioning
ros2 topic echo /ubx_nav_hp_pos_llh  # h_acc/v_acc in mm (scale by 0.1)
ros2 topic echo /ubx_nav_status     # diff_soln: true indicates RTCM corrections active
```

### USB State Monitoring
Monitor USB connection state and parameter synchronization through log messages indicating state transitions and parameter restoration events.

## Coordinate Transformations

High-precision coordinate calculation:
```cpp
double lat = msg->lat * 1e-7 + msg->lat_hp * 1e-9;
double lon = msg->lon * 1e-7 + msg->lon_hp * 1e-9;  
double alt = msg->height * 1e-3 + msg->height_hp * 1e-4;
```

Accuracy values: `h_acc` and `v_acc` in millimeters, scale by 0.1 for actual precision.