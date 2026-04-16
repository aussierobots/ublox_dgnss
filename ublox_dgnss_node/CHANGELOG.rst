^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ublox_dgnss_node
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.4 (2026-04-16)
------------------
* Merge pull request `#62 <https://github.com/aussierobots/ublox_dgnss/issues/62>`_ from gakutasu/fix/init-attach
  Fix USB hotplug attach skip on startup by initializing `attached\_`
* fix init
* Contributors: Nick Hortovanyi, gakutasu

0.7.3 (2026-03-29)
------------------
* Added UBX_RXM_SFRBX and fixed bug for param set being sent to usb device upon change
* fix uncrustify formatting
* fix: handle concurrent access to parameters
  and remove deadlock risk
* Merge pull request `#60 <https://github.com/aussierobots/ublox_dgnss/issues/60>`_ from SayedMuhamad/fix/rtcm-callback-resize-to-reserve
  fix(rtcm_callback): replace resize() with reserve() to prevent leading zero bytes
* fix(rtcm_callback): replace resize() with reserve() to prevent leading zero bytes
  In rtcm_callback(), the output buffer was built using:
  data_out.resize(N);          // pre-fills N zero bytes at indices [0..N-1]
  for (auto b : msg.message)
  data_out.push_back(b);     // appends real bytes at indices [N..2N-1]
  This produced a 2N-byte buffer starting with N zero bytes, which
  corrupts SPARTN/RTCM3 framing. The u-blox F9P USB parser scans for
  the 0x73 (SPARTN) or 0xD3 (RTCM3) preamble at byte 0; with leading
  zeros it never found a valid frame start and silently discarded every
  correction message.
  Fix: replace resize() with reserve(). This pre-allocates capacity
  without filling, so push_back() places the real data at indices
  [0..N-1] and the resulting buffer is exactly N bytes with the
  preamble byte at index 0.
  Fixes: corrections delivered via /ntrip_client/rtcm having zero
  effect on receiver positioning accuracy.
* Merge pull request `#59 <https://github.com/aussierobots/ublox_dgnss/issues/59>`_ from wentasah/fix
  Fix missing include with GCC 14
* Fix missing include with GCC 14
  Without this, the compiler complains with:
  include/ublox_dgnss_node/ubx/utils.hpp:188:8: error: 'reverse_copy' is not a member of 'std'
* Contributors: Michal Sojka, Nick Hortovanyi, Sayed Muhammad

0.7.2 (2026-03-19)
------------------
* uncrustify fixes
* Add flags for LPP and has correction used
* Add lpp_corr_used and has_corr_used to flags
* Contributors: Nick Hortovanyi

0.7.1 (2026-03-18)
------------------
* fixed uncrustitfy comment space
* reverted fix for warning as not supported jazzy & humble
* fix c++ header include for crustify
* Fixed iterator bug and case labels
* Fix deprecation warning
* Merge remote-tracking branch 'refs/remotes/origin/main'
* Merge pull request `#57 <https://github.com/aussierobots/ublox_dgnss/issues/57>`_ from stan-guer/fix/nav-status-parsing
  fix nav-status decoding
* fix nav-status decoding
  According to u-blox 20 HPG 2.00 , the status bits in the fixStat and
  flag2 fields in the ubx-nav-status message are not always consecutive,
  there are padding bits in there that are not accounted for in this
  decoding logic. Fix that.
  An obvious outcome is that the ubx-nav-status now properly shows
  RTK float or int status, the same status value that is in the (correctly
  decoded) ubx-nav-pvt message.
* sync write lock scope changed
* Merge pull request `#56 <https://github.com/aussierobots/ublox_dgnss/issues/56>`_ from BravoBravoIX/fix/usb-timeout-handling
* Improve USB write stability: 250ms timeout and mutex protection
  - Set timeout_ms\_ to 250ms instead of 0 (blocking) for hang safety
  - Add write_mutex\_ to serialize write_buffer() calls
* Fix USB timeout crash in rtcm_callback
  Add exception handling for USB write failures in RTCM callback to prevent
  node crashes when USB bulk transfers timeout.
  Changes:
  - Wrap write_buffer() call in try/catch in rtcm_callback()
  - Change timeout_ms from 45ms to 0 (blocking) to match v0.6.1 behavior
  The 45ms timeout was too tight for slower USB 1.1 devices (F9P) causing
  intermittent LIBUSB_ERROR_TIMEOUT exceptions that crashed the node.
  Setting timeout to 0 restores blocking behavior consistent with
  write_buffer_async() and the previous stable release.
* Fixed X20P parameters
* build: Remove libtoml11-dev from package dependencies
* build: Remove toml11 dependency from CMakeLists
* feat: Replace toml11 with embedded SimpleTomlParser
  Implement custom TOML parser in anonymous namespace to handle UBX
  config files without external dependencies. Supports sections, string
  values, arrays, and numeric values.
* feat: Implement three-priority UBX config loading system
  - Add UBX_CONFIG_FILE parameter for custom TOML configs
  - Load device family default TOML based on DEVICE_FAMILY parameter
  - Fallback to F9P default, then full static map if loading fails
  - Add check_for_ubx_config_file_param() following existing pattern
* feat: Add device family annotations to UBX parameter definitions
  - Add @exclude and @only annotations for device-specific parameters
  - Mark UART2 parameters as X20P-only (@only: X20P)
  - Mark ESF sensor fusion parameters as F9R-only (@only: F9R)
  - Mark L5/L6 signal parameters as X20P-only (@exclude: F9P,F9R)
  - Fix typo: CFG_ODO_VALLPGAIN -> CFG_ODO_VELLPGAIN
* refactor: Move ubx_cfg_item_map_t type alias to ubx::cfg namespace
  - Relocate type alias from ublox_dgnss to ubx::cfg namespace for consistency
  - Add type alias reference in parameters.hpp for convenience
* feat: Add TOML-based device family configuration filtering
  - Implement UbxConfigLoader for filtering parameters by device family
  - Add Python script to generate TOML files with @exclude/@only annotations
  - Generate F9P, F9R, X20P config files from existing parameter map
  - Enable three-priority loading: UBX_CONFIG_FILE > DEVICE_FAMILY > F9P default
* build: Add toml11 and ament_index_cpp dependencies
  - Add libtoml11-dev dependency for TOML config parsing
  - Add ament_index_cpp for runtime package resource lookup
  - Install TOML config files to share directory
* New feature for param val get state reporting
* fix(params): Set PARAM_LOADED when device response arrives, not PARAM_VALGET
  CFG-VALGET responses should transition directly to PARAM_LOADED rather than
  temporarily using PARAM_VALGET status. Eliminates invalid semantic state and
  callback dependency. Foundation for reliable async completion tracking.
* change to cfg_val_get_poll_async_all_layers()
* Contributors: BravoBravoIX, Nick Hortovanyi, Stan Guerassimov

0.7.0 (2025-10-27)
------------------
* Merge pull request `#49 <https://github.com/aussierobots/ublox_dgnss/issues/49>`_ from aussierobots/X20P
  Add X20P device family support with USB architecture adaptation
* - Add clear error message directing users to main interface (0x01ab)
  - Update documentation with interface limitations and GitHub Issue `#48 <https://github.com/aussierobots/ublox_dgnss/issues/48>`_
  - Comment out experimental header stripping code
  - Add X20P configuration parameters to launch files
* Documentation updates to match code
* added new device family for X20P
* Added support for X20P and changed USB connection to support USB UARTs
* added new device family to add X20P
* Contributors: Nick Hortovanyi

0.6.0 (2025-07-28)
------------------
* Fix for handle_usb_event starting before usb connection
* Major refactor: Parameter state management and USB hotplug system
  This is a comprehensive overhaul of the parameter management and USB hotplug systems:
  **Parameter Management Architecture:**
  - Added new ParameterManager class with complete parameter lifecycle management
  - Implemented ParamValueSource tracking (UNKNOWN/DEVICE_ACTUAL/START_ARG/RUNTIME_USER)
  - Added parameter state machine (PARAM_INITIAL/USER/LOADED/VALSET/VALGET/ACKNAK)
  - Enhanced thread safety with mutex-protected parameter cache operations
  - Fixed parameter initialization timing with proper 3-phase approach
  - Added smart parameter callbacks to prevent CFG-VALGET responses becoming PARAM_USER
  **USB Hotplug System:**
  - Fixed USB transfer queue cleanup during device disconnect
  - Added cleanup_all_transfers() to properly cancel and clear pending transfers
  - Prevents "too many transfers" warnings and potential memory leaks from stale transfers
  - Enhanced hotplug parameter handling: reset device params on disconnect, restore user params on reconnect
  - Improved USB driver state management with proper state transitions
  **Thread Safety & Memory Management:**
  - Added comprehensive mutex protection for parameter cache and transfer queue operations
  - Fixed potential memory leaks in USB transfer cleanup during hotplug events
  - Enhanced callback group separation for USB events vs parameter processing
  **New Files:**
  - parameters.hpp/cpp: Complete parameter management system
  - ubx_cfg_item_map.hpp: UBX configuration item mapping
  - parameter_lifecycle.md/usb_device_state.md: Architecture documentation
  **Statistics:** 7 files changed, 1436 insertions(+), 993 deletions(-), 4 new files
  This resolves parameter synchronization issues during hotplug events and ensures proper parameter state transitions between device and user-initiated changes.
* Added nullptr check on transfer->user_data
* buildfarm crustify issue
* Contributors: Nick Hortovanyi

0.5.8 (2025-07-09)
------------------
* formatting changes
* added debug logging and fixed USB device detection at startup
* refactored dev_valid and added devh_valid
* Contributors: Nick Hortovanyi

0.5.7 (2025-05-29)
------------------
* fixed whitespace comment
* Contributors: Nick Hortovanyi

0.5.6 (2025-05-29)
------------------
* changed ament_target_dependencies to target_link_libraries
* Merge pull request `#45 <https://github.com/aussierobots/ublox_dgnss/issues/45>`_ from ARK3r/main
  fix: show correct port_id
* now it shows reasonable port ids
* Merge pull request `#43 <https://github.com/aussierobots/ublox_dgnss/issues/43>`_ from bvsam/ubx-nav-svin-fix
  Fixing UBX-NAV-SVIN reading
* Merge pull request `#40 <https://github.com/aussierobots/ublox_dgnss/issues/40>`_ from mak22223/main
  Fixed U4, X4, I4 values interpretation
* Fixed U4, X4, I4 values interpretation
* Fixing UBX-NAV-SVIN reading
* Contributors: ARK3r, Benjamin Sam, Markin Maxim, Nick Hortovanyi

0.5.5 (2025-02-12)
------------------
* Merge pull request `#33 <https://github.com/aussierobots/ublox_dgnss/issues/33>`_ from bvsam/base-rtcm-support
  Base rtcm support
* fixed style
* Merge branch 'main' into base-rtcm-support
* Merge pull request `#32 <https://github.com/aussierobots/ublox_dgnss/issues/32>`_ from bvsam/mb+r_fix
  Updating mb+r_base launch file and adding missing parameters
* Fixing formatting to allow for tests to pass
* Fixing formatting to enable tests to pass
* Fixing errors in code to ensure successful builds
* Adding more CONFIG_TMODE parameters
* Updating mb+r_base launch file and adding missing parameters
* Adding initial code for fixed base use case
* uncrustify formatting fixes
* Merge branch 'main' of github.com:aussierobots/ublox_dgnss
* uncrustify formatting changes
* Merge pull request `#30 <https://github.com/aussierobots/ublox_dgnss/issues/30>`_ from bvsam/base-station-rtcm
  Add base station rtcm message publishing support
* Updating rtcm publisher QOS for subscription compatibility
* Minor: fixing error in code
* Changing rtcm publisher topic name
* Adding payload logging for rtcm publishing
* Making subscription topics absolute for backwards compatbility
* Updating subscription topic names to be non-absolute
* Fixing errors in code
* Adding basic rtcm receival and publishing functionality
* Added Subcription Options Qos Override on subscriptions
* Add Publisher Option with QOS Overriding for all publishers
* uncrustify formatting changes
* parameter pub for ubx_mon_ver ubx_sec_uniqid
* ubx_sec_sig msg ver 2 changes
* added CFG_SIGNAL for GPS, SBAS, GAL, BDS, QZSS and GLO
* updated cfg items
* Contributors: Benjamin Sam, Nick Hortovanyi, Xiran Zhou, ryan

0.5.4 (2024-10-16)
------------------
* fixed uncrustify formatting errros
* Added copyright and fixed formatting
* Merge pull request `#27 <https://github.com/aussierobots/ublox_dgnss/issues/27>`_ from aussierobots/spartn-dev
  initial spartn changes
* Merge pull request `#25 <https://github.com/aussierobots/ublox_dgnss/issues/25>`_ from ARK3r/spartn-dev
* fix iteration variable override
* add UBX-MON-COMMS
* move UBX-MON-VER to mon folder
* Added UBX Rxm Cor|Spartn|SpartnKey
* Merge pull request `#24 <https://github.com/aussierobots/ublox_dgnss/issues/24>`_ from ARK3r/spartn-key-f9p-d9s
  Spartn configuration / monitoring support
* Merge pull request `#23 <https://github.com/aussierobots/ublox_dgnss/issues/23>`_ from icos-pit/main
  added printing serial_str\_ & serial_num_string to error of not finding device with specificed serial string.
* added uart2 and spartn source cfg items
* add serial_str\_ & serial_num_string to error stream  of finding USB device with specified serial string
* Contributors: ARK3r, Nick Hortovanyi, iman01

0.5.3 (2024-03-22)
------------------
* Jamming and interference monitor configuration
* Contributors: Nick Hortovanyi

0.5.2 (2023-11-05)
------------------
* removed saving of read serial str in connection
* uncrustify format issue
* fixed error messages
* added rc logic and throws
* Contributors: Nick Hortovanyi

0.5.1 (2023-10-13)
------------------
* Merge branch 'aussierobots:main' into main
* Contributors: Geoff Sokoll

0.5.0 (2023-10-13)
------------------
* code formatting issues
* fixed line length
* Merge branch 'main' of github.com:aussierobots/ublox_dgnss
* Merge pull request `#12 <https://github.com/aussierobots/ublox_dgnss/issues/12>`_ from gsokoll/main
  Add multiple device support, and moving base+rover example
* Added new messages for satellite data & security
* increase buffer size
* Add multiple device support, and moving base+rover example
* Contributors: Geoff Sokoll, Nick Hortovanyi

0.4.4 (2023-05-24)
------------------
* cmake uncrustify changes
* Contributors: Nick Hortovanyi

0.4.3 (2023-05-24)
------------------
* added ament_cmake_uncrustify
* Contributors: Nick Hortovanyi

0.4.2 (2023-05-24)
------------------

0.4.1 (2023-05-24)
------------------
* updates for release
* Merge pull request `#10 <https://github.com/aussierobots/ublox_dgnss/issues/10>`_ from aussierobots/F9R-and-RTCM
  F9 r and rtcm
* changed verbosity of logging
* Merge pull request `#9 <https://github.com/aussierobots/ublox_dgnss/issues/9>`_ from gsokoll/F9R-and-RTCM
  Composable node version of navsatfix
* Composable node version of navsatfix.  Added example launch file.
* Merge pull request `#8 <https://github.com/aussierobots/ublox_dgnss/issues/8>`_ from gsokoll/F9R-and-RTCM
  Change to rtcm_msgs instead of mavros_msgs.
* Change to rtcm_msgs instead of mavros_msgs.
  This is in keeping with a similar PR for the LORD MicrosStrain NTRIP client.
  https://github.com/LORD-MicroStrain/ntrip_client/pull/34
  where it is commented that
  "rtcm_msgs is a smaller dependency with support for both ROS and ROS 2
  now, and preferred by some downstream dependencies like ublox."
  Format of the messages are strcturally identical, with the only difference
  being the name of the payload ("data" in mavros vs "message" in rtcm_msgs)
* Merge pull request `#6 <https://github.com/aussierobots/ublox_dgnss/issues/6>`_ from gsokoll/gsokoll-patch-1
  Add CFG_SFIMU_AUTO_MNTALG_ENA
* if usb detached warn not sending to device
* added guards not send to device if usb devh null
* usb connection changes
* added return for esf_meas warnings to not send
* added guards and warnings for ubx_esf_meas inbound
* Add CFG_SFIMU_AUTO_MNTALG_ENA
* fix to pub full ubx_esf_meas
* changed some msgs from INFO to DEBUG
* renamed ubx_esf_meas_sub topic
* changed calibTtag from an array to single value
* uncrustify changes
* uncrustify format changes
* fixed wrong esf msg
* added rtcm msg processing
* added ubx_esf_meas full poll payload to device
* added ubx_esf_meas output publishing
* fixed spelling mistake
* notation and spelling fixes
* added initial ubx esf status code
* renamed F9P usb variable to F9
* fixed CFG_SFDO_LATENCY naming
* Added CFG_MSGOUT_UBX_ESF* items
* Added CFG_SFODO\_* items
* added new DYN_MODEL SFIMU_IMU_MNTALG\_* cfg items
* changed USb and UBX event times from 10ns to 10ms
* Merge pull request `#4 <https://github.com/aussierobots/ublox_dgnss/issues/4>`_ from gsokoll/patch-1
  Add UBX_ESF message class and id's
* Add UBX_ESF message class and id's
* added ubx_rxm_out_frame for poll frame out
* fixed version output
* added ubx-rxm-rtcm publishing
* Contributors: Geoff Sokoll, Nick Hortovanyi

* Merge pull request `#10 <https://github.com/aussierobots/ublox_dgnss/issues/10>`_ from aussierobots/F9R-and-RTCM
  F9 r and rtcm
* changed verbosity of logging
* Merge pull request `#9 <https://github.com/aussierobots/ublox_dgnss/issues/9>`_ from gsokoll/F9R-and-RTCM
  Composable node version of navsatfix
* Composable node version of navsatfix.  Added example launch file.
* Merge pull request `#8 <https://github.com/aussierobots/ublox_dgnss/issues/8>`_ from gsokoll/F9R-and-RTCM
  Change to rtcm_msgs instead of mavros_msgs.
* Change to rtcm_msgs instead of mavros_msgs.
  This is in keeping with a similar PR for the LORD MicrosStrain NTRIP client.
  https://github.com/LORD-MicroStrain/ntrip_client/pull/34
  where it is commented that
  "rtcm_msgs is a smaller dependency with support for both ROS and ROS 2
  now, and preferred by some downstream dependencies like ublox."
  Format of the messages are strcturally identical, with the only difference
  being the name of the payload ("data" in mavros vs "message" in rtcm_msgs)
* Merge pull request `#6 <https://github.com/aussierobots/ublox_dgnss/issues/6>`_ from gsokoll/gsokoll-patch-1
  Add CFG_SFIMU_AUTO_MNTALG_ENA
* if usb detached warn not sending to device
* added guards not send to device if usb devh null
* usb connection changes
* added return for esf_meas warnings to not send
* added guards and warnings for ubx_esf_meas inbound
* Add CFG_SFIMU_AUTO_MNTALG_ENA
* fix to pub full ubx_esf_meas
* changed some msgs from INFO to DEBUG
* renamed ubx_esf_meas_sub topic
* changed calibTtag from an array to single value
* uncrustify changes
* uncrustify format changes
* fixed wrong esf msg
* added rtcm msg processing
* added ubx_esf_meas full poll payload to device
* added ubx_esf_meas output publishing
* fixed spelling mistake
* notation and spelling fixes
* added initial ubx esf status code
* renamed F9P usb variable to F9
* fixed CFG_SFDO_LATENCY naming
* Added CFG_MSGOUT_UBX_ESF* items
* Added CFG_SFODO\_* items
* added new DYN_MODEL SFIMU_IMU_MNTALG\_* cfg items
* changed USb and UBX event times from 10ns to 10ms
* Merge pull request `#4 <https://github.com/aussierobots/ublox_dgnss/issues/4>`_ from gsokoll/patch-1
  Add UBX_ESF message class and id's
* Add UBX_ESF message class and id's
* added ubx_rxm_out_frame for poll frame out
* fixed version output
* added ubx-rxm-rtcm publishing
* Contributors: Geoff Sokoll, Nick Hortovanyi

0.3.5 (2022-05-24)
------------------
* uncrustify changes
* reverted uncrustify to ros ament default
* fixed title underline
* Contributors: Nick Hortovanyi

0.3.4 (2022-05-24)
------------------
* uncrustify
* Contributors: Nick Hortovanyi

0.3.3 (2022-05-24)
------------------
* added <build_depend>pkg-configi</build_depend>
* Contributors: Nick Hortovanyi

0.3.2 (2022-05-13)
------------------
* updated CMAKE_CXX_STANDARD 17
* Contributors: Nick Hortovanyi

0.3.1 (2022-05-13 12:43)
------------------------
* uncrustify format changes
* Contributors: Nick Hortovanyi

0.3.0 (2022-05-13 10:26)
------------------------
* changes for humble
* uncrustify
* uncrustify
* removed author element
* altered transfer_queue erase algo
* removed whitespace
* build test changes
* added ubx_nav_resetodo poll
* updated define statements
* Contributors: Nick Hortovanyi

0.2.3 (2021-07-25)
------------------
* changed build dependencies
* Contributors: Nick Hortovanyi

0.2.2 (2021-07-22 07:14)
------------------------
* 0.2.2 build farm dependency changes
* removed unused find_packages
* removed unused messages
* Contributors: Nick Hortovanyi

0.2.1 (2021-07-22 05:24)
------------------------
* updated version 0.2.1
* added check for libusb
* updated dependencies
* Contributors: Nick Hortovanyi

0.2.0 (2021-07-20)
------------------
* added ubx_nav_cov message
* updated version number and email
* Fixed license to Apache License, Version 2.0
* Contributors: Nick Hortovanyi

0.1.0 (2021-07-11)
------------------
* removed install for launch dir
* Initial release
* Contributors: Nick Hortovanyi
