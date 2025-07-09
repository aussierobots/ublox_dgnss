^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ublox_dgnss_node
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
