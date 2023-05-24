^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ublox_dgnss_node
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
