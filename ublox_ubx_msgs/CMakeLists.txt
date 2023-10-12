cmake_minimum_required(VERSION 3.5)
project(ublox_ubx_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(ublox_ubx_msgs
  "msg/CarrSoln.msg"
  "msg/ESFMeasDataItem.msg"
  "msg/ESFSensorStatus.msg"
  "msg/GpsFix.msg"
  "msg/MapMatching.msg"
  "msg/MeasxData.msg"
  "msg/OrbAlmInfo.msg"
  "msg/OrbEphInfo.msg"
  "msg/OrbSVFlag.msg"
  "msg/OrbSVInfo.msg"
  "msg/OtherOrbInfo.msg"
  "msg/PSMPVT.msg"
  "msg/PSMStatus.msg"
  "msg/RawxData.msg"
  "msg/RecStat.msg"
  "msg/SatFlags.msg"
  "msg/SatInfo.msg"
  "msg/SBASService.msg"
  "msg/SBASStatusFlags.msg"
  "msg/SBASSvData.msg"
  "msg/SigData.msg"
  "msg/SigFlags.msg"
  "msg/SigLogEvent.msg"
  "msg/SpoofDet.msg"
  "msg/TrkStat.msg"
  "msg/UBXEsfMeas.msg"
  "msg/UBXEsfStatus.msg"
  "msg/UBXNavEOE.msg"
  "msg/UBXNavClock.msg"
  "msg/UBXNavCov.msg"
  "msg/UBXNavDOP.msg"
  "msg/UBXNavHPPosECEF.msg"
  "msg/UBXNavHPPosLLH.msg"
  "msg/UBXNavOdo.msg"
  "msg/UBXNavOrb.msg"
  "msg/UBXNavPosECEF.msg"
  "msg/UBXNavPosLLH.msg"
  "msg/UBXNavPVT.msg"
  "msg/UBXNavRelPosNED.msg"
  "msg/UBXNavSat.msg"
  "msg/UBXNavSBAS.msg"
  "msg/UBXNavSig.msg"
  "msg/UBXNavStatus.msg"
  "msg/UBXNavTimeUTC.msg"
  "msg/UBXNavVelECEF.msg"
  "msg/UBXNavVelNED.msg"
  "msg/UBXRxmMeasx.msg"
  "msg/UBXRxmRawx.msg"
  "msg/UBXRxmRTCM.msg"
  "msg/UBXSecSig.msg"
  "msg/UBXSecSigLog.msg"
  "msg/UBXSecUniqid.msg"
  "msg/UtcStd.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
