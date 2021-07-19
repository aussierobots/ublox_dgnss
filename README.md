# ublox-dgnss
This usb based driver is focused on UBLOX Generation 9 UBX messaging, for a rover for DGNSS. Its assumed that RTCM messages are being delivered externally. High precision data is available.

It will only work with later generation ublox devices. Testing and development was performed against a ZED-F9P connected via USB, under Ubuntu 20.04. The driver uses libusb api 1.0.

You may need to create a udev rule as follows:

/etc/udev/rules.d/99-ublox-gnss.rules
```
  #UBLOX ZED-F9P
  ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", MODE="0666", GROUP="plugdev"
```

This driver follows the UBX standards used for the ZED-F9P as documented in the [interface description](https://www.u-blox.com/sites/default/files/ZED-F9P_InterfaceDescription_%28UBX-18010854%29.pdf). 

It implements a subset of the specification related to achieving high precision output as a rover. U-center can be used to alter settings. Any configuration parameter changed by this driver, will be applied only in RAM. Upon a restart (hot, cold or warm) or after a hot plug usb attach event, the configuration stored in the driver will be sent to the device.

## Start commands

There are multiple ways to start the node. Its been built using composition. An executable is provider. An example follows which turns off NMEA output at startup.
```
ros2 run ublox_dgnss_node ublox_dgnss_node --ros-args -p CFG_USBOUTPROT_NMEA:=False
```
Parameters can be set at launch and some examples are provided to launch high precision pos ecef|llh 
```
ros2 launch ublox_dgnss ublox_rover_hpposecef.launch.py   
```
```
ros2 launch ublox_dgnss ublox_rover_hpposllh.launch.py 
```

## Services

Four services are provided to reset odo, cold start, warm start and hot start.
```
ros2 service call /ublox_dgnss/reset_odo ublox_ubx_interfaces/srv/ResetODO
```
```
ros2 service call /ublox_dgnss/cold_start ublox_ubx_interfaces/srv/ColdStart '{reset_type: 1}' 
```
```
ros2 service call /ublox_dgnss/warm_start ublox_ubx_interfaces/srv/WarmStart '{reset_type: 1}' 
```
```
ros2 service call /ublox_dgnss/hot_start ublox_ubx_interfaces/srv/HotStart '{reset_type: 1}' 
```

## Parameters

Parameter values can be set at any time or passed when the node starts  

```
ros2 param set /ublox_dgnss CFG_RATE_NAV 2
```
or the current value retrieved
```
ros2 param get /ublox_dgnss CFG_RATE_NAV
```
### UBX Parameters  

The following parameters may be set. Its a subset of the full UBX Gen 9 configuration parameter list.

Values will be as described in the integration manual (without scaling applied). Changing a CFG_MSGOUT_* type parameter to a value >0 will cause the corresponding ublox_ubx_msgs to be published.

  CFG_INFMSG_NMEA_USB
  CFG_INFMSG_UBX_USB
  CFG_MSGOUT_UBX_NAV_CLOCK_USB
  CFG_MSGOUT_UBX_NAV_COV_USB
  CFG_MSGOUT_UBX_NAV_DOP_USB
  CFG_MSGOUT_UBX_NAV_EOE_USB
  CFG_MSGOUT_UBX_NAV_HPPOSECEF_USB
  CFG_MSGOUT_UBX_NAV_HPPOSLLH_USB
  CFG_MSGOUT_UBX_NAV_ODO_USB
  CFG_MSGOUT_UBX_NAV_POSECEF_USB
  CFG_MSGOUT_UBX_NAV_POSLLH_USB
  CFG_MSGOUT_UBX_NAV_PVT_USB
  CFG_MSGOUT_UBX_NAV_RELPOSNED_USB
  CFG_MSGOUT_UBX_NAV_STATUS_USB
  CFG_MSGOUT_UBX_NAV_TIMEUTC_USB
  CFG_MSGOUT_UBX_NAV_VELECEF_USB
  CFG_MSGOUT_UBX_NAV_VELNED_USB
  CFG_NAVHPG_DGNSSMODE
  CFG_NAVSPG_DYNMODEL
  CFG_NAVSPG_FIXMODE
  CFG_NAVSPG_INIFIX3D
  CFG_NAVSPG_UTCSTANDARD
  CFG_ODO_COGLPGAIN
  CFG_ODO_COGMAXPOSACC
  CFG_ODO_COGMAXSPEED
  CFG_ODO_OUTLPCOG
  CFG_ODO_OUTLPVEL
  CFG_ODO_PROFILE
  CFG_ODO_USE_COG
  CFG_ODO_USE_ODO
  CFG_ODO_VALLPGAIN
  CFG_RATE_MEAS
  CFG_RATE_NAV
  CFG_RATE_TIMEREF
  CFG_TMODE_MODE
  CFG_TMODE_POS_TYPE
  CFG_UART1INPROT_NMEA
  CFG_UART1INPROT_RTCM3X
  CFG_UART1INPROT_UBX
  CFG_UART1OUTPROT_NMEA
  CFG_UART1OUTPROT_RTCM3X
  CFG_UART1OUTPROT_UBX
  CFG_USBINPROT_NMEA
  CFG_USBINPROT_RTCM3X
  CFG_USBINPROT_UBX
  CFG_USBOUTPROT_NMEA
  CFG_USBOUTPROT_RTCM3X
  CFG_USBOUTPROT_UBX

# ROS2 UBX NAV Messages

The following messages may be outputed. They included a `std_msgs/Header header` with the remainder of the fields matching the UBX output. Note that field names use different notation as the UBX field name notation is not compliant with the ROS IDL field names.
```
/ubx_nav_clock
/ubx_nav_cov
/ubx_nav_dop
/ubx_nav_eoe
/ubx_nav_hp_pos_ecef
/ubx_nav_hp_pos_llh
/ubx_nav_odo
/ubx_nav_pos_ecef
/ubx_nav_pos_llh
/ubx_nav_pvt
/ubx_nav_rel_pos_ned
/ubx_nav_status
/ubx_nav_time_utc
/ubx_nav_vel_ecef
/ubx_nav_vel_ned
```
