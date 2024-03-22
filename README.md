# ublox-dgnss

This usb based driver is focused on UBLOX Generation 9 UBX messaging, for a DGNSS rover. High precision data is available. A moving base station configuration has been added.

RTCM messages can be delivered externally. Alternately the ntrip_client_node can be utilised to retrieve RTCM messages from a castor to publish `/ntrip_client/rtcm` messages which will be received by the ublox_dgnss_node.

It will only work with later generation ublox devices. Testing and development was performed against a ZED-F9P and a ZED-F9R connected via USB, under Ubuntu 22.04. The driver uses libusb api 1.0.

This release works with Rolling, Humble and Iron.

You may need to create a udev rule as follows:

/etc/udev/rules.d/99-ublox-gnss.rules
```
  #UBLOX ZED-F9
  ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", MODE="0666", GROUP="plugdev"
```

This driver follows the UBX standards used for the ZED-F9P/F9R as documented in the [F9P interface description](https://content.u-blox.com/sites/default/files/documents/u-blox-F9-HPG-1.32_InterfaceDescription_UBX-22008968.pdf) and [F9R interface description](https://content.u-blox.com/sites/default/files/documents/u-blox-F9-HPS-1.30_InterfaceDescription_UBX-22010984.pdf).

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
  CFG_MSGOUT_UBX_NAV_ORB_USB
  CFG_MSGOUT_UBX_NAV_SAT_USB
  CFG_MSGOUT_UBX_NAV_SIG_USB
  CFG_MSGOUT_UBX_NAV_POSECEF_USB
  CFG_MSGOUT_UBX_NAV_POSLLH_USB
  CFG_MSGOUT_UBX_NAV_PVT_USB
  CFG_MSGOUT_UBX_NAV_RELPOSNED_USB
  CFG_MSGOUT_UBX_NAV_STATUS_USB
  CFG_MSGOUT_UBX_NAV_TIMEUTC_USB
  CFG_MSGOUT_UBX_NAV_VELECEF_USB
  CFG_MSGOUT_UBX_NAV_VELNED_USB
  CFG_MSGOUT_UBX_RXM_RTCM_USB
  CFG_MSGOUT_UBX_RXM_MEASX_USB
  CFG_MSGOUT_UBX_RXM_RAWX_USB
  CFG_MSGOUT_UBX_ESF_MEAS_USB
  CFG_MSGOUT_UBX_ESF_STATUS_USB
  CFG_MSGOUT_UBX_SEC_SIG_USB
  CFG_MSGOUT_UBX_SEC_SIGLOG_USB
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

  CFG_SFIMU_AUTO_MNTALG_ENA
  CFG_SFIMU_IMU_MNTALG_YAW
  CFG_SFIMU_IMU_MNTALG_PITCH
  CFG_SFIMU_IMU_MNTALG_ROLL

  CFG_SFODO_COMBINE_TICKS
  CFG_SFODO_COUNT_MAX
  CFG_SFODO_DIS_AUTOCOUNTMAX
  CFG_SFODO_DIS_AUTODIRPINPOL
  CFG_SFODO_FACTOR
  CFG_SFODO_LATENCY
  CFG_SFODO_QUANT_ERROR

  CFG_ITFM_BBTHRESHOLD
  CFG_ITFM_CWTHRESHOLD
  CFG_ITFM_ENABLE
  CFG_ITFM_ANTSETTING
  CFG_ITFM_ENABLE_AUX

### Device Identification Parameters

aka Use with Multiple Devices

By default, the ublox_dgnss node will search for and connect to the first device which matches the ublox USB ID's (vendor ID of 0x1546 and product ID of 0x1546).  If multiple devices are connected simultaneously, the remaining devices will be ignored.  In this situation you have no control over which device is used, since the order in which they are found may depend on the order in which they were physically attached to the host.

If you have multiple ublox devices attached simultaneously and wish to connect to a specific device, you can specify a launch parameter "DEVICE_SERIAL_STRING".  The node will then search for and connect to the first device with this matching serial string.  The device serial string itself should be programmed into the ublox device beforehand using u-center software.

The frame ID used in ROS2 messages for that device can also be specified using the launch parameter "FRAME_ID".

Note: these parameters are not written to the device.  They only instruct the node on how to behave.

# ROS2 UBX NAV Messages

The following messages may be outputed. They included a `std_msgs/Header header` with the remainder of the fields matching the UBX output. Note that field names use different notation as the UBX field name notation is not compliant with the ROS IDL field names.
```
/ubx_esf_meas

/ubx_esf_status
/ubx_nav_clock
/ubx_nav_cov
/ubx_nav_dop
/ubx_nav_eoe
/ubx_nav_hp_pos_ecef
/ubx_nav_hp_pos_llh
/ubx_nav_odo
/ubx_nav_orb
/ubx_nav_sat
/ubx_nav_sig
/ubx_nav_pos_ecef
/ubx_nav_pos_llh
/ubx_nav_pvt
/ubx_nav_rel_pos_ned
/ubx_nav_status
/ubx_nav_time_utc
/ubx_nav_vel_ecef
/ubx_nav_vel_ned
/ubx_rxm_rtcm
/ubx_rxm_measx
/ubx_rxm_rawx
/ubx_sec_sig
/ubx_sec_sig_log

```

The following topics are subscribed to
/ubx_esf_meas_to_device
/ntrip_client/rtcm

# ROS2 NAVSATFIX Messages

In addition to the UBX NAV messages shown above, [NavSatFix messages](https://docs.ros2.org/latest/api/sensor_msgs/msg/NavSatFix.html) can also be published on topic /fix.

Note that this node relies on (and hence subscribes to) three UBX Nav messages, namely /ubx_nav_hp_pos_llh, /ubx_nav_cov, /ubx_nav_status.

If the ublox device is not configured to publish these three messages, the navsatfix message will likewise not be published.

```
/fix
```

# NTRIP Client - RTCM messages

A basic NTRIP client is provided, that focuses in binary RTCM data. Its purpose is to acquire the RTCM data from an internet based NTRIP Castor and publish [RTCM Messages](https://index.ros.org/p/rtcm_msgs/).

An example launch file is provided. You will need to set values appropriate for the NTRIP Castor intended to be used
```
ros2 launch ublox_dgnss ntrip_client.launch.py use_https:=true host:=ntrip.data.gnss.ga.gov.au port:=443 mountpoint:=MBCH00AUS0 username:=username password:=password
```
Note the launch file has been configured to also use environment variables `NTRIP_USERNAME` & `NTRIP_PASSWORD` such that you dont need credentials in the launch scripts.

# Moving Base and Rover

The ZED-F9P devices support the "moving base and rover" mode of operation, using two devices on a mobile vehicle to provide both position and orientation data.  Further information on this mode, including physical connections between the two devices and the host machine are included in the following ublox application note.

https://www.u-blox.com/sites/default/files/documents/ZED-F9P-MovingBase_AppNote_UBX-19009093.pdf

Note: the ZED-F9R device does not support this mode.

Example launch configuration files are included as "ublox_mb+r_base.launch.py" and "ublox_mb+r_rover.launch.py" for the base and rover devices respectively.  The base and rover are physically connected similar to that described in Section 2.2.1 "Wired connection between base and rover" in the ublox app note, with UART2 used for the wired connection between base and rover.  Both base and rover connect to the host machine via USB.  Device configuration is similar to that described in Section 2.3.2 "5 Hz navigation rate application" in the app note.

Both base and rover will output position data which can be used to publish NavSatFix messages (as described above).  The rover also outputs heading data via UBX-NAV-RELPOSNED messages.  Refer to Section 2.4 "Using the heading output" in the app note for more information.

Note: as this mode required two ZED-F9P devices connected simultaneously, you will need to configure each device with a different serial string and specify these in the launch files.  See description above on device identification parameters.

# Helpful tips

## Did the F9 device receive and use the RTCM data?
If you want to see if the UBX device has received and processed the RTCM data run this, after ensuring that CFG_MSGOUT_UBX_RXM_RTCM_USB is set to 1 via `rqt`
```
ros2 topic echo /ubx_rxm_rtcm
```
If `msg_used` shows 2 then the RTCM message has been used by the device.

This [RTCM 3 Message cheat sheet](https://www.use-snip.com/kb/knowledge-base/an-rtcm-message-cheat-sheet/) can be used to determine wha the `msg_type` field represents eg GPS, GLONASS, Galileo, SBAS, QZSS or BeiDou.

## Was a high precision location calculated?

Ensure CFG_MSGOUT_UBX_NAV_STATUS_USB is set to 1. It is lower priority message but can be used to determine what type of GPS fix and the type of differential error correction that has been supplied.

```
ros2 topic echo /ubx_nav_status
```

should output a message similar to
```
header:
  stamp:
    sec: 1684907069
    nanosec: 214298394
  frame_id: ubx
itow: 279887200
gps_fix:
  fix_type: 3
gps_fix_ok: true
diff_soln: true
wkn_set: true
tow_set: true
diff_corr: true
carr_soln_valid: true
map_matching:
  status: 0
psm:
  state: 0
spoof_det:
  state: 2
carr_soln:
  status: 0
ttff: 709
msss: 3416912
```

If the `gps_fix.fix_type` = 3, it means its a 3D fix. The `diff_soln` and `diff_corr` implies that the differential error correction has occurred.

## How precise is my solution?


After making sure that the deivce is processing RTCM messages and assuming that you have launched the node with `ros2 launch ublox_dgnss ublox_rover_hpposllh.launch.py`, run the following the command

```
ros2 topic echo /ubx_nav_hp_pos_llh
```
You should see something similiar to
```
header:
  stamp:
    sec: 1684907675
    nanosec: 765881914
  frame_id: ubx
version: 0
invalid_lon: false
invalid_lat: false
invalid_height: false
invalid_hmsl: false
invalid_lon_hp: false
invalid_lat_hp: false
invalid_height_hp: false
invalid_hmsl_hp: false
itow: 280493800
lon: 1534256850
lat: -280021154
height: 87232
hmsl: 49963
lon_hp: 31
lat_hp: -7
height_hp: -1
hmsl_hp: 1
h_acc: 169
v_acc: 123
```

The driver output the raw UBX data into the message so some interpreting is required.

`h_acc` and `v_acc` are in millimeters but need to be scaled by 0.1 - `h_acc: 16.9 mm` and v_acc: 12.3 mm` per the above. Such that there is centimeter level accruacy.

If in doubt as to what the scaling is, please look at the F9 interface description for an explanation. All debug log output have had scaling applied.

If you would like to calculate the high precision lon, lat and height values use the following code as a guide
```
   // Extract the LLH and high-precision components
    double lat = ubx_hppos_llh_msg->lat * 1e-7 + ubx_hppos_llh_msg->lat_hp * 1e-9;
    double lon = ubx_hppos_llh_msg->lon * 1e-7 + ubx_hppos_llh_msg->lon_hp * 1e-9;
    double alt = ubx_hppos_llh_msg->height * 1e-3 + ubx_hppos_llh_msg->height_hp * 1e-4;
```
