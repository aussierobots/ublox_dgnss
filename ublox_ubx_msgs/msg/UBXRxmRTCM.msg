# this message contains a UBX-RXM-RTCM (0x02 0x32) record
# RTCM Input status
# This message shows info on a received RTCM input message. It is output upon successful parsing of an RTCM
# input message, irrespective of whether the RTCM message is supported or not by the receiver.
std_msgs/Header header # Header timestamp should be acquisition time

uint8 version           # message version (0x02 for this version)

bool crc_failed         # 0 When RTCM received & CRC Ok, 1 when failed
uint8 msg_used          # 2 = RTCM msg used sucessfully, 1 = Not Used, 0 = do not know

uint16 sub_type         # message subtype, only applicable to ublox proprietary RTCM 4072
uint16 ref_station      # reference station ID refer interface description
uint16 msg_type         # message type