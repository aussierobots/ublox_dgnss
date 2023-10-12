# UBXRxmRAWX.msg
# UBX-RXM-RAWX (0x02 0x15) - Multi-GNSS raw measurements

std_msgs/Header header  # Header timestamp should be acquisition time

# Fields
float64 rcv_tow      # s - Measurement time of week
uint16 week          # weeks - GPS week number
int8 leap_s          # s - GPS leap seconds
uint8 num_meas       # Number of measurements
RecStat rec_stat       # Receiver tracking status bitfield
uint8 version        # Message version
# uint8[2] reserved0   # Reserved

RawxData[] rawx_data # Repeated group of data for each satellite