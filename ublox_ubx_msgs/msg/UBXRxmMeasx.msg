# UBX-RXM-MEASX (0x02 0x14) - Satellite measurements for RRLP
std_msgs/Header header  # Header timestamp should be acquisition time

# Constants for TOW Set
uint8 TOW_NOT_SET = 0
uint8 TOW_SET = 1
uint8 TOW_SET2 = 2

# Fields
uint8 version         # Message version
# uint8[3] reserved_0   # Reserved
uint32 gps_tow        # ms - GPS measurement reference time
uint32 glo_tow        # ms - GLONASS measurement reference time
uint32 bds_tow        # ms - BeiDou measurement reference time
# uint8[4] reserved_1   # Reserved
uint32 qzss_tow       # ms - QZSS measurement reference time
uint16 gps_tow_acc    # 2^-4 ms - GPS measurement reference time accuracy
uint16 glo_tow_acc    # 2^-4 ms - GLONASS measurement reference time accuracy
uint16 bds_tow_acc    # 2^-4 ms - BeiDou measurement reference time accuracy
# uint8[2] reserved_2   # Reserved
uint16 qzss_tow_acc   # 2^-4 ms - QZSS measurement reference time accuracy
uint8 num_sv          # Number of satellites in repeated block
uint8 flags           # Flags (refer to constants for TOW Set)
# uint8[8] reserved_3   # Reserved

MeasxData[] sv_data   # Repeated group of data for each satellite