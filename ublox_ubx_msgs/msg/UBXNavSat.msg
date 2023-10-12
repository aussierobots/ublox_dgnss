# UBX-NAV-SAT (0x01 0x35) - Satellite information
std_msgs/Header header  # Header timestamp should be acquisition time

uint32 itow             # ms - GPS Time of week of the navigation epoch
uint8 version           # Message version (0x01 for this version)
uint8 num_svs           # Number of satellites
# uint8[2] reserved_0     # Reserved

SatInfo[] sv_info      # Repeated group of data for each satellite