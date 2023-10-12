# UBX-NAV-ORB (0x01 0x34) - GNSS orbit database info
std_msgs/Header header  # Header timestamp should be acquisition time

uint32 itow             # ms - GPS Time of week of the navigation epoch
uint8 version           # Message version (0x01 for this version)
uint8 num_sv            # Number of SVs in the database
uint8[2] reserved_0     # Reserved

OrbSVInfo[] sv_info        # Repeated group of data for each SV