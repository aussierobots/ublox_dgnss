# UBX-NAV-SBAS (0x01 0x32) - SBAS status data
std_msgs/Header header  # Header timestamp should be acquisition time

# Constants for SBAS Mode
uint8 MODE_DISABLED = 0
uint8 MODE_ENABLED_INTEGRITY = 1
uint8 MODE_ENABLED_TEST = 3

# Constants for SBAS System
int8 SYS_UNKNOWN = -1
int8 SYS_WAAS = 0
int8 SYS_EGNOS = 1
int8 SYS_MSAS = 2
int8 SYS_GAGAN = 3
int8 SYS_GPS = 16

uint32 itow             # ms - GPS Time of week of the navigation epoch
uint8 geo               # PRN Number of the GEO where correction and integrity data is used from
uint8 mode              # SBAS Mode
int8 sys                # SBAS System (WAAS/EGNOS/...)
SBASService service     # SBAS Services available
uint8 cnt               # Number of SV data following
SBASStatusFlags status_flags  # SBAS status flags
uint8[2] reserved_0     # Reserved

SBASSvData[] sv_data    # Repeated group of data for each satellite
