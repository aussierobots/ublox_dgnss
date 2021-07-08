# this message contains a UBX-NAV-STATUS (0x01 0x03) record
std_msgs/Header header # Header timestamp should be acquisition time

uint32 itow           #  ms - GPS Time of week of the navigation epoch

GpsFix gps_fix         # gps fix

# navigation status flags
bool gps_fix_ok       # position and velocity valid and within DOP and ACCMasks
bool diff_soln        # differential corrections were applied
bool wkn_set          # week number valid
bool tow_set          # time of week valid

# navigation fix status information 
bool diff_corr        # differential corrections available 
bool carr_soln_valid  # valid carrSoln

MapMatching map_matching    # map matching status

# further information about navigation output
PSMStatus psm       # power save mode state
SpoofDet spoof_det
CarrSoln carr_soln         # Carrier phase solution status 

uint32 ttff             # time to first fix (milliseond time tag)
uint32 msss             # milliseconds since Startup/Reset
