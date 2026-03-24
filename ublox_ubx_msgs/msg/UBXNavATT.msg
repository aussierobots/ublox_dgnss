# this message contains a UBX-NAV-ATT (0x01 0x05) record
# Vehicle Attitude Solution
std_msgs/Header header # Header timestamp should be acquisition time

uint32 itow            # ms - GPS Time of week of the navigation epoch
uint8 version          # Message version (0 for this version)
int32 roll             # deg scale 1e-5 - Vehicle roll
int32 pitch            # deg scale 1e-5 - Vehicle pitch
int32 heading          # deg scale 1e-5 - Vehicle heading
uint32 acc_roll        # deg scale 1e-5 - Vehicle roll accuracy
uint32 acc_pitch       # deg scale 1e-5 - Vehicle pitch accuracy
uint32 acc_heading     # deg scale 1e-5 - Vehicle heading accuracy
