# this message contains a UBX-NAV-ODO (0x01 0x09) record Odometer solution
std_msgs/Header header # Header timestamp should be acquisition time

uint8 version         # message version
uint32 itow           # ms - GPS Time of week of the navigation epoch
uint32 distance       # m - ground distance since last reset
uint32 total_distance # m - total comulative ground distance (since last cold start)
uint32 distance_std   # m - ground distance accuracy (1-sigma)