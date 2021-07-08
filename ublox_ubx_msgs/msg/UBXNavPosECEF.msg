# this message contains a UBX-NAV-POSECEF (0x01 0x01) record 
std_msgs/Header header # Header timestamp should be acquisition time

uint32 itow           # ms - GPS Time of week of the navigation epoch
int32 ecef_x          # cm - ECEF X coordinate
int32 ecef_y          # cm - ECEF Y coordinate
int32 ecef_z          # cm - ECEF Z coordinate
uint32 p_acc          # cm - position accuracy estimate 