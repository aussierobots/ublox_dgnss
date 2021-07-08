# this message contains a UBX-NAV-VELNED (0x01 0x12) record
# Velocity solution in NED frame
std_msgs/Header header # Header timestamp should be acquisition time

uint32 itow            # ms - GPS Time of week of the navigation epoch
int32 vel_n            # cm/s - NED north velocity
int32 vel_e            # cm/s - NED east velocity
int32 vel_d            # cm/s - NED down velocity
uint32 speed           # cm/s - speed (3-D)
uint32 g_speed         # cm/s - ground speed (2-D)
int32 heading          # deg scale ie-5 - heading of motion (2-D) 
uint32 s_acc           # cm/s - speed accuracy estimate
uint32 c_acc           # deg scale ie-5 - course/heading accruracy estime (both motion and vehicle)
