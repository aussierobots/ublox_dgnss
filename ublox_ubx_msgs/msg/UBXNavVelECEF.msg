# this message contains a UBX-NAV-VELECEF (0x01 0x11) record
# Velocity solution in ECEF
std_msgs/Header header # Header timestamp should be acquisition time

uint32 itow            # ms - GPS Time of week of the navigation epoch
int32 ecef_vx          # cm/s - ECEF X velocity
int32 ecef_vy          # cm/s - ECEF Y velocity
int32 ecef_vz          # cm/s - ECEF Z velocity
uint32 s_acc           # cm/s - velocity accuracy estimate