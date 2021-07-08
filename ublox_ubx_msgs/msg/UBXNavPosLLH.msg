# this message contains a UBX-NAV-POSLLH (0x01 0x02) record 
std_msgs/Header header # Header timestamp should be acquisition time

uint32 itow           # ms - GPS Time of week of the navigation epoch
int32 lon             # deg scale 1e-7 - longitude
int32 lat             # deg scale 1e-7 - latitude
int32 height          # mm - height above ellipsoid
int32 hmsl            # mm - height above mean sea level
uint32 h_acc          # mm - horizontal accuracy estimate
uint32 v_acc          # mm - vertical accuracy estimate