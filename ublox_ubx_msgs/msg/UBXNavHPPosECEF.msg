# this message contains a UBX-NAV-HPPOSECEF (0x01 0x13) record 
std_msgs/Header header # Header timestamp should be acquisition time

uint8 version         # message version
uint32 itow           # ms - GPS Time of week of the navigation epoch
int32 ecef_x          # cm - ECEF X coordinate
int32 ecef_y          # cm - ECEF Y coordinate
int32 ecef_z          # cm - ECEF Z coordinate
# HPs must be in the range of -99..+99. Precise coordinate in cm = ecef_? + (ecef_?_hp * 1e-2)
int8 ecef_x_hp        # mm scale 0.1 - high precision ECEF X coordinate
int8 ecef_y_hp        # mm scale 0.1 - high precision ECEF Y coordinate
int8 ecef_z_hp        # mm scale 0.1 - high precision ECEF Z coordinate
bool invalid_ecef_x
bool invalid_ecef_y
bool invalid_ecef_z
bool invalid_ecef_x_hp
bool invalid_ecef_y_hp
bool invalid_ecef_z_hp
uint32 p_acc          # mm scale 0.1 - position accuracy estimate 