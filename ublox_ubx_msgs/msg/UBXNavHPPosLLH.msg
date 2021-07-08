# this message contains a UBX-NAV-HPPOSLLH (0x01 0x14) record 
std_msgs/Header header # Header timestamp should be acquisition time

uint8 version         # message version
bool invalid_lon        
bool invalid_lat
bool invalid_height
bool invalid_hmsl     # invalid height above mean sea level
bool invalid_lon_hp
bool invalid_lat_hp
bool invalid_height_hp
bool invalid_hmsl_hp  # invalid height above mean sea level
uint32 itow           # ms - GPS Time of week of the navigation epoch
int32 lon             # deg scale 1e-7 - longitude
int32 lat             # deg scale 1e-7 - latitude
int32 height          # mm - height above ellipsoid
int32 hmsl            # mm - height above mean sea level
int8 lon_hp           # deg scale 1e-9 - Precision longitude in deg * 1e-7 = lon +(lonHp * 1e-2)
int8 lat_hp           # deg scale 1e-9 - Precise latitude in deg  *  1e-7  =  lat  +(latHp * 1e-2) 
int8 height_hp        # mm scale 0.1 - Precise height in mm = height + (heightHp * 0.1)
int8 hmsl_hp          # mm scale 0.1 - Precise height in mm =hMSL + (hMSLHp * 0.1)
uint32 h_acc          # mm scale 0.1 - horizontal accuracy estimate
uint32 v_acc          # mm scale 0.1 - vertical accuracy estimate