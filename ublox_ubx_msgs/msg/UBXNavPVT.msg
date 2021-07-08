# this message contains a UBX-NAV-PVT (0x01 0x07) record
# Navigation position velocity time solution
std_msgs/Header header # Header timestamp should be acquisition time

uint32 itow           # ms - GPS Time of week of the navigation epoch
uint16 year           # y - year utc
uint8 month           # month - month utc
uint8 day             # d -day utc
uint8 hour            # h - hour utc
uint8 min             # min - min utc
uint8 sec             # s - sec utc
bool valid_date       # valid UTC date
bool valid_time       # valid UTC time of day
bool fully_resolved   # UTC Time of day has been fully resolved (no seconds uncertainty)
bool valid_mag        # valid magnetic declination
uint32 t_acc          # ns - time accuracy estimate (utc)
int32 nano            # ns - fraction of second, range -1e9 to 1e9 (utc)
GpsFix gps_fix        # GNSSFix type
bool gnss_fix_ok      # valid fix (ie within DOP & accruacy masks)
bool diff_soln        # differential corrections were applied
PSMPVT psm            # power save mode state
bool head_veh_valid   # heading of vehicle is valid, only set if the receiver is in sensor fusion mode
CarrSoln carr_soln    # carrier phase rnage solution status
bool confirmed_avail  # information about UTC date and time of day validity confirmation is available 
bool confirmed_date   # UTC Date validity could be confirmed
bool confirmed_time   # UTC Time of Day validity could be confirmed
uint8 num_sv          # Number of satellites used in Nav Solution
int32 lon             # deg scale ie-7 - longitude
int32 lat             # deg scale ie-7 - latitude
int32 height          # mm - Height above ellipsoid
int32 hmsl            # mm - Height above mean sea level
uint32 h_acc          # mm - Horizontal accuracy estimate 
uint32 v_acc          # mm - Vertical accuracy estimate 
int32 vel_n           # mm/s - NED north velocity
int32 vel_e           # mm/s - NED east velocity
int32 vel_d           # mm/s - NED down velocity
int32 g_speed         # mm/s - ground speed (2-D)
int32 head_mot        # deg scale ie-5 - heading of motion (2-D) 
uint32 s_acc          # mm/s - speed accuracy estimate
uint32 head_acc       # deg scale ie-5 - heading accruracy estime (both motion and vehicle)
uint16 p_dop          # scale 0.01 - Position DOP
bool invalid_llh      # invalid lon, lat, height, hmsl
int32 head_veh        # deg scale 1e-5 - Heading of vehicle (2-D), this is only valid when head_veh_valid is set, 
                      # otherwise the output is set to the heading of motion
int16 mag_dec         # deg scale 1e-2 - Magnetic declination. Only supported in ADR 4.10 and later
uint16 mag_acc        # deg scale ie-2 - Mangetic declination accuracy. Only supported in ADR 4.10 and later