# this message contains a UBX-NAV-ODO (0x01 0x3c) record 
# Relative position information in NED frame
std_msgs/Header header # Header timestamp should be acquisition time

uint8 version             # message version
uint16 ref_station_id     # reference station ID. Must be in the range 0..4096
uint32 itow               # ms - GPS Time of week of the navigation epoch
int32 rel_pos_n           # cm - North component of relative position vector 
int32 rel_pos_e           # cm - East component of relative position vector 
int32 rel_pos_d           # cm - Down component of relative position vector 
int32 rel_pos_length      # cm - Length of the relative position vector
int32 rel_pos_heading     # deg scale 1e-5 - Heading of the relative position vector
int8 rel_pos_hp_n         # mm scale 0.1 - full HP North is given by relPosN + (relPosHPN * 1e-2) 
int8 rel_pos_hp_e         # mm scale 0.1 - full HP East is given by relPosE + (relPosHPE * 1e-2) 
int8 rel_pos_hp_d         # mm scale 0.1 - full HP Down is given by relPosD + (relPosHPD * 1e-2) 
int8 rel_pos_hp_length    # mm scale 0.1 - full HP length is given by relPosLength + (relPosHPLength * 1e-2) 
uint32 acc_n              # mm scale 0.1 - Accuracy of relative position North Component
uint32 acc_e              # mm scale 0.1 - Accuracy of relative position East Component
uint32 acc_d              # mm scale 0.1 - Accuracy of relative position Down Component
uint32 acc_length         # mm scale 0.1 - Accuracy of length of the relative position vector
uint32 acc_heading        # deg scale 1e-5 - Accuracy of heading of the relative position vector
bool gnss_fix_ok          # A valid fix (i.e within DOP & accuracy masks) 
bool diff_soln            # differential corrections were applied
bool rel_pos_valid        # relative position components and accuracies are valid and, in moving base mode only, if baseline is valid
CarrSoln carr_soln        # carrier phase range solution
bool is_moving            # if the received is operating in moving base mode
bool ref_pos_miss         # if extrapolated reference position was used to compute moving base solution this epoch. 
bool ref_obs_miss         # if extrapolated reference observations were used to compute moving base solution this epoch. 
bool rel_pos_heading_valid # relPosHeading is valid
bool rel_pos_normalized   # components of the relative position vector (including the high-precision parts) are normalized
