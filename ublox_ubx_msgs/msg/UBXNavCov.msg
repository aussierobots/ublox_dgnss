# This message contains a UBX-NAV-COV (0x01 0x36) record clock solution
#
# It outputs the covariance matrices for the position and velocity
# solutions in the topocentric coordinate system defined as the local-level North
# (N), East (E), Down (D) frame. As the covariance matrices are symmetric, only
# the upper triangular part is output.
std_msgs/Header header # Header timestamp should be acquisition time

uint32 itow           #  ms - GPS Time of week of the navigation epoch
uint8 version         # message version (0x00 for this version)
bool pos_cor_valid    # position covariance matrix validity flag
bool vel_cor_valid    # velocity covariance matrix validity flag
float32 pos_cov_nn    # m^2 - Position covariance matric value p_NN
float32 pos_cov_ne    # m^2 - Position covariance matric value p_NE
float32 pos_cov_nd    # m^2 - Position covariance matric value p_ND
float32 pos_cov_ee    # m^2 - Position covariance matric value p_EE
float32 pos_cov_ed    # m^2 - Position covariance matric value p_ED
float32 pos_cov_dd    # m^2 - Position covariance matric value p_DD
float32 vel_cov_nn    # m^2/s^2 - Velocity covariance matric value v_NN
float32 vel_cov_ne    # m^2/s^2 - Velocity covariance matric value v_NE
float32 vel_cov_nd    # m^2/s^2 - Velocity covariance matric value v_ND
float32 vel_cov_ee    # m^2/s^2 - Velocity covariance matric value v_EE
float32 vel_cov_ed    # m^2/s^2 - Velocity covariance matric value v_ED
float32 vel_cov_dd    # m^2/s^2 - Velocity covariance matric value v_DD