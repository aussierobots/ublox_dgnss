# UBX-NAV-DAHEADING (0x01 0x45), version 0x02, 60-byte UBX payload.
# Relative position in NED from antenna 1 to antenna 2.
# Positions and accuracies use the receiver's selected geodetic datum.
# Reserved bytes are omitted; flags are decoded by the driver.
std_msgs/Header header    # Timestamp should be acquisition time
uint8 version            # Message version: 0x02
uint32 itow              # ms - GPS time of week of the navigation epoch
int32 rel_pos_n          # mm - North component of the antenna baseline
int32 rel_pos_e          # mm - East component of the antenna baseline
int32 rel_pos_d          # mm - Down component of the antenna baseline
int32 rel_pos_length     # mm - Length of the antenna baseline
# 1e-5 deg - Clockwise heading from True North, range 0..360 deg.
# Valid only when flags.rel_pos_heading_valid is true.
# CFG-NAVSPG-DAHEADING_OFFSET adjusts this field to the vehicle heading.
# The offset does not alter the baseline components or length.
int32 rel_pos_heading
uint32 acc_n             # mm - North component accuracy
uint32 acc_e             # mm - East component accuracy
uint32 acc_d             # mm - Down component accuracy
uint32 acc_length        # mm - Baseline length accuracy
uint32 acc_heading       # 1e-5 deg - Heading accuracy
DualAntFlags flags