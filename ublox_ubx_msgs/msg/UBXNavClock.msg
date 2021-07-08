# this message contains a UBX-NAV-CLOCK (0x01 0x22) record clock solution
std_msgs/Header header # Header timestamp should be acquisition time

uint32 itow           #  ms - GPS Time of week of the navigation epoch
int32 clk_b           # ns - Clock bias
int32 clk_d           # ns/s - Clock drift
uint32 t_acc          # ns - Time accuracy estimate
uint32 f_acc          # ps/s - Frequency accuracy estimate