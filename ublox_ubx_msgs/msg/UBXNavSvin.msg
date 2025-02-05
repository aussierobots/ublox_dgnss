# this message contains a UBX-NAV-SVIN (0x01 0x3b) record
std_msgs/Header header  # Header timestamp should be acquisition time

uint8 version         # Message version
uint32 itow           # ms - GPS Time of week of the navigation epoch 
uint32 dur            # s - Passed survey-in observation time
int32  mean_x         # cm - Current survey-in mean position ECEF X coordinate
int32  mean_y         # cm - Current survey-in mean position ECEF Y coordinate
int32  mean_z         # cm - Current survey-in mean position ECEF Z coordinate
int8   mean_x_hp      # 0.1mm - Current high-precision survey-in mean position
                      # ECEF X coordinate. Must be in the range -99..+99.
                      # The current survey-in mean position ECEF X coordinate,
                      # in units of cm, is given by meanX + (0.01 * meanXHP)
int8   mean_y_hp      # 0.1mm - Current high-precision survey-in mean position
                      # ECEF Y coordinate. Must be in the range -99..+99.
                      # The current survey-in mean position ECEF Y coordinate,
                      # in units of cm, is given by meanY + (0.01 * meanYHP)
int8   mean_z_hp      # 0.1mm - Current high-precision survey-in mean position
                      # ECEF Z coordinate. Must be in the range -99..+99.
                      # The current survey-in mean position ECEF Z coordinate,
                      # in units of cm, is given by meanZ + (0.01 * meanZHP)
uint32 mean_acc       # 0.1mm - Current survey-in mean position accuracy
uint32 obs            # Number of position observations used during survey-in
bool   valid          # Survey-in position validity flag, 1 = valid, otherwise
                      # 0
bool   active         # Survey-in in progress flag, 1 = in-progress, otherwise
                      # 0