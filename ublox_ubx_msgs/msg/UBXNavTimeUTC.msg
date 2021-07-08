# this message contains a UBX-NAV-EOE (0x01 0x21) record UTC Time Solution
std_msgs/Header header # Header timestamp should be acquisition time

uint32 itow           #  ms - GPS Time of week of the navigation epoch
uint32 t_acc          # ns - time accuracy estimate (utc)
int32 nano            # ns - fraction of second, range -1e9 to 1e9 (utc)
int16 year            # y - year utc
int8 month            # month - month utc
int8 day              # d -day utc
int8 hour             # h - hour utc
int8 min              # min - min utc
int8 sec              # s - sec utc
bool valid_tow        # valid Time of Week
bool valid_wkn        # valid Week Number
bool valid_utc        # valid UTC time
UtcStd utc_std        # UTC standard identifier