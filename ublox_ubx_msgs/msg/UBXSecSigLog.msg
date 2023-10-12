# UBX-SEC-SIGLOG (0x27 0x10) - Signal security log
std_msgs/Header header  # Header timestamp should be acquisition time

# Fields
uint8 version           # Message version
uint8 num_events        # Number of events
# uint8[6] reserved_0     # Reserved

# Repeated group of events
SigLogEvent[] events    # Array of SigLogEvent messages