# UBX-NAV-SIG (0x01 0x43) - Signal information
std_msgs/Header header  # Header timestamp should be acquisition time

uint32 itow            # ms - GPS Time of week of the navigation epoch
uint8 version          # Message version
uint8 num_sigs         # Number of signals
uint8[2] reserved_0    # Reserved

SigData[] sig_data     # Repeated group of data for each signal