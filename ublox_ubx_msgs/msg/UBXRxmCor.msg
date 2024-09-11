# UBX-RXM-COR (0x02 0x34) - Differential correction input status
std_msgs/Header header  # Header timestamp should be acquisition time

# Fields
uint8 version         # Message version
uint8 ebno            # Energy per bit to noise ratio (Eb/N0) in dB
# uint8[2] reserved0    # Reserved
CorStatusInfo status_info  # Status info (separated into its own message)
uint16 msg_type       # Message type
uint16 msg_sub_type   # Message subtype