# UBX-SEC-UNIQID (0x27 0x03) - Unique chip ID
std_msgs/Header header  # Header timestamp should be acquisition time

# Fields
uint8 version          # Message version
# uint8[3] reserved_0    # Reserved
uint8[5] unique_id     # Unique chip ID