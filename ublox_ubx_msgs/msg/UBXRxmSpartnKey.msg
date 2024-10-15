# UBX-RXM-SPARTNKEY (0x02 0x36) - Transfer dynamic SPARTN keys
std_msgs/Header header  # Header timestamp should be acquisition time

# Fields
uint8 version       # Message version
uint8 num_keys      # Number of keys in the message (0, 1, or 2)
uint8[2] reserved0  # Reserved

SpartnKeyInfo[] key_info   # Repeated group for key info
uint8[] key_payload        # Concatenated key payloads (raw bytes)