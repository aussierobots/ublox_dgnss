# UBX-RXM-SPARTN (0x02 0x33) - SPARTN input status
std_msgs/Header header  # Header timestamp should be acquisition time

# Constants for message used (flags bits 2...1)
uint8 MSG_UNKNOWN = 0
uint8 MSG_NOT_USED = 1
uint8 MSG_USED = 2

# Fields
uint8 version       # Message version
uint8 msg_used      # Status of the SPARTN message used (refer to constants)
uint16 sub_type     # Message subtype
# uint8[2] reserved0  # Reserved
uint16 msg_type     # Message type