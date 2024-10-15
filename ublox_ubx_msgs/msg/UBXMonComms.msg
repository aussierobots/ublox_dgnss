# UBXMonComms - Overall communications monitoring message
std_msgs/Header header # Header timestamp should be acquisition time

# Constants for protocol in prot_ids
uint8 PROTOCOL_UBX = 0
uint8 PROTOCOL_NMEA = 1
uint8 PROTOCOL_RTCM2 = 2
uint8 PROTOCOL_RTCM3 = 5
uint8 PROTOCOL_SPARTN = 6
uint8 NO_PROTOCOL = 255

uint8 version          # Message version
uint8 n_ports          # Number of ports reported
uint8 tx_errors        # Transmit errors flags
uint8 reserved0        # Reserved byte
uint8[4] prot_ids      # Protocol identifiers active on this device: [0-3]
CommsPortInfo[] ports  # Array of CommsPortInfo for each reported port