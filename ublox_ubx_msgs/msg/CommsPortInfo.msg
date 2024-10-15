# CommsPortInfo - Communication port status information

uint16 port_id         # Port ID
uint16 tx_pending      # Number of bytes pending for transmission
uint32 tx_bytes        # Total number of bytes transmitted
uint8 tx_usage         # Percentage of transmit buffer currently used
uint8 tx_peak_usage    # Peak usage percentage of the transmit buffer
uint16 rx_pending      # Number of bytes pending for reception
uint32 rx_bytes        # Total number of bytes received
uint8 rx_usage         # Percentage of receive buffer currently used
uint8 rx_peak_usage    # Peak usage percentage of the receive buffer
uint16 overrun_errs    # Number of buffer overrun errors
uint16[4] msgs         # Count of different message types processed: [0-3]
uint32 skipped         # Number of messages skipped