# UBXRxmSFRBX.msg
# UBX-RXM-SFRBX (0x02 0x13) - Broadcast navigation data subframe

std_msgs/Header header  # Header timestamp should be acquisition time

# Fields
uint8 gnss_id       # GNSS identifier (see Satellite Numbering)
uint8 sv_id         # Satellite identifier (see Satellite Numbering)
uint8 sig_id        # Signal identifier (see Signal Identifiers)
uint8 freq_id       # Only used for GLONASS: frequency slot + 7 (range 0 to 13)
uint8 num_words     # Number of data words contained in this message (up to 16)
uint8 chn           # Tracking channel number the message was received on
uint8 version       # Message version (0x02 for this version)

uint32[] dwrd       # Data words (numWords entries)
