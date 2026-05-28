# UBXRxmPMP.msg
# UBX-RXM-PMP (0x02 0x72) - Point to Multipoint (L-band) input message from NEO-D9S

std_msgs/Header header  # Header timestamp should be acquisition time

# Fields
uint8 version                 # Message version (0x01 for this version)
uint16 num_bytes_user_data    # Number of bytes the userData block has in this frame (0..504)
uint32 time_tag               # ms - time since startup when frame started
uint32[2] unique_word         # Received unique words
uint16 service_identifier     # Received service identifier
uint8 spare                   # Received spare data
uint8 unique_word_bit_errors  # Number of bit errors in both unique words
uint16 fec_bits               # Number of bits corrected by FEC
uint8 ebno                    # 2^-3 dB - energy per bit to noise power spectral density ratio

uint8[] user_data             # Received user data (= num_bytes_user_data)
