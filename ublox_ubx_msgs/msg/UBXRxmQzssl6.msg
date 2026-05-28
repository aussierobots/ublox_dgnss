# UBXRxmQZSSL6.msg
# UBX-RXM-QZSSL6 (0x02 0x73) - QZSS L6 (CLAS) input message from NEO-D9C

std_msgs/Header header  # Header timestamp should be acquisition time

# msg_name values (chInfo bit 10)
uint8 MSG_NAME_L6D = 0
uint8 MSG_NAME_L6E = 1

# err_status values (chInfo bits 13...12)
uint8 ERR_UNKNOWN = 0
uint8 ERR_ERROR_FREE = 1
uint8 ERR_ERRONEOUS = 2

# ch_name values (chInfo bits 15...14)
uint8 CH_NAME_A = 0
uint8 CH_NAME_B = 1

# Fields
uint8 version       # Message version (0x01 for this version)
uint8 sv_id         # Satellite identifier (see Satellite Numbering)
uint16 cno          # 2^-8 dBHz - mean C/N0
uint32 time_tag     # ms - local time tag at start of received QZSS L6 message
uint8 group_delay   # ns - L6 group delay w.r.t. L2 on channel
uint8 bit_err_corr  # Number of bit errors corrected by Reed-Solomon decoder

# chInfo decoded fields
uint8 chn           # Receiver channel (0, 1)
uint8 msg_name      # Message name, 0=L6D, 1=L6E
uint8 err_status    # Error status, 0=unknown, 1=error-free, 2=erroneous
uint8 ch_name       # Channel name, 0=channel A, 1=channel B

uint8[] msg_bytes   # Bytes in a QZSS L6 message (250 bytes)
