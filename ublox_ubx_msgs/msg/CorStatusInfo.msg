# CorStatusInfo - Status information for UBX-RXM-COR

# Constants for protocol in statusInfo
uint8 PROTOCOL_UNKNOWN = 0
uint8 PROTOCOL_RTCM3 = 1
uint8 PROTOCOL_SPARTN = 2
uint8 PROTOCOL_UBX_RXM_PMP = 29
uint8 PROTOCOL_UBX_RXM_QZSSL6 = 30

# Constants for error status
uint8 ERR_UNKNOWN = 0
uint8 ERR_ERROR_FREE = 1
uint8 ERR_ERRONEOUS = 2

# Constants for message usage
uint8 MSG_USED_UNKNOWN = 0
uint8 MSG_NOT_USED = 1
uint8 MSG_USED = 2

# Constants for encryption and decryption status
uint8 MSG_ENCRYPTION_UNKNOWN = 0
uint8 MSG_NOT_ENCRYPTED = 1
uint8 MSG_ENCRYPTED = 2

uint8 MSG_DECRYPTION_UNKNOWN = 0
uint8 MSG_NOT_DECRYPTED = 1
uint8 MSG_DECRYPTED = 2

# Fields
uint8 protocol         # Input correction data protocol (refer to constants)
uint8 err_status       # Error status of the received correction message
uint8 msg_used         # Status of receiver using the input message
uint16 correction_id   # Identifier for the correction stream (station ID)
bool msg_type_valid    # Validity of the msgType field
bool msg_sub_type_valid  # Validity of the msgSubType field
bool msg_input_handle  # Input handling support of the input message
uint8 msg_encrypted    # Encryption status of the input message
uint8 msg_decrypted    # Decryption status of the input message