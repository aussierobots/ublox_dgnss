# Bit field structure for flags in UBX-NAV-DAHEADING (0x01 0x45).
# Decoded from the uint32 flags word at UBX payload byte offset 56.
# Bit 0: valid fix within DOP and accuracy masks
bool gnss_fix_ok
# Bit 1: differential corrections applied
bool diff_soln
# Bit 2: relative position components, accuracy and baseline valid
bool rel_pos_valid
# Bits 3..4: carrier phase range solution status
uint8 CARR_SOLN_NONE = 0
uint8 CARR_SOLN_FLOAT = 1
uint8 CARR_SOLN_FIXED = 2
uint8 carr_soln
# Bit 6: relative position heading valid
bool rel_pos_heading_valid