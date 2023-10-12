# Information for each Satellite in SBAS
uint8 svid        # SV ID
uint8 reserved_1  # Reserved
uint8 udre        # Monitoring status
uint8 sv_sys      # System (WAAS/EGNOS/...)
uint8 sv_service  # Services available, same as SERVICE
uint8 reserved_2  # Reserved
int16 prc         # Pseudo Range correction in [cm]
uint8[2] reserved_3  # Reserved
int16 ic          # Ionosphere correction in [cm]