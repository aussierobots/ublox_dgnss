# MeasxData - Satellite measurements data for UBX-RXM-MEASX (0x02 0x14)

# Constants for Multipath Indicators
uint8 MPATH_NOT_MEASURED = 0  # Not measured
uint8 MPATH_LOW = 1           # Low multipath
uint8 MPATH_MEDIUM = 2        # Medium multipath
uint8 MPATH_HIGH = 3          # High multipath

# Fields
uint8 gnss_id         # GNSS identifier
uint8 sv_id           # Satellite identifier
uint8 c_no            # dBHz - Carrier noise ratio
uint8 mpath_indic     # Multipath indicator (refer to constants)
int32 doppler_ms      # 0.04 m/s - Doppler measurement in m/s
int32 doppler_hz      # 0.2 Hz - Doppler measurement in Hz
uint16 whole_chips    # Whole value of the code phase measurement
uint16 frac_chips     # Fractional value of the code phase measurement
uint32 code_phase     # 2^-21 ms - Code phase in ms
uint8 int_code_phase  # ms - Integer part of the code phase
uint8 pseu_range_rms_err  # Pseudorange RMS error index
# uint8[2] reserved_4   # Reserved