# Information for each Signal in UBX-NAV-SIG

# Constants for Signal Quality Indicator
uint8 QUALITY_NO_SIGNAL = 0
uint8 QUALITY_SEARCHING_SIGNAL = 1
uint8 QUALITY_SIGNAL_ACQUIRED = 2
uint8 QUALITY_SIGNAL_UNUSABLE = 3
uint8 QUALITY_CODE_LOCKED = 4
uint8 QUALITY_CODE_CARRIER_LOCKED = 5
# 6 and 7 are also considered as code and carrier locked and time synchronized

# Constants for Correction Source
uint8 CORR_NONE = 0
uint8 CORR_SBAS = 1
uint8 CORR_BEIDOU = 2
uint8 CORR_RTCM2 = 3
uint8 CORR_RTCM3_OSR = 4
uint8 CORR_RTCM3_SSR = 5
uint8 CORR_QZSS_SLAS = 6
uint8 CORR_SPARTN = 7
uint8 CORR_CLAS = 8

# Constants for Ionospheric Model
uint8 IONO_NONE = 0
uint8 IONO_KLOB_GPS = 1
uint8 IONO_SBAS = 2
uint8 IONO_KLOB_BEIDOU = 3
# 8 is considered as Iono delay derived from dual frequency observations

uint8 gnss_id       # GNSS identifier
uint8 sv_id         # Satellite identifier
uint8 sig_id        # New style signal identifier
uint8 freq_id       # Frequency slot + 7 (GLONASS)
int16 pr_res        # Pseudorange residual (scaled by 0.1)
uint8 cno           # Carrier-to-noise density ratio
uint8 quality_ind   # Signal quality indicator
uint8 corr_source   # Correction source
uint8 iono_model    # Ionospheric model used
SigFlags sig_flags  # Signal related flags
# uint8[4] reserved_1 # Reserved