# Bit field structure for flags in UBX-NAV-SAT (0x01 0x35)

# Signal quality indicator
uint8 QUALITY_NO_SIGNAL = 0
uint8 QUALITY_SEARCHING = 1
uint8 QUALITY_ACQUIRED = 2
uint8 QUALITY_UNUSABLE = 3
uint8 QUALITY_CODE_LOCKED = 4
uint8 QUALITY_CARRIER_LOCKED = 5
uint8 quality_ind

# Signal currently being used for navigation
bool sv_used

# Signal health flag
uint8 HEALTH_UNKNOWN = 0
uint8 HEALTH_HEALTHY = 1
uint8 HEALTH_UNHEALTHY = 2
uint8 health

# Differential correction data availability
bool diff_corr

# Carrier smoothed pseudorange used
bool smoothed

# Orbit source
uint8 ORBIT_NO_INFO = 0
uint8 ORBIT_EPH_USED = 1
uint8 ORBIT_ALM_USED = 2
uint8 ORBIT_ASSISTNOW_OFFLINE = 3
uint8 ORBIT_ASSISTNOW_AUTONOMOUS = 4
uint8 orbit_source

# Ephemeris availability
bool eph_avail

# Almanac availability
bool alm_avail

# AssistNow Offline data availability
bool ano_avail

# AssistNow Autonomous data availability
bool aop_avail

# Corrections used flags
bool sbas_corr_used
bool rtcm_corr_used
bool slas_corr_used
bool spartn_corr_used
bool pr_corr_used
bool cr_corr_used
bool do_corr_used
bool clas_corr_used