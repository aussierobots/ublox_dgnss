# Signal related flags
uint8 HEALTH_UNKNOWN = 0
uint8 HEALTH_HEALTHY = 1
uint8 HEALTH_UNHEALTHY = 2
uint8 health         # Signal health flag

bool pr_smoothed     # Pseudorange has been smoothed
bool pr_used         # Pseudorange has been used for this signal
bool cr_used         # Carrier range has been used for this signal
bool do_used         # Range rate (Doppler) has been used for this signal
bool pr_corr_used    # Pseudorange corrections have been used for this signal
bool cr_corr_used    # Carrier range corrections have been used for this signal
bool do_corr_used    # Range rate (Doppler) corrections have been used for this signal