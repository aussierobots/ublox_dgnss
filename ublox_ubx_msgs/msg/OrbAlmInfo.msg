# Bit field structure for alm
uint8 ALM_USABILITY_UNKNOWN = 31      # The usability period is unknown
uint8 ALM_USABILITY_OVER_30_DAYS = 30 # The usability period is more than 30 days
uint8 ALM_USABILITY_EXPIRED = 0       # Almanac can no longer be used
uint8 alm_usability                   # How long the receiver will be able to use the stored almanac data from now on.
                                     # For 30 > n > 0, the usability period is between n-1 and n days.

uint8 ALM_SOURCE_NOT_AVAILABLE = 0
uint8 ALM_SOURCE_GNSS_TRANSMISSION = 1
uint8 ALM_SOURCE_EXTERNAL_AIDING = 2
uint8 alm_source                      # Almanac source