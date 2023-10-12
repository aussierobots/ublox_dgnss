# Bit field structure for eph
uint8 EPH_USABILITY_UNKNOWN = 31      # The usability period is unknown
uint8 EPH_USABILITY_OVER_450_MIN = 30 # The usability period is more than 450 minutes
uint8 EPH_USABILITY_EXPIRED = 0       # Ephemeris can no longer be used
uint8 eph_usability                   # How long the receiver will be able to use the stored ephemeris data from now on.
                                      # For 30 > n > 0, the usability period is between (n-1)*15 and n*15 minutes.

uint8 EPH_SOURCE_NOT_AVAILABLE = 0
uint8 EPH_SOURCE_GNSS_TRANSMISSION = 1
uint8 EPH_SOURCE_EXTERNAL_AIDING = 2
uint8 eph_source                     # Ephemeris source