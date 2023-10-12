# Bit field structure for otherOrb
uint8 ANO_AOP_USABILITY_UNKNOWN = 31      # The usability period is unknown
uint8 ANO_AOP_USABILITY_OVER_30_DAYS = 30 # The usability period is more than 30 days
uint8 ANO_AOP_USABILITY_EXPIRED = 0       # Data can no longer be used
uint8 ano_aop_usability                   # How long the receiver will be able to use the orbit data from now on.
                                          # For 30 > n > 0, the usability period is between n-1 and n days.

uint8 TYPE_NO_ORBIT_DATA = 0
uint8 TYPE_ASSISTNOW_OFFLINE = 1
uint8 TYPE_ASSISTNOW_AUTONOMOUS = 2
uint8 orb_type                                # Type of orbit data