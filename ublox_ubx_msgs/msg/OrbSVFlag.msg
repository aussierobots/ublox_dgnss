# Bit field structure for svFlag
uint8 HEALTH_UNKNOWN = 0
uint8 HEALTH_HEALTHY = 1
uint8 HEALTH_NOT_HEALTHY = 2
uint8 health          # SV health

uint8 VISIBILITY_UNKNOWN = 0
uint8 VISIBILITY_BELOW_HORIZON = 1
uint8 VISIBILITY_ABOVE_HORIZON = 2
uint8 VISIBILITY_ABOVE_ELEVATION_MASK = 3
uint8 visibility       # SV visibility