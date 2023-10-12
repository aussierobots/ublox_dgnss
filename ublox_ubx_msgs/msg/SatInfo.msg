# Information for each Satellite Vehicle
uint8 gnss_id  # GNSS identifier
uint8 sv_id    # Satellite identifier
uint8 cno      # dBHz - Carrier to noise ratio (signal strength)
int8 elev      # deg - Elevation (range: +/-90), unknown if out of range
int16 azim     # deg - Azimuth (range 0-360), unknown if elevation is out of range
int16 pr_res   # Pseudorange residual (scaled by 0.1)

SatFlags flags # Bitmask with satellite information