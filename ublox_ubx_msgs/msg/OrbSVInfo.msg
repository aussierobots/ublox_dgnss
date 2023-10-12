# Information for each Satellite Vehicle (SV) in the GNSS orbit database
uint8 gnss_id  # GNSS ID
uint8 sv_id    # Satellite ID

OrbSVFlag sv_flag # Information Flags for SV
OrbEphInfo eph    # Ephemeris data
OrbAlmInfo alm    # Almanac data
OtherOrbInfo other_orb # Other orbit data available