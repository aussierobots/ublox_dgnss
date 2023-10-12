# TrkStat.msg
# UBX-RXM-RAWX (0x02 0x15) - Tracking status bitfield for Multi-GNSS raw measurements

# Fields
bool pr_valid       # Pseudorange valid
bool cp_valid       # Carrier phase valid
bool half_cyc       # Half cycle valid
bool sub_half_cyc   # Half cycle subtracted from phase