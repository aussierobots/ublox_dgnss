# RawxData.msg
# UBX-RXM-RAWX (0x02 0x15) - Multi-GNSS raw measurements

# Fields
float64 pr_mes      # m - Pseudorange measurement
float64 cp_mes      # cycles - Carrier phase measurement
float32 do_mes      # Hz - Doppler measurement
uint8 gnss_id       # GNSS identifier
uint8 sv_id         # Satellite identifier
uint8 sig_id        # Signal identifier
uint8 freq_id       # Frequency slot for GLONASS
uint16 locktime     # ms - Carrier phase locktime counter
uint8 c_no          # dBHz - Carrier-to-noise density ratio
uint8 pr_stdev      # 0.01*2^n m - Estimated pseudorange standard deviation
uint8 cp_stdev      # 0.004 cycles - Estimated carrier phase standard deviation
uint8 do_stdev      # 0.002*2^n Hz - Estimated Doppler standard deviation
TrkStat trk_stat      # Tracking status bitfield
# uint8 reserved1     # Reserved