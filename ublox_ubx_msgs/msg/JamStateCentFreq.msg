# Jamming State of signals related to at least one in-use signal,
# for which a sufficient amount of information is current available
# to determine if it is affected by jamming

uint32 cent_freq  # Center frequency in [kHz], floored to the nearest kHzmultiple
bool jammed       # Flag indicates whether signals on the given center frequency are considered jammed