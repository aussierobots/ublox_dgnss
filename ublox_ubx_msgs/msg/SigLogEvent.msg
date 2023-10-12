# Constants for Detection Types
uint8 DETECTION_SIMULATED_SIGNAL = 0
uint8 DETECTION_ABNORMAL_SIGNAL = 1
uint8 DETECTION_INS_GNSS_MISMATCH = 2
uint8 DETECTION_ABRUPT_CHANGES = 3
uint8 DETECTION_BROADBAND_JAMMING = 4
uint8 DETECTION_NARROWBAND_JAMMING = 5

# Constants for Event Types
uint8 EVENT_STARTED = 0
uint8 EVENT_STOPPED = 1
uint8 EVENT_TRIGGERED = 2
uint8 EVENT_TIMED_OUT = 3

# Fields
uint32 time_elapsed    # s - Seconds elapsed since this event. Special value 0xFFFFFFFF: more than 45 days
uint8 detection_type   # Type of the spoofing or jamming detection
uint8 event_type       # Type of the event
# uint8[2] reserved_1    # Reserved