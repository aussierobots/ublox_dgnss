# this message contains a UBX-ESF-STATUS (0x10 0x10) record
# External sesnor fusion status
std_msgs/Header header # Header timestamp should be acquisition time

uint32 itow             #  ms - GPS Time of week of the navigation epoch
uint8 version           # message version (0x02 for this version)

uint8 WT_INIT_STATUS_OFF = 0
uint8 WT_INIT_STATUS_INITIALIZING = 1
uint8 WT_INIT_STATUS_INITIALIZED = 2
uint8 wt_init_status    # Wheel tick factor initialization status (0: off, 1:initializing, 2: initialized).

uint8 MBT_ALG_STATUS_OFF = 0
uint8 MBT_ALG_STATUS_INITIALIZING = 1
uint8 MBT_ALG_STATUS_INITIALIZED0 = 2
uint8 MBT_ALG_STATUS_INITIALIZED1 = 3
uint8 mnt_alg_status    # Automatic IMU-mount alignment status (0: off, 1:initializing, 2: initialized, 3: initialized).

uint8 INS_INIT_STATUS_OFF = 0
uint8 INS_INIT_STATUS_INITIALIZING = 1
uint8 INS_INIT_STATUS_INITIALIZED = 2
uint8 ins_init_status   # INS initialization status (0: off, 1: initializing, 2:initialized).

uint8 IMU_INIT_STATUS_OFF = 0
uint8 IMU_INIT_STATUS_INITIALIZING = 1
uint8 IMU_INIT_STATUS_INITIALIZED = 2
uint8 imu_init_status   # IMU initialization status (0: off, 1: initializing, 2:initialized).

uint8 FUSION_MODE_INITIALIZATION = 0
uint8 FUSION_MODE_WORKING = 1
uint8 FUSION_MODE_SUSPENDED = 2
uint8 FUSION_MODE_DISABLED = 3
uint8 fusion_mode       # 0: initialisation, 1 Fusion, 2 Suspended, 3 Disabled

uint8 num_sens          # number of sensors

ESFSensorStatus[] sensor_statuses