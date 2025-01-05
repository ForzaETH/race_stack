# Message definition for the MIP field https://s3.amazonaws.com/files.microstrain.com/GQ7+User+Manual/external_content/dcp/Data/filter_data/data/mip_field_filter_status.htm

# Header containing common information
#   header.frame_id has no meaning in this message
microstrain_inertial_msgs/MipHeader header

# Device-specific filter state. Please consult the user manual for definition. See FILTER_STATE_* enums for possible values
uint16 filter_state

# Device-specific dynamics mode. Please consult the user manual for definition. See DYNAMICS_MODE_* enums for possible values
# Note that for prospect devices (GQ7, CV7, etc), this field has a default value, and should be mostly ignored
uint16 dynamics_mode

# Parsed out version of the Status Flags bitfield when the connected device is a philo device (GX5, CX5, CV5, etc)
microstrain_inertial_msgs/MipFilterStatusGx5StatusFlags gx5_status_flags

# Parsed out version of the Status Flags bitfield when the connected device is a prospect device (GQ7, CV7, etc)
microstrain_inertial_msgs/MipFilterStatusGq7StatusFlags gq7_status_flags

# Possible values for filter_state when the connected device is a philo device (GX5, CX5, CV5, etc)
uint16 FILTER_STATE_GX5_STARTUP            = 0
uint16 FILTER_STATE_GX5_INIT               = 1
uint16 FILTER_STATE_GX5_RUN_SOLUTION_VALID = 2
uint16 FILTER_STATE_GX5_RUN_SOLUTION_ERROR = 3

# Possible values for filter_state when the connected device is a prospect device (GQ7, CV7, etc)
uint16 FILTER_STATE_GQ7_INIT      = 1
uint16 FILTER_STATE_GQ7_VERT_GYRO = 2
uint16 FILTER_STATE_GQ7_AHRS      = 3
uint16 FILTER_STATE_GQ7_FULL_NAV  = 4

# Possible values for dynamics_mode when the connected device is a philo device (GX5, CX5, CV5, etc)
uint16 DYNAMICS_MODE_GX5_PORTABLE   = 1
uint16 DYNAMICS_MODE_GX5_AUTOMOTIVE = 2
uint16 DYNAMICS_MODE_GX5_AIRBORNE   = 3

# Possible values for dynamics_mode when the connected device is a prospect device (GQ7, CV7, etc)
# Note that on older firmware the GQ7 will often report 0 for it's dynamics mode, it is strongly advised to ignore this field
uint16 DYNAMICS_MODE_GQ7_DEFAULT = 1