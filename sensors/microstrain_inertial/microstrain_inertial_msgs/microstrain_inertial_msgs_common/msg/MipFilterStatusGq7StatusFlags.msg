# Message definition for the Status Flags (GQ7, CV7, etc.) field of https://s3.amazonaws.com/files.microstrain.com/GQ7+User+Manual/external_content/dcp/Data/filter_data/data/mip_field_filter_status.htm?Highlight=filter%20status
#   Note: This message will never be published on it's own, only included in other messages.

uint8 filter_condition  # See FILTER_CONDITION_* enums for possible values
bool  roll_pitch_warning
bool  heading_warning
bool  position_warning
bool  velocity_warning
bool  imu_bias_warning
bool  gnss_clk_warning
bool  antenna_lever_arm_warning
bool  mounting_transform_warning
bool  time_sync_warning  # No time synchronization pulse detected
bool  solution_error  # This includes all bits between 12 and 15, if any of them are set there is a GQ7 solution error

# Possible values for filter_condition when the connected device is a prospect device (GQ7, CV7, etc)
uint8 FILTER_CONDITION_STABLE     = 1
uint8 FILTER_CONDITION_CONVERGING = 2
uint8 FILTER_CONDITION_UNSTABLE   = 3