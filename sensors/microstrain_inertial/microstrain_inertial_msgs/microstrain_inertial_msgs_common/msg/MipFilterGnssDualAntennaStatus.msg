# Message definition for the MIP field https://s3.amazonaws.com/files.microstrain.com/GQ7+User+Manual/external_content/dcp/Data/filter_data/data/mip_field_filter_gnss_dual_antenna_status.htm

# Header containing common information
#   header.frame_id has no meaning in this message
microstrain_inertial_msgs/MipHeader header

# Last dual-antenna GNSS aiding measurement time of week (seconds)
float32 time_of_week

# Heading in radians
float32 heading

# Heading uncertainty in radians
float32 heading_unc

# Fix type indicator. See FIX_TYPE_* enums for possible values
uint8 fix_type

# Parsed out version of the Status Flags bitfield
microstrain_inertial_msgs/MipFilterGnssDualAntennaStatusStatusFlags status_flags

# Valid flags (0 = invalid, 1 = valid)
uint16 valid_flags

# Possible values for fix_type
uint8 FIX_TYPE_FIX_NONE     = 0
uint8 FIX_TYPE_FIX_DA_FLOAT = 1
uint8 FIX_TYPE_FIX_DA_FIXED = 2