# Message definition for the MIP field https://s3.amazonaws.com/files.microstrain.com/GQ7+User+Manual/external_content/dcp/Data/gnss_recv_1/data/mip_field_gnss_fix_info.htm

# Header containing common information
#   header.frame_id has no meaning in this message
microstrain_inertial_msgs/MipHeader header

# Fix type. See FIX_TYPE_* enums for possible values
uint8 fix_type

# Number of satellites in view for this receiver
uint8 num_sv

# Parsed out version of the fix_flags bitfield
microstrain_inertial_msgs/MipGnssFixInfoFixFlags fix_flags

# Valid values for the fix_type field
uint8 FIX_TYPE_FIX_3D        = 0
uint8 FIX_TYPE_FIX_2D        = 1
uint8 FIX_TYPE_FIX_TIME_ONLY = 2
uint8 FIX_TYPE_FIX_NONE      = 3
uint8 FIX_TYPE_FIX_INVALID   = 4
uint8 FIX_TYPE_FIX_RTK_FLOAT = 5
uint8 FIX_TYPE_FIX_RTK_FIXED = 6