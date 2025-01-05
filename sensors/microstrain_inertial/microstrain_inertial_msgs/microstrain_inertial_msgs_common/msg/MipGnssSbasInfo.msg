# Message definition for the MIP field https://s3.amazonaws.com/files.microstrain.com/GQ7+User+Manual/external_content/dcp/Data/gnss_recv_1/data/mip_field_gnss_sbas_info.htm

# Header containing common information
#   header.frame_id has no meaning in this message
microstrain_inertial_msgs/MipHeader header

# GPS time of week (seconds)
float64 time_of_week

# GPS weeks since 1980 (weeks)
uint16 week_number

# SBAS system if. See SBAS_SYSTEM_* enums for possible values
uint8 sbas_system

# SBAS satellite id
uint8 sbas_id

# Number of SBAS corrections
uint8 count

# Parsed out version of the SBAS status bitfield
microstrain_inertial_msgs/MipGnssSbasInfoSbasStatus sbas_status

# Possible values for the sbas_system field
uint8 SBAS_SYSTEM_UNKNOWN = 0
uint8 SBAS_SYSTEM_WAAS    = 1
uint8 SBAS_SYSTEM_EGNOS   = 2
uint8 SBAS_SYSTEM_MSAS    = 3
uint8 SBAS_SYSTEM_GAGAN   = 4