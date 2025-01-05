# Message definition for the MIP field https://s3.amazonaws.com/files.microstrain.com/GQ7+User+Manual/external_content/dcp/Data/filter_data/data/mip_field_filter_aiding_measurement_summary.htm

# Header containing common information
#   header.frame_id has no meaning in this message
microstrain_inertial_msgs/MipHeader header

# GPS time of week (seconds)
float64 time_of_week

# Source used when type is set to TYPE_GNSS to specify which GNSS module this message is for
uint8 source

# Type of the aiding measurement. See TYPE_* enums for possible values
uint8 type

# Parsed version of indicator bitfield
microstrain_inertial_msgs/MipFilterAidingMeasurementSummaryIndicator indicator

# Possible values for type
uint8 TYPE_GNSS         = 1
uint8 TYPE_DUAL_ANTENNA = 2
uint8 TYPE_HEADING      = 3
uint8 TYPE_PRESSURE     = 4
uint8 TYPE_MAGNETOMETER = 5
uint8 TYPE_SPEED        = 6