# Message definition for the MIP field https://s3.amazonaws.com/files.microstrain.com/GQ7+User+Manual/external_content/dcp/Data/gnss_recv_1/data/mip_field_gnss_rf_error_detection.htm

# Header containing common information
#   header.frame_id has no meaning in this message
microstrain_inertial_msgs/MipHeader header

# RF Band of the reported information. See RF_BAND_* enums for possible values
uint8 rf_band

# GNSS Jamming State (as reported by the GNSS module). See JAMMING_STATE_* enums for possible values
uint8 jamming_state

# GNSS Spoofing State (as reported by the GNSS module). See SPOOFING_STATE_* enums for possible values
uint8 spoofing_state

# Enum values for rf_band field
uint8 RF_BAND_UNKNOWN = 0
uint8 RF_BAND_L1      = 1
uint8 RF_BAND_L2      = 2
uint8 RF_BAND_L5      = 5

# Enum values for jamming_state field
uint8 JAMMING_STATE_UNKNOWN     = 0
uint8 JAMMING_STATE_NONE        = 1
uint8 JAMMING_STATE_PARTIAL     = 2
uint8 JAMMING_STATE_SIGNIFICANT = 3

# Enum values for spoofing_state field
uint8 SPOOFING_STATE_UNKNOWN     = 0
uint8 SPOOFING_STATE_NONE        = 1
uint8 SPOOFING_STATE_PARTIAL     = 2
uint8 SPOOFING_STATE_SIGNIFICANT = 3