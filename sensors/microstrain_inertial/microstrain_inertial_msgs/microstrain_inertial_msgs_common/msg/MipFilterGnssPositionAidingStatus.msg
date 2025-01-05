# Message definition for the MIP field https://s3.amazonaws.com/files.microstrain.com/GQ7+User+Manual/external_content/dcp/Data/filter_data/data/mip_field_filter_gnss_pos_aid_status.htm

# Header containing common information
#   header.frame_id has no meaning in this message
microstrain_inertial_msgs/MipHeader header

# Receiver ID. For GQ7, this will be 1 for GNSS1 and 2 for GNSS2
uint8 receiver_id

# Last GNSS aiding measurement time of week (seconds)
float64 time_of_week

# Parsed out version of the status bit field
microstrain_inertial_msgs/MipFilterGnssPositionAidingStatusStatus status