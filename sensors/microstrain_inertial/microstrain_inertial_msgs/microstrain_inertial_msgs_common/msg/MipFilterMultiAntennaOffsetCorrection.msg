# Message definition for the MIP field https://s3.amazonaws.com/files.microstrain.com/GQ7+User+Manual/external_content/dcp/Data/filter_data/data/mip_field_filter_multi_antenna_offset_correction.htm

# Header containing common information
#   header.frame_id has no meaning in this message
microstrain_inertial_msgs/MipHeader header

# Receiver ID. For GQ7, this will be 1 for GNSS1 and 2 for GNSS2
uint8 receiver_id

# Offset (x, y, z) in meters
float32[3] offset