# Message definition for the MIP field https://files.microstrain.com/GQ7+User+Manual/external_content/dcp/Data/0x80/data/0x14.htm

# Header containing common information
#   header.frame_id has no meaning in this message
microstrain_inertial_msgs/MipHeader header

float32 min_temp   # Degrees Celsius
float32 max_temp   # Degrees Celsius
float32 mean_temp  # Degrees Celsius