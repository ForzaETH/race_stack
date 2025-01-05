# Message definition for the MIP field https://s3.amazonaws.com/files.microstrain.com/CV7+Online/external_content/dcp/Data/sensor_data/data/mip_field_sensor_overrange_status.htm

# Header containing common information
#   header.frame_id has no meaning in this message
microstrain_inertial_msgs/MipHeader header

# Parsed out representation of the status bitfield
microstrain_inertial_msgs/MipSensorOverrangeStatusStatus status