# Message definition for the MIP field https://s3.amazonaws.com/files.microstrain.com/GQ7+User+Manual/external_content/dcp/Data/gnss_corrections/data/mip_field_gnss_rtk_corrections_status.htm

# Header containing common information
#   header.frame_id has no meaning in this message
microstrain_inertial_msgs/MipHeader header

# GPS time of week (seconds)
float64 time_of_week

# GPS weeks since 1980 (weeks)
uint16 week_number

# Parsed out version of the Epoch Status bitfield
microstrain_inertial_msgs/MipGnssCorrectionsRtkCorrectionsStatusEpochStatus epoch_status

# Parsed out version of the Dongle Status bitfield
microstrain_inertial_msgs/MipGnssCorrectionsRtkCorrectionsStatusDongleStatus dongle_status

# Latency of last GPS correction (seconds)
float32 gps_correction_latency

# Latency of last GLONASS correction (seconds)
float32 glonass_correction_latency

# Latency of last Galileo correction (seconds)
float32 galileo_correction_latency

# Latency of last Beidou correction (seconds)
float32 beidou_correction_latency
