# Message definition for the Epoch Status field of https://s3.amazonaws.com/files.microstrain.com/GQ7+User+Manual/external_content/dcp/Data/gnss_corrections/data/mip_field_gnss_rtk_corrections_status.htm?Highlight=rtk%20status
#   Note: This message will never be published on it's own, only included in other messages.

bool antenna_location_received
bool antenna_description_received
bool gps_received
bool galileo_received
bool glonass_received
bool beidou_received
bool using_gps_msm_messages  # Using MSM messages for GPS corrections instead of RTCM messages 1001-1004
bool using_glonass_msm_messages  # Using MSM messages for GLONASS corrections instead of RTCM messages 1009-1012
bool dongle_status_read_failed  # A read of the dongle status was attempted, but failed