# Message definition for the Dongle Status field of https://s3.amazonaws.com/files.microstrain.com/GQ7+User+Manual/external_content/dcp/Data/gnss_corrections/data/mip_field_gnss_rtk_corrections_status.htm?Highlight=rtk%20status
#   Note: This message will never be published on it's own, only included in other messages.

# Modem State bits from the Dongle Status field. See MODEM_STATE_* enums for possible values
uint8 modem_state

# Connection Type bits from the Dongle Status field. See CONNECTION_TYPE_* enums for possible values
uint8 connection_type

# RSSI bits from the Dongle Status field. Units are in dBm
int8 rssi

# Signal Quality bits from the Dongle Status field.
# 0-10 indication of signal quality, 0 means unavailable/invalid/not connected, 1 is poor, 10 is excellent.
uint8 signal_quality

# Tower Change Indicator bits from the Dongle Status field.
# 4-bit value that increments each time a cell tower change occurs. Rolls over from 15 to 0.
uint8 tower_change_indicator

# NMEA Timeout Flag bit from the Dongle Status field.
# The device has not received a valid NMEA message from the GQ7 in the past 3 seconds
bool nmea_timeout_flag

# Server Timeout Flag bit from the Dongle Status field.
# The device has not received any communications with the server in the past 3 seconds.
bool server_timeout_flag

# RTCM Timeout Flag bit from the Dongle Status field.
# The device has not received a valid RTCM message from the server in the past 3 seconds.
bool rtcm_timeout_flag

# Device Out of Range Flag bit from the Dongle Status field
# The device's reported position is beyond the server-side configured distance to a base station and will not receive corrections
bool device_out_of_range_flag

# Corrections Unavailable Flag bit from the Dongle Status field
# The server is reporting that corrections are not available given the user's account settings
bool corrections_unavailable_flag

# Possible values for dongle_modem_state
uint8 MODEM_STATE_OFF = 0
uint8 MODEM_STATE_NO_NETWORK = 1
uint8 MODEM_STATE_NETWORK_CONNECTED = 2
uint8 MODEM_STATE_CONFIGURING_DATA_CONTEXT = 3
uint8 MODEM_STATE_ACTIVATING_DATA_CONTEXT = 4
uint8 MODEM_STATE_CONFIGURING_SOCKET = 5
uint8 MODEM_STATE_WAITING_ON_SERVER_HANDSHAKE = 6
uint8 MODEM_STATE_CONNECTED_AND_IDLE = 7
uint8 MODEM_STATE_CONNECTED_AND_STREAMING = 8

# Possible values for dongle_connection_type
uint8 CONNECTION_TYPE_NO_CONNECTION = 0
uint8 CONNECTION_TYPE_CONNECTION_2G = 2
uint8 CONNECTION_TYPE_CONNECTION_3G = 3
uint8 CONNECTION_TYPE_CONNECTION_4G = 4
uint8 CONNECTION_TYPE_CONNECTION_5G = 5