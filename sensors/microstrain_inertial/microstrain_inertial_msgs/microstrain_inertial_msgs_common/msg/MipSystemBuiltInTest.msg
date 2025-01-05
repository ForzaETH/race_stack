# Message definition for the MIP field https://files.microstrain.com/GQ7+User+Manual/external_content/dcp/Data/0xa0/data/0x01.htm

# Header containing common information
#   header.frame_id has no meaning in this message
microstrain_inertial_msgs/MipHeader header

# Device-specific bitfield (128 bits).
# See device user manual.
# Bits are least-significant-byte first.
# For example, bit 0 is located at bit 0 of result[0], bit 1 is located at bit 1 of result[0], bit 8 is located at bit 0 of result[1], and bit 127 is located at bit 7 of result[15].
uint8[16] result