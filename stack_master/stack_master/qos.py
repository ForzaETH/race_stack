from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

# ROS1-style latching: late subscribers get the last message.
# Publisher AND subscriber must both use it, a volatile subscriber never receives the stored message.
LATCHED_QOS = QoSProfile(
    depth=1,
    history=QoSHistoryPolicy.KEEP_LAST,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)

# Sensor data, newest message only (best effort like qos_profile_sensor_data, but depth 1).
# For callbacks that can be slower than the sensor rate: they skip messages instead of working through a stale queue.
LATEST_SENSOR_QOS = QoSProfile(
    depth=1,
    history=QoSHistoryPolicy.KEEP_LAST,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)
