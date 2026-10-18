# Car state in Frenet coordinates along the global raceline
std_msgs/Header header
float64 s   # [m] progress along the raceline
float64 d   # [m] lateral offset from the raceline, positive to the left
float64 vs  # [m/s] velocity along the raceline
float64 vd  # [m/s] lateral velocity, positive to the left
