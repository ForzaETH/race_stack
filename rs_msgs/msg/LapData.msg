std_msgs/Header header
int32   lap_count
float64 lap_time                    # [s]
float64 mean_abs_d                  # [m] mean lateral error to the raceline
float64 max_abs_d                   # [m] max lateral error to the raceline
float64 min_boundary_dist           # [m] closest approach to a wall
float64 mean_vs                     # [m/s] 
float64 max_vs                      # [m/s]
rs_msgs/SectorData[] sectors
