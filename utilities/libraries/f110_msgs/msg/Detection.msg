# One object as seen in a single LiDAR scan, in Frenet coordinates (only what was observed, else NaN)

float64 s_rear        # rear face [m] (NaN if not seen)
float64 s_front       # front face [m] (NaN if not seen)
float64 d_left        # left edge [m] (NaN if not a real edge, e.g. occluded or at the edge of the field of view)
float64 d_right       # right edge [m] (NaN if not a real edge)
float64 heading       # heading in the map frame from the L-shape fit [rad]

# Bounding box of all the object's points
float64 s_center      # [m]
float64 d_center      # [m]
float64 length        # along s [m]
float64 width         # along d [m]
