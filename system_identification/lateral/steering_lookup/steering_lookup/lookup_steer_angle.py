import numpy as np
from forza_config_ros import load_steering_lut_ros

def find_nearest(array, value):
    # array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return array[idx], idx

def find_closest_neighbors(array, value):
    # index of first nan
    is_nan_array = np.argwhere(np.isnan(array))
    if len(is_nan_array) > 0:
      first_nan = is_nan_array[0][0]
      array = array[0:first_nan]
    closest, closest_idx = find_nearest(array, value)
    if closest_idx == 0:
        return array[0], 0, array[0], 0
    elif closest_idx == (len(array) - 1):
        return array[closest_idx], closest_idx, array[closest_idx], closest_idx
    else:
      second_closest, second_idx = \
        find_nearest(array[[max(closest_idx-1, 0), 
        min(closest_idx+1, len(array)-1)]], value)
      second_idx = -1 + closest_idx + 2 * second_idx
      return closest, closest_idx, second_closest, second_idx


class LookupSteerAngle:
    """
    LookupSteerAngle: steering angle for a lateral acceleration at a velocity,
    from the steering LUT of the car and floor in stack_master/config
    """
    def __init__(self, racecar_version, floor, logger):
      self.lu = load_steering_lut_ros(racecar_version, floor)
      self.logger = logger

    def lookup_steer_angle(self, accel, vel):
        """
        lookup_steer_angle:
        """
        if accel > 0.0:
          sign_accel = 1.0
        else:
          sign_accel = -1.0
        # lookup only for positive accelerations
        accel = abs(accel)
        lu_vs = self.lu[0, 1:]
        lu_steers = self.lu[1:, 0]
        
        #if (vel > lu_vs[-1]):
        # self.logger(5, "Velocity exceeds lookup table, generating steering angle for v :" + str(lu_vs[-1]))

        # find closest velocities to vel
        c_v, c_v_idx = find_nearest(lu_vs, vel)

        # find two closest accelerations to accel
        c_a, c_a_idx, s_a, s_a_idx = find_closest_neighbors(self.lu[1:, c_v_idx + 1], accel)
        if c_a_idx == s_a_idx:
          steer_angle = lu_steers[c_a_idx]
        else :
          # interpolate between two closest accelerations to find steering angle
          # np.interp needs increasing sample points; the second closest may lie below the closest
          (a_lo, steer_lo), (a_hi, steer_hi) = sorted([(c_a, lu_steers[c_a_idx]), (s_a, lu_steers[s_a_idx])])
          steer_angle = np.interp(accel, [a_lo, a_hi], [steer_lo, steer_hi])
        return steer_angle * sign_accel

if __name__ == "__main__":
  detective =  LookupSteerAngle("NUC6", "glc", print)
  steer_angle = detective.lookup_steer_angle(9, 7)
  print(steer_angle)