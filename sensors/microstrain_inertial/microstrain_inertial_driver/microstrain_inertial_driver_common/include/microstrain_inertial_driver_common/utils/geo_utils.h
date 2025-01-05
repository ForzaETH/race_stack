#ifndef MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_GEO_UTILS_H
#define MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_GEO_UTILS_H

#include "microstrain_inertial_driver_common/utils/ros_compat.h"

namespace microstrain
{

static inline tf2::Matrix3x3 ecefToNedTransform(double lat_deg, double lon_deg)
{
  const double lat = lat_deg * (M_PI / 180);
  const double lon = lon_deg * (M_PI / 180);
  return tf2::Matrix3x3(-sin(lat) * cos(lon),  -sin(lat) * sin(lon),  cos(lat), 
                        -sin(lon),             cos(lon),              0,
                        -cos(lat) * cos(lon),  -cos(lat) * sin(lon),  -sin(lat));
}

static inline tf2::Matrix3x3 ecefToEnuTransform(double lat_deg, double lon_deg)
{
  const tf2::Matrix3x3 ned_to_enu(
    0, 1, 0,
    1, 0, 0,
    0, 0, -1
  );
  return ned_to_enu * ecefToNedTransform(lat_deg, lon_deg);
}

static inline tf2::Quaternion ecefToEnuTransformQuat(double lat, double lon)
{
  tf2::Quaternion ecef_to_enu_quat;
  ecefToEnuTransform(lat, lon).getRotation(ecef_to_enu_quat);
  return ecef_to_enu_quat.inverse();
}

static inline tf2::Quaternion ecefToNedTransformQuat(double lat, double lon)
{
  tf2::Quaternion ecef_to_ned_quat;
  ecefToNedTransform(lat, lon).getRotation(ecef_to_ned_quat);
  return ecef_to_ned_quat.inverse();
}

}

#endif  // MICROSTRAIN_INERTIAL_DRIVER_COMMON_UTILS_GEO_UTILS_H