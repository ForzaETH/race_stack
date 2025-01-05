# Microstrain Inertial Driver v4.0 Detailed changes

This document outlines the changes to the microstrain inertial driver for version 4.0.0, as well as the reasoning behind the changes.

## Reasoning

The `microstrain_inertial_driver` has seen more widespread adoption as a replacement or compliment to existing ROS packages such as
`robot_localization`. As we have worked with customers to get things working with their systems, it is clear that a decent amount of work is required
in order to integrate the `microstrain_inertial_driver`. However, most of the pain points in this integration are in the messages we have chosen to publish,
what we publish in these messages, and how this does not match up with what the ROS standard tools expect.

Specifically, nodes like `move_base`, and the ROS2 `nav2` stack should integrate easily with `microstrain_inertial`

## Publishers

A big pain point for customers is in our published messages. After doing some research into other standard ROS tools, as well as talking to some customers,
it seems to make the most sense for the driver to publish the following ROS messages on the associated topics:

### Standard Message Types
| Topic                      | Message Type                                                                                                                         | Devices                                                | Description                                                                             | Reference(s)                                                 |
| -------------------------- | ------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------------ | --------------------------------------------------------------------------------------- | ------------------------------------------------------------ |
| `imu/mag`                  | [`sensor_msgs/MagneticField`](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/MagneticField.html)                            | Devices with a magnetometer                            | Raw magnetometer data                                                                   | [REP-1045](https://www.ros.org/reps/rep-0145.html)           |
| `imu/data`                 | [`sensor_msgs/Imu`](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html)                                                | Devices with an IMU                                    | Raw IMU measurements for linear acceleration, angular velocity, and orientation.        | [REP-1045](https://www.ros.org/reps/rep-0145.html)           |
| `imu/pressure`             | [`sensor_msgs/FluidPressure`](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/FluidPressure.html)                            | Devices with a barometer                               | Raw pressure measurement from sensor                                                    | None                                                         |
| `imu/wheel_speed`          | [`geometry_msgs/TwistWithCovarianceStamped`](http://docs.ros.org/en/api/geometry_msgs/html/msg/TwistWithCovarianceStamped.html)      | Devices with the ability to input wheel speed          | External wheel speed measurements input into the sensor                                 | None                                                         |
| `gnss_1/llh_position`      | [`sensor_msgs/NavSatFix`](http://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html)                                            | Devices with at least 1 GPS antenna                    | Raw GPS LLH position information from antenna 1.                                        | [nmea_navsat_driver](http://wiki.ros.org/nmea_navsat_driver) |
| `gnss_1/velocity`          | [`geometry_msgs/TwistWithCovarianceStamped`](http://docs.ros.org/en/api/geometry_msgs/html/msg/TwistWithCovarianceStamped.html)      | Devices with at least 1 GPS antenna                    | Raw GPS velocity information from antenna 1 in ENU / NED depending on `use_enu_frame`.  | [nmea_navsat_driver](http://wiki.ros.org/nmea_navsat_driver) |
| `gnss_1/velocity_ecef`     | [`geometry_msgs/TwistWithCovarianceStamped`](http://docs.ros.org/en/api/geometry_msgs/html/msg/TwistWithCovarianceStamped.html)      | Devices with at least 1 GPS antenna                    | Raw GPS velocity information from antenna 1 in ECEF.                                    | None                                                         |
| `gnss_1/odometry_earth`    | [`nav_msgs/Odometry`](http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Odometry.html)                                            | Devices with at least 1 GPS antenna                    | Raw ECEF position data in the earth frame from antenna 1.                               | [REP-105](https://www.ros.org/reps/rep-0105.html#earth)      |
| `gnss_1/time`              | [`sensor_msgs/TimeReference`](http://docs.ros.org/en/api/sensor_msgs/html/msg/TimeReference.html)                                    | Devices with at least 1 GPS antenna                    | GPS timestamp data from antenna 2.                                                      | [nmea_navsat_driver](http://wiki.ros.org/nmea_navsat_driver) |
| `gnss_2/llh_position`      | [`sensor_msgs/NavSatFix`](http://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html)                                            | Devices with at least 2 GPS antenna                    | Raw GPS LLH position information from antenna 2.                                        | [nmea_navsat_driver](http://wiki.ros.org/nmea_navsat_driver) |
| `gnss_2/velocity`          | [`geometry_msgs/TwistWithCovarianceStamped`](http://docs.ros.org/en/api/geometry_msgs/html/msg/TwistWithCovarianceStamped.html)      | Devices with at least 2 GPS antenna                    | Raw GPS velocity information from antenna 2 in ENU / NED depending on `use_enu_frame`.  | [nmea_navsat_driver](http://wiki.ros.org/nmea_navsat_driver) |
| `gnss_2/velocity_ecef`     | [`geometry_msgs/TwistWithCovarianceStamped`](http://docs.ros.org/en/api/geometry_msgs/html/msg/TwistWithCovarianceStamped.html)      | Devices with at least 2 GPS antenna                    | Raw GPS velocity information from antenna 2 in ECEF.                                    | None                                                         |
| `gnss_2/odometry_earth`    | [`nav_msgs/Odometry`](http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Odometry.html)                                            | Devices with at least 2 GPS antenna                    | Raw ECEF position data in the earth frame from antenna 2.                               | [REP-105](https://www.ros.org/reps/rep-0105.html#earth)      |
| `gnss_2/time`              | [`sensor_msgs/TimeReference`](http://docs.ros.org/en/api/sensor_msgs/html/msg/TimeReference.html)                                    | Devices with at least 2 GPS antenna                    | GPS timestamp data from antenna 2.                                                      | [nmea_navsat_driver](http://wiki.ros.org/nmea_navsat_driver) |
| `ekf/imu/data`             | [`sensor_msgs/Imu`](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html)                                                | Devices with an IMU and and EKF                        | Filtered IMU measurements for linear acceleration, angular velocity, and orientation.   | [REP-1045](https://www.ros.org/reps/rep-0145.html)           |
| `ekf/llh_position`         | [`sensor_msgs/NavSatFix`](http://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html)                                            | Devices with an EKF and the ability to output position | Filtered LLH position information from the EKF                                          | [nmea_navsat_driver](http://wiki.ros.org/nmea_navsat_driver) |
| `ekf/velocity`             | [`geometry_msgs/TwistWithCovarianceStamped`](http://docs.ros.org/en/api/geometry_msgs/html/msg/TwistWithCovarianceStamped.html)      | Devices with an EKF                                    | Filtered Velocity information from the EKF in ENU / NED depending on `use_enu_frame`    | [nmea_navsat_driver](http://wiki.ros.org/nmea_navsat_driver) |
| `ekf/velocity_ecef`        | [`geometry_msgs/TwistWithCovarianceStamped`](http://docs.ros.org/en/api/geometry_msgs/html/msg/TwistWithCovarianceStamped.html)      | Devices with an EKF                                    | Filtered Velocity information from the EKF in ECEF                                      | [nmea_navsat_driver](http://wiki.ros.org/nmea_navsat_driver) |
| `ekf/odometry_earth`       | [`nav_msgs/Odometry`](http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Odometry.html)                                            | Devices with an EKF and the ability to output position | Filtered ECEF odometry data in the earth frame from the EKF.                            | [REP-105](https://www.ros.org/reps/rep-0105.html#earth)      |
| `ekf/odometry_map`         | [`nav_msgs/Odometry`](http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Odometry.html)                                            | Devices with an EKF and the ability to output position | Filtered position data in `map_frame_id` from the EKF.                                  | [REP-105](https://www.ros.org/reps/rep-0105.html#earth)      |
| `ekf/dual_antenna_heading` | [`geometry_msgs/PoseWithCovarianceStamped`](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html) | Devices with at least 2 GPS antenna                    | Dual antenna heading                                                                    | None                                                         |

### Human Readable Status

The topic `ekf/status` publishes a message of type `microstrain_inertial_msgs/HumanReadableStatus`.

This message is the only custom message that is not exactly mapped to one MIP field.
The purpose of this message is to provide an easy way to check the status of the device using `rostopic echo` or `ros2 topic echo`.
Users **SHOULD NOT** attempt to use this in their production code. It is for debug purposes only


### MIP messages

The remaining messages are published on topics using the format `mip/<descriptor_set_name>/<field_descriptor_name>`.
The contents of the messages should be exactly the same as the MIP fields they represent with one exception.
They will all contain a `microstrain_inertial_msgs/MipHeader` which will contain a `std_msgs/Header` as well as several MIP shared fields.

For more information on where `descriptor_set_name` and `field_descriptor_name` come from, see [MIP Protocol](https://s3.amazonaws.com/files.microstrain.com/GQ7+User+Manual/dcp_content/introduction/MIP%20Protocol.htm).
For more information on shared fields see [Shared Data Descriptors](https://s3.amazonaws.com/files.microstrain.com/GQ7+User+Manual/dcp_content/introduction/Shared%20Data%20Descriptors.htm)

## Transforms

**NOTE**: The frame IDs mentioned below are the defaults. They can be changed via parameters

Previously, the driver provided a single transform between two frames which did not have sensible defaults for most ROS systems.
In this new version of the driver we attempted to fix that by publishing transforms to more standard frames as well as providing more transforms

### Transform modes

The driver now has the ability to run in 3 modes with respect to transform publishing.
This is controlled by the `tf_mode` parameter.

| Name     | `tf_mode` | Description                                                                                                                                                                                                                   |
| -------- | --------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Off      | 0         | No dynamic transforms will be published. Things like antenna offsets and other independently configured frames will still be published                                                                                        |
| Global   | 1         | The transform between `earth` and `imu_link` will be published using the ECEF position produced by the device                                                                                                                 |
| Relative | 2         | The transform between `earth` and `map` will be published using relative position configuration. The transform between `map` and `base_link` will then be published using that offset and the position produced by the device |

### Published transforms

The following transforms may be published between the following frames depending on the tf_mode described above:

| Frame ID   | Child Frame ID              | Condition                   | Description                                                                                                                                                                                                                                                                            |
| ---------- | --------------------------- | --------------------------- | ------------------------------------------------------------------------------------------------------------------------ |
| `earth`    | `imu_link`                  | `tf_mode == 1`              | Global position of the device in ECEF                                                                                    |
| `earth`    | `map`                       | `tf_mode == 2`              | Global position of the map frame in ECEF. Rotation from ECEF to NED/ENU will be handled automatically by this transform. |
| `map`      | `base_link`                 | `tf_mode == 2`              | Relative position WRT the map frame of the `base_link` frame                                                             |
| `imu_link` | `gnss_1_antenna_link`       | Device has 1 GNSS antenna   | Location of GNSS antenna WRT the device                                                                                  |
| `imu_link` | `gnss_2_antenna_link`       | Device has 2 GNSS antennas  | Location of GNSS antenna WRT the device                                                                                  |
| `imu_link` | `odometer_link`             | Device has odometer support | Location of odometer WRT the device                                                                                      |

## Subscibers

The following subscribers have been removed for the associated reasons:

* `/ang_zupt` -- This was an artifact from the original driver. Most customers did not use it, and the way it worked was confusing. For now it has no replacement, but if it is requested, it may be added back as a service
* `/vel_zupt` -- This was an artifact from the original driver. Most customers did not use it, and the way it worked was confusing. For now it has no replacement, but if it is requested, it may be added back as a service
* `/external_speed` -- This topic was hastily thrown together and used a custom message which was a mistake. For now it has been removed as it is a rare usecase, but could be added back with a standard message

Topic names are no longer configurable via parameters. Instead, users should use the topic remapping tools available in the ROS or ROS 2 ecosystems

## Services

The driver had many services which partially mapped to MIP commands, but in some cases didn't directly map, and were often completely unused
due to either being too confusing or just not useful. In this update, we have removed all of the services that seemed to not be useful.
The thought is that if someone needs any of the services we removed, they can be added back in a more standard way.

The remaining services are currently all directly mapped to MIP commands, and have the format `mip/<descriptor_set_name>/<field_descriptor_name>` just like publishers