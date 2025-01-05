## Description

Collection of simple examples to get started using the `microstrain_inertial_driver`.

## Table of Contents

* [Common Steps](#common-steps)
* [Device Examples](#device-examples)
    * [CV7-INS / GV7-INS](#cv7-ins--gv7-ins)
    * [CV7 / GV7](#cv7--gv7)
    * [GQ7](#gq7)
    * [GX5-GNSS / CX5-GNSS](#gx5-gnss--cx5-gnss)
    * [GX5-AHRS / CX5-AHRS](#gx5-ahrs--cx5-ahrs)
    * [GX5-AR / CX5-AR](#gx5-ar--cx5-ar)
* [Other Examples](#other-examples)
    * [CV7-INS / GV7-INS with NMEA over aux](#cv7-ins--gv7-ins-with-nmea-over-aux-port)

## Common Steps

For all the below examples, you should make sure you take the following steps before running them:

1. Install the [udev rules](../README.md#udev-rules) (This should only be done once and if using the ROS package index, these will be installed automatically)
2. Connect your device.
3. Modify the `port` and `baudrate` parameters in the `.yml` file for the example you are running to point to the port
your device is connected to. This is especially important if using serial instead of USB.
4. [Install or build](../README.md#install-instructions) the `microstrain_inertial_examples` and `microstrain_inertial_driver` packages
5. Source your `setup.bash`
    1. If building from source, run: `source install/setup.bash` after building
    2. If installing from the ROS package index, run: `source /opt/ros/<ROS Version>/setup.bash` after installing

## Device Examples

This package contains some example launch files to show users a simple starting configuration when using different devices.

### CV7-INS / GV7-INS

**NOTE**: This example works for the [3DM-CV7-INS](https://www.microstrain.com/inertial-sensors/3DM-CV7-INS), and
[3DM-GV7-INS](https://www.microstrain.com/inertial-sensors/3DM-GV7-INS)

The CV7-INS example contains a simple example of outputting relative position data from a CV7-INS and displaying that data in rviz.
This example also shows how one would input external position and velocity into the sensor.

To run this example, run the following command after doing the [common steps](#common-steps):

```bash
ros2 launch microstrain_inertial_examples cv7_ins_launch.py
```

This example contains the following files:

* [`launch/cv7_ins_launch.py`](./launch/cv7_ins_launch.py)
* [`config/cv7_ins/cv7_ins.yml`](./config/cv7_ins/cv7_ins.yml)
* [`config/cv7_ins/display.rviz`](./config/cv7_ins/display.rviz)

### CV7 / GV7

**NOTE**: This example works for the [3DM-CV7-AR](https://www.microstrain.com/inertial-sensors/3dmcv7-ar),
[3DM-CV7-AHRS](https://www.microstrain.com/inertial-sensors/3dm-cv7-ahrs),
[3DM-GV7-AR](https://www.microstrain.com/inertial-sensors/3DM-GV7-AR), and
[3DM-GV7-AHRS](https://www.microstrain.com/inertial-sensors/3DM-GV7-AHRS).

The CV7 example contains a simple example of outputting
orientation, angular and linear velocity from a CV7 and displaying that data in rviz.

To run this example, run the following command after doing the [common steps](#common-steps):

```bash
ros2 launch microstrain_inertial_examples cv7_launch.py
```

This example contains the following files:

* [`launch/cv7_launch.py`](./launch/cv7_launch.py)
* [`config/cv7/cv7.yml`](./config/cv7/cv7.yml)
* [`config/cv7/display.rviz`](./config/cv7/display.rviz)

### GQ7

**NOTE**: This example works for the [3DM-GQ7](https://www.microstrain.com/inertial-sensors/3dm-gq7)

The GQ7 example contains a simple example of outputting
global and relative position data from a GQ7 and displaying that data in rviz.
This example also assumes that the user will be using a [3DM-RTK](https://www.microstrain.com/inertial-sensors/3dm-rtk)
connected to the GQ7 aux port, but can be tweaked to not require the 3DM-RTK.

To run this example

1. Do the [common steps](#common-steps)
2. Ensure that your GQ7 has both antennas connected. See the microstrain
[Antenna Installation](https://s3.amazonaws.com/files.microstrain.com/GQ7+User+Manual/user_manual_content/installation/Antenna.htm)
page for more help with this.
3. Update `gnss1_antenna_offset` and `gnss2_antenna_offset` in `config/gq7/gq7.yml` to your antenna offsets
4. Run the following command:
```bash
ros2 launch microstrain_inertial_examples gq7_launch.py
```

This example contains the following files:

* [`launch/gq7_launch.py`](./launch/gq7_launch.py)
* [`config/gq7/gq7.yml`](./config/gq7/gq7.yml)
* [`config/gq7/display.rviz`](./config/gq7/display.rviz)

### GX5-GNSS / CX5-GNSS

**NOTE**: This example works for the [3DM-GX5-GNSS](https://www.microstrain.com/inertial-sensors/3DM-GX5-45) (3DM-GX5-45), and
[3DM-CX5-45](https://www.microstrain.com/inertial-sensors/3dm-cx5-45) (3DM-CX5-45)

The GX5-GNSS example contains a simple example of outputting global and relative position data from a GX5-GNSS
and displaying that data in rviz.

1. Do the [common steps](#common-steps)
2. Ensure that your GX5-GNSS has it's antenna connected. See the microstrain
[Antenna Installation](https://s3.amazonaws.com/files.microstrain.com/GQ7+User+Manual/user_manual_content/installation/Antenna.htm)
page for more help with this.
3. Update `gnss1_antenna_offset` in `config/gx5_45/gx5_45.yml` to your antenna offset
4. Run the following command:
```bash
ros2 launch microstrain_inertial_examples gx5_45_launch.py
```

### GX5-AHRS / CX5-AHRS

**NOTE**: This example works for the [3DM-GX5-AHRS](https://www.microstrain.com/inertial-sensors/3DM-GX5-25) (3DM-GX5-25), and
[3DM-CX5-AHRS](https://www.microstrain.com/inertial-sensors/3dm-cx5-25) (3DM-CX5-25)

The GX5-AHRS example contains a simple example of outputting
orientation, angular and linear velocity from a GX5-AHRS and displaying that data in rviz.

To run this example, run the following command after doing the [common steps](#common-steps):

```bash
ros2 launch microstrain_inertial_examples gx5_25_launch.py
```

This example contains the following files:

* [`launch/gx5_25_launch.py`](./launch/gx5_25_launch.py)
* [`config/gx5_25/gx5_25.yml`](./config/gx5_25/gx5_25.yml)
* [`config/gx5_25/display.rviz`](./config/gx5_25/display.rviz)

### GX5-AR / CX5-AR

**NOTE**: This example works for the [3DM-GX5-AR](https://www.microstrain.com/inertial-sensors/3DM-GX5-15) (3DM-GX5-15), and
[3DM-CX5-AR](https://www.microstrain.com/inertial-sensors/3dm-cx5-15) (3DM-CX5-15)

The GX5-AR example contains a simple example of outputting
orientation, angular and linear velocity from a GX5-AR and displaying that data in rviz.

To run this example, run the following command after doing the [common steps](#common-steps):

```bash
ros2 launch microstrain_inertial_examples gx5_15_launch.py
```

This example contains the following files:

* [`launch/gx5_15_launch.py`](./launch/gx5_15_launch.py)
* [`config/gx5_15/gx5_15.yml`](./config/gx5_15/gx5_15.yml)
* [`config/gx5_15/display.rviz`](./config/gx5_15/display.rviz)

## Other Examples

Other assorted examples users may find useful

### CV7-INS / GV7-INS with NMEA over aux port

**NOTE**: This example works for the [3DM-CV7-INS](https://www.microstrain.com/inertial-sensors/3DM-CV7-INS), and
[3DM-GV7-INS](https://www.microstrain.com/inertial-sensors/3DM-GV7-INS) and requires an external GNSS receiver such as the
[SparkFun ZED-F9P](https://www.sparkfun.com/products/16481?gad_source=1&gclid=CjwKCAjwps-zBhAiEiwALwsVYTSItzsET_3hKDL1FDr5R1byfLifyTtszQVOrpOVjDAC2XtMhd6FwxoCXjEQAvD_BwE)

This example is similar to the [CV7-INS / GV7-INS](#cv7-ins--gv7-ins), but instead of getting the data over the ROS network,
the data will be provided by GPIO pins setup as an "aux" port on the device. This example follows the wiring setup provided in
the [NMEA Input](https://mip-documentation-test.s3.ca-central-1.amazonaws.com/CV7-INS+Online/user_manual_content/additional_features/NMEA%20Input.htm)
page on the MicroStrain manual.

To run this example, run the following command after doing the [common steps](#common-steps):

```bash
ros2 launch microstrain_inertial_examples cv7_ins_nmea_aux_launch.py
```

This example contains the following files:

* [`launch/cv7_ins_nmea_aux_launch.py`](./launch/cv7_ins_nmea_aux_launch.py)
* [`config/cv7_ins_nmea_aux/cv7_ins.yml`](./config/cv7_ins_nmea_aux/cv7_ins.yml)
* [`config/cv7_ins_nmea_aux/display.rviz`](./config/cv7_ins_nmea_aux/display.rviz)