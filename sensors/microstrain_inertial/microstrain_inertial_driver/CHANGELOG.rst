^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package microstrain_inertial_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.5.0 (2024-11-07)
------------------
* ROS submodule update 11 07 2024 (`#363 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/363>`_)
* Contributors: Rob

4.4.0 (2024-10-07)
------------------
* ROS submodule update 10/07/2024 (`#356 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/356>`_)
* ROS submodule update 08/23/2024 (`#349 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/349>`_)
* Contributors: Rob

4.3.0 (2024-05-15)
------------------
* Updates CV7 INS example yaml (`https://github.com/LORD-MicroStrain/microstrain_inertial/pull/330` _)
* Update udev to specify manufacturer (`https://github.com/LORD-MicroStrain/microstrain_inertial/pull/327` _)
* Rename gx5_15 config file to match folder name (`https://github.com/LORD-MicroStrain/microstrain_inertial/pull/321 _`)
* Updates submodule (`#328 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/329>`_)
  * Adds ability for ROS2 implementation to be a non-lifecycle node microstrain_inertial_driver_common#68
  * Remove dongle version check microstrain_inertial_driver_common#72
  * Updates MIP SDK to fully support CV7-INS microstrain_inertial_driver_common#73
  * Waits for GNSS antenna transforms instead of erroring if they cannot be found microstrain_inertial_driver_common#74
  * Fixes the gnss_state in human readable status microstrain_inertial_driver_common#75
* Contributors: hilary-luo, GreatAlexander, robbiefish

4.2.0 (2024-04-04)
------------------
* Adds ability for node to be launched as a normal non lifecycle node (`#317 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/317>`_)
* Contributors: Rob

4.1.0 (2024-04-02)
------------------
* ROS updates microstrain_inertial_driver_common submodule (`#315 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/315>`_)
* Moves meshes and urdf files to seperate package (`#313 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/313>`_)
* Contributors: Rob

4.0.1 (2024-03-12)
------------------
* Updates microstrain_inertial_driver_common submodule (`#307 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/307>`_)
* Resets forced Cmake variables properly (`#299 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/299>`_)
* Contributors: Rob

4.0.0 (2024-01-26)
------------------
* ROS2 Standardize and update to work better with ROS standard tools (`#295 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/295>`_)
* Contributors: Rob

3.2.1 (2024-01-26)
------------------
* Fixes math to properly put the velocity into the sensor frame when using ENU (`#292 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/292>`_)
* Contributors: Rob

3.2.0 (2023-12-21)
------------------
* Updates submodule with GNSS signal configuration options (`#288 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/288>`_)
* Contributors: Rob

3.1.0 (2023-07-25)
------------------
* Adds several PRs from submodule (`#263 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/263>`_)
* Updates submodule to fix TF warning (`#262 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/262>`_)
* ROS Turns on antenna calibration by default, and publishes the amount corrected by (`#237 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/237>`_)
* Feature/ros relative position base station (`#235 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/235>`_)
* Contributors: Rob

3.0.1 (2023-02-10)
------------------
* Publishes after every packet to fix lower data rate problem (`#229 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/229>`_)
  * Publishes after every packet to fix lower data rate problem
* Updates submodule with microseconds to nanoseconds fix (`#227 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/227>`_)
* ROS Fixes odom data rate mapping to refer to the correct topic (`#224 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/224>`_)
  * Fixes odom data rate mapping to refer to the correct topic
* Contributors: Rob

3.0.0 (2023-01-11)
------------------
* Fixes for devices that do not support the extended descriptor set command in ROS (`#216 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/216>`_)
  * Fixes for devices that do not support the extended descriptor set command in ROS
* Validates that all publishers are documented on the wiki for ROS (`#211 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/211>`_)
  * Fixes time_ref topics and validates that wiki has all publishers documented
* Adds IMU overrange status publishers for ROS (`#207 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/207>`_)
  * Adds IMU overrange status publishers for ROS
* Adds RF error detection publishers for ROS (`#206 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/206>`_)
  * Adds RF error detection to ROS
* Adds SBAS info publishers for ROS (`#204 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/204>`_)
  * Adds SBAS info publishers for ROS
* Adds SBAS settings support to ROS (`#202 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/202>`_)
  * Adds SBAS settings support to ROS
* ROS Implements the filter lever arm offset command (`#196 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/196>`_)
  * Adds ability to configure filter lever arm offset
* Feature/ros nmea main port (`#192 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/192>`_)
  * Adds ability to parse and publish NMEA from the main port
* Feature/ros mip sdk (`#191 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/191>`_)
  * Converts to use the mip_sdk instead of MSCL
  * BREAKING Removes publish_* configuration options and instead relies on more granular *_data_rate options to enable/disable data streams
  * Switches to compile as a static binary
* Contributors: Rob

2.7.1 (2022-11-09)
------------------
* Updates submodule with CV7 mag aiding bugfix (`#188 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/188>`_)
* Enable usage of substitution arguments in override params YAML file. (`#187 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/187>`_)
* ROS Do not error when the device does not support antenna offset or S2V commands (`#182 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/182>`_)
  * Do not error when the device does not support antenna offset or S2V commands
* Contributors: Joey Yang, Rob

2.7.0 (2022-09-23)
------------------
* ROS2 serial improvements (`#177 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/178>`_)
  * Adds logging loop every second that will print the number of bytes read and written
  * Adds ability to configure the baudrate on the device using `set_baud`
  * Changes `*_data_rate` fields to floating point numbers to allow users to configure data rates at non whole numbers
  * Fixes bug where a quaternion would be indexed into before it was populated
* ROS2 Updates params file to note required changes for devices, and corrects incorrect documentation (`#170 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/169>`_)
* Contributors: Rob

2.6.0 (2022-05-25)
------------------
* ROS Fixes NMEA parsing to not fail when we find certain MIP packets (`#159 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/159>`_)
  * Fixes NMEA parsing to not fail when we find certain MIP packets
* Runs roslint on the buildfarm (`#154 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/154>`_)
* Fixed reporting of filter pitch and yaw when using ENU frame for ROS (`#150 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/150>`_)
* Adds velocity covarianve for the GNSS odometry message for ROS (`#149 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/149>`_)
* Adds ability to publish velocity in the vehicle frame for ROS (`#145 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/145>`_)
* ROS More granular data rates (`#131 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/131>`_)
  * Adds more granular data rates to ROS
* ROS Check supported aiding measurements (`#140 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/140>`_)
  * Checks if the device supports the requested aiding measurements before enabling/disabling
* Adds ability to switch between compensated and linear acceleration for filtered IMU (`#128 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/128>`_)
* Contributors: Lucas Walter, robbiefish

2.5.1 (2022-02-15)
------------------
* Configures GNSS Antenna offset even if publish_gnss* is False (`#124 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/124>`_)
* Contributors: robbiefish

2.5.0 (2022-01-27)
------------------
* Only attempts to publish GNSS aiding status if the pointer has been initialized
* Added RTK v2 support
  * Upgraded to MSCL 63.1.0
* Adds ability to use ROS time when populating messages
* Fixes "does not support" logs
* Fixed submodule initialization
* Contributors: dacuster, robbiefish

2.4.0 (2022-01-05)
------------------
* BREAKING: Changes default namespace from /gx5 to / which is more generic to all usecases as customers are often not using a gx5 device
* BREAKING: Changes the name argument to namespace which is more descriptive of what it was doing
* Moves most launch file parameters to yml file and allows user to override all default parameters by specifying a different yml file via the params_file argument
* Adds new argument node_name to launch file that allows users to change the name of the node
* Adds documentation for each launch file argument
* Contributors: robbiefish

2.3.0 (2021-12-22)
------------------
* BREAKING: Updates device_report_service to return the device information instead of just printing it
* Publishes Aiding Measurement Summary messages to topic nav/aiding_summary
* Publishes Fix Info messages to topic gnss1/fix_info and gnss2/fix_info
* Contributors: robbiefish

2.2.0 (2021-12-03)
------------------
* Adds set filter speed lever arm service to allow users to configure Measurement Speed Lever Arm at runtime with a service call
* Subscribes to external speed measurements
* Adds ability to configure hardware odometer at config time by sending the Odometer Settings command based on launch config
* Adds RTCM subscriber that will subscribe to RTCM corrections as mavros_msgs/RTCM messages and send them to the GQ7 through the aux port
* Adds NMEA publisher that will read NMEA sentences from the GQ7 aux port and publish them as nmea_msgs/Sentence messages to a topic
* Updates to use FACTORY_STREAMING_MERGE instead of manually casting the hex value when factory streaming is enabled
* Updates udev rules to differentiate between main and aux ports
* Contributors: ianmooreparker, robbiefish

2.1.0 (2021-11-12)
------------------
* Adds transform broadcaster that will publish transform between filter_frame_id and filter_child_frame_id
* Corrects some ENU conversions that were not being properly made
* Properly disables/enables RTK dongle based on launch config
* Publishes RTK data even when device_setup is set to false if the device was configured to send RTK data
* Contributors: ianmooreparker, robbiefish

2.0.6 (2021-10-22)
------------------
* Fixes CMake build errors experienced on the build farm
* Contributors: Rob Fisher, robbiefish

2.0.5 (2021-10-21)
------------------
* Updates maintainers and dependencies in preparation for ROS build farm
* Updates submodule to check for correct architecture
* Moves submodules to subdirectory to get bloom working
* Renames packages to be more consistent with ROS naming conventions
* Contributors: Rob Fisher, robbiefish

1.1.4 (2021-07-30)
------------------
* Installs MSCL from CMake to hopefully allow this package to be built in the buildfarm
* Merge pull request `#70 <https://github.com/LORD-MicroStrain/ROS-MSCL/issues/70>`_ from ori-drs/master
  [ros_mscl] Turn filter_data_rate and imu_data_rate into an argument
* [ros_mscl] Turn filter_data_rate and imu_data_rate into an argument
* Eliminated build warnings
* Fixed a bug that wouldn't allow the rtk dongle to be enabled as it was using the wrong variable to enable it.
* See changelog
* Added aiding measurement summary for each GNSS (GQ7 only)
  Added MSCL version output when node starts
* Merge pull request `#50 <https://github.com/LORD-MicroStrain/ROS-MSCL/issues/50>`_ from civerachb-cpr/master
  Add an arg to enable setting NED/ENU frame parameter
* Add an arg to enable setting NED/ENU frame parameter
* Contributors: Chris Iverach-Brereton, Nathan Miller, Wolfgang Merkt, nathanmillerparker, robbiefish

1.1.3 (2021-04-21)
------------------
* Removed duplicate Filter LLH Pos entry in message format
  Preparing for release on Bloom
* Merge pull request `#49 <https://github.com/LORD-MicroStrain/ROS-MSCL/issues/49>`_ from civerachb-cpr/rosdep-fix
  Add tf2_geometry_msgs as a dependency
* Add tf2_geometry_msgs as a dependency
* Merge pull request `#48 <https://github.com/LORD-MicroStrain/ROS-MSCL/issues/48>`_ from civerachb-cpr/master
  Make frame IDs configurable
* Add args for all of the frame ids to allow them to be modified when launching.  Keep the old static values as the defaults.
* Added frame ids back in to not break existing configurations
* Added a flag to set ENU as the local reference frame
  Moved sensor2vehicle frame transformation setting code to occur if filter data is not enabled
  See changelog for more info
* Added user notifications in the case a command isn't supported by a device.
  Added support for the speedometer lever arm offset command
* Corrected description in launch file to point out the quaternion version of the sensor2vehicle frame transformation is not currently supported on the GQ7
* Added ROS_INFO/ROS_ERROR reporting for setting sensor2vehicle frame transformation... had a silent error for the quaternion version on the GQ7.
* Added the filter GPS timestamp packet to the configured messages.
* - Driver modified to support MSCL version 61.1.6
  - Fixed missing boolean set for RTK status message publishing
* Timestamp change:
  1. Launch file setting "use_device_timestamp" (bool) created to allow user to select between device generated timestamp and packet received time (generated using PC time upon packet reception.)
  - Some applications require the PC received time to sync with other packages
  - Some applications require the device generated timestamp for accurate time of when the data was generated
  Hopefully, this satisfies both needs.
* Merge pull request `#36 <https://github.com/LORD-MicroStrain/ROS-MSCL/issues/36>`_ from arpg/master
  Fixed issue including mscl_msgs
* Fixed issue including mscl_msgs
* Merge pull request `#34 <https://github.com/LORD-MicroStrain/ROS-MSCL/issues/34>`_ from CaptKrasno/msg
  Moved Messages to Separate Package and renamed them to match ros convention
* Merge branch 'master' into msg
* Warning: Contains breaking change to /nav/odom message!
  Code cleanup, new features, bug fixes
  See changelog for complete list of changes
* Separated Messages into a second package and changed naming to match ros convention
* Merge remote-tracking branch 'upstream/master'
* Merge pull request `#30 <https://github.com/LORD-MicroStrain/ROS-MSCL/issues/30>`_ from CaptKrasno/gps_corr
  Added support for gps_correlation_timestamp packet
* changed default value for  m_publish_gps_corr to false
* Merge branch 'master' into gps_corr
* Merge pull request `#31 <https://github.com/LORD-MicroStrain/ROS-MSCL/issues/31>`_ from CaptKrasno/gravity
  redefined g according to the spec
* redefined g according to the spec
* Added support for gps_correlation_timestamp packet
* Modified filter, GNSS, and RTK timestamp handling to disregard valid flags (to match IMU handling)
* Added IMU GPS timestamp as a default data setup quantity.
  Removed IMU timestamp validity check so time still streams prior to GPS lock.
* Fixed bug preventing device report service from working on a GQ7.
* Added support for raw binary file output and RTK status message (see changelog for details)
* Added PPS Source, GPIO Config, and external GPS time updating
* Added feature checking for filter reset and imu category
* Fixed driver error that tried to publish magnetometer data when it is not present
* 1) Added device Idle prior to shutdown to play nice across host power cycles
  2) Fixed flags used to determine valid time for GNSS time message
* Fixed time reference output to use ROS time for header timestamp
* sensor_msgs::TimeReference added per user request
* Added a resume command at the end of device setup as the GQ7 needs it.
* 1) Changed GQ7 filter init alignment selector to a bitfield in the example launch file
  2) Fixed quaternion sensor2vehicle frame rotation (negated the indices instead of the values by accident)
* See changelog for full details.
  Added support for GQ7
  Changed "GPS" topic to "GNSS1" and added "GNSS2"
  Refactored code
* Added Device Settings service:  Supports function selectors: 3 (Save), 4 (Load Saved), 5 (Load Defaults)
* Added nav filter heading state feedback
* Only doing device_status_callback() at 1 Hz now
* Fully filled-out device status message
* Added missing system timer to device status message
* Added a nav heading message to easily interpret current filter heading
* Fixed firmware version number reporting in device_report service
* 1) Fixed missing CMakeList services
  2) Updated "Get" services to output data in response (still being tested)
* 1) Changes to CMakeLists committed (changes were made previously, but didn't update for unknown reasons)
  2) Removed unused files
* Launch file didn't commit in previous attempts:
  1) Cleaned-up the file
  2) Renamed the frames for more clear indication of origin
* 1) Code restructured and commented more fully
  2) Quaternions now correct and relative to NED frame
* Changes to cleanup driver:
  1) Services renamed for better interpretation of functionality
  2) Quaternion now output correctly (i.e. wrt NED frame)
  3) Frame definitions changed to represent NED frame
* Update microstrain_3dm.cpp
  Adjusted gyro bias capture to 10 seconds
* Update microstrain_3dm.cpp
* Update microstrain_3dm.cpp
* Merge pull request `#15 <https://github.com/LORD-MicroStrain/ROS-MSCL/issues/15>`_ from allenh1/get-set-transform-service-improvements
  Get/Set Transform Service Improvements
* Merge pull request `#16 <https://github.com/LORD-MicroStrain/ROS-MSCL/issues/16>`_ from allenh1/store-mscl-as-unique-ptr
  Store msclInternalNode as a std::unique_ptr<mscl::InertialNode>
* Use the msclInertialNode pointer to check supported commands
* Store the mscl::InertialNode as a std::unique_ptr, and remove unused variable from diagnostic updater
* Add a service call to get the full transform from sensor to vehicle frame
* Replace empty destructor with default keyword
* Rename vehicle translation and rotation offset setting services to better match their function
* Remove unused service
* Fixed sensor to vehicle frame services
* Added ZUPT services
  - cmded_ang_rate_zupt
  - cmded_vel_zupt
  - set_heading_source
  - get_zero_velocity_update_threshold
  - set_zero_velocity_update_threshold
  added optional parameters
  - velocity_zupt_topic
  - angular_zupt_topic
* Added new estfilter channels
* Updated frames
* Added header info to mag msg
* new fields
* Custom message for filter status
* New fields
* New Fields
* Update microstrain_3dm.cpp
* Publishes nav_status
* device_setup parameter for pre-configured nodes
* Change heading_source default value
* Removed structured bindings
  No longer requires support for c++17
* Switched to device and received timestamps
* Added heading_source parameter
* Added heading_source parameter
* Added /filtered/imu/data
* Added /filtered/imu/data
* Added realpath to Connection
* Update Status Messages
  Updated status reporting to list only supported diagnostic features. This requires mscl version 55.0.1 or later.
* * move driver package content to ros_mscl folder
  * add name argument to microstrain.launch file to specify the namespace (default: gx5)
  * update README.md
  * add basic subscriber example (C++)
* Contributors: Chris Iverach-Brereton, Hunter L. Allen, Kristopher Krasnosky, Nathan Miller, harelb, mgill, nathanmillerparker, rdslord

0.0.4 (2019-10-07)
------------------

0.0.3 (2019-08-05)
------------------

0.0.2 (2019-05-28)
------------------
