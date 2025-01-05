^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package microstrain_inertial_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.5.0 (2024-11-07)
------------------
* ROS submodule update 11 07 2024 (`#363 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/363>`_)
* Contributors: Rob

4.4.0 (2024-10-07)
------------------
* ROS submodule update 08/23/2024 (`#349 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/349>`_)
* Contributors: Rob

4.3.0 (2024-05-15)
------------------

4.2.0 (2024-04-04)
------------------

4.1.0 (2024-04-02)
------------------
* ROS puts all messages into single dir (`#311 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/311>`_)
* Contributors: Rob

4.0.1 (2024-03-12)
------------------

4.0.0 (2024-01-26)
------------------
* ROS2 Standardize and update to work better with ROS standard tools (`#295 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/295>`_)
* Contributors: Rob

3.2.1 (2024-01-26)
------------------

3.2.0 (2023-12-21)
------------------

3.1.0 (2023-07-25)
------------------
* ROS Turns on antenna calibration by default, and publishes the amount corrected by (`#237 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/237>`_)
  * Turns on antenna calibration by default, and publishes the amount corrected by
  * Updates submodules to main
* Contributors: Rob

3.0.1 (2023-02-10)
------------------

3.0.0 (2023-01-11)
------------------
* Adds IMU overrange status publishers for ROS (`#207 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/207>`_)
  * Adds IMU overrange status publishers for ROS
* Adds RF error detection publishers for ROS (`#206 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/206>`_)
  * Adds RF error detection to ROS
* Adds SBAS info publishers for ROS (`#204 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/204>`_)
  * Adds SBAS info publishers for ROS
  * Updates submodule to main
* Contributors: Rob

2.7.1 (2022-11-09)
------------------

2.7.0 (2022-09-23)
------------------

2.6.0 (2022-05-25)
------------------
* Converts message definitions to use unix line endings (`#138 <https://github.com/LORD-MicroStrain/microstrain_inertial/issues/138>`_)
* Contributors: Lucas Walter

2.5.1 (2022-02-15)
------------------

2.5.0 (2022-01-27)
------------------
* Added RTK v2 support
* Fixed submodule initialization
* Contributors: dacuster, robbiefish

2.4.0 (2022-01-05)
------------------

2.3.0 (2021-12-22)
------------------
* BREAKING: Updates DeviceReport.srv to return the device report instead of just printing it
* Adds Fix Info message
* Adds Aiding Measurement Summary message
* Updates license files to be accurate for each package
* Contributors: robbiefish

2.2.0 (2021-12-03)
------------------
* Adds set filter speed lever arm service definition to allow users to configure Measurement Speed Lever Arm at runtime with a service call
* Adds Input Speed Measurement message
* Contributors: robbiefish

2.1.0 (2021-11-12)
------------------

2.0.5 (2021-10-21)
------------------
* Updates maintainers and dependencies in preparation for ROS build farm
* Moves submodules to subdirectory to get bloom working
* Renames packages to be more consistent with ROS naming conventions
* Contributors: Rob Fisher, robbiefish

1.1.4 (2021-07-30)
------------------
* Please see changelog
* Added aiding measurement summary for each GNSS (GQ7 only)
  Added MSCL version output when node starts
* Merge pull request `#54 <https://github.com/LORD-MicroStrain/ROS-MSCL/issues/54>`_ from civerachb-cpr/master
  Explicitly name the mscl_msgs package
* Explicitly name the mscl_msgs package instead of using the directory name; the dir name isn't reliable on some build-farms that extract each package into a separate working directory
* Contributors: Chris Iverach-Brereton, Nathan Miller, nathanmillerparker

1.1.3 (2021-04-21)
------------------
* Removed duplicate Filter LLH Pos entry in message format
  Preparing for release on Bloom
* - Driver modified to support MSCL version 61.1.6
  - Fixed missing boolean set for RTK status message publishing
* Merge pull request `#34 <https://github.com/LORD-MicroStrain/ROS-MSCL/issues/34>`_ from CaptKrasno/msg
  Moved Messages to Separate Package and renamed them to match ros convention
* Separated Messages into a second package and changed naming to match ros convention
* Contributors: Kristopher Krasnosky, Nathan Miller, nathanmillerparker

0.0.4 (2019-10-07)
------------------

0.0.3 (2019-08-05)
------------------

0.0.2 (2019-05-28)
------------------
