^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package urg_node
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.1 (2023-03-18)
------------------
* add branch information
* add license file, same as ROS1
* increased delay in diagnostics thread to slow down publish rate (`#102 <https://github.com/ros-drivers/urg_node/issues/102>`_)
  This is just a quick PR to increase the thread sleep in the diagnostics thread. Currently the diagnostics status is updated at ~96hz. Which is way too fast and really messes with the Frequency Status Monitor which jumps between too low and too high
* Added URDF for UST10. (`#103 <https://github.com/ros-drivers/urg_node/issues/103>`_)
* Contributors: Michael Ferguson, Richard, Tony Baltovski

1.1.0 (2021-03-31)
------------------
* Merge pull request `#86 <https://github.com/ros-drivers/urg_node/issues/86>`_ from ros-drivers/clalancette/galactic-fixes
  Fixes for ROS 2 Galactic
* Don't error out on unknown parameter names.
  In general, parameter callbacks should not error out on unknown
  parameter names.  That's because a) the callbacks may be
  chained, and b) there may be internal parameters that are used.
  Just ignore anything we don't know about.
* Switch to PRIu32 for printing a uint32_t.
  This just ensures that it works on all platforms, and
  removes a warning on Galactic.
* Fix rclcpp::Duration construction.
  As of Galactic, initializing an rclcpp::Duration with just
  a number is deprecated.  We now have to explicitly tell it
  the units of the number, generally done through std::chrono.
* Remove unused reconfigure method.
* Contributors: Chris Lalancette, Michael Ferguson

1.0.3 (2021-03-31)
------------------
* Use Python 3 specifically in helper script (`#85 <https://github.com/ros-drivers/urg_node/issues/85>`_)
  ROS 2 only targets Python 3. Making the shebang specific will ensure it
  isn't accidentally executed using the Python 2 interpreter.
* travis-ci doesn't work
  But ros build farm is giving us coverage
* depend only on parts of boost needed by the package (`#75 <https://github.com/ros-drivers/urg_node/issues/75>`_)
* Contributors: Michael Ferguson, Mikael Arguedas, Scott K Logan

1.0.2 (2020-07-12)
------------------
* uncrustify for f/r (`#74 <https://github.com/ros-drivers/urg_node/issues/74>`_)
* fix deprecation warning (`#69 <https://github.com/ros-drivers/urg_node/issues/69>`_)
* Contributors: Michael Ferguson

1.0.1 (2020-06-10)
------------------
* node name is now urg_node_driver (`#70 <https://github.com/ros-drivers/urg_node/issues/70>`_)
  The node was renamed as part of the composable refactor
  At runtime, it still defaults to urg_node as the graph
  name
* call run in a thread, fixes `#66 <https://github.com/ros-drivers/urg_node/issues/66>`_ (`#71 <https://github.com/ros-drivers/urg_node/issues/71>`_)
* add myself as maintainer for ros2 (`#73 <https://github.com/ros-drivers/urg_node/issues/73>`_)
* Contributors: Michael Ferguson

1.0.0 (2020-03-24)
------------------
* migrate ros2 devel (`#50 <https://github.com/ros-drivers/urg_node/issues/50>`_)
* Merge pull request `#42 <https://github.com/ros-drivers/urg_node/issues/42>`_ from BadgerTechnologies/detect-time-warp-and-reset
* synchronize_time: reset when clock is warped
* Merge pull request `#41 <https://github.com/ros-drivers/urg_node/issues/41>`_ from BadgerTechnologies/synchronize-time
* synchronize system clock to hardware time
* Add Travis config.
* Fixed linter errors.
* Contributors: Aarush Gupta, aswinthomas, Brett, C. Andy Martin, Chris Lalancette, Gu Chao Jie, Karsten Knese, Marc-Antoine Testier, Tony Baltovski, Zoe

0.1.11 (2017-10-17)
-------------------
* Add support for URG-04LX in SCIP 1.1 mode
  The urg_node does not support SCIP 1.1. The Hokuyo URG-04LX supports both
  SCIP 1.1 and SCIP 2.0, but needs to be switched to SCIP 2.0 at every startup
  in its default configuration. For this purpose the function
  URGCWrapper::setToSCIP2() was added.
  A URG-04LX in SCIP 1.1 mode used to lead to an exception being thrown in
  URGCwrapper::initialize. Now, before throwing the exception an attempt to
  switch the sensor to SCIP 2.0 is made.
* Fixed comments in launch file and added roslaunch.
* Add flag to prevent updating of detailed status.
  If using a model that does not support AR00 command, hide it
  behind a rosparam.
* Add safety stop heading and distance values (`#28 <https://github.com/ros-drivers/urg_node/issues/28>`_)
  Added to the laser status field the last report of a safety
  stop of distance and angle reported. If this fails or is unavailable
  it will just report 0.
* Updating depend and roslint.
  Fixing some roslint error after moving a header name.
  Additionally fixing the gencfg to be on the lib and not the node.
* Adding missing std_srvs depend.
  Adding missing std_srvs depend to package.xml and CMakelists.txt
* Move urg_node to be a library.
  Moving urg_node to urg_node_driver as a library.
  This allows for other nodes to include this as an object instead
  of spawning another separate process.
* Add getAR00 status command.
  Added ability to pull the status of the lidar AR00 status command.
  This then publishes a latched topic with the current status of the
  lidar's error code and lockout status.
* Update urg_node to be a self contained class
  Updating urg node to be a self contained class. This allows
  for it to be imported in other nodes.
* Roslint
* Contributors: Benjamin Scholz, Mike O'Driscoll, Tony Baltovski

0.1.10 (2017-03-21)
-------------------
* Updated maintainer.
* Error handling for connection failures
* Created urg_lidar.launch
* Installed getID
* Contributors: Eric Tappan, Jeff Schmidt, Kei Okada, Tony Baltovski

0.1.9 (2014-08-13)
------------------
* Merge pull request `#7 <https://github.com/ros-drivers/urg_node/issues/7>`_ from mikeferguson/indigo-devel
  add a script to set the IP address of an URG laser
* Updated diagnostics to support configurable parameters.
* add a script to set the IP address of an URG laser
* Contributors: Chad Rockey, Michael Ferguson

0.1.8 (2014-06-16)
------------------
* Merge pull request `#6 <https://github.com/ros-drivers/urg_node/issues/6>`_ from mikeferguson/indigo-devel
  Add default device status on UST-20LX
* Add default device status on UST-20LX
* Contributors: Chad Rockey, Michael Ferguson

0.1.7 (2014-04-21)
------------------
* Added more robust plug/unplug reconnect behavior.
* Added more robustness and the ability to continually reloop and reconnect until node is shutdown.
* Fix initialization crash.
* Install fix for Android.
* Missed a willowgarage email.
* Contributors: Chad Rockey

0.1.6 (2013-10-24)
------------------
* Added getID executable for udev users.

0.1.5 (2013-08-22)
------------------
* Missing diagnostic_updater depend

0.1.4 (2013-08-22)
------------------
* Merge pull request `#2 <https://github.com/ros-drivers/urg_node/issues/2>`_ from mitll-ros-pkg/diagnostics
  Added diagnostics to the URG Node.
* Added diagnostics to the URG Node.

0.1.3 (2013-08-21)
------------------
* No more Willow Garage email.

0.1.2 (2013-03-14)
------------------
* Be more tolerant of connection dropouts and try to reconnect.
* Fixed poor initilization causing uncertain output.
* Updated consts

0.1.1 (2013-03-04)
------------------
* Only advertise for single or multiecho, not both.
* Generalized multi echo grab function
* Updated to use laser_proc to automatically publish compatibility messages.
* Optimize the fill multi echo laserscan message to use reserve instead of resize wherever possible.

0.1.0 (2013-03-03)
------------------
* Added install rules.
* Renamed package to urg_node.
* Updated to use better timestamping.
* Updated to use urg_c name for library.
* Added information functions for future diagnostics.
* Added experimental timestamp synchronization.  Fixed segfault for multiecho intensity.
* Fixed skip being cluster.  Added skip functionallity.
* Connected dynamic reconfigure, including angle limit requests.
* Added ability to publish both single and multi echo scans.
* Added dynamic reconfigure; can update reconfigure limits
* Initial commit.  Connecting to both ethernet and serial devices.
* Initial commit
