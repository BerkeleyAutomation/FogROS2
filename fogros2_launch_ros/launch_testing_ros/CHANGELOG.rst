^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package launch_testing_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.17.0 (2021-12-14)
-------------------
* Revert WaitForTopics (`#288 <https://github.com/ros2/launch_ros/issues/288>`_)
* Contributors: Jacob Perron

0.16.0 (2021-11-29)
-------------------
* Update maintainers in setup.py (`#287 <https://github.com/ros2/launch_ros/issues/287>`_)
* Move pytest entrypoints to own module (`#278 <https://github.com/ros2/launch_ros/issues/278>`_)
* Update package maintainers (`#284 <https://github.com/ros2/launch_ros/issues/284>`_)
* Check that future is done, and always call rclpy.shutdown (`#273 <https://github.com/ros2/launch_ros/issues/273>`_)
* Revert "launch testing : Wait for topics to publish (`#274 <https://github.com/ros2/launch_ros/issues/274>`_)" (`#276 <https://github.com/ros2/launch_ros/issues/276>`_)
* Contributors: Audrow Nash, Jorge Perez, Michel Hidalgo, Shane Loretz

0.15.0 (2021-10-07)
-------------------
* Add WaitForTopics utility for waiting on publishers (`#274 <https://github.com/ros2/launch_ros/issues/274>`_)
* Remove unused code, Future.result() already raises (`#270 <https://github.com/ros2/launch_ros/issues/270>`_)
* Add timeout to wait for service response in example (`#271 <https://github.com/ros2/launch_ros/issues/271>`_)
* Add examples (`#263 <https://github.com/ros2/launch_ros/issues/263>`_)
* Contributors: Aditya Pande, Shane Loretz

0.14.2 (2021-04-26)
-------------------
* Use underscores in setup.cfg instead of dashes. (`#227 <https://github.com/ros2/launch_ros/issues/227>`_)
* Contributors: Mike Purvis

0.14.1 (2021-04-12)
-------------------

0.14.0 (2021-04-06)
-------------------

0.13.0 (2021-01-25)
-------------------

0.12.0 (2020-12-08)
-------------------
* Merge pull request `#183 <https://github.com/ros2/launch_ros/issues/183>`_ from ros2/update-maintainers
* Move Pete to author, per clalancette
* Update the package.xml files with the latest Open Robotics maintainers
* Add pytest.ini so local tests don't display warning (`#152 <https://github.com/ros2/launch_ros/issues/152>`_)
* Contributors: Chris Lalancette, Michael Jeronimo

0.10.2 (2020-05-26)
-------------------
* Call LaunchROSTestModule with the new API. (`#150 <https://github.com/ros2/launch_ros/issues/150>`_)
* Contributors: Chris Lalancette

0.10.1 (2020-05-13)
-------------------

0.10.0 (2020-04-29)
-------------------
* Deprecated 'node_executable' parameter and replace with 'executable' (`#140 <https://github.com/ros2/launch_ros/issues/140>`_)
* Avoid deprecation warning, use from_parent (`#141 <https://github.com/ros2/launch_ros/issues/141>`_)
* Show error strings as part of the flake8 test (`#135 <https://github.com/ros2/launch_ros/issues/135>`_)
* Remove unused 'launch' import (`#133 <https://github.com/ros2/launch_ros/issues/133>`_)
* Enable implicit ROS startup by launch_ros actions  (`#128 <https://github.com/ros2/launch_ros/issues/128>`_)
* Fix launch_testing_ros example (`#121 <https://github.com/ros2/launch_ros/issues/121>`_)
* Contributors: Dirk Thomas, Jacob Perron, Michel Hidalgo

0.9.4 (2019-11-19)
------------------
* fix new linter warnings as of flake8-comprehensions 3.1.0 (`#94 <https://github.com/ros2/launch_ros/issues/94>`_)
* Contributors: Dirk Thomas

0.9.3 (2019-11-13)
------------------

0.9.2 (2019-10-23)
------------------
* Remove self.proc_output and ready_fn (`#90 <https://github.com/ros2/launch_ros/issues/90>`_)
* Add support for launching nodes not in a package (`#82 <https://github.com/ros2/launch_ros/issues/82>`_)
* Contributors: Michel Hidalgo, Peter Baughman

0.9.1 (2019-09-28)
------------------
* Make launch_testing_ros examples standalone. (`#80 <https://github.com/ros2/launch_ros/issues/80>`_)
* Contributors: Michel Hidalgo

0.9.0 (2019-09-25)
------------------
* install package manifest (`#71 <https://github.com/ros2/launch_ros/issues/71>`_)
* Do not import rclpy nor launch_ros at module level. (`#69 <https://github.com/ros2/launch_ros/issues/69>`_)
* Unindent setup.cfg options. (`#66 <https://github.com/ros2/launch_ros/issues/66>`_)
* Support launch_ros test runner in pytest (`#54 <https://github.com/ros2/launch_ros/issues/54>`_)
* Contributors: Dirk Thomas, Michel Hidalgo

0.8.4 (2019-05-30)
------------------

0.8.3 (2019-05-29)
------------------

0.8.2 (2019-05-20)
------------------
* fix example test logic (`#28 <https://github.com/ros2/launch_ros/issues/28>`_)
* Add custom LaunchTestRunner with ROS specific preamble (`#26 <https://github.com/ros2/launch_ros/issues/26>`_)
* Fix deprecation warnings (`#25 <https://github.com/ros2/launch_ros/issues/25>`_)
* Contributors: Dirk Thomas, Jacob Perron, Michel Hidalgo

0.8.1 (2019-05-08)
------------------
* try local import (`#20 <https://github.com/ros2/launch_ros/issues/20>`_)
* Merge apex_launchtest_ros functionality into launch_testing_ros (`#8 <https://github.com/ros2/launch_ros/issues/8>`_)
* Contributors: Dirk Thomas, Michel Hidalgo

0.8.0 (2019-04-14)
------------------
