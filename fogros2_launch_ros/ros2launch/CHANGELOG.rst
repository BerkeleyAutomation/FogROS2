^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros2launch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.17.0 (2021-12-14)
-------------------

0.16.0 (2021-11-29)
-------------------
* Update maintainers in setup.py (`#287 <https://github.com/ros2/launch_ros/issues/287>`_)
* Use frontend group dependency & explicit dependencies in ros2launch (`#256 <https://github.com/ros2/launch_ros/issues/256>`_)
* Update package maintainers (`#284 <https://github.com/ros2/launch_ros/issues/284>`_)
* Contributors: Audrow Nash, Christophe Bedard, Michel Hidalgo

0.15.0 (2021-10-07)
-------------------
* Add regex filter for selective launch-prefix application (`#261 <https://github.com/ros2/launch_ros/issues/261>`_)
* Resolves `#37 <https://github.com/ros2/launch_ros/issues/37>`_ - Added --launch-prefix argument for 'ros2 launch' command (`#254 <https://github.com/ros2/launch_ros/issues/254>`_)
* Use sets of file extensions provided by parser extensions (`#252 <https://github.com/ros2/launch_ros/issues/252>`_)
* Simplify logic to fix absolute paths (`#230 <https://github.com/ros2/launch_ros/issues/230>`_)
* Contributors: Cameron Miller, Christophe Bedard, rob-clarke

0.14.2 (2021-04-26)
-------------------

0.14.1 (2021-04-12)
-------------------

0.14.0 (2021-04-06)
-------------------
* Add options extensions to ros2launch and extensibility to the node action (`#216 <https://github.com/ros2/launch_ros/issues/216>`_)
* Contributors: Geoffrey Biggs

0.13.0 (2021-01-25)
-------------------
* Support non-interactive ros2 launch executions (`#210 <https://github.com/ros2/launch_ros/issues/210>`_)
* Contributors: Michel Hidalgo

0.12.0 (2020-12-08)
-------------------
* Merge pull request `#183 <https://github.com/ros2/launch_ros/issues/183>`_ from ros2/update-maintainers
* Move previous maintainer to <author>
* Update the package.xml files with the latest Open Robotics maintainers
* Add pytest.ini so local tests don't display warning (`#152 <https://github.com/ros2/launch_ros/issues/152>`_)
* Contributors: Chris Lalancette, Michael Jeronimo

0.10.2 (2020-05-26)
-------------------

0.10.1 (2020-05-13)
-------------------
* argcomplete is optional (`#147 <https://github.com/ros2/launch_ros/issues/147>`_)
* Contributors: Alejandro Hernández Cordero

0.10.0 (2020-04-29)
-------------------
* Deprecated 'node_executable' parameter and replace with 'executable' (`#140 <https://github.com/ros2/launch_ros/issues/140>`_)
* More verbose test_flake8 error messages (same as `ros2/launch_ros#135 <https://github.com/ros2/launch_ros/issues/135>`_)
* Enable implicit ROS startup by launch_ros actions  (`#128 <https://github.com/ros2/launch_ros/issues/128>`_)
* Rename node-related parameters (`#122 <https://github.com/ros2/launch_ros/issues/122>`_)
* Legal tab completion of launch files (`#126 <https://github.com/ros2/launch_ros/issues/126>`_)
* Fix linter by removing unused import (`#110 <https://github.com/ros2/launch_ros/issues/110>`_)
* Fix misleading deprecated warnings when using launch arguments (`#106 <https://github.com/ros2/launch_ros/issues/106>`_)
* Use imperative mood in constructor docstrings (`#103 <https://github.com/ros2/launch_ros/issues/103>`_)
* Contributors: Dirk Thomas, Emerson Knapp, Ivan Santiago Paunovic, Jacob Perron, Michel Hidalgo, Shane Loretz, Steven! Ragnarök

0.9.4 (2019-11-19)
------------------

0.9.3 (2019-11-13)
------------------

0.9.2 (2019-10-23)
------------------

0.9.1 (2019-09-28)
------------------

0.9.0 (2019-09-25)
------------------
* install resource marker file for package (`#78 <https://github.com/ros2/launch_ros/issues/78>`_)
* install package manifest (`#71 <https://github.com/ros2/launch_ros/issues/71>`_)
* Support xml and yaml files in ros2launch (`#40 <https://github.com/ros2/launch_ros/issues/40>`_)
* Contributors: Dirk Thomas, ivanpauno

0.8.4 (2019-05-30)
------------------

0.8.3 (2019-05-29)
------------------

0.8.2 (2019-05-20)
------------------
* fix calling of print_arguments_of_launch_description() (`#27 <https://github.com/ros2/launch_ros/issues/27>`_)
* Launch autocomplete doesnt require dot (`#24 <https://github.com/ros2/launch_ros/issues/24>`_)
* Contributors: Matt Hansen, William Woodall

0.8.1 (2019-05-08)
------------------

0.8.0 (2019-04-14)
------------------
* Added --show-all-subprocesses-output command line option. (`#10 <https://github.com/ros2/launch/issues/10>`_)
* Make 'ros2 launch' work again. (`#201 <https://github.com/ros2/launch/issues/201>`_)
* Added plumb rclpy.init context to get_default_launch_description. (`#193 <https://github.com/ros2/launch/issues/193>`_)
* Refactored arg print functions (`#172 <https://github.com/ros2/launch/issues/172>`_)
* Contributors: Chris Lalancette, Michel Hidalgo, Peter Baughman

0.7.3 (2018-12-13)
------------------

0.7.2 (2018-12-06)
------------------

0.7.1 (2018-11-16)
------------------
* Fixed setup.py versions (`#155 <https://github.com/ros2/launch/issues/155>`_)
* Contributors: Steven! Ragnarök

0.7.0 (2018-11-16)
------------------
* Added ability to define and pass launch arguments to launch files (`#123 <https://github.com/ros2/launch/issues/123>`_)
  * See changelog in ``launch`` for details.
  * Signed-off-by: William Woodall <william@osrfoundation.org>
* Contributors: William Woodall

0.6.0 (2018-08-20)
------------------
* add way to include other Python launch files (`#122 <https://github.com/ros2/launch/issues/122>`_)
  * Signed-off-by: William Woodall <william@osrfoundation.org>
* Contributors: William Woodall

0.5.2 (2018-07-17)
------------------

0.5.1 (2018-06-27)
------------------
* Improved error handling in the ``ros2 launch`` command line tool. (`#93 <https://github.com/ros2/launch/issues/93>`_)
* Contributors: William Woodall

0.5.0 (2018-06-19)
------------------
* First commit of the `ros2launch` package and the `ros2 launch` CLI tool (`#76 <https://github.com/ros2/launch/issues/76>`_)
* Contributors: William Woodall
