^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package test_launch_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.17.0 (2021-12-14)
-------------------

0.16.0 (2021-11-29)
-------------------
* More Helpful Error Messages (`#275 <https://github.com/ros2/launch_ros/issues/275>`_)
* Update maintainers in setup.py (`#287 <https://github.com/ros2/launch_ros/issues/287>`_)
* Set parameters from file for composable nodes (`#281 <https://github.com/ros2/launch_ros/issues/281>`_)
* Update package maintainers (`#284 <https://github.com/ros2/launch_ros/issues/284>`_)
* Update node name matcher (`#282 <https://github.com/ros2/launch_ros/issues/282>`_)
* Support both parameter file configurations for composable nodes (`#259 <https://github.com/ros2/launch_ros/issues/259>`_)
* Contributors: Aditya Pande, Audrow Nash, David V. Lu!!, Jacob Perron, Michel Hidalgo, Rebecca Butler

0.15.0 (2021-10-07)
-------------------
* Shutdown context after test (`#267 <https://github.com/ros2/launch_ros/issues/267>`_)
* Handle substitutions in RosTimer (`#264 <https://github.com/ros2/launch_ros/issues/264>`_)
* Add SetParametersFromFile action (`#260 <https://github.com/ros2/launch_ros/issues/260>`_)
* Properly support ros_args attribute through launch frontends (`#253 <https://github.com/ros2/launch_ros/issues/253>`_)
* Add 'push_ros_namespace' alias to 'push-ros-namespace' (`#250 <https://github.com/ros2/launch_ros/issues/250>`_)
* Add ros_arguments option to Node action (`#249 <https://github.com/ros2/launch_ros/issues/249>`_)
* ROS Timer Action (`#244 <https://github.com/ros2/launch_ros/issues/244>`_)
* Support container in frontend (`#235 <https://github.com/ros2/launch_ros/issues/235>`_)
* Contributors: Aditya Pande, Christophe Bedard, Jacob Perron, Kenji Miyake, Rebecca Butler

0.14.2 (2021-04-26)
-------------------

0.14.1 (2021-04-12)
-------------------
* Add a package marker to test_launch_ros. (`#226 <https://github.com/ros2/launch_ros/issues/226>`_)
* Contributors: Chris Lalancette

0.14.0 (2021-04-06)
-------------------

0.13.0 (2021-01-25)
-------------------
* Re-order shutdown vs node destruction (`#213 <https://github.com/ros2/launch_ros/issues/213>`_)
* Contributors: Scott K Logan

0.12.0 (2020-12-08)
-------------------
* Increase test_composable_node_container timeout (`#195 <https://github.com/ros2/launch_ros/issues/195>`_)
* Remove constructors arguments deprecated since Foxy (`#190 <https://github.com/ros2/launch_ros/issues/190>`_)
* Merge pull request `#183 <https://github.com/ros2/launch_ros/issues/183>`_ from ros2/update-maintainers
* Move previous maintainer to <author>
* Update the package.xml files with the latest Open Robotics maintainers
* Handle any substitution types for SetParameter name argument (`#182 <https://github.com/ros2/launch_ros/issues/182>`_)
* Address security bug in yaml loading (`#175 <https://github.com/ros2/launch_ros/issues/175>`_)
* Resolve TODO in test (`#172 <https://github.com/ros2/launch_ros/issues/172>`_)
* Fix case where list of composable nodes is zero (`#173 <https://github.com/ros2/launch_ros/issues/173>`_)
* Do not use event handler for loading composable nodes (`#170 <https://github.com/ros2/launch_ros/issues/170>`_)
* Fix race with launch context changes when loading composable nodes (`#166 <https://github.com/ros2/launch_ros/issues/166>`_)
* Substitutions in parameter files (`#168 <https://github.com/ros2/launch_ros/issues/168>`_)
* Fix problems when parsing a `Command` `Substitution` as a parameter value (`#137 <https://github.com/ros2/launch_ros/issues/137>`_)
* Drop double single-quoted params. (`#164 <https://github.com/ros2/launch_ros/issues/164>`_)
* Add a way to set remapping rules for all nodes in the same scope (`#163 <https://github.com/ros2/launch_ros/issues/163>`_)
* Fix ComposableNode ignoring PushRosNamespace actions (`#162 <https://github.com/ros2/launch_ros/issues/162>`_)
* Add a SetParameter action that sets a parameter to all nodes in the same scope (`#158 <https://github.com/ros2/launch_ros/issues/158>`_)
* Make namespace parameter mandatory in LifecycleNode constructor (`#157 <https://github.com/ros2/launch_ros/issues/157>`_)
* Avoid using a wildcard to specify parameters if possible (`#154 <https://github.com/ros2/launch_ros/issues/154>`_)
* Remove the loop parameter from async.sleep. (`#155 <https://github.com/ros2/launch_ros/issues/155>`_)
* Fix no specified namespace (`#153 <https://github.com/ros2/launch_ros/issues/153>`_)
* Fix test_node_frontend (`#146 <https://github.com/ros2/launch_ros/issues/146>`_)
* Add pytest.ini so local tests don't display warning (`#152 <https://github.com/ros2/launch_ros/issues/152>`_)
* Contributors: Chris Lalancette, Dan Rose, Ivan Santiago Paunovic, Jacob Perron, Michael Jeronimo, Michel Hidalgo, Víctor Mayoral Vilches

0.10.2 (2020-05-26)
-------------------

0.10.1 (2020-05-13)
-------------------
* Clean up various pytest warnings (`#143 <https://github.com/ros2/launch_ros/issues/143>`_)
* Contributors: Michael Carroll

0.10.0 (2020-04-29)
-------------------
* Deprecated 'node_executable' parameter and replace with 'executable' (`#140 <https://github.com/ros2/launch_ros/issues/140>`_)
* More verbose test_flake8 error messages (same as `ros2/launch_ros#135 <https://github.com/ros2/launch_ros/issues/135>`_)
* Enable implicit ROS startup by launch_ros actions.  (`#128 <https://github.com/ros2/launch_ros/issues/128>`_)
* Fix flake8 linter errors (`#130 <https://github.com/ros2/launch_ros/issues/130>`_)
* Add warning message when launching Non-Uniquely Named Nodes (`#127 <https://github.com/ros2/launch_ros/issues/127>`_)
* Rename node-related parameters (`#122 <https://github.com/ros2/launch_ros/issues/122>`_)
* Fix frontend topic remapping (`#111 <https://github.com/ros2/launch_ros/issues/111>`_)
* Maintain order of parameters regarding name and from (`#99 <https://github.com/ros2/launch_ros/issues/99>`_)
* Fix push-ros-namespace in xml/yaml launch files (`#100 <https://github.com/ros2/launch_ros/issues/100>`_)
* Contributors: Brian Marchi, Dirk Thomas, Ivan Santiago Paunovic, Jacob Perron, Miaofei Mei, Michel Hidalgo

0.9.4 (2019-11-19)
------------------

0.9.3 (2019-11-13)
------------------
* Install package.xml (`#92 <https://github.com/ros2/launch_ros/issues/92>`_)
* Contributors: Gaël Écorchard

0.9.2 (2019-10-23)
------------------
* Fix launch_ros.actions.Node parsing function (`#83 <https://github.com/ros2/launch_ros/issues/83>`_)
* Contributors: Michel Hidalgo

0.9.1 (2019-09-28)
------------------

0.9.0 (2019-09-25)
------------------
* Handle zero-width string parameters. (`#72 <https://github.com/ros2/launch_ros/issues/72>`_)
* Add substitution for finding package share directory (`#57 <https://github.com/ros2/launch_ros/issues/57>`_)
* Adapt to '--ros-args ... [--]'-based ROS args extraction (`#52 <https://github.com/ros2/launch_ros/issues/52>`_)
* Use node namespace if no other was specified (`#51 <https://github.com/ros2/launch_ros/issues/51>`_)
* [launch frontend] Rename some tag attributes (`#47 <https://github.com/ros2/launch_ros/issues/47>`_)
* Fix PushRosNamespace action (`#44 <https://github.com/ros2/launch_ros/issues/44>`_)
* Add PushRosNamespace action (`#42 <https://github.com/ros2/launch_ros/issues/42>`_)
* Add frontend parsing methods for Node, ExecutableInPackage and FindPackage substitution (`#23 <https://github.com/ros2/launch_ros/issues/23>`_)
* Restrict yaml loading in evaluate_parameters (`#33 <https://github.com/ros2/launch_ros/issues/33>`_)
* Use wildcard syntax in generated parameter YAML files (`#35 <https://github.com/ros2/launch_ros/issues/35>`_)
* Contributors: Jacob Perron, Michel Hidalgo, Scott K Logan, ivanpauno

0.8.4 (2019-05-30)
------------------

0.8.3 (2019-05-29)
------------------
* Added the ``FindPackage`` substitution. (`#22 <https://github.com/ros2/launch_ros/issues/22>`_)
* Changed interpretation of Parameter values which are passed to ``Node()`` so that they get evaluated by yaml rules. (`#31 <https://github.com/ros2/launch_ros/issues/31>`_)
* Contributors: Shane Loretz, ivanpauno

0.8.2 (2019-05-20)
------------------

0.8.1 (2019-05-08)
------------------

0.8.0 (2019-04-14)
------------------
* Added normalize_parameters and evaluate_paramters. (`#192 <https://github.com/ros2/launch/issues/192>`_)
* Added normalize_remap_rule and types. (`#173 <https://github.com/ros2/launch/issues/173>`_)
* Added support for required nodes. (`#179 <https://github.com/ros2/launch/issues/179>`_)
* Contributors: Kyle Fazzari, Shane Loretz

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
* Fixed a bug to ensure that shutdown event is handled correctly (`#154 <https://github.com/ros2/launch/issues/154>`_)
  * There was a potential race condition in between when the shutdown event is emitted and the rest of the shutdown handling code.
  * This introduces an additional await to ensure that the event is emitted before proceeding.
* Added support for passing parameters as a dictionary to a Node (`#138 <https://github.com/ros2/launch/issues/138>`_)
* Made various fixes and added tests for remappings passed to Node actions (`#137 <https://github.com/ros2/launch/issues/137>`_)
* Added ability to pass parameter files to Node actions (`#135 <https://github.com/ros2/launch/issues/135>`_)
* Contributors: Michael Carroll, dhood
