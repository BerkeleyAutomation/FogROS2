^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package launch_testing
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.21.0 (2022-01-14)
-------------------
* Renamed three files from example_processes (`#573 <https://github.com/ros2/launch/issues/573>`_)
* Fix launch_testing README.md proc keyword to process. (`#554 <https://github.com/ros2/launch/issues/554>`_) (`#560 <https://github.com/ros2/launch/issues/560>`_)
* Contributors: Jacob Perron, Khush Jain

0.20.0 (2021-11-29)
-------------------
* Declare frontend group dependency & use explicit dependencies in launch_testing (`#520 <https://github.com/ros2/launch/issues/520>`_)
* Update maintainers to Aditya Pande and Michel Hidalgo (`#559 <https://github.com/ros2/launch/issues/559>`_)
* Updated maintainers (`#555 <https://github.com/ros2/launch/issues/555>`_)
* First prototype of native pytest plugin for launch based tests (`#528 <https://github.com/ros2/launch/issues/528>`_)
* Adding Executable description class (`#454 <https://github.com/ros2/launch/issues/454>`_)
* Add a "hello world" style example (`#532 <https://github.com/ros2/launch/issues/532>`_)
* Contributors: Aditya Pande, Audrow Nash, Christophe Bedard, Ivan Santiago Paunovic, roger-strain

0.19.0 (2021-07-15)
-------------------

0.18.0 (2021-06-18)
-------------------

0.17.0 (2021-04-06)
-------------------

0.16.0 (2021-03-19)
-------------------
* Use unittest.mock instead of mock (`#487 <https://github.com/ros2/launch/issues/487>`_)
* Contributors: Michel Hidalgo

0.15.0 (2021-01-25)
-------------------

0.14.0 (2020-12-08)
-------------------

0.13.0 (2020-11-04)
-------------------
* Update package maintainers (`#465 <https://github.com/ros2/launch/issues/465>`_)
* Contributors: Michel Hidalgo

0.12.0 (2020-08-18)
-------------------

0.11.1 (2020-08-14)
-------------------

0.11.0 (2020-08-04)
-------------------
* Disable cleanup of test cases once they have been run (`#406 <https://github.com/ros2/launch/issues/406>`_)
* Fix max() with empty sequence (`#440 <https://github.com/ros2/launch/issues/440>`_)
* Use unittest.TestCase.id() for pytest failure reprs. (`#436 <https://github.com/ros2/launch/issues/436>`_)
* Use unittest.TestCase.id() to put together jUnit XML output. (`#435 <https://github.com/ros2/launch/issues/435>`_)
* Claim ownership (`#433 <https://github.com/ros2/launch/issues/433>`_)
* Contributors: Dirk Thomas, Michel Hidalgo, Scott K Logan, William Woodall

0.10.2 (2020-05-26)
-------------------
* Set junit_family to xunit2 in pytest.ini
* Stop using implicit variables in example testing.
* Switch to from_parent to remove deprecation warning.
* Fix new flake8 errors. (`#420 <https://github.com/ros2/launch/issues/420>`_)
* Remove uses of deprecated ready_fn. (`#419 <https://github.com/ros2/launch/issues/419>`_)
* Contributors: Chris Lalancette, Michel Hidalgo

0.10.1 (2020-05-08)
-------------------
* fixed depcrecation warning of imp to importlib (issue `#387 <https://github.com/ros2/launch/issues/387>`_) (`#407 <https://github.com/ros2/launch/issues/407>`_)
* wait_for_ouput() repr includes actual text (`#408 <https://github.com/ros2/launch/issues/408>`_)
* Contributors: Shane Loretz, Zahi Kakish

0.10.0 (2020-04-24)
-------------------
* Improve jUnit output for launch tests when run with py.test (`#404 <https://github.com/ros2/launch/issues/404>`_)
* avoid deprecation warning, use from_parent (`#402 <https://github.com/ros2/launch/issues/402>`_)
* Warn that old-style ready_fn and test attributes will be deprecated (`#346 <https://github.com/ros2/launch/issues/346>`_)
* more verbose test_flake8 error messages (same as `ros2/launch_ros#135 <https://github.com/ros2/launch_ros/issues/135>`_)
* stop using constructors deprecated in pytest 5.4 (`#391 <https://github.com/ros2/launch/issues/391>`_)
* Add the ability to assert in stdout or stderr. (`#378 <https://github.com/ros2/launch/issues/378>`_)
* Add delay parameter to retry_on_failure decorator (`#390 <https://github.com/ros2/launch/issues/390>`_)
* Make RegisterEventHandler describe its sub-entities (`#386 <https://github.com/ros2/launch/issues/386>`_)
* Import test file without contaminating sys.modules (`#360 <https://github.com/ros2/launch/issues/360>`_)
* Update reference to example launch test file (`#363 <https://github.com/ros2/launch/issues/363>`_)
* Use imperative mood in docstrings. (`#362 <https://github.com/ros2/launch/issues/362>`_)
* Fix a documentation typo. (`#361 <https://github.com/ros2/launch/issues/361>`_)
* Fix junit XML when launch dies early (`#358 <https://github.com/ros2/launch/issues/358>`_)
* Contributors: Chris Lalancette, Dan Rose, Dirk Thomas, Jacob Perron, Michel Hidalgo, Peter Baughman, Steven! Ragnarök

0.9.5 (2019-11-13)
------------------
* Make launch_testing.markers.retry_on_failure decorator more robust. (`#352 <https://github.com/ros2/launch/issues/352>`_)
* Contributors: Michel Hidalgo

0.9.4 (2019-11-08)
------------------
* Fix a small typo in the launch_testing README. (`#351 <https://github.com/ros2/launch/issues/351>`_)
* Contributors: Chris Lalancette

0.9.3 (2019-10-23)
------------------

0.9.2 (2019-10-23)
------------------
* Support launch test reruns when using pytest (`#348 <https://github.com/ros2/launch/issues/348>`_)
* Support CLI commands testing (`#279 <https://github.com/ros2/launch/issues/279>`_)
* Contributors: Michel Hidalgo

0.9.1 (2019-09-25)
------------------
* Optionally remove ready fn arg from generate_test_description (`#322 <https://github.com/ros2/launch/issues/322>`_)
* Contributors: Michel Hidalgo, Peter Baughman

0.9.0 (2019-09-18)
------------------
* install package manifest (`#330 <https://github.com/ros2/launch/issues/330>`_)
* Unindent setup.cfg options. (`#326 <https://github.com/ros2/launch/issues/326>`_)
* Use renamed remove_ansi_escape_sequences. (`#302 <https://github.com/ros2/launch/issues/302>`_)0
* Enable launch test discovery in pytest (`#312 <https://github.com/ros2/launch/issues/312>`_)
* Support LaunchService injection into pre-shutdown tests. (`#308 <https://github.com/ros2/launch/issues/308>`_)
* Add assertWaitForStartup method to match assertWaitForShutdown (`#278 <https://github.com/ros2/launch/issues/278>`_)
* Fix a simple typo in an error message. (`#301 <https://github.com/ros2/launch/issues/301>`_)
* Fix launch_testing output filtering (`#296 <https://github.com/ros2/launch/issues/296>`_)
* Revert "Revert "[execute_process] emulate_tty configurable and defaults to true"" (`#277 <https://github.com/ros2/launch/issues/277>`_)
* Fix formatting (`#262 <https://github.com/ros2/launch/issues/262>`_)
* Fix proc lookup for processes with multiple command-line arguments (`#229 <https://github.com/ros2/launch/issues/229>`_)
* Remove ros domain ID dependency (`#256 <https://github.com/ros2/launch/issues/256>`_)
* Contributors: Chris Lalancette, Dirk Thomas, Esteve Fernandez, Michel Hidalgo, Peter Baughman, William Woodall, ivanpauno

0.8.3 (2019-05-29)
------------------
* Changed behavior to use ``--isolated`` if no ``ROS_DOMAIN_ID`` is set to help parallel testing. (`#251 <https://github.com/ros2/launch/issues/251>`_)
* Contributors: Peter Baughman

0.8.2 (2019-05-20)
------------------
* add non-asserting waitFor method (`#243 <https://github.com/ros2/launch/issues/243>`_)
* Enable reuse of launch testing functionality (`#236 <https://github.com/ros2/launch/issues/236>`_)
* Stop randomizing ROS_DOMAIN_ID by default in launch tests (`#240 <https://github.com/ros2/launch/issues/240>`_)
* Contributors: Dirk Thomas, Michel Hidalgo

0.8.1 (2019-05-08)
------------------

0.8.0 (2019-04-13)
------------------
* Added test actions. (`#178 <https://github.com/ros2/launch/issues/178>`_)
* Fixed test_env_testing test (`#200 <https://github.com/ros2/launch/issues/200>`_)
* Dropped legacy launch package. (`#191 <https://github.com/ros2/launch/issues/191>`_)
* Migrated legacy launch API tests. (`#167 <https://github.com/ros2/launch/issues/167>`_)
* Contributors: Dirk Thomas, Michel Hidalgo, ivanpauno

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
* Fixed lint warnings from invalid escape sequences (`#151 <https://github.com/ros2/launch/issues/151>`_)
  Use raw strings for regex patterns to avoid warnings.
* Fixed linter errors from `#131 <https://github.com/ros2/launch/issues/131>`_. (`#132 <https://github.com/ros2/launch/issues/132>`_)
* Added class to provide some limitted testing options (`#131 <https://github.com/ros2/launch/issues/131>`_)
* Moved ``launch_testing`` into ``launch_testing.legacy`` namespace (`#130 <https://github.com/ros2/launch/issues/130>`_)
* Contributors: Dirk Thomas, Jacob Perron, Steven! Ragnarök

0.6.0 (2018-08-20)
------------------

0.5.2 (2018-07-17)
------------------

0.5.1 (2018-06-27)
------------------

0.5.0 (2018-06-19)
------------------
* Updated to use new launch.legacy namespace (`#73 <https://github.com/ros2/launch/issues/73>`_)
* Contributors: Dirk Thomas, Mikael Arguedas, William Woodall
