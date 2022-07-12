^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fogros2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
0.1.7 (2022-07-11)
------------------
* temporarily removing awscli as dependency due to regression

0.1.6 (2022-06-29)
------------------
* added wgconfig and wireguard as dependencies

0.1.5 (2022-06-08)
------------------
* added dependencies to package.xml to reduce those that are manually downloaded

0.1.4 (2022-05-15)
------------------
* fixed QUICKSTART.md documentation and removed unnecessary dependencies
* prepared package.xml with list of authors and maintainers for release
* changed environment variable checks from asserts to exceptions
* cleaned up README.md

0.1.3 (2022-05-09)
------------------
* readded human readable instance name generation
* added checks and interterpretable feedback for client errors
* added environment variable checks
* updated documentation to include new potential issues and remove unnecessary instructions
* fixed colcon warning message about arg parsing in the cloud instance

0.1.2 (2022-05-02)
------------------
* removed unique name generator and fixed package.xml

0.1.1 (2022-05-02)
------------------
* updates CLI to use AWS APIs to interface with running instances
* adds support for Ubuntu 20.04 and 22.04
* decouples the launch file and only include the inherited launch
* adds local peer to fix the DDS node discovery
* changes cloud containers to use wg interface
* adds lock to protect cloud instance ready state
* first public release for Humble