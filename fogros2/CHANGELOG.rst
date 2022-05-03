^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fogros2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
0.1.0 (2022-05-02)
-----------
* updates CLI to use AWS APIs to interface with running instances
* adds support for Ubuntu 20.04 and 22.04
* decouples the launch file and only include the inherited launch
* adds local peer to fix the DDS node discovery
* changes cloud containers to use wg interface
* adds lock to protect cloud instance ready state
* first public release for Humble