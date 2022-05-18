Getting Started
===
FogROS 2 streamlines the process of running parts of a ROS 2 application in the cloud.  The main benefit is to speed up compute-intensive nodes by using cloud-based high-end computers and hardware acceleration.  The only change typically needed is the launch configuration.

Before describing how to configure a ROS 2 application to use FogROS 2, we will discuss two critical things: _region_ and _instance type_.

Region
---
The region is where the cloud computer is physically located.  Since network latency increases as the distance between robot and cloud increases, selecting a physically close region is essential.  A great tool to understand regions and latency is [cloudping.info].  Usually, it is best to select the region with the lowest latency (we will discuss in a moment).

Instance Type
---
Cloud computers have different hardware specifications with a range of options: the number of CPU cores, amount of memory, types of included hardware accelerators (e.g., GPU, TPU, FPGA), and more.  Here are a few example considerations:

Does the ROS node use a deep neural network and NOT have a GPU or TPU?  If so, a cloud computer with a GPU or TPU could speed up computation significantly.
Does the ROS node make use of multi-core concurrency?  If so, a cloud computer with many more cores than the robot has (e.g., 32-cores, 72-cores, 96-cores) could speed up computation significantly.
Could the ROS node use a specialized hardware accelerator such as an FPGA?  If so, a cloud computer with the appropriate hardware could speed up computation significantly.
Are there more nodes running than the robot can handle?  If so, hardware acceleration is less important than just getting enough computer cores to meet the demand.

The second part of instance type selection is determining the instance size, or how much computing is enough to best price/performance ratio (or to just fit into a budge).  Higher-end instance types cost more money per hour, so ideally, one should select the minimum specification to meet performance criteria.  For example, if the application gains no benefit beyond a 32-core computer, selecting a 92-core computer would waste money.  How to best determine the instance size can come from a deep understanding of the application code or running a series of experiments with different instance sizes.

Instance types and Regions
---
The one caveat in the above discussion is that not all regions have all instance types.  You may have to look at different regions or instance types to find the best computing resource for your application.

