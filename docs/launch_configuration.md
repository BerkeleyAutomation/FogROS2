---
layout: page
title: Launch Configuration
permalink: /launch_configuration
nav_order: 5
---

# Launch Configuration
ROS 2 utilizes Python to create launch files which can specify what ROS 2 nodes should be launched. FogROS 2 utilizes these launch files to not only specify what nodes are launched, but where they are launched (i.e. on the robot or on the cloud machine). This page provides information about modifying existing launch files to work with FogROS 2 for deploying nodes to the cloud. 

## Robot Nodes 
Robot nodes are ROS 2 nodes that are only running on the robot. There are no modifications necessary to the normal launch file to launch nodes that are running on the robot.

## Cloud Nodes 
Launching ROS 2 nodes in the cloud can be completed in 4 steps:
1. Add the following import to the top of the launch file: ```import fogros2```

2. Replace the ```LaunchDescription``` in the launch file with ```fogros2.FogROSLaunchDescription```

3. Specify AWS EC2 machine parameters by creating a ```fogros2.AWSCloudInstance``` object 

4. Replace all ```Node``` objects that need to be executed in the cloud with ```fogros2.CloudNode``` 

### fogros2.AWSCloudInstance

#### Definition
```fogros2.AWSCloudInstance(ami_image, region="us-west-1", ec2_instance_type="t2.micro", disk_size=30)```

#### Description
An ```AWSCloudInstance``` is an object that represents an AWS EC2 instance that performs computation in the cloud. By specifying certain parameters for the machine, the AWS EC2 machine may be launched in different AWS regions or with different hardware. For help deciding some of the parameters, please visit the [Getting Started]({{site.baseurl}}/getting_started/) page.

#### Parameters
```ami-image```: An AWS AMI is a template that is used to launch machines. The parameter itself is the  Two recommended AMI images are 'ami-00f25057ddc9b310b' for Ubuntu 20.04 and 'ami-0b6030c78f8b2f076' for Ubuntu 22.04. Custom AMI images may be created to speed up deployment time, as any necessary dependencies can be installed in the AMI, so that every launch need not install all dependencies. For more information about AMIs, visit this [link](https://docs.aws.amazon.com/AWSEC2/latest/UserGuide/AMIs.html).

```region```: The AWS region the machine will be launched in. By default, this is ```us-west-1```.

```ec2_instance_type```: The type of machine that will be launched by AWS. For help deciding what machine to launch, please visit [Getting Started]({{site.baseurl}}/getting_started). By default, this is a ```t2.micro``` instance, which is a lower compute free instance type.

```disk_size```: The size of the disk of the EC2 instance in gigabytes. The default is 30GB.

### fogros2.CloudNode

#### Definition
```fogros2.CloudNode(machine, stream_topics=[], ...)```

#### Description
A ```CloudNode``` is a ROS 2 node that is executing in the cloud.

#### Parameters
```machine```: machine is the ```fogros2.AWSCloudInstance``` created in step 3.

```stream_topics```: stream_topics is used to specify any topics that are used for video streaming so that appropriate compression can be deployed. For more details, visit [Video Compression (H.264)]({{site.baseurl}}/video_compression).

Make sure to leave in any normal arguments that are a part of a normal ROS 2 node. An example will be provided to make the transition more clear.

## Talker-Listener Example 
Consider the following talker-listener launch file, which launches two nodes, where the listener subscribes to a topic and talker publishes to a topic (more details [here](https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html)): 
```
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    listener_node = Node(
        package="fogros2_examples", executable="listener", output="screen"
    )
    talker_node = Node(
        package="fogros2_examples", executable="talker", output="screen"
    )
    ld.add_action(talker_node)
    ld.add_action(listener_node)
    return ld
```

This section will show how to deploy the talker node onto the cloud, while the listener node stays on the robot.

### Step 1: Add the following import to the top of the launch file: ```import fogros2```
After running through this step, the launch file should now look like this:
```
import fogros2
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    listener_node = Node(
        package="fogros2_examples", executable="listener", output="screen"
    )
    talker_node = Node(
        package="fogros2_examples", executable="talker", output="screen"
    )
    ld.add_action(talker_node)
    ld.add_action(listener_node)
    return ld
```

### Step 2: Replace the ```LaunchDescription``` in the launch file with ```fogros2.FogROSLaunchDescription```
After running through this step, the launch file should now look like this:
```
import fogros2
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = fogros2.FogROSLaunchDescription()

    listener_node = Node(
        package="fogros2_examples", executable="listener", output="screen"
    )
    talker_node = Node(
        package="fogros2_examples", executable="talker", output="screen"
    )
    ld.add_action(talker_node)
    ld.add_action(listener_node)
    return ld
```

### Step 3: Specify AWS EC2 machine parameters by creating a ```fogros2.AWSCloudInstance``` object 
The ```AWSCloudInstance``` that needs to be created will be called machine, as follows:
```aws_machine = fogros2.AWSCloudInstance(region="us-west-1", ec2_instance_type="t2.micro", ami_image=ami-0b6030c78f8b2f076)```

The AWS region the machine will launch in is ```us-west-1``` with a t2.micro instance, which is low compute and free, and with an AMI that will make the machine run Ubuntu 22.04 (the AMI is one of the recommended ones stated earlier).

Incorporating this into the launch file,
```
import fogros2
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = fogros2.FogROSLaunchDescription()
    aws_machine = fogros2.AWSCloudInstance(
                    region="us-west-1", 
                    ec2_instance_type="t2.micro", 
                    ami_image=ami-0b6030c78f8b2f076
                )
    listener_node = Node(
        package="fogros2_examples", executable="listener", output="screen"
    )
    talker_node = Node(
        package="fogros2_examples", executable="talker", output="screen"
    )
    ld.add_action(talker_node)
    ld.add_action(listener_node)
    return ld
```

### Step 4: Replace all ```Node``` objects that need to be executed in the cloud with ```fogros2.CloudNode``` 

Since the talker node will be deployed on the cloud, the modification will be to the talker node only.
```
import fogros2
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = fogros2.FogROSLaunchDescription()
    aws_machine = fogros2.AWSCloudInstance(
                    region="us-west-1", 
                    ec2_instance_type="t2.micro", 
                    ami_image=ami-0b6030c78f8b2f076
                )
    listener_node = Node(
        package="fogros2_examples", executable="listener", output="screen"
    )
    talker_node = fogros2.CloudNode(
        package="fogros2_examples", executable="talker", output="screen", machine=aws_machine
    )
    ld.add_action(talker_node)
    ld.add_action(listener_node)
    return ld
```
Notice that in this example, the machine was set to the aws_machine that was created earlier.