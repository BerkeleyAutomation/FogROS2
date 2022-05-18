---
layout: page
title: Cloud Robotics
permalink: /cloud_robotics
nav_order: 1
---

Cloud Robotics
===
Cloud robotics encompasses many relations between the cloud and robotics.  Here we focus on what the cloud offers, explicitly focusing on the capabilities FogROS 2 can give robots by using the cloud.

What is the cloud?
---
The cloud, in short, offers pay-per-use networked access to computing resources.  Cloud service providers, such as Amazon Web Services (AWS), Google Cloud Platform (GCP), and Microsoft Azure, set up and maintain computing hardware, such as multi-core servers, graphics processing units (GPUs), tensor processing units (TPUs), field-programmable gate arrays (FPGAs), and more.  To use the computing hardware, one signs up for the service using a credit card, then uses either a browser-based or program-based interface to turn on and off, configure, and access these computers.

Cloud service providers house their computers in different _data centers_ in different _regions_ around the world.  The reason for this will be explained next.

Cloud computing latency
---
Latency of computation is often critical in robot applications.  Whether computing a new path, reconstructing an environment, or planning a manipulation, it is desirable to do it fast.  Robots may find that computations are too slow when using their onboard computing capabilities alone.  Using high-end or hardware-accelerated computing in the cloud can speed up the computation but at the cost of the network round-trip time between robot and cloud.  Using the cloud thus only gains an advantage over robot-only computation when the combined cloud computing time and the network round-trip time is faster than robot-only.

For example, robots often do not have onboard GPUs, but many modern robot algorithms benefit significantly from GPU processing.  When a forward pass on a deep neural network takes 14 seconds on a robot's CPU and requires only 0.6 seconds on a GPU, the potential speedup using the cloud is significant.

Network latency to the cloud
---
The network latency to the cloud can be surprisingly short.  The most crucial factor is the distance between the robot and the cloud.  The farther away, the longer the latency (this is due to the speed of light).  A secondary factor is the bandwidth or bitrate of the network connection between the robot and the cloud.  

To understand bandwidth or bitrate, try out an internet speed test tool. 

To understand the latency to the cloud, try out [cloudping.info](https://cloudping.info/).  This tool sends a small packet to the different data centers and measures the time before the response comes back.  You can expect response times in the 10 to 40 ms range for nearby data centers.

With [cloudping.info](https://cloudping.info/), you can quickly understand which data center you should use for you cloud robotics applications.

What is the edge?
---
The edge refers to computing resources as close as possible to the robot.  As latency is often a critical factor in many applications, having computers closer to the robot is often desirable.  

What is the fog?
---
The fog encompasses everything between and including the cloud and the edge computing resources.

FogROS 2 and Cloud Robotics
---
FogROS 2 integrates ROS 2 applications with the cloud by streamlining the process of running ROS 2 nodes in the cloud and having them communicate with the robot.  FogROS 2 takes care of setting up cloud computers, installing ROS and dependencies, securing network communications, starting remote nodes, and more.  As a user, you will need only to configure which nodes get deployed to which region and computer type.
