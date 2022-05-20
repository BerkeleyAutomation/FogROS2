---
layout: page
title: Video Compression (H.264)
permalink: /video_compression
nav_order: 6
---

# Video Compression

One consideration that may pop up when utilizing the cloud for computation is streaming video or images for computer vision. Due to the large size of an uncompressed image, there may be adverse effects to round trip time or frames per second when streaming video in a bandwidth constrained environment. FogROS 2 is able to utilize ```image_transport``` to set up encoder and decoder nodes that compress and decompress the raw images. In particular, JPEG/PNG compression and the Theora compression format are supported through the ```image_transport_plugins```, and FogROS 2 adds additional support for H.264 encoding in ROS 2. H.264 decoding is provided through another community package.

Note that out of the options listed above only PNG is lossless. If the computer vision algorithm used in the cloud is not robust to the the lossy output of H.264, JPEG, Theora, then it is recommended to use PNG compression. Note that the encoding/decoding computational power increases from JPEG, Theora, to H.264, but the compression efficiency also increases in that order.

## PNG/JPEG/Theora Setup

Download the ```image_transport_plugins``` package by running ```sudo apt-get install ros-<distro>-image-transport-plugins```, where ```<distro>``` should be replaced by the ROS distribution installed.

## H.264 Setup

Since the H.264 image transport is not a publicly available ROS package, the repositories necessary will have to be manually downloaded.

1. Download the repository [linked here](https://github.com/BerkeleyAutomation/FogROS2/tree/main) using the command ```git clone -b main --recurse-submodules https://github.com/BerkeleyAutomation/FogROS2```.

2. Place only the subdirectories `ros2_h264_encoder` and `h264_image_transport` in the ROS 2 workspace. Everything else should be deleted. 

3. Install dependencies using the following command
```
sudo apt-get install libavdevice-dev libavformat-dev libavcodec-dev libavutil-dev libswscale-dev libx264-dev 
```

## General Setup

1. Identify all topics that need to be compressed. For sake of example, the topic `/camera/image_raw` will be used.

2. When modifying an existing launch file, and in Step 4 of this [guide]({{site.baseurl}}/launch_configuration), add a `stream_topics` argument to the `fogros2.CloudNode`. The `stream_topics` argument is a list of tuples, where the first argument in the tuple is the topic name, and the second argument is the transport/compression type. 
For the `/camera/image_raw` example, if H.264 was desired, the argument to `fogros2.CloudNode` would be `stream_topics=[('/camera/image_raw', 'h264')]`. Repeat this step for any/all topics that should be compressed, and add each topic as a separate tuple in the `stream_topics` list. 
Valid options for the 2nd element of a given tuple are `h264`, `theora`, or `compressed`. `compressed` will default to JPEG compression, so the ROS 2 parameter of `<base_topic>/compressed/format` needs to be changed to 'png'. More details are [here](http://wiki.ros.org/compressed_image_transport). 

3. The node on the cloud that is expecting to receive the raw images should have the listener's topic name changed from `<base_topic>`to `<base_topic>/cloud`. For example, the node that is subscribing to `/camera/image_raw` in the cloud should now listen for `/camera/image_raw/cloud`. This is the only change in the code that will have to be done to reduce any potential namespace conflicts. This step may become optional when automatic topic remapping is implemented. 