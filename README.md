# IFM O3Mxxx drivers for ROS

## Overview
This package includes the basic drivers for IFM O3Mxxx for publishing Pointcloud2 messges, depth image and raw message with sensor specific information.

## Installation
Download or clone the folder into you workspace. Run catkin_make.

## ROS API
### Published topics
* ifm_o3mxxx_pc ([sensor_msgs/PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html))
  Valid points of the PointCloud
  
* ifm_o3mxxx_raw (ifm_o3mxxx/IFMO3mxxx)
  Raw sensor information coming from IFM sensor
  
  * Header header
  * uint16[1024] distance_data
  * float32[1024] x
  * float32[1024] y
  * float32[1024] z
  * uint16[1024] confidence 
  * uint16[1024] amplitude
  * float32[4] amplitude_normalization
  * uint32 available
  * uint16 height
  * uint16 width
  * float32 trans_x
  * float32 trans_y
  * float32 trans_z
  * float32 rot_x
  * float32 rot_y
  * float32 rot_z

* ifm_o3mxxx/depth/raw ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))
  Depth image of valid points

### Parameters
* Port (int, default: 4599)
* IP (string, default: "255.255.255.255")
* frame_id (string, default: "ifm")
* OutDepthImageTopic (string, default: "ifm_o3mxxx_depth_image")
* PointCloudTopic (string, default: "ifm_o3mxxx_pc")
* RawTopic (string, default: "ifm_o3mxxx_raw")
  

