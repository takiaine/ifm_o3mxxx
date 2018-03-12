# IFM O3Mxxx drivers for ROS

## Overview
This package includes the basic ROS drivers for IFM O3Mxxx for publishing Pointcloud2 messages, depth image and raw message with sensor specific information. It has been tested with O3M150 and O3M250. The code is based on IFM sample codes for parsing the ethernet based communication data. The data is then repacked into ROS compatible formats. 

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

## TODO
Only the PointCloud2 information and raw sensor information has been used. The depth image has not been used much(other than visualization), so it should be tested before before you start using it. Also the image interface might need more work(mo calibration etc. is taken into consideration yet).

O3M151 and O3M251 produce object information instead of point cloud. These functionalities could (probably) be added to the driver but currently I do not have such decive at hand.
  
## Acknowledgement
This driver has been developed at [Tampere University of Technology](http://www.tut.fi/en/home), [Laboratory of Automation and Hydraulics(AUT)](http://www.tut.fi/en/research/research-fields/automation-and-hydraulic-engineering/index.htm)

