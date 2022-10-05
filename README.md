## Installation

### ROS Installation:
Install appropriate ROS distribution depending on linux version: [ROS Download](http://wiki.ros.org/ROS/Installation)

### Intel Realsense Installation:
Install Intel Realsense from the following link: [Intel Realsense Installation](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)

Follow the instructions to install the pre-built packages, including dev and debug.

### OpenCV Installation:
This project leverages the Aruco AprilTag library from OpenCV, specifically the 36h11 tag family. This was introduced in the 3.4.2 version. Older versions of OpenCV will not be compatible with the current code. 

As a precaution, install the latest version of OpenCV from the following link: [OpenCV Installation](https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html). 

**Be sure to also include the OpenCV Contrib Libraries**

### Eigen Installation:
Install the Eigen library by following the instruction in this link: [Eigen Installation](https://eigen.tuxfamily.org/dox/GettingStarted.html)

### Install Hark: 
To install the HARK Audio Library, visit the following link: [HARK Installation](https://hark.jp/install/linux/). Install from source to be able to edit the source code (we will change some sampling rate parameters later)
