## Installation Dependencies

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
To install the HARK Audio Library, visit the following link: [HARK Installation](https://hark.jp/install/linux/). Install from source to be able to edit the source code (we will change some sampling rate parameters later). Make sure to install all the listed libraries!

Install hark_designer and harktool5_gui using prebuilt packages:
```
sudo apt install hark-designer
sudo apt install harktool5 harktool5-gui
```

### Install ROS for HARK:
To install the HARK ROS Bridge: [HARK ROS](https://hark.jp/hark-ros-msgs-installation-instructions/).
**Make sure to install from source**
Once you install the hark-ros-msgs directory, cd into it and run `catkin_make`. This will create a devel folder.


### Change Code Parameters
hark/hark-linux-3.3.0/librecorder/ALSARecorder.cpp
Change buffer time to 30000 in line 138:

```
unsigned buffer_time = 500000 //Change this to 30000 to enable real-time processing
```

## LiDAR Code

## File Structure
The src folder is divided into two folders, **cam_launch**, and also **combine_cloud**. **cam_launch** handles the bulk of the LiDAR point cloud generation and processing, outputting the aggregated separated point clouds. There are two cpp files in **cam_launch/src**, **launcher1.cpp** and **launcher2.cpp** which are meant to interface with two separate LiDAR cameras. They both will stream the information to the combine_cloud node. 

The **combine_cloud** node subscribes to the message topic of both LiDAR cameras, performs some minute adjustments to line up the two point clouds, and then publishes the aggregated point clouds under one topic. At the end of **combine_cloud**, we should have 



## Compiling the code
Follow tutorial listed [here](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) to create a catkin_workspace. After running `catkin_make` once, download the project files, and then copy the src directory into the catkin_ws directory in the previous step, replacing the src folder created by `catkin_make`.

Copy the hark message files in **{HARK DOWNLOAD DIRECTORY}/hark-ros-msgs-{VERSION}/devel/include** to **catkin_ws/devel/include**.

Run catkin_make another time. This should now build with no errors!

## Running the Code

Run the following command with a realsense camera to make sure everything is functional: 
```
rosrun cam_launch cam_launch1
```
If you run into errors, make sure there is a roscore running, and also make sure to source the devel/setup.bash file.

This will print out the camera model number. This will be different from the default camera number in the **src/cam_launch/src/launcher1.cpp** file. Change the 
`CAM1_NAME` parameter on line 75 to the appropriate serial number. 

Recompile with `catkin_make` and run with `rosrun cam_launch cam_launch1`. Exit the scene while the terminal prints initializing, and once the latency is displayed, you may enter the scene. 

## Viewing Point Clouds

You can view the subtracted point clouds by utiling **RVIZ**. RVIZ comes with ROS installation. Run RVIZ with `rviz` in any terminal.

Subscribe to the appropriate topic. Note that the point cloud will not display due to an undefined coordinate frame. Establish a coordinate frame by running:
```
rosrun tf static_transform_publisher 0 0 0 0 0 0 map backgone1 30
```
on any terminal, making sure to source the devel/setup.bash directory if it is not found.

The separated point cloud should now be shown on RVIZ
