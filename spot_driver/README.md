# Spot Driver

This packages handles low level control of the Spot robot and provides a bridge between the original Boston Dynamics python API and ROS2. 

## Hierarchy

### Spot Wrapper
The ```SpotWrapper``` python class handles the interface with the Boston Dyamics API, including claiming a Lease on the robot, managing internal robot state management, access to cameras, managing the E-stop, etc. This interface also manages most other commands you would normally send to the robot via the tablet controller such as docking, pose commands, and instantaneous velocity commands.

### SpotROS
The Boston Dynamics API uses Google protobuf to publish messages internally, which is similar but incompatible with the communication means of both ROS1 and ROS2. The ```SpotROS``` python ROS2 node therefore acts as the bridge between the two architectures. Using the ```SpotWrapper``` interface, ```SpotROS``` performs tasks such as publishing TF data, odometry, camera topics, battery state, etc. to the ROS2 framework. It also provides ROS access to built-in commands like those present on the control tablet through the contained ```SpotWrapper``` instance. These are all available as ROS services. Finally, this wrapper subscribes to the traditional ```~/cmd_vel``` topic for robot mobililty commands, as well as a ```~/body_pose``` topic which commands the robot body's elevation and orientation (note that x and y components of the pose are ignored).

As a convenience, the ```ros_helpers.py``` library provides one-line conversions between native BD data types and ROS message/service types. This includes transforms, images, joint states, as well as custom status messages like dock state and power state.

### SpotDriver
```spot_driver.py``` is a ROS2 executable script which simply creates an instance of ```SpotROS``` and attempts to claim a Lease on the designated robot, and then spins until shutdown. Note that the ```exec``` token for this executable is simply called ```driver```. For most users, this will be your point of entry in controlling Spot through ROS. However, check the ```SpotBringup``` package for a more streamlined experience which also brings up the robot description for use in RViz.

### Launch

This package contains a single launch file ```driver.launch.py``` which runs the ```driver``` executable with all provided arguments, and additionally brings up the Velodyne LiDAR in Spot's payload. See the documentation for the Velodyne ROS interface [here](https://index.ros.org/p/velodyne/github-ros-drivers-velodyne/). 