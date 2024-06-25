# Spot Bringup

This package is the main entrypoint into operating Spot, and will be the default go-to approach for basic ROS users. This package contains only two launch files: ```bringup.launch.py``` and ```bringup_with_nav.launch.py```. 

```bringup.launch.py``` launches the main ```spot_driver``` launch file as well as the robot description and robot state publisher. Therefore after launching this file you be able to the visualize Spot ROS sensor streams (body RGDB streams, LiDAR, robot model, etc.) in RViz. The robot will also respond to the ```/cmd_vel``` topic.

```bringup_with_nav.launch.py```, as the name suggests, launches not only the base bringup launch file, but also the main navigation launch file from the ```spot_navigation``` package. After launching this file, additional information will be available (assuming the navigation configuration has been properly set up - see the ```spot_navigation``` package for details). This includes the stored map, robot localization, global and local planners, as well as their respective costmaps.

In order to properly configure payloads for each unique robot, payload configuration is set using environment variables. For instance, if the spot robot is equipped with an arm and the EAP package, you would set the following in your `.bashrc` (or wherever else makes sense for your application)

```bash
export SPOT_ACCESSORIES='ARM EAP'
```

Each payload is written in all capital letters and separated by a single space. Currently supported options are 

- EAP
- EAP2
- ARM
- CAM
- REALSENSE

**Note**: the CAM option is not supported in the URDF at this time, though the software interface is partially supported