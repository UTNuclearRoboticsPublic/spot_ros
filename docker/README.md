# Building

The following environment variables are used during the build process:

 - TARGET_ROS_DISTRO - The ROS distro for which you want the driver built. This driver has been mainly tested on ROS2 Humble, but later version should also work.
 - SPOT_VERSION - The version number of the Boston Dyanmics Spot API to install with the driver. This should match the firmware version of your robot. Leave this blank to install the most recent version.

```bash
export TARGET_ROS_DISTRO=humble
export SPOT_VERSION=4.1.0
docker compose build
```

# Running Spot with Docker

Please make sure you have the following before attempting to run Spot with Docker:

 1. A computer connected to Spot, which we will call the host. This can be the mounted payload, or it can be your own personal computer as long as you are connected to Spot's WiFi hotspot.
 2. Docker installed on the host
 3. Spot's username and password set as environment variables on the host. You can find these on Stache. Set them to the variables `BOSDYN_CLIENT_USERNAME` and `BOSDYN_CLIENT_PASSWORD`
 4. **[Optional]** The `SPOT_ACCESSORIES` environment variables indicating which standard accessories are on Spot. Important ones might be `ARM` and `RL_KIT`. Other options include `EAP` and `EAP2`. These values should be separated by spaces, eg. `export SPOT_ACCESSORIES='ARM EAP2'`
 5. **[Optional]** The `SPOT_URDF_EXTRAS` environment variable indicating which non-standard accessories are on Spot. The only valid option in this image is `/colcon_ws/src/spot_ros/spot_description/urdf/accessories/rl_kit_velodyne_mount`, which adds the Velodyne VLP16 LiDAR to the RL kit. If you wish to add custom attachments to Spot, you will need to extend this image or mount the add-on xacro file and its supporting mesh files as a volume.

 ## Bringup

 To bring up the main spot driver, you can use the following services

```bash
# with a controller attached to the host
# controller_configuration options are [Logitech, Dualsense]
docker compose run --rm bringup controller_configuration:=Dualsense

# or with no controller attached to host
docker compose up bringup_autonomous 
```

In general, these will use the standard robot hostname of `192.168.50.3`. However, if you are on the robot's wifi, you should add `hostname:=192.168.80.3` and if you are connected directly to the robot via ethernet, you can also use `hostname:=10.0.0.3`. For a more detailed view of the available options which include namespace options and lidar networking options, run 

```bash
docker compose run --rm bringup_autonomous --show-args
```

## Navigation with Nav2

The main thing that you need to run navigation is an up-to-date map of Spot's environment. We do our best to keep the default map of the AHG lab current, but things get moved around frequenntly so even so you may still need to remap. To create a new map, you can run the `slam` service.

```bash
docker compose up slam
```

Walk the robot around to create map, and remember to keep yourself moving relative to the map so that mapping algorithm can ignore you properly. Once the map is created, run the `save_map` service to save the map as `/tmp/spot_nav_map.pgm` alongside the config file `/tmp/spot_nav_map.yaml`. You are then free to copy these files to wherever you wish and rename them however you want. Keep in mind when renaming that you also need to edit the name of the PGM file in the `image` header in the general config file, otherwise the map will fail to load. 

```bash
docker compose up save_map
cp /tmp/spot_nav_map.yaml colcon_ws/src/my_awesome_project/maps/my_place.yaml
cp /tmp/spot_nav_map.pgm colcon_ws/src/my_awesome_project/maps/my_place.pgm
# Don't forget to now change the first line of my_place.yaml from 'image: spot_nav_map.pgm' to 'image: my_place.pgm'
```

## Manipulation with MoveIt

[Coming Soon]