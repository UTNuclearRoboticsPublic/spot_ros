# Spot ROS
This repository houses the collection of packages required to run the Spot robot with ROS. The current version supports ROS Humble. If you wish to also use the official Spot Arm attachment, then you will also need to clone and build the [spot_manipulation](https://github.com/UTNuclearRoboticsPublic/spot_manipulation.git) package.

# Launching the Spot Driver

### Set Environment Variables
1. **Set Robot Configuration State**
   
    Spot can have many different attachments and accessories, and this configuration is managed with environment variables. Set the environment variable `SPOT_ACCESSORIES` to a string of space-separated identifiers for each accessory. Currently supported accessories are:
    
    | Accessory                         | Identifier  |
    |-----------------------------------|-------------|
    | Spot Arm                          | `ARM`       |
    | Extended Autonomy Package         | `EAP`       |
    | Updated Extended Autonomy Package | `EAP2`      |
    | Reinforcement Learning Kit        | `RL_KIT`    |
    | Wrist Mounted Realsense Camera    | `REALSENSE` |
    
    For example, using the Spot at NRG, you would add the following to your `~/.bashrc`
    ```bash
    export SPOT_ACCESSORIES="ARM RL_KIT" # in any order
    ```

    Additional components can be attached to spot via the `SPOT_URDF_EXTRAS` environment variable. This is a full path to a xacro file which is imported into the robot xacro, and so it must attach to a known link on the robot. For example:

   ```bash
   export SPOT_URDF_EXTRAS=$(ros2 pkg prefix --share spot_proprietary_description)/urdf/rl_kit_velodyne_mount.xacro
   ```

3. **Set Login Credentials**
   
    Add your Boston Dynamics login credentials to your `~/.bashrc` as environment variables.
    > NRG Users can find these credentials on [Stache](https://stache.utexas.edu/)
    
    ```bash
    export BOSDYN_CLIENT_USERNAME=
    export BOSDYN_CLIENT_PASSWORD=
    ```

4. **Set & Configure DDS Middleware**

   Depending on your desired communication configuration, you may want to set a DDS middleware configuration like `cyclonedds`. To do this, create your configuration file and set the required environment variables to your `~/.bashrc`.


### Driver Launch Commands
To launch the driver for Spot, execute the following command in a terminal:

```bash
ros2 launch spot_bringup bringup.launch.py hostname:=192.168.50.3 controller_configuration:=Dualsense5
```

This launch file accepts a number of launch arguments, including those for nested launch files such as the realsense launch file. To see all available options, run

```bash
ros2 launch spot_bringup bringup.launch.py --show-args
```
Some common arguments are summarized here:
- `publish_images`, possible values: `True`, `False`
- `publish_depth_images`, possible values: `True`, `False`
- `launch_pointcloud_service`, possible values: `True`, `False`
- `controller_configuration`, possible values: `Dualsense5`, `Logitech`
- `use_proprietary_meshes`, possible values: `True`, `False`

## Running the Kinematic Simulation
This package provides a convenience kinematics simulation for navigation, motion planning, teleoperation, or other similar applications. It is fairly limited in scope, however, and cannot detect or react to collisions or dynamic objects, making it unsuitable for testing pick-and-place, door opening, or other environment interaction focused tasks. 

The simulation works by projecting rays from the robot's sensors to a pre-defined mesh or primitive environment to simulate sensor data. It also reacts to standard robot commands such as the `/spot_driver/cmd_vel` topic, the `/spot_manipulation_driver/stow` and `/spot_manipulation_driver/unstow` services, and the `/follow_joint_trajectory` action server, among others.

To install dependencies run the following commands from your colcon workspace root directory

```bash
python3 -m pip install -U open3d "scipy>=1.8"
rosdep install --from-paths src/spot_ros/spot_simulation -i -y
```

Also due to a bug in the Humble release of the [Generate Parameters Library](https://github.com/PickNikRobotics/generate_parameter_library), it is necessary to build from source for proper parameter generation

```bash
git clone -b humble https://github.com/PickNikRobotics/generate_parameter_library 
```

Additionally, you will want to configure your simulation environment with objects and sensors. Refer to the  sample [environment configuration file](spot_simulation/config/environment_config.yaml) for examples on configuring the environment and the [robot configuration file](spot_simulation/config/spot_config.yaml) for examples of how to modify the robot and sensors. The simulation pulls the same accessories and URDF extensions and the main description launch. To run the simulation, execute

```bash
ros2 launch spot_simulation simulation.launch.xml
```

You can then move the robot around by publishing to the `/spot_driver/cmd_vel` topic and you should see the sensor data react to the robot's position. If you are also running the arm, you can control the arm using the MoveIt interface through 

```bash
ros2 launch spot_moveit_config moveit_rviz.launch.py
```

You should then see the hand depth data react to the pose of the end effector.

## Gamepad Mapping for Dualsense5

Refer to the table below for the button mappings to command the robot with a DualSense5 gamepad. The driver must be launched with the bringup launch file for these to have any effect.

| Command                 | Button(s)              | Notes                                                                             |
|-------------------------|------------------------|-----------------------------------------------------------------------------------|
| Hard EStop              | X+O+◻+△+R1+L1          | This will drop the robot unceremoniously. Only use in emergencies                 |
| Soft EStop              | O                      | The robot will stop whatever it is doing, sit down, and power off                 |
| Freeze Estop            | X                      | The robot will stop whatever it is doing and refuse any further commands           |
| Claim Lease             | Start/Options          | This is required prior to any command which causes the robot to move              |
| Release Lease           | Select/Share           | The robot will settle before releasing the lease                                  | 
| Power On                | △                      | ---                                                                               |
| Power Off               | ---                    | This effect can be achieved by triggering the soft estop                          |
| Undock                  | Right Stick            | The right stick is pressable as a button - that's what this means                 |
| Dock                    | Left Stick             | Hard coded to Dock ID 520                                                         |
| Move Robot              | L1 + Sticks            | Left stick is position, right stick is yaw rotation                               |
| Adjust Body Pose        | L2 + Sticks            | Left stick is body height, right stick is body orientation                        |
| Move Arm                | R1 + Sticks            | Left stick moves in the XY plane, right stick moves up and down                   | 
| Adjust Hand Orientation | R2 + Sticks            | Left stick is pitch and yaw, right stick is roll                                  | 
| Unstow Arm              | D-Pad Right            | The arm unstows far in front of the robot, just pressing R2 does a smaller unstow | 
| Stow Arm                | D-Pad Left             | This can result in fairly erratic movements if the arm is at an awkward angle     | 
| Sit Robot               | D-Pad Down             | ---                                                                               | 
| Stand Robot             | D-Pad Up               | ---                                                                               | 
| Gripper Toggle          |  ◻                     | Toggle gripper open and close                                                     | 

## Gamepad Mapping for Logitech

Refer to the table below for the button mappings to command the robot with a Logitech gamepad.

| Command                 | Button(s)              | Notes                                                                             |
|-------------------------|------------------------|-----------------------------------------------------------------------------------|
| Hard EStop              | A+B+X+Y+RB+LB          | This will drop the robot unceremoniously. Only use in emergencies                 |
| Soft EStop              | B                      | The robot will stop whatever it is doing, sit down, and power off                 |
| Freeze Estop            | A                      | The robot will stop whatever it is doing and refuse any further commands           |
| Claim Lease             | Start                  | This is required prior to any command which causes the robot to move              |
| Release Lease           | Back                   | The robot will settle before releasing the lease                                  | 
| Power On                | Y                      | ---                                                                               |
| Power Off               | ---                    | This effect can be achieved by triggering the soft estop                          |
| Undock                  | Right Stick            | The right stick is pressable as a button - that's what this means                 |
| Dock                    | Left Stick             | Hard coded to Dock ID 520                                                         |
| Move Robot              | LB + Sticks            | Left stick is position, right stick is yaw rotation                               |
| Adjust Body Pose        | Left Trigger + Sticks  | Left stick is body height, right stick is body orientation                        |
| Move Arm                | RB + Sticks            | Left stick moves in the XY plane, right stick moves up and down                   | 
| Adjust Hand Orientation | Right Trigger + Sticks | Left stick is pitch and yaw, right stick is roll                                  | 
| Unstow Arm              | D-Pad Right            | The arm unstows far in front of the robot, just pressing RB does a smaller unstow | 
| Stow Arm                | D-Pad Left             | This can result in fairly erratic movements if the arm is at an awkward angle     | 
| Sit Robot               | D-Pad Down             | ---                                                                               | 
| Stand Robot             | D-Pad Up               | ---                                                                               | 
| Gripper Toggle          |  X                     | Toggle gripper open and close                                                     | 

# Running ROS Navigation 
Requirement: Ensure the environment variable `SPOT_ACCESSORIES` includes `RL_KIT`, and `SPOT_URDF_EXTRAS` is set to the Velodyne mount, as discussed in the [Set Environment Variables](#set-environment-variables) section.

Spot is configured to use the ROS2 navigation stack, Nav2. Official documentation for Nav2 can be found [here](https://docs.nav2.org/). To localize spot within NRG's AHG labspace, make sure the robot is docked (to match the initial location) an execute
```bash
ros2 launch spot_navigation amcl.launch.py map:=<path_to_map/map.yaml>
```

To command the robot using Nav2, run the following in a separate terminal.
```bash
ros2 launch spot_navigation bringup_launch.py
```

# Commanding Spot in a Behavior Tree
The `spot_behaviors` package provides a library of basic commands that can be sent to spot in a behavior tree. Note that some behaviors require other services to be running such as Nav2, MoveIt and/or the [Manipulation Driver](https://github.com/UTNuclearRobotics/nrg_spot_manipulation). Others such as `dock_robot` or `check_battery` require only that the Spot Driver is running. The current list of behvaiors is 

| Behavior | Inputs | Outputs | Effect | 
|----------|--------| ------- | ------ | 
| `CheckArmStowed` | --- | --- | Returns `SUCCESS` if the arm is stowed and `FAILURE` otherwise | 
| `CheckBattery` | `battery_threshold` | --- | Returns `SUCCESS` if the battery percentage is above the mandatory `battery_threshold` input port, and `FAILURE` otherwise |
| `DockRobot` | `dock_id` | --- | Triggers the robot to dock, and return `SUCCESS` if the dock action was successful |
| `MoveHandToPose` | `target_pose` `target_frame` `planning_group` | --- |  Commands a general non-cartesian motion to the target pose using MoveIt |
| `MoveHandThroughPoses` | `waypoints` `position_tolerance` `angular_tolerance` | --- | Uses MoveIt to command the hand to perform cartesian motions through a set a wayposes. If cartesian motions are not possible, a non-cartesian reconfiguration motion will be commanded to the next pose. If even that is not possible, the node is skipped and tries again with the next one. Always returns `SUCCESS` |
| `TriggerService` | `service_name` `timeout` `empty` | --- | Calls a service with `std_srvs/Trigger` (or `std_srvs/Empty` if `empty` is True) and waits for `timeout` seconds for a response. Returns the success value of the response (always `SUCCESS` for Empty), or `FAILURE` if no response is received |
| `WalkToPose` | `target_pose` | --- | Command the robot to walk to a given pose using the Boston Dynamics API | 
| `GetImages` | `camera_names` | `image_list` | Get images from requested cameras. Xml Example: `<GetImages camera_names='LEFT; RIGHT' image_list="{image_list}" />` | 
| `GetGripperHoldingState` | --- | `is_holding` | Query whether the gripper is holding something. Xml Example: `<GetGripperHoldingState is_holding="{is_holding}" />` | 
| `GetStowState` | --- | `is_stowed` | Query whether the arm is stowed. Xml Example: `<GetStowState is_stowed="{is_stowed}" />` | 

# Proprietary Mesh Usage
If you have access to NRG’s private [spot_proprietary](https://github.com/UTNuclearRobotics/spot_proprietary.git) repository, clone and build it to enable higher-fidelity meshes for Spot. These meshes can be activated by passing the launch argument `use_proprietary_meshes:=True` when launching Spot bringup or loading the robot description. Alternatively, if you have your own higher-fidelity meshes for the arm and accessories (e.g., EAP, EAP2, RL Kit), you can use them by placing them in a separate ROS package—such as `spot_proprietary_description`—with the following structure:

```text
spot_proprietary_description/
└── meshes/
    ├── arm/
    └── accessories/
```

The mesh files must follow specific naming conventions. Refer to the following files—look for sections tagged with `<xacro:if value="$(arg use_proprietary_meshes)">`—to ensure compatibility:

- **Arm meshes**: `spot_description/urdf/spot.urdf.xacro`
- **RL Kit**: `spot_description/urdf/accessories/rl_kit.xacro` and `spot_description/urdf/accessories/rl_kit_velodyne_mount.xacro`
- **Other accessories**: `spot_description/urdf/spot_arm.urdf.xacro`

To use the proprietary meshes, launch `spot_bringup` or any robot description launch file with the following arguments:

- `proprietary_pkg` – e.g., `spot_proprietary_description`
- `proprietary_mesh_format` – mesh format: `dae`, `stl`, or `obj`
- `use_proprietary_meshes` – set to `True` to enable them

## Authors
Janak Panthi (aka Crasun Jans), Alex Navarro, Kevin Torres, and Blake Anderson
