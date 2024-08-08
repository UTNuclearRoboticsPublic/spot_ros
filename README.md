# Spot ROS
This repository houses the collection of packages required to run the Spot robot with ROS. The current vesion supports ROS Humble. If you wish to also use the official Spot Arm attachment, then you will also need to clone and build the [nrg_spot_manipulation](https://github.com/UTNuclearRobotics/nrg_spot_manipulation) package.

# Launching the Spot Driver

Spot can have many different attachments and accesories, and this configuration is managed with environment variables. Set the environment variable `SPOT_ACCESSORIES` to a string of space-separated identifiers for each accessory. Currently supported accessories are:

| Accessory                         | Identifier  |
|-----------------------------------|-------------|
| Spot Arm                          | `ARM`       |
| Extended Autonomy Package         | `EAP`       |
| Updated Extended Autonomy Package | `EAP2`      |
| Wrist Mounted Realsense Camera    | `REALSENSE` |

For example, using the Spot at NRG, you would add the following to your `~/.bashrc`
```bash
export SPOT_ACCESSORIES="ARM EAP" # in any order
```

To launch the driver for Spot, execute the following command in a terminal:

```bash
ros2 launch spot_bringup bringup.launch.py hostname:=192.168.50.3
```

This launch file accepts a number of launch arguments, including those for nested launch files such as the realsense launch file. To see all available options, run

```bash
ros2 launch spot_bringup bringup.launch.py --show-args
```

## Gamepad Mapping

Refer to the table below for the button mappings to command the robot with a Logitech gamepad. The driver must be launched with the bringup launch file for these to have any effect.

| Command                 | Button(s)              | Notes                                                                             |
|-------------------------|------------------------|-----------------------------------------------------------------------------------|
| Hard EStop              | A+B+X+Y+RB+LB          | This will drop the robot unceremoniously. Only use in emergencies                 |
| Soft EStop              | B                      | The robot will stop whatever it is doing, sit down, and power off                 |
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
| Unstow Arm              | D-Pad Up               | The arm unstows far in front of the robot, just pressing RB does a smaller unstow | 
| Stow Arm                | D-Pad Down             | This can result in fairly erratic movements if the arm is at an awkward angle     | 
| Sit Robot               | A                      | When the robot is standing                                                        | 
| Stand Robot             | A                      | When the robot is sitting                                                         | 
| Open Gripper            | X                      | When the gripper is closed                                                        | 
| Close Gripper           | X                      | When the gripper is open                                                          |

# Running ROS Navigation 

Spot is configured to use the ROS2 navigation stack, Nav2. Official documentation for Nav2 can be found [here](https://docs.nav2.org/). To localize spot within NRG's AHG labspace, make sure the robot is docked (to match the initial location) an execute
```bash
ros2 launch spot_navigation amcl.launch.py
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
| `MoveHandThroughPoses` | `waypoints` `position_tolerance` `angular_tolerance` | --- | Uses MoveIt to command the hand to perform cartesian motions through a set a wayposes. If cartesian motions are not possible, a non-cartesian reconfiguration motion will be commanded to the next pose. If even that is not possible, the node is skipped and tries again with the next one. Always returns `SUCCESS` |
| `MoveHandToPose` | `target_pose` `target_frame` `planning_group` | --- |  Commands a general non-cartesian motion to the target pose using MoveIt |
| `NavigateToPose` | `target_pose` | --- | Commands spot through Nav2 to navigate to the given pose | 
| `RecordCurrentLocation` | `global_frame` `robot_frame` | `recorded_pose` | Records the pose of the robot frame in the global frame. Useful for returning to the dock |
| `TriggerService` | `service_name` `timeout` `empty` | --- | Calls a service with `std_srvs/Trigger` (or `std_srvs/Empty` if `empty` is True) and waits for `timeout` seconds for a response. Returns the success value of the response (always `SUCCESS` for Empty), or `FAILURE` if no response is received |
| `WalkToPose` | `target_pose` | --- | Command the robot to walk to a given pose using the Boston Dynamics API | 
