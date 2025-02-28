# Spot ROS
This repository houses the collection of packages required to run the Spot robot with ROS. The current vesion supports ROS Humble. If you wish to also use the official Spot Arm attachment, then you will also need to clone and build the [nrg_spot_manipulation](https://github.com/UTNuclearRobotics/nrg_spot_manipulation) package.

# Launching the Spot Driver

### Set Environment Variables
1. **Set Robot Configuration State**
   
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

2. **Set Login Credentials**
   
    Add your Boston Dynamics login credentials to your `~/.bashrc` as environment variables.
    > NRG Users can find these credentials on [Stache](https://stache.utexas.edu/)
    
    ```bash
    export BOSDYN_CLIENT_USERNAME=
    export BOSDYN_CLIENT_PASSWORD=
    ```

3. **Set & Configure DDS Middleware**

   Depending on your desired communication configuration, you may want to set a DDS middleware configuration like `cyclonedds`. To do this, create your configuration file and set the required environment variables to your `~/.bashrc`.


### Driver Launch Commmands
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
## Gamepad Mapping for Dualsense5

Refer to the table below for the button mappings to command the robot with a Logitech gamepad. The driver must be launched with the bringup launch file for these to have any effect.

| Command                 | Button(s)              | Notes                                                                             |
|-------------------------|------------------------|-----------------------------------------------------------------------------------|
| Hard EStop              | X+O+◻+△+R1+L1          | This will drop the robot unceremoniously. Only use in emergencies                 |
| Soft EStop              | O                      | The robot will stop whatever it is doing, sit down, and power off                 |
| Freeze Estop            | X                      | The robot will stop whatever it is doing and refuse any futher commands           |
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
| Unstow Arm              | D-Pad Right            | The arm unstows far in front of the robot, just pressing RB does a smaller unstow | 
| Stow Arm                | D-Pad Left             | This can result in fairly erratic movements if the arm is at an awkward angle     | 
| Sit Robot               | D-Pad Down             | ---                                                                               | 
| Stand Robot             | D-Pad Up               | ---                                                                               | 
| Gripper Toggle          |  ◻                     | Toggle gripper open and close                                                     | 

## Gamepad Mapping for Logitech

Refer to the table below for the button mappings to command the robot with a Logitech gamepad. The driver must be launched with the bringup launch file for these to have any effect.

| Command                 | Button(s)              | Notes                                                                             |
|-------------------------|------------------------|-----------------------------------------------------------------------------------|
| Hard EStop              | A+B+X+Y+RB+LB          | This will drop the robot unceremoniously. Only use in emergencies                 |
| Soft EStop              | B                      | The robot will stop whatever it is doing, sit down, and power off                 |
| Freeze Estop            | A                      | The robot will stop whatever it is doing and refuse any futher commands           |
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

## Authors
Janak Panthi (aka Crasun Jans), Alex Navarro, and Blake Anderson
