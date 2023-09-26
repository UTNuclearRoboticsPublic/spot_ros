# Stow State
uint8 STOWSTATE_UNKNOWN = 0
uint8 STOWSTATE_STOWED = 1
uint8 STOWSTATE_DEPLOYED = 2

# Carry State
uint8 CARRY_STATE_UNKNOWN = 0
uint8 CARRY_STATE_NOT_CARRIABLE = 1
uint8 CARRY_STATE_CARRIABLE = 3
uint8 CARRY_STATE_CARRIABLE_AND_STOWABLE = 3

float64 gripper_open_percentage
bool is_gripper_holding_item
geometry_msgs/Point estimated_end_effector_force_in_hand
uint8 stow_state
#geometry_msgs/Twist velocity_of_hand_in_vision
#geometry_msgs/Twist velocity_of_hand_in_odom
uint8 carry_state