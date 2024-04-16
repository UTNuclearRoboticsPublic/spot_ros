# Positional Interpolaiton
uint8 POS_INTERP_UNKNOWN = 0
uint8 POS_INTERP_LINEAR = 1
uint8 POS_INTERP_CUBIC = 2

# Angular Interpolaiton
uint8 ANG_INTERP_UNKNOWN = 0
uint8 ANG_INTERP_LINEAR = 1
uint8 ANG_ITNERP_CUBIC_EULER = 2

SE3TrajectoryPoint points
builtin_interfaces/Time reference_time
uint8 pos_interpolation
uint8 ang_interpolation