# The mass of the payload
float32 total_mass

# The center of mass of the payload defined in the payload frame
geometry_msgs/Vector3 center_of_mass

# Moment of inertia of hte payload, arranged as [xx, yy, zz, xy, xz, yz]
float32[6] moment_of_inertia

# Collision bounding boxes of the payload
# All entries must be of type shape_msgs::SolidPrimitive::BOX
# All boxes are represented in the payload frame
shape_msgs/SolidPrimitive[] bounding_boxes

# The poses of each of the bounding boxes, defined in the payload frame
geometry_msgs/Pose[] box_poses
