# Waypoint Type Constants
uint8 WAYPOINT_TYPE_TAKEOFF = 0
uint8 WAYPOINT_TYPE_WAYPOINT = 1
uint8 WAYPOINT_TYPE_LAND = 2
uint8 WAYPOINT_TYPE_PAYLOAD_DROP = 3

std_msgs/Header header
float32[3] position
float32 yaw
uint8 type