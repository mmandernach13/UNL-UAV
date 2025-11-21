# UavPos Type Constants
uint8 UAV_POS_TYPE_TAKEOFF = 0
uint8 UAV_POS_TYPE_WAYPOINT = 1
uint8 UAV_POS_TYPE_LAND = 2

uint64 stamp  
string frame_id
float32[3] pos
float32 yaw
uint8 type
