# UavPos Type Constants
uint8 TAKEOFF = 0
uint8 WAYPOINT = 1
uint8 LAND = 2

uint64 stamp  
string frame_id
float32[3] pos
float32 yaw
uint8 type
