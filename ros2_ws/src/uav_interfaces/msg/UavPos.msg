# Position type constants
uint8 UAV_POS_TYPE_TAKEOFF = 0
uint8 UAV_POS_TYPE_WAYPOINT = 1
uint8 UAV_POS_TYPE_LAND = 2
uint8 UAV_POS_TYPE_PAYLOAD_DROP = 3

uint64 stamp # microseconds since start (matches PX4 timestamp)
string frame_id
float32[3] pos      # [x,y,z] in NED frame
float32 yaw         # direction in degrees
uint8 type          # what the current pose is doing