uint8 TAKEOFF = 0
uint8 WAYPOINT = 1
uint8 LAND = 2
unit8 RTL = 3

uint64 stamp        # time since start usec
string frame_id
float32[3] pos      # [x,y,z] in NED frame
float32 yaw         # direction in degrees
uint8 type          # what the current pose is doing