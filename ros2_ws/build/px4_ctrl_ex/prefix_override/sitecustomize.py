import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/uav/uav/ros2_ws/install/px4_ctrl_ex'
