import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/brian_holland/Desktop/capstone/ros_ws/install/ht301_thermal_ros'
