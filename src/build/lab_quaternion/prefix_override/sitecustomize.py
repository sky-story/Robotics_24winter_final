import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ningbo@netid.washington.edu/robotics/my_ros2_ws/src/install/lab_quaternion'
