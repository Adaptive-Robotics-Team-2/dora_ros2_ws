import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/dora/dora_ros2_ws/robot_ws/src/install/wheel_controller'
