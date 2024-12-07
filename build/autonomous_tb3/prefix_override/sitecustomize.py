import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/kanishk/DRL_robot_navigation_ros2/install/autonomous_tb3'
