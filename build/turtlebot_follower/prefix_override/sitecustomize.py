import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/eth/turtlebot_follow_ws/install/turtlebot_follower'
