import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/pranavdm/UMD/Sem-1/ENPM662/Projects/Project-0/install/turtlebot3_teleop'
