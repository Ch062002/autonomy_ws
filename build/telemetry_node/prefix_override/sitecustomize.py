import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/chirag/Aerospace/ROS2/autonomy_ws/install/telemetry_node'
