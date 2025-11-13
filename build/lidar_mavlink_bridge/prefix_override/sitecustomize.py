import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/enelsis-pc/enelsis_moab_ws/install/lidar_mavlink_bridge'
