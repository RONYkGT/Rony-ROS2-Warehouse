import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rony/Documents/inmind/Session06/Rony-ROS2-Warehouse/install/warehouse_robot'
