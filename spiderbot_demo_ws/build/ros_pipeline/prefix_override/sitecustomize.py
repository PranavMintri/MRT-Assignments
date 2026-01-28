import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/pranavmintri/spiderbot_demo_ws/install/ros_pipeline'
