import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/student/git/41068_ws/src/rs1_2025_soil/install/ugv_dashboard_gui'
