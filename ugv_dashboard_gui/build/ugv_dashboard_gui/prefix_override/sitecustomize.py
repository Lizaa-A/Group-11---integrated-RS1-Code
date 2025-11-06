import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/liv/rs1_2025_clone_ws/src/rs1_2025_soil/ugv_dashboard_gui/install/ugv_dashboard_gui'
