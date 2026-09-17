import sys
if sys.prefix == '/home/kalgaonp/anaconda3/envs/olins_ros':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/kalgaonp/vfh_adaptive/install/thesis_test'
