import sys
if sys.prefix == 'c:\\python38':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = 'C:\\dev\\MAE148FinalProject\\install'
