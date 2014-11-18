#!/usr/bin/env python

import os
import shutil
import tempfile
from os.path import expanduser


""" Set me """
catkin_ws_dir = '/home/robolab/catkin_ws'
swi_lib_dir    =      '/usr/lib/swipl-6.6.6/lib/x86_64-linux'
"""        """


shutil.copy2(catkin_ws_dir+'/devel/lib/libretalis_output_interface.so', swi_lib_dir+'/retalis_output_interface.so')
shutil.copy2(catkin_ws_dir+'/devel/lib/libswiglm.so', swi_lib_dir+'/swiglm.so')

