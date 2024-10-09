#!/usr/bin/env python
import numpy as np
import rospy
import os, sys

# import files from the parent directory
parrentdir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, parrentdir)

from utils.derm import *

from matplotlib import pyplot as plt
from matplotlib import rcParams
params={ 'font.family':'Times New Roman', 
#                     # 'font.style':'italic',
#                     'font.weight':'normal', #or 'bold'
                "mathtext.fontset": 'stix',
                'font.size': 20, #or large,small
                'pdf.fonttype': 42,
                'ps.fonttype': 42
                }
rcParams.update(params)

# -----------------------------------------------------------------

num_fps = rospy.get_param("dlo_configs/num_fps")
project_dir = rospy.get_param("project_configs/project_dir")
data_dir = project_dir + "data/code_test/vis_derm_projection/"

dlo_state_rand = np.load(data_dir + "rand.npy")
dlo_state_opt = np.load(data_dir + "opt.npy")

fig = plt.figure(figsize=(5, 5))
ax = fig.gca(projection='3d')

def plotDLOState(ax, dlo_state, label, color, plot_edge_frame=False):
    plotRod(ax, dlo_state[:3*num_fps], dlo_state[3*num_fps:3*num_fps+4], dlo_state[3*num_fps+4:3*num_fps+8], dlo_state[-1], 
            label=label, color=color, plot_edge_frame=plot_edge_frame, axessize=0.02)

plotDLOState(ax, dlo_state_rand, label="Random sample", color='#393e46')
plotDLOState(ax, dlo_state_opt, label="After projection", color='#e84545', plot_edge_frame=True)
ax.dist = 9.5
ax.azim = -160
ax.elev = 20
plt.legend(bbox_to_anchor=(1.0, 1.05), loc='upper right')

set_aspect_equal_3d(ax)
ax.tick_params(axis='both', which='major', labelsize=0, colors='white')

plt.subplots_adjust(left=0.00, bottom=0.00, right=1.0, top=0.95, wspace=0.0, hspace=0)

plt.show()