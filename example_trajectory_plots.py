# -*- coding: utf-8 -*-

"""
This script takes two trajectory input files which are specified in the data loading section and plots them.  
"""

import os
import matplotlib.pyplot as plt

from evo.tools import file_interface
from evo.core.trajectory import PoseTrajectory3D

###########################
# data loading

file_path = os.path.join(os.path.dirname(__file__), "..", "..", "devel", "lib", "orb_slam2_ros")
file_one = "slam_FrameTrajectory_TUM_Format.txt"
file_two = "localization_FrameTrajectory_TUM_Format.txt"
file_path_one = os.path.join(file_path, file_one)
file_path_two = os.path.join(file_path, file_two)

poses = file_interface.read_tum_trajectory_file(file_path_one)
x = poses.positions_xyz[:, 0]
y = poses.positions_xyz[:, 2]

poses_two = file_interface.read_tum_trajectory_file(file_path_two)
x_two = poses_two.positions_xyz[:, 0]
y_two = poses_two.positions_xyz[:, 2]

#############################
# configure plot
fig, ax = plt.subplots(1, 1)
fig.tight_layout()

plt.plot(x,y,  color="#0065BD", linewidth=3)
plt.plot(x_two,y_two,  color="#E37222", linewidth=2)

label_one = "          SLAM at 1x speed"
label_two = "Localization at 2x speed"
leg = plt.legend([label_one, label_two])

xlabel = "x in m"
ylabel = "y in m"
ax.set_xlabel(xlabel)
ax.set_ylabel(ylabel)

# plt.grid()

plt.gca().set_aspect('equal', adjustable='box')

plt.show()

