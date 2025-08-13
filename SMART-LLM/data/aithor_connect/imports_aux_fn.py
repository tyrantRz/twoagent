import math
import re
import shutil
import subprocess
import time
import threading
import cv2
import numpy as np
from ai2thor.controller import Controller
from scipy.spatial import distance
from typing import Tuple
from collections import deque
import random
import os
from glob import glob
import sys
BASE_DIR = os.path.dirname(os.path.abspath(sys.argv[0]))



def closest_node(node, nodes, no_robot, clost_node_location):
    crps = []
    distances = distance.cdist([node], nodes)[0]
    dist_indices = np.argsort(np.array(distances))
    total = len(dist_indices)
    for i in range(no_robot):
        k = clost_node_location[i] if i < len(clost_node_location) else 0
        pos_index = (i * 5) + k
        if pos_index >= total:
            pos_index = total - 1
        crps.append(nodes[dist_indices[pos_index]])
    return crps

def distance_pts(p1: Tuple[float, float, float], p2: Tuple[float, float, float]):
    return ((p1[0] - p2[0]) ** 2 + (p1[2] - p2[2]) ** 2) ** 0.5

def generate_video():
    frame_rate = 5
    # input_path, prefix, char_id=0, image_synthesis=['normal'], frame_rate=5
    cur_path = os.path.join(BASE_DIR, "agent_*")
    for imgs_folder in glob(cur_path, recursive = False):
        view = os.path.basename(imgs_folder.rstrip("/"))
        if not os.path.isdir(imgs_folder):
            print("The input path: {} you specified does not exist.".format(imgs_folder))
        else:
            command_set = [
                'ffmpeg', '-i', 
                f'{imgs_folder}/img_%05d.png',
                '-framerate', str(frame_rate),
                '-pix_fmt', 'yuv420p',
                os.path.join(BASE_DIR, f'video_{view}.mp4')]
            subprocess.call(command_set)
        


