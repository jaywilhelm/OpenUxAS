import struct, array, os, zmq,time, sys, json, argparse
import pickle
import math
# For Dubins Vehicle
from matplotlib import pyplot as plt
import numpy as np
from dubinsUAV import dubinsUAV
from AStar import a_star_planning

adata = pickle.load(open('astar_input.p', 'rb'))

start = adata['start']
goal = adata['goal']
border = adata['border']
koz = adata['koz']
scalefactor = adata['scale']
offset = adata['offset']
#bool(uavh_other.staticAreaLength))

ox, oy = [], []
for pt in border:
    ox.append(pt[0])
    oy.append(pt[1])

for pt in koz:
    ox.append(pt[0])
    oy.append(pt[1])



INTERVAL_SIZE = 1
SHOW_ANIMATION = True
path_x, path_y = a_star_planning(start[0], start[1],
goal[0], goal[1],
ox, oy,
INTERVAL_SIZE, (2 * INTERVAL_SIZE))

path_x += offset[0]
path_x /= scalefactor
path_y += offset[1]
path_y /= scalefactor

start += offset
start /= scalefactor

goal += offset
goal /= scalefactor

ox += offset[0]
ox /= scalefactor

oy += offset[1]
oy /= scalefactor

print(path_x)
print(path_y)
plt.plot(path_y, path_x, "xb", label='Path')
plt.plot(oy, ox, ".k", label='Search Area Obstacles')
plt.plot(start[1], start[0], "xg", label='UAV0 Position')
plt.plot(goal[1], goal[0], "xr", label='UAV0 Goal')
plt.grid(True)
plt.axis("equal")
plt.show()
plt.pause(0.0000001)


