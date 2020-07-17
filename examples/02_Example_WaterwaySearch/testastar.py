import struct, array, os, zmq,time, sys, json, argparse
import pickle
from scipy import stats
import math
# For Dubins Vehicle
from matplotlib import pyplot as plt
import numpy as np
from dubinsUAV import dubinsUAV
from AStar import a_star_planning
from UAVHeading import UAVHeading

# def convertPathToUniqueWaypoints(path_x, path_y):
#     path_x = np.array(path_x)
#     path_y = np.array(path_y)
#     # waypoints come out goal first
#     path_x = np.flip(path_x)
#     path_y = np.flip(path_y)
#
#     psize = len(path_x)
#     waypoints = np.array([[path_x[0], path_y[0]]])
#     for i in range(2, psize):
#         x = [path_x[i - 2], path_x[i - 1]]
#         y = [path_y[i - 2], path_y[i - 1]]
#         slope, intercept, r_value, p_value, std_err = stats.linregress(x, y)
#
#         # print("slope: "+str(slope)+ "\t Intercept:"+str(intercept))
#         testy = slope * path_x[i] + intercept
#         # print("testy :" + str(testy) + "y: " + str(path_y[i]))
#         if (np.isnan(slope) and path_x[i] == path_x[i - 1]):
#             # print("x still on line")
#             continue
#         elif (np.isnan(slope) and path_y[i] == path_y[i - 1]):
#             # print("y still on line")
#             continue
#         elif (testy == path_y[i]):
#             # print("same " + str(i))
#             continue
#         else:
#             # print("diff " + str(i))
#             waypoints = np.concatenate((waypoints, [[path_x[i - 1], path_y[i - 1]]]), axis=0)
#     return waypoints

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

waypoints = UAVHeading.convertPathToUniqueWaypoints(path_x, path_y)


# print("all points")
# for i in range(0,len(path_x)-1):
#     print(str(path_x[i]) + "\t "+ str(path_y[i]))
# print("select points")
# print(waypoints)
waypoints += offset
waypoints /= scalefactor

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

# print(path_x)
# print(path_y)
plt.plot(path_y, path_x, "xb", label='Path')
plt.plot(oy, ox, ".k", label='Search Area Obstacles')
plt.plot(start[1], start[0], "xg", label='UAV0 Position')
plt.plot(goal[1], goal[0], "xr", label='UAV0 Goal')
plt.plot([pt[1] for pt in waypoints], [pt[0] for pt in waypoints], "ok", label='WPs')
plt.grid(True)
plt.axis("equal")
plt.show()
plt.pause(0.0000001)


