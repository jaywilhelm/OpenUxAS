'''
Jeremy Browne
November 2020
Test script for converting longitude and latitude data to meters
'''

import pyproj
import numpy as np
from matplotlib import pyplot as plt

lon1 = 39.3264051914717
lat1 = -82.1101289994580

lon2 = 39.3245914749791
lat2 = -82.1099233430367

wgs84 = pyproj.Proj(init="epsg:4326")

epsg3035 = pyproj.Proj(init="epsg:3035")

x1, y1 = pyproj.transform(wgs84, epsg3035, lon1, lat1)
# print(x1, y1)

x2, y2 = pyproj.transform(wgs84, epsg3035, lon2, lat2)
# print(x2, y2)

# a Pythagore's theorem is sufficient to compute an approximate distance
distance_m = np.sqrt((x2-x1)**2 + (y2-y1)**2)
# print(distance_m)

# ===========================================================================# 

PruitTrack =[
[39.3264051914717, -82.1101289994580], [39.3263499334731, -82.1103352244684], [39.3261989661035, -82.1104861915330],
[39.3259927415369, -82.1105414491151], [39.3249489949075, -82.1105414491151], [39.3247433390045, -82.1104865192473],
[39.3245921315912, -82.1103357926091], [39.3245365454954, -82.1101296557923], [39.3245914749791, -82.1099233430367],
[39.3247422017190, -82.1097721357249], [39.3249489949075, -82.1097165492456], [39.3259927415369, -82.1097165492456],
[39.3261989665845, -82.1097718071053], [39.3263499337508, -82.1099227743733], [39.3264051914717, -82.1101289994580]]

XYpruitTrack = []
for pt in PruitTrack:
    lon1 = pt[0]
    lat1 = pt[1]
    x1, y1 = pyproj.transform(wgs84, epsg3035, lat1, lon1)


    XYpruitTrack.append([x1,y1])

# print(XYpruitTrack)

# fig, ax = plt.subplots()
# plt.plot([pt[0] for pt in XYpruitTrack], [pt[1] for pt in XYpruitTrack], c='b', marker='.', markersize=8)
# ax.axis('equal')
# plt.grid(True)

# plt.show()

# ===========================================================================# 
import utm


XYpruitTrack = []
for pt in PruitTrack:
    lon1 = pt[0]
    lat1 = pt[1]
    utmPts = utm.from_latlon(lon1, np.abs(lat1), 32, 'S')


    XYpruitTrack.append([utmPts[0], utmPts[1]])

# print(XYpruitTrack)

# fig, ax = plt.subplots()
# plt.plot([pt[0] for pt in XYpruitTrack], [pt[1] for pt in XYpruitTrack], c='b', marker='.', markersize=8)
# ax.axis('equal')
# plt.grid(True)

# plt.show()

#==================================================
# this seems to work the best !!!!!
from pyproj import Proj
p = Proj(proj='utm',zone=17, ellps='WGS84', preserve_units=False)

XYpruitTrack = []
for pt in PruitTrack:
    lon = pt[0]
    lat = pt[1]
    x,y = p(lat, lon)

    XYpruitTrack.append([x, y])

print('Pruit Track Meters = ' + '\n' + str(XYpruitTrack))

fig, ax = plt.subplots()
plt.plot([pt[0] for pt in XYpruitTrack], [pt[1] for pt in XYpruitTrack], c='b', marker='.', markersize=8)
ax.axis('equal')
plt.grid(True)
plt.show()

XYinverse = []
for pt in XYpruitTrack:
    x = pt[0]
    y = pt[1]
    lon, lat = p(x,y,inverse=True)

    XYinverse.append([lon, lat])

print('Pruit Track Long/Lat = ' + '\n' + str(XYinverse))

fig, ax = plt.subplots()
plt.plot([pt[0] for pt in XYinverse], [pt[1] for pt in XYinverse], c='b', marker='.', markersize=8)
ax.axis('equal')
plt.grid(True)
plt.show()

