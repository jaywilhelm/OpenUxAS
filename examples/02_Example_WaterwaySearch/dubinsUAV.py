from matplotlib import pyplot as plt
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
#from lineSegmentAoE import *
import numpy as np
import sys

class dubinsUAV():

    def __init__(self, position, velocity, heading, dt):

        self.velocity = velocity
        self.useDubins = True
        self.v = velocity
        self.dt = dt
        self.t = 0
        self.turnrate = 100 #0.35
        self.turn_radius = []

        #Current state
        self.position = [position[0], position[1]]  # added for CAS
        self.x = position[0]
        self.y = position[1]
        self.vx = []
        self.vy = []
        self.lastDist = 9999999
        self.cmdHeading = []
        self.flightEnvX = []
        self.flightEnvY = []
        
        self.heading = heading
        self.currentWPIndex = 1
        self.withinThreshold = False

        # History
        self.xs = np.array([])
        self.ys = np.array([])
        self.vxs = np.array([])
        self.vys = np.array([])
        self.headings = np.array([])
        self.headingcmds = np.array([])
        self.ts = np.array([])

        self.vx = velocity * np.cos(heading)
        self.vy = velocity * np.sin(heading)
        self.dt = dt
        self.turn_radius = self.v / self.turnrate

    def simulateWPDubins(self, wpList, wpRadius):
        # currentWPIndex = 0
        # withinThreshold = False
        # lastDist = sys.maxsize
        
        if len(wpList) > 1:
            self.currentWPIndex = self.currentWPIndex
        else:
            self.currentWPIndex = 0
        
        dist = self.distance(wpList[self.currentWPIndex], (self.x, self.y))

        print('D: ' + str(dist) + ' ' + str(dist < wpRadius) + ' ' + str(dist > self.lastDist) + ' ' + str(self.currentWPIndex) + ' ' + str(self.lastDist))
        if dist < wpRadius:
            self.withinThreshold = True

        if self.withinThreshold and (dist > self.lastDist) and (self.currentWPIndex < len(wpList) - 1):
            self.currentWPIndex += 1
            self.withinThreshold = False

        heading = np.arctan2(wpList[self.currentWPIndex][1] - self.y, wpList[self.currentWPIndex][0] - self.x) + np.deg2rad(180)
        print('Head2: ' + str(np.rad2deg(heading)))
        self.update_pos(heading)
        print('Head3: ' + str(np.rad2deg(heading)))

        self.lastDist = dist

    def distance(self, a, b):
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)


    def update_pos(self, heading):

            VF_heading = heading

            if self.useDubins:

                theta = np.arctan2(self.vy, self.vx)
                if abs(theta - VF_heading) < np.pi:
                    if theta - VF_heading < 0:
                        theta = theta + self.turnrate * self.dt
                    else:
                        theta = theta - self.turnrate * self.dt

                else:
                    if theta - VF_heading > 0:
                        theta = theta + self.turnrate * self.dt
                    else:
                        theta = theta - self.turnrate * self.dt

            else:
                theta = VF_heading
            print('theta ' + str(theta))
            # Update States
            self.t = self.t + self.dt
            self.heading = theta
            self.cmdHeading = VF_heading

            self.vx = self.v * np.cos(theta)
            self.vy = self.v * np.sin(theta)
            self.x = self.x + self.vx * self.dt
            self.y = self.y + self.vy * self.dt
            self.WaypointX = []
            self.WaypointY = []

            # Update History
            self.xs = np.append(self.xs,self.x)
            self.ys = np.append(self.ys,self.y)
            self.vxs = np.append(self.vxs, self.vx)
            self.vys = np.append(self.vys, self.vy)
            self.headings = np.append(self.headings,self.heading)
            self.ts = np.append(self.ts,self.t)

    def update_pos_simple(self):
        # Update States
        self.t = self.t + self.dt
        theta = self.heading
        self.vx = self.v * np.cos(theta)
        self.vy = self.v * np.sin(theta)
        self.x = self.x + self.vx * self.dt
        self.y = self.y + self.vy * self.dt
        self.position = [(self.x, self.y)] # added for CAS

        # Update History
        self.xs = np.append(self.xs, self.x)
        self.ys = np.append(self.ys, self.y)
        self.vxs = np.append(self.vxs, self.vx)
        self.vys = np.append(self.vys, self.vy)
        self.headings = np.append(self.headings, self.heading)
        self.ts = np.append(self.ts, self.t)

