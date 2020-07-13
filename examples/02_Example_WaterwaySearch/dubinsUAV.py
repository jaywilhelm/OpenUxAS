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
        self.heading = heading
        self.cmdHeading = []
        self.flightEnvX = []
        self.flightEnvY = []

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


    def update_pos(self):
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

