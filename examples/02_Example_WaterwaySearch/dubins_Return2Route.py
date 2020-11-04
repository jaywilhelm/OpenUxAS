# Jeremy Browne
# October 2020
#
# This dubins class is meant to accompany the 
# Return to rout python script and contain the functions 
# needed for a dubins type vehicle to:
# - follow a path
# - 


from matplotlib import pyplot as plt
from shapely.geometry import Point, MultiPoint
from shapely.geometry.polygon import Polygon
import numpy as np
import sys

class dubinsUAV():
    def __init__(self, position, velocity, heading, dt=0.1):

        self.velocity = velocity
        self.turnRateLimited = True
        self.velocity = velocity
        self.dt = dt
        self.t = 0
        self.turnrate = np.deg2rad(10)
        # from Ch 5 in the Pilot's Handbook of Aernautical Knowledge 
        self.turnRadius = ((self.velocity * 360/(np.rad2deg(self.turnrate)))/np.pi)/2

        #Current state
        self.x = position[0]
        self.y = position[1]
        self.vx = []
        self.vy = []
        self.lastDist = np.inf
        
        self.heading = heading
        self.currentWPIndex = 0
        self.lapCounter =0 # how many track laps have been completed
        self.withinThreshold = False
        
        self.lastWP = False # Lets system knonw that last waypoint has been reached
        
        # History
        self.xs = np.array([])
        self.ys = np.array([])
        self.vxs = np.array([])
        self.vys = np.array([])
        self.headings = np.array([])
        self.ts = np.array([])

        self.vx = self.velocity * np.cos(heading)
        self.vy = self.velocity * np.sin(heading)
        self.dt = dt

    def getPosition(self):
        return [self.position[0], self.position[1]]

    def setWaypoints(self, newwps, newradius):
        self.waypoints = newwps
        self.wpRadius = newradius

    def getWaypoints(self):
        return self.waypoints
    
    def getActiveWaypoint(self):
        return self.waypoints[self.currentWPIndex]

    def distance(self, a, b):
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def simulateWPDubins(self):
        wpRadius = self.wpRadius
        activeWP = self.getActiveWaypoint()
        UAVdist = self.distance(activeWP, (self.x, self.y))

        if (UAVdist <= wpRadius and UAVdist >= self.lastDist):
            if not self.currentWPIndex > len(self.waypoints):
                self.currentWPIndex += 1
                if self.currentWPIndex >= len(self.waypoints): # note for RaceTrack - 1st and last wpt are identical - mainly for plotting reasons
                    self.currentWPIndex = 0
                    self.lapCounter +=1 
                # print("WP Increment")
                # update distance...
                # UAVdist = self.distance(self.getActiveWaypoint(), activeWP)
            else:
                #print("Last Waypoint")
                # Used to tell system to switch from A* path back to original path
                self.lastWP = True

        RHeading = np.arctan2(self.y - activeWP[1], self.x - activeWP[0])
        RHeading += np.deg2rad(180)
        if(RHeading >= np.pi*2):
            RHeading -= np.pi*2 
        self.update_pos(RHeading)
        self.lastDist = UAVdist    

    def update_pos(self, RequestedHeading):

        if self.turnRateLimited:
            theta = self.heading 
            if(np.abs(RequestedHeading - theta) < self.turnrate * self.dt):
                turnrate = np.abs(RequestedHeading - theta) / self.dt
            else:
                turnrate = self.turnrate

            if abs(theta - RequestedHeading) < np.pi:
                if theta - RequestedHeading < 0:
                    theta = theta + turnrate * self.dt
                else:
                    theta = theta - turnrate * self.dt

            else:
                if theta - RequestedHeading > 0:
                    theta = theta + turnrate * self.dt
                else:
                    theta = theta - turnrate * self.dt
            # if(np.abs(RequestedHeading - theta) > self.turnrate * self.dt):
            #     if theta - RequestedHeading < 0:
            #          theta = theta + self.turnrate * self.dt
            #     else:
            #          theta = theta - self.turnrate * self.dt
            # else:
            #     theta = RequestedHeading
        else:
            theta = RequestedHeading

        # Ensures positve theta values between 0-360
        if(theta >= np.pi*2):
            theta -= np.pi*2
        if(theta < 0):
            theta += np.pi*2
        #print('Req: '+ str(np.rad2deg(RequestedHeading)) + '\ttheta ' + str(np.rad2deg(theta)))
        # Update States
        self.t = self.t + self.dt
        self.heading = theta
        #self.cmdHeading = VF_heading
        self.update_pos_simple()

    def update_pos_simple(self):
        # Update States
        self.t = self.t + self.dt
        theta = self.heading
        self.vx = self.velocity * np.cos(theta)
        self.vy = self.velocity * np.sin(theta)
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