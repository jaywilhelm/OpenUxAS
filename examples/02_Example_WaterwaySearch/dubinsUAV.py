from matplotlib import pyplot as plt
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon
#from lineSegmentAoE import *
import numpy as np
import sys

class dubinsUAV():

    def __init__(self, position, velocity, heading, dt=0.1):

        self.velocity = velocity
        self.turnRateLimited = True
        self.v = velocity
        self.dt = dt
        self.t = 0
        self.turnrate = np.deg2rad(20)
        #self.turn_radius = []


        #Current state
        self.x = position[0]
        self.y = position[1]
        self.vx = []
        self.vy = []
        self.lastDist = np.inf
        #self.cmdHeading = []
        #self.flightEnvX = []
        #self.flightEnvY = []
        
        self.heading = heading
        self.currentWPIndex = 0
        self.withinThreshold = False

        # History
        self.xs = np.array([])
        self.ys = np.array([])
        self.vxs = np.array([])
        self.vys = np.array([])
        self.headings = np.array([])
        #self.headingcmds = np.array([])
        self.ts = np.array([])

        self.vx = velocity * np.cos(heading)
        self.vy = velocity * np.sin(heading)
        self.dt = dt
        #self.turn_radius = self.v / self.turnrate
    def getPosition(self):
        return [self.position[0], self.position[1]]

    def setWaypoints(self, newwps, newradius=0.01):
        self.waypoints = newwps
        self.wpRadius = newradius

    def getWaypoints(self):
        return self.waypoints
    
    def getActiveWaypoint(self):
        return self.waypoints[self.currentWPIndex]

    def CarrotChaseWP(self):
        ''' Determine target carrot point coordinates between waypoints '''

        delta = 0.01
        wp_1 = self.waypoints[self.currentWPIndex-1]
        wp_2 = self.waypoints[self.currentWPIndex]

        dist2wp = self.distance(wp_2, (self.x, self.y))

        # if self.currentWPIndex >= (len(self.waypoints)-1):
        #     print('Last Waypoint')
        #     carrotTarget_x = wp_2[0]
        #     carrotTarget_y = wp_2[1]
        # else:
        R_u = self.distance(wp_1, (self.x, self.y))
        dely = wp_2[1] - wp_1[1]
        delx = wp_2[0] - wp_1[0]
        theta = np.arctan2(dely, delx)

        dely = self.y - wp_1[1]
        delx = self.x - wp_1[0]
        theta_u = np.arctan2(dely, delx)


        beta = theta - theta_u

        print('Carrot Point Angles: ' 
                    + str(np.degrees(theta)) + 
                ' ' + str(np.degrees(theta_u)) + 
                ' ' + str(np.degrees(beta)) )


        R = np.sqrt(R_u**2 - ((R_u*np.sin(beta))**2))
        carrotTarget_x = wp_1[0] + (R+delta)*np.cos(theta)
        carrotTarget_y = wp_1[1] + (R+delta)*np.sin(theta)

        carrotPos = [carrotTarget_x, carrotTarget_y]
        return carrotPos

    def simulateWPDubins(self):
        # currentWPIndex = 0
        # withinThreshold = False
        # lastDist = sys.maxsize
        UseCarrotChaseWP = True
        wpRadius = self.wpRadius
        activeWP = self.getActiveWaypoint()
        carrotPoint = self.CarrotChaseWP()
        dist = self.distance(activeWP, (self.x, self.y))

        print('D: ' + str(dist) + '\t ' + str(dist < wpRadius) + '\t ' + str(dist > self.lastDist) + '\t# ' + str(self.currentWPIndex) + '\tLast: ' + str(self.lastDist))

        if (dist < wpRadius and dist > self.lastDist):
            if(self.currentWPIndex < len(self.waypoints)-1):
                self.currentWPIndex += 1
                print("WP Increment")
                #update distance...
                dist = self.distance(self.getActiveWaypoint(), (self.x, self.y))
            else:
                print("end of list, do something")

        if UseCarrotChaseWP:
            RHeading = np.arctan2(self.y - carrotPoint[1], self.x - carrotPoint[0])
            RHeading += np.deg2rad(180)
            if(RHeading >= np.pi*2):
                RHeading -= np.pi*2 
            self.update_pos(RHeading)
        else:
            RHeading = np.arctan2(self.y - activeWP[1], self.x - activeWP[0])
            RHeading += np.deg2rad(180)
            if(RHeading >= np.pi*2):
                RHeading -= np.pi*2
            self.update_pos(RHeading)

        self.lastDist = dist

    def distance(self, a, b):
        return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)


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
        if(theta >= np.pi*2):
            theta -= np.pi*2
        print('Req: '+ str(np.rad2deg(RequestedHeading)) + '\ttheta ' + str(np.rad2deg(theta)))
        # Update States
        self.t = self.t + self.dt
        self.heading = theta
        #self.cmdHeading = VF_heading
        self.update_pos_simple()

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

