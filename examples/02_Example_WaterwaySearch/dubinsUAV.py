from matplotlib import pyplot as plt
from shapely.geometry import Point, MultiPoint
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
        
        self.lastWP = False # Lets system knonw that last waypoint has been reached
        
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

    def setWaypoints(self, newwps, newradius):
        self.waypoints = newwps
        self.wpRadius = newradius

    def getWaypoints(self):
        return self.waypoints
    
    def getActiveWaypoint(self):
        return self.waypoints[self.currentWPIndex]
    

    def getDist2otherUAVs(self, uavh_others_all):
        uavNameList = []
        name = 'uav'
        keys = ['ID', 'dist', 'lastDist', 'tracking', 'avoiding', 'clearedUAV']
        # for i in range(0, len(uavh_others_all)):
        #     uavName = name + str(i)
        #     uavNameList.append(uavName)

        self.trackUAV={ i:{ key:[] for key in keys} for i in range(0, len(uavh_others_all))}
        
        for i in range(0, len(uavh_others_all)):
                dist = self.distance([self.x, self.y], uavh_others_all[i]['uavobj'].position)
                self.trackUAV[i]['ID'] = uavh_others_all[i]['ID']
                self.trackUAV[i]['lastDist'] = dist

    def getOtherUAVStates(self, uavh_others_all, uavID):

        for i in range(0,len(uavh_others_all)):
            if self.trackUAV[i]['ID'] == uavh_others_all[i]['ID']: # IDs in both Lists should line up
                dist = self.distance([self.x, self.y], uavh_others_all[i]['uavobj'].position)
                self.trackUAV[i]['dist'] = dist

            # is the distance to other UAV increaseing or decreasing?
            if dist < self.trackUAV[i]['lastDist']:
                self.trackUAV[i]['clearedUAV'] = False
                print('UAV ' + str(self.trackUAV[i]['ID']) + ' is approaching')
            else:
                print('UAV ' + str(self.trackUAV[i]['ID']) + ' is moving away')
                self.trackUAV[i]['clearedUAV'] = True

            # Is A* avoiding a UAV?
            if len(uavID) > 0: # IDs in both lists will not always line up
                for ID in uavID: # which UAV is being avoided
                    if self.trackUAV[i]['ID'] == ID:
                        self.trackUAV[i]['tracking'] = True
                        self.trackUAV[i]['avoding'] = True
                        print('Avoiding UAV ' + str(ID))
                    else:
                        self.trackUAV[i]['avoding'] = False
            else:
                self.trackUAV[i]['avoding'] = False

            # want to keep tracking until far enough away / moving away or clearedUAV = True
            if self.trackUAV[i]['tracking'] == True and self.trackUAV[i]['clearedUAV'] == False:
                print('Tracking UAV ' + str(self.trackUAV[i]['ID']))
            else:
                print('NOT Tracking UAV ' + str(self.trackUAV[i]['ID']))

            self.trackUAV[i]['lastDist'] = dist


        # # comparing two lists for UAV IDs
        # for uav in uavh_others_all:
        #     for otherUAV in self.OtherUAVlastdist:
        #             if otherUAV['ID'] == uav['ID']:
        #                 trackUAV['ID'] = uav['ID']
        #                 dist = self.distance([self.x, self.y], uav['uavobj'].position)
        #                 trackUAV['dist'] = dist
        #                 trackUAV['lastDist'] = otherUAV['dist']
        #                 if dist < trackUAV['lastDist']:
        #                     trackUAV['clearedUAV'] = False
        #                     state = 'Moving towards'
        #                 else:
        #                     state = 'Moving away'
        #                     trackUAV['clearedUAV'] = True

        # uavDistances = []
        # for uav in uavh_others_all:
        #     dist = self.distance([self.x, self.y], uav['uavobj'].position)
        #     self.trackOtherUAVS.append([uav['ID'], dist])
        #     uavDistances.append([uav['ID'], dist])
        #     trackUAV['ID'] = uav['ID']
        #     trackUAV['dist'] = dist

        #     if len(uavID) > 0:
        #         for ID in uavID:
        #             if uav['ID'] == ID:
        #                 trackUAV['tracking'] = True
        #                 print('Potential Collision with UAV ' + str(ID))

        #                 if trackUAV['clearedUAV'] == True:
        #                     trackUAV['tracking'] = False


        #     else:
        #         print('No pot. collision')
                
        #     if dist < self.OtherUAVlastdist:
        #         trackUAV['clearedUAV'] = False
        #         state = 'Moving towards'
        #     else:
        #         state = 'Moving away'
        #         trackUAV['clearedUAV'] = True

            
            
            
        #     uavList.append(trackUAV)

        # self.OtherUAVlastdist = uavDistances


        # return uavDistAndID

    def collisionUAVs(self, uavh_others_all, uavID):
        uavDistances = []
        if len(uavID) > 0:
            for ID in uavID:
                for uav in uavh_others_all:
                    if uav['ID'] == ID:
                        trackDist.append([ID, uav['uavobj']].poisition)

        else:
            print('No pot. collision')

        dist2UAVs = uav['dubins'].distance(CASuavPos[0], NCuavPos[0])
        if dist2UAVs < previousDist:
            state = 'Moving towards'
        else:
            state = 'Moving away'
            clearedOtherUAV = True


        return uavDistAndID

    def makePath(self, pathType, numbOfPoints, dist):
        '''                
        Parameters: pathType        specify path type: Line, Sine
                    numbOfPoints    number of points used to build the path
                    dist            distance between each point
        Description:
                    Generate waypoints for several paths types to follow.
                    Currnent options: Line, Sine
        '''

        points = []
        points.append([self.x, self.y])

        if pathType == 'Line':
            point_x = self.x
            point_y = self.y
            for i in range(0, numbOfPoints, 1):
                point_x = point_x + dist*np.cos(self.heading)
                point_y = point_y + dist*np.sin(self.heading)
                points.append([point_x, point_y])

        if pathType == 'Sine':
            point_x = self.x
            point_y = self.y
            theta = np.linspace(0, 4*np.pi, numbOfPoints)
            for i in range(0, numbOfPoints, 1):
                point_x = point_x + 0.05*np.sin(self.heading + theta[i])
                point_y = point_y - dist
                points.append([point_x, point_y])

        return points   

    def detectClosestWP(self, dist, theta_possible, alpha, targetPath, returnMethod):
        '''                
            Parameters: dist            distance each point is placed away from UAV
                        theta_possible  max angle the UAV can turn - semi circle is defined by this angle
                        alpha           mulitplier to increase the arc of the generated semi-circle
                        target path     waypoint list of original path to follow
                        returnMethod    how to determine which waypoint to on target path to reconnect to.
                                        current methods: useClosestPt, useSmallestAngle
            Description:
                        Detect the closest waypoint to the CAS UAV within a specified distance.
                        Used to reconnect to the original path generated by makePath()
                        after avoiding another UAV or obstacle 
        '''
            
        points = [[self.x, self.y]]

        # Detection area - semi-circle looking ahead of UAV
        for div in range(-2, -10, -1):
            pt_x = self.x + (dist * np.cos(self.heading + (theta_possible*alpha / div)))
            pt_y = self.y + (dist * np.sin(self.heading + (theta_possible*alpha / div)))
            points.append([pt_x, pt_y])

        # +-0
        pt_x = self.x + (dist * np.cos(self.heading))
        pt_y = self.y + (dist * np.sin(self.heading))
        points.append([pt_x, pt_y])

        for div in range(10, 1, -1):
            pt_x = self.x + (dist * np.cos(self.heading + (theta_possible*alpha / div)))
            pt_y = self.y + (dist * np.sin(self.heading + (theta_possible*alpha / div)))
            points.append([pt_x, pt_y])

        points.append(list([self.x, self.y]))

        # == Determine which waypoint to return to on the original path == #
        index = 0               # keeps track of which entries in targetPath are inside the detection area
        detectedPt  = []        # list of detected wp inside detection area
        detectedPtIndex = []    # Index of the detected waypoints in the targetPath 
        targetPt = []           # coordinates of the closest wp after completing the A* path
        targetIndex = []        # index of the closest wp - will be used to update currentWPIndex
        astarGoalPt = []
        astarGoalIndex = []
        # Check to see if targetPath is inside detection area
        detect_polygon = Polygon(points)
        targetPath_polygon = Polygon(targetPath)
        
        if detect_polygon.intersects(targetPath_polygon) or detect_polygon.touches(targetPath_polygon):
            # Find waypoints within detection area
            for pt in targetPath:
                checkPT = Point(pt)
                if detect_polygon.contains(checkPT):
                    # If point in targetPath is inside detect_polygon add it to a list
                    detectedPt.append(pt)
                    detectedPtIndex.append(index)
                index += 1  # keep track of the entry in target path list

            # If waypoint is detected determine which waypoint to reconnect to
            if len(detectedPt)>0:
                index = 0           # reset index
                furthestPt = 0      # temporary most distant point away from UAV - used to determing A* goal point

                # Find goal point for A*
                for pt in detectedPt:
                    # Find most distant waypoint between the UAV and detected wps from targetPath
                    dist2WP = self.distance(pt, (self.x, self.y))
                    temp = detectedPtIndex[index]
                    if dist2WP > furthestPt:
                        furthestPt = dist2WP
                        astarGoalPt = pt
                        astarGoalIndex = temp
                    index += 1           
                index = 0   # reset index

                # Decide which waypoint on the original path to recconect to
                # after clearing A* path or passing NC uav
                # Using specific cases becasue more might be added later
                if returnMethod == 'useClosestPt':
                    closestPt = 9999999 # temporary large distance
                    for pt in detectedPt:
                        # Find closes waypoint between the UAV and detected wps from targetPath
                        dist2WP = self.distance(pt, (self.x, self.y))
                        temp = detectedPtIndex[index]
                        if dist2WP < closestPt:
                            closestPt = dist2WP
                            targetPt = pt
                            targetIndex = temp
                        index += 1

                elif returnMethod == 'useSmallestAngle':
                    smallestAngle = 99999999 # temporary large angle
                    for pt in detectedPt:
                        # Find the waypoint with the smallest required heading correction
                        angle2pt = np.arctan2(self.y - pt[1], self.x - pt[0])
                        angle2pt += np.deg2rad(180)
                        theta = self.heading - angle2pt
                        temp = detectedPtIndex[index]
                        if abs(theta) < smallestAngle and abs(theta) < self.turnrate*0.5:
                            smallestAngle = angle2pt
                            targetPt = pt
                            targetIndex = temp
                        index += 1

                return points, targetPt, targetIndex, astarGoalIndex, astarGoalPt
                
        return points, targetPt, targetIndex, astarGoalIndex, astarGoalPt

    def CarrotChaseWP(self, delta ):
        ''' Determine carrot point coordinates between waypoints '''
        # delta = 0.01 # distance carrot is placed ahead of UAV
        wp_1 = self.waypoints[self.currentWPIndex-1]
        wp_2 = self.waypoints[self.currentWPIndex]

        dist2wp = self.distance(wp_2, (self.x, self.y))

        R_u = self.distance(wp_1, (self.x, self.y))
        dely = wp_2[1] - wp_1[1]
        delx = wp_2[0] - wp_1[0]
        theta = np.arctan2(dely, delx)

        dely = self.y - wp_1[1]
        delx = self.x - wp_1[0]
        theta_u = np.arctan2(dely, delx)


        beta = theta - theta_u

        # print('Carrot Point Angles: ' 
        #             + str(np.degrees(theta)) + 
        #         ' ' + str(np.degrees(theta_u)) + 
        #         ' ' + str(np.degrees(beta)) )


        R = np.sqrt(R_u**2 - ((R_u*np.sin(beta))**2))
        carrotTarget_x = wp_1[0] + (R+delta)*np.cos(theta)
        carrotTarget_y = wp_1[1] + (R+delta)*np.sin(theta)

        # # bound carrot point between wp1 and wp2 - not working....
        if carrotTarget_x > wp_2[0]:
            carrotTarget_x = wp_2[0]
        # elif carrotTarget_x < wp_1[0]:
        #     carrotTarget_x = wp_1[0]

        if carrotTarget_y > wp_2[1]:
            carrotTarget_y = wp_2[1]
        # elif carrotTarget_y < wp_1[1]:
        #     carrotTarget_x = wp_1[1]

        carrotPos = [carrotTarget_x, carrotTarget_y]
        return carrotPos

    def simulateWPDubins(self, UseCarrotChase, delta):
        wpRadius = self.wpRadius
        activeWP = self.getActiveWaypoint()
        # carrotPoint = self.CarrotChaseWP()

        if UseCarrotChase:
            carrotPoint = self.CarrotChaseWP(delta)
            CarrotDist = self.distance(activeWP, carrotPoint)
            # print('Carrot2WP: ' + str(CarrotDist) + '\t ' + str(CarrotDist < wpRadius) + '\t ' + str(CarrotDist > self.lastDist) + '\t# ' + str(self.currentWPIndex) + '\tLast: ' + str(self.lastDist))

            if (CarrotDist < wpRadius and CarrotDist > self.lastDist):
                if(self.currentWPIndex < len(self.waypoints)-1):
                    self.currentWPIndex += 1
                    #print("WP Increment")
                    #update distance...
                    CarrotDist = self.distance(self.getActiveWaypoint(), carrotPoint)
                else:
                    #print("Last Waypoint")
                    # Used to tell system to switch from A* path back to original path
                    self.lastWP = True 

            RHeading = np.arctan2(self.y - carrotPoint[1], self.x - carrotPoint[0])
            RHeading += np.deg2rad(180)
            if(RHeading >= np.pi*2):
                RHeading -= np.pi*2 
            self.update_pos(RHeading)
            self.lastDist = CarrotDist

        else:
            UAVdist = self.distance(activeWP, (self.x, self.y))

            if (UAVdist <= wpRadius and UAVdist >= self.lastDist):
                if not self.currentWPIndex == len(self.waypoints)-1:
                    self.currentWPIndex += 1
                    #print("WP Increment")
                    #update distance...
                    UAVdist = self.distance(self.getActiveWaypoint(), activeWP)
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

