
# Jeremy Browne Summer 2020
# Test script to evaluate recovery path after successful UAV collision avoidance:
# 1) Follow refference path - search area pattern
# 2) Insert fake A* waypoints to simulate an avoidance maneuver
# 3) After clearing the fake NC UAV, use clothoids to determine closest path back to the 
#    reference path (search area) without violating the UAVs turn radius
#
# This particular script is meant to look at a more realistic scenario with appropriatly distanced waypoints

# For Dubins Vehicle
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
import sys, math, os, subprocess, time
import numpy as np
from dubinsUAV import dubinsUAV
from TerminalColors import TerminalColors as TC

# For clothoid path generation
import pyclothoids
from pyclothoids import Clothoid
from scipy import linalg

# For A* Collision Avoidance
from lineSegmentAoE import *
sys.path.append('../UAVHeading-CollisionAvoidance/src')
from UAVHeading import UAVHeading

# For converting WGS84 degrees to meters
import pyproj 


def distance(a, b):
    return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

def fit_circle_2d(x, y, w=[]):
        x = np.array(x)
        y = np.array(y)
        A = np.array([x, y, np.ones(len(x))]).T
        b = x ** 2 + y ** 2
        # Modify A,b for weighted least squares
        if len(w) == len(x):
                W = diag(w)
                A = dot(W, A)
                b = dot(W, b)

        # Solve by method of least squares
        c = linalg.lstsq(A, b)[0]
        # Get circle parameters from solution c
        xc = c[0] / 2
        yc = c[1] / 2
        r = np.sqrt(c[2] + xc ** 2 + yc ** 2)
        return xc, yc, r

def finduavbyID(uavlist,uavID):
    for uav in uavlist:
        if(uav['ID'] == uavID):
            return uav

    return None
def findotheruavs(uavlist, uavNOT):
    otherlist = []
    otherjustobj = []
    for uav in uavlist:
        if(uav['ID'] != uavNOT):
            otherlist.append(uav)
            otherjustobj.append(uav['uavobj'])
    return otherlist, otherjustobj
    

def syncAVSfromDubins(uav):
    lat = uav['dubins'].x
    lon = uav['dubins'].y
    vel = uav['dubins'].v
    heading = uav['dubins'].heading
    IsAvoidanceUAV = uav['IsAvoidanceUAV']
    uav['uavobj'] = UAVHeading(pos=[lat, lon],
                               waypt=[], speed=vel, heading=heading,
                               tPossible=math.radians(45), IsAvoidanceUAV=IsAvoidanceUAV)
    return uav

RaceTrack = [[39.9680674, -82.8318678],[39.9659200, -82.8531621],[39.9576365, -82.8601268],
             [39.9439513, -82.8569246],[39.9407597, -82.8349099],[39.9399004, -82.8072913],[39.9453629, -82.7862372],
             [39.9576365, -82.7748696],[39.9683128, -82.7939224],[39.9681901, -82.8157770],[39.9680674, -82.8318678]]

#keept Track of what waypoints belongs to the reference path
Path = {}
for pt in RaceTrack:
    refPath['refPt'] = True
    refPath['pt'] = pt
# RaceTrack_meters = []
# for pt in RaceTrack:
#     x, y = pyproj.transform(wgs84, epsg3035, pt[1], pt[0])
#     RaceTrack_meters.append([x, y])

fig, ax = plt.subplots()
savePlots = []  # stores figure frames used to make a movie
# Calcuate heading angle between each waypoint -> used for clothoid path generation
pathHeadings = []
i = 0
for i in range(0, len(RaceTrack)-1):
    wpt0 = RaceTrack[i]
    wpt1 = RaceTrack[i+1]
    theta = np.arctan2((wpt1[0] - wpt0[0]), (wpt1[1] - wpt0[1])) 

    # Make sure that theta is between 0 and 360
    if(theta >= np.pi*2):
        theta -= np.pi*2
    if(theta < 0):
        theta += np.pi*2

    pathHeadings.append(theta)

# Create Dubins Vehicle
dt = 0.25 # 0.1
v = 0.00025
wptRad = 0.0005 # distance threshold to satisfy each waypoint
#
uavlist = []    # holds dictionary items for specific UAVs
uav1 = {}
uavlist.append(uav1)
uav2 = {}
uavlist.append(uav2)
#
thetaRef = np.deg2rad(270)
uavlist[0]['dubins'] = dubinsUAV(position=[39.9680674, -82.8308678], velocity=v,          
                                    heading=thetaRef, dt=dt)
deadpoint = [39.40, -82.1380578]
uavlist[0]['ID'] = 1
uavlist[0]['IsAvoidanceUAV'] = True
uavlist[0]['dubins'].setWaypoints(newwps=RaceTrack, newradius = wptRad )
uavlist[0]['dubins'].currentWPIndex = 0

#
thetaRef = np.deg2rad(0)
uavlist[1]['dubins'] = dubinsUAV(position=[39.9600674, -82.8418678], velocity=v,         
                                    heading=thetaRef, dt=dt)
uavlist[1]['ID'] = 4
uavlist[1]['IsAvoidanceUAV'] = False
#
uavlist[0] = syncAVSfromDubins(uavlist[0])
uavlist[1] = syncAVSfromDubins(uavlist[1])

# from Ch 5 in the Pilot's Handbook of Aernautical Knowledge 
# Using 50% of maximum turn radius - conservative value - given by np.degrees(uavlist[0]['dubins'].turnrate)/2
uavTurnRadius = ((uavlist[0]['dubins'].v * 360/(np.degrees(uavlist[0]['dubins'].turnrate)))/np.pi)/2
halfTurnRadius = uavTurnRadius*2
#
print('UAV turn radius with full turn rate: ' + str(uavTurnRadius) + ' and half turn rate: ' + str(halfTurnRadius))
Recovery_dict = {'X': [], 'Y' : [], 'Index1' : [], 'Index2' : [], 'wpt1' : [], 'wpt2' : [],
                 'pathLength' : [], 'turnRadii': []   }
#
hasPath = False
hadPlan = False
onlyOnce = False
wpList2 =None
usetargetPath = False
TargetWPList = None
hasPlan = False

area_length = 0.005
numbOfAstarPts = 0
NewPath = []
indexTracker = 0 # TO DO - find another way to change the index to the appropriate waypoint
numbOfRecoveryPts = 0

step = 0
while step < 600: # uavlist[0]['dubins'].currentWPIndex < len(uavlist[0]['dubins'].waypoints)-1: 
    ''' Identify UAVs using collision avoidence '''
    mainUAV = finduavbyID(uavlist, 1) # IDtoWatch
    uavh_others_all, uavh_others = findotheruavs(uavlist, 1) # ID not to watch for

    activeWP = uavlist[0]['dubins'].waypoints[uavlist[0]['dubins'].currentWPIndex]
        
    '''Generate the main path. Goal is to reconncet to this path after avoiding an intruder UAV/obstacle'''
    if TargetWPList == None:
        mainUAV['dubins'].getDist2otherUAVs(uavh_others_all)
        usetargetPath = True
        TargetWPList = RaceTrack
        uavlist[0]['dubins'].setWaypoints(TargetWPList, newradius=wptRad)
        uavlist[0]['dubins'].currentWPIndex = 0
        # print('TargetWPList: ' + str(TargetWPList))

    ''' Locate closest and furthest waypoints laying on reference path '''
    detectRange, targetWP, targetIndex, astarGoalIndex, astarGoalPt = mainUAV['dubins'].detectClosestWP(dist=0.015, theta_possible=mainUAV['uavobj'].thetaPossible, alpha=1, targetPath=TargetWPList, returnMethod='useSmallestAngle')
    #print('targetWP: ' + str(targetWP) + ' astarGoal: ' + str(astarGoalPt) + ' not expanding detection cone')
    if len(targetWP)==0 or len(astarGoalPt)==0:
        #print('targetWP: ' + str(targetWP) + ' astarGoal: ' + str(astarGoalPt) + ' expanding detection cone')
        detectRange, targetWP, targetIndex, astarGoalIndex, astarGoalPt = mainUAV['dubins'].detectClosestWP(dist=0.02, theta_possible=mainUAV['uavobj'].thetaPossible, alpha=1, targetPath=TargetWPList, returnMethod='useSmallestAngle')

    plotDetectRange, = plt.plot([pt[1] for pt in detectRange], [pt[0] for pt in detectRange], c='y')
            
    if len(targetWP) > 0:
        plotTargetWP, = plt.plot(targetWP[1], targetWP[0], c='r', marker = '*')
    else:
        plotTargetWP = None
    if len(astarGoalPt) > 0:
        plotAstarGoalPt, = plt.plot(astarGoalPt[1], astarGoalPt[0], c='g', marker = 'X', markersize=7)
    else:
        plotAstarGoalPt = None

    ''' Use A* to generate a replan path to avoid a potential collision with another UAV'''
    if len(uavlist) > 1:
        if not hasPlan:
            replan, wplist, avoid, full_path, uavID = mainUAV['uavobj'].avoid(uavh_others, area_length=area_length, static_koz=[], TargetPathWP=astarGoalPt, useAstarGoal=True)
            if len(uavID) > 0:
                for i in range(0, len(uavID)):    
                    uavID[i] = uavh_others_all[uavID[i]]['ID']
                    print('Potential Collision with UAV ' + str(uavID[i]))

        if(replan and not hasPlan):
            hasPlan = True
            hadPlan = True
            indexRecall = mainUAV['dubins'].currentWPIndex 
            wplist[0][0] = mainUAV['uavobj'].position[0]
            wplist[0][1] = mainUAV['uavobj'].position[1]
            numbOfAstarPts= len(wplist)
            # Insert A* points (wplist) into RaceTrack path
            RaceTrack[mainUAV['dubins'].currentWPIndex:mainUAV['dubins'].currentWPIndex] = wplist.tolist()
            print('Insert ' + str(numbOfAstarPts) + ' Fake Astar Points at wpt index ' + str(indexRecall))

            closingDist = mainUAV['dubins'].distance(mainUAV['uavobj'].position, uavh_others[0].position)
            # print('Distance to other UAV: ' + str(closingDist))
            # print('POS: ' + str(mainUAV['uavobj'].position))
            # print('WP: ' + str(wplist))
            # print('DP: ' + str(deadpoint))  
            # useWPfollower = True
            # usetargetPath = False
            # clearedOtherUAV = False
        else:
            print("Not re-planning" )
    else:
        print('\tOnly one UAV')


    checkDistance = 9999999     # used to evaluate shorted clothoid path using interpolated target waypoints
    if hadPlan and mainUAV['dubins'].trackUAV[0]['clearedUAV']:
        hadPlan = False
        # convert uav North East Down angle convention to cartesion for clothoid heading: uavHeading = manUAV['dubins'].heading + np.radians(90)
        # Needed an axis flip to find the angle between  UAV and active waypoint: the x's in the numerator and y's in the denom, then add 180 deg

        # Use dummy waypoint in frong of UAC to determine heading for clotoid calcs
        xy = (mainUAV['dubins'].x, mainUAV['dubins'].y)
        r = 0.3
        px = xy[0] + r * np.cos(mainUAV['dubins'].heading)
        py = xy[1] + r * np.sin(mainUAV['dubins'].heading)

        uavHeading = np.arctan2(mainUAV['dubins'].x - px, mainUAV['dubins'].y - py) + np.radians(180)

        # Keep uavHeading within [0, 360] degrees
        if(uavHeading >= np.pi*2):
            uavHeading -= np.pi*2
        if(uavHeading < 0):
            uavHeading += np.pi*2

        numbOfwpts = 10     # how many waypoints per clothoid will be use for path follwing (3 clothoids needed for a solution)
        clothoid_paths = [] # List of information on each clothoid path generated between UAV and waypoint on reference path
    
        '''
        Find shortest Clothoid path that does not violate UAV turning radius
        Choose the closest point on the ref path as injection point
        Chosen point is interpolated between original waypoint index sets
        Chosen point must not violate turn radius
        '''

        for index in range(indexRecall+numbOfAstarPts, 7+numbOfAstarPts ): # TO DO change 7 to some wp index within a detected range of the UAV
            numbOfPts = 5                       # number of interpolate points betwee index and index+1
            x1 = mainUAV['dubins'].waypoints[index][0]
            x2 = mainUAV['dubins'].waypoints[index+1][0]
            linX = np.linspace(x1, x2, numbOfPts,endpoint=False )

            y1 = mainUAV['dubins'].waypoints[index][1]
            y2 = mainUAV['dubins'].waypoints[index+1][1]
            linY = np.linspace(y1, y2, numbOfPts, endpoint=False )

            print('Interpolating ' + str(numbOfPts) + ' points between index ' + str(index) + ' and ' + str(index+1))
            Recovery_dict['X'].append(linX)
            Recovery_dict['Y'].append(linY)
            Recovery_dict['Index1'].append(index)
            Recovery_dict['Index2'].append(index+1)
            Recovery_dict['wpt1'].append([x1, y1])
            Recovery_dict['wpt2'].append([x2, y2])

            for pt in range(0, numbOfPts):
                print('\tChecking pt ' + str(pt+1))

                # targetWPT = [Path[index][0], Path[index][1], Headings[index]]
                targetHeading = np.arctan2(linX[pt] - x2, linY[pt] - y2) + np.radians(180) # heading between interpolated point and wp2
                targetWPT = [linX[pt], linY[pt], targetHeading]  # heading should still be the same?
                plt.plot(targetWPT[1], targetWPT[0], 'o', markersize=5)

                if(targetWPT[2] >= np.pi*2):
                    targetWPT[2] -= np.pi*2
                if(targetWPT[2] < 0):
                    targetWPT[2] += np.pi*2

                # Provides clothoid parameters used to generate a path between the UAV and target waypoint on reference path: 3 clothoids per path are needed
                clothoid_list = pyclothoids.SolveG2(mainUAV['dubins'].y, mainUAV['dubins'].x, uavHeading, 0, targetWPT[1], targetWPT[0], targetWPT[2], 0) # stiches a path of multiple clothoids
                
                circle_list = []    # Stores circle fit info for each clothoid segment in a single path: [cx,cy,r]
                ClothoidPath = []         # Stores a specified number of waypoints for each clothoid segment
                wpList2 = []        # Stored the 1st, middle, and last wpt from the 1st, 2nd, and 3rd clothoid respectively
                jj = 0              # Keeps track of which clothoid segment is being evaluated
                clothoidLength = 0  # keeps track of clothoid path length
                temp = []           # temporary variable - used for ploting purposes
                completeClothoidPts = []    # stores a the full path of each clothoid generated - mainly used for plotting purposes

                for i in clothoid_list:
                    plt.plot(*i.SampleXY(500))
                    pltpts = i.SampleXY(500)
                    for ii in range(0, len(pltpts[0])):
                        temp.append([pltpts[1][ii], pltpts[0][ii]])

                    points = i.SampleXY(numbOfwpts)         # used for circle fitting to calcuate a turn radius for each clothoid path and used for waypoint path following
                    ClothoidPath.append(points)

                    x0, y0, t0, k0, dk, s = i.Parameters    # start x, start y, initial curvature, change in curvature, clothoid segment length
                    clothoidLength += s                     # Sum up the length of each clothoid segment to find the total distance of the clothoid
                    
                    # print('Index: ' + str(index)  + ' Clothoid Path Distance: ' + str(clothoidLength) + ' and s: ' + str(s))  

                    # Select the first point of the 1st clothoid
                    # middle point of the 2nd clothoid and
                    # the last point of the 3rd clothoid segment to follow
                    midPt = int(len(points[0])/2)
                    endPt = len(points[0])-1
                    if jj == 0:
                        wpList2.append([points[0][1],points[1][1]])
                        mid_x = i.X(s/2)
                        mid_y = i.Y(s/2)
                        mid_x2 = i.X(0.75*s)
                        mid_y2 = i.Y(0.75*s)
                        wpList2.append([mid_x,mid_y])
                        wpList2.append([mid_x2,mid_y2])

                    elif jj == 1:
                        wpList2.append([points[0][1],points[1][1]])
                        mid_x = i.X(s/2)
                        mid_y = i.Y(s/2)
                        mid_x2 = i.X(0.75*s)
                        mid_y2 = i.Y(0.75*s)
                        wpList2.append([mid_x,mid_y])
                        wpList2.append([mid_x2,mid_y2])

                    elif jj == 2:
                        wpList2.append([points[0][1],points[1][1]])
                        mid_x = i.X(s/2)
                        mid_y = i.Y(s/2)
                        wpList2.append([mid_x,mid_y])
                        wpList2.append([points[0][endPt],points[1][endPt]])


                    '''
                    Fit circle to clothoid segment to 
                    calculate Turn Radius of clothoid segment
                    '''
                    # Full clothoid segment pt set
                    x_points = points[0]
                    y_points = points[1]

                    # Middle clothoid has two curvatures --> break into two halves 
                    # and fit a circle to each half. 
                    if jj == 1:
                        #First Half
                        halfX_points1 = x_points[:(int(len(x_points)/2))]   
                        halfY_points1 = y_points[:(int(len(y_points)/2))] 
                        half_xc1,half_yc1,half_r1 = fit_circle_2d(halfX_points1, halfY_points1)

                        #Second Half
                        halfX_points2 = x_points[(int(len(x_points)/2)):]   
                        halfY_points2 = y_points[(int(len(y_points)/2)):] 
                        half_xc2,half_yc2,half_r2 = fit_circle_2d(halfX_points2, halfY_points2)
                        
                        circle_list.append([half_xc1, half_yc1, half_r1])
                        circle_list.append([half_xc2, half_yc2, half_r2])

                    else:
                        xc,yc,r = fit_circle_2d(x_points, y_points)
                        circle_list.append([xc,yc,r])

                    jj+=1

                # plt.show(100)
                completeClothoidPts.append(temp)                                                                    # store points of entire clothoid path
                clothoid_paths.append([index, clothoidLength, ClothoidPath, wpList2, completeClothoidPts, circle_list])   # store info for each clothoid
                Recovery_dict['pathLength'].append(clothoidLength)
                Recovery_dict['turnRadii'].append(circle_list)

                'Determine if path violates Turn Radius'
                print('\tPath Distance: ' + str(round(clothoidLength,3)) + '\tClothoid Radii: ' + str(round(circle_list[0][2],3)) + 
                      '   ' + str(round(circle_list[1][2],3)) + '   ' + str(round(circle_list[2][2],3)) + '   ' + str(round(circle_list[3][2],3)))
            
                checkPassed = False
                for r in range(0, len(circle_list)):                        
                    if circle_list[r][2] < uavTurnRadius:
                        print('\t\t\tTurn radius too small')
                        checkPassed = False
                        break
                    else:
                        checkPassed = True

                if checkPassed == True and clothoidLength < checkDistance:
                    selectPath = index+1
                    checkDistance = clothoidLength
                    hasPath = True
                    Recovery_dict['chosenPathFull'] = completeClothoidPts
                    Recovery_dict['chosenPath'] = wpList2
                    Recovery_dict['cirlces'] = circle_list
                    numbOfRecoveryPts = len(wpList2)
                    Recovery_dict['chosenIndex'] = selectPath + numbOfRecoveryPts
                    print('\t\t\tSelected index pt: ' + str(selectPath) + ' which is now at index ' + str(Recovery_dict['chosenIndex']))
                elif checkPassed == True and clothoidLength > checkDistance:
                    print('\t\t\tPath Too Long')
                    

            for clothoid in clothoid_paths:
                plt.plot([pt[1] for pt in clothoid[4][0]], [pt[0] for pt in clothoid[4][0]])

                # circle1 = plt.Circle((clothoid[5][0][0],clothoid[5][0][1]),clothoid[5][0][2],color='magenta',alpha=0.2)
                # plt.gca().add_artist(circle1)
                # plt.scatter(clothoid[5][0][0],clothoid[5][0][0])

                # circle2a = plt.Circle((clothoid[5][1][0],clothoid[5][1][1]),clothoid[5][1][2],color='yellow',alpha=0.4)
                # plt.gca().add_artist(circle2a)
                # plt.scatter(clothoid[5][1][0],clothoid[5][1][1])
                # circle2b = plt.Circle((clothoid[5][2][0],clothoid[5][2][1]),clothoid[5][2][2],color='yellow',alpha=0.4)
                # plt.gca().add_artist(circle2b)
                # plt.scatter(clothoid[5][2][0],clothoid[5][2][1])

                # circle3 = plt.Circle((clothoid[5][3][0],clothoid[5][3][1]),clothoid[5][3][2],color='magenta',alpha=0.2)
                # plt.gca().add_artist(circle3)
                # plt.scatter(clothoid[5][3][0],clothoid[5][3][1])

        plt.plot(mainUAV['dubins'].ys, mainUAV['dubins'].xs, c='r', marker='o' )
        plt.plot([pt[1] for pt in RaceTrack], [pt[0] for pt in RaceTrack], c='b', marker='.')
        plt.plot( activeWP[1], activeWP[0], c='k', marker='X', markersize = 5 )
        plt.axis('equal')
        plt.grid(True)
        # plt.ylim(45.3, 45.45) #(45.0, 45.5)
        # plt.xlim(-121.0, -120.5)   
        plt.show(100)

        if hasPath == False:
            print('No valid path available')
        else:
            print('\nSelected point between wp index ' + str(selectPath-1) + ' and ' + str(selectPath) + ' as the recovery insertion point')

            plt.plot([pt[1] for pt in Recovery_dict['chosenPathFull'][0]], [pt[0] for pt in Recovery_dict['chosenPathFull'][0]])

            circle1 = plt.Circle((Recovery_dict['cirlces'][0][0], Recovery_dict['cirlces'][0][1]), Recovery_dict['cirlces'][0][2],color='magenta',alpha=0.2)
            plt.gca().add_artist(circle1)
            plt.scatter(Recovery_dict['cirlces'][0][0],Recovery_dict['cirlces'][0][1])

            circle2 = plt.Circle((Recovery_dict['cirlces'][1][0], Recovery_dict['cirlces'][1][1]), Recovery_dict['cirlces'][1][2],color='yellow',alpha=0.4)
            plt.gca().add_artist(circle2)
            plt.scatter(Recovery_dict['cirlces'][1][0],Recovery_dict['cirlces'][1][1])


            circle3 = plt.Circle((Recovery_dict['cirlces'][2][0], Recovery_dict['cirlces'][2][1]), Recovery_dict['cirlces'][2][2],color='yellow',alpha=0.4)
            plt.gca().add_artist(circle3)
            plt.scatter(Recovery_dict['cirlces'][2][0],Recovery_dict['cirlces'][2][1])

            circle4 = plt.Circle((Recovery_dict['cirlces'][3][0], Recovery_dict['cirlces'][3][1]), Recovery_dict['cirlces'][3][2],color='magenta',alpha=0.2)
            plt.gca().add_artist(circle4)
            plt.scatter(Recovery_dict['cirlces'][3][0],Recovery_dict['cirlces'][3][1])

            for ptList in Recovery_dict['chosenPath']:
                NewPath.append([ptList[1], ptList[0]])

            plt.plot([pt[1] for pt in NewPath], [pt[0] for pt in NewPath], c='green', marker='o',markersize=5)

            plt.plot(mainUAV['dubins'].ys, mainUAV['dubins'].xs, c='r', marker='o' )
            plt.plot([pt[1] for pt in RaceTrack], [pt[0] for pt in RaceTrack], c='b', marker='.')
            plt.plot( activeWP[1], activeWP[0], c='k', marker='X', markersize = 5 )
            plt.axis('equal')
            plt.grid(True)
            plt.show(100)


    if wpList2 !=None and hasPath == True and onlyOnce == False:
        onlyOnce = True 
        indexTracker = 0
        lastIndex = mainUAV['dubins'].currentWPIndex
        numbOfRecoveryPts = len(NewPath)
        insertIndex = indexRecall + numbOfAstarPts 
        RaceTrack[insertIndex:insertIndex] = NewPath
        mainUAV['dubins'].currentWPIndex = insertIndex 
        print('Insert ' + str(numbOfRecoveryPts) + ' Recovery Points at wpt index ' + str(insertIndex) )

    if indexTracker > (numbOfRecoveryPts) and hasPath == True: # TO DO - update wp in dubinsUAV class to appropriate wp after completing clothoid
        mainUAV['dubins'].currentWPIndex = Recovery_dict['chosenIndex'] # note - briefly targets wrong wp b/c there is a wp update, then this line is executed. 
        hasPath = False
        onlyOnce = False
        indexTracker = 0
        #plt.pause(30)   

    if  onlyOnce == True and mainUAV['dubins'].currentWPIndex > lastIndex:
        indexTracker+=1
        lastIndex = mainUAV['dubins'].currentWPIndex

    '''Update vehicle positions'''
    for uav in uavlist:     
        pts = uav['uavobj'].possibleFlightAreaStatic(area_length=area_length*1.0)
        if uav['ID'] == 1:
            dist2ncUAVs = uav['dubins'].getOtherUAVStates(uavh_others_all, uavID)

            plotCASpos, = plt.plot(uav['dubins'].ys, uav['dubins'].xs, 'o', c='r')
            color = '-g'
            if(replan):
                color = '-y'
            plotCAScone, = plt.plot([pt[1] for pt in pts], [pt[0] for pt in pts], color)
            plotCurrentWypt = plt.plot(mainUAV['dubins'].waypoints[mainUAV['dubins'].currentWPIndex][1], mainUAV['dubins'].waypoints[mainUAV['dubins'].currentWPIndex][0], c='black', marker='X')
            
            if usetargetPath: 
                #print(TC.OKBLUE+ '\tUsing Target Path' + TC.ENDC)
                uav['dubins'].simulateWPDubins(UseCarrotChase=False, delta=0.01)
                carrot = uav['dubins'].CarrotChaseWP(delta=0.01)
                CASuavPos = uav['dubins'].position
            elif useWPfollower == True:
                #print(TC.WARNING + '\tUsing A* Path' + TC.ENDC)
                # if uav['dubins'].lastWP or clearedOtherUAV:
                #     # Switch back to original path if the other UAV has moved away
                #     # or if the last A* waypoint has been reached
                #     # print(TC.WARNING + '\tRevert to original path' + TC.ENDC)
                #     useWPfollower = False
                #     usetargetPath = True
                #     uav['dubins'].lastWP = False
                #     uav['dubins'].setWaypoints(TargetWPList, newradius=0.01)
                #     detectRange, targetWP, targetIndex, astarGoalIndex, astarGoalPt = uav['dubins'].detectClosestWP(dist=0.3, theta_possible=uav['uavobj'].thetaPossible, alpha=4, targetPath=TargetWPList, returnMethod='useSmalletsAngle')
                #     if not targetWP:
                #         detectRange, targetWP, targetIndex, astarGoalIndex, astarGoalPt = uav['dubins'].detectClosestWP(dist=0.3, theta_possible=uav['uavobj'].thetaPossible, alpha=5, targetPath=TargetWPList, returnMethod='useSmalletsAngle')
                #         plotDetectRange.remove()
                #         plotTargetWP, = plt.plot(targetWP[1], targetWP[0], c='r', marker = '*')
                #         plotDetectRange, = plt.plot([pt[1] for pt in detectRange], [pt[0] for pt in detectRange], c='y')

                #     uav['dubins'].currentWPIndex = targetIndex

                uav['dubins'].simulateWPDubins(UseCarrotChase=True, delta=0.01)
                carrot = uav['dubins'].CarrotChaseWP(delta=0.01)
                plotCarrot, = plt.plot(carrot[1], carrot[0], c='orange', marker='^' )
                # if len(avoid)>1:
                #     plotCASkoz, = plt.plot([pt[1] for pt in avoid[1]], [pt[0] for pt in avoid[1]], '--m')
                #     plotNCkoz, = plt.plot([pt[1] for pt in avoid[1]], [pt[0] for pt in avoid[1]], '--m')
                # else:
                #     plotNCkoz = None
                #     plotCASkoz = None
                    
                CASuavPos = uav['dubins'].position
            else:
                uav['dubins'].update_pos_simple()
                #plotWypt, = ax.plot(mainUAV['uavobj'].waypoint[1], mainUAV['uavobj'].waypoint[0], 'X')
                CASuavPos = uav['dubins'].position

        if uav['IsAvoidanceUAV'] == False:
            plotNCpos, = plt.plot(uav['uavobj'].position[1], uav['uavobj'].position[0], 'o', c='orange')
            # plotNCpos, = plt.plot(uav['dubins'].ys, uav['dubins'].xs, 'o', c='orange')

            plotNCcone, = plt.plot([pt[1] for pt in pts], [pt[0] for pt in pts], "-r")
            uav['dubins'].update_pos_simple()
            NCuavPos = uav['dubins'].position

        uav = syncAVSfromDubins(uav)

    plotRefPath = plt.plot([pt[1] for pt in RaceTrack], [pt[0] for pt in RaceTrack], c='b', marker = '.', markersize = 8)

    for pts in RaceTrack:
            # plot circles around each waypoint - viusal aid for waypoint updating
            wptCircle = plt.Circle((pts[1], pts[0]), wptRad, color='green', alpha=0.2)
            plt.gca().add_artist(wptCircle)
            plt.scatter(pts[1],pts[0])

    if wpList2 !=None:
        for pts in NewPath:
                wptCircle = plt.Circle((pts[1], pts[0]), wptRad, color='green', alpha=0.2)
                plt.gca().add_artist(wptCircle)
                plt.scatter(pts[1],pts[0])

    if wpList2 != None:
        plt.plot([pt[1] for pt in NewPath], [pt[0] for pt in NewPath], c='green', marker='o',markersize=5)


    checkIfClear = []
    for i in range(0, len(uavh_others_all)):
        checkIfClear.append(mainUAV['dubins'].trackUAV[i]['clearedUAV'])
    
    if all(checkIfClear): # if all false then still tracking other UAVs
        #usetargetPath = False
        clearedOtherUAV = True
    else:
        #usetargetPath = True
        clearedOtherUAV = False
    print('\t\t' + str(checkIfClear))

    dist2WP = distance( [mainUAV['dubins'].x, mainUAV['dubins'].y], [mainUAV['dubins'].waypoints[mainUAV['dubins'].currentWPIndex][0], mainUAV['dubins'].waypoints[mainUAV['dubins'].currentWPIndex][1]] )

    plt.plot( activeWP[1], activeWP[0], c='k', marker='X', markersize = 8 )

    plt.axis('equal')
    plt.grid(True)

    plt.ylim((mainUAV['dubins'].x - 0.01, mainUAV['dubins'].x + 0.01))
    plt.xlim((mainUAV['dubins'].y - 0.01, mainUAV['dubins'].y + 0.01))
    plt.pause(0.05) 

    # if step > 68:
    #     plt.show(100)

    print('Step: ' + str(step) + '\tCurrent wpt: ' + str(uavlist[0]['dubins'].currentWPIndex) + '\tUAV Heading(deg): ' + str(round(uavlist[0]['dubins'].heading,2)) + ' (' + str(round(np.degrees(uavlist[0]['dubins'].heading),2)) + ')') 

    wd = os.getcwd()
    path=(wd + '/Movies')
    fname = '_tmp%03d.png' % step
    fname = os.path.join(path,fname)
    plt.savefig(fname)
    savePlots.append(fname)

    plt.clf()
    step+=1

print('Change directory to Movies... ')
wd = os.getcwd()
path = ('Movies')
newDir = os.path.join(wd,path)
os.chdir(str(newDir))
print('Making sim movie...')
# source: https://matplotlib.org/gallery/animation/movie_demo_sgskip.html
# additional resource: https://linux.die.net/man/1/mencoder
subprocess.call("mencoder 'mf://_tmp*.png' -mf type=png:fps=10 -ovc lavc "
            "-lavcopts vcodec=mpeg4 -oac copy -o animation.mp4", shell=True)

print('Clean up...')
for fname in savePlots:
    os.remove(fname)   

print('Reverting to previous Directory')
os.chdir(wd)
