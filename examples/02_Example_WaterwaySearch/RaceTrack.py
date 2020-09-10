
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
import sys, math, os, subprocess, time, shutil
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
import pickle


def del_folder_contents(folderPath):
    folder = folderPath
    for filename in os.listdir(folder):
        file_path = os.path.join(folder, filename)
        try:
            if os.path.isfile(file_path) or os.path.islink(file_path):
                os.unlink(file_path)
            elif os.path.isdir(file_path):
                shutil.rmtree(file_path)
        except Exception as e:
            print('Failed to delete %s. Reason: %s' % (file_path, e))

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
    

def crossTrackError(Path, uavPos):
    lastDist = 9999999
    for i in range(0, len(Path)-1):

        y1 = Path[i][0]
        y2 = Path[i+1][0]
        x1 = Path[i][1]
        x2 = Path[i+1][1]
        uavX = uavPos[1]
        uavY = uavPos[0]

        m = (y2-y1) / (x2-x1) # slope
        b = y2 - m*x2                      # y intercept
        if abs(uavX) < abs(x2) and abs(uavX) > abs(x1):
                y = m*uavX + b
                dist = distance([uavX, y], [uavX, uavY])
                if dist < lastDist:                                 # find the closest point for cross track error
                    crossError = dist
                    slope = m
                    yint = b
                    lastDist = dist 
        else:
            crossError = 0
            slope = 0
            yint = 0

    return crossError, slope, yint

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

wd = os.getcwd()

folderPath=(wd + '/RaceTrack_AstarFormatedInput')
del_folder_contents(folderPath)

folderPath=(wd + '/RaceTrack_AstarResults')
del_folder_contents(folderPath)

folderPath=(wd + '/RaceTrack_RecoveryPaths')
del_folder_contents(folderPath)

folderPath=(wd + '/RaceTrack_SelectedPaths')
del_folder_contents(folderPath)

RefRaceTrack = [[39.9680674, -82.8318678],[39.9659200, -82.8531621],[39.9576365, -82.8601268],
             [39.9439513, -82.8569246],[39.9407597, -82.8349099],[39.9399004, -82.8072913],[39.9453629, -82.7862372],
             [39.9576365, -82.7748696],[39.9683128, -82.7939224],[39.9681901, -82.8157770],[39.9680674, -82.8318678]]


Reverse_RefRaceTrack =[[39.9680674, -82.8318678], [39.9681901, -82.815777], [39.9683128, -82.7939224], [39.9576365, -82.7748696], 
                        [39.9453629, -82.7862372], [39.9399004, -82.8072913], [39.9407597, -82.8349099], [39.9439513, -82.8569246], 
                        [39.9576365, -82.8601268], [39.96592, -82.8531621], [39.9680674, -82.8318678]]


RaceTrack = [[39.9680674, -82.8318678],[39.9659200, -82.8531621],[39.9576365, -82.8601268],
             [39.9439513, -82.8569246],[39.9407597, -82.8349099],[39.9399004, -82.8072913],[39.9453629, -82.7862372],
             [39.9576365, -82.7748696],[39.9683128, -82.7939224],[39.9681901, -82.8157770],[39.9680674, -82.8318678]]
#keept Track of what waypoints belongs to the reference path
refPath = []
for i in range(0, len(RaceTrack)):
    refWPT = {}
    refPath.append(refWPT)
    refPath[i]['refPt'] = True
    refPath[i]['pt'] = RaceTrack[i]

# RaceTrack_meters = []
# for pt in RaceTrack:
#     x, y = pyproj.transform(wgs84, epsg3035, pt[1], pt[0])
#     RaceTrack_meters.append([x, y])

fig, ax = plt.subplots()
savePlots = []  # stores figure frames used to make a movie
savePlots1 = []
savePlots2 = []
savePlots3 = []
savePlots4 = []
savePlots5 = []
savePlots6 = []
savePlots7 = []
savePlots8 = []
savePlots9 = []
savePlots10 = []
savePlots11 = []
savePlots12 = []
savePlots13 = []
savePlots14 = []
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
wptRad = 0.001 # distance threshold to satisfy each waypoint
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
uavlist[0]['dubins'].setWaypoints(newwps=RefRaceTrack, newradius = wptRad )
uavlist[0]['dubins'].currentWPIndex = 0

#
v1 = 2*0.00025
thetaRef = np.deg2rad(0)
uavlist[1]['dubins'] = dubinsUAV(position=[39.9576365, -82.8601268], velocity=v1,         
                                    heading=thetaRef, dt=dt)
uavlist[1]['ID'] = 4
uavlist[1]['IsAvoidanceUAV'] = False
#
uavlist[0] = syncAVSfromDubins(uavlist[0])
uavlist[1] = syncAVSfromDubins(uavlist[1])
uavlist[1]['dubins'].setWaypoints(newwps=Reverse_RefRaceTrack, newradius = wptRad )
uavlist[1]['dubins'].currentWPIndex = 9

# from Ch 5 in the Pilot's Handbook of Aernautical Knowledge 
# Using 50% of maximum turn radius - conservative value - given by np.degrees(uavlist[0]['dubins'].turnrate)/2
uavTurnRadius = ((uavlist[0]['dubins'].v * 360/(np.degrees(uavlist[0]['dubins'].turnrate)))/np.pi)/2
halfTurnRadius = uavTurnRadius*4
#
print('UAV turn radius with full turn rate: ' + str(uavTurnRadius) + ' and half turn rate: ' + str(halfTurnRadius))
Recovery_dict = {'X': [], 'Y' : [], 'Index1' : [], 'Index2' : [], 'wpt1' : [], 'wpt2' : [],
                 'pathLength' : [], 'turnRadii': []   }
#
hasPath = False
hadPlan = False
hasPlan = False
onlyOnce = False
wpList2 = None
usetargetPath = False
TargetWPList = None
hasAstarPlan = False
hasRecoveryPlan = False

area_length = 0.005
numbOfAstarPts = 0
NewPath = []
indexTracker = 0 # TO DO - find another way to change the index to the appropriate waypoint
numbOfRecoveryPts = 0

Log_list=[[],[],[]]
AstarList_dict = {}
RecoveryList_dict= {}
TargetWPList_dict = {}

# Log_list[0].append(AstarList_dict)
# Log_list[1].append(RecoveryList_dict)
# Log_list[2].append(Log_dict)


step = 0
while step < 7500: #uavlist[0]['dubins'].lapCounter < 2: # uavlist[0]['dubins'].currentWPIndex < len(uavlist[0]['dubins'].waypoints)-1: 
    ''' Identify UAVs using collision avoidence '''
    mainUAV = finduavbyID(uavlist, 1) # IDtoWatch
    uavh_others_all, uavh_others = findotheruavs(uavlist, 1) # ID not to watch for

    activeWP = uavlist[0]['dubins'].waypoints[uavlist[0]['dubins'].currentWPIndex]
        
    '''
    Generate the main path. Goal is to reconnect to this path after avoiding an intruder UAV/obstacle
    - Note: This step is here in case I want to change the type of reference path to another shape -
    '''
    if TargetWPList == None:
        mainUAV['dubins'].getDist2otherUAVs(uavh_others_all)
        usetargetPath = True
        TargetWPList = RefRaceTrack[:]
        uavlist[0]['dubins'].setWaypoints(TargetWPList, newradius=wptRad)
        uavlist[0]['dubins'].currentWPIndex = 0
        TargetWPList_dict['SimStep']= step
        TargetWPList_dict['CurrentWPIndex']= mainUAV['dubins'].currentWPIndex
        TargetWPList_dict['TargetWPList'] = TargetWPList
        TargetWPList_dict['With A*'] = False
        TargetWPList_dict['With Recovery'] = False
        TargetWPList_dict['Original Path'] = True
        Log_list[2].append(TargetWPList_dict.copy())

    ''' 
    Locate closest and furthest waypoints laying on reference path
    Valid points are located inside a forward facing "Detection semi-circle"
    Furthest pt - Used as A* goal point
    Closest pt - Was used as the recovery point - old news, now using clothoids    
    If nothing is detected the detection area is enlarged
    '''
    detectRange, targetWP, targetIndex, astarGoalIndex, astarGoalPt = mainUAV['dubins'].detectClosestWP(dist=0.015, theta_possible=mainUAV['uavobj'].thetaPossible, alpha=1, targetPath=TargetWPList, returnMethod='useSmallestAngle')
    #print('targetWP: ' + str(targetWP) + ' astarGoal: ' + str(astarGoalPt) + ' not expanding detection cone')
    if len(targetWP)==0 or len(astarGoalPt)==0:
        # Nothing Detected - increase detection area size
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

    ''' 
    UAVHeading.avoid() - Use A* to generate a replan path to avoid a potential collision with another UAV
    replan - T/F - confirms that a replan occured and A* waypoints are available
    wplist - A* waypoint list
    avoid - List of lists - contains keep out zone for CAS UAV (which include the NC KOZ and the reverse KOZ)
    full_path - 
    uavID - Useful if using multiple NC UAVs. When A* is finding a solution, it only recieves a snapshot of keep out zones.
            If a potential collision is detected, it is unclear which NC UAV is the offending aircraft.
            Since I know the order of which NC UAV keep out zone is sent to A* to find a solution, I can use that knowledge to match each 
            keep out zone with a specific UAV ID. 
    '''
    if len(uavlist) > 1:
        if not hasPlan:
            astarGoalPt = activeWP
            hasAstarPlan = False
            replan, wplist, avoid, full_path, uavID, AstarFail = mainUAV['uavobj'].avoid(uavh_others, area_length=area_length, static_koz=[], TargetPathWP=astarGoalPt, useAstarGoal=True, simStep=step)
            AstarTargetIndex = mainUAV['dubins'].currentWPIndex

            # Deals with cases where the NC UAV may be on top of the A* target/active waypoint
            if AstarFail:
                print(TC.WARNING +'A* failed to find path' + TC.ENDC)
                if uavlist[0]['dubins'].currentWPIndex < len(RefRaceTrack):
                    Next_activeWP = RefRaceTrack[uavlist[0]['dubins'].currentWPIndex+1]
                    AstarTargetIndex = mainUAV['dubins'].currentWPIndex+1
                else:
                    Next_activeWP = RefRaceTrack[0]
                    AstarTargetIndex = 0
                print('Checking if next wpt on ref path is valid')
                astarGoalPt = Next_activeWP
                replan, wplist, avoid, full_path, uavID, AstarFail = mainUAV['uavobj'].avoid(uavh_others, area_length=area_length, static_koz=[], TargetPathWP=astarGoalPt, useAstarGoal=True, simStep=step)
                      
            ''' Use uavID to determine with NC UAV is being Avoided '''
            if len(uavID) > 0:
                for i in range(0, len(uavID)):    
                    uavID[i] = uavh_others_all[uavID[i]]['ID']
                    print('Potential Collision with UAV ' + str(uavID[i]))

        if replan or AstarFail==True:
            plotCASkoz, = plt.plot([pt[1] for pt in avoid[0]], [pt[0] for pt in avoid[0]], '--m')
            plotNCkoz, = plt.plot([pt[1] for pt in avoid[1]], [pt[0] for pt in avoid[1]], '--m')

        if(replan and not hasPlan):
            hasPlan = True
            hasAstarPlan = True
            hadPlan = True
            indexRecall = mainUAV['dubins'].currentWPIndex # Used to reset the currentWPIndex after A* and/or Recovery path is complete
            saveCurrentPOS = [mainUAV['dubins'].x, mainUAV['dubins'].y ]
            # insert current vehicle plostion into A* wplist
            wplist[0][0] = mainUAV['uavobj'].position[0]
            wplist[0][1] = mainUAV['uavobj'].position[1]
            wplist = np.append(wplist, np.array([[astarGoalPt[0], astarGoalPt[1]]]), axis=0)
            numbOfAstarPts= len(wplist)-1  # -1 accounts for the added astarGoalPt which is already on the reference path
            AstarList_dict['SimStep']= step
            AstarList_dict['CurrentWPIndex']= mainUAV['dubins'].currentWPIndex
            AstarList_dict['WPlist'] = wplist.tolist()
            AstarList_dict['A*TargetIndex'] = AstarTargetIndex
            AstarList_dict['A*Goal'] = astarGoalPt
            AstarList_dict['AstarFail'] = AstarFail
            Log_list[0].append(AstarList_dict.copy())

            # Insert A* points (wplist) into Reference path
            TargetWPList[mainUAV['dubins'].currentWPIndex:mainUAV['dubins'].currentWPIndex] = wplist.tolist()
            activeWP = uavlist[0]['dubins'].waypoints[uavlist[0]['dubins'].currentWPIndex]

            print(TC.OKBLUE + 'Insert ' + str(numbOfAstarPts) + ' Astar Points at wpt index ' + str(indexRecall) + TC.ENDC)
            closingDist = mainUAV['dubins'].distance(mainUAV['uavobj'].position, uavh_others[0].position)
            # print('Distance to other UAV: ' + str(closingDist))
            # print('POS: ' + str(mainUAV['uavobj'].position))
            # print('WP: ' + str(wplist))
            # print('DP: ' + str(deadpoint))  
            # useWPfollower = True
            # usetargetPath = False
            # clearedOtherUAV = False
            TargetWPList_dict['SimStep']= step
            TargetWPList_dict['CurrentWPIndex']= mainUAV['dubins'].currentWPIndex
            TargetWPList_dict['TargetWPList'] = TargetWPList
            TargetWPList_dict['A*Goal'] = astarGoalPt
            TargetWPList_dict['With A*'] = True
            TargetWPList_dict['With Recovery'] = False
            TargetWPList_dict['Original Path'] = False

            Log_list[2].append(TargetWPList_dict.copy())

            # Plot snapshot 
            plt.plot(mainUAV['dubins'].ys, mainUAV['dubins'].xs, c='r', marker='o' )
            plt.plot(uavh_others_all[0]['dubins'].y, uavh_others_all[0]['dubins'].x, c='y', marker='o' )
            plot_AstarPlan = plt.plot([pt[1] for pt in wplist.tolist()], [pt[0] for pt in wplist.tolist()], c = 'k', marker='*', markersize=8)
            plt.plot([pt[1] for pt in RefRaceTrack], [pt[0] for pt in RefRaceTrack], c='b', marker='.', markersize=8)

            plt.plot( activeWP[1], activeWP[0], c='k', marker='X', markersize = 5 )
            plt.plot( astarGoalPt[1], astarGoalPt[0], c='k', marker='^', markersize = 6 )

            # plot keep out zones from UAVHeading.avoid() function
            plotCASkoz, = plt.plot([pt[1] for pt in avoid[0]], [pt[0] for pt in avoid[0]], '--m')
            plotNCkoz, = plt.plot([pt[1] for pt in avoid[1]], [pt[0] for pt in avoid[1]], '--m')
            plt.axis('equal')
            plt.grid(True)
            plt.ylim((mainUAV['dubins'].x - 0.01, mainUAV['dubins'].x + 0.01))
            plt.xlim((mainUAV['dubins'].y - 0.01, mainUAV['dubins'].y + 0.01))
            #plt.show()
            fig.set_size_inches((12, 10)) 

            wd = os.getcwd()
            path=(wd + '/RaceTrack_AstarPath')
            AstarPath = 'AstarPath%03d.png' % step
            AstarPaths = os.path.join(path, AstarPath)
            plt.savefig(AstarPaths)
            plt.clf()

            # plt.pause(1)


        else:
            print("Not re-planning" )
    else:
        print('\tOnly one UAV')

    checkDistance = 9999999     # used to evaluate shortest clothoid path using interpolated target waypoints
    if hadPlan and mainUAV['dubins'].trackUAV[0]['clearedUAV']:
        hadPlan = False
        '''
        Convert uav North East Down angle convention to cartesion for clothoid heading:
        Needed an axis flip to find the angle between UAV and active waypoint: the x's in the numerator and y's in the denominator, then add 180 deg
        '''
        # Use dummy waypoint projected in front of CAS UAC to determine start heading for clothoid calcs
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

        numbOfwpts = 10     # how many waypoints per clothoid will be used for circle fitting (3 clothoids needed for a solution)
        clothoid_paths = [] # List of information on each clothoid path generated between UAV and waypoint on reference path
    
        '''
        Find shortest Clothoid path from the CAS UAV's current position back to the reference path
        Choose the closest interpolated point on the ref path as injection point
        Chosen point is interpolated between original waypoint index sets
        Chosen point provdes the shortes path that does not violate turn radius
        '''
        for index in range(0, len(RefRaceTrack)-1):  # TO DO change 7 to some wp index within a detected range of the UAV
            numbOfPts = 5                       # Number of interpolate points between each Reference Path index and index+1

            # Note that RefRaceTrack is the unchanged original refernce path
            if index == indexRecall-1:
                x1 = saveCurrentPOS[0]
                y1 = saveCurrentPOS[1]
            else:
                x1 = RefRaceTrack[index][0]
                y1 = RefRaceTrack[index][1]

            x2 = RefRaceTrack[index+1][0]
            y2 = RefRaceTrack[index+1][1]

            '''
            Use a linear space to interpolate points between each reference path waypoint
            Also, do not include the endpoint - it is included in the next index set
            '''
            linX = np.linspace(x1, x2, numbOfPts, endpoint=False )
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
                ClothoidPath = []   # Stores a specified number of waypoints for each clothoid segment
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
                        wpList2.append([points[0][1],points[1][1]])     # 1st point on first clothoid and for then entire path 
                        mid_x = i.X(s/2)
                        mid_y = i.Y(s/2)
                        mid_x2 = i.X(0.75*s)
                        mid_y2 = i.Y(0.75*s)
                        wpList2.append([mid_x,mid_y])                   # 2nd point
                        wpList2.append([mid_x2,mid_y2])                 # 3rd point

                    elif jj == 1:
                        wpList2.append([points[0][1],points[1][1]])     # 4th point - 1st point on the middle clothoid
                        mid_x = i.X(s/2)
                        mid_y = i.Y(s/2)
                        mid_x2 = i.X(0.75*s)
                        mid_y2 = i.Y(0.75*s)
                        wpList2.append([mid_x,mid_y])                   # 5th point 
                        wpList2.append([mid_x2,mid_y2])                 # 6th point

                    elif jj == 2:
                        wpList2.append([points[0][1],points[1][1]])     # 7th point - 1st point on the last clothoid
                        mid_x = i.X(s/2)        
                        mid_y = i.Y(s/2)
                        wpList2.append([mid_x,mid_y])                   # 8th point
                        wpList2.append([points[0][endPt],points[1][endPt]]) # 9th point - last point of the last clothoid - should be original interploated point


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
                        #First Half - provides circle center corrdinates and the circle radius
                        halfX_points1 = x_points[:(int(len(x_points)/2))]   
                        halfY_points1 = y_points[:(int(len(y_points)/2))] 
                        half_xc1,half_yc1,half_r1 = fit_circle_2d(halfX_points1, halfY_points1)

                        #Second Half - provides circle center corrdinates and the circle radius
                        halfX_points2 = x_points[(int(len(x_points)/2)):]   
                        halfY_points2 = y_points[(int(len(y_points)/2)):] 
                        half_xc2,half_yc2,half_r2 = fit_circle_2d(halfX_points2, halfY_points2)
                        
                        circle_list.append([half_xc1, half_yc1, half_r1])
                        circle_list.append([half_xc2, half_yc2, half_r2])

                    else:
                        xc,yc,r = fit_circle_2d(x_points, y_points)
                        circle_list.append([xc,yc,r])

                    jj+=1

                # plt.show(100) # uncomment if you want to see every clothoid enerated by itself
                completeClothoidPts.append(temp)        # store points of entire clothoid path
                clothoid_paths.append([index, clothoidLength, ClothoidPath, wpList2, completeClothoidPts, circle_list])   # store info for each clothoid
                Recovery_dict['pathLength'].append(clothoidLength)
                Recovery_dict['turnRadii'].append(circle_list)

                'Determine if path violates Turn Radius'
                print('\tPath Distance: ' + str(round(clothoidLength,3)) + '\tClothoid Radii: ' + str(round(circle_list[0][2],3)) + 
                      '   ' + str(round(circle_list[1][2],3)) + '   ' + str(round(circle_list[2][2],3)) + '   ' + str(round(circle_list[3][2],3)))
            
                checkPassed = False
                for r in range(0, len(circle_list)):                        
                    if circle_list[r][2] < uavTurnRadius:
                        print(TC.WARNING + '\t\t\tTurn radius too small' + TC.ENDC)
                        checkPassed = False
                        break
                    else:
                        checkPassed = True

                if checkPassed == True and clothoidLength < checkDistance:
                    selectPath = index+1
                    checkDistance = clothoidLength
                    hasRecoveryPlan = True
                    hasPlan = False
                    Recovery_dict['chosenPathFull'] = completeClothoidPts
                    Recovery_dict['chosenPath'] = wpList2
                    Recovery_dict['cirlces'] = circle_list
                    numbOfRecoveryPts = len(wpList2)
                    #Recovery_dict['chosenIndex'] = selectPath + numbOfRecoveryPts # + numbOfAstarPts
                    Recovery_dict['chosenIndex'] = mainUAV['dubins'].currentWPIndex + numbOfRecoveryPts # + numbOfAstarPts
                    Recovery_dict['returnIndex'] = selectPath
                    print(TC.OKGREEN + '\t\t\tSelected index pt: ' + str(selectPath) + ' which is now at index ' + str(Recovery_dict['chosenIndex']) + TC.ENDC)

                elif checkPassed == True and clothoidLength > checkDistance:
                    print(TC.WARNING + '\t\t\tPath Too Long' + TC.ENDC)
                    

            for clothoid in clothoid_paths:
                plt.plot([pt[1] for pt in clothoid[4][0]], [pt[0] for pt in clothoid[4][0]])

                # Uncomment if you want circle fits plotted for each interpolated path
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
        plt.plot(uavh_others_all[0]['dubins'].y, uavh_others_all[0]['dubins'].x, c='y', marker='o' )
        NCcone = uavh_others_all[0]['uavobj'].possibleFlightAreaStatic(area_length=area_length*1.0)
        plotNCcone, = plt.plot([pt[1] for pt in NCcone], [pt[0] for pt in NCcone], "-r")

        plot_AstarPlan = plt.plot([pt[1] for pt in wplist.tolist()], [pt[0] for pt in wplist.tolist()], c = 'k', marker='*', markersize=8)
        plt.plot([pt[1] for pt in RefRaceTrack], [pt[0] for pt in RefRaceTrack], c='b', marker='.', markersize=8)

        plt.plot( activeWP[1], activeWP[0], c='k', marker='X', markersize = 5 )
        # plot keep out zones from UAVHeading.avoid() function
        if replan:
            plotCASkoz, = plt.plot([pt[1] for pt in avoid[0]], [pt[0] for pt in avoid[0]], '--m')
            plotNCkoz, = plt.plot([pt[1] for pt in avoid[1]], [pt[0] for pt in avoid[1]], '--m')
        plt.axis('equal')
        plt.grid(True)
        plt.ylim((mainUAV['dubins'].x - 0.01, mainUAV['dubins'].x + 0.01))
        plt.xlim((mainUAV['dubins'].y - 0.01, mainUAV['dubins'].y + 0.01))
        # plt.show(100)
        #fig.set_size_inches((12, 10)) 
        # plt.pause(1)
        
        wd = os.getcwd()
        path=(wd + '/RaceTrack_RecoveryPaths')
        RecoveryPaths = 'RecoveryPaths%03d.png' % step
        RecoveryPaths = os.path.join(path,RecoveryPaths)
        plt.savefig(RecoveryPaths)
        plt.clf()
        # plt.close('all')


        if hasRecoveryPlan == False:
            print('No valid recovery path available')
        else:
            print(TC.OKGREEN + '\nSelected point between wp index ' + str(selectPath-1) + ' and ' + str(selectPath) + ' as the recovery insertion point' + TC.ENDC)

            plt.plot([pt[1] for pt in Recovery_dict['chosenPathFull'][0]], [pt[0] for pt in Recovery_dict['chosenPathFull'][0]])

            circle1 = plt.Circle((Recovery_dict['cirlces'][0][0], Recovery_dict['cirlces'][0][1]), Recovery_dict['cirlces'][0][2],color='magenta',alpha=0.2)
            plt.gca().add_artist(circle1)
            plt.scatter(Recovery_dict['cirlces'][0][0],Recovery_dict['cirlces'][0][1])

            circle2 = plt.Circle((Recovery_dict['cirlces'][1][0], Recovery_dict['cirlces'][1][1]), Recovery_dict['cirlces'][1][2],color='yellow',alpha=0.4)
            plt.gca().add_artist(circle2)
            plt.scatter(Recovery_dict['cirlces'][1][0],Recovery_dict['cirlces'][1][1])

            plt.scatter(Recovery_dict['cirlces'][2][0],Recovery_dict['cirlces'][2][1])

            circle4 = plt.Circle((Recovery_dict['cirlces'][3][0], Recovery_dict['cirlces'][3][1]), Recovery_dict['cirlces'][3][2],color='magenta',alpha=0.2)
            plt.gca().add_artist(circle4)
            plt.scatter(Recovery_dict['cirlces'][3][0],Recovery_dict['cirlces'][3][1])


            # Re-upload original path - Recovery points will later be inserted into waypoint list
            TargetWPList = RefRaceTrack[:]
            mainUAV['dubins'].setWaypoints(TargetWPList, newradius=wptRad)
            NewPath = []
            for ptList in Recovery_dict['chosenPath']:
                NewPath.append([ptList[1], ptList[0]])

            plt.plot([pt[1] for pt in NewPath], [pt[0] for pt in NewPath], c='green', marker='o',markersize=5)

            # plot keep out zones from UAVHeading.avoid() function
            if len(avoid)>1:
                plotCASkoz, = plt.plot([pt[1] for pt in avoid[0]], [pt[0] for pt in avoid[0]], '--m')
                plotNCkoz, = plt.plot([pt[1] for pt in avoid[1]], [pt[0] for pt in avoid[1]], '--m')

            #fig.set_size_inches((12, 10)) 

            plt.plot(mainUAV['dubins'].ys, mainUAV['dubins'].xs, c='r', marker='o' )
            plt.plot(uavh_others_all[0]['dubins'].y, uavh_others_all[0]['dubins'].x, c='y', marker='o' )

            NCcone = uavh_others_all[0]['uavobj'].possibleFlightAreaStatic(area_length=area_length*1.0)
            plotNCcone, = plt.plot([pt[1] for pt in NCcone], [pt[0] for pt in NCcone], "-r")

            plot_RecoveryPlan = plt.plot([pt[1] for pt in NewPath], [pt[0] for pt in NewPath], c = 'g', marker='o', markersize=8)
            plot_AstarPlan = plt.plot([pt[1] for pt in wplist.tolist()], [pt[0] for pt in wplist.tolist()], c = 'k', marker='*', markersize=8)
            plt.plot([pt[1] for pt in RefRaceTrack], [pt[0] for pt in RefRaceTrack], c='b', marker='.', markersize=8)
            plt.plot( activeWP[1], activeWP[0], c='k', marker='X', markersize = 5 )
            plt.axis('equal')
            plt.grid(True)
            plt.ylim((mainUAV['dubins'].x - 0.01, mainUAV['dubins'].x + 0.01))
            plt.xlim((mainUAV['dubins'].y - 0.01, mainUAV['dubins'].y + 0.01))
            # plt.show(100)
            # plt.pause(1)

            wd = os.getcwd()
            path=(wd + '/RaceTrack_SelectedPaths')
            SelectedPath = 'SelectedPath%03d.png' % step
            SelectedPath = os.path.join(path,SelectedPath)
            plt.savefig(SelectedPath)
            #plt.close('all')
            plt.clf()

    '''
    Update waypoint list with Recovery path or reload original reference path 
    if A* and/or Recovery path has been completed
    '''

    if hasAstarPlan == True and hasRecoveryPlan == False:
        if mainUAV['dubins'].currentWPIndex >= AstarTargetIndex + numbOfAstarPts:
            TargetWPList = RefRaceTrack[:]
            mainUAV['dubins'].setWaypoints(TargetWPList, newradius=wptRad)
            mainUAV['dubins'].currentWPIndex = AstarTargetIndex  # Need to test
            activeWP = uavlist[0]['dubins'].waypoints[uavlist[0]['dubins'].currentWPIndex]

            TargetWPList_dict['SimStep']= step
            TargetWPList_dict['CurrentWPIndex']= mainUAV['dubins'].currentWPIndex
            TargetWPList_dict['TargetWPList'] = TargetWPList
            TargetWPList_dict['With A*'] = False
            TargetWPList_dict['With Recovery'] = False
            TargetWPList_dict['Original Path'] = True

            Log_list[2].append(TargetWPList_dict.copy())
            print('Completed A* path without needing recover path')

            hasAstarPlan = False
            hadPlan = False
            hasPlan = False 

    elif hasAstarPlan == True or hasRecoveryPlan == True:
        if not onlyOnce:
            onlyOnce = True
            lastIndex = mainUAV['dubins'].currentWPIndex
            numbOfRecoveryPts = len(NewPath)
            # insertIndex = indexRecall + numbOfAstarPts 
            insertIndex = mainUAV['dubins'].currentWPIndex

            TargetWPList[insertIndex:insertIndex] = NewPath
          #  mainUAV['dubins'].currentWPIndex = insertIndex 
            activeWP = uavlist[0]['dubins'].waypoints[uavlist[0]['dubins'].currentWPIndex]

            hasAstarPlan = False
            print(TC.OKBLUE + 'Insert ' + str(numbOfRecoveryPts) + ' Recovery Points at wpt index ' + str(insertIndex) + TC.ENDC)

            RecoveryList_dict['SimStep'] = step
            RecoveryList_dict['CurrentWPIndex'] = mainUAV['dubins'].currentWPIndex
            RecoveryList_dict['WPlist'] = NewPath
            RecoveryList_dict['Recovery Index'] = Recovery_dict['chosenIndex']
            RecoveryList_dict['RefPath return Index'] = Recovery_dict['returnIndex']
            Log_list[1].append(RecoveryList_dict.copy())

            TargetWPList_dict['SimStep']= step
            TargetWPList_dict['CurrentWPIndex']= mainUAV['dubins'].currentWPIndex
            TargetWPList_dict['TargetWPList'] = TargetWPList
            TargetWPList_dict['Recovery Goal'] = NewPath[numbOfRecoveryPts-1]
            TargetWPList_dict['With A*'] = False
            TargetWPList_dict['With Recovery'] = True
            TargetWPList_dict['Original Path'] = False

            Log_list[2].append(TargetWPList_dict.copy())

        elif mainUAV['dubins'].currentWPIndex >= Recovery_dict['chosenIndex']:
            hasRecoveryPlan = False
            onlyOnce = False
            TargetWPList = RefRaceTrack[:]
            mainUAV['dubins'].setWaypoints(TargetWPList, newradius=wptRad)

            mainUAV['dubins'].currentWPIndex = Recovery_dict['returnIndex']  # Need to test
            activeWP = uavlist[0]['dubins'].waypoints[uavlist[0]['dubins'].currentWPIndex]


            TargetWPList_dict['SimStep']= step
            TargetWPList_dict['CurrentWPIndex']= mainUAV['dubins'].currentWPIndex
            TargetWPList_dict['TargetWPList'] = TargetWPList
            TargetWPList_dict['With A*'] = False
            TargetWPList_dict['With Recovery'] = False
            TargetWPList_dict['Original Path'] = True

            Log_list[2].append(TargetWPList_dict.copy())
            print('Completed Recovery path')



    '''===Update vehicle positions==='''
    for uav in uavlist:     
        pts = uav['uavobj'].possibleFlightAreaStatic(area_length=area_length*1.0)
        if uav['ID'] == 1:
            dist2ncUAVs = uav['dubins'].getOtherUAVStates(uavh_others_all, uavID)

            plotCASpos, = plt.plot(uav['dubins'].ys, uav['dubins'].xs, 'o', c='r')
            plotMainUAV =  plt.plot(uav['dubins'].y, uav['dubins'].x, 'o', c='k')

            color = '-g'
            if(replan):
                color = '-y'
                replan = False
            plotCAScone, = plt.plot([pt[1] for pt in pts], [pt[0] for pt in pts], color)
            plotCurrentWypt = plt.plot(mainUAV['dubins'].waypoints[mainUAV['dubins'].currentWPIndex][1], mainUAV['dubins'].waypoints[mainUAV['dubins'].currentWPIndex][0], c='black', marker='X')
            
            crossError, m, b = crossTrackError(RefRaceTrack, [uav['dubins'].x, uav['dubins'].y])

            if usetargetPath: 
                #print(TC.OKBLUE+ '\tUsing Target Path' + TC.ENDC)
                uav['dubins'].simulateWPDubins(UseCarrotChase=False, delta=0.01)
                carrot = uav['dubins'].CarrotChaseWP(delta=0.01)
                CASuavPos = uav['dubins'].position
            elif useWPfollower == True:
                uav['dubins'].simulateWPDubins(UseCarrotChase=True, delta=0.01)
                carrot = uav['dubins'].CarrotChaseWP(delta=0.01)
                plotCarrot, = plt.plot(carrot[1], carrot[0], c='orange', marker='^' )                
                CASuavPos = uav['dubins'].position
            else:
                uav['dubins'].update_pos_simple()
                CASuavPos = uav['dubins'].position

        if uav['IsAvoidanceUAV'] == False:
            if step == 343:
                debud=1

            plotNCpos, = plt.plot(uav['uavobj'].position[1], uav['uavobj'].position[0], 'o', c='orange')
            # plotNCpos, = plt.plot(uav['dubins'].ys, uav['dubins'].xs, 'o', c='orange')
            NCactiveWPt = uav['dubins'].getActiveWaypoint()
            plt.plot( NCactiveWPt[1], NCactiveWPt[0], c='k', marker='v', markersize = 8 )

            plotNCcone, = plt.plot([pt[1] for pt in pts], [pt[0] for pt in pts], "-r")
            uav['dubins'].simulateWPDubins(UseCarrotChase=False, delta=0.01)
            NCuavPos = uav['dubins'].position

        uav = syncAVSfromDubins(uav)

        print('\nStep: ' + str(step) + '\tUAV ID: ' + str(uav['ID']) + '\tCurrent wpt: ' + str(uav['dubins'].currentWPIndex) + 
              '\tUAV Heading (deg): ' + str(round(uav['dubins'].heading,2)) + 
              ' (' + str(round(np.degrees(uav['dubins'].heading),2)) + ')'   )
        

    ''' Plotting '''
    plotRefPath = plt.plot([pt[1] for pt in RefRaceTrack], [pt[0] for pt in RefRaceTrack], c='b', marker = '.', markersize = 8)
    # plot keep out zones from UAVHeading.avoid() function
    if replan:
        plotCASkoz, = plt.plot([pt[1] for pt in avoid[0]], [pt[0] for pt in avoid[0]], '--m')
        plotNCkoz, = plt.plot([pt[1] for pt in avoid[1]], [pt[0] for pt in avoid[1]], '--m')
    
    for pts in RefRaceTrack:
            # plot circles around each waypoint - viusal aid for waypoint updating
            wptCircle = plt.Circle((pts[1], pts[0]), wptRad, color='green', alpha=0.2)
            plt.gca().add_artist(wptCircle)
            plt.scatter(pts[1],pts[0])

    if hasAstarPlan:
        plot_AstarPlan = plt.plot([pt[1] for pt in wplist.tolist()], [pt[0] for pt in wplist.tolist()], c = 'k', marker='*', markersize=8)
        for pts in wplist.tolist():
                wptCircle = plt.Circle((pts[1], pts[0]), wptRad, color='green', alpha=0.2)
                plt.gca().add_artist(wptCircle)
                plt.scatter(pts[1],pts[0])


    if hasRecoveryPlan :
        plot_RecoveryPlan = plt.plot([pt[1] for pt in NewPath], [pt[0] for pt in NewPath], c = 'g', marker='o', markersize=8)
        for pts in NewPath:
                wptCircle = plt.Circle((pts[1], pts[0]), wptRad, color='green', alpha=0.2)
                plt.gca().add_artist(wptCircle)
                plt.scatter(pts[1],pts[0])

    checkIfClear = []
    for i in range(0, len(uavh_others_all)):
        checkIfClear.append(mainUAV['dubins'].trackUAV[i]['clearedUAV'])
    
    if all(checkIfClear): # if all false then still tracking other UAVs
        #usetargetPath = False
        clearedOtherUAV = True
    else:
        #usetargetPath = True
        clearedOtherUAV = False
    #print('\t\t' + str(checkIfClear))

    dist2WP = distance( [mainUAV['dubins'].x, mainUAV['dubins'].y], [mainUAV['dubins'].waypoints[mainUAV['dubins'].currentWPIndex][0], mainUAV['dubins'].waypoints[mainUAV['dubins'].currentWPIndex][1]] )

    plt.plot( activeWP[1], activeWP[0], c='k', marker='X', markersize = 8 )

    fig.set_size_inches((12, 10))  
    plt.axis('equal')
    plt.grid(True)
    # plt.ylim(39.92, 39.99)
    # plt.xlim(-82.85, -82.78)

    # plt.ylim((fakeCenter[0] - zoom, fakeCenter[0] + zoom))
    # plt.xlim((fakeCenter[1] - zoom, fakeCenter[1] + zoom+0.0005))

    plt.ylim((mainUAV['dubins'].x - 0.01, mainUAV['dubins'].x + 0.01))
    plt.xlim((mainUAV['dubins'].y - 0.01, mainUAV['dubins'].y + 0.01))


    # plt.ylim((uavlist[1]['dubins'].x - 0.01, uavlist[1]['dubins'].x + 0.01))
    # plt.xlim((uavlist[1]['dubins'].y - 0.01, uavlist[1]['dubins'].y + 0.01))
    text = ("Sim Step: " + str(step) + ' Lap: ' + str(uavlist[0]['dubins'].lapCounter) + "\nCAS/NC Current wpt: " + str(uavlist[0]['dubins'].currentWPIndex) + '/' + str(uavlist[1]['dubins'].currentWPIndex) +
            '\nCross Track Error: ' + str(np.round(crossError,3))            ) 
    
    if step > 3361:
        mydebug =1
    if step > 1282:
        mydebug =1

    if hasAstarPlan == False and hasRecoveryPlan == False:
        text1 = ("Currently Following: Ref Path")
    elif hasAstarPlan == True and hasRecoveryPlan == False:
        text1 = ("Currently Following: A* path")    
    elif hasAstarPlan == False and hasRecoveryPlan == True:
        text1 = ("Currently Following: Recovery Path")

    plt.text(0.1, 0.05, text, transform=ax.transAxes)
    plt.text(0.1, 0.9, text1, transform=ax.transAxes)

    # plt.pause(0.005) 
    #time.sleep(0.001)

    # print('Step: ' + str(step) + '\tCurrent wpt: ' + str(uavlist[0]['dubins'].currentWPIndex) + '\tUAV POS: ' + str(mainUAV['dubins'].x) + ', ' + str(mainUAV['dubins'].y) + 
    #         '\tUAV Heading (deg): ' + str(round(uavlist[0]['dubins'].heading,2)) + ' (' + str(round(np.degrees(uavlist[0]['dubins'].heading),2)) + ')' + '\tCross Track Error: ' 
    #         + str(crossError) + ' m: ' + str(m) + ' b: ' + str(b)) 

    ''' 
    Save frames for a movie 
    '''
    wd = os.getcwd()
    path=(wd + '/Movies')
    # fname = '_tmp%03d.png' % step
    # fname = os.path.join(path,fname)
    # plt.savefig(fname)

    if step < 500:
        fname = '_tmpz%03d.png' % step
        fname = os.path.join(path,fname)
        plt.savefig(fname)
        savePlots.append(fname)

    elif step >= 500 and step < 1000:
        fname1 = '_tmpa%03d.png' % step
        fname1 = os.path.join(path,fname1)
        plt.savefig(fname1)        
        savePlots1.append(fname1)

    elif step >= 1000 and step < 1500:
        fname2 = '_tmpb%03d.png' % step
        fname2 = os.path.join(path,fname2)
        plt.savefig(fname2)
        savePlots2.append(fname2)  

    elif step >= 1500 and step < 2000:
        fname3 = '_tmpc%03d.png' % step
        fname3 = os.path.join(path,fname3)
        plt.savefig(fname3)
        savePlots3.append(fname3)  

    elif step >= 2000 and step < 2500:
        fname4 = '_tmpd%03d.png' % step
        fname4 = os.path.join(path,fname4)
        plt.savefig(fname4)
        savePlots4.append(fname4)  

    elif step >= 2500 and step < 3000:
        fname5 = '_tmpe%03d.png' % step
        fname5 = os.path.join(path,fname5)
        plt.savefig(fname5)
        savePlots5.append(fname5)  

    elif step >= 3000 and step < 3500:
        fname6 = '_tmpf%03d.png' % step
        fname6 = os.path.join(path,fname6)
        plt.savefig(fname6)
        savePlots6.append(fname6)  

    elif step >= 3500 and step < 4000:
        fname7 = '_tmpg%03d.png' % step
        fname7 = os.path.join(path,fname7)
        plt.savefig(fname7)
        savePlots7.append(fname7)

    elif step >= 4000 and step < 4500:
        fname8 = '_tmph%03d.png' % step
        fname8 = os.path.join(path,fname8)
        plt.savefig(fname8)
        savePlots8.append(fname8)  

    elif step >= 4500 and step < 5000:
        fname9 = '_tmpi%03d.png' % step
        fname9 = os.path.join(path,fname9)
        plt.savefig(fname9)
        savePlots9.append(fname9)  

    elif step >= 5000 and step < 5500:
        fname10 = '_tmpj%03d.png' % step
        fname10 = os.path.join(path,fname10)
        plt.savefig(fname10)
        savePlots10.append(fname10)  

    elif step >= 5500 and step < 6000:
        fname11 = '_tmpk%03d.png' % step
        fname11 = os.path.join(path,fname11)
        plt.savefig(fname11)
        savePlots11.append(fname11)

    elif step >= 6000 and step < 6500:
        fname12 = '_tmpl%03d.png' % step
        fname12= os.path.join(path,fname12)
        plt.savefig(fname12)
        savePlots12.append(fname12)  

    elif step >= 6500 and step <7000:
        fname13 = '_tmpm%03d.png' % step
        fname13 = os.path.join(path,fname13)
        plt.savefig(fname13)
        savePlots13.append(fname13)  

    elif step >= 7000 and step < 7500:
        fname14 = '_tmpn%03d.png' % step
        fname14 = os.path.join(path,fname14)
        plt.savefig(fname14)
        savePlots14.append(fname14)

    plt.clf()
    step+=1



pickle.dump({"uavlist": uavlist}, open('RaceTrack_result.p', 'wb'))
pickle.dump({"LogList": Log_list}, open('RaceTrack_Log.p', 'wb'))
# pickle.dump({"uavlist": uavlist}, open('recovery_result.p', 'wb'))

'''
Make a Movie
'''
print('Change directory to Movies... ')
wd = os.getcwd()
path = ('Movies')
newDir = os.path.join(wd,path)
os.chdir(str(newDir))
print('Making sim movie...')
# source: https://matplotlib.org/gallery/animation/movie_demo_sgskip.html
# additional resource: https://linux.die.net/man/1/mencoder
subprocess.call("mencoder 'mf://_tmpz*.png' -mf type=png:fps=10 -ovc lavc "
                "-lavcopts vcodec=mpeg4 -oac copy -o animation.mp4", shell=True)

subprocess.call("mencoder 'mf://_tmpa*.png' -mf type=png:fps=10 -ovc lavc "
                "-lavcopts vcodec=mpeg4 -oac copy -o animation1.mp4", shell=True)

subprocess.call("mencoder 'mf://_tmpb*.png' -mf type=png:fps=10 -ovc lavc "
                "-lavcopts vcodec=mpeg4 -oac copy -o animation2.mp4", shell=True)

subprocess.call("mencoder 'mf://_tmpc*.png' -mf type=png:fps=10 -ovc lavc "
                "-lavcopts vcodec=mpeg4 -oac copy -o animation3.mp4", shell=True)

subprocess.call("mencoder 'mf://_tmpd*.png' -mf type=png:fps=10 -ovc lavc "
                "-lavcopts vcodec=mpeg4 -oac copy -o animation4.mp4", shell=True)

subprocess.call("mencoder 'mf://_tmpe*.png' -mf type=png:fps=10 -ovc lavc "
                "-lavcopts vcodec=mpeg4 -oac copy -o animation5.mp4", shell=True)

subprocess.call("mencoder 'mf://_tmpf*.png' -mf type=png:fps=10 -ovc lavc "
                "-lavcopts vcodec=mpeg4 -oac copy -o animation6.mp4", shell=True)

subprocess.call("mencoder 'mf://_tmpg*.png' -mf type=png:fps=10 -ovc lavc "
                "-lavcopts vcodec=mpeg4 -oac copy -o animation7.mp4", shell=True)

#####
subprocess.call("mencoder 'mf://_tmph*.png' -mf type=png:fps=10 -ovc lavc "
                "-lavcopts vcodec=mpeg4 -oac copy -o animation7.mp4", shell=True)
subprocess.call("mencoder 'mf://_tmpi*.png' -mf type=png:fps=10 -ovc lavc "
                "-lavcopts vcodec=mpeg4 -oac copy -o animation7.mp4", shell=True)
subprocess.call("mencoder 'mf://_tmpj*.png' -mf type=png:fps=10 -ovc lavc "
                    "-lavcopts vcodec=mpeg4 -oac copy -o animation7.mp4", shell=True)
subprocess.call("mencoder 'mf://_tmpk*.png' -mf type=png:fps=10 -ovc lavc "
                "-lavcopts vcodec=mpeg4 -oac copy -o animation7.mp4", shell=True)
subprocess.call("mencoder 'mf://_tmpl*.png' -mf type=png:fps=10 -ovc lavc "
                "-lavcopts vcodec=mpeg4 -oac copy -o animation7.mp4", shell=True)
subprocess.call("mencoder 'mf://_tmpm*.png' -mf type=png:fps=10 -ovc lavc "
                    "-lavcopts vcodec=mpeg4 -oac copy -o animation7.mp4", shell=True)
subprocess.call("mencoder 'mf://_tmpn*.png' -mf type=png:fps=10 -ovc lavc "
                "-lavcopts vcodec=mpeg4 -oac copy -o animation7.mp4", shell=True)
print('Clean up...')
for fname in savePlots:
    os.remove(fname)   
print('Clean up...')
for fname in savePlots1:
    os.remove(fname) 
print('Clean up...')
for fname in savePlots2:
    os.remove(fname) 
print('Clean up...')
for fname in savePlots3:
    os.remove(fname)   
print('Clean up...')
for fname in savePlots4:
    os.remove(fname) 
print('Clean up...')
for fname in savePlots5:
    os.remove(fname) 
    print('Clean up...')
for fname in savePlots6:
    os.remove(fname)   
print('Clean up...')
for fname in savePlots7:
    os.remove(fname) 
for fname in savePlots8:
    os.remove(fname) 
print('Clean up...')
for fname in savePlots9:
    os.remove(fname) 
print('Clean up...')
for fname in savePlots10:
    os.remove(fname)   
print('Clean up...')
for fname in savePlots11:
    os.remove(fname) 
print('Clean up...')
for fname in savePlots12:
    os.remove(fname) 
    print('Clean up...')
for fname in savePlots13:
    os.remove(fname)   
print('Clean up...')
for fname in savePlots14:
    os.remove(fname) 

print('Reverting to previous Directory')
os.chdir(wd)
