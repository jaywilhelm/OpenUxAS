# Return2RoutneFunctions_TestScript

# Jeremy Browne October 2020
# Function version of waypoint based return-to-route script
# Procedure:
# 1) Follow refference path until collision event with 
#    a non-coopterative UAV
# 2) Avoid using A* generated collision-free replan path
# 3) Continue to avoid until the offending UAV has passed
# 4) Return to reference path using a clothoid path
#

import numpy as np
from matplotlib import pyplot as plt
import sys, math, os, subprocess, time, shutil
from dubins_Return2Route import dubinsUAV
from TerminalColors import TerminalColors as TC

# For clothoid path generation
import pyclothoids
from pyclothoids import Clothoid
from scipy import linalg

# For A* Collision Avoidance
from lineSegmentAoE import *
sys.path.append('../UAVHeading-CollisionAvoidance/src')
from UAVHeading import UAVHeading

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

def distance(a, b):
    return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

#############################################
### NEW FUNCTIONS ###########################
#############################################

''' Function to find the current index number assigned to each reference path waypoint '''
def findRefPathIndex(ActivePath_List):
    temp_list = []
    temp_dict = {}                    # temporary dictionary entry
    for i in range(0, len(ActivePath_List)):
        if ActivePath_List[i]['Belongs to'] == 'Reference Path' or ActivePath_List[i]['Belongs to'] == 'Recovery' :
            temp_dict['Index'] = i
            temp_dict['pt'] = ActivePath_List[i]['pt']
            temp_list.append(temp_dict.copy())

    return temp_list

# Find the A* goal point 
def findAstarGoal(refWaypoint_list, ReferencePath_List, uavObj, area_length, lookAheadDist):
    last_targetIndex = 0
    for i in range(0, len(refWaypoint_list)):
        if uavObj['dubins'].currentWPIndex == refWaypoint_list[i]['Index']:
            currentIndex = i

    astarGoalPoint_list = refWaypoint_list
    astarGoalPoint_dict = {}
    # Add in aditional Reference path in case the UAV is about to make a lap
    # This step is performed after the 'currentIndex' step above 
    # because there will be douplicate 'Index' entries
    for j in range(0, len(ReferencePath_List)):
        astarGoalPoint_dict['Index'] = j
        astarGoalPoint_dict['pt'] = ReferencePath_List[j]['pt']
        astarGoalPoint_list.append(astarGoalPoint_dict.copy()) 

    distTotal = 0
    pointList = []
    for index in range(currentIndex, len(astarGoalPoint_list)):
        if index == currentIndex:
            # Used to find the distance between current UAV position and the next waypoint
            x1 = uavObj['dubins'].x
            y1 = uavObj['dubins'].y 

            x2 = astarGoalPoint_list[index]['pt'][0]
            y2 = astarGoalPoint_list[index]['pt'][1]
        else:
            # Used to find the distance between two successive waypoints
            x1 = astarGoalPoint_list[index-1]['pt'][0]
            y1 = astarGoalPoint_list[index-1]['pt'][1]

            x2 = astarGoalPoint_list[index]['pt'][0]
            y2 = astarGoalPoint_list[index]['pt'][1]
        
        '''
        Use a linear space to interpolate points between each reference or recovery path waypoint
        Note - process does not include the endpoint ( ie. [x2,y2]) - it is included in the next index set
        '''
        dist2WP = distance ([y1, x1], [y2, x2])
        numbOfPts = int(dist2WP/0.000005)   # number of interpolated points based on distance between waypoints
        if numbOfPts == 0:
            numbOfPts = 50
        linX = np.linspace(x1, x2, numbOfPts, endpoint=False )
        linY = np.linspace(y1, y2, numbOfPts, endpoint=False )

        ''' 
        Look for an interpolated point some distance forward along the reference path as the astarGoal point
        '''
        for pt in range(0, numbOfPts-1):
            d = distance([linX[pt], linY[pt]], [linX[pt+1], linY[pt+1]]) # Calculate distance between interpolated points
            distTotal += d  
            pointList.append([linX[pt], linY[pt]])
            if distTotal >= area_length*lookAheadDist:
                astarGoalPt = [linX[pt], linY[pt]]
                ' Report if the astarGoalPoint is on the next lap'
                targetIndex = astarGoalPoint_list[index]['Index']
                if targetIndex <= last_targetIndex:
                    lap = True
                else:
                    lap = False
                    last_targetIndex = targetIndex
                break
        if distTotal >= area_length*lookAheadDist:
            break

    return astarGoalPt, targetIndex, pointList, lap 

def UpdateWPList(ActivePath_List, wypts2add, numbOfPts, List_belongsTo, insertIndex):
    ''' Update Active path with avoidance or recovery path waypoints'''
    Waypoint_List = []
    Waypoint_dict = {}
    for i in range(0, len(wypts2add)):
        Waypoint_dict['pt'] = wypts2add[i]    
        Waypoint_dict['Belongs to'] = List_belongsTo
        Waypoint_dict['A* Return Index'] = mainUAV['dubins'].currentWPIndex + numbOfPts
        Waypoint_dict['A* Return Pt'] = 'Astar'
        Waypoint_dict['Recover Return Index']  = None
        Waypoint_dict['Recover Return Pt']  = None
        Waypoint_dict['Is Go-To'] = False

        Waypoint_List.append(Waypoint_dict.copy())

    ''' Insert new points into Active waypoint list at specified index'''
    ActivePath_List[insertIndex : insertIndex] = Waypoint_List
            
def clearWPList(ActivePath_List, ReferencePath_List, lap_Counter, uavClass ):
    ''' Completed a lap - clear AcitvePath_List and replace with Reference Path'''
    if uav['dubins'].lapCounter > lap_Counter:
        for i in range(0, len(ActivePath_List)):
            if ActivePath_List[i]['Is Go-To'] == True:
                uav['dubins'].currentWPIndex = i
                break

        ActivePath_List = ReferencePath_List
        TargetWPList = []
        for i in range(0, len(ReferencePath_List)):
            TargetWPList.append(ActivePath_List[i]['pt'])

        uav['dubins'].currentWPIndex = 0
        uav['dubins'].setWaypoints(TargetWPList, newradius=wptRad)

    return ActivePath_List

def getPotentialRecoveryPoints(ActivePath_List, saveCurrentPOS, indexRecall, numbOfPts):

    InsertionPoint_list = findRefPathIndex(ActivePath_List)    
    InsetionPoint_dict = {}

    # Add in additional Reference path points in case the UAV is about to make a lap
    for j in range(0, 2):
        InsetionPoint_dict['Index'] = j
        InsetionPoint_dict['pt'] = ReferencePath_List[j]['pt']
        InsertionPoint_list.append(InsetionPoint_dict.copy()) 

    # Determine which waypoint on the referece path the UAV is heading towards and report that index
    closestIndex = 9999
    for i in range(0, len(InsertionPoint_list)):
        checkIndex = InsertionPoint_list[i]['Index']
        diff = np.abs(checkIndex - indexRecall)
        if diff < closestIndex:
            currentIndex = i
            closestIndex = diff

    RecoveryPoints = []
    referencePathPoint = []
    for index in range(currentIndex, len(InsertionPoint_list)-1):  
        ' Find potential insetion points along the reference path ' 
        if index == currentIndex:
            # This is where the CAS UAV left the reference path to follow A* replan
            x1 = saveCurrentPOS[0]
            y1 = saveCurrentPOS[1]
        else:
            # Point in the reference path
            x1 = InsertionPoint_list[index]['pt'][0]
            y1 = InsertionPoint_list[index]['pt'][1]

        # Next point on the reference path
        x2 = InsertionPoint_list[index+1]['pt'][0]
        y2 = InsertionPoint_list[index+1]['pt'][1]

        '''
        Use a linear space to interpolate points between each reference path waypoint [x1,y1] and [x2, y2]
        Note - interpolation does not include the endpoint - which is included in the next index set
        '''
        linX = np.linspace(x1, x2, numbOfPts, endpoint=False )
        linY = np.linspace(y1, y2, numbOfPts, endpoint=False )
        
        for i in range(0, len(linX)):
            RecoveryPoints.append([linX[i], linY[i]])
            referencePathPoint.append([x2, y2, index])

    return RecoveryPoints, referencePathPoint, InsertionPoint_list

def getRecoveryPaths(RecoveryPoints, referencePathPoint, UAVx, UAVy):
#     print('Interpolating ' + str(numbOfPts) + ' points between index ' + str(index) + ' and ' + str(index+1))

    '''
    Convert uav North East Down angle convention to cartesion for clothoid heading:
    Needed an axis flip to find the angle between UAV and active waypoint- x's in the numerator and y's in the denominator, then add 180 deg
    '''
    # Use dummy waypoint projected in front of CAS UAV to determine start heading for clothoid calcs
    xy = (mainUAV['dubins'].x, mainUAV['dubins'].y)
    r = 0.3
    px = xy[0] + r * np.cos(mainUAV['dubins'].heading)
    py = xy[1] + r * np.sin(mainUAV['dubins'].heading)

    uavHeading = np.arctan2(UAVx - px, UAVy - py) + np.radians(180)
    # Keep uavHeading within [0, 360] degrees
    if(uavHeading >= np.pi*2):
        uavHeading -= np.pi*2
    if(uavHeading < 0):
        uavHeading += np.pi*2

    clothoid_List = []
    index_List = []
    offset = 3
    for pt in range(offset, len(RecoveryPoints)-7):

        ''' 
        Calculate heading between interpolated point and next reference waypoint 
        This is used for the clothoid curvature calculation
        '''
        x1 = RecoveryPoints[pt][0]
        y1 = RecoveryPoints[pt][1]
        x2 = referencePathPoint[pt][0]
        y2 = referencePathPoint[pt][1]   
        index = referencePathPoint[pt][2]

        targetHeading = np.arctan2(x1 - x2, y1 - y2) + np.radians(180) 

        '''
        Clothoid solver will be finding a path between the current CAS UAV position
        and the targetWPT interpolated point in the path 
        '''
        targetWPT = [x1, y1, targetHeading]  

#         plt.plot(targetWPT[1], targetWPT[0], 'o', markersize=5)

        if(targetWPT[2] >= np.pi*2):
            targetWPT[2] -= np.pi*2
        if(targetWPT[2] < 0):
            targetWPT[2] += np.pi*2

        # Provides clothoid parameters used to generate a path between the UAV and target waypoint on reference path: 3 clothoids per path are needed
        # Stiches the 3 clothoids together
        clothoid = pyclothoids.SolveG2(UAVy, UAVx, uavHeading, 0, targetWPT[1], targetWPT[0], targetWPT[2], 0) 

        clothoid_List.append(clothoid) # list of clothoids between UAV and each interpolated recovery point
        index_List.append(index)

    return clothoid_List, index_List

def selectRecoveryPath(clothoid_List, index_List, numbOfwpts):
    'List and counter variables for clothoid path selection'
    ClothoidPath = []           # Stores a specified number of waypoints for each clothoid segment
    recoveryPoints = []           # Stored the 1st, middle, and last wpt from the 1st, 2nd, and 3rd clothoid respectively

    completeClothoidPts = []    # stores a the full path of each clothoid generated -  used for plotting purposes
    counter = 0
    checkDistance = 99999
    ''' Generate and Evaluate clothoid paths between UAV and recovery points''' 
    for i in range(0, len(clothoid_List)):
        clothoid = clothoid_List[i]
        index = index_List[i]
        j=0                           # Keeps track of which clothoid segment is being evaluated
        clothoidLength = 0          # keeps track of clothoid path length
        temp = []                   # temporary variable - used for ploting purposes
        circle_list = []

        for segmt in clothoid:
            'Sample points from the clothoid'
            points = segmt.SampleXY(numbOfwpts)         # used for circle fitting to calcuate a turn radius for each clothoid path and used for waypoint path following
            ClothoidPath.append(points)

            x0, y0, t0, k0, dk, s = segmt.Parameters    # start x, start y, initial curvature, change in curvature, clothoid segment length
            clothoidLength += s                     # Sum up the length of each clothoid segment to find the total distance of the full clothoid
            
            '''
            Fit a circle to clothoid segment  
            Used to determine the needed Turn Radius of clothoid segment
            '''
            # Full set of clothoid points broken into x and y lists
            x_points = points[0]
            y_points = points[1]

            # Middle clothoid has two curvatures --> break into two halves 
            # and fit a circle to each half. 
            if j == 1: 
                # First Half - provides circle center coordinates and the circle radius
                halfX_points1 = x_points[:(int(len(x_points)/2))]   
                halfY_points1 = y_points[:(int(len(y_points)/2))] 
                half_xc1, half_yc1, half_r1 = fit_circle_2d(halfX_points1, halfY_points1)

                # Second Half - provides circle center coordinates and the circle radius
                halfX_points2 = x_points[(int(len(x_points)/2)):]   
                halfY_points2 = y_points[(int(len(y_points)/2)):] 
                half_xc2, half_yc2, half_r2 = fit_circle_2d(halfX_points2, halfY_points2)
                
                circle_list.append([half_xc1, half_yc1, half_r1])
                circle_list.append([half_xc2, half_yc2, half_r2])

            else:
                xc,yc,r = fit_circle_2d(x_points, y_points)
                circle_list.append([xc,yc,r])

            j+=1


        print('\tChecking pt ' + str(counter+1) + '/' + str(len(clothoid_List)))
        counter+=1

        'Determine if clothoid violates UAV Turn Radius'
        print('\tPath Distance: ' + str(round(clothoidLength,3)) + '\tClothoid Segment Radii: ' + str(round(circle_list[0][2],3)) + 
                '   ' + str(round(circle_list[1][2],3)) + '   ' + str(round(circle_list[2][2],3)) + '   ' + str(round(circle_list[3][2],3)))

        checkPassed = False 
        for i in range(0, len(circle_list)):                        
            if circle_list[i][2] < halfTurnRadius:
                print(TC.WARNING + '\t\t\tTurn radius too small' + TC.ENDC)
                checkPassed = False
                break
            else:
                checkPassed = True # means that clothoid path satisfies requierments and UAV dynamics

        if checkPassed == True and clothoidLength < checkDistance:               
                checkDistance = clothoidLength
                chosenClothoid = clothoid
                selectIndex = index+1
                print(TC.OKGREEN + '\t\t\tSelected index pt: ' +  str(selectIndex) + TC.ENDC)

        elif checkPassed == True and clothoidLength > checkDistance:
            print(TC.WARNING + '\t\t\tPath Too Long' + TC.ENDC)

    return chosenClothoid, selectIndex

def getRecoveryPathWPs(chosenClothoid, numbOfwpts):
    '''
    The core set of points from clothoids that define the recovery path:
    - first point of the 1st clothoid
    - middle point of the 2nd clothoid and
    - last point of the 3rd clothoid segment to follow
    With some additional points
    '''
    RecoveryPathWPs = []
    pltPts_List = []
    j = 0
    for segmt in chosenClothoid:
        'Sample points from the clothoid'
        pltpts = segmt.SampleXY(500)
        for i in range(0, len(pltpts[0])):
            pltPts_List.append([pltpts[1][i], pltpts[0][i]])
        
        points = segmt.SampleXY(numbOfwpts)  
        x0, y0, t0, k0, dk, s = segmt.Parameters    # start x, start y, initial curvature, change in curvature, clothoid segment length

        midPt = int(len(points[0])/2)
        endPt = len(points[0])-1
        if j == 0:
            RecoveryPathWPs.append([points[1][1],points[0][1]])     # 1st point on first clothoid and for then entire path 
            mid_x = segmt.X(s/2)
            mid_y = segmt.Y(s/2)
            mid_x2 = segmt.X(0.75*s)
            mid_y2 = segmt.Y(0.75*s)
            RecoveryPathWPs.append([mid_y,mid_x])                   # 2nd point
            RecoveryPathWPs.append([mid_y2,mid_x2])                 # 3rd point

        elif j == 1:
            RecoveryPathWPs.append([points[1][1],points[0][1]])     # 4th point - 1st point on the middle clothoid
            mid_x = segmt.X(s/2)
            mid_y = segmt.Y(s/2)
            mid_x2 = segmt.X(0.75*s)
            mid_y2 = segmt.Y(0.75*s)
            RecoveryPathWPs.append([mid_y,mid_x])                   # 5th point 
            RecoveryPathWPs.append([mid_y2,mid_x2])                 # 6th point

        elif j == 2:
            RecoveryPathWPs.append([points[1][1],points[0][1]])     # 7th point - 1st point on the last clothoid
            mid_x = segmt.X(s/2)        
            mid_y = segmt.Y(s/2)
            RecoveryPathWPs.append([mid_y,mid_x])                       # 8th point
            RecoveryPathWPs.append([points[1][endPt],points[0][endPt]]) # 9th point - last point of the last clothoid - should be original interploated point

        j+=1
    
    return RecoveryPathWPs, pltPts_List

def check_changePath(ActivePath_List, ReferencePath_List, TargetWPList, uavObj, Index2Watch4, targetIndex, lap):
    
    changePath = False
    if uavObj['dubins'].currentWPIndex > Index2Watch4-1:
        if lap == True:
            ActivePath_List = ReferencePath_List[:]
            TargetWPList = []
            for i in range(0, len(ReferencePath_List)):
                TargetWPList.append(ActivePath_List[i]['pt'])
            uavObj['dubins'].setWaypoints(TargetWPList, newradius=wptRad) 

            uavObj['dubins'].currentWPIndex = targetIndex
            activeWP = ActivePath_List[uavObj['dubins'].currentWPIndex]['pt']

            print('Completed A* path and a lap - no recovery path required')  
            last_targetIndex = targetIndex
            changePath = True

        else: 
            uavObj['dubins'].currentWPIndex = targetIndex + numbOfAstarPts 
            # activeWP = ActivePath_List[uavObj['dubins'].currentWPIndex]['pt']
            print('Completed A* path')  
            changePath = True

    return changePath, ActivePath_List, TargetWPList


###########################################
#### END NEW FUNCTIONS ####################
###########################################

'RaceTack points taken from Pruitt Field using Goolgle maps'
PruitTrack =[
[39.3264051914717, -82.1101289994580], [39.3263499334731, -82.1103352244684], [39.3261989661035, -82.1104861915330],
[39.3259927415369, -82.1105414491151], [39.3249489949075, -82.1105414491151],[39.3247433390045, -82.1104865192473],
[39.3245921315912, -82.1103357926091], [39.3245365454954, -82.1101296557923], [39.3245914749791, -82.1099233430367],
[39.3247422017190, -82.1097721357249], [39.3249489949075, -82.1097165492456], [39.3259927415369, -82.1097165492456],
[39.3261989665845, -82.1097718071053], [39.3263499337508, -82.1099227743733]]

PruitTrack_reverse = [
[39.3261989661035, -82.1104861915330], [39.3263499334731, -82.1103352244684], [39.3264051914717, -82.1101289994580], 
[39.3263499337508, -82.1099227743733], [39.3261989665845, -82.1097718071053], [39.3259927415369, -82.1097165492456], 
[39.3249489949075, -82.1097165492456], [39.3247422017190, -82.1097721357249], [39.3245914749791, -82.1099233430367], 
[39.3245365454954, -82.1101296557923], [39.3245921315912, -82.1103357926091], [39.3247433390045, -82.1104865192473], 
[39.3249489949075, -82.1105414491151], [39.3259927415369, -82.1105414491151]]

'''
Generate 2 lists of dictionary entries 
Each dictionary entry contains the racetrack waypoint
ActivePath_List - The active list the CAS UAV will follow
ReferencePath_List - Used to reset the active list
'''
Waypoint_dict = {}      # Temporary dictionary entry
ActivePath_List = []    
ReferencePath_List = [] 
for i in range(0, len(PruitTrack)):
    Waypoint_dict['pt'] = PruitTrack[i]    
    Waypoint_dict['Belongs to'] = 'Reference Path'  # Identifies which path this point belongs to
    if i == 0:
        Waypoint_dict['Is Go-To'] = True        # This is the first point in the track
    else:
        Waypoint_dict['Is Go-To'] = False

    ActivePath_List.append(Waypoint_dict.copy())
    ReferencePath_List.append(Waypoint_dict.copy())

dt = 0.2 # 0.1
wptRad = 0.0001 # distance threshold to satisfy each waypoint

'''
Create list of dictionary entries
Each entry contains a dubins vehicle 
'''

uavlist = []    # holds dictionary items for specific UAVs
uav1 = {}
uavlist.append(uav1)
uav2 = {}
uavlist.append(uav2)

' CAS UAV '
v = 0.00005/2.75
thetaRef = np.deg2rad(270)
uavlist[0]['dubins'] = dubinsUAV(position=[39.3264051914717, -82.1101289994580], velocity=v,          
                                    heading=thetaRef, dt=dt)
uavlist[0]['dubins'].setWaypoints(newwps=PruitTrack, newradius = wptRad )
uavlist[0]['dubins'].currentWPIndex = 0                                    
uavlist[0]['ID'] = 1
uavlist[0]['IsAvoidanceUAV'] = True
lap_Counter = uavlist[0]['dubins'].lapCounter 

# from Ch 5 in the Pilot's Handbook of Aernautical Knowledge 
uavTurnRadius = ((uavlist[0]['dubins'].v * 360/(np.degrees(uavlist[0]['dubins'].turnrate)))/np.pi)/2

# Using 50% of maximum turn radius - conservative value for Clothoid path selection
halfTurnRadius = uavTurnRadius*2

' Non-cooperative UAV '
v1 = 0.00005/2              # is faster than the CAS UAV
thetaRef = np.deg2rad(135)
uavlist[1]['dubins'] = dubinsUAV(position=[39.3261989665845, -82.1097718071053], velocity=v1,         
                                    heading=thetaRef, dt=dt)
uavlist[1]['dubins'].setWaypoints(newwps=PruitTrack_reverse, newradius = wptRad )
uavlist[1]['dubins'].currentWPIndex = 4
uavlist[1]['ID'] = 4
uavlist[1]['IsAvoidanceUAV'] = False

uavlist[0] = syncAVSfromDubins(uavlist[0])
uavlist[1] = syncAVSfromDubins(uavlist[1])


'''
Simulation Variables
'''
dt = 0.2 # 0.1
area_length = 0.0002
TargetWPList = None
last_targetIndex = 0

hasAstarPlan = False
hasRecoveryPlan = False
ClearedCollision = False

Show_AstarPlan = False
Show_RecoveryPlan = False
TrackUAV = False

fig, ax = plt.subplots()
step = 0
while step < 2500:

    ''' Identify UAVs using collision avoidence '''
    mainUAV = finduavbyID(uavlist, 1)                           # ID to Watch for
    uavh_others_all, uavh_others = findotheruavs(uavlist, 1)    # ID not to watch for

    activeWP = mainUAV['dubins'].waypoints[mainUAV['dubins'].currentWPIndex]

    ''' 
    Extract waypoint coordinates from the ReferencePath_list  
    Easier to use a list of waypoints
    '''
    if TargetWPList == None:       
        TargetWPList = []
        for i in range(0, len(ReferencePath_List)):
            TargetWPList.append(ActivePath_List[i]['pt'])

        mainUAV['dubins'].setWaypoints(TargetWPList, newradius=wptRad)
        mainUAV['dubins'].currentWPIndex = 0


    # Need Function to detect potential collision

    # If collision detected - Function to determine A* goal point
    if not hasAstarPlan:
        lookAheadDist = 2.5
        astarGoalPoint_list = findRefPathIndex(ActivePath_List)
        astarGoalPt, targetIndex, pointList, lap = findAstarGoal(astarGoalPoint_list, ReferencePath_List, mainUAV, area_length, 
                                                                    lookAheadDist)

        # A* replan - already a function
        astarReplan, astarwpts, KOZpoints, full_path, uavID, AstarFailure = mainUAV['uavobj'].avoid(uavh_others, area_length=area_length, static_koz=[], TargetPathWP=astarGoalPt, useAstarGoal=True, simStep=step)



        # some kind of waypoint list update function?
            # needs an insert point - where to put the new waypoints
            # new return to path point - both the index to switch to another point and that points index
        if (astarReplan and not hasAstarPlan):
            hasAstarPlan = True
            Show_AstarPlan = True

            indexRecall = mainUAV['dubins'].currentWPIndex-1 # used to inform Astar goal and Revocery path logic where the UAV left the reference path

            # Current positions is saved to be used later as a possible recovery insetions point
            saveCurrentPOS = [mainUAV['dubins'].x, mainUAV['dubins'].y ]

            ''' Insert current vehicle postion and astar goal into A* replan '''
            astarwpts[0][0] = mainUAV['uavobj'].position[0]
            astarwpts[0][1] = mainUAV['uavobj'].position[1]
            astarwpts = np.append(astarwpts, np.array([[astarGoalPt[0], astarGoalPt[1]]]), axis=0)
            numbOfAstarPts= len(astarwpts) 

            # Waypoint index to watch for - signals return to the reference path
            Astar_Index2Watch4 = mainUAV['dubins'].currentWPIndex + numbOfAstarPts
            # Update flight plan with A* replan waypoints - insert waypoints at current waypoint index
            UpdateWPList(ActivePath_List, wypts2add=astarwpts, numbOfPts=numbOfAstarPts, List_belongsTo='Astar', insertIndex=mainUAV['dubins'].currentWPIndex)

            ''' Update the List of Waypoints '''
            TargetWPList = []
            for i in range(0, len(ActivePath_List)):
                TargetWPList.append(ActivePath_List[i]['pt'])

            mainUAV['dubins'].setWaypoints(TargetWPList, newradius=wptRad/2)
            activeWP = mainUAV['dubins'].waypoints[mainUAV['dubins'].currentWPIndex]

            print(TC.OKBLUE + 'Insert ' + str(numbOfAstarPts) + ' Astar Points at wpt index ' + str(mainUAV['dubins'].currentWPIndex) + TC.ENDC)

            TrackUAV = True
            last_dist2UAV = distance([mainUAV['dubins'].x, mainUAV['dubins'].y] , [uavlist[1]['dubins'].x, uavlist[1]['dubins'].y])
    # Start Distance Tracking between CAS and NC UAVs
    # Currently only useful for scenarios with only 1 NC UAV
    distThreshold = area_length*1.11
    if TrackUAV == True:
        dist2UAV = distance([mainUAV['dubins'].x, mainUAV['dubins'].y] , [uavlist[1]['dubins'].x, uavlist[1]['dubins'].y])
        if dist2UAV > last_dist2UAV and dist2UAV > distThreshold:
            TrackUAV = False
            ClearedCollision = True
        else:
            last_dist2UAV = dist2UAV

    if hasAstarPlan and ClearedCollision == True:
        # Function for Generateing clothoid paths
        RecoveryPoints, referencePathPoint, InsertionPoint_list = getPotentialRecoveryPoints(ActivePath_List, saveCurrentPOS, indexRecall, numbOfPts=5)
        
        # Generate clothoid paths and corresponding reference path indices
        clothoid_List, index_List = getRecoveryPaths(RecoveryPoints, referencePathPoint, mainUAV['dubins'].x, mainUAV['dubins'].y,)
        # Select clothoid with shortest path that does not violate turn radius and report the reference path index 
        chosenClothoid, selectIndex = selectRecoveryPath(clothoid_List, index_List, 10)
        # Sample waypoints from chosen clothoid to be used as the recovery path
        RecoveryPathWPs, pltPts_List = getRecoveryPathWPs(chosenClothoid, numbOfwpts=10)
        # Waypoint update funtion
        numbOfRecoveryPts = len(RecoveryPathWPs)
        # waypoint index to watch for - Waypoint indext that identifies when to switch back to reference path
        Recover_Index2Watch4 = Astar_Index2Watch4 + numbOfRecoveryPts

        # Waypoint Index that identifies index to change to after completing recovery path
        Recovery_Return_Index = InsertionPoint_list[selectIndex]['Index'] + numbOfRecoveryPts 
        # Update flight plan with Recovery waypoints - insert waypoints after the A* waypoints
        UpdateWPList(ActivePath_List, wypts2add=RecoveryPathWPs, numbOfPts=numbOfRecoveryPts, List_belongsTo='Recovery', insertIndex=Astar_Index2Watch4)
        
        ''' Update the List of Waypoints '''
        TargetWPList = []
        for i in range(0, len(ActivePath_List)):
            TargetWPList.append(ActivePath_List[i]['pt'])

        mainUAV['dubins'].setWaypoints(TargetWPList, newradius=wptRad/2)
        mainUAV['dubins'].currentWPIndex = Astar_Index2Watch4
        activeWP = mainUAV['dubins'].waypoints[mainUAV['dubins'].currentWPIndex]

        hasRecoveryPlan = True
        ClearedCollision = False
        Show_RecoveryPlan = True
        Show_AstarPlan = False

    # clearing waypoints after completing a lap

    Show_RecoverySnapShot = False

    if hasRecoveryPlan and Show_RecoverySnapShot == True:
        # ==== Plot Snapshot of all possible Clothoid paths ====
        for i in range(0, len(clothoid_List)):
            clothoid = clothoid_List[i]
            for segmnt in clothoid:
                plt.plot(*segmnt.SampleXY(500))

        plt.scatter([pt[1] for pt in RecoveryPoints], [pt[0] for pt in RecoveryPoints])
        plt.plot(mainUAV['dubins'].ys, mainUAV['dubins'].xs, c='r', marker='o' )
        plt.plot(uavh_others_all[0]['dubins'].y, uavh_others_all[0]['dubins'].x, c='y', marker='o' )
        NCcone = uavh_others_all[0]['uavobj'].possibleFlightAreaStatic(area_length=area_length*1.0)
        plotNCcone, = plt.plot([pt[1] for pt in NCcone], [pt[0] for pt in NCcone], "-r")

        plot_AstarPlan = plt.plot([pt[1] for pt in astarwpts.tolist()], [pt[0] for pt in astarwpts.tolist()], c = 'k', marker='*', markersize=8)
        plt.plot([pt[1] for pt in PruitTrack], [pt[0] for pt in PruitTrack], c='b', marker='.', markersize=8)

        plt.plot( activeWP[1], activeWP[0], c='k', marker='X', markersize = 5 )
        # plot keep out zones from UAVHeading.avoid() function
        if astarReplan:
            plotCASkoz, = plt.plot([pt[1] for pt in KOZpoints[0]], [pt[0] for pt in KOZpoints[0]], '--m')
            plotNCkoz, = plt.plot([pt[1] for pt in KOZpoints[1]], [pt[0] for pt in KOZpoints[1]], '--m')
        plt.axis('equal')
        plt.grid(True)
        # plt.ylim((mainUAV['dubins'].x - 0.001, mainUAV['dubins'].x + 0.001))
        # plt.xlim((mainUAV['dubins'].y - 0.001, mainUAV['dubins'].y + 0.001))
        plt.xlim((-82.11100, -82.1090))
        plt.ylim((39.32435, 39.32657))

        fig.set_size_inches((12, 10)) 
        plt.show()
        # plt.pause(1)
        
        # wd = os.getcwd()
        # path=(wd + '/RaceTrack_RecoveryPaths')
        # RecoveryPaths = 'RecoveryPaths%03d.png' % step
        # RecoveryPaths = os.path.join(path,RecoveryPaths)
        # plt.savefig(RecoveryPaths)
        # plt.clf()

        plt.scatter([pt[1] for pt in pltPts_List], [pt[0] for pt in pltPts_List])
        plt.scatter([pt[1] for pt in RecoveryPathWPs], [pt[0] for pt in RecoveryPathWPs])
        plt.plot(mainUAV['dubins'].ys, mainUAV['dubins'].xs, c='r', marker='o' )
        plt.plot(uavh_others_all[0]['dubins'].y, uavh_others_all[0]['dubins'].x, c='y', marker='o' )
        NCcone = uavh_others_all[0]['uavobj'].possibleFlightAreaStatic(area_length=area_length*1.0)
        plotNCcone, = plt.plot([pt[1] for pt in NCcone], [pt[0] for pt in NCcone], "-r")

        plot_AstarPlan = plt.plot([pt[1] for pt in astarwpts.tolist()], [pt[0] for pt in astarwpts.tolist()], c = 'k', marker='*', markersize=8)
        plt.plot([pt[1] for pt in PruitTrack], [pt[0] for pt in PruitTrack], c='b', marker='.', markersize=8)

        plt.plot( activeWP[1], activeWP[0], c='k', marker='X', markersize = 5 )
        # plot keep out zones from UAVHeading.avoid() function
        if astarReplan:
            plotCASkoz, = plt.plot([pt[1] for pt in KOZpoints[0]], [pt[0] for pt in KOZpoints[0]], '--m')
            plotNCkoz, = plt.plot([pt[1] for pt in KOZpoints[1]], [pt[0] for pt in KOZpoints[1]], '--m')
        plt.axis('equal')
        plt.grid(True)
        # plt.ylim((mainUAV['dubins'].x - 0.001, mainUAV['dubins'].x + 0.001))
        # plt.xlim((mainUAV['dubins'].y - 0.001, mainUAV['dubins'].y + 0.001))
        plt.xlim((-82.11100, -82.1090))
        plt.ylim((39.32435, 39.32657))

        fig.set_size_inches((12, 10)) 
        plt.show()













    # function to check when following Ref/Avoid/Recovery path and swap between them appropriately
    if hasAstarPlan == True and hasRecoveryPlan == False:
        if step == 1581:
            checking = 1

        changePath, ActivePath_List, TargetWPList = check_changePath(ActivePath_List, ReferencePath_List, TargetWPList, mainUAV, Astar_Index2Watch4, targetIndex, lap)        
        if changePath == True:
            hasAstarPlan = False
            Show_AstarPlan = False
            print('Completed A* path')

    elif hasAstarPlan == True or hasRecoveryPlan == True:
        changePath, ActivePath_List, TargetWPList = check_changePath(ActivePath_List, ReferencePath_List, TargetWPList, mainUAV, Recover_Index2Watch4, Recovery_Return_Index, lap)        
        if changePath == True:
            hasAstarPlan = False
            hasRecoveryPlan = False
            Show_RecoveryPlan = False
            ClearedCollision = False
            mainUAV['dubins'].currentWPIndex = Recovery_Return_Index  
            activeWP = mainUAV['dubins'].waypoints[mainUAV['dubins'].currentWPIndex]
            print('Completed Recovery path')

    # Update UAV positions - already functions
    '''
    ================================
    === Update vehicle positions ===
    ================================
    '''
    for uav in uavlist:     
        pts = uav['uavobj'].possibleFlightAreaStatic(area_length=area_length*1.0)
        if uav['ID'] == 1:
            # dist2ncUAVs = uav['dubins'].getOtherUAVStates(uavh_others_all, uavID)

            plotCASpos, = plt.plot(uav['dubins'].ys, uav['dubins'].xs, 'o', c='r')
            plotMainUAV =  plt.plot(uav['dubins'].y, uav['dubins'].x, 'o', c='k')

            color = '-g'
            if(astarReplan):
                color = '-y'
                replan = False
            plotCAScone, = plt.plot([pt[1] for pt in pts], [pt[0] for pt in pts], color)
            plotCurrentWypt = plt.plot(uav['dubins'].waypoints[uav['dubins'].currentWPIndex][1], uav['dubins'].waypoints[uav['dubins'].currentWPIndex][0], c='black', marker='X')
            # plotCASkoz, = plt.plot([pt[1] for pt in avoid[0]], [pt[0] for pt in avoid[0]], '--m')

            # cross track error measurement currently broken...   
            # crossError, m, b = crossTrackError(PruitTrack, [uav['dubins'].x, uav['dubins'].y]) 

            uav['dubins'].simulateWPDubins(UseCarrotChase=False, delta=0.01)
            carrot = uav['dubins'].CarrotChaseWP(delta=0.01)
            CASuavPos = uav['dubins'].position

            # clearWPList(ActivePath_List, ReferencePath_List, lap_Counter, uavClass=uav )
            if uav['dubins'].lapCounter > lap_Counter:
                # for i in range(0, len(ActivePath_List)):
                #     if ActivePath_List[i]['Is Go-To'] == True:
                #         uav['dubins'].currentWPIndex = i
                #         break

                ActivePath_List = ReferencePath_List[:]
                TargetWPList = []
                for i in range(0, len(ReferencePath_List)):
                    TargetWPList.append(ActivePath_List[i]['pt'])

                uav['dubins'].currentWPIndex = 0
                uav['dubins'].setWaypoints(TargetWPList, newradius=wptRad)

            lap_Counter= uav['dubins'].lapCounter 

        if uav['IsAvoidanceUAV'] == False:
 
            plotNCpos, = plt.plot(uav['uavobj'].position[1], uav['uavobj'].position[0], 'o', c='orange')
            # plotNCpos, = plt.plot(uav['dubins'].ys, uav['dubins'].xs, 'o', c='orange')
            NCactiveWPt = uav['dubins'].getActiveWaypoint()
            plt.plot( NCactiveWPt[1], NCactiveWPt[0], c='k', marker='v', markersize = 8 )

            plotNCcone, = plt.plot([pt[1] for pt in pts], [pt[0] for pt in pts], "-r")
            uav['dubins'].simulateWPDubins(UseCarrotChase=False, delta=0.01)
            uav['dubins'].update_pos_simple()
            NCuavPos = uav['dubins'].position


        uav = syncAVSfromDubins(uav)

        print('\nStep: ' + str(step) + '\tUAV ID: ' + str(uav['ID']) + '\tCurrent wpt: ' + str(uav['dubins'].currentWPIndex) + 
              '\tUAV Heading (deg): ' + str(round(uav['dubins'].heading,2)) + 
              ' (' + str(round(np.degrees(uav['dubins'].heading),2)) + ')'   )

    '''======================
       ===== Plotting =======
       ====================== '''

    if Show_AstarPlan:
        plot_AstarPlan = plt.plot([pt[1] for pt in astarwpts.tolist()], [pt[0] for pt in astarwpts.tolist()], c = 'k', marker='*', markersize=8)
        # for pts in astarwpts.tolist():
        #         wptCircle = plt.Circle((pts[1], pts[0]), wptRad, color='green', alpha=0.2)
        #         plt.gca().add_artist(wptCircle)
        #         plt.scatter(pts[1],pts[0])

    if Show_RecoveryPlan :
        plot_RecoveryPlan = plt.plot([pt[1] for pt in RecoveryPathWPs], [pt[0] for pt in RecoveryPathWPs], c = 'g', marker='o', markersize=8)
        # for pts in RecoveryPath_List:
        #         wptCircle = plt.Circle((pts[1], pts[0]), wptRad, color='green', alpha=0.2)
        #         plt.gca().add_artist(wptCircle)
        #         plt.scatter(pts[1],pts[0])

    if Show_RecoveryPlan :
        plot_RecoveryPlan_Full = plt.plot([pt[1] for pt in pltPts_List], [pt[0] for pt in pltPts_List], c = 'b', marker='.', markersize=2)
        # for pts in RecoveryPath_List:
        #         wptCircle = plt.Circle((pts[1], pts[0]), wptRad, color='green', alpha=0.2)
        #         plt.gca().add_artist(wptCircle)
        #         plt.scatter(pts[1],pts[0])


    plt.plot([pt[1] for pt in pointList], [pt[0] for pt in pointList], c='k', marker='.', markersize = 8)
    plt.plot(astarGoalPt[1], astarGoalPt[0], c='r', marker='*', markersize = 12)


    plt.plot([pt[1] for pt in PruitTrack], [pt[0] for pt in PruitTrack], c='b', marker='.', markersize=8)
    plt.plot([pt[1] for pt in PruitTrack], [pt[0] for pt in PruitTrack], c='b', marker='.', markersize=8)

    text = ("Simulation Step: " + str(step) + "\nCAS Current wypt (Total): " + str(uavlist[0]['dubins'].currentWPIndex) + '(' + str(len(ActivePath_List)) + ')') 

    if hasAstarPlan == False and hasRecoveryPlan == False:
        text1 = ("Currently Following: Ref Path")
    elif hasAstarPlan == True and hasRecoveryPlan == False:
        text1 = ("Currently Following: A* path")    
    elif hasAstarPlan == False and hasRecoveryPlan == True:
        text1 = ("Currently Following: Recovery Path")


    plt.text(0.1, 0.05, text, transform=ax.transAxes)
    plt.text(0.1, 0.9, text1, transform=ax.transAxes)

    ax.axis('equal')
    # plt.ylim((39.32450, 39.32657))
    plt.xlim((-82.11100, -82.1090))

    plt.ylim((39.32435, 39.32657))
    # plt.xlim((-82.1117, -82.1095))


    # plt.ylim((39.788, 39.802))
    # plt.xlim((-86.241, -86.227))

    fig.set_size_inches((12, 10))  
    plt.grid(True)
    plt.pause(0.01)

    plt.clf()
    step+=1



