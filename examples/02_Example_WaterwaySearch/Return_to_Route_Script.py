# Jeremy Browne October 2020
# Script to test CAS with waypoint based return-to-route 
# Procedure:
# 1) Follow refference path until collision event with 
#    a non-coopterative UAV
# 2) Avoid using A* generated collision-free replan path
# 3) Continue to avoid until the offending UAV has passed
# 4) Return to reference path using a clothoid path
#

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
hasAstarPlan = False
hasRecoveryPlan = False
onlyOnce = False

usetargetPath = True

hadPlan = False
hasPlan = False


'''
Ploting variables
'''
Show_AstarPlan = False
Show_RecoveryPlan = False

Show_AstarSnapShot = True
Show_RecoverySnapShot = False
Show_SelectedRecoveryPathSnapShot = False
'''
Pickle and save for later Variables
'''
# Movie variables
savePlots = []  
savePlots1 = []
savePlots2 = []
savePlots3 = []
savePlots4 = []
savePlots5 = []

fig, ax = plt.subplots()
step = 0
while step < 2000:    
    ''' Identify UAVs using collision avoidence '''
    mainUAV = finduavbyID(uavlist, 1)                           # ID to Watch for
    uavh_others_all, uavh_others = findotheruavs(uavlist, 1)    # ID not to watch for

    activeWP = uavlist[0]['dubins'].waypoints[uavlist[0]['dubins'].currentWPIndex]

    ''' Extract the waypoint infrmation from the ReferencePath_list  
        Easier to use a list of waypoints
    '''
    if TargetWPList == None:
        mainUAV['dubins'].getDist2otherUAVs(uavh_others_all)
        
        TargetWPList = []
        for i in range(0, len(ReferencePath_List)):
            TargetWPList.append(ActivePath_List[i]['pt'])

        uavlist[0]['dubins'].setWaypoints(TargetWPList, newradius=wptRad)
        uavlist[0]['dubins'].currentWPIndex = 0

    '''
    ======================================================================
    ==== Generate Collision Free A* replan if pos. collision detected ====
    ======================================================================
    '''

    if len(uavlist) > 1:
        if not hasAstarPlan:
            ''' 
            === Astar goal point selection === 
            Make a list of dictionary entries that contains the Reference/original or Recovery path waypoints
            and their corresponding index in the Active waypoint list
            '''
            astarGoalPoint_list = []
            astarGoalPoint_dict = {}                    # temporary dictionary entry
            for i in range(0, len(ActivePath_List)):
                if ActivePath_List[i]['Belongs to'] == 'Reference Path' or ActivePath_List[i]['Belongs to'] == 'Recovery' :
                    astarGoalPoint_dict['Index'] = i
                    astarGoalPoint_dict['pt'] = ActivePath_List[i]['pt']
                    astarGoalPoint_list.append(astarGoalPoint_dict.copy())

            'Determine which waypoint in astarGoalPoint_list the CAS UAV is heading towards'
            for i in range(0, len(astarGoalPoint_list)):
                if mainUAV['dubins'].currentWPIndex == astarGoalPoint_list[i]['Index']:
                    currentIndex = i

            # Add in Reference path incase the UAV is about to make a lap
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
                    x1 = mainUAV['dubins'].x
                    y1 = mainUAV['dubins'].y 

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
                numbOfPts = 50
                linX = np.linspace(x1, x2, numbOfPts, endpoint=False )
                linY = np.linspace(y1, y2, numbOfPts, endpoint=False )

                ''' 
                Look for an interpolated point some distance forward along the reference path as the astarGoal point
                '''
                lookAheadDist = 3
                for pt in range(0, numbOfPts-1):
                    d = distance([linX[pt], linY[pt]], [linX[pt+1], linY[pt+1]]) # Calculate distance between interpolated points
                    distTotal += d  
                    pointList.append([linX[pt], linY[pt]])
                    if distTotal >= area_length*lookAheadDist:
                        astarGoalPt = [linX[pt], linY[pt]]
                        targetIndex = astarGoalPoint_list[index]['Index']
                        break
                if distTotal >= area_length*lookAheadDist:
                    break

            plt.plot([pt[1] for pt in pointList], [pt[0] for pt in pointList], c='k', marker='.', markersize = 8)
            plt.plot(astarGoalPt[1], astarGoalPt[0], c='r', marker='*', markersize = 12)


            '''
            ================================
            ==== Avoid Function is here ====
            ================================
            '''
            # hasAstarPlan = False
            astarReplan, astarwpts, KOZpoints, full_path, uavID, AstarFailure = mainUAV['uavobj'].avoid(uavh_others, area_length=area_length, static_koz=[], TargetPathWP=astarGoalPt, useAstarGoal=True, simStep=step)

            ''' Use uavID to determine with NC UAV is being Avoided '''
            if len(uavID) > 0:
                for i in range(0, len(uavID)):    
                    uavID[i] = uavh_others_all[uavID[i]]['ID']
                    print('Potential Collision with UAV ' + str(uavID[i]))

            if (astarReplan and not hasAstarPlan):
                hasAstarPlan = True
                Show_AstarPlan = True
                # indexRecall - used to reset the currentWPIndex after A* and/or Recovery path is complete
                indexRecall = mainUAV['dubins'].currentWPIndex 

                # Current positions is saved to be used later as a possible recovery insetions point
                saveCurrentPOS = [mainUAV['dubins'].x, mainUAV['dubins'].y ]

                ''' Insert current vehicle postion into A* wplist '''
                [0][0] = mainUAV['uavobj'].position[0]
                astarwpts[0][1] = mainUAV['uavobj'].position[1]
                astarwpts = np.append(astarwpts, np.array([[astarGoalPt[0], astarGoalPt[1]]]), axis=0)
                numbOfAstarPts= len(astarwpts) 

                Astar_Return_Index = mainUAV['dubins'].currentWPIndex + numbOfAstarPts
                
                ''' Update Active path with A* replan'''
                temp = []
                for i in range(0, len(astarwpts)):
                    Waypoint_dict['pt'] = astarwpts[i]    
                    Waypoint_dict['Belongs to'] = 'Astar'
                    Waypoint_dict['A* Return Index'] = mainUAV['dubins'].currentWPIndex + numbOfAstarPts
                    Waypoint_dict['A* Return Pt'] = 'Astar'
                    Waypoint_dict['Recover Return Index']  = None
                    Waypoint_dict['Recover Return Pt']  = None
                    Waypoint_dict['Is Go-To'] = False

                    temp.append(Waypoint_dict.copy())

                ''' Insert A* points (astarwpts) into Active path '''
                ActivePath_List[mainUAV['dubins'].currentWPIndex : mainUAV['dubins'].currentWPIndex] = temp

                # # If the astar goal point is looking into the next lap
                # # add additional reference waypoints to the active list
                # if targetIndex < mainUAV['dubins'].currentWPIndex:
                #     Waypoint_dict = {}
                #     for j in range(0, len(ReferencePath_List)):
                #         Waypoint_dict['pt'] = ReferencePath_List[j]['pt']
                #         Waypoint_dict['Belongs to'] = 'Reference Path'

                #         ActivePath_List.append(Waypoint_dict.copy())


                ''' Update the List of Waypoints '''
                TargetWPList = []
                for i in range(0, len(ActivePath_List)):
                    TargetWPList.append(ActivePath_List[i]['pt'])

                uavlist[0]['dubins'].setWaypoints(TargetWPList, newradius=wptRad)
                activeWP = uavlist[0]['dubins'].waypoints[uavlist[0]['dubins'].currentWPIndex]

                print(TC.OKBLUE + 'Insert ' + str(numbOfAstarPts) + ' Astar Points at wpt index ' + str(indexRecall) + TC.ENDC)

                if Show_AstarSnapShot == True:
                    # ===== Plot snapshot of Astar Path and save frame ==============
                    plt.plot(mainUAV['dubins'].ys, mainUAV['dubins'].xs, c='r', marker='o' )
                    plt.plot(uavh_others_all[0]['dubins'].y, uavh_others_all[0]['dubins'].x, c='y', marker='o' )
                    plot_AstarPlan = plt.plot([pt[1] for pt in astarwpts.tolist()], [pt[0] for pt in astarwpts.tolist()], c = 'k', marker='*', markersize=8)
                    plt.plot([pt[1] for pt in PruitTrack], [pt[0] for pt in PruitTrack], c='b', marker='.', markersize=8)

                    plt.plot( activeWP[1], activeWP[0], c='k', marker='X', markersize = 5 )
                    plt.plot( astarGoalPt[1], astarGoalPt[0], c='k', marker='^', markersize = 6 )

                    # plot keep out zones from UAVHeading.avoid() function
                    plotCASkoz, = plt.plot([pt[1] for pt in KOZpoints[0]], [pt[0] for pt in KOZpoints[0]], '--m')
                    plotNCkoz, = plt.plot([pt[1] for pt in KOZpoints[1]], [pt[0] for pt in KOZpoints[1]], '--m')
                    plt.axis('equal')
                    plt.grid(True)
                    plt.ylim((mainUAV['dubins'].x - 0.001, mainUAV['dubins'].x + 0.001))
                    plt.xlim((mainUAV['dubins'].y - 0.001, mainUAV['dubins'].y + 0.001))
                    fig.set_size_inches((12, 10)) 
                    # plt.show()

                    wd = os.getcwd()
                    path=(wd + '/RaceTrack_AstarPath')
                    AstarPath = 'AstarPath%03d.png' % step
                    AstarPaths = os.path.join(path, AstarPath)
                    plt.savefig(AstarPaths)
                    plt.clf()
        else:
            print("Not re-planning" )
    else:
        print('\tOnly one UAV')


    '''
    =========================================
    ==== Generate Clothoid Recovery Path ====
    =========================================
    '''
    checkDistance = 9999999     # used to evaluate shortest clothoid path using interpolated target waypoints
    if hasAstarPlan and mainUAV['dubins'].trackUAV[0]['clearedUAV']:
        hasAstarPlan = False
        '''
        List of dictionary entries
        Each entry contains the extracted reference waypoints from the active path list
        and the corresponding index in the active waypoint list
        The corresponding index defines the waypoint to follow when the CAS UAV switches
        from the recovery path back to the reference path
        '''
        InsertionPoint_list = []
        InsetionPoint_dict = {}
        for i in range(0, len(ActivePath_List)):
            if ActivePath_List[i]['Belongs to'] == 'Reference Path':
                InsetionPoint_dict['Index'] = i
                InsetionPoint_dict['pt'] = ActivePath_List[i]['pt']
                InsertionPoint_list.append(InsetionPoint_dict.copy())

        '''
        Convert uav North East Down angle convention to cartesion for clothoid heading:
        Needed an axis flip to find the angle between UAV and active waypoint- x's in the numerator and y's in the denominator, then add 180 deg
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
        Choose the closest interpolated point on the reference path as injection point
        Chosen point is interpolated between original waypoint index sets
        Chosen point provdes the shortes path that does not violate turn radius
        '''
        for index in range(0, len(InsertionPoint_list)-1):  
            ' Find potential insetion points along the reference path ' 
            if index == indexRecall-1:
                # This is the point that the CAS UAV left the reference path to follow A* replan
                x1 = saveCurrentPOS[0]
                y1 = saveCurrentPOS[1]
            else:
                # A point in the reference path
                x1 = InsertionPoint_list[index]['pt'][0]
                y1 = InsertionPoint_list[index]['pt'][1]

            # Next point on the reference path
            x2 = InsertionPoint_list[index+1]['pt'][0]
            y2 = InsertionPoint_list[index+1]['pt'][1]

            '''
            Use a linear space to interpolate points between each above reference path waypoint [x1,y1] and [x2, y2]
            Note - interpolation does not include the endpoint - it is included in the next index set
            '''
            numbOfPts = 5       # Number of interpolated points
            linX = np.linspace(x1, x2, numbOfPts, endpoint=False )
            linY = np.linspace(y1, y2, numbOfPts, endpoint=False )
            print('Interpolating ' + str(numbOfPts) + ' points between index ' + str(index) + ' and ' + str(index+1))
            
            for pt in range(0, numbOfPts):
                print('\tChecking pt ' + str(pt+1))
                
                ''' 
                Calculate heading between interpolated point and next reference waypoint 
                This is used for the clothoid curvature calculation
                '''
                targetHeading = np.arctan2(linX[pt] - x2, linY[pt] - y2) + np.radians(180) 

                '''
                Clothoid solver will be finding a path between the current CAS UAV position
                and the targetWPT interpolated point in the path 
                '''
                targetWPT = [linX[pt], linY[pt], targetHeading]  

                plt.plot(targetWPT[1], targetWPT[0], 'o', markersize=5)

                if(targetWPT[2] >= np.pi*2):
                    targetWPT[2] -= np.pi*2
                if(targetWPT[2] < 0):
                    targetWPT[2] += np.pi*2

                # Provides clothoid parameters used to generate a path between the UAV and target waypoint on reference path: 3 clothoids per path are needed
                # Stiches the 3 clothoids together
                clothoid_list = pyclothoids.SolveG2(mainUAV['dubins'].y, mainUAV['dubins'].x, uavHeading, 0, targetWPT[1], targetWPT[0], targetWPT[2], 0) 
                
                'List and counter variables for clothoid path selection'
                circle_list = []            # Stores circle fit info for each clothoid segment in a single path: [cx,cy,r]
                ClothoidPath = []           # Stores a specified number of waypoints for each clothoid segment
                recoveryPath = []           # Stored the 1st, middle, and last wpt from the 1st, 2nd, and 3rd clothoid respectively
                j = 0                       # Keeps track of which clothoid segment is being evaluated
                clothoidLength = 0          # keeps track of clothoid path length

                temp = []                   # temporary variable - used for ploting purposes
                completeClothoidPts = []    # stores a the full path of each clothoid generated -  used for plotting purposes

                for i in clothoid_list:
                    # plt.plot(*i.SampleXY(500))
                    pltpts = i.SampleXY(500)
                    for ii in range(0, len(pltpts[0])):
                        temp.append([pltpts[1][ii], pltpts[0][ii]])

                    'Sample points from the clothoid'
                    points = i.SampleXY(numbOfwpts)         # used for circle fitting to calcuate a turn radius for each clothoid path and used for waypoint path following
                    ClothoidPath.append(points)

                    x0, y0, t0, k0, dk, s = i.Parameters    # start x, start y, initial curvature, change in curvature, clothoid segment length
                    clothoidLength += s                     # Sum up the length of each clothoid segment to find the total distance of the full clothoid
                    
                    '''
                    The core set of points from clothoids that define the recovery path:
                    - first point of the 1st clothoid
                    - middle point of the 2nd clothoid and
                    - last point of the 3rd clothoid segment to follow
                    With some additional points
                    '''
                    midPt = int(len(points[0])/2)
                    endPt = len(points[0])-1
                    if j == 0:
                        recoveryPath.append([points[0][1],points[1][1]])     # 1st point on first clothoid and for then entire path 
                        mid_x = i.X(s/2)
                        mid_y = i.Y(s/2)
                        mid_x2 = i.X(0.75*s)
                        mid_y2 = i.Y(0.75*s)
                        recoveryPath.append([mid_x,mid_y])                   # 2nd point
                        recoveryPath.append([mid_x2,mid_y2])                 # 3rd point

                    elif j == 1:
                        recoveryPath.append([points[0][1],points[1][1]])     # 4th point - 1st point on the middle clothoid
                        mid_x = i.X(s/2)
                        mid_y = i.Y(s/2)
                        mid_x2 = i.X(0.75*s)
                        mid_y2 = i.Y(0.75*s)
                        recoveryPath.append([mid_x,mid_y])                   # 5th point 
                        recoveryPath.append([mid_x2,mid_y2])                 # 6th point

                    elif j == 2:
                        recoveryPath.append([points[0][1],points[1][1]])     # 7th point - 1st point on the last clothoid
                        mid_x = i.X(s/2)        
                        mid_y = i.Y(s/2)
                        recoveryPath.append([mid_x,mid_y])                       # 8th point
                        recoveryPath.append([points[0][endPt],points[1][endPt]]) # 9th point - last point of the last clothoid - should be original interploated point
                    
                    '''
                    Fit a circle to clothoid segment  
                    Used to determine the needed Turn Radius of clothoid segment
                    '''
                    # Full clothoid segment pt set
                    x_points = points[0]
                    y_points = points[1]

                    # Middle clothoid has two curvatures --> break into two halves 
                    # and fit a circle to each half. 
                    if j == 1:
                        # First Half - provides circle center coordinates and the circle radius
                        halfX_points1 = x_points[:(int(len(x_points)/2))]   
                        halfY_points1 = y_points[:(int(len(y_points)/2))] 
                        half_xc1,half_yc1,half_r1 = fit_circle_2d(halfX_points1, halfY_points1)

                        # Second Half - provides circle center coordinates and the circle radius
                        halfX_points2 = x_points[(int(len(x_points)/2)):]   
                        halfY_points2 = y_points[(int(len(y_points)/2)):] 
                        half_xc2,half_yc2,half_r2 = fit_circle_2d(halfX_points2, halfY_points2)
                        
                        circle_list.append([half_xc1, half_yc1, half_r1])
                        circle_list.append([half_xc2, half_yc2, half_r2])

                    else:
                        xc,yc,r = fit_circle_2d(x_points, y_points)
                        circle_list.append([xc,yc,r])

                    j+=1

                # plt.show(100) # uncomment if you want to see every clothoid generated by itself
                completeClothoidPts.append(temp)        # store points of entire clothoid path
                clothoid_paths.append([index, clothoidLength, ClothoidPath, recoveryPath, completeClothoidPts, circle_list])   # store info for each clothoid
                Recovery_dict['pathLength'].append(clothoidLength)
                Recovery_dict['turnRadii'].append(circle_list)

                'Determine if path violates UAV Turn Radius'
                print('\tPath Distance: ' + str(round(clothoidLength,3)) + '\tClothoid Radii: ' + str(round(circle_list[0][2],3)) + 
                      '   ' + str(round(circle_list[1][2],3)) + '   ' + str(round(circle_list[2][2],3)) + '   ' + str(round(circle_list[3][2],3)))
            
                checkPassed = False # means that clothoid path satisfies UAV requierments
                for r in range(0, len(circle_list)):                        
                    if circle_list[r][2] < halfTurnRadius:
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
                    Recovery_dict['chosenPath'] = recoveryPath
                    Recovery_dict['cirlces'] = circle_list
                    numbOfRecoveryPts = len(recoveryPath)
                    Return2Ref_Index = InsertionPoint_list[index+1]['Index'] + numbOfRecoveryPts

                    #Recovery_dict['chosenIndex'] = selectPath + numbOfRecoveryPts # + numbOfAstarPts
                    Recovery_dict['chosenIndex'] =  Astar_Return_Index
                    Recovery_dict['returnIndex'] = Astar_Return_Index + numbOfRecoveryPts
                    print(TC.OKGREEN + '\t\t\tSelected index pt: ' + str(selectPath) + ' which is now at index ' + str(Recovery_dict['chosenIndex']) + TC.ENDC)

                elif checkPassed == True and clothoidLength > checkDistance:
                    print(TC.WARNING + '\t\t\tPath Too Long' + TC.ENDC)

            # for clothoid in clothoid_paths:
            #     plt.plot([pt[1] for pt in clothoid[4][0]], [pt[0] for pt in clothoid[4][0]])


        if Show_RecoverySnapShot == True:
            # ==== Plot Snapshot of all possible Clothoid paths ====
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
            plt.ylim((mainUAV['dubins'].x - 0.001, mainUAV['dubins'].x + 0.001))
            plt.xlim((mainUAV['dubins'].y - 0.001, mainUAV['dubins'].y + 0.001))
            # plt.show(100)
            fig.set_size_inches((12, 10)) 
            plt.pause(1)
            
            wd = os.getcwd()
            path=(wd + '/RaceTrack_RecoveryPaths')
            RecoveryPaths = 'RecoveryPaths%03d.png' % step
            RecoveryPaths = os.path.join(path,RecoveryPaths)
            plt.savefig(RecoveryPaths)
            plt.clf()

        if hasRecoveryPlan == False:
            print('No valid recovery path available')
        else:
            print(TC.OKGREEN + '\nSelected point between wp index ' + str(selectPath-1) + ' and ' + str(selectPath) + ' as the recovery insertion point' + TC.ENDC)
            NewPath = []
            for ptList in Recovery_dict['chosenPath']:
                NewPath.append([ptList[1], ptList[0]])

            if Show_SelectedRecoveryPathSnapShot == True:
                # plt.plot([pt[1] for pt in Recovery_dict['chosenPathFull'][0]], [pt[0] for pt in Recovery_dict['chosenPathFull'][0]])

                # circle1 = plt.Circle((Recovery_dict['cirlces'][0][0], Recovery_dict['cirlces'][0][1]), Recovery_dict['cirlces'][0][2],color='magenta',alpha=0.2)
                # plt.gca().add_artist(circle1)
                # plt.scatter(Recovery_dict['cirlces'][0][0],Recovery_dict['cirlces'][0][1])

                # circle2 = plt.Circle((Recovery_dict['cirlces'][1][0], Recovery_dict['cirlces'][1][1]), Recovery_dict['cirlces'][1][2],color='yellow',alpha=0.4)
                # plt.gca().add_artist(circle2)
                # plt.scatter(Recovery_dict['cirlces'][1][0],Recovery_dict['cirlces'][1][1])

                # plt.scatter(Recovery_dict['cirlces'][2][0],Recovery_dict['cirlces'][2][1])

                # circle4 = plt.Circle((Recovery_dict['cirlces'][3][0], Recovery_dict['cirlces'][3][1]), Recovery_dict['cirlces'][3][2],color='magenta',alpha=0.2)
                # plt.gca().add_artist(circle4)
                # plt.scatter(Recovery_dict['cirlces'][3][0],Recovery_dict['cirlces'][3][1])

                # # ==== Plot Selected Recovery Path Snapshot ====
                # plt.plot([pt[1] for pt in NewPath], [pt[0] for pt in NewPath], c='green', marker='o',markersize=5)

                # # plot keep out zones from UAVHeading.avoid() function
                # if len(KOZpoints)>1:
                #     plotCASkoz, = plt.plot([pt[1] for pt in KOZpoints[0]], [pt[0] for pt in KOZpoints[0]], '--m')
                #     plotNCkoz, = plt.plot([pt[1] for pt in KOZpoints[1]], [pt[0] for pt in KOZpoints[1]], '--m')

                # fig.set_size_inches((12, 10)) 

                # plt.plot(mainUAV['dubins'].ys, mainUAV['dubins'].xs, c='r', marker='o' )
                # plt.plot(uavh_others_all[0]['dubins'].y, uavh_others_all[0]['dubins'].x, c='y', marker='o' )

                # NCcone = uavh_others_all[0]['uavobj'].possibleFlightAreaStatic(area_length=area_length*1.0)
                # plotNCcone, = plt.plot([pt[1] for pt in NCcone], [pt[0] for pt in NCcone], "-r")

                # plot_RecoveryPlan = plt.plot([pt[1] for pt in NewPath], [pt[0] for pt in NewPath], c = 'g', marker='o', markersize=8)
                # plot_AstarPlan = plt.plot([pt[1] for pt in astarwpts.tolist()], [pt[0] for pt in astarwpts.tolist()], c = 'k', marker='*', markersize=8)
                # plt.plot([pt[1] for pt in PruitTrack], [pt[0] for pt in PruitTrack], c='b', marker='.', markersize=8)
                # plt.plot( activeWP[1], activeWP[0], c='k', marker='X', markersize = 5 )
                # plt.axis('equal')
                # plt.grid(True)
                # plt.ylim((mainUAV['dubins'].x - 0.001, mainUAV['dubins'].x + 0.001))
                # plt.xlim((mainUAV['dubins'].y - 0.001, mainUAV['dubins'].y + 0.001))
                # # plt.show(100)
                # # plt.pause(1)

                # wd = os.getcwd()
                # path=(wd + '/RaceTrack_SelectedPaths')
                # SelectedPath = 'SelectedPath%03d.png' % step
                # SelectedPath = os.path.join(path,SelectedPath)
                # plt.savefig(SelectedPath)
                #plt.close('all')
                plt.clf()

    if hasAstarPlan == True and hasRecoveryPlan == False:
        if mainUAV['dubins'].currentWPIndex > Astar_Return_Index-1:

            if targetIndex < mainUAV['dubins'].currentWPIndex:

                ActivePath_List = ReferencePath_List[:]
                TargetWPList = []
                for i in range(0, len(ReferencePath_List)):
                    TargetWPList.append(ActivePath_List[i]['pt'])
                mainUAV['dubins'].setWaypoints(TargetWPList, newradius=wptRad) 

                mainUAV['dubins'].currentWPIndex = targetIndex
                activeWP = ActivePath_List[mainUAV['dubins'].currentWPIndex]['pt']

                print('Completed A* path and a lap - no recovery path required')  

            else: 
                mainUAV['dubins'].currentWPIndex = Astar_Return_Index  # Need to test
                activeWP = ActivePath_List[mainUAV['dubins'].currentWPIndex]['pt']

            print('Completed A* path - no recovery path required')  

            hasAstarPlan = False
            Show_AstarPlan = False



    elif hasAstarPlan == True or hasRecoveryPlan == True:
        if not onlyOnce:
            onlyOnce = True
            lastIndex = mainUAV['dubins'].currentWPIndex
            numbOfRecoveryPts = len(NewPath)
            # insertIndex = indexRecall + numbOfAstarPts 
            insertIndex = mainUAV['dubins'].currentWPIndex

            # Update Active path with Recovery plan
            temp = []
            for i in range(0, len(NewPath)):
                Waypoint_dict['pt'] = NewPath[i]    
                Waypoint_dict['Belongs to'] = 'Recovery'
                Waypoint_dict['A* Return Index'] = None
                Waypoint_dict['A* Return Pt'] = None
                Waypoint_dict['Recover Return Index']  = None
                Waypoint_dict['Recover Return Pt']  = None
                Waypoint_dict['Is Go-To'] = False

                temp.append(Waypoint_dict.copy())
            # Insert A* points (wplist) into Active path
            ActivePath_List[Astar_Return_Index : Astar_Return_Index] = temp
            Recover_Return_Index = Astar_Return_Index + numbOfRecoveryPts
            TargetWPList = []
            for i in range(0, len(ActivePath_List)):
                TargetWPList.append(ActivePath_List[i]['pt'])

            uavlist[0]['dubins'].setWaypoints(TargetWPList, newradius=wptRad)
            mainUAV['dubins'].currentWPIndex = Astar_Return_Index
            activeWP = uavlist[0]['dubins'].waypoints[Astar_Return_Index]          #  mainUAV['dubins'].currentWPIndex = insertIndex 

            hasAstarPlan = False
            print(TC.OKBLUE + 'Insert ' + str(numbOfRecoveryPts) + ' Recovery Points at wpt index ' + str(insertIndex) + TC.ENDC)
        
        elif mainUAV['dubins'].currentWPIndex >= Recover_Return_Index :
            hasRecoveryPlan = False
            onlyOnce = False
            
            mainUAV['dubins'].currentWPIndex = Return2Ref_Index  # Need to test
            activeWP = uavlist[0]['dubins'].waypoints[uavlist[0]['dubins'].currentWPIndex]
            print('Completed Recovery path')

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

            # crossError, m, b = crossTrackError(PruitTrack, [uav['dubins'].x, uav['dubins'].y])

            if uav['dubins'].currentWPIndex >= len(ActivePath_List):
                for i in range(0, len(ActivePath_List)):
                    if ActivePath_List[i]['Is Go-To'] == True:
                        uav['dubins'].currentWPIndex = i
                        break

                ActivePath_List = ReferencePath_List[:]
                TargetWPList = []
                for i in range(0, len(ReferencePath_List)):
                    TargetWPList.append(ActivePath_List[i]['pt'])
                uav['dubins'].setwaypoints(TargetWPList, newradius=wptRad)

            uav['dubins'].simulateWPDubins(UseCarrotChase=False, delta=0.01)
            carrot = uav['dubins'].CarrotChaseWP(delta=0.01)
            CASuavPos = uav['dubins'].position

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
        plot_RecoveryPlan = plt.plot([pt[1] for pt in NewPath], [pt[0] for pt in NewPath], c = 'g', marker='o', markersize=8)
        # for pts in NewPath:
        #         wptCircle = plt.Circle((pts[1], pts[0]), wptRad, color='green', alpha=0.2)
        #         plt.gca().add_artist(wptCircle)
        #         plt.scatter(pts[1],pts[0])

    plt.plot([pt[1] for pt in PruitTrack], [pt[0] for pt in PruitTrack], c='b', marker='.', markersize=8)
    # plt.plot([pt[1] for pt in TargetWPList], [pt[0] for pt in TargetWPList], c='b', marker='+', markersize=8)
    plt.plot([pt[1] for pt in PruitTrack], [pt[0] for pt in PruitTrack], c='b', marker='.', markersize=8)


    # for pts in PruitTrack:
    #         # plot circles around each waypoint - viusal aid for waypoint updating
    #         wptCircle = plt.Circle((pts[1], pts[0]), wptRad, color='green', alpha=0.2)
    #         plt.gca().add_artist(wptCircle)
    #         plt.scatter(pts[1],pts[0])

    # for pts in PruitTrack:
    #         # plot circles around each waypoint - viusal aid for waypoint updating
    #         wptCircle = plt.Circle((pts[1], pts[0]), wptRad, color='green', alpha=0.2)
    #         plt.gca().add_artist(wptCircle)
    #         plt.scatter(pts[1],pts[0])


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
    step +=1