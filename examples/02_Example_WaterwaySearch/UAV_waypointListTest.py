# Jeremy Browne Fall 2020
# Script to test list of dictonary entries for CAS UAV waypoint updating
# 1) Follow refference path - search area pattern
# 2) Insert fake A* waypoints to simulate an avoidance maneuver
# 3) After clearing the fake NC UAV, use clothoids to determine closest path back to the 
#    reference path (search area) without violating the UAVs turn radius
#
# Each List entry will be a dictonary containing:
# 1) The waypoint coordinates
# 2) A label to identify which set that waypoint belongs to
#         a) Referecent Path
#         b) A-star replan/avoid point list
#         c) Recovery point list
#         d) go-to point - identifies begninig of race track
#
# Continue to insert new a-star and recovery points as the CAS UAV travels ther reference path
# Reset/upload the original race track waypoint list when the CAS UAV completes a lap
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

# RaceTack points taken from Pruitt Field using Goolgle maps


PruitTrack =[
[39.3264051914717, -82.1101289994580], 
[39.3263499334731, -82.1103352244684], 
[39.3261989661035, -82.1104861915330],

[39.3259927415369, -82.1105414491151], 
[39.3249489949075, -82.1105414491151],
[39.3247433390045, -82.1104865192473],
[39.3245921315912, -82.1103357926091],
[39.3245365454954, -82.1101296557923],
[39.3245914749791, -82.1099233430367],
[39.3247422017190, -82.1097721357249],

[39.3249489949075, -82.1097165492456], 
[39.3259927415369, -82.1097165492456],
 
[39.3261989665845, -82.1097718071053], 
[39.3263499337508, -82.1099227743733],
]

PruitTrack_reverse = [
    [39.3261989661035, -82.1104861915330], 
[39.3263499334731, -82.1103352244684], 
[39.3264051914717, -82.1101289994580], 
[39.3263499337508, -82.1099227743733], 
[39.3261989665845, -82.1097718071053], 
[39.3259927415369, -82.1097165492456], 
[39.3249489949075, -82.1097165492456], 
[39.3247422017190, -82.1097721357249], 
[39.3245914749791, -82.1099233430367], 
[39.3245365454954, -82.1101296557923], 
[39.3245921315912, -82.1103357926091], 
[39.3247433390045, -82.1104865192473], 
[39.3249489949075, -82.1105414491151], 
[39.3259927415369, -82.1105414491151]
]

# PruitTrack = [[39.32628292131059, -82.11015604454057], [39.32626145051805, -82.11026814786695], [39.32622134219231, -82.11036301599236],
#               [39.32615604526910, -82.11045259331949], [39.32606878323577, -82.11051085088990], [39.32599274153694, -82.11054144911510],
#               [39.32496507981838, -82.11035891577949], [39.32487946196044, -82.11031124013499], [39.32482219330970, -82.11023990355510],
#               [39.32477723258300, -82.11015140625042], [39.32474662137297, -82.11002965875700], [39.32475050696790, -82.10991519781285],
#               [39.32477817497424, -82.10979249702034], [39.32482490415428, -82.10970003394840], [39.32488214561512, -82.10962612568042],
#               [39.32495478705711, -82.10957178856511], [39.32504987702541, -82.10954174636318], [39.32607266604762, -82.10972043031671], 
#               [39.32613754430359, -82.10978019543180], [39.32620606612441, -82.10985404007286], [39.32625375907247, -82.10995031576066],
#               [39.32628781654859, -82.11007381400570]]

# PruitTrack_reverse = [[39.32628781654859, -82.11007381400570], [39.32625375907247, -82.10995031576066], [39.32620606612441, -82.10985404007286], 
#                       [39.32613754430359, -82.10978019543180], [39.32607266604762, -82.10972043031671], [39.32504987702541, -82.10954174636318], 
#                       [39.32495478705711, -82.10957178856510], [39.32488214561512, -82.10962612568042], [39.32482490415428, -82.10970003394840], 
#                       [39.32477817497424, -82.10979249702034], [39.32475050696790, -82.10991519781285], [39.32474662137297, -82.11002965875700], 
#                       [39.32477723258300, -82.11015140625042], [39.32482219330970, -82.11023990355510], [39.32487946196044, -82.11031124013499], 
#                       [39.32496507981838, -82.11035891577949], [39.32599274153694, -82.11054144911510], [39.32606878323577, -82.11051085088990], 
#                       [39.32615604526910, -82.11045259331950], [39.32622134219231, -82.11036301599236], [39.32626145051805, -82.11026814786695], 
#                       [39.32628292131059, -82.11015604454057]]

# PruitTrack = [[39.32628292131059, -82.11015604454057], [39.32626145051805, -82.11026814786695], [39.32622134219231, -82.11036301599236],
# [39.32615604526910, -82.11045259331949], [39.32606878323577, -82.11051085088990], [39.32599748514458, -82.11055043769464],
# [39.32547276161426, -82.11060085940424], [39.32496507981838, -82.11035891577949], [39.32487946196044, -82.11031124013499], 
# [39.32482219330970, -82.11023990355510], [39.32477723258300, -82.11015140625042], [39.32474662137297, -82.11002965875700],
# [39.32475050696790, -82.10991519781285], [39.32477817497424, -82.10979249702034], [39.32482490415428, -82.10970003394840],
# [39.32488214561512, -82.10962612568042], [39.32495478705711, -82.10957178856511], [39.32504170830252, -82.10953169097074],
# [39.32557045383421, -82.10949902984331], [39.32607266604762, -82.10972043031671], [39.32613754430359, -82.10978019543180],
# [39.32620606612441, -82.10985404007286], [39.32625375907247, -82.10995031576066], [39.32628528279560, -82.11005357934607]]

# PruitTrack_reverse = [
#     [39.32628528279560, -82.11005357934607], [39.32625375907247, -82.10995031576066], [39.32620606612441, -82.10985404007286], 
#     [39.32613754430359, -82.10978019543180], [39.32607266604762, -82.10972043031671], [39.32557045383421, -82.10949902984330], 
#     [39.32504170830252, -82.10953169097074], [39.32495478705711, -82.10957178856510], [39.32488214561512, -82.10962612568042], 
#     [39.32482490415428, -82.10970003394840], [39.32477817497424, -82.10979249702034], [39.32475050696790, -82.10991519781285], 
#     [39.32474662137297, -82.11002965875700], [39.32477723258300, -82.11015140625042], [39.32482219330970, -82.11023990355510], 
#     [39.32487946196044, -82.11031124013499], [39.32496507981838, -82.11035891577949], [39.32547276161426, -82.11060085940424], 
#     [39.32599748514458, -82.11055043769464], [39.32606878323577, -82.11051085088990], [39.32615604526910, -82.11045259331950], 
#     [39.32622134219231, -82.11036301599236], [39.32626145051805, -82.11026814786695], [39.32628292131059, -82.11015604454057]]


# IndianaSpeedWay = [
#     [39.80188151410402, -86.23492138262816],[39.80184132168075, -86.23632269247335],[39.80148703904483, -86.23758538753252],
#     [39.80064475627571, -86.23859431116239],[39.79951048065981, -86.23896838699319],[39.79047393450715, -86.23879828484318],
#     [39.78938933243088, -86.23847634938477],[39.78852787277844, -86.23744837003628],[39.78814010357731, -86.23613800097900],
#     [39.78812189134263, -86.23471949300118],[39.78815809591920, -86.23327790344395],[39.78849732406272, -86.23172499235579],
#     [39.78950037132575, -86.23061488381596],[39.79051611204371, -86.23036523085713],[39.79960822266639, -86.23049483271154],
#     [39.80060161259791, -86.23080886759983],[39.80156614442307, -86.23193581703875],[39.80188309112465, -86.23336029077471]]


Waypoint_dict = {}
ActivePath_List = []    # Active path that the CAS UAV will follow - will be modified by A* and recovery paths
ReferencePath_List = [] # Use to re-upload original path
for i in range(0, len(PruitTrack)):
    Waypoint_dict['pt'] = PruitTrack[i]    
    Waypoint_dict['Belongs to'] = 'Reference Path'
    if i == 0:
        Waypoint_dict['Is Go-To'] = True
    else:
        Waypoint_dict['Is Go-To'] = False

    ActivePath_List.append(Waypoint_dict.copy())
    ReferencePath_List.append(Waypoint_dict.copy())


# Create Dubins Vehicle
dt = 0.2 # 0.1
v = 0.00005/2.75
wptRad = 0.0001 # distance threshold to satisfy each waypoint
#
uavlist = []    # holds dictionary items for specific UAVs
uav1 = {}
uavlist.append(uav1)
uav2 = {}
uavlist.append(uav2)
#
thetaRef = np.deg2rad(270)
uavlist[0]['dubins'] = dubinsUAV(position=[39.3264051914717, -82.1101289994580], velocity=v,          
                                    heading=thetaRef, dt=dt)
deadpoint = [39.40, -82.1380578]
uavlist[0]['ID'] = 1
uavlist[0]['IsAvoidanceUAV'] = True
uavlist[0]['dubins'].setWaypoints(newwps=PruitTrack, newradius = wptRad )
uavlist[0]['dubins'].currentWPIndex = 0
#
v1 = 0.00005/2 # 2*0.00025
thetaRef = np.deg2rad(135)
uavlist[1]['dubins'] = dubinsUAV(position=[39.3261989665845, -82.1097718071053], velocity=v1,         
                                    heading=thetaRef, dt=dt)
uavlist[1]['ID'] = 4
uavlist[1]['IsAvoidanceUAV'] = False
#
uavlist[0] = syncAVSfromDubins(uavlist[0])
uavlist[1] = syncAVSfromDubins(uavlist[1])
uavlist[1]['dubins'].setWaypoints(newwps=PruitTrack_reverse, newradius = wptRad )
uavlist[1]['dubins'].currentWPIndex = 4

# from Ch 5 in the Pilot's Handbook of Aernautical Knowledge 
# Using 50% of maximum turn radius - conservative value - given by np.degrees(uavlist[0]['dubins'].turnrate)/2
uavTurnRadius = ((uavlist[0]['dubins'].v * 360/(np.degrees(uavlist[0]['dubins'].turnrate)))/np.pi)/2
halfTurnRadius = uavTurnRadius*2

# Some fudge factor that scales the A* format to the vehicle capability 
# resolution = fudgFactor * constant * halfTurnRadius or turn rate

area_length = 0.0002

usetargetPath = True
hasPath = False
hadPlan = False
hasPlan = False
hasAstarPlan = False
hasRecoveryPlan = False
onlyOnce = False

# stores figure frames used to make a movie
savePlots = []  
savePlots1 = []
savePlots2 = []
savePlots3 = []
savePlots4 = []
savePlots5 = []


TargetWPList = None
TargetWPList_dict = {}
Recovery_dict = {'X': [], 'Y' : [], 'Index1' : [], 'Index2' : [], 'wpt1' : [], 'wpt2' : [],
                 'pathLength' : [], 'turnRadii': []   }

fig, ax = plt.subplots()
step = 0
while step < 1499:
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
        
        TargetWPList = []
        for i in range(0, len(ReferencePath_List)):
            TargetWPList.append(ActivePath_List[i]['pt'])

        uavlist[0]['dubins'].setWaypoints(TargetWPList, newradius=wptRad)
        uavlist[0]['dubins'].currentWPIndex = 0
        TargetWPList_dict['SimStep']= step
        TargetWPList_dict['CurrentWPIndex']= mainUAV['dubins'].currentWPIndex
        TargetWPList_dict['TargetWPList'] = TargetWPList
        TargetWPList_dict['With A*'] = False
        TargetWPList_dict['With Recovery'] = False
        TargetWPList_dict['Original Path'] = True
        # Log_list[2].append(TargetWPList_dict.copy())

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
            '''
            Astar goal point selection
            '''
            ''' 
            Make a list of dictionary entries that containts onf the reference path waypoints
            and the corresponding index in the Active waypoint list
            '''
            astarGoalPoint_list = []
            astarGoalPoint_dict = {}
            for i in range(0, len(ActivePath_List)):
                if ActivePath_List[i]['Belongs to'] == 'Reference Path' or ActivePath_List[i]['Belongs to'] == 'Recovery' :
                    astarGoalPoint_dict['Index'] = i
                    astarGoalPoint_dict['pt'] = ActivePath_List[i]['pt']
                    astarGoalPoint_list.append(astarGoalPoint_dict.copy())

            for i in range(0, len(astarGoalPoint_list)):
                if mainUAV['dubins'].currentWPIndex == astarGoalPoint_list[i]['Index']:
                    currentIndex = i

            distTotal = 0
            pointList=[]
            for index in range(currentIndex, len(astarGoalPoint_list)):
                if index == currentIndex:
                    x1 = mainUAV['dubins'].x
                    y1 = mainUAV['dubins'].y 

                    x2 = astarGoalPoint_list[index]['pt'][0]
                    y2 = astarGoalPoint_list[index]['pt'][1]
                else:
                    x1 = astarGoalPoint_list[index-1]['pt'][0]
                    y1 = astarGoalPoint_list[index-1]['pt'][1]

                    x2 = astarGoalPoint_list[index]['pt'][0]
                    y2 = astarGoalPoint_list[index]['pt'][1]

                '''
                Use a linear space to interpolate points between each reference path waypoint
                Also, does not include the endpoint ( ie. [x2,y2]) - it is included in the next index set
                '''
                numbOfPts = 50
                linX = np.linspace(x1, x2, numbOfPts, endpoint=False )
                linY = np.linspace(y1, y2, numbOfPts, endpoint=False )

                ''' 
                Look for an interpolated point some distance forward along the reference path as the astarGoal point
                '''
                lookAheadDist = 3
                for pt in range(0, numbOfPts-1):
                    #print('\tChecking pt ' + str(pt+1))
                    # if pt == 0:
                    #     d = distance([posX, posY], [linX[pt+1], linY[pt+1]]) # Calculate distance between interpolated points
                    # else:
                    d = distance([linX[pt], linY[pt]], [linX[pt+1], linY[pt+1]]) # Calculate distance between interpolated points
                    distTotal += d  
                    pointList.append([linX[pt], linY[pt]])
                    if distTotal >= area_length*lookAheadDist:
                        astarGoalPt = [linX[pt], linY[pt]]
                        targetIndex = astarGoalPoint_list[index]['Index']
                        # print(TC.WARNING + 'Found Astar Goal Point' + TC.ENDC)
                        break
                if distTotal >= area_length*lookAheadDist:
                    break

            plt.plot([pt[1] for pt in pointList], [pt[0] for pt in pointList], c='k', marker='.', markersize = 8)
            plt.plot(astarGoalPt[1], astarGoalPt[0], c='r', marker='*', markersize = 12)

            # lookAhead = 5       
            # if mainUAV['dubins'].currentWPIndex+lookAhead < len(ActivePath_List):
            #     astarGoalPt = ActivePath_List[mainUAV['dubins'].currentWPIndex+lookAhead]['pt']
            # else:
            #     temp = mainUAV['dubins'].currentWPIndex + lookAhead - len(ActivePath_List)
            #     astarGoalPt = ActivePath_List[temp]['pt']


            hasAstarPlan = False
            replan, wplist, avoid, full_path, uavID, AstarFail = mainUAV['uavobj'].avoid(uavh_others, area_length=area_length, static_koz=[], TargetPathWP=astarGoalPt, useAstarGoal=True, simStep=step)

            # Deals with cases where the NC UAV may be on top of the A* target/active waypoint
            # if AstarFail:
            #     print(TC.WARNING +'A* failed to find path' + TC.ENDC)
            #     lookAhead = 5      
            #     if mainUAV['dubins'].currentWPIndex + lookAhead < len(ActivePath_List):
            #         astarGoalPt = ActivePath_List[mainUAV['dubins'].currentWPIndex+lookAhead]['pt']
            #     else:
            #         temp = mainUAV['dubins'].currentWPIndex + lookAhead - len(ActivePath_List)
            #         astarGoalPt = ActivePath_List[temp]['pt']

            #     print('Checking if next wpt on ref path is valid')
            #     replan, wplist, avoid, full_path, uavID, AstarFail = mainUAV['uavobj'].avoid(uavh_others, area_length=area_length, static_koz=[], TargetPathWP=astarGoalPt, useAstarGoal=True, simStep=step)
                      
            ''' Use uavID to determine with NC UAV is being Avoided '''
            if len(uavID) > 0:
                for i in range(0, len(uavID)):    
                    uavID[i] = uavh_others_all[uavID[i]]['ID']
                    print('Potential Collision with UAV ' + str(uavID[i]))

        # if replan or AstarFail==True:
        #     plotCASkoz, = plt.plot([pt[1] for pt in avoid[0]], [pt[0] for pt in avoid[0]], '--m')
        #     plotNCkoz, = plt.plot([pt[1] for pt in avoid[1]], [pt[0] for pt in avoid[1]], '--m')

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
            numbOfAstarPts= len(wplist)  # -1 accounts for the added astarGoalPt which is already on the reference path

            watch4Index = targetIndex
            Astar_Return_Index = mainUAV['dubins'].currentWPIndex + numbOfAstarPts

            # Update Active path with A* replan
            temp = []
            for i in range(0, len(wplist)):
                Waypoint_dict['pt'] = wplist[i]    
                Waypoint_dict['Belongs to'] = 'Astar'
                Waypoint_dict['A* Return Index'] = mainUAV['dubins'].currentWPIndex + numbOfAstarPts
                Waypoint_dict['A* Return Pt'] = 'Astar'
                Waypoint_dict['Recover Return Index']  = None
                Waypoint_dict['Recover Return Pt']  = None
                Waypoint_dict['Is Go-To'] = False

                temp.append(Waypoint_dict.copy())
            # Insert A* points (wplist) into Active path
            ActivePath_List[mainUAV['dubins'].currentWPIndex : mainUAV['dubins'].currentWPIndex] = temp

            TargetWPList = []
            for i in range(0, len(ActivePath_List)):
                TargetWPList.append(ActivePath_List[i]['pt'])

            uavlist[0]['dubins'].setWaypoints(TargetWPList, newradius=wptRad)
            activeWP = uavlist[0]['dubins'].waypoints[uavlist[0]['dubins'].currentWPIndex]

            print(TC.OKBLUE + 'Insert ' + str(numbOfAstarPts) + ' Astar Points at wpt index ' + str(indexRecall) + TC.ENDC)
            closingDist = mainUAV['dubins'].distance(mainUAV['uavobj'].position, uavh_others[0].position)

            # =====Plot snapshot ==============
            plt.plot(mainUAV['dubins'].ys, mainUAV['dubins'].xs, c='r', marker='o' )
            plt.plot(uavh_others_all[0]['dubins'].y, uavh_others_all[0]['dubins'].x, c='y', marker='o' )
            plot_AstarPlan = plt.plot([pt[1] for pt in wplist.tolist()], [pt[0] for pt in wplist.tolist()], c = 'k', marker='*', markersize=8)
            plt.plot([pt[1] for pt in PruitTrack], [pt[0] for pt in PruitTrack], c='b', marker='.', markersize=8)

            plt.plot( activeWP[1], activeWP[0], c='k', marker='X', markersize = 5 )
            plt.plot( astarGoalPt[1], astarGoalPt[0], c='k', marker='^', markersize = 6 )

            # plot keep out zones from UAVHeading.avoid() function
            plotCASkoz, = plt.plot([pt[1] for pt in avoid[0]], [pt[0] for pt in avoid[0]], '--m')
            plotNCkoz, = plt.plot([pt[1] for pt in avoid[1]], [pt[0] for pt in avoid[1]], '--m')
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

            #plt.pause(1)


        else:
            print("Not re-planning" )
    else:
        print('\tOnly one UAV')

    checkDistance = 9999999     # used to evaluate shortest clothoid path using interpolated target waypoints
    if hadPlan and mainUAV['dubins'].trackUAV[0]['clearedUAV']:
        hadPlan = False

        InsertionPoint_list = []
        InsetionPoint_dict = {}
        for i in range(0, len(ActivePath_List)):
            if ActivePath_List[i]['Belongs to'] == 'Reference Path':
                InsetionPoint_dict['Index'] = i
                InsetionPoint_dict['pt'] = ActivePath_List[i]['pt']
                InsertionPoint_list.append(InsetionPoint_dict.copy())

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
        for index in range(0, len(InsertionPoint_list)-1):  # TO DO change 7 to some wp index within a detected range of the UAV
            numbOfPts = 5                       # Number of interpolate points between each Reference Path index and index+1

            # Note that RefRaceTrack is the unchanged original refernce path
            if index == indexRecall-1:
                x1 = saveCurrentPOS[0]
                y1 = saveCurrentPOS[1]
            else:
                x1 = InsertionPoint_list[index]['pt'][0]
                y1 = InsertionPoint_list[index]['pt'][1]

            x2 = InsertionPoint_list[index+1]['pt'][0]
            y2 = InsertionPoint_list[index+1]['pt'][1]

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
                    # plt.plot(*i.SampleXY(500))
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
                    Recovery_dict['chosenPath'] = wpList2
                    Recovery_dict['cirlces'] = circle_list
                    numbOfRecoveryPts = len(wpList2)
                    Return2Ref_Index = InsertionPoint_list[index+1]['Index'] + numbOfRecoveryPts

                    #Recovery_dict['chosenIndex'] = selectPath + numbOfRecoveryPts # + numbOfAstarPts
                    Recovery_dict['chosenIndex'] =  Astar_Return_Index
                    Recovery_dict['returnIndex'] = Astar_Return_Index + numbOfRecoveryPts
                    print(TC.OKGREEN + '\t\t\tSelected index pt: ' + str(selectPath) + ' which is now at index ' + str(Recovery_dict['chosenIndex']) + TC.ENDC)

                elif checkPassed == True and clothoidLength > checkDistance:
                    print(TC.WARNING + '\t\t\tPath Too Long' + TC.ENDC)
                    

            # for clothoid in clothoid_paths:
            #     plt.plot([pt[1] for pt in clothoid[4][0]], [pt[0] for pt in clothoid[4][0]])


        # ==== Plot Snapshot ====
        plt.plot(mainUAV['dubins'].ys, mainUAV['dubins'].xs, c='r', marker='o' )
        plt.plot(uavh_others_all[0]['dubins'].y, uavh_others_all[0]['dubins'].x, c='y', marker='o' )
        NCcone = uavh_others_all[0]['uavobj'].possibleFlightAreaStatic(area_length=area_length*1.0)
        plotNCcone, = plt.plot([pt[1] for pt in NCcone], [pt[0] for pt in NCcone], "-r")

        plot_AstarPlan = plt.plot([pt[1] for pt in wplist.tolist()], [pt[0] for pt in wplist.tolist()], c = 'k', marker='*', markersize=8)
        plt.plot([pt[1] for pt in PruitTrack], [pt[0] for pt in PruitTrack], c='b', marker='.', markersize=8)

        plt.plot( activeWP[1], activeWP[0], c='k', marker='X', markersize = 5 )
        # plot keep out zones from UAVHeading.avoid() function
        if replan:
            plotCASkoz, = plt.plot([pt[1] for pt in avoid[0]], [pt[0] for pt in avoid[0]], '--m')
            plotNCkoz, = plt.plot([pt[1] for pt in avoid[1]], [pt[0] for pt in avoid[1]], '--m')
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
            # TargetWPList = ReferencePath_List[:]
            # mainUAV['dubins'].setWaypoints(TargetWPList, newradius=wptRad)
            NewPath = []
            for ptList in Recovery_dict['chosenPath']:
                NewPath.append([ptList[1], ptList[0]])

            # ==== Plot Snapshot ====
            plt.plot([pt[1] for pt in NewPath], [pt[0] for pt in NewPath], c='green', marker='o',markersize=5)

            # plot keep out zones from UAVHeading.avoid() function
            if len(avoid)>1:
                plotCASkoz, = plt.plot([pt[1] for pt in avoid[0]], [pt[0] for pt in avoid[0]], '--m')
                plotNCkoz, = plt.plot([pt[1] for pt in avoid[1]], [pt[0] for pt in avoid[1]], '--m')

            fig.set_size_inches((12, 10)) 

            plt.plot(mainUAV['dubins'].ys, mainUAV['dubins'].xs, c='r', marker='o' )
            plt.plot(uavh_others_all[0]['dubins'].y, uavh_others_all[0]['dubins'].x, c='y', marker='o' )

            NCcone = uavh_others_all[0]['uavobj'].possibleFlightAreaStatic(area_length=area_length*1.0)
            plotNCcone, = plt.plot([pt[1] for pt in NCcone], [pt[0] for pt in NCcone], "-r")

            plot_RecoveryPlan = plt.plot([pt[1] for pt in NewPath], [pt[0] for pt in NewPath], c = 'g', marker='o', markersize=8)
            plot_AstarPlan = plt.plot([pt[1] for pt in wplist.tolist()], [pt[0] for pt in wplist.tolist()], c = 'k', marker='*', markersize=8)
            plt.plot([pt[1] for pt in PruitTrack], [pt[0] for pt in PruitTrack], c='b', marker='.', markersize=8)
            plt.plot( activeWP[1], activeWP[0], c='k', marker='X', markersize = 5 )
            plt.axis('equal')
            plt.grid(True)
            plt.ylim((mainUAV['dubins'].x - 0.001, mainUAV['dubins'].x + 0.001))
            plt.xlim((mainUAV['dubins'].y - 0.001, mainUAV['dubins'].y + 0.001))
            # plt.show(100)
            # plt.pause(1)

            wd = os.getcwd()
            path=(wd + '/RaceTrack_SelectedPaths')
            SelectedPath = 'SelectedPath%03d.png' % step
            SelectedPath = os.path.join(path,SelectedPath)
            plt.savefig(SelectedPath)
            #plt.close('all')
            plt.clf()


    if hasAstarPlan == True and hasRecoveryPlan == False:
        if mainUAV['dubins'].currentWPIndex > Astar_Return_Index-1:

            if  Astar_Return_Index + numbOfAstarPts < len(ActivePath_List):
                mainUAV['dubins'].currentWPIndex = Astar_Return_Index + numbOfAstarPts  # Need to test
                activeWP = ActivePath_List[mainUAV['dubins'].currentWPIndex]['pt']
            else:
                # TargetWPList = []
                # for i in range(0, len(ActivePath_List)):
                #     if ActivePath_List[i]['Is Go-To'] == True:
                #         uav['dubins'].currentWPIndex = i
                #         break
                mainUAV['dubins'].currentWPIndex = Astar_Return_Index + numbOfAstarPts - len(ActivePath_List)

                ActivePath_List = ReferencePath_List[:]
                TargetWPList = []
                for i in range(0, len(ReferencePath_List)):
                    TargetWPList.append(ActivePath_List[i]['pt'])
                mainUAV['dubins'].setWaypoints(TargetWPList, newradius=wptRad)

                activeWP = ActivePath_List[mainUAV['dubins'].currentWPIndex]['pt']

            print('Completed A* path - no recovery path required')

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

            # RecoveryList_dict['SimStep'] = step
            # RecoveryList_dict['CurrentWPIndex'] = mainUAV['dubins'].currentWPIndex
            # RecoveryList_dict['WPlist'] = NewPath
            # RecoveryList_dict['Recovery Index'] = Recovery_dict['chosenIndex']
            # RecoveryList_dict['RefPath return Index'] = Recovery_dict['returnIndex']
            # Log_list[1].append(RecoveryList_dict.copy())

            # TargetWPList_dict['SimStep']= step
            # TargetWPList_dict['CurrentWPIndex']= mainUAV['dubins'].currentWPIndex
            # TargetWPList_dict['TargetWPList'] = TargetWPList
            # TargetWPList_dict['Recovery Goal'] = NewPath[numbOfRecoveryPts-1]
            # TargetWPList_dict['With A*'] = False
            # TargetWPList_dict['With Recovery'] = True
            # TargetWPList_dict['Original Path'] = False

            # Log_list[2].append(TargetWPList_dict.copy())

        elif mainUAV['dubins'].currentWPIndex >= Recover_Return_Index :
            hasRecoveryPlan = False
            onlyOnce = False
            
            mainUAV['dubins'].currentWPIndex = Return2Ref_Index  # Need to test
            activeWP = uavlist[0]['dubins'].waypoints[uavlist[0]['dubins'].currentWPIndex]


            TargetWPList_dict['SimStep']= step
            TargetWPList_dict['CurrentWPIndex']= mainUAV['dubins'].currentWPIndex
            TargetWPList_dict['TargetWPList'] = TargetWPList
            TargetWPList_dict['With A*'] = False
            TargetWPList_dict['With Recovery'] = False
            TargetWPList_dict['Original Path'] = True

            # Log_list[2].append(TargetWPList_dict.copy())
            print('Completed Recovery path')

    '''================================
       === Update vehicle positions ===
       ================================'''
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
            plotCurrentWypt = plt.plot(uav['dubins'].waypoints[uav['dubins'].currentWPIndex][1], uav['dubins'].waypoints[uav['dubins'].currentWPIndex][0], c='black', marker='X')
            # plotCASkoz, = plt.plot([pt[1] for pt in avoid[0]], [pt[0] for pt in avoid[0]], '--m')

            # crossError, m, b = crossTrackError(PruitTrack, [uav['dubins'].x, uav['dubins'].y])

            if usetargetPath: 
                if step >= 1198:
                    mycheck = 1

                #print(TC.OKBLUE+ '\tUsing Target Path' + TC.ENDC)
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
            else:
                uav['dubins'].update_pos_simple()
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
    if hasAstarPlan:
        plot_AstarPlan = plt.plot([pt[1] for pt in wplist.tolist()], [pt[0] for pt in wplist.tolist()], c = 'k', marker='*', markersize=8)
        # for pts in wplist.tolist():
        #         wptCircle = plt.Circle((pts[1], pts[0]), wptRad, color='green', alpha=0.2)
        #         plt.gca().add_artist(wptCircle)
        #         plt.scatter(pts[1],pts[0])

    if hasRecoveryPlan :
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

    # plt.text(0.1, 0.05, text, transform=ax.transAxes)
    # plt.text(0.1, 0.9, text1, transform=ax.transAxes)

    ax.axis('equal')
    # plt.ylim((39.32450, 39.32657))
    plt.xlim((-82.11100, -82.1090))

    plt.ylim((39.32435, 39.32657))
    # plt.xlim((-82.1117, -82.1095))


    # plt.ylim((39.788, 39.802))
    # plt.xlim((-86.241, -86.227))

    fig.set_size_inches((12, 10))  
    plt.grid(True)
    # plt.pause(0.01)

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

    plt.clf()

    step+=1

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

# subprocess.call("mencoder 'mf://_tmpc*.png' -mf type=png:fps=10 -ovc lavc "
#                 "-lavcopts vcodec=mpeg4 -oac copy -o animation3.mp4", shell=True)

# subprocess.call("mencoder 'mf://_tmpd*.png' -mf type=png:fps=10 -ovc lavc "
#                 "-lavcopts vcodec=mpeg4 -oac copy -o animation4.mp4", shell=True)

# subprocess.call("mencoder 'mf://_tmpe*.png' -mf type=png:fps=10 -ovc lavc "
#                 "-lavcopts vcodec=mpeg4 -oac copy -o animation5.mp4", shell=True)

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
# for fname in savePlots3:
#     os.remove(fname)   
# print('Clean up...')
# for fname in savePlots4:
#     os.remove(fname) 
# print('Clean up...')
# for fname in savePlots5:
#     os.remove(fname) 
#     print('Clean up...')


print('Reverting to previous Directory')
os.chdir(wd)








