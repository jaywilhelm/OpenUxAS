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

def distance(a, b):
    return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

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

''' Function to find the current index number assigned to each reference path waypoint in the updated waypoint list'''
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
    last_targetIndex = uavObj['dubins'].currentWPIndex
    for i in range(0, len(refWaypoint_list)):
        if uavObj['dubins'].currentWPIndex == refWaypoint_list[i]['Index']:
            currentIndex = i

    astarGoalPoint_list = refWaypoint_list
    astarGoalPoint_dict = {}
    # Add in aditional Reference path in case the UAV is about to make a lap
    # This step is performed after the 'currentIndex' step above 
    # to avoid douplicate 'Index' entries
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
            # Used to find the distance between two successive reference path waypoints
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
        Look for a point some distance forward along the reference path to be used as the astarGoal point
        '''
        for pt in range(0, numbOfPts-1):
            d = distance([linX[pt], linY[pt]], [linX[pt+1], linY[pt+1]]) # Calculate distance between interpolated points
            distTotal += d                                               # total distance across interpolated points
            pointList.append([linX[pt], linY[pt]])
            if distTotal >= area_length*lookAheadDist:
                astarGoalPt = [linX[pt], linY[pt]]
                ' Report if the astarGoalPoint is on the next lap'
                targetIndex = astarGoalPoint_list[index]['Index']
                if targetIndex < last_targetIndex:
                    lap = True
                    last_targetIndex = targetIndex
                else:
                    lap = False
                    last_targetIndex = targetIndex
                break
        if distTotal >= area_length*lookAheadDist:
            break

    return astarGoalPt, targetIndex, pointList, lap 

def UpdateWPList(ActivePath_List, uavObj, wypts2add, numbOfPts, List_belongsTo, insertIndex):
    ''' Update Active path with avoidance or recovery path waypoints'''
    Waypoint_List = []
    Waypoint_dict = {}
    for i in range(0, len(wypts2add)):
        Waypoint_dict['pt'] = wypts2add[i]    
        Waypoint_dict['Belongs to'] = List_belongsTo
        Waypoint_dict['A* Return Index'] = uavObj['dubins'].currentWPIndex + numbOfPts
        Waypoint_dict['A* Return Pt'] = 'Astar'
        Waypoint_dict['Recover Return Index']  = None
        Waypoint_dict['Recover Return Pt']  = None
        Waypoint_dict['Is Go-To'] = False

        Waypoint_List.append(Waypoint_dict.copy())

    ''' Insert new points into Active waypoint list at specified index'''
    ActivePath_List[insertIndex : insertIndex] = Waypoint_List
            
def clearWPList(ActivePath_List, ReferencePath_List, lap_Counter, uavObj, wptRad ):
    ''' Completed a lap - clear AcitvePath_List and replace with Reference Path'''
    if uavObj['dubins'].lapCounter > lap_Counter:
        for i in range(0, len(ActivePath_List)):
            if ActivePath_List[i]['Is Go-To'] == True:
                uav['dubins'].currentWPIndex = i
                break

        ActivePath_List = ReferencePath_List
        TargetWPList = []
        for i in range(0, len(ReferencePath_List)):
            TargetWPList.append(ActivePath_List[i]['pt'])

        uavObj['dubins'].currentWPIndex = 0
        uavObj['dubins'].setWaypoints(TargetWPList, newradius=wptRad)

    return ActivePath_List

def getPotentialRecoveryPoints(ActivePath_List, ReferencePath_List, saveCurrentPOS, indexRecall, numbOfPts):

    InsertionPoint_list = findRefPathIndex(ActivePath_List)    
    InsetionPoint_dict = {}

    # Add in additional Reference path points in case the UAV is about to make a lap
    for j in range(0, 6):
        InsetionPoint_dict['Index'] = j
        InsetionPoint_dict['pt'] = ReferencePath_List[j]['pt']
        InsertionPoint_list.append(InsetionPoint_dict.copy()) 

    # Determine which waypoint on the referece path the UAV is currently heading towards and report that index
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

def getRecoveryPaths(RecoveryPoints, referencePathPoint, uavObj):
    # print('Interpolating ' + str(numbOfPts) + ' points between index ' + str(index) + ' and ' + str(index+1))

    '''
    Convert uav North East Down angle convention to cartesion for clothoid heading:
    Needed an axis flip to find the angle between UAV and active waypoint- x's in the numerator and y's in the denominator, then add 180 deg
    '''
    # Use dummy waypoint projected in front of CAS UAV to determine start heading for clothoid calcs
    UAVx = uavObj['dubins'].x
    UAVy = uavObj['dubins'].y
    r = 0.3
    px = UAVx + r * np.cos(uavObj['dubins'].heading)
    py = UAVy + r * np.sin(uavObj['dubins'].heading)

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

def selectRecoveryPath(clothoid_List, index_List, indexRecall, halfTurnRadius, numbOfwpts, lap):
    'List and counter variables for clothoid path selection'
    ClothoidPath = []           # Stores a specified number of waypoints for each clothoid segment
    counter = 0
    checkSuccess = False        # evaluates to True if a valid recovery path is found
    checkDistance = 99999       # used to evalauate the length of each recovery path
    last_targetIndex = indexRecall
    
    ''' Generate and Evaluate clothoid paths between UAV and recovery points''' 
    for i in range(0, len(clothoid_List)):
        clothoid = clothoid_List[i]     # clothoid parameters
        if lap:
            index = index_List[i]       # current index on reference path waypoints - used to inform where the clothoid is going 
        else:
            index = index_List[i]
        j=0                             # Keeps track of which clothoid segment is being evaluated
        clothoidLength = 0              # keeps track of clothoid path length - used for path selection
        temp = []                       # temporary variable - used for ploting purposes
        circle_list = []                # stored circle fit info for each clothoid segment - used for path selection

        for segmt in clothoid:
            'Sample points from the clothoid'
            points = segmt.SampleXY(numbOfwpts)         # used for circle fitting to calcuate a turn radius for each clothoid path and used for waypoint path following
            ClothoidPath.append(points)

            x0, y0, t0, k0, dk, s = segmt.Parameters    # (start x, start y, initial curvature, change in curvature, clothoid segment length)
            clothoidLength += s                     # Sum up the length of each clothoid segment to find the total distance of the full clothoid
            
            '''
            Fit a circle to clothoid segment  
            Used to determine the Turn Radius of each clothoid segment
            '''
            # Full set of clothoid points broken into x and y lists
            x_points = points[0]
            y_points = points[1]

            if j == 1: # <-- middle clothoid
                # Middle clothoid has two curvatures --> break into two halves 
                # and fit a circle to each half. 

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

            else: # <-- for 1st and 2nd clothoid
                xc,yc,r = fit_circle_2d(x_points, y_points)
                circle_list.append([xc,yc,r])

            j+=1


        print('\tChecking pt ' + str(counter+1) + '/' + str(len(clothoid_List)))
        counter+=1

        'Determine if clothoid violates UAV Turn Radius'
        print('\tPath Distance: ' + str(round(clothoidLength,3)) + '\tClothoid Segment Radii: ' + str(round(circle_list[0][2],3)) + 
                '   ' + str(round(circle_list[1][2],3)) + '   ' + str(round(circle_list[2][2],3)) + '   ' + str(round(circle_list[3][2],3)))

        checkPassed = False                 # keeps track if this particular path meets requirements
        for i in range(0, len(circle_list)):                        
            if circle_list[i][2] < halfTurnRadius:
                print(TC.WARNING + '\t\t\tTurn radius too small' + TC.ENDC)
                checkPassed = False
                break
            else:
                checkPassed = True # means that clothoid path satisfies requierments and UAV dynamics

        if checkPassed == True and clothoidLength < checkDistance:   
                checkSuccess = True            
                checkDistance = clothoidLength
                chosenClothoid = clothoid
                selectIndex = index+1
                print(TC.OKGREEN + '\t\t\tSelected index pt: ' +  str(selectIndex) + TC.ENDC)

        elif checkPassed == True and clothoidLength > checkDistance:
            print(TC.WARNING + '\t\t\tPath Too Long' + TC.ENDC)
                 
    if checkSuccess:
        return checkSuccess, chosenClothoid, selectIndex, lap
    else: 
        return checkSuccess, [], [], lap

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

def check_changePath(ActivePath_List, ReferencePath_List, TargetWPList, uavObj, Index2Watch4, lap, wptRad):
    
    changePath = False
    if uavObj['dubins'].currentWPIndex >= Index2Watch4:
        changePath = True

        # Reset waypoint list if lap is completed. 
        if lap == True:
            ActivePath_List = ReferencePath_List[:]
            TargetWPList = []
            for i in range(0, len(ReferencePath_List)):
                TargetWPList.append(ActivePath_List[i]['pt'])
            uavObj['dubins'].setWaypoints(TargetWPList, newradius=wptRad) 

            print('Lap completed - reset waypoint list')  

    return changePath, ActivePath_List, TargetWPList

def RecoveryPaths_SnapShot(file_path, step, clothoid_List, refPathpts, astarwpts, KOZpoints, show_keepOutZones, RecoveryPoints, activeWP, mainUAV, uavh_others_all, area_length, fig, ax  ):

    # fig, ax = plt.subplots()
    # ==== Plot Snapshot of all possible Clothoid paths ====
    for i in range(0, len(clothoid_List)):
        clothoid = clothoid_List[i]
        for segmnt in clothoid:
            plt.plot(*segmnt.SampleXY(500))

    # Plot full clothoid path(s) with larger number of sample points
    plt.plot(mainUAV['dubins'].ys, mainUAV['dubins'].xs, c='r', marker='o' )
    plt.plot(mainUAV['dubins'].y, mainUAV['dubins'].x, c='k', marker='o' )

    plt.scatter([pt[1] for pt in RecoveryPoints], [pt[0] for pt in RecoveryPoints])
    plt.plot(mainUAV['dubins'].ys, mainUAV['dubins'].xs, c='r', marker='o' )
    plt.plot(uavh_others_all[0]['dubins'].y, uavh_others_all[0]['dubins'].x, c='y', marker='o' )
    NCcone = uavh_others_all[0]['uavobj'].possibleFlightAreaStatic(area_length=area_length*1.0)
    plotNCcone, = plt.plot([pt[1] for pt in NCcone], [pt[0] for pt in NCcone], "-r")

    plot_AstarPlan = plt.plot([pt[1] for pt in astarwpts.tolist()], [pt[0] for pt in astarwpts.tolist()], c = 'k', marker='*', markersize=8)
    plt.plot([pt[1] for pt in refPathpts], [pt[0] for pt in refPathpts], c='b', marker='.', markersize=8)

    plt.plot( activeWP[1], activeWP[0], c='k', marker='X', markersize = 5 )
    # plot keep out zones from UAVHeading.avoid() function
    if show_keepOutZones:
        plotCASkoz, = plt.plot([pt[1] for pt in KOZpoints[0]], [pt[0] for pt in KOZpoints[0]], '--m')
        plotNCkoz, = plt.plot([pt[1] for pt in KOZpoints[1]], [pt[0] for pt in KOZpoints[1]], '--m')
    plt.axis('equal')
    plt.grid(True)
    # plt.ylim((mainUAV['dubins'].x - 0.001, mainUAV['dubins'].x + 0.001))
    # plt.xlim((mainUAV['dubins'].y - 0.001, mainUAV['dubins'].y + 0.001))
    plt.xlim((-82.11100, -82.1090))
    plt.ylim((39.32435, 39.32657))

    fig.set_size_inches((12, 10)) 
    # plt.show()
    # plt.pause(1)
    
    wd = os.getcwd()
    path=(wd + file_path)
    RecoveryPaths = 'RecoveryPaths%03d.png' % step
    RecoveryPaths = os.path.join(path,RecoveryPaths)
    plt.savefig(RecoveryPaths)
    plt.clf()

def RecoveryPath_SnapShot(file_path, step, refPath, astarwpts, pltPts_List, RecoveryPathWPs, activeWP, mainUAV, uavh_others_all, area_length, fig, ax ):

    # fig, ax = plt.subplots()
    plt.plot(mainUAV['dubins'].ys, mainUAV['dubins'].xs, c='r', marker='o' )
    plt.plot(mainUAV['dubins'].y, mainUAV['dubins'].x, c='k', marker='o' )

    plt.plot([pt[1] for pt in pltPts_List], [pt[0] for pt in pltPts_List], marker='.', c = 'b', markersize=4)
    plt.plot([pt[1] for pt in RecoveryPathWPs], [pt[0] for pt in RecoveryPathWPs], c='g', marker='o')
    plt.plot(uavh_others_all[0]['dubins'].y, uavh_others_all[0]['dubins'].x, c='y', marker='o' )
    CAScone = mainUAV['uavobj'].possibleFlightAreaStatic(area_length=area_length*1.0)
    plotCAScone, = plt.plot([pt[1] for pt in CAScone], [pt[0] for pt in CAScone], "-g")

    NCcone = uavh_others_all[0]['uavobj'].possibleFlightAreaStatic(area_length=area_length*1.0)
    plotNCcone, = plt.plot([pt[1] for pt in NCcone], [pt[0] for pt in NCcone], "-r")

    plot_AstarPlan = plt.plot([pt[1] for pt in astarwpts.tolist()], [pt[0] for pt in astarwpts.tolist()], c = 'k', marker='*', markersize=8)
    plt.plot([pt[1] for pt in refPath], [pt[0] for pt in refPath], c='b', marker='.', markersize=8)

    plt.plot( activeWP[1], activeWP[0], c='k', marker='X', markersize = 5 )
    # plot keep out zones from UAVHeading.avoid() function
    plt.axis('equal')
    plt.grid(True)
    # plt.ylim((mainUAV['dubins'].x - 0.001, mainUAV['dubins'].x + 0.001))
    # plt.xlim((mainUAV['dubins'].y - 0.001, mainUAV['dubins'].y + 0.001))
    plt.xlim((-82.11100, -82.1090))
    plt.ylim((39.32435, 39.32657))

    fig.set_size_inches((12, 10)) 
    wd = os.getcwd()
    path=(wd + file_path)
    SelectedPath = 'SelectedPath%03d.png' % step
    SelectedPath = os.path.join(path,SelectedPath)
    plt.savefig(SelectedPath)
    plt.clf()
    # plt.show()


def Show_AstarSnapShot(file_path, step, activeWP, mainUAV, uavh_others_all, refPath, astarwpts, astarGoalPt, KOZpoints, area_length, fig, ax):

    #fig, ax = plt.subplots()

    # ===== Plot snapshot of Astar Path and save frame ==============
    plt.plot(mainUAV['dubins'].ys, mainUAV['dubins'].xs, c='r', marker='o' )
    plt.plot(uavh_others_all[0]['dubins'].y, uavh_others_all[0]['dubins'].x, c='y', marker='o' )
    plot_AstarPlan = plt.plot([pt[1] for pt in astarwpts.tolist()], [pt[0] for pt in astarwpts.tolist()], c = 'k', marker='*', markersize=8)
    plt.plot([pt[1] for pt in refPath], [pt[0] for pt in refPath], c='b', marker='.', markersize=8)

    plt.plot( activeWP[1], activeWP[0], c='k', marker='X', markersize = 5 )
    plt.plot( astarGoalPt[1], astarGoalPt[0], c='k', marker='^', markersize = 6 )

    # plot keep out zones from UAVHeading.avoid() function
    # plotCASkoz, = plt.plot([pt[1] for pt in KOZpoints[0]], [pt[0] for pt in KOZpoints[0]], '--m')
    CAScone = mainUAV['uavobj'].possibleFlightAreaStatic(area_length=area_length*1.0)
    plotCAScone, = plt.plot([pt[1] for pt in CAScone], [pt[0] for pt in CAScone], "-g")
    plotNCkoz, = plt.plot([pt[1] for pt in KOZpoints[1]], [pt[0] for pt in KOZpoints[1]], '--m')
    plt.axis('equal')
    plt.grid(True)
    plt.ylim((mainUAV['dubins'].x - 0.001, mainUAV['dubins'].x + 0.001))
    plt.xlim((mainUAV['dubins'].y - 0.001, mainUAV['dubins'].y + 0.001))
    fig.set_size_inches((12, 10)) 
    # plt.show()

    wd = os.getcwd()
    path=(wd + file_path)
    AstarPath = 'AstarPath%03d.png' % step
    AstarPaths = os.path.join(path, AstarPath)
    plt.savefig(AstarPaths)
    plt.clf()

def flightProj(posX, posY, heading, velocity, turnRate, dt, lookAhead_time):
    areaLength = lookAhead_time*velocity
    turnLengths = 0
    time = np.arange(0, lookAhead_time, dt)
    TRs = np.arange(-turnRate*1, turnRate*1, np.deg2rad(1))

    save = []
    edgePts = [[posX, posY]]
    for i in range(0, len(TRs)):
        x=posX
        y=posY
        head = heading
        for j in range(0, len(time)):
            head = head + TRs[i]*dt

            vx = velocity * np.cos(head)
            vy = velocity * np.sin(head)
            
            x = x + vx * dt
            y = y + vy * dt
            save.append([x, y])
        edgePts.append([x,y])

    edgePts.append([posX,posY])

    # plt.scatter([pt[1] for pt in save], [pt[0] for pt in save])
    # plt.scatter([pt[1] for pt in edgePts], [pt[0] for pt in edgePts])
    # plt.plot([pt[1] for pt in edgePts], [pt[0] for pt in edgePts])
    # plt.scatter(posY, posX)

    # plt.axis('equal')
    # plt.grid(True)
    # plt.show()
    # plt.clf()
    return edgePts