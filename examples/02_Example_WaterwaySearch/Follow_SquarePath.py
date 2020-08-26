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
import os, subprocess, time
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

testPath = [[-82.1380578,39.4103728],[-82.1379970,39.4037904],[-82.1379970,39.4037005],[-82.1379973,39.3812916],[-82.1379973,39.3812017],[-82.1368338,39.3816193]]

# [ Long, Lat] QGC kml saved it this way?
refPath = [[-82.1380578,39.4103728],[-82.1379970,39.4037904],[-82.1379970,39.4037005],[-82.1379973,39.3812916],[-82.1379973,39.3812017],[-82.1368338,39.3816193],
           [-82.1368338,39.3817093],[-82.1368331,39.4037635],[-82.1368331,39.4038534],[-82.1356692,39.4039164],[-82.1356692,39.4038265],[-82.1356703,39.3821269],
           [-82.1356703,39.3820369],[-82.1345067,39.3824546],[-82.1345067,39.3825445],[-82.1345053,39.4038895],[-82.1345053,39.4039794],[-82.1333414,39.4040424],
           [-82.1333414,39.4039525],[-82.1333432,39.3829621],[-82.1333432,39.3828721],[-82.1321796,39.3832897],[-82.1321796,39.3833797],[-82.1321776,39.4040155],
           [-82.1321775,39.4041054],[-82.1310136,39.4041684],[-82.1310137,39.4040784],[-82.1310160,39.3837972],[-82.1310160,39.3837073],[-82.1298524,39.3841249],
           [-82.1298524,39.3842148],[-82.1298498,39.4041414],[-82.1298498,39.4042313],[-82.1286859,39.4042942],[-82.1286859,39.4042043],[-82.1286888,39.3846323],
           [-82.1286888,39.3845424],[-82.1275252,39.3849599],[-82.1275252,39.3850499],[-82.1275220,39.4042672],[-82.1275220,39.4043572],[-82.1263580,39.4044201],
           [-82.1263581,39.4043302],[-82.1263615,39.3854674],[-82.1263615,39.3853775],[-82.1251979,39.3857950],[-82.1251979,39.3858849],[-82.1251942,39.4043931],
           [-82.1251941,39.4044830],[-82.1240302,39.4045459],[-82.1240303,39.4044560],[-82.1240342,39.3863024],[-82.1240342,39.3862125],[-82.1228705,39.3866300],
           [-82.1228705,39.3867199],[-82.1228663,39.4045188],[-82.1228663,39.4046088],[-82.1217024,39.4046716],[-82.1217024,39.4045817],[-82.1217068,39.3871374],
           [-82.1217068,39.3870474],[-82.1205431,39.3874649],[-82.1205431,39.3875548],[-82.1205385,39.4046446],[-82.1205385,39.4047345],[-82.1193746,39.4047973],
           [-82.1193746,39.4047074],[-82.1193793,39.3879723],[-82.1193794,39.3878824],[-82.1182156,39.3882998],[-82.1182156,39.3883897],[-82.1182107,39.4047702],
           [-82.1182107,39.4048602],[-82.1170468,39.4049230],[-82.1170468,39.4048331],[-82.1170518,39.3888072],[-82.1170519,39.3887172],[-82.1158881,39.3891346],
           [-82.1158881,39.3892246],[-82.1158829,39.4048959],[-82.1158828,39.4049858],[-82.1147189,39.4050486],[-82.1147189,39.4049587],[-82.1147243,39.3896420],
           [-82.1147243,39.3895521],[-82.1135605,39.3899694],[-82.1135605,39.3900594],[-82.1135550,39.4050215],[-82.1135550,39.4051114],[-82.1123911,39.4051742],
           [-82.1123911,39.4050842],[-82.1123967,39.3904768],[-82.1123967,39.3903868],[-82.1112329,39.3908042],[-82.1112329,39.3908941],[-82.1112272,39.4051470],
           [-82.1112271,39.4052369],[-82.1100632,39.4052997],[-82.1100632,39.4052098],[-82.1100690,39.3913115],[-82.1100691,39.3912216],[-82.1089052,39.3916389],
           [-82.1089052,39.3917288],[-82.1088993,39.4052725],[-82.1088993,39.4053624],[-82.1077353,39.4054252],[-82.1077354,39.4053352],[-82.1077413,39.3921462],
           [-82.1077413,39.3920563],[-82.1065775,39.3924736],[-82.1065774,39.3925635],[-82.1065714,39.4053979],[-82.1065714,39.4054879],[-82.1054074,39.4055506],
           [-82.1054075,39.4054607],[-82.1054135,39.3929808],[-82.1054136,39.3928909],[-82.1042496,39.3933082],[-82.1042496,39.3933981],[-82.1042436,39.4055111],
           [-82.1042435,39.4056010],[-82.1030807,39.4033618],[-82.1030808,39.4032719],[-82.1030857,39.3938154],[-82.1030857,39.3937255],[-82.1019218,39.3941428],
           [-82.1019217,39.3942327],[-82.1019181,39.4010326],[-82.1019181,39.4011226],[-82.1007555,39.3988833],[-82.1007555,39.3987934],[-82.1007578,39.3946500],
           [-82.1007578,39.3945600],[-82.0995939,39.3949773],[-82.0995938,39.3950672],[-82.0995930,39.3965541],[-82.0995929,39.3966440],[-82.0925783,39.3866215],
           [-82.1072662,39.3761598]]

        
# Convert refPath to km, source: https://sciencing.com/convert-latitude-longtitude-feet-2724.html
refPathkm = []
for pos in refPath:
    Latkm = pos[1] * 10000/90 # distance from the equator
    Longkm = pos[0] * 10000/90  # distance from prime meridian
    refPathkm.append([Longkm, Latkm])

print('Ref path in km: ' + str(refPathkm))

fig, ax = plt.subplots()

savePlots = []  # stores figure frames used to make a movie

# Calcuate heading angle between each waypoint -> used for clothoid path generation
pathHeadings = []
i = 0
for i in range(0, len(refPath)-1):
    wpt0 = refPath[i]
    wpt1 = refPath[i+1]
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
wptRad = 0.0005
np.deg2rad(30)
thetaRef = np.deg2rad(270)
uav1 = dubinsUAV(position=[-82.1380578, 39.4103728], velocity=v,          # recall: 45.35, -120.5
                                    heading=thetaRef, dt=dt)

uav1.setWaypoints(newwps=refPath, newradius = wptRad )

# from Ch 5 in the Pilot's Handbook of Aernautical Knowledge 
# Using 50% of maximum turn radius - conservative value - given by np.degrees(uav1.turnrate)/2
uavTurnRadius = ((uav1.v * 360/(np.degrees(uav1.turnrate)))/np.pi)/2
halfTurnRadius = uavTurnRadius*2

print('UAV turn radius with full turn rate: ' + str(uavTurnRadius) + ' and half turn rate: ' + str(halfTurnRadius))
Recovery_dict = {'X': [], 'Y' : [], 'Index1' : [], 'Index2' : [], 'wpt1' : [], 'wpt2' : [],
                 'pathLength' : [], 'turnRadii': []   }


useAstar = True

hasPath = False
onlyOnce = False
wpList =None
numbOfAstarPts = 0
NewPath = []
indexTracker = 0 # TO DO - find another way to change the index to the appropriate waypoint
numbOfRecoveryPts = 0

step = 0
while uav1.currentWPIndex < len(uav1.waypoints)-1: #step < 550:
    # Update Dubins pseudo wp
    xy = (uav1.x, uav1.y)
    r = 0.3
    px = xy[0] + r * np.cos(uav1.heading)
    py = xy[1] + r * np.sin(uav1.heading)

    # Follow the refference path or follow a pseudo waypoint projected in front of the dubins uav
    followRefPath = True
    if followRefPath == False:
        if wpList == None:
            wpts = [ [px,py],[0,0]]
            # wpts = [px,py]
            uav1.setWaypoints(newwps=wpts, newradius = wptRad )
            uav1.simulateWPDubins(UseCarrotChase=False, delta=wptRad)
    else:
        uav1.simulateWPDubins(UseCarrotChase=False, delta=wptRad)
        carrot = uav1.CarrotChaseWP(delta = 0.01)
        # plotCarrot, = plt.plot(carrot[1], carrot[0], c='k', marker='^' )

    activeWP = uav1.getActiveWaypoint()     # get the current waypoint coordinates 

    # TO DO: Insert Fake A* points
    if step == 34 :  
        indexRecall = uav1.currentWPIndex 
        fakeAstarPath = [[uav1.x, uav1.y], [-82.139, 39.4025], [-82.139, 39.397], [-82.139, 39.385], [-82.1379973, 39.3812916] ]  # [-82.1379970, 39.4037904]
        #fakeAstarPath = [[uav1.x, uav1.y], [45.4, -120.58], [45.35, -120.66], [45.26835347140579, -120.82]]
        numbOfAstarPts= len(fakeAstarPath)
        refPath[uav1.currentWPIndex:uav1.currentWPIndex] = fakeAstarPath
        print('Insert ' + str(numbOfAstarPts) + ' Fake Astar Points at wpt index ' + str(indexRecall))

    checkDistance = 9999999     # used to evaluate shorted clothoid path using interpolated target waypoints
    if step == 100:
        # convert uav North East Down angle convention to cartesion for clothoid heading: uavHeading = uav1.heading + np.radians(90)
        # Needed an axis flip to find the angle between  UAV and active waypoint: the x's in the numerator and y's in the denom, then add 180 deg

        uavHeading = np.arctan2(uav1.x - px, uav1.y - py) + np.radians(180)

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
            x1 = uav1.waypoints[index][0]
            x2 = uav1.waypoints[index+1][0]
            linX = np.linspace(x1, x2, numbOfPts,endpoint=False )

            y1 = uav1.waypoints[index][1]
            y2 = uav1.waypoints[index+1][1]
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

                targetHeading = np.arctan2(linX[pt] - x2, linY[pt] - y2) + np.radians(180) # heading between interpolated point and wp2
                targetWPT = [linX[pt], linY[pt], targetHeading]  # heading should still be the same?
                plt.plot(targetWPT[0], targetWPT[1], 'o', markersize=5)

                if(targetWPT[2] >= np.pi*2):
                    targetWPT[2] -= np.pi*2
                if(targetWPT[2] < 0):
                    targetWPT[2] += np.pi*2

                # Provides clothoid parameters used to generate a path between the UAV and target waypoint on reference path: 3 clothoids per path are needed
                clothoid_list = pyclothoids.SolveG2(uav1.y, uav1.x, uavHeading, 0, targetWPT[1], targetWPT[0], targetWPT[2], 0) # stiches a path of multiple clothoids
                
                circle_list = []    # Stores circle fit info for each clothoid segment in a single path: [cx,cy,r]
                wpList = []         # Stores a specified number of waypoints for each clothoid segment
                wpList2 = []        # Stored the 1st, middle, and last wpt from the 1st, 2nd, and 3rd clothoid respectively
                jj = 0              # Keeps track of which clothoid segment is being evaluated
                clothoidLength = 0  # keeps track of clothoid path length
                temp = []           # temporary variable - used for ploting purposes
                completeClothoidPts = []    # stores a the full path of each clothoid generated - mainly used for plotting purposes

                for i in clothoid_list:
                    #plt.plot(*i.SampleXY(500))
                    pltpts = i.SampleXY(500)
                    for ii in range(0, len(pltpts[0])):
                        temp.append([pltpts[1][ii], pltpts[0][ii]])

                    points = i.SampleXY(numbOfwpts)         # used for circle fitting to calcuate a turn radius for each clothoid path and used for waypoint path following
                    wpList.append(points)

                    x0, y0, t0, k0, dk, s = i.Parameters    # start x, start y, initial curvature, change in curvature, clothoid segment length
                    clothoidLength += s                     # Sum up the length of each clothoid segment to find the total distance of the clothoid
                    
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
                        #wpList2.append([mid_x,mid_y])
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
                clothoid_paths.append([index, clothoidLength, wpList, wpList2, completeClothoidPts, circle_list])   # store info for each clothoid
                Recovery_dict['pathLength'].append(clothoidLength)
                Recovery_dict['turnRadii'].append(circle_list)

                'Determine if path violates Turn Radius'
                print('\tPath Distance: ' + str(round(clothoidLength,3)) + '\tClothoid Radii: ' + str(round(circle_list[0][2],3)) + 
                        '   ' + str(round(circle_list[1][2],3)) + '   ' + str(round(circle_list[2][2],3)) + '   ' + str(round(circle_list[3][2],3)))
            
                checkPassed = False
                for r in range(0, len(circle_list)):                        
                    if circle_list[r][2] < halfTurnRadius:
                        print('\tTurn radius too small')
                        checkPassed = False
                        break
                    else:
                        checkPassed = True

                if checkPassed == True and clothoidLength < checkDistance:
                    selectPath = index+1
                    checkDistance = clothoidLength
                    hasPath = True
                    Recovery_dict['chosenPathFull'] = wpList
                    Recovery_dict['chosenPath'] = wpList2
                    Recovery_dict['cirlces'] = circle_list
                    numbOfRecoveryPts = len(wpList2)
                    Recovery_dict['chosenIndex'] = index+1+numbOfRecoveryPts
                    print('Chosen Index: ' + str(Recovery_dict['chosenIndex']))

                elif checkPassed == True and clothoidLength > checkDistance:
                    print('Path Too Long')
                

            for clothoid in clothoid_paths:
                plt.plot([pt[0] for pt in clothoid[4][0]], [pt[1] for pt in clothoid[4][0]])




        plt.plot(uav1.xs, uav1.ys, c='r', marker='o' )
        plt.plot([pt[0] for pt in refPath], [pt[1] for pt in refPath], c='b', marker='.')
        plt.plot( activeWP[0], activeWP[1], c='k', marker='X', markersize = 5 )
        plt.plot([pt[0] for pt in fakeAstarPath], [pt[1] for pt in fakeAstarPath], c='magenta', marker='o',markersize=5)
        plt.axis('equal')
        plt.grid(True)
  
        plt.show(100)
    
        if hasPath == False:
            print('No valid path available')
        else:
            print('\nSelected point between wp index ' + str(selectPath-1) + ' and ' + str(selectPath) + ' as the recovery insertion point\n')

            # plt.plot([pt[0] for pt in Recovery_dict['chosenPath']], [pt[1] for pt in Recovery_dict['chosenPath']])

            circle1 = plt.Circle((Recovery_dict['cirlces'][0][1], Recovery_dict['cirlces'][0][0]), Recovery_dict['cirlces'][0][2],color='magenta',alpha=0.2)
            plt.gca().add_artist(circle1)
            plt.scatter(Recovery_dict['cirlces'][0][1],Recovery_dict['cirlces'][0][0])

            circle2 = plt.Circle((Recovery_dict['cirlces'][1][1], Recovery_dict['cirlces'][1][0]), Recovery_dict['cirlces'][1][2],color='yellow',alpha=0.4)
            plt.gca().add_artist(circle2)
            plt.scatter(Recovery_dict['cirlces'][1][1],Recovery_dict['cirlces'][1][0])

            circle3 = plt.Circle((Recovery_dict['cirlces'][2][1], Recovery_dict['cirlces'][2][0]), Recovery_dict['cirlces'][2][2],color='yellow',alpha=0.4)
            plt.gca().add_artist(circle3)
            plt.scatter(Recovery_dict['cirlces'][2][1],Recovery_dict['cirlces'][2][0])

            circle4 = plt.Circle((Recovery_dict['cirlces'][3][1], Recovery_dict['cirlces'][3][0]), Recovery_dict['cirlces'][3][2],color='magenta',alpha=0.2)
            plt.gca().add_artist(circle4)
            plt.scatter(Recovery_dict['cirlces'][3][1],Recovery_dict['cirlces'][3][0])

            for ptList in Recovery_dict['chosenPath']:
                NewPath.append([ptList[1], ptList[0]])

            plt.plot([pt[0] for pt in NewPath], [pt[1] for pt in NewPath], c='green', marker='o',markersize=5)

            plt.plot(uav1.xs, uav1.ys, c='r', marker='o' )
            plt.plot([pt[0] for pt in refPath], [pt[1] for pt in refPath], c='b', marker='.')
            plt.plot([pt[0] for pt in fakeAstarPath], [pt[1] for pt in fakeAstarPath], c='magenta', marker='o',markersize=5)
            plt.plot( activeWP[0], activeWP[1], c='k', marker='X', markersize = 5 )
            plt.axis('equal')
            plt.grid(True)
            plt.xlim((uav1.x - 0.004, uav1.x + 0.004)) #scale this to where the recovery waypoints are. maybe center on the middle?
            plt.ylim((uav1.y - 0.004, uav1.y + 0.004))
            plt.show(100)

 
    # for pts in refPath:
    #     # plot circles around each waypoint - viusal aid for waypoint updating
    #     wptCircle = plt.Circle((pts[0], pts[1]), wptRad, color='green', alpha=0.2)
    #     plt.gca().add_artist(wptCircle)
    #     plt.scatter(pts[0],pts[1])

    # if wpList !=None:
    #     for pts in NewPath:
    #             wptCircle = plt.Circle((pts[0], pts[1]), wptRad, color='green', alpha=0.2)
    #             plt.gca().add_artist(wptCircle)
    #             plt.scatter(pts[0],pts[1])

    if wpList !=None and hasPath == True and onlyOnce == False:
    # for ptList in wpList:
    #     for i in range(0, len(ptList[0])):
    #         NewPath.append([ptList[1][i], ptList[0][i]])
        onlyOnce = True
        if followRefPath == False:
            uav1.setWaypoints(newwps=NewPath, newradius = wptRad )
        else:
            indexTracker = 0
            lastIndex = uav1.currentWPIndex
            numbOfRecoveryPts = len(NewPath)
            insertIndex = indexRecall + numbOfAstarPts + 2 # To Do Find a better way to select the next Index....
            refPath[insertIndex:insertIndex] = NewPath
            uav1.currentWPIndex = insertIndex 
            print('Insert ' + str(numbOfRecoveryPts) + ' Recovery Points at wpt index ' + str(insertIndex) )
          

    if uav1.currentWPIndex == (len(NewPath)-1) and hasPath == True and followRefPath == False:
        uav1.setWaypoints(newwps=Path, newradius = wptRad )
        uav1.currentWPIndex = index
        hasPath = False
        #plt.pause(30) 
    elif indexTracker > (numbOfRecoveryPts) and hasPath == True and followRefPath == True: # TO DO - update wp in dubinsUAV class to appropriate wp after completing clothoid
        # uav1.currentWPIndex = Recovery_dict['chosenIndex'] # note - briefly targets wrong wp b/c there is a wp update, then this line is executed. 
        hasPath = False
        onlyOnce = False
        indexTracker = 0
        #plt.pause(30)   

    if  onlyOnce == True and uav1.currentWPIndex > lastIndex:
        indexTracker+=1
        lastIndex = uav1.currentWPIndex


    plt.plot([pt[0] for pt in refPath], [pt[1] for pt in refPath], c='b', marker='.')
    plt.plot(uav1.xs, uav1.ys, c='r', marker='o', markersize=5 )

    if wpList != None:
        plt.plot([pt[0] for pt in NewPath], [pt[1] for pt in NewPath], c='green', marker='o',markersize=5)

    if step>1000000:
        plt.plot([pt[0] for pt in fakeAstarPath], [pt[1] for pt in fakeAstarPath], c='magenta', marker='o',markersize=5)

    plt.plot( activeWP[0], activeWP[1], c='k', marker='X', markersize = 5 )

    plt.axis('equal')
    plt.grid(True)
    scale = 0.01

    dist2WP = distance( [uav1.x, uav1.y], [uav1.waypoints[uav1.currentWPIndex][0], uav1.waypoints[uav1.currentWPIndex][1]] )
    if dist2WP < 0.005:
        plt.xlim((uav1.x - 0.002, uav1.x + 0.002))
        plt.ylim((uav1.y - 0.002, uav1.y + 0.002))

    if uav1.currentWPIndex >0:
        dist2WP2 = distance( [uav1.x, uav1.y], [uav1.waypoints[uav1.currentWPIndex-1][0], uav1.waypoints[uav1.currentWPIndex-1][1]] )
        if dist2WP2 < 0.005:
            plt.xlim((uav1.x - 0.002, uav1.x + 0.002))
            plt.ylim((uav1.y - 0.002, uav1.y + 0.002))




    #plt.xlim(-82.14 - scale, -82.10 + scale) 
    #plt.ylim(39.375 + scale, 39.410 - scale) 
    plt.pause(0.05) 
    # if wpList != None:
    #     plt.show(100) 
    # time.sleep(0.1)


    print('Step: ' + str(step) + '\tCurrent wpt: ' + str(uav1.currentWPIndex) + '\tUAV Heading(deg): ' + str(round(uav1.heading,2)) + ' (' + str(round(np.degrees(uav1.heading),2)) + ')') 

    wd = os.getcwd()
    path=(wd + '/Movies')
    fname = '_tmp%03d.png' % step
    fname = os.path.join(path,fname)
    plt.savefig(fname)
    savePlots.append(fname)

    plt.clf()
    step+=1

print('Change directory to Movies: ')
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
