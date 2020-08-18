# For Dubins Vehicle
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
import os, subprocess
import numpy as np
from dubinsUAV import dubinsUAV
from TerminalColors import TerminalColors as TC
# For clothoid path
import pyclothoids
from pyclothoids import Clothoid
from scipy import linalg


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

fig, ax = plt.subplots()

# Create Dubins Vehicle
dt = 0.25 # 0.1
v = 0.01
wptRad = 0.01
np.deg2rad(30)
thetaRef = np.deg2rad(250)
uav1 = dubinsUAV(position=[45.5, -120.5], velocity=v,          # recall: 45.35, -120.5
                                    heading=thetaRef, dt=dt)

xy = (uav1.x, uav1.y)
r = 0.3
px = xy[0] + r * np.cos(uav1.heading)
py = xy[1] + r * np.sin(uav1.heading)
uav1.setWaypoints(newwps=[px, py], newradius = wptRad )

# ''' Generate the main path. Goal is to reconncet to this path after avoiding another UAV/obstacle'''
# # usetargetPath = True
# TargetWPList = uav1.makePath(pathType='Sine', numbOfPoints=20, dist=0.08)
# uav1.setWaypoints(TargetWPList, newradius=0.01)
uav1.currentWPIndex = 0
# plt.plot([pt[1] for pt in TargetWPList], [pt[0] for pt in TargetWPList], c='b', marker='.')
# print(TargetWPList)

LinePath = [[45.35, -120.5], [45.35, -120.58], [45.35, -120.66], 
[45.35, -120.74], [45.35, -120.82], [45.35, -120.89999999999999], 
[45.35, -120.97999999999999], [45.35, -121.05999999999999], [45.35, -121.13999999999999], 
[45.35, -121.21999999999998], [45.35, -121.29999999999998], [45.35, -121.37999999999998], 
[45.35, -121.45999999999998], [45.35, -121.53999999999998], [45.35, -121.61999999999998], 
[45.35, -121.69999999999997], [45.35, -121.77999999999997], [45.35, -121.85999999999997], 
[45.35, -121.93999999999997], [45.35, -122.01999999999997], [45.35, -122.09999999999997]]

lineHeadings = []
i = 0
for i in range(0, len(LinePath)-1):
    wpt0 = LinePath[i]
    wpt1 = LinePath[i+1]
    theta = np.arctan2((wpt1[0] - wpt0[0]), (wpt1[1] - wpt0[1])) 
    if(theta >= np.pi*2):
        theta -= np.pi*2
    if(theta < 0):
        theta += np.pi*2

    lineHeadings.append(theta)

SinePath = [[45.35, -120.5], [45.300000000000004, -120.58], [45.26054297453018, -120.66], 
            [45.24826870017314, -120.74], [45.26835347140579, -120.82], [45.31232715896611, -120.89999999999999], 
            [45.36164522413625, -120.97999999999999], [45.395509302717535, -121.05999999999999], [45.39963826999115, -121.13999999999999], 
            [45.37229086208503, -121.21999999999998], [45.324999999999996, -121.29999999999998], [45.27770913791496, -121.37999999999998], 
            [45.25036173000884, -121.45999999999998], [45.254490697282456, -121.53999999999998], [45.288354775863745, -121.61999999999998], 
            [45.33767284103388, -121.69999999999997], [45.3816465285942, -121.77999999999997], [45.40173129982685, -121.85999999999997], 
            [45.38945702546981, -121.93999999999997], [45.34999999999999, -122.01999999999997], [45.29999999999999, -122.09999999999997]]


sineHeadings = []
i = 0
for i in range(0, len(SinePath)-1):
    wpt0 = SinePath[i]
    wpt1 = SinePath[i+1]
    theta = np.arctan2((wpt1[0] - wpt0[0]), (wpt1[1] - wpt0[1])) 
    if(theta >= np.pi*2):
        theta -= np.pi*2
    if(theta < 0):
        theta += np.pi*2

    sineHeadings.append(theta)

UseSingleUAV = True
usePath = 'Sine'
if usePath == 'Line':
    Path = LinePath
    Headings = lineHeadings

elif usePath == 'Sine':
    Path = SinePath
    Headings = sineHeadings

plt.plot([pt[1] for pt in Path], [pt[0] for pt in Path], c='b', marker='.')

wpList = None
plotCarrot = None
hasPath = False
onlyOnce = False
step = 0
NewPath = []
diff = []
savePlots = []

NewPath1 = []
NewPath2 = []
NewPath3 = []
wpList1 = None
wpList2 = None


# from Ch 5 in the Pilot's Handbook of Aernautical Knowledge 
# Using 50% of maximum turn radius - conservative value - given by np.degrees(uav1.turnrate)/2
uavTurnRadius = ((uav1.v * 360/(np.degrees(uav1.turnrate)/2))/np.pi)/2
print('UAV turn radius: ' + str(uavTurnRadius))
while step < 250:
    # Update Dubins wp
    xy = (uav1.x, uav1.y)
    r = 0.3
    px = xy[0] + r * np.cos(uav1.heading)
    py = xy[1] + r * np.sin(uav1.heading)

    if wpList == None:
        wpts = [ [px,py],[0,0]]
        # wpts = [px,py]
        uav1.setWaypoints(newwps=wpts, newradius = wptRad )
        uav1.simulateWPDubins(UseCarrotChase=False, delta=0.01)
    else:
        uav1.simulateWPDubins(UseCarrotChase=False, delta=0.01)
        carrot = uav1.CarrotChaseWP(delta = 0.01)
        # plotCarrot, = plt.plot(carrot[1], carrot[0], c='k', marker='^' )

    activeWP = uav1.getActiveWaypoint()

    if step == 50:
        # convert uav North East Down angle convention to cartesion for clothoid heading: uavHeading = uav1.heading + np.radians(90)
        # Needed an axis flip to find the angle between  UAV and active waypoint: the x's in the numerator and y's in the denom, then add 180 deg
        uavHeading = np.arctan2(uav1.x - activeWP[0], uav1.y - activeWP[1]) + np.radians(180)

        # Keep uavHeading within [0, 360] degrees
        if(uavHeading >= np.pi*2):
            uavHeading -= np.pi*2
        if(uavHeading < 0):
            uavHeading += np.pi*2

        numbOfwpts = 10     # how many waypoints per clothoid will be use for path follwing (3 clothoids needed for a solution)
        clothoid_paths = [] # List of information on each clothoid path generated between UAV and waypoint on reference path
        
        'Find shortest Clothoid path that does not violate UAV turning radius'
        for index in range(0, 7):
            targetWPT = [Path[index][0], Path[index][1], Headings[index]]
            if(targetWPT[2] >= np.pi*2):
                targetWPT[2] -= np.pi*2
            if(targetWPT[2] < 0):
                targetWPT[2] += np.pi*2

            # Provides clothoid parameters used to generate a path between the UAV and target waypoint on reference path: 3 clothoids per path are needed
            clothoid_list = pyclothoids.SolveG2(uav1.y, uav1.x, uavHeading, np.radians(0), targetWPT[1], targetWPT[0], targetWPT[2], np.radians(0)) # stiches a path of multiple clothoids
            
            circle_list = []    # Stores circle fit info for each clothoid segment in a single path: [cx,cy,r]
            wpList = []         # Stores a specified number of waypoints for each clothoid segment
            wpList2 = []        # Stored the 1st, middle, and last wpt from the 1st, 2nd, and 3rd clothoid respectively
            jj = 0              # Keeps track of which clothoid segment is being evaluated
            clothoidLength = 0  # keeps track of clothoid path length
            temp = []           # temporary variable - used for ploting purposes
            completeClothoidPts = []    # stores a the full path of each clothoid generated - mainly used for plotting purposes

            for i in clothoid_list:
                pltpts = i.SampleXY(500)
                for ii in range(0, len(pltpts[0])):
                    temp.append([pltpts[1][ii], pltpts[0][ii]])

                points = i.SampleXY(numbOfwpts)         # used for circle fitting to calcuate a turn radius for each clothoid path and used for waypoint path following
                wpList.append(points)

                x0, y0, t0, k0, dk, s = i.Parameters    # start x, start y, initial curvature, change in curvature, clothoid segment length
                clothoidLength += s                     # Sum up the length of each clothoid segment to find the total distance of the clothoid
                
                print('Index: ' + str(index)  + ' Clothoid Path Distance: ' + str(clothoidLength) + ' and s: ' + str(s))  

                # Select the first point of the 1st clothoid
                # middle point of the 2nd clothoid and
                # the last point of the 3rd clothoid segment to follow
                midPt = int(len(points[0])/2)
                endPt = len(points[0])-1
                if jj == 0:
                    wpList2.append([points[0][1],points[1][1]])
                elif jj == 1:
                    mid_x = i.X(s/2)
                    mid_y = i.Y(s/2)
                    wpList2.append([mid_x,mid_y])
                elif jj == 2:
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

            completeClothoidPts.append(temp)                                                                    # store points of entire clothoid path
            clothoid_paths.append([index, clothoidLength, wpList, wpList2, completeClothoidPts, circle_list])   # store info for each clothoid


        'Determing which path to choose. Looking for the shortest path that does not violate the UAV turn radius'
        clothoidIndex = 0
        checkDistance = 9999999
        for clothoid in clothoid_paths:
            print('Target Wypt: ' + str(clothoid[0]) +  '\tPath Distance: ' + str(round(clothoid[1],3)) + 
                  '\tClothoid Radii: ' + str(round(clothoid[5][0][2],3)) + '   ' + str(round(clothoid[5][1][2],3)) + 
                  '   ' + str(round(clothoid[5][2][2],3)) + '   ' + str(round(clothoid[5][3][2],3)))

            checkPassed = False
            for rad in range(0, len(clothoid[5])):                        
                if clothoid[5][rad][2] < uavTurnRadius:
                    print('Turn radius too small in path to waypoint: ' + str(clothoid[0]))
                    checkPassed = False
                    break
                else:
                    checkPassed = True

            if clothoid[1] <= checkDistance and checkPassed == True:
                selectPath = clothoid[0]
                selectIndex = clothoidIndex
                checkDistance = clothoid[1]
                index = clothoid[0]
                hasPath = True
            
            clothoidIndex +=1
 
        if hasPath == False:
            print('No valid path available')
        else:
            print('Choose waypoint: ' + str(selectPath) + ' as the reconnection point')

            plt.plot([pt[1] for pt in clothoid_paths[selectIndex][4][0]], [pt[0] for pt in clothoid_paths[selectIndex][4][0]])

            circle1 = plt.Circle((clothoid_paths[selectIndex][5][0][0],clothoid_paths[selectIndex][5][0][1]),clothoid_paths[selectIndex][5][0][2],color='magenta',alpha=0.2)
            plt.gca().add_artist(circle1)
            plt.scatter(clothoid_paths[selectIndex][5][1][0],clothoid_paths[selectIndex][5][1][0])

            circle2a = plt.Circle((clothoid_paths[selectIndex][5][1][0],clothoid_paths[selectIndex][5][1][1]),clothoid_paths[selectIndex][5][1][2],color='yellow',alpha=0.2)
            plt.gca().add_artist(circle2a)
            plt.scatter(clothoid_paths[selectIndex][5][1][0],clothoid_paths[selectIndex][5][1][1])
            circle2b = plt.Circle((clothoid_paths[selectIndex][5][2][0],clothoid_paths[selectIndex][5][2][1]),clothoid_paths[selectIndex][5][2][2],color='yellow',alpha=0.2)
            plt.gca().add_artist(circle2b)
            plt.scatter(clothoid_paths[selectIndex][5][2][0],clothoid_paths[selectIndex][5][2][1])

            circle3 = plt.Circle((clothoid_paths[selectIndex][5][3][0],clothoid_paths[selectIndex][5][3][1]),clothoid_paths[selectIndex][5][3][2],color='magenta',alpha=0.2)
            plt.gca().add_artist(circle3)
            plt.scatter(clothoid_paths[selectIndex][5][3][0],clothoid_paths[selectIndex][5][3][1])

            for ptList in clothoid_paths[selectIndex][3]:
                NewPath.append([ptList[1], ptList[0]])

            plt.plot([pt[1] for pt in NewPath], [pt[0] for pt in NewPath], c='green', marker='o',markersize=5)

            plt.plot(uav1.ys, uav1.xs, c='r', marker='o' )
            plt.plot([pt[1] for pt in Path], [pt[0] for pt in Path], c='b', marker='.')
            plt.plot( activeWP[1], activeWP[0], c='k', marker='X', markersize = 5 )
            plt.axis('equal')
            plt.grid(True)
            plt.ylim(45.3, 45.45) #(45.0, 45.5)
            plt.xlim(-121.0, -120.5)   
            plt.show(100)

    for pts in Path:
            # plot circles around each waypoint - viusal aid for waypoint updating
            wptCircle = plt.Circle((pts[1], pts[0]), wptRad, color='green', alpha=0.2)
            plt.gca().add_artist(wptCircle)
            plt.scatter(pts[1],pts[0])

    if wpList !=None:
        for pts in NewPath:
                wptCircle = plt.Circle((pts[1], pts[0]), wptRad, color='green', alpha=0.2)
                plt.gca().add_artist(wptCircle)
                plt.scatter(pts[1],pts[0])
                
        # plt.show(50)
    
    # if wpList !=None:
    #     # Continue plotting turn radius circles
    #     circle = plt.Circle((circle_list[0][0],circle_list[0][1]),circle_list[0][2],color='magenta',alpha=0.2)
    #     plt.gca().add_artist(circle)
    #     plt.scatter(circle_list[0][0],circle_list[0][1])

    #     circle1 = plt.Circle((circle_list[1][0],circle_list[1][1]),circle_list[1][2],color='yellow',alpha=0.4)
    #     plt.gca().add_artist(circle1)
    #     plt.scatter(circle_list[1][0],circle_list[1][1])

    #     circle2 = plt.Circle((circle_list[2][0],circle_list[2][1]),circle_list[2][2],color='yellow',alpha=0.4)
    #     plt.gca().add_artist(circle2)
    #     plt.scatter(circle_list[2][0],circle_list[2][1])  

    #     circle3 = plt.Circle((circle_list[3][0],circle_list[3][1]),circle_list[3][2],color='magenta',alpha=0.2)
    #     plt.gca().add_artist(circle3)
    #     plt.scatter(circle_list[3][0],circle_list[3][1])



    if wpList !=None and hasPath == True and onlyOnce == False:
        # for ptList in wpList:
        #     for i in range(0, len(ptList[0])):
        #         NewPath.append([ptList[1][i], ptList[0][i]])
        onlyOnce = True
        uav1.setWaypoints(newwps=NewPath, newradius = wptRad )

    plt.plot(uav1.ys, uav1.xs, c='r', marker='o' )

    if wpList != None:
        plt.plot([pt[1] for pt in NewPath], [pt[0] for pt in NewPath], c='green', marker='o',markersize=5)

    # plt.plot([pt[1] for pt in NewPath], [pt[0] for pt in NewPath], c='g', marker='o')
    plt.plot([pt[1] for pt in Path], [pt[0] for pt in Path], c='b', marker='.')
    plt.plot( activeWP[1], activeWP[0], c='k', marker='X', markersize = 5 )

    plt.axis('equal')
    plt.grid(True)
    plt.ylim(45.3, 45.45) #(45.0, 45.5)
    plt.xlim(-121.0, -120.5)  
    # plt.ylim((uav1.x - 0.01, uav1.x + 0.01))
    # plt.xlim((uav1.y - 0.01, uav1.y + 0.01))

    plt.pause(0.01)  
    # if step == 140:
    #     plt.show(100) 

    print('Step: ' + str(step) + '\tCurrent wpt: ' + str(uav1.currentWPIndex) + '\tUAV Heading(deg): ' + str(round(uav1.heading,3)) + '(' + str(np.degrees(round(uav1.heading,3))) + ')') 

    if uav1.currentWPIndex == (len(NewPath)-1) and hasPath == True:
        uav1.setWaypoints(newwps=Path, newradius = wptRad )
        uav1.currentWPIndex = index
        hasPath = False
        #plt.pause(30) 

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

