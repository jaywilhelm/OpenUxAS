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
wptRad = 0.005
np.deg2rad(30)
thetaRef = np.deg2rad(270)
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
usePath = 'Line'
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
wpList3 = None
hasPath1 = False
hasPath2 = False
hasPath3 = False


if UseSingleUAV == True:
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
            # convert uav NWD angle convention to cartesion for clothoid
            # uavHeading = uav1.heading + np.radians(90)
            print('uav1 pos: ' + str(uav1.x) + ' ' + str(uav1.y))
            print('uav1 wpt: ' + str(activeWP[0]) + ' ' + str(activeWP[1]))

            # This hurts my head, but use the x's in the numerator and y's in the denom, then add 180 deg
            uavHeading = np.arctan2(uav1.x - activeWP[0], uav1.y - activeWP[1]) + np.radians(180)
            print('uav1 heading: ' + str(np.degrees(uavHeading)))
            if(uavHeading >= np.pi*2):
                uavHeading -= np.pi*2
            if(uavHeading < 0):
                uavHeading += np.pi*2
            print('uav1 heading: ' + str(np.degrees(uavHeading)))

            targetHeading = (np.radians(180))

            #index = 5 # which waypoint on the reference path to attach to
            a = 10    # how many waypoints per clothoid will be use for path follwing ( 3 clothoids needed for a solution)
            NewPath = []
            for index in range(4, 7):
                targetWPT = [Path[index][0], Path[index][1], Headings[index]]

                if(targetWPT[2] >= np.pi*2):
                    targetWPT[2] -= np.pi*2
                if(targetWPT[2] < 0):
                    targetWPT[2] += np.pi*2
                print('Target Heading: ' + str(np.degrees(targetWPT[2])))

                #clothoid0 = Clothoid.G1Hermite(uav1.y, uav1.x, uavHeading, targetWPT[1], targetWPT[0], targetWPT[2]) # Line: -121.13999999999999, 45.35,
                #x0, y0, t0, k0, dk, s = clothoid0.Parameters
                #print('x0: ' + str(x0) + '\ty0: ' + str(y0) + '\tk0: ' + str(k0) + '\tdk: ' + str(dk) + '\ts: ' + str(s))
                
                # clothoid1 = Clothoid.StandardParams(x0, y0, t0, 0, dk, s)

                clothoid_list = pyclothoids.SolveG2(uav1.y, uav1.x, uavHeading, np.radians(0), targetWPT[1], targetWPT[0], targetWPT[2], np.radians(0)) # stiches a path of multiple clothoids
                circle_list = []
                wpList = []
                wpList2 = []
                jj = 0
                for i in clothoid_list:
                    plt.plot(*i.SampleXY(500)) # plot 500 points for each clothoid
                    points = i.SampleXY(a)
                    plt.scatter( *i.SampleXY(a) )
                    wpList.append(points)
                    x0, y0, t0, k0, dk, s = i.Parameters     # start x, start y, initial curvature, change in curvature, lenght?

                    # Check Distance of each path
                    distPath = []
                    for j in range(0, len(points[0])):
                            distPath.append([points[1][j], points[0][j]])
                    
                    clothoidLength = 0
                    for j in range(0, len(distPath)-1):
                            wpt0 = distPath[j]
                            wpt1 = distPath[j+1]
                            dist = np.sqrt((wpt1[0] - wpt0[0])**2 + (wpt1[1] - wpt0[1])**2)
                            clothoidLength += dist
                    
                    print('Path Distance: ' + str(clothoidLength))

                    b = int(len(points[0])/2)
                    c = len(points[0])-1
                    if jj == 0:
                        wpList2.append([points[0][0],points[1][0]])
                    elif jj == 1:
                        wpList2.append([points[0][b],points[1][b]])
                    elif jj == 2:
                        wpList2.append([points[0][c],points[1][c]])

                    # Calculate Turn Radius of clothoid
                    # Full Set
                    x_points = points[0]
                    y_points = points[1]

                    if jj == 1:
                        #First Half
                        halfX_points1 = x_points[:(int(len(x_points)/2))]   
                        halfY_points1 = y_points[:(int(len(y_points)/2))]  

                        half_xc1,half_yc1,half_r1 = fit_circle_2d(halfX_points1, halfY_points1)
                        circle1 = plt.Circle((half_xc1,half_yc1),half_r1,color='yellow',alpha=0.4)
                        plt.gca().add_artist(circle1)
                        plt.scatter(half_xc1,half_yc1)
                        print('1st Center X: ' + str(half_xc1) + '\t1st Center Y: ' + str(half_yc1) + '\t1st Radius: ' + str(half_r1))

                        #Second Half
                        halfX_points2 = x_points[(int(len(x_points)/2)):]   
                        halfY_points2 = y_points[(int(len(y_points)/2)):] 

                        half_xc2,half_yc2,half_r2 = fit_circle_2d(halfX_points2, halfY_points2)
                        circle2 = plt.Circle((half_xc2,half_yc2),half_r2,color='yellow',alpha=0.4)
                        plt.gca().add_artist(circle2)
                        plt.scatter(half_xc2,half_yc2)
                        print('2nd Center X: ' + str(half_xc2) + '\t2nd Center Y: ' + str(half_yc2) + '\t2nd Radius: ' + str(half_r2))
                        
                        circle_list.append([half_xc1, half_yc1, half_r1])
                        circle_list.append([half_xc2, half_yc2, half_r2])

                    else:
                        xc,yc,r = fit_circle_2d(x_points, y_points)
                        circle = plt.Circle((xc,yc),r,color='magenta',alpha=0.2)
                        plt.gca().add_artist(circle)
                        plt.scatter(xc,yc)
                        print('Center X: ' + str(xc) + '\tCenter Y: ' + str(yc) + '\tRadius: ' + str(r))
                        circle_list.append([xc,yc,r])

                    jj+=1
                    # print('WP: ' + str(index) + ' x0: ' + str(round(x0,3)) + 
                    #     ' y0: ' + str(round(y0,2)) + '\tk0: ' + str(round(k0,2)) + 
                    #     '\t\tdk: ' + str(round(dk,2)) + '\t\ts: ' + str(round(s,2)))
                
                
                plt.plot(uav1.ys, uav1.xs, c='r', marker='o' )
                plt.plot([pt[1] for pt in Path], [pt[0] for pt in Path], c='b', marker='.')
                plt.plot( activeWP[1], activeWP[0], c='k', marker='X', markersize = 5 )
                plt.axis('equal')
                plt.grid(True)
                # plt.ylim(45.3, 45.45) #(45.0, 45.5)
                # plt.xlim(-121.0, -120.5)   

                plt.ylim((uav1.x - 0.01, uav1.x + 0.01))
                plt.xlim((uav1.y - 0.01, uav1.y + 0.01))
                plt.show(100)

                for ptList in wpList:
                    for i in range(0, len(ptList[0])):
                        NewPath.append([ptList[1][i], ptList[0][i]])

                clothoidLength = 0
                for j in range(0, len(NewPath)-1):
                        wpt0 = NewPath[j]
                        wpt1 = NewPath[j+1]
                        dist = np.sqrt((wpt1[0] - wpt0[0])**2 + (wpt1[1] - wpt0[1])**2)
                        clothoidLength += dist

                print('Total Path Distance: ' + str(clothoidLength))



                for ptList in wpList2:
                    NewPath2.append([ptList[1], ptList[0]])

                LastPt = [NewPath[len(NewPath)-1][0], NewPath[len(NewPath)-1][1]]
                next2LastPt = [NewPath[len(NewPath)-2][0], NewPath[len(NewPath)-2][1]]
                next3LastPt = [NewPath[len(NewPath)-3][0], NewPath[len(NewPath)-3][1]]

                # Angle between the last clothoid point and the next point on the target path. 
                check_InjectionHeading1 = np.arctan2(Path[index+1][1]-LastPt[1], Path[index+1][0]-LastPt[0])
                if(check_InjectionHeading1 >= np.pi*2):
                    check_InjectionHeading1 -= np.pi*2
                if(check_InjectionHeading1 < 0):
                    check_InjectionHeading1 += np.pi*2

                # Angle between the last and second to last clothoid point. 
                check_InjectionHeading2 = np.arctan2(LastPt[1]-next2LastPt[1], LastPt[0]-next2LastPt[0])
                if(check_InjectionHeading2 >= np.pi*2):
                    check_InjectionHeading2 -= np.pi*2
                if(check_InjectionHeading2 < 0):
                    check_InjectionHeading2 += np.pi*2

                # Angle between the second to last and third to last clothoid point. 
                check_InjectionHeading3 = np.arctan2(next2LastPt[1]-next3LastPt[1], next2LastPt[0]-next3LastPt[0])
                if(check_InjectionHeading3 >= np.pi*2):
                    check_InjectionHeading3 -= np.pi*2
                if(check_InjectionHeading3 < 0):
                    check_InjectionHeading3 += np.pi*2

                temp = abs(check_InjectionHeading2 - check_InjectionHeading1)
                if(temp >= np.pi*2):
                    temp -= np.pi*2
                if(temp < 0):
                    temp += np.pi*2

                temp2 = abs(check_InjectionHeading2 - check_InjectionHeading3)
                if(temp2 >= np.pi*2):
                    temp2 -= np.pi*2
                if(temp2 < 0):
                    temp2 += np.pi*2

                diff.append([check_InjectionHeading1, temp, check_InjectionHeading3, temp2])
                
                hasPath = True
            #plt.plot( *clothoid0.SampleXY(500) )
            #plt.plot( *clothoid1.SampleXY(500) )
        


        for pts in Path:
                wptCircle = plt.Circle((pts[1], pts[0]), wptRad, color='green', alpha=0.2)
                plt.gca().add_artist(wptCircle)
                plt.scatter(pts[1],pts[0])

        if wpList !=None:
            for pts in NewPath2:
                    wptCircle = plt.Circle((pts[1], pts[0]), wptRad, color='green', alpha=0.2)
                    plt.gca().add_artist(wptCircle)
                    plt.scatter(pts[1],pts[0])
                    
            # plt.show(50)
        
        if wpList !=None:
            # Continue plotting turn radius circles
            circle = plt.Circle((circle_list[0][0],circle_list[0][1]),circle_list[0][2],color='magenta',alpha=0.2)
            plt.gca().add_artist(circle)
            plt.scatter(circle_list[0][0],circle_list[0][1])

            circle1 = plt.Circle((circle_list[1][0],circle_list[1][1]),circle_list[1][2],color='yellow',alpha=0.4)
            plt.gca().add_artist(circle1)
            plt.scatter(circle_list[1][0],circle_list[1][1])

            circle2 = plt.Circle((circle_list[2][0],circle_list[2][1]),circle_list[2][2],color='yellow',alpha=0.4)
            plt.gca().add_artist(circle2)
            plt.scatter(circle_list[2][0],circle_list[2][1])  

            circle3 = plt.Circle((circle_list[3][0],circle_list[3][1]),circle_list[3][2],color='magenta',alpha=0.2)
            plt.gca().add_artist(circle3)
            plt.scatter(circle_list[3][0],circle_list[3][1])

 

        if wpList !=None and hasPath == True and onlyOnce == False:
            # for ptList in wpList:
            #     for i in range(0, len(ptList[0])):
            #         NewPath.append([ptList[1][i], ptList[0][i]])
            onlyOnce = True
            uav1.setWaypoints(newwps=NewPath2, newradius = wptRad )

        plt.plot(uav1.ys, uav1.xs, c='r', marker='o' )

        if wpList != None:
            for i in clothoid_list:
                plt.plot(*i.SampleXY(500), markersize=5) # plot 500 points for each clothoid
            #     plt.scatter( *i.SampleXY(a) )
            plt.plot([pt[1] for pt in NewPath2], [pt[0] for pt in NewPath2], c='green', marker='o',markersize=5)

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

        print('Step: ' + str(step) + '\tCurrent wpt: ' + str(uav1.currentWPIndex) + '\tUAV Heading: ' + str(uav1.heading) + '(' + str(np.degrees(uav1.heading)) + ')') 

        if step == 200:
            check = 1

        if uav1.currentWPIndex == (len(NewPath2)-1) and hasPath == True:
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

else:

    ## ============================================================================= ###
    # Make Composite Plost
    # Study waypoint target, # of points, and distance from path 
    # on curvature between clothoid segments and ability to follow path.

    TargetWpts = [3, 4 ,5 ,6]
    NumberOfWpts = [3, 6, 10]
    DistFromPath = [0.05, 0.1, 0.15]

    # cycle through vehicle distances
    #for i in range(0,1):
    # Create Dubins Vehicle
    dt = 0.25
    v = 0.01
    r = 0.3
    thetaRef = np.deg2rad(270)

    uav1 = dubinsUAV(position=[45.35 + DistFromPath[0], -120.5], velocity=v, heading=thetaRef, dt=dt)
    uav2 = dubinsUAV(position=[45.35 + DistFromPath[1], -120.5], velocity=v, heading=thetaRef, dt=dt)
    uav3 = dubinsUAV(position=[45.35 + DistFromPath[2], -120.5], velocity=v, heading=thetaRef, dt=dt)

    xy1 = (uav1.x, uav1.y)
    px1 = xy1[0] + r * np.cos(uav1.heading)
    py1 = xy1[1] + r * np.sin(uav1.heading)
    uav1.setWaypoints(newwps=[px1, py1], newradius = 0.01 )

    xy2 = (uav2.x, uav2.y)
    px2 = xy2[0] + r * np.cos(uav2.heading)
    py2 = xy2[1] + r * np.sin(uav2.heading)
    uav1.setWaypoints(newwps=[px2, py2], newradius = 0.01 )

    xy3 = (uav3.x, uav3.y)
    px3 = xy3[0] + r * np.cos(uav3.heading)
    py3 = xy3[1] + r * np.sin(uav3.heading)
    uav1.setWaypoints(newwps=[px3, py3], newradius = 0.01 )

    step = 0
    index = 4       # which waypoint on the reference path to attach to
    a = 6          # how many waypoints per clothoid will be use for path follwing ( 3 clothoids needed for a solution)

    # Line_MultiUAVClothoidsIndex3With3

    uav1ClothoidData = []
    uav2ClothoidData = []
    uav3ClothoidData = []
 
    while step < 250:

        # Update Dubins wp
        xy1 = (uav1.x, uav1.y)
        px1 = xy1[0] + r * np.cos(uav1.heading)
        py1 = xy1[1] + r * np.sin(uav1.heading)

        xy2 = (uav2.x, uav2.y)
        px2 = xy2[0] + r * np.cos(uav2.heading)
        py2 = xy2[1] + r * np.sin(uav2.heading)

        xy3 = (uav3.x, uav3.y)
        px3 = xy3[0] + r * np.cos(uav3.heading)
        py3 = xy3[1] + r * np.sin(uav3.heading)

        if wpList1 == None:
            uav1.setWaypoints(newwps=[[px1, py1]], newradius = 0.01 )
            uav2.setWaypoints(newwps=[[px2, py2]], newradius = 0.01 )
            uav3.setWaypoints(newwps=[[px3, py3]], newradius = 0.01 )

            uav1.simulateWPDubins(UseCarrotChase=False)
            uav2.simulateWPDubins(UseCarrotChase=False)
            uav3.simulateWPDubins(UseCarrotChase=False)

        else:
            uav1.simulateWPDubins(UseCarrotChase=False)
            uav2.simulateWPDubins(UseCarrotChase=False)
            uav3.simulateWPDubins(UseCarrotChase=False)

            carrot = uav1.CarrotChaseWP(delta = 0.01)
            # plotCarrot, = plt.plot(carrot[1], carrot[0], c='k', marker='^' )

        activeWP1 = uav1.getActiveWaypoint()
        activeWP2 = uav2.getActiveWaypoint()
        activeWP3 = uav3.getActiveWaypoint()

        if step == 50:
            # convert uav NWD angle convention to cartesion for clothoid
            # uavHeading = uav1.heading + np.radians(90)
            # print('uav1 pos: ' + str(uav1.x) + ' ' + str(uav1.y))
            # print('uav1 wpt: ' + str(activeWP[0]) + ' ' + str(activeWP[1]))

            # This hurts my head, but use the x's in the numerator and y's in the denom, then add 180 deg
            uavHeading1 = np.arctan2(uav1.x - activeWP1[0], uav1.y - activeWP1[1]) + np.radians(180)
            uavHeading2 = np.arctan2(uav2.x - activeWP2[0], uav2.y - activeWP2[1]) + np.radians(180)
            uavHeading3 = np.arctan2(uav3.x - activeWP3[0], uav3.y - activeWP3[1]) + np.radians(180)

            if(uavHeading1 >= np.pi*2):
                uavHeading1 -= np.pi*2
            if(uavHeading1 < 0):
                uavHeading1 += np.pi*2

            if(uavHeading2 >= np.pi*2):
                uavHeading2 -= np.pi*2
            if(uavHeading2 < 0):
                uavHeading2 += np.pi*2        
            
            if(uavHeading3 >= np.pi*2):
                uavHeading3 -= np.pi*2
            if(uavHeading3 < 0):
                uavHeading3 += np.pi*2

            print('uav1 heading: ' + str(np.degrees(uavHeading1)) + 
                ' uav2 heading: ' + str(np.degrees(uavHeading2)) + 
                ' uav3 heading: ' + str(np.degrees(uavHeading3)))

            targetWPT = [Path[index][0], Path[index][1], Headings[index]]

            if(targetWPT[2] >= np.pi*2):
                targetWPT[2] -= np.pi*2
            if(targetWPT[2] < 0):
                targetWPT[2] += np.pi*2
            print('Target Heading: ' + str(np.degrees(targetWPT[2])))

            clothoid_list1 = pyclothoids.SolveG2(uav1.y, uav1.x, uavHeading1, np.radians(0), targetWPT[1], targetWPT[0], targetWPT[2], np.radians(0)) # stiches a path of multiple clothoids
            clothoid_list2 = pyclothoids.SolveG2(uav2.y, uav2.x, uavHeading2, np.radians(0), targetWPT[1], targetWPT[0], targetWPT[2], np.radians(0)) # stiches a path of multiple clothoids
            clothoid_list3 = pyclothoids.SolveG2(uav3.y, uav3.x, uavHeading3, np.radians(0), targetWPT[1], targetWPT[0], targetWPT[2], np.radians(0)) # stiches a path of multiple clothoids

            # Generate Clothoid Paths =========================
            wpList1 = []
            for i in clothoid_list1:
                plt.plot(*i.SampleXY(500)) # plot 500 points for each clothoid
                points = i.SampleXY(a)
                plt.plot( *i.SampleXY(a) )
                wpList1.append(points)
                x0, y0, t0, k0, dk, s = i.Parameters     # start x, start y, initial curvature, change in curvature, lenght?
                uav1ClothoidData.append([x0, y0, t0, k0, dk, s])
                print('UAV1 WP: ' + str(index) + ' x0: ' + str(round(x0,3)) + 
                    ' y0: ' + str(round(y0,2)) + '\tk0: ' + str(round(k0,2)) + 
                    '\t\tdk: ' + str(round(dk,2)) + '\t\ts: ' + str(round(s,2)))

            for ptList in wpList1:
                for i in range(0, len(ptList[0])):
                    NewPath1.append([ptList[1][i], ptList[0][i]])

            wpList2 = []
            for i in clothoid_list2:
                plt.plot(*i.SampleXY(500)) # plot 500 points for each clothoid
                points = i.SampleXY(a)
                plt.plot( *i.SampleXY(a) )
                wpList2.append(points)
                x0, y0, t0, k0, dk, s = i.Parameters     # start x, start y, initial curvature, change in curvature, lenght?
                uav2ClothoidData.append([x0, y0, t0, k0, dk, s])

                print('UAV2 WP: ' + str(index) + ' x0: ' + str(round(x0,3)) + 
                    ' y0: ' + str(round(y0,2)) + '\tk0: ' + str(round(k0,2)) + 
                    '\tdk: ' + str(round(dk,2)) + '\ts: ' + str(round(s,2)))

            for ptList in wpList2:
                for i in range(0, len(ptList[0])):
                    NewPath2.append([ptList[1][i], ptList[0][i]]) 

            wpList3 = []
            for i in clothoid_list3:
                plt.plot(*i.SampleXY(500)) # plot 500 points for each clothoid
                points = i.SampleXY(a)
                plt.plot( *i.SampleXY(a) )
                wpList3.append(points)
                x0, y0, t0, k0, dk, s = i.Parameters     # start x, start y, initial curvature, change in curvature, lenght?
                uav3ClothoidData.append([x0, y0, t0, k0, dk, s])

                print('UAV3 WP: ' + str(index) + ' x0: ' + str(round(x0,3)) + 
                    ' y0: ' + str(round(y0,2)) + '\tk0: ' + str(round(k0,2)) + 
                    '\t\tdk: ' + str(round(dk,2)) + '\t\ts: ' + str(round(s,2)))

            for ptList in wpList3:
                for i in range(0, len(ptList[0])):
                    NewPath3.append([ptList[1][i], ptList[0][i]])
            
            hasPath1 = True
            hasPath2 = True
            hasPath3 = True


        if wpList1 !=None and hasPath1 == True and onlyOnce == False:
            onlyOnce = True
            print('Follow Clothoid')
            uav1.setWaypoints(newwps=NewPath1, newradius = 0.01 )
            uav2.setWaypoints(newwps=NewPath2, newradius = 0.01 )
            uav3.setWaypoints(newwps=NewPath3, newradius = 0.01 )

        plt.plot(uav1.ys, uav1.xs, c='r', marker='o' )
        plt.plot(uav2.ys, uav2.xs, c='g', marker='o' )
        plt.plot(uav3.ys, uav3.xs, c='m', marker='o' )

        if wpList1 != None:
            for i in clothoid_list1:
                plt.plot(*i.SampleXY(500)) # plot 500 points for each clothoid
                plt.scatter( *i.SampleXY(a) )
            for i in clothoid_list2:
                plt.plot(*i.SampleXY(500)) # plot 500 points for each clothoid
                plt.scatter( *i.SampleXY(a) )
            for i in clothoid_list3:
                plt.plot(*i.SampleXY(500)) # plot 500 points for each clothoid
                plt.scatter( *i.SampleXY(a) )

        plt.plot([pt[1] for pt in Path], [pt[0] for pt in Path], c='b', marker='.')
        plt.plot( activeWP1[1], activeWP1[0], c='k', marker='X', markersize = 6 )
        plt.plot( activeWP2[1], activeWP2[0], c='k', marker='v', markersize = 6 )
        plt.plot( activeWP3[1], activeWP3[0], c='k', marker='^', markersize = 6 )

        plt.axis('equal')
        plt.grid(True)
        plt.ylim(45.3, 45.45) #(45.0, 45.5)
        plt.xlim(-121.0, -120.5)  
        plt.pause(0.005)  
    
        print('Step: ' + str(step) + '\tUAV1 WPT: ' + str(uav1.currentWPIndex) + 
                    '\tUAV2 WPT: ' + str(uav2.currentWPIndex) + '\tUAV3 WPT: ' + str(uav3.currentWPIndex))

        if uav1.currentWPIndex == (len(NewPath1)-1):
            check = 1

        if uav1.currentWPIndex == (len(NewPath1)-1) and hasPath1 == True:
            uav1.setWaypoints(newwps=Path, newradius = 0.1 )
            uav1.currentWPIndex = index
            hasPath1 = False
        if uav2.currentWPIndex == (len(NewPath2)-1) and hasPath2 == True:
            uav2.setWaypoints(newwps=Path, newradius = 0.1 )
            uav2.currentWPIndex = index
            hasPath2 = False   
        if uav3.currentWPIndex == (len(NewPath3)-1) and hasPath3 == True:
            uav3.setWaypoints(newwps=Path, newradius = 0.1 )
            uav3.currentWPIndex = index
            hasPath3 = False

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



# Open function to open the file "MyFile1.txt" 
# (same directory) in append mode and 
fileName = usePath + '_MultiUAVClothoidAtIndex' + str(index) + 'With' + str(a) + 'wpts'
file1 = open(fileName,"w") 

for i in uav1ClothoidData:
    file1.write('UAV: 1 \tTarget WP: ' + str(index) + ' No. of Wpts: ' + str(a) + ' Distance:  ' + str(DistFromPath[0]) + '\tt0: ' + str(round(i[2],2)) +
            '\tk0: ' + str(round(i[3],2)) + ' \tdk: ' + str(round(i[4],2)) + ' \ts: ' + str(round(i[5],2)) + '\n')
for i in uav2ClothoidData:
    file1.write('UAV: 2 \tTarget WP: ' + str(index) + ' No. of Wpts: ' + str(a) + ' Distance:  ' + str(DistFromPath[1]) + '\tt0: ' + str(round(i[2],2)) +
          '\tk0: ' + str(round(i[3],2)) + ' \tdk: ' + str(round(i[4],2)) + ' \ts: ' + str(round(i[5],2)) + '\n')
for i in uav3ClothoidData:
    file1.write('UAV: 3 \tTarget WP: ' + str(index) + ' No. of Wpts: ' + str(a) + ' Distance:  ' + str(DistFromPath[2]) + '\tt0: ' + str(round(i[2],2)) +
          '\tk0: ' + str(round(i[3],2)) + ' \tdk: ' + str(round(i[4],2)) + ' \ts: ' + str(round(i[5],2)) + '\n')

file1.close()

for i in uav1ClothoidData:
    print('UAV: 1 \tTarget WP: ' + str(index) + ' Wpts: ' + str(a) + ' Distance:  ' + str(DistFromPath[1]) + '\tt0: ' + str(round(i[2],2)) +
          '\tk0: ' + str(round(i[3],2)) + ' \tdk: ' + str(round(i[4],2)) + ' \ts: ' + str(round(i[5],2)))
print('\n')
for i in uav2ClothoidData:
    print('UAV: 2 \tTarget WP: ' + str(index) + ' Wpts: ' + str(a) + ' Distance:  ' + str(DistFromPath[2]) + '\tt0: ' + str(round(i[2],2)) +
          '\tk0: ' + str(round(i[3],2)) + ' \tdk: ' + str(round(i[4],2)) + ' \ts: ' + str(round(i[5],2)))
print('\n')
for i in uav3ClothoidData:
    print('UAV: 3 \tTarget WP: ' + str(index) + ' Wpts: ' + str(a) + ' Distance:  ' + str(DistFromPath[2]) + '\tt0: ' + str(round(i[2],2)) +
          '\tk0: ' + str(round(i[3],2)) + ' \tdk: ' + str(round(i[4],2)) + ' \ts: ' + str(round(i[5],2)))
print('\n')


print('Reverting to previous Directory')
os.chdir(wd)
print('\n')

