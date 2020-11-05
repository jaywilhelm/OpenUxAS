# Author(s):
#           Jacob English, je787413@ohio.edu (Original)
#           Jay Wilhelm, jwilhelm@ohio.edu
#           Jeremy Browne jb780612@ohio.edu
# Contains A* replan logic used to produce collision avoidance 
# paths in Return-to-route UAV CAS system. 
############################################


import math
import matplotlib.pyplot as plt
import numpy as np
from scipy import stats

from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

from AStar import a_star_planning
from UAVHcfg import *
from TerminalColors import TerminalColors as TC
from uavData import UAV_TYPE, UAV_AVOID
import os

import pickle

def distance( a, b):
    return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)



def convertPathToUniqueWaypoints(path_x, path_y):
    path_x = np.array(path_x)
    path_y = np.array(path_y)
    # waypoints come out goal first
    path_x = np.flip(path_x)
    path_y = np.flip(path_y)

    psize = len(path_x)
    waypoints = np.array([[path_x[0], path_y[0]]])
    for i in range(2, psize):
        x = [path_x[i - 2], path_x[i - 1]]
        y = [path_y[i - 2], path_y[i - 1]]
        slope, intercept, r_value, p_value, std_err = stats.linregress(x, y)

        # print("slope: "+str(slope)+ "\t Intercept:"+str(intercept))
        testy = slope * path_x[i] + intercept
        # print("testy :" + str(testy) + "y: " + str(path_y[i]))
        if (np.isnan(slope) and path_x[i] == path_x[i - 1]):
            # print("x still on line")
            continue
        elif (np.isnan(slope) and path_y[i] == path_y[i - 1]):
            # print("y still on line")
            continue
        elif (testy == path_y[i]):
            # print("same " + str(i))
            continue
        else:
            # print("diff " + str(i))
            waypoints = np.concatenate((waypoints, [[path_x[i - 1], path_y[i - 1]]]), axis=0)
    return waypoints

'''
AvoidanceLogic Function: scale_border
    Parameters:
                border: List of points to define search
                        border for A*,
                center: center point of border region,
                offset: value to offset border by
    Description:
                Returns the list points to define the scaled border.
'''
def scale_border(border, center, offset):
    for pt in border:
        if pt[0] > center[0]:
            pt[0] += offset
        else:
            pt[0] -= offset
        if pt[1] > center[1]: 
            pt[1] += offset
        else:
            pt[1] -= offset
    return border

'''
AvoidanceLogic Function: findMidpoint
    Parameters:
                a: first point (x,y),
                b: second point (x,y)
    Description:
                Returns the midpoint of a and b
'''
def findMidpoint(a, b):
    a = (float(a[0]), float(a[1]))
    b = (float(b[0]), float(b[1]))
    return [ (a[0]+b[0])/2, (a[1]+b[1])/2 ]
    
'''
AvoidanceLogic Function: intermediates
    Parameters:
                p1: first point (x,y),
                p2: end point (x,y),
                interval: distance between points on line
    Description:
                Returns the list of points spaced between
                p1 and p2.
'''
def intermediates(p1, p2, interval):
    """ Credit:
        https://stackoverflow.com/questions/43594646/how-to-calculate-the-coordinates-of-the-line-between-two-points-in-python
    """
    nb_points = int(distance(p1, p2) / interval)

    x_spacing = (p2[0] - p1[0]) / (nb_points + 1)
    y_spacing = (p2[1] - p1[1]) / (nb_points + 1)

    return [[p1[0] + i * x_spacing, p1[1] +  i * y_spacing] 
        for i in range(1, nb_points+1)]

'''
    AvoidanceLogic Function: distance
    Parameters:
                a: point (x, y),
                b: point (x, y)
    Description:
                Returns the distance from point a to b.
'''


def make_uavtoavoid_koz(kozList, scalef, zero_pos):
    newkoz = np.array([])
    firstpt = True        
    for pts in kozList:
        #print('orriginal: ' + str(pts))
        #print('orriginal*50: ' + '[' + str(pts[0]*scalef) + ', ' + str(pts[1]*scalef) + ']')
        pts = convertToScaleInt(pts, scalef)
        #print('pts: ' + str(pts))
        pts -= zero_pos
        #print('zeroed pts: ' + str(pts))

        if(firstpt):
            firstpt = False
            #print('First pts: ' + str(pts))
        else:
            #print('Lastpt: ' + str(lastpt))
            newset = intermediates(lastpt, pts, 1)
            #print('Newset: ' + str(newset))
            for i in newset:
                #print('Points: ' + str(i) + ' in newset')
                fo = np.array([[i[0]], [i[1]]])
                # print('fo: ' + str(fo))
                if(len(newkoz) == 0):
                    newkoz = fo
                else:
                    newkoz = np.concatenate((newkoz, fo), axis=1)

        lastpt = pts

    return np.transpose(newkoz)

def convertToScaleInt(item, scalef):
    newitem = []
    for i in item:
        i = int(i*scalef)
        newitem.append(i)
    return newitem

def make_border_cells(mypos, scalef, offset):
    border_pts = np.array([[mypos[0] - offset, mypos[1] - offset],
                        [mypos[0] + offset, mypos[1] + offset]])
    border_pts[0] = convertToScaleInt(border_pts[0], scalef)
    border_pts[1] = convertToScaleInt(border_pts[1], scalef)
    zero_x = border_pts[0][0]
    zero_y = border_pts[0][1]
    zero_pos = np.array([zero_x, zero_y])
    border_pts -= zero_pos

    # bottom-left to br
    border_fill = np.arange(border_pts[0][0], border_pts[1][0], 1) # increment from 0-50 in intervals of 1
    bsize = len(border_fill)  # will be square
    border_cell = [border_fill, np.ones(bsize) * border_pts[0][1]]  # outer border

    border_fills = np.arange(border_pts[0][0], border_pts[1][0], 1) # increment from 0-50 in intervals of 1
    border_cells = [border_fill, np.ones(bsize) * border_pts[0][1]] # inner grid points

    for pt in border_fills:
        border_fills= np.array([np.ones(bsize) * pt, 
                            np.arange(border_pts[0][1], border_pts[1][1], 1)])
        border_cells = np.concatenate((border_cells, border_fills), axis=1)

    # bottom-right to top-right
    border_fill = np.array([np.ones(bsize) * border_pts[1][0],
                            np.arange(border_pts[0][1], border_pts[1][1], 1)])
    border_cell = np.concatenate((border_cell, border_fill), axis=1)
    # top-right to top-left
    border_fill = np.array([np.arange(border_pts[1][0], border_pts[0][0], -1),
                            np.ones(bsize) * border_pts[1][1]])
    border_cell = np.concatenate((border_cell, border_fill), axis=1)
    # top-left to bottom-left
    border_fill = np.array([np.ones(bsize) * border_pts[0][0],
                            np.arange(border_pts[1][1], border_pts[0][1], -1), ])
    border_cell = np.concatenate((border_cell, border_fill), axis=1)

    border_cell = np.transpose(border_cell)
    border_cells = np.transpose(border_cells)

    return border_cell, zero_pos, border_cells


'''
AvoidanceLogic Function: format_astar_input
    Parameters:
                koz: area points list to avoid from 
                        other UAV
    Description:
                Returns formatted data for A*:
                    - Start Position
                    - Goal Position
                    - Border for Search Area
                    - KeepOut Zone Points for other UAV


                    - use_pseudo_target
'''

def format_astar_input(mainUAV, kozList, scalef, AstarGoal, simStep):
    mypos = [mainUAV.position[0], mainUAV.position[1]]
    mygoal = [AstarGoal[0], AstarGoal[1]]

    # print('MyGoal: ' + str(mygoal))

    offset = 0.25 #in deg?
    #scalef = 50#10e7

    border_pts, zero_pos, border_cells = make_border_cells(mypos, scalef, offset)
    mypos = convertToScaleInt(mypos, scalef) 
    mygoal = convertToScaleInt(mygoal, scalef)

    mypos = np.array(mypos) - zero_pos
    mygoal = np.array(mygoal) - zero_pos

    newkoz = []
    for koz in kozList:
        temp = make_uavtoavoid_koz(koz, scalef, zero_pos)
        if (len(newkoz) == 0):
            newkoz = [temp.tolist()]
        else:
            newkoz.append(temp.tolist())

    start_pt = mypos
    goal_pt = mygoal
    koz_pts = newkoz

    showFormatImage = True
    if showFormatImage:
        plt.clf()
        fig, ax = plt.subplots()
        #ax.plot(t, s)
        ax.scatter(mypos[0], mypos[1], c='r')
        ax.scatter(mygoal[0], mygoal[1], c='b')
        ax.scatter([pt[0] for pt in border_pts], [pt[1] for pt in border_pts], c='b')
        ax.scatter([pt[0] for pt in border_cells], [pt[1] for pt in border_cells], c='k', marker='.')
        color = ['g', 'y', 'm']
        cc = 0
        for koz in newkoz:
            for pt in koz:
                print(pt)
            ax.scatter([pt[0] for pt in koz], [pt[1] for pt in koz], c = color[cc] )
            cc +=1

        plt.axis('equal')
        fig.set_size_inches((12, 10)) 
        ax.set(xlabel='x', ylabel='y',
            title='A* formatted map')
        ax.grid()
        
        wd = os.getcwd()
        path=(wd + '/RaceTrack_AstarFormatedInput')
        FormatedInput = 'FormatedInput%03d.png' % simStep
        FormatedInput = os.path.join(path,FormatedInput)
        plt.savefig(FormatedInput)
        plt.close('A* formatted map')
        plt.show()
        plt.clf()
        #fig.savefig("test.png")
        
        # plt.pause(1)
    return start_pt, goal_pt, border_pts, koz_pts, zero_pos, border_cells

def flightProjection(mainUAV, dt, lookAhead_time):
    time = np.arange(0, lookAhead_time, dt)
    projectionRange = np.arange(-mainUAV.data['turnRate']*1, mainUAV.data['turnRate']*1, np.deg2rad(1))

    save = []
    projectionPts = [list(mainUAV.position)]
    for i in range(0, len(projectionRange)):
        x=mainUAV.position[0]
        y=mainUAV.position[1]
        projectionHeading = mainUAV.heading
        for j in range(0, len(time)):
            projectionHeading = projectionHeading + projectionRange[i]*dt
            
            vx = mainUAV.velocity * np.cos(projectionHeading)
            vy = mainUAV.velocity* np.sin(projectionHeading)
            x = x + vx * dt
            y = y + vy * dt

            save.append([x, y])
        
        if i == 0:
            linX = np.linspace(mainUAV.position[0], x, int(len(projectionRange)/2), endpoint=False )
            liny = np.linspace(mainUAV.position[1], y, int(len(projectionRange)/2), endpoint=False ) 
            for k in range(0, len(linX)):     
                projectionPts.append([linX[k],liny[k]])

        elif i == len(projectionRange)-1:
            linX = np.linspace(mainUAV.position[0], x, int(len(projectionRange)/2), endpoint=False )
            liny = np.linspace(mainUAV.position[1], y, int(len(projectionRange)/2), endpoint=False )     
            for k in range(0, len(linX)):     
                projectionPts.append([linX[k],liny[k]]) 

        projectionPts.append([x,y])

    # plt.scatter([pt[1] for pt in save], [pt[0] for pt in save])
    # plt.scatter([pt[1] for pt in edgePts], [pt[0] for pt in edgePts])

    # plt.axis('equal')
    # plt.grid(True)
    # plt.show()

    return projectionPts
 
 

def reverseKOZ(mainUAV):
    ''' Keep out zone placed behind the CAS UAV
        Prevents A* from generating a path going backwards '''

    area_length = 0.00025 # !!!! REPALCE with look ahead based on time


    # Including the CAS UAVs position in the reverse koz prevents an A* solution
    # The "offset" offsets the koz off of the UAV current position
    d=0.000075 # Changes the distance the reverse koz is placed behind the UAV
    offsetPos = [0,0]
    offsetPos[0] = mainUAV.position[0] - d*np.cos(mainUAV.heading)
    offsetPos[1] = mainUAV.position[1] - d*np.sin(mainUAV.heading)

    # print('offset: ' + str(offsetPos))
    points = [list(offsetPos)] 

    # Generate two sets of points to make an arc behind the CAS UAV
    # thetaPossible and alpha change the arc lenght of the CAS UAV reverse KOZ
    theta = np.deg2rad(45)
    alpha = 4 # Changes the arc lengh of the CAS UAV koz
    for div in range(2, 5, 1):
        pt_x = offsetPos[0] - (area_length * np.cos(mainUAV.heading - (theta*alpha / div)))
        pt_y = offsetPos[1] - (area_length * np.sin(mainUAV.heading - (theta*alpha / div)))
        points.append([pt_x, pt_y])

    # +-0
    pt_x = offsetPos[0] - (area_length * np.cos(mainUAV.heading))
    pt_y = offsetPos[1] - (area_length * np.sin(mainUAV.heading))
    points.append([pt_x, pt_y]) 

    for div in range(-4, -1, 1):
        pt_x = offsetPos[0] - (area_length * np.cos(mainUAV.heading - (theta*alpha / div)))
        pt_y = offsetPos[1] - (area_length * np.sin(mainUAV.heading - (theta*alpha / div)))
        points.append([pt_x, pt_y])

    points.append(offsetPos)

    return points


def avoidCheck(mainUAV, otherUAVs, additionalKOZ, lookAhead_time):
    
    if mainUAV.uavType != UAV_TYPE.DUBINS:
        dt = 0.1
    else:
        dt = mainUAV.data['dt']

    CollisionUavIDs =[] # which UAVs have a potential collision?
    mypot_area = flightProjection(mainUAV, dt, lookAhead_time)

    mypoly = Polygon(mypot_area)

    PointInPoint = False 
    # Include other keep out zones not associated with other UAVs 
    avoid_areas = additionalKOZ[:] # '[:]' removes python references to static_koz, so I can make a copy of that list

    # if mainUAV.avoidance == UAV_AVOID.ACTIVE:
    #     avoid_areas.append(reverseKOZ(mainUAV))
    
    index = 0
    for otherUAV in otherUAVs:
        if otherUAV.uavType != UAV_TYPE.DUBINS:
            dt = 0.1
        else:
            dt = otherUAV.data['dt']

        thier_area = flightProjection(otherUAV, dt, lookAhead_time)
        thier_poly = Polygon(thier_area)
        if(thier_poly.intersects(mypoly)):
            PointInPoint = True
            avoid_areas.append(thier_area)
            # print("Pot. Collision ")
            # break
        index +=1

    return PointInPoint, avoid_areas

def avoid( mainUAV, otherUAVs, avoid_areas= [], astarGoalPt= [],  simStep=0):
    print(TC.WARNING + 'AVOID.' + TC.ENDC)

    AstarFailure = False
    static_koz = []

    # Get optimal path to destination
    # Format uav data for A* input
    # And prepare koz points
    if not len(astarGoalPt) > 0:
        print(TC.WARNING + 'Collisions detected but no A* goal point provided' + TC.ENDC)

    scaleFactor0 = 1 # (was 100) A* uses integers - most of the position info in in the Long/Lat decimal places
    if len(avoid_areas) > 0:
        for i in range(0, len(avoid_areas)):
            for j in range(0,len(avoid_areas[i])):
                avoid_areas[i][j] = [avoid_areas[i][j][0]*scaleFactor0,avoid_areas[i][j][1]*scaleFactor0]
    else:
        print(TC.WARNING + 'Collisions detected but no keep out zones' + TC.ENDC)


    astarGoalPt = [astarGoalPt[0]*scaleFactor0, astarGoalPt[1]*scaleFactor0]
    mainUAV.position = [mainUAV.position[0]*scaleFactor0, mainUAV.position[1]*scaleFactor0]

    scalefactor1 = 50 # 100 gives a 50x50 ob map; (was 200) 75? lager value => more dense A* problem space (increased resolution)
    start, goal, border, koz, offset, border_cells = format_astar_input(mainUAV, avoid_areas, scalefactor1, astarGoalPt, simStep)

    mainUAV.position = [mainUAV.position[0]/scaleFactor0, mainUAV.position[1]/scaleFactor0]

    pickle.dump({"start": start,
                "goal": goal,
                "border": border,
                "cells": border_cells,
                "koz": koz,
                "scale": scalefactor1,
                "offset": offset}, open('astar_Formated_Input.p', 'wb'))

    ox, oy = [], []
    for pt in border:
        ox.append(pt[0])
        oy.append(pt[1])

    for pts in koz:
        for pt in pts:
            ox.append(pt[0])
            oy.append(pt[1])

    try:
        INTERVAL_SIZE = 1
        path_x, path_y, obmap, nstart, ngoal = a_star_planning(start[0], start[1],
                                            goal[0], goal[1],
                                            ox, oy,
                                            INTERVAL_SIZE, (2 * INTERVAL_SIZE))
        pickle.dump({"start": start,
                    "goal": goal,
                    "ox": ox,
                    "oy": oy,
                    "INTERVAL_SIZE": INTERVAL_SIZE,
                    "path_x": path_x,
                    "path_y": path_y}, open('astar_result.p', 'wb'))
        AstarFail = False

    except ValueError:
        print(TC.FAIL + '\t\t**No valid path found.**' + TC.ENDC)
        for i in range(0, len(avoid_areas)):
            for j in range(0,len(avoid_areas[i])):
                avoid_areas[i][j] = [avoid_areas[i][j][0]/scaleFactor0,avoid_areas[i][j][1]/scaleFactor0]
        #plt.close('all')
        plt.clf()
        AstarFail = True
        return False, [], avoid_areas, [], AstarFail

    waypoints = convertPathToUniqueWaypoints(path_x, path_y)
    waypoints += offset
    waypoints /= scalefactor1
    waypoints /= scaleFactor0

    full_path = np.transpose(np.array([path_x, path_y]))
    full_path += offset
    full_path /= scalefactor1
    full_path /= scaleFactor0

    

    showAstarPath = False
    ### v== Plot Astar result ==v ###
    if showAstarPath:
        fig, ax = plt.subplots()
        #ax.plot(t, s)
        ax.scatter(start[1], start[0], c='r')
        ax.scatter([path_y], [path_x], c='m')
        ax.scatter(goal[1], goal[0], c='b')

        ax.scatter([pt[1] for pt in border], [pt[0] for pt in border], c='b')
        ax.scatter([pt[1] for pt in border_cells], [pt[0] for pt in border_cells], c='k', marker='.')
        color = ['g', 'y', 'm']
        cc = 0
        for x in koz:
            for pt in x:
                print(pt)
            ax.scatter([pt[1] for pt in x], [pt[0] for pt in x], c = color[cc] )
            cc +=1

        plt.axis('equal')
        fig.set_size_inches((12, 10)) 
        ax.set(xlabel='Lon', ylabel='Lat',
            title='A* Result')
        ax.grid()
        
        #fig.savefig("test.png")
        # plt.show()
        # plt.pause(1)
                    
        wd = os.getcwd()
        path=(wd + '/RaceTrack_AstarResults')
        AstarResult = 'AstarResult%03d.png' % simStep
        AstarResult = os.path.join(path, AstarResult)
        plt.savefig(AstarResult)
        plt.close('A* Result')
        # plt.show()
        plt.clf()



    plt.clf()
    return True, waypoints, avoid_areas, full_path, AstarFail
