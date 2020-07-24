# File: UAVHeading.py
# Author(s):
#           Jacob English, je787413@ohio.edu (Original)
#           Jay Wilhelm, jwilhelm@ohio.edu
#           Jeremy Browne
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

import pickle
'''
 Class: UAVHeading
'''
class UAVHeading:
    position = []
    waypoint = []
    speed = 0
    time = 0
    thetaRef = 0
    thetaPossible = 0

    avoidanceUAV = False

    staticAreaLength = False
    shift_x = 0
    shift_y = 0
    lastClear = False

    '''
     UAVHeading Function: __init__
        Parameters: 
                    pos: UAV position (x, y),
                    waypt: UAV target position (x, y),
                    speed: UAV Speed (m/s),
                    heading: UAV heading (radians),
                    tPossible: possible turn angle for UAV (radians)
        Description:
                    Constructor for UAVHeading Class.
    '''
    def __init__(self, pos, waypt, speed, heading, tPossible, avoidanceUAV):
        self.position = list(pos)
        self.waypoint = list(waypt)
        self.speed = speed
        self.thetaRef = heading
        # self.thetaRef = 90 - heading
        self.thetaPossible = tPossible
        # self.staticAreaLength = False
        self.avoidanceUAV = avoidanceUAV

    '''
     UAVHeading Function: __weightedSideDecision
        Parameters:
                    uav0: UAVHeading running the avoidance function,
                    uav_others: list of other UAVHeading objects,
                    keepOutZones: list of keep out zone objects from scenario environment
        Description:
                    Returns a polygon defining the possible flight
                    area for the UAV calculated using the init values.
    '''
    def __weightedSideDecision(self, uav0, uav_others, keepOutZones):
        side_sum = 0

        if (math.radians(45) < self.thetaRef < math.radians(135)) or (math.radians(225) < self.thetaRef < math.radians(315)): # use y position difference
        
            side_sum += DECISION_WEIGHTS[0] * abs(self.position[1] - uav0.position[1])
            side_sum += DECISION_WEIGHTS[1] * abs(self.position[1] - uav0.waypoint[1])

            for uav in uav_others:
                side_sum -= DECISION_WEIGHTS[2] * abs(self.position[1] - uav.position[1])
            for koz in keepOutZones:
                kx = [pt[0] for pt in koz]
                ky = [pt[0] for pt in koz]
                centroid = (sum(kx) / len(koz), sum(ky) / len(koz))
                kPoly = Polygon(koz)
                side_sum -= DECISION_WEIGHTS[3] * kPoly.area * abs(self.position[1] - centroid[1])

            if abs(self.thetaRef - math.radians(90)) > abs(self.thetaRef - math.radians(270)):
                if side_sum > 0:
                    return 1
                else:
                    return -1
            else:
                if side_sum > 0:
                    return -1
                else:
                    return 1
        else: # use x position difference

            side_sum += DECISION_WEIGHTS[0] * abs(self.position[0] - uav0.position[0])
            side_sum += DECISION_WEIGHTS[1] * abs(self.position[0] - uav0.waypoint[0])

            for uav in uav_others:
                side_sum -= DECISION_WEIGHTS[2] * abs(self.position[0] - uav.position[0])
            for koz in keepOutZones:
                kx = [pt[0] for pt in koz]
                ky = [pt[0] for pt in koz]
                centroid = (sum(kx) / len(koz), sum(ky) / len(koz))
                kPoly = Polygon(koz)
                side_sum -= DECISION_WEIGHTS[3] * kPoly.area * abs(self.position[0] - centroid[0])

            if abs(self.thetaRef) > abs(self.thetaRef - math.radians(180)):
                if side_sum > 0:
                    return -1
                else:
                    return 1
            else:
                if side_sum > 0:
                    return 1
                else:
                    return -1

    def possibleFlightAreaStatic(self, area_length):
        theta_ref = self.thetaRef
        theta_possible = self.thetaPossible
        side_decision = 0

        points = [list(self.position)] 

        if self.staticAreaLength:
            area_length = self.staticAreaLength
            # side_decision = self.__weightedSideDecision(uav0, uavh_others, static_kozs) # stub uav_others and koz lists for now # Browne: commented our b/c Dubins UAV does not have a waypoint variable
            #
            # if side_decision < 0:
            #     points[-1][0] = self.position[0] + (3 * area_length * math.cos(theta_ref - (theta_possible / 2)))
            #     points[-1][1] = self.position[1] + (3 * area_length * math.sin(theta_ref - (theta_possible / 2)))
        
        for div in range(-2, -5, -1):
            pt_x = self.position[0] + (area_length * math.cos(theta_ref + (theta_possible / div)))
            pt_y = self.position[1] + (area_length * math.sin(theta_ref + (theta_possible / div)))
            points.append([pt_x, pt_y])

        # +-0
        pt_x = self.position[0] + (area_length * math.cos(theta_ref))
        pt_y = self.position[1] + (area_length * math.sin(theta_ref))
        points.append([pt_x, pt_y])

        for div in range(4, 1, -1):
            pt_x = self.position[0] + (area_length * math.cos(theta_ref + (theta_possible / div)))
            pt_y = self.position[1] + (area_length * math.sin(theta_ref + (theta_possible / div)))
            points.append([pt_x, pt_y])

        # ===========================

        if self.avoidanceUAV:
            points.append(list(self.position))

            for div in range(2, 5, 1):
                pt_x = self.position[0] - (area_length * math.cos(theta_ref - (theta_possible*3 / div)))
                pt_y = self.position[1] - (area_length * math.sin(theta_ref - (theta_possible*3 / div)))
                points.append([pt_x, pt_y])

            # +-0
            pt_x = self.position[0] - (area_length * math.cos(theta_ref))
            pt_y = self.position[1] - (area_length * math.sin(theta_ref))
            points.append([pt_x, pt_y]) 

            for div in range(-4, -1, 1):
                pt_x = self.position[0] - (area_length * math.cos(theta_ref - (theta_possible*3 / div)))
                pt_y = self.position[1] - (area_length * math.sin(theta_ref - (theta_possible*3 / div)))
                points.append([pt_x, pt_y])


        points.append(list(self.position))
        return points

    '''
     UAVHeading Function: possibleFlightArea
        Parameters: area_length
                    uav0            specific UAV to avoid
                    uavh_others     list of all other UAVs ???
                    static_kozs     keep out zone(s)
        Description:
                    Returns a polygon defining the possible flight
                    area for the UAV calculated using the init values.
    '''
    def possibleFlightArea(self, area_length, uav0, uavh_others, static_kozs):
        # theta_ref = math.radians(self.thetaRef)
        # theta_possible = math.radians(self.thetaPossible)
        theta_ref = self.thetaRef
        theta_possible = self.thetaPossible

        side_decision = 0

        points = [list(self.position)]

        if self.staticAreaLength:
            area_length = self.staticAreaLength
            # side_decision = self.__weightedSideDecision(uav0, uavh_others, static_kozs) # stub uav_others and koz lists for now # Browne: commented our b/c Dubins UAV does not have a waypoint variable
            #
            # if side_decision < 0:
            #     points[-1][0] = self.position[0] + (3 * area_length * math.cos(theta_ref - (theta_possible / 2)))
            #     points[-1][1] = self.position[1] + (3 * area_length * math.sin(theta_ref - (theta_possible / 2)))

        for div in range(-2, -5, -1):
            pt_x = self.position[0] + (area_length * math.cos(theta_ref + (theta_possible / div)))
            pt_y = self.position[1] + (area_length * math.sin(theta_ref + (theta_possible / div)))
            points.append([pt_x, pt_y])

        # +-0
        pt_x = self.position[0] + (area_length * math.cos(theta_ref))
        pt_y = self.position[1] + (area_length * math.sin(theta_ref))
        points.append([pt_x, pt_y])

        for div in range(4, 1, -1):
            pt_x = self.position[0] + (area_length * math.cos(theta_ref + (theta_possible / div)))
            pt_y = self.position[1] + (area_length * math.sin(theta_ref + (theta_possible / div)))
            points.append([pt_x, pt_y])

        # if self.staticAreaLength and side_decision > 0:
        #     points[-1][0] = self.position[0] + (2 * area_length * math.cos(theta_ref + (theta_possible / 2)))
        #     points[-1][1] = self.position[1] + (2 * area_length * math.sin(theta_ref + (theta_possible / 2)))

        points.append(list(self.position))

        # if uav0 is in possible flight area, recalculate with length/2
        pt = Point(uav0.position[0], uav0.position[1])
        koz_polygon = Polygon(points)
        if koz_polygon.contains(pt):
            print(TC.FAIL + '[HOTFIX - Line 152 | Area Length for Head-On Collision]' + TC.ENDC)
            if self.staticAreaLength:
                self.staticAreaLength = self.staticAreaLength / 2
            else:
                self.staticAreaLength = area_length / 4
            points = self.possibleFlightArea(area_length, uav0, uavh_others, static_kozs)

        return points

    '''
     UAVHeading Function: __lineIntersect
        Parameters:
                    line1: [(x0, y0), (x1, y1)],
                    line2: [(x0, y0), (x1, y1)]
        Description:
                    Returns intersection point (x, y) of two lines.
    '''
    def __lineIntersect(self, line1, line2):
        xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
        ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

        def det(a, b):
            return a[0] * b[1] - a[1] * b[0]

        div = det(xdiff, ydiff)
        if div == 0:
           raise ValueError('lines do not intersect')

        d = (det(*line1), det(*line2))
        x = det(d, xdiff) / div
        y = det(d, ydiff) / div
        return x, y

    '''
     UAVHeading Function: __distance
        Parameters:
                    a: point (x, y),
                    b: point (x, y)
        Description:
                    Returns the distance from point a to b.
    '''
    def __distance(self, a, b):
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    '''
     UAVHeading Function: __isBetween
        Parameters:
                    pt0: point (x, y),
                    intersect: point (x, y),
                    pt1: point (x, y),
        Description:
                    Returns True if the intersect point is on the
                    line segment defined by pt0 and pt1. If not,
                    the function returns False.
    '''
    def __isBetween(self, pt0, intersect, pt1):
        distAB = self.__distance(pt0, intersect)
        distBC = self.__distance(intersect, pt1)
        distAC = self.__distance(pt0, pt1)

        # triangle inequality
        return math.isclose((distAB + distBC), distAC)

    '''
     UAVHeading Function: __findIntersects
        Parameters:
                    uavh_other: UAVHeading object to avoid
        Description:
                    Finds intersection points between the
                    UAVHeading path and the possible flight area polygon
                    of the UAVHeading uavh_other.
                    Returns:
                        - Intersection list
                        - UAVHeading possible flight polygon point list
    '''
    def __findIntersects(self, uavh_others, static_kozs):
        intersects = []
        koz_areas = []
        if(not self.waypoint):
            xy = (self.position[0], self.position[1])
            r = 10
            px = xy[0] + r * np.sin(self.thetaRef)
            py = xy[1] + r * np.cos(self.thetaRef)
            self_line=[xy,(px,py)]
        else:
            self_line = [(self.position[0], self.position[1]), (self.waypoint[0], self.waypoint[1])]

        # check for potential UAV collisions
        for uavh_other in uavh_others:
            tmp_intersects = []
            other_area_points = []
            distance_to_other = self.__distance(self.position, uavh_other.position)

            if distance_to_other < DISTANCE_THRESHOLD:
                other_area_points = uavh_other.possibleFlightArea((2 * distance_to_other), self, uavh_others, static_kozs)
                for j in range(len(other_area_points) -1):
                    other_line = [other_area_points[j], other_area_points[j+1]]
                    try:
                        point = self.__lineIntersect(self_line, other_line)

                        if (self.__isBetween(self_line[0], point, self_line[1]) and self.__isBetween(other_line[0], point, other_line[1])):
                            tmp_intersects.append(point)
                    except ValueError:
                        continue

                koz_areas.append(other_area_points)
                intersects = intersects + tmp_intersects

        # check for potential static KoZ collisions
        for area in static_kozs:
            koz_areas.append(area)
            for i in range(len(area) - 1):
                other_line = [area[i], area[i+1]]
                try:
                    point = self.__lineIntersect(self_line, other_line)

                    if (self.__isBetween(self_line[0], point, self_line[1]) and self.__isBetween(other_line[0], point, other_line[1])):
                        intersects.append(point)
                        break
                except ValueError:
                    continue

        return intersects, koz_areas

    '''
    UAVHeading Function: __scale_border
        Parameters:
                    border: List of points to define search
                            border for A*,
                    center: center point of border region,
                    offset: value to offset border by
        Description:
                    Returns the list points to define the scaled border.
    '''
    def __scale_border(self, border, center, offset):
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
    UAVHeading Function: __intermediates
        Parameters:
                    p1: first point (x,y),
                    p2: end point (x,y),
                    interval: distance between points on line
        Description:
                    Returns the list of points spaced between
                    p1 and p2.
    '''
    def __intermediates(self, p1, p2, interval):
        """ Credit:
            https://stackoverflow.com/questions/43594646/how-to-calculate-the-coordinates-of-the-line-between-two-points-in-python
        """
        nb_points = int(self.__distance(p1, p2) / interval)

        x_spacing = (p2[0] - p1[0]) / (nb_points + 1)
        y_spacing = (p2[1] - p1[1]) / (nb_points + 1)

        return [[p1[0] + i * x_spacing, p1[1] +  i * y_spacing] 
            for i in range(1, nb_points+1)]

    '''
    UAVHeading Function: __midpoint
        Parameters:
                    a: first point (x,y),
                    b: second point (x,y)
        Description:
                    Returns the midpoint of a and b
    '''
    def __midpoint(self, a, b):
        a = (float(a[0]), float(a[1]))
        b = (float(b[0]), float(b[1]))
        return [ (a[0]+b[0])/2, (a[1]+b[1])/2 ]

    '''
    UAVHeading Function: __format_astar_input
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
    def __format_astar_input(self, kozList, staticAreaLength):
        if staticAreaLength:
            print(TC.OKBLUE + '\t<Using static avoid area length>' + TC.ENDC)
        
        # Make Border - find min and max for x and y values
        x_min, y_min = self.position[0], self.position[1]
        x_max, y_max = self.position[0], self.position[1]

        # pseudo_target = self.__midpoint(self.position, self.waypoint)

        # check if pseudo-target is reachable
        # pt = Point(pseudo_target[0], pseudo_target[1])
        # koz_polygon = Polygon(koz)
        # koz_scale = koz_polygon.buffer(2 * INTERVAL_SIZE) # buffer size for uav0 in A* search

        use_pseudo_target = False #not koz_scale.contains(pt)

        if not use_pseudo_target:# and not staticAreaLength:
            print(TC.OKBLUE + '\t<Using real target position>' + TC.ENDC)

            # compare with target position
            if x_min > self.waypoint[0]:
                x_min = self.waypoint[0]
            if y_min > self.waypoint[1]:
                y_min = self.waypoint[1]

            if x_max < self.waypoint[0]:
                x_max = self.waypoint[0]
            if y_max < self.waypoint[1]:
                y_max = self.waypoint[1]
        else:
            print(TC.OKBLUE + '\t<Using pseudo-target position>' + TC.ENDC)

            # compare with pseudo-target position
            if x_min > pseudo_target[0]:
                x_min = pseudo_target[0]
            if y_min > pseudo_target[1]:
                y_min = pseudo_target[1]

            if x_max < pseudo_target[0]:
                x_max = pseudo_target[0]
            if y_max < pseudo_target[1]:
                y_max = pseudo_target[1]

        if not staticAreaLength:
            # compare with uav other position
            if x_min > koz[0][0]:
                x_min = koz[0][0]
            if y_min > koz[0][1]:
                y_min = koz[0][1]

            if x_max < koz[0][0]:
                x_max = koz[0][0]
            if y_max < koz[0][1]:
                y_max = koz[0][1]
        else:
            # compare with all koz points
            for koz in kozList:
                for pt in koz:
                    if x_min > pt[0]:
                        x_min = pt[0]
                    if y_min > pt[1]:
                        y_min = pt[1]

                    if x_max < pt[0]:
                        x_max = pt[0]
                    if y_max < pt[1]:
                        y_max = pt[1]
        
        border_pts = [[x_max, y_max], 
                      [x_max, y_min],
                      [x_min, y_max],
                      [x_min, y_min]]

        # add padding to border
        center = self.__midpoint((x_max, y_max), (x_min, y_min))
        border_pts = self.__scale_border(border_pts, center, (5 * INTERVAL_SIZE))

        # shift (minx, miny) to (0, 0) for A*
        if (border_pts[3][0] < 0): # x min < 0
            self.shift_x = abs(border_pts[3][0])
        elif (border_pts[3][0] > 0): # x min > 0
            self.shift_x = -abs(border_pts[3][0])
        if (border_pts[3][1] < 0): # y min < 0
            self.shift_y = abs(border_pts[3][1])
        elif (border_pts[3][1] > 0): # y min > 0
            self.shift_y = -abs(border_pts[3][1])
        
        # shift border corners
        for i in range(len(border_pts)):
            border_pts[i][0] += self.shift_x
            border_pts[i][1] += self.shift_y

        # add interval points for border
        border_pts += self.__intermediates(border_pts[0], border_pts[1], INTERVAL_SIZE)
        border_pts += self.__intermediates(border_pts[1], border_pts[3], INTERVAL_SIZE)
        border_pts += self.__intermediates(border_pts[2], border_pts[0], INTERVAL_SIZE)
        border_pts += self.__intermediates(border_pts[3], border_pts[2], INTERVAL_SIZE)

        # modifying koz list passed by reference causes a bug for using real target case
        _kozList = [] 
        # shift KeepOut zone points
        for koz in kozList:
            tmp = []
            for pt in koz:
                tmp.append([(pt[0] + self.shift_x), (pt[1] + self.shift_y)])
            _kozList.append(tmp)

        # add interval points for koz
        koz_pts = []
        for _koz in _kozList:
            for i in range(len(_koz) -1):
                koz_pts += self.__intermediates(_koz[i], _koz[i+1], INTERVAL_SIZE)
            koz_pts += self.__intermediates(_koz[-1], _koz[0], INTERVAL_SIZE)
            koz_pts += self.__intermediates(_koz[0], _koz[1], INTERVAL_SIZE)

        # shift start and goal positions
        start_pt = [(self.position[0] + self.shift_x),
                    (self.position[1] + self.shift_y)]
        goal_pt = []
        if not use_pseudo_target:
            goal_pt = [(self.waypoint[0] + self.shift_x),
                       (self.waypoint[1] + self.shift_y)]
        else:
            goal_pt = [(pseudo_target[0] + self.shift_x),
                    (pseudo_target[1] + self.shift_y)]

        return start_pt, goal_pt, border_pts, koz_pts, use_pseudo_target

    def __convertToScaleInt(self, item, scalef):
        newitem = []
        for i in item:
            i = int(i*scalef)
            newitem.append(i)
        return newitem

    def make_border_cells(self, mypos, scalef, offset):
        border_pts = np.array([[mypos[0] - offset, mypos[1] - offset],
                               [mypos[0] + offset, mypos[1] + offset]])
        border_pts[0] = self.__convertToScaleInt(border_pts[0], scalef)
        border_pts[1] = self.__convertToScaleInt(border_pts[1], scalef)
        zero_x = border_pts[0][0]
        zero_y = border_pts[0][1]
        zero_pos = np.array([zero_x, zero_y])
        border_pts -= zero_pos

        # bottom-left to br
        border_fill = np.arange(border_pts[0][0], border_pts[1][0], 1)
        bsize = len(border_fill)  # will be square
        border_cell = [border_fill, np.ones(bsize) * border_pts[0][1]]
        # bottom-right to tr
        border_fill = np.array([np.ones(bsize) * border_pts[1][0],
                                np.arange(border_pts[0][1], border_pts[1][1], 1)])
        border_cell = np.concatenate((border_cell, border_fill), axis=1)
        # top-right to tl
        border_fill = np.array([np.arange(border_pts[1][0], border_pts[0][0], -1),
                                np.ones(bsize) * border_pts[1][1]])

        border_cell = np.concatenate((border_cell, border_fill), axis=1)
        # top-left to bl
        border_fill = np.array([np.ones(bsize) * border_pts[0][0],
                                np.arange(border_pts[1][1], border_pts[0][1], -1), ])
        border_cell = np.concatenate((border_cell, border_fill), axis=1)

        border_cell = np.transpose(border_cell)
        return border_cell, zero_pos

    def make_uavtoavoid_koz(self, kozList, scalef, zero_pos):
        newkoz = np.array([])
        firstpt = True
        for pts in kozList:
            pts = self.__convertToScaleInt(pts, scalef)
            pts -= zero_pos

            if(firstpt):
                firstpt = False

            else:
                newset = self.__intermediates(lastpt, pts, 1)
                for i in newset:
                    fo = np.array([[i[0]], [i[1]]])
                    if(len(newkoz) == 0):
                        newkoz = fo
                    else:
                        newkoz = np.concatenate((newkoz, fo), axis=1)

            lastpt = pts

        return np.transpose(newkoz)
    def format_astar_input(self, kozList, scalef):
        mypos = self.position
        mygoal = self.waypoint

        offset = 0.5 #in deg.
        #scalef = 50#10e7

        border_pts, zero_pos = self.make_border_cells(mypos, scalef, offset)
        mypos = self.__convertToScaleInt(mypos, scalef)
        mygoal = self.__convertToScaleInt(mygoal, scalef)

        mypos = np.array(mypos) - zero_pos
        mygoal = np.array(mygoal) - zero_pos

        newkoz = self.make_uavtoavoid_koz(kozList, scalef, zero_pos)


        use_pseudo_target = False
        start_pt = mypos
        goal_pt = mygoal
        koz_pts = newkoz

        # fig, ax = plt.subplots()
        # #ax.plot(t, s)
        # ax.scatter(mypos[1], mypos[0])
        # ax.scatter(mygoal[1], mygoal[0])
        # ax.scatter([pt[1] for pt in border_pts], [pt[0] for pt in border_pts])
        # ax.scatter([pt[1] for pt in newkoz], [pt[0] for pt in newkoz])
        #
        # ax.set(xlabel='Lon', ylabel='Lat',
        #        title='A* formatted map')
        # ax.grid()
        #
        # #fig.savefig("test.png")
        # plt.show()
        # plt.pause(120)
        return start_pt, goal_pt, border_pts, koz_pts, zero_pos

    def findPotentialIntersects(self, uavh_others, area_length):

        mypot_area = self.possibleFlightAreaStatic(area_length)
        mypoly = Polygon(mypot_area)

        PinP = False
        avoid_areas = []
        for ouav in uavh_others:
            thier_area = ouav.possibleFlightAreaStatic(area_length)
            thier_poly = Polygon(thier_area)
            if(thier_poly.intersects(mypoly)):
                PinP = True
                avoid_areas = thier_area
                print("Pot. Collision")
                break

        return PinP, avoid_areas

    @staticmethod
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
    UAVHeading Function: avoid
        Parameters:
                    uavh_other: UAVHeading object to avoid
        Description:
                    Returns the list of waypoints generated by
                    the A* search algorithm.
    '''
    def avoid(self, uavh_others, area_length, static_koz):
        
        if(not self.waypoint):
            xy = (self.position[0], self.position[1])
            r = 0.3
            px = xy[0] + r * np.cos(self.thetaRef)
            py = xy[1] + r * np.sin(self.thetaRef)
            self.waypoint = [px, py]

        intersects, avoid_areas = self.findPotentialIntersects(uavh_others, area_length)

        if(intersects):
            intersects = [1, 1]
        else:
            intersects = {}
        #intersects, avoid_areas = self.__findIntersects(uavh_others, static_koz)

        if len(intersects) == 0:
            if not self.lastClear:
                print(TC.OKGREEN + 'PATH CLEAR.' + TC.ENDC)
            self.lastClear = True
            return False, [self.waypoint], avoid_areas, []

        #do it again with larger KOZ
        intersects, avoid_areas = self.findPotentialIntersects(uavh_others, area_length*1.33)
        print(TC.WARNING + 'AVOID.' + TC.ENDC)
        self.lastClear = False
        use_pseudo_target = False
        # get optimal path to destination
        # format UAVHeading data for A* input
        scalefactor = 50
        start, goal, border, koz, offset = self.format_astar_input(avoid_areas, scalefactor)
        #start, goal, border, koz, use_pseudo_target = self.__format_astar_input(avoid_areas, False)
        pickle.dump({"start": start,
                     "goal": goal,
                     "border": border,
                     "koz": koz,
                     "scale": scalefactor,
                     "offset": offset}, open('astar_input.p', 'wb'))
        #bool(uavh_other.staticAreaLength))

        ox, oy = [], []
        for pt in border:
            ox.append(pt[0])
            oy.append(pt[1])

        for pt in koz:
            ox.append(pt[0])
            oy.append(pt[1])

        # SHOW_ANIMATION = True

        if SHOW_ANIMATION:  # pragma: no cover
            fig, ax = plt.subplots()
            ax.plot(ox, oy, ".k", label='Search Area Obstacles')
            ax.plot(start[0], start[1], "xg", label='UAV0 Position')
            ax.plot(goal[0], goal[1], "xr", label='UAV0 Goal')
            ax.grid(True)
            ax.axis("equal")


        try:
            INTERVAL_SIZE = 1
            path_x, path_y = a_star_planning(start[0], start[1],
                                             goal[0], goal[1],
                                             ox, oy,
                                             INTERVAL_SIZE, (2 * INTERVAL_SIZE))
        except ValueError:
            print(TC.FAIL + '\t\t**No valid path found.**' + TC.ENDC)
            return False, [], avoid_areas, []

        waypoints = self.convertPathToUniqueWaypoints(path_x, path_y)
        waypoints += offset
        waypoints /= scalefactor

        full_path = np.transpose(np.array([path_x, path_y]))
        full_path += offset
        full_path /= scalefactor

        if SHOW_ANIMATION:  # pragma: no cover
            plt.plot(path_x, path_y, "-r", label='Shortest Path')
            plt.legend()
            plt.show()

        # format A* output for waypoint list
        # path_pts = []
        # if use_pseudo_target:
        #     path_pts.append(self.waypoint)
        # for i in range(len(path_x)):
        #     pt = []
        #     pt.append(path_x[i] - self.shift_x)
        #     pt.append(path_y[i] - self.shift_y)
        #     path_pts.append(pt)

            # ignore extra waypoints that are between the previous and next
            # if (i > 0) and (i < len(path_x) - 1):
            #     last_pt = []
            #     last_pt.append(path_x[i-1] - self.shift_x)
            #     last_pt.append(path_y[i-1] - self.shift_y)
            #
            #     next_pt = []
            #     next_pt.append(path_x[i+1] - self.shift_x)
            #     next_pt.append(path_y[i+1] - self.shift_y)
            #
            #     if not (self.__isBetween(last_pt, pt, next_pt)):
            #         path_pts.append(pt)
            # else:
            #     path_pts.append(pt)

        return True, waypoints, avoid_areas, full_path