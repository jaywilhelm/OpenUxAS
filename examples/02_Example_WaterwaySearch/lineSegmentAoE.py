import numpy as np
from matplotlib import pyplot as plt
#from vectorFieldMO import *

def distance(a, b):
     return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

def getAngle(p0, p1, p2):
     # shift line intersection to (0,0)
     _p0 = ((p0[0] - p1[0]), (p0[1] - p1[1]))
     _p2 = ((p2[0] - p1[0]), (p2[1] - p1[1]))

     v0_angle = np.arctan2(_p0[1], _p0[0])
     if v0_angle > 180:
          v0_angle = v0_angle - 360
     
     v2_angle = np.arctan2(_p2[1], _p2[0])
     if v2_angle > 180:
          v2_angle = v2_angle - 360

     angle = (v0_angle + v2_angle) / 2

     return angle


# https://stackoverflow.com/questions/28417604/plotting-a-line-from-a-coordinate-with-and-angle
def getAngleLine(point, angle, length):
     x, y = point

     startY = y + length * np.sin(angle)
     startX = x + length * np.cos(angle)

     endY = y + length * np.sin(np.pi + angle)
     endX = x + length * np.cos(np.pi + angle)

     return [(startX, startY), (endX, endY)]

def getBorder(wpList, uavPosition, paddingScalar):
     xlist = [pt[0] for pt in wpList]
     ylist = [pt[1] for pt in wpList]
 
     x_max = np.max(xlist)
     if uavPosition[0] > x_max:
          x_max = uavPosition[0]
     x_min = np.min(xlist)
     if uavPosition[0] < x_min:
          x_min = uavPosition[0]
     y_max = np.max(ylist)
     if uavPosition[1] > y_max:
          y_max = uavPosition[1]
     y_min = np.min(ylist)
     if uavPosition[1] < y_min:
          y_min = uavPosition[1]

     padding = paddingScalar * x_max
     if (padding < paddingScalar * np.abs(x_min)):
          padding = paddingScalar * np.abs(x_min)
     if (padding < paddingScalar * y_max):
          padding = paddingScalar * y_max
     if (padding < paddingScalar * np.abs(y_min)):
          padding = paddingScalar * np.abs(y_min)

     x_max += padding
     x_min -= padding
     y_max += padding
     y_min -= padding

     border_pts =  [(x_max, y_max), 
                    (x_max, y_min),
                    (x_min, y_min),
                    (x_min, y_max),
                    (x_max, y_max)]
     return border_pts

def lineIntersect(line1, line2):
        xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
        ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

        def det(a, b):
            return a[0] * b[1] - a[1] * b[0]

        div = det(xdiff, ydiff)
        if div == 0:
           return (np.NaN, np.NaN) 

        d = (det(*line1), det(*line2))
        x = det(d, xdiff) / div
        y = det(d, ydiff) / div

        return x, y

def isBetween(pt0, intersect, pt1):
        distAB = distance(pt0, intersect)
        distBC = distance(intersect, pt1)
        distAC = distance(pt0, pt1)

        # triangle inequality
        return np.isclose((distAB + distBC), distAC)

def getAoEPolygon(border, divLine):
     foundIntersect = False
     AoEPolygon = []

     for i in range(len(border) - 1):
          intersect = lineIntersect(divLine, [border[i], border[i + 1]])

          if np.isclose(intersect[0], border[i][0]) and np.isclose(intersect[1], border[i][1]):
               continue

          if (not np.isnan(intersect[0])) and (isBetween(border[i], intersect, border[i + 1])):
                    if foundIntersect:
                         AoEPolygon.append(border[i])
                         AoEPolygon.append(intersect)

                         break
                    else:
                         foundIntersect = True
                         AoEPolygon.append(intersect)

          elif foundIntersect:
               AoEPolygon.append(border[i])

     AoEPolygon.append(AoEPolygon[0])
     return AoEPolygon

def main():
     wpList = [(0,0), (1,1), (2,0), (3,0), (4,1), (4,0), (3,-1)]
     # wpList = [(0,0), (1,1), (2,0), (3,0), (3,1)]
     # wpList = [(3,1), (3,0), (2,0), (1,1), (0,0)]
     # wpList = [(2,0), (3,0), (3,1)]
     # wpList = [(0,0), (1,1), (2,0)]
     # wpList = [(0,0), (0,3), (2,5)]
     divLines = []
     border = getBorder(wpList, 0.1)

     wpListX = [pt[0] for pt in wpList]
     wpListY = [pt[1] for pt in wpList]

     borderX = [pt[0] for pt in border]
     borderY = [pt[1] for pt in border]

     plt.plot(wpListX, wpListY, 'k')
     plt.plot(borderX, borderY, 'c')
     plt.axis('equal')
     plt.show()

     # 2 lines at a time
     for i in range(len(wpList) - 2):
          plt.plot(wpListX, wpListY, 'k')
          plt.plot(borderX, borderY, 'c')

          print('--------------------------------')
          
          angle = getAngle(wpList[i], wpList[i + 1], wpList[i + 2])
          # avgLen = (distance(wpList[i], wpList[i + 1]) + distance(wpList[i + 1], wpList[i + 2])) / 2
          divLen = distance(border[0], border[2])

          divLine = getAngleLine(wpList[i + 1], angle, divLen)
          divLines.append(divLine)

          aoe = getAoEPolygon(border, divLine)
          print(aoe)

          print('--------------------------------')

          divX = [pt[0] for pt in divLine]
          divY = [pt[1] for pt in divLine]
          plt.plot(divX, divY, '--c')

          aoeX = [pt[0] for pt in aoe]
          aoeY = [pt[1] for pt in aoe]
          plt.plot(aoeX, aoeY, '--r')

          plt.axis('equal')
          plt.show()

def wpLineSegmentVF():
     wpList = [(0,0), (1,1), (2,0), (3,0), (4,1), (4,0), (3,-1)]
     for i in range(len(wpList) - 1):
          plt.plot([wpList[i][0], wpList[i+1][0]], [wpList[i][1], wpList[i+1][1]], 'c') # line: [i, i+1]

          VFLine = VectorField(G=1,H =1, name="Line Path")
          NormTotal = True
          Plot_VF = True

          v = (wpList[i+1][0] - wpList[i][0], wpList[i+1][1] - wpList[i][1])
          delta = np.arctan2(v[1], v[0]) + np.radians(90)
          print(np.degrees(delta))

          VFLine.VFLine(wpList[i][0], wpList[i][1], delta)
          VFList = VectorFieldList([VFLine], NormTotal)
          VFPlotMachine.TotalPlotList(VFList,Plot_VF)

          plt.axis('equal')
          plt.show()

if __name__ == '__main__':
     wpLineSegmentVF()
