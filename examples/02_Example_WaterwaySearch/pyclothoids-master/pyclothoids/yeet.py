from numpy import pi
import numpy as np
import matplotlib.pyplot as plt
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

angle = 180

clothoid0 = Clothoid.G1Hermite(0, 0, pi, 1, 1, 0)
#clothoid2 = Clothoid.G1Hermite(0, 0, pi, -1, 1, pi)
# clothoid2 = Clothoid.G1Hermite(0, 0, pi, 3, 1, 0)
# clothoid3 = Clothoid.G1Hermite(0, 0, pi, 4, 1, 0)

x0, y0, t0, k0, dk, s = clothoid0.Parameters
#print('x0: ' + str(x0) + '\ty0: ' + str(y0) + '\tk0: ' + str(k0) + '\tdk: ' + str(dk) + '\ts: ' + str(s))
clothoid1 = Clothoid.StandardParams(x0, y0, t0, 0, dk, s)
#plt.plot( *clothoid1.SampleXY(500) )

# plt.plot( *clothoid0.SampleXY(500) )
#plt.plot( *clothoid1.SampleXY(500) )

print("length: " + str(round(clothoid0.length,2)) + "\tdk: " + str(round(clothoid0.dk,2)) +  "\tThetaStart: " + str(round(clothoid0.ThetaStart,2)) + 
    "\tThetaEnd: " + str(round(clothoid0.ThetaEnd,2)) + "\tXStart: " + str(round(clothoid0.XStart,2)) + "\tXEnd: " + str(round(clothoid0.XEnd,2)) + 
    "\tYStart: " + str(round(clothoid0.YStart,2)) + "\tYEnd: " + str(round(clothoid0.YEnd,2)) + "\tKappaStart: " + str(round(clothoid0.KappaStart,2)) + 
    "\tKappaEnd: " + str(round(clothoid0.KappaEnd,2)))
print('\n')

#print(clothoid0.dk, clothoid0.KappaStart, clothoid0.KappaEnd)

clothoid_list = pyclothoids.SolveG2(-120.6, 45.35, pi, np.radians(0), -120.8, 45.32, np.radians(angle), np.radians(0)) # stiches a path of multiple clothoids
plt.figure()
wpList = []
j=0
for i in clothoid_list:
        if j == 1:
                plt.plot( *i.SampleXY(500) ) # plot 500 points for each clothoid
                points = i.SampleXY(50)

                # Full Set
                x_points = points[0]
                y_points = points[1]

                #First Half
                halfX_points1 = x_points[:(int(len(x_points)/2))]   
                halfY_points1 = y_points[:(int(len(y_points)/2))]  

                half_xc,half_yc,half_r = fit_circle_2d(halfX_points1, halfY_points1)
                circle = plt.Circle((half_xc,half_yc),half_r,color='magenta',alpha=0.2)
                plt.gca().add_artist(circle)
                plt.scatter(half_xc,half_yc)
                print(half_xc,half_yc,half_r)

                #Second Half
                halfX_points2 = x_points[(int(len(x_points)/2)):]   
                halfY_points2 = y_points[(int(len(y_points)/2)):] 

                half_xc,half_yc,half_r = fit_circle_2d(halfX_points2, halfY_points2)
                circle = plt.Circle((half_xc,half_yc),half_r,color='magenta',alpha=0.2)
                plt.gca().add_artist(circle)
                plt.scatter(half_xc,half_yc)
                print(half_xc,half_yc,half_r)


                xc,yc,r = fit_circle_2d(x_points, y_points)

                circle = plt.Circle((xc,yc),r,color='magenta',alpha=0.2)
                # plt.gca().add_artist(circle)
                # plt.scatter(xc,yc)
                # print(xc,yc,r)

                plt.scatter( *i.SampleXY(10) )
                wpList.append(points)

                x_m = i.X(0.07)
                y_m = i.Y(0.07)
                t_m = i.Theta(0.07)
                print('Check: ' + str(round(x_m,2)) + ' ' + str(round(y_m,2)) + ' ' + str(round(t_m,2)) + ' ' + str(s))

                print("length: " + str(round(i.length,2)) + "\tdk rad: " + str(round(i.dk,2)) +  "\tThetaStart rad: " + str(round(i.ThetaStart,2)) + 
                "\tThetaEnd rad: " + str(round(i.ThetaEnd,2)) + "\tXStart: " + str(round(i.XStart,2)) + "\tXEnd: " + str(round(i.XEnd,2)) + 
                "\tYStart: " + str(round(i.YStart,2)) + "\tYEnd: " + str(round(i.YEnd,2)) + "\tKappaStart rad: " + str(round(i.KappaStart,2)) + 
                "\tKappaEnd rad: " + str(round(i.KappaEnd,2)))

        j+=1


plt.axis('Equal')
plt.show()
