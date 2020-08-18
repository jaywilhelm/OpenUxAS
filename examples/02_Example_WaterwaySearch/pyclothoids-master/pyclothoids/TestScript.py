from numpy import pi
import numpy as np
import matplotlib.pyplot as plt
import pyclothoids
from pyclothoids import Clothoid

angle = 180

clothoid0 = Clothoid.G1Hermite(0, 0, pi, 1, 1, 0)
#clothoid2 = Clothoid.G1Hermite(0, 0, pi, -1, 1, pi)
# clothoid2 = Clothoid.G1Hermite(0, 0, pi, 3, 1, 0)
# clothoid3 = Clothoid.G1Hermite(0, 0, pi, 4, 1, 0)

x0, y0, t0, k0, dk, s = clothoid0.Parameters
#print('x0: ' + str(x0) + '\ty0: ' + str(y0) + '\tk0: ' + str(k0) + '\tdk: ' + str(dk) + '\ts: ' + str(s))
clothoid1 = Clothoid.StandardParams(x0, y0, t0, 0, dk, s)
#plt.plot( *clothoid1.SampleXY(500) )

plt.plot( *clothoid0.SampleXY(500) )
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
for i in clothoid_list:
        plt.plot( *i.SampleXY(500) ) # plot 500 points for each clothoid
        points = i.SampleXY(10)
        #plt.scatter( *i.SampleXY(10) )
        wpList.append(points)

        x_m = i.X(0.07)
        y_m = i.Y(0.07)
        t_m = i.Theta(0.07)
        print('Check: ' + str(round(x_m,2)) + ' ' + str(round(y_m,2)) + ' ' + str(round(t_m,2)) + ' ' + str(s))

        print("length: " + str(round(i.length,2)) + "\tdk rad: " + str(round(i.dk,2)) +  "\tThetaStart rad: " + str(round(i.ThetaStart,2)) + 
        "\tThetaEnd rad: " + str(round(i.ThetaEnd,2)) + "\tXStart: " + str(round(i.XStart,2)) + "\tXEnd: " + str(round(i.XEnd,2)) + 
        "\tYStart: " + str(round(i.YStart,2)) + "\tYEnd: " + str(round(i.YEnd,2)) + "\tKappaStart rad: " + str(round(i.KappaStart,2)) + 
        "\tKappaEnd rad: " + str(round(i.KappaEnd,2)))



        if abs(i.ThetaStart) > 2*np.pi:
                temp = np.degrees(i.ThetaStart)
                temp = temp/360 
                if temp > 0:
                        temp = temp - abs(int(temp))
                else:
                        temp = temp + abs(int(temp))
                i.ThetaStart = temp*360
        else:
                i.ThetaStart = np.degrees(i.ThetaStart)
        
        if abs(i.ThetaEnd) > 2*np.pi:
                temp = np.degrees(i.ThetaEnd)
                temp = temp/360 
                if temp > 0:
                        temp = temp - abs(int(temp))
                else:
                        temp = temp + abs(int(temp))
                i.ThetaEnd = temp*360
        else:
                i.ThetaEnd = np.degrees(i.ThetaEnd)

        if abs(i.dk) > 2*np.pi:
                temp = np.degrees(i.dk)
                temp = temp/360 
                if temp > 0:
                        temp = temp - abs(int(temp))
                else:
                        temp = temp + abs(int(temp))
                i.dk = temp*360
        else:
                i.dk = np.degrees(i.dk)

        if abs(i.KappaStart) > 2*np.pi:
                temp = np.degrees(i.KappaStart)
                temp = temp/360 
                if temp > 0:
                        temp = temp - abs(int(temp))
                else:
                        temp = temp + abs(int(temp))
                i.KappaStart = temp*360
        else:
                i.KappaStart = np.degrees(i.KappaStart)

        if abs(i.KappaEnd) > 2*np.pi:
                temp = np.degrees(i.KappaEnd)
                temp = temp/360 
                if temp > 0:
                        temp = temp - abs(int(temp))
                else:
                   temp = temp + abs(int(temp))
                i.KappaEnd = temp*360               
        else:
                i.KappaEnd = np.degrees(i.KappaEnd)

        print("length: " + str(round(i.length,2)) + "\tdk deg: " + str(round(i.dk,2)) +  "\tThetaStart deg: " + str(round(i.ThetaStart,2)) + 
        "\tThetaEnd deg: " + str(round(i.ThetaEnd,2)) + "\tXStart: " + str(round(i.XStart,2)) + "\tXEnd: " + str(round(i.XEnd,2)) + 
        "\tYStart: " + str(round(i.YStart,2)) + "\tYEnd: " + str(round(i.YEnd,2)) + "\tKappaStart deg: " + str(round(i.KappaStart,2)) + 
        "\tKappaEnd deg: " + str(round(i.KappaEnd,2)) + '\n')

plt.show()
