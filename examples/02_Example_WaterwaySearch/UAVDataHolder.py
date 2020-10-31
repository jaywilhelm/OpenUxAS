'''
Jeremy Browne
Nov 2020
Script provides  position and other UAV data/info to the Return To Route simulation
UAV could be in the form of a dubins vehicle or an actual vehicle 
Dubins vehicle provides position and additional data from a vehicle simulaiton
Not using a Dubins vehicle should send and receive MAVLink or similar messages
'''

import numpy as np
from dubins_Return2Route import dubinsUAV
from UAVHeading import UAVHeading

#            uavType,      ID,            v,        thetaRef, currentWPIndex, pathWpts
# init_CAS = [[      0,       1, 0.00005/2.75, np.deg2rad(270),              3, PruitTrack]]

# class uavData():

# def __init__(self):
#     # real aircraft or Dubins?
#     self.uavType = uavType

#     self.v = velocity
#     self.turnrate = np.deg2rad(20)
#     # from Ch 5 in the Pilot's Handbook of Aernautical Knowledge 
#     self.turnRadius = ((self.v * 360/(np.rad2deg(self.turnrate)))/np.pi)/2

#     #Current state
#     self.x = position[0]
#     self.y = position[1]
#     self.vx = []
#     self.vy = []
    
#     self.heading = heading
#     self.currentWPIndex = 0
#     self.lapCounter =0 # how many laps have been completed
#     self.withinThreshold = False
    
#     self.lastWP = False # Lets system knonw that last waypoint has been reached
    
#     # History
#     self.xs = np.array([])
#     self.ys = np.array([])
#     self.vxs = np.array([])
#     self.vys = np.array([])
#     self.headings = np.array([])
#     self.ts = np.array([])

#     self.vx = v * np.cos(heading)
#     self.vy = v * np.sin(heading)

def updateStates(uavlist):
    for uav in uavlist:     
            uav['dubins'].simulateWPDubins(UseCarrotChase=False, delta=0.01)
            uav = syncAVSfromDubins(uav)

def syncAVSfromDubins(uav):
        lat = uav['dubins'].x
        lon = uav['dubins'].y
        vel = uav['dubins'].v
        heading = uav['dubins'].heading
        IsAvoidanceUAV = uav['IsAvoidanceUAV']
        uav['uavobj'] = UAVHeading(pos=[lat, lon],
                                waypt=[], speed=vel, heading=heading,
                                tPossible=np.deg2rad(45), IsAvoidanceUAV=IsAvoidanceUAV)
        return uav

def createPRACASuav( dt, wptRad, uavType, ID, v, thetaRef, currentWPIndex, pathWpts ):
        ' CAS + Return to Route UAV '
        uavDict = {}   
        uavDict['dubins'] = dubinsUAV(position=pathWpts[currentWPIndex], velocity=v,          
                                            heading=thetaRef, dt=dt)
        uavDict['dubins'].setWaypoints(newwps=pathWpts, newradius = wptRad )
        uavDict['dubins'].currentWPIndex = currentWPIndex                                    
        uavDict['ID'] = ID
        uavDict['IsAvoidanceUAV'] = True
        
        return uavDict

def createNCuav( dt, wptRad, uavType, ID, v, thetaRef, currentWPIndex, pathWpts ):
        ' Non-cooperative UAV '
        uavDict = {}
        uavDict['dubins'] = dubinsUAV(position=pathWpts[currentWPIndex], velocity=v,         
                                            heading=thetaRef, dt=dt)
        uavDict['dubins'].setWaypoints(newwps=pathWpts, newradius = wptRad )
        uavDict['dubins'].currentWPIndex = currentWPIndex
        uavDict['ID'] = ID
        uavDict['IsAvoidanceUAV'] = False

        return uavDict

def createUAVList(dt, wptRad, init_CAS, init_NC): 
        '''
        Create list of dictionary entries
        Each entry contains a dubins vehicle 
        '''
        uavlist = []    # holds dictionary items for specific UAVs

        for i in range(0, len(init_CAS)):
            uavDict_CAS = createPRACASuav(dt, wptRad, init_CAS[i][0], init_CAS[i][1], init_CAS[i][2], init_CAS[i][3], init_CAS[i][4], init_CAS[i][5])
            uavlist.append(uavDict_CAS)

        for i in range(0, len(init_NC)):
            uavDict_NC = createNCuav( dt, wptRad, init_NC[i][0], init_NC[i][1], init_NC[i][2], init_NC[i][3], init_NC[i][4], init_NC[i][5])
            uavlist.append(uavDict_NC)

        for i in range(0, len(uavlist)):
            uavlist[i] = syncAVSfromDubins(uavlist[i])

        return uavlist

    # def testMain():

    #     PruitTrack =[
    #     [39.3264051914717, -82.1101289994580], [39.3263499334731, -82.1103352244684], [39.3261989661035, -82.1104861915330],
    #     [39.3259927415369, -82.1105414491151], [39.3249489949075, -82.1105414491151],[39.3247433390045, -82.1104865192473],
    #     [39.3245921315912, -82.1103357926091], [39.3245365454954, -82.1101296557923], [39.3245914749791, -82.1099233430367],
    #     [39.3247422017190, -82.1097721357249], [39.3249489949075, -82.1097165492456], [39.3259927415369, -82.1097165492456],
    #     [39.3261989665845, -82.1097718071053], [39.3263499337508, -82.1099227743733], [39.3264051914717, -82.1101289994580]]

    #     init_NC = [[4, 0.75, np.deg2rad(0), 1, PruitTrack]]
    #     init_CAS = [[1, 0.5, np.deg2rad(270), 0, PruitTrack]]
    #     uavlist = createUAVList(1, 1, init_NC, init_CAS )

    #     print(uavlist[0]['ID'])


    # if __name__ == '__main__':
    #     testMain()