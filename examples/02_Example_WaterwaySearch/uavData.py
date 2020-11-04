
import numpy as np
from dubins_Return2Route import dubinsUAV
#from UAVHeading import UAVHeading
from enum import Enum

class UAV_TYPE(Enum):
    DUBINS  = 1
    MAVLINK = 2
    CMASI   = 3
    UNKNOWN = 0 
class UAV_AVOID(Enum):
    ACTIVE = 1
    INACTIVE = 0

# uavinfo = 
# uavinfo['waypoints'] = PruitTrack
# uavinfo['waypoint_radius'] = wptRad 
# uavinfo['current_waypoint'] = 0                                    
# uavinfo['ID'] = 1
# uavinfo['IsAvoidanceUAV'] = True
# uavtype = UAV_TYPE.DUBINS
# uavtype = UAV_TYPE.MAVLINK
# uav1 = uavData(uavinfo, uavtype, UAV_AVOID.ACTIVE)

# AvoidanceLogic.avoidCheck(mainUAV, otherUAVs)
class uavData():

    def __init__(self, uavinfo, uavType, avoidanceType ):
        # real aircraft or Dubins?
        self.data = uavinfo
        self.uavType = uavType

        if self.uavType == UAV_TYPE.DUBINS:

            # Set up Dubins UAV 
            self.uavDubins = dubinsUAV(position=self.data['waypoints'][self.data['current_waypoint']], 
                                        velocity=self.data['velocity'], 
                                        heading=self.data['heading']) #, self.data.dt=dt)
            
            self.uavDubins.setWaypoints(newwps=self.data['waypoints'], newradius = self.data['waypoint_radius'])
            self.uavDubins.currentWPIndex = self.data['current_waypoint']                                   
            
            # from Ch 5 in the Pilot's Handbook of Aernautical Knowledge 
            self.data['turnRadius'] = self.uavDubins.turnRadius
            self.data['turnRate'] = self.uavDubins.turnrate

        self.position = self.data['waypoints'][self.data['current_waypoint']]
        self.velocity = self.data['velocity']
        self.heading = self.data['heading']
        self.avoidance = avoidanceType
        # History
        self.xs = np.array([])
        self.ys = np.array([])
        self.vxs = np.array([])
        self.vys = np.array([])
        self.headings = np.array([])
        self.ts = np.array([])

        self.vx = self.velocity * np.cos(self.heading)
        self.vy = self.velocity * np.sin(self.heading)
  
  
    def getPosition(self):
        return [self.position[0], self.position[1]]

    def setWaypoints(self, newwps, newradius):
        self.data['waypoints'] = newwps
        self.data['waypoint_radius'] = newradius

    def getWaypoints(self):
        return self.data['waypoints']
    
    def getActiveWaypointIndex(self):
        return self.data['current_waypoint']
        
    def setActiveWaypointIndex(self, newwpi):
        self.data['current_waypoint'] = newwpi

    def getActiveWaypointPosition(self):
        return self.data['waypoints'][self.data['current_waypoint']]

    def update(self, message):
        if self.uavType == UAV_TYPE.DUBINS:
            self.uavDubins.simulateWPDubins()
            #format data for dubins
            #call dubins update(self.uavDubins) - impacts position, heading, active waypoint?
            #format data from dubins to this class
            #uav['dubins'].simulateWPDubins(UseCarrotChase=False, delta=0.01)
            #uav = syncAVSfromDubins(uav)
            self.position = [self.uavDubins.x, self.uavDubins.y]
            self.heading = self.uavDubins.heading
            self.data['current_waypoint'] = self.uavDubins.currentWPIndex
            self.xs = self.uavDubins.xs
            self.ys = self.uavDubins.ys

        elif self.uavType == UAV_TYPE.MAVLINK:
            x = None
            # format from message data
            # impacts position, heading, active waypoint?
        elif self.uavType == UAV_TYPE.CMASI:
            x = None
            # AIR VEHICLE STATE MESSAGE
            # format from message data
            # impacts position, heading, active waypoint?

