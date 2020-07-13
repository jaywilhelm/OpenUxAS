# General Imports
import struct, array, os, zmq,time, sys, json, argparse

# For Dubins Vehicle
from matplotlib import pyplot as plt
import numpy as np
from dubinsUAV import dubinsUAV

# Include LMCP Python files
lmcplocation = '../../src/LMCP/py'
sys.path.append(lmcplocation)
from lmcp import LMCPFactory
from afrl.cmasi import *

sys.path.append(lmcplocation + '/afrl')
sys.path.append(lmcplocation + '/uxas')
from afrl.cmasi import SessionStatus
from uxas.messages.uxnative import StartupComplete
from afrl.cmasi import AirVehicleState
from afrl.cmasi import Location3D
from afrl.cmasi import EntityState
from afrl.cmasi import AirVehicleConfiguration

# For Collision Avoidance
from lineSegmentAoE import *
sys.path.append('../UAVHeading-CollisionAvoidance/src')
from UAVHeading import UAVHeading

def get_from_uxas(socket, factory):
    """
    Get an LCMP object from uxas.

    Keyword arguments:
        socket -- a ZMQ socket connected to uxas
        factory -- an LMCP factory
    """

    client_id, message = socket.recv_multipart()
    address, attributes, msg = message.split(bytearray('$', 'ascii'), 2)
    msg_format, msg_type, msg_group, entityid, serviceid = attributes.split(bytearray('|', 'ascii'), 4)
    #print('\n\tmsg0\t' + str(msg_type) + ' ' + str(msg_format) + ' ' + str(msg_group) + ' ' + str(entityid)+ ' ' + str(serviceid))
    
    return factory.getObject(msg)

def send_to_uxas(obj, socket, client_id):
    """
    Send an LCMP object to AMASE.
    Keyword arguments:
        obj -- an LMCP message object
        socket -- a ZMQ socket connected to AMASE
        client_id -- the client id for the AMASE socket
    """

    attributes = bytearray(str(obj.FULL_LMCP_TYPE_NAME) + "$lmcp|" + str(obj.FULL_LMCP_TYPE_NAME) + "||0|0$",
                           'ascii')
    smsg = LMCPFactory.packMessage(obj, True)

    sentinel = "+=+=+=+=";
    sentinel += str(len(attributes) + len(smsg));
    sentinel += "#@#@#@#@";

    addressedPayload = attributes;
    addressedPayload.extend(smsg);

    # sentinelized checksum
    val = 0;
    for i in range(0, len(addressedPayload)):
        val += int(addressedPayload[i] & 0xFF);

    footer = "!%!%!%!%" + str(val) + "?^?^?^?^";

    totalmsg = bytearray(sentinel, 'ascii');
    totalmsg.extend(addressedPayload);
    totalmsg.extend(bytearray(footer, 'ascii'));

    socket.send(client_id, flags=zmq.SNDMORE, copy=False)
    socket.send(totalmsg, copy=False)
    print("\n  Sent:   " + obj.FULL_LMCP_TYPE_NAME + "\n")

def finduavbyID(uavlist,uavID):
    for uav in uavlist:
        if(uav['ID'] == uavID):
            return uav

    return None

# Connect to UxAS port and send Dubins state to AMASE using CMASI messages
context = zmq.Context()
socket = context.socket(zmq.STREAM)
socket.connect("tcp://localhost:9999") # a ZMQ socket connected to UxAS
# Store the client ID for this socket
client_id, message = socket.recv_multipart()
print("Client ID: " + str(client_id))
print("Message: " + str(message))

factory = LMCPFactory.LMCPFactory()

dt=1
# uavCAS = {'position': (45.3115, -120.9850), 'targetWP': (45.3115, -120.9000),'speed': 0.0001, 'heading': np.deg2rad(270), 'thetaPossible': 30}
# uavCAS = UAVHeading(uavCAS['position'], uavCAS['targetWP'], uavCAS['speed'], uavCAS['heading'], uavCAS['thetaPossible'])
# # uavDubins = {'position': (45.3115, -120.9850), 'velocity': 0.0001, 'heading': np.deg2rad(270)}
# uavDubins = {'position': (45.32043603, -120.99309598 ), 'targetWP': (45.0000, -120.99309598 ),'speed': 0.0001, 'heading': np.deg2rad(180), 'thetaPossible': 30}
# uavDubins = UAVHeading(uavDubins['position'], uavDubins['targetWP'], uavDubins['speed'], uavDubins['heading'], uavDubins['thetaPossible'])
   

# uav = {}
# uav['CAS'] = uavCAS
# uav['altitude'] = 100
# # uav['id'] = 1
# # uav['mavlink'] = mavutil.mavlink_connection(connstring, source_system=uavx['id'], source_component=1)

# uavx = {}
# uavx['dubins'] = uavDubins
# uavx['altitude'] = 100
# uavx['id'] = 4
# uavx['mavlink'] = mavutil.mavlink_connection(connstring, source_system=uavx['id'], source_component=1)
# UAVList.append(uavx)
wp=0
wpList = []
checkMC = 0
Astar_wpList = []
koz =[]
obstX =[]
obstY =[]
tmpObstaclePoints = []

uavlist = []


while True:
    #try:
    msg_obj = get_from_uxas(socket, factory)
    #print('\n\tmsg1\t' + str(msg_obj.FULL_LMCP_TYPE_NAME))
    #except:
    #    print('\n\tProblem: Sometimes the message does not have enough stuff...not sure why\n')

    if  (msg_obj.FULL_LMCP_TYPE_NAME == 'afrl.cmasi.AirVehicleState'):      

        uavsearch = finduavbyID(uavlist, msg_obj.get_ID())
        if(not uavsearch):
            newuav = {}
            newuav['ID'] = msg_obj.get_ID()
            newuav['uavobj'] = UAVHeading(pos=[msg_obj.get_Location().get_Latitude(),msg_obj.get_Location().get_Longitude()],
            waypt=[], speed=msg_obj.get_Airspeed(), heading=msg_obj.get_Heading(), tPossible=30)
            newuav['AVS'] = msg_obj
            uavlist.append(newuav)
            print("new uav " + str(newuav['ID']))
        else:
            #update uav...
            uavsearch['AVS'] = msg_obj

        uavsearch = finduavbyID(uavlist, 1)#IDtoWatch
        #run ACS here...
        #if needed, send waypoints

        for uav in uavlist:
            lastAVS = uav['AVS']
            plt.scatter(lastAVS.get_Location().get_Latitude(), lastAVS.get_Location().get_Longitude())
            print(str(lastAVS.get_Time()) + 
            '\tAVS ID: ' + str(lastAVS.get_ID()) + 
            '\tlat: ' + str(lastAVS.get_Location().get_Latitude()) + 
            '\tlon: ' + str(lastAVS.get_Location().get_Longitude()) +
            '\tv: ' + str(lastAVS.get_Airspeed()) + 
            '\tcog: ' + str(lastAVS.get_Heading()))



        # Astar_wpList, koz = uavtemp.avoid([], [])
        # tmpObstaclePoints = []
        # print(koz)
        # print(Astar_wpList)
        # for obst in koz:
        #     obstX = [pt[0] for pt in obst]
        #     obstY = [pt[1] for pt in obst]
        #     print(str(obstX) + "\t" +str(obstY))
        #     plotLine, = plt.plot(obstX, obstY, '--r')
        #     tmpObstaclePoints.append(plotLine)
        #     print('In obst')
      

        # update uav objects
        #if msg_obj.get_ID() == 1: #UxAS 1
        #    ID = msg_obj.get_ID()
        #    loc = msg_obj.get_Location()
        #    wpTarget = msg_obj.get_CurrentWaypoint()
            # wp1 = msg_obj.get_CurrentWaypoint().get_Latitude()

            # print('\tAVS ID:  ' + str(ID) + '\tAVS Loc:  ' + str(loc.get_Latitude()) + ' ' + str(loc.get_Longitude()))

            #uavCAS.position = (msg_obj.get_Location().get_Latitude(), msg_obj.get_Location().get_Longitude())
            #uavCAS.speed = msg_obj.get_Airspeed()
            #uavCAS.waypoint = wpList[wpTarget]
            # print('\tAVS ID:  ' + str(ID) + '\tAVS Loc:  ' + str(uavCAS.position[0]) + '\t' + str(uavCAS.position[1]))

        #elif msg_obj.get_ID() == 4: #Dubins 4
        #    ID = msg_obj.get_ID()
        #    loc = msg_obj.get_Location()
            # print('\tAVS ID:  ' + str(ID) + '\tAVS Loc:  ' + str(loc.get_Latitude()) + ' ' + str(loc.get_Longitude()))

        #    uavDubins.position = (msg_obj.get_Location().get_Latitude(), msg_obj.get_Location().get_Longitude())
        #    uavDubins.speed = msg_obj.get_Airspeed()
        #    uavDubins.thetaRef = msg_obj.get_Heading()
            # print('\t\t\t\t\t\t\t\t\t\t\tAVS ID:  ' + str(ID) + '\tAVS Loc:  ' + str(uavDubins.position[0]) + '\t' + str(uavDubins.position[1]))

        #print('\tAVS ID: 1 ' + '\tAVS Loc:  ' +  str(uavCAS.position[0]) + '\t' + 
        #        str(uavCAS.position[1]) + '')#'\tTargetWP: ' + str(uavCAS.waypoint) + 
                #('\t\t\t\t\tAVS ID: 4 ' + '\tAVS Loc:  ' + str(uavDubins.position[0]) + 
                #'\t' + str(uavDubins.position[1])+ '\t' + str(uavDubins.thetaRef)))

        #uavCASPositionPlot = plt.plot(uavCAS.position[1], uavCAS.position[0], 'oc', label='CAS UAV Position')
        #uavDubinsPositionPlot, = plt.plot(uavDubins.position[1], uavDubins.position[0], 'or', label='ncUAV UAV Position')

        

        # wpPathPlot, = plt.plot([pt[0] for pt in uav0Dubins.stepDictWPVF['lastWPList']], [pt[1] for pt in uav0Dubins.stepDictWPVF['lastWPList']], '--g', label='Avoidance UAV Waypoint Path')

    # if (checkMC == 1):
    #     Astar_wpList, koz = uavCAS.avoid([uavDubins], [])

    # if tmpObstaclePoints is not None:
    #     for tmp in tmpObstaclePoints:
    #         tmp.remove()
    # tmpObstaclePoints = []
    # for obst in koz:
    #     obstX = [pt[0] for pt in obst]
    #     obstY = [pt[1] for pt in obst]
        
    #     tmpObstaclePoints.append(plotLine)
    #     print('In obst')
    # plotLine, = plt.plot(obstX, obstY, '--r')

    # if len(Astar_wpList) > 1:
    #     print('\tUsing Astar Path: ')
    #     wpList = Astar_wpList[::-1][1:] # reverse output and remove first element
    #     print("\t\tWaypoint solution:" + str(wpList))

    #     # make Waypoints for uavCAS and send MissionCommand
    #     waypoint_obj_list = []
    #     wpt_id = 201
    #     for i in range(len(wpList)):
    #         wp_obj = factory.createObjectByName("CMASI", "Waypoint")
    #         wp_obj.set_Number(wpt_id)
    #         wp_obj.set_Speed(uavCAS.speed)
    #         wp_obj.set_Latitude(wpList[i][0])
    #         wp_obj.set_Longitude(wpList[i][1])
    #         wp_obj.set_Altitude(500.0)
    #         if i < (len(wpList) - 1):
    #             wp_obj.set_NextWaypoint(wpt_id + 1)

    #         waypoint_obj_list.append(wp_obj)
    #         wpt_id += 1
        
    #     mc21_obj = factory.createObjectByName("CMASI", "MissionCommand")
    #     mc21_obj.set_FirstWaypoint(wpt_id)
    #     mc21_obj.set_CommandID(21)
    #     mc21_obj.set_VehicleID(1)
    #     mc21_obj.WaypointList = waypoint_obj_list

    #     # time.sleep(0.1)
    #     send_to_uxas(mc21_obj, socket, client_id)

    # elif len(wpList)==0:
    #     print('\tHead for target Coordinates: ')
    #     #wpList = [uav['CAS'].waypoint]
    #     print("\t\tWaypoint solution:" + str(wpList))
   
    plt.axis('equal')
    plt.grid(True)
    #plt.ylim((45.31, 45.33))
    #plt.xlim((-120.994, -120.991 ))
    plt.pause(0.0000000001)
    # else:
    #     print('\tUse previously valid path: ')
    #     # wpList = lastWPlist  # Use the last valid wp path
    #     wpList = wpList     # Head directly for the target coordinates once the path is clear






    #time.sleep(0.1)
